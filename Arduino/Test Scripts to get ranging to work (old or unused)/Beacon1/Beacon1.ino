/*
  Cycle through all channels 1-7 (!6) after distance calcuation and switch who's the tag and anchor
 */

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = PA3; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
// last computed range/time
DW1000Time timeComputedRange;
// data buffer
#define LEN_DATA 18 // last two bytes are [to who, from who]
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;

uint8_t maxNumTimeouts = 10; // wait this number of timeouts if switching frequencies before switching back
uint8_t numTimeouts = 0;

// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

// Special Protocol Stuff
uint8_t my_number = 2; // 1 indicates it'll start as master (or a tag)
char syncCode = 's';
char instructionCode = 'i';

uint8_t msg_sent = 0;
uint8_t my_freq = 1;
uint8_t switchedFreq = 0;

byte MY_NUM = 0;
byte OTHER_NUM = 0;
byte FREQUENCY = 1;

typedef enum State_t {
  TAG = 0,
  ANCHOR = 1
} State;

State my_state;

void setup() {
    Serial.begin(115200);
     delay(1000);
//    Serial.flush();
//    delay(0.5);
//    
//    Serial.setTimeout(250);
    //    Serial.flush();
    // possibly move this b/c it's also taken care of below...
    /* syncPC(); */
    /* getLocalinoNumber(); */
    /* reply_ack(); */
    
    initDW1000();
    becomeAnchor(); // start as anchor
}

void initDW1000() {
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  // general configuration
  if ( setDW(FREQUENCY) ) freezeError(0);
  // attach callback for (successfully) sent and received message
  char msg[128];
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1);//change this for every device maybe 1 should be rover 2,3,ect should be pods
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
}

void becomeTag() {
  resetPeriod = 250;
  my_state = TAG;
  msg_sent = 1;
  Serial.println("------------TAG-----------");
  //  delay(transition_delay);
  
  // transmit a poll msg
  expectedMsgId = POLL_ACK;
  receiver();
  transmitPoll();
  noteActivity();
  numTimeouts = 0;
}

void becomeAnchor() { // can I move the bulk of this to just the initializing state ?
  resetPeriod = 500;   // wait longer as a tag for a msg as an anchor
  my_state = ANCHOR;
  OTHER_NUM = 0;
  msg_sent = 1;
  Serial.println("----------ANCHOR----------");
  // delay(transition_delay);
  // wait for a poll msg
  expectedMsgId = POLL;
  receiver();
  noteActivity();
  // for first time ranging frequency computation
  rangingCountPeriod = millis();
}

int8_t setDW(byte freq) {
  if ( !freq || freq > 7 || freq == 6 ) // check for invalid frequency
    return -1;
  my_freq = freq; // only time we modify is when we actually change
  //  Serial.print("\rSetting freq = "); Serial.println(freq);
  DW1000.newConfiguration();
  DW1000.setDefaults(); // we'll change this to dyanimically take a 
  DW1000.setDeviceAddress(my_number);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, freq);
  DW1000.commitConfiguration();
  return 0;
}


void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void freezeError(uint8_t num) { // let's just head back to default
  Serial.print('e');
  Serial.print(num);
  OTHER_NUM = 0;
  FREQUENCY = 1;
  becomeAnchor();
}

void resetInactive() {
    // anchor listens for POLL
  if ( my_state == ANCHOR ) {

    // we didn't fully send a message
    if ( msg_sent == 0) {
      if ( expectedMsgId == RANGE ) {
  transmitPollAck();
      }
      else {
  freezeError(1); 
      }
    } else { // we'll restart
      OTHER_NUM = 0;
      receiver();
    }
  } // end ANCHOR

  /* ######################## TAG #################################### */

  else {
    // check for didn't send a message
    if ( msg_sent == 0 ) {
      if ( expectedMsgId == POLL_ACK ) {
  transmitPoll(); 
      }
      else if (expectedMsgId == RANGE_REPORT) {
  transmitRange();
      }
      else
  freezeError(3);
    }

    // didn't receive a msg
    else {
      numTimeouts++;
      if (numTimeouts > maxNumTimeouts) {
  becomeAnchor();
      } else {
      Serial.print('t');
      expectedMsgId = POLL_ACK;
      transmitPoll();
      }
    }
  }
  noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
    Serial.println("sent");
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
    Serial.println("received");
}

void transmitPoll() {
    msg_sent = 0;  
    DW1000.newTransmit();
    DW1000.setDefaults();
    
    data[0] = POLL;
//    getAddress(data + 16);
    
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    noteActivity();
}

void transmitPollAck() {
  msg_sent = 0;
  DW1000.newTransmit();
  DW1000.setDefaults();
  
  data[0] = POLL_ACK;
//  getAddress(data + 16);
  
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  noteActivity();
}

void transmitRange() {
  msg_sent = 0;  
  DW1000.newTransmit();
  DW1000.setDefaults();
  
  data[0] = RANGE;
//  getAddress(data + 16);
  
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data + 1);
  timePollAckReceived.getTimestamp(data + 6);
  timeRangeSent.getTimestamp(data + 11);
  DW1000.setData(data, LEN_DATA);
  receivedAck = false;  // in case we just received something, ignore it
  DW1000.startTransmit();
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
  noteActivity();
}

void transmitRangeReport(float curRange) {
  DW1000.newTransmit();
  DW1000.setDefaults();
  
  data[0] = RANGE_REPORT;
  //getAddress(data + 16);
  // write final ranging result
  memcpy(data + 1, &curRange, 4);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  noteActivity();
}



void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
    noteActivity();
}



/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

void computeRangeAsymmetric() {
    // asymmetric two-way ranging (more computation intense, less error prone)
    DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
    DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
    DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
    // symmetric two-way ranging (less computation intense, more error prone on clock drift)
    DW1000Time tof = ((timePollAckReceived - timePollSent) - (timePollAckSent - timePollReceived) +
                      (timeRangeReceived - timePollAckSent) - (timeRangeSent - timePollAckReceived)) * 0.25f;
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */


 void loop() {
   data[16] = 1;
   data[17] = 2;
   switch( my_state ) {
    case TAG:
      loop_tag();
      break;
    case ANCHOR:
      loop_anchor();
      break;
    }
}


void loop_tag() {
//  Serial.print('l');
  delay(0.01);
//  if( handleSerial() ) {
//    return;
//  }
//  
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (millis() - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  // continue on any success confirmation  
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    msg_sent = 1;
    if (msgId == POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
      Serial.print("Pod ");Serial.print(data[16]);Serial.println(" trying to contact rover");
      
      Serial.print('p');
      //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
    } else if (msgId == RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
      Serial.print('r');
    }
    noteActivity();
  }
  if (receivedAck) {
    //    Serial.print('R');
    Serial.print("Rover receive message from pod ");Serial.println(data[16]);
    receivedAck = false;
    // get message and parse

    DW1000.getData(data, LEN_DATA);
    byte id = data[0];
    // check address of the message to make sure it's for us
//    if (checkReceiver()) {
//      data[0] = id;
 //     return;
  //  }
    byte msgId = data[0];
    if (msgId != expectedMsgId) { // start over again
    expectedMsgId = POLL_ACK;
    transmitPoll();
    noteActivity();
      return;
    }
    if (msgId == POLL_ACK) {
      Serial.print('o');
      DW1000.getReceiveTimestamp(timePollAckReceived);
      expectedMsgId = RANGE_REPORT;
      transmitRange();      
      noteActivity();
      //      Serial.println("POLL_ACK");
    } else if (msgId == RANGE_REPORT) {
      //      Serial.print("RANGE REPORT: ");
      expectedMsgId = POLL_ACK;
      float curRange;
      memcpy(&curRange, data + 1, 4);
      Serial.print('c'); // tell localino node we completed
//      print2msi(curRange); // send the range to the msi
      becomeAnchor(); // we got the distance !!
      noteActivity();
      }
    else
      freezeError(9);
    }
  }

int8_t check_reset(char in) {
  if (in == syncCode)
    return 1;
  else if (in == 'a')
    return 2;
  else
    return 0;
}

/* inform msi computer that we've received the instruction */
void reply_ack(byte num) {
  Serial.print('a');
  Serial.print(num);
}

/* ################################################################### */

void loop_anchor() {
//  if ( handleSerial() ) {
//    return;
//  }
  //Serial.println("Anchor");
  int32_t curMillis = millis(); 
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (curMillis - lastActivity > resetPeriod) {
     // becomeAnchor();
      resetInactive();
      Serial.println("No contact resetting state");
    }
    return;
  }

  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    msg_sent = 1;
    if (msgId == POLL_ACK) {
      DW1000.getTransmitTimestamp(timePollAckSent);
    }
    else if (msgId == RANGE_REPORT) {
      OTHER_NUM = 0; // open it up to listen to anyone
    }
    else
      freezeError(8);
    noteActivity();
    //becomeTag();
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte id = data[0];
    // check address of the message to make sure it's for us
//    if (checkReceiver()) { 
 //     data[0] = id;
 //     return;
  //    }
    
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      expectedMsgId = POLL;
      OTHER_NUM = 0;
      noteActivity();
      return;
    }
    if (msgId == POLL) {
      // on POLL we (re-)start, so no protocol failure
      DW1000.getReceiveTimestamp(timePollReceived);
      expectedMsgId = RANGE;
      transmitPollAck();
      noteActivity();
      //      Serial.println("\rPOLL");
    }
    else if (msgId == RANGE) {
      DW1000.getReceiveTimestamp(timeRangeReceived);
      expectedMsgId = POLL;
      timePollSent.setTimestamp(data + 1);
      timePollAckReceived.setTimestamp(data + 6);
      timeRangeSent.setTimestamp(data + 11);
      // (re-)compute range as two-way ranging is done
      computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
      float distance = timeComputedRange.getAsMeters();
      transmitRangeReport(distance);
      Serial.print("Distance from rover to pod ");Serial.print(data[16]);Serial.print("is ");Serial.print(distance);Serial.println(" m");
      noteActivity(); // update sampling rate (each second)
      OTHER_NUM = 0;
    }
  }
}
