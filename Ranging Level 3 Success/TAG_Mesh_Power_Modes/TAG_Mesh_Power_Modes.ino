// Author: Ivan Yurkin
//Base Ranging Code obtained from https://github.com/thotro/arduino-dw1000
//Serial Handler and node identification was added on top of base code
//Tag code is used on the rover beacon to allow the rover to function as a mobile router to obtain ranging measurements to up
//to 10 nodes
//Tag code waits for serial input ranging from 0-9 for pods 1-10. Index starts at 0. Then tag takes 30 measurements and reports
//them or times out 3 times before waiting for new serial input
//Mesh network allows Tag to receive distance between two pods by inputting two characters in the serial input that are not repeat
//numbers ie. 11 is bad but 01 is good.

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = PB0; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define INITIALIZATION 4
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean serialInput = false;
volatile boolean serialEnd = false;
volatile boolean dump = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 19//
byte myNum = 255;
volatile byte targetNum;
volatile byte nextHop;
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 100;//250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
byte serialAnswer;
byte dumpChar;
volatile int numReceive = 1;
volatile uint8_t numTimeOut = 0;
volatile uint8_t serialCount = 0;

void setup() {
  // DEBUG monitoring
  Serial.begin(2000000);//increased serial speed to attempt speeding things up. will haved to test what rover can handle
  delay(1000);
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);//not sure what this does currently
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 1);//changed
  DW1000.commitConfiguration();
  // Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  //Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  //Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  //Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  //Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  data[0] = POLL;
  receiver();
  noteActivity();
}

void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {//times out if more than 250 ms has passed since last activity or 1 second if pod to pod communication is occuring
  // tag sends POLL and listens for POLL_ACK
  expectedMsgId = POLL_ACK;
  transmitPoll();
  noteActivity();
  //  Serial.println("Reset");
  numTimeOut++;
  if (data[18] == myNum) {
    if (numTimeOut > 3) {//change back to 3 after error testing
      serialInput = false;
      numTimeOut = 0;
      numReceive = 1;
      Serial.print("Timed out for link from "); Serial.print(targetNum); Serial.print(" to "); Serial.println(data[18]);
    }
  } else {
    if (numTimeOut > 3) {//change back to 2 after error testing
      serialInput = false;
      numTimeOut = 0;
      numReceive = 1;
      resetPeriod = 100;//250;
      expectedMsgId = POLL_ACK;
      Serial.print("Timed out for link from "); Serial.print(targetNum); Serial.print(" to "); Serial.println(data[18]);
    }
  }
}

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
}

void transmitPoll() {//transmits first ping to node
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  //  Serial.println("Send Poll");
}

void transmitRange() {//transmits second ping to node
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE;
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data + 1);
  timePollAckReceived.getTimestamp(data + 6);
  timeRangeSent.getTimestamp(data + 11);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitInitialization() {//initialize pods to set timestamps
  //serialInput = false;
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = INITIALIZATION;
  uint32_t roverTime = millis();
  //  Serial.println(roverTime);
  memcpy(data + 1, &roverTime, 4);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  Serial.println("Initializing Pods");
  // Serial.println(roverTime);
  data[0] = POLL;
}
void receiver() {//receive transmit from nodes
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void handleSerialInput() {//handle serial inputs
  // Serial.println("Ready for input");
  while (Serial.available()) {
    //  Serial.println("Reading Input");
    serialEnd = true;
    serialAnswer = Serial.read();
    //add decifierng of serial read
    if (!dump) {
      if (serialAnswer == 73 && serialCount == 0)
      {
        data[0] = INITIALIZATION;
        dump = true;
        serialCount = 2;
      }
      if ( serialAnswer != (10) && serialCount == 1) {//ignore enter key and reads in second character
        nextHop = serialAnswer - 48;//convert ascii to number
        data[18] = nextHop;
        serialCount++;
        resetPeriod = 100;//500;
        expectedMsgId = RANGE_REPORT;//response should be distance from pod to pod
        dump = true;
      }
      if ( serialAnswer != (10) && serialCount == 0) {//ignore enter key and reads in first character
        targetNum = serialAnswer - 48;//convert ascii to number
        data[16] = myNum;
        data[17] = targetNum;
        serialCount++;
      }
    }
    if (dump) {
      dumpChar  = Serial.read();//read rest of serial input and discard extra chars
    }
  }
  if (serialEnd && serialCount != 0) {
    //  Serial.println("Hung up here");
    if (serialCount < 2 && data[0] != INITIALIZATION) {// if only 1 char was input
      nextHop = 255;
      data[18] = 255;
      //data[0] = POLL
      resetPeriod = 100;//250;
      expectedMsgId = POLL_ACK;
      // Serial.println("1 input");
    }
    serialCount = 0;
    serialInput = true;
    serialEnd = false;
    dump = false;
    //  Serial.println(data[0]);
    // Serial.print("Next hop is ");Serial.println(data[18]);
    if (data[0] == INITIALIZATION) {
      data[17] = 255;
      data[18] = 255;
      transmitInitialization();
      serialInput = false;
      sentAck = false;
      return;
    }
    if (data[0] != INITIALIZATION) {
      transmitPoll();
      //      Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.print(data[16]); Serial.print(" Next hop is "); Serial.println(data[18]);
    }
    noteActivity();
  }
}

void loop() {
  if (serialInput) {
    if (!sentAck && !receivedAck) {
      // check if inactive
      if (millis() - lastActivity > resetPeriod ) {
        resetInactive();
      }
      return;
    }

    // continue on any success confirmation
    if (sentAck) {
      sentAck = false;
      byte msgId = data[0];
      if (msgId == POLL) {
        DW1000.getTransmitTimestamp(timePollSent);
        //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
      } else if (msgId == RANGE) {
        DW1000.getTransmitTimestamp(timeRangeSent);
        noteActivity();
      }
    }
    if (receivedAck) {

      receivedAck = false;
      // get message and parse
      DW1000.getData(data, LEN_DATA);
      byte msgId = data[0];
      if (msgId != expectedMsgId && data[17] == myNum) {
        // unexpected message, start over again
        if (data[18] == myNum || data[18] == targetNum) {//waiting for reponse from target
          expectedMsgId = POLL_ACK;
        } else {//waiting for distance from pod to pod link
          expectedMsgId = RANGE_REPORT;
        }
        data[16] = myNum;
        data[17] = targetNum;
        transmitPoll();
        return;
      }
      if (msgId == POLL_ACK && data[17] == myNum) {
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
        data[16] = myNum;
        data[17] = targetNum;
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT && data[17] == myNum) {
        targetNum = data[16];
        expectedMsgId = POLL_ACK;
        float curRange;
        memcpy(&curRange, data + 1, 4);
        // Serial.print("Next hop is ");Serial.println(data[18]);
        //   Serial.print("Time [ms]: "); Serial.print(millis()); Serial.print(" Range from "); Serial.print(targetNum); Serial.print(" to "); Serial.print(data[18]); Serial.print(" : "); Serial.println(curRange,3); //Serial.print(" measure count is ");Serial.println(numReceive);
        Serial.print(millis()); Serial.print(" "); Serial.print(targetNum); Serial.print(" "); Serial.print(nextHop); Serial.print(" "); Serial.println(curRange, 3); //Serial.print(" measure count is ");Serial.println(numReceive);

        //  Serial.println(curRange,3);
        if (numReceive == 10) {//change back to 30 after error testing
          serialInput = false;
          handleSerialInput();
          numReceive = 1;
          return;
        }
        else {
          numReceive++;
          data[16] = myNum;
          data[17] = targetNum;
          if (data[18] != myNum && data[18] != targetNum) {
            expectedMsgId = RANGE_REPORT;
          }
          transmitPoll();
        }
        noteActivity();
      } else if (msgId == RANGE_FAILED && data[17] == myNum) {
        expectedMsgId = POLL_ACK;
        data[16] = myNum;
        data[17] = targetNum;
        data[18] = nextHop;
        transmitPoll();
        noteActivity();
      }
    }
  } else {
    handleSerialInput();
  }
}
