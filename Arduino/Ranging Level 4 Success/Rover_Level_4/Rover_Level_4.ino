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
#define DATA_REQUEST 5
#define DATA_REQUEST_LAST_SEND 6
#define DATA_COMPLETE 7
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
#define LEN_Enviro 101
byte myNum = 255;
volatile byte targetNum;
volatile byte nextHop;
byte data[LEN_DATA];
byte dataEnviro[LEN_Enviro];
byte dataCheck[LEN_Enviro];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 50;//250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
byte serialAnswer;
byte dumpChar;
volatile int numReceive = 1;
volatile uint8_t numTimeOut = 0;
volatile uint8_t serialCount = 0;
byte serialState = 0;
int numMeasure = 0;
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
  //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 1);//slower long range 110 kbps
  DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY);//, 1);//faster long range 6.8 Mbps
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

  //  Serial.println("Reset");
  numTimeOut++;
  if (data[18] == myNum) {
    if (numTimeOut > 3 + numMeasure / 10) {//change back to 3 after error testing
      serialInput = false;
      numTimeOut = 0;
      numReceive = 1;
      Serial.print("Timed out for link from "); Serial.print(targetNum); Serial.print(" to "); Serial.println(data[18]);
    }
  } else {
    if (numTimeOut > 3 + numMeasure / 10) {//added increased timeouts based on number of measurements taken
      serialInput = false;
      numTimeOut = 0;
      numReceive = 1;
      resetPeriod = 100;//250;
      expectedMsgId = POLL_ACK;
      Serial.print("Timed out for link from "); Serial.print(targetNum); Serial.print(" to "); Serial.println(data[18]);
    }
  }
    noteActivity();
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
  for (int j = 0; j<10; j++){
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = INITIALIZATION;
  uint32_t roverTime = millis();
  //  Serial.println(roverTime);
  memcpy(data + 1, &roverTime, 4);
  DW1000.setData(data, LEN_DATA);//send timestampp during initialization
  DW1000.startTransmit();
  delay(100);
  }
  Serial.println("Initializing Pods");
  // Serial.println(roverTime);
  data[0] = POLL;
}

void transmitEnviro() {//request environmental data
  //serialInput = false;
  Serial.print("Sending Envrio Ping to "); Serial.println(targetNum);
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = DATA_REQUEST;
  data[1] = targetNum;
  data[2] = myNum;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  noteActivity();
  int commTry = 0;
  int check = 0;
  boolean same = true;
  dataEnviro[0] = 0;
  dataCheck[0] = 0;
  while (1) {
    if (Serial.available()) {
      dump = Serial.read();
      while (Serial.available()) {
        dump = Serial.read();
      }
      break;
        noteActivity();
    }
    if (millis() - lastActivity > 250 ) {
      //  if (commTry > 100) {
      //    break;
      //  }
      //   commTry++;
      DW1000.newTransmit();
      DW1000.setDefaults();
      data[0] = DATA_REQUEST_LAST_SEND;
      data[1] = targetNum;
      data[2] = myNum;
      DW1000.setData(data, LEN_DATA);
      DW1000.startTransmit();
      //   Serial.println("RESEND REQUEST");
      noteActivity();
    }

    if (receivedAck) {
      if (check == 1) {//request the preivious segment of data to be resent for error checking
        DW1000.getData(dataCheck, LEN_Enviro);
        //  Serial.println(dataCheck[1]);
        if (dataCheck[1] == myNum) {//target check
          //   Serial.println("Check Check");
          check = 2;
        }
      }
      if (check == 0) {//request new segment of data
        DW1000.getData(dataEnviro, LEN_Enviro);
        //   Serial.println(dataEnviro[1]);
        if (dataEnviro[1] == myNum) {//target check
          check = 1;
        }
      }

      // Serial.println("Received Data");
      receivedAck = false;
      if (dataEnviro[0] == DATA_COMPLETE) {//trnsfer complete
        check = 10;
      }
      if (check == 1) {
        DW1000.newTransmit();
        DW1000.setDefaults();
        data[0] = DATA_REQUEST_LAST_SEND;
        data[1] = targetNum;
        data[2] = myNum;
        DW1000.setData(data, LEN_DATA);
        DW1000.startTransmit();
      }
      if (check == 2) {
        //  Serial.println("Check CHeck");
        same = true;
        for (int n = 2; n < LEN_Enviro; n++) {//check each element of the data for errors
          if (dataEnviro[n] != dataCheck[n]) {
            same = false;
          }
        }
        //     Serial.println("Check CHeck");
        if (!same) {//transfer error resend data 
          //     Serial.println("RESEND");
          //  Serial.println("Stuck");
          DW1000.newTransmit();
          DW1000.setDefaults();
          data[0] = DATA_REQUEST_LAST_SEND;
          data[1] = targetNum;
          data[2] = myNum;
          DW1000.setData(data, LEN_DATA);
          DW1000.startTransmit();

        }
        else {
          for (int i = 2; i < LEN_Enviro; i++) {
            Serial.write(dataEnviro[i]);//send data to rover
          }
          //  Serial.println("Stuck");
          DW1000.newTransmit();
          DW1000.setDefaults();
          data[0] = DATA_REQUEST;//request next line of data
          data[1] = targetNum;
          data[2] = myNum;
          DW1000.setData(data, LEN_DATA);
          DW1000.startTransmit();
        }
        check = 0;
      }
      noteActivity();
    }
    if (check == 10) {
      DW1000.newTransmit();
      DW1000.setDefaults();
      data[0] = DATA_COMPLETE;
      data[16] = myNum;
      data[17] = targetNum;
      data[18] = nextHop;
      uint32_t roverTime = millis();
      //  Serial.println(roverTime);
      memcpy(data + 1, &roverTime, 4);//send new timestamp to pod for syncronization
      DW1000.setData(data, LEN_DATA);
      DW1000.startTransmit();
      Serial.println(" ");
      Serial.println("Transmission Complete");
      break;
    }
  }
}
void receiver() {//receive transmit from nodes
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void readChar (byte initialVal) {//, byte tar1, byte tar2, int num) {
  byte tempVal = 0;
  byte currentVal = 0;
  currentVal = initialVal;

  while (1) {//read in the serial input and populate target and mesh id for ranging or enviro data transfer
    tempVal += currentVal;
    currentVal = Serial.read();//read in first target
    if (currentVal == 32) {//break at space character
      break;
    } else {
      tempVal *= 10;
    }
    currentVal -= 48;
  }
  targetNum = tempVal;
  tempVal = 0;
  currentVal = 0;
  while (1) {
    tempVal += currentVal;
    currentVal = Serial.read();//read in second target
    if (currentVal == 32) {//break at space character
      break;
    } else {
      tempVal *= 10;
    }
    currentVal -= 48;
  }
  nextHop = tempVal;
  tempVal = 0;
  currentVal = 0;
  int numVal = 0;
  while (1) {//read number of measurements requested or if enviro data is requested
    numVal += (int) currentVal;
    currentVal = Serial.read();
    if (currentVal == 32) {//remove everything if space key is pressed more for error checking and clearing buffer
      while (Serial.available()) {
        dumpChar  = Serial.read();
      }
    }
    if (currentVal == 68) {//enviro data request
      serialState = 3;
      while (Serial.available()) {
        dumpChar  = Serial.read();
      }
    }
    if (currentVal == 10 || !Serial.available()) {//break on \n
      break;
    }
    numVal *= 10;
    currentVal -= 48;
  }
  numMeasure = numVal;
}


void handleSerialInput() {//handle serial inputs
  if (Serial.available()) {
    serialEnd = true;
    serialAnswer = Serial.read();
    //add decifierng of serial read
    if (serialAnswer == 73)
    {
      data[0] = INITIALIZATION;
      while (Serial.available()) {
        dumpChar  = Serial.read();//read rest of serial input and discard extra chars

      }
      serialState = 1;
    } else {
      serialAnswer -= 48;
      readChar(serialAnswer);//, targetNum, nextHop, numMeasure);
      if (numMeasure != 0) {
        serialState = 2;
      }
    }
    if (serialState != 0) {
      if (serialState == 1) {//initialization
        data[17] = 255;
        data[18] = 255;
        transmitInitialization();
        serialInput = false;
        sentAck = false;
      }
      if (serialState == 2) {//ranging
        data[16] = myNum;
        data[17] = targetNum;
        data[18] = nextHop;
        numTimeOut = 0;
        transmitPoll();
        serialInput = true;
      }
      if (serialState == 3) {//enviro data transfer
        data[17] = 255;
        data[18] = 255;
        numTimeOut = 0;
        transmitEnviro();
        serialInput = false;
        sentAck = false;
      }
      serialState = 0;
      noteActivity();
    } else {//error case
      Serial.println("Error check input");
    }
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
        if (numReceive >= numMeasure) {//change back to 30 after error testing
          serialInput = false;
          handleSerialInput();
          numReceive = 1;
          numMeasure = 0;
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
