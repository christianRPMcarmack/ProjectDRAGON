// Author: Ivan Yurkin
//Base Ranging Code obtained from https://github.com/thotro/arduino-dw1000
//Serial Handler and node identification was added on top of base code
//Tag code is used on the rover beacon to allow the rover to function as a mobile router to obtain ranging measurements to up
//to 10 nodes 
//Tag code waits for serial input ranging from 0-9 for pods 1-10. Index starts at 0. Then tag takes 30 measurements and reports 
//them or times out 3 times before waiting for new serial input




/*
  Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
  Decawave DW1000 library for arduino.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0
  

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  @file RangingTag.ino
  Use this to test two-way ranging functionality with two DW1000. This is
  the tag component's code which polls for range computation. Addressing and
  frame filtering is currently done in a custom way, as no MAC features are
  implemented yet.

  Complements the "RangingAnchor" example sketch.

  @todo
  - use enum instead of define
  - move strings to flash (less RAM consumption)
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
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean serialInput = false;
volatile boolean serialEnd = false;
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
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
byte serialAnswer;
byte dumpChar;
volatile int numReceive = 1;
volatile uint8_t numTimeOut = 0;
volatile uint8_t serialCount = 0;

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  delay(1000);
 // Serial.println(F("### DW1000-arduino-ranging-tag ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  //Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);
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
  // anchor starts by transmitting a POLL message
  receiver();
  //transmitPoll();
  noteActivity();
}

void noteActivity() {//updates last activity time for timeouts
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {//times out if more than 250 ms has passed since last activity
  // tag sends POLL and listens for POLL_ACK
  expectedMsgId = POLL_ACK;
  transmitPoll();
  noteActivity();
  //  Serial.println("Reset");
  numTimeOut++;
  if (numTimeOut > 3) {//change back to 3 after error testing
    serialInput = false;
    numTimeOut = 0;
    Serial.print("Timed out for link from ");Serial.print(data[17]); Serial.print(" to "); Serial.println(data[16]);
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
  timePollSent.getTimestamp(data + 1);//transmits timestamps for ranging calculations
  timePollAckReceived.getTimestamp(data + 6);
  timeRangeSent.getTimestamp(data + 11);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {//receive transmit from nodes
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void handleSerialInput() {//handle serial inputs
  while (Serial.available()) {
    serialEnd = true;
    serialAnswer = Serial.read();
    //add decifierng of serial read
    if ( serialAnswer != (10) && serialCount == 1) {//ignore enter key and reads in second character
      nextHop = serialAnswer - 48;
      data[18] = nextHop;
      serialCount++;
    }
    if ( serialAnswer != (10) && serialCount == 0) {//ignore enter key and reads in first character
      targetNum = serialAnswer - 48;
      data[16] = myNum;
      data[17] = targetNum;
      serialCount++;
    } 
    
    else {//ignore characters after the first 2
      dumpChar  = Serial.read();
    }
  }
  
    if (serialEnd && serialCount != 0) {//sets return if needed
      if (serialCount < 2) {
        data[18] = 255;
      }
      serialCount = 0;
      serialInput = true;
      transmitPoll();
     // Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.print(data[16]);Serial.print(" Next hop is ");Serial.println(data[18]);
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
        //Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        //   Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
        //Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitPoll();
        return;
      }
      if (msgId == POLL_ACK && data[17] == myNum) {
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
        //Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
        // Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT && data[17] == myNum) {
        expectedMsgId = POLL_ACK;
        float curRange;
        memcpy(&curRange, data + 1, 4);
        //if (data[18] == 255) {
        Serial.print("Time [ms]: "); Serial.print(millis()); Serial.print(" Range from "); Serial.print(data[17]); Serial.print(" to "); Serial.print(data[16]);Serial.print(" : ");Serial.println(curRange);//Serial.print(" measure count is ");Serial.println(numReceive);
        //}//transmitPoll();
        if (numReceive == 30) {//change back to 30 after error testing
          serialInput = false;
          handleSerialInput();
          numReceive = 1;
          return;
        }
        else {
          numReceive++;
          data[16] = myNum;
          data[17] = targetNum;
        //  Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
          transmitPoll();
        }
        noteActivity();
      } else if (msgId == RANGE_FAILED && data[17] == myNum) {
        expectedMsgId = POLL_ACK;
        //Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
        //Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitPoll();
        noteActivity();
      }
    } //else {
    // Serial.println("hung up here");
    //}
  } else {
    //Serial.println("Waiting for input");
    handleSerialInput();
  }
}
