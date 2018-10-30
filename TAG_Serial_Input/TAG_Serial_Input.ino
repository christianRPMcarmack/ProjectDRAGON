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
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 18
byte myNum = 255;
volatile byte targetNum;
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
byte serialAnswer;
volatile uint8_t numReceive = 1;
volatile uint8_t numTimeOut = 0;
void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("### DW1000-arduino-ranging-tag ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 1);//changed
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message
  receiver();
  //transmitPoll();
  noteActivity();
}

void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {
  // tag sends POLL and listens for POLL_ACK
  expectedMsgId = POLL_ACK;
  transmitPoll();
  noteActivity();
//  Serial.println("Reset");
  numTimeOut++;
  if (numTimeOut > 3) {
    serialInput = false;
    numTimeOut = 0;
    Serial.println("Timed Out");
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

void transmitPoll() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  //  Serial.println("Send Poll");
}

void transmitRange() {
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
  //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void handleSerialInput() {
  if (Serial.available()) {
    serialAnswer = Serial.read();
    //add decifierng of serial read
    targetNum = serialAnswer - 48;
    data[16] = myNum;
    data[17] = targetNum;
    serialInput = true;
    transmitPoll();
    Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
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
    if (sentAck) { // && data[17] == myNum && data[16] == targetNum) {
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
    if (receivedAck) { // && myNum == data[17]) { //  && targetNum == data[16]) {

      receivedAck = false;
      // get message and parse
      DW1000.getData(data, LEN_DATA);
      byte msgId = data[0];
      if (msgId != expectedMsgId) {//add a check for receieve target for every input
        // unexpected message, start over again
        //Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        //Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
        //Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitPoll();
        return;
      }
      if (msgId == POLL_ACK) {// && data[17] == myNum && data[16] == targetNum) {
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
       // Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
       // Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT) {// && data[17] == myNum && data[16] == targetNum) {
        expectedMsgId = POLL_ACK;
        float curRange;
        memcpy(&curRange, data + 1, 5);
        Serial.print("Time [ms]: "); Serial.print(millis()); Serial.print(" Range: "); Serial.println(curRange);
        //transmitPoll();
        if (numReceive == 29) {
          serialInput = false;
          handleSerialInput();
          numReceive = 0;
          return;
        }
        else {
          numReceive++;
          data[16] = myNum;
          data[17] = targetNum;
          Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
          transmitPoll();
        }
        noteActivity();
      } else { //if (msgId == RANGE_FAILED && data[17] == myNum && data[16] == targetNum) {
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
