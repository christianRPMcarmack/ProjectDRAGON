// Author: Ivan Yurkin
//Base Ranging Code obtained from https://github.com/thotro/arduino-dw1000
//Serial Handler and node identification was added on top of base code
//Tag code is used on the rover beacon to allow the rover to function as a mobile router to obtain ranging measurements to up
//to 10 nodes 
//Tag code waits for serial input ranging from 0-9 for pods 1-10. Index starts at 0. Then tag takes 30 measurements and reports 
//them or times out 3 times before waiting for new serial input
//added functionality for mesh networking by getting pod to pod distance an reporting back to tag.
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

   @file RangingAnchor.ino
   Use this to test two-way ranging functionality with two
   DW1000. This is the anchor component's code which computes range after
   exchanging some messages. Addressing and frame filtering is currently done
   in a custom way, as no MAC features are implemented yet.

   Complements the "RangingTag" example sketch.

   @todo
    - weighted average of ranging results based on signal quality
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
#define MESH_RANGE_REPORT 4
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
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
#define LEN_DATA 19
byte myNum = 3;
volatile byte returnNum;
volatile byte targetNum;
byte data[LEN_DATA];
// watchdog and reset period

uint8_t my_freq = 1;
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

void setup() {
  // DEBUG monitoring
  Serial.begin(2000000);
  delay(1000);
  Serial.println(F("### DW1000-arduino-ranging-anchor ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 1);//changed
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  //Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  //Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
 // Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
 // Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // anchor starts in receiving mode, awaiting a ranging poll message
  receiver();
  noteActivity();
  // for first time ranging frequency computation
  rangingCountPeriod = millis();
}

void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {// reset itself if nothing received. Add later functionality for a start time from receiving a "initialization"
  //signal from the tag and have it check if 20 min has elapsed.(add in level 3 success software)
  // anchor listens for POLL
  expectedMsgId = POLL;
  receiver();
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
  //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void transmitPollAck() {//first response to sender
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL_ACK;
  data[17] = targetNum;
  data[16] = myNum;
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {//report range back to sender
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_REPORT;
  data[17] = targetNum;
  data[16] = myNum;
  // write final ranging result
  memcpy(data + 1, &curRange, 4);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeFailed() {//error case
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_FAILED;
  data[17] = targetNum;
  data[16] = myNum;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void receiver() {//receive signal
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

/*
   RANGING ALGORITHMS
   ------------------
   Either of the below functions can be used for range computation (see line "CHOSEN
   RANGING ALGORITHM" in the code).
   - Asymmetric is more computation intense but least error prone
   - Symmetric is less computation intense but more error prone to clock drifts

   The anchors and tags of this reference example use the same reply delay times, hence
   are capable of symmetric ranging (and of asymmetric ranging anyway).
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
   END RANGING ALGORITHMS
   ----------------------
*/

void loop() {
  int32_t curMillis = millis();
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
      //  Serial.println("Reset");
    }
    return;
  }
  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL) {//take timestamp
      DW1000.getTransmitTimestamp(timePollSent);
      //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
    } if (msgId == RANGE) {//take timestamp
      DW1000.getTransmitTimestamp(timeRangeSent);
      noteActivity();
    }
    if (msgId == POLL_ACK) {//take timestamp
      targetNum = data[16];
      DW1000.getTransmitTimestamp(timePollAckSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    targetNum = data[16];
    byte msgId = data[0];
    if (msgId != expectedMsgId && data[17] == myNum) {// && (data[18] == 255 || data[18] == myNum)) {
      // unexpected message, start over again (except if already POLL)
      protocolFailed = true;//if you get to this point something bad happened and expectedMsgId was not set properly
    }
    //////////mesh it

    if (data[18] != 255 && data[18] != myNum) {//runs if pod to pod distance is requested
      if (msgId == POLL && data[17] == myNum ) {
        returnNum = data[16];
        protocolFailed = false;
        DW1000.getReceiveTimestamp(timePollReceived);
        targetNum = data[18];
        data[17] = targetNum;
        data[16] = myNum;
        expectedMsgId = POLL_ACK;
        transmitPoll();
      }


      if (msgId != expectedMsgId && data[17 ] == myNum) {//error check, should not be here unless expectedMsgId was set incorrectly
        // unexpected message, start over again
        //Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        //   Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
        // Serial.print("Next hop is ");Serial.println(data[18]);
        //Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitPoll();
        return;
      }
      if (msgId == POLL_ACK && data[17] == myNum) {//respond to pod
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
        //Serial.print("Receive target is "); Serial.print(data[17]); Serial.print(" Receive return is "); Serial.println(data[16]);
        data[16] = myNum;
        data[17] = targetNum;
        //  Serial.print("Next hop is ");Serial.println(data[18]);
        // Serial.print("Target is "); Serial.print(data[17]); Serial.print(" Return is "); Serial.println(data[16]);
        transmitRange();
        noteActivity();
      }
      if (msgId == RANGE_REPORT && data[17] == myNum) {//if another pod reports distnace, send the distance back to the rover through the requested link
        expectedMsgId = POLL;
        float curRange1;
        targetNum = returnNum;
        memcpy(&curRange1, data + 1, 4);
        //delay(1);
        transmitRangeReport(curRange1);
      }
    }

    if (msgId == POLL && data[17] == myNum && (data[18] == 255 || data[18] == myNum)) {//rover requests to talk to current pod
      // on POLL we (re-)start, so no protocol failure
      // Serial.println("Poll Received");
      protocolFailed = false;
      DW1000.getReceiveTimestamp(timePollReceived);
      expectedMsgId = RANGE;
      data[17] = targetNum;
      data[16] = myNum;
      // Serial.print("Tarnget is ");Serial.print(data[17]);Serial.print(" Return is ");Serial.println(data[16]);
      transmitPollAck();
      noteActivity();
    }
    else if (msgId == RANGE && data[17] == myNum && (data[18] == 255 || data[18] == myNum)) {//rover requests to talk to current pod
      DW1000.getReceiveTimestamp(timeRangeReceived);
      expectedMsgId = POLL;
      if (!protocolFailed) {
        timePollSent.setTimestamp(data + 1);
        timePollAckReceived.setTimestamp(data + 6);
        timeRangeSent.setTimestamp(data + 11);
        // (re-)compute range as two-way ranging is done
        computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
        data[17] = targetNum;
        data[16] = myNum;
        //Serial.print("Target is ");Serial.print(data[17]);Serial.print(" Return is ");Serial.println(data[16]);
        transmitRangeReport(timeComputedRange.getAsMeters());//change .getAsMircoSeconds() to .getAsMeters()
        float distance = timeComputedRange.getAsMeters();
        // Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
        //  Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
        // update sampling rate (each second)
        // successRangingCount++;
        //if (curMillis - rangingCountPeriod > 1000) {
        //  samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
        //  rangingCountPeriod = curMillis;
        // successRangingCount = 0;
        // }
      }
      else {// if(protocolFailed && data[17] == myNum && data[16] == targetNum) {
        if ( data[17] == myNum && (data[18] == 255 || data[18] == myNum)) {
          data[17] = targetNum;
          data[16] = myNum;
          //Serial.print("Target is ");Serial.print(data[17]);Serial.print(" Return is ");Serial.println(data[16]);
          transmitRangeFailed();
        }
      }
      noteActivity();
    }
  }
}
