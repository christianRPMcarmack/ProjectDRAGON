/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file RangingAnchor.ino
 * Use this to test two-way ranging functionality with two
 * DW1000. This is the anchor component's code which computes range after
 * exchanging some messages. Addressing and frame filtering is currently done
 * in a custom way, as no MAC features are implemented yet.
 *
 * Complements the "RangingTag" example sketch.
 *
 * @todo
 *  - weighted average of ranging results based on signal quality
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
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
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
const char NAME_POD[] ="FLAMING DRAGON";
const char POD1[] = "POD 1";
const char POD2[] = "POD 2";
const char POD3[] = "POD 3";
char TARGET_1[6];
char TARGET_2[6];
char TARGET_3[6];
int pod[2];
void setup() {
    // DEBUG monitoring=
    Serial.begin(115200);
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
//    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
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
    Serial.print("Pod Name: "); Serial.println(NAME_POD);
    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    // anchor starts in receiving mode, awaiting a ranging poll message
    receiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();

    //initaliize first 3 pods
    pod[0] = 1;
    pod[1] = 2;
    pod[2] = 3;
    Serial.print("Pods in range: ");Serial.print(pod[0]);Serial.print(", ");Serial.print(pod[1]);Serial.print(", ");Serial.println(pod[2]);
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
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

void transmitPollAck() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL_ACK;
    // delay the same amount as ranging tag
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeFailed() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_FAILED;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}
void transmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
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

//void loop() {
//    int32_t curMillis = millis();
//    if (!sentAck && !receivedAck) {
//        // check if inactive
//        if (curMillis - lastActivity > resetPeriod) {
//            resetInactive();
//        }
//        //Serial.print("No Signal Receieved");Serial.println(millis());
//        return;
//    }
//    // continue on any success confirmation
//    if (sentAck) {
//        sentAck = false;
//        byte msgId = data[0];
//        if (msgId == POLL_ACK) {
//            DW1000.getTransmitTimestamp(timePollAckSent);
//            noteActivity();
//        }
//    }
//    if (receivedAck) {
//        receivedAck = false;
//        // get message and parse
//        DW1000.getData(data, LEN_DATA);
//        byte msgId = data[0];
//        if (msgId != expectedMsgId) {
//            // unexpected message, start over again (except if already POLL)
//            protocolFailed = true;
//        }
//        if (msgId == POLL) {
//            // on POLL we (re-)start, so no protocol failure
//            protocolFailed = false;
//            DW1000.getReceiveTimestamp(timePollReceived);
//            expectedMsgId = RANGE;
//            transmitPollAck();
//            noteActivity();
//        }
//        else if (msgId == RANGE) {
//            DW1000.getReceiveTimestamp(timeRangeReceived);
//            expectedMsgId = POLL;
//            if (!protocolFailed) {
//                timePollSent.setTimestamp(data + 1);
//                timePollAckReceived.setTimestamp(data + 6);
//                timeRangeSent.setTimestamp(data + 11);
//                // (re-)compute range as two-way ranging is done
//                computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
//                transmitRangeReport(timeComputedRange.getAsMicroSeconds());
//                float distance = timeComputedRange.getAsMeters();
//                Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
//                Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
//                Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
//                //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
//                //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
//                //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
//                // update sampling rate (each second)
//                successRangingCount++;
//                if (curMillis - rangingCountPeriod > 1000) {
//                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
//                    rangingCountPeriod = curMillis;
//                    successRangingCount = 0;
//                }
//            }
//            else {
//                transmitRangeFailed();
//            }
//
//            noteActivity();
//        }
//    }
//}
void loop() {

//    switch (pod[0]) {
//    case 1:
//      case 1:
//      TARGET_1 = "POD 1";
//      break;
//    case 2:
//      TARGET_1 = "POD 2";
//      break;
//    case 3:
//      TARGET_1 = "POD 3";
//      break;
//    }
//    switch (pod[1]) {
//    case 1:
//      TARGET_2 = "POD 1";
//      break;
//    case 2:
//      TARGET_2 = "POD 2";
//      break;
//    case 3:
//      TARGET_2 = "POD 3";
//      break;
//    }
//    switch (pod[2]) {
//    case 1:
//      TARGET_3 = "POD 1";
//      break;
//    case 2:
//      TARGET_3 = "POD 2";
//      break;
//    case 3:
//      TARGET_3 = "POD 3";
//      break;
//    }
  //ADD a serial read from rover for what pods to read from
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
        if (msgId != expectedMsgId) {
            // unexpected message, start over again
            //Serial.print("Received wrong message # "); Serial.println(msgId);
            expectedMsgId = POLL_ACK;
            transmitPoll();
            return;
        }
        if (msgId == POLL_ACK) {
            DW1000.getReceiveTimestamp(timePollAckReceived);
            expectedMsgId = RANGE_REPORT;
            transmitRange();
            noteActivity();
        } else if (msgId == RANGE_REPORT) {
            expectedMsgId = POLL_ACK;
            float curRange;
            memcpy(&curRange, data + 1, 4);
            transmitPoll();
            noteActivity();
        } else if (msgId == RANGE_FAILED) {
            expectedMsgId = POLL_ACK;
            transmitPoll();
            noteActivity();
        }
    }
}
