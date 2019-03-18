// Author: Ivan Yurkin, Amanda Sirrola
//Base Ranging Code obtained from https://github.com/thotro/arduino-dw1000
//Code development was also inspired by the COHIRNT lab to jumpstart desgin
//Serial Handler and node identification was added on top of base code
//Tag code is used on the rover beacon to allow the rover to function as a mobile router to obtain ranging measurements to up
//to 10 nodes
//Tag code waits for serial input ranging from 0-9 for pods 1-10. Index starts at 0. Then tag takes 30 measurements and reports
//them or times out 3 times before waiting for new serial input
//added functionality for mesh networking by getting pod to pod distance an reporting back to tag.

#include <SPI.h>
#include <DW1000.h>
//For both I2C sensors (Accel and Enviro.)
#include <Wire.h>
#include <stdint.h>
// power modes
#include <STM32Sleep.h>
#include <RTClock.h>
#include <SD.h>

// Pod Number
byte myNum = 7;
//


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
#define NOT_INITIALIZATION 8
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = INITIALIZATION;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
boolean enviroTransfer = false;
boolean file_true = false;
// protocol error state
boolean protocolFailed = false;
boolean initialized = false;
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
#define LEN_Enviro 102
int sdPointer = 0;
int sdPointerOld = 0;
volatile byte returnNum;
volatile byte targetNum;
byte data[LEN_DATA];
byte dataEnviro[LEN_Enviro];
// watchdog and reset period
uint32_t startTime = 0;
uint32_t endTime = 1;
uint32_t roverTime;
uint8_t my_freq = 1;
uint32_t lastActivity;
uint32_t resetPeriod = 50;
char resetCount = 0;
int onTime = 1200000;
int offTime = 300000;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
//////////////////////////////////////////////////////////////////////

// ADDRESSES ________________________________________________________________

// MPL3115A2 I2C address is 0x60(196)
#define AddrMPL 0x60

// H3LIS331DL I2C address is 0x18(24)
#define AddrH3L 0x19 //Some debate on weather its address is 18 or 19
//Using 19 because SD0 pin is connected (LBS is 1)

// ASSIGNING VARIABLES ______________________________________________________

unsigned long time_since_turn_on; //If printing to Serial Monitor

// Enviromental Sensor
unsigned int data_MPL[6];
int tHeight, temp;
long pres;
float altitude, cTemp, fTemp, pressure;

// Accelerometer
uint8_t data_H3L[6];
int16_t  xAccl_bin, yAccl_bin, zAccl_bin;
float xAccl, yAccl, zAccl;

// SD Card
File myFile;
String file_pod = "pod_";
String file_pod1;
String file = "d";
String file0;
String file1;
String file2;
////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // DEBUG monitoring
  Serial.begin(2000000);
  delay(1000);
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1);
  DW1000. setNetworkId(10);
  //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 1);//slower long range 110 kbps
  DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY);//, 1);//faster long range 6.8 Mbps
  DW1000.commitConfiguration();
  // Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);

  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // anchor starts in receiving mode, awaiting a ranging poll message
  receiver();
  noteActivity();
  // for first time ranging frequency computation
  rangingCountPeriod = millis();
  endTime = millis() + offTime; //add 5 min

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  //SD CARD READER___________________________________________________________


  SPI.setModule (2);  // SPI port 2
  //SPI.setClockDivider(SPI_CLOCK_DIV2);

  //INITIAL SET-UP (SERIAL)__________________________________________________

  //Serial.print("Initializing SD card...");

  if (!SD.begin(PB12)) {
    //  Serial.println("initialization failed!");
  }
  // Serial.println("initialization done.");
  //  Serial.println(millis());
  int i = 1;
  file_pod1 = String(file_pod + myNum);
  //file0 = String(file_pod1 + file);
  //file1 = String(file0 + i);
  file2 = String(file_pod1 + ".txt");
  if (SD.exists(file2)) {
    file_true = true;
  }
  myFile = SD.open(file2, FILE_WRITE);
  myFile.print("Time [ms]");
  myFile.print("\t");
  myFile.print("Altitude [m]");
  myFile.print("\t");
  myFile.print("Pressure [KPa]");
  myFile.print("\t");
  myFile.print("Temp [C]");
  myFile.print("\t");
  myFile.print("Temp [F]");
  myFile.print("\t");
  myFile.print("xAccl [g]");
  myFile.print("\t");
  myFile.print("yAccl [g]");
  myFile.print("\t");
  myFile.println("zAccl [g]");
  myFile.close();
  // Initialise I2C communication
  //Serial.println(millis());
  //INITIAL SET-UP (WIRE)_____________________________________________________
  // Initialise I2C communication
  Wire.begin();

  //ENVIRONMENTAL SENSOR_____________________________________________________
  // Start I2C transmission
  Wire.beginTransmission(AddrMPL);
  // Select data configuration register
  Wire.write(0x13);
  // Data ready event enabled for altitude, pressure, temperature
  Wire.write(0x07);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  //  Serial.println(millis());
  //ACCELEROMETER SENSOR____________________________________________________
  // Initialise I2C communication as MASTER

  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select control register 1
  Wire.write(0x20);
  // Enable X, Y, Z axis, power on mode, data output rate 50Hz
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select control register 4 to get range
  Wire.write(0x23);
  // Set full scale, +/- 100g, continuous update
  // 00 = +/- 100 , 10 = +/- 200, 11 = +/- 400g
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();
  //Serial.println("setup complete");
  SPI.setModule (1);  // SPI port 1
  // Serial.println(millis());
  //  Serial.println(endTime);
  //  Serial.println(millis());
}


void  takeEnviro() {
  Wire.beginTransmission(AddrMPL);
  // Select control register 1
  Wire.write(0x26);
  // Active mode, OSR = 128, altimeter mode
  Wire.write(0xB9); //0xB9 (Active mode with 128 oversample with altimeter mode) \\0xB8 (Standby mode with 128 oversample with altimeter mode))
  // Stop I2C transmission
  Wire.endTransmission();

  // Start I2C transmission
  Wire.beginTransmission(AddrMPL);
  // Select data register
  Wire.write(0x00);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 6 bytes of data
  Wire.requestFrom(AddrMPL, 6);

  // Read 6 bytes of data from address 0x00(00)
  // status, pressure msb, pressure csb, pressure lsb, temp msb, temp lsb
  if (Wire.available() == 6)
  {
    data_MPL[0] = Wire.read(); //status
    data_MPL[1] = Wire.read(); //OUT_P_MSB (20 bits)
    data_MPL[2] = Wire.read(); //OUT_P_CSB (20 bits)
    data_MPL[3] = Wire.read(); //OUT_P_LSB (20 bits)
    data_MPL[4] = Wire.read(); //OUT_T_MSB (12 bits)
    data_MPL[5] = Wire.read(); //OUT_T_LSB (12 bits)
    //Shift to the right by four?? (each register has 8)
  }

  //Convert the data to 20-bits
  tHeight = (((long)(data_MPL[1] * (long)65536) + (data_MPL[2] * 256) + (data_MPL[3] & 0xF0)) / 16);
  temp = ((data_MPL[4] * 256) + (data_MPL[5] & 0xF0)) / 16;
  altitude = tHeight / 205; //******
  cTemp = (temp / 16.0);
  fTemp = cTemp * 1.8 + 32;


  //BAROMETER MODE
  // Start I2C transmission
  Wire.beginTransmission(AddrMPL);
  // Select control register
  Wire.write(0x26);
  // Active mode, OSR = 128, barometer mode
  Wire.write(0x39);
  // Stop I2C transmission
  Wire.endTransmission();

  // Start I2C transmission
  Wire.beginTransmission(AddrMPL);
  // Select data register
  Wire.write(0x00);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 4 bytes of data
  Wire.requestFrom(AddrMPL, 4);

  // Read 4 bytes of data
  // status, pres msb1, pres msb, pres lsb
  if (Wire.available() == 4)
  {
    data_MPL[0] = Wire.read();
    data_MPL[1] = Wire.read();
    data_MPL[2] = Wire.read();
    data_MPL[3] = Wire.read();
  }

  // Convert the data to 20-bits
  pres = (((long)data_MPL[1] * (long)65536) + (data_MPL[2] * 256) + (data_MPL[3] & 0xF0)) / 16;
  pressure = (pres / 4.0) / 1000.0;

  //ACCELEROMETER SENSOR__________________________________________________________

  // OUTPUT ORDER: xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
  //                    0          1          2          3          4          5

  //X AXIS LOW (28) AND HIGH (29)
  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select data register
  Wire.write(0x28);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(AddrH3L, 1);
  // xAccl lsb
  data_H3L[0] = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select data register
  Wire.write(0x29);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(AddrH3L, 1);
  // xAccl msb
  data_H3L[1] = Wire.read();

  //Y AXIS LOW (2A) AND HIGH (2B)
  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select data register
  Wire.write(0x2A);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(AddrH3L, 1);
  // yAccl lsb
  data_H3L[2] = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select data register
  Wire.write(0x2B);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(AddrH3L, 1);
  // yAccl msb
  data_H3L[3] = Wire.read();

  //Z AXIS LOW (2C) AND HIGH (2D)
  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select data register
  Wire.write(0x2C);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(AddrH3L, 1);
  // zAccl lsb
  data_H3L[4] = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(AddrH3L);
  // Select data register
  Wire.write(0x2D);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(AddrH3L, 1);
  // zAccl msb
  data_H3L[5] = Wire.read();


  //CONVERSIONS - Concatinate the MSB with the LSB...will get 16 bit output. Multiply by the gravity value (100, 200, or 400) / 2048
  //NOTE: This sensor has a 12 bit resolution. The sensor will not see all 16 bits output from this code. So shift the vlaue calculated to the right to get rid of the LSBs.
  //The sensor will read the 12 most significant bits. Move the binary over 8 spots (creates 8 zeros in LBS positions)

  xAccl_bin = data_H3L[1] << 8 | data_H3L[0];
  xAccl = (xAccl_bin >> 4) * 100 / 2048;

  yAccl_bin = data_H3L[3] << 8 | data_H3L[2];
  yAccl = (yAccl_bin >> 4) * 100 / 2048;

  zAccl_bin = data_H3L[5] << 8 | data_H3L[4];
  zAccl = (zAccl_bin >> 4) * 100 / 2048;

  // WRITE TO SD CARD_________________________________________________________________
  // Open the file.
  // NOTE: only one file can be open at a time, so you have to close this one before opening another.

  myFile = SD.open(file2, FILE_WRITE);

  if (myFile) {

    myFile.print(millis());
    myFile.print("\t");
    myFile.print(altitude);
    myFile.print("\t");
    myFile.print(pressure);
    myFile.print("\t");
    myFile.print(cTemp);
    myFile.print("\t");
    myFile.print(fTemp);
    myFile.print("\t");
    myFile.print(xAccl);
    myFile.print("\t");
    myFile.print(yAccl);
    myFile.print("\t");
    myFile.println(zAccl);

    // close the file:
    myFile.close();
    //  Serial.println("file okay");
  } else {
    // if the file did not open, print an error:
    //  Serial.println("error opening file");
  }

}

/////////add something to read sd card
///////////////////////////////////


void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {// reset itself if nothing received. Add later functionality for a start time from receiving a "initialization"
  //signal from the tag and have it check if 20 min has elapsed.(add in level 3 success software)
  // anchor listens for POLL
  resetCount++;
  if (resetCount >= 19) {
    SPI.setModule (2);  //SPI port 2
    takeEnviro();
    Serial.println("Took Enviro");
    SPI.setModule (1);  // SPI port 1
    if (initialized == true) {
      expectedMsgId = POLL;
    } else {
      expectedMsgId = INITIALIZATION;
    }
    resetCount = 0;
  }
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

void transmitEnviro() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(dataEnviro, LEN_Enviro);
  DW1000.startTransmit();
}

void transmitPoll() {//transmits first ping to node
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
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

void sleep_mode() {
  DW1000.deepSleep();
  //low power stm32
  RTClock rt(RTCSEL_LSE);
  long int alarmDelay = 300;//seconds in sleep
  //long int alarmDelay = 20;//seconds in sleep

  // Serial.println("Goto Sleep");
  sleepAndWakeUp(STANDBY, &rt, alarmDelay);
  //  delay(10000);
  //   Serial.println("Wake up");
  //wakeup dwm1000
  DW1000.spiWakeup();
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1);
  DW1000.setNetworkId(10);
  //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 1);//slower long range 110 kbps
  DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY);//, 1);//faster long range 6.8 Mbps
  DW1000.commitConfiguration();
  endTime = millis() + offTime; //add 5 min
  // endTime = millis()+20000;//add 20 sec
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
  //  Serial.println("test");
  int32_t curMillis = millis();
  //Serial.println(curMillis);
  //  Serial.println(lastActivity);
  if (!sentAck && !receivedAck) {
    // check if inactive
    if ( initialized == true && endTime < millis()) {
      //   Serial.println("Sleep");
      sleep_mode();
    }
    if (file_true == true && endTime < millis() && initialized == false) {
      //    Serial.println("Sleep");
      sleep_mode();
    }
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
      //    Serial.println("Reset");
    }
    return;
  }
  // continue on any success confirmation
  if (sentAck) {
    //  Serial.println("Sending");
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL) {//take timestamp
      DW1000.getTransmitTimestamp(timePollSent);
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
    //   Serial.println("Receiving");
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    targetNum = data[16];
    byte msgId = data[0];
    //  Serial.println("Msg Received");
    /////////////////////adding low power stuff here remove if not working
    if (msgId == INITIALIZATION) {
      initialized = true;
      startTime = millis();
      //    endTime = 10000 + startTime;//10 sec

      memcpy(&roverTime, data + 1, 4);
      endTime = onTime + startTime;//20min
      SPI.setModule (2);  //SPI port 2
      myFile = SD.open(file2, FILE_WRITE);
      //if (myFile) {
      myFile.print("Rover time at pod intialization[ms]:");
      myFile.print("\t");
      myFile.println(roverTime);

      // close the file:
      myFile.close();
      SPI.setModule (1);  //SPI port 1
      //   }
      Serial.print("Initialized for ranging ");
      //  Serial.println(roverTime);
      expectedMsgId = POLL;
      noteActivity();
    }
    /*
      //if (initialized == true && msgId != INITIALIZATION) {
      if (msgId != expectedMsgId && data[17] == myNum) {// && (data[18] == 255 || data[18] == myNum)) {
      // unexpected message, start over again (except if already POLL)
      Serial.println("error");
      protocolFailed = true;//if you get to this point something bad happened and expectedMsgId was not set properly
      }
      //////////mesh it
    */
    //  Serial.println(initialized);
    if (initialized == false) {
      if (data[17] == myNum || data[18] == myNum) {
        DW1000.newTransmit();
        DW1000.setDefaults();
        data[17] = targetNum;
        data[16] = myNum;
        data[0] = NOT_INITIALIZATION;
        DW1000.setData(data, LEN_DATA);
        DW1000.startTransmit();
      }
    }
    if (initialized == true) {
      ////////enviro transfer
      if (msgId ==  DATA_COMPLETE && data[17] == myNum) {//re initalize the sd card file with rover timestamp
        memcpy(&roverTime, data + 1, 4);
        SPI.setModule (2);  //SPI port 2
        myFile = SD.open(file2, FILE_WRITE);
        myFile.print("Rover time at pod intialization[ms]:");
        myFile.print("\t");
        myFile.println(roverTime);

        // close the file:
        myFile.close();
        SPI.setModule (1);  //SPI port 1
        noteActivity();
      }
      if (msgId ==  DATA_REQUEST && data[1] == myNum) { //send next portion of sd card file to rover beacon
        // enviroTransfer == true) {
        //    Serial.println("Getting ready to print file");
        //  Serial.println(data[1]);
        //  Serial.println(data[2]);
        enviroTransfer = true;
        SPI.setModule (2);  //SPI port 2
        myFile = SD.open(file2);
        if (myFile) {
          dataEnviro[0] = DATA_REQUEST;
          dataEnviro[1] = data[2];
          if (sdPointer != 0) {
            myFile.seek(sdPointer);
          }
          for (int i = 2; i < LEN_Enviro; i++) {
            if ( myFile.available()) {
              dataEnviro[i] = myFile.read();
              Serial.write(dataEnviro[i]);
            }
            else {
              dataEnviro[0] = DATA_COMPLETE;
              dataEnviro[i] = 0;
              //    break;
            }
          }
          noteActivity();
        }
        if (!myFile) {
          Serial.println("error with sd");
        }
        if (dataEnviro[0] == DATA_COMPLETE) {
          // Serial.println("Transfer Complete");
          myFile.close();
          sdPointerOld = 0;
          sdPointer = 0;
          SD.remove(file2);
          myFile = SD.open(file2, FILE_WRITE);
          myFile.print("Time [ms]");
          myFile.print("\t");
          myFile.print("Altitude [m]");
          myFile.print("\t");
          myFile.print("Pressure [KPa]");
          myFile.print("\t");
          myFile.print("Temp [C]");
          myFile.print("\t");
          myFile.print("Temp [F]");
          myFile.print("\t");
          myFile.print("xAccl [g]");
          myFile.print("\t");
          myFile.print("yAccl [g]");
          myFile.print("\t");
          myFile.println("zAccl [g]");
          myFile.close();
        }
        else {
          sdPointerOld = sdPointer;
          sdPointer = myFile.position();
          myFile.close();
        }
        //    Serial.println(sdPointer);
        SPI.setModule (1);  //SPI port 2
        transmitEnviro();
        //  Serial.println("Sending Enviro");
        noteActivity();
      }


      if (msgId == DATA_REQUEST_LAST_SEND && data[1] == myNum) {
        //  Serial.println("RESENDING");
        enviroTransfer = true;
        SPI.setModule (2);  //SPI port 2
        myFile = SD.open(file2);

        dataEnviro[0] = DATA_REQUEST;
        dataEnviro[1] = data[2];
        myFile.seek(sdPointerOld);
        for (int i = 2; i < LEN_Enviro; i++) {
          if (myFile) {
            if ( myFile.available()) {
              dataEnviro[i] = myFile.read();
              //          Serial.write(dataEnviro[i]);
            }
          }
        }
        myFile.close();
        //   Serial.println(sdPointer);
        SPI.setModule (1);  //SPI port 2
        transmitEnviro();
        noteActivity();
      }

      /////////
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
          noteActivity();
        }
        //unspupress if it breaks things
        /*
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
        */
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
          noteActivity();
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
          transmitRangeReport(timeComputedRange.getAsMeters());//change .getAsMircoSeconds() to .getAsMeters()
          float distance = timeComputedRange.getAsMeters();
        }
        else {// if(protocolFailed && data[17] == myNum && data[16] == targetNum) {
          if ( data[17] == myNum && (data[18] == 255 || data[18] == myNum)) {
            data[17] = targetNum;
            data[16] = myNum;
            transmitRangeFailed();
          }
        }
        noteActivity();
      }
    }
  }
}
