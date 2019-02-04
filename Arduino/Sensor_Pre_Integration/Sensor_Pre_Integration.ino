// Created by: Amanda Siirola
// Date Created: November 27, 2018
// Date Last Modified: December 5, 2018 8:20pm
// Project: DRAGON
// Both Sensors plus SD Card!

// V1

// LIBRARIES_________________________________________________________________

//For both I2C sensors (Accel and Enviro.)
#include <Wire.h>
#include <stdint.h>

//For SPI (SD card)
#include <SPI.h>
#include <SD.h>

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
  String file = "data";
  String file1;
  String file2;
// __________________________________________________________________________
void setup()
{

  //SD CARD READER___________________________________________________________
  SPI.setModule (2);  // SPI port 2
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  //INITIAL SET-UP (SERIAL)__________________________________________________
  Serial.begin(2000000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(PB12)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  int i = 1;

  file1 = String(file + i);
  file2 = String(file1 + ".txt");
  while (1) {
    if(SD.exists(file2)){
    i++;
    file1 = String(file + i);
    file2 = String(file1 + ".txt");
    }
    else{
      break;
    }
  }
  Serial.print(file2);
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

  //INITIAL SET-UP (WIRE)_____________________________________________________
  // Initialise I2C communication
  Wire.begin();
  // Initialise Serial Communication, set baud rate = 9600
  Serial.begin(2000000);
  delay(1000); // 1 second to boot-up

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

  //ACCELEROMETER SENSOR____________________________________________________
  // Initialise I2C communication as MASTER
  Wire.begin();
  // Initialise Serial Communication, set baud rate = 9600
  Serial.begin(2000000);
  delay(1000);

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



}

// __________________________________________________________________________
void loop()
{

  //ENVIRONMENTAL SENSORS____________________________________________________
  //ALTIMETER MODE
  // Start I2C transmission (FOR POLLING NOT INTURRUPT)
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

  // OUTPUTS TO SERIAL MONITOR_______________________________________________________

  // // TIME
  //  time_since_turn_on = millis();
  //  Serial.print(time_since_turn_on);
  //  Serial.print("\t");
  //
  //  //DATA
  //
  //  Serial.print(altitude,2);
  //  Serial.print("\t");
  //  Serial.print(pressure,2);
  //  Serial.print("\t");
  //  Serial.print(cTemp,2);
  //  Serial.print("\t");
  //  Serial.print(fTemp,2);
  //  Serial.print("\t");
  //
  //  Serial.print(xAccl,2);
  //  Serial.print("\t");
  //  Serial.print(yAccl,2);
  //  Serial.print("\t");
  //  Serial.println(zAccl,2);
  //  Serial.print("");
  //
  //  delay(500);

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

  } else {
    // if the file did not open, print an error:
    Serial.println("error opening test.txt");
  }

  //READ FROM SD CARD TO SERIAL MONITOR________________________________________________
  //  // re-open the file for reading:
  //  myFile = SD.open("data.txt");
  //  if (myFile) {
  //    Serial.println("data.txt:");
  //
  //    // read from the file until there's nothing else in it:
  //    while (myFile.available()) {
  //      Serial.write(myFile.read());
  //    }
  //    // close the file after everything has been read:
  //    myFile.close();
  //  } else {
  //    // if the file didn't open, print an error:
  //    Serial.println("error opening data.txt");
  //  }

  //  delay(500);
}
