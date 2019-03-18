/* L298N Motor Demonstration
   DC_Motor.ino
   https://www.youtube.com/watch?v=dyjo_ggEtVU Starts at 13:40
   Encoder Reading:
   https://github.com/jumejume1/Arduino/blob/master/ROTARY_ENCODER/ROTARY_ENCODER.ino
*/

/* 5V and GND to breadboard from Arduino, Encoder and motor driver need 5 V and GND.
  Output of A and B from encoder are to pins 2 and 3, respectively, though this just uses output B.
  EnA pin on driver to pin 10, in1 and in2 to pins 8 and 7, respectively.
  Motor driver has schematic for the motor's power cables. (heat sink away from you) On the Left (Motor A): Top should be positive, bottom negative*/

// For the Encoder
int outputA = 2;
int outputB = 3;
int aState;
int aLastState;
// This variable will increase or decrease depending on the rotation of encoder
// Variables changes within interupts are volatile
volatile int counter = 0;

// For Driving the Motor (MOTOR DRIVER!!!)
int enA = 10; // enable pins must be pins capable of pulse width modulation (indicated w/ ~)
int in1 = 8;
int in2 = 7;

//120 gives a max current to the motor of ~0.5-0.6 A, this is good for full compression
//do not go above 200 might damage motor
int MotorSpeed = 150; // Motor speed between 0 and 255
String dir;
char dump;

// SETUP
void setup()
{
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  Serial.begin(9600);                 // I want to look at the encoder values
  aLastState = digitalRead(outputB);

  pinMode(2, INPUT_PULLUP); // Detect when encoder pins rise from 0 to 1
  pinMode(3, INPUT_PULLUP);
  // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin 2 on most Arduinos.
  //////attachInterrupt(0, aiA, RISING);
  // A rising pulse from encodenren activated ai0(). AttachInterrupt 1 is DigitalPin 3 on most Arduinos.
  //attachInterrupt(1, aiB, RISING);
}

// attachInterrupt A
/*void aiA()
  { // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(outputB)) {
  counter++;
  }else{
  counter--; }
  Serial.println(counter);
  }*/

/*// attachInterrupt B
  void aiB()
  { // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(outputA) == digitalRead(outputB)) {
  counter++;
  }else{
  counter--; }
  Serial.println(counter);}*/


// Motor Driver
void drive(String dir) {
  analogWrite(enA, MotorSpeed);
  if (dir == "CW") {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == "CCW") {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}



// MAIN LOOP
void loop()
{ // Spin motor clockwise
  if (Serial.available()) {
    char command = Serial.read();
   
    while (!Serial.available()){}
    
    dump = Serial.read();
    
    if (command == 'F') {
      //         for (int i = 0;i<3;i++){
      //    Serial.println("driving");
      counter = 0;
      //CW is compressing the spring
      drive("CW");
      //for no load use 2525 counts
      //need to test still for a load
      while (counter < 2525) {//this is close but more testing needs to be done to limit drift over a full mission test with 30
        if (Serial.available()) {
          drive("OFF");
          dump = Serial.read();
          while (!Serial.available()) {
          }
          dump = Serial.read();
          delay(250);
          drive("CCW");
          while (counter >= 0) {
            aState = digitalRead(outputB);
            if (aState != aLastState) {
              counter--;
              //   delay(1);
              Serial.println(counter);
            }
            aLastState = aState;
          }
          break;
        }
        aState = digitalRead(outputB);
        if (aState != aLastState) {
          counter++;
          //delay(1);
          Serial.println(counter);
        }
        aLastState = aState;

      }
      /*if (counter > 399){
        drive("OFF");
        delay(5000);*/

      drive("OFF");
      Serial.println("DONE");
      //   delay(250);
      //     }
    }
     if (command == 'U') {
      drive("CCW");
      while (1) {
        if (Serial.available()) {
          drive("OFF");
          dump = Serial.read();
          while (!Serial.available()) {
          }
          dump = Serial.read();
          break;
        }
      }
    }
    if (command == 'C') {
      drive("CW");
      while (1) {
        if (Serial.available()) {
          drive("OFF");
          dump = Serial.read();
          while (!Serial.available()) {
          }
          dump = Serial.read();
          break;
        }
      }
    }
  }
}
