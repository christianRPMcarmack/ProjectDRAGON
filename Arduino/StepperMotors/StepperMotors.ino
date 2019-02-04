#define enable  8
#define xDir    5   // X Axis Direction Pin   Right Reloader
#define xStep   2   // X Axis Step Pin
#define yDir    6   // Y Axis Direction Pin   Left Reloader
#define yStep   3   // Y Axis Step Pin
#define zDir    7   // Z Axis Direction Pin   Azimuth Control
#define zStep   4   // Z Axis Step Pin
#define limitSwitch 11 //whatever pin its on
int load = 0;       // Start loading sequence with side 'Y'
int stepDelay = 1000;    // Delay between each pause (1 ms)
int degTurn = 0;
byte charVal = 0;
byte dump = 0;
int steps = 0;
float actDegTurn = 0;
int limit = 0;
int sign = 1;
int i = 0;
char buffer[] = {' ',' ',' ',' '};
//
void setup() {
  pinMode(xDir, OUTPUT); pinMode(xStep, OUTPUT);
  pinMode(yDir, OUTPUT); pinMode(yStep, OUTPUT);
  pinMode(zDir, OUTPUT); pinMode(zStep, OUTPUT);
  pinMode(limitSwitch, INPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW);
  Serial.begin(9600);
}

void serialHandler() {
  if (Serial.available()) {
    charVal = Serial.read();
  //  Serial.println(charVal);
    if (charVal == 'R') {
      if ((load % 2) == 0) {
        step(false, yDir, yStep, 200);   // Rotate y axis CW 1 turn (RIGHT Reloader?)
      }
      if ((load % 2) == 1) {
        step(true, xDir, xStep, 200);    // Rotate x axis CCW 1 turn (LEFT Reloader?)
      }
      load++;
    }
    if (charVal == 'B') {
      while(!Serial.available()){}
      degTurn = 0;
      Serial.readBytesUntil('\n',buffer,4);
      degTurn = atoi(buffer);
      //Serial.println(degTurn);
      degTurn += 140;
      Serial.println(degTurn);
      if (degTurn < 0 || degTurn > 280) {
      Serial.println("Error requested pointing location is outside of bounds");
      }
      else {
        //Using 1.8 deg per rotation for nema 17 change if the base motor is different
        //Nema 23 should be the same deg per rotatation and but will need to be tested
        steps = degTurn / 1.8; // this will be an int but accuracy isnt the biggest thing will spit back out what the actual deg is to rover
   //     Serial.println(steps);
        actDegTurn = (double) steps * 1.8-140.0;
        limit = digitalRead(limitSwitch);
        while (limit == LOW) {
          step(false, zDir, zStep, 1);//might change to true depending on rotation direction assuming CW is positive
          limit = digitalRead(limitSwitch);
        }
        delay(250);
        step(true, zDir, zStep, steps);
        Serial.print("Actual pointing location: ");Serial.print(actDegTurn);Serial.println(" from current rover heading (positive is CW looking down at the rover)");
      }
    }
  }
}


void step(boolean dir, byte dirPin, byte stepperPin, int steps)
{
  digitalWrite(dirPin, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

void loop() {
  serialHandler();
/*
 steps = degTurn / 1.8; // this will be an int but accuracy isnt the biggest thing will spit back out what the actual deg is to rover
   //     Serial.println(steps);
        actDegTurn = (double) steps * 1.8-140.0;
        limit = digitalRead(limitSwitch);
        while (limit == LOW) {
          step(true, xDir, xStep, 1);//might change to true depending on rotation direction assuming CW is positive
          limit = digitalRead(limitSwitch);
       //   Serial.println("Zeroing");
        }
        delay(1000);
        Serial.println(actDegTurn);
        step(false, xDir, xStep, steps);
 delay(5000);
 */
}
