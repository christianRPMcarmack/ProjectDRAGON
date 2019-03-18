#define enable  8
#define xDir    5   // X Axis Direction Pin   Right Reloader
#define xStep   2   // X Axis Step Pin
#define yDir    6   // Y Axis Direction Pin   Left Reloader
#define yStep   3   // Y Axis Step Pin
#define zDir    7   // Z Axis Direction Pin   Azimuth Control
#define zStep   4   // Z Axis Step Pin
#define limitSwitch 10 //whatever pin its one
int load = 0;       // Start loading sequence with side 'Y'
int stepDelay = 1000;    // Delay between each pause (1 ms)
int degTurn = 0;
char charVal = 0;
char dump = 0;
int steps = 0;
float actDegTurn = 0;
int limit = 0;
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
    if (Serial.read() == 'R') {
      if ((load % 2) == 0) {
        step(false, yDir, yStep, 201);   // Rotate y axis CW 1 turn (RIGHT Reloader?)
      }
      if ((load % 2) == 1) {
        step(true, xDir, xStep, 201);    // Rotate x axis CCW 1 turn (LEFT Reloader?)
      }
      load++;
    }
    if (Serial.read() == 'B') {
      while (Serial.available()) {
        degTurn *= 10;
        charVal = Serial.read() - 48;
        degTurn += charVal;
      }
      //making assumption that headding give is relative to rover face
      degTurn += 140;
      if (degTurn < 0 || degTurn > 280) {
        Serial.println("Error requested pointing location is outside of bounds");
      }
      else {
        //Using 1.8 deg per rotation for nema 17 change if the base motor is different
        steps = degTurn / 1.8; // this will be an int but accuracy isnt the biggest thing will spit back out what the actual deg is to rover
        actDegTurn = (double) steps * 1.8;
        limit = digitalRead(limitSwitch);
        while (limit == LOW) {
          step(false, zDir, zStep, 1);//might change to true depending on rotation direction assuming CW is positive
          limit = digitalRead(limitSwitch);
        }
        step(true, zDir, zStep, steps);
      }
    }
    else {
      while (Serial.available()) {
        dump = Serial.read();
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
}
