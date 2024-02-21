#include <Servo.h>
#include <EEPROM.h>

//#define DEBUG

Servo servoL;
Servo servoR;

const int GREEN = 7;
const int YELLOW = 12;
const int RED = 13;
const int BUTTONL = 4;
const int BUTTONR = 2;
const int IRSENSOR = 2;
// digitalRead returns LOW if pressed, HIGH if not pressed
const int LDRL = A2;
const int LDRM = A1;
const int LDRR = A0;
const int IR = 3;
const long IRPULSES = 38000;

int LEFTSTOP = 86;// Looking from the side where the wheels are closest
int RIGHTSTOP = 86;

int MEDIUMSPEEDR = 28;
int MEDIUMSPEEDL = 15;
//speed is 100/12.2 = 8.1967 cm/s @ +20
//150 milliseconds per cm
int millisPerSec = 150;

// Setting up the LDR calibration variables
String leftMiddleRight[] =  {"left", "left", "middle", "middle", "right", "right"};
String brightNdark[] = {"bright", "dark", "bright", "dark", "bright", "dark"};
int getLDR[] = {LDRL, LDRL, LDRM, LDRM, LDRR, LDRR};
int meanLightLDR[6];

int sDxStore[10];
long sDStore;
int sDLDR[6];
int LEDone[] = {1, 0, 0, 1, 0, 0};
int LEDtwo[] = {0, 1, 0, 0, 1, 0};
int LEDthree[] = {0, 0, 1, 0, 0, 1};

int eeAddress = 0;

// Start of functions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveStop() {
  servoR.write(RIGHTSTOP);
  servoL.write(LEFTSTOP);
}

void motorCalibration() {
  Serial.println("You're about to calibrate the motors, press PB1 to increase Left stop and PB2 to decrease it.");
  Serial.println("Type 'Right' to move onto the right motor");
  Serial.println("Left stop is currently at: ");
  Serial.println(LEFTSTOP);
  delay(250);

  while (1) {
    driveStop();

    if (digitalRead(BUTTONL) == LOW) {
      delay(250);
      LEFTSTOP++;
      Serial.println(LEFTSTOP);
    }
    if (digitalRead(BUTTONR) == LOW) {
      delay(250);
      LEFTSTOP--;
      Serial.println(LEFTSTOP);
    }
    if (Serial.available() > 0) { // if there is something in the serial thingy
      char c = Serial.read(); // assign first char to c
      if (c == "R");
      Serial.println("Right"); // if c is R move on
      break;
    }
  }
  Serial.println("You're about to calibrate the motors, press PB1 to increase Right stop and PB2 to decrease it.");
  Serial.println("Type 'Done' to continue to offset calibration");
  Serial.println("Right stop is currently at: ");
  Serial.println(RIGHTSTOP);
  delay(250);

  while (1) {
    driveStop();

    if (digitalRead(BUTTONL) == LOW) {
      delay(250);
      RIGHTSTOP++;
      Serial.println(RIGHTSTOP);
    }
    if (digitalRead(BUTTONR) == LOW) {
      delay(250);
      RIGHTSTOP--;
      Serial.println(RIGHTSTOP);
    }
    if (Serial.available() > 0) {
      char e = Serial.read();
      if (e == "D");
      Serial.println("Done");
      break;
    }
  }
  Serial.println("In a few moments, the robot will start driving, press PB1 to increase the speed of the left wheel.");
  Serial.println("Press PB2 to decrease speed of left wheel");
  Serial.println("Type 'Finish' to finish Motor Calibration");
  delay(500);

  while (1) {
    servoR.write(RIGHTSTOP - MEDIUMSPEEDR);
    servoL.write(LEFTSTOP + MEDIUMSPEEDL);

    if (digitalRead(BUTTONL) == LOW) {
      delay(250);
      MEDIUMSPEEDL++;
      Serial.println(MEDIUMSPEEDL);
    }
    if (digitalRead(BUTTONR) == LOW) {
      delay(250);
      MEDIUMSPEEDL--;
      Serial.println(MEDIUMSPEEDL);
    }
    if (Serial.available() > 0) { // if there is something in the serial thingy
      char t = Serial.read(); // assign first char to c
      if (t == "F");
      Serial.println("Finish"); // if c is R move on

      EEPROM.put(0, LEFTSTOP);
      EEPROM.put(1, MEDIUMSPEEDL);
      EEPROM.put(2, RIGHTSTOP);
      EEPROM.put(3, MEDIUMSPEEDR);
      break;
    }
  }
}

void driveSpeed(long speedsl, long speedsr) {
  servoR.write(RIGHTSTOP - speedsr);
  servoL.write(LEFTSTOP + speedsl);
  // positive for forwards negative for backwards
}

// distance defined in cm
void moveDist(int dist) {
  if (dist < 0) {
    servoR.write(RIGHTSTOP + MEDIUMSPEEDR);
    servoL.write(LEFTSTOP - MEDIUMSPEEDL);
    delay(150 * abs(dist)); // absolute so move dist can take negatives as well
    driveStop();
  }
  else {
    servoR.write(RIGHTSTOP - MEDIUMSPEEDR);
    servoL.write(LEFTSTOP + MEDIUMSPEEDL);
    delay(150 * abs(dist));
    driveStop();
  }
}

void moveAngle(int angle) {
  // 7.40 secs to complete 360 degrees
  // 7400 milliseconds
  // 20.56 milliseconds per degree

  if (angle < 0) {
    servoR.write(RIGHTSTOP - MEDIUMSPEEDR);
    servoL.write(LEFTSTOP - MEDIUMSPEEDL);
    delay(20.56 * abs(angle));
    driveStop();
  }
  else {
    servoR.write(RIGHTSTOP + MEDIUMSPEEDR);
    servoL.write(LEFTSTOP + MEDIUMSPEEDL);
    delay(20.56 * abs(angle));
    driveStop();
  }
}

void setLEDs(int gState, int yState, int rState) {
  // 0 for off 1 for on
  digitalWrite(GREEN, gState);
  digitalWrite(YELLOW, yState);
  digitalWrite(RED, rState);
}

void waitKey(int pin) {
  while (digitalRead(pin) == LOW) {}
  delay(500);
  while (digitalRead(pin) == HIGH) {}
  delay(500);
  return;
}

void LDRcalibration() {
  long runningTotal = 0;
  int count = 10;

  for (int a = 0; a < 6; a++) {
    Serial.print("Place the ");
    Serial.print(leftMiddleRight[a]);
    Serial.print(" sensor on a ");
    Serial.print(brightNdark[a]);
    Serial.println(" surface and press the right button to start");
    waitKey(BUTTONR);
    Serial.println("Calibrating in progress...");
    setLEDs(LEDone[a], LEDtwo[a], LEDthree[a]); // Setting LED's for fancy effect y'know
    for (int i = 0; i < count; i++) { // looping 10 times for 10 values for each light, dark and standard deviation value
      runningTotal += analogRead(getLDR[a]); // getting the total of all the values
      sDxStore[i] = analogRead(getLDR[a]);
      delay(50);
    }
    meanLightLDR[a] = runningTotal / count; // mean = sum of data points / amount of them

    for (int k = 0; k < count; k++) {
      sDStore += sq(sDxStore[k] - meanLightLDR[a]); // getting the square of the sum of the mean values
    }
    sDLDR[a] = sqrt(sDStore / (count - 1)); // total standard deviation for the light and dark of each mean LDR value
    setLEDs(0, 0, 0);
    Serial.println("Calibrating complete");
    Serial.println("");
    Serial.println("The mean is: ");
    Serial.println(meanLightLDR[a]);
    Serial.println("The standard deviation is: ");
    Serial.println(sDLDR[a]);
    Serial.println("");
    runningTotal = 0;
    sDStore = 0;
  }
  delay(500);

  for (int i; i < 6; i++) {
    EEPROM.put((i * 2) + 4, meanLightLDR[i]);
    EEPROM.put((i * 2) + 16, sDLDR[i]);
  }
}

void setup() {
  Serial.begin(9600);

  //tells servos which pin to use
  servoR.attach(5);
  servoL.attach(6);

  //sets up IR, LED and LDR pins as inputs and outputs
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BUTTONL, INPUT);
  pinMode(BUTTONR, INPUT);
  pinMode(LDRL, INPUT);
  pinMode(LDRM, INPUT);
  pinMode(LDRR, INPUT);
  pinMode(IR, OUTPUT);

  ////// Code actually starts doing stuff here ////////////////////////////////////////////////////////////////////////////////////////

  // these two are to reset all motors and IR pulses at the start of the booting thingy
  driveStop();
  noTone(IR);

  Serial.println("Press PB1 button to start without calibrating, press PB2 button to Calibrate and then start");
  while (1) {
    if (digitalRead(BUTTONL) == LOW) {
      delay(500);
      for (int i = 0; i < 6; i++ ) {
        EEPROM.get((i * 2) + 4, meanLightLDR[i]);
        EEPROM.get((i * 2) + 16, sDLDR[i]);
      }
      LEFTSTOP = EEPROM.read(0);
      MEDIUMSPEEDL = EEPROM.read(1);
      RIGHTSTOP = EEPROM.read(2);
      MEDIUMSPEEDR = EEPROM.read(3);
      // reading all values from eeprom to prepare for the start of the code

      Serial.println("You have chosen to continue without calibrating, press PB1 again to start");
      break;
    }
    else if (digitalRead(BUTTONR) == LOW) {
      delay(500);
      motorCalibration();
      driveStop();
      Serial.println(" ");
      Serial.println("Press PB2 for LDR Calibration");
      Serial.println("Press PB1 to start");
      delay(100);
      break;
    }
  }
  while (1) {
    if (digitalRead(BUTTONR) == LOW) {
      LDRcalibration();
      Serial.println("Press PB1 again to start");
      break;
    }
    if (digitalRead(BUTTONL) == LOW) {
      delay(200);
      break;
    }
  }

  // changing the variable names for my own sanity whilst coding
  int meanLightLDRL = meanLightLDR[0];
  int meanDarkLDRL = meanLightLDR[1];
  int meanLightLDRM = meanLightLDR[2];
  int meanDarkLDRM = meanLightLDR[3];
  int meanLightLDRR = meanLightLDR[4];
  int meanDarkLDRR = meanLightLDR[5];

  int sDLightLDRL = sDLDR[0];
  int sDDarkLDRL = sDLDR[1];
  int sDLightLDRM = sDLDR[2];
  int sDDarkLDRM = sDLDR[3];
  int sDLightLDRR = sDLDR[4];
  int sDDarkLDRR = sDLDR[5];

#ifdef DEBUG
  // checking the EEPROM is working
  Serial.println(meanLightLDRL);
  Serial.println(meanDarkLDRL);
  Serial.println(meanLightLDRM);
  Serial.println(meanDarkLDRM);
  Serial.println(meanLightLDRR);
  Serial.println(meanDarkLDRR);
  Serial.print("LEFTSTOP: ");
  Serial.println(LEFTSTOP);
  Serial.print("MEDIUMSPEEDL: ");
  Serial.println(MEDIUMSPEEDL);
  Serial.print("RIGHTSTOP: ");
  Serial.println(RIGHTSTOP);
  Serial.print("MEDIUMSPEEDR: ");
  Serial.println(MEDIUMSPEEDR);
#endif

    moveDist(10);
    moveDist(-10);
  
    moveAngle(45);
    moveAngle(-90);
    moveAngle(45);
  
  //if you want to see the reading of LDR sensors and printing mean and standard deviation, see LDR calibration
  
    tone(IR, IRPULSES); //start sending IR pulses @ 38kHz
    //moveDist(26);
  
    int lineDetect = 0;
    int linesCrossed = 0;
  
    while (1) {
      driveSpeed(MEDIUMSPEEDL, MEDIUMSPEEDR);
  
      while (digitalRead(IRSENSOR) == LOW) {
        driveStop();
      }
      if (analogRead(LDRR) < meanDarkLDRR + sDDarkLDRR) { // mean Dark value is lowest value
        driveSpeed(20, 0);
        delay(200);
        setLEDs(1, 0, 0);
//        if (analogRead(LDRL) < meanDarkLDRL + sDDarkLDRL ) {
//          setLEDs(0, 0, 0);
//          Serial.println("Detected first strip");
//          lineDetect++;
//          //break;
//        }
      }
      if (analogRead(LDRL) < meanDarkLDRL + sDDarkLDRL) {
        driveSpeed(0, 20);
        delay(200);
        setLEDs(0, 0, 1);
//        if (analogRead(LDRR) < meanDarkLDRR + sDDarkLDRR ) {
//          setLEDs(0, 0, 0);
//          Serial.println("Detected first strip");
//          lineDetect++;
//          //break;
//        }
      }
      /*
      if (lineDetect>0){
        moveDist(26);
        if(analogRead(LDRL) < meanDarkLDRL + sDDarkLDRL && analogRead(LDRR) < meanDarkLDRR + sDDarkLDRR){
          linesCrossed++;
          delay(500);
        }
      }
      if(linesCrossed == 3){
        setLEDs(1,0,0);
      }
      if (linesCrossed == 2){
        setLEDS(0,0,1);
      }
       */
      if (digitalRead(BUTTONL) == LOW) {
        driveStop();
        noTone(IR);
        break;
      }
    }
}
void loop() {}
