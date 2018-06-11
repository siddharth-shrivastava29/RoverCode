/* // Definition for fast analog read
  #define FASTADC 1
  // defines for setting and clearing register bits
  #ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
  #endif
  #ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
  #endif */

#include <Servo.h>
//#include <Stepper.h>

Servo GripperServo;  // create servo object to control a servo


// Define time variable
long previousMillis = 0;
unsigned long currentMillis;
long interval = 3000;           // fluidization time
long intervalFirgelli = 10000;  // firgelli moving up time
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
int stepCount = 0;

// Define step motors pins
#define stepPinX 5
#define dirPinX  3
#define enblPinX  2

#define stepPinY  8
#define dirPinY  7
#define enblPinY  6

#define stepPinZ  12
#define dirPinZ  10
#define enblPinZ  4
//
//Stepper xStepper(stepsPerRevolution, stepPinX,dirPinX);
//Stepper yStepper(stepsPerRevolution, stepPinY,dirPinY);
//Stepper zStepper(stepsPerRevolution, stepPinZ,dirPinZ);
//

// Define limit switch's pins

#define XrightLS_plus A0   // select the input pin for the limit switch 1
#define YdownLS_min A1   // select the input pin for the limit switch 2
#define XleftLS_min A2   // select the input pin for the limit switch 3
#define YupLS_plus A3   // select the input pin for the limit switch 4
#define ZretractLS_plus A4   // select the input pin for the limit switch 5

int LS1_value = LOW;  // variable to store the value coming from the analog input
int LS2_value = LOW;  // variable to store the value coming from the analog input
int LS3_value = LOW;  // variable to store the value coming from the analog input
int LS4_value = LOW;  // variable to store the value coming from the analog input
int LS5_value = LOW;  // variable to store the value coming from the analog input

// Define Tilt Firgelli Relay Pins
#define BFD 10  // Connect Digital Pin 10 on Arduino to BFU on Relay Module Bed firgelli down
#define BFU 11  // Connect Digital Pin 11 on Arduino to BFD on Relay Module Bed firgelli up

// Blower Relay Pin
#define BR 34   // Connect Digital Pin 10 on Arduino to Blower Relay Module

// Define Electromagnet Relay Pin
#define EMR 31 // Connect Digital Pin 35 on Arduino to Electromagnet Relay Module

// Define Gripper Servo Signal Pin
#define GripServo 39 // Connect Digital Pin 39 on Arduino to Servo Signal Pin

long maxStepMove = 25000;
long stepDelay = 20;
long FirgelliDelay = 8000;
void setup() {

  /* #if FASTADC
    // set prescale to 16
    sbi(ADCSRA, ADPS2) ;
    cbi(ADCSRA, ADPS1) ;
    cbi(ADCSRA, ADPS0) ;
    #endif */
  GripperServo.attach(GripServo);  // attaches the servo on pin 39 to the servo object


  // Setup limit switches' pin modes
  pinMode(XrightLS_plus, INPUT);
  pinMode(YdownLS_min, INPUT);
  pinMode(XleftLS_min, INPUT);
  pinMode(YupLS_plus, INPUT);
  pinMode(ZretractLS_plus, INPUT);

  //Setup all the Arduino Pins for Firgelli relays
  pinMode(BFU, OUTPUT);
  pinMode(BFD, OUTPUT);
  pinMode(BR, OUTPUT);
  pinMode(EMR, OUTPUT);

  //Turn OFF any power to the Relay channels
  digitalWrite(BFU, HIGH);
  digitalWrite(BFD, HIGH);
  digitalWrite(BR,  LOW);
  digitalWrite(EMR, HIGH);

  // Define all pin modes of Stepper Motors
  pinMode (stepPinX, OUTPUT);
  pinMode (dirPinX, OUTPUT);
  pinMode (enblPinX, OUTPUT);
  digitalWrite(stepPinX, LOW);
  digitalWrite(dirPinX, LOW);
  digitalWrite(enblPinX, HIGH);

  pinMode (stepPinY, OUTPUT);
  pinMode (dirPinY, OUTPUT);
  pinMode (enblPinY, OUTPUT);
  digitalWrite(stepPinY, LOW);
  digitalWrite(dirPinY, LOW);
  digitalWrite(enblPinY, HIGH);

  pinMode (stepPinZ, OUTPUT);
  pinMode (dirPinZ, OUTPUT);
  pinMode (enblPinZ, OUTPUT);
  digitalWrite(stepPinZ, LOW);
  digitalWrite(dirPinZ, LOW);
  digitalWrite(enblPinZ, HIGH);

  Serial.begin(9600);

  //  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  //  }
  //  Serial.println("Mode: (0) Initilization (1) Experiment (2) Pick Up");
  //  Serial.println("Enter the below numbers if the Mode is 1");
  //  Serial.println("Mode; Desired Bed Angle");
  //  Serial.println("Enter the below numbers if the Mode is 2");
  //  Serial.println("Mode; Desired Bed Angle; Robot FinalX; Robot FinalY; Angle");
  //GripperServo.writeMicroseconds(1400);
  //zFullUp();
  //gotoCorner();
}

static inline int8_t sgn(float val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

//_________________________________________________________________
void turnOffBlower() {
  currentMillis = millis();
  if ((currentMillis - previousMillis) > interval) {
    // set the blower relay off
    digitalWrite(BR, HIGH);
  }
}
//_________________________________________________________________
void turnOnBlower() {
  currentMillis = millis();
  if ((currentMillis - previousMillis) > interval) {
    // set the blower relay on
    digitalWrite(BR, LOW);
  }
}
//_________________________________________________________________
void readLimitSwitches() {
  LS1_value = digitalRead(XrightLS_plus);
  LS2_value = digitalRead(YdownLS_min);
  LS3_value = digitalRead(XleftLS_min);
  LS4_value = digitalRead(YupLS_plus);
  LS5_value = digitalRead(ZretractLS_plus);
}
//_________________________________________________________________
void servoWrite(float Angle) {
  if (Angle < 0) {
    Angle = -(Angle);
  }
  if (Angle > 90) {
    GripperServo.writeMicroseconds(1400 + (Angle - 90) * 5.5);
    delay(1000);
  }
  else {
    GripperServo.writeMicroseconds(1400 - (90 - Angle) * 5.5);
    delay(1000);
  }
}
//_________________________________________________________________
void goXRight(float FinalX) {
  bool broken = false;
  for (int k = 0; k < 5; k++) {
    digitalWrite(dirPinX, HIGH); //go X - RIGHT
    for (int x = 0; x < (FinalX) * 440; x++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(stepDelay);
      LS1_value = digitalRead(XrightLS_plus);
      if (LS1_value == HIGH) {    // Hit the rightmost limit switch.
        broken = true;
        break;
      }
    }
    if (broken) {
      break;
    }
  }
}

//_________________________________________________________________
void goYUp(float FinalY) {
  for (int k = 0; k < 5; k++) {
    digitalWrite(dirPinY, HIGH); //go up
    for (int x = 0; x < (FinalY) * 440; x++) {
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(stepDelay);
    }
  }
}
//_________________________________________________________________
void goZUp(float FinalZ) { // Go z up FinalZ amount of cm
  boolean broken = false;
  for (int k = 0; k < FinalZ; k++) {
    digitalWrite(dirPinZ, HIGH); //gripper goes up
    for (int x = 0; x < (FinalZ) * 440; x++) {
      digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPinZ, LOW);
      delayMicroseconds(stepDelay);
      LS5_value = digitalRead(ZretractLS_plus);
      if (LS5_value == HIGH) {
        broken = true;
        break;
      }
    }
    if (broken) {
      break;
    }
  }
}
//_________________________________________________________________
void goZDown(float FinalZ) { // Go z down FinalZ amount of cm
  for (int k = 0; k < FinalZ; k++) {
    digitalWrite(dirPinZ, LOW); //gripper goes down
    for (int x = 0; x < (FinalZ) * 440; x++) {
      digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPinZ, LOW);
      delayMicroseconds(stepDelay);
    }
  }
}
//_________________________________________________________________
void zFullUp() {
  readLimitSwitches();
  // Move the gripper up until it touches the Limit switch 5
  digitalWrite(dirPinZ, HIGH); //gripper goes up
  while (LS5_value == LOW) {
    digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelay);
    LS5_value = digitalRead(ZretractLS_plus);
    digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelay);
  }
}
//_________________________________________________________________
void zFullDown() {
  // Move the gripper down
  for (int k = 0; k < 5; k++) {
    digitalWrite(dirPinZ, LOW); //gripper goes down 5cm
    for (int x = 0; x < (5) * 440; x++) {
      digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPinZ, LOW);
      delayMicroseconds(stepDelay);
    }
  }
}
//_________________________________________________________________
void gotoCenterX() {
  // Move the gantry to most left until the gantry touches the Limit switch 3
  digitalWrite(dirPinX, LOW); // go left
  while (LS3_value == LOW) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelay);
    LS3_value = digitalRead(XleftLS_min);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelay);
  }
}
//_________________________________________________________________
void gotoCenterY(long FinalY) {

  if (FinalY > 10) {
    for (int k = 0; k < 5; k++) {
      digitalWrite(dirPinY, LOW); //go down
      for (int x = 0; x < (FinalY - 10) * 440; x++) {
        digitalWrite(stepPinY, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPinY, LOW);
        delayMicroseconds(stepDelay);
      }
    }
  }
  if (FinalY < 10) {
    for (int k = 0; k < 5; k++) {
      digitalWrite(dirPinY, HIGH); //go up
      for (int x = 0; x < (10 - FinalY) * 440; x++) {
        digitalWrite(stepPinY, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPinY, LOW);
        delayMicroseconds(stepDelay);
      }
    }
  }
}
//_________________________________________________________________
void gotoCorner() {
  readLimitSwitches();
  // Move the gantry to most left until the gantry touches the Limit switch 3
  digitalWrite(dirPinX, LOW); // go left
  while (LS3_value == LOW) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelay);
    LS3_value = digitalRead(XleftLS_min);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelay);
  }
  // Move the gantry to most down until the gantry touches the Limit switch 2
  digitalWrite(dirPinY, LOW); //go down
  while (LS2_value == LOW) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelay);
    LS2_value = digitalRead(YdownLS_min);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelay);
  }
}
//_________________________________________________________________
boolean checkSwitchesToCont(float xSpeed, float ySpeed) {
  return true;
}
//_________________________________________________________________
void moveXYAtSpeed(float xSpeed, float ySpeed, int mainTime) {
  //int decimFactor = 4;      // Equals 2*num of steppers
  unsigned long startTime = millis();
  unsigned long currTime = startTime;
  switch (sgn(xSpeed)) {
    case -1: // X is negative/left
      digitalWrite(dirPinX, LOW);
      break;
    case 1:  // X is positive/right
      digitalWrite(dirPinX, HIGH);
      break;
  }
  switch (sgn(ySpeed)) {
    case -1: // Y is negative, down
      digitalWrite(dirPinY, LOW);
      break;
    case 1:  // Y is positive, up
      digitalWrite(dirPinY, HIGH);
      break;
  }
  //  Gantry has 2mm pitch belt, 30 tooth pulleys
  // 3/10 mm per step.
  if (xSpeed == 0 && ySpeed == 0) { // Do not move
    while ((currTime - startTime) <= mainTime) {
      currTime = millis();
    }
  }
  else if (xSpeed == 0) {
    // Only move Y (cm/s)
    int yStepDelay = abs(225 / (ySpeed));
    switch (sgn(ySpeed)) {
      case -1: // Y is negative, down
        while (((currTime - startTime) <= mainTime) && (LS2_value == LOW)) {
          digitalWrite(stepPinY, HIGH);
          delayMicroseconds(yStepDelay);
          digitalWrite(stepPinY, LOW);
          delayMicroseconds(yStepDelay);
          currTime = millis();
          readLimitSwitches();
        }
      case 1:  // Y is positive, up
        while (((currTime - startTime) <= mainTime) && (LS4_value == LOW)) {
          digitalWrite(stepPinY, HIGH);
          delayMicroseconds(yStepDelay);
          digitalWrite(stepPinY, LOW);
          delayMicroseconds(yStepDelay);
          currTime = millis();
          readLimitSwitches();
        }
    }
  }
  else if (ySpeed == 0) {
    // Only move X (cm/s)
    int xStepDelay = abs(225 / (xSpeed));
    switch (sgn(xSpeed)) {
      case -1: // X is negative/left
        while (((currTime - startTime) <= mainTime) && (LS3_value == LOW)) {
          digitalWrite(stepPinX, HIGH);
          delayMicroseconds(xStepDelay);
          digitalWrite(stepPinX, LOW);
          delayMicroseconds(xStepDelay);
          currTime = millis();
          readLimitSwitches();
        }
      case 1:   // X is positive/right
        while (((currTime - startTime) <= mainTime) && (LS1_value == LOW)) {
          digitalWrite(stepPinX, HIGH);
          delayMicroseconds(xStepDelay);
          digitalWrite(stepPinX, LOW);
          delayMicroseconds(xStepDelay);
          currTime = millis();
          readLimitSwitches();
        }
    }
  } else {
    int xStepDelay = abs(225 / (2 * xSpeed));
    int yStepDelay = abs(225 / (2 * ySpeed ));
    boolean cont = true;
    while (((currTime - startTime) <= mainTime) && cont == true) {
      // Add limit switch checks here
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(xStepDelay);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(xStepDelay);
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(yStepDelay);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(yStepDelay);
      currTime = millis();
      readLimitSwitches();
      cont = checkSwitchesToCont(xSpeed,ySpeed);
//      switch (sgn(xSpeed)) {
//        case 1: switch (sgn(ySpeed)) {
//            case 1: if (LS4_value == HIGH) {
//                cont = false;
//                break;
//              }
//            case -1: if (LS2_value == HIGH) {
//                cont = false;
//                break;
//              }
//              if (LS1_value == HIGH) {
//                cont = false;
//              }
//              break;
//          }
//        case -1: switch (sgn(ySpeed)) {
//            case 1: if (LS4_value == HIGH) {
//                cont = false;
//                break;
//              }
//            case -1: if (LS2_value == HIGH) {
//                cont = false;
//                break;
//              }
//              if (LS3_value == HIGH) {
//                cont = false;
//              }
//              break;
//          }
//      }
    }
  }
}
//_________________________________________________________________
void blowerOnOff() {
  // Blower ON/OFF
  digitalWrite(BR, HIGH);
  delay(4000);
  digitalWrite(BR, LOW);
}
//_________________________________________________________________
void electromagnetOn() {
  digitalWrite(EMR, LOW); // Electromagnet ON
}
//_________________________________________________________________
void electromagnetOff() {
  digitalWrite(EMR, HIGH); // Electromagnet Off
}
//_________________________________________________________________
void loop() {
  if (Serial.available() > 0)
  {
    // RUNNING MODE enter the numbers from serial port --- Mode; rFinalX; rFinalY; rFinalZ; xSpeed; ySpeed; mainTime
    // Example input:  1;10;5;100;200
    int mode    = Serial.readStringUntil(';').toInt();          // Gantry state mode (controlled by Labview VISA protocol)
    float rFinalX = Serial.readStringUntil(';').toFloat();      // Gantry start point in x (cm)
    float rFinalY = Serial.readStringUntil(';').toFloat();      // Gantry start point in y (cm)
    float rFinalZ  = Serial.readStringUntil(';').toFloat();     // Gantry set down in z (cm)
    float xSpeed = Serial.readStringUntil(';').toFloat();       // Gantry speed during main loop in x (cm/s)
    float ySpeed = Serial.readStringUntil(';').toFloat();       // Gantry speed during main loop in y (cm/s)
    int mainTime = Serial.readStringUntil('\n').toInt();        // Time from start until end of main loop (ms)
    // Remember, this board only controls the gantry and blower.

    switch (mode) {
      case 1: // Init/reset case
        //_________________________________________________________________
        // Initilization of the setup Bed angle is set to zero (From Arduino uno), gripper firgelli is up positon, gantry is at the left bottom corner.
        // Move the gantry to initial position
//        readLimitSwitches();
        blowerOnOff() ;  // turn on blower for 4 seconds, then off
//        previousMillis = millis();
//        gotoCorner();
//        zFullUp();
//        delay(2000);
        //Serial.write(1);
        break;
      //_________________________________________________________________
      case 2: // Test case

        // RUNNING MODE enter the numbers from serial port --- Mode; FinalRobot X; FinalRobotY; RobotAngle
        // Example input:  1;10;5;100;200

        goXRight(rFinalX);
        goYUp(rFinalY);
        delay(1000);
        gotoCorner();
        //Serial.write(2);
        break;
      //_________________________________________________________________
      case 3: // 'SetXY' case
        // RUNNING MODE enter the numbers from serial port --- Mode; FinalRobot X; FinalRobotY; RobotAngle
        // Example input:  1;10;5;100;200
        goXRight(rFinalX);
        goYUp(rFinalY);
        //Serial.write(3);
        break;
      //_________________________________________________________________
      case 4: // 'Set Z' case
        goZDown(rFinalZ);
        //Serial.write(4);
        break;
      //_________________________________________________________________
      case 5: // 'Do main' case
        moveXYAtSpeed(xSpeed, ySpeed, mainTime);
        //Serial.write(5);
        break;
      //_________________________________________________________________
      case 6: // 'Retract Z' case
        zFullUp();
       // Serial.write(6);
        break;
      //_________________________________________________________________
      case 7: // Fluidize case
        blowerOnOff(); // May need longer time, but likely not
       // Serial.write(7);
        break;
      default: // No mode cmd received, do nothing
        break;
    }
  }
}


//      //_________________________________________________________________
//      if (mode==2){
//        // RUNNING MODE enter the numbers from serial port --- Mode; FinalRobot X; FinalRobotY; RobotAngle
//        // Example input:  1;10;10;50
//
//        goXRight(rFinalX);
//        goYUp(rFinalY);
//        delay(1000);
//        servoWrite(rAngle);
//        electromagnetOn();
//        delay(1000);
//        gripperDown();
//        delay(2000);
//        gripperUp();
//
//        GripperServo.writeMicroseconds(1400); // Align robot
//        gotoCenterX();
//        gotoCenterY(rFinalY);
//
//        Serial.println("Done");
//        delay(2000);
//  }
//  if (mode==3){
//        delay(2000);
//        blowerOnOff();
//        delay(2000);
//        gripperDown();
//        electromagnetOff();
//        delay(1000);
//        gripperUp();
//        gotoCorner();
//
//        Serial.println("Done");
//        delay(2000);
//
//
//  }


