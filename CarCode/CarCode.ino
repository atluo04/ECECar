#include <ECE3.h>

//PINS
//const int calibratePin =
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

//ERROR VALUES
uint16_t currentValues[8];
uint16_t firstPreviousValues[8];
float error = 0;
float previousError = 0;

//WEIGHTS
float W4 = -2;
float W3 = -1.5;
float W2 = -1.25;
float W1 = -1;
int minimum[8] = {807, 713, 689, 549, 619, 689, 619, 689};
int maximum[8] = {2500,2500,2500,2500,2500,2500,2500,2500};

//PID CONSTANTS
float Kp = 0.065;
float Kd = 0.36;

const float Kp40 = 0.04;
const float Kd40 = 0.35;

const float Kp100 = 0.065;
const float Kd100 = 0.36;

const float Kp200 = 0.03;   //need to find values
const float Kd200 = 0.373;   


//SPEEDs
int BASESPEED = 100;
int leftSpeed = BASESPEED;
int rightSpeed = BASESPEED;
const int TURNINGSPEED = 180;

//TRACK IDENTIFIERS
const int CROSSPIECETHRESHOLD = 1600;
const int TRACKBREAK = 3400;
const int TRACKBREAKEND = 3800;
int STRAIGHTSTART = 1850;
const int TURNSTART = 2400;
bool boosted = false;
bool slowed = false;
bool returnToBase = false;
bool passedCross = false;

/*
  void calibrate() {
  for (int i = 0; i < 8; i++) {
    ECE3_read_IR(minimum);
  }
  } */

void setup() {
  ECE3_Init();
  ECE3_read_IR(firstPreviousValues);
  ECE3_read_IR(currentValues);

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  Serial.begin(9600);
  error = findError();
  previousError = error;
  delay(2000);
  changeWheelSpeeds(0, BASESPEED, 0, BASESPEED);
}

void loop() {
  ECE3_read_IR(currentValues);
  if (atCrossPiece()) {
    if (!passedCross) {
      turnAround();
      Kp = Kp200;
      Kd = Kd200;
      BASESPEED = 155;
      passedCross = true;
      boosted = false;
      slowed = false;
      STRAIGHTSTART = 1700;
    }
    else {
      changeWheelSpeeds(leftSpeed, 0, rightSpeed, 0);
      exit(0);
    }
  }
  else {
    int leftEncoder = getEncoderCount_left();

    error = findError();
    float rateError = error - previousError;
    float output = Kp * error + Kd * rateError;

    leftSpeed = BASESPEED - output;
    rightSpeed = BASESPEED + output;

    if(leftSpeed > 255){
      leftSpeed = 254;
    }
    if(leftSpeed < 0){
      leftSpeed = 0;
    }
    if(rightSpeed > 255){
      rightSpeed = 254;
    }
    if(rightSpeed < 0){
      rightSpeed = 0;
    }
    
    analogWrite(left_pwm_pin, leftSpeed);
    analogWrite(right_pwm_pin, rightSpeed);

    //for speeding up car
    if (!passedCross) {
      if (!boosted && leftEncoder > STRAIGHTSTART) {
        BASESPEED = 155;
        Kp = Kp200;
        Kd = Kd200;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        boosted = true;
      }
      else if (!slowed && leftEncoder > TRACKBREAK) {
        BASESPEED = 30;
        Kp = Kp40;
        Kd = Kd40;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        slowed = true;
      }
      else if(slowed && !returnToBase && leftEncoder > TRACKBREAKEND){
        BASESPEED = 100;
        Kp = Kp100;
        Kd = Kd100;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        returnToBase = true;
      }
    }
    else if(passedCross){
      /*if (!boosted && leftEncoder > STRAIGHTSTART) {
        BASESPEED = 150;
        Kp = Kp200;
        Kd = Kd200;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        boosted = true;
      }
      else*/ if (!slowed && leftEncoder > TURNSTART) {
        BASESPEED = 100;
        Kp = Kp100;
        Kd = Kd100;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        slowed = true;
      }
    }

    for (int i = 0; i < 8; i++) {
      firstPreviousValues[i] = currentValues[i];
    }
    previousError = error;
  }
}

double findError() {
  double error;
  int normalizedValues[8];
  for (int i = 0; i < 8; i++) {
    int value1 = currentValues[i];

    //normalize to minimums
    normalizedValues[i] = value1 - minimum[i];

    //normalize to maximums
    normalizedValues[i] = normalizedValues[i] * 1000 / maximum[i];
  }
  error = normalizedValues[0] * W4 + normalizedValues[1] * W3 + normalizedValues[2] * W2 + normalizedValues[3] * W1
          - normalizedValues[4] * W1 - normalizedValues[5] * W2 - normalizedValues[6] * W3 - normalizedValues[7] * W4;
  return error;
}

bool atCrossPiece() {
  for (int i = 0; i < 8; i++) {
    if (currentValues[i] < CROSSPIECETHRESHOLD) //&& firstPreviousValues[i] < CROSSPIECETHRESHOLD)
      return false;
  }
  return true;
}

void turnAround(){
      changeWheelSpeeds(BASESPEED, 0, BASESPEED, 0);
      resetEncoderCount_left();
      resetEncoderCount_right();
      digitalWrite(left_dir_pin, HIGH);
      changeWheelSpeeds(0, TURNINGSPEED, 0, TURNINGSPEED);
      bool turning = true;       
      while (turning) {
        if (getEncoderCount_left() > 170 && getEncoderCount_right() > 170) {  //for speed 50 -> 350,speed 150 -> 220
          turning = false;
        }
      }
      changeWheelSpeeds(TURNINGSPEED, 0, TURNINGSPEED, 0);
      digitalWrite(left_dir_pin, LOW);
      resetEncoderCount_left();
      changeWheelSpeeds(TURNINGSPEED, BASESPEED, TURNINGSPEED, BASESPEED);
}

void changeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
  int diffLeft = finalLeftSpd - initialLeftSpd;
  int diffRight = finalRightSpd - initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft) / stepIncrement;
  int numStepsRight = abs(diffRight) / stepIncrement;
  int numSteps = max(numStepsLeft, numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft) / numSteps; // left in(de)crement
  int deltaRight = (diffRight) / numSteps; // right in(de)crement
  for (int k = 0; k < numSteps; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin, pwmLeftVal);
    analogWrite(right_pwm_pin, pwmRightVal);
    delay(30);
  }
  analogWrite(left_pwm_pin, finalLeftSpd);
  analogWrite(right_pwm_pin, finalRightSpd);
}
