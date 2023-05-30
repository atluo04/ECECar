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
uint16_t secondPreviousValues[8];
double error = 0;
double previousError = 0;

//WEIGHTS
double W4 = 2;
double W3 = 1.5;
double W2 = 1.25;
double W1 = 1;
int minimum[8] = {740, 692, 716,575,528,528,366,482};
int maximum[8] = {2500,2500,2500,2500,2500,2500,2500,2500};

//PID CONSTANTS
float Kp = 0.068;
float Kd = 0.4;

const float Kp40 = 0.01;
const float Kd40 = 0.04;

const float Kp70 = 0.04;
const float Kd70 = 0.06;

const float Kp200 = 0.008;   //need to find values
const float Kd200 = 0.08;   


//SPEEDs
int BASESPEED = 100;
int leftSpeed = BASESPEED;
int rightSpeed = BASESPEED;
const int TURNINGSPEED = 150;

//TRACK IDENTIFIERS
const int CROSSPIECETHRESHOLD = 1600;
const int TRACKBREAK = 3350;
const int TRACKBREAKEND = 3750;
int STRAIGHTSTART = 2000;
const int TURNSTART = 2800;
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
  ECE3_read_IR(secondPreviousValues);
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
  delay(2000);
   changeWheelSpeeds(0, BASESPEED, 0, BASESPEED);
}

void loop() {
  ECE3_read_IR(currentValues);
  if (atCrossPiece()) {
    if (!passedCross) {
      changeWheelSpeeds(leftSpeed, 0, rightSpeed, 0);
      resetEncoderCount_left();
      resetEncoderCount_right();

      digitalWrite(left_dir_pin, HIGH);
      changeWheelSpeeds(0, TURNINGSPEED, 0, TURNINGSPEED);
      bool turning = true;       
      while (turning) {
        if (getEncoderCount_left() > 220) {  //for speed 50 -> 350,speed 150 -> 220
          turning = false;
        }
      }
      changeWheelSpeeds(TURNINGSPEED, 0, TURNINGSPEED, 0);
      digitalWrite(left_dir_pin, LOW);
      BASESPEED = 70;
      resetEncoderCount_left();
      changeWheelSpeeds(0, BASESPEED, 0, BASESPEED);
      passedCross = true;
      boosted = false;
      slowed = false;
      STRAIGHTSTART = 1500;
    }
    else {
      changeWheelSpeeds(leftSpeed, 0, rightSpeed, 0);
      exit(0);
    }
  }
  else {
    int leftEncoder = getEncoderCount_left();

    error = findError();
    double rateError = error - previousError;
    leftSpeed = BASESPEED + error * Kp + rateError * Kd;
    rightSpeed = BASESPEED - error * Kp - rateError * Kd;
    analogWrite(left_pwm_pin, leftSpeed);
    analogWrite(right_pwm_pin, rightSpeed);

    if(slowed){
      Serial.print(error);
      Serial.println();
    }

    //for speeding up car
    if (!passedCross) {
      if (!boosted && leftEncoder > STRAIGHTSTART) {
        BASESPEED = 150;
        Kp = Kp200;
        Kd = Kd200;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        boosted = true;
      }
      else if (!slowed && leftEncoder > TRACKBREAK) {
        BASESPEED = 25;
        Kp = Kp40;
        Kd = Kd40;
        W4 = 0;
        W3 = 1.5;
        W2 = 1.25;
        W1 = 1;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        slowed = true;
      }
      else if(slowed && !returnToBase && leftEncoder > TRACKBREAKEND){
        BASESPEED = 80;
        W4 = 2;
        W3 = 1.5;
        W2 = 1.25;
        W1 = 1;
        Kp = Kp70;
        Kd = Kd70;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        returnToBase = true;
      }
      leftSpeed = BASESPEED;
      rightSpeed = BASESPEED;
    }
    else if(passedCross){
      if (!boosted && leftEncoder > STRAIGHTSTART) {
        BASESPEED = 150;
        Kp = Kp200;
        Kd = Kd200;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        leftSpeed = BASESPEED;
        rightSpeed = BASESPEED;
        boosted = true;
      }
      else if (!slowed && leftEncoder > TURNSTART) {
        BASESPEED = 70;
        Kp = Kp70;
        Kd = Kd70;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        leftSpeed = BASESPEED;
        rightSpeed = BASESPEED;
        slowed = true;
      }
    }

    for (int i = 0; i < 8; i++) {
      secondPreviousValues[i] = firstPreviousValues[i];
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
    if (currentValues[i] < CROSSPIECETHRESHOLD && firstPreviousValues[i] < CROSSPIECETHRESHOLD)
      return false;
  }
  return true;
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
