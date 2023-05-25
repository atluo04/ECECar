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
int minimum[8] = {689, 629, 643, 574, 597, 713, 597, 762};
int maximum[8] = {1811,  1768.8,  1803,  1587,  1612,  1787,  1659,  1738};

//PID CONSTANTS
const float Kp = 0.03;
const float Kd = 0.05;

//SPEEDs
int BASESPEED = 70;
int leftSpeed = BASESPEED;
int rightSpeed = BASESPEED;
const int TURNINGSPEED = 150;

//TRACK IDENTIFIERS
bool passedCross = false;
const int CROSSPIECETHRESHOLD = 1700;
const int TRACKBREAK = 3400;
int STRAIGHTSTART = 800;
bool boosted = false;
bool slowed = false;
const int TURNSTART = 2500;

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

  changeWheelSpeeds(0, BASESPEED, 0, BASESPEED);

  Serial.begin(9600);
}

void loop() {
  ECE3_read_IR(currentValues);
  error = findError();
  if (atCrossPiece()) {
    if(!passedCross){
    changeWheelSpeeds(leftSpeed,0, rightSpeed, 0);
    resetEncoderCount_left();
    resetEncoderCount_right();

    digitalWrite(left_dir_pin, HIGH);
    changeWheelSpeeds(0, TURNINGSPEED, 0, TURNINGSPEED);
    bool turning = true;
    while(turning){
        if(getEncoderCount_left() > 220){    //for speed 50 -> 350,speed 150 -> 220
            turning = false;
        }
    }
    changeWheelSpeeds(TURNINGSPEED, 0, TURNINGSPEED,0);
    digitalWrite(left_dir_pin, LOW);
    BASESPEED = 70;
    changeWheelSpeeds(0, BASESPEED, 0, BASESPEED);
    passedCross = true;
    boosted = false;
    slowed = false;
    }
    else {
      changeWheelSpeeds(leftSpeed, 0, rightSpeed, 0);
      exit(0);
    }
  }
  else {
    double rateError = error - previousError;
    int leftEncoder = getEncoderCount_left();
    
    leftSpeed = BASESPEED + error * Kp + rateError * Kd;
    rightSpeed = BASESPEED - error * Kp - rateError * Kd;
  
    if(!boosted && leftEncoder > STRAIGHTSTART){
      BASESPEED = 200;
      changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
      boosted = true;
    }
    
    if(!passedCross && !slowed && leftEncoder > TRACKBREAK){
        BASESPEED = 40;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
        slowed = true;
    }

    if(passedCross){
      if(!boosted && leftEncoder > STRAIGHTSTART){
          BASESPEED = 200;
          changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
      }
      if(!slowed && leftEncoder > TURNSTART){
        BASESPEED = 70;
        changeWheelSpeeds(leftSpeed, BASESPEED, rightSpeed, BASESPEED);
      }
    }

    analogWrite(left_pwm_pin, leftSpeed);
    analogWrite(right_pwm_pin, rightSpeed);

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
