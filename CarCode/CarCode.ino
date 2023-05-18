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
const double W4 = 2;
const double W3 = 1.5;
const double W2 = 1.25;
const double W1 = 1;
int minimum[8] = {689, 629, 643, 574, 597, 713, 597, 762};
int maximum[8] = {1811,  1768.8,  1803,  1587,  1612,  1787,  1659,  1738};

//PID CONSTANTS
const float Kp = 0.03;
const float Kd = 0.025;

//SPEEDs
const int BASERIGHTSPEED = 80;
const int BASELEFTSPEED = 80;
int leftSpeed = 0;
int rightSpeed = 0;
const int TURNINGSPEED = 150;
int speedIncrease = 0;

//TRACK IDENTIFIERS
bool onStraight = true;
bool passedCross = false;
/*
  void calibrate() {
  for (int i = 0; i < 8; i++) {
    ECE3_read_IR(minimum);
  }
  } */

void setup() {
  ECE3_Init();
  ECE3_read_IR(currentValues);
  ECE3_read_IR(firstPreviousValues);
  ECE3_read_IR(secondPreviousValues);

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

  changeWheelSpeeds(0, BASELEFTSPEED, 0, BASERIGHTSPEED);

  Serial.begin(9600);
}

void loop() {
  ECE3_read_IR(currentValues);
  if (atCrossPiece() && !passedCross) {
    changeWheelSpeeds(leftSpeed,0, rightSpeed, 0);
    resetEncoderCount_left();
    resetEncoderCount_right();

    digitalWrite(left_dir_pin, HIGH);
    changeWheelSpeeds(0, TURNINGSPEED, 0, TURNINGSPEED);
    bool turning = true;
    while(turning){
        if(getEncoderCount_left() > 7200){
            turning = false;
        }
    }
    digitalWrite(left_dir_pin, LOW);
    passedCross = true;
  }
  else {
    error = findError();
    double rateError = error - previousError;

    leftSpeed = BASELEFTSPEED + error * Kp + rateError * Kd + speedIncrease;
    rightSpeed = BASERIGHTSPEED - error * Kp - rateError * Kd + speedIncrease;
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
    if (currentValues[i] < 1700 && firstPreviousValues[i] < 1700)
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
