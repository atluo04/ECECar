#include <ECE3.h>

//PINS
//const int calibratePin = 

//ERROR VALUES
uint16_t currentValues[8];
uint16_t firstPreviousValues[8];
uint16_t secondPreviousValues[8];

//WEIGHTS
const int W4 = 8;
const int W3 = 4;
const int W2 = 2;
const int W1 = 1;

int minimum[8] = {689, 629, 643, 574, 597, 713, 597, 762};
int maximum[8] = {1811,  1768.8,  1803,  1587,  1612,  1787,  1659,  1738};


void calibrate(){
  for(int i = 0; i < 8; i++){
      ECE3_read_IR(minimum);
  }
}

int findError() {
  int error = 0;
  uint16_t normalizedValues[8];
  for (int i = 0; i < 8; i++) {
    uint16_t value1 = currentValues[i];
    uint16_t value2 = firstPreviousValues[i];
    uint16_t value3 = secondPreviousValues[i];

    //normalize to minimums
    normalizedValues[i] = (value1 + value2 + value3) / 3 - minimum[i];

    //normalize to maximums
    normalizedValues[i] = normalizedValues[i] * 1000 / maximum[i];
  }
  error = normalizedValues[0] * W4 + normalizedValues[1] * W3 + normalizedValues[2] * W2 + normalizedValues[3] * W1 
          - normalizedValues[4] * W1 - normalizedValues[5] * W2 - normalizedValues[6] * W3 - normalizedValues[7] * W4;
  return error;
}

void setup() {
  ECE3_Init();
  ECE3_read_IR(currentValues);
  ECE3_read_IR(firstPreviousValues);
  ECE3_read_IR(secondPreviousValues);
  
  Serial.begin(9600);
}


void loop() {
  int error;
  for(int i = 0; i < 8; i++){
    secondPreviousValues[i] = firstPreviousValues[i];
    firstPreviousValues[i] = currentValues[i];
  }
  ECE3_read_IR(currentValues);
  error = findError();

}