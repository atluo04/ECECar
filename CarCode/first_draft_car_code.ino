#define EMITEVEN 61
#define EMITODD_R 45
#define REFL0 65
#define REFL1 48
#define REFL2 64
#define REFL3 47
#define REFL4 52
#define REFL5 68
#define REFL6 53
#define REFL7 69

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int leftStartSpeed = 80;
const int rightStartSpeed = 79;

const double Kp = 15.0;
const double Kd = 7.5;
 
const int number_derivative_samples = 10;
int counter = 0;
double prev_error = 0; //6 samples back

int turnAroundState = 0;
    //0 = not reached turn
    //1 = reached first black line

double samples [] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
    Serial.begin(9600);

    pinMode(EMITEVEN,OUTPUT);
    pinMode(EMITODD_R,OUTPUT);

    digitalWrite(EMITEVEN, HIGH);
    digitalWrite(EMITODD_R, HIGH);

    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);
    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);
   

    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(right_nslp_pin, HIGH);
    
    analogWrite(left_pwm_pin , leftStartSpeed);
    analogWrite(right_pwm_pin, rightStartSpeed);
  turnAroundState = 0;  
}

void loop() {
pinMode(REFL0,OUTPUT);
    pinMode(REFL1,OUTPUT);
    pinMode(REFL2,OUTPUT);
    pinMode(REFL3,OUTPUT);
    pinMode(REFL4,OUTPUT);
    pinMode(REFL5,OUTPUT);
    pinMode(REFL6,OUTPUT);
    pinMode(REFL7,OUTPUT);   

    digitalWrite(REFL0, HIGH);
    digitalWrite(REFL1, HIGH);
    digitalWrite(REFL2, HIGH);
    digitalWrite(REFL3, HIGH);
    digitalWrite(REFL4, HIGH);
    digitalWrite(REFL5, HIGH);
    digitalWrite(REFL6, HIGH);
    digitalWrite(REFL7, HIGH);

    delayMicroseconds(10000);
       
    pinMode(REFL0, INPUT);
    pinMode(REFL1, INPUT);
    pinMode(REFL2, INPUT);
    pinMode(REFL3, INPUT);
    pinMode(REFL4, INPUT);
    pinMode(REFL5, INPUT);
    pinMode(REFL6, INPUT);
    pinMode(REFL7, INPUT);
    
    delay(10);

    int refl0 = digitalRead(REFL0);
    int refl1 = digitalRead(REFL1);
    int refl2 = digitalRead(REFL2);
    int refl3 = digitalRead(REFL3);
    int refl4 = digitalRead(REFL4);
    int refl5 = digitalRead(REFL5);
    int refl6 = digitalRead(REFL6);
    int refl7 = digitalRead(REFL7);

    //sensor fusion
    double r0 = refl0 * -1.75;
    double r1 = refl1 * -1.25;
    double r2 = refl2 * -.75;
    double r3 = refl3 * -.25;
    double r4 = refl4 * .25;
    double r5 = refl5 * .75;
    double r6 = refl6 * 1.25;
    double r7 = refl7 * 1.75;
    
    if (turnAroundState == 0 && refl1 && refl2 && refl3 && refl4 &&refl5 && refl6 && refl7) {
        turnAroundState = 1;
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        delay(5);
        
        digitalWrite(right_dir_pin, HIGH);
        analogWrite(left_pwm_pin, leftStartSpeed);
        analogWrite(right_pwm_pin, rightStartSpeed);
        delay(500);
        digitalWrite(right_dir_pin, LOW);
    }

    else if (turnAroundState == 1 && refl1 && refl2 && refl3 && refl4 &&refl5 && refl6 && refl7) {
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        digitalWrite(left_nslp_pin, LOW);
        digitalWrite(right_nslp_pin, LOW);
    }
  
    double sum = (r0 + r1 + r2 + r3 +r4 + r5 + r6 + r7);
    double error = Kp * sum + ((counter >= number_derivative_samples) ? (Kd * (sum - samples[0])) : 0);
    
    for (int i = 0; i < number_derivative_samples-1; i++) {
        samples[i] = samples[i + 1];
    }

    samples[number_derivative_samples-1] = sum;    
    analogWrite(left_pwm_pin, leftStartSpeed - error);
    analogWrite(right_pwm_pin, rightStartSpeed + error);
    
    counter++;
}
