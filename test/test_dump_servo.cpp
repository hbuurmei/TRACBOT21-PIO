
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 100 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;

#define SWIVEL_MAX 1700
#define SWIVEL_MID 1000
#define SWIVEL_MIN 500 / 1000000/SERVO_FREQ * 4096

#define SWIVEL_MIN

#define SWIVEL_MIN_USEC 450
// #define SWIVEL_MAX_USEC 2500
#define SWIVEL_MAX_USEC 1700

#define SWIVEL_MIN_DEG 0
//#define SWIVEL_MAX_DEG 270
#define SWIVEL_MAX_DEG 180

#define HATCH_CLOSED 200
#define HATCH_OPEN 750

uint16_t deg_to_pwm(long degrees){
    long usec = map(degrees, SWIVEL_MIN_DEG, SWIVEL_MAX_DEG, SWIVEL_MIN_USEC, SWIVEL_MAX_USEC);
    return 4096 * usec / (1000000/SERVO_FREQ);
}

void setup() {
    Serial.begin(9600);
    Serial.println("8 channel Servo test!");

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    pwm.setPWM(0, 0, deg_to_pwm(90));

    pwm.setPWM(1,0,200);
    delay(1000);
    pwm.setPWM(1, 0, 750);


    // pwm.setPWM(0, 0, deg_to_pwm(0));
    // delay(1000);
    // 
    // delay(1000);
    // pwm.setPWM(0, 0, deg_to_pwm(180));
    // delay(1000);

    // pwm.writeMicroseconds(0, SWIVEL_MIN);
    // delay(1000);
    // pwm.writeMicroseconds(0, SWIVEL_MID);
    // delay(1000);
    // pwm.writeMicroseconds(0, SWIVEL_MAX);
    // delay(1000);

    delay(10);
}



void loop() {
  // Drive each servo one at a time using setPWM()

 }
