#define SERVO_CTRL_PIN  3
#define CLOSE   0   //for dumper hatch servo
// #define RELEASE 360 //for dumper hatch servo
#define RELEASE 80

#define IN1     13
#define IN2     12
#define IN3     8
#define IN4     7
#define EnA     11
#define EnB     6

#include <Arduino.h>
#include <Servo.h>

Servo my_servo;
void setup() {
    //turn off motors
    Serial.begin(9600);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
    pinMode(EnA,OUTPUT);
    pinMode(EnB,OUTPUT);
    // Configure Motor 1 (LEFT)
    digitalWrite(IN1,LOW);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,LOW);  // Opposite of IN1
    analogWrite(EnA,200);
    // Configure Motor 2 (RIGHT)
    digitalWrite(IN3,LOW);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,LOW);  // Opposite of IN3
    analogWrite(EnB,200);

    // put your setup code here, to run once:
    pinMode(SERVO_CTRL_PIN,OUTPUT);
    // analogWrite(SERVO_CTRL_PIN,50);
    
    my_servo.attach(SERVO_CTRL_PIN);
    my_servo.write(RELEASE);
    delay(3000);
    my_servo.write(CLOSE);
    
}

void loop() {
}