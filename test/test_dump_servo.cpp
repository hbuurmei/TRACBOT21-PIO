#define SERVO_CTRL_PIN  3
#include <Arduino.h>
#include <Servo.h>

Servo my_servo;
void setup() {
    // put your setup code here, to run once:
    pinMode(SERVO_CTRL_PIN,OUTPUT);
    // analogWrite(SERVO_CTRL_PIN,50);
    
    my_servo.attach(SERVO_CTRL_PIN);
    my_servo.write(180);
    
}

void loop() {
}