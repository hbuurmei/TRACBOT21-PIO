#define SERVO_CTRL_PIN  10
#include <Arduino.h>
#include <Servo.h>
// #include <motor_control/motor_control.cpp>

Servo my_servo;
void setup() {
    // put your setup code here, to run once:
    // stop();
    // pinMode(SERVO_CTRL_PIN,OUTPUT);
    // analogWrite(SERVO_CTRL_PIN,255);
    
    my_servo.attach(SERVO_CTRL_PIN);
    my_servo.write(0);
    delay(500);
    my_servo.write(90);
    delay(500);
    my_servo.write(0);
    delay(500);
    my_servo.write(90);
    delay(500);
    my_servo.write(0);
    delay(500);
    my_servo.write(90);
    delay(500);
    my_servo.write(0);
    delay(500);
    my_servo.write(90);
    delay(500);
    my_servo.write(0);
    delay(500);
    my_servo.write(90);
    delay(500);
    my_servo.write(0);
    
}

void loop() {
}