#define SERVO_CTRL_PIN  5
#define USE_TIMER     true
#include <Arduino.h>
#include <Servo.h>
#include <motor_control/motor_control.cpp>
#include <TimerInterrupt.h>
#include <sensors/imu.cpp>

IMU imu;
Servo my_servo;
void setup() {
    // put your setup code here, to run once:
    stop();
    imu.initialize();
    // pinMode(SERVO_CTRL_PIN,OUTPUT);
    // analogWrite(SERVO_CTRL_PIN,255);
    ITimer.init();

    my_servo.attach(SERVO_CTRL_PIN);
    // my_servo.write(130);
    my_servo.write(180);
    
}

void loop() {
    imu.update_measurement();
}