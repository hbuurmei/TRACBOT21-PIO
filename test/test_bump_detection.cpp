#define bump_acc_threshold 5.0

#include <Arduino.h>
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>

IMU imu;

void setup() {
    Serial.begin(9600);
    imu.initialize();
    imu.calibrate();
    forward();
}

void loop() {
    imu.update_measurement();
    if (imu.accelY > bump_acc_threshold) {
        Serial.print("a_y:\t");
        Serial.print(imu.accelY); Serial.println("\t");
    }
}
