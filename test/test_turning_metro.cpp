#include <Arduino.h>
#include <Metro.h>
#include <sensors/imu.cpp>
#include <motor_control/motor_control.cpp>

IMU imu;
Metro timer = Metro(40);
void check_turn();
void setup() {
    Serial.begin(9600);
    delay(3000);
    imu.initialize();
    imu.calibrate();
    imu.reset_integrators();
    turn_right(MIDDLE);
}
void loop() {
    imu.update_measurement();
    check_turn();
}
void check_turn() {
    static float angZ = 0;
    static unsigned long prev_time = 0;
    unsigned long curr_time = millis();
    angZ += imu.gyroZ * ((curr_time - prev_time)/1000.);
    Serial.println(angZ);
    if (angZ < -100*PI/180) {stop();}
    prev_time = curr_time;
}