// #include <Arduino.h>
// #define USE_TIMER_1 true
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>
#include <servo/servo.cpp>
#include <ISR_Timer.h>

// Servo servo;
ISR_Timer timer;
IMU imu;
void check_turn();
void setup() {
    Serial.begin(9600);
    stop();
    imu.initialize();
    delay(1000);
    imu.calibrate();
    // turn_right(FORWARD, 2.5*PI);
    turn_left(FORWARD, 2.0*PI);
    imu.reset_integrators();
    // ITimer1.init();
    // ITimer1.setFrequency(25, check_turn);
    timer.init();
    timer.setInterval(40, check_turn);
    // servo.write(60);
}

void loop() {
    imu.update_measurement();
    Serial.println(imu.angZ);
    timer.run();
}

void check_turn() {
    imu.update_integrator();
    if (abs(imu.angZ) > PI/2*85/90) { //90 deg turn
        stop();
    }
}
