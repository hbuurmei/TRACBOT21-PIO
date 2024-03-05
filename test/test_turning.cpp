// #include <Arduino.h>
// #define USE_TIMER_1 true
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>
// #include <Servo.h>
#include <servo/servo.cpp>
// #include <TimerInterrupt.h>
#include <ISR_Timer.h>

Servo servo;
ISR_Timer timer;
IMU imu;
void check_turn();
void setup() {
    Serial.begin(9600);
    stop();
    imu.initialize();
    // delay(5000);
    imu.calibrate();
    turn_left(MIDDLE,DEFAULT_MOTOR_SPEED);
    imu.reset_integrators();
    // ITimer1.init();
    // ITimer1.setFrequency(25, check_turn);
    timer.init();
    timer.setInterval(40, check_turn);
    servo.attach(3);
    servo.write(60);
}

void loop() {
    imu.update_measurement();
    Serial.println(imu.angZ);
    timer.run();
}

void check_turn() {
    imu.update_integrator();
    if (abs(imu.angZ) > PI/2) { //90 deg turn
        stop();
    }
}
