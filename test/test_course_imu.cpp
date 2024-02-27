// #include <Arduino.h>
#define USE_TIMER_1 true
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>

IMU imu;
float dist_cmd = 0.3;
bool check_distance();
bool check_turn();
void update_state();
void controller();
void setup() {
    Serial.begin(9600);
    stop();
    imu.initialize();
    delay(5000);
    imu.calibrate();
    forward();
    imu.reset_integrators();
    ITimer1.init();
    ITimer1.setFrequency(25, update_state);
}
bool (*check) () = check_distance;
void loop() {
    imu.update_measurement();
    if(check()) {
        stop();
    }
    // Serial.println(imu.angZ);
}

void update_state() {
    imu.update_integrator();
    controller();
}

bool check_distance() {
    if (sqrt(imu.posY*imu.posY + imu.posX*imu.posX) > dist_cmd) {
        return true;
    }
    return false;
}

bool check_turn() {
    if (imu.angZ > dist_cmd) {
        return true;
    }
    return false;
}

float kp = 2.;
float ki = 0.5;
float dw;
float wr_cmd;
void controller() {
    // static float iwz = 0;
    // if (abs(wz) > 0.01) {
    dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_WIDTH/WHEEL_RADIUS;
    wr_cmd = dw + DEFAULT_MOTOR_SPEED;
    analogWrite(EnB,wr_cmd * RPS_TO_ANALOG);
    // iwz += wz/CONTROLLER_FREQ;
    // } else {Serial.print("\t");}
}
