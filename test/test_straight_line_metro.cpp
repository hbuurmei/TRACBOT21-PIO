#define CONTROLLER_FREQ 25.

#include <Arduino.h>
#include <sensors/imu.cpp>
#include <Metro.h>
#include <motor_control/motor_control.cpp>

IMU imu;
Metro timer = Metro(40);
enum MODE {
    STOP,
    GO_FORWARD
};
MODE curr_mode = GO_FORWARD;
void handle_mode(MODE mode);
void controller();
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    imu.initialize();
    imu.calibrate();
    imu.reset_integrators();
    handle_mode(curr_mode);
}

void handle_mode(MODE mode) {
    switch(mode) {
        case STOP:
            // Left Motor
            digitalWrite(IN1,LOW);
            digitalWrite(IN2,LOW);
            analogWrite(EnA,0);
            // Right Motor
            digitalWrite(IN3,LOW);
            digitalWrite(IN4,LOW);
            analogWrite(EnB,0);
        case GO_FORWARD:
            forward();
    }
}
float wz;
float dw;
float wr_cmd;
void loop() {   
    imu.update_measurement();
    if (timer.check()) {
        timer.reset();
        // controller();
    }
    Serial.print("wz: ");Serial.print(wz);Serial.print("\t\t");
    Serial.print("wr_cmd: ");Serial.println(wr_cmd);
}

float kp = 2.;
float ki = 5;
void controller() {
    // static float iwz = 0;
    // wz = imu.gyroZ;
    // if (abs(wz) > 0.01) {
    imu.update_integrator();
    dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_WIDTH/WHEEL_RADIUS;
    wr_cmd = dw + DEFAULT_MOTOR_SPEED;
    analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
    // iwz += wz/CONTROLLER_FREQ;
    // } else {Serial.print("\t");}
} 
