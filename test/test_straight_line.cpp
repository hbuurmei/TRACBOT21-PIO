#define USE_TIMER_1         true
#define CONTROLLER_FREQ 25.

#include <Arduino.h>
#include <sensors/imu.cpp>
// #include <TimerInterrupt.h>
#include <motor_control/motor_control.cpp>

IMU imu;
enum MODE {
    STOP,
    GO_FORWARD
};
MODE curr_mode = GO_FORWARD;
void handle_mode(MODE mode);
void controller();
void controller2();
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    imu.initialize();
    imu.calibrate();
    imu.reset_integrators();
    handle_mode(curr_mode);
    // ITimer1.init();
    // ITimer1.setFrequency(CONTROLLER_FREQ, controller);
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
            forward(3*PI);
    }
}
float wz;
float dw;
float wr_cmd;
void loop() {   
    imu.update_measurement();
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
// float kp = 2.;
// float ki = 5.;
// void controller() {    
//     static volatile float wz;
//     static volatile float iwz;
//     wz = imu.gyroZ;
//     iwz = imu.angZ;
//     imu.update_integrator();
//     if (true) {
//         // dw = -(kp*imu.gyroZ + ki*sin(imu.angZ)*min(abs(imu.angZ), 20))*BASE_WIDTH/WHEEL_RADIUS;
//         dw = -(kp*wz + ki*iwz)*BASE_WIDTH/WHEEL_RADIUS;
//         wr_cmd = dw + DEFAULT_MOTOR_SPEED;
//         analogWrite(EnA,(DEFAULT_MOTOR_SPEED + dw/2) * RPS_TO_ANALOG);
//         analogWrite(EnB,(DEFAULT_MOTOR_SPEED - dw/2) * RPS_TO_ANALOG);
//     } else {
//         analogWrite(EnA,DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
//         analogWrite(EnB,DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
//     }
// }

void controller2() {
    static float yi_1 = 0;
    static float yi_2 = 0;
    static float ui_1 = 0;
    static float ui_2 = 0;
    dw = 0.03764*yi_1 - 0.03764*yi_2 + 1.879*ui_1 - 0.8869*ui_2;
    wr_cmd = dw*BASE_WIDTH/WHEEL_RADIUS + DEFAULT_MOTOR_SPEED;
    analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
    yi_2 = yi_1;
    yi_1 = imu.gyroZ;
    ui_2 = ui_1;
    ui_1 = dw;
}