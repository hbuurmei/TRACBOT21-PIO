#define IN1     13
#define IN2     12
#define IN3     8
#define IN4     7
#define EnA     11
#define EnB     6
#define DEFAULT_MOTOR_SPEED 4*PI // rad/s
#define MAX_MOTOR_SPEED 20.9440 // rad/s
#define RPS_TO_ANALOG 256 / MAX_MOTOR_SPEED
#define WHEEL_RADIUS 0.042 // metets
#define BASE_WIDTH 0.3048
#define USE_TIMER_1         true
#define CONTROLLER_FREQ 25.

#include <Arduino.h>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>

IMU imu;
enum MODE {
    STOP,
    FORWARD
};
MODE curr_mode = FORWARD;
void handle_mode(MODE mode);
void controller();
void controller2();
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    handle_mode(curr_mode);
    imu.initialize();
    imu.calibrate();
    ITimer1.init();
    ITimer1.setFrequency(CONTROLLER_FREQ, controller2);
}

void handle_mode(MODE mode) {
    switch(mode) {
        case STOP:
            // Right Motor
            digitalWrite(IN1,LOW);
            digitalWrite(IN2,LOW);
            analogWrite(EnA,0);
            // Left Motor
            digitalWrite(IN3,LOW);
            digitalWrite(IN4,LOW);
            analogWrite(EnB,0);
        case FORWARD:
            // Right Motor
            digitalWrite(IN1,HIGH);
            digitalWrite(IN2,LOW);
            analogWrite(EnA,DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
            // Left Motor
            digitalWrite(IN3,LOW);
            digitalWrite(IN4,HIGH);
            analogWrite(EnB,DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
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
float ki = 0.5;
void controller() {
    static float iwz = 0;
    wz = imu.gyroZ;
    // if (abs(wz) > 0.01) {
    dw = -(kp*wz + ki*iwz)*BASE_WIDTH/WHEEL_RADIUS;
    wr_cmd = dw + DEFAULT_MOTOR_SPEED;
    analogWrite(EnB,wr_cmd * RPS_TO_ANALOG);
    iwz += wz/CONTROLLER_FREQ;
    // } else {Serial.print("\t");}
}

void controller2() {
    static float yi_1 = 0;
    static float yi_2 = 0;
    static float ui_1 = 0;
    static float ui_2 = 0;
    dw = 0.03764*yi_1 - 0.03764*yi_2 + 1.879*ui_1 - 0.8869*ui_2;
    wr_cmd = dw*BASE_WIDTH/WHEEL_RADIUS + DEFAULT_MOTOR_SPEED;
    analogWrite(EnB,wr_cmd * RPS_TO_ANALOG);
    yi_2 = yi_1;
    yi_1 = imu.gyroZ;
    ui_2 = ui_1;
    ui_1 = dw;
}