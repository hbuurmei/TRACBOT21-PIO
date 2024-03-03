#define CONTROLLER_FREQ 25.
#define USE_TIMER_1     true
#define USE_TIMER_2     true

#include <Arduino.h>
#include <sensors/ir_line.cpp>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>
#include <motor_control/motor_control.cpp>



IMU imu;
// State functions
void start();
void driving_to_gap();
void (*state) (void) = start;
// Control functions
void update_integrator();
void controller();
bool gyro_controller_on = false;
int8_t course = 1;

void setup() {
    Serial.begin(9600);
    forward();
    // forward();
    // stop();
    imu.initialize();
    imu.calibrate();
    
    ITimer1.init();
    ITimer1.setFrequency(CONTROLLER_FREQ, controller);
    ITimer1.setFrequency(CONTROLLER_FREQ,update_integrator);
    // ITimer2.init();
    // ITimer2.setFrequency(CONTROLLER_FREQ, update_integrator);
}
void loop() {
    // state();
    // forward();
}
void start() {
    if (millis() > 3000) {
        imu.reset_integrators();
        forward();
        gyro_controller_on = true;
        state = driving_to_gap;
        Serial.println("Entering driving_to_gap");
        delay(2000); // To prevent interference from box line
    }
}
void driving_to_gap() {
    bool gap_detected = false;
    switch (course) {
        case 0:
            gap_detected = line_detected(IR_LEFT);
            break;
        case 1:
            gap_detected = line_detected(IR_RIGHT);
            break;
    }
    if (gap_detected) {
        switch (course) {
            case 0:
                turn_left(FORWARD);
                break;
            case 1:
                turn_right(FORWARD);
                break;
        }
        gyro_controller_on = false;
        Serial.println("Entering aligning_with_gap");
    }
}

float wz;
float dw;
float wr_cmd;
float kp = 2.;
float ki = 5;
void controller() {    
    if (gyro_controller_on) {
        dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_WIDTH/WHEEL_RADIUS;
        wr_cmd = dw + DEFAULT_MOTOR_SPEED;
        analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
    } else {
        analogWrite(EnA,DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
    }
}
void update_integrator() {
    imu.update_integrator();
}