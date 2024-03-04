#define CONTROLLER_FREQ 25.
#define USE_TIMER_1     true
// #define USE_TIMER_2     true

#include <Arduino.h>
#include <sensors/ir_line.cpp>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>
#include <motor_control/motor_control.cpp>
#include <Metro/Metro.h>



IMU imu;
// State functions
void start();
void driving_to_box();
void aligning_with_gap();
void turning_to_gap();
void driving_through_gap();
void turning_to_contact_zone();
void driving_to_contact_zone();
void turning_to_shooting_zone();
void driving_to_shooting_zone();
void (*state) (void) = start;
// Control functions
void controller();
static volatile float wz;
static volatile float iwz;
static bool gyro_controller_on = false;
int8_t course = 1;

void setup() {
    Serial.begin(9600);
    stop();
}
volatile float wr_cmd;
void loop() {
    imu.update_measurement();
    state();
    Serial.println(iwz);
    Serial.print("Left:");
    Serial.print(ir_left_triggers);
    Serial.print(",");
    Serial.print("Mid:");
    Serial.print(ir_mid_triggers);
    Serial.print(",");
    Serial.print("Right:");
    Serial.println(ir_right_triggers);
}
void start() {
    // if (Serial.available()) {
    if (millis() > 5000) {
        imu.initialize();
        imu.calibrate();
        forward();
        gyro_controller_on = true;
        ITimer1.init();
        ITimer1.setFrequency(CONTROLLER_FREQ, controller);
        state = driving_to_box;
        Serial.println("Entering driving_to_gap");
    }
}
volatile int time_box_reached;
void driving_to_box() {
    static volatile int left;
    static volatile int middle;
    static volatile int right;
    left = ir_left_triggers;
    middle = ir_mid_triggers;
    right = ir_right_triggers;
    int8_t counter = min(left,min(right,middle));
    if (counter >= 1) {
        state = aligning_with_gap;
        time_box_reached = millis();
    }
}
void aligning_with_gap() {
    if (millis() - time_box_reached > 500) {
        gyro_controller_on = false;
        stop();
        delay(1000);
        // imu.initialize();
        imu.calibrate();
        // Conisder if we should add this. might be more robust without?
        switch (course) {
            case 0:
                turn_left(FORWARD,2*PI);
                break;
            case 1:
                turn_right(FORWARD,2*PI);
                break;
        }
        imu.reset_integrators();
        state = turning_to_gap;
        Serial.println("Entering turning_to_gap");
    }
}
void turning_to_gap() {
    static volatile float angZ;
    static volatile bool turn_complete = false;
    angZ = imu.angZ;
    Serial.println(angZ);
    switch (course) {
        case 0:
            turn_complete = angZ > PI/2;
            break;
        case 1:
            turn_complete = angZ < -PI/2;
            break;
    }
    if (turn_complete) {
        stop();
        delay(1000);
        imu.calibrate();
        imu.reset_integrators();
        wz = imu.gyroZ;
        iwz = 0;
        reset_ir_triggers();
        forward();
        gyro_controller_on = true;
        state = driving_through_gap;
        Serial.println("Entering driving_through_gap");
        delay(3000);    
    }
}
int time_line_reached;
void driving_through_gap() {
    int8_t counter = min(ir_left_triggers,min(ir_right_triggers,ir_mid_triggers));
    // Serial.print(counter);
    // if (counter >= 3) {
    if(ir_left_on_line || ir_mid_on_line || ir_right_on_line) {
        stop();
        delay(1000);
        imu.calibrate();
        gyro_controller_on = false;
        switch (course) {
            case 0:
                turn_right(FORWARD,2*PI);
                break;
            case 1:
                turn_left(FORWARD,2*PI);
                break;
        }
        imu.reset_integrators();
        state = turning_to_contact_zone;
        Serial.println("Entering turning_to_contact_zone");
        time_line_reached = millis();
    }
}
void turning_to_contact_zone() {
    // static int angZ = 0;
    // static unsigned long prev_time = 0;
    // unsigned long curr_time = millis();
    // angZ += imu.gyroZ * (curr_time - prev_time)/1000.;
    // prev_time = curr_time;
    bool turn_complete = false;
    // Serial.println(imu.angZ);
    switch (course) {
        case 0:
            turn_complete = imu.angZ < -PI/2;
            break;
        case 1:
            turn_complete = imu.angZ > PI/2;
            break;
    }
    if (turn_complete || millis() - time_line_reached > 5000) { //90 deg turn left
        stop();
        delay(1000);
        imu.calibrate();
        wz = imu.gyroZ;
        iwz = 0;
        forward();
        imu.reset_integrators();
        gyro_controller_on = true;
        state = driving_to_contact_zone;
        Serial.println("Entering driving_to_contact_zone");
        delay(3000);
    }
}
void driving_to_contact_zone() {
    // if (abs(imu.dist_rate) < 0.05) {
        //TO BE CONTINUED
        gyro_controller_on = false;
        stop();
        delay(200);
        imu.calibrate();
        imu.reset_integrators();
        switch (course) {
            case 0:
                turn_left(BACKWARD,2*PI);
                break;
            case 1:
                turn_right(BACKWARD,2*PI);
                break;
        }
        state = turning_to_shooting_zone;
        Serial.println("Entering turning_to_shooting_zone");
    // }
}
void turning_to_shooting_zone() {
    bool turn_complete = false;
    switch (course) {
        case 0:
            turn_complete = imu.angZ > PI/2;
            break;
        case 1:
            turn_complete = imu.angZ < -PI/2;
            break;
    }
    if (turn_complete) {
        stop();
        delay(200);
        imu.calibrate();
        imu.reset_integrators();
        wz = imu.gyroZ;
        iwz = 0;
        forward();
        gyro_controller_on = true;
        state = driving_to_shooting_zone;
        Serial.println("Entering driving_to_shooting_zone");
        delay(6000);
    }
}
void driving_to_shooting_zone() {
    gyro_controller_on = false;
    stop();
    // static int timer;
    // timer++;
    // if (timer > 6000) {
        
    // }
}


float dw;
float kp = 2.;
float ki = 5.;
float sign(float x) {x >= 0 ? 1. : -1.;}
void controller() {    
    wz = imu.gyroZ;
    iwz = imu.angZ;
    imu.update_integrator();
    update_ir_states();
    if (gyro_controller_on) {
        // dw = -(kp*wz + ki*sign(iwz)*min(abs(iwz), 10))*BASE_WIDTH/WHEEL_RADIUS;
        dw = -(kp*wz + ki*iwz)*BASE_WIDTH/WHEEL_RADIUS;
        wr_cmd = dw + DEFAULT_MOTOR_SPEED;
        // analogWrite(EnA,(DEFAULT_MOTOR_SPEED + dw/2) * RPS_TO_ANALOG);
        // analogWrite(EnB,(DEFAULT_MOTOR_SPEED - dw/2) * RPS_TO_ANALOG);
        // analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
    }
    // } else {
    //     analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
    // }
}