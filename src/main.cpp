#define CONTROLLER_FREQ 50.
#define USE_TIMER_1     true
// #define USE_TIMER_2     true

#include <Arduino.h>
#include <sensors/ir_line.cpp>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>
#include <motor_control/motor_control.cpp>
#include <Metro.h>

Metro controller_timer = Metro(1000/CONTROLLER_FREQ);
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
    // imu.initialize();
    // imu.calibrate();
    // forward();
    // gyro_controller_on = true;
    // ITimer1.init();
    // ITimer1.setFrequency(CONTROLLER_FREQ, controller);
}
volatile float wr_cmd;
void loop() {
    imu.update_measurement();
    // imu.update_integrator();
    // update_ir_states();
    state();
    // Serial.println(iwz);
    // Serial.println(imu.angZ);
    // Serial.print("Left:");
    // Serial.print(ir_left_triggers);
    // Serial.print(",");
    // Serial.print("Mid:");
    // Serial.print(ir_mid_triggers);
    // Serial.print(",");
    // Serial.print("Right:");
    // Serial.println(ir_right_triggers);

    // if (controller_timer.check()) {
    //     controller_timer.reset();
    //     controller();
    // }
    // delay(1);
}
void start() {
    // if (Serial.available()) {
    if (millis() > 5000) {
        imu.initialize();
        imu.calibrate();
        imu.reset_integrators();
        forward();
        ITimer1.init();
        ITimer1.setFrequency(CONTROLLER_FREQ,[](){imu.update_integrator(CONTROLLER_FREQ);});
        gyro_controller_on = true;
        state = driving_to_box;
        Serial.println("Entering driving_to_gap");
    }
}
volatile int time_box_reached;
void driving_to_box() {
    // static volatile int left;
    // static volatile int middle;
    // static volatile int right;
    // left = ir_left_triggers;
    // middle = ir_mid_triggers;
    // right = ir_right_triggers;
    static bool ir_left_on_line = false;
    static int ir_left_triggers = 0;
    static bool ir_right_on_line = false;
    static int ir_right_triggers = 0;
    static bool ir_mid_on_line = false;
    static int ir_mid_triggers = 0;
    update_ir_states(ir_left_on_line, ir_left_triggers, ir_right_on_line, ir_right_triggers, ir_mid_on_line, ir_mid_triggers);
    int8_t counter = min(ir_left_triggers,min(ir_right_triggers,ir_mid_triggers));
    Serial.println(counter);
    Serial.print("Left:");
    Serial.print(ir_left());
    Serial.print(",");
    Serial.print("Mid:");
    Serial.print(ir_middle());
    Serial.print(",");
    Serial.print("Right:");
    Serial.println(ir_right());
    if (counter >= 1) {
        state = aligning_with_gap;
        time_box_reached = millis();
    }
}
void aligning_with_gap() {
    // if (imu.dist > 0.15) {
    if (millis() - time_box_reached > 600) {
        gyro_controller_on = false;
        stop();
        delay(1000);
        imu.reset_integrators();
        // imu.initialize();
        // imu.calibrate();
        // Conisder if we should add this. might be more robust without?
        switch (course) {
            case 0:
                turn_left(FORWARD,2*PI);
                break;
            case 1:
                turn_right(FORWARD,2*PI);
                break;
        }
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
        // Serial.println("Done!");
        stop();
        delay(500);
        imu.reset_integrators();
        // imu.initialize();
        // imu.calibrate();
        // wz = imu.gyroZ;
        // iwz = 0;
        // reset_ir_triggers();
        forward();
        gyro_controller_on = true;
        state = driving_through_gap;
        Serial.println("Entering driving_through_gap");
    }
}
int time_line_reached;
void driving_through_gap() {
    static bool ir_left_on_line = false;
    static int ir_left_triggers = 0;
    static bool ir_right_on_line = false;
    static int ir_right_triggers = 0;
    static bool ir_mid_on_line = false;
    static int ir_mid_triggers = 0;
    update_ir_states(ir_left_on_line, ir_left_triggers, ir_right_on_line, ir_right_triggers, ir_mid_on_line, ir_mid_triggers);
    int8_t counter = min(ir_left_triggers,min(ir_right_triggers,ir_mid_triggers));
    Serial.println(counter);
    if (counter >= 3) {
        gyro_controller_on = false;
        stop();
        delay(500);
        // imu.calibrate();
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
volatile int time_drive_to_contact_zone;
void turning_to_contact_zone() {
    bool turn_complete = false;
    Serial.println(imu.angZ);
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
        delay(500);
        // imu.calibrate();
        imu.reset_integrators();
        // wz = imu.gyroZ;
        // iwz = 0;
        forward();
        gyro_controller_on = true;
        state = driving_to_contact_zone;
        Serial.println("Entering driving_to_contact_zone");
        time_drive_to_contact_zone = millis();
        // delay(3000);
    }
}
void driving_to_contact_zone() {
    // if (abs(imu.dist_rate) < 0.05) {
        //TO BE CONTINUED
    if (millis() - time_drive_to_contact_zone > 3000) {
        gyro_controller_on = false;
        stop();
        delay(500);
        // imu.calibrate();
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
    }
}
volatile int time_drive_to_shooting_zone;
void turning_to_shooting_zone() {
    bool turn_complete = false;
    switch (course) {
        case 0:
            turn_complete = imu.angZ > PI/2;//100 * PI/180.;
            break;
        case 1:
            turn_complete = imu.angZ < -PI/2;//-100 * PI/180.;
            break;
    }
    if (turn_complete) {
        stop();
        delay(500);
        // imu.calibrate();
        imu.reset_integrators();
        // wz = imu.gyroZ;
        // iwz = 0;
        forward();
        gyro_controller_on = true;
        state = driving_to_shooting_zone;
        Serial.println("Entering driving_to_shooting_zone");
        time_drive_to_shooting_zone = millis();
        // delay(6000);
    }
}
void driving_to_shooting_zone() {
    if (millis() - time_drive_to_shooting_zone > 6000) {
        gyro_controller_on = false;
        stop();
    }
}


// float dw;
// float kp = 2.;
// float ki = 5.;
// float sign(float x) {x >= 0 ? 1. : -1.;}
// void controller() {    
//     wz = imu.gyroZ;
//     iwz = imu.angZ;
//     // Serial.println(iwz);
    
//     if (gyro_controller_on) {
//         // dw = -(kp*wz + ki*sign(iwz)*min(abs(iwz), 10))*BASE_WIDTH/WHEEL_RADIUS;
//         // dw = -(kp*wz + ki*iwz)*BASE_WIDTH/WHEEL_RADIUS;
//         // wr_cmd = dw + DEFAULT_MOTOR_SPEED;
//         // // analogWrite(EnA,(DEFAULT_MOTOR_SPEED + dw/2) * RPS_TO_ANALOG);
//         // // analogWrite(EnB,(DEFAULT_MOTOR_SPEED - dw/2) * RPS_TO_ANALOG);
//         // analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
//     }
//     // } else {
//     //     analogWrite(EnA,wr_cmd * RPS_TO_ANALOG);
//     // }
// }