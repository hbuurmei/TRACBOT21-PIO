#define CONTROLLER_FREQ 25.
// #define USE_TIMER_0 true
#define USE_TIMER_1     true
// #define USE_TIMER_2     true

#include <Arduino.h>
#include <sensors/ir_line.cpp>
// #include <sensors/imu.cpp>
// #include <TimerInterrupt.h>
#include <motor_control/motor_control.cpp>
#include <Metro.h>
#include <servo/servo.cpp>
#include <button/button.cpp>
#include <sensors/ir_beacon.cpp>

#define RIGHT_TURN_90_DEG 70*PI/180
#define LEFT_TURN_90_DEG -100*PI/180

IMU imu;
IR_Beacon ir;
// State functions
void waiting_for_button();
void start();
void orienting();
void driving_to_box();
void aligning_with_gap();
void turning_to_gap();
void driving_through_gap();
void turning_to_contact_zone();
void driving_to_contact_zone();
void turning_to_shooting_zone();
void driving_to_shooting_zone();
void turning_swivel();
void dropping_balls();
void celebrating();
void (*state) (void) = waiting_for_button;

// Control functions
void controller();
static volatile float wz;
static volatile float iwz;
static bool gyro_controller_on = false;
enum course_config {
    B,  // B --> 0
    A   // A --> 1
};
course_config course = A;

void setup() {
    Serial.begin(9600);
    stop();
    swivel.attach(SWIVEL_SERVO_PIN);
    hatch.attach(HATCH_SERVO_PIN);
    button_setup();
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

void waiting_for_button() { // button for course selection
    uint16_t t = millis();
    while (millis() - t < 10000 && course == A) {
        bool button_read = digitalRead(buttonPin);  // 0 if button is pressed, 1 if not
        course = button_read ? A : B;
    }
    state = start;
}
void start() {
    // if (Serial.available()) {
    //wave hatch servo at start, before loading balls, to meet performance requirement 2
    delay(300);
    hatch.write(HATCH_OPEN);
    delay(300);
    hatch.write(HATCH_CLOSED);
    delay(300);
    hatch.write(HATCH_OPEN);
    delay(300);
    hatch.write(HATCH_CLOSED);
    delay(5000);
    //Load balls during the 5s delay
    // IR sensor reorientation here (or as it's own state, then change the state transitions)
    imu.initialize();
    imu.calibrate();
    imu.reset_integrators();
    gyro_controller_on = true;

    ITimer1.init();
    ITimer1.setFrequency(CONTROLLER_FREQ, controller);

    ir.initialize();

    turn_left(MIDDLE, 1.7*PI);
    state = orienting;
}

void orienting(){
    static float angle_target = 0;
    static bool found_target = 0;
    static float orientation_angle = 66*PI/180; //may need refining
    ir.update(imu.angZ);
    if (!found_target){
        if (imu.angZ > 2*PI){
            stop();
            angle_target = ir.angle;
            found_target = 1;
        }
    } // end if(!found_target)
    else{
        // turn to angle_target plus (or minus) 66 degrees
        bool turn_complete = false;
        switch (course) {
            case B:
                turn_right(MIDDLE, 1.7*PI);
                turn_complete = imu.angZ < angle_target - orientation_angle;
                break;
            case A:
                turn_left(MIDDLE, 1.7*PI);
                turn_complete = imu.angZ > angle_target + orientation_angle;
                break;
        } // end switch(course)
        if (turn_complete) { 
            stop();
            delay(1000);
            imu.calibrate();
            wz = imu.gyroZ;
            iwz = 0;
            forward();
            imu.reset_integrators();
            gyro_controller_on = true;
            state = driving_to_box;
            Serial.println("Entering driving_to_box");
            delay(3000);
        } // end if turn_complete
    } //end else
}

volatile unsigned long time_box_reached;
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
            case B:
                turn_left(FORWARD,2*PI);
                break;
            case A:
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
        case B:
            turn_complete = angZ > LEFT_TURN_90_DEG;
            break;
        case A:
            turn_complete = angZ < RIGHT_TURN_90_DEG;
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
            case B:
                turn_right(FORWARD,2*PI);
                break;
            case A:
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
        case B:
            turn_complete = imu.angZ < RIGHT_TURN_90_DEG;
            break;
        case A:
            turn_complete = imu.angZ > LEFT_TURN_90_DEG;
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
            case B:
                turn_left(BACKWARD,2*PI);
                break;
            case A:
                turn_right(BACKWARD,2*PI);
                break;
        }
        state = turning_to_shooting_zone;
        Serial.println("Entering turning_to_shooting_zone");
    // }
}
volatile unsigned long time_driving_to_shooting_zone;
void turning_to_shooting_zone() {
    bool turn_complete = false;
    switch (course) {
        case B:
            turn_complete = imu.angZ > LEFT_TURN_90_DEG;
            break;
        case A:
            turn_complete = imu.angZ < RIGHT_TURN_90_DEG;
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
        time_driving_to_shooting_zone = millis();
        // delay(6000);

    }
}
volatile unsigned long time_swivel_turn;
void driving_to_shooting_zone() {
    if (millis() - time_driving_to_shooting_zone > 6000) {
        stop();
        delay(3000);
        gyro_controller_on = false;
        switch (course) {
            case B:
                swivel.write(SWIVEL_RIGHT_ANGLE);
                break;
            case A:
                swivel.write(SWIVEL_LEFT_ANGLE);
                break;
        }
        time_swivel_turn = millis();
        state = turning_swivel;
    }
}
void turning_swivel() {
    if (millis() - time_swivel_turn > 3000) {
        state = dropping_balls;
        hatch.write(HATCH_OPEN);
    }
}
void dropping_balls() { //temporary -- change when we add ramp climbing
    delay(2000);
    state = celebrating;
    hatch.write(HATCH_CLOSED);
}

void celebrating() {   
    delay(300);
    hatch.write(HATCH_OPEN);
    delay(300);
    hatch.write(HATCH_CLOSED);
    delay(300);
    hatch.write(HATCH_OPEN);
    delay(300);
    hatch.write(HATCH_CLOSED);
    //end point / terminal state
}

// float dw;
// float kp = 2.;
// float ki = 5.;
// float sign(float x) {x >= 0 ? 1. : -1.;}
// void controller() {    
//     wz = imu.gyroZ;
//     iwz = imu.angZ;
//     imu.update_integrator();
//     update_ir_states();
// }
