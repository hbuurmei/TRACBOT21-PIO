#define CONTROLLER_FREQ 25.
#define LEFT_90_TURN 85*PI/180
#define RIGHT_90_TURN 82.5*PI/180
#define swivel_interval 2    //2 degree turn intervals for now

#include <Arduino.h>
#include <sensors/ir_line.cpp>
#include <sensors/imu.cpp>
#include <ISR_Timer.h>
#include <motor_control/motor_control.cpp>
#include <servo/servo.cpp>
#include <button/button.cpp>
#include <sensors/ir_beacon.cpp>
ISR_Timer timer;
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

// Initialize state
void (*state) (void) = start;  // usually we would use waiting_for_button

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
    Serial.println("START");
    stop();
    ir.initialize();
    imu.initialize();
    imu.calibrate();
    swivel.attach(SWIVEL_SERVO_PIN);
    // Move swivel to neutral position
    swivel.write(0);
    hatch.attach(HATCH_SERVO_PIN);
    button_setup();
    timer.init();
    timer.setInterval(40, controller);
}

volatile float wr_cmd;
void loop() {
    imu.update_measurement();
    state();
    timer.run();
}

void waiting_for_button() { // button for course selection
    // if button is pressed, choose course B, else choose course A
    uint16_t t = millis();
    while (millis() - t < 10000 && course == A) {
        bool button_read = digitalRead(buttonPin);  // 0 if button is pressed, 1 if not
        course = button_read ? A : B;
    }
    state = start;
}
void start() {
    // Wave hatch servo at start, before loading balls, to meet performance requirement 2
    // hatch.write(HATCH_OPEN);
    // delay(500);
    // hatch.write(HATCH_CLOSED);
    // delay(500);
    // hatch.write(HATCH_OPEN);
    // delay(500);
    // hatch.write(HATCH_CLOSED);
    // delay(5000);
    //Load balls during the 5s delay
    // IR sensor reorientation here (or as it's own state, then change the state transitions)
    imu.initialize();
    imu.calibrate();
    imu.reset_integrators();
    gyro_controller_on = true;

    // ITimer1.init();
    // ITimer1.setFrequency(CONTROLLER_FREQ, controller);

    // turn_left(MIDDLE, 1.5*PI);
    stop();
    state = orienting;
}

void orienting(){
    static float angle = 0;
    static long angle_target = 0;
    static float angle_target_body = 0;
    // static bool found_target = 0;
    static float orientation_angle = 66*PI/180; //FLAG: may need refining
    static float max_ir_reading = 0;
    static bool turn_complete = 0;
    static unsigned long last_servo_move = 0;
    
    if (angle <= 130){
        if (millis()>last_servo_move+250){
            last_servo_move = millis();
            swivel.write(angle);
            angle += swivel_interval;
            Serial.print("Angle: ");
            Serial.print(angle);
            Serial.print(" Reading: ");
            Serial.println(ir.value);
            if (ir.value > max_ir_reading){
                max_ir_reading = ir.value;
                angle_target = angle;
                angle_target_body = float(map(angle_target, 0, 130, -90, 90)) * PI/180;
            }
        }
        imu.reset_integrators();
    }    
    else if(!turn_complete){
        Serial.print("Target angle: ");
        Serial.print(angle_target);
        Serial.print(" Mapped Angle: ");
        Serial.print(angle_target_body);
        Serial.print(" Current Angle: ");
        Serial.println(imu.angZ);
        swivel.write(60);

        // switch (course) {
        //     case B:
        //         turn_left(MIDDLE, 1.7*PI);
        //         turn_complete = imu.angZ > angle_target_body; // - orientation_angle + 2 * PI;
        //         break;
        //     case A:
        //         turn_left(MIDDLE, 1.7*PI);
        //         turn_complete = imu.angZ > angle_target_body; // + orientation_angle;
        //         break;
        //} // end switch(course)
        
        // turn_complete = abs(angle_target_body - imu.angZ) < PI/16;
        turn_complete = abs(imu.angZ) >= abs(angle_target_body);

        if (turn_complete) { 
            stop();
            Serial.println("DONE!");
            imu.reset_integrators();
        } else{
            if (angle_target_body < imu.angZ){
                turn_right(MIDDLE, 1.7*PI);
            }
            else{
                turn_left(MIDDLE, 1.7*PI);
            }
        }
        // turn_complete = false;  //now turn the offset
        // switch (course) {
        //     case B:
        //         turn_right(MIDDLE, 1.7*PI);
        //         turn_complete = abs(imu.angZ) > orientation_angle;
        //         break;
        //     case A:
        //         turn_left(MIDDLE, 1.7*PI);
        //         turn_complete = abs(imu.angZ) > orientation_angle;
        //         break;
        // } // end switch(course)
        // if (turn_complete) { 
        //     stop();
        //     delay(1000);
        //     imu.calibrate();
        //     wz = imu.gyroZ;
        //     iwz = 0;
        //     forward();
        //     imu.reset_integrators();
        //     gyro_controller_on = true;
        //     timer.init();
        //     timer.setInterval(40,controller);
        //     state = driving_to_box;
        //     Serial.println("Entering driving_to_box");
        //     delay(1000);
        // } // end if turn_complete
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
        Serial.println("Entering aligning_with_gap");
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
            turn_complete = angZ >= RIGHT_90_TURN;
            break;
        case A:
            turn_complete = angZ <= -LEFT_90_TURN;
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
            turn_complete = imu.angZ < -PI/2;
            break;
        case A:
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
            turn_complete = imu.angZ > PI/2;
            break;
        case A:
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
                // swivel.write(SWIVEL_RIGHT_ANGLE);
                break;
            case A:
                // swivel.write(SWIVEL_LEFT_ANGLE);
                break;
        }
        time_swivel_turn = millis();
        state = turning_swivel;
    }
}
void turning_swivel() {
    if (millis() - time_swivel_turn > 3000) {
        state = dropping_balls;
        // hatch.write(HATCH_OPEN);
    }
}
void dropping_balls() { //temporary -- change when we add ramp climbing
    delay(2000);
    state = celebrating;
    // hatch.write(HATCH_CLOSED);
}

void celebrating() {   
    // delay(300);
    // hatch.write(HATCH_OPEN);
    // delay(300);
    // hatch.write(HATCH_CLOSED);
    // delay(300);
    // hatch.write(HATCH_OPEN);
    // delay(300);
    // hatch.write(HATCH_CLOSED);
    //end point / terminal state
}

// float dw;
// float kp = 2.;
// float ki = 5.;
// float sign(float x) {x >= 0 ? 1. : -1.;}
void controller() {    
    wz = imu.gyroZ;
    iwz = imu.angZ;
    imu.update_integrator();
    update_ir_states();
    ir.update(0);
}
