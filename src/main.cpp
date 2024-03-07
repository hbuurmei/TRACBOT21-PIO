
// CONFIGURATION -- ALL SHOULD BE 1 FOR REAL RUNS
#define DO_CELEBRATE 0  // 1 for real run
#define DO_ORIENT 1     // 1 for real run
#define DO_TEST 0       // 0 for real run

#define DEBUG_ORIENTING 0
#define DEBUG_CONTROLLER 0
#define DEBUG_EXECUTE_TURN 0

#define LEFT_90_TURN 85*PI/180  //verified as 85
#define RIGHT_90_TURN 82.5*PI/180   //verified as 82.5
#define swivel_interval 2    //2 degree turn intervals for now

#include <Arduino.h>
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
void retreating_from_contact_zone();
void turning_to_shooting_zone();
void driving_to_shooting_zone();
void turning_swivel();
void dropping_balls();
void celebrating();

void test_state_init();
void test_state();

// Initialize state
void (*state) (void) = start;  // usually we would use waiting_for_button

// Control functions
void controller();
bool execute_turn(float target, float speed = 1.5*PI, float tolerance = PI/64);

struct result {bool done_sweep; bool found_target; float angle_target_body;};
auto find_beacon_relative(bool rst = 0) -> result;

static volatile bool forward_controller = 0; //idea: set to 1 whenever trying to drive straight, 0 otherwise, logic in controller

enum course_config {
    B,  // B --> 0
    A   // A --> 1
};
course_config course = B;

// Dictates behavior of the controller function.
enum control_state{
    CONTROL_OFF,
    CONTROL_FORWARD,
    CONTROL_TURN
};
control_state controller_mode = CONTROL_OFF;

unsigned long time_state_change = 0;

/*
Setup Initialization Function
*/
void setup() {
    // Initialize Serial
    Serial.begin(9600);
    Serial.println("START");

    // Configure Servos, set to defaults
    swivel.attach(SWIVEL_SERVO_PIN);
    swivel.write(60);

    hatch.attach(HATCH_SERVO_PIN);
    hatch.write(HATCH_CLOSED);

    // Stop any drive motor motion 
    stop();

    // Set up button
    button_setup();

    // Initialize IMU, IR beacon sensor
    ir.initialize();

    imu.initialize();
    imu.calibrate();
    
    // Begin controller timer
    timer.init();
    timer.setInterval(40, controller);
}

/*
Main Loop Code
*/
void loop() {
    imu.update_measurement();
    state();
    timer.run();
}

/*
STATE FUNCTIONS
*/

/*
waiting_for_button()
Default start state. If the button is pressed, set course to B, otherwise set course to A.
Transition to: start
*/
void waiting_for_button() { // button for course selection
    // if button is pressed, choose course B, else choose course A
    unsigned long t = millis();
    while (millis() - t < 10000 && course == A) {
        bool button_read = digitalRead(buttonPin);  // 0 if button is pressed, 1 if not
        course = button_read ? A : B;
    }
    state = start;
}

/*
start()
Execute initial celebration. Wave the hatch servo. 
Transition to: orienting
*/
void start() {
    // Wave hatch servo at start, before loading balls, to meet performance requirement 2
    if (DO_CELEBRATE){
        Serial.println("HATCH OPEN");
        hatch.write(HATCH_OPEN);
        delay(500);
        Serial.println("HATCH CLOSED");
        hatch.write(HATCH_CLOSED);
        delay(500);
        Serial.println("HATCH OPEN");
        hatch.write(HATCH_OPEN);
        delay(500);
        Serial.println("HATCH CLOSED");
        hatch.write(HATCH_CLOSED);
        delay(5000);
    }
    
    //Load balls during the 5s delay

    // Reset integrator and move to orientation phase.
    imu.reset_integrators();
    if (DO_TEST){
        state = test_state_init;
        Serial.println("entering TEST");
    }
    else if (DO_ORIENT){
        state = orienting;
        find_beacon_relative(1);
        Serial.println("entering orienting");
    } else{
        state = driving_to_box;
        Serial.println("entering DRIVING_TO_BOX");
        forward();
        forward_controller = 1;
    }
    
}

/*
Orient the robot with the line trace exiting the starting box.
End State: The robot is moving forward() towards the edge of the box
*/
void orienting(){
    /*
    - sweep 180 degrees with servo, measure maximum value & angle 
    - if value not higher than some threshold (100 ?), turn 120 and repeat
    - else, turn to angle of beacon, offset by value based on course A or course B
    */
    static result res;
    static float beacon_offset = 66*PI/180;
    if (course == B){
        beacon_offset *= -1;
    }

    if (res.done_sweep){
        if (res.found_target){
            swivel.write(60);
            bool turn_complete = execute_turn(res.angle_target_body + beacon_offset);
            if (turn_complete){
                stop();
                imu.reset_integrators();
                state = driving_to_box;
                time_state_change = millis();
                forward();
                forward_controller = 1;
                Serial.println("Entering driving to box");
                reset_ir_triggers();
            }
        }
        else{
            // implement 120 deg turn, continue orienting 
        }
    }
    else{
        res = find_beacon_relative();
    } //end else
}

#define SERVO_SWEEP 130
auto find_beacon_relative(bool rst) -> result{
    static int servo_angle_deg = 0; // current angle of servo
    static int angle_target = 0; // estimated servo angle to target
    static float angle_target_body = 0; // angle to target in body frame

    static int max_ir_reading = 0;

    #define LEN_CONV 15
    static int conv_weights[LEN_CONV] = {-10, -10, 1, 2, 2, 6, 6, 10, 6, 6, 2, 2, 1, -10, -10};
    // static int conv_weights[LEN_CONV] = {0, 0, 1, 2, 2, 6, 6, 6, 6, 6, 2, 2, 1, 0, 0};
    // static int conv_weights[LEN_CONV] = {5,5,5,5,5};
    static int last_angles[LEN_CONV];
    static int ir_readings[LEN_CONV];

    static unsigned long last_servo_move = 0;

    if (rst){
        servo_angle_deg = 0;
        angle_target = 0;
        angle_target_body = 0;
        for (int ii = 0; ii < LEN_CONV; ii++){
            last_angles[ii] = 0;
            ir_readings[ii] = 0;
        }
    }

    if (servo_angle_deg <= SERVO_SWEEP){
        if (millis()>last_servo_move+250){
            last_servo_move = millis();
            swivel.write(servo_angle_deg);
            servo_angle_deg += swivel_interval;
            
            int this_sum = 0;
            for (int ii=LEN_CONV-1; ii>0; ii--){
                ir_readings[ii] = ir_readings[ii-1];
                last_angles[ii] = last_angles[ii-1];
                this_sum += ir_readings[ii] * conv_weights[ii];
            }
            ir_readings[0] = ir.value;
            last_angles[0] = servo_angle_deg;
            this_sum += ir_readings[0] * conv_weights[0];
            this_sum /= LEN_CONV;

            if (this_sum > max_ir_reading){
                max_ir_reading = this_sum;
                angle_target = last_angles[LEN_CONV/2]; // take middle of conv.
                angle_target_body = (float(map(angle_target, 0, 130, -90, 90)) * PI/180);
            }
            if (DEBUG_ORIENTING){
                Serial.print(">ServoAngle: ");
                Serial.println(servo_angle_deg);
                Serial.print(">IRReading: ");
                Serial.println(ir.value);
                Serial.print(">ConvValue: ");
                Serial.println(this_sum);
                Serial.print(">TargetAngle: ");
                Serial.println(angle_target);
            }
        }
    } else{
        return result{1, 1, angle_target_body};
    }
    return result{0, 0, 0};
}

/*
Robot is driving towards the edge of the box.
When any line sensor crosses a line, transition to aligning_with_gap. 
End State: The robot is moving forward() out of the box.
*/
void driving_to_box() {
    Serial.print(ir_left_triggers);
    Serial.print(ir_mid_triggers);
    Serial.println(ir_right_triggers);

    if (ir_left_triggers || ir_right_triggers || ir_mid_triggers){
        state = aligning_with_gap;
        Serial.println("Entering aligning_with_gap");
        time_state_change = millis();
        forward();
        forward_controller = 1;
    }
}

/*
Robot is driving out of the box.
When 1000ms elapses, the robot should stop and transition to turning_to_gap.
End State: The robot is stopped, aligned with the middle of the gap.
*/
void aligning_with_gap() {
    if (millis() - time_state_change > 1000) {
        stop();
        forward_controller = 0;
        imu.reset_integrators();
        state = turning_to_gap;
        Serial.println("Entering turning_to_gap");
    }
}

/*
Robot is turning towards the gap.
Once the robot has turned 90degrees, robot stops and transitions to driving_through_gap.
*/
void turning_to_gap() {
    bool turn_complete = execute_turn(course == A ? -PI/2 + PI/16 : PI/2 - PI/16);

    if (turn_complete) {
        stop();
        imu.reset_integrators();
        time_state_change = millis();
        state = driving_through_gap;
        Serial.println("Entering driving_through_gap");

        forward();
        forward_controller = 1;
    }
}

/*
Robot is driving through the gap.
Robot will not pay attention to any lines until 3.5s have elapsed. Once 3.5s have elapsed, begin looking 
for lines. Once line crossed, transition to turning_to_contact_zone.
*/
void driving_through_gap() {
    // Do nothing until 3.5s elapsed
    if (millis() < time_state_change + 3000){
        reset_ir_triggers();
        return;
    }

    if(ir_left_triggers || ir_mid_triggers || ir_right_triggers){
        stop();
        forward_controller = 0;
        imu.reset_integrators();

        state = turning_to_contact_zone;
        Serial.println("Entering turning_to_contact_zone");
        time_state_change = millis();
    }
}

/*
Turn towards contact zone
*/
void turning_to_contact_zone() {
    bool turn_complete = execute_turn(course==A ? PI/2 : -PI/2);

    if (turn_complete) { 
        stop();
        imu.reset_integrators();

        state = driving_to_contact_zone;
        Serial.println("Entering driving_to_contact_zone");
        time_state_change = millis();

        forward_controller = 1;
        forward();
    }
}

/*

*/
void driving_to_contact_zone() {  
    if(millis() > time_state_change + 5000){
        stop();
        forward_controller = 0;
        time_state_change = millis();
        state = retreating_from_contact_zone;
        Serial.println("Entering retreating_from_contact_zone");
    }
}

void retreating_from_contact_zone(){
    backward();
    if(millis() > time_state_change + 500){
        stop();
        time_state_change = millis();
        state = turning_to_shooting_zone;
        Serial.println("Entering turning_to_shooting_zone");
    }
}

void turning_to_shooting_zone() {
    bool turn_complete = execute_turn(course == A ? -PI/2 : PI/2);

    if (turn_complete) {
        stop();
        imu.reset_integrators();
        state = driving_to_shooting_zone;
        Serial.println("Entering driving_to_shooting_zone");
        time_state_change = millis();
    }
}

void driving_to_shooting_zone() {
    forward();
    forward_controller = 1;

    if (millis() - time_state_change > 6000) {
        stop();
        forward_controller = 0;
        state = turning_swivel;
        time_state_change = millis();
    }
}

void turning_swivel() {
    switch (course) {
        case B:
            swivel.write(SWIVEL_RIGHT_ANGLE);
            break;
        case A:
            swivel.write(SWIVEL_LEFT_ANGLE);
            break;
    }
    if (millis() - time_state_change > 3000) {
        state = dropping_balls;
        hatch.write(HATCH_OPEN);
    }
}

void dropping_balls() { 
    hatch.write(HATCH_OPEN);
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

volatile float dw;
volatile int wr_cmd;
volatile int wl_cmd;
volatile float kp = 2.;
volatile float ki = 5.;
// float sign(float x) {x >= 0 ? 1. : -1.;}

void controller() {    
    // Update sensors
    imu.update_integrator();  // IMU integrator
    update_ir_states();       //black tape ir sensors
    ir.update();              //beacon IR

    // 
    if(forward_controller){
        dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_WIDTH/WHEEL_RADIUS;
        wr_cmd = DEFAULT_MOTOR_SPEED + dw/2;
        wl_cmd = DEFAULT_MOTOR_SPEED - dw/2;
        analogWrite(EnA, wr_cmd * RPS_TO_ANALOG);
        analogWrite(EnB, wl_cmd * RPS_TO_ANALOG);
    }
}

#define TURN_ADJUSTMENT_FACTOR 1.57
#define N_CONSECUTIVE_COMPLETES 5
// #define TURN_ADJUSTMENT_FACTOR 1
bool execute_turn(float raw_target, float speed, float tolerance){
    static bool last_states[N_CONSECUTIVE_COMPLETES];

    float target = raw_target / TURN_ADJUSTMENT_FACTOR; 
    bool turn_complete = abs(imu.angZ - target) < tolerance / TURN_ADJUSTMENT_FACTOR;

    int sum = 0;
    for (int ii=N_CONSECUTIVE_COMPLETES-1; ii>0; ii--){
        last_states[ii] = last_states[ii-1];
        sum += last_states[ii];
    }
    last_states[0] = turn_complete;
    sum += turn_complete;

    if (turn_complete){
        stop();
    }
    else{
        if(imu.angZ > target){
            turn_right(MIDDLE, speed);
        }
        else{
            turn_left(MIDDLE, speed);
        }
    }

    if (DEBUG_EXECUTE_TURN){
        Serial.print(">angZ:");
        Serial.println(imu.angZ);
        Serial.print(">raw_target:");
        Serial.println(raw_target);
        Serial.print(">adj_target:");
        Serial.println(target);
        Serial.print(">cons_comp:");
        Serial.println(sum);
    }
    return sum == N_CONSECUTIVE_COMPLETES;
}

void test_state_init(){
    imu.calibrate();
    imu.reset_integrators();
    state = test_state;
}

unsigned long last_turn_complete = 0;
void test_state(){
    // static int ii = 0;
    static float turn_target = PI/4;
    static bool turn_complete = 0;

    turn_complete = execute_turn(turn_target);

    Serial.print(">turn_complete:");
    Serial.println(turn_complete);
}
