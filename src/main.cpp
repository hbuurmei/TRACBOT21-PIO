
// CONFIGURATION -- ALL SHOULD BE 1 FOR REAL RUNS
#define DO_CELEBRATE 0
#define DO_ORIENT 0

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
void set_turn_target(float target);
bool execute_turn(float speed = 1.5*PI, float tolerance = PI/64);

static volatile bool forward_controller = 0; //idea: set to 1 whenever trying to drive straight, 0 otherwise, logic in controller

enum course_config {
    B,  // B --> 0
    A   // A --> 1
};
course_config course = A;

// Dictates behavior of the controller function.
enum control_state{
    CONTROL_OFF,
    CONTROL_FORWARD,
    CONTROL_TURN
};
control_state controller_mode = CONTROL_OFF;

float controller_turn_target = 0;

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
    hatch.write(0);

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
#define DEBUG_EXECUTE_TURN 1
void loop() {
    imu.update_measurement();
    state();
    timer.run();

    if (DEBUG_EXECUTE_TURN){
        Serial.print(">angZ:");
        Serial.println(imu.angZ);
        Serial.print(">controller_turn_target:");
        Serial.println(controller_turn_target);
    }
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
        hatch.write(HATCH_OPEN);
        delay(500);
        hatch.write(HATCH_CLOSED);
        delay(500);
        hatch.write(HATCH_OPEN);
        delay(500);
        hatch.write(HATCH_CLOSED);
        delay(5000);
    }
    
    //Load balls during the 5s delay

    // Reset integrator and move to orientation phase.
    imu.reset_integrators();
    if (DO_ORIENT){
        state = orienting;
        Serial.println("entering orienting");
    } else{
        state = driving_to_box;
        Serial.println("entering DRIVING_TO_BOX");
    }

    state = test_state_init;
}

/*
Orient the robot with the line trace exiting the starting box.
End State: The robot is moving forward() towards the edge of the box
*/
#define DEBUG_ORIENTING 1
void orienting(){
    /*
    - sweep 180 degrees with servo, measure maximum value & angle 
    - if value not higher than some threshold (100 ?), turn 120 and repeat
    - else, turn to angle of beacon, offset by value based on course A or course B
    */
    static int servo_angle_deg = 0;
    static int angle_target = 0;
    static float angle_target_body = 0;

    static int max_ir_reading = 0;

    static float beacon_offset = 66*PI/180;
    if (course == B){
        beacon_offset *= -1;
    }

    #define LEN_CONV 15
    static int conv_weights[LEN_CONV] = {-10, -10, 1, 2, 2, 6, 6, 10, 6, 6, 2, 2, 1, -10, -10};
    // static int conv_weights[LEN_CONV] = {0, 0, 1, 2, 2, 6, 6, 6, 6, 6, 2, 2, 1, 0, 0};
    // static int conv_weights[LEN_CONV] = {5,5,5,5,5};
    static int last_angles[LEN_CONV];
    static int ir_readings[LEN_CONV];

    static unsigned long last_servo_move = 0;
    
    if (servo_angle_deg <= 130){
        if (millis()>last_servo_move+250){
            last_servo_move = millis();
            swivel.write(servo_angle_deg);
            servo_angle_deg += swivel_interval;
            
            // top hat convolve of last LEN_CONV values
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
                angle_target_body = (float(map(angle_target, 0, 130, -90, 90)) * PI/180 + beacon_offset);
                set_turn_target(angle_target_body);
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
    }    
    else{
        swivel.write(60);
        bool turn_complete = execute_turn();
        if (turn_complete) { 
            stop();
            // imu.reset_integrators();
            // state = ;
        }
    } //end else
}

/*
Robot is driving towards the edge of the box.
When any line sensor crosses a line, transition to aligning_with_gap. 
End State: The robot is moving forward() out of the box.
*/
void driving_to_box() {
    forward(); // TODO: note this won't work with current controller architecture;
    forward_controller = 1;

    if (ir_left_triggers || ir_right_triggers || ir_mid_triggers){
        state = aligning_with_gap;
        Serial.println("Entering aligning_with_gap");
        time_state_change = millis();
    }
}

/*
Robot is driving out of the box.
When 1000ms elapses, the robot should stop and transition to turning_to_gap.
End State: The robot is stopped, aligned with the middle of the gap.
*/
void aligning_with_gap() {
    forward();
    forward_controller = 1;

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
    if (course == A){
        set_turn_target(-PI/2); // 90deg RIGHT turn 
    } else{
        set_turn_target(PI/2); // 90deg LEFT turn
    }

    bool turn_complete = execute_turn();

    if (turn_complete) {
        stop();
        imu.reset_integrators();
        time_state_change = millis();
        state = driving_through_gap;
        Serial.println("Entering driving_through_gap");
    }
}

/*
Robot is driving through the gap.
Robot will not pay attention to any lines until 3.5s have elapsed. Once 3.5s have elapsed, begin looking 
for lines. Once line crossed, transition to turning_to_contact_zone.
*/
void driving_through_gap() {
    forward();
    forward_controller = 1;
    // Do nothing until 3.5s elapsed
    if (millis() < time_state_change + 3500){
        reset_ir_triggers();
        return;
    }

    if(ir_left_triggers || ir_mid_triggers || ir_right_triggers){
        stop();
        forward_controller = 0;

        switch (course) {
            case B:
                turn_right(MIDDLE,1.5*PI);
                break;
            case A:
                turn_left(MIDDLE,1.5*PI);
                break;
        }
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
    if (course == A){
        set_turn_target(-PI/2); // 90deg RIGHT turn 
    } else{
        set_turn_target(PI/2); // 90deg LEFT turn
    }

    bool turn_complete = execute_turn();

    if (turn_complete) { 
        stop();
        imu.reset_integrators();
        //line_follow();  //flag -- line follow preliminary attempt

        state = driving_to_contact_zone;
        Serial.println("Entering driving_to_contact_zone");
        time_state_change = millis();
    }
}

/*

*/
void driving_to_contact_zone() {   //FLAG pretty good til here, line tracking worked surprisingly well for course A
    forward_controller = 1;
    forward();
    if(millis() > time_state_change + 1000){
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
    if (course == A){
        set_turn_target(-PI/2); // 90deg RIGHT turn 
    } else{
        set_turn_target(PI/2); // 90deg LEFT turn
    }
    bool turn_complete = execute_turn();

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

void dropping_balls() { //temporary -- change when we add ramp climbing
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

float dw;
float wr_cmd;
float wl_cmd;
float kp = 2.;
float ki = 5.;
// float sign(float x) {x >= 0 ? 1. : -1.;}

#define DEBUG_CONTROLLER 0
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

#define TURN_ADJUSTMENT_FACTOR 1.57;
void set_turn_target(float target){
    controller_turn_target = target / TURN_ADJUSTMENT_FACTOR;
}

// #define TURN_ADJUSTMENT_FACTOR 1
bool execute_turn(float speed, float tolerance){
    bool turn_complete = abs(imu.angZ - controller_turn_target) < tolerance / TURN_ADJUSTMENT_FACTOR;
    if (turn_complete){
        stop();
    }
    else{
        if(imu.angZ > controller_turn_target){
            turn_right(MIDDLE, speed);
        }
        else{
            turn_left(MIDDLE, speed);
        }
    }
    return turn_complete;
}


void test_state_init(){
    set_turn_target(PI/2);
    imu.calibrate();
    imu.reset_integrators();
    state = test_state;
}

unsigned long last_turn_complete = 0;
void test_state(){
    static int ii = 0;
    bool turn_complete = execute_turn();
    if (turn_complete && (millis() - last_turn_complete > 5000)){
        set_turn_target(PI/4*(ii%8));
        ii += 1;
        last_turn_complete = millis();
    }

    Serial.print(">turn_complete:");
    Serial.println(turn_complete);
}
