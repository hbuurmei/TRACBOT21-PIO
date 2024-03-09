// CONFIGURATION
#define DO_CELEBRATE 0          // 1 for real run - Toggle celebration behaviors 
#define DO_ORIENT 1             // 1 for real run - Toggle initial IR beacon orientation
#define DO_TEST 0               // 0 for real run - Toggle a test state

// DEBUG FLAGS - ALL ZERO FOR REAL RUNS
#define DEBUG_GENERAL 0         // Enable for serial output and general debug flags
#define DEBUG_ORIENTING 0       // Enable to output IR beacon data in orientation phase
#define DEBUG_EXECUTE_TURN 0    // Enable to output kinematic data in execute_turn
#define DEBUG_DRIVING_TO_BOX 0  // 

// COURSE SELECTION
enum course_config {
    B,  // B --> 0
    A   // A --> 1
};
course_config course = B;

// INCLUDE LIBRARIES
#define USE_TIMER_1     true

#include <Arduino.h>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>
#include <motor_control/motor_control.cpp>
#include <servo/servo.cpp>
#include <button/button.cpp>
#include <sensors/ir_beacon.cpp>

// TUNING FLAGS
#define BEACON_OFFSET 66*PI/180     // Angle between beacon and desired starting position
#define SERVO_SWEEP 180             // Maximum sweep angle for swivel when searching for beacon
#define SWIVEL_INTERVAL 2           // Swivel angle increment when searching for beacon

#define CONTROLLER_INTERVAL 1000/CONTROLLER_SAMPLES_PER_SEC // Interval at which to call controller, ms.


// DEFINE SENSOR & ACTUATOR CLASSES
IMU imu;
IR_Beacon ir;
ServoDriver servos;

// DECLARE STATE FUNCTIONS
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
void reversing_to_contact_zone();
void drive_straight();
void pause_forward();   //pause between turning and driving straight (or really anything)
void pause_right();
void pause_left();
void idle();
void test_turns();
void do_nothing();

// INITIALIZE STATE
void (*state) (void) = start;  // usually we would use waiting_for_button
void (*state_after_pause) (void) = start; // next state to use after a pause

// INITIALIZE CONTROL FUNCTIONS
void controller();
bool execute_turn(float target, float speed = DEFAULT_MOTOR_SPEED, float tolerance = 2 * PI/180);//PI/64); //FLAG: changed speed from 1.7 to 2*PI //flag here
struct result {bool done_sweep; bool found_target; float angle_target_body;};
auto find_beacon_relative(bool rst = 0) -> result;
static volatile bool forward_controller = 0; 

// Variable to keep track of the time since the last state change
unsigned long time_state_change = 0;

/*
Setup Initialization Function
*/
void setup() {
    // Initialize Serial
    if (DEBUG_GENERAL){
        Serial.begin(9600);
        Serial.println("START");
    }
    // Configure Servos, set to defaults
    servos.initialize();

    // Stop any drive motor motion 
    stop();

    // Set up button
    // button_setup();

    // Initialize IMU, IR beacon sensor
    ir.initialize();
    imu.initialize();
    imu.calibrate();

    // Start controller timer
    ITimer1.init();
    ITimer1.setInterval(CONTROLLER_INTERVAL, controller);
}

/*
Main Loop Code
*/
void loop() {
    // Update IMU measurements
    imu.update_measurement();
    // Update IR beacon measurements
    ir.update();
    // Execute state function
    state();
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
Execute initial celebration. 
Transition to: orienting
*/
void start() {
    if (DO_CELEBRATE){
        servos.setSwivelAngle(SWIVEL_LEFT);
        delay(2000);
        servos.setSwivelAngle(SWIVEL_MIDDLE);
        delay(2000);
        servos.setSwivelAngle(SWIVEL_RIGHT);
        delay(2000);
        servos.setSwivelAngle(SWIVEL_MIDDLE);
        delay(1000);
    }
    
    // Reset integrator and move to orientation phase.
    imu.reset_integrators();
    if (DO_TEST){
        state = test_state_init;
        time_state_change = millis();
        if (DEBUG_GENERAL) {Serial.println("entering TEST");}
    }
    else if (DO_ORIENT){
        state = orienting;
        find_beacon_relative(1);
        time_state_change = millis();
        if (DEBUG_GENERAL) {Serial.println("entering orienting");}
    } else{
        // state = pause;
        // state_after_pause = driving_to_box;
        //don't think we need pause anymore
        state = pause_forward;
        state_after_pause = driving_to_box;
        Serial.println("entering DRIVING_TO_BOX");
        // forward();
        time_state_change = millis();
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
    // res stores the result of find_beacon_relative()
    static result res;
    // Check if we've completed a full sweep (initializes zero)
    if (res.done_sweep){
        // Check if we've found a target (currently always 1)
        if (res.found_target){
            // Set the swivel to the middle position
            servos.setSwivelAngle(SWIVEL_MIDDLE);
            // Execute turn to the position of the beacon, plus a fixed beacon offset
            bool turn_complete = execute_turn(res.angle_target_body + (course == A ? BEACON_OFFSET : -BEACON_OFFSET));
            // Once this turn is complete, stop and transition state
            if (turn_complete){
                stop();
                imu.reset_integrators();
                state_after_pause = driving_to_box;
                state = pause_forward;
                time_state_change = millis();
                if (DEBUG_GENERAL) {Serial.println("Entering driving to box");}
                reset_ir_triggers();
            }
        }
        else{
            // to implement: 120 deg turn
        }
    }
    // Continue if we haven't finished a full sweep
    else{
        res = find_beacon_relative();
    } 
}

/*
Robot is driving towards the edge of the box.
When any line sensor crosses a line, transition to aligning_with_gap. 
End State: The robot is moving forward() out of the box.
*/
void driving_to_box() {
    if (DEBUG_GENERAL){
        Serial.print(ir_left_triggers);
        Serial.print(ir_mid_triggers);
        Serial.println(ir_right_triggers);
    }
    /*
    
        TODO: This behavior resets the ki portion of the forward controller, but not the kp - may lead to unexpected behavior.

    */
    //FLAG test if this fixes the starting forward issue
    // forward(); // Not necessary, see line 202
    imu.reset_integrators();

    // If any IR sensor crosses a line (since the last state change), transition state. 
    if (ir_left_triggers || ir_right_triggers || ir_mid_triggers){
        state = aligning_with_gap;
        if (DEBUG_GENERAL) {Serial.println("Entering aligning_with_gap");}
        time_state_change = millis();
        // Note we continue moving forward() here- this will begin a timer.
        // forward(); // Not necessary, see 257
        imu.reset_integrators(); // Maybe not necessary
        forward_controller = 1;
    }
}

/*
Robot is driving out of the box.
When 1000ms elapses, the robot should stop and transition to turning_to_gap.
End State: The robot is stopped, aligned with the middle of the gap.
*/
void aligning_with_gap() {
    // Once 1000ms elapses, turn to face the gap.
    if (millis() - time_state_change > 1000) {
        stop();
        forward_controller = 0;
        imu.reset_integrators();
        state = turning_to_gap;
        time_state_change = millis();
        if (DEBUG_GENERAL) {Serial.println("Entering turning_to_gap");}
    }
}

/*
Robot is turning towards the gap.
Once the robot has turned 90degrees, robot stops and transitions to driving_through_gap.
*/
void turning_to_gap() {
    // Execute a 90deg turn, direction depending on course
    if (DEBUG_GENERAL) {
        Serial.print("AngZ: ");
        Serial.println(imu.angZ);
    }
    bool turn_complete = execute_turn(course == A ? -PI/2: PI/2); // Probably do something simpler

    // When turn finishes, transition state.
    if (turn_complete) {
        stop();
        imu.reset_integrators();
        state = pause_forward;
        state_after_pause = driving_through_gap;
        // state = driving_through_gap;
        // forward();
        time_state_change = millis();
        // forward_controller = 1;
        if (DEBUG_GENERAL) {Serial.println("Entering driving_through_gap");}
    }
}

/*
Robot is driving through the gap.
The robot here drives straight, ignoring any lines on the ground until a fixed time has elapsed. 
It then stops once any line is detected. The fixed time *should* allow it to travel through the gap 
and into the open part of the field before stopping on the line leading to the contact zone. 

Once line crossed, transition to turning_to_contact_zone.
*/
void driving_through_gap() {
    // Do nothing until 3s elapsed
    if (millis() - time_state_change >= 3000){ //FLAG this value needs tuning
        Serial.println("ir sensors should be triggering");
        if (DEBUG_GENERAL) {Serial.print("left: ");
                Serial.println(ir_left_triggers);
                Serial.print("middle: ");
                Serial.println(ir_mid_triggers);
                Serial.print("right ");
                Serial.println(ir_right_triggers);
        }
        if(ir_left_triggers || ir_mid_triggers || ir_right_triggers){
            Serial.println("line sensors active");
            forward_controller = 0;
            stop();
            imu.reset_integrators();
            
            state_after_pause = turning_to_contact_zone;
            switch (course) {
            case B:
                // turn_right(MIDDLE);
                state = pause_right;
                break;
            case A:
                // turn_left(MIDDLE);
                state = pause_left;
                break;
            }
            if (DEBUG_GENERAL) {Serial.println("Entering turning_to_contact_zone");}
            imu.reset_integrators();
            time_state_change = millis();
        }
    }
    else{
        reset_ir_triggers();
    }
}

/*
Robot has reached the line leading to the contact zone.
It then executes a 90deg turn, direction depending on course, before transitioning.
*/
void turning_to_contact_zone() {
    if (DEBUG_GENERAL) {
        Serial.print("AngZ: ");
        Serial.println(imu.angZ);
    }
    bool turn_complete = execute_turn(course==A ? PI/2 : -PI/2); // Quite problematic

    if (turn_complete) { 
        stop();
        imu.reset_integrators();
        state = pause_forward;
        state_after_pause = driving_to_contact_zone;
        // forward();
        time_state_change = millis();
        forward_controller = 1;
        // state = pause_forward;
        if (DEBUG_GENERAL) {Serial.println("Entering driving_to_contact_zone");}
        

    }
}

/*
Robot is facing the contact zone.
Drive forward for two seconds, then transition. 
*/
void driving_to_contact_zone() {  
    if(millis() > time_state_change + 2000){ 
        stop();
        imu.reset_integrators();
        forward_controller = 0;
        backward();
        time_state_change = millis();
        state = retreating_from_contact_zone; //testing up until hitting contact zone
        if (DEBUG_GENERAL) {Serial.println("Entering retreating_from_contact_zone");}
    }
}

/*
Robot has hit the contact zone.
Drive backwards for .35 seconds, then transition. 
*/
void retreating_from_contact_zone(){
    if(millis() > time_state_change + 350){ //reduced from 500 ms to 350 ms
        stop();
        imu.reset_integrators();
        switch (course) {
            case B:
                // turn_left(MIDDLE);
                state = pause_left;
                break;
            case A:
                // turn_right(MIDDLE);
                state = pause_right;
                break;
        }
        imu.reset_integrators();
        state_after_pause = turning_to_shooting_zone;
        time_state_change = millis();
        if (DEBUG_GENERAL) {Serial.println("Entering turning_to_shooting_zone");}
    }
}

/*
Robot has retreated from contact zone. 
Turn 90deg to face the shooting zone, then transition.
*/
void turning_to_shooting_zone() {
    bool turn_complete = execute_turn(course == A ? -PI/2 : PI/2);

    if (turn_complete) {
        stop();
        imu.reset_integrators();
        state = pause_forward;
        state_after_pause =  driving_to_shooting_zone;
        // forward();
        time_state_change = millis();
        imu.reset_integrators();
        forward_controller = 1;
        if (DEBUG_GENERAL) {Serial.println("Entering driving_to_shooting_zone");}
    }
}

/*
Robot is facing the shooting zone.
Drive forwards for 3s along the wall, then transition to turn swivel and release balls.
*/

/*
    TODO: Need to fix the following behavior. 
*/
void driving_to_shooting_zone() {
    if (millis() - time_state_change > 3000) {
        stop();
        forward_controller = 0;
        state = turning_swivel; //flag temporarily disabled
        backward(); //flag: remove /move to proper place later
        state = reversing_to_contact_zone;
        time_state_change = millis();
    }
}

/*
    TODO: Will require a backwards controller to make this work properly 
*/
//state to reverse and hit contact zone after shooting in case we missed it. Move to after shooting when fully integrating
void reversing_to_contact_zone(){
    if (millis() - time_state_change >= 5000){
        stop();
    }
}

/*
Robot is adjacent to the shooting zone. Turn the swivel to face the shooting zone. 
*/
void turning_swivel() {
    switch (course) {
        case B:
            servos.setSwivelAngle(SWIVEL_LEFT);
            break;
        case A:
            servos.setSwivelAngle(SWIVEL_RIGHT);
            break;
    }
    if (millis() - time_state_change > 3000) {
        state = dropping_balls;
        servos.openHatch();
    }
}

/*
Swivel is above the shooting zone. Drop balls and transition to celebrating
*/
void dropping_balls() { 
    servos.openHatch();
    delay(5000);
    state = celebrating;
}

/*
We're done. Celebrate. 
*/
void celebrating() {   
    servos.openHatch();
    delay(1000);
    servos.closeHatch();
    delay(1000);
    servos.openHatch();
    delay(1000);
    servos.closeHatch();
    delay(1000);
    // //end point / terminal state
}

/*
FLOW CONTROL AND MOTION FUNCTIONS
*/

void pause_forward(){
    if(millis()-time_state_change >= 1000){
        forward();
        forward_controller = 1;
        imu.reset_integrators();
        time_state_change = millis();
        state = state_after_pause;
        if (DEBUG_GENERAL) {Serial.println("Entering NEXT STAGE AFTER PAUSE");}
    }
}
void pause_right(){
    if(millis()-time_state_change >= 1000){
        forward_controller = 0;
        turn_right(MIDDLE);
        imu.reset_integrators();
        time_state_change = millis();
        state = state_after_pause;
        if (DEBUG_GENERAL) {Serial.println("Entering NEXT STAGE AFTER PAUSE");}
    }
}
void pause_left(){
    if(millis()-time_state_change >= 1000){
        forward_controller = 0;
        turn_left(MIDDLE);
        imu.reset_integrators();
        time_state_change = millis();
        state = state_after_pause;
        if (DEBUG_GENERAL) {Serial.println("Entering NEXT STAGE AFTER PAUSE");}
    }
}


/*
Forward Controller.
An control routine, executed on a hardware timer, that adjusts motor speed to maintain straight travel. 
*/
volatile float dw;
volatile int wr_cmd;
volatile int wl_cmd;
volatile float kp = 5.;
volatile float ki = 3.;
// float sign(float x) {x >= 0 ? 1. : -1.;}
void controller() {    
    // //Update sensors
    imu.update_integrator();  // IMU integrator
    update_ir_states();       //black tape ir sensors
    // // 
    // if(forward_controller){
    //     dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_HALF_WIDTH/WHEEL_RADIUS;
    //     // wr_cmd = max(DEFAULT_MOTOR_SPEED + dw, MIN_MOTOR_SPEED);
    //     // wl_cmd = max(DEFAULT_MOTOR_SPEED - dw, MIN_MOTOR_SPEED);
    //     wr_cmd = DEFAULT_MOTOR_SPEED + dw;
    //     wl_cmd = DEFAULT_MOTOR_SPEED - dw;
    //     analogWrite(EnA, wr_cmd * RPS_TO_ANALOG);
    //     analogWrite(EnB, wl_cmd * RPS_TO_ANALOG);
    //     //if needed, implement floor (reach min speed, fix it by turning one way til good)
    // }
}

/*
Turning.
Routine to execute a turn. Ensures the robot is stationary and is within correct tolerance for 
N_CONSECUTIVE_COMPLETES cycles before indicating that the turn is complete. 
*/
#define N_CONSECUTIVE_COMPLETES 5
bool execute_turn(float raw_target, float speed, float tolerance){
    static bool last_states[N_CONSECUTIVE_COMPLETES];
    static float last_raw_target;

    bool rst = 0;
    if (raw_target != last_raw_target){
        rst = 1;
    }
    last_raw_target = raw_target;
    float target = raw_target;  // Note we used to scale the target angle here to fix errors from ISR timer. Now should not need to.

    float error = abs(imu.angZ - target); 
    bool turn_complete = error < tolerance;

    int sum = 0;
    for (int ii=N_CONSECUTIVE_COMPLETES-1; ii>0; ii--){
        if(rst){
            last_states[ii] = 0;
        }
        else{
            last_states[ii] = last_states[ii-1];
        }
        
        sum += last_states[ii];
    }
    last_states[0] = turn_complete;
    sum += turn_complete;

    if (turn_complete){
        stop();
    }
    else{
        float speed_factor = map(error, 0, PI/2, PI*1.5, speed);
        if(imu.angZ > target){
            turn_right(MIDDLE, speed_factor);
        }
        else{
            turn_left(MIDDLE, speed_factor);
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

/*
Routine to find the angle of he beacon relative to the robot. 
Currently assumes beacon is within field of view of robot.
*/
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
            servos.setSwivelAngle(servo_angle_deg);
            servo_angle_deg += SWIVEL_INTERVAL;
            
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
                angle_target_body = (float(map(angle_target, 0, 180, -90, 90)) * PI/180);
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
Idle state. Unused.
*/
void idle(){

}

/*
TEST FUNCTIONS & STATES
*/


enum direction_config {
    R,  // right
    L   // left
};
direction_config direction = R;
void test_turns(){
    // bool turn_complete = execute_turn(course == A ? -PI/2: PI/2);
    bool turn_complete = execute_turn(direction == R ? -PI/2 : PI/2);
    // bool turn_complete = abs(imu.angZ) >= PI/2;
    // Serial.println(imu.angZ);
    if (turn_complete) {
        stop();
        Serial.println("turn complete");
        imu.reset_integrators();
        time_state_change = millis();
        state = do_nothing;
    }
}
void do_nothing(){
    Serial.println("move forward");
    forward();
    imu.reset_integrators();
    forward_controller = 1;

}

void test_state_init(){
    imu.reset_integrators();
    state = test_state;
    Serial.println("TEST START");
 
    forward();
    forward_controller = 1;
}

unsigned long last_turn_complete = 0;
void test_state(){
    servos.closeHatch();
    delay(1000);
    servos.openHatch();
    delay(1000);
    servos.closeHatch();
    delay(1000);

    servos.setSwivelAngle(SWIVEL_LEFT);
    delay(1000);
    servos.setSwivelAngle(SWIVEL_RIGHT);
    delay(1000);
    servos.setSwivelAngle(SWIVEL_MIDDLE);
    delay(1000);
    
}