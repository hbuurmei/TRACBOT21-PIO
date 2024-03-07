// #define CONTROLLER_FREQ 25.  //unused
#define LEFT_90_TURN 85*PI/180  //verified as 85
#define RIGHT_90_TURN 82.5*PI/180   //verified as 82.5
#define swivel_interval 2    //2 degree turn intervals for now

#include <Arduino.h>
// #include <sensors/ir_line.cpp>   //commented out bc now included in motor_control for line follow implementation
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
void test_turn();

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

static bool forward_controller = 0; //idea: set to 1 whenever trying to drive straight, 0 otherwise, logic in controller
enum course_config {
    B,  // B --> 0
    A   // A --> 1
};
course_config course = A;

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

    // Set up buttons
    button_setup();

    // Initialize IMU, IR beacon sensor
    ir.initialize();
    imu.initialize();
    imu.calibrate();
    
    // Begin intervals
    timer.init();
    timer.setInterval(40, controller);
}

// volatile float wr_cmd;

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
Execute initial celebration. Wave the hatch servo. Phases:


Transition to: orienting
*/
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
    imu.reset_integrators();

    // ITimer1.init();
    // ITimer1.setFrequency(CONTROLLER_FREQ, controller);

    // turn_left(MIDDLE, 1.5*PI);
    stop();
    // state = orienting;

    //flag: turn testing, remove and restore old state transition later
    forward_controller = 1;
    forward();
    imu.reset_integrators();
    state = driving_to_box;
    Serial.println("entering DRIVING_TO_BOX");
}

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
    static bool turn_complete = 0;

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
                angle_target_body = (float(map(angle_target, 0, 130, -90, 90)) * PI/180 +beacon_offset)/ 1.301;
            }

            Serial.print(">ServoAngle: ");
            Serial.println(servo_angle_deg);
            Serial.print(">IRReading: ");
            Serial.println(ir.value);
            Serial.print(">ConvValue: ");
            Serial.println(this_sum);
            Serial.print(">TargetAngle: ");
            Serial.println(angle_target);
        }
        imu.reset_integrators();
    }    
    else if(!turn_complete){
        Serial.print(">Target:");
        Serial.println(angle_target);
        Serial.print(">Mapped:");
        Serial.println(angle_target_body);
        Serial.print(">Current:");
        Serial.println(imu.angZ);
        swivel.write(60);

        turn_complete = abs(imu.angZ - angle_target_body) < PI/32;

        if (turn_complete) { 
            stop();
            Serial.println("DONE!");
            imu.reset_integrators();
            forward();
            state = driving_to_box;

        } else{
            if (angle_target_body < imu.angZ){
                turn_right(MIDDLE, 1.5*PI);
            }
            else{
                turn_left(MIDDLE, 1.5*PI);
            }
        }
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

    // if (counter >= 1) {
    if (1){
        state = aligning_with_gap;
        Serial.println("Entering aligning_with_gap");
        time_box_reached = millis();
    }
}
void aligning_with_gap() {
    if (millis() - time_box_reached > 1000) {
        stop();
        forward_controller = 0;
        imu.reset_integrators();
        switch (course) {
            case B:
                turn_left(MIDDLE, 1.5*PI);
                break;
            case A:
                turn_right(MIDDLE, 1.5*PI);
                break;
        }
        state = turning_to_gap;
        Serial.println("Entering turning_to_gap");
    }
}

#define DEBUG_TURNING_TO_GAP 1
unsigned long time_passed_gap;
void turning_to_gap() {
    static volatile bool turn_complete = false;
    if (DEBUG_TURNING_TO_GAP){
        Serial.print(">angZ:");
        Serial.println(imu.angZ);
    }

    switch (course) {
        case B:
            turn_complete = abs(imu.angZ - PI/2 / 1.3) < PI/32;
            if (DEBUG_TURNING_TO_GAP){
                Serial.print(">AngTarget:");
                Serial.println(PI/2 / 1.3);
            }
            break;
        case A:
            turn_complete = abs(imu.angZ - (-PI/2 / 1.3)) < PI/32;
            if (DEBUG_TURNING_TO_GAP){
                Serial.print(">AngTarget:");
                Serial.println(-PI/2 / 1.3);
            }
            break;
    }

    if (turn_complete) {
        stop();
        imu.reset_integrators();
        forward_controller = 1;
        forward();

        time_passed_gap = millis();

        state = driving_through_gap;
        Serial.println("Entering driving_through_gap");
    }
}

unsigned long time_line_reached;
void driving_through_gap() {
    // int8_t counter = min(ir_left_triggers,min(ir_right_triggers,ir_mid_triggers));
    // Serial.print(counter);
    // if (counter >= 3) {
    if (millis() < time_passed_gap + 3500){
        return;
    }
    reset_ir_triggers();
    // Serial.println(max(ir_left_on_line,max(ir_mid_on_line,ir_right_on_line)));
    if(ir_left()<IR_MIN || ir_right()<IR_MIN || ir_middle()<IR_MIN) {
    // if(ir_left_on_line || ir_mid_on_line || ir_right_on_line) {
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
        time_line_reached = millis();
    }
}

unsigned long time_driving_to_contact_zone;
void turning_to_contact_zone() {

    bool turn_complete = false;

    switch (course) {
        case B:
            // turn_complete = abs(imu.angZ) > RIGHT_90_TURN;
            turn_complete = abs(imu.angZ - (-PI/2 / 1.3)) < PI/32;

            break;
        case A:
            turn_complete = abs(imu.angZ - PI/2 / 1.3) < PI/32;
            break;
    }
    if (turn_complete || millis() - time_line_reached > 3000) { //90 deg turn left
        stop();

        imu.reset_integrators();
        forward_controller = 1;
        forward();
        //line_follow();  //flag -- line follow preliminary attempt

        state = driving_to_contact_zone;
        Serial.println("Entering driving_to_contact_zone");
        time_driving_to_contact_zone = millis();
    }
}

bool dtcz_stopped = 0;
unsigned long dtcz_time_stopped = 0;
void driving_to_contact_zone() {   //FLAG pretty good til here, line tracking worked surprisingly well for course A
    if( !dtcz_stopped){
        if(millis() > time_driving_to_contact_zone + 1000){
        //TO BE CONTINUED
            stop();
            forward_controller = 0;
            dtcz_stopped = 1;
            dtcz_time_stopped = millis();
            backward();
        }
    }
    else if (millis() > dtcz_time_stopped + 500){
        stop();
        switch (course) {
            case B:
                turn_left(BACKWARD,2*PI);
                break;
            case A:
                // turn_right(BACKWARD,2*PI);
                turn_right(MIDDLE,1.7*PI); //fffflag -- currently testing this, course A good up to post contact zone turn
                break;
        }
        imu.reset_integrators();
        state = turning_to_shooting_zone;
        Serial.println("Entering turning_to_shooting_zone");
    }
}

volatile unsigned long time_driving_to_shooting_zone;
void turning_to_shooting_zone() {
    bool turn_complete = false;
    switch (course) {
        case B:
            turn_complete = abs(imu.angZ - (PI/2 / 1.3)) < PI/32;
            break;
        case A:
            turn_complete = abs(imu.angZ - (-PI/2 / 1.3)) < PI/32;
            break;
    }
    if (turn_complete) {
        stop();

        imu.reset_integrators();
        forward_controller = 1;
        forward();
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
        forward_controller = 0;

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

float dw;
float wr_cmd;
float wl_cmd;
float kp = 2.;
float ki = 5.;
// float sign(float x) {x >= 0 ? 1. : -1.;}

#define DEBUG_CONTROLLER 0
void controller() {    

    imu.update_integrator();
    update_ir_states(); //black tape ir sensors
    ir.update();        //beacon IR

    if(forward_controller){
        dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_WIDTH/WHEEL_RADIUS;
        wr_cmd = DEFAULT_MOTOR_SPEED + dw/2;
        wl_cmd = DEFAULT_MOTOR_SPEED - dw/2;
        analogWrite(EnA, wr_cmd * RPS_TO_ANALOG);
        analogWrite(EnB, wl_cmd * RPS_TO_ANALOG);
        if (DEBUG_CONTROLLER){
            Serial.print(">imu.gyroZ:");
            Serial.println(imu.gyroZ);
            Serial.print(">imu.angZ:");
            Serial.println(imu.angZ);
            Serial.print(">wr_cmd:");
            Serial.println(wr_cmd);
            Serial.print(">wl_cmd:");
            Serial.println(wl_cmd);
        }
    }
}

/*
TEST CODE AND GRAVEYARD
*/
void test_turn(){
    // a testing state for turning.
    static int angle_target = 90;
    // static float angle_target_body = float(map(angle_target, 0, 130, -90, 90)) * PI/180;
    static float angle_target_body = float(angle_target) * PI/180 / 1.301;  
    //1.301 is calibrated for 1.5*PI speed -- can find calibration levels using serial plotter for other speeds/turn types

    static bool turn_complete = 0;
    static bool flag = 0;

    Serial.print(">Target:");
    Serial.println(angle_target);
    Serial.print(">Mapped:");
    Serial.println(angle_target_body);
    Serial.print(">Current:");
    Serial.println(imu.angZ);
    Serial.print(">FLAG:");
    Serial.println(flag);

    flag = 0;

    turn_complete = abs(imu.angZ - angle_target_body) < PI/32;

    if (turn_complete) { 
        stop();
        Serial.println("DONE!");
        flag=1;
        //imu.reset_integrators();
    } else{
        if (angle_target_body < imu.angZ){
            turn_right(MIDDLE, 1.5*PI);
        }
        else{
            turn_left(MIDDLE, 1.5*PI);
        }
    }
}
