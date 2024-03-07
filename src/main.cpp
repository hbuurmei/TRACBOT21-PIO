#define LEFT_90_TURN 85*PI/180
#define RIGHT_90_TURN 82.5*PI/180
#define swivel_interval 2    //2 degree turn intervals for now
#define ANGLE_TBD   PI/4    //TBD: angle to turn to hit contact zone after hitting black tape from beacon tracking

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

void waiting_for_button();
void start();
void orienting();
void driving_to_contact_zone();
void turning_to_shooting_zone();
void driving_to_shooting_zone();
void dropping_balls();
void celebrating();
void tracking_beacon();
void turn_to_contact_zone();
void back_up();

// Initialize state
void (*state) (void) = start;  // usually we would use waiting_for_button

// Control functions
void controller();
static bool forward_controller = 0; //set to 1 whenever trying to drive straight, 0 otherwise, logic in controller
static volatile float wz;
static volatile float iwz;
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
    swivel.write(60);
    hatch.attach(HATCH_SERVO_PIN);
    button_setup();
    timer.init();
    timer.setInterval(40, controller);
}

void loop() {
    imu.update_measurement();
    state();
    timer.run();
}

//currently not using this state
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
    hatch.write(HATCH_OPEN);
    delay(500);
    hatch.write(HATCH_CLOSED);
    delay(500);
    hatch.write(HATCH_OPEN);
    delay(500);
    hatch.write(HATCH_CLOSED);
    delay(5000);
    //Load balls during the 5s delay
    //Could start timer here instead of in setup if we want to avoid any potential delay issues for the celebration / ball loading

    imu.reset_integrators();
    stop();
    state = orienting;
}

//this is an old orienting, should be replaced with whatever version works the best
void orienting(){
    /*
    - sweep 180 degrees with servo, measure maximum value & angle 
    - if value not higher than some threshold (100 ?), turn 120 and repeat
    - else, turn to angle of beacon, offset by value based on course A or course B
    */
    static int servo_angle_deg = 0;
    static int angle_target = 0;
    static float angle_target_body = 0;
    // static bool found_target = 0;
    // static float orientation_angle = 66*PI/180; //FLAG: may need refining
    static int max_ir_reading = 0;
    static bool turn_complete = 0;

    static unsigned long last_servo_move = 0;
    
    if (servo_angle_deg <= 130){
        if (millis()>last_servo_move+250){
            last_servo_move = millis();
            swivel.write(servo_angle_deg);
            servo_angle_deg += swivel_interval;
            Serial.print(">Angle: ");
            Serial.println(servo_angle_deg);
            Serial.print(">Reading: ");
            Serial.println(ir.value);
            if (ir.value > max_ir_reading){
                max_ir_reading = ir.value;
                angle_target = servo_angle_deg;
                angle_target_body = float(map(angle_target, 0, 130, -90, 90)) * PI/180 / 1.301;
            }
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
        
        turn_complete = abs(imu.angZ - angle_target_body) < PI/32;
        // turn_complete = abs(imu.angZ) >= abs(angle_target_body);

        if (turn_complete) { 
            stop();
            Serial.println("DONE!");
        } else{
            if (angle_target_body < imu.angZ){
                turn_right(MIDDLE, 1.5*PI);
            }
            else{
                turn_left(MIDDLE, 1.5*PI);
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

    //start tracking beacon
    forward();
    state = tracking_beacon;
}

//This would probably be the second iteration of tracking beacon (track a bit, reorient, go back to tracking through gate)
//that logic has not yet been implemented bc will probably involve some trial and error in terms of optimal place to reset beacon tracking to get through gate
//Entry to this state assumes robot is already past the starting zone gate, and is moving straight towards the beacon before reaching black tape to contact zone
void tracking_beacon(){
    if(ir_left()<IR_MIN || ir_right()<IR_MIN || ir_middle()<IR_MIN){    //we see contact zone tape
        //may need some time offset here to drive a lil past the black line
        stop();
        switch (course) {   //start turn to contact zone
            case B:
                turn_right(MIDDLE,1.7*PI); 
                break;
            case A:
                turn_left(MIDDLE,1.7*PI);
                break;
        }
        imu.reset_integrators();
        state = turn_to_contact_zone;
        Serial.println("Entering turn_to_contact_zone");
    }
}

int time_tracker;   //reusing this as a timer throughout the states
void turn_to_contact_zone(){
    static volatile float angZ;
    static volatile bool turn_complete = false;
    angZ = imu.angZ;
    // Serial.println(angZ);
    switch (course) {
        case B:
            turn_complete = angZ <= -ANGLE_TBD; //maybe change how turns are handled, depending on how Bobby's turn refinement turns out
            break;
        case A:
            turn_complete = angZ >= ANGLE_TBD;
            break;
    }
    if (turn_complete) {    //now facing contact zone
        stop();
        time_tracker = millis();
        forward_controller = 1; //activate forward controller
        forward();
        imu.reset_integrators();
        state = driving_to_contact_zone;
        Serial.println("Entering driving_to_contact_zone"); 
    }
}

void driving_to_contact_zone() {
    if(millis()-time_tracker >= 1000){  //drive forward for 1 second, hopefully hit contact zone withtin that (refine)
        stop();
        forward_controller = 0;
        time_tracker = millis();
        imu.reset_integrators();
        state = back_up;
    }
}

void back_up(){
    backward(); //bck up from contact zone
    if(millis()-time_tracker >= 800){
        stop(); //stop after 800 ms (refine)
        switch (course) {
        case B:
            turn_left(MIDDLE,1.7*PI);
            break;
        case A:
            turn_right(MIDDLE,1.7*PI);
            break;
        }
        imu.reset_integrators();
        state = turning_to_shooting_zone;
        Serial.println("Entering turning_to_shooting_zone");
    }
}

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
        imu.reset_integrators();
        time_tracker = millis();
        forward_controller = 1;
        forward();
        state = driving_to_shooting_zone;
        Serial.println("Entering driving_to_shooting_zone");
    }
}
void driving_to_shooting_zone() {
    if (millis() - time_tracker > 4000) {   //value TBD, 4 seconds as placeholder
        stop();
        forward_controller = 0;
        imu.reset_integrators();
        switch (course) {
            case B:
                swivel.write(SWIVEL_RIGHT_ANGLE);
                break;
            case A:
                swivel.write(SWIVEL_LEFT_ANGLE);
                break;
        }
        time_tracker = millis();
        state = dropping_balls;
    }
}

void dropping_balls() {
    hatch.write(HATCH_OPEN);
    if(millis()-time_tracker>=1000){
        state = celebrating;
    }
}

void celebrating() {   
    delay(300);
    hatch.write(HATCH_CLOSED);
    delay(300);
    hatch.write(HATCH_OPEN);
    delay(300);
    hatch.write(HATCH_CLOSED);
    delay(300);
    hatch.write(HATCH_OPEN);
}


float dw;
float wr_cmd;
float kp = 2.;
float ki = 5.;
void controller() {    
    imu.update_integrator();
    update_ir_states(); //black tape ir sensors
    ir.update();        //beacon IR

    if(forward_controller){
        dw = -(kp*imu.gyroZ + ki*imu.angZ)*BASE_WIDTH/WHEEL_RADIUS;
        wr_cmd = dw + DEFAULT_MOTOR_SPEED;
        analogWrite(EnA, wr_cmd/2 * RPS_TO_ANALOG);
        analogWrite(EnB, -wr_cmd/2 * RPS_TO_ANALOG);
    }
}