// #include <Arduino.h>
#define USE_TIMER_1 1
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>
#include <sensors/ir_beacon.cpp>
#include <TimerInterrupt.h>

#define ANGLE_INCREMENT PI/13
#define MEASURE_DELAY 500

IMU imu;
IR_Beacon ir;

float angle_target = 0; 
unsigned long state_change = 0;

void next_angle();
void take_measurement();

void (*state) (void) = next_angle;

void setup() {
    Serial.begin(9600);
    stop();

    ir.initialize();
    imu.initialize();
    // delay(5000);
    imu.calibrate();
    imu.reset_integrators();
    
    ITimer1.init();
    ITimer1.setFrequency(25,[](){imu.update_integrator();});

    angle_target=0;
}


void loop() {
    imu.update_measurement();
    ir.update(imu.angZ);
    state();
}

void next_angle(){
    //if(imu.angZ >= angle_target){
    if (millis() > state_change + 500){
        state=take_measurement;
        stop();
        state_change= millis();
    }
    else{
       // turn_left(MIDDLE, 1.7*PI);
    }
}

void take_measurement(){
    if (millis() - state_change > MEASURE_DELAY){
        Serial.print(">ANGLE:");
        Serial.println(imu.angZ);
        Serial.print(">RAW:");
        Serial.println(ir.raw);
        Serial.print(">MAP:");
        Serial.println(ir.value);
        Serial.print(">MAV:");
        Serial.println(ir.mav);

        state = next_angle;
        state_change = millis();
        angle_target += ANGLE_INCREMENT;
    }
}
