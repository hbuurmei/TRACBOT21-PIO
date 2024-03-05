// #include <Arduino.h>
#define USE_TIMER_1 true
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>
#include <sensors/ir_beacon.cpp>
#include <TimerInterrupt.h>

IMU imu;
IR_Beacon ir;

void check_turn();
void setup() {
    Serial.begin(9600);
    stop();

    ir.initialize();
    imu.initialize();
    // delay(5000);
    imu.calibrate();
    turn_left(MIDDLE,DEFAULT_MOTOR_SPEED);
    imu.reset_integrators();
    ITimer1.init();
    ITimer1.setFrequency(25, check_turn);
}

void loop() {
    static int lastMax = 0;
    imu.update_measurement();
    ir.update();

    if (lastMax != ir.max){
        Serial.println("NEW MAX!");
        lastMax = ir.max;
    }

    Serial.print("ANGLE: ");
    Serial.print(imu.angZ);
    Serial.print("RAW: ");
    Serial.print(ir.value);
    Serial.print("MAV: ");
    Serial.print(ir.mav);
}
