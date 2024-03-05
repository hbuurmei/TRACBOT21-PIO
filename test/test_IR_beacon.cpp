// #include <Arduino.h>
#define USE_TIMER_1 1
#include <motor_control/motor_control.cpp>
#include <sensors/imu.cpp>
#include <sensors/ir_beacon.cpp>
#include <TimerInterrupt.h>

IMU imu;
IR_Beacon ir;

void setup() {
    Serial.begin(9600);
    stop();

    ir.initialize();
    imu.initialize();
    // delay(5000);
    imu.calibrate();
    turn_left(MIDDLE,0);
    imu.reset_integrators();

    ITimer1.init();
    ITimer1.setFrequency(25,[](){imu.update_integrator();});
}

void loop() {
    imu.update_measurement();
    ir.update();

    if (ir.max > 5 && ir.value < 0.95*ir.max){
        stop();
    }

    Serial.print(">ANGLE:");
    Serial.println(imu.angZ);
    Serial.print(">RAW:");
    Serial.println(ir.raw);
    Serial.print(">MAP:");
    Serial.println(ir.value);
    Serial.print(">MAV:");
    Serial.println(ir.mav);
}
