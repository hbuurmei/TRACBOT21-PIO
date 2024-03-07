// #define IR_LEFT     A5
// #define IR_MID      A4
// #define IR_RIGHT    A3
#define IR_RIGHT    A0
#define IR_MID      A1
#define IR_LEFT     A2



#include <Arduino.h>
#include <sensors/ir_line.cpp>

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
    update_ir_states();

    Serial.print("Left:");
    Serial.print(ir_left());
    Serial.print(",");
    Serial.print("Mid:");
    Serial.print(ir_middle());
    Serial.print(",");
    Serial.print("Right:");
    Serial.print(ir_right());

    Serial.print(" dLeft:");
    Serial.print(ir_left_triggers);
    Serial.print(",");
    Serial.print(" dMid:");
    Serial.print(ir_mid_triggers);
    Serial.print(",");
    Serial.print(" dRight:");
    Serial.println(ir_right_triggers);

    // Serial.print("Left:");
    // Serial.print(line_detected(IR_LEFT));
    // Serial.print(",");
    // Serial.print("Mid:");
    // Serial.print(line_detected(IR_MID));
    // Serial.print(",");
    // Serial.print("Right:");
    // Serial.println(line_detected(IR_RIGHT));
}