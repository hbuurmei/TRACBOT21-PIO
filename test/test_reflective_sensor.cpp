#define IR_LEFT     A5
#define IR_MID      A4
#define IR_RIGHT    A3



#include <Arduino.h>

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
    Serial.print("Left:");
    Serial.print(analogRead(IR_LEFT));
    Serial.print(",");
    Serial.print("Mid:");
    Serial.print(analogRead(IR_MID));
    Serial.print(",");
    Serial.print("Right:");
    Serial.println(analogRead(IR_RIGHT));
}