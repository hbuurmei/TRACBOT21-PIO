#include <Arduino.h>

void setup() {
    // put your setup code here, to run once:
    pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
}