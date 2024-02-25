#include <Arduino.h>
#include <unity.h>

void setup() {
    // put your setup code here, to run once:
    pinMode(LED_BUILTIN,OUTPUT);
    UNITY_BEGIN();
}

void loop() {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
    if (millis() > 20000) {UNITY_END();}
}