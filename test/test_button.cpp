#define buttonPin 2

#include <Arduino.h>


void setup() {
    Serial.begin(9600);
    pinMode(buttonPin, INPUT_PULLUP);  // set the button pin as input with pullup resistor
}

void loop() {
    // Will print 0 if button is pressed, 1 if not
    Serial.println(digitalRead(buttonPin));
}
