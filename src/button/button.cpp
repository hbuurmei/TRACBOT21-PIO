#define buttonPin 2

#include <Arduino.h>

void button_setup() {
    // Set the button pin as input with pullup resistor
    pinMode(buttonPin, INPUT_PULLUP);
}
