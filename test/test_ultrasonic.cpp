#define TRIG_PIN 6
#define ECHO_PIN 5

#include <Arduino.h>


void setup() {  
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	Serial.begin(9600);  
}

void loop() { 
    static float duration = 0;
    static float distance = 0;
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);  
	digitalWrite(ECHO_PIN, HIGH);  
	delayMicroseconds(10);
	digitalWrite(ECHO_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration*.0343)/2;  

	Serial.print("Distance in cm: ");  
	Serial.println(distance);  
	delay(100);  
}
