//changed ir_beacon class to Ultrasonic class
//Ultrasonic:
#define trigPin 6
#define echoPin 5

#include <Arduino.h>

class Ultrasonic{
    public:
        long duration;
        float distance; //maybe make int
        // int target_time;

        void initialize();
        void update(); 
        
};

void Ultrasonic::initialize(){
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void Ultrasonic::update(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
}



//--------------------------
// long duration;
// int distance;
// void run_ultrasonic(){
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);
//     // Sets the trigPin on HIGH state for 10 micro seconds
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);
//     // Reads the echoPin, returns the sound wave travel time in microseconds
//     duration = pulseIn(echoPin, HIGH);
//     // Calculating the distance
//     distance = duration * 0.034 / 2;
//     // Prints the distance on the Serial Monitor
//     Serial.print("Distance: ");
//     Serial.println(distance);
// }