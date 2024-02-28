#define IN1     13
#define IN2     12
#define IN3     8
#define IN4     7
#define EnA     11
#define EnB     6

#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(EnA,OUTPUT);
  pinMode(EnB,OUTPUT);
  // Configure Motor 1 (LEFT)
  digitalWrite(IN1,HIGH);  // HIGH = Forward, LOW = Backward
  digitalWrite(IN2,LOW);  // Opposite of IN1
  analogWrite(EnA,140);
  // Configure Motor 2 (RIGHT)
  digitalWrite(IN3,LOW);  // LOW = Forward, HIGH = Backward
  digitalWrite(IN4,HIGH);  // Opposite of IN3
  analogWrite(EnB,140);
}

void loop() {
  static char input;
  // put your main code here, to run repeatedly:
  
  // Serial.print("Hello");
  if (Serial.available()) {
    input = Serial.read();
    Serial.println(input);
    while (Serial.available()) {Serial.read();}
  }
  
}