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
  // Configure Motor 1
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(EnA,200);
  // Configure Motor 2
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(EnB,200);
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