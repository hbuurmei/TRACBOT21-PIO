// LEFT MOTOR
#define IN1     13
#define IN2     12
#define EnA     11
// RIGHT MOTOR
#define IN3     8
#define IN4     7
#define EnB     6
#include <Arduino.h>
// #include <motor_control/motor_control.h>
#define DEFAULT_MOTOR_SPEED 3.5*PI // rad/s
#define MAX_MOTOR_SPEED 20.9440 // rad/s
#define RPS_TO_ANALOG 256 / MAX_MOTOR_SPEED
#define WHEEL_RADIUS 0.042 // metets
#define BASE_WIDTH 0.3048

// enum MOTOR_STATE {
//     STOPPED,

// }
// Basic Motor commands
void left_forward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN1,HIGH);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,LOW);  // Opposite of IN1
    analogWrite(EnA,speed * RPS_TO_ANALOG);
}
void right_forward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN3,LOW);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,HIGH);  // Opposite of IN3
    analogWrite(EnB,speed * RPS_TO_ANALOG);
}
void left_backward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN1,LOW);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,HIGH);  // Opposite of IN1
    analogWrite(EnA,speed * RPS_TO_ANALOG);
}
void right_backward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN3,HIGH);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,LOW);  // Opposite of IN3
    analogWrite(EnB,speed * RPS_TO_ANALOG);
}
void left_stop() {
    digitalWrite(IN1,LOW);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,LOW);  // Opposite of IN1
    analogWrite(EnA,0);
}
void right_stop() {
    digitalWrite(IN3,LOW);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,LOW);  // Opposite of IN3
    analogWrite(EnB,0);
}
void stop() {left_stop(); right_stop();}
void forward(float left_speed = DEFAULT_MOTOR_SPEED, float right_speed = DEFAULT_MOTOR_SPEED) {
    left_forward(left_speed); right_forward(right_speed);
}
void backward(float left_speed = DEFAULT_MOTOR_SPEED,float right_speed = DEFAULT_MOTOR_SPEED) {
    left_backward(left_speed); right_backward(right_speed);
}

// Higher level steering commands
enum TURN_MODE {
    FORWARD,
    BACKWARD
};
void turn_left(TURN_MODE turn_mode, float speed = DEFAULT_MOTOR_SPEED) {
    stop();
    switch (turn_mode) {
        case FORWARD:   right_forward(speed); break;
        case BACKWARD:  left_backward(speed); break;
    }
}
void turn_right(TURN_MODE turn_mode, float speed = DEFAULT_MOTOR_SPEED) {
    stop();
    switch (turn_mode) {
        case FORWARD:   left_forward(speed); break;
        case BACKWARD:  right_backward(speed); break;
    }
}