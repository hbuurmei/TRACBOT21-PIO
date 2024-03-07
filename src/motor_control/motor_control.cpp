// RIGHT MOTOR
#define IN1     13
#define IN2     12
#define EnA     11
// LEFT MOTOR
#define IN3     8
#define IN4     7
#define EnB     3
#define LOGIC_5V 
#include <Arduino.h>
#include <sensors/ir_line.cpp>  //flag - for line follow attempt
// #include <motor_control/motor_control.h>
#define DEFAULT_MOTOR_SPEED 2.5*PI // rad/s    //used to be 1.75*PI
#define MAX_MOTOR_SPEED 10.472 // rad/s
#define RPS_TO_ANALOG 256 / MAX_MOTOR_SPEED
#define WHEEL_RADIUS 0.042 // metets
#define BASE_WIDTH 0.3048

// Basic Motor commands
void right_forward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN1,HIGH);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,LOW);  // Opposite of IN1
    analogWrite(EnA,speed * RPS_TO_ANALOG);
}
void left_forward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN3,LOW);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,HIGH);  // Opposite of IN3
    analogWrite(EnB,(speed + 0.0) * RPS_TO_ANALOG);
}
void right_backward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN1,LOW);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,HIGH);  // Opposite of IN1
    analogWrite(EnA,speed * RPS_TO_ANALOG);
}
void left_backward(float speed = DEFAULT_MOTOR_SPEED) {
    digitalWrite(IN3,HIGH);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,LOW);  // Opposite of IN3
    analogWrite(EnB,(speed + 0.0) * RPS_TO_ANALOG);
}
void right_stop() {
    digitalWrite(IN1,LOW);  // HIGH = Forward, LOW = Backward
    digitalWrite(IN2,LOW);  // Opposite of IN1
    analogWrite(EnA,0);
}
void left_stop() {
    digitalWrite(IN3,LOW);  // LOW = Forward, HIGH = Backward
    digitalWrite(IN4,LOW);  // Opposite of IN3
    analogWrite(EnB,0);
}
void stop() {left_stop(); right_stop();}
void forward(float left_speed = DEFAULT_MOTOR_SPEED, float right_speed = DEFAULT_MOTOR_SPEED) {
    left_forward(left_speed); 
    right_forward(right_speed);
}
//3*PI speed forward to hopefully go straighter
void forward_3pi(float left_speed = 3*PI, float right_speed = 3*PI) {
    left_forward(left_speed); 
    right_forward(right_speed);
}

void backward(float left_speed = DEFAULT_MOTOR_SPEED,float right_speed = DEFAULT_MOTOR_SPEED) {
    left_backward(left_speed); 
    right_backward(right_speed);
}

// Higher level steering commands
enum TURN_MODE {
    FORWARD,
    BACKWARD,
    MIDDLE
};
void turn_left(TURN_MODE turn_mode, float speed = DEFAULT_MOTOR_SPEED) {
    switch (turn_mode) {
        case FORWARD:   right_forward(speed); left_stop(); break;
        case BACKWARD:  left_backward(speed); right_stop(); break;
        case MIDDLE:    right_forward(speed); left_backward(speed); break;
    }
}
void turn_right(TURN_MODE turn_mode, float speed = DEFAULT_MOTOR_SPEED) {
    switch (turn_mode) {
        case FORWARD:   left_forward(speed); right_stop(); break;
        case BACKWARD:  right_backward(speed); left_stop(); break;
        case MIDDLE:    left_forward(speed); right_backward(speed); break;
    }
}

//FLAG line follow test
void line_follow(float speed = DEFAULT_MOTOR_SPEED){
    if (ir_left()<IR_MIN){
        turn_left(FORWARD, 1.7*PI);
    }
    else if (ir_right()<IR_MIN){
        turn_right(FORWARD,1.7*PI);
    }
    else{
        forward();
    }
}

// //context -- delete later (or at least comment out)
// void forward(float left_speed = DEFAULT_MOTOR_SPEED, float right_speed = DEFAULT_MOTOR_SPEED) {
//     left_forward(left_speed); 
//     right_forward(right_speed);
// }
// void right_forward(float speed = DEFAULT_MOTOR_SPEED) {
//     digitalWrite(IN1,HIGH);  // HIGH = Forward, LOW = Backward
//     digitalWrite(IN2,LOW);  // Opposite of IN1
//     analogWrite(EnA,speed * RPS_TO_ANALOG);
// }