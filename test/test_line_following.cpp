#define IN1     13
#define IN2     12
#define IN3     8
#define IN4     7
#define EnA     11
#define EnB     6

#define IR_LEFT     A0
#define IR_MID      A1
#define IR_RIGHT    A2

// Tune the values below
#define threshold 30
#define DEFAULT_MOTOR_SPEED 2.5*PI  // rad/s
#define TURN_SPEED_INCREMENT 0.25*PI  // rad/s
#define MAX_MOTOR_SPEED 20.9440 // rad/s
#define RPS_TO_ANALOG 256 / MAX_MOTOR_SPEED
#define WHEEL_RADIUS 0.042 // metets
#define BASE_WIDTH 0.3048

#include <Arduino.h>

typedef enum {
    LEFT,
    MID,
    RIGHT,
    OFF_TRACK
} States_t;

typedef enum {
  NO_DEBUG,
  DEBUG_TEST_LIGHT,
  DEBUG_TEST_LINE
} Debugs_t;

// Note that the robot state is the inverse of what the sensors see, i.e.
// if the LEFT sensor sees the line, the robot is in the RIGHT state
States_t robot_state;
States_t control_state;
Debugs_t debug;

void updateRobotState();
void handleLeftState();
void handleMidState();
void handleRightState();
void handleGoStraight();
void handleStop();
void handleOffTrack();
void handleRightTurn();
void handleLeftTurn();
void handleDebugTestLight();
void handleDebugTestLine();


void setup() {
    Serial.begin(9600);
    // Initialize the robot state to MID
    robot_state = MID;
    control_state = MID;
    debug = DEBUG_TEST_LINE;
    // Start by going straight
    handleGoStraight();
}


void loop() {
    updateRobotState();
    switch (robot_state) {
        case LEFT:
            handleLeftState();
            break;
        case MID:
            handleMidState();
            break;
        case RIGHT:
            handleRightState();
            break;
        case OFF_TRACK:
            handleOffTrack();
            break;
        default:
            // Should never get into this unhandled state
            Serial.println("Entered unknown state...");
    }
    switch (debug) {
        case DEBUG_TEST_LIGHT:
            handleDebugTestLight();
            break;
        case DEBUG_TEST_LINE:
            handleDebugTestLine();
            break;
        default:
            // Do nothing
            break;
    }
}


void updateRobotState() {
    // Update the robot state based on the sensor readings
    if (analogRead(IR_LEFT) < threshold) {
        robot_state = RIGHT;
    } else if (analogRead(IR_MID) < threshold) {
        robot_state = MID;
    } else if (analogRead(IR_RIGHT) < threshold) {
        robot_state = LEFT;
    }
    else {
        robot_state = OFF_TRACK;
    }
};

void handleOffTrack() {
    // Set the motor speeds to stop
    handleStop();
    Serial.println("Robot went off track so it stopped...");
};

void handleLeftState() {
    switch (control_state) {
        case LEFT:
            // We do not want to turn left if the robot is already left
            control_state = MID;
            break;
        case MID:
            // You want to turn right if the robot is left
            handleRightTurn();
            control_state = RIGHT;
            break;
        case RIGHT:
            // We are turning right
            break;
        case OFF_TRACK:
            control_state = MID;
            break;
    }
};

void handleMidState() {
    switch (control_state) {
        case LEFT:
            // Stay mid
            handleGoStraight();
            control_state = MID;
            break;
        case MID:
            // We are in nominal line following condition
            break;
        case RIGHT:
            // Stay mid
            handleGoStraight();
            control_state = MID;
            break;
        case OFF_TRACK:
            control_state = MID;
            break;
    }
};

void handleRightState() {
    switch (control_state) {
    case LEFT:
        // We are turning left
        break;
    case MID:
        // You want to turn left if the robot is right
        handleLeftTurn();
        control_state = LEFT;
        break;
    case RIGHT:
        // We do not want to turn right if the robot is already right
        control_state = MID;
        break;
    case OFF_TRACK:
            control_state = MID;
            break;
    }
};

void handleGoStraight() {
    // Set the motor speeds to go straight
    // Left Motor
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EnA, DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
    // Right Motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EnB, DEFAULT_MOTOR_SPEED * RPS_TO_ANALOG);
};

void handleStop() {
    // Set the motor speeds to stop
    // Left Motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EnA, 0);
    // Right Motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EnB, 0);
};

void handleRightTurn() {
    // Turning right is defined as negative angular velocity
    float drps_left = TURN_SPEED_INCREMENT * BASE_WIDTH/2 / WHEEL_RADIUS;
    float rps_cmd_left = DEFAULT_MOTOR_SPEED + drps_left;
    analogWrite(EnA, rps_cmd_left * RPS_TO_ANALOG);
    float drps_right = -drps_left;
    float rps_cmd_right = DEFAULT_MOTOR_SPEED + drps_right;
    analogWrite(EnB, rps_cmd_right * RPS_TO_ANALOG);
};

void handleLeftTurn() {
    // Turning left is defined as positive angular velocity
    float drps_right = TURN_SPEED_INCREMENT * BASE_WIDTH/2 / WHEEL_RADIUS;
    float rps_cmd_right = DEFAULT_MOTOR_SPEED + drps_right;
    analogWrite(EnB, rps_cmd_right * RPS_TO_ANALOG);
    float drps_left = -drps_right;
    float rps_cmd_left = DEFAULT_MOTOR_SPEED + drps_left;
    analogWrite(EnA, rps_cmd_left * RPS_TO_ANALOG);
};

void handleDebugTestLight(){
    // Print raw light sensor values
    Serial.print("Left:");
    Serial.print(analogRead(IR_LEFT));
    Serial.print(",");
    Serial.print("Mid:");
    Serial.print(analogRead(IR_MID));
    Serial.print(",");
    Serial.print("Right:");
    Serial.println(analogRead(IR_RIGHT));
};

void handleDebugTestLine(){
    // Print checks if IR values below or above threshold for each sensor
    Serial.print("Left:");
    Serial.print(analogRead(IR_LEFT) < threshold);
    Serial.print(",");
    Serial.print("Mid:");
    Serial.print(analogRead(IR_MID) < threshold);
    Serial.print(",");
    Serial.print("Right:");
    Serial.println(analogRead(IR_RIGHT) < threshold);
};
