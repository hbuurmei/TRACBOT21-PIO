// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#define USE_TIMER_1     true
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Arduino.h>
#include <sensors/imu.cpp>
#include <TimerInterrupt.h>
#define CONTROLLER_FREQ 25.

// #define DEBUG

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// #define OUTPUT_IMU_SETTINGS

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
// #define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// For un


#define LED_PIN 13
bool blinkState = false;


void integrator();
// IMU mpu6050 = IMU(accelgyro);
IMU accelgyro;
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // // initialize device
    // Serial.println("Initializing I2C devices...");
    // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.initialize();
    Serial.print("Hello");
    accelgyro.calibrate();
    ITimer1.init();
    ITimer1.setFrequency(CONTROLLER_FREQ, integrator);

    // // verify connection
    // Serial.println("Testing device connections...");
    // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

}
bool reset = true;
void loop() {
    accelgyro.update_measurement();
    if (millis() > 5000 && reset) {accelgyro.reset_integrators();reset = false;}
    #ifndef DEBUG
        Serial.print("a/g:\t");
        Serial.print(accelgyro.accelX); Serial.print("\t");
        Serial.print(accelgyro.accelY); Serial.print("\t");
        Serial.print(accelgyro.accelZ); Serial.print("\t");
        Serial.print(accelgyro.gyroX); Serial.print("\t");
        Serial.print(accelgyro.gyroY); Serial.print("\t");
        Serial.println(accelgyro.gyroZ);
        // Serial.print("a/g:\t");
        // Serial.print(accelgyro.posX); Serial.print("\t");
        // Serial.print(accelgyro.posY); Serial.print("\t");
        // Serial.print(accelgyro.posZ); Serial.print("\t");
        // Serial.print(accelgyro.angX); Serial.print("\t");
        // Serial.print(accelgyro.angY); Serial.print("\t");
        // Serial.println(accelgyro.angZ);
    #endif
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        // Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
void integrator() {
    accelgyro.update_integrator();
}