// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Arduino.h>
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

class IMU : public MPU6050 {
    public:
        void initialize() {
            MPU6050::initialize();
            update_config();
        }
        void update_config() {
            int8_t accel_setting = getFullScaleAccelRange();
            switch (accel_setting) {
                case 0: //+-2g
                    accel_g_to_raw = 16384.;
                    accel_raw_to_g = 1./16384.;
                    break;
                case 1: //+-4g
                    accel_g_to_raw = 8192.;
                    accel_raw_to_g = 1./8192.;
                    break;
                case 2: //+-8g
                    accel_g_to_raw = 4096.;
                    accel_raw_to_g = 1./4096.;
                    break;
                case 3: //+-16g
                    accel_g_to_raw = 2048.;
                    accel_raw_to_g = 1./2048.;
                    break;
            }
            int8_t gyro_setting = getFullScaleGyroRange();
            switch (gyro_setting) {
                case 0: //+-250deg/s
                    gyro_dps_to_raw = 131.;
                    gyro_raw_to_dps = 1./131.;
                    break;
                case 1: //+-500deg/s
                    gyro_dps_to_raw = 65.5;
                    gyro_raw_to_dps = 1./65.5;
                    break;
                case 2: //+-1000deg/s
                    gyro_dps_to_raw = 32.8;
                    gyro_raw_to_dps = 1./32.8;
                    break;
                case 3: //+-2000deg/s
                    gyro_dps_to_raw = 16.4;
                    gyro_raw_to_dps = 1./16.4;
                    break;
            }
        }
        void update_measurement() {
            getMotion6(&accelX_raw,&accelY_raw,&accelZ_raw,&gyroX_raw,&gyroY_raw,&gyroZ_raw);
            accelX = accelX_raw * accel_raw_to_g * g_to_m - accelX_bias;
            accelY = accelY_raw * accel_raw_to_g * g_to_m - accelY_bias;
            accelZ = accelZ_raw * accel_raw_to_g * g_to_m - accelZ_bias;
            gyroX = gyroX_raw * gyro_raw_to_dps * deg_to_rad - gyroX_bias;
            gyroY = gyroY_raw * gyro_raw_to_dps * deg_to_rad - gyroY_bias;
            gyroZ = gyroZ_raw * gyro_raw_to_dps * deg_to_rad - gyroZ_bias;
            #ifdef DEBUG
                Serial.print("a/g:\t");
                Serial.print(accelX_raw); Serial.print("\t");
                Serial.print(accelY_raw); Serial.print("\t");
                Serial.print(accelZ_raw); Serial.print("\t");
                Serial.print(gyroX_raw); Serial.print("\t");
                Serial.print(gyroY_raw); Serial.print("\t");
                Serial.print(gyroZ_raw); Serial.print("\t");
                Serial.print(accel_g_to_raw); Serial.print("\t");
                Serial.print(gyro_dps_to_raw); Serial.print("\t");
                Serial.print(getFullScaleAccelRange()); Serial.print("\t");
                Serial.println(getFullScaleGyroRange());
            #endif
        }
        void calibrate() {
            update_measurement();
            accelX_bias = accelX;
            accelY_bias = accelY;
            accelZ_bias = accelZ;
            gyroX_bias = gyroX;
            gyroY_bias = gyroY;
            gyroZ_bias = gyroZ;
        }
        float accelX;       // meters/sec^2
        float accelX_bias;  //meters/sec^2
        float accelY;       // meters/sec^2
        float accelY_bias;  //meters/sec^2
        float accelZ;       // meters/sec^2
        float accelZ_bias;  //meters/sec^2
        float gyroX;        // rad/sec
        float gyroX_bias;   // rad/sec
        float gyroY;        // rad/sec
        float gyroY_bias;   // rad/sec
        float gyroZ;        // rad/sec
        float gyroZ_bias;   // rad/sec
    private:
        const float g_to_m = 9.81;
        const float deg_to_rad = PI/180.;
        int16_t accelX_raw; // int16_t from min to max range
        int16_t accelY_raw; // int16_t from min to max range
        int16_t accelZ_raw; // int16_t from min to max range
        int16_t gyroX_raw; // int16_t from min to max range
        int16_t gyroY_raw; // int16_t from min to max range
        int16_t gyroZ_raw; // int16_t from min to max range
        float accel_raw_to_g;
        float accel_g_to_raw;
        float gyro_raw_to_dps;
        float gyro_dps_to_raw;
        // enum IMU_ACCEL_RANGE accel_range;
};

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
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.calibrate();

    // // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

}
void loop() {
    accelgyro.update_measurement();
    #ifndef DEBUG
        Serial.print("a/g:\t");
        Serial.print(accelgyro.accelX); Serial.print("\t");
        Serial.print(accelgyro.accelY); Serial.print("\t");
        Serial.print(accelgyro.accelZ); Serial.print("\t");
        Serial.print(accelgyro.gyroX); Serial.print("\t");
        Serial.print(accelgyro.gyroY); Serial.print("\t");
        Serial.println(accelgyro.gyroZ);
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
