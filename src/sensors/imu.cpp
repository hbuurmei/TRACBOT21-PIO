#include <Arduino.h>
#include <MPU6050.h>
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
};