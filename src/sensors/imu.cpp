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
            // accelX = accelX_raw * accel_raw_to_g * g_to_m - accelX_bias;
            // accelY = accelY_raw * accel_raw_to_g * g_to_m - accelY_bias;
            // accelZ = accelZ_raw * accel_raw_to_g * g_to_m - accelZ_bias;
            // gyroX = gyroX_raw * gyro_raw_to_dps * deg_to_rad - gyroX_bias;
            // gyroY = gyroY_raw * gyro_raw_to_dps * deg_to_rad - gyroY_bias;
            // gyroZ = gyroZ_raw * gyro_raw_to_dps * deg_to_rad - gyroZ_bias;
            accelX = (accelX_raw * accel_raw_to_g * g_to_m - accelX_bias + accelX)/2;
            accelY = (accelY_raw * accel_raw_to_g * g_to_m - accelY_bias + accelY)/2;
            accelZ = (accelZ_raw * accel_raw_to_g * g_to_m - accelZ_bias + accelZ)/2;
            gyroX = (gyroX_raw * gyro_raw_to_dps * deg_to_rad - gyroX_bias + gyroX)/2;
            gyroY = (gyroY_raw * gyro_raw_to_dps * deg_to_rad - gyroY_bias + gyroY)/2;
            gyroZ = (gyroZ_raw * gyro_raw_to_dps * deg_to_rad - gyroZ_bias + gyroZ)/2;
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
        void calibrate(int samples = 500) {
            float accelX_sum = 0;
            float accelY_sum = 0;
            float accelZ_sum = 0;
            float gyroX_sum = 0;
            float gyroY_sum = 0;
            float gyroZ_sum = 0;
            for (int i = 0; i < samples; i++) {
                update_measurement();
                accelX_sum += accelX;
                accelY_sum += accelY;
                accelZ_sum += accelZ;
                gyroX_sum += gyroX;
                gyroY_sum += gyroY;
                gyroZ_sum += gyroZ;
                delay(1);
            }
            accelX_bias = accelX_sum/samples;
            accelY_bias = accelY_sum/samples;
            accelZ_bias = accelZ_sum/samples;
            gyroX_bias = gyroX_sum/samples;
            gyroY_bias = gyroY_sum/samples;
            gyroZ_bias = gyroZ_sum/samples;
        }
        void update_integrator() {
            angX += gyroX/25.;
            angY += gyroY/25.;
            angZ += gyroZ/25.;
            posX += velX/25.;
            posY += velY/25.;
            posZ += velZ/25.;
            velX += accelX/25.;
            velY += accelX/25.;
            velZ += accelX/25.;
            dist = sqrt(posX*posX + posY*posY);
            dist_rate = sqrt(velX*velX + velY*velY);
        }
        void reset_integrators() {
            angX = 0.;
            angY = 0.;
            angZ = 0.;
            velX = 0.;
            velY = 0.;
            velZ = 0.;
            posX = 0.;
            posY = 0.;
            posZ = 0.;
            accelX = 0.;
            accelY = 0.;
            accelZ = 0.;
            dist = 0.;
            dist_rate = 0.;
        }
        volatile float accelX;       // meters/sec^2
        float accelX_bias;  // meters/sec^2
        volatile float accelY;       // meters/sec^2
        float accelY_bias;  // meters/sec^2
        volatile float accelZ;       // meters/sec^2
        float accelZ_bias;  // meters/sec^2
        volatile float gyroX;        // rad/sec
        float gyroX_bias;   // rad/sec
        volatile float gyroY;        // rad/sec
        float gyroY_bias;   // rad/sec
        volatile float gyroZ;        // rad/sec
        float gyroZ_bias;   // rad/sec
        volatile float angX = 0.;     // rad
        volatile float angY = 0.;     // rad
        volatile float angZ = 0.;     // rad
        volatile float velX = 0.;     // meters/sec
        volatile float velY = 0.;     // meters/sec
        volatile float velZ = 0.;     // meters/sec
        volatile float posX = 0.;     // meters
        volatile float posY = 0.;     // meters
        volatile float posZ = 0.;     // meters
        volatile float dist = 0.;     // meters
        volatile float dist_rate = 0.; // meters
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
        unsigned long prev_time = 0; //ms
};