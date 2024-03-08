#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SWIVEL_SERVO        0
#define HATCH_SERVO         1

#define SERVO_FREQ 100 // Analog servos run at ~50 Hz updates

#define SWIVEL_MAX 1700
#define SWIVEL_MID 1000
#define SWIVEL_MIN 500 / 1000000/SERVO_FREQ * 4096

#define SWIVEL_MIN

#define SWIVEL_MIN_USEC 450
// #define SWIVEL_MAX_USEC 2500
#define SWIVEL_MAX_USEC 1700

#define SWIVEL_MIN_DEG 0
//#define SWIVEL_MAX_DEG 270
#define SWIVEL_MAX_DEG 180

class ServoDriver{
    public:
        void initialize(){
            pwm = Adafruit_PWMServoDriver();
            pwm.begin();
            pwm.setOscillatorFrequency(27000000);
            pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
        }

        void setSwivelAngle(long angle){
            angle = min(max(angle, 0), 180);
            pwm.setPWM(SWIVEL_SERVO, 0, deg_to_pwm(angle));
        }

        void closeHatch(){
            pwm.setPWM(HATCH_SERVO,0,200);
        }

        void openHatch(){
            pwm.setPWM(HATCH_SERVO,0,750);
        }

    private:
        Adafruit_PWMServoDriver pwm;

        uint16_t deg_to_pwm(long degrees){
            long usec = map(degrees, SWIVEL_MIN_DEG, SWIVEL_MAX_DEG, SWIVEL_MIN_USEC, SWIVEL_MAX_USEC);
            return 4096 * usec / (1000000/SERVO_FREQ);
        }
    
};