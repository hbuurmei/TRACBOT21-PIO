#define IR_BEACON A3
#define IR_RST 4

#define IR_RAW_MIN 561
#define IR_RAW_MAX 1000

#define IR_POLL_INT 75
#define IR_AV_LEN 10

#include <Arduino.h>

class IR_Beacon{
    public:
        float mav;
        int raw;
        int value;
        int max;
        float angle;

        void initialize();
        void update(float curr_ang); 
        void reset();
    private:
        float alpha = 0.1;
};

void IR_Beacon::initialize(){
    pinMode(IR_RST, OUTPUT);
    pinMode(IR_BEACON, INPUT);
    digitalWrite(IR_RST, LOW);
    analogReference(DEFAULT);
}

void IR_Beacon::reset(){
    mav = 0;
    max = 0;
}

void IR_Beacon::update(float curr_ang){
    static volatile unsigned long last_check = 0;
    if (millis() > last_check + IR_POLL_INT){
        last_check = millis();
        raw = analogRead(IR_BEACON);
        value = map(
            raw,
            IR_RAW_MIN, IR_RAW_MAX,
            0, 1023
        );
        // mav = (alpha*value)+ (1.0-alpha) * mav;
        // if (value > max){
        //     max = value;
        //     angle = curr_ang;
        // }
        digitalWrite(IR_RST, HIGH);
    }
    else {
        digitalWrite(IR_RST, LOW);
    }
}
