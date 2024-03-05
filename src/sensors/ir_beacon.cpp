#define IR_BEACON A3
#define IR_RST 4

#define IR_RAW_MAX 512
#define IR_RAW_MIN 1000

#define IR_POLL_FREQ 250
#define IR_AV_LEN 10

#include <Arduino.h>

class IR_Beacon{
    public:
     float mav;
     int value;
     int max;
     void initialize();
     void update(); 
     void reset();
    private:
     float alpha = 0.1;
};

void IR_Beacon::initialize(){
    pinMode(IR_RST, OUTPUT);
    digitalWrite(IR_RST, LOW);
}

void IR_Beacon::reset(){
    mav = 0;
    max = 0;
}

void IR_Beacon::update(){
    static int last_check = 0;
    if (millis() + IR_POLL_FREQ > last_check){
        last_check = millis();
        value = map(
            analogRead(IR_BEACON),
            IR_RAW_MIN, IR_RAW_MAX,
            0, 1023
        );
        mav = (alpha*value)+ (1.0-alpha) * mav;
        if (value > max){
            max = value;
        }
        digitalWrite(IR_RST, HIGH);
    }
    else if (millis() > last_check){
        digitalWrite(IR_RST, LOW);
    }
} 
