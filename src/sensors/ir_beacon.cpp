#define IR_BEACON A3
#define IR_RST 4

#define IR_RAW_MIN 561
#define IR_RAW_MAX 1000

#define IR_POLL_INT 75
#define IR_AV_LEN 10

#include <Arduino.h>

class IR_Beacon{
    public:
        int raw;
        int value;

        void initialize();
        void update(); 
};

void IR_Beacon::initialize(){
    pinMode(IR_RST, OUTPUT);
    pinMode(IR_BEACON, INPUT);
    digitalWrite(IR_RST, LOW);
    analogReference(DEFAULT);
}

void IR_Beacon::update(){
    static volatile unsigned long last_check = 0;
    if (millis() > last_check + IR_POLL_INT){
        last_check = millis();
        raw = analogRead(IR_BEACON);
        value = map(
            raw,
            IR_RAW_MIN, IR_RAW_MAX,
            0, 1023
        );
        digitalWrite(IR_RST, HIGH);
    }
    else {
        digitalWrite(IR_RST, LOW);
    }
}
