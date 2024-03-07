#define IR_RIGHT    A0
#define IR_MID      A1
#define IR_LEFT     A2
#define IR_THRESHOLD 600 //550
#define IR_MIN       250 //300
#define IR_MAX       600 //550
#include <Arduino.h>

static volatile bool ir_left_on_line = false;
static volatile int ir_left_triggers = 0;
static volatile bool ir_right_on_line = false;
static volatile int ir_right_triggers = 0;
static volatile bool ir_mid_on_line = false;
static volatile int ir_mid_triggers = 0;


int ir_left() {return analogRead(IR_LEFT);}
int ir_middle() {return analogRead(IR_MID);}
int ir_right() {return analogRead(IR_RIGHT);}

void update_ir_states() {
    // Left
    if (ir_left_on_line == false && ir_left() < IR_MIN) {ir_left_on_line = true;ir_left_triggers++;}
    else if (ir_left_on_line == true && ir_left() > IR_MAX) {ir_left_on_line = false;ir_left_triggers++;}
    // Middle
    if (ir_mid_on_line == false && ir_middle() < IR_MIN) {ir_mid_on_line = true;ir_mid_triggers++;}
    else if (ir_mid_on_line == true && ir_middle() > IR_MAX) {ir_mid_on_line = false;ir_mid_triggers++;}
    // Right
    if (ir_right_on_line == false && ir_right() < IR_MIN) {ir_right_on_line = true;ir_right_triggers++;}
    else if (ir_right_on_line == true && ir_right() > IR_MAX) {ir_right_on_line = false;ir_right_triggers++;}
}

void reset_ir_triggers() {
    ir_left_triggers = 0;
    ir_mid_triggers = 0;
    ir_right_triggers = 0;
}

bool line_detected(uint8_t ir_sensor) {return analogRead(ir_sensor) < IR_THRESHOLD;}
