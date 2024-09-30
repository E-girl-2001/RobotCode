
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "Arduino.h"
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h"



// ------ State Variables ------ //
enum State {
  IDLE = 0, SEARCH = 1, HUNT = 2, SCAN = 3, COLLECT = 4, HOMING = 5, DROPPING = 6 
};

extern int currentState;

void update_state();
void hunt_length_timer();
void scan_freq_timer();
void scan_length_timer();
void activate_idle();




#endif // STATEMACHINE_H