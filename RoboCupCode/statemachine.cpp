
#include "statemachine.h"
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"

#define SCAN_LENGTH 100
#define SCAN_FREQUENCY 1000
#define HUNT_LENGTH 1000

// State Variables
int currentState = IDLE;

// Button to activate robot
const int activateButton = 20; 
bool start_robot = 0;

// Timers to trigger scanning
int scan_freq_time = 0;
int scan_length_time = 0;
int hunt_length_time = 0;

void update_state() {
  // Serial.print("STATE: ");
  // Serial.print(currentState);
  // Serial.print("\n");
  switch (currentState) {
    case IDLE:
        // Handle the IDLE state
        activate_idle();
        idle_drive();
        break;


    case SEARCH:
        scan_freq_timer();
        search_drive();
        Serial.print("Searching\n");
        if (detected) {
          currentState = HUNT;
        } else if (scan_freq_time == 0) {
          currentState = SCAN;
        } 
        break;


    case HUNT:
        hunt_drive();
        Serial.print("Hunting\n");
        // Cancel out of HUNT after set period or if weight is detected
        if (detected) {
          currentState = COLLECT;
        } else if (hunt_length_time == 0) {
          currentState = SEARCH;
        }

        break;


    case SCAN:
        if (scan_length_time == 0) {
          currentState = SEARCH;
        } else {
          scan_drive();
        }
        break;


    case COLLECT:
        collect_drive();
        weight_collect();
        Serial.print("Collecting\n");
        if (weight_counter == 3) {
          currentState = HOMING;
        } else {
          currentState = SEARCH;
        }
        break;


    case HOMING:
        // Handle the HOMING state
        homing_drive();
        Serial.print("Homing\n");
        // Unload weights only if we are back at home base after having left
        if (isOnHomeBase && leftHomeBase) {
          leftHomeBase = false;  // Reset flag since we've just unloaded
          currentState = DROPPING;
        } 
        break;

    case DROPPING:
        // Handle the DROPPING state
        Serial.print("Dropping\n");
        dropping();
        if (released_the_massive_load){
          released_the_massive_load = false;
          currentState = SEARCH;
        }
        break;


    default:
        // Handle unknown states
        break;
  }
}


void scan_freq_timer() {
  scan_freq_time++;
  if(scan_freq_time == SCAN_FREQUENCY) {
    scan_freq_time = 0;
  }
}

void scan_length_timer() {
  scan_length_time++;
  if(scan_length_time == SCAN_LENGTH) {
    scan_length_time = 0;
  }
}

void hunt_length_timer() {
  hunt_length_time++;
  if(hunt_length_time == HUNT_LENGTH) {
    hunt_length_time = 0;
  }
}



void activate_idle() {
    bool debounce = start_robot;
    delay(20);
    start_robot = analogRead(activateButton);
    if(start_robot && debounce == 1 && currentState != IDLE) {
      currentState = IDLE;
    } else if(start_robot && debounce == 1 && currentState == IDLE) {
      currentState = SEARCH;
    }
}
