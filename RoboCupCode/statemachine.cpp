
#include "statemachine.h"
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"

#define SCAN_LENGTH 10
#define SCAN_FREQUENCY 10
#define HUNT_LENGTH 10
#define VIBRATE_LENGTH 4

// State Variables
int currentState = IDLE;

// Button to activate robot
const int activateButton = 20; 
bool start_robot = 0;
bool vibrate = 0;

// Timers to trigger scanning
int scan_freq_time = 0;
int scan_length_time = 0;
int hunt_length_time = 0;
int vibrate_length_time = 0;

void update_state() {

  // if (!read_limit()) {
  //   Serial.print("Limit\n");
  //   currentState = COLLECT;
  // }
  print_state();
  switch (currentState) {

    case SEARCH:
        // scan_freq_timer();
        search_drive();
        if (detected || right_detected || left_detected) {
          currentState = HUNT;
          hunt_length_time = 0;
        } else 
        // if (scan_freq_time == 0) {
        //   scan_length_time = 1;
        //   currentState = SCAN;
        // } 
        break;


    case HUNT:
        hunt_drive();
        // vibrate_length_timer();
        //Cancel out of HUNT after set period or if weight is detected
        // if (hunt_length_time > 10) {
        //   currentState = SEARCH;
        //   hunt_length_time = 0;

        // } else {
        //   //hunt_length_time++;
        // }

        if (!detected && !left_detected && !right_detected) {
          currentState = SEARCH;
        }

        break;


    case SCAN:
        scan_length_timer();
        if (scan_length_time == 0) {
          scan_freq_time = 1;
          currentState = SEARCH;
        } else {
          scan_drive();
        }
        break;

    case COLLECT:
      collect_drive();

      weight_collect();
      weight_counter++;
      Serial.print(weight_counter);
      Serial.print("\n");


      // if (weight_counter == 3) {
      //   currentState = HOMING;
      // } else {
      //   currentState = SEARCH;
      // }
      currentState = SEARCH;
      break;

    case RAMP:
      ramp_drive();
      currentState = SEARCH;
      break;

    case HOMING:
        // Handle the HOMING state
        homing_drive();
        // Serial.print("Homing\n");
        // Unload weights only if we are back at home base after having left
        if (isOnHomeBase && leftHomeBase) {
          leftHomeBase = false;  // Reset flag since we've just unloaded
          currentState = DROPPING;
        } 
        break;

    case DROPPING:
        // Handle the DROPPING state
        // Serial.print("Dropping\n");
        drop_weight();
        if (released_the_massive_load){
          released_the_massive_load = false;
          currentState = SEARCH;
        }
        break;
    case IDLE:
    // Handle the IDLE state
    activate_idle();
    idle_drive();
    break;

    default:
        // Handle unknown states
        break;
  }
}

void print_state() {
  Serial.print("STATE: ");
  if (currentState == IDLE) {
    Serial.print("IDLE\n");
  } else if (currentState == SEARCH) {
    Serial.print("SEARCH\n");
  } else if (currentState == HUNT) {
    Serial.print("HUNT\n");
  } else if (currentState == SCAN) {
    Serial.print("SCAN\n");
  } else if (currentState == COLLECT) {
    Serial.print("COLLECT\n");
  } else if (currentState == HOMING) {
    Serial.print("HOMING\n");
  } else if (currentState == DROPPING) {
    Serial.print("DROPPING\n");
  } else if (currentState == RAMP) {
    Serial.print("RAMP\n");
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

// void vibrate_length_timer() {
//   vibrate_length_time++;
//   if(vibrate_length_time == VIBRATE_LENGTH) {
//     vibrate_length_time = 0;
//     vibrate = !vibrate;
//   }
//   if (vibrate) {
//     controlV = 1900;
//   } else {
//     controlV = 1500;
//   }
// }


void activate_idle() {
    bool debounce = start_robot;
    delay(50);
    //Serial.print("check\n");
    start_robot = analogRead(activateButton);
    if(start_robot && debounce && currentState == IDLE) {
      Serial.print("activate\n");
      currentState = SEARCH;
      delay(1000); //prevents going back into idle
    } else if(start_robot && debounce && currentState != IDLE) {
      Serial.print("Deactivate\n");
      currentState = IDLE;
    }
}