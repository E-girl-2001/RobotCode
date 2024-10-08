/************************************
 *        weight_collection.cpp       *
 *************************************/

 /* This is for functions and tasks for
  *  finding and collecting weights  */
#include "weight_collection.h"
#include "Arduino.h"
#include "motors.h"
#include "sensors.h"               //will need sensor library to detect weights
#include "statemachine.h"



#define long_detect_tolerance 15
#define short_detect_tolerance 15
#define ramp_tolerance 40
#define max_front_detection 100
#define max_side_detection 40
#define COLLECTION_DETECTED_TIMEOUT 15

bool detected = 0;
bool right_detected = 0;
bool left_detected = 0;
bool front_detected = 0;
bool collection_detected = 0;

int left_turn_timer = 0;
int right_turn_timer = 0;

bool ramp = 0;
bool inductive = 0;
int weight_counter = 0;
int detected_timer = 0;
int detected_timeout = 10;
int collection_detected_timer = 0;

bool adjust_right = 0;
bool adjust_left = 0;

void weight_scan() {
  if(longLow < (longHigh - long_detect_tolerance) && (longLow < max_front_detection)) {
    detected = true;
    Serial.print("detected_front\n");
    left_detected = false;
    right_detected = false;
  }

  if(detected) {
    if(longLow < (longHigh - long_detect_tolerance) && longLow < (shortLeft - long_detect_tolerance) && longLow < (shortRight - long_detect_tolerance) && longLow < 3) {
      detected = false;
      adjust_right = false;
      front_detected = false;
      adjust_left = false;
      collection_detected = true;
    } else if((shortRight < longLow) && (shortRight < shortLeft) && shortRight < max_front_detection) {
      adjust_right = true;
      front_detected = false;
      adjust_left = false;
    } else if(shortLeft < longLow && (shortLeft < shortRight) && shortLeft < max_front_detection) {
      adjust_right = false;
      front_detected = false;
      adjust_left = true;
    } else if(longLow < (longHigh - long_detect_tolerance) && (longLow < max_front_detection)) {
      adjust_right = false;
      front_detected = true;
      adjust_left = false;   
    } else {
      adjust_right = false;
      front_detected = false;
      adjust_left = false;
      detected_timer++;
      if (detected_timer > 5) {
        detected = false;
      }     
    }
  } else if(!detected) {
    if(shortLowLeft < (shortHighLeft - short_detect_tolerance) && (shortLowLeft < max_side_detection) && (shortHighLeft > 10) && (right_turn_timer > 20)) {
      left_detected = true;
      left_turn_timer = 0;
    } else if (shortLowRight < (shortHighRight - short_detect_tolerance) && (shortLowRight < max_side_detection) && (shortHighRight > 10) && (left_turn_timer > 20)) {
      right_detected = true;
      right_turn_timer = 0;
    }
  }

  if(collection_detected) {
    collection_detected_timer++;
    if (collection_detected_timer > COLLECTION_DETECTED_TIMEOUT) {
      collection_detected = false;
      collection_detected_timer = 0;
    }
  }

  left_turn_timer++;
  right_turn_timer++;

  // if (currentState != IDLE) {
  //   // check all TOF sensors for weight
  //   // if (longLow < (longHigh - ramp_tolerance) && shortLowLeft < (longHigh - ramp_tolerance) && shortLowRight < (longHigh - ramp_tolerance) && longLow < 30 && longLow > 10) {
  //   //   currentState = RAMP;
  //   if (longLow < (longHigh - long_detect_tolerance) && (longLow < max_front_detection)) {
  //     front_detected = true;
  //     detected = true;
  //     detected_timer = 0;
  //     left_detected = false;
  //     right_detected = false;
  //     Serial.print("detected\n");
  //   } else if ((shortLowLeft < (shortHighLeft - short_detect_tolerance))&& (shortLowLeft < max_side_detection) && (L_sonic > 10) && right_turn_timer > 20) {
  //     left_detected = true;
  //     right_detected = false;
  //     front_detected = false;
  //     left_turn_timer = 0;
  //     detected_timer = 0;
  //   } else if ((shortLowRight < (shortHighRight - short_detect_tolerance )) && (shortLowRight < max_side_detection) && (R_sonic > 10) && !left_detected && left_turn_timer > 20) {
  //     right_detected = true;
  //     left_detected = false;
  //     front_detected = false;
  //     right_turn_timer = 0;
  //     detected_timer = 0;
  //   } else {
  //     detected_timer++;
  //     if (detected_timer > detected_timeout) {
  //       detected = false;
  //     }
  //     //currentState = SEARCH;
  //   print_weight_detection_status();
  //   left_turn_timer++;
  //   right_turn_timer++;
  //   }
  //}


  // Priotises the right side turning arbitarily if both sides are detected
  

  
  
}

void print_weight_detection_status() {
  Serial.print("Detected: ");
  Serial.print(detected);
  Serial.print(" Left: ");
  Serial.print(left_detected);
  Serial.print(" Right: ");
  Serial.print(right_detected);
  Serial.print(" Counter: ");
  Serial.print(weight_counter);
  Serial.print("\n");
}


void weight_collect()
{
  control_servoA = ServoA_start + ServoA_drop_angle;
  set_servo_mag(control_servoA);
  digitalWrite(magnet_pin, HIGH);
  delay(500);
  control_servoA = ServoA_start - ServoA_lift_angle;
  set_servo_mag(control_servoA);
  delay(2000);
  digitalWrite(magnet_pin, LOW);
  control_servoA = ServoA_start;
  set_servo_mag(control_servoA);
  delay(2000);

  // Increment weight counter
  //weight_counter++;
}

void drop_weight()
{
  control_servoB = ServoB_start + ServoB_travel_angle;
  set_servo_bay(control_servoB);
  delay(2000);
  control_servoB = ServoB_start;
  set_servo_bay(control_servoB);
}

