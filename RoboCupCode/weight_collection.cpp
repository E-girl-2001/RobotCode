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



#define long_detect_tolerance 5
#define short_detect_tolerance 15
#define max_front_detection 100
#define max_side_detection 30

bool detected = 0;
bool right_detected = 0;
bool left_detected = 0;
bool inductive = 0;
int weight_counter = 0;


void weight_scan() {
  // check all TOF sensors for weight
  if (currentState == SEARCH) {
    if (longLow < (longHigh - long_detect_tolerance) && (longLow < max_front_detection)) {
      detected = true;
      left_detected = false;
      right_detected = false;
    } else if ((shortLowLeft < (shortHighLeft - short_detect_tolerance)) && !detected && !right_detected && (shortLowLeft < max_side_detection) && (L_sonic > 10)) {
      left_detected = true;
    } else if ((shortLowRight < (shortHighRight - short_detect_tolerance )) && !detected && !left_detected && (shortLowRight < max_side_detection) && (R_sonic > 10)) {
      right_detected = true;
    } else {
      detected = false;
      //currentState = SEARCH;
    }
    print_weight_detection_status();
  }

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

  // Increment weight counter
  weight_counter++;
}

