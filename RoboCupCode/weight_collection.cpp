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



#define long_detect_tolerance 50
#define short_detect_tolerance 50

bool detected = 0;
bool right_detected = 0;
bool left_detected = 0;
bool inductive = 0;
int weight_counter = 0;


void weight_scan() {
  // check all TOF sensors for weight
  if (longLow < (longHigh - long_detect_tolerance)) {
    detected = true;
    left_detected = false;
    right_detected = false;
  } else {
    detected = false;
  }
  // Priotises the right side turning arbitarily if both sides are detected
  if ((shortLowLeft < (shortHighLeft - short_detect_tolerance)) && !detected && !left_detected) {
    right_detected = true;
  }

  if ((shortLowRight < (shortHighRight - short_detect_tolerance )) && !detected) {
    left_detected = true;
  }
  
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

