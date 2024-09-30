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



#define detect_tolerance 20

bool detected = 0;
bool inductive = 0;
int weight_counter = 0;


void weight_scan()
{
  if(L_tof < (H_tof - detect_tolerance)) {
    detected = true;
  } else {
    detected = false;
  }
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

