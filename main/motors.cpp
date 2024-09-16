#include "motors.h"
#include "Arduino.h"
#include "sensors.h"
#include <Servo.h>
#include "weight_collection.h"   

Servo motorA, motorB;      // create servo object to control a servo
Servo servoA, servoB;    // create servo object to control a servo

#define MOTOR_STOP 1500
#define MOTOR_FULL_FWD 1900
#define MOTOR_FULL_REV 1100
#define MOTOR_SLOW_FWD 1600
#define MOTOR_SLOW_REV 1400

void motor_setup()
{
  motorA.attach(LEFT_MOTOR_ADDRESS);
  motorB.attach(RIGHT_MOTOR_ADDRESS);
  servoA.attach(SERVO_ADDRESS_A);
  servoA.write(ServoA_start);
  Serial.print("Motor Setup");
} 

void set_motor(int controlA, int controlB) 
{
  motorA.write(controlA);
  motorB.write(controlB);
}

void set_servo_pos(int control_servoA) {
  servoA.write(control_servoA);
}

void search_drive(int16_t L_sonic, int16_t R_sonic)
{
  if (L_sonic > side_distance && R_sonic > side_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_FULL_FWD);
  }
  else if (L_sonic < side_distance && R_sonic > side_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_SLOW_FWD);
  }
  else if (L_sonic > side_distance && R_sonic < side_distance)
  {
    set_motor(MOTOR_SLOW_FWD, MOTOR_FULL_FWD);
  }
  else if (L_sonic < side_distance && R_sonic < side_distance)
  {
    set_motor(MOTOR_FULL_REV, MOTOR_FULL_REV);
  }
}

void hunt_drive(int16_t H_tof, int16_t L_tof, int16_t S1_tof, int16_t S2_tof)
{
  if (H_tof > front_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_FULL_FWD);
  }
  else if (H_tof < front_distance && L_tof > side_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_SLOW_FWD);
  }
  else if (H_tof < front_distance && L_tof < side_distance && S1_tof > side_distance)
  {
    set_motor(MOTOR_SLOW_FWD, MOTOR_FULL_FWD);
  }
  else if (H_tof < front_distance && L_tof < side_distance && S1_tof < side_distance && S2_tof > side_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_SLOW_FWD);
  }
  else if (H_tof < front_distance && L_tof < side_distance && S1_tof < side_distance && S2_tof < side_distance)
  {
    set_motor(MOTOR_FULL_REV, MOTOR_FULL_REV);
  }
}

void scan_area(int16_t H_tof, int16_t L_tof, int16_t S1_tof, int16_t S2_tof)
{
  if (H_tof > front_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_FULL_FWD);
  }
  else if (H_tof < front_distance && L_tof > side_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_SLOW_FWD);
  }
  else if (H_tof < front_distance && L_tof < side_distance && S1_tof > side_distance)
  {
    set_motor(MOTOR_SLOW_FWD, MOTOR_FULL_FWD);
  }
  else if (H_tof < front_distance && L_tof < side_distance && S1_tof < side_distance && S2_tof > side_distance)
  {
    set_motor(MOTOR_FULL_FWD, MOTOR_SLOW_FWD);
  }
  else if (H_tof < front_distance && L_tof < side_distance && S1_tof < side_distance && S2_tof < side_distance)
  {
    set_motor(MOTOR_FULL_REV, MOTOR_FULL_REV);
  }
}

void collect_target(void)
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
    set_state(1);
}

void homing_drive()
{
  continue;
}

void drop_targets()
{
  continue;
}
