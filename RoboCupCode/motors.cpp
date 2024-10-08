#include "motors.h"
#include "Arduino.h"
#include "sensors.h"
#include <Servo.h>
#include "weight_collection.h"   
#include "statemachine.h"


Servo motorA, motorB, motorV;      // create servo object to control a servo
Servo servoA, servoB;      // create servo object to control a servo

#define MOTOR_FULL_FWD 1900
#define MOTOR_FULL_REV 1100
#define MOTOR_STOP 1500 
#define MOTOR_SLOW_FWD 1800
#define MOTOR_SLOW_REV 1200
#define MOTOR_TURN_FWD 1850
#define MOTOR_TURN_REV 1150

#define side_distance 20
#define front_distance 20

#define TURN_TIMEOUT 8

int controlA = MOTOR_STOP; // control signal for motor A
int controlB = MOTOR_STOP; // control signal for motor B
int controlV = MOTOR_STOP;
int control_servoA = ServoA_start;
int control_servoB = ServoB_start;

int currentA = controlA;
int currentB = controlB;
int Kp = 2;


int turn_timer = 0;
bool released_the_massive_load = false;

// Global variables to manage timing and state
unsigned long dropStartTime = 0;
const int gateOpenTime = 5000;  // Gate remains open for 5 seconds
const int motorDelay = 1000;  // Delay between motor direction changes
bool isDroppingWeight = false;
bool motorDirectionForward = true;
unsigned long lastMotorChangeTime = 0;


void motor_setup()
{
  motorA.attach(LEFT_MOTOR_ADDRESS);
  motorB.attach(RIGHT_MOTOR_ADDRESS);
  motorV.attach(VIBRATOR_MOTOR_ADDRESS);
  servoA.attach(1);
  servoB.attach(0);
  servoA.write(ServoA_start);
  servoB.write(ServoB_start);
  Serial.print("Motor Setup");
} 


void set_motor() {

  motorA.write(controlA);
  motorB.write(controlB);
}

void set_servo_mag(int control_servoA) {
  servoA.write(control_servoA);
}

void set_servo_drop(int control_servoB) {
  servoB.write(control_servoB);
}

void set_servo_bay(int control_servoB) {
  servoB.write(control_servoB);
}

void idle_drive(){
  controlA = MOTOR_STOP;
  controlB = MOTOR_STOP;
}

void search_drive(){
    if ((longHigh < front_distance) && ((R_sonic < L_sonic))) { // reverse right
        Serial.print("REV RIGHT\n");
            controlA = MOTOR_FULL_REV;
            controlB = MOTOR_SLOW_FWD;
        } else if ((longHigh < front_distance) && (L_sonic < R_sonic)) { // reverse left
        Serial.print("REV LEFT\n");
            controlA = MOTOR_SLOW_FWD;
            controlB = MOTOR_FULL_REV;
        }else if ((R_sonic < side_distance) || (shortHighRight < side_distance)) {  // turn right
            Serial.print("RIGHT\n");
            controlA = MOTOR_FULL_REV;
            controlB = MOTOR_FULL_FWD;
        } else if ((L_sonic < side_distance)|| (shortHighLeft < side_distance)) { // turn left
            Serial.print("LEFT\n");
            controlA = MOTOR_FULL_FWD;
            controlB = MOTOR_FULL_REV;
        } else { // forward
          Serial.print("FWD\n");
          if(collection_detected) {
            controlA = MOTOR_SLOW_FWD;
            controlB = MOTOR_SLOW_FWD;            
          } else {
            controlA = MOTOR_FULL_FWD;
            controlB = MOTOR_FULL_FWD;
          }
        }
}

void hunt_drive() {
    
    if (detected) {
        if (adjust_right) { //Hunting right
          Serial.print("BANG RIGHT");
          controlA = MOTOR_FULL_FWD;
          controlB = MOTOR_SLOW_FWD;

        } else if (adjust_left) { //Hunting left
          Serial.print("BANG LEFT");
          controlA = MOTOR_SLOW_FWD;
          controlB = MOTOR_FULL_FWD;

        } else { //Hunting forward
          controlA = MOTOR_SLOW_FWD;
          controlB = MOTOR_SLOW_FWD;

        }
    } else if (turn_timer > TURN_TIMEOUT) {
      Serial.print("TURN TIMEOUT");
        turn_timer = 0;
        left_detected = false;
        right_detected = false;
        currentState = SEARCH;
    }
    else if (right_detected) {
        // turn right
        Serial.print("HUNTING RIGHT\n");
        controlA = MOTOR_TURN_FWD;
        controlB = MOTOR_TURN_REV;
        turn_timer++;
        Serial.print("Turn timer: ");
        Serial.print(turn_timer);
        Serial.print("\n");
    } else if (left_detected) {
        // turn left
        Serial.print("HUNTING LEFT\n");
        controlA = MOTOR_TURN_REV;
        controlB = MOTOR_TURN_FWD;
        turn_timer++;
        Serial.print("Turn timer: ");
        Serial.print(turn_timer);
        Serial.print("\n");
    } else {
        // move forward
      turn_timer = 0;
      currentState = SEARCH;
    }
}

void fuckoff_drive() {
  motorA.writeMicroseconds(MOTOR_FULL_REV); 
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
}

void scan_drive() {
    controlA = MOTOR_SLOW_FWD;
    controlB = MOTOR_SLOW_REV;
}

void collect_drive() {
    motorA.writeMicroseconds(MOTOR_STOP); 
    motorB.writeMicroseconds(MOTOR_STOP);
}


void homing_drive() {
    const int targetDistance = 15;  // Desired distance from the wall in cm
    const int tolerance = 2;  // Acceptable error tolerance in cm
    const int headOnThreshold = 20;  // Threshold to detect a head-on collision (in cm)

    // TOF sensor readings
    int middleDistance = longHigh;   // Front-facing TOF
    int leftDistance = shortHighLeft;   // Left-facing TOF
    int rightDistance = shortHighRight;   // Right-facing TOF

    // Proportional control factor
    int maxSpeed = 1900;  // Maximum motor speed
    int minSpeed = 1500;  // Minimum motor speed for smooth motion
    float adjustmentFactor = 9;  // Proportional control scaling factor

    // Calculate speed adjustments based on distance from the wall
    int leftSpeedAdjustment = (targetDistance - leftDistance) * adjustmentFactor;
    int rightSpeedAdjustment = (targetDistance - rightDistance) * adjustmentFactor;

    // Constrain motor speeds between minSpeed and maxSpeed
    leftSpeedAdjustment = constrain(minSpeed + leftSpeedAdjustment, minSpeed, maxSpeed);
    rightSpeedAdjustment = constrain(minSpeed + rightSpeedAdjustment, minSpeed, maxSpeed);

    // If the robot detects a wall head-on
    if (middleDistance < headOnThreshold) {
        // Turn sharply to avoid hitting the wall head-on
        if (leftDistance < rightDistance) {
            Serial.println("Head-on collision detected! Turning sharply right.");
            motorA.writeMicroseconds(MOTOR_FULL_FWD);  // Left motor moves forward
            motorB.writeMicroseconds(MOTOR_FULL_REV);  // Right motor moves backward
        } else {
            Serial.println("Head-on collision detected! Turning sharply left.");
            motorA.writeMicroseconds(MOTOR_FULL_REV);  // Left motor moves backward
            motorB.writeMicroseconds(MOTOR_FULL_FWD);  // Right motor moves forward
        }
    }
    // If the left TOF sensor detects a wall closer than the target distance
    else if (leftDistance < targetDistance - tolerance) {
        // Turn slightly right, adjust proportional to distance from the wall
        Serial.println("Adjusting Right - Too close to Left Wall");
        motorA.writeMicroseconds(leftSpeedAdjustment);  // Left motor adjusts based on proximity
        motorB.writeMicroseconds(MOTOR_FULL_REV);  // Right motor moves at full speed
    }
    // If the right TOF sensor detects a wall closer than the target distance
    else if (rightDistance < targetDistance - tolerance) {
        // Turn slightly left, adjust proportional to distance from the wall
        Serial.println("Adjusting Left - Too close to Right Wall");
        motorA.writeMicroseconds(MOTOR_FULL_REV);  // Left motor moves at full speed
        motorB.writeMicroseconds(rightSpeedAdjustment);  // Right motor adjusts based on proximity
    }
    // If both sensors are detecting similar distances (robot is aligned)
    else {
        // Move forward
        Serial.println("Moving Forward - Aligned");
        motorA.writeMicroseconds(maxSpeed);
        motorB.writeMicroseconds(maxSpeed);
    }
}


void ramp_drive() {
  if ((R_sonic < L_sonic)) { // reverse right
      Serial.print("RAMP RIGHT\n");
      motorA.writeMicroseconds(MOTOR_FULL_REV);
      motorB.writeMicroseconds(MOTOR_FULL_FWD);
  } else if ((L_sonic < R_sonic)) { // reverse left
    Serial.print("RAMP LEFT\n");
    motorA.writeMicroseconds(MOTOR_FULL_FWD);
    motorA.writeMicroseconds(MOTOR_FULL_REV);
  }
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_FWD); 
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
  delay(1000);
  ramp = false;
}

void drop_weight()
{
  control_servoB = ServoB_start + ServoB_travel_angle;
  set_servo_bay(control_servoB);
  Serial.print(control_servoB);

  motorA.writeMicroseconds(MOTOR_FULL_FWD);
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_REV);
  motorB.writeMicroseconds(MOTOR_FULL_REV);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_FWD);
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_REV);
  motorB.writeMicroseconds(MOTOR_FULL_REV);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_FWD);
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_REV);
  motorB.writeMicroseconds(MOTOR_FULL_REV);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_FWD);
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_REV);
  motorB.writeMicroseconds(MOTOR_FULL_REV);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_FWD);
  motorB.writeMicroseconds(MOTOR_FULL_FWD);
  delay(500);
  motorA.writeMicroseconds(MOTOR_FULL_REV);
  motorB.writeMicroseconds(MOTOR_FULL_REV);
  delay(500);
  motorA.writeMicroseconds(1500);
  motorB.writeMicroseconds(1500);
  
  control_servoB = ServoB_start;
  set_servo_bay(control_servoB);
  Serial.print(control_servoB);
  released_the_massive_load = true;
}



