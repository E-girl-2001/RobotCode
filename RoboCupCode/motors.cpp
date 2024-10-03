#include "motors.h"
#include "Arduino.h"
#include "sensors.h"
#include <Servo.h>
#include "weight_collection.h"   
#include "statemachine.h"


Servo motorA, motorB, motorG;      // create servo object to control a servo
Servo servoA, servoB;      // create servo object to control a servo

#define MOTOR_FULL_FWD 1900
#define MOTOR_FULL_REV 1100
#define MOTOR_STOP 1500 
#define MOTOR_SLOW_FWD 1800
#define MOTOR_SLOW_REV 1200

int controlA = MOTOR_STOP; // control signal for motor A
int controlB = MOTOR_STOP; // control signal for motor B
int controlG = MOTOR_STOP;
int control_servoA = ServoA_start;
int control_DropMotor = 0;
bool released_the_massive_load = false;



void motor_setup()
{
  motorA.attach(LEFT_MOTOR_ADDRESS);
  motorB.attach(RIGHT_MOTOR_ADDRESS);
  motorG.attach(GATE_MOTOR_ADDRESS);
  servoA.attach(1);
  servoA.write(ServoA_start);
  Serial.print("Motor Setup");
} 


void set_motor() {
  motorA.write(controlA);
  motorB.write(controlB);
  motorG.write(controlG);
}

void set_servo_mag(int control_servoA) {
  servoA.write(control_servoA);
}

void set_servo_bay(int control_servoB) {
  servoB.write(control_servoB);
}

void idle_drive(){
  controlA = MOTOR_STOP;
  controlB = MOTOR_STOP;
}

void search_drive(){
    if (BR_flag) { // reverse right
        Serial.print("REV RIGHT\n");
            controlA = MOTOR_FULL_REV;
            controlB = MOTOR_SLOW_FWD;
        } else if (BL_flag) { // reverse left
        Serial.print("REV LEFT\n");
            controlA = MOTOR_SLOW_FWD;
            controlB = MOTOR_FULL_REV;
        }else if (R_flag) {  // turn right
            Serial.print("RIGHT\n");
            controlA = MOTOR_FULL_REV;
            controlB = MOTOR_FULL_FWD;
        } else if (L_flag) { // turn left
            Serial.print("LEFT\n");
            controlA = MOTOR_FULL_FWD;
            controlB = MOTOR_FULL_REV;
        } else { // forward
          Serial.print("FWD\n");
            controlA = MOTOR_FULL_FWD;
            controlB = MOTOR_FULL_FWD;
        }
}

void hunt_drive() {
    
    if (detected) {
        if (HL_flag) { //Hunting right
            motorA.writeMicroseconds(MOTOR_FULL_FWD);  // L motor full forward
            motorB.writeMicroseconds(MOTOR_SLOW_FWD); // R motor slow forward
            controlA = MOTOR_FULL_FWD;
            controlB = MOTOR_SLOW_FWD;

        } else if (HR_flag) { //Hunting left
            motorA.writeMicroseconds(MOTOR_SLOW_FWD);  // L motor slow forward
            motorB.writeMicroseconds(MOTOR_FULL_FWD); // R motor full forward
            controlA = MOTOR_SLOW_FWD;
            controlB = MOTOR_FULL_FWD;

        } else { //Hunting forward
            controlA = MOTOR_FULL_FWD;
            controlB = MOTOR_FULL_FWD;
        }
    }else if (right_detected) {
        // turn right
        controlA = MOTOR_SLOW_REV;
        controlB = MOTOR_FULL_FWD;
    } else if (left_detected) {
        // turn left
        controlA = MOTOR_FULL_FWD;
        controlB = MOTOR_SLOW_REV;
    } else {
        // move forward
      currentState = SEARCH;
    }
}



void scan_drive() {
    controlA = MOTOR_SLOW_FWD;
    controlB = MOTOR_SLOW_REV;
}

void collect_drive() {
    controlA = 1700;
    controlB = 1700;
}


void homing_drive() {
    const int targetDistance = 20;  // Desired distance from the wall in cm
    const int tolerance = 2;  // Acceptable error tolerance in cm
    const int correctionSpeed = 1600;  // Speed to correct direction

    // If the left sensor detects a wall closer than the target distance
    if (L_sonic < targetDistance - tolerance) {
        // Turn slightly right
        Serial.println("Adjusting Right - Too close to Left Wall");
        controlA = MOTOR_SLOW_FWD;   // Left motor moves forward slower
        controlB = MOTOR_FULL_FWD;   // Right motor moves forward at full speed

    // If the right sensor detects a wall closer than the target distance
    } else if (R_sonic < targetDistance - tolerance) {
        // Turn slightly left
        Serial.println("Adjusting Left - Too close to Right Wall");
        controlA = MOTOR_FULL_FWD;   // Left motor moves forward at full speed
        controlB = MOTOR_SLOW_FWD;   // Right motor moves forward slower

    // If both sensors are detecting similar distances (robot is aligned)
    } else {
        // Move forward
        Serial.println("Moving Forward - Aligned");
        controlA = MOTOR_FULL_FWD;
        controlB = MOTOR_FULL_FWD;
    }
}


void dropping() {
    // Handle the DROPPING state
  controlA = MOTOR_STOP;
  controlB = MOTOR_STOP;
  controlG = 1; //Some abritrary value to open the gate and then close the gate 
  released_the_massive_load = true;
}
