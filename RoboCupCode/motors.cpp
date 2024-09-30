#include "motors.h"
#include "Arduino.h"
#include "sensors.h"
#include <Servo.h>
#include "weight_collection.h"   
#include "statemachine.h"


Servo motorA, motorB;      // create servo object to control a servo
Servo servoA, servoB;      // create servo object to control a servo

#define MOTOR_FULL_FWD 1900
#define MOTOR_FULL_REV 1100
#define MOTOR_STOP 1500 
#define MOTOR_SLOW_FWD 1600
#define MOTOR_SLOW_REV 1400

int controlA = MOTOR_STOP; // control signal for motor A
int controlB = MOTOR_STOP; // control signal for motor B
int control_servoA = ServoA_start;
int control_DropMotor = 0;


void motor_setup()
{
  motorA.attach(LEFT_MOTOR_ADDRESS);
  motorB.attach(RIGHT_MOTOR_ADDRESS);
  servoA.attach(1);
  servoA.write(ServoA_start);
  Serial.print("Motor Setup");
} 


void set_motor() {
  motorA.write(controlA);
  motorB.write(controlB);
}

void set_servo_mag(int control_servoA) {
  servoA.write(control_servoA);
}

void set_servo_bay(int control_servoB) {
  servoB.write(control_servoB);
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
}

void scan_drive() {
    controlA = MOTOR_SLOW_FWD;
    controlB = MOTOR_SLOW_REV;
}

void collect_drive() {
    controlA = MOTOR_SLOW_FWD;
    controlB = MOTOR_SLOW_FWD;
}


void homing_drive() {
    // Handle the HOMING state
}


void dropping() {
    // Handle the DROPPING state

}
