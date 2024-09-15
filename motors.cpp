#include "motors.h"
#include "Arduino.h"
#include "sensors.h"
#include <Servo.h>
#include "weight_collection.h"   
#include "statemachine.h"


/* Check whether the speed value to be written is within the maximum
 *  and minimum speed caps. Act accordingly.
 *
 */
Servo motorA, motorB;      // create servo object to control a servo
Servo servoA, servoB;      // create servo object to control a servo

#define LEFT_MOTOR_FULL_FWD 1100
#define RIGHT_MOTOR_FULL_FWD 1900
#define LEFT_MOTOR_FULL_REV 1900
#define RIGHT_MOTOR_FULL_REV 1100
#define MOTOR_STOP 1500
#define LEFT_MOTOR_SLOW_FWD 1150
#define RIGHT_MOTOR_SLOW_FWD 1850
#define LEFT_MOTOR_SLOW_REV 1850
#define RIGHT_MOTOR_SLOW_REV 1150


void motor_setup()
{
    // Pins 7 and 8 correspond to serial 2 on the teensy board
  motorA.attach(LEFT_MOTOR_ADDRESS);
  motorB.attach(RIGHT_MOTOR_ADDRESS);
  // Pins 0 and 1 correspond to serial 1 on the teensy board
  servoA.attach(1);
  servoB.attach(0);
  servoA.write(ServoA_start);
  //servoB.write(170);
  Serial.print("Motor Setup");
} 

void check_speed_limits(/*parameters*/) {
  Serial.println("Check the motor speed limit \n");
}


/* In this section, the motor speeds should be updated/written.
 *It is also a good idea to check whether value to write is valid.
 *It is also a good idea to do so atomically!
 */

void set_motor(int controlA, int controlB) {

  Serial.println("Change the motor speed \n");
  motorA.write(controlA);
  motorB.write(controlB);
  check_speed_limits();

}

void set_servo_mag(int control_servoA) {
  servoA.write(control_servoA);
}

void set_servo_bay(int control_servoB) {
  servoB.write(control_servoB);
}

void drive()
{
  // A Max = 1100, B Max = 1900

    bool R_flag = get_R_flag();
    bool L_flag = get_L_flag();
    bool BL_flag = get_BL_flag();
    bool BR_flag = get_BR_flag();

    State State = get_currentState();
    
    switch (State) {
        case SEARCH: {
            Serial.print(", 0");
            if (R_flag) {  // turn right
                motorA.writeMicroseconds(LEFT_MOTOR_FULL_FWD);  // L motor full forward
                motorB.writeMicroseconds(RIGHT_MOTOR_FULL_REV);  // R motor full reverse
            } else if (L_flag) { // turn left
                motorA.writeMicroseconds(LEFT_MOTOR_FULL_FWD); // L motor full forward
                motorB.writeMicroseconds(RIGHT_MOTOR_FULL_REV); // R motor full reverse
            } else if (BR_flag) { // reverse right
                motorA.writeMicroseconds(LEFT_MOTOR_FULL_REV);  // L motor full reverse
                motorB.writeMicroseconds(RIGHT_MOTOR_SLOW_REV); // R motor slow reverse
            } else if (BL_flag) { // reverse left
                motorA.writeMicroseconds(LEFT_MOTOR_SLOW_REV);  // L motor slow reverse
                motorB.writeMicroseconds(RIGHT_MOTOR_FULL_REV); // R motor slow forward
            } else { // forward
                motorA.writeMicroseconds(LEFT_MOTOR_FULL_FWD);  // L motor full forward
                motorB.writeMicroseconds(RIGHT_MOTOR_FULL_FWD); // R motor full forward
            }
            break;
        }
        case HUNT: {
            Serial.print(", 1000");
            bool HR_flag = get_HR_flag();
            bool HL_flag = get_HL_flag();
            if (HL_flag) { //Hunting right
                motorA.writeMicroseconds(LEFT_MOTOR_FULL_FWD);  // L motor full forward
                motorB.writeMicroseconds(RIGHT_MOTOR_SLOW_FWD); // R motor slow forward
            } else if (HR_flag) { //Hunting left
                motorA.writeMicroseconds(LEFT_MOTOR_SLOW_FWD);  // L motor slow forward
                motorB.writeMicroseconds(RIGHT_MOTOR_FULL_FWD); // R motor full forward
            } else { //Hunting forward
                motorA.writeMicroseconds(LEFT_MOTOR_FULL_FWD);  // L motor full forward
                motorB.writeMicroseconds(RIGHT_MOTOR_FULL_FWD); // R motor full forward
            }
            break;
        }

        case SCAN:
            motorA.writeMicroseconds(LEFT_MOTOR_SLOW_FWD);
            motorB.writeMicroseconds(RIGHT_MOTOR_SLOW_REV);
            break;

        case COLLECT: 
            motorA.writeMicroseconds(LEFT_MOTOR_SLOW_FWD);
            motorB.writeMicroseconds(RIGHT_MOTOR_SLOW_FWD);
            break;

        case IDLE:
            motorA.writeMicroseconds(MOTOR_STOP);
            motorB.writeMicroseconds(MOTOR_STOP);
            break;

        case HOMING:
            // Handle the HOMING state
            break;

        case DROPPING:
            // Handle the DROPPING state
            break;

        default:
            // Handle unknown states
            break;
    }

    Serial.print("\n");
    delay(50); // waits for the servo to get there 
}

