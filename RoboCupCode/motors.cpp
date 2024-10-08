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

#define side_distance 14
#define front_distance 20

#define TURN_TIMEOUT 15
#define HUNT_TIMEOIUT 5

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



void motor_setup()
{
  motorA.attach(LEFT_MOTOR_ADDRESS);
  motorB.attach(RIGHT_MOTOR_ADDRESS);
  motorV.attach(VIBRATOR_MOTOR_ADDRESS);
  servoA.attach(1);
  servoB.attach(0);
  servoA.write(ServoA_start);
  Serial.print("Motor Setup");
} 


void set_motor() {
  // int errorA = (controlA - currentA);
  // int errorB = (controlB - currentB);
  // int control_effortA = MOTOR_STOP + Kp * errorA;
  // int control_effortB = MOTOR_STOP + Kp * errorB;

  motorA.write(controlA);
  motorB.write(controlB);
  // motorV.write(controlV);

  // currentA = control_effortA;
  // currentB = control_effortB;
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
    if ((longHigh < front_distance) && (R_sonic < L_sonic)) { // reverse right
        Serial.print("REV RIGHT\n");
            controlA = MOTOR_FULL_REV;
            controlB = MOTOR_SLOW_FWD;
        } else if ((longHigh < front_distance) && (L_sonic < R_sonic)) { // reverse left
        Serial.print("REV LEFT\n");
            controlA = MOTOR_SLOW_FWD;
            controlB = MOTOR_FULL_REV;
        }else if ((R_sonic < side_distance)) {  // turn right
            Serial.print("RIGHT\n");
            controlA = MOTOR_FULL_REV;
            controlB = MOTOR_FULL_FWD;
        } else if ((L_sonic < side_distance)) { // turn left
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
        if (shortRight < longLow && shortRight < shortLeft) { //Hunting right
          Serial.print("BANG RIGHT");
          controlA = MOTOR_FULL_FWD;
          controlB = MOTOR_SLOW_FWD;

        } else if (shortLeft < longLow && shortLeft < shortRight) { //Hunting left
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



void scan_drive() {
    controlA = MOTOR_SLOW_FWD;
    controlB = MOTOR_SLOW_REV;
}

void collect_drive() {
    motorA.writeMicroseconds(MOTOR_STOP); 
    motorB.writeMicroseconds(MOTOR_STOP);
}


// void homing_drive() {
//     const int targetDistance = 20;  // Desired distance from the wall in cm
//     const int tolerance = 2;  // Acceptable error tolerance in cm
//     const int headOnThreshold = 20;  // Threshold to detect a head-on collision (in cm)

//     // If the robot detects a wall head-on and needs to turn right (L_sonic < R_sonic)
//     if (longHigh < headOnThreshold && L_sonic < R_sonic) {
//         // Rotate in place to avoid crashing (turning right)
//         Serial.println("Head-on collision detected! Turning right to avoid.");
//         motorA.writeMicroseconds(MOTOR_FULL_REV);  // Turn right by moving left motor forward
//         motorB.writeMicroseconds(MOTOR_SLOW_REV); // And the right motor backward
//     }
//     // If the robot detects a wall head-on and needs to turn left (R_sonic < L_sonic)
//     else if (longHigh < headOnThreshold && R_sonic < L_sonic) {
//         // Rotate in place to avoid crashing (turning left)
//         Serial.println("Head-on collision detected! Turning left to avoid.");
//         motorA.writeMicroseconds(MOTOR_SLOW_REV);  // Turn left by moving left motor backward
//         motorB.writeMicroseconds(MOTOR_FULL_REV);  // And the right motor forward
//     }
//     // If the left sensor detects a wall closer than the target distance
//     else if (L_sonic < targetDistance - tolerance) {
//         // Turn slightly right
//         Serial.println("Adjusting Right - Too close to Left Wall");
//         motorA.writeMicroseconds(MOTOR_FULL_FWD);  // Left motor moves forward slower
//         motorB.writeMicroseconds(MOTOR_SLOW_REV);  // Right motor moves forward at full speed
//     }
//     // If the right sensor detects a wall closer than the target distance
//     else if (R_sonic < targetDistance - tolerance) {
//         // Turn slightly left
//         Serial.println("Adjusting Left - Too close to Right Wall");
//         motorA.writeMicroseconds(MOTOR_SLOW_REV);  // Left motor moves forward at full speed
//         motorB.writeMicroseconds(MOTOR_FULL_FWD);  // Right motor moves forward slower
//     }
//     // If both sensors are detecting similar distances (robot is aligned)
//     else {
//         // Move forward
//         Serial.println("Moving Forward - Aligned");
//         motorA.writeMicroseconds(MOTOR_FULL_FWD);
//         motorB.writeMicroseconds(MOTOR_FULL_FWD);
//     }
// }

void homing_drive() {
    const int targetDistance = 15;  // Desired distance from the wall in cm
    const int tolerance = 3;  // Acceptable error tolerance in cm
    const int headOnThreshold = 20;  // Threshold to detect a head-on collision (in cm)

    // Assuming the TOF sensors are named middleTOF, leftTOF, and rightTOF
    int middleDistance = longHigh; // Front-facing TOF
    int leftDistance =  shortHighLeft;  // Left-facing TOF
    int rightDistance =  shortHighRight;  // Right-facing TOF

    // If the robot detects a wall head-on
    if (middleDistance < headOnThreshold) {
        // Rotate in place to avoid crashing (turning right)
        Serial.println("Head-on collision detected! Turning right to avoid.");
        motorA.writeMicroseconds(MOTOR_SLOW_REV);  // Turn right by moving left motor backward
        motorB.writeMicroseconds(MOTOR_FULL_REV);  // And the right motor backward
    }
    // If the robot detects a wall head-on and needs to turn left
    // else if (middleDistance < headOnThreshold && rightDistance < leftDistance) {
    //     // Rotate in place to avoid crashing (turning left)
    //     Serial.println("Head-on collision detected! Turning left to avoid.");
    //     motorA.writeMicroseconds(MOTOR_FULL_REV);  // Turn left by moving left motor backward
    //     motorB.writeMicroseconds(MOTOR_SLOW_REV);  // And the right motor forward
    // }
    // If the left TOF sensor detects a wall closer than the target distance
    else if (leftDistance < targetDistance - tolerance) {
        // Turn slightly right
        Serial.println("Adjusting Right - Too close to Left Wall");
        motorA.writeMicroseconds(MOTOR_FULL_FWD);  // Left motor moves forward
        motorB.writeMicroseconds(MOTOR_SLOW_REV);  // Right motor moves forward slower
    }
    // If the right TOF sensor detects a wall closer than the target distance
    else if (rightDistance < targetDistance - tolerance) {
        // Turn slightly left
        Serial.println("Adjusting Left - Too close to Right Wall");
        motorA.writeMicroseconds(MOTOR_SLOW_REV);  // Left motor moves slower
        motorB.writeMicroseconds(MOTOR_FULL_FWD);  // Right motor moves forward
    }
    // If both sensors are detecting similar distances (robot is aligned)
    else {
        // Move forward
        Serial.println("Moving Forward - Aligned");
        motorA.writeMicroseconds(MOTOR_FULL_FWD);
        motorB.writeMicroseconds(MOTOR_FULL_FWD);
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

void dropping() {
    // Handle the DROPPING state
  controlA = MOTOR_STOP;
  controlB = MOTOR_STOP; 
  released_the_massive_load = true;
}
