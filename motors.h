//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 7      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 8     //Pin corresponding to the right dc motor
#define MIN_SPEED_CAP 1           //Set the minimum speed value that can be written to the motors
#define MAX_SPEED_CAP 1           //Set the maximum speed value that can be written to the motors

#define ServoA_start 145
#define ServoA_lift_angle 80
#define ServoA_drop_angle 10

void motor_setup();
void check_speed_limits(/*parameters*/);
void set_motor(int controlA, int controlB);
void set_servo_mag(int control_servoA);
void set_servo_bay(int control_servoB);
void drive();


#endif /* MOTORS_H_ */
