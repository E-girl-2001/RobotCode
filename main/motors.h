//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 7      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 8     //Pin corresponding to the right dc motor
#define SERVO_ADDRESS_A 1           //Pin corresponding to the servo motor

#define ServoA_start 145
#define ServoA_lift_angle 80
#define ServoA_drop_angle 10

void motor_setup();
void set_motor(int controlA, int controlB);
void set_servo_pos(int control_servoA);
void search_drive(int16_t L_sonic, int16_t R_sonic);
void hunt_drive(int16_t H_tof, int16_t L_tof, int16_t S1_tof, int16_t S2_tof);
void scan_area(int16_t H_tof, int16_t L_tof, int16_t S1_tof, int16_t S2_tof);
void collect_target();
void homing_drive();
void drop_targets();



#endif /* MOTORS_H_ */
