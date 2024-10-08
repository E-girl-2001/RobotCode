// motors.h
#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 29
#define RIGHT_MOTOR_ADDRESS 28
#define VIBRATOR_MOTOR_ADDRESS 8
#define MIN_SPEED_CAP 1
#define MAX_SPEED_CAP 1

#define ServoA_start 142
#define ServoA_lift_angle 85
#define ServoA_drop_angle 10

#define ServoB_start 35
#define ServoB_travel_angle 120


// #define ServoB_start 145
// #define ServoB_lift_angle 80
// #define ServoB_drop_angle 10


extern int L_sonic, R_sonic, H_tof, L_tof, S1_tof, S2_tof;
extern bool R_flag, L_flag, HR_flag, HL_flag, BL_flag, BR_flag;
extern bool released_the_massive_load;

extern int controlA; // control signal for motor A
extern int controlB; // control signal for motor B
extern int controlV;
extern int control_servoA;
extern int control_servoB;
// extern int control_servoB;

void motor_setup();
void set_motor();
void set_servo_mag(int control_servoA);
void set_servo_bay(int control_servoB);
void idle_drive();
void search_drive();
void hunt_drive();
void scan_drive();
void collect_drive();
void homing_drive();
void dropping();
void ramp_drive();
void drop_weight(void);

#endif /* MOTORS_H_ */