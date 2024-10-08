//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

// #define SX1509_ADDRESS  0x3E
#define SX1509_ADDRESS_2  0x3E // SX1509 I2C address


// #define VL53L0X_ADDRESS_START 0x30
// #define VL53L1X_ADDRESS_START 0x35

const int inductor_pin = 14;
const int magnet_pin = 24;
const int limit_pin = 27;

extern long shortLeft, shortRight, shortHighLeft, shortHighRight, shortLowLeft, shortLowRight, longHigh, longLow;
extern int L_sonic, R_sonic;
extern bool HR_flag, HL_flag, BL_flag, BR_flag, R_flag, L_flag;

void ultrasonic_setup();

void tof_setup();

void pick_up_setup();

// Read ultrasonic value
void ultrasonic_read(void);
// Read TOF
void tof_read(void);

void update_flags();

void read_limit();

void read_inductive();

void print_tof();

void print_ultrasonic();

void long_TOF_reinit();

//void read_limit_switch();

// Read infrared value
//void read_infrared(/* Parameters */);

//void read_colour(/* Parameters */);

// Pass in data and average the lot
//void sensor_average(/* Parameters */);

// int get_H_tof();
// int get_L_tof();
// int get_S1_tof();
// int get_S2_tof();
// int get_R_sonic();
// int get_L_sonic();
// int get_L_flag();
// int get_R_flag();
// int get_BL_flag();
// int get_BR_flag();
// int get_HL_flag();
// int get_HR_flag();
// int get_limit_switch();

#endif /* SENSORS_H_ */
