//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

#define SX1509_ADDRESS  0x3E
#define SX1509_ADDRESS_2  0x3F // SX1509 I2C address


#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

#define inductor_pin 14
#define magnet_pin 24

// Local definitions
int16_t R_sonic, L_sonic, H_tof, L_tof, S1_tof, S2_tof;

void ultrasonic_setup(void);
void tof_setup(void);
void pick_up_setup(void);
long microsecondsToCentimeters(void);

void ultrasonic_read(void);
void tof_read(void);
bool read_inductive(void);

bool check_tof_for_weights(int16_t *H_tof, int16_t *L_tof); 

#endif /* SENSORS_H_ */
