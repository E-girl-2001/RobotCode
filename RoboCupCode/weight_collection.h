
/************************************
 *        weight_collection.h       *
 *************************************/

 /* This header is for functions and tasks for
  *  finding and collecting weights  */

#ifndef WEIGHT_COLLECTION_H_
#define WEIGHT_COLLECTION_H_

#include <Servo.h>                  //control the DC motors
#include <Wire.h>                   //for I2C and SPI

//states for swapping between searching and collecting
#define NO_WEIGHT               0   
#define WEIGHT_FOUND            1

extern bool detected;
extern bool right_detected;
extern bool left_detected;
extern bool ramp;
extern int weight_counter;
extern bool detected;

void weight_collect(void);
void weight_scan(void);
void print_weight_detection_status(void);
void drop_weight(void);


#endif /* WEIGHT_COLLECTION_H_ */
