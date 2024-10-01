//************************************
//         return_to_base.h    
//************************************


#ifndef RETURN_TO_BASE_H_
#define RETURN_TO_BASE_H_

#include <Wire.h>
#include <Adafruit_TCS34725.h>


extern bool isOnHomeBase;
extern bool leftHomeBase;

void colour_setup();
void colour_read();
void calibrateHomeBase();
void detect_base();

// Return to home base
void return_to_base();

// Unload weights in home base
void unload_weights();




#endif /* RETURN_TO_BASE_H_ */