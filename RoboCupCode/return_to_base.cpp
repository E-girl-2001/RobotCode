//************************************
//         return_to_base.cpp       
//************************************

 // This file contains functions used to return to and
 // detect bases

#include "return_to_base.h"
#include "Arduino.h"

// Local definitions
//#define 

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void colour_setup() 
{
  
  if (tcs.begin()) 
  {
    Serial.println("Found sensor");
  } else 
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void colour_read() 
{
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();
}



// Return to home base
void return_to_base(/* Parameters */){
  Serial.println("Returning to base \n");
}

// Detect what base (if any) the robot is above
void detect_base(/* Parameters */){
  Serial.println("Base detected \n");
}

// Unload weights in home base
void unload_weights(/* Parameters */){
  Serial.println("Unloading weights \n");
}


