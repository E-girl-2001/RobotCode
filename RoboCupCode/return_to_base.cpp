//************************************
//         return_to_base.cpp       
//************************************

 // This file contains functions used to return to and
 // detect bases

#include "return_to_base.h"
#include "Arduino.h"


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Define the color ranges for Blue and Green (home base regions)
int blueClearMin = 540, blueClearMax = 580;
int blueRedMin = 110, blueRedMax = 130;
int blueGreenMin = 170, blueGreenMax = 190;
int blueBlueMin = 235, blueBlueMax = 260;

int greenClearMin = 360, greenClearMax = 450;
int greenRedMin = 110, greenRedMax = 140;
int greenGreenMin = 160, greenGreenMax = 190;
int greenBlueMin = 75, greenBlueMax = 100;

// Define the ranges for the black floor (avoid false positives)
int blackClearMin = 230, blackClearMax = 270;
int blackRedMin = 90, blackRedMax = 110;
int blackGreenMin = 70, blackGreenMax = 90;
int blackBlueMin = 55, blackBlueMax = 75;

// Variables to track home base color
bool homeBaseIsGreen = false;
bool enemyBaseIsGreen = false;
bool homeBaseIsBlue = false;
bool enemyBaseIsBlue = false;
bool homeBaseDetected = false;
bool leftHomeBase = false;
bool isOnHomeBase= true;

// Function to check if sensor is detecting a blue area
bool isBlue(int clear, int red, int green, int blue) {
  return (clear >= blueClearMin && clear <= blueClearMax &&
          red >= blueRedMin && red <= blueRedMax &&
          green >= blueGreenMin && green <= blueGreenMax &&
          blue >= blueBlueMin && blue <= blueBlueMax);
}

// Function to check if sensor is detecting a green area
bool isGreen(int clear, int red, int green, int blue) {
  return (clear >= greenClearMin && clear <= greenClearMax &&
          red >= greenRedMin && red <= greenRedMax &&
          green >= greenGreenMin && green <= greenGreenMax &&
          blue >= greenBlueMin && blue <= greenBlueMax);
}

// Function to check if the sensor is detecting black floor (avoid opening)
bool isBlack(int clear, int red, int green, int blue) {
  return (clear >= blackClearMin && clear <= blackClearMax &&
          red >= blackRedMin && red <= blackRedMax &&
          green >= blackGreenMin && green <= blackGreenMax &&
          blue >= blackBlueMin && blue <= blackBlueMax);
}


void colour_setup() 
{
  if (!tcs.begin(0x29, &Wire1)) {
    Serial.println("Failed to find TCS34725 chip");
    while (1); // halt further execution
  }
  // Initial Calibration Phase to detect home base
  calibrateHomeBase();
  Serial.println("Colour Setup");
}

void colour_read() 
{
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(120);  // takes 50ms to read 
  
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

void calibrateHomeBase() {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED
  delay(60);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED

  // Check if starting on Green or Blue
  if (isGreen(clear, red, green, blue)) {
    homeBaseIsGreen = true;
    enemyBaseIsBlue = true;
    homeBaseDetected = true;
    Serial.println("Home Base Detected as Green");
  } else if (isBlue(clear, red, green, blue)) {
    homeBaseIsBlue = true;
    enemyBaseIsGreen = true;
    homeBaseDetected = true;
    Serial.println("Home Base Detected as Blue");
  } else {
    homeBaseDetected = false;
    Serial.println("Home Base Not Detected. Check the surface.");
  }
}

// Function to check base periodically (this replaces loop)
void detect_base() {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED
  delay(60);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED

  // Print current color readings
  // Serial.print("Clear: "); Serial.print(clear);
  // Serial.print("\tRed: "); Serial.print(red);
  // Serial.print("\tGreen: "); Serial.print(green);
  // Serial.print("\tBlue: "); Serial.println(blue);

  // Check if we're over the home base (either green or blue)
  isOnHomeBase = (homeBaseIsGreen && isGreen(clear, red, green, blue)) ||
                      (homeBaseIsBlue && isBlue(clear, red, green, blue));

  if (!isOnHomeBase && isBlack(clear, red, green, blue)) {
    // If we leave the home base (but not on black floor), set the flag
    leftHomeBase = true;
    Serial.println("Left the home base.");
    unload_weights();
  } 
 
}

// Return to home base
void return_to_base(/* Parameters */){
  Serial.println("Returning to base \n");
}

// Unload weights in home base
void unload_weights(){
  Serial.println("Unloading weights \n");
}


