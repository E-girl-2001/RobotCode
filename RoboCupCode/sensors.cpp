//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.

#include "sensors.h"
#include "Arduino.h"
#include "statemachine.h"
#include "motors.h"
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

#define side_distance 14
#define front_distance 14

#define BUFFER_SIZE 3

//#define 
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

const byte SX1509_AIO5 = 5;
const byte SX1509_INTERRUPT_PIN = 2; 

const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START_1 0x30
#define VL53L0X_ADDRESS_START_2 0x36
#define VL53L0X_ADDRESS_START_3 0x3A


// The number of sensors in your system.
const uint8_t sensorCount_1 = 4;
const uint8_t sensorCount_2 = 2;
const uint8_t sensorCount_3 = 2;


// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins_1[8] = {0,1,2,3,4,5,6,7};
const uint8_t xshutPins_3[8] = {4,5,6,7};
const uint8_t xshutPins_2[8] = {6,7};

uint8_t x_ROI = 16;
uint8_t y_ROI = 16;

int max_LTOF_range = 60;
int max_STOF_range = 60;

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensors_1[sensorCount_1];
VL53L0X sensors_2[sensorCount_2];
VL53L0X sensors_3[sensorCount_3];

unsigned long lastDebounceTime = 0;   // For debouncing
unsigned long debounceDelay = 50;     // Debounce delay (in ms)


int longLow_reading, longHigh_reading, shortLeft_reading, shortRight_reading;
int shortHighLeft_reading, shortHighRight_reading, shortLowLeft_reading, shortLowRight_reading;


//SETUP
//***********************************************************************

void ultrasonic_setup()
{
  pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(AechoPin, INPUT);
  pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(BechoPin, INPUT);
  Serial.print("US setup\n");

}


void tof_setup()
{
  while (!Serial) {}
  Serial.begin(115200);

  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount_1; i++)
  {
    io.pinMode(xshutPins_1[i], OUTPUT);
    io.digitalWrite(xshutPins_1[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCount_2; i++)
  {
    io.pinMode(xshutPins_2[i], OUTPUT);
    io.digitalWrite(xshutPins_2[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCount_3; i++)
  {
    io.pinMode(xshutPins_3[i], OUTPUT);
    io.digitalWrite(xshutPins_3[i], LOW);
  }


  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount_1; i++)
  {
    io.digitalWrite(xshutPins_1[i], HIGH);
    delay(10);
    sensors_1[i].setTimeout(500);
    if (!sensors_1[i].init())
    {
      Serial.print("1: ");
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    sensors_1[i].setAddress(VL53L0X_ADDRESS_START_1 + i);
    sensors_1[i].startContinuous(50);

  }
  for (uint8_t i = 0; i < sensorCount_2; i++)
  {
    io.digitalWrite(xshutPins_2[i], HIGH);
    delay(10);
    sensors_2[i].setTimeout(500);
    if (!sensors_2[i].init())
    {
      Serial.print("2: ");
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    sensors_2[i].setAddress(VL53L0X_ADDRESS_START_2 + i);
    sensors_2[i].startContinuous(50);


  }
  for (uint8_t i = 0; i < sensorCount_3; i++)
  {
    io.digitalWrite(xshutPins_3[i], HIGH);
    delay(10);
    sensors_3[i].setTimeout(500);
    if (!sensors_3[i].init())
    {\
      Serial.print("3: ");
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    sensors_3[i].setAddress(VL53L0X_ADDRESS_START_3 + i);
    sensors_3[i].startContinuous(50);

    // sensors_3[i].setROISize(x_ROI, y_ROI);
    // sensors_3[i].setROICenter(199);
    // sensors_3[i].setDistanceMode(VL53L1X::Long);
    // sensors_3[i].setMeasurementTimingBudget(250000);
    // sensors_3[i].readRangeSingleMillimeters(false);


    

  }

  Serial.print("TOF Setup\n");
}


// void long_TOF_reinit() {
//   // Disable/reset all sensors by driving their XSHUT pins low.
//   for (uint8_t i = 0; i < sensorCount_3; i++)
//     {
//       io.pinMode(xshutPins_3[i], OUTPUT);
//       io.digitalWrite(xshutPins_3[i], LOW);
//     }
//   // Enable, initialize, and start each sensor, one by one.
//   for (uint8_t i = 0; i < sensorCount_3; i++)
//   {
//     io.digitalWrite(xshutPins_3[i], HIGH);
//     delay(10);
//     sensors_3[i].setTimeout(500);
//     if (!sensors_3[i].init())
//     {
//       Serial.print("Reinit Failed");
//       Serial.println(i);
//       while (1);
//     }
//     sensors_3[i].setAddress(VL53L1X_ADDRESS_START_3 + i);
//     sensors_3[i].startContinuous(50);

//   }
// }





void pick_up_setup() {
  pinMode(magnet_pin, OUTPUT);   // Electromagnet setup
  pinMode(limit_pin, INPUT_PULLUP);

  pinMode(inductor_pin, INPUT);  // Inductor sensor setup
  attachInterrupt(digitalPinToInterrupt(inductor_pin), read_inductive, FALLING);

  // Set up SX1509 communication
  // if (!io.begin(SX1509_ADDRESS)) {
  //   Serial.println("Failed to communicate.");
  //   while (1);  // Halt if communication fails
  // }

  // io.pinMode(SX1509_AIO5, INPUT);  // Limit switch setup
  // io.enableInterrupt(SX1509_AIO5, FALLING);  // Enable interrupt on falling edge

  // //Attach interrupt to the Arduino pin connected to SX1509's interrupt pin
  // pinMode(SX1509_INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(SX1509_INTERRUPT_PIN), limitSwitchISR, CHANGE);
}


//READ
//***********************************************************************

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 

// Read ultrasonic value
void ultrasonic_read(void)
{
  long durationA,durationB, Acm,Bcm;

  // Left sensor
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(AtrigPin, LOW);
  durationA = pulseIn(AechoPin, HIGH);

  // Right sensor
  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);

  // convert the time into a distance
  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);
  L_sonic = int(Acm);
  R_sonic = int(Bcm);

}

void print_ultrasonic()
{
  Serial.print("L_sonic: ");
  Serial.print(L_sonic);
  Serial.print(" R_sonic: ");
  Serial.print(R_sonic);
  Serial.print("\n");
}

int buffer_pos = 0;
int longLow_buff[BUFFER_SIZE] = {0, 0, 0};
int longHigh_buff[BUFFER_SIZE] = {0, 0, 0};
int shortLeft_buff[BUFFER_SIZE] = {0, 0, 0};
int shortRight_buff[BUFFER_SIZE] = {0, 0, 0};
int shortHighLeft_buff[BUFFER_SIZE] = {0, 0, 0};
int shortHighRight_buff[BUFFER_SIZE] = {0, 0, 0};
int shortLowLeft_buff[BUFFER_SIZE] = {0, 0, 0};
int shortLowRight_buff[BUFFER_SIZE] = {0, 0, 0};


int average_filter(int* buffer, int new_val, char identifier) {

  // Shift values and add new reading to buffer
  if (identifier == 'S' && new_val > max_STOF_range) {
    buffer[buffer_pos] = max_STOF_range;
  } else if (identifier == 'L' && new_val > max_LTOF_range) {
    buffer[buffer_pos] = max_LTOF_range;
  } else {
    buffer[buffer_pos] = new_val;
  }

  // Calculate the moving average
  int sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += buffer[i];
  }

  return sum/BUFFER_SIZE;
}


int low_filter(int new_val, char identifier) {
    if (identifier == 'S' && new_val > max_STOF_range) {
    return max_STOF_range;
  } else if (identifier == 'L' && new_val > max_LTOF_range) {
    return max_LTOF_range;
  } else {
    return new_val;
  }
}


void tof_read(void)
{
  
  // if (sensors_3[0].dataReady())
  // {
  //   longLow_reading = sensors_3[0].read(false);
  //   sensors_3[0].readRangeContinuousMillimeters(false);
  // } else {
  //   Serial.print("Long Low not ready\n");
  //   // tof_setup();
  // }
  // if (sensors_3[1].dataReady()) {
  //   longHigh_reading = sensors_3[1].read(false);
  //   sensors_3[1].readRangeContinuousMillimeters(false);
  // } else {
  //   Serial.print("Long High not ready\n");
  //   // tof_setup();
  // }


  longLow_reading = sensors_3[0].readRangeContinuousMillimeters()/10;
  longHigh_reading = sensors_3[1].readRangeContinuousMillimeters()/10;
  shortLeft_reading = sensors_1[0].readRangeContinuousMillimeters()/10;
  shortRight_reading = sensors_1[1].readRangeContinuousMillimeters()/10;
  shortHighLeft_reading = sensors_1[2].readRangeContinuousMillimeters()/10;
  shortHighRight_reading = sensors_1[3].readRangeContinuousMillimeters()/10;
  shortLowLeft_reading = sensors_2[0].readRangeContinuousMillimeters()/10;
  shortLowRight_reading = sensors_2[1].readRangeContinuousMillimeters()/10;

  // Update buffers and get averaged values



  longLow = low_filter(longLow_reading, 'L');
  longHigh = low_filter(longHigh_reading, 'L');
  shortLeft = low_filter(shortLeft_reading, 'S');
  shortRight = low_filter(shortRight_reading, 'S');
  shortHighLeft = low_filter(shortHighLeft_reading, 'S');
  shortHighRight = low_filter(shortHighRight_reading, 'S');
  shortLowLeft = low_filter(shortLowLeft_reading, 'S');
  shortLowRight = low_filter(shortLowRight_reading, 'S');

  // buffer_pos = (buffer_pos + 1) % BUFFER_SIZE;

  // longLow = average_filter( longLow_buff, longLow_reading, 'L');
  // longHigh = average_filter(longHigh_buff, longHigh_reading, 'L');
  // shortLeft = average_filter(shortLeft_buff, shortLeft_reading, 'S');
  // shortRight = average_filter(shortRight_buff, shortRight_reading, 'S');
  // shortHighLeft = average_filter(shortHighLeft_buff, shortHighLeft_reading, 'S');
  // shortHighRight = average_filter(shortHighRight_buff, shortHighRight_reading, 'S');
  // shortLowLeft = average_filter(shortLowLeft_buff, shortLowLeft_reading, 'S');
  // shortLowRight = average_filter(shortLowRight_buff, shortLowRight_reading, 'S');

  // Increment buffer position and wrap around
  

  print_tof();  
}

void print_tof()
{
  // Note: left and right are from the robot perspective
  Serial.print(longLow);
  Serial.print(", \t");
  Serial.print(longHigh);
  Serial.print(", \t");
  Serial.print(shortLeft);
  Serial.print(", \t");
  Serial.print(shortRight);
  Serial.print(", \t");
  Serial.print(shortHighLeft);
  Serial.print(", \t");
  Serial.print(shortLowLeft);
  Serial.print(", \t");
  Serial.print(shortHighRight);
  Serial.print(", \t");
  Serial.print(shortLowRight);
  Serial.print("\n");

}

void read_limit()
{
  bool limit = digitalRead(limit_pin);

  if(!limit) {
    Serial.print("limit\n");
    collect_drive();
    currentState = COLLECT;

  }
  //return limit;
}



void read_inductive()
{
  // collect_drive();
  currentState = COLLECT;
  // delay(100);
}


