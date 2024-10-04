//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.

#include "sensors.h"
#include "Arduino.h"
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

#define side_distance 14
#define front_distance 14

//#define 
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;


const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START_1 0x30
#define VL53L0X_ADDRESS_START_2 0x40
#define VL53L1X_ADDRESS_START_3 0x50


// The number of sensors in your system.
const uint8_t sensorCount_1 = 4;
const uint8_t sensorCount_2 = 2;
const uint8_t sensorCount_3 = 2;


// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins_1[8] = {0,1,2,3,4,5,6,7};
const uint8_t xshutPins_2[8] = {4,5,6,7};
const uint8_t xshutPins_3[8] = {6,7};

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensors_1[sensorCount_1];
VL53L0X sensors_2[sensorCount_2];
VL53L1X sensors_3[sensorCount_3];


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
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    sensors_3[i].setAddress(VL53L1X_ADDRESS_START_3 + i);
    sensors_3[i].startContinuous(50);
  }
}

void pick_up_setup()
{
  pinMode(inductor_pin, INPUT); //inductor sensor setup
  pinMode(magnet_pin, OUTPUT); //electromagnet setup
  // io.pinMode(SX1509_AIO5, INPUT); //limit_switch setup
  // if (!io.begin(SX1509_ADDRESS))
  // {
  //   Serial.println("Failed to communicate.");
  //   while (1) ;
  // }
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

void tof_read(void)
{

//const uint8_t xshutPins_1[8] = {0,1,2,3,4,5,6,7};
//const uint8_t xshutPins_2[8] = {4,5,6,7};
//const uint8_t xshutPins_3[8] = {6,7};

  longLow = sensors_3[0].readRangeContinuousMillimeters()/10;
  longHigh = sensors_3[1].readRangeContinuousMillimeters()/10;
  shortLeft = sensors_1[0].readRangeContinuousMillimeters()/10;
  shortRight = sensors_1[1].readRangeContinuousMillimeters()/10;
  shortHighLeft = sensors_1[2].readRangeContinuousMillimeters()/10;
  shortHighRight = sensors_1[3].readRangeContinuousMillimeters()/10;
  shortLowLeft = sensors_2[0].readRangeContinuousMillimeters()/10;
  shortLowRight = sensors_2[1].readRangeContinuousMillimeters()/10;
  //   for (uint8_t i = 0; i < sensorCount_1; i++)
  // {
  //   Serial.print(sensors_1[i].readRangeContinuousMillimeters());
  //   if (sensors_2[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  //   Serial.print('\t');
  // }
  // for (uint8_t i = 0; i < sensorCount_2; i++)
  // {
  //   Serial.print(sensors_2[i].readRangeContinuousMillimeters());
  //   if (sensors_2[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  //   Serial.print('\t');
  // }
  // for (uint8_t i = 0; i < sensorCount_3; i++)
  // {
  //   Serial.print(sensors_3[i].read());
  //   if (sensors_3[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  //   Serial.print('\t');
  // }
  // Serial.println();
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
  Serial.print(shortHighRight);
  Serial.print(", \t");
  Serial.print(shortLowLeft);
  Serial.print(", \t");
  Serial.print(shortLowRight);
  Serial.print("\n");

}

// bool read_limit()
// {
//   bool limit = io.digitalRead(SX1509_AIO5);
//   return limit;
// }
bool read_inductive()
{
  bool inductive = digitalRead(inductor_pin);
  return inductive;
}

//STATES
//***********************************************************************

void update_flags()
{
  if((L_sonic < side_distance) && (!R_flag)) {
    L_flag = 1;
  } else if (L_sonic > 8) {
    L_flag = 0;
  }

  if((R_sonic < side_distance) && (!L_flag)) {
    R_flag = 1;
  } else if (R_sonic > 8) {
    R_flag = 0;
  }

  if(longHigh < front_distance) {
    if (L_sonic < R_sonic) {
      BL_flag = 1;
    } else {
      BR_flag = 1;
    }
  } else if(longHigh > 12 && BL_flag == 1 && L_sonic > 10) {
    BL_flag = 0;
  } else if(longHigh > 12 && BR_flag == 1 && R_sonic > 10) {
    BR_flag = 0;
  }

  if(shortLeft > longLow && shortLeft > shortRight) {
    HR_flag = 1;
  } else {
    HR_flag = 0;
  }

  if(shortRight > longLow && shortRight > shortLeft) {
    HL_flag = 1;
  } else {
    HL_flag = 0;
  }
}
