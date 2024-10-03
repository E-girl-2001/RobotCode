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

const byte SX1509_AIO5 = 5;
const uint8_t xshutPinsL1[8] = {0,1};
const uint8_t xshutPinsL0[8] = {2,3,4,5,6,7};
const uint8_t longSensorCount = 2; 
const uint8_t shortSensorCount = 6;

uint8_t x_ROI = 16;
uint8_t y_ROI = 4;

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensorsL0[shortSensorCount];
VL53L1X sensorsL1[longSensorCount];


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
  if (!io.begin(SX1509_ADDRESS_2))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }
  Wire.begin();
  Wire.setClock(400000);

  // ---------- Disable/reset all sensors by driving their XSHUT pins low. ---------
  for (uint8_t i = 0; i < longSensorCount; i++)
  {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }
  for (uint8_t i = 0; i < shortSensorCount; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }

  // --------- L0 Enable, initialize, and start each sensor, one by one. ---------
  for (uint8_t i = 0; i < shortSensorCount; i++)
  {
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensorsL0[i].startContinuous(50);
  }

    // ---------- L1 Enable, initialize, and start each sensor, one by one. ----------
  for (uint8_t i = 0; i < longSensorCount; i++)
  {
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);
    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }
    // Set both long range sensors to be filtered with SPADs
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
    sensorsL1[i].startContinuous(50);
    sensorsL1[i].setROISize(x_ROI, y_ROI);
    delay(10);
    sensorsL1[i].setROICenter(199);
    delay(10);
  }
  
  Serial.print("Tof setup\n");
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
  longLow = sensorsL1[0].readRangeContinuousMillimeters(); // Long low
  longHigh = sensorsL1[1].readRangeContinuousMillimeters(); // Long high
  shortLeft = sensorsL0[0].readRangeContinuousMillimeters(); //short 1
  shortRight = sensorsL0[1].readRangeContinuousMillimeters(); //short 2
  shortHighLeft = sensorsL0[2].readRangeContinuousMillimeters(); //short 3
  shortHighRight = sensorsL0[3].readRangeContinuousMillimeters(); //short 4
  shortLowLeft = sensorsL0[4].readRangeContinuousMillimeters(); //short 5
  shortLowRight = sensorsL0[5].readRangeContinuousMillimeters(); //short 6
  // Serial.print("LL0, \tLH1, \tSL2, \tSR3, \tSHL4, \tSHR5, \tSLL6, \tSLR7\n"); 
  // print_tof();
  
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

bool read_limit()
{
  bool limit = io.digitalRead(SX1509_AIO5);
  return limit;
}
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
  } else if(longHigh > 12) {
    BR_flag = 0;
    BL_flag = 0;
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
