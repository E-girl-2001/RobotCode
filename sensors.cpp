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

// Local definitions
static int R_sonic, L_sonic, H_tof, L_tof, S1_tof, S2_tof, HR_flag, HL_flag;

bool L_flag, R_flag, BL_flag, BR_flag, limit_switch;

#define side_distance 12
#define front_distance 14

//#define 
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

const byte SX1509_AIO5 = 5;
const uint8_t xshutPinsL0[8] = {0,1,2,3,4,5,6,7};
const uint8_t xshutPinsL1[8] = {2,3,4,5,6,7};
const uint8_t sensorCount = 2; 

uint8_t x_ROI = 16;
uint8_t y_ROI = 4;

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensorsL0[sensorCount];
VL53L1X sensorsL1[sensorCount];

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
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }


  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
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

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(50);
  }
    // L1 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
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
  io.pinMode(SX1509_AIO5, INPUT); //limit_switch setup
  
  if (!io.begin(SX1509_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }
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

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  durationA = pulseIn(AechoPin, HIGH);
  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);
  // convert the time into a distance
  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);

  // Serial.print(Acm);
  // Serial.print(" ");
  // Serial.print(Bcm);
  // Serial.println();
  L_sonic = int(Acm);
  R_sonic = int(Bcm);

}

void tof_read(void)
{
  long short1 = sensorsL0[0].readRangeContinuousMillimeters(); //short 1
  long short2 = sensorsL0[1].readRangeContinuousMillimeters(); //short 2
  long high = sensorsL1[1].readRangeContinuousMillimeters(); //left
  long low = sensorsL1[0].readRangeContinuousMillimeters(); //right
  Serial.print("\n");
  

// convert to cm
  H_tof = (high / 10);
  L_tof = (low / 10);
  S1_tof = (short1 / 10);
  S2_tof = (short2 / 10);


  // Serial.print("High, Low, Short1, Short2 \n");

  Serial.print(H_tof);
  Serial.print(", ");
  Serial.print(L_tof);
  Serial.print(", ");
  Serial.print(S1_tof);
  Serial.print(", ");
  Serial.print(S2_tof);
  
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

  if(H_tof < front_distance) {
    if (L_sonic < R_sonic) {
      BL_flag = 1;
    } else {
      BR_flag = 1;
    }
  } else if(H_tof > 12) {
    BR_flag = 0;
    BL_flag = 0;
  }

  if(S1_tof > L_tof && S1_tof > S2_tof) {
    HR_flag = 1;
  } else {
    HR_flag = 0;
  }

  if(S2_tof > L_tof && S2_tof > S1_tof) {
    HL_flag = 1;
  } else {
    HL_flag = 0;
  }
}





// GET FUNKY
//***********************************************************************

int get_H_tof()
{
  return H_tof;
}

int get_L_tof()
{
  return L_tof;
}

int get_S1_tof()
{
  return S1_tof;
}

int get_S2_tof()
{
  return S2_tof;
}

int get_R_sonic()
{
  return R_sonic;
}

int get_L_sonic()
{
  return L_sonic;
}

int get_L_flag()
{
  return L_flag;
}

int get_R_flag()
{
  return R_flag;
}

int get_BL_flag()
{
  return BL_flag;
}

int get_BR_flag()
{
  return BR_flag;
}

int get_HR_flag()
{
  return HR_flag;
}

int get_HL_flag()
{
  return HL_flag;
}


int get_limit_switch()
{
  return limit_switch;
}
