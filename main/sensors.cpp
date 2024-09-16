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

  // ----------- Setup the TOF sensors ------------
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }


  // ----- Initialize SHORT range TOF sensors -----
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
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(50);
  }

  // ----- Initialize LONG range TOF sensors -----
  for (uint8_t i = 0; i < sensorCount; i++)
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
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 

// Read ultrasonic value
void ultrasonic_read(void)
{
  long durationA, durationB, Acm, Bcm;
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
 
  durationA = pulseIn(AechoPin, HIGH);
  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);

  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);
  *L_sonic = Acm;
  *R_sonic = Bcm;
}

void tof_read(void)
{
  long short1 = sensorsL0[0].readRangeContinuousMillimeters(); //short 1
  long short2 = sensorsL0[1].readRangeContinuousMillimeters(); //short 2
  long high = sensorsL1[1].readRangeContinuousMillimeters(); //left
  long low = sensorsL1[0].readRangeContinuousMillimeters(); //right
  *L_tof = (high);
  *H_tof = (low);
  *S1_tof = (short1);
  *S2_tof = (short2);
  
  Serial.print("High, Low, Short1, Short2 \n");
  Serial.print("\n");
  Serial.print(*L_tof);
  Serial.print(", ");
  Serial.print(*H_tof);
  Serial.print(", ");
  Serial.print(*S1_tof);
  Serial.print(", ");
  Serial.print(*S2_tof);

}

bool read_inductive()
{
  bool inductive = digitalRead(inductor_pin);
  return inductive;
}
//***********************************************************************

bool check_tof_for_weights(int16_t *H_tof, int16_t *L_tof)
{
  if (*H_tof < (*L_tof + 20))
  {
    return true;
  }
  else
  {
    return false;
  }
}