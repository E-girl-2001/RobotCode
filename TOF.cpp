/*
This example shows how to set up and read multiple VL53L1X sensors connected to
the same I2C bus. Each sensor needs to have its XSHUT pin connected to a
different Arduino pin, and you should change sensorCount and the xshutPins array
below to match your setup.

For more information, see ST's application note AN4846 ("Using multiple VL53L0X
in a single design"). The principles described there apply to the VL53L1X as
well.
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>
#include <Servo.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

Servo motorA, motorB;      // create servo object to control a servo

// The number of sensors in your system.
const uint8_t sensorCount = 2;

long H_tof = 0;
long L_tof = 0;
long S1_tof = 0;
long S2_tof = 0;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL0[8] = {0,1};
const uint8_t xshutPinsL1[8] = {3,4};

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensorsL0[sensorCount];
VL53L1X sensorsL1[sensorCount];

void setup()
{
  // Pins 7 and 8 correspond to serial 2 on the teensy board
  motorA.attach(28);
  motorB.attach(29);

  while (!Serial) {}
  Serial.begin(115200);

  io.begin(SX1509_ADDRESS);

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

  Serial.print("Hello World\n");
}
}



// void drive(uint32_t left, uint32_t right)
// {
//   if(left < 300 && right < 300) {
//    motorA.writeMicroseconds(1900);      // sets the servo position full speed forward
//    motorB.writeMicroseconds(1100);      // sets the servo position full speed forward
//   if(right < 300) {
//       motorA.writeMicroseconds(1900);      // sets the servo position full speed backward
//       motorB.writeMicroseconds(1500);      // sets the servo position full speed backward
//     delay(200);
//   } else if(left < 300) {
//     motorA.writeMicroseconds(1500);      // sets the servo position full speed backward
//     motorB.writeMicroseconds(1100);      // sets the servo position full speed backward
//     delay(200);
//   } else {
//     motorA.writeMicroseconds(1100);      // sets the servo position full speed forward
//     motorB.writeMicroseconds(1900);      // sets the servo position full speed forward
//   }

//     delay(100);                           // waits for the servo to get there 
// }

void loop()
{
  tof_read();
  
//   drive(left, right);
}

void tof_read()
{
  H_tof = sensorsL0[0].readRangeContinuousMillimeters(); //left
  L_tof = sensorsL0[1].readRangeContinuousMillimeters(); //right
  S1_tof = sensorsL1[0].readRangeContinuousMillimeters(); //short 1
  S2_tof = sensorsL1[1].readRangeContinuousMillimeters(); //short 2

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(sensorsL0[i].readRangeContinuousMillimeters());
    if (sensorsL0[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print('\t');
  }

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(sensorsL1[i].read());
    if (sensorsL1[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print('\t');
  }
  Serial.println();


}