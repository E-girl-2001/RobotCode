#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>
#include <Servo.h>


Servo motorA, motorB;      // create servo object to control a servo

Servo servoA;  // create servo object to control a servo

void setup() 
{ 
    // Pins 7 and 8 for the drive motors
    motorA.attach(7);
    motorB.attach(8);

    servoA.attach(0);

    // Serial.begin(9600);
    // Serial.println("Starting up");
}


void loop() 
{

    // motorA.write(1900);
    // motorB.write(1100);
    // delay(1000);
    motorA.write(1100);
    motorB.write(1900);
    delay(1000);
}



