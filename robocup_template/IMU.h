#ifndef IMU_H_
#define IMU_H_

#include <Wire.h>       
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

void IMU_setup();
void IMU_read();
void printEvent(sensors_event_t* event);

#endif