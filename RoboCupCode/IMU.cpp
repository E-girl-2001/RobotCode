#include "IMU.h"

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Initialize the BNO055 sensor using Wire1
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);


void IMU_setup(void)
{
  Serial.println("Orientation Sensor Test"); Serial.println("");

  Wire1.begin(); // Initialize the second I2C bus
  Wire1.setClock(400000); // Set clock speed for I2C communication to 400 kHz

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}



void IMU_read(void)
{
 // unsigned long tStart = micros();
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData, linearAccelData;// angVelocityData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // printEvent(&orientationData);
  // printEvent(&linearAccelData);
  // printEvent(&angVelocityData);
  // printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  // printEvent(&gravityData);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Heading: ");
    Serial.println(orientationData.orientation.x);
    Serial.print("Position: ");
    Serial.print(xPos);
    Serial.print(" , ");
    Serial.println(yPos);
    Serial.print("Speed: ");
    Serial.println(headingVel);
    Serial.println("-------");

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }

  // while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  // {
  //   //poll until the next sample is ready
  // }
  
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  // else if (event->type == SENSOR_TYPE_GYROSCOPE) {
  //   Serial.print("Gyro:");
  //   x = event->gyro.x;
  //   y = event->gyro.y;
  //   z = event->gyro.z;
  // }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  // else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
  //   Serial.print("Rot:");
  //   x = event->gyro.x;
  //   y = event->gyro.y;
  //   z = event->gyro.z;
  // }
  // else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
  //   Serial.print("Linear:");
  //   x = event->acceleration.x;
  //   y = event->acceleration.y;
  //   z = event->acceleration.z;
  // }
  // else if (event->type == SENSOR_TYPE_GRAVITY) {
  //   Serial.print("Gravity:");
  //   x = event->acceleration.x;
  //   y = event->acceleration.y;
  //   z = event->acceleration.z;
  // }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
