#include <Servo.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h> 
#include "motors.h"
#include "sensors.h"
// #include <Adafruit_TCS34725.h>

//**********************************************************************************

// Task period Definitions
#define US_READ_TASK_PERIOD                 40
#define TOF_READ_TASK_PERIOD                40
#define COLOUR_READ_TASK_PERIOD             40
#define SENSOR_AVERAGE_PERIOD               40
#define SET_MOTOR_TASK_PERIOD               40

// Task execution amount definitions
// -1 means indefinitely
#define US_READ_TASK_NUM_EXECUTE           -1
#define TOF_READ_TASK_NUM_EXECUTE          -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1
#define SENSOR_AVERAGE_NUM_EXECUTE         -1
#define SET_MOTOR_TASK_NUM_EXECUTE         -1

#define IO_POWER  49

#define BAUD_RATE 9600

enum State {
  IDLE = 0, SEARCH = 1, HUNT = 2, SCAN = 3, COLLECT = 4, HOMING = 5, DROPPING = 6 };

State currentState = IDLE;
int16_t weight_count = 0;

Task tRead_ultrasonic(US_READ_TASK_PERIOD,       US_READ_TASK_NUM_EXECUTE,        &ultrasonic_read);
Task tRead_tof(TOF_READ_TASK_PERIOD,               TOF_READ_TASK_NUM_EXECUTE,        &tof_read);
// Task tRead_colour(COLOUR_READ_TASK_PERIOD,       COLOUR_READ_TASK_NUM_EXECUTE,    &read_colour);
// Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);
Task tSet_motor(SET_MOTOR_TASK_PERIOD,           SET_MOTOR_TASK_NUM_EXECUTE,      &set_motor);

Scheduler taskManager;

int16_t L_sonic, R_sonic, H_tof, L_tof, S1_tof, S2_tof;
bool start_button = 0;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();

void setup() {
  Serial.begin(BAUD_RATE);
  ultrasonic_setup();
  tof_setup();
  pick_up_setup();
  motor_setup();
  robot_init();
  task_init();
  Wire.begin();
  Serial.print("Setup Complete\n");
  Serial.print("High, Low, Short1, Short2 \n");

}

void pin_init(){
    Serial.println("Pins have been initialised \n"); 
    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}

void task_init() {  
  // This is a class/library function. Initialise the task scheduler
  taskManager.init();     
  taskManager.addTask(tRead_ultrasonic);   //reading ultrasonic 
  taskManager.addTask(tRead_tof);
  // taskManager.addTask(tRead_colour);
  // taskManager.addTask(tSensor_average);
  taskManager.addTask(tSet_motor);    

  tRead_ultrasonic.enable();
  tRead_tof.enable();
  // tRead_colour.enable();
  // tSensor_average.enable();
  tSet_motor.enable();
  Serial.println("Tasks have been initialised \n");
}

void loop() {
  
  taskManager.execute();    //execute the scheduler


  switch (currentState) {
  case IDLE:
      // Check if button is pressed and change state to SEARCH
      // start_button = io.digitalRead(0);
      // if (start_button == 1) {
      //     currentState = SEARCH;
      // }
      currentState = SEARCH;
      break;


  case SEARCH:
      // Handle the SEARCH state
      search_drive(L_sonic, R_sonic);
      check_tof_for_weights(&H_tof, &L_tof);
      
      if (read_inductive()) {
          currentState = COLLECT;
      }
      break;


  case HUNT:
      // Head toward the target using the short range TOF sensors
      hunt_drive(H_tof, L_tof, S1_tof, S2_tof);
      
      if (read_inductive()) {
          currentState = COLLECT;
      }
      break;


  case SCAN:
      // Periodically scan the area using the long range TOF sensors
      scan_area(H_tof, L_tof, S1_tof, S2_tof);
      
      if (read_inductive()) {
          currentState = COLLECT;
      }
      break;


  case COLLECT:
      // Collect the target using the inductive sensor in combination with the electromagnet
      collect_target();
      weight_count++;
      currentState = SEARCH;
      break;


  case HOMING:
      // When enough targets have been collected, return to the starting position
      homing_drive();
      break;


  case DROPPING:
      // Drop the collected targets
      drop_targets();
      break;


  default:
      // Handle unknown states
      break;
}

  //Serial.println("Another scheduler execution cycle has oocured \n");
}