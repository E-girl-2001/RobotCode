
//**********************************************************************************
// ROBOCUP 2024 
// Team: 23




#include <Servo.h> 
// #include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>




// Custom headers
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h" 
#include "statemachine.h"
#include "IMU.h"

//**********************************************************************************
// Local Definitions
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

const int activateButton = 20; 
bool debounce = 0;


//**********************************************************************************
#define US_READ_TASK_PERIOD                 40
#define TOF_READ_TASK_PERIOD                40
#define COLOUR_READ_TASK_PERIOD             40
#define SENSOR_AVERAGE_PERIOD               40
#define SET_MOTOR_TASK_PERIOD               40
#define WEIGHT_SCAN_TASK_PERIOD             40
#define COLLECT_WEIGHT_TASK_PERIOD          40
#define RETURN_TO_BASE_TASK_PERIOD          40
#define DETECT_BASE_TASK_PERIOD             40
#define UNLOAD_WEIGHTS_TASK_PERIOD          40
#define CHECK_WATCHDOG_TASK_PERIOD          40
#define VICTORY_DANCE_TASK_PERIOD           40
#define DRIVE_TASK_PERIOD                   50
#define UPDATE_FLAG_PERIOD                  10
#define UPDATE_STATE_PERIOD                 10
#define WEIGHT_DETECT_PERIOD                50
#define ACTIVATE_IDLE_PERIOD                80
#define IMU_TASK_PERIOD                     40

// Task execution amount definitions
// -1 means indefinitely
#define US_READ_TASK_NUM_EXECUTE           -1
#define TOF_READ_TASK_NUM_EXECUTE          -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1
#define SENSOR_AVERAGE_NUM_EXECUTE         -1
#define SET_MOTOR_TASK_NUM_EXECUTE         -1
#define WEIGHT_SCAN_TASK_NUM_EXECUTE       -1
#define COLLECT_WEIGHT_TASK_NUM_EXECUTE    -1
#define RETURN_TO_BASE_TASK_NUM_EXECUTE    -1
#define DETECT_BASE_TASK_NUM_EXECUTE       -1
#define UNLOAD_WEIGHTS_TASK_NUM_EXECUTE    -1
#define CHECK_WATCHDOG_TASK_NUM_EXECUTE    -1
#define VICTORY_DANCE_TASK_NUM_EXECUTE     -1
#define DRIVE_TASK_NUM_EXECUTE             -1
#define UPDATE_FLAG_TASK_NUM_PERIOD        -1
#define UPDATE_STATE_TASK_NUM_PERIOD       -1
#define WEIGHT_DETECT_NUM_PERIOD           -1
#define ACTIVATE_IDLE_NUM_PERIOD           -1
#define IMU_TASK_NUM_EXECUTE               -1


// Pin deffinitions
#define IO_POWER  49

// Serial deffinitions
#define BAUD_RATE 276000

// Global variables used in sensors and motors
int L_sonic, R_sonic, H_tof, L_tof, S1_tof, S2_tof;
bool R_flag, L_flag, HR_flag, HL_flag, BL_flag, BR_flag;

//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

// Tasks for reading sensors 
Task tRead_ultrasonic(US_READ_TASK_PERIOD,       US_READ_TASK_NUM_EXECUTE,        &ultrasonic_read);
Task tRead_tof(TOF_READ_TASK_PERIOD,               TOF_READ_TASK_NUM_EXECUTE,        &tof_read);
//Task tRead_colour(COLOUR_READ_TASK_PERIOD,       COLOUR_READ_TASK_NUM_EXECUTE,    &read_colour);
//Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);
Task tRead_IMU(IMU_TASK_PERIOD,    IMU_TASK_NUM_EXECUTE, &IMU_read);

// Task to set the motor speeds and direction
Task tSet_motor(SET_MOTOR_TASK_PERIOD,           SET_MOTOR_TASK_NUM_EXECUTE,      &set_motor);
Task tUpdate_flags(UPDATE_FLAG_PERIOD,       UPDATE_FLAG_TASK_NUM_PERIOD,    &update_flags);
Task tUpdate_state(UPDATE_STATE_PERIOD,       UPDATE_STATE_TASK_NUM_PERIOD,    &update_state);

// Tasks to scan for weights and collection upon detection
Task tWeight_scan(WEIGHT_SCAN_TASK_PERIOD,       WEIGHT_SCAN_TASK_NUM_EXECUTE,    &weight_scan);
Task tCollect_weight(COLLECT_WEIGHT_TASK_PERIOD, COLLECT_WEIGHT_TASK_NUM_EXECUTE, &weight_collect);
Scheduler taskManager;

// ---------- Function Definitions -----------
void pin_init();
void task_init();


void setup() {
  Serial.begin(BAUD_RATE);
  ultrasonic_setup();
  tof_setup();
  //IMU_setup();
  //colour_setup();
  pick_up_setup();
  delay(10);
  motor_setup();
  delay(10);
  task_init();
  delay(10);
  Wire.begin();
  Serial.print("Setup Complete\n");
  delay(100);
  //activate();
}


void pin_init(){
    pinMode(activateButton, INPUT);
    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
    Serial.println("Pins have been initialised \n"); 
}


void task_init() {  
  
  taskManager.init();     
  // --------- Add tasks to the scheduler ----------
  taskManager.addTask(tRead_ultrasonic);
  taskManager.addTask(tRead_tof);
  taskManager.addTask(tRead_IMU);
  taskManager.addTask(tUpdate_flags);
  taskManager.addTask(tUpdate_state);
  taskManager.addTask(tSet_motor); 
  taskManager.addTask(tWeight_scan);
  taskManager.addTask(tCollect_weight);
  //taskManager.addTask(tRead_colour);
  //taskManager.addTask(tSensor_average);

  // ---------- Enable the tasks ----------
  tRead_ultrasonic.enable();
  tRead_tof.enable();
  tRead_IMU.enable();
  tUpdate_flags.enable();
  tUpdate_state.enable();
  tSet_motor.enable();
  tWeight_scan.enable();
  //tRead_colour.enable();
  //tSensor_average.enable();
  Serial.println("Tasks have been initialised \n");
}



void loop() {
  taskManager.execute();
}
