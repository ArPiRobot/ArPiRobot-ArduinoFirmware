
// How many samples and how long to wait between samples (ms)
#define CALIBRATE_SAMPLES 5000
#define SAMPLE_SPACING 1

// Uncomment a single sensor to calibrate
#define OLD_ADA_9DOF
//#define NXP_ADA_9DOF




// Gravitational acceleration (used to calibrate accelerometer z axis)
const double g = 9.80665;




#ifdef OLD_ADA_9DOF

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

sensors_event_t gyro_event, accel_event;
double gyro_x_calib = 0,
       gyro_y_calib = 0,
       gyro_z_calib = 0,
       accel_x_calib = 0,
       accel_y_calib = 0,
       accel_z_calib = 0;

void setup(void){
  Serial.begin(9600);
  while(!Serial);
  
  Serial.println("Old Adafruit 9DOF (LSM303 + L3GD20) Calibration.\nMake sure to keep sensor board level and completely still during calibration!"); 
  
  if(!accel.begin()){
    Serial.println("Failed to initialize accelerometer!");
    while(1);
  }
  if(!gyro.begin()) {
    Serial.println("Failed to initialize gyroscope!");
  }

  Serial.println("Waiting 5 seconds before starting calibration.");
  delay(5000);
  Serial.print("Getting samples for calibration");
  
  for(uint32_t i = 0; i < CALIBRATE_SAMPLES; ++i){
    gyro.getEvent(&gyro_event);
    accel.getEvent(&accel_event);
    gyro_x_calib += gyro_event.gyro.x;
    gyro_y_calib += gyro_event.gyro.y;
    gyro_z_calib += gyro_event.gyro.z;
    accel_x_calib += accel_event.acceleration.x;
    accel_y_calib += accel_event.acceleration.y;
    accel_z_calib += accel_event.acceleration.z;
    delay(SAMPLE_SPACING);

    if(i % (1000 / SAMPLE_SPACING) == 0)
      Serial.print(".");
  }

  gyro_x_calib /= CALIBRATE_SAMPLES;
  gyro_y_calib /= CALIBRATE_SAMPLES;
  gyro_z_calib /= CALIBRATE_SAMPLES;
  accel_x_calib /= CALIBRATE_SAMPLES;
  accel_y_calib /= CALIBRATE_SAMPLES;
  accel_z_calib /= CALIBRATE_SAMPLES;

  // When stationary the z axis reads g not zero
  accel_z_calib -= g;

  Serial.println();
  Serial.println("Calibration complete.");
  
  Serial.print("Gyro X Offset:          ");
  Serial.println(gyro_x_calib, 50);

  Serial.print("Gyro Y Offset:          ");
  Serial.println(gyro_y_calib, 50);

  Serial.print("Gyro Z Offset:          ");
  Serial.println(gyro_z_calib, 50);

  Serial.print("Accelerometer X Offset: ");
  Serial.println(accel_x_calib, 50);

  Serial.print("Accelerometer Y Offset: ");
  Serial.println(accel_y_calib, 50);

  Serial.print("Accelerometer Z Offset: ");
  Serial.println(accel_z_calib, 50);
}

#endif //OLD_ADA_9DOF

#ifdef NXP_ADA_9DOF

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

Adafruit_FXOS8700   accel = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro  = Adafruit_FXAS21002C(0x0021002C);

sensors_event_t gyro_event, accel_event;
double gyro_x_calib = 0,
       gyro_y_calib = 0,
       gyro_z_calib = 0,
       accel_x_calib = 0,
       accel_y_calib = 0,
       accel_z_calib = 0;

void setup(void){
  Serial.begin(9600);
  while(!Serial);
  
  Serial.println("Adafruit NXP Precision 9DOF (FXOS8700 + FXAS21002C) Calibration.\nMake sure to keep sensor board level and completely still during calibration!"); 
  
  if(!accel.begin()){
    Serial.println("Failed to initialize accelerometer!");
    while(1);
  }
  if(!gyro.begin()) {
    Serial.println("Failed to initialize gyroscope!");
  }

  Serial.println("Waiting 5 seconds before starting calibration.");
  delay(5000);
  Serial.print("Getting samples for calibration");
  
  for(uint32_t i = 0; i < CALIBRATE_SAMPLES; ++i){
    gyro.getEvent(&gyro_event);
    accel.getEvent(&accel_event);
    gyro_x_calib += gyro_event.gyro.x;
    gyro_y_calib += gyro_event.gyro.y;
    gyro_z_calib += gyro_event.gyro.z;
    accel_x_calib += accel_event.acceleration.x;
    accel_y_calib += accel_event.acceleration.y;
    accel_z_calib += accel_event.acceleration.z;
    delay(SAMPLE_SPACING);

    if(i % (1000 / SAMPLE_SPACING) == 0)
      Serial.print(".");
  }
  gyro_x_calib /= CALIBRATE_SAMPLES;
  gyro_y_calib /= CALIBRATE_SAMPLES;
  gyro_z_calib /= CALIBRATE_SAMPLES;
  accel_x_calib /= CALIBRATE_SAMPLES;
  accel_y_calib /= CALIBRATE_SAMPLES;
  accel_z_calib /= CALIBRATE_SAMPLES;

  // When stationary the z axis reads g not zero
  accel_z_calib -= g;

  Serial.println();
  Serial.println("Calibration complete.");
  
  Serial.print("Gyro X Offset:          ");
  Serial.println(gyro_x_calib, 50);

  Serial.print("Gyro Y Offset:          ");
  Serial.println(gyro_y_calib, 50);

  Serial.print("Gyro Z Offset:          ");
  Serial.println(gyro_z_calib, 50);

  Serial.print("Accelerometer X Offset: ");
  Serial.println(accel_x_calib, 50);

  Serial.print("Accelerometer Y Offset: ");
  Serial.println(accel_y_calib, 50);

  Serial.print("Accelerometer Z Offset: ");
  Serial.println(accel_z_calib, 50);
}

#endif //NXP_ADA_9DOF

void loop(void){
  
}
