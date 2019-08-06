#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#define CALIBRATE_SAMPLES 5000
#define SAMPLE_SPACING 1

// Uncomment a single sensor to calibrate
#define OLD_ADA_9DOF


#ifdef OLD_ADA_9DOF

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

sensors_event_t event;
double gyro_x_calib = 0,
      gyro_y_calib = 0,
      gyro_z_calib = 0;

void setup(void){
  Serial.begin(9600);
  while(!Serial);
  
  Serial.println("Old Adafruit 9DOF (LSM303 + L3GD20) Calibration.\nMake sure to keep sensor board level and completely still during calibration!"); 
  
  if(!accel.begin()){
    Serial.println("Failed to initialize accelerometer!");
    while(1);
  }
  if(!mag.begin()) {
    Serial.println("Failed to initialize magnetometer!");
    while(1);
  }
  if(!gyro.begin()) {
    Serial.println("Failed to initialize gyroscope!");
  }

  Serial.println("Waiting 5 seconds before starting calibration.");
  delay(5000);
  Serial.print("Getting samples for calibration");
  
  for(uint32_t i = 0; i < CALIBRATE_SAMPLES; ++i){
    gyro.getEvent(&event);
    gyro_x_calib += event.gyro.x;
    gyro_y_calib += event.gyro.y;
    gyro_z_calib += event.gyro.z;
    delay(SAMPLE_SPACING);

    if(i % (1000 / SAMPLE_SPACING) == 0)
      Serial.print(".");
  }
  gyro_x_calib /= CALIBRATE_SAMPLES;
  gyro_y_calib /= CALIBRATE_SAMPLES;
  gyro_z_calib /= CALIBRATE_SAMPLES;

  Serial.println();
  Serial.println("Calibration complete.");
  
  Serial.print("Gyro X Offset: ");
  Serial.println(gyro_x_calib, 50);

  Serial.print("Gyro Y Offset: ");
  Serial.println(gyro_y_calib, 50);

  Serial.print("Gyro Z Offset: ");
  Serial.println(gyro_z_calib, 50);
}

#endif //OLD_ADA_9DOF

void loop(void){
  
}
