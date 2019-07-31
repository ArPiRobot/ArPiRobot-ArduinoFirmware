#include "ArduinoDevice.h"

uint8_t ArduinoDevice::sendTimeOffset = 0;
uint8_t ArduinoDevice::nextId = 0;
bool OldAdafruit9Dof::locked = false;

ArduinoDevice::ArduinoDevice(){

  deviceId = nextId++;
  
	nextSendTime = sendTimeOffset;
	sendTimeOffset += OFFSET_STEP; // Stager the next by 5ms

	// Don't stager by more than 20ms because data is sent every 20ms
	if(sendTimeOffset >= SEND_RATE){
		sendTimeOffset = 0;
	}
}



SingleEncoder::SingleEncoder(int pin) : pin(pin){
  pinMode(pin, INPUT);
  lastState = digitalRead(pin);
}

bool SingleEncoder::poll(uint8_t *buffer, uint8_t *count) {
  bool shouldSend = false;
  if(lastState == LOW){
    lastState = digitalRead(pin);
    if(lastState == HIGH){
      this->count++;
      shouldSend = true;
    }
  }
  if(lastState == HIGH){
    lastState = digitalRead(pin);
    if(lastState == LOW){
      this->count++;
      shouldSend = true;
    }
  }

  if(shouldSend){
    buffer[0] = this->count >> 8;
    buffer[1] = this->count;
    *count = 2;
  }
  return shouldSend;
}

Ultrasonic4Pin::Ultrasonic4Pin(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin){
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

bool Ultrasonic4Pin::poll(uint8_t *buffer, uint8_t *count){
  bool shouldSend = false;
  pollIterationCounter++;

  if(pollIterationCounter >= ULTRASONIC_4PIN_POLL_RATE){
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
  
    uint16_t duration = pulseIn(echoPin, HIGH, 5000);
    if(duration > 0){
      uint16_t newDistance = duration * 0.034 / 2;
      shouldSend = newDistance == distance;
      distance = newDistance;
    }
    pollIterationCounter = 0;
  }

  if(shouldSend){
    buffer[0] = distance >> 8;
    buffer[1] = distance;
    *count = 2;
  }
  return shouldSend;
}

OldAdafruit9Dof::OldAdafruit9Dof(){
  if(locked)
    return;

  locked = true;
  accel = new Adafruit_LSM303_Accel_Unified(30301);
  mag = new Adafruit_LSM303_Mag_Unified(30302);
  gyro = new Adafruit_L3GD20_Unified(20);
  dof = new Adafruit_9DOF();

  bool success = accel->begin() && mag->begin() && gyro->begin();
  if(!success){
    delete accel;
    delete gyro;
    delete mag;
    accel = NULL;
    mag = NULL;
    gyro = NULL;
    locked = false;
  }

  gyro_x.fval = 0;
  gyro_y.fval = 0;
  gyro_z.fval = 0;
  accel_x.fval = 0;
  accel_y.fval = 0;
  accel_z.fval = 0;
  pitch.fval = 0;
  roll.fval = 0;
  yaw.fval = 0;
}

OldAdafruit9Dof::~OldAdafruit9Dof(){
  if(accel != NULL){
    delete accel;
    delete gyro;
    delete mag;
    locked = false;
  }
}

bool OldAdafruit9Dof::poll(uint8_t *buffer, uint8_t *count){
  if(accel == NULL) return;
  
  long now = micros();
  double dt = (now - lastSample) / 1e6;
  lastSample = now;

  if(dt < 0){
    // Micros rolled over. Some data has been lost...
    return;
  }

  sensors_event_t accel_event, mag_event, gyro_event;
  accel->getEvent(&accel_event);
  gyro->getEvent(&gyro_event);

#ifdef OLDADA9DOF_ENABLE_AHRS
  mag->getEvent(&mag_event);
#endif

  accel_x.fval = accel_event.acceleration.x;
  accel_y.fval = accel_event.acceleration.y;
  accel_z.fval = accel_event.acceleration.z;

  gyro_x.fval += gyro_event.gyro.x / SENSORS_DPS_TO_RADS * dt;
  gyro_y.fval += gyro_event.gyro.y / SENSORS_DPS_TO_RADS * dt;
  gyro_z.fval += gyro_event.gyro.z / SENSORS_DPS_TO_RADS * dt;

#ifdef OLDADA9DOF_ENABLE_AHRS
  sensors_vec_t orientation;
  if(dof->fusionGetOrientation(&accel_event, &mag_event, &orientation)){
    pitch.fval = orientation.pitch;
    roll.fval = orientation.roll;
    yaw.fval = orientation.heading;
  }
#endif

  buffer[0] = gyro_x.bval[0];
  buffer[1] = gyro_x.bval[1];
  buffer[2] = gyro_x.bval[2];
  buffer[3] = gyro_x.bval[3];
  buffer[4] = gyro_y.bval[0];
  buffer[5] = gyro_y.bval[1];
  buffer[6] = gyro_y.bval[2];
  buffer[7] = gyro_y.bval[3];
  buffer[8] = gyro_z.bval[0];
  buffer[9] = gyro_z.bval[1];
  buffer[10] = gyro_z.bval[2];
  buffer[11] = gyro_z.bval[3];
  buffer[12] = accel_x.bval[0];
  buffer[13] = accel_x.bval[1];
  buffer[14] = accel_x.bval[2];
  buffer[15] = accel_x.bval[3];
  buffer[16] = accel_y.bval[0];
  buffer[17] = accel_y.bval[1];
  buffer[18] = accel_y.bval[2];
  buffer[19] = accel_y.bval[3];
  buffer[20] = accel_z.bval[0];
  buffer[21] = accel_z.bval[1];
  buffer[22] = accel_z.bval[2];
  buffer[23] = accel_z.bval[3];
  buffer[24] = pitch.bval[0];
  buffer[25] = pitch.bval[1];
  buffer[26] = pitch.bval[2];
  buffer[27] = pitch.bval[3];
  buffer[28] = roll.bval[0];
  buffer[29] = roll.bval[1];
  buffer[30] = roll.bval[2];
  buffer[31] = roll.bval[3];
  buffer[32] = yaw.bval[0];
  buffer[33] = yaw.bval[1];
  buffer[34] = yaw.bval[2];
  buffer[35] = yaw.bval[3];
  *count = 36;
  return true;
}
