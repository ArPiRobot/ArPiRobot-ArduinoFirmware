#include "ArduinoDevice.h"
#include "RPiInterface.h"

uint8_t ArduinoDevice::sendTimeOffset = 0;

#ifdef OLDADA9DOF_ENABLE
bool OldAdafruit9Dof::locked = false;
#endif

#ifdef NXPADA9DOF_ENABLE
bool NxpAdafruit9Dof::locked = false;
#endif

ArduinoDevice::ArduinoDevice(uint8_t buffer_len){  
	nextSendTime = sendTimeOffset;
	sendTimeOffset += OFFSET_STEP; // Stager the next by 5ms

	// Don't stager by more than 20ms because data is sent every 20ms
	if(sendTimeOffset >= SEND_RATE){
		sendTimeOffset = 0;
	}

  buffer = new uint8_t[buffer_len];
}

ArduinoDevice::~ArduinoDevice(){
  
}


void ArduinoDevice::assignDeviceId(uint8_t deviceId){
  this->deviceId = deviceId;
}

void ArduinoDevice::sendBuffer(RPiInterface &rpi){
  if(buffer_count > 0){
    rpi.writeData(buffer, buffer_count);
    rpi.flush();
    buffer_count = 0;
  }
}


SingleEncoder::SingleEncoder(int pin)  : ArduinoDevice(3), pin(pin){
  pinMode(pin, INPUT);
  lastState = digitalRead(pin);
}

void SingleEncoder::poll() {
  if(lastState == LOW){
    lastState = digitalRead(pin);
    if(lastState == HIGH){
      this->count++;
      changed = true;
    }
  }
  if(lastState == HIGH){
    lastState = digitalRead(pin);
    if(lastState == LOW){
      this->count++;
      changed = true;
    }
  }

  if(changed){
    buffer[0] = deviceId;
    buffer[1] = count;
    buffer[2] = count >> 8;
    buffer_count = 3;
    changed = false;
  }
}

void SingleEncoder::handleData(uint8_t *data, uint8_t len){
  // This device does not accept data from the pi
}

Ultrasonic4Pin::Ultrasonic4Pin(int triggerPin, int echoPin)  : ArduinoDevice(3), triggerPin(triggerPin), echoPin(echoPin){
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void Ultrasonic4Pin::poll(){
  bool shouldSend = false;
  pollIterationCounter++;

  if(pollIterationCounter >= ULTRASONIC_4PIN_POLL_RATE){
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
  
    uint16_t duration = pulseIn(echoPin, HIGH, 5000);
    if(duration > 0){
      uint16_t newDistance = duration * 0.034 / 2;
      shouldSend = newDistance != distance;
      distance = newDistance;
    }
    pollIterationCounter = 0;
  }

  if(shouldSend){
    buffer[0] = deviceId;
    buffer[1] = distance;
    buffer[2] = distance >> 8;
    buffer_count = 3;
  }
}

void Ultrasonic4Pin::handleData(uint8_t *data, uint8_t len){
    // This device does not accept data from the pi
}

#ifdef OLDADA9DOF_ENABLE

OldAdafruit9Dof::OldAdafruit9Dof() : ArduinoDevice(25){
  if(locked)
    return;

  locked = true;
  accel = new Adafruit_LSM303_Accel_Unified(30301);
  gyro = new Adafruit_L3GD20_Unified(20);

  bool success = accel->begin() && gyro->begin();
  if(!success){
    delete accel;
    delete gyro;
    accel = NULL;
    gyro = NULL;
    locked = false;
  }

  gyro_x.fval = 0;
  gyro_y.fval = 0;
  gyro_z.fval = 0;
  accel_x.fval = 0;
  accel_y.fval = 0;
  accel_z.fval = 0; 
}

OldAdafruit9Dof::~OldAdafruit9Dof(){
  if(accel != NULL){
    delete accel;
    delete gyro;
    locked = false;
  }
}

void OldAdafruit9Dof::poll(){
  if(accel == NULL) return;
  
  long now = micros();
  double dt = (now - lastSample) / 1e6;
  lastSample = now;

  if(dt < 0){
    // Micros rolled over. Some data has been lost...
    return;
  }

  accel->getEvent(&accel_event);
  gyro->getEvent(&gyro_event);

  accel_x.fval = accel_event.acceleration.x - OLDADA9DOF_ACCEL_X_OFFSET;
  accel_y.fval = accel_event.acceleration.y - OLDADA9DOF_ACCEL_Y_OFFSET;
  accel_z.fval = accel_event.acceleration.z - OLDADA9DOF_ACCEL_Z_OFFSET;

  gyro_x.fval += (gyro_event.gyro.x - OLDADA9DOF_GYRO_X_OFFSET) / SENSORS_DPS_TO_RADS * dt;
  gyro_y.fval += (gyro_event.gyro.y - OLDADA9DOF_GYRO_Y_OFFSET) / SENSORS_DPS_TO_RADS * dt;
  gyro_z.fval += (gyro_event.gyro.z - OLDADA9DOF_GYRO_Z_OFFSET) / SENSORS_DPS_TO_RADS * dt;

  //TODO: Handle if big endian here...
  //      If big endian need to reverse order of bytes in buffer so it looks little endian to the pi
  buffer[0] = deviceId;
  buffer[1] = gyro_x.bval[0];
  buffer[2] = gyro_x.bval[1];
  buffer[3] = gyro_x.bval[2];
  buffer[4] = gyro_x.bval[3];
  buffer[5] = gyro_y.bval[0];
  buffer[6] = gyro_y.bval[1];
  buffer[7] = gyro_y.bval[2];
  buffer[8] = gyro_y.bval[3];
  buffer[9] = gyro_z.bval[0];
  buffer[10] = gyro_z.bval[1];
  buffer[11] = gyro_z.bval[2];
  buffer[12] = gyro_z.bval[3];
  buffer[13] = accel_x.bval[0];
  buffer[14] = accel_x.bval[1];
  buffer[15] = accel_x.bval[2];
  buffer[16] = accel_x.bval[3];
  buffer[17] = accel_y.bval[0];
  buffer[18] = accel_y.bval[1];
  buffer[19] = accel_y.bval[2];
  buffer[20] = accel_y.bval[3];
  buffer[21] = accel_z.bval[0];
  buffer[22] = accel_z.bval[1];
  buffer[23] = accel_z.bval[2];
  buffer[24] = accel_z.bval[3];
  buffer_count = 25;
}

void OldAdafruit9Dof::handleData(uint8_t *data, uint8_t len){
    // This device does not accept data from the pi
}

#endif // OLDADA9DOF_ENABLE

VoltageMonitor::VoltageMonitor(uint8_t readPin, float vboard, uint32_t r1, uint32_t r2) : ArduinoDevice(5), readPin(readPin), vboard(vboard), r1(r1), r2(r2){
  pinMode(readPin, INPUT);
}

void VoltageMonitor::poll(){
  // TODO: Handle big endian system
  voltage.fval = (((float)analogRead(readPin) / 1023 * vboard) * (r1 + r2)) / r2;
  buffer[0] = deviceId;
  buffer[1] = voltage.bval[0];
  buffer[2] = voltage.bval[1];
  buffer[3] = voltage.bval[2];
  buffer[4] = voltage.bval[3];
  buffer_count = 5;
}

void VoltageMonitor::handleData(uint8_t *data, uint8_t len){
    // This device does not accept data from the pi
}

#ifdef NXPADA9DOF_ENABLE

NxpAdafruit9Dof::NxpAdafruit9Dof() : ArduinoDevice(25){
  if(locked)
    return;

  locked = true;
  accelmag = new Adafruit_FXOS8700(0x8700A, 0x8700B);
  gyro = new Adafruit_FXAS21002C(0x0021002C);

  bool success = accelmag->begin(ACCEL_RANGE_4G) && gyro->begin();
  if(!success){
    delete accelmag;
    delete gyro;
    accelmag = NULL;
    gyro = NULL;
    locked = false;
  }

  gyro_x.fval = 0;
  gyro_y.fval = 0;
  gyro_z.fval = 0;
  accel_x.fval = 0;
  accel_y.fval = 0;
  accel_z.fval = 0;
}

NxpAdafruit9Dof::~NxpAdafruit9Dof(){
  if(accelmag != NULL){
    delete accelmag;
    delete gyro;
    locked = false;
  }
}

void NxpAdafruit9Dof::poll(){
  if(accelmag == NULL) return;
  
  long now = micros();
  double dt = (now - lastSample) / 1e6;
  lastSample = now;

  if(dt < 0){
    // Micros rolled over. Some data has been lost...
    return;
  }

  accelmag->getEvent(&accel_event, &mag_event);
  gyro->getEvent(&gyro_event);

  accel_x.fval = accel_event.acceleration.x - NXPADA9DOF_ACCEL_X_OFFSET;
  accel_y.fval = accel_event.acceleration.y - NXPADA9DOF_ACCEL_Y_OFFSET;
  accel_z.fval = accel_event.acceleration.z - NXPADA9DOF_ACCEL_Z_OFFSET;

  gyro_x.fval += (gyro_event.gyro.x - NXPADA9DOF_GYRO_X_OFFSET) / SENSORS_DPS_TO_RADS * dt;
  gyro_y.fval += (gyro_event.gyro.y - NXPADA9DOF_GYRO_Y_OFFSET) / SENSORS_DPS_TO_RADS * dt;
  gyro_z.fval += (gyro_event.gyro.z - NXPADA9DOF_GYRO_Z_OFFSET) / SENSORS_DPS_TO_RADS * dt;

  //TODO: Handle if big endian here...
  //      If big endian need to reverse order of bytes in buffer so it looks little endian to the pi
  //      In setup() should do a big endian test and write create a global bool indicating if big endian
  buffer[0] = deviceId;
  buffer[1] = gyro_x.bval[0];
  buffer[2] = gyro_x.bval[1];
  buffer[3] = gyro_x.bval[2];
  buffer[4] = gyro_x.bval[3];
  buffer[5] = gyro_y.bval[0];
  buffer[6] = gyro_y.bval[1];
  buffer[7] = gyro_y.bval[2];
  buffer[8] = gyro_y.bval[3];
  buffer[9] = gyro_z.bval[0];
  buffer[10] = gyro_z.bval[1];
  buffer[11] = gyro_z.bval[2];
  buffer[12] = gyro_z.bval[3];
  buffer[13] = accel_x.bval[0];
  buffer[14] = accel_x.bval[1];
  buffer[15] = accel_x.bval[2];
  buffer[16] = accel_x.bval[3];
  buffer[17] = accel_y.bval[0];
  buffer[18] = accel_y.bval[1];
  buffer[19] = accel_y.bval[2];
  buffer[20] = accel_y.bval[3];
  buffer[21] = accel_z.bval[0];
  buffer[22] = accel_z.bval[1];
  buffer[23] = accel_z.bval[2];
  buffer[24] = accel_z.bval[3];
  buffer_count = 25;
  return true;
}

void NxpAdafruit9Dof::handleData(uint8_t *data, uint8_t len){
    // This device does not accept data from the pi
}

#endif // NXPADA9DOF_ENABLE
