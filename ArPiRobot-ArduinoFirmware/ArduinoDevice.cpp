/*
 * Copyright 2020 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>. 
 */

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


SingleEncoder::SingleEncoder(int pin, bool pullup)  : ArduinoDevice(3), pin(pin){
  if(pullup){
    pinMode(pin, INPUT_PULLUP);
  }else{
    pinMode(pin, INPUT);
  }
  lastState = digitalRead(pin);
  count.uival = 0;
}

void SingleEncoder::poll() {
  if(lastState == LOW){
    lastState = digitalRead(pin);
    if(lastState == HIGH){
      this->count.uival++;
      changed = true;
    }
  }
  if(lastState == HIGH){
    lastState = digitalRead(pin);
    if(lastState == LOW){
      this->count.uival++;
      changed = true;
    }
  }

  if(changed){
    buffer[0] = deviceId;

    // Buffer count little endian
    bufferValue16(count, true, buffer, 1);

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
  distance.uival = 0;
}

void Ultrasonic4Pin::poll(){
  bool shouldSend = false;
  pollIterationCounter++;

  if(pollIterationCounter >= ULTRASONIC_4PIN_POLL_RATE){
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
  
    uint16_t duration = pulseIn(echoPin, HIGH, 5000);
    uint16_t newDistance;

    if(duration > 0){
       newDistance = duration * 0.034 / 2;
    }else{
      newDistance = 999;
    }
    
    shouldSend = newDistance != distance.uival;
    distance.uival = newDistance;
    pollIterationCounter = 0;
  }

  if(shouldSend){
    buffer[0] = deviceId;

    // Buffer distance little endian
    bufferValue16(distance, true, buffer, 1);
    
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

  buffer[0] = deviceId;

  // Buffer each little endian
  bufferValue32(gyro_x, true, buffer, 1);
  bufferValue32(gyro_y, true, buffer, 5);
  bufferValue32(gyro_z, true, buffer, 9);
  bufferValue32(accel_x, true, buffer, 13);
  bufferValue32(accel_y, true, buffer, 17);
  bufferValue32(accel_z, true, buffer, 21);
  
  buffer_count = 25;
}

void OldAdafruit9Dof::handleData(uint8_t *data, uint8_t len){
    // This device does not accept data from the pi
}

#endif // OLDADA9DOF_ENABLE

VoltageMonitor::VoltageMonitor(uint8_t readPin, float vboard, uint32_t r1, uint32_t r2) : ArduinoDevice(5), readPin(readPin), vboard(vboard), r1(r1), r2(r2){
  pinMode(readPin, INPUT);

  // Zero the samples buffer
  for(uint8_t i = 0; i < AVG_READINGS; ++i){
    readings[i] = 0;
  }

  readingScaleFactor = vboard * (r1 + r2) / r2 / 1023 / AVG_READINGS;
}

void VoltageMonitor::poll(){
  readingRunningSum -= readings[readingIndex];
  readings[readingIndex] = analogRead(readPin);
  readingRunningSum += readings[readingIndex];
  readingIndex++;
  if(readingIndex >= AVG_READINGS) readingIndex = 0;
  
  voltage.fval = readingRunningSum * readingScaleFactor;

  // Don't need to send voltage data every iteration. Just slows down Pi to handle that much data.
  if(sendIterationCounter >= VMON_MAX_SEND_RATE){
    sendIterationCounter = 0;
    if(fabs(lastSentVoltage - voltage.fval) > .02){
      buffer[0] = deviceId;
    
      // Buffer voltage little endian
      bufferValue32(voltage, true, buffer, 1);
      
      buffer_count = 5;
      lastSentVoltage = voltage.fval;
    } 
  }else{
    sendIterationCounter++;
  }
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

  buffer[0] = deviceId;

  // Buffer each little endian
  bufferValue32(gyro_x, true, buffer, 1);
  bufferValue32(gyro_y, true, buffer, 5);
  bufferValue32(gyro_z, true, buffer, 9);
  bufferValue32(accel_x, true, buffer, 13);
  bufferValue32(accel_y, true, buffer, 17);
  bufferValue32(accel_z, true, buffer, 21);
  
  buffer_count = 25;
  return true;
}

void NxpAdafruit9Dof::handleData(uint8_t *data, uint8_t len){
    // This device does not accept data from the pi
}

#endif // NXPADA9DOF_ENABLE

IRReflectorModule::IRReflectorModule(uint8_t digitalPin, uint8_t analogPin) : ArduinoDevice(4), digitalPin(digitalPin), analogPin(analogPin){
  pinMode(digitalPin, INPUT);
  lastDigitalState = digitalRead(digitalPin);
  if(analogPin != 255)
    lastAnalogValue.uival = analogRead(analogPin);
}

void IRReflectorModule::poll(){
  uint8_t state = digitalRead(digitalPin);
  if(state != lastDigitalState){
    changed = true;
  }
  lastDigitalState = state == HIGH;

  if(analogPin != 255){
    uint16_t analogVal = analogRead(analogPin);
    if(analogVal != lastAnalogValue.uival){
      changed = true;
    }
    lastAnalogValue.uival = analogVal;
  }

  if(changed){
    buffer[0] = deviceId;

    // Invert this so 1 means reflection detected and 0 means no reflection detected. This way it matches the onboard LED
    buffer[1] = !lastDigitalState;

    if(analogPin != 255){
      bufferValue16(lastAnalogValue, true, buffer, 2);
      buffer_count = 4;
    }else{
      buffer_count = 2;
    }
    changed = false;
  }
}

void IRReflectorModule::handleData(uint8_t *data, uint8_t len){
  
}
