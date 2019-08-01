#include "settings.h"
#include "Arduino.h"
#include "ArduinoDevice.h"

// Sensor data buffer
uint8_t buffer[255];
uint8_t len;

ArduinoDevice *devices[255];
uint8_t deviceCount = 0;

void setup(){
  Serial.begin(250000);
  
	// Configuration stage
	devices[0] = new SingleEncoder(2);
  devices[1] = new SingleEncoder(3);
  devices[2] = new Ultrasonic4Pin(7, 8);
  devices[3] = new OldAdafruit9Dof();
  devices[4] = new VoltageMonitor(A0, 3.33, 471000, 410000);
	deviceCount = 5;
}

void loop(){
	unsigned long now = millis();
	for(uint8_t i = 0; i < deviceCount; ++i){
		bool shouldSend = devices[i]->poll(buffer, &len);
		if(shouldSend && now >= devices[i]->nextSendTime){
      Serial.write(devices[i]->deviceId);
			Serial.write(buffer, len);
      Serial.write('\n');
			devices[i]->nextSendTime += SEND_RATE; // Send again
		}
	}
}
