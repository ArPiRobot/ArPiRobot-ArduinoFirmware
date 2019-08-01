#include "settings.h"
#include "Arduino.h"
#include "ArduinoDevice.h"

// Sensor data buffer
uint8_t buffer[255];
uint8_t len;

// Serial read buffer
String readBuffer = "";

ArduinoDevice *devices[255];
uint8_t deviceCount = 0;

int addDevice(String buf){
  if(buf.startsWith("ADD_SENC")){
    String readPinStr = buf.substring(9, buf.length() - 1);
    int readPin = 0;
    if(readPinStr.startsWith("A"))
      readPin = analogInputToDigitalPin(readPinStr.substring(1).toInt());
    else
      readPin = readPinStr.toInt();

    devices[deviceCount] = new SingleEncoder(readPin);
    deviceCount++;
    return deviceCount - 1;
  }else if(buf.startsWith("ADD_OLDADA9DOF")){
    devices[deviceCount] = new OldAdafruit9Dof();
    deviceCount++;
    return deviceCount - 1;
  }else if(buf.startsWith("ADD_USONIC4_7_8")){
    String triggerPinStr = buf.substring(12, buf.indexOf("_", 12));
    String echoPinStr = buf.substring(buf.indexOf("_", 12) + 1, buf.length() - 1);
    devices[deviceCount] = new Ultrasonic4Pin(triggerPinStr.toInt(), echoPinStr.toInt());
    deviceCount++;
    return deviceCount - 1;
  }else if(buf.startsWith("ADD_VMON")){
    String pinStr = buf.substring(9, buf.indexOf("_", 9));
    String vboardStr = buf.substring(10 + pinStr.length(), buf.indexOf("_", 10 + pinStr.length()));
    String r1Str = buf.substring(11 + pinStr.length() + vboardStr.length(), buf.indexOf("_", 11 + pinStr.length() + vboardStr.length()));
    String r2Str = buf.substring(12 + pinStr.length() + vboardStr.length() + r1Str.length(), buf.indexOf("_", 12 + pinStr.length() + vboardStr.length() + r1Str.length()));
    devices[deviceCount] = new VoltageMonitor(analogInputToDigitalPin(pinStr.substring(1).toInt()), vboardStr.toFloat(), r1Str.toInt(), r2Str.toInt());
    deviceCount++;
    return deviceCount - 1;
  }
  return -1;
}

void reset(){
  for(uint8_t i = 0; i < deviceCount; ++i){
    delete devices[i];
  }
  ArduinoDevice::nextId = 0;
  deviceCount = 0;
  readBuffer = "";
}

void configure(){
  Serial.print("START\n");
  String buf = "";
  while(true){
    if(Serial.available())
      buf += (char) Serial.read();

    if(buf.endsWith("\n")){
      if(buf.startsWith("ADD")){
        int deviceId = addDevice(buf);
        if(deviceId == -1){
          Serial.print("ADDFAIL\n");
        }else{
          Serial.print("ADDSUCCESS_" + String(deviceId) + "\n");
        }
      }else if(buf.startsWith("END")){
        break;
      }
      buf = "";
    }
    delay(10);
  }
}

void setup(){
  Serial.begin(250000);

  while(!Serial);

  configure();

  Serial.print("END\n");

  delay(1000);
}

void loop(){  
	unsigned long now = millis();
	for(uint8_t i = 0; i < deviceCount; ++i){
		bool shouldSend = devices[i]->poll(buffer, &len);
		if(shouldSend && now >= devices[i]->nextSendTime){
      Serial.write(devices[i]->deviceId);
			Serial.write(buffer, len);
      Serial.write('\n');
      Serial.flush();
			devices[i]->nextSendTime += SEND_RATE; // Send again
		}
	}

  while(Serial.available()){
    readBuffer += (char) Serial.read();
    if(readBuffer.endsWith("\n")){
      if(readBuffer.startsWith("RESET")){
        reset();
        configure();
        return;
      }
      readBuffer = "";
    }
  }
}
