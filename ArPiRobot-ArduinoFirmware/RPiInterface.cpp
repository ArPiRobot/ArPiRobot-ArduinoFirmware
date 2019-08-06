#include "RPiInterface.h"


int RPiInterface::addDevice(String buf){
  if(deviceCount >= MAX_DEVICES){
    return -1;
  }
  if(buf.startsWith("ADD_SENC_")){
    String readPinStr = buf.substring(9, buf.length() - 1);
    int readPin = 0;
    if(readPinStr.startsWith("A"))
      readPin = analogInputToDigitalPin(readPinStr.substring(1).toInt());
    else
      readPin = readPinStr.toInt();    
    devices[deviceCount] = new SingleEncoder(readPin);
    deviceCount++;
    return deviceCount - 1;
  }else if(buf.equals("ADD_OLDADA9DOF\n")){
    devices[deviceCount] = new OldAdafruit9Dof();
    deviceCount++;
    return deviceCount - 1;
  }else if(buf.startsWith("ADD_USONIC4_")){
    String triggerPinStr = buf.substring(12, buf.indexOf("_", 12));
    String echoPinStr = buf.substring(buf.indexOf("_", 12) + 1, buf.length() - 1);
    devices[deviceCount] = new Ultrasonic4Pin(triggerPinStr.toInt(), echoPinStr.toInt());
    deviceCount++;
    return deviceCount - 1;
  }else if(buf.startsWith("ADD_VMON_")){
    String pinStr = buf.substring(9, buf.indexOf("_", 9));
    String vboardStr = buf.substring(10 + pinStr.length(), buf.indexOf("_", 10 + pinStr.length()));
    String r1Str = buf.substring(11 + pinStr.length() + vboardStr.length(), buf.indexOf("_", 11 + pinStr.length() + vboardStr.length()));
    String r2Str = buf.substring(12 + pinStr.length() + vboardStr.length() + r1Str.length(), buf.length() - 1);
    devices[deviceCount] = new VoltageMonitor(analogInputToDigitalPin(pinStr.substring(1).toInt()), vboardStr.toFloat(), r1Str.toInt(), r2Str.toInt());
    deviceCount++;
    return deviceCount - 1;
  }
  return -1;
}

void RPiInterface::reset(){
  for(uint8_t i = 0; i < deviceCount; ++i){
    delete devices[i];
  }
  ArduinoDevice::nextId = 0;
  deviceCount = 0;
  readBuffer = "";
}

void RPiInterface::configure(){
  print("START\n");
  String buf = "";
  while(true){
    if(available())
      buf += (char) read();

    if(buf.endsWith("\n")){
      if(buf.startsWith("ADD_")){
        int deviceId = addDevice(buf);
        if(deviceId == -1){
          print("ADDFAIL\n");
        }else{
          print("ADDSUCCESS_" + String(deviceId) + "\n");
        }
      }else if(buf.equals("END\n")){
        break;
      }else if(buf.equals("RESET\n")){
        reset();
        configure();
        break;
      }
      buf = "";
    }
    delay(10);
  }
  print("END\n");
}

void RPiInterface::feed(){
  unsigned long now = millis();
  for(uint8_t i = 0; i < deviceCount; ++i){
    bool shouldSend = devices[i]->poll(buffer, &len);
    if(shouldSend && now >= devices[i]->nextSendTime){
      write(devices[i]->deviceId);
      write(buffer, len);
      print("\n");
      flush();
      devices[i]->nextSendTime += SEND_RATE; // Send again
    }
  }

  while(available()){
    readBuffer += (char) read();
    if(readBuffer.endsWith("\n")){
      if(readBuffer.startsWith("RESET")){
        reset();
        configure();
        return;
      }
    }
  }
}


#if defined(INTERFACE_HW_SERIAL) || defined(INTERFACE_TEENSY_USB_SERIAL) || defined(INTERFACE_SW_SERIAL)

RPiUartInterface::RPiUartInterface(Serial_t &serial, uint32_t baud) : serial(serial), baud(baud){
  
}

void RPiUartInterface::open(){
  serial.begin(baud);
  while(!serial);
}

int RPiUartInterface::available(){
  return serial.available();
}

char RPiUartInterface::read(){
  return serial.read();
}

void RPiUartInterface::print(String data){
  serial.print(data);
}

void RPiUartInterface::write(uint8_t data){
  serial.write(data);
}

void RPiUartInterface::write(uint8_t *data, uint8_t len){
  serial.write(data, len);
}

void RPiUartInterface::flush(){
  serial.flush();
}

#endif
