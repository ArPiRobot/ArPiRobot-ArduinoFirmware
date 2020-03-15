#include "settings.h"
#include "RPiInterface.h"

#ifdef INTERFACE_SW_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial swSerial(SW_SERIAL_RX, SW_SERIAL_TX);
#endif

RPiInterface *rpi;

void setup(){

#ifdef DEBUG
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
  DEBUG_SERIAL.println("At setup()");
#endif

#if defined(INTERFACE_HW_SERIAL) || defined(INTERFACE_TEENSY_USB_SERIAL)
  rpi = new RPiUartInterface(HW_SERIAL_PORT, HW_SERIAL_BAUD);
#endif
#ifdef INTERFACE_SW_SERIAL
  rpi = new RPiUartInterface(swSerial, SW_SERIAL_BAUD);
#endif

  ///////////////////////////////////////////////////////
  /// Add any static devices here
  ///////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////

  
  rpi->open();
  rpi->configure();
  delay(1000);
}

void loop(){  
	rpi->feed();
}
