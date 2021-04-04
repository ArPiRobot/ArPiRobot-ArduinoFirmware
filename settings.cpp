#include "settings.h"

#ifdef ARPFW_DEBUG

#include <SoftwareSerial.h>
#include <Arduino.h>

SoftwareSerial debugSer(2, 3); // RX, TX

#endif