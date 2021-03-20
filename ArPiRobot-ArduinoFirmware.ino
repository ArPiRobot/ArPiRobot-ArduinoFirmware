#include "comm.hpp"

UartComm<HardwareSerial> comm(Serial, 115200);

void setup() {
  // startComm();
}

void loop() {
  // handleIncomingData();

  // handlePeriodicDigitalReads() = periodically read digital pins
  // hanlePeriodicAnalogReads() = periodically read analog pins
  // handleCountInterrupts() = count based on interrupts
  // handleTimeInterrupts() = time based on digitalWrite then interrupt pulse
  
  // handlePeriodicPollCounts = count based on polling digitalRead
  // handlePeriodicPulsein = time based on digitalWrite then pulsein
}
