from arduino import ArduinoUartInterface, PinMode, PinState
import time


arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")

A0 = arduino.analogInputToDigitalPin(0)

while True:
    print("Read: {0}".format(arduino.analogRead(A0)))
    time.sleep(1)
