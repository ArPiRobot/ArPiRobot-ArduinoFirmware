from arduino import ArduinoUartInterface, PinMode, PinState
import time


arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")

# arduino.pinMode(10, PinMode.OUTPUT)
arduino.analogWrite(10, 253)
print("HERE")