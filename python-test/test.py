from arduino import ArduinoUartInterface, PinMode, PinState
import time


arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")

arduino.pinMode(13, PinMode.OUTPUT)
while True:
    arduino.digitalWrite(13, PinState.HIGH)
    time.sleep(1)
    arduino.digitalWrite(13, PinState.LOW)
    time.sleep(1)
