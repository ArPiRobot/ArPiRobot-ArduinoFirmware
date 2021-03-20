from arduino import ArduinoUartInterface, PinMode, PinState
import time


arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")

arduino.pinMode(10, PinMode.INPUT_PULLUP)
while True:
    res = arduino.digitalRead(10)
    print(res)
    time.sleep(1)
