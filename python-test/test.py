from arduino import ArduinoUartInterface, PinMode, PinState
import time


arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")

arduino.pinMode(10, PinMode.INPUT_PULLUP)

obj = arduino.startAutoDigitalRead(10)

if obj == None:
    print("FAILED TO START")
    exit()

print("STARTED")

while True:
    print(obj.current_state)
    time.sleep(1)
