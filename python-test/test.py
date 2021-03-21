from arduino import ArduinoUartInterface, PinMode, PinState
import time


arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")

arduino.pinMode(13, PinMode.OUTPUT)
arduino.digitalWrite(13, PinState.LOW)

A0 = arduino.analogInputToDigitalPin(0)
obj = arduino.startAutoAnalogRead(A0, 10, 50)

if obj == None:
    print("FAILED TO START")
    exit()

print("STARTED")

while True:
    if(obj.current_state >= 500):
        arduino.digitalWrite(13, PinState.HIGH)
    else:
        arduino.digitalWrite(13, PinState.LOW)
    time.sleep(0.05)
