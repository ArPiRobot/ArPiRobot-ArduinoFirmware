from arduino import ArduinoUartInterface, PinMode, PinState
import time

arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")


count = 0

def handle_status(data: bytearray):
    duration = arduino.parse_poll_pulsein_status(data)
    distance = duration * 0.034 / 2
    print("{0} cm".format(distance))


arduino.pinMode(10, PinMode.INPUT_PULLUP)
id = arduino.startAutoPollingPulsein(7, True, 10, 8, True, 100, 20, handle_status)

if id == -1:
    print("FAILED TO START")
    exit()

print("STARTED")

while True:
    time.sleep(1)
