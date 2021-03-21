from arduino import ArduinoUartInterface, PinMode, PinState
import time

arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")


count = 0

def handle_data(data: bytearray):
    global count
    new_count, dt = arduino.parse_poll_dig_count_status(data)
    count += new_count
    print(count)


arduino.pinMode(10, PinMode.INPUT_PULLUP)
id = arduino.startAutoPollingDigitalCount(10, 5, 50, handle_data)

if id == -1:
    print("FAILED TO START")
    exit()

print("STARTED")

while True:
    time.sleep(1)
