from arduino import ArduinoUartInterface, PinMode, PinState
import time

arduino = ArduinoUartInterface("COM5", 115200)
print("ARDUINO READY")


def handle_data(data: bytearray):
    print(len(data))
    state, dt = arduino.parse_ana_read_status(data)
    print(state)


arduino.pinMode(10, PinMode.INPUT_PULLUP)
A0 = arduino.analogInputToDigitalPin(0)
id = arduino.startAutoAnalogRead(A0, 10, 50, handle_data)

if id == -1:
    print("FAILED TO START")
    exit()

print("STARTED")

while True:
    time.sleep(1)
