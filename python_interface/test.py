from device import ArduinoDevice
from interface import ArduinoUartInterface
from sensors import SingleEncoder, OldAdafruit9Dof, Ultrasonic4Pin, VoltageMonitor
import time
import threading
import sys


rlast_count = 0
llast_count = 0
last_distance = 0


arduino = ArduinoUartInterface("COM10", 250000)

print("Waiting for arduino to become ready...", end="")
if arduino.wait_for_ready(5000):
    print("OK")
else:
    print("Failed.")
    sys.exit(1)

print("Resetting arduino...", end="")
if arduino.reset():
    print("OK")
else:
    print("Failed.")
    sys.exit(1)

print("Creating right encoder...", end="")
renc = SingleEncoder.create(arduino, 2)
if renc is None:
    print("Failed.")
    sys.exit(1)
print("OK")

print("Creating left encoder...", end="")
lenc = SingleEncoder.create(arduino, 3)
if lenc is None:
    print("Failed.")
    sys.exit(1)
print("OK")

print("Creating Old Adafruit 9DOF IMU...", end="")
imu = OldAdafruit9Dof.create(arduino)
if imu is None:
    print("Failed.")
    sys.exit(1)
print("OK")

print("Creating ultrasonic sensor...", end="")
usonic = Ultrasonic4Pin.create(arduino, 7, 8)
if usonic is None:
    print("Failed.")
    sys.exit(1)
print("OK")

print("Creating voltage monitor...", end="")
vmon = VoltageMonitor.create(arduino, "A0", 3.3, 30000, 7500)
if vmon is None:
    print("Failed.")
    sys.exit(1)
print("OK")


print("Starting sensor processing...", end="")
if not arduino.start_processing():
    print("Failed.")
    sys.exit(1)
print("OK")

def feed_arduino():
    while True:
        arduino.feed()

arduino_thread = threading.Thread(target=feed_arduino)
arduino_thread.daemon = True
arduino_thread.start()

try:
    while True:
        if renc.count != rlast_count:
            print("RightEncoder: " + str(renc.count))
            rlast_count = renc.count
        
        if lenc.count != llast_count:
            print("LeftEncoder: " + str(lenc.count))
            llast_count = lenc.count

        if usonic.distance != last_distance:
            print("Distance: " + str(usonic.distance))
            last_distance = usonic.distance

        # print("AngleZ: " + str(imu.gyro_z))
        # print("Voltage: " + str(vmon.voltage))

        time.sleep(.01)
except KeyboardInterrupt:
    pass
