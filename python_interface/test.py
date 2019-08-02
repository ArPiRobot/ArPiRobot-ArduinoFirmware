from device import ArduinoDevice
from interface import ArduinoUartInterface
from sensors import SingleEncoder, OldAdafruit9Dof, Ultrasonic4Pin, VoltageMonitor
import time
import threading
import sys


arduino = ArduinoUartInterface("COM7", 250000)

rlast_count = 0
llast_count = 0
last_distance = 0


arduino.reset()

renc = SingleEncoder.create(arduino, 2)
lenc = SingleEncoder.create(arduino, 3)
imu = OldAdafruit9Dof.create(arduino)
usonic = Ultrasonic4Pin.create(arduino, 7, 8)
vmon = VoltageMonitor.create(arduino, "A0", 3.3, 30000, 7500)

if renc is None or lenc is None or imu is None or usonic is None or vmon is None:
    print("Failed to create one or more sensors.")
    sys.exit(1)

print("Sensor processing started.")
arduino.start_processing()

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

        #print("AngleZ: " + str(imu.gyro_z))

        #print("Voltage: " + str(vmon.voltage))

        time.sleep(.01)
except KeyboardInterrupt:
    pass
