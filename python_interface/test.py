from device import ArduinoDevice
from interface import ArduinoUartInterface
from sensors import SingleEncoder, OldAdafruit9Dof, Ultrasonic4Pin, VoltageMonitor
import time
import threading
import sys


arduino = ArduinoUartInterface("COM7", 250000)


# Create devices
#arduino.write(b"RESET\n")
#arduino.write(b"ADD_OLDADA9DOF\n")
#arduino.write(b"ADD_USONIC4_7_8\n")
#arduino.write(b"ADD_VMON_A0_3.3_30000_7500\n")
#arduino.write(b"END\n")

#time.sleep(6)

#renc = SingleEncoder(0)
#lenc = SingleEncoder(1)
#imu = OldAdafruit9Dof(2)
#usonic = Ultrasonic4Pin(3)
#vmon = VoltageMonitor(4)

#arduino.register_device(renc)
#arduino.register_device(lenc)
#arduino.register_device(imu)
#arduino.register_device(usonic)
#arduino.register_device(vmon)

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
