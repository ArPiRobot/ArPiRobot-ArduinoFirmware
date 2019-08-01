from device import ArduinoDevice
from interface import ArduinoUartInterface
from sensors import SingleEncoder, OldAdafruit9Dof, Ultrasonic4Pin, VoltageMonitor
import time
import threading


arduino = ArduinoUartInterface("COM7", 250000)


# Create devices
arduino.write(b"RESET\n")
arduino.write(b"ADD_SENC_2\n")
arduino.write(b"ADD_SENC_3\n")
arduino.write(b"ADD_OLDADA9DOF\n")
arduino.write(b"ADD_USONIC4_7_8\n")
arduino.write(b"ADD_VMON_A0_3.3_30000_7500\n")
arduino.write(b"END\n")


time.sleep(6)


renc = SingleEncoder(0)
lenc = SingleEncoder(1)
imu = OldAdafruit9Dof(2)
usonic = Ultrasonic4Pin(3)
vmon = VoltageMonitor(4)

rlast_count = 0
llast_count = 0
last_distance = 0

arduino.register_device(renc)
arduino.register_device(lenc)
arduino.register_device(imu)
arduino.register_device(usonic)
arduino.register_device(vmon)

def feed_arduino():
    while True:
        arduino.feed()
        time.sleep(.005)

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

        print("Voltage: " + str(vmon.voltage))

        time.sleep(.01)
except KeyboardInterrupt:
    pass
