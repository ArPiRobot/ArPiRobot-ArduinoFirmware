from device import ArduinoDevice
from interface import ArduinoUartInterface
from sensors import SingleEncoder, OldAdafruit9Dof, Ultrasonic4Pin
import time
import threading


arduino = ArduinoUartInterface("COM10", 250000)
renc = SingleEncoder(0)
lenc = SingleEncoder(1)
usonic = Ultrasonic4Pin(2)
imu = OldAdafruit9Dof(3)

rlast_count = 0
llast_count = 0
last_distance = 0

arduino.register_device(renc)
arduino.register_device(lenc)
arduino.register_device(usonic)
arduino.register_device(imu)

def feed_arduino():
    while True:
        arduino.feed()
        time.sleep(.005)

arduino_thread = threading.Thread(target=feed_arduino)
arduino_thread.daemon = True
arduino_thread.start()

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

    print("AngleZ: " + str(imu.gyro_z))

    time.sleep(.01)

