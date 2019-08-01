import serial
import struct

port = serial.Serial("COM7", 250000)

last_distance = 0

while True:
    if port.in_waiting:
        data = port.readline()
        device_id = data[0]
        if device_id == 4:
            if len(data) >= 6:
                voltage = struct.unpack_from("<f", data, offset=1)[0]
                print(voltage, end=" V\n")
