from device import ArduinoDevice
import struct

class SingleEncoder(ArduinoDevice):
    def __init__(self, device_id: int):
        super().__init__(device_id)
        self.__count = 0

    @property
    def count(self):
        return self.__count

    def device_name(self):
        return "SingleEncoder"

    def handle_data(self, data: bytes):
        if len(data) >= 4:
            self.__count = struct.unpack_from('<H', data, offset=1)[0]

class OldAdafruit9Dof(ArduinoDevice):
    def __init__(self, device_id: int):
        super().__init__(device_id)
        self.__gyro_x = 0
        self.__gyro_y = 0
        self.__gyro_z = 0
        self.__accel_x = 0
        self.__accel_y = 0
        self.__accel_z = 0
        self.__pitch = 0
        self.__roll = 0
        self.__yaw = 0

    @property
    def gyro_x(self):
        return self.__gyro_x

    @property
    def gyro_y(self):
        return self.__gyro_y

    @property
    def gyro_z(self):
        return self.__gyro_z

    @property
    def accel_x(self):
        return self.__accel_x

    @property
    def accel_y(self):
        return self.__accel_y

    @property
    def accel_z(self):
        return self.__accel_z

    @property
    def pitch(self):
        return self.__pitch

    @property
    def roll(self):
        return self.__roll

    @property
    def yaw(self):
        return self.__yaw

    def device_name(self):
        return "OldAdafruit9Dof"

    def handle_data(self, data: bytes):
        if len(data) >= 38:
            self.__gyro_x = struct.unpack_from('<f', data, offset=1)[0]
            self.__gyro_y = struct.unpack_from('<f', data, offset=5)[0]
            self.__gyro_z = struct.unpack_from('<f', data, offset=9)[0]
            self.__accel_x = struct.unpack_from('<f', data, offset=13)[0]
            self.__accel_y = struct.unpack_from('<f', data, offset=17)[0]
            self.__accel_z = struct.unpack_from('<f', data, offset=21)[0]
            self.__pitch = struct.unpack_from('<f', data, offset=25)[0]
            self.__roll = struct.unpack_from('<f', data, offset=29)[0]
            self.__yaw = struct.unpack_from('<f', data, offset=33)[0]

class Ultrasonic4Pin(ArduinoDevice):
    def __init__(self, device_id):
        super().__init__(device_id)
        self.__distance = 0

    @property
    def distance(self) -> int:
        return self.__distance

    def device_name(self):
        return "Ultrasonic4Pin"

    def handle_data(self, data: bytes):
        if len(data) >= 4:
            self.__distance = struct.unpack_from('<H', data, offset=1)[0]

class VoltageMonitor(ArduinoDevice):
    def __init__(self, device_id: int):
        super().__init__(device_id)
        self.__voltage = 0

    @property
    def voltage(self):
        return self.__voltage

    def device_name(self):
        return "VoltageMonitor"

    def handle_data(self, data: bytes):
        if len(data) >= 6:
            self.__voltage = struct.unpack_from('<f', data, offset=1)[0]
