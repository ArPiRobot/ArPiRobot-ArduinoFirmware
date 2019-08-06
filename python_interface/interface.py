import serial
from abc import ABC, abstractmethod
from device import ArduinoDevice
import struct
import time


millis = lambda: int(round(time.time() * 1000))


class ArduinoInterface(ABC):
    def __init__(self):
        self.open()
        self.__devices = []

    def reset(self, timeout_ms: int = 1000) -> bool:
        self.write(b'RESET\n')
        return self.wait_for_ready(timeout_ms)

    def start_processing(self, timeout_ms: int = 1000):
        self.write(b'END\n')
        while not self.readline() == b'END\n':
            if millis() - start_time >= timeout_ms:
                return False
        return True

    def wait_for_ready(self, timeout_ms: int = 5000) -> bool:
        start_time = millis()
        while not self.readline() == b'START\n':
            if millis() - start_time >= timeout_ms:
                return False
        return True
    
    @abstractmethod
    def open(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def readline(self) -> bytes:
        pass

    @abstractmethod
    def write(self, data: bytes):
        pass

    def register_device(self, device: ArduinoDevice):
        if not device in self.__devices:
            self.__devices.append(device)
    
    def feed(self):
        data_line = self.readline()
        if data_line != b'':
            for device in self.__devices:
                if len(data_line) >= 1 and device.device_id == data_line[0]:
                    device.handle_data(data_line)
                    break


class ArduinoUartInterface(ArduinoInterface):
    def __init__(self, port: str, baud: int):
        self.__port = port
        self.__baud = baud
        self.__serial = None
        super().__init__()

    def open(self):
        self.__serial = serial.Serial(self.__port, self.__baud, timeout=1)

    def close(self):
        self.__serial.close()

    def readline(self) -> bytes:
        return self.__serial.readline()
    
    def write(self, data: bytes):
        self.__serial.write(data)
