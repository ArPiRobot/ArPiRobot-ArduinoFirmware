import serial
from abc import ABC, abstractmethod
from device import ArduinoDevice
import struct


class ArduinoInterface(ABC):
    def __init__(self):
        self.open()
        self.__devices = []

    def reset(self):
        self.write(b'RESET\n')

    def start_processing(self):
        self.write(b'END\n')
    
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
