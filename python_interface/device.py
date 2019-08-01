from abc import ABC, abstractmethod

class ArduinoDevice(ABC):
    def __init__(self, device_id: int):
        self.__device_id = device_id

    @property
    def device_id(self):
        return self.__device_id

    @property
    @abstractmethod
    def device_name(self) -> str:
        pass

    @abstractmethod
    def handle_data(self, data: bytes):
        pass
