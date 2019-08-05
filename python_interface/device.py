from abc import ABC, abstractmethod
import time

millis = lambda: int(round(time.time() * 1000))

class ArduinoDevice(ABC):
    def __init__(self, device_id: int):
        self.__device_id = device_id

    @staticmethod
    def await_add_response(arduino) -> int:
        start_time = millis()
        while millis() - start_time <= 3000:
            response = arduino.readline()
            print(response)
            if response.startswith(b'ADDSUCCESS'):
                try:
                    device_id = int(response[11:len(response)-1].decode())
                    return device_id
                except:
                    return -1
            elif response.startswith(b'ADDFAIL'):
                return -1
        return -1
    
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
