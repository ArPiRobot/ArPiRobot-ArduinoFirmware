import serial
import struct
import sys
from threading import Thread, Event, Lock
from enum import IntEnum
from abc import ABC, abstractmethod


class MessageType(IntEnum):
    COMMAND = 0
    RESPONSE = 1
    STATUS = 2


class Command(IntEnum):
    PIN_MODE = 0,
    DIGITAL_WRITE = 1
    DIGITAL_READ = 2
    ANALOG_READ = 3
    ANALOG_WRITE = 4
    ANALOG_INPUT_TO_DIGITAL_PIN = 5


class ErrorCode(IntEnum):
    TIMEOUT = -1
    NONE = 0
    INVALID_ARG = 1
    NOT_ENOUGH_ARGS = 2
    EXECUTION = 3
    UNKNOWN_COMMAND = 4


class PinMode(IntEnum):
    OUTPUT = 0
    INPUT = 1
    INPUT_PULLUP = 2


class PinState(IntEnum):
    LOW = 0
    HIGH = 1


class Response:
    def __init__(self, error_code: int, response_data: bytearray):
        self.error_code = error_code
        self.response_data = response_data


class ArduinoInterface(ABC):

    # TODO: Thread safety
    # Lock mutex before sending command
    # Unlock mutex after receiving result
    # Internal data read thread would ignore this lock

    OUTPUT = 0
    INPUT = 1
    INPUT_PULLUP = 2

    LOW = 0
    HIGH = 1

    START_BYTE = b'\xfd'
    END_BYTE = b'\xfe'
    ESCAPE_BYTE = b'\xff'

    def __init__(self):
        self.__parse_started = False
        self.__parse_escaped = False
        self.__read_buffer = bytearray()
        self.open()

        # Wait for "READY" message from arduino
        while True:
            if self.read_data():
                if self.check_data():
                    if self.__read_buffer[0:-2] == b'READY':
                        break
                self.__read_buffer.clear()
        self.__read_buffer.clear()
        
        self.__last_response = None
        self.__response_ready = Event()
        self.__read_thread = Thread(target=self.run_forever, daemon=True)
        self.__read_thread.start()

        # Not ok to have multiple threads running commands at the same time
        self.__cmd_lock = Lock()

    def __del__(self):
        self.close()
    

    ############################################################################
    # GPIO Commands
    ############################################################################

    def pinMode(self, pin: int, mode: PinMode):
        with self.__cmd_lock:
            msg = bytearray()
            msg.extend(int(MessageType.COMMAND).to_bytes(1, 'big'))
            msg.extend(int(Command.PIN_MODE).to_bytes(1, 'big'))
            msg.extend(pin.to_bytes(1, 'big'))
            msg.extend(int(mode).to_bytes(1, 'big'))
            self.write_data(msg)
            res = self.wait_for_response()
            if res.error_code != 0:
                self.print_error(sys._getframe().f_code.co_name, res.error_code)
    
    def digitalWrite(self, pin: int, state: PinState):
        with self.__cmd_lock:
            msg = bytearray()
            msg.extend(int(MessageType.COMMAND).to_bytes(1, 'big'))
            msg.extend(int(Command.DIGITAL_WRITE).to_bytes(1, 'big'))
            msg.extend(pin.to_bytes(1, 'big'))
            msg.extend(int(state).to_bytes(1, 'big'))
            self.write_data(msg)
            res = self.wait_for_response()
            if res.error_code != 0:
                self.print_error(sys._getframe().f_code.co_name, res.error_code)
    
    def digitalRead(self, pin: int) -> PinState:
        with self.__cmd_lock:
            msg = bytearray()
            msg.extend(int(MessageType.COMMAND).to_bytes(1, 'big'))
            msg.extend(int(Command.DIGITAL_READ).to_bytes(1, 'big'))
            msg.extend(pin.to_bytes(1, 'big'))
            self.write_data(msg)
            res = self.wait_for_response()
            if res.error_code != 0:
                self.print_error(sys._getframe().f_code.co_name, res.error_code)
                return PinState.LOW
            print(res.response_data)
            return PinState(res.response_data[0])

    def analogWrite(self, pin: int, pwm: int):
        with self.__cmd_lock:
            msg = bytearray()
            msg.extend(int(MessageType.COMMAND).to_bytes(1, 'big'))
            msg.extend(int(Command.ANALOG_WRITE).to_bytes(1, 'big'))
            msg.extend(pin.to_bytes(1, 'big'))
            msg.extend(int(pwm).to_bytes(1, 'big'))
            self.write_data(msg)
            res = self.wait_for_response()
            if res.error_code != 0:
                self.print_error(sys._getframe().f_code.co_name, res.error_code)
    
    def analogRead(self, pin: int) -> int:
        with self.__cmd_lock:
            msg = bytearray()
            msg.extend(int(MessageType.COMMAND).to_bytes(1, 'big'))
            msg.extend(int(Command.ANALOG_READ).to_bytes(1, 'big'))
            msg.extend(pin.to_bytes(1, 'big'))
            self.write_data(msg)
            res = self.wait_for_response()
            if res.error_code != 0:
                self.print_error(sys._getframe().f_code.co_name, res.error_code)
                return 0
            return struct.unpack_from('>H', res.response_data)[0]
    
    def analogInputToDigitalPin(self, pin: int) -> int:
        with self.__cmd_lock:
            msg = bytearray()
            msg.extend(int(MessageType.COMMAND).to_bytes(1, 'big'))
            msg.extend(int(Command.ANALOG_INPUT_TO_DIGITAL_PIN).to_bytes(1, 'big'))
            msg.extend(pin.to_bytes(1, 'big'))
            self.write_data(msg)
            res = self.wait_for_response()
            if res.error_code != 0:
                self.print_error(sys._getframe().f_code.co_name, res.error_code)
                return PinState.LOW
            return res.response_data[0]

    ############################################################################
    # Communication functions
    ############################################################################

    def run_forever(self):
        while True:
            # read_data is blocking (I/O)
            if self.read_data():
                if self.check_data():
                    if(self.__read_buffer[0] == MessageType.RESPONSE):
                        self.__last_response = self.get_response()
                        self.__response_ready.set()
                    else:
                        # TODO: Handle status messages
                        pass
                self.__read_buffer.clear()

    def clear_response(self):
        self.__response_ready.clear()
    
    def wait_for_response(self, timeout_sec: float = 1):
        # Wait for last response to be set (using a threading Event to avoid busy loop)
        if self.__response_ready.wait(timeout_sec):
            return self.__last_response
        return Response(ErrorCode.TIMEOUT, b'')

    def print_error(self, fcn: str, error_code: ErrorCode):
        if error_code == ErrorCode.TIMEOUT:
            print("Error running {0}. Timed out.".format(fcn))
        elif error_code == ErrorCode.INVALID_ARG:
            print("Error running {0}. Invalid argument(s).".format(fcn))
        elif error_code == ErrorCode.NOT_ENOUGH_ARGS:
            print("Error running {0}. Not enough arguments.".format(fcn))
        elif error_code == ErrorCode.EXECUTION:
            print("Error running {0}. Failed to execute command.".format(fcn))
        elif error_code == ErrorCode.UNKNOWN_COMMAND:
            print("Error running {0}. Unknown command.".format(fcn))
        else:
            print("Error running {0}. Unknown error ({1}).".format(fcn, error_code))

    def crc16(self, data : bytearray, offset , length):
        if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
            return 0
        crc = 0xFFFF
        for i in range(0, length):
            crc ^= data[offset + i] << 8
            for j in range(0,8):
                if (crc & 0x8000) > 0:
                    crc =(crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
        return crc & 0xFFFF

    def get_response(self) -> Response:
        return Response(self.__read_buffer[1], self.__read_buffer[2:-2])

    def write_data(self, data: bytearray):
        self.write(self.START_BYTE)
        for i in range(len(data)):
            b = data[i:i+1]
            if b == self.START_BYTE or b == self.END_BYTE or b == self.ESCAPE_BYTE:
                self.write(self.ESCAPE_BYTE)
            self.write(b)
        crc = self.crc16(data, 0, len(data)).to_bytes(2, 'big')
        for i in range(len(crc)):
            b = crc[i:i+1]
            if b == self.START_BYTE or b == self.END_BYTE or b == self.ESCAPE_BYTE:
                self.write(self.ESCAPE_BYTE)
            self.write(b)
        self.write(self.END_BYTE)
    
    def read_data(self) -> bool:
        # Blocks until a byte is read
        c = self.read()
        if self.__parse_escaped:
            if c == self.START_BYTE or c == self.END_BYTE or c == self.ESCAPE_BYTE:
                self.__read_buffer.extend(c)
            self.__parse_escaped = False
        else:
            if c == self.START_BYTE:
                self.__parse_started = True
                self.__read_buffer.clear()
            elif c == self.END_BYTE and self.__parse_started:
                self.__parse_started = False
                return True
            elif c == self.ESCAPE_BYTE and self.__parse_started:
                self.__parse_escaped = True
            else:
                self.__read_buffer.extend(c)

    def check_data(self) -> bool:
        read_crc = struct.unpack_from(">H", self.__read_buffer, offset=len(self.__read_buffer) - 2)[0]
        calc_crc = self.crc16(self.__read_buffer, 0, len(self.__read_buffer) - 2)
        return read_crc == calc_crc

    ############################################################################
    # Implementation specific communication functions
    ############################################################################

    @abstractmethod
    def open(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def available(self) -> int:
        pass

    @abstractmethod
    def read(self) -> bytes:
        pass

    @abstractmethod
    def write(self, b: bytes):
        pass


class ArduinoUartInterface(ArduinoInterface):
    def __init__(self, port: str, baud: int) -> None:
        self.__serial = serial.Serial(port, baud)
        super().__init__()
    
    def open(self):
        if not self.__serial.is_open:
            self.__serial.open()

    def close(self):
        self.__serial.close()

    def available(self) -> int:
        return self.__serial.in_waiting

    def read(self) -> bytes:
        return self.__serial.read()

    def write(self, b: bytes):
        self.__serial.write(b)
    

