import serial
import time

class ArduinoInterface:
    INPUT = b'in'
    OUTPUT = b'out'
    INPUT_PULLUP = b'inpu'

    HIGH = b'1'
    LOW = b'0'

    def __init__(self, port: str, baud: int):
        self.s = serial.Serial(port, baud)
        r = b''
        while r != b'READY':
            r = self.read_response()


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


    def write_data(self, data: bytes):
        data.replace(b'\xff', b'\xff\xff')
        data.replace(b'\xfe', b'\xff\xfe')
        data.replace(b'\xfd', b'\xff\xfd')
        crc = self.crc16(data, 0, len(data)).to_bytes(2, 'big')
        crc.replace(b'\xff', b'\xff\xff')
        crc.replace(b'\xfe', b'\xff\xfe')
        crc.replace(b'\xfd', b'\xff\xfd')
        self.s.write(b'\xfd')
        self.s.write(data)
        self.s.write(crc)
        self.s.write(b'\xfe')

    def read_response(self):
        parse_started = False
        parse_escaped = False
        buffer = b''
        while True:
            if self.s.in_waiting:
                c = self.s.read()
                if parse_escaped:
                    if c == b'\xfd' or c == b'\xfe' or c == b'\xff':
                        buffer = buffer + c
                    parse_escaped = False
                else:
                    if c == b'\xfd':
                        parse_started = True
                        buffer = b''
                    elif c == b'\xfe' and parse_started:
                        parse_started = False
                        break
                    elif c == b'\xff' and parse_started:
                        parse_excaped = True
                    elif(parse_started):
                        buffer = buffer + c
        return buffer[0:-2]

    ############################################################################
    # Core GPIO methods
    ############################################################################

    def pinMode(self, pin: int, mode: bytes):
        self.write_data(b'GPPM,' + str(pin).encode() + b',' + mode)
        if(self.read_response() != b'SUCCESS'):
            self.pinMode(pin, mode)

    def digitalWrite(self, pin: int, state: bytes):
        self.write_data(b'GPDW,' + str(pin).encode() + b',' + state)
        if(self.read_response() != b'SUCCESS'):
            self.digitalWrite(pin, state)
    
    def digitalRead(self, pin: int) -> int:
        self.write_data(b'GPDR,' + str(pin).encode())
        res = self.read_response()
        if(res[0:7] == b'SUCCESS'):
            if(res[8:] == b'1'):
                return 1
            return 0
        else:
            return self.digitalRead(pin)
    
    def analogRead(self, pin: int) -> int:
        self.write_data(b'GPAR,' + str(pin).encode())
        res = self.read_response()
        if(res[0:7] == b'SUCCESS'):
            return int(res[8:].decode())
        else:
            return self.analogRead(pin)
    
    def analogInputToDigitalPin(self, pin: int) -> int:
        self.write_data(b'GPAD,' + str(pin).encode())
        res = self.read_response()
        if(res[0:7] == b'SUCCESS'):
            return int(res[8:].decode())
        else:
            return self.analogInputToDigitalPin(pin)
    
    def analogWrite(self, pin: int, value: int):
        self.write_data(b'GPAW,' + str(pin).encode() + b',' + str(value).encode())
        if(self.read_response() != b'SUCCESS'):
            self.analogWrite(pin, value)
    
    
    ############################################################################
    # GPIO automatic data acquisition
    ############################################################################

    def startAutoDigitalRead(self, pin: int):
        self.write_data(b'GPADR')


arduino = ArduinoInterface("COM3", 115200)

arduino.pinMode(3, arduino.OUTPUT)
arduino.analogWrite(3, 128)
while True:
    print(arduino.analogRead(arduino.analogInputToDigitalPin(1)))
    time.sleep(0.1)
