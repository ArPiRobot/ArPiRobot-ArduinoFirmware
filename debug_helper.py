import serial
from crccheck.crc import Crc16CcittFalse
import time


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


s = serial.Serial("COM3", 57600)


def write_data(data: bytes):
    global s, START_BYTE, END_BYTE, ESCAPE_BYTE
    
    s.write(START_BYTE)
    data.replace(START_BYTE, ESCAPE_BYTE + START_BYTE)
    data.replace(END_BYTE, ESCAPE_BYTE + END_BYTE)
    data.replace(ESCAPE_BYTE, ESCAPE_BYTE + ESCAPE_BYTE)
    s.write(data)

    crc = Crc16CcittFalse.calcbytes(data)
    crc.replace(START_BYTE, ESCAPE_BYTE + START_BYTE)
    crc.replace(END_BYTE, ESCAPE_BYTE + END_BYTE)
    crc.replace(ESCAPE_BYTE, ESCAPE_BYTE + ESCAPE_BYTE)
    s.write(crc)

    s.write(END_BYTE)
    
def wait_for_data() -> bytes:
    global s, START_BYTE, END_BYTE, ESCAPE_BYTE

    data = bytearray()
    parse_escaped = False
    parse_started = False

    while True:
        c = s.read()
        if parse_escaped:
            if c == START_BYTE or c == END_BYTE or c == ESCAPE_BYTE:
                data.extend(c)
            parse_escaped = False
        else:
            if c == START_BYTE:
                if parse_started:
                    data = bytearray()
                parse_started = True
            elif c == END_BYTE and parse_started:
                parse_started = False
                return bytes(data)
            elif c == ESCAPE_BYTE and parse_started:
                parse_escaped = True
            elif parse_started:
                data.extend(c)

time.sleep(2)
write_data(b'RESET')

while True:
    msg = wait_for_data()
    print(msg)
    if(msg.startswith(b'START')):
        write_data(b'END')

