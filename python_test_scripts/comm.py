import serial
from crccheck.crc import Crc16CcittFalse
import time
import struct


START_BYTE = b'\xfd'
END_BYTE = b'\xfe'
ESCAPE_BYTE = b'\xff'


s = None

def comm_init(port: str, baud: int):
    global s
    s = serial.Serial(port, baud)

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
                read_crc = struct.unpack_from(">H", data, len(data) - 2)[0]
                calc_crc = Crc16CcittFalse.calc(data[0:len(data) - 2])
                if read_crc == calc_crc:
                    return bytes(data[0:len(data)-2])
            elif c == ESCAPE_BYTE and parse_started:
                parse_escaped = True
            elif parse_started:
                data.extend(c)
    