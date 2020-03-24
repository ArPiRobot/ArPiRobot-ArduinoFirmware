import serial
import struct
import time
import atexit
from crccheck.crc import Crc16CcittFalse

start_byte = b'\xfd'  # 253
end_byte = b'\xfe'    # 254
escape_byte = b'\xff' # 255

s = serial.Serial("/dev/ttyUSB0", 500000)
read_buffer = b''

atexit.register(s.close)

# Keep track of state as read_data function is called periodically
parse_started = False
parse_escaped = False

def read_data() -> bool:
    global parse_escaped, parse_started, read_buffer

    c = b''
    if s.in_waiting > 0:
        c = s.read()
    else:
        return False
    
    if(parse_escaped):
        # Ignore invalid escaped data
        if c == start_byte or c == end_byte or c == escape_byte:
            read_buffer += c
        parse_escaped = False # Past the next byte no longer escaped
    else:
        if c == start_byte:
            if parse_started:
                # Got another start byte. Trash the buffer
                read_buffer = b''
            parse_started = True
        elif c == end_byte and parse_started:
            parse_started = False
            return True # have complete dataset
        elif c == escape_byte and parse_started:
            parse_escaped = True
        elif parse_started:
            read_buffer += c

    # Not complete yet
    return False

def check_data() -> bool:
    global read_buffer
    read_crc = struct.unpack_from(">H", read_buffer, offset=len(read_buffer) - 2)[0]
    calc_crc = Crc16CcittFalse.calc(read_buffer[0:len(read_buffer) - 2])
    return read_crc == calc_crc

def write(b: bytes):
    s.write(b)

def write_data(data: bytes):
    global s
    write(start_byte)
    for i in range(len(data)):
        b = data[i:i+1]
        if b == start_byte:
            write(escape_byte)
            write(start_byte)
        elif b == end_byte:
            write(escape_byte)
            write(end_byte)
        elif b == escape_byte:
            write(escape_byte)
            write(escape_byte)
        else:
            write(b)
    crc = Crc16CcittFalse.calc(data)
    crc_buf = struct.pack(">H", crc)
    for i in range(len(crc_buf)):
        b = crc_buf[i:i+1]
        if b == end_byte or b == start_byte or b == escape_byte:
            write(escape_byte)
        write(b)
    write(end_byte)

# Arduino may already be resetting (most do, but some like teensy do not)
time.sleep(2)

# If the arduino reset on open it will have already sent a start command
# Trash it if it exists
s.read_all()

# Send reset command (in case working with an arduio that does not reset on open)
print("Sending reset command")
write_data(b'RESET')

# Wait for START command (indicating arduino is ready)
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'START'):
            break
    time.sleep(0.001)
read_buffer = b''
print("Arduino signaled ready")

# Add some devices and get their ids
print("Adding encoder on digital pin 2")
write_data(b'ADDSENC\x00\x02')
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'ADD'):
            break
    time.sleep(0.001)
if read_buffer.startswith(b'ADDSUCCESS'):
    print("Device ID is " + str(read_buffer[10]))
else:
    print("Failed to add device")
read_buffer = b''

print("Adding encoder on analog pin 1")
write_data(b'ADDSENC\x01\x01')
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'ADD'):
            break
    time.sleep(0.001)
if read_buffer.startswith(b'ADDSUCCESS'):
    print("Device ID is " + str(read_buffer[10]))
else:
    print("Failed to add device")
read_buffer = b''


print("Adding IMU")
write_data(b'ADDOLDADA9DOF')
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'ADD'):
            break
    time.sleep(0.001)
if read_buffer.startswith(b'ADDSUCCESS'):
    print("Device ID is " + str(read_buffer[10]))
else:
    print("Failed to add device")
read_buffer = b''

print("Adding USONIC4 on 7, 8")
write_data(b'ADDUSONIC4\x00\x07\x00\x08')
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'ADD'):
            break
    time.sleep(0.001)
if read_buffer.startswith(b'ADDSUCCESS'):
    print("Device ID is " + str(read_buffer[10]))
else:
    print("Failed to add device")
read_buffer = b''

print("Adding vmon(A2, 5.00, 30000, 7500)")
write_data(b'ADDVMON\x02' + struct.pack(">f", 4.95) + struct.pack(">I", 30000) + struct.pack(">I", 7500))
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'ADD'):
            break
    time.sleep(0.001)
if read_buffer.startswith(b'ADDSUCCESS'):
    print("Device ID is " + str(read_buffer[10]))
else:
    print("Failed to add device")
read_buffer = b''

# Send end command
print("Starting sensor processing.")
write_data(b'END')
while True:
    if(read_data() and check_data()):
        if read_buffer.startswith(b'END'):
            break
    time.sleep(0.001)
read_buffer = b''
print("Got confirmation that sensor processing is starting.")

# Test sending data to device
print("Testing send data to device")
write_data(b'-\x0A\x00\x01\x02')
