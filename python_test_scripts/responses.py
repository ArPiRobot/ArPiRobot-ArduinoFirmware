from comm import comm_init, wait_for_data, write_data
import time


comm_init("COM4", 57600)
time.sleep(2)
write_data(b'Testing123')
msg = wait_for_data()
print(msg)