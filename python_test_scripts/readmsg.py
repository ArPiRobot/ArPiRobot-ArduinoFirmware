from comm import comm_init, wait_for_data, write_data

comm_init("COM4", 57600)
while True:
    msg = wait_for_data()
    print(msg)