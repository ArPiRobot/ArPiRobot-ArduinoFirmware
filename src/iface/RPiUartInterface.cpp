#include <iface/RPiUartInterface.hpp>

RPiUartInterface::RPiUartInterface(HW_SERIAL_T &serial, uint32_t baud) : serial(serial), baud(baud){
    
}

void RPiUartInterface::open(){
    serial.begin(baud);
}

uint16_t RPiUartInterface::available(){
    return serial.available();
}

int16_t RPiUartInterface::read(){
    return serial.read();
}

void RPiUartInterface::write(uint8_t data){
    serial.write(data);
}

void RPiUartInterface::flush(){
    serial.flush();
}
