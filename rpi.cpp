/*
 * Copyright 2021 Marcus Behel
 *
 * This file is part of ArPiRobot-ArduinoFirmware.
 * 
 * ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ArPiRobot-ArduinoFirmware. If not, see <https://www.gnu.org/licenses/>. 
 */

#if 0

#include "rpi.h"
#include "conversions.h"
#include "util.h"

#include <FastCRC.h>

////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////

uint8_t rpi_read_buffer[RPI_READ_BUFFER_SIZE];
uint16_t rpi_read_buffer_len;
bool rpi_read_started;
bool rpi_read_escaped;
FastCRC16 rpi_crc;


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

bool rpi_read_data(){
    if(PI_SERIAL.available()){
        uint8_t c = PI_SERIAL.read();

        if(rpi_read_escaped){
            // Ignore invalid escaped data
            if(c == RPI_START_BYTE || c == RPI_END_BYTE || c == RPI_ESCAPE_BYTE){
                rpi_read_buffer[rpi_read_buffer_len++] = c;
                if(rpi_read_buffer_len == RPI_READ_BUFFER_SIZE)
                    rpi_read_buffer_len = 0;
            }
            rpi_read_escaped = false;
        }else{
            if(c == RPI_START_BYTE){
                rpi_read_buffer_len = 0;
                rpi_read_started = true;
            }else if(c == RPI_END_BYTE && rpi_read_started){
                rpi_read_started = false;
                return true;
            }else if(c == RPI_ESCAPE_BYTE && rpi_read_started){
                rpi_read_escaped = true;
            }else if(rpi_read_started){
                rpi_read_buffer[rpi_read_buffer_len++] = c;
                if(rpi_read_buffer_len == RPI_READ_BUFFER_SIZE)
                    rpi_read_buffer_len = 0;
            }
        }
    }
    return false;
}

bool rpi_check_data(){
    uint16_t read_crc = convert_data_to_int16(&rpi_read_buffer[rpi_read_buffer_len - 2], false);
    uint16_t calc_crc = rpi_crc.ccitt(rpi_read_buffer, rpi_read_buffer_len - 2);
    if(read_crc == calc_crc){
        return true;
    }else{
        log_write("CRC!=\n");
        return false;
    }
}

void rpi_write_data(uint8_t *data, uint16_t len){

    // Messages are in the form [START_BYTE],[MESSAGE_DATA]...,[CRC_HIGH],[CRC_LOW],[END_BYTE]
    // If the MESSAGE_DATA contains a START_BYTE or END_BYTE, it is prefixed with an ESCAPE_BYTE
    // If the message contains an ESCAPE_BYTE, it is also prefixed with an ESCAPE_BYTE
    // If the CRC bytes are START, END, OR ESCAPE BYTES they are prefixed with ESCAPE_BYTE

    uint8_t i;
    PI_SERIAL.write(RPI_START_BYTE);
    for(i = 0; i < len; ++i){
        if(data[i] == RPI_START_BYTE || data[i] == RPI_END_BYTE || data[i] == RPI_ESCAPE_BYTE)
            PI_SERIAL.write(RPI_ESCAPE_BYTE);
        PI_SERIAL.write(data[i]);
    }

    uint16_t crc = rpi_crc.ccitt(data, len);
    uint8_t crc_data[2];
    convert_int16_to_data(crc, crc_data, false);
    if(crc_data[0] == RPI_START_BYTE || crc_data[0] == RPI_END_BYTE || crc_data[0] == RPI_ESCAPE_BYTE)
        PI_SERIAL.write(RPI_ESCAPE_BYTE);
    PI_SERIAL.write(crc_data[0]);
    if(crc_data[1] == RPI_START_BYTE || crc_data[1] == RPI_END_BYTE || crc_data[1] == RPI_ESCAPE_BYTE)
        PI_SERIAL.write(RPI_ESCAPE_BYTE);
    PI_SERIAL.write(crc_data[1]);

    PI_SERIAL.write(RPI_END_BYTE);
}

void rpi_init(){
    rpi_read_buffer_len = 0;
    rpi_read_started = false;
    rpi_read_escaped = false;
    PI_SERIAL.begin(PI_BAUD);
}

void rpi_process(){
    if(rpi_read_data() && rpi_check_data()){
        // Ignore CRC now
        rpi_read_buffer_len -= 2;

        // Data starting with '^' is sent to a device

        // Clear read buffer
        rpi_read_buffer_len = 0;
    }
}

#endif