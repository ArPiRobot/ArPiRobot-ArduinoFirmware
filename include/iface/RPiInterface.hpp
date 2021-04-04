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
 * along with ArPiRobot-ArduinoFirmware.    If not, see <https://www.gnu.org/licenses/>. 
 */

#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#include <FastCRC.h>

#include <board.h>
#include <device/ArduinoDevice.hpp>

/*
 * Interface for raspberry pi
 * The Pi sends commands to the arduino using the same method as described with writeData.
 * Commands (as ascii) include:
 *    "RESET": Clear configured devices and wait for more add device commands
 *    "END": Done configuring devices. Start sensor processing.
 *    "ADD[DEVICE_CODE][PARAMS]": Add a device based on the device code and its parameters (vary by device)
 * All data sent to the pi is sent as described with the writeData function (using a start and end byte escaping data)
 *    This includes responses to commands and the initial indication that the arduino is ready.
 *    
 * Any command starting with a hyphen ("-" ASCII 45) is data to be sent to a device and will be handled as such if 
 *    sensor processing has started (not in the configure stage)
 * 
 * The general communication will work something like:
 *    Arduino is reset on uart connection from Pi (if not like with teensy the Pi sends a reset command anyway)
 *    The arduino then writes a ready command "START" (using writeData method)
 *    The arduino waits for add commands from the pi. For each add command a sensor is created as described by the 
 *        parameters and added to the interfaces device list.
 *    The Pi then writes an "END" command
 *    The arduino responds with "END" (writeData method) then starts sensor processing
 *    The arduino sends sensor data as necessary until a RESET command is read (ASCII newline delimited).
 *    No other data should be sent form the Pi to the arduino.
 *    
 */


extern void(*reset) (void);

extern FastCRC16 CRC16;

class RPiInterface{
public:

    // Start, end, and escape byte for sending data from sensors
    const uint8_t START_BYTE = 253;
    const uint8_t END_BYTE = 254;
    const uint8_t ESCAPE_BYTE = 255;

    virtual ~RPiInterface();

    /**
     * Add a device to the pi from arduino code (not based on data received from the Pi)
     * This device will always exist. This must be done before calling run.
     * @param device A pointer to the device to add
     * @return The device id for the added device.
     */
    int16_t addStaticDevice(ArduinoDevice *device);

    /**
     * Add a device to the Pi interface. Device will be created from readBuffer data.
     * @return The device ID of the created device
     */
    int16_t addDevice();

    /**
     * Wait for the pi to add devices to the interface via commands, then service
     * sensors. This method never returns (blocks forever)
     */
    void run();

protected:

    // These functions are implemented by child classes using specific communication protocols

    /**
     * Open communication with the Pi
     */
    virtual void open() = 0;

    /**
     * Check the number of bytes available to read
     * @return Number of bytes available to read
     */
    virtual uint16_t available() = 0;

    /**
     * Read a byte
     * @return The byte read (-1 if no data to read)
     */
    virtual int16_t read() = 0;

    /**
     * Write a byte
     * @param data The byte to write
     */
    virtual void write(uint8_t data) = 0;

    /**
     * Flush write buffer
     */
    virtual void flush() = 0;

private:

    /** 
     * Read data handling escape sequences as described with writeData
     * @return true if a complete data set is in the readBuffer
     */
    bool readData();

    /**
     * Write some data to the Pi using the proper escape sequences.
     * 
     *  Data packets are in the following format
     *  start_byte, data..., crc16_high, crc16_low, end_byte
     * 
     * 
     * Where data varies in length and can contain start_byte, end_byte, and escape_byte
     * Before sent data is modified so that 
     *        start_byte is replaced with escape_byte, start_byte
     *        end_byte is replaced with escape_byte, end_byte
     *        escape_byte is replaced with escape_byte, escape_byte
     * The CRC16 (ccitt false) is calculated on the original unmodified (not escaped) data. The high and low bytes are
     *        modified the same way (escape sequences) if the values match start, end, or escape bytes
     * 
     * @param data The data to send
     * @param len The number of bytes to send
     */
    void writeData(uint8_t *data, uint16_t len);

    /**
     * Check that received data crc is valid (checks readBuffer)
     * @return true if valid, else false
     */
    bool checkData();

    /**
     * Checks if a set of data starts with another set of data
     * @param data1 The set of data to search in
     * @param len1 The length of data1
     * @param data2 The set of data to seach for in data1
     * @param len2 The length of data2
     * @return true if data1 starts with data2, else false
     */
    bool dataStartsWith(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2);

    /**
     * Checks if two sets of data match
     * @param data1 The first dataset
     * @param len1 Length of the first dataset
     * @param data2 The second dataset
     * @param len2 Length of the second dataset
     */
    bool dataDoesMatch(uint8_t *data1, uint16_t len1, uint8_t *data2, uint16_t len2);
    
    // Serial read buffer (reading data from the Pi)
    uint8_t readBuffer[DATA_READ_BUFFER_SIZE];
    uint16_t readBufferLen = 0;

    // Status of parsing data (readData also handles parsing)
    bool parseStarted = false;    // Has start byte
    bool parseEscaped = false;    // Is parse escaped
    
    // LinkedList<ArduinoDevice*> devices;
    ArduinoDevice *devices[10];
    uint8_t devicesLen = 0;
    uint8_t staticDeviceCount = 0;

    bool canAddStatic = true;
};
