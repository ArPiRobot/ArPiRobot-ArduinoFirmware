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

#pragma once

#include <Arduino.h>

// Supported device types
#define DEVICE_VOLTAGE_MONITOR                      0
#define DEVICE_SINGLE_ENCODER                       1
#define DEVICE_ULTRASONIC_4                         2
#define DEVICE_IR_DETECTOR                          3
#define DEVICE_OLDADA9DOF                           4
#define DEVICE_NXPADA9DOF                           5
#define DEVICE_MPU6060                              6


// Holds configuration information for a device
typedef struct {
    uint8_t type;
    uint8_t primary_pin;
    uint8_t secondary_pin;
    bool uses_interrupt;
} Device;


