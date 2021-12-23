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

#include "settings.h"

// Prevents string literals from wasting RAM / flash if DEBUG_SERIAL is not defined
#ifdef DEBUG_SERIAL
    #define log_message(x)      log_internal(x)
    #define log_char(x)         log_char_internal(x)
    #define log_int(x)          log_int_internal(x)
    #define log_float(x)        log_float_internal(x)
    #define log_double(x)       log_double_internal(x)
#else
    #define log_message(x)
    #define log_char(x) 
    #define log_int(x)
    #define log_float(x)
    #define log_double(x)
#endif


/**
 * Log a message
 * @param message The message to log (string)
 */
void log_internal(const char *message);

/**
 * Log a character
 * @param c The character to log
 */
void log_char_internal(char c);

/**
 * Write an integer's ASCII representation to the log
 * @param value The integer to write
 */
void log_int_internal(int value);

/**
 * Write an float's ASCII representation to the log
 * @param value The float to write
 */
void log_float_internal(float value);

/**
 * Write an double's ASCII representation to the log
 * @param value The double to write
 */
void log_double_internal(double value);

