# ArPiRobot-ArduinoFirmware
Arduino firmware with command interface for use as an ArPiRobot sensor co-processor.

## Building & Uploading
- Download the source code from the releases tab
- Extract the project
- Open the project in the Arduino IDE
- Install required libraries (see list below) using library manager 
- Select board (see list of supported boards below) and port
- Click upload button


## Supported Boards
| Board               | Core                  | Notes                                                                   |
| ------------------- | --------------------- | ----------------------------------------------------------------------- |
| Arduino Uno & Nano  | Official AVR Core     | Includes compatible boards such as Adafruit Metro 328 & Metro Mini 328  |
| Arduino Nano Every  | Official megaAVR Core |                                                                         |
| Teensy 3.1 / 3.2    | [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) |                                   |
| Raspberry Pi Pico   | [arduino-pico](https://github.com/earlephilhower/arduino-pico) |                           |
| Feather S2          | [arduino-esp32](https://github.com/espressif/arduino-esp32) | Unexpected Maker Feather S2 board [feathers2.io](https://feathers2.io/) |

*Other boards will likely work, but may require changes to `board.h`. Most commonly, any board will work, however interrupts will not be available. By modifying `board.h`, interrupt support can be added and other board specific modifications / fixes can be made.*


## Required Libraries
- FastCRC (1.31+)


## License

```
ArPiRobot-ArduinoFirmware is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ArPiRobot-ArduinoFirmware is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with ArPiRobot-ArduinoFirmware.  If not, see <https://www.gnu.org/licenses/>.
```
