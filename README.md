# ArPiRobot-ArduinoFirmware
Arduino firmware with command interface for use as an ArPiRobot sensor co-processor.

## Building & Uploading
This firmware is a [platformio](https://platformio.org/) project and is **not** compatible with the Arduino IDE. To build this project, you will need to install the platformio extension in [VSCode](https://code.visualstudio.com/).

Once the project is opened in VSCode, select the profile for the board you are using (bottom bar). If you are using a board that is not listed, add the configuration in the `platformio.ini` file. Once selected, click the check mark icon on the bottom bar to build the firmware.

To upload, select the correct `upload_port` in the `platformio.ini` file, then use the arrow icon to upload to the specified board.

## Settings
The `settings.h` file contains many settings for the firmware, some of which may be beneficial to change in some cases. Most of the time, the only settings you may want to change will be the enabled sensors. On some devices with less flash, it may be a good idea to disable some sensors you will never use to reduce flash storage usage of the firmware.

Aside from the `setting.h` file, there is sometimes a reason to modify a few lines in `main.cpp`. You can edit the instantiated RPiInterface to use a different UART port and you can add static devices to the interface in the setup function.

## License

**Note: This license does not apply to third party libraries in the `lib` folder. Each of these libraries is distributed under the terms of its own license (included with the libraries).**

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