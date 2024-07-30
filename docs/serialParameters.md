# Serial user interface

Our project is based on the [hackuarium/esp32-s3](https://github.com/Hackuarium/esp32-c3) repository, which implement a very useful debug serial interface, that we decided to reuse in this project.

## Storing variables

The system allows to store various state parameters in the EEPROM, as well as reading and setting them through the serial monitor. Each variable is encoded using a one or two upper case letters. Typing the code of a variable in the serial monitor and pressing enter allows to read the current value of the parameter.

**Example:** A + enter -> returns the current value of parameter A

On the other hand, the parameters can be set by following the variable code by the new value.

**Example:** A123 + enter -> sets variable A to 123

**Remark:** all variables are stored as `int16`, so the only acceptable values are integers in the range [-32768, 32767].

## Accessing the menu

The second main feature of the serial interface is a menu that allows to inspect the current status of the robot and display various debug information.
The navigation in the menu is done using lowercase letters.

Use the `h` command to see the help, with all possible commands.

## List of serial parameters

The three table below show all of the parameters that are stored in the serial parameters, as well as what they correspond to. These parameter can be read/set through the serial monitor.

| Parameter | Mode | Description                                    |
| --------- | ---- | ---------------------------------------------- |
| A         | R/W  | Choose what info to debug                      |
| B         | R/W  | Choose the debug mode for the distance sensors |
| C         | R/W  | Enable / disable the buzzer                    |
| D         | R/W  | Buzzer mode                                    |
| E         | R    | Current battery voltage                        |
| F         | R    | Current VCC voltage                            |
| G         | R/W  | RGB LED mode                                   |
| H         | R/W  | RGB LED color                                  |
| I         | R/W  | RGB LED brightness                             |
| J         | R    | Button mode (pressed or not pressed)           |
| K         | R/W  | Left motor command                             |
| L         | R/W  | Right motor command                            |
| M         | R/W  | Left motor mode                                |
| N         | R/W  | Right motor mode                               |
| O         | R/W  | Duration of the motors' acceleration           |
| P         | R/W  | Wheels command for basic robot control         |
| Q         | R/W  | Desired wheels speed in rpm                    |
| R         | R/W  | Desired robot's linear acceleration in mm/s    |
| S         | R/W  | Desired robot's angular acceleration in deg/s  |
| T         | R/W  | Robot mode                                     |
| U         | R/W  | Desired rotation angle in deg                  |
| V         | R/W  | Minimum distance to obstacle before stopping   |
| W         | R/W  | Speed calibration mode                         |
| X         | R/W  | Speed calibration command step                 |
| Y         |      |                                                |
| Z         |      |                                                |

| Parameter | Mode | Description                           |
| --------- | ---- | ------------------------------------- |
| AA        | R    | Acceleration along X in m/s^2 \* 100  |
| AB        | R    | Acceleration along Y in m/s^2 \* 100  |
| AC        | R    | Acceleration along Z in m/s^2 \* 100  |
| AD        | R    | Angular speed along X in rad/s \* 100 |
| AE        | R    | Angular speed along Y in rad/s \* 100 |
| AF        | R    | Angular speed along Z in rad/s \* 100 |
| AG        | R    | Left distance in mm                   |
| AH        | R    | Front-left distance in mm             |
| AI        | R    | Front distance in mm                  |
| AJ        | R    | Front-right distance in mm            |
| AK        | R    | Right distance in mm                  |
| AL        |      |                                       |
| AM        |      |                                       |
| AN        |      |                                       |
| AO        |      |                                       |
| AP        |      |                                       |
| AQ        |      |                                       |
| AR        |      |                                       |
| AS        |      |                                       |
| AT        |      |                                       |
| AU        |      |                                       |
| AV        |      |                                       |
| AW        |      |                                       |
| AX        |      |                                       |
| AY        |      |                                       |
| AZ        |      |                                       |

| Parameter | Mode | Description                                       |
| --------- | ---- | ------------------------------------------------- |
| BA        | R/W  | Enable / disable linear robot's speed controller  |
| BB        | R/W  | Enable / disable angular robot's speed controller |
| BC        | R/W  | Enable / disable wall following controller        |
| BD        | R/W  | Wheel speed controller, Kp                        |
| BE        | R/W  | Wheel speed controller, Ki                        |
| BF        | R/W  | Wheel speed controller, Kd                        |
| BG        | R/W  | Linear robot's speed speed controller, Kp         |
| BH        | R/W  | Linear robot's speed speed controller, Ki         |
| BI        | R/W  | Linear robot's speed speed controller, Kd         |
| BJ        | R/W  | Angular robot's speed controller, Kp              |
| BK        | R/W  | Angular robot's speed controller, Ki              |
| BL        | R/W  | Angular robot's speed controller, Kd              |
| BM        |      |                                                   |
| BN        |      |                                                   |
| BO        |      |                                                   |
| BP        |      |                                                   |
| BQ        |      |                                                   |
| BR        |      |                                                   |
| BS        |      |                                                   |
| BT        |      |                                                   |
| BU        |      |                                                   |
| BV        |      |                                                   |
| BW        |      |                                                   |
| BX        | R    | WiFi status                                       |
| BY        | R    | Current network RSSI                              |
| BZ        | R    | Checks if there is a serial error                 |
