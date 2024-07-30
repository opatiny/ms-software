# Algernon micromouse software

This repository contains all of the software that was written for my custom micromouse robot. It is a PlatformIO project, based on an `esp32-s3-devkitc-1` board and using the Arduino framework.

To access all of the hardware files, as well as more documentation on this project, checkout the following repository: [opatiny/micromouse](https://github.com/opatiny/micromouse).

## Code structure

This project uses [FreeRTOS](https://www.freertos.org/index.html), a real time operating system for microcontroller. This allows us to have multiple tasks that are running concurrently. In our case, we have 15 different tasks. This project is initially based on the [hackuarium/esp32-s3](https://github.com/Hackuarium/esp32-c3) repository, which gave us a code base to work with. The tasks that were taken from that repository are in the `lib/Hack` folder, whereas our fully custom tasks are in `src/tasks`. Some of the tasks that were copied still had to be modified for our own application. All of the utilities that we developped were also place in the `lib` folder in `lib/utilities`

Here is the list of all the tasks that are running on our robot, as well as what they are responsible for.

Tasks reused from `hackuarium/esp32-s3`:

- [`taskBlink`](./lib/Hack/taskBlink.cpp): Lowest priority task, which makes an LED blink at all times. Allows to see if the device crashes.
- [`taskEventSourceSender`](./lib/Hack/taskEventSourceSender.cpp): Allows to post messages for different topics over WiFi.
- [`taskGY521`](./lib/Hack/taskGY521.cpp): Accelerometer task.
- [`taskSerial`](./lib/Hack/taskSerial.cpp): Handles the "serial parameters", a custom system to store variables in the EEPROM and read and write to them using the serial interface. More about in the next section.
- [`taskWebServer`](./lib/Hack/taskWebServer.cpp): Allows to open an HTTP connection over WiFi and continuously send data.
- [`taskWifi`](./lib/Hack/taskWifi.cpp): Responsible for managing the connection of the device to an existing WiFi rooter.
- [`taskWire`](./lib/Hack/taskWire.cpp): Manages the I2C devices, allows to scan the I2C buses to detect the connected devices.

Custom tasks:

- [`taskButton`](./lib/Hack/taskButton.cpp): Handles the push button.
- [`taskBuzzer`](./lib/Hack/taskBuzzer.cpp): Controls the buzzer, allows to make various sounds depending on the mode, which can be modified in any other task.
- [`taskCalibrateSpeed`](./lib/Hack/taskCalibrateSpeed.cpp): Task for an automatic speed calibration of the wheels. This allows to establish the feedforward controller for the wheels speed.
- [`taskEncodersX4`](./lib/Hack/taskEncodersX4.cpp): Handles the encoders data, counts the encoder pulses and computes the motors' speeds.
- [`taskOdometry`](./lib/Hack/taskOdometry.cpp): Allows to track the robot's position and orientation.
- [`taskRgbLed`](./lib/Hack/taskRgbLed.cpp): Control of the RBG LED brightness and color.
- [`taskRobotMove`](./lib/Hack/taskRobotMove.cpp): Manages the robot's speed. They are different control modes. Either the motors' duty cycle is set for each wheel, or a desired rpm speed is set for both wheels at the same time. Finally, the robot's speed can be controlled directly, using speed regulators.
- [`taskVL53L1X`](./lib/Hack/taskVL53L1X.cpp): Handles the 5 distance sensors.
- [`taskVoltage`](./lib/Hack/taskVoltage.cpp): Measurement of the battery voltage.

All of the tasks are called in the main program: `src/main.cpp`, which allows to easily enable or disable them as needed.

## More documentation

The `docs` folder of this repository contains more md file with useful information:

- [`setupAndTasks.md`](./docs/setupAndTasks.md): Explanation about how to set up platformio, as well as how some of the tasks were implemented.
- [`unitTesting.md`](./docs/unitTesting.md): Documentation on how to do basic cpp unit testing of functions.
- [`serialParameters.md`](./docs/serialParameters.md): Documentation about the serial interface of the robot.
