# Setting up prog environment on esp32-s3 on mac

## Linting / auto-formatting

- install the C/C++ vscode extension
- install clang-format with brew: `brew install clang-format`
- add settings to setting.json:

```
"C_Cpp.clang_format_style": "{BasedOnStyle: 'Chromium'}",

"[cpp]": {
    "editor.defaultFormatter": "ms-vscode.cpptools"
  },
```

## PlatformIO

Follow PlatformIO quickstart:https://docs.platformio.org/en/latest/integration/ide/vscode.html#quick-start

- when choosing the board take Xiaomi ESP32S3
- Framework: Arduino

## Multi-threading

We base ourself on this repository: https://github.com/Hackuarium/esp32-c3

- copy the `taskBlink.cpp` file
- in main
  - the `setup` function must call all the tasks
  - the `loop` function is empty and only contains a `vTaskDelay`

**Caution!!** Never use actual `delay()` because it will be blocking all the tasks! Use `vTaskDelay()` instead.

## Setup serial task and serial parameters

- start by copying the `taskSerial` files and the related useful files / tasks
- `globalConfig.h`: create a config file that will be called everywhere and that contains the list of all the serial parameters

## Adding dependencies arduino (libraries) to PIO project

- go to PlatformIO home page (click on little home in the bottom menu)
- in the left menu, click on Libraries
- search for the lib
- click on the one you need and then click on "Add to project"

## Setting up wifi task

WiFi task:

- copy the `taskWifi` file
- using the default serial parameters
- you can set the wifi name with ws+theWifiName (SSID) and the wp+theWifiPassword
- if the device doesn't try to connect after that, check that the NVS parameters are loaded in your main script!
- if it's not done, add the following line to the top of the `setup()` function: `setupParameters();`
- use `wi` to get wifi information

There is another task called `taskWifiAP` which allows the esp to appear as an access point to which we can connect.

## Access the `pio` command

- to access all executables that are installed by the platformio extension
- add this line to .zshrc : `export PATH=$PATH:~/.platformio/penv/bin`

With this, we can for instance dump a folder in the microcontroller memory. An example of this can be found in the next section.
If you have problems with the pio commands, check that all serial monitors are closed, because pio is also using serial.

## Setting up webserver task

- first copy the `taskWebserver.cpp` file

- create a folder at the first level called `data`.

To upload the data, run this command in a terminal. The esp must be connected to the computer. The data must be uploaded every time the folder is modified. This loads the contents of the folder onto the esp file system.

```
pio run --target uploadfs
```

To access the page, you must know the device IP address and be connected to the same wifi. The address can be known using the `wi` serial command.
Url example: `192.168.1.193`

- to check that the device is connected to the wifi, try to `ping` it from a terminal
- in a browser: just paste the IP address

Alternatively, the page can be built locally. Right-click on the html file and choose "Open with live server". You can then see the page and still interact with the device. For this to work, you have to set `localDevelopment = true` in `index.html`.

In the data folder, we create an `index.html` file, which will be served by the esp by default.

There is a way to update files and code over the air with the OTA (over the air update). We did not configure this though.

## Event handler task

- It is a task that allows to open an HTTP connection to the device and hence regularly update some data without closing the connection every time (the device can upload variables every x seconds).
- copy the `taskEventSource` file
- we added the `events.html` page that currently shows the accelerometer data

## Battery votage task

- we want to read an analog voltage on a pin
- we create voltage divider on the battery in order to have voltage that won't exceed max acceptable voltage (around 1.1 V)  
   https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html
  use analogReadMilis
