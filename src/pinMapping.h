#pragma once

#define XIAO 0
#define ALGERNON_V1_0_0 1
#define ALGERNON_V1_1_0 2

#define BOARD ALGERNON_V1_1_0

#if BOARD == XIAO

#define BATTERY_PIN A0
#define BUTTON_PIN D2
#define BUZZER_PIN D0
#define BLINK_LED_PIN D3

// motor pins
#define MOTOR_LEFT_PIN1 D9
#define MOTOR_LEFT_PIN2 D10
#define MOTOR_RIGHT_PIN1 1
#define MOTOR_RIGHT_PIN2 2

#define LEFT_ENCODER_PIN1 D2
#define LEFT_ENCODER_PIN2 D3

// distance sensors shutdown pins: device is off when pin is low
#define XSHUT_PIN_LEFT D8
#define XSHUT_PIN_FRONT_LEFT D7
#define XSHUT_PIN_FRONT 0
#define XSHUT_PIN_FRONT_RIGHT 0
#define XSHUT_PIN_RIGHT 0

#define NB_DISTANCE_SENSORS 2

// I2C
#define SDA_PIN SDA
#define SCL_PIN SCL

// serial communication
#define SERIAL_SPEED 115200

#elif BOARD == ALGERNON_V1_0_0

#define BATTERY_PIN 20
#define BUTTON_PIN 47
#define BUZZER_PIN 6
#define BLINK_LED_PIN 2
#define RGB_LED_PIN 38

// motor pins
#define MOTOR_LEFT_PIN1 36
#define MOTOR_LEFT_PIN2 35
#define MOTOR_RIGHT_PIN1 0
#define MOTOR_RIGHT_PIN2 37

#define LEFT_ENCODER_PIN1 21
#define LEFT_ENCODER_PIN2 19
#define RIGHT_ENCODER_PIN1 41
#define RIGHT_ENCODER_PIN2 42

// distance sensors shutdown pins: device is off when pin is low
#define XSHUT_PIN_LEFT 13
#define XSHUT_PIN_FRONT_LEFT 14
#define XSHUT_PIN_FRONT 12
#define XSHUT_PIN_FRONT_RIGHT 10
#define XSHUT_PIN_RIGHT 11

#define NB_DISTANCE_SENSORS 4

// I2C
#define SDA_PIN 4
#define SCL_PIN 5

// serial communication
#define SERIAL_SPEED 115200

#elif BOARD == ALGERNON_V1_1_0

#define BATTERY_PIN 3  // battery voltage
#define VCC_PIN 48     // 3.3 V
#define BUTTON_PIN 17
#define BUZZER_PIN 6
#define BLINK_LED_PIN 2
#define RGB_LED_PIN 38

// motor pins
#define MOTOR_LEFT_PIN1 46
#define MOTOR_LEFT_PIN2 9
#define MOTOR_RIGHT_PIN1 10
#define MOTOR_RIGHT_PIN2 11

#define LEFT_ENCODER_PIN1 5
#define LEFT_ENCODER_PIN2 4
#define RIGHT_ENCODER_PIN1 13
#define RIGHT_ENCODER_PIN2 14

// distance sensors shutdown pins: device is off when pin is low
#define XSHUT_PIN_LEFT 40
#define XSHUT_PIN_FRONT_LEFT 39
#define XSHUT_PIN_FRONT 0
#define XSHUT_PIN_FRONT_RIGHT 20
#define XSHUT_PIN_RIGHT 19

#define NB_DISTANCE_SENSORS 5

// I2C
#define SDA_PIN 47
#define SCL_PIN 21

// serial communication
#define SERIAL_SPEED 115200
#endif
