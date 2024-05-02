#pragma once

#define XIAO 0
#define ESPRESSIF 1

#define BOARD ESPRESSIF

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

// I2C
#define SDA_PIN SDA
#define SCL_PIN SCL

// serial communication
#define SERIAL_SPEED 115200

#elif BOARD == ESPRESSIF

#define BATTERY_PIN 20
#define BUTTON_PIN 47
#define BUZZER_PIN 48
#define BLINK_LED_PIN 2
#define RGB_LED_PIN 38

// motor pins
#define MOTOR_LEFT_PIN1 36   // CANT BE USED -> MEMORY
#define MOTOR_LEFT_PIN2 35   // CANT BE USED -> MEMORY
#define MOTOR_RIGHT_PIN1 38  // CANT BE USED -> RGB LED
#define MOTOR_RIGHT_PIN2 37  // CANT BE USED -> MEMORY

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

// I2C
#define SDA_PIN SDA
#define SCL_PIN SCL

// serial communication
#define SERIAL_SPEED 115200

#endif
