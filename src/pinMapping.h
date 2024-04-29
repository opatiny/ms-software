#pragma once

#define XIAO 0
#define ESPRESSIF 1

#define BOARD ESPRESSIF

#if BOARD == XIAO

#define BATTERY_PIN A0
#define BUTTON_PIN D2
#define BUZZER_PIN D0
#define LED_PIN D3

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

#elif BOARD == ESPRESSIF

#define BATTERY_PIN A8
#define BUTTON_PIN 47
#define BUZZER_PIN 48
#define LED_PIN LED_BUILTIN

// motor pins
#define MOTOR_LEFT_PIN1 36
#define MOTOR_LEFT_PIN2 35
#define MOTOR_RIGHT_PIN1 38
#define MOTOR_RIGHT_PIN2 37

#define LEFT_ENCODER_PIN1 40
#define LEFT_ENCODER_PIN2 39
#define RIGHT_ENCODER_PIN1 42
#define RIGHT_ENCODER_PIN2 41

// distance sensors shutdown pins: device is off when pin is low
#define XSHUT_PIN_LEFT 13
#define XSHUT_PIN_FRONT_LEFT 14
#define XSHUT_PIN_FRONT 12
#define XSHUT_PIN_FRONT_RIGHT 10
#define XSHUT_PIN_RIGHT 11

// I2C
#define SDA_PIN SDA
#define SCL_PIN SCL

#endif
