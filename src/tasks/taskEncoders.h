#pragma once

#include <stdint.h>

#define LEFT_ENCODER_PIN1 D2
#define LEFT_ENCODER_PIN2 D3

typedef int64_t Encoder;

extern Encoder leftEncoderCounts;
extern Encoder rightEncoderCounts;

void taskEncodersX1();
void taskEncodersX4();
