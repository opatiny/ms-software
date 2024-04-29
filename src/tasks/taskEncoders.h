#pragma once

#include <stdint.h>

typedef int64_t Encoder;

extern Encoder leftEncoderCounts;
extern Encoder rightEncoderCounts;

void taskEncodersX1();
void taskEncodersX4();
