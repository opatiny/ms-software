#pragma once

// motors and wheels properties
#define GEAR_RATIO 30              // motor gear ratio
#define ENCODER_COUNTS_PER_REV 12  // encoder counts per revolution
#define WHEEL_BASE 0.103           // in m, distance between centers of wheels
#define WHEEL_DIAMETER 0.032       // in m

#define COUNTS_PER_REV (GEAR_RATIO * ENCODER_COUNTS_PER_REV)

/**
 * Distance per count of the encoder in meters.
 */
#define DISTANCE_PER_COUNT (WHEEL_DIAMETER * PI / COUNTS_PER_REV)