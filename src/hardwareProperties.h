#pragma once

// motors and wheels properties
#define GEAR_RATIO 30              // motor gear ratio
#define ENCODER_COUNTS_PER_REV 12  // encoder counts per revolution
#define WHEEL_BASE 103             // in mm, distance between centers of wheels
#define WHEEL_DIAMETER 32          // in mm

#define COUNTS_PER_REV (GEAR_RATIO * ENCODER_COUNTS_PER_REV)

/**
 * Distance per count of the encoder in mm.
 */
#define DISTANCE_PER_COUNT (WHEEL_DIAMETER * PI / COUNTS_PER_REV)