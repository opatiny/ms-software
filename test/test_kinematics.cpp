#include <kinematics.h>
#include "../src/hardwareProperties.h"
#include "unity.h"

void test_countsToAngle(void) {
  TEST_ASSERT_EQUAL_FLOAT(360,
                          countsToAngle(ENCODER_COUNTS_PER_REV * GEAR_RATIO));
}

void test_computeWheelRpm(void) {
  TEST_ASSERT_EQUAL_FLOAT(60, computeWheelRpm(360, 1));
  TEST_ASSERT_EQUAL_FLOAT(600, computeWheelRpm(360, 0.1));
}
