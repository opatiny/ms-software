#include <kinematics.h>
#include <motorProperties.h>
#include "unity.h"

void test_countsToAngle(void) {
  TEST_ASSERT_EQUAL_DOUBLE(360,
                           countsToAngle(ENCODER_COUNTS_PER_REV * GEAR_RATIO));
}

void test_computeWheelRpm(void) {
  TEST_ASSERT_EQUAL_DOUBLE(60, computeWheelRpm(360, 1));
  TEST_ASSERT_EQUAL_DOUBLE(600, computeWheelRpm(360, 0.1));
  TEST_ASSERT_EQUAL_DOUBLE(120, computeWheelRpm(30, 0.01));
}
