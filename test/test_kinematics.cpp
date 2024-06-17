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
  TEST_ASSERT_EQUAL_DOUBLE(500, computeWheelRpm(30, 0.01));
}

void test_getLowSpeedRpm(void) {
  TEST_ASSERT_EQUAL_DOUBLE(100.0 / 6, getLowSpeedRpm(0.0001));
}