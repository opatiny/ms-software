#include <kinematics.h>
#include "../src/hardwareProperties.h"
#include "Arduino.h"
#include "unity.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}
void test_countsToAngle(void) {
  TEST_ASSERT_EQUAL_FLOAT(360,
                          countsToAngle(ENCODER_COUNTS_PER_REV * GEAR_RATIO));
}

void test_computeWheelRpm(void) {
  TEST_ASSERT_EQUAL_FLOAT(60, computeWheelRpm(360, 1));
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_computeWheelRpm);
  RUN_TEST(test_countsToAngle);
  return UNITY_END();
}

/**
 * For Arduino framework
 */
void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);
  runUnityTests();
}
void loop() {}