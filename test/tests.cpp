#include "Arduino.h"
#include "unity.h"

#include "./tests.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
} /**
   * For Arduino framework
   */

int runUnityTests(void) {
  UNITY_BEGIN();
  // kinematics
  RUN_TEST(test_computeWheelRpm);
  RUN_TEST(test_countsToAngle);
  RUN_TEST(test_getLowSpeedRpm);

  // pid
  RUN_TEST(test_pidController);

  // regressions
  RUN_TEST(test_findIndex);
  RUN_TEST(test_findIndex2);
  RUN_TEST(test_findZeros);
  // RUN_TEST(test_curveFitting);
  RUN_TEST(test_polyVal);
  //  RUN_TEST(test_findRegressions);

  return UNITY_END();
}

void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);
  runUnityTests();
}
void loop() {}