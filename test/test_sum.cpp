#include "../src/sum.cpp"
#include "Arduino.h"
#include "unity.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_sum(void) {
  TEST_ASSERT_EQUAL_INT(3, sum(1, 2));
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_sum);
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