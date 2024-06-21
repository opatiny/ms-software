#include <motorProperties.h>
#include <pidController.h>
#include "unity.h"

void test_pidController(void) {
  PidController navigation;
  navigation.params.kp = 0.5;
  navigation.params.ki = 0;
  navigation.params.kd = 0;

  TEST_ASSERT_EQUAL_FLOAT(0, getNewPidValue(&navigation, 0));
  TEST_ASSERT_EQUAL_FLOAT(0.5, getNewPidValue(&navigation, 1));

  navigation.params.ki = 0.1;
  TEST_ASSERT_EQUAL_FLOAT(0.5, getNewPidValue(&navigation, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.6, getNewPidValue(&navigation, 1));

  navigation.params.kd = 0.1;
  TEST_ASSERT_EQUAL_FLOAT(0.6, getNewPidValue(&navigation, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.7, getNewPidValue(&navigation, 1));
}
