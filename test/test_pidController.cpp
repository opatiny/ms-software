#include <motorProperties.h>
#include <pidController.h>
#include "unity.h"

void test_pidController(void) {
  PidController controller;
  PidParams pidParams = {0.5, 0, 0};
  initialisePidController(&controller, &pidParams);

  TEST_ASSERT_EQUAL_FLOAT(0, getNewPidValue(&controller, 0));
  TEST_ASSERT_EQUAL_FLOAT(0.5, getNewPidValue(&controller, 1));

  controller.ki = 0.1;
  TEST_ASSERT_EQUAL_FLOAT(0.5, getNewPidValue(&controller, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.6, getNewPidValue(&controller, 1));

  controller.kd = 0.1;
  TEST_ASSERT_EQUAL_FLOAT(0.6, getNewPidValue(&controller, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.7, getNewPidValue(&controller, 1));
}
