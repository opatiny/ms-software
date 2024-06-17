#include <motorProperties.h>
#include <pidController.h>
#include "unity.h"

void test_pidController(void) {
  PidController controller;
  controller.params.kp = 0.5;
  controller.params.ki = 0;
  controller.params.kd = 0;

  TEST_ASSERT_EQUAL_FLOAT(0, getNewPidValue(&controller, 0));
  TEST_ASSERT_EQUAL_FLOAT(0.5, getNewPidValue(&controller, 1));

  controller.params.ki = 0.1;
  TEST_ASSERT_EQUAL_FLOAT(0.5, getNewPidValue(&controller, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.6, getNewPidValue(&controller, 1));

  controller.params.kd = 0.1;
  TEST_ASSERT_EQUAL_FLOAT(0.6, getNewPidValue(&controller, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.7, getNewPidValue(&controller, 1));
}
