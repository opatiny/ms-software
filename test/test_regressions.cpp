#include <Arduino.h>
#include <regressions.h>
#include "../src/hardwareProperties.h"
#include "curveFitting.h"
#include "unity.h"

void test_findIndex(void) {
  DataArray x = {-4, -2, 0, 2, 4};
  TEST_ASSERT_EQUAL_INT(1, findMinIndex(x, -2));
  TEST_ASSERT_EQUAL_INT(3, findMaxIndex(x, 2));
}

void test_findIndex2(void) {
  DataArray x = {0.00,  -500.00, -500.00, -500.00, -333.33, 0.00,   0.00,
                 0.00,  0.00,    333.33,  500.00,  500.00,  500.00, -0.00,
                 -0.00, -0.00,   -0.00,   -0.00,   -0.00,   -0.00,  -0.00};
  TEST_ASSERT_EQUAL_INT(1, findMinIndex(x, -550));
  TEST_ASSERT_EQUAL_INT(12, findMaxIndex(x, 550));
}

void test_findZeros(void) {
  DataArray x = {0.00,  -500.00, -500.00, -500.00, -333.33, 0.00,   0.00,
                 0.00,  0.00,    333.33,  500.00,  500.00,  500.00, -0.00,
                 -0.00, -0.00,   -0.00,   -0.00,   -0.00,   -0.00,  -0.00};
  TEST_ASSERT_EQUAL_INT(5, findMinZero(x));
  TEST_ASSERT_EQUAL_INT(8, findMaxZero(x));
}

void test_getRegressions(void) {
  Regressions reg;
  DataArray x = {-5, -4, -3, -2, -1, 0, 0, 0, 1, 2, 3, 4, 5};
  DataArray y = {-5, -4, -3, -2, -1, 0, 0, 0, 1, 2, 3, 4, 5};
  Serial.println("test_findRegressions");
  getRegressions(&reg, x, y, 3);
  Serial.println("after getRegressions");

  Regressions expected = {.pNeg = {0, 0, 0, 0, 0}, .pPos = {0, 0, 0, 0, 0}};
  TEST_ASSERT_EQUAL_DOUBLE_ARRAY(expected.pNeg, reg.pNeg, POLYNOM_DEGREE + 1);
}

/**
 * Test the curve fitting function
 * Polynom coefficients are stored from the highest degree to the lowest.
 */
void test_curveFitting(void) {
  int degree = 1;
  int nbCoeff = degree + 1;
  double polynom[nbCoeff];

  DataArray x = {0, 1, 2, 3, 4, 5};
  DataArray y = {0, 1, 2, 3, 4, 5};
  fitCurve(degree, 6, x, y, nbCoeff, polynom);

  double expected[2] = {1, 0};

  TEST_ASSERT_EQUAL_DOUBLE_ARRAY(expected, polynom, nbCoeff);
}

void test_polyVal(void) {
  double polynom[2] = {0, 1};
  TEST_ASSERT_EQUAL_DOUBLE(1, polyVal(polynom, 1, 1));
  double p2[3] = {1, 2, 3};
  TEST_ASSERT_EQUAL_DOUBLE(6, polyVal(p2, 2, 1));
}