#include "eeprom.h"
#include <printUtilities.h>

#define PREFERENCES_NAMESPACE "calibration"

PolynomKeys negLeftKeys = {"negLeft0", "negLeft1", "negLeft2", "negLeft3",
                           "negLeft4"};
PolynomKeys posLeftKeys = {"posLeft0", "posLeft1", "posLeft2", "posLeft3",
                           "posLeft4"};
PolynomKeys negRightKeys = {"negRight0", "negRight1", "negRight2", "negRight3",
                            "negRight4"};
PolynomKeys posRightKeys = {"posRight0", "posRight1", "posRight2", "posRight3",
                            "posRight4"};

Preferences preferences;

/**
 * @brief Save the polynom coefficients in the EEPROM at the given keys.
 */
void savePolynom(Polynom* polynom, PolynomKeys keys) {
  for (int i = 0; i < NB_COEFF; i++) {
    preferences.putDouble(keys[i], (*polynom)[i]);
  }
}

/**
 * @brief Save the regressions coefficients of both wheels in the EEPROM.
 */
void saveWheelsRegressions(Regressions* left, Regressions* right) {
  preferences.begin(PREFERENCES_NAMESPACE, false);
  savePolynom(&left->pNeg, negLeftKeys);
  savePolynom(&left->pPos, posLeftKeys);
  savePolynom(&right->pNeg, negRightKeys);
  savePolynom(&right->pPos, posRightKeys);
  preferences.end();
}

/**
 * @brief Load the polynom coefficients from the EEPROM at the given keys. Fills
 * the coefficients with 0 if the keys are not found.
 */
void loadPolynom(Polynom* polynom, PolynomKeys keys) {
  for (int i = 0; i < NB_COEFF; i++) {
    (*polynom)[i] = preferences.getDouble(keys[i], 0);
  }
}

void loadWheelsRegressions(Regressions* left, Regressions* right) {
  preferences.begin(PREFERENCES_NAMESPACE, true);
  loadPolynom(&left->pNeg, negLeftKeys);
  loadPolynom(&left->pPos, posLeftKeys);
  loadPolynom(&right->pNeg, negRightKeys);
  loadPolynom(&right->pPos, posRightKeys);
  preferences.end();
}
