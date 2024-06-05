#pragma once
#include <Preferences.h>
#include <regressions.h>

typedef const char* PolynomKeys[NB_COEFF];

extern PolynomKeys negLeftKeys;
extern PolynomKeys posLeftKeys;
extern PolynomKeys negRightKeys;
extern PolynomKeys posRightKeys;

extern Preferences preferences;

void saveWheelsRegressions(Regressions* left, Regressions* right);
void loadWheelsRegressions(Regressions* left, Regressions* right);