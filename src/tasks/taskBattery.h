#pragma once

#define BATTERY_PIN A0

// volage divider resistors
#define BATTERY_R1 9.96
#define BATTERY_R2 1.02

#define VBAT(Vmes) (Vmes * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2)

void taskBattery();