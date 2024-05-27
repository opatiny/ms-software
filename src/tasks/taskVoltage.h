#pragma once

// volage divider resistors in kOhm
#define BATTERY_R1 10
#define BATTERY_R2 1

#define VCC_R1 10
#define VCC_R2 1

// battery parameters
#define ONE_CELL_MIN_VOLTAGE 2.6
#define NB_CELLS_SERIES 2
#define BATTERY_VOLTAGE_MARGIN 0.4

/**
 * Minimum battery voltage in mV. Use to warn the user that the battery is
 * empty. Around 5.6 V.
 */
#define BATTERY_EMPTY \
  ((ONE_CELL_MIN_VOLTAGE * NB_CELLS_SERIES + BATTERY_VOLTAGE_MARGIN) * 1000)

// voltage on Vin from which warning is triggered
#define VCC_WARNING 1
void taskVoltage();