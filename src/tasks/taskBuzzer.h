#pragma once

#include <pwmWrite.h>

#define BUZZER_PIN D0

enum BuzzerModes {
  BUZZER_OFF = 0,
  BUZZER_SINGLE_NOTE = 1,
  BUZZER_ALARM = 2,
  BUZZER_SCALE = 3,
};

const int notes = 8;
const note_t cScale[notes] = {NOTE_C, NOTE_D, NOTE_E, NOTE_F,
                              NOTE_G, NOTE_A, NOTE_B, NOTE_C};
const int cScaleOctaves[notes] = {4, 4, 4, 4, 4, 4, 4, 5};

void TaskBuzzer();
