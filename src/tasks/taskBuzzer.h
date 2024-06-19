#pragma once

#include <pwmWrite.h>

#define NB_REPETITIONS_ALARM 4

enum SoundMode {
  SOUND_OFF,  // 0
  SOUND_ON    // 1
};

enum BuzzerModes {
  BUZZER_OFF,          // 0
  BUZZER_SINGLE_NOTE,  // 1
  BUZZER_ALARM,        // 2
  BUZZER_SCALE,        // 3
  BUZZER_BOOT          // 4
};

const int nbNotesScale = 8;
const note_t scale[nbNotesScale] = {NOTE_C, NOTE_D, NOTE_E, NOTE_F,
                                    NOTE_G, NOTE_A, NOTE_B, NOTE_C};
const int scaleOctaves[nbNotesScale] = {4, 4, 4, 4, 4, 4, 4, 5};

const int nbNotesBoot = 3;
const note_t bootNotes[nbNotesBoot] = {NOTE_C, NOTE_E, NOTE_G};
const int bootNotesLengths[nbNotesBoot] = {50, 50, 100};

void taskBuzzer();
