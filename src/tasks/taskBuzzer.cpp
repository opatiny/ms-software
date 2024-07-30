/**
 * Thread for the control of the buzzer.
 *
 * Buzzer mode: T
 * Debug: A9
 */

#include <globalConfig.h>
#include <pinMapping.h>
#include <utilities/params.h>

#include "./taskBuzzer.h"

void playNote(note_t note, int octave, int duration);

Pwm buzzer = Pwm();  // constructor

void TaskBuzzer(void* pvParameters) {
  buzzer.attach(BUZZER_PIN);

  int repetitionsAlarm = 0;
  setAndSaveParameter(PARAM_BUZZER_MODE, BUZZER_OFF);
  int previousMode = BUZZER_OFF;
  while (true) {
    if (getParameter(PARAM_DEBUG) == DEBUG_BUZZER &&
        getParameter(PARAM_BUZZER_MODE) != previousMode) {
      Serial.print("Buzzer mode: ");
      Serial.println(getParameter(PARAM_BUZZER_MODE));
      previousMode = getParameter(PARAM_BUZZER_MODE);
    }
    switch (getParameter(PARAM_BUZZER_MODE)) {
      case BUZZER_OFF:
        // duration of zero turns output off
        buzzer.note(BUZZER_PIN, NOTE_E, 3, 0, 0);
        vTaskDelay(10);
        break;
      case BUZZER_SINGLE_NOTE:
        playNote(NOTE_C, 4, 10);
        setParameter(PARAM_BUZZER_MODE, BUZZER_OFF);
        break;
      case BUZZER_ALARM:
        playNote(NOTE_C, 4, 10);
        vTaskDelay(100);
        playNote(NOTE_E, 4, 10);
        vTaskDelay(100);
        repetitionsAlarm++;

        if (repetitionsAlarm == NB_REPETITIONS_ALARM) {
          setParameter(PARAM_BUZZER_MODE, BUZZER_OFF);
          repetitionsAlarm = 0;
        }
        break;
      case BUZZER_SCALE:
        for (int i = 0; i < nbNotesScale; i++) {
          playNote(scale[i], scaleOctaves[i], 100);
        }
        setParameter(PARAM_BUZZER_MODE, BUZZER_OFF);
        break;
      case BUZZER_BOOT:
        for (int i = 0; i < nbNotesBoot; i++) {
          playNote(bootNotes[i], 5, bootNotesLengths[i]);
        }
        setParameter(PARAM_BUZZER_MODE, BUZZER_OFF);
        break;
      default:
        Serial.println("Unknown buzzer mode");
        setParameter(PARAM_BUZZER_MODE, BUZZER_OFF);
        break;
    }
  }
}

void taskBuzzer() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskBuzzer, "TaskBuzzer",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          1,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

/**
 * Play a note on the buzzer.
 *
 * @param note Note to play
 * @param octave Octave of the note
 * @param duration Duration of the note
 */
void playNote(note_t note, int octave, int duration) {
  buzzer.note(BUZZER_PIN, note, octave, duration, 0);
  vTaskDelay(duration);
}