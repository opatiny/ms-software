#include "./taskBuzzer.h"
#include <globalConfig.h>
#include <utilities/params.h>

Pwm buzzer = Pwm();  // constructor

void TaskBuzzer(void* pvParameters) {
  buzzer.attach(BUZZER_PIN);

  setParameter(PARAM_BUZZER, 1);

  while (true) {
    if (getParameter(PARAM_BUZZER)) {
      buzzer.note(BUZZER_PIN, NOTE_C, 4, 10, 0);
      vTaskDelay(100);
      buzzer.note(BUZZER_PIN, NOTE_D, 4, 10, 0);
      vTaskDelay(100);

    } else {
      buzzer.note(BUZZER_PIN, NOTE_E, 4, 0, 0);
    }
  }
}

void taskBuzzer() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskBuzzer, "TaskBuzzer",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void buzzerModes(Pwm buzzer) {
  switch (getParameter(PARAM_BUZZER)) {
    case BUZZER_OFF:
      break;
    case BUZZER_SINGLE_NOTE:
      Serial.println("Buzzer single note");
      buzzer.note(BUZZER_PIN, NOTE_C, 4, 10, 0);
      break;
    case BUZZER_ALARM:
      for (int i = 0; i < 3; i++) {
        buzzer.note(BUZZER_PIN, NOTE_C, 4, 500, 500);
      }
      break;
    case BUZZER_SCALE:
      for (int i = 0; i < notes; i++) {
        note_t note = cScale[i];
        int octave = cScaleOctaves[i];
        buzzer.note(BUZZER_PIN, note, octave, 10,
                    0);  // auto attaches to ch 0
      }
      break;
    default:
      Serial.println("Unknown buzzer mode");
      break;
  }
}