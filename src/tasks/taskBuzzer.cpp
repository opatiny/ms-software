#include "./taskBuzzer.h"
#include <globalConfig.h>
#include <utilities/params.h>

Pwm buzzer = Pwm();  // constructor

void TaskBuzzer(void* pvParameters) {
  buzzer.attach(BUZZER_PIN);

  setParameter(PARAM_BUZZER, BUZZER_SINGLE_NOTE);

  int repetitionsAlarm = 0;
  while (true) {
    switch (getParameter(PARAM_BUZZER)) {
      case BUZZER_OFF:
        // duration of zero turn output off
        buzzer.note(BUZZER_PIN, NOTE_E, 3, 0, 0);
        vTaskDelay(10);
        break;
      case BUZZER_SINGLE_NOTE:
        buzzer.note(BUZZER_PIN, NOTE_C, 4, 10, 0);
        vTaskDelay(100);
        setParameter(PARAM_BUZZER, BUZZER_OFF);
        break;
      case BUZZER_ALARM:
        buzzer.note(BUZZER_PIN, NOTE_C, 4, 10, 0);
        vTaskDelay(100);
        buzzer.note(BUZZER_PIN, NOTE_D, 4, 10, 0);
        vTaskDelay(100);
        repetitionsAlarm++;

        if (repetitionsAlarm == NB_REPETITIONS_ALARM) {
          setParameter(PARAM_BUZZER, BUZZER_OFF);
          repetitionsAlarm = 0;
        }
        break;
      case BUZZER_SCALE:
        for (int i = 0; i < nbNotesScale; i++) {
          buzzer.note(BUZZER_PIN, scale[i], scaleOctaves[i], 10, 0);
          vTaskDelay(100);
        }
        setParameter(PARAM_BUZZER, BUZZER_OFF);
        break;
      case BUZZER_BOOT:
        for (int i = 0; i < nbNotesBoot; i++) {
          buzzer.note(BUZZER_PIN, bootNotes[i], bootOctaves[i], 10, 0);
          vTaskDelay(100);
        }
        setParameter(PARAM_BUZZER, BUZZER_OFF);
        break;
      default:
        Serial.println("Unknown buzzer mode");
        setParameter(PARAM_BUZZER, BUZZER_OFF);
        vTaskDelay(10);
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
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}
