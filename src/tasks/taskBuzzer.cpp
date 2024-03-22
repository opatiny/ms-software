#include <pwmWrite.h>

#define BUZZER_PIN D0
const int notes = 8;
const note_t cScale[notes] = {NOTE_C, NOTE_D, NOTE_E, NOTE_F,
                              NOTE_G, NOTE_A, NOTE_B, NOTE_C};
const int cScaleOctaves[notes] = {4, 4, 4, 4, 4, 4, 4, 5};
const int duration = 500;  // note duration ms
const int interval = 0;    // pause between notes ms

Pwm pwm = Pwm();  // constructor

void TaskBuzzer(void* pvParameters) {
  while (true) {
    for (int i = 0; i < notes; i++) {
      note_t note = cScale[i];
      int octave = cScaleOctaves[i];
      pwm.note(BUZZER_PIN, note, octave, duration,
               interval);  // auto attaches to ch 0
      vTaskDelay(100);
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