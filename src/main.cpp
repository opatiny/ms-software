/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

// functions prototypes
void taskBlink();


void setup(){
  taskBlink();
}

void loop(){
  vTaskDelay(1000);
}