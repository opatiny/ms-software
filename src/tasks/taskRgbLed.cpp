/**
 * Task for the control of the on board RGB LED.
 *   - color of the LED is set by the parameter PARAM_RGB_LED_COLOR.
 *   - brightness of the LED is set by the parameter PARAM_RGB_LED_BRIGHTNESS
 * (0-255).
 */

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>
#include "../pinMapping.h"
#include "./taskRgbLed.h"

uint32_t getColorFromIndex(int colorIndex);

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

void TaskRgbLed(void* pvParameters) {
  pixels.begin();
  pixels.clear();  // turn all pixels off
  while (true) {
    int colorIndex = getParameter(PARAM_RGB_LED_COLOR);
    int brightness = getParameter(PARAM_RGB_LED_BRIGHTNESS);
    if (getParameter(PARAM_DEBUG) == DEBUG_RGB_LED) {
      Serial.print("Color index: ");
      Serial.print(colorIndex);
      Serial.print(", brightness: ");
      Serial.println(brightness);
    }

    pixels.setBrightness(brightness);
    pixels.setPixelColor(
        0, pixels.Color(colors[colorIndex].r, colors[colorIndex].g,
                        colors[colorIndex].b));
    pixels.show();  // todo: go check the doc, there's something about
                    // interrupts being disabled for a short time!!
    vTaskDelay(1000);
  }
}

void taskRgbLed() {
  xTaskCreatePinnedToCore(TaskRgbLed, "TaskRgbLed",
                          2048,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          1,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}

uint32_t getColorFromIndex(int colorIndex) {
  return pixels.Color(colors[colorIndex].r, colors[colorIndex].g,
                      colors[colorIndex].b);
}