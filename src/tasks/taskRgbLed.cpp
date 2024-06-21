/**
 * Task for the control of the on board RGB LED.
 *   - color: AL
 *   - brightness: AM (0-255)
 *   - mode: AS
 * Debug: U8
 */

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include <globalConfig.h>
#include <pinMapping.h>
#include <utilities/params.h>

#include "taskButton.h"
#include "taskRgbLed.h"

#define DEFAULT_RGB_BRIGHNESS 5
#define BUTTON_PRESSED_COLOR 2
#define OBSTACLE_DETECTED_COLOR 3

RgbLedFlags rgbLedFlags;

const char colorNames[NB_RGB_COLORS][10] = {
    "black", "red", "green", "blue", "yellow", "cyan", "magenta", "white"};

uint32_t getColorFromIndex(int colorIndex);
uint32_t getColorFromName(char const* colorName);
void processRgbFlags();

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

void TaskRgbLed(void* pvParameters) {
  pixels.begin();
  pixels.clear();  // turn all pixels off

  setParameter(PARAM_RGB_LED_MODE, LED_OFF);
  setParameter(PARAM_RGB_LED_BRIGHTNESS, DEFAULT_RGB_BRIGHNESS);

  while (true) {
    processRgbFlags();

    int ledMode = getParameter(PARAM_RGB_LED_MODE);
    int colorIndex = getParameter(PARAM_RGB_LED_COLOR);
    int brightness = getParameter(PARAM_RGB_LED_BRIGHTNESS);
    if (getParameter(PARAM_DEBUG) == DEBUG_RGB_LED) {
      Serial.print("RGB LED mode: ");
      Serial.print(ledMode);
      Serial.print(", Color index: ");
      Serial.print(colorIndex);
      Serial.print(", Brightness: ");
      Serial.println(brightness);
    }
    switch (ledMode) {
      case LED_OFF:
        pixels.setBrightness(0);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();  // todo: go check the doc, there's something about
                        // interrupts being disabled for a short time!!
        vTaskDelay(100);
        break;
      case LED_ON: {
        pixels.setBrightness(brightness);
        pixels.setPixelColor(0, getColorFromIndex(colorIndex));
        pixels.show();  // todo: go check the doc, there's something about
                        // interrupts being disabled for a short time!!
        vTaskDelay(200);
        break;
      }
      case LED_BLINK: {
        pixels.setBrightness(brightness);
        pixels.setPixelColor(0, getColorFromIndex(colorIndex));
        pixels.show();
        vTaskDelay(200);
        pixels.setPixelColor(0, getColorFromName("black"));
        pixels.show();
        vTaskDelay(200);
        break;
      };
      default:
        Serial.println("Error: unknown RGB LED mode");
        break;
    }
  }
}

void taskRgbLed() {
  xTaskCreatePinnedToCore(TaskRgbLed, "TaskRgbLed",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          1,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}

/**
 * @brief Convert the index to the corresponding RGB color.
 */
uint32_t getColorFromIndex(int colorIndex) {
  if (colorIndex >= NB_RGB_COLORS) {
    colorIndex = 0;
  }
  return pixels.Color(colors[colorIndex].r, colors[colorIndex].g,
                      colors[colorIndex].b);
}

/**
 * @brief Find the index of the color in the colorNames array.
 */
int findColorIndex(const char* colorName) {
  for (int i = 0; i < NB_RGB_COLORS; i++) {
    if (strcmp(colorName, colorNames[i]) == 0) {
      return i;
    }
  }
  Serial.print("Error: unknown color name: ");
  Serial.println(colorName);
  return 0;
}

/**
 * @brief Get RGB color from the color name.
 */
uint32_t getColorFromName(char const* colorName) {
  int colorIndex = findColorIndex(colorName);
  return getColorFromIndex(colorIndex);
}

/**
 * @brief Process the RGB LED flags to set mode, color and brightness depending
 * on the tasks flags.
 */
void processRgbFlags() {
  if (rgbLedFlags.buttonPressed) {
    rgbLedFlags.buttonPressed = 0;
    setParameter(PARAM_RGB_LED_MODE, LED_ON);
    setParameter(PARAM_RGB_LED_COLOR, BUTTON_PRESSED_COLOR);
    setParameter(PARAM_RGB_LED_BRIGHTNESS, DEFAULT_RGB_BRIGHNESS);
    return;
  } else {
    setParameter(PARAM_RGB_LED_MODE, LED_OFF);
  }
  if (rgbLedFlags.obstacleDetected) {
    setParameter(PARAM_RGB_LED_MODE, LED_ON);
    setParameter(PARAM_RGB_LED_COLOR, OBSTACLE_DETECTED_COLOR);
    setParameter(PARAM_RGB_LED_BRIGHTNESS, DEFAULT_RGB_BRIGHNESS);
    return;
  } else {
    setParameter(PARAM_RGB_LED_MODE, LED_OFF);
  }
  if (rgbLedFlags.batteryEmpty) {
    setParameter(PARAM_RGB_LED_MODE, LED_BLINK);
    setParameter(PARAM_RGB_LED_COLOR, 1);
    setParameter(PARAM_RGB_LED_BRIGHTNESS, DEFAULT_RGB_BRIGHNESS);
    return;
  }
}