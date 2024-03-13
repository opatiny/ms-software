/**
 * Thread to enable the connection to the wifi network.
 *
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/taskWifi.cpp
 */
#include <WiFi.h>
#include "utilities/params.h"

#include "esp_wpa2.h"

char ssid[30];
char password[30];
// the wifi also requires username and password
char username[30];
char identity[30];

void retrieveWifiParameters() {
  getParameter("wifi.ssid", ssid);
  getParameter("wifi.password", password);
  getParameter("wifi.username", username);
  getParameter("wifi.identity", identity);
}

void TaskWifi(void* pvParameters) {
  vTaskDelay(2500);

  retrieveWifiParameters();

  if (strlen(ssid) < 2 || strlen(password) < 2) {
    Serial.println("No wifi ssid or password defined");
    while (strlen(ssid) < 2 || strlen(password) < 2) {
      vTaskDelay(1000);
      retrieveWifiParameters();
    }
  }

  Serial.print("Trying to connect to ");
  Serial.println(ssid);
  Serial.print("Using password ");
  Serial.println(password);

  WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // The following line is required because of a bug in the LOLIN esp32-c3 board
  // https://github.com/espressif/arduino-esp32/issues/6767
  // It could be removed in the other cases and yield to better connection range
  // WiFi.setTxPower(WIFI_POWER_8_5dBm);
  // WiFi.disconnect(true);
  //
  // if identity not defined use username as identity
  if (strlen(identity) == 0) {
    strcpy(identity, username);
  }
  // parameter wifi type
  bool wifi_Enterprise_type = false;
  // if identity and username not defined use domestic WPA2 to connect to wifi
  if (strlen(identity) == 0 && strlen(username) == 0) {
    Serial.println("Using domestic WPA2");
    wifi_Enterprise_type = false;
  }
  // if identity and username defined use WPA2 Enterprise to connect to wifi
  else {
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)identity, strlen(identity));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t*)username, strlen(username));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t*)password, strlen(password));
    esp_wifi_sta_wpa2_ent_enable();
    wifi_Enterprise_type = true;
  }

  while (true) {
    vTaskDelay(1000);
    byte counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter++ < 10) {
      setParameter(PARAM_WIFI_RSSI, -1);
      Serial.println("WIFI not connected, trying to connect");

      if (wifi_Enterprise_type == true) {
        WiFi.begin(ssid);
      } else {
        WiFi.begin(ssid, password);
      }

      // WiFi.reconnect();
      vTaskDelay(2000);
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WIFI connected");
      }
    }
    if (WiFi.status() != WL_CONNECTED) {
      setParameterBit(PARAM_STATUS, PARAM_STATUS_FLAG_NO_WIFI);
    } else {
      clearParameterBit(PARAM_STATUS, PARAM_STATUS_FLAG_NO_WIFI);
      setParameter(PARAM_WIFI_RSSI, WiFi.RSSI());
    }
  }
}

void taskWifi() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskWifi, "TaskWifi",
                          12000,  // This stack size can be checked & adjusted
                                  // by reading the Stack Highwater
                          NULL,
                          0,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}
