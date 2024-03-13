/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/wifiUtilities.cpp
 */

#include <WiFi.h>

#include "../globalConfig.h"
#include "./params.h"

char wifiTemp[40];

void printWifiHelp(Print* output) {
  output->println(F("(wa) Wifi Access Point"));
  output->println(F("(wc) Wifi AP password"));
  output->println(F("(wi) Wifi info"));
  output->println(F("(ws) Wifi SSID"));
  output->println(F("(wu) Wifi Username"));
  output->println(F("(wp) Wifi password"));
  output->println(F("(wd) Wifi identify"));
  output->println(F("(wm) Wifi MDNS"));
  output->println(F("(wq) MQTT broker"));
  output->println(F("(wl) MQTT log publish topic"));
  output->println(F("(wb) MQTT publish topic"));
  output->println(F("(wt) MQTT subscribe topic"));
}

void processWifiCommand(char command,
                        char* paramValue,
                        Print* output) {  // char and char* ??
  switch (command) {
    case 'i':
      output->println("Wifi information");
      output->print("RSSI: ");
      output->println(WiFi.RSSI());
      output->print("SSID: ");
      getParameter("wifi.ssid", wifiTemp);
      output->println(wifiTemp);
      output->print("IP address: ");
      output->println(WiFi.localIP());
      output->print("Mac address: ");
      output->println(WiFi.macAddress());
      output->print("Username: ");
      getParameter("wifi.username", wifiTemp);
      output->println(wifiTemp);
      output->print("Identity: ");
      getParameter("wifi.identity", wifiTemp);
      output->println(wifiTemp);
      output->print("Password: ");
      getParameter("wifi.password", wifiTemp);
      output->println(wifiTemp);
      output->print("AP ssid: ");
      getParameter("wifi.ap.ssid", wifiTemp);
      output->println(wifiTemp);
      output->print("AP password: ");
      getParameter("wifi.ap.pass", wifiTemp);
      output->println(wifiTemp);
      output->print("AP IP address: ");
      output->println(WiFi.softAPIP());
      output->print("MDNS: ");
      getParameter("wifi.mdns", wifiTemp);
      output->println(wifiTemp);
      output->print("MQTT broker: ");
      getParameter("mqtt.broker", wifiTemp);
      output->println(wifiTemp);
      output->print("MQTT subscribe topic: ");
      getParameter("mqtt.subscribe", wifiTemp);
      output->println(wifiTemp);
      output->print("MQTT log publish topic: ");
      getParameter("mqtt.logpublish", wifiTemp);
      output->println(wifiTemp);
      output->print("MQTT publish topic: ");
      getParameter("mqtt.publish", wifiTemp);
      output->println(wifiTemp);
      break;
    case 'm':
      setParameter("wifi.mdns", paramValue);
      output->println(paramValue);
      break;
    case 'p':
      setParameter("wifi.password", paramValue);
      output->println(paramValue);
      break;
    case 'q':  // MQTT server
      setParameter("mqtt.broker", paramValue);
      output->println(paramValue);
      break;
    case 't':  // MQTT subscribe topic
      setParameter("mqtt.subscribe", paramValue);
      output->println(paramValue);
      break;
    case 'b':  // MQTT publish topic
      setParameter("mqtt.publish", paramValue);
      output->println(paramValue);
      break;
    case 'l':  // MQTT publish log topic
      setParameter("mqtt.logpublish", paramValue);
      output->println(paramValue);
      break;
    case 's':
      if (paramValue[0] == '\0') {
        getParameter("wifi.ssid", paramValue);
        output->println(paramValue);
      } else {
        setParameter("wifi.ssid", paramValue);
        output->println(paramValue);
      }
      break;
    case 'u':
      setParameter("wifi.username", paramValue);
      output->println(paramValue);
      break;
    case 'd':
      setParameter("wifi.identity", paramValue);
      output->println(paramValue);
      break;
    case 'a':
      if (paramValue[0] == '\0') {
        getParameter("wifi.ap.ssid", paramValue);
        output->println(paramValue);
      } else {
        setParameter("wifi.ap.ssid", paramValue);
        output->println(paramValue);
      }
      break;
    case 'c':
      if (paramValue[0] == '\0') {
        getParameter("wifi.ap.pass", paramValue);
        output->println(paramValue);
      } else {
        setParameter("wifi.ap.pass", paramValue);
        output->println(paramValue);
      }
      break;
    default:
      printWifiHelp(output);
  }
}
