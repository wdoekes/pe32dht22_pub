/**
 * pe32dht22_pub // Get DHT22 readings, export to MQTT
 *
 * Components:
 * - ESP32 (possibly works with ESP8266 too?)
 * - DHT22, with pin 1 on 3V3, pin 2 on GPIO17, pin 4 on GND
 *
 * Building/dependencies:
 * - Arduino IDE
 * - package_esp32_index.json Boards Managers URL
 * - DHTesp, TinyConsole, TinyMqtt (but see hsaturn/TinyMqtt#53)
 *
 * Current MQTT key/values (may change in the future):
 * - <TOPIC>/temperature/EUI48:11:22:33:44:55:66 value
 * - <TOPIC>/humidity/EUI48:11:22:33:44:55:66 value
 * - ...
 *
 * Configuration goes in arduino_secrets.h
 */
#define DHT_VERSION 22  // for DHT-22
#define DHT_PIN 17      // DHT sensor data pin GPIO-17

// ESP32 libraries: http://boardsmanager/All#esp32
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiMulti.h>

// DHT11/22 support
#include <DHTesp.h>  // ^click: http://librarymanager/All#DHTesp
// Serial<< operator
#include <TinyStreaming.h>  // ^click: http://librarymanager/All#TinyConsole
// MQTT
#include <TinyMqtt.h>  // ^click: http://librarymanager/All#TinyMqtt

#include "arduino_secrets.h"

#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY !)
#error Select ESP32 board; e.g. "DOIT ESP32 DEVKIT V1" for ESP32-WROOM-32
#endif

#if DHT_VERSION == 11
#define DHTX DHT11
#elif DHT_VERSION == 22
#define DHTX DHT22
#else
#error Please select DHT_VERSION 11 or 22
#endif

#define EXPAND_VALUE_THEN_AS_CSTR(EXP) #EXP
#define AS_CSTR(EXP) EXPAND_VALUE_THEN_AS_CSTR(EXP)

/* We use the guid to store something unique to identify the device by.
 * For now, we'll populate it with the ESP Wifi MAC address,
 * if available. */
static char guid[24] = "<no_wifi_found>";  // "EUI48:11:22:33:44:55:66"

WiFiMulti WiFiMulti;
MqttClient MqttClient;

DHTesp dht;

TaskHandle_t tempTaskHandle = NULL;
Ticker tempTicker;
const int dhtPin = DHT_PIN;

void initGuid() {
  strncpy(guid, "EUI48:", 6);
  strncpy(guid + 6, WiFi.macAddress().c_str(), sizeof(guid) - (6 + 1));
}

void initSerial() {
  Serial.begin(115200);
  delay(200);  // wait a bit to avoid start/restart spam
  Serial << "DHT11/22 ESP32 example with tasks [" << guid << "]\r\n";
}

void initWifi() {
  WiFiMulti.addAP(SECRET_WIFI_SSID, SECRET_WIFI_PASS);

  Serial << "Waiting for WiFi " << SECRET_WIFI_SSID << "...";
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial << ".";
    delay(500);
  }
  Serial << "\r\n";

  Serial << "WiFi connected, local-IP " << WiFi.localIP().toString() << "\r\n";
}

void initMqtt() {
  Serial << "Connecting to MQTT at " SECRET_MQTT_BROKER "\r\n";
  MqttClient.id(guid);
  MqttClient.connect(SECRET_MQTT_BROKER, SECRET_MQTT_PORT);
  if (!MqttClient.connected()) {
    Serial << "NOT CONNECTED!\r\n";
  }
}

void sendMqtt(String subtopic, String value) {
  if (MqttClient.connected()) {
    String topic(SECRET_MQTT_TOPIC);
    topic += "/";
    topic += subtopic;
    topic += "/";
    topic += guid;
    MqttClient.publish(topic.c_str(), value);
  }
}

/**
 * initTemp
 * Setup DHT library
 * Setup task and timer for repeated measurement
 * @return bool
 *    true if task and timer are started
 *    false if task or timer couldn't be started
 */
bool initTemperatureTask() {
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHTX);
  Serial << AS_CSTR(DHTX) " initialized on pin " AS_CSTR(DHT_PIN) "\r\n";

  // Start task to get temperature
  xTaskCreatePinnedToCore(
    tempTask,        /* Function to implement the task */
    "tempTask ",     /* Name of the task */
    4000,            /* Stack size in words */
    NULL,            /* Task input parameter */
    5,               /* Priority of the task */
    &tempTaskHandle, /* Task handle. */
    1);              /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial << "Failed to start task for temperature update\r\n";
    return false;
  }

  // Start update of environment data every N seconds
  tempTicker.attach(60, triggerGetTemp);
  return true;
}

/**
 * triggerGetTemp
 * Sets flag dhtUpdated to true for handling in loop()
 * called by Ticker getTempTimer
 */
void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
    xTaskResumeFromISR(tempTaskHandle);
  }
}

/**
 * Task to reads temperature from DHTx sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void tempTask(void* pvParameters) {
  //Serial << "[in-task] loop started\r\n";
  while (1) {
    getTemperature();
    // Go to sleep again
    //Serial << "[in-task] going to sleep\r\n";
    vTaskSuspend(NULL);  // can be resumed by: vTaskResume(tempTaskHandle);
    //Serial << "[in-task] awoken\r\n";
  }
}

/**
 * getTemperature
 * Reads temperature from DHTx sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
 */
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial << "DHT11/22 error status: " << dht.getStatusString() << "\r\n";
    return false;
  }

  ComfortState cf;
  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  // FIXME: Surely we cannot do serial print unsynchronised from a subtasks.. can we?
  Serial <<
    " T:" << newValues.temperature <<
    " H:" << newValues.humidity <<
    " I:" << heatIndex <<
    " D:" << dewPoint <<
    " R:" << cr << " (" << getComfortStatusString(cf) << ")\r\n";

  // FIXME: should send this from main thread/proc?
#if 1
  sendMqtt("temperature", String(newValues.temperature));
  sendMqtt("humidity", String(newValues.humidity));
  sendMqtt("heatindex", String(heatIndex));
  sendMqtt("dewpoint", String(dewPoint));
  sendMqtt("comfortidx", String(cr));
  sendMqtt("comfort", getComfortStatusString(cf));
#endif

  return true;
}

const char* getComfortStatusString(ComfortState cf) {
  switch (cf) {
    case Comfort_OK: return "Comfort_OK";
    case Comfort_TooHot: return "Comfort_TooHot";
    case Comfort_TooCold: return "Comfort_TooCold";
    case Comfort_TooDry: return "Comfort_TooDry";
    case Comfort_TooHumid: return "Comfort_TooHumid";
    case Comfort_HotAndHumid: return "Comfort_HotAndHumid";
    case Comfort_HotAndDry: return "Comfort_HotAndDry";
    case Comfort_ColdAndHumid: return "Comfort_ColdAndHumid";
    case Comfort_ColdAndDry: return "Comfort_ColdAndDry";
    default: return "Comfort_UNKNOWN";
  }
}

void setup() {
  // Setup
  initGuid();
  initSerial();
  initWifi();
  initMqtt();

  // Init task and schedule for forever
  initTemperatureTask();
  Serial << "\r\n";
}

void loop() {
  MqttClient.loop();

#if 0
  static long int next_send = millis();
  if (millis() >= next_send) {
    sendMqtt("test", String(123));
    next_send += 5 * 1000;
  }
#endif

  // TODO: esp32 pthread_cond_wait here? And then do stuff with the values from the bg job..
  yield();
}