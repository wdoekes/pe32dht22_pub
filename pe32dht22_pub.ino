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
 * This fits reasonably well with this topic layout:
 * > All MQTT clients using the Sparkplug specification will use the
 * > following Topic Namespace structure:
 * >   namespace/group_id/message_type/edge_node_id/[device_id]
 * In our case, that would be:
 * - <namespace/location>/temperature//device_id (we skip the edge_node_id)
 *
 * For now, we're not bothered that the messages may not arrive in a
 * single batch. If we want to align the values, we'll round to the
 * nearest time when storing.
 *
 * Configuration goes in arduino_secrets.h
 *
 * TODO:
 * - right now, the dead mans switch (GLOBAL_RESET_TIME) does the
 *   reconnect after mqtt has been restarted (proper tcp-fin teardown);
 *   should be (likely fixed by this commit?)
 * - right now, it pushes the temperature message in a first tcp packet and
 *   then the rest in a second one (maybe we can delay the first one?)
 */
#define DHT_VERSION 22  // for DHT-22
#define DHT_PIN 17      // DHT sensor data pin GPIO-17
#define GLOBAL_RESET_TIME 300 // reboot device if no publish for X seconds
#define PIN_MQ135 32 // GPIO32, ADC1_CH4 (cannot use ADC2 because of WiFi)

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

#include <MQ135.h>

#include "arduino_secrets.h"
#include "build_version.h"

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

// https://circuitdigest.com/microcontroller-projects/interfacing-mq135-gas-sensor-with-arduino-to-measure-co2-levels-in-ppm
// https://github.com/tricoos/m135-lua#calibration-and-burn-in
/*
What are safe levels of CO2 in rooms?
- 250-400ppm	Normal background concentration in outdoor ambient air
- 400-1,000ppm	Concentrations typical of occupied indoor spaces with good air exchange
- 1,000-2,000ppm	Complaints of drowsiness and poor air.
*/
const float RZERO = 4.82;
const float RLOAD = 10.0; // on-board resistor = 10kOhm ??
MQ135 mq135(PIN_MQ135, RZERO, RLOAD);

TaskHandle_t tempTaskHandle = NULL;
Ticker tempTicker;
const int dhtPin = DHT_PIN;

unsigned long lastMqtt;
float lastTemperature;
float lastHumidity;

float lastRzero = RZERO;
float lastRawPpm = 420;
float lastCorrPpm = 420;

void initGuid() {
  strncpy(guid, "EUI48:", 6);
  strncpy(guid + 6, WiFi.macAddress().c_str(), sizeof(guid) - (6 + 1));
}

void initSerial() {
  Serial.begin(115200);
  delay(200);  // wait a bit to avoid start/restart spam
  Serial << "\x1b[0;32mDHT11/22 ESP32 example with tasks [" << guid << "]\x1b[0m\r\n";
  Serial << "  GIT_VERSION = " GIT_VERSION << "\r\n";
  Serial << "  BUILD_HOST = " BUILD_HOST << "\r\n";
  Serial << "  BUILD_TIME = " BUILD_TIME << "\r\n";
}

void initWifi() {
  WiFiMulti.addAP(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
}

void initMqtt() {
  MqttClient.id(guid);
}

void sendMqtt(String subtopic, String value) {
  if (MqttClient.connected()) {
    String topic(SECRET_MQTT_TOPIC);
    topic += "/";
    topic += subtopic;
    topic += "/";
    topic += guid;
    MqttClient.publish(topic.c_str(), value);
    lastMqtt = millis();
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

  // Set globals
  lastTemperature = newValues.temperature;
  lastHumidity = newValues.humidity;

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
  sendMqtt("buildversion", GIT_VERSION);
  sendMqtt("buildtime", BUILD_TIME);

  sendMqtt("mq135rzero", String(lastRzero));
  sendMqtt("mq135rawppm", String(lastRawPpm));
  sendMqtt("mq135corrppm", String(lastCorrPpm));
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

void loopGasSensor(float temperature, float humidity) {
  static unsigned long nextGasSensorRun = 0;
  unsigned long now = millis();
  if (nextGasSensorRun > now) {
    return;
  }
  nextGasSensorRun = now + 10*1000; // next in 10s

  float rzero = mq135.getRZero();
  float correctedRZero = mq135.getCorrectedRZero(temperature, humidity);
  float resistance = mq135.getResistance();
  float ppm = mq135.getPPM();
  float correctedPPM = mq135.getCorrectedPPM(temperature, humidity);

  Serial.print("MQ135 RZero: ");
  Serial.print(rzero);
  Serial.print("\t Corrected RZero: ");
  Serial.print(correctedRZero);
  Serial.print("\t Resistance: ");
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);
  Serial.print("\t Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");

  lastRzero = ((7 * lastRzero) + rzero) / 8;
  lastRawPpm = ((7 * lastRawPpm) + ppm) / 8;
  lastCorrPpm = ((7 * lastCorrPpm) + correctedPPM) / 8;
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

  lastTemperature = 20.0;
  lastHumidity = 25.0;
}

void loop() {
  static bool firstRun = true;
  String bootlog;

  if (firstRun) {
    firstRun = false;
    lastMqtt = millis();
    bootlog += "initial_run ";
  }
  if (WiFi.status() != WL_CONNECTED) {
    bootlog += "wifi_was_down ";
    Serial << "Waiting for WiFi " << SECRET_WIFI_SSID << "...";
    WiFiMulti.run(15000);
    if (WiFi.status() == WL_CONNECTED) {
      Serial << "WiFi connected, local-IP " << WiFi.localIP().toString() << "\r\n";
      bootlog += "wifi_connected ";
    } else {
      Serial << "(not connected yet)\r\n";
    }
  }
  if (WiFi.status() == WL_CONNECTED && !MqttClient.connected()) {
    bootlog += "mqtt_was_down ";
    Serial << "Connecting to MQTT at " SECRET_MQTT_BROKER "\r\n";
    MqttClient.connect(SECRET_MQTT_BROKER, SECRET_MQTT_PORT);
    if (MqttClient.connected()) {
      bootlog += "mqtt_connected ";
      Serial << "Connected to MQTT\r\n";
    } else {
      Serial << "(not connected yet)\r\n";
    }
  }
  if(!bootlog.isEmpty()) {
    sendMqtt("bootlog", bootlog);
  }

  // Manual keepalives, but not reconnects
  MqttClient.loop();

  // Dead mans switch
  if ((millis() - lastMqtt) > GLOBAL_RESET_TIME*1000) {
    sendMqtt("bootlog", AS_CSTR(GLOBAL_RESET_TIME) "s_no_talking");
    ESP.restart();
  }

  loopGasSensor(lastTemperature, lastHumidity);

  // TODO: esp32 pthread_cond_wait here? And then do stuff with the values from the bg job..
  yield();
}
