#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#define SSID "Jack"
#define PASSWORD "10705470"
#define HOST "192.168.0.15"
#define PORT 3333

#define PIR_PIN 23
#define TEMP_SENSOR_PIN 33

#define LAMP1_PIN 4
#define LAMP2_PIN 5
#define LAMP3_PIN 18
#define LAMP4_PIN 19

#define FAN1_PIN 21
#define FAN2_PIN 22

#define GENERAL1_PIN 25
#define GENERAL2_PIN 26

WiFiMulti wifiMulti;
HTTPClient http;

bool pirState = false;
int temperatureRaw = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  pinMode(LAMP1_PIN, OUTPUT);
  pinMode(LAMP2_PIN, OUTPUT);
  pinMode(LAMP3_PIN, OUTPUT);
  pinMode(LAMP4_PIN, OUTPUT);

  pinMode(FAN1_PIN, OUTPUT);
  pinMode(FAN2_PIN, OUTPUT);

  pinMode(GENERAL1_PIN, OUTPUT);
  pinMode(GENERAL2_PIN, OUTPUT);

  wifiMulti.addAP(SSID, PASSWORD);
  http.setReuse(true);
}

bool digitalReadBool(int pin) {
  return digitalRead(pin) == HIGH;
}

void readSensors() {
  pirState = digitalReadBool(PIR_PIN);
  // temperatureRaw = analogRead(TEMP_SENSOR_PIN);
  if (temperatureRaw > 4) {
    temperatureRaw = 0;
  } else {
    temperatureRaw += 1;
  }
}

void sendSensorData() {
  JsonDocument doc;
  doc["pir"] = pirState;
  doc["temperature"] = temperatureRaw;

  Serial.print("[HTTP] POST /sensors body: ");
  serializeJsonPretty(doc, Serial);

  http.begin(HOST, PORT, "/sensors");
  http.addHeader("Content-Type", "application/json");

  String requestBody;
  serializeJson(doc, requestBody);

  int httpCode = http.POST(requestBody);
  if (httpCode > 0) {
    Serial.printf("[HTTP] POST /sensors code: %d\n", httpCode);
  } else {
    Serial.printf("[HTTP] POST /sensors error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

void syncActuatorStates() {
  JsonDocument doc;

  doc["lamp1"] = digitalReadBool(LAMP1_PIN);
  doc["lamp2"] = digitalReadBool(LAMP2_PIN);
  doc["lamp3"] = digitalReadBool(LAMP3_PIN);
  doc["lamp4"] = digitalReadBool(LAMP4_PIN);

  doc["fan1"] = digitalReadBool(FAN1_PIN);
  doc["fan2"] = digitalReadBool(FAN2_PIN);

  doc["general1"] = digitalReadBool(GENERAL1_PIN);
  doc["general2"] = digitalReadBool(GENERAL2_PIN);

  Serial.print("[HTTP] POST /actuators/state body: ");
  serializeJsonPretty(doc, Serial);

  http.begin(HOST, PORT, "/actuators/state");
  http.addHeader("Content-Type", "application/json");

  String requestBody;
  serializeJson(doc, requestBody);

  int httpCode = http.POST(requestBody);
  if (httpCode > 0) {
    Serial.printf("[HTTP] POST /actuators/state code: %d\n", httpCode);

    String response = http.getString();
    JsonDocument resDoc;
    DeserializationError error = deserializeJson(resDoc, response);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
    } else {
      Serial.print("[HTTP] POST /actuators/state response: ");
      serializeJsonPretty(resDoc, Serial);

      digitalWrite(LAMP1_PIN, resDoc["lamp1"] ? HIGH : LOW);
      digitalWrite(LAMP2_PIN, resDoc["lamp2"] ? HIGH : LOW);
      digitalWrite(LAMP3_PIN, resDoc["lamp3"] ? HIGH : LOW);
      digitalWrite(LAMP4_PIN, resDoc["lamp4"] ? HIGH : LOW);

      digitalWrite(FAN1_PIN, resDoc["fan1"] ? HIGH : LOW);
      digitalWrite(FAN2_PIN, resDoc["fan2"] ? HIGH : LOW);

      digitalWrite(GENERAL1_PIN, resDoc["general1"] ? HIGH : LOW);
      digitalWrite(GENERAL2_PIN, resDoc["general2"] ? HIGH : LOW);
    }
  } else {           
    Serial.printf("[HTTP] POST /actuators/state error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

void loop() {
  readSensors();

  if (wifiMulti.run() == WL_CONNECTED) {
    sendSensorData();
    syncActuatorStates();
  } else {
    Serial.println("WiFi not connected");
  }

  delay(2500);
}
