#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <ArduinoOTA.h>
#include <ElegantOTA.h>
#include <WebServer.h>

#include "ModbusMaster.h"
#include "PubSubClient.h"

const char* ver = "1.0.0";

const char* host = "SolarOTA";

const char* ssid     = "...";
const char* password = "...";
#define MQTT_ROOT_TOPIC  "..."


bool DEBUG = true;

#define RxD1 23 // MOSI
#define TxD1 22 // SCL
#define RTS  -1

#define MQTT_SERVER      "..."
#define MQTT_PORT        8883
#define MQTT_CLIENTID    "..."
#define MQTT_USERNAME    "..."
#define MQTT_KEY         "..."



#define MQTT_KEEPALIVE 60000
#define KEEPALIVE 5000

WiFiClientSecure mqttclient;
PubSubClient mqtt(mqttclient);

ModbusMaster node;
WebServer server(80);


void MQTT_connect() {
  // Loop until we're reconnected
  int retry = 0;
  while (!mqtt.connected()) {
    retry++;
    Serial.print(F("Attempting MQTT connection..."));

    // Create a random client ID
    String clientId = MQTT_CLIENTID;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str(), MQTT_USERNAME, MQTT_KEY)) {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      mqtt.publish(MQTT_ROOT_TOPIC, MQTT_CLIENTID);
      mqtt.subscribe(String(MQTT_ROOT_TOPIC + String(F("/read"))).c_str());
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqtt.state());
      if (retry > 3) {
        Serial.println("Failed connect to MQTT, continue");
        return;
      }

      Serial.println(F(" try again in 5 seconds"));
      delay(5000);
    }
  }
}

void WEB_Setup() {
  server.on("/", []() {
    server.send(200, "text/plain", "Hi! I am ESP32 Solar OTA.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
}

void OTA_Setup() {
  ArduinoOTA.setHostname(String(F("LAN OTA")).c_str());

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println(F("\nEnd"));
    Serial.println(F("Disconnect from wifi"));
    WiFi.disconnect();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR) Serial.println(F("End Failed"));
  });

  ArduinoOTA.begin();
}


void handlePCSerial() {
  if (Serial.available()) {
    char r = Serial.read();

    if (r == 'B') {
      ESP.restart();
      return;
    }
  }
}


long previousKeepAliveMillis = 0;
void handleKeepAlive() {
  if (millis() - previousKeepAliveMillis >= KEEPALIVE) {
    previousKeepAliveMillis = millis();

    Serial.println("Keep Alive");
  }
}


void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (length > 0 && payload[0] == 'R') {
    return;
  }

  if (length > 0 && payload[0] == 'D') {
    DEBUG = !DEBUG;
    Serial.print("Debug mode is ");
    if (DEBUG) {
      Serial.println("ON");
    }
    else {
      Serial.println("OFF");
    }

    return;
  }

  if (length > 0 && payload[0] == 'B') {
    Serial.println(F("Rebooting"));
    mqtt.publish(String(MQTT_ROOT_TOPIC).c_str(), "REBOOT");
    ESP.restart();
    return;
  }
}

long previousMQTTKeepAliveMillis = 0;
long mqtt_keepalive_counter = 0;
void handleMQTTKeepAlive() {
  if (millis() - previousMQTTKeepAliveMillis >= MQTT_KEEPALIVE) {
    previousMQTTKeepAliveMillis = millis();

    mqtt_keepalive_counter++;

    if (!mqtt.connected()) {
      return;
    }

    mqtt.publish(String(MQTT_ROOT_TOPIC + String("/keepalive")).c_str(), String(mqtt_keepalive_counter).c_str());
    Serial.println("MQTT Keep Alive");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Epever Solar Booting..."));
  Serial.print(F("Version: "));
  Serial.println(ver);

    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

  Serial.println(F("Connected"));

  Serial.print(F("Configure OTA... "));
  OTA_Setup();
  Serial.println(F("OK"));

  Serial.print(F("Configure WEB... "));
  WEB_Setup();
  Serial.println(F("OK"));

  Serial.print(F("Configure MQTT... "));
  mqttclient.setInsecure();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_callback);
  MQTT_connect();
  Serial.println(F("OK"));

  Serial.println(F("Ready"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
}

void loop() {
  handleKeepAlive();
  ArduinoOTA.handle();
  server.handleClient();
  handlePCSerial();
  mqtt.loop();
  handleMQTTKeepAlive();
}
