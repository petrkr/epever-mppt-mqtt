#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <ArduinoOTA.h>
#include <ElegantOTA.h>
#include <WebServer.h>

#include "ModbusMaster.h"
#include "PubSubClient.h"

const char* ver = "0.0.2";

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

uint8_t j, result;

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


void mqtt_print_debug(char* message) {
  if (!mqtt.connected()) {
    return;
  }

  mqtt.publish(String(MQTT_ROOT_TOPIC + String("/debug")).c_str(), message);
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


void readModbus(uint16_t reg, uint8_t len, bool sendMqtt) {
  uint16_t buff[len];
  result = node.readInputRegisters(reg, len);
  // do something with data if read is successful
  if (result != node.ku8MBSuccess)
  {
    Serial.print("Modbus read failed: register: ");
    Serial.println(reg, HEX);

    if (sendMqtt && mqtt.connected()) {
      mqtt.publish(String(MQTT_ROOT_TOPIC + String("/fail/0x") + String(reg, HEX)).c_str(), "Failed");
    }

    return;
  }

  if (DEBUG)
  {
    Serial.print("Register 0x");
    Serial.print(reg, HEX);
    Serial.print(": ");
  }

  for (j = 0; j < len; j++)
  {
    buff[j] = node.getResponseBuffer(j);

    if (DEBUG) {
      Serial.print(buff[j], HEX);
    }
  }

  if (DEBUG) {
    Serial.println();
  }

  Serial.print("Size buffer: ");
  Serial.println(sizeof(buff));

  if (sendMqtt && mqtt.connected()) {
    mqtt.publish(String(MQTT_ROOT_TOPIC + String("/0x") + String(reg, HEX)).c_str(), (uint8_t*)buff, sizeof(buff));
  }
}

long previousMQTTKeepAliveMillis = 0;
long mqtt_keepalive_counter = 0;
void handleMQTTKeepAlive() {
  if (millis() - previousMQTTKeepAliveMillis >= MQTT_KEEPALIVE) {
    previousMQTTKeepAliveMillis = millis();

    mqtt_keepalive_counter++;

    if (!mqtt.connected()) {
      MQTT_connect();
    }

    if (!mqtt.connected()) {
      return;
    }

    readModbus(0x3100, 18, true);
    delay(5);

    readModbus(0x3304, 16, true);
    delay(5);

    readModbus(0x311A, 1, true);
    delay(5);

    mqtt.publish(String(MQTT_ROOT_TOPIC + String("/keepalive")).c_str(), String(mqtt_keepalive_counter).c_str());
    Serial.println("MQTT Keep Alive");
  }
}

void preTransmission()
{
  digitalWrite(RTS, 1);
  delay(1);
}

void postTransmission()
{
  digitalWrite(RTS, 0);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Epever Solar Booting..."));
  Serial.print(F("Version: "));
  Serial.println(ver);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (retry > 30) {
      ESP.restart();
    }
    retry++;
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

  Serial.print(F("Configure modbus... "));

  Serial2.begin(115200, SERIAL_8N1, RxD1, TxD1);


  node.begin(1, Serial2);

  if (RTS > 0) {
    pinMode(RTS, OUTPUT);
    digitalWrite(RTS, 0);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
  }

  Serial.println(F("OK"));

  Serial.println(F("Ready"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

  mqtt_print_debug("Boot done, running main loop");

}

void loop() {
  handleKeepAlive();
  ArduinoOTA.handle();
  server.handleClient();
  handlePCSerial();
  mqtt.loop();
  handleMQTTKeepAlive();
}
