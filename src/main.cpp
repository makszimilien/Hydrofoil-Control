#include "SPIFFS.h"
#include "filehandling.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// Watchdog timeout in milliseconds
const int WATCHDOG_TIMEOUT = 8000;

// WiFi timer variables
unsigned long previousMillis = 0;
const long interval = 10000;
unsigned long currentMillis = 0;

// Search for parameter in HTTP POST request
const char *PARAM_INPUT_1 = "ssid";
const char *PARAM_INPUT_2 = "pass";
const char *PARAM_INPUT_3 = "slave";
const char *PARAM_INPUT_4 = "mac";

// Variables to save values from HTML form
String ssid;
String pass;
String slave;
String first;
String ip = "192.168.1.200";
String gateway = "192.168.1.1";
String sliderValue;
String mac;
String macAddresses[] = {"", "", "", "", ""};

// File paths to save input values permanently
const char *jsonWifiPath = "/wifi.json";
const char *jsonConfigPath = "/config.json";

// Setting hostname
const char *hostname = "hydrofoil-control";

// Variables for Local IP address, gateway and mask
IPAddress localIP;
IPAddress localGateway;
IPAddress subnet(255, 255, 0, 0);

// "Watchdog" variable for the filesystem
boolean restart = false;

// Define pins for various components
const int ledPin = GPIO_NUM_32;

// Create web server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void sendData() {
  ws.printfAll("{\"mac\":\"%s\",\"slave\":\"%s\",\"slider\":\"%s\"}",
               WiFi.macAddress().c_str(), slave.c_str(), sliderValue.c_str());
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->ping();
    sendData();
  } else if (type == WS_EVT_DISCONNECT) {
  } else if (type == WS_EVT_ERROR) {
  } else if (type == WS_EVT_PONG) {
  }
}

void setupWifiFirst() {

  Serial.println("Setting AP (Access Point)");
  WiFi.mode(WIFI_MODE_NULL);
  WiFi.setHostname("hydrofoil-control");
  // NULL sets an open Access Point
  WiFi.softAP("Hydrofoil-Control-WiFi-Manager", NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/wifimanager.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    slave = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "SLAVE", slave.c_str());
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter *p = request->getParam(i);
      if (p->isPost()) {
        // HTTP POST ssid value
        if (p->name() == PARAM_INPUT_1) {
          ssid = p->value().c_str();
          Serial.print("SSID set to: ");
          Serial.println(ssid);
          // Write file to save value
          writeFileJson(SPIFFS, jsonWifiPath, "SSID", ssid.c_str());
        }
        // HTTP POST pass value
        if (p->name() == PARAM_INPUT_2) {
          pass = p->value().c_str();
          Serial.print("Password set to: ");
          Serial.println(pass);
          // Write file to save value
          writeFileJson(SPIFFS, jsonWifiPath, "PASS", pass.c_str());
        }

        // HTTP POST slave value
        if (p->name() == PARAM_INPUT_3) {
          slave = "True";
          Serial.print("Slave device: ");
          Serial.println(slave);
          // Write file to save value
          writeFileJson(SPIFFS, jsonWifiPath, "SLAVE", slave.c_str());
        }
      }
    }
    restart = true;
    request->send(200, "text/plain",
                  "Done. ESP will restart, and create Master hotspot, or "
                  "connect as a Slave.");
    first = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "FIRST", first.c_str());
  });
  server.begin();
};

void setupWifiMaster() {
  Serial.println("Setting AP (Access Point)");
  // NULL sets an open Access Point
  WiFi.softAP("Hydrofoil-Control", NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/set-output", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Check if all required parameters are present
    if (request->hasParam("output", true) && request->hasParam("value", true)) {
      // Extract parameters
      String output = request->getParam("output", true)->value();
      // get state bool parameter
      String value = request->getParam("value", true)->value();

      // Serial.println(output);
      // Serial.println(value);

      analogWrite(ledPin, value.toInt());
      writeFileJson(SPIFFS, jsonConfigPath, output.c_str(), value.c_str());
      sliderValue = readFileJson(SPIFFS, jsonConfigPath, output.c_str());
      Serial.println(sliderValue);

      ws.printfAll("{\"slider\":\"%s\"}", value.c_str());

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

  server.on("/add-mac", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Check if all required parameters are present
    if (request->hasParam("mac", true)) {
      // Extract parameters
      String mac = request->getParam("mac", true)->value();

      Serial.println(mac);

      // writeFileJson(SPIFFS, jsonConfigPath, output.c_str(), value.c_str());
      // sliderValue = readFileJson(SPIFFS, jsonConfigPath, output.c_str());
      // Serial.println(sliderValue);

      // ws.printfAll("{\"slider\":\"%s\"}", value.c_str());

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

  server.begin();
  Serial.print("Master has started");
};

void setupWifiSlave(){};

void setup() {

  // Enable the Watchdog Timer
  esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // Begin serial communication
  Serial.begin(115200);

  // Configure pin modes
  pinMode(ledPin, OUTPUT);

  // Mount SPIFFS
  initFS();

  // Load values saved in SPIFFS
  ssid = readFileJson(SPIFFS, jsonWifiPath, "SSID");
  pass = readFileJson(SPIFFS, jsonWifiPath, "PASS");
  slave = readFileJson(SPIFFS, jsonWifiPath, "SLAVE");
  first = readFileJson(SPIFFS, jsonWifiPath, "FIRST");
  sliderValue = readFileJson(SPIFFS, jsonConfigPath, "slider");

  macAddresses[0] = "MAC1";
  macAddresses[1] = "MAC2";
  macAddresses[2] = "MAC3";
  writeArrayJson(SPIFFS, jsonConfigPath, "addresses", macAddresses);
  readArrayJson(SPIFFS, jsonConfigPath, "addresses", macAddresses);
  Serial.println(macAddresses[0]);
  Serial.println(macAddresses[1]);
  Serial.println(macAddresses[2]);

  // Set up WebSocket event handler
  ws.onEvent(onWsEvent);

  // Add WebSocket handler to the server
  server.addHandler(&ws);

  if (first == "True")
    setupWifiFirst();
  else if (slave == "False") {
    setupWifiMaster();
  } else
    setupWifiSlave();

  // Initialize mDNS
  if (!MDNS.begin("hydrofoil-control")) {
    Serial.println("Error setting up mDNS.");
  } else {
    Serial.println("mDNS responder started");
  }

  // Add a service to mDNS
  MDNS.addService("http", "tcp", 80);
}

void loop() {

  // Clean up and close inactive WebSocket connections
  ws.cleanupClients();

  // Reset the Watchdog Timer to prevent a system reset
  esp_task_wdt_reset();

  // Resboot ESP after SSID and PASS were set
  if (restart) {
    delay(5000);
    ESP.restart();
  }
  delay(1000);
}
