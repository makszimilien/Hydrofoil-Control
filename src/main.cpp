#include "SPIFFS.h"
#include "filehandling.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>

// Watchdog timeout in milliseconds
const int WATCHDOG_TIMEOUT = 8000;

// WiFi timer variables
unsigned long previousMillis = 0;
const long interval = 10000;
unsigned long currentMillis = 0;

// Variables to save values from HTML form
String slaveString;
String firstString;
String ip = "192.168.1.200";
String gateway = "192.168.1.1";
String sliderValue;

// State variables for setting up device
bool slave;
bool first;

// Variables for ESP-NOW
String macString;
String macAddresses[] = {"", "", "", "", ""};
uint8_t broadcastAddress1[6];
uint8_t broadcastAddress2[] = {0x70, 0xB8, 0xF6, 0x5C, 0xCD, 0xAC};
uint8_t broadcastAddress3[6];
uint8_t broadcastAddress4[6];
uint8_t broadcastAddress5[6];

// File paths to save input values permanently
const char *jsonWifiPath = "/wifi.json";
const char *jsonConfigPath = "/config.json";
const char *jsonAddressesPath = "/addresses.json";

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

// Variables for ESP-NOW
typedef struct dataStruct {
  int p;
  int i;
  int d;
} dataStruct;

dataStruct pidParamsSend;
dataStruct pidParamsReceive;
esp_now_peer_info_t peerInfo;

// Callbacks for ESP-NOW send and Receive
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS
                     ? "Packet delivered successfully"
                     : "Packet delivery failed");
};

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&pidParamsReceive, incomingData, sizeof(pidParamsReceive));
  analogWrite(ledPin, pidParamsReceive.p);
}

// Convert string to bool
bool stringToBool(String state) {
  if (state == "true" || state == "1" || state == "True") {
    return true;
  } else
    return false;
}
// Send addresses from string array through websocket
// Modify later to have only one ws.printfAll() by concatenating into one string
void sendAddresses() {
  for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
    String macAddress = macAddresses[i];
    ws.printfAll("{\"broadcastAddress%d\":\"%s\"}", i, macAddress.c_str());
    Serial.println(macAddress);
  }
}

// Send data through websocket when page reloaded
void sendData() {
  ws.printfAll("{\"mac\":\"%s\",\"slider\":\"%s\"}", WiFi.macAddress().c_str(),
               sliderValue.c_str());
  sendAddresses();
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

// Convert MAC string into intiger array
void stringToMac(String macString, u_int8_t *array) {
  int j = 0;
  for (int i = 0; i < macString.length(); i = i + 3) {
    String hexPairString = macString.substring(i, i + 2);
    array[j] = strtol(hexPairString.c_str(), nullptr, 16);
    Serial.print(array[j]);
    j++;
    if (j < 6)
      Serial.print(":");
    else
      Serial.println("");
  }
};

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
    slaveString = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "SLAVE", slaveString.c_str());
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter *p = request->getParam(i);
      if (p->isPost()) {

        // HTTP POST slave value
        if (p->name() == "slave") {
          slaveString = "True";
          Serial.print("Slave device: ");
          Serial.println(slaveString);
          // Write file to save value
          writeFileJson(SPIFFS, jsonWifiPath, "SLAVE", slaveString.c_str());
        }
      }
    }
    restart = true;
    request->send(200, "text/plain",
                  "Done. ESP will restart, and create Master hotspot, or "
                  "connect as a Slave.");
    firstString = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "FIRST", firstString.c_str());
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
      // Get value from POST request
      String value = request->getParam("value", true)->value();

      analogWrite(ledPin, value.toInt());
      writeFileJson(SPIFFS, jsonConfigPath, output.c_str(), value.c_str());
      sliderValue = readFileJson(SPIFFS, jsonConfigPath, output.c_str());

      Serial.println("Slider value:");
      Serial.println(sliderValue);

      ws.printfAll("{\"slider\":\"%s\"}", value.c_str());

      // Store value in datastruct for ESP-NOW
      pidParamsSend.p = sliderValue.toInt();
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
      String macString = request->getParam("mac", true)->value();

      Serial.println("MAC Address from POST request");
      Serial.println(macString);

      // Proper array handling needed later for more addresses !!
      macAddresses[0] = macString;
      writeArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses);
      readArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses);
      stringToMac(macAddresses[0], broadcastAddress1);
      sendAddresses();

      memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
      } else
        Serial.println("Peer added");

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

  server.begin();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else
    Serial.println("ESP-NOW initialized");

  esp_err_t resultOfRegisterSend = esp_now_register_send_cb(onDataSent);
  Serial.println("Result of esp_now_register_send_cb:");
  Serial.println(resultOfRegisterSend);
  // Specify  channel and encryption
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  Serial.println("Master has started");
};

void setupWifiSlave() {
  // Init ESP-NOW
  Serial.println("Start slave initialization");
  WiFi.mode(WIFI_STA);
  esp_err_t resultOfSlaveInit = esp_now_init();
  Serial.println("Result of esp_now_init (Slave):");
  Serial.println(resultOfSlaveInit);
  if (resultOfSlaveInit != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else
    Serial.println("ESP-NOW initialized");

  Serial.println("Register callback for on-receive");
  // Once ESPNow is successfully Init, register receiver CB to
  // get receiver packer info
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Slave has started");
  Serial.println("Device\'s MAC Address:");
  Serial.println(WiFi.macAddress());
}

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
  slaveString = readFileJson(SPIFFS, jsonWifiPath, "SLAVE");
  slave = stringToBool(slaveString);
  firstString = readFileJson(SPIFFS, jsonWifiPath, "FIRST");
  first = stringToBool(firstString);
  sliderValue = readFileJson(SPIFFS, jsonConfigPath, "slider");
  analogWrite(ledPin, sliderValue.toInt());

  // Read MAC Addresses from JSON and store to macAddresses array
  readArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses);

  if (!slave) { // Set up WebSocket event handler
    ws.onEvent(onWsEvent);

    // Add WebSocket handler to the server
    server.addHandler(&ws);
  }

  // Configure devices according to first and slave variables
  if (first)
    setupWifiFirst();
  else if (!slave) {
    setupWifiMaster();
  } else
    setupWifiSlave();

  if (!slave) { // Initialize mDNS
    if (!MDNS.begin("hydrofoil-control")) {
      Serial.println("Error setting up mDNS.");
    } else {
      Serial.println("mDNS responder started");
    }
    // Add a service to mDNS
    MDNS.addService("http", "tcp", 80);
  }
}

void loop() {

  // Clean up and close inactive WebSocket connections
  if (!slave) {
    ws.cleanupClients();
  }

  // Reset the Watchdog Timer to prevent a system reset
  esp_task_wdt_reset();

  // Resboot ESP after SSID and PASS were set
  if (restart) {
    delay(5000);
    ESP.restart();
  }

  // Send through ESP-NOW
  if (!slave) {
    esp_err_t resultOfSend =
        esp_now_send(0, (uint8_t *)&pidParamsSend, sizeof(dataStruct));
    Serial.println("Result of esp_now_send (Master):");
    Serial.println(
        resultOfSend); // Returns 12393 (0x3069): ESP_ERR_ESPNOW_NOT_FOUND
                       // (0x3069): ESPNOW peer is not found
    if (resultOfSend == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }

  } else if (!first)
    analogWrite(ledPin, pidParamsReceive.p);

  delay(1000);
}
