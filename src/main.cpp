#include "SPIFFS.h"
#include "filehandling.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <QuickPID.h>
#include <TickTwo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <vector>

// Define pins for various functions
const int ledPin = GPIO_NUM_8;
const int resetPin = GPIO_NUM_5;
const int servoPin = GPIO_NUM_1;
const int pwmPin = GPIO_NUM_3;
const int capacitancePin = GPIO_NUM_6;
const int refPin = GPIO_NUM_7;

// Watchdog timeout in milliseconds
const int WATCHDOG_TIMEOUT = 4000;

// Variables for resetting device
unsigned long startTime = 0;
bool ledBlinked200ms = false;
bool ledBlinked5s = false;
bool ledBlinked10s = false;

// Variables to save values from HTML form
String slaveString;
String firstString;
String enableString;
String uploadString;
String routerSsid;
String routerPassword;

// State variables for setting up device
bool slave;
bool first;
bool wifiEnable;
bool upload;

// Variables for ESP-NOW
String macString;
String macAddresses[] = {"", ""};
uint8_t broadcastAddress[6];

// File paths to save input values permanently
const char *jsonWifiPath = "/wifi.json";
const char *jsonConfigsPath = "/configs.json";
const char *jsonAddressesPath = "/addresses.json";

// Setting hostname
const char *hostname = "hydrofoil-control";

// WiFi timer variables
unsigned long previousMillis = 0;
const long interval = 10000;
unsigned long currentMillis = 0;

// "Watchdog" variable for the filesystem
boolean restart = false;

// Create web server
AsyncWebServer server(80);

// Variables for managing device settings and data flow
dataStruct controlParams;
dataStruct tempParams;

allBoards boardsParams;

String boardSelector = "master";

esp_now_peer_info_t peerInfo;

// Variables for creating SSID and getting the channel of the AP
String deviceMac = "";
String ssid = "Hydrofoil-Control-";
int channel = 1;

// RC servo
Servo elevator;

// PID controller
float setpoint, input, output;
QuickPID elevatorPid(
    &input, &output, &setpoint, controlParams.p, controlParams.i,
    controlParams.d,
    elevatorPid.pMode::pOnErrorMeas,   /* pOnError, pOnMeas, pOnErrorMeas */
    elevatorPid.dMode::dOnMeas,        /* dOnError, dOnMeas */
    elevatorPid.iAwMode::iAwCondition, /* iAwCondition, iAwClamp, iAwOff */
    elevatorPid.Action::direct);       /* direct, reverse */
int servoPos;

// PWM Input variables
volatile long pwmRead = 0;
volatile int control = 0;
volatile unsigned long pulsInTimeBegin;
volatile unsigned long pulsInTimeEnd;
volatile bool newPulseDurationAvailable = false;

// Capacitance measurement
std::vector<int> rawValues;
hw_timer_t *timer = NULL;
volatile int minMeasured = 3000;
volatile int maxMeasured = 17000;
volatile int position = 0;
volatile int median = 0;
volatile int prevRawValue = 2000;

// Interruptable delay function
void delayWhile(long delayMillis) {
  long currentTime = 0;
  long startTime = millis();
  while (currentTime < startTime + delayMillis)
    currentTime = millis();
};

// Interruptable delay function
void delayWhileMicros(long delayMicros) {
  unsigned long currentTime = 0;
  unsigned long startTime = micros();
  while (currentTime < startTime + delayMicros)
    currentTime = micros();
};

// Callbacks for ESP-NOW send
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS
                     ? "Packet delivered successfully"
                     : "Packet delivery failed");
};

// Callbacks for ESP-NOW receive
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&controlParams, incomingData, sizeof(controlParams));
  elevatorPid.SetTunings(controlParams.p, controlParams.i, controlParams.d);
  // Slaves devices only receive and store their own values, so it's always
  // stored in slave1 struct regardless of the actual slave identifier
  boardsParams.slave1 = controlParams;
  // Write all params to flash memory
  writeStructJson(SPIFFS, jsonConfigsPath, boardsParams);
  if (controlParams.servoTarget == 3) {
    elevator.writeMicroseconds(controlParams.servoMin);
    delayWhile(2000);
  } else if (controlParams.servoTarget == 4) {
    elevator.writeMicroseconds(controlParams.servoMax);
    delayWhile(2000);
  }
}

// Convert string to bool
bool stringToBool(String state) {
  if (state == "true" || state == "1" || state == "True") {
    return true;
  } else
    return false;
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

// Add new ESP-NOW peer
void addNewPeerEspNow() {
  Serial.println("Adding new peer");
  // Add new address to the peerInfo
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  // Specify  channel and encryption
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else
    Serial.println("Peer added");
}

// Init ESP-NOW
void initEspNow() {
  esp_err_t resultOfEspNowInit = esp_now_init();
  Serial.println("Result of esp_now_init:");
  Serial.println(resultOfEspNowInit);
  if (resultOfEspNowInit != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else
    Serial.println("ESP-NOW initialized");
}

// Send data via ESP-NOW
void sendEspNow(uint8_t *broadcastAddress, const void *boardParams,
                size_t dataSize) {
  for (int i = 0; i <= 4; i++) {
    esp_err_t resultOfSend =
        esp_now_send(broadcastAddress, (uint8_t *)boardParams, dataSize);
    if (resultOfSend == ESP_OK) {
      Serial.println("Sent successfully");
      break;
    } else if (i == 4) {
      Serial.println("Faild to send data");
    }
  }
}

// Re-enable WiFi on master
void resetWifiEnable() {
  enableString = "True";
  writeFileJson(SPIFFS, jsonWifiPath, "enable", enableString.c_str());
}

// Reset MAC addresses
void resetMacAddresses() {
  for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
    macAddresses[i] = "";
  }
  writeArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses,
                 sizeof(macAddresses) / sizeof(macAddresses[0]));
  Serial.println("MAC Addresses have been deleted");
}

// Reset device
void resetDevice() {
  firstString = "True";
  slaveString = "False";
  enableString = "True";
  uploadString = "False";
  routerSsid = "";
  routerPassword = "";

  tempParams.p = 2;
  tempParams.i = 0;
  tempParams.d = 0;
  tempParams.setpoint = 1500;
  tempParams.factor = 40;
  tempParams.enable = 1;
  tempParams.servoMin = 1000;
  tempParams.servoMax = 2000;
  tempParams.servoTarget = 0;

  boardsParams.master = tempParams;
  boardsParams.slave1 = tempParams;
  boardsParams.slave2 = tempParams;

  // Write all params to flash memory
  writeFileJson(SPIFFS, jsonWifiPath, "first", firstString.c_str());
  writeFileJson(SPIFFS, jsonWifiPath, "slave", slaveString.c_str());
  writeFileJson(SPIFFS, jsonWifiPath, "enable", enableString.c_str());
  writeFileJson(SPIFFS, jsonWifiPath, "upload", uploadString.c_str());
  writeFileJson(SPIFFS, jsonWifiPath, "ssid", routerSsid.c_str());
  writeFileJson(SPIFFS, jsonWifiPath, "password", routerPassword.c_str());

  writeStructJson(SPIFFS, jsonConfigsPath, boardsParams);

  Serial.println("Device has been reset");
}

// Blink LED given times
void blinkLed(int times) {
  for (int i = 0; i <= times; i++) {
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(300);
    digitalWrite(ledPin, LOW);
  }
}

// Set upload variable to reboot to OTA Upload mode
void setUpload(bool enable) {
  uploadString = enable ? "true" : "false";
  writeFileJson(SPIFFS, jsonWifiPath, "upload", uploadString.c_str());
}

// Interrupt callback for non-blocking PWM reading
void pwmReadInterrupt() {
  if (digitalRead(pwmPin) == HIGH) {
    pulsInTimeBegin = micros();
  } else {
    pulsInTimeEnd = micros();
    pwmRead = pulsInTimeEnd - pulsInTimeBegin;
    if (pwmRead >= 985 && pwmRead <= 2015) {
      control = (pwmRead - 1500) / 100.00 * controlParams.factor;
    } else
      control = 0;
  }
};

// Calculate median of raw measurement values
int getMedian() {
  std::vector<int> temp = rawValues;
  std::sort(temp.begin(), temp.end());
  return temp[temp.size() / 2];
};

// Start charge up time measurement
void startMeasurement() {
  pinMode(capacitancePin, OUTPUT);
  digitalWrite(capacitancePin, LOW);
  delayMicroseconds(50);
  pinMode(capacitancePin, INPUT);
  timerRestart(timer);
  while (digitalRead(capacitancePin) == LOW)
    ;
  int rawValue = timerRead(timer);
  if (abs(rawValue - prevRawValue) < 3200) {
    rawValues.push_back(rawValue);
    prevRawValue = rawValue;
  }
  if (rawValues.size() > 10) {
    rawValues.erase(rawValues.begin());
  }
};

// Calculate water level from measurement
void calculatePosition() {
  if (rawValues.size() == 0) {
    return;
  }

  median = getMedian();

  float progress =
      static_cast<float>(median - minMeasured) / (maxMeasured - minMeasured);

  // Scale the progress value between 1000 and 2000us
  position = 1000 + 1000 * progress;
}

// Calculate PID output and move the servo accordingly
void calculatePid() {
  calculatePosition();

  input = position;
  setpoint = controlParams.setpoint + control;
  if (setpoint < 1000)
    setpoint = 1000;
  else if (setpoint > 2000)
    setpoint = 2000;
  elevatorPid.Compute();
  servoPos =
      map(output, 1000, 2000, controlParams.servoMin, controlParams.servoMax);
  elevator.writeMicroseconds(servoPos);
};

// Log params to UART
void logPid(){
    // Serial.print("measured:");
    // Serial.print(median);
    // Serial.print(":");
    // Serial.print("input:");
    // Serial.print(input);
    // Serial.print(":");
    // Serial.print("setpoint:");
    // Serial.print(setpoint);
    // Serial.print(":");
    // Serial.print("output:");
    // Serial.print(output);
    // Serial.print(":");
    // Serial.print("PWM read:");
    // Serial.print(pwmRead);
    // Serial.print(":");
    // Serial.print("control:");
    // Serial.println(control);
    // Serial.print("kp:");
    // Serial.print(controlParams.p);
    // Serial.print(":");
    // Serial.print("ki:");
    // Serial.print(controlParams.i);
    // Serial.print(":");
    // Serial.print("kd:");
    // Serial.println(controlParams.d);
};

// Set up tickers
TickTwo measurementTicker([]() { startMeasurement(); }, 1, 0, MILLIS);
TickTwo pidTicker([]() { calculatePid(); }, 10, 0, MILLIS);
TickTwo selfTestTicker([]() { calculatePosition(); }, 10, 0, MILLIS);
TickTwo loggerTicker([]() { logPid(); }, 50, 0, MILLIS);

// Set up wifi and webserver for first device start
void setupWifiFirst() {

  Serial.println("Setting up WiFi Station");
  WiFi.mode(WIFI_MODE_NULL);
  WiFi.setHostname(hostname);
  // NULL sets an open Access Point
  WiFi.softAP(ssid.c_str(), NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/wifimanager.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  // Send MAC address to client
  server.on("/get-mac", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", WiFi.macAddress().c_str());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress().c_str());
  });

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    slaveString = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "slave", slaveString.c_str());
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter *p = request->getParam(i);
      // HTTP POST slave value
      if (p->name() == "slave") {
        slaveString = "True";
        Serial.print("Slave device: ");
        Serial.println(slaveString);
        writeFileJson(SPIFFS, jsonWifiPath, "slave", slaveString.c_str());
        //
      } else if (p->name() == "ssid") {
        routerSsid = p->value();
        writeFileJson(SPIFFS, jsonWifiPath, "ssid", routerSsid.c_str());
        //
      } else if (p->name() == "password") {
        routerPassword = p->value();
        writeFileJson(SPIFFS, jsonWifiPath, "password", routerPassword.c_str());
      }
    }
    restart = true;
    request->send(200, "text/plain",
                  "Done. ESP will restart, and create Master hotspot, or "
                  "connect as a Slave.");

    firstString = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "first", firstString.c_str());
  });
  server.begin();
};

// Set up wifi, webserver and ESP-NOW for master device
void setupWifiMaster() {
  Serial.println("Setting up WiFi Station");
  // NULL sets an open Access Point
  WiFi.softAP(ssid.c_str(), NULL, channel);

  IPAddress IP = WiFi.softAPIP();
  if (wifiEnable)
    WiFi.mode(WIFI_AP_STA);
  else
    WiFi.mode(WIFI_OFF);
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Web Server Root URL
  server.serveStatic("/", SPIFFS, "/");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  // Send settings to client
  server.on("/get-settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<200> jsonDoc;

    if (boardSelector == "master") {
      tempParams = boardsParams.master;
    }

    else if (boardSelector == "slave-1") {
      tempParams = boardsParams.slave1;
    }

    else if (boardSelector == "slave-2") {
      tempParams = boardsParams.slave2;
    }
    jsonDoc["slider-p"] = tempParams.p;
    jsonDoc["slider-i"] = tempParams.i;
    jsonDoc["slider-d"] = tempParams.d;
    jsonDoc["slider-setpoint"] = tempParams.setpoint;
    jsonDoc["slider-factor"] = tempParams.factor;
    jsonDoc["slider-enable"] = tempParams.enable;
    jsonDoc["slider-servo-min"] = tempParams.servoMin;
    jsonDoc["slider-servo-max"] = tempParams.servoMax;
    jsonDoc["board-selector"] = boardSelector;

    String response;
    serializeJson(jsonDoc, response);

    request->send(200, "application/json", response);
    Serial.println("Settings have been sent");
  });

  // Send MAC addresses to client
  server.on("/get-addresses", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<200> jsonDoc;

    // Variables to send
    jsonDoc["broadcastAddress0"] = macAddresses[0];
    jsonDoc["broadcastAddress1"] = macAddresses[1];

    String response;
    serializeJson(jsonDoc, response);

    request->send(200, "application/json", response);
    Serial.println("MAC addresses have been sent");
  });

  // Send process values to client
  server.on("/get-values", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<200> jsonDoc;

    // Variables to send
    jsonDoc["PID-input"] = input;
    jsonDoc["PID-output"] = output;
    jsonDoc["PID-setpoint"] = setpoint;
    jsonDoc["PWM-input"] = pwmRead;
    jsonDoc["Servo-position"] = servoPos;
    jsonDoc["Min-measured-value"] = minMeasured;
    jsonDoc["Max-measured-value"] = maxMeasured;
    jsonDoc["Actual-measured-value"] = median;

    String response;
    serializeJson(jsonDoc, response);

    request->send(200, "application/json", response);
  });

  // Select board to configure
  server.on("/select-board", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("board-selector", true)) {
      // Send success response
      boardSelector = request->getParam(0)->value();
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
    Serial.println(boardSelector);
  });

  // Receive and manage param inputs
  server.on("/set-sliders", HTTP_POST, [](AsyncWebServerRequest *request) {
    String servoTarget;
    // Check if all required parameters are present
    if (request->hasParam("slider-p", true) &&
        request->hasParam("slider-i", true) &&
        request->hasParam("slider-d", true) &&
        request->hasParam("slider-setpoint", true) &&
        request->hasParam("slider-factor", true) &&
        request->hasParam("slider-enable", true) &&
        request->hasParam("slider-servo-min", true) &&
        request->hasParam("slider-servo-max", true) &&
        request->hasParam("servo-target", true) &&
        request->hasParam("board-selector", true)) {

      // Extract parameters
      tempParams.p = request->getParam("slider-p", true)->value().toFloat();
      tempParams.i = request->getParam("slider-i", true)->value().toFloat();
      tempParams.d = request->getParam("slider-d", true)->value().toFloat();
      tempParams.setpoint =
          request->getParam("slider-setpoint", true)->value().toInt();
      tempParams.factor =
          request->getParam("slider-factor", true)->value().toInt();
      tempParams.enable =
          request->getParam("slider-enable", true)->value().toInt();
      tempParams.servoMin =
          request->getParam("slider-servo-min", true)->value().toInt();
      tempParams.servoMax =
          request->getParam("slider-servo-max", true)->value().toInt();
      servoTarget = request->getParam("servo-target", true)->value().c_str();

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }

    // Set target based on JS event
    if (servoTarget.indexOf("master") != -1) {
      if (servoTarget.indexOf("servo-min") != -1) {
        tempParams.servoTarget = 1;
      } else if (servoTarget.indexOf("servo-max") != -1) {
        tempParams.servoTarget = 2;
      }
    } else if (servoTarget.indexOf("slave") != -1) {
      if (servoTarget.indexOf("servo-min") != -1) {
        tempParams.servoTarget = 3;
      } else if (servoTarget.indexOf("servo-max") != -1) {
        tempParams.servoTarget = 4;
      }
    } else {
      tempParams.servoTarget = 0;
    }

    // Store received values, send slave params to devices
    if (boardSelector == "master") {
      boardsParams.master = tempParams;
      controlParams = boardsParams.master;
      elevatorPid.SetTunings(controlParams.p, controlParams.i, controlParams.d);
    }

    else if (boardSelector == "slave-1") {
      boardsParams.slave1 = tempParams;
      if (strcmp(macAddresses[0].c_str(), "") != 0) {
        stringToMac(macAddresses[0], broadcastAddress);
        sendEspNow(broadcastAddress, &boardsParams.slave1,
                   sizeof(boardsParams.slave1));
      }
    }

    else if (boardSelector == "slave-2") {
      boardsParams.slave2 = tempParams;
      if (strcmp(macAddresses[1].c_str(), "") != 0) {
        stringToMac(macAddresses[1], broadcastAddress);
        sendEspNow(broadcastAddress, &boardsParams.slave2,
                   sizeof(boardsParams.slave2));
      }
    }

    // Move servo to endposition if min or max slider was moved
    if (controlParams.servoTarget == 1) {
      elevator.writeMicroseconds(controlParams.servoMin);
      delayWhile(2000);
      controlParams.servoTarget = 0;
    } else if (controlParams.servoTarget == 2) {
      elevator.writeMicroseconds(controlParams.servoMax);
      delayWhile(2000);
      controlParams.servoTarget = 0;
    }

    // Write all params to flash memory
    writeStructJson(SPIFFS, jsonConfigsPath, boardsParams);
  });

  // Add MAC address to ESP-NOW broadcast addresses
  server.on("/add-mac", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Check if all required parameters are present
    if (request->hasParam("mac", true)) {
      // Extract parameters
      String macString = request->getParam("mac", true)->value();
      // Send success response
      request->send(200, "text/plain", "OK");

      Serial.println("MAC Address from POST request");
      Serial.println(macString);

      bool existing = false;
      bool added = false;
      // Find empty space in the array, store the value then add new peer
      for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
        if (macAddresses[i] == macString)
          existing = true;
        if (macAddresses[i] == "" && !existing) {
          macAddresses[i] = macString;
          stringToMac(macAddresses[i], broadcastAddress);
          added = true;
          break;
        } else if (existing) {
          Serial.println("Address already added");
          Serial.println(macAddresses[i]);
          break;
        }
      }

      // Store value in the JSON file, add new ESP NOW peer
      if (!existing && added) {
        writeArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses,
                       sizeof(macAddresses) / sizeof(macAddresses[0]));
        Serial.println("New MAC address stored");

        addNewPeerEspNow();
      } else if (!existing && !added) {
        Serial.println("Cannot connect more slave devices");
      }

    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

  // Turn off WiFi on master device
  server.on("/wifi-off", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("wifi-off", true)) {
      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
    enableString = "False";
    writeFileJson(SPIFFS, jsonWifiPath, "enable", enableString.c_str());
    ESP.restart();
  });

  // Remove added slave devices from master
  server.on("/remove-slaves", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("remove-slaves", true)) {
      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
    resetMacAddresses();
  });

  if (wifiEnable) {
    server.begin();

    // Init ESP-NOW
    initEspNow();

    // Once ESP-NOW is successfully Init, register sender callback
    esp_err_t resultOfRegisterSend = esp_now_register_send_cb(onDataSent);
    Serial.println("Result of esp_now_register_send_cb:");
    Serial.println(resultOfRegisterSend);

    // Set up peers at boot up
    for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
      if (strcmp(macAddresses[i].c_str(), "") != 0) {
        stringToMac(macAddresses[i], broadcastAddress);
        addNewPeerEspNow();
      }
    }
  }

  // Send params to slave 1 at startup
  if (strcmp(macAddresses[0].c_str(), "") != 0) {
    stringToMac(macAddresses[0], broadcastAddress);
    sendEspNow(broadcastAddress, &boardsParams.slave1,
               sizeof(boardsParams.slave1));
  }

  // Send params to slave 2 at startup
  if (strcmp(macAddresses[1].c_str(), "") != 0) {
    stringToMac(macAddresses[1], broadcastAddress);
    sendEspNow(broadcastAddress, &boardsParams.slave2,
               sizeof(boardsParams.slave2));
  }

  Serial.println("Master has started");
  Serial.print("Wifi is enabled: ");
  Serial.println(wifiEnable);
};

// Set up wifi and ESP-NOW for slave device
void setupWifiSlave() {
  // Init ESP-NOW
  Serial.println("Start slave initialization");
  WiFi.mode(WIFI_STA);

  // Get the channel of the Master AP and set Slave's channel accordingly
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.println("WiFi diagnostics");
  WiFi.printDiag(Serial);

  initEspNow();

  // Once ESP-NOW is successfully Init, register receiver callback
  esp_err_t resultOfRegisterReveived = esp_now_register_recv_cb(onDataRecv);
  Serial.println("Result of esp_now_register_recv_cb:");
  Serial.println(resultOfRegisterReveived);

  Serial.println("Slave has started");
  Serial.println("Device\'s MAC Address:");
  Serial.println(WiFi.macAddress());
}

void setupWifiUpload() {
  // Check if SSID is provided
  if (routerSsid.isEmpty()) {
    Serial.println("No SSID provided");
    setUpload(false);
    ESP.restart();
  }
  // Set the ESP32 to Station mode
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(routerSsid.c_str(), routerPassword.c_str());

  Serial.println("Connecting to WiFi...");
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 10000; // 10 seconds timeout

  // Wait for connection or timeout
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime >= wifiTimeout) {
      Serial.println("Failed to connect to WiFi.");
      setUpload(false);
    }
    delay(500);        // Small delay to avoid busy-waiting
    Serial.print("."); // Print dots while waiting for connection
  }

  Serial.println("\nConnected to WiFi.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  WiFi.setHostname(hostname);
  Serial.print("Hostname set to: ");
  Serial.println(WiFi.getHostname());

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/upload.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.begin();

  // Initialize ArduinoOTA with a hostname and start
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() { Serial.println("OTA update started"); });
  ArduinoOTA.begin();
};

void setup() {
  // Enable the Watchdog Timer
  esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // Begin serial communication
  Serial.begin(115200);
  delayWhile(500);
  Serial.println("Booting");

  // Configure pin modes
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPin, INPUT);
  pinMode(capacitancePin, INPUT);

  // Mount SPIFFS
  initFS();

  // Load values saved in SPIFFS
  slaveString = readFileJson(SPIFFS, jsonWifiPath, "slave");
  slave = stringToBool(slaveString);
  firstString = readFileJson(SPIFFS, jsonWifiPath, "first");
  first = stringToBool(firstString);
  enableString = readFileJson(SPIFFS, jsonWifiPath, "enable");
  wifiEnable = stringToBool(enableString);
  uploadString = readFileJson(SPIFFS, jsonWifiPath, "upload");
  upload = stringToBool(uploadString);
  routerSsid = readFileJson(SPIFFS, jsonWifiPath, "ssid");
  routerPassword = readFileJson(SPIFFS, jsonWifiPath, "password");

  readStructJson(SPIFFS, jsonConfigsPath, boardsParams);

  // Read MAC Addresses from JSON and store to macAddresses array
  readArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses);
  Serial.println("MAC addresses from SPIFFS:");
  for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
    if (macAddresses[i] != "")
      Serial.println(macAddresses[i]);
  }

  // Create unique SSID
  deviceMac = WiFi.macAddress().c_str();
  ssid += deviceMac;

  // Configure devices according to first and slave variables
  if (first)
    setupWifiFirst();
  else if (upload) {
    setupWifiUpload();
  } else if (!slave) {
    controlParams = boardsParams.master;
    setupWifiMaster();
  } else {
    // Slave devices always store their params in slave1 struct, regardless of
    // their actual identifier
    controlParams = boardsParams.slave1;
    setupWifiSlave();
  }

  if (!slave) {
    // Initialize mDNS
    if (!MDNS.begin("hydrofoil-control")) {
      Serial.println("Error setting up mDNS.");
    } else {
      Serial.println("mDNS responder started");
    }
    // Add a service to mDNS
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS service added");
  }

  // Set up Servo
  elevator.attach(servoPin);
  Serial.println("Servo has been attached to servo pin");
  // elevator.write(controlParams.servoMid);

  // Apply PID gains
  elevatorPid.SetTunings(controlParams.p, controlParams.i, controlParams.d);
  Serial.println("PID tunings have been set");

  // Turn the PID on
  elevatorPid.SetMode(elevatorPid.Control::timer);
  elevatorPid.SetOutputLimits(1000, 2000);
  Serial.println("PID mode has been set to timer");

  // Interrupt for non-blocking PWM reading
  attachInterrupt(digitalPinToInterrupt(pwmPin), pwmReadInterrupt, CHANGE);

  Serial.println("Interrupts have been attached");

  // Set up timers for capacitance measurement
  timer = timerBegin(0, 2, true);
  Serial.println("Measurement timer has been set");

  // Set up ticker for measurement
  measurementTicker.start();

  // Set up ticker for the PID control
  pidTicker.start();

  // Set up ticker for selftest measurement
  selfTestTicker.start();

  // Set up ticker for the logger
  loggerTicker.start();

  // Set up ticker for the logger
  Serial.println("Tickers have been started");
  Serial.println("Setup is complete");
}

void loop() {
  // Reset the Watchdog Timer to prevent a system reset
  esp_task_wdt_reset();

  // Resboot ESP after SSID and PASS were set
  if (restart) {
    delay(5000);
    ESP.restart();
  }

  // Reset device
  if (digitalRead(resetPin) == LOW) {
    startTime = millis(); // Record the time when the reset button is pressed
    digitalWrite(ledPin, LOW);

    while (digitalRead(resetPin) == LOW) {
      unsigned long elapsedTime = millis() - startTime;

      if (elapsedTime >= 10000 &&
          !ledBlinked10s) { // Held for 10 seconds or more
        blinkLed(3);        // 3 blinks signal 10 seconds reached
        ledBlinked10s = true;
      } else if (elapsedTime >= 5000 &&
                 !ledBlinked5s) { // Held for 5 seconds or more
        blinkLed(2);              // 2 blinks signal 5 seconds reached
        ledBlinked5s = true;
      } else if (elapsedTime >= 200 &&
                 !ledBlinked200ms) { // Held for 200 ms or more
        blinkLed(1);                 // 1 blink signals 200 ms reached
        ledBlinked200ms = true;
      }
    }

    // Execute operation based on how long the button was held after release
    if (ledBlinked10s) {
      setUpload(true); // Reset to default settings
    } else if (ledBlinked5s) {
      resetDevice(); // Reset MAC Addresses
    } else if (ledBlinked200ms) {
      resetWifiEnable(); // Reset WiFi
    }

    // Restart after executing the operation
    ESP.restart();
  }

  // Code that runs in First Start Mode only
  // Selftest
  if (first) {
    int pwmOut = 0;
    measurementTicker.update();
    delayMicroseconds(200);
    selfTestTicker.update();
    Serial.print("Pisition: ");
    Serial.println(position);

    if (Serial.available() > 0) {
      // Read the incoming data as a string
      String incomingString = Serial.readStringUntil('\n');

      // Convert the received string to an integer and write to servo output
      pwmOut = incomingString.toInt();
      if (pwmOut > 990 && pwmOut < 2010) {
        elevator.writeMicroseconds(pwmOut);
      }

      // You can now use the receivedNumber variable as needed
    }
    // Serial.print("Median: ");
    // Serial.println(median);
    // Serial.print("PWM Out: ");
    // Serial.println(pwmOut);
    // Serial.print("PWM In: ");
    // Serial.println(pwmRead);
    // delay(100);
  }

  // Code that runs only after the device has been configured either as a master
  // or a slave
  if (!first) {
    // Handle OTA updates for the Arduino board
    if (upload) {
      ArduinoOTA.handle();
    }

    measurementTicker.update();
    delayWhileMicros(200);
    if (controlParams.enable == 1)
      pidTicker.update();
    else {
      if (controlParams.servoMax > controlParams.servoMin) {
        int midPos = (controlParams.servoMax - controlParams.servoMin) / 2 +
                     controlParams.servoMin;

        int manualPos = midPos + control;
        if (manualPos < controlParams.servoMin)
          elevator.writeMicroseconds(controlParams.servoMin);
        else if (manualPos > controlParams.servoMax)
          elevator.writeMicroseconds(controlParams.servoMax);
        else
          elevator.writeMicroseconds(manualPos);
      } else {

        int midPos = (controlParams.servoMin - controlParams.servoMax) / 2 +
                     controlParams.servoMax;

        int manualPos = midPos - control;
        if (manualPos < controlParams.servoMax)
          elevator.writeMicroseconds(controlParams.servoMax);
        else if (manualPos > controlParams.servoMin)
          elevator.writeMicroseconds(controlParams.servoMin);
        else
          elevator.writeMicroseconds(manualPos);
      }
    }

    if (input + 30 > controlParams.setpoint &&
        input - 30 < controlParams.setpoint)
      digitalWrite(ledPin, LOW);
    else
      digitalWrite(ledPin, HIGH);

    // loggerTicker.update();
  }
}
