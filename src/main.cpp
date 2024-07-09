#include "SPIFFS.h"
#include "filehandling.h"
#include <Arduino.h>
#include <ArduinoJson.h>
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

// Variables to save values from HTML form
String slaveString;
String firstString;
String enableString;

// State variables for setting up device
bool slave;
bool first;
bool wifiEnable;

// Variables for ESP-NOW
String macString;
String macAddresses[] = {"", ""};
uint8_t broadcastAddress[6];

// File paths to save input values permanently
const char *jsonWifiPath = "/wifi.json";
const char *jsonConfigPath = "/config.json";
const char *jsonConfigsPath = "/configs.json";
const char *jsonAddressesPath = "/addresses.json";

// Setting hostname
const char *hostname = "hydrofoil-control";

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
    elevatorPid.pMode::pOnMeas,        /* pOnError, pOnMeas, pOnErrorMeas */
    elevatorPid.dMode::dOnMeas,        /* dOnError, dOnMeas */
    elevatorPid.iAwMode::iAwCondition, /* iAwCondition, iAwClamp, iAwOff */
    elevatorPid.Action::direct);       /* direct, reverse */
int servoPos;

// Cycle time
unsigned long prevTime = 0;
unsigned long currTime = 0;

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
  // if (controlParams.servoTarget == 1) {
  //   elevator.writeMicroseconds(controlParams.servoMin);
  //   delayWhile(2000);
  // } else if (controlParams.servoTarget == 2) {
  //   elevator.writeMicroseconds(controlParams.servoMax);
  //   delayWhile(2000);
  // }
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

// Reset Wifi enable
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

  writeStructJson(SPIFFS, jsonConfigsPath, boardsParams);

  Serial.println("Device has been reset");
}

// Blink LED given times
void blinkLed(int times) {
  for (int i = 0; i <= times; i++) {
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
  }
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

// Calculate PID output and move the servo accordingly
void calculatePid() {
  if (rawValues.size() == 0) {
    return;
  }

  median = getMedian();

  float progress =
      static_cast<float>(median - minMeasured) / (maxMeasured - minMeasured);

  // Scale the progress value between 1000 and 2000us
  position = 1000 + 1000 * progress;

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

void logPid() {
  Serial.print("measured:");
  Serial.print(median);
  Serial.print(":");
  Serial.print("input:");
  Serial.print(input);
  Serial.print(":");
  Serial.print("setpoint:");
  Serial.print(setpoint);
  Serial.print(":");
  Serial.print("output:");
  Serial.print(output);
  Serial.print(":");
  Serial.print("PWM read:");
  Serial.print(pwmRead);
  Serial.print(":");
  Serial.print("control:");
  Serial.println(control);
  // Serial.print("kp:");
  // Serial.print(controlParams.p);
  // Serial.print(":");
  // Serial.print("ki:");
  // Serial.print(controlParams.i);
  // Serial.print(":");
  // Serial.print("kd:");
  // Serial.println(controlParams.d);
};

TickTwo measurementTicker([]() { startMeasurement(); }, 1, 0, MILLIS);
TickTwo pidTicker([]() { calculatePid(); }, 10, 0, MILLIS);
TickTwo loggerTicker([]() { logPid(); }, 50, 0, MILLIS);

// Set up wifi and webserver for first device start
void setupWifiFirst() {

  Serial.println("Setting AP (Access Point)");
  WiFi.mode(WIFI_MODE_NULL);
  WiFi.setHostname("hydrofoil-control");
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
      if (p->isPost()) {

        // HTTP POST slave value
        if (p->name() == "slave") {
          slaveString = "True";
          Serial.print("Slave device: ");
          Serial.println(slaveString);
          // Write file to save value
          writeFileJson(SPIFFS, jsonWifiPath, "slave", slaveString.c_str());
        }
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
  Serial.println("Setting AP (Access Point)");
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

  server.on("/set-sliders", HTTP_POST, [](AsyncWebServerRequest *request) {
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
      tempParams.servoTarget =
          request->getParam("servo-target", true)->value().c_str();

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }

    // Store received values, handle master's servo end positions
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
    if (strcmp(controlParams.servoTarget, "master-slider-servo-min") == 0) {
      elevator.writeMicroseconds(controlParams.servoMin);
      delayWhile(2000);
    } else if (strcmp(controlParams.servoTarget, "master-slider-servo-max") ==
               0) {
      elevator.writeMicroseconds(controlParams.servoMax);
      delayWhile(2000);
    } else
      controlParams.servoTarget = "";

    // Write all params to flash memory
    writeStructJson(SPIFFS, jsonConfigsPath, boardsParams);
  });

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
      if (macAddresses[i] != "") {
        stringToMac(macAddresses[i], broadcastAddress);
        addNewPeerEspNow();
      }
    }
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
  else if (!slave) {
    controlParams = boardsParams.master;
    setupWifiMaster();
  } else {
    if (macAddresses[0] == deviceMac)
      controlParams = boardsParams.slave1;
    else
      controlParams = boardsParams.slave2;
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

  // Interrupt for capacitance measurement
  // attachInterrupt(digitalPinToInterrupt(capacitancePin), finishMeasurement,
  //                 RISING);
  Serial.println("Interrupts have been attached");

  // Set up timers for capacitance measurement
  timer = timerBegin(0, 2, true);
  Serial.println("Measurement timer has been set");

  // Set up ticker for measurement
  measurementTicker.start();

  // Set up ticker for the PID control
  pidTicker.start();

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
    digitalWrite(ledPin, LOW);
    delayWhile(200);
    if (digitalRead(resetPin) == LOW) {
      // Reset Wifi enable after 200 ms
      resetWifiEnable();
      blinkLed(1);
      delayWhile(2300);
      if (digitalRead(resetPin) == LOW) {
        // Reset MAC Addresses only after 3s
        resetMacAddresses();
        blinkLed(2);
        delayWhile(2000);
        if (digitalRead(resetPin) == LOW) {
          // Reset to default after 6s
          resetDevice();
          blinkLed(3);
          ESP.restart();
        }
      } else
        ESP.restart();
    } else
      ESP.restart();
  }
  // Code that runs only after the device has been configured as master or
  // slave
  if (!first) {
    measurementTicker.update();
    delayWhileMicros(200);
    if (controlParams.enable == 1)
      pidTicker.update();
    else {
      int midPos = (controlParams.servoMax - controlParams.servoMin) / 2 +
                   controlParams.servoMin;
      if (midPos >= 1000 && midPos <= 2000)
        elevator.writeMicroseconds(midPos);
    }

    // loggerTicker.update();

    // Master's main loop
    if (!slave) {
      analogWrite(ledPin, output);

      // Slave's main loop
    } else {
      analogWrite(ledPin, output);
    }
  }
}
