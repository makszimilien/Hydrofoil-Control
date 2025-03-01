#include "SPIFFS.h"
#include "driver/rmt.h"
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

// Non-blocking timer variables
unsigned long previousMillis = 0;
const long interval = 2000;
unsigned long currentMillis = 0;

// Self test variables
bool startTest = false;
int pwmOut = 1000;

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
String ssid = "HF-Control-";
int channel = 1;

// RC servo
Servo elevator;

// PID controller
float setpoint, input, output;
QuickPID elevatorPid(
    &input, &output, &setpoint, controlParams.p, controlParams.i,
    controlParams.d,
    elevatorPid.pMode::pOnError,   /* pOnError, pOnMeas, pOnErrorMeas */
    elevatorPid.dMode::dOnError,   /* dOnError, dOnMeas */
    elevatorPid.iAwMode::iAwClamp, /* iAwCondition, iAwClamp, iAwOff */
    elevatorPid.Action::direct);   /* direct, reverse */
int servoPos;

// PWM Input variables
volatile long pwmRead = 0;
volatile int control = 0;

#define RMT_RX_CHANNEL RMT_CHANNEL_3 // Use RMT channel 0
#define RMT_RX_GPIO GPIO_NUM_3       // GPIO3 for PWM input
#define RMT_CLK_DIV 40               // 1 MHz resolution (40 MHz / 40)
#define RMT_FILTER_US 50             // Ignore pulses < 50µs
#define RMT_IDLE_THRES_US 20100      // Idle threshold at 20.1 ms (for 50Hz PWM)

volatile uint32_t highPulseDuration = 0; // Latest high pulse duration
RingbufHandle_t rb =
    NULL; // RMT ring buffer handle             // RMT ring buffer

// Capacitance measurement
std::vector<int> rawValues;
hw_timer_t *timer = NULL;
volatile int position = 0;
volatile int median = 0;

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
// Slaves devices only receive and store their own values, so it's always
// stored in slave1 struct regardless of the actual slave identifier
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&boardsParams.slave1, incomingData, sizeof(controlParams));

  if (boardsParams.slave1.servoMin != controlParams.servoMin) {
    elevator.writeMicroseconds(boardsParams.slave1.servoMin);
    delayWhile(2000);
  } else if (boardsParams.slave1.servoMax != controlParams.servoMax) {
    elevator.writeMicroseconds(boardsParams.slave1.servoMax);
    delayWhile(2000);
  }

  // Save the min and maxMeasured values of the slave before refreshing
  // controlParams

  if (boardsParams.slave1.calibration != controlParams.calibration) {
    if (boardsParams.slave1.calibration == 1) {
      boardsParams.slave1.minMeasured = 8000;
      boardsParams.slave1.maxMeasured = 9000;
    }
  } else {
    boardsParams.slave1.minMeasured = controlParams.minMeasured;
    boardsParams.slave1.maxMeasured = controlParams.maxMeasured;
  }
  controlParams = boardsParams.slave1;
  elevatorPid.SetTunings(controlParams.p, controlParams.i, controlParams.d);

  // Write all params to flash memory
  writeStructJson(SPIFFS, jsonConfigsPath, boardsParams);
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

  tempParams.p = 1;
  tempParams.i = 0;
  tempParams.d = 0;
  tempParams.setpoint = 1500;
  tempParams.factor = 40;
  tempParams.enable = 1;
  tempParams.calibration = 0;
  tempParams.servoMin = 1000;
  tempParams.servoMax = 2000;
  tempParams.minMeasured = 3000;
  tempParams.maxMeasured = 17000;

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

// Calculate median of raw measurement values
int getMedian(const std::vector<int> &values) {
  std::vector<int> temp = values;
  std::sort(temp.begin(), temp.end());
  return temp[temp.size() / 2];
};

void getPulseDuration() {
  size_t rx_size = 0;
  rmt_item32_t *items = (rmt_item32_t *)xRingbufferReceiveFromISR(rb, &rx_size);

  if (items && rx_size >= sizeof(rmt_item32_t)) {
    pwmRead = items[0].duration0 / 2; // Store high time in microseconds
    vRingbufferReturnItemFromISR(rb, (void *)items, NULL);
    if (pwmRead >= 985 && pwmRead <= 2015) {
      control = (pwmRead - 1500) / 100.00 * controlParams.factor;
    } else
      control = 0;
  }
}

void setupRMTReceiver() {
  rmt_config_t rmt_rx_config = {};
  rmt_rx_config.rmt_mode = RMT_MODE_RX;
  rmt_rx_config.channel = RMT_RX_CHANNEL;
  rmt_rx_config.gpio_num = RMT_RX_GPIO;
  rmt_rx_config.clk_div = RMT_CLK_DIV;
  rmt_rx_config.mem_block_num = 1;
  rmt_rx_config.rx_config.filter_en = true;
  rmt_rx_config.rx_config.filter_ticks_thresh = RMT_FILTER_US;
  rmt_rx_config.rx_config.idle_threshold = RMT_IDLE_THRES_US;

  esp_err_t err = rmt_config(&rmt_rx_config);
  if (err != ESP_OK) {
    Serial.print("RMT config failed with error: ");
    Serial.println(esp_err_to_name(err));
  }

  if (rmt_driver_install(RMT_RX_CHANNEL, 2000, 0) != ESP_OK) {
    Serial.println("RMT driver install failed!");
    return;
  }

  // Get ring buffer handle
  if (rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb) != ESP_OK || rb == NULL) {
    Serial.println("Failed to get RMT ring buffer!");
    return;
  }

  rmt_rx_start(RMT_RX_CHANNEL, true);
}

// Start charge up time measurement
void startMeasurement() {
  pinMode(capacitancePin, OUTPUT);
  digitalWrite(capacitancePin, LOW);
  delayMicroseconds(50);
  pinMode(capacitancePin, INPUT);
  noInterrupts();
  timerRestart(timer);
  while (digitalRead(capacitancePin) == LOW)
    ;
  int rawValue = timerRead(timer);
  interrupts();

  rawValues.push_back(rawValue);
  if (rawValues.size() > 20) {
    rawValues.erase(rawValues.begin());
  }
};

// Calculate water level from measurement
void calculatePosition() {
  if (rawValues.size() == 0) {
    return;
  }

  median = getMedian(rawValues);
  if (controlParams.calibration == 1) {
    if (median < controlParams.minMeasured)
      controlParams.minMeasured = median;
    else if (median > controlParams.maxMeasured)
      controlParams.maxMeasured = median;
  }
  float progress = static_cast<float>(median - controlParams.minMeasured) /
                   (controlParams.maxMeasured - controlParams.minMeasured);

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
      map(output, -500, 500, controlParams.servoMin, controlParams.servoMax);
  elevator.writeMicroseconds(servoPos);
};

// Log params to UART
void logPid() {
  // Serial.print("measured:");
  // Serial.print(median);
  // Serial.print(":");
  Serial.print("Input: ");
  Serial.print(input);
  // Serial.print("  ");
  // Serial.print("setpoint: ");
  // Serial.print(setpoint);
  Serial.print("  Output: ");
  Serial.print(output);
  // Serial.print("  ");
  // Serial.print("PWM read: ");
  // Serial.print(pwmRead);
  Serial.print("  PWM value: ");
  Serial.print(pwmRead);
  Serial.print("  Free heap memory: ");
  Serial.println(ESP.getFreeHeap());
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
TickTwo measurementTicker([]() { startMeasurement(); }, 2, 0, MILLIS);
TickTwo pidTicker([]() { calculatePid(); }, 10, 0, MILLIS);
TickTwo loggerTicker([]() { logPid(); }, 100, 0, MILLIS);
TickTwo rmtTicker([]() { getPulseDuration(); }, 20, 0, MILLIS);

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
    jsonDoc["slider-calibration"] = tempParams.calibration;
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
    jsonDoc["Min-measured-value"] = controlParams.minMeasured;
    jsonDoc["Max-measured-value"] = controlParams.maxMeasured;
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
    // Check if all required parameters are present
    if (request->hasParam("slider-p", true) &&
        request->hasParam("slider-i", true) &&
        request->hasParam("slider-d", true) &&
        request->hasParam("slider-setpoint", true) &&
        request->hasParam("slider-factor", true) &&
        request->hasParam("slider-enable", true) &&
        request->hasParam("slider-calibration", true) &&
        request->hasParam("slider-servo-min", true) &&
        request->hasParam("slider-servo-max", true) &&
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
      tempParams.calibration =
          request->getParam("slider-calibration", true)->value().toInt();
      tempParams.servoMin =
          request->getParam("slider-servo-min", true)->value().toInt();
      tempParams.servoMax =
          request->getParam("slider-servo-max", true)->value().toInt();
      tempParams.minMeasured = controlParams.minMeasured;
      tempParams.maxMeasured = controlParams.maxMeasured;

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
    // Store received values, send slave params to devices
    if (boardSelector == "master") {
      boardsParams.master = tempParams;
      if (boardsParams.master.servoMin != controlParams.servoMin) {
        elevator.writeMicroseconds(boardsParams.master.servoMin);
        delayWhile(2000);
      } else if (boardsParams.master.servoMax != controlParams.servoMax) {
        elevator.writeMicroseconds(boardsParams.master.servoMax);
        delayWhile(2000);
      }
      if (boardsParams.master.calibration != controlParams.calibration) {
        if (boardsParams.master.calibration == 1) {
          boardsParams.master.minMeasured = 8000;
          boardsParams.master.maxMeasured = 9000;
        } else {
          boardsParams.master.minMeasured = controlParams.minMeasured;
          boardsParams.master.maxMeasured = controlParams.maxMeasured;
        }
      }

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
  if (first) {
    controlParams = boardsParams.master;
    setupWifiFirst();
  } else if (upload) {
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
  elevatorPid.SetOutputLimits(-500, 500);
  Serial.println("PID mode has been set to timer");

  // Set up RMT receiver for reading PWM input value
  setupRMTReceiver();

  // Set up timers for capacitance measurement
  timer = timerBegin(0, 2, true);
  Serial.println("Measurement timer has been set");

  // Set up ticker for measurement
  measurementTicker.start();

  // Set up ticker for the PID control
  pidTicker.start();

  // Set up ticker for the logger
  loggerTicker.start();

  // Set up ticker for RMT for reading PWM value
  rmtTicker.start();

  // Set up ticker for the logger
  Serial.println("Tickers have been started");
  Serial.println("Setup is complete");
}

void loop() {
  // Reset the Watchdog Timer to prevent a system reset
  esp_task_wdt_reset();

  // Handle RMT ring buffer (PWM input)
  rmtTicker.update();

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

    measurementTicker.update();

    if (Serial.available() > 0 && !startTest) {
      // Read the incoming data as a string
      String incomingString = Serial.readStringUntil('\n');

      // Convert the received string to an integer and check the value
      int pwmTemp;
      pwmTemp = incomingString.toInt();
      if (pwmTemp > 990 && pwmTemp < 2010) {
        pwmOut = pwmTemp;
        startTest = true;
        previousMillis = millis();
      }
    }

    elevator.writeMicroseconds(pwmOut);
    currentMillis = millis();

    // Wait for the servo to move before start measurement
    if (currentMillis - previousMillis >= interval && startTest) {
      calculatePosition();
      Serial.print("median:");
      Serial.println(median);
      Serial.print("pwmRead:");
      Serial.println(pwmRead);
      Serial.print("position:");
      Serial.println(position);

      startTest = false;
      pwmOut = 1000;
    }
  }

  // Code that runs only after the device has been configured either as a
  // master or a slave
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
      int midPos;
      int manualPos;
      if (controlParams.servoMax > controlParams.servoMin) {
        midPos = (controlParams.servoMax - controlParams.servoMin) / 2 +
                 controlParams.servoMin;

        manualPos = midPos + control;
        if (manualPos < controlParams.servoMin)
          elevator.writeMicroseconds(controlParams.servoMin);
        else if (manualPos > controlParams.servoMax)
          elevator.writeMicroseconds(controlParams.servoMax);
        else
          elevator.writeMicroseconds(manualPos);
      } else {

        midPos = (controlParams.servoMin - controlParams.servoMax) / 2 +
                 controlParams.servoMax;

        manualPos = midPos - control;
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
