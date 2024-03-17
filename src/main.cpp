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
const int WATCHDOG_TIMEOUT = 8000;

// Variables to save values from HTML form
String slaveString;
String firstString;

// State variables for setting up device
bool slave;
bool first;

// Variables for ESP-NOW
String macString;
String macAddresses[] = {"", "", "", "", ""};
uint8_t broadcastAddress[6];

// File paths to save input values permanently
const char *jsonWifiPath = "/wifi.json";
const char *jsonConfigPath = "/config.json";
const char *jsonAddressesPath = "/addresses.json";

// Setting hostname
const char *hostname = "hydrofoil-control";

// "Watchdog" variable for the filesystem
boolean restart = false;

// Create web server
AsyncWebServer server(80);

// Variables for ESP-NOW
typedef struct dataStruct {
  float p;
  float i;
  float d;
  int setpoint;
  int enable;
  int servoMin;
  int servoMax;
  int target;
} dataStruct;

dataStruct controlParams;
esp_now_peer_info_t peerInfo;

// Variable to get the channel of the AP
const char WIFI_SSID[] = "Hydrofoil-Control";

// RC servo
Servo elevator;

// PID controller
float setpoint, input, output;
int pidRange = 255;
QuickPID elevatorPid(
    &input, &output, &setpoint, controlParams.p, controlParams.i,
    controlParams.d,
    elevatorPid.pMode::pOnError,       /* pOnError, pOnMeas, pOnErrorMeas */
    elevatorPid.dMode::dOnMeas,        /* dOnError, dOnMeas */
    elevatorPid.iAwMode::iAwCondition, /* iAwCondition, iAwClamp, iAwOff */
    elevatorPid.Action::direct);       /* direct, reverse */
int servoPos;

// Cycle time
unsigned long prevTime = 0;
unsigned long currTime = 0;

// PWM Input variables
volatile unsigned long pwmRead = 0;
volatile int pwmValue = 0;
volatile int control = 0;
int factor = 40;
volatile unsigned long pulsInTimeBegin;
volatile unsigned long pulsInTimeEnd;
volatile bool newPulseDurationAvailable = false;

// Capacitance measurement
std::vector<int> rawValues;
hw_timer_t *timer = NULL;
volatile int minMeasured = 1000;
volatile int maxMeasured = 0;
volatile int position = 0;
volatile int median = 0;

// Interruptable delay function
void delayWhile(long delayMillis) {
  long currentTime = 0;
  long startTime = millis();
  while (currentTime < startTime + delayMillis)
    currentTime = millis();
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
  if (controlParams.target == 1) {
    elevator.write(controlParams.servoMin);
    delayWhile(2000);
  } else if (controlParams.target == 2) {
    elevator.write(controlParams.servoMax);
    delayWhile(2000);
  }
  Serial.println(controlParams.target);
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

// Get the channel of the AP
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

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
void sendEspNow() {
  for (int i = 0; i <= 4; i++) {
    // Serial.println("Sending data");
    esp_err_t resultOfSend =
        esp_now_send(0, (uint8_t *)&controlParams, sizeof(dataStruct));
    if (resultOfSend == ESP_OK) {
      Serial.println("Sent successfully");
      break;
    } else if (i == 4) {
      Serial.println("Faild to send data");
      // Serial.println("Result of esp_now_send (Master):");
      // Serial.println(
      //     resultOfSend); // Returns 12393 (0x3069): ESP_ERR_ESPNOW_NOT_FOUND
      //                    // (0x3069): ESPNOW peer is not found
    }
    // else
    // Serial.println("Resending data");
  }
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

  controlParams.p = 2;
  controlParams.i = 4;
  controlParams.d = 1;
  controlParams.setpoint = 127;
  controlParams.enable = 0;
  controlParams.servoMin = 0;
  controlParams.servoMax = 180;
  controlParams.target = 0;

  writeFileJson(SPIFFS, jsonWifiPath, "FIRST", firstString.c_str());
  writeFileJson(SPIFFS, jsonWifiPath, "SLAVE", slaveString.c_str());

  writeFileJson(SPIFFS, jsonConfigPath, "p", String(controlParams.p).c_str());
  writeFileJson(SPIFFS, jsonConfigPath, "i", String(controlParams.i).c_str());
  writeFileJson(SPIFFS, jsonConfigPath, "d", String(controlParams.d).c_str());
  writeFileJson(SPIFFS, jsonConfigPath, "setpoint",
                String(controlParams.setpoint).c_str());
  writeFileJson(SPIFFS, jsonConfigPath, "enable",
                String(controlParams.enable).c_str());
  writeFileJson(SPIFFS, jsonConfigPath, "servoMin",
                String(controlParams.servoMin).c_str());
  writeFileJson(SPIFFS, jsonConfigPath, "servoMax",
                String(controlParams.servoMax).c_str());

  Serial.println("Device has been reset");
}

// Interrupt callback for non-blocking PWM reading
void pwmPinInterrupt() {
  if (digitalRead(pwmPin) == HIGH) {
    pulsInTimeBegin = micros();
  } else {
    pulsInTimeEnd = micros();
    pwmRead = pulsInTimeEnd - pulsInTimeBegin;
    if (pwmRead >= 990 && pwmRead <= 2010) {
      pwmValue = map(pwmRead, 990, 2010, 0, 255);
      control = (pwmValue - 127) / 100.00 * factor;

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
};

// Read charge up time when interrupt triggered
void finishMeasurement() {
  rawValues.push_back(timerRead(timer));
  if (rawValues.size() > 50) {
    rawValues.erase(rawValues.begin());
  }
};

// Log measurement data to serial port
void calculatePosition() {
  if (rawValues.size() == 0) {
    return;
  }

  median = getMedian();
  if (median < minMeasured && median > 500) {
    minMeasured = median;
  }
  if (median > maxMeasured && median < 8000) {
    maxMeasured = median;
  }

  float progress =
      static_cast<float>(median - minMeasured) / (maxMeasured - minMeasured);
  position = pidRange * progress;
  // Check if float conversion is OK!!!!!!!!!!
};

// Log position info to serial port
void logPosition() {
  Serial.print("Latest: ");
  Serial.print(rawValues.back());

  Serial.print("  Min value: ");
  Serial.print(minMeasured);

  Serial.print("  Max value: ");
  Serial.print(maxMeasured);

  Serial.print("  Median value: ");
  Serial.println(median);

  int gaugeValue = map(position, 0, 255, 0, 80);
  Serial.print("[");
  for (int i = 0; i < 80; ++i) {
    if (i < gaugeValue)
      Serial.print("=");
    else if (i == gaugeValue)
      Serial.print(">");
    else
      Serial.print(" ");
  }
  Serial.println("]");
};

// Log PID parameters to serial port
void logPid() {
  Serial.print("input: ");
  Serial.print(input);
  Serial.print("  output: ");
  Serial.print(output);
  Serial.print("  setpoint: ");
  Serial.print(setpoint);
  Serial.print("  kp: ");
  Serial.print(controlParams.p);
  Serial.print("  ki: ");
  Serial.print(controlParams.i);
  Serial.print("  kd: ");
  Serial.print(controlParams.d);
  Serial.print("  PWM in: ");
  Serial.print(pwmValue);
  Serial.print("  servo position: ");
  Serial.print(servoPos);
  Serial.print("  scale min: ");
  Serial.print(minMeasured);
  Serial.print("  scale max: ");
  Serial.print(maxMeasured);
  Serial.print("  actual: ");
  Serial.println(median);
};

// Calculate PID output and move the servo accordingly
void calculatePid() {
  input = position;
  setpoint = controlParams.setpoint + control;
  if (setpoint < 0)
    setpoint = 0;
  else if (setpoint > 255)
    setpoint = 255;
  elevatorPid.Compute();
  servoPos =
      map(output, 0, 255, controlParams.servoMin, controlParams.servoMax);
  elevator.write(servoPos);
};

TickTwo measurementTicker([]() { startMeasurement(); }, 5, 0, MILLIS);
TickTwo positionTicker([]() { calculatePosition(); }, 5, 0, MILLIS);
TickTwo pidTicker([]() { calculatePid(); }, 5, 0, MILLIS);
TickTwo loggerTicker(
    []() {
      // logPosition();
      logPid();
    },
    500, 0, MILLIS);

// Set up wifi and webserver for first device start
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

  // Send MAC address to client
  server.on("/get-mac", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", WiFi.macAddress().c_str());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress().c_str());
  });

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

// Set up wifi, webserver and ESP-NOW for master device
void setupWifiMaster() {
  Serial.println("Setting AP (Access Point)");
  // NULL sets an open Access Point
  WiFi.softAP("Hydrofoil-Control", NULL);

  IPAddress IP = WiFi.softAPIP();
  WiFi.mode(WIFI_AP_STA);
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

    // Variables to send
    jsonDoc["slider-p"] = controlParams.p;
    jsonDoc["slider-i"] = controlParams.i;
    jsonDoc["slider-d"] = controlParams.d;
    jsonDoc["slider-setpoint"] = controlParams.setpoint;
    jsonDoc["slider-enable"] = controlParams.enable;
    jsonDoc["slider-servo-min"] = controlParams.servoMin;
    jsonDoc["slider-servo-max"] = controlParams.servoMax;

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
    jsonDoc["broadcastAddress2"] = macAddresses[2];
    jsonDoc["broadcastAddress3"] = macAddresses[3];
    jsonDoc["broadcastAddress4"] = macAddresses[4];

    String response;
    serializeJson(jsonDoc, response);

    request->send(200, "application/json", response);
    Serial.println("MAC addresses have been sent");
  });

  // Send process values to client
  server.on("/get-values", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "bar");
    Serial.println("bar");
  });

  server.on("/set-sliders", HTTP_POST, [](AsyncWebServerRequest *request) {
    String target;
    // Check if all required parameters are present
    if (request->hasParam("slider-p", true) &&
        request->hasParam("slider-i", true) &&
        request->hasParam("slider-d", true) &&
        request->hasParam("slider-setpoint", true) &&
        request->hasParam("slider-enable", true) &&
        request->hasParam("slider-servo-min", true) &&
        request->hasParam("slider-servo-max", true) &&
        request->hasParam("target", true)) {

      // Extract parameters
      controlParams.p = request->getParam("slider-p", true)->value().toFloat();
      controlParams.i = request->getParam("slider-i", true)->value().toFloat();
      controlParams.d = request->getParam("slider-d", true)->value().toFloat();
      controlParams.setpoint =
          request->getParam("slider-setpoint", true)->value().toInt();
      controlParams.enable =
          request->getParam("slider-enable", true)->value().toInt();
      controlParams.servoMin =
          request->getParam("slider-servo-min", true)->value().toInt();
      controlParams.servoMax =
          request->getParam("slider-servo-max", true)->value().toInt();
      target = request->getParam("target", true)->value();

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }

    if (target == "slider-servo-min") {
      controlParams.target = 1;
      elevator.write(controlParams.servoMin);
      delayWhile(2000);
    } else if (target == "slider-servo-max") {
      controlParams.target = 2;
      elevator.write(controlParams.servoMax);
      delayWhile(2000);
    }

    // Serial.print("P value: ");
    // Serial.println(controlParams.p);
    // Serial.print("I value: ");
    // Serial.println(controlParams.i);
    // Serial.print("D value: ");
    // Serial.println(controlParams.d);
    // Serial.print("Setpoint value: ");
    // Serial.println(controlParams.setpoint);

    writeFileJson(SPIFFS, jsonConfigPath, "p", String(controlParams.p).c_str());
    writeFileJson(SPIFFS, jsonConfigPath, "i", String(controlParams.i).c_str());
    writeFileJson(SPIFFS, jsonConfigPath, "d", String(controlParams.d).c_str());
    writeFileJson(SPIFFS, jsonConfigPath, "setpoint",
                  String(controlParams.setpoint).c_str());
    writeFileJson(SPIFFS, jsonConfigPath, "enable",
                  String(controlParams.enable).c_str());
    writeFileJson(SPIFFS, jsonConfigPath, "servoMin",
                  String(controlParams.servoMin).c_str());
    writeFileJson(SPIFFS, jsonConfigPath, "servoMax",
                  String(controlParams.servoMax).c_str());

    sendEspNow();
    elevatorPid.SetTunings(controlParams.p, controlParams.i, controlParams.d);
  });

  server.on("/add-mac", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Check if all required parameters are present
    if (request->hasParam("mac", true)) {
      // Extract parameters
      String macString = request->getParam("mac", true)->value();

      Serial.println("MAC Address from POST request");
      Serial.println(macString);

      bool existing = false;
      // Find empty space in the array, store the value then add new peer
      for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
        if (macString == macAddresses[i])
          existing = true;
        if (macAddresses[i] == "" && !existing) {
          macAddresses[i] = macString;
          stringToMac(macAddresses[i], broadcastAddress);
          break;
        } else if (existing) {
          Serial.println("Address already added");
          Serial.println(macAddresses[i]);
          break;
        }
      }

      // Store value in the JSON file, add new ESP NOW peer
      if (!existing) {
        writeArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses,
                       sizeof(macAddresses) / sizeof(macAddresses[0]));
        Serial.println("New MAC address stored");

        addNewPeerEspNow();
      }

      // Send success response
      request->send(200, "text/plain", "OK");
    } else {
      // Send error response
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

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

  Serial.println("Master has started");
};

// Set up wifi and ESP-NOW for slave device
void setupWifiSlave() {
  // Init ESP-NOW
  Serial.println("Start slave initialization");
  WiFi.mode(WIFI_STA);

  // Get the channel of the Master AP and set Slave's channel accordingly
  int32_t channel = getWiFiChannel(WIFI_SSID);
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
  slaveString = readFileJson(SPIFFS, jsonWifiPath, "SLAVE");
  slave = stringToBool(slaveString);
  firstString = readFileJson(SPIFFS, jsonWifiPath, "FIRST");
  first = stringToBool(firstString);

  controlParams.p = readFileJson(SPIFFS, jsonConfigPath, "p").toFloat();
  controlParams.i = readFileJson(SPIFFS, jsonConfigPath, "i").toFloat();
  controlParams.d = readFileJson(SPIFFS, jsonConfigPath, "d").toFloat();
  controlParams.setpoint =
      readFileJson(SPIFFS, jsonConfigPath, "setpoint").toInt();
  controlParams.enable = readFileJson(SPIFFS, jsonConfigPath, "enable").toInt();
  controlParams.servoMin =
      readFileJson(SPIFFS, jsonConfigPath, "servoMin").toInt();
  controlParams.servoMax =
      readFileJson(SPIFFS, jsonConfigPath, "servoMax").toInt();
  controlParams.target = 0;

  // Read MAC Addresses from JSON and store to macAddresses array
  readArrayJson(SPIFFS, jsonAddressesPath, "addresses", macAddresses);
  Serial.println("MAC addresses from SPIFFS:");
  for (int i = 0; i < sizeof(macAddresses) / sizeof(macAddresses[0]); i++) {
    if (macAddresses[i] != "")
      Serial.println(macAddresses[i]);
  }

  // Configure devices according to first and slave variables
  if (first)
    setupWifiFirst();
  else if (!slave) {
    setupWifiMaster();
  } else
    setupWifiSlave();

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
  elevatorPid.SetMode(elevatorPid.Control::automatic);
  Serial.println("PID mode has been set to automatic");

  // Interrupt for non-blocking PWM reading
  attachInterrupt(digitalPinToInterrupt(pwmPin), pwmPinInterrupt, CHANGE);

  // Interrupt for capacitance measurement
  attachInterrupt(digitalPinToInterrupt(capacitancePin), finishMeasurement,
                  RISING);
  Serial.println("Interrupts have been attached");

  // Set up timers for capacitance measurement
  timer = timerBegin(0, 2, true);
  Serial.println("Measurement timer has been set");
  measurementTicker.start();

  // Set up timer for getting position values
  positionTicker.start();

  // Set up ticker for the PID control
  pidTicker.start();

  // Set up ticker for the logger
  loggerTicker.start();
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
    delayWhile(3000);
    if (digitalRead(resetPin) == LOW) {
      // Reset MAC Addresses only after 3s
      resetMacAddresses();
      digitalWrite(ledPin, HIGH);
      delayWhile(200);
      digitalWrite(ledPin, LOW);
      delayWhile(200);
      digitalWrite(ledPin, HIGH);
      delayWhile(200);
      digitalWrite(ledPin, LOW);
      delayWhile(2400);
      if (digitalRead(resetPin) == LOW) {
        // Reset to default after 6s
        digitalWrite(ledPin, HIGH);
        delayWhile(200);
        digitalWrite(ledPin, LOW);
        delayWhile(200);
        digitalWrite(ledPin, HIGH);
        delayWhile(1000);
        resetDevice();
        ESP.restart();
      } else
        ESP.restart();
    }
  }
  // Code that runs only after the device has been configured as master or slave
  if (!first) {
    measurementTicker.update();
    delayWhile(1);
    positionTicker.update();
    if (controlParams.enable == 1)
      pidTicker.update();
    else {
      int midPos = (controlParams.servoMax - controlParams.servoMin) / 2 +
                   controlParams.servoMin;
      if (midPos >= 0 && midPos <= 180)
        elevator.write(midPos);
    }
    loggerTicker.update();

    // Master's main loop
    if (!slave) {
      analogWrite(ledPin, output);

      // Slave's main loop
    } else {
      analogWrite(ledPin, output);
    }
  }
}
