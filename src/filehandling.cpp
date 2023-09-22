#include "SPIFFS.h"
#include "filehandling.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// Variable for JSON document
StaticJsonDocument<100> jsonDoc;

// Initialize SPIFFS
void initFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return String();
  }

  String fileContent;
  while (file.available()) {
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}

// Read JSON File from SPIFFS
String readFileJson(fs::FS &fs, const char *path, const char *property) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return String();
  }

  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);

  file.readBytes(buf.get(), size);
  deserializeJson(jsonDoc, buf.get());

  String value = jsonDoc[property];
  return value;
}

// Read JSON File Array from SPIFFS
void readArrayJson(fs::FS &fs, const char *path, const char *property,
                   String *array) {
  Serial.printf("Reading array from file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
  }

  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);

  file.readBytes(buf.get(), size);
  deserializeJson(jsonDoc, buf.get());

  JsonArray macArray = jsonDoc[property];
  for (int i = 0; i < macArray.size(); i++) {
    String value = macArray[i].as<String>();
    array[i] = value;
    Serial.println(value);
  }
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
}

// Write JSON file to SPIFFS
void writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }

  // Creating JSON record
  jsonDoc[property] = value;
  // Writing data to JSON file
  serializeJson(jsonDoc, file);
}

// Write JSON file Array to SPIFFS
void writeArrayJson(fs::FS &fs, const char *path, const char *property,
                    String *array) {
  Serial.printf("Writing array to file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }

  JsonArray arrayToJson = jsonDoc.createNestedArray(property);
  for (int i = 0; i < sizeof(array); i++) {
    String value = array[i];
    arrayToJson.add(value);
    Serial.println(value);
  }
  // Writing data to JSON file
  serializeJson(jsonDoc, file);
}