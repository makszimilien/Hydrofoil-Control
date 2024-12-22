#include "SPIFFS.h"
#include "filehandling.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// Variable for JSON document
DynamicJsonDocument jsonDoc(1000);

// Initialize SPIFFS
void initFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
}

// Read JSON File from SPIFFS
String readFileJson(fs::FS &fs, const char *path, const char *property) {
  // Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return String();
  }

  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);

  file.readBytes(buf.get(), size);
  deserializeJson(jsonDoc, buf.get());
  file.close();

  String value = jsonDoc[property];
  return value;
}

// Read JSON File Array from SPIFFS
void readArrayJson(fs::FS &fs, const char *path, const char *property,
                   String *array) {
  // Serial.printf("Reading array from file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
  }

  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);

  file.readBytes(buf.get(), size);
  deserializeJson(jsonDoc, buf.get());
  file.close();
  // JSON array can be longer than string array, fix later !!
  JsonArray macArray = jsonDoc[property];
  // Serial.println("Current element of the array:");
  for (int i = 0; i < macArray.size(); i++) {
    String value = macArray[i].as<String>();
    array[i] = value;
    if (value != "") {
      // Serial.println(value);
    }
  }
}

// Read JSON File Struct from SPIFFS
bool readStructJson(fs::FS &fs, const char *path, allBoards &boards) {
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
  }

  // Allocate a buffer to store contents of the file
  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);

  // Read file to the buffer
  file.readBytes(buf.get(), size);

  // Parse JSON
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buf.get());

  if (error) {
    Serial.println("Failed to parse JSON");
    file.close();
    return false;
  }

  // Read master data
  JsonObject master = doc["master"];
  boards.master.p = master["p"];
  boards.master.i = master["i"];
  boards.master.d = master["d"];
  boards.master.setpoint = master["setpoint"];
  boards.master.enable = master["enable"];
  boards.master.servoMin = master["servoMin"];
  boards.master.servoMax = master["servoMax"];
  boards.master.factor = master["factor"];

  // Read slave1 data
  JsonObject slave1 = doc["slave1"];
  boards.slave1.p = slave1["p"];
  boards.slave1.i = slave1["i"];
  boards.slave1.d = slave1["d"];
  boards.slave1.setpoint = slave1["setpoint"];
  boards.slave1.enable = slave1["enable"];
  boards.slave1.servoMin = slave1["servoMin"];
  boards.slave1.servoMax = slave1["servoMax"];
  boards.slave1.factor = slave1["factor"];

  // Read slave2 data
  JsonObject slave2 = doc["slave2"];
  boards.slave2.p = slave2["p"];
  boards.slave2.i = slave2["i"];
  boards.slave2.d = slave2["d"];
  boards.slave2.setpoint = slave2["setpoint"];
  boards.slave2.enable = slave2["enable"];
  boards.slave2.servoMin = slave2["servoMin"];
  boards.slave2.servoMax = slave2["servoMax"];
  boards.slave2.factor = slave2["factor"];

  file.close();
  return true;
};

// Write JSON file to SPIFFS
void writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value) {
  // Serial.printf("Writing file: %s\r\n", path);

  // Read current content
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }

  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);

  file.readBytes(buf.get(), size);
  deserializeJson(jsonDoc, buf.get());
  file.close();

  // Expand current content and write to file
  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }

  // Creating JSON record
  jsonDoc[property] = value;
  // Writing data to JSON file
  serializeJson(jsonDoc, file);
  file.close();
}

// Write JSON file Array to SPIFFS
void writeArrayJson(fs::FS &fs, const char *path, const char *property,
                    String *array, int size) {
  // Serial.printf("Writing array to file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }

  JsonArray arrayToJson = jsonDoc.createNestedArray(property);
  for (int i = 0; i < size; i++) {
    String value = array[i];
    arrayToJson.add(value);
    // if (value != "")
    // Serial.println(value);
  }
  // Writing data to JSON file
  serializeJson(jsonDoc, file);
  file.close();
}

bool writeStructJson(fs::FS &fs, const char *path, allBoards &boards) {
  // Create a JSON document
  const size_t capacity = JSON_OBJECT_SIZE(3) + 3 * JSON_OBJECT_SIZE(9);
  DynamicJsonDocument doc(capacity);

  // Add master data
  JsonObject master = doc.createNestedObject("master");
  master["p"] = boards.master.p;
  master["i"] = boards.master.i;
  master["d"] = boards.master.d;
  master["setpoint"] = boards.master.setpoint;
  master["enable"] = boards.master.enable;
  master["servoMin"] = boards.master.servoMin;
  master["servoMax"] = boards.master.servoMax;
  master["factor"] = boards.master.factor;

  // Add slave1 data
  JsonObject slave1 = doc.createNestedObject("slave1");
  slave1["p"] = boards.slave1.p;
  slave1["i"] = boards.slave1.i;
  slave1["d"] = boards.slave1.d;
  slave1["setpoint"] = boards.slave1.setpoint;
  slave1["enable"] = boards.slave1.enable;
  slave1["servoMin"] = boards.slave1.servoMin;
  slave1["servoMax"] = boards.slave1.servoMax;
  slave1["factor"] = boards.slave1.factor;

  // Add slave2 data
  JsonObject slave2 = doc.createNestedObject("slave2");
  slave2["p"] = boards.slave2.p;
  slave2["i"] = boards.slave2.i;
  slave2["d"] = boards.slave2.d;
  slave2["setpoint"] = boards.slave2.setpoint;
  slave2["enable"] = boards.slave2.enable;
  slave2["servoMin"] = boards.slave2.servoMin;
  slave2["servoMax"] = boards.slave2.servoMax;
  slave2["factor"] = boards.slave2.factor;

  // Serialize JSON to string
  String jsonString;
  serializeJson(doc, jsonString);

  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return false;
  }

  // Write to file
  if (file.print(jsonString)) {
    Serial.println("File written successfully");
  } else {
    Serial.println("Write failed");
  }

  // Close the file
  file.close();
  return true;
};