#ifndef FILEHANDLING_H
#define FILEHANDLING_H

#include "SPIFFS.h"
#include <Arduino.h>

// Variables for managing device settings and data flow
typedef struct dataStruct {
  float p;
  float i;
  float d;
  int setpoint;
  int enable;
  int calibration;
  int servoMin;
  int servoMax;
  int servoTarget;
  int factor;
} dataStruct;

typedef struct allBoards {
  dataStruct master;
  dataStruct slave1;
  dataStruct slave2;
} allBoards;

void initFS();
String readFileJson(fs::FS &fs, const char *path, const char *property);
void readArrayJson(fs::FS &fs, const char *path, const char *property,
                   String *array);
bool readStructJson(fs::FS &fs, const char *path, allBoards &boards);

void writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value);
void writeArrayJson(fs::FS &fs, const char *path, const char *property,
                    String *array, int size);
bool writeStructJson(fs::FS &fs, const char *path, allBoards &boards);

#endif // FILEHANDLING_H