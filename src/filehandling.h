#ifndef FILEHANDLING_H
#define FILEHANDLING_H

#include "SPIFFS.h"
#include <Arduino.h>

void initFS();
String readFileJson(fs::FS &fs, const char *path, const char *property);
void readArrayJson(fs::FS &fs, const char *path, const char *property,
                   String *array);

void writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value);
void writeArrayJson(fs::FS &fs, const char *path, const char *property,
                    String *array, int size);

#endif // FILEHANDLING_H