#ifndef FILEHANDLING_H // Inclusion Guards
#define FILEHANDLING_H

#include "SPIFFS.h"
#include <Arduino.h>

void initFS();
String readFile(fs::FS &fs, const char *path);
String readFileJson(fs::FS &fs, const char *path, const char *property);
void readArrayJson(fs::FS &fs, const char *path, const char *property,
                   String *array);

void writeFile(fs::FS &fs, const char *path, const char *message);
void writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value);
void writeArrayJson(fs::FS &fs, const char *path, const char *property,
                    String *array);

#endif // FILEHANDLING_H