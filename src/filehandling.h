#ifndef FILEHANDLING_H // Inclusion Guards
#define FILEHANDLING_H

#include "SPIFFS.h"
#include <Arduino.h>

void initFS();
String readFile(fs::FS &fs, const char *path);
String readFileJson(fs::FS &fs, const char *path, const char *property);
void writeFile(fs::FS &fs, const char *path, const char *message);
void writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value);

#endif // FILEHANDLING_H