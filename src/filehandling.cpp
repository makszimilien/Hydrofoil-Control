#include "filehandling.h"
#include <ArduinoJson.h>
#include <cmath>
#include <cstring>
#include <memory>
#include <new>

namespace {
constexpr size_t kMaxFileBytes = 4096;
constexpr size_t kJsonCapacity = 2048;
constexpr size_t kMaxAddresses = 2;

bool readDocumentAt(fs::FS &fs, const char *path, JsonDocument &doc) {
  doc.clear();
  File file = fs.open(path, FILE_READ);
  if (!file || file.isDirectory()) return false;
  const size_t size = file.size();
  if (!size || size > kMaxFileBytes) {
    file.close();
    return false;
  }
  std::unique_ptr<char[]> bytes(new (std::nothrow) char[size]);
  if (!bytes) {
    file.close();
    return false;
  }
  const size_t received = file.readBytes(bytes.get(), size);
  file.close();
  if (received != size) return false;
  // Const input makes ArduinoJson own its strings after bytes is released.
  const auto error = deserializeJson(
      doc, static_cast<const char *>(bytes.get()), size);
  return !error && !doc.overflowed() && doc.is<JsonObject>();
}

bool readDocument(fs::FS &fs, const char *path, JsonDocument &doc) {
  if (!path || !*path) return false;
  if (fs.exists(path)) return readDocumentAt(fs, path, doc);
  // A power interruption between the two renames leaves the old file here.
  const String backup = String(path) + ".bak";
  return readDocumentAt(fs, backup.c_str(), doc);
}

bool fileEquals(fs::FS &fs, const char *path, const String &expected) {
  File file = fs.open(path, FILE_READ);
  if (!file || file.isDirectory() || file.size() != expected.length()) {
    file.close();
    return false;
  }
  char chunk[128];
  size_t offset = 0;
  while (offset < expected.length()) {
    const size_t remaining = expected.length() - offset;
    const size_t count = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
    if (file.readBytes(chunk, count) != count ||
        std::memcmp(chunk, expected.c_str() + offset, count) != 0) {
      file.close();
      return false;
    }
    offset += count;
  }
  file.close();
  return true;
}

bool writeDocument(fs::FS &fs, const char *path, const JsonDocument &doc) {
  if (!path || !*path || doc.overflowed()) return false;
  const size_t length = measureJson(doc);
  if (!length || length > kMaxFileBytes) return false;
  String body;
  if (!body.reserve(length + 1) || serializeJson(doc, body) != length ||
      body.length() != length) return false;

  const String temporary = String(path) + ".tmp";
  const String backup = String(path) + ".bak";
  File file = fs.open(temporary.c_str(), FILE_WRITE);
  if (!file || file.isDirectory()) return false;
  const size_t written = file.write(
      reinterpret_cast<const uint8_t *>(body.c_str()), length);
  file.flush();
  file.close();
  if (written != length || !fileEquals(fs, temporary.c_str(), body)) {
    fs.remove(temporary.c_str());
    return false;
  }

  // SPIFFS does not replace a destination during rename. Keep the original
  // recoverable until the verified replacement has been installed. This is
  // staged replacement with rollback, not a guarantee against flash damage.
  const bool hadOriginal = fs.exists(path);
  if (hadOriginal) {
    if (fs.exists(backup.c_str()) && !fs.remove(backup.c_str())) return false;
    if (!fs.rename(path, backup.c_str())) return false;
  }
  if (!fs.rename(temporary.c_str(), path)) {
    if (hadOriginal) fs.rename(backup.c_str(), path);
    return false;
  }
  // Retain the last complete file as .bak. Readers use it only when the
  // primary name is absent, never to silently hide malformed current data.
  return true;
}

bool finiteFloat(JsonVariantConst value, float &out) {
  if (!value.is<float>()) return false;
  const float candidate = value.as<float>();
  if (!std::isfinite(candidate)) return false;
  out = candidate;
  return true;
}

bool integer(JsonVariantConst value, int &out) {
  if (!value.is<int>()) return false;
  out = value.as<int>();
  return true;
}

bool decodeBoard(JsonObjectConst obj, dataStruct &board) {
  if (obj.isNull()) return false;
  return finiteFloat(obj["p"], board.p) &&
      finiteFloat(obj["i"], board.i) &&
      finiteFloat(obj["d"], board.d) &&
      integer(obj["setpoint"], board.setpoint) &&
      integer(obj["enable"], board.enable) &&
      integer(obj["servoMin"], board.servoMin) &&
      integer(obj["servoMax"], board.servoMax) &&
      integer(obj["factor"], board.factor) &&
      integer(obj["calibration"], board.calibration) &&
      integer(obj["minMeasured"], board.minMeasured) &&
      integer(obj["maxMeasured"], board.maxMeasured);
}

void encodeBoard(JsonObject obj, const dataStruct &board) {
  obj["p"] = board.p;
  obj["i"] = board.i;
  obj["d"] = board.d;
  obj["setpoint"] = board.setpoint;
  obj["enable"] = board.enable;
  obj["servoMin"] = board.servoMin;
  obj["servoMax"] = board.servoMax;
  obj["factor"] = board.factor;
  obj["calibration"] = board.calibration;
  obj["minMeasured"] = board.minMeasured;
  obj["maxMeasured"] = board.maxMeasured;
}

int hexDigit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}

bool decodeAddress(const String &address, uint8_t (&bytes)[6]) {
  if (address.length() != 17) return false;
  const char separator = address[2];
  if (separator != ':' && separator != '-') return false;
  bool anyNonzero = false;
  for (size_t i = 0; i < 6; ++i) {
    const int hi = hexDigit(address[i * 3]);
    const int lo = hexDigit(address[i * 3 + 1]);
    if (hi < 0 || lo < 0 || (i < 5 && address[i * 3 + 2] != separator))
      return false;
    bytes[i] = static_cast<uint8_t>(hi * 16 + lo);
    anyNonzero = anyNonzero || bytes[i] != 0;
  }
  // ESP-NOW configuration expects a specific unicast peer, not a group.
  return anyNonzero && (bytes[0] & 1) == 0;
}

bool validAddresses(const String *addresses, size_t size) {
  if (!addresses || size > kMaxAddresses) return false;
  uint8_t decoded[kMaxAddresses][6] = {};
  for (size_t i = 0; i < size; ++i) {
    if (!addresses[i].length()) continue;
    if (!decodeAddress(addresses[i], decoded[i])) return false;
    for (size_t j = 0; j < i; ++j)
      if (addresses[j].length() && std::memcmp(decoded[i], decoded[j], 6) == 0)
        return false;
  }
  return true;
}
} // namespace

bool initFS() {
  return SPIFFS.begin(false); // Never erase user settings on a mount error.
}

String readFileJson(fs::FS &fs, const char *path, const char *property) {
  if (!property) return String();
  DynamicJsonDocument doc(kJsonCapacity);
  if (!readDocument(fs, path, doc) || !doc[property].is<const char *>())
    return String();
  return doc[property].as<String>();
}

bool readArrayJson(fs::FS &fs, const char *path, const char *property,
                   String *array, size_t capacity) {
  if (!property || !array || capacity > kMaxAddresses) return false;
  DynamicJsonDocument doc(kJsonCapacity);
  if (!readDocument(fs, path, doc) || !doc[property].is<JsonArray>()) return false;
  const JsonArrayConst values = doc[property].as<JsonArrayConst>();
  if (values.size() > capacity) return false;
  String candidate[kMaxAddresses];
  for (size_t i = 0; i < values.size(); ++i) {
    if (!values[i].is<const char *>()) return false;
    candidate[i] = values[i].as<String>();
  }
  if (!validAddresses(candidate, values.size())) return false;
  for (size_t i = 0; i < capacity; ++i) array[i] = candidate[i];
  return true;
}

bool readStructJson(fs::FS &fs, const char *path, allBoards &boards) {
  DynamicJsonDocument doc(kJsonCapacity);
  if (!readDocument(fs, path, doc)) return false;
  allBoards candidate{};
  if (!decodeBoard(doc["master"].as<JsonObjectConst>(), candidate.master) ||
      !decodeBoard(doc["slave1"].as<JsonObjectConst>(), candidate.slave1) ||
      !decodeBoard(doc["slave2"].as<JsonObjectConst>(), candidate.slave2)) return false;
  boards = candidate; // Do not publish partially parsed or defaulted settings.
  return true;
}

bool writeFileJson(fs::FS &fs, const char *path, const char *property,
                   const char *value) {
  if (!property || !value) return false;
  DynamicJsonDocument doc(kJsonCapacity);
  if (!readDocument(fs, path, doc)) return false;
  // Preserve all other properties, including the locally stored credentials.
  doc[property] = value;
  return writeDocument(fs, path, doc);
}

bool writeArrayJson(fs::FS &fs, const char *path, const char *property,
                    String *array, int size) {
  if (!property || size < 0 || !validAddresses(array, static_cast<size_t>(size)))
    return false;
  DynamicJsonDocument doc(kJsonCapacity);
  JsonArray values = doc.createNestedArray(property);
  for (int i = 0; i < size; ++i) values.add(array[i]);
  return writeDocument(fs, path, doc);
}

bool writeStructJson(fs::FS &fs, const char *path, allBoards &boards) {
  // Each board contains eleven fields. The original 9-field allocation
  // silently dropped the last board's calibration and servo settings.
  const size_t capacity = JSON_OBJECT_SIZE(3) + 3 * JSON_OBJECT_SIZE(11);
  DynamicJsonDocument doc(capacity);
  encodeBoard(doc.createNestedObject("master"), boards.master);
  encodeBoard(doc.createNestedObject("slave1"), boards.slave1);
  encodeBoard(doc.createNestedObject("slave2"), boards.slave2);
  allBoards check{};
  if (!decodeBoard(doc["master"].as<JsonObjectConst>(), check.master) ||
      !decodeBoard(doc["slave1"].as<JsonObjectConst>(), check.slave1) ||
      !decodeBoard(doc["slave2"].as<JsonObjectConst>(), check.slave2)) return false;
  return writeDocument(fs, path, doc);
}
