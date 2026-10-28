#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <vector>
#include "bench_recording.h"
#include "bench_page.h"
void enterBenchMode();
void addControllerDiagnostics(JsonObject);
extern bool otaInProgress;
extern bool benchActive;
extern bool simulateSensorTimeout, sensorHealthy, positionValid;
extern uint32_t sensorTimeouts, sensorGoodSamples, sensorLastTicks;
extern std::vector<int> rawValues;
extern hw_timer_t *timer;
extern Servo elevator;
void startMeasurement();
bool calibrationSampling();
constexpr int capacitancePin = 6;
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>

// A temporary bench-only entry point. Settings/UI files are preserved in SPIFFS;
// the controller is intentionally inactive until a later reviewed application.
WebServer benchServer(80);
bool benchFilesystem = false;
bool benchAutomatic = false;
bool benchSingle = false;
uint32_t benchLastSample = 0;
String benchHostname;
bench_recording::Recorder benchRecording;
uint32_t benchRecordingId = 0;
String benchRecordingError;

const char *recordingReason(bench_recording::FinishReason reason) {
  using bench_recording::FinishReason;
  switch (reason) {
    case FinishReason::Completed: return "completed";
    case FinishReason::Stopped: return "stopped";
    case FinishReason::Replaced: return "replaced";
    case FinishReason::Ota: return "ota";
    case FinishReason::Full: return "full";
    default: return "none";
  }
}

void addRecordingStatistics(JsonObject obj,
                            const bench_recording::Statistics &stats) {
  obj["count"] = stats.count;
  if (!stats.count) {
    for (const char *key : {"min", "max", "span", "mean", "stddev"})
      obj[key] = nullptr;
    return;
  }
  obj["min"] = stats.min;
  obj["max"] = stats.max;
  obj["span"] = stats.max - stats.min;
  obj["mean"] = stats.mean;
  obj["stddev"] = stats.populationStddev();
}

void addRecordingDiagnostics(JsonObject obj) {
  obj["session_id"] = benchRecordingId;
  obj["state"] = !benchRecording.hasSession() ? "idle" :
      (benchRecording.active() ? "recording" : "finished");
  obj["reason"] = recordingReason(benchRecording.reason());
  obj["error"] = benchRecordingError;
  obj["duration_ms"] = bench_recording::Recorder::kDurationMs;
  obj["elapsed_ms"] = benchRecording.elapsedMs(millis());
  obj["samples"] = benchRecording.size();
  obj["good"] = benchRecording.goodCount();
  obj["timeouts"] = benchRecording.timeoutCount();
  obj["median_window"] = bench_recording::Recorder::kWindowSize;
  addRecordingStatistics(obj.createNestedObject("raw"), benchRecording.rawStats());
  addRecordingStatistics(obj.createNestedObject("filtered"), benchRecording.medianStats());
}

void stopBenchSamples(bench_recording::FinishReason reason =
                          bench_recording::FinishReason::Stopped) {
  if (benchRecording.active())
    benchRecording.finish(millis(), reason);
  benchAutomatic = false;
  benchSingle = false;
  simulateSensorTimeout = false;
  sensorHealthy = false;
  positionValid = false;
  rawValues.clear();
}

void serviceBenchSamples() {
  const uint32_t now = millis();
  if (benchRecording.dueToFinish(now))
    stopBenchSamples(bench_recording::FinishReason::Completed);
  if (!benchSingle &&
      ((!benchAutomatic && !calibrationSampling()) || uint32_t(now - benchLastSample) < 10))
    return;
  benchSingle = false;
  benchLastSample = now;
  const uint32_t previousGood = sensorGoodSamples;
  const uint32_t previousTimeouts = sensorTimeouts;
  startMeasurement();
  // Recording observes the existing bounded measurement; no PID/servo calls.
  if (benchRecording.active()) {
    if (sensorGoodSamples != previousGood)
      benchRecording.add(now, sensorLastTicks, true);
    else if (sensorTimeouts != previousTimeouts)
      benchRecording.add(now, sensorLastTicks, false);
    else {
      // An unavailable timer leaves a stale raw value but no new acquisition.
      benchRecordingError = "Sensor acquisition unavailable; recording stopped.";
      stopBenchSamples();
    }
    if (!benchRecording.active())
      stopBenchSamples();
    else if (benchRecording.dueToFinish(millis()))
      stopBenchSamples(bench_recording::FinishReason::Completed);
  }
}

void sendRecordingCsv() {
  if (!benchActive) {
    benchServer.send(409, "text/plain", "Pause normal control before downloading a recording.");
    return;
  }
  if (!benchRecording.hasSession()) {
    benchServer.send(404, "text/plain", "No recording yet.");
    return;
  }
  if (benchRecording.active()) {
    benchServer.send(409, "text/plain", "Recording is running. Download after it stops.");
    return;
  }
  if (benchServer.hasArg("session_id") &&
      benchServer.arg("session_id") != String(benchRecordingId)) {
    benchServer.send(409, "text/plain", "A different recording has replaced this one.");
    return;
  }
  benchServer.sendHeader("Cache-Control", "no-store");
  benchServer.sendHeader("Content-Disposition", "attachment; filename=hydrofoil-recording-" +
                         String(benchRecordingId) + ".csv");
  benchServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  benchServer.send(200, "text/csv", "");
  String chunk;
  chunk.reserve(1200);
  chunk = "elapsed_ms,raw_ticks,valid,median20_ticks\n";
  for (size_t i = 0; i < benchRecording.size(); ++i) {
    const auto &entry = *benchRecording.entry(i);
    chunk += String(entry.elapsed_ms) + "," + String(entry.raw_ticks) + ",";
    chunk += (entry.flags & bench_recording::kRawValid) ? "1," : "0,";
    if (entry.flags & bench_recording::kMedianValid)
      chunk += String(entry.median20_ticks);
    chunk += "\n";
    if (chunk.length() >= 1000) {
      benchServer.sendContent(chunk);
      chunk = "";
      delay(0);
    }
  }
  if (chunk.length())
    benchServer.sendContent(chunk);
  benchServer.sendContent("");
}

// Collected before HTTP/OTA service starts, with sensing disabled. Never
// refreshed during sampling. A failed allocation produces an HTTP error.
DynamicJsonDocument benchStaticDiagnostics(4096);
bool benchStaticCollected = false;

void collectBenchStaticDiagnostics() {
  if (benchStaticCollected) return;
  benchStaticCollected = true;
  auto &doc = benchStaticDiagnostics;
  doc["build"] = "HF controller 2026-10-28 r5.5";
  doc["reset_reason"] = int(esp_reset_reason());
  doc["mac"] = WiFi.softAPmacAddress();
  doc["chip"] = ESP.getChipModel();
  doc["chip_revision"] = ESP.getChipRevision();
  doc["flash_bytes"] = ESP.getFlashChipSize();
  const uint32_t sketchStarted = micros();
  doc["app_bytes"] = ESP.getSketchSize();
  doc["startup_sketch_size_us"] = uint32_t(micros() - sketchStarted);
  doc["app_md5"] = ESP.getSketchMD5();
  doc["ota_port"] = 3232;
  const esp_partition_t *running = esp_ota_get_running_partition();
  const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
  doc["running_partition"] = running ? running->label : "unavailable";
  doc["next_ota_bytes"] = next ? next->size : 0;
  JsonArray partitions = doc.createNestedArray("partitions");
  esp_partition_iterator_t it = esp_partition_find(
      ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  while (it) {
    const esp_partition_t *p = esp_partition_get(it);
    JsonObject entry = partitions.createNestedObject();
    entry["label"] = p->label;
    entry["address"] = p->address;
    entry["bytes"] = p->size;
    entry["type"] = int(p->type);
    entry["subtype"] = int(p->subtype);
    it = esp_partition_next(it);
  }
}

void sendBenchDiagnostics() {
  DynamicJsonDocument doc(6144);
  if (benchStaticDiagnostics.overflowed() || !benchStaticCollected) {
    benchServer.send(500, "text/plain", "Static diagnostics unavailable.");
    return;
  }
  doc.set(benchStaticDiagnostics.as<JsonObjectConst>());
  doc["uptime_ms"] = millis();
  doc["ip"] = WiFi.softAPIP().toString();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["filesystem_mounted"] = benchFilesystem;
  doc["automatic_sensing"] = benchAutomatic;
  doc["simulated_timeout"] = simulateSensorTimeout;
  addControllerDiagnostics(doc.as<JsonObject>());
  addRecordingDiagnostics(doc.createNestedObject("recording"));
  if (doc.overflowed()) {
    benchServer.send(500, "text/plain", "Diagnostics memory limit reached.");
    return;
  }
  String body;
  serializeJson(doc, body);
  benchServer.sendHeader("Cache-Control", "no-store");
  benchServer.send(200, "application/json", body);
}

void setupBenchRoutes() {
  collectBenchStaticDiagnostics();
  benchServer.on("/bench", HTTP_GET, []() {
    benchServer.sendHeader("Cache-Control", "no-store");
    benchServer.send_P(200, "text/html", kBenchPage);
  });
  benchServer.on("/recording.csv", HTTP_GET, sendRecordingCsv);
  benchServer.on("/record", HTTP_POST, []() {
    if (benchRecording.active()) {
      benchServer.send(409, "text/plain", "A recording is already running.");
      return;
    }
    if (timer == nullptr) {
      benchServer.send(503, "text/plain", "Sensor timer unavailable; recording was not started.");
      return;
    }
    enterBenchMode();
    stopBenchSamples(bench_recording::FinishReason::Replaced);
    benchRecording.begin(millis());
    ++benchRecordingId;
    benchRecordingError = "";
    benchAutomatic = true;
    benchLastSample = millis() - 10; // Allow an immediate first sample.
    benchServer.sendHeader("Location", "/bench");
    benchServer.send(303);
  });
  benchServer.on("/diagnostics", HTTP_GET, sendBenchDiagnostics);
  benchServer.on("/mode", HTTP_POST, []() {
    const String mode = benchServer.arg("mode");
    if (mode != "off" && mode != "single" &&
        mode != "auto" && mode != "timeout") {
      benchServer.send(400, "text/plain", "Unknown mode");
      return;
    }
    enterBenchMode();
    stopBenchSamples();
    benchSingle = mode == "single";
    benchAutomatic = mode == "auto" || mode == "timeout";
    simulateSensorTimeout = mode == "timeout";
    benchServer.sendHeader("Location", "/bench");
    benchServer.send(303);
  });
  for (const char *path : {"/configs.json", "/addresses.json"}) {
    String filename(path);
    benchServer.on(("/settings" + filename).c_str(), HTTP_GET, [filename]() {
      if (!benchFilesystem) {
        benchServer.send(503, "text/plain", "Filesystem unavailable; not formatted");
        return;
      }
      File file = SPIFFS.open(filename, FILE_READ);
      if (!file) {
        benchServer.send(404, "text/plain", "File not found");
        return;
      }
      benchServer.sendHeader("Cache-Control", "no-store");
      benchServer.streamFile(file, "application/json");
      file.close();
    });
  }

  ArduinoOTA.setHostname(benchHostname.c_str());
  ArduinoOTA.onStart([]() {
    enterBenchMode();
    otaInProgress = true;
    stopBenchSamples(bench_recording::FinishReason::Ota);
    // ArduinoOTA calls this after Update.begin but before receiving/writing
    // payload. Update.begin allocates only; abort any filesystem request.
    if (ArduinoOTA.getCommand() != U_FLASH)
      Update.abort();
  });
  ArduinoOTA.onError([](ota_error_t) { otaInProgress = false; stopBenchSamples(); });
  ArduinoOTA.begin();
}
