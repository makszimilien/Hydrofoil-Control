#include <Arduino.h>
#include <WiFi.h>
#include <TickTwo.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <vector>

#define IO_PIN 0

double loop_freq = 0.0;
std::vector<int> raw_values;
hw_timer_t *timer = NULL;
int min_measured = 0;
int max_measured = 0;

int getMedian()
{
    std::vector<int> temp = raw_values;
    std::sort(temp.begin(), temp.end());
    return temp[temp.size() / 2];
}

void stat()
{
    Serial.println("");
    Serial.print("Loop freq: ");
    Serial.print(loop_freq);
    Serial.println(" Hz");
    Serial.print("Median values: ");

    if (raw_values.size() == 0)
    {
        Serial.println("No values");
        return;
    }
    Serial.print("Latest: ");
    Serial.print(raw_values.back());
    int median = getMedian();
    if (median < min_measured)
    {
        min_measured = median;
    }
    if (median > max_measured)
    {
        max_measured = median;
    }
    Serial.println(median);

    int barWidth = 80;
    float progress = static_cast<float>(median - min_measured) / (max_measured - min_measured);
    int pos = barWidth * progress;

    Serial.print("[");
    for (int i = 0; i < barWidth; ++i)
    {
        if (i < pos)
            Serial.print("=");
        else if (i == pos)
            Serial.print(">");
        else
            Serial.print(" ");
    }
    Serial.println("]");
}

void measure()
{
    pinMode(IO_PIN, OUTPUT);
    digitalWrite(IO_PIN, LOW);
    delayMicroseconds(50);
    pinMode(IO_PIN, INPUT);
    timerRestart(timer);
    while (digitalRead(IO_PIN) == LOW)
        ;
    raw_values.push_back(timerRead(timer));
    if (raw_values.size() > 50)
    {
        raw_values.erase(raw_values.begin());
    }
}

TickTwo stat_ticker([]()
                    { stat(); },
                    20, 0, MILLIS);
TickTwo arduino_ota_ticker([]()
                           { ArduinoOTA.handle(); },
                           1, 0, MILLIS);
TickTwo measure_ticker([]()
                       { measure(); },
                       1, 0, MILLIS);

void initFS()
{
    // Initialize LittleFS/SPIFFS file-system
    if (!LittleFS.begin())
    {
        LittleFS.format();

        if (!LittleFS.begin())
        {
            while (true)
            {
                Serial.println(F("LittleFS failed!"));

                // Stay forever here as useless to go further
                delay(5000);
            }
        }
    }
}

void setup()
{
    pinMode(IO_PIN, INPUT);

    stat_ticker.start();
    Serial.begin(1000000);
    Serial.println("Booting");
    const char *ssid = "honeypot_lf";
    const char *password = "Best-Kept-Secret";
    WiFi.setHostname("sensor");
    ArduinoOTA.setHostname("sensor");
    WiFi.begin(ssid, password);
    WiFi.waitForConnectResult(10000);
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connection failed! Continuing anyway...");
    }
    else
    {
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    }
    ArduinoOTA
        .onStart([]()
                 {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
      stat_ticker.stop();
      measure_ticker.stop(); });
    ArduinoOTA.onEnd([]()
                     { Serial.println("\nEnd");
                     stat_ticker.start();
                     measure_ticker.start(); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

    ArduinoOTA.setPassword("78dkj6u8t23489puihsef");
    ArduinoOTA.setPort(3232);
    ArduinoOTA.setRebootOnSuccess(true);
    ArduinoOTA.begin();
    initFS();
    arduino_ota_ticker.start();
    Serial.println("Boot complete");
    timer = timerBegin(1, 2, true);
    measure_ticker.start();
}

void loop()
{
    auto prev_time = micros();
    arduino_ota_ticker.update();
    stat_ticker.update();
    measure_ticker.update();
    loop_freq = 1000000.0 / (micros() - prev_time);
    prev_time = micros();
}
