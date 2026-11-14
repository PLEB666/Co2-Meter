/*
  Web page - see web.h for the overview.
*/
#include "web.h"

#include <WebServer.h>
#include <WiFi.h>
#include <esp_timer.h>

#include "history.h"
#include "web_page.h"

// Longest time span one data request may cover. This keeps every answer below about
// 60 KB, so the sensors are still read on time while the page loads.
constexpr uint32_t SECONDS_PER_DAY = 86400;
constexpr uint32_t MAX_SPAN_MINUTE = 2 * SECONDS_PER_DAY;   // 1-minute values, 40 KB
constexpr uint32_t MAX_SPAN_FIVE = 14 * SECONDS_PER_DAY;    // 5-minute values, 56 KB
constexpr uint32_t MAX_SPAN_DAYS = 45 * SECONDS_PER_DAY;    // calendar markers

static WebServer server(80);
static bool started = false;
static LiveValues live = {0, NAN, NAN, NAN, NAN, 0, 0, 0, false, 0, true};
static uint16_t co2Limit[3];
static uint16_t vocLimit[3];
static void (*sleepModeSetter)(bool on) = nullptr;
static uint8_t (*brightnessSetter)(int level) = nullptr;

// Sends everything printed to it as one HTTP answer, in pieces ("chunked"),
// so the size doesn't have to be known in advance
class ChunkedResponse : public Print {
 public:
  explicit ChunkedResponse(const char *contentType) {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, contentType, "");
  }
  ~ChunkedResponse() {
    sendBuffer();
    server.sendContent("");  // an empty piece ends the answer
  }
  size_t write(uint8_t c) override {
    return write(&c, 1);
  }
  size_t write(const uint8_t *data, size_t size) override {
    size_t done = 0;
    while (done < size) {
      size_t n = min(size - done, sizeof buffer - length);
      memcpy(buffer + length, data + done, n);
      length += n;
      done += n;
      if (length == sizeof buffer) sendBuffer();
    }
    return size;
  }

 private:
  void sendBuffer() {
    if (length > 0) server.sendContent((const char *)buffer, length);
    length = 0;
  }
  uint8_t buffer[1024];
  size_t length = 0;
};

// Prints "name":value, or "name":null for a missing value
static void printJsonNumber(Print &out, const char *name, float value, uint8_t decimals) {
  out.printf("\"%s\":", name);
  if (isnan(value)) {
    out.print("null");
  } else {
    out.print(value, decimals);
  }
}

// Reads the "from" and "to" arguments. Answers with an error and returns false if
// they are missing, the wrong way round or further apart than maxSpan.
static bool readSpan(uint32_t maxSpan, uint32_t &from, uint32_t &to) {
  from = strtoul(server.arg("from").c_str(), nullptr, 10);
  to = strtoul(server.arg("to").c_str(), nullptr, 10);
  if (from == 0 || to <= from || to - from > maxSpan) {
    server.send(400, "text/plain", "Needs from and to (UTC seconds), at most " + String(maxSpan / SECONDS_PER_DAY) + " days apart");
    return false;
  }
  return true;
}

static void handlePage() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html; charset=utf-8", (PGM_P)WEB_PAGE, sizeof WEB_PAGE);
}

static void handleNow() {
  ChunkedResponse out("application/json");
  out.printf("{\"time\":%lu,\"clock\":%s,\"uptime\":%lu,\"warmup\":%u,", (unsigned long)time(nullptr),
             historyClockSet() ? "true" : "false", (unsigned long)(esp_timer_get_time() / 1000000), live.warmupSecondsLeft);
  printJsonNumber(out, "co2", live.co2Ppm > 0 ? live.co2Ppm : NAN, 0);
  out.print(',');
  printJsonNumber(out, "temperature", live.temperatureC, 2);
  out.print(',');
  printJsonNumber(out, "humidity", live.humidityPct, 1);
  out.print(',');
  printJsonNumber(out, "pressure", live.pressureHpa, 2);
  out.print(',');
  printJsonNumber(out, "light", live.lightLux, 1);
  out.print(',');
  printJsonNumber(out, "voc", live.vocIndex > 0 ? live.vocIndex : NAN, 0);
  out.printf(",\"vocLearnLeft\":%lu,\"vocLimits\":[%u,%u,%u]", (unsigned long)live.vocLearnSecondsLeft, vocLimit[0],
             vocLimit[1], vocLimit[2]);
  out.printf(",\"sleep\":%s,\"brightness\":%u,\"autoBrightness\":%s,\"limits\":[%u,%u,%u],\"rssi\":%d}",
             live.sleepMode ? "true" : "false", live.brightness, live.autoBrightness ? "true" : "false", co2Limit[0],
             co2Limit[1], co2Limit[2], WiFi.RSSI());
}

static void handleSleep() {
  String on = server.arg("on");
  if (on != "0" && on != "1") {
    server.send(400, "text/plain", "on must be 0 or 1");
    return;
  }
  live.sleepMode = on == "1";
  sleepModeSetter(live.sleepMode);
  server.send(200, "application/json", live.sleepMode ? "{\"sleep\":true}" : "{\"sleep\":false}");
}

// auto=1, or level=0..100 (whole %)
static void handleBrightness() {
  String level = server.arg("level");
  int wanted;
  if (server.arg("auto") == "1") {
    wanted = BRIGHTNESS_AUTO;
  } else if (level.length() >= 1 && level.length() <= 3 && strspn(level.c_str(), "0123456789") == level.length() &&
             level.toInt() <= 100) {
    wanted = level.toInt();
  } else {
    server.send(400, "text/plain", "Needs auto=1 or level=0..100");
    return;
  }
  live.brightness = brightnessSetter(wanted);
  live.autoBrightness = wanted == BRIGHTNESS_AUTO;
  char answer[48];
  snprintf(answer, sizeof answer, "{\"auto\":%s,\"brightness\":%u}", live.autoBrightness ? "true" : "false",
           live.brightness);
  server.send(200, "application/json", answer);
}

static void handleInfo() {
  ChunkedResponse out("application/json");
  historyWriteInfoJson(out);
}

static void handleDays() {
  uint32_t from, to;
  if (!readSpan(MAX_SPAN_DAYS, from, to)) return;
  ChunkedResponse out("application/json");
  historyWriteDaysJson(out, from, to);
}

static void handleData() {
  long minutes = server.arg("res").toInt();
  uint32_t maxSpan = minutes == 1 ? MAX_SPAN_MINUTE : minutes == 5 ? MAX_SPAN_FIVE : 0;
  if (maxSpan == 0) {
    server.send(400, "text/plain", "res must be 1 or 5");
    return;
  }
  uint32_t from, to;
  if (!readSpan(maxSpan, from, to)) return;
  ChunkedResponse out("application/octet-stream");
  historyWriteRecords(out, minutes, from, to);
}

void webBegin(const uint16_t co2Limits[3], const uint16_t vocLimits[3], void (*setSleepMode)(bool on),
              uint8_t (*setBrightness)(int level)) {
  if (started) return;
  memcpy(co2Limit, co2Limits, sizeof co2Limit);
  memcpy(vocLimit, vocLimits, sizeof vocLimit);
  sleepModeSetter = setSleepMode;
  brightnessSetter = setBrightness;

  // Lets a copy of the page opened on the PC ask the monitor (index.html?device=<address>)
  server.enableCORS(true);
  server.on("/", HTTP_GET, handlePage);
  server.on("/api/now", HTTP_GET, handleNow);
  server.on("/api/info", HTTP_GET, handleInfo);
  server.on("/api/days", HTTP_GET, handleDays);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/sleep", HTTP_POST, handleSleep);
  server.on("/api/brightness", HTTP_POST, handleBrightness);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  started = true;
}

void webLoop(const LiveValues &values) {
  live = values;
  if (started) server.handleClient();
}
