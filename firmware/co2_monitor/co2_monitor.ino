/*
  CO2 Monitor - step 2
  ESP32-C6-DevKitC-1 + Adafruit SHT45 + Adafruit SCD-41 + Joy-it 0.96" OLED (SSD1306)

  - SHT45  -> temperature and humidity (more accurate than the SCD-41's own values)
  - SCD-41 -> CO2 in periodic mode (a new value every 5 s)
  - Warm-up: for the first 30 s after start-up the sensors measure, but no value
    is shown, printed or saved. The display counts down meanwhile.
  - The OLED switches off after 30 s. The button on GPIO2 wakes it for another 30 s.
  - The RGB LED on the board shows the CO2 level all the time, dimmed:
    green -> yellow -> red.
  - Sleep mode for the night: the RGB LED is off and the display shows a small moon.
    Hold the button for 2 s, or use the switch on the web page, to turn it on or off.
  - All readings are also printed at 115200 baud (Serial Monitor / Serial Plotter).
  - Wi-Fi gets the time from the internet. The values are stored in the flash
    as 1-minute, 5-minute and hourly averages (history.h explains how).
  - A web page on the home network shows the live values, graphs of any stored
    day and CSV downloads (web.h explains how).
  - Serial Monitor commands: status, files, csv m1|m5|h1 [days], save, sleep on|off

  The sensors keep measuring while the display is off and in sleep mode. This matters for the
  SCD-41's automatic self-calibration, which needs uninterrupted operation.

  Libraries (Arduino IDE -> Library Manager):
    Adafruit SSD1306, Adafruit SHT4x Library, Sensirion I2C SCD4x
  Board: "ESP32C6 Dev Module" (esp32 by Espressif Systems)
    Flash Size "8MB", Partition Scheme "8M with spiffs (3MB APP/1.5MB SPIFFS)"
  Wi-Fi: copy secrets_example.h to secrets.h and enter your network there.
*/

#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT4x.h>
#include <SensirionI2cScd4x.h>
#include "history.h"
#include "web.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "secrets.h is missing: copy secrets_example.h to secrets.h and enter your Wi-Fi name and password"
#endif

// ---------- Pins ----------
constexpr uint8_t I2C_SDA_PIN = 23;  // J3 pin 5
constexpr uint8_t I2C_SCL_PIN = 22;  // J3 pin 6
constexpr uint8_t BUTTON_PIN = 2;    // J1 pin 12, the other side of the button goes to GND

// ---------- Display ----------
constexpr uint8_t OLED_ADDRESS = 0x3C;  // 0x3D if the address resistor on the back was moved
// 0 = normal (header pins at the top). Use 2 if the text appears upside down.
constexpr uint8_t OLED_ROTATION = 0;
// Brightness 0-255 (the controller's "contrast"). 207 is the library default.
// Even 0 stays readable, at roughly half brightness. Lower also slows OLED burn-in.
constexpr uint8_t OLED_BRIGHTNESS = 10;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr uint32_t DISPLAY_ON_TIME_MS = 30000;

// ---------- Sensors ----------
// Height of your location above sea level in metres. The SCD-41 uses it to
// compensate the CO2 reading for air pressure. Find it on any map / GPS app.
constexpr uint16_t SENSOR_ALTITUDE_M = 0;
// After start-up the sensors measure this long before any value is shown, printed
// or saved. Values measured during the warm-up are thrown away.
constexpr uint32_t WARMUP_TIME_MS = 30000;
constexpr uint32_t SHT_INTERVAL_MS = 2000;
constexpr uint32_t SCD_POLL_INTERVAL_MS = 1000;

// CO2 rating limits in ppm (shown in the top right corner of the display)
constexpr uint16_t CO2_GOOD_BELOW = 800;
constexpr uint16_t CO2_OK_BELOW = 1000;
constexpr uint16_t CO2_VENTILATE_BELOW = 2000;

// ---------- RGB LED (on the board, GPIO8) ----------
// Brightness 1-255, 0 = LED off. For comparison, the BlinkRGB example uses 64.
constexpr uint8_t LED_BRIGHTNESS = 5;

struct LedColour {
  uint8_t r, g, b;
};
// The colour changes smoothly: green below CO2_GOOD_BELOW, yellow at CO2_OK_BELOW,
// red from CO2_VENTILATE_BELOW. Yellow has less green because the green LED is brighter.
constexpr LedColour LED_GREEN = {0, 255, 0};
constexpr LedColour LED_YELLOW = {255, 160, 0};
constexpr LedColour LED_RED = {255, 0, 0};

// ---------- Sleep mode ----------
// For the night: the RGB LED stays off and the display shows a small moon. The sensors,
// the history and the web page keep running. The setting survives a restart.
// Holding the button this long turns sleep mode on or off.
constexpr uint32_t LONG_PRESS_MS = 2000;
// After a long press the display confirms the new mode for this long. In sleep mode it
// then goes dark at once.
constexpr uint32_t SLEEP_MESSAGE_MS = 2500;

// ---------- Wi-Fi and clock ----------
// The name the monitor shows in your router's list of devices
constexpr char WIFI_HOSTNAME[] = "co2-monitor";
// Internet time servers. The clock runs on UTC; the web page shows local time.
constexpr char NTP_SERVER_1[] = "pool.ntp.org";
constexpr char NTP_SERVER_2[] = "time.google.com";

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_SHT4x sht4;
SensirionI2cScd4x scd4x;
Preferences settings;  // keeps the sleep mode through a restart

// 8 x 8 moon, shown in the top line of the display while sleep mode is on
const uint8_t MOON_ICON[] PROGMEM = {0x30, 0x60, 0xE0, 0xE0, 0xE0, 0xF8, 0x7E, 0x3C};

bool displayFound = false;
bool shtFound = false;
bool scdFound = false;

bool displayOn = false;
uint32_t displayOnSince = 0;

bool sleepMode = false;
bool sleepMessageShown = false;  // the display confirms a long press
uint32_t sleepMessageSince = 0;

bool warmingUp = true;
uint32_t warmupStart = 0;  // when the sensors started measuring

float temperatureC = NAN;
float humidityPct = NAN;
uint16_t co2Ppm = 0;  // 0 = no valid reading yet
bool newData = false;  // something on the display has changed

uint32_t lastShtRead = 0;
uint32_t lastScdPoll = 0;

enum class ButtonEvent { None, Press, LongPress };

// Debounced, the button pulls the pin to GND. Press comes as soon as the button goes
// down, LongPress once more when it has been held for LONG_PRESS_MS.
ButtonEvent readButton() {
  static bool stableState = HIGH;
  static bool lastReading = HIGH;
  static uint32_t lastChange = 0;
  static uint32_t pressedSince = 0;
  static bool longPressReported = false;

  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) {
    lastReading = reading;
    lastChange = millis();
  }
  if (millis() - lastChange > 30 && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      pressedSince = millis();
      longPressReported = false;
      return ButtonEvent::Press;
    }
  }
  if (stableState == LOW && !longPressReported && millis() - pressedSince >= LONG_PRESS_MS) {
    longPressReported = true;
    return ButtonEvent::LongPress;
  }
  return ButtonEvent::None;
}

const char* co2Rating(uint16_t ppm) {
  if (ppm == 0) return "";
  if (ppm < CO2_GOOD_BELOW) return "GOOD";
  if (ppm < CO2_OK_BELOW) return "OK";
  if (ppm < CO2_VENTILATE_BELOW) return "VENTILATE";
  return "BAD AIR";
}

// step 0 = from, step == steps = to
LedColour blendColour(LedColour from, LedColour to, uint16_t step, uint16_t steps) {
  return {(uint8_t)(from.r + ((int32_t)to.r - from.r) * step / steps),
          (uint8_t)(from.g + ((int32_t)to.g - from.g) * step / steps),
          (uint8_t)(from.b + ((int32_t)to.b - from.b) * step / steps)};
}

LedColour co2Colour(uint16_t ppm) {
  if (ppm < CO2_GOOD_BELOW) return LED_GREEN;
  if (ppm < CO2_OK_BELOW) {
    return blendColour(LED_GREEN, LED_YELLOW, ppm - CO2_GOOD_BELOW, CO2_OK_BELOW - CO2_GOOD_BELOW);
  }
  if (ppm < CO2_VENTILATE_BELOW) {
    return blendColour(LED_YELLOW, LED_RED, ppm - CO2_OK_BELOW, CO2_VENTILATE_BELOW - CO2_OK_BELOW);
  }
  return LED_RED;
}

uint8_t dimmed(uint8_t value) {
  return (value * LED_BRIGHTNESS + 127) / 255;
}

// LED off until the first CO2 value arrives, and in sleep mode
void updateLed() {
  LedColour colour = co2Ppm > 0 && !sleepMode ? co2Colour(co2Ppm) : LedColour{0, 0, 0};
  rgbLedWrite(RGB_BUILTIN, dimmed(colour.r), dimmed(colour.g), dimmed(colour.b));
}

// Seconds of warm-up left, rounded up. 0 once it is over.
uint8_t warmupSecondsLeft() {
  uint32_t elapsed = millis() - warmupStart;
  return elapsed >= WARMUP_TIME_MS ? 0 : (WARMUP_TIME_MS - elapsed + 999) / 1000;
}

// The built-in font is 6 pixels wide per character at size 1
void printCentred(const char *text, uint8_t size, int16_t y) {
  display.setTextSize(size);
  display.setCursor((OLED_WIDTH - (int16_t)strlen(text) * 6 * size) / 2, y);
  display.print(text);
}

// Big moon or sun with the new mode, shown for a moment after a long press
void drawSleepMessage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (sleepMode) {
    display.fillCircle(64, 14, 12, SSD1306_WHITE);
    display.fillCircle(71, 8, 11, SSD1306_BLACK);
  } else {
    display.fillCircle(64, 14, 6, SSD1306_WHITE);
    for (uint8_t i = 0; i < 8; i++) {
      float angle = i * PI / 4;
      display.drawLine(64 + lroundf(cosf(angle) * 9), 14 + lroundf(sinf(angle) * 9), 64 + lroundf(cosf(angle) * 12),
                       14 + lroundf(sinf(angle) * 12), SSD1306_WHITE);
    }
  }
  printCentred("Sleep mode", 2, 32);
  printCentred(sleepMode ? "on - LED off" : "off - LED on", 1, 54);
  display.display();
}

void drawScreen() {
  char text[12];
  if (sleepMessageShown) {
    drawSleepMessage();
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Top line: label left, moon in sleep mode, rating right (inverted when the air is bad)
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("CO2 ppm"));
  if (sleepMode) display.drawBitmap(53, 0, MOON_ICON, 8, 8, SSD1306_WHITE);
  const char* rating = co2Rating(co2Ppm);
  int16_t ratingX = OLED_WIDTH - strlen(rating) * 6;
  if (co2Ppm >= CO2_OK_BELOW) {
    display.fillRect(ratingX - 2, 0, OLED_WIDTH - ratingX + 2, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.setCursor(ratingX, 1);
  display.print(rating);
  display.setTextColor(SSD1306_WHITE);

  // CO2 value, large and centred
  if (!scdFound) {
    printCentred("SCD-41 missing", 1, 24);
  } else if (warmingUp) {
    printCentred("warming up", 1, 13);
    snprintf(text, sizeof text, "%u s", warmupSecondsLeft());
    printCentred(text, 2, 26);
  } else if (co2Ppm > 0) {
    snprintf(text, sizeof text, "%u", co2Ppm);
    printCentred(text, 3, 15);
  } else {
    printCentred("measuring...", 1, 24);  // first value after the warm-up, within 5 s
  }

  display.drawFastHLine(0, 44, OLED_WIDTH, SSD1306_WHITE);

  // Bottom line: temperature left, humidity right
  display.setTextSize(2);
  display.setCursor(0, 49);
  if (isnan(temperatureC)) {
    display.print(F("--.-"));
  } else {
    snprintf(text, sizeof text, "%.1f", temperatureC);
    display.print(text);
  }
  int16_t unitX = display.getCursorX() + 3;
  display.drawCircle(unitX, 50, 1, SSD1306_WHITE);  // degree sign
  display.setTextSize(1);
  display.setCursor(unitX + 3, 49);
  display.print('C');

  if (isnan(humidityPct)) {
    strcpy(text, "--%");
  } else {
    snprintf(text, sizeof text, "%d%%", (int)lroundf(humidityPct));
  }
  display.setTextSize(2);
  display.setCursor(OLED_WIDTH - strlen(text) * 12, 49);
  display.print(text);

  display.display();
}

void displayWake() {
  if (!displayFound) return;
  if (!displayOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    displayOn = true;
  }
  displayOnSince = millis();
  drawScreen();
}

void displayOff() {
  if (!displayFound) return;
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  displayOn = false;
  sleepMessageShown = false;
}

// From the button, the web page or the Serial Monitor
void setSleepMode(bool on) {
  if (on != sleepMode) {
    sleepMode = on;
    settings.putBool("sleep", on);
    updateLed();
    newData = true;  // adds or removes the moon if the display is on
  }
  Serial.println(on ? F("Sleep mode on: LED off") : F("Sleep mode off: LED on"));
}

// Long press: switches sleep mode and confirms it on the display
void toggleSleepMode() {
  setSleepMode(!sleepMode);
  sleepMessageShown = true;
  sleepMessageSince = millis();
  displayWake();
}

bool initScd41() {
  scd4x.begin(Wire, SCD41_I2C_ADDR_62);

  // Bring the sensor to a known idle state. It keeps measuring when only the
  // ESP32 is reset, and some commands are only accepted while it is idle.
  scd4x.wakeUp();  // returns an error if the sensor was not asleep - that's fine
  scd4x.stopPeriodicMeasurement();
  scd4x.reinit();

  uint64_t serialNumber = 0;
  if (scd4x.getSerialNumber(serialNumber) != 0) return false;
  Serial.printf("SCD-41 found, serial 0x%04lX%08lX\n", (unsigned long)(serialNumber >> 32),
                (unsigned long)(serialNumber & 0xFFFFFFFF));

  uint16_t ascEnabled = 0;
  if (scd4x.getAutomaticSelfCalibrationEnabled(ascEnabled) == 0) {
    Serial.printf("SCD-41 automatic self-calibration (ASC): %s\n", ascEnabled ? "enabled" : "disabled");
  }

  // Stored in the sensor's RAM only, so it is sent on every boot (no EEPROM wear).
  scd4x.setSensorAltitude(SENSOR_ALTITUDE_M);

  return scd4x.startPeriodicMeasurement() == 0;
}

void readSht45() {
  sensors_event_t humidity, temperature;
  if (sht4.getEvent(&humidity, &temperature)) {
    temperatureC = temperature.temperature;
    humidityPct = humidity.relative_humidity;
  } else {
    temperatureC = NAN;
    humidityPct = NAN;
  }
  newData = true;
}

void readScd41() {
  bool dataReady = false;
  if (scd4x.getDataReadyStatus(dataReady) != 0 || !dataReady) return;

  uint16_t co2 = 0;
  float scdTemperature = 0;  // not used, the SHT45 is more accurate
  float scdHumidity = 0;
  if (scd4x.readMeasurement(co2, scdTemperature, scdHumidity) != 0) return;
  if (co2 == 0) return;  // the sensor reports 0 for an invalid sample
  if (warmingUp) return;  // read anyway, so the next value is a new one

  co2Ppm = co2;
  newData = true;
  updateLed();
  historyAddSample(co2Ppm, temperatureC, humidityPct, NAN);  // no pressure sensor yet

  // "name:value" pairs can be read by people and by the Serial Plotter
  Serial.printf("CO2_ppm:%u Temperature_C:%.2f Humidity_pct:%.1f\n", co2Ppm, temperatureC, humidityPct);
}

// Counts the warm-up down on the display and ends it after WARMUP_TIME_MS
void updateWarmup() {
  static uint8_t secondsShown = 0;
  if (!warmingUp) return;

  uint8_t left = warmupSecondsLeft();
  if (left > 0) {
    if (left != secondsShown) {
      secondsShown = left;
      newData = true;
    }
    return;
  }

  warmingUp = false;
  Serial.println(F("Warm-up done, values are shown and saved from now on"));
  if (shtFound) {
    lastShtRead = millis();
    readSht45();
  }
  displayWake();  // shows the first values for DISPLAY_ON_TIME_MS
}

void startWifi() {
  if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "your-wifi-name") == 0) {
    Serial.println(F("Wi-Fi: enter your network in secrets.h - no internet time, so no history"));
    return;
  }
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // connects in the background and reconnects by itself
  Serial.printf("Wi-Fi: connecting to \"%s\"\n", WIFI_SSID);
}

// Reports connection changes and starts the internet time and the web page once Wi-Fi is up
void updateWifi() {
  static bool wasConnected = false;
  static bool timeStarted = false;

  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected == wasConnected) return;
  wasConnected = connected;
  if (connected) {
    Serial.printf("Wi-Fi connected, web page at http://%s/\n", WiFi.localIP().toString().c_str());
    if (!timeStarted) {
      configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);  // UTC, updated every 3 hours
      timeStarted = true;
    }
    webBegin(CO2_GOOD_BELOW, CO2_OK_BELOW, CO2_VENTILATE_BELOW, setSleepMode);  // only the first time
  } else {
    Serial.println(F("Wi-Fi lost, reconnecting"));
  }
}

void printStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi: connected to \"%s\", signal %d dBm, web page at http://%s/\n", WIFI_SSID, WiFi.RSSI(),
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println(F("Wi-Fi: not connected"));
  }
  if (warmingUp) Serial.printf("Warm-up: %u s left\n", warmupSecondsLeft());
  Serial.printf("Sleep mode: %s\n", sleepMode ? "on (LED off)" : "off");
  historyPrintStatus(Serial);
}

// Commands typed in the Serial Monitor (line ending: New Line, Carriage Return or Both)
void runCommand(char *line) {
  const char *command = strtok(line, " ");
  const char *option = strtok(nullptr, " ");  // csv: the level, sleep: on or off
  const char *days = strtok(nullptr, " ");
  if (!command) return;

  if (strcmp(command, "status") == 0) {
    printStatus();
  } else if (strcmp(command, "files") == 0) {
    historyPrintFiles(Serial);
  } else if (strcmp(command, "csv") == 0) {
    if (!option || !historyPrintCsv(Serial, option, days ? atoi(days) : 1)) {
      Serial.println(F("Use: csv m1|m5|h1 [days], for example \"csv m1 1\""));
    }
  } else if (strcmp(command, "save") == 0) {
    Serial.println(historySave() ? F("History saved") : F("Not saved - see \"status\""));
  } else if (strcmp(command, "sleep") == 0 && option && strcmp(option, "on") == 0) {
    setSleepMode(true);
  } else if (strcmp(command, "sleep") == 0 && option && strcmp(option, "off") == 0) {
    setSleepMode(false);
  } else {
    Serial.println(F("Commands: status, files, csv m1|m5|h1 [days], save, sleep on|off"));
  }
}

void readSerialCommands() {
  static char line[32];
  static uint8_t length = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[length] = '\0';
      if (length > 0) runCommand(line);
      length = 0;
    } else if (length < sizeof line - 1) {
      line[length++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) delay(10);  // don't hang when no PC is connected
  Serial.println(F("\nCO2 monitor starting"));

  historyBegin();
  startWifi();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  settings.begin("co2monitor");
  sleepMode = settings.getBool("sleep", false);
  if (sleepMode) Serial.println(F("Sleep mode on: LED off"));

  // The LED keeps its last colour through a reset (for example from BlinkRGB), so clear it
  updateLed();

  displayFound = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false);
  if (displayFound) {
    display.setRotation(OLED_ROTATION);
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(OLED_BRIGHTNESS);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(28, 16);
    display.print(F("CO2"));
    display.setTextSize(1);
    display.setCursor(28, 40);
    display.print(F("starting..."));
    display.display();
    displayOn = true;
  } else {
    Serial.println(F("OLED not found - check wiring and OLED_ADDRESS"));
  }

  shtFound = sht4.begin(&Wire);
  if (shtFound) {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT45 found"));
  } else {
    Serial.println(F("SHT45 not found - check wiring"));
  }

  scdFound = initScd41();
  if (scdFound) {
    Serial.println(F("SCD-41 measuring"));
  } else {
    Serial.println(F("SCD-41 not found - check wiring"));
  }

  warmupStart = millis();
  Serial.printf("Warm-up: the first values follow in %lu s\n", (unsigned long)(WARMUP_TIME_MS / 1000));
  displayWake();
}

void loop() {
  ButtonEvent button = readButton();
  if (button == ButtonEvent::Press) displayWake();
  if (button == ButtonEvent::LongPress) toggleSleepMode();
  readSerialCommands();
  updateWifi();
  historyLoop();
  updateWarmup();
  webLoop({co2Ppm, temperatureC, humidityPct, warmupSecondsLeft(), sleepMode});

  if (shtFound && !warmingUp && millis() - lastShtRead >= SHT_INTERVAL_MS) {
    lastShtRead = millis();
    readSht45();
  }
  if (scdFound && millis() - lastScdPoll >= SCD_POLL_INTERVAL_MS) {
    lastScdPoll = millis();
    readScd41();
  }

  if (displayOn) {
    if (sleepMessageShown && millis() - sleepMessageSince >= SLEEP_MESSAGE_MS) {
      sleepMessageShown = false;
      if (sleepMode) {
        displayOff();  // dark at once for the night
      } else {
        drawScreen();  // the values, until the display times out
      }
    } else if (millis() - displayOnSince >= DISPLAY_ON_TIME_MS) {
      displayOff();
    } else if (newData) {
      drawScreen();
    }
  }
  newData = false;
}
