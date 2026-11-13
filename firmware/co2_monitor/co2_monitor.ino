/*
  CO2 Monitor - case version
  Waveshare ESP32-C6-DEV-KIT-N16 + Adafruit SCD-41, SHT45, BMP580, SGP40, VEML7700
  + Adafruit 1.5" 128x128 grayscale OLED (SSD1327) + 8 mm LED (PL9823-F8) + button

  - SCD-41   -> CO2 in periodic mode (a new value every 5 s)
  - SHT45    -> temperature and humidity (more accurate than the SCD-41's own values)
  - BMP580   -> air pressure (0x46 or 0x47, whichever answers)
  - SGP40    -> VOC index (air quality, 100 = the room's usual air), worked out from
                its raw signal once per second with Sensirion's algorithm
  - VEML7700 -> light in lux
  - Warm-up: for the first 30 s after start-up the sensors measure, but no value
    is shown, printed or saved. The display counts down meanwhile.
  - The display shows CO2, temperature and humidity, laid out as on the prototype.
    Pressure, light and the VOC index are only printed (Serial, status, /api/now).
  - The OLED switches off after 30 s. The button on GPIO2 wakes it for another 30 s.
  - The 8 mm LED (PL9823-F8 on GPIO18) shows the CO2 level all the time, dimmed:
    green -> yellow -> red.
  - Sleep mode for the night: the LED is off and the display shows a small moon.
    Hold the button for 2 s, or use the switch on the web page, to turn it on or off.
  - All readings are also printed at 115200 baud (Serial Monitor / Serial Plotter).
  - Wi-Fi gets the time from the internet and uses the fixed address FIXED_IP.
    CO2, temperature and humidity are stored in the flash as 1-minute and 5-minute
    averages (history.h explains how). Pressure, light and VOC are not stored.
  - A web page on the home network shows the live values, graphs of any stored
    day and CSV downloads (web.h explains how).
  - Serial Monitor commands: status, files, csv m1|m5 [days], save, sleep on|off, led

  The sensors keep measuring while the display is off and in sleep mode. This matters for the
  SCD-41's automatic self-calibration, which needs uninterrupted operation, and for the SGP40's
  VOC index, which learns the room's usual air over hours.

  Not used yet: the speaker. The new sensors' values are only printed; what they will
  control (display brightness, pressure for the SCD-41, VOC warnings) comes later.

  Libraries (Arduino IDE -> Library Manager):
    Adafruit SSD1327, Adafruit SHT4x Library, Sensirion I2C SCD4x,
    Adafruit BMP5xx Library, Adafruit VEML7700 Library, Adafruit SGP40 Sensor
  Board: "ESP32C6 Dev Module" (esp32 by Espressif Systems)
    Flash Size "16MB", Partition Scheme "16M Flash (3MB APP/9.9MB FATFS)"
  Wi-Fi: copy secrets_example.h to secrets.h and enter your network there.
*/

#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include <Adafruit_SHT4x.h>
#include <SensirionI2cScd4x.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_SGP40.h>  // also brings Sensirion's VOC index algorithm
#include "history.h"
#include "web.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "secrets.h is missing: copy secrets_example.h to secrets.h and enter your Wi-Fi name and password"
#endif

// ---------- Pins ----------
constexpr uint8_t I2C_SDA_PIN = 23;  // J3 pin 12 (pin 1 = USB end)
constexpr uint8_t I2C_SCL_PIN = 22;  // J3 pin 11
constexpr uint8_t BUTTON_PIN = 2;    // J1 pin 5, the other side of the button goes to GND
constexpr uint8_t LED_PIN = 18;      // J3 pin 7, DIN of the 8 mm LED (its VDD to 5V, GND to GND)

// ---------- Display ----------
constexpr uint8_t OLED_ADDRESS = 0x3D;  // 0x3C if the A0 jumper on its back is closed
// 0-3, each step turns the picture a quarter turn clockwise. 0 = upright with the
// display's pin header at the top. 3 = as in the case, where the display is turned a
// quarter turn clockwise (seen from the front): pin header on the right, next to the
// light sensor. If the text stands on its head there, use 1.
constexpr uint8_t OLED_ROTATION = 3;
// Brightness 0-255 (the controller's "contrast"). 47 is the library default.
// Lower also slows OLED burn-in.
constexpr uint8_t OLED_BRIGHTNESS = 47;
constexpr uint8_t OLED_SIZE = 128;  // 128 x 128 pixels
constexpr uint32_t DISPLAY_ON_TIME_MS = 30000;
// Drawing the whole screen takes about 0.2 s on the I2C bus, so it is redrawn at most
// this often while new values come in.
constexpr uint32_t DISPLAY_REDRAW_MS = 1000;

// ---------- Sensors ----------
// Height of your location above sea level in metres. The SCD-41 uses it to
// compensate the CO2 reading for air pressure. Find it on any map / GPS app.
constexpr uint16_t SENSOR_ALTITUDE_M = 0;
// After start-up the sensors measure this long before any value is shown, printed
// or saved. Values measured during the warm-up are thrown away.
constexpr uint32_t WARMUP_TIME_MS = 30000;
constexpr uint32_t SHT_INTERVAL_MS = 2000;
constexpr uint32_t SCD_POLL_INTERVAL_MS = 1000;
constexpr uint32_t BMP_INTERVAL_MS = 2000;
constexpr uint32_t VEML_INTERVAL_MS = 1000;
// The VOC index algorithm expects exactly one SGP40 reading per second
constexpr uint32_t SGP_INTERVAL_MS = 1000;
constexpr uint32_t SGP_MEASURE_MS = 30;  // the SGP40 needs up to 30 ms per reading

constexpr uint8_t BMP580_ADDRESS_A = 0x46;  // the two addresses its
constexpr uint8_t BMP580_ADDRESS_B = 0x47;  // address jumper can choose
constexpr uint8_t SGP40_ADDRESS = 0x59;

// CO2 rating limits in ppm (shown in the top right corner of the display)
constexpr uint16_t CO2_GOOD_BELOW = 800;
constexpr uint16_t CO2_OK_BELOW = 1000;
constexpr uint16_t CO2_VENTILATE_BELOW = 2000;

// ---------- LED ----------
// The 8 mm LED on LED_PIN shows the CO2 colour.
// Brightness 1-255, 0 = LED off. For comparison, the BlinkRGB example uses 64.
constexpr uint8_t LED_BRIGHTNESS = 5;
// true = the board's own RGB LED (GPIO8) shows the colour as well. In the case the
// board's LED faces the floor, so it stays off there.
constexpr bool BOARD_LED_TOO = false;

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
// The monitor always uses this address, so the web page stays at http://192.168.178.40/.
// No other device may use it: reserve it for the monitor in the router, or pick one
// outside the router's DHCP range. false = the router hands out the address (DHCP).
constexpr bool USE_FIXED_IP = true;
const IPAddress FIXED_IP(192, 168, 178, 40);
const IPAddress ROUTER_IP(192, 168, 178, 1);  // gateway, also asked for names (DNS)
const IPAddress SUBNET_MASK(255, 255, 255, 0);
// Internet time servers. The clock runs on UTC; the web page shows local time.
constexpr char NTP_SERVER_1[] = "pool.ntp.org";
constexpr char NTP_SERVER_2[] = "time.google.com";

Adafruit_SSD1327 display(OLED_SIZE, OLED_SIZE, &Wire, -1);
Adafruit_SHT4x sht4;
SensirionI2cScd4x scd4x;
Adafruit_BMP5xx bmp;
Adafruit_VEML7700 veml;
Adafruit_SGP40 sgp;
VocAlgorithmParams vocAlgorithm;  // Sensirion's VOC index algorithm and what it has learned
Preferences settings;             // keeps the sleep mode through a restart

// 8 x 8 moon, shown in the top line of the display while sleep mode is on
const uint8_t MOON_ICON[] PROGMEM = {0x30, 0x60, 0xE0, 0xE0, 0xE0, 0xF8, 0x7E, 0x3C};

bool displayFound = false;
bool shtFound = false;
bool scdFound = false;
bool bmpFound = false;
uint8_t bmpAddress = 0;
bool vemlFound = false;
bool sgpFound = false;

bool displayOn = false;
uint32_t displayOnSince = 0;
bool screenChanged = false;  // something on the display has changed
uint32_t lastDraw = 0;

bool sleepMode = false;
bool sleepMessageShown = false;  // the display confirms a long press
uint32_t sleepMessageSince = 0;

bool warmingUp = true;
uint32_t warmupStart = 0;  // when the sensors started measuring

float temperatureC = NAN;
float humidityPct = NAN;
uint16_t co2Ppm = 0;  // 0 = no valid reading yet
float pressureHpa = NAN;
float lightLux = NAN;
bool lightTooBright = false;  // the light sensor is at the top of its range
uint16_t vocIndex = 0;        // 0 = no value yet (the algorithm gives none in its first 45 s)
uint16_t vocRaw = 0;          // the SGP40's raw signal, for the Serial Monitor

uint32_t lastShtRead = 0;
uint32_t lastScdPoll = 0;
uint32_t lastBmpRead = 0;
uint32_t lastVemlRead = 0;
uint32_t lastSgpStart = 0;
bool sgpMeasuring = false;  // a reading was started and is not fetched yet

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

// ---------- LED ----------

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

// Sends a colour to the 8 mm LED. The PL9823 takes 24 bits like the board's WS2812B, but
// red first and slower: a 1 is 1.35 us high and 0.35 us low, a 0 the other way round.
// rgbLedWrite() uses the WS2812B's faster timing, so the RMT peripheral sends the bits
// here directly, at 20 MHz (one tick = 0.05 us).
void pl9823Write(uint8_t r, uint8_t g, uint8_t b) {
  static bool ready = false;
  if (!ready) {
    ready = rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000);
    if (!ready) {
      Serial.println(F("LED: RMT start failed on LED_PIN"));
      return;
    }
  }
  uint32_t colour = (uint32_t)r << 16 | (uint32_t)g << 8 | b;
  rmt_data_t bits[24];
  for (uint8_t i = 0; i < 24; i++) {
    bool one = colour & (1UL << (23 - i));
    bits[i].level0 = 1;
    bits[i].duration0 = one ? 27 : 7;
    bits[i].level1 = 0;
    bits[i].duration1 = one ? 7 : 27;
  }
  rmtWrite(LED_PIN, bits, RMT_SYMBOLS_OF(bits), RMT_WAIT_FOR_EVER);
}

// LED off until the first CO2 value arrives, and in sleep mode
void updateLed() {
  LedColour colour = co2Ppm > 0 && !sleepMode ? co2Colour(co2Ppm) : LedColour{0, 0, 0};
  uint8_t r = dimmed(colour.r), g = dimmed(colour.g), b = dimmed(colour.b);
  pl9823Write(r, g, b);
  if (BOARD_LED_TOO) rgbLedWrite(RGB_BUILTIN, r, g, b);
}

// Serial Monitor command "led": red, green and blue for 1 s each, then back to the CO2
// colour. Shows that the LED is wired right and that its colours come in the right order.
void testLed() {
  const LedColour steps[] = {{40, 0, 0}, {0, 40, 0}, {0, 0, 40}};
  const char *names[] = {"red", "green", "blue"};
  for (uint8_t i = 0; i < 3; i++) {
    Serial.printf("LED test: %s\n", names[i]);
    pl9823Write(steps[i].r, steps[i].g, steps[i].b);
    if (BOARD_LED_TOO) rgbLedWrite(RGB_BUILTIN, steps[i].r, steps[i].g, steps[i].b);
    delay(1000);
  }
  updateLed();
  Serial.println(F("LED test done"));
}

// ---------- Display ----------

// Seconds of warm-up left, rounded up. 0 once it is over.
uint8_t warmupSecondsLeft() {
  uint32_t elapsed = millis() - warmupStart;
  return elapsed >= WARMUP_TIME_MS ? 0 : (WARMUP_TIME_MS - elapsed + 999) / 1000;
}

// The built-in font is 6 x 8 pixels per character at size 1, 12 x 16 at size 2, ...
int16_t textWidth(const char *text, uint8_t size) {
  return strlen(text) * 6 * size;
}

void printAt(const char *text, uint8_t size, int16_t x, int16_t y, uint8_t colour = SSD1327_WHITE) {
  display.setTextSize(size);
  display.setTextColor(colour);
  display.setCursor(x, y);
  display.print(text);
}

void printCentred(const char *text, uint8_t size, int16_t y) {
  printAt(text, size, (OLED_SIZE - textWidth(text, size)) / 2, y);
}

void printRight(const char *text, uint8_t size, int16_t y) {
  printAt(text, size, OLED_SIZE - textWidth(text, size), y);
}

// Big moon or sun with the new mode, shown for a moment after a long press
void drawSleepMessage() {
  display.clearDisplay();
  if (sleepMode) {
    display.fillCircle(64, 36, 22, SSD1327_WHITE);
    display.fillCircle(77, 25, 20, SSD1327_BLACK);
  } else {
    display.fillCircle(64, 36, 11, SSD1327_WHITE);
    for (uint8_t i = 0; i < 8; i++) {
      float angle = i * PI / 4;
      display.drawLine(64 + lroundf(cosf(angle) * 16), 36 + lroundf(sinf(angle) * 16), 64 + lroundf(cosf(angle) * 23),
                       36 + lroundf(sinf(angle) * 23), SSD1327_WHITE);
    }
  }
  printCentred("Sleep mode", 2, 72);
  printCentred(sleepMode ? "on - LED off" : "off - LED on", 1, 96);
  display.display();
}

/*
  Screen layout, 128 x 128 pixels, the same as on the prototype's 0.96" display,
  with more room:
    y   1  "CO2 ppm"   (moon in sleep mode)        rating
    y  28            CO2 value, large
    y  80  ------------------------------------------------
    y  96  temperature                             humidity
*/
void drawScreen() {
  char text[16];
  lastDraw = millis();
  screenChanged = false;
  if (sleepMessageShown) {
    drawSleepMessage();
    return;
  }
  display.clearDisplay();

  // Top line: label left, moon in sleep mode, rating right (inverted when the air is bad)
  printAt("CO2 ppm", 1, 0, 1);
  if (sleepMode) display.drawBitmap(53, 1, MOON_ICON, 8, 8, SSD1327_WHITE);
  const char *rating = co2Rating(co2Ppm);
  if (co2Ppm >= CO2_OK_BELOW) {
    int16_t ratingX = OLED_SIZE - textWidth(rating, 1);
    display.fillRect(ratingX - 2, 0, OLED_SIZE - ratingX + 2, 10, SSD1327_WHITE);
    printAt(rating, 1, ratingX, 1, SSD1327_BLACK);
  } else {
    printRight(rating, 1, 1);
  }

  // CO2 value, large and centred
  if (!scdFound) {
    printCentred("SCD-41 missing", 1, 41);
  } else if (warmingUp) {
    printCentred("warming up", 1, 26);
    snprintf(text, sizeof text, "%u s", warmupSecondsLeft());
    printCentred(text, 3, 42);
  } else if (co2Ppm > 0) {
    snprintf(text, sizeof text, "%u", co2Ppm);
    if (strlen(text) <= 4) {
      printCentred(text, 5, 28);  // 30 x 40 pixels per digit
    } else {
      printCentred(text, 4, 31);  // 10000 ppm and more: 5 digits only fit smaller
    }
  } else {
    printCentred("measuring...", 1, 41);  // first value after the warm-up, within 5 s
  }

  display.drawFastHLine(0, 80, OLED_SIZE, SSD1327_WHITE);

  // Bottom line: temperature left, humidity right
  if (isnan(temperatureC)) {
    strcpy(text, "--.-");
  } else {
    snprintf(text, sizeof text, "%.1f", temperatureC);
  }
  printAt(text, 2, 0, 96);
  int16_t unitX = display.getCursorX() + 3;
  display.drawCircle(unitX, 97, 1, SSD1327_WHITE);  // degree sign
  printAt("C", 1, unitX + 3, 96);

  if (isnan(humidityPct)) {
    strcpy(text, "--%");
  } else {
    snprintf(text, sizeof text, "%d%%", (int)lroundf(humidityPct));
  }
  printRight(text, 2, 96);

  display.display();
}

void displayWake() {
  if (!displayFound) return;
  if (!displayOn) {
    display.oled_command(SSD1327_DISPLAYON);
    displayOn = true;
  }
  displayOnSince = millis();
  drawScreen();
}

void displayOff() {
  if (!displayFound) return;
  display.oled_command(SSD1327_DISPLAYOFF);
  displayOn = false;
  sleepMessageShown = false;
}

// From the button, the web page or the Serial Monitor
void setSleepMode(bool on) {
  if (on != sleepMode) {
    sleepMode = on;
    settings.putBool("sleep", on);
    updateLed();
    screenChanged = true;  // adds or removes the moon if the display is on
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

// ---------- Sensors ----------

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

// The BMP580's address depends on its jumper, so both are tried. begin() resets it,
// checks its chip ID and starts it measuring on its own (normal mode).
bool initBmp580() {
  for (uint8_t address : {BMP580_ADDRESS_A, BMP580_ADDRESS_B}) {
    if (bmp.begin(address, &Wire)) {
      bmpAddress = address;
      return true;
    }
  }
  return false;
}

bool initVeml7700() {
  if (!veml.begin(&Wire)) return false;
  // Finer steps than the library's gain 1/8, for dim rooms: 0.06 lux per step, up to
  // about 3700 lux. A new value every 100 ms.
  veml.setGain(VEML7700_GAIN_1);
  return true;
}

// begin() ends with the sensor's self-test (about 0.5 s)
bool initSgp40() {
  if (!sgp.begin(&Wire)) return false;
  VocAlgorithm_init(&vocAlgorithm);
  return true;
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
  screenChanged = true;
}

void readBmp580() {
  pressureHpa = bmp.performReading() ? bmp.pressure : NAN;
}

void readVeml7700() {
  // The newest value, without waiting, with the library's correction for bright light
  lightLux = veml.readLux(VEML_LUX_CORRECTED_NOWAIT);
  lightTooBright = veml.readALS() == 0xFFFF;  // the same value again, raw
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
  screenChanged = true;
  updateLed();
  historyAddSample(co2Ppm, temperatureC, humidityPct, NAN);  // pressure is not stored

  // "name:value" pairs can be read by people and by the Serial Plotter
  Serial.printf("CO2_ppm:%u Temperature_C:%.2f Humidity_pct:%.1f Pressure_hPa:%.2f Light_lux:%.1f VOC_index:%u\n",
                co2Ppm, temperatureC, humidityPct, pressureHpa, lightLux, vocIndex);
}

// Sensirion's checksum, sent and received after every 2 bytes
uint8_t sensirionCrc(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
  }
  return crc;
}

// The SGP40 library waits 250 ms for every reading. Here the reading is started, and
// fetched SGP_MEASURE_MS later, so the button and the web page don't have to wait.
// The SHT45's values compensate it (25 C and 50 %RH while there are none).
bool startSgp40Reading() {
  float t = isnan(temperatureC) ? 25 : constrain(temperatureC, -45.0f, 130.0f);
  float rh = isnan(humidityPct) ? 50 : constrain(humidityPct, 0.0f, 100.0f);
  uint16_t rhTicks = (uint16_t)(rh * 65535 / 100 + 0.5f);
  uint16_t tTicks = (uint16_t)((t + 45) * 65535 / 175 + 0.5f);
  uint8_t command[8] = {0x26, 0x0F, (uint8_t)(rhTicks >> 8), (uint8_t)rhTicks, 0, (uint8_t)(tTicks >> 8), (uint8_t)tTicks, 0};
  command[4] = sensirionCrc(command + 2, 2);
  command[7] = sensirionCrc(command + 5, 2);
  Wire.beginTransmission(SGP40_ADDRESS);
  Wire.write(command, sizeof command);
  return Wire.endTransmission() == 0;
}

bool fetchSgp40Reading(uint16_t &raw) {
  uint8_t reply[3];
  if (Wire.requestFrom(SGP40_ADDRESS, (uint8_t)sizeof reply) != sizeof reply) return false;
  for (uint8_t &b : reply) b = Wire.read();
  if (sensirionCrc(reply, 2) != reply[2]) return false;
  raw = (uint16_t)reply[0] << 8 | reply[1];
  return true;
}

// One reading per second, also during the warm-up: the algorithm learns from every one
void updateSgp40() {
  if (!sgpFound) return;
  if (sgpMeasuring) {
    if (millis() - lastSgpStart < SGP_MEASURE_MS) return;
    sgpMeasuring = false;
    uint16_t raw;
    if (!fetchSgp40Reading(raw)) return;
    int32_t index = 0;
    VocAlgorithm_process(&vocAlgorithm, raw, &index);
    if (warmingUp) return;
    vocRaw = raw;
    vocIndex = index;
  } else if (millis() - lastSgpStart >= SGP_INTERVAL_MS) {
    lastSgpStart = millis();
    sgpMeasuring = startSgp40Reading();
  }
}

// Reads every sensor that is due. The SCD-41 is polled during the warm-up too, so its
// first value after the warm-up is a new one.
void readSensors() {
  uint32_t now = millis();
  if (scdFound && now - lastScdPoll >= SCD_POLL_INTERVAL_MS) {
    lastScdPoll = now;
    readScd41();
  }
  updateSgp40();
  if (warmingUp) return;
  if (shtFound && now - lastShtRead >= SHT_INTERVAL_MS) {
    lastShtRead = now;
    readSht45();
  }
  if (bmpFound && now - lastBmpRead >= BMP_INTERVAL_MS) {
    lastBmpRead = now;
    readBmp580();
  }
  if (vemlFound && now - lastVemlRead >= VEML_INTERVAL_MS) {
    lastVemlRead = now;
    readVeml7700();
  }
}

// Counts the warm-up down on the display and ends it after WARMUP_TIME_MS
void updateWarmup() {
  static uint8_t secondsShown = 0;
  if (!warmingUp) return;

  uint8_t left = warmupSecondsLeft();
  if (left > 0) {
    if (left != secondsShown) {
      secondsShown = left;
      screenChanged = true;
    }
    return;
  }

  warmingUp = false;
  Serial.println(F("Warm-up done, values are shown and saved from now on"));
  // First values at once (the SGP40 and SCD-41 follow within 1 s and 5 s)
  if (shtFound) {
    lastShtRead = millis();
    readSht45();
  }
  if (bmpFound) {
    lastBmpRead = millis();
    readBmp580();
  }
  if (vemlFound) {
    lastVemlRead = millis();
    readVeml7700();
  }
  displayWake();  // shows the first values for DISPLAY_ON_TIME_MS
}

// ---------- Wi-Fi ----------

void startWifi() {
  if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "your-wifi-name") == 0) {
    Serial.println(F("Wi-Fi: enter your network in secrets.h - no internet time, so no history"));
    return;
  }
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  if (USE_FIXED_IP && !WiFi.config(FIXED_IP, ROUTER_IP, SUBNET_MASK, ROUTER_IP)) {
    Serial.println(F("Wi-Fi: the fixed address could not be set, the router gives one instead"));
  }
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

// ---------- Serial Monitor ----------

void printStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi: connected to \"%s\", signal %d dBm, web page at http://%s/\n", WIFI_SSID, WiFi.RSSI(),
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println(F("Wi-Fi: not connected"));
  }
  Serial.printf("Wi-Fi MAC address: %s (%s)\n", WiFi.macAddress().c_str(),
                USE_FIXED_IP ? "fixed address" : "address from the router");
  Serial.printf("Parts: display %s, SCD-41 %s, SHT45 %s, BMP580 %s", displayFound ? "ok" : "MISSING",
                scdFound ? "ok" : "MISSING", shtFound ? "ok" : "MISSING", bmpFound ? "ok" : "MISSING");
  if (bmpFound) Serial.printf(" (0x%02X)", bmpAddress);
  Serial.printf(", SGP40 %s, VEML7700 %s\n", sgpFound ? "ok" : "MISSING", vemlFound ? "ok" : "MISSING");
  if (warmingUp) {
    Serial.printf("Warm-up: %u s left\n", warmupSecondsLeft());
  } else {
    Serial.printf("Now: %u ppm, %.2f C, %.1f %%RH, %.2f hPa, %.1f lux%s, VOC index %u (raw %u)\n", co2Ppm, temperatureC,
                  humidityPct, pressureHpa, lightLux, lightTooBright ? " (or more)" : "", vocIndex, vocRaw);
  }
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
      Serial.println(F("Use: csv m1|m5 [days], for example \"csv m1 1\""));
    }
  } else if (strcmp(command, "save") == 0) {
    Serial.println(historySave() ? F("History saved") : F("Not saved - see \"status\""));
  } else if (strcmp(command, "sleep") == 0 && option && strcmp(option, "on") == 0) {
    setSleepMode(true);
  } else if (strcmp(command, "sleep") == 0 && option && strcmp(option, "off") == 0) {
    setSleepMode(false);
  } else if (strcmp(command, "led") == 0) {
    testLed();
  } else {
    Serial.println(F("Commands: status, files, csv m1|m5 [days], save, sleep on|off, led"));
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

// ---------- Start and main loop ----------

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

  // Both LEDs keep their last colour through a reset (for example from BlinkRGB), so clear them
  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
  updateLed();

  displayFound = display.begin(OLED_ADDRESS);
  if (displayFound) {
    display.setRotation(OLED_ROTATION);
    display.setContrast(OLED_BRIGHTNESS);
    display.clearDisplay();
    printCentred("CO2", 3, 40);
    printCentred("starting...", 1, 76);
    display.display();
    displayOn = true;
    Serial.println(F("Display found"));
  } else {
    Serial.println(F("Display not found - check wiring and OLED_ADDRESS"));
  }

  shtFound = sht4.begin(&Wire);
  if (shtFound) {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT45 found"));
  } else {
    Serial.println(F("SHT45 not found - check wiring"));
  }

  bmpFound = initBmp580();
  if (bmpFound) {
    Serial.printf("BMP580 found at 0x%02X\n", bmpAddress);
  } else {
    Serial.println(F("BMP580 not found - check wiring"));
  }

  vemlFound = initVeml7700();
  Serial.println(vemlFound ? F("VEML7700 found") : F("VEML7700 not found - check wiring"));

  sgpFound = initSgp40();
  Serial.println(sgpFound ? F("SGP40 found, self-test passed") : F("SGP40 not found or self-test failed - check wiring"));

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
  readSensors();
  webLoop({co2Ppm, temperatureC, humidityPct, pressureHpa, lightLux, vocIndex, warmupSecondsLeft(), sleepMode});

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
    } else if (screenChanged && millis() - lastDraw >= DISPLAY_REDRAW_MS) {
      drawScreen();
    }
  }
}
