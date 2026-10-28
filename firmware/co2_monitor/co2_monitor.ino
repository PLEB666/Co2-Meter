/*
  CO2 Monitor - step 1
  ESP32-C6-DevKitC-1 + Adafruit SHT45 + Adafruit SCD-41 + Joy-it 0.96" OLED (SSD1306)

  - SHT45  -> temperature and humidity (more accurate than the SCD-41's own values)
  - SCD-41 -> CO2 in periodic mode (a new value every 5 s)
  - The OLED switches off after 30 s. The button on GPIO2 wakes it for another 30 s.
  - The RGB LED on the board shows the CO2 level all the time, dimmed:
    green -> yellow -> red.
  - All readings are also printed at 115200 baud (Serial Monitor / Serial Plotter).

  The sensors keep measuring while the display is off. This matters for the
  SCD-41's automatic self-calibration, which needs uninterrupted operation.

  Libraries (Arduino IDE -> Library Manager):
    Adafruit SSD1306, Adafruit SHT4x Library, Sensirion I2C SCD4x
  Board: "ESP32C6 Dev Module" (esp32 by Espressif Systems)
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT4x.h>
#include <SensirionI2cScd4x.h>

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

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_SHT4x sht4;
SensirionI2cScd4x scd4x;

bool displayFound = false;
bool shtFound = false;
bool scdFound = false;

bool displayOn = false;
uint32_t displayOnSince = 0;

float temperatureC = NAN;
float humidityPct = NAN;
uint16_t co2Ppm = 0;  // 0 = no valid reading yet
bool newData = false;

uint32_t lastShtRead = 0;
uint32_t lastScdPoll = 0;

// Returns true once per button press (debounced, button pulls the pin to GND).
bool buttonPressed() {
  static bool stableState = HIGH;
  static bool lastReading = HIGH;
  static uint32_t lastChange = 0;

  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) {
    lastReading = reading;
    lastChange = millis();
  }
  if (millis() - lastChange > 30 && reading != stableState) {
    stableState = reading;
    return stableState == LOW;
  }
  return false;
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

// LED off until the first CO2 value arrives
void updateLed() {
  LedColour colour = co2Ppm > 0 ? co2Colour(co2Ppm) : LedColour{0, 0, 0};
  rgbLedWrite(RGB_BUILTIN, dimmed(colour.r), dimmed(colour.g), dimmed(colour.b));
}

void drawScreen() {
  char text[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Top line: label left, rating right (inverted when the air is bad)
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("CO2 ppm"));
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
  if (co2Ppm > 0) {
    snprintf(text, sizeof text, "%u", co2Ppm);
    display.setTextSize(3);
    display.setCursor((OLED_WIDTH - strlen(text) * 18) / 2, 15);
    display.print(text);
  } else {
    display.setTextSize(1);
    display.setCursor(22, 24);
    display.print(scdFound ? F("warming up...") : F("SCD-41 missing"));
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

void displaySleep() {
  if (!displayFound) return;
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  displayOn = false;
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

  co2Ppm = co2;
  newData = true;
  updateLed();

  // "name:value" pairs can be read by people and by the Serial Plotter
  Serial.printf("CO2_ppm:%u Temperature_C:%.2f Humidity_pct:%.1f\n", co2Ppm, temperatureC, humidityPct);
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) delay(10);  // don't hang when no PC is connected
  Serial.println(F("\nCO2 monitor starting"));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

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
    readSht45();
  } else {
    Serial.println(F("SHT45 not found - check wiring"));
  }

  scdFound = initScd41();
  if (scdFound) {
    Serial.println(F("SCD-41 measuring, first CO2 value in about 5 s"));
  } else {
    Serial.println(F("SCD-41 not found - check wiring"));
  }

  displayWake();
}

void loop() {
  if (buttonPressed()) displayWake();

  if (shtFound && millis() - lastShtRead >= SHT_INTERVAL_MS) {
    lastShtRead = millis();
    readSht45();
  }
  if (scdFound && millis() - lastScdPoll >= SCD_POLL_INTERVAL_MS) {
    lastScdPoll = millis();
    readScd41();
  }

  if (displayOn) {
    if (millis() - displayOnSince >= DISPLAY_ON_TIME_MS) {
      displaySleep();
    } else if (newData) {
      drawScreen();
    }
  }
  newData = false;
}
