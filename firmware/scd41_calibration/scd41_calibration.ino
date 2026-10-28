/*
  SCD-41 forced recalibration (FRC) tool - OPTIONAL

  Only needed if the CO2 reading is clearly wrong in fresh air (for example
  550 ppm outdoors). Normally the sensor's automatic self-calibration (ASC)
  keeps it correct on its own.

  How to use:
    1. Upload this sketch.
    2. Take the whole setup outdoors (shade, no wind gusts, nobody breathing
       near it), powered from a USB power bank or a laptop.
    3. Wait 5 minutes. The display then shows "press button".
       The "spread" value should be small (under ~20 ppm) = stable air.
    4. Press the button (or send "c" in the Serial Monitor).
    5. The SCD-41 is corrected to FRC_TARGET_PPM. The correction is stored
       inside the sensor automatically and survives power cycles.
    6. Upload co2_monitor again.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SensirionI2cScd4x.h>

constexpr uint8_t I2C_SDA_PIN = 23;  // J3 pin 5
constexpr uint8_t I2C_SCL_PIN = 22;  // J3 pin 6
constexpr uint8_t BUTTON_PIN = 2;    // J1 pin 12
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint8_t OLED_ROTATION = 0;  // same value as in co2_monitor

// CO2 in outdoor fresh air. The global background level is about 420-430 ppm (2026).
constexpr uint16_t FRC_TARGET_PPM = 425;
constexpr uint16_t SENSOR_ALTITUDE_M = 0;              // same value as in co2_monitor
constexpr uint32_t MIN_RUN_TIME_MS = 5UL * 60 * 1000;  // datasheet asks for at least 3 minutes
constexpr uint8_t HISTORY_SIZE = 12;                   // 12 readings x 5 s = last minute

enum class FrcResult { None, Done, Failed };

Adafruit_SSD1306 display(128, 64, &Wire, -1);
SensirionI2cScd4x scd4x;

bool displayFound = false;
uint32_t measuringSince = 0;
uint16_t co2Ppm = 0;
uint16_t history[HISTORY_SIZE];
uint8_t historyCount = 0;
uint8_t historyIndex = 0;
FrcResult lastResult = FrcResult::None;
int16_t lastCorrection = 0;

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

bool readyForFrc() {
  return millis() - measuringSince >= MIN_RUN_TIME_MS;
}

// Difference between the highest and lowest reading of the last minute
uint16_t spreadPpm() {
  uint16_t lowest = 0xFFFF;
  uint16_t highest = 0;
  for (uint8_t i = 0; i < historyCount; i++) {
    lowest = min(lowest, history[i]);
    highest = max(highest, history[i]);
  }
  return highest - lowest;
}

void startMeasuring() {
  scd4x.startPeriodicMeasurement();
  measuringSince = millis();
  co2Ppm = 0;
  historyCount = 0;
  historyIndex = 0;
}

void performFrc() {
  Serial.printf("Running forced recalibration to %u ppm ...\n", FRC_TARGET_PPM);
  scd4x.stopPeriodicMeasurement();  // the library waits the required 500 ms
  uint16_t correction = 0xFFFF;
  int16_t error = scd4x.performForcedRecalibration(FRC_TARGET_PPM, correction);
  if (error != 0 || correction == 0xFFFF) {
    lastResult = FrcResult::Failed;
    Serial.println(F("FRC FAILED - let the sensor run longer and try again"));
  } else {
    lastResult = FrcResult::Done;
    lastCorrection = (int16_t)((int32_t)correction - 0x8000);
    Serial.printf("FRC done, correction %+d ppm. It is stored in the sensor automatically.\n", lastCorrection);
  }
  startMeasuring();
}

void drawScreen() {
  char text[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("FRC calibration"));

  display.setTextSize(2);
  display.setCursor(0, 12);
  if (co2Ppm > 0) {
    snprintf(text, sizeof text, "%u ppm", co2Ppm);
    display.print(text);
  } else {
    display.print(F("---- ppm"));
  }

  display.setTextSize(1);
  display.setCursor(0, 33);
  if (historyCount == HISTORY_SIZE) {
    snprintf(text, sizeof text, "spread 1 min: %u ppm", spreadPpm());
  } else {
    strcpy(text, "spread 1 min: --");
  }
  display.print(text);

  display.setCursor(0, 44);
  if (lastResult == FrcResult::Done) {
    snprintf(text, sizeof text, "Done! corr %+d ppm", lastCorrection);
  } else if (lastResult == FrcResult::Failed) {
    strcpy(text, "FRC failed, retry");
  } else {
    snprintf(text, sizeof text, "target: %u ppm", FRC_TARGET_PPM);
  }
  display.print(text);

  display.setCursor(0, 56);
  if (readyForFrc()) {
    snprintf(text, sizeof text, "press button -> %u", FRC_TARGET_PPM);
  } else {
    uint32_t secondsLeft = (MIN_RUN_TIME_MS - (millis() - measuringSince)) / 1000;
    snprintf(text, sizeof text, "wait %lu:%02lu", secondsLeft / 60, secondsLeft % 60);
  }
  display.print(text);

  display.display();
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) delay(10);
  Serial.println(F("\nSCD-41 forced recalibration tool"));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  displayFound = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false);
  if (displayFound) display.setRotation(OLED_ROTATION);

  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  scd4x.wakeUp();
  scd4x.stopPeriodicMeasurement();
  scd4x.reinit();

  uint64_t serialNumber = 0;
  if (scd4x.getSerialNumber(serialNumber) != 0) {
    Serial.println(F("SCD-41 not found - check wiring"));
    if (displayFound) {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print(F("SCD-41 not found"));
      display.display();
    }
    while (true) delay(1000);
  }

  uint16_t ascEnabled = 0;
  scd4x.getAutomaticSelfCalibrationEnabled(ascEnabled);
  Serial.printf("ASC is %s\n", ascEnabled ? "enabled" : "disabled");
  scd4x.setSensorAltitude(SENSOR_ALTITUDE_M);

  startMeasuring();
  Serial.printf("Put the sensor in fresh outdoor air. After 5 minutes press the button "
                "or send 'c' to calibrate to %u ppm.\n", FRC_TARGET_PPM);
}

void loop() {
  static uint32_t lastPoll = 0;
  static uint32_t lastDraw = 0;

  if (millis() - lastPoll >= 1000) {
    lastPoll = millis();
    bool dataReady = false;
    uint16_t co2 = 0;
    float temperature = 0;
    float humidity = 0;
    if (scd4x.getDataReadyStatus(dataReady) == 0 && dataReady &&
        scd4x.readMeasurement(co2, temperature, humidity) == 0 && co2 != 0) {
      co2Ppm = co2;
      history[historyIndex] = co2;
      historyIndex = (historyIndex + 1) % HISTORY_SIZE;
      if (historyCount < HISTORY_SIZE) historyCount++;

      Serial.printf("t=%4lus  CO2 %u ppm", (millis() - measuringSince) / 1000, co2Ppm);
      if (historyCount == HISTORY_SIZE) Serial.printf("  spread(1 min) %u ppm", spreadPpm());
      Serial.println();
    }
  }

  bool trigger = buttonPressed();
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') trigger = true;
  }
  if (trigger) {
    if (readyForFrc()) {
      performFrc();
    } else {
      Serial.println(F("Not yet - the sensor has to run for 5 minutes first"));
    }
  }

  if (displayFound && millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawScreen();
  }
}
