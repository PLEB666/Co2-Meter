/*
  SCD-41 forced recalibration (FRC) tool - OPTIONAL

  Only needed if the CO2 reading is clearly wrong in fresh air (for example
  550 ppm outdoors). Normally the sensor's automatic self-calibration (ASC)
  keeps it correct on its own.

  How to use:
    1. Upload this sketch, with the same board settings as co2_monitor.
    2. Take the whole monitor outdoors (shade, no wind gusts, nobody breathing
       near it), powered from a USB power bank or a laptop.
    3. Wait 5 minutes. The display then shows "press the button".
       The "Spread 1 min" value should be small (under ~20 ppm) = stable air.
    4. Press the button (or send "c" in the Serial Monitor).
    5. The SCD-41 is corrected to FRC_TARGET_PPM. The correction is stored
       inside the sensor automatically and survives power cycles.
    6. Upload co2_monitor again.

  If the BMP580 is connected, the SCD-41 corrects for the air pressure it measures, as in
  co2_monitor. The correction FRC stores then fits the main program. Without it, set
  SENSOR_ALTITUDE_M.

  Display: the case's 1.5" 128x128 OLED (SSD1327), turned as in co2_monitor, at full
  brightness for daylight. The 8 mm LED and the board's own LED stay off.
  The stored history stays in the flash; nothing new is recorded while this sketch runs.

  Libraries (Arduino IDE -> Library Manager):
    Adafruit SSD1327, Sensirion I2C SCD4x, Adafruit BMP5xx Library
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include <SensirionI2cScd4x.h>
#include <Adafruit_BMP5xx.h>

constexpr uint8_t I2C_SDA_PIN = 23;  // J3 pin 12 (pin 1 = USB end)
constexpr uint8_t I2C_SCL_PIN = 22;  // J3 pin 11
constexpr uint8_t BUTTON_PIN = 2;    // J1 pin 5, the other side of the button goes to GND
constexpr uint8_t LED_PIN = 18;      // J3 pin 7, DIN of the 8 mm LED (only switched off here)
constexpr uint8_t OLED_ADDRESS = 0x3D;  // same values as in co2_monitor
constexpr uint8_t OLED_ROTATION = 3;
constexpr uint8_t OLED_SIZE = 128;      // 128 x 128 pixels
constexpr uint8_t OLED_CONTRAST = 255;  // full brightness (the library starts at 47)

// CO2 in outdoor fresh air. The global background level is about 420-430 ppm (2026).
constexpr uint16_t FRC_TARGET_PPM = 425;
constexpr uint16_t SENSOR_ALTITUDE_M = 0;              // only used without a BMP580
constexpr float SCD_PRESSURE_STEP_HPA = 1.0f;          // same value as in co2_monitor
constexpr uint32_t BMP_INTERVAL_MS = 2000;
constexpr uint32_t MIN_RUN_TIME_MS = 5UL * 60 * 1000;  // datasheet asks for at least 3 minutes
constexpr uint8_t HISTORY_SIZE = 12;                   // 12 readings x 5 s = last minute

enum class FrcResult { None, Done, Failed };

Adafruit_SSD1327 display(OLED_SIZE, OLED_SIZE, &Wire, -1);
SensirionI2cScd4x scd4x;
Adafruit_BMP5xx bmp;

bool displayFound = false;
bool bmpFound = false;
float pressureHpa = NAN;
float scdPressureHpa = NAN;  // the pressure the SCD-41 corrects for (NAN = from the altitude)
uint32_t measuringSince = 0;
uint16_t co2Ppm = 0;
uint16_t history[HISTORY_SIZE];
uint8_t historyCount = 0;
uint8_t historyIndex = 0;
FrcResult lastResult = FrcResult::None;
int16_t lastCorrection = 0;
bool calibrating = false;  // FRC is running (about 1 s)

// Drawing the display holds the program for about 0.2 s, and a quick press can begin and
// end within that time, unseen by buttonPressed(). So an interrupt watches the pin as well
// and notes a press that held it low for at least 20 ms (shorter = a bounce or noise).
// The same as in co2_monitor.
volatile bool buttonLow = false;       // the interrupt saw the pin go low
volatile uint32_t buttonLowSince = 0;  // micros()
volatile bool buttonPressSeen = false;

void onButtonChange() {
  uint32_t now = micros();
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonLow = true;
    buttonLowSince = now;
  } else if (buttonLow) {
    buttonLow = false;
    if (now - buttonLowSince >= 20000) buttonPressSeen = true;
  }
}

// Debounced, the button pulls the pin to GND. true once per press, as it goes down (or
// after a whole press the interrupt saw while the display was drawn).
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
    if (stableState == LOW) return true;
    buttonPressSeen = false;  // this press has been seen here
    return false;
  }
  if (stableState == HIGH && reading == HIGH && buttonPressSeen && millis() - lastChange > 30) {
    buttonPressSeen = false;
    return true;
  }
  return false;
}

// Both LEDs keep their last colour through an upload (for example the CO2 colour from
// co2_monitor), so they are switched off. The 8 mm LED (PL9823) takes slower bits than
// rgbLedWrite() sends, so the RMT sends it 24 zero bits directly, as in co2_monitor:
// 20 MHz, a 0 is 0.35 us high and 1.35 us low.
void ledsOff() {
  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
  if (!rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000)) {
    Serial.println(F("LED: RMT start failed on LED_PIN"));
    return;
  }
  rmt_data_t bits[24];
  for (rmt_data_t &bit : bits) {
    bit.level0 = 1;
    bit.duration0 = 7;
    bit.level1 = 0;
    bit.duration1 = 27;
  }
  rmtWrite(LED_PIN, bits, RMT_SYMBOLS_OF(bits), RMT_WAIT_FOR_EVER);
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

// The BMP580's address depends on its jumper, so both are tried
bool initBmp580() {
  for (uint8_t address : {0x46, 0x47}) {
    if (bmp.begin(address, &Wire)) {
      Serial.printf("BMP580 found at 0x%02X\n", address);
      return true;
    }
  }
  Serial.printf("BMP580 not found - the SCD-41 uses the altitude (%u m)\n", SENSOR_ALTITUDE_M);
  return false;
}

// Reads the pressure and sends it to the SCD-41 when it has moved by SCD_PRESSURE_STEP_HPA
// (the SCD-41 takes whole hPa, 700-1200, also while it measures)
void updatePressure() {
  pressureHpa = bmp.performReading() ? bmp.pressure : NAN;
  if (isnan(pressureHpa) || pressureHpa < 700 || pressureHpa > 1200) return;
  if (!isnan(scdPressureHpa) && fabsf(pressureHpa - scdPressureHpa) < SCD_PRESSURE_STEP_HPA) return;
  uint16_t hPa = (uint16_t)lroundf(pressureHpa);
  if (scd4x.setAmbientPressureRaw(hPa) == 0) {
    scdPressureHpa = hPa;
    Serial.printf("SCD-41 corrects for %u hPa (BMP580)\n", hPa);
  }
}

void startMeasuring() {
  scd4x.startPeriodicMeasurement();
  measuringSince = millis();
  co2Ppm = 0;
  historyCount = 0;
  historyIndex = 0;
}

// ---------- Display ----------

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

void printCentred(const char *text, uint8_t size, int16_t y, uint8_t colour = SSD1327_WHITE) {
  printAt(text, size, (OLED_SIZE - textWidth(text, size)) / 2, y, colour);
}

void printRight(const char *text, uint8_t size, int16_t y) {
  printAt(text, size, OLED_SIZE - textWidth(text, size), y);
}

// One line of the table: the name on the left, the value on the right
void printLine(int16_t y, const char *name, const char *value) {
  printAt(name, 1, 0, y);
  printRight(value, 1, y);
}

/*
  128 x 128 pixels:
    y   1  "CO2 ppm"                          "calibration"
    y  10  ------------------------------------------------
    y  20               CO2 value, large
    y  64  ------------------------------------------------
    y  69  Spread 1 min                              12 ppm
    y  80  Pressure                               983.1 hPa   (without a BMP580: Altitude 0 m)
    y  91  Target                                   425 ppm
    y 102  Last FRC                                 +12 ppm   (or "failed", "none yet")
    y 113               wait 4:32, large
           once ready: "press the button", black on a white bar
*/
void drawScreen() {
  char text[24];
  display.clearDisplay();

  printAt("CO2 ppm", 1, 0, 1);
  printRight("calibration", 1, 1);
  display.drawFastHLine(0, 10, OLED_SIZE, SSD1327_WHITE);

  // CO2 value, large and centred
  if (co2Ppm > 0) {
    snprintf(text, sizeof text, "%u", co2Ppm);
    if (strlen(text) <= 4) {
      printCentred(text, 5, 20);  // 30 x 40 pixels per digit
    } else {
      printCentred(text, 4, 23);  // 10000 ppm and more: 5 digits only fit smaller
    }
  } else {
    printCentred("measuring...", 1, 34);  // the first value comes within 5 s
  }

  display.drawFastHLine(0, 64, OLED_SIZE, SSD1327_WHITE);

  if (historyCount == HISTORY_SIZE) {
    snprintf(text, sizeof text, "%u ppm", spreadPpm());
  } else {
    strcpy(text, "--");
  }
  printLine(69, "Spread 1 min", text);

  if (!bmpFound) {
    snprintf(text, sizeof text, "%u m", SENSOR_ALTITUDE_M);
    printLine(80, "Altitude", text);
  } else {
    if (isnan(pressureHpa)) {
      strcpy(text, "--");
    } else {
      snprintf(text, sizeof text, "%.1f hPa", pressureHpa);
    }
    printLine(80, "Pressure", text);  // a longer name would run into "1013.2 hPa"
  }

  snprintf(text, sizeof text, "%u ppm", FRC_TARGET_PPM);
  printLine(91, "Target", text);

  if (lastResult == FrcResult::Done) {
    snprintf(text, sizeof text, "%+d ppm", lastCorrection);
  } else if (lastResult == FrcResult::Failed) {
    strcpy(text, "failed");  // measuring starts again; press again after the countdown
  } else {
    strcpy(text, "none yet");
  }
  printLine(102, "Last FRC", text);

  // Bottom: the countdown, or a white bar when the button may be pressed
  if (calibrating) {
    printCentred("calibrating", 1, 117);
  } else if (readyForFrc()) {
    display.fillRect(0, 112, OLED_SIZE, 16, SSD1327_WHITE);
    printCentred("press the button", 1, 116, SSD1327_BLACK);
  } else {
    uint32_t secondsLeft = (MIN_RUN_TIME_MS - (millis() - measuringSince) + 999) / 1000;
    snprintf(text, sizeof text, "wait %u:%02u", (unsigned)(secondsLeft / 60), (unsigned)(secondsLeft % 60));
    printCentred(text, 2, 113);
  }

  display.display();
}

// A short message in the middle of the screen, for errors at the start
void drawMessage(const char *line1, const char *line2) {
  display.clearDisplay();
  printCentred(line1, 1, 52);
  printCentred(line2, 1, 66);
  display.display();
}

void performFrc() {
  Serial.printf("Running forced recalibration to %u ppm ...\n", FRC_TARGET_PPM);
  calibrating = true;
  if (displayFound) drawScreen();
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
  calibrating = false;
  startMeasuring();
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) delay(10);
  Serial.println(F("\nSCD-41 forced recalibration tool"));

  ledsOff();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonChange, CHANGE);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  displayFound = display.begin(OLED_ADDRESS);
  if (displayFound) {
    display.setRotation(OLED_ROTATION);
    display.setTextWrap(false);  // as in co2_monitor: text that ends at the right edge stays on its line
    display.setContrast(OLED_CONTRAST);
    drawMessage("FRC calibration", "starting...");
  } else {
    Serial.println(F("Display not found - check wiring and OLED_ADDRESS"));
  }

  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  scd4x.wakeUp();  // returns an error if the sensor was not asleep - that's fine
  scd4x.stopPeriodicMeasurement();
  scd4x.reinit();

  uint64_t serialNumber = 0;
  if (scd4x.getSerialNumber(serialNumber) != 0) {
    Serial.println(F("SCD-41 not found - check wiring"));
    if (displayFound) drawMessage("SCD-41 not found", "check the wiring");
    while (true) delay(1000);
  }

  uint16_t ascEnabled = 0;
  scd4x.getAutomaticSelfCalibrationEnabled(ascEnabled);
  Serial.printf("ASC is %s\n", ascEnabled ? "enabled" : "disabled");
  scd4x.setSensorAltitude(SENSOR_ALTITUDE_M);
  bmpFound = initBmp580();
  if (bmpFound) updatePressure();  // replaces the altitude

  startMeasuring();
  Serial.printf("Put the sensor in fresh outdoor air. After 5 minutes press the button "
                "or send 'c' to calibrate to %u ppm.\n", FRC_TARGET_PPM);
}

void loop() {
  static uint32_t lastPoll = 0;
  static uint32_t lastDraw = 0;
  static uint32_t lastBmpRead = 0;

  if (bmpFound && millis() - lastBmpRead >= BMP_INTERVAL_MS) {
    lastBmpRead = millis();
    updatePressure();
  }

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
      if (!isnan(pressureHpa)) Serial.printf("  %.1f hPa", pressureHpa);
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
