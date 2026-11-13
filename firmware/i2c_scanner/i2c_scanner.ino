/*
  I2C scanner and sensor check

  Every 5 seconds it lists every device on the I2C bus and reads each sensor
  it knows once, so you can check the wiring and every sensor before
  uploading the CO2 monitor sketch.

  Libraries (Library Manager, section 5.3): Sensirion I2C SCD4x,
  Adafruit SHT4x Library, Adafruit BMP5xx Library, Adafruit VEML7700 Library,
  Adafruit SGP40 Sensor.

  Expected with all parts of the case connected (6 devices):
    0x10         VEML7700 light sensor   light in lux
    0x3D         1.5" OLED (SSD1327)     only found, nothing is drawn
    0x44         SHT45                   temperature and humidity
    0x46 / 0x47  BMP580                  air pressure and temperature
    0x59         SGP40                   self-test once, then its raw signal
    0x62         SCD-41                  CO2, temperature and humidity

  The prototype's 0.96" OLED (SSD1306) shows up as 0x3C.
  Only the normal addresses 0x08-0x77 are scanned; the others are reserved.
  (The BMP580 also answers 0x7E, the I3C broadcast address, so scanners that
  check every address list it twice.)
  A sensor plugged in while the scanner runs is started in the next scan.
  The SCD-41 gives its first value one scan after it was started.
*/

#include <Wire.h>
#include <SensirionI2cScd4x.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_SGP40.h>

constexpr uint8_t I2C_SDA_PIN = 23;  // J3 pin 12 (pin 1 = USB end)
constexpr uint8_t I2C_SCL_PIN = 22;  // J3 pin 11
constexpr uint32_t SCAN_INTERVAL_MS = 5000;
constexpr uint8_t FIRST_ADDRESS = 0x08;  // 0x00-0x07 and 0x78-0x7F are reserved
constexpr uint8_t LAST_ADDRESS = 0x77;

constexpr uint8_t VEML7700_ADDRESS = 0x10;
constexpr uint8_t OLED_SMALL_ADDRESS = 0x3C;  // 0.96" SSD1306 (prototype)
constexpr uint8_t OLED_ADDRESS = 0x3D;        // 1.5" SSD1327 (case)
constexpr uint8_t SHT45_ADDRESS = 0x44;
constexpr uint8_t BMP580_ADDRESS_A = 0x46;  // the two addresses its
constexpr uint8_t BMP580_ADDRESS_B = 0x47;  // address jumper can choose
constexpr uint8_t SGP40_ADDRESS = 0x59;
constexpr uint8_t SCD41_ADDRESS = 0x62;

struct CasePart {
  const char* name;
  uint8_t address;
  uint8_t otherAddress;  // same as address if there is only one
};

const CasePart CASE_PARTS[] = {
  {"VEML7700 (0x10)", VEML7700_ADDRESS, VEML7700_ADDRESS},
  {"1.5\" OLED (0x3D)", OLED_ADDRESS, OLED_ADDRESS},
  {"SHT45 (0x44)", SHT45_ADDRESS, SHT45_ADDRESS},
  {"BMP580 (0x46 or 0x47)", BMP580_ADDRESS_A, BMP580_ADDRESS_B},
  {"SGP40 (0x59)", SGP40_ADDRESS, SGP40_ADDRESS},
  {"SCD-41 (0x62)", SCD41_ADDRESS, SCD41_ADDRESS},
};

SensirionI2cScd4x scd4x;
Adafruit_SHT4x sht4;
Adafruit_BMP5xx bmp;
Adafruit_VEML7700 veml;
Adafruit_SGP40 sgp;

bool found[128];

// Set once a sensor is set up; cleared when it drops off the bus or stops
// answering, so it is set up again when it comes back.
bool veml7700Started = false;
bool sht45Started = false;
bool bmp580Started = false;
bool sgp40Started = false;
bool scd41Started = false;

// The SHT45's values from this scan, for the SGP40's compensation
bool shtValid = false;
float shtTemperature = 25;
float shtHumidity = 50;

// ------------------------------------------------------------------- sensors

void readVeml7700() {
  if (!veml7700Started) {
    if (!veml.begin(&Wire)) {
      Serial.print(F("start failed"));
      return;
    }
    veml.setGain(VEML7700_GAIN_1);  // finer steps than the library's 1/8, for dim rooms
    veml7700Started = true;
  }
  const float lux = veml.readLux(VEML_LUX_CORRECTED);  // waits for a fresh 100 ms reading
  if (veml.readALS() == 0xFFFF) {  // the same reading again, raw
    Serial.print(F("over 3700 lux (too bright for this setting)"));
  } else {
    Serial.printf("%.1f lux", lux);
  }
}

void readSht45() {
  shtValid = false;
  if (!sht45Started) {
    if (!sht4.begin(&Wire)) {
      Serial.print(F("start failed"));
      return;
    }
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    sht45Started = true;
  }
  sensors_event_t humidity, temperature;
  if (!sht4.getEvent(&humidity, &temperature)) {
    Serial.print(F("read failed"));
    sht45Started = false;
    return;
  }
  shtTemperature = temperature.temperature;
  shtHumidity = humidity.relative_humidity;
  shtValid = true;
  Serial.printf("%.1f C  %.1f %%RH", shtTemperature, shtHumidity);
}

void readBmp580(uint8_t address) {
  if (!bmp580Started) {
    // Resets it, checks its chip ID and starts 50 measurements per second
    if (!bmp.begin(address, &Wire)) {
      Serial.print(F("start failed"));
      return;
    }
    bmp580Started = true;
    delay(50);  // wait for the first measurement
  }
  if (!bmp.performReading()) {
    Serial.print(F("read failed"));
    bmp580Started = false;
    return;
  }
  Serial.printf("%.2f hPa  %.1f C", bmp.pressure, bmp.temperature);
}

void readSgp40() {
  if (!sgp40Started) {
    // begin() ends with the sensor's self-test
    if (!sgp.begin(&Wire)) {
      Serial.print(F("start failed or self-test FAILED"));
      return;
    }
    Serial.print(F("self-test passed, "));
    sgp40Started = true;
  }
  // Raw signal, compensated with the SHT45's values (25 C and 50 %RH without it)
  const uint16_t raw = sgp.measureRaw(shtTemperature, shtHumidity);
  if (raw == 0) {
    Serial.print(F("read failed"));
    sgp40Started = false;
    return;
  }
  Serial.printf("raw signal %u", raw);
  if (!shtValid) Serial.print(F(" (no SHT45 values)"));
}

void readScd41() {
  if (!scd41Started) {
    // The CO2 monitor sketch may have left it measuring: stop, then restart.
    scd4x.stopPeriodicMeasurement();  // waits 500 ms
    if (scd4x.startPeriodicMeasurement() != 0) {
      Serial.print(F("no answer"));
      return;
    }
    scd41Started = true;
    Serial.print(F("started, first value in the next scan"));
    return;
  }
  // It has a new value every 5 s; wait up to 1 s for it.
  bool ready = false;
  for (uint8_t i = 0; i < 10 && !ready; i++) {
    if (scd4x.getDataReadyStatus(ready) != 0) break;
    if (!ready) delay(100);
  }
  if (!ready) {
    Serial.print(F("no new value"));
    return;
  }
  uint16_t co2;
  float temperature, humidity;
  if (scd4x.readMeasurement(co2, temperature, humidity) != 0) {
    Serial.print(F("read failed"));
    scd41Started = false;
    return;
  }
  Serial.printf("%u ppm  %.1f C  %.1f %%RH", co2, temperature, humidity);
}

// ---------------------------------------------------------------- the scan

const char* deviceName(uint8_t address) {
  switch (address) {
    case VEML7700_ADDRESS: return "VEML7700 light";
    case OLED_SMALL_ADDRESS: return "OLED 0.96\" (SSD1306)";
    case OLED_ADDRESS: return "OLED 1.5\" (SSD1327)";
    case SHT45_ADDRESS: return "SHT45 temperature / humidity";
    case BMP580_ADDRESS_A:
    case BMP580_ADDRESS_B: return "BMP580 air pressure";
    case SGP40_ADDRESS: return "SGP40 air quality (VOC)";
    case SCD41_ADDRESS: return "SCD-41 CO2";
    default: return "unknown device";
  }
}

void readDevice(uint8_t address) {
  switch (address) {
    case VEML7700_ADDRESS: readVeml7700(); break;
    case SHT45_ADDRESS: readSht45(); break;
    case BMP580_ADDRESS_A:
    case BMP580_ADDRESS_B: readBmp580(address); break;
    case SGP40_ADDRESS: readSgp40(); break;
    case SCD41_ADDRESS: readScd41(); break;
    default: break;  // displays and unknown devices are only listed
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) delay(10);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  scd4x.begin(Wire, SCD41_ADDRESS);
  scd4x.wakeUp();  // in case it was put to sleep; an error here is normal
}

void loop() {
  Serial.println(F("Scanning I2C bus (SDA = GPIO23, SCL = GPIO22) ..."));
  uint8_t count = 0;
  for (uint8_t address = FIRST_ADDRESS; address <= LAST_ADDRESS; address++) {
    Wire.beginTransmission(address);
    found[address] = Wire.endTransmission() == 0;
    if (found[address]) count++;
  }
  if (!found[VEML7700_ADDRESS]) veml7700Started = false;
  if (!found[SHT45_ADDRESS]) sht45Started = false;
  if (!found[BMP580_ADDRESS_A] && !found[BMP580_ADDRESS_B]) bmp580Started = false;
  if (!found[SGP40_ADDRESS]) sgp40Started = false;
  if (!found[SCD41_ADDRESS]) scd41Started = false;
  shtValid = false;
  shtTemperature = 25;
  shtHumidity = 50;

  // In address order, so the SHT45 (0x44) is read before the SGP40 (0x59)
  for (uint8_t address = FIRST_ADDRESS; address <= LAST_ADDRESS; address++) {
    if (!found[address]) continue;
    Serial.printf("  0x%02X  %-29s ", address, deviceName(address));
    readDevice(address);
    Serial.println();
  }

  Serial.printf("%u device(s) found. ", count);
  uint8_t missing = 0;
  for (const CasePart& part : CASE_PARTS) {
    if (found[part.address] || found[part.otherAddress]) continue;
    Serial.print(missing == 0 ? F("Not found: ") : F(", "));
    Serial.print(part.name);
    missing++;
  }
  if (missing == 0) Serial.print(F("All 6 parts of the case are there."));
  Serial.println();
  Serial.println();
  delay(SCAN_INTERVAL_MS);
}
