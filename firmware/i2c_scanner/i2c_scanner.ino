/*
  I2C scanner - lists every device on the I2C bus every 5 seconds.
  Use it to check the wiring before uploading the CO2 monitor sketch.

  Expected result with all parts connected:
    0x3C (or 0x3D)  OLED display
    0x44            SHT45
    0x62            SCD-41
*/

#include <Wire.h>

constexpr uint8_t I2C_SDA_PIN = 23;  // J3 pin 5
constexpr uint8_t I2C_SCL_PIN = 22;  // J3 pin 6

const char* deviceName(uint8_t address) {
  switch (address) {
    case 0x3C:
    case 0x3D: return "OLED display (SSD1306)";
    case 0x44: return "SHT45 temperature / humidity";
    case 0x62: return "SCD-41 CO2";
    default: return "unknown device";
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 2000) delay(10);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop() {
  Serial.println(F("Scanning I2C bus (SDA = GPIO23, SCL = GPIO22) ..."));
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X  %s\n", address, deviceName(address));
      found++;
    }
  }
  Serial.printf("%u device(s) found. Expected: 0x3C (or 0x3D), 0x44, 0x62\n\n", found);
  delay(5000);
}
