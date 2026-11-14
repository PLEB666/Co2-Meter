/*
  Web page - live values, graphs and CSV downloads in a browser on the home network

  Open http://<address of the monitor>/ - the Serial Monitor prints the address when
  Wi-Fi connects. The address is set in co2_monitor.ino (FIXED_IP).

  The page is written in web/index.html. web_page.h holds a compressed copy that is
  built into the firmware; make it again with web/make_web_page.ps1 after a change.

  The page gets its data from these addresses (all times are UTC seconds):
    /api/now                              live values, warm-up, sleep mode, brightness, Wi-Fi signal (JSON)
                                          (voc is not on the page yet)
    /api/info                             state of the history (JSON)
    /api/days?from=..&to=..               which days have values, for the calendar (JSON)
    /api/data?res=1|5&from=..&to=..      stored values as binary records (history.h)
    /api/sleep?on=1|0                     POST: turns sleep mode on or off (JSON)
    /api/brightness?auto=1                POST: the light sensor sets the brightness (JSON)
    /api/brightness?level=0..100          POST: a fixed brightness in %, Auto off (JSON)
*/
#pragma once

#include <Arduino.h>

// For the brightness setter: follow the light sensor instead of a fixed level
constexpr int BRIGHTNESS_AUTO = -1;

// The current readings, handed over by the main program in every loop
struct LiveValues {
  uint16_t co2Ppm;            // 0 = no value yet
  float temperatureC;         // NAN = no value
  float humidityPct;          // NAN = no value
  float pressureHpa;          // NAN = no value
  float lightLux;             // NAN = no value
  uint16_t vocIndex;          // 0 = no value yet
  uint8_t warmupSecondsLeft;  // 0 = warm-up over
  bool sleepMode;             // LED off, moon on the display
  uint8_t brightness;         // 0-100 %, of the display and the LED
  bool autoBrightness;        // true = the light sensor sets it
};

// Call when Wi-Fi has connected (more calls do nothing). The CO2 limits give the
// value on the page the same rating as on the display. setSleepMode is called when
// the sleep mode switch on the page is used, setBrightness when the brightness slider
// or its Auto switch is: with BRIGHTNESS_AUTO or a level 0-100, and it returns the
// level now in use.
void webBegin(uint16_t co2GoodBelow, uint16_t co2OkBelow, uint16_t co2VentilateBelow, void (*setSleepMode)(bool on),
              uint8_t (*setBrightness)(int level));

// Call in loop(). Answers the browser's requests.
void webLoop(const LiveValues &values);
