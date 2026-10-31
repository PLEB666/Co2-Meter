/*
  Web page - live values, graphs and CSV downloads in a browser on the home network

  Open http://<address of the monitor>/ - the Serial Monitor prints the address when
  Wi-Fi connects. Give the monitor a fixed address in your router so it doesn't change.

  The page is written in web/index.html. web_page.h holds a compressed copy that is
  built into the firmware; make it again with web/make_web_page.ps1 after a change.

  The page gets its data from these addresses (all times are UTC seconds):
    /api/now                              live values, warm-up, sleep mode, Wi-Fi signal (JSON)
    /api/info                             state of the history (JSON)
    /api/days?from=..&to=..               which days have values, for the calendar (JSON)
    /api/data?res=1|5|60&from=..&to=..    stored values as binary records (history.h)
    /api/sleep?on=1|0                     POST: turns sleep mode on or off (JSON)
*/
#pragma once

#include <Arduino.h>

// The current readings, handed over by the main program in every loop
struct LiveValues {
  uint16_t co2Ppm;            // 0 = no value yet
  float temperatureC;         // NAN = no value
  float humidityPct;          // NAN = no value
  uint8_t warmupSecondsLeft;  // 0 = warm-up over
  bool sleepMode;             // LED off, moon on the display
};

// Call when Wi-Fi has connected (more calls do nothing). The CO2 limits give the
// value on the page the same rating as on the display. setSleepMode is called when
// the sleep mode switch on the page is used.
void webBegin(uint16_t co2GoodBelow, uint16_t co2OkBelow, uint16_t co2VentilateBelow, void (*setSleepMode)(bool on));

// Call in loop(). Answers the browser's requests.
void webLoop(const LiveValues &values);
