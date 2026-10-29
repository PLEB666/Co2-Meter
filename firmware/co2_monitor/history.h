/*
  History - long-term storage of the measured values in the ESP32's flash

  Stored with LittleFS on the 1.5 MB data partition of the partition scheme
  "8M with spiffs (3MB APP/1.5MB SPIFFS)". Firmware uploads don't touch it.

  Three levels of detail, each in its own folder:
    /m1/YYYYMMDD.bin  1-minute averages, one file per day      kept 7 days
    /m5/YYYYMMDD.bin  5-minute averages, one file per day      kept 90 days
    /h1/YYYYMM.bin    hourly average, minimum and maximum,     kept until the flash is
                      one file per month                       full (about 3.5 years)

  - All times are UTC. The web page converts them to local time.
  - The 1-minute values wait in RAM and are saved every 10 minutes. The 5-minute
    and hourly values are then calculated from the saved 1-minute values, so a
    power cut never leaves a hole in them.
  - Records are only ever appended to a file, never changed. Old files are deleted.
*/
#pragma once

#include <Arduino.h>

// Call once in setup(). Mounts the data partition (formats it on first use).
void historyBegin();

// Call for every new reading. Pass NAN for a value that is missing.
void historyAddSample(uint16_t co2Ppm, float temperatureC, float humidityPct, float pressureHpa);

// Call in loop(). Finishes each minute and saves every 10 minutes.
void historyLoop();

// Saves the values waiting in RAM now. Returns false if nothing could be saved
// (history off, clock not set yet, or a write failed).
bool historySave();

// Serial Monitor helpers
void historyPrintStatus(Print &out);
void historyPrintFiles(Print &out);
// level: "m1", "m5" or "h1". Returns false for an unknown level.
bool historyPrintCsv(Print &out, const char *level, uint16_t days);

// ---------- Web page helpers (web.cpp) ----------
// All times are UTC seconds. minutes is 1, 5 or 60.

// True once the internet time has arrived
bool historyClockSet();

// Writes the stored values from "from" (included) to "to" (not included) as binary
// records, exactly as they are in the files: 12 bytes per 1- or 5-minute value,
// 26 bytes per hourly value (layouts in history.cpp). The 1-minute values include
// the ones not saved yet. Returns false for an unknown number of minutes.
bool historyWriteRecords(Print &out, uint8_t minutes, uint32_t from, uint32_t to);

// Writes a JSON list of the UTC days between from and to that have values:
// [{"day":..,"res":finest minutes stored,"first":..,"last":..}, ...]
// first and last are the start times of the first and last value of that day.
void historyWriteDaysJson(Print &out, uint32_t from, uint32_t to);

// Writes a JSON object with the state of the history: on, clock, flash use,
// minutes not saved yet, the start of the oldest file and the days kept per level
void historyWriteInfoJson(Print &out);
