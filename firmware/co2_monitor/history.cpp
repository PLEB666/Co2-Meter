/*
  History - see history.h for the overview.

  Each file starts with a 16-byte header, followed by fixed-size records in
  time order. Numbers are little-endian (the ESP32's own byte order).
*/
#include "history.h"

#include <LittleFS.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <time.h>

// ---------- Settings ----------
constexpr uint16_t MINUTE_KEEP_DAYS = 7;
constexpr uint16_t FIVE_MINUTE_KEEP_DAYS = 90;
// The 1-minute values are saved every 10 minutes (at :00, :10, ... plus a few
// seconds). A power cut loses at most this much. Saving more often wears the flash faster.
constexpr uint32_t SAVE_INTERVAL_S = 600;
constexpr uint32_t SAVE_DELAY_S = 5;
// RAM for 1-minute values that are not saved yet: 24 hours. It only fills up while
// the clock is not set, for example when the internet is down after a restart.
constexpr uint16_t PENDING_MAX = 1440;
// When less than this is free, the oldest hourly month is deleted
constexpr uint32_t MIN_FREE_BYTES = 64 * 1024;

// The data partition of "8M with spiffs (3MB APP/1.5MB SPIFFS)". With any other
// partition scheme nothing is recorded, so a wrong flash area is never formatted.
constexpr char PARTITION_LABEL[] = "spiffs";
constexpr uint32_t PARTITION_ADDRESS = 0x670000;
constexpr uint32_t PARTITION_SIZE = 0x180000;

// Clock values before 1 Jan 2026 mean "not set yet"
constexpr time_t CLOCK_VALID_AFTER = 1767225600;
constexpr uint32_t SECONDS_PER_DAY = 86400;

// ---------- File format (version 1) ----------
constexpr uint8_t FORMAT_VERSION = 1;
constexpr char FORMAT_MAGIC[4] = {'C', 'O', '2', 'H'};

// Markers for a missing value
constexpr uint16_t NO_CO2 = 0;
constexpr int16_t NO_TEMPERATURE = INT16_MIN;
constexpr uint16_t NO_HUMIDITY = UINT16_MAX;
constexpr uint16_t NO_PRESSURE = 0;

struct __attribute__((packed)) FileHeader {
  char magic[4];        // "CO2H"
  uint8_t version;      // FORMAT_VERSION
  uint8_t minutes;      // minutes per record: 1, 5 or 60
  uint16_t recordSize;  // bytes per record
  uint32_t created;     // UTC seconds
  uint32_t reserved;    // 0
};

// 1-minute and 5-minute averages
struct __attribute__((packed)) SampleRecord {
  uint32_t time;        // UTC seconds at the start of the period
  uint16_t co2;         // ppm
  int16_t temperature;  // 0.01 °C
  uint16_t humidity;    // 0.01 %RH
  uint16_t pressure;    // 0.1 hPa (BMP390, planned)
};

// Hourly values
struct __attribute__((packed)) HourRecord {
  uint32_t time;  // UTC seconds at the start of the hour
  uint16_t co2Avg, co2Min, co2Max;
  int16_t temperatureAvg, temperatureMin, temperatureMax;
  uint16_t humidityAvg, humidityMin, humidityMax;
  uint16_t pressureAvg;
  uint8_t minutes;  // number of 1-minute values in this hour (60 = complete)
  uint8_t flags;    // reserved, 0
};

static_assert(sizeof(FileHeader) == 16, "FileHeader must be 16 bytes");
static_assert(sizeof(SampleRecord) == 12, "SampleRecord must be 12 bytes");
static_assert(sizeof(HourRecord) == 26, "HourRecord must be 26 bytes");

struct Level {
  const char *name;         // folder name, also used by the csv command
  const char *description;  // for the status output
  uint8_t minutes;          // minutes per record
  uint16_t recordSize;
  uint16_t keepDays;        // 0 = until the flash is full
  bool monthFiles;          // one file per month instead of one per day
};

const Level MINUTE_LEVEL = {"m1", "1-minute values", 1, sizeof(SampleRecord), MINUTE_KEEP_DAYS, false};
const Level FIVE_LEVEL = {"m5", "5-minute values", 5, sizeof(SampleRecord), FIVE_MINUTE_KEEP_DAYS, false};
const Level HOUR_LEVEL = {"h1", "Hourly values", 60, sizeof(HourRecord), 0, true};
const Level *const LEVELS[] = {&MINUTE_LEVEL, &FIVE_LEVEL, &HOUR_LEVEL};

// ---------- Averages ----------

// Sum, minimum and maximum of one measured value
struct Channel {
  int32_t sum = 0;
  int32_t min = 0;
  int32_t max = 0;
  uint16_t count = 0;

  void add(int32_t value) {
    if (count == 0 || value < min) min = value;
    if (count == 0 || value > max) max = value;
    sum += value;
    count++;
  }
  // Rounded to the nearest whole number
  int32_t average(int32_t missing) const {
    if (count == 0) return missing;
    return (sum >= 0 ? sum + count / 2 : sum - count / 2) / count;
  }
  int32_t minimum(int32_t missing) const { return count ? min : missing; }
  int32_t maximum(int32_t missing) const { return count ? max : missing; }
};

// Collects the readings of one minute, or the 1-minute values of 5 minutes or an hour.
// Each value has its own count, because one sensor can fail while the others work.
struct Accumulator {
  Channel co2, temperature, humidity, pressure;
  uint8_t records = 0;

  void add(const SampleRecord &r) {
    if (r.co2 != NO_CO2) co2.add(r.co2);
    if (r.temperature != NO_TEMPERATURE) temperature.add(r.temperature);
    if (r.humidity != NO_HUMIDITY) humidity.add(r.humidity);
    if (r.pressure != NO_PRESSURE) pressure.add(r.pressure);
    if (records < UINT8_MAX) records++;
  }

  SampleRecord average(uint32_t start) const {
    return {start, (uint16_t)co2.average(NO_CO2), (int16_t)temperature.average(NO_TEMPERATURE),
            (uint16_t)humidity.average(NO_HUMIDITY), (uint16_t)pressure.average(NO_PRESSURE)};
  }

  HourRecord hour(uint32_t start) const {
    HourRecord h;
    h.time = start;
    h.co2Avg = co2.average(NO_CO2);
    h.co2Min = co2.minimum(NO_CO2);
    h.co2Max = co2.maximum(NO_CO2);
    h.temperatureAvg = temperature.average(NO_TEMPERATURE);
    h.temperatureMin = temperature.minimum(NO_TEMPERATURE);
    h.temperatureMax = temperature.maximum(NO_TEMPERATURE);
    h.humidityAvg = humidity.average(NO_HUMIDITY);
    h.humidityMin = humidity.minimum(NO_HUMIDITY);
    h.humidityMax = humidity.maximum(NO_HUMIDITY);
    h.pressureAvg = pressure.average(NO_PRESSURE);
    h.minutes = records;
    h.flags = 0;
    return h;
  }
};

// Converts a reading to the whole-number units of SampleRecord
static SampleRecord toRecord(uint16_t co2Ppm, float temperatureC, float humidityPct, float pressureHpa) {
  SampleRecord r;
  r.time = 0;
  r.co2 = co2Ppm;
  r.temperature = isnan(temperatureC) ? NO_TEMPERATURE : (int16_t)lroundf(constrain(temperatureC, -100.0f, 200.0f) * 100);
  r.humidity = isnan(humidityPct) ? NO_HUMIDITY : (uint16_t)lroundf(constrain(humidityPct, 0.0f, 100.0f) * 100);
  r.pressure = isnan(pressureHpa) ? NO_PRESSURE : (uint16_t)lroundf(constrain(pressureHpa, 300.0f, 1250.0f) * 10);
  return r;
}

// ---------- State ----------
static bool enabled = false;     // data partition found and mounted
static bool clockValid = false;  // internet time has arrived

static Accumulator minuteSum;
static uint32_t minuteStart = 0;  // start of the minute being collected

// Finished 1-minute values that are not saved yet (ring buffer)
static SampleRecord pending[PENDING_MAX];
static uint16_t pendingFirst = 0;
static uint16_t pendingCount = 0;
static uint32_t pendingDropped = 0;

// Time of the newest record of each level, 0 = none yet
static uint32_t lastMinuteSaved = 0;
static uint32_t lastFiveSaved = 0;
static uint32_t lastHourSaved = 0;

static uint32_t nextSave = 0;

// ---------- Helpers ----------

static uint32_t uptimeSeconds() {
  return (uint32_t)(esp_timer_get_time() / 1000000);
}

// The clock the history runs on: UTC once the internet time has arrived,
// seconds since start-up before that
static uint32_t clockNow() {
  return clockValid ? (uint32_t)time(nullptr) : uptimeSeconds();
}

static void formatUtc(char *text, size_t size, uint32_t seconds, const char *format) {
  time_t t = seconds;
  struct tm tm;
  gmtime_r(&t, &tm);
  strftime(text, size, format, &tm);
}

// File name without the folder, for example "20261028.bin" or "202610.bin"
static void fileName(char *text, size_t size, const Level &level, uint32_t seconds) {
  formatUtc(text, size, seconds, level.monthFiles ? "%Y%m.bin" : "%Y%m%d.bin");
}

static void filePath(char *text, size_t size, const Level &level, const char *name) {
  snprintf(text, size, "/%s/%s", level.name, name);
}

static void filePathFor(char *text, size_t size, const Level &level, uint32_t seconds) {
  char name[16];
  fileName(name, sizeof name, level, seconds);
  filePath(text, size, level, name);
}

// Months counted as year * 12 + month (0-11), for walking through the hourly files
static int monthNumber(uint32_t seconds) {
  time_t t = seconds;
  struct tm tm;
  gmtime_r(&t, &tm);
  return (tm.tm_year + 1900) * 12 + tm.tm_mon;
}

static void monthFilePath(char *text, size_t size, int month) {
  snprintf(text, size, "/%s/%04d%02d.bin", HOUR_LEVEL.name, month / 12, month % 12 + 1);
}

// UTC seconds at midnight of a date. The ESP32's C library has no timegm(), so this
// counts the days since 1970 (Howard Hinnant's days_from_civil, years start in March).
static uint32_t utcSeconds(int year, int month, int day) {
  if (month <= 2) year--;
  int era = year / 400;
  int yearOfEra = year - era * 400;
  int dayOfYear = (153 * (month > 2 ? month - 3 : month + 9) + 2) / 5 + day - 1;
  int dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
  return (uint32_t)(era * 146097 + dayOfEra - 719468) * SECONDS_PER_DAY;
}

// Start of the day or month a file name stands for, 0 if the name doesn't fit
static uint32_t fileNameTime(const Level &level, const char *name) {
  int year = 0, month = 0, day = 1;
  int fields = level.monthFiles ? sscanf(name, "%4d%2d", &year, &month) : sscanf(name, "%4d%2d%2d", &year, &month, &day);
  if (fields != (level.monthFiles ? 2 : 3) || year < 2026 || month < 1 || month > 12) return 0;
  return utcSeconds(year, month, day);
}

static uint32_t lastSavedOf(const Level &level) {
  if (&level == &MINUTE_LEVEL) return lastMinuteSaved;
  if (&level == &FIVE_LEVEL) return lastFiveSaved;
  return lastHourSaved;
}

// Finds the oldest and newest file name of a level. The names are dates, so
// they sort in time order. Returns the number of files.
static uint16_t findFiles(const Level &level, char *oldest, char *newest, size_t size) {
  char folder[8];
  snprintf(folder, sizeof folder, "/%s", level.name);
  File dir = LittleFS.open(folder);
  if (!dir || !dir.isDirectory()) return 0;

  uint16_t count = 0;
  bool isDir = false;
  for (String path = dir.getNextFileName(&isDir); path.length() > 0; path = dir.getNextFileName(&isDir)) {
    if (isDir) continue;
    String name = path.substring(path.lastIndexOf('/') + 1);
    if (name.length() >= size) continue;
    if (count == 0 || strcmp(name.c_str(), oldest) < 0) strcpy(oldest, name.c_str());
    if (count == 0 || strcmp(name.c_str(), newest) > 0) strcpy(newest, name.c_str());
    count++;
  }
  return count;
}

// Opens a data file for reading and checks its header.
// Returns a closed File if it doesn't exist or doesn't match the level.
static File openForReading(const Level &level, const char *path) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f || f.isDirectory()) return File();
  FileHeader header;
  if (f.read((uint8_t *)&header, sizeof header) != sizeof header || memcmp(header.magic, FORMAT_MAGIC, 4) != 0
      || header.version != FORMAT_VERSION || header.minutes != level.minutes || header.recordSize != level.recordSize) {
    Serial.printf("History: %s has an unknown format, skipped\n", path);
    return File();
  }
  return f;
}

// Start times of the first and last record of a file. Returns false if it has none.
static bool recordSpan(const Level &level, const char *path, uint32_t &first, uint32_t &last) {
  File f = openForReading(level, path);
  if (!f || f.size() < sizeof(FileHeader) + level.recordSize) return false;
  size_t records = (f.size() - sizeof(FileHeader)) / level.recordSize;
  f.read((uint8_t *)&first, sizeof first);
  f.seek(sizeof(FileHeader) + (records - 1) * level.recordSize);
  f.read((uint8_t *)&last, sizeof last);
  return true;
}

// Calls onRecord for every record of a file, in time order, until it returns false
template <typename Record, typename Callback>
static void readRecords(const Level &level, const char *path, Callback onRecord) {
  File f = openForReading(level, path);
  if (!f) return;
  Record buffer[16];
  size_t bytes;
  while ((bytes = f.read((uint8_t *)buffer, sizeof buffer)) >= sizeof(Record)) {
    for (size_t i = 0; i < bytes / sizeof(Record); i++) {
      if (!onRecord(buffer[i])) return;
    }
  }
}

// Opens the file for the given time for appending. A new file gets its header first.
static File openForAppend(const Level &level, uint32_t seconds) {
  char path[24];
  filePathFor(path, sizeof path, level, seconds);
  File f = LittleFS.open(path, FILE_APPEND, true);
  if (f && f.size() == 0) {
    FileHeader header = {{FORMAT_MAGIC[0], FORMAT_MAGIC[1], FORMAT_MAGIC[2], FORMAT_MAGIC[3]},
                         FORMAT_VERSION, level.minutes, level.recordSize, (uint32_t)time(nullptr), 0};
    if (f.write((const uint8_t *)&header, sizeof header) != sizeof header) f.close();
  }
  if (!f) Serial.printf("History: could not write %s\n", path);
  return f;
}

// Time of the newest record of a level, 0 if there is none
static uint32_t newestRecordTime(const Level &level) {
  char oldest[16], newest[16], path[24];
  if (findFiles(level, oldest, newest, sizeof oldest) == 0) return 0;
  filePath(path, sizeof path, level, newest);
  uint32_t first = 0, last = 0;
  recordSpan(level, path, first, last);
  return last;
}

static void removeOldestFile(const Level &level, const char *oldest) {
  char path[24];
  filePath(path, sizeof path, level, oldest);
  LittleFS.remove(path);
  Serial.printf("History: deleted %s\n", path);
}

// Deletes files older than the level's retention. When the flash is nearly
// full, the oldest hourly month goes, then the oldest 5-minute day.
static void removeOldFiles(uint32_t now) {
  char oldest[16], newest[16], cutoff[16];
  for (const Level *level : LEVELS) {
    if (level->keepDays == 0) continue;
    fileName(cutoff, sizeof cutoff, *level, now - level->keepDays * SECONDS_PER_DAY);
    while (findFiles(*level, oldest, newest, sizeof oldest) > 0 && strcmp(oldest, cutoff) < 0) {
      removeOldestFile(*level, oldest);
    }
  }
  while (LittleFS.totalBytes() - LittleFS.usedBytes() < MIN_FREE_BYTES) {
    if (findFiles(HOUR_LEVEL, oldest, newest, sizeof oldest) > 1) {
      removeOldestFile(HOUR_LEVEL, oldest);
    } else if (findFiles(FIVE_LEVEL, oldest, newest, sizeof oldest) > 1) {
      removeOldestFile(FIVE_LEVEL, oldest);
    } else {
      break;
    }
  }
}

// ---------- 1-minute values in RAM ----------

static SampleRecord &pendingAt(uint16_t index) {
  return pending[(pendingFirst + index) % PENDING_MAX];
}

static void addPending(const SampleRecord &r) {
  // Ignore a minute that isn't newer than the last one (clock set back)
  if (pendingCount > 0 && r.time <= pendingAt(pendingCount - 1).time) return;
  if (pendingCount == PENDING_MAX) {  // RAM full: the oldest value is lost
    pendingFirst = (pendingFirst + 1) % PENDING_MAX;
    pendingCount--;
    pendingDropped++;
  }
  pendingAt(pendingCount) = r;
  pendingCount++;
}

static void removePending(uint16_t count) {
  pendingFirst = (pendingFirst + count) % PENDING_MAX;
  pendingCount -= count;
}

// Finishes the minute being collected once the clock has moved on
static void closeMinuteIfOver(uint32_t now) {
  uint32_t minute = now - now % 60;
  if (minute == minuteStart) return;
  if (minuteSum.records > 0) addPending(minuteSum.average(minuteStart));
  minuteSum = Accumulator();
  minuteStart = minute;
}

// Switches from "seconds since start-up" to UTC once the internet time arrives.
// The values measured until then get their real times.
static void checkClock() {
  if (clockValid) return;
  time_t now = time(nullptr);
  if (now < CLOCK_VALID_AFTER) return;

  uint32_t offset = (uint32_t)now - uptimeSeconds();
  for (uint16_t i = 0; i < pendingCount; i++) {
    uint32_t t = pendingAt(i).time + offset;
    pendingAt(i).time = t - t % 60;
  }
  uint32_t t = (minuteSum.records > 0 ? minuteStart : uptimeSeconds()) + offset;
  minuteStart = t - t % 60;

  clockValid = true;
  nextSave = (uint32_t)now - (uint32_t)now % SAVE_INTERVAL_S + SAVE_INTERVAL_S + SAVE_DELAY_S;

  char text[24];
  formatUtc(text, sizeof text, now, "%Y-%m-%d %H:%M:%S");
  Serial.printf("History: clock set to %s UTC\n", text);
  if (pendingCount > 0) {
    Serial.printf("History: %u minute%s from before got their times\n", pendingCount, pendingCount == 1 ? "" : "s");
  }
}

// ---------- Saving ----------

// Appends the waiting 1-minute values to their day files
static bool saveMinutes() {
  uint16_t done = 0;
  bool ok = true;
  while (ok && done < pendingCount) {
    uint32_t day = pendingAt(done).time / SECONDS_PER_DAY;
    File f = openForAppend(MINUTE_LEVEL, pendingAt(done).time);
    if (!f) {
      ok = false;
      break;
    }
    while (done < pendingCount && pendingAt(done).time / SECONDS_PER_DAY == day) {
      const SampleRecord &r = pendingAt(done);
      if (r.time > lastMinuteSaved) {
        if (f.write((const uint8_t *)&r, sizeof r) != sizeof r) {
          ok = false;
          break;
        }
        lastMinuteSaved = r.time;
      }
      done++;
    }
    f.close();
  }
  removePending(done);
  return ok;
}

// Output of deriveLevel() for one day
static SampleRecord fiveMinuteOut[SECONDS_PER_DAY / 300];
static HourRecord hourOut[24];

// Calculates finished 5-minute or hourly values from the saved 1-minute values
// and appends them. Periods that end after doneUntil wait for the next save.
// Works day by day, because neither period crosses midnight (UTC).
static bool deriveLevel(const Level &level, uint32_t &lastSaved, uint32_t doneUntil) {
  const bool hourly = &level == &HOUR_LEVEL;
  const uint16_t capacity = hourly ? 24 : SECONDS_PER_DAY / 300;
  const uint32_t period = level.minutes * 60;

  // Older 1-minute files have been deleted, so start no earlier than this
  uint32_t from = doneUntil - doneUntil % SECONDS_PER_DAY - MINUTE_KEEP_DAYS * SECONDS_PER_DAY;
  if (lastSaved + period > from) from = lastSaved + period;

  for (uint32_t day = from - from % SECONDS_PER_DAY; day < doneUntil; day += SECONDS_PER_DAY) {
    char path[24];
    filePathFor(path, sizeof path, MINUTE_LEVEL, day);
    if (!LittleFS.exists(path)) continue;

    Accumulator sum;
    uint32_t sumStart = 0;
    uint16_t count = 0;
    auto finishPeriod = [&]() {
      if (sum.records == 0 || count >= capacity) return;
      if (hourly) {
        hourOut[count++] = sum.hour(sumStart);
      } else {
        fiveMinuteOut[count++] = sum.average(sumStart);
      }
    };
    readRecords<SampleRecord>(MINUTE_LEVEL, path, [&](const SampleRecord &r) {
      uint32_t start = r.time - r.time % period;
      if (start + period > doneUntil) return false;  // not finished yet
      if (start < from || start < sumStart) return true;
      if (start != sumStart) {
        finishPeriod();
        sum = Accumulator();
        sumStart = start;
      }
      sum.add(r);
      return true;
    });
    finishPeriod();
    if (count == 0) continue;

    File f = openForAppend(level, day);
    if (!f) return false;
    size_t bytes = count * level.recordSize;
    bool written = f.write(hourly ? (const uint8_t *)hourOut : (const uint8_t *)fiveMinuteOut, bytes) == bytes;
    f.close();
    if (!written) return false;
    lastSaved = hourly ? hourOut[count - 1].time : fiveMinuteOut[count - 1].time;
  }
  return true;
}

// ---------- Public functions ----------

void historyBegin() {
  const esp_partition_t *partition =
    esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, PARTITION_LABEL);
  if (!partition || partition->address != PARTITION_ADDRESS || partition->size != PARTITION_SIZE) {
    Serial.println(F("History OFF: wrong partition scheme. Tools menu: Flash Size \"8MB\", "
                     "Partition Scheme \"8M with spiffs (3MB APP/1.5MB SPIFFS)\""));
    return;
  }
  if (!LittleFS.begin(false, "/littlefs", 10, PARTITION_LABEL)) {
    // New board, "Erase All Flash", or unreadable content
    Serial.println(F("History: formatting the data partition (first start)"));
    if (!LittleFS.begin(true, "/littlefs", 10, PARTITION_LABEL)) {
      Serial.println(F("History OFF: the data partition could not be formatted"));
      return;
    }
  }

  lastMinuteSaved = newestRecordTime(MINUTE_LEVEL);
  lastFiveSaved = newestRecordTime(FIVE_LEVEL);
  lastHourSaved = newestRecordTime(HOUR_LEVEL);
  enabled = true;
  Serial.printf("History: %u KB of %u KB used\n", LittleFS.usedBytes() / 1024, LittleFS.totalBytes() / 1024);
  checkClock();  // the clock survives a software restart
}

void historyAddSample(uint16_t co2Ppm, float temperatureC, float humidityPct, float pressureHpa) {
  if (!enabled) return;
  checkClock();
  closeMinuteIfOver(clockNow());
  minuteSum.add(toRecord(co2Ppm, temperatureC, humidityPct, pressureHpa));
}

void historyLoop() {
  if (!enabled) return;
  checkClock();
  closeMinuteIfOver(clockNow());
  if (clockValid && (uint32_t)time(nullptr) >= nextSave) historySave();
}

bool historySave() {
  if (!enabled || !clockValid) return false;
  uint32_t now = time(nullptr);
  closeMinuteIfOver(now);
  nextSave = now - now % SAVE_INTERVAL_S + SAVE_INTERVAL_S + SAVE_DELAY_S;

  removeOldFiles(now);
  bool ok = saveMinutes();
  // Every minute before this is final: saved, or not measured at all
  uint32_t doneUntil = pendingCount > 0 ? pendingAt(0).time : minuteStart;
  ok = deriveLevel(FIVE_LEVEL, lastFiveSaved, doneUntil) && ok;
  ok = deriveLevel(HOUR_LEVEL, lastHourSaved, doneUntil) && ok;
  return ok;
}

void historyPrintStatus(Print &out) {
  if (!enabled) {
    out.println(F("History: OFF (see the messages at start-up)"));
    return;
  }
  char text[24];
  uint32_t now = clockNow();
  if (clockValid) {
    formatUtc(text, sizeof text, now, "%Y-%m-%d %H:%M:%S");
    out.printf("Clock: %s UTC\n", text);
  } else {
    out.println(F("Clock: not set yet, waiting for the internet time"));
  }
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  out.printf("Flash: %u KB of %u KB used (%u%%)\n", used / 1024, total / 1024, total ? used * 100 / total : 0);
  unsigned inRam = pendingCount + (minuteSum.records > 0 ? 1 : 0);
  out.printf("Not saved yet (in RAM): %u minute%s", inRam, inRam == 1 ? "" : "s");
  if (clockValid) out.printf(", next save in %u s", (unsigned)(nextSave > now ? nextSave - now : 0));
  out.println();
  if (pendingDropped > 0) out.printf("Lost because RAM was full: %lu minute values\n", (unsigned long)pendingDropped);

  for (const Level *level : LEVELS) {
    char oldest[16], newest[16];
    uint16_t files = findFiles(*level, oldest, newest, sizeof oldest);
    out.printf("%s (%s): %u file%s", level->description, level->name, files, files == 1 ? "" : "s");
    if (files > 0) out.printf(", %s ... %s", oldest, newest);
    if (lastSavedOf(*level) > 0) {
      formatUtc(text, sizeof text, lastSavedOf(*level), "%Y-%m-%d %H:%M");
      out.printf(", newest value %s UTC", text);
    }
    out.println();
  }
}

void historyPrintFiles(Print &out) {
  if (!enabled) {
    out.println(F("History: OFF (see the messages at start-up)"));
    return;
  }
  for (const Level *level : LEVELS) {
    char folder[8];
    snprintf(folder, sizeof folder, "/%s", level->name);
    File dir = LittleFS.open(folder);
    if (!dir || !dir.isDirectory()) continue;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      size_t size = f.size();
      size_t values = size > sizeof(FileHeader) ? (size - sizeof(FileHeader)) / level->recordSize : 0;
      out.printf("%s/%s  %u bytes  %u values\n", folder, f.name(), size, values);
    }
  }
}

// Prints ",value", or just "," when the value is missing
static void printValue(Print &out, int32_t value, int32_t missing, float divisor, uint8_t decimals) {
  out.print(',');
  if (value == missing) return;
  if (decimals == 0) {
    out.print(value);
  } else {
    out.print(value / divisor, decimals);
  }
}

static void printTime(Print &out, uint32_t seconds) {
  char text[20];
  formatUtc(text, sizeof text, seconds, "%Y-%m-%d %H:%M");
  out.print(text);
}

static void printSample(Print &out, const SampleRecord &r) {
  printTime(out, r.time);
  printValue(out, r.co2, NO_CO2, 1, 0);
  printValue(out, r.temperature, NO_TEMPERATURE, 100, 2);
  printValue(out, r.humidity, NO_HUMIDITY, 100, 2);
  printValue(out, r.pressure, NO_PRESSURE, 10, 1);
  out.println();
}

static void printHour(Print &out, const HourRecord &r) {
  printTime(out, r.time);
  printValue(out, r.co2Avg, NO_CO2, 1, 0);
  printValue(out, r.co2Min, NO_CO2, 1, 0);
  printValue(out, r.co2Max, NO_CO2, 1, 0);
  printValue(out, r.temperatureAvg, NO_TEMPERATURE, 100, 2);
  printValue(out, r.temperatureMin, NO_TEMPERATURE, 100, 2);
  printValue(out, r.temperatureMax, NO_TEMPERATURE, 100, 2);
  printValue(out, r.humidityAvg, NO_HUMIDITY, 100, 2);
  printValue(out, r.humidityMin, NO_HUMIDITY, 100, 2);
  printValue(out, r.humidityMax, NO_HUMIDITY, 100, 2);
  printValue(out, r.pressureAvg, NO_PRESSURE, 10, 1);
  printValue(out, r.minutes, -1, 1, 0);
  out.println();
}

bool historyPrintCsv(Print &out, const char *levelName, uint16_t days) {
  const Level *level = nullptr;
  for (const Level *l : LEVELS) {
    if (strcmp(l->name, levelName) == 0) level = l;
  }
  if (!level || days == 0) return false;
  if (!enabled || !clockValid) {
    out.println(F("History: nothing to show yet (history off or clock not set)"));
    return true;
  }

  uint32_t now = time(nullptr);
  uint64_t span = (uint64_t)days * SECONDS_PER_DAY;
  uint32_t from = span < now - CLOCK_VALID_AFTER ? now - (uint32_t)span : (uint32_t)CLOCK_VALID_AFTER;
  if (level->keepDays > 0) {
    // Older files have been deleted anyway
    uint32_t oldestKept = now - now % SECONDS_PER_DAY - level->keepDays * SECONDS_PER_DAY;
    if (from < oldestKept) from = oldestKept;
  }

  char path[24];
  if (level == &HOUR_LEVEL) {
    out.println(F("time_utc,co2_avg_ppm,co2_min_ppm,co2_max_ppm,temperature_avg_c,temperature_min_c,temperature_max_c,"
                  "humidity_avg_pct,humidity_min_pct,humidity_max_pct,pressure_avg_hpa,minutes"));
    for (int month = monthNumber(from); month <= monthNumber(now); month++) {
      monthFilePath(path, sizeof path, month);
      readRecords<HourRecord>(*level, path, [&](const HourRecord &r) {
        if (r.time >= from) printHour(out, r);
        return true;
      });
    }
  } else {
    out.println(F("time_utc,co2_ppm,temperature_c,humidity_pct,pressure_hpa"));
    for (uint32_t day = from - from % SECONDS_PER_DAY; day <= now; day += SECONDS_PER_DAY) {
      filePathFor(path, sizeof path, *level, day);
      readRecords<SampleRecord>(*level, path, [&](const SampleRecord &r) {
        if (r.time >= from) printSample(out, r);
        return true;
      });
    }
    if (level == &MINUTE_LEVEL) {  // plus the values not saved yet
      for (uint16_t i = 0; i < pendingCount; i++) {
        if (pendingAt(i).time > lastMinuteSaved && pendingAt(i).time >= from) printSample(out, pendingAt(i));
      }
    }
  }
  return true;
}

// ---------- Web page helpers ----------

bool historyClockSet() {
  return clockValid;
}

bool historyWriteRecords(Print &out, uint8_t minutes, uint32_t from, uint32_t to) {
  const Level *level = nullptr;
  for (const Level *l : LEVELS) {
    if (l->minutes == minutes) level = l;
  }
  if (!level) return false;
  if (!enabled || !clockValid || from >= to) return true;

  char path[24];
  if (level == &HOUR_LEVEL) {
    for (int month = monthNumber(from); month <= monthNumber(to - 1); month++) {
      monthFilePath(path, sizeof path, month);
      readRecords<HourRecord>(*level, path, [&](const HourRecord &r) {
        if (r.time >= to) return false;
        if (r.time >= from) out.write((const uint8_t *)&r, sizeof r);
        return true;
      });
    }
    return true;
  }

  for (uint32_t day = from - from % SECONDS_PER_DAY; day < to; day += SECONDS_PER_DAY) {
    filePathFor(path, sizeof path, *level, day);
    readRecords<SampleRecord>(*level, path, [&](const SampleRecord &r) {
      if (r.time >= to) return false;
      if (r.time >= from) out.write((const uint8_t *)&r, sizeof r);
      return true;
    });
  }
  if (level == &MINUTE_LEVEL) {  // plus the values not saved yet
    for (uint16_t i = 0; i < pendingCount; i++) {
      const SampleRecord &r = pendingAt(i);
      if (r.time > lastMinuteSaved && r.time >= from && r.time < to) out.write((const uint8_t *)&r, sizeof r);
    }
  }
  return true;
}

void historyWriteDaysJson(Print &out, uint32_t from, uint32_t to) {
  out.print('[');
  if (!enabled || !clockValid || from >= to) {
    out.print(']');
    return;
  }

  // Hourly values of one month, first and last start time per day of the month
  int hourMonth = -1;
  uint32_t hourFirst[31], hourLast[31];

  bool firstEntry = true;
  for (uint32_t day = from - from % SECONDS_PER_DAY; day < to; day += SECONDS_PER_DAY) {
    char path[24];
    uint32_t first = 0, last = 0;
    uint8_t res = 0;

    filePathFor(path, sizeof path, MINUTE_LEVEL, day);
    if (recordSpan(MINUTE_LEVEL, path, first, last)) res = 1;
    for (uint16_t i = 0; i < pendingCount; i++) {  // values not saved yet
      uint32_t t = pendingAt(i).time;
      if (t <= lastMinuteSaved || t < day || t >= day + SECONDS_PER_DAY) continue;
      if (res == 0) first = t;
      res = 1;
      last = t;
    }

    if (res == 0) {
      filePathFor(path, sizeof path, FIVE_LEVEL, day);
      if (recordSpan(FIVE_LEVEL, path, first, last)) res = 5;
    }

    if (res == 0) {
      if (monthNumber(day) != hourMonth) {
        hourMonth = monthNumber(day);
        memset(hourFirst, 0, sizeof hourFirst);
        memset(hourLast, 0, sizeof hourLast);
        monthFilePath(path, sizeof path, hourMonth);
        readRecords<HourRecord>(HOUR_LEVEL, path, [&](const HourRecord &r) {
          time_t t = r.time;
          struct tm tm;
          gmtime_r(&t, &tm);
          if (hourFirst[tm.tm_mday - 1] == 0) hourFirst[tm.tm_mday - 1] = r.time;
          hourLast[tm.tm_mday - 1] = r.time;
          return true;
        });
      }
      time_t t = day;
      struct tm tm;
      gmtime_r(&t, &tm);
      if (hourFirst[tm.tm_mday - 1] != 0) {
        first = hourFirst[tm.tm_mday - 1];
        last = hourLast[tm.tm_mday - 1];
        res = 60;
      }
    }

    if (res == 0) continue;
    out.printf("%s{\"day\":%lu,\"res\":%u,\"first\":%lu,\"last\":%lu}", firstEntry ? "" : ",", (unsigned long)day, res,
               (unsigned long)first, (unsigned long)last);
    firstEntry = false;
  }
  out.print(']');
}

void historyWriteInfoJson(Print &out) {
  out.printf("{\"on\":%s,\"clock\":%s", enabled ? "true" : "false", clockValid ? "true" : "false");
  if (enabled) {
    out.printf(",\"usedKB\":%u,\"totalKB\":%u,\"unsaved\":%u,\"oldest\":{", LittleFS.usedBytes() / 1024,
               LittleFS.totalBytes() / 1024, pendingCount + (minuteSum.records > 0 ? 1 : 0));
    for (const Level *level : LEVELS) {
      char oldest[16], newest[16];
      uint32_t start = findFiles(*level, oldest, newest, sizeof oldest) > 0 ? fileNameTime(*level, oldest) : 0;
      out.printf("%s\"%s\":%lu", level == LEVELS[0] ? "" : ",", level->name, (unsigned long)start);
    }
    out.print("},\"keepDays\":{");
    for (const Level *level : LEVELS) {
      out.printf("%s\"%s\":%u", level == LEVELS[0] ? "" : ",", level->name, level->keepDays);
    }
    out.print('}');
  }
  out.print('}');
}
