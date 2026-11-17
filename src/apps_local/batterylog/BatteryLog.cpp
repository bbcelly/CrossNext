#include "BatteryLog.h"

#if CROSSPOINT_BATTERY_LOG

#include <Arduino.h>
#include <BatteryMonitor.h>
#include <HalDisplay.h>
#include <HalStorage.h>
#include <Logging.h>
#include <WiFi.h>

#include <cstdio>
#include <ctime>

#include "CrossPointSettings.h"

namespace batterylog {
namespace {

// In the card's root, not /.crosspoint: this file is read on a laptop.
constexpr const char* kLog = "/batterylog.csv";
constexpr const char* kHeader = "epoch,uptime_s,event,mv,pct,refreshes,cpu_mhz,wifi,reader,devmode\n";

constexpr unsigned long kIntervalMs = 60000;

// A single ADC read on the S3 wanders by tens of millivolts, which is hours of
// idle drain. Averaging a handful brings a row down to the trend.
constexpr int kSamples = 16;

unsigned long lastRowMs = 0;

void append(const char* event, const bool readerActive) {
  // A function-local static, the same shape HalPowerManager and PowerProbe use.
  static const BatteryMonitor battery;
  uint32_t sum = 0;
  for (int i = 0; i < kSamples; ++i) sum += battery.readMillivolts();

  char line[128];
  std::snprintf(line, sizeof(line), "%ld,%lu,%s,%u,%u,%u,%u,%d,%d,%d\n", static_cast<long>(std::time(nullptr)),
                millis() / 1000UL, event, static_cast<unsigned>(sum / kSamples),
                static_cast<unsigned>(battery.readPercentage()), static_cast<unsigned>(HalDisplay::refreshCount()),
                static_cast<unsigned>(getCpuFrequencyMhz()), WiFi.getMode() != WIFI_MODE_NULL ? 1 : 0,
                readerActive ? 1 : 0, SETTINGS.devMode != 0 ? 1 : 0);

  const bool fresh = !Storage.exists(kLog);
  HalFile file;
  if (!Storage.openFileForAppend("BATLOG", kLog, file)) {
    LOG_ERR("BATLOG", "could not open the log");
    return;
  }
  if (fresh) file.print(kHeader);
  file.print(line);
  lastRowMs = millis();
}

}  // namespace

void afterBoot(const char* wakeReason) { append(wakeReason, false); }

void tick(const bool readerActive) {
  if (millis() - lastRowMs < kIntervalMs) return;
  append("awake", readerActive);
}

void beforeSleep() { append("sleep", false); }

}  // namespace batterylog

#endif  // CROSSPOINT_BATTERY_LOG
