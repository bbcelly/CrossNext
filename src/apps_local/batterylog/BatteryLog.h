#pragma once

// Battery voltage over time, on the card, for boards with no fuel gauge.
//
// PowerProbe prices a sleep from the CW2017's fractional state of charge. The
// plain LilyGo T5-4.7 S3 has only a divider on an ADC pin, so the one thing it
// can record is the cell voltage itself: one CSV row a minute while awake and
// one on each side of a sleep, with enough beside it (panel updates, CPU
// clock, radio, Developer Mode) to say what the device was doing.
//
// Read it as a curve, not a current. A LiPo's voltage is not linear in charge,
// so the honest use is comparing two runs that start from the same voltage.
//
// Off unless the env passes -DCROSSPOINT_BATTERY_LOG=1.

#ifndef CROSSPOINT_BATTERY_LOG
#define CROSSPOINT_BATTERY_LOG 0
#endif

namespace batterylog {

#if CROSSPOINT_BATTERY_LOG
// One row per boot, tagged with what woke the device.
void afterBoot(const char* wakeReason);

// Call every loop pass; writes at most one row a minute.
void tick(bool readerActive);

// One row on the way into deep sleep, while the card is still mounted.
void beforeSleep();
#else
inline void afterBoot(const char*) {}
inline void tick(bool) {}
inline void beforeSleep() {}
#endif

}  // namespace batterylog
