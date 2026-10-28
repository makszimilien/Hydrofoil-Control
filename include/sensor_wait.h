#pragma once
#include <stdint.h>

// Original timer: 80 MHz / divider 2 = 40 ticks per microsecond.
// Original default calibration: 2,000..24,000 ticks (50..600 us).
// A 2 ms ceiling preserves that range and bounds interrupt blocking.
constexpr uint64_t kSensorTimeoutTicks = 80000;
struct SensorWaitResult {
  bool timedOut;
  uint64_t ticks;
};

template <typename ReadHigh, typename ReadTicks>
SensorWaitResult waitForSensorHigh(ReadHigh readHigh, ReadTicks readTicks) {
  for (;;) {
    const bool high = readHigh();
    const uint64_t ticks = readTicks();
    if (ticks >= kSensorTimeoutTicks)
      return {true, ticks};
    if (high)
      return {false, ticks};
  }
}
