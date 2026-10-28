#include "bench_recording.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <vector>

using bench_recording::FinishReason;
using bench_recording::Recorder;
using bench_recording::Statistics;

// The production-sized buffer stays in static storage, including in tests.
static Recorder recording;
static_assert(!std::is_copy_constructible<Recorder>::value, "no buffer copies");
static_assert(!std::is_move_constructible<Recorder>::value, "no buffer moves");

static void near(double actual, double expected, double tolerance = 1e-9) {
  assert(std::fabs(actual - expected) <= tolerance);
}

static void expectStatistics(const Statistics &stats,
                             const std::vector<uint32_t> &values) {
  assert(stats.count == values.size());
  if (values.empty()) return;  // No fabricated numeric observation when empty.
  assert(stats.min == *std::min_element(values.begin(), values.end()));
  assert(stats.max == *std::max_element(values.begin(), values.end()));
  double sum = 0;
  for (uint32_t value : values) sum += value;
  const double mean = sum / values.size();
  double squares = 0;
  for (uint32_t value : values) squares += (value - mean) * (value - mean);
  near(stats.mean, mean);
  near(stats.populationStddev(), std::sqrt(squares / values.size()));
}

static void testInitialEmptyAndAllTimeouts() {
  assert(!recording.hasSession());
  assert(!recording.active());
  assert(recording.size() == 0);
  assert(recording.elapsedMs(123) == 0);
  assert(recording.entry(0) == nullptr);
  assert(!recording.add(0, 100, true));
  recording.finish(10, FinishReason::Stopped);
  assert(recording.reason() == FinishReason::None);

  recording.begin(100);
  assert(recording.hasSession() && recording.active());
  assert(recording.rawStats().count == 0);
  assert(recording.medianStats().count == 0);
  recording.finish(103, FinishReason::Stopped);
  assert(recording.size() == 0 && recording.finishedElapsedMs() == 3);

  recording.begin(200);
  for (uint32_t i = 0; i < 25; ++i) {
    assert(recording.add(200 + i * 10, 80000 + i, false));
    const auto *entry = recording.entry(i);
    assert(entry && entry->raw_ticks == 80000 + i);
    assert(entry->elapsed_ms == i * 10);
    assert(!entry->rawValid() && !entry->medianValid());
  }
  assert(recording.size() == 25 && recording.goodCount() == 0);
  assert(recording.timeoutCount() == 25);
  assert(recording.rawStats().count == 0);
  assert(recording.medianStats().count == 0);
}

static void testEveryAttemptAndOutlierStatistics() {
  recording.begin(1234);
  std::vector<uint32_t> expected;
  for (uint32_t i = 0; i < 101; ++i) {
    const uint32_t value = i == 37 ? 79999 : (i == 63 ? 0 : 4500 + i);
    assert(recording.add(1234 + i * 10, value, true));
    expected.push_back(value);
    const auto *entry = recording.entry(i);
    assert(entry && entry->raw_ticks == value && entry->elapsed_ms == i * 10);
    assert(entry->rawValid());
  }
  assert(recording.size() == 101 && recording.timeoutCount() == 0);
  expectStatistics(recording.rawStats(), expected);
  assert(recording.rawStats().min == 0 && recording.rawStats().max == 79999);
  assert(recording.entry(101) == nullptr);

  recording.begin(0);
  const uint32_t known[] = {2, 4, 4, 4, 5, 5, 7, 9};
  for (uint32_t i = 0; i < 8; ++i) assert(recording.add(i * 10, known[i], true));
  near(recording.rawStats().mean, 5);
  near(recording.rawStats().populationStddev(), 2);
  assert(recording.medianStats().count == 0);
}

static void testUpperMedianWarmupRingAndTimeoutReset() {
  recording.begin(1000);
  std::vector<uint32_t> recent;
  std::vector<uint32_t> medians;
  std::vector<uint32_t> raw;
  // Unsorted, repeated and outlier values exercise ring replacement and the
  // upper median convention; the independent reference sorts a vector.
  for (uint32_t i = 0; i < 100; ++i) {
    const bool valid = i != 45;
    const uint32_t value = valid ? ((i * 31) % 29) + 100 : 80017;
    assert(recording.add(1000 + i * 10, value, valid));
    const auto *entry = recording.entry(i);
    if (!valid) {
      recent.clear();
      assert(!entry->rawValid() && !entry->medianValid());
      continue;
    }
    raw.push_back(value);
    recent.push_back(value);
    if (recent.size() > 20) recent.erase(recent.begin());
    assert(entry->medianValid() == (recent.size() == 20));
    if (recent.size() == 20) {
      auto sorted = recent;
      std::sort(sorted.begin(), sorted.end());
      assert(entry->median20_ticks == sorted[10]);
      medians.push_back(sorted[10]);
    }
  }
  assert(!recording.entry(18)->medianValid());
  assert(recording.entry(19)->medianValid());
  assert(!recording.entry(64)->medianValid());
  assert(recording.entry(65)->medianValid());
  assert(recording.goodCount() == 99 && recording.timeoutCount() == 1);
  expectStatistics(recording.rawStats(), raw);
  expectStatistics(recording.medianStats(), medians);

  recording.begin(0);
  for (uint32_t i = 0; i < 20; ++i) assert(recording.add(i * 10, i, true));
  assert(recording.entry(19)->median20_ticks == 10);  // Not 9 or 9.5.
  assert(recording.add(200, 1000, true));
  assert(recording.entry(20)->median20_ticks == 11);
}

static void testClockWrapAndDurationBoundary() {
  const uint32_t start = UINT32_MAX - 99;
  recording.begin(start);
  assert(recording.add(start, 1, true));
  assert(recording.add(start + uint32_t(120), 2, true));
  assert(recording.entry(1)->elapsed_ms == 120);
  assert(recording.elapsedMs(start + uint32_t(999)) == 999);
  assert(!recording.dueToFinish(start + uint32_t(29999)));
  assert(recording.add(start + uint32_t(29999), 3, true));
  assert(recording.dueToFinish(start + uint32_t(30000)));
  assert(!recording.add(start + uint32_t(30000), 99999, true));
  assert(!recording.active() && recording.reason() == FinishReason::Completed);
  assert(recording.finishedElapsedMs() == 30000 && recording.size() == 3);
  assert(recording.rawStats().max == 3);
  assert(!recording.dueToFinish(start + uint32_t(30001)));
  assert(recording.elapsedMs(start + uint32_t(60000)) == 30000);
}

static void testCapacityStopRetentionAndReplacement() {
  recording.begin(0);
  for (uint32_t i = 0; i < Recorder::kCapacity; ++i)
    assert(recording.add(i * Recorder::kMinSpacingMs, i, true));
  assert(recording.size() == Recorder::kCapacity);
  assert(recording.active() && recording.reason() == FinishReason::None);
  assert(!recording.dueToFinish(29999));
  assert(recording.entry(2999)->raw_ticks == 2999);
  assert(recording.entry(3000) == nullptr);
  assert(recording.dueToFinish(30000));
  recording.finish(30000, FinishReason::Completed);
  assert(recording.reason() == FinishReason::Completed);
  assert(recording.finishedElapsedMs() == 30000 && !recording.active());
  recording.finish(30001, FinishReason::Stopped);
  assert(recording.reason() == FinishReason::Completed);  // First finish wins.
  assert(recording.size() == 3000 && recording.rawStats().max == 2999);

  recording.begin(50000);
  assert(recording.active() && recording.reason() == FinishReason::None);
  assert(recording.size() == 0 && recording.entry(0) == nullptr);
  assert(recording.rawStats().count == 0 && recording.medianStats().count == 0);
  assert(recording.timeoutCount() == 0);
  assert(recording.add(50000, 43, true));
  recording.finish(50011, FinishReason::Stopped);
  assert(!recording.add(50020, 99, true));
  assert(recording.elapsedMs(90000) == 11);
  assert(recording.entry(0)->raw_ticks == 43 && !recording.entry(0)->medianValid());
  assert(recording.entry(1) == nullptr);
  assert(recording.rawStats().count == 1);
  near(recording.rawStats().populationStddev(), 0);

  recording.begin(60000);
  assert(recording.add(60000, 123, true));
  recording.finish(60010, FinishReason::Ota);
  assert(recording.reason() == FinishReason::Ota && recording.size() == 1);
  recording.begin(70000);
  assert(recording.add(70000, 124, true));
  recording.finish(70010, FinishReason::Replaced);
  assert(recording.reason() == FinishReason::Replaced && recording.size() == 1);

  // Even a caller that violates the 10 ms scheduling contract cannot overflow.
  recording.begin(80000);
  for (uint32_t i = 0; i < Recorder::kCapacity; ++i)
    assert(recording.add(80000, i, false));
  assert(recording.timeoutCount() == Recorder::kCapacity);
  assert(recording.active());
  assert(!recording.add(80001, 80000, false));
  assert(!recording.active() && recording.reason() == FinishReason::Full);
  assert(recording.finishedElapsedMs() == 1);
  assert(recording.size() == Recorder::kCapacity);
}

int main() {
  testInitialEmptyAndAllTimeouts();
  testEveryAttemptAndOutlierStatistics();
  testUpperMedianWarmupRingAndTimeoutReset();
  testClockWrapAndDurationBoundary();
  testCapacityStopRetentionAndReplacement();
  std::cout << "PASS: every attempt/extrema, Welford population statistics, "
               "20-sample upper median/warmup/ring/timeout reset, empty/timeouts, "
               "millis wrap/duration, capacity/stop/OTA/replacement retention\n";
  std::cout << "Recorder static bytes: " << sizeof(Recorder) << '\n';
}
