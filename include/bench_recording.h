#ifndef HF_BENCH_RECORDING_H
#define HF_BENCH_RECORDING_H

#include <cmath>
#include <cstddef>
#include <cstdint>

// Portable recorder: no Arduino dependencies, heap allocation, or persistence.
// Own one Recorder in static/global storage (approximately 48 KiB). It is
// deliberately noncopyable. begin() resets metadata, not the entire buffer.
namespace bench_recording {

enum class FinishReason : uint8_t {
  None,
  Completed,
  Stopped,
  Replaced,
  Ota,
  Full
};

enum EntryFlags : uint8_t { kRawValid = 1, kMedianValid = 2 };

struct Entry {
  uint32_t elapsed_ms;
  uint32_t raw_ticks;
  uint32_t median20_ticks;
  uint8_t flags;

  bool rawValid() const { return (flags & kRawValid) != 0; }
  bool medianValid() const { return (flags & kMedianValid) != 0; }
};

struct Statistics {
  uint32_t count;
  uint32_t min;
  uint32_t max;
  double mean;

  Statistics() : count(0), min(0), max(0), mean(0), m2_(0) {}

  // Every numeric result is unavailable when count == 0. Callers must emit
  // null/blank in that case, rather than treating initialized zeros as data.
  double populationStddev() const {
    return count ? std::sqrt(m2_ > 0 ? m2_ / count : 0.0) : 0.0;
  }

  void add(uint32_t value) {
    if (count == 0) {
      min = max = value;
    } else {
      if (value < min) min = value;
      if (value > max) max = value;
    }
    ++count;
    const double delta = static_cast<double>(value) - mean;
    mean += delta / count;
    m2_ += delta * (static_cast<double>(value) - mean);
  }

 private:
  double m2_;
};

class Recorder {
 public:
  enum : uint32_t {
    kCapacity = 3000,
    kDurationMs = 30000,
    kMinSpacingMs = 10,
    kWindowSize = 20
  };

  Recorder()
      : has_session_(false), active_(false), started_ms_(0),
        finished_elapsed_ms_(0), reason_(FinishReason::None), size_(0),
        timeouts_(0), window_size_(0), window_next_(0) {}
  Recorder(const Recorder &) = delete;
  Recorder &operator=(const Recorder &) = delete;
  Recorder(Recorder &&) = delete;
  Recorder &operator=(Recorder &&) = delete;

  // A new session explicitly replaces the previous retained capture. No
  // entries beyond size() are exposed, so old buffer contents need not clear.
  void begin(uint32_t now) {
    has_session_ = true;
    active_ = true;
    started_ms_ = now;
    finished_elapsed_ms_ = 0;
    reason_ = FinishReason::None;
    size_ = 0;
    timeouts_ = 0;
    window_size_ = 0;
    window_next_ = 0;
    raw_stats_ = Statistics();
    median_stats_ = Statistics();
  }

  // Unsigned subtraction supports a millis() rollover during the session.
  // Caller must service the session normally, not leave it for a full wrap.
  bool dueToFinish(uint32_t now) const {
    return active_ && static_cast<uint32_t>(now - started_ms_) >= kDurationMs;
  }

  // now is the acquisition START timestamp. The caller schedules acquisitions
  // at >= kMinSpacingMs; add() retains every attempt rather than silently
  // dropping closely spaced attempts. If that contract is violated, capacity
  // still stops the session safely. Valid=false means a sensor timeout.
  // The interval is [begin, begin + 30000 ms): the endpoint is excluded.
  // Returns true only if this attempt was appended, including the final entry
  // that fills the buffer. A full buffer remains active until the deadline;
  // only an additional in-window attempt ends it with reason Full. Thus a
  // normally spaced 3000-entry session finishes Completed at 30000 ms.
  // A timeout's raw ticks are retained but not included
  // in valid-value statistics; its unavailable median is stored as zero.
  bool add(uint32_t now, uint32_t raw_ticks, bool valid) {
    if (!active_) return false;
    if (dueToFinish(now)) {
      finish(now, FinishReason::Completed);
      return false;
    }
    if (size_ >= kCapacity) {
      finish(now, FinishReason::Full);
      return false;
    }

    Entry &sample = entries_[size_];
    sample.elapsed_ms = static_cast<uint32_t>(now - started_ms_);
    sample.raw_ticks = raw_ticks;
    sample.median20_ticks = 0;
    sample.flags = valid ? kRawValid : 0;

    if (valid) {
      raw_stats_.add(raw_ticks);
      window_[window_next_] = raw_ticks;
      window_next_ = (window_next_ + 1) % kWindowSize;
      if (window_size_ < kWindowSize) ++window_size_;
      if (window_size_ == kWindowSize) {
        sample.median20_ticks = upperMedian();
        sample.flags |= kMedianValid;
        median_stats_.add(sample.median20_ticks);
      }
    } else {
      ++timeouts_;
      window_size_ = 0;
      window_next_ = 0;
    }
    ++size_;
    return true;
  }

  // Idempotent: an already finished capture, its data and its first finish
  // reason are preserved. Explicit stop/OTA do not erase captured entries.
  void finish(uint32_t now, FinishReason reason) {
    if (!active_) return;
    active_ = false;
    finished_elapsed_ms_ = static_cast<uint32_t>(now - started_ms_);
    reason_ = reason;
  }

  bool active() const { return active_; }
  bool hasSession() const { return has_session_; }
  uint32_t size() const { return size_; }
  uint32_t goodCount() const { return raw_stats_.count; }
  uint32_t timeoutCount() const { return timeouts_; }
  uint32_t startedAtMs() const { return started_ms_; }
  uint32_t finishedElapsedMs() const { return finished_elapsed_ms_; }
  uint32_t elapsedMs(uint32_t now) const {
    if (!has_session_) return 0;
    return active_ ? static_cast<uint32_t>(now - started_ms_)
                   : finished_elapsed_ms_;
  }
  FinishReason reason() const { return reason_; }
  const Statistics &rawStats() const { return raw_stats_; }
  const Statistics &medianStats() const { return median_stats_; }
  const Entry *entry(uint32_t index) const {
    return index < size_ ? &entries_[index] : nullptr;
  }

 private:
  uint32_t upperMedian() const {
    // Only twenty integers (80 bytes) are copied, never the capture buffer.
    // Insertion sort is bounded and allocation-free. Index 10 is the upper
    // median, matching original getMedian()'s sorted[size / 2] convention.
    uint32_t sorted[kWindowSize];
    for (uint32_t i = 0; i < kWindowSize; ++i) sorted[i] = window_[i];
    for (uint32_t i = 1; i < kWindowSize; ++i) {
      const uint32_t value = sorted[i];
      uint32_t j = i;
      while (j > 0 && sorted[j - 1] > value) {
        sorted[j] = sorted[j - 1];
        --j;
      }
      sorted[j] = value;
    }
    return sorted[kWindowSize / 2];
  }

  bool has_session_;
  bool active_;
  uint32_t started_ms_;
  uint32_t finished_elapsed_ms_;
  FinishReason reason_;
  uint32_t size_;
  uint32_t timeouts_;
  uint32_t window_size_;
  uint32_t window_next_;
  Statistics raw_stats_;
  Statistics median_stats_;
  uint32_t window_[kWindowSize];
  Entry entries_[kCapacity];
};

}  // namespace bench_recording

#endif  // HF_BENCH_RECORDING_H
