#include "sensor_wait.h"
#include <cassert>
#include <iostream>

int main() {
  // Permanent LOW: must stop at the deadline instead of locking the CPU.
  uint64_t ticks = 0;
  auto stuck = waitForSensorHigh([](){return false;}, [&](){return ++ticks;});
  assert(stuck.timedOut && stuck.ticks == kSensorTimeoutTicks);
  // A normal rise in the original calibration range must retain its raw units.
  ticks = 0;
  auto normal = waitForSensorHigh([&](){return ticks >= 12000;}, [&](){return ++ticks;});
  assert(!normal.timedOut && normal.ticks == 12001);
  // Immediate HIGH must finish, including a zero-tick result.
  auto immediate = waitForSensorHigh([](){return true;}, [](){return uint64_t(0);});
  assert(!immediate.timedOut && immediate.ticks == 0);
  // A rise observed at/after the deadline is not accepted as a good sample.
  auto late = waitForSensorHigh([](){return true;}, [](){return kSensorTimeoutTicks;});
  assert(late.timedOut);
  std::cout << "PASS: stuck LOW, normal rise, immediate HIGH, deadline boundary\n";
}
