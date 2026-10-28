# R5.5 host tests

Build with PlatformIO first. From the repository root run `python3 tests/run_all_r55_tests.py`. Required: Python 3, clang++ with UndefinedBehaviorSanitizer, Node.js, and the resolved QuickPID/ArduinoJson libraries under `.pio/libdeps/esp32-c3-devkitm-1/`.

The runner checks actual extracted firmware functions for settings validation, parser/storage failures, calibration retention, acquisition warmup/timeouts, startup and Run/Pause, receiver influence, task scheduling, shared travel/direction preservation, packet identity and acknowledgements, remote setup snapshots, calibration retries, fixed center and manual pulses. The JavaScript suite checks shipped UI syntax, embedded-asset equality, drafts, target scope, missing replies, shared fields, status colors and recovery. Standalone C++ suites check bounded sensor waits and recorder statistics.

`run_controller_r5_tests.py` and `run_controller_r52_calibration_tests.py` supply inherited literal fixtures to the current tests; do not run those historical entry points as R5.5 acceptance. Older published test entry points may remain in branch history and are superseded by this runner. Firmware functions are read from the current root `src/` and `include/`.

Publication adaptations change only test path resolution from the local investigation layout to the repository root and dependency paths to this build. No firmware source or test expectations are replaced. Tests do not establish RF delivery, actual execution rate, servo angles, water tightness, cold boot, loaded control or sailing stability.
