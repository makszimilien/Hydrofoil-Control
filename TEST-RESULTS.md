# R5.5 verification — October 28, 2026

Status: experimental field-test candidate, not installed. Both project boards remain R5.4.1.

## Completed software checks

- ESP32-C3 PlatformIO build of the candidate: passed. Portable repository-root source build: passed.
- Portable `python3 tests/run_all_r55_tests.py`: passed. Includes shared settings/identity, acquisition and calibration, legacy sensor-mode normalization, remote setup/readback/calibration, center/manual tests, QuickPID scaling, filesystem failure/round-trip tests, validation, receiver influence, startup/modes, task/locking gates, standalone sensor wait/recorder, and UI behavior/embedded assets.
- Firmware-input files in the public package match the prepared local source byte-for-byte. LICENSE remains unchanged. Test path adaptations are documented in tests/README.md.
- Simulated phone views for running, paused, fault, first calibration, calibration in progress and offline states were inspected during development. Per-board colors and mixed states were reviewed. Preview data cannot command a real board.

Candidate binary and source hashes are recorded in BUILD-MANIFEST.json. Host tests use mocked peripherals and storage/radio fault injection; they are not a timing or electrical simulation.

## Earlier physical evidence, not R5.5 acceptance

Both boards received R5.4.1 with settings/roles/peers/calibration preserved and active normal control after OTA restart. User confirmed calibration and settings-transfer behavior in earlier R5.3 work. After correcting direction, a manual reference of 1600 increased both foils' AoA: port actual 1400, starboard actual 1600. Center 1500 produced roughly matching AoA. Electrical pulse readouts are commanded values; physical AoA was observed by the user, not instrumented here.

Sensor sealing: six T1/50 ms user captures contained 17,732 good readings/zero timeouts; with a separate helper capture, 20,681 unique good readings/zero timeouts. They followed sealing both faces of the sensing strip. Exact ingress route and indefinite durability were not established.

## Still unverified

R5.5 installation, real paired-radio remote setup/calibration, current source's physical startup/recovery, standalone cold power cycle, measured loaded response, final travel clearance, long-term coating durability and on-water stability. Use FIELD-TEST-PLAN.md to record these results. No board update or servo command was performed during publication.
