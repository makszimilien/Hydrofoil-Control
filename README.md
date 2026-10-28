# Hydrofoil Control R5.5 — one Wi-Fi for normal setup

**October 28, 2026 · experimental field-test candidate. Built and host-tested; R5.5 is not yet installed or tested on the boat.** The two project boards still run R5.4.1. R5.5 is a development revision of this fork, not an upstream official release.

R5.5 brings everyday two-board setup onto the master's Wi-Fi: control, tuning, calibration, live readings and actual saved settings. It retains application-only Wi-Fi updates and the 30-second diagnostic recorder. Each board still runs its own sensor and servo loop; the master is a configuration interface, not a source of continuous servo commands for the slave.

## At the water's edge

After both boards have R5.5 and have been paired in Device setup, join the **master's Wi-Fi** and open **http://192.168.4.1/**. The heading shows the actual network-name suffix and Master/Slave role; suffixes are discovered, never hardcoded.

- **Control:** see each board's state and Run/Pause both. Amber means paused; red means a fault or missing current status. Text identifies the state as well as color. Pause switches servo output off; it does not change saved tuning. Normal control starts automatically at power-on when settings and calibration are usable.
- **Set up & tune:** choose either board's name button, or Both. Individual views show that board's actual saved settings, fresh sensor/servo readings, calibration, Run/Pause and direction. Edits remain drafts until Apply. Both applies common P/I/D, Set Point, receiver influence and, by default, travel bounds. Different saved values remain blank and marked Different until a common value is entered.
- **Test:** center both at exactly **1500 microseconds**, or enter a reference pulse for one servo. Center ignores trim and direction. Manual values use the receiving board's saved endpoint order: reversed boards mirror around 1500, so reference 1600 becomes actual 1400. The actual command is displayed. Manual tests bypass travel limits, so begin near center and work outward while checking the linkage. End test & run/pause returns to normal operation or switches output off.

Calibration is individual: select a board, Start, move its sensor from fully out of water to the intended immersion depth and back a few times, then Finish & save. A usable candidate replaces that board's saved range; no usable new range retains the previous range. Finish leaves that board paused. A restarted/interrupted calibration needs Finish followed by Start for a new session. Calibration is never copied between boards.

Shared travel uses lower/upper bounds while retaining each board's direction. For example, 1200–1800 becomes 1800/1200 on a reversed board and 1200/1800 on a normal board. A travel change pauses the affected board; use Run in Control afterward. Shared saves are confirmed independently, not atomically or simultaneously. Unconfirmed means no matching reply was received; the change may have arrived. Inspect current readback before retrying. There is no automatic settings push at boot or pairing.

Normal operation always uses the water-height sensor. **Set Point** uses the calibrated 1000–2000 scale; 1500 is its midpoint. Receiver influence remains an optional offset to that target; 0% adds none and a stale receiver adds zero. Receiver-only mode is removed. Legacy mode fields remain in storage/protocol layouts and normalize to sensor control without rewriting other settings at boot.

## What still needs direct board Wi-Fi

Firmware updates, initial role/peer setup, recovery, detailed diagnostics and full 30-second recordings still use the board being accessed. The recorder is under Test → Recording & diagnostics and explicitly names its connected board. Both boards must have this R5.5 revision before relying on its remote setup, shared travel and name discovery.

## Sensor construction is fundamental

**The sensor strip must be electrically isolated at all costs.** Seal the sensing copper on both faces, every edge, both ends and the wire junction, including its backside against the printed mast. The ground electrode intentionally contacts water.

Front-only tape and longer timeouts did not cure immersion-dependent failures. Insulating both faces produced 17,732 good readings and zero timeouts across six user recordings; a separate helper capture brings the unique total to 20,681. These were T1/50 ms diagnostic tests, not R5.5 tests. The precise ingress route was not proven. See [sensor findings](SENSOR-INSULATION.md). Coating abrasion, long soaking and salt-water operation remain to be checked.

## Build and checks

The target is ESP32-C3, PlatformIO `esp32-c3-devkitm-1`, `espressif32@6.12.0`, Arduino ESP32 2.0.17. Tested with PlatformIO Core 6.2.0. Install PlatformIO in a Python virtual environment, then:

```sh
pio run -e esp32-c3-devkitm-1
python3 tests/run_all_r55_tests.py
```

Tests require Python 3, clang++ with UndefinedBehaviorSanitizer and Node.js. Build first to resolve QuickPID and ArduinoJson. [Test instructions](tests/README.md) describe coverage and portability adjustments. [TEST-RESULTS.md](TEST-RESULTS.md) distinguishes software checks from physical evidence. [BUILD-MANIFEST.json](BUILD-MANIFEST.json) records the retained candidate binary, dependency versions and firmware-input hashes. Build-path/timestamp differences can change a rebuilt binary's hash.

The application is `.pio/build/esp32-c3-devkitm-1/firmware.bin`. UI assets are embedded in the application; `data/` contains matching source and generic blank credentials/empty peers. **Do not upload the filesystem over an existing board's saved configuration.** Existing-device installation is application-only after checking exact identity, chip, partition capacity and current firmware. Normal control may start after the update reboot, just as it does at power-on. No universal uploader or complete-flash rollback is included.

## Timing and limits

Normal sensing retains the approximately **2 ms** bounded acquisition, target 500 Hz sampling and 100 Hz PID cadence. Twenty fresh successful readings are required after a timeout before filtered control resumes. A normal-control sensor timeout holds the last commanded servo pulse; it does not automatically switch output off. The separate 30-second recorder targets 100 Hz and records actual timestamps. Diagnostic T1/50 ms, T2/500 ms and creator-style C1 experiments are retired and are not this controller.

Remote snapshots expire after 3.5 seconds. Stale state is shown as unconfirmed and editing is disabled. Explicit Apply, calibration and test commands use matched acknowledgements, bounded retries and duplicate/stale-command checks. A displayed servo pulse is a command, not measured shaft position or foil angle.

R5.5 has no physical two-board, cold-power-cycle, loaded or on-water acceptance yet. Its inherited HTTP/OTA access and ESP-NOW trust model are for a controlled development setup, not an authenticated deployment. [FIELD-TEST-PLAN.md](FIELD-TEST-PLAN.md) is the next acceptance record, not a claim that testing has occurred.

## History and next work

[CHANGELOG.md](CHANGELOG.md) records R5 through R5.5 and the retired sensing experiments. [ROADMAP.md](ROADMAP.md) separates immediate field acceptance from possible later improvements. The working R5.4.1 setup already demonstrated calibration, confirmed tuning transfer and the intended opposite electrical pulses producing the same physical AoA direction. Those observations do not validate the new R5.5 remote protocol on hardware.

## Attribution and license

Based on Tamas Nagy's Hydrofoil Control. The upstream [LICENSE](LICENSE) is retained verbatim. Its title says MIT License, but its text includes additional non-commercial, attribution and hardware-exclusion conditions; this project does not relicense the original software or hardware. PlatformIO libraries retain their own licenses.
