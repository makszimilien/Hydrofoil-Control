# What comes next

## Next: validate R5.5 as built

Install on both boards with configuration preserved; verify remote setup and calibration, direction-aware manual tests, shared travel and per-board confirmations from one Wi-Fi. Check a real cold start, missing-peer recovery, full-depth sensor soak and insulation abrasion. Then evaluate response under load and at the water's edge, keeping settings and recordings with each result. See FIELD-TEST-PLAN.md.

## Decisions already made

Keep the phone UI concise. Control owns shared Run/Pause; Tune owns per-board/Both settings and individual calibration; Test owns servo alignment and diagnostic links. Use actual name suffixes and explicit state text. Center is always 1500 µs. Shared travel preserves per-board direction; sensor calibration remains local. Normal control follows the water sensor; receiver influence remains available. No background settings push at boot.

## Improvements to consider after evidence

- Measured sensor-to-output response and PID tuning under the real foil load.
- Boat-specific behavior during persistent sensor faults; the current behavior holds the last command.
- Durability of the insulation system, sealed wire entry and abrasion protection in the existing rebate.
- Remote recorder access or firmware updates only if direct access proves inconvenient; these are currently board-local.
- Further UI changes guided by actual waterside use, avoiding duplicate controls and unreadable status churn.

These are candidates, not promises or authorized changes to CAD, sensors or control behavior. Keep the present candidate fixed during initial comparisons so physical changes are not confused with firmware changes.
