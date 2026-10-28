# R5.5 field acceptance — not yet performed

The purpose is to validate the prepared candidate on the actual paired boat and retain enough evidence to improve it. Do not assume a successful build or simulated page establishes hardware behavior.

## Installation and setup

1. Record each board's identity, role, current firmware hash, saved tuning, sensor calibration and peer list. Verify the retained R5.5 binary against BUILD-MANIFEST.json and available application partition capacity.
2. Update the slave, then the master, using a newly prepared exact-target application-only helper. Preserve filesystem configuration. Verify the installed build/hash and compare settings/calibration/peers after each restart. R5.4.1 helpers are historical and reject or install the wrong image for this job.
3. Join the master's Wi-Fi. Confirm that both actual name suffixes and roles appear, and that each reports fresh status and its own saved values.
4. Run/Pause both, then each individually. Verify actual output matches each reported state. Pause intentionally disables output; restart returns to automatic operation when calibration/settings are usable.
5. Calibrate each sensor from the master, one at a time. Finish & save, verify each own range, then Run. Confirm a calibration failure does not silently replace a valid prior range.
6. Apply a small common tuning change and then a known travel range. Verify each board's actual readback, individual receipt and retained direction/calibration. Travel changes pause affected boards; resume explicitly.
7. Center at 1500. Use small manual-reference steps on each foil; confirm actual pulse and AoA direction. Recheck linkage clearance across the intended travel. Manual test values bypass normal travel limits.
8. Power off/on both without connecting to Wi-Fi. Confirm automatic control and settings retention. Disconnect/reconnect a slave and check that stale status, unconfirmed commands and recovery remain truthful.

## Sensor and on-water checks

- Repeat dry, shallow and full-depth holds after a soak; collect 30-second CSVs from each board directly. Record depth/movement, insulation and wet/dry state with each file.
- Inspect the sensing-strip edges, back face and wire junction; repeat after mast/trunk movement. The existing rebate must protect insulation from rubbing. Ground contact with water is intentional.
- Compare level versus depth and actual commanded pulse versus measured foil angle. A stable sensor trace and known servo direction precede P/D tuning under load.
- Begin controlled water trials with conservative travel and recorded settings. Record onset of oscillation, asymmetric response, receiver influence, timeout counts and any loss of link. Change one tuning variable at a time.

## Result template

Date / firmware hash / board suffix and role / saved calibration / P-I-D / Set Point / receiver influence / endpoint order / insulation version / depth and motion / soak time / good samples and timeouts / actual pulse and observed AoA / link behavior / result / next change.

Acceptance remains unchecked until observed. This document does not initiate an update or command either servo.
