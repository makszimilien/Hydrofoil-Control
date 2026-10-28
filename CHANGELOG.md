# Development history through R5.5

These are local fork revision labels. Dates and observations distinguish installed builds from prepared candidates.

| Stage | Change and lesson | Evidence/status |
| --- | --- | --- |
| R2–R4, Oct 19–20 | Bounded sensor waits kept Wi-Fi diagnostics reachable; added single/continuous sensing, 30-second raw/median recordings and CSV; cached static diagnostic metadata. | Bench records retained locally. Long/failed reads are faults, not completed large measurements. |
| R5 / R5.1, Oct 20 | Integrated normal controller with diagnostics, embedded pages and explicit defaults recovery. | Two-board bench work verified acquisition and configuration responsiveness, not sailing performance. Initial source published in draft PR #3. |
| R5.2, Oct 21 | Fixed our calibration lifecycle: separate candidate range, reliable OFF, save usable range or retain previous calibration, report save failures. | Calibration correction published on the same contribution branch. The creator's original toggle was not claimed to contain our integration bug. |
| T1 / T2 / C1, Oct 21 | Diagnostic 50 ms, 500 ms and a creator-style polling experiment explored immersion timeouts. | 500 ms and creator-style polling did not solve submerged failures. All are retired diagnostic experiments. |
| Sensor sealing breakthrough, Oct 21 | Insulated sensing copper on both sides, including against the mast. | Six user captures: 17,732 good / zero timeouts; plus helper: 20,681 unique good / zero. Exact ingress path unknown. |
| R5.2P / normal return, Oct 27 | Returned the boat to the normal 2 ms controller while retaining saved settings/calibration. | Port and starboard brought back to normal control. A loose starboard sensor connection was repaired by the user. |
| R5.3, Oct 27 | Explicit calibration/Apply/Run/Pause controls and versioned, confirmed master-to-slave tuning transfers. | Installed on both boards. User confirmed saved calibration and peer settings delivery. No automatic startup settings push. |
| R5.3.1, Oct 27 | Automatic normal startup from usable local saved settings/calibration. Pause remains until restart. | Both OTA restarts verified active output. Separate cold-power-cycle acceptance still pending. |
| R5.4, Oct 27 | Fixed 1500 µs center, independent manual pulses and typed endpoints with range hints. | Installed on both boards; center operation observed. |
| R5.4.1, Oct 27 | Manual reference pulses honor each recipient's saved direction; acknowledgements report actual output. | Both boards installed and verified. After correcting saved direction, reference 1600 increased both foils' AoA: port actual 1400, starboard actual 1600. Both centered at 1500 with approximately matching AoA. |
| R5.5, Oct 28 | Phone-first Control/Tune/Test; name suffix + role; board buttons/Both; shared travel with direction retained; remote calibration, live status and actual settings; sensor-only normal operation; amber pause/red faults; reduced duplicate controls/text. | Current field-test candidate. Source/build/host checks and simulated phone previews passed. Not installed; no R5.5 physical or on-water result yet. |

The firmware evolved to make faults diagnosable and setup explicit. The sensor failure was solved by electrical insulation, not by making the wait arbitrarily long. A functioning graph alone does not prove calibration, direction, loaded servo behavior or stability on water.

Older build notes and evidence are preserved as history. Earlier statements such as “T1 installed” or “R5.3 not installed” describe their dated checkpoints and must not override the current project handoff.
