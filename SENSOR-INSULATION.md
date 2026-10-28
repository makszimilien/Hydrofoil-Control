# Sensor insulation findings — October 21, 2026

## Governing construction requirement

> **“The sensor strip must be electrically isolated at all costs!”**

The green sensing electrode must be insulated from the surrounding water on **both faces, every edge, both ends, and the wire/solder junction**. Insulate its backside from the printed mast too. Do not rely on the 3D print, copper adhesive, or front-only tape as the electrical barrier.

The **black ground electrode intentionally contacts the water**. This requirement concerns the sensing electrode; do not mistakenly insulate the ground as part of the sensing design.

## What we tested and learned

The final mast is Bambu Lab PC with a G10 core and longer copper electrodes than the working PLA prototype. The original short sensors worked on :28 when reattached, but different leads were used; that comparison did not isolate every variable. Board failures must not be inferred simply from these remote-sensor timeouts.

Three distinct insulation versions were tested: original very thin PET packing tape; substantially thicker PVC electrical tape; then replacement PET packing tape, slightly heavier than the original but still thin. Do not merge these histories.

- Thin/front-covered configurations produced immersion-dependent failures. Shallower or out-of-water conditions could work. An early electrical-tape run completed 1,768 readings with zero timeouts, typically about 15 ms. An initial replacement-PET run completed 2,811 readings with zero timeouts, median 0.8745 ms. The latter was reportedly about 90% as deep, so this was not a controlled material comparison. Neither initial success established durable sealing.
- With replacement PET and no reported sensor movement, a later phone test produced 569 timeouts/zero good readings. A Mac repeat produced 566 timeouts/zero good readings. Power-cycling only the PCB did not recover it: 564 timeouts/zero good readings, with restart verified from uptime and counters. This was not merely a phone-page problem.
- User clarified the screenshot sequence: recording 2 OUT (1,632 good/2 timeouts), recording 3 IN (0 good/567 timeouts), recording 4 OUT (1,780 good/0 timeouts). It recovered while still dripping wet without needing a reboot or drying interval. At that stage the correct conclusion was **immersion-dependent failure, not yet a proven leak**.
- Cyclical dipping CSV: 1,261 good/303 timeouts, with repeated recovery and readings reaching 48.54 ms before the 50 ms cutoff. Timeout values are censored observations, not completed charge measurements.
- T2 increased the cutoff to 500 ms. One slow dip yielded 700 good/44 consecutive timeouts; roughly 22 seconds were spent in failed acquisitions. Only one completed measurement exceeded 50 ms, about 101 ms during recovery. More waiting did not solve the submerged failure.
- C1 used creator-style tight pin polling, retaining periodic interrupt release and a 500 ms cutoff for diagnostic access. It was not the original unlimited interrupt-disabled firmware. It yielded 1,140 good/36 consecutive timeouts, about 18 seconds of failed acquisition. Different hand-dip timing prevents treating fewer timeouts as improvement. User retired this experiment.
- **Breakthrough: tape on BOTH sides of the sensing strip, followed by return to the same T1/50 ms firmware that previously failed.** The helper recorded 2,949 good readings/zero timeouts, maximum 0.373025 ms. The immediate user follow-up recorded 2,951 good/zero timeouts, maximum 0.38945 ms.
- Six supplied user recordings (2 through 7) total **17,732 good readings and ZERO timeouts**, maximum **0.40845 ms**. Recordings 3–5 stayed approximately 0.275–0.324 ms. These followed the discussion of a full-depth soak; individual depth/movement timelines and exact soak duration were not recorded. Recording 2 is the same follow-up mentioned above, not an additional independent test. Including the separate helper recording gives **20,681 unique good samples and zero timeouts** across seven captures.

The repeatable improvement after insulating both sides strongly supports an unwanted water-related electrical path involving the sensing copper as the practical root cause. Exact ingress location was not measured. The working theory is water passing through the print to the backside of the copper; water creeping around an edge remains another possible route. Do not claim PC itself is conductive, that a specific pore was found, or that indefinite waterproofing has been established.

Instant recovery does not exclude a water-mediated conductive path: lifting the sensor can break the path through the grounded body of water while droplets remain. Earlier resistance measurements were sometimes contaminated by fingers touching conductors. A later hands-off submerged reading was overrange on the meter; this did not prove adequate insulation for the sensing circuit.


## Where the project is now

The successful sealing recordings used diagnostic T1/50 ms. The project later returned to normal control and reached verified R5.4.1 on both boards. R5.5 is prepared but uninstalled. Its normal sensing cutoff remains approximately 2 ms. The sealing evidence is not a R5.5 hardware test.



The mast **already has recessed copper-strip gutters/rebates**. Do not propose adding them as if absent. Check that copper plus insulation stays below the surrounding bearing surface where the mast passes through the trunk. Electrical success does not prove resistance to rubbing or abrasion.

The PET sandwich is the working reference. A proposed construction uses WEST SYSTEM epoxy with clear hardener: base coats on the dried PC print, sand smooth without breaking through, copper laid down, then continuous top coats overlapping the base beyond all edges and sealing the connection. A very fine glass veil is optional reinforcement, not by itself the water barrier. No epoxy version has yet been built or tested, and PC/copper adhesion must be checked on representative material. Multiple coats reduce reliance on one pinhole-free film; do not promise a single thin coat will seal it.

Thinner insulation is not inherently faster: for otherwise equal geometry/material, reducing dielectric thickness generally increases capacitive coupling. Balance sensitivity, complete insulation and abrasion protection. The successful PET readings show no present need to reduce coating thickness for speed.

Future checks: repeat immersion after coating cure, soak/retest, repeated wet/dry cycles, inspect edges and wire entry, and test trunk/mast rubbing at actual travel. Exact lifespan and salt-water performance remain unverified. Do not start CAD edits, coating purchases, firmware changes or calibration automatically from these notes.
