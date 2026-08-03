# Development Plan and Current Status

This document tracks implemented capabilities separately from validation and hardware completion. Range, latency, rider count, power, and ride stability are not established until their exit criteria have been measured.

[TOC]

---

## Current Baseline

### Implemented in Firmware

- ESP32-S3 audio capture/playback, VOX/DTX, Opus, jitter/PLC handling, and three-source remote mixing
- First-speaker receive-slot retention with 400 ms silent-source eviction
- ESP-NOW mesh scheduling, election, join/leave, control windows, liveness, relay handling, and telemetry
- Relay grants capped at two active speakers
- nRF52840 ESB transport configured for 2 Mbps and +8 dBm
- ESP32/nRF SPI audio and control bridge with a 2 ms nRF polling thread
- Dual-MCU startup, transport detection, command/ACK handling, and shared wire definitions
- Host tests for transport-neutral mesh, bridge, membership, bundling, and audio helper logic

These features are current implementation, not proof of field reliability. Multi-node RF, end-to-end audio, long-duration, and fault-recovery validation remain ongoing.

### Rev2 Hardware Status

The Rev2 KiCad design exists under `schematics/omi-rev2` and includes the ESP32-S3, bare nRF52840, nRF21540 FEM, audio path, USB-C, BQ24074 charger, TPS63020 regulator, and controls.

Rev2 is not completed hardware. Outstanding work includes:

- Keep the currently clean schematic ERC result at zero unexplained violations
- Close PCB DRC, routing, RF, clearance, return-path, and manufacturing-rule findings
- Generate and review Gerbers, drills, placement files, BOM, and assembly drawings
- Build and validate a bare-nRF52840 firmware target instead of relying on the XIAO board definition
- Implement and validate nRF21540 GPIO/SPI control and RF state timing
- Bring up charger, regulator, battery protection assumptions, rail sequencing, and current limits
- Validate native USB flashing/recovery and boot-strapping behavior
- Assemble prototypes and bring up each subsystem before field testing

---

## Guiding Rules

1. No range, latency, power, or runtime claim without a reproducible measurement.
2. No RF power-amplifier testing without conducted/radiated instrumentation and legal-limit review.
3. No mesh scaling claim before stable two-node and four-node tests.
4. No battery testing before charger/regulator bring-up and thermal checks.
5. A KiCad design is not complete hardware until ERC/DRC, manufacturing review, assembly, and bring-up pass.

---

## Milestone 1: Reproducible Firmware Baseline

**Status:** Feature implementation is current; validation and documentation remain active.

Tasks:

- Build ESP32-S3 firmware with ESP-IDF v5.5+
- Build XIAO nRF52840 firmware with the pinned nRF Connect SDK/Zephyr environment
- Run host C/Python tests and compile shared wire headers in CI
- Record firmware versions and configuration with every test result

Exit criteria:

- Clean builds from a fresh checkout
- CI covers both MCU targets and host tests
- Flash and recovery procedures are repeatable on development boards

---

## Milestone 2: Dual-MCU Bench Validation

**Status:** Core transport and audio/control bridge are implemented; validation is ongoing.

Tasks:

- Verify sustained 2 ms SPI polling and transaction error counters
- Exercise START/STOP generation, ACK retry, reconnect, and stale-frame handling
- Measure end-to-end audio latency and packet delivery under simultaneous TX/RX
- Inject MCU resets, bridge disconnects, queue pressure, and malformed frames
- Verify three-source mixer retention/eviction and two-speaker relay grants on devices

Exit criteria:

- Two dual-MCU nodes pass a documented multi-hour bench run
- No uncontrolled queue growth, deadlock, or persistent split state after fault injection
- Audio latency and delivery reports include method, sample count, and worst case

---

## Milestone 3: Bare nRF52840 Target

**Status:** Not started for Rev2 hardware.

Tasks:

- Add a Zephyr board/Devicetree target for the Rev2 bare nRF52840
- Define clocks, flash, USB, SPI bridge, WS synchronization, RF, and FEM pins from the schematic
- Verify boot, logging, reset, high-frequency and low-frequency clocks, and radio operation
- Remove XIAO-specific pin and bootloader assumptions from the Rev2 build

Exit criteria:

- Rev2 nRF firmware builds without the XIAO board target
- Flash, reset, and recovery work on assembled hardware
- SPI bridge and ESB smoke tests pass on the bare device

---

## Milestone 4: Rev2 Power Bring-Up

**Status:** Schematic and layout exist; hardware behavior is unvalidated.

Tasks:

- Review BQ24074 and TPS63020 component values, footprints, thermal paths, and layout against datasheets
- Power rails from a current-limited supply before connecting a cell
- Validate USB input, charge current, termination, NTC behavior, power path, and fault cases
- Validate 3.3 V startup, ripple, transient response, efficiency, and thermal margin
- Measure each firmware power state and transport mode as specified in `docs/power.md`

Exit criteria:

- Charger and regulator tests pass across expected input, battery, and load ranges
- No unsafe thermal or fault behavior
- Whole-device power measurements replace estimates in the power document

---

## Milestone 5: FEM Control and RF Validation

**Status:** nRF21540 is present in Rev2; firmware control is not validated.

Tasks:

- Implement `FEM_TX_EN`, `FEM_RX_EN`, `FEM_MODE`, `FEM_PDN`, and SPI control
- Meet nRF21540 setup/hold timing around every ESB RX/TX transition
- Provide a bypass/disabled mode for baseline nRF52840 operation
- Validate conducted output, harmonics, receive behavior, antenna match, coexistence, and legal power limits
- Compare range and power with FEM disabled/enabled under the same test conditions

Exit criteria:

- No TX/RX overlap or unsafe FEM state during boot, reset, or faults
- Measured RF results and test configuration are recorded
- Firmware defaults are legal and thermally acceptable for the target region

---

## Milestone 6: USB Recovery and Manufacturing Readiness

**Status:** Not validated.

Tasks:

- Validate ESP32 native USB bootloader entry, flash, serial logging, and recovery on Rev2
- Validate nRF USB only after the bare target and board routing are verified
- Confirm boot/reset straps remain recoverable after assembly
- Keep ERC clean, close PCB DRC, and perform independent schematic/layout review
- Generate reproducible Gerber, drill, BOM, centroid, and assembly outputs
- Run fabrication-house rule checks and inspect the final production archive

Exit criteria:

- Both MCUs have a tested recovery path
- ERC/DRC and manufacturing review have no unexplained blockers
- A versioned manufacturing archive can reproduce the reviewed board

---

## Milestone 7: Mesh, Ride, and Power Validation

**Status:** Not established.

Tasks:

- Two-node RF and audio baseline, followed by four-node and then six-to-eight-node tests
- Packet loss, synchronization drift, relay behavior, mixer behavior, and end-to-end latency measurement
- Long-duration bench runs before controlled ride tests
- Representative battery discharge tests with recorded audio level, speech duty cycle, node count, transport, and FEM mode
- Environmental, vibration, connector, enclosure, and thermal evaluation

Exit criteria:

- Published results include raw logs, firmware revision, hardware revision, and test method
- Rider-count, range, latency, and runtime claims do not exceed measured evidence
- Recovery from node loss, reset, and temporary RF interruption is demonstrated

---

## Later Work

- Bluetooth Classic/HFP phone and legacy-intercom integration
- Mesh authentication, encryption, provisioning, and key rotation
- Enclosure, waterproofing, controls, and production test fixtures
- User-selectable ESP-NOW/ESB transport with clear interoperability limits

These items should not block basic Rev2 electrical and dual-MCU validation.

---

## Anti-Goals

- Music streaming
- Stereo mesh audio
- Cloud connectivity
- App-first user experience

These features are outside the current reliability and power scope.

---

## Project Review Conditions

Stop or change the design if measured results show any of these conditions:

- End-to-end latency remains above 120 ms after the audio and transport paths are tuned.
- Representative continuous-use battery runtime remains below 8 hours.
- RF instability remains unresolved after the mesh validation milestone.

These are decision thresholds. They are not claims about current performance.

---

## Plan Authority

This document is the execution plan for the project. Complete milestone exit
criteria before using a later milestone as evidence of readiness. Update this
plan when implementation or measured results change.
