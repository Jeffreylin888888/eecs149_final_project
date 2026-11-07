# Test results — 2026-11-06

## New repository: offline verification

| Check | Result / evidence |
|---|---|
| LED, wheel diagnostic, serial-control firmware | Build succeeds with Arm GNU 15.3.Rel2, `-Wall -Wextra -Werror`; `firmware/msp432/logs/build-test.log` |
| Enabled arm/ADC/encoder calibration code | Separate synthetic fixture cross-compiles; **never flashed, not a calibration** |
| Native C tests | Five executables pass with UndefinedBehaviorSanitizer: diagnostics, motor registers, parser/motion core, servo registers, runtime safety |
| Python tests | 16 pass: eight retained simulator/planner tests and eight new serial/recipe tests; `logs/new-repo-host.log` |
| Simulator demo | Completes; `logs/new-repo-simulation.log` |
| Host CLI | `--help` succeeds; pyserial 3.5 and editable package installed in repository `.venv` |
| Python syntax / shell syntax | `compileall` and per-script `bash -n` pass |
| Control ELF link | No undefined symbols reported by `arm-none-eabi-nm -u` |
| New firmware flash / real motion | **Not attempted** while robot disconnected |

Memory usage from `arm-none-eabi-size` (bytes):

| Image | text | data | BSS |
|---|---:|---:|---:|
| LED | 1836 | 8 | 16 |
| Wheels | 2704 | 8 | 36 |
| Control, uncalibrated defaults | 6024 | 4 | 240 |
| Compile-only synthetic calibration fixture | 7112 | 4 | 268 |

Native tests cover release/press/release debounce, startup stop recovery, active abort, timing wrap, both motor routes, signed PWM bounds, lease expiry, S2/all bump stops, parser integer/length errors, servo bounds/slew/frame staging, movement convergence/stall/timeout, runtime heartbeat expiration across tick wrap, fault clearing, preservation of servo holding during wheel commands, host protocol errors/timeouts, and recipe/feedback rejection. GPIO/timer behavior is mocked; electrical timing and interrupt latency still need hardware observation. ADC and serial peripheral configurations are cross-compiled, not electrically validated.

## Earlier physical evidence (before this repository)

- XDS110 connected to MSP432P401x Rev D: device A000, silicon 44, 256 KiB main flash. `logs/target-identification.log`.
- Original main-flash backup saved and verified in the original Desktop project. No original user firmware is included in this repository.
- LED firmware programmed and verified; user confirmed blinking red LED. `logs/flash-led-20261106-155429.log`, `logs/continued-board-test.log`.
- First wheel image programmed and readback matched. User reported **neither wheel moved**. `logs/flash-wheels-20261106-161116.log`, `logs/wheels-ready.log`.
- Live diagnostic showed ABORTED with a latched stop; current stop inputs were released. Historical triggering input was not recorded. This does not prove battery power, driver wiring, or mechanical motion works. `logs/wheels-no-motion.log`.
- Offline revision removed permanent pre-start stop lockout while preserving active-run abort and requiring a fresh start gesture. Neither revised wheels nor new control firmware has been flashed.
- No physical arm movement has been confirmed.

Earlier detailed notes are preserved under `docs/history/`. This file supersedes their stale implementation-status statements.

## Remaining physical validation

Reconnect with chassis power off for target/upload checks after confirming the intended overwrite. Then verify the previously confirmed mounting/5V jumper, battery condition, supported wheels and disconnected servos before powered testing. Observe each wheel independently, direction, encoder response, S2/bump stop, heartbeat loss and reset behavior. A successful build is not evidence of movement.

Arm wiring, regulated supply/current budget, mechanical clearances, per-servo pulse calibration, feedback ranges and measured drive counts remain required. Defaults intentionally block those unverified features. The new repository completes the software port and offline tests; it does not establish plug-and-play hardware operation or full-project navigation.
