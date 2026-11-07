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
| Control firmware flash / real wheel motion | **Verified upload, matching readback; both wheels turned/stopped, user confirmed** |

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

## Successful reconnect and powered wheel test

On 2026-11-06 the connected probe reported XDS110 firmware 3.0.0.32, serial M4321005; target remained A000 / silicon 44 / 256 KiB. Current application differed from all repository images. Full main flash was saved, and the user explicitly authorized replacing it while confirming wheels lifted, 5V jumper removed and servos disconnected.

`logs/flash-control-20261106-164106.log` reports **Verified OK**. Two full backup reads compared before programming. The serial application port is `/dev/cu.usbmodemM43210051`; 9600-baud STATUS worked with zero faults.

After a physical S1 press/release, host sent `PWM 20 0 250`, paused, then `PWM 0 20 250`, and STOP. `logs/reconnect-wheel-session.json` records:

| Observation | Left encoder | Right encoder | Fault |
|---|---:|---:|---:|
| Before test | 0 | 0 | 0 |
| After left pulse | 242 | 0 | 0 |
| After right pulse | 242 | 258 | 0 |
| Final stopped/disarmed | 242 | 260 | 0 |

No encoder overruns occurred. The user explicitly confirmed: **“Yes—both turned and stopped.”** This establishes the individual wheel-motion milestone. Encoder counts alone were not used as a substitute for that confirmation.

Final read-only inspection: P3.OUT=0x09 (both motor sleep bits low), P2.SEL0=0 (motor PWM disconnected), TA0.CTL=0 (motor timer stopped). The full 6028-byte control image readback exactly matched the compiled binary. Evidence: `logs/control-after-test.log`.

Also corrected interrupt restoration in the standalone wheel diagnostic after its motor-start critical section. Added native interrupt-mask preservation assertions; all five C test programs pass (`logs/reconnect-build.log`). The successful physical session used the separate control target, not that standalone wheel diagnostic.

## Remaining physical validation

Forward/reverse orientation, loaded ground driving, physical S2/bump/heartbeat-stop tests, and calibrated encoder distances remain to be checked before navigation. Arm wiring, regulated supply/current budget, clearances, per-servo pulse calibration, feedback ranges and measured drive counts remain required. Defaults still block uncalibrated arm and closed-loop drive features. No arm motion or full-project navigation is claimed.
