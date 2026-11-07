# MSP432 test evidence — 2026-11-06

| Milestone | Result | Evidence |
|---|---|---|
| MCU firmware builds | PASS, LED and wheels, warnings are errors | `firmware/msp432/logs/build-test.log` |
| Offline diagnostic and motor tests | PASS (native C with UBSan; registers mocked) | Same log |
| Existing simulation | PASS, 8 tests | `firmware/msp432/logs/simulation.log` |
| XDS110 enumeration | PASS, firmware 3.0.0.22, VID/PID 0451/bef3 | `logs/target-identification.log` under firmware directory |
| Actual target connection | PASS, SWD DPIDR 2ba01477, Cortex-M4 r0p1, P401x Rev D, ID A000, revision 44, 256 KiB flash | Same log |
| Original main-flash backup | PASS, 262144 bytes dumped and verified | `logs/original-backup.log` |
| LED upload | PASS, programmed and Verified OK after explicit user approval | `logs/flash-led-20261106-155429.log` |
| Physical LED blink | PASS | User: “Yes, the red LED is blinking” |
| Left/right movement | Pending | Chassis not mounted or powered |
| Arm movement | Pending | Wiring, supply, clearances and calibration unverified |

Commands executed: `brew install open-ocd`; official TI SDK macOS installer in unattended mode with project-local prefix; `make -C firmware/msp432 clean`; `tools/build_msp432.sh`; `python3 -m unittest discover -s tests -v`; shell syntax checks; `arm-none-eabi-readelf -l`; `arm-none-eabi-nm -u`; `ioreg -p IOUSB -w 0`; OpenOCD identify/read/backup/verify commands.

LED image: text 1836, data 4, BSS 16 bytes. Wheel image: text 2432, data 4, BSS 24 bytes. Both binaries fit entirely in the first 4 KiB flash sector. ELF layout uses main flash at 0, initialized data at SRAM 0x20000000 with flash load address, no information-flash load segments, stack top 0x20010000. No unresolved symbols. Hardware peripherals and real timing remain unverified until uploaded and observed.

Native tests cover debounce, held-at-boot button, one-shot sequence, all phase timing boundaries, clock wrap, invalid state, abort in every state, delayed foreground, both motor pin mappings, PWM configuration, independent 300-tick shutdown, S2 and six bump stops. These tests do not emulate Cortex-M4 instructions, electrical waveforms, or mechanics. Apple's installed Clang 14 AddressSanitizer failed inside its runtime initialization on this macOS before test execution; tests were rerun successfully with UndefinedBehaviorSanitizer. This is not an ASan pass.

Original main backup SHA-256:
`7b0b4d7f66c5ff0660eece9f791973938949ed38e50ecab68613913d5c075c9f`.

No hardware movement has been claimed. Prior eight Python tests remain simulation-only. Arm and encoder closed-loop implementation are outside this initial open-loop bring-up and remain unfinished.

Live run check: `logs/led-running.log` reads ticks at ELF symbol address 0x2000000c: 0x5c3c then 0x5e9a (606 ms across a requested 600 ms wait). P1OUT bit 0 toggled (df to de); P2OUT=3f and P3OUT=1d leave PWM/sleep bits 6–7 low. These are device observations, not visual LED confirmation. Board left running the approved LED firmware.

User confirms LaunchPad 5V jumper removed and authorizes a wheel test. Mounting, actual supply, full wiring checks and wheels-lifted confirmation remain pending; wheel firmware has not been flashed.

Latest user direction is to finish as quickly as possible. This does not establish mounting, electrical supply, wiring, or wheel clearance. No wheel image was flashed and no motion was initiated. LED firmware remains running. Next required evidence: physical setup confirmation plus battery/power specification.

## Continued testing

Reconnected successfully to device A000 / silicon 44. Read back all 1840 bytes of the LED image without halting or resetting the board; `cmp` matched the built binary exactly. Both SHA-256 values are `1f8a05a761e166ad7215299c9aa35d1950633eaaba454276316cf438667a3f6d`. Evidence: `firmware/msp432/logs/continued-board-test.log` and `led-readback.bin`.

Live ticks increased from 0x61c66 to 0x61e5f (505 ms during a requested 500 ms wait). P1.0 changed low to high; P2.6/7 PWM outputs and P3.6/7 motor-sleep outputs remained low. Native diagnostic and mock-register motor tests passed again with UBSan. No additional firmware was flashed; no wheel or servo motion was commanded. Physical chassis/power/clearance confirmation remains required.

## Mounted, USB-only check

User reports LaunchPad mounted, battery removed, USB connected. OpenOCD reconnects to A000/44; the 1840-byte LED image readback exactly matches the build. Live ticks advance from 0x2727 to 0x2921 (506 ms), and PWM/sleep outputs remain low. Evidence: `logs/mounted-usb-test.log`. Mounting alignment has not yet been visually checked; battery specification and lifted-wheel confirmation are pending. No motor image flashed or motion commanded.

## Wheel firmware upload

User confirmed mounting/alignment checks, wheels lifted, 5V jumper removed, and six 1.2 V NiMH AA cells (7.2 V nominal). Wheel test authorized. Flashed wheels.elf: `logs/flash-wheels-20261106-161116.log` reports Verified OK. Separate 2436-byte readback exactly matches wheels.bin (`logs/wheels-ready.log`). State=1 (WAIT_PRESS), stop_latched=0, bump inputs high and motor sleep bits low. No physical movement result yet.

Backup CRC verification encountered a target reset/algorithm timeout, then OpenOCD's fallback verified all 262144 bytes. Programming subsequently succeeded. Flash script now uses two independent reads plus host cmp to avoid executing the target CRC helper while the application watchdog is active.

## First physical wheel attempt

User reports neither wheel moved. Live read `logs/wheels-no-motion.log`: diagnostic state 7 (ABORTED), stop_latched=1, Timer_A0 stopped. Current S2 and bump inputs are released; historical triggering input is not recorded by this firmware. Ticks advanced 509 ms across the read interval. No movement success claimed. Requested S3 reset with bump switches clear to distinguish an input abort from a power/drive issue before changing PWM.

## Revision 2 offline debugging (robot unplugged)

Identified a software cause of persistent pre-start lockout: SysTick latched every S2/bump assertion from boot, including transients before the operator intended to start. The previous observation proves a stop event occurred, but cannot identify its source or prove that this is the only cause of no movement.

Changed pre-start stop handling to inhibit and restart the full release/press/release gesture, while preserving latched active-sequence abort, direct motor-enable input checks, per-motor 300 ms cutoff, watchdog, one-shot completion, and conservative 20% PWM. Added RGB state indication and stop-source/start-command telemetry. Added `tools/status_msp432.py`, which verifies the flash image before decoding live symbols.

Both images compile with warnings as errors. Revision 2 wheel image: text 2696, data 8, BSS 36 bytes (2704-byte flash binary). `logs/revision2-build-test.log` records passing UBSan native tests covering pre-start stop recovery, held-start inhibition, fresh start gesture, active-run abort, all motor input interlocks, timing and pin routing. All eight existing simulation tests pass (`logs/revision2-simulation.log`). No unresolved ELF symbols. Status utility passes Python syntax compilation; live operation awaits reconnection.

NOT FLASHED: the user disconnected the robot. Revision 1 remains on the board. Physical wheel motion, motor supply and PWM waveforms are still unverified.
