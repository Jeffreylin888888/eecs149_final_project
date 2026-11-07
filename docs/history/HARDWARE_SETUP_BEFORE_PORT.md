# MSP432 / TI-RSLK MAX bring-up

Status 2026-11-06: user confirms standalone MSP-EXP432P401R, not mounted; chassis and servos unpowered. USB debugger and target identified; LED firmware flashed, verified and blinking visually confirmed. User confirms 5V jumper removed; mounting, supply and physical motor/arm wiring remain UNVERIFIED. This supersedes the Buckler instructions for this target. Do not flash Buckler binaries.

## Tools and reproduction

Installed Arm GNU 15.3.Rel2 and Homebrew OpenOCD 0.12.0, native arm64 macOS. The installed `board/ti_msp432_launchpad.cfg` uses XDS110/SWD and the MSP432 flash driver. No CCS or probe firmware update needed so far. TI documents MSP432 and XDS110 support in CCS v11 on macOS, but this does not establish compatibility with this Mac's current OS; we use the working CLI route.

TI SimpleLink MSP432P4 SDK **3.40.01.02** is under `firmware/msp432/vendor/simplelink_msp432p4_sdk_3_40_01_02/simplelink_msp432p4_sdk_3_40_01_02`. Build uses TI's device/CMSIS headers, startup, system initialization, linker layout and DriverLib archive. No Buckler dependency. Override `SDK` in make if relocated.

Run `tools/build_msp432.sh` for both images and native logic tests. `tools/probe_msp432.sh` connects and halts without firmware writes or resuming unknown code. `tools/debug_msp432.sh` exposes local GDB port 3333; connect with `arm-none-eabi-gdb firmware/msp432/build/led.elf`, then `target remote localhost:3333`. Keep motor/servo power disconnected during debugging; halted CPUs can leave hardware PWM active.

Flash only after explicit target/overwrite confirmation:

```
tools/flash_msp432.sh led --confirmed-target-and-overwrite
```

Script guards production P401x / 256 KiB identity, reads and verifies a full main-flash backup, then erases only sectors needed for the image, programs and verifies before reset/run. Existing application is replaced even though most flash is untouched. No mass erase, debug unlock, BSL, information-flash or probe-firmware update. Backups do not constitute a full device-security/configuration clone. Original backup is in `firmware/msp432/backups/original-main-before-first-flash.bin`.

## LED test

Standalone board: USB to onboard XDS110, onboard target/debug jumpers in normal configuration. Red LED P1.0 toggles every 500 ms. Motor sleep pins are driven low and PWM outputs disabled. No servo outputs are configured. Success requires the user's visual confirmation after upload; debug register observations alone are not that confirmation.

## Motor connections (documentation verified, physical wiring pending)

| Function | MSP432 pin | LaunchPad header |
|---|---|---|
| Left direction / DRV8838 PH | P5.4 | J3.29 |
| Left PWM / EN / TA0.4 | P2.7 | J4.40 |
| Left nSLEEP | P3.7 | J4.31 |
| Right direction / PH | P5.5 | J3.30 |
| Right PWM / EN / TA0.3 | P2.6 | J4.39 |
| Right nSLEEP | P3.6 | J2.11 |
| Left encoder A/B | P10.5 / P5.2 | J5 / J2.12 |
| Right encoder A/B | P10.4 / P5.0 | J5 / J2.13 |

The chassis schematic specifies two DRV8838 drivers. Direction low requests forward per TI; actual rotation remains to be observed. Disable drives nSLEEP low and EN low: expect coasting, not guaranteed instantaneous braking. Encoder inputs currently expose raw levels in `encoder_levels`; quadrature counting and closed-loop control are not implemented.

Before mounting or chassis power: remove the LaunchPad **5V isolation jumper**. TI warns of permanent damage if left connected on this chassis. With all power disconnected, user must confirm correct board orientation and full header alignment, exact chassis revision, battery chemistry/count/voltage, polarity, supply current capacity, common ground, motor plugs, and both endpoints of every loose wire. No motor/servo power may go to GPIO. Do not infer loose-wire destinations. Keep servos disconnected for wheel testing.

Before flashing wheel firmware or applying motor power, obtain these confirmations and **raise/support the chassis with both wheels clear**. Keep the power switch accessible. After separately approved wheel flash, release S1, press for at least 50 ms, then release: left 20% PWM for 300 ms, 1 s stopped, right 20% for 300 ms, permanently disabled until reset. Timer_A0 runs at 1 kHz from explicitly configured 3 MHz SMCLK. Low duty may not overcome friction; do not automatically increase it on a stall.

In firmware revision 2, S2 (P1.4) or any bump switch (P4.0/2/3/5/6/7, active low) inhibits startup and discards any partial start gesture. Once the sequence starts, either input latches abort until reset; inputs are sampled every millisecond. Independent SysTick motor deadline also disables after 300 ticks. ACLK/REFO watchdog (~250 ms) recovers a stuck foreground or missing SysTick; main clears it only on tick progress. This is diagnostic software, not a certified safety controller. Reset requires a new button sequence; it does not automatically start motors.

## Arm: deliberately unenergized

Physical servo channel mapping, actual supply, current budget and assembly positions are unknown. No servo signal pin or neutral angle is guessed; no arm output is enabled. Before implementing/energizing 50 Hz PWM, confirm each lift/tilt/gripper model and signal header, lead order, feedback-wire handling, ground, voltage/current, mechanical support and joint clearance. Establish separate minimum/maximum/start pulse calibration per servo and test one joint at a time. Removing pulses may release support or may not stop every servo immediately; support the arm and retain a physical power cutoff.

Pololu specifies 4.8–6 V nominal for the supplied servos; lift and tilt can each briefly draw about 1.8 A, gripper stall about 0.8 A at 6 V. Do not attach directly to six-cell raw battery voltage. Actual board regulator and simultaneous load capacity must be checked before use. A generic 1500 us midpoint is not an established safe assembly pose.

## Primary references

- TI RSLK MAX user guide (pin tables): https://www.ti.com/lit/ml/sekp166/sekp166.pdf
- TI motor lab (5V warning and motor interfaces): https://www.ti.com.cn/cn/lit/pdf/sekp121
- Manufacturer chassis schematic, inspected pages 1–2: https://www.pololu.com/file/0J1670/ti-rslk-max-chassis-board-v1.0-schematic.pdf
- Pololu Romi Arm guide: https://www.pololu.com/docs/0J76/all
- TI SDK official download: https://www.ti.com/tool/download/SIMPLELINK-MSP432-SDK/3.40.01.02
- TI CCS v11 macOS support: https://software-dl.ti.com/ccs/esd/documents/ccsv11_macos_host_support.html
- TI-hosted board/OpenOCD guidance: https://software-dl.ti.com/zephyr-sdk/esd/MSPM33/01_00_00_02/exports/zephyr/boards/ti/msp_exp432p401r_launchxl/doc/index.html
- OpenOCD target classification/registers: https://github.com/openocd-org/openocd/blob/v0.12.0/src/flash/nor/msp432.c

## Revision 2 status and troubleshooting

RGB LED: green = ready for S1 press/release; blue = wheel sequence (including the pause); cyan = finished; red = waiting for release/stop input active, or aborted after a run started. The separate red P1.0 heartbeat still blinks. No automatic movement on USB connection, power-on, or clearing an input. After a pre-start stop clears, release S1, then make a fresh press-and-release. After active-run abort or completion, reset with S3 for another attempt.

Use `python3 tools/status_msp432.py` to read live state without halt/reset/resume. It compares flash against the built image before interpreting symbol addresses. Telemetry records current and historical stop inputs, the abort reason, start-button edges, and which motor commands were started. Command flags do not prove electrical PWM or physical rotation. If green never appears, inspect stop inputs; if cyan appears but wheels do not move, next investigate chassis power, voltage at the motor drivers, connector seating and PWM at the verified pins. Do not automatically raise duty cycle without these observations.

Revision 2 is built and offline-tested but not yet flashed as of this update; the disconnected robot retains revision 1. Reconnect USB with chassis power off and wheels supported for the next upload.
