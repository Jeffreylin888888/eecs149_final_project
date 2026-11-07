# MSP432 / TI-RSLK MAX hardware setup

Current status: user disconnected the robot after a wheel test produced no movement. The new control firmware was prepared offline. Earlier target identification, LED flash/readback and visible blink succeeded. The user previously confirmed mounting, 5V jumper removal, supported wheels and six 1.2 V rechargeable NiMH AA cells. Arm assembly, servo power, wiring and physical limits have not been verified. Do not use Buckler binaries.

## Verified documentation mapping

| Function | MSP432 pin / peripheral |
|---|---|
| Left direction / PWM / sleep | P5.4 / P2.7 TA0.4 / P3.7 |
| Right direction / PWM / sleep | P5.5 / P2.6 TA0.3 / P3.6 |
| Left encoder A/B | P10.5 TA3.1 / P5.2 |
| Right encoder A/B | P10.4 TA3.0 / P5.0 |
| Six bump inputs, active low | P4.0, P4.2, P4.3, P4.5, P4.6, P4.7 |
| Lift servo | P2.4 mapped to TA1.1 |
| Tilt servo | P3.5 mapped to TA1.2 |
| Gripper servo | P5.7 TA2.2 |
| Lift / tilt / gripper feedback | P8.4 A21 / P8.3 A22 / P8.2 A23 |
| XDS110 UART RX / TX | P1.2 / P1.3, UCA0, 9600 baud |
| S1 start / S2 stop / S3 reset | P1.1 / P1.4 / reset |
| Red LED / RGB | P1.0 / P2.0,1,2 |

Chassis uses two DRV8838 motor drivers. Direction low is documented forward; physical rotation remains unconfirmed. Stopping disables EN and nSLEEP, allowing coast rather than guaranteeing immediate braking. Motor PWM is 1 kHz at 3 MHz SMCLK. Servo timers divide 3 MHz by 3 and use 20,000-count frames (50 Hz).

## Power and mounting

Remove the LaunchPad **5V isolation jumper before mounting on the chassis**. With all power removed, verify orientation, fully aligned sockets, motor connections, loose-wire endpoints, polarity and common ground. LaunchPad USB faces the rear caster; front expansion header faces the bumps. USB powers/debugs the controller; battery power is required for wheel motion. Six 1.2 V NiMH cells are 7.2 V nominal. Confirm their charge, orientation and switch position during hardware testing.

For initial motion keep both wheels supported clear, servos disconnected, and power switch accessible. Do not increase PWM automatically when a wheel fails to move. Read fault/status first and inspect power and connections. No motor/servo supply goes to GPIO.

Pololu arm servos nominally require 4.8–6 V. Lift and tilt can each briefly draw about 1.8 A, gripper stall about 0.8 A at 6 V. Verify actual regulator/current capability before connecting. Do not connect servos to the raw six-cell supply. No generic 1500 us position is assumed mechanically safe. See CONTROL.md for calibration.

Keep motor/servo power disconnected during debugger halt: PWM peripherals may continue while CPU safety logic is stopped. Removing servo pulses may release a loaded arm; support it.

## Mac build/programming

Arm GNU 15.3.Rel2, OpenOCD 0.12.0, TI SimpleLink MSP432P4 SDK 3.40.01.02. No CCS or probe firmware update required. The scripts use XDS110 SWD and the MSP432 flash driver. `probe_msp432.sh` halts the CPU without writes; `debug_msp432.sh` opens GDB port 3333. These are not passive while a robot is moving.

`flash_msp432.sh` verifies the target, backs up all 256 KiB main flash twice and compares, then programs/verifies required sectors. Existing application is overwritten. No mass erase, unlock, BSL or information-flash changes. Original backup remains in the original Desktop project's `firmware/msp432/backups/`; it was intentionally not copied into the public-ready repository.

## Sources

- [TI MAX guide, supplied PDF](https://www.ti.com/lit/ml/sekp166/sekp166.pdf), pin tables and page 4 software archive.
- [TI-RSLK MAX chassis schematic](https://www.pololu.com/file/0J1670/ti-rslk-max-chassis-board-v1.0-schematic.pdf).
- [Pololu Romi Arm user guide](https://www.pololu.com/docs/0J76/all).
- [TI SDK 3.40.01.02](https://www.ti.com/tool/download/SIMPLELINK-MSP432-SDK/3.40.01.02).
- Preserved upstream `RobotArm` / `inc/PWM.c` / `inc/ADC14.c` pin and peripheral configuration, with original license notices.
