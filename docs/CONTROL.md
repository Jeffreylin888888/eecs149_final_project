# Serial control and calibration

9600 baud, 8N1, ASCII uppercase commands terminated by LF (CR ignored). Maximum line 79 characters. XDS110 backchannel uses UCA0, P1.2 RX and P1.3 TX. Keep RX/TX isolation jumpers connected. The host performs synchronous request/reply and sends PING while waiting. No autonomous movement occurs on reset or USB insertion.

| Command | Result / meaning |
|---|---|
| `PING` | `OK`; refresh 500 ms host lease |
| `STATUS` | `STATUS armed fault left right move overruns settled` |
| `STOP` | Stop wheels, remove servo pulses, disarm |
| `CLEAR` | Clear latched fault only while disarmed and stop inputs released; new S1 gesture required |
| `PWM left right ms` | Signed duty -25..25%, duration 1..250 ms |
| `MOVE left right timeout_ms` | Relative signed 2x encoder ticks, each +/-100000; timeout 100..30000 ms; calibration required |
| `SERVO channel microseconds` | Lift 0, tilt 1, gripper 2; calibrated per-channel bounds |
| `POSE index` | Home 0, approach 1, grasp 2, carry 3, release 4 |
| `FEEDBACK` | `FEEDBACK lift tilt gripper` raw 14-bit ADC, or `ERR` if unavailable/stale |

Commands return `OK` or `ERR` except STATUS/FEEDBACK. Rejected actuator commands disarm and stop. Servo/pose commands are rejected while wheels are active. Wheel commands retain already-enabled servo holding pulses. A latched stop always removes all pulses; this may release a load. Support the arm and provide a physical power cutoff.

`move`: 0 idle, 1 running, 2 done, 3 timeout, 4 stall, 5 invalid. Error within +/-2 ticks per wheel completes the movement. No observed encoder changes for 500 ms on an unfinished wheel causes stall. Raw PWM diagnostic does not depend on encoder feedback.

Fault bits: 0/2/3/5/6/7 = corresponding P4 bump pin; 8 = S2; 10 = host heartbeat timeout; 11 = UART error; 12 = encoder overrun/invalid delta; 13 = movement stall/timeout. Encoder overrun is cumulative and requires reset after the cause is corrected. STATUS reports raw signed counter bits, not calibrated distances.

RGB: blue disarmed; green armed; red latched fault. Press/release S1 only after the host announces arming. Each release/press/release phase needs at least 50 ms. The standalone `wheels` target has a different RGB sequence documented in its source and historical bring-up notes.

## Calibration, before enabling the arm

1. Verify physical servo mapping and lead order, 4.8–6 V supply and peak-current capacity, common ground, support and clearance. Raw six-cell battery voltage is not servo power. Confirm feedback voltage is within MCU AVCC limits before connecting A21–A23.
2. Record measured safe min/max/start pulse widths and maximum change per 20 ms frame for each servo. Do not copy `tests/calibration_fixture.h`: those values exist only to test compilation/logic. A removed-pulse restart begins from configured start width; arrange the supported assembly at the verified startup position before enabling again.
3. Update `SERVO_CALIBRATION` and `SERVO_WIRING_VERIFIED` in `src/control/config.h`; first test individual joints with conservative measured limits. Five `ARM_POSES` entries must each contain three measured widths. No pose is accepted if any channel is invalid. Pulse slew completion does not prove mechanical arrival.
4. Enable `FEEDBACK_WIRING_VERIFIED` only after electrical verification. ADC reads are nonblocking and expire after 100 ms without a completed sequence. ADC values are uncalibrated position observations, not torque or current. For a grasp, measure a grip-feedback acceptance range for the actual object and enter it in the recipe; do not infer success from pulses alone.
5. With wheels lifted, verify positive PWM physical direction and both encoder signs. Set `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` to +1 or -1 so positive physical wheel movement counts positive. Count both A edges (2x decoding); do not use unrelated 4x encoder constants. Set `DRIVE_CALIBRATED=1` only when verified. Measure ticks per distance/turn, then fill all zero movement placeholders in the recipe.
6. Validate every recipe step and clearance, then set its `calibrated` flag. Rebuild/reflash firmware after header changes. Host recipes require no flash changes. The full pick/drop route remains a measured sequence, not camera navigation or arbitrary obstacle avoidance; any bump aborts it.

Firmware calibration values can alternatively be supplied via `ROBOT_CALIBRATION_HEADER`; `config.h` preserves externally provided macros. Tests use that mechanism only for synthetic fixtures. Keep a dated record of actual hardware measurements with the repository.

The firmware watchdog resets on stalled foreground/tick progress. Motor leases expire in SysTick even if foreground is delayed. A debugger halt can freeze the safety code while timer outputs continue; remove motor/servo power before halting. This software is not a substitute for mechanical support or a power switch.
