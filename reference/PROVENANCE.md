# Upstream provenance and port coverage

The user supplied `/Users/jeffxcal/sekp166.pdf`. Page 4 links the TI-RSLK MAX example archive hosted by Jonathan Valvano at the University of Texas:

https://users.ece.utexas.edu/~valvano/arm/tirslk_max_1_00_00.zip

Downloaded 2026-11-06; SHA-256:
`2e2e50002ca6a6f5a31d0156d6ef6703061c0913b198e8d7b9c77f093a19b1fc`

The full ZIP was downloaded to the parent workspace's `work/`. `tools/fetch_tirslk.sh` reproduces the download and checks that digest. This is a recorded download digest, not a publisher signature.

The ZIP contains Lab12_Motors and RobotArm directories, not a separate arm Git repository. Selected unchanged originals are under `reference/tirslk/`, with their original Valvano/TI/FreeBSD-style notices retained. Those originals include exercise blanks and are excluded from our build. SDK code is not vendored; its installer/license remains upstream.

| Upstream component | New implementation |
|---|---|
| MotorSimple / Lab12 timed directions | `src/motor.c`, `src/diagnostics.c`, host `lab12` command |
| Motor PWM driver | signed PWM and leases in `src/motor.c` |
| Tachometer / encoder drive | `src/control/encoder.c`, bounded controller in `core.c` |
| PWM_RobotArmInit / duty updates | `src/control/servo.c`, calibrated 50 Hz channels |
| ADC A21/A22/A23 | nonblocking optional `src/control/feedback.c` |
| Arm MANUAL | serial `SERVO`, `POSE`, `FEEDBACK` commands |
| Arm DEMO | validated `config/pick-drop-return.json` recipe, requires measured counts/poses/feedback |
| Arm WAVE | calibrated pose alternation in `config/wave.json` |
| OLED menu | replaced with UART command/status client; no OLED required |

All `src/` paths above are under `firmware/msp432/`. Original 99% duty/long-running exercises and preset servo arrays are not used as safe defaults. Old non-MAX board jumper-cutting instructions in the reference sources do not apply to this MAX chassis. This is a functional port, not a claim that every project in the archive was completed or tested.
