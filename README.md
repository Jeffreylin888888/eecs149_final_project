# EECS149 MSP432 motor and robot-arm repository

Standalone MSP432P401R / TI-RSLK MAX firmware, based on the Lab12_Motors and RobotArm examples linked from TI's `sekp166.pdf`. The Mac builds and offline tests pass. **The new firmware has not been flashed or physically tested.** Earlier LED programming/blinking succeeded; earlier wheel firmware produced no confirmed movement.

Connecting USB does not automatically start the robot. Motors need the chassis battery supply. Arm operation needs a verified servo supply, wiring and measured pulse limits. The application starts disarmed and requires a fresh S1 press/release with a live host connection. S2, any bump, host timeout, and reset stop outputs.

## Included implementations

- LED blink and individual left/right wheel diagnostic firmware.
- Lab12 forward/backward/left/right patterns, limited to 20% duty for 200 ms each.
- Signed motor PWM, 1 ms stop monitoring, independent motor command expiry, watchdog.
- Encoder capture, bounded distance/turn control, stall/timeout detection.
- Three 50 Hz servo outputs, frame-boundary updates, per-joint limits and slew rates.
- Optional ADC feedback for all three arm channels.
- Serial manual joint control, five calibrated poses, wave and pick/drop/return recipe execution.
- Strict command parsing, physical arming, host heartbeat, fault reporting.
- Original Python planning/simulation retained. It is **not automatically connected to physical navigation**.

`reference/tirslk/` contains unchanged upstream examples and license notices; their original exercise blanks are reference material, not compiled application code. This repository implements the relevant motor/arm behavior in `firmware/msp432/`, not all other labs in the ZIP. The OLED menu is replaced with Mac serial control. Old Buckler firmware and documents are historical only.

## Build and test

From this repository:

```sh
bash tools/setup_msp432.sh
bash tools/build_msp432.sh
python3 -m unittest discover -s tests -v
python3 -m eecs149_robot demo
```

Requires Arm GNU Embedded GCC, OpenOCD, Python 3.10+, and TI SimpleLink MSP432P4 SDK 3.40.01.02. These are already installed on the current Mac. The local SDK symlink reuses the original project's SDK; a clean checkout uses `setup_msp432.sh` to download it. The SDK is not redistributed.

Build outputs: `firmware/msp432/build/{led,wheels,control}.elf` and `.bin`. **Never flash `control-fixture.elf`: its calibration is synthetic test data.** The upload helper only accepts the three real targets.

## Next hardware session

Keep chassis power off while checking mounting and connections. Remove the LaunchPad 5V jumper before chassis mounting; verify aligned sockets, common ground, supply polarity, and loose wires. Use the confirmed six 1.2 V NiMH cells; support wheels clear of the surface and keep servos disconnected for wheel testing.

After target/overwrite confirmation and physical checks, upload with:

```sh
bash tools/flash_msp432.sh control --confirmed-target-and-overwrite --confirmed-chassis-safe-wheels-lifted
```

This backs up main flash twice, compares the copies, verifies device identity, programs only the required flash sectors and verifies the new image. It replaces the previous application. No upload was performed while preparing this repository.

Install the optional serial dependency in a virtual environment, then list ports:

```sh
python3 -m venv .venv
.venv/bin/pip install -e '.[serial]'
.venv/bin/python -m serial.tools.list_ports -v
```

Use the XDS110 application/backchannel port (not its auxiliary data port):

```sh
.venv/bin/python tools/robot_control.py --port /dev/cu.YOUR_XDS110_PORT status
.venv/bin/python tools/robot_control.py --port /dev/cu.YOUR_XDS110_PORT pwm 20 0 200
.venv/bin/python tools/robot_control.py --port /dev/cu.YOUR_XDS110_PORT pwm 0 20 200
.venv/bin/python tools/robot_control.py --port /dev/cu.YOUR_XDS110_PORT lab12
```

Each motion command asks for physical S1 arming. Blue RGB = disarmed, green = armed, red = fault. Direction, power and motor response must be observed before ground driving. `status` reports raw encoder counts and fault bits. See [control protocol and calibration](docs/CONTROL.md).

## Arm and closed-loop drive

Defaults deliberately disable servo output, ADC feedback, and encoder-controlled movement until calibrated in `firmware/msp432/src/control/config.h`. Manual bounded wheel PWM is available. No invented servo positions are included.

After verified physical calibration, rebuild/reflash and use `servo CHANNEL MICROSECONDS`, `pose INDEX`, `move LEFT_TICKS RIGHT_TICKS TIMEOUT_MS`, or `recipe config/wave.json` / `recipe config/pick-drop-return.json`. Recipes remain disabled until their measured parameters and `calibrated` flag are set. Successful pulse completion is not proof of joint motion or a grasp; the pick/drop recipe also checks a measured grip-feedback range.

See [test evidence](docs/TEST_RESULTS.md), [hardware setup](docs/HARDWARE_SETUP.md), and [upstream provenance](reference/PROVENANCE.md). LED source: `firmware/msp432/src/main.c` with `WHEEL_TEST=0`.
