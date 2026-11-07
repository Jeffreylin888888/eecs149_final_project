> Historical Buckler bring-up only. For the active MSP432 target, use HARDWARE_SETUP.md.

# Hardware bring-up (MUST follow in order)

This repo does **not** know the actual nRF52832-to-TI-RSLK MAX wire mapping. The original
Buckler is generally used with other robots (such as Kobuki), and the photographed TI-RSLK
chassis and arm are a custom integration. Do not reuse the MSP432 LaunchPad pinout.

1. **Power OFF**, disconnect batteries and USB. Inspect the loose yellow wire; locate both ends,
   identify connector labels with a multimeter and verify voltage/power rails before reconnecting.
2. Verify exact boards and available GPIO pins, motor driver input labels `DIRL`, `PWML`,
   `nSLPL`, `DIRR`, `PWMR`, `nSLPR`, encoder `ELA/ELB/ERA/ERB`, and servo outputs `SRV0..2`.
   The shown six-wire motor control pinout on RSLK MAX is *not* mapped to nRF pins yet.
3. Servo power must be compatible with your actual servos and must not be supplied from a GPIO
   output. Connect signal/5V/GND in the correct order, share the ground with controller, and
   account for peak current. Check controller↔servo logic levels and supply isolation.
4. Copy `firmware/buckler` to `~/buckler/software/apps/eecs149_robot`, run `make`. Default
   firmware blinks Buckler LEDs only and does not touch motors/servos.
5. Connect a **data-capable** USB cable to J-Link port, confirm USB enumeration. Preserve
   existing firmware if needed before `make flash`. It is a *destructive overwrite*.
6. Calibrate encoders for wheel direction / counts per revolution, wheel radius and wheelbase.
   Test wheel PWM with the chassis **off the floor** and a physical kill switch available.
7. Only after continuity and direction tests: edit `robot_wiring.h` with verified nRF pin
   definitions, enable `ROBOT_WIRING_VERIFIED`, keep `ROBOT_ARM_CALIBRATED=0`, and compile.
   Confirm no pins conflict with Buckler LEDs, LCD, SD card, switches, UART, or sensors.
8. Enable `ROBOT_ALLOW_BUTTON_DEMO=1` only for attended, brief, off-ground wheel tests.
9. Assemble the arm, establish mechanical joint limits and servo pulse width calibration,
   then enable `ROBOT_ARM_CALIBRATED` and set physical poses; servo signals not energized
   in default build.
10. Before autonomous movement integrate **encoder feedback, emergency-stop/timeout, obstacle
    sensing, measured calibration, and marker/camera pose**. The guarded diagnostic firmware
    is not yet the hardware navigation executor.

## Exact toolchain/build

```bash
cd ~/buckler/software/apps
cp -R /path/to/eecs149_robot_complete/firmware/buckler eecs149_robot
cd eecs149_robot
make clean && make
```

This uses the already installed ARM GCC 15.3 and SEGGER tools. Build warnings from old
Nordic SDK/Newlib do not necessarily indicate failure; check for `HEX` and `SIZE` lines.

## Outstanding hardware data

- Photo/label showing the lower controller board and any RSLK adapter board
- Continuity map from nRF GPIOs to all six motor inputs
- Encoder signal wiring and direction/cpr calibration
- Servo power source (voltage/current), individual servo channel assignment, calibrated limits
- Camera type, mounting orientation, and tag-to-world calibration
- Physical obstacle sensing / emergency stop

## Hardware documentation

- TI RSLK motor chassis/DRV8838 documentation: https://www.pololu.com/file/0J1670/ti-rslk-max-chassis-board-v1.0-schematic.pdf
- Pololu Romi arm guide: https://www.pololu.com/docs/0J76/all
- Berkeley Buckler source: https://github.com/lab11/buckler
