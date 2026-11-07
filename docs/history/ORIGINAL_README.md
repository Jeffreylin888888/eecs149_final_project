# Active hardware target: MSP432P401R

Use `tools/build_msp432.sh` and `docs/HARDWARE_SETUP.md` for the real TI-RSLK MAX controller. Results are in `docs/TEST_RESULTS.md`. The Buckler material below is retained as historical work and is not compatible firmware for this board.

# EECS149 mobile manipulation with specification-driven control

A **separate, standalone project** for the November 4 EECS149 project proposal:
TI-RSLK MAX mobile robot + Berkeley Buckler/nRF52832 + Pololu Romi 3-servo arm.

**Status:** simulator and offline controller tests pass; real motor/arm execution is
**not plug-and-play** until wiring, arm mounting, sensor calibration, and board mapping are verified.
The firmware's outputs are disabled by default. No batteries or USB are needed for the steps below.

## Start immediately (MacBook or Ubuntu)

```bash
cd eecs149_robot_complete
python3 -m eecs149_robot demo
python3 -m unittest discover -s tests -v
python3 -m eecs149_robot synthesize --item medicine --recipient alice
```

Python 3.10+ stdlib only. Optional camera marker detection:
`python3 -m pip install -e '.[vision]'`.

## Buckler firmware build (on your already configured Mac)

Easiest route: `bash tools/build_on_mac.sh` (build only, no flash).

Alternatively, run the same steps manually:

```bash
mkdir -p ~/buckler/software/apps/eecs149_robot
cp -R firmware/buckler/. ~/buckler/software/apps/eecs149_robot/
cd ~/buckler/software/apps/eecs149_robot
make clean && make
```

You should see an `_build/eecs149_robot_sdk15_s132.hex` file. With
`ROBOT_WIRING_VERIFIED=0` and `ROBOT_ARM_CALIBRATED=0`, only Buckler LEDs are
controlled. Do **not** flash before checking the physical wiring and understanding
that flashing overwrites the existing firmware. See [HARDWARE_BRINGUP](docs/HARDWARE_BRINGUP.md).

## What is synthesized?

We synthesize a shortest path in the **product** of a discrete grid transition
system (moves respecting obstacle locations) and a deterministic finite task
monitor. The implemented temporal-logic fragment is

`G(!collision) & F(PICK(item) & X F(DROP(user) & X F(AtBase)))`

where picking/dropping are explicit actions with preconditions. This handles
ordered reachability tasks with invariants and discrete step deadlines; it is
not a general LTL-to-Buchi synthesizer or continuous STL controller synthesis.

`eecs149_robot/stl.py` additionally provides bounded `G[a,b]` and `F[a,b]`
robustness monitors with conservative incomplete-window semantics.

## Layout

- `eecs149_robot/model.py`: world and requests
- `eecs149_robot/synthesis.py`: product-state BFS automaton synthesis, validator
- `eecs149_robot/scheduler.py`: priority dispatcher
- `eecs149_robot/motion.py`: go-to-goal and differential-drive odometry
- `eecs149_robot/manipulation.py`: three-servo arm API and mock sequences
- `eecs149_robot/vision.py`: optional external-camera ArUco detector
- `eecs149_robot/simulator.py`: execution, collision/precondition assertions
- `firmware/buckler/`: **disabled-by-default**, pin-configurable nRF52 firmware
- `tests/`: offline deterministic regression suite
- `docs/`: bring-up checklist and proposal coverage

## Publishing as a NEW GitHub repo

This ZIP does not modify the existing `Jeffreylin888888/eecs149_final_project` repo.
After extracting, run `bash tools/publish_new_repo.sh` if GitHub CLI is installed and authenticated.
Or do the same steps manually:

```bash
cd eecs149_robot_complete
git init -b main
git add .
git commit -m "Initial EECS149 specification-driven robot prototype"
# If GitHub CLI is installed and authenticated:
gh repo create eecs149_robot_complete --public --source=. --remote=origin --push
```

`gh auth login` may be required first. A remote repository has NOT been created
by simply downloading this package.

## Safety and claims

Never guess wiring, energize servos before mechanical calibration, or drive
wheels on a tabletop. The real-world milestones in the proposal require
additional engineering, and the host Python simulator does not automatically
send commands to the board. The physical firmware implements a guarded
motor/servo **diagnostic**, not autonomous exploration.
