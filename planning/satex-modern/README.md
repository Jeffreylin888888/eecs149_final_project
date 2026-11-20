# SatEX Modern

A runnable Python implementation of temporal-logic motion planning for teams of robots, based on Shoukry et al., **Linear Temporal Logic Motion Planning for Teams of Underactuated Robots Using Satisfiability Modulo Convex Programming**, IEEE CDC 2017 ([paper repository](https://escholarship.org/uc/item/2699090s), [DOI](https://doi.org/10.1109/CDC.2017.8263808)).

The planner combines Z3's discrete search with SciPy/HiGHS linear programming and learns conflicts when a proposed plan is physically infeasible. It runs without CPLEX. It supports linear robot dynamics, 2D/3D convex regions, input and state bounds, inter-robot separation, and temporal operators `!`, `&`, `|`, `X`, `F`, `G`, `U`, and `R`. It includes finite-trace planning and periodic infinite traces with full-state loop closure, plus a direct Z3 baseline, independent result validation, and offline trajectory viewers.

The supplied archive contains an older single-robot reach-avoid example. This project reimplements that scenario family and adds the paper's multi-robot and generic-LTL machinery. It is not an exact reproduction of the paper's tables: the archive does not contain the full experimental geometry or complete CDC 2017 benchmark code. See [implementation notes](docs/IMPLEMENTATION.md) for guarantees, differences, and limits.

The delivered snapshot includes six independently validated planner demos. See the [validation record](VALIDATION.md) for measured results and remaining verification limits.

## Run

Run the commands below from `planning/satex-modern/`. Requires Python 3.11+ and [uv](https://docs.astral.sh/uv/). On macOS, you can also double-click `run-demo.command` to install the locked environment and open the patrol demo.

```bash
uv sync --locked --extra dev
uv run pytest

# Single-robot motion through an obstacle workspace.
uv run satex demo --scenario reach-avoid --output results/reach-avoid

# An infinite mission whose repeated segment closes the entire robot state.
uv run satex demo --scenario patrol --output results/patrol

# Crossing paths and synchronized team visits with a counting proposition.
uv run satex demo --scenario crossing --output results/crossing
uv run satex demo --scenario team --output results/team

# A 3D example and an adaptation of the archive's grid benchmark.
uv run satex demo --scenario 3d --output results/3d
uv run satex demo --scenario legacy --output results/legacy
```

Successful runs write `problem.json`, `result.json`, `trajectory.png`, and `trajectory.html`. Open the HTML file directly in a browser to play or scrub the trajectory; it works offline. A 3D run has both an isometric and top-down view. The animation linearly interpolates displayed samples; all solver guarantees apply at the discrete samples.

```bash
# Search a bounded range of horizons for your own problem.
uv run satex solve examples/reach-avoid.json --min-horizon 3 --max-horizon 20 \
  --time-limit 60 --output results/custom

# Validate saved states, controls, region labels, collisions, and LTL separately.
uv run satex verify results/reach-avoid

# Compare the two backends on small examples; timings are for this machine.
uv run satex benchmark --output results/benchmark

# Direct Z3 baseline or alternate conflict strategy.
uv run satex demo --scenario reach-avoid --backend smt --output results/z3
uv run satex demo --scenario reach-avoid --strategy full --output results/full
```

`--horizon` requests one horizon. `solve` otherwise searches from `--min-horizon` to `--max-horizon`. `--mode finite` uses finite-trace LTL semantics; `--mode lasso` evaluates ordinary infinite LTL on a finite prefix and repeated cycle. A horizon of H has H labeled samples. A lasso result adds one physical endpoint equal to the chosen loop-start state.

Exit status is 0 for a validated plan, 2 for bounded infeasibility, and 3 for an unknown or resource-limited result. Invalid problem input returns 1; command-line usage errors return 2. Unsatisfiability at tested horizons is not a proof that no longer plan exists. `verify` returns 0 only when the independent validator accepts the saved result.

## Customize

Start with a file in `examples/`. Regions use halfspaces `A @ position <= b`; named adjacency lists define allowed transitions. Robots carry explicit discrete-time `A`, `B`, position-state indices, initial state, and bounds. The mission is a structured JSON expression tree: for example, `{"op":"eventually","args":[{"op":"atom","robot":"r1","region":"goal"}]}`. Counting predicates express how many robots must occupy a region simultaneously. The example files are self-contained and can be solved without importing Python code. Python helpers in `satex.models` and `satex.scenarios` construct workspaces and exact zero-order-hold integrator chains.

A robot may also specify a `terminal_state` mask. For a 2D double integrator ordered `[x, vx, y, vy]`, `[null, 0, null, 0]` requires zero final velocity while allowing either final position. In lasso mode this mask constrains the closing endpoint, which equals the selected loop-start state.

Without uv, create a Python 3.11+ environment with `python3 -m venv .venv`, activate it, then run `python -m pip install -e '.[dev]'`. The `satex` command and tests work the same way in that environment.

The primary implementation is in `src/satex/`: models and examples, LTL encoding/evaluation, the SAT/LP engine, planner, validator, CLI, and visualization. `docs/IMPLEMENTATION.md` explains how to interpret results and benchmark comparisons. This is a research motion-planning implementation; it does not implement robot hardware control or robust feedback under disturbances.

This product includes software developed by Cyber-Physical & Systems Lab at UCLA. Original provenance and the full legacy source notice are in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
