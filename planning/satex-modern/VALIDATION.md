# Validation record

Validated locally on November 20, 2026, with Python 3.12.12 on macOS arm64.

- 120 planner tests passed. Coverage includes LTL semantics, model validation, conflict learning, endpoint/loop constraints, independent validation, and offline CLI behavior.
- Ruff lint and formatting checks passed.
- The locked environment synchronized successfully. The distributable wheel built successfully and contains the required third-party notice.
- All six demos returned SAT and passed independent numerical and LTL validation.
- All 30 benchmark runs (five scenarios × two backends × three trials) returned SAT and passed validation.
- Six HTML viewers passed JavaScript syntax checks; 2D and 3D PNG outputs were inspected. Browser playback was not verified because the browser tool blocks local-file URLs.

## Repository publication check

The planner-only repository copy was checked separately on November 20, 2026: all 120 tests, Ruff lint/format, locked environment setup, wheel build, and a fresh team demo with independent verification passed. The repository's existing 30 Python tests, five native firmware test binaries, teleoperation UI test, and offline robot demo also passed. These checks did not operate physical hardware. Benchmark timings below remain the original recorded measurements.

## Demos

The repository includes saved problems, plans, and verification reports. Run the demo commands to regenerate the PNG and offline HTML viewers.

| Scenario | Horizon | Semantics | Valid |
|---|---:|---|---|
| [reach-avoid](results/reach-avoid/verification.json) | 8 | finite | Yes |
| [patrol](results/patrol/verification.json) | 12 | lasso | Yes |
| [crossing](results/crossing/verification.json) | 8 | finite | Yes |
| [team](results/team/verification.json) | 12 | lasso | Yes |
| [3d](results/3d/verification.json) | 8 | finite | Yes |
| [legacy](results/legacy/verification.json) | 12 | finite | Yes |

## Local benchmark medians

These small examples are illustrative, not a reproduction of the paper’s performance tables. They do not establish general speed superiority; direct SMT is faster on several of these small instances.

| Scenario | SMC (s) | Direct SMT (s) |
|---|---:|---:|
| reach-avoid | 0.1210 | 0.0210 |
| crossing | 0.0614 | 0.0360 |
| patrol | 0.4313 | 0.1544 |
| team | 0.2363 | 0.1117 |
| 3d | 0.0251 | 0.0542 |

Recorded benchmark measurements and environment details: [benchmark JSON](results/benchmark/benchmark.json), [CSV](results/benchmark/benchmark.csv), and [validation summary](results/validation-summary.json). The GitHub Actions workflow is supplied for Python 3.11–3.13; hosted CI has not been executed.

## Reproduce

Run from `planning/satex-modern/`:

```bash
uv sync --locked --extra dev
uv run pytest -q
uv run ruff check src tests
uv run ruff format --check src tests
uv run satex benchmark --scenarios reach-avoid crossing patrol team 3d --repeats 3
uv run satex demo --scenario team --output results/team
uv run satex verify results/team
```

The models guarantee sampled discrete-time constraints within the recorded numerical tolerance. The viewer interpolates samples for display only. See [implementation notes](docs/IMPLEMENTATION.md) for finite-trace semantics, lasso assumptions, and unreproduced original benchmark details.
