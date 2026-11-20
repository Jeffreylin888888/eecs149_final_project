"""Command-line entry points for planning, validation, and small comparisons."""

from __future__ import annotations

import argparse
import csv
import json
import platform
import sys
import time
from importlib.metadata import version
from pathlib import Path

from . import __version__
from .models import Problem
from .planner import Planner, PlanResult
from .scenarios import (
    legacy_reach_avoid,
    multi_robot_crossing,
    multi_robot_patrol,
    reach_avoid,
    team_recurrence,
    three_dimensional,
)
from .validation import validate_plan
from .visualization import save_visualizations

SCENARIOS = {
    "reach-avoid": (reach_avoid, 8, "finite"),
    "patrol": (multi_robot_patrol, 12, "lasso"),
    "crossing": (multi_robot_crossing, 8, "finite"),
    "team": (team_recurrence, 12, "lasso"),
    "3d": (three_dimensional, 8, "finite"),
    "legacy": (legacy_reach_avoid, 12, "finite"),
}


def _positive_int(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def _positive_float(value: str) -> float:
    import math

    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def _planning_arguments(parser: argparse.ArgumentParser, *, demo: bool) -> None:
    parser.add_argument("--horizon", type=_positive_int, help="solve exactly H labeled samples")
    parser.add_argument("--mode", choices=("finite", "lasso"), default=None if demo else "finite")
    parser.add_argument("--backend", choices=("smc", "smt"), default="smc")
    parser.add_argument("--strategy", choices=("deletion", "prefix", "full"), default="deletion")
    parser.add_argument(
        "--time-limit",
        type=_positive_float,
        default=60.0,
        help="total solver/search budget in seconds (default: 60)",
    )
    parser.add_argument("--max-iterations", type=_positive_int, default=2000)
    parser.add_argument(
        "--conflict-budget",
        type=_positive_int,
        default=100,
        help="LP checks available to shrink each conflict",
    )
    parser.add_argument("--output", type=Path, default=Path("results/demo" if demo else "results/solve"))
    parser.add_argument("--no-visuals", action="store_true", help="save JSON without plotting or animation")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="satex", description="Sampled LTL motion planning with Z3 and HiGHS"
    )
    parser.add_argument("--version", action="version", version=f"satex-modern {__version__}")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="solve a built-in scenario and save its trajectory")
    demo.add_argument("--scenario", choices=tuple(SCENARIOS), default="reach-avoid")
    _planning_arguments(demo, demo=True)
    solve = commands.add_parser("solve", help="solve a serialized Problem JSON file")
    solve.add_argument("problem", type=Path)
    solve.add_argument("--min-horizon", type=_positive_int, default=2)
    solve.add_argument("--max-horizon", type=_positive_int, default=20)
    _planning_arguments(solve, demo=False)
    verify = commands.add_parser("verify", help="independently validate a saved problem/result directory")
    verify.add_argument("directory", type=Path, help="directory containing problem.json and result.json")
    verify.add_argument("--tolerance", type=_positive_float, default=1e-6)
    benchmark = commands.add_parser("benchmark", help="compare both backends on small built-in problems")
    benchmark.add_argument("--output", type=Path, default=Path("results/benchmark"))
    benchmark.add_argument(
        "--time-limit", type=_positive_float, default=20.0, help="seconds allowed per case/backend/trial"
    )
    benchmark.add_argument("--repetitions", "--repeats", type=_positive_int, default=1)
    benchmark.add_argument("--scenarios", nargs="+", choices=tuple(SCENARIOS), default=["reach-avoid", "3d"])
    return parser


def _write_json(path: Path, data: dict | list) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _save(problem: Problem, result: PlanResult, output: Path, visuals: bool = True) -> None:
    output.mkdir(parents=True, exist_ok=True)
    # A new attempt invalidates all earlier generated views and verification.
    # Otherwise an UNSAT rerun could still display an old successful path.
    for name in ("verification.json", "trajectory.png", "trajectory.html"):
        (output / name).unlink(missing_ok=True)
    problem.save(output / "problem.json")
    result.save(output / "result.json")
    if result.status == "sat" and visuals:
        save_visualizations(problem, result, output)


def _planning(args: argparse.Namespace) -> int:
    if args.command == "demo":
        factory, default_horizon, default_mode = SCENARIOS[args.scenario]
        problem = factory()
        mode = args.mode or default_mode
        horizon = args.horizon or default_horizon
    else:
        problem = Problem.load(args.problem)
        mode, horizon = args.mode, args.horizon
    options = dict(
        mode=mode,
        backend=args.backend,
        strategy=args.strategy,
        time_limit=args.time_limit,
        max_iterations=args.max_iterations,
        conflict_budget=args.conflict_budget,
    )
    planner = Planner(problem)
    print(f"Planning {problem.name}: {mode}, {args.backend}", flush=True)
    result = (
        planner.solve(horizon, **options)
        if horizon is not None
        else planner.search(args.min_horizon, args.max_horizon, **options)
    )
    _save(problem, result, args.output, visuals=not args.no_visuals)
    summary = {
        "status": result.status,
        "horizon": result.horizon,
        "mode": result.mode,
        "loop_start": result.loop_start,
        "valid": result.validation.get("valid", False),
        "elapsed": result.stats.get("search_elapsed", result.stats.get("elapsed")),
        "output": str(args.output.resolve()),
    }
    print(json.dumps(summary, indent=2, allow_nan=False))
    if result.status != "sat":
        print(result.stats.get("message", "No valid trajectory returned"))
    return {"sat": 0, "unsat": 2, "unknown": 3}.get(result.status, 3)


def _verify(args: argparse.Namespace) -> int:
    problem = Problem.load(args.directory / "problem.json")
    data = json.loads((args.directory / "result.json").read_text(encoding="utf-8"))
    result = PlanResult.from_dict(data)
    report = validate_plan(problem, result, tolerance=args.tolerance)
    _write_json(args.directory / "verification.json", report)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["valid"] else 1


def _benchmark(args: argparse.Namespace) -> int:
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    metadata = {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "package": __version__,
        "dependencies": {name: version(name) for name in ("numpy", "scipy", "z3-solver", "matplotlib")},
        "time_limit_per_run": args.time_limit,
        "note": "Local small-case comparison, not reproduction of the paper's performance tables.",
    }
    for name in args.scenarios:
        factory, horizon, mode = SCENARIOS[name]
        problem = factory()
        for trial in range(1, args.repetitions + 1):
            for backend in ("smc", "smt"):
                print(f"{name} / {backend} / trial {trial}", flush=True)
                start = time.perf_counter()
                result = Planner(problem).solve(
                    horizon, mode=mode, backend=backend, time_limit=args.time_limit
                )
                elapsed = time.perf_counter() - start
                folder = args.output / f"{name}-{backend}-{trial}"
                _save(problem, result, folder, visuals=False)
                row = {
                    "scenario": name,
                    "backend": backend,
                    "trial": trial,
                    "horizon": horizon,
                    "mode": mode,
                    "status": result.status,
                    "valid": result.validation.get("valid", False),
                    "elapsed_seconds": elapsed,
                    "iterations": result.stats.get("iterations", 0),
                    "lp_calls": result.stats.get("lp_calls", 0),
                    "conflicts": result.stats.get("conflicts", 0),
                    "control_effort_l1": result.stats.get("control_effort_l1"),
                    "message": result.stats.get("message", ""),
                }
                rows.append(row)
                _write_json(args.output / "benchmark.json", {"metadata": metadata, "runs": rows})
                with (args.output / "benchmark.csv").open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                    writer.writerows(rows)
                print(f"  {result.status} in {elapsed:.3f}s; validated={row['valid']}", flush=True)
    print(f"Saved benchmark CSV/JSON to {args.output.resolve()}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command in {"demo", "solve"}:
            return _planning(args)
        if args.command == "verify":
            return _verify(args)
        return _benchmark(args)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        print(f"satex: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
