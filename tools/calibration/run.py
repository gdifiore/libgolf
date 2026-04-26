#!/usr/bin/env python3
"""Calibration harness for libgolf.

Pipes a reference shot CSV into the C++ `calibration_sim_runner` binary,
diffs simulated vs source-of-truth metrics, classifies each shot, and writes
an iteration snapshot to tools/calibration/history/iteration_NNN.json.

Usage:
    python tools/calibration/run.py
    python tools/calibration/run.py --csv test/data/shots_reference.csv
    python tools/calibration/run.py --fast       # CI subset
    python tools/calibration/run.py --no-snapshot
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CSV = REPO_ROOT / "test" / "data" / "shots_reference.csv"
HISTORY_DIR = REPO_ROOT / "tools" / "calibration" / "history"

# Pass / moderate / severe in yards (carry, total, apex) and yards (side).
THRESHOLDS = {
    "carry":  {"pass": 5.0, "moderate": 12.0},
    "total":  {"pass": 6.0, "moderate": 14.0},
    "apex":   {"pass": 3.0, "moderate": 7.0},
    "side":   {"pass": 3.0, "moderate": 7.0},
}

# CI fast subset: one shot per regime.
FAST_IDS = {"pga_driver", "pga_7iron", "pga_pw", "lpga_pw"}

SIM_INPUT_COLS = [
    "shot_id", "ball_speed_mph", "launch_angle_deg", "direction_deg",
    "backspin_rpm", "sidespin_rpm", "temp_f", "elevation_ft", "wind_mph",
    "wind_dir_deg", "humidity_pct", "pressure_inhg",
]


@dataclass
class ShotDiff:
    shot_id: str
    club: str
    metric_errors: dict       # {metric: (sim, truth, error)}
    classifications: dict     # {metric: pass|moderate|severe}
    overall: str              # worst classification across metrics


def classify(error_yd: float, metric: str) -> str:
    t = THRESHOLDS[metric]
    a = abs(error_yd)
    if a <= t["pass"]:
        return "pass"
    if a <= t["moderate"]:
        return "moderate"
    return "severe"


def find_runner() -> Path:
    candidates = [
        REPO_ROOT / "build" / "calibration_sim_runner",
        REPO_ROOT / "build" / "Release" / "calibration_sim_runner.exe",
        REPO_ROOT / "build" / "Debug" / "calibration_sim_runner.exe",
    ]
    for c in candidates:
        if c.exists():
            return c
    sys.exit(
        "calibration_sim_runner not built. Run:\n"
        "  cmake -B build -DCMAKE_BUILD_TYPE=Release\n"
        "  cmake --build build --target calibration_sim_runner"
    )


def load_truth(csv_path: Path, fast: bool) -> list[dict]:
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    if fast:
        rows = [r for r in rows if r["shot_id"] in FAST_IDS]
    return rows


def build_sim_input(rows: list[dict]) -> str:
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(SIM_INPUT_COLS)
    for r in rows:
        w.writerow([r.get(c, "") for c in SIM_INPUT_COLS])
    return buf.getvalue()


def run_sim(runner: Path, sim_input: str) -> list[dict]:
    proc = subprocess.run(
        [str(runner)],
        input=sim_input,
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        sys.exit(f"sim_runner failed (exit {proc.returncode}):\n{proc.stderr}")
    reader = csv.DictReader(io.StringIO(proc.stdout))
    return list(reader)


def diff_shots(truth_rows: list[dict], sim_rows: list[dict]) -> list[ShotDiff]:
    sim_by_id = {r["shot_id"]: r for r in sim_rows}
    diffs = []
    for t in truth_rows:
        sid = t["shot_id"]
        s = sim_by_id.get(sid)
        if s is None:
            continue
        metrics = {}
        cls = {}
        for m, sim_key, truth_key in [
            ("carry", "carry_yd", "carry_yd"),
            ("total", "total_yd", "total_yd"),
            ("apex",  "apex_yd",  "apex_yd"),
            ("side",  "side_yd",  "side_yd"),
        ]:
            sim_v = float(s[sim_key])
            truth_v = float(t[truth_key])
            err = sim_v - truth_v
            metrics[m] = {"sim": sim_v, "truth": truth_v, "error": err}
            cls[m] = classify(err, m)
        rank = {"pass": 0, "moderate": 1, "severe": 2}
        overall = max(cls.values(), key=lambda c: rank[c])
        diffs.append(ShotDiff(
            shot_id=sid,
            club=t.get("club", ""),
            metric_errors=metrics,
            classifications=cls,
            overall=overall,
        ))
    return diffs


def print_report(diffs: list[ShotDiff]) -> None:
    print(f"{'shot':<14}{'club':<10}{'carry Δ':>10}{'total Δ':>10}{'apex Δ':>10}{'side Δ':>10}  overall")
    print("-" * 78)
    for d in diffs:
        e = d.metric_errors
        print(
            f"{d.shot_id:<14}{d.club:<10}"
            f"{e['carry']['error']:>+10.1f}{e['total']['error']:>+10.1f}"
            f"{e['apex']['error']:>+10.1f}{e['side']['error']:>+10.1f}"
            f"  {d.overall}"
        )
    counts = {"pass": 0, "moderate": 0, "severe": 0}
    for d in diffs:
        counts[d.overall] += 1
    print("-" * 78)
    print(f"summary: {counts['pass']} pass / {counts['moderate']} moderate / {counts['severe']} severe  (n={len(diffs)})")


def next_iter_path() -> Path:
    HISTORY_DIR.mkdir(parents=True, exist_ok=True)
    existing = sorted(HISTORY_DIR.glob("iteration_*.json"))
    n = 1
    if existing:
        last = existing[-1].stem.split("_")[-1]
        n = int(last) + 1
    return HISTORY_DIR / f"iteration_{n:03d}.json"


def write_snapshot(path: Path, diffs: list[ShotDiff], csv_path: Path) -> None:
    payload = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "reference_csv": str(csv_path.relative_to(REPO_ROOT)),
        "thresholds": THRESHOLDS,
        "shots": [asdict(d) for d in diffs],
    }
    path.write_text(json.dumps(payload, indent=2))
    print(f"snapshot: {path.relative_to(REPO_ROOT)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    ap.add_argument("--fast", action="store_true", help="CI subset")
    ap.add_argument("--no-snapshot", action="store_true")
    ap.add_argument("--fail-on-severe", action="store_true",
                    help="exit nonzero if any shot classifies severe")
    ap.add_argument("--baseline", type=Path,
                    help="compare against snapshot; fail if any metric regresses past tolerance")
    ap.add_argument("--regression-tolerance", type=float, default=1.0,
                    help="yards a metric error may grow vs baseline before failing (default 1.0)")
    args = ap.parse_args()

    runner = find_runner()
    truth = load_truth(args.csv, args.fast)
    if not truth:
        sys.exit("no truth rows loaded")

    sim_input = build_sim_input(truth)
    sim_rows = run_sim(runner, sim_input)
    diffs = diff_shots(truth, sim_rows)

    print_report(diffs)

    if not args.no_snapshot:
        write_snapshot(next_iter_path(), diffs, args.csv)

    exit_code = 0
    if args.fail_on_severe and any(d.overall == "severe" for d in diffs):
        exit_code = 1

    if args.baseline:
        regressions = check_regressions(diffs, args.baseline, args.regression_tolerance)
        if regressions:
            print("\nREGRESSIONS vs baseline:")
            for r in regressions:
                print(f"  {r}")
            exit_code = 1
        else:
            print(f"\nno regressions vs {args.baseline.name} "
                  f"(tolerance ±{args.regression_tolerance} yd)")

    return exit_code


def check_regressions(diffs: list[ShotDiff], baseline_path: Path, tol: float) -> list[str]:
    baseline = json.loads(baseline_path.read_text())
    base_by_id = {s["shot_id"]: s for s in baseline["shots"]}
    out = []
    for d in diffs:
        b = base_by_id.get(d.shot_id)
        if b is None:
            continue
        for m, current in d.metric_errors.items():
            base_err = abs(b["metric_errors"][m]["error"])
            cur_err = abs(current["error"])
            if cur_err > base_err + tol:
                out.append(
                    f"{d.shot_id} {m}: |err| {base_err:.2f} -> {cur_err:.2f} yd "
                    f"(+{cur_err - base_err:.2f})"
                )
    return out


if __name__ == "__main__":
    sys.exit(main())
