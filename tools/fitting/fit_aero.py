#!/usr/bin/env python3
"""Fit the three interpretable CalibratedAerodynamicModel scales."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


INPUT_COLUMNS = [
    "shot_id", "ball_speed_mph", "launch_angle_deg", "direction_deg",
    "backspin_rpm", "sidespin_rpm", "temp_f", "elevation_ft", "wind_mph",
    "wind_dir_deg", "humidity_pct", "pressure_inhg",
]


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def run_model(runner: Path, rows: list[dict[str, str]], scales: np.ndarray) -> dict[str, dict[str, float]]:
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=INPUT_COLUMNS)
    writer.writeheader()
    writer.writerows({field: row[field] for field in INPUT_COLUMNS} for row in rows)
    result = subprocess.run(
        [str(runner), "--aero-scales", *(f"{value:.8g}" for value in scales)],
        input=buf.getvalue(), text=True, capture_output=True, check=False,
    )
    if result.returncode:
        raise RuntimeError(f"runner failed: {result.stderr}")
    return {row["shot_id"]: {key: float(value) for key, value in row.items() if key != "shot_id"}
            for row in csv.DictReader(io.StringIO(result.stdout))}


def residuals(runner: Path, rows: list[dict[str, str]], scales: np.ndarray) -> np.ndarray:
    simulated = run_model(runner, rows, scales)
    out: list[float] = []
    for row in rows:
        prediction = simulated[row["shot_id"]]
        out.extend([
            (prediction["carry_yd"] - float(row["carry_yd"])) / 5.0,
            (prediction["apex_yd"] - float(row["apex_yd"])) / 2.5,
            (prediction["side_yd"] - float(row["side_yd"])) / 5.0,
        ])
    return np.asarray(out)


def metrics(runner: Path, rows: list[dict[str, str]], scales: np.ndarray) -> dict[str, float]:
    simulated = run_model(runner, rows, scales)
    errors = {"carry": [], "apex": [], "side": []}
    for row in rows:
        prediction = simulated[row["shot_id"]]
        errors["carry"].append(prediction["carry_yd"] - float(row["carry_yd"]))
        errors["apex"].append(prediction["apex_yd"] - float(row["apex_yd"]))
        errors["side"].append(prediction["side_yd"] - float(row["side_yd"]))
    return {f"{name}_{stat}_yd": float(value)
            for name, values in errors.items()
            for stat, value in (("mae", np.mean(np.abs(values))), ("bias", np.mean(values)),
                                ("rmse", np.sqrt(np.mean(np.square(values))))) }


def split_rows(rows: list[dict[str, str]]) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    """Stable, exact 90/10 split that does not depend on CSV ordering."""
    ordered = sorted(rows, key=lambda row: hashlib.sha256(row["shot_id"].encode()).digest())
    validation_count = max(1, round(len(ordered) * 0.10))
    return ordered[validation_count:], ordered[:validation_count]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, default=Path(__file__).parent / "data" / "prepared" / "garmin_calibration.csv")
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "data" / "fitted_aero.json")
    parser.add_argument("--max-nfev", type=int, default=60)
    args = parser.parse_args()
    rows = read_rows(args.csv)
    train, validation = split_rows(rows)
    if not train or not validation:
        raise SystemExit("split produced an empty train or validation partition")

    baseline = np.asarray([1.0, 1.0, 1.0])
    result = least_squares(
        lambda p: residuals(args.runner, train, p), baseline,
        bounds=(np.asarray([0.75, 0.75, 0.5]), np.asarray([1.25, 1.25, 2.0])),
        diff_step=0.01, loss="soft_l1", f_scale=1.0, max_nfev=args.max_nfev, verbose=1,
    )
    payload = {
        "model": "CalibratedAerodynamicModel",
        "parameters": {"dragScale": float(result.x[0]), "liftScale": float(result.x[1]),
                       "spinDecayScale": float(result.x[2])},
        "rows": {"all": len(rows), "train": len(train), "validation": len(validation)},
        "baseline_metrics": {"train": metrics(args.runner, train, baseline),
                             "validation": metrics(args.runner, validation, baseline)},
        "fitted_metrics": {"train": metrics(args.runner, train, result.x),
                           "validation": metrics(args.runner, validation, result.x)},
        "optimizer": {"status": int(result.status), "message": result.message,
                      "nfev": int(result.nfev), "cost": float(result.cost)},
        "fit_scope": "Garmin R50 aerial carry, apex, and carry-side only; no total-distance fitting.",
        "split": "Deterministic exact 90% training / 10% validation, ordered by SHA-256 shot ID.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
