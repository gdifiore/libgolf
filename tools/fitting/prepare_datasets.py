#!/usr/bin/env python3
"""Normalise the Garmin R50 CSV into a libgolf fitting table."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path


INHG_PER_KPA = 0.2952998751
FPS_PER_MPH = 1.4666666667
GRAVITY_FTPS2 = 32.174
MIN_AERIAL_APEX_FT = 0.5

FIT_COLUMNS = [
    "shot_id", "ball_speed_mph", "launch_angle_deg", "direction_deg",
    "backspin_rpm", "sidespin_rpm", "temp_f", "elevation_ft", "wind_mph",
    "wind_dir_deg", "humidity_pct", "pressure_inhg", "carry_yd", "apex_yd",
    "side_yd",
]


def number(row: dict[str, str], column: str) -> float | None:
    value = row.get(column, "").strip()
    if not value or value == "#NAME?":
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def prepare_garmin(source: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    rows: list[dict[str, object]] = []
    skipped_missing = 0
    skipped_non_aerial = 0
    source_rows = 0
    with source.open(newline="") as f:
        for source_rows, source_row in enumerate(csv.DictReader(f), start=1):
            values = {
                "ball_speed_mph": number(source_row, "Ball Speed (mph)"),
                "launch_angle_deg": number(source_row, "Launch Angle (deg)"),
                "direction_deg": number(source_row, "Launch Direction (deg)"),
                "backspin_rpm": number(source_row, "Backspin (rpm)"),
                "sidespin_rpm": number(source_row, "Sidespin (rpm)"),
                "temp_f": number(source_row, "Temperature (F)"),
                "pressure_kpa": number(source_row, "Air Pressure (kPA)"),
                "carry_yd": number(source_row, "Carry Distance (yards)"),
                "apex_ft": number(source_row, "Apex Height (ft)"),
                "side_yd": number(source_row, "Carry Deviation Distance (yards)"),
            }
            if any(value is None for value in values.values()):
                skipped_missing += 1
                continue
            vertical_speed_fps = (
                values["ball_speed_mph"] * FPS_PER_MPH
                * math.sin(math.radians(values["launch_angle_deg"]))
            )
            vacuum_apex_ft = vertical_speed_fps ** 2 / (2.0 * GRAVITY_FTPS2)
            if vacuum_apex_ft < MIN_AERIAL_APEX_FT:
                skipped_non_aerial += 1
                continue
            # Garmin supplies pressure but not elevation/humidity/wind.
            # Zero elevation prevents a second altitude adjustment.
            rows.append({
                "shot_id": f"garmin_{source_rows:04d}",
                "ball_speed_mph": values["ball_speed_mph"],
                "launch_angle_deg": values["launch_angle_deg"],
                "direction_deg": values["direction_deg"],
                "backspin_rpm": values["backspin_rpm"],
                "sidespin_rpm": values["sidespin_rpm"],
                "temp_f": values["temp_f"],
                "elevation_ft": 0.0,
                "wind_mph": 0.0,
                "wind_dir_deg": 0.0,
                "humidity_pct": 0.0,
                "pressure_inhg": values["pressure_kpa"] * INHG_PER_KPA,
                "carry_yd": values["carry_yd"],
                "apex_yd": values["apex_ft"] / 3.0,
                "side_yd": values["side_yd"],
            })
    return rows, {
        "source_rows": source_rows,
        "usable_rows": len(rows),
        "skipped_missing_or_invalid_rows": skipped_missing,
        "skipped_non_aerial_rows": skipped_non_aerial,
        "minimum_vacuum_apex_ft": MIN_AERIAL_APEX_FT,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", type=Path, default=Path(__file__).parent / "data" / "raw")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "data" / "prepared")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows, summary = prepare_garmin(args.raw_dir / "garmin_r50.csv")
    with (args.output_dir / "garmin_calibration.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    payload = {"garmin": summary, "limitations": [
        "Garmin rows have no club, ball, wind, humidity, surface, or trajectory-time-series identifiers.",
        "Ground-skimming shots below a 0.5 ft vacuum ballistic apex are excluded because this is an aerial flight model.",
    ]}
    (args.output_dir / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
