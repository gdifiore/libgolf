# Calibration harness

Measures simulated trajectory error against reference shot data. Run locally before/after a physics change to gauge whether it moved the needle.

## Layout

- `sim_runner.cpp` — C++ binary (CMake target `calibration_sim_runner`). Reads shot CSV from stdin, prints per-shot result CSV.
- `run.py` — Python orchestrator. Diffs sim vs truth, classifies, snapshots.
- `history/iteration_NNN.json` — per-iteration snapshot, untracked (gitignored).
- `../../test/data/shots_reference.csv` — source-of-truth dataset (PGA + LPGA Tour 2023 TrackMan averages, small CI-friendly subset).

## Run

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target calibration_sim_runner
python tools/calibration/run.py
```

Flags:
- `--fast` — CI subset (driver, 7-iron, PW)
- `--fail-on-severe` — nonzero exit if any shot classifies severe (CI gate)
- `--no-snapshot` — skip writing iteration file
- `--csv PATH` — custom reference CSV

## Thresholds

Per-metric pass / moderate / severe in yards (carry / total / apex / side). Defined at the top of `run.py`. Tighten only with calibration data to back it up.

## Adding shots

Append rows to `test/data/shots_reference.csv`. Required columns: `shot_id, club, ball_speed_mph, launch_angle_deg, direction_deg, backspin_rpm, sidespin_rpm,temp_f, elevation_ft, wind_mph, wind_dir_deg, humidity_pct, pressure_inhg, carry_yd, total_yd, apex_yd, side_yd, source`. `source` should cite where the truth values came from (TrackMan, FlightScope, paper, etc).
