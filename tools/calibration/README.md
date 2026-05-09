# Validation harness

Measures simulated trajectory error against reference shot data and gates regressions across iterations. **No optimizer** — change model constants by hand (or swap models entirely), rerun, compare. The "calibration" naming is historical; this is a validation + regression-check harness, not a fitter.

## Layout

- `sim_runner.cpp` — C++ binary (CMake target `calibration_sim_runner`). Reads shot CSV from stdin, prints per-shot result CSV.
- `run.py` — Python orchestrator. Diffs sim vs truth, classifies, snapshots, optional baseline regression check.
- `history/iteration_NNN.json` — per-iteration snapshot, untracked (gitignored).
- `../../test/data/shots_reference.csv` — bundled truth set (PGA + LPGA Tour 2023 TrackMan averages, small CI-friendly subset). Replace via `--csv` for your own data.

## Build & run

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target calibration_sim_runner
python tools/calibration/run.py
```

## Tuning your own model

`sim_runner.cpp` constructs `FlightSimulator` with the built-in default `AerodynamicModel` / `BounceModel` / `RollModel`. **The runner is not generic over models** — to validate a custom model, edit the construction site (search for the `// To validate a custom model` comment in `sim_runner.cpp`), inject your `std::shared_ptr<AerodynamicModel>` (and/or bounce/roll), and rebuild the target. The CSV I/O contract is stable, so `run.py` will work against your modified runner unchanged.

If you have a fully separate sim binary that already produces the right output CSV (`shot_id, carry_yd, total_yd, apex_yd, side_yd, bearing_deg, time_s`), you can pipe its output into `run.py`'s diff path by skipping `find_runner()` — but the simplest path is editing `sim_runner.cpp`.

## Flags

- `--csv PATH` — custom reference CSV (default: `test/data/shots_reference.csv`)
- `--fast` — CI subset (driver, 7-iron, PW, LPGA-PW)
- `--no-snapshot` — skip writing `history/iteration_NNN.json`
- `--fail-on-severe` — nonzero exit if any shot classifies severe (CI gate)
- `--baseline PATH` — compare against a previous snapshot; nonzero exit on regression
- `--regression-tolerance YD` — yards a metric error may grow vs baseline before failing (default: 1.0)

### Typical tuning loop

```sh
# lock in current state
python tools/calibration/run.py
cp tools/calibration/history/iteration_001.json baseline.json

# change model constants, rebuild
cmake --build build --target calibration_sim_runner

# rerun, fail loudly if anything got worse
python tools/calibration/run.py --baseline baseline.json --regression-tolerance 1.0
```

## Thresholds

Per-metric pass / moderate / severe in yards (carry, total, apex, side). Defined as the `THRESHOLDS` dict at the top of `run.py`. **Edit to taste** for your own data — the bundled values target Tour-data accuracy and are tight for amateur launch-monitor reference sets. Tighten only if you have data to back the new bound.

## Output columns

`sim_runner.cpp` emits `shot_id, carry_yd, total_yd, apex_yd, side_yd, bearing_deg, time_s`. Notes:

- **`side_yd`** is lateral deflection at carry (first ground touch), not after roll. Aero tuners want carry-time x; post-roll lateral is dominated by bounce + slope and is available separately as `lr.xF` inside the runner.
- **`carry_yd`** is `sqrt(x² + y²)` at the carry index — straight-line distance from origin, not "down the target line." Fine for `direction_deg = 0`; for offline shots, replace with `position[1]` (downrange y) if that matches your truth-data convention.
- **Carry detection** uses hysteresis on z (`AIRBORNE_FT`, `CARRY_GROUND_FT` constants in `sim_runner.cpp`) — tune if your tee heights or model bounces are unusual.

## Adding shots

Append rows to `test/data/shots_reference.csv` (or your own file used via `--csv`). Required columns:

```
shot_id, club, ball_speed_mph, launch_angle_deg, direction_deg,
backspin_rpm, sidespin_rpm, temp_f, elevation_ft, wind_mph,
wind_dir_deg, humidity_pct, pressure_inhg,
carry_yd, total_yd, apex_yd, side_yd, source
```

`source` should cite where the truth values came from (TrackMan, FlightScope, paper, etc.) so future readers can audit.
