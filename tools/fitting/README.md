# Public-data aerodynamic fitting

This workflow calibrates the opt-in `CalibratedAerodynamicModel` against the
public MIT-licensed Garmin R50 dataset. The raw data is not committed; fetch it
locally.

```sh
python3 tools/fitting/fetch_datasets.py
python3 tools/fitting/prepare_datasets.py
cmake -S . -B build -DGOLF_BUILD_EXAMPLES=ON
cmake --build build --target fitting_sim_runner
python3 tools/fitting/fit_aero.py --runner build/fitting_sim_runner
python3 tools/fitting/check_fit.py tools/fitting/data/fitted_aero.json
```

`fit_aero.py` fits drag, lift, and aerial spin-decay scales to Garmin carry,
apex, and carry-side results. It excludes total distance because the dataset
does not identify turf, firmness, ball model, or roll conditions.

The fitter uses a deterministic exact 90%/10% Garmin train/validation split
keyed by shot ID. `check_fit.py` enforces the held-out carry target of at most
5 yards mean absolute error.

## Data limits

- Garmin R50 has no club or ball identifier, wind, humidity, surface, or
  per-point trajectory data. It is a calm-condition, mixed-population fit.
- The preparation step excludes ground-skimming rows whose no-drag ballistic
  apex is below 0.5 ft. They fall outside the simulator's aerial carry
  measurement; this is a model-scope filter, not a source-data correction.

Sources and licence declarations are recorded in the raw-data manifest:

- https://www.kaggle.com/datasets/jamieb122/golf-swing-and-trajectory-data
