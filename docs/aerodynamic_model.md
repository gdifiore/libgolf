# Aerodynamic Models

## Overview

The `AerodynamicModel` interface controls aerodynamic force computation and spin decay during the aerial flight phase. Rather than exposing drag and lift coefficients for a fixed force law, the model computes the **full aerodynamic acceleration vector** directly. This means you can implement any force law — change the drag formula, add seam forces, use a full aerodynamic tensor — without touching the integrator.

`AerialPhase` is a pure integrator: it handles timestep advancement, spin decay, and position/velocity updates. Force computation is entirely the model's responsibility.

## AerodynamicState

Both interface methods receive an `AerodynamicState` struct populated by `AerialPhase` each timestep:

```cpp
struct AerodynamicState {
    Vector3D velocity;     // Ball velocity (ft/s)
    Vector3D windVelocity; // Wind at ball height (ft/s; zero below hWind threshold)
    Vector3D spinVector;   // Current spin vector (rad/s); direction = launch axis, magnitude decays
    float    c0;           // Lumped force coefficient (air density × cross-section / mass)
    float    ballRadius;   // Ball radius (ft)
    float    re100;        // Lumped Reynolds reference: Re at 100 mph under current atmospherics
    Vector3D position;     // Ball position (ft; x=lateral, y=forward, z=height)
    float    currentTime;  // Simulation time since launch (s)
};
```

The fields fall into three groups:

**Kinematic state** — `velocity`, `windVelocity`, `spinVector`, `position`, and `currentTime` are snapshots of the ball and its surrounding wind at the current timestep. The default model uses only the first three; `position` and `currentTime` are there so models with altitude-, location-, or time-dependent behaviour can reach for them.

**Ball geometry** — `ballRadius` is populated from `physics_constants::STD_BALL_RADIUS_FT`. The library does not currently support non-standard ball sizes; changing the constant is the only hook.

**Lumped atmosphere** — `c0` and `re100` are precomputed by `ShotPhysicsContext` from temperature, pressure, and humidity. They're a compact encoding for lumped-parameter force laws, but they are lossy — raw air density, viscosity, and gas composition cannot be recovered from them. If your model needs the raw quantities, see [Limitations & Extension](#limitations--extension) below.

One derived quantity shows up in most coefficient correlations: the **spin factor** `S`, a dimensionless ratio of the ball's surface speed to its wind-relative speed.

```
S = |spinVector| * ballRadius / |velocity - windVelocity|
```

## Interface

```cpp
class AerodynamicModel {
public:
    // Return aerodynamic acceleration (ft/s²). Gravity is NOT included —
    // AerialPhase adds -32.174 ft/s² in z separately.
    virtual Vector3D computeAcceleration(const AerodynamicState& state) const = 0;

    // Return spin decay time constant tau (seconds).
    // AerialPhase applies: spinVector *= exp(-dt / tau) (per component, so the
    // spin axis is preserved and magnitude decays exponentially).
    // Return a very large value (e.g. 1e6) to effectively disable spin decay.
    virtual double computeSpinDecayTau(const AerodynamicState& state) const = 0;
};
```

Both methods are `const` — do not store mutable state that changes during flight.

## DefaultAerodynamicModel

The built-in model (`include/DefaultAerodynamicModel.hpp`) is a Reynolds/spin-factor parameterisation. It reads the lumped atmospheric fields, ignores `position` and `currentTime`, and reconstructs Reynolds number as:

```
Re      = (|velocity - windVelocity|_mph / 100) * re100
Re_x_e5 = Re / 1e5
```

The force law (with `vw = |v - v_wind|` and `v_rel = v - v_wind`) is:

```
F_drag   = -c0 * Cd * vw * v_rel
F_magnus =  c0 * (Cl / |omega|) * vw * (omega × v_rel)
```

`c0` folds air density, ball cross-section, and ball mass into a single constant, so these expressions yield acceleration in ft/s² directly — no separate divide by mass is needed.

**Drag** (piecewise-linear through the drag crisis):
```
Re <= RE_THRESHOLD_LOW (0.5):                  Cd = CD_LOW  (0.500)
RE_THRESHOLD_LOW < Re < RE_THRESHOLD_HIGH (1.0): linear + CD_SPIN * S
Re >= RE_THRESHOLD_HIGH (1.0):                 Cd = CD_HIGH (0.200) + CD_SPIN * S
```

**Lift** (quadratic with constant cap):
```
S <= SPIN_FACTOR_THRESHOLD (0.3): Cl = 1.990*S - 3.250*S²
S >  SPIN_FACTOR_THRESHOLD (0.3): Cl = CL_DEFAULT (0.305)
```

**Spin decay:**
```
tau = 1 / (TAU_COEFF * |v| / r)
```

Reference: Washington State University study by Bin Lyu, et al.

`computeCd(Re_x_e5, spinFactor)` and `computeCl(Re_x_e5, spinFactor)` are also available as public non-virtual helpers for inspection or reuse in derived models.

## Basic Example

```cpp
#include <libgolf.hpp>

class ConstantDragModel : public AerodynamicModel {
public:
    Vector3D computeAcceleration(const AerodynamicState& s) const override {
        // Wind-relative velocity
        float vRelX = s.velocity[0] - s.windVelocity[0];
        float vRelY = s.velocity[1] - s.windVelocity[1];
        float vRelZ = s.velocity[2] - s.windVelocity[2];
        float vw = std::sqrt(vRelX*vRelX + vRelY*vRelY + vRelZ*vRelZ);

        if (vw < 0.01f) return {0.0f, 0.0f, 0.0f};

        // Constant drag, no lift
        float scale = -s.c0 * 0.30f * vw;
        return { scale * vRelX, scale * vRelY, scale * vRelZ };
    }

    double computeSpinDecayTau(const AerodynamicState& s) const override {
        double v = std::sqrt(s.velocity[0]*s.velocity[0] +
                             s.velocity[1]*s.velocity[1] +
                             s.velocity[2]*s.velocity[2]);
        return 1.0 / (0.00002 * v / s.ballRadius);
    }
};

auto model = std::make_shared<ConstantDragModel>();
FlightSimulator sim(ball, atmos, ground, model);
```

## Lookup Table Example

For models fitted from wind-tunnel data:

```cpp
#include <libgolf.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

class TableModel : public AerodynamicModel {
public:
    struct Row { double Re_x_e5; double Cd; double Cl_slope; };

    explicit TableModel(std::vector<Row> table) : table_(std::move(table)) {}

    Vector3D computeAcceleration(const AerodynamicState& s) const override {
        float vRelX = s.velocity[0] - s.windVelocity[0];
        float vRelY = s.velocity[1] - s.windVelocity[1];
        float vRelZ = s.velocity[2] - s.windVelocity[2];
        double vw = std::sqrt(vRelX*vRelX + vRelY*vRelY + vRelZ*vRelZ);

        if (vw < 0.01) return {0.0f, 0.0f, 0.0f};

        double vwMph = vw / physics_constants::MPH_TO_FT_PER_S;
        double Re_x_e5 = (vwMph / 100.0) * s.re100 * 1e-5;

        double omegaMag = std::sqrt(s.spinVector[0]*s.spinVector[0] +
                                    s.spinVector[1]*s.spinVector[1] +
                                    s.spinVector[2]*s.spinVector[2]);
        double S = omegaMag * s.ballRadius / vw;

        double Cd = interpolate(Re_x_e5, &Row::Cd);
        double Cl = interpolate(Re_x_e5, &Row::Cl_slope) * S;

        double dragScale = -s.c0 * Cd * vw;
        float dragX = (float)(dragScale * vRelX);
        float dragY = (float)(dragScale * vRelY);
        float dragZ = (float)(dragScale * vRelZ);

        float magnusX = 0, magnusY = 0, magnusZ = 0;
        if (omegaMag > 0.01) {
            double ms = s.c0 * (Cl / omegaMag) * vw;
            magnusX = (float)(ms * (s.spinVector[1]*vRelZ - s.spinVector[2]*vRelY));
            magnusY = (float)(ms * (s.spinVector[2]*vRelX - s.spinVector[0]*vRelZ));
            magnusZ = (float)(ms * (s.spinVector[0]*vRelY - s.spinVector[1]*vRelX));
        }

        return { dragX+magnusX, dragY+magnusY, dragZ+magnusZ };
    }

    double computeSpinDecayTau(const AerodynamicState& s) const override {
        double v = std::sqrt(s.velocity[0]*s.velocity[0] +
                             s.velocity[1]*s.velocity[1] +
                             s.velocity[2]*s.velocity[2]);
        return 1.0 / (0.00002 * v / s.ballRadius);
    }

private:
    std::vector<Row> table_;

    double interpolate(double Re_x_e5, double Row::* field) const {
        if (Re_x_e5 <= table_.front().Re_x_e5) return table_.front().*field;
        if (Re_x_e5 >= table_.back().Re_x_e5)  return table_.back().*field;
        auto it = std::lower_bound(table_.begin(), table_.end(), Re_x_e5,
            [](const Row& r, double v){ return r.Re_x_e5 < v; });
        const Row& hi = *it;
        const Row& lo = *(it - 1);
        double t = (Re_x_e5 - lo.Re_x_e5) / (hi.Re_x_e5 - lo.Re_x_e5);
        return lo.*field + t * (hi.*field - lo.*field);
    }
};
```

## Using with FlightSimulator

Both `FlightSimulator` constructors accept an optional model as the last parameter. Omitting it (or passing `nullptr`) uses `DefaultAerodynamicModel`.

```cpp
GroundSurface ground;
auto terrain = std::make_shared<MyTerrain>();
auto model   = std::make_shared<MyModel>();

// Flat ground
FlightSimulator sim1(launch, atmos, ground, model);

// Custom terrain
FlightSimulator sim2(launch, atmos, terrain, model);
```

The model instance is shared between aerial and bounce phases internally. Models must be thread-safe if simulations run concurrently.

## Implementation Notes

- Both methods are `const` — implement as stateless functions over the provided state
- The library does not clamp or validate return values; unphysical outputs produce unphysical trajectories
- Gravity (`-32.174 ft/s²` in z) is added by `AerialPhase` after `computeAcceleration` returns — do not include it
- `spinVector` is in rad/s and carries the full 3D spin axis. Its magnitude times `ballRadius` gives the ball's surface speed in ft/s — the quantity classical references write as `r·ω`
- `computeSpinDecayTau` uses velocity before position/velocity integration, consistent with aerodynamic damping physics (faster ball → faster spin loss)

## Limitations & Extension

`AerodynamicState` captures what the simulator natively tracks, which is deliberately narrower than "everything an aerodynamic model could ever want." Some model families need information that the library does not carry, and adding it means changing the simulator itself rather than just subclassing `AerodynamicModel`. The common cases:

- **Raw atmospheric quantities** — air density ρ, dynamic viscosity μ, temperature, humidity, pressure. `c0` and `re100` are lumped into `ShotPhysicsContext`; they bundle these quantities and cannot be inverted back to the raw values. Exposing the raw values means plumbing them through `ShotPhysicsContext` alongside the lumped ones.
- **Ball orientation or attitude** — the simulator treats the ball as axisymmetric, so `BallState` has no quaternion or rotation matrix. Dimple-pattern-aware models, non-axisymmetric wake models, and full rigid-body rotational dynamics all require extending `BallState` and the integrator.
- **Rigid-body inertia** — if your model wants to return a torque and evolve a full angular-momentum vector, it needs the ball's moment of inertia and an integrator that consumes it. The current `spinVector *= exp(-dt/τ)` assumes isotropic scalar decay and exposes no torque pathway.
- **Per-step history or transient effects** — the state passed to the model is memoryless. Models with hysteresis, added-mass, or vortex shedding need to carry their own state. Because the interface methods are `const`, that state has to live in a `mutable` member or be keyed externally by `currentTime`.
- **Spatial or non-uniform wind fields** — the library's wind is a single horizontal vector applied above altitude `hWind` and zero below. A terrain-aware or fully 3D wind field would ideally be a proper atmospheric interface. A custom model can also synthesise one inside `computeAcceleration`: `position` is available on the state, so the model can override whatever wind vector it was handed.

The library intentionally exposes the minimum state its reference model needs, rather than a speculative superset. If you hit one of these limits, please open an issue describing the use case — broadening the interface in-tree is better for everyone than maintaining a fork.

## See Also

- **Interface**: `include/AerodynamicModel.hpp`
- **State struct**: `include/AerodynamicModel.hpp` (`AerodynamicState`)
- **Default implementation**: `include/DefaultAerodynamicModel.hpp`
- **Aerodynamic constants**: `include/physics_constants.hpp`
- **Integrator**: `src/FlightPhase.cpp` (`AerialPhase`)
- **Ground customization**: [Terrain System](terrain.md)
