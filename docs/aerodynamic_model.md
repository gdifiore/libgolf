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
    float    c0;           // Force coefficient (encodes air density and ball geometry)
    float    ballRadius;   // Ball radius (ft)
    float    re100;        // Reynolds number at 100 mph under current atmospheric conditions
};
```

**Reynolds number** at any speed:
```
Re      = (|velocity - windVelocity|_mph / 100) * re100
Re_x_e5 = Re / 1e5
```

**Spin factor** (dimensionless surface-to-translational speed ratio):
```
S = |spinVector| * ballRadius / |velocity - windVelocity|
```

**`c0`** encodes air density, ball mass, and ball cross-section into a single force coefficient. The standard drag force law is `F_drag = -c0 * Cd * vw * v_rel`.

## Interface

```cpp
class AerodynamicModel {
public:
    // Return aerodynamic acceleration (ft/s²). Gravity is NOT included —
    // AerialPhase adds -32.174 ft/s² in z separately.
    virtual Vector3D computeAcceleration(const AerodynamicState& state) const = 0;

    // Return spin decay time constant tau (seconds).
    // AerialPhase applies: spinRate *= exp(-dt / tau)
    // Return 1e6 to effectively disable spin decay.
    virtual double computeSpinDecayTau(const AerodynamicState& state) const = 0;
};
```

Both methods are `const` — do not store mutable state that changes during flight.

## DefaultAerodynamicModel

The built-in model (`include/DefaultAerodynamicModel.hpp`) implements:

**Force law:**
```
F_drag   = -c0 * Cd * vw * (v - v_wind)
F_magnus =  c0 * (Cl / |omega|) * vw * (omega × v_rel)
```

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

All three `FlightSimulator` constructors accept an optional model as the last parameter. Omitting it (or passing `nullptr`) uses `DefaultAerodynamicModel`.

```cpp
GroundSurface ground;
auto model = std::make_shared<MyModel>();

// Flat ground
FlightSimulator sim1(ball, atmos, ground, model);

// Dynamic ground provider
GolfHoleProvider provider;
FlightSimulator sim2(ball, atmos, provider, model);

// Custom terrain
auto terrain = std::make_shared<SlopedTerrain>(5.0f, ground);
FlightSimulator sim3(ball, atmos, ground, terrain, model);
```

The model instance is shared between aerial and bounce phases internally. Models must be thread-safe if simulations run concurrently.

## Implementation Notes

- Both methods are `const` — implement as stateless functions over the provided state
- The library does not clamp or validate return values; unphysical outputs produce unphysical trajectories
- Gravity (`-32.174 ft/s²` in z) is added by `AerialPhase` after `computeAcceleration` returns — do not include it
- `spinVector` is in rad/s and carries the full 3D spin axis. `|spinVector| * ballRadius` gives the surface speed in ft/s (equivalent to `state.spinRate` as stored internally)
- `computeSpinDecayTau` uses velocity before position/velocity integration, consistent with aerodynamic damping physics (faster ball → faster spin loss)

## See Also

- **Interface**: `include/AerodynamicModel.hpp`
- **State struct**: `include/AerodynamicModel.hpp` (`AerodynamicState`)
- **Default implementation**: `include/DefaultAerodynamicModel.hpp`
- **Aerodynamic constants**: `include/physics_constants.hpp`
- **Integrator**: `src/FlightPhase.cpp` (`AerialPhase`)
- **Ground customization**: [Ground Providers](ground_providers.md), [Terrain System](terrain.md)
