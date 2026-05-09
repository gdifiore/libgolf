# Roll Models

## Overview

The `RollModel` interface controls ball-on-ground roll physics. The library detects the bounce-to-roll transition, queries the surface normal and properties at the ball position, and hands the model a per-step state snapshot. The model returns post-step kinematics and an `atRest` flag.

`RollPhase` is a thin orchestrator: terrain query, model dispatch, then terrain-height clamping. Friction law, integrator, and stopping criterion are entirely the model's responsibility.

This mirrors `BounceModel` (see [bounce_model.md](bounce_model.md)) and `AerodynamicModel` (see [aerodynamic_model.md](aerodynamic_model.md)). The library provides phase plumbing and terrain access; you bring the physics.

## Why a step interface, not a force interface

`BounceModel::resolveBounce` returns a single post-impulse velocity — bounces are events. Roll is an integration over many small steps. Handing the model `dt` lets it choose its integrator (semi-implicit Euler, RK4, custom), apply substeps, or carry continuous state through history if it needs to.

The model also owns the stop decision through `RollResult::atRest`. A custom model can stop on energy thresholds, static-friction balance on slopes, or whatever criterion fits its physics.

## RollState

```cpp
struct RollState {
    Vector3D position;       // Pre-step position (ft)
    Vector3D velocity;       // Pre-step velocity (ft/s)
    Vector3D spinVector;     // Pre-step spin (rad/s)
    Vector3D surfaceNormal;  // Unit normal at ball position
    float    ballRadius;     // Ball radius (ft)
    float    dt;             // Time step (s)
};
```

## RollResult

```cpp
struct RollResult {
    Vector3D newPosition;    // Post-step position (ft)
    Vector3D newVelocity;    // Post-step velocity (ft/s)
    Vector3D newSpinVector;  // Post-step spin (rad/s)
    bool     atRest;         // Phase complete when true
};
```

The library overwrites `newPosition[2]` with the terrain height after the call — the model has no terrain handle and cannot evaluate height itself. Set the X/Y components; Z is advisory.

## Interface

```cpp
class RollModel {
public:
    virtual RollResult step(
        const RollState& state,
        const GroundSurface& surface) const = 0;
};
```

`step` is `const` — do not store mutable state that changes during flight.

## DefaultRollModel

The built-in implementation lives in `include/DefaultRollModel.hpp` and is used when no custom model is supplied.

Algorithm:

```
a       = -g·sin(θ) along slope - μ·g·cos(θ) opposing motion         (Coulomb friction)
v'      = v + a·dt
v_xy   := 0 if sign flipped across the step and |v_old_xy| > ε       (prevents reversal)
p'      = p + v'·dt
ω'      = ω · max(0, 1 - SPIN_DECAY_RATE·dt / |ω|)                   (linear, axis preserved)
atRest  = |v'_horizontal| < STOPPING_VELOCITY
```

Where:
- `μ = surface.frictionDynamic`.
- `θ` derived from `surfaceNormal[2]`. Surfaces with `cos(θ) > FLAT_SURFACE_THRESHOLD` skip slope decomposition.
- `STOPPING_VELOCITY = 0.1 ft/s` and `SPIN_DECAY_RATE = 2.0 rad/s²` live as `static constexpr` members on `DefaultRollModel`.

## Injecting a custom model

A minimal subclass with linear deceleration and no slope handling. Builds and runs from the standard CMake build (`build/custom_roll_model`); source at [`examples/custom_roll_model.cpp`](../examples/custom_roll_model.cpp).

```cpp
class LinearDecelRollModel : public RollModel
{
public:
    static constexpr float STOP_VELOCITY = 0.1F; // ft/s
    static constexpr float DECEL = 6.0F;         // ft/s^2

    RollResult step(const RollState &s,
                    const GroundSurface & /*surface*/) const override
    {
        const float speed = math_utils::magnitude(s.velocity);
        if (speed < STOP_VELOCITY)
        {
            return {s.position, {0.0F, 0.0F, 0.0F}, s.spinVector, true};
        }

        const float dv = DECEL * s.dt;
        const float scale = (dv >= speed) ? 0.0F : (speed - dv) / speed;
        const Vector3D vNew{s.velocity[0] * scale,
                            s.velocity[1] * scale,
                            s.velocity[2] * scale};
        const Vector3D pNew{s.position[0] + vNew[0] * s.dt,
                            s.position[1] + vNew[1] * s.dt,
                            s.position[2] + vNew[2] * s.dt};

        const bool atRest = math_utils::magnitude(vNew) < STOP_VELOCITY;
        return {pNew, vNew, s.spinVector, atRest};
    }
};

auto roll = std::make_shared<LinearDecelRollModel>();
FlightSimulator sim(launch, atmos, ground,
                    /*aero*/ nullptr,
                    /*bounce*/ nullptr,
                    roll);
sim.run();
```

All three model slots are independent — pass `nullptr` to use the default for any of them.

## Limitations

- The library decides *when* the roll phase begins (see `GroundPhysics::shouldTransitionToRoll`). The model has no say. If you need a different transition criterion, that lives in `BouncePhase`.
- `RollState` does not currently carry rolling resistance history, slip ratio, or contact patch geometry. Compute these inside the model from successive calls if needed (state must live on the model and be reset between simulations).
- The library overwrites the Z position with terrain height after each step. A roll model cannot model bouncing micro-hops or airborne traversal — those are bounce-phase concerns.
