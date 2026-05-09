# Bounce Models

## Overview

The `BounceModel` interface controls ball-ground bounce physics. The library detects ground impact, computes the surface normal, and looks up surface properties (restitution, friction, firmness, spin retention) from the active terrain. The model receives this snapshot and returns the post-bounce velocity and spin.

`BouncePhase` is a thin orchestrator: terrain query, impact detection, model dispatch, then the standard aerodynamic + gravity integration step until the next impact or the bounce-to-roll transition. Bounce force law is entirely the model's responsibility.

This mirrors the `AerodynamicModel` interface (see [aerodynamic_model.md](aerodynamic_model.md)). The library provides phase plumbing and terrain access; you bring the physics.

## BounceState

```cpp
struct BounceState {
    Vector3D velocity;       // Pre-bounce velocity (ft/s)
    Vector3D surfaceNormal;  // Unit normal at impact (away from ground)
    Vector3D spinVector;     // Pre-bounce spin (rad/s)
    float    ballRadius;     // Ball radius (ft)
};
```

Spin is a vector, not a scalar — the model can implement direction-dependent effects (axis tilt, sidespin coupling) without losing information.

## BounceResult

```cpp
struct BounceResult {
    Vector3D newVelocity;    // Post-bounce velocity (ft/s)
    Vector3D newSpinVector;  // Post-bounce spin (rad/s)
};
```

`newSpinVector` may differ in direction as well as magnitude — useful for axis-tilting or wedge-bite models.

## Interface

```cpp
class BounceModel {
public:
    virtual BounceResult resolveBounce(
        const BounceState& state,
        const GroundSurface& surface) const = 0;
};
```

`resolveBounce` is `const` — do not store mutable state that changes during flight.

`GroundSurface` exposes the surface's COR, friction (static/dynamic), firmness, and spin retention. Anything model-specific (critical-angle thresholds, COR curves, regime gates) belongs in the model implementation, not on the surface struct.

## DefaultBounceModel

The built-in implementation lives in `include/DefaultBounceModel.hpp` and is used when no custom model is supplied.

Algorithm:

```
v_n     = (v · n̂) n̂
v_t     = v - v_n
v_n'    = -COR · v_n
v_t'    = friction · v_t                                    (frictionFactor in [0, 1])
spin_t  = (2 · r · |ω|) / 7                                 (Penner spin term, magnitude only)
v_t'   *= max(0, 1 - spin_t / |v_t'|)                       (cannot reverse direction)
ω'      = spinRetention · ω
```

Where:
- `frictionFactor = 1 - frictionStatic · (1 - firmness)` — clamped to `[0, 1]`.
- COR, friction, firmness, and spin retention come from `GroundSurface`.

Reference: Penner, A.R. *The physics of golf* (Reports on Progress in Physics, 2003).

## Injecting a custom model

A minimal subclass with fixed COR and no spin coupling. Builds and runs from the standard CMake build (`build/custom_bounce_model`); source at [`examples/custom_bounce_model.cpp`](../examples/custom_bounce_model.cpp).

{% raw %}
```cpp
class FixedCorBounceModel : public BounceModel
{
public:
    FixedCorBounceModel(float cor, float tangentialRetention)
        : cor_(cor), tang_(tangentialRetention) {}

    BounceResult resolveBounce(const BounceState &s,
                               const GroundSurface & /*surface*/) const override
    {
        const Vector3D vn = math_utils::project(s.velocity, s.surfaceNormal);
        const Vector3D vt{s.velocity[0] - vn[0],
                          s.velocity[1] - vn[1],
                          s.velocity[2] - vn[2]};

        const Vector3D vnPost{-cor_ * vn[0], -cor_ * vn[1], -cor_ * vn[2]};
        const Vector3D vtPost{tang_ * vt[0], tang_ * vt[1], tang_ * vt[2]};

        return {{vnPost[0] + vtPost[0],
                 vnPost[1] + vtPost[1],
                 vnPost[2] + vtPost[2]},
                s.spinVector};
    }

private:
    float cor_;
    float tang_;
};

auto bounce = std::make_shared<FixedCorBounceModel>(0.35F, 0.6F);
FlightSimulator sim(launch, atmos, ground, /*aero*/ nullptr, bounce);
sim.run();
```
{% endraw %}

All three model slots on `FlightSimulator` are independent — pass `nullptr` for any slot to keep the default.

## Limitations

- The model is invoked on each detected impact, but the library decides *when* an impact has occurred (height vs. terrain plus a vertical velocity check). If you need a different impact criterion (e.g. embed-and-stop on soft sand), that lives in `BouncePhase`, not the model.
- `BounceState` does not currently carry impact angle, rolling angle, or contact duration. Compute these from `velocity` and `surfaceNormal` inside the model if needed.
- Roll physics is still a free function in `ground_physics`. A `RollModel` interface analogous to this one is plausible future work.
