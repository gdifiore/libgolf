# Usage Guide

## Requirements

C++20 or later is required. Ensure your CMake toolchain targets C++20:

```cmake
set(CMAKE_CXX_STANDARD 20)
```

## Installation

Build with CMake:

```sh
cmake -B build
cmake --build build
```

Include the main header in your source files:

```c++
#include <libgolf.hpp>
```

## Basic Setup

The library uses a phase-based flight simulation architecture that automatically transitions between aerial, bounce, and roll phases. A complete simulation requires two inputs: launch data and atmospheric conditions.

### 1. Launch Data

```c++
const LaunchData ball{
    .ballSpeedMph = 160.0f,
    .launchAngleDeg = 11.0f,
    .directionDeg = 0.0f,
    .backspinRpm = 3000.0f,
    .sidespinRpm = 0.0f,
};
```

These fields match the output of a typical launch monitor. An optional start position can be provided in feet:

```c++
LaunchData ball{
    .ballSpeedMph = 160.0f,
    .launchAngleDeg = 11.0f,
    .directionDeg = 0.0f,
    .backspinRpm = 3000.0f,
    .sidespinRpm = 0.0f,
};
ball.startX = 0.0f; // feet, lateral
ball.startY = 0.0f; // feet, downrange
ball.startZ = 0.0f; // feet, height above ground
```

### 2. Atmospheric Data

```c++
const AtmosphericData atmos{
    .temp = 70.0f,
    .elevation = 0.0f,
    .vWind = 0.0f,
    .phiWind = 0.0f,
    .hWind = 0.0f,
    .relHumidity = 50.0f,
    .pressure = 29.92f,
};
```

Every field defaults to a sea-level standard day (59°F, 29.92 inHg, no wind,
dry air), so `AtmosphericData{}` is a valid baseline and you only need to set
the fields that differ from standard:

```c++
AtmosphericData atmos{}; // standard day
atmos.elevation = 5280.0f; // mile-high course
```

Field definitions are documented in `include/atmospheric_data.hpp`.

### 3. Ground Surface Properties

#### Single Ground Surface (Simple)

```c++
GroundSurface ground; // Uses default fairway properties

// Or with custom values:
// GroundSurface green{0.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
// {height, restitution, frictionStatic, frictionDynamic, firmness, spinRetention}
```

#### Dynamic Ground Surfaces (Advanced)

For position-dependent surfaces (e.g., fairway → rough → green), implement `TerrainInterface`:

```c++
class MyTerrain : public TerrainInterface {
public:
    float getHeight(float x, float y) const override { return 0.0f; }
    Vector3D getNormal(float x, float y) const override { return {0.0f, 0.0f, 1.0f}; }
    const GroundSurface& getSurfaceProperties(float x, float y) const override {
        // Return surface based on position
    }
};

FlightSimulator sim(ball, atmos, std::make_shared<MyTerrain>());
```

See [Terrain System](terrain.md) for details.

## Running the Simulation

### Run to Completion

```c++
FlightSimulator sim(ball, atmos, ground);
sim.run(); // uses default 10ms time step

LandingResult result = sim.getLandingResult();
printf("Distance: %.1f yards\n", result.distance);
printf("Bearing:  %.1f degrees\n", result.bearing);
```

`LandingResult` contains:
- `xF`, `yF`, `zF` — final position in yards
- `distance` — total distance in yards
- `bearing` — direction in degrees
- `timeOfFlight` — total simulation time in seconds

### Trajectory Collection

To capture the complete flight path for visualization or analysis:

```c++
FlightSimulator sim(ball, atmos, ground);
auto trajectory = sim.runAndGetTrajectory(); // vector of BallState

for (const auto& state : trajectory) {
    printf("%.1f %.1f %.1f\n",
           state.position[0] / physics_constants::YARDS_TO_FEET,
           state.position[1] / physics_constants::YARDS_TO_FEET,
           state.position[2]);
}
```

See `examples/calculate_ball_trajectory.cpp` for a complete implementation.

### Custom Time Step

Both `run()` and `runAndGetTrajectory()` accept an optional time step in seconds (default `0.01f`):

```c++
sim.run(0.005f); // 5ms time step for higher resolution
```

## Querying State

After `run()`, the final ball state is available via `getState()`:

```c++
const BallState& finalState = sim.getState();
// finalState.position — Vector3D in feet
// finalState.velocity — Vector3D in ft/s
// finalState.spinVector — Vector3D in rad/s (|spinVector| * ballRadius = r·ω, ft/s)
// finalState.currentTime — seconds
```

Physics variables computed at launch (air density, Reynolds number, etc.) are accessible via:

```c++
const ShotPhysicsContext& vars = sim.getPhysicsVariables();
float rho = vars.getRhoImperial(); // lb/ft³
```

## Coordinate System

The library uses a right-handed coordinate system:
- **x-axis**: Lateral direction (positive = right of target line)
- **y-axis**: Forward/downrange direction (direction = 0° points along +y)
- **z-axis**: Vertical/height (positive = up)

All position and velocity components are in feet. Use `physics_constants::YARDS_TO_FEET` for unit conversion when needed.

## Flight Phases

The simulator automatically manages three flight phases:

1. **Aerial**: Ball in flight subject to aerodynamic forces
2. **Bounce**: Ball impacting and rebounding from the ground surface
3. **Roll**: Ball rolling along the ground until coming to rest

The current phase can be queried using `sim.getCurrentPhaseName()`, which returns `"aerial"`, `"bounce"`, `"roll"`, or `"complete"`.

## Advanced Features

### Dynamic Ground Surfaces

When passing a `TerrainInterface` instead of a flat `GroundSurface`, the simulator queries terrain properties at the ball's position during phase transitions. This enables fairways, roughs, greens, slopes, and elevated surfaces. See the [Terrain System](terrain.md) for implementation details.

### Custom Aerodynamic Model

By default, the simulator uses a built-in drag/lift model for a standard golf ball. You can replace it by implementing `AerodynamicModel` and passing it to `FlightSimulator`:

```c++
#include <libgolf.hpp>

class MyModel : public AerodynamicModel {
public:
    Vector3D computeAcceleration(const AerodynamicState& s) const override {
        float vRelX = s.velocity[0] - s.windVelocity[0];
        float vRelY = s.velocity[1] - s.windVelocity[1];
        float vRelZ = s.velocity[2] - s.windVelocity[2];
        float vw = std::sqrt(vRelX*vRelX + vRelY*vRelY + vRelZ*vRelZ);

        if (vw < 0.01f) return {0.0f, 0.0f, 0.0f};

        // Constant drag, no lift
        float scale = -s.c0 * 0.30f * vw;
        return { scale * vRelX, scale * vRelY, scale * vRelZ };
    }

    float computeSpinDecayTau(const AerodynamicState& s) const override {
        float v = std::sqrt(s.velocity[0]*s.velocity[0] +
                            s.velocity[1]*s.velocity[1] +
                            s.velocity[2]*s.velocity[2]);
        return 1.0F / (0.00002F * v / s.ballRadius);
    }
};

auto model = std::make_shared<MyModel>();
FlightSimulator sim(ball, atmos, ground, model);
```

See [Aerodynamic Models](aerodynamic_model.md) for full details and worked examples.

### Custom Bounce Model

Replace ball-ground bounce physics by implementing `BounceModel`. Pass it as the fifth argument to `FlightSimulator`; pass `nullptr` for the aero slot to keep the default:

```c++
auto bounce = std::make_shared<MyBounceModel>();
FlightSimulator sim(ball, atmos, ground, /*aero*/ nullptr, bounce);
```

See [Bounce Models](bounce_model.md) for the interface, the default algorithm, and a worked example.

### Custom Roll Model

Replace ball-on-ground roll physics (friction law, integrator, stop criterion) by implementing `RollModel`. Pass it as the sixth argument:

```c++
auto roll = std::make_shared<MyRollModel>();
FlightSimulator sim(ball, atmos, ground, /*aero*/ nullptr, /*bounce*/ nullptr, roll);
```

See [Roll Models](roll_model.md) for the interface and a worked example.

### Custom Ball Properties

The simulator models a standard golf ball by default. Pass a `BallProperties`
to simulate a different ball; its mass and circumference feed the aerodynamic
coefficients and the radius the force models receive:

```c++
BallProperties ball{.massOz = 1.80f, .circumferenceIn = 5.30f};
FlightSimulator sim(launch, atmos, ground,
                    /*aero*/ nullptr, /*bounce*/ nullptr, /*roll*/ nullptr, ball);
```

A default-constructed `BallProperties{}` reproduces the standard ball exactly,
so omitting the argument leaves results unchanged.

### Custom Gravity

Gravity defaults to Earth (`32.174 ft/s²`) and can be set through the
`FlightSimulator` constructor. It is applied to the aerial and between-bounce
flight integration:

```c++
constexpr float kMoonGravity = 5.31f; // ft/s²
FlightSimulator sim(launch, atmos, ground,
                    /*aero*/ nullptr, /*bounce*/ nullptr, /*roll*/ nullptr,
                    /*ball*/ BallProperties{}, kMoonGravity);
```

The built-in roll model decelerates under Earth gravity; a custom `RollModel`
can use any value.

### Custom Integrator

The aerial and between-bounce flight integration uses a semi-implicit Euler
scheme by default. Implement `Integrator` to substitute your own (e.g. RK4 or
an adaptive step) and pass it to `FlightSimulator`:

```c++
auto integrator = std::make_shared<MyRK4Integrator>();
FlightSimulator sim(launch, atmos, ground,
                    /*aero*/ nullptr, /*bounce*/ nullptr, /*roll*/ nullptr,
                    /*ball*/ BallProperties{},
                    physics_constants::GRAVITY_FT_PER_S2, integrator);
```

The flight phase owns spin decay, wind, and the acceleration model; the
integrator owns only how position and velocity advance. It receives an
acceleration field it can sample at trial states. The roll phase runs its own
integrator inside `RollModel`.

### What Isn't Pluggable

You can replace the three per-phase physics models (aerodynamics, bounce, roll) and the terrain. Everything else is fixed in the current release:

- **Air model** — the air-density, viscosity, and saturation-vapor-pressure formulas are fixed. You supply `AtmosphericData` inputs; you cannot swap the model that converts them into density.
- **Phase machine** — the aerial → bounce → roll transition logic and the criteria for when each transition fires are internal. You can replace what each phase *computes* and how the flight phases step (see Custom Integrator), but not how the phases are sequenced.
- **Launch transform** — the mapping from `LaunchData` (launch-monitor inputs) to the initial state vector is fixed.

### Example Programs

The `examples/` directory contains complete working implementations:

- **calculate_ball_landing.cpp**: Compute final landing position only
- **calculate_ball_trajectory.cpp**: Collect full trajectory for visualization
- **multi_ground_simulation.cpp**: Demonstrate dynamic ground types (fairway/rough/green)
- **custom_aerodynamic_model.cpp**: Inject a custom `AerodynamicModel`
- **custom_bounce_model.cpp**: Inject a custom `BounceModel`
- **custom_roll_model.cpp**: Inject a custom `RollModel`
