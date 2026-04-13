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

For simulations requiring different ground types throughout the trajectory (e.g., fairway → rough → green), implement the `GroundProvider` interface:

```c++
class MyGroundProvider : public GroundProvider {
public:
    GroundSurface getGroundAt(float x, float y) const override {
        // Return different surfaces based on position
        // x = lateral position (feet), y = downrange position (feet)
    }
};

MyGroundProvider provider;
FlightSimulator sim(ball, atmos, provider);
```

See [Ground Providers Guide](ground_providers.md) for details.

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
float rho = vars.getRhoImperial(); // slugs/ft³
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

When using a `GroundProvider` instead of a single `GroundSurface`, the simulator automatically queries ground properties at the ball's current position during phase transitions. This enables realistic modeling of:

- Golf holes with fairways, roughs, and greens
- Elevated surfaces (e.g., raised greens)
- Varying terrain firmness and friction
- Bunkers and other hazards

Ground is queried before each bounce and periodically during rolling (every 0.1s), so the ball automatically picks up surface changes as it moves. See the [Ground Providers Guide](ground_providers.md) for details.

### Custom Aerodynamic Model

By default, the simulator uses a built-in drag/lift model for a standard golf ball. You can replace it by implementing `AerodynamicModel` and passing it to `FlightSimulator`:

```c++
#include <libgolf.hpp>

class MyModel : public AerodynamicModel {
public:
    double computeCd(double Re_x_e5, double spinFactor) const override {
        // Return drag coefficient as a function of Reynolds number and spin factor
        return 0.35;  // placeholder: constant drag
    }

    double computeCl(double Re_x_e5, double spinFactor) const override {
        // Return lift coefficient as a function of Reynolds number and spin factor
        return 1.5 * spinFactor;  // placeholder: linear lift
    }

    double computeSpinDecayTau(double velocity, double ballRadius) const override {
        // Return spin decay time constant in seconds
        return 1.0 / (0.00002 * velocity / ballRadius);
    }
};

auto model = std::make_shared<MyModel>();
FlightSimulator sim(ball, atmos, ground, model);
```

The two dimensionless inputs — `Re_x_e5` (Reynolds number / 100,000) and `spinFactor` (S = ω·r/v) — are computed from the current ball state each time step; your implementation only needs to express the physics. Omitting the model argument uses `DefaultAerodynamicModel`, which implements the built-in piecewise-linear Cd/Cl model.

See [Aerodynamic Models](aerodynamic_model.md) for full details and worked examples.

### Example Programs

The `examples/` directory contains complete working implementations:

- **calculate_ball_landing.cpp**: Compute final landing position only
- **calculate_ball_trajectory.cpp**: Collect full trajectory for visualization
- **multi_ground_simulation.cpp**: Demonstrate dynamic ground types (fairway/rough/green)
