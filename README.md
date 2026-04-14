# libgolf
`libgolf` is a C++ library that simulates golf ball trajectories from initial conditions: velocity, spin, and atmospheric data. It computes full flight paths including aerial phase, bounce, and roll.

The in-air math here is based on [work done](http://baseball.physics.illinois.edu/trajectory-calculator-golf.html) by Prof. Alan M. Nathan at the  University of Illinois Urbana-Champaign.

## Requirements

- C++20 or later
- CMake 3.14+

## Build
```bash
git clone https://github.com/gdifiore/libgolf.git

cd libgolf

chmod +x build.sh

./build.sh
```

## Features

- Full trajectory simulation with automatic phase transitions (aerial → bounce → roll)
- Dynamic ground surfaces - fairways, roughs, greens, elevation changes
- 3D terrain system with slopes and varying surface normals
- Pluggable aerodynamic model — implement custom Cd/Cl/spin-decay behaviour
- Efficient step-by-step numerical integration

## Documentation

- [Getting Started](/docs/how.md) - Basic usage and examples
- [Terrain System](/docs/terrain.md) - Custom terrain with elevation, slopes, and varying surfaces
- [Aerodynamic Models](/docs/aerodynamic_model.md) - Custom drag, lift, and spin-decay models

### Quick Example

```cpp
#include <libgolf.hpp>

const LaunchData ball{
    .ballSpeedMph = 160.0f,
    .launchAngleDeg = 11.0f,
    .directionDeg = 0.0f,
    .backspinRpm = 3000.0f,
    .sidespinRpm = 0.0f,
};
const AtmosphericData atmos{
    .temp = 70.0f,
    .elevation = 0.0f,
    .vWind = 0.0f,
    .phiWind = 0.0f,
    .hWind = 0.0f,
    .relHumidity = 50.0f,
    .pressure = 29.92f,
};
GroundSurface ground; // Default fairway

FlightSimulator sim(ball, atmos, ground);
sim.run();

LandingResult result = sim.getLandingResult();
printf("Distance: %.1f yards\n", result.distance);
```

For terrain with slopes or position-dependent surfaces, see the [Terrain System](/docs/terrain.md).
