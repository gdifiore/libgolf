# libgolf
`libgolf` is a C++ library designed to simulate golf ball trajectories based on initial conditions like velocity, atmospheric data, and more. It provides easy-to-use functions to visualize or calculate the ball's flight path and landing point (now including bounces, rolling, and dynamic ground surfaces!).

The in-air math here is based on [work done](http://baseball.physics.illinois.edu/trajectory-calculator-golf.html) by Prof. Alan M. Nathan at the  University of Illinois Urbana-Champaign.

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
- Efficient step-by-step numerical integration

## Documentation

- [Getting Started](/docs/how.md) - Basic usage and examples
- [Ground Providers](/docs/ground_providers.md) - Dynamic ground surfaces (fairways, roughs, greens)
- [Terrain System](/docs/terrain.md) - 3D terrain with elevation and slopes

### Quick Example

```cpp
#include <libgolf.hpp>

const LaunchData ball{160.0f, 11.0f, 0.0f, 3000.0f, 0.0f}; // mph, °, °, rpm, rpm
const AtmosphericData atmos{70.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 29.92f};
GroundSurface ground; // Default fairway

FlightSimulator sim(ball, atmos, ground);
sim.run();

LandingResult result = sim.getLandingResult();
printf("Distance: %.1f yards\n", result.distance);
```

For dynamic ground surfaces (fairway/rough/green), see the [Ground Providers Guide](/docs/ground_providers.md).
