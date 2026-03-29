#include "FlightSimulator.hpp"
#include "ground_surface.hpp"

#include <stdio.h>

int main()
{
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

    GroundSurface ground; // Default ground properties

    FlightSimulator sim(ball, atmos, ground);
    sim.run();

    LandingResult result = sim.getLandingResult();
    printf("Landing spot: %.1f %.1f %.1f yards\n",
           result.xF, result.yF, result.zF);

    return 0;
}
