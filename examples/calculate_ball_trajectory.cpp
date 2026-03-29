#include "FlightSimulator.hpp"
#include "ground_surface.hpp"
#include "physics_constants.hpp"

#include <stdio.h>
#include <vector>

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
    auto trajectory = sim.runAndGetTrajectory();

    printf("Entire ball trajectory:\n");

    for (const auto& state : trajectory)
    {
        printf("%.1f %.1f %.1f\n",
               state.position[0] / physics_constants::YARDS_TO_FEET,
               state.position[1] / physics_constants::YARDS_TO_FEET,
               state.position[2]);
    }

    return 0;
}
