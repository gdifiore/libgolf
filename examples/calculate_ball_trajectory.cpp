#include "FlightSimulator.hpp"
#include "ground_surface.hpp"
#include "physics_constants.hpp"

#include <stdio.h>
#include <vector>

int main()
{
    const LaunchData ball{160.0, 11.0, 0.0, 3000.0, 0.0};
    const AtmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

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
