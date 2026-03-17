#include "FlightSimulator.hpp"
#include "ground_surface.hpp"

#include <stdio.h>

int main()
{
    const LaunchData ball{160.0, 11.0, 0.0, 3000.0, 0.0};
    const AtmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GroundSurface ground; // Default ground properties

    FlightSimulator sim(ball, atmos, ground);
    sim.run();

    LandingResult result = sim.getLandingResult();
    printf("Landing spot: %.1f %.1f %.1f yards\n",
           result.xF, result.yF, result.zF);

    return 0;
}
