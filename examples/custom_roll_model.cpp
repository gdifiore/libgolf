// Minimal custom RollModel: linear deceleration, no slope handling.
// Apply constant deceleration along velocity direction, integrate
// semi-implicit Euler, stop below a fixed velocity threshold.

#include "FlightSimulator.hpp"
#include "RollModel.hpp"
#include "ground_surface.hpp"
#include "math_utils.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

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

int main()
{
    const LaunchData launch{160.0F, 11.0F, 0.0F, 3000.0F, 0.0F};
    const AtmosphericData atmos{70.0F, 0.0F, 0.0F, 0.0F, 0.0F, 50.0F, 29.92F};
    GroundSurface ground;

    auto roll = std::make_shared<LinearDecelRollModel>();
    FlightSimulator sim(launch, atmos, ground,
                        /*aero*/ nullptr, /*bounce*/ nullptr, roll);
    sim.run();

    const LandingResult r = sim.getLandingResult();
    std::printf("Landing: %.1f %.1f %.1f yards\n", r.xF, r.yF, r.zF);
    return 0;
}
