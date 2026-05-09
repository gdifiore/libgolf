// Minimal custom AerodynamicModel: constant Cd, no lift, no spin coupling.
// Demonstrates subclassing AerodynamicModel and injecting via FlightSimulator.

#include "AerodynamicModel.hpp"
#include "FlightSimulator.hpp"
#include "ground_surface.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

class ConstantCdModel : public AerodynamicModel
{
public:
    explicit ConstantCdModel(float cd) : cd_(cd) {}

    Vector3D computeAcceleration(const AerodynamicState &s) const override
    {
        const float vRelX = s.velocity[0] - s.windVelocity[0];
        const float vRelY = s.velocity[1] - s.windVelocity[1];
        const float vRelZ = s.velocity[2] - s.windVelocity[2];
        const float vw = std::sqrt(vRelX * vRelX + vRelY * vRelY + vRelZ * vRelZ);

        if (vw < 0.01F) return {0.0F, 0.0F, 0.0F};

        // F = -c0 * Cd * vw * vRel  (ignores Magnus / spin)
        const float scale = -s.c0 * cd_ * vw;
        return {scale * vRelX, scale * vRelY, scale * vRelZ};
    }

    double computeSpinDecayTau(const AerodynamicState & /*s*/) const override
    {
        return 1.0e6; // effectively no decay
    }

private:
    float cd_;
};

int main()
{
    const LaunchData launch{160.0F, 11.0F, 0.0F, 3000.0F, 0.0F};
    const AtmosphericData atmos{70.0F, 0.0F, 0.0F, 0.0F, 0.0F, 50.0F, 29.92F};
    GroundSurface ground;

    auto aero = std::make_shared<ConstantCdModel>(0.30F);
    FlightSimulator sim(launch, atmos, ground, aero);
    sim.run();

    const LandingResult r = sim.getLandingResult();
    std::printf("Landing: %.1f %.1f %.1f yards\n", r.xF, r.yF, r.zF);
    return 0;
}
