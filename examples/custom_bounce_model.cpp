// Minimal custom BounceModel: fixed COR, no spin coupling.
// Decompose velocity along surface normal, reflect normal component, scale
// tangential by a fixed friction factor. Spin passes through unchanged.

#include "BounceModel.hpp"
#include "FlightSimulator.hpp"
#include "ground_surface.hpp"
#include "math_utils.hpp"

#include <cstdio>
#include <memory>

class FixedCorBounceModel : public BounceModel
{
public:
    FixedCorBounceModel(float cor, float tangentialRetention)
        : cor_(cor), tang_(tangentialRetention) {}

    BounceResult resolveBounce(const BounceState &s,
                               const GroundSurface & /*surface*/) const override
    {
        const Vector3D vn = math_utils::project(s.velocity, s.surfaceNormal);
        const Vector3D vt{s.velocity[0] - vn[0],
                          s.velocity[1] - vn[1],
                          s.velocity[2] - vn[2]};

        const Vector3D vnPost{-cor_ * vn[0], -cor_ * vn[1], -cor_ * vn[2]};
        const Vector3D vtPost{tang_ * vt[0], tang_ * vt[1], tang_ * vt[2]};

        return {{vnPost[0] + vtPost[0],
                 vnPost[1] + vtPost[1],
                 vnPost[2] + vtPost[2]},
                s.spinVector};
    }

private:
    float cor_;
    float tang_;
};

int main()
{
    const LaunchData launch{160.0F, 11.0F, 0.0F, 3000.0F, 0.0F};
    const AtmosphericData atmos{70.0F, 0.0F, 0.0F, 0.0F, 0.0F, 50.0F, 29.92F};
    GroundSurface ground;

    auto bounce = std::make_shared<FixedCorBounceModel>(0.35F, 0.6F);
    FlightSimulator sim(launch, atmos, ground, /*aero*/ nullptr, bounce);
    sim.run();

    const LandingResult r = sim.getLandingResult();
    std::printf("Landing: %.1f %.1f %.1f yards\n", r.xF, r.yF, r.zF);
    return 0;
}
