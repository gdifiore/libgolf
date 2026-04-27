#include <gtest/gtest.h>

#include <BounceModel.hpp>
#include <DefaultBounceModel.hpp>
#include <ground_surface.hpp>
#include <math_utils.hpp>
#include <physics_constants.hpp>

#include <cmath>

namespace
{
constexpr float R = physics_constants::STD_BALL_RADIUS_FT;

// Helper: build a BounceState. Spin is laid along +X — for ball moving +Y on
// a +Z normal, t̂ × n̂ = +X, so positive scalar = backspin (golf convention).
BounceState makeState(Vector3D velocity, Vector3D normal, float spinScalarRadPerSec)
{
    return BounceState{
        velocity,
        normal,
        Vector3D{spinScalarRadPerSec, 0.0F, 0.0F},
        R
    };
}
} // namespace

// Low-energy impact (~3.4 m/s normal speed): simple friction retention path.
// The Penner spinback term does NOT engage — chips and slow drops can't check.
TEST(DefaultBounceModelTest, BounceOnFlatGroundLowEnergy)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.3F;
    surface.firmness = 0.8F;
    surface.spinRetention = 0.75F;

    auto result = model.resolveBounce(
        makeState({0.0F, 10.0F, -5.0F}, {0.0F, 0.0F, 1.0F}, 100.0F),
        surface);

    // Vertical: COR slightly reduced by spin × velScale, but velScale is low
    // (~0.063) and spinRpm ~955 → reduction <2%, so newVz ≈ -(-5)*0.6 ≈ 3.0.
    EXPECT_NEAR(result.newVelocity[2], 3.0F, 0.1F);

    // Below Penner energy gate → simple friction retention only.
    // frictionFactor = 1 - 0.3 * (1 - 0.8) = 0.94 → 10 * 0.94 = 9.4
    EXPECT_NEAR(result.newVelocity[1], 9.4F, 0.05F);

    // Spin retention scales the spin vector uniformly.
    EXPECT_NEAR(result.newSpinVector[0], 75.0F, 0.1F);
}

// Steep + energetic + high backspin → Penner reverses tangent (wedge check).
TEST(DefaultBounceModelTest, HighSpinSteepImpactReversesTangent)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    // ~72 ft/s ≈ 22 m/s, above Penner energy gate.
    // Angle from surface = atan(60/40) ≈ 56°, well above 15° crit.
    auto result = model.resolveBounce(
        makeState({0.0F, 40.0F, -60.0F}, {0.0F, 0.0F, 1.0F}, 3000.0F),
        surface);

    EXPECT_LT(result.newVelocity[1], 0.0F);
}

// Penner branch: higher spin reduces tangent more (combined spinback +
// spin-coupled retention curve).
TEST(DefaultBounceModelTest, HighSpinReducesTangentVelocityMoreThanLowSpin)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    // ~78 ft/s ≈ 23.8 m/s, comfortably above Penner gate.
    // Angle to surface = atan(30/72) ≈ 22.6° > 15° crit.
    Vector3D velocity{0.0F, 72.0F, -30.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    auto resultLow  = model.resolveBounce(makeState(velocity, normal, 50.0F),  surface);
    auto resultHigh = model.resolveBounce(makeState(velocity, normal, 300.0F), surface);

    EXPECT_LT(resultHigh.newVelocity[1], resultLow.newVelocity[1]);

    const float diff = resultLow.newVelocity[1] - resultHigh.newVelocity[1];
    EXPECT_GT(diff, 5.0F);
    EXPECT_LT(diff, 9.0F);
}

// Zero spin + no friction: tangent unchanged on the simple-retention path.
TEST(DefaultBounceModelTest, ZeroSpinNoTangentReduction)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = model.resolveBounce(
        makeState({0.0F, 30.0F, -10.0F}, {0.0F, 0.0F, 1.0F}, 0.0F),
        surface);

    EXPECT_NEAR(result.newVelocity[1], 30.0F, 0.01F);
}

// Critical-angle gate: shallow drive-like impact must NOT spin back.
TEST(DefaultBounceModelTest, ShallowImpactDoesNotSpinBackEvenAtHighSpin)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    // 100 ft/s tangent, 18 ft/s normal: angle ≈ 10.2° < 15° crit.
    auto result = model.resolveBounce(
        makeState({0.0F, 100.0F, -18.0F}, {0.0F, 0.0F, 1.0F}, 500.0F),
        surface);

    EXPECT_NEAR(result.newVelocity[1], 100.0F, 0.01F);
    EXPECT_GT(result.newVelocity[1], 0.0F);
}

// Energy gate: chip-shot impact (steep but slow) must NOT spin back.
TEST(DefaultBounceModelTest, LowEnergyChipDoesNotSpinBackEvenWithHighSpin)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    // ~14 ft/s ≈ 4.3 m/s, well below 20 m/s Penner energy gate.
    auto result = model.resolveBounce(
        makeState({0.0F, 10.0F, -10.0F}, {0.0F, 0.0F, 1.0F}, 600.0F),
        surface);

    EXPECT_GT(result.newVelocity[1], 0.0F);
    EXPECT_NEAR(result.newVelocity[1], 10.0F, 0.01F);
}

// Per-surface critical angle: a softer green raises the gate so the same
// impact geometry no longer triggers Penner.
TEST(DefaultBounceModelTest, HigherCriticalAngleSuppressesSpinback)
{
    DefaultBounceModel model;

    GroundSurface firm;
    firm.frictionStatic = 0.0F;
    firm.firmness = 1.0F;
    firm.criticalAngle = 15.0F * physics_constants::DEG_TO_RAD;

    GroundSurface soft;
    soft.frictionStatic = 0.0F;
    soft.firmness = 1.0F;
    soft.criticalAngle = 25.0F * physics_constants::DEG_TO_RAD;

    // ~75 ft/s, angle ≈ 21.8°.
    Vector3D velocity{0.0F, 70.0F, -28.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    auto resultFirm = model.resolveBounce(makeState(velocity, normal, 250.0F), firm);
    auto resultSoft = model.resolveBounce(makeState(velocity, normal, 250.0F), soft);

    EXPECT_LT(resultFirm.newVelocity[1], resultSoft.newVelocity[1]);
    EXPECT_NEAR(resultSoft.newVelocity[1], 70.0F, 0.01F);
}

// Flop shot: high spin + high normal-impact speed → big COR reduction.
TEST(DefaultBounceModelTest, FlopShotHighSpinReducesNormalRebound)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    Vector3D velocity{0.0F, 5.0F, -80.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    auto resultHigh = model.resolveBounce(makeState(velocity, normal, 366.0F), surface);
    auto resultLow  = model.resolveBounce(makeState(velocity, normal, 50.0F),  surface);

    EXPECT_LT(resultHigh.newVelocity[2], resultLow.newVelocity[2]);
    EXPECT_LT(resultHigh.newVelocity[2], 25.0F);
    EXPECT_GT(resultHigh.newVelocity[2], 0.0F);
    EXPECT_GT(resultLow.newVelocity[2], 30.0F);
}

// Velocity scale clamps the COR penalty for low-speed drops, even at high spin.
TEST(DefaultBounceModelTest, LowSpeedHighSpinKeepsCorNearBase)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = model.resolveBounce(
        makeState({0.0F, 0.0F, -10.0F}, {0.0F, 0.0F, 1.0F}, 400.0F),
        surface);

    EXPECT_GT(result.newVelocity[2], 4.4F);
    EXPECT_LT(result.newVelocity[2], 5.0F);
}

TEST(DefaultBounceModelTest, BounceOn45DegreeSlope)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.2F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    const float angle = 45.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    auto result = model.resolveBounce(
        makeState({0.0F, 0.0F, -10.0F}, normal, 0.0F),
        surface);

    EXPECT_GT(result.newVelocity[1], 0.0F);
    EXPECT_GT(math_utils::dot(result.newVelocity, normal), 0.0F);
}

TEST(DefaultBounceModelTest, BouncePreservesTangentialDirection)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.7F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = model.resolveBounce(
        makeState({5.0F, 10.0F, -8.0F}, {0.0F, 0.0F, 1.0F}, 0.0F),
        surface);

    EXPECT_NEAR(result.newVelocity[0], 5.0F, 0.001F);
    EXPECT_NEAR(result.newVelocity[1], 10.0F, 0.001F);
    EXPECT_NEAR(result.newVelocity[2], 5.6F, 0.1F);
}

TEST(DefaultBounceModelTest, BounceWorksWithUnitNormal)
{
    DefaultBounceModel model;
    const float angle = 30.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    EXPECT_NEAR(math_utils::magnitude(normal), 1.0F, 0.001F);

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.2F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = model.resolveBounce(
        makeState({0.0F, 0.0F, -10.0F}, normal, 0.0F),
        surface);

    EXPECT_GT(math_utils::magnitude(result.newVelocity), 0.0F);
}

TEST(DefaultBounceModelTest, HighFrictionReducesTangentialVelocity)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.8F;
    surface.firmness = 0.5F;
    surface.spinRetention = 0.5F;

    auto result = model.resolveBounce(
        makeState({10.0F, 10.0F, -5.0F}, {0.0F, 0.0F, 1.0F}, 100.0F),
        surface);

    const float tangent = std::sqrt(
        result.newVelocity[0] * result.newVelocity[0] +
        result.newVelocity[1] * result.newVelocity[1]);
    const float originalTangent = std::sqrt(10.0F * 10.0F + 10.0F * 10.0F);

    EXPECT_LT(tangent, originalTangent * 0.8F);
}

TEST(DefaultBounceModelTest, BounceOnSteepSlope)
{
    DefaultBounceModel model;
    const float angle = 80.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.3F;
    surface.firmness = 0.8F;
    surface.spinRetention = 0.5F;

    auto state = makeState({0.0F, -2.0F, -10.0F}, normal, 0.0F);
    EXPECT_LT(math_utils::dot(state.velocity, normal), 0.0F);

    auto result = model.resolveBounce(state, surface);

    EXPECT_GT(math_utils::dot(result.newVelocity, normal), 0.0F);
    EXPECT_GT(math_utils::magnitude(result.newVelocity), 0.0F);
}

TEST(DefaultBounceModelTest, BounceOnVerticalWall)
{
    DefaultBounceModel model;
    Vector3D velocity{5.0F, 0.0F, -5.0F};
    Vector3D normal{1.0F, 0.0F, 0.0F};

    GroundSurface surface;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.4F;
    surface.firmness = 0.5F;
    surface.spinRetention = 0.5F;

    auto result = model.resolveBounce(makeState(velocity, normal, 0.0F), surface);

    EXPECT_LT(result.newVelocity[0], 0.0F);
    EXPECT_NEAR(result.newVelocity[0], -velocity[0] * surface.restitution, 0.5F);

    EXPECT_LT(result.newVelocity[2], 0.0F);
    EXPECT_GT(result.newVelocity[2], velocity[2]);
    EXPECT_NEAR(result.newVelocity[2], velocity[2] * 0.8F, 0.5F);
}

TEST(DefaultBounceModelTest, SpinRetentionScalesAllSpinAxes)
{
    DefaultBounceModel model;
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.spinRetention = 0.6F;

    BounceState state{
        {0.0F, 0.0F, -5.0F},
        {0.0F, 0.0F, 1.0F},
        {100.0F, -50.0F, 25.0F},
        R
    };

    auto result = model.resolveBounce(state, surface);

    EXPECT_NEAR(result.newSpinVector[0], 60.0F, 1e-4F);
    EXPECT_NEAR(result.newSpinVector[1], -30.0F, 1e-4F);
    EXPECT_NEAR(result.newSpinVector[2], 15.0F, 1e-4F);
}
