#include <gtest/gtest.h>

#include <DefaultRollModel.hpp>
#include <RollModel.hpp>
#include <ground_surface.hpp>
#include <math_utils.hpp>
#include <physics_constants.hpp>

#include <cmath>

namespace
{
constexpr float R = physics_constants::STD_BALL_RADIUS_FT;

RollState makeState(Vector3D velocity, Vector3D normal, float dt = 0.01F)
{
    return RollState{
        {0.0F, 0.0F, 0.0F},
        velocity,
        {0.0F, 0.0F, 0.0F},
        normal,
        R,
        dt
    };
}
} // namespace

TEST(DefaultRollModelTest, FlatGroundDecelerates)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto result = model.step(
        makeState({10.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.1F),
        surface);

    EXPECT_LT(result.newVelocity[0], 10.0F);
    EXPECT_GT(result.newVelocity[0], 0.0F);
    EXPECT_NEAR(result.newVelocity[1], 0.0F, 1e-4F);
}

TEST(DefaultRollModelTest, DownhillAccelerates)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.1F;

    const float angle = 30.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    auto result = model.step(
        makeState({0.0F, 1.0F, 0.0F}, normal, 0.1F),
        surface);

    EXPECT_GT(result.newVelocity[1], 1.0F);
}

TEST(DefaultRollModelTest, UphillDecelerates)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.15F;

    const float angle = 20.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, -std::sin(angle), std::cos(angle)};

    auto result = model.step(
        makeState({0.0F, 5.0F, 0.0F}, normal, 0.1F),
        surface);

    EXPECT_LT(result.newVelocity[1], 5.0F);
}

TEST(DefaultRollModelTest, SteepSlopeAcceleratesSignificantly)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;
    surface.firmness = 0.8F;

    const float angle = 60.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    auto result = model.step(
        makeState({0.0F, 2.0F, 0.0F}, normal, 0.1F),
        surface);

    // dv/dt > 5 ft/s² implied by old test → over 0.1 s, gain >0.5 ft/s.
    EXPECT_GT(result.newVelocity[1] - 2.0F, 0.5F);
}

TEST(DefaultRollModelTest, DirectionPreservedAt45Degrees)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto result = model.step(
        makeState({10.0F, 10.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.1F),
        surface);

    EXPECT_NEAR(result.newVelocity[0], result.newVelocity[1], 1e-3F);
}

TEST(DefaultRollModelTest, AtRestWhenVelocityBelowStoppingThreshold)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    // Initial speed below STOPPING_VELOCITY: stops immediately.
    auto result = model.step(
        makeState({0.05F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.01F),
        surface);

    EXPECT_TRUE(result.atRest);
}

TEST(DefaultRollModelTest, NotAtRestWhileMovingFast)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto result = model.step(
        makeState({15.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.01F),
        surface);

    EXPECT_FALSE(result.atRest);
}

TEST(DefaultRollModelTest, SignFlipClampsToZero)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    // Slow forward velocity + huge dt → friction would reverse direction.
    auto result = model.step(
        makeState({0.2F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 1.0F),
        surface);

    EXPECT_GE(result.newVelocity[0], 0.0F);
}

TEST(DefaultRollModelTest, NearZeroSlopeStartGetsAccelerated)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.15F;

    const float angle = 10.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    auto result = model.step(
        makeState({0.0F, 0.02F, 0.0F}, normal, 0.01F),
        surface);

    // Velocity below STOPPING_VELOCITY: sign-flip clamp must NOT kick in,
    // so gravity along slope can grow it.
    EXPECT_GT(result.newVelocity[1], 0.02F);
}

TEST(DefaultRollModelTest, SpinDecaysButPreservesAxis)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    RollState state{
        {0.0F, 0.0F, 0.0F},
        {10.0F, 0.0F, 0.0F},
        {500.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        R,
        0.1F
    };

    auto result = model.step(state, surface);

    EXPECT_LT(math_utils::magnitude(result.newSpinVector), 500.0F);
    EXPECT_GT(math_utils::magnitude(result.newSpinVector), 0.0F);
    EXPECT_NEAR(result.newSpinVector[1], 0.0F, 1e-4F);
    EXPECT_NEAR(result.newSpinVector[2], 0.0F, 1e-4F);
}

TEST(DefaultRollModelTest, SpinDecaysToZeroOverTime)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    RollState state{
        {0.0F, 0.0F, 0.0F},
        {5.0F, 0.0F, 0.0F},
        // Magnitude < SPIN_DECAY_RATE * dt for one step → snaps to zero.
        {1.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
        R,
        1.0F
    };

    auto result = model.step(state, surface);

    EXPECT_NEAR(math_utils::magnitude(result.newSpinVector), 0.0F, 1e-6F);
}

TEST(DefaultRollModelTest, ZeroVelocityProducesZeroAcceleration)
{
    DefaultRollModel model;
    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto result = model.step(
        makeState({0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.1F),
        surface);

    EXPECT_NEAR(result.newVelocity[0], 0.0F, 1e-6F);
    EXPECT_NEAR(result.newVelocity[1], 0.0F, 1e-6F);
    EXPECT_TRUE(result.atRest);
}

TEST(DefaultRollModelTest, HigherFrictionSlowsFaster)
{
    DefaultRollModel model;

    GroundSurface low;
    low.frictionDynamic = 0.15F;
    GroundSurface high;
    high.frictionDynamic = 0.5F;

    auto resultLow = model.step(
        makeState({20.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.1F),
        low);
    auto resultHigh = model.step(
        makeState({20.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 1.0F}, 0.1F),
        high);

    EXPECT_LT(resultHigh.newVelocity[0], resultLow.newVelocity[0]);
}
