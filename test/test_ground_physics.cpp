#include <gtest/gtest.h>

#include <libgolf.hpp>
#include <ground_physics.hpp>

#include <cmath>

// This file tests the rolling and bounce/roll transition free functions.
// Bounce physics is tested through DefaultBounceModel; see
// test_default_bounce_model.cpp.

TEST(GroundPhysicsTest, RollOnFlatGroundDecelerates)
{
    Vector3D velocity{10.0F, 0.0F, 0.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    EXPECT_LT(accel[0], 0.0F);
    EXPECT_NEAR(accel[1], 0.0F, 0.001F);
    EXPECT_NEAR(accel[2], 0.0F, 0.001F);

    float expectedMag = surface.frictionDynamic * physics_constants::GRAVITY_FT_PER_S2;
    float actualMag = math_utils::magnitude(accel);
    EXPECT_NEAR(actualMag, expectedMag, 0.1F);
}

TEST(GroundPhysicsTest, RollDownhillAccelerates)
{
    Vector3D velocity{0.0F, 1.0F, 0.0F};

    float angle = 30.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.frictionDynamic = 0.1F;

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    EXPECT_GT(accel[1], 0.0F);
}

TEST(GroundPhysicsTest, RollUphillDecelerates)
{
    Vector3D velocity{0.0F, 5.0F, 0.0F};

    float angle = 20.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, -std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.frictionDynamic = 0.15F;

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    EXPECT_LT(accel[1], 0.0F);
}

TEST(GroundPhysicsTest, RollWithZeroVelocityReturnsZeroAcceleration)
{
    Vector3D velocity{0.0F, 0.0F, 0.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    EXPECT_NEAR(accel[0], 0.0F, 0.001F);
    EXPECT_NEAR(accel[1], 0.0F, 0.001F);
    EXPECT_NEAR(accel[2], 0.0F, 0.001F);
}

TEST(GroundPhysicsTest, ShouldTransitionToRollWhenCloseAndSlow)
{
    Vector3D velocity{5.0F, 5.0F, 0.5F};
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float height = 0.05F;

    EXPECT_TRUE(GroundPhysics::shouldTransitionToRoll(velocity, normal, height));
}

TEST(GroundPhysicsTest, ShouldNotTransitionToRollWhenTooHigh)
{
    Vector3D velocity{5.0F, 5.0F, 0.5F};
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float height = 1.0F;

    EXPECT_FALSE(GroundPhysics::shouldTransitionToRoll(velocity, normal, height));
}

TEST(GroundPhysicsTest, ShouldNotTransitionToRollWhenMovingFastVertically)
{
    Vector3D velocity{5.0F, 5.0F, -5.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float height = 0.05F;

    EXPECT_FALSE(GroundPhysics::shouldTransitionToRoll(velocity, normal, height));
}

TEST(GroundPhysicsTest, RollOnSteepSlopeAcceleratesSignificantly)
{
    Vector3D velocity{0.0F, 2.0F, 0.0F};

    float angle = 60.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.frictionDynamic = 0.2F;
    surface.firmness = 0.8F;

    auto acceleration = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    float accelMag = math_utils::magnitude(acceleration);
    EXPECT_GT(accelMag, 5.0F);
    EXPECT_GT(acceleration[1], 0.0F);
}
