#include <gtest/gtest.h>

#include <libgolf.hpp>
#include <ground_physics.hpp>

#include <cmath>

// This file tests the bounce/roll transition free function. Bounce physics
// is tested through DefaultBounceModel; see test_default_bounce_model.cpp.
// Roll physics is tested through DefaultRollModel; see
// test_default_roll_model.cpp.

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
