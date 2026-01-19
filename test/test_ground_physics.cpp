#include <gtest/gtest.h>

#include <libgolf.hpp>
#include <ground_physics.hpp>

#include <cmath>

// Test bounce on flat horizontal surface (backward compatibility test)
TEST(GroundPhysicsTest, BounceOnFlatGround)
{
    Vector3D velocity{0.0F, 10.0F, -5.0F};  // Moving forward and downward
    Vector3D normal{0.0F, 0.0F, 1.0F};       // Flat horizontal surface
    float spinRate = 100.0F;

    GroundSurface surface;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.3F;
    surface.firmness = 0.8F;
    surface.spinRetention = 0.75F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, spinRate, surface);

    // Vertical velocity should reverse and reduce by COR
    EXPECT_NEAR(result.newVelocity[2], 3.0F, 0.1F);  // -(-5) * 0.6 = 3.0

    // Horizontal velocity should be reduced by friction AND spin (Penner model)
    // frictionFactor = 1 - 0.3 * (1 - 0.8) = 0.94
    // After friction: 10.0 * 0.94 = 9.4
    // spinContribution = (2 * 0.07 * 100) / 7 = 2.0
    // reductionFactor = 1 - 2.0/9.4 = 0.787
    // Final: 9.4 * 0.787 = 7.4
    EXPECT_NEAR(result.newVelocity[1], 7.4F, 0.5F);

    // Spin should be reduced
    EXPECT_NEAR(result.newSpinRate, 75.0F, 0.1F);  // 100 * 0.75
}

// Test Penner model: high spin reduces tangent velocity more than low spin
TEST(GroundPhysicsTest, HighSpinReducesTangentVelocityMoreThanLowSpin)
{
    Vector3D velocity{0.0F, 50.0F, -20.0F};  // Fast forward motion with downward component
    Vector3D normal{0.0F, 0.0F, 1.0F};       // Flat horizontal surface

    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;  // No friction to isolate spin effect
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    // Low spin case (typical driver)
    float lowSpin = 50.0F;  // rad/s (~500 rpm)
    auto resultLow = GroundPhysics::calculateBounce(velocity, normal, lowSpin, surface);

    // High spin case (typical wedge)
    float highSpin = 300.0F;  // rad/s (~3000 rpm)
    auto resultHigh = GroundPhysics::calculateBounce(velocity, normal, highSpin, surface);

    // High spin should result in lower tangent velocity
    EXPECT_LT(resultHigh.newVelocity[1], resultLow.newVelocity[1]);

    // Quantify the difference using Penner model:
    // spinContribution = (2 * r * omega) / 7 = (2 * 0.07 * omega) / 7 = 0.02 * omega
    // Low: 0.02 * 50 = 1.0 ft/s reduction
    // High: 0.02 * 300 = 6.0 ft/s reduction
    // Difference should be approximately 5.0 ft/s
    float velocityDifference = resultLow.newVelocity[1] - resultHigh.newVelocity[1];
    EXPECT_NEAR(velocityDifference, 5.0F, 0.5F);
}

// Test Penner model: zero spin has no additional velocity reduction
TEST(GroundPhysicsTest, ZeroSpinNoAdditionalVelocityReduction)
{
    Vector3D velocity{0.0F, 30.0F, -10.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.0F;  // No friction
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // With zero spin and no friction, tangent velocity should be unchanged
    EXPECT_NEAR(result.newVelocity[1], 30.0F, 0.01F);
}

// Test bounce on 45-degree slope
TEST(GroundPhysicsTest, BounceOn45DegreeSlope)
{
    // Ball moving straight down
    Vector3D velocity{0.0F, 0.0F, -10.0F};

    // 45-degree slope (normal pointing up and to the side)
    float angle = 45.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.2F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // Velocity should be reflected off the slope
    // The bounce should redirect the ball in +Y direction
    EXPECT_GT(result.newVelocity[1], 0.0F);  // Should have positive Y component

    // Check that velocity is moving away from the surface (not into it)
    float velocityDotNormal = math_utils::dot(result.newVelocity, normal);
    EXPECT_GT(velocityDotNormal, 0.0F);  // Should be moving away from surface
}

// Test bounce conserves tangential direction
TEST(GroundPhysicsTest, BouncePreservesTangentialDirection)
{
    Vector3D velocity{5.0F, 10.0F, -8.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.7F;
    surface.frictionStatic = 0.0F;  // No friction
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // Tangential components should be preserved (no friction)
    EXPECT_NEAR(result.newVelocity[0], 5.0F, 0.001F);
    EXPECT_NEAR(result.newVelocity[1], 10.0F, 0.001F);

    // Normal component should reverse and reduce
    EXPECT_NEAR(result.newVelocity[2], 5.6F, 0.1F);  // -(-8) * 0.7
}

// Test rolling friction on flat ground
TEST(GroundPhysicsTest, RollOnFlatGroundDecelerates)
{
    Vector3D velocity{10.0F, 0.0F, 0.0F};  // Moving laterally
    Vector3D normal{0.0F, 0.0F, 1.0F};      // Flat surface

    GroundSurface surface;
    surface.frictionDynamic = 0.2F;

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    // Should decelerate in the direction of motion
    EXPECT_LT(accel[0], 0.0F);  // Negative acceleration (deceleration)
    EXPECT_NEAR(accel[1], 0.0F, 0.001F);
    EXPECT_NEAR(accel[2], 0.0F, 0.001F);

    // Magnitude should be friction * gravity
    float expectedMag = surface.frictionDynamic * physics_constants::GRAVITY_FT_PER_S2;
    float actualMag = math_utils::magnitude(accel);
    EXPECT_NEAR(actualMag, expectedMag, 0.1F);
}

// Test rolling downhill accelerates
TEST(GroundPhysicsTest, RollDownhillAccelerates)
{
    Vector3D velocity{0.0F, 1.0F, 0.0F};  // Moving slowly forward

    // 30-degree downhill slope
    float angle = 30.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.frictionDynamic = 0.1F;  // Low friction

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    // Should accelerate forward (positive Y) due to gravity down the slope
    // Gravity component down slope should overcome friction
    EXPECT_GT(accel[1], 0.0F);  // Net acceleration downhill
}

// Test rolling uphill decelerates
TEST(GroundPhysicsTest, RollUphillDecelerates)
{
    Vector3D velocity{0.0F, 5.0F, 0.0F};  // Moving forward (uphill)

    // 20-degree uphill slope (normal tilted backward)
    float angle = 20.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, -std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.frictionDynamic = 0.15F;

    auto accel = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    // Should decelerate (gravity pulls backward, friction opposes forward motion)
    EXPECT_LT(accel[1], 0.0F);  // Negative acceleration (deceleration)
}

// Test rolling with zero velocity returns zero acceleration
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

// Test transition to roll - should transition when close to ground and slow
TEST(GroundPhysicsTest, ShouldTransitionToRollWhenCloseAndSlow)
{
    Vector3D velocity{5.0F, 5.0F, 0.5F};  // Slow vertical velocity
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float height = 0.05F;  // Close to ground

    bool shouldTransition = GroundPhysics::shouldTransitionToRoll(velocity, normal, height);

    EXPECT_TRUE(shouldTransition);
}

// Test transition to roll - should NOT transition when too high
TEST(GroundPhysicsTest, ShouldNotTransitionToRollWhenTooHigh)
{
    Vector3D velocity{5.0F, 5.0F, 0.5F};
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float height = 1.0F;  // Too high above ground

    bool shouldTransition = GroundPhysics::shouldTransitionToRoll(velocity, normal, height);

    EXPECT_FALSE(shouldTransition);
}

// Test transition to roll - should NOT transition when moving fast vertically
TEST(GroundPhysicsTest, ShouldNotTransitionToRollWhenMovingFastVertically)
{
    Vector3D velocity{5.0F, 5.0F, -5.0F};  // Fast downward velocity
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float height = 0.05F;  // Close to ground

    bool shouldTransition = GroundPhysics::shouldTransitionToRoll(velocity, normal, height);

    EXPECT_FALSE(shouldTransition);
}

// Test bounce normal vector is unit length requirement
TEST(GroundPhysicsTest, BounceWorksWithUnitNormal)
{
    Vector3D velocity{0.0F, 0.0F, -10.0F};

    // Create a normalized 30-degree slope normal
    float angle = 30.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    // Verify it's unit length
    float normalMag = math_utils::magnitude(normal);
    EXPECT_NEAR(normalMag, 1.0F, 0.001F);

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.2F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // Should produce valid result without errors
    float resultMag = math_utils::magnitude(result.newVelocity);
    EXPECT_GT(resultMag, 0.0F);
}

// Test high friction surface significantly reduces tangential velocity
TEST(GroundPhysicsTest, HighFrictionReducesTangentialVelocity)
{
    Vector3D velocity{10.0F, 10.0F, -5.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface highFriction;
    highFriction.restitution = 0.5F;
    highFriction.frictionStatic = 0.8F;  // High friction
    highFriction.firmness = 0.5F;        // Soft surface
    highFriction.spinRetention = 0.5F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 100.0F, highFriction);

    // Tangential velocity should be significantly reduced
    float tangentialSpeed = std::sqrt(
        result.newVelocity[0] * result.newVelocity[0] +
        result.newVelocity[1] * result.newVelocity[1]
    );

    float originalTangentialSpeed = std::sqrt(10.0F * 10.0F + 10.0F * 10.0F);

    EXPECT_LT(tangentialSpeed, originalTangentialSpeed * 0.8F);
}

// Test bounce on very steep slope (80 degrees)
TEST(GroundPhysicsTest, BounceOnSteepSlope)
{
    // Ball falling down and slightly backward onto steep slope
    Vector3D velocity{0.0F, -2.0F, -10.0F};

    // 80-degree slope (very steep, near vertical)
    float angle = 80.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    // Verify normal is unit length
    float normalMag = math_utils::magnitude(normal);
    EXPECT_NEAR(normalMag, 1.0F, 0.001F);

    // Verify ball is moving toward surface (v · n < 0)
    float vDotNBefore = math_utils::dot(velocity, normal);
    EXPECT_LT(vDotNBefore, 0.0F);

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.3F;
    surface.firmness = 0.8F;
    surface.spinRetention = 0.5F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // Verify velocity is reflected away from surface after bounce
    float vDotNAfter = math_utils::dot(result.newVelocity, normal);
    EXPECT_GT(vDotNAfter, 0.0F);  // Should be moving away from surface

    // Result should be valid (non-zero)
    float resultMag = math_utils::magnitude(result.newVelocity);
    EXPECT_GT(resultMag, 0.0F);
}

// Test bounce on vertical wall (90 degrees)
TEST(GroundPhysicsTest, BounceOnVerticalWall)
{
    Vector3D velocity{5.0F, 0.0F, -5.0F};

    // 90-degree wall (vertical surface, normal points horizontally)
    Vector3D normal{1.0F, 0.0F, 0.0F};

    GroundSurface surface;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.4F;
    surface.firmness = 0.5F;  // Medium firmness allows friction
    surface.spinRetention = 0.5F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // Horizontal component should reverse (with COR applied)
    EXPECT_LT(result.newVelocity[0], 0.0F);  // Should bounce back
    EXPECT_NEAR(result.newVelocity[0], -velocity[0] * surface.restitution, 0.5F);

    // Vertical component should be reduced by friction but not reversed
    // frictionFactor = 1.0 - 0.4 * (1.0 - 0.5) = 0.8
    EXPECT_LT(result.newVelocity[2], 0.0F);  // Still falling
    EXPECT_GT(result.newVelocity[2], velocity[2]);  // But slower (friction)
    EXPECT_NEAR(result.newVelocity[2], velocity[2] * 0.8F, 0.5F);
}

// Test roll acceleration on steep slope
TEST(GroundPhysicsTest, RollOnSteepSlopeAcceleratesSignificantly)
{
    Vector3D velocity{0.0F, 2.0F, 0.0F};

    // 60-degree slope (steep)
    float angle = 60.0F * physics_constants::DEG_TO_RAD;
    Vector3D normal{0.0F, std::sin(angle), std::cos(angle)};

    GroundSurface surface;
    surface.frictionDynamic = 0.2F;
    surface.firmness = 0.8F;

    auto acceleration = GroundPhysics::calculateRollAcceleration(velocity, normal, 0.0F, surface);

    // On steep slope, gravity component should cause significant acceleration
    float accelMag = math_utils::magnitude(acceleration);
    EXPECT_GT(accelMag, 5.0F);  // Should be significant on 60-degree slope

    // Acceleration should be primarily in +Y direction (downslope)
    EXPECT_GT(acceleration[1], 0.0F);
}
