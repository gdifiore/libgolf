#include <gtest/gtest.h>

#include <libgolf.hpp>
#include <ground_physics.hpp>

#include <cmath>

// Test bounce on flat horizontal surface — low-energy impact uses simple
// friction retention (Penner gating: speed below 20 m/s threshold).
TEST(GroundPhysicsTest, BounceOnFlatGround)
{
    Vector3D velocity{0.0F, 10.0F, -5.0F};  // Slow impact, ~3.4 m/s — below Penner threshold
    Vector3D normal{0.0F, 0.0F, 1.0F};
    float spinRate = 100.0F;

    GroundSurface surface;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.3F;
    surface.firmness = 0.8F;
    surface.spinRetention = 0.75F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, spinRate, surface);

    // Vertical velocity should reverse and reduce by COR
    EXPECT_NEAR(result.newVelocity[2], 3.0F, 0.1F);  // -(-5) * 0.6 = 3.0

    // Below Penner speed threshold → simple friction retention only.
    // frictionFactor = 1 - 0.3 * (1 - 0.8) = 0.94 → 10 * 0.94 = 9.4
    EXPECT_NEAR(result.newVelocity[1], 9.4F, 0.05F);

    // Spin should be reduced
    EXPECT_NEAR(result.newSpinRate, 75.0F, 0.1F);  // 100 * 0.75
}

// Penner spin-back: steep, fast impact with high backspin reverses tangent.
// This is the wedge-check / spin-back behavior.
TEST(GroundPhysicsTest, HighSpinSteepImpactReversesTangent)
{
    // ~72 ft/s ≈ 22 m/s, above Penner speed threshold.
    // Steep impact: vy = 40, vz = -60 → angle from surface = atan(60/40) ≈ 56°,
    // well above 15° critical.
    Vector3D velocity{0.0F, 40.0F, -60.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;
    // criticalAngle = 15° default

    // Forward Penner term = 1.0 * 72.1 * sin(56.3° - 15°) ≈ 47.6 ft/s.
    // Spinback term = 2 * 0.07 * 3000 / 7 = 60 ft/s.
    // Net ≈ -12.4 ft/s → tangent reverses.
    float extremeSpin = 3000.0F;  // rad/s ≈ 28650 rpm (extreme but valid for test)
    auto result = GroundPhysics::calculateBounce(velocity, normal, extremeSpin, surface);

    // Tangent velocity should be reversed (negative y) — ball spins back.
    EXPECT_LT(result.newVelocity[1], 0.0F);
}

// High-spin steep impact: high spin reduces (and can reverse) tangent
// relative to low spin under same kinematics.
TEST(GroundPhysicsTest, HighSpinReducesTangentVelocityMoreThanLowSpin)
{
    // ~78 ft/s ≈ 23.8 m/s, comfortably above Penner threshold.
    // Angle to surface = atan(30/72) ≈ 22.6° > 15° crit.
    Vector3D velocity{0.0F, 72.0F, -30.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;  // Isolate spin effect
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    float lowSpin = 50.0F;    // ~480 rpm
    float highSpin = 300.0F;  // ~2865 rpm
    auto resultLow = GroundPhysics::calculateBounce(velocity, normal, lowSpin, surface);
    auto resultHigh = GroundPhysics::calculateBounce(velocity, normal, highSpin, surface);

    // High spin → smaller forward tangent (Penner spinback term subtracts more)
    EXPECT_LT(resultHigh.newVelocity[1], resultLow.newVelocity[1]);

    // Difference = 2R(ωhigh - ωlow)/7 = 2 * 0.07 * 250 / 7 = 5.0 ft/s
    float velocityDifference = resultLow.newVelocity[1] - resultHigh.newVelocity[1];
    EXPECT_NEAR(velocityDifference, 5.0F, 0.5F);
}

// Zero spin steep+fast impact: only the sin(θ-θ_crit) retention term acts;
// no spinback subtraction, so forward motion is preserved (reduced).
TEST(GroundPhysicsTest, ZeroSpinNoSpinbackReduction)
{
    // ~52 ft/s ≈ 15.8 m/s — below threshold, falls to simple retention.
    Vector3D velocity{0.0F, 30.0F, -10.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    auto result = GroundPhysics::calculateBounce(velocity, normal, 0.0F, surface);

    // No friction, no Penner gating triggered → tangent unchanged.
    EXPECT_NEAR(result.newVelocity[1], 30.0F, 0.01F);
}

// Critical-angle gating: shallow impact with high backspin must NOT spin
// back. Driver-like trajectories cannot reverse tangent.
TEST(GroundPhysicsTest, ShallowImpactDoesNotSpinBackEvenAtHighSpin)
{
    // Driver-like impact: ~10° from surface, fast. 100 ft/s tangent, 18 ft/s normal.
    // Angle to surface = atan(18/100) ≈ 10.2° < 15° crit.
    Vector3D velocity{0.0F, 100.0F, -18.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;  // Isolate gating
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    float highSpin = 500.0F;  // ~4775 rpm — extreme backspin

    auto result = GroundPhysics::calculateBounce(velocity, normal, highSpin, surface);

    // Below critical angle → simple retention path → tangent unchanged.
    EXPECT_NEAR(result.newVelocity[1], 100.0F, 0.01F);
    // Definitely not reversed.
    EXPECT_GT(result.newVelocity[1], 0.0F);
}

// Low-energy gating: chip-shot impact (steep but slow) must NOT spin back.
TEST(GroundPhysicsTest, LowEnergyChipDoesNotSpinBackEvenWithHighSpin)
{
    // Chip: steep angle (~45°) but slow (~14 ft/s ≈ 4.3 m/s ≪ 20 m/s).
    Vector3D velocity{0.0F, 10.0F, -10.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface surface;
    surface.restitution = 0.4F;
    surface.frictionStatic = 0.0F;
    surface.firmness = 1.0F;
    surface.spinRetention = 1.0F;

    float highSpin = 600.0F;  // ~5730 rpm — well above any real chip

    auto result = GroundPhysics::calculateBounce(velocity, normal, highSpin, surface);

    // Below energy threshold → simple retention → forward tangent preserved.
    EXPECT_GT(result.newVelocity[1], 0.0F);
    EXPECT_NEAR(result.newVelocity[1], 10.0F, 0.01F);
}

// Per-surface critical angle: a softer green (higher critAngle) suppresses
// Penner gating for the same impact geometry.
TEST(GroundPhysicsTest, HigherCriticalAngleSuppressesSpinback)
{
    // ~75 ft/s ≈ 22.9 m/s, above threshold. Angle = atan(28/70) ≈ 21.8°.
    Vector3D velocity{0.0F, 70.0F, -28.0F};
    Vector3D normal{0.0F, 0.0F, 1.0F};

    GroundSurface firm;
    firm.frictionStatic = 0.0F;
    firm.firmness = 1.0F;
    firm.criticalAngle = 15.0F * physics_constants::DEG_TO_RAD;  // standard

    GroundSurface soft;
    soft.frictionStatic = 0.0F;
    soft.firmness = 1.0F;
    soft.criticalAngle = 25.0F * physics_constants::DEG_TO_RAD;  // 21.8° < 25° → gated out

    float spin = 250.0F;
    auto resultFirm = GroundPhysics::calculateBounce(velocity, normal, spin, firm);
    auto resultSoft = GroundPhysics::calculateBounce(velocity, normal, spin, soft);

    // Firm surface engages Penner (subtracts spinback) → smaller tangent.
    // Soft surface bypasses Penner (simple retention = unchanged) → full tangent.
    EXPECT_LT(resultFirm.newVelocity[1], resultSoft.newVelocity[1]);
    EXPECT_NEAR(resultSoft.newVelocity[1], 70.0F, 0.01F);
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
