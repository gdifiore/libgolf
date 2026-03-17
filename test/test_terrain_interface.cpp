#include <gtest/gtest.h>

#include <libgolf.hpp>
#include <terrain_interface.hpp>
#include <physics_constants.hpp>

#include <cmath>

// Simple mock terrain with a constant slope for testing
class MockSlopedTerrain : public TerrainInterface
{
public:
    MockSlopedTerrain(float slopeAngleDegrees, const GroundSurface& surface)
        : surface(surface)
    {
        // Calculate slope in Y direction (downrange)
        float angleRad = slopeAngleDegrees * physics_constants::DEG_TO_RAD;
        slopeRise = std::tan(angleRad);

        // Normal vector for a slope descending in +Y direction
        // Perpendicular to slope surface, pointing upward
        normal[0] = 0.0F;
        normal[1] = std::sin(angleRad);  // Tilts forward for downslope
        normal[2] = std::cos(angleRad);  // Upward component
    }

    float getHeight(float x, float y) const override
    {
        (void)x;  // Slope only in Y direction
        return -y * slopeRise;  // Descends as Y increases
    }

    Vector3D getNormal(float x, float y) const override
    {
        (void)x;
        (void)y;
        return normal;
    }

    const GroundSurface& getSurfaceProperties(float x, float y) const override
    {
        (void)x;
        (void)y;
        return surface;
    }

private:
    GroundSurface surface;
    float slopeRise;
    Vector3D normal;
};

TEST(TerrainInterfaceTest, FlatTerrainReturnsConstantHeight)
{
    GroundSurface surface;
    surface.height = 10.0F;

    FlatTerrain terrain(surface);

    EXPECT_NEAR(terrain.getHeight(0.0F, 0.0F), 10.0F, 0.001F);
    EXPECT_NEAR(terrain.getHeight(100.0F, 200.0F), 10.0F, 0.001F);
    EXPECT_NEAR(terrain.getHeight(-50.0F, -75.0F), 10.0F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainReturnsVerticalNormal)
{
    GroundSurface surface;
    FlatTerrain terrain(surface);

    Vector3D normal1 = terrain.getNormal(0.0F, 0.0F);
    Vector3D normal2 = terrain.getNormal(100.0F, 200.0F);

    EXPECT_NEAR(normal1[0], 0.0F, 0.001F);
    EXPECT_NEAR(normal1[1], 0.0F, 0.001F);
    EXPECT_NEAR(normal1[2], 1.0F, 0.001F);

    EXPECT_NEAR(normal2[0], 0.0F, 0.001F);
    EXPECT_NEAR(normal2[1], 0.0F, 0.001F);
    EXPECT_NEAR(normal2[2], 1.0F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainReturnsConstantSurfaceProperties)
{
    GroundSurface surface;
    surface.height = 5.0F;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.4F;
    surface.frictionDynamic = 0.3F;
    surface.firmness = 0.9F;
    surface.spinRetention = 0.8F;

    FlatTerrain terrain(surface);

    const GroundSurface& props1 = terrain.getSurfaceProperties(0.0F, 0.0F);
    const GroundSurface& props2 = terrain.getSurfaceProperties(50.0F, 100.0F);

    EXPECT_NEAR(props1.height, 5.0F, 0.001F);
    EXPECT_NEAR(props1.restitution, 0.6F, 0.001F);
    EXPECT_NEAR(props1.frictionStatic, 0.4F, 0.001F);
    EXPECT_NEAR(props1.frictionDynamic, 0.3F, 0.001F);
    EXPECT_NEAR(props1.firmness, 0.9F, 0.001F);
    EXPECT_NEAR(props1.spinRetention, 0.8F, 0.001F);

    EXPECT_NEAR(props2.height, 5.0F, 0.001F);
    EXPECT_NEAR(props2.restitution, 0.6F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainNormalIsUnitLength)
{
    GroundSurface surface;
    FlatTerrain terrain(surface);

    Vector3D normal = terrain.getNormal(0.0F, 0.0F);
    float magnitude = math_utils::magnitude(normal);

    EXPECT_NEAR(magnitude, 1.0F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainWorksWithDefaultGroundSurface)
{
    GroundSurface surface;  // Default values
    FlatTerrain terrain(surface);

    EXPECT_NEAR(terrain.getHeight(0.0F, 0.0F), 0.0F, 0.001F);

    const GroundSurface& props = terrain.getSurfaceProperties(0.0F, 0.0F);
    EXPECT_NEAR(props.restitution, 0.4F, 0.001F);  // Default COR
}

// Basic sanity checks for sloped terrain integration

TEST(TerrainInterfaceTest, MockSlopedTerrainCalculatesHeightCorrectly)
{
    GroundSurface surface;
    MockSlopedTerrain terrain(10.0F, surface);  // 10 degree downward slope

    // At y=0, height should be 0
    EXPECT_NEAR(terrain.getHeight(0.0F, 0.0F), 0.0F, 0.001F);

    // As y increases (downrange), height decreases
    float height10 = terrain.getHeight(0.0F, 10.0F);
    float height20 = terrain.getHeight(0.0F, 20.0F);

    EXPECT_LT(height10, 0.0F);     // Below starting point
    EXPECT_LT(height20, height10); // Even lower
}

TEST(TerrainInterfaceTest, MockSlopedTerrainNormalIsUnitLength)
{
    GroundSurface surface;
    MockSlopedTerrain terrain(20.0F, surface);

    Vector3D normal = terrain.getNormal(0.0F, 0.0F);
    float mag = math_utils::magnitude(normal);

    EXPECT_NEAR(mag, 1.0F, 0.001F);
}

TEST(TerrainInterfaceTest, BallBouncesOnSlopedTerrain)
{
    // Set up sloped terrain
    GroundSurface surface;
    surface.restitution = 0.5F;
    surface.frictionStatic = 0.3F;
    surface.firmness = 1.0F;
    auto terrain = std::make_shared<MockSlopedTerrain>(15.0F, surface);

    // Set up physics
    LaunchData ball{100.0, 10.0, 0.0, 2000.0, 0.0};
    AtmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};
    GolfBallPhysicsVariables physicsVars(ball, atmos);

    // Create bounce phase with sloped terrain
    BouncePhase bounce(physicsVars, ball, atmos, terrain);

    // Ball falling straight down onto slope
    BallState state;
    state.position = {0.0F, 10.0F, terrain->getHeight(0.0F, 10.0F)};
    state.velocity = {0.0F, 0.0F, -10.0F};
    state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
    state.currentTime = 0.0F;

    bounce.calculateStep(state, 0.01F);

    // After bounce on slope, ball should have forward velocity component
    EXPECT_GT(state.velocity[1], 0.0F);  // Should redirect downslope
}

TEST(TerrainInterfaceTest, BallRollsDownSlope)
{
    // Set up sloped terrain
    GroundSurface surface;
    surface.frictionDynamic = 0.15F;
    auto terrain = std::make_shared<MockSlopedTerrain>(10.0F, surface);

    // Set up physics
    LaunchData ball{100.0, 10.0, 0.0, 2000.0, 0.0};
    AtmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};
    GolfBallPhysicsVariables physicsVars(ball, atmos);

    // Create roll phase
    RollPhase roll(physicsVars, ball, atmos, terrain);

    // Ball rolling slowly on slope
    BallState state;
    state.position = {0.0F, 10.0F, terrain->getHeight(0.0F, 10.0F)};
    state.velocity = {0.0F, 2.0F, 0.0F};  // Slow forward velocity
    state.acceleration = {0.0F, 0.0F, 0.0F};
    state.currentTime = 0.0F;

    float initialVelocity = state.velocity[1];

    // Roll for a bit
    for (int i = 0; i < 10; ++i)
    {
        roll.calculateStep(state, 0.01F);
    }

    // Ball should accelerate downslope
    EXPECT_GT(state.velocity[1], initialVelocity);

    // Ball should stay on terrain surface
    float expectedHeight = terrain->getHeight(state.position[0], state.position[1]);
    EXPECT_NEAR(state.position[2], expectedHeight, 0.001F);
}
