#include <gtest/gtest.h>
#include <cmath>

#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "atmospheric_data.hpp"
#include "launch_data.hpp"
#include "ground_surface.hpp"
#include "terrain_interface.hpp"
#include "physics_constants.hpp"

#include <memory>

class BouncePhaseTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Default test ball and atmosphere
		ball = {100.0, 10.0, 0.0, 2000.0, 0.0};
		atmos = {70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

		// Default ground surface (fairway)
		ground.height = 0.0F;
		ground.restitution = 0.4F;
		ground.frictionStatic = 0.5F;
		ground.firmness = 0.8F;

		// Create flat terrain from ground
		terrain = std::make_shared<FlatTerrain>(ground);
	}

	LaunchData ball;
	AtmosphericData atmos;
	GroundSurface ground;
	std::shared_ptr<TerrainInterface> terrain;
};

TEST_F(BouncePhaseTest, AppliesCoefficientOfRestitution)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Ball falling straight down at 30 ft/s
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.0F, 0.0F, -30.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	float dt = 0.01F;
	bounce.calculateStep(state, dt);

	// After bounce: vz = -0.4 * (-30) = 12 ft/s
	// Then gravity acts during dt: vz_final = 12 + (-g * dt) = 12 - 0.322 = 11.678
	float expectedVz = 12.0F + (-physics_constants::GRAVITY_FT_PER_S2 * dt);
	EXPECT_NEAR(state.velocity[2], expectedVz, 0.01F);
	EXPECT_GE(state.position[2], ground.height); // Should be at or above ground
}

TEST_F(BouncePhaseTest, AppliesFrictionToHorizontalVelocity)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Ball with horizontal and vertical velocity
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {20.0F, 10.0F, -30.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	bounce.calculateStep(state, 0.01F);

	// Expected friction factor: 1 - 0.5 * (1 - 0.8) = 0.9
	// vx: 20 * 0.9 = 18, vy: 10 * 0.9 = 9
	// Note: Small deviations due to aerodynamic forces during the timestep
	EXPECT_NEAR(state.velocity[0], 18.0F, 0.05F);
	EXPECT_NEAR(state.velocity[1], 9.0F, 0.05F);
}

TEST_F(BouncePhaseTest, EnergyRetentionMatchesCORSquared)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Ball falling vertically
	float impactVelocity = -30.0F;
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.0F, 0.0F, impactVelocity};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	float impactKE = 0.5F * impactVelocity * impactVelocity;

	bounce.calculateStep(state, 0.01F);

	float bounceKE = 0.5F * state.velocity[2] * state.velocity[2];
	float energyRetention = bounceKE / impactKE;
	float expectedRetention = ground.restitution * ground.restitution;

	EXPECT_NEAR(energyRetention, expectedRetention, 0.01F);
}

TEST_F(BouncePhaseTest, DifferentCORValues)
{
	// Test with high COR (hard surface)
	ground.restitution = 0.8F;
	terrain = std::make_shared<FlatTerrain>(ground);  // Recreate with updated ground

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.0F, 0.0F, -25.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	float dt = 0.01F;
	bounce.calculateStep(state, dt);

	// After bounce: vz = -0.8 * (-25) = 20 ft/s, then gravity/aero acts during dt
	float expectedVz = 20.0F + (-physics_constants::GRAVITY_FT_PER_S2 * dt);
	EXPECT_NEAR(state.velocity[2], expectedVz, 0.05F);
}

TEST_F(BouncePhaseTest, HandlesMultipleBounces)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Ball falling down
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {5.0F, 0.0F, -20.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	float dt = 0.01F;

	// First bounce
	bounce.calculateStep(state, dt);
	float vzAfterFirstBounce = state.velocity[2];
	EXPECT_GT(vzAfterFirstBounce, 0.0F); // Should be moving upward

	// Simulate until ball comes back down
	for (int i = 0; i < 50; ++i)
	{
		bounce.calculateStep(state, dt);
	}

	// Ball should have hit ground again (second bounce)
	// After multiple steps with gravity, ball should have bounced again
	EXPECT_GE(state.position[2], ground.height);

	// Phase should not be complete as ball still has some vertical velocity
	EXPECT_FALSE(bounce.isPhaseComplete(state));
}

TEST_F(BouncePhaseTest, TransitionsToRollPhaseWhenVelocityIsLow)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Ball on ground with low vertical velocity
	BallState state;
	state.position = {0.0F, 0.0F, 0.05F}; // Very close to ground
	state.velocity = {5.0F, 0.0F, 0.5F}; // Low vertical velocity
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.2F;

	EXPECT_TRUE(bounce.isPhaseComplete(state));
}

TEST_F(BouncePhaseTest, DoesNotCompleteWhileBallIsDescending)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Ball still falling
	BallState state;
	state.position = {0.0F, 0.0F, 5.0F};
	state.velocity = {10.0F, 0.0F, -10.0F}; // Still moving downward
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.1F;

	EXPECT_FALSE(bounce.isPhaseComplete(state));
}

TEST_F(BouncePhaseTest, DoesNotReApplyBounceWhileBallIsAirborne)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	// Initial impact
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.0F, 0.0F, -30.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	float dt = 0.01F;

	// First step - bounce should be applied
	bounce.calculateStep(state, dt);
	float expectedVz = 12.0F + (-physics_constants::GRAVITY_FT_PER_S2 * dt);
	EXPECT_NEAR(state.velocity[2], expectedVz, 0.01F);

	float velocityAfterFirstStep = state.velocity[2];

	// Second step - ball is airborne, bounce should NOT be applied
	bounce.calculateStep(state, dt);
	// Velocity should have decreased due to gravity, not bounced again
	EXPECT_LT(state.velocity[2], velocityAfterFirstStep);
}

TEST_F(BouncePhaseTest, HighFrictionSurface)
{
	// Soft surface with high friction
	ground.frictionStatic = 0.8F;
	ground.firmness = 0.3F;
	terrain = std::make_shared<FlatTerrain>(ground);  // Recreate with updated ground

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {20.0F, 0.0F, -30.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	bounce.calculateStep(state, 0.01F);

	// Expected friction factor: 1 - 0.8 * (1 - 0.3) = 1 - 0.56 = 0.44
	// vx: 20 * 0.44 = 8.8
	EXPECT_NEAR(state.velocity[0], 8.8F, 0.1F);
}

TEST_F(BouncePhaseTest, PreventsBallFromGoingBelowGround)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, -0.5F}; // Below ground (shouldn't happen but test safety)
	state.velocity = {0.0F, 0.0F, -10.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	bounce.calculateStep(state, 0.01F);

	EXPECT_GE(state.position[2], ground.height);
}

TEST_F(BouncePhaseTest, NonZeroGroundHeight)
{
	ground.height = 5.0F; // Ground at 5 feet
	terrain = std::make_shared<FlatTerrain>(ground);  // Recreate with updated ground

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	BouncePhase bounce(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 5.0F}; // At ground level
	state.velocity = {0.0F, 0.0F, -20.0F};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	float dt = 0.01F;
	bounce.calculateStep(state, dt);

	// Ball bounces and moves upward during the time step
	EXPECT_GE(state.position[2], ground.height); // Should be at or above ground

	// After bounce: vz = -0.4 * (-20) = 8 ft/s, then gravity acts during dt
	float expectedVz = 8.0F + (-physics_constants::GRAVITY_FT_PER_S2 * dt);
	EXPECT_NEAR(state.velocity[2], expectedVz, 0.01F);
}
