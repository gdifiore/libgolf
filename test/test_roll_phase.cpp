#include <gtest/gtest.h>
#include <cmath>

#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "ground_surface.hpp"
#include "terrain_interface.hpp"
#include "physics_constants.hpp"

#include <memory>

class RollPhaseTest : public ::testing::Test
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

TEST_F(RollPhaseTest, DeceleratesFromRollingFriction)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	// Ball rolling on ground at 10 ft/s
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {10.0F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	float initialSpeed = state.velocity[0];
	float dt = 0.1F;

	roll.calculateStep(state, dt);

	// Velocity should have decreased
	EXPECT_LT(state.velocity[0], initialSpeed);
	EXPECT_GT(state.velocity[0], 0.0F); // But not stopped yet
}

TEST_F(RollPhaseTest, KeepsBallOnGround)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 5.0F}; // Start above ground
	state.velocity = {10.0F, 0.0F, 5.0F}; // Some vertical velocity
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	roll.calculateStep(state, 0.01F);

	// Ball should be clamped to ground
	EXPECT_EQ(state.position[2], ground.height);
	EXPECT_EQ(state.velocity[2], 0.0F);
}

TEST_F(RollPhaseTest, UpdatesPositionBasedOnVelocity)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {10.0F, 5.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	float dt = 0.1F;
	roll.calculateStep(state, dt);

	// Position should have advanced (accounting for deceleration)
	EXPECT_GT(state.position[0], 0.0F);
	EXPECT_GT(state.position[1], 0.0F);
	EXPECT_NEAR(state.position[0] / state.position[1], 10.0F / 5.0F, 0.1F); // Direction preserved
}

TEST_F(RollPhaseTest, PreservesDirectionWhileSlowing)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	// Ball rolling at 45 degrees
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {10.0F, 10.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	float initialAngle = atan2(state.velocity[1], state.velocity[0]);

	roll.calculateStep(state, 0.1F);

	float finalAngle = atan2(state.velocity[1], state.velocity[0]);

	// Direction should be preserved
	EXPECT_NEAR(initialAngle, finalAngle, 0.01F);
}

TEST_F(RollPhaseTest, StopsWhenVelocityTooLow)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	// Ball rolling very slowly
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.5F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	// Roll for a while
	for (int i = 0; i < 100; ++i)
	{
		roll.calculateStep(state, 0.01F);
		if (roll.isPhaseComplete(state))
		{
			break;
		}
	}

	// Ball should have stopped
	EXPECT_TRUE(roll.isPhaseComplete(state));

	// Velocity should be very small
	float finalSpeed = sqrt(state.velocity[0] * state.velocity[0] +
	                        state.velocity[1] * state.velocity[1]);
	EXPECT_LT(finalSpeed, 0.1F);
}

TEST_F(RollPhaseTest, DoesNotReverseDirection)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	// Ball rolling slowly forward
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.2F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	// Large timestep that would reverse velocity if not clamped
	roll.calculateStep(state, 1.0F);

	// Velocity should be zero or positive, never negative
	EXPECT_GE(state.velocity[0], 0.0F);
}

TEST_F(RollPhaseTest, HigherFrictionSlowsFaster)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);

	// Low friction surface (fairway)
	ground.frictionDynamic = 0.15F;
	auto terrainLow = std::make_shared<FlatTerrain>(ground);
	RollPhase rollLow(physicsVars, ball, atmos, terrainLow);

	// High friction surface (rough)
	ground.frictionDynamic = 0.5F;
	auto terrainHigh = std::make_shared<FlatTerrain>(ground);
	RollPhase rollHigh(physicsVars, ball, atmos, terrainHigh);

	// Same initial state
	BallState stateLow, stateHigh;
	stateLow.position = {0.0F, 0.0F, 0.0F};
	stateLow.velocity = {20.0F, 0.0F, 0.0F};
	stateLow.acceleration = {0.0F, 0.0F, 0.0F};
	stateLow.currentTime = 0.0F;

	stateHigh = stateLow;

	float dt = 0.1F;
	rollLow.calculateStep(stateLow, dt);
	rollHigh.calculateStep(stateHigh, dt);

	// High friction should decelerate more
	EXPECT_LT(stateHigh.velocity[0], stateLow.velocity[0]);
}

TEST_F(RollPhaseTest, NonZeroGroundHeight)
{
	ground.height = 10.0F;
	terrain = std::make_shared<FlatTerrain>(ground);  // Recreate with updated ground

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 0.0F}; // Start at z=0
	state.velocity = {10.0F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	roll.calculateStep(state, 0.01F);

	// Ball should be at ground height
	EXPECT_EQ(state.position[2], ground.height);
}

TEST_F(RollPhaseTest, EventuallyStops)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {15.0F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	bool stopped = false;
	int maxSteps = 10000;

	for (int i = 0; i < maxSteps; ++i)
	{
		roll.calculateStep(state, 0.01F);
		if (roll.isPhaseComplete(state))
		{
			stopped = true;
			break;
		}
	}

	EXPECT_TRUE(stopped);
	EXPECT_LT(state.currentTime, 100.0F); // Should stop in reasonable time
}

TEST_F(RollPhaseTest, HandlesNegativeSpinRate)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	// Ball rolling with backspin (negative spin rate)
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {10.0F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.spinRate = -500.0F;  // Backspin
	state.currentTime = 0.0F;

	// Roll for several steps
	for (int i = 0; i < 10; ++i)
	{
		roll.calculateStep(state, 0.01F);
	}

	// Spin should decay toward zero but remain negative
	EXPECT_LT(state.spinRate, 0.0F);  // Still backspin
	EXPECT_GT(state.spinRate, -500.0F);  // But reduced in magnitude
}

TEST_F(RollPhaseTest, SpinDecaysToZeroFromNegative)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, terrain);

	// Ball rolling slowly with small backspin
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {5.0F, 0.0F, 0.0F};
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.spinRate = -5.0F;  // Small backspin
	state.currentTime = 0.0F;

	float initialSpinMag = std::abs(state.spinRate);

	// Roll for several steps
	for (int i = 0; i < 100; ++i)
	{
		roll.calculateStep(state, 0.01F);
	}

	// Spin magnitude should have decreased
	EXPECT_LT(std::abs(state.spinRate), initialSpinMag);

	// If spin hasn't reached zero yet, it should still be negative (backspin)
	if (state.spinRate != 0.0F)
	{
		EXPECT_LT(state.spinRate, 0.0F);
	}
}

TEST_F(RollPhaseTest, BallCanStartRollingFromNearZeroVelocityOnSlope)
{
	// Create sloped terrain (10 degree downslope)
	ground.frictionDynamic = 0.15F;

	// Create a simple sloped terrain for testing
	class TestSlopedTerrain : public TerrainInterface
	{
	private:
		GroundSurface surface_;
		Vector3D normal_;
	public:
		TestSlopedTerrain(const GroundSurface& surface) : surface_(surface)
		{
			float angle = 10.0F * physics_constants::DEG_TO_RAD;
			normal_ = {0.0F, std::sin(angle), std::cos(angle)};
		}

		float getHeight(float x, float y) const override {
			(void)x; (void)y;
			return 0.0F;
		}

		Vector3D getNormal(float x, float y) const override {
			(void)x; (void)y;
			return normal_;
		}

		const GroundSurface& getSurfaceProperties(float x, float y) const override {
			(void)x; (void)y;
			return surface_;
		}
	};

	auto slopedTerrain = std::make_shared<TestSlopedTerrain>(ground);

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	RollPhase roll(physicsVars, ball, atmos, slopedTerrain);

	// Ball starting with very small velocity (just above MIN_VELOCITY_THRESHOLD)
	// This tests that the velocity reversal fix allows the ball to accelerate
	// from near-zero velocity without getting stuck
	BallState state;
	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {0.0F, 0.02F, 0.0F};  // Small velocity above MIN_VELOCITY_THRESHOLD
	state.acceleration = {0.0F, 0.0F, 0.0F};
	state.currentTime = 0.0F;

	float initialVelocity = state.velocity[1];

	// Roll for several steps
	for (int i = 0; i < 20; ++i)
	{
		roll.calculateStep(state, 0.01F);
	}

	// Ball should accelerate downslope (gravity > friction on 10° slope)
	// Without the fix, the ball could get stuck at zero if velocity reversed
	EXPECT_GT(state.velocity[1], initialVelocity);
}
