/**
 * @file test_flight_simulator.cpp
 * @brief Unit tests for the FlightSimulator class
 */

#include <gtest/gtest.h>
#include <cmath>
#include "FlightSimulator.hpp"
#include "physics_constants.hpp"

class FlightSimulatorTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Standard test ball parameters: 100 mph, 30° launch, 3000 rpm backspin
		ball = {100.0F, 30.0F, 0.0F, 3000.0F, 0.0F};

		// Standard atmospheric conditions
		atmos = {
			70.0F,  // temperature (F)
			0.0F,   // wind x
			0.0F,   // wind y
			0.0F,   // wind z
			0.0F,   // wind direction
			50.0F,  // humidity
			29.92F  // pressure
		};

		// Standard ground conditions
		ground.height = 0.0F;
		ground.restitution = 0.4F;
		ground.frictionStatic = 0.5F;
		ground.frictionDynamic = 0.2F;
		ground.firmness = 0.8F;
	}

	LaunchData ball;
	AtmosphericData atmos;
	GroundSurface ground;
};

TEST_F(FlightSimulatorTest, InitializesCorrectly)
{
	FlightSimulator sim(ball, atmos, ground);
	EXPECT_STREQ(sim.getCurrentPhaseName(), "aerial");
}

TEST_F(FlightSimulatorTest, RunsToCompletion)
{
	FlightSimulator sim(ball, atmos, ground);
	sim.run(0.01F);
	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");
}

TEST_F(FlightSimulatorTest, TransitionsThroughAllPhases)
{
	FlightSimulator sim(ball, atmos, ground);

	// Starts in aerial
	EXPECT_STREQ(sim.getCurrentPhaseName(), "aerial");

	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");

	const BallState& finalState = sim.getState();

	// Ball ends on the ground (implies bounce and roll phases occurred)
	EXPECT_NEAR(finalState.position[2], ground.height, 0.1F);

	// Ball moved forward (aerial phase occurred)
	EXPECT_GT(finalState.position[1], 10.0F);
}

TEST_F(FlightSimulatorTest, ProducesReasonableTrajectory)
{
	FlightSimulator sim(ball, atmos, ground);
	auto trajectory = sim.runAndGetTrajectory(0.01F);

	ASSERT_GT(trajectory.size(), 1U);

	float maxHeight = 0.0F;
	for (const auto& s : trajectory)
		maxHeight = std::max(maxHeight, s.position[2]);

	const BallState& finalState = trajectory.back();
	float finalDistance = std::sqrt(
		finalState.position[0] * finalState.position[0] +
		finalState.position[1] * finalState.position[1]
	);

	// Sanity checks for 100 mph, 30 degree shot
	EXPECT_GT(maxHeight, 10.0F) << "Ball should reach reasonable max height";
	EXPECT_LT(maxHeight, 120.0F) << "Max height should be realistic";
	EXPECT_GT(finalDistance, 100.0F) << "Ball should travel reasonable distance";
	EXPECT_LT(finalDistance, 1000.0F) << "Distance should be realistic";
	EXPECT_NEAR(finalState.position[2], ground.height, 0.1F) << "Ball should end on ground";
}

TEST_F(FlightSimulatorTest, RunIsIdempotent)
{
	FlightSimulator sim(ball, atmos, ground);
	sim.run(0.01F);

	BallState finalState = sim.getState();

	// Calling run() again on a completed simulation should be a no-op
	sim.run(0.01F);

	const BallState& afterState = sim.getState();
	EXPECT_EQ(finalState.position[0], afterState.position[0]);
	EXPECT_EQ(finalState.position[1], afterState.position[1]);
	EXPECT_EQ(finalState.position[2], afterState.position[2]);
	EXPECT_EQ(finalState.currentTime, afterState.currentTime);
}

TEST_F(FlightSimulatorTest, HandlesDifferentGroundConditions)
{
	// Test with very bouncy ground
	GroundSurface bouncyGround;
	bouncyGround.height = 0.0F;
	bouncyGround.restitution = 0.8F;
	bouncyGround.frictionStatic = 0.2F;
	bouncyGround.frictionDynamic = 0.1F;
	bouncyGround.firmness = 1.0F;

	FlightSimulator sim(ball, atmos, bouncyGround);
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");
}

TEST_F(FlightSimulatorTest, HandlesNonZeroGroundHeight)
{
	GroundSurface elevatedGround;
	elevatedGround.height = 10.0F;
	elevatedGround.restitution = 0.4F;
	elevatedGround.frictionStatic = 0.5F;
	elevatedGround.frictionDynamic = 0.2F;
	elevatedGround.firmness = 0.8F;

	// Start ball at the elevated ground height
	LaunchData elevatedBall = ball;
	elevatedBall.startZ = elevatedGround.height;

	FlightSimulator sim(elevatedBall, atmos, elevatedGround);
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");

	const BallState& finalState = sim.getState();
	EXPECT_NEAR(finalState.position[2], elevatedGround.height, 0.1F)
		<< "Ball should end at ground height";

	float finalSpeed = std::sqrt(
		finalState.velocity[0] * finalState.velocity[0] +
		finalState.velocity[1] * finalState.velocity[1] +
		finalState.velocity[2] * finalState.velocity[2]
	);
	EXPECT_LT(finalSpeed, 1.0F) << "Ball should be stopped or nearly stopped";
}

TEST_F(FlightSimulatorTest, SpinDecaysAcrossAllPhases)
{
	FlightSimulator sim(ball, atmos, ground);
	auto trajectory = sim.runAndGetTrajectory(0.01F);

	ASSERT_GT(trajectory.size(), 1U);

	float initialSpin = trajectory.front().spinRate;
	float finalSpin = trajectory.back().spinRate;

	EXPECT_GT(initialSpin, 0.0F) << "Should start with non-zero spin";
	EXPECT_LE(finalSpin, initialSpin) << "Final spin should not exceed initial spin";
}

TEST_F(FlightSimulatorTest, GetLandingResultReturnsYards)
{
	FlightSimulator sim(ball, atmos, ground);
	sim.run(0.01F);

	LandingResult result = sim.getLandingResult();
	const BallState& state = sim.getState();

	EXPECT_NEAR(result.xF, state.position[0] / physics_constants::YARDS_TO_FEET, 0.01F);
	EXPECT_NEAR(result.yF, state.position[1] / physics_constants::YARDS_TO_FEET, 0.01F);
	EXPECT_GT(result.distance, 0.0F);
}
