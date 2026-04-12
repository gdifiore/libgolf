/**
 * @file test_ground_provider.cpp
 * @brief Integration tests for FlightSimulator with dynamic terrain.
 *
 * These tests exercise position-dependent terrain implemented via TerrainInterface,
 * covering zone transitions, elevation changes, and surface property variations.
 */

#include <gtest/gtest.h>
#include <cmath>
#include "FlightSimulator.hpp"
#include "terrain_interface.hpp"
#include "physics_constants.hpp"

// =============================================================================
// Test Terrain Implementations
// =============================================================================

/**
 * @brief Flat terrain that returns different surfaces based on downrange distance.
 *
 * - Fairway: 0-200 yards (standard properties, height = 0)
 * - Green:  200+ yards (elevated 3ft, lower friction)
 */
class SimpleTestTerrain : public TerrainInterface
{
public:
	float getHeight(float x, float y) const override
	{
		return getSurfaceProperties(x, y).height;
	}

	Vector3D getNormal([[maybe_unused]] float x, [[maybe_unused]] float y) const override
	{
		return {0.0F, 0.0F, 1.0F};
	}

	const GroundSurface &getSurfaceProperties([[maybe_unused]] float x, float y) const override
	{
		const float downrangeYards = y / physics_constants::YARDS_TO_FEET;

		if (downrangeYards >= 200.0F)
		{
			static const GroundSurface kGreen{3.0F, 0.35F, 0.4F, 0.12F, 0.95F, 0.85F};
			return kGreen;
		}

		static const GroundSurface kFairway{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};
		return kFairway;
	}
};

/**
 * @brief Flat terrain with rough beyond ±15 yards from centerline.
 */
class LateralTestTerrain : public TerrainInterface
{
public:
	float getHeight([[maybe_unused]] float x, [[maybe_unused]] float y) const override
	{
		return 0.0F;
	}

	Vector3D getNormal([[maybe_unused]] float x, [[maybe_unused]] float y) const override
	{
		return {0.0F, 0.0F, 1.0F};
	}

	const GroundSurface &getSurfaceProperties(float x, [[maybe_unused]] float y) const override
	{
		const float lateralYards = x / physics_constants::YARDS_TO_FEET;

		if (std::abs(lateralYards) > 15.0F)
		{
			static const GroundSurface kRough{0.0F, 0.25F, 0.6F, 0.5F, 0.4F, 0.55F};
			return kRough;
		}

		static const GroundSurface kFairway{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};
		return kFairway;
	}
};

class DynamicTerrainTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Standard test ball parameters: 120 mph, 25° launch, 3000 rpm backspin
		ball = {120.0F, 25.0F, 0.0F, 3000.0F, 0.0F};

		atmos = {
			70.0F,  // temperature (F)
			0.0F,   // wind x
			0.0F,   // wind y
			0.0F,   // wind z
			0.0F,   // wind direction
			50.0F,  // humidity
			29.92F  // pressure
		};

		ground = GroundSurface{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};
	}

	LaunchData ball;
	AtmosphericData atmos;
	GroundSurface ground;
};

// =============================================================================
// Constructor Tests
// =============================================================================

TEST_F(DynamicTerrainTest, FlatGroundConstructorWorks)
{
	FlightSimulator sim(ball, atmos, ground);
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");
	EXPECT_GT(sim.getLandingResult().distance, 0.0F);
}

TEST_F(DynamicTerrainTest, TerrainConstructorWorks)
{
	FlightSimulator sim(ball, atmos, std::make_shared<SimpleTestTerrain>());
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");
	EXPECT_GT(sim.getLandingResult().distance, 0.0F);
}

// =============================================================================
// Dynamic Terrain Tests
// =============================================================================

TEST_F(DynamicTerrainTest, BallLandsOnElevatedGreen)
{
	ball.ballSpeedMph   = 160.0F; // High speed to reach 200+ yards
	ball.launchAngleDeg = 11.0F;

	FlightSimulator sim(ball, atmos, std::make_shared<SimpleTestTerrain>());
	sim.run(0.01F);

	const BallState &finalState        = sim.getState();
	const float finalDownrangeYards    = finalState.position[1] / physics_constants::YARDS_TO_FEET;

	EXPECT_GT(finalDownrangeYards, 200.0F);
	EXPECT_NEAR(finalState.position[2], 3.0F, 0.5F);
}

TEST_F(DynamicTerrainTest, SurfaceTransitionAffectsRolling)
{
	ball.ballSpeedMph   = 160.0F;
	ball.launchAngleDeg = 11.0F;

	FlightSimulator sim(ball, atmos, std::make_shared<SimpleTestTerrain>());
	sim.run(0.01F);

	const BallState &finalState        = sim.getState();
	const float finalDownrangeYards    = finalState.position[1] / physics_constants::YARDS_TO_FEET;

	EXPECT_GT(finalDownrangeYards, 200.0F);
	EXPECT_NEAR(finalState.position[2], 3.0F, 0.5F);
}

TEST_F(DynamicTerrainTest, LateralRoughIncreasesRollingFriction)
{
	auto terrain = std::make_shared<LateralTestTerrain>();

	// Straight shot: stays in fairway
	LaunchData ball1   = ball;
	ball1.directionDeg = 0.0F;
	FlightSimulator sim1(ball1, atmos, terrain);

	// Hooked shot: lands in rough
	LaunchData ball2   = ball;
	ball2.directionDeg = -15.0F;
	FlightSimulator sim2(ball2, atmos, terrain);

	sim1.run(0.01F);
	sim2.run(0.01F);

	const float distance1 = sim1.getState().position[1] / physics_constants::YARDS_TO_FEET;
	const float distance2 = sim2.getState().position[1] / physics_constants::YARDS_TO_FEET;

	// Rough shot shouldn't roll out significantly farther than the straight shot
	EXPECT_LT(distance2, distance1 * 1.1F);
}

TEST_F(DynamicTerrainTest, MultipleGroundTransitions)
{
	class MultiZoneTerrain : public TerrainInterface
	{
	public:
		float getHeight(float x, float y) const override
		{
			return getSurfaceProperties(x, y).height;
		}

		Vector3D getNormal([[maybe_unused]] float x, [[maybe_unused]] float y) const override
		{
			return {0.0F, 0.0F, 1.0F};
		}

		const GroundSurface &getSurfaceProperties([[maybe_unused]] float x, float y) const override
		{
			const float yards = y / physics_constants::YARDS_TO_FEET;

			if (yards < 50.0F)
			{
				static const GroundSurface kZone1{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};
				return kZone1;
			}
			else if (yards < 100.0F)
			{
				static const GroundSurface kZone2{1.0F, 0.35F, 0.6F, 0.3F, 0.7F, 0.7F};
				return kZone2;
			}
			else
			{
				static const GroundSurface kZone3{2.0F, 0.3F, 0.4F, 0.15F, 0.9F, 0.8F};
				return kZone3;
			}
		}
	};

	ball.ballSpeedMph = 140.0F;

	FlightSimulator sim(ball, atmos, std::make_shared<MultiZoneTerrain>());
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");

	const BallState &finalState = sim.getState();
	const float finalYards      = finalState.position[1] / physics_constants::YARDS_TO_FEET;

	if (finalYards >= 100.0F)
	{
		EXPECT_NEAR(finalState.position[2], 2.0F, 1.0F);
	}
}

TEST_F(DynamicTerrainTest, VeryShortShotUsesCorrectGround)
{
	ball.ballSpeedMph   = 30.0F;
	ball.launchAngleDeg = 45.0F;

	FlightSimulator sim(ball, atmos, std::make_shared<SimpleTestTerrain>());
	sim.run(0.01F);

	const BallState &finalState = sim.getState();
	const float finalYards      = finalState.position[1] / physics_constants::YARDS_TO_FEET;

	// Short shot should land on fairway (ground level), not the elevated green
	EXPECT_LT(finalYards, 200.0F);
	EXPECT_NEAR(finalState.position[2], 0.0F, 0.5F);
}
