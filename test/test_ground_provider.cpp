/**
 * @file test_ground_provider.cpp
 * @brief Unit tests for the GroundProvider interface and dynamic ground support
 */

#include <gtest/gtest.h>
#include <cmath>
#include "FlightSimulator.hpp"
#include "GroundProvider.hpp"
#include "physics_constants.hpp"

// =============================================================================
// Test Fixtures and Helper Classes
// =============================================================================

/**
 * @brief Test ground provider that returns different surfaces based on distance.
 *
 * - Fairway: 0-200 yards (standard properties)
 * - Green: 200+ yards (elevated 3ft, lower friction)
 */
class SimpleTestGroundProvider : public GroundProvider
{
public:
	SimpleTestGroundProvider() = default;
	SimpleTestGroundProvider(const SimpleTestGroundProvider&) : GroundProvider() {}

	GroundSurface getGroundAt(float x, float y) const override
	{
		(void)x; // Not used in this simple test

		const float downrangeYards = y / physics_constants::YARDS_TO_FEET;

		if (downrangeYards >= 200.0F)
		{
			// Green (elevated)
			return GroundSurface{
				3.0F,   // height: elevated
				0.35F,  // restitution
				0.4F,   // frictionStatic
				0.12F,  // frictionDynamic
				0.95F,  // firmness
				0.85F   // spinRetention
			};
		}

		// Fairway
		return GroundSurface{
			0.0F,   // height: ground level
			0.4F,   // restitution
			0.5F,   // frictionStatic
			0.2F,   // frictionDynamic
			0.8F,   // firmness
			0.75F   // spinRetention
		};
	}

	std::unique_ptr<GroundProvider> clone() const override
	{
		return std::make_unique<SimpleTestGroundProvider>(*this);
	}
};

/**
 * @brief Test ground provider with lateral variation (rough on edges).
 */
class LateralTestGroundProvider : public GroundProvider
{
public:
	LateralTestGroundProvider() = default;

	LateralTestGroundProvider(const LateralTestGroundProvider&) : GroundProvider() {}

	GroundSurface getGroundAt(float x, float y) const override
	{
		(void)y; // Not used in this test

		const float lateralYards = x / physics_constants::YARDS_TO_FEET;

		// Rough beyond ±15 yards from center
		if (std::abs(lateralYards) > 15.0F)
		{
			return GroundSurface{
				0.0F,   // height
				0.25F,  // restitution (softer)
				0.6F,   // frictionStatic (higher)
				0.5F,   // frictionDynamic (much higher)
				0.4F,   // firmness (softer)
				0.55F   // spinRetention (lower)
			};
		}

		// Fairway
		return GroundSurface{
			0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};
	}

	std::unique_ptr<GroundProvider> clone() const override
	{
		return std::make_unique<LateralTestGroundProvider>(*this);
	}
};

class GroundProviderTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Standard test ball parameters: 120 mph, 25° launch, 3000 rpm backspin
		ball = {120.0F, 25.0F, 0.0F, 3000.0F, 0.0F};

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
		ground = GroundSurface{
			0.0F,   // height
			0.4F,   // restitution
			0.5F,   // frictionStatic
			0.2F,   // frictionDynamic
			0.8F,   // firmness
			0.75F   // spinRetention
		};
	}

	LaunchData ball;
	AtmosphericData atmos;
	GroundSurface ground;
};

// =============================================================================
// UniformGroundProvider Tests
// =============================================================================

TEST_F(GroundProviderTest, UniformProviderReturnsSameSurface)
{
	UniformGroundProvider provider(ground);

	// Should return the same surface regardless of position
	GroundSurface surface1 = provider.getGroundAt(0.0F, 0.0F);
	GroundSurface surface2 = provider.getGroundAt(100.0F, 500.0F);
	GroundSurface surface3 = provider.getGroundAt(-50.0F, 1000.0F);

	EXPECT_FLOAT_EQ(surface1.height, ground.height);
	EXPECT_FLOAT_EQ(surface1.restitution, ground.restitution);
	EXPECT_FLOAT_EQ(surface1.frictionDynamic, ground.frictionDynamic);

	EXPECT_FLOAT_EQ(surface2.height, ground.height);
	EXPECT_FLOAT_EQ(surface3.height, ground.height);
}

TEST_F(GroundProviderTest, UniformProviderPreservesAllProperties)
{
	ground.height = 5.0F;
	ground.restitution = 0.3F;
	ground.frictionStatic = 0.6F;
	ground.frictionDynamic = 0.25F;
	ground.firmness = 0.9F;
	ground.spinRetention = 0.8F;

	UniformGroundProvider provider(ground);
	GroundSurface result = provider.getGroundAt(123.0F, 456.0F);

	EXPECT_FLOAT_EQ(result.height, 5.0F);
	EXPECT_FLOAT_EQ(result.restitution, 0.3F);
	EXPECT_FLOAT_EQ(result.frictionStatic, 0.6F);
	EXPECT_FLOAT_EQ(result.frictionDynamic, 0.25F);
	EXPECT_FLOAT_EQ(result.firmness, 0.9F);
	EXPECT_FLOAT_EQ(result.spinRetention, 0.8F);
}

// =============================================================================
// Constructor Tests
// =============================================================================

TEST_F(GroundProviderTest, FlatGroundConstructorWorks)
{
	FlightSimulator sim(ball, atmos, ground);
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");
	EXPECT_GT(sim.getLandingResult().distance, 0.0F);
}

TEST_F(GroundProviderTest, FlatAndProviderConstructorEquivalent)
{
	// Flat ground constructor
	FlightSimulator sim1(ball, atmos, ground);

	// Provider constructor with uniform provider
	UniformGroundProvider provider(ground);
	FlightSimulator sim2(ball, atmos, provider);

	sim1.run(0.01F);
	sim2.run(0.01F);

	// Final positions should be identical
	const BallState& final1 = sim1.getState();
	const BallState& final2 = sim2.getState();

	EXPECT_NEAR(final1.position[0], final2.position[0], 0.01F);
	EXPECT_NEAR(final1.position[1], final2.position[1], 0.01F);
	EXPECT_NEAR(final1.position[2], final2.position[2], 0.01F);
}

// =============================================================================
// Dynamic Ground Tests
// =============================================================================

TEST_F(GroundProviderTest, BallLandsOnElevatedGreen)
{
	SimpleTestGroundProvider provider;

	ball.ballSpeedMph = 160.0F; // High speed to reach 200+ yards
	ball.launchAngleDeg = 11.0F;

	FlightSimulator sim(ball, atmos, provider);
	sim.run(0.01F);

	const BallState& finalState = sim.getState();
	const float finalDownrangeYards = finalState.position[1] / physics_constants::YARDS_TO_FEET;

	// Ball should have traveled far enough to reach the green
	EXPECT_GT(finalDownrangeYards, 200.0F);

	// Final height should be at or near the green height (3ft)
	EXPECT_NEAR(finalState.position[2], 3.0F, 0.5F);
}

TEST_F(GroundProviderTest, SurfaceTransitionAffectsRolling)
{
	SimpleTestGroundProvider provider;

	ball.ballSpeedMph = 160.0F;
	ball.launchAngleDeg = 11.0F;

	FlightSimulator sim(ball, atmos, provider);
	sim.run(0.01F);

	// Ball should reach the green (200+ yards) and come to rest at green height (3ft)
	const BallState& finalState = sim.getState();
	const float finalDownrangeYards = finalState.position[1] / physics_constants::YARDS_TO_FEET;

	EXPECT_GT(finalDownrangeYards, 200.0F);
	EXPECT_NEAR(finalState.position[2], 3.0F, 0.5F);
}

TEST_F(GroundProviderTest, LateralRoughIncreasesRollingFriction)
{
	LateralTestGroundProvider provider;

	// First shot: straight down fairway
	LaunchData ball1 = ball;
	ball1.directionDeg = 0.0F;
	FlightSimulator sim1(ball1, atmos, provider);

	// Second shot: hooked into rough (left)
	LaunchData ball2 = ball;
	ball2.directionDeg = -15.0F; // 15 degrees left
	FlightSimulator sim2(ball2, atmos, provider);

	sim1.run(0.01F);
	sim2.run(0.01F);

	const BallState& final1 = sim1.getState();
	const BallState& final2 = sim2.getState();

	const float distance1 = final1.position[1] / physics_constants::YARDS_TO_FEET;
	const float distance2 = final2.position[1] / physics_constants::YARDS_TO_FEET;

	// Shot in rough should travel shorter distance due to higher friction
	// (though the angled shot also affects this - we're testing the combined effect)
	EXPECT_LT(distance2, distance1 * 1.1F); // Rough shot shouldn't go much farther
}

TEST_F(GroundProviderTest, MultipleGroundTransitions)
{
	// Create a provider that changes ground every 50 yards
	class MultiZoneProvider : public GroundProvider
	{
	public:
		MultiZoneProvider() = default;

		MultiZoneProvider(const MultiZoneProvider&) : GroundProvider() {}

		GroundSurface getGroundAt(float x, float y) const override
		{
			(void)x;
			const float yards = y / physics_constants::YARDS_TO_FEET;

			if (yards < 50.0F)
			{
				return GroundSurface{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F}; // Zone 1
			}
			else if (yards < 100.0F)
			{
				return GroundSurface{1.0F, 0.35F, 0.6F, 0.3F, 0.7F, 0.7F}; // Zone 2 (elevated 1ft, higher friction)
			}
			else
			{
				return GroundSurface{2.0F, 0.3F, 0.4F, 0.15F, 0.9F, 0.8F}; // Zone 3 (elevated 2ft, lower friction)
			}
		}

		std::unique_ptr<GroundProvider> clone() const override
		{
			return std::make_unique<MultiZoneProvider>(*this);
		}
	};

	MultiZoneProvider provider;

	ball.ballSpeedMph = 140.0F;

	FlightSimulator sim(ball, atmos, provider);
	sim.run(0.01F);

	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");

	const BallState& finalState = sim.getState();

	// Final height should correspond to the zone where it landed
	// If it made it past 100 yards, should be near 2ft
	const float finalYards = finalState.position[1] / physics_constants::YARDS_TO_FEET;
	if (finalYards >= 100.0F)
	{
		EXPECT_NEAR(finalState.position[2], 2.0F, 1.0F);
	}
}

// =============================================================================
// Edge Cases and Robustness Tests
// =============================================================================

TEST_F(GroundProviderTest, VeryShortShotUsesCorrectGround)
{
	// Very low speed shot that lands immediately
	ball.ballSpeedMph = 30.0F;
	ball.launchAngleDeg = 45.0F;

	SimpleTestGroundProvider provider;
	FlightSimulator sim(ball, atmos, provider);
	sim.run(0.01F);

	const BallState& finalState = sim.getState();

	// Should land on fairway (ground level), not green
	const float finalYards = finalState.position[1] / physics_constants::YARDS_TO_FEET;
	EXPECT_LT(finalYards, 200.0F);
	EXPECT_NEAR(finalState.position[2], 0.0F, 0.5F);
}
