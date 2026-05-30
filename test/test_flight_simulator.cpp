/**
 * @file test_flight_simulator.cpp
 * @brief Unit tests for the FlightSimulator class
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "FlightSimulator.hpp"
#include "AerodynamicModel.hpp"
#include "terrain_interface.hpp"
#include "math_utils.hpp"
#include "physics_constants.hpp"

namespace
{
	// Constant downhill plane z = -slope * y. With slope (~45°) far steeper than
	// the surface's dynamic friction, gravity-along-slope always exceeds friction,
	// so the roll phase accelerates without bound and never reaches rest.
	class SteepDownhillTerrain : public TerrainInterface
	{
	public:
		explicit SteepDownhillTerrain(float slope) : slope_(slope)
		{
			surface_.height = 0.0F;
			surface_.restitution = 0.4F;
			surface_.frictionStatic = 0.1F;
			surface_.frictionDynamic = 0.1F;
			surface_.firmness = 0.8F;
		}

		[[nodiscard]] auto getHeight([[maybe_unused]] float x, float y) const -> float override
		{
			return -slope_ * y;
		}

		[[nodiscard]] auto getNormal([[maybe_unused]] float x, [[maybe_unused]] float y) const -> Vector3D override
		{
			// Plane z + slope*y = 0 → gradient (0, slope, 1), normalized.
			const float inv = 1.0F / std::sqrt(1.0F + slope_ * slope_);
			return {0.0F, slope_ * inv, inv};
		}

		[[nodiscard]] auto getSurfaceProperties([[maybe_unused]] float x, [[maybe_unused]] float y) const
			-> const GroundSurface & override
		{
			return surface_;
		}

	private:
		float slope_;
		GroundSurface surface_;
	};

	// Custom aero model that poisons the trajectory with NaN. `NaN <= terrainHeight`
	// is false, so AerialPhase::isPhaseComplete never trips — the loop would hang
	// without a convergence cap.
	class NanAerodynamicModel : public AerodynamicModel
	{
	public:
		[[nodiscard]] Vector3D computeAcceleration([[maybe_unused]] const AerodynamicState &s) const override
		{
			const float nan = std::numeric_limits<float>::quiet_NaN();
			return {nan, nan, nan};
		}

		[[nodiscard]] float computeSpinDecayTau([[maybe_unused]] const AerodynamicState &s) const override
		{
			return 1e6F;
		}
	};
}

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

	float initialSpin = math_utils::magnitude(trajectory.front().spinVector);
	float finalSpin   = math_utils::magnitude(trajectory.back().spinVector);

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

// xF/yF/distance/bearing are measured from the launch start, not the world
// origin. On flat ground with no wind the physics is
// translation-invariant, so offsetting the start must leave the start-relative
// landing result unchanged while the absolute resting position shifts by
// exactly the offset. The second half guards against "fix" by ignoring start.
TEST_F(FlightSimulatorTest, LandingResultIsRelativeToNonzeroStart)
{
	FlightSimulator atOrigin(ball, atmos, ground);
	atOrigin.run(0.01F);
	const LandingResult base = atOrigin.getLandingResult();

	LaunchData offset = ball;
	offset.startX = 30.0F;   // feet right of the target line
	offset.startY = 150.0F;  // feet downrange, e.g. an approach shot
	FlightSimulator shifted(offset, atmos, ground);
	shifted.run(0.01F);
	const LandingResult moved = shifted.getLandingResult();

	// Start-relative outputs are invariant to where the shot started.
	EXPECT_NEAR(moved.xF, base.xF, 0.01F);
	EXPECT_NEAR(moved.yF, base.yF, 0.01F);
	EXPECT_NEAR(moved.distance, base.distance, 0.01F);
	EXPECT_NEAR(moved.bearing, base.bearing, 0.01F);

	// The absolute resting position really shifted by the start offset, so the
	// invariance above is a subtraction, not a dropped start.
	const Vector3D &absPos = shifted.getState().position;
	EXPECT_NEAR(absPos[0] / physics_constants::YARDS_TO_FEET,
	            base.xF + offset.startX / physics_constants::YARDS_TO_FEET, 0.05F);
	EXPECT_NEAR(absPos[1] / physics_constants::YARDS_TO_FEET,
	            base.yF + offset.startY / physics_constants::YARDS_TO_FEET, 0.05F);
}

// --- Convergence guard ----------------------------------------------------

TEST_F(FlightSimulatorTest, SteepDownhillTerrainThrowsInsteadOfHanging)
{
	// 45° downhill: gravity-along-slope >> dynamic friction → roll never rests.
	auto terrain = std::make_shared<SteepDownhillTerrain>(1.0F);
	FlightSimulator sim(ball, atmos, terrain);

	EXPECT_THROW(sim.run(0.01F), std::runtime_error);
}

TEST_F(FlightSimulatorTest, NanModelThrowsInsteadOfHanging)
{
	// Custom model emitting NaN: aerial phase-complete check never trips.
	auto model = std::make_shared<NanAerodynamicModel>();
	FlightSimulator sim(ball, atmos, ground, model);

	EXPECT_THROW(sim.run(0.01F), std::runtime_error);
}

TEST_F(FlightSimulatorTest, TrajectoryVariantAlsoGuardsAgainstHang)
{
	auto model = std::make_shared<NanAerodynamicModel>();
	FlightSimulator sim(ball, atmos, ground, model);

	EXPECT_THROW(sim.runAndGetTrajectory(0.01F), std::runtime_error);
}

TEST_F(FlightSimulatorTest, NonPositiveDtThrows)
{
	FlightSimulator sim(ball, atmos, ground);

	// dt <= 0 never advances time → phase-complete conditions can never trip.
	EXPECT_THROW(sim.run(0.0F), std::invalid_argument);
	EXPECT_THROW(sim.run(-0.01F), std::invalid_argument);
}
