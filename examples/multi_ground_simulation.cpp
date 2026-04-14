/**
 * @file multi_ground_simulation.cpp
 * @brief Example demonstrating dynamic ground type changes during trajectory.
 *
 * This example shows how to use a custom TerrainInterface to model a golf hole
 * with different ground surfaces: fairway, rough, and an elevated green.
 *
 * The simulated hole layout:
 * - Fairway: 0-250 yards (standard properties)
 * - Rough: Lateral edges beyond ±20 yards from centerline (higher friction, softer)
 * - Green: 250-270 yards, elevated 3 feet (low friction, high spin retention)
 */

#include "FlightSimulator.hpp"
#include "terrain_interface.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdio.h>
#include <vector>

/**
 * @brief Custom terrain for a golf hole with fairway, rough, and an elevated green.
 *
 * Ground properties and height change based on ball XY position.
 * The green is elevated 3 feet; all other surfaces are at ground level.
 */
class GolfHoleTerrain : public TerrainInterface
{
public:
	GolfHoleTerrain() = default;

	float getHeight(float x, float y) const override
	{
		return getSurfaceProperties(x, y).height;
	}

	Vector3D getNormal([[maybe_unused]] float x, [[maybe_unused]] float y) const override
	{
		return {0.0F, 0.0F, 1.0F};
	}

	const GroundSurface &getSurfaceProperties(float x, float y) const override
	{
		const float lateralYards   = x / physics_constants::YARDS_TO_FEET;
		const float downrangeYards = y / physics_constants::YARDS_TO_FEET;

		// Green: 250-270 yards downrange, elevated 3 feet
		if (downrangeYards >= 250.0F && downrangeYards <= 270.0F)
		{
			static const GroundSurface kGreen{
				3.0F,   // height: elevated 3 feet
				0.35F,  // restitution: lower bounce on green
				0.4F,   // frictionStatic: moderate
				0.12F,  // frictionDynamic: low for fast greens
				0.95F,  // firmness: very firm
				0.85F   // spinRetention: high (soft greens retain spin)
			};
			return kGreen;
		}

		// Rough: beyond ±20 yards from centerline
		if (std::abs(lateralYards) > 20.0F)
		{
			static const GroundSurface kRough{
				0.0F,   // height: ground level
				0.25F,  // restitution: softer bounce
				0.6F,   // frictionStatic: higher impact friction
				0.5F,   // frictionDynamic: much higher rolling resistance
				0.4F,   // firmness: softer ground
				0.55F   // spinRetention: lower (thick grass kills spin)
			};
			return kRough;
		}

		// Fairway: default area (centerline, before green)
		static const GroundSurface kFairway{
			0.0F,   // height: ground level
			0.4F,   // restitution: standard bounce
			0.5F,   // frictionStatic: moderate
			0.2F,   // frictionDynamic: standard rolling resistance
			0.8F,   // firmness: firm fairway
			0.75F   // spinRetention: good spin retention
		};
		return kFairway;
	}
};

int main()
{
	// Shot parameters: 160 mph ball speed, 11° launch angle, straight shot
	const LaunchData ball{
		.ballSpeedMph   = 160.0F,
		.launchAngleDeg = 11.0F,
		.directionDeg   = 0.0F,
		.backspinRpm    = 3000.0F,
		.sidespinRpm    = 0.0F,
	};
	const AtmosphericData atmos{
		.temp        = 70.0F,
		.elevation   = 0.0F,
		.vWind       = 0.0F,
		.phiWind     = 0.0F,
		.hWind       = 0.0F,
		.relHumidity = 50.0F,
		.pressure    = 29.92F,
	};

	FlightSimulator sim(ball, atmos, std::make_shared<GolfHoleTerrain>());

	// Run simulation and collect trajectory points
	auto states = sim.runAndGetTrajectory();

	std::vector<Vector3D> trajectory;
	for (const auto &s : states)
		trajectory.push_back(s.position);

	// Print summary
	const BallState &finalState        = sim.getState();
	const float finalLateralYards      = finalState.position[0] / physics_constants::YARDS_TO_FEET;
	const float finalDownrangeYards    = finalState.position[1] / physics_constants::YARDS_TO_FEET;
	const float finalHeight            = finalState.position[2];

	printf("=== Multi-Ground Trajectory Simulation ===\n\n");

	printf("Shot Parameters:\n");
	printf("  Ball speed: %.1f mph\n", ball.ballSpeedMph);
	printf("  Launch angle: %.1f°\n", ball.launchAngleDeg);
	printf("  Backspin: %.1f rpm\n\n", ball.backspinRpm);

	printf("Final Landing:\n");
	printf("  Lateral: %.1f yards\n", finalLateralYards);
	printf("  Downrange: %.1f yards\n", finalDownrangeYards);
	printf("  Height: %.1f feet\n\n", finalHeight);

	// Determine final surface type
	const char *finalSurface;
	if (finalDownrangeYards >= 250.0F && finalDownrangeYards <= 270.0F)
		finalSurface = "Green (elevated)";
	else if (std::abs(finalLateralYards) > 20.0F)
		finalSurface = "Rough";
	else
		finalSurface = "Fairway";

	printf("Final Surface: %s\n\n", finalSurface);

	printf("Trajectory points (lateral, downrange, height in yards/feet):\n");
	printf("Lateral    Downrange  Height\n");
	printf("--------   ---------  ------\n");

	// Print every 10th point to keep output manageable
	for (size_t i = 0; i < trajectory.size(); i += 10)
	{
		const auto &pos = trajectory[i];
		printf("%8.1f   %9.1f  %6.1f\n",
		       pos[0] / physics_constants::YARDS_TO_FEET,
		       pos[1] / physics_constants::YARDS_TO_FEET,
		       pos[2]);
	}

	// Always print final point
	if (trajectory.size() % 10 != 1)
	{
		const auto &pos = trajectory.back();
		printf("%8.1f   %9.1f  %6.1f\n",
		       pos[0] / physics_constants::YARDS_TO_FEET,
		       pos[1] / physics_constants::YARDS_TO_FEET,
		       pos[2]);
	}

	printf("\n");
	printf("Note: The ground type changes automatically as the ball\n");
	printf("      moves across different surfaces during the trajectory.\n");

	return 0;
}
