/**
 * @file FlightSimulator.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of the FlightSimulator class for automated phase management.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "FlightSimulator.hpp"
#include "math_utils.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

FlightSimulator::FlightSimulator(
	const LaunchData &launch,
	const AtmosphericData &atmos,
	const GroundSurface &ground,
	std::shared_ptr<AerodynamicModel> aeroModel,
	std::shared_ptr<BounceModel> bounceModel,
	std::shared_ptr<RollModel> rollModel,
	const BallProperties &ball,
	float gravity,
	std::shared_ptr<Integrator> integrator)
	: currentPhase(Phase::Aerial),
	  gravity_(gravity),
	  physicsVars_(launch, atmos, ball),
	  terrainStorage_(std::make_shared<FlatTerrain>(ground)),
	  aerialPhase(physicsVars_, launch, atmos, terrainStorage_, aeroModel, ball, gravity, integrator),
	  bouncePhase(physicsVars_, launch, atmos, terrainStorage_, aeroModel, bounceModel, ball, gravity, integrator),
	  rollPhase(terrainStorage_, rollModel, ball)
{
	initializeFromLaunch(launch);
}

FlightSimulator::FlightSimulator(
	const LaunchData &launch,
	const AtmosphericData &atmos,
	std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<AerodynamicModel> aeroModel,
	std::shared_ptr<BounceModel> bounceModel,
	std::shared_ptr<RollModel> rollModel,
	const BallProperties &ball,
	float gravity,
	std::shared_ptr<Integrator> integrator)
	: currentPhase(Phase::Aerial),
	  gravity_(gravity),
	  physicsVars_(launch, atmos, ball),
	  terrainStorage_(terrain),
	  aerialPhase(physicsVars_, launch, atmos, terrainStorage_, aeroModel, ball, gravity, integrator),
	  bouncePhase(physicsVars_, launch, atmos, terrainStorage_, aeroModel, bounceModel, ball, gravity, integrator),
	  rollPhase(terrainStorage_, rollModel, ball)
{
	initializeFromLaunch(launch);
}

void FlightSimulator::initializeFromLaunch(const LaunchData &launch)
{
	const float v0_fps = launch.ballSpeedMph * physics_constants::MPH_TO_FT_PER_S;
	startPosition_ = Vector3D{launch.startX, launch.startY, launch.startZ};
	const Vector3D &startPos = startPosition_;

	state = BallState::fromLaunchParameters(
		v0_fps,
		launch.launchAngleDeg,
		launch.directionDeg,
		startPos,
		gravity_,
		physicsVars_.getW());

	aerialPhase.initialize(state);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

namespace
{
	// Largest step count the loop may take before declaring non-convergence.
	// dt must be positive: a non-positive dt never advances time, so the
	// phase-complete conditions can never trip.
	long convergenceStepCap(float dt)
	{
		if (!(dt > 0.0F))
		{
			throw std::invalid_argument("FlightSimulator: dt must be positive");
		}
		return static_cast<long>(physics_constants::MAX_SIMULATION_TIME / dt);
	}
}

void FlightSimulator::run(float dt)
{
	const long maxSteps = convergenceStepCap(dt);

	for (long step = 0; currentPhase != Phase::Complete; ++step)
	{
		if (step >= maxSteps)
		{
			throw std::runtime_error(
				std::string("FlightSimulator::run did not converge within ") +
				std::to_string(physics_constants::MAX_SIMULATION_TIME) +
				"s (stuck in " + getCurrentPhaseName() +
				" phase) — check terrain slope vs. friction or custom-model output for NaN");
		}
		stepOnce(dt);
	}
}

std::vector<BallState> FlightSimulator::runAndGetTrajectory(float dt)
{
	const long maxSteps = convergenceStepCap(dt);

	std::vector<BallState> trajectory;
	trajectory.reserve(static_cast<size_t>(10.0F / dt)); // ~10s at dt typical shots

	for (long step = 0; currentPhase != Phase::Complete; ++step)
	{
		if (step >= maxSteps)
		{
			throw std::runtime_error(
				std::string("FlightSimulator::runAndGetTrajectory did not converge within ") +
				std::to_string(physics_constants::MAX_SIMULATION_TIME) +
				"s (stuck in " + getCurrentPhaseName() +
				" phase) — check terrain slope vs. friction or custom-model output for NaN");
		}
		trajectory.push_back(state);
		stepOnce(dt);
	}

	trajectory.push_back(state); // final resting state

	return trajectory;
}

const BallState &FlightSimulator::getState() const
{
	return state;
}

LandingResult FlightSimulator::getLandingResult() const
{
	// Measure relative to the launch start, not the origin
	const Vector3D relative = state.position - startPosition_;

	LandingResult result;
	result.xF = relative[0] / physics_constants::YARDS_TO_FEET;
	result.yF = relative[1] / physics_constants::YARDS_TO_FEET;
	result.zF = relative[2] / physics_constants::YARDS_TO_FEET;
	result.timeOfFlight = state.currentTime;
	result.bearing = std::atan2(relative[0], relative[1]) *
	                 180.0F / physics_constants::PI;
	result.distance = math_utils::getDistanceInYards(relative);

	return result;
}

const ShotPhysicsContext &FlightSimulator::getPhysicsVariables() const
{
	return physicsVars_;
}

const char *FlightSimulator::getCurrentPhaseName() const
{
	switch (currentPhase)
	{
	case Phase::Aerial:   return "aerial";
	case Phase::Bounce:   return "bounce";
	case Phase::Roll:     return "roll";
	case Phase::Complete: return "complete";
	default:              return "unknown";
	}
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void FlightSimulator::stepOnce(float dt)
{
	switch (currentPhase)
	{
	case Phase::Aerial:
		aerialPhase.calculateStep(state, dt);
		break;
	case Phase::Bounce:
		bouncePhase.calculateStep(state, dt);
		break;
	case Phase::Roll:
		rollPhase.calculateStep(state, dt);
		break;
	case Phase::Complete:
		break;
	}

	checkPhaseTransition();
}

void FlightSimulator::checkPhaseTransition()
{
	switch (currentPhase)
	{
	case Phase::Aerial:
		if (aerialPhase.isPhaseComplete(state))
		{
			currentPhase = Phase::Bounce;
		}
		break;

	case Phase::Bounce:
		if (bouncePhase.isPhaseComplete(state))
		{
			currentPhase = Phase::Roll;
		}
		break;

	case Phase::Roll:
		if (rollPhase.isPhaseComplete(state))
		{
			currentPhase = Phase::Complete;
		}
		break;

	case Phase::Complete:
		break;
	}
}
