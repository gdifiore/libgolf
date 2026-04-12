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

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

FlightSimulator::FlightSimulator(
	const LaunchData &launch,
	const AtmosphericData &atmos,
	const GroundSurface &ground,
	std::shared_ptr<AerodynamicModel> model)
	: currentPhase(Phase::Aerial),
	  physicsVars_(launch, atmos),
	  terrainStorage_(std::make_shared<FlatTerrain>(ground)),
	  aerialPhase(physicsVars_, launch, atmos, terrainStorage_, model),
	  bouncePhase(physicsVars_, launch, atmos, terrainStorage_, std::move(model)),
	  rollPhase(physicsVars_, launch, atmos, terrainStorage_)
{
	initializeFromLaunch(launch);
}

FlightSimulator::FlightSimulator(
	const LaunchData &launch,
	const AtmosphericData &atmos,
	const GroundProvider &groundProvider,
	std::shared_ptr<AerodynamicModel> model)
	: currentPhase(Phase::Aerial),
	  physicsVars_(launch, atmos),
	  terrainStorage_(std::make_shared<TerrainProviderAdapter>(groundProvider)),
	  aerialPhase(physicsVars_, launch, atmos, terrainStorage_, model),
	  bouncePhase(physicsVars_, launch, atmos, terrainStorage_, std::move(model)),
	  rollPhase(physicsVars_, launch, atmos, terrainStorage_)
{
	initializeFromLaunch(launch);
}

FlightSimulator::FlightSimulator(
	const LaunchData &launch,
	const AtmosphericData &atmos,
	const GroundSurface &ground,
	std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<AerodynamicModel> model)
	: currentPhase(Phase::Aerial),
	  physicsVars_(launch, atmos),
	  terrainStorage_(terrain ? terrain : std::make_shared<FlatTerrain>(ground)),
	  aerialPhase(physicsVars_, launch, atmos, terrainStorage_, model),
	  bouncePhase(physicsVars_, launch, atmos, terrainStorage_, std::move(model)),
	  rollPhase(physicsVars_, launch, atmos, terrainStorage_)
{
	initializeFromLaunch(launch);
}

void FlightSimulator::initializeFromLaunch(const LaunchData &launch)
{
	const float v0_fps = launch.ballSpeedMph * physics_constants::MPH_TO_FT_PER_S;
	Vector3D startPos{launch.startX, launch.startY, launch.startZ};

	state = BallState::fromLaunchParameters(
		v0_fps,
		launch.launchAngleDeg,
		launch.directionDeg,
		startPos,
		physics_constants::GRAVITY_FT_PER_S2,
		physicsVars_.getROmega());

	aerialPhase.initialize(state);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void FlightSimulator::run(float dt)
{
	while (currentPhase != Phase::Complete)
	{
		stepOnce(dt);
	}
}

std::vector<BallState> FlightSimulator::runAndGetTrajectory(float dt)
{
	std::vector<BallState> trajectory;
	trajectory.reserve(static_cast<size_t>(10.0F / dt)); // ~10s at dt typical shots

	while (currentPhase != Phase::Complete)
	{
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
	LandingResult result;
	result.xF = state.position[0] / physics_constants::YARDS_TO_FEET;
	result.yF = state.position[1] / physics_constants::YARDS_TO_FEET;
	result.zF = state.position[2] / physics_constants::YARDS_TO_FEET;
	result.timeOfFlight = state.currentTime;
	result.bearing = std::atan2(state.position[0], state.position[1]) *
	                 180.0F / physics_constants::PI;
	result.distance = math_utils::getDistanceInYards(state.position);
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
