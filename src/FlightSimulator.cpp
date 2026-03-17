/**
 * @file FlightSimulator.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of the FlightSimulator class for automated phase management.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "FlightSimulator.hpp"
#include <stdexcept>

FlightSimulator::FlightSimulator(
	const GolfBallPhysicsVariables &physicsVars, const golfBall &ball,
	const atmosphericData &atmos, const GroundSurface &ground,
	std::shared_ptr<TerrainInterface> terrain)
	: currentPhase(Phase::Aerial),
	  // state is default-constructed
	  initialized(false),
	  physicsVars_(physicsVars),
	  terrainStorage_(terrain ? terrain : std::make_shared<FlatTerrain>(ground)),
	  aerialPhase(physicsVars_, ball, atmos, terrainStorage_),
	  bouncePhase(physicsVars_, ball, atmos, terrainStorage_),
	  rollPhase(physicsVars_, ball, atmos, terrainStorage_)
{
}

FlightSimulator::FlightSimulator(
	const GolfBallPhysicsVariables &physicsVars, const golfBall &ball,
	const atmosphericData &atmos, const GroundProvider &groundProvider)
	: currentPhase(Phase::Aerial),
	  // state is default-constructed
	  initialized(false),
	  physicsVars_(physicsVars),
	  terrainStorage_(std::make_shared<TerrainProviderAdapter>(groundProvider)),
	  aerialPhase(physicsVars_, ball, atmos, terrainStorage_),
	  bouncePhase(physicsVars_, ball, atmos, terrainStorage_),
	  rollPhase(physicsVars_, ball, atmos, terrainStorage_)
{
}

void FlightSimulator::initialize(const BallState &initialState)
{
	state = initialState;
	currentPhase = Phase::Aerial;
	initialized = true;

	// Initialize the aerial phase with the starting state
	aerialPhase.initialize(state);
}

void FlightSimulator::step(float dt)
{
	// Ensure that the simulator has been initialized
	if (!initialized)
	{
		throw std::logic_error("FlightSimulator::step() called before initialize()");
	}

	// Don't step if already complete
	if (currentPhase == Phase::Complete)
	{
		return;
	}

	// Execute physics step for current phase
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
		// Already complete, do nothing
		break;
	}

	// Check if we need to transition to the next phase
	checkPhaseTransition();
}

bool FlightSimulator::isComplete() const
{
	return currentPhase == Phase::Complete;
}

const BallState &FlightSimulator::getState() const
{
	// Ensure that the simulator has been initialized
	if (!initialized)
	{
		throw std::logic_error("FlightSimulator::getState() called before initialize()");
	}
	return state;
}

const char *FlightSimulator::getCurrentPhaseName() const
{
	switch (currentPhase)
	{
	case Phase::Aerial:
		return "aerial";
	case Phase::Bounce:
		return "bounce";
	case Phase::Roll:
		return "roll";
	case Phase::Complete:
		return "complete";
	default:
		return "unknown";
	}
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
		// Already complete, stay in this state
		break;
	}
}
