/**
 * @file FlightPhase.cpp
 * @author Gabriel DiFiore
 * @brief Contains the implementation of flight phase classes.
 *
 * This file defines the different flight phases for golf ball simulation:
 * AerialPhase, BouncePhase, and RollPhase.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "FlightPhase.hpp"
#include "DefaultAerodynamicModel.hpp"
#include "atmospheric_data.hpp"
#include "launch_data.hpp"
#include "ground_physics.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdexcept>

// ============================================================================
// AerialPhase Implementation
// ============================================================================

AerialPhase::AerialPhase(
	ShotPhysicsContext &physicsVars, const LaunchData &launch,
	const AtmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<AerodynamicModel> model)
	: physicsVars(physicsVars), launch(launch), atmos(atmos), terrain(terrain),
	  model_(model ? std::move(model) : std::make_shared<DefaultAerodynamicModel>())
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}

	v    = 0.0F;
	vMph = 0.0F;
	phi  = 0.0F;
	tau  = 0.0F;
	rw   = 0.0F;
	vw   = 0.0F;
	vwMph = 0.0F;
	velocity3D_w = {0.0F, 0.0F, 0.0F};
}

void AerialPhase::initialize(BallState &state)
{
	if (std::abs(state.spinRate) < physics_constants::MIN_VELOCITY_THRESHOLD)
	{
		state.spinRate = physicsVars.getROmega();
	}

	v    = std::sqrt(state.velocity[0] * state.velocity[0] +
	                 state.velocity[1] * state.velocity[1] +
	                 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

	calculateVelocityw(state);
	vw = v;

	calculatePhi(state);
	calculateTau(state);
	calculateRw(state);
	calculateAccel(state);
}

void AerialPhase::calculateAccelerations(BallState &state)
{
	v    = std::sqrt(state.velocity[0] * state.velocity[0] +
	                 state.velocity[1] * state.velocity[1] +
	                 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

	calculateVelocityw(state);
	calculatePhi(state);
	calculateTau(state);
	calculateRw(state);
	calculateAccel(state);
}

void AerialPhase::calculateStep(BallState &state, float dt)
{
	state.currentTime += dt;

	// Spin decay uses the velocity from the previous step (v is still current).
	// Exponential model: torque opposing spin is proportional to spin rate itself.
	calculateTau(state);
	state.spinRate = state.spinRate * std::exp(-dt / tau);

	calculatePosition(state, dt);
	calculateV(state, dt);       // updates v, vMph, state.velocity
	calculateVelocityw(state);   // updates velocity3D_w, vw, vwMph
	calculatePhi(state);
	calculateRw(state);
	calculateAccel(state);       // calls model, then adds gravity
}

bool AerialPhase::isPhaseComplete(const BallState &state) const
{
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	return state.position[2] <= terrainHeight;
}

void AerialPhase::calculatePosition(BallState &state, float dt)
{
	state.position[0] = state.position[0] + state.velocity[0] * dt +
						physics_constants::HALF * state.acceleration[0] * dt * dt;
	state.position[1] = state.position[1] + state.velocity[1] * dt +
						physics_constants::HALF * state.acceleration[1] * dt * dt;
	state.position[2] = state.position[2] + state.velocity[2] * dt +
						physics_constants::HALF * state.acceleration[2] * dt * dt;
}

void AerialPhase::calculateV(BallState &state, float dt)
{
	float vx = state.velocity[0] + state.acceleration[0] * dt;
	float vy = state.velocity[1] + state.acceleration[1] * dt;
	float vz = state.velocity[2] + state.acceleration[2] * dt;

	state.velocity = {vx, vy, vz};

	v    = std::sqrt(vx * vx + vy * vy + vz * vz);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculateVelocityw(const BallState &state)
{
	if (state.position[2] >= atmos.hWind)
	{
		velocity3D_w[0] = physicsVars.getVw()[0];
		velocity3D_w[1] = physicsVars.getVw()[1];
		vw = std::sqrt(std::pow(state.velocity[0] - velocity3D_w[0], 2) +
		               std::pow(state.velocity[1] - velocity3D_w[1], 2) +
		               std::pow(state.velocity[2], 2));
	}
	else
	{
		velocity3D_w[0] = 0.0F;
		velocity3D_w[1] = 0.0F;
		vw = v;
	}

	vwMph = vw / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculatePhi(const BallState &state)
{
	// Currently unused in force calculations — kept for potential future use.
	phi = std::atan2(state.position[1], state.position[2]) * 180.0F / physics_constants::PI;
}

void AerialPhase::calculateTau(const BallState &state)
{
	if (v < physics_constants::MIN_VELOCITY_THRESHOLD)
	{
		tau = 1e6F;
		return;
	}

	tau = static_cast<float>(model_->computeSpinDecayTau(buildAerodynamicState(state)));
}

void AerialPhase::calculateRw(const BallState &state)
{
	rw = state.spinRate;
}

void AerialPhase::calculateAccel(BallState &state)
{
	Vector3D aeroAccel = model_->computeAcceleration(buildAerodynamicState(state));
	state.acceleration[0] = aeroAccel[0];
	state.acceleration[1] = aeroAccel[1];
	state.acceleration[2] = aeroAccel[2] - physics_constants::GRAVITY_FT_PER_S2;
}

AerodynamicState AerialPhase::buildAerodynamicState(const BallState &state) const
{
	// Reconstruct the current spin vector in rad/s from the decayed scalar spin rate.
	//
	// state.spinRate stores surface speed (r * omega) in ft/s.
	// physicsVars.getW() is the full initial spin vector (rad/s components).
	// physicsVars.getROmega() is the initial surface speed (r * omega_0) in ft/s.
	//
	// Scaling W by (spinRate / rOmega) preserves the launch spin axis direction
	// while reducing the magnitude to match the current decayed spin rate.
	const float rOmega = physicsVars.getROmega();
	const float spinDecayRatio = (rOmega > physics_constants::MIN_VELOCITY_THRESHOLD)
	                                 ? (state.spinRate / rOmega)
	                                 : 0.0F;

	const Vector3D &W = physicsVars.getW();
	const Vector3D currentSpinVector = {
	    W[0] * spinDecayRatio,
	    W[1] * spinDecayRatio,
	    W[2] * spinDecayRatio
	};

	return AerodynamicState{
	    .velocity     = state.velocity,
	    .windVelocity = {velocity3D_w[0], velocity3D_w[1], 0.0F},
	    .spinVector   = currentSpinVector,
	    .c0           = physicsVars.getC0(),
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = physicsVars.getRe100()
	};
}

// ============================================================================
// BouncePhase Implementation
// ============================================================================

BouncePhase::BouncePhase(
	ShotPhysicsContext &physicsVars, const LaunchData &launch,
	const AtmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<AerodynamicModel> model)
	: physicsVars(physicsVars), launch(launch), atmos(atmos), terrain(terrain),
	  aerialPhase(physicsVars, launch, atmos, terrain, std::move(model))
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}
}

void BouncePhase::calculateStep(BallState &state, float dt)
{
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	const GroundSurface &surface = terrain->getSurfaceProperties(state.position[0], state.position[1]);

	float velocityDotNormal = state.velocity[0] * surfaceNormal[0] +
							  state.velocity[1] * surfaceNormal[1] +
							  state.velocity[2] * surfaceNormal[2];

	if (state.position[2] <= terrainHeight && velocityDotNormal < 0.0F)
	{
		auto result = GroundPhysics::calculateBounce(state.velocity, surfaceNormal, state.spinRate, surface);
		state.velocity  = result.newVelocity;
		state.spinRate  = result.newSpinRate;
		state.position[2] = terrainHeight;
	}

	aerialPhase.calculateAccelerations(state);

	state.position[0] += state.velocity[0] * dt + 0.5F * state.acceleration[0] * dt * dt;
	state.position[1] += state.velocity[1] * dt + 0.5F * state.acceleration[1] * dt * dt;
	state.position[2] += state.velocity[2] * dt + 0.5F * state.acceleration[2] * dt * dt;

	state.velocity[0] += state.acceleration[0] * dt;
	state.velocity[1] += state.acceleration[1] * dt;
	state.velocity[2] += state.acceleration[2] * dt;

	state.currentTime += dt;

	terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	if (state.position[2] < terrainHeight)
	{
		state.position[2] = terrainHeight;
	}
}

bool BouncePhase::isPhaseComplete(const BallState &state) const
{
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	float heightAboveGround = state.position[2] - terrainHeight;

	return GroundPhysics::shouldTransitionToRoll(state.velocity, surfaceNormal, heightAboveGround);
}

// ============================================================================
// RollPhase Implementation
// ============================================================================

RollPhase::RollPhase(
	ShotPhysicsContext &physicsVars, const LaunchData &launch,
	const AtmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain)
	: physicsVars(physicsVars), launch(launch), atmos(atmos), terrain(terrain)
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}
}

void RollPhase::calculateStep(BallState &state, float dt)
{
	Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	const GroundSurface &surface = terrain->getSurfaceProperties(state.position[0], state.position[1]);

	float oldVelX = state.velocity[0];
	float oldVelY = state.velocity[1];

	state.acceleration = GroundPhysics::calculateRollAcceleration(state.velocity, surfaceNormal, state.spinRate, surface);

	state.velocity[0] += state.acceleration[0] * dt;
	state.velocity[1] += state.acceleration[1] * dt;
	state.velocity[2] += state.acceleration[2] * dt;

	if (std::abs(oldVelX) > physics_constants::STOPPING_VELOCITY && oldVelX * state.velocity[0] < 0.0F)
	{
		state.velocity[0] = 0.0F;
	}
	if (std::abs(oldVelY) > physics_constants::STOPPING_VELOCITY && oldVelY * state.velocity[1] < 0.0F)
	{
		state.velocity[1] = 0.0F;
	}

	state.position[0] += state.velocity[0] * dt;
	state.position[1] += state.velocity[1] * dt;

	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	state.position[2] = terrainHeight;
	state.velocity[2] = 0.0F;

	// Linear spin decay: rolling friction applies approximately constant torque
	float spinDecay = physics_constants::ROLL_SPIN_DECAY_RATE * dt;
	if (std::abs(state.spinRate) > spinDecay)
	{
		state.spinRate -= std::copysign(spinDecay, state.spinRate);
	}
	else
	{
		state.spinRate = 0.0F;
	}

	state.currentTime += dt;
}

bool RollPhase::isPhaseComplete(const BallState &state) const
{
	float vHorizontal = std::sqrt(state.velocity[0] * state.velocity[0] +
	                              state.velocity[1] * state.velocity[1]);
	return vHorizontal < physics_constants::STOPPING_VELOCITY;
}
