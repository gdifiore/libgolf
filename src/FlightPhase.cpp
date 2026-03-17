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
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "ground_physics.hpp"
#include "physics_constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

// ============================================================================
// AerialPhase Implementation
// ============================================================================

AerialPhase::AerialPhase(
	GolfBallPhysicsVariables &physicsVars, const golfBall &ball,
	const atmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), terrain(terrain)
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}

	// Initialize calculated variables
	v = 0.0F;
	vMph = 0.0F;
	phi = 0.0F;
	tau = 0.0F;
	rw = 0.0F;
	Re_x_e5 = 0.0F;
	vw = 0.0F;
	vwMph = 0.0F;
	spinFactor = 0.0F;
	velocity3D_w = {0.0F, 0.0F, 0.0F};
	accelerationDrag3D = {0.0F, 0.0F, 0.0F};
	accelerationMagnitude3D = {0.0F, 0.0F, 0.0F};
}

void AerialPhase::initialize(BallState &state)
{
	// Initialize spin from physicsVars if not already set
	if (std::abs(state.spinRate) < physics_constants::MIN_VELOCITY_THRESHOLD)
	{
		state.spinRate = physicsVars.getROmega();
	}

	// Calculate initial derived values without advancing time or position
	v = sqrt(state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1] +
			 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

	calculateVelocityw(state);
	vw = v;

	calculatePhi(state);
	calculateTau();
	calculateRw(state);
	calculateRe_x_e5();
	calculateSpinFactor();

	calculateAccelD(state);
	calculateAccelM(state);
	calculateAccel(state);
}

void AerialPhase::calculateAccelerations(BallState &state)
{
	// Calculate derived values and accelerations without modifying position or time
	v = sqrt(state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1] +
			 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

	calculateVelocityw(state);
	calculatePhi(state);
	calculateTau();
	calculateRw(state);
	calculateRe_x_e5();
	calculateSpinFactor();

	calculateAccelD(state);
	calculateAccelM(state);
	calculateAccel(state);
}

void AerialPhase::calculateStep(BallState &state, float dt)
{
	state.currentTime += dt;

	// Update spin with exponential decay
	// Exponential model is appropriate for aerodynamic damping, where
	// the torque opposing spin is proportional to the spin rate itself
	calculateTau();
	state.spinRate = state.spinRate * exp(-dt / tau);

	calculatePosition(state, dt);
	calculateV(state, dt);
	calculateVelocityw(state);
	calculatePhi(state);
	calculateRw(state);
	calculateRe_x_e5();
	calculateSpinFactor();
	calculateAccelD(state);
	calculateAccelM(state);
	calculateAccel(state);
}

bool AerialPhase::isPhaseComplete(const BallState &state) const
{
	// Aerial phase is complete when ball reaches ground level
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

	v = sqrt(vx * vx + vy * vy + vz * vz);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculateVelocityw(const BallState &state)
{
	if (state.position[2] >= atmos.hWind)
	{
		velocity3D_w[0] = physicsVars.getVw()[0];
		velocity3D_w[1] = physicsVars.getVw()[1];
		vw = sqrt(pow(state.velocity[0] - velocity3D_w[0], 2) +
				  pow(state.velocity[1] - velocity3D_w[1], 2) + pow(state.velocity[2], 2));
	}
	else
	{
		velocity3D_w[0] = 0;
		velocity3D_w[1] = 0;
		vw = v;
	}

	vwMph = vw / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculatePhi(const BallState &state)
{
	// Currently unused in force calculations — kept for potential future use.
	// As a private member function, the optimizer will compile this out in
	// release builds if it remains uncalled.
	phi = atan2(state.position[1], state.position[2]) * 180 / physics_constants::PI;
}

void AerialPhase::calculateTau()
{
	const float ballRadius = physics_constants::STD_BALL_CIRCUMFERENCE_IN /
							 (2 * physics_constants::PI * physics_constants::INCHES_PER_FOOT);

	// Prevent division by zero or near-zero velocity
	if (v < physics_constants::MIN_VELOCITY_THRESHOLD)
	{
		tau = 1e6F;
		return;
	}

	tau = 1.0F / (physics_constants::TAU_COEFF * v / ballRadius);
}

void AerialPhase::calculateRw(const BallState &state)
{
	// Use current spin rate from state (which decays over time)
	rw = state.spinRate;
}

void AerialPhase::calculateRe_x_e5()
{
	Re_x_e5 = (vwMph / physics_constants::RE_VELOCITY_DIVISOR) *
			  physicsVars.getRe100() * physics_constants::RE_SCALE_FACTOR;
}

float AerialPhase::determineCoefficientOfDrag()
{
	if (getRe_x_e5() <= physics_constants::RE_THRESHOLD_LOW)
	{
		return physics_constants::CD_LOW;
	}
	else if (getRe_x_e5() < physics_constants::RE_THRESHOLD_HIGH)
	{
		return physics_constants::CD_LOW -
			   (physics_constants::CD_LOW - physics_constants::CD_HIGH) *
				   (Re_x_e5 - physics_constants::RE_THRESHOLD_LOW) / (physics_constants::RE_THRESHOLD_HIGH - physics_constants::RE_THRESHOLD_LOW) +
			   physics_constants::CD_SPIN * spinFactor;
	}
	else
	{
		return physics_constants::CD_HIGH + physics_constants::CD_SPIN * spinFactor;
	}
}

float AerialPhase::determineCoefficientOfLift()
{
	if (spinFactor <= physics_constants::SPIN_FACTOR_THRESHOLD)
	{
		return physics_constants::LIFT_COEFF1 * spinFactor +
			   physics_constants::LIFT_COEFF2 * pow(spinFactor, 2);
	}
	else
	{
		return physics_constants::CL_DEFAULT;
	}
}

void AerialPhase::calculateSpinFactor()
{
	if (vw < physics_constants::MIN_VELOCITY_THRESHOLD)
	{
		spinFactor = 0.0F; // Ball essentially stationary, no meaningful spin factor
	}
	else
	{
		spinFactor = rw / vw;
	}
}

void AerialPhase::calculateAccelD(const BallState &state)
{
	accelerationDrag3D[0] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * (state.velocity[0] - velocity3D_w[0]);
	accelerationDrag3D[1] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * (state.velocity[1] - velocity3D_w[1]);
	accelerationDrag3D[2] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * state.velocity[2];
}

void AerialPhase::calculateAccelM(const BallState &state)
{
	accelerationMagnitude3D[0] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[1] * state.velocity[2] -
		 physicsVars.getW()[2] * (state.velocity[1] - velocity3D_w[1])) /
		w_perp_div_w;
	accelerationMagnitude3D[1] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[2] * (state.velocity[0] - velocity3D_w[0]) -
		 physicsVars.getW()[0] * state.velocity[2]) /
		w_perp_div_w;
	accelerationMagnitude3D[2] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[0] * (state.velocity[1] - velocity3D_w[1]) -
		 physicsVars.getW()[1] * (state.velocity[0] - velocity3D_w[0])) /
		w_perp_div_w;
}

void AerialPhase::calculateAccel(BallState &state)
{
	state.acceleration[0] = accelerationDrag3D[0] + accelerationMagnitude3D[0];
	state.acceleration[1] = accelerationDrag3D[1] + accelerationMagnitude3D[1];
	state.acceleration[2] = accelerationDrag3D[2] + accelerationMagnitude3D[2] -
							physics_constants::GRAVITY_FT_PER_S2;
}

// ============================================================================
// BouncePhase Implementation
// ============================================================================

BouncePhase::BouncePhase(
	GolfBallPhysicsVariables &physicsVars, const golfBall &ball,
	const atmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), terrain(terrain),
	  aerialPhase(physicsVars, ball, atmos, terrain)
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}
}

void BouncePhase::calculateStep(BallState &state, float dt)
{
	// Get terrain data at ball position
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	const GroundSurface &surface = terrain->getSurfaceProperties(state.position[0], state.position[1]);

	// Apply bounce impact when ball contacts ground while moving downward
	float velocityDotNormal = state.velocity[0] * surfaceNormal[0] +
							  state.velocity[1] * surfaceNormal[1] +
							  state.velocity[2] * surfaceNormal[2];

	if (state.position[2] <= terrainHeight && velocityDotNormal < 0.0F)
	{
		// Use ground physics module for realistic bounce
		auto result = GroundPhysics::calculateBounce(state.velocity, surfaceNormal, state.spinRate, surface);
		state.velocity = result.newVelocity;
		state.spinRate = result.newSpinRate;
		state.position[2] = terrainHeight;
	}

	// Calculate aerodynamic forces (drag, lift, Magnus effect)
	aerialPhase.calculateAccelerations(state);

	// Update position and velocity
	state.position[0] += state.velocity[0] * dt + 0.5F * state.acceleration[0] * dt * dt;
	state.position[1] += state.velocity[1] * dt + 0.5F * state.acceleration[1] * dt * dt;
	state.position[2] += state.velocity[2] * dt + 0.5F * state.acceleration[2] * dt * dt;

	state.velocity[0] += state.acceleration[0] * dt;
	state.velocity[1] += state.acceleration[1] * dt;
	state.velocity[2] += state.acceleration[2] * dt;

	state.currentTime += dt;

	// Ensure ball doesn't go below terrain (recalculate since position changed)
	terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	if (state.position[2] < terrainHeight)
	{
		state.position[2] = terrainHeight;
	}
}

bool BouncePhase::isPhaseComplete(const BallState &state) const
{
	// Get terrain data at ball position
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	float heightAboveGround = state.position[2] - terrainHeight;

	// Use ground physics module to determine transition
	return GroundPhysics::shouldTransitionToRoll(state.velocity, surfaceNormal, heightAboveGround);
}

// ============================================================================
// RollPhase Implementation
// ============================================================================

RollPhase::RollPhase(
	GolfBallPhysicsVariables &physicsVars, const golfBall &ball,
	const atmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), terrain(terrain)
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}
}

void RollPhase::calculateStep(BallState &state, float dt)
{
	// Get terrain data at ball position
	Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	const GroundSurface &surface = terrain->getSurfaceProperties(state.position[0], state.position[1]);

	// Store old velocity direction for reversal check
	float oldVelX = state.velocity[0];
	float oldVelY = state.velocity[1];

	// Use ground physics module to calculate acceleration on slope
	state.acceleration = GroundPhysics::calculateRollAcceleration(state.velocity, surfaceNormal, state.spinRate, surface);

	// Update velocity
	state.velocity[0] += state.acceleration[0] * dt;
	state.velocity[1] += state.acceleration[1] * dt;
	state.velocity[2] += state.acceleration[2] * dt;

	// Prevent velocity from reversing direction (clamp to zero instead)
	// Only prevent reversal if we had meaningful initial velocity
	// If velocity was near zero, allow direction change (e.g., ball starting to roll downhill)
	// Use same threshold as STOPPING_VELOCITY to avoid unrealistic stopping on slopes
	if (std::abs(oldVelX) > physics_constants::STOPPING_VELOCITY && oldVelX * state.velocity[0] < 0.0F)
	{
		state.velocity[0] = 0.0F;
	}
	if (std::abs(oldVelY) > physics_constants::STOPPING_VELOCITY && oldVelY * state.velocity[1] < 0.0F)
	{
		state.velocity[1] = 0.0F;
	}

	// Update position
	state.position[0] += state.velocity[0] * dt;
	state.position[1] += state.velocity[1] * dt;

	// Keep ball on terrain surface (recalculate height at new position)
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	state.position[2] = terrainHeight;
	state.velocity[2] = 0.0F;

	// Apply spin decay during rolling (handles both backspin and topspin)
	// Linear model is appropriate for rolling friction, where the ground
	// applies an approximately constant torque opposing the spin
	// Note: This differs from aerial phase which uses exponential decay
	// due to the different physics of aerodynamic vs. contact friction
	float spinDecay = physics_constants::ROLL_SPIN_DECAY_RATE * dt;
	if (std::abs(state.spinRate) > spinDecay)
	{
		// Decay toward zero, preserving spin direction
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
	// Roll is complete when ball velocity drops below stopping threshold
	float vHorizontal = sqrt(state.velocity[0] * state.velocity[0] +
							 state.velocity[1] * state.velocity[1]);
	return vHorizontal < physics_constants::STOPPING_VELOCITY;
}
