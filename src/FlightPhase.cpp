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
#include "DefaultBounceModel.hpp"
#include "DefaultIntegrator.hpp"
#include "DefaultRollModel.hpp"
#include "atmospheric_data.hpp"
#include "math_utils.hpp"
#include "launch_data.hpp"
#include "ground_physics.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace
{
	template <typename Default, typename Base>
	std::shared_ptr<Base> orDefault(std::shared_ptr<Base> p)
	{
		return p ? std::move(p) : std::make_shared<Default>();
	}

	// Effective wind at the ball: the launch wind above hWind, otherwise none.
	Vector3D effectiveWind(const ShotPhysicsContext &physicsVars,
	                       const AtmosphericData &atmos, float heightZ)
	{
		if (heightZ >= atmos.hWind)
		{
			return {physicsVars.getVw()[0], physicsVars.getVw()[1], 0.0F};
		}
		return {0.0F, 0.0F, 0.0F};
	}

	// Snapshot the ball + launch atmosphere into the model-facing state.
	AerodynamicState buildAeroState(const BallState &state,
	                                const ShotPhysicsContext &physicsVars,
	                                const AtmosphericData &atmos, float ballRadius)
	{
		const Vector3D wind = effectiveWind(physicsVars, atmos, state.position[2]);
		return AerodynamicState{
		    .velocity          = state.velocity,
		    .windVelocity      = {wind[0], wind[1], 0.0F}, // vertical wind not modelled
		    .spinVector        = state.spinVector,
		    .position          = state.position,
		    .currentTime       = state.currentTime,
		    .ballRadius        = ballRadius,
		    .airDensityKgPerM3 = physicsVars.getRhoMetric(),
		    .airViscosity      = physicsVars.getAirViscosity(),
		    .tempKelvin        = physicsVars.getTempKelvin(),
		    .pressureMmHg      = physicsVars.getBarometricPressure(),
		    .relHumidity       = physicsVars.getRelHumidity(),
		    .c0                = physicsVars.getC0(),
		    .re100             = physicsVars.getRe100()
		};
	}

	// Aerodynamic acceleration plus gravity, shared by the aerial and
	// between-bounce flight steps.
	Vector3D flightAcceleration(const AerodynamicModel &model,
	                            const AerodynamicState &aeroState, float gravity)
	{
		const Vector3D aero = model.computeAcceleration(aeroState);
		return {aero[0], aero[1], aero[2] - gravity};
	}

	// Acceleration field an Integrator can sample at trial states, built once per
	// phase and reused every step. The per-step spin (decayed once, before the
	// step, then held fixed) is read through stepSpin so the field need not be
	// rebuilt. Captures are move-safe: the model is held by shared_ptr, atmos by
	// value, and physicsVars outlives the phase, so a moved-from phase does not
	// dangle the field.
	AccelerationField makeAccelField(std::shared_ptr<AerodynamicModel> model,
	                                 const ShotPhysicsContext &physicsVars,
	                                 const AtmosphericData &atmos, float ballRadius,
	                                 float gravity, std::shared_ptr<Vector3D> stepSpin)
	{
		return [model = std::move(model), &physicsVars, atmos, ballRadius, gravity,
		        stepSpin = std::move(stepSpin)](const IntegratorState &s) -> Vector3D {
			BallState trial;
			trial.position    = s.position;
			trial.velocity    = s.velocity;
			trial.spinVector  = *stepSpin;
			trial.currentTime = s.time;
			return flightAcceleration(
			    *model, buildAeroState(trial, physicsVars, atmos, ballRadius), gravity);
		};
	}
}

// ============================================================================
// AerialPhase Implementation
// ============================================================================

AerialPhase::AerialPhase(
	ShotPhysicsContext &physicsVars, [[maybe_unused]] const LaunchData &launch,
	const AtmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<AerodynamicModel> model, const BallProperties &ball,
	float gravity, std::shared_ptr<Integrator> integrator)
	: physicsVars(physicsVars), atmos(atmos), terrain(terrain),
	  model(orDefault<DefaultAerodynamicModel>(std::move(model))),
	  integrator(orDefault<DefaultIntegrator>(std::move(integrator))),
	  ballRadius(ball.radiusFt()), gravity(gravity),
	  stepSpin(std::make_shared<Vector3D>()),
	  accelField(makeAccelField(this->model, physicsVars, atmos, ballRadius, gravity, stepSpin))
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}

	v    = 0.0F;
	vMph = 0.0F;
	tau  = 0.0F;
	rw   = 0.0F;
	vw   = 0.0F;
	vwMph = 0.0F;
	velocity3D_w = {0.0F, 0.0F, 0.0F};
}

void AerialPhase::initialize(BallState &state)
{
	if (math_utils::magnitude(state.spinVector) < physics_constants::MIN_SPIN)
	{
		state.spinVector = physicsVars.getW();
	}

	v    = std::sqrt(state.velocity[0] * state.velocity[0] +
	                 state.velocity[1] * state.velocity[1] +
	                 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

	calculateVelocityw(state);

	calculateTau(state);
	calculateRw(state);
	calculateAccel(state);
}

void AerialPhase::calculateStep(BallState &state, float dt)
{
	state.currentTime += dt;

	// Spin decay uses the velocity from the previous step (v is still current).
	// Exponential model: torque opposing spin is proportional to spin rate itself.
	// All components decay uniformly — axis direction is preserved.
	calculateTau(state);
	const float decay = std::exp(-dt / tau);
	state.spinVector[0] *= decay;
	state.spinVector[1] *= decay;
	state.spinVector[2] *= decay;

	// state.acceleration holds the start-of-step acceleration; advance position
	// and velocity through the (pluggable) integrator. The field samples the
	// just-decayed spin held in stepSpin.
	*stepSpin = state.spinVector;
	integrator->step(state, dt, accelField);

	v    = math_utils::magnitude(state.velocity);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;
	calculateVelocityw(state);   // updates velocity3D_w, vw, vwMph
	calculateRw(state);
	calculateAccel(state);       // start-of-step acceleration for the next step
}

bool AerialPhase::isPhaseComplete(const BallState &state) const
{
	float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	return state.position[2] <= terrainHeight;
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

void AerialPhase::calculateTau(const BallState &state)
{
	if (v < physics_constants::MIN_SPEED)
	{
		tau = 1e6F;
		return;
	}

	tau = model->computeSpinDecayTau(buildAeroState(state, physicsVars, atmos, ballRadius));
}

void AerialPhase::calculateRw(const BallState &state)
{
	rw = math_utils::magnitude(state.spinVector);
}

void AerialPhase::calculateAccel(BallState &state)
{
	*stepSpin = state.spinVector;
	state.acceleration =
	    accelField(IntegratorState{state.position, state.velocity, state.currentTime});
}

// ============================================================================
// BouncePhase Implementation
// ============================================================================

BouncePhase::BouncePhase(
	ShotPhysicsContext &physicsVars, [[maybe_unused]] const LaunchData &launch,
	const AtmosphericData &atmos, std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<AerodynamicModel> aeroModel,
	std::shared_ptr<BounceModel> bounceModel,
	const BallProperties &ball,
	float gravity, std::shared_ptr<Integrator> integrator)
	: physicsVars(physicsVars), atmos(atmos), terrain(terrain),
	  model(orDefault<DefaultAerodynamicModel>(std::move(aeroModel))),
	  bounceModel(orDefault<DefaultBounceModel>(std::move(bounceModel))),
	  integrator(orDefault<DefaultIntegrator>(std::move(integrator))),
	  ballRadius(ball.radiusFt()), gravity(gravity),
	  stepSpin(std::make_shared<Vector3D>()),
	  accelField(makeAccelField(this->model, physicsVars, atmos, ballRadius, gravity, stepSpin))
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
		BounceState bounceState{
			state.velocity,
			surfaceNormal,
			state.spinVector,
			ballRadius
		};
		BounceResult result = bounceModel->resolveBounce(bounceState, surface);
		state.velocity = result.newVelocity;
		state.spinVector = result.newSpinVector;
		state.position[2] = terrainHeight;
	}

	// Aerodynamic acceleration for the free-flight arc between bounces, advanced
	// through the same (pluggable) integrator the aerial phase uses. Both the
	// start-of-step acceleration and the integrator's trial samples come from the
	// one cached field, so they cannot drift apart. Spin is fixed across the arc.
	*stepSpin = state.spinVector;
	state.acceleration =
	    accelField(IntegratorState{state.position, state.velocity, state.currentTime});

	integrator->step(state, dt, accelField);

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
	std::shared_ptr<TerrainInterface> terrain,
	std::shared_ptr<RollModel> model,
	const BallProperties &ball)
	: terrain(terrain),
	  model(orDefault<DefaultRollModel>(std::move(model))),
	  ballRadius(ball.radiusFt())
{
	if (!terrain)
	{
		throw std::invalid_argument(std::string(__func__) + ": Terrain interface must not be null");
	}
}

void RollPhase::calculateStep(BallState &state, float dt)
{
	const Vector3D surfaceNormal = terrain->getNormal(state.position[0], state.position[1]);
	const GroundSurface &surface = terrain->getSurfaceProperties(state.position[0], state.position[1]);

	const RollState rollState{
		state.position,
		state.velocity,
		state.spinVector,
		surfaceNormal,
		ballRadius,
		dt,
		terrain.get()
	};

	const RollResult result = model->step(rollState, surface);

	state.position = result.newPosition;
	state.velocity = result.newVelocity;
	state.spinVector = result.newSpinVector;

	// Library still owns final terrain clamping even though the model can
	// peek at terrain mid-step via `RollState::terrain`.
	const float terrainHeight = terrain->getHeight(state.position[0], state.position[1]);
	state.position[2] = terrainHeight;
	state.velocity[2] = 0.0F;

	state.currentTime += dt;
	atRest = result.atRest;
}

bool RollPhase::isPhaseComplete(const BallState &state) const
{
	(void)state;
	return atRest;
}
