#ifndef DEFAULT_ROLL_MODEL_HPP
#define DEFAULT_ROLL_MODEL_HPP

#include "RollModel.hpp"
#include "physics_constants.hpp"

#include <cmath>

/**
 * @brief Built-in roll model.
 *
 * Computes acceleration from gravity component along the slope and
 * Coulomb friction opposing horizontal motion. Integrates with explicit
 * Euler. Stops the horizontal component when the velocity sign flips
 * across a step (prevents Coulomb friction from reversing direction at
 * low speeds). Clamps the ball to the terrain height. Decays spin
 * linearly while preserving axis direction.
 *
 * Phase completes when horizontal velocity drops below
 * `STOPPING_VELOCITY`.
 *
 * Model parameters live as static constexpr members on this class. To
 * change them, subclass or implement RollModel directly.
 */
class DefaultRollModel : public RollModel
{
public:
	/// Stopping velocity threshold for roll phase (ft/s)
	/// Below this, the ball is considered at rest.
	static constexpr float STOPPING_VELOCITY = 0.1F;

	/// Spin decay rate during rolling (rad/s per second)
	/// Ground friction decays spin faster than air drag.
	static constexpr float SPIN_DECAY_RATE = 2.0F;

	RollResult step(const RollState &state,
	                const GroundSurface &surface) const override
	{
		const float dt = state.dt;

		const Vector3D accel = computeAcceleration(state.velocity, state.surfaceNormal, surface);

		const float oldVelX = state.velocity[0];
		const float oldVelY = state.velocity[1];

		Vector3D newVel = {
			state.velocity[0] + accel[0] * dt,
			state.velocity[1] + accel[1] * dt,
			state.velocity[2] + accel[2] * dt
		};

		// Sign-flip stop: if friction reversed velocity in this step, clamp
		// to zero. Only kicks in when the previous-step speed was already
		// above STOPPING_VELOCITY — avoids freezing a slope-accelerating ball
		// that legitimately starts near zero.
		if (std::abs(oldVelX) > STOPPING_VELOCITY && oldVelX * newVel[0] < 0.0F)
		{
			newVel[0] = 0.0F;
		}
		if (std::abs(oldVelY) > STOPPING_VELOCITY && oldVelY * newVel[1] < 0.0F)
		{
			newVel[1] = 0.0F;
		}

		Vector3D newPos = {
			state.position[0] + newVel[0] * dt,
			state.position[1] + newVel[1] * dt,
			state.position[2]
		};
		newVel[2] = 0.0F;

		// Linear spin decay: rolling friction approximates constant torque.
		// Magnitude drops by a fixed amount per second; axis is preserved.
		const float spinMag = std::sqrt(
			state.spinVector[0] * state.spinVector[0] +
			state.spinVector[1] * state.spinVector[1] +
			state.spinVector[2] * state.spinVector[2]);
		const float decay = SPIN_DECAY_RATE * dt;
		Vector3D newSpin{};
		if (spinMag > decay)
		{
			const float scale = (spinMag - decay) / spinMag;
			newSpin = {
				state.spinVector[0] * scale,
				state.spinVector[1] * scale,
				state.spinVector[2] * scale
			};
		}
		else
		{
			newSpin = {0.0F, 0.0F, 0.0F};
		}

		const float vHorizontal = std::sqrt(newVel[0] * newVel[0] + newVel[1] * newVel[1]);
		const bool atRest = vHorizontal < STOPPING_VELOCITY;

		return RollResult{newPos, newVel, newSpin, atRest};
	}

private:
	static Vector3D computeAcceleration(const Vector3D &velocity,
	                                    const Vector3D &surfaceNormal,
	                                    const GroundSurface &surface)
	{
		Vector3D acceleration = {0.0F, 0.0F, 0.0F};

		const float vHorizontal = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);

		// Stationary: friction direction undefined, gravity along slope is
		// balanced by static friction at rest.
		if (vHorizontal < physics_constants::MIN_VELOCITY_THRESHOLD)
		{
			return acceleration;
		}

		const float cosTheta = surfaceNormal[2];

		// Near-flat: skip slope decomposition.
		if (cosTheta > physics_constants::FLAT_SURFACE_THRESHOLD)
		{
			const float deceleration = surface.frictionDynamic * physics_constants::GRAVITY_FT_PER_S2;
			acceleration[0] = -deceleration * (velocity[0] / vHorizontal);
			acceleration[1] = -deceleration * (velocity[1] / vHorizontal);
			acceleration[2] = 0.0F;
			return acceleration;
		}

		const Vector3D gravity = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};

		const float gravityDotNormal = math_utils::dot(gravity, surfaceNormal);
		const Vector3D gravityNormal = {
			surfaceNormal[0] * gravityDotNormal,
			surfaceNormal[1] * gravityDotNormal,
			surfaceNormal[2] * gravityDotNormal
		};

		const Vector3D gravityTangent = {
			gravity[0] - gravityNormal[0],
			gravity[1] - gravityNormal[1],
			gravity[2] - gravityNormal[2]
		};

		acceleration[0] = gravityTangent[0];
		acceleration[1] = gravityTangent[1];
		acceleration[2] = gravityTangent[2];

		const float normalForce = std::abs(gravityDotNormal);
		const float frictionDeceleration = surface.frictionDynamic * normalForce;

		acceleration[0] -= frictionDeceleration * (velocity[0] / vHorizontal);
		acceleration[1] -= frictionDeceleration * (velocity[1] / vHorizontal);

		return acceleration;
	}
};

#endif // DEFAULT_ROLL_MODEL_HPP
