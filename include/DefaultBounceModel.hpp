#ifndef DEFAULT_BOUNCE_MODEL_HPP
#define DEFAULT_BOUNCE_MODEL_HPP

#include "BounceModel.hpp"
#include "physics_constants.hpp"

#include <algorithm>
#include <cmath>

/**
 * @brief Built-in bounce model.
 *
 * Decomposes velocity into normal and tangent components relative to
 * the surface. Applies a spin- and velocity-coupled effective COR to
 * the normal component (high backspin + high normal-impact speed
 * causes the ball to bite rather than spring off). Two tangent paths:
 *
 *   - Steep AND energetic impacts (above `criticalAngle` and
 *     `MIN_PENNER_BOUNCE_SPEED_FT_PER_S`): Penner spin-back model.
 *     Tangent can reverse — this is the wedge-check / spin-back
 *     behaviour. Retention is itself spin-coupled, tightening wedge
 *     total-distance at high spin.
 *   - Shallow OR low-energy impacts: simple friction retention.
 *     No spin coupling — chips and drives cannot reverse.
 *
 * Spin is uniformly scaled by `surface.spinRetention`. The lateral
 * spin component used for the Penner spinback term is the projection
 * `ω · (t̂ × n̂)` — for a ball moving +Y on a +Z normal, +X spin axis is
 * backspin (matches golf convention).
 *
 * Model parameters live as static constexpr members on this class; the
 * COR and retention curves draw from the universal `physics_constants`
 * `BOUNCE_*` block. To change them, subclass or implement BounceModel
 * directly.
 *
 * Reference: Penner, A.R. "The physics of golf" (Reports on Progress
 * in Physics, 2003); openfairway BounceCalculator.cs:215-250.
 */
class DefaultBounceModel : public BounceModel
{
public:
	BounceResult resolveBounce(const BounceState &state,
	                           const GroundSurface &surface) const override
	{
		const float vDotN =
			state.velocity[0] * state.surfaceNormal[0] +
			state.velocity[1] * state.surfaceNormal[1] +
			state.velocity[2] * state.surfaceNormal[2];

		const Vector3D vNormal = {
			state.surfaceNormal[0] * vDotN,
			state.surfaceNormal[1] * vDotN,
			state.surfaceNormal[2] * vDotN
		};
		const Vector3D vTangent = {
			state.velocity[0] - vNormal[0],
			state.velocity[1] - vNormal[1],
			state.velocity[2] - vNormal[2]
		};

		const float tangentMag = std::sqrt(
			vTangent[0] * vTangent[0] +
			vTangent[1] * vTangent[1] +
			vTangent[2] * vTangent[2]);

		const float impactSpeed = std::sqrt(
			state.velocity[0] * state.velocity[0] +
			state.velocity[1] * state.velocity[1] +
			state.velocity[2] * state.velocity[2]);

		const float omegaMag = std::sqrt(
			state.spinVector[0] * state.spinVector[0] +
			state.spinVector[1] * state.spinVector[1] +
			state.spinVector[2] * state.spinVector[2]);

		const float spinRpm = omegaMag / physics_constants::RPM_TO_RAD_PER_S;
		const float speedNormalMs =
			std::abs(vDotN) * physics_constants::FEET_TO_METERS;

		const float effectiveCor = surface.restitution *
			(1.0F - corMaxReduction(spinRpm) * corVelocityScale(speedNormalMs));

		const Vector3D vNormalAfter = {
			-effectiveCor * vNormal[0],
			-effectiveCor * vNormal[1],
			-effectiveCor * vNormal[2]
		};

		float impactAngle = 0.0F;
		if (impactSpeed > physics_constants::MIN_VELOCITY_THRESHOLD)
		{
			const float sinAngle = std::clamp(-vDotN / impactSpeed, -1.0F, 1.0F);
			impactAngle = std::asin(sinAngle);
		}

		const bool steepImpact = impactAngle >= surface.criticalAngle;
		const bool energeticImpact =
			impactSpeed >= physics_constants::MIN_PENNER_BOUNCE_SPEED_FT_PER_S;

		Vector3D vTangentAfter{};

		if (steepImpact && energeticImpact &&
		    tangentMag > physics_constants::MIN_VELOCITY_THRESHOLD)
		{
			// Backspin scalar: positive when spin axis aligns with t̂ × n̂.
			// For ball moving +Y, normal +Z: t̂ × n̂ = +X — matches golf
			// convention (right-hand axis = backspin).
			const Vector3D tHat = {
				vTangent[0] / tangentMag,
				vTangent[1] / tangentMag,
				vTangent[2] / tangentMag
			};
			const Vector3D lateralAxis = {
				tHat[1] * state.surfaceNormal[2] - tHat[2] * state.surfaceNormal[1],
				tHat[2] * state.surfaceNormal[0] - tHat[0] * state.surfaceNormal[2],
				tHat[0] * state.surfaceNormal[1] - tHat[1] * state.surfaceNormal[0]
			};
			const float backspinScalar =
				state.spinVector[0] * lateralAxis[0] +
				state.spinVector[1] * lateralAxis[1] +
				state.spinVector[2] * lateralAxis[2];

			const float retention =
				physics_constants::BOUNCE_RETENTION_BASE *
				std::clamp(1.0F - spinRpm / physics_constants::BOUNCE_RETENTION_RPM_NORM,
				           physics_constants::BOUNCE_RETENTION_FLOOR, 1.0F);
			const float spinbackTerm =
				(2.0F * state.ballRadius * backspinScalar) / 7.0F;
			const float newTangentSpeed =
				retention * impactSpeed * std::sin(impactAngle - surface.criticalAngle) -
				spinbackTerm;

			const float scale = newTangentSpeed / tangentMag;
			vTangentAfter = {
				vTangent[0] * scale,
				vTangent[1] * scale,
				vTangent[2] * scale
			};
		}
		else
		{
			float frictionFactor = 1.0F - surface.frictionStatic * (1.0F - surface.firmness);
			frictionFactor = std::clamp(frictionFactor, 0.0F, 1.0F);
			vTangentAfter = {
				vTangent[0] * frictionFactor,
				vTangent[1] * frictionFactor,
				vTangent[2] * frictionFactor
			};
		}

		BounceResult out;
		out.newVelocity = {
			vNormalAfter[0] + vTangentAfter[0],
			vNormalAfter[1] + vTangentAfter[1],
			vNormalAfter[2] + vTangentAfter[2]
		};
		out.newSpinVector = {
			state.spinVector[0] * surface.spinRetention,
			state.spinVector[1] * surface.spinRetention,
			state.spinVector[2] * surface.spinRetention
		};
		return out;
	}

private:
	static float corVelocityScale(float speedNormalMs)
	{
		if (speedNormalMs < physics_constants::BOUNCE_COR_VEL_LOW_MS)
		{
			return physics_constants::BOUNCE_COR_VEL_MID_SCALE *
			       (speedNormalMs / physics_constants::BOUNCE_COR_VEL_LOW_MS);
		}
		if (speedNormalMs < physics_constants::BOUNCE_COR_VEL_HIGH_MS)
		{
			const float t =
				(speedNormalMs - physics_constants::BOUNCE_COR_VEL_LOW_MS) /
				(physics_constants::BOUNCE_COR_VEL_HIGH_MS -
				 physics_constants::BOUNCE_COR_VEL_LOW_MS);
			return physics_constants::BOUNCE_COR_VEL_MID_SCALE +
			       (1.0F - physics_constants::BOUNCE_COR_VEL_MID_SCALE) * t;
		}
		return 1.0F;
	}

	static float corMaxReduction(float spinRpm)
	{
		if (spinRpm < physics_constants::BOUNCE_COR_SPIN_KNEE_RPM)
		{
			return (spinRpm / physics_constants::BOUNCE_COR_SPIN_KNEE_RPM) *
			       physics_constants::BOUNCE_COR_SPIN_LOW_MAX_REDUCTION;
		}
		const float excess = spinRpm - physics_constants::BOUNCE_COR_SPIN_KNEE_RPM;
		const float t = std::min(
			excess / physics_constants::BOUNCE_COR_SPIN_HIGH_BAND_RPM, 1.0F);
		return physics_constants::BOUNCE_COR_SPIN_LOW_MAX_REDUCTION +
		       (physics_constants::BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION -
		        physics_constants::BOUNCE_COR_SPIN_LOW_MAX_REDUCTION) * t;
	}
};

#endif // DEFAULT_BOUNCE_MODEL_HPP
