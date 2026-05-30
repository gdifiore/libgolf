#ifndef DEFAULT_BOUNCE_MODEL_HPP
#define DEFAULT_BOUNCE_MODEL_HPP

#include "BounceModel.hpp"
#include "math_utils.hpp"
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
 * Model parameters live as static constexpr members on this class. To
 * change them, subclass or implement BounceModel directly.
 *
 * Reference: Penner, A.R. "The physics of golf" (Reports on Progress
 * in Physics, 2003); openfairway BounceCalculator.cs:215-250.
 */
class DefaultBounceModel : public BounceModel
{
public:
	// ========================================================================
	// PENNER REGIME GATE
	// ========================================================================

	/// Minimum impact speed (ft/s) for Penner spin-back tangential model.
	/// Below this, simple friction retention is used regardless of impact
	/// angle — prevents non-physical spin-back on low-energy chip shots.
	/// 20 m/s ≈ 65.617 ft/s. Reference: Penner (2003) regime where the
	/// rigid-body slip / no-slip transition matches measured wedge bounces.
	static constexpr float MIN_PENNER_BOUNCE_SPEED_FT_PER_S =
		20.0F * physics_constants::METERS_TO_FEET;

	// ========================================================================
	// COR — SPIN/VELOCITY MODULATION
	// ========================================================================
	// Effective COR = surface.restitution * (1 - reduction), where
	//   reduction = maxReduction(rpm) * velocityScale(speedNormal_m/s).
	// High spin causes the ball to bite into turf rather than spring off
	// (flop / wedge bite). The velocity scale prevents low-energy chip
	// shots from getting an unwarranted COR penalty just from carried spin.
	// Reference: openfairway BounceCalculator.cs:215-250.

	/// Spin RPM at/below which there is no COR reduction. Reduction ramps
	/// linearly from 0 here to BOUNCE_COR_SPIN_LOW_MAX_REDUCTION at the knee.
	static constexpr float BOUNCE_COR_SPIN_KNEE_RPM = 1500.0F;

	/// Above the knee, additional spin extends the reduction up to
	/// BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION at KNEE_RPM + HIGH_BAND_RPM.
	static constexpr float BOUNCE_COR_SPIN_HIGH_BAND_RPM = 1500.0F;

	/// Reduction at the knee (low-spin asymptote).
	static constexpr float BOUNCE_COR_SPIN_LOW_MAX_REDUCTION = 0.30F;

	/// Reduction at saturation (high-spin asymptote).
	static constexpr float BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION = 0.70F;

	/// Normal-component impact speed (m/s) below which the velocity scale
	/// ramps from 0 to BOUNCE_COR_VEL_MID_SCALE.
	static constexpr float BOUNCE_COR_VEL_LOW_MS = 12.0F;

	/// Velocity scale at BOUNCE_COR_VEL_LOW_MS — partial COR penalty.
	static constexpr float BOUNCE_COR_VEL_MID_SCALE = 0.50F;

	/// At/above this normal speed the full COR penalty applies.
	static constexpr float BOUNCE_COR_VEL_HIGH_MS = 25.0F;

	// ========================================================================
	// RETENTION (PENNER BRANCH) — SPIN-COUPLED
	// ========================================================================
	// The `retention` multiplier in v_t' = retention * |v| * sin(θ - θ_crit)
	// is reduced as spin grows: higher spin loses more forward push to bite,
	// tightening wedge total-distance. Reference: openfairway
	// 0.55 * clamp(1 - rpm/8000, 0.4, 1.0).

	static constexpr float BOUNCE_RETENTION_BASE = 0.55F;
	static constexpr float BOUNCE_RETENTION_RPM_NORM = 8000.0F;
	static constexpr float BOUNCE_RETENTION_FLOOR = 0.40F;

	[[nodiscard]] BounceResult resolveBounce(const BounceState &state,
	                                         const GroundSurface &surface) const override
	{
		const float vDotN = math_utils::dot(state.velocity, state.surfaceNormal);

		const Vector3D vNormal = state.surfaceNormal * vDotN;
		const Vector3D vTangent = state.velocity - vNormal;

		const float tangentMag = math_utils::magnitude(vTangent);
		const float impactSpeed = math_utils::magnitude(state.velocity);
		const float omegaMag = math_utils::magnitude(state.spinVector);

		const float spinRpm = omegaMag / physics_constants::RPM_TO_RAD_PER_S;
		const float speedNormalMs =
			std::abs(vDotN) * physics_constants::FEET_TO_METERS;

		const float effectiveCor = surface.restitution *
			(1.0F - corMaxReduction(spinRpm) * corVelocityScale(speedNormalMs));

		const Vector3D vNormalAfter = vNormal * -effectiveCor;

		float impactAngle = 0.0F;
		if (impactSpeed > physics_constants::MIN_SPEED)
		{
			const float sinAngle = std::clamp(-vDotN / impactSpeed, -1.0F, 1.0F);
			impactAngle = std::asin(sinAngle);
		}

		const bool steepImpact = impactAngle >= surface.criticalAngle;
		const bool energeticImpact =
			impactSpeed >= MIN_PENNER_BOUNCE_SPEED_FT_PER_S;

		Vector3D vTangentAfter{};

		if (steepImpact && energeticImpact &&
		    tangentMag > physics_constants::MIN_SPEED)
		{
			// Backspin scalar: positive when spin axis aligns with t̂ × n̂.
			// For ball moving +Y, normal +Z: t̂ × n̂ = +X — matches golf
			// convention (right-hand axis = backspin).
			const Vector3D tHat = vTangent * (1.0F / tangentMag);
			const Vector3D lateralAxis = math_utils::cross(tHat, state.surfaceNormal);
			const float backspinScalar = math_utils::dot(state.spinVector, lateralAxis);

			const float retention =
				BOUNCE_RETENTION_BASE *
				std::clamp(1.0F - spinRpm / BOUNCE_RETENTION_RPM_NORM,
				           BOUNCE_RETENTION_FLOOR, 1.0F);
			const float spinbackTerm =
				(2.0F * state.ballRadius * backspinScalar) / 7.0F;
			const float newTangentSpeed =
				retention * impactSpeed * std::sin(impactAngle - surface.criticalAngle) -
				spinbackTerm;

			vTangentAfter = vTangent * (newTangentSpeed / tangentMag);
		}
		else
		{
			float frictionFactor = 1.0F - surface.frictionStatic * (1.0F - surface.firmness);
			frictionFactor = std::clamp(frictionFactor, 0.0F, 1.0F);
			vTangentAfter = vTangent * frictionFactor;
		}

		return BounceResult{
			vNormalAfter + vTangentAfter,
			state.spinVector * surface.spinRetention
		};
	}

private:
	static float corVelocityScale(float speedNormalMs)
	{
		if (speedNormalMs < BOUNCE_COR_VEL_LOW_MS)
		{
			return BOUNCE_COR_VEL_MID_SCALE *
			       (speedNormalMs / BOUNCE_COR_VEL_LOW_MS);
		}
		if (speedNormalMs < BOUNCE_COR_VEL_HIGH_MS)
		{
			const float t =
				(speedNormalMs - BOUNCE_COR_VEL_LOW_MS) /
				(BOUNCE_COR_VEL_HIGH_MS - BOUNCE_COR_VEL_LOW_MS);
			return BOUNCE_COR_VEL_MID_SCALE +
			       (1.0F - BOUNCE_COR_VEL_MID_SCALE) * t;
		}
		return 1.0F;
	}

	static float corMaxReduction(float spinRpm)
	{
		if (spinRpm < BOUNCE_COR_SPIN_KNEE_RPM)
		{
			return (spinRpm / BOUNCE_COR_SPIN_KNEE_RPM) *
			       BOUNCE_COR_SPIN_LOW_MAX_REDUCTION;
		}
		const float excess = spinRpm - BOUNCE_COR_SPIN_KNEE_RPM;
		const float t = std::min(
			excess / BOUNCE_COR_SPIN_HIGH_BAND_RPM, 1.0F);
		return BOUNCE_COR_SPIN_LOW_MAX_REDUCTION +
		       (BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION -
		        BOUNCE_COR_SPIN_LOW_MAX_REDUCTION) * t;
	}
};

#endif // DEFAULT_BOUNCE_MODEL_HPP
