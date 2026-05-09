#ifndef DEFAULT_AERODYNAMIC_MODEL_HPP
#define DEFAULT_AERODYNAMIC_MODEL_HPP

#include "AerodynamicModel.hpp"
#include "math_utils.hpp"
#include "physics_constants.hpp"

#include <algorithm>
#include <cmath>

/**
 * @brief Built-in aerodynamic model for a standard golf ball.
 *
 * Force law:
 *   F_drag   = -C0 * Cd * vw * (v - v_wind)
 *   F_magnus =  C0 * (Cl / |omega|) * vw * (omega × v_rel)
 *
 * where vw = |v - v_wind| and omega is the current spin vector.
 *
 * Drag model (piecewise-linear through the drag crisis):
 *   Re <= RE_THRESHOLD_LOW:                  Cd = CD_LOW
 *   RE_THRESHOLD_LOW < Re < RE_THRESHOLD_HIGH: linear + CD_SPIN * S
 *   Re >= RE_THRESHOLD_HIGH:                 Cd = CD_HIGH + CD_SPIN * S
 *
 * Lift model (Reynolds-binned, units Re_x_e5 = Re/1e5):
 *   Re <= 0.3:       Cl = 0
 *   0.3 < Re < 0.5:  smoothstep ramp toward Cl_50k(S)
 *   0.5 <= Re <= 0.7: linear interp between adjacent {50k,60k,65k,70k} bins
 *   Re > 0.7:        Hill saturation Cl = ClMax(S)·S·g / (1 + S·g)
 * All branches clamped to [0, ClMax(S)]. Bins are Bearman dimpled-sphere fits.
 * ClMax(S) lerps 0.268 → 0.32 across spin factor 0.35 → 0.50 — the asymptote
 * is not flat in the high-spin regime.
 *
 * Spin decay:
 *   tau = 1 / (TAU_COEFF * |v| / r)
 *
 * Reference: Washington State University study by Bin Lyu, et al.
 *
 * Model parameters live as static constexpr members on this class. To change
 * them, subclass or implement AerodynamicModel directly.
 *
 * computeCd() and computeCl() are public non-virtual helpers — useful for
 * inspection and as building blocks in derived models.
 */
class DefaultAerodynamicModel : public AerodynamicModel
{
public:
	// ========================================================================
	// DRAG FORCE LUMPED-SCALAR DERIVATION (consumed by ShotPhysicsContext)
	// ========================================================================
	// c0 = DRAG_FORCE_CONST * rho * (REF_BALL_MASS_OZ / mass) * (circ / REF_BALL_CIRC_IN)^2

	/// Drag force constant. c0 = 0.07182 * rho * (5.125/mass) * (circ/9.125)²
	static constexpr float DRAG_FORCE_CONST = 0.07182F;

	/// Reference golf ball mass for drag calculation (oz)
	static constexpr float REF_BALL_MASS_OZ = 5.125F;

	/// Reference golf ball circumference for drag calculation (inches)
	static constexpr float REF_BALL_CIRC_IN = 9.125F;

	// ========================================================================
	// SPIN DECAY
	// ========================================================================

	/// Tau (spin decay time constant) coefficient. tau = 1 / (TAU_COEFF * v / r)
	static constexpr float TAU_COEFF = 0.00002F;

	// ========================================================================
	// REYNOLDS NUMBER REGIME
	// ========================================================================

	/// Low Reynolds number threshold (× 10⁵). Below this, Cd = CD_LOW.
	static constexpr float RE_THRESHOLD_LOW = 0.5F;

	/// High Reynolds number threshold (× 10⁵). Above this, Cd = CD_HIGH.
	static constexpr float RE_THRESHOLD_HIGH = 1.0F;

	/// Reynolds-number scaling factor for comparison with thresholds.
	/// Re_scaled = (v_mph / 100) * Re100 * RE_SCALE_FACTOR
	static constexpr float RE_SCALE_FACTOR = 0.00001F;

	/// Velocity divisor for Reynolds-number calculation (mph)
	static constexpr float RE_VELOCITY_DIVISOR = 100.0F;

	// ========================================================================
	// DRAG COEFFICIENTS
	// ========================================================================
	// Reference: Washington State University study by Bin Lyu, et al.

	/// Drag coefficient with spin effect
	static constexpr float CD_SPIN = 0.180F;

	/// Drag coefficient for low Reynolds number (laminar). Re <= 0.5e5.
	static constexpr float CD_LOW = 0.500F;

	/// Drag coefficient for high Reynolds number (turbulent). Re >= 1.0e5.
	static constexpr float CD_HIGH = 0.200F;

	// ========================================================================
	// LIFT COEFFICIENTS — REYNOLDS-BINNED
	// ========================================================================
	// Reynolds-dependent Cl(S) curves, fit to Bearman/Harvey wind-tunnel data
	// for dimpled spheres. Applied in Re_x_e5 units (Re/1e5):
	//
	//   Re_x_e5 <= 0.3 :       Cl = 0   (no measurable lift below ~30k)
	//   0.3 < Re_x_e5 < 0.5 :  smoothstep ramp toward Cl_50k
	//   0.5 <= Re_x_e5 <= 0.7: lerp between adjacent bins {50k, 60k, 65k, 70k}
	//   Re_x_e5 > 0.7 :        Hill saturation Cl = ClMax · S·g / (1 + S·g)

	/// Reynolds-bin sentinels (Re_x_e5 = Re / 1e5).
	static constexpr float RE_BIN_NO_LIFT_X_E5 = 0.3F;   ///< below: Cl = 0
	static constexpr float RE_BIN_LOW_X_E5     = 0.5F;   ///< Re = 50,000
	static constexpr float RE_BIN_MID_LOW_X_E5 = 0.6F;   ///< Re = 60,000
	static constexpr float RE_BIN_MID_HIGH_X_E5 = 0.65F; ///< Re = 65,000
	static constexpr float RE_BIN_HIGH_X_E5    = 0.7F;   ///< Re = 70,000

	/// Cl(S) cubic at Re = 50,000 (fits Bearman 50k bin).
	static constexpr float CL_RE50K_A0 =  0.0472121F;
	static constexpr float CL_RE50K_A1 =  2.84795F;
	static constexpr float CL_RE50K_A2 = -23.4342F;
	static constexpr float CL_RE50K_A3 =  45.4849F;

	/// Cl(S) quadratic at Re = 60,000.
	static constexpr float CL_RE60K_A0 =  0.320524F;
	static constexpr float CL_RE60K_A1 = -4.7032F;
	static constexpr float CL_RE60K_A2 = 14.0613F;

	/// Cl(S) quadratic at Re = 65,000.
	static constexpr float CL_RE65K_A0 =  0.266667F;
	static constexpr float CL_RE65K_A1 = -4.0F;
	static constexpr float CL_RE65K_A2 = 13.3333F;

	/// Cl(S) quadratic at Re = 70,000.
	static constexpr float CL_RE70K_A0 =  0.0496189F;
	static constexpr float CL_RE70K_A1 =  0.00211396F;
	static constexpr float CL_RE70K_A2 =  2.34201F;

	/// Spin-factor-dependent maximum lift coefficient.
	/// Cap on bin output and Hill saturation. ClMax(S) is a piecewise-linear
	/// lerp:
	///   S ≤ CL_MAX_SR_LERP_LOW  : ClMax = CL_MAX_BASE   (0.268)
	///   S ≥ CL_MAX_SR_LERP_HIGH : ClMax = CL_MAX_HIGH_SR (0.320)
	///   between                 : linear interpolation
	static constexpr float CL_MAX_BASE        = 0.268F;
	static constexpr float CL_MAX_HIGH_SR     = 0.320F;
	static constexpr float CL_MAX_SR_LERP_LOW  = 0.35F;
	static constexpr float CL_MAX_SR_LERP_HIGH = 0.50F;

	/// Spin gain in the high-Re Hill saturation Cl = ClMax·S·g / (1 + S·g).
	static constexpr float HIGH_RE_SPIN_GAIN = 16.0F;

	[[nodiscard]] Vector3D computeAcceleration(const AerodynamicState &state) const override
	{
		// Wind-relative velocity
		const float vRelX = state.velocity[0] - state.windVelocity[0];
		const float vRelY = state.velocity[1] - state.windVelocity[1];
		const float vRelZ = state.velocity[2] - state.windVelocity[2];
		const double vw = std::sqrt(static_cast<double>(vRelX * vRelX + vRelY * vRelY + vRelZ * vRelZ));

		if (vw < static_cast<double>(physics_constants::MIN_VELOCITY_THRESHOLD))
		{
			return {0.0F, 0.0F, 0.0F};
		}

		const double vwMph = vw / static_cast<double>(physics_constants::MPH_TO_FT_PER_S);
		const double Re_x_e5 = (vwMph / static_cast<double>(RE_VELOCITY_DIVISOR)) *
							   static_cast<double>(state.re100) *
							   static_cast<double>(RE_SCALE_FACTOR);

		// Spin factor S = omega * r / vw
		const double omegaX = static_cast<double>(state.spinVector[0]);
		const double omegaY = static_cast<double>(state.spinVector[1]);
		const double omegaZ = static_cast<double>(state.spinVector[2]);
		const double omegaMag = std::sqrt(omegaX * omegaX + omegaY * omegaY + omegaZ * omegaZ);
		const double spinFactor = omegaMag * static_cast<double>(state.ballRadius) / vw;

		const double Cd = computeCd(Re_x_e5, spinFactor);
		const double Cl = computeCl(Re_x_e5, spinFactor);

		// Drag: -C0 * Cd * vw * vRel
		const double dragScale = -static_cast<double>(state.c0) * Cd * vw;
		const float dragX = static_cast<float>(dragScale * vRelX);
		const float dragY = static_cast<float>(dragScale * vRelY);
		const float dragZ = static_cast<float>(dragScale * vRelZ);

		// Magnus: C0 * (Cl / omega) * vw * (spinVector × vRel)
		float magnusX = 0.0F, magnusY = 0.0F, magnusZ = 0.0F;
		if (omegaMag > static_cast<double>(physics_constants::MIN_VELOCITY_THRESHOLD))
		{
			const double magnusScale = static_cast<double>(state.c0) * (Cl / omegaMag) * vw;
			magnusX = static_cast<float>(magnusScale * (omegaY * vRelZ - omegaZ * vRelY));
			magnusY = static_cast<float>(magnusScale * (omegaZ * vRelX - omegaX * vRelZ));
			magnusZ = static_cast<float>(magnusScale * (omegaX * vRelY - omegaY * vRelX));
		}

		return {dragX + magnusX, dragY + magnusY, dragZ + magnusZ};
	}

	[[nodiscard]] float computeSpinDecayTau(const AerodynamicState &state) const override
	{
		const float v = math_utils::magnitude(state.velocity);
		return 1.0F / (TAU_COEFF * v / state.ballRadius);
	}

	[[nodiscard]] double computeCd(double Re_x_e5, double spinFactor) const
	{
		const double cdLow  = static_cast<double>(CD_LOW);
		const double cdHigh = static_cast<double>(CD_HIGH);
		const double reLow  = static_cast<double>(RE_THRESHOLD_LOW);
		const double reHigh = static_cast<double>(RE_THRESHOLD_HIGH);
		const double cdSpin = static_cast<double>(CD_SPIN);

		if (Re_x_e5 <= reLow)
		{
			return cdLow;
		}
		else if (Re_x_e5 < reHigh)
		{
			return cdLow -
				   (cdLow - cdHigh) * (Re_x_e5 - reLow) / (reHigh - reLow) +
				   cdSpin * spinFactor;
		}
		else
		{
			return cdHigh + cdSpin * spinFactor;
		}
	}

	[[nodiscard]] double computeCl(double Re_x_e5, double spinFactor) const
	{
		const double S = std::max(0.0, spinFactor);
		if (S <= 0.0)
		{
			return 0.0;
		}

		const double clMax = clMaxForSpinFactor(S);
		const double reNoLift = static_cast<double>(RE_BIN_NO_LIFT_X_E5);
		const double reLow = static_cast<double>(RE_BIN_LOW_X_E5);
		const double reMidLow = static_cast<double>(RE_BIN_MID_LOW_X_E5);
		const double reMidHigh = static_cast<double>(RE_BIN_MID_HIGH_X_E5);
		const double reHigh = static_cast<double>(RE_BIN_HIGH_X_E5);

		if (Re_x_e5 <= reNoLift)
		{
			return 0.0;
		}

		if (Re_x_e5 < reLow)
		{
			const double t = smoothStep01((Re_x_e5 - reNoLift) / (reLow - reNoLift));
			return std::clamp(clRe50k(S) * t, 0.0, clMax);
		}

		if (Re_x_e5 >= reHigh)
		{
			const double g = static_cast<double>(HIGH_RE_SPIN_GAIN);
			return std::clamp(clMax * S * g / (1.0 + S * g), 0.0, clMax);
		}

		// Bin lerp across {50k, 60k, 65k, 70k}.
		double reA, reB, clA, clB;
		if (Re_x_e5 < reMidLow)
		{
			reA = reLow; reB = reMidLow;
			clA = clRe50k(S); clB = clRe60k(S);
		}
		else if (Re_x_e5 < reMidHigh)
		{
			reA = reMidLow; reB = reMidHigh;
			clA = clRe60k(S); clB = clRe65k(S);
		}
		else
		{
			reA = reMidHigh; reB = reHigh;
			clA = clRe65k(S); clB = clRe70k(S);
		}
		const double w = (Re_x_e5 - reA) / (reB - reA);
		const double cl = clA + (clB - clA) * w;
		return std::clamp(cl, 0.0, clMax);
	}

private:
	static double smoothStep01(double x)
	{
		const double t = std::clamp(x, 0.0, 1.0);
		return t * t * (3.0 - 2.0 * t);
	}

	// ClMax lerps from CL_MAX_BASE up to CL_MAX_HIGH_SR across the spin-factor
	// band [CL_MAX_SR_LERP_LOW, CL_MAX_SR_LERP_HIGH]. Saturates outside.
	static double clMaxForSpinFactor(double S)
	{
		const double base = static_cast<double>(CL_MAX_BASE);
		const double high = static_cast<double>(CL_MAX_HIGH_SR);
		const double sLow = static_cast<double>(CL_MAX_SR_LERP_LOW);
		const double sHigh = static_cast<double>(CL_MAX_SR_LERP_HIGH);
		if (S <= sLow) return base;
		if (S >= sHigh) return high;
		const double t = (S - sLow) / (sHigh - sLow);
		return base + (high - base) * t;
	}

	static double clRe50k(double S)
	{
		return static_cast<double>(CL_RE50K_A0) +
			   static_cast<double>(CL_RE50K_A1) * S +
			   static_cast<double>(CL_RE50K_A2) * S * S +
			   static_cast<double>(CL_RE50K_A3) * S * S * S;
	}

	static double clRe60k(double S)
	{
		return static_cast<double>(CL_RE60K_A0) +
			   static_cast<double>(CL_RE60K_A1) * S +
			   static_cast<double>(CL_RE60K_A2) * S * S;
	}

	static double clRe65k(double S)
	{
		return static_cast<double>(CL_RE65K_A0) +
			   static_cast<double>(CL_RE65K_A1) * S +
			   static_cast<double>(CL_RE65K_A2) * S * S;
	}

	static double clRe70k(double S)
	{
		return static_cast<double>(CL_RE70K_A0) +
			   static_cast<double>(CL_RE70K_A1) * S +
			   static_cast<double>(CL_RE70K_A2) * S * S;
	}
};

#endif // DEFAULT_AERODYNAMIC_MODEL_HPP
