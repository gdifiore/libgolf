#ifndef DEFAULT_AERODYNAMIC_MODEL_HPP
#define DEFAULT_AERODYNAMIC_MODEL_HPP

#include "AerodynamicModel.hpp"
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
 *   Re > 0.7:        Hill saturation Cl = CL_MAX·S·g / (1 + S·g)
 * All branches clamped to [0, CL_MAX]. Bins are Bearman dimpled-sphere fits.
 *
 * Spin decay:
 *   tau = 1 / (TAU_COEFF * |v| / r)
 *
 * Reference: Washington State University study by Bin Lyu, et al.
 *
 * computeCd() and computeCl() are public non-virtual helpers — useful for
 * inspection and as building blocks in derived models.
 */
class DefaultAerodynamicModel : public AerodynamicModel
{
public:
	Vector3D computeAcceleration(const AerodynamicState &state) const override
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
		const double Re_x_e5 = (vwMph / static_cast<double>(physics_constants::RE_VELOCITY_DIVISOR)) *
							   static_cast<double>(state.re100) *
							   static_cast<double>(physics_constants::RE_SCALE_FACTOR);

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

	double computeSpinDecayTau(const AerodynamicState &state) const override
	{
		const double vx = static_cast<double>(state.velocity[0]);
		const double vy = static_cast<double>(state.velocity[1]);
		const double vz = static_cast<double>(state.velocity[2]);
		const double v = std::sqrt(vx * vx + vy * vy + vz * vz);
		return 1.0 / (static_cast<double>(physics_constants::TAU_COEFF) *
					  v / static_cast<double>(state.ballRadius));
	}

	double computeCd(double Re_x_e5, double spinFactor) const
	{
		const double cdLow  = static_cast<double>(physics_constants::CD_LOW);
		const double cdHigh = static_cast<double>(physics_constants::CD_HIGH);
		const double reLow  = static_cast<double>(physics_constants::RE_THRESHOLD_LOW);
		const double reHigh = static_cast<double>(physics_constants::RE_THRESHOLD_HIGH);
		const double cdSpin = static_cast<double>(physics_constants::CD_SPIN);

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

	double computeCl(double Re_x_e5, double spinFactor) const
	{
		const double S = std::max(0.0, spinFactor);
		if (S <= 0.0)
		{
			return 0.0;
		}

		const double clMax = static_cast<double>(physics_constants::CL_MAX);
		const double reNoLift = static_cast<double>(physics_constants::RE_BIN_NO_LIFT_X_E5);
		const double reLow = static_cast<double>(physics_constants::RE_BIN_LOW_X_E5);
		const double reMidLow = static_cast<double>(physics_constants::RE_BIN_MID_LOW_X_E5);
		const double reMidHigh = static_cast<double>(physics_constants::RE_BIN_MID_HIGH_X_E5);
		const double reHigh = static_cast<double>(physics_constants::RE_BIN_HIGH_X_E5);

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
			const double g = static_cast<double>(physics_constants::HIGH_RE_SPIN_GAIN);
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

	static double clRe50k(double S)
	{
		return static_cast<double>(physics_constants::CL_RE50K_A0) +
			   static_cast<double>(physics_constants::CL_RE50K_A1) * S +
			   static_cast<double>(physics_constants::CL_RE50K_A2) * S * S +
			   static_cast<double>(physics_constants::CL_RE50K_A3) * S * S * S;
	}

	static double clRe60k(double S)
	{
		return static_cast<double>(physics_constants::CL_RE60K_A0) +
			   static_cast<double>(physics_constants::CL_RE60K_A1) * S +
			   static_cast<double>(physics_constants::CL_RE60K_A2) * S * S;
	}

	static double clRe65k(double S)
	{
		return static_cast<double>(physics_constants::CL_RE65K_A0) +
			   static_cast<double>(physics_constants::CL_RE65K_A1) * S +
			   static_cast<double>(physics_constants::CL_RE65K_A2) * S * S;
	}

	static double clRe70k(double S)
	{
		return static_cast<double>(physics_constants::CL_RE70K_A0) +
			   static_cast<double>(physics_constants::CL_RE70K_A1) * S +
			   static_cast<double>(physics_constants::CL_RE70K_A2) * S * S;
	}
};

#endif // DEFAULT_AERODYNAMIC_MODEL_HPP
