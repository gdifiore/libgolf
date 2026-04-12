#ifndef AERODYNAMIC_MODEL_HPP
#define AERODYNAMIC_MODEL_HPP

#include "math_utils.hpp"

/**
 * @brief Physical state passed to the aerodynamic model each timestep.
 *
 * All quantities are in imperial units consistent with the rest of the library
 * (feet, seconds, ft/s). `re100` encodes the atmospheric conditions so the
 * model can compute Reynolds number without needing raw air density or viscosity.
 *
 * Reynolds number at any speed:
 *   Re       = (v_mph / 100) * re100
 *   Re_x_e5  = Re / 1e5  =  (v_mph / 100) * re100 * 1e-5
 *
 * Spin factor (dimensionless surface-to-translational speed ratio):
 *   S = |spinVector| * ballRadius / |velocity - windVelocity|
 */
struct AerodynamicState
{
	Vector3D velocity;     ///< Ball velocity (ft/s)
	Vector3D windVelocity; ///< Effective wind velocity at ball height (ft/s; zero below hWind)
	Vector3D spinVector;   ///< Current spin vector (rad/s); direction = launch spin axis, magnitude decays
	float    c0;           ///< Aerodynamic force coefficient (encodes air density and ball geometry)
	float    ballRadius;   ///< Ball radius (ft)
	float    re100;        ///< Reynolds number at 100 mph under current atmospheric conditions
};

/**
 * @brief Abstract interface for aerodynamic force and spin-decay models.
 *
 * Implement this interface to replace the built-in aerodynamics for the aerial
 * flight phase. Two responsibilities:
 *
 *   1. computeAcceleration — return the total aerodynamic acceleration vector
 *      (drag + Magnus + any other forces). Gravity is NOT included; AerialPhase
 *      adds it separately so the model only needs to express aerodynamics.
 *
 *   2. computeSpinDecayTau — return the exponential decay time constant (s).
 *      AerialPhase applies:  spinRate *= exp(-dt / tau)
 *      Return a very large value (e.g. 1e6) to effectively disable spin decay.
 *
 * Both methods are called once per simulation timestep. The AerodynamicState
 * provides the complete physical context: velocity vectors, full spin vector,
 * ball geometry, and the atmospheric Reynolds reference.
 *
 * @code
 * class MyModel : public AerodynamicModel {
 * public:
 *     Vector3D computeAcceleration(const AerodynamicState& s) const override { ... }
 *     double   computeSpinDecayTau(const AerodynamicState& s) const override { ... }
 * };
 * FlightSimulator sim(launch, atmos, ground, std::make_shared<MyModel>());
 * @endcode
 */
class AerodynamicModel
{
public:
	virtual ~AerodynamicModel() = default;

	AerodynamicModel(const AerodynamicModel &) = default;
	AerodynamicModel &operator=(const AerodynamicModel &) = default;
	AerodynamicModel(AerodynamicModel &&) = default;
	AerodynamicModel &operator=(AerodynamicModel &&) = default;

	/**
	 * @brief Computes aerodynamic acceleration (drag + Magnus; gravity excluded).
	 *
	 * @param state Current physical state of the ball and atmosphere
	 * @return Acceleration vector in ft/s². Gravity (-32.174 ft/s² in z) is added
	 *         by AerialPhase; do not include it here.
	 */
	virtual Vector3D computeAcceleration(const AerodynamicState &state) const = 0;

	/**
	 * @brief Computes the spin decay time constant.
	 *
	 * @param state Current physical state (velocity and ballRadius are typically
	 *              sufficient for aerodynamic damping models)
	 * @return Time constant tau (seconds). Larger = slower decay.
	 */
	virtual double computeSpinDecayTau(const AerodynamicState &state) const = 0;

protected:
	AerodynamicModel() = default;
};

#endif // AERODYNAMIC_MODEL_HPP
