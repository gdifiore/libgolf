#ifndef AERODYNAMIC_MODEL_HPP
#define AERODYNAMIC_MODEL_HPP

#include "math_utils.hpp"

/**
 * @brief Physical state snapshot passed to the aerodynamic model each timestep.
 *
 * Imperial units for kinematic fields (feet, seconds, ft/s); SI units for
 * raw atmosphere (kg/m³, Pa·s, K) since those are the natural outputs of
 * Sutherland's law and the standard atmosphere model. Fields fall into four
 * groups:
 *
 *   - Kinematic: `velocity`, `windVelocity`, `spinVector`, `position`, `currentTime`
 *   - Ball geometry: `ballRadius`
 *   - Atmosphere (raw): `airDensityKgPerM3`, `airViscosity`, `tempKelvin`,
 *                       `pressureMmHg`, `relHumidity`
 *   - Atmosphere (lumped): `c0`, `re100`
 *
 * The lumped fields `c0` and `re100` are precomputed convenience scalars for
 * the default `F = c0 * Cd(Re, S) * vw * vRel` form. Models that need
 * altitude-dependent density, Mach corrections, custom humidity coupling, or
 * non-Reynolds-binned drag should consume the raw fields directly.
 *
 * Lumped atmospheric identities (applied by the default model):
 *   Re       = (v_mph / 100) * re100
 *   Re_x_e5  = Re / 1e5  =  (v_mph / 100) * re100 * 1e-5
 *
 * Both lumped fields are computed once at launch from the launch atmosphere
 * and held static for the shot. Models needing altitude-varying lumped
 * scalars must re-derive them per step from the raw fields.
 *
 * Spin factor (dimensionless surface-to-translational speed ratio):
 *   S = |spinVector| * ballRadius / |velocity - windVelocity|
 */
struct AerodynamicState
{
	Vector3D velocity;          ///< Ball velocity (ft/s)
	Vector3D windVelocity;      ///< Effective wind velocity at ball height (ft/s; zero below hWind)
	Vector3D spinVector;        ///< Current spin vector (rad/s); direction = launch spin axis, magnitude decays
	float    c0;                ///< Lumped aerodynamic force coefficient (air density × ball cross-section / mass)
	float    ballRadius;        ///< Ball radius (ft)
	float    re100;             ///< Lumped Reynolds reference: Re at 100 mph under current atmospherics
	float    airDensityKgPerM3 = 0.0F; ///< Raw air density at launch atmosphere (kg/m³)
	float    airViscosity      = 0.0F; ///< Sutherland-law dynamic viscosity (Pa·s = kg/(m·s))
	float    tempKelvin        = 0.0F; ///< Air temperature (K)
	float    pressureMmHg      = 0.0F; ///< Barometric pressure (mmHg)
	float    relHumidity       = 0.0F; ///< Relative humidity (0..100)
	Vector3D position;          ///< Ball position (ft; x=lateral, y=forward, z=height)
	float    currentTime;       ///< Simulation time since launch (s)
};

/**
 * @brief Abstract interface for aerodynamic force and spin-decay models.
 *
 * Implement this interface to replace the built-in aerodynamics for the aerial
 * flight phase. Two responsibilities:
 *
 *   1. computeAcceleration — return the total aerodynamic acceleration vector
 *      (drag + Magnus + any other forces). Gravity is excluded; see method docs.
 *
 *   2. computeSpinDecayTau — return the exponential decay time constant (s).
 *      AerialPhase applies:  spinVector *= exp(-dt / tau)  (each component)
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

	AerodynamicModel(const AerodynamicModel &) = delete;
	AerodynamicModel &operator=(const AerodynamicModel &) = delete;
	AerodynamicModel(AerodynamicModel &&) = delete;
	AerodynamicModel &operator=(AerodynamicModel &&) = delete;

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
