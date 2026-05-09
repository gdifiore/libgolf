#ifndef ROLL_MODEL_HPP
#define ROLL_MODEL_HPP

#include "ground_surface.hpp"
#include "math_utils.hpp"

class TerrainInterface;

/**
 * @brief Physical state snapshot passed to the roll model on each step.
 *
 * All quantities are in imperial units consistent with the rest of the
 * library (feet, seconds, ft/s, rad/s).
 *
 * Unlike `BounceModel`, the roll interface advances state by `dt` rather
 * than returning a force or acceleration. Roll is an integration over
 * many small steps, not a single impulse — handing the model the
 * timestep gives it freedom over its integrator (e.g. semi-implicit
 * Euler vs. RK4) and its own stopping criterion.
 */
struct RollState
{
	Vector3D position;       ///< Pre-step position (ft)
	Vector3D velocity;       ///< Pre-step velocity (ft/s)
	Vector3D spinVector;     ///< Pre-step spin vector (rad/s)
	Vector3D surfaceNormal;  ///< Unit normal of the surface at the ball position
	float    ballRadius;     ///< Ball radius (ft)
	float    dt;             ///< Time step duration (s)

	/**
	 * @brief Optional terrain handle for sub-step re-sampling.
	 *
	 * `surfaceNormal` is a single snapshot at the pre-step position. A
	 * model that wants to re-query the surface mid-step (e.g. a putting
	 * model crossing a slope inflection within one tick) can use this
	 * pointer. May be null when `RollState` is constructed outside the
	 * simulator (tests, isolated benchmarks). The default model ignores
	 * it.
	 */
	const TerrainInterface* terrain = nullptr;
};

/**
 * @brief Result returned by a roll model after one step.
 *
 * `atRest` is what `RollPhase::isPhaseComplete` reads — the model owns
 * the stop decision so custom implementations can pick their own cutoff
 * (energy threshold, static-friction balance, etc).
 */
struct RollResult
{
	Vector3D newPosition;    ///< Post-step position (ft)
	Vector3D newVelocity;    ///< Post-step velocity (ft/s)
	Vector3D newSpinVector;  ///< Post-step spin vector (rad/s)
	bool     atRest;         ///< True if the ball has come to rest
};

/**
 * @brief Abstract interface for ball-on-ground roll models.
 *
 * Implement this interface to replace the built-in roll physics. The
 * library invokes `step` once per simulation tick during the roll phase.
 *
 * The model owns its integrator, friction law, spin decay, and stopping
 * criterion. The shared library header `physics_constants.hpp` exposes
 * only universal physics — anything model-specific (rolling resistance
 * curves, grain effects, green firmness coupling) belongs in the model.
 *
 * @code
 * class MyRollModel : public RollModel {
 * public:
 *     RollResult step(const RollState& s,
 *                     const GroundSurface& surface) const override { ... }
 * };
 * FlightSimulator sim(launch, atmos, ground, aero, bounce,
 *                     std::make_shared<MyRollModel>());
 * @endcode
 */
class RollModel
{
public:
	virtual ~RollModel() = default;

	RollModel(const RollModel &) = delete;
	RollModel &operator=(const RollModel &) = delete;
	RollModel(RollModel &&) = delete;
	RollModel &operator=(RollModel &&) = delete;

	/**
	 * @brief Advances rolling state by one timestep.
	 *
	 * @param state   Pre-step kinematic snapshot (includes dt)
	 * @param surface Surface properties at the ball position
	 * @return        Post-step position, velocity, spin, and atRest flag
	 */
	[[nodiscard]] virtual RollResult step(const RollState &state,
	                                      const GroundSurface &surface) const = 0;

protected:
	RollModel() = default;
};

#endif // ROLL_MODEL_HPP
