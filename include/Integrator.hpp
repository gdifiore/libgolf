#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include "BallState.hpp"
#include "math_utils.hpp"

#include <functional>

/**
 * @brief Kinematic sample the integrator evaluates the acceleration field at.
 *
 * A trial position/velocity/time that a higher-order or adaptive scheme may
 * probe between the start and end of a step.
 */
struct IntegratorState
{
	Vector3D position; ///< Trial position (ft)
	Vector3D velocity; ///< Trial velocity (ft/s)
	float    time;     ///< Trial simulation time (s)
};

/**
 * @brief Total acceleration field (aerodynamic + gravity, ft/s²) for the flight
 *        phases, evaluated at an arbitrary trial state.
 */
using AccelerationField = std::function<Vector3D(const IntegratorState &)>;

/**
 * @brief Pluggable time integrator for the aerial and between-bounce flight.
 *
 * Implement this to replace the built-in semi-implicit Euler scheme with, e.g.,
 * RK4 or an adaptive step. The flight phase owns spin decay, wind, and the
 * acceleration model; the integrator owns only how position and velocity
 * advance over a step.
 *
 * `step` receives the current ball state — with `state.acceleration` already
 * holding the acceleration at the start of the step — and an @ref
 * AccelerationField the implementation may sample at trial states. It must
 * update `state.position` and `state.velocity` in place; the phase recomputes
 * the start-of-step acceleration for the next step afterwards.
 *
 * @code
 * class ForwardEuler : public Integrator {
 * public:
 *     void step(BallState& s, float dt, const AccelerationField&) const override {
 *         const Vector3D a = s.acceleration;
 *         for (int i = 0; i < 3; ++i) {
 *             s.position[i] += s.velocity[i] * dt;
 *             s.velocity[i] += a[i] * dt;
 *         }
 *     }
 * };
 * FlightSimulator sim(launch, atmos, ground, nullptr, nullptr, nullptr,
 *                     BallProperties{}, physics_constants::GRAVITY_FT_PER_S2,
 *                     std::make_shared<ForwardEuler>());
 * @endcode
 */
class Integrator
{
public:
	virtual ~Integrator() = default;

	Integrator(const Integrator &) = delete;
	Integrator &operator=(const Integrator &) = delete;
	Integrator(Integrator &&) = delete;
	Integrator &operator=(Integrator &&) = delete;

	/**
	 * @brief Advances position and velocity by one step of size dt.
	 *
	 * @param state Ball state to update; `state.acceleration` is the start-of-step value
	 * @param dt Time step (s)
	 * @param accel Acceleration field sampleable at trial states
	 */
	virtual void step(BallState &state, float dt, const AccelerationField &accel) const = 0;

protected:
	Integrator() = default;
};

#endif // INTEGRATOR_HPP
