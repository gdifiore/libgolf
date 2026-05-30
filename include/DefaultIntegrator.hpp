#ifndef DEFAULT_INTEGRATOR_HPP
#define DEFAULT_INTEGRATOR_HPP

#include "Integrator.hpp"
#include "physics_constants.hpp"

/**
 * @brief Built-in semi-implicit (symplectic) Euler integrator.
 *
 * Advances on the acceleration from the start of the step:
 *   position += velocity * dt + 0.5 * a * dt²
 *   velocity += a * dt
 *
 * Position carries a 2nd-order term; velocity is forward-Euler (1st-order
 * accurate). This scheme needs only the start-of-step acceleration already in
 * `state.acceleration`, so it does not sample the acceleration field.
 */
class DefaultIntegrator : public Integrator
{
public:
	void step(BallState &state, float dt, const AccelerationField &accel) const override
	{
		(void)accel; // start-of-step acceleration is sufficient for this scheme

		const Vector3D a = state.acceleration;
		for (int i = 0; i < 3; ++i)
		{
			state.position[i] += state.velocity[i] * dt +
			                     physics_constants::HALF * a[i] * dt * dt;
			state.velocity[i] += a[i] * dt;
		}
	}
};

#endif // DEFAULT_INTEGRATOR_HPP
