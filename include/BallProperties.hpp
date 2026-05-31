#ifndef BALL_PROPERTIES_HPP
#define BALL_PROPERTIES_HPP

#include "physics_constants.hpp"

/**
 * @brief Physical properties of the ball being simulated.
 *
 * Defaults reproduce a standard golf ball, so a default-constructed
 * BallProperties{} leaves the simulation identical to the historical
 * hardcoded behaviour. Supply custom values to simulate a different ball:
 *
 * @code
 * BallProperties heavy{.massOz = 1.80f, .circumferenceIn = 5.30f};
 * FlightSimulator sim(launch, atmos, ground,
 *                     nullptr, nullptr, nullptr, heavy);
 * @endcode
 *
 * Mass and circumference feed the lumped aerodynamic coefficients (c0, the
 * Reynolds reference, and surface spin speed); the radius the phases hand to
 * the force models is derived from circumference.
 */
struct BallProperties
{
	/// Ball mass (ounces).
	float massOz = physics_constants::STD_BALL_MASS_OZ;

	/// Ball circumference (inches).
	float circumferenceIn = physics_constants::STD_BALL_CIRCUMFERENCE_IN;

	/// Ball radius (feet), derived from circumference.
	[[nodiscard]] float radiusFt() const
	{
		return circumferenceIn / (2.0F * physics_constants::PI) /
		       physics_constants::INCHES_PER_FOOT;
	}
};

#endif // BALL_PROPERTIES_HPP
