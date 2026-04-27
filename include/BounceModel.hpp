#ifndef BOUNCE_MODEL_HPP
#define BOUNCE_MODEL_HPP

#include "ground_surface.hpp"
#include "math_utils.hpp"

/**
 * @brief Physical state snapshot passed to the bounce model on impact.
 *
 * All quantities are in imperial units consistent with the rest of the
 * library (feet, seconds, ft/s, rad/s).
 *
 * The library detects ground impact, computes the surface normal and
 * surface properties from the active terrain, and invokes the bounce
 * model exactly once per impact. The model returns the post-bounce
 * velocity and spin vectors. Spin is passed and returned as a vector
 * (rad/s) so the model can implement direction-dependent effects (lateral
 * spin, axis tilt) without losing information.
 */
struct BounceState
{
	Vector3D velocity;       ///< Pre-bounce velocity (ft/s)
	Vector3D surfaceNormal;  ///< Unit normal of the surface at the impact point (points away from ground)
	Vector3D spinVector;     ///< Pre-bounce spin vector (rad/s)
	float    ballRadius;     ///< Ball radius (ft)
};

/**
 * @brief Result returned by a bounce model.
 *
 * `newSpinVector` may differ in direction as well as magnitude from the
 * incoming spin — useful for models that tilt the axis on impact.
 */
struct BounceResult
{
	Vector3D newVelocity;    ///< Post-bounce velocity (ft/s)
	Vector3D newSpinVector;  ///< Post-bounce spin vector (rad/s)
};

/**
 * @brief Abstract interface for ball-ground bounce models.
 *
 * Implement this interface to replace the built-in bounce physics. The
 * library calls `resolveBounce` exactly once per ground impact during the
 * bounce phase. Between bounces, the standard aerodynamic + gravity
 * integration runs.
 *
 * The model owns its own units and constants. The shared library header
 * `physics_constants.hpp` exposes only universal physics (gravity,
 * geometry, unit conversions) — anything model-specific (friction laws,
 * COR curves, spin coupling) should live in the model implementation.
 *
 * @code
 * class MyBounceModel : public BounceModel {
 * public:
 *     BounceResult resolveBounce(const BounceState& s,
 *                                const GroundSurface& surface) const override { ... }
 * };
 * FlightSimulator sim(launch, atmos, ground, aero, std::make_shared<MyBounceModel>());
 * @endcode
 */
class BounceModel
{
public:
	virtual ~BounceModel() = default;

	BounceModel(const BounceModel &) = delete;
	BounceModel &operator=(const BounceModel &) = delete;
	BounceModel(BounceModel &&) = delete;
	BounceModel &operator=(BounceModel &&) = delete;

	/**
	 * @brief Computes post-bounce velocity and spin.
	 *
	 * @param state   Pre-bounce kinematic snapshot
	 * @param surface Surface properties at the impact point
	 * @return        Post-bounce velocity and spin vectors
	 */
	virtual BounceResult resolveBounce(const BounceState &state,
	                                   const GroundSurface &surface) const = 0;

protected:
	BounceModel() = default;
};

#endif // BOUNCE_MODEL_HPP
