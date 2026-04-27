/**
 * @file ground_physics.hpp
 * @author Gabriel DiFiore
 * @brief Ground interaction physics for rolling and bounce/roll transition.
 *
 * Bounce physics lives behind the BounceModel interface (see BounceModel.hpp,
 * DefaultBounceModel.hpp). This header retains rolling and the bounce-to-roll
 * transition heuristic, both of which are still free functions used by the
 * library's RollPhase and BouncePhase.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#ifndef GROUND_PHYSICS_HPP
#define GROUND_PHYSICS_HPP

#include "ground_surface.hpp"
#include "math_utils.hpp"

namespace GroundPhysics
{
    /**
     * Calculates the acceleration for a rolling ball on a slope.
     *
     * This function computes the net acceleration from:
     * - Gravity component along the slope
     * - Rolling friction opposing motion
     *
     * @param velocity The current velocity (ft/s).
     * @param surfaceNormal The unit normal vector of the surface (pointing upward).
     * @param spinRate The current spin rate (rad/s).
     * @param surface The ground surface properties (friction, etc.).
     * @return The acceleration vector (ft/s²).
     */
    [[nodiscard]] auto calculateRollAcceleration(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float spinRate,
        const GroundSurface& surface
    ) -> Vector3D;

    /**
     * Determines if the ball should transition from bouncing to rolling.
     *
     * The ball transitions to rolling when it's close to the ground and
     * moving slowly in the vertical direction.
     *
     * @param velocity The current velocity (ft/s).
     * @param surfaceNormal The unit normal vector of the surface (pointing upward).
     * @param heightAboveGround The height above the terrain surface (ft).
     * @return True if the ball should transition to rolling, false otherwise.
     */
    [[nodiscard]] auto shouldTransitionToRoll(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float heightAboveGround
    ) -> bool;

} // namespace GroundPhysics

#endif // GROUND_PHYSICS_HPP
