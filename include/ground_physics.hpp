/**
 * @file ground_physics.hpp
 * @author Gabriel DiFiore
 * @brief Ground interaction physics for the bounce/roll transition.
 *
 * Bounce physics lives behind the BounceModel interface (see BounceModel.hpp,
 * DefaultBounceModel.hpp). Roll physics lives behind the RollModel interface
 * (see RollModel.hpp, DefaultRollModel.hpp). This header retains only the
 * bounce-to-roll transition heuristic — a phase-management decision used by
 * BouncePhase, not roll physics.
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
