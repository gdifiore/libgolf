/**
 * @file ground_physics.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of ground interaction physics.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "ground_physics.hpp"
#include "physics_constants.hpp"

#include <cmath>

namespace GroundPhysics
{
    bool shouldTransitionToRoll(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float heightAboveGround)
    {
        // Check if ball is close to the ground
        if (heightAboveGround > physics_constants::GROUND_CONTACT_THRESHOLD)
        {
            return false;
        }

        // Calculate velocity component normal to surface
        float velocityDotNormal = math_utils::dot(velocity, surfaceNormal);

        // Transition to roll if moving slowly in the normal direction
        return std::abs(velocityDotNormal) < physics_constants::MIN_BOUNCE_VELOCITY;
    }

} // namespace GroundPhysics
