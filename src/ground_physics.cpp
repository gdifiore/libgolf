/**
 * @file ground_physics.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of ground interaction physics.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "ground_physics.hpp"
#include "physics_constants.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace GroundPhysics
{
    Vector3D calculateRollAcceleration(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        [[maybe_unused]] float spinRate,
        const GroundSurface& surface)
    {
        Vector3D acceleration = {0.0F, 0.0F, 0.0F};

        // Get horizontal velocity magnitude
        float vHorizontal = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);

        // Zero or near-zero velocity: no rolling acceleration applied
        // This is physically correct as friction direction is undefined when stationary
        // and gravity component along slope is balanced by static friction at rest
        if (vHorizontal < physics_constants::MIN_VELOCITY_THRESHOLD)
        {
            return acceleration;  // {0.0F, 0.0F, 0.0F}
        }

        // Calculate slope angle from normal
        // cos(θ) = n · [0,0,1] = n_z
        float cosTheta = surfaceNormal[2];

        // For very flat surfaces, use simplified calculation
        if (cosTheta > physics_constants::FLAT_SURFACE_THRESHOLD)
        {
            // Nearly flat surface: only rolling friction opposes motion
            float deceleration = surface.frictionDynamic * physics_constants::GRAVITY_FT_PER_S2;
            acceleration[0] = -deceleration * (velocity[0] / vHorizontal);
            acceleration[1] = -deceleration * (velocity[1] / vHorizontal);
            acceleration[2] = 0.0F;
            return acceleration;
        }

        // Calculate gravity vector
        Vector3D gravity = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};

        // Decompose gravity into normal and tangent components
        float gravityDotNormal = math_utils::dot(gravity, surfaceNormal);
        Vector3D gravityNormal = {
            surfaceNormal[0] * gravityDotNormal,
            surfaceNormal[1] * gravityDotNormal,
            surfaceNormal[2] * gravityDotNormal
        };

        Vector3D gravityTangent = {
            gravity[0] - gravityNormal[0],
            gravity[1] - gravityNormal[1],
            gravity[2] - gravityNormal[2]
        };

        // Gravity component along slope (down-slope acceleration)
        acceleration[0] = gravityTangent[0];
        acceleration[1] = gravityTangent[1];
        acceleration[2] = gravityTangent[2];

        // Add rolling friction (opposes motion)
        float normalForce = std::abs(gravityDotNormal);
        float frictionDeceleration = surface.frictionDynamic * normalForce;

        acceleration[0] -= frictionDeceleration * (velocity[0] / vHorizontal);
        acceleration[1] -= frictionDeceleration * (velocity[1] / vHorizontal);

        return acceleration;
    }

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
