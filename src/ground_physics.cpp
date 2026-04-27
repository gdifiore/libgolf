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
    BounceResult calculateBounce(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float spinRate,
        const GroundSurface& surface)
    {
        BounceResult result;

        // Decompose velocity into normal and tangent components
        // v_normal = (v · n) * n
        // v_tangent = v - v_normal
        float velocityDotNormal = math_utils::dot(velocity, surfaceNormal);
        Vector3D velocityNormal = {
            surfaceNormal[0] * velocityDotNormal,
            surfaceNormal[1] * velocityDotNormal,
            surfaceNormal[2] * velocityDotNormal
        };

        Vector3D velocityTangent = {
            velocity[0] - velocityNormal[0],
            velocity[1] - velocityNormal[1],
            velocity[2] - velocityNormal[2]
        };

        // Effective COR — modulated by spin × normal-impact-speed.
        // High spin causes the ball to bite into turf rather than spring off.
        // The velocity scale prevents low-energy chip shots from getting an
        // unwarranted COR penalty just from carried spin. Reference:
        // openfairway BounceCalculator.cs:215-250.
        float spinRpm =
            std::abs(spinRate) / physics_constants::RPM_TO_RAD_PER_S;
        float speedNormalMs =
            std::abs(velocityDotNormal) * physics_constants::FEET_TO_METERS;

        float velScale;
        if (speedNormalMs < physics_constants::BOUNCE_COR_VEL_LOW_MS)
        {
            velScale = physics_constants::BOUNCE_COR_VEL_MID_SCALE *
                       (speedNormalMs / physics_constants::BOUNCE_COR_VEL_LOW_MS);
        }
        else if (speedNormalMs < physics_constants::BOUNCE_COR_VEL_HIGH_MS)
        {
            float t = (speedNormalMs - physics_constants::BOUNCE_COR_VEL_LOW_MS) /
                      (physics_constants::BOUNCE_COR_VEL_HIGH_MS -
                       physics_constants::BOUNCE_COR_VEL_LOW_MS);
            velScale = physics_constants::BOUNCE_COR_VEL_MID_SCALE +
                       (1.0F - physics_constants::BOUNCE_COR_VEL_MID_SCALE) * t;
        }
        else
        {
            velScale = 1.0F;
        }

        float maxReduction;
        if (spinRpm < physics_constants::BOUNCE_COR_SPIN_KNEE_RPM)
        {
            maxReduction =
                (spinRpm / physics_constants::BOUNCE_COR_SPIN_KNEE_RPM) *
                physics_constants::BOUNCE_COR_SPIN_LOW_MAX_REDUCTION;
        }
        else
        {
            float excess = spinRpm - physics_constants::BOUNCE_COR_SPIN_KNEE_RPM;
            float t = std::min(
                excess / physics_constants::BOUNCE_COR_SPIN_HIGH_BAND_RPM, 1.0F);
            maxReduction =
                physics_constants::BOUNCE_COR_SPIN_LOW_MAX_REDUCTION +
                (physics_constants::BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION -
                 physics_constants::BOUNCE_COR_SPIN_LOW_MAX_REDUCTION) *
                    t;
        }

        float effectiveCor = surface.restitution * (1.0F - maxReduction * velScale);

        // Apply effective COR to normal component
        // v'_normal = -COR * v_normal (reversed direction)
        Vector3D velocityNormalAfter = {
            -effectiveCor * velocityNormal[0],
            -effectiveCor * velocityNormal[1],
            -effectiveCor * velocityNormal[2]
        };

        // Friction retention factor for tangential motion (simple-retention
        // path; the Penner branch uses a spin-coupled retention curve below).
        float frictionFactor = 1.0F - surface.frictionStatic * (1.0F - surface.firmness);
        frictionFactor = std::max(0.0F, std::min(1.0F, frictionFactor));

        float tangentMagnitude = std::sqrt(
            velocityTangent[0] * velocityTangent[0] +
            velocityTangent[1] * velocityTangent[1] +
            velocityTangent[2] * velocityTangent[2]
        );

        // Impact angle measured from the surface plane (not the normal).
        // For a unit normal and v · n < 0 (ball moving into surface):
        //   sin(impactAngle) = -v · n / |v|
        float impactSpeed = std::sqrt(
            velocity[0] * velocity[0] +
            velocity[1] * velocity[1] +
            velocity[2] * velocity[2]
        );
        float impactAngle = 0.0F;
        if (impactSpeed > physics_constants::MIN_VELOCITY_THRESHOLD)
        {
            float sinAngle = std::max(-1.0F, std::min(1.0F, -velocityDotNormal / impactSpeed));
            impactAngle = std::asin(sinAngle);
        }

        bool steepImpact = impactAngle >= surface.criticalAngle;
        bool energeticImpact = impactSpeed >= physics_constants::MIN_PENNER_BOUNCE_SPEED_FT_PER_S;

        Vector3D velocityTangentAfter;

        if (steepImpact && energeticImpact && tangentMagnitude > physics_constants::MIN_VELOCITY_THRESHOLD)
        {
            // Penner tangential model (Penner 2003, eq. for steep wedge bounce):
            //   v_t' = retention * |v| * sin(θ - θ_crit) - 2 R ω / 7
            // Allowed to go negative, which reverses tangent direction
            // (real wedge check / spin-back).
            //
            // Retention is spin-coupled: at high spin the ball loses more
            // forward push to bite, tightening wedge total-distance. Curve
            // matches openfairway: 0.55 * clamp(1 - rpm/8000, 0.4, 1.0).
            float retention =
                physics_constants::BOUNCE_RETENTION_BASE *
                std::max(physics_constants::BOUNCE_RETENTION_FLOOR,
                         std::min(1.0F,
                                  1.0F - spinRpm /
                                             physics_constants::BOUNCE_RETENTION_RPM_NORM));
            float spinbackTerm =
                (2.0F * physics_constants::STD_BALL_RADIUS_FT * spinRate) / 7.0F;
            float newTangentSpeed =
                retention * impactSpeed *
                    std::sin(impactAngle - surface.criticalAngle) -
                spinbackTerm;

            float scale = newTangentSpeed / tangentMagnitude;
            velocityTangentAfter = {
                velocityTangent[0] * scale,
                velocityTangent[1] * scale,
                velocityTangent[2] * scale
            };
        }
        else
        {
            // Shallow or low-energy impact: simple friction retention.
            // No spin coupling — chips and drives cannot spin back.
            velocityTangentAfter = {
                velocityTangent[0] * frictionFactor,
                velocityTangent[1] * frictionFactor,
                velocityTangent[2] * frictionFactor
            };
        }

        // Combine components to get final velocity
        result.newVelocity = {
            velocityNormalAfter[0] + velocityTangentAfter[0],
            velocityNormalAfter[1] + velocityTangentAfter[1],
            velocityNormalAfter[2] + velocityTangentAfter[2]
        };

        // Apply spin retention
        result.newSpinRate = spinRate * surface.spinRetention;

        return result;
    }

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
