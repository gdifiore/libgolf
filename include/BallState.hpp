#ifndef BALLSTATE_HPP
#define BALLSTATE_HPP

#include "math_utils.hpp"
#include "physics_constants.hpp"
#include <cmath>

/**
 * @brief Represents the physics state of a golf ball at a specific point in time.
 *
 * This class encapsulates the essential state variables needed to track a golf ball's
 * position, velocity, and acceleration during flight, bounce, or roll phases.
 * Separating state from calculations makes it easier to transition between different
 * physics phases while maintaining consistent state data.
 *
 * Coordinate system:
 *   x-axis: Lateral direction (positive = right of target line)
 *   y-axis: Forward/downrange direction (direction=0 points along +y)
 *   z-axis: Vertical/height (positive = up)
 */
class BallState
{
public:
	/**
	 * @brief Constructs a BallState with zero-initialized values.
	 */
	BallState() : position{0.0F, 0.0F, 0.0F},
				  velocity{0.0F, 0.0F, 0.0F},
				  acceleration{0.0F, 0.0F, 0.0F},
				  currentTime(0.0F),
				  spinVector{0.0F, 0.0F, 0.0F} {}

	/**
	 * @brief Constructs a BallState with specified initial values.
	 *
	 * @param pos Initial position vector
	 * @param vel Initial velocity vector
	 * @param accel Initial acceleration vector
	 * @param time Initial time value
	 * @param spin Initial spin angular velocity (rad/s; default: zero vector)
	 */
	BallState(const Vector3D &pos, const Vector3D &vel, const Vector3D &accel, float time,
	          Vector3D spin = {0.0F, 0.0F, 0.0F})
		: position(pos), velocity(vel), acceleration(accel), currentTime(time), spinVector(spin) {}

	/**
	 * @brief Creates a BallState from launch parameters with sensible defaults.
	 *
	 * This factory method calculates the initial velocity vector from launch speed,
	 * angle, and direction, while providing sensible defaults for position (origin),
	 * acceleration (earth gravity), and time (0).
	 *
	 * @param speed_fps Exit speed in feet per second
	 * @param launch_angle_deg Launch angle in degrees
	 * @param direction_deg Direction angle in degrees (0 = straight, positive = right)
	 * @param start_pos Starting position (default: origin)
	 * @param gravity Gravitational acceleration in ft/s² (default: earth gravity)
	 * @param initialSpinVector Initial spin angular velocity vector (rad/s; default: zero)
	 * @return BallState initialized with calculated velocity and default values
	 */
	static BallState fromLaunchParameters(
		float speed_fps,
		float launch_angle_deg,
		float direction_deg,
		const Vector3D &start_pos = Vector3D{0.0F, 0.0F, 0.0F},
		float gravity = physics_constants::GRAVITY_FT_PER_S2,
		Vector3D initialSpinVector = {0.0F, 0.0F, 0.0F})
	{
		const float theta_rad = launch_angle_deg * physics_constants::DEG_TO_RAD;
		const float phi_rad = direction_deg * physics_constants::DEG_TO_RAD;

		// Coordinate system: x=lateral, y=forward, z=height
		// direction=0 (straight) should give vy=max, vx=0
		// direction>0 (right) should give vx>0
		Vector3D velocity{
			speed_fps * std::cos(theta_rad) * std::sin(phi_rad), // vx: lateral (right)
			speed_fps * std::cos(theta_rad) * std::cos(phi_rad), // vy: forward (downrange)
			speed_fps * std::sin(theta_rad)						 // vz: vertical (up)
		};

		Vector3D acceleration{0.0F, 0.0F, -gravity};

		return BallState(start_pos, velocity, acceleration, 0.0F, initialSpinVector);
	}

	// State variables
	Vector3D position;	   // Current position (x=lateral, y=forward, z=height) in feet
	Vector3D velocity;	   // Current velocity (vx, vy, vz) in ft/s
	Vector3D acceleration; // Current acceleration (ax, ay, az) in ft/s²
	float currentTime;	   // Current simulation time in seconds
	Vector3D spinVector;   // Current spin angular velocity (rad/s); direction = spin axis, magnitude decays over time
};

#endif // BALLSTATE_HPP
