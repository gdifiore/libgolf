#ifndef GOLF_BALL_HPP
#define GOLF_BALL_HPP

/**
 * @brief Launch parameters from a launch monitor or sensor.
 *
 * All fields match standard launch monitor output directly.
 * Position fields are in feet to match the internal coordinate system.
 *
 * Coordinate system:
 *   x-axis: Lateral direction (positive = right of target line)
 *   y-axis: Forward/downrange direction (0 degrees points along +y)
 *   z-axis: Vertical/height (positive = up)
 */
struct LaunchData
{
	/**
	 * @brief Ball speed at launch (mph).
	 *
	 * Typical values:
	 * - Driver: ~150-180 mph
	 * - Mid iron: ~100-120 mph
	 * - Wedge: ~70-90 mph
	 */
	float ballSpeedMph;

	/**
	 * @brief Vertical launch angle (degrees).
	 *
	 * Angle above horizontal plane.
	 * Typical values:
	 * - Driver: ~10-15 deg
	 * - Mid iron: ~20-30 deg
	 * - Wedge: ~40-50 deg
	 */
	float launchAngleDeg;

	/**
	 * @brief Horizontal launch direction (degrees).
	 *
	 * Angle from target line (0 = straight, + = right, - = left).
	 * Typical range: -45 to +45 deg
	 */
	float directionDeg;

	/**
	 * @brief Backspin rate (rpm).
	 *
	 * Positive values indicate backspin.
	 * Typical values:
	 * - Driver: ~2000-3000 rpm
	 * - Mid iron: ~5000-7000 rpm
	 * - Wedge: ~8000-10000 rpm
	 */
	float backspinRpm;

	/**
	 * @brief Sidespin rate (rpm).
	 *
	 * Positive values produce hook spin, negative values produce slice spin.
	 * Typical range: -3000 to +3000 rpm
	 */
	float sidespinRpm;

	/**
	 * @brief Starting lateral position (feet).
	 *
	 * Positive values are right of target line. Usually 0.
	 */
	float startX = 0.0f;

	/**
	 * @brief Starting downrange position (feet).
	 *
	 * Distance along target line from origin. Usually 0.
	 */
	float startY = 0.0f;

	/**
	 * @brief Starting height above ground (feet).
	 *
	 * Typically 0.0 for ground-level shots.
	 */
	float startZ = 0.0f;
};

/**
 * @brief Final resting position and flight statistics after simulation.
 *
 * Populated by FlightSimulator::getLandingResult() after run() completes.
 */
struct LandingResult
{
	/**
	 * @brief Final lateral position (yards).
	 *
	 * Positive values are right of target line.
	 */
	float xF;

	/**
	 * @brief Final downrange position (yards).
	 */
	float yF;

	/**
	 * @brief Final height above ground (yards).
	 *
	 * Typically 0.0 when ball comes to rest on ground.
	 */
	float zF;

	/**
	 * @brief Total simulation time from launch to rest (seconds).
	 */
	float timeOfFlight;

	/**
	 * @brief Bearing angle from start to landing position (degrees).
	 *
	 * Measured from target line (y-axis). Matches directionDeg convention.
	 */
	float bearing;

	/**
	 * @brief Straight-line distance from launch to final position (yards).
	 */
	float distance;
};

#endif // GOLF_BALL_HPP
