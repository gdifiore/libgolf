#ifndef ATMOSPHERE_HPP
#define ATMOSPHERE_HPP

/**
 * @brief Atmospheric conditions for flight simulation
 */
struct AtmosphericData
{
	/**
	 * @brief Air temperature (in degrees Fahrenheit).
	 *
	 * Affects air density and ball flight.
	 * Typical range: 0-120°F (Earth conditions)
	 */
	float temp;

	/**
	 * @brief Elevation above sea level (in feet).
	 *
	 * Higher elevations have lower air density.
	 * Typical range: -500 to 15000 ft (most golf courses 0-8000 ft)
	 */
	float elevation;

	/**
	 * @brief Wind speed (in mph).
	 *
	 * Magnitude of wind velocity.
	 * Typical range: 0-40 mph
	 */
	float vWind;

	/**
	 * @brief Wind direction (in degrees).
	 *
	 * Angle relative to Y-axis (target line).
	 * - 0 deg = crosswind from left
	 * - 45 deg = right-forward wind
	 * - 90 deg = headwind
	 * - 180 deg = tailwind
	 * Range: -180 to 180 deg
	 */
	float phiWind;

	/**
	 * @brief Height at which wind acts (in feet).
	 *
	 * Wind affects ball above this altitude.
	 */
	float hWind;

	/**
	 * @brief Relative humidity (in percent).
	 *
	 * Affects air density slightly.
	 * Range: 0-100%
	 */
	float relHumidity;

	/**
	 * @brief Barometric pressure (in inches of mercury).
	 *
	 * Standard sea level pressure is 29.92 inHg.
	 * Typical range: 28-31 inHg
	 */
	float pressure;
};

#endif // ATMOSPHERE_HPP