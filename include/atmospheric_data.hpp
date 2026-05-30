#ifndef ATMOSPHERIC_DATA_HPP
#define ATMOSPHERIC_DATA_HPP

/**
 * @brief Atmospheric conditions for flight simulation.
 *
 * Defaults describe a sea-level standard day (59°F, 29.92 inHg, no wind, dry
 * air), so a default-constructed AtmosphericData{} is a usable baseline and
 * designated initializers need only override the fields that differ.
 */
struct AtmosphericData
{
	/**
	 * @brief Air temperature (in degrees Fahrenheit).
	 *
	 * Affects air density and ball flight.
	 * Typical range: 0-120°F (Earth conditions)
	 */
	float temp = 59.0F;

	/**
	 * @brief Elevation above sea level (in feet).
	 *
	 * Higher elevations have lower air density.
	 * Typical range: -500 to 15000 ft (most golf courses 0-8000 ft)
	 */
	float elevation = 0.0F;

	/**
	 * @brief Wind speed (in mph).
	 *
	 * Magnitude of wind velocity.
	 * Typical range: 0-40 mph
	 */
	float vWind = 0.0F;

	/**
	 * @brief Wind direction (in degrees).
	 *
	 * Angle relative to the Y-axis (target line), measured so that the wind
	 * velocity vector is (vWind*sin(phiWind), vWind*cos(phiWind)) in field
	 * coordinates (x = right, y = downrange).
	 *
	 *   phiWind   | wind vector        | effect on the shot
	 *   ----------|--------------------|----------------------------
	 *     0 deg   | (0, +w) downrange  | tailwind
	 *    90 deg   | (+w, 0) rightward  | crosswind, left -> right
	 *   180 deg   | (0, -w)            | headwind
	 *   270/-90   | (-w, 0)            | crosswind, right -> left
	 *
	 * Range: -180 to 180 deg
	 */
	float phiWind = 0.0F;

	/**
	 * @brief Height at which wind acts (in feet).
	 *
	 * Wind affects ball above this altitude.
	 */
	float hWind = 0.0F;

	/**
	 * @brief Relative humidity (in percent).
	 *
	 * Affects air density slightly.
	 * Range: 0-100%
	 */
	float relHumidity = 0.0F;

	/**
	 * @brief Barometric pressure (in inches of mercury).
	 *
	 * Standard sea level pressure is 29.92 inHg.
	 * Typical range: 28-31 inHg
	 */
	float pressure = 29.92F;
};

#endif // ATMOSPHERIC_DATA_HPP