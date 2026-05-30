/**
 * @file math_utils.cpp
 * @author Gabriel DiFiore
 * @brief Contains functions for various mathematical utilities.
 *
 * This file contains functions that perform mathematical
 * utilities, e.g. converting between units.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "math_utils.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdexcept>

/**
 * Converts Fahrenheit to Celsius.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Celsius.
 */
float math_utils::convertFahrenheitToCelsius(float fahrenheit)
{
    return (fahrenheit - physics_constants::FAHRENHEIT_OFFSET) * physics_constants::FAHRENHEIT_TO_CELSIUS_SCALE;
}

/**
 * Converts Celsius to Kelvin.
 *
 * @param celsius The temperature in Celsius.
 * @return The temperature in Kelvin.
 */
float math_utils::convertCelsiusToKelvin(float celsius)
{
    return celsius + physics_constants::KELVIN_OFFSET;
}

/**
 * Converts Fahrenheit to Kelvin.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Kelvin.
 */
float math_utils::convertFahrenheitToKelvin(float fahrenheit)
{
    return ((fahrenheit - physics_constants::FAHRENHEIT_OFFSET) * physics_constants::FAHRENHEIT_TO_CELSIUS_SCALE) + physics_constants::KELVIN_OFFSET;
}

/**
 * Converts feet to meters.
 *
 * @param feet The length in feet.
 * @return The length in meters.
 */
float math_utils::convertFeetToMeters(float feet)
{
    return feet * physics_constants::FEET_TO_METERS;
}

/**
 * Converts meters to feet.
 *
 * @param meters The length in meters.
 * @return The length in feet.
 */
float math_utils::convertMetersToFeet(float meters)
{
    return meters * physics_constants::METERS_TO_FEET;
}

/**
 * Calculates the distance in yards from the origin to the given landing position.
 *
 * @param position The position in 3D space.
 * @return The distance in yards.
 */
float math_utils::getDistanceInYards(Vector3D position)
{
    float distance = std::sqrt(position[0] * position[0] + position[1] * position[1]);

    float distanceInYards = distance / physics_constants::YARDS_TO_FEET;

    return distanceInYards;
}

/**
 * Calculates the dot product of two 3D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The dot product (scalar).
 */
float math_utils::dot(const Vector3D& a, const Vector3D& b) noexcept
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Calculates the cross product of two 3D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The cross product vector (perpendicular to both a and b).
 */
Vector3D math_utils::cross(const Vector3D& a, const Vector3D& b) noexcept
{
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

/**
 * Calculates the magnitude (length) of a 3D vector.
 *
 * @param v The vector.
 * @return The magnitude of the vector.
 */
float math_utils::magnitude(const Vector3D& v) noexcept
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * Normalizes a 3D vector to unit length.
 *
 * @param v The vector to normalize.
 * @return The normalized vector (magnitude = 1).
 * @throws std::invalid_argument if the vector has zero magnitude.
 */
Vector3D math_utils::normalize(const Vector3D& v)
{
    float mag = magnitude(v);

    if (mag < physics_constants::MIN_LENGTH)
    {
        throw std::invalid_argument(std::string(__func__) + ": Cannot normalize zero-length vector");
    }

    return {v[0] / mag, v[1] / mag, v[2] / mag};
}

/**
 * Projects vector v onto vector 'onto'.
 *
 * @param v The vector to project.
 * @param onto The vector to project onto.
 * @return The projection of v onto 'onto'.
 * @throws std::invalid_argument if 'onto' has zero magnitude.
 */
Vector3D math_utils::project(const Vector3D& v, const Vector3D& onto)
{
    float ontoMagSquared = dot(onto, onto);

    if (ontoMagSquared < physics_constants::MIN_LENGTH * physics_constants::MIN_LENGTH)
    {
        throw std::invalid_argument(std::string(__func__) + ": Cannot project onto zero-length vector");
    }

    float scale = dot(v, onto) / ontoMagSquared;

    return {onto[0] * scale, onto[1] * scale, onto[2] * scale};
}
