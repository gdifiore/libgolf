/**
 * @file bindings.cpp
 * @brief Emscripten bindings exposing libgolf to JavaScript via Embind.
 */

#include <emscripten/bind.h>

#include "FlightSimulator.hpp"
#include "atmospheric_data.hpp"
#include "ground_surface.hpp"
#include "launch_data.hpp"
#include "physics_constants.hpp"

#include <vector>

using namespace emscripten;

/**
 * Run a full ball flight simulation and return the trajectory as a flat array.
 *
 * @return Flat array of [x, y, z, x, y, z, ...] positions in yards.
 *         x = lateral (right = positive)
 *         y = downrange (forward = positive)
 *         z = height (up = positive)
 *
 * The caller must call .delete() on the returned VectorFloat to free memory.
 */
std::vector<float> runTrajectory(
    float ballSpeedMph,
    float launchAngleDeg,
    float directionDeg,
    float backspinRpm,
    float sidespinRpm,
    float windSpeedMph,
    float windDirDeg)
{
    LaunchData launch{
        .ballSpeedMph = ballSpeedMph,
        .launchAngleDeg = launchAngleDeg,
        .directionDeg = directionDeg,
        .backspinRpm = backspinRpm,
        .sidespinRpm = sidespinRpm,
    };

    // Standard sea-level atmosphere with caller-supplied wind
    AtmosphericData atmos{
        .temp = 70.0f,
        .elevation = 0.0f,
        .vWind = windSpeedMph,
        .phiWind = windDirDeg,
        .hWind = 0.0f,
        .relHumidity = 50.0f,
        .pressure = 29.92f,
    };

    GroundSurface ground; // default fairway values

    FlightSimulator sim(launch, atmos, ground);
    const auto trajectory = sim.runAndGetTrajectory();

    std::vector<float> result;
    result.reserve(trajectory.size() * 3);

    for (const auto &state : trajectory)
    {
        result.push_back(state.position[0] / physics_constants::YARDS_TO_FEET);
        result.push_back(state.position[1] / physics_constants::YARDS_TO_FEET);
        result.push_back(state.position[2] / physics_constants::YARDS_TO_FEET);
    }

    return result;
}

EMSCRIPTEN_BINDINGS(libgolf)
{
    function("runTrajectory", &runTrajectory);
    register_vector<float>("VectorFloat");
}
