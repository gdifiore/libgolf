/**
 * @file bindings.cpp
 * @brief Emscripten bindings exposing libgolf to JavaScript via Embind.
 *
 * The JavaScript API mirrors the C++ public structs (LaunchData,
 * AtmosphericData, GroundSurface) as Embind value_objects so callers can pass
 * plain JS object literals. runShot() returns a ShotResult containing the full
 * trajectory plus carry/total/apex statistics derived in C++.
 */

#include <emscripten/bind.h>

#include "FlightSimulator.hpp"
#include "atmospheric_data.hpp"
#include "ground_surface.hpp"
#include "launch_data.hpp"
#include "physics_constants.hpp"

#include <algorithm>
#include <vector>

using namespace emscripten;

/**
 * Result of a shot simulation, returned to JavaScript as a plain object.
 *
 * trajectory: Flat [x, y, z, x, y, z, ...] array of positions in YARDS
 *   x = lateral (right = positive)
 *   y = downrange (forward = positive)
 *   z = height (up = positive)
 *
 * Indices and scalar stats are pre-computed in C++ so the JS layer doesn't
 * have to reproduce phase-detection logic.
 */
struct ShotResult
{
    std::vector<float> trajectory;
    int carryIndex = 0;          // index of first ground contact (point count, not float count)
    float carryYards = 0.0F;     // downrange distance at first ground contact
    float totalYards = 0.0F;     // downrange distance at rest
    float apexYards = 0.0F;      // peak height above ground
    float offlineYards = 0.0F;   // lateral position at rest (right = +)
    float timeOfFlight = 0.0F;   // total simulation time, seconds
    float bearingDeg = 0.0F;     // bearing from launch to rest, degrees
};

ShotResult runShot(const LaunchData &launch,
                   const AtmosphericData &atmos,
                   const GroundSurface &ground)
{
    FlightSimulator sim(launch, atmos, ground);
    const auto trajectory = sim.runAndGetTrajectory();
    const auto landing = sim.getLandingResult();

    ShotResult out;
    out.trajectory.reserve(trajectory.size() * 3);

    constexpr float FT_PER_YD = physics_constants::YARDS_TO_FEET;

    // -ffast-math is on, so std::numeric_limits<float>::infinity() is UB.
    // Track first iteration via index instead of a sentinel.
    int apexIdx = 0;
    float apexFt = 0.0F;
    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        const auto &p = trajectory[i].position;
        out.trajectory.push_back(p[0] / FT_PER_YD);
        out.trajectory.push_back(p[1] / FT_PER_YD);
        out.trajectory.push_back(p[2] / FT_PER_YD);
        if (i == 0 || p[2] > apexFt)
        {
            apexFt = p[2];
            apexIdx = static_cast<int>(i);
        }
    }

    // Carry = first state after apex where height drops to (or below) ground.
    // ground.height is in feet; allow a small tolerance for the discrete dt.
    int carryIdx = static_cast<int>(trajectory.size()) - 1;
    const float groundFt = ground.height + 0.05F;
    for (size_t i = static_cast<size_t>(apexIdx); i < trajectory.size(); ++i)
    {
        if (trajectory[i].position[2] <= groundFt)
        {
            carryIdx = static_cast<int>(i);
            break;
        }
    }

    out.carryIndex = carryIdx;
    out.carryYards = trajectory.empty() ? 0.0F : trajectory[carryIdx].position[1] / FT_PER_YD;
    out.apexYards = std::max(0.0F, apexFt) / FT_PER_YD;
    out.totalYards = landing.yF;       // landing fields already in yards
    out.offlineYards = landing.xF;
    out.timeOfFlight = landing.timeOfFlight;
    out.bearingDeg = landing.bearing;

    return out;
}

EMSCRIPTEN_BINDINGS(libgolf)
{
    register_vector<float>("VectorFloat");

    value_object<LaunchData>("LaunchData")
        .field("ballSpeedMph", &LaunchData::ballSpeedMph)
        .field("launchAngleDeg", &LaunchData::launchAngleDeg)
        .field("directionDeg", &LaunchData::directionDeg)
        .field("backspinRpm", &LaunchData::backspinRpm)
        .field("sidespinRpm", &LaunchData::sidespinRpm)
        .field("startX", &LaunchData::startX)
        .field("startY", &LaunchData::startY)
        .field("startZ", &LaunchData::startZ);

    value_object<AtmosphericData>("AtmosphericData")
        .field("temp", &AtmosphericData::temp)
        .field("elevation", &AtmosphericData::elevation)
        .field("vWind", &AtmosphericData::vWind)
        .field("phiWind", &AtmosphericData::phiWind)
        .field("hWind", &AtmosphericData::hWind)
        .field("relHumidity", &AtmosphericData::relHumidity)
        .field("pressure", &AtmosphericData::pressure);

    value_object<GroundSurface>("GroundSurface")
        .field("height", &GroundSurface::height)
        .field("restitution", &GroundSurface::restitution)
        .field("frictionStatic", &GroundSurface::frictionStatic)
        .field("frictionDynamic", &GroundSurface::frictionDynamic)
        .field("firmness", &GroundSurface::firmness)
        .field("spinRetention", &GroundSurface::spinRetention)
        .field("criticalAngle", &GroundSurface::criticalAngle);

    value_object<ShotResult>("ShotResult")
        .field("trajectory", &ShotResult::trajectory)
        .field("carryIndex", &ShotResult::carryIndex)
        .field("carryYards", &ShotResult::carryYards)
        .field("totalYards", &ShotResult::totalYards)
        .field("apexYards", &ShotResult::apexYards)
        .field("offlineYards", &ShotResult::offlineYards)
        .field("timeOfFlight", &ShotResult::timeOfFlight)
        .field("bearingDeg", &ShotResult::bearingDeg);

    function("runShot", &runShot);
}
