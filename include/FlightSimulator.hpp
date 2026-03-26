#ifndef FLIGHT_SIMULATOR_HPP
#define FLIGHT_SIMULATOR_HPP

#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "GroundProvider.hpp"
#include "atmospheric_data.hpp"
#include "launch_data.hpp"
#include "ground_surface.hpp"
#include "terrain_interface.hpp"

#include <memory>
#include <vector>

/**
 * @brief Manages the complete flight simulation with automatic phase transitions.
 *
 * FlightSimulator encapsulates the complexity of managing different flight phases
 * (aerial, bounce, roll) and handles transitions automatically.
 *
 * Basic usage:
 * @code
 * LaunchData launch{160.0f, 11.0f, 0.0f, 3000.0f, 0.0f};
 * AtmosphericData atmos{70.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 29.92f};
 * GroundSurface ground;
 *
 * FlightSimulator sim(launch, atmos, ground);
 * sim.run();
 *
 * const BallState& final = sim.getState();
 * LandingResult result = sim.getLandingResult();
 * @endcode
 *
 * For full trajectory:
 * @code
 * FlightSimulator sim(launch, atmos, ground);
 * auto trajectory = sim.runAndGetTrajectory();
 * @endcode
 */
class FlightSimulator
{
public:
	/**
	 * @brief Constructs a flight simulator with flat, uniform ground.
	 *
	 * @param launch Launch monitor data for the shot
	 * @param atmos Atmospheric conditions
	 * @param ground Ground surface properties (uniform everywhere)
	 */
	FlightSimulator(const LaunchData &launch,
	                const AtmosphericData &atmos,
	                const GroundSurface &ground);

	/**
	 * @brief Constructs a flight simulator with a custom ground provider.
	 *
	 * Allows position-dependent ground properties (fairway, rough, green, etc.).
	 *
	 * @param launch Launch monitor data for the shot
	 * @param atmos Atmospheric conditions
	 * @param groundProvider Ground provider for position-dependent surface properties
	 */
	FlightSimulator(const LaunchData &launch,
	                const AtmosphericData &atmos,
	                const GroundProvider &groundProvider);

	/**
	 * @brief Constructs a flight simulator with a custom terrain interface.
	 *
	 * Allows full 3D terrain with height, slope, and surface properties.
	 *
	 * @param launch Launch monitor data for the shot
	 * @param atmos Atmospheric conditions
	 * @param ground Ground surface properties (used as fallback)
	 * @param terrain Custom terrain implementation for height and normal queries
	 */
	FlightSimulator(const LaunchData &launch,
	                const AtmosphericData &atmos,
	                const GroundSurface &ground,
	                std::shared_ptr<TerrainInterface> terrain);

	/**
	 * @brief Runs the simulation to completion.
	 *
	 * Advances the simulation through all phases until the ball comes to rest.
	 * Safe to call on an already-complete simulation (no-op).
	 *
	 * @param dt Time step in seconds (default: SIMULATION_TIME_STEP)
	 */
	void run(float dt = physics_constants::SIMULATION_TIME_STEP);

	/**
	 * @brief Runs the simulation and returns the full trajectory.
	 *
	 * Equivalent to run() but collects and returns all intermediate states.
	 * Useful for visualization or analysis.
	 *
	 * @param dt Time step in seconds (default: SIMULATION_TIME_STEP)
	 * @return Vector of BallState snapshots from launch to rest (inclusive)
	 */
	std::vector<BallState> runAndGetTrajectory(float dt = physics_constants::SIMULATION_TIME_STEP);

	/**
	 * @brief Gets the current ball state.
	 *
	 * Returns the initial state before run(), or the final state after.
	 *
	 * @return Reference to the current ball state
	 */
	[[nodiscard]] const BallState &getState() const;

	/**
	 * @brief Computes landing result from the current state.
	 *
	 * Should be called after run() for meaningful results.
	 *
	 * @return LandingResult with distance, bearing, and time of flight
	 */
	[[nodiscard]] LandingResult getLandingResult() const;

	/**
	 * @brief Gets the derived physics variables for this shot.
	 *
	 * Provides access to computed quantities like air density, Reynolds number,
	 * initial spin rate, etc.
	 *
	 * @return Reference to the physics variables
	 */
	[[nodiscard]] const GolfBallPhysicsVariables &getPhysicsVariables() const;

	/**
	 * @brief Gets the name of the current flight phase.
	 *
	 * @return Phase name: "aerial", "bounce", "roll", or "complete"
	 */
	[[nodiscard]] const char *getCurrentPhaseName() const;

private:
	enum class Phase
	{
		Aerial,
		Bounce,
		Roll,
		Complete
	};

	Phase currentPhase;
	BallState state;

	// Must be declared before phases since phases hold a reference to it
	GolfBallPhysicsVariables physicsVars_;

	// Must be declared before phases since phases depend on it
	std::shared_ptr<TerrainInterface> terrainStorage_;

	AerialPhase aerialPhase;
	BouncePhase bouncePhase;
	RollPhase rollPhase;

	void stepOnce(float dt);
	void checkPhaseTransition();

	// Shared initialization called by all constructors
	void initializeFromLaunch(const LaunchData &launch);
};

#endif // FLIGHT_SIMULATOR_HPP
