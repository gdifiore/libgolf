#ifndef FLIGHTPHASE_HPP
#define FLIGHTPHASE_HPP

#include "AerodynamicModel.hpp"
#include "BallState.hpp"
#include "ShotPhysicsContext.hpp"
#include "atmospheric_data.hpp"
#include "launch_data.hpp"
#include "ground_surface.hpp"
#include "terrain_interface.hpp"

#include <memory>

/**
 * @brief Base class for different flight phases of a golf ball.
 *
 * This abstract class defines the interface for different phases of golf ball motion:
 * aerial flight, bouncing, and rolling. Each phase implements its own physics
 * calculations while maintaining a consistent state representation.
 */
class FlightPhase
{
public:
	virtual ~FlightPhase() = default;

	FlightPhase(const FlightPhase&) = delete;
	FlightPhase& operator=(const FlightPhase&) = delete;

	FlightPhase(FlightPhase&&) = default;
	FlightPhase& operator=(FlightPhase&&) = default;

	/**
	 * @brief Calculates a single time step for this phase.
	 *
	 * @param state The current ball state to be updated
	 * @param dt The time step duration
	 */
	virtual void calculateStep(BallState &state, float dt) = 0;

	/**
	 * @brief Checks if this phase has completed.
	 *
	 * @param state The current ball state
	 * @return true if the phase is complete and a transition should occur
	 */
	virtual bool isPhaseComplete(const BallState &state) const = 0;

protected:
	FlightPhase() = default;
};

/**
 * @brief Implements the aerial flight phase with full aerodynamic calculations.
 *
 * This phase handles the golf ball's flight through the air, including:
 * - Drag and lift forces (delegated to AerodynamicModel)
 * - Spin decay (delegated to AerodynamicModel)
 * - Wind effects
 * - Velocity-Verlet integration
 */
class AerialPhase : public FlightPhase
{
public:
	AerialPhase(ShotPhysicsContext &physicsVars,
	            const LaunchData &launch,
	            const AtmosphericData &atmos,
	            std::shared_ptr<TerrainInterface> terrain,
	            std::shared_ptr<AerodynamicModel> model = nullptr);

	void initialize(BallState &state);
	void calculateStep(BallState &state, float dt) override;
	void calculateAccelerations(BallState &state);
	bool isPhaseComplete(const BallState &state) const override;

	// Getters for observable flight quantities (useful for testing and diagnostics)
	[[nodiscard]] auto getV() const -> float { return v; }
	[[nodiscard]] auto getVMph() const -> float { return vMph; }
	[[nodiscard]] auto getPhi() const -> float { return phi; }
	[[nodiscard]] auto getTau() const -> float { return tau; }
	[[nodiscard]] auto getRw() const -> float { return rw; }
	[[nodiscard]] auto getVw() const -> float { return vw; }
	[[nodiscard]] auto getVwMph() const -> float { return vwMph; }

private:
	ShotPhysicsContext &physicsVars;
	LaunchData launch;
	AtmosphericData atmos;
	std::shared_ptr<TerrainInterface> terrain;
	std::shared_ptr<AerodynamicModel> model;

	// Cached scalar quantities derived from BallState each step
	float v;
	float vMph;
	float phi;
	float tau;
	float rw;
	float vw;
	float vwMph;
	Vector3D velocity3D_w;

	// Private calculation methods
	void calculatePosition(BallState &state, float dt);
	void calculateV(BallState &state, float dt);
	void calculateVelocityw(const BallState &state);
	void calculatePhi(const BallState &state);
	void calculateTau(const BallState &state);
	void calculateRw(const BallState &state);
	void calculateAccel(BallState &state);

	[[nodiscard]] AerodynamicState buildAerodynamicState(const BallState &state) const;
};

/**
 * @brief Handles the ball's bounce when it contacts the ground.
 *
 * Applies coefficient of restitution to vertical velocity and friction
 * to horizontal velocity based on surface properties. Uses aerodynamic
 * calculations for trajectory between bounces to preserve Magnus effect
 * and drag forces. Transitions to roll phase when velocity is low.
 */
class BouncePhase : public FlightPhase
{
public:
	BouncePhase(ShotPhysicsContext &physicsVars,
	            const LaunchData &launch,
	            const AtmosphericData &atmos,
	            std::shared_ptr<TerrainInterface> terrain,
	            std::shared_ptr<AerodynamicModel> model = nullptr);

	void calculateStep(BallState &state, float dt) override;
	bool isPhaseComplete(const BallState &state) const override;

private:
	ShotPhysicsContext &physicsVars;
	LaunchData launch;
	AtmosphericData atmos;
	std::shared_ptr<TerrainInterface> terrain;
	AerialPhase aerialPhase; // Used for aerodynamic calculations between bounces
};

/**
 * @brief Handles the ball's rolling motion on the ground.
 *
 * Applies rolling friction to decelerate the ball and spin decay.
 * The ball is kept on the ground surface with only horizontal motion.
 * Phase completes when velocity drops below stopping threshold.
 */
class RollPhase : public FlightPhase
{
public:
	RollPhase(ShotPhysicsContext &physicsVars,
	          const LaunchData &launch,
	          const AtmosphericData &atmos,
	          std::shared_ptr<TerrainInterface> terrain);

	void calculateStep(BallState &state, float dt) override;
	bool isPhaseComplete(const BallState &state) const override;

private:
	ShotPhysicsContext &physicsVars;
	LaunchData launch;
	AtmosphericData atmos;
	std::shared_ptr<TerrainInterface> terrain;
};

#endif // FLIGHTPHASE_HPP
