#ifndef FLIGHTPHASE_HPP
#define FLIGHTPHASE_HPP

#include "BallState.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
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
 * - Drag and lift forces
 * - Spin decay
 * - Wind effects
 * - Reynolds number calculations
 */
class AerialPhase : public FlightPhase
{
public:
	AerialPhase(GolfBallPhysicsVariables &physicsVars,
	            const LaunchData &launch,
	            const AtmosphericData &atmos,
	            std::shared_ptr<TerrainInterface> terrain);

	void initialize(BallState &state);
	void calculateStep(BallState &state, float dt) override;
	void calculateAccelerations(BallState &state);
	bool isPhaseComplete(const BallState &state) const override;

	// Getters for physics variables (needed for testing)
	[[nodiscard]] auto getV() const -> float { return v; }
	[[nodiscard]] auto getVMph() const -> float { return vMph; }
	[[nodiscard]] auto getPhi() const -> float { return phi; }
	[[nodiscard]] auto getTau() const -> float { return tau; }
	[[nodiscard]] auto getRw() const -> float { return rw; }
	[[nodiscard]] auto getWPerpDivW() const -> float { return w_perp_div_w; }
	[[nodiscard]] auto getRe_x_e5() const -> float { return Re_x_e5; }
	[[nodiscard]] auto getVw() const -> float { return vw; }
	[[nodiscard]] auto getVwMph() const -> float { return vwMph; }
	[[nodiscard]] auto getSpinFactor() const -> float { return spinFactor; }

	auto determineCoefficientOfDrag() -> float;
	auto determineCoefficientOfLift() -> float;

private:
	GolfBallPhysicsVariables &physicsVars;
	LaunchData launch;
	AtmosphericData atmos;
	std::shared_ptr<TerrainInterface> terrain;

	// Calculated variables (derived from state)
	float v;
	float vMph;
	float phi;
	float tau;
	float rw;
	// Perpendicular-spin correction factor for Magnus force.
	// Fixed at 1.0 per the Nathan (UIUC) reference model — a placeholder
	// for a future correction accounting for non-pure-backspin trajectories.
	float w_perp_div_w = 1.0F;
	float Re_x_e5;
	float vw;
	float vwMph;
	float spinFactor;
	Vector3D velocity3D_w;
	Vector3D accelerationDrag3D;
	Vector3D accelerationMagnitude3D;

	// Private calculation methods
	void calculatePosition(BallState &state, float dt);
	void calculateV(BallState &state, float dt);
	void calculateVelocityw(const BallState &state);
	void calculatePhi(const BallState &state);
	void calculateTau();
	void calculateRw(const BallState &state);
	void calculateRe_x_e5();
	void calculateSpinFactor();
	void calculateAccelD(const BallState &state);
	void calculateAccelM(const BallState &state);
	void calculateAccel(BallState &state);
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
	BouncePhase(GolfBallPhysicsVariables &physicsVars,
	            const LaunchData &launch,
	            const AtmosphericData &atmos,
	            std::shared_ptr<TerrainInterface> terrain);

	void calculateStep(BallState &state, float dt) override;
	bool isPhaseComplete(const BallState &state) const override;

private:
	GolfBallPhysicsVariables &physicsVars;
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
	RollPhase(GolfBallPhysicsVariables &physicsVars,
	          const LaunchData &launch,
	          const AtmosphericData &atmos,
	          std::shared_ptr<TerrainInterface> terrain);

	void calculateStep(BallState &state, float dt) override;
	bool isPhaseComplete(const BallState &state) const override;

private:
	GolfBallPhysicsVariables &physicsVars;
	LaunchData launch;
	AtmosphericData atmos;
	std::shared_ptr<TerrainInterface> terrain;
};

#endif // FLIGHTPHASE_HPP
