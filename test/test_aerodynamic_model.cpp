/**
 * @file test_aerodynamic_model.cpp
 * @brief Unit tests for AerodynamicModel interface and DefaultAerodynamicModel.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "AerodynamicModel.hpp"
#include "DefaultAerodynamicModel.hpp"
#include "FlightSimulator.hpp"
#include "atmospheric_data.hpp"
#include "ground_surface.hpp"
#include "launch_data.hpp"
#include "physics_constants.hpp"

// ============================================================================
// DefaultAerodynamicModel — coefficient helpers
// ============================================================================

class DefaultModelTest : public ::testing::Test
{
protected:
	DefaultAerodynamicModel model;
};

// --- computeCd ---------------------------------------------------------------

TEST_F(DefaultModelTest, CdBelowLowReThreshold)
{
	// Re_x_e5 < RE_THRESHOLD_LOW (0.5) → CD_LOW regardless of spin
	EXPECT_NEAR(model.computeCd(0.25, 0.0), physics_constants::CD_LOW, 1e-6);
}

TEST_F(DefaultModelTest, CdAtLowReThresholdIsInclusive)
{
	// Re_x_e5 == RE_THRESHOLD_LOW uses <= branch → still CD_LOW
	EXPECT_NEAR(model.computeCd(0.5, 0.0), physics_constants::CD_LOW, 1e-6);
}

TEST_F(DefaultModelTest, CdMidRangeNoSpin)
{
	// Re = 0.75, S = 0:
	// Cd = CD_LOW - (CD_LOW - CD_HIGH) * (0.75 - 0.5) / (1.0 - 0.5) = 0.350
	double expected = static_cast<double>(physics_constants::CD_LOW) -
	                  (static_cast<double>(physics_constants::CD_LOW) -
	                   static_cast<double>(physics_constants::CD_HIGH)) *
	                      (0.75 - 0.5) / (1.0 - 0.5);
	EXPECT_NEAR(model.computeCd(0.75, 0.0), expected, 1e-6);
}

TEST_F(DefaultModelTest, CdMidRangeWithSpin)
{
	// Spin factor adds CD_SPIN * S to the interpolated base
	double base = static_cast<double>(physics_constants::CD_LOW) -
	              (static_cast<double>(physics_constants::CD_LOW) -
	               static_cast<double>(physics_constants::CD_HIGH)) *
	                  (0.75 - 0.5) / (1.0 - 0.5);
	double expected = base + static_cast<double>(physics_constants::CD_SPIN) * 0.2;
	EXPECT_NEAR(model.computeCd(0.75, 0.2), expected, 1e-6);
}

TEST_F(DefaultModelTest, CdAtHighReThreshold)
{
	// Re_x_e5 == RE_THRESHOLD_HIGH → uses >= branch → CD_HIGH
	EXPECT_NEAR(model.computeCd(1.0, 0.0), physics_constants::CD_HIGH, 1e-6);
}

TEST_F(DefaultModelTest, CdAboveHighReThreshold)
{
	// Re_x_e5 = 2.0, S = 0.5 → CD_HIGH + CD_SPIN * 0.5
	double expected = static_cast<double>(physics_constants::CD_HIGH) +
	                  static_cast<double>(physics_constants::CD_SPIN) * 0.5;
	EXPECT_NEAR(model.computeCd(2.0, 0.5), expected, 1e-6);
}

// --- computeCl ---------------------------------------------------------------

TEST_F(DefaultModelTest, ClZeroSpin)
{
	EXPECT_NEAR(model.computeCl(0.0), 0.0, 1e-6);
}

TEST_F(DefaultModelTest, ClLowSpin)
{
	// S = 0.1: Cl = LIFT_COEFF1*0.1 + LIFT_COEFF2*0.01
	double expected = static_cast<double>(physics_constants::LIFT_COEFF1) * 0.1 +
	                  static_cast<double>(physics_constants::LIFT_COEFF2) * 0.01;
	EXPECT_NEAR(model.computeCl(0.1), expected, 1e-5);
}

TEST_F(DefaultModelTest, ClAtSpinThresholdUsesQuadratic)
{
	// S == SPIN_FACTOR_THRESHOLD (0.3): <= branch, quadratic still applies
	double s = static_cast<double>(physics_constants::SPIN_FACTOR_THRESHOLD);
	double expected = static_cast<double>(physics_constants::LIFT_COEFF1) * s +
	                  static_cast<double>(physics_constants::LIFT_COEFF2) * s * s;
	EXPECT_NEAR(model.computeCl(s), expected, 1e-5);
}

TEST_F(DefaultModelTest, ClAboveThresholdClamped)
{
	// S > SPIN_FACTOR_THRESHOLD → CL_DEFAULT
	EXPECT_NEAR(model.computeCl(0.5), physics_constants::CL_DEFAULT, 1e-6);
	EXPECT_NEAR(model.computeCl(1.0), physics_constants::CL_DEFAULT, 1e-6);
}

// ============================================================================
// DefaultAerodynamicModel — computeSpinDecayTau
// ============================================================================

TEST_F(DefaultModelTest, TauAtTypicalSpeed)
{
	// tau = r / (TAU_COEFF * |v|)
	AerodynamicState state{
	    .velocity     = {0.0F, 100.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	double r        = static_cast<double>(physics_constants::STD_BALL_RADIUS_FT);
	double expected = r / (static_cast<double>(physics_constants::TAU_COEFF) * 100.0);
	EXPECT_NEAR(model.computeSpinDecayTau(state), expected, 0.1);
}

TEST_F(DefaultModelTest, TauScalesInverselyWithSpeed)
{
	// Doubling speed should halve tau
	AerodynamicState slow{
	    .velocity     = {0.0F, 50.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	AerodynamicState fast{
	    .velocity     = {0.0F, 100.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	EXPECT_NEAR(model.computeSpinDecayTau(slow),
	            model.computeSpinDecayTau(fast) * 2.0, 0.1);
}

// ============================================================================
// DefaultAerodynamicModel — computeAcceleration
// ============================================================================

TEST_F(DefaultModelTest, ZeroVelocityReturnsZeroAcceleration)
{
	AerodynamicState state{
	    .velocity     = {0.0F, 0.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {314.16F, 0.0F, 0.0F},
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	auto a = model.computeAcceleration(state);
	EXPECT_NEAR(a[0], 0.0F, 1e-6F);
	EXPECT_NEAR(a[1], 0.0F, 1e-6F);
	EXPECT_NEAR(a[2], 0.0F, 1e-6F);
}

TEST_F(DefaultModelTest, DragOpposesBallMotion)
{
	// Ball moving in +y, no spin, no wind → acceleration purely in -y
	AerodynamicState state{
	    .velocity     = {0.0F, 100.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	auto a = model.computeAcceleration(state);
	EXPECT_NEAR(a[0], 0.0F, 1e-4F);
	EXPECT_LT(a[1], 0.0F);             // drag opposes +y motion
	EXPECT_NEAR(a[2], 0.0F, 1e-4F);
}

TEST_F(DefaultModelTest, BackspinProducesUpwardMagnusForce)
{
	// Ball moving in +y, backspin axis in +x:
	// spinVector × vRel = {wx,0,0} × {0,vy,0} = {0, 0, wx*vy} → +z (upward)
	AerodynamicState state{
	    .velocity     = {0.0F, 100.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {314.16F, 0.0F, 0.0F},  // ~3000 rpm
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	auto a = model.computeAcceleration(state);
	EXPECT_LT(a[1], 0.0F);   // drag still opposes forward motion
	EXPECT_GT(a[2], 0.0F);   // Magnus lift is upward
}

TEST_F(DefaultModelTest, TailwindReducesEffectiveDrag)
{
	// Wind in the same direction as ball motion → smaller wind-relative speed → less drag.
	// This holds regardless of Re regime since vRel is strictly reduced.
	AerodynamicState noWind{
	    .velocity     = {0.0F, 100.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .c0           = 0.005682F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .re100        = 123600.0F,
	};
	AerodynamicState tailWind = noWind;
	tailWind.windVelocity     = {0.0F, 30.0F, 0.0F};  // same direction

	auto aNoWind   = model.computeAcceleration(noWind);
	auto aTailWind = model.computeAcceleration(tailWind);

	// Less deceleration with tailwind (both negative, tailwind closer to zero)
	EXPECT_GT(aTailWind[1], aNoWind[1]);
}

// ============================================================================
// Custom model — FlightSimulator integration
// ============================================================================

// Eliminates all aerodynamic forces. With only gravity acting on the ball, carry
// distance should far exceed the default-model result.
class NoDragModel : public AerodynamicModel
{
public:
	Vector3D computeAcceleration(const AerodynamicState &) const override
	{
		return {0.0F, 0.0F, 0.0F};
	}
	double computeSpinDecayTau(const AerodynamicState &) const override
	{
		return 1e9;
	}
};

TEST(CustomAerodynamicModelTest, CustomModelIsUsedByFlightSimulator)
{
	const LaunchData launch{
	    .ballSpeedMph   = 100.0f,
	    .launchAngleDeg = 30.0f,
	    .directionDeg   = 0.0f,
	    .backspinRpm    = 3000.0f,
	    .sidespinRpm    = 0.0f,
	};
	const AtmosphericData atmos{
	    .temp        = 70.0f,
	    .elevation   = 0.0f,
	    .vWind       = 0.0f,
	    .phiWind     = 0.0f,
	    .hWind       = 0.0f,
	    .relHumidity = 50.0f,
	    .pressure    = 29.92f,
	};
	GroundSurface ground;
	ground.height        = 0.0F;
	ground.restitution   = 0.4F;
	ground.frictionStatic  = 0.5F;
	ground.frictionDynamic = 0.2F;
	ground.firmness      = 0.8F;

	FlightSimulator defaultSim(launch, atmos, ground);
	FlightSimulator noDragSim(launch, atmos, ground, std::make_shared<NoDragModel>());

	defaultSim.run();
	noDragSim.run();

	float defaultDist = defaultSim.getLandingResult().distance;
	float noDragDist  = noDragSim.getLandingResult().distance;

	// A ball with no drag or lift travels significantly farther than one with
	// realistic aerodynamics — confirms the custom model is actually exercised.
	EXPECT_GT(noDragDist, defaultDist * 1.5F);
}

// ============================================================================
// Spin decay integration — verifies exponential rate, not just direction
// ============================================================================

// Returns a fixed tau regardless of velocity, with no aerodynamic forces.
// This lets us verify the decay rate precisely without coupling to drag physics.
class ConstantTauModel : public AerodynamicModel
{
public:
	explicit ConstantTauModel(double tau) : tau_(tau) {}
	Vector3D computeAcceleration(const AerodynamicState &) const override { return {}; }
	double computeSpinDecayTau(const AerodynamicState &) const override { return tau_; }
private:
	double tau_;
};

TEST(SpinDecayIntegrationTest, SpinDecaysAtExpectedExponentialRate)
{
	// With tau = 1.0 s and dt = 0.01 s, after 100 steps (1.0 s of flight)
	// the expected remaining spin fraction is exp(-1.0) ≈ 0.368.
	// A no-op model returning tau=1e9 would give ≈1.000 — clearly distinguishable.
	constexpr double kTau = 1.0;
	constexpr float  kDt  = 0.01F;
	constexpr int    kSteps = 100; // = 1.0 s

	const LaunchData launch{
	    .ballSpeedMph   = 100.0f,
	    .launchAngleDeg = 30.0f,
	    .directionDeg   = 0.0f,
	    .backspinRpm    = 3000.0f,
	    .sidespinRpm    = 0.0f,
	};
	const AtmosphericData atmos{
	    .temp        = 70.0f,
	    .elevation   = 0.0f,
	    .vWind       = 0.0f,
	    .phiWind     = 0.0f,
	    .hWind       = 0.0f,
	    .relHumidity = 50.0f,
	    .pressure    = 29.92f,
	};
	GroundSurface ground;
	ground.height          = 0.0F;
	ground.restitution     = 0.4F;
	ground.frictionStatic  = 0.5F;
	ground.frictionDynamic = 0.2F;
	ground.firmness        = 0.8F;

	FlightSimulator sim(launch, atmos, ground, std::make_shared<ConstantTauModel>(kTau));
	auto trajectory = sim.runAndGetTrajectory(kDt);

	ASSERT_GT(trajectory.size(), static_cast<size_t>(kSteps))
	    << "Trajectory too short to sample spin after " << kSteps << " steps";

	float spin0 = math_utils::magnitude(trajectory[0].spinVector);
	ASSERT_GT(spin0, 0.0F);

	float spinT = math_utils::magnitude(trajectory[kSteps].spinVector);

	float expectedFraction = std::exp(-static_cast<float>(kSteps) * kDt /
	                                  static_cast<float>(kTau));
	float actualFraction   = spinT / spin0;

	EXPECT_NEAR(actualFraction, expectedFraction, 0.02F);
}
