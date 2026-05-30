/**
 * @file test_aerodynamic_model.cpp
 * @brief Unit tests for AerodynamicModel interface and DefaultAerodynamicModel.
 */

#include <gtest/gtest.h>
#include <algorithm>
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
	// Re_x_e5 < RE_THRESHOLD_LOW (0.5), no spin → CD_LOW
	EXPECT_NEAR(model.computeCd(0.25, 0.0), DefaultAerodynamicModel::CD_LOW, 1e-6);
}

TEST_F(DefaultModelTest, CdAtLowReThresholdIsInclusive)
{
	// Re_x_e5 == RE_THRESHOLD_LOW uses <= branch; no spin → CD_LOW
	EXPECT_NEAR(model.computeCd(0.5, 0.0), DefaultAerodynamicModel::CD_LOW, 1e-6);
}

TEST_F(DefaultModelTest, CdLowReCarriesSpinTerm)
{
	// The low-Re branch includes CD_SPIN * S, matching the linear branch at
	// the boundary so Cd is continuous there.
	const double S = 0.2;
	const double expected = static_cast<double>(DefaultAerodynamicModel::CD_LOW) +
	                        static_cast<double>(DefaultAerodynamicModel::CD_SPIN) * S;
	EXPECT_NEAR(model.computeCd(0.25, S), expected, 1e-6);
}

TEST_F(DefaultModelTest, CdIsContinuousAcrossLowReThreshold)
{
	// No step at RE_THRESHOLD_LOW: approaching 0.5 from below and from above
	// converges to the same value even with spin.
	const double S = 0.2;
	const double below = model.computeCd(0.5 - 1e-6, S);
	const double above = model.computeCd(0.5 + 1e-6, S);
	EXPECT_NEAR(below, above, 1e-4);
}

TEST_F(DefaultModelTest, CdMidRangeNoSpin)
{
	// Re = 0.75, S = 0:
	// Cd = CD_LOW - (CD_LOW - CD_HIGH) * (0.75 - 0.5) / (1.0 - 0.5) = 0.350
	double expected = static_cast<double>(DefaultAerodynamicModel::CD_LOW) -
	                  (static_cast<double>(DefaultAerodynamicModel::CD_LOW) -
	                   static_cast<double>(DefaultAerodynamicModel::CD_HIGH)) *
	                      (0.75 - 0.5) / (1.0 - 0.5);
	EXPECT_NEAR(model.computeCd(0.75, 0.0), expected, 1e-6);
}

TEST_F(DefaultModelTest, CdMidRangeWithSpin)
{
	// Spin factor adds CD_SPIN * S to the interpolated base
	double base = static_cast<double>(DefaultAerodynamicModel::CD_LOW) -
	              (static_cast<double>(DefaultAerodynamicModel::CD_LOW) -
	               static_cast<double>(DefaultAerodynamicModel::CD_HIGH)) *
	                  (0.75 - 0.5) / (1.0 - 0.5);
	double expected = base + static_cast<double>(DefaultAerodynamicModel::CD_SPIN) * 0.2;
	EXPECT_NEAR(model.computeCd(0.75, 0.2), expected, 1e-6);
}

TEST_F(DefaultModelTest, CdAtHighReThreshold)
{
	// Re_x_e5 == RE_THRESHOLD_HIGH → uses >= branch → CD_HIGH
	EXPECT_NEAR(model.computeCd(1.0, 0.0), DefaultAerodynamicModel::CD_HIGH, 1e-6);
}

TEST_F(DefaultModelTest, CdAboveHighReThreshold)
{
	// Re_x_e5 = 2.0, S = 0.5 → CD_HIGH + CD_SPIN * 0.5
	double expected = static_cast<double>(DefaultAerodynamicModel::CD_HIGH) +
	                  static_cast<double>(DefaultAerodynamicModel::CD_SPIN) * 0.5;
	EXPECT_NEAR(model.computeCd(2.0, 0.5), expected, 1e-6);
}

// --- computeCl ---------------------------------------------------------------

TEST_F(DefaultModelTest, ClZeroSpinZero)
{
	// Any Re, S=0 → Cl=0
	EXPECT_NEAR(model.computeCl(0.6, 0.0), 0.0, 1e-6);
	EXPECT_NEAR(model.computeCl(1.5, 0.0), 0.0, 1e-6);
}

TEST_F(DefaultModelTest, ClBelowNoLiftRe)
{
	// Re_x_e5 <= 0.3 → Cl = 0 regardless of spin
	EXPECT_NEAR(model.computeCl(0.2, 0.15), 0.0, 1e-6);
	EXPECT_NEAR(model.computeCl(0.3, 0.25), 0.0, 1e-6);
}

TEST_F(DefaultModelTest, ClLowReRamp)
{
	// 0.3 < Re < 0.5: smoothstep ramp toward Cl_50k. At Re=0.4, t=smoothstep(0.5)=0.5.
	const double S = 0.10;
	const double cl50 = static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A0) +
	                    static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A1) * S +
	                    static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A2) * S * S +
	                    static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A3) * S * S * S;
	// S=0.10 < CL_MAX_SR_LERP_LOW (0.35) → ClMax = CL_MAX_BASE.
	double expected = std::clamp(cl50 * 0.5, 0.0, static_cast<double>(DefaultAerodynamicModel::CL_MAX_BASE));
	EXPECT_NEAR(model.computeCl(0.4, S), expected, 1e-5);
}

TEST_F(DefaultModelTest, ClAt50kBinExact)
{
	// At Re_x_e5 = 0.5 use the 50k cubic exactly.
	const double S = 0.12;
	double expected = static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A0) +
	                  static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A1) * S +
	                  static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A2) * S * S +
	                  static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A3) * S * S * S;
	// S=0.12 < lerp band → ClMax = CL_MAX_BASE.
	expected = std::clamp(expected, 0.0, static_cast<double>(DefaultAerodynamicModel::CL_MAX_BASE));
	EXPECT_NEAR(model.computeCl(0.5, S), expected, 1e-5);
}

TEST_F(DefaultModelTest, ClBetweenBinsLerp)
{
	// At Re_x_e5 = 0.55 (midway 50k–60k), expect equal blend of clRe50k and clRe60k.
	const double S = 0.18;
	double cl50 = static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A0) +
	              static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A1) * S +
	              static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A2) * S * S +
	              static_cast<double>(DefaultAerodynamicModel::CL_RE50K_A3) * S * S * S;
	double cl60 = static_cast<double>(DefaultAerodynamicModel::CL_RE60K_A0) +
	              static_cast<double>(DefaultAerodynamicModel::CL_RE60K_A1) * S +
	              static_cast<double>(DefaultAerodynamicModel::CL_RE60K_A2) * S * S;
	// S=0.18 < lerp band → ClMax = CL_MAX_BASE.
	double expected = std::clamp(0.5 * (cl50 + cl60), 0.0, static_cast<double>(DefaultAerodynamicModel::CL_MAX_BASE));
	EXPECT_NEAR(model.computeCl(0.55, S), expected, 1e-5);
}

TEST_F(DefaultModelTest, ClHighReHillSaturation)
{
	// Re_x_e5 >= 0.7 uses Hill saturation Cl = ClMax(S)·S·g / (1 + S·g).
	// S=0.20 sits below the ClMax(S) lerp band → ClMax = CL_MAX_BASE.
	const double S = 0.20;
	const double g = static_cast<double>(DefaultAerodynamicModel::HIGH_RE_SPIN_GAIN);
	const double clMax = static_cast<double>(DefaultAerodynamicModel::CL_MAX_BASE);
	double expected = std::clamp(clMax * S * g / (1.0 + S * g), 0.0, clMax);
	EXPECT_NEAR(model.computeCl(1.5, S), expected, 1e-6);
	EXPECT_NEAR(model.computeCl(0.7, S), expected, 1e-6);
}

TEST_F(DefaultModelTest, ClClampedToMax)
{
	// Hill saturation asymptote → ClMax(S) as S → ∞. At S=5 the cap is the
	// high-spin asymptote CL_MAX_HIGH_SR.
	EXPECT_LE(model.computeCl(2.0, 5.0),
	          static_cast<double>(DefaultAerodynamicModel::CL_MAX_HIGH_SR) + 1e-6);
}

TEST_F(DefaultModelTest, ClMaxLerpsAtHighSpinFactor)
{
	// At S well above CL_MAX_SR_LERP_HIGH (0.50), the Hill cap = CL_MAX_HIGH_SR
	// (0.32) > CL_MAX_BASE (0.268). Verifies the dynamic ClMax is wired in.
	const double clHigh = model.computeCl(2.0, 1.0);
	EXPECT_GT(clHigh, static_cast<double>(DefaultAerodynamicModel::CL_MAX_BASE));
	EXPECT_LE(clHigh, static_cast<double>(DefaultAerodynamicModel::CL_MAX_HIGH_SR) + 1e-6);
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
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
	    .re100        = 123600.0F,
	};
	double r        = static_cast<double>(physics_constants::STD_BALL_RADIUS_FT);
	double expected = r / (static_cast<double>(DefaultAerodynamicModel::TAU_COEFF) * 100.0);
	EXPECT_NEAR(model.computeSpinDecayTau(state), expected, 0.1);
}

TEST_F(DefaultModelTest, TauScalesInverselyWithSpeed)
{
	// Doubling speed should halve tau
	AerodynamicState slow{
	    .velocity     = {0.0F, 50.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
	    .re100        = 123600.0F,
	};
	AerodynamicState fast{
	    .velocity     = {0.0F, 100.0F, 0.0F},
	    .windVelocity = {0.0F, 0.0F, 0.0F},
	    .spinVector   = {0.0F, 0.0F, 0.0F},
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
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
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
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
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
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
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
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
	    .position     = {0.0F, 0.0F, 0.0F},
	    .currentTime  = 0.0F,
	    .ballRadius   = physics_constants::STD_BALL_RADIUS_FT,
	    .c0           = 0.005682F,
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
	float computeSpinDecayTau(const AerodynamicState &) const override
	{
		return 1e9F;
	}
};

TEST(CustomAerodynamicModelTest, CustomModelIsUsedByFlightSimulator)
{
	// Shallow launch keeps impact angle below the Penner critical angle
	// (~15° from surface) so bounce physics behave identically for both
	// runs — isolating the drag-vs-no-drag comparison in carry distance.
	const LaunchData launch{
	    .ballSpeedMph   = 100.0f,
	    .launchAngleDeg = 12.0f,
	    .directionDeg   = 0.0f,
	    .backspinRpm    = 2500.0f,
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
	explicit ConstantTauModel(float tau) : tau_(tau) {}
	Vector3D computeAcceleration(const AerodynamicState &) const override { return {}; }
	float computeSpinDecayTau(const AerodynamicState &) const override { return tau_; }
private:
	float tau_;
};

TEST(SpinDecayIntegrationTest, SpinDecaysAtExpectedExponentialRate)
{
	// With tau = 1.0 s and dt = 0.01 s, after 100 steps (1.0 s of flight)
	// the expected remaining spin fraction is exp(-1.0) ≈ 0.368.
	// A no-op model returning tau=1e9 would give ≈1.000 — clearly distinguishable.
	constexpr float kTau = 1.0F;
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

	float expectedFraction = std::exp(-static_cast<float>(kSteps) * kDt / kTau);
	float actualFraction   = spinT / spin0;

	EXPECT_NEAR(actualFraction, expectedFraction, 0.02F);
}

// ============================================================================
// Raw atmosphere fields — populated on the AerodynamicState
// ============================================================================

// Captures the last AerodynamicState handed to the model so the test can
// inspect what AerialPhase actually populated.
class CapturingAeroModel : public AerodynamicModel
{
public:
	mutable AerodynamicState lastState{};
	mutable bool seen = false;

	Vector3D computeAcceleration(const AerodynamicState &s) const override
	{
		lastState = s;
		seen = true;
		return {0.0F, 0.0F, 0.0F};
	}
	float computeSpinDecayTau(const AerodynamicState &) const override { return 1e9F; }
};

TEST(AerodynamicStateRawAtmosphereTest, RawFieldsPopulatedFromContext)
{
	const LaunchData launch{
	    .ballSpeedMph   = 100.0f,
	    .launchAngleDeg = 12.0f,
	    .directionDeg   = 0.0f,
	    .backspinRpm    = 2500.0f,
	    .sidespinRpm    = 0.0f,
	};
	const AtmosphericData atmos{
	    .temp        = 70.0f,
	    .elevation   = 0.0f,
	    .vWind       = 0.0f,
	    .phiWind     = 0.0f,
	    .hWind       = 0.0f,
	    .relHumidity = 55.0f,
	    .pressure    = 29.92f,
	};
	GroundSurface ground;
	ground.height          = 0.0F;
	ground.restitution     = 0.4F;
	ground.frictionStatic  = 0.5F;
	ground.frictionDynamic = 0.2F;
	ground.firmness        = 0.8F;

	auto capture = std::make_shared<CapturingAeroModel>();
	FlightSimulator sim(launch, atmos, ground, capture);
	sim.run();

	ASSERT_TRUE(capture->seen);
	const auto &s = capture->lastState;

	// Sea level air density ~1.225 kg/m³; allow generous band for humidity/temp deviation.
	EXPECT_GT(s.airDensityKgPerM3, 1.0F);
	EXPECT_LT(s.airDensityKgPerM3, 1.4F);

	// Sutherland μ for air near room temp ≈ 1.8e-5 Pa·s.
	EXPECT_GT(s.airViscosity, 1.5e-5F);
	EXPECT_LT(s.airViscosity, 2.1e-5F);

	// 70°F = 21.11°C ≈ 294.26 K.
	EXPECT_NEAR(s.tempKelvin, 294.26F, 0.5F);

	// 29.92 inHg → ~760 mmHg.
	EXPECT_NEAR(s.pressureMmHg, 760.0F, 1.0F);

	// Pass-through of atmos.relHumidity.
	EXPECT_FLOAT_EQ(s.relHumidity, 55.0F);
}
