#include <gtest/gtest.h>

#include <BounceModel.hpp>
#include <FlightSimulator.hpp>
#include <atmospheric_data.hpp>
#include <ground_surface.hpp>
#include <launch_data.hpp>

#include <atomic>
#include <memory>

// Minimal model that records every call and returns deterministic output.
// The point of this fixture is to verify FlightSimulator wires a custom
// BounceModel implementation through to ground impact, not to test physics.
class RecordingBounceModel : public BounceModel
{
public:
	BounceResult resolveBounce(const BounceState &state,
	                           const GroundSurface &surface) const override
	{
		++callCount_;
		lastSpinMagnitude_ = std::sqrt(
			state.spinVector[0] * state.spinVector[0] +
			state.spinVector[1] * state.spinVector[1] +
			state.spinVector[2] * state.spinVector[2]);
		// Send the ball to rest above the ground so the simulator transitions
		// quickly to roll/complete and we don't time out.
		(void)surface;
		return BounceResult{
			{0.0F, 0.0F, 0.0F},
			{0.0F, 0.0F, 0.0F}
		};
	}

	int callCount() const { return callCount_.load(); }
	float lastSpinMagnitude() const { return lastSpinMagnitude_; }

private:
	mutable std::atomic<int> callCount_{0};
	mutable float lastSpinMagnitude_{0.0F};
};

TEST(BounceModelInterfaceTest, CustomModelReceivesBounceCalls)
{
	auto model = std::make_shared<RecordingBounceModel>();

	LaunchData launch{
		.ballSpeedMph = 100.0F,
		.launchAngleDeg = 12.0F,
		.directionDeg = 0.0F,
		.backspinRpm = 3000.0F,
		.sidespinRpm = 0.0F,
		.startX = 0.0F,
		.startY = 0.0F,
		.startZ = 0.0F};
	AtmosphericData atmos{
		.temp = 70.0F,
		.elevation = 0.0F,
		.vWind = 0.0F,
		.phiWind = 0.0F,
		.hWind = 0.0F,
		.relHumidity = 50.0F,
		.pressure = 29.92F};
	GroundSurface ground;

	FlightSimulator sim(launch, atmos, ground, /*aero*/ nullptr, model);
	sim.run();

	EXPECT_GT(model->callCount(), 0);
	EXPECT_GT(model->lastSpinMagnitude(), 0.0F);
}
