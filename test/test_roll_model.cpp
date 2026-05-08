#include <gtest/gtest.h>

#include <FlightSimulator.hpp>
#include <RollModel.hpp>
#include <atmospheric_data.hpp>
#include <ground_surface.hpp>
#include <launch_data.hpp>

#include <atomic>
#include <memory>

// Minimal model that records every call and returns deterministic output.
// The point of this fixture is to verify FlightSimulator wires a custom
// RollModel implementation through to the roll phase, not to test physics.
class RecordingRollModel : public RollModel
{
public:
	RollResult step(const RollState &state,
	                const GroundSurface &surface) const override
	{
		++callCount_;
		(void)surface;
		// Return atRest=true on the first call so the simulator transitions
		// to Complete quickly and the test does not time out.
		return RollResult{
			state.position,
			{0.0F, 0.0F, 0.0F},
			{0.0F, 0.0F, 0.0F},
			true
		};
	}

	int callCount() const { return callCount_.load(); }

private:
	mutable std::atomic<int> callCount_{0};
};

TEST(RollModelInterfaceTest, CustomModelReceivesStepCalls)
{
	auto model = std::make_shared<RecordingRollModel>();

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

	FlightSimulator sim(launch, atmos, ground, /*aero*/ nullptr, /*bounce*/ nullptr, model);
	sim.run();

	EXPECT_GT(model->callCount(), 0);
}
