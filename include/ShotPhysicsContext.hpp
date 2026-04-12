#ifndef SHOT_PHYSICS_CONTEXT_HPP
#define SHOT_PHYSICS_CONTEXT_HPP

#include "atmospheric_data.hpp"
#include "launch_data.hpp"
#include "math_utils.hpp"
#include "physics_constants.hpp"

class ShotPhysicsContext
{
public:
	ShotPhysicsContext(const LaunchData &launch,
							 const AtmosphericData &atmos);

	// Getters
	[[nodiscard]] auto getRhoImperial() const -> float
	{
		return rhoImperial;
	}
	[[nodiscard]] auto getRhoMetric() const -> float { return rhoMetric; }
	[[nodiscard]] auto getC0() const -> float { return c0; }
	[[nodiscard]] auto getV0() const -> float { return v0_magnitude; }
	[[nodiscard]] auto getV0Vector() const -> Vector3D { return v0; }
	[[nodiscard]] auto getW() const -> Vector3D { return w; }
	[[nodiscard]] auto getOmega() const -> float { return omega; }
	[[nodiscard]] auto getROmega() const -> float { return rOmega; }
	[[nodiscard]] auto getTempC() const -> float { return tempC; }
	[[nodiscard]] auto getElevationM() const -> float { return elevationM; }
	[[nodiscard]] auto getVw() const -> Vector3D { return vw; }
	[[nodiscard]] auto getSVP() const -> float { return SVP; }
	[[nodiscard]] auto getBarometricPressure() const -> float
	{
		return barometricPressure;
	}
	[[nodiscard]] auto getRe100() const -> float { return Re100; }

	void calculateAllVariables();

private:
	LaunchData launch;
	AtmosphericData atmos;

	// Member variables
	float rhoImperial = 0.0F;
	float rhoMetric = 0.0F;
	float c0 = 0.0F;
	float tempC = 0.0F;
	float elevationM = 0.0F;
	float v0_magnitude;

	Vector3D v0;
	Vector3D w;
	Vector3D vw;
	float omega;
	float rOmega;
	float SVP;
	float barometricPressure;
	float Re100;

	// Private calculation methods
	void calculateRhoMetric();
	void calculateRhoImperial();
	void calculateC0();
	void calculateV0();
	void calculateW();
	void calculateOmega();
	void calculateROmega();
	void calculateVw();
	void calculateSVP();
	void calculateBarometricPressure();
	void calculateRe100();
};

#endif // SHOT_PHYSICS_CONTEXT_HPP