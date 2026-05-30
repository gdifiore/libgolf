#ifndef SHOT_PHYSICS_CONTEXT_HPP
#define SHOT_PHYSICS_CONTEXT_HPP

#include "BallProperties.hpp"
#include "atmospheric_data.hpp"
#include "launch_data.hpp"
#include "math_utils.hpp"

class ShotPhysicsContext
{
public:
	ShotPhysicsContext(const LaunchData &launch,
							 const AtmosphericData &atmos,
							 const BallProperties &ball = {});

	// Getters
	// Mixed unit convention, matching AerodynamicState: imperial for kinematics
	// (ft, s, ft/s), SI for raw atmosphere (kg/m³, Pa·s, K), spin in rad/s.
	/// Air density at launch, lb/ft³ (pound-mass per cubic foot).
	[[nodiscard]] auto getRhoImperial() const -> float
	{
		return rhoImperial;
	}
	/// Air density at launch, kg/m³.
	[[nodiscard]] auto getRhoMetric() const -> float { return rhoMetric; }
	/// Lumped aerodynamic force coefficient (air density × cross-section / mass); c0·v² yields ft/s².
	[[nodiscard]] auto getC0() const -> float { return c0; }
	/// Launch speed magnitude, ft/s.
	[[nodiscard]] auto getV0() const -> float { return v0_magnitude; }
	/// Launch velocity vector, ft/s.
	[[nodiscard]] auto getV0Vector() const -> Vector3D { return v0; }
	/// Spin vector, rad/s (direction = spin axis).
	[[nodiscard]] auto getW() const -> Vector3D { return w; }
	/// Spin magnitude, rad/s.
	[[nodiscard]] auto getOmega() const -> float { return omega; }
	/// Surface speed r·ω, ft/s.
	[[nodiscard]] auto getROmega() const -> float { return rOmega; }
	/// Air temperature, °C.
	[[nodiscard]] auto getTempC() const -> float { return tempC; }
	/// Elevation above sea level, m.
	[[nodiscard]] auto getElevationM() const -> float { return elevationM; }
	/// Wind velocity vector, ft/s.
	[[nodiscard]] auto getVw() const -> Vector3D { return vw; }
	/// Saturation vapor pressure, mmHg.
	[[nodiscard]] auto getSVP() const -> float { return SVP; }
	/// Barometric pressure, mmHg.
	[[nodiscard]] auto getBarometricPressure() const -> float
	{
		return barometricPressure;
	}
	/// Reynolds number at 100 mph under launch atmospherics (dimensionless).
	[[nodiscard]] auto getRe100() const -> float { return Re100; }
	/// Dynamic viscosity of air, Pa·s (= kg/(m·s)).
	[[nodiscard]] auto getAirViscosity() const -> float { return airViscosity; }
	/// Air temperature, K.
	[[nodiscard]] auto getTempKelvin() const -> float;
	/// Relative humidity, percent.
	[[nodiscard]] auto getRelHumidity() const -> float { return atmos.relHumidity; }

private:
	void calculateAllVariables();
	LaunchData launch;
	AtmosphericData atmos;
	BallProperties ball;

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
	float airViscosity = 0.0F;

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
	void calculateAirViscosity();
	void calculateRe100();
};

#endif // SHOT_PHYSICS_CONTEXT_HPP