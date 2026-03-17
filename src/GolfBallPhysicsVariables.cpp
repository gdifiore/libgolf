/**
 * @file GolfBallPhysicsVariables.cpp
 * @author Gabriel DiFiore
 * @brief Contains the implementation of the GolfBallPhysicsVariables class.
 *
 * This file defines the GolfBallPhysicsVariables class, which is responsible for calculating
 * various intermediate physics variables related to a golf ball. It includes functions for
 * calculating air density, barometric pressure, spin rates, launch angles, wind effects, and
 * other variables used in golf ball physics calculations.
 *
 * The GolfBallPhysicsVariables class takes a golf ball object and atmospheric data as input,
 * and provides methods to calculate all the required variables.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "GolfBallPhysicsVariables.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include "math_utils.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <iostream>

/**
 * @brief Constructs a GolfBallPhysicsVariables object.
 *
 * This constructor initializes a GolfBallPhysicsVariables object with the given golf ball and atmospheric data.
 *
 * @param ball The golf ball structure containing relevant data.
 * @param atmos The atmospheric data structure containing relevant data.
 *
 * @note The user is responsible for validating the ball and atmos structures before passing them to this constructor.
 *       This class assumes that the input data is valid and within physically reasonable ranges.
 *       Passing invalid or out-of-range data may lead to unexpected behavior or incorrect calculations.
 */
GolfBallPhysicsVariables::GolfBallPhysicsVariables(const golfBall &ball, const atmosphericData &atmos)
    : ball(ball), atmos(atmos)
{
    tempC = math_utils::convertFahrenheitToCelsius(atmos.temp);
    elevationM = math_utils::convertFeetToMeters(atmos.elevation);

    calculateAllVariables();
}

void GolfBallPhysicsVariables::calculateAllVariables()
{
    calculateBarometricPressure();
    calculateSVP();
    calculateOmega();
    calculateROmega();
    calculateRhoMetric();
    calculateRhoImperial();
    calculateC0();
    calculateV0();
    calculateW();
    calculateVw();
    calculateRe100();
}

void GolfBallPhysicsVariables::calculateRhoMetric()
{
    rhoMetric = physics_constants::STD_AIR_DENSITY_KG_PER_M3 * ((physics_constants::KELVIN_OFFSET / (math_utils::convertCelsiusToKelvin(tempC)) *
                                                                 ((barometricPressure * std::exp(-physics_constants::BETA_PRESSURE_DECAY * elevationM) - physics_constants::WATER_VAPOR_COEFF * atmos.relHumidity * (SVP / 100.0F)) / physics_constants::STD_PRESSURE_MMHG)));
}

void GolfBallPhysicsVariables::calculateRhoImperial()
{
    rhoImperial = rhoMetric * physics_constants::KG_PER_M3_TO_LB_PER_FT3;
}

void GolfBallPhysicsVariables::calculateC0()
{
    c0 = physics_constants::DRAG_FORCE_CONST * rhoImperial * (physics_constants::REF_BALL_MASS_OZ / physics_constants::STD_BALL_MASS_OZ) *
         std::pow(physics_constants::STD_BALL_CIRCUMFERENCE_IN / physics_constants::REF_BALL_CIRC_IN, 2);
}

void GolfBallPhysicsVariables::calculateV0()
{
    v0_magnitude = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
    float v0x = v0_magnitude * std::cos(ball.launchAngle * physics_constants::DEG_TO_RAD) * std::sin(ball.direction * physics_constants::DEG_TO_RAD);
    float v0y = v0_magnitude * std::cos(ball.launchAngle * physics_constants::DEG_TO_RAD) * std::cos(ball.direction * physics_constants::DEG_TO_RAD);
    float v0z = v0_magnitude * std::sin(ball.launchAngle * physics_constants::DEG_TO_RAD);
    v0 = Vector3D{v0x, v0y, v0z};
}

void GolfBallPhysicsVariables::calculateW()
{
    float wx = (ball.backspin * std::cos(ball.direction * physics_constants::DEG_TO_RAD) -
                ball.sidespin * std::sin(ball.launchAngle * physics_constants::DEG_TO_RAD) * std::sin(ball.direction * physics_constants::DEG_TO_RAD)) *
               physics_constants::RPM_TO_RAD_PER_S;
    float wy = (-ball.backspin * std::sin(ball.direction * physics_constants::DEG_TO_RAD) -
                ball.sidespin * std::sin(ball.launchAngle * physics_constants::DEG_TO_RAD) * std::cos(ball.direction * physics_constants::DEG_TO_RAD)) *
               physics_constants::RPM_TO_RAD_PER_S;
    float wz = (ball.sidespin * std::cos(ball.launchAngle * physics_constants::DEG_TO_RAD)) * physics_constants::RPM_TO_RAD_PER_S;
    w = Vector3D{wx, wy, wz};
}

void GolfBallPhysicsVariables::calculateOmega()
{
    omega = std::sqrt(std::pow(ball.backspin, 2) + std::pow(ball.sidespin, 2)) * physics_constants::RPM_TO_RAD_PER_S;
}

void GolfBallPhysicsVariables::calculateROmega()
{
    rOmega = (physics_constants::STD_BALL_CIRCUMFERENCE_IN / (2 * physics_constants::PI)) * (omega / physics_constants::INCHES_PER_FOOT);
}

void GolfBallPhysicsVariables::calculateVw()
{
    float vxw = atmos.vWind * physics_constants::MPH_TO_FT_PER_S * std::sin(atmos.phiWind * physics_constants::DEG_TO_RAD);
    float vyw = atmos.vWind * physics_constants::MPH_TO_FT_PER_S * std::cos(atmos.phiWind * physics_constants::DEG_TO_RAD);
    vw = Vector3D{vxw, vyw, 0.0f};
}

void GolfBallPhysicsVariables::calculateSVP()
{
    SVP = physics_constants::SVP_COEFF_A * std::exp((physics_constants::SVP_COEFF_B - tempC / physics_constants::SVP_COEFF_C) * tempC / (physics_constants::SVP_COEFF_D + tempC));
}

void GolfBallPhysicsVariables::calculateBarometricPressure()
{
    barometricPressure = atmos.pressure * physics_constants::INHG_TO_MMHG;
}

void GolfBallPhysicsVariables::calculateRe100()
{
    Re100 = rhoMetric * physics_constants::RE100_VELOCITY_M_PER_S * (physics_constants::STD_BALL_CIRCUMFERENCE_IN / (physics_constants::PI * physics_constants::INCHES_PER_METER)) *
            (math_utils::convertCelsiusToKelvin(tempC) + physics_constants::SUTHERLAND_CONSTANT) /
            (physics_constants::SUTHERLAND_VISCOSITY_COEFF * std::pow(math_utils::convertCelsiusToKelvin(tempC), 1.5));
}
