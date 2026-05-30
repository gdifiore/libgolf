#ifndef PHYSICS_CONSTANTS_HPP
#define PHYSICS_CONSTANTS_HPP

/**
 * @file physics_constants.hpp
 * @brief Universal physical constants and library-wide simulation parameters.
 *
 * Contains constants that are not specific to any one model implementation:
 * fundamental physics, ball geometry, unit conversions, atmospheric/SVP/
 * Sutherland constants (consumed by ShotPhysicsContext to derive state),
 * and shared simulation thresholds.
 *
 * Model-specific tuning lives on the model class itself as
 * `static constexpr` members — see DefaultAerodynamicModel,
 * DefaultBounceModel, DefaultRollModel.
 *
 * Reference: Based on work by Prof. Alan M. Nathan, University of Illinois
 * Urbana-Champaign (TrajectoryCalculatorGolf spreadsheet)
 */

namespace physics_constants
{
    // ========================================================================
    // GOLF BALL CONSTANTS
    // ========================================================================

    /// Standard golf ball circumference (inches)
    constexpr float STD_BALL_CIRCUMFERENCE_IN = 5.277F;

    /// Standard golf ball mass (ounces)
    constexpr float STD_BALL_MASS_OZ = 1.62F;

    // ------------------------------------------------------------------------
    // Lumped drag-coefficient (c0) derivation inputs
    // ------------------------------------------------------------------------
    // c0 = DRAG_FORCE_CONST * rho * (REF_BALL_MASS_OZ / mass) * (circ / REF_BALL_CIRC_IN)^2
    // These describe the reference ball the empirical drag fit was taken
    // against; they are an input to the derivation, not a tunable of any one
    // model, so they live here rather than on a concrete model class.

    /// Drag force constant for the c0 derivation
    constexpr float DRAG_FORCE_CONST = 0.07182F;

    /// Reference golf ball mass for the c0 derivation (oz)
    constexpr float REF_BALL_MASS_OZ = 5.125F;

    /// Reference golf ball circumference for the c0 derivation (inches)
    constexpr float REF_BALL_CIRC_IN = 9.125F;

    // ========================================================================
    // FUNDAMENTAL PHYSICAL CONSTANTS
    // ========================================================================

    /// Acceleration due to gravity at Earth's surface (ft/s²)
    constexpr float GRAVITY_FT_PER_S2 = 32.174F;

    /// Standard air density at STP (kg/m³)
    /// At 0°C (273.15 K) and 1 atm (760 mmHg)
    constexpr float STD_AIR_DENSITY_KG_PER_M3 = 1.2929F;

    /// Standard atmospheric pressure at sea level (mmHg)
    constexpr float STD_PRESSURE_MMHG = 760.0F;

    // ========================================================================
    // TEMPERATURE CONSTANTS
    // ========================================================================

    /// Offset to convert Celsius to Kelvin
    constexpr float KELVIN_OFFSET = 273.15F;

    /// Offset for Fahrenheit to Celsius conversion
    constexpr float FAHRENHEIT_OFFSET = 32.0F;

    /// Scale factor for Fahrenheit to Celsius conversion (5/9)
    constexpr float FAHRENHEIT_TO_CELSIUS_SCALE = 5.0F / 9.0F;

    // ========================================================================
    // LENGTH CONVERSION FACTORS
    // ========================================================================

    /// Meters to feet conversion factor
    constexpr float METERS_TO_FEET = 3.28084F;

    /// Feet to meters conversion factor
    constexpr float FEET_TO_METERS = 1.0F / METERS_TO_FEET;

    /// Yards to feet conversion factor
    constexpr float YARDS_TO_FEET = 3.0F;

    /// Inches per foot
    constexpr float INCHES_PER_FOOT = 12.0F;

    /// Inches per meter
    constexpr float INCHES_PER_METER = 1.0F / 0.0254F;

    // ========================================================================
    // VELOCITY CONVERSION FACTORS
    // ========================================================================

    /// Miles per hour to feet per second conversion factor
    /// 1 mph = 5280 ft / 3600 s = 1.46667 ft/s
    constexpr float MPH_TO_FT_PER_S = 5280.0F / 3600.0F;

    /// Reference velocity for Reynolds number calculation (m/s)
    /// Corresponds to 100 mph
    constexpr float RE100_VELOCITY_M_PER_S = 44.7F;

    // ========================================================================
    // MATHEMATICAL CONSTANTS
    // ========================================================================

    /// Pi constant (for MSVC compatibility where M_PI is not standard)
    constexpr float PI = 3.14159265358979323846F;

    /// Standard golf ball radius (feet)
    constexpr float STD_BALL_RADIUS_FT = STD_BALL_CIRCUMFERENCE_IN / (2.0F * PI) / INCHES_PER_FOOT;

    // ========================================================================
    // ANGULAR CONVERSION FACTORS
    // ========================================================================

    /// Degrees to radians conversion factor (π/180)
    constexpr float DEG_TO_RAD = PI / 180.0F;

    /// RPM to radians per second conversion factor (π/30)
    /// 1 RPM = 2π rad / 60 s = π/30 rad/s
    constexpr float RPM_TO_RAD_PER_S = PI / 30.0F;

    // ========================================================================
    // PRESSURE CONVERSION FACTORS
    // ========================================================================

    /// Inches of mercury to millimeters of mercury conversion
    /// 1 inHg = 25.4 mmHg exactly (mm per inch)
    constexpr float INHG_TO_MMHG = 1000.0F / INCHES_PER_METER;

    // ========================================================================
    // DENSITY CONVERSION FACTORS
    // ========================================================================

    /// kg/m³ to lb/ft³ conversion factor
    constexpr float KG_PER_M3_TO_LB_PER_FT3 = 0.06261F;

    // ========================================================================
    // ATMOSPHERIC PHYSICS CONSTANTS
    // ========================================================================

    /// Atmospheric pressure decay constant (1/m)
    /// Used in barometric formula: P = P0 * exp(-beta * elevation)
    constexpr float BETA_PRESSURE_DECAY = 0.0001217F;

    /// Water vapor pressure coefficient
    /// Used in humidity correction for air density
    constexpr float WATER_VAPOR_COEFF = 0.3783F;

    // ========================================================================
    // SATURATION VAPOR PRESSURE (SVP) CONSTANTS
    // ========================================================================
    // Magnus-Tetens formula constants for SVP calculation:
    // SVP = A * exp((B - T/C) * T / (D + T))
    // where T is temperature in Celsius

    /// SVP formula coefficient A (mmHg)
    constexpr float SVP_COEFF_A = 4.5841F;

    /// SVP formula coefficient B
    constexpr float SVP_COEFF_B = 18.687F;

    /// SVP formula coefficient C (°C)
    constexpr float SVP_COEFF_C = 234.5F;

    /// SVP formula coefficient D (°C)
    constexpr float SVP_COEFF_D = 257.14F;

    // ========================================================================
    // SUTHERLAND'S LAW CONSTANTS
    // ========================================================================
    // Used for calculating dynamic viscosity of air:
    // μ = μ_ref * (T/T_ref)^1.5 * (T_ref + S) / (T + S)

    /// Sutherland constant for air (K)
    constexpr float SUTHERLAND_CONSTANT = 120.0F;

    /// Reference dynamic viscosity coefficient
    /// Used in Reynolds number calculation
    constexpr float SUTHERLAND_VISCOSITY_COEFF = 0.000001512F;

    // ========================================================================
    // SIMULATION PARAMETERS
    // ========================================================================

    /// Time step for numerical integration (seconds)
    constexpr float SIMULATION_TIME_STEP = 0.01F;

    /// Integration coefficient (0.5 for Euler's method)
    constexpr float HALF = 0.5F;

    /// Convergence cap for the simulation loop (seconds of simulated time).
    /// FlightSimulator::run aborts past this to guard against non-terminating
    /// shots — e.g. terrain steeper than dynamic friction (runaway roll) or a
    /// custom model emitting NaN (phase-complete comparisons never trip).
    /// Generous vs. a realistic shot (~15 s); only genuine hangs reach it.
    constexpr float MAX_SIMULATION_TIME = 120.0F;

    // ========================================================================
    // NUMERICAL STABILITY THRESHOLDS
    // ========================================================================
    // Dimensionally distinct floors guarding against division by zero and
    // degenerate cases. Equal in value today, but split by dimension so each
    // can be retuned independently.

    /// Minimum speed magnitude before a velocity is treated as zero (ft/s)
    constexpr float MIN_SPEED = 0.01F;

    /// Minimum spin magnitude before spin is treated as zero (rad/s)
    constexpr float MIN_SPIN = 0.01F;

    /// Minimum vector length before it is treated as zero (ft)
    constexpr float MIN_LENGTH = 0.01F;

    // ========================================================================
    // PHASE TRANSITION THRESHOLDS
    // ========================================================================

    /// Minimum vertical velocity for bounce transition (ft/s)
    /// Below this, ball transitions from bounce to roll phase
    constexpr float MIN_BOUNCE_VELOCITY = 1.0F;

    /// Ground contact threshold for phase detection (ft)
    /// Ball is considered "on ground" when within this distance
    constexpr float GROUND_CONTACT_THRESHOLD = 0.1F;

    /// Flat surface threshold for slope calculations (cosine of angle)
    /// Surfaces with cos(θ) > this value (~2.5 degrees) use simplified physics
    constexpr float FLAT_SURFACE_THRESHOLD = 0.999F;

} // namespace physics_constants

#endif // PHYSICS_CONSTANTS_HPP
