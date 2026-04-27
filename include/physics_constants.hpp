#ifndef PHYSICS_CONSTANTS_HPP
#define PHYSICS_CONSTANTS_HPP

/**
 * @file physics_constants.hpp
 * @brief Physical constants used in golf ball trajectory calculations.
 *
 * This file consolidates all physical constants, conversion factors, and
 * empirical coefficients used throughout the libgolf library. Constants
 * are organized by category for easy reference.
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
    constexpr float INCHES_PER_METER = 39.37F;

    // ========================================================================
    // VELOCITY CONVERSION FACTORS
    // ========================================================================

    /// Miles per hour to feet per second conversion factor
    /// 1 mph = 5280 ft / 3600 s = 1.46667 ft/s
    constexpr float MPH_TO_FT_PER_S = 1.467F;

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
    /// 1 inHg = 1000/39.37 mmHg ≈ 25.4 mmHg
    constexpr float INHG_TO_MMHG = 1000.0F / 39.37F;

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
    // GOLF BALL DRAG FORCE CONSTANTS
    // ========================================================================

    /// Drag force constant
    /// Derived from: c0 = 0.07182 * rho * (5.125/mass) * (circ/9.125)²
    /// where 5.125 oz and 9.125 in are reference ball parameters
    constexpr float DRAG_FORCE_CONST = 0.07182F;

    /// Reference golf ball mass for drag calculation (oz)
    constexpr float REF_BALL_MASS_OZ = 5.125F;

    /// Reference golf ball circumference for drag calculation (inches)
    constexpr float REF_BALL_CIRC_IN = 9.125F;

    // ========================================================================
    // SPIN DECAY CONSTANTS
    // ========================================================================

    /// Tau (spin decay time constant) coefficient
    /// tau = 1 / (TAU_COEFF * v / r)
    /// where v is velocity and r is ball radius
    constexpr float TAU_COEFF = 0.00002F;

    // ========================================================================
    // REYNOLDS NUMBER THRESHOLDS
    // ========================================================================
    // Used for determining drag coefficient regime

    /// Low Reynolds number threshold (× 10⁵)
    /// Below this, Cd = CdL (laminar flow dominated)
    constexpr float RE_THRESHOLD_LOW = 0.5F;

    /// High Reynolds number threshold (× 10⁵)
    /// Above this, Cd = CdH (turbulent flow dominated)
    constexpr float RE_THRESHOLD_HIGH = 1.0F;

    /// Reynolds number scaling factor for comparison with thresholds
    /// Re_scaled = (v_mph / 100) * Re100 * RE_SCALE_FACTOR
    constexpr float RE_SCALE_FACTOR = 0.00001F;

    /// Velocity divisor for Reynolds number calculation (mph)
    constexpr float RE_VELOCITY_DIVISOR = 100.0F;

    // ========================================================================
    // AERODYNAMIC COEFFICIENTS — DRAG
    // ========================================================================
    // Reference: Washington State University study by Bin Lyu, et al.

    /// Drag coefficient with spin effect
    constexpr float CD_SPIN = 0.180F;

    /// Drag coefficient for low Reynolds number (laminar flow)
    /// Used when Re <= 0.5e5
    constexpr float CD_LOW = 0.500F;

    /// Drag coefficient for high Reynolds number (turbulent flow)
    /// Used when Re >= 1.0e5
    constexpr float CD_HIGH = 0.200F;

    // ========================================================================
    // AERODYNAMIC COEFFICIENTS — LIFT (Re-binned)
    // ========================================================================
    // Reynolds-dependent Cl(S) curves, fit to Bearman/Harvey wind-tunnel data
    // for dimpled spheres. Library applies them in Re_x_e5 units (Re/1e5):
    //
    //   Re_x_e5 <= 0.3 :       Cl = 0   (no measurable lift below ~30k)
    //   0.3 < Re_x_e5 < 0.5 :  smoothstep ramp toward Cl_50k
    //   0.5 <= Re_x_e5 <= 0.7: lerp between adjacent bins {50k, 60k, 65k, 70k}
    //   Re_x_e5 > 0.7 :        Hill saturation Cl = ClMax · S·g / (1 + S·g)
    //
    // The bins replace the legacy single-quadratic Cl(S) — that overestimated
    // lift on slow shots (chip / short iron, Re ~50k) where the real curve
    // peaks early then drops sharply.

    /// Reynolds-bin sentinels (Re_x_e5 = Re / 1e5).
    constexpr float RE_BIN_NO_LIFT_X_E5 = 0.3F;  ///< below: Cl = 0
    constexpr float RE_BIN_LOW_X_E5     = 0.5F;  ///< Re = 50,000
    constexpr float RE_BIN_MID_LOW_X_E5 = 0.6F;  ///< Re = 60,000
    constexpr float RE_BIN_MID_HIGH_X_E5 = 0.65F; ///< Re = 65,000
    constexpr float RE_BIN_HIGH_X_E5    = 0.7F;  ///< Re = 70,000

    /// Cl(S) cubic at Re = 50,000 (fits Bearman 50k bin).
    constexpr float CL_RE50K_A0 =  0.0472121F;
    constexpr float CL_RE50K_A1 =  2.84795F;
    constexpr float CL_RE50K_A2 = -23.4342F;
    constexpr float CL_RE50K_A3 =  45.4849F;

    /// Cl(S) quadratic at Re = 60,000.
    constexpr float CL_RE60K_A0 =  0.320524F;
    constexpr float CL_RE60K_A1 = -4.7032F;
    constexpr float CL_RE60K_A2 = 14.0613F;

    /// Cl(S) quadratic at Re = 65,000.
    constexpr float CL_RE65K_A0 =  0.266667F;
    constexpr float CL_RE65K_A1 = -4.0F;
    constexpr float CL_RE65K_A2 = 13.3333F;

    /// Cl(S) quadratic at Re = 70,000.
    constexpr float CL_RE70K_A0 =  0.0496189F;
    constexpr float CL_RE70K_A1 =  0.00211396F;
    constexpr float CL_RE70K_A2 =  2.34201F;

    /// Spin-factor-dependent maximum lift coefficient.
    /// Cap on bin output and Hill saturation. The Bearman dimpled-sphere data
    /// shows a higher asymptote at extreme spin (S → 0.5) than at moderate
    /// spin (S ≤ 0.35). A flat cap clipped wedge / short-iron lift.
    /// ClMax(S) is a piecewise-linear lerp:
    ///   S ≤ CL_MAX_SR_LERP_LOW  : ClMax = CL_MAX_BASE   (0.268)
    ///   S ≥ CL_MAX_SR_LERP_HIGH : ClMax = CL_MAX_HIGH_SR (0.320)
    ///   between                 : linear interpolation
    constexpr float CL_MAX_BASE       = 0.268F;
    constexpr float CL_MAX_HIGH_SR    = 0.320F;
    constexpr float CL_MAX_SR_LERP_LOW  = 0.35F;
    constexpr float CL_MAX_SR_LERP_HIGH = 0.50F;

    /// Spin gain in the high-Re Hill saturation Cl = ClMax·S·g / (1 + S·g).
    constexpr float HIGH_RE_SPIN_GAIN = 16.0F;

    // ========================================================================
    // SIMULATION PARAMETERS
    // ========================================================================

    /// Time step for numerical integration (seconds)
    constexpr float SIMULATION_TIME_STEP = 0.01F;

    /// Integration coefficient (0.5 for Euler's method)
    constexpr float HALF = 0.5F;

    // ========================================================================
    // NUMERICAL STABILITY THRESHOLDS
    // ========================================================================

    /// Minimum velocity magnitude to avoid division by zero in calculations (ft/s)
    /// Used in spin factor and friction calculations
    constexpr float MIN_VELOCITY_THRESHOLD = 0.01F;

    /// Minimum horizontal velocity for friction application (ft/s)
    /// Below this, horizontal velocity is treated as zero to prevent numerical instability
    constexpr float MIN_HORIZONTAL_VELOCITY = 0.0001F;

    // ========================================================================
    // PHASE TRANSITION THRESHOLDS
    // ========================================================================

    /// Minimum vertical velocity for bounce transition (ft/s)
    /// Below this, ball transitions from bounce to roll phase
    constexpr float MIN_BOUNCE_VELOCITY = 1.0F;

    /// Minimum impact speed (ft/s) for Penner spin-back tangential model.
    /// Below this, simple friction retention is used regardless of impact
    /// angle — prevents non-physical spin-back on low-energy chip shots.
    /// 20 m/s ≈ 65.617 ft/s. Reference: Penner (2003) regime where the
    /// rigid-body slip / no-slip transition matches measured wedge bounces.
    constexpr float MIN_PENNER_BOUNCE_SPEED_FT_PER_S = 20.0F * METERS_TO_FEET;

    // ========================================================================
    // BOUNCE COR — SPIN/VELOCITY MODULATION
    // ========================================================================
    // Effective COR = surface.restitution * (1 - reduction), where
    //   reduction = maxReduction(rpm) * velocityScale(speedNormal_m/s).
    // High spin causes the ball to bite into turf rather than spring off
    // (flop / wedge bite). The velocity scale prevents low-energy chip
    // shots from getting an unwarranted COR penalty just from carried spin.
    // Reference: openfairway BounceCalculator.cs:215-250.

    /// Spin RPM at/below which there is no COR reduction. The reduction
    /// ramps linearly from 0 here to BOUNCE_COR_SPIN_LOW_MAX_REDUCTION at
    /// BOUNCE_COR_SPIN_KNEE_RPM.
    constexpr float BOUNCE_COR_SPIN_KNEE_RPM = 1500.0F;

    /// Above the knee, additional spin extends the reduction up to
    /// BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION at BOUNCE_COR_SPIN_KNEE_RPM +
    /// BOUNCE_COR_SPIN_HIGH_BAND_RPM (default: 3000 rpm).
    constexpr float BOUNCE_COR_SPIN_HIGH_BAND_RPM = 1500.0F;

    /// Reduction at the knee (low-spin asymptote).
    constexpr float BOUNCE_COR_SPIN_LOW_MAX_REDUCTION = 0.30F;

    /// Reduction at saturation (high-spin asymptote).
    constexpr float BOUNCE_COR_SPIN_HIGH_MAX_REDUCTION = 0.70F;

    /// Normal-component impact speed (m/s) below which the velocity scale
    /// ramps from 0 to BOUNCE_COR_VEL_MID_SCALE.
    constexpr float BOUNCE_COR_VEL_LOW_MS = 12.0F;

    /// Velocity scale at BOUNCE_COR_VEL_LOW_MS — partial COR penalty.
    constexpr float BOUNCE_COR_VEL_MID_SCALE = 0.50F;

    /// At/above this normal speed the full COR penalty applies.
    constexpr float BOUNCE_COR_VEL_HIGH_MS = 25.0F;

    // ========================================================================
    // BOUNCE RETENTION (PENNER BRANCH) — SPIN-COUPLED
    // ========================================================================
    // The `retention` multiplier in v_t' = retention * |v| * sin(θ - θ_crit)
    // is reduced as spin grows: higher spin loses more forward push to bite,
    // tightening wedge total-distance. Reference: openfairway
    // 0.55 * clamp(1 - rpm/8000, 0.4, 1.0).

    constexpr float BOUNCE_RETENTION_BASE = 0.55F;
    constexpr float BOUNCE_RETENTION_RPM_NORM = 8000.0F;
    constexpr float BOUNCE_RETENTION_FLOOR = 0.40F;

    /// Ground contact threshold for phase detection (ft)
    /// Ball is considered "on ground" when within this distance
    constexpr float GROUND_CONTACT_THRESHOLD = 0.1F;

    /// Position comparison tolerance for cache validation
    /// Prevents cache misses from floating-point precision issues
    constexpr float POSITION_EPSILON = 1e-6F;

    /// Flat surface threshold for slope calculations (cosine of angle)
    /// Surfaces with cos(θ) > this value (~2.5 degrees) use simplified physics
    constexpr float FLAT_SURFACE_THRESHOLD = 0.999F;

    /// Stopping velocity threshold for roll phase (ft/s)
    /// Below this, ball is considered stopped and simulation completes
    constexpr float STOPPING_VELOCITY = 0.1F;

    // ========================================================================
    // SPIN DECAY DURING ROLLING
    // ========================================================================

    /// Spin decay rate during rolling phase (rad/s per second)
    /// Ground friction causes faster spin decay than in air
    constexpr float ROLL_SPIN_DECAY_RATE = 2.0F;

} // namespace physics_constants

#endif // PHYSICS_CONSTANTS_HPP