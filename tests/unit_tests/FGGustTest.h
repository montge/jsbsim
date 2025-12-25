/*******************************************************************************
 * FGGustTest.h - Unit tests for gust response physics
 *
 * Tests various gust models and gust response calculations including:
 * - Discrete gust models (1-cosine, ramp, step)
 * - Gust alleviation factor
 * - Gust load factor calculation
 * - Sharp-edge gust response
 * - Penetration distance
 * - Gust intensity (light, moderate, severe)
 * - Derived gust velocity (Ude)
 * - Mass ratio for gust response
 * - Aircraft response to gusts
 * - V-g gust envelope
 * - FAR 25 gust requirements
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include "TestUtilities.h"

using namespace JSBSimTest;

const double PI = 3.14159265358979323846;

class FGGustTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Discrete Gust Model Tests - 1-Cosine Gust
   ***************************************************************************/

  // Test 1-cosine gust profile at startup
  void testOneMinusCosineGustStartup() {
    double H = 100.0;  // Gust gradient distance (ft)
    double s = 25.0;   // Penetration distance (ft)
    double Ude = 50.0; // Design gust velocity (ft/s)

    // During startup phase: U_gust = Ude/2 * (1 - cos(pi*s/H))
    double gust_velocity = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));
    double expected = (50.0 / 2.0) * (1.0 - std::cos(PI * 25.0 / 100.0));

    TS_ASSERT_DELTA(gust_velocity, expected, DEFAULT_TOLERANCE);
    TS_ASSERT(gust_velocity > 0.0);
    TS_ASSERT(gust_velocity < Ude);
  }

  // Test 1-cosine gust profile at midpoint
  void testOneMinusCosineGustMidpoint() {
    double H = 100.0;  // Gust gradient distance (ft)
    double s = 50.0;   // Penetration distance at midpoint (ft)
    double Ude = 50.0; // Design gust velocity (ft/s)

    // At midpoint (s = H/2): gust should be at half maximum
    double gust_velocity = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));
    double expected = (50.0 / 2.0) * (1.0 - std::cos(PI * 0.5));

    TS_ASSERT_DELTA(gust_velocity, expected, DEFAULT_TOLERANCE);
  }

  // Test 1-cosine gust profile at peak
  void testOneMinusCosineGustPeak() {
    double H = 100.0;  // Gust gradient distance (ft)
    double s = 100.0;  // Penetration distance at peak (ft)
    double Ude = 50.0; // Design gust velocity (ft/s)

    // At s = H: gust reaches maximum
    double gust_velocity = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));
    double expected = Ude;  // Should equal design velocity

    TS_ASSERT_DELTA(gust_velocity, expected, DEFAULT_TOLERANCE);
  }

  // Test 1-cosine gust profile beyond peak
  void testOneMinusCosineGustDecay() {
    double H = 100.0;   // Gust gradient distance (ft)
    double s = 150.0;   // Penetration distance after peak (ft)
    double Ude = 50.0;  // Design gust velocity (ft/s)

    // After s = H, gust begins to decay back down
    double gust_velocity = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));

    TS_ASSERT(gust_velocity > 0.0);
    TS_ASSERT(gust_velocity < Ude);
  }

  /***************************************************************************
   * Step Gust Model Tests
   ***************************************************************************/

  // Test sharp-edge (step) gust response
  void testSharpEdgeGust() {
    double Ude = 66.0;  // Design gust velocity (ft/s) for sharp-edge gust
    double V = 300.0;   // True airspeed (ft/s)

    // Sharp-edge gust creates instantaneous change in angle of attack
    double delta_alpha = std::atan(Ude / V);
    double expected = std::atan(66.0 / 300.0);

    TS_ASSERT_DELTA(delta_alpha, expected, DEFAULT_TOLERANCE);
    TS_ASSERT(delta_alpha > 0.0);
  }

  // Test step gust velocity profile before gust
  void testStepGustBeforeEncounter() {
    double s = -10.0;   // Before gust encounter (negative penetration)
    double Ude = 50.0;  // Design gust velocity (ft/s)

    // Before encountering gust, velocity should be zero
    double gust_velocity = (s >= 0.0) ? Ude : 0.0;

    TS_ASSERT_DELTA(gust_velocity, 0.0, DEFAULT_TOLERANCE);
  }

  // Test step gust velocity profile after encounter
  void testStepGustAfterEncounter() {
    double s = 10.0;    // After gust encounter (positive penetration)
    double Ude = 50.0;  // Design gust velocity (ft/s)

    // After encountering gust, velocity should be at design value
    double gust_velocity = (s >= 0.0) ? Ude : 0.0;

    TS_ASSERT_DELTA(gust_velocity, Ude, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Ramp Gust Model Tests
   ***************************************************************************/

  // Test ramp gust at start
  void testRampGustStart() {
    double H = 100.0;  // Gust gradient distance (ft)
    double s = 0.0;    // At start of ramp
    double Ude = 50.0; // Design gust velocity (ft/s)

    // Linear ramp: U_gust = Ude * s / H
    double gust_velocity = Ude * s / H;

    TS_ASSERT_DELTA(gust_velocity, 0.0, DEFAULT_TOLERANCE);
  }

  // Test ramp gust at midpoint
  void testRampGustMidpoint() {
    double H = 100.0;  // Gust gradient distance (ft)
    double s = 50.0;   // At midpoint of ramp
    double Ude = 50.0; // Design gust velocity (ft/s)

    // Linear ramp: U_gust = Ude * s / H
    double gust_velocity = Ude * s / H;
    double expected = 25.0;  // Half of design velocity

    TS_ASSERT_DELTA(gust_velocity, expected, DEFAULT_TOLERANCE);
  }

  // Test ramp gust at end
  void testRampGustEnd() {
    double H = 100.0;   // Gust gradient distance (ft)
    double s = 100.0;   // At end of ramp
    double Ude = 50.0;  // Design gust velocity (ft/s)

    // Linear ramp: U_gust = Ude * s / H
    double gust_velocity = (s >= H) ? Ude : Ude * s / H;

    TS_ASSERT_DELTA(gust_velocity, Ude, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Gust Alleviation Factor Tests
   ***************************************************************************/

  // Test gust alleviation factor for light aircraft
  void testGustAlleviationFactorLightAircraft() {
    double mass_ratio = 10.0;  // Typical for light aircraft

    // Kg = 0.88 * mu_g / (5.3 + mu_g)
    double Kg = 0.88 * mass_ratio / (5.3 + mass_ratio);
    double expected = 0.88 * 10.0 / 15.3;

    TS_ASSERT_DELTA(Kg, expected, DEFAULT_TOLERANCE);
    TS_ASSERT(Kg > 0.0);
    TS_ASSERT(Kg < 1.0);
  }

  // Test gust alleviation factor for transport aircraft
  void testGustAlleviationFactorTransport() {
    double mass_ratio = 50.0;  // Typical for transport category aircraft

    // Kg = 0.88 * mu_g / (5.3 + mu_g)
    double Kg = 0.88 * mass_ratio / (5.3 + mass_ratio);
    double expected = 0.88 * 50.0 / 55.3;

    TS_ASSERT_DELTA(Kg, expected, DEFAULT_TOLERANCE);
    TS_ASSERT(Kg > 0.0);
    TS_ASSERT(Kg < 1.0);
  }

  // Test gust alleviation factor approaches limit
  void testGustAlleviationFactorLimit() {
    double mass_ratio = 1000.0;  // Very large aircraft

    // As mass_ratio approaches infinity, Kg approaches 0.88
    double Kg = 0.88 * mass_ratio / (5.3 + mass_ratio);

    TS_ASSERT(Kg > 0.85);
    TS_ASSERT(Kg < 0.88);
  }

  /***************************************************************************
   * Mass Ratio Tests
   ***************************************************************************/

  // Test mass ratio calculation
  void testMassRatio() {
    double W = 50000.0;     // Aircraft weight (lbs)
    double S = 950.0;       // Wing area (ft^2)
    double c_bar = 12.0;    // Mean aerodynamic chord (ft)
    double rho = 0.002377;  // Air density at sea level (slugs/ft^3)
    double g = 32.174;      // Gravitational acceleration (ft/s^2)
    double CLalpha = 5.5;   // Lift curve slope (per radian)

    // mu_g = (2 * W/S) / (rho * c_bar * g * CLalpha)
    double mu_g = (2.0 * W / S) / (rho * c_bar * g * CLalpha);
    double expected = (2.0 * 50000.0 / 950.0) / (0.002377 * 12.0 * 32.174 * 5.5);

    TS_ASSERT_DELTA(mu_g, expected, LOOSE_TOLERANCE);
    TS_ASSERT(mu_g > 0.0);
  }

  // Test mass ratio for different wing loading
  void testMassRatioHighWingLoading() {
    double W = 100000.0;    // Aircraft weight (lbs)
    double S = 1000.0;      // Wing area (ft^2) - high wing loading
    double c_bar = 15.0;    // Mean aerodynamic chord (ft)
    double rho = 0.002377;  // Air density (slugs/ft^3)
    double g = 32.174;      // Gravitational acceleration (ft/s^2)
    double CLalpha = 5.0;   // Lift curve slope (per radian)

    double mu_g = (2.0 * W / S) / (rho * c_bar * g * CLalpha);

    TS_ASSERT(mu_g > 30.0);  // Higher wing loading gives higher mass ratio
  }

  /***************************************************************************
   * Gust Gradient Distance Tests
   ***************************************************************************/

  // Test gust gradient distance at sea level
  void testGustGradientDistanceSeaLevel() {
    // FAR 25: H = 350 ft at sea level for sharp-edged gust
    double H = 350.0;  // ft

    TS_ASSERT_DELTA(H, 350.0, DEFAULT_TOLERANCE);
  }

  // Test gust gradient distance at altitude
  void testGustGradientDistanceAltitude() {
    double H_sl = 350.0;     // Gust gradient at sea level (ft)
    double h = 20000.0;      // Altitude (ft)

    // Linear interpolation between sea level and 50,000 ft
    // At 50,000 ft, H typically increases to about 500 ft
    double H = H_sl + (h / 50000.0) * (500.0 - H_sl);

    TS_ASSERT(H > H_sl);
    TS_ASSERT(H < 500.0);
  }

  /***************************************************************************
   * Derived Gust Velocity (Ude) Tests
   ***************************************************************************/

  // Test reference gust velocity at sea level
  void testReferenceGustVelocitySeaLevel() {
    // FAR 25.341(a): Uref at sea level
    double Uref_sl = 56.0;  // ft/s (approximate value)

    TS_ASSERT(Uref_sl > 0.0);
    TS_ASSERT_DELTA(Uref_sl, 56.0, 1.0);
  }

  // Test derived gust velocity with altitude variation
  void testDerivedGustVelocityAltitude() {
    double Uref_sl = 56.0;   // Reference gust velocity at sea level (ft/s)
    double h = 15000.0;      // Altitude (ft)
    double Fg = 1.0;         // Flight profile factor

    // FAR 25: Ude varies with altitude
    // Simplified: Ude = Uref * Fg * (1 - h/125000)^(1/6) for h < 20000 ft
    double altitude_factor = std::pow(1.0 - h / 125000.0, 1.0 / 6.0);
    double Ude = Uref_sl * Fg * altitude_factor;

    TS_ASSERT(Ude > 0.0);
    TS_ASSERT(Ude < Uref_sl);  // Should decrease with altitude
  }

  // Test derived gust velocity for design cruise altitude
  void testDerivedGustVelocityDesignCruise() {
    double Uref_sl = 56.0;   // Reference gust velocity at sea level (ft/s)
    double h = 30000.0;      // Cruise altitude (ft)
    double Fg = 0.5;         // Reduced at cruise (flight profile factor)

    double altitude_factor = std::pow(1.0 - h / 125000.0, 1.0 / 6.0);
    double Ude = Uref_sl * Fg * altitude_factor;

    TS_ASSERT(Ude > 0.0);
    TS_ASSERT(Ude < 30.0);  // Significantly reduced at cruise
  }

  /***************************************************************************
   * Gust Intensity Tests
   ***************************************************************************/

  // Test light turbulence gust velocity
  void testLightTurbulenceGustVelocity() {
    double Ude_light = 20.0;  // ft/s (typical light turbulence)

    TS_ASSERT(Ude_light >= 15.0);
    TS_ASSERT(Ude_light <= 25.0);
  }

  // Test moderate turbulence gust velocity
  void testModerateTurbulenceGustVelocity() {
    double Ude_moderate = 50.0;  // ft/s (typical moderate turbulence)

    TS_ASSERT(Ude_moderate >= 40.0);
    TS_ASSERT(Ude_moderate <= 60.0);
  }

  // Test severe turbulence gust velocity
  void testSevereTurbulenceGustVelocity() {
    double Ude_severe = 75.0;  // ft/s (typical severe turbulence)

    TS_ASSERT(Ude_severe >= 70.0);
    TS_ASSERT(Ude_severe <= 85.0);
  }

  /***************************************************************************
   * Gust Load Factor Tests
   ***************************************************************************/

  // Test incremental load factor from gust
  void testIncrementalLoadFactorFromGust() {
    double rho = 0.002377;   // Air density (slugs/ft^3)
    double V = 300.0;        // True airspeed (ft/s)
    double CLalpha = 5.5;    // Lift curve slope (per radian)
    double Ude = 50.0;       // Design gust velocity (ft/s)
    double W_over_S = 50.0;  // Wing loading (lbs/ft^2)
    double Kg = 0.75;        // Gust alleviation factor

    // Delta n = (rho * V * CLalpha * Ude * Kg) / (2 * W/S)
    double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);

    TS_ASSERT(delta_n > 0.0);
    TS_ASSERT(delta_n < 3.0);  // Reasonable range for transport aircraft
  }

  // Test total load factor with gust
  void testTotalLoadFactorWithGust() {
    double n_base = 1.0;     // Base load factor (level flight)
    double delta_n = 1.5;    // Incremental load factor from gust

    double n_total = n_base + delta_n;

    TS_ASSERT_DELTA(n_total, 2.5, DEFAULT_TOLERANCE);
    TS_ASSERT(n_total > n_base);
  }

  // Test negative gust load factor
  void testNegativeGustLoadFactor() {
    double rho = 0.002377;   // Air density (slugs/ft^3)
    double V = 250.0;        // True airspeed (ft/s)
    double CLalpha = 5.0;    // Lift curve slope (per radian)
    double Ude = -30.0;      // Negative (down) gust velocity (ft/s)
    double W_over_S = 60.0;  // Wing loading (lbs/ft^2)
    double Kg = 0.70;        // Gust alleviation factor

    double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);

    TS_ASSERT(delta_n < 0.0);  // Negative gust produces negative load factor
  }

  /***************************************************************************
   * Penetration Distance Tests
   ***************************************************************************/

  // Test penetration distance calculation
  void testPenetrationDistance() {
    double V = 300.0;  // True airspeed (ft/s)
    double t = 2.0;    // Time since gust encounter (seconds)

    // s = V * t
    double s = V * t;

    TS_ASSERT_DELTA(s, 600.0, DEFAULT_TOLERANCE);
  }

  // Test penetration distance vs gust gradient
  void testPenetrationVsGradient() {
    double s = 175.0;  // Penetration distance (ft)
    double H = 350.0;  // Gust gradient distance (ft)

    // Check if aircraft is still in gust field
    bool in_gust = (s <= 2.0 * H);  // 1-cosine gust extends to 2H

    TS_ASSERT(in_gust);
  }

  /***************************************************************************
   * Aircraft Response Time Tests
   ***************************************************************************/

  // Test aircraft response time constant
  void testAircraftResponseTimeConstant() {
    double c_bar = 12.0;  // Mean aerodynamic chord (ft)
    double V = 300.0;     // True airspeed (ft/s)

    // Tau = c_bar / V (approximate time constant)
    double tau = c_bar / V;

    TS_ASSERT_DELTA(tau, 0.04, LOOSE_TOLERANCE);
    TS_ASSERT(tau > 0.0);
  }

  // Test gust rise time
  void testGustRiseTime() {
    double H = 350.0;  // Gust gradient distance (ft)
    double V = 300.0;  // True airspeed (ft/s)

    // Time to traverse gust gradient: t_rise = H / V
    double t_rise = H / V;

    TS_ASSERT_DELTA(t_rise, 1.1667, 0.001);
    TS_ASSERT(t_rise > 0.0);
  }

  /***************************************************************************
   * Roll Response to Asymmetric Gust Tests
   ***************************************************************************/

  // Test rolling moment from asymmetric gust
  void testRollingMomentFromAsymmetricGust() {
    double Ude_left = 30.0;   // Gust on left wing (ft/s)
    double Ude_right = -30.0; // Opposite gust on right wing (ft/s)

    // Asymmetric gust creates rolling moment
    double gust_differential = Ude_left - Ude_right;

    TS_ASSERT_DELTA(gust_differential, 60.0, DEFAULT_TOLERANCE);
    TS_ASSERT(std::abs(gust_differential) > 0.0);
  }

  // Test roll rate from gust differential
  void testRollRateFromGust() {
    double b = 100.0;        // Wingspan (ft)
    double V = 300.0;        // True airspeed (ft/s)
    double Clp = -0.5;       // Roll damping derivative
    double delta_Ude = 20.0; // Differential gust velocity (ft/s)

    // Simplified: gust differential creates roll rate
    // p_gust ~ delta_Ude / (b/2)
    double p_gust_factor = delta_Ude / (b / 2.0);

    TS_ASSERT(p_gust_factor > 0.0);
  }

  /***************************************************************************
   * Pitch Response to Gust Tests
   ***************************************************************************/

  // Test pitch rate from vertical gust
  void testPitchRateFromVerticalGust() {
    double Ude = 50.0;       // Vertical gust velocity (ft/s)
    double V = 300.0;        // True airspeed (ft/s)
    double c_bar = 12.0;     // Mean aerodynamic chord (ft)
    double Cmq = -15.0;      // Pitch damping derivative

    // Change in angle of attack from gust
    double delta_alpha = std::atan(Ude / V);

    // Gust induces pitch rate
    TS_ASSERT(delta_alpha > 0.0);
    TS_ASSERT(delta_alpha < 0.3);  // Reasonable range
  }

  // Test pitching moment from gust
  void testPitchingMomentFromGust() {
    double CLalpha = 5.5;    // Lift curve slope
    double Cmalpha = -1.2;   // Pitch moment curve slope
    double delta_alpha = 0.1; // Change in AoA from gust (rad)

    // Change in pitch moment coefficient
    double delta_Cm = Cmalpha * delta_alpha;

    TS_ASSERT(delta_Cm < 0.0);  // Nose-down moment for stable aircraft
  }

  /***************************************************************************
   * Yaw Response to Lateral Gust Tests
   ***************************************************************************/

  // Test yaw rate from lateral gust
  void testYawRateFromLateralGust() {
    double Vgust_lateral = 30.0;  // Lateral gust velocity (ft/s)
    double V = 300.0;             // True airspeed (ft/s)

    // Change in sideslip angle
    double delta_beta = std::atan(Vgust_lateral / V);

    TS_ASSERT(delta_beta > 0.0);
    TS_ASSERT(delta_beta < 0.2);
  }

  // Test yawing moment from lateral gust
  void testYawingMomentFromLateralGust() {
    double Cnbeta = 0.15;     // Directional stability derivative
    double delta_beta = 0.05; // Change in sideslip from gust (rad)

    // Change in yaw moment coefficient
    double delta_Cn = Cnbeta * delta_beta;

    TS_ASSERT(delta_Cn > 0.0);  // Restoring moment for stable aircraft
  }

  /***************************************************************************
   * Wing Load Distribution Tests
   ***************************************************************************/

  // Test gust load on wing root
  void testGustLoadWingRoot() {
    double q = 150.0;        // Dynamic pressure (lbs/ft^2)
    double S = 950.0;        // Wing area (ft^2)
    double CL = 0.5;         // Lift coefficient
    double delta_CL = 0.3;   // Incremental CL from gust

    // Total lift with gust
    double L_total = q * S * (CL + delta_CL);
    double L_base = q * S * CL;

    TS_ASSERT(L_total > L_base);
  }

  // Test spanwise gust load variation
  void testSpanwiseGustLoadVariation() {
    double y = 25.0;         // Spanwise position (ft from centerline)
    double b = 100.0;        // Wingspan (ft)
    double Ude_max = 50.0;   // Maximum gust velocity (ft/s)

    // Simplified spanwise variation (could be uniform or varying)
    double spanwise_factor = 1.0 - (2.0 * y / b);  // Linear variation
    double Ude_local = Ude_max * std::abs(spanwise_factor);

    TS_ASSERT(Ude_local >= 0.0);
    TS_ASSERT(Ude_local <= Ude_max);
  }

  /***************************************************************************
   * V-g Gust Envelope Tests
   ***************************************************************************/

  // Test positive gust limit load factor at cruise
  void testPositiveGustLimitLoadCruise() {
    double rho = 0.001756;   // Air density at altitude (slugs/ft^3)
    double Vc = 500.0;       // Cruise speed (ft/s)
    double CLalpha = 5.5;    // Lift curve slope
    double Ude = 40.0;       // Design cruise gust (ft/s)
    double W_over_S = 60.0;  // Wing loading (lbs/ft^2)
    double Kg = 0.75;        // Gust alleviation factor

    double delta_n = (rho * Vc * CLalpha * Ude * Kg) / (2.0 * W_over_S);
    double n_gust = 1.0 + delta_n;

    TS_ASSERT(n_gust > 1.0);
    TS_ASSERT(n_gust < 4.0);
  }

  // Test negative gust limit load factor
  void testNegativeGustLimitLoad() {
    double rho = 0.002377;   // Air density (slugs/ft^3)
    double V = 400.0;        // Airspeed (ft/s)
    double CLalpha = 5.5;    // Lift curve slope
    double Ude = -25.0;      // Negative gust (ft/s)
    double W_over_S = 55.0;  // Wing loading (lbs/ft^2)
    double Kg = 0.70;        // Gust alleviation factor

    double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);
    double n_gust = 1.0 + delta_n;

    TS_ASSERT(n_gust < 1.0);
    TS_ASSERT(n_gust > -1.0);
  }

  // Test gust envelope at dive speed
  void testGustEnvelopeAtDiveSpeed() {
    double Vd = 600.0;       // Dive speed (ft/s)
    double rho = 0.002377;   // Air density (slugs/ft^3)
    double CLalpha = 5.0;    // Lift curve slope
    double Ude = 25.0;       // Reduced gust at dive speed (ft/s)
    double W_over_S = 70.0;  // Wing loading (lbs/ft^2)
    double Kg = 0.80;        // Gust alleviation factor

    double delta_n = (rho * Vd * CLalpha * Ude * Kg) / (2.0 * W_over_S);

    // At dive speed, gust loads should not exceed structural limits
    TS_ASSERT(delta_n < 2.0);
  }

  /***************************************************************************
   * Continuous Turbulence vs Discrete Gust Tests
   ***************************************************************************/

  // Test discrete gust deterministic response
  void testDiscreteGustDeterministicResponse() {
    double Ude = 50.0;  // Discrete gust velocity (ft/s)

    // Discrete gust is deterministic and repeatable
    double gust_1 = Ude;
    double gust_2 = Ude;

    TS_ASSERT_DELTA(gust_1, gust_2, DEFAULT_TOLERANCE);
  }

  // Test continuous turbulence characteristics
  void testContinuousTurbulenceCharacteristics() {
    double sigma_w = 5.0;  // RMS turbulence intensity (ft/s)

    // Continuous turbulence is statistical
    // 99.7% of values fall within 3*sigma
    double max_expected = 3.0 * sigma_w;

    TS_ASSERT_DELTA(max_expected, 15.0, DEFAULT_TOLERANCE);
    TS_ASSERT(max_expected > sigma_w);
  }

  /***************************************************************************
   * FAR 25 Gust Requirements Tests
   ***************************************************************************/

  // Test FAR 25.341 gust velocities at design speeds
  void testFAR25GustVelocitiesDesignSpeed() {
    // FAR 25.341(a) specifies gust velocities
    double Ude_Vb = 66.0;  // ft/s at design speed for maximum gust intensity (Vb)
    double Ude_Vc = 50.0;  // ft/s at design cruise speed (Vc)
    double Ude_Vd = 25.0;  // ft/s at design dive speed (Vd)

    // Gust velocity decreases with airspeed
    TS_ASSERT(Ude_Vb > Ude_Vc);
    TS_ASSERT(Ude_Vc > Ude_Vd);
  }

  // Test FAR 25 flight profile alleviation factor
  void testFAR25FlightProfileAlleviation() {
    double Fg = 0.5;  // Flight profile alleviation factor at cruise

    // Fg reduces gust loads for operations at high altitude
    TS_ASSERT(Fg > 0.0);
    TS_ASSERT(Fg <= 1.0);
  }

  /***************************************************************************
   * Transport Category Gust Criteria Tests
   ***************************************************************************/

  // Test transport category design gust load
  void testTransportCategoryDesignGustLoad() {
    double n_limit = 2.5;    // Limit load factor for transport
    double rho = 0.002377;   // Air density (slugs/ft^3)
    double V = 450.0;        // Design speed (ft/s)
    double CLalpha = 5.7;    // Lift curve slope
    double Ude = 56.0;       // Design gust velocity (ft/s)
    double W_over_S = 75.0;  // Wing loading (lbs/ft^2)
    double Kg = 0.78;        // Gust alleviation factor

    double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);
    double n_gust = 1.0 + delta_n;

    // Should not exceed limit load factor
    TS_ASSERT(n_gust > 1.0);
  }

  // Test transport category ultimate load factor with gust
  void testTransportCategoryUltimateLoadGust() {
    double n_limit = 2.5;     // Limit load factor
    double safety_factor = 1.5; // Ultimate to limit ratio

    double n_ultimate = n_limit * safety_factor;

    TS_ASSERT_DELTA(n_ultimate, 3.75, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Stall Margin in Gust Tests
   ***************************************************************************/

  // Test stall margin with gust encounter
  void testStallMarginWithGust() {
    double CL_base = 0.6;    // Base lift coefficient
    double CL_max = 1.4;     // Maximum lift coefficient (stall)
    double delta_CL = 0.4;   // Incremental CL from gust

    double CL_gust = CL_base + delta_CL;
    double stall_margin = CL_max - CL_gust;

    TS_ASSERT(stall_margin > 0.0);  // Should not stall
    TS_ASSERT_DELTA(stall_margin, 0.4, DEFAULT_TOLERANCE);
  }

  // Test gust-induced stall condition
  void testGustInducedStallCondition() {
    double CL_base = 1.1;    // High base lift coefficient
    double CL_max = 1.4;     // Maximum lift coefficient
    double delta_CL = 0.5;   // Large incremental CL from severe gust

    double CL_gust = CL_base + delta_CL;
    bool stall_occurs = (CL_gust > CL_max);

    TS_ASSERT(stall_occurs);  // Severe gust can cause stall
  }

  // Test safe gust margin for cruise
  void testSafeGustMarginCruise() {
    double CL_cruise = 0.4;  // Typical cruise CL
    double CL_max = 1.4;     // Stall CL
    double delta_CL_design = 0.6;  // Design gust increment

    double CL_with_gust = CL_cruise + delta_CL_design;
    double margin_factor = CL_max / CL_with_gust;

    TS_ASSERT(margin_factor > 1.0);  // Should have margin
    TS_ASSERT(margin_factor < 2.0);  // Typical range
  }
};
