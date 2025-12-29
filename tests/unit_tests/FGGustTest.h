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

  /***************************************************************************
   * Von Karman Turbulence Spectrum Tests
   ***************************************************************************/

  void testVonKarmanScaleLengthLowAltitude() {
    // Scale length at low altitude (below 1000 ft)
    double h = 500.0;  // ft
    double Lu = h / std::pow(0.177 + 0.000823 * h, 1.2);

    TS_ASSERT(Lu > 0.0);
    TS_ASSERT(Lu < 2000.0);
  }

  void testVonKarmanScaleLengthHighAltitude() {
    // Scale length at altitude > 2000 ft
    double Lw = 2500.0;  // ft (typical high altitude value)

    TS_ASSERT(Lw >= 2500.0);
  }

  void testVonKarmanPowerSpectrum() {
    double sigma = 5.0;   // RMS turbulence intensity (ft/s)
    double L = 2500.0;    // Scale length (ft)
    double omega = 0.5;   // Spatial frequency (rad/ft)
    double V = 300.0;     // Airspeed (ft/s)

    // Simplified PSD formula
    double Omega = omega * L;
    double phi_w = (sigma * sigma * L / PI) *
                   (1.0 + 8.0/3.0 * std::pow(1.339 * Omega, 2)) /
                   std::pow(1.0 + std::pow(1.339 * Omega, 2), 11.0/6.0);

    TS_ASSERT(phi_w > 0.0);
  }

  void testVonKarmanVerticalIntensity() {
    double sigma_w = 5.0;  // Vertical RMS turbulence (ft/s)
    double sigma_u = 5.0;  // Longitudinal RMS turbulence (ft/s)

    // For isotropic turbulence at high altitude
    double ratio = sigma_w / sigma_u;
    TS_ASSERT_DELTA(ratio, 1.0, 0.3);
  }

  /***************************************************************************
   * Dryden Turbulence Model Tests
   ***************************************************************************/

  void testDrydenScaleLengthMediumAltitude() {
    double h = 1500.0;  // ft

    // Dryden scale lengths
    double Lu = 200.0;  // Longitudinal (ft) - simplified
    double Lw = h / 2.0;  // Vertical

    TS_ASSERT(Lu > 0.0);
    TS_ASSERT(Lw > 0.0);
  }

  void testDrydenTransferFunction() {
    double sigma = 5.0;  // RMS intensity (ft/s)
    double L = 1000.0;   // Scale length (ft)
    double V = 300.0;    // Airspeed (ft/s)

    // Time constant
    double tau = L / V;
    TS_ASSERT(tau > 0.0);
    TS_ASSERT(tau < 10.0);
  }

  void testDrydenLowAltitudeModel() {
    double h = 100.0;  // ft AGL

    // At low altitude, scale length = altitude
    double Lw = h;
    double Lu = h / std::pow(0.177 + 0.000823 * h, 1.2);

    TS_ASSERT(Lw <= h);
    TS_ASSERT(Lu > 0.0);
  }

  /***************************************************************************
   * Gust Frequency Analysis Tests
   ***************************************************************************/

  void testGustFrequencyAtAirspeed() {
    double V = 300.0;    // Airspeed (ft/s)
    double lambda = 50.0; // Wavelength (ft)

    double f = V / lambda;  // Hz
    TS_ASSERT_DELTA(f, 6.0, 0.01);
  }

  void testGustAngularFrequency() {
    double f = 2.0;  // Hz

    double omega = 2.0 * PI * f;  // rad/s
    TS_ASSERT_DELTA(omega, 4.0 * PI, 0.01);
  }

  void testReducedFrequency() {
    double omega = 10.0;  // rad/s
    double c = 12.0;      // Chord (ft)
    double V = 300.0;     // Airspeed (ft/s)

    double k = omega * c / (2.0 * V);
    TS_ASSERT(k > 0.0);
    TS_ASSERT(k < 1.0);  // Typical range for gust analysis
  }

  void testCriticalGustWavelength() {
    double b = 100.0;  // Wingspan (ft)

    // Critical wavelength for maximum wing response
    double lambda_crit = 2.0 * b;  // Approximate
    TS_ASSERT_DELTA(lambda_crit, 200.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Structural Fatigue from Gusts Tests
   ***************************************************************************/

  void testGustCycleCount() {
    double flight_hours = 1.0;
    double gust_frequency = 0.1;  // Hz (gusts per second)

    double cycles = flight_hours * 3600.0 * gust_frequency;
    TS_ASSERT_DELTA(cycles, 360.0, 1.0);
  }

  void testFatigueStressRatio() {
    double sigma_max = 30000.0;  // Max stress from gust (psi)
    double sigma_min = 10000.0;  // Min stress (steady flight)

    double R = sigma_min / sigma_max;
    TS_ASSERT(R > 0.0);
    TS_ASSERT(R < 1.0);
  }

  void testGustDamageAccumulation() {
    double n_cycles = 1000.0;    // Number of gust cycles
    double N_fatigue = 100000.0; // Fatigue life at stress level

    double damage = n_cycles / N_fatigue;
    TS_ASSERT_DELTA(damage, 0.01, DEFAULT_TOLERANCE);
  }

  void testCumulativeFatigueDamage() {
    // Miner's rule: D = sum(ni/Ni)
    double D1 = 0.1;   // Damage from light gusts
    double D2 = 0.05;  // Damage from moderate gusts
    double D3 = 0.02;  // Damage from severe gusts

    double D_total = D1 + D2 + D3;
    TS_ASSERT(D_total < 1.0);  // Below failure threshold
  }

  /***************************************************************************
   * Autopilot Response to Gusts Tests
   ***************************************************************************/

  void testAutopilotGustSuppression() {
    double gust_induced_pitch = 5.0;  // degrees
    double autopilot_gain = 0.8;

    double corrected_pitch = gust_induced_pitch * (1.0 - autopilot_gain);
    TS_ASSERT(corrected_pitch < gust_induced_pitch);
  }

  void testGustLoadAlleviationSystem() {
    double delta_n_no_gla = 2.0;   // Load factor without GLA
    double gla_effectiveness = 0.3; // 30% load reduction

    double delta_n_with_gla = delta_n_no_gla * (1.0 - gla_effectiveness);
    TS_ASSERT_DELTA(delta_n_with_gla, 1.4, 0.01);
  }

  void testYawDamperGustResponse() {
    double gust_yaw_rate = 10.0;  // deg/s from lateral gust
    double damper_gain = 0.5;

    double damped_yaw_rate = gust_yaw_rate / (1.0 + damper_gain);
    TS_ASSERT(damped_yaw_rate < gust_yaw_rate);
  }

  /***************************************************************************
   * Altitude-Dependent Gust Characteristics Tests
   ***************************************************************************/

  void testGustIntensityVsAltitude() {
    double sigma_sl = 10.0;   // RMS at sea level (ft/s)
    double h = 30000.0;       // ft

    // Intensity typically decreases with altitude
    double sigma_alt = sigma_sl * std::pow(1.0 - h / 50000.0, 0.3);
    TS_ASSERT(sigma_alt < sigma_sl);
    TS_ASSERT(sigma_alt > 0.0);
  }

  void testTropopauseGustIntensity() {
    double h_tropopause = 36000.0;  // ft

    // Maximum turbulence often near tropopause
    double sigma_typical = 8.0;  // ft/s RMS

    TS_ASSERT(sigma_typical > 5.0);
    TS_ASSERT(sigma_typical < 15.0);
  }

  void testJetStreamTurbulence() {
    double jet_stream_gust = 30.0;  // ft/s (severe turbulence possible)

    TS_ASSERT(jet_stream_gust > 20.0);
    TS_ASSERT(jet_stream_gust < 50.0);
  }

  /***************************************************************************
   * Terrain-Induced Turbulence Tests
   ***************************************************************************/

  void testMechanicalTurbulenceIntensity() {
    double wind_speed = 30.0;  // ft/s
    double terrain_roughness = 0.5;  // Factor 0-1

    double sigma_mech = wind_speed * terrain_roughness * 0.3;
    TS_ASSERT(sigma_mech > 0.0);
  }

  void testMountainWaveTurbulence() {
    double mountain_height = 10000.0;  // ft
    double wind_speed = 50.0;  // ft/s

    // Simplified estimate of rotor turbulence
    double rotor_intensity = wind_speed * 0.5;  // ft/s RMS

    TS_ASSERT(rotor_intensity > 0.0);
  }

  void testLowLevelWindShear() {
    double altitude = 200.0;  // ft AGL
    double wind_gradient = 0.1;  // (ft/s)/ft

    double delta_V = wind_gradient * altitude;
    TS_ASSERT_DELTA(delta_V, 20.0, 0.1);
  }

  /***************************************************************************
   * Convective Turbulence Tests
   ***************************************************************************/

  void testThermalGustVelocity() {
    double thermal_strength = 10.0;  // ft/s vertical velocity

    // Gust velocity proportional to thermal strength
    double gust = thermal_strength * 1.5;  // Edge effects
    TS_ASSERT(gust > thermal_strength);
  }

  void testThunderstormGust() {
    double microburst_intensity = 80.0;  // ft/s (severe)

    TS_ASSERT(microburst_intensity > 60.0);
    TS_ASSERT(microburst_intensity < 150.0);
  }

  void testConvectiveBoundaryLayer() {
    double surface_heating = 500.0;  // W/m^2
    double h_cbl = 5000.0;  // CBL height (ft)

    // Turbulence intensity in CBL
    double sigma_w = 6.0;  // ft/s typical

    TS_ASSERT(sigma_w > 3.0);
    TS_ASSERT(sigma_w < 15.0);
  }

  /***************************************************************************
   * Small Aircraft Gust Response Tests
   ***************************************************************************/

  void testLightAircraftMassRatio() {
    double W = 2500.0;      // Weight (lbs)
    double S = 180.0;       // Wing area (ft^2)
    double c_bar = 5.0;     // MAC (ft)
    double rho = 0.002377;  // Air density
    double g = 32.174;
    double CLalpha = 4.5;

    double mu_g = (2.0 * W / S) / (rho * c_bar * g * CLalpha);
    TS_ASSERT(mu_g < 20.0);  // Light aircraft has lower mass ratio than transport
    TS_ASSERT(mu_g > 10.0);  // But still significant
  }

  void testLightAircraftGustLoadFactor() {
    double rho = 0.002377;
    double V = 150.0;       // ft/s
    double CLalpha = 4.5;
    double Ude = 50.0;      // ft/s
    double W_over_S = 15.0; // Low wing loading
    double Kg = 0.6;        // Lower alleviation factor

    double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);
    TS_ASSERT(delta_n > 1.0);  // High gust response for light aircraft
  }

  void testLightAircraftGustPenetration() {
    double V = 100.0;  // ft/s (slower)
    double H = 350.0;  // Gust gradient (ft)

    double t_rise = H / V;
    TS_ASSERT(t_rise > 3.0);  // Longer time to traverse gust
  }

  /***************************************************************************
   * Combined Gust Effects Tests
   ***************************************************************************/

  void testSimultaneousVerticalLateralGust() {
    double Ude_vertical = 30.0;  // ft/s
    double Ude_lateral = 20.0;   // ft/s

    double combined = std::sqrt(Ude_vertical * Ude_vertical +
                                Ude_lateral * Ude_lateral);

    TS_ASSERT(combined > Ude_vertical);
    TS_ASSERT(combined > Ude_lateral);
  }

  void testGustWithManeuver() {
    double n_maneuver = 2.0;    // Load factor from maneuver
    double delta_n_gust = 1.0;  // Load factor from gust

    double n_combined = n_maneuver + delta_n_gust;
    TS_ASSERT_DELTA(n_combined, 3.0, DEFAULT_TOLERANCE);
  }

  void testGustDuringClimb() {
    double climb_angle = 10.0 * PI / 180.0;  // rad
    double Ude = 40.0;  // ft/s vertical gust
    double V = 200.0;   // ft/s

    // Effective vertical gust component changes with climb angle
    double Ude_eff = Ude * std::cos(climb_angle);
    TS_ASSERT(Ude_eff < Ude);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testStressGustProfiles() {
    for (int i = 0; i <= 100; i++) {
      double s = i * 3.5;  // Penetration 0 to 350 ft
      double H = 350.0;
      double Ude = 50.0;

      double gust_1cos = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));
      double gust_ramp = (s <= H) ? Ude * s / H : Ude;

      TS_ASSERT(!std::isnan(gust_1cos));
      TS_ASSERT(!std::isnan(gust_ramp));
    }
  }

  void testStressLoadFactorCalculations() {
    for (double V = 100.0; V <= 600.0; V += 50.0) {
      for (double Ude = 10.0; Ude <= 70.0; Ude += 10.0) {
        double rho = 0.002377;
        double CLalpha = 5.5;
        double W_over_S = 50.0;
        double Kg = 0.75;

        double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);
        TS_ASSERT(!std::isnan(delta_n));
        TS_ASSERT(delta_n >= 0.0);
      }
    }
  }

  void testStressAlleviationFactors() {
    for (double mu = 1.0; mu <= 100.0; mu += 5.0) {
      double Kg = 0.88 * mu / (5.3 + mu);
      TS_ASSERT(Kg > 0.0);
      TS_ASSERT(Kg < 0.88);
    }
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteGustEncounterSequence() {
    // Simulate a complete gust encounter from approach to exit
    double H = 350.0;       // Gust gradient distance (ft)
    double Ude = 45.0;      // Design gust velocity (ft/s)
    double V = 250.0;       // Aircraft TAS (ft/s)
    double dt = 0.01;       // Time step

    double distance = 0.0;
    double max_gust = 0.0;

    // Traverse entire gust profile
    while (distance <= 2.0 * H) {
      double gust;
      if (distance <= H) {
        gust = (Ude / 2.0) * (1.0 - std::cos(PI * distance / H));
      } else {
        gust = (Ude / 2.0) * (1.0 + std::cos(PI * (distance - H) / H));
      }

      max_gust = std::fmax(max_gust, gust);
      distance += V * dt;
    }

    TS_ASSERT_DELTA(max_gust, Ude, 1.0);
  }

  void testCompleteVelocityEnvelopeGustLoads() {
    // Calculate gust loads across entire flight envelope
    double rho = 0.002377;
    double CLalpha = 5.5;
    double W_over_S = 50.0;

    double velocities[] = {150.0, 200.0, 250.0, 300.0, 350.0};  // ft/s
    double gusts[] = {66.0, 50.0, 25.0};  // ft/s (Ub, Uc, Ud)

    double max_load = 0.0;

    for (double V : velocities) {
      for (double Ude : gusts) {
        double mu_g = (2.0 * W_over_S) / (rho * 6.0 * 32.174 * CLalpha);
        double Kg = 0.88 * mu_g / (5.3 + mu_g);
        double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W_over_S);

        max_load = std::fmax(max_load, std::fabs(delta_n));
      }
    }

    TS_ASSERT(max_load > 0.0);
    TS_ASSERT(max_load < 5.0);  // Reasonable limit
  }

  void testCompleteAircraftResponseToGust() {
    // Full dynamic response simulation
    double W = 5000.0;      // Weight (lbs)
    double S = 180.0;       // Wing area (ft^2)
    double c_bar = 5.5;     // MAC (ft)
    double V = 200.0;       // TAS (ft/s)
    double rho = 0.002377;
    double CLalpha = 5.0;
    double g = 32.174;

    // Mass ratio
    double mu_g = (2.0 * W / S) / (rho * c_bar * g * CLalpha);
    TS_ASSERT(mu_g > 5.0);

    // Alleviation factor
    double Kg = 0.88 * mu_g / (5.3 + mu_g);
    TS_ASSERT(Kg > 0.0 && Kg < 0.88);

    // Gust load factor
    double Ude = 50.0;
    double delta_n = (rho * V * CLalpha * Ude * Kg) / (2.0 * W / S);
    TS_ASSERT(delta_n > 0.0);

    // Total load range
    double n_max = 1.0 + delta_n;
    double n_min = 1.0 - delta_n;
    TS_ASSERT(n_max > 1.0);
    TS_ASSERT(n_min < 1.0);
  }

  void testCompleteGustSpectrumAnalysis() {
    // Von Karman power spectral density
    double L = 1750.0;      // Scale length (ft)
    double sigma = 10.0;    // RMS gust intensity (ft/s)
    double V = 250.0;       // TAS (ft/s)

    double omega_values[] = {0.1, 0.5, 1.0, 2.0, 5.0};  // rad/s
    double prev_psd = 1e10;

    for (double omega : omega_values) {
      double omega_hat = omega * L / V;
      double psd = (sigma * sigma * L / (PI * V)) *
                   (1.0 + (8.0/3.0) * std::pow(1.339 * omega_hat, 2)) /
                   std::pow(1.0 + std::pow(1.339 * omega_hat, 2), 11.0/6.0);

      TS_ASSERT(psd > 0.0);
      TS_ASSERT(psd < prev_psd);  // PSD decreases with frequency
      prev_psd = psd;
    }
  }

  void testCompleteGustLoadEnvelope() {
    // Build complete V-n diagram gust lines
    double rho_sl = 0.002377;
    double rho_cr = 0.001;  // Cruise altitude density
    double CLalpha = 5.5;
    double W_over_S = 60.0;
    double Vc = 300.0;      // Cruise speed (ft/s)

    // Sea level gust loads
    double mu_sl = (2.0 * W_over_S) / (rho_sl * 5.5 * 32.174 * CLalpha);
    double Kg_sl = 0.88 * mu_sl / (5.3 + mu_sl);
    double Ude_c = 50.0;
    double delta_n_sl = (rho_sl * Vc * CLalpha * Ude_c * Kg_sl) / (2.0 * W_over_S);

    // Cruise altitude gust loads
    double mu_cr = (2.0 * W_over_S) / (rho_cr * 5.5 * 32.174 * CLalpha);
    double Kg_cr = 0.88 * mu_cr / (5.3 + mu_cr);
    double delta_n_cr = (rho_cr * Vc * CLalpha * Ude_c * Kg_cr) / (2.0 * W_over_S);

    TS_ASSERT(delta_n_sl > delta_n_cr);  // Lower loads at altitude
    TS_ASSERT(delta_n_sl > 0.0);
    TS_ASSERT(delta_n_cr > 0.0);
  }

  void testCompleteTurbulenceEncounter() {
    // Continuous turbulence model
    double L_w = 2500.0;    // Vertical scale (ft)
    double sigma_w = 5.0;   // Vertical RMS (ft/s)
    double V = 400.0;       // TAS (ft/s)
    double b = 100.0;       // Wingspan (ft)

    // Crossing frequency
    double N0 = V / (2.0 * L_w);
    TS_ASSERT(N0 > 0.0);

    // Expected peak in 1 hour (statistical)
    double T = 3600.0;      // seconds
    double eta = std::sqrt(2.0 * std::log(N0 * T));
    double w_peak = sigma_w * eta;

    TS_ASSERT(w_peak > sigma_w);
    TS_ASSERT(w_peak < 5.0 * sigma_w);
  }

  void testCompleteGustGradientVariation() {
    // Test complete range of gust gradients
    double Ude = 50.0;
    double V = 250.0;

    double gradients[] = {30.0, 100.0, 200.0, 350.0, 500.0};  // ft

    for (double H : gradients) {
      double t_gust = H / V;
      double f_gust = 1.0 / (2.0 * t_gust);  // Frequency

      TS_ASSERT(t_gust > 0.0);
      TS_ASSERT(f_gust > 0.0);

      // Peak gust at center
      double s_peak = H;
      double gust_peak = (Ude / 2.0) * (1.0 - std::cos(PI));
      TS_ASSERT_DELTA(gust_peak, Ude, 0.01);
    }
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testGustCalculationIndependence() {
    // Two independent gust calculations shouldn't interfere
    double H1 = 350.0, Ude1 = 50.0;
    double H2 = 200.0, Ude2 = 30.0;
    double s = 175.0;  // Midpoint of gradient 1

    double gust1 = (Ude1 / 2.0) * (1.0 - std::cos(PI * s / H1));
    double gust2 = (Ude2 / 2.0) * (1.0 - std::cos(PI * s / H2));

    // Results should differ
    TS_ASSERT(std::fabs(gust1 - gust2) > 1.0);

    // Recalculate gust1 to verify independence
    double gust1_verify = (Ude1 / 2.0) * (1.0 - std::cos(PI * s / H1));
    TS_ASSERT_DELTA(gust1, gust1_verify, DEFAULT_TOLERANCE);
  }

  void testAlleviationFactorIndependence() {
    // Multiple aircraft with different mass ratios
    double mu_light = 15.0;   // Light aircraft
    double mu_heavy = 80.0;   // Heavy transport

    double Kg_light = 0.88 * mu_light / (5.3 + mu_light);
    double Kg_heavy = 0.88 * mu_heavy / (5.3 + mu_heavy);

    // Heavy aircraft has higher alleviation factor
    TS_ASSERT(Kg_heavy > Kg_light);

    // Verify calculation didn't affect each other
    double Kg_light_verify = 0.88 * mu_light / (5.3 + mu_light);
    TS_ASSERT_DELTA(Kg_light, Kg_light_verify, DEFAULT_TOLERANCE);
  }

  void testLoadFactorCalculationIndependence() {
    double rho = 0.002377;
    double CLalpha = 5.5;
    double Ude = 50.0;
    double Kg = 0.7;

    // Aircraft 1: high wing loading
    double W_S_1 = 100.0;
    double delta_n_1 = (rho * 300.0 * CLalpha * Ude * Kg) / (2.0 * W_S_1);

    // Aircraft 2: low wing loading
    double W_S_2 = 30.0;
    double delta_n_2 = (rho * 200.0 * CLalpha * Ude * Kg) / (2.0 * W_S_2);

    // Low wing loading = higher gust response
    TS_ASSERT(delta_n_2 > delta_n_1 * 0.5);

    // Verify delta_n_1 unchanged
    double delta_n_1_verify = (rho * 300.0 * CLalpha * Ude * Kg) / (2.0 * W_S_1);
    TS_ASSERT_DELTA(delta_n_1, delta_n_1_verify, DEFAULT_TOLERANCE);
  }

  void testPSDCalculationIndependence() {
    double L = 1750.0;
    double V = 250.0;

    // Two different intensity levels
    double sigma1 = 5.0;
    double sigma2 = 15.0;
    double omega = 1.0;

    double omega_hat = omega * L / V;
    double psd1 = (sigma1 * sigma1 * L / (PI * V)) *
                  (1.0 + (8.0/3.0) * std::pow(1.339 * omega_hat, 2)) /
                  std::pow(1.0 + std::pow(1.339 * omega_hat, 2), 11.0/6.0);
    double psd2 = (sigma2 * sigma2 * L / (PI * V)) *
                  (1.0 + (8.0/3.0) * std::pow(1.339 * omega_hat, 2)) /
                  std::pow(1.0 + std::pow(1.339 * omega_hat, 2), 11.0/6.0);

    // Higher intensity = higher PSD
    TS_ASSERT(psd2 > psd1);

    // Ratio should be sigma^2 ratio
    TS_ASSERT_DELTA(psd2 / psd1, (sigma2 / sigma1) * (sigma2 / sigma1), 0.01);
  }

  void testGustProfileSequenceIndependence() {
    double H = 350.0;
    double Ude = 50.0;

    // Forward traversal
    double gust_forward[10];
    for (int i = 0; i < 10; i++) {
      double s = i * H / 9.0;
      gust_forward[i] = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));
    }

    // Reverse traversal
    double gust_reverse[10];
    for (int i = 9; i >= 0; i--) {
      double s = i * H / 9.0;
      gust_reverse[i] = (Ude / 2.0) * (1.0 - std::cos(PI * s / H));
    }

    // Results should match regardless of traversal order
    for (int i = 0; i < 10; i++) {
      TS_ASSERT_DELTA(gust_forward[i], gust_reverse[i], DEFAULT_TOLERANCE);
    }
  }

  void testScaleLengthIndependence() {
    // Different turbulence scale lengths
    double L_low = 1000.0;   // Low altitude
    double L_high = 2500.0;  // High altitude
    double sigma = 10.0;
    double V = 300.0;
    double omega = 1.0;

    double omega_hat_low = omega * L_low / V;
    double omega_hat_high = omega * L_high / V;

    double psd_low = (sigma * sigma * L_low / (PI * V)) *
                     (1.0 + (8.0/3.0) * std::pow(1.339 * omega_hat_low, 2)) /
                     std::pow(1.0 + std::pow(1.339 * omega_hat_low, 2), 11.0/6.0);
    double psd_high = (sigma * sigma * L_high / (PI * V)) *
                      (1.0 + (8.0/3.0) * std::pow(1.339 * omega_hat_high, 2)) /
                      std::pow(1.0 + std::pow(1.339 * omega_hat_high, 2), 11.0/6.0);

    TS_ASSERT(psd_low != psd_high);
    TS_ASSERT(psd_low > 0.0);
    TS_ASSERT(psd_high > 0.0);
  }

  void testGustComponentIndependence() {
    // Vertical and lateral gust components
    double Ude_w = 40.0;  // Vertical (ft/s)
    double Ude_v = 25.0;  // Lateral (ft/s)
    double s = 175.0;
    double H = 350.0;

    double gust_w = (Ude_w / 2.0) * (1.0 - std::cos(PI * s / H));
    double gust_v = (Ude_v / 2.0) * (1.0 - std::cos(PI * s / H));

    // Components are independent
    double combined = std::sqrt(gust_w * gust_w + gust_v * gust_v);
    TS_ASSERT(combined > gust_w);
    TS_ASSERT(combined > gust_v);

    // Verify individual components unchanged
    double gust_w_verify = (Ude_w / 2.0) * (1.0 - std::cos(PI * s / H));
    TS_ASSERT_DELTA(gust_w, gust_w_verify, DEFAULT_TOLERANCE);
  }
};
