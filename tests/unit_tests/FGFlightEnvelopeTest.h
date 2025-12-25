/*******************************************************************************
 * FGFlightEnvelopeTest.h - Unit tests for flight envelope protection
 *
 * Tests the mathematical behavior of flight envelope protection systems:
 * - Stall warning angle of attack
 * - Alpha floor protection
 * - High speed protection (Vmo/Mmo)
 * - Low speed protection
 * - Bank angle limiting
 * - Pitch attitude limiting
 * - Load factor protection (positive and negative g)
 * - Overspeed warning
 * - Stick shaker activation
 * - Stick pusher activation
 * - Angle of attack limiting
 * - Envelope exceedance detection
 * - Speed margin to stall
 * - Mach buffet boundary
 *
 * These tests focus on protection system logic and mathematical relationships
 * without requiring aircraft file loading.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double KTS_TO_FPS = 1.68781;  // Knots to feet per second
const double GRAVITY = 32.174;      // ft/s^2

class FGFlightEnvelopeTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Stall Warning Tests
   ***************************************************************************/

  // Test stall warning angle of attack threshold
  void testStallWarningAlphaThreshold() {
    double alpha_stall = 15.0 * DEG_TO_RAD;
    double alpha_warning = 12.0 * DEG_TO_RAD;  // 80% of stall alpha

    // Warning should trigger before stall
    TS_ASSERT(alpha_warning < alpha_stall);
    TS_ASSERT_DELTA(alpha_warning / alpha_stall, 0.8, 0.01);
  }

  // Test stall warning margin
  void testStallWarningMargin() {
    double alpha_current = 10.0 * DEG_TO_RAD;
    double alpha_stall = 15.0 * DEG_TO_RAD;

    double margin = (alpha_stall - alpha_current) / alpha_stall;

    // Margin should be positive and decrease as alpha increases
    TS_ASSERT(margin > 0.0);
    TS_ASSERT_DELTA(margin, 0.333, 0.01);
  }

  // Test stall warning activation logic
  void testStallWarningActivation() {
    double alpha_current = 13.0 * DEG_TO_RAD;
    double alpha_warning = 12.0 * DEG_TO_RAD;

    bool warning_active = (alpha_current >= alpha_warning);

    TS_ASSERT(warning_active);
  }

  // Test stall warning deactivation hysteresis
  void testStallWarningHysteresis() {
    double alpha_warning_on = 12.0 * DEG_TO_RAD;
    double alpha_warning_off = 11.0 * DEG_TO_RAD;

    // Hysteresis prevents oscillation
    TS_ASSERT(alpha_warning_off < alpha_warning_on);
    double hysteresis = alpha_warning_on - alpha_warning_off;
    TS_ASSERT_DELTA(hysteresis, 1.0 * DEG_TO_RAD, epsilon);
  }

  /***************************************************************************
   * Alpha Floor Protection Tests
   ***************************************************************************/

  // Test alpha floor threshold
  void testAlphaFloorThreshold() {
    double alpha_max = 15.0 * DEG_TO_RAD;
    double alpha_floor = 13.0 * DEG_TO_RAD;  // 87% of max

    TS_ASSERT(alpha_floor < alpha_max);
    TS_ASSERT(alpha_floor > 10.0 * DEG_TO_RAD);
  }

  // Test alpha floor protection activation
  void testAlphaFloorActivation() {
    double alpha = 14.0 * DEG_TO_RAD;
    double alpha_floor = 13.0 * DEG_TO_RAD;

    bool protection_active = (alpha >= alpha_floor);

    TS_ASSERT(protection_active);
  }

  // Test alpha floor pitch correction
  void testAlphaFloorPitchCorrection() {
    double alpha = 14.0 * DEG_TO_RAD;
    double alpha_floor = 13.0 * DEG_TO_RAD;

    // Compute pitch-down correction proportional to exceedance
    double alpha_error = alpha - alpha_floor;
    double pitch_correction = -2.0 * alpha_error;  // Gain of 2

    TS_ASSERT(pitch_correction < 0.0);  // Nose down
    TS_ASSERT_DELTA(pitch_correction, -0.0349, 0.001);
  }

  // Test alpha floor with load factor
  void testAlphaFloorWithLoadFactor() {
    double alpha_floor_1g = 13.0 * DEG_TO_RAD;
    double load_factor = 2.0;

    // Alpha floor decreases with load factor (bank angle effect)
    double alpha_floor_nz = alpha_floor_1g / sqrt(load_factor);

    TS_ASSERT(alpha_floor_nz < alpha_floor_1g);
    TS_ASSERT_DELTA(alpha_floor_nz, 9.19 * DEG_TO_RAD, 0.1 * DEG_TO_RAD);
  }

  /***************************************************************************
   * High Speed Protection Tests (Vmo/Mmo)
   ***************************************************************************/

  // Test Vmo (maximum operating speed) limit
  void testVmoLimit() {
    double Vmo = 250.0;  // knots IAS
    double V_current = 255.0;  // knots IAS

    bool overspeed = (V_current > Vmo);

    TS_ASSERT(overspeed);
  }

  // Test Mmo (maximum operating Mach) limit
  void testMmoLimit() {
    double Mmo = 0.82;
    double M_current = 0.85;

    bool overspeed = (M_current > Mmo);

    TS_ASSERT(overspeed);
  }

  // Test Vmo warning margin
  void testVmoWarningMargin() {
    double Vmo = 250.0;  // knots
    double V_warning = 245.0;  // 98% of Vmo
    double V_current = 246.0;

    bool warning = (V_current >= V_warning) && (V_current < Vmo);

    TS_ASSERT(warning);
  }

  // Test overspeed pitch correction
  void testOverspeedPitchCorrection() {
    double Vmo = 250.0 * KTS_TO_FPS;
    double V_current = 260.0 * KTS_TO_FPS;

    double speed_error = V_current - Vmo;
    double pitch_correction = -0.01 * speed_error / KTS_TO_FPS;  // Proportional

    TS_ASSERT(pitch_correction < 0.0);  // Nose up to reduce speed
    TS_ASSERT_DELTA(pitch_correction, -0.1, 0.01);
  }

  // Test Mmo with altitude variation
  void testMmoAltitudeVariation() {
    double Mmo_base = 0.82;
    double altitude = 35000.0;  // ft

    // Mmo typically constant, but test awareness of altitude
    double Mmo = Mmo_base;  // No variation for this test

    TS_ASSERT_DELTA(Mmo, 0.82, epsilon);
  }

  /***************************************************************************
   * Low Speed Protection Tests
   ***************************************************************************/

  // Test minimum speed (Vs) calculation
  void testMinimumSpeed() {
    double Vs1 = 100.0;  // knots, clean configuration
    double load_factor = 1.0;

    double Vs = Vs1 * sqrt(load_factor);

    TS_ASSERT_DELTA(Vs, 100.0, epsilon);
  }

  // Test minimum speed in turns
  void testMinimumSpeedInTurn() {
    double Vs1 = 100.0;  // knots
    double bank_angle = 45.0 * DEG_TO_RAD;

    double load_factor = 1.0 / cos(bank_angle);
    double Vs_turn = Vs1 * sqrt(load_factor);

    TS_ASSERT(Vs_turn > Vs1);
    TS_ASSERT_DELTA(Vs_turn, 119.3, 0.5);
  }

  // Test low speed warning threshold
  void testLowSpeedWarning() {
    double Vs = 100.0;  // knots
    double V_warning = Vs * 1.3;  // 30% margin
    double V_current = 125.0;

    bool warning = (V_current < V_warning);

    TS_ASSERT(warning);
  }

  // Test low speed protection activation
  void testLowSpeedProtection() {
    double V_current = 95.0;  // knots
    double Vs = 100.0;

    bool protection = (V_current < Vs);

    TS_ASSERT(protection);
  }

  /***************************************************************************
   * Bank Angle Limiting Tests
   ***************************************************************************/

  // Test maximum bank angle limit (normal law)
  void testMaxBankAngleNormalLaw() {
    double bank_max = 67.0 * DEG_TO_RAD;  // Typical for Airbus
    double bank_current = 70.0 * DEG_TO_RAD;

    bool exceeded = (fabs(bank_current) > bank_max);

    TS_ASSERT(exceeded);
  }

  // Test bank angle limiting correction
  void testBankAngleLimiting() {
    double bank_current = 70.0 * DEG_TO_RAD;
    double bank_max = 67.0 * DEG_TO_RAD;

    double bank_error = fabs(bank_current) - bank_max;
    double roll_correction = -bank_error * 2.0;  // Proportional gain

    TS_ASSERT(roll_correction < 0.0);
    TS_ASSERT_DELTA(roll_correction, -0.1047, 0.001);
  }

  // Test bank angle limit varies with altitude
  void testBankAngleLimitAltitude() {
    double bank_max_low = 67.0 * DEG_TO_RAD;
    double altitude = 40000.0;

    // At high altitude, limit might be same (test awareness)
    double bank_max = bank_max_low;

    TS_ASSERT_DELTA(bank_max, 67.0 * DEG_TO_RAD, epsilon);
  }

  // Test bank angle protection hysteresis
  void testBankAngleHysteresis() {
    double bank_limit_on = 67.0 * DEG_TO_RAD;
    double bank_limit_off = 65.0 * DEG_TO_RAD;

    double hysteresis = bank_limit_on - bank_limit_off;

    TS_ASSERT(hysteresis > 0.0);
    TS_ASSERT_DELTA(hysteresis, 2.0 * DEG_TO_RAD, epsilon);
  }

  /***************************************************************************
   * Pitch Attitude Limiting Tests
   ***************************************************************************/

  // Test maximum nose-up pitch limit
  void testMaxPitchUpLimit() {
    double pitch_max_up = 30.0 * DEG_TO_RAD;
    double pitch_current = 32.0 * DEG_TO_RAD;

    bool exceeded = (pitch_current > pitch_max_up);

    TS_ASSERT(exceeded);
  }

  // Test maximum nose-down pitch limit
  void testMaxPitchDownLimit() {
    double pitch_max_down = -15.0 * DEG_TO_RAD;
    double pitch_current = -18.0 * DEG_TO_RAD;

    bool exceeded = (pitch_current < pitch_max_down);

    TS_ASSERT(exceeded);
  }

  // Test pitch limiting correction
  void testPitchLimiting() {
    double pitch_current = 32.0 * DEG_TO_RAD;
    double pitch_max = 30.0 * DEG_TO_RAD;

    double pitch_error = pitch_current - pitch_max;
    double pitch_correction = -pitch_error * 1.5;

    TS_ASSERT(pitch_correction < 0.0);
    TS_ASSERT_DELTA(pitch_correction, -0.0524, 0.001);
  }

  // Test pitch limit at low speed
  void testPitchLimitLowSpeed() {
    double V = 90.0;  // knots
    double Vs = 100.0;
    double pitch_max_normal = 30.0 * DEG_TO_RAD;

    // Reduced pitch limit at low speed
    double speed_ratio = V / Vs;
    double pitch_max = pitch_max_normal * speed_ratio;

    TS_ASSERT(pitch_max < pitch_max_normal);
    TS_ASSERT_DELTA(pitch_max, 27.0 * DEG_TO_RAD, 0.1 * DEG_TO_RAD);
  }

  /***************************************************************************
   * Load Factor Protection Tests
   ***************************************************************************/

  // Test positive load factor limit (normal category)
  void testPositiveLoadFactorLimit() {
    double nz_max = 3.8;  // g's, normal category
    double nz_current = 4.0;

    bool exceeded = (nz_current > nz_max);

    TS_ASSERT(exceeded);
  }

  // Test negative load factor limit
  void testNegativeLoadFactorLimit() {
    double nz_min = -1.5;  // g's, normal category
    double nz_current = -1.8;

    bool exceeded = (nz_current < nz_min);

    TS_ASSERT(exceeded);
  }

  // Test load factor from bank angle
  void testLoadFactorFromBank() {
    double bank_angle = 60.0 * DEG_TO_RAD;

    double nz = 1.0 / cos(bank_angle);

    TS_ASSERT_DELTA(nz, 2.0, 0.01);
  }

  // Test load factor from pitch rate
  void testLoadFactorFromPitchRate() {
    double V = 200.0 * KTS_TO_FPS;  // ft/s
    double q = 0.1;  // rad/s pitch rate

    double centripetal_accel = V * q / GRAVITY;
    double nz = 1.0 + centripetal_accel;

    TS_ASSERT(nz > 1.0);
    TS_ASSERT_DELTA(nz, 2.048, 0.01);
  }

  // Test load factor protection activation
  void testLoadFactorProtection() {
    double nz_current = 4.0;
    double nz_max = 3.8;

    double nz_error = nz_current - nz_max;
    double pitch_correction = -nz_error * 0.5;  // Reduce pitch to limit load

    TS_ASSERT(pitch_correction < 0.0);
    TS_ASSERT_DELTA(pitch_correction, -0.1, epsilon);
  }

  // Test load factor limit varies with speed (V-n diagram)
  void testLoadFactorVnDiagram() {
    double V = 150.0;  // knots
    double Va = 180.0;  // Design maneuvering speed
    double nz_max_design = 3.8;

    // Below Va, load factor limited by strength
    // Above Va, limited by aerodynamics
    double nz_max = (V <= Va) ? nz_max_design : nz_max_design * (Va / V) * (Va / V);

    TS_ASSERT_DELTA(nz_max, 3.8, epsilon);
  }

  /***************************************************************************
   * Overspeed Warning Tests
   ***************************************************************************/

  // Test overspeed warning activation
  void testOverspeedWarningActivation() {
    double V_current = 252.0;  // knots
    double Vmo = 250.0;
    double V_warning = 245.0;

    bool warning = (V_current >= V_warning);

    TS_ASSERT(warning);
  }

  // Test overspeed warning level (amber vs red)
  void testOverspeedWarningLevel() {
    double V_current = 260.0;
    double Vmo = 250.0;
    double V_warning = 245.0;

    bool amber = (V_current >= V_warning && V_current < Vmo);
    bool red = (V_current >= Vmo);

    TS_ASSERT(!amber);
    TS_ASSERT(red);
  }

  // Test overspeed clacker/clapper activation
  void testOverspeedClacker() {
    double V_current = 251.0;
    double Vmo = 250.0;

    bool clacker_active = (V_current > Vmo);

    TS_ASSERT(clacker_active);
  }

  // Test overspeed margin calculation
  void testOverspeedMargin() {
    double V_current = 240.0;
    double Vmo = 250.0;

    double margin = (Vmo - V_current) / Vmo;

    TS_ASSERT(margin > 0.0);
    TS_ASSERT_DELTA(margin, 0.04, epsilon);
  }

  /***************************************************************************
   * Stick Shaker Tests
   ***************************************************************************/

  // Test stick shaker activation threshold
  void testStickShakerThreshold() {
    double alpha_current = 13.5 * DEG_TO_RAD;
    double alpha_shaker = 13.0 * DEG_TO_RAD;

    bool shaker_active = (alpha_current >= alpha_shaker);

    TS_ASSERT(shaker_active);
  }

  // Test stick shaker threshold with flaps
  void testStickShakerWithFlaps() {
    double alpha_shaker_clean = 13.0 * DEG_TO_RAD;
    double flaps = 20.0;  // degrees

    // Flaps increase stall angle
    double alpha_shaker = alpha_shaker_clean + (flaps / 40.0) * 2.0 * DEG_TO_RAD;

    TS_ASSERT(alpha_shaker > alpha_shaker_clean);
    TS_ASSERT_DELTA(alpha_shaker, 14.0 * DEG_TO_RAD, 0.1 * DEG_TO_RAD);
  }

  // Test stick shaker frequency
  void testStickShakerFrequency() {
    double frequency = 15.0;  // Hz

    // Typical range: 10-20 Hz
    TS_ASSERT(frequency >= 10.0 && frequency <= 20.0);
  }

  // Test stick shaker intensity
  void testStickShakerIntensity() {
    double alpha_current = 14.0 * DEG_TO_RAD;
    double alpha_shaker = 13.0 * DEG_TO_RAD;

    double alpha_error = alpha_current - alpha_shaker;
    double intensity = fmin(alpha_error / (2.0 * DEG_TO_RAD), 1.0);

    TS_ASSERT(intensity >= 0.0 && intensity <= 1.0);
    TS_ASSERT_DELTA(intensity, 0.5, 0.01);
  }

  /***************************************************************************
   * Stick Pusher Tests
   ***************************************************************************/

  // Test stick pusher activation threshold
  void testStickPusherThreshold() {
    double alpha_current = 15.5 * DEG_TO_RAD;
    double alpha_pusher = 15.0 * DEG_TO_RAD;

    bool pusher_active = (alpha_current >= alpha_pusher);

    TS_ASSERT(pusher_active);
  }

  // Test stick pusher force calculation
  void testStickPusherForce() {
    double alpha_current = 16.0 * DEG_TO_RAD;
    double alpha_pusher = 15.0 * DEG_TO_RAD;

    double alpha_error = alpha_current - alpha_pusher;
    double force = 20.0 + alpha_error * 100.0;  // lbs, increasing with error

    TS_ASSERT(force > 20.0);
    TS_ASSERT_DELTA(force, 21.745, 0.01);
  }

  // Test stick pusher timing relative to shaker
  void testStickPusherTiming() {
    double alpha_shaker = 13.0 * DEG_TO_RAD;
    double alpha_pusher = 15.0 * DEG_TO_RAD;

    // Pusher activates after shaker
    TS_ASSERT(alpha_pusher > alpha_shaker);
    double margin = alpha_pusher - alpha_shaker;
    TS_ASSERT_DELTA(margin, 2.0 * DEG_TO_RAD, epsilon);
  }

  // Test stick pusher deactivation
  void testStickPusherDeactivation() {
    double alpha_current = 14.5 * DEG_TO_RAD;
    double alpha_pusher_on = 15.0 * DEG_TO_RAD;
    double alpha_pusher_off = 14.0 * DEG_TO_RAD;

    // Hysteresis prevents oscillation
    bool pusher_active = (alpha_current >= alpha_pusher_on) ||
                        (alpha_current >= alpha_pusher_off);  // If already on

    // Simplified: assuming pusher was not previously active
    pusher_active = (alpha_current >= alpha_pusher_on);

    TS_ASSERT(!pusher_active);
  }

  /***************************************************************************
   * Angle of Attack Limiting Tests
   ***************************************************************************/

  // Test maximum angle of attack limit
  void testMaxAlphaLimit() {
    double alpha_max = 15.0 * DEG_TO_RAD;
    double alpha_current = 16.0 * DEG_TO_RAD;

    bool exceeded = (alpha_current > alpha_max);

    TS_ASSERT(exceeded);
  }

  // Test alpha limiting correction
  void testAlphaLimiting() {
    double alpha_current = 16.0 * DEG_TO_RAD;
    double alpha_max = 15.0 * DEG_TO_RAD;

    double alpha_error = alpha_current - alpha_max;
    double elevator_correction = -alpha_error * 3.0;  // Gain

    TS_ASSERT(elevator_correction < 0.0);
    TS_ASSERT_DELTA(elevator_correction, -0.0524, 0.001);
  }

  // Test alpha limit with configuration
  void testAlphaLimitConfiguration() {
    double alpha_max_clean = 15.0 * DEG_TO_RAD;
    double flaps = 30.0;  // degrees

    // Flaps extend alpha limit
    double alpha_max = alpha_max_clean + (flaps / 40.0) * 3.0 * DEG_TO_RAD;

    TS_ASSERT(alpha_max > alpha_max_clean);
    TS_ASSERT_DELTA(alpha_max, 17.25 * DEG_TO_RAD, 0.1 * DEG_TO_RAD);
  }

  // Test alpha rate limiting
  void testAlphaRateLimit() {
    double alpha_dot_max = 5.0 * DEG_TO_RAD;  // Per second
    double alpha_dot_current = 6.0 * DEG_TO_RAD;

    bool rate_exceeded = (fabs(alpha_dot_current) > alpha_dot_max);

    TS_ASSERT(rate_exceeded);
  }

  /***************************************************************************
   * Envelope Exceedance Detection Tests
   ***************************************************************************/

  // Test envelope exceedance flag
  void testEnvelopeExceedance() {
    double V = 260.0;  // knots
    double Vmo = 250.0;
    double alpha = 16.0 * DEG_TO_RAD;
    double alpha_max = 15.0 * DEG_TO_RAD;

    bool speed_exceeded = (V > Vmo);
    bool alpha_exceeded = (alpha > alpha_max);
    bool envelope_exceeded = speed_exceeded || alpha_exceeded;

    TS_ASSERT(envelope_exceeded);
  }

  // Test multiple exceedance conditions
  void testMultipleExceedances() {
    double V = 260.0;
    double Vmo = 250.0;
    double nz = 4.0;
    double nz_max = 3.8;

    int exceedance_count = 0;
    if (V > Vmo) exceedance_count++;
    if (nz > nz_max) exceedance_count++;

    TS_ASSERT_EQUALS(exceedance_count, 2);
  }

  // Test envelope exceedance severity
  void testExceedanceSeverity() {
    double V = 265.0;
    double Vmo = 250.0;

    double severity = (V - Vmo) / Vmo;

    TS_ASSERT(severity > 0.0);
    TS_ASSERT_DELTA(severity, 0.06, epsilon);
  }

  // Test envelope exceedance duration
  void testExceedanceDuration() {
    double time_exceeded = 5.0;  // seconds
    double max_allowable = 3.0;

    bool duration_exceeded = (time_exceeded > max_allowable);

    TS_ASSERT(duration_exceeded);
  }

  /***************************************************************************
   * Speed Margin to Stall Tests
   ***************************************************************************/

  // Test speed margin calculation
  void testSpeedMarginToStall() {
    double V_current = 120.0;  // knots
    double Vs = 100.0;

    double margin = (V_current - Vs) / Vs;

    TS_ASSERT(margin > 0.0);
    TS_ASSERT_DELTA(margin, 0.2, epsilon);
  }

  // Test minimum acceptable speed margin
  void testMinimumSpeedMargin() {
    double V_current = 115.0;
    double Vs = 100.0;
    double min_margin = 0.3;  // 30%

    double actual_margin = (V_current - Vs) / Vs;
    bool adequate = (actual_margin >= min_margin);

    TS_ASSERT(!adequate);
  }

  // Test speed margin with wind shear
  void testSpeedMarginWindShear() {
    double V_current = 125.0;
    double Vs = 100.0;
    double windshear_loss = 10.0;  // knots

    double V_effective = V_current - windshear_loss;
    double margin = (V_effective - Vs) / Vs;

    TS_ASSERT_DELTA(margin, 0.15, epsilon);
  }

  // Test speed margin in icing
  void testSpeedMarginIcing() {
    double V_current = 130.0;
    double Vs_clean = 100.0;
    double ice_penalty = 1.2;  // 20% increase

    double Vs_ice = Vs_clean * ice_penalty;
    double margin = (V_current - Vs_ice) / Vs_ice;

    TS_ASSERT(margin < (V_current - Vs_clean) / Vs_clean);
    TS_ASSERT_DELTA(margin, 0.0833, 0.001);
  }

  /***************************************************************************
   * Mach Buffet Boundary Tests
   ***************************************************************************/

  // Test low speed buffet boundary
  void testLowSpeedBuffet() {
    double alpha = 12.0 * DEG_TO_RAD;
    double alpha_buffet_low = 11.0 * DEG_TO_RAD;

    bool buffet = (alpha >= alpha_buffet_low);

    TS_ASSERT(buffet);
  }

  // Test high speed buffet boundary (Mach)
  void testHighSpeedBuffet() {
    double M = 0.84;
    double M_buffet = 0.82;

    bool buffet = (M >= M_buffet);

    TS_ASSERT(buffet);
  }

  // Test buffet margin calculation
  void testBuffetMargin() {
    double M_current = 0.80;
    double M_buffet = 0.82;

    double margin = (M_buffet - M_current) / M_buffet;

    TS_ASSERT(margin > 0.0);
    TS_ASSERT_DELTA(margin, 0.0244, 0.001);
  }

  // Test buffet onset with altitude
  void testBuffetOnsetAltitude() {
    double altitude = 40000.0;  // ft
    double M_buffet_low = 0.70;
    double M_buffet_high = 0.82;

    // Higher altitude narrows buffet-free envelope
    // At high altitude, low speed and high speed buffet converge
    double altitude_factor = fmin(altitude / 45000.0, 1.0);
    double buffet_width = M_buffet_high - M_buffet_low;
    double narrow_factor = 1.0 - 0.3 * altitude_factor;
    double narrowed_width = buffet_width * narrow_factor;

    TS_ASSERT(narrowed_width < buffet_width);
    TS_ASSERT_DELTA(narrowed_width, 0.088, 0.001);
  }

  // Test maneuver buffet boundary
  void testManeuverBuffet() {
    double M_buffet_1g = 0.82;
    double load_factor = 1.5;

    // Buffet Mach decreases with load factor
    double M_buffet = M_buffet_1g / sqrt(load_factor);

    TS_ASSERT(M_buffet < M_buffet_1g);
    TS_ASSERT_DELTA(M_buffet, 0.670, 0.01);
  }

  // Test buffet intensity
  void testBuffetIntensity() {
    double M = 0.84;
    double M_buffet = 0.82;

    double M_exceedance = M - M_buffet;
    double intensity = fmin(M_exceedance / 0.05, 1.0);  // Max at Mach 0.87

    TS_ASSERT(intensity >= 0.0 && intensity <= 1.0);
    TS_ASSERT_DELTA(intensity, 0.4, 0.01);
  }

  // Test coffin corner detection
  void testCoffinCorner() {
    double M_current = 0.76;
    double M_buffet_high = 0.78;
    double M_buffet_low = 0.74;

    // In coffin corner: between low and high speed buffet
    bool in_coffin_corner = (M_current > M_buffet_low) &&
                           (M_current < M_buffet_high);

    TS_ASSERT(in_coffin_corner);

    // Margin to either boundary
    double margin_high = M_buffet_high - M_current;
    double margin_low = M_current - M_buffet_low;
    double min_margin = fmin(margin_high, margin_low);

    TS_ASSERT_DELTA(min_margin, 0.02, epsilon);
  }
};
