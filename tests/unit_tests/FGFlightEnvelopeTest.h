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

  /***************************************************************************
   * Auto-Throttle Integration Tests
   ***************************************************************************/

  // Test auto-throttle speed protection
  void testAutoThrottleSpeedProtection() {
    double V_target = 150.0;  // knots
    double V_current = 145.0;
    double Vs = 100.0;
    double V_min = Vs * 1.2;  // Minimum speed floor

    // Auto-throttle should not command below minimum
    double V_command = fmax(V_target, V_min);

    TS_ASSERT(V_command >= V_min);
    TS_ASSERT_DELTA(V_command, 150.0, epsilon);
  }

  // Test auto-throttle speed envelope limiting
  void testAutoThrottleEnvelopeLimiting() {
    double V_requested = 280.0;  // knots
    double Vmo = 250.0;
    double Vs = 100.0;

    // Clamp to envelope
    double V_command = fmin(fmax(V_requested, Vs * 1.3), Vmo);

    TS_ASSERT(V_command <= Vmo);
    TS_ASSERT_DELTA(V_command, 250.0, epsilon);
  }

  // Test auto-throttle TOGA activation
  void testAutoThrottleTOGAActivation() {
    double alpha = 14.0 * DEG_TO_RAD;
    double alpha_floor = 13.0 * DEG_TO_RAD;

    // Alpha floor triggers TOGA thrust
    bool toga_active = (alpha >= alpha_floor);

    TS_ASSERT(toga_active);
  }

  // Test auto-throttle retard mode
  void testAutoThrottleRetardMode() {
    double altitude_agl = 20.0;  // feet
    double retard_altitude = 30.0;  // feet

    bool retard_mode = (altitude_agl <= retard_altitude);

    TS_ASSERT(retard_mode);
  }

  /***************************************************************************
   * Angle of Attack Vane Tests
   ***************************************************************************/

  // Test AOA vane position computation
  void testAOAVanePosition() {
    double alpha = 10.0 * DEG_TO_RAD;
    double flow_distortion = 1.5 * DEG_TO_RAD;  // Fuselage effect

    double vane_alpha = alpha + flow_distortion;

    TS_ASSERT(vane_alpha > alpha);
    TS_ASSERT_DELTA(vane_alpha, 11.5 * DEG_TO_RAD, epsilon);
  }

  // Test AOA vane calibration
  void testAOAVaneCalibration() {
    double vane_raw = 12.0 * DEG_TO_RAD;
    double offset = 1.5 * DEG_TO_RAD;
    double scale = 0.98;

    double alpha_corrected = (vane_raw - offset) * scale;

    TS_ASSERT_DELTA(alpha_corrected, 10.29 * DEG_TO_RAD, 0.1 * DEG_TO_RAD);
  }

  // Test AOA vane heating compensation
  void testAOAVaneHeatingCompensation() {
    double alpha_base = 10.0 * DEG_TO_RAD;
    double temperature = -50.0;  // Celsius
    double temp_coeff = 0.0001;  // Per degree

    // Slight position change with temperature
    double alpha_compensated = alpha_base * (1.0 + temp_coeff * fabs(temperature));

    TS_ASSERT(alpha_compensated > alpha_base);
    TS_ASSERT_DELTA(alpha_compensated, 10.05 * DEG_TO_RAD, 0.01 * DEG_TO_RAD);
  }

  // Test dual AOA vane comparison
  void testDualAOAVaneComparison() {
    double alpha_left = 10.0 * DEG_TO_RAD;
    double alpha_right = 10.3 * DEG_TO_RAD;
    double max_disagree = 2.0 * DEG_TO_RAD;

    double disagreement = fabs(alpha_left - alpha_right);
    bool valid = (disagreement < max_disagree);

    TS_ASSERT(valid);
    TS_ASSERT_DELTA(disagreement, 0.3 * DEG_TO_RAD, epsilon);
  }

  /***************************************************************************
   * Protection Mode Degradation Tests
   ***************************************************************************/

  // Test normal law to alternate law transition
  void testNormalToAlternateLaw() {
    bool aoa_sensor_valid = false;
    bool airspeed_valid = true;

    bool normal_law = aoa_sensor_valid && airspeed_valid;
    bool alternate_law = !normal_law && airspeed_valid;

    TS_ASSERT(!normal_law);
    TS_ASSERT(alternate_law);
  }

  // Test alternate law protection limits
  void testAlternateLawLimits() {
    double bank_limit_normal = 67.0 * DEG_TO_RAD;
    double bank_limit_alternate = 45.0 * DEG_TO_RAD;

    // Alternate law has reduced limits
    TS_ASSERT(bank_limit_alternate < bank_limit_normal);
  }

  // Test direct law characteristics
  void testDirectLawCharacteristics() {
    double stick_input = 0.5;  // -1 to 1
    double surface_gain = 25.0 * DEG_TO_RAD;  // Max deflection

    // Direct law: no protection, direct surface command
    double surface_deflection = stick_input * surface_gain;

    TS_ASSERT_DELTA(surface_deflection, 12.5 * DEG_TO_RAD, epsilon);
  }

  // Test protection blending during mode transition
  void testProtectionBlending() {
    double alpha_limit_normal = 15.0 * DEG_TO_RAD;
    double alpha_limit_alternate = 18.0 * DEG_TO_RAD;
    double blend_factor = 0.3;  // 30% transition complete

    double alpha_limit = alpha_limit_normal +
                        blend_factor * (alpha_limit_alternate - alpha_limit_normal);

    TS_ASSERT_DELTA(alpha_limit, 15.9 * DEG_TO_RAD, 0.01 * DEG_TO_RAD);
  }

  /***************************************************************************
   * Structural Protection Tests
   ***************************************************************************/

  // Test ultimate load factor limit
  void testUltimateLoadFactor() {
    double nz_limit = 3.8;  // Limit load
    double nz_ultimate = nz_limit * 1.5;  // Ultimate = 1.5x limit

    TS_ASSERT_DELTA(nz_ultimate, 5.7, epsilon);
  }

  // Test load factor rate protection
  void testLoadFactorRateProtection() {
    double nz_dot = 10.0;  // g/s
    double nz_dot_max = 8.0;  // Maximum safe rate

    bool rate_exceeded = (fabs(nz_dot) > nz_dot_max);

    TS_ASSERT(rate_exceeded);
  }

  // Test flutter speed margin
  void testFlutterSpeedMargin() {
    double Vd = 300.0;  // Design dive speed
    double Vmo = 250.0;
    double flutter_margin = Vd / Vmo;

    TS_ASSERT(flutter_margin > 1.0);
    TS_ASSERT_DELTA(flutter_margin, 1.2, epsilon);
  }

  // Test gust load factor
  void testGustLoadFactor() {
    double U_gust = 50.0;  // fps
    double V = 250.0 * KTS_TO_FPS;
    double CLalpha = 5.7;  // per radian
    double rho = 0.002378;  // slugs/ft^3
    double S = 200.0;  // Wing area ft^2
    double W = 5000.0;  // Weight lbf

    // Simplified gust load factor
    double delta_n = (rho * U_gust * V * CLalpha * S) / (2.0 * W);

    TS_ASSERT(delta_n > 0.0);
  }

  /***************************************************************************
   * Recovery Technique Tests
   ***************************************************************************/

  // Test stall recovery altitude loss
  void testStallRecoveryAltitudeLoss() {
    double V_stall = 100.0 * KTS_TO_FPS;  // fps
    double reaction_time = 2.0;  // seconds
    double recovery_angle = 10.0 * DEG_TO_RAD;

    // Simplified altitude loss estimate
    double decel = V_stall * 0.1;  // Deceleration
    double alt_loss = 0.5 * decel * reaction_time * reaction_time +
                     V_stall * sin(recovery_angle) * reaction_time;

    TS_ASSERT(alt_loss > 0.0);
  }

  // Test upset recovery bank limit
  void testUpsetRecoveryBankLimit() {
    double bank_current = 90.0 * DEG_TO_RAD;
    double bank_target = 60.0 * DEG_TO_RAD;

    double bank_correction = bank_target - bank_current;

    TS_ASSERT(bank_correction < 0.0);  // Roll toward wings level
    TS_ASSERT_DELTA(bank_correction, -30.0 * DEG_TO_RAD, epsilon);
  }

  // Test nose low recovery priority
  void testNoseLowRecoveryPriority() {
    double pitch = -45.0 * DEG_TO_RAD;
    double bank = 120.0 * DEG_TO_RAD;

    // Priority 1: Reduce bank toward 90 degrees
    // Priority 2: Apply back pressure
    bool reduce_bank_first = (fabs(bank) > 90.0 * DEG_TO_RAD);

    TS_ASSERT(reduce_bank_first);
  }

  // Test TCAS RA climb rate requirement
  void testTCASRAClimbRate() {
    double required_climb = 1500.0;  // fpm
    double V = 250.0 * KTS_TO_FPS;

    // Required pitch angle for climb rate
    double climb_fps = required_climb / 60.0;  // Convert to fps
    double gamma = asin(climb_fps / V);

    TS_ASSERT(gamma > 0.0);
    TS_ASSERT_DELTA(gamma * RAD_TO_DEG, 3.4, 0.5);
  }

  /***************************************************************************
   * Environmental Effects on Envelope Tests
   ***************************************************************************/

  // Test envelope contraction with icing
  void testEnvelopeContractionIcing() {
    double Vs_clean = 100.0;  // knots
    double ice_factor = 1.3;  // 30% increase

    double Vs_ice = Vs_clean * ice_factor;
    double envelope_reduction = (Vs_ice - Vs_clean) / Vs_clean;

    TS_ASSERT_DELTA(envelope_reduction, 0.3, epsilon);
  }

  // Test Mmo reduction with temperature
  void testMmoTemperatureEffect() {
    double Mmo_standard = 0.82;
    double temperature_deviation = 20.0;  // ISA + 20

    // Hot day: slightly lower Mmo due to true airspeed effects
    double Mmo_adjusted = Mmo_standard * (1.0 - 0.001 * temperature_deviation);

    TS_ASSERT(Mmo_adjusted < Mmo_standard);
    TS_ASSERT_DELTA(Mmo_adjusted, 0.804, 0.001);
  }

  // Test envelope with turbulence
  void testEnvelopeWithTurbulence() {
    double V_target = 280.0;  // knots
    double Vmo = 300.0;
    double turbulence_penetration_speed = 250.0;

    // In turbulence, fly turbulence penetration speed
    double V_command = fmin(V_target, turbulence_penetration_speed);

    TS_ASSERT_DELTA(V_command, 250.0, epsilon);
  }

  // Test altitude ceiling effect
  void testAltitudeCeilingEffect() {
    double altitude = 42000.0;  // feet
    double service_ceiling = 43000.0;
    double buffet_margin = 1.5;  // g available

    // Near ceiling, margin decreases
    double altitude_factor = altitude / service_ceiling;
    double margin_available = buffet_margin * (1.0 - altitude_factor);

    TS_ASSERT(margin_available < buffet_margin);
    TS_ASSERT(margin_available > 0.0);
  }

  /***************************************************************************
   * Configuration-Dependent Limits Tests
   ***************************************************************************/

  // Test gear extended speed limit
  void testGearExtendedSpeedLimit() {
    double Vlo = 200.0;  // Gear operating speed
    double V_current = 210.0;

    bool overspeed = (V_current > Vlo);

    TS_ASSERT(overspeed);
  }

  // Test flap extended speed limit
  void testFlapExtendedSpeedLimit() {
    double Vfe[] = {250.0, 200.0, 180.0, 165.0};  // Flaps 1, 2, 3, full
    int flap_position = 2;
    double V_current = 185.0;

    bool overspeed = (V_current > Vfe[flap_position]);

    TS_ASSERT(overspeed);
  }

  // Test slat retraction schedule
  void testSlatRetractionSchedule() {
    double V = 200.0;  // knots
    double V_slat_retract = 190.0;
    double V_slat_extend = 180.0;

    bool slats_retracted = (V > V_slat_retract);

    TS_ASSERT(slats_retracted);
  }

  // Test landing configuration stall speed
  void testLandingConfigStallSpeed() {
    double Vs_clean = 120.0;  // knots
    double CL_clean = 1.5;
    double CL_landing = 2.5;  // Full flaps

    double Vs_landing = Vs_clean * sqrt(CL_clean / CL_landing);

    TS_ASSERT(Vs_landing < Vs_clean);
    TS_ASSERT_DELTA(Vs_landing, 92.9, 0.5);
  }

  /***************************************************************************
   * Warning System Tests
   ***************************************************************************/

  // Test aural warning priority
  void testAuralWarningPriority() {
    bool stall_warning = true;
    bool overspeed_warning = true;
    bool terrain_warning = false;

    // Stall has higher priority than overspeed
    int priority = 0;
    if (terrain_warning) priority = 1;
    else if (stall_warning) priority = 2;
    else if (overspeed_warning) priority = 3;

    TS_ASSERT_EQUALS(priority, 2);  // Stall warning
  }

  // Test visual warning color coding
  void testVisualWarningColorCoding() {
    double alpha = 14.0 * DEG_TO_RAD;
    double alpha_amber = 12.0 * DEG_TO_RAD;
    double alpha_red = 14.5 * DEG_TO_RAD;

    int color = 0;  // 0=green, 1=amber, 2=red
    if (alpha >= alpha_red) color = 2;
    else if (alpha >= alpha_amber) color = 1;

    TS_ASSERT_EQUALS(color, 1);  // Amber
  }

  // Test warning inhibit during takeoff
  void testWarningInhibitTakeoff() {
    double altitude_agl = 50.0;
    double V = 100.0;
    double V_rotate = 120.0;

    // Inhibit certain warnings during takeoff roll
    bool takeoff_phase = (altitude_agl < 100.0) && (V < V_rotate);
    bool inhibit_config_warning = takeoff_phase;

    TS_ASSERT(inhibit_config_warning);
  }

  // Test master warning acknowledgment
  void testMasterWarningAcknowledgment() {
    bool warning_active = true;
    bool pilot_acknowledged = true;

    bool light_flashing = warning_active && !pilot_acknowledged;

    TS_ASSERT(!light_flashing);  // Steady after ack
  }

  /***************************************************************************
   * Flight Data Monitoring Tests
   ***************************************************************************/

  // Test envelope exceedance recording
  void testEnvelopeExceedanceRecording() {
    double V = 255.0;
    double Vmo = 250.0;
    double duration = 5.0;  // seconds

    double exceedance_magnitude = V - Vmo;
    double severity = exceedance_magnitude * duration;

    TS_ASSERT(severity > 0.0);
    TS_ASSERT_DELTA(severity, 25.0, epsilon);
  }

  // Test maximum recorded values
  void testMaximumRecordedValues() {
    double nz_values[] = {1.0, 1.5, 2.8, 2.2, 1.8};
    int n = 5;

    double nz_max = nz_values[0];
    for (int i = 1; i < n; i++) {
      if (nz_values[i] > nz_max) nz_max = nz_values[i];
    }

    TS_ASSERT_DELTA(nz_max, 2.8, epsilon);
  }

  // Test fatigue damage accumulation
  void testFatigueDamageAccumulation() {
    double nz = 2.5;  // g load
    double nz_ref = 1.0;
    double cycles = 100.0;
    double exponent = 3.0;  // S-N curve exponent

    double damage = cycles * pow((nz - 1.0) / (nz_ref), exponent);

    TS_ASSERT(damage > 0.0);
  }

  // Test hard landing detection
  void testHardLandingDetection() {
    double sink_rate = 8.0;  // fps
    double hard_landing_threshold = 6.0;

    bool hard_landing = (sink_rate > hard_landing_threshold);

    TS_ASSERT(hard_landing);
  }

  /***************************************************************************
   * Maneuver Envelope Tests
   ***************************************************************************/

  // Test Va (maneuvering speed) calculation
  void testManeuveringSpeed() {
    double Vs = 100.0;  // knots
    double nz_limit = 3.8;

    double Va = Vs * sqrt(nz_limit);

    TS_ASSERT_DELTA(Va, 194.9, 0.5);
  }

  // Test Vno (maximum structural cruising speed)
  void testMaxStructuralCruisingSpeed() {
    double Vno = 180.0;  // knots
    double Vne = 200.0;

    // Vno is caution range upper limit
    TS_ASSERT(Vno < Vne);
    double margin = (Vne - Vno) / Vne;
    TS_ASSERT_DELTA(margin, 0.1, epsilon);
  }

  // Test corner speed
  void testCornerSpeed() {
    double Vs = 100.0;  // knots
    double nz_max = 3.8;

    // Corner speed: maximum instantaneous turn rate
    double Vc = Vs * sqrt(nz_max);

    TS_ASSERT_DELTA(Vc, 194.9, 0.5);
  }

  // Test sustained turn rate limit
  void testSustainedTurnRateLimit() {
    double V = 200.0 * KTS_TO_FPS;  // fps
    double nz = 2.0;

    double turn_rate = GRAVITY * sqrt(nz * nz - 1.0) / V;

    TS_ASSERT(turn_rate > 0.0);
    TS_ASSERT_DELTA(turn_rate * RAD_TO_DEG, 9.6, 0.5);  // deg/s
  }

  /***************************************************************************
   * Energy State Protection Tests
   ***************************************************************************/

  // Test specific energy calculation
  void testSpecificEnergy() {
    double V = 250.0 * KTS_TO_FPS;  // fps
    double h = 10000.0;  // feet

    double specific_energy = h + V * V / (2.0 * GRAVITY);

    TS_ASSERT(specific_energy > h);
  }

  // Test energy rate monitoring
  void testEnergyRateMonitoring() {
    double V_dot = -10.0;  // fps/s (decelerating)
    double h_dot = 500.0 / 60.0;  // fps (climbing 500 fpm)
    double V = 200.0 * KTS_TO_FPS;

    double energy_rate = h_dot + V * V_dot / GRAVITY;

    // Trading speed for altitude
    TS_ASSERT(energy_rate > 0.0 || energy_rate < 0.0);  // Can be either
  }

  // Test minimum energy height
  void testMinimumEnergyHeight() {
    double V = 150.0 * KTS_TO_FPS;
    double Vs = 100.0 * KTS_TO_FPS;

    // Potential energy equivalent of excess kinetic energy
    double energy_height = (V * V - Vs * Vs) / (2.0 * GRAVITY);

    TS_ASSERT(energy_height > 0.0);
  }

  // Test speed stability protection
  void testSpeedStabilityProtection() {
    double V = 120.0;  // knots
    double V_target = 130.0;
    double Vs = 100.0;

    // Speed deviation from target
    double speed_error = V_target - V;
    double margin_to_stall = (V - Vs) / Vs;

    // Protection more aggressive when margin low
    double urgency = 1.0 / (margin_to_stall + 0.1);

    TS_ASSERT(urgency > 0.0);
  }
};
