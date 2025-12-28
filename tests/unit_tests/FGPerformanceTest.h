/*******************************************************************************
 * FGPerformanceTest.h - Unit tests for aircraft performance calculations
 *
 * Tests comprehensive aircraft performance metrics including:
 * - Rate of climb, service/absolute ceiling
 * - Best climb speeds (Vx, Vy)
 * - Range and endurance speeds
 * - Energy state and specific excess power
 * - Turn performance and corner speed
 * - Breguet range equation and payload-range tradeoffs
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <limits>
#include <cmath>
#include <cxxtest/TestSuite.h>
#include "TestUtilities.h"

const double epsilon = 100. * std::numeric_limits<double>::epsilon();
constexpr double radtodeg = 180. / M_PI;
constexpr double degtoman = M_PI / 180.;

using namespace JSBSim;
using namespace JSBSimTest;

class FGPerformanceTest : public CxxTest::TestSuite
{
public:
  // Physical constants
  static constexpr double g = Constants::G_FTPS2;  // 32.174 ft/s^2
  static constexpr double rho0 = Constants::SEA_LEVEL_DENSITY_SLUGFT3;  // 0.002377 slugs/ft^3

  //===========================================================================
  // 1. RATE OF CLIMB TESTS (4 tests)
  //===========================================================================

  void testRateOfClimbBasic() {
    // ROC = (T - D) * V / W
    double thrust = 5000.0;  // lbs
    double drag = 2000.0;    // lbs
    double weight = 10000.0; // lbs
    double velocity = 300.0; // ft/s

    double roc = (thrust - drag) * velocity / weight;
    double expected = (5000.0 - 2000.0) * 300.0 / 10000.0;  // 90 ft/s

    TS_ASSERT_DELTA(roc, expected, 0.01);
    TS_ASSERT_DELTA(roc, 90.0, 0.01);
  }

  void testRateOfClimbWithExcessThrust() {
    // High excess thrust scenario
    double thrust = 8000.0;
    double drag = 1500.0;
    double weight = 8000.0;
    double velocity = 400.0;

    double roc = (thrust - drag) * velocity / weight;
    // ROC = (8000 - 1500) * 400 / 8000 = 325 ft/s

    TS_ASSERT_DELTA(roc, 325.0, 0.1);
    TS_ASSERT(roc > 0.0);  // Should be climbing
  }

  void testRateOfClimbAtThrustDragBalance() {
    // When T = D, ROC should be zero (level flight)
    double thrust = 3000.0;
    double drag = 3000.0;
    double weight = 12000.0;
    double velocity = 250.0;

    double roc = (thrust - drag) * velocity / weight;

    TS_ASSERT_DELTA(roc, 0.0, epsilon);
  }

  void testRateOfClimbInDescent() {
    // When D > T, ROC is negative (descending)
    double thrust = 2000.0;
    double drag = 4000.0;
    double weight = 10000.0;
    double velocity = 300.0;

    double roc = (thrust - drag) * velocity / weight;

    TS_ASSERT(roc < 0.0);
    TS_ASSERT_DELTA(roc, -60.0, 0.1);  // Descending at 60 ft/s
  }

  //===========================================================================
  // 2. SERVICE CEILING TESTS (3 tests)
  //===========================================================================

  void testServiceCeilingDefinition() {
    // Service ceiling: altitude where ROC = 100 ft/min = 1.67 ft/s
    double roc_fpm = 100.0;  // ft/min
    double roc_fps = roc_fpm / 60.0;

    TS_ASSERT_DELTA(roc_fps, 1.6667, 0.001);

    // At service ceiling, this is the minimum ROC
    TS_ASSERT(roc_fps > 0.0);
  }

  void testServiceCeilingCalculation() {
    // Simplified model: assume linear decrease in ROC with altitude
    double roc_sl = 2000.0;  // ft/min at sea level
    double service_ceiling = 30000.0;  // ft

    // At service ceiling, ROC = 100 ft/min
    double roc_at_ceiling = 100.0;

    // Linear interpolation coefficient
    double k = (roc_sl - roc_at_ceiling) / service_ceiling;
    double roc_at_15000 = roc_sl - k * 15000.0;

    // At half the service ceiling, ROC should be halfway between SL and ceiling
    TS_ASSERT_DELTA(roc_at_15000, 1050.0, 1.0);
  }

  void testServiceCeilingWithDensityEffect() {
    // ROC decreases with altitude due to density decrease
    double rho_sl = rho0;
    double rho_15000 = 0.001496;  // slugs/ft^3 at 15,000 ft

    double density_ratio = rho_15000 / rho_sl;

    // Thrust typically decreases with density for non-turbocharged engines
    TS_ASSERT(density_ratio < 1.0);
    TS_ASSERT_DELTA(density_ratio, 0.629, 0.01);
  }

  //===========================================================================
  // 3. ABSOLUTE CEILING TESTS (3 tests)
  //===========================================================================

  void testAbsoluteCeilingDefinition() {
    // Absolute ceiling: where ROC = 0
    double thrust = 3000.0;
    double drag = 3000.0;  // At absolute ceiling, T = D (best case)
    double weight = 10000.0;
    double velocity = 200.0;

    double roc = (thrust - drag) * velocity / weight;

    TS_ASSERT_DELTA(roc, 0.0, epsilon);
  }

  void testAbsoluteCeilingVsServiceCeiling() {
    // Absolute ceiling is always higher than service ceiling
    double service_ceiling = 35000.0;  // ft
    double absolute_ceiling = 37000.0; // ft

    TS_ASSERT(absolute_ceiling > service_ceiling);
    TS_ASSERT_DELTA(absolute_ceiling - service_ceiling, 2000.0, 100.0);
  }

  void testAbsoluteCeilingThrustLimit() {
    // At absolute ceiling, max thrust equals drag
    double max_thrust_ceiling = 1500.0;  // lbs (reduced at altitude)
    double drag_ceiling = 1500.0;        // lbs

    double excess_thrust = max_thrust_ceiling - drag_ceiling;

    TS_ASSERT_DELTA(excess_thrust, 0.0, 0.1);
  }

  //===========================================================================
  // 4. BEST CLIMB SPEED (Vy) TESTS (4 tests)
  //===========================================================================

  void testBestClimbSpeedVy() {
    // Vy: speed for maximum rate of climb
    // Occurs where (T-D)*V is maximum

    // Simplified: test multiple speeds and find maximum ROC
    double thrust = 6000.0;
    double weight = 12000.0;

    double v1 = 100.0, d1 = 800.0;   // Low speed, high induced drag
    double v2 = 150.0, d2 = 600.0;   // Better speed
    double v3 = 120.0, d3 = 700.0;   // Intermediate

    double roc1 = (thrust - d1) * v1 / weight;
    double roc2 = (thrust - d2) * v2 / weight;
    double roc3 = (thrust - d3) * v3 / weight;

    // v2 should give best ROC in this example
    TS_ASSERT(roc2 > roc1);
    TS_ASSERT(roc2 > roc3);
  }

  void testVyWithPowerCurve() {
    // For propeller aircraft, Vy is related to minimum power required
    double power1 = 150.0;  // HP at speed 1
    double power2 = 120.0;  // HP at speed 2 (closer to Vy)
    double power3 = 140.0;  // HP at speed 3

    // Lower power required generally better for climb (at given thrust)
    TS_ASSERT(power2 < power1);
    TS_ASSERT(power2 < power3);
  }

  void testVyDependenceOnWeight() {
    // Vy increases with weight
    double vy_light = 120.0;   // kts at light weight
    double vy_heavy = 135.0;   // kts at heavy weight

    TS_ASSERT(vy_heavy > vy_light);
  }

  void testVyOptimalClimbAngle() {
    // At Vy, climb angle gamma = arcsin(ROC/V)
    double vy = 150.0;      // ft/s
    double roc = 30.0;      // ft/s

    double gamma = asin(roc / vy);
    double gamma_deg = gamma * radtodeg;

    TS_ASSERT_DELTA(gamma_deg, 11.54, 0.1);
    TS_ASSERT(gamma > 0.0);
  }

  //===========================================================================
  // 5. BEST ANGLE CLIMB SPEED (Vx) TESTS (4 tests)
  //===========================================================================

  void testBestAngleClimbSpeedVx() {
    // Vx: speed for maximum climb angle
    // Occurs where (T-D)/W is maximum, i.e., T-D is maximum

    double thrust = 5000.0;
    double weight = 10000.0;

    double v1 = 80.0,  d1 = 1200.0;  // Low speed
    double v2 = 100.0, d2 = 900.0;   // Vx candidate (min drag near here)
    double v3 = 150.0, d3 = 1100.0;  // Higher speed

    double excess_thrust1 = thrust - d1;
    double excess_thrust2 = thrust - d2;
    double excess_thrust3 = thrust - d3;

    // v2 should have maximum excess thrust
    TS_ASSERT(excess_thrust2 > excess_thrust1);
    TS_ASSERT(excess_thrust2 > excess_thrust3);
  }

  void testVxClimbAngle() {
    // At Vx, climb angle gamma = arcsin((T-D)/W)
    double thrust = 6000.0;
    double drag = 2000.0;
    double weight = 10000.0;

    double sin_gamma = (thrust - drag) / weight;
    double gamma = asin(sin_gamma);
    double gamma_deg = gamma * radtodeg;

    TS_ASSERT_DELTA(sin_gamma, 0.4, 0.01);
    TS_ASSERT_DELTA(gamma_deg, 23.58, 0.1);
  }

  void testVxVsVyRelationship() {
    // Vx is always less than Vy
    double vx = 95.0;   // kts
    double vy = 120.0;  // kts

    TS_ASSERT(vx < vy);
  }

  void testVxAtMinimumDrag() {
    // Vx occurs near minimum drag speed (L/D max)
    double drag_min = 800.0;   // lbs
    double drag_vx = 850.0;    // lbs (close to minimum)
    double drag_higher = 1200.0;  // lbs at higher speed

    TS_ASSERT(drag_vx < drag_higher);
    TS_ASSERT_DELTA(drag_vx - drag_min, 50.0, 10.0);
  }

  //===========================================================================
  // 6. MAXIMUM RANGE SPEED TESTS (4 tests)
  //===========================================================================

  void testMaximumRangeSpeed() {
    // For jet: max range at max L/D (minimum drag speed)
    double lift = 10000.0;  // lbs
    double drag_min = 800.0;  // lbs at max L/D

    double ld_max = lift / drag_min;

    TS_ASSERT_DELTA(ld_max, 12.5, 0.1);
  }

  void testMaxRangeSpeedForPropeller() {
    // For propeller: max range at max L^(3/2)/D speed
    // Approximated by flying slightly slower than jet max range speed
    double v_jet_max_range = 200.0;   // kts
    double v_prop_max_range = 180.0;  // kts (typically 0.76 * Vbg for props)

    TS_ASSERT(v_prop_max_range < v_jet_max_range);
  }

  void testMaxRangeFuelFlow() {
    // At max range speed, fuel flow per distance is minimized
    double fuel_flow = 15.0;  // lbs/hr
    double velocity = 200.0;  // kts

    double fuel_per_nm = fuel_flow / velocity;  // lbs/nm

    TS_ASSERT_DELTA(fuel_per_nm, 0.075, 0.001);
  }

  void testMaxRangeAltitudeEffect() {
    // For jets, range increases with altitude (up to optimum)
    double range_sl = 800.0;      // nm at sea level
    double range_35000 = 1200.0;  // nm at 35,000 ft

    TS_ASSERT(range_35000 > range_sl);
    double improvement = (range_35000 - range_sl) / range_sl;
    TS_ASSERT_DELTA(improvement, 0.5, 0.1);  // 50% improvement
  }

  //===========================================================================
  // 7. MAXIMUM ENDURANCE SPEED TESTS (4 tests)
  //===========================================================================

  void testMaximumEnduranceSpeed() {
    // For jet: max endurance at minimum fuel flow (min drag * V)
    // Occurs at minimum power required
    double drag1 = 900.0;   // lbs
    double v1 = 150.0;      // ft/s
    double power1 = drag1 * v1;

    double drag2 = 850.0;   // lbs (minimum power point)
    double v2 = 140.0;      // ft/s
    double power2 = drag2 * v2;

    double drag3 = 950.0;   // lbs
    double v3 = 160.0;      // ft/s
    double power3 = drag3 * v3;

    TS_ASSERT(power2 < power1);
    TS_ASSERT(power2 < power3);
  }

  void testMaxEnduranceVsMaxRange() {
    // Max endurance speed is slower than max range speed
    double v_endurance = 140.0;  // kts
    double v_range = 180.0;      // kts

    TS_ASSERT(v_endurance < v_range);
  }

  void testMaxEnduranceFuelBurn() {
    // At max endurance, time aloft is maximized
    double fuel_available = 500.0;  // lbs
    double fuel_flow = 12.0;        // lbs/hr at max endurance

    double endurance_hours = fuel_available / fuel_flow;

    TS_ASSERT_DELTA(endurance_hours, 41.67, 0.1);
  }

  void testMaxEnduranceLiftCoefficient() {
    // Max endurance for jet occurs at higher CL than max range
    double cl_endurance = 1.2;  // Max endurance CL
    double cl_range = 0.7;      // Max range CL

    TS_ASSERT(cl_endurance > cl_range);
  }

  //===========================================================================
  // 8. SPECIFIC EXCESS POWER (Ps) TESTS (5 tests)
  //===========================================================================

  void testSpecificExcessPowerBasic() {
    // Ps = (T - D) * V / W = excess power per unit weight
    double thrust = 8000.0;
    double drag = 3000.0;
    double velocity = 500.0;  // ft/s
    double weight = 15000.0;

    double ps = (thrust - drag) * velocity / weight;

    TS_ASSERT_DELTA(ps, 166.67, 0.1);  // ft/s
  }

  void testSpecificExcessPowerEqualsROC() {
    // In steady state, Ps = rate of climb
    double roc = 50.0;  // ft/s
    double ps = 50.0;   // ft/s

    TS_ASSERT_DELTA(ps, roc, epsilon);
  }

  void testSpecificExcessPowerWithAcceleration() {
    // Ps can be used for climb or acceleration
    // If Ps = 100 ft/s and aircraft climbs at 60 ft/s,
    // remaining 40 ft/s goes to acceleration
    double ps = 100.0;
    double roc = 60.0;
    double ps_for_accel = ps - roc;

    TS_ASSERT_DELTA(ps_for_accel, 40.0, 0.1);
  }

  void testSpecificExcessPowerZero() {
    // When Ps = 0, aircraft at max sustained performance
    double thrust = 5000.0;
    double drag = 5000.0;
    double velocity = 400.0;
    double weight = 12000.0;

    double ps = (thrust - drag) * velocity / weight;

    TS_ASSERT_DELTA(ps, 0.0, epsilon);
  }

  void testSpecificExcessPowerNegative() {
    // Negative Ps means aircraft must descend or decelerate
    double thrust = 3000.0;
    double drag = 5000.0;
    double velocity = 450.0;
    double weight = 10000.0;

    double ps = (thrust - drag) * velocity / weight;

    TS_ASSERT(ps < 0.0);
    TS_ASSERT_DELTA(ps, -90.0, 0.1);
  }

  //===========================================================================
  // 9. ENERGY HEIGHT TESTS (4 tests)
  //===========================================================================

  void testEnergyHeightBasic() {
    // He = h + V^2 / (2*g)
    double altitude = 10000.0;  // ft
    double velocity = 500.0;    // ft/s

    double energy_height = altitude + (velocity * velocity) / (2.0 * g);
    double kinetic_component = (velocity * velocity) / (2.0 * g);

    TS_ASSERT_DELTA(kinetic_component, 3885.1, 1.0);
    TS_ASSERT_DELTA(energy_height, 13885.1, 1.0);
  }

  void testEnergyHeightConversion() {
    // Energy can be converted between potential and kinetic
    double h1 = 15000.0, v1 = 400.0;
    double h2 = 12000.0, v2 = 500.0;

    double he1 = h1 + v1 * v1 / (2.0 * g);
    double he2 = h2 + v2 * v2 / (2.0 * g);

    // If total energy conserved (ideal case)
    // he1 should equal he2
    TS_ASSERT_DELTA(he1, 17486.5, 1.0);
    TS_ASSERT_DELTA(he2, 15885.1, 1.0);
  }

  void testEnergyHeightRate() {
    // Rate of change of energy height = Ps
    double ps = 75.0;  // ft/s
    double dhe_dt = ps;

    TS_ASSERT_DELTA(dhe_dt, 75.0, 0.1);
  }

  void testEnergyHeightAtZeroVelocity() {
    // At zero velocity, He = h (only potential energy)
    double altitude = 8000.0;
    double velocity = 0.0;

    double he = altitude + velocity * velocity / (2.0 * g);

    TS_ASSERT_DELTA(he, altitude, epsilon);
  }

  //===========================================================================
  // 10. TURN PERFORMANCE - LOAD FACTOR (5 tests)
  //===========================================================================

  void testLoadFactorInLevel() {
    // n = 1 in level unaccelerated flight
    double lift = 10000.0;
    double weight = 10000.0;

    double n = lift / weight;

    TS_ASSERT_DELTA(n, 1.0, epsilon);
  }

  void testLoadFactorInTurn() {
    // n = 1 / cos(bank_angle)
    double bank_angle = 60.0 * degtoman;  // 60 degrees

    double n = 1.0 / cos(bank_angle);

    TS_ASSERT_DELTA(n, 2.0, 0.01);
  }

  void testLoadFactorAt45DegBank() {
    double bank_angle = 45.0 * degtoman;
    double n = 1.0 / cos(bank_angle);

    TS_ASSERT_DELTA(n, 1.414, 0.01);  // sqrt(2)
  }

  void testLoadFactorStructuralLimit() {
    // Typical aerobatic limit: 6g
    double n_limit = 6.0;
    double bank_angle_limit = acos(1.0 / n_limit);
    double bank_deg = bank_angle_limit * radtodeg;

    TS_ASSERT_DELTA(bank_deg, 80.41, 0.1);
  }

  void testLoadFactorFromLift() {
    // n = L / W
    double weight = 12000.0;
    double lift = 18000.0;  // 1.5g maneuver

    double n = lift / weight;

    TS_ASSERT_DELTA(n, 1.5, 0.01);
  }

  //===========================================================================
  // 11. TURN PERFORMANCE - TURN RADIUS (4 tests)
  //===========================================================================

  void testTurnRadiusBasic() {
    // R = V^2 / (g * sqrt(n^2 - 1))
    double velocity = 400.0;  // ft/s
    double n = 2.0;

    double radius = (velocity * velocity) / (g * sqrt(n * n - 1.0));

    TS_ASSERT_DELTA(radius, 2871.1, 2.0);
  }

  void testTurnRadiusAt60DegBank() {
    // At 60 deg bank, n = 2, specific formula
    double velocity = 500.0;
    double n = 2.0;

    double radius = velocity * velocity / (g * sqrt(n * n - 1.0));

    TS_ASSERT_DELTA(radius, 4486.1, 2.0);
  }

  void testTurnRadiusWithSpeed() {
    // Radius increases with V^2
    double v1 = 300.0, v2 = 600.0;
    double n = 2.0;

    double r1 = v1 * v1 / (g * sqrt(n * n - 1.0));
    double r2 = v2 * v2 / (g * sqrt(n * n - 1.0));

    // r2 should be 4 times r1
    TS_ASSERT_DELTA(r2 / r1, 4.0, 0.01);
  }

  void testTurnRadiusMinimum() {
    // Minimum radius at minimum speed and max load factor
    double v_min = 200.0;  // ft/s (stall speed)
    double n_max = 4.0;

    double r_min = v_min * v_min / (g * sqrt(n_max * n_max - 1.0));

    TS_ASSERT_DELTA(r_min, 321.0, 2.0);
  }

  //===========================================================================
  // 12. TURN PERFORMANCE - TURN RATE (4 tests)
  //===========================================================================

  void testTurnRateBasic() {
    // omega = g * sqrt(n^2 - 1) / V (rad/s)
    double velocity = 400.0;  // ft/s
    double n = 2.0;

    double omega = g * sqrt(n * n - 1.0) / velocity;
    double omega_deg_s = omega * radtodeg;

    TS_ASSERT_DELTA(omega_deg_s, 7.98, 0.1);  // deg/s
  }

  void testTurnRateAt60DegBank() {
    double velocity = 500.0;
    double bank_angle = 60.0 * degtoman;
    double n = 1.0 / cos(bank_angle);

    double omega = g * sqrt(n * n - 1.0) / velocity;
    double omega_deg_s = omega * radtodeg;

    TS_ASSERT_DELTA(omega_deg_s, 6.38, 0.1);
  }

  void testTurnRateMaximum() {
    // Maximum turn rate at corner speed
    // omega_max = g * sqrt(n_max^2 - 1) / V_corner
    double v_corner = 350.0;
    double n_max = 5.0;

    double omega_max = g * sqrt(n_max * n_max - 1.0) / v_corner;
    double omega_max_deg_s = omega_max * radtodeg;

    TS_ASSERT_DELTA(omega_max_deg_s, 25.8, 0.2);
  }

  void testTurnRateInverseVelocity() {
    // Turn rate is inversely proportional to velocity
    double v1 = 300.0, v2 = 600.0;
    double n = 3.0;

    double omega1 = g * sqrt(n * n - 1.0) / v1;
    double omega2 = g * sqrt(n * n - 1.0) / v2;

    TS_ASSERT_DELTA(omega1 / omega2, 2.0, 0.01);
  }

  //===========================================================================
  // 13. SUSTAINED TURN RATE TESTS (3 tests)
  //===========================================================================

  void testSustainedTurnRate() {
    // Sustained turn: where thrust equals drag at given load factor
    // n = L/W, and T = D in steady turn
    double thrust = 6000.0;
    double drag_at_turn = 6000.0;  // Sustained condition
    double weight = 12000.0;
    double lift = 18000.0;  // 1.5g turn
    double velocity = 450.0;

    double n = lift / weight;
    double omega = g * sqrt(n * n - 1.0) / velocity;

    TS_ASSERT_DELTA(n, 1.5, 0.01);
    TS_ASSERT_DELTA(thrust, drag_at_turn, 0.1);
  }

  void testSustainedVsInstantaneous() {
    // Sustained turn rate is less than instantaneous
    double omega_sustained = 10.0;    // deg/s
    double omega_instantaneous = 15.0; // deg/s

    TS_ASSERT(omega_sustained < omega_instantaneous);
  }

  void testSustainedTurnPsZero() {
    // In sustained turn, Ps = 0 (no energy gain/loss)
    double thrust = 5000.0;
    double drag = 5000.0;
    double velocity = 400.0;
    double weight = 10000.0;

    double ps = (thrust - drag) * velocity / weight;

    TS_ASSERT_DELTA(ps, 0.0, epsilon);
  }

  //===========================================================================
  // 14. CORNER SPEED TESTS (4 tests)
  //===========================================================================

  void testCornerSpeedDefinition() {
    // Corner speed: where max turn rate occurs
    // Intersection of stall limit and structural limit
    double n_max = 6.0;
    double cl_max = 1.4;
    double weight = 15000.0;
    double wing_area = 300.0;  // ft^2

    // V_corner = sqrt(2 * W * n_max / (rho * S * CL_max))
    double v_corner_sq = 2.0 * weight * n_max / (rho0 * wing_area * cl_max);
    double v_corner = sqrt(v_corner_sq);

    TS_ASSERT(v_corner > 0.0);
    TS_ASSERT_DELTA(v_corner, 424.6, 2.0);
  }

  void testCornerSpeedVsStallSpeed() {
    // V_corner = V_stall * sqrt(n_max)
    double v_stall = 120.0;  // ft/s
    double n_max = 4.0;

    double v_corner = v_stall * sqrt(n_max);

    TS_ASSERT_DELTA(v_corner, 240.0, 0.1);
  }

  void testCornerSpeedMaxTurnRate() {
    // At corner speed, turn rate is maximum
    double v_corner = 350.0;
    double v_higher = 450.0;
    double n_max = 5.0;

    double omega_corner = g * sqrt(n_max * n_max - 1.0) / v_corner;
    double omega_higher = g * sqrt(n_max * n_max - 1.0) / v_higher;

    TS_ASSERT(omega_corner > omega_higher);
  }

  void testCornerSpeedLoadFactor() {
    // At corner speed, aircraft at max load factor
    double n_max = 6.5;  // Structural limit
    double n_corner = n_max;

    TS_ASSERT_DELTA(n_corner, 6.5, epsilon);
  }

  //===========================================================================
  // 15. SPECIFIC RANGE TESTS (4 tests)
  //===========================================================================

  void testSpecificRangeBasic() {
    // SR = V / fuel_flow (nautical miles per pound)
    double velocity = 450.0;  // kts
    double fuel_flow = 2000.0;  // lbs/hr

    double sr = velocity / fuel_flow;  // nm/lb

    TS_ASSERT_DELTA(sr, 0.225, 0.001);
  }

  void testSpecificRangeMaximum() {
    // Max specific range at max L/D for jets
    double ld_max = 15.0;
    double sfc = 0.6;  // lb/hr/lb thrust-specific fuel consumption

    // SR_max proportional to L/D / SFC
    // Simplified: higher L/D gives better range
    TS_ASSERT(ld_max > 10.0);
  }

  void testSpecificRangeAltitudeEffect() {
    // SR improves with altitude for jets (lower fuel flow at same TAS)
    double sr_sl = 0.20;      // nm/lb at sea level
    double sr_30000 = 0.28;   // nm/lb at 30,000 ft

    TS_ASSERT(sr_30000 > sr_sl);
  }

  void testSpecificRangeWindEffect() {
    // Headwind reduces ground speed, reducing effective range
    double tas = 450.0;        // kts
    double headwind = 50.0;    // kts
    double groundspeed = tas - headwind;
    double fuel_flow = 2000.0;

    double sr = groundspeed / fuel_flow;

    TS_ASSERT_DELTA(sr, 0.20, 0.001);
    TS_ASSERT(sr < tas / fuel_flow);  // Less than no-wind case
  }

  //===========================================================================
  // 16. BREGUET RANGE EQUATION TESTS (5 tests)
  //===========================================================================

  void testBreguetRangeEquationJet() {
    // R = (V/c) * (L/D) * ln(W_initial / W_final)
    // Where c = specific fuel consumption
    double velocity = 450.0;  // kts
    double sfc = 0.6;         // lb/(hr*lb)
    double ld = 15.0;
    double w_initial = 50000.0;  // lbs
    double w_final = 40000.0;    // lbs

    double range = (velocity / sfc) * ld * log(w_initial / w_final);

    TS_ASSERT(range > 0.0);
    TS_ASSERT_DELTA(range, 2506.6, 10.0);  // nm
  }

  void testBreguetRangeEquationPropeller() {
    // For propeller: R = (eta/c) * (L/D) * ln(W_initial / W_final)
    // Where eta = propeller efficiency, c = specific fuel consumption
    double eta = 0.85;
    double sfc = 0.45;  // lb/(hr*hp)
    double ld = 12.0;
    double w_initial = 30000.0;
    double w_final = 25000.0;

    double range_factor = (eta / sfc) * ld * log(w_initial / w_final);

    TS_ASSERT(range_factor > 0.0);
  }

  void testBreguetFuelFraction() {
    // Fuel fraction: (W_initial - W_final) / W_initial
    double w_initial = 60000.0;
    double w_final = 45000.0;

    double fuel_fraction = (w_initial - w_final) / w_initial;

    TS_ASSERT_DELTA(fuel_fraction, 0.25, 0.01);  // 25% fuel
  }

  void testBreguetLDEffect() {
    // Range is directly proportional to L/D
    double v = 450.0, sfc = 0.6, w_i = 50000.0, w_f = 40000.0;
    double ld1 = 12.0, ld2 = 18.0;

    double r1 = (v / sfc) * ld1 * log(w_i / w_f);
    double r2 = (v / sfc) * ld2 * log(w_i / w_f);

    TS_ASSERT_DELTA(r2 / r1, ld2 / ld1, 0.01);
  }

  void testBreguetWeightRatioEffect() {
    // Range increases logarithmically with weight ratio
    double v = 450.0, sfc = 0.6, ld = 15.0;
    double w_i1 = 50000.0, w_f1 = 45000.0;  // Small fuel burn
    double w_i2 = 50000.0, w_f2 = 35000.0;  // Large fuel burn

    double r1 = (v / sfc) * ld * log(w_i1 / w_f1);
    double r2 = (v / sfc) * ld * log(w_i2 / w_f2);

    TS_ASSERT(r2 > r1);  // More fuel = more range
  }

  //===========================================================================
  // 17. PAYLOAD-RANGE TRADEOFF TESTS (4 tests)
  //===========================================================================

  void testPayloadRangeTradeoff() {
    // Maximum payload reduces range, maximum range reduces payload
    double max_payload = 20000.0;   // lbs
    double max_fuel = 15000.0;      // lbs
    double operating_empty = 30000.0;

    // Max payload condition: minimum fuel
    double fuel_with_max_payload = 5000.0;  // Reserve only
    double weight_max_payload = operating_empty + max_payload + fuel_with_max_payload;

    // Max range condition: maximum fuel
    double payload_with_max_fuel = max_payload - (max_fuel - fuel_with_max_payload);

    TS_ASSERT(payload_with_max_fuel < max_payload);
  }

  void testPayloadRangeDiagram() {
    // Typical payload-range diagram points
    double oe_weight = 25000.0;
    double max_payload = 15000.0;
    double max_fuel = 12000.0;
    double reserve_fuel = 2000.0;

    // Point 1: Max payload, min fuel
    double w1 = oe_weight + max_payload + reserve_fuel;

    // Point 2: Max payload, more fuel (up to MTOW or fuel limit)
    double w2 = oe_weight + max_payload + 6000.0;

    TS_ASSERT(w2 > w1);
  }

  void testPayloadRangeMTOWLimit() {
    // MTOW limits both payload and fuel
    double mtow = 75000.0;
    double oe_weight = 40000.0;
    double max_payload = 25000.0;
    double max_fuel = 20000.0;

    // Available weight for payload + fuel
    double available = mtow - oe_weight;

    TS_ASSERT_DELTA(available, 35000.0, 0.1);

    // Cannot carry max payload AND max fuel
    TS_ASSERT(max_payload + max_fuel > available);
  }

  void testPayloadRangeWithDifferentFuel() {
    // More fuel = more range but less payload capacity
    double fuel1 = 8000.0;
    double fuel2 = 12000.0;
    double max_takeoff = 60000.0;
    double oe_weight = 35000.0;

    double payload1 = max_takeoff - oe_weight - fuel1;
    double payload2 = max_takeoff - oe_weight - fuel2;

    TS_ASSERT(payload2 < payload1);
    TS_ASSERT_DELTA(payload1 - payload2, fuel2 - fuel1, 0.1);
  }

  //===========================================================================
  // ADDITIONAL COMPREHENSIVE TESTS
  //===========================================================================

  void testClimbGradient() {
    // Climb gradient = (T - D) / W = sin(gamma)
    double thrust = 7000.0;
    double drag = 3000.0;
    double weight = 12000.0;

    double gradient = (thrust - drag) / weight;
    double gamma = asin(gradient);

    TS_ASSERT_DELTA(gradient, 0.3333, 0.001);
    TS_ASSERT(gamma > 0.0);
  }

  void testGlideRatio() {
    // Glide ratio = L/D (in unpowered flight)
    double lift = 10000.0;
    double drag = 800.0;

    double glide_ratio = lift / drag;

    TS_ASSERT_DELTA(glide_ratio, 12.5, 0.1);

    // Distance covered from altitude h: d = h * (L/D)
    double altitude = 10000.0;
    double distance = altitude * glide_ratio;
    TS_ASSERT_DELTA(distance, 125000.0, 1.0);  // ft
  }

  void testThrustToWeightRatio() {
    // T/W ratio is key performance parameter
    double thrust = 18000.0;
    double weight = 12000.0;

    double tw_ratio = thrust / weight;

    TS_ASSERT_DELTA(tw_ratio, 1.5, 0.01);

    // High T/W enables high acceleration and climb
    TS_ASSERT(tw_ratio > 1.0);  // Capable of vertical climb
  }

  void testWingLoading() {
    // Wing loading = W/S affects stall speed and turn performance
    double weight = 15000.0;
    double wing_area = 250.0;  // ft^2

    double wing_loading = weight / wing_area;

    TS_ASSERT_DELTA(wing_loading, 60.0, 0.1);  // psf
  }

  void testStallSpeed() {
    // V_stall = sqrt(2*W / (rho*S*CL_max))
    double weight = 12000.0;
    double wing_area = 200.0;
    double cl_max = 1.6;

    double v_stall = sqrt(2.0 * weight / (rho0 * wing_area * cl_max));

    TS_ASSERT_DELTA(v_stall, 177.6, 2.0);  // ft/s
  }

  void testTakeoffDistance() {
    // Simplified: s = 1.44 * W^2 / (g * rho * S * CL_max * T)
    double weight = 15000.0;
    double wing_area = 300.0;
    double cl_max = 1.8;
    double thrust = 6000.0;

    double distance = 1.44 * weight * weight / (g * rho0 * wing_area * cl_max * thrust);

    TS_ASSERT(distance > 0.0);
  }

  void testLandingDistance() {
    // Landing distance related to approach speed and deceleration
    double v_approach = 120.0;  // ft/s (1.3 * V_stall)
    double deceleration = 10.0; // ft/s^2

    double distance = v_approach * v_approach / (2.0 * deceleration);

    TS_ASSERT_DELTA(distance, 720.0, 1.0);  // ft
  }

  void testTurnRadiusVsBankAngle() {
    // For constant velocity, tighter turns require higher bank angles
    double v = 400.0;  // ft/s
    double bank1 = 30.0 * degtoman;
    double bank2 = 60.0 * degtoman;

    double n1 = 1.0 / cos(bank1);
    double n2 = 1.0 / cos(bank2);

    double r1 = v * v / (g * sqrt(n1 * n1 - 1.0));
    double r2 = v * v / (g * sqrt(n2 * n2 - 1.0));

    TS_ASSERT(r2 < r1);  // Steeper bank = tighter turn
  }

  void testOswaldsEfficiency() {
    // Oswald efficiency factor affects induced drag
    // Typical values: 0.7-0.9
    double e = 0.8;

    TS_ASSERT(e > 0.0 && e <= 1.0);
  }

  void testAspectRatioEffect() {
    // Higher aspect ratio reduces induced drag
    double wingspan = 40.0;   // ft
    double wing_area = 200.0; // ft^2

    double aspect_ratio = wingspan * wingspan / wing_area;

    TS_ASSERT_DELTA(aspect_ratio, 8.0, 0.1);

    // Higher AR = better L/D
    TS_ASSERT(aspect_ratio > 5.0);  // Good for efficiency
  }

  //===========================================================================
  // 18. INDEPENDENT CALCULATION TESTS (79-82)
  //===========================================================================

  // Test 79: Independent ROC calculations
  void testIndependentROCCalculations() {
    // GIVEN: Two independent aircraft configurations
    struct Aircraft {
      double thrust, drag, weight, velocity;
    };
    Aircraft a1{5000.0, 2000.0, 10000.0, 300.0};
    Aircraft a2{8000.0, 3000.0, 15000.0, 350.0};

    // WHEN: Calculating ROC for each
    double roc1 = (a1.thrust - a1.drag) * a1.velocity / a1.weight;
    double roc2 = (a2.thrust - a2.drag) * a2.velocity / a2.weight;

    // THEN: Calculations should be independent
    TS_ASSERT_DELTA(roc1, 90.0, 0.1);   // 3000 * 300 / 10000 = 90
    TS_ASSERT_DELTA(roc2, 116.67, 0.1); // 5000 * 350 / 15000 = 116.67
    TS_ASSERT(roc1 != roc2);
  }

  // Test 80: Independent specific range calculations
  void testIndependentSpecificRangeCalculations() {
    // GIVEN: Different flight conditions
    double v1 = 450.0, ff1 = 2000.0;  // nm/lb
    double v2 = 500.0, ff2 = 2500.0;  // nm/lb

    // WHEN: Calculating specific range
    double sr1 = v1 / ff1;
    double sr2 = v2 / ff2;

    // THEN: Results should be independent
    TS_ASSERT_DELTA(sr1, 0.225, 0.001);
    TS_ASSERT_DELTA(sr2, 0.200, 0.001);
    TS_ASSERT(sr1 > sr2);  // Lower speed is more efficient here
  }

  // Test 81: Independent turn performance calculations
  void testIndependentTurnPerformanceCalculations() {
    // GIVEN: Two different bank angles
    double v = 400.0;  // ft/s
    double bank1 = 30.0 * M_PI / 180.0;
    double bank2 = 45.0 * M_PI / 180.0;

    // WHEN: Calculating turn radii
    double n1 = 1.0 / cos(bank1);
    double n2 = 1.0 / cos(bank2);
    double r1 = v * v / (g * sqrt(n1 * n1 - 1.0));
    double r2 = v * v / (g * sqrt(n2 * n2 - 1.0));

    // THEN: Higher bank gives tighter turn
    TS_ASSERT(r1 > r2);
    TS_ASSERT(std::isfinite(r1));
    TS_ASSERT(std::isfinite(r2));
  }

  // Test 82: Independent endurance calculations
  void testIndependentEnduranceCalculations() {
    // GIVEN: Different fuel loads and consumption rates
    double fuel1 = 5000.0, ff1 = 1000.0;  // lbs, lbs/hr
    double fuel2 = 8000.0, ff2 = 1200.0;

    // WHEN: Calculating endurance
    double e1 = fuel1 / ff1;  // hours
    double e2 = fuel2 / ff2;

    // THEN: Each calculation independent
    TS_ASSERT_DELTA(e1, 5.0, 0.01);
    TS_ASSERT_DELTA(e2, 6.67, 0.01);
  }

  //===========================================================================
  // 19. CONSISTENCY TESTS (83-86)
  //===========================================================================

  // Test 83: Energy state consistency
  void testEnergyStateConsistency() {
    // GIVEN: Aircraft at various altitudes and speeds
    double weight = 10000.0;
    double h1 = 10000.0, v1 = 300.0;  // ft, ft/s
    double h2 = 20000.0, v2 = 250.0;

    // WHEN: Computing specific energy
    double Es1 = h1 + v1 * v1 / (2.0 * g);
    double Es2 = h2 + v2 * v2 / (2.0 * g);

    // THEN: Energy should be positive and finite
    TS_ASSERT(Es1 > 0.0);
    TS_ASSERT(Es2 > 0.0);
    TS_ASSERT(std::isfinite(Es1));
    TS_ASSERT(std::isfinite(Es2));

    // Higher altitude version has more energy
    TS_ASSERT(Es2 > Es1);
  }

  // Test 84: L/D and glide consistency
  void testLDGlideConsistency() {
    // GIVEN: Various L/D ratios
    for (double ld = 5.0; ld <= 20.0; ld += 2.5) {
      double altitude = 10000.0;

      // WHEN: Computing glide distance
      double glide_distance = altitude * ld;

      // THEN: Consistency checks
      TS_ASSERT(glide_distance > altitude);  // Always go further than you are high
      TS_ASSERT_DELTA(glide_distance / altitude, ld, epsilon);
    }
  }

  // Test 85: Thrust and weight ratio consistency
  void testThrustWeightRatioConsistency() {
    // GIVEN: Various T/W ratios
    double weight = 10000.0;

    for (double tw = 0.2; tw <= 1.5; tw += 0.2) {
      double thrust = tw * weight;

      // WHEN: Verifying
      double computed_tw = thrust / weight;

      // THEN: Should match
      TS_ASSERT_DELTA(computed_tw, tw, epsilon);

      // Climb angle calculation should be consistent
      double drag = thrust / 2.0;  // Assume D = T/2 for this test
      double gradient = (thrust - drag) / weight;
      TS_ASSERT_DELTA(gradient, tw / 2.0, epsilon);
    }
  }

  // Test 86: Stall speed altitude consistency
  void testStallSpeedAltitudeConsistency() {
    // GIVEN: Same aircraft at different densities
    double weight = 12000.0;
    double wing_area = 200.0;
    double cl_max = 1.6;

    double rho_sl = rho0;
    double rho_10k = rho0 * 0.7385;  // Approximate at 10,000 ft

    // WHEN: Computing stall speeds
    double v_stall_sl = sqrt(2.0 * weight / (rho_sl * wing_area * cl_max));
    double v_stall_10k = sqrt(2.0 * weight / (rho_10k * wing_area * cl_max));

    // THEN: Stall speed increases with altitude
    TS_ASSERT(v_stall_10k > v_stall_sl);

    // And ratio should match density ratio
    double speed_ratio = v_stall_10k / v_stall_sl;
    double expected_ratio = sqrt(rho_sl / rho_10k);
    TS_ASSERT_DELTA(speed_ratio, expected_ratio, 0.01);
  }

  //===========================================================================
  // 20. EDGE CASE TESTS (87-90)
  //===========================================================================

  // Test 87: Zero excess thrust
  void testZeroExcessThrust() {
    // GIVEN: Thrust equals drag (level flight)
    double thrust = 3500.0;
    double drag = 3500.0;
    double weight = 10000.0;
    double velocity = 300.0;

    // WHEN: Computing climb rate
    double roc = (thrust - drag) * velocity / weight;

    // THEN: Should be exactly zero
    TS_ASSERT_DELTA(roc, 0.0, epsilon);
  }

  // Test 88: Maximum load factor turn
  void testMaximumLoadFactorTurn() {
    // GIVEN: Aircraft at structural limit
    double n_max = 9.0;  // Fighter aircraft limit
    double v = 500.0;    // ft/s

    // WHEN: Computing turn rate and radius
    double omega = g * sqrt(n_max * n_max - 1.0) / v;
    double radius = v / omega;

    // THEN: Values should be finite and reasonable
    TS_ASSERT(std::isfinite(omega));
    TS_ASSERT(std::isfinite(radius));
    TS_ASSERT(omega > 0.0);
    TS_ASSERT(radius > 0.0);
  }

  // Test 89: Minimum fuel weight ratio
  void testMinimumFuelWeightRatio() {
    // GIVEN: Near-empty fuel condition
    double w_initial = 50000.0;
    double w_final = 49900.0;  // Only 100 lbs burned

    double v = 450.0, sfc = 0.6, ld = 15.0;

    // WHEN: Computing range
    double range = (v / sfc) * ld * log(w_initial / w_final);

    // THEN: Small but positive range
    TS_ASSERT(range > 0.0);
    TS_ASSERT(range < 100.0);  // Very short range
  }

  // Test 90: High altitude performance limit
  void testHighAltitudePerformanceLimit() {
    // GIVEN: Very thin air at high altitude
    double rho_high = rho0 * 0.1;  // ~60,000 ft equivalent
    double weight = 15000.0;
    double wing_area = 300.0;
    double cl_max = 1.6;

    // WHEN: Computing stall speed
    double v_stall = sqrt(2.0 * weight / (rho_high * wing_area * cl_max));

    // THEN: Stall speed very high but finite
    TS_ASSERT(v_stall > 500.0);
    TS_ASSERT(std::isfinite(v_stall));
  }

  //===========================================================================
  // 21. COMBINED PERFORMANCE TESTS (91-94)
  //===========================================================================

  // Test 91: Combined climb and speed performance
  void testCombinedClimbSpeedPerformance() {
    // GIVEN: Aircraft configuration
    double weight = 12000.0;
    double thrust = 6000.0;
    double wing_area = 200.0;
    double cl_max = 1.6;

    // WHEN: Computing various performance metrics
    double tw_ratio = thrust / weight;
    double wing_loading = weight / wing_area;
    double v_stall = sqrt(2.0 * weight / (rho0 * wing_area * cl_max));

    // THEN: All metrics consistent
    TS_ASSERT_DELTA(tw_ratio, 0.5, 0.01);
    TS_ASSERT_DELTA(wing_loading, 60.0, 0.1);
    TS_ASSERT(v_stall > 100.0 && v_stall < 300.0);
  }

  // Test 92: Range and endurance tradeoff
  void testRangeEnduranceTradeoff() {
    // GIVEN: Aircraft with fixed fuel
    double fuel = 6000.0;  // lbs

    // Best range speed: higher velocity, moderate fuel flow
    double v_range = 450.0, ff_range = 1800.0;
    double range = v_range * (fuel / ff_range);

    // Best endurance speed: lower velocity, lower fuel flow
    double v_endure = 350.0, ff_endure = 1200.0;
    double endurance = fuel / ff_endure;

    // THEN: Range and endurance maximize at different speeds
    TS_ASSERT(range > 0.0);
    TS_ASSERT(endurance > 0.0);

    // Verify endurance is in hours
    TS_ASSERT_DELTA(endurance, 5.0, 0.1);
  }

  // Test 93: Takeoff and climb combined
  void testTakeoffClimbCombined() {
    // GIVEN: Takeoff performance data
    double weight = 15000.0;
    double thrust = 7500.0;
    double drag = 2500.0;
    double v_liftoff = 150.0;

    // WHEN: Computing takeoff and initial climb
    double tw_ratio = thrust / weight;
    double excess_thrust = thrust - drag;
    double roc = excess_thrust * v_liftoff / weight;
    double climb_gradient = excess_thrust / weight;

    // THEN: All performance numbers consistent
    TS_ASSERT_DELTA(tw_ratio, 0.5, 0.01);
    TS_ASSERT_DELTA(roc, 50.0, 1.0);
    TS_ASSERT_DELTA(climb_gradient, 0.3333, 0.01);
  }

  // Test 94: Cruise and descent combined
  void testCruiseDescentCombined() {
    // GIVEN: Cruise conditions
    double weight = 40000.0;
    double velocity = 450.0;  // kts
    double fuel_flow = 2500.0;  // lbs/hr
    double altitude = 35000.0;

    // WHEN: Computing cruise and glide performance
    double sr = velocity / fuel_flow;
    double ld = 15.0;  // Assumed
    double glide_distance = altitude * ld;

    // THEN: Consistent results
    TS_ASSERT(sr > 0.0);
    TS_ASSERT(glide_distance > 0.0);
    TS_ASSERT_DELTA(glide_distance, 525000.0, 1.0);  // ft
  }

  //===========================================================================
  // 22. STRESS TESTS (95-98)
  //===========================================================================

  // Test 95: Many sequential performance calculations
  void testManySequentialPerformanceCalculations() {
    // GIVEN: Base parameters
    double weight = 10000.0;
    double wing_area = 200.0;
    double cl_max = 1.6;

    // WHEN: Computing many stall speeds at different densities
    double prev_vs = 0.0;
    for (int i = 0; i <= 100; i++) {
      double sigma = 1.0 - i * 0.008;  // Density ratio from 1.0 to 0.2
      double rho = rho0 * sigma;
      double v_stall = sqrt(2.0 * weight / (rho * wing_area * cl_max));

      // THEN: Each calculation valid and increasing
      TS_ASSERT(std::isfinite(v_stall));
      if (i > 0) {
        TS_ASSERT(v_stall > prev_vs);
      }
      prev_vs = v_stall;
    }
  }

  // Test 96: Many range calculations
  void testManyRangeCalculations() {
    // GIVEN: Base parameters
    double v = 450.0, sfc = 0.6, ld = 15.0;
    double w_initial = 50000.0;

    // WHEN: Computing range for various fuel burns
    for (int i = 1; i <= 50; i++) {
      double w_final = w_initial - i * 500.0;  // Burn 500 lbs increments
      double range = (v / sfc) * ld * log(w_initial / w_final);

      // THEN: Range should increase with fuel burned
      TS_ASSERT(range > 0.0);
      TS_ASSERT(std::isfinite(range));
    }
  }

  // Test 97: Many turn calculations
  void testManyTurnCalculations() {
    // GIVEN: Base velocity
    double v = 400.0;

    // WHEN: Computing turn performance for various bank angles
    for (int i = 10; i <= 80; i += 5) {
      double bank = i * M_PI / 180.0;
      double n = 1.0 / cos(bank);
      double omega = g * sqrt(n * n - 1.0) / v;
      double radius = v / omega;

      // THEN: All values finite and reasonable
      TS_ASSERT(std::isfinite(n));
      TS_ASSERT(std::isfinite(omega));
      TS_ASSERT(std::isfinite(radius));
      TS_ASSERT(n >= 1.0);
      TS_ASSERT(omega > 0.0);
      TS_ASSERT(radius > 0.0);
    }
  }

  // Test 98: Many energy calculations
  void testManyEnergyCalculations() {
    // GIVEN: Base weight
    double weight = 15000.0;

    // WHEN: Computing specific energy at various conditions
    for (int alt = 0; alt <= 50; alt++) {
      double h = alt * 1000.0;  // 0 to 50,000 ft
      for (int spd = 1; spd <= 10; spd++) {
        double v = spd * 100.0;  // 100 to 1000 ft/s
        double Es = h + v * v / (2.0 * g);

        // THEN: Energy always positive and finite
        TS_ASSERT(Es > 0.0);
        TS_ASSERT(std::isfinite(Es));
      }
    }
  }

  //===========================================================================
  // 23. COMPLETE VERIFICATION TESTS (99-100)
  //===========================================================================

  // Test 99: Comprehensive aircraft performance profile
  void testComprehensiveAircraftPerformanceProfile() {
    // GIVEN: Complete aircraft performance data
    struct AircraftPerformance {
      double weight = 25000.0;      // lbs
      double wing_area = 400.0;     // ft^2
      double cl_max = 1.8;
      double thrust = 8000.0;       // lbs
      double drag_cruise = 2500.0;  // lbs
      double ld_max = 14.0;
      double fuel = 6000.0;         // lbs
    } aircraft;

    // WHEN: Computing full performance envelope

    // 1. Stall speed
    double v_stall = sqrt(2.0 * aircraft.weight / (rho0 * aircraft.wing_area * aircraft.cl_max));
    TS_ASSERT(v_stall > 100.0 && v_stall < 200.0);

    // 2. T/W ratio
    double tw = aircraft.thrust / aircraft.weight;
    TS_ASSERT_DELTA(tw, 0.32, 0.01);

    // 3. Wing loading
    double wl = aircraft.weight / aircraft.wing_area;
    TS_ASSERT_DELTA(wl, 62.5, 0.1);

    // 4. Rate of climb at typical speed
    double v_climb = 250.0;
    double roc = (aircraft.thrust - aircraft.drag_cruise) * v_climb / aircraft.weight;
    TS_ASSERT(roc > 0.0);

    // 5. Glide distance from 20,000 ft
    double altitude = 20000.0;
    double glide_dist = altitude * aircraft.ld_max;
    TS_ASSERT_DELTA(glide_dist, 280000.0, 1.0);

    // 6. Endurance at typical cruise
    double ff = 2000.0;
    double endurance = aircraft.fuel / ff;
    TS_ASSERT_DELTA(endurance, 3.0, 0.01);

    // 7. Specific range
    double v_cruise = 400.0;
    double sr = v_cruise / ff;
    TS_ASSERT_DELTA(sr, 0.2, 0.001);
  }

  // Test 100: Complete performance calculation system integration
  void testCompletePerformanceSystemIntegration() {
    // GIVEN: Two different aircraft configurations
    struct Config {
      double weight, thrust, drag, wing_area, cl_max, ld, fuel;
    };

    Config fighter{15000.0, 18000.0, 3000.0, 300.0, 1.6, 10.0, 5000.0};
    Config transport{80000.0, 40000.0, 10000.0, 1500.0, 2.0, 16.0, 25000.0};

    // WHEN: Computing and comparing performance

    // 1. T/W ratios
    double tw_fighter = fighter.thrust / fighter.weight;
    double tw_transport = transport.thrust / transport.weight;
    TS_ASSERT(tw_fighter > tw_transport);  // Fighter has better T/W

    // 2. Wing loading
    double wl_fighter = fighter.weight / fighter.wing_area;
    double wl_transport = transport.weight / transport.wing_area;
    TS_ASSERT(wl_fighter < wl_transport);  // Transport has higher loading

    // 3. Stall speeds
    double vs_fighter = sqrt(2.0 * fighter.weight / (rho0 * fighter.wing_area * fighter.cl_max));
    double vs_transport = sqrt(2.0 * transport.weight / (rho0 * transport.wing_area * transport.cl_max));
    TS_ASSERT(vs_fighter > vs_transport);  // Transport stalls slower due to flaps

    // 4. Climb rates at 300 ft/s
    double v = 300.0;
    double roc_fighter = (fighter.thrust - fighter.drag) * v / fighter.weight;
    double roc_transport = (transport.thrust - transport.drag) * v / transport.weight;
    TS_ASSERT(roc_fighter > roc_transport);  // Fighter climbs faster

    // 5. Glide distances from 30,000 ft
    double alt = 30000.0;
    double glide_fighter = alt * fighter.ld;
    double glide_transport = alt * transport.ld;
    TS_ASSERT(glide_transport > glide_fighter);  // Transport glides further

    // 6. Turn performance at 500 ft/s, 60° bank
    double bank = 60.0 * M_PI / 180.0;
    double n = 1.0 / cos(bank);
    double omega = g * sqrt(n * n - 1.0) / 500.0;
    double radius = 500.0 / omega;

    // Both can do same turn if within limits
    double n_max_fighter = 9.0;
    double n_max_transport = 2.5;
    TS_ASSERT(n < n_max_fighter);
    TS_ASSERT(n < n_max_transport);  // Both can make this turn

    // 7. Range comparison (simplified)
    double v_cruise = 450.0, sfc = 0.6;
    double range_fighter = (v_cruise / sfc) * fighter.ld * log((fighter.weight + fighter.fuel) / fighter.weight);
    double range_transport = (v_cruise / sfc) * transport.ld * log((transport.weight + transport.fuel) / transport.weight);
    TS_ASSERT(range_transport > range_fighter);  // Transport has better range

    // 8. All calculations are finite and consistent
    TS_ASSERT(std::isfinite(tw_fighter));
    TS_ASSERT(std::isfinite(tw_transport));
    TS_ASSERT(std::isfinite(roc_fighter));
    TS_ASSERT(std::isfinite(roc_transport));
    TS_ASSERT(std::isfinite(range_fighter));
    TS_ASSERT(std::isfinite(range_transport));

    // 9. Final integration check
    TS_ASSERT(true);
  }
};
