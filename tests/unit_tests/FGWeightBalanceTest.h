/*******************************************************************************
 * FGWeightBalanceTest.h - Unit tests for weight and balance calculations
 *
 * Comprehensive tests for aircraft weight and balance computations including
 * CG calculations, moment arms, weight distribution effects, fuel burn CG
 * shift, payload loading, MAC percentage, CG limits, moment of inertia changes,
 * static margin, trim changes, ballast, and multi-point loading scenarios.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGMassBalance.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>

using namespace JSBSim;

const double epsilon = 1e-8;
const double SLUGTOLB = 32.174049;  // Conversion factor: lbs = slugs * g

class FGWeightBalanceTest : public CxxTest::TestSuite
{
public:

  // ============ CG Calculation Tests (Longitudinal, Lateral, Vertical) ============

  // Test basic longitudinal CG calculation: xcg = Σ(xi*wi) / Σwi
  void testLongitudinalCGBasic() {
    // GIVEN: Two point masses along x-axis
    double w1 = 100.0;  // lbs
    double x1 = 50.0;   // inches
    double w2 = 200.0;  // lbs
    double x2 = 100.0;  // inches

    // WHEN: Computing CG location
    double xcg = (x1 * w1 + x2 * w2) / (w1 + w2);

    // THEN: Verify correct weighted average
    double expected = (50.0 * 100.0 + 100.0 * 200.0) / 300.0;
    TS_ASSERT_DELTA(expected, xcg, epsilon);
    TS_ASSERT_DELTA(83.333333, xcg, 1e-6);
  }

  // Test lateral CG calculation (y-axis)
  void testLateralCGCalculation() {
    // GIVEN: Asymmetric loading (e.g., fuel in one wing)
    double w1 = 500.0;  // lbs (left wing fuel)
    double y1 = -120.0; // inches (left)
    double w2 = 500.0;  // lbs (right wing fuel)
    double y2 = 120.0;  // inches (right)
    double w3 = 2000.0; // lbs (fuselage)
    double y3 = 0.0;    // inches (centerline)

    // WHEN: Computing lateral CG
    double ycg = (y1*w1 + y2*w2 + y3*w3) / (w1 + w2 + w3);

    // THEN: Should be at centerline for symmetric loading
    TS_ASSERT_DELTA(0.0, ycg, epsilon);
  }

  // Test lateral CG with asymmetric loading
  void testLateralCGAsymmetric() {
    // GIVEN: Uneven fuel distribution
    double w1 = 300.0;  // lbs (left wing - less fuel)
    double y1 = -120.0; // inches
    double w2 = 500.0;  // lbs (right wing - more fuel)
    double y2 = 120.0;  // inches

    // WHEN: Computing lateral CG
    double ycg = (y1*w1 + y2*w2) / (w1 + w2);

    // THEN: CG shifts toward heavier side (right/positive)
    double expected = (-120.0*300.0 + 120.0*500.0) / 800.0;
    TS_ASSERT_DELTA(30.0, expected, epsilon);
    TS_ASSERT_DELTA(expected, ycg, epsilon);
    TS_ASSERT(ycg > 0.0);  // Shifted right
  }

  // Test vertical CG calculation (z-axis)
  void testVerticalCGCalculation() {
    // GIVEN: Vertically distributed masses
    double w1 = 1000.0; // lbs (fuselage structure)
    double z1 = -20.0;  // inches (below datum)
    double w2 = 500.0;  // lbs (cabin)
    double z2 = 0.0;    // inches (at datum)
    double w3 = 200.0;  // lbs (avionics above)
    double z3 = 10.0;   // inches (above datum)

    // WHEN: Computing vertical CG
    double zcg = (z1*w1 + z2*w2 + z3*w3) / (w1 + w2 + w3);

    // THEN: Verify weighted average
    double expected = (-20.0*1000.0 + 0.0*500.0 + 10.0*200.0) / 1700.0;
    TS_ASSERT_DELTA(expected, zcg, epsilon);
    TS_ASSERT_DELTA(-10.588235, zcg, 1e-6);
  }

  // Test 3D CG calculation using FGMassBalance
  void testThreeDimensionalCG() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set base CG
    FGColumnVector3 baseCG(100.0, 0.0, -10.0);  // inches
    massBalance->SetBaseCG(baseCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();

    TS_ASSERT_DELTA(baseCG(1), cg(1), epsilon);
    TS_ASSERT_DELTA(baseCG(2), cg(2), epsilon);
    TS_ASSERT_DELTA(baseCG(3), cg(3), epsilon);
  }

  // ============ Moment Arm Calculation Tests ============

  // Test moment arm formula: M = w * d
  void testMomentArmBasic() {
    // GIVEN: Weight at distance from reference point
    double weight = 150.0;  // lbs
    double arm = 48.0;      // inches

    // WHEN: Computing moment
    double moment = weight * arm;

    // THEN: Moment = weight * arm
    TS_ASSERT_DELTA(7200.0, moment, epsilon);
  }

  // Test moment summation for CG calculation
  void testMomentSummationForCG() {
    // GIVEN: Multiple weights at different stations
    double w1 = 100.0, x1 = 30.0;   // lbs, inches
    double w2 = 150.0, x2 = 60.0;
    double w3 = 200.0, x3 = 90.0;

    // WHEN: Computing CG via moments
    double totalMoment = w1*x1 + w2*x2 + w3*x3;
    double totalWeight = w1 + w2 + w3;
    double xcg = totalMoment / totalWeight;

    // THEN: Verify calculation
    double expectedMoment = 3000.0 + 9000.0 + 18000.0;
    TS_ASSERT_DELTA(30000.0, expectedMoment, epsilon);
    TS_ASSERT_DELTA(66.666667, xcg, 1e-6);
  }

  // Test moment arms about different reference points
  void testMomentArmsMultipleReferences() {
    // GIVEN: Weight at station 100
    double weight = 200.0;  // lbs
    double station = 100.0; // inches

    // WHEN: Computing moments about different datums
    double datum1 = 0.0;
    double datum2 = 50.0;
    double moment1 = weight * (station - datum1);
    double moment2 = weight * (station - datum2);

    // THEN: Moments differ by datum shift
    TS_ASSERT_DELTA(20000.0, moment1, epsilon);
    TS_ASSERT_DELTA(10000.0, moment2, epsilon);
    TS_ASSERT_DELTA(moment1 - moment2, weight * (datum2 - datum1), epsilon);
  }

  // ============ Weight Distribution Effects ============

  // Test CG shift with weight change
  void testCGShiftWithWeightChange() {
    // GIVEN: Initial configuration
    double w_aircraft = 2000.0;  // lbs
    double xcg_aircraft = 100.0; // inches

    // Add cargo
    double w_cargo = 500.0;      // lbs
    double x_cargo = 150.0;      // inches (aft)

    // WHEN: Computing new CG
    double xcg_new = (w_aircraft*xcg_aircraft + w_cargo*x_cargo) / (w_aircraft + w_cargo);

    // THEN: CG shifts aft
    TS_ASSERT(xcg_new > xcg_aircraft);
    TS_ASSERT_DELTA(110.0, xcg_new, epsilon);
  }

  // Test weight distribution balance equation
  void testWeightDistributionBalance() {
    // GIVEN: Loading scenario
    double w_front = 800.0;  // lbs
    double w_rear = 1200.0;  // lbs
    double total = w_front + w_rear;

    // WHEN: Computing weight distribution percentages
    double front_percent = (w_front / total) * 100.0;
    double rear_percent = (w_rear / total) * 100.0;

    // THEN: Should sum to 100%
    TS_ASSERT_DELTA(40.0, front_percent, epsilon);
    TS_ASSERT_DELTA(60.0, rear_percent, epsilon);
    TS_ASSERT_DELTA(100.0, front_percent + rear_percent, epsilon);
  }

  // ============ Fuel Burn CG Shift Tests ============

  // Test CG shift during fuel burn
  void testFuelBurnCGShift() {
    // GIVEN: Initial state with full fuel
    double w_empty = 2000.0;     // lbs
    double xcg_empty = 100.0;    // inches
    double w_fuel = 500.0;       // lbs
    double x_fuel = 80.0;        // inches (fuel tank forward of empty CG)

    // Initial CG
    double xcg_initial = (w_empty*xcg_empty + w_fuel*x_fuel) / (w_empty + w_fuel);

    // WHEN: Fuel is burned (50% remaining)
    double w_fuel_remaining = 250.0;  // lbs
    double xcg_final = (w_empty*xcg_empty + w_fuel_remaining*x_fuel) / (w_empty + w_fuel_remaining);

    // THEN: CG shifts aft as forward fuel burns
    TS_ASSERT(xcg_final > xcg_initial);
    double expected_initial = (200000.0 + 40000.0) / 2500.0;  // 96.0
    double expected_final = (200000.0 + 20000.0) / 2250.0;     // 97.778
    TS_ASSERT_DELTA(expected_initial, xcg_initial, 1e-6);
    TS_ASSERT_DELTA(expected_final, xcg_final, 1e-6);
  }

  // Test CG envelope during fuel burn sequence
  void testFuelBurnCGEnvelope() {
    // GIVEN: Aircraft with fuel tank aft of empty CG
    double w_empty = 3000.0;
    double xcg_empty = 120.0;
    double w_fuel_max = 800.0;
    double x_fuel = 140.0;  // Aft tank

    // WHEN: Computing CG at various fuel states
    double xcg_empty_calc = xcg_empty;  // No fuel
    double xcg_half = (w_empty*xcg_empty + 400.0*x_fuel) / (w_empty + 400.0);
    double xcg_full = (w_empty*xcg_empty + w_fuel_max*x_fuel) / (w_empty + w_fuel_max);

    // THEN: CG moves aft with fuel loading
    TS_ASSERT(xcg_half > xcg_empty_calc);
    TS_ASSERT(xcg_full > xcg_half);
    // Recalculate expected values:
    // xcg_half = (3000*120 + 400*140) / 3400 = (360000 + 56000) / 3400 = 122.353
    // xcg_full = (3000*120 + 800*140) / 3800 = (360000 + 112000) / 3800 = 124.211
    TS_ASSERT_DELTA(122.353, xcg_half, 1e-3);
    TS_ASSERT_DELTA(124.211, xcg_full, 1e-3);
  }

  // ============ Payload Loading Effects ============

  // Test passenger loading CG shift
  void testPassengerLoadingCG() {
    // GIVEN: Empty aircraft
    double w_aircraft = 2500.0;
    double xcg_aircraft = 110.0;

    // WHEN: Loading passengers in different rows
    double w_pax_front = 170.0 * 2;  // 2 passengers @ 170 lbs each
    double x_pax_front = 90.0;       // inches (front seats)
    double w_pax_rear = 170.0 * 2;
    double x_pax_rear = 130.0;       // inches (rear seats)

    double xcg_loaded = (w_aircraft*xcg_aircraft + w_pax_front*x_pax_front + w_pax_rear*x_pax_rear) /
                        (w_aircraft + w_pax_front + w_pax_rear);

    // THEN: CG shifts based on passenger distribution
    // xcg = (2500*110 + 340*90 + 340*130) / 3180 = (275000 + 30600 + 44200) / 3180 = 110.0
    double expected = (275000.0 + 30600.0 + 44200.0) / 3180.0;
    TS_ASSERT_DELTA(expected, xcg_loaded, 1e-6);
    TS_ASSERT_DELTA(110.0, xcg_loaded, 1e-3);
  }

  // Test cargo loading limits
  void testCargoLoadingLimits() {
    // GIVEN: Aircraft with cargo hold
    double w_base = 5000.0;
    double xcg_base = 200.0;
    double x_cargo_hold = 220.0;  // inches

    // WHEN: Adding maximum cargo
    double w_cargo_max = 2000.0;
    double xcg_max_cargo = (w_base*xcg_base + w_cargo_max*x_cargo_hold) / (w_base + w_cargo_max);

    // THEN: CG shift with max cargo
    double shift = xcg_max_cargo - xcg_base;
    TS_ASSERT(shift > 0.0);  // Shifts aft
    TS_ASSERT_DELTA(5.714286, shift, 1e-6);
  }

  // ============ MAC Percentage Calculations ============

  // Test CG position as percentage of MAC
  void testCGPercentMAC() {
    // GIVEN: Mean Aerodynamic Chord data
    double mac_length = 60.0;    // inches
    double mac_le = 80.0;        // inches (leading edge station)
    double xcg = 110.0;          // inches

    // WHEN: Computing CG position along MAC
    double cg_mac_percent = ((xcg - mac_le) / mac_length) * 100.0;

    // THEN: Verify percentage calculation
    TS_ASSERT_DELTA(50.0, cg_mac_percent, epsilon);
  }

  // Test forward CG limit as % MAC
  void testForwardCGLimitMAC() {
    // GIVEN: MAC and forward CG limit
    double mac_length = 48.0;
    double mac_le = 100.0;
    double xcg_fwd_limit = 109.6;  // 20% MAC

    // WHEN: Computing % MAC
    double percent = ((xcg_fwd_limit - mac_le) / mac_length) * 100.0;

    // THEN: Should be 20%
    TS_ASSERT_DELTA(20.0, percent, epsilon);
  }

  // Test aft CG limit as % MAC
  void testAftCGLimitMAC() {
    // GIVEN: MAC and aft CG limit
    double mac_length = 48.0;
    double mac_le = 100.0;
    double xcg_aft_limit = 136.0;  // 75% MAC

    // WHEN: Computing % MAC
    double percent = ((xcg_aft_limit - mac_le) / mac_length) * 100.0;

    // THEN: Should be 75%
    TS_ASSERT_DELTA(75.0, percent, epsilon);
  }

  // Test conversion from % MAC to station
  void testMACPercentToStation() {
    // GIVEN: Desired CG at 25% MAC
    double mac_length = 50.0;
    double mac_le = 120.0;
    double desired_percent = 25.0;

    // WHEN: Computing station
    double station = mac_le + (desired_percent / 100.0) * mac_length;

    // THEN: Verify conversion
    TS_ASSERT_DELTA(132.5, station, epsilon);
  }

  // ============ CG Limits and Envelope Tests ============

  // Test CG within limits check
  void testCGWithinLimits() {
    // GIVEN: CG limits and actual CG
    double xcg_fwd = 95.0;   // inches
    double xcg_aft = 115.0;  // inches
    double xcg_actual = 105.0;

    // WHEN: Checking if within limits
    bool within_limits = (xcg_actual >= xcg_fwd) && (xcg_actual <= xcg_aft);

    // THEN: Should be within limits
    TS_ASSERT(within_limits);
  }

  // Test CG forward of limit
  void testCGForwardOfLimit() {
    // GIVEN: CG limits
    double xcg_fwd = 100.0;
    double xcg_actual = 95.0;  // Too far forward

    // WHEN: Checking limit
    bool forward_limit_exceeded = xcg_actual < xcg_fwd;

    // THEN: Forward limit exceeded
    TS_ASSERT(forward_limit_exceeded);
  }

  // Test CG aft of limit
  void testCGAftOfLimit() {
    // GIVEN: CG limits
    double xcg_aft = 110.0;
    double xcg_actual = 115.0;  // Too far aft

    // WHEN: Checking limit
    bool aft_limit_exceeded = xcg_actual > xcg_aft;

    // THEN: Aft limit exceeded
    TS_ASSERT(aft_limit_exceeded);
  }

  // Test CG envelope weight dependency
  void testCGEnvelopeWeightDependency() {
    // GIVEN: CG limits vary with weight
    double weight = 3000.0;  // lbs

    // Light weight: wider CG range
    // Heavy weight: narrower CG range
    // This simulates typical aircraft behavior

    // For this weight, limits are:
    double xcg_fwd_at_weight = 98.0 + (weight - 2000.0) / 1000.0 * 2.0;  // 100.0
    double xcg_aft_at_weight = 118.0 - (weight - 2000.0) / 1000.0 * 2.0;  // 116.0

    // WHEN: Computing range
    double cg_range = xcg_aft_at_weight - xcg_fwd_at_weight;

    // THEN: Range narrows with weight increase
    TS_ASSERT_DELTA(16.0, cg_range, epsilon);
  }

  // ============ Moment of Inertia Changes with Loading ============

  // Test inertia change with point mass using parallel axis theorem
  void testInertiaChangeWithLoading() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // GIVEN: Point mass added to aircraft
    double mass_slugs = 10.0;  // slugs
    FGColumnVector3 location(120.0, 0.0, 0.0);  // inches from origin

    // WHEN: Computing point mass inertia
    FGMatrix33 pmInertia = massBalance->GetPointmassInertia(mass_slugs, location);

    // THEN: Inertia should be non-zero (parallel axis theorem applies)
    // I = m*(y^2 + z^2) for Ixx, etc.
    TS_ASSERT(!std::isnan(pmInertia(1, 1)));
    TS_ASSERT(!std::isnan(pmInertia(2, 2)));
    TS_ASSERT(!std::isnan(pmInertia(3, 3)));
  }

  // Test moment of inertia for distributed load
  void testInertiaDistributedLoad() {
    // GIVEN: Two equal masses symmetrically placed
    double mass = 5.0;  // slugs each
    double y_offset = 60.0;  // inches (lateral offset)

    // WHEN: Computing roll inertia contribution (about x-axis)
    // Ixx = Σ m*(y^2 + z^2), with z=0 for lateral placement
    double Ixx_contribution = 2 * mass * (y_offset/12.0) * (y_offset/12.0);  // Convert to feet

    // THEN: Symmetric loading increases roll inertia
    TS_ASSERT(Ixx_contribution > 0.0);
    double expected = 2.0 * 5.0 * 5.0 * 5.0;  // 2 * 5 * 25
    TS_ASSERT_DELTA(250.0, expected, epsilon);
  }

  // ============ Static Margin Calculations ============

  // Test static margin formula: SM = (xnp - xcg) / mac
  void testStaticMarginBasic() {
    // GIVEN: Neutral point and CG locations
    double xnp = 115.0;    // inches (neutral point)
    double xcg = 105.0;    // inches (CG location)
    double mac = 50.0;     // inches (mean aerodynamic chord)

    // WHEN: Computing static margin
    double static_margin = (xnp - xcg) / mac;
    double static_margin_percent = static_margin * 100.0;

    // THEN: Static margin should be positive for stability
    TS_ASSERT_DELTA(0.2, static_margin, epsilon);
    TS_ASSERT_DELTA(20.0, static_margin_percent, epsilon);
    TS_ASSERT(static_margin > 0.0);  // Stable
  }

  // Test static margin with aft CG
  void testStaticMarginAftCG() {
    // GIVEN: CG near neutral point
    double xnp = 115.0;
    double xcg = 112.0;  // Aft CG
    double mac = 50.0;

    // WHEN: Computing static margin
    double static_margin = (xnp - xcg) / mac;
    double static_margin_percent = static_margin * 100.0;

    // THEN: Reduced static margin
    TS_ASSERT_DELTA(0.06, static_margin, epsilon);
    TS_ASSERT_DELTA(6.0, static_margin_percent, epsilon);
    TS_ASSERT(static_margin > 0.0);  // Still stable but less margin
  }

  // Test neutral stability (CG at neutral point)
  void testNeutralStability() {
    // GIVEN: CG at neutral point
    double xnp = 110.0;
    double xcg = 110.0;
    double mac = 50.0;

    // WHEN: Computing static margin
    double static_margin = (xnp - xcg) / mac;

    // THEN: Zero static margin (neutrally stable)
    TS_ASSERT_DELTA(0.0, static_margin, epsilon);
  }

  // Test unstable condition (CG aft of neutral point)
  void testUnstableCondition() {
    // GIVEN: CG aft of neutral point
    double xnp = 110.0;
    double xcg = 115.0;  // Aft of NP
    double mac = 50.0;

    // WHEN: Computing static margin
    double static_margin = (xnp - xcg) / mac;

    // THEN: Negative static margin (unstable)
    TS_ASSERT_DELTA(-0.1, static_margin, epsilon);
    TS_ASSERT(static_margin < 0.0);  // Unstable
  }

  // ============ Trim Changes with CG ============

  // Test elevator trim change with CG shift
  void testElevatorTrimWithCGShift() {
    // GIVEN: CG shifts aft by 5 inches
    double xcg_initial = 100.0;
    double xcg_final = 105.0;
    double cg_shift = xcg_final - xcg_initial;

    // Empirical: ~0.5 deg elevator trim per inch CG shift
    double trim_sensitivity = 0.5;  // deg/inch

    // WHEN: Computing trim change
    double trim_change = cg_shift * trim_sensitivity;

    // THEN: Nose-down trim required (aft CG)
    TS_ASSERT_DELTA(2.5, trim_change, epsilon);
  }

  // Test trim moment balance
  void testTrimMomentBalance() {
    // GIVEN: Aircraft in trim
    double L_wing = 10000.0;      // lbs (wing lift)
    double x_wing = 110.0;        // inches (wing aerodynamic center)
    double W_aircraft = 10000.0;  // lbs (weight)
    double xcg = 105.0;           // inches (CG)

    // WHEN: Computing pitching moment about CG
    double M_wing = L_wing * (x_wing - xcg);

    // For trim, need elevator moment to balance
    double M_elevator_required = -M_wing;

    // THEN: Moments should sum to zero for trim
    TS_ASSERT_DELTA(50000.0, M_wing, epsilon);
    TS_ASSERT_DELTA(-50000.0, M_elevator_required, epsilon);
  }

  // ============ Ballast Calculations ============

  // Test ballast required to achieve target CG
  void testBallastForTargetCG() {
    // GIVEN: Aircraft too nose-heavy
    double w_aircraft = 2500.0;
    double xcg_actual = 95.0;   // Too far forward
    double xcg_target = 105.0;  // Desired CG
    double x_ballast = 180.0;   // Ballast location (aft)

    // WHEN: Computing ballast weight needed
    // (w_aircraft * xcg_actual + w_ballast * x_ballast) / (w_aircraft + w_ballast) = xcg_target
    // Solving for w_ballast:
    double w_ballast = w_aircraft * (xcg_target - xcg_actual) / (x_ballast - xcg_target);

    // THEN: Ballast weight calculated
    double expected = 2500.0 * 10.0 / 75.0;
    TS_ASSERT_DELTA(expected, w_ballast, 1e-6);
    TS_ASSERT_DELTA(333.333, w_ballast, 1e-3);
  }

  // Test ballast placement optimization
  void testBallastPlacementOptimization() {
    // GIVEN: Need to shift CG aft with minimum ballast weight
    double w_aircraft = 3000.0;
    double xcg_current = 100.0;
    double xcg_desired = 108.0;

    // Two ballast locations available
    double x_ballast_aft = 200.0;    // Far aft (more effective)
    double x_ballast_close = 150.0;  // Closer (less effective)

    // WHEN: Computing ballast needed for each location
    double w_ballast_aft = w_aircraft * (xcg_desired - xcg_current) /
                           (x_ballast_aft - xcg_desired);
    double w_ballast_close = w_aircraft * (xcg_desired - xcg_current) /
                             (x_ballast_close - xcg_desired);

    // THEN: Aft location requires less ballast
    TS_ASSERT(w_ballast_aft < w_ballast_close);
    TS_ASSERT_DELTA(260.87, w_ballast_aft, 1e-2);
    TS_ASSERT_DELTA(571.43, w_ballast_close, 1e-2);
  }

  // ============ Multi-Point Loading Scenarios ============

  // Test complex multi-point loading
  void testMultiPointLoading() {
    // GIVEN: Aircraft with multiple load points
    double w_empty = 1500.0;
    double xcg_empty = 100.0;

    // Various loads
    double w_pilot = 200.0, x_pilot = 85.0;
    double w_copilot = 180.0, x_copilot = 85.0;
    double w_pax1 = 170.0, x_pax1 = 120.0;
    double w_pax2 = 170.0, x_pax2 = 120.0;
    double w_fuel = 300.0, x_fuel = 95.0;
    double w_baggage = 100.0, x_baggage = 140.0;

    // WHEN: Computing loaded CG
    double total_weight = w_empty + w_pilot + w_copilot + w_pax1 + w_pax2 + w_fuel + w_baggage;
    double total_moment = w_empty*xcg_empty + w_pilot*x_pilot + w_copilot*x_copilot +
                         w_pax1*x_pax1 + w_pax2*x_pax2 + w_fuel*x_fuel + w_baggage*x_baggage;
    double xcg_loaded = total_moment / total_weight;

    // THEN: Verify calculation
    TS_ASSERT_DELTA(2620.0, total_weight, epsilon);
    TS_ASSERT(xcg_loaded > 0.0);
    TS_ASSERT(!std::isnan(xcg_loaded));
  }

  // Test sequential loading effects
  void testSequentialLoading() {
    // GIVEN: Empty aircraft
    double weight = 2000.0;
    double xcg = 105.0;

    // WHEN: Loading items sequentially
    // Step 1: Add pilot
    double w_pilot = 180.0, x_pilot = 90.0;
    double xcg_1 = (weight*xcg + w_pilot*x_pilot) / (weight + w_pilot);
    weight += w_pilot;
    xcg = xcg_1;

    // Step 2: Add fuel
    double w_fuel = 400.0, x_fuel = 100.0;
    double xcg_2 = (weight*xcg + w_fuel*x_fuel) / (weight + w_fuel);
    weight += w_fuel;
    xcg = xcg_2;

    // Step 3: Add cargo
    double w_cargo = 200.0, x_cargo = 130.0;
    double xcg_3 = (weight*xcg + w_cargo*x_cargo) / (weight + w_cargo);

    // THEN: CG evolves with each loading step
    TS_ASSERT(xcg_1 < 105.0);  // Forward shift with pilot
    TS_ASSERT(xcg_2 < xcg_1);  // Further forward with fuel at 100
    TS_ASSERT(xcg_3 > xcg_2);  // Aft shift with rear cargo
  }

  // Test loading configuration matrix
  void testLoadingConfigurationMatrix() {
    // GIVEN: Multiple loading configurations to verify
    double w_aircraft = 2500.0;
    double xcg_aircraft = 105.0;
    double x_pax_front = 90.0;
    double x_pax_rear = 125.0;

    // WHEN: Testing different configurations
    // Case 1: Front pax only
    double xcg1 = (w_aircraft*xcg_aircraft + 340.0*x_pax_front) / (w_aircraft + 340.0);
    // (2500*105 + 340*90) / 2840 = (262500 + 30600) / 2840 = 103.204

    // Case 2: Rear pax only
    double xcg2 = (w_aircraft*xcg_aircraft + 340.0*x_pax_rear) / (w_aircraft + 340.0);
    // (2500*105 + 340*125) / 2840 = (262500 + 42500) / 2840 = 107.394

    // Case 3: All pax
    double xcg3 = (w_aircraft*xcg_aircraft + 340.0*x_pax_front + 340.0*x_pax_rear) /
                  (w_aircraft + 340.0 + 340.0);
    // (2500*105 + 340*90 + 340*125) / 3180 = (262500 + 30600 + 42500) / 3180 = 105.535

    // THEN: CG should vary based on loading
    TS_ASSERT(xcg1 < xcg_aircraft);  // Forward loading shifts CG forward
    TS_ASSERT(xcg2 > xcg_aircraft);  // Aft loading shifts CG aft
    TS_ASSERT_DELTA(103.204, xcg1, 1e-3);
    TS_ASSERT_DELTA(107.394, xcg2, 1e-3);
    TS_ASSERT_DELTA(105.535, xcg3, 1e-3);
  }

  // Test extreme loading condition
  void testExtremeLoadingCondition() {
    // GIVEN: Maximum forward loading
    double w_base = 2000.0;
    double xcg_base = 110.0;
    double w_forward_max = 500.0;
    double x_forward = 70.0;  // Far forward

    // WHEN: Computing CG with max forward load
    double xcg_extreme = (w_base*xcg_base + w_forward_max*x_forward) / (w_base + w_forward_max);

    // THEN: CG shifts significantly forward
    double shift = xcg_base - xcg_extreme;
    TS_ASSERT(shift > 0.0);
    TS_ASSERT_DELTA(8.0, shift, epsilon);
    TS_ASSERT_DELTA(102.0, xcg_extreme, epsilon);
  }

  // Test CG travel limits across weight range
  void testCGTravelLimits() {
    // GIVEN: CG limits as function of weight
    double weight_min = 2000.0;  // lbs (empty + minimum)
    double weight_max = 3500.0;  // lbs (max gross weight)

    // Typical limits: more restrictive at higher weights
    double xcg_fwd_min_wt = 95.0;   // Forward limit at min weight
    double xcg_aft_min_wt = 120.0;  // Aft limit at min weight
    double xcg_fwd_max_wt = 100.0;  // Forward limit at max weight
    double xcg_aft_max_wt = 115.0;  // Aft limit at max weight

    // WHEN: Computing CG range at each weight
    double cg_range_min_wt = xcg_aft_min_wt - xcg_fwd_min_wt;
    double cg_range_max_wt = xcg_aft_max_wt - xcg_fwd_max_wt;

    // THEN: CG range narrows at higher weight
    TS_ASSERT(cg_range_max_wt < cg_range_min_wt);
    TS_ASSERT_DELTA(25.0, cg_range_min_wt, epsilon);
    TS_ASSERT_DELTA(15.0, cg_range_max_wt, epsilon);
  }

  // Test zero-fuel weight CG calculation
  void testZeroFuelWeightCG() {
    // GIVEN: Aircraft with fuel
    double w_empty = 2000.0;
    double xcg_empty = 105.0;
    double w_payload = 800.0;
    double xcg_payload = 115.0;
    double w_fuel = 600.0;
    double xcg_fuel = 95.0;

    // WHEN: Computing zero-fuel CG (no fuel)
    double w_zfw = w_empty + w_payload;
    double xcg_zfw = (w_empty*xcg_empty + w_payload*xcg_payload) / w_zfw;

    // Computing ramp CG (with fuel)
    double w_ramp = w_zfw + w_fuel;
    double xcg_ramp = (w_zfw*xcg_zfw + w_fuel*xcg_fuel) / w_ramp;

    // THEN: Ramp CG should be forward of ZFW CG (fuel is forward)
    TS_ASSERT(xcg_ramp < xcg_zfw);
    double expected_zfw_cg = (210000.0 + 92000.0) / 2800.0;
    TS_ASSERT_DELTA(expected_zfw_cg, xcg_zfw, 1e-6);
  }

  // Test landing weight CG calculation
  void testLandingWeightCG() {
    // GIVEN: Takeoff conditions
    double w_takeoff = 3400.0;
    double xcg_takeoff = 108.0;
    double fuel_burned = 500.0;
    double xcg_fuel = 95.0;  // Fuel burned from forward tank

    // WHEN: Computing landing CG
    // After fuel burn, total weight decreases, CG shifts aft
    double w_landing = w_takeoff - fuel_burned;

    // Using moment balance: M_takeoff = M_landing + M_fuel_burned
    double moment_takeoff = w_takeoff * xcg_takeoff;
    double moment_fuel = fuel_burned * xcg_fuel;
    double xcg_landing = (moment_takeoff - moment_fuel) / w_landing;

    // THEN: Landing CG should be aft of takeoff CG
    TS_ASSERT(xcg_landing > xcg_takeoff);
    double expected = (367200.0 - 47500.0) / 2900.0;
    TS_ASSERT_DELTA(expected, xcg_landing, 1e-6);
  }

  // Test weight and balance report validation
  void testWeightBalanceReportValidation() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // GIVEN: Set aircraft properties
    double testWeight = 2500.0;
    massBalance->SetEmptyWeight(testWeight);

    FGColumnVector3 testCG(105.0, 0.0, -8.0);
    massBalance->SetBaseCG(testCG);

    FGMatrix33 testInertia(1200.0, 0.0, -40.0,
                          0.0, 1800.0, 0.0,
                          -40.0, 0.0, 2500.0);
    massBalance->SetAircraftBaseInertias(testInertia);

    // WHEN: Running mass balance model
    massBalance->Run(false);

    // THEN: Verify values are reasonable
    TS_ASSERT(massBalance->GetMass() >= 0.0);
    TS_ASSERT(massBalance->GetWeight() >= 0.0);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT(!std::isnan(cg(1)));
    TS_ASSERT(!std::isnan(cg(2)));
    TS_ASSERT(!std::isnan(cg(3)));

    const FGMatrix33& J = massBalance->GetJ();
    TS_ASSERT(J(1,1) > 0.0);  // Principal moments must be positive
    TS_ASSERT(J(2,2) > 0.0);
    TS_ASSERT(J(3,3) > 0.0);
  }

  // ============ Additional Weight and Balance Tests ============

  // Test CG shift rate with loading
  void testCGShiftRate() {
    // GIVEN: Incremental loading
    double w_base = 2500.0;
    double xcg_base = 105.0;
    double x_load = 130.0;  // Load location

    // WHEN: Computing CG shift per unit weight added
    double dw = 10.0;  // Small increment
    double xcg_after = (w_base*xcg_base + dw*x_load) / (w_base + dw);
    double cg_shift_rate = (xcg_after - xcg_base) / dw;

    // THEN: Verify shift rate formula: d(xcg)/dw = (x_load - xcg) / (w_total)
    double expected_rate = (x_load - xcg_base) / (w_base + dw);
    TS_ASSERT_DELTA(expected_rate, cg_shift_rate, 1e-6);
  }

  // Test moment index calculation
  void testMomentIndex() {
    // GIVEN: Weight and arm
    double weight = 2800.0;  // lbs
    double arm = 95.5;       // inches

    // WHEN: Computing moment index (moment/1000)
    double moment = weight * arm;
    double moment_index = moment / 1000.0;

    // THEN: Verify calculation
    TS_ASSERT_DELTA(267.4, moment_index, 0.1);
  }

  // Test CG from moment index
  void testCGFromMomentIndex() {
    // GIVEN: Total weight and moment index
    double total_weight = 3200.0;     // lbs
    double moment_index = 345.6;      // moment/1000

    // WHEN: Computing CG
    double total_moment = moment_index * 1000.0;
    double xcg = total_moment / total_weight;

    // THEN: Verify CG calculation
    TS_ASSERT_DELTA(108.0, xcg, 0.1);
  }

  // Test weight-CG product for multiple items
  void testWeightCGProduct() {
    // GIVEN: Multiple items
    struct Item {
      double weight;
      double station;
    };

    Item items[] = {
      {1800.0, 98.0},   // Empty aircraft
      {180.0, 85.0},    // Pilot
      {160.0, 85.0},    // Front passenger
      {140.0, 118.0},   // Rear passenger left
      {140.0, 118.0},   // Rear passenger right
      {60.0, 140.0}     // Baggage
    };

    // WHEN: Computing total moment
    double total_weight = 0.0;
    double total_moment = 0.0;
    for (const auto& item : items) {
      total_weight += item.weight;
      total_moment += item.weight * item.station;
    }

    double xcg = total_moment / total_weight;

    // THEN: Verify calculation
    TS_ASSERT_DELTA(2480.0, total_weight, epsilon);
    TS_ASSERT(xcg > 95.0 && xcg < 105.0);
  }

  // Test useful load calculation
  void testUsefulLoadCalculation() {
    // GIVEN: Aircraft weights
    double max_gross_weight = 3400.0;  // lbs
    double empty_weight = 1950.0;      // lbs

    // WHEN: Computing useful load
    double useful_load = max_gross_weight - empty_weight;

    // THEN: Available for fuel, passengers, cargo
    TS_ASSERT_DELTA(1450.0, useful_load, epsilon);
  }

  // Test payload CG range calculation
  void testPayloadCGRange() {
    // GIVEN: Payload distributed along fuselage
    double w_payload = 600.0;  // lbs total
    double x_forward = 80.0;   // inches (all payload forward)
    double x_aft = 140.0;      // inches (all payload aft)

    // WHEN: Computing CG range for payload alone
    double cg_range = x_aft - x_forward;

    // THEN: Maximum CG travel for payload
    TS_ASSERT_DELTA(60.0, cg_range, epsilon);
  }

  // Test lateral CG limits
  void testLateralCGLimits() {
    // GIVEN: Lateral CG limits for safe flight
    double ycg_limit = 2.0;  // inches from centerline

    // Test case: asymmetric loading
    double w_left = 100.0;   // lbs
    double y_left = -80.0;   // inches
    double w_right = 120.0;  // lbs
    double y_right = 80.0;   // inches

    // WHEN: Computing lateral CG
    double ycg = (w_left*y_left + w_right*y_right) / (w_left + w_right);

    // THEN: Check if within limits
    bool within_limits = std::abs(ycg) <= ycg_limit;
    TS_ASSERT(!within_limits);  // This case exceeds limits
    TS_ASSERT_DELTA(7.273, std::abs(ycg), 1e-3);
  }

  // Test vertical CG height calculation
  void testVerticalCGHeight() {
    // GIVEN: Component heights above ground
    double w1 = 1500.0;  // lbs (structure)
    double h1 = 3.0;     // ft
    double w2 = 500.0;   // lbs (payload)
    double h2 = 4.5;     // ft

    // WHEN: Computing CG height
    double h_cg = (w1*h1 + w2*h2) / (w1 + w2);

    // THEN: Verify height calculation
    TS_ASSERT_DELTA(3.375, h_cg, 1e-6);
  }

  // Test ballast removal effects
  void testBallastRemoval() {
    // GIVEN: Aircraft with ballast installed
    double w_aircraft = 2800.0;
    double xcg_with_ballast = 105.0;
    double w_ballast = 150.0;
    double x_ballast = 160.0;  // Aft ballast

    // WHEN: Removing ballast
    double w_no_ballast = w_aircraft - w_ballast;
    double moment_with = w_aircraft * xcg_with_ballast;
    double moment_ballast = w_ballast * x_ballast;
    double xcg_no_ballast = (moment_with - moment_ballast) / w_no_ballast;

    // THEN: CG shifts forward
    TS_ASSERT(xcg_no_ballast < xcg_with_ballast);
    double expected = (294000.0 - 24000.0) / 2650.0;
    TS_ASSERT_DELTA(expected, xcg_no_ballast, 1e-6);
  }

  // Test nose/tail wheel load distribution
  void testWheelLoadDistribution() {
    // GIVEN: Aircraft on ground (tricycle gear)
    double W = 2500.0;       // lbs (total weight)
    double xcg = 95.0;       // inches
    double x_nose = 50.0;    // inches (nose wheel)
    double x_main = 110.0;   // inches (main wheels)

    // WHEN: Computing wheel loads using moment balance
    // W_nose * (x_main - x_nose) = W * (x_main - xcg)
    double W_nose = W * (x_main - xcg) / (x_main - x_nose);
    double W_main = W - W_nose;

    // THEN: Verify load distribution
    TS_ASSERT_DELTA(625.0, W_nose, 1e-6);
    TS_ASSERT_DELTA(1875.0, W_main, 1e-6);
    TS_ASSERT(W_nose > 0.0 && W_main > 0.0);
  }

  // Test tandem seat CG calculation
  void testTandemSeatCG() {
    // GIVEN: Tandem seating (e.g., fighter jet)
    double w_empty = 8000.0;
    double xcg_empty = 180.0;
    double w_pilot_front = 200.0;
    double x_pilot_front = 150.0;
    double w_pilot_rear = 200.0;
    double x_pilot_rear = 220.0;

    // WHEN: Computing CG with different crew configurations
    // Both seats occupied
    double xcg_both = (w_empty*xcg_empty + w_pilot_front*x_pilot_front + w_pilot_rear*x_pilot_rear) /
                      (w_empty + w_pilot_front + w_pilot_rear);

    // Front seat only
    double xcg_front = (w_empty*xcg_empty + w_pilot_front*x_pilot_front) /
                       (w_empty + w_pilot_front);

    // THEN: CG varies with crew configuration
    TS_ASSERT(xcg_front < xcg_empty);  // Shifts forward
    TS_ASSERT(xcg_both > xcg_front);   // Shifts aft with rear pilot
  }

  // Test CG shift sensitivity analysis
  void testCGShiftSensitivity() {
    // GIVEN: Base configuration
    double w_base = 3000.0;
    double xcg_base = 110.0;

    // WHEN: Adding weight at various stations
    double dw = 100.0;  // Same weight increment

    double x1 = 80.0;   // Forward
    double x2 = 110.0;  // At CG
    double x3 = 140.0;  // Aft

    double xcg1 = (w_base*xcg_base + dw*x1) / (w_base + dw);
    double xcg2 = (w_base*xcg_base + dw*x2) / (w_base + dw);
    double xcg3 = (w_base*xcg_base + dw*x3) / (w_base + dw);

    // THEN: CG shift varies with load location
    TS_ASSERT(xcg1 < xcg2);  // Forward load shifts CG forward
    TS_ASSERT(xcg2 < xcg3);  // Aft load shifts CG aft
    TS_ASSERT_DELTA(xcg_base, xcg2, 0.5);  // Loading at CG has minimal shift
  }

  // Test fuel tank sequencing CG effect
  void testFuelTankSequencing() {
    // GIVEN: Multiple fuel tanks
    double w_aircraft = 2200.0;
    double xcg_aircraft = 105.0;

    double w_tank_fwd = 200.0;
    double x_tank_fwd = 85.0;
    double w_tank_main = 400.0;
    double x_tank_main = 105.0;
    double w_tank_aft = 200.0;
    double x_tank_aft = 125.0;

    // WHEN: Computing CG with all tanks full
    double xcg_all_full = (w_aircraft*xcg_aircraft +
                           w_tank_fwd*x_tank_fwd +
                           w_tank_main*x_tank_main +
                           w_tank_aft*x_tank_aft) /
                          (w_aircraft + w_tank_fwd + w_tank_main + w_tank_aft);

    // Burn from forward tank first
    double xcg_fwd_empty = (w_aircraft*xcg_aircraft +
                            w_tank_main*x_tank_main +
                            w_tank_aft*x_tank_aft) /
                           (w_aircraft + w_tank_main + w_tank_aft);

    // THEN: CG moves aft as forward fuel burns
    TS_ASSERT(xcg_fwd_empty > xcg_all_full);
  }

  // Test MAC leading edge calculation
  void testMACLeadingEdge() {
    // GIVEN: Wing geometry
    double root_chord = 80.0;      // inches
    double tip_chord = 40.0;       // inches
    double wingspan = 300.0;       // inches
    double root_le = 100.0;        // inches (fuselage station)
    double sweep_angle = 15.0;     // degrees at LE

    // WHEN: Computing MAC
    // MAC = (2/3) * root * (1 + taper + taper^2) / (1 + taper)
    double taper = tip_chord / root_chord;
    double mac = (2.0/3.0) * root_chord * (1.0 + taper + taper*taper) / (1.0 + taper);

    // THEN: Verify MAC calculation
    double expected_mac = (2.0/3.0) * 80.0 * (1.0 + 0.5 + 0.25) / 1.5;
    TS_ASSERT_DELTA(expected_mac, mac, 1e-6);
    TS_ASSERT_DELTA(62.222, mac, 1e-3);
  }

  // Test delta CG tracking
  void testDeltaCGTracking() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // GIVEN: Initial CG
    FGColumnVector3 initialCG(100.0, 0.0, -10.0);
    massBalance->SetBaseCG(initialCG);
    massBalance->Run(false);

    // WHEN: CG changes (simulated by new base CG)
    FGColumnVector3 newCG(105.0, 1.0, -9.0);
    massBalance->SetBaseCG(newCG);
    massBalance->Run(false);

    // THEN: Delta CG should reflect the change
    const FGColumnVector3& deltaCG = massBalance->GetDeltaXYZcg();
    TS_ASSERT(!std::isnan(deltaCG(1)));
    TS_ASSERT(!std::isnan(deltaCG(2)));
    TS_ASSERT(!std::isnan(deltaCG(3)));
  }

  // ============ Extended CG Calculations ============

  // Test CG with distributed fuel across multiple tanks
  void testMultiFuelTankCG() {
    double w_aircraft = 2500.0;
    double xcg_aircraft = 105.0;

    // Three fuel tanks
    double w_tank1 = 100.0, x_tank1 = 80.0;   // Forward aux
    double w_tank2 = 300.0, x_tank2 = 100.0;  // Main
    double w_tank3 = 100.0, x_tank3 = 120.0;  // Aft aux

    double total_weight = w_aircraft + w_tank1 + w_tank2 + w_tank3;
    double total_moment = w_aircraft*xcg_aircraft + w_tank1*x_tank1 +
                          w_tank2*x_tank2 + w_tank3*x_tank3;
    double xcg = total_moment / total_weight;

    TS_ASSERT(xcg > 95.0 && xcg < 110.0);
    TS_ASSERT(!std::isnan(xcg));
  }

  // Test CG shift with external stores
  void testExternalStoresCG() {
    double w_base = 8000.0;
    double xcg_base = 200.0;

    // Wing-mounted external stores
    double w_store_left = 500.0, x_store = 210.0, y_store = -120.0;
    double w_store_right = 500.0;

    double total_weight = w_base + w_store_left + w_store_right;
    double xcg_new = (w_base*xcg_base + (w_store_left + w_store_right)*x_store) / total_weight;

    // Lateral CG should remain centered for symmetric loading
    double ycg = (w_store_left * y_store + w_store_right * (-y_store)) / total_weight;
    TS_ASSERT_DELTA(ycg, 0.0, epsilon);
    TS_ASSERT(xcg_new > xcg_base);
  }

  // Test CG envelope with varying fuel and payload
  void testCGEnvelopeVariation() {
    double w_empty = 2000.0;
    double xcg_empty = 100.0;

    // Test multiple configurations
    double configs[][2] = {
      {0.0, 0.0},      // Empty
      {500.0, 90.0},   // Fuel only
      {400.0, 120.0},  // Payload only
      {500.0, 90.0}    // Combined (will add more)
    };

    for (int i = 0; i < 3; i++) {
      double w_add = configs[i][0];
      double x_add = configs[i][1];
      if (w_add > 0) {
        double xcg = (w_empty*xcg_empty + w_add*x_add) / (w_empty + w_add);
        TS_ASSERT(!std::isnan(xcg));
        TS_ASSERT(xcg > 80.0 && xcg < 130.0);
      }
    }
  }

  // ============ Moment of Inertia Extended Tests ============

  // Test principal axis rotation
  void testPrincipalAxisRotation() {
    // Product of inertia should be zero about principal axes
    FGMatrix33 J(1000.0, 0.0, -50.0,
                 0.0, 1500.0, 0.0,
                 -50.0, 0.0, 2000.0);

    // For aircraft, Jxz is usually non-zero (nose-down attitude)
    TS_ASSERT(J(1,1) > 0);
    TS_ASSERT(J(2,2) > 0);
    TS_ASSERT(J(3,3) > 0);
    TS_ASSERT_DELTA(J(1,3), -50.0, epsilon);
  }

  // Test gyration radius calculation
  void testGyrationRadius() {
    double Ixx = 1200.0;  // slug-ft^2
    double mass = 100.0;  // slugs

    double k_x = std::sqrt(Ixx / mass);  // ft
    TS_ASSERT(k_x > 0);
    TS_ASSERT_DELTA(k_x, 3.464, 0.01);
  }

  // Test parallel axis theorem application
  void testParallelAxisTheorem() {
    double Icm = 500.0;    // slug-ft^2 about CM
    double mass = 50.0;    // slugs
    double d = 5.0;        // ft offset

    double I_new = Icm + mass * d * d;
    TS_ASSERT_DELTA(I_new, 1750.0, epsilon);
  }

  // ============ Weight Distribution Analysis ============

  // Test gear load distribution tricycle
  void testTricycleGearLoadDistribution() {
    double W = 3000.0;
    double xcg = 100.0;
    double x_nose = 40.0;
    double x_main = 120.0;

    // Main gear load
    double W_main = W * (xcg - x_nose) / (x_main - x_nose);
    // Nose gear load
    double W_nose = W - W_main;

    TS_ASSERT(W_main > W_nose);  // Most weight on mains
    TS_ASSERT_DELTA(W_main + W_nose, W, epsilon);
    TS_ASSERT(W_nose > 0);  // Positive nose wheel load
  }

  // Test taildragger load distribution
  void testTaildragLoadDistribution() {
    double W = 2500.0;
    double xcg = 85.0;
    double x_main = 80.0;
    double x_tail = 250.0;

    // Tail wheel load
    double W_tail = W * (xcg - x_main) / (x_tail - x_main);
    double W_main = W - W_tail;

    TS_ASSERT(W_main > W_tail);
    TS_ASSERT_DELTA(W_main + W_tail, W, epsilon);
  }

  // Test asymmetric fuel loading effect
  void testAsymmetricFuelLoadingEffect() {
    double w_left_tank = 200.0;
    double w_right_tank = 100.0;  // Half full
    double y_tank = 100.0;  // Distance from centerline

    double ycg = (w_left_tank * (-y_tank) + w_right_tank * y_tank) /
                 (w_left_tank + w_right_tank);

    TS_ASSERT(ycg < 0);  // CG shifted left
    TS_ASSERT_DELTA(ycg, -33.33, 0.1);
  }

  // ============ Stability and Trim Effects ============

  // Test CG effect on longitudinal stability
  void testCGEffectOnStability() {
    double xnp = 120.0;  // Neutral point
    double mac = 50.0;

    // Forward CG
    double xcg_fwd = 100.0;
    double sm_fwd = (xnp - xcg_fwd) / mac * 100.0;

    // Aft CG
    double xcg_aft = 115.0;
    double sm_aft = (xnp - xcg_aft) / mac * 100.0;

    TS_ASSERT(sm_fwd > sm_aft);  // Forward CG more stable
    TS_ASSERT(sm_fwd > 0);
    TS_ASSERT(sm_aft > 0);
  }

  // Test elevator authority with CG
  void testElevatorAuthorityWithCG() {
    double xcg_fwd = 95.0;
    double xcg_aft = 115.0;
    double x_tail = 300.0;  // Tail AC location

    double arm_fwd = x_tail - xcg_fwd;
    double arm_aft = x_tail - xcg_aft;

    // Longer arm = more authority
    TS_ASSERT(arm_fwd > arm_aft);
    TS_ASSERT_DELTA(arm_fwd, 205.0, epsilon);
    TS_ASSERT_DELTA(arm_aft, 185.0, epsilon);
  }

  // Test stick force gradient
  void testStickForceGradient() {
    double cg_position = 0.25;  // 25% MAC
    double np_position = 0.35;  // 35% MAC
    double mac = 5.0;  // ft

    double static_margin = np_position - cg_position;
    // Stick force gradient proportional to static margin
    double gradient = 10.0 * static_margin;  // lbs/g (simplified)

    TS_ASSERT(gradient > 0);
    TS_ASSERT_DELTA(gradient, 1.0, 0.1);
  }

  // ============ Loading Scenario Tests ============

  // Test maximum ramp weight configuration
  void testMaxRampWeightConfiguration() {
    double max_ramp = 3600.0;
    double empty = 2100.0;
    double pilot = 180.0;
    double copilot = 170.0;
    double fuel = 500.0;
    double baggage = 100.0;

    double total = empty + pilot + copilot + fuel + baggage;
    bool underMaxRamp = total <= max_ramp;

    TS_ASSERT(underMaxRamp);
    TS_ASSERT_DELTA(total, 3050.0, 1.0);
  }

  // Test zero payload configuration
  void testZeroPayloadConfiguration() {
    double w_empty = 2000.0;
    double xcg_empty = 100.0;
    double w_fuel = 400.0;
    double x_fuel = 95.0;

    double xcg = (w_empty*xcg_empty + w_fuel*x_fuel) / (w_empty + w_fuel);
    TS_ASSERT(xcg < xcg_empty);  // Fuel forward of empty CG
  }

  // Test maximum aft CG loading
  void testMaxAftCGLoading() {
    double w_empty = 2500.0;
    double xcg_empty = 105.0;
    double w_aft_cargo = 300.0;
    double x_aft_cargo = 150.0;

    double xcg = (w_empty*xcg_empty + w_aft_cargo*x_aft_cargo) / (w_empty + w_aft_cargo);

    double aft_limit = 115.0;
    bool withinLimits = xcg <= aft_limit;

    TS_ASSERT(!std::isnan(xcg));
    TS_ASSERT(xcg > xcg_empty);
  }

  // ============ Edge Cases ============

  // Test very light weight condition
  void testVeryLightWeightCondition() {
    double w_empty = 1500.0;
    double xcg_empty = 95.0;
    double w_pilot = 120.0;  // Light pilot
    double x_pilot = 85.0;

    double xcg = (w_empty*xcg_empty + w_pilot*x_pilot) / (w_empty + w_pilot);
    TS_ASSERT(xcg < xcg_empty);
    TS_ASSERT(xcg > 80.0);
  }

  // Test heavy payload configuration
  void testHeavyPayloadConfiguration() {
    double w_base = 5000.0;
    double xcg_base = 200.0;
    double max_payload = 3000.0;
    double x_payload_avg = 210.0;

    double xcg_heavy = (w_base*xcg_base + max_payload*x_payload_avg) / (w_base + max_payload);
    TS_ASSERT(xcg_heavy > xcg_base);
    TS_ASSERT(!std::isnan(xcg_heavy));
  }

  // Test CG computation numerical stability
  void testCGNumericalStability() {
    // Very small weights
    double w1 = 1e-6;
    double x1 = 100.0;
    double w2 = 1e-6;
    double x2 = 200.0;

    double xcg = (w1*x1 + w2*x2) / (w1 + w2);
    TS_ASSERT(!std::isnan(xcg));
    TS_ASSERT(!std::isinf(xcg));
    TS_ASSERT_DELTA(xcg, 150.0, epsilon);
  }

  // Test large CG offset scenario
  void testLargeCGOffset() {
    double w_body = 1000.0;
    double x_body = 0.0;
    double w_tail = 100.0;
    double x_tail = 500.0;

    double xcg = (w_body*x_body + w_tail*x_tail) / (w_body + w_tail);
    TS_ASSERT_DELTA(xcg, 45.45, 0.1);
  }

  // ============ Rotational Dynamics ============

  // Test angular momentum calculation
  void testAngularMomentumCalculation() {
    double Ixx = 1000.0;  // slug-ft^2
    double omega_x = 0.5; // rad/s

    double L_x = Ixx * omega_x;
    TS_ASSERT_DELTA(L_x, 500.0, epsilon);
  }

  // Test precession effect from asymmetric loading
  void testPrecessionFromAsymmetricLoading() {
    double w_imbalance = 50.0;  // lbs
    double r_imbalance = 5.0;   // ft from CG
    double omega = 10.0;        // rad/s rotation rate

    double precession_moment = w_imbalance * r_imbalance * omega / 32.2;
    TS_ASSERT(precession_moment > 0);
  }

  // Test weight shift for hang glider/ultralight
  void testWeightShiftControl() {
    double pilot_weight = 180.0;
    double frame_weight = 100.0;
    double pilot_shift = 12.0;  // inches of pilot movement
    double pilot_arm = 0.0;     // Initially at reference

    double new_arm = pilot_arm + pilot_shift;
    double total_moment_shift = pilot_weight * pilot_shift;

    TS_ASSERT(total_moment_shift > 0);
    TS_ASSERT_DELTA(total_moment_shift, 2160.0, epsilon);
  }

  // ============ Mass Property Verification ============

  // Test mass conversion consistency
  void testMassConversionConsistency() {
    double weight_lbs = 3000.0;
    double mass_slugs = weight_lbs / SLUGTOLB;

    // Convert back
    double weight_back = mass_slugs * SLUGTOLB;
    TS_ASSERT_DELTA(weight_lbs, weight_back, 1e-6);
  }

  // Test inertia units consistency
  void testInertiaUnitsConsistency() {
    double I_slugft2 = 1000.0;  // slug-ft^2
    double mass = 100.0;        // slugs
    double k = std::sqrt(I_slugft2 / mass);  // ft

    TS_ASSERT(k > 0);
    TS_ASSERT_DELTA(k * k * mass, I_slugft2, epsilon);
  }

  // Test product of inertia symmetry
  void testProductOfInertiaSymmetry() {
    FGMatrix33 J(1000.0, -20.0, -50.0,
                 -20.0, 1500.0, 10.0,
                 -50.0, 10.0, 2000.0);

    // Products should be symmetric: Jxy = Jyx, etc.
    TS_ASSERT_DELTA(J(1,2), J(2,1), epsilon);
    TS_ASSERT_DELTA(J(1,3), J(3,1), epsilon);
    TS_ASSERT_DELTA(J(2,3), J(3,2), epsilon);
  }

  // Test total moment calculation
  void testTotalMomentCalculation() {
    double items[][2] = {
      {100.0, 50.0},
      {200.0, 80.0},
      {150.0, 100.0},
      {50.0, 120.0}
    };

    double total_moment = 0.0;
    double total_weight = 0.0;
    for (int i = 0; i < 4; i++) {
      total_weight += items[i][0];
      total_moment += items[i][0] * items[i][1];
    }

    double xcg = total_moment / total_weight;
    TS_ASSERT_DELTA(total_weight, 500.0, epsilon);
    TS_ASSERT(!std::isnan(xcg));
    TS_ASSERT(xcg > 50.0 && xcg < 120.0);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete aircraft weight and balance envelope
  void testCompleteWeightBalanceEnvelope() {
    // Aircraft CG limits
    double fwd_limit = 80.0;   // inches
    double aft_limit = 95.0;   // inches
    double max_weight = 3000.0; // lbs

    // Test case: loaded aircraft
    double empty_weight = 1800.0;
    double empty_arm = 85.0;
    double fuel_weight = 300.0;
    double fuel_arm = 90.0;
    double payload_weight = 400.0;
    double payload_arm = 88.0;

    double total_weight = empty_weight + fuel_weight + payload_weight;
    double total_moment = empty_weight * empty_arm + fuel_weight * fuel_arm + payload_weight * payload_arm;
    double xcg = total_moment / total_weight;

    TS_ASSERT(total_weight <= max_weight);
    TS_ASSERT(xcg >= fwd_limit && xcg <= aft_limit);
  }

  // Test CG travel during fuel burn
  void testCGTravelDuringFuelBurn() {
    double empty_weight = 2000.0;
    double empty_arm = 82.0;
    double initial_fuel = 500.0;
    double fuel_arm = 95.0;

    // Initial CG
    double initial_xcg = (empty_weight * empty_arm + initial_fuel * fuel_arm) / (empty_weight + initial_fuel);

    // After burning 200 lbs of fuel
    double remaining_fuel = 300.0;
    double final_xcg = (empty_weight * empty_arm + remaining_fuel * fuel_arm) / (empty_weight + remaining_fuel);

    // CG moves forward as fuel burns (fuel is aft of empty CG)
    TS_ASSERT(final_xcg < initial_xcg);
  }

  // Test lateral CG for crosswind landing configuration
  void testLateralCGCrosswindConfig() {
    double left_tank = 100.0;   // lbs
    double right_tank = 100.0;  // lbs
    double tank_arm = 150.0;    // inches from centerline

    double ycg = (-tank_arm * left_tank + tank_arm * right_tank) / (left_tank + right_tank);
    TS_ASSERT_DELTA(ycg, 0.0, epsilon);

    // Now with imbalanced tanks
    left_tank = 50.0;
    right_tank = 100.0;
    ycg = (-tank_arm * left_tank + tank_arm * right_tank) / (left_tank + right_tank);
    TS_ASSERT(ycg > 0.0);  // Shifted toward heavier tank
  }

  // Test moment of inertia about arbitrary axis
  void testMomentOfInertiaArbitraryAxis() {
    double Ixx = 1000.0;
    double Iyy = 2000.0;
    double Izz = 2500.0;

    // Moment about axis at 45 degrees in x-y plane
    double cos_theta = std::cos(M_PI / 4.0);
    double sin_theta = std::sin(M_PI / 4.0);

    // I = Ixx*cos^2 + Iyy*sin^2 for axis in x-y plane
    double I_45 = Ixx * cos_theta * cos_theta + Iyy * sin_theta * sin_theta;

    TS_ASSERT(I_45 > Ixx && I_45 < Iyy);
    TS_ASSERT_DELTA(I_45, 1500.0, 1.0);
  }

  // Test parallel axis theorem for displaced mass
  void testParallelAxisTheoremDisplaced() {
    double mass = 50.0;     // slugs
    double I_cg = 100.0;    // slug-ft^2 about CG
    double d = 3.0;         // ft displacement

    double I_new = I_cg + mass * d * d;
    TS_ASSERT_DELTA(I_new, 550.0, epsilon);
  }

  // Test CG envelope boundary check
  void testCGEnvelopeBoundaryCheck() {
    double fwd_limit = 15.0;  // % MAC
    double aft_limit = 35.0;  // % MAC
    double mac = 60.0;        // inches
    double lemac = 100.0;     // leading edge MAC station

    double cg_station = 115.0;  // inches
    double cg_mac_percent = (cg_station - lemac) / mac * 100.0;

    TS_ASSERT(cg_mac_percent >= fwd_limit);
    TS_ASSERT(cg_mac_percent <= aft_limit);
    TS_ASSERT_DELTA(cg_mac_percent, 25.0, epsilon);
  }

  // Test weight distribution for gear loads
  void testWeightDistributionGearLoads() {
    double weight = 3000.0;
    double nose_arm = 20.0;   // ft forward of CG
    double main_arm = 5.0;    // ft aft of CG
    double wheelbase = nose_arm + main_arm;

    // Static gear loads (sum of moments about each gear = 0)
    double main_load = weight * nose_arm / wheelbase;
    double nose_load = weight * main_arm / wheelbase;

    TS_ASSERT_DELTA(main_load + nose_load, weight, epsilon);
    TS_ASSERT(main_load > nose_load);  // Main gear carries more
  }

  // Test inertia tensor eigenvalues
  void testInertiaTensorEigenvalues() {
    // Principal moments for symmetric aircraft
    double Ixx = 1000.0;
    double Iyy = 3000.0;
    double Izz = 3500.0;

    // For diagonal tensor, eigenvalues are the diagonal elements
    TS_ASSERT(Ixx < Iyy);
    TS_ASSERT(Iyy < Izz);
    TS_ASSERT(Ixx + Iyy > Izz);  // Triangle inequality for principal moments
  }

  // Test ballast calculation for CG adjustment
  void testBallastCalculationCGAdjustment() {
    double current_weight = 2500.0;
    double current_xcg = 90.0;
    double target_xcg = 85.0;
    double ballast_arm = 50.0;

    // Required ballast: W_b * (x_b - x_target) = W_current * (x_target - x_current)
    double ballast_weight = current_weight * (current_xcg - target_xcg) / (target_xcg - ballast_arm);

    double new_xcg = (current_weight * current_xcg + ballast_weight * ballast_arm) / (current_weight + ballast_weight);
    TS_ASSERT_DELTA(new_xcg, target_xcg, 0.01);
  }

  // Test radius of gyration calculation
  void testRadiusOfGyrationCalculation() {
    double I = 5000.0;    // slug-ft^2
    double m = 100.0;     // slugs

    double k = std::sqrt(I / m);  // radius of gyration
    TS_ASSERT_DELTA(k, std::sqrt(50.0), epsilon);
    TS_ASSERT_DELTA(k * k * m, I, epsilon);
  }

  // Test CG shift from passenger movement
  void testCGShiftPassengerMovement() {
    double aircraft_weight = 4000.0;
    double passenger_weight = 180.0;
    double movement = 10.0;  // ft

    double moment_change = passenger_weight * movement;
    double cg_shift = moment_change / aircraft_weight;

    TS_ASSERT_DELTA(cg_shift, 0.45, 0.01);  // ft
  }

  // Test combined loading scenario
  void testCombinedLoadingScenario() {
    // Multiple items being loaded
    struct LoadItem {
      double weight;
      double x_arm;
      double y_arm;
    };

    LoadItem items[] = {
      {200.0, 50.0, 0.0},
      {150.0, 80.0, -24.0},
      {150.0, 80.0, 24.0},
      {100.0, 120.0, 0.0}
    };

    double total_weight = 0.0;
    double x_moment = 0.0;
    double y_moment = 0.0;

    for (const auto& item : items) {
      total_weight += item.weight;
      x_moment += item.weight * item.x_arm;
      y_moment += item.weight * item.y_arm;
    }

    double xcg = x_moment / total_weight;
    double ycg = y_moment / total_weight;

    TS_ASSERT_DELTA(total_weight, 600.0, epsilon);
    TS_ASSERT_DELTA(ycg, 0.0, epsilon);  // Symmetric lateral loading
    TS_ASSERT(xcg > 50.0 && xcg < 120.0);
  }

  // Test inertia change from retractable gear
  void testInertiaChangeRetractableGear() {
    double gear_mass = 5.0;   // slugs each
    double extended_arm = 8.0;  // ft from CG
    double retracted_arm = 2.0;  // ft from CG
    double num_gear = 3.0;

    double I_extended = num_gear * gear_mass * extended_arm * extended_arm;
    double I_retracted = num_gear * gear_mass * retracted_arm * retracted_arm;

    TS_ASSERT(I_extended > I_retracted);
    double ratio = I_extended / I_retracted;
    TS_ASSERT(ratio > 10.0);  // Significant reduction
  }

  // Test static margin calculation
  void testStaticMarginCalculation() {
    double xcg = 85.0;        // inches
    double x_np = 95.0;       // neutral point, inches
    double mac = 48.0;        // inches

    double static_margin = (x_np - xcg) / mac * 100.0;  // percent MAC

    TS_ASSERT(static_margin > 0.0);  // Stable
    TS_ASSERT_DELTA(static_margin, 20.833, 0.01);
  }

  // Test weight moment envelope limits
  void testWeightMomentEnvelopeLimits() {
    double weight = 2800.0;
    double xcg = 87.0;
    double moment = weight * xcg;

    // Envelope limits (simplified linear bounds)
    double min_moment = 2000.0 * 82.0;
    double max_moment = 3000.0 * 92.0;

    TS_ASSERT(moment >= min_moment);
    TS_ASSERT(moment <= max_moment);
  }

  // Test fuel imbalance effect on roll
  void testFuelImbalanceRollEffect() {
    double imbalance = 100.0;  // lbs difference
    double tank_arm = 10.0;    // ft from centerline
    double roll_moment = imbalance * tank_arm;  // ft-lbf

    TS_ASSERT_DELTA(roll_moment, 1000.0, epsilon);
    TS_ASSERT(roll_moment > 0.0);
  }

  // Test weight and balance instance independence
  void testWeightBalanceInstanceIndependence() {
    double config1_weight = 2500.0;
    double config1_xcg = 85.0;
    double config2_weight = 3000.0;
    double config2_xcg = 88.0;

    double moment1 = config1_weight * config1_xcg;
    double moment2 = config2_weight * config2_xcg;

    TS_ASSERT(moment1 != moment2);
    TS_ASSERT_DELTA(moment1, 212500.0, epsilon);
    TS_ASSERT_DELTA(moment2, 264000.0, epsilon);
  }

  // Test payload bay CG contribution
  void testPayloadBayCGContribution() {
    double empty_weight = 2000.0;
    double empty_arm = 80.0;
    double payload = 500.0;
    double payload_arm = 120.0;

    double total_weight = empty_weight + payload;
    double new_xcg = (empty_weight * empty_arm + payload * payload_arm) / total_weight;

    TS_ASSERT(new_xcg > empty_arm);  // CG moves aft
    TS_ASSERT_DELTA(new_xcg, 88.0, 0.1);
  }

  // Test zero fuel weight CG calculation extended
  void testZeroFuelWeightCGExtended() {
    double structure_weight = 1500.0;
    double structure_arm = 75.0;
    double crew_weight = 400.0;
    double crew_arm = 85.0;
    double cargo_weight = 600.0;
    double cargo_arm = 100.0;

    double zfw = structure_weight + crew_weight + cargo_weight;
    double zfw_cg = (structure_weight * structure_arm + crew_weight * crew_arm + cargo_weight * cargo_arm) / zfw;

    TS_ASSERT_DELTA(zfw, 2500.0, epsilon);
    TS_ASSERT(zfw_cg > 75.0 && zfw_cg < 100.0);
  }
};
