/*******************************************************************************
 * FGGroundEffectTest.h - Unit tests for ground effect aerodynamics
 *
 * Tests the mathematical behavior of ground effect aerodynamic calculations:
 * - Induced drag reduction in ground effect
 * - Lift coefficient increase near ground
 * - Height-to-span ratio effects
 * - Ground effect factor calculations
 * - Downwash reduction near ground
 * - Pitch moment changes in ground effect
 * - Flare characteristics
 * - Float tendency modeling
 * - Cushion effect
 * - Wing-in-ground effect vehicles
 * - Runway proximity effects
 * - Crosswind ground effect
 *
 * Ground effect is a phenomenon where aerodynamic forces change when an aircraft
 * is flying very close to the ground. The primary effects are:
 * 1. Reduced induced drag due to interference with wingtip vortices
 * 2. Increased lift coefficient
 * 3. Decreased downwash angle
 * 4. Changes in pitch moment
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double PI = 3.14159265358979323846;

class FGGroundEffectTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Ground Effect Factor Calculations
   ***************************************************************************/

  // Test ground effect factor at various heights
  void testGroundEffectFactorHighAltitude() {
    double height = 100.0;  // ft
    double wingspan = 30.0; // ft
    double h_over_b = height / wingspan;

    // At high altitude (h/b >> 1), ground effect factor should be ~1.0 (no effect)
    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT_DELTA(phi, 1.0, 0.01);
  }

  // Test ground effect factor at very low height
  void testGroundEffectFactorVeryLow() {
    double height = 1.0;   // ft
    double wingspan = 30.0; // ft
    double h_over_b = height / wingspan;

    // At very low altitude (h/b << 1), ground effect factor should be << 1.0
    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT(phi < 0.5);
    TS_ASSERT(phi > 0.0);
  }

  // Test ground effect factor at typical landing height
  void testGroundEffectFactorLandingHeight() {
    double height = 5.0;   // ft (typical flare height)
    double wingspan = 30.0; // ft
    double h_over_b = height / wingspan;

    // At landing height, should have noticeable ground effect
    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT(phi < 1.0);
    TS_ASSERT(phi > 0.3);
  }

  // Test ground effect factor monotonicity
  void testGroundEffectFactorMonotonic() {
    double wingspan = 30.0;
    double h1 = 2.0;
    double h2 = 5.0;
    double h3 = 10.0;

    double phi1 = calculateGroundEffectFactor(h1 / wingspan);
    double phi2 = calculateGroundEffectFactor(h2 / wingspan);
    double phi3 = calculateGroundEffectFactor(h3 / wingspan);

    // Ground effect factor should increase with height
    TS_ASSERT(phi1 < phi2);
    TS_ASSERT(phi2 < phi3);
  }

  /***************************************************************************
   * Induced Drag Reduction Tests
   ***************************************************************************/

  // Test induced drag reduction formula
  void testInducedDragReduction() {
    double CDi_freestream = 0.05;  // Induced drag coefficient out of ground effect
    double h_over_b = 0.1;         // h/b = 0.1 (low altitude)

    // Ground effect reduces induced drag
    double phi = calculateGroundEffectFactor(h_over_b);
    double CDi_ground = CDi_freestream * phi * phi;

    TS_ASSERT(CDi_ground < CDi_freestream);
    TS_ASSERT(CDi_ground > 0.0);
  }

  // Test induced drag at increasing heights
  void testInducedDragWithHeight() {
    double CDi_freestream = 0.05;
    double wingspan = 30.0;

    double CDi_low = CDi_freestream * pow(calculateGroundEffectFactor(2.0 / wingspan), 2);
    double CDi_mid = CDi_freestream * pow(calculateGroundEffectFactor(5.0 / wingspan), 2);
    double CDi_high = CDi_freestream * pow(calculateGroundEffectFactor(15.0 / wingspan), 2);

    // Induced drag should increase with height
    TS_ASSERT(CDi_low < CDi_mid);
    TS_ASSERT(CDi_mid < CDi_high);
    TS_ASSERT_DELTA(CDi_high, CDi_freestream, 0.005);  // Within 10% at h/b = 0.5
  }

  // Test induced drag zero at zero lift
  void testInducedDragZeroLift() {
    double CL = 0.0;
    double AR = 8.0;  // Aspect ratio
    double e = 0.85;  // Oswald efficiency factor

    double CDi = (CL * CL) / (PI * AR * e);
    TS_ASSERT_DELTA(CDi, 0.0, epsilon);
  }

  // Test induced drag formula with ground effect
  void testInducedDragFormulaGroundEffect() {
    double CL = 1.0;
    double AR = 8.0;
    double e = 0.85;
    double h_over_b = 0.15;

    double CDi_freestream = (CL * CL) / (PI * AR * e);
    double phi = calculateGroundEffectFactor(h_over_b);
    double CDi_ground = CDi_freestream * phi * phi;

    TS_ASSERT(CDi_ground > 0.0);
    TS_ASSERT(CDi_ground < CDi_freestream);
  }

  /***************************************************************************
   * Lift Coefficient Changes
   ***************************************************************************/

  // Test lift coefficient increase in ground effect
  void testLiftCoefficientIncrease() {
    double CL_freestream = 1.0;
    double h_over_b = 0.1;

    // Empirical formula: CL increases by reducing induced angle of attack
    double phi = calculateGroundEffectFactor(h_over_b);
    double CL_ground = CL_freestream / phi;  // Simplified model

    TS_ASSERT(CL_ground > CL_freestream);
  }

  // Test lift slope increase in ground effect
  void testLiftSlopeIncrease() {
    double CL_alpha_freestream = 5.5;  // per radian
    double h_over_b = 0.2;

    // Lift slope increases in ground effect
    double phi = calculateGroundEffectFactor(h_over_b);
    double CL_alpha_ground = CL_alpha_freestream / phi;

    TS_ASSERT(CL_alpha_ground > CL_alpha_freestream);
  }

  // Test lift at different heights
  void testLiftAtDifferentHeights() {
    double CL_base = 1.0;
    double wingspan = 30.0;

    double CL_low = CL_base / calculateGroundEffectFactor(2.0 / wingspan);
    double CL_mid = CL_base / calculateGroundEffectFactor(5.0 / wingspan);
    double CL_high = CL_base / calculateGroundEffectFactor(20.0 / wingspan);

    // Lift should decrease as height increases
    TS_ASSERT(CL_low > CL_mid);
    TS_ASSERT(CL_mid > CL_high);
    TS_ASSERT_DELTA(CL_high, CL_base, 0.05);
  }

  /***************************************************************************
   * Downwash Reduction Tests
   ***************************************************************************/

  // Test downwash angle reduction
  void testDownwashReduction() {
    double epsilon_freestream = 0.1;  // radians (downwash angle)
    double h_over_b = 0.15;

    double phi = calculateGroundEffectFactor(h_over_b);
    double epsilon_ground = epsilon_freestream * phi;

    TS_ASSERT(epsilon_ground < epsilon_freestream);
    TS_ASSERT(epsilon_ground > 0.0);
  }

  // Test downwash gradient reduction
  void testDownwashGradientReduction() {
    double de_dalpha_freestream = 0.4;  // Downwash gradient
    double h_over_b = 0.12;

    double phi = calculateGroundEffectFactor(h_over_b);
    double de_dalpha_ground = de_dalpha_freestream * phi;

    TS_ASSERT(de_dalpha_ground < de_dalpha_freestream);
    TS_ASSERT(de_dalpha_ground > 0.0);
  }

  // Test effective angle of attack increase
  void testEffectiveAngleOfAttackIncrease() {
    double alpha_geometric = 5.0;  // degrees
    double downwash_freestream = 2.0;  // degrees
    double h_over_b = 0.1;

    double phi = calculateGroundEffectFactor(h_over_b);
    double downwash_ground = downwash_freestream * phi;

    double alpha_eff_freestream = alpha_geometric - downwash_freestream;
    double alpha_eff_ground = alpha_geometric - downwash_ground;

    // Effective angle of attack increases in ground effect
    TS_ASSERT(alpha_eff_ground > alpha_eff_freestream);
  }

  /***************************************************************************
   * Pitch Moment Changes
   ***************************************************************************/

  // Test nose-down pitch moment in ground effect
  void testPitchMomentChange() {
    double Cm_freestream = 0.05;  // Positive (nose up)
    double h_over_b = 0.1;

    // Ground effect typically causes nose-down moment
    double delta_Cm = calculatePitchMomentDelta(h_over_b);
    double Cm_ground = Cm_freestream + delta_Cm;

    TS_ASSERT(delta_Cm < 0.0);  // Nose-down increment
  }

  // Test pitch moment varies with height
  void testPitchMomentVariesWithHeight() {
    double wingspan = 30.0;

    double delta_Cm_low = calculatePitchMomentDelta(2.0 / wingspan);
    double delta_Cm_mid = calculatePitchMomentDelta(5.0 / wingspan);
    double delta_Cm_high = calculatePitchMomentDelta(15.0 / wingspan);

    // Nose-down moment should be strongest at low altitude
    TS_ASSERT(delta_Cm_low < delta_Cm_mid);
    TS_ASSERT(delta_Cm_mid < delta_Cm_high);
    TS_ASSERT_DELTA(delta_Cm_high, 0.0, 0.001);
  }

  // Test center of pressure shift
  void testCenterOfPressureShift() {
    double h_over_b = 0.15;
    double chord = 5.0;  // ft

    // Ground effect shifts CP forward
    double x_cp_shift = calculateCPShift(h_over_b, chord);
    TS_ASSERT(x_cp_shift > 0.0);  // Forward shift (positive)
  }

  /***************************************************************************
   * Flare Characteristics
   ***************************************************************************/

  // Test float tendency at flare initiation
  void testFloatTendency() {
    double height_flare_start = 20.0;  // ft
    double height_flare_end = 2.0;     // ft
    double wingspan = 30.0;

    double h_b_start = height_flare_start / wingspan;
    double h_b_end = height_flare_end / wingspan;

    double phi_start = calculateGroundEffectFactor(h_b_start);
    double phi_end = calculateGroundEffectFactor(h_b_end);

    // Rapid change in ground effect during flare
    double phi_change = phi_end - phi_start;
    TS_ASSERT(phi_change < -0.1);  // Significant decrease
  }

  // Test sink rate reduction in ground effect
  void testSinkRateReduction() {
    double sink_rate_freestream = -5.0;  // ft/s (negative = descending)
    double h_over_b = 0.1;

    // Ground effect creates cushion, reducing sink rate
    double cushion_factor = calculateCushionFactor(h_over_b);
    double sink_rate_ground = sink_rate_freestream * cushion_factor;

    TS_ASSERT(sink_rate_ground > sink_rate_freestream);  // Less negative
    TS_ASSERT(cushion_factor > 0.0);
    TS_ASSERT(cushion_factor < 1.0);
  }

  // Test flare float distance estimation
  void testFlareFloatDistance() {
    double velocity = 100.0;  // ft/s
    double height = 3.0;      // ft
    double wingspan = 30.0;

    // Simplified float distance based on excess lift
    double excess_lift_factor = 1.0 / calculateGroundEffectFactor(height / wingspan) - 1.0;
    double float_time = excess_lift_factor * 2.0;  // Simplified
    double float_distance = velocity * float_time;

    TS_ASSERT(float_distance > 0.0);
    TS_ASSERT(excess_lift_factor > 0.0);
  }

  /***************************************************************************
   * Cushion Effect
   ***************************************************************************/

  // Test cushion effect at very low height
  void testCushionEffectVeryLow() {
    double h_over_b = 0.05;  // Very close to ground

    double cushion = calculateCushionFactor(h_over_b);
    TS_ASSERT(cushion < 0.5);  // Strong cushion effect
    TS_ASSERT(cushion > 0.0);
  }

  // Test cushion effect diminishes with height
  void testCushionEffectDiminishes() {
    double wingspan = 30.0;

    double cushion_low = calculateCushionFactor(1.0 / wingspan);
    double cushion_mid = calculateCushionFactor(5.0 / wingspan);
    double cushion_high = calculateCushionFactor(20.0 / wingspan);

    TS_ASSERT(cushion_low < cushion_mid);
    TS_ASSERT(cushion_mid < cushion_high);
    TS_ASSERT_DELTA(cushion_high, 1.0, 0.05);
  }

  // Test ram air pressure increase
  void testRamAirPressureIncrease() {
    double velocity = 100.0;  // ft/s
    double density = 0.002377;  // slugs/ft^3
    double h_over_b = 0.08;

    // Dynamic pressure increase due to ram effect
    double q_base = 0.5 * density * velocity * velocity;
    double ram_factor = 1.0 + (1.0 - h_over_b) * 0.1;  // Simplified
    double q_ground = q_base * ram_factor;

    TS_ASSERT(q_ground > q_base);
  }

  /***************************************************************************
   * Wing-In-Ground Effect Vehicles (WIG/Ekranoplan)
   ***************************************************************************/

  // Test WIG extreme ground effect
  void testWIGExtremeGroundEffect() {
    double h_over_b = 0.02;  // Very low (2% of wingspan)

    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT(phi < 0.2);  // Very strong ground effect
  }

  // Test WIG lift-to-drag ratio improvement
  void testWIGLiftToDragImprovement() {
    double CL = 0.8;
    double CD0 = 0.015;  // Parasitic drag
    double AR = 6.0;
    double e = 0.85;

    // Freestream L/D
    double CDi_free = (CL * CL) / (PI * AR * e);
    double LD_free = CL / (CD0 + CDi_free);

    // In ground effect
    double h_over_b = 0.03;
    double phi = calculateGroundEffectFactor(h_over_b);
    double CDi_wig = CDi_free * phi * phi;
    double CL_wig = CL / phi;
    double LD_wig = CL_wig / (CD0 + CDi_wig);

    // WIG should have significantly better L/D
    TS_ASSERT(LD_wig > LD_free * 1.5);
  }

  // Test WIG cruise efficiency
  void testWIGCruiseEfficiency() {
    double h_over_b = 0.05;

    // At very low altitude, induced drag is minimal
    double phi = calculateGroundEffectFactor(h_over_b);
    double drag_reduction = 1.0 - phi * phi;

    TS_ASSERT(drag_reduction > 0.5);  // >50% reduction in induced drag
  }

  /***************************************************************************
   * Height-to-Span Ratio Effects
   ***************************************************************************/

  // Test critical ground effect height
  void testCriticalGroundEffectHeight() {
    // Ground effect becomes significant below h/b ≈ 1.0
    double wingspan = 30.0;
    double h_critical = wingspan;  // One wingspan

    double phi_critical = calculateGroundEffectFactor(h_critical / wingspan);
    // At h/b = 1.0, ground effect is minimal but measurable
    TS_ASSERT(phi_critical < 0.997);  // Measurable effect (within 0.3%)
    TS_ASSERT(phi_critical > 0.9);
  }

  // Test ground effect boundary
  void testGroundEffectBoundary() {
    // Ground effect negligible above h/b ≈ 1.5
    double h_over_b = 1.5;

    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT_DELTA(phi, 1.0, 0.02);  // Within 2% of no ground effect
  }

  // Test aspect ratio influence
  void testAspectRatioInfluence() {
    double height = 5.0;  // ft

    // High aspect ratio wing (more sensitive to ground effect)
    double wingspan_high_AR = 40.0;
    double phi_high_AR = calculateGroundEffectFactor(height / wingspan_high_AR);

    // Low aspect ratio wing
    double wingspan_low_AR = 20.0;
    double phi_low_AR = calculateGroundEffectFactor(height / wingspan_low_AR);

    // High AR wing experiences stronger ground effect at same height
    TS_ASSERT(phi_high_AR < phi_low_AR);
  }

  /***************************************************************************
   * Runway Proximity Effects
   ***************************************************************************/

  // Test ground effect on approach
  void testGroundEffectOnApproach() {
    double glide_slope = 3.0;  // degrees
    double wingspan = 30.0;
    double runway_distance = 500.0;  // ft from threshold

    // Height decreases linearly on approach
    double height = runway_distance * tan(glide_slope * PI / 180.0);
    double h_over_b = height / wingspan;

    if (height < wingspan) {
      double phi = calculateGroundEffectFactor(h_over_b);
      TS_ASSERT(phi < 1.0);  // In ground effect
    }
  }

  // Test touchdown zone ground effect
  void testTouchdownZoneEffect() {
    double height_main_wheels = 3.0;  // ft at touchdown attitude
    double wingspan = 30.0;

    double phi = calculateGroundEffectFactor(height_main_wheels / wingspan);
    TS_ASSERT(phi < 0.8);  // Strong ground effect at touchdown
  }

  // Test ground effect during go-around
  void testGroundEffectGoAround() {
    double initial_height = 5.0;   // ft
    double final_height = 50.0;    // ft
    double wingspan = 30.0;

    double phi_initial = calculateGroundEffectFactor(initial_height / wingspan);
    double phi_final = calculateGroundEffectFactor(final_height / wingspan);

    // Drag increases during go-around climb
    double drag_ratio = (phi_final * phi_final) / (phi_initial * phi_initial);
    TS_ASSERT(drag_ratio > 1.2);  // Induced drag increases by at least 20%
  }

  /***************************************************************************
   * Crosswind Ground Effect
   ***************************************************************************/

  // Test asymmetric ground effect in crosswind
  void testAsymmetricGroundEffectCrosswind() {
    double wingspan = 30.0;
    double height_upwind = 5.0;   // ft
    double bank_angle = 5.0;      // degrees
    double height_downwind = height_upwind - wingspan * sin(bank_angle * PI / 180.0) / 2;

    double phi_upwind = calculateGroundEffectFactor(height_upwind / wingspan);
    double phi_downwind = calculateGroundEffectFactor(height_downwind / wingspan);

    // Downwind wing experiences stronger ground effect
    TS_ASSERT(phi_downwind < phi_upwind);
  }

  // Test rolling moment due to asymmetric ground effect
  void testRollingMomentAsymmetric() {
    double wingspan = 30.0;
    double height_left = 4.0;   // ft
    double height_right = 6.0;  // ft

    double phi_left = calculateGroundEffectFactor(height_left / wingspan);
    double phi_right = calculateGroundEffectFactor(height_right / wingspan);

    // Differential lift creates rolling moment
    double CL_base = 1.0;
    double CL_left = CL_base / phi_left;
    double CL_right = CL_base / phi_right;
    double delta_CL = CL_left - CL_right;

    TS_ASSERT(delta_CL > 0.0);  // Left wing has more lift
  }

  // Test yawing moment in crosswind landing
  void testYawingMomentCrosswind() {
    double wingspan = 30.0;
    double height = 4.0;
    double sideslip = 10.0;  // degrees

    // Asymmetric flow creates yaw moment
    double h_over_b = height / wingspan;
    double phi = calculateGroundEffectFactor(h_over_b);

    // Simplified: yaw moment proportional to sideslip and ground effect
    double Cn_ground = sideslip * (1.0 - phi) * 0.001;  // Empirical
    TS_ASSERT(Cn_ground > 0.0);
  }

  /***************************************************************************
   * Advanced Ground Effect Formulas
   ***************************************************************************/

  // Test ground effect with dihedral
  void testGroundEffectWithDihedral() {
    double wingspan = 30.0;
    double height_fuselage = 5.0;  // ft
    double dihedral = 5.0;         // degrees

    // Effective height is at wing mid-span, higher due to dihedral
    double height_effective = height_fuselage + wingspan * sin(dihedral * PI / 180.0) / 4;
    double phi = calculateGroundEffectFactor(height_effective / wingspan);

    TS_ASSERT(phi > calculateGroundEffectFactor(height_fuselage / wingspan));
  }

  // Test ground effect with anhedral
  void testGroundEffectWithAnhedral() {
    double wingspan = 30.0;
    double height_fuselage = 5.0;  // ft
    double anhedral = -3.0;        // degrees (negative)

    // Effective height is lower with anhedral
    double height_effective = height_fuselage + wingspan * sin(anhedral * PI / 180.0) / 4;
    double phi = calculateGroundEffectFactor(height_effective / wingspan);

    TS_ASSERT(phi < calculateGroundEffectFactor(height_fuselage / wingspan));
  }

  // Test ground effect on swept wing
  void testGroundEffectSweptWing() {
    double wingspan = 30.0;
    double sweep_angle = 25.0;  // degrees
    double height = 5.0;

    // Effective span may differ for swept wing
    double effective_span = wingspan * cos(sweep_angle * PI / 180.0);
    double phi_swept = calculateGroundEffectFactor(height / effective_span);
    double phi_straight = calculateGroundEffectFactor(height / wingspan);

    // Swept wing experiences different ground effect
    TS_ASSERT(phi_swept != phi_straight);
  }

  /***************************************************************************
   * Ground Effect Onset and Departure
   ***************************************************************************/

  // Test gradual onset during descent
  void testGradualOnset() {
    double wingspan = 30.0;
    double heights[] = {100.0, 50.0, 30.0, 20.0, 10.0, 5.0, 2.0};
    int n = sizeof(heights) / sizeof(heights[0]);

    double previous_phi = 1.0;
    for (int i = 0; i < n; i++) {
      double phi = calculateGroundEffectFactor(heights[i] / wingspan);
      TS_ASSERT(phi <= previous_phi);  // Monotonically decreasing
      previous_phi = phi;
    }
  }

  // Test departure during climb
  void testDepartureDuringClimb() {
    double wingspan = 30.0;
    double heights[] = {2.0, 5.0, 10.0, 20.0, 30.0, 50.0, 100.0};
    int n = sizeof(heights) / sizeof(heights[0]);

    double previous_phi = 0.0;
    for (int i = 0; i < n; i++) {
      double phi = calculateGroundEffectFactor(heights[i] / wingspan);
      TS_ASSERT(phi >= previous_phi);  // Monotonically increasing
      previous_phi = phi;
    }
  }

  // Test rate of change of ground effect
  void testRateOfChange() {
    double wingspan = 30.0;
    double h1 = 3.0;
    double h2 = 3.1;  // Small increment

    double phi1 = calculateGroundEffectFactor(h1 / wingspan);
    double phi2 = calculateGroundEffectFactor(h2 / wingspan);

    double dphi_dh = (phi2 - phi1) / (h2 - h1);
    TS_ASSERT(dphi_dh > 0.0);  // Increases with height
  }

  /***************************************************************************
   * Edge Cases and Boundary Conditions
   ***************************************************************************/

  // Test at zero height (on ground)
  void testZeroHeight() {
    double h_over_b = 0.0;

    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT(phi >= 0.0);  // Should not be negative
    TS_ASSERT(phi < 0.1);   // Very strong effect
  }

  // Test at extreme height
  void testExtremeHeight() {
    double height = 10000.0;  // ft
    double wingspan = 30.0;

    double phi = calculateGroundEffectFactor(height / wingspan);
    TS_ASSERT_DELTA(phi, 1.0, 0.0001);  // Effectively no ground effect
  }

  // Test with very large wingspan
  void testVeryLargeWingspan() {
    double height = 10.0;
    double wingspan = 200.0;  // Large aircraft

    double phi = calculateGroundEffectFactor(height / wingspan);
    TS_ASSERT(phi < 0.8);  // Significant ground effect even at 10 ft
  }

  // Test with very small wingspan
  void testVerySmallWingspan() {
    double height = 10.0;
    double wingspan = 5.0;  // Small UAV

    double phi = calculateGroundEffectFactor(height / wingspan);
    TS_ASSERT_DELTA(phi, 1.0, 0.1);  // Minimal ground effect
  }

private:
  /***************************************************************************
   * Helper Functions for Ground Effect Calculations
   ***************************************************************************/

  // Calculate ground effect factor using simplified empirical formula
  // Based on Wieselsberger ground effect theory
  double calculateGroundEffectFactor(double h_over_b) const {
    // Ensure non-negative input
    if (h_over_b <= 0.0) return 0.01;  // Very strong effect at ground level

    // Empirical formula: phi = (16 * h/b)^2 / (1 + (16 * h/b)^2)
    // This approximates the reduction in induced drag/downwash
    double ratio = 16.0 * h_over_b;
    double phi = (ratio * ratio) / (1.0 + ratio * ratio);

    return phi;
  }

  // Calculate pitch moment change due to ground effect
  double calculatePitchMomentDelta(double h_over_b) const {
    // Ground effect creates nose-down moment
    // Empirical formula based on CP shift
    double phi = calculateGroundEffectFactor(h_over_b);
    double delta_Cm = -0.05 * (1.0 - phi);  // Nose-down moment

    return delta_Cm;
  }

  // Calculate center of pressure shift
  double calculateCPShift(double h_over_b, double chord) const {
    // Forward shift of CP in ground effect
    double phi = calculateGroundEffectFactor(h_over_b);
    double shift_fraction = 0.05 * (1.0 - phi);  // Up to 5% chord forward
    return shift_fraction * chord;
  }

  // Calculate cushion factor for sink rate
  double calculateCushionFactor(double h_over_b) const {
    // Cushion reduces sink rate (0 = full cushion, 1 = no cushion)
    double phi = calculateGroundEffectFactor(h_over_b);
    return phi;  // Simplified: cushion factor equals ground effect factor
  }
};
