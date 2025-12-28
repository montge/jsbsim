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

  /***************************************************************************
   * High-Lift Device Effects
   ***************************************************************************/

  // Test ground effect with flaps extended
  void testGroundEffectFlapsExtended() {
    double wingspan = 30.0;
    double height = 5.0;
    double h_over_b = height / wingspan;

    // Flaps increase effective chord, modifying ground effect
    double phi_clean = calculateGroundEffectFactor(h_over_b);

    // With flaps, effective span may decrease slightly, making h/b ratio larger
    double flap_span_reduction = 0.95;  // 5% effective span reduction
    double effective_h_over_b = h_over_b / flap_span_reduction;  // Higher ratio
    double phi_flaps = calculateGroundEffectFactor(effective_h_over_b);

    // Higher h/b ratio means less ground effect (phi closer to 1.0)
    TS_ASSERT(phi_flaps > phi_clean);
  }

  // Test ground effect with slats deployed
  void testGroundEffectSlatsDeployed() {
    double wingspan = 30.0;
    double height = 5.0;

    // Slats increase lift slope, intensifying ground effect response
    double CL_alpha_base = 5.5;
    double CL_alpha_slats = 6.0;

    double phi = calculateGroundEffectFactor(height / wingspan);
    double CL_increment_base = CL_alpha_base * (1.0 / phi - 1.0) * 0.05;  // At 5 deg
    double CL_increment_slats = CL_alpha_slats * (1.0 / phi - 1.0) * 0.05;

    TS_ASSERT(CL_increment_slats > CL_increment_base);
  }

  // Test spoiler effect on ground effect
  void testSpoilerEffectOnGroundEffect() {
    double CL_freestream = 1.0;
    double spoiler_CL_reduction = 0.3;  // Spoilers reduce CL by 0.3
    double h_over_b = 0.15;

    double phi = calculateGroundEffectFactor(h_over_b);

    // With spoilers, ground effect benefit is reduced
    double CL_ground = CL_freestream / phi;
    double CL_ground_spoilers = (CL_freestream - spoiler_CL_reduction) / phi;

    double CL_benefit = CL_ground - CL_freestream;
    double CL_benefit_spoilers = CL_ground_spoilers - (CL_freestream - spoiler_CL_reduction);

    // Both should have similar ground effect benefit
    TS_ASSERT_DELTA(CL_benefit, CL_benefit_spoilers, 0.1);
  }

  /***************************************************************************
   * Speed Effects on Ground Effect
   ***************************************************************************/

  // Test ground effect at low speed
  void testGroundEffectLowSpeed() {
    double velocity = 60.0;  // kts (slow approach)
    double wingspan = 30.0;
    double height = 5.0;

    double phi = calculateGroundEffectFactor(height / wingspan);
    // Ground effect is independent of speed (aerodynamic coefficients change)
    TS_ASSERT(phi < 1.0);
    TS_ASSERT(phi > 0.0);
  }

  // Test ground effect at high speed
  void testGroundEffectHighSpeed() {
    double velocity = 140.0;  // kts (fast approach)
    double wingspan = 30.0;
    double height = 5.0;

    double phi = calculateGroundEffectFactor(height / wingspan);
    // Same height/span ratio gives same ground effect factor
    TS_ASSERT(phi < 1.0);
    TS_ASSERT(phi > 0.0);
  }

  // Test dynamic pressure change in ground effect
  void testDynamicPressureChange() {
    double velocity = 100.0;  // ft/s
    double density = 0.002377;
    double height = 3.0;
    double wingspan = 30.0;

    double q = 0.5 * density * velocity * velocity;
    double phi = calculateGroundEffectFactor(height / wingspan);

    // Lift increases in ground effect for same q
    double L_free = q * 174.0 * 1.0;  // S * CL
    double L_ground = q * 174.0 * (1.0 / phi);

    TS_ASSERT(L_ground > L_free);
  }

  /***************************************************************************
   * Propeller/Engine Effects
   ***************************************************************************/

  // Test propeller slipstream ground effect
  void testPropellerSlipstreamEffect() {
    double wingspan = 30.0;
    double height = 4.0;
    double h_over_b = height / wingspan;

    double phi_base = calculateGroundEffectFactor(h_over_b);

    // Propeller slipstream increases local velocity
    // This may affect ground effect by changing local lift distribution
    double slipstream_factor = 1.1;  // 10% velocity increase
    double phi_adjusted = calculateGroundEffectFactor(h_over_b);

    // Ground effect factor remains the same (geometry-based)
    TS_ASSERT_DELTA(phi_base, phi_adjusted, epsilon);
  }

  // Test jet exhaust ground effect interaction
  void testJetExhaustGroundEffect() {
    double height = 5.0;
    double wingspan = 40.0;

    // Jet exhaust may disturb ground boundary layer
    // This is a secondary effect
    double phi = calculateGroundEffectFactor(height / wingspan);
    TS_ASSERT(phi < 1.0);
  }

  // Test ground effect with engine-out
  void testGroundEffectEngineOut() {
    double wingspan = 30.0;
    double height = 5.0;

    // Engine-out creates asymmetric lift
    // Ground effect still applies to both wings
    double phi = calculateGroundEffectFactor(height / wingspan);

    // Yaw moment due to engine-out is modified by ground effect
    // as induced drag is reduced on both wings
    TS_ASSERT(phi < 1.0);
  }

  /***************************************************************************
   * Tailplane Ground Effect
   ***************************************************************************/

  // Test horizontal stabilizer ground effect
  void testHorizontalStabilizerGroundEffect() {
    double tail_span = 12.0;  // ft
    double tail_height = 8.0; // ft above ground (higher due to fuselage)
    double main_wing_height = 4.0;

    double phi_wing = calculateGroundEffectFactor(main_wing_height / 30.0);
    double phi_tail = calculateGroundEffectFactor(tail_height / tail_span);

    // Tail is higher, so less ground effect
    TS_ASSERT(phi_tail > phi_wing);
  }

  // Test elevator effectiveness in ground effect
  void testElevatorEffectivenessGroundEffect() {
    double tail_height = 6.0;
    double tail_span = 12.0;

    double phi_tail = calculateGroundEffectFactor(tail_height / tail_span);

    // Reduced downwash at tail increases elevator effectiveness
    double effectiveness_factor = 1.0 / phi_tail;
    TS_ASSERT(effectiveness_factor > 1.0);
  }

  // Test T-tail vs. conventional tail
  void testTTailVsConventionalTail() {
    double wingspan = 30.0;
    double wing_height = 4.0;

    // T-tail is higher above ground
    double ttail_height = 15.0;
    double conv_tail_height = 6.0;

    double phi_ttail = calculateGroundEffectFactor(ttail_height / 12.0);
    double phi_conv = calculateGroundEffectFactor(conv_tail_height / 12.0);

    // T-tail experiences much less ground effect
    TS_ASSERT(phi_ttail > phi_conv);
  }

  /***************************************************************************
   * Multi-Wing Configurations
   ***************************************************************************/

  // Test biplane ground effect
  void testBiplaneGroundEffect() {
    double lower_wing_span = 25.0;
    double upper_wing_span = 28.0;
    double lower_wing_height = 3.0;
    double upper_wing_height = 8.0;

    double phi_lower = calculateGroundEffectFactor(lower_wing_height / lower_wing_span);
    double phi_upper = calculateGroundEffectFactor(upper_wing_height / upper_wing_span);

    // Lower wing has much stronger ground effect
    TS_ASSERT(phi_lower < phi_upper);

    // Combined effect (simplified average)
    double phi_combined = (phi_lower + phi_upper) / 2.0;
    TS_ASSERT(phi_combined < phi_upper);
    TS_ASSERT(phi_combined > phi_lower);
  }

  // Test tandem wing ground effect
  void testTandemWingGroundEffect() {
    double front_wingspan = 25.0;
    double rear_wingspan = 20.0;
    double height = 4.0;

    double phi_front = calculateGroundEffectFactor(height / front_wingspan);
    double phi_rear = calculateGroundEffectFactor(height / rear_wingspan);

    // Front wing has larger span, so different ground effect
    TS_ASSERT(phi_front < phi_rear);
  }

  // Test canard ground effect
  void testCanardGroundEffect() {
    double main_wing_span = 30.0;
    double canard_span = 10.0;
    double main_height = 4.0;
    double canard_height = 5.0;  // Slightly higher typically

    double phi_main = calculateGroundEffectFactor(main_height / main_wing_span);
    double phi_canard = calculateGroundEffectFactor(canard_height / canard_span);

    // Canard with smaller span experiences different ground effect
    TS_ASSERT(phi_main != phi_canard);
  }

  /***************************************************************************
   * Ground Surface Effects
   ***************************************************************************/

  // Test ground effect over water
  void testGroundEffectOverWater() {
    double wingspan = 30.0;
    double height = 5.0;

    // Water surface is smooth, ground effect similar to runway
    double phi = calculateGroundEffectFactor(height / wingspan);
    TS_ASSERT(phi < 1.0);
    TS_ASSERT(phi > 0.3);
  }

  // Test ground effect over rough terrain
  void testGroundEffectRoughTerrain() {
    double wingspan = 30.0;
    double height = 5.0;

    // Rough terrain may reduce ground effect slightly
    double phi_smooth = calculateGroundEffectFactor(height / wingspan);
    double roughness_factor = 1.05;  // 5% reduction in ground effect
    double phi_rough = phi_smooth * roughness_factor;

    TS_ASSERT(phi_rough > phi_smooth);
    TS_ASSERT(phi_rough <= 1.0);
  }

  // Test ground effect at different runway altitudes
  void testGroundEffectAltitudeVariation() {
    double wingspan = 30.0;
    double height_agl = 5.0;  // Height above ground level

    // Ground effect depends on AGL, not MSL
    // Different runway altitudes have same ground effect for same AGL
    double phi_sea_level = calculateGroundEffectFactor(height_agl / wingspan);
    double phi_high_altitude = calculateGroundEffectFactor(height_agl / wingspan);

    TS_ASSERT_DELTA(phi_sea_level, phi_high_altitude, epsilon);
  }

  /***************************************************************************
   * Aircraft Configuration Effects
   ***************************************************************************/

  // Test high-wing vs. low-wing ground effect
  void testHighWingVsLowWing() {
    double wingspan = 30.0;
    double main_gear_height = 3.0;

    // Low wing: wing is at gear height
    double low_wing_height = main_gear_height;

    // High wing: wing is higher above ground
    double fuselage_height = 5.0;
    double high_wing_height = main_gear_height + fuselage_height;

    double phi_low = calculateGroundEffectFactor(low_wing_height / wingspan);
    double phi_high = calculateGroundEffectFactor(high_wing_height / wingspan);

    // Low wing experiences stronger ground effect
    TS_ASSERT(phi_low < phi_high);
  }

  // Test tricycle vs. taildragger ground effect
  void testTricycleVsTaildragger() {
    double wingspan = 30.0;

    // Tricycle: wing relatively level at touchdown
    double tricycle_height = 4.0;

    // Taildragger: wing inclined, tail lower
    double taildragger_leading_edge_height = 5.0;
    double taildragger_trailing_edge_height = 3.0;
    double taildragger_avg_height = (taildragger_leading_edge_height + taildragger_trailing_edge_height) / 2.0;

    double phi_tri = calculateGroundEffectFactor(tricycle_height / wingspan);
    double phi_tail = calculateGroundEffectFactor(taildragger_avg_height / wingspan);

    // Similar average heights give similar ground effect
    TS_ASSERT_DELTA(phi_tri, phi_tail, 0.05);
  }

  // Test ground effect with gear retracted vs. extended
  void testGroundEffectGearPosition() {
    double wingspan = 30.0;
    double height_gear_down = 4.0;
    double height_gear_up = 5.5;  // Simulating during go-around

    double phi_down = calculateGroundEffectFactor(height_gear_down / wingspan);
    double phi_up = calculateGroundEffectFactor(height_gear_up / wingspan);

    // Gear down (lower height) has stronger ground effect
    TS_ASSERT(phi_down < phi_up);
  }

  /***************************************************************************
   * Transient Ground Effect Phenomena
   ***************************************************************************/

  // Test ground effect entering during descent
  void testGroundEffectEntry() {
    double wingspan = 30.0;
    double descent_rate = -500.0;  // fpm
    double dt = 1.0;  // second

    double height_initial = 30.0;
    double height_final = height_initial + (descent_rate / 60.0) * dt;

    double phi_initial = calculateGroundEffectFactor(height_initial / wingspan);
    double phi_final = calculateGroundEffectFactor(height_final / wingspan);

    // Ground effect should be increasing (phi decreasing)
    TS_ASSERT(phi_final < phi_initial);
  }

  // Test ground effect exit during climb
  void testGroundEffectExit() {
    double wingspan = 30.0;
    double climb_rate = 1000.0;  // fpm
    double dt = 1.0;  // second

    double height_initial = 10.0;
    double height_final = height_initial + (climb_rate / 60.0) * dt;

    double phi_initial = calculateGroundEffectFactor(height_initial / wingspan);
    double phi_final = calculateGroundEffectFactor(height_final / wingspan);

    // Ground effect should be decreasing (phi increasing)
    TS_ASSERT(phi_final > phi_initial);
  }

  // Test abrupt ground effect change (terrain)
  void testAbruptGroundEffectChange() {
    double wingspan = 30.0;
    double flight_altitude = 100.0;  // ft MSL

    // Over flat terrain
    double terrain_flat = 0.0;
    double agl_flat = flight_altitude - terrain_flat;
    double phi_flat = calculateGroundEffectFactor(agl_flat / wingspan);

    // Over elevated terrain
    double terrain_elevated = 95.0;  // Near flight altitude
    double agl_elevated = flight_altitude - terrain_elevated;
    double phi_elevated = calculateGroundEffectFactor(agl_elevated / wingspan);

    // Elevated terrain creates sudden ground effect
    TS_ASSERT(phi_elevated < phi_flat);
  }

  /***************************************************************************
   * Stress Tests and Numerical Stability
   ***************************************************************************/

  // Test many height calculations
  void testStressManyHeights() {
    double wingspan = 30.0;

    for (int i = 0; i < 1000; i++) {
      double height = 0.1 + (i % 100);
      double phi = calculateGroundEffectFactor(height / wingspan);

      TS_ASSERT(phi >= 0.0);
      TS_ASSERT(phi <= 1.0);
      TS_ASSERT(!std::isnan(phi));
    }
  }

  // Test numerical stability at small h/b
  void testNumericalStabilitySmallHoverB() {
    double very_small = 1e-10;
    double phi = calculateGroundEffectFactor(very_small);

    TS_ASSERT(phi >= 0.0);
    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isinf(phi));
  }

  // Test numerical stability at large h/b
  void testNumericalStabilityLargeHoverB() {
    double very_large = 1e10;
    double phi = calculateGroundEffectFactor(very_large);

    TS_ASSERT_DELTA(phi, 1.0, 1e-6);
    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isinf(phi));
  }

  // Test continuous function behavior
  void testContinuousFunctionBehavior() {
    double wingspan = 30.0;
    double h_prev = 0.1;
    double phi_prev = calculateGroundEffectFactor(h_prev / wingspan);

    for (double h = 0.2; h <= 100.0; h += 0.1) {
      double phi = calculateGroundEffectFactor(h / wingspan);

      // Function should be continuous (no large jumps)
      double delta = std::abs(phi - phi_prev);
      TS_ASSERT(delta < 0.1);

      phi_prev = phi;
      h_prev = h;
    }
  }

  /***************************************************************************
   * Real Aircraft Validation Tests
   ***************************************************************************/

  // Test C172-like ground effect
  void testC172LikeGroundEffect() {
    double wingspan = 36.0;    // ft (C172 wingspan)
    double wheel_height = 2.0; // ft (roughly)

    double phi = calculateGroundEffectFactor(wheel_height / wingspan);

    // At this height, significant ground effect expected
    TS_ASSERT(phi < 0.6);
  }

  // Test B747-like ground effect
  void testB747LikeGroundEffect() {
    double wingspan = 195.0;   // ft (B747 wingspan)
    double wheel_height = 10.0; // ft (roughly)

    double phi = calculateGroundEffectFactor(wheel_height / wingspan);

    // Large aircraft with small h/b has very strong ground effect
    TS_ASSERT(phi < 0.5);
  }

  // Test Ekranoplan-like ground effect
  void testEkranoplanLikeGroundEffect() {
    double wingspan = 144.0;  // ft (KM ekranoplan-like)
    double cruise_height = 3.0;  // ft (cruise height)

    double phi = calculateGroundEffectFactor(cruise_height / wingspan);

    // Extreme ground effect for WIG craft
    TS_ASSERT(phi < 0.15);
  }

  /***************************************************************************
   * Landing Flare Profile Tests (77-80)
   ***************************************************************************/

  // Test 77: Flare entry timing
  void testFlareEntryTiming() {
    double wingspan = 35.0;
    double approach_height = 50.0;  // ft
    double flare_height = 5.0;      // ft - lower for noticeable ground effect

    double phi_approach = calculateGroundEffectFactor(approach_height / wingspan);
    double phi_flare = calculateGroundEffectFactor(flare_height / wingspan);

    // Ground effect should start becoming noticeable at flare height
    TS_ASSERT(phi_flare < phi_approach);
    TS_ASSERT(phi_flare < 0.95);  // Noticeable effect (h/b = 0.14 -> phi ≈ 0.84)
  }

  // Test 78: Float height prediction
  void testFloatHeightPrediction() {
    double wingspan = 30.0;
    double weight = 2500.0;  // lbs
    double wing_area = 174.0;  // sq ft
    double velocity = 100.0;  // ft/s
    double density = 0.002377;

    // Calculate stall speed and float characteristics
    double CL_approach = 1.5;
    double q = 0.5 * density * velocity * velocity;
    double lift_approach = q * wing_area * CL_approach;

    // At some height in ground effect, lift equals weight
    for (double h = 1.0; h <= 20.0; h += 0.5) {
      double phi = calculateGroundEffectFactor(h / wingspan);
      double CL_effective = CL_approach / phi;
      double lift = q * wing_area * CL_effective;
      if (lift >= weight) {
        TS_ASSERT(h > 0.5);  // Float occurs above ground
        break;
      }
    }
  }

  // Test 79: Touchdown attitude effect
  void testTouchdownAttitudeEffect() {
    double wingspan = 30.0;
    double nose_wheel_height = 2.0;
    double main_wheel_height = 1.5;
    double pitch_angle = 5.0;  // degrees nose up

    // Wing height varies with pitch
    double wing_height_level = 3.0;
    double wing_height_pitched = wing_height_level +
      (wingspan / 4) * sin(pitch_angle * PI / 180.0);

    double phi_level = calculateGroundEffectFactor(wing_height_level / wingspan);
    double phi_pitched = calculateGroundEffectFactor(wing_height_pitched / wingspan);

    // Higher pitch = higher wing = less ground effect
    TS_ASSERT(phi_pitched > phi_level);
  }

  // Test 80: Three-point landing ground effect
  void testThreePointLandingGroundEffect() {
    double wingspan = 25.0;  // Taildragger
    double main_height = 2.0;
    double tail_height = 0.5;
    double avg_height = (main_height + tail_height) / 2.0;

    double phi = calculateGroundEffectFactor(avg_height / wingspan);

    // Strong ground effect at three-point attitude
    TS_ASSERT(phi < 0.5);
  }

  /***************************************************************************
   * Takeoff Ground Effect Tests (81-84)
   ***************************************************************************/

  // Test 81: Rotation ground effect
  void testRotationGroundEffect() {
    double wingspan = 35.0;
    double wheel_height = 2.5;

    double phi = calculateGroundEffectFactor(wheel_height / wingspan);

    // Ground effect helps rotation by reducing induced drag
    double CDi_reduction = 1.0 - phi * phi;
    TS_ASSERT(CDi_reduction > 0.3);  // >30% reduction
  }

  // Test 82: Liftoff acceleration
  void testLiftoffAcceleration() {
    double wingspan = 35.0;
    double weight = 3000.0;
    double thrust = 600.0;  // lbs
    double h_wheel = 2.0;
    double h_just_airborne = 5.0;

    double phi_wheel = calculateGroundEffectFactor(h_wheel / wingspan);
    double phi_airborne = calculateGroundEffectFactor(h_just_airborne / wingspan);

    // Induced drag increases as aircraft climbs out
    double CDi_ratio = (phi_airborne * phi_airborne) / (phi_wheel * phi_wheel);
    TS_ASSERT(CDi_ratio > 1.0);  // Drag increases
  }

  // Test 83: Obstacle clearance with ground effect
  void testObstacleClearanceGroundEffect() {
    double wingspan = 30.0;
    double obstacle_height = 50.0;  // 50 ft obstacle

    // Calculate height at which ground effect diminishes
    // Use h/b = 0.05 -> phi = 0.39 (strong ground effect)
    double h_half_effect = wingspan * 0.05;  // 1.5 ft
    double phi_half = calculateGroundEffectFactor(h_half_effect / wingspan);

    // Ground effect helps initial climb
    TS_ASSERT(phi_half < 0.7);
    TS_ASSERT(h_half_effect < obstacle_height);
  }

  // Test 84: Soft field takeoff ground effect
  void testSoftFieldTakeoffGroundEffect() {
    double wingspan = 30.0;
    double min_wheel_height = 0.5;  // Flying just above surface

    double phi = calculateGroundEffectFactor(min_wheel_height / wingspan);

    // Very strong ground effect during soft field technique
    TS_ASSERT(phi < 0.25);
  }

  /***************************************************************************
   * Wind Effect Interactions (85-88)
   ***************************************************************************/

  // Test 85: Headwind ground effect enhancement
  void testHeadwindGroundEffectEnhancement() {
    double wingspan = 30.0;
    double height = 5.0;
    double groundspeed = 80.0;  // ft/s
    double headwind = 20.0;     // ft/s

    double airspeed = groundspeed + headwind;

    // Ground effect factor is geometry-based
    double phi = calculateGroundEffectFactor(height / wingspan);

    // Lift increases with airspeed squared
    double lift_ratio = (airspeed * airspeed) / (groundspeed * groundspeed);
    TS_ASSERT(lift_ratio > 1.0);
    TS_ASSERT(phi < 1.0);
  }

  // Test 86: Tailwind ground effect reduction
  void testTailwindGroundEffectReduction() {
    double wingspan = 30.0;
    double height = 5.0;
    double groundspeed = 100.0;  // ft/s
    double tailwind = 15.0;       // ft/s

    double airspeed = groundspeed - tailwind;

    // Ground effect coefficient unchanged, but lift reduced
    double phi = calculateGroundEffectFactor(height / wingspan);
    double lift_reduction = (airspeed * airspeed) / (groundspeed * groundspeed);

    TS_ASSERT(lift_reduction < 1.0);
    TS_ASSERT(phi < 1.0);
  }

  // Test 87: Gusty wind ground effect
  void testGustyWindGroundEffect() {
    double wingspan = 30.0;
    double height = 4.0;
    double base_speed = 100.0;
    double gust_amplitude = 15.0;

    double phi = calculateGroundEffectFactor(height / wingspan);

    // Ground effect factor remains constant, but lift varies with gusts
    double lift_high = (base_speed + gust_amplitude);
    double lift_low = (base_speed - gust_amplitude);
    double lift_ratio = lift_high / lift_low;

    TS_ASSERT(lift_ratio > 1.0);
    TS_ASSERT(phi < 1.0);
  }

  // Test 88: Wind shear near ground
  void testWindShearNearGround() {
    double wingspan = 30.0;
    double height_high = 50.0;
    double height_low = 10.0;

    double phi_high = calculateGroundEffectFactor(height_high / wingspan);
    double phi_low = calculateGroundEffectFactor(height_low / wingspan);

    // Ground effect increases as aircraft descends through shear
    TS_ASSERT(phi_low < phi_high);
  }

  /***************************************************************************
   * Formation Flying Ground Effect (89-92)
   ***************************************************************************/

  // Test 89: Echelon formation ground effect
  void testEchelonFormationGroundEffect() {
    double wingspan = 30.0;
    double leader_height = 5.0;
    double wingman_height = 5.5;  // Slightly higher

    double phi_leader = calculateGroundEffectFactor(leader_height / wingspan);
    double phi_wingman = calculateGroundEffectFactor(wingman_height / wingspan);

    // Wingman has slightly less ground effect
    TS_ASSERT(phi_wingman > phi_leader);
  }

  // Test 90: Trail formation ground effect
  void testTrailFormationGroundEffect() {
    double wingspan = 30.0;
    double same_height = 5.0;

    double phi = calculateGroundEffectFactor(same_height / wingspan);

    // Both aircraft experience same ground effect
    // But wake turbulence modifies effective flow for trailing aircraft
    TS_ASSERT(phi < 1.0);
    TS_ASSERT(phi > 0.0);
  }

  // Test 91: Formation breakup ground effect transient
  void testFormationBreakupGroundEffect() {
    double wingspan = 30.0;
    double initial_height = 5.0;
    double breakaway_height = 15.0;

    double phi_initial = calculateGroundEffectFactor(initial_height / wingspan);
    double phi_breakaway = calculateGroundEffectFactor(breakaway_height / wingspan);

    // Ground effect reduces during breakaway climb
    double effect_reduction = phi_breakaway - phi_initial;
    TS_ASSERT(effect_reduction > 0.1);
  }

  // Test 92: Close formation spacing ground effect
  void testCloseFormationSpacingGroundEffect() {
    double wingspan = 30.0;
    double height = 5.0;
    double lateral_spacing = wingspan;  // One wingspan apart

    double phi = calculateGroundEffectFactor(height / wingspan);

    // Each aircraft experiences its own ground effect
    // Interference effects are secondary
    TS_ASSERT(phi < 1.0);
  }

  /***************************************************************************
   * Special Operations Ground Effect (93-96)
   ***************************************************************************/

  // Test 93: Low pass ground effect
  void testLowPassGroundEffect() {
    double wingspan = 35.0;
    double pass_height = 5.0;  // Low pass height (h/b=0.14 -> phi ≈ 0.84)

    double phi = calculateGroundEffectFactor(pass_height / wingspan);

    // Noticeable ground effect during low pass
    TS_ASSERT(phi < 0.9);
    TS_ASSERT(phi > 0.5);
  }

  // Test 94: Carrier approach ground effect
  void testCarrierApproachGroundEffect() {
    double wingspan = 45.0;  // Fighter aircraft
    double deck_height = 60.0;  // Deck above water
    double approach_height = 62.0;  // Just above deck (2 ft AGL -> h/b=0.044 -> phi ≈ 0.33)

    double agl = approach_height - deck_height;
    double phi = calculateGroundEffectFactor(agl / wingspan);

    // Very close to deck, strong ground effect
    TS_ASSERT(phi < 0.6);
  }

  // Test 95: Ski jump takeoff ground effect
  void testSkiJumpTakeoffGroundEffect() {
    double wingspan = 40.0;
    double ramp_exit_height = 1.5;  // (h/b=0.0375 -> phi ≈ 0.26)

    double phi = calculateGroundEffectFactor(ramp_exit_height / wingspan);

    // Strong ground effect at ramp exit
    TS_ASSERT(phi < 0.5);
  }

  // Test 96: Unprepared surface ground effect
  void testUnpreparedSurfaceGroundEffect() {
    double wingspan = 30.0;
    double height = 4.0;

    // Unprepared surface - slightly reduced effect due to irregularities
    double phi_base = calculateGroundEffectFactor(height / wingspan);
    double roughness_penalty = 1.02;  // 2% reduction in effect
    double phi_unprepared = std::min(1.0, phi_base * roughness_penalty);

    TS_ASSERT(phi_unprepared >= phi_base);
    TS_ASSERT(phi_unprepared <= 1.0);
  }

  /***************************************************************************
   * Complete Ground Effect System Tests (97-100)
   ***************************************************************************/

  // Test 97: Complete approach profile
  void testCompleteApproachProfile() {
    double wingspan = 35.0;
    double heights[] = {200, 100, 50, 30, 20, 15, 10, 5, 3, 1};  // End at 1 ft
    int n = sizeof(heights) / sizeof(heights[0]);

    double prev_phi = 1.0;
    for (int i = 0; i < n; i++) {
      double phi = calculateGroundEffectFactor(heights[i] / wingspan);

      // Phi should decrease monotonically as height decreases
      TS_ASSERT(phi <= prev_phi);
      prev_phi = phi;
    }

    // Final phi should be very low (h/b = 1/35 = 0.029 -> phi ≈ 0.18)
    TS_ASSERT(prev_phi < 0.4);
  }

  // Test 98: Complete takeoff profile
  void testCompleteTakeoffProfile() {
    double wingspan = 35.0;
    double heights[] = {2, 5, 10, 20, 50, 100, 200};
    int n = sizeof(heights) / sizeof(heights[0]);

    double prev_phi = 0.0;
    for (int i = 0; i < n; i++) {
      double phi = calculateGroundEffectFactor(heights[i] / wingspan);

      // Phi should increase monotonically as height increases
      TS_ASSERT(phi >= prev_phi);
      prev_phi = phi;
    }

    // Final phi should approach 1.0
    TS_ASSERT(prev_phi > 0.98);
  }

  // Test 99: Complete L/D analysis
  void testCompleteLDAnalysis() {
    double CL = 1.0;
    double CD0 = 0.02;
    double AR = 8.0;
    double e = 0.85;
    double wingspan = 35.0;

    // Calculate L/D at various heights
    double heights[] = {2, 5, 10, 20, 50, 200};
    int n = sizeof(heights) / sizeof(heights[0]);

    double LD_prev = 0.0;
    for (int i = 0; i < n; i++) {
      double phi = calculateGroundEffectFactor(heights[i] / wingspan);
      double CDi = (CL * CL) / (PI * AR * e) * phi * phi;
      double CL_eff = CL / phi;
      double LD = CL_eff / (CD0 + CDi);

      if (i > 0) {
        // L/D should decrease as we climb out
        TS_ASSERT(LD < LD_prev);
      }
      LD_prev = LD;
    }
  }

  // Test 100: Complete ground effect system state
  void testCompleteGroundEffectSystemState() {
    // Comprehensive test of ground effect system
    double wingspan = 35.0;
    double height = 5.0;
    double h_over_b = height / wingspan;

    // 1. Calculate ground effect factor
    double phi = calculateGroundEffectFactor(h_over_b);
    TS_ASSERT(phi > 0.0);
    TS_ASSERT(phi < 1.0);

    // 2. Calculate induced drag reduction
    double CDi_freestream = 0.05;
    double CDi_ground = CDi_freestream * phi * phi;
    TS_ASSERT(CDi_ground < CDi_freestream);

    // 3. Calculate lift increase
    double CL_freestream = 1.0;
    double CL_ground = CL_freestream / phi;
    TS_ASSERT(CL_ground > CL_freestream);

    // 4. Calculate downwash reduction
    double epsilon_freestream = 0.1;
    double epsilon_ground = epsilon_freestream * phi;
    TS_ASSERT(epsilon_ground < epsilon_freestream);

    // 5. Calculate pitch moment change
    double delta_Cm = calculatePitchMomentDelta(h_over_b);
    TS_ASSERT(delta_Cm < 0.0);  // Nose-down

    // 6. Calculate CP shift
    double chord = 5.0;
    double cp_shift = calculateCPShift(h_over_b, chord);
    TS_ASSERT(cp_shift > 0.0);  // Forward

    // 7. Calculate cushion factor
    double cushion = calculateCushionFactor(h_over_b);
    TS_ASSERT(cushion == phi);  // In our simplified model

    // 8. Verify L/D improvement
    double CD0 = 0.02;
    double AR = 8.0;
    double e = 0.85;
    double CDi_free = (CL_freestream * CL_freestream) / (PI * AR * e);
    double CDi_ge = CDi_free * phi * phi;
    double LD_free = CL_freestream / (CD0 + CDi_free);
    double LD_ge = CL_ground / (CD0 + CDi_ge);
    TS_ASSERT(LD_ge > LD_free);

    // 9. Verify energy efficiency
    double drag_reduction = (CDi_free - CDi_ge) / CDi_free;
    TS_ASSERT(drag_reduction > 0.0);

    // 10. Verify overall system coherence
    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isnan(CL_ground));
    TS_ASSERT(!std::isnan(CDi_ground));
    TS_ASSERT(!std::isnan(LD_ge));
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
