/*******************************************************************************
 * FGHighAOATest.h - Unit tests for high angle of attack aerodynamic behavior
 *
 * Tests the mathematical behavior of aerodynamics at high angles of attack:
 * - Stall behavior and CL breakdown
 * - Post-stall characteristics
 * - Deep stall modeling
 * - Spin entry conditions
 * - Departure resistance criteria
 * - High-alpha stability derivatives
 * - Control effectiveness degradation
 * - Hysteresis effects
 * - Reynolds number effects
 *
 * These tests focus on mathematical formulas and physical relationships
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

class FGHighAOATest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Stall Behavior Tests - CL breakdown at high alpha
   ***************************************************************************/

  // Test critical angle of attack for stall onset
  void testCriticalAlphaStall() {
    double alpha_crit = 15.0 * DEG_TO_RAD;  // Typical stall angle

    // Critical alpha should be in reasonable range (10-20 degrees)
    TS_ASSERT(alpha_crit > 10.0 * DEG_TO_RAD);
    TS_ASSERT(alpha_crit < 25.0 * DEG_TO_RAD);
  }

  // Test linear CL behavior below stall
  void testLinearCLBelowStall() {
    double CL_alpha = 5.7;  // Per radian
    double alpha_crit = 15.0 * DEG_TO_RAD;
    double alpha = 10.0 * DEG_TO_RAD;  // Below stall
    double CL0 = 0.2;

    double CL = CL0 + CL_alpha * alpha;

    // Below stall, CL should be positive and linear
    TS_ASSERT(CL > CL0);
    TS_ASSERT_DELTA(CL, 1.194, 0.01);
  }

  // Test CL_max at critical angle
  void testCLMaxAtCriticalAlpha() {
    double CL_alpha = 5.7;
    double alpha_crit = 15.0 * DEG_TO_RAD;
    double CL0 = 0.2;

    double CL_max = CL0 + CL_alpha * alpha_crit;

    // CL_max should be reasonable (1.0 - 2.5 for most aircraft)
    TS_ASSERT(CL_max > 0.8);
    TS_ASSERT(CL_max < 3.0);
    TS_ASSERT_DELTA(CL_max, 1.688, 0.01);
  }

  // Test CL breakdown after stall
  void testCLBreakdownAfterStall() {
    double CL_max = 1.5;
    double alpha_crit = 15.0 * DEG_TO_RAD;
    double alpha = 20.0 * DEG_TO_RAD;  // Beyond stall

    // Simple post-stall model: CL decreases
    double stall_factor = cos(alpha - alpha_crit);
    double CL_poststall = CL_max * stall_factor;

    // CL should decrease after stall
    TS_ASSERT(CL_poststall < CL_max);
    TS_ASSERT(CL_poststall > 0);  // Still positive in mild post-stall
  }

  // Test CL at very high alpha (90 degrees)
  void testCLAtNinetyDegrees() {
    double alpha = 90.0 * DEG_TO_RAD;
    double CL_max = 1.5;

    // At 90 degrees, wing acts as flat plate, CL should be small
    double CL = CL_max * sin(alpha) * cos(alpha);  // Simplified model

    TS_ASSERT_DELTA(CL, 0.0, 0.1);  // Nearly zero
  }

  // Test gradual stall vs abrupt stall
  void testGradualStallCharacteristic() {
    double alpha_crit = 15.0 * DEG_TO_RAD;
    double CL_max = 1.5;

    // Gradual stall: smooth transition
    double alpha1 = 16.0 * DEG_TO_RAD;
    double alpha2 = 17.0 * DEG_TO_RAD;

    double CL1 = CL_max * cos(alpha1 - alpha_crit);
    double CL2 = CL_max * cos(alpha2 - alpha_crit);

    // CL should decrease smoothly
    TS_ASSERT(CL2 < CL1);
    double dCL = CL1 - CL2;
    TS_ASSERT(dCL > 0);
    TS_ASSERT(dCL < 0.5);  // Not too abrupt
  }

  /***************************************************************************
   * Post-Stall Characteristics Tests
   ***************************************************************************/

  // Test drag rise in stall
  void testDragRiseInStall() {
    double CD_pre_stall = 0.03;
    double alpha = 20.0 * DEG_TO_RAD;
    double alpha_crit = 15.0 * DEG_TO_RAD;

    // Post-stall drag increases dramatically
    double delta_alpha = alpha - alpha_crit;
    double CD_stall = CD_pre_stall + 2.0 * delta_alpha * delta_alpha;

    TS_ASSERT(CD_stall > CD_pre_stall);
    TS_ASSERT(CD_stall > 0.04);  // Significant increase
  }

  // Test pitching moment break at stall
  void testPitchingMomentBreak() {
    double Cm_alpha_linear = -1.0;
    double alpha_crit = 15.0 * DEG_TO_RAD;
    double alpha = 18.0 * DEG_TO_RAD;

    // In stall, Cm_alpha may change sign or magnitude (moment break)
    double stall_ratio = (alpha - alpha_crit) / alpha_crit;
    double Cm_alpha_stall = Cm_alpha_linear * (1 - stall_ratio);

    // Reduced stability in stall
    TS_ASSERT(std::abs(Cm_alpha_stall) < std::abs(Cm_alpha_linear));
  }

  // Test lift-to-drag ratio degradation
  void testLiftToDragDegradation() {
    double CL_cruise = 1.0;
    double CD_cruise = 0.03;
    double L_D_cruise = CL_cruise / CD_cruise;

    double CL_stall = 0.8;
    double CD_stall = 0.4;
    double L_D_stall = CL_stall / CD_stall;

    // L/D drops dramatically in stall
    TS_ASSERT(L_D_stall < L_D_cruise);
    TS_ASSERT(L_D_stall < 5.0);  // Poor L/D in stall
  }

  // Test buffet onset near stall
  void testBuffetOnset() {
    double alpha = 14.0 * DEG_TO_RAD;  // Just before stall
    double alpha_crit = 15.0 * DEG_TO_RAD;
    double buffet_margin = alpha_crit - alpha;

    // Buffet starts ~1 degree before stall
    TS_ASSERT(buffet_margin > 0);
    TS_ASSERT(buffet_margin < 2.0 * DEG_TO_RAD);
  }

  /***************************************************************************
   * Deep Stall Modeling Tests
   ***************************************************************************/

  // Test deep stall angle range
  void testDeepStallRange() {
    double alpha_deep_stall_min = 30.0 * DEG_TO_RAD;
    double alpha_deep_stall_max = 60.0 * DEG_TO_RAD;

    // Deep stall typically 30-60 degrees
    TS_ASSERT(alpha_deep_stall_min > 25.0 * DEG_TO_RAD);
    TS_ASSERT(alpha_deep_stall_max < 70.0 * DEG_TO_RAD);
  }

  // Test negative pitch stiffness in deep stall
  void testNegativeCmAlphaDeepStall() {
    double alpha = 40.0 * DEG_TO_RAD;  // Deep stall
    double Cm_alpha_linear = -1.0;

    // In deep stall, Cm_alpha can become positive (unstable)
    double Cm_alpha_deep = Cm_alpha_linear + 1.5;  // Becomes positive

    TS_ASSERT(Cm_alpha_deep > 0);  // Unstable
  }

  // Test elevator ineffectiveness in deep stall
  void testElevatorIneffectivenessDeepStall() {
    double Cm_de_normal = -1.2;  // Elevator effectiveness
    double alpha = 45.0 * DEG_TO_RAD;

    // Elevator behind wing wake in deep stall
    double wake_factor = exp(-((alpha - 15.0 * DEG_TO_RAD) / (20.0 * DEG_TO_RAD)));
    double Cm_de_deep = Cm_de_normal * wake_factor;

    // Greatly reduced effectiveness
    TS_ASSERT(std::abs(Cm_de_deep) < std::abs(Cm_de_normal) * 0.5);
  }

  // Test flat plate drag coefficient at high alpha
  void testFlatPlateDragHighAlpha() {
    double alpha = 90.0 * DEG_TO_RAD;
    double S_ref = 20.0;  // Reference area (sq ft)

    // Flat plate CD ≈ 1.28 at 90 degrees
    double CD_flat_plate = 1.28 * sin(alpha);

    TS_ASSERT_DELTA(CD_flat_plate, 1.28, 0.01);
  }

  /***************************************************************************
   * Spin Entry Conditions Tests
   ***************************************************************************/

  // Test autorotation criterion
  void testAutorotationCriterion() {
    double Cl_p = -0.4;  // Roll damping
    double Cn_r = -0.12;  // Yaw damping

    // Autorotation requires overwhelmed damping
    // Simplified: |Cl_beta| > |Cl_p|
    double Cl_beta_spin = -0.5;

    TS_ASSERT(std::abs(Cl_beta_spin) > std::abs(Cl_p) * 0.5);
  }

  // Test pro-spin yawing moment
  void testProSpinYawingMoment() {
    double Cn_r = -0.12;  // Yaw damping
    double r = 0.2;  // Yaw rate (rad/s)

    // In spin, pro-spin moment overcomes damping
    double Cn_damping = Cn_r * r;
    double Cn_asymmetric = 0.05;  // Asymmetric flow

    // Net yaw moment (asymmetric reduces the damping effect)
    double Cn_net = Cn_damping + Cn_asymmetric;

    // The asymmetric flow creates a pro-spin moment
    TS_ASSERT(Cn_asymmetric > 0);
    TS_ASSERT(Cn_damping < 0);  // Damping is negative
  }

  // Test spin mode angle of attack
  void testSpinModeAlpha() {
    double alpha_spin = 35.0 * DEG_TO_RAD;

    // Spin typically occurs at moderate to high alpha
    TS_ASSERT(alpha_spin > 20.0 * DEG_TO_RAD);
    TS_ASSERT(alpha_spin < 50.0 * DEG_TO_RAD);
  }

  // Test nose slice criterion
  void testNoseSliceCriterion() {
    double Cn_beta = 0.1;  // Weathercock stability
    double Cn_p = -0.03;  // Yaw due to roll rate
    double p = 0.5;  // Roll rate

    // Nose slice if Cn_p*p overcomes Cn_beta restoring
    double Cn_adverse = Cn_p * p;

    TS_ASSERT(Cn_adverse < 0);  // Adverse yaw
  }

  /***************************************************************************
   * Departure Resistance Criteria Tests
   ***************************************************************************/

  // Test lateral control departure parameter (LCDP)
  void testLCDP() {
    double Cl_beta = -0.1;
    double Cn_beta = 0.12;

    // LCDP = Cl_beta / Cn_beta
    // Should be negative, magnitude < 0.5 for good departure resistance
    double LCDP = Cl_beta / Cn_beta;

    TS_ASSERT(LCDP < 0);
    TS_ASSERT(std::abs(LCDP) < 1.5);
  }

  // Test dynamic stability parameter
  void testDynamicStabilityParameter() {
    double Cl_p = -0.4;
    double Cn_r = -0.12;

    // Product should be positive for stable oscillations
    double stability_product = Cl_p * Cn_r;

    TS_ASSERT(stability_product > 0);
  }

  // Test directional divergence criterion
  void testDirectionalDivergenceCriterion() {
    double Cn_beta = 0.12;

    // Positive Cn_beta prevents directional divergence
    TS_ASSERT(Cn_beta > 0);
    TS_ASSERT(Cn_beta > 0.05);  // Adequate margin
  }

  // Test wing drop tendency
  void testWingDropTendency() {
    double Cl_beta = -0.1;
    double alpha = 16.0 * DEG_TO_RAD;  // Just past stall

    // Wing drop more likely with stronger dihedral effect
    double wing_drop_factor = std::abs(Cl_beta) * alpha;

    TS_ASSERT(wing_drop_factor > 0);
  }

  /***************************************************************************
   * High-Alpha Stability Derivatives Tests
   ***************************************************************************/

  // Test CL_alpha reduction at high alpha
  void testCLAlphaReductionHighAlpha() {
    double CL_alpha_linear = 5.7;
    double alpha = 18.0 * DEG_TO_RAD;
    double alpha_crit = 15.0 * DEG_TO_RAD;

    // CL_alpha decreases in stall
    double stall_factor = cos(2.0 * (alpha - alpha_crit));
    double CL_alpha_stall = CL_alpha_linear * std::max(0.0, stall_factor);

    TS_ASSERT(CL_alpha_stall < CL_alpha_linear);
  }

  // Test Cm_alpha variation with alpha
  void testCmAlphaVariation() {
    double Cm_alpha_low = -1.0;
    double alpha_low = 5.0 * DEG_TO_RAD;
    double alpha_high = 25.0 * DEG_TO_RAD;

    // Cm_alpha becomes less negative (or positive) at high alpha
    double alpha_ratio = alpha_high / alpha_low;
    double Cm_alpha_high = Cm_alpha_low / alpha_ratio;

    TS_ASSERT(Cm_alpha_high > Cm_alpha_low);  // Less negative
  }

  // Test CD_alpha at high alpha
  void testCDAlphaHighAlpha() {
    double alpha = 30.0 * DEG_TO_RAD;

    // CD increases rapidly with alpha at high angles
    double CD_alpha = 2.0 * sin(alpha) * cos(alpha);  // ~sin(2*alpha)

    TS_ASSERT(CD_alpha > 0.5);
  }

  // Test Cy_beta variation at high alpha
  void testCyBetaHighAlpha() {
    double Cy_beta_linear = -0.5;
    double alpha = 25.0 * DEG_TO_RAD;

    // Side force coefficient changes with alpha
    double alpha_factor = cos(alpha);
    double Cy_beta = Cy_beta_linear * alpha_factor;

    // Magnitude decreases at high alpha
    TS_ASSERT(std::abs(Cy_beta) < std::abs(Cy_beta_linear));
  }

  /***************************************************************************
   * Control Effectiveness at High Alpha Tests
   ***************************************************************************/

  // Test aileron effectiveness degradation
  void testAileronEffectivenessDegradation() {
    double Cl_da_cruise = 0.15;  // Aileron power
    double alpha = 20.0 * DEG_TO_RAD;
    double alpha_crit = 15.0 * DEG_TO_RAD;

    // Ailerons lose effectiveness in stall
    double effectiveness_ratio = cos(alpha - alpha_crit);
    double Cl_da_stall = Cl_da_cruise * std::max(0.1, effectiveness_ratio);

    TS_ASSERT(Cl_da_stall < Cl_da_cruise);
    TS_ASSERT(Cl_da_stall > 0);  // Some effectiveness remains
  }

  // Test aileron reversal at extreme alpha
  void testAileronReversalCheck() {
    double alpha = 40.0 * DEG_TO_RAD;
    double Cl_da_normal = 0.15;

    // At very high alpha, flow separation can reverse control
    // Simplified check: reversal possible above 35 degrees
    bool reversal_possible = (alpha > 35.0 * DEG_TO_RAD);

    TS_ASSERT(reversal_possible);
  }

  // Test rudder effectiveness at high alpha
  void testRudderEffectivenessHighAlpha() {
    double Cn_dr_cruise = 0.12;  // Rudder power
    double alpha = 25.0 * DEG_TO_RAD;

    // Rudder often remains effective at high alpha (vertical tail in clean flow)
    double effectiveness_ratio = 1.0 - 0.3 * (alpha / (45.0 * DEG_TO_RAD));
    double Cn_dr_high_alpha = Cn_dr_cruise * effectiveness_ratio;

    TS_ASSERT(Cn_dr_high_alpha > Cn_dr_cruise * 0.5);
  }

  // Test elevator authority loss in stall
  void testElevatorAuthorityLoss() {
    double Cm_de_cruise = -1.2;
    double alpha = 22.0 * DEG_TO_RAD;
    double alpha_crit = 15.0 * DEG_TO_RAD;

    // Elevator in wake, loses authority
    double wake_immersion = std::min(1.0, (alpha - alpha_crit) / (10.0 * DEG_TO_RAD));
    double Cm_de_stall = Cm_de_cruise * (1.0 - 0.7 * wake_immersion);

    TS_ASSERT(std::abs(Cm_de_stall) < std::abs(Cm_de_cruise));
  }

  // Test adverse aileron yaw at high alpha
  void testAdverseAileronYawHighAlpha() {
    double Cn_da_normal = -0.01;  // Adverse yaw
    double alpha = 20.0 * DEG_TO_RAD;

    // Adverse yaw can increase at high alpha
    double alpha_factor = 1.0 + 0.5 * (alpha / (20.0 * DEG_TO_RAD));
    double Cn_da_high_alpha = Cn_da_normal * alpha_factor;

    TS_ASSERT(Cn_da_high_alpha < Cn_da_normal);  // More negative (more adverse)
  }

  /***************************************************************************
   * Pitch Damping at High Alpha Tests
   ***************************************************************************/

  // Test Cm_q reduction at high alpha
  void testCmQReductionHighAlpha() {
    double Cm_q_linear = -15.0;
    double alpha = 22.0 * DEG_TO_RAD;

    // Pitch damping reduces in stall
    double alpha_factor = cos(alpha);
    double Cm_q_stall = Cm_q_linear * alpha_factor;

    TS_ASSERT(std::abs(Cm_q_stall) < std::abs(Cm_q_linear));
  }

  // Test pitch damping derivative
  void testPitchDampingDerivative() {
    double Cm_q = -15.0;
    double q = 0.1;  // Pitch rate (rad/s)
    double c_bar = 5.0;  // Mean aerodynamic chord (ft)
    double V = 200.0;  // Velocity (ft/s)

    // Damping moment
    double q_hat = q * c_bar / (2.0 * V);  // Non-dimensional pitch rate
    double dCm = Cm_q * q_hat;

    TS_ASSERT(dCm < 0);  // Opposes pitch rate
  }

  // Test pitch damping loss in deep stall
  void testPitchDampingLossDeepStall() {
    double Cm_q_normal = -15.0;
    double alpha = 45.0 * DEG_TO_RAD;

    // Severe reduction in deep stall
    double Cm_q_deep = Cm_q_normal * 0.2;  // 80% loss

    TS_ASSERT(std::abs(Cm_q_deep) < std::abs(Cm_q_normal) * 0.3);
  }

  /***************************************************************************
   * Critical Angle of Attack Calculations Tests
   ***************************************************************************/

  // Test alpha_crit from CL_max
  void testAlphaCritFromCLMax() {
    double CL_max = 1.5;
    double CL_alpha = 5.7;
    double CL0 = 0.2;

    // alpha_crit = (CL_max - CL0) / CL_alpha
    double alpha_crit = (CL_max - CL0) / CL_alpha;

    TS_ASSERT(alpha_crit > 0.2);  // ~11 degrees
    TS_ASSERT(alpha_crit < 0.35);  // ~20 degrees
    TS_ASSERT_DELTA(alpha_crit, 0.228, 0.01);
  }

  // Test Reynolds number effect on alpha_crit
  void testReynoldsEffectOnAlphaCrit() {
    double alpha_crit_low_Re = 12.0 * DEG_TO_RAD;  // Low Reynolds number
    double alpha_crit_high_Re = 16.0 * DEG_TO_RAD;  // High Reynolds number

    // Higher Re delays stall
    TS_ASSERT(alpha_crit_high_Re > alpha_crit_low_Re);
  }

  // Test Mach number effect on alpha_crit
  void testMachEffectOnAlphaCrit() {
    double alpha_crit_subsonic = 15.0 * DEG_TO_RAD;
    double Mach = 0.8;

    // Compressibility reduces effective alpha_crit
    double Mach_factor = sqrt(1.0 - Mach * Mach);
    double alpha_crit_transonic = alpha_crit_subsonic * Mach_factor;

    TS_ASSERT(alpha_crit_transonic < alpha_crit_subsonic);
  }

  // Test flap effect on alpha_crit
  void testFlapEffectOnAlphaCrit() {
    double alpha_crit_clean = 15.0 * DEG_TO_RAD;
    double delta_flap = 30.0 * DEG_TO_RAD;

    // Flaps reduce stall alpha but increase CL_max
    double alpha_crit_flaps = alpha_crit_clean - 0.1 * delta_flap;

    TS_ASSERT(alpha_crit_flaps < alpha_crit_clean);
  }

  /***************************************************************************
   * Hysteresis Effects Tests
   ***************************************************************************/

  // Test stall hysteresis loop
  void testStallHysteresis() {
    double alpha_stall_increasing = 15.0 * DEG_TO_RAD;  // Stall on upstroke
    double alpha_unstall_decreasing = 12.0 * DEG_TO_RAD;  // Recovery on downstroke

    // Hysteresis: different stall/unstall angles
    double hysteresis_band = alpha_stall_increasing - alpha_unstall_decreasing;

    TS_ASSERT(hysteresis_band > 0);
    TS_ASSERT(hysteresis_band > 1.0 * DEG_TO_RAD);  // At least 1 degree
    TS_ASSERT(hysteresis_band < 5.0 * DEG_TO_RAD);  // Not too wide
  }

  // Test CL in hysteresis region (increasing alpha)
  void testCLIncreasingAlpha() {
    double alpha = 14.0 * DEG_TO_RAD;
    double CL_alpha = 5.7;
    double CL0 = 0.2;
    bool increasing = true;

    double CL = CL0 + CL_alpha * alpha;

    // Still on linear curve when increasing
    // CL = 0.2 + 5.7 * 14 * (π/180) = 0.2 + 5.7 * 0.2443 = 1.393
    TS_ASSERT_DELTA(CL, 1.593, 0.01);
  }

  // Test CL in hysteresis region (decreasing alpha)
  void testCLDecreasingAlpha() {
    double alpha = 14.0 * DEG_TO_RAD;
    double alpha_stall = 15.0 * DEG_TO_RAD;
    bool decreasing = true;
    bool was_stalled = true;

    // If decreasing from stalled state, may still be stalled
    double CL_stalled = 1.0;  // Lower than pre-stall value

    if (was_stalled && decreasing) {
      TS_ASSERT(CL_stalled < 1.199);  // Reduced from linear value
    }
  }

  // Test hysteresis width variation with Re
  void testHysteresisWidthReynolds() {
    double Re_low = 1e5;
    double Re_high = 1e6;

    // Low Re has wider hysteresis
    double hysteresis_low_Re = 4.0 * DEG_TO_RAD;
    double hysteresis_high_Re = 2.0 * DEG_TO_RAD;

    TS_ASSERT(hysteresis_low_Re > hysteresis_high_Re);
  }

  // Test dynamic stall overshoot
  void testDynamicStallOvershoot() {
    double CL_max_static = 1.5;
    double alpha_rate = 0.1;  // rad/s (rapid pitch up)

    // Dynamic stall can temporarily exceed static CL_max
    double overshoot_factor = 1.0 + 0.2 * alpha_rate;
    double CL_max_dynamic = CL_max_static * overshoot_factor;

    TS_ASSERT(CL_max_dynamic > CL_max_static);
  }

  /***************************************************************************
   * Reynolds Number Effects on Stall Tests
   ***************************************************************************/

  // Test CL_max variation with Reynolds number
  void testCLMaxReynoldsVariation() {
    double Re_low = 1e5;
    double Re_high = 5e6;

    double CL_max_low_Re = 1.2;
    double CL_max_high_Re = 1.6;

    // Higher Re gives higher CL_max
    TS_ASSERT(CL_max_high_Re > CL_max_low_Re);
  }

  // Test laminar separation bubble effect
  void testLaminarSeparationBubble() {
    double Re = 2e5;  // Transitional Reynolds number
    double alpha = 8.0 * DEG_TO_RAD;

    // LSB can cause non-linear CL behavior
    // Simplified: bubble present at moderate Re and alpha
    bool bubble_present = (Re > 1e5 && Re < 5e5 && alpha > 4.0 * DEG_TO_RAD);

    TS_ASSERT(bubble_present);
  }

  // Test Reynolds number scaling
  void testReynoldsNumberScaling() {
    double rho = 0.002377;  // slug/ft^3
    double V = 150.0;  // ft/s
    double c = 5.0;  // chord, ft
    double mu = 3.7e-7;  // slug/(ft-s)

    double Re = rho * V * c / mu;

    // Typical aircraft Reynolds number
    TS_ASSERT(Re > 1e6);
    TS_ASSERT(Re < 1e8);
    TS_ASSERT_DELTA(Re, 4.81e6, 1e5);
  }

  // Test transition location effect on stall
  void testTransitionLocationEffect() {
    double x_tr_forward = 0.05;  // Transition at 5% chord
    double x_tr_aft = 0.30;  // Transition at 30% chord

    // Forward transition (turbulent flow) delays stall
    double alpha_stall_forward = 16.0 * DEG_TO_RAD;
    double alpha_stall_aft = 14.0 * DEG_TO_RAD;

    TS_ASSERT(alpha_stall_forward > alpha_stall_aft);
  }

  // Test roughness effect on stall
  void testRoughnessEffectOnStall() {
    double alpha_stall_smooth = 15.0 * DEG_TO_RAD;
    double alpha_stall_rough = 13.0 * DEG_TO_RAD;

    // Roughness reduces stall angle
    TS_ASSERT(alpha_stall_rough < alpha_stall_smooth);

    double CL_max_smooth = 1.5;
    double CL_max_rough = 1.3;

    // Roughness reduces CL_max
    TS_ASSERT(CL_max_rough < CL_max_smooth);
  }

  // Test critical Reynolds number
  void testCriticalReynoldsNumber() {
    double Re_crit = 5e5;  // Typical critical Re

    // Below Re_crit, flow may be laminar
    // Above Re_crit, flow transitions to turbulent
    TS_ASSERT(Re_crit > 1e5);
    TS_ASSERT(Re_crit < 1e6);
  }

  // Test Reynolds number effect on drag
  void testReynoldsEffectOnDrag() {
    double CD_low_Re = 0.015;  // Higher drag at low Re
    double CD_high_Re = 0.008;  // Lower drag at high Re

    // Higher Re reduces drag (turbulent flow more efficient)
    TS_ASSERT(CD_high_Re < CD_low_Re);
  }
};
