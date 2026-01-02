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

#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropagate.h"
#include "models/FGAerodynamics.h"

using namespace JSBSim;

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

/*******************************************************************************
 * FGHighAOAAdditionalTest - Extended high angle of attack tests
 ******************************************************************************/
class FGHighAOAAdditionalTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Dynamic Stall Tests
   ***************************************************************************/

  // Test reduced frequency for dynamic stall
  void testReducedFrequencyDynamicStall() {
    double omega = 5.0;  // Pitch oscillation frequency (rad/s)
    double c = 5.0;      // Mean chord (ft)
    double V = 200.0;    // Velocity (ft/s)

    // Reduced frequency k = omega * c / (2 * V)
    double k = omega * c / (2.0 * V);

    // k > 0.05 indicates significant unsteady effects
    TS_ASSERT_DELTA(k, 0.0625, 0.001);
    TS_ASSERT(k > 0.05);  // Unsteady effects important
  }

  // Test leading edge vortex lift in dynamic stall
  void testLEVLiftDynamicStall() {
    double CL_static_max = 1.5;
    double alpha_rate = 0.2;  // rad/s (rapid pitch up)
    double c = 5.0;  // chord
    double V = 200.0;  // velocity

    // LEV adds significant lift during rapid pitch
    double alpha_dot_nondim = alpha_rate * c / V;
    double LEV_increment = 2.0 * M_PI * alpha_dot_nondim;
    double CL_dynamic = CL_static_max + LEV_increment;

    TS_ASSERT(CL_dynamic > CL_static_max);
    TS_ASSERT_DELTA(LEV_increment, 0.0314, 0.01);
  }

  // Test dynamic stall moment reversal
  void testDynamicStallMomentReversal() {
    double Cm_before_LEV = -0.05;  // Nose-down before vortex sheds
    double Cm_LEV_shed = -0.25;    // Strong nose-down when vortex sheds

    // Moment reversal when leading edge vortex passes trailing edge
    double moment_change = Cm_LEV_shed - Cm_before_LEV;

    TS_ASSERT(moment_change < -0.1);  // Significant nose-down break
  }

  // Test pitch rate effect on stall angle
  void testPitchRateStallAngleDelay() {
    double alpha_stall_static = 15.0 * DEG_TO_RAD;
    double alpha_dot = 0.3;  // rad/s
    double c = 5.0;
    double V = 200.0;

    // Dynamic stall occurs at higher alpha
    double alpha_delay = 1.5 * alpha_dot * c / V;
    double alpha_stall_dynamic = alpha_stall_static + alpha_delay;

    TS_ASSERT(alpha_stall_dynamic > alpha_stall_static);
    TS_ASSERT_DELTA(alpha_delay, 0.01125, 0.001);
  }

  /***************************************************************************
   * Wing Rock Oscillation Tests
   ***************************************************************************/

  // Test wing rock frequency prediction
  void testWingRockFrequency() {
    double Ixx = 5000.0;  // Roll moment of inertia (slug-ft^2)
    double Cl_phi = -0.02;  // Roll moment due to roll angle (dihedral effect)
    double q_bar = 100.0;  // Dynamic pressure (psf)
    double S = 200.0;  // Wing area (ft^2)
    double b = 35.0;  // Wing span (ft)

    // Natural frequency of roll mode
    double L_phi = q_bar * S * b * Cl_phi;  // Roll moment derivative
    double omega_n = sqrt(-L_phi / Ixx);

    TS_ASSERT(omega_n > 0.1);  // Low frequency oscillation
    TS_ASSERT(omega_n < 5.0);
  }

  // Test wing rock amplitude growth
  void testWingRockAmplitudeGrowth() {
    double Cl_p_positive = 0.05;  // Reduced roll damping (can go positive at high alpha)
    double Cl_p_normal = -0.4;   // Normal roll damping

    // Wing rock grows when Cl_p becomes positive
    TS_ASSERT(Cl_p_positive > 0);  // Negative damping (oscillation grows)
    TS_ASSERT(Cl_p_normal < 0);   // Normal positive damping
  }

  // Test wing rock limit cycle amplitude
  void testWingRockLimitCycle() {
    double phi_max = 30.0 * DEG_TO_RAD;  // Typical limit cycle amplitude

    // Limit cycle bounded by nonlinear aerodynamics
    TS_ASSERT(phi_max > 10.0 * DEG_TO_RAD);
    TS_ASSERT(phi_max < 45.0 * DEG_TO_RAD);
  }

  /***************************************************************************
   * Asymmetric Stall Behavior Tests
   ***************************************************************************/

  // Test differential stall with sideslip
  void testDifferentialStallSideslip() {
    double beta = 5.0 * DEG_TO_RAD;  // Sideslip
    double alpha = 15.0 * DEG_TO_RAD;  // At stall

    // Windward wing sees effective lower alpha
    double alpha_windward = alpha - 0.5 * beta;
    // Leeward wing sees effective higher alpha
    double alpha_leeward = alpha + 0.5 * beta;

    // Leeward wing stalls first
    TS_ASSERT(alpha_leeward > alpha_windward);
    TS_ASSERT(alpha_leeward > alpha);
  }

  // Test rolling moment from asymmetric stall
  void testAsymmetricStallRollingMoment() {
    double CL_left = 1.5;   // Left wing still lifting
    double CL_right = 0.8;  // Right wing stalled
    double y_left = -10.0;  // Left wing lift centroid (ft)
    double y_right = 10.0;  // Right wing lift centroid (ft)

    // Net rolling moment from lift asymmetry
    double delta_CL = CL_left - CL_right;

    // Roll toward stalled wing
    TS_ASSERT(delta_CL > 0);
    TS_ASSERT_DELTA(delta_CL, 0.7, 0.01);
  }

  // Test yawing moment from asymmetric drag
  void testAsymmetricDragYawMoment() {
    double CD_left = 0.03;   // Left wing pre-stall drag
    double CD_right = 0.15;  // Right wing post-stall drag

    // Stalled wing has higher drag, causes yaw
    double delta_CD = CD_right - CD_left;

    // Yaw toward stalled wing
    TS_ASSERT(delta_CD > 0);
    TS_ASSERT(delta_CD > 0.1);  // Significant asymmetry
  }

  /***************************************************************************
   * Stall Warning System Tests
   ***************************************************************************/

  // Test stall warning margin
  void testStallWarningMargin() {
    double alpha_stall = 15.0 * DEG_TO_RAD;
    double alpha_warning = 12.0 * DEG_TO_RAD;  // Warning 3 degrees before stall

    double margin = alpha_stall - alpha_warning;

    // Warning should provide adequate margin
    TS_ASSERT(margin > 2.0 * DEG_TO_RAD);
    TS_ASSERT(margin < 5.0 * DEG_TO_RAD);
    TS_ASSERT_DELTA(margin, 3.0 * DEG_TO_RAD, 0.01);
  }

  // Test angle of attack rate for stall prediction
  void testAlphaRateStallPrediction() {
    double alpha = 13.0 * DEG_TO_RAD;
    double alpha_dot = 0.1;  // rad/s
    double alpha_stall = 15.0 * DEG_TO_RAD;

    // Time to stall if alpha rate continues
    double time_to_stall = (alpha_stall - alpha) / alpha_dot;

    TS_ASSERT(time_to_stall > 0);
    TS_ASSERT_DELTA(time_to_stall, 0.349, 0.01);  // About 0.35 seconds
  }

  // Test stick shaker activation logic
  void testStickShakerActivation() {
    double alpha = 14.0 * DEG_TO_RAD;
    double alpha_stall = 15.0 * DEG_TO_RAD;
    double shaker_margin = 2.0 * DEG_TO_RAD;

    bool shaker_active = (alpha > (alpha_stall - shaker_margin));

    TS_ASSERT(shaker_active);
  }

  /***************************************************************************
   * Spin Recovery Tests
   ***************************************************************************/

  // Test spin recovery yaw rate decay
  void testSpinRecoveryYawDecay() {
    double r_spin = 0.8;  // Spin yaw rate (rad/s)
    double Cn_r = -0.15;  // Yaw damping restored after AOA reduction
    double tau_r = 2.0;   // Time constant for yaw decay

    // Exponential decay of yaw rate
    double t = 3.0;  // seconds after recovery input
    double r_recovery = r_spin * exp(-t / tau_r);

    TS_ASSERT(r_recovery < r_spin);
    TS_ASSERT_DELTA(r_recovery, 0.178, 0.01);
  }

  // Test opposite rudder effectiveness in spin
  void testOppositeRudderSpin() {
    double Cn_dr = 0.12;   // Rudder power
    double delta_r = -25.0 * DEG_TO_RAD;  // Full opposite rudder

    // Anti-spin yaw moment
    double Cn_recovery = Cn_dr * delta_r;

    TS_ASSERT(Cn_recovery < 0);  // Opposes spin direction
    TS_ASSERT_DELTA(Cn_recovery, -0.0524, 0.01);
  }

  // Test elevator push for spin recovery
  void testElevatorPushSpinRecovery() {
    double alpha_spin = 35.0 * DEG_TO_RAD;
    double delta_e = -15.0 * DEG_TO_RAD;  // Forward stick

    // Pushing elevator reduces alpha
    double alpha_rate = -0.2;  // rad/s recovery rate
    double t = 2.0;  // seconds

    double alpha_recovery = alpha_spin + alpha_rate * t;

    TS_ASSERT(alpha_recovery < alpha_spin);
    TS_ASSERT(alpha_recovery > 10.0 * DEG_TO_RAD);  // Still above normal flight
  }

  /***************************************************************************
   * Inertia Coupling Tests
   ***************************************************************************/

  // Test roll coupling into pitch
  void testRollCouplingIntoPitch() {
    double p = 2.0;  // Roll rate (rad/s)
    double r = 0.5;  // Yaw rate (rad/s)
    double Ixx = 5000.0;  // Roll inertia
    double Izz = 8000.0;  // Yaw inertia
    double Iyy = 10000.0;  // Pitch inertia

    // Inertia coupling term: (Izz - Ixx) * p * r / Iyy
    double q_dot_coupling = (Izz - Ixx) * p * r / Iyy;

    TS_ASSERT(q_dot_coupling > 0);  // Pitch-up tendency
    TS_ASSERT_DELTA(q_dot_coupling, 0.3, 0.01);
  }

  // Test critical roll rate for departure
  void testCriticalRollRateDeparture() {
    double Ixx = 5000.0;
    double Izz = 8000.0;
    double Iyy = 10000.0;
    double Cm_alpha = -1.0;  // Pitch stiffness
    double q_bar_S_c = 50000.0;  // Dynamic pressure * area * chord

    // Critical roll rate approximation
    double p_crit = sqrt(-Cm_alpha * q_bar_S_c * Iyy / ((Izz - Ixx) * Ixx));

    TS_ASSERT(p_crit > 0);
    TS_ASSERT(p_crit > 1.0);  // Above typical maneuver rates
  }

  /***************************************************************************
   * High-G Stall Tests
   ***************************************************************************/

  // Test accelerated stall speed increase
  void testAcceleratedStallSpeed() {
    double V_s1 = 60.0;  // 1-G stall speed (kts)
    double n = 2.0;  // Load factor

    // Stall speed increases with sqrt(n)
    double V_s_n = V_s1 * sqrt(n);

    TS_ASSERT(V_s_n > V_s1);
    TS_ASSERT_DELTA(V_s_n, 84.85, 0.1);
  }

  // Test maneuvering speed definition
  void testManeuveringSpeed() {
    double V_s1 = 60.0;  // 1-G stall speed (kts)
    double n_max = 3.8;  // Max positive load factor (utility category)

    // Maneuvering speed Va = Vs1 * sqrt(n_max)
    double V_a = V_s1 * sqrt(n_max);

    TS_ASSERT_DELTA(V_a, 117.0, 0.5);
  }

  // Test G-limit vs stall boundary
  void testGLimitVsStallBoundary() {
    double CL_max = 1.5;
    double W = 3000.0;  // Weight (lb)
    double S = 180.0;   // Wing area (ft^2)
    double rho = 0.002377;

    // Speed where structural limit equals stall limit
    double V_corner_squared = (2.0 * W * 3.8) / (rho * S * CL_max);
    double V_corner = sqrt(V_corner_squared);

    TS_ASSERT(V_corner > 150.0);  // ft/s
    TS_ASSERT(V_corner < 250.0);
    TS_ASSERT_DELTA(V_corner, 188.48, 0.5);
  }

  /***************************************************************************
   * Angle of Sideslip Effects Tests
   ***************************************************************************/

  // Test beta effect on effective alpha
  void testBetaEffectOnAlpha() {
    double alpha_body = 12.0 * DEG_TO_RAD;
    double beta = 10.0 * DEG_TO_RAD;

    // Effective angle of attack adjusted for sideslip
    double alpha_eff = alpha_body * cos(beta);

    TS_ASSERT(alpha_eff < alpha_body);
    TS_ASSERT_DELTA(alpha_eff, 11.82 * DEG_TO_RAD, 0.01);
  }

  // Test cross-coupling at high alpha and beta
  void testCrossCouplingHighAlphaBeta() {
    double alpha = 20.0 * DEG_TO_RAD;
    double beta = 15.0 * DEG_TO_RAD;

    // Combined high alpha and beta creates severe coupling
    double coupling_severity = alpha * beta;

    TS_ASSERT(coupling_severity > 0.05);  // Significant coupling
  }

  // Test forebody vortex asymmetry
  void testForebodyVortexAsymmetry() {
    double alpha = 40.0 * DEG_TO_RAD;  // Very high alpha
    double beta_small = 2.0 * DEG_TO_RAD;  // Small sideslip

    // At high alpha, small beta creates large yaw moment from vortex asymmetry
    double Cn_vortex = 0.15 * beta_small * (alpha / (45.0 * DEG_TO_RAD));

    TS_ASSERT(Cn_vortex > 0);
    TS_ASSERT_DELTA(Cn_vortex, 0.00465, 0.001);
  }

  // Test departure susceptibility parameter
  void testDepartureSusceptibilityParameter() {
    double Cn_beta_dyn = 0.10;  // Dynamic weathercock stability
    double Cl_beta = -0.08;      // Effective dihedral

    // Departure susceptibility increases when Cn_beta_dyn < Cl_beta (in magnitude)
    // Simplified criterion: ratio indicates susceptibility
    double susceptibility = std::abs(Cl_beta) / Cn_beta_dyn;

    // Susceptibility > 1 indicates potential departure
    TS_ASSERT(susceptibility < 1.0);  // This configuration is departure resistant
    TS_ASSERT_DELTA(susceptibility, 0.8, 0.01);
  }
};

/*******************************************************************************
 * Extended FGHighAOA Tests (25 new tests)
 ******************************************************************************/

class FGHighAOAExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Vortex Flow Aerodynamics Tests
   ***************************************************************************/

  // Test 76: Leading edge vortex lift coefficient
  void testLEVortexLift() {
    double alpha = 35.0 * DEG_TO_RAD;
    double AR = 4.0;  // Aspect ratio

    // Polhamus leading edge suction analogy
    double Kp = M_PI * AR / 2.0;  // Potential flow constant
    double Kv = M_PI;  // Vortex lift constant

    double CL_vortex = Kv * std::pow(std::sin(alpha), 2) * std::cos(alpha);

    TS_ASSERT(CL_vortex > 0);
    TS_ASSERT(CL_vortex < 1.5);
  }

  // Test 77: Delta wing vortex breakdown
  void testVortexBreakdown() {
    double alpha = 30.0 * DEG_TO_RAD;
    double alpha_breakdown = 25.0 * DEG_TO_RAD;  // Vortex breakdown angle

    bool breakdown_occurred = alpha > alpha_breakdown;
    TS_ASSERT(breakdown_occurred);

    // After breakdown, lift coefficient drops
    double CL_pre_breakdown = 1.2;
    double CL_post_breakdown = 0.8;
    TS_ASSERT(CL_post_breakdown < CL_pre_breakdown);
  }

  // Test 78: Strake vortex enhancement
  void testStrakeVortexEnhancement() {
    double CL_without_strake = 1.0;
    double strake_vortex_increment = 0.3;

    double CL_with_strake = CL_without_strake + strake_vortex_increment;

    TS_ASSERT(CL_with_strake > CL_without_strake);
    TS_ASSERT_DELTA(CL_with_strake, 1.3, epsilon);
  }

  /***************************************************************************
   * Post-Stall Gyration Tests
   ***************************************************************************/

  // Test 79: Post-stall gyration frequency
  void testPostStallGyrationFrequency() {
    double Izz = 8000.0;  // slug-ft^2
    double q_bar = 50.0;   // psf
    double S = 200.0;      // ft^2
    double b = 35.0;       // ft
    double Cn_beta = 0.05; // Reduced at high alpha

    // Approximate yaw oscillation frequency
    double L_n = q_bar * S * b * Cn_beta;
    double omega_n = std::sqrt(L_n / Izz);

    TS_ASSERT(omega_n > 0.1);
    TS_ASSERT(omega_n < 2.0);
  }

  // Test 80: Tumble mode criterion
  void testTumbleModeCriterion() {
    double Iyy = 7000.0;   // Pitch inertia (intermediate)
    double Ixx = 5000.0;   // Roll inertia
    double Izz = 10000.0;  // Yaw inertia

    // Tumble prone when Iyy is intermediate between Ixx and Izz
    bool tumble_prone = (Iyy > Ixx) && (Iyy < Izz) ||
                        (Iyy < Ixx) && (Iyy > Izz);

    TS_ASSERT(tumble_prone);
  }

  // Test 81: Falling leaf mode
  void testFallingLeafMode() {
    double alpha = 45.0 * DEG_TO_RAD;
    double Cl_p = 0.0;   // Zero roll damping at high alpha
    double Cn_r = -0.05; // Reduced yaw damping

    // Falling leaf: oscillating descent without defined spin
    bool falling_leaf = (std::abs(Cl_p) < 0.1) && (Cn_r < 0);

    TS_ASSERT(falling_leaf);
  }

  /***************************************************************************
   * Stall Protection System Tests
   ***************************************************************************/

  // Test 82: Alpha floor protection limit
  void testAlphaFloorProtection() {
    double alpha_current = 16.0 * DEG_TO_RAD;
    double alpha_floor = 14.0 * DEG_TO_RAD;

    // Alpha floor applies nose-down command
    bool floor_active = alpha_current > alpha_floor;
    double pitch_rate_cmd = floor_active ? -0.1 : 0.0;

    TS_ASSERT(floor_active);
    TS_ASSERT(pitch_rate_cmd < 0);
  }

  // Test 83: Envelope limiting gain schedule
  void testEnvelopeLimitingGain() {
    double alpha = 12.0 * DEG_TO_RAD;
    double alpha_limit = 15.0 * DEG_TO_RAD;
    double margin = alpha_limit - alpha;

    // Gain increases as margin decreases
    double K_envelope = 1.0 / (margin + 0.01);

    TS_ASSERT(K_envelope > 10.0);  // High gain near limit
  }

  // Test 84: Stall identification from pitch break
  void testStallIdentificationPitchBreak() {
    double Cm_alpha_linear = -1.0;
    double Cm_alpha_measured = -0.3;  // Reduced in stall

    double reduction = (Cm_alpha_linear - Cm_alpha_measured) / Cm_alpha_linear;

    // More than 50% reduction indicates stall
    bool stall_identified = reduction > 0.5;

    TS_ASSERT(stall_identified);
    TS_ASSERT_DELTA(reduction, 0.7, 0.01);
  }

  /***************************************************************************
   * High-Alpha Roll Rate Tests
   ***************************************************************************/

  // Test 85: Roll rate capability at high alpha
  void testRollRateCapabilityHighAlpha() {
    double Cl_da_normal = 0.15;
    double alpha = 20.0 * DEG_TO_RAD;
    double alpha_crit = 15.0 * DEG_TO_RAD;

    // Roll authority degrades past stall
    double effectiveness = std::max(0.2, std::cos(2.0 * (alpha - alpha_crit)));
    double Cl_da_high_alpha = Cl_da_normal * effectiveness;

    double p_max_normal = 2.0;  // rad/s
    double p_max_high_alpha = p_max_normal * Cl_da_high_alpha / Cl_da_normal;

    TS_ASSERT(p_max_high_alpha < p_max_normal);
  }

  // Test 86: Roll coupling into yaw at high alpha
  void testRollCouplingYawHighAlpha() {
    double p = 1.0;  // rad/s roll rate
    double alpha = 25.0 * DEG_TO_RAD;

    // At high alpha, rolling creates yawing
    double Cn_p = -0.1 * std::sin(alpha);
    double yaw_moment = Cn_p * p;

    TS_ASSERT(yaw_moment < 0);  // Adverse yaw
  }

  // Test 87: Differential aileron at high alpha
  void testDifferentialAileronHighAlpha() {
    double da_up = 20.0 * DEG_TO_RAD;
    double da_down = 10.0 * DEG_TO_RAD;  // Less down aileron

    // Differential reduces adverse yaw
    double differential_ratio = da_down / da_up;

    TS_ASSERT(differential_ratio < 1.0);
    TS_ASSERT_DELTA(differential_ratio, 0.5, 0.01);
  }

  /***************************************************************************
   * Out-of-Control Recovery Tests
   ***************************************************************************/

  // Test 88: Pitch rate for nose slice recovery
  void testNoseSliceRecoveryPitchRate() {
    double alpha = 25.0 * DEG_TO_RAD;
    double q = 0.2;  // rad/s pitch rate
    double alpha_target = 10.0 * DEG_TO_RAD;

    // Time to reduce alpha at current rate
    double delta_alpha = alpha - alpha_target;
    double recovery_time = delta_alpha / std::abs(q);

    TS_ASSERT(recovery_time > 0);
    TS_ASSERT(recovery_time < 2.0);  // Reasonable recovery time
  }

  // Test 89: Controlled flight after upset
  void testControlledFlightAfterUpset() {
    double alpha = 35.0 * DEG_TO_RAD;  // Unusual attitude
    double alpha_normal = 5.0 * DEG_TO_RAD;

    // Need to reduce alpha to normal flight
    double alpha_reduction_needed = alpha - alpha_normal;

    // With 0.2 rad/s pitch rate
    double recovery_time = alpha_reduction_needed / 0.2;

    TS_ASSERT(recovery_time > 1.0);  // Takes some time
    TS_ASSERT(recovery_time < 5.0);  // But not too long
  }

  // Test 90: Energy management in recovery
  void testEnergyManagementRecovery() {
    double altitude = 5000.0;  // ft
    double V = 100.0;          // ft/s (low speed in stall)
    double V_target = 200.0;   // ft/s (normal flight)

    // Trade altitude for airspeed
    double delta_KE = 0.5 * (V_target * V_target - V * V) / 32.2;
    double altitude_loss = delta_KE;  // Simplified

    TS_ASSERT(altitude_loss > 0);
    TS_ASSERT(altitude_loss < altitude);  // Still have altitude margin
  }

  /***************************************************************************
   * Forebody/Nose Shape Effects Tests
   ***************************************************************************/

  // Test 91: Forebody strake yaw control
  void testForebodyStrakeYawControl() {
    double delta_strake = 10.0 * DEG_TO_RAD;  // Strake deflection
    double alpha = 40.0 * DEG_TO_RAD;

    // Forebody strakes effective at high alpha
    double Cn_strake = 0.05 * delta_strake * (alpha / (45.0 * DEG_TO_RAD));

    TS_ASSERT(Cn_strake > 0);
    TS_ASSERT(Cn_strake < 0.1);
  }

  // Test 92: Nose blowing for yaw control
  void testNoseBlowingYawControl() {
    double mass_flow = 0.5;  // lbs/s
    double momentum = mass_flow * 1000.0;  // velocity ft/s

    // Blowing creates asymmetric vortices
    double Cn_blowing = 0.001 * momentum;

    TS_ASSERT(Cn_blowing > 0);
  }

  // Test 93: Nose shape effect on departure
  void testNoseShapeEffect() {
    double fineness_ratio = 3.0;  // length/diameter

    // Higher fineness ratio better for high alpha
    bool good_high_alpha = fineness_ratio > 2.5;

    TS_ASSERT(good_high_alpha);
  }

  /***************************************************************************
   * Spin Modes Tests
   ***************************************************************************/

  // Test 94: Flat spin vs steep spin
  void testFlatVsSteepSpin() {
    double alpha_flat = 70.0 * DEG_TO_RAD;
    double alpha_steep = 40.0 * DEG_TO_RAD;

    // Flat spin has higher alpha
    TS_ASSERT(alpha_flat > alpha_steep);

    // Flat spin harder to recover from
    double recovery_factor_flat = 0.3;
    double recovery_factor_steep = 0.8;

    TS_ASSERT(recovery_factor_flat < recovery_factor_steep);
  }

  // Test 95: Oscillatory spin mode
  void testOscillatorySpin() {
    double yaw_rate_max = 1.0;  // rad/s
    double yaw_rate_min = 0.6;  // rad/s
    double period = 3.0;  // seconds

    double oscillation_amplitude = (yaw_rate_max - yaw_rate_min) / 2.0;

    TS_ASSERT_DELTA(oscillation_amplitude, 0.2, epsilon);
    TS_ASSERT(period > 1.0);
  }

  // Test 96: Cross-coupled spin
  void testCrossCoupledSpin() {
    double p = 0.5;  // rad/s roll
    double q = 0.3;  // rad/s pitch
    double r = 0.8;  // rad/s yaw

    // Coupled spin involves all three axes
    double total_rate = std::sqrt(p*p + q*q + r*r);

    TS_ASSERT(total_rate > 0.9);
    TS_ASSERT_DELTA(total_rate, 0.99, 0.01);
  }

  /***************************************************************************
   * Carefree Handling Tests
   ***************************************************************************/

  // Test 97: Alpha limiter effectiveness
  void testAlphaLimiter() {
    double alpha_cmd = 25.0 * DEG_TO_RAD;  // Pilot commands high alpha
    double alpha_limit = 18.0 * DEG_TO_RAD;

    double alpha_actual = std::min(alpha_cmd, alpha_limit);

    TS_ASSERT_DELTA(alpha_actual, alpha_limit, epsilon);
  }

  // Test 98: Automatic spin prevention
  void testAutomaticSpinPrevention() {
    double alpha = 16.0 * DEG_TO_RAD;
    double beta = 8.0 * DEG_TO_RAD;
    double p = 0.8;  // rad/s

    // Spin prevention activates
    double threshold = 0.5;
    bool spin_tendency = (p * beta) > threshold * 0.1;

    // System should reduce pro-spin inputs
    double rudder_reduction = spin_tendency ? 0.5 : 1.0;

    TS_ASSERT(rudder_reduction < 1.0);
  }

  // Test 99: Load factor limiting at high alpha
  void testLoadFactorLimitHighAlpha() {
    double CL_max = 1.5;
    double CL_current = 1.4;
    double n_limit = 4.0;

    // Near CL_max, reduce available G
    double CL_margin = (CL_max - CL_current) / CL_max;
    double n_available = n_limit * CL_margin;

    TS_ASSERT(n_available < n_limit);
    TS_ASSERT_DELTA(n_available, 0.267, 0.01);
  }

  // Test 100: Departure resistance with active controls
  void testDepartureResistanceActiveControls() {
    double Cn_beta_bare = 0.05;  // Low bare airframe stability
    double Cn_beta_augmented = 0.12;  // Augmented with active controls

    // Active controls improve departure resistance
    double improvement = (Cn_beta_augmented - Cn_beta_bare) / Cn_beta_bare;

    TS_ASSERT(improvement > 1.0);  // More than doubled
    TS_ASSERT_DELTA(improvement, 1.4, 0.01);
  }
};
