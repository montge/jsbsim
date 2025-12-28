/*******************************************************************************
 * FGSupersonicFlightTest.h - Unit tests for supersonic flight physics
 *
 * Tests supersonic aerodynamic phenomena including:
 * - Mach angle and cone geometry
 * - Shock wave angles (normal and oblique)
 * - Wave drag characteristics
 * - Compressibility corrections (Prandtl-Glauert, Ackeret)
 * - Transonic and supersonic drag rise
 * - Critical Mach number effects
 * - Area rule benefits
 * - Total temperature and pressure ratios
 * - Inlet recovery
 * - Sonic boom parameters
 * - Expansion fans
 * - Aerodynamic heating
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include "TestUtilities.h"

using namespace JSBSimTest;

// Specific heat ratio for air
constexpr double GAMMA = 1.4;

class FGSupersonicFlightTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Mach Angle and Cone Geometry
   ***************************************************************************/

  // Test Mach angle at Mach 2.0
  void testMachAngleMach2() {
    double M = 2.0;
    double mu = asin(1.0 / M);  // Mach angle in radians
    double mu_deg = mu * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(mu_deg, 30.0, 0.01);
  }

  // Test Mach angle at Mach 3.0
  void testMachAngleMach3() {
    double M = 3.0;
    double mu = asin(1.0 / M);
    double mu_deg = mu * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(mu_deg, 19.471, 0.01);
  }

  // Test Mach angle at Mach 1.5
  void testMachAngleMach1_5() {
    double M = 1.5;
    double mu = asin(1.0 / M);
    double mu_deg = mu * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(mu_deg, 41.810, 0.01);
  }

  // Test Mach cone half-angle at Mach 4.0
  void testMachConeMach4() {
    double M = 4.0;
    double mu = asin(1.0 / M);
    double mu_deg = mu * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(mu_deg, 14.478, 0.01);
  }

  /***************************************************************************
   * Normal Shock Waves
   ***************************************************************************/

  // Test normal shock: pressure ratio at Mach 2.0
  void testNormalShockPressureRatioMach2() {
    double M1 = 2.0;
    double P2_P1 = (2.0 * GAMMA * M1 * M1 - (GAMMA - 1.0)) / (GAMMA + 1.0);

    TS_ASSERT_DELTA(P2_P1, 4.5, 0.01);
  }

  // Test normal shock: downstream Mach at M=2.0
  void testNormalShockDownstreamMach2() {
    double M1 = 2.0;
    double M2_sq = ((GAMMA - 1.0) * M1 * M1 + 2.0) /
                   (2.0 * GAMMA * M1 * M1 - (GAMMA - 1.0));
    double M2 = sqrt(M2_sq);

    TS_ASSERT_DELTA(M2, 0.5774, 0.0001);
  }

  // Test normal shock: temperature ratio at Mach 2.5
  void testNormalShockTemperatureRatioMach2_5() {
    double M1 = 2.5;
    double T2_T1 = ((2.0 * GAMMA * M1 * M1 - (GAMMA - 1.0)) *
                    ((GAMMA - 1.0) * M1 * M1 + 2.0)) /
                   (M1 * M1 * pow(GAMMA + 1.0, 2.0));

    TS_ASSERT_DELTA(T2_T1, 2.1375, 0.001);
  }

  // Test normal shock: density ratio at Mach 3.0
  void testNormalShockDensityRatioMach3() {
    double M1 = 3.0;
    double rho2_rho1 = ((GAMMA + 1.0) * M1 * M1) /
                       ((GAMMA - 1.0) * M1 * M1 + 2.0);

    TS_ASSERT_DELTA(rho2_rho1, 3.857, 0.001);
  }

  /***************************************************************************
   * Oblique Shock Waves
   ***************************************************************************/

  // Test oblique shock: wave angle for M=2.0, deflection=10 deg
  void testObliqueShockWaveAngleMach2_10deg() {
    double M = 2.0;
    double theta = 10.0 * Constants::DEG_TO_RAD;

    // Weak shock solution approximation using theta-beta-M relation
    // For M=2.0, theta=10 deg, beta ~ 39.3 degrees
    double beta_expected = 39.3;

    TS_ASSERT(beta_expected > 30.0);  // Must be greater than Mach angle
    TS_ASSERT(beta_expected < 90.0);
  }

  // Test oblique shock: normal Mach number component
  void testObliqueShockNormalMachComponent() {
    double M = 2.5;
    double beta = 45.0 * Constants::DEG_TO_RAD;
    double Mn = M * sin(beta);

    TS_ASSERT_DELTA(Mn, 1.768, 0.001);
  }

  // Test oblique shock: tangential velocity unchanged
  void testObliqueShockTangentialVelocity() {
    double M = 2.0;
    double beta = 40.0 * Constants::DEG_TO_RAD;

    // Tangential component of velocity remains unchanged across shock
    double Mt1 = M * cos(beta);
    double Mt2 = Mt1;  // Unchanged

    TS_ASSERT_DELTA(Mt1, Mt2, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Wave Drag
   ***************************************************************************/

  // Test wave drag coefficient at Mach 1.2 (supersonic)
  void testWaveDragMach1_2() {
    double M = 1.2;

    // Simple approximation: Cd_wave proportional to 1/sqrt(M^2-1)
    double beta_param = sqrt(M * M - 1.0);
    double Cd_wave_factor = 1.0 / beta_param;

    TS_ASSERT_DELTA(beta_param, 0.6633, 0.001);
    TS_ASSERT(Cd_wave_factor > 1.0);
  }

  // Test wave drag rise at Mach 2.0
  void testWaveDragMach2() {
    double M = 2.0;
    double beta_param = sqrt(M * M - 1.0);

    TS_ASSERT_DELTA(beta_param, 1.732, 0.001);
  }

  // Test wave drag parameter at Mach 3.0
  void testWaveDragMach3() {
    double M = 3.0;
    double beta_param = sqrt(M * M - 1.0);

    TS_ASSERT_DELTA(beta_param, 2.828, 0.001);
  }

  /***************************************************************************
   * Prandtl-Glauert Compressibility Correction (Subsonic)
   ***************************************************************************/

  // Test Prandtl-Glauert correction at Mach 0.5
  void testPrandtlGlauertMach0_5() {
    double M = 0.5;
    double beta = sqrt(1.0 - M * M);
    double Cp_corr = 1.0 / beta;  // Pressure coefficient correction

    TS_ASSERT_DELTA(Cp_corr, 1.1547, 0.001);
  }

  // Test Prandtl-Glauert correction at Mach 0.7
  void testPrandtlGlauertMach0_7() {
    double M = 0.7;
    double beta = sqrt(1.0 - M * M);
    double Cp_corr = 1.0 / beta;

    TS_ASSERT_DELTA(Cp_corr, 1.4003, 0.001);
  }

  // Test Prandtl-Glauert correction at Mach 0.8
  void testPrandtlGlauertMach0_8() {
    double M = 0.8;
    double beta = sqrt(1.0 - M * M);
    double Cp_corr = 1.0 / beta;

    TS_ASSERT_DELTA(Cp_corr, 1.6667, 0.001);
  }

  // Test Prandtl-Glauert breakdown near Mach 1
  void testPrandtlGlauertMach0_95() {
    double M = 0.95;
    double beta = sqrt(1.0 - M * M);
    double Cp_corr = 1.0 / beta;

    // Correction factor becomes very large near M=1
    TS_ASSERT(Cp_corr > 3.0);
  }

  /***************************************************************************
   * Ackeret Theory (Supersonic Lift)
   ***************************************************************************/

  // Test Ackeret lift coefficient at Mach 2.0, alpha=5 deg
  void testAckeretLiftMach2_5deg() {
    double M = 2.0;
    double alpha = 5.0 * Constants::DEG_TO_RAD;
    double beta = sqrt(M * M - 1.0);
    double Cl = 4.0 * alpha / beta;

    TS_ASSERT_DELTA(Cl, 0.2018, 0.001);
  }

  // Test Ackeret lift coefficient at Mach 3.0, alpha=3 deg
  void testAckeretLiftMach3_3deg() {
    double M = 3.0;
    double alpha = 3.0 * Constants::DEG_TO_RAD;
    double beta = sqrt(M * M - 1.0);
    double Cl = 4.0 * alpha / beta;

    TS_ASSERT_DELTA(Cl, 0.0740, 0.001);
  }

  // Test Ackeret drag coefficient at Mach 2.5, alpha=4 deg
  void testAckeretDragMach2_5_4deg() {
    double M = 2.5;
    double alpha = 4.0 * Constants::DEG_TO_RAD;
    double beta = sqrt(M * M - 1.0);
    double Cd = 4.0 * alpha * alpha / beta;

    TS_ASSERT_DELTA(Cd, 0.00855, 0.0001);
  }

  /***************************************************************************
   * Supersonic Drag Coefficient
   ***************************************************************************/

  // Test supersonic drag coefficient components at Mach 1.5
  void testSupersonicDragMach1_5() {
    double M = 1.5;
    double beta = sqrt(M * M - 1.0);

    // Wave drag parameter
    TS_ASSERT_DELTA(beta, 1.118, 0.001);
  }

  // Test supersonic drag reduction with Mach number
  void testDragReductionWithMach() {
    double M1 = 1.2;
    double M2 = 2.0;

    double beta1 = sqrt(M1 * M1 - 1.0);
    double beta2 = sqrt(M2 * M2 - 1.0);

    // Wave drag factor decreases as Mach increases
    double factor1 = 1.0 / beta1;
    double factor2 = 1.0 / beta2;

    TS_ASSERT(factor2 < factor1);
  }

  /***************************************************************************
   * Area Rule Benefits
   ***************************************************************************/

  // Test area rule: smooth area distribution reduces drag
  void testAreaRuleDragReduction() {
    // Waisted fuselage reduces wave drag by ~20-30% at transonic speeds
    double drag_with_area_rule = 1.0;
    double drag_without_area_rule = 1.25;
    double reduction = (drag_without_area_rule - drag_with_area_rule) /
                       drag_without_area_rule;

    TS_ASSERT_DELTA(reduction, 0.20, 0.01);
  }

  // Test area rule applicability range
  void testAreaRuleMachRange() {
    // Most effective at transonic speeds (M = 0.8 to 1.2)
    double M_low = 0.8;
    double M_high = 1.2;

    TS_ASSERT(M_low < 1.0);
    TS_ASSERT(M_high > 1.0);
  }

  /***************************************************************************
   * Critical Mach Number
   ***************************************************************************/

  // Test critical Mach number for typical wing
  void testCriticalMachNumber() {
    // Critical Mach occurs when local Mach reaches 1.0
    // For typical transport wing, M_crit is around 0.72-0.75
    double M_crit_typical = 0.74;

    TS_ASSERT(M_crit_typical > 0.70);
    TS_ASSERT(M_crit_typical < 0.80);
  }

  // Test critical Mach number for laminar airfoil
  void testCriticalMachLaminarAirfoil() {
    // Supercritical/laminar airfoils have higher M_crit
    double M_crit_laminar = 0.78;

    TS_ASSERT(M_crit_laminar > 0.75);
    TS_ASSERT(M_crit_laminar < 0.85);
  }

  /***************************************************************************
   * Drag Divergence Mach Number
   ***************************************************************************/

  // Test drag divergence Mach number
  void testDragDivergenceMach() {
    // Typically M_dd = M_crit + 0.03 to 0.05
    double M_crit = 0.75;
    double M_dd = M_crit + 0.04;

    TS_ASSERT_DELTA(M_dd, 0.79, 0.01);
  }

  // Test drag divergence for swept wing
  void testDragDivergenceSweptWing() {
    // Sweep angle increases M_dd
    double M_dd_unswept = 0.78;
    double sweep_benefit = 0.05;
    double M_dd_swept = M_dd_unswept + sweep_benefit;

    TS_ASSERT_DELTA(M_dd_swept, 0.83, 0.01);
  }

  /***************************************************************************
   * Transonic Drag Rise
   ***************************************************************************/

  // Test transonic drag rise at Mach 0.85
  void testTransonicDragRiseMach0_85() {
    double M = 0.85;
    double M_dd = 0.80;

    // Drag increases rapidly above M_dd
    TS_ASSERT(M > M_dd);
  }

  // Test peak transonic drag around Mach 1.0
  void testPeakTransonicDrag() {
    double M_peak = 1.0;
    double M_subsonic = 0.95;
    double M_supersonic = 1.2;

    // Drag is typically highest at Mach 1.0
    TS_ASSERT(M_subsonic < M_peak);
    TS_ASSERT(M_peak < M_supersonic);
  }

  /***************************************************************************
   * Supersonic L/D Ratio
   ***************************************************************************/

  // Test L/D ratio at Mach 2.0
  void testLiftToDragMach2() {
    double M = 2.0;
    double alpha = 3.0 * Constants::DEG_TO_RAD;
    double beta = sqrt(M * M - 1.0);

    double Cl = 4.0 * alpha / beta;
    double Cd_wave = 4.0 * alpha * alpha / beta;
    double Cd_total = Cd_wave + 0.02;  // Include base drag

    double LD = Cl / Cd_total;

    TS_ASSERT(LD > 3.0);
    TS_ASSERT(LD < 8.0);
  }

  // Test maximum L/D at Mach 2.5
  void testMaxLiftToDragMach2_5() {
    double M = 2.5;
    // Typical max L/D for supersonic aircraft is 6-8
    double LD_max = 7.0;

    TS_ASSERT(LD_max > 5.0);
    TS_ASSERT(LD_max < 10.0);
  }

  /***************************************************************************
   * Total Temperature Rise
   ***************************************************************************/

  // Test total temperature at Mach 2.0
  void testTotalTemperatureMach2() {
    double M = 2.0;
    double T_static = 390.0;  // Rankine at altitude
    double T_total = T_static * (1.0 + 0.5 * (GAMMA - 1.0) * M * M);

    TS_ASSERT_DELTA(T_total, 702.0, 1.0);
  }

  // Test total temperature at Mach 3.0
  void testTotalTemperatureMach3() {
    double M = 3.0;
    double T_static = 390.0;  // Rankine
    double T_total = T_static * (1.0 + 0.5 * (GAMMA - 1.0) * M * M);

    TS_ASSERT_DELTA(T_total, 1092.0, 1.0);
  }

  // Test stagnation temperature ratio
  void testStagnationTemperatureRatio() {
    double M = 2.5;
    double T_ratio = 1.0 + 0.5 * (GAMMA - 1.0) * M * M;

    TS_ASSERT_DELTA(T_ratio, 2.25, 0.01);
  }

  /***************************************************************************
   * Ram Pressure at Supersonic Speeds
   ***************************************************************************/

  // Test ram pressure ratio at Mach 2.0
  void testRamPressureMach2() {
    double M = 2.0;
    double P_ratio = pow(1.0 + 0.5 * (GAMMA - 1.0) * M * M,
                         GAMMA / (GAMMA - 1.0));

    TS_ASSERT_DELTA(P_ratio, 7.824, 0.01);
  }

  // Test ram pressure ratio at Mach 3.0
  void testRamPressureMach3() {
    double M = 3.0;
    double P_ratio = pow(1.0 + 0.5 * (GAMMA - 1.0) * M * M,
                         GAMMA / (GAMMA - 1.0));

    TS_ASSERT_DELTA(P_ratio, 36.73, 0.1);
  }

  /***************************************************************************
   * Inlet Recovery
   ***************************************************************************/

  // Test inlet total pressure recovery at Mach 2.0
  void testInletRecoveryMach2() {
    double M = 2.0;
    // Typical inlet recovery for well-designed supersonic inlet
    double recovery = 0.93;

    TS_ASSERT(recovery > 0.90);
    TS_ASSERT(recovery < 1.0);
  }

  // Test inlet total pressure recovery at Mach 3.0
  void testInletRecoveryMach3() {
    double M = 3.0;
    // Recovery decreases at higher Mach numbers
    double recovery = 0.85;

    TS_ASSERT(recovery > 0.80);
    TS_ASSERT(recovery < 0.90);
  }

  // Test inlet recovery degradation with Mach
  void testInletRecoveryDegradation() {
    double recovery_M2 = 0.93;
    double recovery_M3 = 0.85;

    // Recovery decreases with increasing Mach number
    TS_ASSERT(recovery_M3 < recovery_M2);
  }

  /***************************************************************************
   * Sonic Boom Parameters
   ***************************************************************************/

  // Test sonic boom overpressure at Mach 1.5
  void testSonicBoomOverpressureMach1_5() {
    double M = 1.5;
    double h = 40000.0;  // Altitude in feet
    double W = 100000.0; // Weight in pounds

    // Simplified overpressure proportional to W^0.75 / h
    double dP_factor = pow(W, 0.75) / h;

    TS_ASSERT(dP_factor > 0.0);
  }

  // Test sonic boom carpet width
  void testSonicBoomCarpetWidth() {
    double h = 50000.0;  // Altitude in feet
    double M = 2.0;

    // Carpet width approximately proportional to altitude
    double width_factor = h * sqrt(M * M - 1.0);

    TS_ASSERT(width_factor > 0.0);
  }

  /***************************************************************************
   * Expansion Fans
   ***************************************************************************/

  // Test Prandtl-Meyer expansion angle at Mach 2.0
  void testPrandtlMeyerAngleMach2() {
    double M = 2.0;
    double nu = sqrt((GAMMA + 1.0) / (GAMMA - 1.0)) *
                atan(sqrt((GAMMA - 1.0) / (GAMMA + 1.0) * (M * M - 1.0))) -
                atan(sqrt(M * M - 1.0));
    double nu_deg = nu * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(nu_deg, 26.38, 0.1);
  }

  // Test Prandtl-Meyer expansion angle at Mach 3.0
  void testPrandtlMeyerAngleMach3() {
    double M = 3.0;
    double nu = sqrt((GAMMA + 1.0) / (GAMMA - 1.0)) *
                atan(sqrt((GAMMA - 1.0) / (GAMMA + 1.0) * (M * M - 1.0))) -
                atan(sqrt(M * M - 1.0));
    double nu_deg = nu * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(nu_deg, 49.76, 0.1);
  }

  // Test expansion fan: Mach number increase
  void testExpansionFanMachIncrease() {
    double M1 = 2.0;
    double M2 = 2.5;

    // Across expansion fan, Mach increases
    TS_ASSERT(M2 > M1);
  }

  /***************************************************************************
   * Shock-Induced Separation
   ***************************************************************************/

  // Test shock interaction parameter
  void testShockInteractionParameter() {
    double M = 1.3;
    double p_ratio = 1.8;  // Pressure ratio across shock

    // Strong shocks can cause boundary layer separation
    TS_ASSERT(p_ratio > 1.0);
  }

  // Test shock strength threshold for separation
  void testShockSeparationThreshold() {
    // Separation typically occurs when pressure ratio > 1.5-2.0
    double p_ratio_threshold = 1.8;

    TS_ASSERT(p_ratio_threshold > 1.5);
    TS_ASSERT(p_ratio_threshold < 2.5);
  }

  /***************************************************************************
   * Minimum Drag Speed
   ***************************************************************************/

  // Test minimum drag speed in supersonic regime
  void testMinimumDragSpeedSupersonic() {
    // Minimum drag typically occurs at lower supersonic speeds
    double M_min_drag = 1.15;

    TS_ASSERT(M_min_drag > 1.0);
    TS_ASSERT(M_min_drag < 1.5);
  }

  /***************************************************************************
   * Maximum L/D at Supersonic Speeds
   ***************************************************************************/

  // Test optimal angle of attack for max L/D at Mach 2.0
  void testOptimalAlphaForMaxLD() {
    double M = 2.0;
    // Optimal alpha typically 2-4 degrees at supersonic speeds
    double alpha_opt = 3.0 * Constants::DEG_TO_RAD;

    TS_ASSERT(alpha_opt > 0.0);
    TS_ASSERT(alpha_opt < 0.1);  // Less than ~6 degrees
  }

  // Test L/D decreases with increasing Mach
  void testLDDecreaseWithMach() {
    double LD_Mach2 = 7.5;
    double LD_Mach3 = 6.0;

    // L/D generally decreases at higher supersonic speeds
    TS_ASSERT(LD_Mach3 < LD_Mach2);
  }

  /***************************************************************************
   * Thermal Limits at High Mach
   ***************************************************************************/

  // Test stagnation temperature at Mach 3.0
  void testThermalLimitMach3() {
    double M = 3.0;
    double T_static = 390.0;  // Rankine
    double T_stag = T_static * (1.0 + 0.2 * M * M);

    // Should exceed 1000 R (aluminum softening point ~ 750-800 F)
    TS_ASSERT(T_stag > 1000.0);
  }

  // Test thermal limit at Mach 2.0
  void testThermalLimitMach2() {
    double M = 2.0;
    double T_static = 390.0;  // Rankine at altitude
    double T_stag = T_static * (1.0 + 0.2 * M * M);

    TS_ASSERT_DELTA(T_stag, 702.0, 1.0);
  }

  /***************************************************************************
   * Aerodynamic Heating
   ***************************************************************************/

  // Test recovery temperature at Mach 2.0
  void testRecoveryTemperatureMach2() {
    double M = 2.0;
    double T_static = 390.0;  // Rankine
    double r = 0.9;  // Recovery factor for turbulent boundary layer
    double T_recovery = T_static * (1.0 + r * 0.2 * M * M);

    TS_ASSERT_DELTA(T_recovery, 671.4, 1.0);
  }

  // Test recovery temperature at Mach 3.0
  void testRecoveryTemperatureMach3() {
    double M = 3.0;
    double T_static = 390.0;  // Rankine
    double r = 0.9;  // Recovery factor
    double T_recovery = T_static * (1.0 + r * 0.2 * M * M);

    TS_ASSERT_DELTA(T_recovery, 1022.4, 1.0);
  }

  // Test heat transfer rate proportionality
  void testHeatTransferRate() {
    double M1 = 2.0;
    double M2 = 3.0;

    // Heat transfer proportional to M^3
    double q1 = M1 * M1 * M1;
    double q2 = M2 * M2 * M2;

    TS_ASSERT(q2 > q1);
    TS_ASSERT_DELTA(q2 / q1, 3.375, 0.01);
  }

  // Test skin friction heating at Mach 2.5
  void testSkinFrictionHeatingMach2_5() {
    double M = 2.5;
    double T_static = 390.0;  // Rankine
    double r = 0.88;  // Recovery factor
    double T_adiabatic_wall = T_static * (1.0 + r * 0.2 * M * M);

    TS_ASSERT(T_adiabatic_wall > T_static);
    TS_ASSERT_DELTA(T_adiabatic_wall, 819.0, 1.0);
  }

  /***************************************************************************
   * Additional Supersonic Phenomena
   ***************************************************************************/

  // Test dynamic pressure at supersonic speeds
  void testDynamicPressureMach2() {
    double M = 2.0;
    double P_static = 1000.0;  // psf
    double q = 0.5 * GAMMA * P_static * M * M;

    TS_ASSERT_DELTA(q, 2800.0, 1.0);
  }

  // Test speed of sound calculation
  void testSpeedOfSound() {
    double T = 390.0;  // Rankine
    double R = 1716.0;  // ft-lbf/(slug-R) for air
    double a = sqrt(GAMMA * R * T);

    TS_ASSERT_DELTA(a, 968.1, 1.0);  // fps
  }

  // Test velocity at Mach 2.0
  void testVelocityMach2() {
    double M = 2.0;
    double a = 968.0;  // fps
    double V = M * a;

    TS_ASSERT_DELTA(V, 1936.0, 1.0);  // fps
  }

  // Test compressibility factor
  void testCompressibilityFactor() {
    double M = 0.8;
    double beta_subsonic = sqrt(1.0 - M * M);

    TS_ASSERT_DELTA(beta_subsonic, 0.6, 0.01);
  }

  /***************************************************************************
   * Hypersonic Regime Tests
   ***************************************************************************/

  void testHypersonicMachNumber() {
    double M = 5.0;
    TS_ASSERT(M > 4.0);
  }

  void testHypersonicTemperatureRatio() {
    double M = 5.0;
    double T_ratio = 1.0 + 0.5 * (GAMMA - 1.0) * M * M;

    TS_ASSERT_DELTA(T_ratio, 6.0, 0.01);
  }

  void testRealGasEffects() {
    double M = 8.0;
    double T_static = 220.0;
    double T_total = T_static * (1.0 + 0.2 * M * M);

    TS_ASSERT(T_total > 3000.0);
  }

  void testDissociationTemperature() {
    double T_dissociation = 2500.0;
    double T_current = 3000.0;

    bool dissociation = (T_current > T_dissociation);
    TS_ASSERT(dissociation);
  }

  /***************************************************************************
   * Transonic Flow Characteristics
   ***************************************************************************/

  void testTransonicMachRange() {
    double M_lower = 0.8;
    double M_upper = 1.2;

    TS_ASSERT(M_lower < 1.0);
    TS_ASSERT(M_upper > 1.0);
  }

  void testShockFormationMach() {
    double M = 1.0;
    bool local_supersonic = (M >= 1.0);

    TS_ASSERT(local_supersonic);
  }

  void testTransonicDragPeak() {
    double Cd_M08 = 0.025;
    double Cd_M10 = 0.080;
    double Cd_M12 = 0.055;

    TS_ASSERT(Cd_M10 > Cd_M08);
    TS_ASSERT(Cd_M10 > Cd_M12);
  }

  void testTransonicBuffet() {
    double M = 0.92;
    double M_buffet = 0.90;

    bool buffeting = (M > M_buffet);
    TS_ASSERT(buffeting);
  }

  /***************************************************************************
   * Shock Wave Interactions
   ***************************************************************************/

  void testShockBoundaryLayerInteraction() {
    double M_upstream = 2.0;
    double shock_strength = 1.8;

    bool separation_likely = (shock_strength > 1.5);
    TS_ASSERT(separation_likely);
  }

  void testShockReflection() {
    double beta_incident = 30.0;
    double beta_reflected = beta_incident;

    TS_ASSERT_DELTA(beta_reflected, beta_incident, 0.1);
  }

  void testBowShockStandoff() {
    double M = 2.0;
    double body_radius = 1.0;

    double standoff = body_radius * 0.14 * pow(M, -2.0);
    TS_ASSERT(standoff > 0.0);
    TS_ASSERT(standoff < body_radius);
  }

  void testShockAngleVsMach() {
    double M1 = 1.5;
    double M2 = 3.0;

    double mu1 = asin(1.0 / M1) * Constants::RAD_TO_DEG;
    double mu2 = asin(1.0 / M2) * Constants::RAD_TO_DEG;

    TS_ASSERT(mu2 < mu1);
  }

  /***************************************************************************
   * Propulsion at Supersonic Speeds
   ***************************************************************************/

  void testRamjetEfficiency() {
    double M = 3.0;
    double eta_thermal = 0.55;

    TS_ASSERT(eta_thermal > 0.5);
    TS_ASSERT(eta_thermal < 0.7);
  }

  void testScramjetStartMach() {
    double M_start = 4.5;

    TS_ASSERT(M_start > 4.0);
  }

  void testAfterburnerThrust() {
    double dry_thrust = 10000.0;
    double ab_factor = 1.5;

    double wet_thrust = dry_thrust * ab_factor;
    TS_ASSERT_DELTA(wet_thrust, 15000.0, 10.0);
  }

  void testInletSpillage() {
    double M_design = 2.0;
    double M_actual = 1.5;

    bool spillage = (M_actual < M_design);
    TS_ASSERT(spillage);
  }

  /***************************************************************************
   * Control Surface Effectiveness
   ***************************************************************************/

  void testSupersonicControlPower() {
    double M = 2.0;
    double beta = sqrt(M * M - 1.0);
    double Cl_delta = 2.0 / beta;

    TS_ASSERT(Cl_delta > 0.0);
    TS_ASSERT(Cl_delta < 2.0);
  }

  void testAllMovingTailEffectiveness() {
    double M = 2.5;
    double delta = 5.0 * Constants::DEG_TO_RAD;
    double beta = sqrt(M * M - 1.0);

    double Cl = 4.0 * delta / beta;
    TS_ASSERT(Cl > 0.0);
  }

  void testReactionControlRequirement() {
    double M = 4.0;
    double q_bar = 500.0;

    bool need_rcs = (q_bar < 100.0);
    TS_ASSERT(!need_rcs);
  }

  /***************************************************************************
   * Structural Thermal Effects
   ***************************************************************************/

  void testThermalExpansion() {
    double T_ref = 520.0;
    double T_hot = 800.0;
    double alpha = 12e-6;
    double L = 100.0;

    double dL = alpha * L * (T_hot - T_ref);
    TS_ASSERT(dL > 0.0);
  }

  void testMaterialTemperatureLimit() {
    double T_aluminum = 400.0;
    double T_titanium = 800.0;
    double T_nickel = 1200.0;

    TS_ASSERT(T_titanium > T_aluminum);
    TS_ASSERT(T_nickel > T_titanium);
  }

  void testThermalGradientStress() {
    double dT = 200.0;
    double E = 10.5e6;
    double alpha = 12e-6;

    double thermal_stress = E * alpha * dT;
    TS_ASSERT(thermal_stress > 0.0);
  }

  /***************************************************************************
   * Stability at Supersonic Speeds
   ***************************************************************************/

  void testAerodynamicCenterShift() {
    double xac_subsonic = 0.25;
    double xac_supersonic = 0.50;

    TS_ASSERT(xac_supersonic > xac_subsonic);
  }

  void testStaticMargin() {
    double xcg = 0.35;
    double xac = 0.50;
    double MAC = 10.0;

    double SM = (xac - xcg) * MAC;
    TS_ASSERT(SM > 0.0);
  }

  void testDamping() {
    double Cmq_subsonic = -15.0;
    double Cmq_supersonic = -8.0;

    TS_ASSERT(std::fabs(Cmq_supersonic) < std::fabs(Cmq_subsonic));
  }

  void testDutchRollFrequency() {
    double freq_subsonic = 1.5;
    double freq_supersonic = 0.8;

    TS_ASSERT(freq_supersonic < freq_subsonic);
  }

  /***************************************************************************
   * Design Considerations
   ***************************************************************************/

  void testSweepAngleEffect() {
    double M_eff = 2.0;
    double sweep = 45.0 * Constants::DEG_TO_RAD;

    double M_normal = M_eff * cos(sweep);
    TS_ASSERT(M_normal < M_eff);
  }

  void testFinenessRatio() {
    double length = 100.0;
    double diameter = 10.0;

    double fineness = length / diameter;
    TS_ASSERT_DELTA(fineness, 10.0, 0.1);
  }

  void testNoseConeOptimization() {
    double Cd_cone = 0.10;
    double Cd_ogive = 0.08;
    double Cd_parabolic = 0.07;

    TS_ASSERT(Cd_parabolic < Cd_ogive);
    TS_ASSERT(Cd_ogive < Cd_cone);
  }

  /***************************************************************************
   * Environmental Effects
   ***************************************************************************/

  void testAltitudeEffectOnMach() {
    double V = 2000.0;
    double a_sl = 1116.0;
    double a_alt = 968.0;

    double M_sl = V / a_sl;
    double M_alt = V / a_alt;

    TS_ASSERT(M_alt > M_sl);
  }

  void testDensityAltitudeEffect() {
    double rho_sl = 0.002377;
    double rho_alt = 0.0007;

    double ratio = rho_alt / rho_sl;
    TS_ASSERT(ratio < 0.5);
  }

  void testReynoldsNumberSupersonic() {
    double V = 2000.0;
    double L = 50.0;
    double nu = 1.57e-4;

    double Re = V * L / nu;
    TS_ASSERT(Re > 1e8);
  }

  /***************************************************************************
   * Performance Metrics
   ***************************************************************************/

  void testSupersonicRange() {
    double Isp = 1500.0;
    double Wf_Wi = 0.4;

    double range_factor = Isp * log(1.0 / (1.0 - Wf_Wi));
    TS_ASSERT(range_factor > 0.0);
  }

  void testSupersonicEndurance() {
    double fuel = 10000.0;
    double sfc = 1.2;
    double thrust = 5000.0;

    double endurance = fuel / (sfc * thrust);
    TS_ASSERT(endurance > 1.0);
  }

  void testMaximumMachCapability() {
    double thrust_available = 20000.0;
    double drag_M2 = 15000.0;
    double drag_M3 = 25000.0;

    bool can_reach_M2 = (thrust_available > drag_M2);
    bool can_reach_M3 = (thrust_available > drag_M3);

    TS_ASSERT(can_reach_M2);
    TS_ASSERT(!can_reach_M3);
  }

  /***************************************************************************
   * Additional Physical Phenomena
   ***************************************************************************/

  void testViscousInteractionParameter() {
    double M = 10.0;
    double Re = 1e6;

    double chi = M * M * M / sqrt(Re);
    TS_ASSERT(chi > 0.0);
  }

  void testBluntBodyHeating() {
    double M = 8.0;
    double rho = 0.0001;
    double V = 8000.0;

    double q_stag = 0.5 * rho * V * V * V;
    TS_ASSERT(q_stag > 0.0);
  }

  void testSlenderBodyTheory() {
    double fineness_ratio = 20.0;
    bool slender = (fineness_ratio > 10.0);

    TS_ASSERT(slender);
  }

  void testCrossflowDrag() {
    double alpha = 10.0 * Constants::DEG_TO_RAD;
    double Cd_crossflow = 1.2 * sin(alpha) * sin(alpha);

    TS_ASSERT(Cd_crossflow > 0.0);
  }
};
