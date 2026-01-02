/*******************************************************************************
 * FGIcingTest.h - Unit tests for aircraft icing effects
 *
 * Tests ice accretion and degradation formulas including:
 * - Ice accretion rate calculations
 * - Lift degradation from ice
 * - Drag increase from ice
 * - Stall speed increase
 * - Control surface effectiveness reduction
 * - Pitot-static system icing
 * - Engine intake icing
 * - Propeller icing effects
 * - Tail plane icing
 * - Freezing rain vs rime ice
 * - Temperature/humidity conditions
 * - Anti-ice/de-ice heat requirements
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAerodynamics.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;

class FGIcingTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Ice Accretion Rate Tests
   ***************************************************************************/

  // Test Langmuir-Blodgett ice accretion rate formula
  void testIceAccretionRateLangmuirBlodgett() {
    // Ice accretion rate: dm/dt = LWC * V * A * E
    // LWC = Liquid Water Content (g/m^3)
    // V = Velocity (m/s)
    // A = Collection area (m^2)
    // E = Collection efficiency (dimensionless)

    double LWC = 0.5;        // g/m^3 (moderate icing)
    double velocity = 80.0;  // m/s (155 knots)
    double area = 20.0;      // m^2 (wing area)
    double efficiency = 0.8; // 80% collection efficiency

    double accretion_rate = LWC * velocity * area * efficiency;
    // Rate = 0.5 * 80 * 20 * 0.8 = 640 g/s
    TS_ASSERT_DELTA(accretion_rate, 640.0, epsilon);
  }

  // Test ice accretion rate at different LWC levels
  void testIceAccretionAtDifferentLWC() {
    double velocity = 100.0;
    double area = 15.0;
    double efficiency = 0.75;

    // Light icing (LWC < 0.3 g/m^3)
    double LWC_light = 0.2;
    double rate_light = LWC_light * velocity * area * efficiency;
    TS_ASSERT_DELTA(rate_light, 225.0, epsilon);

    // Moderate icing (0.3 - 0.6 g/m^3)
    double LWC_moderate = 0.5;
    double rate_moderate = LWC_moderate * velocity * area * efficiency;
    TS_ASSERT_DELTA(rate_moderate, 562.5, epsilon);

    // Heavy icing (> 0.6 g/m^3)
    double LWC_heavy = 1.0;
    double rate_heavy = LWC_heavy * velocity * area * efficiency;
    TS_ASSERT_DELTA(rate_heavy, 1125.0, epsilon);
  }

  // Test ice accretion mass accumulation over time
  void testIceAccretionMassAccumulation() {
    double accretion_rate = 500.0;  // g/s
    double time = 60.0;             // seconds

    double ice_mass = accretion_rate * time;
    // Mass = 500 * 60 = 30,000 g = 30 kg
    TS_ASSERT_DELTA(ice_mass, 30000.0, epsilon);

    // Convert to pounds
    double ice_mass_lbs = ice_mass / 453.592;
    TS_ASSERT_DELTA(ice_mass_lbs, 66.14, 0.01);
  }

  // Test collection efficiency based on droplet size
  void testCollectionEfficiencyDropletSize() {
    // Collection efficiency increases with droplet diameter
    // Simplified model: E = min(1.0, K * MVD)
    // MVD = Median Volume Diameter (micrometers)
    // K = empirical constant

    double K = 0.05;

    // Small droplets (10 μm)
    double MVD_small = 10.0;
    double E_small = std::min(1.0, K * MVD_small);
    TS_ASSERT_DELTA(E_small, 0.5, epsilon);

    // Medium droplets (20 μm)
    double MVD_medium = 20.0;
    double E_medium = std::min(1.0, K * MVD_medium);
    TS_ASSERT_DELTA(E_medium, 1.0, epsilon);

    // Large droplets (40 μm)
    double MVD_large = 40.0;
    double E_large = std::min(1.0, K * MVD_large);
    TS_ASSERT_EQUALS(E_large, 1.0);
  }

  // Test Messinger ice accretion energy balance
  void testMessingerIceAccretionModel() {
    // Energy balance: heat in = heat out + latent heat
    // Simplified: accretion fraction = (T_freeze - T_surface) / (T_freeze - T_static)

    double T_freeze = 273.15;    // K (0°C)
    double T_static = 263.15;    // K (-10°C)
    double T_surface = 268.15;   // K (-5°C)

    double freeze_fraction = (T_freeze - T_surface) / (T_freeze - T_static);
    // Fraction = (273.15 - 268.15) / (273.15 - 263.15) = 5 / 10 = 0.5
    TS_ASSERT_DELTA(freeze_fraction, 0.5, epsilon);
  }

  /***************************************************************************
   * Lift Degradation Tests
   ***************************************************************************/

  // Test lift coefficient degradation from leading edge ice
  void testLiftCoefficientDegradationLeadingEdge() {
    // CL_ice = CL_clean * (1 - k_roughness * (ice_thickness/chord))
    double CL_clean = 1.2;
    double k_roughness = 0.3;      // Roughness degradation factor
    double ice_thickness = 0.02;   // meters (2 cm)
    double chord = 2.0;            // meters

    double CL_ice = CL_clean * (1.0 - k_roughness * (ice_thickness / chord));
    // CL_ice = 1.2 * (1 - 0.3 * 0.01) = 1.2 * 0.997 = 1.1964
    TS_ASSERT_DELTA(CL_ice, 1.1964, 0.0001);
  }

  // Test CLmax reduction from ice
  void testCLmaxReductionFromIce() {
    // CLmax can be reduced by 20-40% with significant ice
    double CLmax_clean = 1.6;

    // Light ice (5% reduction)
    double reduction_light = 0.05;
    double CLmax_light_ice = CLmax_clean * (1.0 - reduction_light);
    TS_ASSERT_DELTA(CLmax_light_ice, 1.52, epsilon);

    // Moderate ice (20% reduction)
    double reduction_moderate = 0.20;
    double CLmax_moderate_ice = CLmax_clean * (1.0 - reduction_moderate);
    TS_ASSERT_DELTA(CLmax_moderate_ice, 1.28, epsilon);

    // Heavy ice (40% reduction)
    double reduction_heavy = 0.40;
    double CLmax_heavy_ice = CLmax_clean * (1.0 - reduction_heavy);
    TS_ASSERT_DELTA(CLmax_heavy_ice, 0.96, epsilon);
  }

  // Test lift loss from ice roughness
  void testLiftLossFromRoughness() {
    // Roughness height effects on CL
    // ΔCL/CL = -k * (k_s/c)^n where k_s is roughness height
    double CL_clean = 1.0;
    double k = 2.0;               // Roughness constant
    double k_s = 0.001;           // 1 mm roughness
    double chord = 2.0;           // 2 m chord
    double n = 0.5;               // Exponent

    double roughness_factor = pow(k_s / chord, n);
    double CL_degradation = k * roughness_factor;
    double CL_rough = CL_clean * (1.0 - CL_degradation);

    TS_ASSERT_DELTA(roughness_factor, 0.02236, 0.0001);
    TS_ASSERT_DELTA(CL_rough, 0.9553, 0.001);
  }

  // Test spanwise ice distribution effect on lift
  void testSpanwiseIceDistributionLift() {
    // Tip ice more critical than root ice
    // CL_eff = CL_clean * (1 - k_span * fraction_iced)
    double CL_clean = 1.3;

    // 25% span iced at root (less critical)
    double k_root = 0.15;
    double fraction_root = 0.25;
    double CL_root_ice = CL_clean * (1.0 - k_root * fraction_root);
    TS_ASSERT_DELTA(CL_root_ice, 1.251, 0.001);

    // 25% span iced at tip (more critical)
    double k_tip = 0.40;
    double fraction_tip = 0.25;
    double CL_tip_ice = CL_clean * (1.0 - k_tip * fraction_tip);
    TS_ASSERT_DELTA(CL_tip_ice, 1.17, 0.01);
  }

  /***************************************************************************
   * Drag Increase Tests
   ***************************************************************************/

  // Test drag coefficient increase from ice
  void testDragCoefficientIncreaseFromIce() {
    // CD_ice = CD_clean + ΔCD_ice
    // ΔCD_ice = k_ice * (ice_thickness/chord)^2
    double CD_clean = 0.025;
    double k_ice = 0.5;
    double ice_thickness = 0.03;   // 3 cm
    double chord = 2.0;            // 2 m

    double delta_CD = k_ice * pow(ice_thickness / chord, 2);
    double CD_ice = CD_clean + delta_CD;

    // delta_CD = 0.5 * (0.015)^2 = 0.5 * 0.000225 = 0.0001125
    TS_ASSERT_DELTA(delta_CD, 0.0001125, 1e-7);
    TS_ASSERT_DELTA(CD_ice, 0.0251125, 1e-7);
  }

  // Test drag increase from different ice shapes
  void testDragIncreaseIceShapes() {
    double CD_clean = 0.030;

    // Rime ice (rough, opaque) - lower drag increase
    double k_rime = 0.3;
    double thickness_rime = 0.02;
    double chord = 2.0;
    double CD_rime = CD_clean + k_rime * pow(thickness_rime / chord, 2);
    TS_ASSERT_DELTA(CD_rime, 0.03003, 0.0001);

    // Clear ice (smooth, glaze) - horn shape, higher drag
    double k_clear = 0.8;
    double thickness_clear = 0.02;
    double CD_clear = CD_clean + k_clear * pow(thickness_clear / chord, 2);
    TS_ASSERT_DELTA(CD_clear, 0.03008, 0.0001);
  }

  // Test parasite drag from ice on fuselage
  void testParasiteDragFromIce() {
    // Parasite drag: D_p = 0.5 * rho * V^2 * CD_p * S_wet
    double rho = 1.225;           // kg/m^3
    double velocity = 80.0;       // m/s
    double CD_p_clean = 0.020;
    double CD_p_ice = 0.028;      // 40% increase
    double S_wet = 50.0;          // m^2

    double drag_clean = 0.5 * rho * velocity * velocity * CD_p_clean * S_wet;
    double drag_ice = 0.5 * rho * velocity * velocity * CD_p_ice * S_wet;
    double drag_increase = drag_ice - drag_clean;

    TS_ASSERT_DELTA(drag_clean, 3920.0, 1.0);
    TS_ASSERT_DELTA(drag_ice, 5488.0, 1.0);
    TS_ASSERT_DELTA(drag_increase, 1568.0, 1.0);
  }

  // Test ice horn drag penalty
  void testIceHornDragPenalty() {
    // Ice horns create significant drag increase
    // CD_horn = CD_clean + k_horn * (horn_height/chord)^1.5
    double CD_clean = 0.025;
    double k_horn = 2.0;
    double horn_height = 0.015;    // 1.5 cm horn
    double chord = 2.0;

    double delta_CD_horn = k_horn * pow(horn_height / chord, 1.5);
    double CD_with_horn = CD_clean + delta_CD_horn;

    TS_ASSERT_DELTA(delta_CD_horn, 0.001299, 0.0001);
    TS_ASSERT_DELTA(CD_with_horn, 0.026299, 0.0001);
  }

  /***************************************************************************
   * Stall Speed Increase Tests
   ***************************************************************************/

  // Test stall speed increase from CLmax reduction
  void testStallSpeedIncreaseFromIce() {
    // V_stall = sqrt(2 * W / (rho * S * CLmax))
    double weight = 10000.0;      // N
    double rho = 1.225;           // kg/m^3
    double area = 15.0;           // m^2
    double CLmax_clean = 1.6;
    double CLmax_ice = 1.2;       // 25% reduction

    double V_stall_clean = sqrt(2.0 * weight / (rho * area * CLmax_clean));
    double V_stall_ice = sqrt(2.0 * weight / (rho * area * CLmax_ice));
    double V_stall_increase = V_stall_ice - V_stall_clean;

    // sqrt(20000/(1.225*15*1.6)) = 26.08, sqrt(20000/(1.225*15*1.2)) = 30.12
    TS_ASSERT_DELTA(V_stall_clean, 26.08, 0.01);
    TS_ASSERT_DELTA(V_stall_ice, 30.12, 0.01);
    TS_ASSERT_DELTA(V_stall_increase, 4.04, 0.01);
  }

  // Test stall speed increase percentage
  void testStallSpeedIncreasePercentage() {
    // Percentage increase = sqrt(CLmax_clean / CLmax_ice) - 1
    double CLmax_clean = 1.5;
    double CLmax_ice = 1.2;

    double speed_ratio = sqrt(CLmax_clean / CLmax_ice);
    double percent_increase = (speed_ratio - 1.0) * 100.0;

    TS_ASSERT_DELTA(speed_ratio, 1.118, 0.001);
    TS_ASSERT_DELTA(percent_increase, 11.8, 0.1);
  }

  // Test critical angle of attack reduction
  void testCriticalAOAReduction() {
    // Ice reduces critical AOA by shifting stall to lower angles
    double alpha_stall_clean = 16.0;  // degrees

    // Light ice (1-2 degree reduction)
    double reduction_light = 1.5;
    double alpha_stall_light = alpha_stall_clean - reduction_light;
    TS_ASSERT_DELTA(alpha_stall_light, 14.5, epsilon);

    // Heavy ice (3-5 degree reduction)
    double reduction_heavy = 4.0;
    double alpha_stall_heavy = alpha_stall_clean - reduction_heavy;
    TS_ASSERT_DELTA(alpha_stall_heavy, 12.0, epsilon);
  }

  /***************************************************************************
   * Control Surface Effectiveness Tests
   ***************************************************************************/

  // Test elevator effectiveness reduction
  void testElevatorEffectivenessReduction() {
    // Control power: dCL/dδ_e (per degree)
    double dCL_dE_clean = 0.04;    // per degree

    // Ice on horizontal tail reduces effectiveness
    double ice_factor = 0.75;      // 25% reduction
    double dCL_dE_ice = dCL_dE_clean * ice_factor;

    TS_ASSERT_DELTA(dCL_dE_ice, 0.03, epsilon);

    // For 10 degree deflection
    double deflection = 10.0;
    double dCL_clean = dCL_dE_clean * deflection;
    double dCL_ice = dCL_dE_ice * deflection;

    TS_ASSERT_DELTA(dCL_clean, 0.4, epsilon);
    TS_ASSERT_DELTA(dCL_ice, 0.3, epsilon);
  }

  // Test aileron effectiveness reduction
  void testAileronEffectivenessReduction() {
    // Roll control power: dCl/dδ_a (per degree)
    double dCl_dA_clean = 0.025;   // per degree

    // Ice reduces aileron effectiveness by 15-30%
    double reduction = 0.20;
    double dCl_dA_ice = dCl_dA_clean * (1.0 - reduction);

    TS_ASSERT_DELTA(dCl_dA_ice, 0.020, epsilon);
  }

  // Test rudder hinge moment increase from ice
  void testRudderHingeMomentIncrease() {
    // Hinge moment coefficient: CH = CH0 + CH_delta * delta
    double CH0_clean = 0.01;
    double CH_delta_clean = -0.003;
    double delta = 20.0;           // degrees

    double CH_clean = CH0_clean + CH_delta_clean * delta;

    // Ice increases hinge moments by adding friction
    double ice_friction_factor = 1.3;
    double CH_ice = CH_clean * ice_friction_factor;

    TS_ASSERT_DELTA(CH_clean, -0.05, epsilon);
    TS_ASSERT_DELTA(CH_ice, -0.065, 0.001);
  }

  // Test control surface mass balance change
  void testControlSurfaceMassBalanceChange() {
    // Ice mass on control surface shifts CG
    double surface_mass = 5.0;     // kg
    double ice_mass = 1.0;         // kg (20% of surface mass)
    double total_mass = surface_mass + ice_mass;

    // Mass ratio
    double mass_ratio = total_mass / surface_mass;
    TS_ASSERT_DELTA(mass_ratio, 1.2, epsilon);

    // Moment of inertia increases (assuming ice at leading edge)
    // I_ice/I_clean ≈ (1 + m_ice/m_surface)
    double inertia_ratio = 1.0 + (ice_mass / surface_mass);
    TS_ASSERT_DELTA(inertia_ratio, 1.2, epsilon);
  }

  /***************************************************************************
   * Pitot-Static System Icing Tests
   ***************************************************************************/

  // Test pitot tube blockage effects
  void testPitotTubeBlockage() {
    // Pitot pressure: P_pitot = P_static + 0.5 * rho * V^2
    double P_static = 101325.0;    // Pa
    double rho = 1.225;            // kg/m^3
    double velocity = 80.0;        // m/s

    double P_pitot_clean = P_static + 0.5 * rho * velocity * velocity;

    // Blocked pitot: pressure stays constant, reads low airspeed
    // If pitot blocked at 60 m/s
    double V_blocked = 60.0;
    double P_pitot_blocked = P_static + 0.5 * rho * V_blocked * V_blocked;

    TS_ASSERT_DELTA(P_pitot_clean, 105245.0, 1.0);
    TS_ASSERT_DELTA(P_pitot_blocked, 103530.0, 1.0);
  }

  // Test static port blockage effects
  void testStaticPortBlockage() {
    // Static pressure changes with altitude
    // If static port blocked at sea level (101325 Pa)
    double P_static_blocked = 101325.0;

    // At altitude, true static pressure is lower
    double P_static_altitude = 95000.0;    // Pa at ~500m

    // Indicated altitude error
    // Using simplified: ΔP ≈ -rho * g * Δh
    double rho = 1.225;
    double g = 9.81;
    double delta_P = P_static_altitude - P_static_blocked;
    double altitude_error = -delta_P / (rho * g);

    TS_ASSERT_DELTA(altitude_error, 526.5, 1.0);
  }

  // Test total pressure probe icing
  void testTotalPressureProbeIcing() {
    // Ice restricts flow to probe
    // Measured pressure = P_true * (1 - blockage_factor)
    double P_total_true = 105000.0;    // Pa

    // 30% blockage
    double blockage = 0.30;
    double P_total_measured = P_total_true * (1.0 - blockage);

    TS_ASSERT_DELTA(P_total_measured, 73500.0, 1.0);
  }

  /***************************************************************************
   * Engine Intake Icing Tests
   ***************************************************************************/

  // Test carburetor ice temperature drop
  void testCarburetorIceTemperatureDrop() {
    // Temperature drop from fuel vaporization
    // ΔT = L_vap * (m_fuel/m_air) / Cp_air
    // Simplified: ΔT ≈ 15-20°C for typical mixture

    double T_ambient = 15.0;       // °C
    double delta_T_carb = -18.0;   // °C (typical drop)
    double T_carb = T_ambient + delta_T_carb;

    TS_ASSERT_DELTA(T_carb, -3.0, epsilon);

    // Ice forms if T_carb < 0°C and moisture present
    TS_ASSERT(T_carb < 0.0);
  }

  // Test intake mass flow reduction from ice
  void testIntakeMassFlowReduction() {
    // Mass flow: m_dot = rho * A * V
    double rho = 1.225;            // kg/m^3
    double A_clean = 0.05;         // m^2
    double velocity = 50.0;        // m/s

    double m_dot_clean = rho * A_clean * velocity;

    // 20% area reduction from ice
    double A_ice = A_clean * 0.8;
    double m_dot_ice = rho * A_ice * velocity;
    double flow_reduction = m_dot_clean - m_dot_ice;

    TS_ASSERT_DELTA(m_dot_clean, 3.0625, 0.001);
    TS_ASSERT_DELTA(m_dot_ice, 2.45, 0.01);
    TS_ASSERT_DELTA(flow_reduction, 0.6125, 0.001);
  }

  // Test compressor inlet temperature with ice
  void testCompressorInletTemperatureIce() {
    // Total temperature recovery: T_total = T_static * (1 + 0.2 * M^2)
    double T_static = 253.15;      // K (-20°C)
    double Mach = 0.5;

    double T_total = T_static * (1.0 + 0.2 * Mach * Mach);

    // 253.15 * 1.05 = 265.81
    TS_ASSERT_DELTA(T_total, 265.81, 0.01);

    // Ice on inlet guide vanes if T_total near freezing
    // Critical range: 270-280 K
    bool ice_risk = (T_total > 270.0 && T_total < 280.0);
    TS_ASSERT_EQUALS(ice_risk, false);
  }

  /***************************************************************************
   * Propeller Icing Tests
   ***************************************************************************/

  // Test propeller thrust loss from ice
  void testPropellerThrustLossFromIce() {
    // Thrust = Ct * rho * n^2 * D^4
    double Ct_clean = 0.10;
    double Ct_ice = 0.085;         // 15% reduction
    double rho = 1.225;
    double n = 40.0;               // rev/sec
    double D = 2.0;                // m

    double T_clean = Ct_clean * rho * n * n * D * D * D * D;
    double T_ice = Ct_ice * rho * n * n * D * D * D * D;
    double thrust_loss = T_clean - T_ice;

    TS_ASSERT_DELTA(T_clean, 3136.0, 1.0);
    TS_ASSERT_DELTA(T_ice, 2665.6, 1.0);
    TS_ASSERT_DELTA(thrust_loss, 470.4, 1.0);
  }

  // Test propeller efficiency degradation
  void testPropellerEfficiencyDegradation() {
    // Efficiency = J * Ct / Cp
    double J = 0.7;
    double Ct_clean = 0.09;
    double Cp_clean = 0.06;
    double Ct_ice = 0.08;
    double Cp_ice = 0.065;

    double eta_clean = J * Ct_clean / Cp_clean;
    double eta_ice = J * Ct_ice / Cp_ice;
    double efficiency_loss = eta_clean - eta_ice;

    TS_ASSERT_DELTA(eta_clean, 1.05, 0.01);
    TS_ASSERT_DELTA(eta_ice, 0.862, 0.001);
    TS_ASSERT_DELTA(efficiency_loss, 0.188, 0.01);
  }

  // Test propeller vibration from asymmetric ice
  void testPropellerVibrationAsymmetricIce() {
    // Ice mass asymmetry creates imbalance
    double blade_mass = 10.0;      // kg per blade
    double ice_mass_blade1 = 0.5;  // kg
    double ice_mass_blade2 = 0.1;  // kg (asymmetric)
    double imbalance = ice_mass_blade1 - ice_mass_blade2;

    TS_ASSERT_DELTA(imbalance, 0.4, epsilon);

    // Centrifugal force imbalance at tip
    double radius = 1.0;           // m
    double omega = 250.0;          // rad/s
    double F_imbalance = imbalance * radius * omega * omega;

    TS_ASSERT_DELTA(F_imbalance, 25000.0, 1.0);
  }

  /***************************************************************************
   * Tail Plane Icing Tests
   ***************************************************************************/

  // Test tail plane stall from ice
  void testTailPlaneStallFromIce() {
    // Tail CL at given AOA
    double alpha_tail = 5.0;       // degrees
    double dCL_dalpha_clean = 0.08; // per degree
    double dCL_dalpha_ice = 0.06;   // 25% reduction

    double CL_tail_clean = dCL_dalpha_clean * alpha_tail;
    double CL_tail_ice = dCL_dalpha_ice * alpha_tail;

    TS_ASSERT_DELTA(CL_tail_clean, 0.40, epsilon);
    TS_ASSERT_DELTA(CL_tail_ice, 0.30, epsilon);
  }

  // Test tail downforce with ice
  void testTailDownforceWithIce() {
    // L_tail = 0.5 * rho * V^2 * S_tail * CL_tail
    double rho = 1.225;
    double velocity = 60.0;
    double S_tail = 3.0;           // m^2
    double CL_tail_clean = -0.3;   // negative (downforce)
    double CL_tail_ice = -0.22;    // less downforce

    double L_tail_clean = 0.5 * rho * velocity * velocity * S_tail * CL_tail_clean;
    double L_tail_ice = 0.5 * rho * velocity * velocity * S_tail * CL_tail_ice;

    TS_ASSERT_DELTA(L_tail_clean, -1984.5, 1.0);
    TS_ASSERT_DELTA(L_tail_ice, -1455.3, 1.0);
  }

  // Test pitch moment change from tail ice
  void testPitchMomentChangeTailIce() {
    // M = L_tail * l_tail (moment arm)
    double L_tail_clean = -2000.0; // N
    double L_tail_ice = -1500.0;   // N (reduced)
    double l_tail = 5.0;           // m

    double M_clean = L_tail_clean * l_tail;
    double M_ice = L_tail_ice * l_tail;
    double delta_M = M_ice - M_clean;

    TS_ASSERT_DELTA(M_clean, -10000.0, epsilon);
    TS_ASSERT_DELTA(M_ice, -7500.0, epsilon);
    TS_ASSERT_DELTA(delta_M, 2500.0, epsilon);  // Nose-up moment
  }

  /***************************************************************************
   * Ice Type Tests (Freezing Rain vs Rime)
   ***************************************************************************/

  // Test rime ice formation conditions
  void testRimeIceFormationConditions() {
    // Rime ice: small droplets, rapid freeze, opaque
    // Conditions: T < -10°C, small droplets (< 20 μm)

    double T_static = -15.0;       // °C
    double MVD = 15.0;             // μm

    bool rime_conditions = (T_static < -10.0) && (MVD < 20.0);
    TS_ASSERT_EQUALS(rime_conditions, true);
  }

  // Test clear ice formation conditions
  void testClearIceFormationConditions() {
    // Clear ice: large droplets, slow freeze, transparent, horns
    // Conditions: T near 0°C, large droplets (> 40 μm)

    double T_static = -3.0;        // °C
    double MVD = 50.0;             // μm

    bool clear_conditions = (T_static > -10.0) && (MVD > 40.0);
    TS_ASSERT_EQUALS(clear_conditions, true);
  }

  // Test mixed ice formation
  void testMixedIceFormation() {
    // Mixed ice: combination of rime and clear
    // Conditions: -10°C < T < -5°C, medium droplets

    double T_static = -7.0;        // °C
    double MVD = 30.0;             // μm

    bool mixed_conditions = (T_static > -10.0) && (T_static < -5.0)
                            && (MVD > 20.0) && (MVD < 40.0);
    TS_ASSERT_EQUALS(mixed_conditions, true);
  }

  // Test freezing rain (supercooled large droplets)
  void testFreezingRainIcing() {
    // SLD: droplets > 50 μm
    // Much higher accretion rates than typical icing
    double MVD_SLD = 100.0;        // μm
    double LWC = 0.8;              // g/m^3

    // Collection efficiency near 1.0 for SLD
    double E_SLD = 0.95;
    double velocity = 80.0;
    double area = 20.0;

    double accretion_rate = LWC * velocity * area * E_SLD;
    TS_ASSERT_DELTA(accretion_rate, 1216.0, 1.0);
  }

  /***************************************************************************
   * Temperature/Humidity Conditions Tests
   ***************************************************************************/

  // Test icing temperature envelope
  void testIcingTemperatureEnvelope() {
    // Icing typically occurs between 0°C and -40°C
    // Most severe: 0°C to -15°C

    // At 0°C
    double T1 = 0.0;
    bool ice_possible_1 = (T1 <= 0.0) && (T1 >= -40.0);
    TS_ASSERT_EQUALS(ice_possible_1, true);

    // At -20°C
    double T2 = -20.0;
    bool ice_possible_2 = (T2 <= 0.0) && (T2 >= -40.0);
    TS_ASSERT_EQUALS(ice_possible_2, true);

    // At -45°C (below icing range)
    double T3 = -45.0;
    bool ice_possible_3 = (T3 <= 0.0) && (T3 >= -40.0);
    TS_ASSERT_EQUALS(ice_possible_3, false);
  }

  // Test relative humidity effect on icing
  void testRelativeHumidityEffect() {
    // Higher RH increases icing severity
    // LWC approximately proportional to RH above saturation

    double RH_low = 0.70;          // 70%
    double RH_high = 0.95;         // 95%

    // Simplified LWC relationship
    double LWC_base = 0.3;
    double LWC_low = LWC_base * (RH_low / 0.95);
    double LWC_high = LWC_base * (RH_high / 0.95);

    TS_ASSERT_DELTA(LWC_low, 0.221, 0.001);
    TS_ASSERT_DELTA(LWC_high, 0.300, 0.001);
  }

  // Test saturation vapor pressure over ice
  void testSaturationVaporPressureIce() {
    // Magnus formula for ice: e_s = 6.112 * exp(22.46 * T / (272.62 + T))
    // T in Celsius

    double T = -10.0;              // °C
    double e_s = 6.112 * exp(22.46 * T / (272.62 + T));

    // 6.112 * exp(-224.6/262.62) ≈ 2.60 hPa
    TS_ASSERT_DELTA(e_s, 2.60, 0.01);
  }

  /***************************************************************************
   * Anti-Ice/De-Ice Heat Requirements Tests
   ***************************************************************************/

  // Test heat required to prevent ice formation
  void testAntiIceHeatRequirement() {
    // Heat flux: q = m_dot_ice * (L_fusion + Cp_ice * ΔT)
    // L_fusion = 334 kJ/kg
    double accretion_rate = 0.5;   // kg/s
    double L_fusion = 334000.0;    // J/kg
    double Cp_ice = 2100.0;        // J/(kg·K)
    double delta_T = 20.0;         // K (heat ice to above freezing)

    double heat_flux = accretion_rate * (L_fusion + Cp_ice * delta_T);

    TS_ASSERT_DELTA(heat_flux, 188000.0, 100.0);  // 188 kW
  }

  // Test evaporative heat requirement
  void testEvaporativeAntiIceHeat() {
    // Must evaporate water to prevent freezing
    // q_evap = m_dot * L_vap where L_vap = 2257 kJ/kg
    double water_flow = 0.3;       // kg/s
    double L_vap = 2257000.0;      // J/kg

    double heat_evap = water_flow * L_vap;

    TS_ASSERT_DELTA(heat_evap, 677100.0, 100.0);  // 677 kW
  }

  // Test de-ice cyclic heat pulse
  void testDeIceCyclicHeatPulse() {
    // De-ice: periodic heat pulse to shed ice
    // Energy = ice_mass * (Cp_ice * ΔT + L_fusion)
    double ice_mass = 2.0;         // kg
    double Cp_ice = 2100.0;        // J/(kg·K)
    double delta_T = 30.0;         // K (heat from -20°C to +10°C)
    double L_fusion = 334000.0;    // J/kg

    double energy_pulse = ice_mass * (Cp_ice * delta_T + L_fusion);

    TS_ASSERT_DELTA(energy_pulse, 794000.0, 100.0);  // 794 kJ
  }

  // Test pneumatic boot power requirement
  void testPneumaticBootPower() {
    // Pneumatic system work: W = P * V
    // P = pressure, V = volume inflated
    double pressure = 200000.0;    // Pa (2 bar)
    double volume = 0.05;          // m^3

    double work = pressure * volume;

    TS_ASSERT_DELTA(work, 10000.0, epsilon);  // 10 kJ per cycle
  }

  // Test electrical heating power density
  void testElectricalHeatingPowerDensity() {
    // Power density: q" = P / A (W/m^2)
    double power = 5000.0;         // W
    double area = 2.0;             // m^2

    double power_density = power / area;

    TS_ASSERT_DELTA(power_density, 2500.0, epsilon);  // 2500 W/m^2

    // Typical values: 2000-5000 W/m^2 for leading edge
    TS_ASSERT(power_density >= 2000.0);
    TS_ASSERT(power_density <= 5000.0);
  }

  // Test bleed air anti-ice system
  void testBleedAirAntiIceSystem() {
    // Bleed air from engine provides heat
    // Q = m_dot_air * Cp_air * (T_bleed - T_surface)
    double m_dot_air = 0.5;        // kg/s
    double Cp_air = 1005.0;        // J/(kg·K)
    double T_bleed = 473.15;       // K (200°C)
    double T_surface = 273.15;     // K (0°C)

    double heat_transfer = m_dot_air * Cp_air * (T_bleed - T_surface);

    TS_ASSERT_DELTA(heat_transfer, 100500.0, 100.0);  // 100.5 kW
  }
};

/*******************************************************************************
 * Additional FGIcing Tests (30 new tests)
 ******************************************************************************/

class FGIcingAdditionalTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Ice Shedding Dynamics Tests
   ***************************************************************************/

  // Test 46: Ice shedding threshold force
  void testIceSheddingThresholdForce() {
    // Ice sheds when aerodynamic/centrifugal force > adhesion force
    // F_adhesion = sigma_adhesion * contact_area
    double sigma_adhesion = 200000.0;  // Pa (ice-metal adhesion)
    double contact_area = 0.01;         // m^2

    double F_adhesion = sigma_adhesion * contact_area;
    TS_ASSERT_DELTA(F_adhesion, 2000.0, epsilon);

    // Centrifugal force on propeller ice
    double ice_mass = 0.2;              // kg
    double radius = 1.0;                // m
    double omega = 250.0;               // rad/s
    double F_centrifugal = ice_mass * radius * omega * omega;

    TS_ASSERT_DELTA(F_centrifugal, 12500.0, 1.0);
    TS_ASSERT(F_centrifugal > F_adhesion);  // Ice will shed
  }

  // Test 47: Ice shedding trajectory
  void testIceSheddingTrajectory() {
    // Ice leaves surface with velocity and follows ballistic path
    double V_shed = 80.0;               // m/s (airspeed at shedding)
    double angle = 30.0 * M_PI / 180.0; // radians
    double g = 9.81;                    // m/s^2

    // Horizontal and vertical components
    double Vx = V_shed * cos(angle);
    double Vy = V_shed * sin(angle);

    TS_ASSERT_DELTA(Vx, 69.28, 0.01);
    TS_ASSERT_DELTA(Vy, 40.0, 0.01);

    // Time to hit fuselage 2m below
    double h = 2.0;
    double t = (Vy + sqrt(Vy * Vy + 2 * g * h)) / g;
    TS_ASSERT_DELTA(t, 8.20, 0.05);
  }

  // Test 48: De-ice boot shedding efficiency
  void testDeIceBootSheddingEfficiency() {
    // Boot efficiency varies with ice thickness and type
    double ice_thickness = 0.02;        // m

    // Rime ice - easier to shed
    double efficiency_rime = 0.95;
    double shed_rime = ice_thickness * efficiency_rime;
    TS_ASSERT_DELTA(shed_rime, 0.019, 0.001);

    // Clear ice - harder to shed (adheres better)
    double efficiency_clear = 0.70;
    double shed_clear = ice_thickness * efficiency_clear;
    TS_ASSERT_DELTA(shed_clear, 0.014, 0.001);

    // Residual ice after cycling
    double residual_rime = ice_thickness - shed_rime;
    double residual_clear = ice_thickness - shed_clear;
    TS_ASSERT_DELTA(residual_rime, 0.001, 0.001);
    TS_ASSERT_DELTA(residual_clear, 0.006, 0.001);
  }

  /***************************************************************************
   * Ice Protection System Failure Tests
   ***************************************************************************/

  // Test 49: Single engine bleed failure effect
  void testSingleEngineBleedFailure() {
    // With one engine bleed failed, heat capacity reduced by ~50%
    double heat_available_normal = 150000.0;  // W
    double heat_available_failed = 75000.0;   // W

    // Required heat for wing anti-ice
    double heat_required = 100000.0;          // W

    bool sufficient_normal = heat_available_normal >= heat_required;
    bool sufficient_failed = heat_available_failed >= heat_required;

    TS_ASSERT_EQUALS(sufficient_normal, true);
    TS_ASSERT_EQUALS(sufficient_failed, false);
  }

  // Test 50: Ice detector failure logic
  void testIceDetectorFailureLogic() {
    // Dual ice detector system
    bool detector1_fault = true;
    bool detector2_fault = false;

    // Single fault: system continues with remaining detector
    bool system_operational = !detector1_fault || !detector2_fault;
    TS_ASSERT_EQUALS(system_operational, true);

    // Dual fault: pilot must rely on visual detection
    detector2_fault = true;
    system_operational = !detector1_fault || !detector2_fault;
    TS_ASSERT_EQUALS(system_operational, false);
  }

  // Test 51: Electrical ice protection power loss
  void testElectricalIceProtectionPowerLoss() {
    // Generator failure reduces available power
    double power_total_normal = 50000.0;      // W
    double power_essential = 20000.0;         // W
    double power_ice_protection = 25000.0;    // W

    // After generator failure
    double power_available = power_total_normal * 0.5;
    bool ice_protection_available = (power_available - power_essential) >= power_ice_protection;

    TS_ASSERT_DELTA(power_available, 25000.0, epsilon);
    TS_ASSERT_EQUALS(ice_protection_available, false);
  }

  /***************************************************************************
   * Wing Leading Edge Ice Shape Tests
   ***************************************************************************/

  // Test 52: Upper horn ice shape aerodynamic effect
  void testUpperHornIceShape() {
    // Upper horn creates flow separation on upper surface
    // CL loss more severe than lower horn
    double CL_clean = 1.4;
    double horn_height = 0.03;          // m
    double chord = 2.0;                 // m

    // Upper horn penalty factor
    double k_upper = 15.0;
    double CL_reduction_upper = k_upper * (horn_height / chord);
    double CL_upper_horn = CL_clean * (1.0 - CL_reduction_upper);

    TS_ASSERT_DELTA(CL_reduction_upper, 0.225, 0.001);
    TS_ASSERT_DELTA(CL_upper_horn, 1.085, 0.01);
  }

  // Test 53: Double horn ice shape effect
  void testDoubleHornIceShape() {
    // Double horn (upper and lower) worst case
    double CD_clean = 0.030;
    double horn_height = 0.025;         // m
    double chord = 2.0;                 // m

    // Double horn drag coefficient
    // delta_CD = k * (h/c)^1.5 = 3.0 * (0.0125)^1.5 = 3.0 * 0.00140 = 0.0042
    double k_double = 3.0;
    double delta_CD = k_double * pow(horn_height / chord, 1.5);
    double CD_double_horn = CD_clean + delta_CD;

    TS_ASSERT_DELTA(delta_CD, 0.0042, 0.0001);
    TS_ASSERT_DELTA(CD_double_horn, 0.0342, 0.0001);
  }

  // Test 54: Streamwise ice (minimal performance impact)
  void testStreamwiseIce() {
    // Streamwise ice forms in cold conditions, lower drag penalty
    double CD_clean = 0.025;
    double ice_thickness = 0.02;        // m
    double chord = 2.0;                 // m

    // Streamwise ice has lower drag coefficient
    double k_streamwise = 0.2;
    double delta_CD = k_streamwise * pow(ice_thickness / chord, 2);
    double CD_streamwise = CD_clean + delta_CD;

    TS_ASSERT_DELTA(delta_CD, 0.00002, 0.00001);
    TS_ASSERT_DELTA(CD_streamwise, 0.02502, 0.0001);
  }

  /***************************************************************************
   * Runback Ice Tests
   ***************************************************************************/

  // Test 55: Runback ice from hot anti-ice system
  void testRunbackIceFormation() {
    // Water melted at leading edge freezes behind heated zone
    double water_flow = 0.01;           // kg/s
    double heated_zone_end = 0.05;      // 5% chord
    double ambient_temp = -20.0;        // °C

    // Water refreezes when leaving heated zone
    // Runback distance depends on temperature and skin thermal conductivity
    double k_runback = 0.02;            // m/°C
    double runback_distance = k_runback * std::abs(ambient_temp);

    TS_ASSERT_DELTA(runback_distance, 0.4, 0.01);  // 40 cm runback
  }

  // Test 56: Runback ice ridge height
  void testRunbackIceRidgeHeight() {
    // Ridge height depends on water flow and freeze rate
    double water_flow = 0.005;          // kg/s per meter span
    double freeze_fraction = 0.3;       // 30% freezes at ridge
    double rho_ice = 917.0;             // kg/m^3
    double ridge_width = 0.02;          // m
    double time = 300.0;                // 5 minutes

    // Ice mass at ridge
    double ice_mass = water_flow * freeze_fraction * time;
    // Ridge cross-section area
    double area = ice_mass / rho_ice;
    // Approximate ridge height
    double ridge_height = area / ridge_width;

    TS_ASSERT_DELTA(ice_mass, 0.45, 0.01);
    TS_ASSERT_DELTA(ridge_height, 0.0245, 0.001);
  }

  /***************************************************************************
   * Ice Crystal Icing Tests
   ***************************************************************************/

  // Test 57: High altitude ice crystal icing
  void testHighAltitudeIceCrystalIcing() {
    // Ice crystals ingested by engine at high altitude
    // Total Water Content (TWC) instead of LWC
    double TWC = 2.0;                   // g/m^3 (typical value)
    double velocity = 200.0;            // m/s
    double inlet_area = 0.5;            // m^2

    // Ice crystal ingestion rate
    double ingestion_rate = TWC * velocity * inlet_area;
    TS_ASSERT_DELTA(ingestion_rate, 200.0, 1.0);  // 200 g/s
  }

  // Test 58: Engine compressor ice buildup
  void testCompressorIceBuildup() {
    // Ice crystals melt and refreeze on compressor stages
    double crystal_rate = 0.1;          // kg/s
    double melt_fraction = 0.8;         // 80% melts
    double refreeze_fraction = 0.3;     // 30% refreezes

    double ice_buildup_rate = crystal_rate * melt_fraction * refreeze_fraction;
    TS_ASSERT_DELTA(ice_buildup_rate, 0.024, 0.001);  // 24 g/s
  }

  // Test 59: Probe ice crystal blocking
  void testProbeIceCrystalBlocking() {
    // Pitot/TAT probes can be blocked by ice crystals
    double probe_area = 0.0001;         // m^2
    double TWC = 3.0;                   // g/m^3
    double velocity = 180.0;            // m/s

    double ice_mass_rate = TWC * velocity * probe_area;
    // Time to accumulate blocking mass
    double blocking_mass = 0.01;        // kg
    double time_to_block = blocking_mass / (ice_mass_rate / 1000.0);

    TS_ASSERT_DELTA(ice_mass_rate, 0.054, 0.001);  // 54 mg/s
    TS_ASSERT_DELTA(time_to_block, 185.2, 1.0);    // ~3 minutes
  }

  /***************************************************************************
   * Ground Icing Tests
   ***************************************************************************/

  // Test 60: Frost formation on cold-soaked wing
  void testFrostFormationColdSoakedWing() {
    // Aircraft descends with cold fuel, lands in warm humid air
    double T_fuel = -30.0;              // °C
    double T_ambient = 15.0;            // °C
    double T_dewpoint = 10.0;           // °C

    // Wing skin temperature approximation
    double T_skin = T_fuel + 0.5 * (T_ambient - T_fuel);
    TS_ASSERT_DELTA(T_skin, -7.5, 0.1);

    // Frost forms if skin temp below dewpoint
    bool frost_forms = T_skin < T_dewpoint;
    TS_ASSERT_EQUALS(frost_forms, true);
  }

  // Test 61: De-icing fluid holdover time
  void testDeIcingFluidHoldoverTime() {
    // Type I fluid holdover in light snow
    double temp = -5.0;                 // °C
    double precipitation_rate = 1.0;    // mm/hr (light snow)

    // Approximate holdover time (simplified model)
    // Holdover = base_time * temp_factor / precip_factor
    double base_time = 15.0;            // minutes
    double temp_factor = 1.0 + 0.02 * temp;  // decreases with cold
    double precip_factor = 1.0 + 0.5 * precipitation_rate;

    double holdover = base_time * temp_factor / precip_factor;
    TS_ASSERT_DELTA(holdover, 9.0, 0.5);  // ~9 minutes
  }

  // Test 62: Anti-icing fluid freeze point
  void testAntiIcingFluidFreezePoint() {
    // Propylene glycol concentration vs freeze point
    double glycol_fraction = 0.50;      // 50% glycol

    // Approximate freeze point (°C)
    // T_freeze ≈ -52 * glycol_fraction for 0.3 < fraction < 0.6
    double T_freeze = -52.0 * glycol_fraction;
    TS_ASSERT_DELTA(T_freeze, -26.0, 1.0);
  }

  /***************************************************************************
   * Performance Degradation Summary Tests
   ***************************************************************************/

  // Test 63: Takeoff distance increase with contaminated wing
  void testTakeoffDistanceContaminatedWing() {
    // CLmax reduction increases rotation speed and distance
    double CLmax_clean = 2.0;
    double CLmax_frost = 1.7;           // 15% reduction from frost

    // V_rot proportional to sqrt(1/CLmax)
    double V_rot_ratio = sqrt(CLmax_clean / CLmax_frost);
    TS_ASSERT_DELTA(V_rot_ratio, 1.085, 0.001);

    // Takeoff distance roughly proportional to V^2
    double distance_increase = V_rot_ratio * V_rot_ratio - 1.0;
    TS_ASSERT_DELTA(distance_increase, 0.176, 0.01);  // 17.6% increase
  }

  // Test 64: Climb gradient reduction
  void testClimbGradientReductionIce() {
    // Climb gradient = (T - D) / W
    double thrust = 50000.0;            // N
    double drag_clean = 20000.0;        // N
    double drag_ice = 26000.0;          // N (30% increase)
    double weight = 100000.0;           // N

    double gradient_clean = (thrust - drag_clean) / weight;
    double gradient_ice = (thrust - drag_ice) / weight;
    double gradient_loss = gradient_clean - gradient_ice;

    TS_ASSERT_DELTA(gradient_clean, 0.30, 0.001);
    TS_ASSERT_DELTA(gradient_ice, 0.24, 0.001);
    TS_ASSERT_DELTA(gradient_loss, 0.06, 0.001);  // 6% gradient loss
  }

  // Test 65: Range reduction from icing drag
  void testRangeReductionIcing() {
    // Range = (eta * L/D) * ln(W_initial/W_final) * (1/c)
    // Simplified: Range proportional to L/D
    double L_D_clean = 15.0;
    double L_D_ice = 11.0;              // Reduced L/D

    double range_ratio = L_D_ice / L_D_clean;
    double range_reduction = (1.0 - range_ratio) * 100.0;

    TS_ASSERT_DELTA(range_ratio, 0.733, 0.001);
    TS_ASSERT_DELTA(range_reduction, 26.7, 0.1);  // 26.7% range reduction
  }

  /***************************************************************************
   * Ice Detection Tests
   ***************************************************************************/

  // Test 66: Vibrating probe ice detection frequency shift
  void testVibratingProbeIceDetection() {
    // Ice accumulation changes probe resonant frequency
    // f = (1/2π) * sqrt(k/m), solving for k at f=4000Hz, m=0.001kg:
    // k = (2π*f)^2 * m = (25133)^2 * 0.001 = 631655 N/m
    double f_clean = 4000.0;            // Hz
    double m_probe = 0.001;             // kg
    double k_spring = 631655.0;         // N/m (gives ~4kHz)

    // Verify clean frequency: f = (1/2π) * sqrt(k/m)
    double f_calc = sqrt(k_spring / m_probe) / (2.0 * M_PI);
    TS_ASSERT_DELTA(f_calc, f_clean, 10.0);

    // With ice mass
    double m_ice = 0.0001;              // 0.1 g ice
    double m_total = m_probe + m_ice;
    double f_ice = sqrt(k_spring / m_total) / (2.0 * M_PI);
    double frequency_shift = f_clean - f_ice;

    TS_ASSERT_DELTA(f_ice, 3814.0, 10.0);
    TS_ASSERT(frequency_shift > 100.0);  // Detectable shift
  }

  // Test 67: Optical ice detector reflectivity
  void testOpticalIceDetectorReflectivity() {
    // Clear ice vs rime ice optical properties
    double reflectivity_clean_surface = 0.05;
    double reflectivity_clear_ice = 0.10;
    double reflectivity_rime_ice = 0.60;

    // Detection threshold
    double threshold = 0.08;

    bool clear_ice_detected = reflectivity_clear_ice > threshold;
    bool rime_ice_detected = reflectivity_rime_ice > threshold;

    TS_ASSERT_EQUALS(clear_ice_detected, true);
    TS_ASSERT_EQUALS(rime_ice_detected, true);
  }

  // Test 68: Total air temperature correction for icing
  void testTATCorrectionIcing() {
    // TAT = SAT * (1 + 0.2 * r * M^2) where r is recovery factor
    double SAT = 253.15;                // K (-20°C)
    double Mach = 0.5;
    double r = 0.95;                    // Typical recovery factor

    double TAT = SAT * (1.0 + 0.2 * r * Mach * Mach);
    double SAT_derived = TAT / (1.0 + 0.2 * r * Mach * Mach);

    TS_ASSERT_DELTA(TAT, 265.15, 0.5);
    TS_ASSERT_DELTA(SAT_derived, SAT, 0.01);

    // Icing possible if SAT between 0 and -40°C
    bool icing_possible = (SAT <= 273.15) && (SAT >= 233.15);
    TS_ASSERT_EQUALS(icing_possible, true);
  }

  /***************************************************************************
   * Flight Envelope Restriction Tests
   ***************************************************************************/

  // Test 69: Minimum speed increase with ice
  void testMinimumSpeedIncreaseIce() {
    // 1.3 * V_stall with ice
    double V_stall_clean = 55.0;        // m/s
    double V_stall_ice = 65.0;          // m/s (with ice)

    double V_min_clean = 1.3 * V_stall_clean;
    double V_min_ice = 1.3 * V_stall_ice;
    double speed_increase = V_min_ice - V_min_clean;

    TS_ASSERT_DELTA(V_min_clean, 71.5, 0.1);
    TS_ASSERT_DELTA(V_min_ice, 84.5, 0.1);
    TS_ASSERT_DELTA(speed_increase, 13.0, 0.1);
  }

  // Test 70: Maximum altitude with ice
  void testMaximumAltitudeWithIce() {
    // Service ceiling limited by reduced climb rate
    double climb_rate_clean = 5.0;      // m/s at altitude
    double drag_increase = 1.3;         // 30% drag increase

    // Simplified: climb rate inversely proportional to drag
    double climb_rate_ice = climb_rate_clean / drag_increase;
    TS_ASSERT_DELTA(climb_rate_ice, 3.85, 0.01);

    // If climb rate < 0.5 m/s, can't maintain altitude
    double ceiling_climb_rate = 0.5;
    // Altitude where climb rate = 0.5 is lower with ice
    bool ceiling_reduced = climb_rate_ice < climb_rate_clean;
    TS_ASSERT_EQUALS(ceiling_reduced, true);
  }

  // Test 71: Autopilot disconnect in severe icing
  void testAutopilotIcingResponse() {
    // Autopilot may disconnect due to unusual attitudes
    double pitch_rate_threshold = 5.0;  // deg/s
    double roll_rate_threshold = 10.0;  // deg/s

    // Ice-induced upset rates
    double pitch_upset = 8.0;           // deg/s (tail stall)
    double roll_upset = 15.0;           // deg/s (asymmetric ice)

    bool pitch_disconnect = pitch_upset > pitch_rate_threshold;
    bool roll_disconnect = roll_upset > roll_rate_threshold;
    bool autopilot_disconnects = pitch_disconnect || roll_disconnect;

    TS_ASSERT_EQUALS(autopilot_disconnects, true);
  }

  /***************************************************************************
   * Weight and Balance Effects Tests
   ***************************************************************************/

  // Test 72: CG shift from ice accumulation
  void testCGShiftFromIceAccumulation() {
    // Ice on horizontal tail shifts CG aft
    double aircraft_weight = 5000.0;    // kg
    double tail_moment_arm = 8.0;       // m (aft of CG)
    double ice_mass_tail = 20.0;        // kg

    // CG shift = (ice_mass * moment_arm) / total_weight
    double total_weight = aircraft_weight + ice_mass_tail;
    double cg_shift = (ice_mass_tail * tail_moment_arm) / total_weight;

    TS_ASSERT_DELTA(cg_shift, 0.0319, 0.0001);  // ~3.2 cm aft
  }

  // Test 73: Moment of inertia change from ice
  void testMomentOfInertiaChangeIce() {
    // Ice on wingtips increases roll inertia
    double I_xx_clean = 5000.0;         // kg·m^2
    double ice_mass_per_tip = 10.0;     // kg
    double tip_distance = 10.0;         // m from CL

    // Additional inertia from ice masses
    double delta_I = 2.0 * ice_mass_per_tip * tip_distance * tip_distance;
    double I_xx_ice = I_xx_clean + delta_I;
    double inertia_increase = (I_xx_ice / I_xx_clean - 1.0) * 100.0;

    TS_ASSERT_DELTA(delta_I, 2000.0, epsilon);
    TS_ASSERT_DELTA(I_xx_ice, 7000.0, epsilon);
    TS_ASSERT_DELTA(inertia_increase, 40.0, 0.1);  // 40% increase
  }

  // Test 74: Roll rate reduction from increased inertia
  void testRollRateReductionFromInertia() {
    // Roll rate for given aileron input: p = L_a / I_xx
    double L_a = 10000.0;               // N·m (roll moment)
    double I_xx_clean = 5000.0;         // kg·m^2
    double I_xx_ice = 7000.0;           // kg·m^2

    double p_clean = L_a / I_xx_clean;
    double p_ice = L_a / I_xx_ice;
    double roll_reduction = (1.0 - p_ice / p_clean) * 100.0;

    TS_ASSERT_DELTA(p_clean, 2.0, epsilon);   // rad/s^2
    TS_ASSERT_DELTA(p_ice, 1.43, 0.01);
    TS_ASSERT_DELTA(roll_reduction, 28.6, 0.1);  // 28.6% reduction
  }

  // Test 75: Total aircraft weight increase from icing
  void testTotalWeightIncreaseFromIcing() {
    // Estimate total ice accumulation on aircraft
    double wing_ice = 50.0;             // kg
    double tail_ice = 20.0;             // kg
    double fuselage_ice = 10.0;         // kg
    double engine_inlet_ice = 5.0;      // kg
    double misc_ice = 5.0;              // kg

    double total_ice_mass = wing_ice + tail_ice + fuselage_ice +
                            engine_inlet_ice + misc_ice;
    double aircraft_weight = 4000.0;    // kg
    double weight_increase_percent = (total_ice_mass / aircraft_weight) * 100.0;

    TS_ASSERT_DELTA(total_ice_mass, 90.0, epsilon);
    TS_ASSERT_DELTA(weight_increase_percent, 2.25, 0.01);  // 2.25% weight increase
  }
};

/*******************************************************************************
 * Extended FGIcing Tests (25 new tests)
 ******************************************************************************/

class FGIcingExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Advanced Ice Accretion Physics Tests
   ***************************************************************************/

  // Test 76: Modified inertia parameter for droplet impingement
  void testModifiedInertiaParameter() {
    // K = (rho_water * d^2 * V) / (18 * mu * c)
    double rho_water = 1000.0;        // kg/m^3
    double d = 20e-6;                 // m (20 μm MVD)
    double V = 100.0;                 // m/s
    double mu = 1.8e-5;               // Pa·s (air viscosity)
    double c = 2.0;                   // m (chord)

    double K = (rho_water * d * d * V) / (18.0 * mu * c);

    TS_ASSERT(K > 0);
    TS_ASSERT_DELTA(K, 0.0617, 0.001);
  }

  // Test 77: Stokes number for droplet dynamics
  void testStokesNumber() {
    // St = (rho_p * d^2 * V) / (18 * mu * L)
    double rho_p = 1000.0;            // kg/m^3
    double d = 30e-6;                 // m (30 μm)
    double V = 80.0;                  // m/s
    double mu = 1.8e-5;               // Pa·s
    double L = 0.1;                   // m (characteristic length)

    double St = (rho_p * d * d * V) / (18.0 * mu * L);

    TS_ASSERT_DELTA(St, 2.22, 0.01);
    TS_ASSERT(St > 1.0);  // Droplets follow streamlines poorly
  }

  // Test 78: Weber number for droplet breakup
  void testWeberNumber() {
    // We = (rho * V^2 * d) / sigma
    double rho = 1.225;               // kg/m^3
    double V = 100.0;                 // m/s
    double d = 50e-6;                 // m (50 μm SLD)
    double sigma = 0.073;             // N/m (surface tension)

    double We = (rho * V * V * d) / sigma;

    TS_ASSERT_DELTA(We, 8.39, 0.01);
  }

  /***************************************************************************
   * Ice Roughness Correlation Tests
   ***************************************************************************/

  // Test 79: Sand grain roughness equivalent
  void testSandGrainRoughnessEquivalent() {
    // ks_eq = 0.5 * peak_height for ice roughness
    double peak_height = 0.003;       // m (3 mm peaks)
    double ks_eq = 0.5 * peak_height;

    TS_ASSERT_DELTA(ks_eq, 0.0015, epsilon);
  }

  // Test 80: Roughness Reynolds number
  void testRoughnessReynoldsNumber() {
    // Re_k = (u_tau * ks) / nu
    double u_tau = 2.0;               // m/s (friction velocity)
    double ks = 0.002;                // m (roughness height)
    double nu = 1.5e-5;               // m^2/s (kinematic viscosity)

    double Re_k = (u_tau * ks) / nu;

    TS_ASSERT_DELTA(Re_k, 266.7, 1.0);

    // Fully rough regime if Re_k > 70
    bool fully_rough = Re_k > 70.0;
    TS_ASSERT(fully_rough);
  }

  // Test 81: Skin friction increase from roughness
  void testSkinFrictionRoughnessIncrease() {
    // Cf_rough / Cf_smooth = (1 + 0.32 * (ks/delta)^0.7)
    double ks = 0.002;                // m
    double delta = 0.02;              // m (boundary layer thickness)
    double Cf_smooth = 0.003;

    double ratio = 1.0 + 0.32 * pow(ks / delta, 0.7);
    double Cf_rough = Cf_smooth * ratio;

    TS_ASSERT_DELTA(ratio, 1.064, 0.01);
    TS_ASSERT_DELTA(Cf_rough, 0.00319, 0.0001);
  }

  /***************************************************************************
   * Ice Shape Prediction Tests
   ***************************************************************************/

  // Test 82: Ice horn angle prediction
  void testIceHornAnglePrediction() {
    // Horn angle related to stagnation point location
    double alpha = 5.0 * M_PI / 180.0;  // AOA in radians
    double Mach = 0.4;

    // Simplified horn angle model
    double horn_angle = 45.0 + 2.0 * alpha * 180.0 / M_PI;

    TS_ASSERT_DELTA(horn_angle, 55.0, 1.0);  // degrees
  }

  // Test 83: Ice thickness distribution along chord
  void testIceThicknessDistribution() {
    // Maximum thickness at stagnation, decreasing aft
    double t_stag = 0.03;             // m (max thickness at leading edge)
    double s = 0.1;                   // m (surface distance from stagnation)
    double s_limit = 0.2;             // m (ice limit location)

    // Exponential decay model
    double t = t_stag * exp(-3.0 * s / s_limit);

    TS_ASSERT_DELTA(t, 0.00669, 0.001);
    TS_ASSERT(t < t_stag);
  }

  // Test 84: Glaze ice water film thickness
  void testGlazeIceWaterFilmThickness() {
    // Film thickness from water mass balance
    double LWC = 0.5;                 // g/m^3
    double V = 80.0;                  // m/s
    double E = 0.8;                   // collection efficiency
    double mu_water = 0.001;          // Pa·s
    double rho_water = 1000.0;        // kg/m^3

    // Approximate film thickness
    double water_flux = LWC * V * E / 1000.0;  // kg/(m^2·s)
    double h_film = pow(3.0 * mu_water * water_flux / (rho_water * 9.81), 1.0/3.0);

    TS_ASSERT(h_film > 0);
    TS_ASSERT(h_film < 0.01);  // Less than 10 mm (thin film assumption)
  }

  /***************************************************************************
   * Thermal Anti-Ice System Tests
   ***************************************************************************/

  // Test 85: Running wet anti-ice heat transfer
  void testRunningWetHeatTransfer() {
    // Heat balance: q_in = q_conv + q_evap + q_runback
    double h_conv = 500.0;            // W/(m^2·K) heat transfer coefficient
    double T_surface = 283.15;        // K (10°C)
    double T_recovery = 263.15;       // K (-10°C)
    double area = 0.5;                // m^2

    double q_conv = h_conv * (T_surface - T_recovery) * area;

    TS_ASSERT_DELTA(q_conv, 5000.0, 10.0);  // 5 kW
  }

  // Test 86: Evaporative anti-ice effectiveness
  void testEvaporativeAntiIceEffectiveness() {
    // Evaporative system: all water evaporated
    double water_catch = 0.01;        // kg/s
    double L_vap = 2.26e6;            // J/kg
    double q_available = 30000.0;     // W

    double q_required = water_catch * L_vap;
    double effectiveness = std::min(1.0, q_available / q_required);

    TS_ASSERT_DELTA(q_required, 22600.0, 100.0);
    TS_ASSERT_DELTA(effectiveness, 1.0, 0.01);  // Fully evaporative
  }

  // Test 87: Heated zone leading edge extent
  void testHeatedZoneExtent() {
    // Required heated zone for running wet operation
    double s_impingement = 0.08;      // m (impingement limit)
    double safety_factor = 1.5;

    double s_heated = s_impingement * safety_factor;

    TS_ASSERT_DELTA(s_heated, 0.12, 0.001);  // 12 cm heated zone
  }

  /***************************************************************************
   * Electrothermal De-Ice Tests
   ***************************************************************************/

  // Test 88: Parting strip heat requirement
  void testPartingStripHeatRequirement() {
    // Parting strip melts ice interface for shedding
    double ice_thickness = 0.01;      // m
    double rho_ice = 917.0;           // kg/m^3
    double area = 0.02;               // m^2
    double L_fusion = 334000.0;       // J/kg
    double cycle_time = 30.0;         // s

    double ice_mass = rho_ice * ice_thickness * area;
    double energy = ice_mass * L_fusion;
    double power = energy / cycle_time;

    TS_ASSERT_DELTA(ice_mass, 0.183, 0.001);
    TS_ASSERT_DELTA(power, 2043.0, 10.0);  // ~2 kW
  }

  // Test 89: Shed zone sequencing
  void testShedZoneSequencing() {
    // Multiple zones heated in sequence
    int num_zones = 5;
    double cycle_period = 120.0;      // s
    double zone_on_time = 20.0;       // s

    double zone_interval = cycle_period / num_zones;
    double duty_cycle = zone_on_time / zone_interval;

    TS_ASSERT_DELTA(zone_interval, 24.0, epsilon);
    TS_ASSERT_DELTA(duty_cycle, 0.833, 0.01);
  }

  // Test 90: Intercycle ice buildup
  void testIntercycleIceBuildup() {
    // Ice accumulated between de-ice cycles
    double accretion_rate = 0.001;    // m/s
    double cycle_period = 60.0;       // s

    double intercycle_thickness = accretion_rate * cycle_period;

    TS_ASSERT_DELTA(intercycle_thickness, 0.06, 0.001);  // 6 cm
  }

  /***************************************************************************
   * Hybrid Ice Protection Tests
   ***************************************************************************/

  // Test 91: Electro-impulse de-ice force
  void testElectroImpulseForce() {
    // Electromagnetic coil generates impulsive force
    double I_peak = 1000.0;           // A (peak current)
    double B = 0.5;                   // T (magnetic field)
    double L = 0.2;                   // m (coil length)

    double F_impulse = B * I_peak * L;

    TS_ASSERT_DELTA(F_impulse, 100.0, epsilon);  // 100 N
  }

  // Test 92: Ice adhesion vs impulse force
  void testIceAdhesionVsImpulse() {
    double sigma_adhesion = 150000.0; // Pa
    double contact_area = 0.005;      // m^2
    double F_adhesion = sigma_adhesion * contact_area;

    double F_impulse = 1000.0;        // N

    bool ice_sheds = F_impulse > F_adhesion;

    TS_ASSERT_DELTA(F_adhesion, 750.0, epsilon);
    TS_ASSERT(ice_sheds);
  }

  // Test 93: Fluid freezing point depressant
  void testFluidFreezingPointDepressant() {
    // TKS fluid (glycol) lowers freezing point
    double T_freeze_water = 273.15;   // K
    double glycol_concentration = 0.4;

    // Raoult's law approximation
    double delta_T = 52.0 * glycol_concentration;  // K depression
    double T_freeze_mix = T_freeze_water - delta_T;

    TS_ASSERT_DELTA(T_freeze_mix, 252.35, 0.1);  // -20.8°C
  }

  /***************************************************************************
   * Windshield Icing Tests
   ***************************************************************************/

  // Test 94: Windshield heat load
  void testWindshieldHeatLoad() {
    // Heat required to keep windshield clear
    double area = 0.3;                // m^2
    double h_conv = 200.0;            // W/(m^2·K)
    double T_surface = 283.15;        // K (10°C)
    double T_ambient = 253.15;        // K (-20°C)

    double q_heat = h_conv * area * (T_surface - T_ambient);

    TS_ASSERT_DELTA(q_heat, 1800.0, 10.0);  // 1.8 kW
  }

  // Test 95: Windshield rain repellent effectiveness
  void testWindshieldRainRepellent() {
    // Contact angle determines water beading
    double contact_angle_clean = 70.0;    // degrees
    double contact_angle_treated = 110.0; // degrees (hydrophobic)

    bool hydrophobic = contact_angle_treated > 90.0;

    TS_ASSERT(hydrophobic);
    TS_ASSERT(contact_angle_treated > contact_angle_clean);
  }

  /***************************************************************************
   * Engine Ice Protection Tests
   ***************************************************************************/

  // Test 96: Engine anti-ice bleed air penalty
  void testEngineAntiIceBleedPenalty() {
    // Bleed air reduces available thrust
    double thrust_clean = 25000.0;    // N
    double bleed_fraction = 0.05;     // 5% bleed

    // Thrust loss approximately proportional to bleed
    double thrust_loss = thrust_clean * bleed_fraction;
    double thrust_antiice = thrust_clean - thrust_loss;

    TS_ASSERT_DELTA(thrust_loss, 1250.0, epsilon);
    TS_ASSERT_DELTA(thrust_antiice, 23750.0, epsilon);
  }

  // Test 97: Inlet lip anti-ice heat requirement
  void testInletLipAntiIceHeat() {
    double inlet_circumference = 3.0; // m
    double lip_width = 0.1;           // m
    double heat_flux = 20000.0;       // W/m^2

    double heated_area = inlet_circumference * lip_width;
    double heat_required = heat_flux * heated_area;

    TS_ASSERT_DELTA(heated_area, 0.3, epsilon);
    TS_ASSERT_DELTA(heat_required, 6000.0, epsilon);  // 6 kW
  }

  // Test 98: Fan blade ice shedding imbalance
  void testFanBladeIceShedding() {
    // Asymmetric shedding causes vibration
    double ice_mass_1 = 0.5;          // kg on blade 1
    double ice_mass_2 = 0.0;          // kg on blade 2 (shed)
    double radius = 0.6;              // m
    double omega = 500.0;             // rad/s

    double imbalance_force = (ice_mass_1 - ice_mass_2) * radius * omega * omega;

    TS_ASSERT_DELTA(imbalance_force, 75000.0, 100.0);  // 75 kN
  }

  /***************************************************************************
   * Certification and Safety Tests
   ***************************************************************************/

  // Test 99: Appendix C envelope compliance
  void testAppendixCEnvelope() {
    // FAR 25 Appendix C icing conditions
    double LWC = 0.6;                 // g/m^3
    double MVD = 20.0;                // μm
    double temp = -10.0;              // °C

    // Continuous maximum envelope
    bool within_cont_max = (LWC <= 0.8) && (MVD <= 40.0) &&
                           (temp >= -30.0) && (temp <= 0.0);

    TS_ASSERT(within_cont_max);
  }

  // Test 100: Ice shape similarity for scaling
  void testIceShapeSimilarity() {
    // Modified Ruff parameters for ice shape scaling
    double K_full = 1.0;              // Full scale inertia parameter
    double K_model = 1.0;             // Model scale (matched)
    double Ac_full = 0.5;             // Full scale accumulation
    double Ac_model = 0.5;            // Model scale (matched)

    // Ice shapes similar if parameters matched
    double K_error = std::abs(K_full - K_model) / K_full;
    double Ac_error = std::abs(Ac_full - Ac_model) / Ac_full;

    TS_ASSERT_DELTA(K_error, 0.0, 0.05);
    TS_ASSERT_DELTA(Ac_error, 0.0, 0.05);
  }
};

// ============ C172x Aircraft Icing Integration Tests ============
class FGIcingC172xTest : public CxxTest::TestSuite
{
public:

  // Test C172x atmospheric conditions for icing
  void testC172xAtmosphericConditions() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(8000.0);

    fdmex.RunIC();

    auto atm = fdmex.GetAtmosphere();
    double tempR = atm->GetTemperature();
    double tempC = (tempR - 491.67) * 5.0 / 9.0;

    TS_ASSERT(std::isfinite(tempC));
    // At 8000 ft, temp should be colder than sea level
    TS_ASSERT(tempC < 15.0);
  }

  // Test C172x flight in potential icing conditions
  void testC172xFlightInIcingConditions() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(10000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto atm = fdmex.GetAtmosphere();
    prop->InitRunning(-1);

    // Simulate flight
    for (int i = 0; i < 100; i++) {
      fdmex.Run();

      double tempR = atm->GetTemperature();
      double alt = propagate->GetAltitudeASL();

      TS_ASSERT(std::isfinite(tempR));
      TS_ASSERT(std::isfinite(alt));
    }
  }

  // Test C172x aerodynamic properties access
  void testC172xAerodynamicProperties() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    TS_ASSERT(aero != nullptr);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Check basic aerodynamic values are valid
    double lift = aero->GetLoD();
    TS_ASSERT(std::isfinite(lift));
  }

  // Test C172x flight stability at altitude
  void testC172xFlightStabilityAtAltitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(12000.0);
    ic->SetVcalibratedKtsIC(110.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    prop->InitRunning(-1);

    for (int i = 0; i < 200; i++) {
      fdmex.Run();

      double phi = propagate->GetEulerDeg(1);
      double theta = propagate->GetEulerDeg(2);

      TS_ASSERT(std::isfinite(phi));
      TS_ASSERT(std::isfinite(theta));
      TS_ASSERT(std::abs(phi) < 90.0);
      TS_ASSERT(std::abs(theta) < 90.0);
    }
  }

  // Test C172x TAS calculation at cold altitude
  void testC172xTASAtColdAltitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(15000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    double vtas = aux->GetVtrueFPS();

    TS_ASSERT(std::isfinite(vtas));
    TS_ASSERT(vtas > 0.0);
  }

  // Test C172x engine performance at high altitude cold
  void testC172xEnginePerformanceColdAltitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(12000.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 1.0);
    fcs->SetMixtureCmd(-1, 0.8);  // Lean for altitude

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double power = prop->GetEngine(0)->GetPowerAvailable();
    TS_ASSERT(std::isfinite(power));
  }

  // Test C172x approach in cold conditions
  void testC172xApproachInColdConditions() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(3000.0);
    ic->SetVcalibratedKtsIC(80.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Approach configuration
    fcs->SetThrottleCmd(-1, 0.3);
    fcs->SetDeCmd(-0.1);

    for (int i = 0; i < 200; i++) {
      fdmex.Run();

      double alt = propagate->GetAltitudeASL();
      double vcas = fdmex.GetAuxiliary()->GetVcalibratedKTS();

      TS_ASSERT(std::isfinite(alt));
      TS_ASSERT(std::isfinite(vcas));
    }
  }

  // Test C172x carburetor heat scenario
  void testC172xCarburetorHeatScenario() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto atm = fdmex.GetAtmosphere();
    prop->InitRunning(-1);

    // Check temperature conditions
    double tempR = atm->GetTemperature();
    double tempC = (tempR - 491.67) * 5.0 / 9.0;

    // Simulate reduced power (carb heat on reduces power)
    auto fcs = fdmex.GetFCS();
    fcs->SetThrottleCmd(-1, 0.5);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double power = prop->GetEngine(0)->GetPowerAvailable();
    TS_ASSERT(std::isfinite(power));
    TS_ASSERT(std::isfinite(tempC));
  }

  // Test C172x descent through icing layer
  void testC172xDescentThroughIcingLayer() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(10000.0);
    ic->SetVcalibratedKtsIC(100.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Descend
    fcs->SetThrottleCmd(-1, 0.2);
    fcs->SetDeCmd(0.1);  // Nose down

    double prevAlt = propagate->GetAltitudeASL();

    for (int i = 0; i < 300; i++) {
      fdmex.Run();

      double alt = propagate->GetAltitudeASL();
      TS_ASSERT(std::isfinite(alt));

      if (i > 50) {
        // Should be descending
        TS_ASSERT(alt <= prevAlt + 50.0);  // Allow some oscillation
      }
      prevAlt = alt;
    }
  }

  // Test C172x stall characteristics at altitude
  void testC172xStallCharacteristicsAtAltitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(8000.0);
    ic->SetVcalibratedKtsIC(60.0);  // Near stall speed

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();

      double aoa = propagate->GetAlphaDeg();
      TS_ASSERT(std::isfinite(aoa));
    }
  }

  // Test C172x extended flight at icing altitude
  void testC172xExtendedFlightIcingAltitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(9000.0);
    ic->SetVcalibratedKtsIC(110.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto atm = fdmex.GetAtmosphere();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.7);

    for (int i = 0; i < 500; i++) {
      fdmex.Run();

      double alt = propagate->GetAltitudeASL();
      double temp = atm->GetTemperature();
      double vcas = fdmex.GetAuxiliary()->GetVcalibratedKTS();

      TS_ASSERT(std::isfinite(alt));
      TS_ASSERT(std::isfinite(temp));
      TS_ASSERT(std::isfinite(vcas));
      TS_ASSERT(temp > 0.0);
    }
  }

  // Test C172x temperature gradient with altitude
  void testC172xTemperatureGradientWithAltitude() {
    double temps[3];
    double alts[] = {2000.0, 6000.0, 10000.0};

    for (int j = 0; j < 3; j++) {
      FGFDMExec fdmex;
      fdmex.LoadModel("c172x");

      auto ic = fdmex.GetIC();
      ic->SetAltitudeASLFtIC(alts[j]);

      fdmex.RunIC();

      auto atm = fdmex.GetAtmosphere();
      temps[j] = atm->GetTemperature();

      TS_ASSERT(std::isfinite(temps[j]));
      TS_ASSERT(temps[j] > 0.0);
    }

    // Temperature should decrease with altitude
    TS_ASSERT(temps[0] > temps[1]);
    TS_ASSERT(temps[1] > temps[2]);
  }
};
