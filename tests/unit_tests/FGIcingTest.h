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

#include "TestUtilities.h"

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
