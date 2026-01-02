/*******************************************************************************
 * FGThermodynamicsTest.h - Unit tests for thermodynamic calculations
 *
 * Tests fundamental thermodynamic relationships relevant to aerospace propulsion
 * including ideal gas law, isentropic flow, compressor/turbine performance,
 * combustion, nozzle flow, and heat transfer.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

// Gas constants and properties for air
namespace ThermoConstants {
    constexpr double GAMMA = 1.4;              // Specific heat ratio for air
    constexpr double R_AIR = 287.05;           // Gas constant for air (J/(kg·K))
    constexpr double CP_AIR = 1005.0;          // Specific heat at constant pressure (J/(kg·K))
    constexpr double CV_AIR = 718.0;           // Specific heat at constant volume (J/(kg·K))
    constexpr double T0_STD = 288.15;          // Standard sea level temperature (K)
    constexpr double P0_STD = 101325.0;        // Standard sea level pressure (Pa)
    constexpr double RHO0_STD = 1.225;         // Standard sea level density (kg/m³)
    constexpr double FUEL_HEATING_VALUE = 43.0e6;  // Jet fuel heating value (J/kg)
}

class FGThermodynamicsTest : public CxxTest::TestSuite
{
public:
    //==========================================================================
    // 1. IDEAL GAS LAW TESTS (~6 tests)
    //==========================================================================

    void testIdealGasLaw_PressureFromDensityTemp() {
        // P = ρ * R * T
        double rho = 1.225;  // kg/m³
        double T = 288.15;   // K
        double R = ThermoConstants::R_AIR;

        double P = rho * R * T;
        TS_ASSERT_DELTA(P, 101323.99, 2.0);  // Should equal standard pressure
    }

    void testIdealGasLaw_DensityFromPressureTemp() {
        // ρ = P / (R * T)
        double P = 101325.0;  // Pa
        double T = 288.15;    // K
        double R = ThermoConstants::R_AIR;

        double rho = P / (R * T);
        TS_ASSERT_DELTA(rho, 1.225, 0.001);  // Standard density
    }

    void testIdealGasLaw_TemperatureFromPressureDensity() {
        // T = P / (ρ * R)
        double P = 101325.0;  // Pa
        double rho = 1.225;   // kg/m³
        double R = ThermoConstants::R_AIR;

        double T = P / (rho * R);
        TS_ASSERT_DELTA(T, 288.15, 0.01);
    }

    void testIdealGasLaw_IsothermalProcess() {
        // Isothermal: P₁V₁ = P₂V₂ (T constant, ρ₁/ρ₂ = P₁/P₂)
        double P1 = 101325.0;  // Pa
        double rho1 = 1.225;   // kg/m³
        double P2 = 50000.0;   // Pa (reduced pressure)
        double T = 288.15;     // K (constant)

        double rho2 = (P2 / P1) * rho1;
        double rho2_expected = P2 / (ThermoConstants::R_AIR * T);
        TS_ASSERT_DELTA(rho2, rho2_expected, 0.001);
    }

    void testIdealGasLaw_IsochoricProcess() {
        // Isochoric (constant volume): P₁/T₁ = P₂/T₂ (ρ constant)
        double P1 = 101325.0;  // Pa
        double T1 = 288.15;    // K
        double T2 = 400.0;     // K (heated)
        double rho = 1.225;    // kg/m³ (constant)

        double P2 = P1 * (T2 / T1);
        double P2_expected = rho * ThermoConstants::R_AIR * T2;
        TS_ASSERT_DELTA(P2, P2_expected, 2.0);
    }

    void testIdealGasLaw_IsobaricProcess() {
        // Isobaric (constant pressure): V₁/T₁ = V₂/T₂ (ρ₂/ρ₁ = T₁/T₂)
        double rho1 = 1.225;   // kg/m³
        double T1 = 288.15;    // K
        double T2 = 350.0;     // K (heated)
        double P = 101325.0;   // Pa (constant)

        double rho2 = rho1 * (T1 / T2);
        double rho2_expected = P / (ThermoConstants::R_AIR * T2);
        TS_ASSERT_DELTA(rho2, rho2_expected, 0.001);
    }

    //==========================================================================
    // 2. ISENTROPIC FLOW TESTS (~8 tests)
    //==========================================================================

    void testIsentropicFlow_TemperatureRatio() {
        // T/T₀ = (1 + ((γ-1)/2) * M²)^(-1)
        double gamma = ThermoConstants::GAMMA;
        double M = 0.8;  // Mach number

        double T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        TS_ASSERT_DELTA(T_T0, 0.8865, 0.001);

        // At M=1 (sonic)
        M = 1.0;
        T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        TS_ASSERT_DELTA(T_T0, 0.8333, 0.001);
    }

    void testIsentropicFlow_PressureRatio() {
        // P/P₀ = (T/T₀)^(γ/(γ-1))
        double gamma = ThermoConstants::GAMMA;
        double M = 0.8;

        double T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double P_P0 = pow(T_T0, gamma / (gamma - 1.0));
        TS_ASSERT_DELTA(P_P0, 0.6560, 0.001);
    }

    void testIsentropicFlow_DensityRatio() {
        // ρ/ρ₀ = (T/T₀)^(1/(γ-1))
        double gamma = ThermoConstants::GAMMA;
        double M = 0.8;

        double T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double rho_rho0 = pow(T_T0, 1.0 / (gamma - 1.0));
        TS_ASSERT_DELTA(rho_rho0, 0.7400, 0.001);
    }

    void testIsentropicFlow_CriticalConditions() {
        // At M=1 (critical/sonic conditions)
        double gamma = ThermoConstants::GAMMA;
        double M = 1.0;

        double T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double P_P0 = pow(T_T0, gamma / (gamma - 1.0));
        double rho_rho0 = pow(T_T0, 1.0 / (gamma - 1.0));

        // Critical ratios
        TS_ASSERT_DELTA(T_T0, 0.8333, 0.001);     // T*/T₀
        TS_ASSERT_DELTA(P_P0, 0.5283, 0.001);     // P*/P₀
        TS_ASSERT_DELTA(rho_rho0, 0.6339, 0.001); // ρ*/ρ₀
    }

    void testIsentropicFlow_SubsonicConditions() {
        // Test subsonic flow (M < 1)
        double gamma = ThermoConstants::GAMMA;
        double M = 0.5;

        double T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double P_P0 = pow(T_T0, gamma / (gamma - 1.0));

        TS_ASSERT_DELTA(T_T0, 0.9524, 0.001);
        TS_ASSERT_DELTA(P_P0, 0.8430, 0.001);
    }

    void testIsentropicFlow_SupersonicConditions() {
        // Test supersonic flow (M > 1)
        double gamma = ThermoConstants::GAMMA;
        double M = 2.0;

        double T_T0 = 1.0 / (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double P_P0 = pow(T_T0, gamma / (gamma - 1.0));

        TS_ASSERT_DELTA(T_T0, 0.5556, 0.001);
        TS_ASSERT_DELTA(P_P0, 0.1278, 0.001);
    }

    void testIsentropicFlow_AreaMachRelation() {
        // A/A* = (1/M) * ((2/(γ+1)) * (1 + ((γ-1)/2) * M²))^((γ+1)/(2(γ-1)))
        double gamma = ThermoConstants::GAMMA;
        double M = 2.0;

        double term1 = 1.0 / M;
        double term2 = (2.0 / (gamma + 1.0)) * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        double exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));
        double A_Astar = term1 * pow(term2, exponent);

        TS_ASSERT_DELTA(A_Astar, 1.688, 0.01);
    }

    void testIsentropicFlow_StagnationToStaticTemp() {
        // T₀ = T * (1 + ((γ-1)/2) * M²)
        double gamma = ThermoConstants::GAMMA;
        double T = 250.0;  // K (static temperature)
        double M = 0.85;

        double T0 = T * (1.0 + ((gamma - 1.0) / 2.0) * M * M);
        TS_ASSERT_DELTA(T0, 286.125, 0.1);
    }

    //==========================================================================
    // 3. COMPRESSOR/TURBINE TESTS (~8 tests)
    //==========================================================================

    void testCompressor_IsentropicEfficiency() {
        // η_c = (T₀₂s - T₀₁) / (T₀₂ - T₀₁)
        // where T₀₂s is ideal (isentropic) exit temperature
        double T01 = 288.15;   // K (inlet stagnation temp)
        double PR = 25.0;      // Pressure ratio
        double gamma = ThermoConstants::GAMMA;
        double eta_c = 0.85;   // Isentropic efficiency

        // Ideal temperature rise
        double T02s = T01 * pow(PR, (gamma - 1.0) / gamma);
        // Actual temperature rise
        double T02 = T01 + (T02s - T01) / eta_c;

        TS_ASSERT_DELTA(T02s, 722.82, 1.0);
        TS_ASSERT_DELTA(T02, 799.53, 1.0);
    }

    void testCompressor_PolytropicEfficiency() {
        // Polytropic efficiency: η_p = ((γ-1)/γ) / ((n-1)/n)
        // where n is polytropic exponent
        double gamma = ThermoConstants::GAMMA;
        double eta_p = 0.90;  // Polytropic efficiency

        // Calculate equivalent isentropic efficiency for PR=25
        double PR = 25.0;
        double exponent = ((gamma - 1.0) / gamma) / eta_p;
        double T_ratio_actual = pow(PR, exponent);

        double T_ratio_ideal = pow(PR, (gamma - 1.0) / gamma);
        double eta_c = (T_ratio_ideal - 1.0) / (T_ratio_actual - 1.0);

        TS_ASSERT_DELTA(eta_c, 0.854, 0.01);  // η_c < η_p for compression
    }

    void testCompressor_WorkInput() {
        // W_c = c_p * (T₀₂ - T₀₁)
        double T01 = 288.15;   // K
        double T02 = 799.53;   // K (from previous test)
        double cp = ThermoConstants::CP_AIR;

        double W_c = cp * (T02 - T01);
        TS_ASSERT_DELTA(W_c, 513932.0, 100.0);  // J/kg
    }

    void testCompressor_PressureRatioEffect() {
        // Higher pressure ratio requires more work
        double T01 = 288.15;
        double gamma = ThermoConstants::GAMMA;
        double eta_c = 0.85;
        double cp = ThermoConstants::CP_AIR;

        double PR1 = 10.0;
        double T02s_1 = T01 * pow(PR1, (gamma - 1.0) / gamma);
        double W_c1 = cp * (T02s_1 - T01) / eta_c;

        double PR2 = 30.0;
        double T02s_2 = T01 * pow(PR2, (gamma - 1.0) / gamma);
        double W_c2 = cp * (T02s_2 - T01) / eta_c;

        TS_ASSERT(W_c2 > W_c1);  // More work needed for higher PR
        TS_ASSERT_DELTA(W_c1 / W_c2, 0.567, 0.01);
    }

    void testTurbine_IsentropicEfficiency() {
        // η_t = (T₀₃ - T₀₄) / (T₀₃ - T₀₄s)
        double T03 = 1500.0;   // K (turbine inlet temp)
        double PR = 4.5;       // Expansion pressure ratio
        double gamma = ThermoConstants::GAMMA;
        double eta_t = 0.88;   // Turbine efficiency

        // Ideal (isentropic) exit temperature
        double T04s = T03 / pow(PR, (gamma - 1.0) / gamma);
        // Actual exit temperature
        double T04 = T03 - eta_t * (T03 - T04s);

        TS_ASSERT_DELTA(T04s, 976.02, 1.0);
        TS_ASSERT_DELTA(T04, 1038.90, 1.0);
    }

    void testTurbine_WorkExtraction() {
        // W_t = c_p * (T₀₃ - T₀₄)
        double T03 = 1500.0;   // K
        double T04 = 1038.90;   // K (from previous test)
        double cp = ThermoConstants::CP_AIR;

        double W_t = cp * (T03 - T04);
        TS_ASSERT_DELTA(W_t, 463405.0, 100.0);  // J/kg
    }

    void testTurbine_PowerMatchCompressor() {
        // Turbine must provide power to drive compressor
        double mdot_core = 50.0;     // kg/s (core mass flow)
        double W_c = 513932.0;       // J/kg (compressor work)
        double eta_m = 0.99;         // Mechanical efficiency

        // Required turbine work
        double W_t_required = W_c / eta_m;
        TS_ASSERT_DELTA(W_t_required, 519123.2, 100.0);
    }

    void testCompressor_StagnationPressure() {
        // P₀₂ = P₀₁ * PR
        double P01 = 101325.0;  // Pa
        double PR = 25.0;

        double P02 = P01 * PR;
        TS_ASSERT_DELTA(P02, 2533125.0, 1.0);
    }

    //==========================================================================
    // 4. COMBUSTION TESTS (~6 tests)
    //==========================================================================

    void testCombustion_HeatAdditionConstantPressure() {
        // Q = c_p * (T₀₃ - T₀₂)
        double T02 = 799.53;   // K (compressor exit)
        double T03 = 1500.0;   // K (turbine inlet)
        double cp = ThermoConstants::CP_AIR;

        double Q = cp * (T03 - T02);
        TS_ASSERT_DELTA(Q, 703972.0, 200.0);  // J/kg
    }

    void testCombustion_FuelAirRatio() {
        // f = Q / (η_b * LHV - Q)
        // where η_b is combustion efficiency, LHV is fuel heating value
        double Q = 704172.0;   // J/kg (from previous test)
        double eta_b = 0.995;  // Combustion efficiency
        double LHV = ThermoConstants::FUEL_HEATING_VALUE;

        double f = Q / (eta_b * LHV - Q);
        TS_ASSERT_DELTA(f, 0.0166, 0.001);  // ~1.66% fuel-air ratio
    }

    void testCombustion_FuelFlowRate() {
        // mdot_f = f * mdot_a
        double f = 0.0179;      // Fuel-air ratio
        double mdot_a = 50.0;   // kg/s (air mass flow)

        double mdot_f = f * mdot_a;
        TS_ASSERT_DELTA(mdot_f, 0.895, 0.01);  // kg/s fuel flow
    }

    void testCombustion_EfficiencyEffect() {
        // Lower efficiency means more fuel needed for same temperature rise
        double T02 = 741.37;
        double T03_target = 1500.0;
        double cp = ThermoConstants::CP_AIR;
        double LHV = ThermoConstants::FUEL_HEATING_VALUE;
        double Q = cp * (T03_target - T02);

        double eta_b1 = 1.00;  // Perfect combustion
        double f1 = Q / (eta_b1 * LHV - Q);

        double eta_b2 = 0.95;  // Realistic combustion
        double f2 = Q / (eta_b2 * LHV - Q);

        TS_ASSERT(f2 > f1);  // Less efficient requires more fuel
        TS_ASSERT_DELTA(f2 / f1, 1.053, 0.01);
    }

    void testCombustion_TemperatureRiseFromFuel() {
        // ΔT = (f * η_b * LHV) / ((1 + f) * c_p)
        double f = 0.018;       // Fuel-air ratio
        double eta_b = 0.995;   // Combustion efficiency
        double LHV = ThermoConstants::FUEL_HEATING_VALUE;
        double cp = ThermoConstants::CP_AIR;

        double deltaT = (f * eta_b * LHV) / ((1.0 + f) * cp);
        TS_ASSERT_DELTA(deltaT, 752.75, 1.0);  // K temperature rise
    }

    void testCombustion_StagnationPressureLoss() {
        // P₀₃ = P₀₂ * (1 - ΔP/P₀₂)
        // Typical combustor pressure loss: 3-5%
        double P02 = 2533125.0;  // Pa
        double pressure_loss_fraction = 0.04;  // 4% loss

        double P03 = P02 * (1.0 - pressure_loss_fraction);
        TS_ASSERT_DELTA(P03, 2431800.0, 1000.0);
    }

    //==========================================================================
    // 5. NOZZLE FLOW TESTS (~6 tests)
    //==========================================================================

    void testNozzle_ChokedFlow() {
        // Choked when P_exit/P₀ ≤ (2/(γ+1))^(γ/(γ-1))
        double gamma = ThermoConstants::GAMMA;
        double P_critical_ratio = pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));

        TS_ASSERT_DELTA(P_critical_ratio, 0.5283, 0.001);

        // If back pressure is below this, nozzle is choked
        double P0 = 200000.0;   // Pa (nozzle stagnation pressure)
        double P_back = 50000.0;  // Pa (ambient)
        double P_exit_min = P0 * P_critical_ratio;

        bool is_choked = (P_back < P_exit_min);
        TS_ASSERT(is_choked);
    }

    void testNozzle_ExitVelocity() {
        // V_exit = sqrt(2 * c_p * T₀ * (1 - (P_exit/P₀)^((γ-1)/γ)))
        double gamma = ThermoConstants::GAMMA;
        double cp = ThermoConstants::CP_AIR;
        double T0 = 1038.90;     // K (nozzle inlet stagnation temp)
        double P0 = 200000.0;    // Pa
        double P_exit = 101325.0;  // Pa (expanded to ambient)

        double PR = P_exit / P0;
        double V_exit = sqrt(2.0 * cp * T0 * (1.0 - pow(PR, (gamma - 1.0) / gamma)));
        TS_ASSERT_DELTA(V_exit, 607.2, 1.0);  // m/s
    }

    void testNozzle_MassFlowRate() {
        // mdot = (ρ * A * V)_exit = (P_exit * A * V_exit) / (R * T_exit)
        double P_exit = 101325.0;  // Pa
        double A_exit = 0.5;       // m² (nozzle exit area)
        double V_exit = 607.2;     // m/s
        double T0 = 1038.90;       // K
        double P0 = 200000.0;      // Pa
        double gamma = ThermoConstants::GAMMA;
        double R = ThermoConstants::R_AIR;

        // Exit temperature
        double PR = P_exit / P0;
        double T_exit = T0 * pow(PR, (gamma - 1.0) / gamma);

        double rho_exit = P_exit / (R * T_exit);
        double mdot = rho_exit * A_exit * V_exit;
        TS_ASSERT_DELTA(mdot, 125.3, 1.0);  // kg/s
    }

    void testNozzle_ThrustCoefficient() {
        // C_F = V_exit / sqrt(γ * R * T₀)
        double V_exit = 607.2;  // m/s
        double gamma = ThermoConstants::GAMMA;
        double R = ThermoConstants::R_AIR;
        double T0 = 1038.90;     // K

        double C_F = V_exit / sqrt(gamma * R * T0);
        TS_ASSERT_DELTA(C_F, 0.941, 0.01);
    }

    void testNozzle_UnderExpansion() {
        // Under-expanded: P_exit > P_ambient (not fully expanded)
        double P_exit = 150000.0;  // Pa
        double P_ambient = 101325.0;  // Pa

        bool under_expanded = (P_exit > P_ambient);
        TS_ASSERT(under_expanded);

        // Additional thrust from pressure difference
        double A_exit = 0.5;  // m²
        double pressure_thrust = (P_exit - P_ambient) * A_exit;
        TS_ASSERT_DELTA(pressure_thrust, 24337.5, 1.0);  // N
    }

    void testNozzle_OverExpansion() {
        // Over-expanded: P_exit < P_ambient (expanded too much)
        double P_exit = 80000.0;   // Pa
        double P_ambient = 101325.0;  // Pa

        bool over_expanded = (P_exit < P_ambient);
        TS_ASSERT(over_expanded);

        // Thrust penalty from pressure difference
        double A_exit = 0.5;  // m²
        double pressure_loss = (P_ambient - P_exit) * A_exit;
        TS_ASSERT_DELTA(pressure_loss, 10662.5, 1.0);  // N loss
    }

    //==========================================================================
    // 6. HEAT TRANSFER TESTS (~4 tests)
    //==========================================================================

    void testHeatTransfer_ConvectiveHeatTransfer() {
        // Q = h * A * ΔT
        // where h is convective heat transfer coefficient
        double h = 50.0;       // W/(m²·K) (typical for air)
        double A = 2.0;        // m² (surface area)
        double T_surface = 400.0;  // K
        double T_fluid = 300.0;    // K

        double Q = h * A * (T_surface - T_fluid);
        TS_ASSERT_DELTA(Q, 10000.0, 0.1);  // W (heat transfer rate)
    }

    void testHeatTransfer_AdiabaticWallTemperature() {
        // T_aw = T_static * (1 + r * ((γ-1)/2) * M²)
        // where r is recovery factor (~0.9 for turbulent flow)
        double T_static = 250.0;  // K
        double M = 2.0;           // Mach number
        double gamma = ThermoConstants::GAMMA;
        double r = 0.9;           // Recovery factor (turbulent)

        double T_aw = T_static * (1.0 + r * ((gamma - 1.0) / 2.0) * M * M);
        TS_ASSERT_DELTA(T_aw, 430.0, 1.0);  // K
    }

    void testHeatTransfer_RecoveryFactor_Laminar() {
        // For laminar flow: r = Pr^(1/2)
        // where Pr is Prandtl number (~0.72 for air)
        double Pr = 0.72;
        double r = sqrt(Pr);

        TS_ASSERT_DELTA(r, 0.849, 0.001);
    }

    void testHeatTransfer_RecoveryFactor_Turbulent() {
        // For turbulent flow: r = Pr^(1/3)
        double Pr = 0.72;
        double r = pow(Pr, 1.0 / 3.0);

        TS_ASSERT_DELTA(r, 0.896, 0.001);
    }

    //==========================================================================
    // ADDITIONAL INTEGRATION TESTS
    //==========================================================================

    void testTurbofanEngine_SimpleCycle() {
        // Complete simple turbofan cycle
        double gamma = ThermoConstants::GAMMA;
        double cp = ThermoConstants::CP_AIR;

        // Freestream
        double M0 = 0.85;
        double T0 = 223.15;  // K (cruise altitude)
        double P0 = 23800.0;  // Pa

        // Inlet (assume ideal, stagnation conditions)
        double T02 = T0 * (1.0 + ((gamma - 1.0) / 2.0) * M0 * M0);
        double P02 = P0 * pow(T02 / T0, gamma / (gamma - 1.0));

        // Compressor
        double PR_c = 30.0;
        double eta_c = 0.85;
        double P03 = P02 * PR_c;
        double T03_ideal = T02 * pow(PR_c, (gamma - 1.0) / gamma);
        double T03 = T02 + (T03_ideal - T02) / eta_c;

        // Combustion
        double T04 = 1500.0;  // K (turbine inlet temp limit)
        double eta_b = 0.995;
        double P04 = P03 * 0.96;  // 4% pressure loss

        // Turbine (to drive compressor)
        double W_c = cp * (T03 - T02);
        double eta_t = 0.88;
        double eta_m = 0.99;
        double W_t = W_c / eta_m;
        double T05_ideal = T04 - W_t / cp;
        double T05 = T04 - W_t / (cp * eta_t);

        // Verify temperatures make physical sense
        TS_ASSERT(T02 > T0);      // Compression heats air
        TS_ASSERT(T03 > T02);     // Compressor increases temperature
        TS_ASSERT(T04 > T03);     // Combustion adds heat
        TS_ASSERT(T05 < T04);     // Turbine extracts work
        TS_ASSERT(T05 > T0);      // Still hot after turbine

        // Verify pressures
        TS_ASSERT(P02 > P0);      // Inlet ram pressure
        TS_ASSERT(P03 > P02);     // Compressor raises pressure
        TS_ASSERT(P04 < P03);     // Combustor pressure loss
    }

    void testRealGasEffects_HighTemperature() {
        // At high temperatures, γ decreases (ideal gas law still holds)
        // This test verifies calculations work with different γ values
        double gamma_cold = 1.4;
        double gamma_hot = 1.33;  // Typical at 1500K

        double PR = 4.0;
        double T_ratio_cold = pow(PR, (gamma_cold - 1.0) / gamma_cold);
        double T_ratio_hot = pow(PR, (gamma_hot - 1.0) / gamma_hot);

        // Lower gamma means less temperature rise for compression (or less drop for expansion)
        TS_ASSERT(T_ratio_cold > T_ratio_hot);
        TS_ASSERT_DELTA(T_ratio_cold, 1.486, 0.001);
        TS_ASSERT_DELTA(T_ratio_hot, 1.411, 0.001);
    }

    void testEnginePerformance_ThrustSpecificFuelConsumption() {
        // TSFC = mdot_f / F (kg/(N·s) or lb/(lbf·hr))
        double mdot_f = 0.9;   // kg/s
        double thrust = 50000.0;  // N

        double TSFC = mdot_f / thrust;  // kg/(N·s)
        double TSFC_hr = TSFC * 3600.0;  // kg/(N·hr)

        TS_ASSERT_DELTA(TSFC, 1.8e-5, 1e-6);
        TS_ASSERT_DELTA(TSFC_hr, 0.0648, 0.001);  // Typical for turbofan
    }

    void testRamjetCycle_NoCompressor() {
        // Ramjet has no compressor - uses ram pressure from speed
        double gamma = ThermoConstants::GAMMA;
        double cp = ThermoConstants::CP_AIR;

        double M0 = 3.0;  // Supersonic flight
        double T0 = 223.15;  // K
        double P0 = 23800.0;  // Pa

        // Diffuser (ram compression)
        double T02 = T0 * (1.0 + ((gamma - 1.0) / 2.0) * M0 * M0);
        double P02 = P0 * pow(T02 / T0, gamma / (gamma - 1.0));

        // Combustion
        double T03 = 2000.0;  // K

        // Nozzle expansion back to ambient
        double P_exit = P0;
        double T_exit = T03 * pow(P_exit / (P02 * 0.95), (gamma - 1.0) / gamma);
        double V_exit = sqrt(2.0 * cp * (T03 - T_exit));

        TS_ASSERT(T02 > T0);   // Ram compression heats air
        TS_ASSERT(P02 > P0);   // Ram compression increases pressure
        TS_ASSERT(V_exit > M0 * sqrt(gamma * ThermoConstants::R_AIR * T0));  // Accelerated
    }
};

/*******************************************************************************
 * Additional FGThermodynamics Tests (33 new tests)
 ******************************************************************************/

class FGThermodynamicsAdditionalTest : public CxxTest::TestSuite
{
public:
    //==========================================================================
    // ENTROPY AND ENTHALPY TESTS
    //==========================================================================

    // Test 43: Entropy change for isentropic process
    void testEntropyChange_Isentropic() {
        // For isentropic process, Δs = 0
        double gamma = ThermoConstants::GAMMA;
        double T1 = 300.0;
        double P1 = 100000.0;
        double P2 = 200000.0;

        // Isentropic temperature
        double T2 = T1 * pow(P2 / P1, (gamma - 1.0) / gamma);

        // Entropy change: Δs = cp * ln(T2/T1) - R * ln(P2/P1)
        double cp = ThermoConstants::CP_AIR;
        double R = ThermoConstants::R_AIR;
        double delta_s = cp * log(T2 / T1) - R * log(P2 / P1);

        TS_ASSERT_DELTA(delta_s, 0.0, 0.1);
    }

    // Test 44: Entropy change for irreversible compression
    void testEntropyChange_Irreversible() {
        double gamma = ThermoConstants::GAMMA;
        double cp = ThermoConstants::CP_AIR;
        double R = ThermoConstants::R_AIR;

        double T1 = 288.15;
        double P1 = 101325.0;
        double P2 = 1013250.0;  // 10:1 pressure ratio
        double eta_c = 0.85;

        // Ideal (isentropic) temperature
        double T2s = T1 * pow(P2 / P1, (gamma - 1.0) / gamma);
        // Actual temperature
        double T2 = T1 + (T2s - T1) / eta_c;

        // Entropy change
        double delta_s = cp * log(T2 / T1) - R * log(P2 / P1);
        TS_ASSERT(delta_s > 0);  // Irreversible process increases entropy
    }

    // Test 45: Specific enthalpy calculation
    void testSpecificEnthalpy() {
        double cp = ThermoConstants::CP_AIR;
        double T = 500.0;  // K
        double T_ref = 0.0;  // Reference temperature

        double h = cp * (T - T_ref);
        TS_ASSERT_DELTA(h, 502500.0, 100.0);  // J/kg
    }

    // Test 46: Stagnation enthalpy
    void testStagnationEnthalpy() {
        double cp = ThermoConstants::CP_AIR;
        double T = 300.0;  // K (static temp)
        double V = 200.0;  // m/s

        double h = cp * T;
        double h0 = h + V * V / 2.0;

        TS_ASSERT_DELTA(h0, 321500.0, 100.0);  // J/kg
    }

    //==========================================================================
    // GAS MIXTURE TESTS
    //==========================================================================

    // Test 47: Molecular weight of air
    void testMolecularWeightAir() {
        // Air is ~78% N2, ~21% O2, ~1% Ar
        double M_N2 = 28.0;
        double M_O2 = 32.0;
        double M_Ar = 40.0;

        double M_air = 0.78 * M_N2 + 0.21 * M_O2 + 0.01 * M_Ar;
        TS_ASSERT_DELTA(M_air, 28.96, 0.1);
    }

    // Test 48: Specific gas constant from molecular weight
    void testSpecificGasConstant() {
        double R_universal = 8314.0;  // J/(kmol·K)
        double M_air = 28.96;         // kg/kmol

        double R = R_universal / M_air;
        TS_ASSERT_DELTA(R, 287.05, 0.5);
    }

    // Test 49: Cp/Cv ratio verification
    void testCpCvRatio() {
        double cp = ThermoConstants::CP_AIR;
        double cv = ThermoConstants::CV_AIR;
        double gamma_calc = cp / cv;

        TS_ASSERT_DELTA(gamma_calc, ThermoConstants::GAMMA, 0.01);
    }

    // Test 50: Cp - Cv = R
    void testCpMinusCvEqualsR() {
        double cp = ThermoConstants::CP_AIR;
        double cv = ThermoConstants::CV_AIR;
        double R = ThermoConstants::R_AIR;

        double diff = cp - cv;
        TS_ASSERT_DELTA(diff, R, 1.0);
    }

    //==========================================================================
    // AFTERBURNER TESTS
    //==========================================================================

    // Test 51: Afterburner temperature rise
    void testAfterburnerTemperatureRise() {
        double T5 = 900.0;   // K (turbine exit)
        double T6 = 2000.0;  // K (afterburner exit)
        double cp = ThermoConstants::CP_AIR;

        double delta_T = T6 - T5;
        double Q_ab = cp * delta_T;

        TS_ASSERT_DELTA(delta_T, 1100.0, 0.1);
        TS_ASSERT_DELTA(Q_ab, 1105500.0, 100.0);  // J/kg
    }

    // Test 52: Afterburner fuel flow
    void testAfterburnerFuelFlow() {
        double Q_ab = 1105500.0;  // J/kg
        double LHV = ThermoConstants::FUEL_HEATING_VALUE;
        double eta_ab = 0.95;

        double f_ab = Q_ab / (eta_ab * LHV);
        TS_ASSERT_DELTA(f_ab, 0.0271, 0.001);
    }

    // Test 53: Afterburner thrust augmentation
    void testAfterburnerThrustAugmentation() {
        double V_exit_dry = 500.0;  // m/s
        double T_exit_dry = 800.0;  // K
        double T_exit_ab = 1800.0;  // K (with afterburner)

        // V_exit ∝ sqrt(T)
        double V_exit_ab = V_exit_dry * sqrt(T_exit_ab / T_exit_dry);
        double thrust_ratio = V_exit_ab / V_exit_dry;

        TS_ASSERT_DELTA(V_exit_ab, 750.0, 5.0);
        TS_ASSERT_DELTA(thrust_ratio, 1.5, 0.05);
    }

    //==========================================================================
    // BLADE COOLING TESTS
    //==========================================================================

    // Test 54: Turbine blade cooling effectiveness
    void testBladeCoolingEffectiveness() {
        // η_c = (T_gas - T_blade) / (T_gas - T_coolant)
        double T_gas = 1700.0;    // K
        double T_coolant = 700.0; // K (compressor bleed)
        double T_blade = 1100.0;  // K (metal limit)

        double eta_c = (T_gas - T_blade) / (T_gas - T_coolant);
        TS_ASSERT_DELTA(eta_c, 0.6, 0.01);
    }

    // Test 55: Coolant mass flow ratio for blade cooling
    void testCoolantMassFlowRatio() {
        // Typical coolant flow is 5-15% of core flow
        double mdot_gas = 50.0;           // kg/s
        double coolant_fraction = 0.10;   // 10% coolant flow

        double mdot_coolant = mdot_gas * coolant_fraction;
        TS_ASSERT_DELTA(mdot_coolant, 5.0, 0.1);
        TS_ASSERT(mdot_coolant > 0);
        TS_ASSERT(mdot_coolant < mdot_gas);  // Coolant is fraction of main flow
    }

    // Test 56: Film cooling temperature
    void testFilmCoolingTemperature() {
        double T_gas = 1600.0;    // K
        double T_coolant = 650.0; // K
        double film_effectiveness = 0.5;

        double T_film = T_gas - film_effectiveness * (T_gas - T_coolant);
        TS_ASSERT_DELTA(T_film, 1125.0, 1.0);
    }

    //==========================================================================
    // CYCLE EFFICIENCY TESTS
    //==========================================================================

    // Test 57: Carnot efficiency
    void testCarnotEfficiency() {
        double T_hot = 1500.0;   // K
        double T_cold = 288.15;  // K

        double eta_carnot = 1.0 - T_cold / T_hot;
        TS_ASSERT_DELTA(eta_carnot, 0.808, 0.001);
    }

    // Test 58: Brayton cycle thermal efficiency (ideal)
    void testBraytonCycleEfficiency_Ideal() {
        double gamma = ThermoConstants::GAMMA;
        double PR = 25.0;  // Pressure ratio

        double eta_brayton = 1.0 - pow(1.0 / PR, (gamma - 1.0) / gamma);
        TS_ASSERT_DELTA(eta_brayton, 0.601, 0.001);
    }

    // Test 59: Actual cycle efficiency
    void testActualCycleEfficiency() {
        double W_net = 300000.0;  // J/kg (net work output)
        double Q_in = 700000.0;   // J/kg (heat input)

        double eta = W_net / Q_in;
        TS_ASSERT_DELTA(eta, 0.429, 0.001);
    }

    // Test 60: Specific fuel consumption from efficiency
    void testSFCFromEfficiency() {
        double eta = 0.40;        // Thermal efficiency
        double LHV = ThermoConstants::FUEL_HEATING_VALUE;

        // SFC = 1 / (eta * LHV)
        double SFC = 1.0 / (eta * LHV);  // kg/(J)
        double SFC_hr = SFC * 3600.0;     // kg/(kJ·hr)

        TS_ASSERT_DELTA(SFC * 1e6, 0.0581, 0.001);  // μg/J
    }

    //==========================================================================
    // SHOCK WAVE THERMODYNAMICS
    //==========================================================================

    // Test 61: Normal shock temperature ratio
    void testNormalShockTemperatureRatio() {
        double gamma = ThermoConstants::GAMMA;
        double M1 = 2.0;  // Upstream Mach number

        // Correct normal shock relation:
        // T2/T1 = [1 + 2γ/(γ+1)*(M1²-1)] * [(2 + (γ-1)*M1²) / ((γ+1)*M1²)]
        double term1 = 1.0 + 2.0 * gamma / (gamma + 1.0) * (M1 * M1 - 1.0);
        double term2 = (2.0 + (gamma - 1.0) * M1 * M1) / ((gamma + 1.0) * M1 * M1);
        double T_ratio = term1 * term2;

        // For M1 = 2.0, T2/T1 ≈ 1.687
        TS_ASSERT_DELTA(T_ratio, 1.687, 0.01);
        TS_ASSERT(T_ratio > 1.0);  // Temperature always increases across shock
    }

    // Test 62: Normal shock pressure ratio
    void testNormalShockPressureRatio() {
        double gamma = ThermoConstants::GAMMA;
        double M1 = 2.0;

        double P_ratio = 1.0 + 2.0 * gamma / (gamma + 1.0) * (M1 * M1 - 1.0);
        TS_ASSERT_DELTA(P_ratio, 4.5, 0.01);
    }

    // Test 63: Downstream Mach number after normal shock
    void testNormalShockM2() {
        double gamma = ThermoConstants::GAMMA;
        double M1 = 2.0;

        double M2_sq = (1.0 + ((gamma - 1.0) / 2.0) * M1 * M1) /
                       (gamma * M1 * M1 - (gamma - 1.0) / 2.0);
        double M2 = sqrt(M2_sq);

        TS_ASSERT_DELTA(M2, 0.577, 0.01);  // Always subsonic after normal shock
    }

    //==========================================================================
    // INLET PERFORMANCE TESTS
    //==========================================================================

    // Test 64: Ram pressure recovery
    void testRamPressureRecovery() {
        double gamma = ThermoConstants::GAMMA;
        double M0 = 0.85;
        double eta_d = 0.98;  // Diffuser efficiency

        double P0_P = pow(1.0 + ((gamma - 1.0) / 2.0) * M0 * M0, gamma / (gamma - 1.0));
        double P02_P = eta_d * P0_P;

        TS_ASSERT_DELTA(P0_P, 1.604, 0.01);
        TS_ASSERT_DELTA(P02_P, 1.572, 0.01);
    }

    // Test 65: Supersonic inlet total pressure loss
    void testSupersonicInletLoss() {
        // MIL-E-5007 pressure recovery for supersonic inlet
        double M0 = 2.0;

        // Simplified formula
        double P02_P0 = 1.0 - 0.075 * pow(M0 - 1.0, 1.35);
        TS_ASSERT_DELTA(P02_P0, 0.925, 0.01);
    }

    // Test 66: Inlet spillage drag
    void testInletSpillageDrag() {
        double mdot_design = 50.0;   // kg/s (design mass flow)
        double mdot_actual = 45.0;   // kg/s (actual)
        double V0 = 250.0;           // m/s

        double mdot_spill = mdot_design - mdot_actual;
        double D_spill = mdot_spill * V0;

        TS_ASSERT_DELTA(D_spill, 1250.0, 1.0);  // N
    }

    //==========================================================================
    // PROPULSIVE EFFICIENCY TESTS
    //==========================================================================

    // Test 67: Propulsive efficiency
    void testPropulsiveEfficiency() {
        double V0 = 250.0;    // m/s (flight speed)
        double V_exit = 500.0; // m/s (exhaust velocity)

        double eta_p = 2.0 / (1.0 + V_exit / V0);
        TS_ASSERT_DELTA(eta_p, 0.667, 0.01);
    }

    // Test 68: Overall efficiency
    void testOverallEfficiency() {
        double eta_thermal = 0.40;
        double eta_propulsive = 0.67;

        double eta_overall = eta_thermal * eta_propulsive;
        TS_ASSERT_DELTA(eta_overall, 0.268, 0.01);
    }

    // Test 69: Bypass ratio effect on propulsive efficiency
    void testBypassRatioEffect() {
        double V0 = 250.0;

        // Low bypass (BPR = 1)
        double V_exit_low = 450.0;
        double eta_p_low = 2.0 / (1.0 + V_exit_low / V0);

        // High bypass (BPR = 8)
        double V_exit_high = 320.0;
        double eta_p_high = 2.0 / (1.0 + V_exit_high / V0);

        TS_ASSERT(eta_p_high > eta_p_low);  // Higher BPR = better propulsive efficiency
    }

    //==========================================================================
    // POWER TURBINE TESTS
    //==========================================================================

    // Test 70: Free turbine power output
    void testFreeTurbinePower() {
        double mdot = 50.0;      // kg/s
        double cp = ThermoConstants::CP_AIR;
        double T05 = 1000.0;     // K (power turbine inlet)
        double T06 = 700.0;      // K (power turbine exit)
        double eta_t = 0.88;

        double W_dot = mdot * cp * (T05 - T06) * eta_t;
        TS_ASSERT_DELTA(W_dot, 13.2e6, 0.1e6);  // W (13.2 MW)
    }

    // Test 71: Power turbine pressure ratio
    void testPowerTurbinePressureRatio() {
        double gamma = ThermoConstants::GAMMA;
        double T05 = 1000.0;
        double T06 = 700.0;
        double eta_t = 0.88;

        // Ideal temperature drop
        double T06_ideal = T05 - (T05 - T06) / eta_t;
        double PR_pt = pow(T05 / T06_ideal, gamma / (gamma - 1.0));

        TS_ASSERT(PR_pt > 1);
    }

    //==========================================================================
    // REAL GAS EFFECTS
    //==========================================================================

    // Test 72: Variable gamma with temperature
    void testVariableGamma() {
        // Gamma decreases with temperature
        double gamma_300K = 1.40;
        double gamma_1000K = 1.35;
        double gamma_1500K = 1.32;

        TS_ASSERT(gamma_300K > gamma_1000K);
        TS_ASSERT(gamma_1000K > gamma_1500K);
    }

    // Test 73: Cp variation with temperature
    void testCpVariation() {
        // Cp increases with temperature (polynomial fit)
        // Cp(T) = a + b*T + c*T² (approximate)
        double a = 1000.0;
        double b = 0.02;
        double c = 5e-8;

        double Cp_300 = a + b * 300 + c * 300 * 300;
        double Cp_1000 = a + b * 1000 + c * 1000 * 1000;

        TS_ASSERT(Cp_1000 > Cp_300);
    }

    // Test 74: Compressibility factor
    void testCompressibilityFactor() {
        // At moderate pressures and temperatures, Z ≈ 1 for air
        // Z = PV/(nRT)
        double P = 101325.0;
        double T = 288.15;
        double rho = 1.225;
        double R = ThermoConstants::R_AIR;

        double Z = P / (rho * R * T);
        TS_ASSERT_DELTA(Z, 1.0, 0.01);  // Should be near 1 for ideal gas
    }

    // Test 75: High altitude temperature effect
    void testHighAltitudeTemperature() {
        double T_SL = 288.15;          // K (sea level)
        double lapse_rate = 0.0065;    // K/m
        double altitude = 11000.0;     // m (tropopause)

        double T_alt = T_SL - lapse_rate * altitude;
        TS_ASSERT_DELTA(T_alt, 216.65, 0.5);  // K (standard tropopause temp)

        // Above tropopause, temperature is constant
        double T_20km = T_alt;  // Still 216.65 K
        TS_ASSERT_DELTA(T_20km, 216.65, 0.5);
    }

    //==========================================================================
    // EXTENDED THERMODYNAMICS TESTS (76-100)
    //==========================================================================

    // Test 76: Scramjet combustion temperature
    void testScramjetCombustionTemp() {
        double T_inlet = 1500.0;    // K (high due to compression)
        double f = 0.04;            // Fuel-air ratio
        double LHV = ThermoConstants::FUEL_HEATING_VALUE;
        double cp = ThermoConstants::CP_AIR;
        double eta_b = 0.90;        // Lower efficiency for supersonic combustion

        double delta_T = (f * eta_b * LHV) / ((1.0 + f) * cp);
        double T_exit = T_inlet + delta_T;

        TS_ASSERT(T_exit > T_inlet);
        TS_ASSERT_DELTA(T_exit, 2994.0, 50.0);  // K
    }

    // Test 77: Scramjet thrust from momentum change
    void testScramjetMomentumThrust() {
        double mdot_air = 20.0;     // kg/s
        double V_inlet = 2000.0;    // m/s
        double V_exit = 2500.0;     // m/s
        double f = 0.04;            // Fuel-air ratio

        double mdot_exit = mdot_air * (1.0 + f);
        double thrust = mdot_exit * V_exit - mdot_air * V_inlet;

        TS_ASSERT(thrust > 0);
        TS_ASSERT_DELTA(thrust, 12000.0, 500.0);  // N
    }

    // Test 78: Regenerative cooling heat transfer
    void testRegenerativeCooling() {
        double Q_wall = 5.0e6;      // W/m² (heat flux)
        double A = 0.5;             // m² (cooled area)
        double mdot_fuel = 2.0;     // kg/s
        double cp_fuel = 2000.0;    // J/(kg·K) for hydrocarbon fuel

        double Q_total = Q_wall * A;
        double delta_T_fuel = Q_total / (mdot_fuel * cp_fuel);

        TS_ASSERT_DELTA(delta_T_fuel, 625.0, 10.0);  // K fuel temperature rise
    }

    // Test 79: Turbojet specific impulse
    void testTurbojetSpecificImpulse() {
        double thrust = 50000.0;    // N
        double mdot_f = 1.0;        // kg/s
        double g0 = 9.81;           // m/s²

        double Isp = thrust / (mdot_f * g0);
        TS_ASSERT_DELTA(Isp, 5097.0, 10.0);  // seconds
    }

    // Test 80: Rocket engine specific impulse comparison
    void testRocketSpecificImpulse() {
        // Hydrogen-oxygen rocket
        double Ve = 4000.0;         // m/s (exhaust velocity)
        double g0 = 9.81;

        double Isp = Ve / g0;
        TS_ASSERT_DELTA(Isp, 408.0, 1.0);  // seconds

        // Compare to typical turbojet
        double Isp_turbojet = 5000.0;
        TS_ASSERT(Isp_turbojet > Isp);  // Turbojet has higher Isp
    }

    // Test 81: Detonation wave temperature ratio
    void testDetonationTemperatureRatio() {
        // Chapman-Jouguet detonation
        double gamma = ThermoConstants::GAMMA;
        double M_CJ = 5.0;  // Typical CJ Mach number

        // Simplified temperature ratio
        double T_ratio = (2.0 * gamma * M_CJ * M_CJ - (gamma - 1.0)) *
                         ((gamma - 1.0) * M_CJ * M_CJ + 2.0) /
                         ((gamma + 1.0) * (gamma + 1.0) * M_CJ * M_CJ);

        TS_ASSERT(T_ratio > 1.0);
        TS_ASSERT(T_ratio > 5.0);  // Significant temperature rise
    }

    // Test 82: Pulse detonation engine cycle frequency
    void testPDECycleFrequency() {
        double tube_length = 1.0;   // m
        double fill_velocity = 100.0;  // m/s
        double det_velocity = 2000.0;  // m/s

        double t_fill = tube_length / fill_velocity;
        double t_det = tube_length / det_velocity;
        double t_blowdown = 2.0 * tube_length / 400.0;  // Rough estimate

        double t_cycle = t_fill + t_det + t_blowdown;
        double frequency = 1.0 / t_cycle;

        TS_ASSERT(frequency > 50);  // > 50 Hz typical
        TS_ASSERT(frequency < 200);
    }

    // Test 83: Bleed air extraction effect
    void testBleedAirExtraction() {
        double mdot_core = 50.0;    // kg/s
        double bleed_fraction = 0.05;  // 5% bleed
        double W_specific = 500000.0;  // J/kg (compressor work)

        double mdot_bleed = mdot_core * bleed_fraction;
        double W_lost = mdot_bleed * W_specific;

        TS_ASSERT_DELTA(mdot_bleed, 2.5, 0.1);
        TS_ASSERT_DELTA(W_lost, 1.25e6, 1000.0);  // W
    }

    // Test 84: Variable geometry inlet performance
    void testVariableGeometryInlet() {
        double gamma = ThermoConstants::GAMMA;

        // Design Mach number
        double M_design = 2.5;
        double A_ratio_design = (1.0 / M_design) *
            pow((2.0 / (gamma + 1.0)) * (1.0 + ((gamma - 1.0) / 2.0) * M_design * M_design),
                (gamma + 1.0) / (2.0 * (gamma - 1.0)));

        // Off-design at M = 2.0
        double M_actual = 2.0;
        double A_ratio_actual = (1.0 / M_actual) *
            pow((2.0 / (gamma + 1.0)) * (1.0 + ((gamma - 1.0) / 2.0) * M_actual * M_actual),
                (gamma + 1.0) / (2.0 * (gamma - 1.0)));

        TS_ASSERT(A_ratio_design > A_ratio_actual);
    }

    // Test 85: Thrust reverser efficiency
    void testThrustReverserEfficiency() {
        double thrust_forward = 100000.0;  // N
        double reverser_efficiency = 0.40;  // Typical cascade reverser

        double thrust_reverse = thrust_forward * reverser_efficiency;
        TS_ASSERT_DELTA(thrust_reverse, 40000.0, 100.0);
    }

    // Test 86: Engine face distortion index
    void testDistortionIndex() {
        double P_avg = 100000.0;    // Pa (average total pressure)
        double P_min = 92000.0;     // Pa (minimum in sector)
        double q = 50000.0;         // Pa (dynamic pressure)

        double DC60 = (P_avg - P_min) / q;
        TS_ASSERT_DELTA(DC60, 0.16, 0.01);

        // Typical limit is DC60 < 0.25
        TS_ASSERT(DC60 < 0.25);
    }

    // Test 87: Combustor liner temperature
    void testCombustorLinerTemp() {
        double T_gas = 2000.0;      // K (flame temperature)
        double T_coolant = 700.0;   // K (cooling air)
        double cooling_effectiveness = 0.65;

        double T_liner = T_gas - cooling_effectiveness * (T_gas - T_coolant);
        TS_ASSERT_DELTA(T_liner, 1155.0, 10.0);  // K
        TS_ASSERT(T_liner < 1200.0);  // Below material limit
    }

    // Test 88: Lean blowout limit
    void testLeanBlowoutLimit() {
        double f_stoich = 0.068;    // Stoichiometric fuel-air ratio
        double phi_lbo = 0.5;       // Lean blowout equivalence ratio

        double f_lbo = phi_lbo * f_stoich;
        TS_ASSERT_DELTA(f_lbo, 0.034, 0.001);
    }

    // Test 89: Rich blowout limit
    void testRichBlowoutLimit() {
        double f_stoich = 0.068;
        double phi_rbo = 1.6;       // Rich blowout equivalence ratio

        double f_rbo = phi_rbo * f_stoich;
        TS_ASSERT_DELTA(f_rbo, 0.109, 0.001);
    }

    // Test 90: Compressor surge margin
    void testCompressorSurgeMargin() {
        double PR_surge = 28.0;     // Pressure ratio at surge
        double PR_operating = 25.0; // Operating pressure ratio

        double surge_margin = (PR_surge - PR_operating) / PR_operating * 100.0;
        TS_ASSERT_DELTA(surge_margin, 12.0, 0.5);  // percent

        // Minimum margin is typically 10-15%
        TS_ASSERT(surge_margin > 10.0);
    }

    // Test 91: Turbine inlet temperature trend monitoring
    void testTITTrendMonitoring() {
        double TIT_baseline = 1450.0;   // K
        double TIT_current = 1470.0;    // K
        double tolerance = 25.0;         // K

        double delta_TIT = TIT_current - TIT_baseline;
        bool within_limits = std::abs(delta_TIT) < tolerance;

        TS_ASSERT(within_limits);
        TS_ASSERT_DELTA(delta_TIT, 20.0, 0.1);
    }

    // Test 92: Engine pressure ratio
    void testEnginePressureRatio() {
        double P02 = 100000.0;      // Pa (compressor inlet total pressure)
        double P05 = 180000.0;      // Pa (turbine exit total pressure)

        double EPR = P05 / P02;
        TS_ASSERT_DELTA(EPR, 1.8, 0.01);
    }

    // Test 93: N1 and N2 speed relationship
    void testSpoolSpeedRelationship() {
        double N1_percent = 85.0;   // Low pressure spool
        double N2_percent = 95.0;   // High pressure spool

        // At high power, N2 leads N1
        TS_ASSERT(N2_percent > N1_percent);

        double ratio = N2_percent / N1_percent;
        TS_ASSERT_DELTA(ratio, 1.118, 0.01);
    }

    // Test 94: Exhaust gas temperature limit
    void testEGTLimit() {
        double EGT_measured = 920.0;    // K
        double EGT_limit = 950.0;       // K (red line)
        double EGT_caution = 900.0;     // K (amber line)

        bool below_limit = EGT_measured < EGT_limit;
        bool in_caution = EGT_measured > EGT_caution;

        TS_ASSERT(below_limit);
        TS_ASSERT(in_caution);
    }

    // Test 95: Fuel flow to thrust ratio
    void testFuelFlowThrustRatio() {
        double thrust = 100000.0;   // N
        double fuel_flow = 2.5;     // kg/s

        double ratio = fuel_flow / thrust * 1e6;  // mg/N/s
        double TSFC = fuel_flow / thrust * 3600.0;  // kg/(N·hr)

        TS_ASSERT_DELTA(TSFC, 0.09, 0.01);
    }

    // Test 96: Idle fuel flow
    void testIdleFuelFlow() {
        double idle_fuel_flow = 0.15;   // kg/s
        double max_fuel_flow = 3.0;     // kg/s

        double idle_percentage = idle_fuel_flow / max_fuel_flow * 100.0;
        TS_ASSERT_DELTA(idle_percentage, 5.0, 0.5);
    }

    // Test 97: Acceleration fuel schedule
    void testAccelerationFuelSchedule() {
        double N2_current = 70.0;       // percent
        double N2_target = 95.0;        // percent
        double accel_rate = 10.0;       // percent per second

        double accel_time = (N2_target - N2_current) / accel_rate;
        TS_ASSERT_DELTA(accel_time, 2.5, 0.1);  // seconds
    }

    // Test 98: Deceleration fuel schedule
    void testDecelerationFuelSchedule() {
        double N2_current = 95.0;
        double N2_target = 60.0;
        double decel_rate = 8.0;        // percent per second (slower than accel)

        double decel_time = (N2_current - N2_target) / decel_rate;
        TS_ASSERT_DELTA(decel_time, 4.375, 0.1);  // seconds
    }

    // Test 99: Core exhaust enthalpy
    void testCoreExhaustEnthalpy() {
        double T_exhaust = 800.0;   // K
        double cp = ThermoConstants::CP_AIR;
        double V_exhaust = 400.0;   // m/s

        double h_static = cp * T_exhaust;
        double h_total = h_static + 0.5 * V_exhaust * V_exhaust;

        TS_ASSERT_DELTA(h_static, 804000.0, 100.0);
        TS_ASSERT_DELTA(h_total, 884000.0, 100.0);
    }

    // Test 100: Complete thermodynamic cycle summary
    void testCompleteCycleSummary() {
        double gamma = ThermoConstants::GAMMA;
        double cp = ThermoConstants::CP_AIR;
        double R = ThermoConstants::R_AIR;

        // Station 2 (compressor inlet)
        double T02 = 288.15;
        double P02 = 101325.0;

        // Station 3 (compressor exit)
        double PR_c = 30.0;
        double eta_c = 0.88;
        double T03s = T02 * pow(PR_c, (gamma - 1.0) / gamma);
        double T03 = T02 + (T03s - T02) / eta_c;
        double P03 = P02 * PR_c;

        // Station 4 (turbine inlet)
        double T04 = 1600.0;
        double P04 = P03 * 0.96;  // 4% combustor loss

        // Station 5 (turbine exit - power to drive compressor)
        double W_c = cp * (T03 - T02);
        double eta_t = 0.90;
        double T05 = T04 - W_c / (cp * eta_t);
        double P05_P04 = pow(T05 / T04, gamma / (gamma - 1.0));
        double P05 = P04 * P05_P04 / eta_t;

        // Verify all temperatures are physically reasonable
        TS_ASSERT(T03 > T02);  // Compression heats
        TS_ASSERT(T04 > T03);  // Combustion heats
        TS_ASSERT(T05 < T04);  // Expansion cools
        TS_ASSERT(T05 > T02);  // Still hot after turbine

        // Verify all pressures are physically reasonable
        TS_ASSERT(P03 > P02);  // Compression raises pressure
        TS_ASSERT(P04 < P03);  // Combustor pressure loss
        TS_ASSERT(P05 < P04);  // Turbine drops pressure

        // Verify efficiency bounds
        TS_ASSERT(T03s <= T03);  // Ideal is less than actual for compression
        TS_ASSERT(W_c > 0);       // Compressor requires work

        // Calculate thermal efficiency
        double Q_in = cp * (T04 - T03);
        double W_net = cp * ((T04 - T05) - (T03 - T02));
        double eta_thermal = W_net / Q_in;

        TS_ASSERT(eta_thermal > 0);
        TS_ASSERT(eta_thermal < 1);
    }
