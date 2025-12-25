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
#include "TestUtilities.h"

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
