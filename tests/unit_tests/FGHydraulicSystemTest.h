/*******************************************************************************
 * FGHydraulicSystemTest.h - Unit tests for hydraulic system calculations
 *
 * Tests comprehensive hydraulic system physics including pressure calculations,
 * pump flow rates, actuator force generation, fluid compressibility, pressure
 * regulation, accumulator dynamics, priority valve logic, temperature effects,
 * pump efficiency, leakage modeling, system redundancy, emergency extension,
 * and actuator rate limiting.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>
#include "TestUtilities.h"

using namespace JSBSimTest;

// Constants for hydraulic system calculations
namespace HydraulicConstants {
    // Hydraulic fluid properties (MIL-PRF-83282 - typical aircraft hydraulic fluid)
    constexpr double FLUID_DENSITY_LBM_IN3 = 0.0313;       // ~31.3 lb/gal at 70°F
    constexpr double FLUID_DENSITY_KG_M3 = 865.0;          // kg/m³
    constexpr double BULK_MODULUS_PSI = 250000.0;          // Effective bulk modulus (psi)
    constexpr double BULK_MODULUS_PA = 1.724e9;            // Pascal
    constexpr double KINEMATIC_VISCOSITY_CST = 13.5;       // Centistokes at 100°F
    constexpr double SPECIFIC_HEAT_BTU_LBM_F = 0.43;       // BTU/(lbm·°F)

    // System pressure parameters
    constexpr double SYSTEM_PRESSURE_PSI = 3000.0;         // Typical aircraft hydraulic system pressure
    constexpr double MIN_OPERATING_PRESSURE_PSI = 2800.0;  // Minimum operating pressure
    constexpr double MAX_PRESSURE_PSI = 3300.0;            // Pressure relief valve setting
    constexpr double ACCUMULATOR_PRECHARGE_PSI = 1000.0;   // Accumulator gas precharge
    constexpr double RETURN_PRESSURE_PSI = 50.0;           // Return line pressure

    // Pump parameters
    constexpr double PUMP_DISPLACEMENT_IN3_REV = 1.5;      // Pump displacement (cubic inches per revolution)
    constexpr double PUMP_VOLUMETRIC_EFFICIENCY = 0.95;    // Volumetric efficiency
    constexpr double PUMP_MECHANICAL_EFFICIENCY = 0.90;    // Mechanical efficiency
    constexpr double PUMP_OVERALL_EFFICIENCY = 0.855;      // Overall efficiency (0.95 * 0.90)
    constexpr double NOMINAL_PUMP_RPM = 4800.0;            // Nominal engine-driven pump RPM

    // Actuator parameters
    constexpr double ACTUATOR_PISTON_AREA_IN2 = 3.14;      // Piston area (1" diameter)
    constexpr double ACTUATOR_ROD_AREA_IN2 = 0.785;        // Rod area (1" diameter piston, 0.5" rod)
    constexpr double ACTUATOR_EXTEND_AREA_IN2 = 3.14;      // Full piston area
    constexpr double ACTUATOR_RETRACT_AREA_IN2 = 2.355;    // Piston area - rod area

    // Leakage and losses
    constexpr double INTERNAL_LEAKAGE_GPM_1000PSI = 0.05;  // Internal leakage per 1000 psi
    constexpr double EXTERNAL_LEAKAGE_GPM = 0.01;          // External leakage
    constexpr double VALVE_PRESSURE_DROP_PSI = 50.0;       // Pressure drop across control valve

    // Accumulator parameters (gas-charged bladder type)
    constexpr double ACCUMULATOR_VOLUME_IN3 = 100.0;       // Total accumulator volume
    constexpr double GAS_CONSTANT_NITROGEN = 1.4;          // Polytropic exponent for nitrogen

    // Temperature effects
    constexpr double NOMINAL_TEMP_F = 100.0;               // Nominal operating temperature
    constexpr double MIN_TEMP_F = -40.0;                   // Minimum operating temperature
    constexpr double MAX_TEMP_F = 250.0;                   // Maximum operating temperature
    constexpr double TEMP_COEFFICIENT_VISCOSITY = 0.93;    // Viscosity change per 10°F

    // Conversion factors
    constexpr double GPM_TO_IN3_S = 3.85;                  // Gallons per minute to cubic inches per second
    constexpr double IN3_S_TO_GPM = 0.2597;                // Cubic inches per second to gallons per minute
    constexpr double HP_TO_IN_LBF_S = 6600.0;              // Horsepower to in·lbf/s
}

class FGHydraulicSystemTest : public CxxTest::TestSuite
{
public:
    //==========================================================================
    // 1. HYDRAULIC PRESSURE CALCULATIONS (~5 tests)
    //==========================================================================

    void testPressure_SystemOperatingPressure() {
        // Test nominal system operating pressure
        double system_pressure_psi = HydraulicConstants::SYSTEM_PRESSURE_PSI;

        TS_ASSERT_DELTA(system_pressure_psi, 3000.0, 0.1);
        TS_ASSERT(system_pressure_psi >= HydraulicConstants::MIN_OPERATING_PRESSURE_PSI);
        TS_ASSERT(system_pressure_psi <= HydraulicConstants::MAX_PRESSURE_PSI);
    }

    void testPressure_PressureRegulation() {
        // Test pressure regulator maintaining pressure within limits
        double current_pressure_psi = 3100.0;  // Above nominal
        double max_pressure_psi = HydraulicConstants::MAX_PRESSURE_PSI;
        double relief_valve_setting_psi = 3300.0;

        bool relief_valve_open = (current_pressure_psi >= relief_valve_setting_psi);
        bool pressure_within_limits = (current_pressure_psi <= max_pressure_psi);

        TS_ASSERT(!relief_valve_open);  // Below relief valve setting
        TS_ASSERT(pressure_within_limits);
    }

    void testPressure_ReliefValveCracking() {
        // Test relief valve cracking pressure
        double pressure_psi = 3350.0;  // Above relief valve setting
        double relief_setting_psi = HydraulicConstants::MAX_PRESSURE_PSI;

        bool valve_opens = (pressure_psi >= relief_setting_psi);
        double excess_pressure_psi = pressure_psi - relief_setting_psi;

        TS_ASSERT(valve_opens);
        TS_ASSERT_DELTA(excess_pressure_psi, 50.0, 0.1);
    }

    void testPressure_PascalConversion() {
        // Convert pressure between psi and Pascal
        double pressure_psi = 3000.0;
        double psi_to_pa = 6894.76;

        double pressure_pa = pressure_psi * psi_to_pa;

        TS_ASSERT_DELTA(pressure_pa, 20684280.0, 10.0);
    }

    void testPressure_DifferentialPressure() {
        // Calculate differential pressure across actuator
        double supply_pressure_psi = 3000.0;
        double return_pressure_psi = HydraulicConstants::RETURN_PRESSURE_PSI;

        double differential_psi = supply_pressure_psi - return_pressure_psi;

        TS_ASSERT_DELTA(differential_psi, 2950.0, 0.1);
    }

    //==========================================================================
    // 2. PUMP FLOW RATE VS RPM (~5 tests)
    //==========================================================================

    void testPump_FlowRateVsRPM() {
        // Calculate pump flow rate from RPM
        double pump_rpm = HydraulicConstants::NOMINAL_PUMP_RPM;
        double displacement_in3_rev = HydraulicConstants::PUMP_DISPLACEMENT_IN3_REV;
        double vol_efficiency = HydraulicConstants::PUMP_VOLUMETRIC_EFFICIENCY;

        // Flow rate (in³/min) = displacement × RPM × efficiency
        double flow_in3_min = displacement_in3_rev * pump_rpm * vol_efficiency;
        double flow_gpm = flow_in3_min / 231.0;  // 231 in³ = 1 gallon

        TS_ASSERT_DELTA(flow_in3_min, 6840.0, 10.0);
        TS_ASSERT_DELTA(flow_gpm, 29.6, 0.2);
    }

    void testPump_FlowAtHalfSpeed() {
        // Flow rate at 50% RPM
        double pump_rpm = HydraulicConstants::NOMINAL_PUMP_RPM * 0.5;
        double displacement_in3_rev = HydraulicConstants::PUMP_DISPLACEMENT_IN3_REV;
        double vol_efficiency = HydraulicConstants::PUMP_VOLUMETRIC_EFFICIENCY;

        double flow_in3_min = displacement_in3_rev * pump_rpm * vol_efficiency;
        double flow_gpm = flow_in3_min / 231.0;

        TS_ASSERT_DELTA(flow_gpm, 14.8, 0.2);  // Half of full speed flow
    }

    void testPump_VolumetricEfficiency() {
        // Test volumetric efficiency effect on flow
        double theoretical_flow_in3_min = 1.5 * 4800.0;  // 7200 in³/min
        double actual_flow_in3_min = theoretical_flow_in3_min * 0.95;

        double efficiency = actual_flow_in3_min / theoretical_flow_in3_min;

        TS_ASSERT_DELTA(efficiency, 0.95, 0.001);
        TS_ASSERT_DELTA(actual_flow_in3_min, 6840.0, 1.0);
    }

    void testPump_PowerRequirement() {
        // Calculate pump power requirement
        double flow_gpm = 30.0;
        double pressure_psi = 3000.0;
        double overall_efficiency = HydraulicConstants::PUMP_OVERALL_EFFICIENCY;

        // Hydraulic power (HP) = (flow_gpm × pressure_psi) / (1714 × efficiency)
        double hydraulic_hp = (flow_gpm * pressure_psi) / 1714.0;
        double input_hp = hydraulic_hp / overall_efficiency;

        TS_ASSERT_DELTA(hydraulic_hp, 52.5, 0.5);
        TS_ASSERT_DELTA(input_hp, 61.4, 0.5);
    }

    void testPump_FlowAtZeroRPM() {
        // No flow when pump is stopped
        double pump_rpm = 0.0;
        double displacement_in3_rev = HydraulicConstants::PUMP_DISPLACEMENT_IN3_REV;

        double flow_in3_min = displacement_in3_rev * pump_rpm;

        TS_ASSERT_DELTA(flow_in3_min, 0.0, 0.001);
    }

    //==========================================================================
    // 3. ACTUATOR FORCE FROM PRESSURE (~5 tests)
    //==========================================================================

    void testActuator_ExtendingForce() {
        // Calculate force during extension (pressure on full piston area)
        double pressure_psi = 3000.0;
        double piston_area_in2 = HydraulicConstants::ACTUATOR_EXTEND_AREA_IN2;

        double force_lbf = pressure_psi * piston_area_in2;

        TS_ASSERT_DELTA(force_lbf, 9420.0, 10.0);
    }

    void testActuator_RetractingForce() {
        // Calculate force during retraction (pressure on annular area)
        double pressure_psi = 3000.0;
        double annular_area_in2 = HydraulicConstants::ACTUATOR_RETRACT_AREA_IN2;

        double force_lbf = pressure_psi * annular_area_in2;

        TS_ASSERT_DELTA(force_lbf, 7065.0, 10.0);
    }

    void testActuator_ForceAsymmetry() {
        // Retraction force is less than extension force due to rod area
        double pressure_psi = 3000.0;
        double extend_force_lbf = pressure_psi * HydraulicConstants::ACTUATOR_EXTEND_AREA_IN2;
        double retract_force_lbf = pressure_psi * HydraulicConstants::ACTUATOR_RETRACT_AREA_IN2;

        double force_ratio = retract_force_lbf / extend_force_lbf;

        TS_ASSERT_DELTA(force_ratio, 0.75, 0.01);
        TS_ASSERT(retract_force_lbf < extend_force_lbf);
    }

    void testActuator_MinimumOperatingForce() {
        // Force at minimum operating pressure
        double min_pressure_psi = HydraulicConstants::MIN_OPERATING_PRESSURE_PSI;
        double piston_area_in2 = HydraulicConstants::ACTUATOR_EXTEND_AREA_IN2;

        double force_lbf = min_pressure_psi * piston_area_in2;

        TS_ASSERT_DELTA(force_lbf, 8791.2, 10.0);
    }

    void testActuator_LoadResistance() {
        // Calculate required pressure to overcome load
        double load_force_lbf = 5000.0;
        double piston_area_in2 = HydraulicConstants::ACTUATOR_EXTEND_AREA_IN2;

        double required_pressure_psi = load_force_lbf / piston_area_in2;

        TS_ASSERT_DELTA(required_pressure_psi, 1592.4, 1.0);
        TS_ASSERT(required_pressure_psi < HydraulicConstants::SYSTEM_PRESSURE_PSI);
    }

    //==========================================================================
    // 4. FLUID COMPRESSIBILITY EFFECTS (~5 tests)
    //==========================================================================

    void testCompressibility_BulkModulus() {
        // Test fluid bulk modulus value
        double bulk_modulus_psi = HydraulicConstants::BULK_MODULUS_PSI;

        TS_ASSERT_DELTA(bulk_modulus_psi, 250000.0, 1000.0);
        TS_ASSERT(bulk_modulus_psi > 200000.0);  // Typical range
    }

    void testCompressibility_VolumeChange() {
        // Calculate volume change due to pressure change
        // ΔV/V = ΔP / K (where K is bulk modulus)
        double initial_volume_in3 = 100.0;
        double pressure_change_psi = 1000.0;
        double bulk_modulus_psi = HydraulicConstants::BULK_MODULUS_PSI;

        double volume_strain = pressure_change_psi / bulk_modulus_psi;
        double volume_change_in3 = initial_volume_in3 * volume_strain;

        TS_ASSERT_DELTA(volume_strain, 0.004, 0.0001);  // 0.4% compression
        TS_ASSERT_DELTA(volume_change_in3, 0.4, 0.01);
    }

    void testCompressibility_PressureRiseTime() {
        // Time to build pressure in a fixed volume
        double volume_in3 = 50.0;
        double flow_rate_in3_s = 10.0;
        double bulk_modulus_psi = HydraulicConstants::BULK_MODULUS_PSI;
        double target_pressure_psi = 3000.0;

        // Simplified: time ≈ (V × ΔP) / (K × Q)
        // More accurate: t = (V / Q) × (ΔP / K)
        double time_to_pressure_s = (volume_in3 * target_pressure_psi) / (bulk_modulus_psi * flow_rate_in3_s);

        TS_ASSERT_DELTA(time_to_pressure_s, 0.06, 0.01);  // Very fast due to high bulk modulus
    }

    void testCompressibility_SpringRate() {
        // Hydraulic spring rate (stiffness) from fluid compressibility
        double piston_area_in2 = 3.14;
        double fluid_column_length_in = 10.0;
        double bulk_modulus_psi = HydraulicConstants::BULK_MODULUS_PSI;

        // Spring rate k = (A² × K) / L (lbf/in)
        double spring_rate_lbf_in = (piston_area_in2 * piston_area_in2 * bulk_modulus_psi) /
                                     fluid_column_length_in;

        TS_ASSERT_DELTA(spring_rate_lbf_in, 246490.0, 100.0);
        TS_ASSERT(spring_rate_lbf_in > 0.0);
    }

    void testCompressibility_EnergyStorage() {
        // Energy stored in compressed fluid
        double volume_in3 = 100.0;
        double pressure_psi = 3000.0;
        double bulk_modulus_psi = HydraulicConstants::BULK_MODULUS_PSI;

        // Energy = (P² × V) / (2 × K) (in·lbf)
        double stored_energy_in_lbf = (pressure_psi * pressure_psi * volume_in3) /
                                       (2.0 * bulk_modulus_psi);

        TS_ASSERT_DELTA(stored_energy_in_lbf, 1800.0, 10.0);
    }

    //==========================================================================
    // 5. SYSTEM PRESSURE REGULATION (~5 tests)
    //==========================================================================

    void testRegulation_PressureSwitch() {
        // Pressure switch hysteresis
        double switch_on_psi = 2800.0;   // Pump starts
        double switch_off_psi = 3000.0;  // Pump stops

        double current_pressure = 2750.0;
        bool pump_running = (current_pressure <= switch_on_psi);

        TS_ASSERT(pump_running);

        current_pressure = 3050.0;
        pump_running = (current_pressure < switch_off_psi);
        TS_ASSERT(!pump_running);
    }

    void testRegulation_UnloadValve() {
        // Unload valve diverts flow to tank at high pressure
        double system_pressure_psi = 3050.0;
        double unload_setting_psi = 3000.0;

        bool valve_open = (system_pressure_psi >= unload_setting_psi);

        TS_ASSERT(valve_open);
    }

    void testRegulation_PressureRipple() {
        // Pressure ripple amplitude
        double nominal_pressure_psi = 3000.0;
        double ripple_amplitude_psi = 50.0;

        double max_pressure = nominal_pressure_psi + ripple_amplitude_psi;
        double min_pressure = nominal_pressure_psi - ripple_amplitude_psi;

        TS_ASSERT_DELTA(max_pressure, 3050.0, 0.1);
        TS_ASSERT_DELTA(min_pressure, 2950.0, 0.1);
        TS_ASSERT(min_pressure >= HydraulicConstants::MIN_OPERATING_PRESSURE_PSI);
    }

    void testRegulation_ProportionalControl() {
        // Proportional pressure control valve
        double target_pressure_psi = 3000.0;
        double current_pressure_psi = 2900.0;
        double k_p = 0.1;  // Proportional gain

        double error_psi = target_pressure_psi - current_pressure_psi;
        double control_signal = k_p * error_psi;

        TS_ASSERT_DELTA(error_psi, 100.0, 0.1);
        TS_ASSERT_DELTA(control_signal, 10.0, 0.1);
    }

    void testRegulation_PressureTransducerAccuracy() {
        // Pressure transducer measurement accuracy
        double actual_pressure_psi = 3000.0;
        double accuracy_percent = 0.5;  // ±0.5% full scale
        double full_scale_psi = 5000.0;

        double max_error_psi = (accuracy_percent / 100.0) * full_scale_psi;
        double measured_pressure_psi = actual_pressure_psi + max_error_psi;

        TS_ASSERT_DELTA(max_error_psi, 25.0, 0.1);
        TS_ASSERT(std::abs(measured_pressure_psi - actual_pressure_psi) <= max_error_psi);
    }

    //==========================================================================
    // 6. ACCUMULATOR PRECHARGE (~5 tests)
    //==========================================================================

    void testAccumulator_PrechargeRatio() {
        // Typical precharge is 33% of system pressure
        double system_pressure_psi = HydraulicConstants::SYSTEM_PRESSURE_PSI;
        double precharge_psi = HydraulicConstants::ACCUMULATOR_PRECHARGE_PSI;

        double precharge_ratio = precharge_psi / system_pressure_psi;

        TS_ASSERT_DELTA(precharge_ratio, 0.333, 0.01);
    }

    void testAccumulator_GasLawExpansion() {
        // Gas expansion using polytropic process: P1 × V1^n = P2 × V2^n
        double precharge_psi = 1000.0;
        double precharge_volume_in3 = 100.0;
        double expanded_volume_in3 = 120.0;
        double n = HydraulicConstants::GAS_CONSTANT_NITROGEN;

        // P2 = P1 × (V1/V2)^n
        double expanded_pressure_psi = precharge_psi *
                                        pow(precharge_volume_in3 / expanded_volume_in3, n);

        TS_ASSERT_DELTA(expanded_pressure_psi, 774.7, 5.0);
    }

    void testAccumulator_UsableVolume() {
        // Usable fluid volume between min and max pressure
        double total_volume_in3 = HydraulicConstants::ACCUMULATOR_VOLUME_IN3;
        double precharge_psi = 1000.0;
        double min_pressure_psi = 2800.0;
        double max_pressure_psi = 3000.0;
        double n = 1.4;

        // V_min = V0 × (P0/P_min)^(1/n)
        // V_max = V0 × (P0/P_max)^(1/n)
        double V_min = total_volume_in3 * pow(precharge_psi / min_pressure_psi, 1.0 / n);
        double V_max = total_volume_in3 * pow(precharge_psi / max_pressure_psi, 1.0 / n);
        double usable_volume_in3 = V_max - V_min;

        TS_ASSERT_DELTA(V_min, 47.9, 1.0);
        TS_ASSERT_DELTA(V_max, 45.6, 1.0);
        TS_ASSERT(usable_volume_in3 < 0);  // Note: V decreases as P increases
    }

    void testAccumulator_FluidDischarge() {
        // Fluid discharged from accumulator
        double total_volume_in3 = 100.0;
        double gas_volume_at_3000psi_in3 = 43.7;
        double gas_volume_at_2800psi_in3 = 45.8;

        // Fluid volume = Total volume - Gas volume
        double fluid_at_3000psi = total_volume_in3 - gas_volume_at_3000psi_in3;
        double fluid_at_2800psi = total_volume_in3 - gas_volume_at_2800psi_in3;
        double fluid_discharged_in3 = fluid_at_3000psi - fluid_at_2800psi;

        TS_ASSERT_DELTA(fluid_discharged_in3, 2.1, 0.2);
    }

    void testAccumulator_EmergencyCapacity() {
        // Number of actuator cycles supported by accumulator
        double usable_fluid_in3 = 2.0;
        double actuator_volume_per_cycle_in3 = 10.0;  // Volume for one full stroke

        double emergency_cycles = usable_fluid_in3 / actuator_volume_per_cycle_in3;

        TS_ASSERT_DELTA(emergency_cycles, 0.2, 0.02);
        // Typically designed for 1-3 emergency cycles, so may need larger accumulator
    }

    //==========================================================================
    // 7. PRIORITY VALVE LOGIC (~5 tests)
    //==========================================================================

    void testPriority_FlightControlPriority() {
        // Priority valve ensures flight controls get flow first
        double total_flow_gpm = 20.0;
        double flight_control_demand_gpm = 15.0;
        double utility_demand_gpm = 10.0;

        // Flight controls get full demand
        double flight_control_actual_gpm = std::min(flight_control_demand_gpm, total_flow_gpm);
        // Utility systems get remainder
        double utility_actual_gpm = std::max(0.0, total_flow_gpm - flight_control_actual_gpm);

        TS_ASSERT_DELTA(flight_control_actual_gpm, 15.0, 0.1);
        TS_ASSERT_DELTA(utility_actual_gpm, 5.0, 0.1);
    }

    void testPriority_InsufficientFlow() {
        // When flow is insufficient, utility systems are starved
        double total_flow_gpm = 10.0;
        double priority_demand_gpm = 12.0;
        double utility_demand_gpm = 5.0;

        double priority_actual_gpm = std::min(priority_demand_gpm, total_flow_gpm);
        double utility_actual_gpm = std::max(0.0, total_flow_gpm - priority_actual_gpm);

        TS_ASSERT_DELTA(priority_actual_gpm, 10.0, 0.1);
        TS_ASSERT_DELTA(utility_actual_gpm, 0.0, 0.1);
    }

    void testPriority_ThreeSystemLevels() {
        // Three-level priority: flight controls > landing gear > utilities
        double total_flow_gpm = 25.0;
        double flight_control_demand_gpm = 10.0;
        double landing_gear_demand_gpm = 12.0;
        double utility_demand_gpm = 8.0;

        double fc_flow = std::min(flight_control_demand_gpm, total_flow_gpm);
        double remaining_1 = total_flow_gpm - fc_flow;
        double lg_flow = std::min(landing_gear_demand_gpm, remaining_1);
        double remaining_2 = remaining_1 - lg_flow;
        double util_flow = std::min(utility_demand_gpm, remaining_2);

        TS_ASSERT_DELTA(fc_flow, 10.0, 0.1);
        TS_ASSERT_DELTA(lg_flow, 12.0, 0.1);
        TS_ASSERT_DELTA(util_flow, 3.0, 0.1);
    }

    void testPriority_PressureCompensation() {
        // Priority valve maintains pressure to priority circuit
        double inlet_pressure_psi = 3000.0;
        double priority_circuit_load_drop_psi = 200.0;
        double compensator_setting_psi = 2900.0;

        double priority_pressure_psi = inlet_pressure_psi - priority_circuit_load_drop_psi;
        bool compensator_active = (priority_pressure_psi < compensator_setting_psi);

        TS_ASSERT_DELTA(priority_pressure_psi, 2800.0, 0.1);
        TS_ASSERT(compensator_active);
    }

    void testPriority_ValveResponseTime() {
        // Priority valve switching time
        double response_time_ms = 50.0;  // Typical response time
        double max_response_ms = 100.0;

        TS_ASSERT(response_time_ms < max_response_ms);
        TS_ASSERT(response_time_ms > 0.0);
    }

    //==========================================================================
    // 8. HYDRAULIC FLUID TEMPERATURE (~5 tests)
    //==========================================================================

    void testTemperature_NominalOperating() {
        // Normal operating temperature range
        double nominal_temp_f = HydraulicConstants::NOMINAL_TEMP_F;
        double min_temp_f = HydraulicConstants::MIN_TEMP_F;
        double max_temp_f = HydraulicConstants::MAX_TEMP_F;

        TS_ASSERT(nominal_temp_f > min_temp_f);
        TS_ASSERT(nominal_temp_f < max_temp_f);
        TS_ASSERT_DELTA(nominal_temp_f, 100.0, 5.0);
    }

    void testTemperature_ViscosityChange() {
        // Viscosity increases at low temperature
        double viscosity_100f_cst = 13.5;
        double temp_coefficient = HydraulicConstants::TEMP_COEFFICIENT_VISCOSITY;

        // At 90°F (10°F lower)
        double viscosity_90f_cst = viscosity_100f_cst / temp_coefficient;

        TS_ASSERT_DELTA(viscosity_90f_cst, 14.5, 0.2);
        TS_ASSERT(viscosity_90f_cst > viscosity_100f_cst);
    }

    void testTemperature_HeatGeneration() {
        // Heat generated by pump inefficiency
        double input_power_hp = 60.0;
        double hydraulic_power_hp = 52.0;
        double heat_hp = input_power_hp - hydraulic_power_hp;

        // Convert to BTU/min (1 HP = 42.4 BTU/min)
        double heat_btu_min = heat_hp * 42.4;

        TS_ASSERT_DELTA(heat_hp, 8.0, 0.1);
        TS_ASSERT_DELTA(heat_btu_min, 339.2, 1.0);
    }

    void testTemperature_CoolingRequirement() {
        // Heat exchanger cooling capacity
        double heat_load_btu_min = 340.0;
        double flow_rate_gpm = 30.0;
        double fluid_density_lbm_gal = 7.2;
        double specific_heat_btu_lbm_f = HydraulicConstants::SPECIFIC_HEAT_BTU_LBM_F;

        // Temperature rise: ΔT = Q / (m_dot × cp)
        double mass_flow_lbm_min = flow_rate_gpm * fluid_density_lbm_gal;
        double temp_rise_f = heat_load_btu_min / (mass_flow_lbm_min * specific_heat_btu_lbm_f);

        TS_ASSERT_DELTA(temp_rise_f, 3.7, 0.2);
    }

    void testTemperature_ThermalExpansion() {
        // Fluid volume expansion with temperature
        double initial_volume_gal = 10.0;
        double temp_change_f = 50.0;
        double expansion_coefficient = 0.0005;  // Per °F

        double volume_change_gal = initial_volume_gal * expansion_coefficient * temp_change_f;

        TS_ASSERT_DELTA(volume_change_gal, 0.25, 0.01);
    }

    //==========================================================================
    // 9. PUMP EFFICIENCY (~5 tests)
    //==========================================================================

    void testEfficiency_VolumetricEfficiency() {
        // Volumetric efficiency from internal leakage
        double theoretical_flow_gpm = 30.0;
        double actual_flow_gpm = 28.5;

        double vol_efficiency = actual_flow_gpm / theoretical_flow_gpm;

        TS_ASSERT_DELTA(vol_efficiency, 0.95, 0.01);
    }

    void testEfficiency_MechanicalEfficiency() {
        // Mechanical efficiency from friction losses
        double hydraulic_torque_in_lbf = 1000.0;
        double input_torque_in_lbf = 1111.0;

        double mech_efficiency = hydraulic_torque_in_lbf / input_torque_in_lbf;

        TS_ASSERT_DELTA(mech_efficiency, 0.90, 0.01);
    }

    void testEfficiency_OverallEfficiency() {
        // Overall efficiency is product of volumetric and mechanical
        double vol_eff = 0.95;
        double mech_eff = 0.90;

        double overall_eff = vol_eff * mech_eff;

        TS_ASSERT_DELTA(overall_eff, 0.855, 0.001);
    }

    void testEfficiency_EfficiencyVsPressure() {
        // Efficiency decreases slightly at higher pressure
        double efficiency_1000psi = 0.90;
        double efficiency_3000psi = 0.855;

        TS_ASSERT(efficiency_3000psi < efficiency_1000psi);
        TS_ASSERT(efficiency_3000psi > 0.80);  // Still reasonable
    }

    void testEfficiency_PowerLoss() {
        // Power loss due to inefficiency
        double input_power_hp = 60.0;
        double overall_efficiency = 0.855;

        double output_power_hp = input_power_hp * overall_efficiency;
        double power_loss_hp = input_power_hp - output_power_hp;

        TS_ASSERT_DELTA(output_power_hp, 51.3, 0.2);
        TS_ASSERT_DELTA(power_loss_hp, 8.7, 0.2);
    }

    //==========================================================================
    // 10. LEAKAGE EFFECTS (~5 tests)
    //==========================================================================

    void testLeakage_InternalLeakage() {
        // Internal leakage across pump
        double pressure_psi = 3000.0;
        double leakage_per_1000psi_gpm = HydraulicConstants::INTERNAL_LEAKAGE_GPM_1000PSI;

        double internal_leakage_gpm = (pressure_psi / 1000.0) * leakage_per_1000psi_gpm;

        TS_ASSERT_DELTA(internal_leakage_gpm, 0.15, 0.01);
    }

    void testLeakage_ExternalLeakage() {
        // External leakage to environment (should be minimal)
        double external_leakage_gpm = HydraulicConstants::EXTERNAL_LEAKAGE_GPM;
        double max_acceptable_gpm = 0.05;

        TS_ASSERT(external_leakage_gpm <= max_acceptable_gpm);
        TS_ASSERT_DELTA(external_leakage_gpm, 0.01, 0.001);
    }

    void testLeakage_TotalLeakage() {
        // Total system leakage
        double internal_leakage_gpm = 0.15;
        double external_leakage_gpm = 0.01;
        double valve_leakage_gpm = 0.05;

        double total_leakage_gpm = internal_leakage_gpm + external_leakage_gpm + valve_leakage_gpm;

        TS_ASSERT_DELTA(total_leakage_gpm, 0.21, 0.01);
    }

    void testLeakage_LeakagePowerLoss() {
        // Power wasted by leakage
        double leakage_gpm = 0.2;
        double pressure_psi = 3000.0;

        double power_loss_hp = (leakage_gpm * pressure_psi) / 1714.0;

        TS_ASSERT_DELTA(power_loss_hp, 0.35, 0.01);
    }

    void testLeakage_FluidReplenishment() {
        // Time to lose 1 quart through leakage
        double leakage_gpm = 0.05;
        double volume_loss_gal = 0.25;  // 1 quart

        double time_to_loss_min = volume_loss_gal / leakage_gpm;

        TS_ASSERT_DELTA(time_to_loss_min, 5.0, 0.1);
    }

    //==========================================================================
    // 11. MULTIPLE SYSTEM REDUNDANCY (~5 tests)
    //==========================================================================

    void testRedundancy_DualSystemConfiguration() {
        // Aircraft with two independent hydraulic systems
        double system_a_pressure_psi = 3000.0;
        double system_b_pressure_psi = 3000.0;

        bool system_a_operative = (system_a_pressure_psi >= 2800.0);
        bool system_b_operative = (system_b_pressure_psi >= 2800.0);
        bool aircraft_controllable = (system_a_operative || system_b_operative);

        TS_ASSERT(system_a_operative);
        TS_ASSERT(system_b_operative);
        TS_ASSERT(aircraft_controllable);
    }

    void testRedundancy_SystemAFailure() {
        // System A fails, System B maintains control
        double system_a_pressure_psi = 500.0;  // Failed
        double system_b_pressure_psi = 3000.0;

        bool system_a_operative = (system_a_pressure_psi >= 2800.0);
        bool system_b_operative = (system_b_pressure_psi >= 2800.0);

        TS_ASSERT(!system_a_operative);
        TS_ASSERT(system_b_operative);
    }

    void testRedundancy_LoadSharing() {
        // Load sharing between systems
        double total_load_gpm = 40.0;
        double system_a_capacity_gpm = 30.0;
        double system_b_capacity_gpm = 30.0;

        // Equal sharing if both operative
        double system_a_flow_gpm = total_load_gpm / 2.0;
        double system_b_flow_gpm = total_load_gpm / 2.0;

        TS_ASSERT_DELTA(system_a_flow_gpm, 20.0, 0.1);
        TS_ASSERT_DELTA(system_b_flow_gpm, 20.0, 0.1);
        TS_ASSERT(system_a_flow_gpm < system_a_capacity_gpm);
    }

    void testRedundancy_CrossfeedValve() {
        // Crossfeed allows one system to power both sides
        bool system_a_operative = true;
        bool system_b_operative = false;
        bool crossfeed_open = !system_b_operative;

        bool right_side_powered = system_b_operative || (crossfeed_open && system_a_operative);

        TS_ASSERT(crossfeed_open);
        TS_ASSERT(right_side_powered);
    }

    void testRedundancy_IsolationValve() {
        // Isolation valve prevents contamination spread
        bool system_a_contaminated = true;
        bool isolation_valve_closed = system_a_contaminated;

        TS_ASSERT(isolation_valve_closed);
    }

    //==========================================================================
    // 12. EMERGENCY EXTENSION (GRAVITY/SPRING) (~5 tests)
    //==========================================================================

    void testEmergency_GravityExtension() {
        // Landing gear gravity extension
        bool hydraulic_failure = true;
        bool emergency_handle_pulled = true;
        bool uplocks_released = (hydraulic_failure && emergency_handle_pulled);

        double gear_weight_lbf = 500.0;
        double aerodynamic_drag_lbf = 100.0;
        double net_extending_force_lbf = gear_weight_lbf - aerodynamic_drag_lbf;

        TS_ASSERT(uplocks_released);
        TS_ASSERT_DELTA(net_extending_force_lbf, 400.0, 0.1);
        TS_ASSERT(net_extending_force_lbf > 0.0);  // Gravity will extend
    }

    void testEmergency_SpringAssist() {
        // Spring-assisted emergency extension
        double spring_force_lbf = 200.0;
        double gravity_force_lbf = 400.0;

        double total_extending_force_lbf = spring_force_lbf + gravity_force_lbf;

        TS_ASSERT_DELTA(total_extending_force_lbf, 600.0, 0.1);
    }

    void testEmergency_ExtensionTime() {
        // Time for gravity extension
        double stroke_length_in = 24.0;
        double avg_velocity_in_s = 2.0;  // Slower than hydraulic

        double extension_time_s = stroke_length_in / avg_velocity_in_s;

        TS_ASSERT_DELTA(extension_time_s, 12.0, 0.1);
        TS_ASSERT(extension_time_s > 5.0);  // Much slower than hydraulic
    }

    void testEmergency_NitrogenAccumulator() {
        // Nitrogen accumulator for emergency braking
        double accumulator_volume_in3 = 50.0;
        double precharge_psi = 1000.0;
        double brake_applications = 3.0;
        double volume_per_application_in3 = 5.0;

        double total_volume_required_in3 = brake_applications * volume_per_application_in3;
        bool sufficient_capacity = (total_volume_required_in3 < accumulator_volume_in3 * 0.5);

        TS_ASSERT(sufficient_capacity);
        TS_ASSERT_DELTA(total_volume_required_in3, 15.0, 0.1);
    }

    void testEmergency_ManualPumpBackup() {
        // Manual hydraulic pump for emergency
        double strokes_per_minute = 30.0;
        double volume_per_stroke_in3 = 0.5;

        double flow_in3_min = strokes_per_minute * volume_per_stroke_in3;
        double flow_gpm = flow_in3_min / 231.0;

        TS_ASSERT_DELTA(flow_gpm, 0.065, 0.005);  // Very low but usable
    }

    //==========================================================================
    // 13. ACTUATOR RATE LIMITING (~5 tests)
    //==========================================================================

    void testRateLimit_FlowLimitedSpeed() {
        // Actuator speed limited by flow
        double flow_rate_in3_s = 10.0;
        double piston_area_in2 = 3.14;

        double velocity_in_s = flow_rate_in3_s / piston_area_in2;

        TS_ASSERT_DELTA(velocity_in_s, 3.18, 0.1);
    }

    void testRateLimit_MaximumExtensionRate() {
        // Maximum extension rate with full pump flow
        double pump_flow_gpm = 30.0;
        double flow_in3_s = pump_flow_gpm * HydraulicConstants::GPM_TO_IN3_S;
        double piston_area_in2 = 3.14;

        double max_velocity_in_s = flow_in3_s / piston_area_in2;

        TS_ASSERT_DELTA(max_velocity_in_s, 36.8, 1.0);
    }

    void testRateLimit_AsymmetricRates() {
        // Extension vs retraction rate asymmetry
        double flow_rate_in3_s = 10.0;
        double extend_area_in2 = HydraulicConstants::ACTUATOR_EXTEND_AREA_IN2;
        double retract_area_in2 = HydraulicConstants::ACTUATOR_RETRACT_AREA_IN2;

        double extend_velocity_in_s = flow_rate_in3_s / extend_area_in2;
        double retract_velocity_in_s = flow_rate_in3_s / retract_area_in2;

        TS_ASSERT(retract_velocity_in_s > extend_velocity_in_s);
        TS_ASSERT_DELTA(extend_velocity_in_s, 3.18, 0.1);
        TS_ASSERT_DELTA(retract_velocity_in_s, 4.25, 0.1);
    }

    void testRateLimit_ValveOrificeRestriction() {
        // Flow control valve limits actuator speed
        double max_flow_gpm = 5.0;
        double piston_area_in2 = 3.14;
        double flow_in3_s = max_flow_gpm * HydraulicConstants::GPM_TO_IN3_S;

        double controlled_velocity_in_s = flow_in3_s / piston_area_in2;

        TS_ASSERT_DELTA(controlled_velocity_in_s, 6.1, 0.2);
    }

    void testRateLimit_TimeToFullStroke() {
        // Time for actuator to complete full stroke
        double stroke_length_in = 12.0;
        double velocity_in_s = 6.0;

        double stroke_time_s = stroke_length_in / velocity_in_s;

        TS_ASSERT_DELTA(stroke_time_s, 2.0, 0.1);
    }
};
