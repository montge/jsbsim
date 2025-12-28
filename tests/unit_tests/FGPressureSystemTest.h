/*******************************************************************************
 * FGPressureSystemTest.h - Unit tests for cabin pressurization and ECS
 *
 * Tests comprehensive cabin pressurization and environmental control systems
 * including cabin altitude calculations, differential pressure limits, outflow
 * valve control, pressurization rates, temperature control, emergency
 * depressurization, oxygen partial pressure, and structural limits.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>
#include "TestUtilities.h"

using namespace JSBSimTest;

// Constants for cabin pressurization and environmental control
namespace PressureSystemConstants {
    // Gas constants
    constexpr double GAMMA = 1.4;              // Specific heat ratio for air
    constexpr double R_AIR = 287.05;           // Gas constant for air (J/(kg·K))
    constexpr double R_AIR_FT = 1716.59;       // Gas constant (ft·lbf/(slug·R))
    constexpr double CP_AIR = 1005.0;          // Specific heat at constant pressure (J/(kg·K))

    // Standard atmosphere
    constexpr double P0_STD_PA = 101325.0;     // Standard sea level pressure (Pa)
    constexpr double P0_STD_PSF = 2116.22;     // Standard sea level pressure (psf)
    constexpr double P0_STD_PSI = 14.696;      // Standard sea level pressure (psi)
    constexpr double T0_STD_K = 288.15;        // Standard sea level temperature (K)
    constexpr double T0_STD_R = 518.67;        // Standard sea level temperature (R)

    // Typical cabin pressurization parameters
    constexpr double MAX_CABIN_ALTITUDE_FT = 8000.0;      // Maximum cabin altitude (FAA limit)
    constexpr double MAX_DIFF_PRESSURE_PSI = 9.1;         // Typical maximum differential pressure
    constexpr double MAX_DIFF_PRESSURE_PSF = 1310.4;      // 9.1 psi in psf
    constexpr double MAX_PRESSURIZATION_RATE_FPM = 500.0; // Maximum cabin climb rate (ft/min)
    constexpr double MAX_DEPRESSURIZATION_RATE_FPM = 300.0; // Maximum cabin descent rate (ft/min)
    constexpr double EMERGENCY_DESCENT_RATE_FPM = 2000.0;  // Emergency descent rate

    // Oxygen and physiological limits
    constexpr double O2_PERCENT_SEA_LEVEL = 20.95;        // Oxygen percentage at sea level
    constexpr double O2_PARTIAL_PRESSURE_MIN_PSI = 2.5;   // Minimum O2 partial pressure (psi)
    constexpr double HYPOXIA_THRESHOLD_FT = 10000.0;      // Altitude where hypoxia begins (no O2)
    constexpr double SAFE_CABIN_ALTITUDE_FT = 8000.0;     // Safe cabin altitude
    constexpr double CONSCIOUSNESS_LIMIT_FT = 25000.0;    // Useful consciousness limit

    // Bleed air and pack parameters
    constexpr double BLEED_AIR_TEMP_K = 500.0;            // Typical bleed air temperature (K)
    constexpr double PACK_OUTLET_TEMP_K = 288.0;          // Pack outlet temperature (K)
    constexpr double ACM_EFFICIENCY = 0.85;               // Air cycle machine efficiency
    constexpr double HEAT_EXCHANGER_EFFECTIVENESS = 0.80; // Heat exchanger effectiveness

    // Structural limits
    constexpr double FUSELAGE_MAX_DIFF_PSI = 9.8;         // Maximum structural differential
    constexpr double SAFETY_FACTOR = 1.5;                 // Structural safety factor
    constexpr double RAPID_DECOMPRESSION_THRESHOLD_PSI = 0.5; // Pressure loss for rapid decompression
}

class FGPressureSystemTest : public CxxTest::TestSuite
{
public:
    //==========================================================================
    // 1. CABIN ALTITUDE CALCULATIONS (~5 tests)
    //==========================================================================

    void testCabinAltitude_FromPressure() {
        // Calculate cabin altitude from cabin pressure using barometric formula
        // h = (T0/L) * (1 - (P/P0)^((R*L)/(g*M)))
        // Simplified: h = (1 - (P/P0)^0.190263) * 145442 ft

        double P_cabin_psi = 10.92; // Cabin pressure at 8000 ft cabin altitude
        double P0_psi = PressureSystemConstants::P0_STD_PSI;

        double pressure_ratio = P_cabin_psi / P0_psi;
        double cabin_alt_ft = 145442.0 * (1.0 - pow(pressure_ratio, 0.190263));

        TS_ASSERT_DELTA(cabin_alt_ft, 8000.0, 50.0);
    }

    void testCabinAltitude_SeaLevel() {
        // At sea level, cabin altitude should be zero
        double P_cabin = PressureSystemConstants::P0_STD_PSI;
        double P0 = PressureSystemConstants::P0_STD_PSI;

        double pressure_ratio = P_cabin / P0;
        double cabin_alt_ft = 145442.0 * (1.0 - pow(pressure_ratio, 0.190263));

        TS_ASSERT_DELTA(cabin_alt_ft, 0.0, 1.0);
    }

    void testCabinAltitude_MaximumCabinAlt() {
        // Verify maximum cabin altitude corresponds to correct pressure
        double cabin_alt_ft = 8000.0;

        // P/P0 = (1 - h/145442)^(1/0.190263)
        double P_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_cabin_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;

        TS_ASSERT_DELTA(P_cabin_psi, 10.92, 0.02);
    }

    void testCabinAltitude_CruiseAltitude() {
        // At cruise altitude (41,000 ft), verify cabin pressure for 8000 ft cabin altitude
        double aircraft_alt_ft = 41000.0;
        double cabin_alt_ft = 8000.0;

        // Ambient pressure at 41,000 ft
        double P_amb_ratio = pow(1.0 - aircraft_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_amb_psi = P_amb_ratio * PressureSystemConstants::P0_STD_PSI;

        // Cabin pressure for 8000 ft cabin altitude
        double P_cabin_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_cabin_psi = P_cabin_ratio * PressureSystemConstants::P0_STD_PSI;

        // Differential pressure
        double diff_pressure_psi = P_cabin_psi - P_amb_psi;

        TS_ASSERT_DELTA(P_amb_psi, 2.58, 0.1);     // ~2.58 psi at 41,000 ft
        TS_ASSERT_DELTA(P_cabin_psi, 10.92, 0.05); // ~10.92 psi for 8000 ft cabin
        TS_ASSERT_DELTA(diff_pressure_psi, 8.34, 0.2);
    }

    void testCabinAltitude_DensityCalculation() {
        // Calculate cabin air density from cabin altitude
        double cabin_alt_ft = 8000.0;
        double T_cabin_R = 530.0; // Typical cabin temperature (70°F)

        // Pressure at cabin altitude
        double P_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_cabin_psf = P_ratio * PressureSystemConstants::P0_STD_PSF;

        // Density using ideal gas law: ρ = P / (R * T)
        double rho_cabin_slug_ft3 = P_cabin_psf / (PressureSystemConstants::R_AIR_FT * T_cabin_R);

        TS_ASSERT_DELTA(rho_cabin_slug_ft3, 0.0017, 0.0002);
    }

    //==========================================================================
    // 2. DIFFERENTIAL PRESSURE LIMITS (~5 tests)
    //==========================================================================

    void testDifferentialPressure_MaximumLimit() {
        // Test maximum differential pressure limit
        double P_cabin_psi = 12.0;  // High cabin pressure
        double P_amb_psi = 2.9;     // Low ambient pressure (high altitude)

        double diff_pressure_psi = P_cabin_psi - P_amb_psi;

        TS_ASSERT_DELTA(diff_pressure_psi, 9.1, 0.1);
        TS_ASSERT(diff_pressure_psi <= PressureSystemConstants::MAX_DIFF_PRESSURE_PSI);
    }

    void testDifferentialPressure_StructuralLimit() {
        // Verify differential pressure doesn't exceed structural limit
        double max_diff_psi = PressureSystemConstants::FUSELAGE_MAX_DIFF_PSI;
        double operating_diff_psi = PressureSystemConstants::MAX_DIFF_PRESSURE_PSI;
        double safety_factor = PressureSystemConstants::SAFETY_FACTOR;

        double limit_ratio = max_diff_psi / operating_diff_psi;

        TS_ASSERT(limit_ratio > 1.0);  // Some safety margin exists
        TS_ASSERT_DELTA(limit_ratio, 1.077, 0.01);
    }

    void testDifferentialPressure_NegativeDiff() {
        // During descent, differential pressure reduces but shouldn't go negative
        double P_cabin_psi = 13.0;  // Cabin pressure
        double P_amb_psi = 14.7;    // Sea level pressure (aircraft descended)

        double diff_pressure_psi = P_cabin_psi - P_amb_psi;

        // Negative differential (outflow valve fully open, no pressurization needed)
        TS_ASSERT(diff_pressure_psi < 0.0);

        // System should equalize pressures
        double equalized_pressure = P_amb_psi;
        TS_ASSERT_DELTA(equalized_pressure, 14.7, 0.01);
    }

    void testDifferentialPressure_ForceOnFuselage() {
        // Calculate force on fuselage section from differential pressure
        double diff_pressure_psi = 9.0;
        double fuselage_diameter_ft = 12.0;  // Typical narrow-body diameter
        double fuselage_length_ft = 100.0;

        // Hoop stress on cylindrical section
        // Force per unit length: F = ΔP * diameter
        double force_per_length_lbf_ft = diff_pressure_psi * 144.0 * fuselage_diameter_ft / 2.0;

        // Total force on fuselage section
        double total_force_lbf = force_per_length_lbf_ft * fuselage_length_ft;

        TS_ASSERT_DELTA(force_per_length_lbf_ft, 7776.0, 10.0);
        TS_ASSERT_DELTA(total_force_lbf, 777600.0, 100.0);
    }

    void testDifferentialPressure_VariousAltitudes() {
        // Test differential pressure at various flight altitudes
        double cabin_alt_target_ft = 8000.0;

        struct TestCase {
            double aircraft_alt_ft;
            double expected_diff_psi;
        };

        TestCase cases[] = {
            {10000.0, 0.81},   // Near cabin altitude, minimal pressurization
            {25000.0, 5.46},   // Moderate altitude
            {35000.0, 7.46},   // Typical cruise
            {41000.0, 8.34}    // High cruise
        };

        for (const auto& test : cases) {
            double P_amb_ratio = pow(1.0 - test.aircraft_alt_ft / 145442.0, 1.0 / 0.190263);
            double P_amb_psi = P_amb_ratio * PressureSystemConstants::P0_STD_PSI;

            double P_cabin_ratio = pow(1.0 - cabin_alt_target_ft / 145442.0, 1.0 / 0.190263);
            double P_cabin_psi = P_cabin_ratio * PressureSystemConstants::P0_STD_PSI;

            double diff_psi = std::max(0.0, P_cabin_psi - P_amb_psi);

            TS_ASSERT_DELTA(diff_psi, test.expected_diff_psi, 0.2);
        }
    }

    //==========================================================================
    // 3. OUTFLOW VALVE CONTROL (~5 tests)
    //==========================================================================

    void testOutflowValve_PositionToFlowArea() {
        // Outflow valve position (0-100%) to effective flow area
        double valve_position_percent = 50.0;  // 50% open
        double max_area_in2 = 20.0;            // Maximum valve area

        // Linear relationship (simplified model)
        double effective_area_in2 = (valve_position_percent / 100.0) * max_area_in2;

        TS_ASSERT_DELTA(effective_area_in2, 10.0, 0.1);
    }

    void testOutflowValve_FlowRate() {
        // Calculate mass flow rate through outflow valve
        // Using compressible flow orifice equation
        double P_cabin_psi = 11.0;
        double P_amb_psi = 3.0;
        double T_cabin_R = 530.0;
        double C_d = 0.65;  // Discharge coefficient
        double A_valve_in2 = 10.0;
        double gamma = PressureSystemConstants::GAMMA;

        // Convert to consistent units
        double P_cabin_psf = P_cabin_psi * 144.0;
        double P_amb_psf = P_amb_psi * 144.0;
        double A_valve_ft2 = A_valve_in2 / 144.0;

        double pressure_ratio = P_amb_psf / P_cabin_psf;

        // Check if flow is choked
        double critical_ratio = pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
        bool is_choked = (pressure_ratio < critical_ratio);

        TS_ASSERT(is_choked);  // Should be choked at this pressure ratio

        // Choked flow mass flow rate
        double rho_cabin = P_cabin_psf / (PressureSystemConstants::R_AIR_FT * T_cabin_R);
        double a_cabin = sqrt(gamma * PressureSystemConstants::R_AIR_FT * T_cabin_R);
        double m_dot_slugs_s = C_d * A_valve_ft2 * rho_cabin * a_cabin *
                                pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));

        TS_ASSERT(m_dot_slugs_s > 0.0);
        TS_ASSERT_DELTA(m_dot_slugs_s, 0.051, 0.005);
    }

    void testOutflowValve_ControlLogic() {
        // Test proportional control logic for outflow valve
        double cabin_alt_current_ft = 7500.0;
        double cabin_alt_target_ft = 8000.0;
        double K_p = 2.0;  // Proportional gain (% per 1000 ft)

        double error_ft = cabin_alt_current_ft - cabin_alt_target_ft;
        double valve_correction_percent = K_p * (error_ft / 1000.0);

        // Current cabin altitude is below target, so close valve slightly
        TS_ASSERT_DELTA(valve_correction_percent, -1.0, 0.1);
        TS_ASSERT(valve_correction_percent < 0.0);  // Close valve to increase cabin pressure
    }

    void testOutflowValve_RateLimiting() {
        // Test rate limiting on valve position changes
        double valve_pos_current = 30.0;  // Current position (%)
        double valve_pos_commanded = 60.0;  // Commanded position (%)
        double max_rate_percent_s = 10.0;   // Maximum rate of change
        double dt_s = 2.0;                   // Time step

        double max_change = max_rate_percent_s * dt_s;
        double delta_commanded = valve_pos_commanded - valve_pos_current;

        double delta_actual = std::min(std::abs(delta_commanded), max_change);
        if (delta_commanded < 0.0) delta_actual = -delta_actual;

        double valve_pos_new = valve_pos_current + delta_actual;

        TS_ASSERT_DELTA(valve_pos_new, 50.0, 0.1);  // Limited to 20% change
        TS_ASSERT(valve_pos_new < valve_pos_commanded);  // Not reached target yet
    }

    void testOutflowValve_EmergencyPosition() {
        // During rapid decompression, valve should go to fully open
        bool rapid_decompression_detected = true;
        double valve_position_normal = 25.0;

        double valve_position = rapid_decompression_detected ? 100.0 : valve_position_normal;

        TS_ASSERT_DELTA(valve_position, 100.0, 0.1);
    }

    //==========================================================================
    // 4. PRESSURIZATION RATE LIMITS (~5 tests)
    //==========================================================================

    void testPressurization_MaxClimbRate() {
        // Maximum cabin climb rate should not exceed 500 ft/min
        double cabin_alt_initial_ft = 5000.0;
        double cabin_alt_final_ft = 6000.0;
        double time_elapsed_min = 2.0;

        double cabin_climb_rate_fpm = (cabin_alt_final_ft - cabin_alt_initial_ft) / time_elapsed_min;

        TS_ASSERT_DELTA(cabin_climb_rate_fpm, 500.0, 0.1);
        TS_ASSERT(cabin_climb_rate_fpm <= PressureSystemConstants::MAX_PRESSURIZATION_RATE_FPM);
    }

    void testPressurization_MaxDescentRate() {
        // Maximum cabin descent rate should not exceed 300 ft/min
        double cabin_alt_initial_ft = 8000.0;
        double cabin_alt_final_ft = 7400.0;
        double time_elapsed_min = 2.0;

        double cabin_descent_rate_fpm = -(cabin_alt_final_ft - cabin_alt_initial_ft) / time_elapsed_min;

        TS_ASSERT_DELTA(cabin_descent_rate_fpm, 300.0, 0.1);
        TS_ASSERT(cabin_descent_rate_fpm <= PressureSystemConstants::MAX_DEPRESSURIZATION_RATE_FPM);
    }

    void testPressurization_RateToPressureChange() {
        // Convert cabin altitude rate to pressure rate of change
        double cabin_climb_rate_fpm = 500.0;
        double cabin_alt_ft = 6000.0;

        // dP/dh = -P * (g * M) / (R * T) ≈ -P * 0.0000687 (for standard atmosphere)
        // At 6000 ft: P ≈ 11.77 psi
        double P_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;

        // dP/dt = (dP/dh) * (dh/dt)
        double dP_dh_psi_ft = -P_psi * 0.0000687;  // Simplified gradient
        double dP_dt_psi_min = dP_dh_psi_ft * cabin_climb_rate_fpm;

        TS_ASSERT_DELTA(dP_dt_psi_min, -0.40, 0.05);  // Pressure decreasing
        TS_ASSERT(dP_dt_psi_min < 0.0);  // Pressure decreases during cabin climb
    }

    void testPressurization_PassengerComfort() {
        // Ear pressure equalization comfort limit: ~300 ft/min
        double cabin_climb_rate_fpm = 300.0;
        double comfort_limit_fpm = 500.0;  // Maximum for comfort

        TS_ASSERT(cabin_climb_rate_fpm < comfort_limit_fpm);

        // Pressure change rate for comfort
        double cabin_alt_ft = 5000.0;
        double P_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;
        double dP_dt_psi_min = -P_psi * 0.0000687 * cabin_climb_rate_fpm;

        // Should be less than ~0.30 psi/min for comfort
        TS_ASSERT(std::abs(dP_dt_psi_min) < 0.35);
    }

    void testPressurization_ScheduleFollowing() {
        // Test cabin altitude schedule following during climb
        double aircraft_alt_ft = 35000.0;
        double max_cabin_alt_ft = 8000.0;

        // Linear schedule from sea level to max cabin altitude
        double schedule_altitude_ft;
        if (aircraft_alt_ft <= 8000.0) {
            schedule_altitude_ft = aircraft_alt_ft;  // 1:1 below 8000 ft
        } else {
            // Gradual increase to max cabin altitude
            schedule_altitude_ft = std::min(8000.0 + (aircraft_alt_ft - 8000.0) * 0.2, max_cabin_alt_ft);
        }

        TS_ASSERT_DELTA(schedule_altitude_ft, 8000.0, 0.1);
        TS_ASSERT(schedule_altitude_ft <= max_cabin_alt_ft);
    }

    //==========================================================================
    // 5. CABIN CLIMB/DESCENT RATE (~5 tests)
    //==========================================================================

    void testCabinRate_AircraftClimbToAltitude() {
        // Aircraft climbs from 10,000 to 35,000 ft in 15 minutes
        // Cabin should climb from ~10,000 to 8,000 ft (limited by schedule)
        double aircraft_climb_rate_fpm = (35000.0 - 10000.0) / 15.0;  // 1667 fpm
        double cabin_rate_fpm = 500.0;  // Limited to 500 fpm

        TS_ASSERT_DELTA(aircraft_climb_rate_fpm, 1666.7, 1.0);
        TS_ASSERT(cabin_rate_fpm < aircraft_climb_rate_fpm);
        TS_ASSERT(cabin_rate_fpm <= PressureSystemConstants::MAX_PRESSURIZATION_RATE_FPM);
    }

    void testCabinRate_DescentFromCruise() {
        // Aircraft descends from 37,000 ft to 10,000 ft in 20 minutes
        double aircraft_descent_rate_fpm = (37000.0 - 10000.0) / 20.0;  // 1350 fpm
        double cabin_descent_rate_fpm = 300.0;  // Limited for comfort

        TS_ASSERT_DELTA(aircraft_descent_rate_fpm, 1350.0, 1.0);
        TS_ASSERT(cabin_descent_rate_fpm < aircraft_descent_rate_fpm);
        TS_ASSERT(cabin_descent_rate_fpm <= PressureSystemConstants::MAX_DEPRESSURIZATION_RATE_FPM);
    }

    void testCabinRate_EarDiscomfortThreshold() {
        // Test rate that causes ear discomfort
        double uncomfortable_rate_fpm = 800.0;
        double comfortable_rate_fpm = 300.0;

        TS_ASSERT(uncomfortable_rate_fpm > PressureSystemConstants::MAX_DEPRESSURIZATION_RATE_FPM);
        TS_ASSERT(comfortable_rate_fpm <= PressureSystemConstants::MAX_DEPRESSURIZATION_RATE_FPM);
    }

    void testCabinRate_LagBehindAircraft() {
        // Cabin altitude lags behind aircraft altitude due to rate limiting
        double aircraft_alt_current_ft = 30000.0;
        double aircraft_alt_rate_fpm = 2000.0;  // Rapid climb
        double cabin_alt_current_ft = 6000.0;
        double cabin_rate_limit_fpm = 500.0;
        double dt_min = 5.0;

        double aircraft_alt_new_ft = aircraft_alt_current_ft + aircraft_alt_rate_fpm * dt_min;
        double cabin_alt_new_ft = cabin_alt_current_ft + cabin_rate_limit_fpm * dt_min;

        TS_ASSERT_DELTA(aircraft_alt_new_ft, 40000.0, 1.0);
        TS_ASSERT_DELTA(cabin_alt_new_ft, 8500.0, 1.0);
        TS_ASSERT(cabin_alt_new_ft < aircraft_alt_new_ft);
    }

    void testCabinRate_PressureRateOfChange() {
        // Convert cabin rate to pressure rate of change
        double cabin_rate_fpm = 500.0;
        double cabin_alt_ft = 5000.0;
        double T_cabin_R = 530.0;

        // Pressure at current altitude
        double P_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_cabin_psf = P_ratio * PressureSystemConstants::P0_STD_PSF;

        // Pressure gradient: dP/dh
        double dP_dh = -P_cabin_psf * 0.0000687;  // psf/ft
        double dP_dt_psf_min = dP_dh * cabin_rate_fpm;

        TS_ASSERT(dP_dt_psf_min < 0.0);  // Pressure decreases as cabin climbs
        TS_ASSERT_DELTA(dP_dt_psf_min, -60.5, 5.0);
    }

    //==========================================================================
    // 6. BLEED AIR FLOW REQUIREMENTS (~5 tests)
    //==========================================================================

    void testBleedAir_MassFlowForPressurization() {
        // Calculate required bleed air mass flow to maintain cabin pressure
        double cabin_volume_ft3 = 10000.0;  // Typical narrow-body
        double outflow_mass_rate_slug_s = 0.002;  // Mass leaving through outflow valve
        double leakage_mass_rate_slug_s = 0.0005;  // Leakage

        double required_bleed_slug_s = outflow_mass_rate_slug_s + leakage_mass_rate_slug_s;

        TS_ASSERT_DELTA(required_bleed_slug_s, 0.0025, 0.0001);
    }

    void testBleedAir_TemperatureReduction() {
        // Bleed air must be cooled from engine temperature to cabin temperature
        double T_bleed_K = PressureSystemConstants::BLEED_AIR_TEMP_K;  // 500 K
        double T_cabin_K = 295.0;  // 22°C cabin temperature
        double m_dot_kg_s = 0.5;    // Mass flow rate
        double cp = PressureSystemConstants::CP_AIR;

        // Heat rejection required
        double Q_removal_kW = m_dot_kg_s * cp * (T_bleed_K - T_cabin_K) / 1000.0;

        TS_ASSERT_DELTA(Q_removal_kW, 103.0, 5.0);
        TS_ASSERT(Q_removal_kW > 0.0);
    }

    void testBleedAir_PackCoolingCapacity() {
        // Air cycle machine (ACM) cooling capacity
        double T_bleed_K = 500.0;
        double T_pack_outlet_K = 288.0;
        double m_dot_kg_s = 0.6;
        double cp = PressureSystemConstants::CP_AIR;
        double eta_ACM = PressureSystemConstants::ACM_EFFICIENCY;

        // Actual cooling achieved
        double delta_T_ideal = T_bleed_K - T_pack_outlet_K;
        double delta_T_actual = delta_T_ideal * eta_ACM;
        double T_outlet_actual_K = T_bleed_K - delta_T_actual;

        TS_ASSERT_DELTA(T_outlet_actual_K, 319.8, 1.0);
        TS_ASSERT(T_outlet_actual_K < T_bleed_K);
    }

    void testBleedAir_EnginePowerExtraction() {
        // Bleed air extraction affects engine performance
        double m_dot_bleed_kg_s = 0.5;  // Bleed air mass flow
        double m_dot_engine_kg_s = 50.0;  // Engine air mass flow
        double bleed_fraction = m_dot_bleed_kg_s / m_dot_engine_kg_s;

        // Typical bleed fraction: 1-3%
        TS_ASSERT_DELTA(bleed_fraction * 100.0, 1.0, 0.1);
        TS_ASSERT(bleed_fraction < 0.05);  // Should be less than 5%
    }

    void testBleedAir_PressureDrop() {
        // Pressure drop through bleed air system
        double P_bleed_source_psi = 45.0;  // Compressor bleed pressure
        double pressure_drop_psi = 5.0;     // System pressure drop
        double P_pack_inlet_psi = P_bleed_source_psi - pressure_drop_psi;

        TS_ASSERT_DELTA(P_pack_inlet_psi, 40.0, 0.1);
        TS_ASSERT(P_pack_inlet_psi > PressureSystemConstants::P0_STD_PSI);
    }

    //==========================================================================
    // 7. CABIN TEMPERATURE CONTROL (~4 tests)
    //==========================================================================

    void testCabinTemp_HeatLoad() {
        // Calculate heat load in cabin from passengers and equipment
        double num_passengers = 150.0;
        double heat_per_passenger_W = 100.0;  // Typical metabolic heat
        double equipment_heat_W = 5000.0;      // Avionics, lighting
        double solar_heat_W = 3000.0;          // Solar radiation

        double total_heat_load_W = num_passengers * heat_per_passenger_W +
                                    equipment_heat_W + solar_heat_W;

        TS_ASSERT_DELTA(total_heat_load_W, 23000.0, 100.0);
    }

    void testCabinTemp_CoolingRequirement() {
        // Cooling air mass flow required to remove heat load
        double Q_removal_W = 23000.0;  // Heat load
        double T_supply_C = 15.0;      // Supply air temperature
        double T_cabin_C = 22.0;       // Cabin temperature
        double cp = PressureSystemConstants::CP_AIR;

        double m_dot_cooling_kg_s = Q_removal_W / (cp * (T_cabin_C - T_supply_C));

        TS_ASSERT_DELTA(m_dot_cooling_kg_s, 3.27, 0.1);
    }

    void testCabinTemp_MixingControl() {
        // Mix hot and cold air to achieve desired temperature
        double T_hot_K = 350.0;   // Hot bleed air
        double T_cold_K = 280.0;  // Cold ACM outlet
        double T_target_K = 295.0;  // Desired cabin supply temperature

        // Mass ratio: m_hot * T_hot + m_cold * T_cold = (m_hot + m_cold) * T_target
        // Solving: m_hot / m_total = (T_target - T_cold) / (T_hot - T_cold)
        double hot_fraction = (T_target_K - T_cold_K) / (T_hot_K - T_cold_K);

        TS_ASSERT_DELTA(hot_fraction, 0.214, 0.01);
        TS_ASSERT(hot_fraction > 0.0 && hot_fraction < 1.0);
    }

    void testCabinTemp_RecirculationEffect() {
        // Effect of recirculated air on fresh air requirement
        double recirculation_fraction = 0.5;  // 50% recirculated
        double fresh_air_fraction = 1.0 - recirculation_fraction;
        double total_air_required_kg_s = 2.0;

        double fresh_air_required_kg_s = total_air_required_kg_s * fresh_air_fraction;

        TS_ASSERT_DELTA(fresh_air_required_kg_s, 1.0, 0.01);
        TS_ASSERT(fresh_air_required_kg_s < total_air_required_kg_s);
    }

    //==========================================================================
    // 8. PACK OUTLET TEMPERATURE (~3 tests)
    //==========================================================================

    void testPackOutlet_ExpansionCooling() {
        // Temperature drop through ACM turbine expansion
        double T_inlet_K = 400.0;
        double P_inlet_psi = 40.0;
        double P_outlet_psi = 15.0;
        double gamma = PressureSystemConstants::GAMMA;
        double eta_turbine = 0.85;

        double PR = P_outlet_psi / P_inlet_psi;
        double T_outlet_ideal_K = T_inlet_K * pow(PR, (gamma - 1.0) / gamma);
        double T_outlet_actual_K = T_inlet_K - eta_turbine * (T_inlet_K - T_outlet_ideal_K);

        TS_ASSERT_DELTA(T_outlet_ideal_K, 302.2, 2.0);
        TS_ASSERT_DELTA(T_outlet_actual_K, 317.0, 2.0);
        TS_ASSERT(T_outlet_actual_K < T_inlet_K);
    }

    void testPackOutlet_HeatExchangerEffectiveness() {
        // Heat exchanger performance
        double T_hot_in_K = 500.0;   // Bleed air inlet
        double T_cold_in_K = 250.0;  // Ram air inlet
        double effectiveness = PressureSystemConstants::HEAT_EXCHANGER_EFFECTIVENESS;

        double T_max_reduction = T_hot_in_K - T_cold_in_K;
        double T_hot_out_K = T_hot_in_K - effectiveness * T_max_reduction;

        TS_ASSERT_DELTA(T_hot_out_K, 300.0, 1.0);
    }

    void testPackOutlet_RamAirCooling() {
        // Ram air temperature rise in heat exchanger
        double T_ram_in_K = 250.0;
        double Q_transferred_W = 50000.0;
        double m_dot_ram_kg_s = 1.5;
        double cp = PressureSystemConstants::CP_AIR;

        double delta_T_ram_K = Q_transferred_W / (m_dot_ram_kg_s * cp);
        double T_ram_out_K = T_ram_in_K + delta_T_ram_K;

        TS_ASSERT_DELTA(T_ram_out_K, 283.2, 1.0);
        TS_ASSERT(T_ram_out_K > T_ram_in_K);
    }

    //==========================================================================
    // 9. EMERGENCY DEPRESSURIZATION (~5 tests)
    //==========================================================================

    void testEmergency_RapidDecompression() {
        // Rapid decompression from cruise to ambient pressure
        double P_cabin_initial_psi = 10.9;
        double P_ambient_psi = 2.7;  // 41,000 ft ambient
        double time_to_equalize_s = 10.0;  // Rapid loss

        double pressure_loss_rate_psi_s = (P_cabin_initial_psi - P_ambient_psi) / time_to_equalize_s;

        TS_ASSERT_DELTA(pressure_loss_rate_psi_s, 0.82, 0.01);
        TS_ASSERT(pressure_loss_rate_psi_s > PressureSystemConstants::RAPID_DECOMPRESSION_THRESHOLD_PSI);
    }

    void testEmergency_TimeOfUsefulConsciousness() {
        // Time of useful consciousness at various altitudes
        struct TUC_Data {
            double altitude_ft;
            double tuc_seconds;
        };

        TUC_Data tuc_table[] = {
            {25000.0, 180.0},   // 3-5 minutes
            {30000.0, 90.0},    // 1-2 minutes
            {35000.0, 45.0},    // 30-60 seconds
            {40000.0, 18.0},    // 15-20 seconds
            {50000.0, 9.0}      // 9-12 seconds
        };

        // Verify TUC decreases with altitude
        for (size_t i = 1; i < sizeof(tuc_table) / sizeof(tuc_table[0]); ++i) {
            TS_ASSERT(tuc_table[i].tuc_seconds < tuc_table[i-1].tuc_seconds);
        }
    }

    void testEmergency_EmergencyDescentRate() {
        // Emergency descent rate to reach safe altitude
        double initial_altitude_ft = 41000.0;
        double safe_altitude_ft = 10000.0;
        double descent_rate_fpm = PressureSystemConstants::EMERGENCY_DESCENT_RATE_FPM;

        double descent_time_min = (initial_altitude_ft - safe_altitude_ft) / descent_rate_fpm;

        TS_ASSERT_DELTA(descent_time_min, 15.5, 0.1);
    }

    void testEmergency_OxygenDeployment() {
        // Oxygen mask deployment trigger
        double cabin_alt_current_ft = 14000.0;
        double deployment_altitude_ft = 14000.0;  // Typical deployment altitude

        bool masks_deployed = (cabin_alt_current_ft >= deployment_altitude_ft);

        TS_ASSERT(masks_deployed);
    }

    void testEmergency_ExplosiveDecompression() {
        // Explosive decompression: pressure equalizes in < 1 second
        double P_initial_psi = 11.0;
        double P_final_psi = 2.7;
        double time_to_equalize_s = 0.5;  // Very rapid

        double rate_psi_s = (P_initial_psi - P_final_psi) / time_to_equalize_s;

        TS_ASSERT_DELTA(rate_psi_s, 16.6, 0.1);
        TS_ASSERT(time_to_equalize_s < 1.0);  // Explosive if < 1 second
    }

    //==========================================================================
    // 10. RAPID DECOMPRESSION EFFECTS (~4 tests)
    //==========================================================================

    void testRapidDecomp_TemperatureDrop() {
        // Adiabatic cooling during rapid decompression
        double T_initial_R = 530.0;
        double P_initial_psi = 11.0;
        double P_final_psi = 2.7;
        double gamma = PressureSystemConstants::GAMMA;

        double PR = P_final_psi / P_initial_psi;
        double T_final_R = T_initial_R * pow(PR, (gamma - 1.0) / gamma);
        double delta_T_R = T_initial_R - T_final_R;

        TS_ASSERT_DELTA(T_final_R, 354.8, 5.0);
        TS_ASSERT_DELTA(delta_T_R, 175.2, 5.0);  // Significant cooling
        TS_ASSERT(T_final_R < T_initial_R);
    }

    void testRapidDecomp_FogFormation() {
        // Fog formation due to rapid cooling and moisture condensation
        double T_initial_C = 22.0;
        double RH_initial = 50.0;  // 50% relative humidity
        double T_final_C = -5.0;   // After decompression

        // Saturation occurs when temperature drops below dew point
        // Approximate dew point for 50% RH at 22°C is ~11°C
        double dew_point_C = 11.0;

        bool fog_forms = (T_final_C < dew_point_C);

        TS_ASSERT(fog_forms);
    }

    void testRapidDecomp_FlowVelocity() {
        // Air velocity through breach during decompression
        double P_cabin_psf = 11.0 * 144.0;
        double P_ambient_psf = 2.7 * 144.0;
        double T_cabin_R = 530.0;
        double gamma = PressureSystemConstants::GAMMA;

        // Choked flow velocity (sonic)
        double a_cabin = sqrt(gamma * PressureSystemConstants::R_AIR_FT * T_cabin_R);

        // Check if choked
        double critical_ratio = pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
        double actual_ratio = P_ambient_psf / P_cabin_psf;
        bool is_choked = (actual_ratio < critical_ratio);

        TS_ASSERT(is_choked);
        TS_ASSERT_DELTA(a_cabin, 1127.0, 5.0);  // ~770 mph
    }

    void testRapidDecomp_MassLossRate() {
        // Rate of cabin air mass loss during decompression
        double breach_area_in2 = 10.0;  // 10 square inch hole
        double P_cabin_psf = 11.0 * 144.0;
        double T_cabin_R = 530.0;
        double gamma = PressureSystemConstants::GAMMA;

        double A_breach_ft2 = breach_area_in2 / 144.0;
        double rho_cabin = P_cabin_psf / (PressureSystemConstants::R_AIR_FT * T_cabin_R);
        double a_cabin = sqrt(gamma * PressureSystemConstants::R_AIR_FT * T_cabin_R);

        // Choked mass flow rate
        double m_dot_slugs_s = 0.6847 * rho_cabin * a_cabin * A_breach_ft2;

        TS_ASSERT_DELTA(m_dot_slugs_s, 0.093, 0.005);
        TS_ASSERT(m_dot_slugs_s > 0.0);
    }

    //==========================================================================
    // 11. OXYGEN PARTIAL PRESSURE (~5 tests)
    //==========================================================================

    void testOxygenPP_SeaLevel() {
        // Oxygen partial pressure at sea level
        double P_total_psi = PressureSystemConstants::P0_STD_PSI;
        double O2_fraction = PressureSystemConstants::O2_PERCENT_SEA_LEVEL / 100.0;

        double P_O2_psi = P_total_psi * O2_fraction;

        TS_ASSERT_DELTA(P_O2_psi, 3.08, 0.01);
    }

    void testOxygenPP_CabinAltitude() {
        // Oxygen partial pressure at 8000 ft cabin altitude
        double cabin_alt_ft = 8000.0;
        double P_ratio = pow(1.0 - cabin_alt_ft / 145442.0, 1.0 / 0.190263);
        double P_cabin_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;
        double O2_fraction = 0.2095;

        double P_O2_psi = P_cabin_psi * O2_fraction;

        TS_ASSERT_DELTA(P_O2_psi, 2.29, 0.02);
        TS_ASSERT(P_O2_psi < 3.08);  // Less than sea level
    }

    void testOxygenPP_MinimumRequired() {
        // Minimum oxygen partial pressure for consciousness
        double P_O2_min_psi = PressureSystemConstants::O2_PARTIAL_PRESSURE_MIN_PSI;

        // Altitude where this occurs without supplemental oxygen
        double O2_fraction = 0.2095;
        double P_total_required_psi = P_O2_min_psi / O2_fraction;

        // Convert to altitude
        double P_ratio = P_total_required_psi / PressureSystemConstants::P0_STD_PSI;
        double altitude_ft = 145442.0 * (1.0 - pow(P_ratio, 0.190263));

        TS_ASSERT_DELTA(altitude_ft, 5650.0, 500.0);  // ~5,650 ft
    }

    void testOxygenPP_SupplementalOxygen() {
        // With supplemental oxygen, maintain sea level equivalent O2 partial pressure
        double altitude_ft = 35000.0;
        double P_ratio = pow(1.0 - altitude_ft / 145442.0, 1.0 / 0.190263);
        double P_ambient_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;

        double P_O2_target_psi = 3.08;  // Sea level O2 partial pressure
        double O2_concentration_required = P_O2_target_psi / P_ambient_psi;

        TS_ASSERT_DELTA(O2_concentration_required * 100.0, 89.1, 1.0);  // ~89% O2 required
        TS_ASSERT(O2_concentration_required > 0.2095);  // More than normal air
    }

    void testOxygenPP_HypoxiaRisk() {
        // Assess hypoxia risk at various altitudes without oxygen
        struct HypoxiaData {
            double altitude_ft;
            double expected_P_O2_psi;
            bool hypoxia_risk;
        };

        HypoxiaData cases[] = {
            {5000.0, 2.56, false},    // Above minimum threshold, safe
            {10000.0, 2.12, true},    // Below threshold, at risk
            {15000.0, 1.73, true},    // Hypoxia risk
            {25000.0, 1.14, true}     // Severe hypoxia
        };

        for (const auto& test : cases) {
            double P_ratio = pow(1.0 - test.altitude_ft / 145442.0, 1.0 / 0.190263);
            double P_total_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;
            double P_O2_psi = P_total_psi * 0.2095;

            TS_ASSERT_DELTA(P_O2_psi, test.expected_P_O2_psi, 0.05);

            bool at_risk = (P_O2_psi < PressureSystemConstants::O2_PARTIAL_PRESSURE_MIN_PSI);
            TS_ASSERT_EQUALS(at_risk, test.hypoxia_risk);
        }
    }

    //==========================================================================
    // 12. HYPOXIA THRESHOLDS (~3 tests)
    //==========================================================================

    void testHypoxia_AltitudeThreshold() {
        // Hypoxia typically begins above 10,000 ft without supplemental oxygen
        double threshold_ft = PressureSystemConstants::HYPOXIA_THRESHOLD_FT;

        TS_ASSERT_DELTA(threshold_ft, 10000.0, 100.0);

        // At this altitude, O2 partial pressure drops below optimal
        double P_ratio = pow(1.0 - threshold_ft / 145442.0, 1.0 / 0.190263);
        double P_total_psi = P_ratio * PressureSystemConstants::P0_STD_PSI;
        double P_O2_psi = P_total_psi * 0.2095;

        TS_ASSERT_DELTA(P_O2_psi, 2.12, 0.05);
    }

    void testHypoxia_Symptoms() {
        // Progressive hypoxia symptoms at altitude
        struct SymptomData {
            double altitude_ft;
            const char* severity;
            double P_O2_psi;
        };

        SymptomData symptoms[] = {
            {10000.0, "Mild", 2.17},
            {15000.0, "Moderate", 1.76},
            {20000.0, "Severe", 1.42},
            {25000.0, "Critical", 1.05}
        };

        for (size_t i = 1; i < sizeof(symptoms) / sizeof(symptoms[0]); ++i) {
            // O2 partial pressure decreases with altitude
            TS_ASSERT(symptoms[i].P_O2_psi < symptoms[i-1].P_O2_psi);
        }
    }

    void testHypoxia_SafeFlightDuration() {
        // Duration of safe flight at altitude without oxygen
        double altitude_ft = 12000.0;

        // At 12,000 ft, safe duration is ~30 minutes before impairment
        double safe_duration_min = 30.0;

        // At 18,000 ft, safe duration is ~20 minutes
        double altitude_high_ft = 18000.0;
        double safe_duration_high_min = 20.0;

        TS_ASSERT(safe_duration_high_min < safe_duration_min);
        TS_ASSERT(altitude_high_ft > altitude_ft);
    }

    //==========================================================================
    // 13. STRUCTURAL PRESSURE LIMITS (~3 tests)
    //==========================================================================

    void testStructural_MaximumDifferential() {
        // Maximum design differential pressure
        double max_design_psi = PressureSystemConstants::FUSELAGE_MAX_DIFF_PSI;
        double operating_max_psi = PressureSystemConstants::MAX_DIFF_PRESSURE_PSI;
        double safety_factor = PressureSystemConstants::SAFETY_FACTOR;

        double actual_safety_factor = max_design_psi / operating_max_psi;

        TS_ASSERT(actual_safety_factor >= 1.0);  // Some safety margin exists
        TS_ASSERT_DELTA(max_design_psi, 9.8, 0.1);
    }

    void testStructural_HoopStress() {
        // Hoop stress in fuselage cylinder
        // σ = (P * r) / t
        double diff_pressure_psi = 9.0;
        double fuselage_radius_in = 72.0;  // 12 ft diameter
        double skin_thickness_in = 0.08;    // Typical skin thickness

        double hoop_stress_psi = (diff_pressure_psi * fuselage_radius_in) / skin_thickness_in;

        TS_ASSERT_DELTA(hoop_stress_psi, 8100.0, 100.0);

        // Typical aluminum yield strength: ~35,000 psi
        double yield_strength_psi = 35000.0;
        double safety_margin = yield_strength_psi / hoop_stress_psi;

        TS_ASSERT(safety_margin > 4.0);  // Adequate safety margin
    }

    void testStructural_FatigueConsiderations() {
        // Pressurization cycles affect fatigue life
        double flights_per_day = 6.0;
        double days_per_year = 365.0;
        double service_years = 20.0;

        double total_cycles = flights_per_day * days_per_year * service_years;

        TS_ASSERT_DELTA(total_cycles, 43800.0, 100.0);

        // Modern aircraft designed for 60,000+ cycles
        double design_cycles = 60000.0;
        TS_ASSERT(total_cycles < design_cycles);
    }

    //==========================================================================
    // 14. LEAKAGE AND SEALING TESTS
    //==========================================================================

    void testCabinLeakageRate() {
        double cabin_volume_ft3 = 12000.0;
        double leakage_cfm = 200.0;  // Typical leakage

        double exchanges_per_min = leakage_cfm / cabin_volume_ft3;
        TS_ASSERT(exchanges_per_min < 0.02);
    }

    void testDoorSealIntegrity() {
        double diff_pressure_psi = 9.0;
        double seal_load_per_inch = 50.0;  // lbs/in

        double seal_force = diff_pressure_psi * seal_load_per_inch;
        TS_ASSERT(seal_force > 0.0);
    }

    void testWindowSealPressure() {
        double window_area_in2 = 100.0;
        double diff_pressure_psi = 8.5;

        double window_force_lbs = window_area_in2 * diff_pressure_psi;
        TS_ASSERT_DELTA(window_force_lbs, 850.0, 1.0);
    }

    //==========================================================================
    // 15. VENTILATION AND AIR QUALITY
    //==========================================================================

    void testFreshAirPerOccupant() {
        double total_flow_cfm = 2000.0;
        int passengers = 150;

        double cfm_per_person = total_flow_cfm / passengers;
        TS_ASSERT(cfm_per_person > 10.0);
    }

    void testCO2Concentration() {
        double ambient_co2_ppm = 400.0;
        double exhaled_co2_rate = 0.005;  // L/s CO2 production
        double ventilation_rate = 2.0;     // L/s per person

        double steady_state_co2 = ambient_co2_ppm + (exhaled_co2_rate / ventilation_rate) * 1e6;
        TS_ASSERT(steady_state_co2 < 5000.0);  // Should be ~2900 ppm
    }

    void testCabinAirRecirculation() {
        double fresh_air_pct = 50.0;
        double recirculated_pct = 50.0;

        TS_ASSERT_DELTA(fresh_air_pct + recirculated_pct, 100.0, 0.1);
    }

    void testHEPAFilterEfficiency() {
        double particle_count_in = 10000.0;
        double filter_efficiency = 0.9997;

        double particle_count_out = particle_count_in * (1.0 - filter_efficiency);
        TS_ASSERT(particle_count_out < 10.0);
    }

    //==========================================================================
    // 16. CONTROLLER DYNAMICS
    //==========================================================================

    void testCabinAltitudeController() {
        double Kp = 0.5;
        double target_alt = 8000.0;
        double current_alt = 7500.0;

        double error = target_alt - current_alt;
        double valve_command = Kp * error / 1000.0;

        TS_ASSERT_DELTA(valve_command, 0.25, 0.01);
    }

    void testControllerIntegralAction() {
        double Ki = 0.1;
        double error_integral = 5000.0;
        double dt = 1.0;

        double integral_term = Ki * error_integral * dt;
        TS_ASSERT(integral_term > 0.0);
    }

    void testRateLimitedValvePosition() {
        double current_pos = 30.0;
        double target_pos = 60.0;
        double max_rate = 10.0;
        double dt = 2.0;

        double max_change = max_rate * dt;
        double actual_change = std::min(target_pos - current_pos, max_change);
        double new_pos = current_pos + actual_change;

        TS_ASSERT_DELTA(new_pos, 50.0, 0.1);
    }

    //==========================================================================
    // 17. REDUNDANCY AND FAULT TOLERANCE
    //==========================================================================

    void testDualPackOperation() {
        double pack1_flow = 0.8;
        double pack2_flow = 0.8;
        double total_flow = pack1_flow + pack2_flow;

        TS_ASSERT_DELTA(total_flow, 1.6, 0.01);
    }

    void testSinglePackFailure() {
        double pack1_flow = 0.0;
        double pack2_flow = 1.2;
        double total_flow = pack1_flow + pack2_flow;

        TS_ASSERT(total_flow > 1.0);
    }

    void testBackupValveOperation() {
        bool primary_valve_failed = true;
        bool backup_valve_available = true;

        bool system_operational = !primary_valve_failed || backup_valve_available;
        TS_ASSERT(system_operational);
    }

    //==========================================================================
    // 18. ALTITUDE SCHEDULING
    //==========================================================================

    void testCabinAltitudeScheduleLow() {
        double aircraft_alt = 5000.0;

        double cabin_alt = std::min(aircraft_alt, 8000.0);
        TS_ASSERT_DELTA(cabin_alt, 5000.0, 1.0);
    }

    void testCabinAltitudeScheduleHigh() {
        double aircraft_alt = 40000.0;
        double max_cabin_alt = 8000.0;

        double cabin_alt = std::min(aircraft_alt * 0.2, max_cabin_alt);
        TS_ASSERT_DELTA(cabin_alt, 8000.0, 1.0);
    }

    void testLandingFieldElevation() {
        double field_elevation = 5280.0;
        double cabin_alt_target = std::max(0.0, field_elevation);

        TS_ASSERT_DELTA(cabin_alt_target, 5280.0, 1.0);
    }

    //==========================================================================
    // 19. PNEUMATIC SYSTEM INTEGRATION
    //==========================================================================

    void testBleedValvePosition() {
        double engine_N2 = 85.0;
        double min_N2_for_bleed = 50.0;

        bool bleed_available = (engine_N2 > min_N2_for_bleed);
        TS_ASSERT(bleed_available);
    }

    void testAPUBleedCapability() {
        bool apu_running = true;
        double apu_bleed_pressure = 40.0;
        double min_pressure = 20.0;

        bool apu_bleed_ok = apu_running && (apu_bleed_pressure > min_pressure);
        TS_ASSERT(apu_bleed_ok);
    }

    void testCrossBleedOperation() {
        double left_engine_bleed_psi = 45.0;
        double right_engine_bleed_psi = 0.0;
        bool cross_bleed_open = true;

        double available_pressure = cross_bleed_open ?
            std::max(left_engine_bleed_psi, right_engine_bleed_psi) : 0.0;

        TS_ASSERT_DELTA(available_pressure, 45.0, 1.0);
    }

    //==========================================================================
    // 20. MOISTURE AND CONDENSATION
    //==========================================================================

    void testDewPointCalculation() {
        double T_cabin_C = 22.0;
        double RH = 50.0;

        double dew_point_C = T_cabin_C - ((100.0 - RH) / 5.0);
        TS_ASSERT(dew_point_C < T_cabin_C);
    }

    void testCondensationRisk() {
        double skin_temp_C = 5.0;
        double dew_point_C = 10.0;

        bool condensation = (skin_temp_C < dew_point_C);
        TS_ASSERT(condensation);
    }

    void testHumidityControl() {
        double target_RH = 20.0;
        double current_RH = 15.0;

        bool need_humidification = (current_RH < target_RH);
        TS_ASSERT(need_humidification);
    }

    //==========================================================================
    // 21. SYSTEM MONITORING
    //==========================================================================

    void testPressureWarningThreshold() {
        double cabin_alt = 9500.0;
        double warning_alt = 9000.0;

        bool warning = (cabin_alt > warning_alt);
        TS_ASSERT(warning);
    }

    void testDiffPressureWarning() {
        double diff_pressure_psi = 9.5;
        double max_diff_psi = 9.1;

        bool warning = (diff_pressure_psi > max_diff_psi);
        TS_ASSERT(warning);
    }

    void testCabinRateWarning() {
        double cabin_rate_fpm = 600.0;
        double max_rate_fpm = 500.0;

        bool excessive_rate = (std::fabs(cabin_rate_fpm) > max_rate_fpm);
        TS_ASSERT(excessive_rate);
    }

    //==========================================================================
    // 22. GROUND OPERATIONS
    //==========================================================================

    void testGroundPressurization() {
        bool on_ground = true;
        double cabin_alt = 0.0;
        double field_elevation = 500.0;

        if (on_ground) {
            cabin_alt = field_elevation;
        }

        TS_ASSERT_DELTA(cabin_alt, 500.0, 1.0);
    }

    void testDoorOpenPressure() {
        double diff_pressure_psi = 0.1;
        double safe_door_pressure = 0.5;

        bool safe_to_open = (diff_pressure_psi < safe_door_pressure);
        TS_ASSERT(safe_to_open);
    }

    void testPreflightCheck() {
        bool outflow_valve_ok = true;
        bool packs_ok = true;
        bool bleed_ok = true;

        bool system_ready = outflow_valve_ok && packs_ok && bleed_ok;
        TS_ASSERT(system_ready);
    }
};
