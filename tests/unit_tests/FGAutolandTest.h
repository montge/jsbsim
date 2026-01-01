/*******************************************************************************
 * FGAutolandTest.h - Unit tests for autoland system physics
 *
 * This file contains comprehensive tests for autoland system calculations
 * including ILS guidance, glideslope/localizer tracking, flare dynamics,
 * and landing performance.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <algorithm>

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <models/FGGroundReactions.h>
#include <initialization/FGInitialCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGAutolandTest : public CxxTest::TestSuite
{
public:
  // ============ Glideslope Tests ============

  void testGlideslopeAngleCalculation() {
    // GIVEN: Standard 3-degree glideslope
    double glideslope_deg = 3.0;
    double glideslope_rad = glideslope_deg * Constants::DEG_TO_RAD;

    // WHEN: Aircraft is at various distances from threshold
    double distance_ft = 10000.0;  // 10,000 ft from threshold

    // THEN: Expected altitude should follow glideslope
    double expected_alt_ft = distance_ft * std::tan(glideslope_rad);

    TS_ASSERT_DELTA(expected_alt_ft, 524.08, 0.1);
  }

  void testGlideslopeDeviationAtVariousAltitudes() {
    // GIVEN: 3-degree glideslope
    double gs_angle = 3.0 * Constants::DEG_TO_RAD;

    for (double dist_nm = 1.0; dist_nm <= 10.0; dist_nm += 1.0) {
      double dist_ft = dist_nm * 6076.12;  // nautical miles to feet
      double target_alt = dist_ft * std::tan(gs_angle);

      // WHEN: Aircraft is 50 ft high
      double actual_alt = target_alt + 50.0;

      // THEN: Deviation should be calculable
      double deviation = actual_alt - target_alt;
      TS_ASSERT_DELTA(deviation, 50.0, DEFAULT_TOLERANCE);
    }
  }

  void testGlideslopeDeviationInDots() {
    // GIVEN: Standard ILS glideslope (1 dot = 0.35 degrees)
    double dot_sensitivity = 0.35 * Constants::DEG_TO_RAD;  // radians per dot
    double distance_ft = 5000.0;
    double gs_angle = 3.0 * Constants::DEG_TO_RAD;

    // WHEN: Aircraft is 100 ft high
    double target_alt = distance_ft * std::tan(gs_angle);
    double actual_alt = target_alt + 100.0;

    // THEN: Calculate deviation in dots
    double alt_deviation = actual_alt - target_alt;
    double angular_dev = std::atan(alt_deviation / distance_ft);
    double dots = angular_dev / dot_sensitivity;

    TS_ASSERT_DELTA(dots, 3.27, 0.01);  // Approximately 3.27 dots high
  }

  void testGlideslopeInterceptCalculation() {
    // GIVEN: Aircraft approaching glideslope from below
    double gs_angle = 3.0 * Constants::DEG_TO_RAD;
    double aircraft_alt = 2000.0;  // ft

    // WHEN: Finding intercept point
    double intercept_distance = aircraft_alt / std::tan(gs_angle);

    // THEN: Should be at correct distance
    TS_ASSERT_DELTA(intercept_distance, 38162.27, 1.0);
  }

  void testGlideslopeCaptureFromAbove() {
    // GIVEN: Aircraft descending to intercept glideslope
    double gs_angle = 3.0 * Constants::DEG_TO_RAD;
    double distance_ft = 15000.0;
    double target_alt = distance_ft * std::tan(gs_angle);
    double actual_alt = target_alt + 200.0;  // 200 ft above

    // WHEN: Calculating required descent rate
    double groundspeed_kts = 140.0;
    double groundspeed_fps = groundspeed_kts * Constants::KTS_TO_FTPS;
    double vertical_speed_to_gs = groundspeed_fps * std::tan(gs_angle);

    // THEN: Should match expected descent rate
    TS_ASSERT_DELTA(vertical_speed_to_gs, 12.38, 0.1);  // fps (absolute value)
  }

  // ============ Localizer Tests ============

  void testLocalizerDeviationCalculation() {
    // GIVEN: Runway centerline and aircraft position
    double runway_heading = 90.0 * Constants::DEG_TO_RAD;  // Due east
    double aircraft_heading = 92.0 * Constants::DEG_TO_RAD;  // 2 degrees right

    // WHEN: Calculating lateral deviation
    double heading_error = aircraft_heading - runway_heading;

    // THEN: Should show 2 degree deviation
    TS_ASSERT_DELTA(heading_error * Constants::RAD_TO_DEG, 2.0, DEFAULT_TOLERANCE);
  }

  void testLocalizerDeviationInDots() {
    // GIVEN: Standard localizer (1 dot = 2.5 degrees)
    double dot_sensitivity = 2.5 * Constants::DEG_TO_RAD;
    double lateral_offset_ft = 500.0;  // 500 ft left of centerline
    double distance_ft = 10000.0;

    // WHEN: Calculating angular deviation
    double angular_dev = std::atan(lateral_offset_ft / distance_ft);
    double dots = angular_dev / dot_sensitivity;

    // THEN: Should be within scale
    TS_ASSERT_DELTA(dots, 1.15, 0.01);
  }

  void testLocalizerCaptureAngle() {
    // GIVEN: Aircraft offset from centerline
    double lateral_offset_ft = 1000.0;
    double distance_to_runway_ft = 20000.0;

    // WHEN: Calculating intercept angle (30-degree rule)
    double intercept_angle = std::atan(lateral_offset_ft / distance_to_runway_ft);
    double intercept_deg = intercept_angle * Constants::RAD_TO_DEG;

    // THEN: Should be small angle
    TS_ASSERT_DELTA(intercept_deg, 2.86, 0.01);
  }

  void testLocalizerCourseWidth() {
    // GIVEN: Standard localizer beam width (typically 5 degrees total, 2.5 degrees per dot)
    double full_scale_deviation_deg = 2.5;
    double distance_nm = 5.0;
    double distance_ft = distance_nm * 6076.12;

    // WHEN: Calculating full-scale width at distance
    double half_width_ft = distance_ft * std::tan(full_scale_deviation_deg * Constants::DEG_TO_RAD);
    double full_width_ft = 2.0 * half_width_ft;

    // THEN: Width should increase with distance
    TS_ASSERT_DELTA(full_width_ft, 2652.89, 5.0);
  }

  // ============ Decision Height Tests ============

  void testCategoryIDecisionHeight() {
    // GIVEN: CAT I approach minimums
    double cat_I_DH_ft = 200.0;
    double current_alt_ft = 250.0;

    // WHEN: Checking if above DH
    bool above_DH = current_alt_ft > cat_I_DH_ft;

    // THEN: Should be above minimums
    TS_ASSERT(above_DH);
  }

  void testCategoryIIDecisionHeight() {
    // GIVEN: CAT II approach minimums
    double cat_II_DH_ft = 100.0;
    double current_alt_ft = 150.0;

    // WHEN: Checking if above DH
    bool above_DH = current_alt_ft > cat_II_DH_ft;

    // THEN: Should be above minimums
    TS_ASSERT(above_DH);
  }

  void testCategoryIIIDecisionHeight() {
    // GIVEN: CAT IIIa minimums (typically 50 ft or no DH)
    double cat_IIIa_DH_ft = 50.0;
    double current_alt_ft = 75.0;

    // WHEN: Checking if above DH
    bool above_DH = current_alt_ft > cat_IIIa_DH_ft;

    // THEN: Should be above minimums
    TS_ASSERT(above_DH);
  }

  void testDecisionHeightWithBaroError() {
    // GIVEN: Indicated altitude with barometric error
    double indicated_alt_ft = 210.0;
    double baro_error_ft = -15.0;  // 15 ft low
    double cat_I_DH = 200.0;

    // WHEN: Correcting for baro error
    double true_alt = indicated_alt_ft + baro_error_ft;
    bool above_DH = true_alt > cat_I_DH;

    // THEN: Should be below minimums with correction
    TS_ASSERT(!above_DH);
  }

  // ============ Flare Initiation Tests ============

  void testFlareInitiationHeight() {
    // GIVEN: Typical flare initiation at 50 ft radio altitude
    double flare_height_ft = 50.0;
    double current_radio_alt_ft = 55.0;

    // WHEN: Checking if flare should initiate
    bool initiate_flare = current_radio_alt_ft <= flare_height_ft;

    // THEN: Should not yet initiate
    TS_ASSERT(!initiate_flare);

    current_radio_alt_ft = 45.0;
    initiate_flare = current_radio_alt_ft <= flare_height_ft;
    TS_ASSERT(initiate_flare);
  }

  void testFlareCommandedPitchRate() {
    // GIVEN: Flare maneuver parameters
    double initial_pitch_deg = -3.0;  // 3 degrees nose down on glideslope
    double target_pitch_deg = 2.0;    // 2 degrees nose up at touchdown
    double flare_duration_sec = 6.0;

    // WHEN: Calculating pitch rate during flare
    double pitch_change_deg = target_pitch_deg - initial_pitch_deg;
    double pitch_rate_deg_per_sec = pitch_change_deg / flare_duration_sec;

    // THEN: Should be smooth pitch rate
    TS_ASSERT_DELTA(pitch_rate_deg_per_sec, 0.833, 0.01);
  }

  void testFlareDescentRateReduction() {
    // GIVEN: Initial descent rate on glideslope
    double initial_vsi_fpm = -700.0;  // 700 fpm descent
    double flare_start_alt_ft = 50.0;
    double touchdown_vsi_fpm = -100.0;  // Target 100 fpm at touchdown

    // WHEN: Calculating exponential flare law
    double altitude_ft = 30.0;  // Current altitude in flare
    double flare_constant = 2.0;  // Exponential decay constant

    double vsi_fpm = touchdown_vsi_fpm + (initial_vsi_fpm - touchdown_vsi_fpm) *
                     std::exp(-flare_constant * (flare_start_alt_ft - altitude_ft) / flare_start_alt_ft);

    // THEN: Descent rate should be decreasing (more negative than initial, less negative than touchdown)
    TS_ASSERT(vsi_fpm > initial_vsi_fpm);  // Less negative (closer to zero)
    TS_ASSERT(vsi_fpm < touchdown_vsi_fpm);  // More negative than target touchdown
  }

  void testFlarePathCalculation() {
    // GIVEN: Exponential flare from 50 ft
    double h0 = 50.0;  // Initial height
    double time_constant = 3.0;  // seconds

    for (double t = 0.0; t <= 6.0; t += 1.0) {
      // WHEN: Computing height during flare
      double h = h0 * std::exp(-t / time_constant);

      // THEN: Height should decrease exponentially
      TS_ASSERT(h <= h0);
      TS_ASSERT(h >= 0.0);
    }
  }

  // ============ Touchdown Tests ============

  void testTouchdownDispersionLongitudinal() {
    // GIVEN: Target touchdown zone 1000 ft from threshold
    double target_touchdown_ft = 1000.0;
    double actual_touchdown_ft = 1050.0;

    // WHEN: Calculating longitudinal dispersion
    double dispersion_ft = actual_touchdown_ft - target_touchdown_ft;

    // THEN: Should be within acceptable limits (typically ±500 ft)
    TS_ASSERT(std::abs(dispersion_ft) < 500.0);
  }

  void testTouchdownDispersionLateral() {
    // GIVEN: Runway centerline
    double lateral_offset_ft = 5.0;  // 5 ft from centerline

    // WHEN: Checking lateral dispersion
    // THEN: Should be within acceptable limits (typically ±30 ft)
    TS_ASSERT(std::abs(lateral_offset_ft) < 30.0);
  }

  void testTouchdownSinkRate() {
    // GIVEN: Touchdown sink rate
    double sink_rate_fpm = 120.0;

    // WHEN: Checking if within limits
    // THEN: Should be less than 600 fpm for normal landing
    TS_ASSERT(sink_rate_fpm < 600.0);
    // And less than 200 fpm for smooth autoland
    TS_ASSERT(sink_rate_fpm < 200.0);
  }

  void testTouchdownVerticalAcceleration() {
    // GIVEN: Sink rate and impact
    double sink_rate_fps = 2.0;  // 2 ft/s = 120 fpm
    double spring_constant = 5000.0;  // lb/ft
    double aircraft_mass_slug = 1000.0;

    // WHEN: Computing impact acceleration
    double compression_ft = 0.5;  // 6 inches
    double force_lb = spring_constant * compression_ft;
    double accel_fps2 = force_lb / aircraft_mass_slug;
    double accel_g = accel_fps2 / Constants::G_FTPS2;

    // THEN: Should be within structural limits
    TS_ASSERT(accel_g < 2.0);  // Less than 2g
  }

  // ============ ILS Deviation Tests ============

  void testILSGlideslopeFullScaleDeviation() {
    // GIVEN: Full scale glideslope deviation (typically 0.7 degrees)
    double full_scale_deg = 0.7;
    double distance_ft = 5280.0;  // 1 nautical mile

    // WHEN: Computing vertical displacement for full scale
    double vertical_displacement_ft = distance_ft * std::tan(full_scale_deg * Constants::DEG_TO_RAD);

    // THEN: Should match expected value
    TS_ASSERT_DELTA(vertical_displacement_ft, 64.5, 0.5);
  }

  void testILSLocalizerFullScaleDeviation() {
    // GIVEN: Full scale localizer deviation (typically 2.5 degrees)
    double full_scale_deg = 2.5;
    double distance_ft = 5280.0;  // 1 nautical mile

    // WHEN: Computing lateral displacement for full scale
    double lateral_displacement_ft = distance_ft * std::tan(full_scale_deg * Constants::DEG_TO_RAD);

    // THEN: Should match expected value
    TS_ASSERT_DELTA(lateral_displacement_ft, 230.5, 0.5);
  }

  void testILSCombinedDeviation() {
    // GIVEN: Both lateral and vertical deviations
    double lateral_dots = 1.0;
    double vertical_dots = 0.5;

    // WHEN: Computing total deviation magnitude
    double total_dots = std::sqrt(lateral_dots * lateral_dots + vertical_dots * vertical_dots);

    // THEN: Should be vector sum
    TS_ASSERT_DELTA(total_dots, 1.118, 0.001);
  }

  // ============ Radio Altimeter Tests ============

  void testRadioAltimeterAccuracy() {
    // GIVEN: Radio altimeter reading
    double radio_alt_ft = 100.0;
    double terrain_slope_deg = 2.0;

    // WHEN: Accounting for terrain slope
    double effective_altitude = radio_alt_ft / std::cos(terrain_slope_deg * Constants::DEG_TO_RAD);

    // THEN: Should show slight increase
    TS_ASSERT(effective_altitude > radio_alt_ft);
  }

  void testRadioAltimeterVsBarometric() {
    // GIVEN: Radio and barometric altitude
    double radio_alt_ft = 50.0;
    double baro_alt_ft = 225.0;
    double field_elevation_ft = 175.0;

    // WHEN: Computing height above ground from baro
    double baro_agl = baro_alt_ft - field_elevation_ft;

    // THEN: Should match radio altitude
    TS_ASSERT_DELTA(baro_agl, radio_alt_ft, DEFAULT_TOLERANCE);
  }

  void testRadioAltimeterResponseTime() {
    // GIVEN: Rapid descent
    double descent_rate_fps = 10.0;  // 600 fpm
    double response_time_sec = 0.5;  // Typical RA response time

    // WHEN: Computing altitude error due to lag
    double lag_error_ft = descent_rate_fps * response_time_sec;

    // THEN: Should be small error
    TS_ASSERT_DELTA(lag_error_ft, 5.0, DEFAULT_TOLERANCE);
  }

  // ============ Approach Speed Tests ============

  void testVrefCalculation() {
    // GIVEN: Aircraft parameters
    double stall_speed_kts = 100.0;
    double vref_factor = 1.3;

    // WHEN: Computing reference speed
    double vref_kts = stall_speed_kts * vref_factor;

    // THEN: Should be 130% of stall
    TS_ASSERT_DELTA(vref_kts, 130.0, DEFAULT_TOLERANCE);
  }

  void testApproachSpeedWithWind() {
    // GIVEN: Vref and wind
    double vref_kts = 130.0;
    double headwind_kts = 20.0;
    double gust_kts = 10.0;

    // WHEN: Adding half the gust to approach speed
    double vapp_kts = vref_kts + 0.5 * gust_kts;

    // THEN: Should be increased
    TS_ASSERT_DELTA(vapp_kts, 135.0, DEFAULT_TOLERANCE);
  }

  void testApproachSpeedDecay() {
    // GIVEN: Initial approach speed
    double vapp_kts = 140.0;
    double vref_kts = 130.0;
    double altitude_ft = 500.0;

    // WHEN: Computing speed reduction schedule (1 kt per 100 ft)
    double speed_reduction_kts = (1000.0 - altitude_ft) / 100.0;
    double target_speed_kts = std::max(vref_kts, vapp_kts - speed_reduction_kts);

    // THEN: Speed should decrease with altitude
    TS_ASSERT_DELTA(target_speed_kts, 135.0, DEFAULT_TOLERANCE);
  }

  void testGroundspeedVsAirspeed() {
    // GIVEN: Airspeed and wind
    double airspeed_kts = 130.0;
    double headwind_kts = 25.0;

    // WHEN: Computing groundspeed
    double groundspeed_kts = airspeed_kts - headwind_kts;

    // THEN: Should be reduced
    TS_ASSERT_DELTA(groundspeed_kts, 105.0, DEFAULT_TOLERANCE);
  }

  // ============ Sink Rate Control Tests ============

  void testGlideslopeSinkRate() {
    // GIVEN: On glideslope at approach speed
    double groundspeed_kts = 140.0;
    double gs_angle_deg = 3.0;

    // WHEN: Computing required sink rate
    double groundspeed_fpm = groundspeed_kts * 101.27;  // kts to fpm
    double sink_rate_fpm = groundspeed_fpm * std::tan(gs_angle_deg * Constants::DEG_TO_RAD);

    // THEN: Should be approximately 700-750 fpm
    TS_ASSERT_DELTA(sink_rate_fpm, 742.0, 5.0);
  }

  void testSinkRateDuringFlare() {
    // GIVEN: Flare parameters
    double initial_sink_rate_fpm = 700.0;
    double altitude_ft = 30.0;
    double flare_height_ft = 50.0;

    // WHEN: Computing sink rate reduction (linear approximation)
    double reduction_factor = altitude_ft / flare_height_ft;
    double current_sink_rate_fpm = initial_sink_rate_fpm * reduction_factor;

    // THEN: Should be reduced
    TS_ASSERT_DELTA(current_sink_rate_fpm, 420.0, DEFAULT_TOLERANCE);
  }

  void testSinkRateLimit() {
    // GIVEN: Commanded sink rate
    double commanded_vsi_fpm = -1200.0;
    double max_sink_rate_fpm = -1000.0;

    // WHEN: Applying limit
    double limited_vsi_fpm = std::max(commanded_vsi_fpm, max_sink_rate_fpm);

    // THEN: Should be limited
    TS_ASSERT_DELTA(limited_vsi_fpm, -1000.0, DEFAULT_TOLERANCE);
  }

  // ============ Rollout and Braking Tests ============

  void testTouchdownGroundRoll() {
    // GIVEN: Touchdown conditions
    double touchdown_speed_kts = 130.0;
    double touchdown_speed_fps = touchdown_speed_kts * Constants::KTS_TO_FTPS;
    double deceleration_fps2 = 8.0;  // With brakes and reversers

    // WHEN: Computing stopping distance
    double stopping_distance_ft = (touchdown_speed_fps * touchdown_speed_fps) / (2.0 * deceleration_fps2);

    // THEN: Should be reasonable
    TS_ASSERT_DELTA(stopping_distance_ft, 3009.0, 50.0);
  }

  void testBrakingDeceleration() {
    // GIVEN: Braking parameters
    double friction_coef = 0.4;  // Wet runway
    double brake_efficiency = 0.8;

    // WHEN: Computing deceleration
    double max_decel_fps2 = Constants::G_FTPS2 * friction_coef * brake_efficiency;

    // THEN: Should be reasonable value
    TS_ASSERT_DELTA(max_decel_fps2, 10.3, 0.1);
  }

  void testReverserEffectiveness() {
    // GIVEN: Thrust reverser deployment
    double forward_thrust_lb = 20000.0;
    double reverser_efficiency = 0.5;
    double aircraft_mass_slug = 3000.0;

    // WHEN: Computing deceleration from reversers
    double reverse_thrust_lb = forward_thrust_lb * reverser_efficiency;
    double decel_fps2 = reverse_thrust_lb / aircraft_mass_slug;

    // THEN: Should contribute to stopping
    TS_ASSERT_DELTA(decel_fps2, 3.33, 0.01);
  }

  void testRolloutDirectionalControl() {
    // GIVEN: Crosswind during rollout
    double crosswind_kts = 15.0;
    double groundspeed_kts = 80.0;

    // WHEN: Computing drift angle
    double drift_angle_rad = std::atan(crosswind_kts / groundspeed_kts);
    double drift_angle_deg = drift_angle_rad * Constants::RAD_TO_DEG;

    // THEN: Should be small angle
    TS_ASSERT_DELTA(drift_angle_deg, 10.62, 0.01);
  }

  // ============ Go-Around Decision Tests ============

  void testGoAroundFromDecisionHeight() {
    // GIVEN: At decision height with poor conditions
    double current_alt_ft = 200.0;
    double decision_height_ft = 200.0;
    bool runway_in_sight = false;

    // WHEN: Making go-around decision
    bool execute_go_around = (current_alt_ft <= decision_height_ft) && !runway_in_sight;

    // THEN: Should execute go-around
    TS_ASSERT(execute_go_around);
  }

  void testGoAroundClimbGradient() {
    // GIVEN: Go-around thrust and configuration
    double thrust_lb = 30000.0;
    double drag_lb = 8000.0;
    double weight_lb = 150000.0;

    // WHEN: Computing climb gradient
    double excess_thrust_lb = thrust_lb - drag_lb;
    double climb_gradient = excess_thrust_lb / weight_lb;

    // THEN: Should meet minimum 2.5% for CAT I
    TS_ASSERT(climb_gradient > 0.025);
  }

  void testGoAroundPitchTarget() {
    // GIVEN: Go-around initiation
    double current_pitch_deg = 2.0;  // Touchdown attitude
    double target_pitch_deg = 15.0;  // Initial climb attitude
    double pitch_rate_deg_per_sec = 3.0;

    // WHEN: Computing time to reach target
    double time_to_target_sec = (target_pitch_deg - current_pitch_deg) / pitch_rate_deg_per_sec;

    // THEN: Should be reasonable transition time
    TS_ASSERT_DELTA(time_to_target_sec, 4.33, 0.01);
  }

  // ============ RVR Requirements Tests ============

  void testCategoryIRVR() {
    // GIVEN: CAT I RVR requirement
    double required_rvr_ft = 1800.0;  // Typical CAT I minimum
    double actual_rvr_ft = 2400.0;

    // WHEN: Checking if minimums met
    bool minimums_met = actual_rvr_ft >= required_rvr_ft;

    // THEN: Should meet requirements
    TS_ASSERT(minimums_met);
  }

  void testCategoryIIRVR() {
    // GIVEN: CAT II RVR requirement
    double required_rvr_ft = 1200.0;
    double actual_rvr_ft = 1500.0;

    // WHEN: Checking if minimums met
    bool minimums_met = actual_rvr_ft >= required_rvr_ft;

    // THEN: Should meet requirements
    TS_ASSERT(minimums_met);
  }

  void testCategoryIIIRVR() {
    // GIVEN: CAT IIIa RVR requirement
    double required_rvr_ft = 700.0;
    double actual_rvr_ft = 800.0;

    // WHEN: Checking if minimums met
    bool minimums_met = actual_rvr_ft >= required_rvr_ft;

    // THEN: Should meet requirements
    TS_ASSERT(minimums_met);
  }

  // ============ Crosswind Limits Tests ============

  void testCrosswindComponentCalculation() {
    // GIVEN: Wind and runway heading
    double wind_speed_kts = 25.0;
    double wind_direction_deg = 100.0;
    double runway_heading_deg = 90.0;

    // WHEN: Computing crosswind component
    double wind_angle_deg = wind_direction_deg - runway_heading_deg;
    double crosswind_kts = wind_speed_kts * std::sin(wind_angle_deg * Constants::DEG_TO_RAD);

    // THEN: Should be reasonable
    TS_ASSERT_DELTA(crosswind_kts, 4.34, 0.1);
  }

  void testAutolandCrosswindLimit() {
    // GIVEN: Crosswind limit for autoland (typically 15 kts)
    double max_crosswind_kts = 15.0;
    double actual_crosswind_kts = 12.0;

    // WHEN: Checking if within limits
    bool autoland_approved = std::abs(actual_crosswind_kts) <= max_crosswind_kts;

    // THEN: Should be approved
    TS_ASSERT(autoland_approved);
  }

  void testHeadwindComponent() {
    // GIVEN: Wind and runway
    double wind_speed_kts = 30.0;
    double wind_direction_deg = 85.0;
    double runway_heading_deg = 90.0;

    // WHEN: Computing headwind component
    double wind_angle_deg = wind_direction_deg - runway_heading_deg;
    double headwind_kts = wind_speed_kts * std::cos(wind_angle_deg * Constants::DEG_TO_RAD);

    // THEN: Should be positive for headwind
    TS_ASSERT(headwind_kts > 0.0);
    TS_ASSERT_DELTA(headwind_kts, 29.89, 0.01);
  }

  // ============ Touchdown Zone Tests ============

  void testTouchdownZoneTarget() {
    // GIVEN: Standard touchdown zone (1000 ft from threshold)
    double threshold_distance_ft = 0.0;
    double touchdown_zone_start_ft = 500.0;
    double touchdown_zone_end_ft = 1500.0;
    double actual_touchdown_ft = 1000.0;

    // WHEN: Checking if in touchdown zone
    bool in_tdz = (actual_touchdown_ft >= touchdown_zone_start_ft) &&
                  (actual_touchdown_ft <= touchdown_zone_end_ft);

    // THEN: Should be in zone
    TS_ASSERT(in_tdz);
  }

  void testTouchdownAimPoint() {
    // GIVEN: Aiming for touchdown zone markers
    double aim_point_distance_ft = 1000.0;  // From threshold
    double gs_angle_rad = 3.0 * Constants::DEG_TO_RAD;

    // WHEN: Computing altitude at aim point
    double altitude_at_aim_ft = aim_point_distance_ft * std::tan(gs_angle_rad);

    // THEN: Should be about 52 ft
    TS_ASSERT_DELTA(altitude_at_aim_ft, 52.41, 0.1);
  }

  // ============ Thrust Management Tests ============

  void testIdleThrustDuringFlare() {
    // GIVEN: Thrust during flare
    double current_thrust_percent = 40.0;
    double flare_thrust_reduction_rate = 10.0;  // percent per second
    double dt = 1.0;  // seconds

    // WHEN: Reducing thrust during flare
    double new_thrust_percent = current_thrust_percent - flare_thrust_reduction_rate * dt;
    new_thrust_percent = std::max(0.0, new_thrust_percent);

    // THEN: Should decrease
    TS_ASSERT_DELTA(new_thrust_percent, 30.0, DEFAULT_TOLERANCE);
  }

  void testThrustSymmetry() {
    // GIVEN: Multi-engine aircraft
    double left_thrust_lb = 10000.0;
    double right_thrust_lb = 10050.0;
    double thrust_asymmetry_limit_lb = 500.0;

    // WHEN: Checking thrust symmetry
    double thrust_diff_lb = std::abs(left_thrust_lb - right_thrust_lb);
    bool symmetric = thrust_diff_lb < thrust_asymmetry_limit_lb;

    // THEN: Should be symmetric
    TS_ASSERT(symmetric);
  }

  // ============ Speed Brake Tests ============

  void testSpeedBrakeDeployment() {
    // GIVEN: Touchdown detection
    double radio_alt_ft = 5.0;
    double weight_on_wheels = true;

    // WHEN: Deploying speed brakes
    bool deploy_speedbrakes = weight_on_wheels && (radio_alt_ft < 10.0);

    // THEN: Should deploy
    TS_ASSERT(deploy_speedbrakes);
  }

  void testSpeedBrakeDragEffect() {
    // GIVEN: Speed brake deployment
    double base_drag_lb = 5000.0;
    double speedbrake_drag_lb = 3000.0;

    // WHEN: Computing total drag
    double total_drag_lb = base_drag_lb + speedbrake_drag_lb;

    // THEN: Should increase significantly
    TS_ASSERT_DELTA(total_drag_lb, 8000.0, DEFAULT_TOLERANCE);
  }

  // ============ Spoiler Tests ============

  void testSpoilerAutoDeployment() {
    // GIVEN: Landing conditions
    double groundspeed_kts = 100.0;
    double wheel_speed_kts = 95.0;
    double speed_threshold_kts = 60.0;

    // WHEN: Checking auto-spoiler deployment
    bool weight_on_wheels = (wheel_speed_kts > 10.0);
    bool speed_above_threshold = (groundspeed_kts > speed_threshold_kts);
    bool deploy_spoilers = weight_on_wheels && speed_above_threshold;

    // THEN: Should deploy
    TS_ASSERT(deploy_spoilers);
  }

  void testSpoilerLiftDumping() {
    // GIVEN: Wing lift and spoilers
    double wing_lift_lb = 150000.0;
    double spoiler_effectiveness = 0.6;  // 60% lift reduction

    // WHEN: Deploying spoilers
    double residual_lift_lb = wing_lift_lb * (1.0 - spoiler_effectiveness);

    // THEN: Lift should be greatly reduced
    TS_ASSERT_DELTA(residual_lift_lb, 60000.0, DEFAULT_TOLERANCE);
  }

  // ============ Nose Wheel Steering Tests ============

  void testNoseWheelSteeringEngagement() {
    // GIVEN: Rollout conditions
    double groundspeed_kts = 50.0;
    double nws_engagement_speed_kts = 100.0;

    // WHEN: Checking NWS engagement
    bool nws_engaged = groundspeed_kts < nws_engagement_speed_kts;

    // THEN: Should be engaged
    TS_ASSERT(nws_engaged);
  }

  void testNoseWheelSteeringAngle() {
    // GIVEN: Lateral deviation from centerline
    double lateral_offset_ft = 10.0;
    double lookahead_distance_ft = 200.0;

    // WHEN: Computing steering angle
    double steering_angle_rad = std::atan(lateral_offset_ft / lookahead_distance_ft);
    double steering_angle_deg = steering_angle_rad * Constants::RAD_TO_DEG;

    // THEN: Should be small correction
    TS_ASSERT_DELTA(steering_angle_deg, 2.86, 0.01);
  }

  // ============ CAT III Rollout Guidance Tests ============

  void testCATIIIRolloutGuidance() {
    // GIVEN: CAT III rollout with zero visibility
    double rvr_ft = 200.0;  // Very low visibility
    double cat_III_rvr_minimum_ft = 0.0;  // CAT IIIc has no minimum

    // WHEN: Checking if rollout guidance required
    bool rollout_guidance_required = rvr_ft < 600.0;

    // THEN: Should require rollout guidance
    TS_ASSERT(rollout_guidance_required);
  }

  void testRolloutGuidanceCenterlineTracking() {
    // GIVEN: Centerline deviation during rollout
    double lateral_deviation_ft = 3.0;
    double centerline_tolerance_ft = 15.0;

    // WHEN: Checking tracking performance
    bool within_tolerance = std::abs(lateral_deviation_ft) < centerline_tolerance_ft;

    // THEN: Should be tracking well
    TS_ASSERT(within_tolerance);
  }

  void testRolloutDecelerationProfile() {
    // GIVEN: Initial and final speeds
    double initial_speed_kts = 130.0;
    double final_speed_kts = 10.0;
    double deceleration_fps2 = 8.0;

    // WHEN: Computing deceleration time
    double speed_change_fps = (initial_speed_kts - final_speed_kts) * Constants::KTS_TO_FTPS;
    double decel_time_sec = speed_change_fps / deceleration_fps2;

    // THEN: Should be reasonable time
    TS_ASSERT(decel_time_sec > 0.0);
    TS_ASSERT(decel_time_sec < 60.0);  // Should stop within a minute
  }

  // ============ GBAS/GLS Approach Tests ============

  void testGBASVerticalError() {
    // GIVEN: GBAS approach with GPS-based guidance
    double gbas_vertical_error_ft = 0.5;  // Typical GBAS accuracy
    double ils_vertical_error_ft = 5.0;   // Typical ILS accuracy

    // WHEN: Comparing accuracy
    // THEN: GBAS should be more accurate
    TS_ASSERT(gbas_vertical_error_ft < ils_vertical_error_ft);
  }

  void testGBASLateralError() {
    // GIVEN: GBAS lateral accuracy
    double gbas_lateral_error_ft = 1.0;
    double tolerance_ft = 5.0;

    // WHEN: Checking lateral performance
    // THEN: Should be within tolerance
    TS_ASSERT(gbas_lateral_error_ft < tolerance_ft);
  }

  void testGBASGlideslopeAngle() {
    // GIVEN: GBAS can use steeper glideslopes
    double standard_gs_deg = 3.0;
    double steep_gs_deg = 4.5;  // GBAS allows steeper approaches

    // WHEN: Calculating altitude difference
    double distance_ft = 5000.0;
    double alt_standard = distance_ft * std::tan(standard_gs_deg * Constants::DEG_TO_RAD);
    double alt_steep = distance_ft * std::tan(steep_gs_deg * Constants::DEG_TO_RAD);

    // THEN: Steep approach requires higher altitude at same distance
    TS_ASSERT(alt_steep > alt_standard);
  }

  // ============ Autothrottle Tests ============

  void testAutothrottleSpeedHold() {
    // GIVEN: Target and actual airspeed
    double target_speed_kts = 140.0;
    double actual_speed_kts = 138.0;
    double kp = 0.1;  // Proportional gain

    // WHEN: Computing throttle adjustment
    double speed_error = target_speed_kts - actual_speed_kts;
    double throttle_adjustment = kp * speed_error;

    // THEN: Should command increase
    TS_ASSERT(throttle_adjustment > 0.0);
    TS_ASSERT_DELTA(throttle_adjustment, 0.2, 0.01);
  }

  void testAutothrottleDescentMode() {
    // GIVEN: Glideslope descent
    double thrust_lever_position = 0.35;  // 35%
    double idle_thrust = 0.10;

    // WHEN: On approach
    // THEN: Thrust should be above idle but low
    TS_ASSERT(thrust_lever_position > idle_thrust);
    TS_ASSERT(thrust_lever_position < 0.50);
  }

  void testAutothrottleRetard() {
    // GIVEN: Retard height (typically 30-50 ft)
    double retard_height_ft = 40.0;
    double current_radio_alt_ft = 35.0;

    // WHEN: Checking if retard should commence
    bool retard_commanded = current_radio_alt_ft <= retard_height_ft;

    // THEN: Retard should be commanded
    TS_ASSERT(retard_commanded);
  }

  // ============ Pitch Trim Tests ============

  void testApproachTrimSetting() {
    // GIVEN: Approach configuration
    double cg_percent = 25.0;  // 25% MAC
    double approach_speed_kts = 140.0;

    // WHEN: Computing trim setting (simplified)
    double trim_units = cg_percent * 0.1 + approach_speed_kts * 0.01;

    // THEN: Should be reasonable trim setting
    TS_ASSERT(trim_units > 0.0);
  }

  void testTrimChangeInFlare() {
    // GIVEN: Pitch change during flare
    double flare_pitch_change_deg = 5.0;
    double trim_rate_per_deg = 0.5;

    // WHEN: Computing trim change
    double trim_change = flare_pitch_change_deg * trim_rate_per_deg;

    // THEN: Should need trim adjustment
    TS_ASSERT_DELTA(trim_change, 2.5, 0.1);
  }

  // ============ Configuration Monitoring Tests ============

  void testLandingGearDownVerification() {
    // GIVEN: Landing gear position
    double gear_position = 1.0;  // 1.0 = fully down
    double gear_down_threshold = 0.95;

    // WHEN: Checking gear down
    bool gear_down_locked = gear_position >= gear_down_threshold;

    // THEN: Should be down and locked
    TS_ASSERT(gear_down_locked);
  }

  void testFlapPositionForLanding() {
    // GIVEN: Flap setting
    double flap_position_deg = 40.0;
    double landing_flap_min_deg = 30.0;
    double landing_flap_max_deg = 45.0;

    // WHEN: Checking flap position
    bool flaps_in_range = (flap_position_deg >= landing_flap_min_deg) &&
                          (flap_position_deg <= landing_flap_max_deg);

    // THEN: Should be in landing range
    TS_ASSERT(flaps_in_range);
  }

  void testSpeedbrakeArmed() {
    // GIVEN: Speedbrake position
    bool speedbrake_armed = true;
    double altitude_ft = 1000.0;

    // WHEN: Below arming altitude
    bool ready_to_deploy = speedbrake_armed && (altitude_ft < 2000.0);

    // THEN: Should be ready
    TS_ASSERT(ready_to_deploy);
  }

  // ============ HUD Guidance Tests ============

  void testFlightPathVectorCalculation() {
    // GIVEN: Aircraft state
    double pitch_deg = -3.0;
    double aoa_deg = 5.0;

    // WHEN: Computing flight path angle
    double gamma_deg = pitch_deg - aoa_deg;

    // THEN: Should be negative (descending)
    TS_ASSERT_DELTA(gamma_deg, -8.0, 0.1);
  }

  void testHUDGlideslopeDeviation() {
    // GIVEN: Glideslope deviation
    double deviation_dots = 0.5;
    double max_display_dots = 2.5;

    // WHEN: Scaling for display
    double display_ratio = deviation_dots / max_display_dots;

    // THEN: Should be reasonable ratio
    TS_ASSERT(display_ratio < 1.0);
    TS_ASSERT_DELTA(display_ratio, 0.2, 0.01);
  }

  // ============ EGPWS Integration Tests ============

  void testGlideslopeAlert() {
    // GIVEN: Below glideslope condition
    double deviation_dots = -1.5;
    double alert_threshold_dots = -1.0;

    // WHEN: Checking for alert
    bool glideslope_alert = deviation_dots < alert_threshold_dots;

    // THEN: Should trigger alert
    TS_ASSERT(glideslope_alert);
  }

  void testTOOLOWFLAPSWarning() {
    // GIVEN: Low altitude with incorrect configuration
    double altitude_ft = 500.0;
    double flap_position_deg = 5.0;
    double min_flap_for_altitude_deg = 20.0;

    // WHEN: Checking configuration
    bool flap_warning = (altitude_ft < 1000.0) && (flap_position_deg < min_flap_for_altitude_deg);

    // THEN: Should trigger warning
    TS_ASSERT(flap_warning);
  }

  // ============ Runway Occupancy Tests ============

  void testRunwayOccupancyTime() {
    // GIVEN: Landing roll parameters
    double touchdown_speed_kts = 130.0;
    double exit_speed_kts = 20.0;
    double decel_rate_fps2 = 8.0;

    // WHEN: Computing occupancy time
    double speed_change_fps = (touchdown_speed_kts - exit_speed_kts) * Constants::KTS_TO_FTPS;
    double occupancy_time_sec = speed_change_fps / decel_rate_fps2;

    // THEN: Should be reasonable time
    TS_ASSERT(occupancy_time_sec > 20.0);
    TS_ASSERT(occupancy_time_sec < 60.0);
  }

  void testTaxiClearanceDistance() {
    // GIVEN: High-speed exit parameters
    double exit_speed_kts = 50.0;
    double exit_speed_fps = exit_speed_kts * Constants::KTS_TO_FTPS;
    double turn_radius_ft = 500.0;

    // WHEN: Computing required visibility for exit
    double visibility_required_ft = exit_speed_fps * 5.0;  // 5 second lookahead

    // THEN: Should be reasonable distance
    TS_ASSERT(visibility_required_ft > 300.0);
  }

  // ============ Wind Shear Detection Tests ============

  void testWindshearDetection() {
    // GIVEN: Airspeed change rate
    double airspeed_change_rate_kts_per_sec = -5.0;  // Losing 5 kts/s
    double windshear_threshold = -3.0;

    // WHEN: Checking for windshear
    bool windshear_detected = airspeed_change_rate_kts_per_sec < windshear_threshold;

    // THEN: Should detect windshear
    TS_ASSERT(windshear_detected);
  }

  void testWindshearEscapeProfile() {
    // GIVEN: Windshear escape parameters
    double pitch_target_deg = 15.0;
    double current_pitch_deg = 5.0;
    double max_pitch_rate = 5.0;  // deg/s

    // WHEN: Computing time to target
    double time_to_target = (pitch_target_deg - current_pitch_deg) / max_pitch_rate;

    // THEN: Should be quick response
    TS_ASSERT_DELTA(time_to_target, 2.0, 0.1);
  }

  // ============ Dual/Triple Channel Tests ============

  void testDualChannelComparison() {
    // GIVEN: Two autopilot channel outputs
    double channel_a_command_deg = 2.5;
    double channel_b_command_deg = 2.6;
    double disagreement_limit_deg = 1.0;

    // WHEN: Comparing channels
    double disagreement = std::abs(channel_a_command_deg - channel_b_command_deg);
    bool channels_agree = disagreement < disagreement_limit_deg;

    // THEN: Channels should agree
    TS_ASSERT(channels_agree);
  }

  void testTripleChannelVoting() {
    // GIVEN: Three channel outputs
    double channel_a = 2.5;
    double channel_b = 2.6;
    double channel_c = 2.4;

    // WHEN: Computing median (voting)
    double channels[] = {channel_a, channel_b, channel_c};
    std::sort(channels, channels + 3);
    double voted_output = channels[1];  // Median

    // THEN: Should select middle value
    TS_ASSERT_DELTA(voted_output, 2.5, 0.01);
  }

  // ============ Complete System Tests ============

  void testCompleteAutolandSequence() {
    // Verify complete autoland from 1000 ft to touchdown
    double altitude = 1000.0;
    double glideslope_angle = 3.0 * Constants::DEG_TO_RAD;
    double groundspeed_fps = 200.0;
    double descent_rate_fps = groundspeed_fps * std::tan(glideslope_angle);
    double time_to_touchdown = altitude / descent_rate_fps;

    TS_ASSERT(time_to_touchdown > 90.0 && time_to_touchdown < 110.0);
  }

  void testFlareEnergyManagement() {
    // Test kinetic energy during flare
    double mass_slug = 3000.0;
    double initial_speed_fps = 230.0;
    double final_speed_fps = 220.0;
    double ke_initial = 0.5 * mass_slug * initial_speed_fps * initial_speed_fps;
    double ke_final = 0.5 * mass_slug * final_speed_fps * final_speed_fps;
    double energy_lost = ke_initial - ke_final;

    TS_ASSERT(energy_lost > 0.0);
  }

  void testTouchdownPointAccuracy() {
    // CAT III touchdown accuracy
    double target_ft = 1000.0;
    double tolerance_ft = 200.0;
    double actual_ft = 1050.0;

    bool within_tolerance = std::abs(actual_ft - target_ft) <= tolerance_ft;
    TS_ASSERT(within_tolerance);
  }

  void testAutolandArmedConditions() {
    bool ils_captured = true;
    bool gear_down = true;
    bool flaps_landing = true;
    double altitude_ft = 1500.0;
    bool below_arm_altitude = altitude_ft < 2500.0;

    bool autoland_armed = ils_captured && gear_down && flaps_landing && below_arm_altitude;
    TS_ASSERT(autoland_armed);
  }

  void testGlideslopeCaptureLogic() {
    double gs_deviation_dots = 0.3;
    double capture_threshold = 0.5;
    bool captured = std::abs(gs_deviation_dots) < capture_threshold;
    TS_ASSERT(captured);
  }

  void testLocalizerCaptureLogic() {
    double loc_deviation_dots = 0.2;
    double capture_threshold = 0.5;
    bool captured = std::abs(loc_deviation_dots) < capture_threshold;
    TS_ASSERT(captured);
  }

  void testFlareGainSchedule() {
    // Flare gain increases as altitude decreases
    double alt_50 = 50.0;
    double alt_10 = 10.0;
    double gain_at_50 = 1.0;
    double gain_at_10 = 2.0;

    TS_ASSERT(gain_at_10 > gain_at_50);
  }

  void testRolloutBrakeApplication() {
    double weight_on_wheels = true;
    double ground_speed_kts = 100.0;
    double brake_speed_threshold = 60.0;
    bool auto_brake_active = weight_on_wheels && (ground_speed_kts > brake_speed_threshold);
    TS_ASSERT(auto_brake_active);
  }

  void testAutolandDisengageConditions() {
    bool pilot_takeover = false;
    bool system_fault = false;
    double deviation_dots = 0.8;
    double max_deviation = 1.0;
    bool excessive_deviation = std::abs(deviation_dots) > max_deviation;

    bool should_disengage = pilot_takeover || system_fault || excessive_deviation;
    TS_ASSERT(!should_disengage);
  }

  void testWeatherMinimumCheck() {
    double rvr_ft = 1000.0;
    double cat_ii_minimum = 1200.0;
    double cat_iii_minimum = 700.0;

    bool cat_ii_ok = rvr_ft >= cat_ii_minimum;
    bool cat_iii_ok = rvr_ft >= cat_iii_minimum;

    TS_ASSERT(!cat_ii_ok);
    TS_ASSERT(cat_iii_ok);
  }

  void testAutolandModeAnnunciation() {
    bool approach_mode = true;
    bool capture_mode = true;
    bool land_mode = false;

    int mode_level = approach_mode ? 1 : 0;
    mode_level += capture_mode ? 1 : 0;
    mode_level += land_mode ? 1 : 0;

    TS_ASSERT_EQUALS(mode_level, 2);
  }

  void testTouchdownRateLimit() {
    double sink_rate_fps = 3.0;
    double max_sink_rate_fps = 5.0;
    bool within_limits = sink_rate_fps <= max_sink_rate_fps;
    TS_ASSERT(within_limits);
  }

  void testAutolandRollCommand() {
    double localizer_deviation_dots = 0.5;
    double roll_gain = 10.0;  // deg per dot
    double roll_command = localizer_deviation_dots * roll_gain;
    TS_ASSERT_DELTA(roll_command, 5.0, DEFAULT_TOLERANCE);
  }

  void testAutolandPitchCommand() {
    double glideslope_deviation_dots = -0.3;
    double pitch_gain = 2.0;  // deg per dot
    double pitch_command = -glideslope_deviation_dots * pitch_gain;
    TS_ASSERT_DELTA(pitch_command, 0.6, DEFAULT_TOLERANCE);
  }

  void testApproachLightingCheck() {
    double rvr_ft = 600.0;
    bool approach_lights = true;
    bool touchdown_lights = true;
    bool centerline_lights = true;

    bool lighting_adequate = approach_lights && touchdown_lights && centerline_lights;
    TS_ASSERT(lighting_adequate);
  }

  void testAutopilotChannelMonitor() {
    double channel_1 = 5.0;
    double channel_2 = 5.1;
    double max_difference = 1.0;
    bool channels_agree = std::abs(channel_1 - channel_2) < max_difference;
    TS_ASSERT(channels_agree);
  }

  void testDecrabManeuver() {
    double crosswind_kts = 10.0;
    double approach_speed_kts = 140.0;
    double crab_angle_rad = std::atan(crosswind_kts / approach_speed_kts);
    double crab_angle_deg = crab_angle_rad * Constants::RAD_TO_DEG;
    TS_ASSERT(crab_angle_deg > 3.0 && crab_angle_deg < 5.0);
  }

  void testAutolandSystemRedundancy() {
    int autopilot_channels = 3;
    int required_for_catiii = 2;
    int failures = 1;
    int remaining = autopilot_channels - failures;
    bool can_continue = remaining >= required_for_catiii;
    TS_ASSERT(can_continue);
  }

  void testCompleteAutolandVerification() {
    // Verify all autoland conditions
    double gs_angle_deg = 3.0;
    double airspeed_kts = 140.0;
    double flare_height_ft = 50.0;
    double touchdown_zone_ft = 1000.0;
    double sink_rate_limit_fps = 5.0;

    double gs_rad = gs_angle_deg * Constants::DEG_TO_RAD;
    double gs_fps = airspeed_kts * 1.68781;
    double descent_rate_fps = gs_fps * std::tan(gs_rad);

    TS_ASSERT(descent_rate_fps < 15.0);
    TS_ASSERT(flare_height_ft >= 30.0 && flare_height_ft <= 60.0);
    TS_ASSERT(touchdown_zone_ft >= 500.0 && touchdown_zone_ft <= 1500.0);
    TS_ASSERT(sink_rate_limit_fps <= 6.0);
  }
};

// ============ C172x Aircraft Approach and Landing Integration Tests ============
class FGAutolandC172xTest : public CxxTest::TestSuite
{
public:

  // Test C172x initial approach configuration
  void testC172xInitialApproachConfiguration() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(3000.0);
    ic->SetVcalibratedKtsIC(90.0);

    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();

    double alt = propagate->GetAltitudeASL();
    double vcas = propagate->GetVcalibratedKts();

    TS_ASSERT(std::isfinite(alt));
    TS_ASSERT(std::isfinite(vcas));
    TS_ASSERT(alt > 2500.0);
  }

  // Test C172x glideslope tracking descent
  void testC172xGlideslopeTracking() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(2000.0);
    ic->SetVcalibratedKtsIC(80.0);
    ic->SetFlightPathAngleDegIC(-3.0);  // 3 degree descent

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.3);

    double initialAlt = propagate->GetAltitudeASL();

    for (int i = 0; i < 200; i++) {
      fdmex.Run();

      double alt = propagate->GetAltitudeASL();
      TS_ASSERT(std::isfinite(alt));
    }

    double finalAlt = propagate->GetAltitudeASL();
    // Should have descended
    TS_ASSERT(finalAlt < initialAlt);
  }

  // Test C172x approach speed stability
  void testC172xApproachSpeedStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(1500.0);
    ic->SetVcalibratedKtsIC(70.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.35);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();

      double vcas = propagate->GetVcalibratedKts();
      TS_ASSERT(std::isfinite(vcas));
      TS_ASSERT(vcas > 40.0);  // Above stall
      TS_ASSERT(vcas < 150.0); // Reasonable speed
    }
  }

  // Test C172x pitch attitude during approach
  void testC172xPitchAttitudeApproach() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(1000.0);
    ic->SetVcalibratedKtsIC(75.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.3);
    fcs->SetDeCmd(-0.05);  // Slight nose up for approach

    for (int i = 0; i < 100; i++) {
      fdmex.Run();

      double theta = propagate->GetEulerDeg(2);
      TS_ASSERT(std::isfinite(theta));
      TS_ASSERT(std::abs(theta) < 30.0);  // Reasonable pitch angle
    }
  }

  // Test C172x descent rate calculation
  void testC172xDescentRateCalculation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(2000.0);
    ic->SetVcalibratedKtsIC(80.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto aux = fdmex.GetAuxiliary();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.2);
    fcs->SetDeCmd(0.05);  // Nose down

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double hdot = propagate->Gethdot();
    TS_ASSERT(std::isfinite(hdot));
  }

  // Test C172x landing gear state
  void testC172xLandingGearState() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gear = fdmex.GetGroundReactions();
    TS_ASSERT(gear != nullptr);

    int numGear = gear->GetNumGearUnits();
    TS_ASSERT(numGear > 0);

    // C172x has fixed gear, always extended
    for (int i = 0; i < numGear; i++) {
      auto gearUnit = gear->GetGearUnit(i);
      TS_ASSERT(gearUnit != nullptr);
    }
  }

  // Test C172x final approach flight path
  void testC172xFinalApproachFlightPath() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(500.0);
    ic->SetVcalibratedKtsIC(65.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.25);

    for (int i = 0; i < 150; i++) {
      fdmex.Run();

      double alt = propagate->GetAltitudeASL();
      double vcas = propagate->GetVcalibratedKts();
      double gamma = propagate->GetGamma();

      TS_ASSERT(std::isfinite(alt));
      TS_ASSERT(std::isfinite(vcas));
      TS_ASSERT(std::isfinite(gamma));
    }
  }

  // Test C172x wings level approach
  void testC172xWingsLevelApproach() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(1000.0);
    ic->SetVcalibratedKtsIC(75.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();

      double phi = propagate->GetEulerDeg(1);
      TS_ASSERT(std::isfinite(phi));
      // Should be relatively wings level
      TS_ASSERT(std::abs(phi) < 45.0);
    }
  }

  // Test C172x approach at various airspeeds
  void testC172xApproachAtVariousAirspeeds() {
    double speeds[] = {65.0, 70.0, 75.0, 80.0};

    for (double spd : speeds) {
      FGFDMExec fdmex;
      fdmex.LoadModel("c172x");

      auto ic = fdmex.GetIC();
      ic->SetAltitudeASLFtIC(1500.0);
      ic->SetVcalibratedKtsIC(spd);

      fdmex.RunIC();

      auto prop = fdmex.GetPropulsion();
      auto propagate = fdmex.GetPropagate();
      prop->InitRunning(-1);

      for (int i = 0; i < 50; i++) {
        fdmex.Run();
      }

      double vcas = propagate->GetVcalibratedKts();
      TS_ASSERT(std::isfinite(vcas));
    }
  }

  // Test C172x power management during descent
  void testC172xPowerManagementDescent() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(3000.0);
    ic->SetVcalibratedKtsIC(90.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Reduce power for descent
    fcs->SetThrottleCmd(-1, 0.2);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double power = prop->GetEngine(0)->GetPowerAvailable();
    TS_ASSERT(std::isfinite(power));
  }

  // Test C172x ground proximity
  void testC172xGroundProximity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(200.0);
    ic->SetVcalibratedKtsIC(70.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    prop->InitRunning(-1);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();

      double altAGL = propagate->GetDistanceAGL();
      TS_ASSERT(std::isfinite(altAGL));
    }
  }

  // Test C172x flare initiation altitude
  void testC172xFlareInitiationAltitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(50.0);  // Flare initiation height
    ic->SetVcalibratedKtsIC(65.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.1);
    fcs->SetDeCmd(-0.1);  // Flare nose up

    for (int i = 0; i < 100; i++) {
      fdmex.Run();

      double alt = propagate->GetDistanceAGL();
      double theta = propagate->GetEulerDeg(2);

      TS_ASSERT(std::isfinite(alt));
      TS_ASSERT(std::isfinite(theta));
    }
  }

  // Test C172x extended approach simulation
  void testC172xExtendedApproachSimulation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(2500.0);
    ic->SetVcalibratedKtsIC(85.0);

    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    fcs->SetThrottleCmd(-1, 0.3);

    for (int i = 0; i < 500; i++) {
      fdmex.Run();

      double alt = propagate->GetAltitudeASL();
      double vcas = propagate->GetVcalibratedKts();
      double phi = propagate->GetEulerDeg(1);
      double theta = propagate->GetEulerDeg(2);

      TS_ASSERT(std::isfinite(alt));
      TS_ASSERT(std::isfinite(vcas));
      TS_ASSERT(std::isfinite(phi));
      TS_ASSERT(std::isfinite(theta));
      TS_ASSERT(alt > -1000.0);  // Not crashed through ground
    }
  }
};
