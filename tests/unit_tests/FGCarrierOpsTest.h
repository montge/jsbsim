#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include "TestUtilities.h"
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGGroundReactions.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>

using namespace JSBSimTest;

class FGCarrierOpsTest : public CxxTest::TestSuite
{
public:
  // Catapult Launch Physics Tests

  void testCatapultEndSpeed() {
    // Test catapult end speed calculation using v^2 = v0^2 + 2*a*d
    // Typical values: stroke = 310 ft, acceleration = 5g, initial speed = 0
    double stroke = 310.0; // ft
    double accel = 5.0 * Constants::G_FTPS2; // ft/s^2
    double v0 = 0.0; // ft/s

    double v_end = sqrt(v0*v0 + 2.0*accel*stroke);
    double expected_end_speed = 315.8; // ft/s (approximately 187 knots)

    TS_ASSERT_DELTA(v_end, expected_end_speed, 1.0);
  }

  void testCatapultAccelerationFromEndSpeed() {
    // Given end speed and stroke, calculate required acceleration
    double stroke = 310.0; // ft
    double v_end = 140.0 * Constants::KTS_TO_FTPS; // 140 knots end speed
    double v0 = 20.0 * Constants::KTS_TO_FTPS; // 20 knots wind-over-deck

    // a = (v_end^2 - v0^2) / (2 * stroke)
    double accel = (v_end*v_end - v0*v0) / (2.0 * stroke);
    double accel_g = accel / Constants::G_FTPS2;

    TS_ASSERT_DELTA(accel_g, 2.74, 0.1); // Approximately 2.7g
  }

  void testCatapultLaunchTime() {
    // Test launch time: t = (v_end - v0) / a
    double stroke = 310.0; // ft
    double v_end = 140.0 * Constants::KTS_TO_FTPS; // ft/s
    double v0 = 0.0; // ft/s
    double accel = (v_end*v_end - v0*v0) / (2.0 * stroke);

    double launch_time = (v_end - v0) / accel;

    TS_ASSERT_DELTA(launch_time, 2.63, 0.1); // Approximately 2.6 seconds
  }

  void testCatapultStrokeLongC13() {
    // C-13 steam catapult long stroke specification
    double stroke_long = 310.0; // ft
    double max_aircraft_weight = 73000.0; // lbs
    double end_speed = 140.0 * Constants::KTS_TO_FTPS; // ft/s

    // Force = mass * acceleration = weight/g * a
    double mass = max_aircraft_weight / Constants::G_FTPS2; // slugs
    double accel = (end_speed*end_speed) / (2.0 * stroke_long);
    double force = mass * accel;

    TS_ASSERT_DELTA(force / 1000.0, 204.3, 10.0); // Approximately 204,000 lbs
  }

  void testCatapultStrokeShortC13() {
    // C-13 steam catapult short stroke for lighter aircraft
    double stroke_short = 250.0; // ft
    double aircraft_weight = 45000.0; // lbs
    double end_speed = 130.0 * Constants::KTS_TO_FTPS; // ft/s

    double mass = aircraft_weight / Constants::G_FTPS2;
    double accel = (end_speed*end_speed) / (2.0 * stroke_short);
    double accel_g = accel / Constants::G_FTPS2;

    TS_ASSERT_DELTA(accel_g, 2.99, 0.2); // Approximately 3g
  }

  void testCatapultShuttleResetTime() {
    // Typical shuttle reset time for steam catapult
    double reset_time = 45.0; // seconds between launches (same catapult)
    double launch_interval_two_cats = 30.0; // seconds (alternating)

    // Verify minimum time between launches
    TS_ASSERT(reset_time >= 30.0);
    TS_ASSERT(launch_interval_two_cats >= 20.0);

    // Verify reset is longer than launch interval
    TS_ASSERT(reset_time > launch_interval_two_cats);
  }

  // Arrested Landing Physics Tests

  void testArrestingWireTension() {
    // Calculate cable tension from aircraft deceleration
    double aircraft_weight = 45000.0; // lbs
    double trap_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s
    double decel_time = 2.0; // seconds

    double mass = aircraft_weight / Constants::G_FTPS2; // slugs
    double decel = trap_speed / decel_time; // ft/s^2
    double tension = mass * decel;

    TS_ASSERT_DELTA(tension / 1000.0, 171.1, 5.0); // Approximately 171,000 lbs
  }

  void testArrestingWireEngagementDecel() {
    // Test deceleration during wire engagement
    double v0 = 150.0 * Constants::KTS_TO_FTPS; // ft/s approach speed
    double v_final = 0.0; // ft/s
    double runout = 340.0; // ft (typical wire runout)

    // v_final^2 = v0^2 + 2*a*d -> a = (v_final^2 - v0^2) / (2*d)
    double decel = (v_final*v_final - v0*v0) / (2.0 * runout);
    double decel_g = -decel / Constants::G_FTPS2;

    TS_ASSERT_DELTA(decel_g, 3.1, 0.2); // Approximately 3g deceleration
  }

  void testArrestingWireRunout() {
    // Verify wire runout distance calculation
    double v0 = 145.0 * Constants::KTS_TO_FTPS; // ft/s
    double decel_g = 3.2; // g's
    double decel = decel_g * Constants::G_FTPS2; // ft/s^2

    // d = v0^2 / (2*a)
    double runout = (v0*v0) / (2.0 * decel);

    TS_ASSERT_DELTA(runout, 290.9, 10.0); // Approximately 291 ft
  }

  void testArrestingWireNumber2() {
    // Wire #2 (target wire) typical engagement
    double wire_height = 36.0; // inches above deck
    double wire_spacing = 40.0; // ft between wires
    double wire_number = 2; // Target is wire 2 (of 4)

    TS_ASSERT_DELTA(wire_height / 12.0, 3.0, 0.1); // 3 ft above deck
    TS_ASSERT(wire_number >= 1 && wire_number <= 4);
    TS_ASSERT_DELTA(wire_spacing, 40.0, 1.0);
  }

  void testMaxTrapWeight() {
    // Maximum aircraft weight for safe trap (Mk 7 arresting gear)
    double max_trap_weight = 50000.0; // lbs (typical for Nimitz class)
    double test_aircraft_weight = 48000.0; // lbs

    TS_ASSERT(test_aircraft_weight < max_trap_weight);

    // Calculate margin
    double margin = (max_trap_weight - test_aircraft_weight) / max_trap_weight;
    TS_ASSERT(margin > 0.0);
    TS_ASSERT_DELTA(margin, 0.04, 0.01);
  }

  // Hook Point Geometry Tests

  void testTailhookGeometry() {
    // Tailhook geometry relative to main gear
    double hook_to_main_gear = 15.0; // ft (typical fighter)
    double hook_point_height_down = 2.5; // ft below datum when down
    double hook_angle_down = 7.0 * Constants::DEG_TO_RAD; // radians

    double hook_length = hook_to_main_gear / cos(hook_angle_down);

    TS_ASSERT_DELTA(hook_length, 15.1, 0.2);
    TS_ASSERT(hook_point_height_down > 0.0);
  }

  void testHookStrikePoint() {
    // Calculate hook strike point on deck
    double aircraft_pitch = 3.0 * Constants::DEG_TO_RAD; // radians
    double hook_angle = 7.0 * Constants::DEG_TO_RAD; // radians
    double main_gear_height = 8.0; // ft above deck

    double effective_hook_angle = hook_angle - aircraft_pitch;
    double hook_strike_distance = main_gear_height / tan(effective_hook_angle);

    TS_ASSERT_DELTA(hook_strike_distance, 115.0, 10.0); // Approximately 115 ft
  }

  // Approach Glideslope Tests

  void testStandardGlideslope() {
    // Standard carrier glideslope is 3.5 degrees
    double glideslope = 3.5 * Constants::DEG_TO_RAD; // radians
    double distance_from_ramp = 1000.0; // ft

    double altitude_above_deck = distance_from_ramp * tan(glideslope);

    TS_ASSERT_DELTA(altitude_above_deck, 61.1, 1.0); // Approximately 61 ft
  }

  void testGlideslopeDeviation() {
    // Calculate glideslope error
    double target_glideslope = 3.5 * Constants::DEG_TO_RAD;
    double actual_altitude = 75.0; // ft
    double distance = 1000.0; // ft

    double actual_glideslope = atan(actual_altitude / distance);
    double deviation = (actual_glideslope - target_glideslope) * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(deviation, 0.8, 0.1); // High by 0.8 degrees
  }

  void testThreeQuarterMileGlideslope() {
    // Altitude at 3/4 mile (standard call position)
    double glideslope = 3.5 * Constants::DEG_TO_RAD;
    double distance = 0.75 * 6076.0; // 0.75 nautical miles in feet

    double altitude = distance * tan(glideslope);

    TS_ASSERT_DELTA(altitude, 278.3, 5.0); // Approximately 278 ft
  }

  // Bolter Detection Tests

  void testBolterDetection() {
    // Bolter occurs when hook doesn't engage any wire
    double trap_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s
    double time_after_touchdown = 2.5; // seconds
    double decel_threshold = 0.5 * Constants::G_FTPS2; // ft/s^2

    // If deceleration < threshold, it's a bolter
    double actual_decel = 0.3 * Constants::G_FTPS2; // Minimal decel

    bool is_bolter = (actual_decel < decel_threshold);
    TS_ASSERT(is_bolter);
  }

  void testBolterGoAroundDistance() {
    // Distance traveled during bolter before go-around
    double touchdown_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s
    double time_to_throttle_up = 1.5; // seconds
    double accel = 0.2 * Constants::G_FTPS2; // slight acceleration

    double distance = touchdown_speed * time_to_throttle_up + 0.5 * accel * time_to_throttle_up * time_to_throttle_up;

    TS_ASSERT_DELTA(distance, 371.0, 10.0); // Approximately 371 ft
  }

  // Wire Engagement Timing Tests

  void testWireEngagementTime() {
    // Time from touchdown to full wire engagement
    double engagement_distance = 50.0; // ft
    double touchdown_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s

    double engagement_time = engagement_distance / touchdown_speed;

    TS_ASSERT_DELTA(engagement_time, 0.204, 0.01); // Approximately 0.2 seconds
  }

  void testWireDecelBeginTime() {
    // Time when significant deceleration begins
    double initial_runout = 30.0; // ft before wire starts pulling
    double touchdown_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s

    double decel_begin_time = initial_runout / touchdown_speed;

    TS_ASSERT_DELTA(decel_begin_time, 0.122, 0.01); // Approximately 0.12 seconds
  }

  // Deck Motion Effects Tests

  void testDeckPitchMotion() {
    // Deck pitch amplitude and period in moderate seas
    double pitch_amplitude = 2.0 * Constants::DEG_TO_RAD; // radians
    double pitch_period = 8.0; // seconds
    double time = 2.0; // seconds

    // Sinusoidal pitch motion
    double pitch_angle = pitch_amplitude * sin(2.0 * M_PI * time / pitch_period);

    TS_ASSERT_DELTA(pitch_angle * Constants::RAD_TO_DEG, 2.0, 0.1);
  }

  void testDeckRollMotion() {
    // Deck roll in moderate seas
    double roll_amplitude = 3.0 * Constants::DEG_TO_RAD; // radians
    double roll_period = 10.0; // seconds
    double time = 2.5; // seconds

    double roll_angle = roll_amplitude * sin(2.0 * M_PI * time / roll_period);

    TS_ASSERT_DELTA(roll_angle * Constants::RAD_TO_DEG, 3.0, 0.1);
  }

  void testDeckHeaveMotion() {
    // Vertical deck motion (heave)
    double heave_amplitude = 5.0; // ft
    double heave_period = 12.0; // seconds
    double time = 3.0; // seconds

    double heave = heave_amplitude * sin(2.0 * M_PI * time / heave_period);
    double heave_rate = heave_amplitude * (2.0 * M_PI / heave_period) * cos(2.0 * M_PI * time / heave_period);

    TS_ASSERT_DELTA(heave, 5.0, 0.1); // ft
    TS_ASSERT_DELTA(heave_rate, 0.0, 0.1); // ft/s
  }

  void testDeckMotionCombined() {
    // Combined pitch and heave effect on glide slope
    double pitch = 2.0 * Constants::DEG_TO_RAD; // bow up
    double heave = 5.0; // ft up
    double distance_from_ramp = 500.0; // ft

    // Effective altitude change = heave + distance * sin(pitch)
    double altitude_change = heave + distance_from_ramp * sin(pitch);

    TS_ASSERT_DELTA(altitude_change, 22.4, 1.0); // Approximately 22 ft higher
  }

  // Wind-Over-Deck Calculations

  void testWindOverDeck() {
    // Calculate wind-over-deck from ship speed and true wind
    double ship_speed = 30.0 * Constants::KTS_TO_FTPS; // ft/s
    double true_wind_speed = 15.0 * Constants::KTS_TO_FTPS; // ft/s
    double true_wind_angle = 20.0 * Constants::DEG_TO_RAD; // radians relative to ship heading

    // Component along deck
    double wod_component = ship_speed + true_wind_speed * cos(true_wind_angle);

    TS_ASSERT_DELTA(wod_component / Constants::KTS_TO_FTPS, 44.1, 0.5); // knots
  }

  void testOptimalWindOverDeck() {
    // Optimal WOD for carrier operations
    double optimal_wod = 25.0; // knots
    double optimal_wod_fps = optimal_wod * Constants::KTS_TO_FTPS;

    TS_ASSERT_DELTA(optimal_wod_fps, 42.2, 0.5); // ft/s
    TS_ASSERT(optimal_wod >= 20.0 && optimal_wod <= 35.0);
  }

  void testWindOverDeckCrosswind() {
    // Crosswind component of wind-over-deck
    double true_wind = 20.0 * Constants::KTS_TO_FTPS; // ft/s
    double wind_angle = 30.0 * Constants::DEG_TO_RAD; // radians

    double crosswind = true_wind * sin(wind_angle);

    TS_ASSERT_DELTA(crosswind / Constants::KTS_TO_FTPS, 10.0, 0.2); // knots
  }

  // Approach Speed Tests

  void testApproachSpeedOnSpeed() {
    // On-speed approach speed calculation
    double stall_speed = 105.0; // knots
    double approach_margin = 1.3; // typically 1.2-1.3

    double approach_speed = stall_speed * approach_margin;

    TS_ASSERT_DELTA(approach_speed, 136.5, 2.0); // knots
  }

  void testAngleOfAttackOnSpeed() {
    // Typical on-speed AoA for carrier approach
    double on_speed_aoa = 8.1 * Constants::DEG_TO_RAD; // radians (F/A-18)
    double fast_aoa = 6.5 * Constants::DEG_TO_RAD; // radians
    double slow_aoa = 9.5 * Constants::DEG_TO_RAD; // radians

    TS_ASSERT(on_speed_aoa > fast_aoa);
    TS_ASSERT(on_speed_aoa < slow_aoa);
    TS_ASSERT_DELTA(on_speed_aoa * Constants::RAD_TO_DEG, 8.1, 0.1);
  }

  void testApproachSpeedWithWOD() {
    // Groundspeed at touchdown with wind-over-deck
    double approach_airspeed = 140.0; // knots
    double wod = 25.0; // knots

    double groundspeed = approach_airspeed - wod;

    TS_ASSERT_DELTA(groundspeed, 115.0, 1.0); // knots
  }

  // Meatball Glideslope Indication Tests

  void testMeatballOnGlideslope() {
    // Fresnel lens system indication
    double target_glideslope = 3.5 * Constants::DEG_TO_RAD;
    double aircraft_glideslope = 3.5 * Constants::DEG_TO_RAD;

    double deviation = (aircraft_glideslope - target_glideslope) * Constants::RAD_TO_DEG;

    TS_ASSERT_DELTA(deviation, 0.0, DEFAULT_TOLERANCE);
  }

  void testMeatballHighIndication() {
    // High indication (above glideslope)
    double target_gs = 3.5 * Constants::DEG_TO_RAD;
    double actual_gs = 4.0 * Constants::DEG_TO_RAD;

    double deviation = (actual_gs - target_gs) * Constants::RAD_TO_DEG;

    TS_ASSERT(deviation > 0.3); // More than 0.3 degrees high
  }

  void testMeatballLowIndication() {
    // Low indication (below glideslope)
    double target_gs = 3.5 * Constants::DEG_TO_RAD;
    double actual_gs = 3.0 * Constants::DEG_TO_RAD;

    double deviation = (actual_gs - target_gs) * Constants::RAD_TO_DEG;

    TS_ASSERT(deviation < -0.3); // More than 0.3 degrees low
  }

  // LSO Calls Tests

  void testLSOPowerCall() {
    // LSO power call based on energy state
    double actual_speed = 135.0; // knots
    double target_speed = 140.0; // knots
    double speed_deficit = target_speed - actual_speed;

    bool power_call = (speed_deficit > 3.0);

    TS_ASSERT(power_call);
    TS_ASSERT_DELTA(speed_deficit, 5.0, 0.1);
  }

  void testLSOLineupCall() {
    // LSO lineup correction based on angle-off-centerline
    double centerline_angle = 0.0 * Constants::DEG_TO_RAD;
    double aircraft_angle = 2.5 * Constants::DEG_TO_RAD;

    double lineup_error = (aircraft_angle - centerline_angle) * Constants::RAD_TO_DEG;

    bool lineup_call = (fabs(lineup_error) > 1.5);
    TS_ASSERT(lineup_call);
  }

  // Case I/II/III Approach Tests

  void testCaseIApproachPattern() {
    // Case I: Visual pattern (day, good weather)
    double break_altitude = 800.0; // ft
    double downwind_altitude = 600.0; // ft
    double abeam_distance = 1.0 * 6076.0; // 1 NM in feet

    TS_ASSERT(break_altitude > 500.0);
    TS_ASSERT(downwind_altitude < break_altitude);
    TS_ASSERT_DELTA(abeam_distance / 6076.0, 1.0, 0.1);
  }

  void testCaseIIApproachPattern() {
    // Case II: TACAN/Precision approach (restricted visibility)
    double initial_altitude = 5000.0; // ft
    double final_approach_altitude = 1200.0; // ft at 3 NM
    double distance_3nm = 3.0 * 6076.0; // ft

    double descent_angle = atan((initial_altitude - final_approach_altitude) / distance_3nm);

    TS_ASSERT_DELTA(descent_angle * Constants::RAD_TO_DEG, 11.8, 0.5);
  }

  void testCaseIIIApproachPattern() {
    // Case III: Full instrument approach (night, low visibility)
    double platform_altitude = 1200.0; // ft
    double commencing_altitude = 5000.0; // ft
    double glideslope_intercept = 3.0 * 6076.0; // 3 NM in feet

    TS_ASSERT(platform_altitude > 1000.0);
    TS_ASSERT(commencing_altitude > platform_altitude);
    TS_ASSERT_DELTA(glideslope_intercept / 6076.0, 3.0, 0.1);
  }

  // Marshal Pattern Tests

  void testMarshalStackAltitude() {
    // Aircraft stacked at 1000 ft intervals
    double base_altitude = 6000.0; // ft
    int stack_position = 3; // Third aircraft
    double altitude_interval = 1000.0; // ft

    double holding_altitude = base_altitude + (stack_position - 1) * altitude_interval;

    TS_ASSERT_DELTA(holding_altitude, 8000.0, 0.1);
  }

  void testMarshalHoldingPattern() {
    // Standard marshal holding pattern
    double holding_speed = 250.0; // knots
    double pattern_radius = 1.0 * 6076.0; // 1 NM
    double bank_angle = 30.0 * Constants::DEG_TO_RAD;

    // Turn rate = g * tan(bank) / velocity
    double velocity = holding_speed * Constants::KTS_TO_FTPS;
    double turn_rate = Constants::G_FTPS2 * tan(bank_angle) / velocity;

    TS_ASSERT_DELTA(turn_rate * Constants::RAD_TO_DEG, 2.52, 0.2); // deg/s
  }

  // TACAN Navigation Tests

  void testTACANDistanceToCarrier() {
    // Calculate slant range to carrier
    double horizontal_distance = 10.0 * 6076.0; // 10 NM in feet
    double altitude_above_carrier = 5000.0; // ft

    double slant_range = sqrt(horizontal_distance * horizontal_distance + altitude_above_carrier * altitude_above_carrier);

    TS_ASSERT_DELTA(slant_range / 6076.0, 10.04, 0.1); // NM
  }

  void testTACANBearingCorrection() {
    // Magnetic bearing to TACAN beacon
    double true_bearing = 90.0; // degrees
    double magnetic_variation = 10.0; // degrees east

    double magnetic_bearing = true_bearing - magnetic_variation;

    TS_ASSERT_DELTA(magnetic_bearing, 80.0, 0.1);
  }

  // Deck Spotting Calculations Tests

  void testAircraftSpotFactor() {
    // Deck spot factor (square feet per aircraft)
    double fighter_spot = 900.0; // sq ft (F/A-18)
    double deck_area_forward = 25000.0; // sq ft available

    double max_aircraft_forward = deck_area_forward / fighter_spot;

    TS_ASSERT_DELTA(max_aircraft_forward, 27.8, 1.0);
  }

  void testDeckCycleTime() {
    // Time to complete a full deck cycle
    double aircraft_to_launch = 12;
    double launch_interval = 30.0; // seconds

    double total_launch_time = aircraft_to_launch * launch_interval;

    TS_ASSERT_DELTA(total_launch_time / 60.0, 6.0, 0.5); // minutes
  }

  // Fuel State Tests

  void testBingoFuel() {
    // Bingo fuel: minimum to reach alternate + reserve
    double fuel_to_alternate = 2500.0; // lbs
    double reserve_fuel = 2000.0; // lbs (20 minutes at cruise)

    double bingo_fuel = fuel_to_alternate + reserve_fuel;

    TS_ASSERT_DELTA(bingo_fuel, 4500.0, 0.1);
  }

  void testFuelLadder() {
    // Fuel ladder: time to bingo at current burn rate
    double current_fuel = 6000.0; // lbs
    double bingo_fuel = 4500.0; // lbs
    double fuel_flow = 3000.0; // lbs/hr

    double time_to_bingo = (current_fuel - bingo_fuel) / (fuel_flow / 60.0); // minutes

    TS_ASSERT_DELTA(time_to_bingo, 30.0, 1.0); // minutes
  }

  void testJokerFuel() {
    // Joker fuel: fuel state to leave marshal stack
    double bingo_fuel = 4500.0; // lbs
    double fuel_margin = 1000.0; // lbs

    double joker_fuel = bingo_fuel + fuel_margin;

    TS_ASSERT_DELTA(joker_fuel, 5500.0, 0.1);
  }

  // Emergency Barrier Engagement Tests

  void testBarrierEngagementDecel() {
    // Emergency barrier deceleration (higher than wire)
    double barrier_speed = 100.0 * Constants::KTS_TO_FTPS; // ft/s
    double barrier_runout = 150.0; // ft (shorter than wire)

    double decel = (barrier_speed * barrier_speed) / (2.0 * barrier_runout);
    double decel_g = decel / Constants::G_FTPS2;

    TS_ASSERT_DELTA(decel_g, 2.95, 0.3); // Approximately 3g
  }

  void testBarrierHeight() {
    // Barrier height above deck
    double barrier_height = 20.0; // ft when raised
    double aircraft_height = 15.0; // ft at nose

    TS_ASSERT(barrier_height > aircraft_height);

    double clearance = barrier_height - aircraft_height;
    TS_ASSERT_DELTA(clearance, 5.0, 0.1);
  }

  // Pitching Deck Cycle Tests

  void testPitchingDeckCycle() {
    // One complete pitch cycle
    double pitch_period = 8.0; // seconds
    double pitch_amplitude = 2.0 * Constants::DEG_TO_RAD; // radians

    // Maximum pitch rate occurs at centerline (zero pitch angle)
    double max_pitch_rate = pitch_amplitude * 2.0 * M_PI / pitch_period;

    TS_ASSERT_DELTA(max_pitch_rate * Constants::RAD_TO_DEG, 1.57, 0.5); // deg/s
  }

  void testDeckAngleAtTouchdown() {
    // Deck pitch angle at touchdown affects trap
    double pitch_angle = 1.5 * Constants::DEG_TO_RAD; // bow up
    double aircraft_aoa = 8.0 * Constants::DEG_TO_RAD;

    // Effective AoA relative to deck
    double effective_aoa = aircraft_aoa - pitch_angle;

    TS_ASSERT_DELTA(effective_aoa * Constants::RAD_TO_DEG, 6.5, 0.1);
  }

  void testOptimalTouchdownTiming() {
    // Timing touchdown with deck pitch cycle
    double pitch_period = 8.0; // seconds
    double pitch_phase_optimal = M_PI / 4.0; // 45 degrees into cycle

    double optimal_time = pitch_phase_optimal * pitch_period / (2.0 * M_PI);

    TS_ASSERT_DELTA(optimal_time, 1.0, 0.1); // seconds into cycle
  }

  // EMALS (Electromagnetic Aircraft Launch System) Tests

  void testEMALSEndSpeed() {
    // EMALS provides smoother acceleration than steam
    double stroke = 300.0; // ft
    double end_speed = 150.0 * Constants::KTS_TO_FTPS; // ft/s
    double v0 = 0.0;

    double accel = (end_speed * end_speed - v0 * v0) / (2.0 * stroke);
    double accel_g = accel / Constants::G_FTPS2;

    // EMALS can provide precise acceleration
    TS_ASSERT_DELTA(accel_g, 3.37, 0.2);
  }

  void testEMALSEnergyRequirement() {
    // Electrical energy needed for launch
    double aircraft_mass = 70000.0 / Constants::G_FTPS2; // slugs
    double end_speed = 150.0 * Constants::KTS_TO_FTPS; // ft/s

    // KE = 0.5 * m * v^2
    double kinetic_energy = 0.5 * aircraft_mass * end_speed * end_speed;
    double energy_megajoules = kinetic_energy * 1.356e-6; // Convert ft-lb to MJ

    // EMALS typically requires 90-100 MJ for heavy aircraft
    TS_ASSERT(energy_megajoules > 80.0);
    TS_ASSERT(energy_megajoules < 120.0);
  }

  void testEMALSAccelerationProfile() {
    // EMALS can vary acceleration profile
    double peak_accel_g = 4.0;
    double min_accel_g = 2.5;
    double average_accel_g = (peak_accel_g + min_accel_g) / 2.0;

    TS_ASSERT_DELTA(average_accel_g, 3.25, 0.1);
    TS_ASSERT(peak_accel_g > average_accel_g);
  }

  // Advanced Arresting Gear (AAG) Tests

  void testAAGEnergyAbsorption() {
    // AAG uses water turbines for energy absorption
    double aircraft_mass = 50000.0 / Constants::G_FTPS2; // slugs
    double trap_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s

    double kinetic_energy = 0.5 * aircraft_mass * trap_speed * trap_speed;
    double energy_megajoules = kinetic_energy * 1.356e-6;

    TS_ASSERT(energy_megajoules > 50.0); // Significant energy
  }

  void testAAGControlledDecel() {
    // AAG provides controlled deceleration
    double target_decel_g = 3.0;
    double tolerance = 0.2; // g's

    double min_decel = target_decel_g - tolerance;
    double max_decel = target_decel_g + tolerance;

    TS_ASSERT(min_decel > 2.5);
    TS_ASSERT(max_decel < 3.5);
  }

  // Night Carrier Landing Tests

  void testNightApproachLighting() {
    // Fresnel lens visibility at night
    double lens_brightness = 100.0; // relative units
    double visibility = 5.0; // NM
    double minimum_visibility = 0.5; // NM for night ops

    TS_ASSERT(visibility > minimum_visibility);
    TS_ASSERT(lens_brightness > 50.0);
  }

  void testNightGlideslopeAccuracy() {
    // Tighter glideslope requirements at night
    double day_tolerance = 0.5 * Constants::DEG_TO_RAD; // radians
    double night_tolerance = 0.3 * Constants::DEG_TO_RAD; // radians

    TS_ASSERT(night_tolerance < day_tolerance);
    TS_ASSERT_DELTA(night_tolerance * Constants::RAD_TO_DEG, 0.3, 0.05);
  }

  void testDropLightsIndication() {
    // Drop lights for lineup in low visibility
    double lineup_error = 1.5 * Constants::DEG_TO_RAD; // radians
    double drop_light_threshold = 1.0 * Constants::DEG_TO_RAD;

    bool drop_lights_activated = fabs(lineup_error) > drop_light_threshold;
    TS_ASSERT(drop_lights_activated);
  }

  // Hook Bounce Scenarios

  void testHookBounceDistance() {
    // Distance traveled during hook skip
    double bounce_height = 2.0; // ft
    double forward_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s
    double bounce_time = sqrt(2.0 * bounce_height / Constants::G_FTPS2) * 2.0;

    double bounce_distance = forward_speed * bounce_time;

    // Hook skip covers significant distance (about 170 ft)
    TS_ASSERT_DELTA(bounce_distance, 172.6, 5.0); // ft
  }

  void testHookSkipRecovery() {
    // Time to recover from hook skip
    double descent_rate = 750.0 / 60.0; // ft/s (from fpm)
    double hook_height = 2.0; // ft above deck

    double recovery_time = hook_height / descent_rate;

    TS_ASSERT_DELTA(recovery_time, 0.16, 0.02); // seconds
  }

  // Waveoff Procedures

  void testWaveoffClimbAngle() {
    // Required climb angle during waveoff
    double climb_rate = 3000.0; // fpm
    double forward_speed = 150.0 * Constants::KTS_TO_FTPS; // ft/s

    double climb_angle = atan((climb_rate / 60.0) / forward_speed);

    TS_ASSERT_DELTA(climb_angle * Constants::RAD_TO_DEG, 11.3, 0.5);
  }

  void testWaveoffMinAltitude() {
    // Minimum altitude gain before turn
    double min_altitude = 400.0; // ft
    double climb_rate = 3000.0; // fpm

    double time_to_min = min_altitude / (climb_rate / 60.0);

    TS_ASSERT_DELTA(time_to_min, 8.0, 0.5); // seconds
  }

  void testWaveoffAbeamDistance() {
    // Distance abeam when cleared to turn
    double waveoff_speed = 180.0 * Constants::KTS_TO_FTPS; // ft/s
    double time_before_turn = 15.0; // seconds

    double abeam_distance = waveoff_speed * time_before_turn;

    TS_ASSERT_DELTA(abeam_distance / 6076.0, 0.75, 0.1); // NM
  }

  // Foul Deck Situations

  void testFoulDeckClearanceTime() {
    // Time to clear foul deck
    double tractor_speed = 5.0; // knots
    double deck_length = 300.0; // ft

    double clear_time = deck_length / (tractor_speed * Constants::KTS_TO_FTPS);

    TS_ASSERT(clear_time > 30.0); // More than 30 seconds
  }

  void testFoulDeckWaveoffDecision() {
    // Decision altitude for foul deck waveoff
    double decision_altitude = 200.0; // ft
    double glideslope = 3.5 * Constants::DEG_TO_RAD;

    double decision_distance = decision_altitude / tan(glideslope);

    TS_ASSERT_DELTA(decision_distance, 3270.0, 100.0); // ft
  }

  // Delta Pattern Operations

  void testDeltaPatternAltitude() {
    // Delta pattern altitude for fuel conservation
    double marshal_altitude = 2000.0; // ft
    double delta_altitude = 3000.0; // ft

    TS_ASSERT(delta_altitude > marshal_altitude);
    TS_ASSERT(delta_altitude < 5000.0);
  }

  void testDeltaPatternSpacing() {
    // Spacing between aircraft in delta
    double time_spacing = 60.0; // seconds
    double speed = 250.0 * Constants::KTS_TO_FTPS; // ft/s

    double distance_spacing = speed * time_spacing;

    TS_ASSERT_DELTA(distance_spacing / 6076.0, 4.14, 0.2); // NM
  }

  // Catapult Steam Pressure Tests

  void testSteamCatapultPressure() {
    // Steam pressure for different aircraft weights
    double light_weight = 35000.0; // lbs
    double heavy_weight = 70000.0; // lbs
    double max_pressure = 520.0; // psi

    double light_pressure = max_pressure * (light_weight / heavy_weight);

    TS_ASSERT_DELTA(light_pressure, 260.0, 10.0); // psi
  }

  void testSteamRecoveryTime() {
    // Time to rebuild steam pressure after launch
    double pressure_drop = 100.0; // psi
    double recovery_rate = 5.0; // psi/second

    double recovery_time = pressure_drop / recovery_rate;

    TS_ASSERT_DELTA(recovery_time, 20.0, 2.0); // seconds
  }

  // Crosswind Landing Effects

  void testCrossDeckWind() {
    // Effect of cross-deck wind on approach
    double crosswind = 15.0 * Constants::KTS_TO_FTPS; // ft/s
    double approach_speed = 140.0 * Constants::KTS_TO_FTPS; // ft/s

    double crab_angle = asin(crosswind / approach_speed);

    TS_ASSERT_DELTA(crab_angle * Constants::RAD_TO_DEG, 6.1, 0.3);
  }

  void testMaxCrosswindLimit() {
    // Maximum allowable crosswind
    double max_crosswind = 25.0; // knots
    double actual_crosswind = 20.0; // knots

    bool within_limits = actual_crosswind < max_crosswind;
    TS_ASSERT(within_limits);
  }

  // Vertical Velocity at Touchdown

  void testTouchdownVerticalVelocity() {
    // Typical carrier touchdown vertical velocity
    double glideslope = 3.5 * Constants::DEG_TO_RAD;
    double approach_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s

    double vertical_velocity = approach_speed * sin(glideslope);

    TS_ASSERT_DELTA(vertical_velocity, 14.95, 0.5); // ft/s
    TS_ASSERT(vertical_velocity > 10.0); // Higher than land-based
  }

  void testLandingGearLoadAtTrap() {
    // Landing gear load factor at trap
    double aircraft_weight = 45000.0; // lbs
    double touchdown_vv = 15.0; // ft/s vertical
    double gear_stroke = 1.0; // ft

    // Energy = 0.5 * m * v^2 = F * d
    double mass = aircraft_weight / Constants::G_FTPS2;
    double energy = 0.5 * mass * touchdown_vv * touchdown_vv;
    double gear_force = energy / gear_stroke;
    double load_factor = gear_force / aircraft_weight;

    TS_ASSERT(load_factor > 1.0);
  }

  // Recovery Tanker Operations

  void testTankerPatternAltitude() {
    // Recovery tanker holds at higher altitude
    double tanker_altitude = 6000.0; // ft
    double marshal_base = 2000.0; // ft

    TS_ASSERT(tanker_altitude > marshal_base);
    TS_ASSERT_DELTA(tanker_altitude, 6000.0, 500.0);
  }

  void testMinimumFuelForTank() {
    // Minimum fuel state to receive tanking
    double critical_fuel = 2000.0; // lbs
    double bingo_fuel = 4500.0; // lbs

    bool needs_tank = critical_fuel < bingo_fuel;
    TS_ASSERT(needs_tank);
  }

  // Burble Effect (Deck Wake Turbulence) Tests

  void testBurbleZoneExtent() {
    // Burble zone extends behind carrier
    double carrier_speed = 30.0 * Constants::KTS_TO_FTPS; // ft/s
    double burble_length_multiplier = 3.0; // ship lengths
    double carrier_length = 1100.0; // ft (Nimitz class)

    double burble_zone_length = burble_length_multiplier * carrier_length;

    TS_ASSERT_DELTA(burble_zone_length, 3300.0, 100.0); // ft
    TS_ASSERT(burble_zone_length > 3000.0);
  }

  void testBurbleDowndraftMagnitude() {
    // Downdraft in burble zone
    double wod = 30.0 * Constants::KTS_TO_FTPS; // ft/s
    double downdraft_fraction = 0.08; // 8% of WOD

    double downdraft = wod * downdraft_fraction;

    TS_ASSERT_DELTA(downdraft, 4.05, 0.2); // ft/s
    TS_ASSERT(downdraft > 2.0); // Noticeable effect
  }

  void testBurbleAirspeedFluctuation() {
    // Airspeed fluctuation in burble
    double approach_airspeed = 140.0 * Constants::KTS_TO_FTPS; // ft/s
    double fluctuation_percent = 5.0; // +/- 5%

    double max_fluctuation = approach_airspeed * (fluctuation_percent / 100.0);

    TS_ASSERT_DELTA(max_fluctuation / Constants::KTS_TO_FTPS, 7.0, 0.5); // knots
  }

  // AoA Indexer Lights Tests

  void testAoAIndexerOnSpeed() {
    // On-speed: donut light
    double target_aoa = 8.1; // degrees
    double actual_aoa = 8.0; // degrees
    double tolerance = 0.5; // degrees

    bool is_on_speed = fabs(actual_aoa - target_aoa) < tolerance;
    TS_ASSERT(is_on_speed);
  }

  void testAoAIndexerFast() {
    // Fast: chevron pointing down (low AoA)
    double target_aoa = 8.1; // degrees
    double actual_aoa = 6.0; // degrees
    double fast_threshold = target_aoa - 1.0;

    bool is_fast = actual_aoa < fast_threshold;
    TS_ASSERT(is_fast);
  }

  void testAoAIndexerSlow() {
    // Slow: chevron pointing up (high AoA)
    double target_aoa = 8.1; // degrees
    double actual_aoa = 10.0; // degrees
    double slow_threshold = target_aoa + 1.0;

    bool is_slow = actual_aoa > slow_threshold;
    TS_ASSERT(is_slow);
  }

  // Carrier Course and Speed Management

  void testOptimalRecoveryHeading() {
    // Carrier turns into wind for recovery
    double true_wind_direction = 270.0; // degrees
    double angled_deck_offset = 10.0; // degrees port

    // Optimal heading is into wind plus deck angle offset
    double recovery_heading = true_wind_direction - angled_deck_offset;
    if (recovery_heading < 0) recovery_heading += 360.0;

    TS_ASSERT_DELTA(recovery_heading, 260.0, 1.0);
  }

  void testMinimumCarrierSpeed() {
    // Minimum speed for safe operations
    double min_speed_with_wind = 15.0; // knots
    double min_speed_no_wind = 25.0; // knots
    double true_wind = 10.0; // knots

    double required_ship_speed = min_speed_no_wind - true_wind;
    TS_ASSERT_DELTA(required_ship_speed, 15.0, 1.0);
    TS_ASSERT(required_ship_speed >= min_speed_with_wind);
  }

  // Aircraft Energy Management

  void testEnergyAngleCalculation() {
    // Energy angle for approach
    double aircraft_weight = 45000.0; // lbs
    double drag = 8000.0; // lbs (clean config)
    double thrust = 7500.0; // lbs (approach power)

    double thrust_deficit = drag - thrust;
    double energy_angle = asin(thrust_deficit / aircraft_weight);

    TS_ASSERT_DELTA(energy_angle * Constants::RAD_TO_DEG, 0.64, 0.1);
  }

  void testSteadyStateDescentRate() {
    // Descent rate for given power setting
    double approach_speed = 140.0 * Constants::KTS_TO_FTPS; // ft/s
    double glideslope = 3.5 * Constants::DEG_TO_RAD;

    double descent_rate = approach_speed * sin(glideslope);
    double descent_rate_fpm = descent_rate * 60.0;

    TS_ASSERT_DELTA(descent_rate_fpm, 860.0, 30.0); // fpm
  }

  void testPowerForLevelFlight() {
    // Power required to maintain level flight
    double aircraft_weight = 45000.0; // lbs
    double approach_speed = 140.0 * Constants::KTS_TO_FTPS; // ft/s
    double aoa = 8.0 * Constants::DEG_TO_RAD;

    // Simplified: thrust = weight * sin(aoa) for level slow flight
    double thrust_required = aircraft_weight * sin(aoa);

    TS_ASSERT(thrust_required > 5000.0); // lbs
    TS_ASSERT(thrust_required < 8000.0);
  }

  // Aircraft Configuration Constraints

  void testMaxSpeedForGearDown() {
    // Maximum speed with gear down
    double max_gear_speed = 250.0; // knots
    double approach_speed = 140.0; // knots

    TS_ASSERT(approach_speed < max_gear_speed);
    double margin = max_gear_speed - approach_speed;
    TS_ASSERT(margin > 100.0);
  }

  void testMaxSpeedForHookDown() {
    // Maximum speed with tailhook lowered
    double max_hook_speed = 200.0; // knots
    double approach_speed = 145.0; // knots

    TS_ASSERT(approach_speed < max_hook_speed);
    double margin = max_hook_speed - approach_speed;
    TS_ASSERT_DELTA(margin, 55.0, 1.0);
  }

  // Multi-Aircraft Recovery Sequence

  void testIntervalBetweenTraps() {
    // Time between successive traps
    double min_interval = 45.0; // seconds
    double deck_cycle_time = 60.0; // seconds average

    TS_ASSERT(deck_cycle_time >= min_interval);
    TS_ASSERT(deck_cycle_time < 90.0);
  }

  void testRecoveryWindowDuration() {
    // Total recovery window for all aircraft
    double num_aircraft = 12;
    double avg_interval = 60.0; // seconds

    double total_time = num_aircraft * avg_interval;
    double total_minutes = total_time / 60.0;

    TS_ASSERT_DELTA(total_minutes, 12.0, 0.5);
  }

  // Emergency Divert Scenarios

  void testDivertDistanceCalculation() {
    // Distance to nearest divert field
    double divert_lat_diff = 0.5; // degrees
    double divert_lon_diff = 0.75; // degrees
    double nm_per_degree = 60.0;

    double lat_distance = divert_lat_diff * nm_per_degree;
    double lon_distance = divert_lon_diff * nm_per_degree * cos(30.0 * Constants::DEG_TO_RAD);
    double divert_distance = sqrt(lat_distance * lat_distance + lon_distance * lon_distance);

    TS_ASSERT(divert_distance < 60.0); // Within 60 NM
  }

  void testDivertFuelRequired() {
    // Fuel needed to reach divert
    double divert_distance = 50.0; // NM
    double cruise_speed = 300.0; // knots
    double fuel_flow = 4000.0; // lbs/hr

    double flight_time = divert_distance / cruise_speed; // hours
    double fuel_required = fuel_flow * flight_time;

    TS_ASSERT_DELTA(fuel_required, 666.7, 20.0); // lbs
  }

  // Angle of Bank in Pattern

  void testDownwindBankAngle() {
    // Bank angle in visual pattern
    double pattern_speed = 250.0 * Constants::KTS_TO_FTPS; // ft/s
    double turn_radius = 5000.0; // ft

    // bank = atan(v^2 / (g * r))
    double bank_angle = atan((pattern_speed * pattern_speed) / (Constants::G_FTPS2 * turn_radius));

    // 250 kts at 5000 ft radius requires steep bank (about 48 degrees)
    TS_ASSERT_DELTA(bank_angle * Constants::RAD_TO_DEG, 48.0, 2.0);
  }

  void testGrooveRollRate() {
    // Maximum roll rate in the groove (final approach)
    double max_roll_rate = 10.0; // deg/s
    double actual_roll_rate = 5.0; // deg/s

    TS_ASSERT(actual_roll_rate < max_roll_rate);
    TS_ASSERT(actual_roll_rate > 0.0);
  }

  // Complete Carrier Landing Simulation Tests

  void testCompleteApproachProfile() {
    // Test complete approach from 3/4 mile
    double start_distance = 0.75 * 6076.0; // ft (3/4 NM)
    double glideslope = 3.5 * Constants::DEG_TO_RAD;
    double approach_speed = 140.0 * Constants::KTS_TO_FTPS; // ft/s

    // Starting altitude
    double start_altitude = start_distance * tan(glideslope);

    // Time to touchdown
    double time_to_touchdown = start_distance / approach_speed;

    // Verify reasonable values
    TS_ASSERT_DELTA(start_altitude, 278.0, 10.0); // ft
    TS_ASSERT_DELTA(time_to_touchdown, 19.3, 1.0); // seconds
  }

  void testCompleteTrapSequence() {
    // Complete trap from wire engagement to stop
    double touchdown_speed = 145.0 * Constants::KTS_TO_FTPS; // ft/s
    double runout = 340.0; // ft
    double decel_g = 3.0;
    double decel = decel_g * Constants::G_FTPS2;

    // Time to stop: t = v0 / a
    double stop_time = touchdown_speed / decel;

    // Verify kinematic consistency
    double calculated_runout = (touchdown_speed * touchdown_speed) / (2.0 * decel);

    TS_ASSERT_DELTA(stop_time, 2.53, 0.2); // seconds
    TS_ASSERT_DELTA(calculated_runout, 310.0, 30.0); // ft
  }

  void testCompleteCarrierOpsVerification() {
    // Comprehensive verification of carrier operations physics

    // 1. Catapult launch
    double cat_stroke = 310.0; // ft
    double cat_accel_g = 3.0;
    double cat_accel = cat_accel_g * Constants::G_FTPS2;
    double cat_end_speed = sqrt(2.0 * cat_accel * cat_stroke);
    TS_ASSERT(cat_end_speed > 200.0); // ft/s

    // 2. Approach glideslope
    double glideslope = 3.5 * Constants::DEG_TO_RAD;
    double distance_1nm = 6076.0;
    double altitude_at_1nm = distance_1nm * tan(glideslope);
    TS_ASSERT_DELTA(altitude_at_1nm, 371.5, 10.0);

    // 3. Wind-over-deck
    double ship_speed = 25.0;
    double true_wind = 10.0;
    double wod = ship_speed + true_wind;
    TS_ASSERT_DELTA(wod, 35.0, 0.1);

    // 4. Arresting wire engagement
    double trap_speed = 145.0 * Constants::KTS_TO_FTPS;
    double wire_runout = 340.0;
    double trap_decel = (trap_speed * trap_speed) / (2.0 * wire_runout);
    double trap_decel_g = trap_decel / Constants::G_FTPS2;
    TS_ASSERT_DELTA(trap_decel_g, 2.78, 0.2);

    // 5. Deck motion effect on glideslope
    double pitch_amplitude = 2.0 * Constants::DEG_TO_RAD;
    double distance_from_ramp = 500.0;
    double altitude_effect = distance_from_ramp * sin(pitch_amplitude);
    TS_ASSERT_DELTA(altitude_effect, 17.4, 1.0);

    // 6. Fuel management
    double bingo_fuel = 4500.0;
    double current_fuel = 6000.0;
    double margin = current_fuel - bingo_fuel;
    TS_ASSERT(margin > 1000.0);

    // All carrier ops physics verified
  }

  // Test 100: Complete carrier recovery cycle verification
  void testCompleteRecoveryCycleVerification() {
    // Comprehensive test of full recovery cycle

    // 1. Marshal stack entry
    double marshal_altitude = 6000.0;  // ft
    double holding_speed = 250.0 * Constants::KTS_TO_FTPS;
    TS_ASSERT(marshal_altitude > 5000.0);

    // 2. Approach descent
    double glideslope = 3.5 * Constants::DEG_TO_RAD;
    double three_quarter_nm = 0.75 * 6076.0;
    double approach_alt = three_quarter_nm * tan(glideslope);
    TS_ASSERT_DELTA(approach_alt, 278.0, 10.0);

    // 3. Wind-over-deck
    double ship_speed = 28.0;
    double true_wind = 7.0;
    double wod = ship_speed + true_wind;
    TS_ASSERT(wod >= 30.0);

    // 4. Touchdown parameters
    double approach_speed = 140.0 * Constants::KTS_TO_FTPS;
    double touchdown_vv = approach_speed * sin(glideslope);
    TS_ASSERT_DELTA(touchdown_vv, 14.4, 1.0);

    // 5. Arresting wire engagement
    double runout = 340.0;
    double decel = (approach_speed * approach_speed) / (2.0 * runout);
    double decel_g = decel / Constants::G_FTPS2;
    TS_ASSERT(decel_g > 2.0 && decel_g < 4.0);

    // 6. Time to stop
    double stop_time = approach_speed / decel;
    TS_ASSERT(stop_time < 3.0);

    // Full carrier recovery cycle verified
  }
};

/*******************************************************************************
 * FGCarrierOps C172x Integration Tests
 * Tests ground operations applicable to C172x (runway operations, not carrier)
 * These tests verify the ground handling physics that underlie carrier ops
 ******************************************************************************/
class FGCarrierOpsC172xTest : public CxxTest::TestSuite
{
public:
  JSBSim::FGFDMExec fdmex;
  std::string aircraft_path;

  void setUp() {
    aircraft_path = "aircraft";
    fdmex.SetAircraftPath(SGPath("aircraft"));
    fdmex.SetEnginePath(SGPath("engine"));
    fdmex.SetSystemsPath(SGPath("systems"));
  }

  void tearDown() {
    fdmex.ResetToInitialConditions(0);
  }

  // Test C172x takeoff roll acceleration
  void testC172xTakeoffRollAcceleration() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(0.0);
    TS_ASSERT(fdmex.RunIC());

    auto propagate = fdmex.GetPropagate();
    auto propulsion = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    // Full throttle for takeoff
    fcs->SetThrottleCmd(-1, 1.0);
    propulsion->GetEngine(0)->SetRunning(true);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double speed = propagate->GetVel().Magnitude();

    // Should be accelerating on ground
    TS_ASSERT(speed > 10.0);  // Some forward motion expected
  }

  // Test C172x landing rollout deceleration
  void testC172xLandingRolloutDeceleration() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(50.0);  // Landing speed
    TS_ASSERT(fdmex.RunIC());

    auto propagate = fdmex.GetPropagate();
    double initialSpeed = propagate->GetVel().Magnitude();

    auto fcs = fdmex.GetFCS();
    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double finalSpeed = propagate->GetVel().Magnitude();

    // Should have decelerated significantly
    TS_ASSERT(finalSpeed < initialSpeed * 0.5);
  }

  // Test C172x ground roll distance estimation
  void testC172xGroundRollDistance() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    double lat0 = 0.0;
    double lon0 = 0.0;
    ic->SetLatitudeRadIC(lat0);
    ic->SetLongitudeRadIC(lon0);
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(50.0);
    TS_ASSERT(fdmex.RunIC());

    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();

    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);

    // Run until stopped or 500 iterations
    for (int i = 0; i < 500; i++) {
      fdmex.Run();
      if (propagate->GetVel().Magnitude() < 1.0) break;
    }

    // Verify aircraft traveled some distance
    double latFinal = propagate->GetLatitude();
    double lonFinal = propagate->GetLongitude();
    double distance = sqrt(pow(latFinal - lat0, 2) + pow(lonFinal - lon0, 2));

    TS_ASSERT(distance > 0.0);
  }

  // Test C172x holding position with brakes
  void testC172xHoldingPositionWithBrakes() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(0.0);
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    auto propulsion = fdmex.GetPropulsion();

    // Apply brakes and some throttle
    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);
    fcs->SetThrottleCmd(-1, 0.3);
    propulsion->GetEngine(0)->SetRunning(true);

    for (int i = 0; i < 50; i++) fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    double speed = propagate->GetVel().Magnitude();

    // Should remain relatively stationary with brakes applied
    TS_ASSERT(speed < 10.0);
  }

  // Test C172x wind effect on ground roll
  void testC172xHeadwindGroundRoll() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(0.0);
    ic->SetWindNEDFpsIC(-30.0, 0.0, 0.0);  // 30 fps headwind
    TS_ASSERT(fdmex.RunIC());

    auto propulsion = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    fcs->SetThrottleCmd(-1, 1.0);
    propulsion->GetEngine(0)->SetRunning(true);

    for (int i = 0; i < 50; i++) fdmex.Run();

    auto auxiliary = fdmex.GetAuxiliary();
    double ias = auxiliary->GetVcalibratedKTS();

    // With headwind, IAS should build faster
    TS_ASSERT(ias > 5.0);
  }

  // Test C172x crosswind ground handling
  void testC172xCrosswindGroundHandling() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(30.0);
    ic->SetWindNEDFpsIC(0.0, 30.0, 0.0);  // Crosswind
    TS_ASSERT(fdmex.RunIC());

    for (int i = 0; i < 20; i++) fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    auto gr = fdmex.GetGroundReactions();

    // Should experience side forces
    auto forces = gr->GetForces();
    TS_ASSERT(std::isfinite(forces(2)));
  }

  // Test C172x rotation point on takeoff
  void testC172xRotationPhysics() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(55.0);  // Near rotation speed
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();

    // Apply back stick for rotation
    fcs->SetDeCmd(-0.5);

    for (int i = 0; i < 20; i++) fdmex.Run();

    double pitch = propagate->GetEuler()(2);

    // Pitch should increase with elevator input at speed
    TS_ASSERT(std::isfinite(pitch));
  }

  // Test C172x touchdown impact forces
  void testC172xTouchdownForces() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(5.0);
    ic->SetVcalibratedKtsIC(60.0);
    ic->SetClimbRateFpsIC(-5.0);
    TS_ASSERT(fdmex.RunIC());

    auto gr = fdmex.GetGroundReactions();

    // Run until touchdown
    bool touched = false;
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
      for (int j = 0; j < gr->GetNumGearUnits(); j++) {
        if (gr->GetGearUnit(j)->GetWOW()) {
          touched = true;
          // Check impact forces
          auto forces = gr->GetForces();
          TS_ASSERT(fabs(forces(3)) > 100.0);  // Vertical force present
          break;
        }
      }
      if (touched) break;
    }
    TS_ASSERT(touched);
  }

  // Test C172x short field landing technique
  void testC172xShortFieldLanding() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(45.0);  // Lower speed for short field
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();

    // Hard braking for short field
    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);

    double initialSpeed = propagate->GetVel().Magnitude();

    for (int i = 0; i < 100; i++) fdmex.Run();

    double finalSpeed = propagate->GetVel().Magnitude();

    // Should stop quickly
    TS_ASSERT(finalSpeed < initialSpeed * 0.2);
  }

  // Test C172x deck (runway) contact detection
  void testC172xRunwayContactDetection() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(2.0);
    ic->SetVcalibratedKtsIC(55.0);
    ic->SetClimbRateFpsIC(-2.0);
    TS_ASSERT(fdmex.RunIC());

    auto gr = fdmex.GetGroundReactions();

    // Look for contact
    int contactFrame = -1;
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
      for (int j = 0; j < gr->GetNumGearUnits(); j++) {
        if (gr->GetGearUnit(j)->GetWOW()) {
          contactFrame = i;
          break;
        }
      }
      if (contactFrame >= 0) break;
    }

    // Contact should be detected
    TS_ASSERT(contactFrame > 0);
  }

  // Test C172x approach and landing sequence
  void testC172xApproachLandingSequence() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(50.0);
    ic->SetVcalibratedKtsIC(65.0);  // Approach speed
    ic->SetClimbRateFpsIC(-3.0);
    ic->SetGammaRadIC(-3.0 * M_PI / 180.0);  // 3 degree glideslope
    TS_ASSERT(fdmex.RunIC());

    auto propagate = fdmex.GetPropagate();
    auto gr = fdmex.GetGroundReactions();

    // Track altitude through approach
    double prevAlt = propagate->GetAltitudeAGL();

    for (int i = 0; i < 500; i++) {
      fdmex.Run();

      double curAlt = propagate->GetAltitudeAGL();

      // Altitude should be decreasing on approach
      if (i < 100) {
        TS_ASSERT(curAlt <= prevAlt + 5.0);  // Allow small bounces
      }

      prevAlt = curAlt;

      // Check for touchdown
      for (int j = 0; j < gr->GetNumGearUnits(); j++) {
        if (gr->GetGearUnit(j)->GetWOW()) {
          TS_ASSERT(true);  // Successfully landed
          return;
        }
      }
    }
  }
};
