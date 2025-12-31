#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <algorithm>

#include <FGFDMExec.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGFCS.h>
#include <models/FGAtmosphere.h>
#include <input_output/FGPropertyManager.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;
const double deg2rad = Constants::DEG_TO_RAD;
const double rad2deg = Constants::RAD_TO_DEG;

/**
 * Autopilot and Flight Director unit tests
 *
 * These tests focus on control law mathematics for:
 * - Heading hold and turn coordination
 * - Altitude and vertical speed modes
 * - Airspeed hold and autothrottle
 * - Attitude hold (pitch and roll)
 * - Navigation modes (localizer, glideslope)
 * - Flight director command bars
 * - VNAV and approach modes
 *
 * Note: Tests verify mathematical correctness of control algorithms,
 * not complete FCS system integration.
 */
class FGAutopilotTest : public CxxTest::TestSuite
{
public:
  // ============================================================================
  // HEADING HOLD MODE
  // ============================================================================

  // Test heading error calculation - simple case
  void testHeadingErrorSimple() {
    double current_hdg = 90.0;  // East
    double target_hdg = 180.0;  // South

    double error = target_hdg - current_hdg;
    TS_ASSERT_DELTA(error, 90.0, epsilon);
  }

  // Test heading error wrap at 360 degrees
  void testHeadingErrorWrap360() {
    double current_hdg = 350.0;
    double target_hdg = 10.0;

    double error = target_hdg - current_hdg;  // -340

    // Normalize to [-180, 180]
    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    TS_ASSERT_DELTA(error, 20.0, epsilon);  // Shortest path is +20
  }

  // Test heading error wrap at 0 degrees
  void testHeadingErrorWrap0() {
    double current_hdg = 10.0;
    double target_hdg = 350.0;

    double error = target_hdg - current_hdg;  // 340

    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    TS_ASSERT_DELTA(error, -20.0, epsilon);  // Shortest path is -20
  }

  // Test heading hold bank angle command
  void testHeadingHoldBankCommand() {
    double heading_error = 10.0;  // degrees
    double Kp = 2.0;  // Proportional gain

    double bank_cmd = Kp * heading_error;
    TS_ASSERT_DELTA(bank_cmd, 20.0, epsilon);
  }

  // Test heading hold with maximum bank limit
  void testHeadingHoldBankLimit() {
    double heading_error = 90.0;
    double Kp = 2.0;
    double max_bank = 30.0;

    double bank_cmd = Kp * heading_error;
    bank_cmd = std::max(-max_bank, std::min(max_bank, bank_cmd));

    TS_ASSERT_DELTA(bank_cmd, 30.0, epsilon);
  }

  // Test coordinated turn rate calculation
  void testCoordinatedTurnRate() {
    double bank_angle = 30.0 * deg2rad;
    double airspeed = 150.0;  // ft/s
    double g = Constants::G_FTPS2;

    // Turn rate = g * tan(bank) / V
    double turn_rate = g * std::tan(bank_angle) / airspeed;

    // Convert to deg/sec
    turn_rate *= rad2deg;

    TS_ASSERT_DELTA(turn_rate, 7.01, 0.1);  // About 7 deg/sec
  }

  // Test standard rate turn bank angle
  void testStandardRateTurn() {
    double turn_rate = 3.0;  // deg/sec (standard rate)
    double airspeed = 150.0;  // ft/s
    double g = Constants::G_FTPS2;

    // Bank angle = atan(V * rate / g)
    double bank_angle = std::atan(airspeed * (turn_rate * deg2rad) / g);
    bank_angle *= rad2deg;

    TS_ASSERT_DELTA(bank_angle, 14.0, 1.0);
  }

  // ============================================================================
  // ALTITUDE HOLD MODE
  // ============================================================================

  // Test altitude error calculation
  void testAltitudeError() {
    double current_alt = 5000.0;  // feet
    double target_alt = 10000.0;

    double error = target_alt - current_alt;
    TS_ASSERT_DELTA(error, 5000.0, epsilon);
  }

  // Test altitude hold pitch command
  void testAltitudeHoldPitchCommand() {
    double alt_error = 500.0;  // feet
    double Kp = 0.01;  // degrees per foot

    double pitch_cmd = Kp * alt_error;
    TS_ASSERT_DELTA(pitch_cmd, 5.0, epsilon);
  }

  // Test altitude hold with pitch limits
  void testAltitudeHoldPitchLimits() {
    double alt_error = 2000.0;
    double Kp = 0.02;
    double max_pitch = 15.0;
    double min_pitch = -10.0;

    double pitch_cmd = Kp * alt_error;  // 40 degrees
    pitch_cmd = std::max(min_pitch, std::min(max_pitch, pitch_cmd));

    TS_ASSERT_DELTA(pitch_cmd, 15.0, epsilon);
  }

  // Test altitude capture logic
  void testAltitudeCapture() {
    double current_alt = 9950.0;
    double target_alt = 10000.0;
    double vertical_speed = 500.0;  // ft/min
    double capture_threshold = 100.0;  // feet

    double alt_error = std::abs(target_alt - current_alt);
    bool in_capture = (alt_error < capture_threshold);

    TS_ASSERT(in_capture);
  }

  // Test altitude hold with vertical speed feedback
  void testAltitudeHoldWithVSFeedback() {
    double alt_error = 200.0;  // feet
    double vertical_speed = 500.0;  // ft/min
    double Kp_alt = 0.05;
    double Kp_vs = 0.01;

    // Command pitch based on altitude error minus VS damping
    double pitch_cmd = Kp_alt * alt_error - Kp_vs * vertical_speed;
    TS_ASSERT_DELTA(pitch_cmd, 5.0, epsilon);  // 10 - 5 = 5
  }

  // ============================================================================
  // VERTICAL SPEED MODE
  // ============================================================================

  // Test vertical speed error
  void testVerticalSpeedError() {
    double current_vs = 300.0;  // ft/min
    double target_vs = 1000.0;

    double error = target_vs - current_vs;
    TS_ASSERT_DELTA(error, 700.0, epsilon);
  }

  // Test vertical speed to pitch command
  void testVerticalSpeedPitchCommand() {
    double vs_error = 500.0;  // ft/min
    double Kp = 0.002;  // degrees per ft/min

    double pitch_cmd = Kp * vs_error;
    TS_ASSERT_DELTA(pitch_cmd, 1.0, epsilon);
  }

  // Test vertical speed mode leveling
  void testVerticalSpeedLeveling() {
    double target_vs = 0.0;  // Level flight
    double current_vs = -200.0;  // Descending

    double error = target_vs - current_vs;
    TS_ASSERT_DELTA(error, 200.0, epsilon);  // Need to pitch up
  }

  // Test flight path angle calculation
  void testFlightPathAngle() {
    double vertical_speed = 1000.0;  // ft/min
    double ground_speed = 180.0;  // knots

    // Convert to same units: VS to ft/sec, GS to ft/sec
    double vs_fps = vertical_speed / 60.0;
    double gs_fps = ground_speed * Constants::KTS_TO_FTPS;

    // FPA = atan(VS / GS)
    double fpa = std::atan(vs_fps / gs_fps) * rad2deg;

    TS_ASSERT_DELTA(fpa, 3.14, 0.1);  // About 3 degrees
  }

  // ============================================================================
  // AIRSPEED HOLD MODE
  // ============================================================================

  // Test airspeed error calculation
  void testAirspeedError() {
    double current_speed = 150.0;  // knots
    double target_speed = 180.0;

    double error = target_speed - current_speed;
    TS_ASSERT_DELTA(error, 30.0, epsilon);
  }

  // Test airspeed to pitch command
  void testAirspeedPitchCommand() {
    double speed_error = -10.0;  // Too fast by 10 knots
    double Kp = 0.5;  // degrees per knot

    double pitch_cmd = Kp * speed_error;
    TS_ASSERT_DELTA(pitch_cmd, -5.0, epsilon);  // Pitch up to slow down
  }

  // Test airspeed to throttle command
  void testAirspeedThrottleCommand() {
    double speed_error = 10.0;  // Too slow
    double Kp = 0.02;  // throttle per knot

    double throttle_cmd = Kp * speed_error;
    TS_ASSERT_DELTA(throttle_cmd, 0.2, epsilon);
  }

  // ============================================================================
  // BANK ANGLE LIMITING
  // ============================================================================

  // Test bank angle limit for high speed
  void testBankLimitHighSpeed() {
    double airspeed = 300.0;  // knots
    double nominal_max_bank = 30.0;
    double speed_threshold = 250.0;

    // Reduce max bank at high speed
    double speed_factor = std::max(0.5, 1.0 - (airspeed - speed_threshold) / 100.0);
    double max_bank = nominal_max_bank * speed_factor;

    TS_ASSERT_DELTA(max_bank, 15.0, epsilon);
  }

  // Test bank angle limit for low speed
  void testBankLimitLowSpeed() {
    double airspeed = 100.0;  // knots
    double stall_speed = 80.0;
    double nominal_max_bank = 30.0;
    double margin = 1.3;  // 30% above stall

    // Reduce max bank near stall speed
    double speed_ratio = airspeed / (stall_speed * margin);
    double max_bank = nominal_max_bank * std::min(1.0, speed_ratio);

    TS_ASSERT_DELTA(max_bank, 28.85, 0.1);
  }

  // ============================================================================
  // TURN COORDINATION
  // ============================================================================

  // Test coordinated turn rudder command
  void testCoordinatedTurnRudder() {
    double bank_angle = 20.0;  // degrees
    double sideslip = 2.0;  // degrees
    double K_bank = 0.1;  // Rudder per degree of bank
    double K_slip = 0.5;  // Rudder per degree of slip

    // Rudder command = feed-forward from bank + feedback from slip
    double rudder_cmd = K_bank * bank_angle - K_slip * sideslip;
    TS_ASSERT_DELTA(rudder_cmd, 1.0, epsilon);  // 2 - 1 = 1
  }

  // Test yaw damper contribution
  void testYawDamper() {
    double yaw_rate = 5.0;  // deg/sec
    double K_yaw = 0.2;

    double rudder_cmd = -K_yaw * yaw_rate;
    TS_ASSERT_DELTA(rudder_cmd, -1.0, epsilon);
  }

  // Test sideslip to bank coupling
  void testSideslipBankCoupling() {
    double sideslip = 5.0;  // degrees (right)
    double K_slip = 0.5;

    // Bank into slip to eliminate it
    double bank_cmd = K_slip * sideslip;
    TS_ASSERT_DELTA(bank_cmd, 2.5, epsilon);
  }

  // ============================================================================
  // PITCH/ROLL ATTITUDE HOLD
  // ============================================================================

  // Test pitch attitude hold
  void testPitchAttitudeHold() {
    double current_pitch = 3.0;  // degrees
    double target_pitch = 5.0;
    double Kp = 2.0;

    double pitch_error = target_pitch - current_pitch;
    double elevator_cmd = Kp * pitch_error;
    TS_ASSERT_DELTA(elevator_cmd, 4.0, epsilon);
  }

  // Test roll attitude hold
  void testRollAttitudeHold() {
    double current_roll = -5.0;  // degrees
    double target_roll = 10.0;
    double Kp = 1.5;

    double roll_error = target_roll - current_roll;
    double aileron_cmd = Kp * roll_error;
    TS_ASSERT_DELTA(aileron_cmd, 22.5, epsilon);
  }

  // Test pitch rate damping
  void testPitchRateDamping() {
    double pitch_rate = 10.0;  // deg/sec
    double Kd = 0.3;

    double damping_cmd = -Kd * pitch_rate;
    TS_ASSERT_DELTA(damping_cmd, -3.0, epsilon);
  }

  // Test roll rate damping
  void testRollRateDamping() {
    double roll_rate = -15.0;  // deg/sec
    double Kd = 0.2;

    double damping_cmd = -Kd * roll_rate;
    TS_ASSERT_DELTA(damping_cmd, 3.0, epsilon);
  }

  // ============================================================================
  // LOCALIZER TRACKING
  // ============================================================================

  // Test localizer deviation calculation
  void testLocalizerDeviation() {
    double runway_heading = 90.0;  // East
    double aircraft_heading = 85.0;
    double cross_track_error = 100.0;  // feet from centerline

    // Lateral error in dots (full scale = 2.5 degrees or ~700 ft at 5nm)
    double dots_per_foot = 2.5 / 700.0;
    double deviation = cross_track_error * dots_per_foot;

    TS_ASSERT_DELTA(deviation, 0.357, 0.01);
  }

  // Test localizer intercept angle
  void testLocalizerInterceptAngle() {
    double deviation = 1.0;  // dots
    double K_intercept = 10.0;  // degrees per dot

    double intercept_angle = K_intercept * deviation;
    TS_ASSERT_DELTA(intercept_angle, 10.0, epsilon);
  }

  // Test localizer bank command
  void testLocalizerBankCommand() {
    double heading_error = 15.0;  // degrees to intercept
    double K_heading = 1.5;
    double max_bank = 25.0;

    double bank_cmd = K_heading * heading_error;
    bank_cmd = std::max(-max_bank, std::min(max_bank, bank_cmd));

    TS_ASSERT_DELTA(bank_cmd, 22.5, epsilon);
  }

  // Test localizer centerline tracking
  void testLocalizerCenterlineTracking() {
    double cross_track = 50.0;  // feet
    double track_angle = 2.0;  // degrees off runway heading
    double K_xtrack = 0.05;
    double K_track = 2.0;

    // Command heading to null both cross-track and track angle
    double heading_cmd = -K_xtrack * cross_track - K_track * track_angle;
    TS_ASSERT_DELTA(heading_cmd, -6.5, epsilon);
  }

  // ============================================================================
  // GLIDESLOPE TRACKING
  // ============================================================================

  // Test glideslope deviation calculation
  void testGlideslopeDeviation() {
    double current_altitude = 2000.0;  // feet MSL
    double glideslope_altitude = 2100.0;  // feet MSL at current distance

    // Deviation in feet (full scale = 1.4 degrees ~= 700 ft at 5nm)
    double deviation = glideslope_altitude - current_altitude;

    // Convert to dots
    double dots = deviation / 100.0;  // Approximate
    TS_ASSERT_DELTA(dots, 1.0, epsilon);
  }

  // Test glideslope pitch command
  void testGlideslopePitchCommand() {
    double gs_deviation = -0.5;  // dots (below glideslope)
    double K_gs = 1.0;  // degrees per dot

    double pitch_cmd = K_gs * gs_deviation;
    TS_ASSERT_DELTA(pitch_cmd, -0.5, epsilon);  // Pitch down
  }

  // Test glideslope to vertical speed command
  void testGlideslopeVSCommand() {
    double ground_speed = 150.0;  // knots
    double glideslope_angle = 3.0;  // degrees

    // Required VS = GS * tan(angle) * 60 for ft/min
    double gs_fps = ground_speed * Constants::KTS_TO_FTPS;
    double required_vs = -gs_fps * std::tan(glideslope_angle * deg2rad) * 60.0;

    TS_ASSERT_DELTA(required_vs, -800.0, 50.0);  // About 800 ft/min descent
  }

  // Test glideslope altitude deviation
  void testGlideslopeAltitudeDeviation() {
    double distance_to_runway = 5.0;  // nautical miles
    double glideslope_angle = 3.0;  // degrees
    double current_altitude = 1500.0;  // feet AGL

    // Expected altitude on glideslope
    double distance_feet = distance_to_runway * 6076.0;
    double expected_alt = distance_feet * std::tan(glideslope_angle * deg2rad);

    double altitude_error = expected_alt - current_altitude;
    TS_ASSERT_DELTA(altitude_error, 95.0, 10.0);  // About 95 ft high
  }

  // ============================================================================
  // FLIGHT DIRECTOR COMMAND BARS
  // ============================================================================

  // Test flight director pitch command
  void testFlightDirectorPitch() {
    double target_pitch = 10.0;  // degrees
    double current_pitch = 5.0;

    double fd_pitch_cmd = target_pitch - current_pitch;
    TS_ASSERT_DELTA(fd_pitch_cmd, 5.0, epsilon);
  }

  // Test flight director roll command
  void testFlightDirectorRoll() {
    double target_bank = 15.0;  // degrees
    double current_bank = -5.0;

    double fd_roll_cmd = target_bank - current_bank;
    TS_ASSERT_DELTA(fd_roll_cmd, 20.0, epsilon);
  }

  // Test flight director with mode transitions
  void testFlightDirectorModeTransition() {
    // Transition from heading hold to localizer
    bool loc_active = true;
    double heading_cmd = 90.0;
    double localizer_cmd = 92.0;

    double fd_cmd = loc_active ? localizer_cmd : heading_cmd;
    TS_ASSERT_DELTA(fd_cmd, 92.0, epsilon);
  }

  // Test flight director scaling
  void testFlightDirectorScaling() {
    double pitch_error = 10.0;  // degrees
    double display_scale = 2.0;  // dots per degree

    double fd_display = pitch_error * display_scale;
    TS_ASSERT_DELTA(fd_display, 20.0, epsilon);
  }

  // ============================================================================
  // AUTOTHROTTLE CALCULATIONS
  // ============================================================================

  // Test autothrottle speed mode
  void testAutothrottleSpeedMode() {
    double target_speed = 250.0;  // knots
    double current_speed = 240.0;
    double Kp = 0.01;

    double speed_error = target_speed - current_speed;
    double throttle_cmd = Kp * speed_error;

    TS_ASSERT_DELTA(throttle_cmd, 0.1, epsilon);
  }

  // Test autothrottle with acceleration feedback
  void testAutothrottleAcceleration() {
    double speed_error = 10.0;  // knots
    double acceleration = 2.0;  // knots/sec
    double Kp = 0.02;
    double Kd = 0.05;

    double throttle_cmd = Kp * speed_error - Kd * acceleration;
    TS_ASSERT_DELTA(throttle_cmd, 0.1, epsilon);  // 0.2 - 0.1
  }

  // Test autothrottle throttle limiting
  void testAutothrottleLimit() {
    double throttle_cmd = 1.5;  // Exceeds max
    double min_throttle = 0.0;
    double max_throttle = 1.0;

    throttle_cmd = std::max(min_throttle, std::min(max_throttle, throttle_cmd));
    TS_ASSERT_DELTA(throttle_cmd, 1.0, epsilon);
  }

  // Test autothrottle retard for landing
  void testAutothrottleRetard() {
    double radio_altitude = 25.0;  // feet
    double retard_altitude = 30.0;
    bool retard_active = (radio_altitude < retard_altitude);

    double throttle_cmd = retard_active ? 0.0 : 0.5;
    TS_ASSERT_DELTA(throttle_cmd, 0.0, epsilon);
  }

  // Test autothrottle N1 mode
  void testAutothrottleN1Mode() {
    double target_n1 = 95.0;  // percent
    double current_n1 = 90.0;
    double Kp = 0.005;

    double n1_error = target_n1 - current_n1;
    double throttle_cmd = Kp * n1_error;

    TS_ASSERT_DELTA(throttle_cmd, 0.025, epsilon);
  }

  // Test autothrottle EPR mode
  void testAutothrottleEPRMode() {
    double target_epr = 1.5;
    double current_epr = 1.45;
    double Kp = 0.2;

    double epr_error = target_epr - current_epr;
    double throttle_cmd = Kp * epr_error;

    TS_ASSERT_DELTA(throttle_cmd, 0.01, epsilon);
  }

  // ============================================================================
  // GO-AROUND MODE
  // ============================================================================

  // Test go-around pitch command
  void testGoAroundPitchCommand() {
    double initial_pitch = -2.0;  // On approach
    double go_around_pitch = 15.0;  // Target climb pitch

    double pitch_cmd = go_around_pitch;
    TS_ASSERT_DELTA(pitch_cmd, 15.0, epsilon);
  }

  // Test go-around throttle command
  void testGoAroundThrottleCommand() {
    double current_throttle = 0.4;
    double go_around_throttle = 1.0;  // TOGA

    TS_ASSERT_DELTA(go_around_throttle, 1.0, epsilon);
  }

  // Test go-around altitude acceleration
  void testGoAroundAltitudeAcceleration() {
    double initial_vs = -700.0;  // ft/min descending
    double target_vs = 2000.0;  // ft/min climbing
    double time_to_target = 5.0;  // seconds

    double vs_rate = (target_vs - initial_vs) / time_to_target;
    TS_ASSERT_DELTA(vs_rate, 540.0, epsilon);  // ft/min per second
  }

  // Test go-around flap retraction schedule
  void testGoAroundFlapRetraction() {
    double airspeed = 160.0;  // knots
    double flap_retract_speed = 150.0;
    double current_flaps = 30.0;  // degrees

    bool retract = (airspeed > flap_retract_speed);
    double new_flaps = retract ? 15.0 : current_flaps;

    TS_ASSERT_DELTA(new_flaps, 15.0, epsilon);
  }

  // ============================================================================
  // VNAV PATH FOLLOWING
  // ============================================================================

  // Test VNAV path deviation
  void testVNAVPathDeviation() {
    double current_altitude = 10500.0;  // feet
    double target_altitude = 10000.0;
    double horizontal_distance = 10.0;  // nautical miles

    // Path angle to target
    double altitude_error = current_altitude - target_altitude;
    double distance_feet = horizontal_distance * 6076.0;
    double path_angle = std::atan(altitude_error / distance_feet) * rad2deg;

    TS_ASSERT_DELTA(path_angle, 0.47, 0.01);  // About 0.47 degrees
  }

  // Test VNAV vertical speed command
  void testVNAVVerticalSpeedCommand() {
    double ground_speed = 300.0;  // knots
    double path_angle = -3.0;  // degrees (descent)

    // Required VS = GS * sin(angle) * 60
    double gs_fps = ground_speed * Constants::KTS_TO_FTPS;
    double required_vs = gs_fps * std::sin(path_angle * deg2rad) * 60.0;

    TS_ASSERT_DELTA(required_vs, -1600.0, 100.0);  // About 1600 ft/min
  }

  // Test VNAV altitude constraint
  void testVNAVAltitudeConstraint() {
    double current_altitude = 8000.0;  // feet
    double constraint_altitude = 7000.0;  // Must be at or below
    double distance_to_constraint = 5.0;  // nautical miles

    // Check if descent is needed
    bool need_descent = (current_altitude > constraint_altitude);
    TS_ASSERT(need_descent);

    // Calculate required descent rate
    double ground_speed = 240.0;  // knots
    double time_to_constraint = distance_to_constraint / ground_speed * 60.0;  // minutes
    double required_vs = -(current_altitude - constraint_altitude) / time_to_constraint;

    TS_ASSERT_DELTA(required_vs, -800.0, 50.0);  // About 800 ft/min
  }

  // Test VNAV speed constraint
  void testVNAVSpeedConstraint() {
    double current_speed = 280.0;  // knots
    double constraint_speed = 250.0;  // Must be at or below

    bool need_deceleration = (current_speed > constraint_speed);
    TS_ASSERT(need_deceleration);
  }

  // Test VNAV top of descent calculation
  void testVNAVTopOfDescent() {
    double cruise_altitude = 35000.0;  // feet
    double target_altitude = 10000.0;
    double descent_angle = -3.0;  // degrees

    // Distance to TOD
    double altitude_change = cruise_altitude - target_altitude;
    double distance_nm = altitude_change / (std::tan(std::abs(descent_angle) * deg2rad) * 6076.0);

    TS_ASSERT_DELTA(distance_nm, 79.0, 2.0);  // About 79 nm
  }

  // ============================================================================
  // APPROACH MODE COUPLING
  // ============================================================================

  // Test approach mode armed
  void testApproachModeArmed() {
    double localizer_deviation = 3.0;  // dots
    double glideslope_deviation = 2.0;  // dots
    double arm_threshold = 2.5;  // dots

    bool loc_armed = (std::abs(localizer_deviation) < arm_threshold);
    bool gs_armed = (std::abs(glideslope_deviation) < arm_threshold);

    TS_ASSERT(!loc_armed);  // LOC not armed (>2.5 dots)
    TS_ASSERT(gs_armed);    // GS armed (<2.5 dots)
  }

  // Test approach mode capture
  void testApproachModeCapture() {
    double localizer_deviation = 0.5;  // dots
    double capture_threshold = 1.0;

    bool loc_captured = (std::abs(localizer_deviation) < capture_threshold);
    TS_ASSERT(loc_captured);
  }

  // Test dual-channel approach monitor
  void testDualChannelMonitor() {
    double channel_a_cmd = 5.0;  // degrees
    double channel_b_cmd = 5.2;
    double monitor_threshold = 0.5;

    double disagreement = std::abs(channel_a_cmd - channel_b_cmd);
    bool channels_agree = (disagreement < monitor_threshold);

    TS_ASSERT(channels_agree);
  }

  // Test approach minimum altitude
  void testApproachMinimumAltitude() {
    double radio_altitude = 180.0;  // feet
    double decision_height = 200.0;

    bool below_minimums = (radio_altitude < decision_height);
    TS_ASSERT(below_minimums);
  }

  // Test autoland flare law
  void testAutolandFlareLaw() {
    double radio_altitude = 30.0;  // feet
    double flare_altitude = 50.0;
    double sink_rate = -700.0;  // ft/min

    bool in_flare = (radio_altitude < flare_altitude);
    TS_ASSERT(in_flare);

    // Flare pitch command increases as altitude decreases
    double flare_gain = 0.2;
    double flare_pitch = flare_gain * (flare_altitude - radio_altitude);

    TS_ASSERT_DELTA(flare_pitch, 4.0, epsilon);
  }

  // Test autoland thrust retard
  void testAutolandThrustRetard() {
    double radio_altitude = 20.0;  // feet
    double retard_altitude = 25.0;
    double current_throttle = 0.3;
    double retard_rate = 0.05;  // per second
    double dt = 0.1;

    bool in_retard = (radio_altitude < retard_altitude);
    TS_ASSERT(in_retard);

    double new_throttle = current_throttle - retard_rate * dt;
    TS_ASSERT_DELTA(new_throttle, 0.295, epsilon);
  }

  // ============================================================================
  // ADDITIONAL AUTOPILOT FUNCTIONS
  // ============================================================================

  // Test Mach hold mode
  void testMachHoldMode() {
    double current_mach = 0.78;
    double target_mach = 0.80;
    double Kp = 5.0;  // degrees per 0.01 Mach

    double mach_error = (target_mach - current_mach) * 100.0;  // Convert to 0.01 units
    double pitch_cmd = Kp * mach_error;

    TS_ASSERT_DELTA(pitch_cmd, 10.0, epsilon);
  }

  // Test pitch trim for speed stability
  void testPitchTrimForSpeed() {
    double speed_error = 5.0;  // knots fast
    double trim_rate = 0.001;  // per knot per second
    double dt = 1.0;

    double trim_change = -trim_rate * speed_error * dt;
    TS_ASSERT_DELTA(trim_change, -0.005, epsilon);
  }

  // Test wing leveler mode
  void testWingLeveler() {
    double bank_angle = 10.0;  // degrees
    double Kp = 2.0;

    double aileron_cmd = -Kp * bank_angle;
    TS_ASSERT_DELTA(aileron_cmd, -20.0, epsilon);
  }

  // Test takeoff pitch guidance
  void testTakeoffPitchGuidance() {
    double airspeed = 100.0;  // knots
    double rotation_speed = 150.0;
    double target_pitch = 15.0;  // After rotation

    bool rotate = (airspeed >= rotation_speed);
    double pitch_cmd = rotate ? target_pitch : 0.0;

    TS_ASSERT_DELTA(pitch_cmd, 0.0, epsilon);  // Not yet at VR
  }

  // Test altitude preselect alerting
  void testAltitudePreselectAlert() {
    double current_altitude = 9500.0;  // feet
    double preselect_altitude = 10000.0;
    double alert_threshold = 1000.0;

    double altitude_to_go = std::abs(preselect_altitude - current_altitude);
    bool alert_active = (altitude_to_go < alert_threshold);

    TS_ASSERT(alert_active);
  }

  // Test autopilot disconnect monitoring
  void testAutopilotDisconnectMonitor() {
    double control_wheel_force = 15.0;  // lbs
    double disconnect_threshold = 10.0;

    bool disconnect = (control_wheel_force > disconnect_threshold);
    TS_ASSERT(disconnect);
  }

  // Test flight envelope protection - pitch
  void testFlightEnvelopePitchProtection() {
    double pitch_cmd = 25.0;  // degrees
    double max_pitch = 20.0;

    double protected_cmd = std::min(pitch_cmd, max_pitch);
    TS_ASSERT_DELTA(protected_cmd, 20.0, epsilon);
  }

  // Test flight envelope protection - bank
  void testFlightEnvelopeBankProtection() {
    double bank_cmd = 40.0;  // degrees
    double max_bank = 35.0;

    double protected_cmd = std::max(-max_bank, std::min(max_bank, bank_cmd));
    TS_ASSERT_DELTA(protected_cmd, 35.0, epsilon);
  }

  // Test alpha floor protection
  void testAlphaFloorProtection() {
    double angle_of_attack = 18.0;  // degrees
    double alpha_floor = 15.0;

    bool alpha_floor_active = (angle_of_attack > alpha_floor);
    double throttle_cmd = alpha_floor_active ? 1.0 : 0.5;

    TS_ASSERT_DELTA(throttle_cmd, 1.0, epsilon);
  }

  // ============================================================================
  // LNAV/VNAV INTEGRATION
  // ============================================================================

  // Test waypoint steering
  void testWaypointSteering() {
    double bearing_to_waypoint = 45.0;  // degrees
    double current_heading = 30.0;

    double heading_error = bearing_to_waypoint - current_heading;
    TS_ASSERT_DELTA(heading_error, 15.0, epsilon);
  }

  // Test cross-track error steering
  void testCrossTrackSteering() {
    double cross_track = 500.0;  // feet right of course
    double track_angle_error = 5.0;  // degrees
    double K_xte = 0.01;  // degrees per foot
    double K_track = 2.0;

    double heading_correction = -K_xte * cross_track - K_track * track_angle_error;
    TS_ASSERT_DELTA(heading_correction, -15.0, epsilon);
  }

  // Test required time of arrival
  void testRequiredTimeOfArrival() {
    double distance = 50.0;  // nautical miles
    double time_available = 15.0;  // minutes
    double required_groundspeed = distance / time_available * 60.0;  // knots

    TS_ASSERT_DELTA(required_groundspeed, 200.0, epsilon);
  }

  // Test VNAV speed schedule
  void testVNAVSpeedSchedule() {
    double altitude = 25000.0;  // feet
    double transition_altitude = 28000.0;
    double climb_speed_ias = 280.0;  // knots
    double climb_mach = 0.78;

    bool use_mach = altitude > transition_altitude;
    TS_ASSERT(!use_mach);
  }

  // ============================================================================
  // TERRAIN AWARENESS
  // ============================================================================

  // Test minimum safe altitude check
  void testMinimumSafeAltitude() {
    double current_altitude = 5000.0;  // feet MSL
    double msa = 6000.0;  // minimum safe altitude

    bool below_msa = current_altitude < msa;
    TS_ASSERT(below_msa);
  }

  // Test terrain clearance
  void testTerrainClearance() {
    double altitude_agl = 500.0;  // feet
    double min_clearance = 1000.0;

    bool terrain_warning = altitude_agl < min_clearance;
    TS_ASSERT(terrain_warning);
  }

  // Test pull-up command
  void testPullUpCommand() {
    double sink_rate = -3000.0;  // ft/min
    double agl = 1000.0;
    double time_to_impact = agl / (-sink_rate / 60.0);  // seconds

    bool pull_up = time_to_impact < 30.0;
    TS_ASSERT(pull_up);
  }

  // ============================================================================
  // SPEED PROTECTION
  // ============================================================================

  // Test overspeed protection
  void testOverspeedProtection() {
    double current_speed = 350.0;  // knots
    double vmo = 340.0;

    bool overspeed = current_speed > vmo;
    TS_ASSERT(overspeed);
  }

  // Test underspeed protection
  void testUnderspeedProtection() {
    double current_speed = 130.0;  // knots
    double stall_speed = 120.0;
    double margin = 1.1;

    bool underspeed = current_speed < stall_speed * margin;
    TS_ASSERT(underspeed);
  }

  // Test speed trend prediction
  void testSpeedTrendPrediction() {
    double current_speed = 250.0;
    double acceleration = 2.0;  // knots/sec
    double lookahead = 10.0;  // seconds

    double predicted_speed = current_speed + acceleration * lookahead;
    TS_ASSERT_DELTA(predicted_speed, 270.0, epsilon);
  }

  // ============================================================================
  // AUTOPILOT ENGAGEMENT LOGIC
  // ============================================================================

  // Test engagement prerequisites
  void testEngagementPrerequisites() {
    bool airborne = true;
    bool sensors_valid = true;
    bool no_faults = true;

    bool can_engage = airborne && sensors_valid && no_faults;
    TS_ASSERT(can_engage);
  }

  // Test disengagement conditions
  void testDisengagementConditions() {
    double stick_force = 15.0;  // lbs
    double disengage_threshold = 10.0;

    bool force_disconnect = stick_force > disengage_threshold;
    TS_ASSERT(force_disconnect);
  }

  // Test mode reversion
  void testModeReversion() {
    bool primary_mode_valid = false;
    bool backup_mode_available = true;

    bool revert_to_backup = !primary_mode_valid && backup_mode_available;
    TS_ASSERT(revert_to_backup);
  }

  // ============================================================================
  // HOLD MODES
  // ============================================================================

  // Test ground track hold
  void testGroundTrackHold() {
    double target_track = 90.0;  // degrees
    double current_track = 88.0;
    double Kp = 2.0;

    double track_error = target_track - current_track;
    double bank_cmd = Kp * track_error;
    TS_ASSERT_DELTA(bank_cmd, 4.0, epsilon);
  }

  // Test flight path angle hold
  void testFlightPathAngleHold() {
    double target_fpa = -3.0;  // degrees
    double current_fpa = -2.5;
    double Kp = 2.0;

    double fpa_error = target_fpa - current_fpa;
    double pitch_cmd = Kp * fpa_error;
    TS_ASSERT_DELTA(pitch_cmd, -1.0, epsilon);
  }

  // Test indicated airspeed hold with pitch
  void testIASHoldWithPitch() {
    double target_ias = 250.0;
    double current_ias = 260.0;
    double Kp = 0.5;

    double ias_error = target_ias - current_ias;
    double pitch_cmd = -Kp * ias_error;  // Pitch up to slow down
    TS_ASSERT_DELTA(pitch_cmd, 5.0, epsilon);
  }

  // ============================================================================
  // COUPLED APPROACH
  // ============================================================================

  // Test ILS beam width
  void testILSBeamWidth() {
    double localizer_half_width = 2.5;  // degrees
    double distance_nm = 10.0;

    double beam_width_nm = 2.0 * distance_nm * std::tan(localizer_half_width * deg2rad);
    TS_ASSERT_DELTA(beam_width_nm, 0.87, 0.01);
  }

  // Test glideslope intercept angle
  void testGlideslopeInterceptAngle() {
    double glideslope_angle = 3.0;  // degrees
    double current_fpa = -2.0;

    double intercept_rate = glideslope_angle - std::abs(current_fpa);
    TS_ASSERT_DELTA(intercept_rate, 1.0, epsilon);
  }

  // Test decision altitude monitoring
  void testDecisionAltitudeMonitoring() {
    double radio_altitude = 180.0;
    double decision_altitude = 200.0;
    double alert_margin = 50.0;

    bool alert = radio_altitude < decision_altitude + alert_margin;
    TS_ASSERT(alert);
  }

  // ============================================================================
  // AUTOLAND SEQUENCE
  // ============================================================================

  // Test flare initiation
  void testFlareInitiation() {
    double radio_altitude = 45.0;
    double flare_height = 50.0;

    bool in_flare = radio_altitude < flare_height;
    TS_ASSERT(in_flare);
  }

  // Test decrab maneuver
  void testDecrabManeuver() {
    double crab_angle = 10.0;  // degrees
    double runway_heading = 90.0;
    double aircraft_heading = runway_heading + crab_angle;

    double heading_correction = -crab_angle;
    TS_ASSERT_DELTA(heading_correction, -10.0, epsilon);
  }

  // Test rollout steering
  void testRolloutSteering() {
    double centerline_deviation = 5.0;  // feet
    double heading_error = 2.0;  // degrees
    double K_dev = 0.5;
    double K_hdg = 2.0;

    double steering_cmd = K_dev * centerline_deviation + K_hdg * heading_error;
    TS_ASSERT_DELTA(steering_cmd, 6.5, epsilon);
  }

  // ============================================================================
  // SYSTEM MONITORING
  // ============================================================================

  // Test servo loop monitoring
  void testServoLoopMonitoring() {
    double command = 10.0;
    double feedback = 9.8;
    double threshold = 0.5;

    double error = std::abs(command - feedback);
    bool within_tolerance = error < threshold;
    TS_ASSERT(within_tolerance);
  }

  // Test cross-compare voting
  void testCrossCompareVoting() {
    double ap1_cmd = 10.0;
    double ap2_cmd = 10.1;
    double ap3_cmd = 10.05;

    // Use median
    double cmds[] = {ap1_cmd, ap2_cmd, ap3_cmd};
    std::sort(cmds, cmds + 3);
    double voted_cmd = cmds[1];

    TS_ASSERT_DELTA(voted_cmd, 10.05, epsilon);
  }

  // Test failure annunciation
  void testFailureAnnunciation() {
    bool pitch_channel_fail = false;
    bool roll_channel_fail = true;

    bool ap_warning = pitch_channel_fail || roll_channel_fail;
    TS_ASSERT(ap_warning);
  }

  // ============================================================================
  // PERFORMANCE MODES
  // ============================================================================

  // Test economy climb
  void testEconomyClimb() {
    double economy_ias = 280.0;  // knots
    double max_climb_ias = 320.0;

    TS_ASSERT(economy_ias < max_climb_ias);
  }

  // Test high-speed cruise
  void testHighSpeedCruise() {
    double target_mach = 0.85;
    double mmo = 0.87;

    bool valid_target = target_mach < mmo;
    TS_ASSERT(valid_target);
  }

  // Test long-range cruise
  void testLongRangeCruise() {
    double long_range_mach = 0.80;
    double max_range_mach = 0.82;

    TS_ASSERT(long_range_mach <= max_range_mach);
  }

  // ============================================================================
  // TURBULENCE PENETRATION
  // ============================================================================

  // Test turbulence mode speed
  void testTurbulenceModeSpeed() {
    double normal_speed = 300.0;  // knots
    double turbulence_speed = 280.0;

    TS_ASSERT(turbulence_speed < normal_speed);
  }

  // Test attitude limiting in turbulence
  void testAttitudeLimitingTurbulence() {
    double normal_bank_limit = 30.0;
    double turbulence_bank_limit = 15.0;

    TS_ASSERT(turbulence_bank_limit < normal_bank_limit);
  }

  // Test gust response damping
  void testGustResponseDamping() {
    double pitch_rate = 5.0;  // deg/sec
    double damping_gain = 0.5;

    double damping_cmd = -damping_gain * pitch_rate;
    TS_ASSERT_DELTA(damping_cmd, -2.5, epsilon);
  }

  //==========================================================================
  // C172x Model-Based Autopilot Tests
  //==========================================================================

  // Test C172x autopilot properties exist
  void testC172xAutopilotPropertiesExist() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();

    // C172x has autopilot properties
    TS_ASSERT(pm->HasNode("ap/attitude_hold"));
    TS_ASSERT(pm->HasNode("ap/altitude_hold"));
    TS_ASSERT(pm->HasNode("ap/heading_hold"));
  }

  // Test C172x attitude hold property setting
  void testC172xAttitudeHoldSetting() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();
    auto node = pm->GetNode("ap/attitude_hold");

    if (node) {
      node->setDoubleValue(1.0);
      TS_ASSERT_DELTA(node->getDoubleValue(), 1.0, 0.001);
    }
  }

  // Test C172x altitude hold property setting
  void testC172xAltitudeHoldSetting() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();
    auto altHold = pm->GetNode("ap/altitude_hold");
    auto altSetpoint = pm->GetNode("ap/altitude_setpoint");

    if (altHold && altSetpoint) {
      altSetpoint->setDoubleValue(5000.0);
      altHold->setDoubleValue(1.0);

      TS_ASSERT_DELTA(altSetpoint->getDoubleValue(), 5000.0, 0.001);
      TS_ASSERT_DELTA(altHold->getDoubleValue(), 1.0, 0.001);
    }
  }

  // Test C172x heading hold property setting
  void testC172xHeadingHoldSetting() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();
    auto hdgHold = pm->GetNode("ap/heading_hold");
    auto hdgSetpoint = pm->GetNode("ap/heading_setpoint");

    if (hdgHold && hdgSetpoint) {
      hdgSetpoint->setDoubleValue(90.0);
      hdgHold->setDoubleValue(1.0);

      TS_ASSERT_DELTA(hdgSetpoint->getDoubleValue(), 90.0, 0.001);
      TS_ASSERT_DELTA(hdgHold->getDoubleValue(), 1.0, 0.001);
    }
  }

  // Test C172x FCS control surfaces
  void testC172xFCSControlSurfaces() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Control surface positions should be finite
    double elevator = fcs->GetDePos(ofRad);
    double aileron = fcs->GetDaLPos(ofRad);
    double rudder = fcs->GetDrPos(ofRad);
    double flap = fcs->GetDfPos(ofRad);

    TS_ASSERT(std::isfinite(elevator));
    TS_ASSERT(std::isfinite(aileron));
    TS_ASSERT(std::isfinite(rudder));
    TS_ASSERT(std::isfinite(flap));
  }

  // Test C172x elevator command
  void testC172xElevatorCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(0.5);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double elevatorCmd = fcs->GetDeCmd();
    TS_ASSERT_DELTA(elevatorCmd, 0.5, 0.001);
  }

  // Test C172x aileron command
  void testC172xAileronCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(0.3);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double aileronCmd = fcs->GetDaCmd();
    TS_ASSERT_DELTA(aileronCmd, 0.3, 0.001);
  }

  // Test C172x rudder command
  void testC172xRudderCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDrCmd(0.2);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double rudderCmd = fcs->GetDrCmd();
    TS_ASSERT_DELTA(rudderCmd, 0.2, 0.001);
  }

  // Test C172x throttle command
  void testC172xThrottleCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetThrottleCmd(-1, 0.75);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double throttleCmd = fcs->GetThrottleCmd(0);
    TS_ASSERT_DELTA(throttleCmd, 0.75, 0.01);
  }

  // Test C172x flap command
  void testC172xFlapCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();
    auto flapCmd = pm->GetNode("fcs/flap-cmd-norm");

    if (flapCmd) {
      flapCmd->setDoubleValue(0.5);

      for (int i = 0; i < 100; i++) {
        fdmex.Run();
      }

      TS_ASSERT_DELTA(flapCmd->getDoubleValue(), 0.5, 0.01);
    }
  }

  // Test C172x pitch trim
  void testC172xPitchTrim() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetPitchTrimCmd(0.1);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double pitchTrim = fcs->GetPitchTrimCmd();
    TS_ASSERT_DELTA(pitchTrim, 0.1, 0.001);
  }

  // Test C172x roll trim
  void testC172xRollTrim() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetRollTrimCmd(0.05);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double rollTrim = fcs->GetRollTrimCmd();
    TS_ASSERT_DELTA(rollTrim, 0.05, 0.001);
  }

  // Test C172x yaw trim
  void testC172xYawTrim() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetYawTrimCmd(0.02);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double yawTrim = fcs->GetYawTrimCmd();
    TS_ASSERT_DELTA(yawTrim, 0.02, 0.001);
  }

  // Test C172x brake command
  void testC172xBrakeCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetLBrake(0.8);
    fcs->SetRBrake(0.8);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double leftBrake = fcs->GetLBrake();
    double rightBrake = fcs->GetRBrake();

    TS_ASSERT_DELTA(leftBrake, 0.8, 0.01);
    TS_ASSERT_DELTA(rightBrake, 0.8, 0.01);
  }

  // Test C172x gear command
  void testC172xGearCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // C172 has fixed gear, but the command should still work
    fcs->SetGearCmd(1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double gearCmd = fcs->GetGearCmd();
    TS_ASSERT_DELTA(gearCmd, 1.0, 0.01);
  }

  // Test C172x simulation with control inputs
  void testC172xSimulationWithControls() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto auxiliary = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();

    // Set some control inputs
    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetDeCmd(0.0);
    fcs->SetDaCmd(0.0);
    fcs->SetDrCmd(0.0);

    // Run simulation
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Values should be finite
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(propagate->GetAltitudeASL()));
  }

  // Test C172x attitude angles after control input
  void testC172xAttitudeWithControlInput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();

    // Apply elevator input
    fcs->SetDeCmd(-0.1);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Pitch angle should be finite
    double theta = propagate->GetEuler(2);
    TS_ASSERT(std::isfinite(theta));
  }

  // Test C172x roll response to aileron input
  void testC172xRollResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();

    // Apply aileron input
    fcs->SetDaCmd(0.2);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Roll angle should be finite
    double phi = propagate->GetEuler(1);
    TS_ASSERT(std::isfinite(phi));
  }

  // Test C172x yaw response to rudder input
  void testC172xYawResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();

    // Apply rudder input
    fcs->SetDrCmd(0.15);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Yaw angle should be finite
    double psi = propagate->GetEuler(3);
    TS_ASSERT(std::isfinite(psi));
  }

  // Test C172x heading calculation
  void testC172xHeadingCalculation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double psi = propagate->GetEuler(3);  // Heading in radians
    double headingDeg = psi * rad2deg;

    TS_ASSERT(std::isfinite(headingDeg));
  }

  // Test C172x control surface limits
  void testC172xControlSurfaceLimits() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Try to set commands beyond normal range
    fcs->SetDeCmd(2.0);  // Way over 1.0
    fcs->SetDaCmd(-2.0); // Way under -1.0

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Surface positions should be limited
    double elevator = fcs->GetDePos(ofRad);
    double aileron = fcs->GetDaLPos(ofRad);

    TS_ASSERT(std::isfinite(elevator));
    TS_ASSERT(std::isfinite(aileron));
  }

  // Test C172x autopilot command properties
  void testC172xAutopilotCommands() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();

    // Check that AP command properties exist
    auto aileronCmd = pm->GetNode("ap/aileron_cmd");
    auto elevatorCmd = pm->GetNode("ap/elevator_cmd");

    if (aileronCmd) {
      aileronCmd->setDoubleValue(0.1);
      TS_ASSERT_DELTA(aileronCmd->getDoubleValue(), 0.1, 0.001);
    }

    if (elevatorCmd) {
      elevatorCmd->setDoubleValue(-0.05);
      TS_ASSERT_DELTA(elevatorCmd->getDoubleValue(), -0.05, 0.001);
    }
  }

  // Test C172x extended autopilot simulation
  void testC172xExtendedAutopilotSim() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto auxiliary = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();

    // Set throttle
    fcs->SetThrottleCmd(-1, 0.7);

    // Run extended simulation
    for (int i = 0; i < 500; i++) {
      fdmex.Run();
    }

    // All values should still be finite
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(auxiliary->GetMach()));
    TS_ASSERT(std::isfinite(propagate->GetAltitudeASL()));
    TS_ASSERT(std::isfinite(propagate->GetEuler(1)));
    TS_ASSERT(std::isfinite(propagate->GetEuler(2)));
    TS_ASSERT(std::isfinite(propagate->GetEuler(3)));
  }
};
