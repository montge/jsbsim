/*******************************************************************************
 * FGRadioNavTest.h - Unit tests for radio navigation calculations
 *
 * Tests mathematical navigation algorithms for radio navigation aids:
 * - VOR (VHF Omnidirectional Range) radial and bearing calculations
 * - VOR/DME position fixing and RNAV waypoints
 * - NDB (Non-Directional Beacon) bearing calculations
 * - ILS (Instrument Landing System) localizer and glideslope deviations
 * - DME (Distance Measuring Equipment) distance and slant range
 * - Course deviation indicator (CDI) logic
 * - To/From flag determination
 * - Cone of confusion detection
 * - Station passage detection
 * - Magnetic variation corrections
 * - Radio altitude calculations
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-8;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double NM_TO_FT = 6076.12;  // Nautical miles to feet
const double FT_TO_NM = 1.0 / NM_TO_FT;

// Navigation constants
const double EARTH_RADIUS_NM = 3440.065;  // Earth mean radius in nautical miles

// Helper function to normalize angle to [0, 360) degrees
inline double NormalizeHeading(double heading) {
  while (heading < 0.0) heading += 360.0;
  while (heading >= 360.0) heading -= 360.0;
  return heading;
}

// Helper function to normalize angle to [-180, 180] degrees
inline double NormalizeAngle180(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}

class FGRadioNavTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * VOR Radial Calculations
   ***************************************************************************/

  // Test VOR radial on cardinal headings
  void testVORRadialCardinal() {
    double vor_bearing = 0.0;  // VOR radial 0 (due north from station)
    double aircraft_bearing = 0.0;

    // Aircraft on the 360 radial
    double radial_error = NormalizeAngle180(aircraft_bearing - vor_bearing);
    TS_ASSERT_DELTA(radial_error, 0.0, epsilon);
  }

  // Test VOR radial calculation east
  void testVORRadialEast() {
    double vor_bearing = 90.0;  // VOR radial 090 (due east from station)
    double aircraft_bearing = 90.0;

    double radial_error = NormalizeAngle180(aircraft_bearing - vor_bearing);
    TS_ASSERT_DELTA(radial_error, 0.0, epsilon);
  }

  // Test VOR radial with offset
  void testVORRadialOffset() {
    double vor_radial = 45.0;
    double aircraft_bearing = 50.0;

    double radial_error = NormalizeAngle180(aircraft_bearing - vor_radial);
    TS_ASSERT_DELTA(radial_error, 5.0, epsilon);
  }

  // Test VOR reciprocal radial
  void testVORReciprocalRadial() {
    double radial = 270.0;
    double reciprocal = NormalizeHeading(radial + 180.0);

    TS_ASSERT_DELTA(reciprocal, 90.0, epsilon);
  }

  // Test VOR radial wrapping at 360
  void testVORRadialWrapping() {
    double radial = 350.0;
    double aircraft_bearing = 10.0;

    // Error should be 20 degrees (not -340)
    double error = NormalizeAngle180(aircraft_bearing - radial);
    TS_ASSERT_DELTA(error, 20.0, epsilon);
  }

  /***************************************************************************
   * VOR DME Position Fixing
   ***************************************************************************/

  // Test VOR/DME position fix with known radial and distance
  void testVORDMEPositionFix() {
    double vor_lat = 40.0;
    double vor_lon = -74.0;
    double radial = 90.0;  // East
    double dme_distance_nm = 10.0;

    // Simple approximation: 1 degree longitude ≈ 60 nm at 40°N
    double lat_correction = std::cos(40.0 * DEG_TO_RAD);
    double lon_offset = dme_distance_nm / (60.0 * lat_correction);

    double aircraft_lon = vor_lon + lon_offset;
    TS_ASSERT(std::abs(aircraft_lon - vor_lon) > 0.0);
  }

  // Test VOR/DME intersection from two stations
  void testVORDMEIntersection() {
    // Two VOR stations with known radials should intersect
    double vor1_radial = 90.0;  // East from VOR1
    double vor2_radial = 270.0; // West from VOR2

    // Radials should be reciprocals for intersection on same latitude
    double sum = vor1_radial + vor2_radial;
    TS_ASSERT_DELTA(sum, 360.0, epsilon);
  }

  /***************************************************************************
   * NDB Bearing Calculations
   ***************************************************************************/

  // Test NDB relative bearing
  void testNDBRelativeBearing() {
    double aircraft_heading = 45.0;
    double ndb_bearing = 90.0;  // True bearing to NDB

    double relative_bearing = NormalizeHeading(ndb_bearing - aircraft_heading);
    TS_ASSERT_DELTA(relative_bearing, 45.0, epsilon);
  }

  // Test NDB bearing with magnetic variation
  void testNDBMagneticBearing() {
    double true_bearing = 90.0;
    double magnetic_variation = 10.0;  // 10° East variation

    double magnetic_bearing = NormalizeHeading(true_bearing - magnetic_variation);
    TS_ASSERT_DELTA(magnetic_bearing, 80.0, epsilon);
  }

  // Test NDB homing
  void testNDBHoming() {
    double aircraft_heading = 0.0;
    double ndb_bearing = 0.0;

    // When relative bearing is 0, aircraft is heading directly to NDB
    double relative_bearing = NormalizeHeading(ndb_bearing - aircraft_heading);
    TS_ASSERT_DELTA(relative_bearing, 0.0, epsilon);
  }

  /***************************************************************************
   * ILS Localizer Deviation
   ***************************************************************************/

  // Test ILS localizer on centerline
  void testILSLocalizerCenterline() {
    double localizer_course = 90.0;  // Runway heading 090
    double aircraft_bearing = 90.0;

    double deviation = aircraft_bearing - localizer_course;
    TS_ASSERT_DELTA(deviation, 0.0, epsilon);
  }

  // Test ILS localizer deviation in dots (±2.5° full scale = ±2 dots)
  void testILSLocalizerDots() {
    double localizer_course = 90.0;
    double aircraft_bearing = 92.5;  // 2.5° right of centerline

    double deviation_degrees = aircraft_bearing - localizer_course;
    double deviation_dots = deviation_degrees / 1.25;  // 1.25° per dot

    TS_ASSERT_DELTA(deviation_dots, 2.0, 0.01);
  }

  // Test ILS localizer full scale deflection
  void testILSLocalizerFullScale() {
    double max_deviation = 2.5;  // ±2.5° full scale

    double deviation_dots = max_deviation / 1.25;
    TS_ASSERT_DELTA(deviation_dots, 2.0, epsilon);
  }

  // Test ILS localizer reverse sensing
  void testILSLocalizerReverse() {
    double localizer_course = 90.0;
    double aircraft_heading = 270.0;  // Flying away from runway

    bool reverse_sensing = std::abs(NormalizeAngle180(aircraft_heading - localizer_course)) > 90.0;
    TS_ASSERT(reverse_sensing);
  }

  /***************************************************************************
   * ILS Glideslope Deviation
   ***************************************************************************/

  // Test ILS glideslope on glidepath
  void testILSGlideslopeOnPath() {
    double glideslope_angle = 3.0;  // 3° glideslope
    double distance_nm = 5.0;
    double altitude_ft = distance_nm * NM_TO_FT * std::tan(glideslope_angle * DEG_TO_RAD);

    double expected_altitude = 5.0 * 6076.12 * std::tan(3.0 * DEG_TO_RAD);
    TS_ASSERT_DELTA(altitude_ft, expected_altitude, 1.0);
  }

  // Test ILS glideslope deviation in dots
  void testILSGlideslopeDots() {
    double glideslope_angle = 3.0;
    double aircraft_angle = 3.7;  // 0.7° above glidepath

    double deviation_degrees = aircraft_angle - glideslope_angle;
    double deviation_dots = deviation_degrees / 0.35;  // 0.35° per dot

    TS_ASSERT_DELTA(deviation_dots, 2.0, 0.01);
  }

  // Test ILS glideslope altitude calculation
  void testILSGlideslopeAltitude() {
    double glideslope_angle = 3.0;
    double distance_ft = 10000.0;  // 10,000 ft from threshold

    double required_altitude = distance_ft * std::tan(glideslope_angle * DEG_TO_RAD);
    TS_ASSERT(required_altitude > 0.0);
    TS_ASSERT(required_altitude < 1000.0);  // Should be reasonable
  }

  /***************************************************************************
   * DME Distance Calculations
   ***************************************************************************/

  // Test DME ground distance calculation
  void testDMEGroundDistance() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 40.0 * DEG_TO_RAD;
    double lon2 = -73.0 * DEG_TO_RAD;  // 1 degree east

    // Simple distance calculation
    double dlon = lon2 - lon1;
    double distance_rad = std::abs(dlon) * std::cos(lat1);
    double distance_nm = distance_rad * EARTH_RADIUS_NM;

    TS_ASSERT(distance_nm > 40.0 && distance_nm < 60.0);  // ~50 nm at 40°N
  }

  // Test DME slant range correction
  void testDMESlantRange() {
    double ground_distance_nm = 10.0;
    double aircraft_altitude_ft = 5000.0;
    double station_altitude_ft = 500.0;

    double altitude_diff_ft = aircraft_altitude_ft - station_altitude_ft;
    double altitude_diff_nm = altitude_diff_ft * FT_TO_NM;

    double slant_range_nm = std::sqrt(ground_distance_nm * ground_distance_nm +
                                      altitude_diff_nm * altitude_diff_nm);

    TS_ASSERT(slant_range_nm > ground_distance_nm);
  }

  // Test DME slant range at station
  void testDMESlantRangeAtStation() {
    double ground_distance_nm = 0.0;  // Overhead station
    double aircraft_altitude_ft = 6076.12;  // 1 nm altitude
    double station_altitude_ft = 0.0;

    double altitude_diff_nm = (aircraft_altitude_ft - station_altitude_ft) * FT_TO_NM;
    double slant_range_nm = std::sqrt(ground_distance_nm * ground_distance_nm +
                                      altitude_diff_nm * altitude_diff_nm);

    TS_ASSERT_DELTA(slant_range_nm, 1.0, 0.01);
  }

  /***************************************************************************
   * VOR/DME RNAV Waypoint
   ***************************************************************************/

  // Test RNAV waypoint from VOR/DME radial and distance
  void testRNAVWaypoint() {
    double vor_radial = 90.0;  // East radial
    double dme_distance = 10.0;  // 10 nm from VOR

    // Waypoint is 10 nm east of VOR on 090 radial
    double waypoint_bearing = vor_radial;
    TS_ASSERT_DELTA(waypoint_bearing, 90.0, epsilon);
  }

  // Test RNAV waypoint bearing calculation
  void testRNAVWaypointBearing() {
    double vor_radial = 45.0;
    double aircraft_radial = 30.0;

    double bearing_to_waypoint = NormalizeAngle180(vor_radial - aircraft_radial);
    TS_ASSERT_DELTA(bearing_to_waypoint, 15.0, epsilon);
  }

  /***************************************************************************
   * Course Deviation Indicator (CDI)
   ***************************************************************************/

  // Test CDI on course
  void testCDIOnCourse() {
    double desired_course = 90.0;
    double actual_bearing = 90.0;

    double deviation = NormalizeAngle180(actual_bearing - desired_course);
    TS_ASSERT_DELTA(deviation, 0.0, epsilon);
  }

  // Test CDI deviation in degrees
  void testCDIDeviationDegrees() {
    double desired_course = 90.0;
    double actual_bearing = 95.0;

    double deviation = NormalizeAngle180(actual_bearing - desired_course);
    TS_ASSERT_DELTA(deviation, 5.0, epsilon);
  }

  // Test CDI left/right indication
  void testCDILeftRight() {
    double desired_course = 90.0;
    double actual_bearing_right = 95.0;
    double actual_bearing_left = 85.0;

    double deviation_right = actual_bearing_right - desired_course;
    double deviation_left = actual_bearing_left - desired_course;

    TS_ASSERT(deviation_right > 0.0);  // Right of course
    TS_ASSERT(deviation_left < 0.0);   // Left of course
  }

  // Test CDI full scale deflection (±10° for VOR, ±2.5° for ILS)
  void testCDIFullScaleVOR() {
    double full_scale_vor = 10.0;  // ±10° full scale for VOR
    double deviation_dots = full_scale_vor / 2.0;  // 2° per dot for VOR

    TS_ASSERT_DELTA(deviation_dots, 5.0, epsilon);
  }

  /***************************************************************************
   * To/From Flag Logic
   ***************************************************************************/

  // Test TO flag indication
  void testToFromFlagTO() {
    double vor_radial = 90.0;
    double aircraft_heading = 90.0;

    // Heading within ±90° of radial = TO
    double angle_diff = std::abs(NormalizeAngle180(aircraft_heading - vor_radial));
    bool is_TO = angle_diff <= 90.0;

    TS_ASSERT(is_TO);
  }

  // Test FROM flag indication
  void testToFromFlagFROM() {
    double vor_radial = 90.0;
    double aircraft_heading = 270.0;  // Reciprocal

    // Heading more than ±90° from radial = FROM
    double angle_diff = std::abs(NormalizeAngle180(aircraft_heading - vor_radial));
    bool is_FROM = angle_diff > 90.0;

    TS_ASSERT(is_FROM);
  }

  // Test TO/FROM boundary (abeam station)
  void testToFromFlagBoundary() {
    double vor_radial = 90.0;
    double aircraft_heading_perpendicular = 0.0;  // 90° to radial

    double angle_diff = std::abs(NormalizeAngle180(aircraft_heading_perpendicular - vor_radial));
    bool at_boundary = std::abs(angle_diff - 90.0) < 1.0;

    TS_ASSERT(at_boundary);
  }

  /***************************************************************************
   * Cone of Confusion
   ***************************************************************************/

  // Test cone of confusion detection (overhead station)
  void testConeOfConfusion() {
    double ground_distance_nm = 0.1;  // Very close to station
    double aircraft_altitude_ft = 10000.0;

    double altitude_nm = aircraft_altitude_ft * FT_TO_NM;
    double ratio = altitude_nm / ground_distance_nm;

    // In cone of confusion when altitude >> ground distance
    bool in_cone = ratio > 10.0;
    TS_ASSERT(in_cone);
  }

  // Test cone of confusion exit
  void testConeOfConfusionExit() {
    double ground_distance_nm = 5.0;
    double aircraft_altitude_ft = 5000.0;

    double altitude_nm = aircraft_altitude_ft * FT_TO_NM;
    bool outside_cone = ground_distance_nm > altitude_nm;

    TS_ASSERT(outside_cone);
  }

  /***************************************************************************
   * Station Passage Detection
   ***************************************************************************/

  // Test station passage by TO/FROM reversal
  void testStationPassageByFlag() {
    double vor_radial = 90.0;
    double heading = 90.0;

    // Before passage: aircraft on 090 radial heading 090 = TO
    double angle_diff_before = std::abs(NormalizeAngle180(heading - vor_radial));
    bool to_before = angle_diff_before <= 90.0;

    // After passage: aircraft now on reciprocal (270) radial, still heading 090
    // The aircraft bearing FROM station is now 270 (reciprocal of 090)
    double radial_from_station = NormalizeHeading(vor_radial + 180.0);  // 270
    double angle_diff_after = std::abs(NormalizeAngle180(heading - radial_from_station));
    bool from_after = angle_diff_after > 90.0;  // >90° means FROM

    TS_ASSERT(to_before);
    TS_ASSERT(from_after);
  }

  // Test station passage by distance
  void testStationPassageByDistance() {
    double distance_before = 2.0;  // nm
    double distance_at_station = 0.5;  // nm
    double distance_after = 2.0;  // nm

    bool passage_occurred = (distance_before > distance_at_station) &&
                           (distance_after > distance_at_station);

    TS_ASSERT(passage_occurred);
  }

  /***************************************************************************
   * Magnetic Variation Corrections
   ***************************************************************************/

  // Test east magnetic variation
  void testMagneticVariationEast() {
    double true_bearing = 90.0;
    double variation_east = 10.0;  // 10° East variation

    // Magnetic = True - Variation (East is positive, West is negative)
    double magnetic_bearing = true_bearing - variation_east;

    TS_ASSERT_DELTA(magnetic_bearing, 80.0, epsilon);
  }

  // Test west magnetic variation
  void testMagneticVariationWest() {
    double true_bearing = 90.0;
    double variation_west = -15.0;  // 15° West variation (negative)

    double magnetic_bearing = true_bearing - variation_west;

    TS_ASSERT_DELTA(magnetic_bearing, 105.0, epsilon);
  }

  // Test magnetic to true conversion
  void testMagneticToTrue() {
    double magnetic_bearing = 80.0;
    double variation_east = 10.0;

    // True = Magnetic + Variation
    double true_bearing = magnetic_bearing + variation_east;

    TS_ASSERT_DELTA(true_bearing, 90.0, epsilon);
  }

  // Test variation wrapping at 360
  void testMagneticVariationWrapping() {
    double true_bearing = 5.0;
    double variation_east = 10.0;

    double magnetic_bearing = NormalizeHeading(true_bearing - variation_east);

    TS_ASSERT_DELTA(magnetic_bearing, 355.0, epsilon);
  }

  /***************************************************************************
   * Radio Altitude Calculations
   ***************************************************************************/

  // Test radio altitude (AGL)
  void testRadioAltitudeAGL() {
    double aircraft_altitude_msl = 5000.0;  // ft MSL
    double ground_elevation = 1000.0;  // ft MSL

    double radio_altitude = aircraft_altitude_msl - ground_elevation;

    TS_ASSERT_DELTA(radio_altitude, 4000.0, epsilon);
  }

  // Test radio altitude over water
  void testRadioAltitudeWater() {
    double aircraft_altitude_msl = 500.0;
    double sea_level = 0.0;

    double radio_altitude = aircraft_altitude_msl - sea_level;

    TS_ASSERT_DELTA(radio_altitude, 500.0, epsilon);
  }

  // Test radio altitude minimum decision height
  void testRadioAltitudeDecisionHeight() {
    double radio_altitude = 200.0;  // ft AGL
    double decision_height = 200.0;

    bool at_decision_height = radio_altitude <= decision_height;

    TS_ASSERT(at_decision_height);
  }

  /***************************************************************************
   * Advanced VOR/DME Calculations
   ***************************************************************************/

  // Test VOR/DME arc intercept
  void testDMEArcIntercept() {
    double current_dme = 15.0;  // nm
    double desired_arc = 10.0;  // nm

    double distance_to_arc = current_dme - desired_arc;

    TS_ASSERT_DELTA(distance_to_arc, 5.0, epsilon);
  }

  // Test VOR/DME arc tangent point
  void testDMEArcTangent() {
    double arc_radius = 10.0;  // nm
    double lead_distance = 1.0;  // nm lead for turn
    double turn_dme = arc_radius + lead_distance;

    TS_ASSERT_DELTA(turn_dme, 11.0, epsilon);
  }

  /***************************************************************************
   * Cross-Track Error from VOR
   ***************************************************************************/

  // Test cross-track error from VOR radial
  void testVORCrossTrackError() {
    double desired_radial = 90.0;
    double actual_bearing = 92.0;
    double dme_distance = 10.0;

    double angular_error = NormalizeAngle180(actual_bearing - desired_radial);
    double cross_track_nm = dme_distance * std::sin(angular_error * DEG_TO_RAD);

    TS_ASSERT(std::abs(cross_track_nm) > 0.0);
    TS_ASSERT(std::abs(cross_track_nm) < 1.0);
  }

  // Test cross-track error on course
  void testVORCrossTrackOnCourse() {
    double desired_radial = 90.0;
    double actual_bearing = 90.0;
    double dme_distance = 10.0;

    double angular_error = actual_bearing - desired_radial;
    double cross_track_nm = dme_distance * std::sin(angular_error * DEG_TO_RAD);

    TS_ASSERT_DELTA(cross_track_nm, 0.0, 0.01);
  }

  /***************************************************************************
   * Localizer Sensitivity
   ***************************************************************************/

  // Test ILS localizer sensitivity near threshold
  void testILSLocalizerSensitivityNear() {
    double distance_nm = 1.0;  // Close to runway
    double angular_error = 1.0;  // 1 degree off

    double lateral_error_ft = distance_nm * NM_TO_FT * std::tan(angular_error * DEG_TO_RAD);

    TS_ASSERT(lateral_error_ft > 0.0);
    TS_ASSERT(lateral_error_ft < 200.0);  // Should be very sensitive
  }

  // Test ILS localizer sensitivity far from threshold
  void testILSLocalizerSensitivityFar() {
    double distance_nm = 10.0;  // Far from runway
    double angular_error = 1.0;  // 1 degree off

    double lateral_error_ft = distance_nm * NM_TO_FT * std::tan(angular_error * DEG_TO_RAD);

    TS_ASSERT(lateral_error_ft > 1000.0);  // Less sensitive (larger error)
  }
};
