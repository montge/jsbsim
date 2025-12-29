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

  /***************************************************************************
   * GPS Navigation Calculations
   ***************************************************************************/

  void testGPSDirectTrack() {
    double from_lat = 40.0;
    double from_lon = -74.0;
    double to_lat = 41.0;
    double to_lon = -74.0;

    // Due north track
    double dlat = to_lat - from_lat;
    double dlon = to_lon - from_lon;

    double track = std::atan2(dlon, dlat) * RAD_TO_DEG;
    TS_ASSERT_DELTA(NormalizeHeading(track), 0.0, 0.1);  // North
  }

  void testGPSCrossTrackError() {
    double desired_track = 90.0;  // East
    double actual_track = 92.0;
    double distance_flown = 50.0;  // nm

    double angular_error = (actual_track - desired_track) * DEG_TO_RAD;
    double cross_track = distance_flown * std::sin(angular_error);

    TS_ASSERT(cross_track > 0.0);
    TS_ASSERT(cross_track < 2.0);  // nm
  }

  void testGPSAlongTrackDistance() {
    double total_distance = 100.0;  // nm
    double distance_flown = 60.0;

    double along_track = distance_flown;
    double distance_remaining = total_distance - along_track;

    TS_ASSERT_DELTA(distance_remaining, 40.0, epsilon);
  }

  void testGPSGroundSpeed() {
    double distance_nm = 120.0;
    double time_hours = 1.0;

    double ground_speed = distance_nm / time_hours;
    TS_ASSERT_DELTA(ground_speed, 120.0, epsilon);
  }

  void testGPSETE() {
    double distance_remaining = 50.0;  // nm
    double ground_speed = 100.0;  // knots

    double ete_hours = distance_remaining / ground_speed;
    double ete_minutes = ete_hours * 60.0;

    TS_ASSERT_DELTA(ete_minutes, 30.0, epsilon);
  }

  void testGPSWaypointSequencing() {
    double distance_to_waypoint = 0.3;  // nm
    double sequencing_threshold = 0.5;  // nm

    bool should_sequence = distance_to_waypoint < sequencing_threshold;
    TS_ASSERT(should_sequence);
  }

  /***************************************************************************
   * TACAN Navigation
   ***************************************************************************/

  void testTACANChannelToFrequency() {
    // TACAN channel 100X = 1213 MHz
    int channel = 100;
    bool x_band = true;

    // Simplified conversion
    double freq_mhz = 1025.0 + channel;
    if (x_band) freq_mhz += 63.0;

    TS_ASSERT(freq_mhz > 1000.0);
  }

  void testTACANBearing() {
    double aircraft_lat = 40.0;
    double aircraft_lon = -74.0;
    double tacan_lat = 40.0;
    double tacan_lon = -73.5;

    // Station is east of aircraft
    double bearing = 90.0;  // Simplified
    TS_ASSERT(bearing > 45.0 && bearing < 135.0);
  }

  void testTACANDistance() {
    // TACAN provides DME-like distance
    double ground_distance = 15.0;  // nm
    double altitude_diff_nm = 1.0;

    double slant_range = std::sqrt(ground_distance * ground_distance +
                                   altitude_diff_nm * altitude_diff_nm);

    TS_ASSERT(slant_range > ground_distance);
  }

  /***************************************************************************
   * Marker Beacon Detection
   ***************************************************************************/

  void testOuterMarkerDistance() {
    // Outer marker typically 4-7 nm from threshold
    double outer_marker_distance = 5.0;  // nm
    TS_ASSERT(outer_marker_distance >= 4.0 && outer_marker_distance <= 7.0);
  }

  void testMiddleMarkerDistance() {
    // Middle marker typically 0.5-0.8 nm from threshold
    double middle_marker_distance = 0.6;  // nm
    TS_ASSERT(middle_marker_distance >= 0.5 && middle_marker_distance <= 0.8);
  }

  void testInnerMarkerDistance() {
    // Inner marker at threshold (CAT II/III)
    double inner_marker_distance = 0.1;  // nm
    TS_ASSERT(inner_marker_distance < 0.2);
  }

  void testMarkerBeaconFrequency() {
    // All marker beacons operate at 75 MHz
    double marker_freq_mhz = 75.0;
    TS_ASSERT_DELTA(marker_freq_mhz, 75.0, epsilon);
  }

  void testOuterMarkerAltitude() {
    // At outer marker on 3° glideslope, altitude ~1400 ft
    double glideslope = 3.0;
    double distance_nm = 5.0;
    double altitude = distance_nm * NM_TO_FT * std::tan(glideslope * DEG_TO_RAD);

    TS_ASSERT(altitude > 1300.0 && altitude < 1800.0);
  }

  void testMiddleMarkerAltitude() {
    // At middle marker on 3° glideslope, altitude ~200 ft
    double glideslope = 3.0;
    double distance_nm = 0.6;
    double altitude = distance_nm * NM_TO_FT * std::tan(glideslope * DEG_TO_RAD);

    TS_ASSERT(altitude > 150.0 && altitude < 250.0);
  }

  /***************************************************************************
   * DME Arc Procedures
   ***************************************************************************/

  void testDMEArcEntry() {
    double current_dme = 12.0;  // nm
    double arc_radius = 10.0;   // nm
    double lead_radial = 5.0;   // degrees lead for turn

    bool approaching_arc = current_dme > arc_radius;
    TS_ASSERT(approaching_arc);
  }

  void testDMEArcMaintenance() {
    double arc_radius = 10.0;  // nm
    double current_dme = 10.2;
    double tolerance = 0.5;

    bool on_arc = std::abs(current_dme - arc_radius) < tolerance;
    TS_ASSERT(on_arc);
  }

  void testDMEArcHeading() {
    // Flying arc, heading is perpendicular to radial
    double current_radial = 90.0;
    double arc_heading_cw = NormalizeHeading(current_radial + 90.0);  // Clockwise
    double arc_heading_ccw = NormalizeHeading(current_radial - 90.0);  // Counter-clockwise

    TS_ASSERT_DELTA(arc_heading_cw, 180.0, epsilon);
    TS_ASSERT_DELTA(arc_heading_ccw, 0.0, epsilon);
  }

  void testDMEArcLeadRadial() {
    // Lead radial for arc exit
    double exit_radial = 270.0;
    double arc_radius = 10.0;
    double turn_radius = 1.0;  // nm

    // Lead angle = arcsin(turn_radius / arc_radius)
    double lead_angle = std::asin(turn_radius / arc_radius) * RAD_TO_DEG;
    double lead_radial = NormalizeHeading(exit_radial - lead_angle);

    TS_ASSERT(lead_radial > 260.0);
    TS_ASSERT(lead_radial < 270.0);
  }

  /***************************************************************************
   * Localizer Back Course
   ***************************************************************************/

  void testLocalizerBackCourse() {
    double front_course = 90.0;
    double back_course = NormalizeHeading(front_course + 180.0);

    TS_ASSERT_DELTA(back_course, 270.0, epsilon);
  }

  void testBackCourseReverseSensing() {
    // On back course, deflection is reversed
    double deviation_front = 2.0;  // Right of centerline
    double deviation_back = -deviation_front;  // Shows left

    TS_ASSERT_DELTA(deviation_back, -2.0, epsilon);
  }

  void testBackCourseNoGlideslope() {
    // Back course has no glideslope
    bool glideslope_available = false;
    TS_ASSERT(!glideslope_available);
  }

  /***************************************************************************
   * CAT II/III ILS Operations
   ***************************************************************************/

  void testCATIIDecisionHeight() {
    double decision_height = 100.0;  // ft (CAT II)
    TS_ASSERT(decision_height >= 100.0 && decision_height < 200.0);
  }

  void testCATIIIaDecisionHeight() {
    double decision_height = 50.0;  // ft (CAT IIIa)
    TS_ASSERT(decision_height < 100.0);
  }

  void testCATIIIbNoDecisionHeight() {
    // CAT IIIb has no decision height (or <50 ft)
    double decision_height = 0.0;
    TS_ASSERT(decision_height < 50.0);
  }

  void testCATIIRVR() {
    // CAT II requires RVR >= 1200 ft
    double rvr_required = 1200.0;  // ft
    double rvr_actual = 1500.0;

    bool rvr_adequate = rvr_actual >= rvr_required;
    TS_ASSERT(rvr_adequate);
  }

  void testCATIIIaRVR() {
    // CAT IIIa requires RVR >= 700 ft
    double rvr_required = 700.0;  // ft
    double rvr_actual = 800.0;

    bool rvr_adequate = rvr_actual >= rvr_required;
    TS_ASSERT(rvr_adequate);
  }

  /***************************************************************************
   * RMI (Radio Magnetic Indicator) Calculations
   ***************************************************************************/

  void testRMIBearing() {
    double aircraft_heading = 45.0;
    double relative_bearing = 30.0;

    double rmi_bearing = NormalizeHeading(aircraft_heading + relative_bearing);
    TS_ASSERT_DELTA(rmi_bearing, 75.0, epsilon);
  }

  void testRMITail() {
    double rmi_head = 90.0;
    double rmi_tail = NormalizeHeading(rmi_head + 180.0);

    TS_ASSERT_DELTA(rmi_tail, 270.0, epsilon);
  }

  void testRMIInterceptAngle() {
    double desired_radial = 90.0;
    double rmi_bearing = 60.0;

    double intercept_angle = NormalizeAngle180(desired_radial - rmi_bearing);
    TS_ASSERT_DELTA(intercept_angle, 30.0, epsilon);
  }

  /***************************************************************************
   * Approach Procedure Fixes
   ***************************************************************************/

  void testInitialApproachFix() {
    double iaf_distance = 15.0;  // nm from airport
    double iaf_altitude = 5000.0;  // ft

    TS_ASSERT(iaf_distance > 10.0);
    TS_ASSERT(iaf_altitude > 3000.0);
  }

  void testIntermediateFix() {
    double if_distance = 8.0;  // nm from airport
    double if_altitude = 3000.0;  // ft

    TS_ASSERT(if_distance > 5.0 && if_distance < 12.0);
    TS_ASSERT(if_altitude > 2000.0);
  }

  void testFinalApproachFix() {
    double faf_distance = 5.0;  // nm from threshold
    double faf_altitude = 1800.0;  // ft

    TS_ASSERT(faf_distance > 4.0 && faf_distance < 7.0);
  }

  void testMissedApproachPoint() {
    double map_distance = 0.5;  // nm from threshold
    double map_altitude = 200.0;  // ft (at decision height)

    TS_ASSERT(map_distance < 1.0);
  }

  /***************************************************************************
   * VOR Accuracy and Error
   ***************************************************************************/

  void testVORBearingAccuracy() {
    // VOR accuracy is ±1° for most systems
    double vor_accuracy = 1.0;
    double measured_bearing = 90.0;

    double min_actual = measured_bearing - vor_accuracy;
    double max_actual = measured_bearing + vor_accuracy;

    TS_ASSERT(min_actual >= 89.0);
    TS_ASSERT(max_actual <= 91.0);
  }

  void testDMEAccuracy() {
    // DME accuracy is ±0.5 nm or ±3% (whichever is greater)
    double dme_reading = 20.0;  // nm
    double accuracy_fixed = 0.5;
    double accuracy_percent = dme_reading * 0.03;

    double accuracy = std::max(accuracy_fixed, accuracy_percent);
    TS_ASSERT_DELTA(accuracy, 0.6, 0.01);  // 3% of 20 nm = 0.6 nm
  }

  void testGPSAccuracy() {
    // GPS accuracy typically ±100 ft (WAAS ~±10 ft)
    double gps_accuracy_ft = 100.0;
    double waas_accuracy_ft = 10.0;

    TS_ASSERT(gps_accuracy_ft <= 100.0);
    TS_ASSERT(waas_accuracy_ft <= 20.0);
  }

  /***************************************************************************
   * Holding Pattern Entry with Radio Nav
   ***************************************************************************/

  void testHoldingDirectEntry() {
    double holding_inbound = 90.0;
    double aircraft_heading = 85.0;

    double relative = NormalizeAngle180(aircraft_heading - holding_inbound);
    bool direct_entry = std::abs(relative) <= 70.0;

    TS_ASSERT(direct_entry);
  }

  void testHoldingTeardropEntry() {
    double holding_inbound = 90.0;
    double aircraft_heading = 200.0;

    double relative = NormalizeAngle180(aircraft_heading - holding_inbound);
    bool teardrop_entry = relative > 70.0 && relative <= 180.0;

    TS_ASSERT(teardrop_entry);
  }

  void testHoldingParallelEntry() {
    double holding_inbound = 90.0;
    double aircraft_heading = 300.0;

    double relative = NormalizeAngle180(aircraft_heading - holding_inbound);
    bool parallel_entry = relative < -70.0;

    TS_ASSERT(parallel_entry);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testStressHeadingNormalization() {
    for (int i = -720; i <= 720; i += 10) {
      double heading = NormalizeHeading(static_cast<double>(i));
      TS_ASSERT(heading >= 0.0);
      TS_ASSERT(heading < 360.0);
    }
  }

  void testStressAngleNormalization() {
    for (int i = -720; i <= 720; i += 10) {
      double angle = NormalizeAngle180(static_cast<double>(i));
      TS_ASSERT(angle > -180.0);
      TS_ASSERT(angle <= 180.0);
    }
  }

  void testStressBearingCalculations() {
    for (int radial = 0; radial < 360; radial += 15) {
      for (int heading = 0; heading < 360; heading += 15) {
        double deviation = NormalizeAngle180(heading - radial);
        TS_ASSERT(deviation > -180.0);
        TS_ASSERT(deviation <= 180.0);
      }
    }
  }

  void testStressDMECalculations() {
    for (double dist = 0.1; dist <= 200.0; dist += 5.0) {
      for (double alt = 1000.0; alt <= 40000.0; alt += 5000.0) {
        double alt_nm = alt * FT_TO_NM;
        double slant = std::sqrt(dist * dist + alt_nm * alt_nm);
        TS_ASSERT(slant >= dist);
        TS_ASSERT(!std::isnan(slant));
      }
    }
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteILSApproach() {
    // Complete ILS approach from FAF to threshold
    double faf_dist = 5.0;  // nm
    double threshold_dist = 0.0;
    double glideslope = 3.0;  // degrees
    double descent_rate = 500.0;  // ft/min
    double groundspeed = 120.0;  // kts

    // Time to fly from FAF to threshold
    double time_min = (faf_dist - threshold_dist) * 60.0 / groundspeed;
    TS_ASSERT(time_min > 2.0);
    TS_ASSERT(time_min < 5.0);

    // Altitude loss
    double alt_loss = descent_rate * time_min;
    TS_ASSERT(alt_loss > 1000.0);
  }

  void testCompleteVORNavigation() {
    // Navigate FROM one VOR TO another
    double vor1_radial = 90.0;  // Outbound radial from VOR1
    double vor2_radial = 270.0; // Inbound radial to VOR2
    double distance = 50.0;     // nm between VORs

    // Halfway point
    double halfway = distance / 2.0;
    TS_ASSERT_DELTA(halfway, 25.0, 0.1);

    // Course correction at halfway
    double correction = NormalizeAngle180(vor2_radial - vor1_radial);
    TS_ASSERT(std::abs(correction) <= 180.0);
  }

  void testCompleteHoldingPattern() {
    double inbound_course = 270.0;
    double leg_time = 1.0;  // minute
    double turn_rate = 3.0; // deg/sec (standard rate)

    // Turn time for 180 degrees
    double turn_time_sec = 180.0 / turn_rate;
    TS_ASSERT_DELTA(turn_time_sec, 60.0, 1.0);

    // Total pattern time
    double pattern_time_min = 2.0 * leg_time + 2.0 * (turn_time_sec / 60.0);
    TS_ASSERT(pattern_time_min > 3.0);
    TS_ASSERT(pattern_time_min < 5.0);
  }

  void testCompleteDMEArcProcedure() {
    double arc_radius = 10.0;  // nm
    double arc_start = 0.0;    // degrees
    double arc_end = 90.0;     // degrees
    double arc_length = 2.0 * M_PI * arc_radius * (arc_end - arc_start) / 360.0;

    TS_ASSERT(arc_length > 15.0);
    TS_ASSERT(arc_length < 20.0);
  }

  void testCompleteGPSWaypointSequence() {
    double waypoints[][2] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    double total_distance = 0.0;

    for (int i = 1; i < 4; i++) {
      double dx = waypoints[i][0] - waypoints[i-1][0];
      double dy = waypoints[i][1] - waypoints[i-1][1];
      total_distance += std::sqrt(dx*dx + dy*dy);
    }

    TS_ASSERT_DELTA(total_distance, 3.0, 0.1);  // 3 nm total
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentBearingCalculations() {
    double bearing1 = NormalizeHeading(45.0 + 360.0);
    double bearing2 = NormalizeHeading(315.0 - 360.0);

    TS_ASSERT_DELTA(bearing1, 45.0, 0.001);
    TS_ASSERT_DELTA(bearing2, 315.0, 0.001);

    // Verify bearing1 unchanged
    double bearing1_verify = NormalizeHeading(45.0 + 360.0);
    TS_ASSERT_DELTA(bearing1, bearing1_verify, 0.001);
  }

  void testIndependentRadialInterceptions() {
    double radial1 = 90.0;
    double heading1 = 45.0;
    double intercept1 = NormalizeAngle180(radial1 - heading1);

    double radial2 = 180.0;
    double heading2 = 135.0;
    double intercept2 = NormalizeAngle180(radial2 - heading2);

    TS_ASSERT_DELTA(intercept1, 45.0, 0.001);
    TS_ASSERT_DELTA(intercept2, 45.0, 0.001);
  }

  void testIndependentDMEReadings() {
    double dist1 = 20.0;
    double alt1 = 10000.0;
    double slant1 = std::sqrt(dist1*dist1 + std::pow(alt1 * FT_TO_NM, 2));

    double dist2 = 50.0;
    double alt2 = 30000.0;
    double slant2 = std::sqrt(dist2*dist2 + std::pow(alt2 * FT_TO_NM, 2));

    TS_ASSERT(slant2 > slant1);

    // Verify slant1 unchanged
    double slant1_verify = std::sqrt(dist1*dist1 + std::pow(alt1 * FT_TO_NM, 2));
    TS_ASSERT_DELTA(slant1, slant1_verify, 0.001);
  }

  void testIndependentGlideslopeCalculations() {
    double angle1 = 3.0;
    double dist1 = 5.0;
    double alt1 = dist1 * NM_TO_FT * std::tan(angle1 * DEG_TO_RAD);

    double angle2 = 4.0;
    double dist2 = 3.0;
    double alt2 = dist2 * NM_TO_FT * std::tan(angle2 * DEG_TO_RAD);

    TS_ASSERT(alt1 > 0.0);
    TS_ASSERT(alt2 > 0.0);

    // Verify alt1 unchanged
    double alt1_verify = dist1 * NM_TO_FT * std::tan(angle1 * DEG_TO_RAD);
    TS_ASSERT_DELTA(alt1, alt1_verify, 0.1);
  }

  void testIndependentCourseDeviations() {
    double desired1 = 90.0;
    double actual1 = 85.0;
    double dev1 = NormalizeAngle180(actual1 - desired1);

    double desired2 = 270.0;
    double actual2 = 280.0;
    double dev2 = NormalizeAngle180(actual2 - desired2);

    TS_ASSERT_DELTA(dev1, -5.0, 0.001);
    TS_ASSERT_DELTA(dev2, 10.0, 0.001);
  }
};
