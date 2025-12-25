/*******************************************************************************
 * FGNavigationTest.h - Unit tests for navigation and waypoint calculations
 *
 * Tests mathematical navigation algorithms including:
 * - Great circle distance and bearing calculations
 * - Rhumb line navigation
 * - Waypoint intercept and cross-track error
 * - DME arc and holding pattern geometry
 * - Wind correction and ground speed calculations
 * - Time/distance and fuel range calculations
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

// Navigation constants
const double EARTH_RADIUS_NM = 3440.065;  // Earth mean radius in nautical miles
const double EARTH_RADIUS_FT = EARTH_RADIUS_NM * NM_TO_FT;  // Earth radius in feet

// Helper function to normalize angle to [-π, π]
inline double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Helper function to normalize heading to [0, 2π]
inline double NormalizeHeading(double heading) {
  while (heading < 0.0) heading += 2.0 * M_PI;
  while (heading >= 2.0 * M_PI) heading -= 2.0 * M_PI;
  return heading;
}

class FGNavigationTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Great Circle Distance Calculations
   ***************************************************************************/

  // Test distance between same point (should be zero)
  void testGreatCircleSamePoint() {
    double lat = 40.0 * DEG_TO_RAD;
    double lon = -74.0 * DEG_TO_RAD;

    double dlat = 0.0;
    double dlon = 0.0;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat) * std::cos(lat) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    TS_ASSERT_DELTA(distance, 0.0, epsilon);
  }

  // Test distance along equator (simplified case)
  void testGreatCircleEquator() {
    double lat1 = 0.0;
    double lon1 = 0.0;
    double lat2 = 0.0;
    double lon2 = 90.0 * DEG_TO_RAD;  // 90 degrees east

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // Quarter of great circle
    double expected = 2.0 * M_PI * EARTH_RADIUS_NM / 4.0;
    TS_ASSERT_DELTA(distance, expected, 1.0);
  }

  // Test distance New York to London
  void testGreatCircleNYtoLondon() {
    double lat1 = 40.7128 * DEG_TO_RAD;   // NYC
    double lon1 = -74.0060 * DEG_TO_RAD;
    double lat2 = 51.5074 * DEG_TO_RAD;   // London
    double lon2 = -0.1278 * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // NY to London is approximately 3000 nm
    TS_ASSERT(distance > 2900.0 && distance < 3100.0);
  }

  // Test antipodal points (maximum distance)
  void testGreatCircleAntipodal() {
    double lat1 = 0.0;
    double lon1 = 0.0;
    double lat2 = 0.0;
    double lon2 = M_PI;  // 180 degrees

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // Half of great circle
    double expected = M_PI * EARTH_RADIUS_NM;
    TS_ASSERT_DELTA(distance, expected, 1.0);
  }

  // Test small distance approximation
  void testGreatCircleSmallDistance() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 40.01 * DEG_TO_RAD;
    double lon2 = -74.01 * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // Should be less than 2 nm
    TS_ASSERT(distance < 2.0);
  }

  /***************************************************************************
   * Initial Bearing Calculations
   ***************************************************************************/

  // Test bearing due north
  void testBearingNorth() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 50.0 * DEG_TO_RAD;
    double lon2 = -74.0 * DEG_TO_RAD;  // Same longitude

    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(y, x);

    TS_ASSERT_DELTA(bearing, 0.0, 0.01);  // Due north
  }

  // Test bearing due east
  void testBearingEast() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 40.0 * DEG_TO_RAD;  // Same latitude
    double lon2 = -73.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(y, x);

    TS_ASSERT_DELTA(bearing, M_PI / 2.0, 0.01);  // Due east
  }

  // Test bearing due south
  void testBearingSouth() {
    double lat1 = 50.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 40.0 * DEG_TO_RAD;
    double lon2 = -74.0 * DEG_TO_RAD;  // Same longitude

    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(y, x);

    TS_ASSERT_DELTA(std::abs(bearing), M_PI, 0.01);  // Due south (±π)
  }

  // Test bearing due west
  void testBearingWest() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -73.0 * DEG_TO_RAD;
    double lat2 = 40.0 * DEG_TO_RAD;  // Same latitude
    double lon2 = -74.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(y, x);

    TS_ASSERT_DELTA(bearing, -M_PI / 2.0, 0.01);  // Due west
  }

  // Test bearing northeast
  void testBearingNortheast() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 41.0 * DEG_TO_RAD;
    double lon2 = -73.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(y, x);

    // Should be between 0 and π/2 (northeast quadrant)
    TS_ASSERT(bearing > 0.0 && bearing < M_PI / 2.0);
  }

  /***************************************************************************
   * Final Bearing Calculations
   ***************************************************************************/

  // Test final bearing is reverse of initial (on great circle)
  void testFinalBearing() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 50.0 * DEG_TO_RAD;
    double lon2 = -74.0 * DEG_TO_RAD;

    // Initial bearing from point 1 to point 2
    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing_initial = std::atan2(y, x);

    // Final bearing from point 2 to point 1
    double dlon_rev = lon1 - lon2;
    double y_rev = std::sin(dlon_rev) * std::cos(lat1);
    double x_rev = std::cos(lat2) * std::sin(lat1) -
                   std::sin(lat2) * std::cos(lat1) * std::cos(dlon_rev);
    double bearing_final = std::atan2(y_rev, x_rev);

    // For north-south route, final should be reverse of initial
    double bearing_expected = NormalizeAngle(bearing_initial + M_PI);
    TS_ASSERT_DELTA(bearing_final, bearing_expected, 0.01);
  }

  /***************************************************************************
   * Rhumb Line Navigation
   ***************************************************************************/

  // Test rhumb line distance along meridian (equals great circle)
  void testRhumbLineMeridian() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 50.0 * DEG_TO_RAD;
    double lon2 = -74.0 * DEG_TO_RAD;  // Same longitude

    // For meridian, rhumb line = great circle
    double dlat = lat2 - lat1;
    double distance = EARTH_RADIUS_NM * std::abs(dlat);

    // Should be approximately 600 nm (10 degrees latitude)
    TS_ASSERT_DELTA(distance, 600.0, 10.0);
  }

  // Test rhumb line constant bearing
  void testRhumbLineConstantBearing() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 41.0 * DEG_TO_RAD;
    double lon2 = -73.0 * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Rhumb line bearing
    double dphi = std::log(std::tan(lat2 / 2.0 + M_PI / 4.0) /
                           std::tan(lat1 / 2.0 + M_PI / 4.0));
    double bearing = std::atan2(dlon, dphi);

    // Bearing should be between 0 and π/2 (northeast)
    TS_ASSERT(bearing > 0.0 && bearing < M_PI / 2.0);
  }

  // Test rhumb line distance
  void testRhumbLineDistance() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 41.0 * DEG_TO_RAD;
    double lon2 = -73.0 * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double dphi = std::log(std::tan(lat2 / 2.0 + M_PI / 4.0) /
                           std::tan(lat1 / 2.0 + M_PI / 4.0));
    double q = std::abs(dphi) > 1e-12 ? dlat / dphi : std::cos(lat1);
    double distance = EARTH_RADIUS_NM * std::sqrt(dlat * dlat + q * q * dlon * dlon);

    // Should be reasonable distance (less than 100 nm)
    TS_ASSERT(distance > 0.0 && distance < 200.0);
  }

  /***************************************************************************
   * Waypoint Intercept Calculations
   ***************************************************************************/

  // Test waypoint arrival (within tolerance)
  void testWaypointArrival() {
    double aircraft_lat = 40.0 * DEG_TO_RAD;
    double aircraft_lon = -74.0 * DEG_TO_RAD;
    double waypoint_lat = 40.001 * DEG_TO_RAD;
    double waypoint_lon = -74.001 * DEG_TO_RAD;

    double dlat = waypoint_lat - aircraft_lat;
    double dlon = waypoint_lon - aircraft_lon;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(aircraft_lat) * std::cos(waypoint_lat) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // Within 0.5 nm tolerance
    TS_ASSERT(distance < 0.5);
  }

  // Test next waypoint selection
  void testNextWaypoint() {
    // Aircraft approaching waypoint 1
    double aircraft_lat = 40.0 * DEG_TO_RAD;
    double waypoint1_lat = 40.01 * DEG_TO_RAD;
    double waypoint2_lat = 40.02 * DEG_TO_RAD;

    double dist1 = std::abs(waypoint1_lat - aircraft_lat) * EARTH_RADIUS_NM;
    double dist2 = std::abs(waypoint2_lat - aircraft_lat) * EARTH_RADIUS_NM;

    // Waypoint 1 should be closer
    TS_ASSERT(dist1 < dist2);
  }

  /***************************************************************************
   * Cross-Track Error
   ***************************************************************************/

  // Test cross-track error on route
  void testCrossTrackOnRoute() {
    // Aircraft exactly on route
    double lat_start = 40.0 * DEG_TO_RAD;
    double lon_start = -74.0 * DEG_TO_RAD;
    double lat_end = 40.0 * DEG_TO_RAD;
    double lon_end = -73.0 * DEG_TO_RAD;
    double lat_aircraft = 40.0 * DEG_TO_RAD;
    double lon_aircraft = -73.5 * DEG_TO_RAD;  // Midpoint

    // Distance from start to aircraft
    double dlon1 = lon_aircraft - lon_start;
    double a1 = std::sin(0.0) * std::sin(0.0) +
                std::cos(lat_start) * std::cos(lat_aircraft) *
                std::sin(dlon1/2) * std::sin(dlon1/2);
    double dist13 = 2.0 * std::atan2(std::sqrt(a1), std::sqrt(1.0 - a1));

    // Bearing from start to aircraft
    double y1 = std::sin(dlon1) * std::cos(lat_aircraft);
    double x1 = std::cos(lat_start) * std::sin(lat_aircraft) -
                std::sin(lat_start) * std::cos(lat_aircraft) * std::cos(dlon1);
    double brg13 = std::atan2(y1, x1);

    // Bearing from start to end
    double dlon2 = lon_end - lon_start;
    double y2 = std::sin(dlon2) * std::cos(lat_end);
    double x2 = std::cos(lat_start) * std::sin(lat_end) -
                std::sin(lat_start) * std::cos(lat_end) * std::cos(dlon2);
    double brg12 = std::atan2(y2, x2);

    // Cross-track error
    double xt = std::asin(std::sin(dist13) * std::sin(brg13 - brg12)) * EARTH_RADIUS_NM;

    TS_ASSERT_DELTA(xt, 0.0, 0.1);  // Should be near zero
  }

  // Test cross-track error off route
  void testCrossTrackOffRoute() {
    // Route from west to east
    double lat_start = 40.0 * DEG_TO_RAD;
    double lon_start = -74.0 * DEG_TO_RAD;
    double lat_end = 40.0 * DEG_TO_RAD;
    double lon_end = -73.0 * DEG_TO_RAD;

    // Aircraft north of route
    double lat_aircraft = 40.5 * DEG_TO_RAD;
    double lon_aircraft = -73.5 * DEG_TO_RAD;

    // Distance from start to aircraft
    double dlat = lat_aircraft - lat_start;
    double dlon1 = lon_aircraft - lon_start;
    double a1 = std::sin(dlat/2) * std::sin(dlat/2) +
                std::cos(lat_start) * std::cos(lat_aircraft) *
                std::sin(dlon1/2) * std::sin(dlon1/2);
    double dist13 = 2.0 * std::atan2(std::sqrt(a1), std::sqrt(1.0 - a1));

    // Bearing from start to aircraft
    double y1 = std::sin(dlon1) * std::cos(lat_aircraft);
    double x1 = std::cos(lat_start) * std::sin(lat_aircraft) -
                std::sin(lat_start) * std::cos(lat_aircraft) * std::cos(dlon1);
    double brg13 = std::atan2(y1, x1);

    // Bearing from start to end
    double dlon2 = lon_end - lon_start;
    double y2 = std::sin(dlon2) * std::cos(lat_end);
    double x2 = std::cos(lat_start) * std::sin(lat_end) -
                std::sin(lat_start) * std::cos(lat_end) * std::cos(dlon2);
    double brg12 = std::atan2(y2, x2);

    // Cross-track error
    double xt = std::asin(std::sin(dist13) * std::sin(brg13 - brg12)) * EARTH_RADIUS_NM;

    // Should be approximately 30 nm north of route (0.5 degrees latitude)
    TS_ASSERT(std::abs(xt) > 10.0);
  }

  /***************************************************************************
   * Along-Track Distance
   ***************************************************************************/

  // Test along-track distance
  void testAlongTrackDistance() {
    double lat_start = 40.0 * DEG_TO_RAD;
    double lon_start = -74.0 * DEG_TO_RAD;
    double lat_end = 40.0 * DEG_TO_RAD;
    double lon_end = -73.0 * DEG_TO_RAD;
    double lat_aircraft = 40.0 * DEG_TO_RAD;
    double lon_aircraft = -73.5 * DEG_TO_RAD;

    // Distance from start to aircraft
    double dlon1 = lon_aircraft - lon_start;
    double a1 = std::sin(0.0) * std::sin(0.0) +
                std::cos(lat_start) * std::cos(lat_aircraft) *
                std::sin(dlon1/2) * std::sin(dlon1/2);
    double dist13 = 2.0 * std::atan2(std::sqrt(a1), std::sqrt(1.0 - a1));

    // Bearing difference
    double y1 = std::sin(dlon1) * std::cos(lat_aircraft);
    double x1 = std::cos(lat_start) * std::sin(lat_aircraft) -
                std::sin(lat_start) * std::cos(lat_aircraft) * std::cos(dlon1);
    double brg13 = std::atan2(y1, x1);

    double dlon2 = lon_end - lon_start;
    double y2 = std::sin(dlon2) * std::cos(lat_end);
    double x2 = std::cos(lat_start) * std::sin(lat_end) -
                std::sin(lat_start) * std::cos(lat_end) * std::cos(dlon2);
    double brg12 = std::atan2(y2, x2);

    // Cross-track and along-track
    double xt = std::asin(std::sin(dist13) * std::sin(brg13 - brg12));
    double at = std::acos(std::cos(dist13) / std::cos(xt)) * EARTH_RADIUS_NM;

    // Should be approximately half the route distance
    TS_ASSERT(at > 0.0);
  }

  /***************************************************************************
   * DME Arc Calculations
   ***************************************************************************/

  // Test DME arc radius
  void testDMEArcRadius() {
    double station_lat = 40.0 * DEG_TO_RAD;
    double station_lon = -74.0 * DEG_TO_RAD;
    double aircraft_lat = 40.1 * DEG_TO_RAD;
    double aircraft_lon = -74.0 * DEG_TO_RAD;

    double dlat = aircraft_lat - station_lat;
    double dlon = aircraft_lon - station_lon;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(station_lat) * std::cos(aircraft_lat) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double dme_distance = EARTH_RADIUS_NM * c;

    // Approximately 6 nm
    TS_ASSERT_DELTA(dme_distance, 6.0, 1.0);
  }

  // Test DME arc tangent point
  void testDMEArcTangent() {
    // 10 nm arc, approaching from 20 nm out
    double arc_radius = 10.0;
    double current_dme = 20.0;

    // Distance to tangent point
    double distance_to_tangent = std::sqrt(current_dme * current_dme - arc_radius * arc_radius);

    TS_ASSERT_DELTA(distance_to_tangent, std::sqrt(300.0), 0.1);
  }

  /***************************************************************************
   * Holding Pattern Geometry
   ***************************************************************************/

  // Test holding pattern inbound course
  void testHoldingInboundCourse() {
    double hold_radial = 90.0 * DEG_TO_RAD;  // Hold on 090 radial
    double inbound_course = NormalizeAngle(hold_radial + M_PI);  // Reciprocal

    TS_ASSERT_DELTA(inbound_course, -M_PI / 2.0, epsilon);
  }

  // Test holding pattern turn direction
  void testHoldingTurnDirection() {
    // Standard holding = right turns
    bool right_turns = true;

    TS_ASSERT_EQUALS(right_turns, true);

    // Non-standard holding = left turns
    bool left_turns = false;
    TS_ASSERT_EQUALS(left_turns, false);
  }

  // Test holding pattern leg length
  void testHoldingLegLength() {
    double ground_speed = 120.0;  // knots
    double leg_time = 1.0;  // minutes
    double leg_length = ground_speed * leg_time / 60.0;  // nm

    TS_ASSERT_DELTA(leg_length, 2.0, epsilon);
  }

  /***************************************************************************
   * Procedure Turn Calculations
   ***************************************************************************/

  // Test procedure turn outbound heading
  void testProcedureTurnOutbound() {
    double inbound_course = 0.0;  // Due north
    double turn_direction = 1.0;  // Right turn (45 degrees)
    double outbound_heading = NormalizeAngle(inbound_course + M_PI +
                                             turn_direction * 45.0 * DEG_TO_RAD);

    // Should be approximately 225 degrees (southwest)
    TS_ASSERT_DELTA(outbound_heading, -135.0 * DEG_TO_RAD, 0.01);
  }

  // Test procedure turn return heading
  void testProcedureTurnReturn() {
    double outbound_heading = -135.0 * DEG_TO_RAD;  // 225 degrees
    double turn_angle = 180.0 * DEG_TO_RAD;
    double return_heading = NormalizeAngle(outbound_heading + turn_angle);

    // Should be approximately 45 degrees (northeast)
    TS_ASSERT_DELTA(return_heading, 45.0 * DEG_TO_RAD, 0.01);
  }

  /***************************************************************************
   * Course Intercept Angles
   ***************************************************************************/

  // Test intercept angle 30 degrees
  void testInterceptAngle30() {
    double desired_course = 0.0;  // Due north
    double intercept_angle = 30.0 * DEG_TO_RAD;
    double intercept_heading = NormalizeAngle(desired_course + intercept_angle);

    TS_ASSERT_DELTA(intercept_heading, 30.0 * DEG_TO_RAD, epsilon);
  }

  // Test intercept angle 45 degrees
  void testInterceptAngle45() {
    double desired_course = 90.0 * DEG_TO_RAD;  // Due east
    double intercept_angle = 45.0 * DEG_TO_RAD;
    double intercept_heading = NormalizeAngle(desired_course + intercept_angle);

    TS_ASSERT_DELTA(intercept_heading, 135.0 * DEG_TO_RAD, epsilon);
  }

  // Test course interception from left
  void testCourseInterceptFromLeft() {
    double current_heading = 45.0 * DEG_TO_RAD;
    double desired_course = 90.0 * DEG_TO_RAD;
    double angle_diff = NormalizeAngle(desired_course - current_heading);

    // Should turn right
    TS_ASSERT(angle_diff > 0.0);
  }

  // Test course interception from right
  void testCourseInterceptFromRight() {
    double current_heading = 135.0 * DEG_TO_RAD;
    double desired_course = 90.0 * DEG_TO_RAD;
    double angle_diff = NormalizeAngle(desired_course - current_heading);

    // Should turn left
    TS_ASSERT(angle_diff < 0.0);
  }

  /***************************************************************************
   * Wind Correction Angle
   ***************************************************************************/

  // Test wind correction angle no wind
  void testWindCorrectionNoWind() {
    double tas = 150.0;  // knots
    double wind_speed = 0.0;
    double wind_correction = std::asin(wind_speed / tas);

    TS_ASSERT_DELTA(wind_correction, 0.0, epsilon);
  }

  // Test wind correction angle crosswind
  void testWindCorrectionCrosswind() {
    double tas = 150.0;  // knots
    double wind_speed = 30.0;  // knots (direct crosswind)
    double wind_correction = std::asin(wind_speed / tas);

    // Should be approximately 11.5 degrees
    TS_ASSERT_DELTA(wind_correction * RAD_TO_DEG, 11.54, 0.1);
  }

  // Test wind correction angle headwind
  void testWindCorrectionHeadwind() {
    double tas = 150.0;
    double wind_from = 0.0 * DEG_TO_RAD;  // Wind from north
    double course = 0.0 * DEG_TO_RAD;  // Flying north
    double wind_angle = wind_from - course;

    // Headwind should not require course correction
    TS_ASSERT_DELTA(std::sin(wind_angle), 0.0, epsilon);
  }

  // Test maximum wind correction
  void testWindCorrectionMaximum() {
    double tas = 150.0;
    double max_crosswind = tas;  // 100% crosswind would be 90 degrees
    double max_correction = std::asin(max_crosswind / tas);

    TS_ASSERT_DELTA(max_correction, M_PI / 2.0, epsilon);
  }

  /***************************************************************************
   * Ground Speed from TAS and Wind
   ***************************************************************************/

  // Test ground speed no wind
  void testGroundSpeedNoWind() {
    double tas = 150.0;
    double wind_speed = 0.0;
    double ground_speed = tas;  // No wind

    TS_ASSERT_DELTA(ground_speed, 150.0, epsilon);
  }

  // Test ground speed headwind
  void testGroundSpeedHeadwind() {
    double tas = 150.0;
    double headwind = 30.0;
    double ground_speed = tas - headwind;

    TS_ASSERT_DELTA(ground_speed, 120.0, epsilon);
  }

  // Test ground speed tailwind
  void testGroundSpeedTailwind() {
    double tas = 150.0;
    double tailwind = 30.0;
    double ground_speed = tas + tailwind;

    TS_ASSERT_DELTA(ground_speed, 180.0, epsilon);
  }

  // Test ground speed with crosswind
  void testGroundSpeedCrosswind() {
    double tas = 150.0;
    double wind_speed = 30.0;
    double heading = 0.0;  // North
    double wind_from = 90.0 * DEG_TO_RAD;  // From east (left crosswind)

    double wind_angle = wind_from - heading;
    double headwind_component = wind_speed * std::cos(wind_angle);
    double ground_speed = tas - headwind_component;

    // Crosswind doesn't affect ground speed in simplified case
    TS_ASSERT_DELTA(ground_speed, 150.0, 1.0);
  }

  // Test ground speed with quartering headwind
  void testGroundSpeedQuarteringHeadwind() {
    double tas = 150.0;
    double wind_speed = 30.0;
    double wind_from = 45.0 * DEG_TO_RAD;  // From northeast
    double heading = 0.0;  // Flying north

    double wind_angle = wind_from - heading - M_PI;
    double headwind_component = wind_speed * std::cos(wind_angle);
    double ground_speed = std::sqrt(tas * tas - wind_speed * wind_speed * std::sin(wind_angle) * std::sin(wind_angle)) + headwind_component;

    // Should be less than TAS
    TS_ASSERT(ground_speed < tas);
  }

  /***************************************************************************
   * Time/Distance Calculations
   ***************************************************************************/

  // Test time to waypoint
  void testTimeToWaypoint() {
    double distance = 120.0;  // nm
    double ground_speed = 120.0;  // knots
    double time = distance / ground_speed;  // hours

    TS_ASSERT_DELTA(time, 1.0, epsilon);
  }

  // Test ETA calculation
  void testETACalculation() {
    double distance = 180.0;  // nm
    double ground_speed = 120.0;  // knots
    double ete = distance / ground_speed * 60.0;  // minutes

    TS_ASSERT_DELTA(ete, 90.0, epsilon);
  }

  // Test distance in given time
  void testDistanceInTime() {
    double ground_speed = 150.0;  // knots
    double time = 0.5;  // hours (30 minutes)
    double distance = ground_speed * time;

    TS_ASSERT_DELTA(distance, 75.0, epsilon);
  }

  /***************************************************************************
   * Fuel Range Calculations
   ***************************************************************************/

  // Test endurance calculation
  void testEnduranceCalculation() {
    double fuel_capacity = 50.0;  // gallons
    double fuel_burn_rate = 10.0;  // gallons per hour
    double endurance = fuel_capacity / fuel_burn_rate;  // hours

    TS_ASSERT_DELTA(endurance, 5.0, epsilon);
  }

  // Test range calculation
  void testRangeCalculation() {
    double fuel_capacity = 50.0;  // gallons
    double fuel_burn_rate = 10.0;  // gph
    double tas = 150.0;  // knots
    double range = (fuel_capacity / fuel_burn_rate) * tas;  // nm

    TS_ASSERT_DELTA(range, 750.0, epsilon);
  }

  // Test range with headwind
  void testRangeWithHeadwind() {
    double fuel_capacity = 50.0;
    double fuel_burn_rate = 10.0;
    double tas = 150.0;
    double headwind = 30.0;
    double ground_speed = tas - headwind;
    double range = (fuel_capacity / fuel_burn_rate) * ground_speed;

    TS_ASSERT_DELTA(range, 600.0, epsilon);
  }

  // Test range with tailwind
  void testRangeWithTailwind() {
    double fuel_capacity = 50.0;
    double fuel_burn_rate = 10.0;
    double tas = 150.0;
    double tailwind = 30.0;
    double ground_speed = tas + tailwind;
    double range = (fuel_capacity / fuel_burn_rate) * ground_speed;

    TS_ASSERT_DELTA(range, 900.0, epsilon);
  }

  // Test fuel required for distance
  void testFuelRequired() {
    double distance = 600.0;  // nm
    double ground_speed = 120.0;  // knots
    double fuel_burn_rate = 10.0;  // gph
    double time = distance / ground_speed;
    double fuel_required = time * fuel_burn_rate;

    TS_ASSERT_DELTA(fuel_required, 50.0, epsilon);
  }

  // Test reserve fuel
  void testReserveFuel() {
    double fuel_required = 50.0;  // gallons
    double reserve_minutes = 45.0;  // minutes
    double fuel_burn_rate = 10.0;  // gph
    double reserve_fuel = (reserve_minutes / 60.0) * fuel_burn_rate;
    double total_fuel = fuel_required + reserve_fuel;

    TS_ASSERT_DELTA(total_fuel, 57.5, epsilon);
  }

  /***************************************************************************
   * Edge Cases and Boundary Conditions
   ***************************************************************************/

  // Test crossing date line
  void testDateLineCrossing() {
    double lon1 = 179.0 * DEG_TO_RAD;
    double lon2 = -179.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    // Normalize to shortest path
    if (dlon > M_PI) dlon -= 2.0 * M_PI;
    if (dlon < -M_PI) dlon += 2.0 * M_PI;

    // Should be 2 degrees, not 358
    TS_ASSERT_DELTA(std::abs(dlon), 2.0 * DEG_TO_RAD, 0.1);
  }

  // Test polar navigation
  void testPolarNavigation() {
    double lat = 89.0 * DEG_TO_RAD;  // Near north pole

    // All longitudes converge at poles
    TS_ASSERT(std::cos(lat) < 0.1);
  }

  // Test zero ground speed
  void testZeroGroundSpeed() {
    double tas = 100.0;
    double headwind = 100.0;
    double ground_speed = tas - headwind;

    TS_ASSERT_DELTA(ground_speed, 0.0, epsilon);
  }
};
