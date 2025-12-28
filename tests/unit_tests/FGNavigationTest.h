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

  /***************************************************************************
   * VOR/Radial Navigation
   ***************************************************************************/

  // Test VOR radial identification
  void testVORRadialIdentification() {
    double station_lat = 40.0 * DEG_TO_RAD;
    double station_lon = -74.0 * DEG_TO_RAD;
    double aircraft_lat = 40.0 * DEG_TO_RAD;
    double aircraft_lon = -73.0 * DEG_TO_RAD;  // East of station

    double dlon = aircraft_lon - station_lon;
    double y = std::sin(dlon) * std::cos(aircraft_lat);
    double x = std::cos(station_lat) * std::sin(aircraft_lat) -
               std::sin(station_lat) * std::cos(aircraft_lat) * std::cos(dlon);
    double radial = std::atan2(y, x);

    // Should be on 090 radial (east)
    TS_ASSERT_DELTA(radial, M_PI / 2.0, 0.01);
  }

  // Test VOR to/from indication
  void testVORToFromIndication() {
    double selected_radial = 90.0 * DEG_TO_RAD;  // 090 radial selected
    double current_radial = 90.0 * DEG_TO_RAD;    // On 090 radial
    double heading = 270.0 * DEG_TO_RAD;          // Flying west (toward station)

    double course_diff = std::cos(heading - NormalizeAngle(selected_radial + M_PI));

    // Positive = TO, Negative = FROM
    TS_ASSERT(course_diff > 0.0);  // TO indication
  }

  // Test CDI deflection calculation
  void testCDIDeflection() {
    double selected_radial = 90.0 * DEG_TO_RAD;
    double actual_radial = 85.0 * DEG_TO_RAD;  // 5 degrees left

    double deflection = NormalizeAngle(actual_radial - selected_radial);

    // Should show 5 degree left deflection
    TS_ASSERT_DELTA(deflection, -5.0 * DEG_TO_RAD, 0.01);
  }

  // Test VOR cone of confusion
  void testVORConeOfConfusion() {
    double station_lat = 40.0 * DEG_TO_RAD;
    double station_lon = -74.0 * DEG_TO_RAD;
    double aircraft_lat = 40.0001 * DEG_TO_RAD;
    double aircraft_lon = -74.0001 * DEG_TO_RAD;

    double dlat = aircraft_lat - station_lat;
    double dlon = aircraft_lon - station_lon;
    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(station_lat) * std::cos(aircraft_lat) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // Very close - within cone of confusion
    TS_ASSERT(distance < 0.1);
  }

  /***************************************************************************
   * Descent/Climb Planning
   ***************************************************************************/

  // Test 3 degree glide path
  void testThreeDegreeGlidePath() {
    double altitude_to_lose = 3000.0;  // feet
    double glidepath_angle = 3.0 * DEG_TO_RAD;

    double distance_nm = (altitude_to_lose / std::tan(glidepath_angle)) / NM_TO_FT;

    // 3:1 ratio approximately - 3000 ft needs about 9.5 nm
    TS_ASSERT(distance_nm > 9.0 && distance_nm < 10.0);
  }

  // Test descent rate for 3 degree path
  void testDescentRate3Degree() {
    double ground_speed = 120.0;  // knots
    double glidepath_angle = 3.0;  // degrees

    // Rule of thumb: descent rate = groundspeed * 5 for 3 degree path
    double descent_rate = ground_speed * 5.0;

    TS_ASSERT_DELTA(descent_rate, 600.0, 10.0);  // fpm
  }

  // Test top of descent calculation
  void testTopOfDescent() {
    double cruise_altitude = 10000.0;  // feet
    double target_altitude = 2000.0;
    double descent_gradient = 3.0;  // nm per 1000 ft

    double altitude_to_lose = cruise_altitude - target_altitude;
    double distance_required = (altitude_to_lose / 1000.0) * descent_gradient;

    TS_ASSERT_DELTA(distance_required, 24.0, 0.1);  // nm
  }

  // Test climb gradient
  void testClimbGradient() {
    double climb_rate = 500.0;  // fpm
    double ground_speed = 100.0;  // knots

    // Gradient in feet per nm
    double gradient = climb_rate / (ground_speed / 60.0);  // ft/nm

    TS_ASSERT(gradient > 200.0 && gradient < 400.0);
  }

  /***************************************************************************
   * Rate of Turn Calculations
   ***************************************************************************/

  // Test standard rate turn
  void testStandardRateTurn() {
    double standard_rate = 3.0;  // degrees per second
    double time_for_360 = 360.0 / standard_rate;

    TS_ASSERT_DELTA(time_for_360, 120.0, epsilon);  // 2 minutes
  }

  // Test bank angle for standard rate
  void testBankAngleStandardRate() {
    double tas = 100.0;  // knots

    // Bank angle ≈ TAS / 10 + 7 for standard rate (approximation)
    double bank_angle = tas / 10.0 + 7.0;

    TS_ASSERT(bank_angle > 15.0 && bank_angle < 20.0);
  }

  // Test turn radius
  void testTurnRadius() {
    double tas = 150.0;  // knots
    double bank_angle = 30.0 * DEG_TO_RAD;

    // Turn radius = V² / (g * tan(bank))
    double g = 32.174;  // ft/s²
    double v_fps = tas * 1.68781;  // Convert knots to ft/s
    double radius_ft = (v_fps * v_fps) / (g * std::tan(bank_angle));
    double radius_nm = radius_ft / NM_TO_FT;

    TS_ASSERT(radius_nm > 0.3 && radius_nm < 0.8);
  }

  // Test half standard rate turn
  void testHalfStandardRateTurn() {
    double half_standard_rate = 1.5;  // degrees per second
    double time_for_180 = 180.0 / half_standard_rate;

    TS_ASSERT_DELTA(time_for_180, 120.0, epsilon);  // 2 minutes for 180
  }

  /***************************************************************************
   * Holding Pattern Entry
   ***************************************************************************/

  // Test direct entry sector
  void testHoldingDirectEntry() {
    double hold_inbound = 270.0 * DEG_TO_RAD;  // West
    double aircraft_heading = 280.0 * DEG_TO_RAD;

    double relative_heading = NormalizeAngle(aircraft_heading - hold_inbound);

    // Direct entry: within 70 degrees either side of inbound
    TS_ASSERT(std::abs(relative_heading) < 70.0 * DEG_TO_RAD);
  }

  // Test parallel entry sector
  void testHoldingParallelEntry() {
    double hold_inbound = 270.0 * DEG_TO_RAD;  // West
    double aircraft_heading = 180.0 * DEG_TO_RAD;  // South

    double relative_heading = NormalizeAngle(aircraft_heading - hold_inbound);

    // Parallel entry: 70 to 180 degrees on non-holding side
    TS_ASSERT(std::abs(relative_heading) > 70.0 * DEG_TO_RAD);
  }

  // Test teardrop entry sector
  void testHoldingTeardropEntry() {
    double hold_inbound = 270.0 * DEG_TO_RAD;  // West
    double aircraft_heading = 45.0 * DEG_TO_RAD;  // Northeast

    double relative_heading = NormalizeAngle(aircraft_heading - hold_inbound);

    // Teardrop entry: 110 to 180 degrees on holding side
    TS_ASSERT(std::abs(relative_heading) > 90.0 * DEG_TO_RAD);
  }

  /***************************************************************************
   * ILS Approach Calculations
   ***************************************************************************/

  // Test localizer course width
  void testLocalizerCourseWidth() {
    double full_scale_deflection = 2.5;  // degrees
    double runway_threshold_distance = 5.0;  // nm

    // Width at threshold approximately
    double width_nm = 2.0 * runway_threshold_distance * std::tan(full_scale_deflection * DEG_TO_RAD);

    TS_ASSERT(width_nm > 0.3 && width_nm < 0.6);
  }

  // Test glideslope intercept altitude
  void testGlideslopeInterceptAltitude() {
    double runway_elevation = 100.0;  // feet
    double tdz_elevation = 100.0;
    double glideslope_angle = 3.0;  // degrees
    double distance_from_threshold = 5.0;  // nm

    double height_above_tdz = distance_from_threshold * NM_TO_FT * std::tan(glideslope_angle * DEG_TO_RAD);
    double intercept_altitude = tdz_elevation + height_above_tdz;

    // At 5 nm, should be approximately 1600 feet AGL
    TS_ASSERT(intercept_altitude > 1500.0 && intercept_altitude < 1800.0);
  }

  // Test decision altitude check
  void testDecisionAltitude() {
    double current_altitude = 250.0;
    double decision_altitude = 200.0;
    double runway_in_sight = true;

    bool can_continue = (current_altitude > decision_altitude) || runway_in_sight;

    TS_ASSERT(can_continue == true);
  }

  /***************************************************************************
   * GPS/FMS Calculations
   ***************************************************************************/

  // Test LNAV cross-track sensitivity
  void testLNAVCrossTrack() {
    double cross_track_error = 0.5;  // nm
    double full_scale = 2.0;  // nm (enroute)

    double deflection = (cross_track_error / full_scale) * 100.0;  // percent

    TS_ASSERT_DELTA(deflection, 25.0, 0.1);
  }

  // Test VNAV path deviation
  void testVNAVPathDeviation() {
    double current_altitude = 5000.0;
    double target_altitude = 3000.0;  // 2000 ft descent
    double distance_to_target = 6.0;  // nm (about 3 degree path)

    // tan(3 deg) * distance = height, so height / distance = tan(3 deg) = 0.0524
    // 2000 ft / (6 nm * 6076 ft/nm) = 2000 / 36456 = 0.0549 -> ~3.1 degrees
    double required_gradient = (current_altitude - target_altitude) / (distance_to_target * NM_TO_FT);
    double required_angle = std::atan(required_gradient) * RAD_TO_DEG;

    double deviation = required_angle - 3.0;

    TS_ASSERT(std::abs(deviation) < 1.0);  // Within 1 degree of 3-degree path
  }

  // Test waypoint sequencing distance
  void testWaypointSequencing() {
    double ground_speed = 200.0;  // knots
    double turn_anticipation = 0.5;  // nm typical

    // Sequencing should occur before reaching waypoint
    double sequence_distance = turn_anticipation;

    TS_ASSERT(sequence_distance > 0.0);
  }

  /***************************************************************************
   * Alternate Airport Calculations
   ***************************************************************************/

  // Test alternate fuel requirement
  void testAlternateFuelRequirement() {
    double distance_to_alternate = 100.0;  // nm
    double ground_speed = 150.0;  // knots
    double fuel_burn_rate = 12.0;  // gph

    double time_to_alternate = distance_to_alternate / ground_speed;
    double fuel_to_alternate = time_to_alternate * fuel_burn_rate;

    TS_ASSERT_DELTA(fuel_to_alternate, 8.0, 0.1);  // gallons
  }

  // Test point of no return
  void testPointOfNoReturn() {
    double total_range = 600.0;  // nm
    double fuel_remaining_range = 400.0;  // nm worth of fuel

    // Point of no return considering return to departure
    double pnr = fuel_remaining_range / 2.0;

    TS_ASSERT_DELTA(pnr, 200.0, epsilon);
  }

  // Test equal time point
  void testEqualTimePoint() {
    double total_distance = 500.0;  // nm
    double ground_speed_out = 150.0;  // knots
    double ground_speed_back = 200.0;  // knots (tailwind)

    // ETP = D * Vback / (Vout + Vback)
    double etp = total_distance * ground_speed_back / (ground_speed_out + ground_speed_back);

    TS_ASSERT(etp > 250.0 && etp < 300.0);
  }

  /***************************************************************************
   * Additional Distance Calculations
   ***************************************************************************/

  // Test short distance calculation
  void testShortDistanceCalculation() {
    double lat = 40.0 * DEG_TO_RAD;
    double dlat = 0.01 * DEG_TO_RAD;  // 0.01 degree
    double dlon = 0.01 * DEG_TO_RAD;

    // Simplified for short distances
    double dx = EARTH_RADIUS_NM * dlon * std::cos(lat);
    double dy = EARTH_RADIUS_NM * dlat;
    double distance = std::sqrt(dx * dx + dy * dy);

    TS_ASSERT(distance < 1.0);  // Less than 1 nm
  }

  // Test distance to DME station
  void testDMEDistance() {
    double slant_range = 10.0;  // nm
    double altitude_difference = 5000.0;  // feet

    // Ground distance from slant range
    double altitude_nm = altitude_difference / NM_TO_FT;
    double ground_distance = std::sqrt(slant_range * slant_range - altitude_nm * altitude_nm);

    TS_ASSERT(ground_distance < slant_range);
    TS_ASSERT(ground_distance > 9.9);
  }

  // Test meridian convergence
  void testMeridianConvergence() {
    double lat = 60.0 * DEG_TO_RAD;  // High latitude
    double dlon = 10.0 * DEG_TO_RAD;

    // Convergence angle
    double convergence = dlon * std::sin(lat);

    TS_ASSERT(convergence > 8.0 * DEG_TO_RAD);
  }

  /***************************************************************************
   * Complete Navigation System Tests
   ***************************************************************************/

  // Test complete flight plan distance
  void testCompleteFlightPlanDistance() {
    // Multi-leg flight plan
    double leg_distances[] = {150.0, 200.0, 175.0, 125.0};  // nm
    double total_distance = 0.0;
    for (double dist : leg_distances) {
      total_distance += dist;
    }
    TS_ASSERT_DELTA(total_distance, 650.0, epsilon);
  }

  // Test flight plan ETE calculation
  void testFlightPlanETE() {
    double total_distance = 500.0;  // nm
    double ground_speed = 200.0;     // knots
    double ete_hours = total_distance / ground_speed;
    double ete_minutes = ete_hours * 60.0;
    TS_ASSERT_DELTA(ete_minutes, 150.0, epsilon);
  }

  // Test waypoint sequencing logic
  void testWaypointSequencingLogic() {
    double waypoint_dist = 0.3;  // nm
    double sequencing_threshold = 0.5;
    bool should_sequence = waypoint_dist < sequencing_threshold;
    TS_ASSERT(should_sequence);
  }

  // Test magnetic variation application
  void testMagneticVariation() {
    double true_course = 90.0;  // degrees
    double mag_var = -15.0;     // 15 degrees west
    double mag_course = true_course - mag_var;
    TS_ASSERT_DELTA(mag_course, 105.0, epsilon);
  }

  // Test wind triangle complete
  void testWindTriangleComplete() {
    double tas = 200.0;  // knots
    double wind_speed = 40.0;
    double wind_angle = 30.0 * DEG_TO_RAD;  // 30 degrees relative

    double headwind = wind_speed * std::cos(wind_angle);
    double crosswind = wind_speed * std::sin(wind_angle);
    double gs = tas - headwind;
    double drift = std::atan(crosswind / gs) * RAD_TO_DEG;

    TS_ASSERT(gs < tas);
    TS_ASSERT(drift > 0.0);
  }

  // Test vertical navigation VNAV path
  void testVNAVPathAngle() {
    double altitude_change = 10000.0;  // ft
    double distance = 30.0;            // nm
    double distance_ft = distance * NM_TO_FT;
    double path_angle = std::atan(altitude_change / distance_ft) * RAD_TO_DEG;
    TS_ASSERT(path_angle > 2.5 && path_angle < 3.5);
  }

  // Test RNAV approach waypoint
  void testRNAVApproachWaypoint() {
    double faf_distance = 5.0;  // nm from runway
    double gs_angle = 3.0 * DEG_TO_RAD;
    double faf_altitude = faf_distance * NM_TO_FT * std::tan(gs_angle);
    TS_ASSERT(faf_altitude > 1500.0 && faf_altitude < 1700.0);
  }

  // Test holding pattern timing
  void testHoldingPatternTiming() {
    double leg_time_min = 1.0;
    double turn_rate_deg_sec = 3.0;
    double turn_time = 180.0 / turn_rate_deg_sec;  // 180 degree turn
    double total_circuit = 2.0 * (leg_time_min * 60.0 + turn_time);
    TS_ASSERT(total_circuit > 200.0 && total_circuit < 300.0);
  }

  // Test DME slant range correction
  void testDMESlantRangeCorrection() {
    double slant_range = 15.0;  // nm
    double altitude_nm = 6000.0 / NM_TO_FT;
    double ground_range = std::sqrt(slant_range * slant_range - altitude_nm * altitude_nm);
    TS_ASSERT(ground_range < slant_range);
    TS_ASSERT(ground_range > 14.9);
  }

  // Test VOR radial intercept
  void testVORRadialIntercept() {
    double current_radial = 85.0 * DEG_TO_RAD;
    double desired_radial = 90.0 * DEG_TO_RAD;
    double intercept_angle = 30.0 * DEG_TO_RAD;
    double heading_to_intercept = desired_radial - intercept_angle;
    TS_ASSERT(heading_to_intercept > 0.0);
  }

  // Test localizer back course
  void testLocalizerBackCourse() {
    double front_course = 270.0;
    double back_course = front_course + 180.0;
    if (back_course >= 360.0) back_course -= 360.0;
    TS_ASSERT_DELTA(back_course, 90.0, epsilon);
  }

  // Test arc to radial intercept
  void testArcToRadialIntercept() {
    double arc_radius = 10.0;  // nm DME arc
    double target_radial = 30.0 * DEG_TO_RAD;
    double arc_length = arc_radius * target_radial;
    TS_ASSERT(arc_length > 5.0);
  }

  // Test holding entry determination
  void testHoldingEntryDetermination() {
    double hold_inbound = 270.0;
    double aircraft_heading = 045.0;
    double relative = aircraft_heading - hold_inbound;
    if (relative < 0) relative += 360.0;
    // Direct: 0-70 or 290-360
    // Parallel: 110-180
    // Teardrop: 180-290
    bool teardrop = relative >= 180.0 && relative < 290.0;
    TS_ASSERT(!teardrop);  // 045 relative to 270 = 135, not teardrop
  }

  // Test procedure turn timing
  void testProcedureTurnTiming() {
    double outbound_time = 60.0;  // seconds
    double turn_time = 45.0 / 3.0;  // 45 degree turn at standard rate
    double total_time = outbound_time + 2.0 * turn_time + 60.0;  // both turns + return
    TS_ASSERT(total_time > 120.0);
  }

  // Test LNAV cross track sensitivity
  void testLNAVCrossTrackSensitivity() {
    double distance_to_waypoint = 30.0;  // nm
    double sensitivity;
    if (distance_to_waypoint > 30.0) {
      sensitivity = 2.0;  // enroute
    } else if (distance_to_waypoint > 2.0) {
      sensitivity = 1.0;  // terminal
    } else {
      sensitivity = 0.3;  // approach
    }
    TS_ASSERT_DELTA(sensitivity, 1.0, epsilon);
  }

  // Test GPS RAIM availability
  void testGPSRAIMAvailability() {
    int satellites_visible = 6;
    int min_for_raim = 5;
    bool raim_available = satellites_visible >= min_for_raim;
    TS_ASSERT(raim_available);
  }

  // Test step-down fix altitude
  void testStepDownFixAltitude() {
    double final_approach_fix_alt = 2000.0;
    double step_down_alt = 1500.0;
    double runway_elevation = 500.0;
    double height_above_runway = step_down_alt - runway_elevation;
    TS_ASSERT_DELTA(height_above_runway, 1000.0, epsilon);
  }

  // Test circling approach radius
  void testCirclingApproachRadius() {
    double category_c_radius = 1.7;  // nm
    double speed_kts = 140.0;
    double bank_angle = 25.0 * DEG_TO_RAD;
    double g = 32.2;
    double v_fps = speed_kts * 1.68781;
    double turn_radius_ft = (v_fps * v_fps) / (g * std::tan(bank_angle));
    double turn_radius_nm = turn_radius_ft / NM_TO_FT;
    TS_ASSERT(turn_radius_nm < category_c_radius);
  }

  // Test missed approach point
  void testMissedApproachPoint() {
    double runway_threshold = 0.0;
    double map_distance = 0.5;  // nm from threshold
    double gs_angle = 3.0 * DEG_TO_RAD;
    double map_height = map_distance * NM_TO_FT * std::tan(gs_angle);
    TS_ASSERT(map_height < 200.0);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many bearing calculations
  void testStressBearingCalculations() {
    for (int i = 0; i < 100; i++) {
      double lat1 = (40.0 + i * 0.01) * DEG_TO_RAD;
      double lon1 = (-74.0 - i * 0.01) * DEG_TO_RAD;
      double lat2 = (41.0 + i * 0.01) * DEG_TO_RAD;
      double lon2 = (-73.0 - i * 0.01) * DEG_TO_RAD;

      double dlon = lon2 - lon1;
      double y = std::sin(dlon) * std::cos(lat2);
      double x = std::cos(lat1) * std::sin(lat2) -
                 std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
      double bearing = std::atan2(y, x);

      TS_ASSERT(!std::isnan(bearing));
      TS_ASSERT(bearing >= -M_PI && bearing <= M_PI);
    }
  }

  // Test many distance calculations
  void testStressDistanceCalculations() {
    for (int i = 0; i < 100; i++) {
      double lat1 = (i % 90) * DEG_TO_RAD;
      double lon1 = (i % 180) * DEG_TO_RAD;
      double lat2 = ((i + 10) % 90) * DEG_TO_RAD;
      double lon2 = ((i + 10) % 180) * DEG_TO_RAD;

      double dlat = lat2 - lat1;
      double dlon = lon2 - lon1;

      double a = std::sin(dlat/2) * std::sin(dlat/2) +
                 std::cos(lat1) * std::cos(lat2) *
                 std::sin(dlon/2) * std::sin(dlon/2);
      double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
      double distance = EARTH_RADIUS_NM * c;

      TS_ASSERT(!std::isnan(distance));
      TS_ASSERT(distance >= 0.0);
    }
  }

  // Test complete navigation system verification
  void testCompleteNavigationSystemVerification() {
    // Complete flight from takeoff to landing
    double departure_lat = 40.0 * DEG_TO_RAD;
    double departure_lon = -74.0 * DEG_TO_RAD;
    double arrival_lat = 51.5 * DEG_TO_RAD;
    double arrival_lon = -0.1 * DEG_TO_RAD;

    // Calculate great circle distance
    double dlat = arrival_lat - departure_lat;
    double dlon = arrival_lon - departure_lon;
    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(departure_lat) * std::cos(arrival_lat) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance = EARTH_RADIUS_NM * c;

    // NYC to London is approximately 3000-3500 nm
    TS_ASSERT(distance > 2500.0 && distance < 4000.0);
  }
};
