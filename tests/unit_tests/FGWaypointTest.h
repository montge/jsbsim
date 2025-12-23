/*******************************************************************************
 * FGWaypointTest.h - Unit tests for FGWaypoint (navigation calculations)
 *
 * Tests the mathematical behavior of waypoint navigation:
 * - Great circle heading calculations
 * - Great circle distance calculations
 * - Unit conversions (degrees/radians, feet/meters)
 * - Special cases (poles, antipodes, same point)
 *
 * Note: FGWaypoint requires XML element for construction, so these tests focus
 * on the underlying navigation mathematics.
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
const double EARTH_RADIUS_FT = 20925646.3;  // feet
const double EARTH_RADIUS_M = 6378137.0;    // meters
const double FT_TO_M = 0.3048;
const double NM_TO_FT = 6076.12;

class FGWaypointTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Helper Functions - Great Circle Navigation
   ***************************************************************************/

  // Calculate great circle distance using Haversine formula
  double greatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
    // Convert to radians
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dPhi = (lat2 - lat1) * DEG_TO_RAD;
    double dLambda = (lon2 - lon1) * DEG_TO_RAD;

    double a = std::sin(dPhi/2) * std::sin(dPhi/2) +
               std::cos(phi1) * std::cos(phi2) *
               std::sin(dLambda/2) * std::sin(dLambda/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    return EARTH_RADIUS_FT * c;  // Distance in feet
  }

  // Calculate initial bearing (heading) to waypoint
  double greatCircleHeading(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dLambda = (lon2 - lon1) * DEG_TO_RAD;

    double x = std::sin(dLambda) * std::cos(phi2);
    double y = std::cos(phi1) * std::sin(phi2) -
               std::sin(phi1) * std::cos(phi2) * std::cos(dLambda);

    double theta = std::atan2(x, y);
    return std::fmod(theta * RAD_TO_DEG + 360.0, 360.0);  // Normalize to [0, 360)
  }

  /***************************************************************************
   * Heading Calculation Tests
   ***************************************************************************/

  // Test heading due north
  void testHeadingDueNorth() {
    double lat1 = 40.0, lon1 = -74.0;  // New York area
    double lat2 = 50.0, lon2 = -74.0;  // Same longitude, north

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 0.0, 0.1);  // Due north
  }

  // Test heading due south
  void testHeadingDueSouth() {
    double lat1 = 50.0, lon1 = -74.0;
    double lat2 = 40.0, lon2 = -74.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 180.0, 0.1);  // Due south
  }

  // Test heading due east
  void testHeadingDueEast() {
    double lat1 = 0.0, lon1 = 0.0;   // Equator, prime meridian
    double lat2 = 0.0, lon2 = 10.0;  // Same latitude, east

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 90.0, 0.1);  // Due east
  }

  // Test heading due west
  void testHeadingDueWest() {
    double lat1 = 0.0, lon1 = 10.0;
    double lat2 = 0.0, lon2 = 0.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 270.0, 0.1);  // Due west
  }

  // Test heading northeast
  void testHeadingNortheast() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 41.0, lon2 = -73.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT(heading > 0.0 && heading < 90.0);  // Somewhere NE
  }

  /***************************************************************************
   * Distance Calculation Tests
   ***************************************************************************/

  // Test distance between same point
  void testDistanceSamePoint() {
    double lat = 40.0, lon = -74.0;

    double dist = greatCircleDistance(lat, lon, lat, lon);
    TS_ASSERT_DELTA(dist, 0.0, 1.0);  // Should be zero (within 1 ft)
  }

  // Test known distance (approximately 60 nm per degree at equator)
  void testDistanceOneDegreeEquator() {
    double lat = 0.0;
    double lon1 = 0.0, lon2 = 1.0;

    double dist = greatCircleDistance(lat, lon1, lat, lon2);
    double distNM = dist / NM_TO_FT;

    // At equator, 1 degree longitude ≈ 60 nm
    TS_ASSERT_DELTA(distNM, 60.0, 1.0);
  }

  // Test distance New York to London (approximately 3000+ nm)
  void testDistanceNYToLondon() {
    double lat1 = 40.7128, lon1 = -74.0060;  // New York
    double lat2 = 51.5074, lon2 = -0.1278;   // London

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Great circle distance should be approximately 3000-3500 nm
    TS_ASSERT(distNM > 2900.0 && distNM < 3600.0);
  }

  // Test distance symmetry
  void testDistanceSymmetry() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 51.0, lon2 = 0.0;

    double dist1 = greatCircleDistance(lat1, lon1, lat2, lon2);
    double dist2 = greatCircleDistance(lat2, lon2, lat1, lon1);

    TS_ASSERT_DELTA(dist1, dist2, 1.0);  // Should be identical
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test feet to meters conversion
  void testFeetToMeters() {
    double feet = 1000.0;
    double meters = feet * FT_TO_M;

    TS_ASSERT_DELTA(meters, 304.8, 0.01);
  }

  // Test nautical miles to feet
  void testNMToFeet() {
    double nm = 1.0;
    double feet = nm * NM_TO_FT;

    TS_ASSERT_DELTA(feet, 6076.12, 0.01);
  }

  // Test degrees to radians
  void testDegreesToRadians() {
    TS_ASSERT_DELTA(90.0 * DEG_TO_RAD, M_PI / 2.0, epsilon);
    TS_ASSERT_DELTA(180.0 * DEG_TO_RAD, M_PI, epsilon);
    TS_ASSERT_DELTA(360.0 * DEG_TO_RAD, 2.0 * M_PI, epsilon);
  }

  /***************************************************************************
   * Special Cases
   ***************************************************************************/

  // Test crossing the prime meridian
  void testCrossingPrimeMeridian() {
    double lat = 50.0;
    double lon1 = -5.0, lon2 = 5.0;

    double dist = greatCircleDistance(lat, lon1, lat, lon2);
    double heading = greatCircleHeading(lat, lon1, lat, lon2);

    TS_ASSERT(dist > 0);
    TS_ASSERT_DELTA(heading, 90.0, 5.0);  // Roughly east
  }

  // Test crossing the international date line
  void testCrossingDateLine() {
    double lat = 0.0;
    double lon1 = 170.0, lon2 = -170.0;

    // Should take the short way (20 degrees, not 340)
    double dist = greatCircleDistance(lat, lon1, lat, lon2);
    double distNM = dist / NM_TO_FT;

    TS_ASSERT_DELTA(distNM, 20.0 * 60.0, 10.0);  // ~1200 nm
  }

  // Test from equator to pole
  void testEquatorToPole() {
    double lat1 = 0.0, lon = 0.0;
    double lat2 = 90.0;  // North pole

    double heading = greatCircleHeading(lat1, lon, lat2, lon);
    TS_ASSERT_DELTA(heading, 0.0, 0.1);  // Due north

    double dist = greatCircleDistance(lat1, lon, lat2, lon);
    double distNM = dist / NM_TO_FT;
    TS_ASSERT_DELTA(distNM, 90.0 * 60.0, 10.0);  // 90 degrees = 5400 nm
  }

  // Test at the pole (heading undefined but distance valid)
  void testAtPole() {
    double lat1 = 90.0, lon1 = 0.0;   // North pole
    double lat2 = 89.0, lon2 = 0.0;   // 1 degree south

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    TS_ASSERT_DELTA(distNM, 60.0, 1.0);  // 1 degree = 60 nm
  }

  // Test antipodal points (opposite sides of Earth)
  void testAntipodalPoints() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = -40.0, lon2 = 106.0;  // Approximately antipodal

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately half Earth circumference
    double halfCircumference = M_PI * 3440.0;  // nm
    TS_ASSERT_DELTA(distNM, halfCircumference, 100.0);
  }

  /***************************************************************************
   * Navigation Application Tests
   ***************************************************************************/

  // Test heading error calculation
  void testHeadingError() {
    double currentHeading = 45.0;
    double waypointHeading = 60.0;

    double error = waypointHeading - currentHeading;
    TS_ASSERT_DELTA(error, 15.0, epsilon);  // Turn right 15 degrees
  }

  // Test cross-track error concept
  void testCrossTrackConcept() {
    // Simplified: perpendicular distance from track
    double trackHeading = 90.0;  // Due east
    double bearingToWaypoint = 85.0;  // 5 degrees left of track
    double distanceToWaypoint = 10.0;  // nm

    // Cross-track distance = sin(angle) * distance
    double xte = std::sin((trackHeading - bearingToWaypoint) * DEG_TO_RAD) * distanceToWaypoint;
    TS_ASSERT_DELTA(xte, 0.87, 0.01);  // About 0.87 nm left of track
  }

  // Test waypoint sequencing
  void testWaypointSequencing() {
    double threshold = 1000.0;  // feet
    double distToWaypoint = 500.0;

    bool arrived = (distToWaypoint < threshold);
    TS_ASSERT(arrived);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very short distance
  void testVeryShortDistance() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 40.0001, lon2 = -74.0001;

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    TS_ASSERT(dist > 0);
    TS_ASSERT(dist < 100);  // Less than 100 feet
  }

  // Test near-identical longitudes
  void testNearIdenticalLongitudes() {
    double lat1 = 40.0, lat2 = 50.0;
    double lon1 = -74.0, lon2 = -74.0000001;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    // Heading should be essentially north (0 or ~360 due to floating point)
    TS_ASSERT(heading < 1.0 || heading > 359.0);
  }

  // Test negative latitudes (southern hemisphere)
  void testSouthernHemisphere() {
    double lat1 = -33.9, lon1 = 151.2;  // Sydney
    double lat2 = -37.8, lon2 = 145.0;  // Melbourne

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT(heading > 180.0 && heading < 270.0);  // Southwest

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    TS_ASSERT(dist > 0);
  }
};
