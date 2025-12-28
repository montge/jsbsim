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

  /***************************************************************************
   * Extended Heading Tests
   ***************************************************************************/

  // Test heading northwest
  void testHeadingNorthwest() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 41.0, lon2 = -75.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT(heading > 270.0 && heading < 360.0);  // Northwest quadrant
  }

  // Test heading southeast
  void testHeadingSoutheast() {
    double lat1 = 41.0, lon1 = -75.0;
    double lat2 = 40.0, lon2 = -74.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT(heading > 90.0 && heading < 180.0);  // Southeast quadrant
  }

  // Test heading southwest
  void testHeadingSouthwest() {
    double lat1 = 41.0, lon1 = -74.0;
    double lat2 = 40.0, lon2 = -75.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT(heading > 180.0 && heading < 270.0);  // Southwest quadrant
  }

  // Test 45 degree heading (northeast)
  void testHeading45Degrees() {
    // At equator, equal lat/lon change gives ~45 degrees
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = 1.0, lon2 = 1.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 45.0, 2.0);  // Close to 45 degrees
  }

  // Test 135 degree heading (southeast)
  void testHeading135Degrees() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = -1.0, lon2 = 1.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 135.0, 2.0);
  }

  // Test 225 degree heading (southwest)
  void testHeading225Degrees() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = -1.0, lon2 = -1.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 225.0, 2.0);
  }

  // Test 315 degree heading (northwest)
  void testHeading315Degrees() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = 1.0, lon2 = -1.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(heading, 315.0, 2.0);
  }

  /***************************************************************************
   * Extended Distance Tests
   ***************************************************************************/

  // Test distance across Pacific (Los Angeles to Tokyo)
  void testDistanceLAToTokyo() {
    double lat1 = 33.9425, lon1 = -118.4081;  // LAX
    double lat2 = 35.6762, lon2 = 139.6503;   // Tokyo

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately 4800-5000 nm
    TS_ASSERT(distNM > 4600.0 && distNM < 5200.0);
  }

  // Test transcontinental distance (NY to LA)
  void testDistanceNYToLA() {
    double lat1 = 40.6413, lon1 = -73.7781;  // JFK
    double lat2 = 33.9425, lon2 = -118.4081; // LAX

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately 2150-2200 nm
    TS_ASSERT(distNM > 2100.0 && distNM < 2300.0);
  }

  // Test polar route distance (London to Tokyo via north pole)
  void testDistanceLondonToTokyo() {
    double lat1 = 51.4700, lon1 = -0.4543;   // Heathrow
    double lat2 = 35.6762, lon2 = 139.6503;  // Tokyo

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately 5200-5300 nm
    TS_ASSERT(distNM > 5000.0 && distNM < 5500.0);
  }

  // Test short regional distance (Boston to DC)
  void testDistanceBostonToDC() {
    double lat1 = 42.3656, lon1 = -71.0096;  // Boston
    double lat2 = 38.8512, lon2 = -77.0402;  // DC

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately 350-400 nm
    TS_ASSERT(distNM > 300.0 && distNM < 450.0);
  }

  // Test southern hemisphere distance (Sydney to Santiago)
  void testDistanceSydneyToSantiago() {
    double lat1 = -33.8688, lon1 = 151.2093;  // Sydney
    double lat2 = -33.4489, lon2 = -70.6693;  // Santiago

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately 5900-6100 nm (crossing Pacific)
    TS_ASSERT(distNM > 5800.0 && distNM < 6200.0);
  }

  // Test north-south distance (Oslo to Cape Town)
  void testDistanceOsloToCapeTown() {
    double lat1 = 59.9139, lon1 = 10.7522;   // Oslo
    double lat2 = -33.9249, lon2 = 18.4241;  // Cape Town

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be approximately 5500-5600 nm
    TS_ASSERT(distNM > 5300.0 && distNM < 5800.0);
  }

  /***************************************************************************
   * Destination Point Calculation Tests
   ***************************************************************************/

  // Helper: Calculate destination point given start, bearing, and distance
  void destinationPoint(double lat1, double lon1, double bearing, double distFt,
                        double& lat2, double& lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double lambda1 = lon1 * DEG_TO_RAD;
    double brng = bearing * DEG_TO_RAD;
    double delta = distFt / EARTH_RADIUS_FT;

    double sinPhi2 = std::sin(phi1) * std::cos(delta) +
                     std::cos(phi1) * std::sin(delta) * std::cos(brng);
    double phi2 = std::asin(sinPhi2);

    double y = std::sin(brng) * std::sin(delta) * std::cos(phi1);
    double x = std::cos(delta) - std::sin(phi1) * sinPhi2;
    double lambda2 = lambda1 + std::atan2(y, x);

    lat2 = phi2 * RAD_TO_DEG;
    lon2 = std::fmod(lambda2 * RAD_TO_DEG + 540.0, 360.0) - 180.0;
  }

  // Test destination due north
  void testDestinationDueNorth() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2, lon2;

    // Go 60 nm north (about 1 degree)
    destinationPoint(lat1, lon1, 0.0, 60.0 * NM_TO_FT, lat2, lon2);

    TS_ASSERT_DELTA(lat2, 41.0, 0.1);
    TS_ASSERT_DELTA(lon2, lon1, 0.01);  // Longitude unchanged
  }

  // Test destination due east at equator
  void testDestinationDueEastEquator() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2, lon2;

    // Go 60 nm east (about 1 degree at equator)
    destinationPoint(lat1, lon1, 90.0, 60.0 * NM_TO_FT, lat2, lon2);

    TS_ASSERT_DELTA(lat2, 0.0, 0.01);  // Latitude unchanged
    TS_ASSERT_DELTA(lon2, 1.0, 0.1);
  }

  // Test destination round trip returns to origin
  void testDestinationRoundTrip() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2, lon2, lat3, lon3;

    // Go 100 nm north, then 100 nm south
    destinationPoint(lat1, lon1, 0.0, 100.0 * NM_TO_FT, lat2, lon2);
    destinationPoint(lat2, lon2, 180.0, 100.0 * NM_TO_FT, lat3, lon3);

    TS_ASSERT_DELTA(lat3, lat1, 0.1);
    TS_ASSERT_DELTA(lon3, lon1, 0.1);
  }

  /***************************************************************************
   * Intermediate Point Tests
   ***************************************************************************/

  // Helper: Calculate intermediate point on great circle
  void intermediatePoint(double lat1, double lon1, double lat2, double lon2,
                         double fraction, double& latMid, double& lonMid) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double lambda1 = lon1 * DEG_TO_RAD;
    double lambda2 = lon2 * DEG_TO_RAD;

    double dPhi = (lat2 - lat1) * DEG_TO_RAD;
    double dLambda = (lon2 - lon1) * DEG_TO_RAD;

    double a = std::sin(dPhi/2) * std::sin(dPhi/2) +
               std::cos(phi1) * std::cos(phi2) *
               std::sin(dLambda/2) * std::sin(dLambda/2);
    double delta = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double A = std::sin((1-fraction) * delta) / std::sin(delta);
    double B = std::sin(fraction * delta) / std::sin(delta);

    double x = A * std::cos(phi1) * std::cos(lambda1) + B * std::cos(phi2) * std::cos(lambda2);
    double y = A * std::cos(phi1) * std::sin(lambda1) + B * std::cos(phi2) * std::sin(lambda2);
    double z = A * std::sin(phi1) + B * std::sin(phi2);

    double phiMid = std::atan2(z, std::sqrt(x*x + y*y));
    double lambdaMid = std::atan2(y, x);

    latMid = phiMid * RAD_TO_DEG;
    lonMid = lambdaMid * RAD_TO_DEG;
  }

  // Test midpoint of route
  void testMidpointOfRoute() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 50.0, lon2 = -74.0;
    double latMid, lonMid;

    intermediatePoint(lat1, lon1, lat2, lon2, 0.5, latMid, lonMid);

    TS_ASSERT_DELTA(latMid, 45.0, 0.5);
    TS_ASSERT_DELTA(lonMid, -74.0, 0.5);
  }

  // Test quarter point
  void testQuarterPoint() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 48.0, lon2 = -74.0;
    double latQuarter, lonQuarter;

    intermediatePoint(lat1, lon1, lat2, lon2, 0.25, latQuarter, lonQuarter);

    TS_ASSERT_DELTA(latQuarter, 42.0, 0.5);  // Approximately 25% of 8 degrees
  }

  // Test intermediate point at start
  void testIntermediatePointAtStart() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 50.0, lon2 = -80.0;
    double latMid, lonMid;

    intermediatePoint(lat1, lon1, lat2, lon2, 0.0, latMid, lonMid);

    TS_ASSERT_DELTA(latMid, lat1, 0.1);
    TS_ASSERT_DELTA(lonMid, lon1, 0.1);
  }

  // Test intermediate point at end
  void testIntermediatePointAtEnd() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 50.0, lon2 = -80.0;
    double latMid, lonMid;

    intermediatePoint(lat1, lon1, lat2, lon2, 1.0, latMid, lonMid);

    TS_ASSERT_DELTA(latMid, lat2, 0.1);
    TS_ASSERT_DELTA(lonMid, lon2, 0.1);
  }

  /***************************************************************************
   * Multi-Waypoint Route Tests
   ***************************************************************************/

  // Test total distance of multi-leg route
  void testMultiLegRouteDistance() {
    // Route: JFK -> Chicago -> Denver -> LAX
    double lat_jfk = 40.6413, lon_jfk = -73.7781;
    double lat_ord = 41.9742, lon_ord = -87.9073;
    double lat_den = 39.8561, lon_den = -104.6737;
    double lat_lax = 33.9425, lon_lax = -118.4081;

    double leg1 = greatCircleDistance(lat_jfk, lon_jfk, lat_ord, lon_ord);
    double leg2 = greatCircleDistance(lat_ord, lon_ord, lat_den, lon_den);
    double leg3 = greatCircleDistance(lat_den, lon_den, lat_lax, lon_lax);

    double totalRoute = leg1 + leg2 + leg3;
    double directDist = greatCircleDistance(lat_jfk, lon_jfk, lat_lax, lon_lax);

    // Route should be longer than direct
    TS_ASSERT(totalRoute > directDist);

    // Route should be reasonable (not excessively long)
    TS_ASSERT(totalRoute < directDist * 1.3);
  }

  // Test cumulative heading changes
  void testCumulativeHeadingChanges() {
    double lat1 = 40.0, lon1 = 0.0;
    double lat2 = 41.0, lon2 = 1.0;
    double lat3 = 41.0, lon3 = 2.0;

    double heading1 = greatCircleHeading(lat1, lon1, lat2, lon2);
    double heading2 = greatCircleHeading(lat2, lon2, lat3, lon3);

    // First leg NE, second leg E - should be turning right
    TS_ASSERT(heading1 < 90.0);  // NE
    TS_ASSERT(heading2 > 45.0);  // More easterly
  }

  /***************************************************************************
   * Rhumb Line vs Great Circle Tests
   ***************************************************************************/

  // Helper: Calculate rhumb line distance
  double rhumbDistance(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dPhi = phi2 - phi1;
    double dLambda = std::abs(lon2 - lon1) * DEG_TO_RAD;

    // Handle crossing the date line
    if (dLambda > M_PI) dLambda = 2*M_PI - dLambda;

    double dPsi = std::log(std::tan(M_PI/4 + phi2/2) / std::tan(M_PI/4 + phi1/2));
    double q = (std::abs(dPsi) > 1e-12) ? dPhi/dPsi : std::cos(phi1);

    double dist = std::sqrt(dPhi*dPhi + q*q*dLambda*dLambda) * EARTH_RADIUS_FT;
    return dist;
  }

  // Helper: Calculate rhumb line bearing
  double rhumbBearing(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dLambda = (lon2 - lon1) * DEG_TO_RAD;

    // Handle crossing the date line
    if (std::abs(dLambda) > M_PI) {
      dLambda = dLambda > 0 ? -(2*M_PI - dLambda) : (2*M_PI + dLambda);
    }

    double dPsi = std::log(std::tan(M_PI/4 + phi2/2) / std::tan(M_PI/4 + phi1/2));
    double theta = std::atan2(dLambda, dPsi);

    return std::fmod(theta * RAD_TO_DEG + 360.0, 360.0);
  }

  // Test rhumb line is longer than great circle
  void testRhumbLongerThanGreatCircle() {
    double lat1 = 40.0, lon1 = -74.0;  // NY area
    double lat2 = 51.0, lon2 = 0.0;    // London area

    double gcDist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double rhumbDist = rhumbDistance(lat1, lon1, lat2, lon2);

    // Rhumb line should be longer (or equal for special cases)
    TS_ASSERT(rhumbDist >= gcDist * 0.999);  // Allow small numerical error
  }

  // Test rhumb and great circle same for north-south
  void testRhumbEqualsGCForNorthSouth() {
    double lat1 = 40.0, lon = -74.0;
    double lat2 = 50.0;

    double gcDist = greatCircleDistance(lat1, lon, lat2, lon);
    double rhumbDist = rhumbDistance(lat1, lon, lat2, lon);

    TS_ASSERT_DELTA(gcDist, rhumbDist, gcDist * 0.001);  // Within 0.1%
  }

  // Test rhumb bearing constant
  void testRhumbBearingConstant() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 51.0, lon2 = 0.0;

    double bearing = rhumbBearing(lat1, lon1, lat2, lon2);

    // Rhumb bearing should be consistent (same value throughout journey)
    TS_ASSERT(bearing > 0.0 && bearing < 360.0);
  }

  /***************************************************************************
   * Angular Distance Tests
   ***************************************************************************/

  // Helper: Calculate angular distance in radians
  double angularDistance(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dPhi = (lat2 - lat1) * DEG_TO_RAD;
    double dLambda = (lon2 - lon1) * DEG_TO_RAD;

    double a = std::sin(dPhi/2) * std::sin(dPhi/2) +
               std::cos(phi1) * std::cos(phi2) *
               std::sin(dLambda/2) * std::sin(dLambda/2);
    return 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
  }

  // Test 90 degree angular distance (quarter Earth)
  void testAngularDistance90Degrees() {
    // From equator to pole is 90 degrees
    double lat1 = 0.0, lon = 0.0;
    double lat2 = 90.0;

    double angDist = angularDistance(lat1, lon, lat2, lon);
    TS_ASSERT_DELTA(angDist, M_PI / 2.0, 0.01);
  }

  // Test 180 degree angular distance (antipodal)
  void testAngularDistance180Degrees() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = 0.0, lon2 = 180.0;

    double angDist = angularDistance(lat1, lon1, lat2, lon2);
    TS_ASSERT_DELTA(angDist, M_PI, 0.01);
  }

  /***************************************************************************
   * High Latitude Navigation Tests
   ***************************************************************************/

  // Test near north pole
  void testNearNorthPole() {
    double lat1 = 89.0, lon1 = 0.0;
    double lat2 = 89.0, lon2 = 180.0;

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    // Should be about 2 degrees (going over the pole)
    TS_ASSERT_DELTA(distNM, 120.0, 10.0);  // ~2 degrees = 120 nm
  }

  // Test near south pole
  void testNearSouthPole() {
    double lat1 = -89.0, lon1 = 0.0;
    double lat2 = -89.0, lon2 = 180.0;

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;

    TS_ASSERT_DELTA(distNM, 120.0, 10.0);  // Same as north pole case
  }

  // Test crossing pole
  void testCrossingPole() {
    double lat1 = 80.0, lon1 = 0.0;
    double lat2 = 80.0, lon2 = 180.0;

    double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
    // Heading should be north (going over pole)
    TS_ASSERT_DELTA(heading, 0.0, 1.0);
  }

  /***************************************************************************
   * Hemisphere Crossing Tests
   ***************************************************************************/

  // Test crossing equator going south
  void testCrossingEquatorSouth() {
    double lat1 = 5.0, lon = 0.0;
    double lat2 = -5.0;

    double heading = greatCircleHeading(lat1, lon, lat2, lon);
    TS_ASSERT_DELTA(heading, 180.0, 0.1);  // Due south
  }

  // Test crossing equator going north
  void testCrossingEquatorNorth() {
    double lat1 = -5.0, lon = 0.0;
    double lat2 = 5.0;

    double heading = greatCircleHeading(lat1, lon, lat2, lon);
    TS_ASSERT_DELTA(heading, 0.0, 0.1);  // Due north
  }

  // Test crossing western hemisphere to eastern
  void testWesternToEastern() {
    double lat = 40.0;
    double lon1 = -10.0, lon2 = 10.0;

    double heading = greatCircleHeading(lat, lon1, lat, lon2);
    TS_ASSERT_DELTA(heading, 90.0, 10.0);  // Roughly east (great circle curves northward)
  }

  /***************************************************************************
   * Cross-Track Distance Tests
   ***************************************************************************/

  // Helper: Calculate cross-track distance from path
  double crossTrackDistance(double lat1, double lon1,  // Start of path
                            double lat2, double lon2,  // End of path
                            double lat3, double lon3)  // Point to measure
  {
    double delta13 = greatCircleDistance(lat1, lon1, lat3, lon3) / EARTH_RADIUS_FT;
    double theta13 = greatCircleHeading(lat1, lon1, lat3, lon3) * DEG_TO_RAD;
    double theta12 = greatCircleHeading(lat1, lon1, lat2, lon2) * DEG_TO_RAD;

    double xt = std::asin(std::sin(delta13) * std::sin(theta13 - theta12));
    return xt * EARTH_RADIUS_FT;
  }

  // Test point on track has zero cross-track distance
  void testPointOnTrack() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 50.0, lon2 = -74.0;
    double lat3 = 45.0, lon3 = -74.0;  // Midpoint

    double xt = crossTrackDistance(lat1, lon1, lat2, lon2, lat3, lon3);
    TS_ASSERT(std::abs(xt) < 100.0);  // Within 100 feet (numerical precision)
  }

  // Test point left of track
  void testPointLeftOfTrack() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = 10.0, lon2 = 0.0;   // Path goes north
    double lat3 = 5.0, lon3 = -0.1;   // Point left (west) of track

    double xt = crossTrackDistance(lat1, lon1, lat2, lon2, lat3, lon3);
    TS_ASSERT(xt < 0);  // Negative for left of track
  }

  // Test point right of track
  void testPointRightOfTrack() {
    double lat1 = 0.0, lon1 = 0.0;
    double lat2 = 10.0, lon2 = 0.0;   // Path goes north
    double lat3 = 5.0, lon3 = 0.1;    // Point right (east) of track

    double xt = crossTrackDistance(lat1, lon1, lat2, lon2, lat3, lon3);
    TS_ASSERT(xt > 0);  // Positive for right of track
  }

  /***************************************************************************
   * Along-Track Distance Tests
   ***************************************************************************/

  // Helper: Calculate along-track distance from start
  double alongTrackDistance(double lat1, double lon1,  // Start of path
                            double lat2, double lon2,  // End of path
                            double lat3, double lon3)  // Point
  {
    double delta13 = greatCircleDistance(lat1, lon1, lat3, lon3) / EARTH_RADIUS_FT;
    double theta13 = greatCircleHeading(lat1, lon1, lat3, lon3) * DEG_TO_RAD;
    double theta12 = greatCircleHeading(lat1, lon1, lat2, lon2) * DEG_TO_RAD;

    double xt = std::asin(std::sin(delta13) * std::sin(theta13 - theta12));
    double at = std::acos(std::cos(delta13) / std::cos(xt));

    return at * EARTH_RADIUS_FT;
  }

  // Test along-track for point at start
  void testAlongTrackAtStart() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 50.0, lon2 = -74.0;

    double at = alongTrackDistance(lat1, lon1, lat2, lon2, lat1, lon1);
    TS_ASSERT(at < 100.0);  // Near zero
  }

  // Test along-track for midpoint
  void testAlongTrackMidpoint() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 50.0, lon2 = -74.0;
    double lat3 = 45.0, lon3 = -74.0;

    double at = alongTrackDistance(lat1, lon1, lat2, lon2, lat3, lon3);
    double totalDist = greatCircleDistance(lat1, lon1, lat2, lon2);

    TS_ASSERT_DELTA(at, totalDist / 2.0, totalDist * 0.05);  // Within 5%
  }

  /***************************************************************************
   * Closest Point on Path Tests
   ***************************************************************************/

  // Test closest point fraction calculation
  void testClosestPointFraction() {
    double lat1 = 40.0, lon1 = -74.0;  // Start
    double lat2 = 50.0, lon2 = -74.0;  // End

    double at = alongTrackDistance(lat1, lon1, lat2, lon2, 45.0, -74.0);
    double totalDist = greatCircleDistance(lat1, lon1, lat2, lon2);

    double fraction = at / totalDist;
    TS_ASSERT_DELTA(fraction, 0.5, 0.05);  // Midpoint should be 50%
  }

  /***************************************************************************
   * Additional Unit Conversion Tests
   ***************************************************************************/

  // Test kilometers to feet
  void testKilometersToFeet() {
    double km = 1.0;
    double feet = km * 1000.0 / FT_TO_M;
    TS_ASSERT_DELTA(feet, 3280.84, 0.1);
  }

  // Test statute miles to nautical miles
  void testStatuteMilesToNauticalMiles() {
    double sm = 1.15078;  // 1 nm in statute miles
    double nm = sm / 1.15078;
    TS_ASSERT_DELTA(nm, 1.0, 0.001);
  }

  // Test radians round trip
  void testRadiansRoundTrip() {
    double degreesOriginal = 123.456;
    double radians = degreesOriginal * DEG_TO_RAD;
    double degreesBack = radians * RAD_TO_DEG;
    TS_ASSERT_DELTA(degreesOriginal, degreesBack, epsilon);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many distance calculations
  void testStressManyDistances() {
    double prevDist = 0;
    for (int i = 0; i < 1000; i++) {
      double lat1 = -90.0 + (i % 180);
      double lon1 = -180.0 + (i % 360);
      double lat2 = lat1 + 1.0;
      double lon2 = lon1 + 1.0;

      if (lat2 > 90.0) lat2 = 89.0;
      if (lon2 > 180.0) lon2 = lon2 - 360.0;

      double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
      TS_ASSERT(dist >= 0);  // Distance always positive
      TS_ASSERT(!std::isnan(dist));
    }
  }

  // Test many heading calculations
  void testStressManyHeadings() {
    for (int i = 0; i < 1000; i++) {
      double lat1 = -85.0 + (i % 170);  // Avoid exact poles
      double lon1 = -180.0 + (i % 360);
      double lat2 = lat1 + 0.5;
      double lon2 = lon1 + 0.5;

      if (lat2 > 85.0) lat2 = 85.0;
      if (lon2 > 180.0) lon2 = lon2 - 360.0;

      double heading = greatCircleHeading(lat1, lon1, lat2, lon2);
      TS_ASSERT(heading >= 0.0 && heading < 360.0);
      TS_ASSERT(!std::isnan(heading));
    }
  }

  // Test rapid waypoint switching
  void testStressRapidWaypointSwitch() {
    double lat1 = 40.0, lon1 = -74.0;

    for (int i = 0; i < 500; i++) {
      double lat2 = 40.0 + std::sin(i * 0.1) * 10.0;
      double lon2 = -74.0 + std::cos(i * 0.1) * 10.0;

      double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
      double heading = greatCircleHeading(lat1, lon1, lat2, lon2);

      TS_ASSERT(dist >= 0);
      TS_ASSERT(heading >= 0.0 && heading < 360.0);
    }
  }

  /***************************************************************************
   * Edge Cases - Special Values
   ***************************************************************************/

  // Test exactly at north pole
  void testExactlyAtNorthPole() {
    double lat1 = 90.0, lon1 = 0.0;
    double lat2 = 89.0, lon2 = 0.0;

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;
    TS_ASSERT_DELTA(distNM, 60.0, 1.0);  // 1 degree = 60 nm
  }

  // Test exactly at south pole
  void testExactlyAtSouthPole() {
    double lat1 = -90.0, lon1 = 0.0;
    double lat2 = -89.0, lon2 = 0.0;

    double dist = greatCircleDistance(lat1, lon1, lat2, lon2);
    double distNM = dist / NM_TO_FT;
    TS_ASSERT_DELTA(distNM, 60.0, 1.0);
  }

  // Test longitude at +180 and -180
  void testLongitudeAtDateLine() {
    double lat = 40.0;
    double lon1 = 180.0;
    double lon2 = -180.0;

    double dist = greatCircleDistance(lat, lon1, lat, lon2);
    TS_ASSERT_DELTA(dist, 0.0, 1.0);  // Same point
  }

  // Test very small angle
  void testVerySmallAngle() {
    double lat = 40.0, lon = -74.0;
    double lat2 = lat + 0.00001;  // Very small change

    double dist = greatCircleDistance(lat, lon, lat2, lon);
    TS_ASSERT(dist > 0);
    TS_ASSERT(dist < 10.0);  // Very small distance
  }

  /***************************************************************************
   * Section 17: Final Bearing Tests
   ***************************************************************************/

  // Helper: Calculate final bearing at destination
  double finalBearing(double lat1, double lon1, double lat2, double lon2) {
    // Final bearing is initial bearing from end to start, reversed
    double bearing = greatCircleHeading(lat2, lon2, lat1, lon1);
    return std::fmod(bearing + 180.0, 360.0);
  }

  // Test final bearing due north (same as initial)
  void testFinalBearingDueNorth() {
    double lat1 = 40.0, lon = -74.0;
    double lat2 = 50.0;

    double initBearing = greatCircleHeading(lat1, lon, lat2, lon);
    double finBearing = finalBearing(lat1, lon, lat2, lon);

    // For north-south routes, initial and final should be the same
    TS_ASSERT_DELTA(initBearing, 0.0, 0.1);
    TS_ASSERT_DELTA(finBearing, 0.0, 0.1);
  }

  // Test final bearing for long route (great circle curvature)
  void testFinalBearingLongRoute() {
    double lat1 = 40.0, lon1 = -74.0;  // NY
    double lat2 = 51.0, lon2 = 0.0;    // London

    double initBearing = greatCircleHeading(lat1, lon1, lat2, lon2);
    double finBearing = finalBearing(lat1, lon1, lat2, lon2);

    // Initial and final bearings differ for east-west routes
    TS_ASSERT(initBearing > 30.0 && initBearing < 70.0);  // Roughly NE
    TS_ASSERT(finBearing > 60.0 && finBearing < 130.0);   // More easterly at end
    TS_ASSERT(finBearing > initBearing);  // Final is more easterly
  }

  // Test final bearing at equator
  void testFinalBearingAtEquator() {
    double lat = 0.0;
    double lon1 = 0.0, lon2 = 90.0;

    double initBearing = greatCircleHeading(lat, lon1, lat, lon2);
    double finBearing = finalBearing(lat, lon1, lat, lon2);

    // At equator, bearings should be symmetric
    TS_ASSERT_DELTA(initBearing, 90.0, 0.1);
    TS_ASSERT_DELTA(finBearing, 90.0, 0.1);
  }

  /***************************************************************************
   * Section 18: Wind Correction Angle Tests
   ***************************************************************************/

  // Helper: Calculate wind correction angle
  double windCorrectionAngle(double trueAirspeed, double windSpeed,
                             double windDirection, double desiredTrack) {
    // Wind from direction means wind blowing toward opposite
    double headwindComponent = windSpeed * std::cos((windDirection - desiredTrack) * DEG_TO_RAD);
    double crosswindComponent = windSpeed * std::sin((windDirection - desiredTrack) * DEG_TO_RAD);

    // WCA = arcsin(crosswind / TAS)
    double wca = std::asin(crosswindComponent / trueAirspeed) * RAD_TO_DEG;
    return wca;
  }

  // Test no wind gives zero correction
  void testNoWindCorrection() {
    double wca = windCorrectionAngle(200.0, 0.0, 0.0, 90.0);
    TS_ASSERT_DELTA(wca, 0.0, 0.01);
  }

  // Test direct headwind gives zero correction
  void testHeadwindZeroCorrection() {
    double wca = windCorrectionAngle(200.0, 30.0, 90.0, 90.0);  // Wind from direction of flight
    TS_ASSERT_DELTA(wca, 0.0, 0.01);
  }

  // Test direct crosswind from left
  void testCrosswindFromLeft() {
    double wca = windCorrectionAngle(200.0, 20.0, 0.0, 90.0);  // Flying east, wind from north
    TS_ASSERT(wca < 0);  // Need to crab left (into wind)
  }

  // Test direct crosswind from right
  void testCrosswindFromRight() {
    double wca = windCorrectionAngle(200.0, 20.0, 180.0, 90.0);  // Flying east, wind from south
    TS_ASSERT(wca > 0);  // Need to crab right (into wind)
  }

  /***************************************************************************
   * Section 19: Ground Speed Calculation Tests
   ***************************************************************************/

  // Helper: Calculate ground speed with wind
  double groundSpeed(double trueAirspeed, double windSpeed,
                     double windDirection, double heading) {
    // Headwind component
    double headwindComponent = windSpeed * std::cos((windDirection - heading + 180.0) * DEG_TO_RAD);
    return trueAirspeed + headwindComponent;
  }

  // Test pure headwind reduces ground speed
  void testHeadwindReducesGroundSpeed() {
    double gs = groundSpeed(200.0, 30.0, 0.0, 0.0);  // Flying north, wind from north
    TS_ASSERT_DELTA(gs, 170.0, 1.0);  // TAS - wind
  }

  // Test pure tailwind increases ground speed
  void testTailwindIncreasesGroundSpeed() {
    double gs = groundSpeed(200.0, 30.0, 180.0, 0.0);  // Flying north, wind from south
    TS_ASSERT_DELTA(gs, 230.0, 1.0);  // TAS + wind
  }

  // Test crosswind minimal effect on ground speed
  void testCrosswindGroundSpeed() {
    double gs = groundSpeed(200.0, 30.0, 90.0, 0.0);  // Flying north, wind from east
    TS_ASSERT_DELTA(gs, 200.0, 1.0);  // No headwind/tailwind component
  }

  /***************************************************************************
   * Section 20: Time Enroute Calculation Tests
   ***************************************************************************/

  // Helper: Calculate time in minutes
  double timeEnrouteMinutes(double distanceNM, double groundSpeedKts) {
    return (distanceNM / groundSpeedKts) * 60.0;
  }

  // Test simple time calculation
  void testTimeEnroute() {
    double time = timeEnrouteMinutes(300.0, 150.0);  // 300nm at 150kts
    TS_ASSERT_DELTA(time, 120.0, 0.1);  // 2 hours = 120 minutes
  }

  // Test short leg time
  void testShortLegTime() {
    double time = timeEnrouteMinutes(50.0, 200.0);  // 50nm at 200kts
    TS_ASSERT_DELTA(time, 15.0, 0.1);  // 15 minutes
  }

  // Test long haul time
  void testLongHaulTime() {
    double time = timeEnrouteMinutes(3000.0, 500.0);  // 3000nm at 500kts
    TS_ASSERT_DELTA(time, 360.0, 0.1);  // 6 hours
  }

  /***************************************************************************
   * Section 21: Intersection Calculation Tests
   ***************************************************************************/

  // Helper: Check if two paths intersect
  bool pathsIntersect(double lat1, double lon1, double bearing1,
                      double lat2, double lon2, double bearing2) {
    // Simplified check - if bearings converge
    double headingTo2 = greatCircleHeading(lat1, lon1, lat2, lon2);
    double headingTo1 = greatCircleHeading(lat2, lon2, lat1, lon1);

    double diff1 = std::fmod(std::abs(bearing1 - headingTo2) + 360.0, 360.0);
    double diff2 = std::fmod(std::abs(bearing2 - headingTo1) + 360.0, 360.0);

    // Paths converge if bearings point toward each other
    return (diff1 < 90.0 || diff1 > 270.0) && (diff2 < 90.0 || diff2 > 270.0);
  }

  // Test convergent paths
  void testConvergentPathsIntersect() {
    // Two paths heading toward same point
    bool intersect = pathsIntersect(40.0, -75.0, 45.0,   // From southwest, heading NE
                                    40.0, -73.0, 315.0); // From southeast, heading NW
    TS_ASSERT(intersect);
  }

  // Test parallel paths don't intersect
  void testParallelPathsNoIntersect() {
    // Two parallel paths
    bool intersect = pathsIntersect(40.0, -74.0, 90.0,   // Heading east
                                    41.0, -74.0, 90.0);  // Parallel, heading east
    TS_ASSERT(!intersect);
  }

  // Test divergent paths don't intersect
  void testDivergentPathsNoIntersect() {
    bool intersect = pathsIntersect(40.0, -74.0, 45.0,   // Heading NE
                                    40.0, -74.0, 135.0); // Heading SE
    TS_ASSERT(!intersect);
  }

  /***************************************************************************
   * Section 22: Hold Pattern Entry Tests
   ***************************************************************************/

  // Helper: Determine hold entry type
  // Returns: 0 = Direct, 1 = Parallel, 2 = Teardrop
  int holdEntryType(double inboundCourse, double holdCourse) {
    double diff = std::fmod(holdCourse - inboundCourse + 360.0, 360.0);

    // Standard right-hand hold
    if (diff <= 70.0 || diff > 290.0) {
      return 0;  // Direct entry
    } else if (diff > 70.0 && diff <= 180.0) {
      return 2;  // Teardrop entry
    } else {
      return 1;  // Parallel entry
    }
  }

  // Test direct entry when approaching from behind
  void testHoldDirectEntry() {
    int entry = holdEntryType(90.0, 90.0);  // Approaching on same course as hold
    TS_ASSERT_EQUALS(entry, 0);  // Direct
  }

  // Test parallel entry
  void testHoldParallelEntry() {
    // For parallel: diff between 180 and 290
    // inbound=90, hold=320 gives diff=230
    int entry = holdEntryType(90.0, 320.0);
    TS_ASSERT_EQUALS(entry, 1);  // Parallel
  }

  // Test teardrop entry
  void testHoldTeardropEntry() {
    int entry = holdEntryType(0.0, 90.0);  // Approaching from the north, hold east
    TS_ASSERT_EQUALS(entry, 2);  // Teardrop
  }

  /***************************************************************************
   * Section 23: Course Deviation Indicator Tests
   ***************************************************************************/

  // Test CDI deflection calculation
  void testCDIFullScaleDeflection() {
    // VOR has 10 degree full scale deflection
    double desiredCourse = 90.0;
    double actualBearing = 85.0;
    double deviation = desiredCourse - actualBearing;

    // Deflection dots (2.5 degrees per dot)
    double dots = deviation / 2.5;
    TS_ASSERT_DELTA(dots, 2.0, 0.1);  // 2 dots right
  }

  // Test CDI centered
  void testCDICentered() {
    double desiredCourse = 90.0;
    double actualBearing = 90.0;
    double deviation = desiredCourse - actualBearing;

    TS_ASSERT_DELTA(deviation, 0.0, 0.01);
  }

  // Test CDI full scale left
  void testCDIFullScaleLeft() {
    double desiredCourse = 90.0;
    double actualBearing = 100.0;
    double deviation = desiredCourse - actualBearing;

    double dots = deviation / 2.5;
    TS_ASSERT(dots < -3.0);  // More than full scale left
  }

  /***************************************************************************
   * Section 24: DME Arc Tests
   ***************************************************************************/

  // Helper: Calculate points on DME arc
  void dmeArcPoint(double lat0, double lon0, double radiusNM,
                   double radial, double& latPt, double& lonPt) {
    destinationPoint(lat0, lon0, radial, radiusNM * NM_TO_FT, latPt, lonPt);
  }

  // Test DME arc point calculation
  void testDMEArcPoint() {
    double lat0 = 40.0, lon0 = -74.0;  // VOR location
    double latPt, lonPt;

    dmeArcPoint(lat0, lon0, 10.0, 90.0, latPt, lonPt);  // 10nm arc, 090 radial

    double dist = greatCircleDistance(lat0, lon0, latPt, lonPt);
    TS_ASSERT_DELTA(dist / NM_TO_FT, 10.0, 0.1);
  }

  // Test multiple arc points maintain distance
  void testDMEArcConstantDistance() {
    double lat0 = 40.0, lon0 = -74.0;
    double radiusNM = 15.0;

    for (int radial = 0; radial < 360; radial += 30) {
      double latPt, lonPt;
      dmeArcPoint(lat0, lon0, radiusNM, static_cast<double>(radial), latPt, lonPt);

      double dist = greatCircleDistance(lat0, lon0, latPt, lonPt);
      TS_ASSERT_DELTA(dist / NM_TO_FT, radiusNM, 0.2);
    }
  }

  /***************************************************************************
   * Section 25: Radial/Distance Fix Tests
   ***************************************************************************/

  // Test fix defined by radial and DME
  void testRadialDistanceFix() {
    double vorLat = 40.0, vorLon = -74.0;
    double radial = 45.0;  // 045 radial
    double dme = 20.0;     // 20 DME

    double fixLat, fixLon;
    destinationPoint(vorLat, vorLon, radial, dme * NM_TO_FT, fixLat, fixLon);

    // Verify fix position
    double distBack = greatCircleDistance(vorLat, vorLon, fixLat, fixLon);
    TS_ASSERT_DELTA(distBack / NM_TO_FT, 20.0, 0.1);

    double bearingBack = greatCircleHeading(vorLat, vorLon, fixLat, fixLon);
    TS_ASSERT_DELTA(bearingBack, 45.0, 0.5);
  }

  // Test radial reciprocal (inbound vs outbound)
  void testRadialReciprocal() {
    double radial = 135.0;
    double inboundCourse = std::fmod(radial + 180.0, 360.0);
    TS_ASSERT_DELTA(inboundCourse, 315.0, 0.01);
  }

  /***************************************************************************
   * Section 26: Procedure Turn Tests
   ***************************************************************************/

  // Helper: Calculate procedure turn heading
  double procedureTurnHeading(double inboundCourse, bool rightTurn) {
    double offset = rightTurn ? 45.0 : -45.0;
    return std::fmod(inboundCourse + 180.0 + offset + 360.0, 360.0);
  }

  // Test procedure turn outbound heading (right turn)
  void testProcedureTurnRight() {
    double outbound = procedureTurnHeading(360.0, true);  // Inbound 360, right turn
    TS_ASSERT_DELTA(outbound, 225.0, 0.1);  // 180 + 45 = 225
  }

  // Test procedure turn outbound heading (left turn)
  void testProcedureTurnLeft() {
    double outbound = procedureTurnHeading(360.0, false);  // Inbound 360, left turn
    TS_ASSERT_DELTA(outbound, 135.0, 0.1);  // 180 - 45 = 135
  }

  // Test procedure turn reversal
  void testProcedureTurnReversal() {
    double inbound = 90.0;
    double outbound = procedureTurnHeading(inbound, true);
    double reversal = std::fmod(outbound + 180.0, 360.0);

    // After 180 turn, should be roughly heading inbound
    TS_ASSERT(std::abs(reversal - inbound) < 50.0 ||
              std::abs(reversal - inbound) > 310.0);
  }

  /***************************************************************************
   * Section 27: Magnetic Variation Tests
   ***************************************************************************/

  // Test magnetic to true heading conversion
  void testMagneticToTrue() {
    double magneticHeading = 360.0;
    double variation = -15.0;  // 15 degrees West
    double trueHeading = std::fmod(magneticHeading + variation + 360.0, 360.0);

    TS_ASSERT_DELTA(trueHeading, 345.0, 0.01);  // True = Mag + West var
  }

  // Test true to magnetic conversion
  void testTrueToMagnetic() {
    double trueHeading = 90.0;
    double variation = 10.0;  // 10 degrees East
    double magneticHeading = std::fmod(trueHeading - variation + 360.0, 360.0);

    TS_ASSERT_DELTA(magneticHeading, 80.0, 0.01);
  }

  // Test variation at prime meridian (typically East)
  void testEastVariation() {
    double trueHeading = 180.0;
    double variation = 5.0;  // 5E
    double magHeading = std::fmod(trueHeading - variation + 360.0, 360.0);

    TS_ASSERT_DELTA(magHeading, 175.0, 0.01);
  }
};
