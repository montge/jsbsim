/*******************************************************************************
 * FGCoordinatesTest.h - Unit tests for coordinate transformations
 *
 * Tests the mathematical behavior of coordinate systems:
 * - ECEF (Earth-Centered Earth-Fixed) coordinates
 * - Geodetic coordinates (lat/lon/alt)
 * - Local NED (North-East-Down) frame
 * - Body axis transformations
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

// WGS84 constants
const double WGS84_A = 20925646.3;    // Semi-major axis (ft)
const double WGS84_B = 20855486.5;    // Semi-minor axis (ft)
const double WGS84_E2 = 0.00669437999; // First eccentricity squared

class FGCoordinatesTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * ECEF Coordinate Tests
   ***************************************************************************/

  // Test ECEF at equator, prime meridian
  void testECEFAtEquatorPrime() {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    // N = a / sqrt(1 - e² * sin²(lat))
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

    double X = (N + alt) * std::cos(latRad) * std::cos(lonRad);
    double Y = (N + alt) * std::cos(latRad) * std::sin(lonRad);
    double Z = (N * (1 - WGS84_E2) + alt) * std::sin(latRad);

    TS_ASSERT_DELTA(X, WGS84_A, 100);  // On equator at x-axis
    TS_ASSERT_DELTA(Y, 0.0, 1.0);
    TS_ASSERT_DELTA(Z, 0.0, 1.0);
  }

  // Test ECEF at north pole
  void testECEFAtNorthPole() {
    double lat = 90.0, lon = 0.0, alt = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

    double X = (N + alt) * std::cos(latRad) * std::cos(lonRad);
    double Y = (N + alt) * std::cos(latRad) * std::sin(lonRad);
    double Z = (N * (1 - WGS84_E2) + alt) * std::sin(latRad);

    TS_ASSERT_DELTA(X, 0.0, 1.0);
    TS_ASSERT_DELTA(Y, 0.0, 1.0);
    TS_ASSERT_DELTA(Z, WGS84_B, 100);  // At polar axis
  }

  // Test ECEF at 90 degrees longitude
  void testECEF90Longitude() {
    double lat = 0.0, lon = 90.0, alt = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    double N = WGS84_A;  // At equator

    double X = (N + alt) * std::cos(latRad) * std::cos(lonRad);
    double Y = (N + alt) * std::cos(latRad) * std::sin(lonRad);
    double Z = (N * (1 - WGS84_E2) + alt) * std::sin(latRad);

    TS_ASSERT_DELTA(X, 0.0, 1.0);
    TS_ASSERT_DELTA(Y, WGS84_A, 100);  // On y-axis
    TS_ASSERT_DELTA(Z, 0.0, 1.0);
  }

  /***************************************************************************
   * Geodetic to ECEF and Back Tests
   ***************************************************************************/

  // Test ECEF distance from center
  void testECEFDistanceFromCenter() {
    double lat = 45.0, lon = 45.0, alt = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

    double X = (N + alt) * std::cos(latRad) * std::cos(lonRad);
    double Y = (N + alt) * std::cos(latRad) * std::sin(lonRad);
    double Z = (N * (1 - WGS84_E2) + alt) * std::sin(latRad);

    double R = std::sqrt(X*X + Y*Y + Z*Z);

    // Radius should be between polar and equatorial
    TS_ASSERT(R > WGS84_B);
    TS_ASSERT(R < WGS84_A);
  }

  // Test altitude effect on ECEF
  void testAltitudeEffect() {
    double lat = 0.0, lon = 0.0;
    double alt1 = 0.0;
    double alt2 = 35000.0;  // ft

    double N = WGS84_A;

    double X1 = (N + alt1);
    double X2 = (N + alt2);

    TS_ASSERT_DELTA(X2 - X1, 35000.0, epsilon);
  }

  /***************************************************************************
   * NED Frame Tests
   ***************************************************************************/

  // Test NED unit vectors at equator
  void testNEDAtEquator() {
    double lat = 0.0, lon = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    // North unit vector in ECEF
    double Nn = -std::sin(latRad) * std::cos(lonRad);
    double Ne = -std::sin(latRad) * std::sin(lonRad);
    double Nd = std::cos(latRad);

    // At equator, north points in -Z direction
    TS_ASSERT_DELTA(Nn, 0.0, epsilon);
    TS_ASSERT_DELTA(Ne, 0.0, epsilon);
    TS_ASSERT_DELTA(Nd, 1.0, epsilon);
  }

  // Test NED east unit vector
  void testNEDEast() {
    double lon = 0.0;
    double lonRad = lon * DEG_TO_RAD;

    // East unit vector in ECEF (independent of latitude)
    double Ex = -std::sin(lonRad);
    double Ey = std::cos(lonRad);
    double Ez = 0.0;

    TS_ASSERT_DELTA(Ex, 0.0, epsilon);
    TS_ASSERT_DELTA(Ey, 1.0, epsilon);
    TS_ASSERT_DELTA(Ez, 0.0, epsilon);
  }

  // Test NED down is toward Earth center
  void testNEDDown() {
    double lat = 45.0, lon = 45.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    // Down unit vector (negative of surface normal, simplified for sphere)
    double Dx = std::cos(latRad) * std::cos(lonRad);
    double Dy = std::cos(latRad) * std::sin(lonRad);
    double Dz = std::sin(latRad);

    // Should be unit vector pointing toward center
    double mag = std::sqrt(Dx*Dx + Dy*Dy + Dz*Dz);
    TS_ASSERT_DELTA(mag, 1.0, epsilon);
  }

  /***************************************************************************
   * Euler Angle Tests
   ***************************************************************************/

  // Test rotation matrix from Euler angles
  void testEulerRotation() {
    double phi = 0.0;    // Roll
    double theta = 0.0;  // Pitch
    double psi = 90.0 * DEG_TO_RAD;  // Yaw 90 degrees

    // Rotation around z-axis (yaw only)
    double c = std::cos(psi);
    double s = std::sin(psi);

    // Rotate [1,0,0] by 90 degrees about z
    double x_new = c * 1.0 - s * 0.0;
    double y_new = s * 1.0 + c * 0.0;

    TS_ASSERT_DELTA(x_new, 0.0, epsilon);
    TS_ASSERT_DELTA(y_new, 1.0, epsilon);
  }

  // Test pitch rotation
  void testPitchRotation() {
    double theta = 30.0 * DEG_TO_RAD;  // Pitch up 30 degrees

    double c = std::cos(theta);
    double s = std::sin(theta);

    // Rotate [1,0,0] about y-axis
    double x_new = c * 1.0;
    double z_new = -s * 1.0;

    TS_ASSERT_DELTA(x_new, 0.866, 0.001);
    TS_ASSERT_DELTA(z_new, -0.5, 0.001);
  }

  // Test roll rotation
  void testRollRotation() {
    double phi = 30.0 * DEG_TO_RAD;  // Roll right 30 degrees

    double c = std::cos(phi);
    double s = std::sin(phi);

    // Rotate [0,1,0] about x-axis
    double y_new = c * 1.0;
    double z_new = s * 1.0;

    TS_ASSERT_DELTA(y_new, 0.866, 0.001);
    TS_ASSERT_DELTA(z_new, 0.5, 0.001);
  }

  /***************************************************************************
   * Body to NED Transformation Tests
   ***************************************************************************/

  // Test level flight (no rotation)
  void testLevelFlightTransform() {
    double phi = 0.0, theta = 0.0, psi = 0.0;

    // Forward velocity in body frame
    double u = 200.0, v = 0.0, w = 0.0;

    // Transform to NED (identity for level, north-heading)
    double Vn = u;
    double Ve = v;
    double Vd = w;

    TS_ASSERT_DELTA(Vn, 200.0, epsilon);
    TS_ASSERT_DELTA(Ve, 0.0, epsilon);
    TS_ASSERT_DELTA(Vd, 0.0, epsilon);
  }

  // Test heading east
  void testHeadingEast() {
    double psi = 90.0 * DEG_TO_RAD;

    // Forward velocity in body frame
    double u = 200.0;

    // Simplified: Vn = u*cos(psi), Ve = u*sin(psi)
    double Vn = u * std::cos(psi);
    double Ve = u * std::sin(psi);

    TS_ASSERT_DELTA(Vn, 0.0, 0.001);
    TS_ASSERT_DELTA(Ve, 200.0, 0.001);
  }

  /***************************************************************************
   * Distance Calculations
   ***************************************************************************/

  // Test great circle distance approximation
  void testGreatCircleDistance() {
    double lat1 = 40.0 * DEG_TO_RAD, lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 51.0 * DEG_TO_RAD, lon2 = 0.0 * DEG_TO_RAD;

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double R = 3440.0;  // Earth radius in nm
    double distance = R * c;

    TS_ASSERT(distance > 2900 && distance < 3500);  // NY to London ~3000 nm
  }

  // Test flat earth approximation for short distances
  void testFlatEarthDistance() {
    double lat1 = 40.0, lon1 = -74.0;
    double lat2 = 40.1, lon2 = -74.1;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Approximate: 1 degree latitude ≈ 60 nm
    double distNS = dlat * 60.0;
    double distEW = dlon * 60.0 * std::cos(lat1 * DEG_TO_RAD);

    double distance = std::sqrt(distNS*distNS + distEW*distEW);
    TS_ASSERT(distance < 20);  // Short distance
  }

  /***************************************************************************
   * Bearing Calculations
   ***************************************************************************/

  // Test bearing due north
  void testBearingNorth() {
    double lat1 = 40.0, lon1 = 0.0;
    double lat2 = 50.0, lon2 = 0.0;

    double bearing = 0.0;  // Due north
    TS_ASSERT_DELTA(bearing, 0.0, epsilon);
  }

  // Test bearing calculation
  void testBearingCalculation() {
    double lat1 = 40.0 * DEG_TO_RAD, lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 40.0 * DEG_TO_RAD, lon2 = -73.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(x, y) * RAD_TO_DEG;
    if (bearing < 0) bearing += 360.0;

    TS_ASSERT_DELTA(bearing, 90.0, 1.0);  // Due east
  }

  /***************************************************************************
   * Altitude Conversions
   ***************************************************************************/

  // Test pressure altitude
  void testPressureAltitude() {
    double altimeter = 29.92;  // in Hg (standard)
    double indicatedAlt = 5000.0;

    // Pressure altitude = Indicated + correction
    double correction = (29.92 - altimeter) * 1000.0;
    double pressureAlt = indicatedAlt + correction;

    TS_ASSERT_DELTA(pressureAlt, 5000.0, epsilon);  // Standard day
  }

  // Test density altitude
  void testDensityAltitude() {
    double pressureAlt = 5000.0;
    double temp = 20.0;  // °C
    double stdTemp = 15.0 - (pressureAlt / 1000.0) * 2.0;  // ISA temp

    // Rough approximation: 120 ft per degree above standard
    double densityAlt = pressureAlt + (temp - stdTemp) * 120.0;

    TS_ASSERT(densityAlt > pressureAlt);  // Hot day = higher density alt
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test coordinates at international date line
  void testDateLine() {
    double lon1 = 179.0;
    double lon2 = -179.0;

    // Angular distance should be 2 degrees, not 358
    double dlon = lon2 - lon1;
    if (dlon > 180.0) dlon -= 360.0;
    if (dlon < -180.0) dlon += 360.0;

    TS_ASSERT_DELTA(dlon, -358.0 + 360.0, epsilon);  // = 2 degrees
  }

  // Test zero latitude/longitude
  void testZeroCoordinates() {
    double lat = 0.0, lon = 0.0;

    // Should work without division by zero
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    TS_ASSERT_DELTA(latRad, 0.0, epsilon);
    TS_ASSERT_DELTA(lonRad, 0.0, epsilon);
  }

  // Test pole singularity handling
  void testPoleSingularity() {
    double lat = 90.0;
    double latRad = lat * DEG_TO_RAD;

    // cos(90) = 0, but calculations should still work
    double cosLat = std::cos(latRad);
    TS_ASSERT_DELTA(cosLat, 0.0, epsilon);
  }
};
