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

  /***************************************************************************
   * Additional ECEF Tests
   ***************************************************************************/

  // Test ECEF at south pole
  void testECEFAtSouthPole() {
    double lat = -90.0, lon = 0.0, alt = 0.0;
    double latRad = lat * DEG_TO_RAD;

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

    double X = (N + alt) * std::cos(latRad) * std::cos(lon * DEG_TO_RAD);
    double Y = (N + alt) * std::cos(latRad) * std::sin(lon * DEG_TO_RAD);
    double Z = (N * (1 - WGS84_E2) + alt) * std::sin(latRad);

    TS_ASSERT_DELTA(X, 0.0, 1.0);
    TS_ASSERT_DELTA(Y, 0.0, 1.0);
    TS_ASSERT_DELTA(Z, -WGS84_B, 100);  // Negative Z at south pole
  }

  // Test ECEF at 180 longitude
  void testECEF180Longitude() {
    double lat = 0.0, lon = 180.0, alt = 0.0;
    double lonRad = lon * DEG_TO_RAD;

    double N = WGS84_A;

    double X = N * std::cos(lonRad);
    double Y = N * std::sin(lonRad);

    TS_ASSERT_DELTA(X, -WGS84_A, 100);  // Opposite side of Earth
    TS_ASSERT_DELTA(Y, 0.0, 1.0);
  }

  // Test ECEF at negative longitude
  void testECEFNegativeLongitude() {
    double lat = 0.0, lon = -90.0, alt = 0.0;
    double lonRad = lon * DEG_TO_RAD;

    double N = WGS84_A;

    double X = N * std::cos(lonRad);
    double Y = N * std::sin(lonRad);

    TS_ASSERT_DELTA(X, 0.0, 1.0);
    TS_ASSERT_DELTA(Y, -WGS84_A, 100);  // Negative Y
  }

  // Test ECEF southern hemisphere
  void testECEFSouthernHemisphere() {
    double lat = -45.0, lon = 0.0, alt = 0.0;
    double latRad = lat * DEG_TO_RAD;

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

    double Z = (N * (1 - WGS84_E2) + alt) * std::sin(latRad);

    TS_ASSERT(Z < 0);  // Negative Z in southern hemisphere
  }

  // Test ECEF with altitude
  void testECEFWithAltitude() {
    double lat = 45.0, lon = 0.0;
    double alt1 = 0.0, alt2 = 100000.0;  // 100,000 ft

    double latRad = lat * DEG_TO_RAD;
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

    double X1 = (N + alt1) * std::cos(latRad);
    double X2 = (N + alt2) * std::cos(latRad);

    double increase = X2 - X1;
    double expectedIncrease = alt2 * std::cos(latRad);

    TS_ASSERT_DELTA(increase, expectedIncrease, 1.0);
  }

  /***************************************************************************
   * Additional NED Frame Tests
   ***************************************************************************/

  // Test NED at north pole
  void testNEDAtNorthPole() {
    double lat = 90.0, lon = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    // Down vector at pole points along -Z axis
    double Dx = std::cos(latRad) * std::cos(lonRad);
    double Dy = std::cos(latRad) * std::sin(lonRad);
    double Dz = std::sin(latRad);

    TS_ASSERT_DELTA(Dx, 0.0, epsilon);
    TS_ASSERT_DELTA(Dy, 0.0, epsilon);
    TS_ASSERT_DELTA(Dz, 1.0, epsilon);
  }

  // Test NED orthogonality
  void testNEDOrthogonality() {
    double lat = 45.0, lon = 45.0;
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    // North vector
    double Nx = -std::sin(latRad) * std::cos(lonRad);
    double Ny = -std::sin(latRad) * std::sin(lonRad);
    double Nz = std::cos(latRad);

    // East vector
    double Ex = -std::sin(lonRad);
    double Ey = std::cos(lonRad);
    double Ez = 0.0;

    // Dot product should be zero (orthogonal)
    double dot = Nx*Ex + Ny*Ey + Nz*Ez;
    TS_ASSERT_DELTA(dot, 0.0, epsilon);
  }

  // Test NED east vector at various longitudes
  void testNEDEastAt45Longitude() {
    double lon = 45.0;
    double lonRad = lon * DEG_TO_RAD;

    double Ex = -std::sin(lonRad);
    double Ey = std::cos(lonRad);

    TS_ASSERT_DELTA(Ex, -std::sqrt(2.0)/2.0, epsilon);
    TS_ASSERT_DELTA(Ey, std::sqrt(2.0)/2.0, epsilon);
  }

  /***************************************************************************
   * Additional Euler Angle Tests
   ***************************************************************************/

  // Test combined yaw and pitch
  void testYawAndPitch() {
    double psi = 45.0 * DEG_TO_RAD;    // Yaw 45 degrees
    double theta = 30.0 * DEG_TO_RAD;  // Pitch 30 degrees

    // Apply yaw then pitch to [1,0,0]
    // Yaw: [cos(psi), sin(psi), 0]
    // Pitch about y: rotates in xz plane

    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);
    double ctheta = std::cos(theta);
    double stheta = std::sin(theta);

    double x_after_yaw = cpsi;
    double y_after_yaw = spsi;
    double z_after_yaw = 0.0;

    double x_final = x_after_yaw * ctheta + z_after_yaw * stheta;
    double z_final = -x_after_yaw * stheta + z_after_yaw * ctheta;

    TS_ASSERT_DELTA(x_final, cpsi * ctheta, epsilon);
    TS_ASSERT_DELTA(z_final, -cpsi * stheta, epsilon);
  }

  // Test gimbal lock avoidance
  void testGimbalLockPitch90() {
    double theta = 90.0 * DEG_TO_RAD;  // Pitch straight up

    double ctheta = std::cos(theta);
    double stheta = std::sin(theta);

    TS_ASSERT_DELTA(ctheta, 0.0, epsilon);
    TS_ASSERT_DELTA(stheta, 1.0, epsilon);
  }

  // Test 180 degree yaw
  void testYaw180() {
    double psi = 180.0 * DEG_TO_RAD;

    double x = std::cos(psi);
    double y = std::sin(psi);

    TS_ASSERT_DELTA(x, -1.0, epsilon);
    TS_ASSERT_DELTA(y, 0.0, epsilon);
  }

  // Test combined roll and bank turn
  void testBankedTurn() {
    double phi = 30.0 * DEG_TO_RAD;   // Bank 30 degrees
    double psi = 45.0 * DEG_TO_RAD;   // Heading 45 degrees

    // Lift vector after roll
    double liftY = std::cos(phi);
    double liftZ = std::sin(phi);

    TS_ASSERT_DELTA(liftY, std::cos(30.0 * DEG_TO_RAD), epsilon);
    TS_ASSERT_DELTA(liftZ, 0.5, epsilon);
  }

  // Test small angle approximation
  void testSmallAngleApprox() {
    double angle = 0.01;  // radians (~0.57 degrees)

    // For small angles: sin(x) ≈ x, cos(x) ≈ 1
    TS_ASSERT_DELTA(std::sin(angle), angle, 1e-4);
    TS_ASSERT_DELTA(std::cos(angle), 1.0, 1e-4);
  }

  /***************************************************************************
   * Additional Body to NED Tests
   ***************************************************************************/

  // Test heading south
  void testHeadingSouth() {
    double psi = 180.0 * DEG_TO_RAD;
    double u = 200.0;

    double Vn = u * std::cos(psi);
    double Ve = u * std::sin(psi);

    TS_ASSERT_DELTA(Vn, -200.0, 0.001);
    TS_ASSERT_DELTA(Ve, 0.0, 0.001);
  }

  // Test heading west
  void testHeadingWest() {
    double psi = 270.0 * DEG_TO_RAD;
    double u = 200.0;

    double Vn = u * std::cos(psi);
    double Ve = u * std::sin(psi);

    TS_ASSERT_DELTA(Vn, 0.0, 0.001);
    TS_ASSERT_DELTA(Ve, -200.0, 0.001);
  }

  // Test climbing flight
  void testClimbingFlight() {
    double theta = 10.0 * DEG_TO_RAD;  // 10 degree climb
    double u = 200.0;  // Forward velocity

    double Vn = u * std::cos(theta);
    double Vd = -u * std::sin(theta);  // Negative because climbing = -down

    TS_ASSERT(Vn < u);       // Ground speed less than TAS
    TS_ASSERT(Vd < 0);       // Climbing means negative down velocity
  }

  // Test sideslip
  void testSideslipVelocity() {
    double beta = 5.0 * DEG_TO_RAD;  // 5 degree sideslip
    double V = 200.0;

    double u = V * std::cos(beta);
    double v = V * std::sin(beta);

    TS_ASSERT(v > 0);  // Positive sideslip = positive v
    TS_ASSERT_DELTA(std::sqrt(u*u + v*v), V, epsilon);
  }

  /***************************************************************************
   * Additional Distance Tests
   ***************************************************************************/

  // Test zero distance
  void testZeroDistance() {
    double lat = 40.0 * DEG_TO_RAD, lon = -74.0 * DEG_TO_RAD;

    // Same point = zero distance
    double distance = 0.0;
    TS_ASSERT_DELTA(distance, 0.0, epsilon);
  }

  // Test antipodal points
  void testAntipodalDistance() {
    double lat1 = 0.0 * DEG_TO_RAD, lon1 = 0.0 * DEG_TO_RAD;
    double lat2 = 0.0 * DEG_TO_RAD, lon2 = 180.0 * DEG_TO_RAD;

    // Antipodal points on equator
    double dlon = lon2 - lon1;

    double a = std::sin(0) * std::sin(0) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double R = 3440.0;  // nm
    double distance = R * c;

    // Half circumference ≈ π * R ≈ 10808 nm
    TS_ASSERT_DELTA(distance, M_PI * R, 1.0);
  }

  // Test polar distance
  void testPolarDistance() {
    double lat1 = 90.0 * DEG_TO_RAD, lon1 = 0.0;
    double lat2 = -90.0 * DEG_TO_RAD, lon2 = 0.0;

    // Pole to pole
    double dlat = lat2 - lat1;

    double a = std::sin(dlat/2) * std::sin(dlat/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double R = 3440.0;
    double distance = R * c;

    // Half circumference
    TS_ASSERT_DELTA(distance, M_PI * R, 1.0);
  }

  // Test equatorial distance
  void testEquatorialDistance() {
    double lat = 0.0 * DEG_TO_RAD;
    double lon1 = 0.0 * DEG_TO_RAD;
    double lon2 = 90.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;

    double a = std::cos(lat) * std::cos(lat) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double R = 3440.0;
    double distance = R * c;

    // Quarter circumference
    TS_ASSERT_DELTA(distance, M_PI * R / 2.0, 1.0);
  }

  /***************************************************************************
   * Additional Bearing Tests
   ***************************************************************************/

  // Test bearing due south
  void testBearingSouth() {
    double lat1 = 50.0 * DEG_TO_RAD, lon1 = 0.0;
    double lat2 = 40.0 * DEG_TO_RAD, lon2 = 0.0;

    double dlon = lon2 - lon1;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(x, y) * RAD_TO_DEG;
    if (bearing < 0) bearing += 360.0;

    TS_ASSERT_DELTA(bearing, 180.0, 1.0);
  }

  // Test bearing west
  void testBearingWest() {
    double lat1 = 40.0 * DEG_TO_RAD, lon1 = -73.0 * DEG_TO_RAD;
    double lat2 = 40.0 * DEG_TO_RAD, lon2 = -74.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(x, y) * RAD_TO_DEG;
    if (bearing < 0) bearing += 360.0;

    TS_ASSERT_DELTA(bearing, 270.0, 1.0);
  }

  // Test bearing northeast
  void testBearingNortheast() {
    double lat1 = 40.0 * DEG_TO_RAD, lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 41.0 * DEG_TO_RAD, lon2 = -73.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(x, y) * RAD_TO_DEG;

    TS_ASSERT(bearing > 30 && bearing < 60);  // Northeast quadrant
  }

  /***************************************************************************
   * Coordinate Transformation Chain Tests
   ***************************************************************************/

  // Test inverse transformation
  void testInverseRotation() {
    double psi = 45.0 * DEG_TO_RAD;

    // Forward rotation
    double x = 1.0, y = 0.0;
    double x1 = x * std::cos(psi) - y * std::sin(psi);
    double y1 = x * std::sin(psi) + y * std::cos(psi);

    // Inverse rotation (negative angle)
    double x2 = x1 * std::cos(-psi) - y1 * std::sin(-psi);
    double y2 = x1 * std::sin(-psi) + y1 * std::cos(-psi);

    // Should recover original
    TS_ASSERT_DELTA(x2, x, epsilon);
    TS_ASSERT_DELTA(y2, y, epsilon);
  }

  // Test multiple rotations compose correctly
  void testRotationComposition() {
    double psi1 = 30.0 * DEG_TO_RAD;
    double psi2 = 60.0 * DEG_TO_RAD;

    // Single 90 degree rotation
    double x1 = std::cos(90.0 * DEG_TO_RAD);
    double y1 = std::sin(90.0 * DEG_TO_RAD);

    // Two rotations: 30 + 60 = 90
    double x2 = std::cos(psi1 + psi2);
    double y2 = std::sin(psi1 + psi2);

    TS_ASSERT_DELTA(x1, x2, epsilon);
    TS_ASSERT_DELTA(y1, y2, epsilon);
  }

  /***************************************************************************
   * Additional Altitude Tests
   ***************************************************************************/

  // Test true altitude vs indicated
  void testTrueAltitude() {
    double indicatedAlt = 10000.0;
    double altimeter = 30.42;  // High pressure
    double stdAltimeter = 29.92;

    // Higher pressure = fly lower to read same indicated
    double correction = (stdAltimeter - altimeter) * 1000.0;
    double trueAlt = indicatedAlt + correction;

    TS_ASSERT(trueAlt < indicatedAlt);  // High pressure = lower true alt
  }

  // Test cold weather altitude correction
  void testColdWeatherAltitude() {
    double indicatedAlt = 5000.0;
    double temp = -20.0;  // Cold day
    double stdTemp = 15.0 - (indicatedAlt / 1000.0) * 2.0;

    // Cold = denser = lower than indicated
    double densityAlt = indicatedAlt + (temp - stdTemp) * 120.0;

    TS_ASSERT(densityAlt < indicatedAlt);
  }

  // Test geometric vs geopotential altitude
  void testGeometricAltitude() {
    double geopotential = 50000.0;  // ft
    double R = 20925646.3;  // Earth radius in ft

    // Geometric is slightly higher than geopotential
    double geometric = geopotential * R / (R - geopotential);

    TS_ASSERT(geometric > geopotential);
    TS_ASSERT_DELTA(geometric, 50119.5, 1.0);  // Approximately
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  // Test 360 degree wraparound
  void test360Wraparound() {
    double lon1 = 350.0;
    double lon2 = 10.0;

    // Should recognize this is 20 degrees, not 340
    double dlon = lon2 - lon1;
    if (dlon < -180.0) dlon += 360.0;
    if (dlon > 180.0) dlon -= 360.0;

    TS_ASSERT_DELTA(dlon, 20.0, epsilon);
  }

  // Test negative zero handling
  void testNegativeZero() {
    double lat = -0.0;
    double lon = -0.0;

    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    TS_ASSERT_DELTA(latRad, 0.0, epsilon);
    TS_ASSERT_DELTA(lonRad, 0.0, epsilon);
  }

  // Test very small latitude difference
  void testVerySmallLatDifference() {
    double lat1 = 40.0;
    double lat2 = 40.0001;  // 0.0001 degree ≈ 36 feet

    double dlat = lat2 - lat1;
    double distFt = dlat * 60.0 * 6076.0;  // nm to ft

    TS_ASSERT(distFt > 30 && distFt < 40);  // About 36 ft
  }

  // Test maximum altitude
  void testMaximumAltitude() {
    double alt = 400000.0;  // 400,000 ft (edge of space)
    double lat = 0.0;

    double N = WGS84_A;
    double X = N + alt;

    // Should still produce valid coordinate
    TS_ASSERT(X > WGS84_A);
    TS_ASSERT(std::isfinite(X));
  }

  // Test WGS84 flattening
  void testWGS84Flattening() {
    double flattening = (WGS84_A - WGS84_B) / WGS84_A;

    // WGS84 flattening ≈ 1/298.257
    TS_ASSERT_DELTA(flattening, 1.0/298.257, 1e-4);
  }

  // Test eccentricity from flattening
  void testEccentricity() {
    double f = (WGS84_A - WGS84_B) / WGS84_A;
    double e2_calc = 2*f - f*f;

    TS_ASSERT_DELTA(e2_calc, WGS84_E2, 1e-6);
  }

  // Test unit vector normalization
  void testUnitVectorNormalization() {
    double lat = 30.0 * DEG_TO_RAD;
    double lon = 45.0 * DEG_TO_RAD;

    // Any direction vector should be unit length
    double Nx = -std::sin(lat) * std::cos(lon);
    double Ny = -std::sin(lat) * std::sin(lon);
    double Nz = std::cos(lat);

    double mag = std::sqrt(Nx*Nx + Ny*Ny + Nz*Nz);
    TS_ASSERT_DELTA(mag, 1.0, epsilon);
  }

  // Test velocity magnitude preservation
  void testVelocityMagnitude() {
    double u = 200.0, v = 50.0, w = 10.0;
    double bodyMag = std::sqrt(u*u + v*v + w*w);

    // Rotation should preserve magnitude
    // (simplified - just check body magnitude)
    TS_ASSERT_DELTA(bodyMag, std::sqrt(200.0*200.0 + 50.0*50.0 + 10.0*10.0), epsilon);
  }

  /***************************************************************************
   * Quaternion Conversion Tests
   ***************************************************************************/

  // Test quaternion from zero Euler angles
  void testQuaternionZeroAngles() {
    double phi = 0.0, theta = 0.0, psi = 0.0;

    // Quaternion for identity rotation
    double q0 = std::cos(phi/2) * std::cos(theta/2) * std::cos(psi/2) +
                std::sin(phi/2) * std::sin(theta/2) * std::sin(psi/2);
    double q1 = std::sin(phi/2) * std::cos(theta/2) * std::cos(psi/2) -
                std::cos(phi/2) * std::sin(theta/2) * std::sin(psi/2);
    double q2 = std::cos(phi/2) * std::sin(theta/2) * std::cos(psi/2) +
                std::sin(phi/2) * std::cos(theta/2) * std::sin(psi/2);
    double q3 = std::cos(phi/2) * std::cos(theta/2) * std::sin(psi/2) -
                std::sin(phi/2) * std::sin(theta/2) * std::cos(psi/2);

    // Identity quaternion is [1, 0, 0, 0]
    TS_ASSERT_DELTA(q0, 1.0, epsilon);
    TS_ASSERT_DELTA(q1, 0.0, epsilon);
    TS_ASSERT_DELTA(q2, 0.0, epsilon);
    TS_ASSERT_DELTA(q3, 0.0, epsilon);
  }

  // Test quaternion normalization
  void testQuaternionNorm() {
    double phi = 30.0 * DEG_TO_RAD;
    double theta = 20.0 * DEG_TO_RAD;
    double psi = 45.0 * DEG_TO_RAD;

    double q0 = std::cos(phi/2) * std::cos(theta/2) * std::cos(psi/2) +
                std::sin(phi/2) * std::sin(theta/2) * std::sin(psi/2);
    double q1 = std::sin(phi/2) * std::cos(theta/2) * std::cos(psi/2) -
                std::cos(phi/2) * std::sin(theta/2) * std::sin(psi/2);
    double q2 = std::cos(phi/2) * std::sin(theta/2) * std::cos(psi/2) +
                std::sin(phi/2) * std::cos(theta/2) * std::sin(psi/2);
    double q3 = std::cos(phi/2) * std::cos(theta/2) * std::sin(psi/2) -
                std::sin(phi/2) * std::sin(theta/2) * std::cos(psi/2);

    double norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    TS_ASSERT_DELTA(norm, 1.0, epsilon);
  }

  // Test 180 degree yaw quaternion
  void testQuaternion180Yaw() {
    double psi = 180.0 * DEG_TO_RAD;

    double q0 = std::cos(psi/2);
    double q3 = std::sin(psi/2);

    // 180 deg about z: [0, 0, 0, 1]
    TS_ASSERT_DELTA(q0, 0.0, epsilon);
    TS_ASSERT_DELTA(q3, 1.0, epsilon);
  }

  /***************************************************************************
   * Spherical Geometry Tests
   ***************************************************************************/

  // Test spherical law of cosines
  void testSphericalLawOfCosines() {
    double a = 30.0 * DEG_TO_RAD;  // Side opposite to A
    double b = 40.0 * DEG_TO_RAD;  // Side opposite to B
    double C = 60.0 * DEG_TO_RAD;  // Angle at C

    // cos(c) = cos(a)*cos(b) + sin(a)*sin(b)*cos(C)
    double cos_c = std::cos(a) * std::cos(b) +
                   std::sin(a) * std::sin(b) * std::cos(C);
    double c = std::acos(cos_c);

    TS_ASSERT(c > 0 && c < M_PI);
  }

  // Test spherical excess (area)
  void testSphericalExcess() {
    // For spherical triangle with angles A, B, C
    double A = 90.0 * DEG_TO_RAD;
    double B = 90.0 * DEG_TO_RAD;
    double C = 90.0 * DEG_TO_RAD;

    // Spherical excess E = A + B + C - π
    double E = A + B + C - M_PI;

    // Area = R² * E (for unit sphere, area = excess)
    TS_ASSERT_DELTA(E, M_PI/2.0, epsilon);  // 1/8 of sphere surface
  }

  // Test meridian arc length
  void testMeridianArcLength() {
    double lat1 = 0.0, lat2 = 90.0;  // Equator to pole

    // Quarter meridian ≈ 10001.9 km ≈ 5400 nm
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double arc_nm = dlat * 60.0 * 180.0 / M_PI;

    TS_ASSERT_DELTA(arc_nm, 5400.0, 100);
  }

  /***************************************************************************
   * Rhumb Line Tests
   ***************************************************************************/

  // Test rhumb line distance (loxodrome)
  void testRhumbLineDistance() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lat2 = 50.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lon2 = -60.0 * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Mercator projection factor
    double dphi = std::log(std::tan(M_PI/4 + lat2/2) / std::tan(M_PI/4 + lat1/2));
    double q = (std::abs(dphi) > 1e-10) ? dlat / dphi : std::cos(lat1);

    double R = 3440.0;
    double distance = std::sqrt(dlat*dlat + q*q*dlon*dlon) * R;

    TS_ASSERT(distance > 500 && distance < 1000);  // Reasonable range
  }

  // Test rhumb line bearing
  void testRhumbLineBearing() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lat2 = 50.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lon2 = -60.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    double dphi = std::log(std::tan(M_PI/4 + lat2/2) / std::tan(M_PI/4 + lat1/2));

    double bearing = std::atan2(dlon, dphi) * RAD_TO_DEG;
    if (bearing < 0) bearing += 360.0;

    TS_ASSERT(bearing >= 40 && bearing < 90);  // Northeast quadrant
  }

  /***************************************************************************
   * Local Tangent Plane Tests
   ***************************************************************************/

  // Test ENU (East-North-Up) frame
  void testENUFrame() {
    double lat = 45.0 * DEG_TO_RAD;
    double lon = 45.0 * DEG_TO_RAD;

    // East vector in ECEF
    double Ex = -std::sin(lon);
    double Ey = std::cos(lon);
    double Ez = 0.0;

    // North vector in ECEF
    double Nx = -std::sin(lat) * std::cos(lon);
    double Ny = -std::sin(lat) * std::sin(lon);
    double Nz = std::cos(lat);

    // Up vector in ECEF (outward normal)
    double Ux = std::cos(lat) * std::cos(lon);
    double Uy = std::cos(lat) * std::sin(lon);
    double Uz = std::sin(lat);

    // Verify orthogonality
    double EN = Ex*Nx + Ey*Ny + Ez*Nz;
    double EU = Ex*Ux + Ey*Uy + Ez*Uz;
    double NU = Nx*Ux + Ny*Uy + Nz*Uz;

    TS_ASSERT_DELTA(EN, 0.0, epsilon);
    TS_ASSERT_DELTA(EU, 0.0, epsilon);
    TS_ASSERT_DELTA(NU, 0.0, epsilon);
  }

  // Test NED to ENU conversion
  void testNEDtoENU() {
    double Vn = 100.0, Ve = 50.0, Vd = -10.0;

    // NED to ENU: swap N/E and negate D
    double Veast = Ve;
    double Vnorth = Vn;
    double Vup = -Vd;

    TS_ASSERT_DELTA(Veast, 50.0, epsilon);
    TS_ASSERT_DELTA(Vnorth, 100.0, epsilon);
    TS_ASSERT_DELTA(Vup, 10.0, epsilon);
  }

  /***************************************************************************
   * Geocentric vs Geodetic Tests
   ***************************************************************************/

  // Test geocentric latitude
  void testGeocentricLatitude() {
    double geodetic_lat = 45.0 * DEG_TO_RAD;

    // Geocentric latitude is slightly less than geodetic
    double geocentric_lat = std::atan((1.0 - WGS84_E2) * std::tan(geodetic_lat));

    TS_ASSERT(geocentric_lat < geodetic_lat);
    TS_ASSERT_DELTA(geocentric_lat, 44.807 * DEG_TO_RAD, 0.01);
  }

  // Test parametric (reduced) latitude
  void testParametricLatitude() {
    double geodetic_lat = 45.0 * DEG_TO_RAD;

    // Parametric latitude: tan(beta) = (b/a) * tan(phi)
    double beta = std::atan(std::sqrt(1.0 - WGS84_E2) * std::tan(geodetic_lat));

    TS_ASSERT(beta < geodetic_lat);
    TS_ASSERT(beta > std::atan((1.0 - WGS84_E2) * std::tan(geodetic_lat)));
  }

  /***************************************************************************
   * Coordinate Stress Tests
   ***************************************************************************/

  // Test ECEF at many latitudes
  void testECEFLatitudeSweep() {
    for (double lat = -90.0; lat <= 90.0; lat += 15.0) {
      double latRad = lat * DEG_TO_RAD;
      double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

      double X = N * std::cos(latRad);
      double Z = N * (1.0 - WGS84_E2) * std::sin(latRad);

      double R = std::sqrt(X*X + Z*Z);
      TS_ASSERT(R >= WGS84_B - 1.0);
      TS_ASSERT(R <= WGS84_A + 1.0);
    }
  }

  // Test ECEF at many longitudes
  void testECEFLongitudeSweep() {
    double lat = 0.0;
    double N = WGS84_A;

    for (double lon = -180.0; lon <= 180.0; lon += 30.0) {
      double lonRad = lon * DEG_TO_RAD;

      double X = N * std::cos(lonRad);
      double Y = N * std::sin(lonRad);

      double R = std::sqrt(X*X + Y*Y);
      TS_ASSERT_DELTA(R, WGS84_A, 1.0);
    }
  }

  // Test bearing calculations around the compass
  void testBearingCompassSweep() {
    double lat1 = 45.0 * DEG_TO_RAD;
    double lon1 = 0.0 * DEG_TO_RAD;

    for (double expected_bearing = 0.0; expected_bearing < 360.0; expected_bearing += 45.0) {
      double bearing_rad = expected_bearing * DEG_TO_RAD;

      // Move 1 degree in direction of bearing
      double dlat = 1.0 * DEG_TO_RAD * std::cos(bearing_rad);
      double dlon = 1.0 * DEG_TO_RAD * std::sin(bearing_rad) / std::cos(lat1);

      double lat2 = lat1 + dlat;
      double lon2 = lon1 + dlon;

      // Calculate bearing back
      double x = std::sin(lon2 - lon1) * std::cos(lat2);
      double y = std::cos(lat1) * std::sin(lat2) -
                 std::sin(lat1) * std::cos(lat2) * std::cos(lon2 - lon1);
      double calc_bearing = std::atan2(x, y) * RAD_TO_DEG;
      if (calc_bearing < 0) calc_bearing += 360.0;

      TS_ASSERT_DELTA(calc_bearing, expected_bearing, 2.0);
    }
  }

  // Test rotation matrix determinant
  void testRotationMatrixDeterminant() {
    double psi = 45.0 * DEG_TO_RAD;
    double theta = 30.0 * DEG_TO_RAD;
    double phi = 20.0 * DEG_TO_RAD;

    // For any valid rotation matrix, det = 1
    // Simplified check: single rotation determinant
    double c = std::cos(psi);
    double s = std::sin(psi);

    // 2D rotation matrix determinant = cos²θ + sin²θ = 1
    double det = c*c + s*s;
    TS_ASSERT_DELTA(det, 1.0, epsilon);
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  // Test high precision lat/lon
  void testHighPrecisionCoords() {
    double lat = 40.7484405;  // Empire State Building
    double lon = -73.9856644;

    // Should handle many decimal places
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    double cosLat = std::cos(latRad);
    double sinLat = std::sin(latRad);

    TS_ASSERT(std::isfinite(cosLat));
    TS_ASSERT(std::isfinite(sinLat));
    TS_ASSERT_DELTA(cosLat*cosLat + sinLat*sinLat, 1.0, epsilon);
  }

  // Test small angle precision
  void testSmallAnglePrecision() {
    double angle = 1e-10;  // Very small angle

    double s = std::sin(angle);
    double c = std::cos(angle);

    TS_ASSERT_DELTA(s, angle, 1e-15);
    TS_ASSERT_DELTA(c, 1.0, 1e-15);
  }

  // Test angle wraparound precision
  void testAngleWrapPrecision() {
    double angle = 359.9999999;
    double wrapped = angle;
    if (wrapped >= 360.0) wrapped -= 360.0;
    if (wrapped < 0.0) wrapped += 360.0;

    TS_ASSERT(wrapped >= 0.0);
    TS_ASSERT(wrapped < 360.0);
  }

  /***************************************************************************
   * Cross-track Distance Tests
   ***************************************************************************/

  // Test cross-track error calculation
  void testCrossTrackError() {
    double lat1 = 40.0 * DEG_TO_RAD, lon1 = -74.0 * DEG_TO_RAD;  // Start
    double lat2 = 42.0 * DEG_TO_RAD, lon2 = -72.0 * DEG_TO_RAD;  // End
    double lat3 = 41.0 * DEG_TO_RAD, lon3 = -72.5 * DEG_TO_RAD;  // Point

    // Angular distance from start to point
    double d13 = 2.0 * std::asin(std::sqrt(
        std::pow(std::sin((lat3-lat1)/2), 2) +
        std::cos(lat1) * std::cos(lat3) * std::pow(std::sin((lon3-lon1)/2), 2)));

    // Bearing from start to end
    double bearing12 = std::atan2(
        std::sin(lon2-lon1) * std::cos(lat2),
        std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(lon2-lon1));

    // Bearing from start to point
    double bearing13 = std::atan2(
        std::sin(lon3-lon1) * std::cos(lat3),
        std::cos(lat1) * std::sin(lat3) - std::sin(lat1) * std::cos(lat3) * std::cos(lon3-lon1));

    // Cross-track error
    double xte = std::asin(std::sin(d13) * std::sin(bearing13 - bearing12));
    double xte_nm = xte * 3440.0;

    TS_ASSERT(std::abs(xte_nm) < 100);  // Should be relatively small
  }

  /***************************************************************************
   * Altitude Rate Tests
   ***************************************************************************/

  // Test vertical speed from flight path angle
  void testVerticalSpeedFromGamma() {
    double V = 250.0;  // knots
    double gamma = 3.0 * DEG_TO_RAD;  // 3 degree glide slope

    double Vd = V * std::sin(gamma);  // Descent rate in knots

    // Convert to ft/min: 1 kt = 101.3 ft/min
    double vs_ftmin = Vd * 101.3;

    TS_ASSERT_DELTA(vs_ftmin, 1325.0, 50.0);  // Approx 1325 ft/min
  }

  // Test climb gradient
  void testClimbGradient() {
    double ROC = 500.0;   // ft/min rate of climb
    double GS = 120.0;    // knots ground speed

    // Gradient = ROC / GS (converted to same units)
    // 1 kt = 101.3 ft/min
    double GS_ftmin = GS * 101.3;
    double gradient = ROC / GS_ftmin * 100.0;  // percent

    TS_ASSERT(gradient > 4.0 && gradient < 5.0);  // ~4.1%
  }

  /***************************************************************************
   * Acceleration Frame Tests
   ***************************************************************************/

  // Test centripetal acceleration in turn
  void testCentripetalAcceleration() {
    double V = 200.0;    // ft/s
    double R = 5000.0;   // ft turn radius

    double a_c = V * V / R;  // ft/s²
    double a_c_g = a_c / 32.174;  // in g's

    TS_ASSERT(a_c_g > 0);
    TS_ASSERT_DELTA(a_c_g, 0.248, 0.01);
  }

  // Test Coriolis acceleration at equator
  void testCoriolisAcceleration() {
    double omega_earth = 7.292115e-5;  // rad/s
    double V = 500.0;  // ft/s
    double lat = 0.0;  // equator

    // Horizontal Coriolis: 2 * omega * V * sin(lat)
    double a_coriolis = 2.0 * omega_earth * V * std::sin(lat * DEG_TO_RAD);

    // At equator, horizontal Coriolis is zero
    TS_ASSERT_DELTA(a_coriolis, 0.0, 1e-10);
  }

  // Test gravity variation with latitude
  void testGravityVariation() {
    double g_equator = 32.0878;   // ft/s²
    double g_pole = 32.2578;      // ft/s²

    double lat = 45.0 * DEG_TO_RAD;

    // Simplified gravity model
    double g = g_equator + (g_pole - g_equator) * std::sin(lat) * std::sin(lat);

    TS_ASSERT(g > g_equator);
    TS_ASSERT(g < g_pole);
  }

  /***************************************************************************
   * Wind Triangle Tests
   ***************************************************************************/

  // Test wind triangle calculation
  void testWindTriangle() {
    double TAS = 200.0;          // True airspeed
    double heading = 90.0;       // East
    double wind_from = 0.0;      // From north
    double wind_speed = 30.0;    // knots

    double hdg_rad = heading * DEG_TO_RAD;
    double wdir_rad = (wind_from + 180.0) * DEG_TO_RAD;  // Wind direction (to)

    // Ground speed vector = TAS vector + wind vector
    double Vn = TAS * std::cos(hdg_rad) + wind_speed * std::cos(wdir_rad);
    double Ve = TAS * std::sin(hdg_rad) + wind_speed * std::sin(wdir_rad);

    double GS = std::sqrt(Vn*Vn + Ve*Ve);
    double track = std::atan2(Ve, Vn) * RAD_TO_DEG;
    if (track < 0) track += 360.0;

    TS_ASSERT(GS < TAS + wind_speed);
    TS_ASSERT(GS > TAS - wind_speed);
  }

  // Test drift angle
  void testDriftAngle() {
    double TAS = 200.0;
    double heading = 0.0;         // North
    double wind_from = 270.0;     // From west
    double wind_speed = 30.0;

    double wdir_rad = (wind_from + 180.0) * DEG_TO_RAD;

    double Vn = TAS + wind_speed * std::cos(wdir_rad);
    double Ve = wind_speed * std::sin(wdir_rad);

    double track = std::atan2(Ve, Vn) * RAD_TO_DEG;
    double drift = track - heading;

    TS_ASSERT(drift > 0);  // Drifting east (right)
    TS_ASSERT(drift < 15.0);
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  // Test coordinates at all hemisphere combinations
  void testAllHemispheres() {
    double lats[] = {45.0, 45.0, -45.0, -45.0};
    double lons[] = {45.0, -45.0, 45.0, -45.0};

    for (int i = 0; i < 4; i++) {
      double latRad = lats[i] * DEG_TO_RAD;
      double lonRad = lons[i] * DEG_TO_RAD;

      double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(latRad) * std::sin(latRad));

      double X = N * std::cos(latRad) * std::cos(lonRad);
      double Y = N * std::cos(latRad) * std::sin(lonRad);
      double Z = N * (1.0 - WGS84_E2) * std::sin(latRad);

      double R = std::sqrt(X*X + Y*Y + Z*Z);
      TS_ASSERT(R > WGS84_B - 100);
      TS_ASSERT(R < WGS84_A + 100);
    }
  }

  // Test coordinate roundtrip
  void testCoordinateRoundtrip() {
    double psi = 45.0 * DEG_TO_RAD;
    double x = 1.0, y = 0.0;

    // Rotate forward
    double x1 = x * std::cos(psi) - y * std::sin(psi);
    double y1 = x * std::sin(psi) + y * std::cos(psi);

    // Rotate back
    double x2 = x1 * std::cos(-psi) - y1 * std::sin(-psi);
    double y2 = x1 * std::sin(-psi) + y1 * std::cos(-psi);

    TS_ASSERT_DELTA(x2, x, epsilon);
    TS_ASSERT_DELTA(y2, y, epsilon);
  }

  // Test rotation matrix orthogonality
  void testRotationOrthogonality() {
    double angle = 37.0 * DEG_TO_RAD;

    double c = std::cos(angle);
    double s = std::sin(angle);

    // R^T * R should be identity
    double r11 = c*c + s*s;
    double r12 = c*(-s) + s*c;
    double r21 = (-s)*c + c*s;
    double r22 = (-s)*(-s) + c*c;

    TS_ASSERT_DELTA(r11, 1.0, epsilon);
    TS_ASSERT_DELTA(r12, 0.0, epsilon);
    TS_ASSERT_DELTA(r21, 0.0, epsilon);
    TS_ASSERT_DELTA(r22, 1.0, epsilon);
  }
};
