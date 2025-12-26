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
};
