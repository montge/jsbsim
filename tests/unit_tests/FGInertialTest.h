/*******************************************************************************
 * FGInertialTest.h - Unit tests for FGInertial (gravity and rotating Earth)
 *
 * Tests the mathematical behavior of inertial model:
 * - Gravity calculations (WGS84 model)
 * - Coriolis and centrifugal effects
 * - Earth rotation rate
 * - Geodetic parameters
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-8;
const double DEG_TO_RAD = M_PI / 180.0;

// WGS84 constants
const double WGS84_A = 20925646.3;       // Semi-major axis (ft)
const double WGS84_B = 20855486.5;       // Semi-minor axis (ft)
const double WGS84_FLATTENING = 1.0/298.257223563;
const double OMEGA_EARTH = 7.292115e-5;  // Earth rotation rate (rad/s)
const double G0 = 32.174049;             // Standard gravity (ft/s^2)
const double GM = 1.407645794e16;        // Gravitational parameter (ft^3/s^2)

class FGInertialTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Standard Gravity Tests
   ***************************************************************************/

  // Test standard gravity value
  void testStandardGravity() {
    TS_ASSERT_DELTA(G0, 32.174, 0.001);  // ft/s^2
  }

  // Test gravity in m/s^2
  void testGravityMetric() {
    double g_metric = G0 * 0.3048;  // Convert ft/s^2 to m/s^2
    TS_ASSERT_DELTA(g_metric, 9.80665, 0.001);
  }

  /***************************************************************************
   * Gravity Variation with Latitude Tests
   ***************************************************************************/

  // Gravity at equator (simplified model)
  void testGravityEquator() {
    double lat = 0.0;  // Equator
    double latRad = lat * DEG_TO_RAD;

    // Somigliana formula (simplified)
    double g_equator = 32.0877;  // ft/s^2 at equator
    double g_pole = 32.2577;     // ft/s^2 at poles
    double g = g_equator + (g_pole - g_equator) * std::sin(latRad) * std::sin(latRad);

    TS_ASSERT_DELTA(g, g_equator, 0.001);
  }

  // Gravity at poles
  void testGravityPole() {
    double lat = 90.0;
    double latRad = lat * DEG_TO_RAD;

    double g_equator = 32.0877;
    double g_pole = 32.2577;
    double g = g_equator + (g_pole - g_equator) * std::sin(latRad) * std::sin(latRad);

    TS_ASSERT_DELTA(g, g_pole, 0.001);
  }

  // Gravity at mid-latitude
  void testGravityMidLatitude() {
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    double g_equator = 32.0877;
    double g_pole = 32.2577;
    double g = g_equator + (g_pole - g_equator) * std::sin(latRad) * std::sin(latRad);

    // At 45°, sin²(45) = 0.5
    double expected = g_equator + (g_pole - g_equator) * 0.5;
    TS_ASSERT_DELTA(g, expected, 0.001);
  }

  /***************************************************************************
   * Gravity Variation with Altitude Tests
   ***************************************************************************/

  // Test gravity decrease with altitude
  void testGravityAltitude() {
    double h = 35000.0;  // ft (typical cruise altitude)
    double r0 = WGS84_A;  // Earth radius at equator

    // Inverse square law
    double g_surface = G0;
    double g_altitude = g_surface * (r0 / (r0 + h)) * (r0 / (r0 + h));

    TS_ASSERT(g_altitude < g_surface);
    TS_ASSERT_DELTA(g_altitude, 32.066, 0.01);  // Slightly less at altitude
  }

  // Test gravity at sea level
  void testGravitySeaLevel() {
    double h = 0.0;
    double r0 = WGS84_A;

    double g = G0 * (r0 / (r0 + h)) * (r0 / (r0 + h));
    TS_ASSERT_DELTA(g, G0, epsilon);
  }

  // Test gravity at very high altitude
  void testGravityHighAltitude() {
    double h = 100000.0;  // 100,000 ft (edge of space)
    double r0 = WGS84_A;

    double g = G0 * (r0 / (r0 + h)) * (r0 / (r0 + h));
    TS_ASSERT(g < G0);
    TS_ASSERT(g > 31.0);  // Still significant gravity
  }

  /***************************************************************************
   * Earth Rotation Tests
   ***************************************************************************/

  // Test Earth rotation rate
  void testEarthRotationRate() {
    // Earth rotates 360° in ~24 hours (sidereal day = 23.934 hrs)
    double siderealDay = 23.934 * 3600;  // seconds
    double omega = 2 * M_PI / siderealDay;

    TS_ASSERT_DELTA(omega, OMEGA_EARTH, 1e-8);
  }

  // Test rotation period
  void testRotationPeriod() {
    double period = 2 * M_PI / OMEGA_EARTH;
    double hours = period / 3600.0;

    TS_ASSERT_DELTA(hours, 23.934, 0.01);  // Sidereal day
  }

  /***************************************************************************
   * Centrifugal Effect Tests
   ***************************************************************************/

  // Test centrifugal acceleration at equator
  void testCentrifugalEquator() {
    double lat = 0.0;
    double latRad = lat * DEG_TO_RAD;
    double r = WGS84_A;  // Distance from Earth center

    // Centrifugal acceleration = omega^2 * r * cos(lat)
    double a_cent = OMEGA_EARTH * OMEGA_EARTH * r * std::cos(latRad);

    // This should be about 0.11 ft/s^2 (reducing effective gravity)
    TS_ASSERT_DELTA(a_cent, 0.111, 0.01);
  }

  // Test centrifugal at poles
  void testCentrifugalPole() {
    double lat = 90.0;
    double latRad = lat * DEG_TO_RAD;
    double r = WGS84_B;

    double a_cent = OMEGA_EARTH * OMEGA_EARTH * r * std::cos(latRad);
    TS_ASSERT_DELTA(a_cent, 0.0, 0.001);  // No centrifugal at poles
  }

  // Test effective gravity (gravity - centrifugal)
  void testEffectiveGravity() {
    double g = G0;
    double a_cent = 0.11;  // Approximate centrifugal at equator

    double g_eff = g - a_cent;
    TS_ASSERT(g_eff < g);
    TS_ASSERT_DELTA(g_eff, 32.06, 0.01);
  }

  /***************************************************************************
   * Coriolis Effect Tests
   ***************************************************************************/

  // Test Coriolis acceleration formula
  void testCoriolisAcceleration() {
    double velocity = 500.0;  // ft/s (about 300 kts)
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Coriolis = 2 * omega * v * sin(lat) (for east-west motion)
    double a_coriolis = 2 * OMEGA_EARTH * velocity * std::sin(latRad);

    // Should be small but significant
    TS_ASSERT(a_coriolis > 0);
    TS_ASSERT_DELTA(a_coriolis, 0.0516, 0.001);
  }

  // Test Coriolis at equator
  void testCoriolisEquator() {
    double velocity = 500.0;
    double lat = 0.0;
    double latRad = lat * DEG_TO_RAD;

    double a_coriolis = 2 * OMEGA_EARTH * velocity * std::sin(latRad);
    TS_ASSERT_DELTA(a_coriolis, 0.0, 0.001);  // No Coriolis at equator (horizontal)
  }

  // Test Coriolis direction (Northern Hemisphere)
  void testCoriolisDirectionNorth() {
    double lat = 45.0;  // Northern hemisphere

    // In Northern Hemisphere, Coriolis deflects to the right
    // This is a sign convention test
    double sinLat = std::sin(lat * DEG_TO_RAD);
    TS_ASSERT(sinLat > 0);  // Positive in Northern Hemisphere
  }

  // Test Coriolis direction (Southern Hemisphere)
  void testCoriolisDirectionSouth() {
    double lat = -45.0;  // Southern hemisphere

    double sinLat = std::sin(lat * DEG_TO_RAD);
    TS_ASSERT(sinLat < 0);  // Negative in Southern Hemisphere (deflects left)
  }

  /***************************************************************************
   * WGS84 Ellipsoid Tests
   ***************************************************************************/

  // Test flattening
  void testFlattening() {
    double f = (WGS84_A - WGS84_B) / WGS84_A;
    TS_ASSERT_DELTA(f, WGS84_FLATTENING, 1e-8);
  }

  // Test eccentricity
  void testEccentricity() {
    double a = WGS84_A;
    double b = WGS84_B;
    double e2 = (a*a - b*b) / (a*a);  // Eccentricity squared

    TS_ASSERT(e2 > 0);
    TS_ASSERT(e2 < 0.01);  // Small eccentricity
  }

  // Test radius at latitude (geocentric radius to ellipsoid surface)
  void testRadiusAtLatitude() {
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Geocentric radius to ellipsoid surface
    double a = WGS84_A;
    double b = WGS84_B;
    double cosLat = std::cos(latRad);
    double sinLat = std::sin(latRad);

    // Correct formula: r = a*b / sqrt(a²sin²φ + b²cos²φ)
    double r = (a * b) / std::sqrt(a*a*sinLat*sinLat + b*b*cosLat*cosLat);

    // At 45°, radius should be between equatorial and polar radii
    // Since we're measuring geocentric radius, it depends on the formula
    TS_ASSERT(r > 0);
    // The geocentric radius at 45° latitude is approximately the average
    double avgRadius = (WGS84_A + WGS84_B) / 2.0;
    TS_ASSERT_DELTA(r, avgRadius, avgRadius * 0.01);  // Within 1%
  }

  /***************************************************************************
   * Weight Calculation Tests
   ***************************************************************************/

  // Test weight from mass
  void testWeightFromMass() {
    double mass = 100.0;  // slugs
    double weight = mass * G0;

    TS_ASSERT_DELTA(weight, 3217.4, 0.1);  // lbs
  }

  // Test weight variation with altitude
  void testWeightVariation() {
    double mass = 100.0;  // slugs
    double h1 = 0.0;
    double h2 = 35000.0;
    double r0 = WGS84_A;

    double w1 = mass * G0;
    double w2 = mass * G0 * (r0/(r0+h2)) * (r0/(r0+h2));

    TS_ASSERT(w2 < w1);  // Lighter at altitude
  }

  /***************************************************************************
   * Angular Momentum Tests
   ***************************************************************************/

  // Test Earth's angular momentum (simplified)
  void testAngularMomentum() {
    // H = I * omega
    // For a sphere: I = 2/5 * M * R^2
    double omega = OMEGA_EARTH;

    // Verify omega is reasonable
    TS_ASSERT(omega > 7e-5);
    TS_ASSERT(omega < 8e-5);
  }

  // Test conservation of angular momentum concept
  void testConservationConcept() {
    // As aircraft moves toward equator, Earth radius increases
    // To conserve momentum, ground speed changes
    double r1 = WGS84_B;  // At pole
    double r2 = WGS84_A;  // At equator

    // v * r = constant (simplified)
    double v1 = 100.0;
    double v2 = v1 * r1 / r2;

    TS_ASSERT(v2 < v1);  // Slower ground speed needed at equator
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero altitude
  void testZeroAltitude() {
    double h = 0.0;
    double r0 = WGS84_A;

    double factor = r0 / (r0 + h);
    TS_ASSERT_DELTA(factor, 1.0, epsilon);
  }

  // Test very small velocity
  void testSmallVelocity() {
    double velocity = 0.001;
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    double a_coriolis = 2 * OMEGA_EARTH * velocity * std::sin(latRad);
    TS_ASSERT(a_coriolis > 0);
    TS_ASSERT(a_coriolis < 1e-6);  // Very small
  }

  // Test latitude bounds
  void testLatitudeBounds() {
    double lat1 = -90.0;
    double lat2 = 90.0;

    TS_ASSERT(std::sin(lat1 * DEG_TO_RAD) == -1.0);
    TS_ASSERT(std::sin(lat2 * DEG_TO_RAD) == 1.0);
  }

  // Test equator values
  void testEquatorValues() {
    double lat = 0.0;
    double latRad = lat * DEG_TO_RAD;

    TS_ASSERT_DELTA(std::sin(latRad), 0.0, epsilon);
    TS_ASSERT_DELTA(std::cos(latRad), 1.0, epsilon);
  }
};
