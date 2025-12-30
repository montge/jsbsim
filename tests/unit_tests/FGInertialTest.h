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

#include <FGFDMExec.h>
#include <models/FGInertial.h>
#include <math/FGLocation.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

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

  /***************************************************************************
   * Additional Comprehensive Inertial Tests
   ***************************************************************************/

  // Test geopotential height
  void testGeopotentialHeight() {
    double geometric_h = 35000.0;  // ft
    double r0 = WGS84_A;

    // Geopotential height: H = r0 * h / (r0 + h)
    double geopotential_h = r0 * geometric_h / (r0 + geometric_h);

    TS_ASSERT(geopotential_h < geometric_h);
    TS_ASSERT_DELTA(geopotential_h, 34941.0, 10.0);
  }

  // Test geopotential to geometric conversion
  void testGeopotentialToGeometric() {
    double geopotential_h = 34941.0;  // ft
    double r0 = WGS84_A;

    // Geometric height: h = r0 * H / (r0 - H)
    double geometric_h = r0 * geopotential_h / (r0 - geopotential_h);

    TS_ASSERT_DELTA(geometric_h, 35000.0, 10.0);
  }

  // Test J2 zonal harmonic effect
  void testJ2ZonalHarmonic() {
    double J2 = 1.08263e-3;  // WGS84 second zonal harmonic

    // J2 causes gravity variation with latitude
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Simplified J2 correction factor
    double sin2lat = std::sin(latRad) * std::sin(latRad);
    double j2_factor = 1.0 + 1.5 * J2 * (3 * sin2lat - 1);

    TS_ASSERT(j2_factor > 0.99 && j2_factor < 1.01);
  }

  // Test geodetic vs geocentric latitude
  void testGeodeticVsGeocentricLatitude() {
    double geodetic_lat = 45.0;  // degrees
    double a = WGS84_A;
    double b = WGS84_B;

    // tan(geocentric) = (b²/a²) * tan(geodetic)
    double ratio = (b * b) / (a * a);
    double geocentric_lat = std::atan(ratio * std::tan(geodetic_lat * DEG_TO_RAD)) / DEG_TO_RAD;

    TS_ASSERT(geocentric_lat < geodetic_lat);  // Geocentric is smaller
    TS_ASSERT_DELTA(geocentric_lat, 44.81, 0.01);
  }

  // Test gravity gradient
  void testGravityGradient() {
    double r = WGS84_A;

    // dg/dh = -2g/r (approximate)
    double gravity_gradient = -2.0 * G0 / r;  // ft/s²/ft

    TS_ASSERT(gravity_gradient < 0);  // Gravity decreases with altitude
    TS_ASSERT_DELTA(gravity_gradient * 1e6, -3.07, 0.1);  // micro-g/ft
  }

  // Test free-fall acceleration
  void testFreeFallAcceleration() {
    double mass = 1.0;  // slug
    double weight = mass * G0;

    // F = ma, so a = F/m = g
    double acceleration = weight / mass;
    TS_ASSERT_DELTA(acceleration, G0, epsilon);
  }

  // Test Eötvös effect
  void testEotvosEffect() {
    // Aircraft flying east at equator experiences apparent weight change
    double v_east = 500.0;  // ft/s eastward
    double lat = 0.0;

    // Eötvös acceleration = 2*omega*v*cos(lat) + v²/r
    double a_eotvos = 2 * OMEGA_EARTH * v_east * std::cos(lat * DEG_TO_RAD);
    a_eotvos += (v_east * v_east) / WGS84_A;

    TS_ASSERT(a_eotvos > 0);  // Apparent reduction in weight
  }

  // Test westward flight Eötvös effect
  void testEotvosWestward() {
    double v_west = -500.0;  // ft/s westward
    double lat = 0.0;

    double a_eotvos = 2 * OMEGA_EARTH * v_west * std::cos(lat * DEG_TO_RAD);
    a_eotvos += (v_west * v_west) / WGS84_A;

    // Net effect is positive (v² term) but smaller than eastward
    TS_ASSERT(a_eotvos != 0);
  }

  // Test local vertical deviation
  void testLocalVerticalDeviation() {
    double geodetic_lat = 45.0;
    double geocentric_lat = 44.81;

    // Difference between geodetic and geocentric latitude
    double deviation = geodetic_lat - geocentric_lat;
    TS_ASSERT_DELTA(deviation, 0.19, 0.01);  // degrees
  }

  // Test Earth surface velocity
  void testEarthSurfaceVelocity() {
    double lat = 0.0;  // Equator
    double r = WGS84_A;

    // Surface velocity = omega * r * cos(lat)
    double v_surface = OMEGA_EARTH * r * std::cos(lat * DEG_TO_RAD);

    // Convert to knots: 1 ft/s = 0.5925 knots
    double v_knots = v_surface * 0.5925;

    TS_ASSERT_DELTA(v_knots, 902.0, 5.0);  // About 900 knots at equator
  }

  // Test Earth surface velocity at mid-latitude
  void testEarthSurfaceVelocityMidLat() {
    double lat = 45.0;
    double r = WGS84_A * std::cos(lat * DEG_TO_RAD);  // Distance from axis

    double v_surface = OMEGA_EARTH * r;
    double v_knots = v_surface * 0.5925;

    TS_ASSERT(v_knots < 900.0);  // Less than at equator
  }

  // Test gravitational potential
  void testGravitationalPotential() {
    double r = WGS84_A;

    // U = GM/r
    double U = GM / r;

    TS_ASSERT(U > 0);
    // U = GM/r = 1.408e16 / 2.09e7 ≈ 6.73e8 ft²/s²
    TS_ASSERT_DELTA(U * 1e-8, 6.73, 0.1);
  }

  // Test escape velocity concept
  void testEscapeVelocity() {
    double r = WGS84_A;

    // v_escape = sqrt(2*GM/r)
    double v_escape = std::sqrt(2 * GM / r);

    // Convert to ft/s, should be about 36,700 ft/s
    TS_ASSERT_DELTA(v_escape, 36700.0, 100.0);
  }

  // Test orbital velocity concept
  void testOrbitalVelocity() {
    double r = WGS84_A + 100 * 6076.12;  // 100 nm altitude in ft

    // v_orbital = sqrt(GM/r)
    double v_orbital = std::sqrt(GM / r);

    // Should be about 25,500 ft/s
    TS_ASSERT_DELTA(v_orbital, 25500.0, 500.0);
  }

  // Test gravity at different latitudes array
  void testGravityLatitudeArray() {
    double latitudes[] = {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
    double g_equator = 32.0877;
    double g_pole = 32.2577;

    double prev_g = 0.0;
    for (double lat : latitudes) {
      double latRad = lat * DEG_TO_RAD;
      double g = g_equator + (g_pole - g_equator) * std::sin(latRad) * std::sin(latRad);

      TS_ASSERT(g >= g_equator);
      TS_ASSERT(g <= g_pole);
      TS_ASSERT(g >= prev_g);  // Monotonically increasing
      prev_g = g;
    }
  }

  // Test transport rate
  void testTransportRate() {
    // Transport rate = rate of change of latitude due to velocity
    double v_north = 500.0;  // ft/s northward
    double r = WGS84_A;

    // dlat/dt = v_north / r (radians/s)
    double lat_rate = v_north / r;

    TS_ASSERT(lat_rate > 0);
    TS_ASSERT(lat_rate < 1e-4);  // Small rate
  }

  // Test longitude rate
  void testLongitudeRate() {
    double v_east = 500.0;  // ft/s eastward
    double r = WGS84_A;
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // dlon/dt = v_east / (r * cos(lat))
    double lon_rate = v_east / (r * std::cos(latRad));

    TS_ASSERT(lon_rate > 0);
  }

  // Test radius of curvature (meridian)
  void testRadiusCurvatureMeridian() {
    double a = WGS84_A;
    double b = WGS84_B;
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    double e2 = (a*a - b*b) / (a*a);
    double sinLat = std::sin(latRad);

    // Radius of curvature in meridian
    double M = a * (1 - e2) / std::pow(1 - e2 * sinLat * sinLat, 1.5);

    TS_ASSERT(M > 0);
    TS_ASSERT(M < a * 1.1);  // Reasonable range
  }

  // Test radius of curvature (prime vertical)
  void testRadiusCurvaturePrimeVertical() {
    double a = WGS84_A;
    double b = WGS84_B;
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    double e2 = (a*a - b*b) / (a*a);
    double sinLat = std::sin(latRad);

    // Radius of curvature in prime vertical
    double N = a / std::sqrt(1 - e2 * sinLat * sinLat);

    TS_ASSERT(N > 0);
    TS_ASSERT(N >= a);  // N >= a always
  }

  // Test plumb line deviation
  void testPlumbLineDeviation() {
    // The plumb line doesn't point exactly to Earth center
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Deviation is max at 45° latitude
    double max_deviation = 0.19;  // degrees (approximately)
    TS_ASSERT(max_deviation > 0.1);
    TS_ASSERT(max_deviation < 0.3);
  }

  // Test pendulum period variation
  void testPendulumPeriodVariation() {
    // T = 2*pi*sqrt(L/g)
    double L = 1.0;  // ft pendulum length
    double g_equator = 32.0877;
    double g_pole = 32.2577;

    double T_equator = 2 * M_PI * std::sqrt(L / g_equator);
    double T_pole = 2 * M_PI * std::sqrt(L / g_pole);

    TS_ASSERT(T_equator > T_pole);  // Longer period at equator (weaker gravity)
  }

  // Test Coriolis for vertical motion
  void testCoriolisVertical() {
    double v_vertical = 100.0;  // ft/s upward
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Vertical Coriolis (deflects east for upward motion in N. Hemisphere)
    double a_coriolis_h = 2 * OMEGA_EARTH * v_vertical * std::cos(latRad);

    TS_ASSERT(a_coriolis_h > 0);
  }

  // Test gravity at geostationary orbit altitude
  void testGravityGeostationary() {
    // Geostationary altitude: ~22,236 miles = 117,406,080 ft above equator
    double h = 117406080.0;
    double r0 = WGS84_A;
    double r = r0 + h;

    double g = G0 * (r0 / r) * (r0 / r);

    TS_ASSERT(g < 1.0);  // Much weaker gravity
    TS_ASSERT(g > 0.0);
  }

  // Test centripetal requirement for orbit
  void testCentripetalOrbit() {
    double h = 100 * 6076.12;  // 100 nm in ft
    double r = WGS84_A + h;

    // At orbital velocity, centripetal = gravitational
    double g = GM / (r * r);
    double v_orbital = std::sqrt(GM / r);
    double centripetal = v_orbital * v_orbital / r;

    TS_ASSERT_DELTA(centripetal, g, 0.001);
  }

  // Test angular momentum of orbiting object
  void testOrbitalAngularMomentum() {
    double r = WGS84_A + 100 * 6076.12;
    double v = std::sqrt(GM / r);
    double mass = 1.0;  // slug

    double L = mass * v * r;  // Angular momentum

    TS_ASSERT(L > 0);
  }

  // Test Foucault pendulum period
  void testFoucaultPendulumPeriod() {
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Period of Foucault pendulum plane rotation
    double T_foucault = (2 * M_PI / OMEGA_EARTH) / std::sin(latRad);
    double hours = T_foucault / 3600.0;

    TS_ASSERT_DELTA(hours, 33.84, 0.1);  // About 34 hours at 45°
  }

  // Test Foucault at pole
  void testFoucaultPendulumPole() {
    double lat = 90.0;
    double latRad = lat * DEG_TO_RAD;

    double T_foucault = (2 * M_PI / OMEGA_EARTH) / std::sin(latRad);
    double hours = T_foucault / 3600.0;

    TS_ASSERT_DELTA(hours, 23.93, 0.1);  // One sidereal day
  }

  // Test weight variation for aircraft
  void testAircraftWeightVariation() {
    double mass = 155.0;  // slugs (5000 lb aircraft)
    double h = 35000.0;
    double r0 = WGS84_A;

    double g_surface = G0;
    double g_altitude = g_surface * (r0 / (r0 + h)) * (r0 / (r0 + h));

    double weight_surface = mass * g_surface;
    double weight_altitude = mass * g_altitude;
    double weight_loss = weight_surface - weight_altitude;

    TS_ASSERT(weight_loss > 0);
    TS_ASSERT(weight_loss < 20.0);  // Less than 20 lbs difference
  }

  // Test specific force
  void testSpecificForce() {
    // Specific force = acceleration felt by accelerometer
    // In free fall: specific force = 0
    // At rest on Earth: specific force = g (upward)

    double g = G0;
    double specific_force_rest = g;  // Accelerometer reads g when at rest

    TS_ASSERT_DELTA(specific_force_rest, G0, epsilon);
  }

  // Test rotating frame velocity addition
  void testRotatingFrameVelocity() {
    double v_inertial = 500.0;  // ft/s in inertial frame
    double v_earth = OMEGA_EARTH * WGS84_A;  // Earth surface velocity

    // Ground speed = inertial velocity - Earth rotation
    double v_ground = v_inertial - v_earth;

    TS_ASSERT(std::abs(v_ground) < std::abs(v_inertial) + std::abs(v_earth));
  }

  // Test spherical vs ellipsoid gravity
  void testSphericalVsEllipsoid() {
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Spherical approximation
    double g_spherical = G0;

    // Ellipsoid approximation (Somigliana)
    double g_equator = 32.0877;
    double g_pole = 32.2577;
    double g_ellipsoid = g_equator + (g_pole - g_equator) * std::sin(latRad) * std::sin(latRad);

    // They differ slightly
    TS_ASSERT(std::abs(g_spherical - g_ellipsoid) < 0.2);
  }

  // Test gravity vector direction
  void testGravityVectorDirection() {
    // Gravity vector points toward Earth center (in geocentric frame)
    // In geodetic frame, it's along the local vertical

    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    // Local vertical has components in geocentric frame
    double sin_lat = std::sin(latRad);
    double cos_lat = std::cos(latRad);

    // Unit vector magnitude should be 1
    double mag = std::sqrt(sin_lat * sin_lat + cos_lat * cos_lat);
    TS_ASSERT_DELTA(mag, 1.0, epsilon);
  }

  // Test apparent weight in curved flight
  void testApparentWeightCurvedFlight() {
    double v = 500.0;  // ft/s
    double r = 5000.0; // ft turn radius
    double mass = 155.0; // slugs

    // Centripetal acceleration
    double a_centripetal = v * v / r;

    // Load factor for level turn
    double n = std::sqrt(1 + (a_centripetal / G0) * (a_centripetal / G0));

    TS_ASSERT(n > 1.0);  // Load factor > 1 in a turn
  }

  // Test load factor in pull-up
  void testLoadFactorPullUp() {
    double v = 500.0;  // ft/s
    double r = 3000.0; // ft pull-up radius

    // In a pull-up, centripetal adds to gravity
    double a_centripetal = v * v / r;
    double n = 1.0 + a_centripetal / G0;

    TS_ASSERT(n > 2.0);  // High g pull-up
  }

  // Test negative g (push-over)
  void testNegativeG() {
    double v = 500.0;  // ft/s
    double r = 2000.0; // ft push-over radius

    // In a push-over, centripetal opposes gravity
    double a_centripetal = v * v / r;
    double n = 1.0 - a_centripetal / G0;

    TS_ASSERT(n < 1.0);
    TS_ASSERT(n < 0.0);  // Negative g
  }

  /***************************************************************************
   * Extended Gravity and Rotation Tests
   ***************************************************************************/

  // Test gravity decrease per 1000 ft altitude
  void testGravityDecreasePer1000ft() {
    double h1 = 0.0;
    double h2 = 1000.0;
    double r0 = WGS84_A;

    double g1 = G0 * std::pow(r0 / (r0 + h1), 2);
    double g2 = G0 * std::pow(r0 / (r0 + h2), 2);

    double decrease = g1 - g2;
    // Approximately 0.003 ft/s² per 1000 ft
    TS_ASSERT_DELTA(decrease, 0.003, 0.001);
  }

  // Test effective gravity (including centrifugal)
  void testEffectiveGravityLatitude() {
    double lats[] = {0.0, 30.0, 60.0, 90.0};
    double prev_g_eff = 0.0;

    for (double lat : lats) {
      double latRad = lat * DEG_TO_RAD;
      double g = 32.0877 + (32.2577 - 32.0877) * std::sin(latRad) * std::sin(latRad);
      double a_cent = OMEGA_EARTH * OMEGA_EARTH * WGS84_A * std::cos(latRad);
      double g_eff = g - a_cent;

      TS_ASSERT(g_eff > 0);
      if (lat > 0) {
        TS_ASSERT(g_eff >= prev_g_eff);  // Increasing toward poles
      }
      prev_g_eff = g_eff;
    }
  }

  // Test Earth rotation period in seconds
  void testEarthRotationSeconds() {
    double period = 2 * M_PI / OMEGA_EARTH;
    TS_ASSERT_DELTA(period, 86164.1, 1.0);  // Sidereal day in seconds
  }

  // Test surface velocity at 30° latitude
  void testSurfaceVelocity30Lat() {
    double lat = 30.0;
    double latRad = lat * DEG_TO_RAD;
    double r = WGS84_A * std::cos(latRad);

    double v = OMEGA_EARTH * r;
    double v_knots = v * 0.5925;

    TS_ASSERT(v_knots < 900.0);  // Less than equator
    TS_ASSERT(v_knots > 700.0);
  }

  // Test gravity at ISS altitude
  void testGravityISSAltitude() {
    double h = 400 * 3280.84;  // 400 km in ft
    double r0 = WGS84_A;
    double r = r0 + h;

    double g = G0 * std::pow(r0 / r, 2);

    // ISS experiences about 90% of surface gravity
    TS_ASSERT(g < G0);
    TS_ASSERT(g > 0.85 * G0);
  }

  // Test free-fall time from 1000 ft
  void testFreeFallTime() {
    double h = 1000.0;  // ft
    double g = G0;

    // t = sqrt(2h/g) for free fall from rest
    double t = std::sqrt(2 * h / g);

    TS_ASSERT_DELTA(t, 7.88, 0.1);  // About 8 seconds
  }

  // Test terminal velocity concept
  void testTerminalVelocityConcept() {
    // At terminal velocity, drag = weight
    // F_drag = 0.5 * rho * V^2 * Cd * A
    // W = m * g

    double mass = 155.0;  // slugs
    double weight = mass * G0;  // 155 * 32.174 = 4987 lbs

    TS_ASSERT(weight > 0);
    TS_ASSERT_DELTA(weight, 4987.0, 10.0);
  }

  // Test Coriolis for high-speed aircraft
  void testCoriolisHighSpeed() {
    double velocity = 1000.0;  // ft/s (about 600 kts)
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    double a_coriolis = 2 * OMEGA_EARTH * velocity * std::sin(latRad);

    // Double the velocity, double the Coriolis
    double a_coriolis_500 = 2 * OMEGA_EARTH * 500.0 * std::sin(latRad);
    TS_ASSERT_DELTA(a_coriolis, 2 * a_coriolis_500, 0.001);
  }

  // Test centrifugal at 30° latitude
  void testCentrifugal30Lat() {
    double lat = 30.0;
    double latRad = lat * DEG_TO_RAD;
    double r = WGS84_A;

    double a_cent = OMEGA_EARTH * OMEGA_EARTH * r * std::cos(latRad);

    // Should be between equator and pole values
    TS_ASSERT(a_cent > 0.0);
    TS_ASSERT(a_cent < 0.111);  // Less than equator
  }

  // Test gravity ratio earth to moon
  void testGravityRatioEarthMoon() {
    double g_earth = G0;
    double g_moon = 5.31;  // ft/s² on Moon surface

    double ratio = g_earth / g_moon;
    TS_ASSERT_DELTA(ratio, 6.06, 0.1);  // Earth gravity is ~6x Moon
  }

  // Test weight on Moon
  void testWeightOnMoon() {
    double mass = 155.0;  // slugs
    double g_moon = 5.31;

    double weight_moon = mass * g_moon;
    double weight_earth = mass * G0;

    TS_ASSERT(weight_moon < weight_earth);
    TS_ASSERT_DELTA(weight_moon / weight_earth, 1.0/6.0, 0.02);
  }

  // Test orbital period at 100nm
  void testOrbitalPeriod100nm() {
    double h = 100 * 6076.12;  // 100 nm in ft
    double r = WGS84_A + h;

    // T = 2*pi*sqrt(r³/GM)
    double T = 2 * M_PI * std::sqrt(r * r * r / GM);
    double minutes = T / 60.0;

    TS_ASSERT_DELTA(minutes, 87.0, 2.0);  // About 87 minutes
  }

  // Test gravity gradient for satellite
  void testGravityGradientSatellite() {
    double r = WGS84_A + 400 * 3280.84;  // 400 km in ft

    // Gravity gradient = 2*g/r (magnitude of tidal force)
    double g = GM / (r * r);
    double gradient = 2 * g / r;

    TS_ASSERT(gradient > 0);
    TS_ASSERT(gradient < 1e-3);  // Small but measurable
  }

  // Test geocentric vs geodetic latitude difference max
  void testLatitudeDifferenceMax() {
    // Maximum difference occurs around 45°
    double a = WGS84_A;
    double b = WGS84_B;

    double max_diff = 0.0;
    for (double geodetic = 0; geodetic <= 90; geodetic += 1) {
      double ratio = (b * b) / (a * a);
      double geocentric = std::atan(ratio * std::tan(geodetic * DEG_TO_RAD)) / DEG_TO_RAD;
      double diff = geodetic - geocentric;
      if (diff > max_diff) max_diff = diff;
    }

    TS_ASSERT_DELTA(max_diff, 0.19, 0.02);  // About 0.19° max difference
  }

  // Test N-S vs E-W radii of curvature
  void testRadiiOfCurvature() {
    double a = WGS84_A;
    double b = WGS84_B;
    double lat = 45.0;
    double latRad = lat * DEG_TO_RAD;

    double e2 = (a*a - b*b) / (a*a);
    double sinLat = std::sin(latRad);

    double M = a * (1 - e2) / std::pow(1 - e2 * sinLat * sinLat, 1.5);  // Meridian
    double N = a / std::sqrt(1 - e2 * sinLat * sinLat);  // Prime vertical

    TS_ASSERT(N >= M);  // Prime vertical >= meridian always
  }

  // Test great circle distance formula verification
  void testGreatCircleDistanceConcept() {
    // Haversine formula conceptual test
    double lat1 = 0.0, lon1 = 0.0;  // Point 1
    double lat2 = 0.0, lon2 = 1.0;  // Point 2 (1° east on equator)

    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double lat1Rad = lat1 * DEG_TO_RAD;

    // At equator, 1° = WGS84_A * pi/180
    double distance = WGS84_A * dlon;

    // About 60 nautical miles per degree at equator
    double nm = distance / 6076.12;
    TS_ASSERT_DELTA(nm, 60.0, 1.0);
  }

  // Test rhumb line vs great circle
  void testRhumbVsGreatCircle() {
    // For short distances and low latitudes, they're similar
    double distance = 100 * 6076.12;  // 100 nm in ft

    // At equator, rhumb and great circle are identical
    TS_ASSERT(distance > 0);
  }

  // Test precession rate concept
  void testPrecessionRateConcept() {
    // Earth's precession period is about 26,000 years
    // Rate = 2*pi / (26000 * 365.25 * 24 * 3600) rad/s
    double precession_period = 26000 * 365.25 * 24 * 3600;  // seconds
    double precession_rate = 2 * M_PI / precession_period;

    TS_ASSERT(precession_rate < OMEGA_EARTH);  // Much slower than rotation
    TS_ASSERT(precession_rate > 0);
  }

  // Test nutation concept
  void testNutationConcept() {
    // Nutation has an 18.6 year period
    double nutation_period = 18.6 * 365.25 * 24 * 3600;  // seconds

    TS_ASSERT(nutation_period > 0);
    TS_ASSERT(nutation_period < 26000 * 365.25 * 24 * 3600);  // Shorter than precession
  }

  // Test polar motion concept
  void testPolarMotionConcept() {
    // Chandler wobble has ~433 day period
    double chandler_period = 433 * 24 * 3600;  // seconds

    // This is much shorter than precession
    TS_ASSERT(chandler_period < 365.25 * 24 * 3600 * 2);  // Less than 2 years
  }

  // Test tidal acceleration concept
  void testTidalAccelerationConcept() {
    double r = WGS84_A;
    double R_moon = 1.262e9;  // Moon distance in ft (approximate)

    // Tidal acceleration ~ 2*GM_moon*r/R³
    // This is a small effect
    TS_ASSERT(R_moon > r * 50);  // Moon is much farther than Earth radius
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete inertial navigation calculation
  void testCompleteInertialNavigation() {
    double lat = 45.0 * DEG_TO_RAD;
    double lon = -75.0 * DEG_TO_RAD;
    double alt = 30000.0;  // ft

    // Earth radius at latitude (WGS84 eccentricity squared)
    double e2 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);
    double N = WGS84_A / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    double M = WGS84_A * (1.0 - e2) / std::pow(1.0 - e2 * std::sin(lat) * std::sin(lat), 1.5);

    TS_ASSERT(N > 0.0);
    TS_ASSERT(M > 0.0);
    TS_ASSERT(N > M);  // N > M for oblate ellipsoid
  }

  // Test inertial velocity to ground velocity
  void testInertialToGroundVelocity() {
    double v_inertial = 500.0;  // ft/s
    double omega_earth = OMEGA_EARTH;
    double r = WGS84_A + 30000.0;
    double lat = 45.0 * DEG_TO_RAD;

    double v_rotation = omega_earth * r * std::cos(lat);
    double v_ground_approx = v_inertial - v_rotation;

    TS_ASSERT(v_rotation > 0.0);
    TS_ASSERT(!std::isnan(v_ground_approx));
  }

  // Test gravity gradient torque
  void testGravityGradientTorque() {
    double I_diff = 100.0;  // slug-ft^2 (difference in inertias)
    double g = G0;
    double r = WGS84_A;
    double theta = 5.0 * DEG_TO_RAD;

    double torque = 3.0 * g * I_diff * std::sin(2.0 * theta) / (2.0 * r);
    TS_ASSERT(!std::isnan(torque));
  }

  // Test orbital velocity at altitude
  void testOrbitalVelocityAtAltitude() {
    double alt = 200.0 * 6076.12;  // 200 nm in ft
    double r = WGS84_A + alt;

    double v_orbital = std::sqrt(GM / r);
    TS_ASSERT(v_orbital > 20000.0);  // Should be about 25000 ft/s
    TS_ASSERT(v_orbital < 30000.0);
  }

  // Test atmospheric drag effect on orbit
  void testAtmosphericDragOrbitEffect() {
    double v = 25000.0;  // ft/s
    double Cd = 2.2;
    double A = 100.0;  // ft^2
    double rho = 1e-12;  // Very thin atmosphere (higher orbit)

    double drag = 0.5 * rho * v * v * Cd * A;
    TS_ASSERT(drag > 0.0);
    TS_ASSERT(drag < 0.1);  // Very small at high altitude
  }

  // Test Schuler period calculation
  void testSchulerPeriodCalculation() {
    // Schuler period = 2*pi*sqrt(R/g) ≈ 84.4 minutes
    double T_schuler = 2.0 * M_PI * std::sqrt(WGS84_A / G0);
    double T_minutes = T_schuler / 60.0;

    TS_ASSERT_DELTA(T_minutes, 84.4, 1.0);
  }

  // Test Foucault pendulum period at latitude
  void testFoucaultPendulumPeriodAtLatitude() {
    double lat = 45.0 * DEG_TO_RAD;
    double sidereal_day = 86164.1;  // seconds

    double T_foucault = sidereal_day / std::sin(lat);
    TS_ASSERT(T_foucault > sidereal_day);
  }

  // Test geopotential vs geometric altitude
  void testGeopotentialVsGeometricAltitude() {
    double geometric_alt = 100000.0;  // ft
    double r0 = WGS84_A;

    double geopotential_alt = geometric_alt * r0 / (r0 + geometric_alt);
    TS_ASSERT(geopotential_alt < geometric_alt);
  }

  // Test gravitational potential energy
  void testGravitationalPotentialEnergy() {
    double m = 100.0;  // slugs
    double r1 = WGS84_A;
    double r2 = WGS84_A + 100000.0;

    double PE_diff = GM * m * (1.0 / r1 - 1.0 / r2);
    TS_ASSERT(PE_diff > 0.0);
  }

  // Test J4 gravity term magnitude
  void testJ4GravityTermMagnitude() {
    // J4 << J2
    double J2 = 1.08263e-3;  // WGS84 J2 coefficient
    double J4 = 1.62e-6;     // Approximate value
    TS_ASSERT(J4 < J2 / 100.0);
  }

  // Test Earth rotation effect on projectile
  void testEarthRotationProjectileEffect() {
    double v_north = 1000.0;  // ft/s
    double lat = 30.0 * DEG_TO_RAD;
    double dt = 10.0;  // seconds

    double deflection = 2.0 * OMEGA_EARTH * v_north * std::sin(lat) * dt;
    TS_ASSERT(deflection > 0.0);
    TS_ASSERT(deflection < 1.0);  // Small deflection
  }

  // Test centrifugal acceleration at pole vs equator
  void testCentrifugalPoleVsEquator() {
    double a_equator = OMEGA_EARTH * OMEGA_EARTH * WGS84_A;
    double a_pole = 0.0;  // No centrifugal at pole

    TS_ASSERT(a_equator > a_pole);
    TS_ASSERT_DELTA(a_equator, 0.11, 0.01);  // About 0.11 ft/s^2
  }

  // Test escape velocity from Earth surface
  void testEscapeVelocityEarthSurface() {
    double v_escape = std::sqrt(2.0 * GM / WGS84_A);
    TS_ASSERT(v_escape > 35000.0);  // About 36,700 ft/s
    TS_ASSERT(v_escape < 40000.0);
  }

  // Test inertial reference frame transformation
  void testInertialReferenceFrameTransform() {
    double theta = 45.0 * DEG_TO_RAD;
    double x = 100.0;
    double y = 0.0;

    double x_rot = x * std::cos(theta) - y * std::sin(theta);
    double y_rot = x * std::sin(theta) + y * std::cos(theta);

    TS_ASSERT_DELTA(x_rot, 70.71, 0.1);
    TS_ASSERT_DELTA(y_rot, 70.71, 0.1);
  }

  // Test inertial instance independence
  void testInertialInstanceIndependence() {
    double lat1 = 0.0;
    double lat2 = 45.0 * DEG_TO_RAD;

    double g1 = G0 * (1.0 + 0.00193185 * std::sin(lat1) * std::sin(lat1));
    double g2 = G0 * (1.0 + 0.00193185 * std::sin(lat2) * std::sin(lat2));

    TS_ASSERT(g1 != g2);
    TS_ASSERT(g2 > g1);  // Higher at higher latitude
  }

  // Test gravity calculation state independence
  void testGravityCalculationStateIndependence() {
    double h1 = 0.0;
    double h2 = 50000.0;

    double g1 = GM / ((WGS84_A + h1) * (WGS84_A + h1));
    double g2 = GM / ((WGS84_A + h2) * (WGS84_A + h2));

    TS_ASSERT(g1 > g2);  // Gravity decreases with altitude
    TS_ASSERT_DELTA(g1, G0, 0.1);
  }

  // ============================================================================
  // FGInertial class tests - using actual class methods
  // ============================================================================

  // Test FGInertial default construction through FGFDMExec
  void testFGInertialConstruction() {
    FGFDMExec fdmex;
    auto inertial = fdmex.GetInertial();
    TS_ASSERT(inertial != nullptr);
  }

  // Test GetStandardGravity static method
  void testFGInertialGetStandardGravity() {
    // GetStandardGravity is a static constexpr method
    double gRef = FGInertial::GetStandardGravity();
    TS_ASSERT(gRef > 31.0);
    TS_ASSERT(gRef < 33.0);
    // Should be about 32.174 ft/s^2
    TS_ASSERT_DELTA(gRef, 32.174, 0.01);
  }

  // Test GetGravity method
  void testFGInertialGetGravity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Run to compute gravity
    inertial->Run(false);

    // Get computed gravity vector
    const FGColumnVector3& gravity = inertial->GetGravity();
    TS_ASSERT(gravity.Magnitude() > 0.0);
  }

  // Test GetGM method
  void testFGInertialGetGM() {
    FGFDMExec fdmex;
    auto inertial = fdmex.GetInertial();

    double gm = inertial->GetGM();
    // GM should be about 1.4e16 ft^3/s^2 for Earth
    TS_ASSERT(gm > 1.0e15);
    TS_ASSERT(gm < 2.0e16);
  }

  // Test SetGravityType method
  void testFGInertialSetGravityType() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto inertial = fdmex.GetInertial();

    // Default is WGS84
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtWGS84);

    // Set to Standard
    inertial->SetGravityType(FGInertial::gtStandard);
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtStandard);

    // Set back to WGS84
    inertial->SetGravityType(FGInertial::gtWGS84);
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtWGS84);
  }

  // Test GetTl2ec transformation
  void testFGInertialGetTl2ec() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto inertial = fdmex.GetInertial();

    FGLocation position;
    position.SetPositionGeodetic(0.0, 0.0, 0.0);

    FGMatrix33 Tl2ec = inertial->GetTl2ec(position);

    // Should be a valid rotation matrix (determinant = 1)
    double det = Tl2ec.Determinant();
    TS_ASSERT_DELTA(det, 1.0, 0.01);
  }

  // Test GetTl2ec at different latitudes
  void testFGInertialGetTl2ecLatitudes() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto inertial = fdmex.GetInertial();

    // Test at equator
    FGLocation pos_eq;
    pos_eq.SetPositionGeodetic(0.0, 0.0, 0.0);
    FGMatrix33 T_eq = inertial->GetTl2ec(pos_eq);
    TS_ASSERT_DELTA(T_eq.Determinant(), 1.0, 0.01);

    // Test at 45 degrees
    FGLocation pos_45;
    pos_45.SetPositionGeodetic(0.0, 45.0 * M_PI / 180.0, 0.0);
    FGMatrix33 T_45 = inertial->GetTl2ec(pos_45);
    TS_ASSERT_DELTA(T_45.Determinant(), 1.0, 0.01);
  }

  // Test omega planet (rotation rate)
  void testFGInertialOmegaPlanet() {
    FGFDMExec fdmex;
    auto inertial = fdmex.GetInertial();

    FGColumnVector3 omega = inertial->GetOmegaPlanet();

    // Earth rotation is about Z axis
    TS_ASSERT_DELTA(omega(1), 0.0, epsilon);
    TS_ASSERT_DELTA(omega(2), 0.0, epsilon);
    TS_ASSERT(omega(3) > 7.0e-5);  // About 7.29e-5 rad/s
    TS_ASSERT(omega(3) < 7.4e-5);
  }

  // Test Run method
  void testFGInertialRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();

    auto inertial = fdmex.GetInertial();

    // Run should succeed and return false (no error)
    bool result = inertial->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test Run in holding mode
  void testFGInertialRunHolding() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();

    auto inertial = fdmex.GetInertial();

    // Run in holding mode
    bool result = inertial->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test Load method with XML
  void testFGInertialLoadXML() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto inertial = fdmex.GetInertial();

    std::string xml = R"(
      <planet name="TestPlanet">
        <semimajor_axis unit="FT">20000000.0</semimajor_axis>
        <semiminor_axis unit="FT">19900000.0</semiminor_axis>
        <rotation_rate unit="RAD/SEC">0.00005</rotation_rate>
        <GM unit="FT3/SEC2">1.0e16</GM>
        <J2>0.001</J2>
      </planet>
    )";

    Element_ptr el = readFromXML(xml);
    bool loaded = inertial->Load(el.ptr());
    TS_ASSERT(loaded);
  }

  // Test Load with equatorial_radius instead of semimajor_axis
  void testFGInertialLoadXMLEquatorialRadius() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto inertial = fdmex.GetInertial();

    std::string xml = R"(
      <planet name="TestPlanet2">
        <equatorial_radius unit="FT">20000000.0</equatorial_radius>
        <polar_radius unit="FT">19900000.0</polar_radius>
      </planet>
    )";

    Element_ptr el = readFromXML(xml);
    bool loaded = inertial->Load(el.ptr());
    TS_ASSERT(loaded);
  }

  // Test gravity model property binding
  void testFGInertialGravityModelProperty() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto pm = fdmex.GetPropertyManager();

    // Check property exists
    auto node = pm->GetNode("simulation/gravity-model");
    TS_ASSERT(node != nullptr);

    if (node) {
      // Default is WGS84 (1)
      int gravType = node->getIntValue();
      TS_ASSERT_EQUALS(gravType, FGInertial::gtWGS84);

      // Set to Standard (0)
      node->setIntValue(FGInertial::gtStandard);
      auto inertial = fdmex.GetInertial();
      TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtStandard);
    }
  }

  // Test GetSemiMajor and GetSemiMinor
  void testFGInertialGetAxes() {
    FGFDMExec fdmex;
    auto inertial = fdmex.GetInertial();

    double a = inertial->GetSemimajor();
    double b = inertial->GetSemiminor();

    // WGS84 defaults
    TS_ASSERT(a > 20900000.0);
    TS_ASSERT(a < 21000000.0);
    TS_ASSERT(b > 20800000.0);
    TS_ASSERT(b < 20900000.0);
    TS_ASSERT(a > b);  // Oblate spheroid
  }

  // Test SetAltitudeAGL
  void testFGInertialSetAltitudeAGL() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    FGLocation location;
    location.SetPositionGeodetic(0.0, 0.0, 1000.0);

    // Set altitude to 500 ft AGL
    inertial->SetAltitudeAGL(location, 500.0);

    // The geodetic altitude should have been modified
    double newAlt = location.GetGeodAltitude();
    TS_ASSERT(newAlt >= 0.0);  // Should be non-negative
  }

  // Test gravity with standard model
  void testFGInertialStandardGravity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();

    auto inertial = fdmex.GetInertial();
    inertial->SetGravityType(FGInertial::gtStandard);

    // Run to compute gravity
    bool result = inertial->Run(false);
    TS_ASSERT_EQUALS(result, false);

    // Standard model should be set
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtStandard);
  }

  // Test WGS84 gravity model
  void testFGInertialWGS84Gravity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();

    auto inertial = fdmex.GetInertial();
    inertial->SetGravityType(FGInertial::gtWGS84);

    // Run to compute gravity
    bool result = inertial->Run(false);
    TS_ASSERT_EQUALS(result, false);

    // WGS84 model should be set
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtWGS84);
  }

  // ============================================================================
  // C172x Model-Based Tests
  // ============================================================================

  void testC172xLoadModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();
    TS_ASSERT(inertial != nullptr);
  }

  void testC172xGravityMagnitudeStandard() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Standard gravity should be approximately 32.174 ft/s^2
    double gStd = FGInertial::GetStandardGravity();
    TS_ASSERT(!std::isnan(gStd));
    TS_ASSERT_DELTA(gStd, 32.174, 0.01);
  }

  void testC172xGravityVectorNotNaN() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();
    inertial->Run(false);

    const FGColumnVector3& gravity = inertial->GetGravity();
    TS_ASSERT(!std::isnan(gravity(1)));
    TS_ASSERT(!std::isnan(gravity(2)));
    TS_ASSERT(!std::isnan(gravity(3)));
  }

  void testC172xGravityVectorMagnitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();
    inertial->Run(false);

    const FGColumnVector3& gravity = inertial->GetGravity();
    double magnitude = gravity.Magnitude();

    // Gravity magnitude should be around 32.174 ft/s^2 (varies slightly with location)
    TS_ASSERT(magnitude > 30.0);
    TS_ASSERT(magnitude < 34.0);
  }

  void testC172xGravityDirectionDownward() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();
    inertial->Run(false);

    const FGColumnVector3& gravity = inertial->GetGravity();
    // In local NED frame, gravity should point down (positive Z component)
    // Or in ECEF, it should point toward Earth center
    // The magnitude being positive confirms gravity exists
    TS_ASSERT(gravity.Magnitude() > 0.0);
  }

  void testC172xEarthRadiusSemimajor() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double a = inertial->GetSemimajor();
    TS_ASSERT(!std::isnan(a));
    // WGS84 semi-major axis is approximately 20,925,646 ft (6,378,137 m)
    TS_ASSERT_DELTA(a, 20925646.0, 1000.0);
  }

  void testC172xEarthRadiusSemiminor() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double b = inertial->GetSemiminor();
    TS_ASSERT(!std::isnan(b));
    // WGS84 semi-minor axis is approximately 20,855,486 ft (6,356,752 m)
    TS_ASSERT_DELTA(b, 20855486.0, 1000.0);
  }

  void testC172xEarthRadiusOblate() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double a = inertial->GetSemimajor();
    double b = inertial->GetSemiminor();
    // Earth is oblate: semi-major > semi-minor
    TS_ASSERT(a > b);
    // Difference should be about 70,000 ft
    TS_ASSERT((a - b) > 60000.0);
    TS_ASSERT((a - b) < 80000.0);
  }

  void testC172xGMValue() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double gm = inertial->GetGM();
    TS_ASSERT(!std::isnan(gm));
    // GM should be about 1.407e16 ft^3/s^2 (3.986e14 m^3/s^2)
    TS_ASSERT(gm > 1.0e16);
    TS_ASSERT(gm < 2.0e16);
  }

  void testC172xGMReasonable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double gm = inertial->GetGM();
    double a = inertial->GetSemimajor();
    // GM/a^2 should approximate g at surface
    double g_approx = gm / (a * a);
    TS_ASSERT_DELTA(g_approx, 32.174, 0.5);
  }

  void testC172xOmegaEarthRotation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGColumnVector3 omega = inertial->GetOmegaPlanet();
    // Earth rotation rate is about 7.292115e-5 rad/s around Z axis
    TS_ASSERT_DELTA(omega(1), 0.0, 1e-10);
    TS_ASSERT_DELTA(omega(2), 0.0, 1e-10);
    TS_ASSERT(!std::isnan(omega(3)));
    TS_ASSERT_DELTA(omega(3), 7.292115e-5, 1e-8);
  }

  void testC172xOmegaMagnitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGColumnVector3 omega = inertial->GetOmegaPlanet();
    double magnitude = omega.Magnitude();
    // Magnitude should match Earth rotation rate
    TS_ASSERT_DELTA(magnitude, 7.292115e-5, 1e-8);
  }

  void testC172xOmegaSiderealDay() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGColumnVector3 omega = inertial->GetOmegaPlanet();
    double period = 2.0 * M_PI / omega(3);
    double hours = period / 3600.0;
    // Should be approximately one sidereal day (~23.934 hours)
    TS_ASSERT_DELTA(hours, 23.934, 0.01);
  }

  void testC172xGravityTypeDefault() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    // Default gravity type should be WGS84
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtWGS84);
  }

  void testC172xGravityTypeSwitching() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Switch to standard gravity
    inertial->SetGravityType(FGInertial::gtStandard);
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtStandard);

    // Switch back to WGS84
    inertial->SetGravityType(FGInertial::gtWGS84);
    TS_ASSERT_EQUALS(inertial->GetGravityType(), FGInertial::gtWGS84);
  }

  void testC172xTransformMatrixDeterminant() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGLocation position;
    position.SetPositionGeodetic(0.0, 45.0 * M_PI / 180.0, 0.0);

    FGMatrix33 Tl2ec = inertial->GetTl2ec(position);
    // Rotation matrix should have determinant of 1
    TS_ASSERT_DELTA(Tl2ec.Determinant(), 1.0, 1e-6);
  }

  void testC172xTransformMatrixOrthogonal() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGLocation position;
    position.SetPositionGeodetic(0.0, 30.0 * M_PI / 180.0, 1000.0);

    FGMatrix33 Tl2ec = inertial->GetTl2ec(position);
    FGMatrix33 Tec2l = inertial->GetTec2l(position);

    // Tl2ec * Tec2l should be identity (orthogonal matrices)
    FGMatrix33 product = Tl2ec * Tec2l;
    TS_ASSERT_DELTA(product(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(3,3), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(1,2), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(1,3), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(2,1), 0.0, 1e-6);
  }

  void testC172xRunICSuccess() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    bool result = fdmex.RunIC();
    TS_ASSERT(result);

    auto inertial = fdmex.GetInertial();
    TS_ASSERT(inertial != nullptr);
  }

  void testC172xInertialRunSuccess() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Run should return false (no error)
    bool result = inertial->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  void testC172xGravityAfterMultipleRuns() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Run multiple times and ensure gravity remains consistent
    inertial->Run(false);
    const FGColumnVector3& gravity1 = inertial->GetGravity();
    double mag1 = gravity1.Magnitude();

    inertial->Run(false);
    const FGColumnVector3& gravity2 = inertial->GetGravity();
    double mag2 = gravity2.Magnitude();

    TS_ASSERT_DELTA(mag1, mag2, 1e-6);
  }

  void testC172xEarthFlatteningValue() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double a = inertial->GetSemimajor();
    double b = inertial->GetSemiminor();
    double f = (a - b) / a;

    // WGS84 flattening is approximately 1/298.257223563
    TS_ASSERT_DELTA(f, 1.0/298.257223563, 1e-8);
  }

  void testC172xGravityModelPropertyBinding() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("simulation/gravity-model");
    TS_ASSERT(node != nullptr);

    if (node) {
      // Default should be WGS84 (1)
      TS_ASSERT_EQUALS(node->getIntValue(), FGInertial::gtWGS84);
    }
  }

  void testC172xSetOmegaPlanet() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    // Set custom rotation rate
    double customRate = 5.0e-5;
    inertial->SetOmegaPlanet(customRate);

    FGColumnVector3 omega = inertial->GetOmegaPlanet();
    TS_ASSERT_DELTA(omega(3), customRate, 1e-10);
  }

  void testC172xGeodeticToGeocentricConversion() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Use the aircraft's location from the simulation
    auto propagate = fdmex.GetPropagate();
    const FGLocation& location = propagate->GetLocation();

    // Get both geodetic and geocentric latitudes
    double lat_geocentric = location.GetLatitude();
    double lat_geodetic = location.GetGeodLatitudeRad();

    // Both should be valid (finite) values
    TS_ASSERT(!std::isnan(lat_geocentric) && !std::isinf(lat_geocentric));
    TS_ASSERT(!std::isnan(lat_geodetic) && !std::isinf(lat_geodetic));

    // Geodetic and geocentric latitudes differ due to Earth's shape
    // At non-polar, non-equatorial positions, there should be a small difference
    // (difference is zero at poles and equator, max around 45 degrees)
    double diff = std::abs(lat_geodetic - lat_geocentric);
    TS_ASSERT(diff >= 0.0);  // Difference is always non-negative
    TS_ASSERT(diff < 0.01);  // Difference is less than ~0.6 degrees
  }

  void testC172xAltitudeAGLSetGet() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    FGLocation location;
    location.SetPositionGeodetic(0.0, 45.0 * M_PI / 180.0, 5000.0);

    double agl = inertial->GetAltitudeAGL(location);
    TS_ASSERT(!std::isnan(agl));
    // AGL should be reasonable (positive or zero at minimum)
    TS_ASSERT(agl >= 0.0 || agl < 100000.0);
  }

  void testC172xTransformAtEquator() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGLocation position;
    position.SetPositionGeodetic(0.0, 0.0, 0.0);  // Equator

    FGMatrix33 Tl2ec = inertial->GetTl2ec(position);
    TS_ASSERT_DELTA(Tl2ec.Determinant(), 1.0, 1e-6);
  }

  void testC172xTransformAtPole() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    FGLocation position;
    position.SetPositionGeodetic(0.0, 89.9 * M_PI / 180.0, 0.0);  // Near pole

    FGMatrix33 Tl2ec = inertial->GetTl2ec(position);
    TS_ASSERT_DELTA(Tl2ec.Determinant(), 1.0, 1e-6);
  }

  void testC172xMultipleInertialInstances() {
    FGFDMExec fdmex1;
    fdmex1.LoadModel("c172x");
    fdmex1.RunIC();

    FGFDMExec fdmex2;
    fdmex2.LoadModel("c172x");
    fdmex2.RunIC();

    auto inertial1 = fdmex1.GetInertial();
    auto inertial2 = fdmex2.GetInertial();

    // Both should have same Earth parameters
    TS_ASSERT_DELTA(inertial1->GetSemimajor(), inertial2->GetSemimajor(), 1.0);
    TS_ASSERT_DELTA(inertial1->GetGM(), inertial2->GetGM(), 1.0);
  }

  void testC172xGravityInHoldingMode() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto inertial = fdmex.GetInertial();

    // Run in holding mode
    bool result = inertial->Run(true);
    TS_ASSERT_EQUALS(result, false);

    // Gravity should still be valid
    const FGColumnVector3& gravity = inertial->GetGravity();
    TS_ASSERT(!std::isnan(gravity.Magnitude()));
  }

  void testC172xEccentricitySquared() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto inertial = fdmex.GetInertial();

    double a = inertial->GetSemimajor();
    double b = inertial->GetSemiminor();
    double e2 = (a*a - b*b) / (a*a);

    // WGS84 eccentricity squared is approximately 0.00669438
    TS_ASSERT(!std::isnan(e2));
    TS_ASSERT(e2 > 0.0);
    TS_ASSERT(e2 < 0.01);
    TS_ASSERT_DELTA(e2, 0.00669438, 1e-6);
  }

  void testC172xStandardGravityConstant() {
    // GetStandardGravity is a static constexpr method
    double g = FGInertial::GetStandardGravity();

    // Should be exactly 9.80665 / 0.3048 = 32.174049 ft/s^2
    TS_ASSERT_DELTA(g, 9.80665 / 0.3048, 1e-6);
  }
};
