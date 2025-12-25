/*******************************************************************************
 * FGWindsTest.h - Unit tests for wind and turbulence calculations
 *
 * Tests the mathematical behavior of wind modeling:
 * - Steady wind components
 * - Wind gradients with altitude
 * - Gusts and turbulence
 * - Wind shear modeling
 * - Body axis wind conversion
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double KTS_TO_FPS = 1.68781;  // Knots to ft/s

class FGWindsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Steady Wind Tests
   ***************************************************************************/

  // Test wind from direction to components
  void testWindComponents() {
    double windSpeed = 20.0;  // kts
    double windDir = 270.0;   // From west (standard meteorological)
    double windDirRad = windDir * DEG_TO_RAD;

    // Wind FROM direction, so components are opposite
    double windN = -windSpeed * std::cos(windDirRad);  // North component
    double windE = -windSpeed * std::sin(windDirRad);  // East component

    // Wind from 270 (west) means wind blowing TO the east
    TS_ASSERT_DELTA(windN, 0.0, 0.001);
    TS_ASSERT_DELTA(windE, 20.0, 0.001);
  }

  // Test wind from north
  void testWindFromNorth() {
    double windSpeed = 10.0;
    double windDir = 0.0;  // From north
    double windDirRad = windDir * DEG_TO_RAD;

    double windN = -windSpeed * std::cos(windDirRad);
    double windE = -windSpeed * std::sin(windDirRad);

    TS_ASSERT_DELTA(windN, -10.0, 0.001);  // Blowing south
    TS_ASSERT_DELTA(windE, 0.0, 0.001);
  }

  // Test wind from south
  void testWindFromSouth() {
    double windSpeed = 10.0;
    double windDir = 180.0;
    double windDirRad = windDir * DEG_TO_RAD;

    double windN = -windSpeed * std::cos(windDirRad);
    double windE = -windSpeed * std::sin(windDirRad);

    TS_ASSERT_DELTA(windN, 10.0, 0.001);  // Blowing north
    TS_ASSERT_DELTA(windE, 0.0, 0.001);
  }

  // Test resultant wind speed
  void testResultantWindSpeed() {
    double windN = 6.0;
    double windE = 8.0;

    double speed = std::sqrt(windN*windN + windE*windE);
    TS_ASSERT_DELTA(speed, 10.0, epsilon);
  }

  // Test resultant wind direction
  void testResultantWindDirection() {
    double windN = 0.0;
    double windE = 10.0;  // Wind blowing east

    // Wind FROM direction (opposite of velocity)
    double fromDir = std::atan2(-windE, -windN) * RAD_TO_DEG;
    if (fromDir < 0) fromDir += 360.0;

    TS_ASSERT_DELTA(fromDir, 270.0, 0.1);  // From west
  }

  /***************************************************************************
   * Wind Gradient Tests
   ***************************************************************************/

  // Test logarithmic wind profile
  void testLogWindProfile() {
    double windRef = 20.0;   // Reference wind speed at 10m
    double zRef = 32.8;      // Reference height (10m in ft)
    double z0 = 0.1;         // Roughness length (ft)
    double z = 100.0;        // Height to calculate (ft)

    // Logarithmic wind profile: V(z) = V_ref * ln(z/z0) / ln(z_ref/z0)
    double windAtZ = windRef * std::log(z / z0) / std::log(zRef / z0);

    TS_ASSERT(windAtZ > windRef);  // Wind increases with height
  }

  // Test power law wind profile
  void testPowerLawProfile() {
    double windRef = 20.0;
    double zRef = 32.8;
    double z = 100.0;
    double alpha = 0.143;  // Typical value for open terrain

    double windAtZ = windRef * std::pow(z / zRef, alpha);
    TS_ASSERT(windAtZ > windRef);
  }

  // Test wind at ground level
  void testWindAtGround() {
    double windRef = 20.0;
    double zRef = 32.8;
    double z = 0.1;  // Near ground
    double alpha = 0.143;

    double windAtZ = windRef * std::pow(z / zRef, alpha);
    TS_ASSERT(windAtZ < windRef);  // Wind slower near ground
  }

  // Test boundary layer gradient
  void testBoundaryLayerGradient() {
    double surfaceWind = 10.0;
    double gradientHeight = 2000.0;  // ft
    double geostrophicWind = 30.0;   // Wind above boundary layer

    // Linear interpolation in boundary layer
    double altitude = 1000.0;
    double wind = surfaceWind + (geostrophicWind - surfaceWind) *
                  (altitude / gradientHeight);

    TS_ASSERT_DELTA(wind, 20.0, epsilon);  // Midpoint
  }

  /***************************************************************************
   * Gust Tests
   ***************************************************************************/

  // Test discrete gust (1-cos profile)
  void testDiscreteGust() {
    double gustAmplitude = 10.0;  // ft/s
    double gustLength = 100.0;    // ft
    double x = 50.0;              // Position in gust (ft)

    // 1-cosine gust profile
    double gust = 0.0;
    if (x >= 0 && x <= gustLength) {
      gust = (gustAmplitude / 2.0) * (1.0 - std::cos(2.0 * M_PI * x / gustLength));
    }

    TS_ASSERT_DELTA(gust, gustAmplitude, epsilon);  // Peak at center
  }

  // Test gust outside region
  void testGustOutside() {
    double gustAmplitude = 10.0;
    double gustLength = 100.0;
    double x = 150.0;  // Beyond gust

    double gust = 0.0;
    if (x >= 0 && x <= gustLength) {
      gust = (gustAmplitude / 2.0) * (1.0 - std::cos(2.0 * M_PI * x / gustLength));
    }

    TS_ASSERT_DELTA(gust, 0.0, epsilon);
  }

  // Test gust at start/end
  void testGustBoundary() {
    double gustAmplitude = 10.0;
    double gustLength = 100.0;

    // At start (x=0)
    double gust0 = (gustAmplitude / 2.0) * (1.0 - std::cos(0.0));
    TS_ASSERT_DELTA(gust0, 0.0, epsilon);

    // At end (x=gustLength)
    double gustEnd = (gustAmplitude / 2.0) *
                     (1.0 - std::cos(2.0 * M_PI));
    TS_ASSERT_DELTA(gustEnd, 0.0, epsilon);
  }

  /***************************************************************************
   * Turbulence Tests
   ***************************************************************************/

  // Test Dryden turbulence intensity
  void testDrydenIntensity() {
    double altitude = 1000.0;  // ft AGL
    double sigma_w = 0.1 * 6.0;  // Light turbulence (W20 = 6 ft/s)

    // At low altitude, intensity varies with height
    double sigma_u = sigma_w / std::pow(0.177 + 0.000823 * altitude, 0.4);

    TS_ASSERT(sigma_u > 0);
  }

  // Test turbulence scale length
  void testTurbulenceScale() {
    double altitude = 1000.0;

    // Dryden scale lengths (simplified)
    double L_w = altitude;  // Vertical scale
    double L_u = altitude / std::pow(0.177 + 0.000823 * altitude, 1.2);

    TS_ASSERT(L_u > 0);
    TS_ASSERT(L_w > 0);
  }

  // Test random turbulence has zero mean
  void testTurbulenceZeroMean() {
    std::mt19937 gen(42);
    std::normal_distribution<> turbulence(0.0, 5.0);

    double sum = 0.0;
    int N = 10000;
    for (int i = 0; i < N; i++) {
      sum += turbulence(gen);
    }

    double mean = sum / N;
    TS_ASSERT_DELTA(mean, 0.0, 0.5);  // Should be near zero
  }

  // Test turbulence intensity levels
  void testTurbulenceIntensityLevels() {
    // MIL-F-8785C categories
    double light = 3.0;     // ft/s RMS
    double moderate = 6.0;  // ft/s RMS
    double severe = 12.0;   // ft/s RMS

    TS_ASSERT(light < moderate);
    TS_ASSERT(moderate < severe);
  }

  /***************************************************************************
   * Wind Shear Tests
   ***************************************************************************/

  // Test microburst wind profile
  void testMicroburstProfile() {
    double maxDowndraft = 50.0;  // ft/s
    double radius = 2000.0;      // ft
    double r = 1000.0;           // Distance from center

    // Simplified radial outflow
    double outflow = maxDowndraft * (r / radius) * std::exp(1 - r/radius);
    TS_ASSERT(outflow > 0);
  }

  // Test headwind to tailwind shear
  void testHeadwindToTailwindShear() {
    double initialHeadwind = 20.0;
    double finalTailwind = -20.0;  // Negative = tailwind
    double shearDistance = 1000.0; // ft

    double shearRate = (finalTailwind - initialHeadwind) / shearDistance;
    TS_ASSERT_DELTA(shearRate, -0.04, epsilon);  // ft/s per ft
  }

  // Test vertical wind shear
  void testVerticalWindShear() {
    double downdraft = 30.0;  // ft/s
    double altitude = 500.0;  // ft AGL

    // Energy loss from downdraft
    double energyLoss = downdraft * altitude;  // ft^2/s
    TS_ASSERT(energyLoss > 0);
  }

  /***************************************************************************
   * Body Axis Conversion Tests
   ***************************************************************************/

  // Test NED to body conversion (no rotation)
  void testNEDToBodyNoRotation() {
    double windN = 10.0, windE = 0.0, windD = 0.0;
    double psi = 0.0;  // Heading north

    // Simplified (level flight, heading north)
    double windX = windN * std::cos(psi) + windE * std::sin(psi);
    double windY = -windN * std::sin(psi) + windE * std::cos(psi);

    TS_ASSERT_DELTA(windX, 10.0, epsilon);  // Headwind
    TS_ASSERT_DELTA(windY, 0.0, epsilon);   // No crosswind
  }

  // Test crosswind calculation
  void testCrosswindCalculation() {
    double windN = 0.0, windE = 10.0;  // Wind from west (blowing east)
    double psi = 0.0;  // Heading north

    double windX = windN * std::cos(psi) + windE * std::sin(psi);
    double windY = -windN * std::sin(psi) + windE * std::cos(psi);

    TS_ASSERT_DELTA(windX, 0.0, epsilon);    // No headwind
    TS_ASSERT_DELTA(windY, 10.0, epsilon);   // Right crosswind
  }

  // Test with aircraft heading east
  void testWindHeadingEast() {
    double windN = 10.0, windE = 0.0;
    double psi = 90.0 * DEG_TO_RAD;  // Heading east

    double windX = windN * std::cos(psi) + windE * std::sin(psi);
    double windY = -windN * std::sin(psi) + windE * std::cos(psi);

    TS_ASSERT_DELTA(windX, 0.0, 0.001);    // No headwind
    TS_ASSERT_DELTA(windY, -10.0, 0.001);  // Left crosswind
  }

  /***************************************************************************
   * Relative Wind Tests
   ***************************************************************************/

  // Test true airspeed from ground speed and wind
  void testTrueAirspeed() {
    double groundSpeed = 200.0;  // ft/s
    double headwind = 30.0;      // ft/s

    double TAS = groundSpeed + headwind;
    TS_ASSERT_DELTA(TAS, 230.0, epsilon);
  }

  // Test tailwind effect
  void testTailwindEffect() {
    double groundSpeed = 200.0;
    double tailwind = -30.0;  // Negative = tailwind

    double TAS = groundSpeed + tailwind;
    TS_ASSERT_DELTA(TAS, 170.0, epsilon);
  }

  // Test crosswind component
  void testCrosswindComponent() {
    double windSpeed = 20.0;
    double windAngle = 30.0 * DEG_TO_RAD;  // 30 degrees off runway

    double crosswind = windSpeed * std::sin(windAngle);
    double headwind = windSpeed * std::cos(windAngle);

    TS_ASSERT_DELTA(crosswind, 10.0, 0.1);
    TS_ASSERT_DELTA(headwind, 17.32, 0.1);
  }

  /***************************************************************************
   * Angle of Attack and Sideslip from Wind
   ***************************************************************************/

  // Test alpha from vertical wind component
  void testAlphaFromVerticalWind() {
    double TAS = 200.0;  // ft/s
    double verticalWind = 10.0;  // Updraft

    // Simplified: alpha change ≈ atan(w_wind / TAS)
    double deltaAlpha = std::atan(verticalWind / TAS) * RAD_TO_DEG;
    TS_ASSERT_DELTA(deltaAlpha, 2.86, 0.1);  // About 3 degrees
  }

  // Test beta from lateral wind component
  void testBetaFromLateralWind() {
    double TAS = 200.0;
    double lateralWind = 10.0;  // Right crosswind

    double deltaBeta = std::atan(lateralWind / TAS) * RAD_TO_DEG;
    TS_ASSERT_DELTA(deltaBeta, 2.86, 0.1);
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test knots to ft/s
  void testKnotsToFPS() {
    double kts = 100.0;
    double fps = kts * KTS_TO_FPS;

    TS_ASSERT_DELTA(fps, 168.781, 0.01);
  }

  // Test m/s to ft/s
  void testMPSToFPS() {
    double mps = 10.0;
    double fps = mps * 3.28084;

    TS_ASSERT_DELTA(fps, 32.8084, 0.01);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero wind
  void testZeroWind() {
    double windN = 0.0, windE = 0.0;

    double speed = std::sqrt(windN*windN + windE*windE);
    TS_ASSERT_DELTA(speed, 0.0, epsilon);
  }

  // Test calm at altitude (rare but possible)
  void testCalmConditions() {
    double windSpeed = 0.0;
    double gustSpeed = 0.0;

    double totalWind = windSpeed + gustSpeed;
    TS_ASSERT_DELTA(totalWind, 0.0, epsilon);
  }

  // Test very high wind
  void testHighWind() {
    double windSpeed = 100.0;  // kts (hurricane force)
    double windFPS = windSpeed * KTS_TO_FPS;

    TS_ASSERT(windFPS > 150.0);  // ft/s
  }
};
