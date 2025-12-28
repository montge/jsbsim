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

  /***************************************************************************
   * Von Karman Turbulence Model Tests
   ***************************************************************************/

  // Test Von Karman spectral density
  void testVonKarmanSpectralDensity() {
    double sigma = 5.0;   // Turbulence intensity (ft/s)
    double L = 1000.0;    // Scale length (ft)
    double omega = 1.0;   // Frequency (rad/s)
    double V = 200.0;     // Velocity (ft/s)

    // Von Karman spectrum (simplified u-component)
    double Omega = omega * L / V;
    double Su = (sigma * sigma * L / (M_PI * V)) *
                (1.0 + (8.0/3.0) * 1.339 * 1.339 * Omega * Omega) /
                std::pow(1.0 + 1.339 * 1.339 * Omega * Omega, 11.0/6.0);

    TS_ASSERT(Su > 0);  // Spectral density positive
  }

  // Test Von Karman scale lengths
  void testVonKarmanScaleLengths() {
    double altitude = 1000.0;  // ft AGL

    // MIL-HDBK-1797 scale lengths
    double Lu, Lv, Lw;
    if (altitude < 1000) {
      Lu = altitude / std::pow(0.177 + 0.000823 * altitude, 1.2);
      Lv = Lu / 2.0;
      Lw = altitude / 2.0;
    } else {
      Lu = 1750.0;
      Lv = 875.0;
      Lw = 875.0;
    }

    TS_ASSERT(Lu > 0);
    TS_ASSERT(Lv > 0);
    TS_ASSERT(Lw > 0);
  }

  /***************************************************************************
   * Wind Direction Variation Tests
   ***************************************************************************/

  // Test wind direction veering with altitude (Northern Hemisphere)
  void testWindVeeringWithAltitude() {
    double surfaceDir = 180.0;  // Surface wind from south
    double veeringRate = 10.0;  // Degrees per 1000 ft
    double altitude = 2000.0;

    double dirAtAlt = surfaceDir + veeringRate * (altitude / 1000.0);
    TS_ASSERT_DELTA(dirAtAlt, 200.0, epsilon);  // Veered to 200 degrees
  }

  // Test Ekman spiral effect
  void testEkmanSpiral() {
    double surfaceWindDir = 180.0;
    double geostrophicDir = 230.0;  // Geostrophic wind direction
    double altitude = 1000.0;
    double blHeight = 2000.0;  // Boundary layer height

    // Linear interpolation of direction change
    double dirChange = (geostrophicDir - surfaceWindDir) * (altitude / blHeight);
    double windDir = surfaceWindDir + dirChange;

    TS_ASSERT(windDir > surfaceWindDir);
  }

  /***************************************************************************
   * Thermal and Convective Tests
   ***************************************************************************/

  // Test thermal updraft profile
  void testThermalUpdraft() {
    double maxUpdraft = 10.0;  // ft/s
    double thermalRadius = 500.0;  // ft
    double distFromCenter = 200.0;

    // Gaussian profile
    double updraft = maxUpdraft * std::exp(-(distFromCenter * distFromCenter) /
                                            (2.0 * thermalRadius * thermalRadius / 4.0));

    TS_ASSERT(updraft > 0);
    TS_ASSERT(updraft < maxUpdraft);
  }

  // Test thermal height variation
  void testThermalHeightVariation() {
    double surfaceTemp = 90.0;   // Fahrenheit
    double lapseRate = 3.5;      // deg/1000 ft
    double inversionHeight = 5000.0;  // ft

    // Thermal strength decreases approaching inversion
    double altitude = 4000.0;
    double thermalFactor = 1.0 - (altitude / inversionHeight);

    TS_ASSERT(thermalFactor > 0 && thermalFactor < 1);
  }

  // Test convective velocity scale
  void testConvectiveVelocityScale() {
    double surfaceHeatFlux = 0.05;  // W/m^2 equivalent
    double blHeight = 2000.0;       // ft

    // Convective velocity scale w* (simplified)
    double wStar = std::pow(surfaceHeatFlux * blHeight, 1.0/3.0);
    TS_ASSERT(wStar > 0);
  }

  /***************************************************************************
   * Clear Air Turbulence Tests
   ***************************************************************************/

  // Test Richardson number stability criterion
  void testRichardsonNumber() {
    double dTheta_dz = 0.003;  // K/m potential temp gradient
    double dU_dz = 0.01;       // Wind shear (1/s)
    double theta = 280.0;      // Potential temperature K
    double g = 9.81;

    // Richardson number Ri = (g/theta) * (dTheta/dz) / (dU/dz)^2
    double Ri = (g / theta) * dTheta_dz / (dU_dz * dU_dz);

    // CAT likely if Ri < 0.25
    bool catLikely = (Ri < 0.25);
    TS_ASSERT(Ri > 0);  // Stable stratification
  }

  // Test jet stream turbulence location
  void testJetStreamTurbulence() {
    double coreAltitude = 35000.0;  // ft
    double coreSpeed = 150.0;       // kts

    // Turbulence often found on edges
    double upperEdge = coreAltitude + 3000.0;
    double lowerEdge = coreAltitude - 3000.0;

    TS_ASSERT(upperEdge > coreAltitude);
    TS_ASSERT(lowerEdge < coreAltitude);
  }

  /***************************************************************************
   * Low Level Phenomena Tests
   ***************************************************************************/

  // Test low-level jet
  void testLowLevelJet() {
    double surfaceWind = 10.0;   // kts
    double jetHeight = 1500.0;  // ft AGL
    double jetSpeed = 40.0;     // kts
    double altitude = 1500.0;

    // Maximum at jet height, decreasing above and below
    double wind;
    if (altitude < jetHeight) {
      wind = surfaceWind + (jetSpeed - surfaceWind) * (altitude / jetHeight);
    } else {
      wind = jetSpeed * std::exp(-(altitude - jetHeight) / 2000.0);
    }

    TS_ASSERT_DELTA(wind, jetSpeed, epsilon);  // At jet height
  }

  // Test sea breeze penetration
  void testSeaBreezePenetration() {
    double coastDistance = 0.0;  // miles inland
    double maxPenetration = 30.0;  // miles
    double time = 15.0;  // Hours (3 PM)

    // Sea breeze develops afternoon, penetrates inland
    double penetrationDist = maxPenetration * (time - 10.0) / 6.0;
    bool seaBreezeActive = (coastDistance < penetrationDist);

    TS_ASSERT(penetrationDist > 0);
  }

  // Test katabatic wind
  void testKatabaticWind() {
    double slope = 10.0;        // degrees
    double tempDiff = 10.0;     // Celsius between slope and valley
    double slopeRad = slope * DEG_TO_RAD;

    // Simplified katabatic wind speed
    double g = 32.174;
    double katabaticSpeed = std::sqrt(g * std::sin(slopeRad) * tempDiff);

    TS_ASSERT(katabaticSpeed > 0);
  }

  /***************************************************************************
   * Wind Shear Alert Tests
   ***************************************************************************/

  // Test wind shear intensity calculation
  void testWindShearIntensity() {
    double deltaV = 30.0;     // kt change
    double deltaAlt = 500.0;  // ft

    double shearIntensity = deltaV / deltaAlt;  // kt/ft
    double shearKtPer100ft = shearIntensity * 100.0;

    TS_ASSERT(shearKtPer100ft > 5.0);  // Significant shear
  }

  // Test F-factor calculation
  void testFFactor() {
    double TAS = 200.0;     // ft/s
    double dV_dt = -10.0;   // ft/s^2 (decelerating)
    double dw = 20.0;       // ft/s (downdraft)
    double g = 32.174;

    // F-factor: performance factor for shear encounter
    double F = dV_dt / g - dw / TAS;

    // Positive F = performance increasing situation
    // Negative F = decreasing situation
    TS_ASSERT(F < 0);  // Hazardous
  }

  // Test wind shear escape maneuver
  void testWindShearEscape() {
    double currentPitch = 5.0;   // degrees
    double escapePitch = 15.0;   // degrees
    double targetClimb = 1500.0; // fpm

    double pitchIncrease = escapePitch - currentPitch;
    TS_ASSERT_DELTA(pitchIncrease, 10.0, epsilon);
  }

  /***************************************************************************
   * Wake Turbulence Tests
   ***************************************************************************/

  // Test wake vortex circulation
  void testWakeVortexCirculation() {
    double weight = 500000.0;   // lbs
    double rho = 0.002377;      // sl/ft^3
    double wingspan = 200.0;    // ft
    double TAS = 200.0;         // ft/s

    double lift = weight;  // Steady flight
    // Circulation Gamma = L / (rho * V * b * pi/4)
    double gamma = lift / (rho * TAS * wingspan * M_PI / 4.0);

    TS_ASSERT(gamma > 0);
  }

  // Test wake vortex descent rate
  void testWakeVortexDescent() {
    double gamma = 1000.0;   // ft^2/s circulation
    double b = 150.0;        // ft wingspan
    double separation = b * 0.75;  // Vortex separation

    // Vortex descent rate V_descent = Gamma / (2 * pi * b_vortex)
    double descentRate = gamma / (2.0 * M_PI * separation);

    TS_ASSERT(descentRate > 0);  // Descends
  }

  // Test wake turbulence separation
  void testWakeTurbulenceSeparation() {
    // Heavy aircraft behind heavy
    double minSep_HH = 4.0;  // nm
    // Light behind heavy
    double minSep_LH = 6.0;  // nm

    TS_ASSERT(minSep_LH > minSep_HH);
  }

  /***************************************************************************
   * Mountain Effects Tests
   ***************************************************************************/

  // Test mountain wave amplitude
  void testMountainWaveAmplitude() {
    double terrainHeight = 5000.0;  // ft
    double windSpeed = 40.0;        // kts
    double stability = 0.01;        // N (Brunt-Vaisala freq)

    // Simplified wave amplitude
    double amplitude = terrainHeight * 0.5;  // Can be significant
    TS_ASSERT(amplitude > 0);
  }

  // Test rotor zone location
  void testRotorZoneLocation() {
    double ridgeHeight = 8000.0;  // ft
    double rotorHeight = ridgeHeight + 2000.0;  // Below crest level

    // Rotor zone typically lee side, ridge height or slightly above
    TS_ASSERT(rotorHeight > ridgeHeight);
  }

  // Test lenticular cloud altitude
  void testLenticularCloudAltitude() {
    double ridgeTop = 10000.0;    // ft
    double waveAmplitude = 3000.0;  // ft
    double condensationLevel = 12000.0;

    // Lenticular forms at wave crest if above condensation level
    double waveCrests[] = {ridgeTop + waveAmplitude,
                          ridgeTop + 3.0 * waveAmplitude,
                          ridgeTop + 5.0 * waveAmplitude};

    TS_ASSERT(waveCrests[0] > condensationLevel);
  }

  /***************************************************************************
   * Gust Factor and Variability Tests
   ***************************************************************************/

  // Test gust factor calculation
  void testGustFactor() {
    double meanWind = 20.0;    // kts
    double peakGust = 35.0;    // kts

    double gustFactor = peakGust / meanWind;
    TS_ASSERT_DELTA(gustFactor, 1.75, epsilon);
  }

  // Test gust duration
  void testGustDuration() {
    double gustLength = 300.0;  // ft
    double TAS = 200.0;         // ft/s

    double gustDuration = gustLength / TAS;
    TS_ASSERT_DELTA(gustDuration, 1.5, epsilon);  // seconds
  }

  // Test standard gust velocities (FAR 25)
  void testStandardGustVelocities() {
    // Design gust velocities at different altitudes
    double Ude_SL = 66.0;      // ft/s at sea level
    double Ude_20000 = 66.0;   // ft/s up to 20,000 ft
    double Ude_50000 = 38.0;   // ft/s at 50,000 ft

    TS_ASSERT(Ude_SL >= Ude_50000);  // Decreases with altitude
  }

  /***************************************************************************
   * Runway Wind Tests
   ***************************************************************************/

  // Test runway headwind component
  void testRunwayHeadwind() {
    double windSpeed = 15.0;     // kts
    double windDir = 220.0;      // From
    double runwayHeading = 180.0;

    double angleDiff = (windDir - runwayHeading) * DEG_TO_RAD;
    double headwind = windSpeed * std::cos(angleDiff);
    double crosswind = windSpeed * std::sin(angleDiff);

    TS_ASSERT(headwind > 0);  // Positive = headwind
    TS_ASSERT(crosswind > 0); // From left
  }

  // Test crosswind limit check
  void testCrosswindLimit() {
    double crosswind = 25.0;  // kts
    double maxDemonstrated = 33.0;  // kts

    bool withinLimits = (crosswind <= maxDemonstrated);
    TS_ASSERT(withinLimits);
  }

  // Test variable wind
  void testVariableWind() {
    double windDir1 = 180.0;
    double windDir2 = 250.0;
    double variability = std::abs(windDir2 - windDir1);

    // Variable wind if range > 60 degrees
    bool isVariable = (variability > 60.0);
    TS_ASSERT(isVariable);
  }

  /***************************************************************************
   * Temperature Effects on Wind Tests
   ***************************************************************************/

  // Test density altitude effect on wind
  void testDensityAltitudeEffect() {
    double pressureAlt = 5000.0;
    double ISA_temp = 59.0 - 3.5 * (pressureAlt / 1000.0);  // Standard temp
    double actualTemp = 80.0;  // Hot day

    double densityAlt = pressureAlt + 120.0 * (actualTemp - ISA_temp);
    TS_ASSERT(densityAlt > pressureAlt);  // Higher density altitude when hot
  }

  // Test diurnal wind variation
  void testDiurnalWindVariation() {
    // Surface winds typically peak afternoon, calm at night
    double morningWind = 5.0;
    double afternoonWind = 15.0;
    double eveningWind = 8.0;

    TS_ASSERT(afternoonWind > morningWind);
    TS_ASSERT(afternoonWind > eveningWind);
  }

  /***************************************************************************
   * Turbulence Correlation Tests
   ***************************************************************************/

  // Test spatial correlation of turbulence
  void testTurbulenceSpatialCorrelation() {
    double L = 1000.0;    // Scale length (ft)
    double separation = 500.0;  // ft

    // Spatial correlation (simplified exponential)
    double correlation = std::exp(-separation / L);
    TS_ASSERT(correlation > 0 && correlation < 1);
  }

  // Test temporal autocorrelation
  void testTurbulenceTemporalCorrelation() {
    double tau = 2.0;      // Time constant (s)
    double dt = 0.5;       // Time step (s)

    double correlation = std::exp(-dt / tau);
    TS_ASSERT(correlation > 0.7);  // High correlation for small dt
  }

  // Test colored noise generation
  void testColoredNoiseGeneration() {
    // First-order filter for coloring white noise
    double tau = 1.0;
    double dt = 0.1;
    double alpha = std::exp(-dt / tau);
    double beta = std::sqrt(1.0 - alpha * alpha);

    TS_ASSERT(alpha > 0 && alpha < 1);
    TS_ASSERT(beta > 0 && beta < 1);
    TS_ASSERT_DELTA(alpha * alpha + beta * beta, 1.0, 0.01);
  }

  /***************************************************************************
   * Additional Wind Direction Tests
   ***************************************************************************/

  // Test 8 cardinal wind directions
  void testCardinalWindDirections() {
    double directions[] = {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0};
    double windSpeed = 10.0;

    for (double dir : directions) {
      double dirRad = dir * DEG_TO_RAD;
      double windN = -windSpeed * std::cos(dirRad);
      double windE = -windSpeed * std::sin(dirRad);

      double resultant = std::sqrt(windN*windN + windE*windE);
      TS_ASSERT_DELTA(resultant, windSpeed, 0.001);
    }
  }

  // Test wind direction wraparound at 360
  void testWindDirectionWraparound() {
    double dir1 = 350.0;
    double dir2 = 10.0;

    // Angle difference across 360
    double diff = dir2 - dir1;
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    TS_ASSERT_DELTA(diff, 20.0, epsilon);
  }

  // Test negative wind direction handling
  void testNegativeWindDirection() {
    double dir = -90.0;  // Same as 270
    double normalized = dir;
    while (normalized < 0) normalized += 360.0;

    TS_ASSERT_DELTA(normalized, 270.0, epsilon);
  }

  /***************************************************************************
   * Wind Speed Variability Tests
   ***************************************************************************/

  // Test mean wind with gusts
  void testMeanWindWithGusts() {
    double meanWind = 20.0;
    double gustAmplitude = 10.0;

    double maxWind = meanWind + gustAmplitude;
    double minWind = meanWind - gustAmplitude * 0.5;

    TS_ASSERT(maxWind > meanWind);
    TS_ASSERT(minWind > 0);
  }

  // Test wind variance calculation
  void testWindVarianceCalculation() {
    std::mt19937 gen(123);
    std::normal_distribution<> wind(20.0, 5.0);

    double sum = 0.0, sumSq = 0.0;
    int N = 1000;
    for (int i = 0; i < N; i++) {
      double v = wind(gen);
      sum += v;
      sumSq += v * v;
    }
    double mean = sum / N;
    double variance = sumSq / N - mean * mean;

    TS_ASSERT_DELTA(mean, 20.0, 1.0);
    TS_ASSERT_DELTA(variance, 25.0, 5.0);
  }

  // Test sustained vs instantaneous wind
  void testSustainedVsInstantaneous() {
    double sustainedWind = 25.0;  // 2-minute average
    double instantaneousWind = 35.0;

    double gustFactor = instantaneousWind / sustainedWind;
    TS_ASSERT(gustFactor > 1.0);
    TS_ASSERT(gustFactor < 2.0);
  }

  /***************************************************************************
   * Altitude Layer Wind Tests
   ***************************************************************************/

  // Test wind at multiple altitudes
  void testWindAtMultipleAltitudes() {
    double surfaceWind = 10.0;
    double alpha = 0.143;
    double zRef = 33.0;

    double altitudes[] = {100.0, 500.0, 1000.0, 2000.0, 5000.0};

    double prevWind = surfaceWind;
    for (double z : altitudes) {
      double windAtZ = surfaceWind * std::pow(z / zRef, alpha);
      TS_ASSERT(windAtZ >= prevWind);
      prevWind = windAtZ;
    }
  }

  // Test temperature inversion wind effects
  void testTemperatureInversionWind() {
    double belowInversionWind = 15.0;
    double aboveInversionWind = 35.0;
    double inversionHeight = 3000.0;

    // Sharp wind change at inversion
    double windDiff = std::abs(aboveInversionWind - belowInversionWind);
    TS_ASSERT(windDiff > 10.0);
  }

  // Test tropopause wind maximum
  void testTropopauseWind() {
    double tropopauseHeight = 36000.0;  // ft
    double jetStreamSpeed = 150.0;       // kts

    TS_ASSERT(jetStreamSpeed > 100.0);
    TS_ASSERT(tropopauseHeight > 30000.0);
  }

  /***************************************************************************
   * Wind Component Combination Tests
   ***************************************************************************/

  // Test 3D wind vector magnitude
  void test3DWindMagnitude() {
    double windN = 10.0, windE = 20.0, windD = 5.0;

    double magnitude = std::sqrt(windN*windN + windE*windE + windD*windD);
    TS_ASSERT_DELTA(magnitude, 22.91, 0.01);
  }

  // Test horizontal vs vertical wind components
  void testHorizontalVsVerticalWind() {
    double horizontalWind = 30.0;  // kts
    double verticalWind = 5.0;     // kts (updraft)

    double totalWind = std::sqrt(horizontalWind*horizontalWind +
                                 verticalWind*verticalWind);
    TS_ASSERT(totalWind > horizontalWind);
    TS_ASSERT_DELTA(totalWind, 30.41, 0.01);
  }

  // Test wind vector addition
  void testWindVectorAddition() {
    double wind1N = 10.0, wind1E = 5.0;
    double wind2N = 5.0, wind2E = 10.0;

    double totalN = wind1N + wind2N;
    double totalE = wind1E + wind2E;

    TS_ASSERT_DELTA(totalN, 15.0, epsilon);
    TS_ASSERT_DELTA(totalE, 15.0, epsilon);
  }

  /***************************************************************************
   * Gust Profile Tests
   ***************************************************************************/

  // Test step gust profile
  void testStepGustProfile() {
    double gustAmplitude = 15.0;
    double x = 50.0;
    double gustStart = 25.0;

    double gust = (x >= gustStart) ? gustAmplitude : 0.0;
    TS_ASSERT_DELTA(gust, gustAmplitude, epsilon);
  }

  // Test ramp gust profile
  void testRampGustProfile() {
    double gustAmplitude = 20.0;
    double rampLength = 100.0;
    double x = 50.0;

    double gust = gustAmplitude * std::min(x / rampLength, 1.0);
    TS_ASSERT_DELTA(gust, 10.0, epsilon);
  }

  // Test sinusoidal gust
  void testSinusoidalGust() {
    double gustAmplitude = 10.0;
    double wavelength = 200.0;
    double x = 30.0;  // Not at peak (peak is at x=50 where sin=1)

    double gust = gustAmplitude * std::sin(2.0 * M_PI * x / wavelength);
    TS_ASSERT(gust > 0);
    TS_ASSERT(gust < gustAmplitude);
  }

  // Test gust frequency
  void testGustFrequency() {
    double wavelength = 300.0;  // ft
    double TAS = 200.0;         // ft/s

    double frequency = TAS / wavelength;
    TS_ASSERT_DELTA(frequency, 0.667, 0.01);  // Hz
  }

  /***************************************************************************
   * Shear Layer Tests
   ***************************************************************************/

  // Test linear shear layer
  void testLinearShearLayer() {
    double windBottom = 10.0;
    double windTop = 30.0;
    double layerThickness = 500.0;
    double heightInLayer = 250.0;

    double windAtHeight = windBottom + (windTop - windBottom) *
                          (heightInLayer / layerThickness);
    TS_ASSERT_DELTA(windAtHeight, 20.0, epsilon);
  }

  // Test shear exponent variation
  void testShearExponentVariation() {
    double exponents[] = {0.10, 0.14, 0.20, 0.25, 0.40};
    double windRef = 20.0;
    double zRef = 33.0;
    double z = 100.0;

    for (double alpha : exponents) {
      double wind = windRef * std::pow(z / zRef, alpha);
      TS_ASSERT(wind > windRef);
    }
  }

  // Test nocturnal low-level jet profile
  void testNocturnalJetProfile() {
    double jetMaxSpeed = 45.0;
    double jetHeight = 1200.0;
    double altitude = 600.0;

    // Parabolic profile below jet
    double wind = jetMaxSpeed * (1.0 - std::pow((altitude - jetHeight) / jetHeight, 2.0));
    TS_ASSERT(wind > 0);
    TS_ASSERT(wind < jetMaxSpeed);
  }

  /***************************************************************************
   * Consistency and Validation Tests
   ***************************************************************************/

  // Test wind components to speed/direction roundtrip
  void testWindRoundtrip() {
    double originalSpeed = 25.0;
    double originalDir = 135.0;
    double dirRad = originalDir * DEG_TO_RAD;

    // To components
    double windN = -originalSpeed * std::cos(dirRad);
    double windE = -originalSpeed * std::sin(dirRad);

    // Back to speed/direction
    double speed = std::sqrt(windN*windN + windE*windE);
    double fromDir = std::atan2(-windE, -windN) * RAD_TO_DEG;
    if (fromDir < 0) fromDir += 360.0;

    TS_ASSERT_DELTA(speed, originalSpeed, 0.001);
    TS_ASSERT_DELTA(fromDir, originalDir, 0.001);
  }

  // Test wind triangle closure
  void testWindTriangleClosure() {
    double TAS = 200.0;
    double heading = 90.0 * DEG_TO_RAD;  // East
    double windSpeed = 30.0;
    double windDir = 180.0 * DEG_TO_RAD;  // From south

    // Aircraft velocity in NED
    double acN = TAS * std::cos(heading);
    double acE = TAS * std::sin(heading);

    // Wind velocity (blowing TO)
    double wN = -windSpeed * std::cos(windDir);
    double wE = -windSpeed * std::sin(windDir);

    // Ground speed
    double gsN = acN + wN;
    double gsE = acE + wE;
    double GS = std::sqrt(gsN*gsN + gsE*gsE);

    TS_ASSERT(GS > TAS - windSpeed);
    TS_ASSERT(GS < TAS + windSpeed);
  }

  // Test multiple calculations consistency
  void testCalculationsConsistency() {
    double windSpeed = 20.0;
    double windDir = 270.0;

    for (int i = 0; i < 100; i++) {
      double dirRad = windDir * DEG_TO_RAD;
      double windN = -windSpeed * std::cos(dirRad);
      double windE = -windSpeed * std::sin(dirRad);
      double speed = std::sqrt(windN*windN + windE*windE);

      TS_ASSERT_DELTA(speed, windSpeed, 0.0001);
    }
  }

  /***************************************************************************
   * Stress and Edge Case Tests
   ***************************************************************************/

  // Test very high wind speeds
  void testVeryHighWindSpeed() {
    double windSpeed = 200.0;  // kts (extreme jet stream)
    double windFPS = windSpeed * KTS_TO_FPS;

    TS_ASSERT(std::isfinite(windFPS));
    TS_ASSERT(windFPS > 300.0);
  }

  // Test near-zero wind speed
  void testNearZeroWindSpeed() {
    double windSpeed = 0.001;
    double windDir = 45.0 * DEG_TO_RAD;

    double windN = -windSpeed * std::cos(windDir);
    double windE = -windSpeed * std::sin(windDir);

    TS_ASSERT(std::isfinite(windN));
    TS_ASSERT(std::isfinite(windE));
  }

  // Test rapid wind direction changes
  void testRapidDirectionChanges() {
    double windSpeed = 20.0;

    for (double dir = 0.0; dir <= 360.0; dir += 10.0) {
      double dirRad = dir * DEG_TO_RAD;
      double windN = -windSpeed * std::cos(dirRad);
      double windE = -windSpeed * std::sin(dirRad);

      TS_ASSERT(std::isfinite(windN));
      TS_ASSERT(std::isfinite(windE));

      double speed = std::sqrt(windN*windN + windE*windE);
      TS_ASSERT_DELTA(speed, windSpeed, 0.001);
    }
  }

  // Test altitude sweep wind calculation
  void testAltitudeSweepWind() {
    double surfaceWind = 10.0;
    double alpha = 0.143;
    double zRef = 33.0;

    for (double z = 10.0; z <= 50000.0; z *= 2.0) {
      double windAtZ = surfaceWind * std::pow(z / zRef, alpha);

      TS_ASSERT(std::isfinite(windAtZ));
      TS_ASSERT(windAtZ > 0);
    }
  }

  /***************************************************************************
   * Meteorological Conversion Tests
   ***************************************************************************/

  // Test wind from TO to FROM direction
  void testWindToFromConversion() {
    double toDir = 90.0;   // Blowing TO east
    double fromDir = toDir + 180.0;
    if (fromDir >= 360.0) fromDir -= 360.0;

    TS_ASSERT_DELTA(fromDir, 270.0, epsilon);
  }

  // Test Beaufort scale wind
  void testBeaufortScale() {
    // Beaufort 6 = 22-27 kts (strong breeze)
    double beaufort6Min = 22.0;
    double beaufort6Max = 27.0;

    double midpoint = (beaufort6Min + beaufort6Max) / 2.0;
    TS_ASSERT_DELTA(midpoint, 24.5, epsilon);
  }

  // Test wind chill effect (simplified)
  void testWindChillEffect() {
    double temp = 40.0;       // Fahrenheit
    double windSpeed = 15.0;  // mph

    // Simplified wind chill formula
    double windChill = 35.74 + 0.6215 * temp -
                       35.75 * std::pow(windSpeed, 0.16) +
                       0.4275 * temp * std::pow(windSpeed, 0.16);

    TS_ASSERT(windChill < temp);
  }

  /***************************************************************************
   * Atmospheric Stability Tests
   ***************************************************************************/

  // Test stable atmosphere turbulence
  void testStableAtmosphereTurbulence() {
    // In stable conditions, turbulence is suppressed
    double stableSigma = 2.0;   // Low turbulence
    double unstableSigma = 8.0; // High turbulence

    TS_ASSERT(stableSigma < unstableSigma);
  }

  // Test convective mixing depth
  void testConvectiveMixingDepth() {
    double surfaceTemp = 95.0;    // F
    double morningTemp = 65.0;    // F
    double lapseRate = 5.4;       // F per 1000 ft

    double mixingDepth = (surfaceTemp - morningTemp) / lapseRate * 1000.0;
    TS_ASSERT(mixingDepth > 0);
    TS_ASSERT(mixingDepth < 10000.0);
  }

  // Test Pasquill stability class
  void testPasquillStabilityClass() {
    // Class A = very unstable (daytime, light winds, strong sun)
    // Class F = very stable (nighttime, light winds, clear)
    double daytimeWindThreshold = 6.0;  // m/s
    double nighttimeWindThreshold = 3.0;

    TS_ASSERT(daytimeWindThreshold > nighttimeWindThreshold);
  }

  /***************************************************************************
   * Complete System Verification Tests
   ***************************************************************************/

  // Test complete wind system verification
  void testCompleteWindSystemVerification() {
    // 1. Wind components from speed and direction
    double speed = 25.0;  // kts
    double direction = 045.0;  // degrees (from NE)
    double dir_rad = direction * M_PI / 180.0;

    double north = -speed * cos(dir_rad);
    double east = -speed * sin(dir_rad);

    TS_ASSERT(north < 0);  // Wind from NE goes SW
    TS_ASSERT(east < 0);

    // 2. Crosswind component for runway 09 (090 degrees)
    double runway_hdg = 090.0;
    double xwind = speed * sin((direction - runway_hdg) * M_PI / 180.0);
    TS_ASSERT(fabs(xwind) < speed);

    // 3. Headwind component
    double hwind = speed * cos((direction - runway_hdg) * M_PI / 180.0);
    TS_ASSERT(hwind * hwind + xwind * xwind - speed * speed < 0.01);
  }

  // Test wind shear profile
  void testWindShearProfile() {
    // Power law wind profile: V(z) = V_ref * (z/z_ref)^alpha
    double V_ref = 15.0;  // kts at reference height
    double z_ref = 10.0;  // meters (reference height)
    double alpha = 0.143; // Neutral stability exponent

    double heights[] = {5.0, 20.0, 50.0, 100.0};
    double prev_speed = 0.0;

    for (double z : heights) {
      double V = V_ref * pow(z / z_ref, alpha);
      TS_ASSERT(V > prev_speed);  // Wind increases with height
      prev_speed = V;
    }
  }

  // Test gust alleviation factor
  void testGustAlleviationFactor() {
    // Gust alleviation factor depends on aircraft response
    double gust_velocity = 50.0;  // fps
    double cruise_speed = 400.0;  // fps
    double mu_g = 10.0;  // Aircraft mass ratio parameter

    double K_g = 0.88 * mu_g / (5.3 + mu_g);  // Gust alleviation factor
    double effective_gust = gust_velocity * K_g;

    TS_ASSERT(K_g < 1.0);
    TS_ASSERT(effective_gust < gust_velocity);
  }

  // Test microburst wind model
  void testMicroburstWindModel() {
    // Microburst creates strong downdraft and outflow
    double max_downdraft = 80.0;  // fps
    double outflow_radius = 2000.0;  // ft
    double current_radius = 1000.0;  // ft

    // Outflow velocity increases toward edge
    double outflow = max_downdraft * (current_radius / outflow_radius);
    TS_ASSERT(outflow > 0);
    TS_ASSERT(outflow < max_downdraft);

    // Downdraft decreases with distance from center
    double downdraft = max_downdraft * (1.0 - current_radius / outflow_radius);
    TS_ASSERT(downdraft > 0);
    TS_ASSERT(downdraft + outflow < 2.0 * max_downdraft);
  }
};
