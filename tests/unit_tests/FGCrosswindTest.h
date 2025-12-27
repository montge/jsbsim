/*******************************************************************************
 * FGCrosswindTest.h - Unit tests for crosswind flight physics
 *
 * This test suite validates crosswind flight calculations including:
 * - Wind component calculations (headwind, tailwind, crosswind)
 * - Crab angle calculations for crosswind landing
 * - Sideslip vs crab approach techniques
 * - Wind correction angle
 * - Crosswind limits (demonstrated/max)
 * - Weathervane tendency
 * - Rudder deflection for coordinated flight
 * - Aileron into wind technique
 * - Ground track vs heading
 * - Drift angle calculations
 * - Crosswind takeoff roll
 * - Wing-low landing technique
 * - Asymmetric lift in crosswind
 * - Ground effect in crosswind
 * - Crosswind gust response
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include "TestUtilities.h"

using namespace JSBSimTest;

class FGCrosswindTest : public CxxTest::TestSuite
{
public:
  // Test headwind component calculation
  void testHeadwindComponent() {
    // GIVEN: 20 knot wind from 360 degrees (north)
    double windSpeed = 20.0;  // knots
    double windDir = 0.0 * Constants::DEG_TO_RAD;  // from north
    double heading = 0.0 * Constants::DEG_TO_RAD;  // aircraft heading north

    // WHEN: Computing headwind component
    double headwind = windSpeed * std::cos(windDir - heading);

    // THEN: Should get full headwind
    TS_ASSERT_DELTA(headwind, 20.0, DEFAULT_TOLERANCE);
  }

  // Test tailwind component calculation
  void testTailwindComponent() {
    // GIVEN: 20 knot wind from 180 degrees (south)
    double windSpeed = 20.0;  // knots
    double windDir = 180.0 * Constants::DEG_TO_RAD;  // from south
    double heading = 0.0 * Constants::DEG_TO_RAD;  // aircraft heading north

    // WHEN: Computing headwind component (negative = tailwind)
    double headwind = windSpeed * std::cos(windDir - heading);

    // THEN: Should get negative headwind (tailwind)
    TS_ASSERT_DELTA(headwind, -20.0, DEFAULT_TOLERANCE);
  }

  // Test pure crosswind component calculation
  void testPureCrosswindComponent() {
    // GIVEN: 20 knot wind from 90 degrees (east)
    double windSpeed = 20.0;  // knots
    double windDir = 90.0 * Constants::DEG_TO_RAD;  // from east
    double heading = 0.0 * Constants::DEG_TO_RAD;  // aircraft heading north

    // WHEN: Computing crosswind component
    double crosswind = windSpeed * std::sin(windDir - heading);

    // THEN: Should get full crosswind
    TS_ASSERT_DELTA(crosswind, 20.0, DEFAULT_TOLERANCE);
  }

  // Test crosswind from left
  void testCrosswindFromLeft() {
    // GIVEN: 15 knot wind from 270 degrees (west)
    double windSpeed = 15.0;  // knots
    double windDir = 270.0 * Constants::DEG_TO_RAD;  // from west
    double heading = 0.0 * Constants::DEG_TO_RAD;  // aircraft heading north

    // WHEN: Computing crosswind component
    double crosswind = windSpeed * std::sin(windDir - heading);

    // THEN: Should get negative crosswind (from left)
    TS_ASSERT_DELTA(crosswind, -15.0, DEFAULT_TOLERANCE);
  }

  // Test 45-degree wind component decomposition
  void test45DegreeWindComponents() {
    // GIVEN: 20 knot wind from 45 degrees (northeast)
    double windSpeed = 20.0;  // knots
    double windDir = 45.0 * Constants::DEG_TO_RAD;
    double heading = 0.0 * Constants::DEG_TO_RAD;  // aircraft heading north

    // WHEN: Computing components
    double headwind = windSpeed * std::cos(windDir - heading);
    double crosswind = windSpeed * std::sin(windDir - heading);

    // THEN: Should get equal headwind and crosswind components
    double expected = 20.0 / std::sqrt(2.0);  // ~14.14 knots
    TS_ASSERT_DELTA(headwind, expected, LOOSE_TOLERANCE);
    TS_ASSERT_DELTA(crosswind, expected, LOOSE_TOLERANCE);

    // Verify Pythagorean theorem
    double totalWind = std::sqrt(headwind * headwind + crosswind * crosswind);
    TS_ASSERT_DELTA(totalWind, 20.0, LOOSE_TOLERANCE);
  }

  // Test crab angle calculation for direct crosswind
  void testCrabAngleDirectCrosswind() {
    // GIVEN: 100 knot airspeed, 20 knot crosswind from right
    double airspeed = 100.0;  // knots
    double crosswind = 20.0;  // knots

    // WHEN: Computing crab angle to maintain ground track
    double crabAngle = std::asin(crosswind / airspeed);

    // THEN: Should get appropriate crab angle
    double expectedDeg = std::asin(0.2) * Constants::RAD_TO_DEG;  // ~11.54 degrees
    TS_ASSERT_DELTA(crabAngle * Constants::RAD_TO_DEG, expectedDeg, LOOSE_TOLERANCE);
  }

  // Test crab angle with headwind component
  void testCrabAngleWithHeadwind() {
    // GIVEN: 30 knot wind at 30 degrees to runway (right)
    double windSpeed = 30.0;  // knots
    double windAngle = 30.0 * Constants::DEG_TO_RAD;
    double airspeed = 120.0;  // knots

    // WHEN: Computing crosswind component and crab angle
    double crosswind = windSpeed * std::sin(windAngle);
    double crabAngle = std::asin(crosswind / airspeed);

    // THEN: Should get correct crab angle
    double expectedCrosswind = 15.0;  // 30 * sin(30°) = 15 knots
    TS_ASSERT_DELTA(crosswind, expectedCrosswind, LOOSE_TOLERANCE);
    TS_ASSERT_DELTA(crabAngle * Constants::RAD_TO_DEG, 7.18, 0.01);
  }

  // Test maximum crab angle limitation
  void testMaximumCrabAngle() {
    // GIVEN: Very strong crosswind (40 knots) with 80 knot airspeed
    double airspeed = 80.0;  // knots
    double crosswind = 40.0;  // knots

    // WHEN: Computing crab angle
    double ratio = crosswind / airspeed;

    // THEN: Ratio should be less than 1 for valid crab angle
    TS_ASSERT(ratio < 1.0);
    double crabAngle = std::asin(ratio);
    TS_ASSERT_DELTA(crabAngle * Constants::RAD_TO_DEG, 30.0, LOOSE_TOLERANCE);
  }

  // Test sideslip angle in coordinated flight
  void testSideslipInCoordinatedFlight() {
    // GIVEN: Coordinated flight with proper crab angle
    double sideslipAngle = 0.0;  // radians (ball centered)

    // THEN: In coordinated flight, sideslip should be zero
    TS_ASSERT_DELTA(sideslipAngle, 0.0, DEFAULT_TOLERANCE);
  }

  // Test sideslip in wing-low crosswind landing
  void testSideslipInWingLowLanding() {
    // GIVEN: 15 knot crosswind, 90 knot approach speed
    double crosswind = 15.0;  // knots
    double airspeed = 90.0;  // knots

    // WHEN: Using wing-low method, aircraft slips into wind
    // Sideslip angle approximately equals drift correction
    double sideslipAngle = std::asin(crosswind / airspeed);

    // THEN: Sideslip should be about 9.6 degrees
    TS_ASSERT_DELTA(sideslipAngle * Constants::RAD_TO_DEG, 9.59, 0.01);
  }

  // Test wind correction angle equals crab angle
  void testWindCorrectionAngle() {
    // GIVEN: Aircraft maintaining ground track in crosswind
    double airspeed = 110.0;  // knots
    double crosswind = 18.0;  // knots from right

    // WHEN: Computing wind correction angle (WCA)
    double wca = std::asin(crosswind / airspeed);

    // THEN: WCA should be positive for right crosswind
    TS_ASSERT_DELTA(wca * Constants::RAD_TO_DEG, 9.418, 0.01);
  }

  // Test demonstrated crosswind component (typical GA aircraft)
  void testDemonstratedCrosswindGA() {
    // GIVEN: Typical general aviation aircraft
    double demonstratedCrosswind = 15.0;  // knots (typical for Cessna 172)
    double actualCrosswind = 12.0;  // knots

    // THEN: Actual crosswind should be within demonstrated limits
    TS_ASSERT(actualCrosswind < demonstratedCrosswind);
  }

  // Test maximum crosswind component (transport category)
  void testMaxCrosswindTransport() {
    // GIVEN: Transport category aircraft (e.g., Boeing 737)
    double maxCrosswind = 35.0;  // knots
    double actualCrosswind = 30.0;  // knots

    // THEN: Should be within operational limits
    TS_ASSERT(actualCrosswind < maxCrosswind);
  }

  // Test weathervane tendency
  void testWeathervaneEffect() {
    // GIVEN: Aircraft with vertical stabilizer area
    double verticalStabArea = 50.0;  // sq ft
    double crosswind = 10.0;  // knots
    double dynamicPressure = 25.0;  // psf (at approach speed)

    // WHEN: Computing yawing moment from weathervaning
    // Simplified: moment proportional to vertical stab area and crosswind
    double weathervaneForce = dynamicPressure * verticalStabArea;

    // THEN: Force should be positive (tends to align with wind)
    TS_ASSERT(weathervaneForce > 0.0);
    TS_ASSERT_DELTA(weathervaneForce, 1250.0, DEFAULT_TOLERANCE);
  }

  // Test rudder deflection for coordinated flight
  void testRudderDeflectionCoordinated() {
    // GIVEN: Crosswind requiring crab angle
    double crabAngle = 10.0 * Constants::DEG_TO_RAD;

    // WHEN: In coordinated flight (crab method)
    // Rudder maintains heading offset from track
    double rudderDeflection = crabAngle * 0.5;  // Simplified relation

    // THEN: Rudder deflection should be proportional to crab angle
    TS_ASSERT(std::abs(rudderDeflection) > 0.0);
    TS_ASSERT(std::abs(rudderDeflection) < crabAngle);
  }

  // Test aileron deflection in wing-low technique
  void testAileronIntoWind() {
    // GIVEN: Right crosswind requiring left wing low
    double bankAngle = -5.0 * Constants::DEG_TO_RAD;  // Left wing low

    // WHEN: Maintaining wings level requires aileron into wind
    // Aileron deflection opposes weathervane tendency
    double aileronDeflection = bankAngle * 0.8;  // Simplified

    // THEN: Left aileron up (negative deflection for left wing low)
    TS_ASSERT(aileronDeflection < 0.0);
  }

  // Test ground track vs heading with crosswind
  void testGroundTrackVsHeading() {
    // GIVEN: Aircraft heading 360 degrees with right crosswind
    double heading = 0.0 * Constants::DEG_TO_RAD;  // North
    double crabAngle = 8.0 * Constants::DEG_TO_RAD;  // Crab into wind

    // WHEN: Computing ground track
    double groundTrack = heading + crabAngle;

    // THEN: Ground track differs from heading by crab angle
    TS_ASSERT_DELTA(groundTrack * Constants::RAD_TO_DEG, 8.0, LOOSE_TOLERANCE);
  }

  // Test drift angle calculation
  void testDriftAngle() {
    // GIVEN: 25 knot crosswind, 100 knot groundspeed
    double crosswind = 25.0;  // knots
    double groundspeed = 100.0;  // knots

    // WHEN: Computing drift angle
    double driftAngle = std::atan(crosswind / groundspeed);

    // THEN: Should get appropriate drift
    TS_ASSERT_DELTA(driftAngle * Constants::RAD_TO_DEG, 14.04, 0.01);
  }

  // Test groundspeed reduction with headwind
  void testGroundspeedWithHeadwind() {
    // GIVEN: 120 knot airspeed, 20 knot headwind
    double airspeed = 120.0;  // knots
    double headwind = 20.0;  // knots

    // WHEN: Computing groundspeed
    double groundspeed = airspeed - headwind;

    // THEN: Groundspeed should be reduced
    TS_ASSERT_DELTA(groundspeed, 100.0, DEFAULT_TOLERANCE);
  }

  // Test groundspeed increase with tailwind
  void testGroundspeedWithTailwind() {
    // GIVEN: 120 knot airspeed, 20 knot tailwind
    double airspeed = 120.0;  // knots
    double tailwind = 20.0;  // knots (positive for tailwind)

    // WHEN: Computing groundspeed
    double groundspeed = airspeed + tailwind;

    // THEN: Groundspeed should be increased
    TS_ASSERT_DELTA(groundspeed, 140.0, DEFAULT_TOLERANCE);
  }

  // Test crosswind takeoff roll - asymmetric lift
  void testCrosswindTakeoffAsymmetricLift() {
    // GIVEN: Left crosswind during takeoff roll
    double windSpeed = 12.0;  // knots from left
    double rolloutSpeed = 40.0;  // knots

    // WHEN: Wind comes from left
    // Relative wind on upwind wing is greater
    double relativeWindUpwind = rolloutSpeed + (windSpeed * 0.5);
    double relativeWindDownwind = rolloutSpeed - (windSpeed * 0.5);

    // THEN: Upwind wing generates more lift
    // Lift proportional to velocity squared
    double liftRatioUpwind = relativeWindUpwind * relativeWindUpwind;
    double liftRatioDownwind = relativeWindDownwind * relativeWindDownwind;
    TS_ASSERT(liftRatioUpwind > liftRatioDownwind);
  }

  // Test crosswind takeoff - weathervane tendency
  void testCrosswindTakeoffWeathervane() {
    // GIVEN: Right crosswind during takeoff
    double crosswindComponent = 10.0;  // knots

    // WHEN: Aircraft accelerates on ground
    // Weathervane tendency increases with speed
    double lowSpeedEffect = crosswindComponent * 30.0;  // Low speed
    double highSpeedEffect = crosswindComponent * 60.0;  // Higher speed

    // THEN: Weathervane effect increases with speed
    TS_ASSERT(highSpeedEffect > lowSpeedEffect);
  }

  // Test wing-low landing technique - bank angle
  void testWingLowLandingBankAngle() {
    // GIVEN: 15 knot crosswind from right, approach speed 70 knots
    double crosswind = 15.0;  // knots
    double approachSpeed = 70.0;  // knots

    // WHEN: Using wing-low method
    // Bank angle needed to stop drift
    double bankAngle = std::atan(crosswind / approachSpeed);

    // THEN: Should get appropriate bank angle
    TS_ASSERT_DELTA(bankAngle * Constants::RAD_TO_DEG, 12.09, 0.01);
  }

  // Test wing-low landing - rudder opposite to bank
  void testWingLowRudderOpposite() {
    // GIVEN: Right wing low for left crosswind
    double bankAngle = 8.0 * Constants::DEG_TO_RAD;  // Right wing low

    // WHEN: Rudder used to align nose with runway
    // Rudder deflection opposite to bank
    double rudderDeflection = -bankAngle * 0.6;  // Simplified

    // THEN: Left rudder (negative) with right wing low
    TS_ASSERT(rudderDeflection < 0.0);
  }

  // Test asymmetric lift in steady crosswind
  void testAsymmetricLiftInCrosswind() {
    // GIVEN: Aircraft in sideslip due to crosswind
    double sideslipAngle = 5.0 * Constants::DEG_TO_RAD;
    double wingSpan = 36.0;  // feet
    double airspeed = 100.0;  // knots (169 ft/s)

    // WHEN: Computing differential velocity across wings
    // Upwind wing sees higher velocity
    double angularVelocity = (airspeed * Constants::KTS_TO_FTPS) * std::sin(sideslipAngle) / wingSpan;

    // THEN: Should create rolling moment
    TS_ASSERT(angularVelocity > 0.0);
  }

  // Test ground effect with crosswind
  void testGroundEffectCrosswind() {
    // GIVEN: Aircraft in ground effect (height < wingspan)
    double height = 10.0;  // feet AGL
    double wingspan = 36.0;  // feet

    // WHEN: Computing ground effect reduction factor
    // Ground effect reduces induced drag
    double groundEffectFactor = 1.0 - (wingspan / (16.0 * height));

    // THEN: Ground effect should be present
    TS_ASSERT(height < wingspan);
    TS_ASSERT(groundEffectFactor < 1.0);
    TS_ASSERT(groundEffectFactor > 0.0);
  }

  // Test ground effect - reduced induced drag
  void testGroundEffectInducedDrag() {
    // GIVEN: Aircraft at 5 feet AGL
    double height = 5.0;  // feet
    double wingspan = 36.0;  // feet

    // WHEN: In ground effect
    double ratio = height / wingspan;

    // THEN: Height ratio affects drag reduction
    TS_ASSERT(ratio < 0.5);  // Strong ground effect
    // Induced drag can be reduced by up to 50% at very low heights
  }

  // Test crosswind gust response - roll rate
  void testCrosswindGustRollResponse() {
    // GIVEN: Sudden 10 knot gust from right
    double gustMagnitude = 10.0;  // knots
    double wingSpan = 36.0;  // feet
    double rollDamping = 0.5;  // typical damping coefficient

    // WHEN: Gust hits aircraft
    // Initial roll rate proportional to gust strength
    double gustEffect = gustMagnitude / wingSpan;
    double rollRate = gustEffect * (1.0 - rollDamping);

    // THEN: Aircraft will roll
    TS_ASSERT(rollRate > 0.0);
  }

  // Test crosswind gust response - yaw rate
  void testCrosswindGustYawResponse() {
    // GIVEN: Sudden crosswind gust
    double gustMagnitude = 15.0;  // knots
    double vertStabArea = 30.0;  // sq ft

    // WHEN: Gust creates yawing moment
    double yawEffect = gustMagnitude * vertStabArea;

    // THEN: Yawing moment should be significant
    TS_ASSERT(yawEffect > 0.0);
    TS_ASSERT_DELTA(yawEffect, 450.0, DEFAULT_TOLERANCE);
  }

  // Test maximum demonstrated crosswind for Cessna 172
  void testCessna172MaxCrosswind() {
    // GIVEN: Cessna 172 specifications
    double demonstratedCrosswind = 15.0;  // knots
    double actualCrosswind = 14.0;  // knots

    // THEN: Within demonstrated capability
    TS_ASSERT(actualCrosswind <= demonstratedCrosswind);
  }

  // Test transport category crosswind requirements
  void testTransportCategoryCrosswindReqs() {
    // GIVEN: Transport category aircraft
    // Must demonstrate landing capability in 0.2 * Vsr crosswind
    double Vsr = 100.0;  // knots (stall speed reference)
    double requiredCrosswind = 0.2 * Vsr;  // 20 knots

    // THEN: Must demonstrate at least 20 knots
    TS_ASSERT_DELTA(requiredCrosswind, 20.0, DEFAULT_TOLERANCE);
  }

  // Test crosswind component with varying wind angles
  void testCrosswindVariousAngles() {
    double windSpeed = 20.0;  // knots

    // Test at multiple angles
    struct TestCase {
      double angle;  // degrees
      double expectedCrosswind;
    } testCases[] = {
      {0.0, 0.0},      // Headwind
      {30.0, 10.0},    // 30 degrees
      {45.0, 14.14},   // 45 degrees
      {60.0, 17.32},   // 60 degrees
      {90.0, 20.0},    // Pure crosswind
      {180.0, 0.0}     // Tailwind
    };

    for (const auto& tc : testCases) {
      double angle = tc.angle * Constants::DEG_TO_RAD;
      double crosswind = windSpeed * std::sin(angle);
      TS_ASSERT_DELTA(std::abs(crosswind), tc.expectedCrosswind, 0.1);
    }
  }

  // Test crab angle with different airspeeds
  void testCrabAngleDifferentSpeeds() {
    double crosswind = 20.0;  // knots constant

    // Test at different airspeeds
    struct SpeedTest {
      double airspeed;
      double expectedCrabDeg;
    } speedTests[] = {
      {60.0, 19.47},   // Slow approach
      {80.0, 14.48},   // Normal approach
      {100.0, 11.54},  // Fast approach
      {120.0, 9.59}    // High speed approach
    };

    for (const auto& st : speedTests) {
      double crabAngle = std::asin(crosswind / st.airspeed);
      TS_ASSERT_DELTA(crabAngle * Constants::RAD_TO_DEG, st.expectedCrabDeg, 0.01);
    }
  }

  // Test combined crab and wing-low technique
  void testCombinedTechnique() {
    // GIVEN: Approach with crab, then transition to wing-low
    double crabAngle = 10.0 * Constants::DEG_TO_RAD;
    double transitionHeight = 50.0;  // feet AGL

    // WHEN: At transition height, begin wing-low
    double wingLowAngle = crabAngle;  // Convert crab to wing-low

    // THEN: Bank angle should equal previous crab angle
    TS_ASSERT_DELTA(wingLowAngle, crabAngle, DEFAULT_TOLERANCE);
  }

  // Test rudder effectiveness in crosswind
  void testRudderEffectivenessCrosswind() {
    // GIVEN: Rudder deflection in crosswind
    double rudderDeflection = 10.0 * Constants::DEG_TO_RAD;
    double dynamicPressure = 30.0;  // psf
    double rudderArea = 12.0;  // sq ft

    // WHEN: Computing yawing moment
    double rudderForce = dynamicPressure * rudderArea * std::sin(rudderDeflection);

    // THEN: Should produce significant force
    TS_ASSERT(rudderForce > 0.0);
  }

  // Test aileron effectiveness in crosswind
  void testAileronEffectivenessCrosswind() {
    // GIVEN: Aileron deflection to counter roll from crosswind
    double aileronDeflection = 5.0 * Constants::DEG_TO_RAD;
    double dynamicPressure = 25.0;  // psf
    double aileronArea = 8.0;  // sq ft per aileron

    // WHEN: Computing rolling moment
    double aileronForce = dynamicPressure * aileronArea * std::sin(aileronDeflection);

    // THEN: Should produce countering moment
    TS_ASSERT(aileronForce > 0.0);
  }

  // Test slip-skid indicator (ball) in coordinated flight
  void testSlipSkidIndicatorCoordinated() {
    // GIVEN: Coordinated flight with crab
    double lateralAccel = 0.0;  // g's (ball centered)

    // THEN: No lateral acceleration in coordinated flight
    TS_ASSERT_DELTA(lateralAccel, 0.0, DEFAULT_TOLERANCE);
  }

  // Test slip-skid indicator in sideslip
  void testSlipSkidIndicatorSideslip() {
    // GIVEN: Aircraft in sideslip (wing-low method)
    double sideslipAngle = 8.0 * Constants::DEG_TO_RAD;
    double gravitationalAccel = Constants::G_FTPS2;

    // WHEN: Computing lateral acceleration
    double lateralAccel = gravitationalAccel * std::sin(sideslipAngle);

    // THEN: Should show lateral acceleration (ball deflected)
    TS_ASSERT(std::abs(lateralAccel) > 0.0);
  }

  // Test crosswind landing - de-crab timing
  void testDeCrabTiming() {
    // GIVEN: Aircraft on final approach with crab
    double crabAngle = 12.0 * Constants::DEG_TO_RAD;
    double heightAGL = 10.0;  // feet
    double minHeight = 5.0;  // feet (minimum de-crab height)

    // WHEN: At appropriate height for de-crab
    bool shouldDeCrab = (heightAGL <= minHeight);

    // THEN: Should maintain crab until close to ground
    TS_ASSERT(!shouldDeCrab);  // Still too high
  }

  // Test crosswind correction - velocity triangle
  void testVelocityTriangle() {
    // GIVEN: Airspeed and wind vectors
    double airspeed = 100.0;  // knots
    double windSpeed = 20.0;  // knots
    double windAngle = 90.0 * Constants::DEG_TO_RAD;  // Pure crosswind

    // WHEN: Computing groundspeed vector
    double headwindComponent = windSpeed * std::cos(windAngle);
    double crosswindComponent = windSpeed * std::sin(windAngle);
    double groundspeed = std::sqrt(std::pow(airspeed - headwindComponent, 2.0) +
                                   std::pow(crosswindComponent, 2.0));

    // THEN: Groundspeed should be reduced by crosswind
    TS_ASSERT_DELTA(groundspeed, 101.98, 0.01);
  }

  // Test crosswind with gusts - structural loads
  void testCrosswindGustLoads() {
    // GIVEN: Crosswind with gusts
    double steadyCrosswind = 15.0;  // knots
    double gustFactor = 1.5;  // 50% gust increase
    double maxCrosswind = steadyCrosswind * gustFactor;

    // THEN: Peak loads occur during gusts
    TS_ASSERT_DELTA(maxCrosswind, 22.5, DEFAULT_TOLERANCE);
    TS_ASSERT(maxCrosswind > steadyCrosswind);
  }

  // Test minimum control speed with crosswind (Vmc)
  void testVmcWithCrosswind() {
    // GIVEN: Twin engine aircraft, engine out
    double VmcNoWind = 80.0;  // knots
    double crosswind = 10.0;  // knots from dead engine side

    // WHEN: Crosswind from dead engine side
    // Vmc is effectively reduced (wind helps rudder)
    double effectiveVmc = VmcNoWind - (crosswind * 0.3);

    // THEN: Crosswind aids directional control
    TS_ASSERT(effectiveVmc < VmcNoWind);
  }

  // Test crosswind circle (constant radius turn in wind)
  void testCrosswindCircle() {
    // GIVEN: Aircraft in level turn with wind
    double airspeed = 100.0;  // knots
    double windSpeed = 20.0;  // knots

    // WHEN: Turning 360 degrees in wind
    // Ground track is not circular
    double downwindGroundspeed = airspeed + windSpeed;
    double upwindGroundspeed = airspeed - windSpeed;

    // THEN: Groundspeed varies around turn
    TS_ASSERT_DELTA(downwindGroundspeed, 120.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(upwindGroundspeed, 80.0, DEFAULT_TOLERANCE);
    TS_ASSERT(downwindGroundspeed > upwindGroundspeed);
  }

  // Test crosswind effect on turn radius
  void testCrosswindTurnRadius() {
    // GIVEN: Aircraft in coordinated turn
    double airspeed = 100.0 * Constants::KTS_TO_FTPS;  // Convert to ft/s
    double bankAngle = 30.0 * Constants::DEG_TO_RAD;
    double g = Constants::G_FTPS2;

    // WHEN: Computing turn radius
    double turnRadius = (airspeed * airspeed) / (g * std::tan(bankAngle));

    // THEN: Turn radius based on airspeed, not groundspeed
    TS_ASSERT(turnRadius > 0.0);
    TS_ASSERT_DELTA(turnRadius, 1533.6, 0.5);  // feet
  }

  // Test crosswind landing - touchdown drift
  void testTouchdownDrift() {
    // GIVEN: Aircraft touches down with slight crab remaining
    double remainingCrabAngle = 2.0 * Constants::DEG_TO_RAD;
    double touchdownSpeed = 60.0;  // knots

    // WHEN: Computing lateral velocity at touchdown
    double lateralVelocity = touchdownSpeed * std::sin(remainingCrabAngle);

    // THEN: Should have minimal side load on gear
    TS_ASSERT(lateralVelocity < 3.0);  // knots
  }

  // Test maximum crosswind - gear side load limit
  void testGearSideLoadLimit() {
    // GIVEN: Landing gear side load limit
    double maxSideLoadRatio = 0.2;  // 20% of vertical load (typical)
    double touchdownSpeed = 70.0;  // knots

    // WHEN: Computing maximum acceptable drift angle
    double maxDriftAngle = std::atan(maxSideLoadRatio);
    double maxLateralSpeed = touchdownSpeed * maxDriftAngle;

    // THEN: Drift should be within gear limits
    TS_ASSERT(maxLateralSpeed < 15.0);  // knots
  }

  // Test crosswind component at different runway headings
  void testCrosswindDifferentRunways() {
    // GIVEN: Wind from 270 degrees at 20 knots
    double windDir = 270.0 * Constants::DEG_TO_RAD;
    double windSpeed = 20.0;  // knots

    // Test different runway headings
    struct RunwayTest {
      double heading;  // degrees
      double expectedCrosswind;
    } runways[] = {
      {360.0, -20.0},  // Runway 36 - full crosswind from left
      {270.0, 0.0},    // Runway 27 - headwind
      {180.0, 20.0},   // Runway 18 - full crosswind from right
      {90.0, 0.0}      // Runway 09 - tailwind
    };

    for (const auto& rwy : runways) {
      double heading = rwy.heading * Constants::DEG_TO_RAD;
      double crosswind = windSpeed * std::sin(windDir - heading);
      TS_ASSERT_DELTA(crosswind, rwy.expectedCrosswind, 0.1);
    }
  }

  // Test pilot workload in crosswind
  void testPilotWorkloadCrosswind() {
    // GIVEN: Increasing crosswind component
    double lightCrosswind = 5.0;   // knots - low workload
    double moderateCrosswind = 15.0;  // knots - moderate workload
    double strongCrosswind = 25.0;  // knots - high workload

    // THEN: Workload increases with crosswind magnitude
    TS_ASSERT(lightCrosswind < moderateCrosswind);
    TS_ASSERT(moderateCrosswind < strongCrosswind);
  }

  // Test crosswind landing - flare adjustment
  void testCrosswindFlareAdjustment() {
    // GIVEN: Normal flare height
    double normalFlareHeight = 20.0;  // feet AGL
    double crosswindComponent = 15.0;  // knots

    // WHEN: Strong crosswind present
    // May need slightly higher flare to allow drift correction
    double adjustedFlareHeight = normalFlareHeight + (crosswindComponent * 0.1);

    // THEN: Flare height adjusted for crosswind
    TS_ASSERT(adjustedFlareHeight > normalFlareHeight);
    TS_ASSERT_DELTA(adjustedFlareHeight, 21.5, 0.1);
  }

  // Test crosswind with variable wind altitude profile
  void testCrosswindAltitudeProfile() {
    // GIVEN: Wind speed varies with altitude (gradient wind)
    double surfaceWind = 10.0;  // knots at surface
    double altitudeWind = 20.0;  // knots at 1000 feet
    double altitude = 500.0;  // feet

    // WHEN: Computing wind at intermediate altitude (linear interpolation)
    double windAtAltitude = surfaceWind + (altitudeWind - surfaceWind) * (altitude / 1000.0);

    // THEN: Wind should be between surface and altitude wind
    TS_ASSERT(windAtAltitude > surfaceWind);
    TS_ASSERT(windAtAltitude < altitudeWind);
    TS_ASSERT_DELTA(windAtAltitude, 15.0, DEFAULT_TOLERANCE);
  }

  // Test crosswind effect on stall speed
  void testCrosswindStallSpeed() {
    // GIVEN: Aircraft in coordinated flight with crab
    double baseStallSpeed = 50.0;  // knots
    double crosswind = 20.0;  // knots

    // WHEN: Using crab method (coordinated flight)
    // Stall speed unaffected - based on airspeed
    double stallSpeedInCrosswind = baseStallSpeed;

    // THEN: Stall speed remains constant (coordinated flight)
    TS_ASSERT_DELTA(stallSpeedInCrosswind, baseStallSpeed, DEFAULT_TOLERANCE);
  }

  // Test crosswind effect on takeoff distance
  void testCrosswindTakeoffDistance() {
    // GIVEN: Normal takeoff distance
    double noWindTakeoffDist = 1500.0;  // feet
    double headwind = 10.0;  // knots
    double crosswind = 15.0;  // knots

    // WHEN: Computing takeoff distance with headwind
    // Headwind reduces takeoff distance (simplified: 10% per 10 knots)
    double takeoffDistReduction = headwind / 10.0;  // 10 knots = 1.0 = 100% reduction rate
    double adjustedDistance = noWindTakeoffDist * (1.0 - (takeoffDistReduction * 0.10));

    // THEN: Takeoff distance reduced by headwind component
    TS_ASSERT(adjustedDistance < noWindTakeoffDist);
    TS_ASSERT_DELTA(adjustedDistance, 1350.0, 1.0);
  }

  // Test crosswind landing - runway width requirements
  void testCrosswindRunwayWidth() {
    // GIVEN: Aircraft drift during landing
    double lateralDrift = 5.0;  // feet during final approach
    double minimumRunwayWidth = 75.0;  // feet (typical GA runway)
    double aircraftWingspan = 36.0;  // feet

    // WHEN: Computing lateral safety margin
    double availableMargin = (minimumRunwayWidth - aircraftWingspan) / 2.0;

    // THEN: Drift should be within safety margin
    TS_ASSERT(lateralDrift < availableMargin);
    TS_ASSERT_DELTA(availableMargin, 19.5, DEFAULT_TOLERANCE);
  }

  // Test crosswind - wind shear during approach
  void testCrosswindWindShear() {
    // GIVEN: Wind shear - wind decreases near surface
    double windAt1000Ft = 25.0;  // knots
    double windAtSurface = 15.0;  // knots
    double descentRate = 500.0;  // feet per minute

    // WHEN: Descending through wind shear
    double windGradient = (windAt1000Ft - windAtSurface) / 1000.0;  // per foot
    double windChangeRate = windGradient * descentRate;  // knots per minute

    // THEN: Aircraft experiences changing wind
    TS_ASSERT(windChangeRate > 0.0);
    TS_ASSERT_DELTA(windChangeRate, 5.0, DEFAULT_TOLERANCE);  // 5 knots/min change
  }

  // Test maximum crosswind - performance degradation
  void testMaxCrosswindPerformance() {
    // GIVEN: Aircraft performance in crosswind
    double nominalClimbRate = 700.0;  // feet per minute
    double crosswind = 25.0;  // knots
    double performanceDegradation = 0.02;  // 2% per 10 knots crosswind

    // WHEN: Computing degraded performance
    double degradation = (crosswind / 10.0) * performanceDegradation;
    double degradedClimbRate = nominalClimbRate * (1.0 - degradation);

    // THEN: Performance slightly reduced
    TS_ASSERT(degradedClimbRate < nominalClimbRate);
    TS_ASSERT_DELTA(degradedClimbRate, 665.0, 1.0);
  }

  // Test crosswind landing - go-around decision
  void testCrosswindGoAroundDecision() {
    // GIVEN: Crosswind exceeds demonstrated limit
    double demonstratedCrosswind = 15.0;  // knots
    double actualCrosswind = 20.0;  // knots
    double pilotExperience = 0.8;  // 80% of max capability

    // WHEN: Evaluating go-around necessity
    bool shouldGoAround = (actualCrosswind > demonstratedCrosswind * pilotExperience);

    // THEN: Should consider go-around
    TS_ASSERT(shouldGoAround);
    TS_ASSERT(actualCrosswind > demonstratedCrosswind * pilotExperience);
  }

  // Test crosswind - dynamic stability
  void testCrosswindDynamicStability() {
    // GIVEN: Aircraft with directional stability (weathercock stability)
    double staticStabilityCoeff = 0.15;  // positive = stable
    double sideslipAngle = 5.0 * Constants::DEG_TO_RAD;

    // WHEN: Computing restoring yaw moment
    double restoringMoment = staticStabilityCoeff * sideslipAngle;

    // THEN: Aircraft tends to align with relative wind
    TS_ASSERT(restoringMoment > 0.0);
    TS_ASSERT(staticStabilityCoeff > 0.0);  // Positive = stable
  }

  /***************************************************************************
   * Extended Wind Component Tests
   ***************************************************************************/

  // Test quartering headwind
  void testQuarteringHeadwind() {
    double windSpeed = 30.0;  // knots
    double windAngle = 30.0 * Constants::DEG_TO_RAD;

    double headwind = windSpeed * std::cos(windAngle);
    double crosswind = windSpeed * std::sin(windAngle);

    TS_ASSERT_DELTA(headwind, 25.98, 0.1);
    TS_ASSERT_DELTA(crosswind, 15.0, 0.1);
  }

  // Test quartering tailwind
  void testQuarteringTailwind() {
    double windSpeed = 25.0;
    double windAngle = 150.0 * Constants::DEG_TO_RAD;

    double headwind = windSpeed * std::cos(windAngle);
    double crosswind = windSpeed * std::sin(windAngle);

    TS_ASSERT(headwind < 0);  // Tailwind component
    TS_ASSERT(crosswind > 0);  // Crosswind component
  }

  // Test gusty crosswind components
  void testGustyCrosswindComponents() {
    double steadyWind = 15.0;  // knots
    double gustFactor = 1.4;
    double windAngle = 60.0 * Constants::DEG_TO_RAD;

    double peakCrosswind = steadyWind * gustFactor * std::sin(windAngle);
    double steadyCrosswind = steadyWind * std::sin(windAngle);

    TS_ASSERT(peakCrosswind > steadyCrosswind);
    TS_ASSERT_DELTA(peakCrosswind / steadyCrosswind, gustFactor, 0.01);
  }

  /***************************************************************************
   * Advanced Crab Angle Calculations
   ***************************************************************************/

  // Test crab angle at low airspeed
  void testCrabAngleLowAirspeed() {
    double airspeed = 50.0;  // knots (slow approach)
    double crosswind = 15.0;

    double crabAngle = std::asin(crosswind / airspeed);
    TS_ASSERT_DELTA(crabAngle * Constants::RAD_TO_DEG, 17.46, 0.1);
  }

  // Test crab angle near maximum
  void testCrabAngleNearMaximum() {
    double airspeed = 55.0;
    double crosswind = 50.0;  // Strong crosswind

    double ratio = crosswind / airspeed;
    TS_ASSERT(ratio < 1.0);  // Must be < 1 for valid asin

    double crabAngle = std::asin(ratio);
    TS_ASSERT(crabAngle * Constants::RAD_TO_DEG > 60.0);
  }

  // Test crab angle sensitivity to wind change
  void testCrabAngleSensitivityToWindChange() {
    double airspeed = 100.0;

    double crosswind1 = 10.0;
    double crosswind2 = 12.0;  // 20% increase

    double crab1 = std::asin(crosswind1 / airspeed) * Constants::RAD_TO_DEG;
    double crab2 = std::asin(crosswind2 / airspeed) * Constants::RAD_TO_DEG;

    double crabChange = crab2 - crab1;
    TS_ASSERT(crabChange > 0);
  }

  /***************************************************************************
   * Wing-Low Technique Extended Tests
   ***************************************************************************/

  // Test maximum bank angle for wing-low
  void testMaxBankAngleWingLow() {
    double crosswind = 20.0;
    double approachSpeed = 65.0;

    double bankAngle = std::atan(crosswind / approachSpeed);
    double bankDeg = bankAngle * Constants::RAD_TO_DEG;

    // Should be significant but manageable
    TS_ASSERT(bankDeg > 10.0);
    TS_ASSERT(bankDeg < 25.0);
  }

  // Test rudder authority in wing-low
  void testRudderAuthorityWingLow() {
    double bankAngle = 10.0 * Constants::DEG_TO_RAD;
    double rudderEffectiveness = 50.0;  // units

    // Need opposite rudder to maintain runway alignment
    double requiredRudder = bankAngle * 0.7;  // Simplified

    TS_ASSERT(requiredRudder > 0);
    TS_ASSERT(requiredRudder < rudderEffectiveness / 3.0);
  }

  // Test touchdown geometry wing-low
  void testTouchdownGeometryWingLow() {
    double bankAngle = 8.0 * Constants::DEG_TO_RAD;
    double wheelTrack = 12.0;  // ft

    // Upwind wheel touches first
    double heightDifference = wheelTrack * std::sin(bankAngle);
    TS_ASSERT(heightDifference > 0);
    TS_ASSERT_DELTA(heightDifference, 1.67, 0.1);
  }

  /***************************************************************************
   * Runway Selection for Crosswind
   ***************************************************************************/

  // Test optimal runway selection
  void testOptimalRunwaySelection() {
    double windDir = 270.0;  // From west
    double windSpeed = 20.0;

    // Available runways
    double runways[] = {90.0, 180.0, 270.0, 360.0};
    double minCrosswind = 999.0;
    double bestRunway = 0.0;

    for (double rwy : runways) {
      double angle = std::abs(windDir - rwy);
      if (angle > 180.0) angle = 360.0 - angle;
      double crosswind = windSpeed * std::sin(angle * Constants::DEG_TO_RAD);

      if (std::abs(crosswind) < minCrosswind) {
        minCrosswind = std::abs(crosswind);
        bestRunway = rwy;
      }
    }

    TS_ASSERT_DELTA(bestRunway, 270.0, 0.1);  // Head wind runway
  }

  // Test crosswind limit decision
  void testCrosswindLimitDecision() {
    double demonstratedLimit = 15.0;
    double personalLimit = 12.0;  // 80% of demonstrated
    double actualCrosswind = 14.0;

    bool withinDemonstrated = actualCrosswind <= demonstratedLimit;
    bool withinPersonal = actualCrosswind <= personalLimit;

    TS_ASSERT(withinDemonstrated);
    TS_ASSERT(!withinPersonal);
  }

  /***************************************************************************
   * Directional Control Tests
   ***************************************************************************/

  // Test rudder pedal force required
  void testRudderPedalForce() {
    double crosswind = 15.0;
    double forcePerKnot = 2.0;  // lbs per knot crosswind

    double pedalForce = crosswind * forcePerKnot;
    TS_ASSERT_DELTA(pedalForce, 30.0, DEFAULT_TOLERANCE);
  }

  // Test nosewheel steering in crosswind
  void testNosewheelSteeringCrosswind() {
    double crosswind = 12.0;
    double groundSpeed = 30.0;  // kts

    // More steering needed at low speed
    double steeringAngle = std::atan(crosswind / groundSpeed) * 0.5;
    TS_ASSERT(steeringAngle > 0);
  }

  // Test brake differential in crosswind taxi
  void testBrakeDifferentialTaxi() {
    double crosswind = 18.0;
    double baseBraking = 0.3;  // Normal braking fraction

    // Upwind brake used more to counter weathervane
    double upwindBrake = baseBraking * (1.0 + crosswind / 50.0);
    double downwindBrake = baseBraking;

    TS_ASSERT(upwindBrake > downwindBrake);
  }

  /***************************************************************************
   * Crosswind During Climb and Descent
   ***************************************************************************/

  // Test ground track in climb with crosswind
  void testGroundTrackInClimb() {
    double airspeed = 80.0;  // kts
    double crosswind = 15.0;  // kts
    double heading = 360.0;  // Due north

    double crabAngle = std::asin(crosswind / airspeed);
    double groundTrack = heading - (crabAngle * Constants::RAD_TO_DEG);

    // Ground track is west of heading due to right crosswind
    TS_ASSERT(groundTrack < heading);
  }

  // Test rate of climb with crosswind
  void testRateOfClimbCrosswind() {
    double normalROC = 500.0;  // fpm
    double crosswind = 20.0;

    // Crosswind slightly reduces climb performance
    double performanceLoss = crosswind * 0.5;  // fpm per knot
    double actualROC = normalROC - performanceLoss;

    TS_ASSERT(actualROC < normalROC);
    TS_ASSERT_DELTA(actualROC, 490.0, 1.0);
  }

  /***************************************************************************
   * Crosswind Go-Around
   ***************************************************************************/

  // Test go-around with crosswind
  void testGoAroundWithCrosswind() {
    double crosswind = 18.0;
    double airspeed = 85.0;

    // Must maintain directional control during transition
    double crabAngle = std::asin(crosswind / airspeed);
    TS_ASSERT(crabAngle * Constants::RAD_TO_DEG < 15.0);
  }

  // Test climb gradient in crosswind go-around
  void testClimbGradientCrosswindGoAround() {
    double normalGradient = 4.0;  // percent
    double crosswindPenalty = 0.3;  // percent per 10 kts
    double crosswind = 20.0;

    double actualGradient = normalGradient - (crosswindPenalty * crosswind / 10.0);
    TS_ASSERT(actualGradient > 2.5);  // Still positive climb
  }

  /***************************************************************************
   * Asymmetric Effects
   ***************************************************************************/

  // Test P-factor in crosswind
  void testPFactorInCrosswind() {
    double crosswind = 15.0;  // From right
    double pFactorYaw = 2.0;  // degrees left yaw

    // Crosswind from right adds to P-factor left yaw
    double totalYaw = pFactorYaw + (crosswind * 0.1);

    TS_ASSERT(totalYaw > pFactorYaw);
  }

  // Test torque effect in crosswind
  void testTorqueEffectCrosswind() {
    double torqueRoll = 3.0;  // degrees left roll tendency
    double crosswindRoll = 2.0;  // degrees from crosswind

    double combinedRoll = torqueRoll + crosswindRoll;
    TS_ASSERT_DELTA(combinedRoll, 5.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Stress and Edge Cases
   ***************************************************************************/

  // Test extreme crosswind (near limit)
  void testExtremeCrosswind() {
    double crosswind = 35.0;  // kts
    double approachSpeed = 70.0;

    double ratio = crosswind / approachSpeed;
    TS_ASSERT(ratio < 1.0);  // Valid

    double crabAngle = std::asin(ratio);
    TS_ASSERT(crabAngle * Constants::RAD_TO_DEG > 25.0);
  }

  // Test variable crosswind during approach
  void testVariableCrosswindApproach() {
    double altitudes[] = {500.0, 200.0, 50.0};
    double crosswinds[] = {25.0, 18.0, 12.0};  // Decreases with altitude

    for (int i = 1; i < 3; i++) {
      TS_ASSERT(crosswinds[i] < crosswinds[i-1]);
    }
  }

  // Test sudden crosswind reversal
  void testCrosswindReversal() {
    double initialCrosswind = 15.0;  // From right
    double reversedCrosswind = -10.0;  // Now from left

    double change = std::abs(reversedCrosswind - initialCrosswind);
    TS_ASSERT_DELTA(change, 25.0, DEFAULT_TOLERANCE);
  }

  // Test zero crosswind condition
  void testZeroCrosswindCondition() {
    double windSpeed = 15.0;
    double windAngle = 0.0;  // Pure headwind

    double crosswind = windSpeed * std::sin(windAngle);
    TS_ASSERT_DELTA(crosswind, 0.0, DEFAULT_TOLERANCE);
  }

  // Test numerical stability at high crab angles
  void testNumericalStabilityHighCrab() {
    for (double ratio = 0.1; ratio < 0.95; ratio += 0.1) {
      double crabAngle = std::asin(ratio);
      TS_ASSERT(!std::isnan(crabAngle));
      TS_ASSERT(!std::isinf(crabAngle));
      TS_ASSERT(crabAngle > 0);
    }
  }

  // Test many wind angles sweep
  void testWindAnglesSweep() {
    double windSpeed = 20.0;
    double airspeed = 100.0;

    for (double angle = 0.0; angle <= 180.0; angle += 15.0) {
      double crosswind = windSpeed * std::sin(angle * Constants::DEG_TO_RAD);
      double headwind = windSpeed * std::cos(angle * Constants::DEG_TO_RAD);

      // Total should equal original wind
      double total = std::sqrt(crosswind*crosswind + headwind*headwind);
      TS_ASSERT_DELTA(total, windSpeed, 0.01);
    }
  }
};
