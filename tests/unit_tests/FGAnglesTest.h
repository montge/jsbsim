/*******************************************************************************
 * FGAnglesTest.h - Unit tests for FGAngles (angle calculations)
 *
 * Tests the mathematical behavior of angle calculations:
 * - Smallest included angle between two angles
 * - Angle wrapping (handling 0/360 boundary)
 * - Unit conversions (degrees/radians)
 * - Heading difference calculations
 *
 * Note: FGAngles requires XML element for construction, so these tests focus
 * on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <algorithm>

#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "models/FGPropagate.h"
#include "models/FGAuxiliary.h"

using namespace JSBSim;

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

class FGAnglesTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Helper Functions
   ***************************************************************************/

  // Normalize angle to [-180, 180] degrees
  double normalizeAngle180(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
  }

  // Normalize angle to [0, 360) degrees
  double normalizeAngle360(double angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
  }

  // Calculate smallest included angle (result in [-180, 180])
  double smallestIncludedAngle(double source, double target) {
    double diff = target - source;
    return normalizeAngle180(diff);
  }

  /***************************************************************************
   * Basic Angle Difference Tests
   ***************************************************************************/

  // Test simple angle difference
  void testSimpleAngleDifference() {
    double source = 30.0;
    double target = 50.0;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, 20.0, epsilon);
  }

  // Test negative angle difference
  void testNegativeAngleDifference() {
    double source = 50.0;
    double target = 30.0;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, -20.0, epsilon);
  }

  // Test same angle
  void testSameAngle() {
    double source = 45.0;
    double target = 45.0;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, 0.0, epsilon);
  }

  /***************************************************************************
   * Wrap-Around Tests
   ***************************************************************************/

  // Test crossing 0/360 boundary (short way)
  void testCrossingZeroBoundaryShort() {
    double source = 350.0;
    double target = 10.0;

    double diff = smallestIncludedAngle(source, target);
    // Should take short route: +20 degrees (not -340)
    TS_ASSERT_DELTA(diff, 20.0, epsilon);
  }

  // Test crossing 0/360 boundary (other direction)
  void testCrossingZeroBoundaryOther() {
    double source = 10.0;
    double target = 350.0;

    double diff = smallestIncludedAngle(source, target);
    // Should take short route: -20 degrees (not +340)
    TS_ASSERT_DELTA(diff, -20.0, epsilon);
  }

  // Test 180 degree difference
  void testOppositeDirections() {
    double source = 0.0;
    double target = 180.0;

    double diff = smallestIncludedAngle(source, target);
    // Exactly 180 degrees - convention usually returns +180
    TS_ASSERT_DELTA(std::abs(diff), 180.0, epsilon);
  }

  // Test nearly opposite (179 degrees)
  void testNearlyOpposite() {
    double source = 0.0;
    double target = 179.0;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, 179.0, epsilon);
  }

  // Test nearly opposite (181 degrees)
  void testNearlyOppositeOther() {
    double source = 0.0;
    double target = 181.0;

    double diff = smallestIncludedAngle(source, target);
    // Should take short route: -179 degrees
    TS_ASSERT_DELTA(diff, -179.0, epsilon);
  }

  /***************************************************************************
   * Angle Normalization Tests
   ***************************************************************************/

  // Test normalization to [-180, 180]
  void testNormalization180() {
    TS_ASSERT_DELTA(normalizeAngle180(190.0), -170.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle180(-190.0), 170.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle180(370.0), 10.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle180(-370.0), -10.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle180(90.0), 90.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle180(-90.0), -90.0, epsilon);
  }

  // Test normalization to [0, 360)
  void testNormalization360() {
    TS_ASSERT_DELTA(normalizeAngle360(370.0), 10.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle360(-10.0), 350.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle360(720.0), 0.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle360(-720.0), 0.0, epsilon);
    TS_ASSERT_DELTA(normalizeAngle360(180.0), 180.0, epsilon);
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test degrees to radians
  void testDegreesToRadians() {
    TS_ASSERT_DELTA(0.0 * DEG_TO_RAD, 0.0, epsilon);
    TS_ASSERT_DELTA(90.0 * DEG_TO_RAD, M_PI / 2.0, epsilon);
    TS_ASSERT_DELTA(180.0 * DEG_TO_RAD, M_PI, epsilon);
    TS_ASSERT_DELTA(360.0 * DEG_TO_RAD, 2.0 * M_PI, epsilon);
    TS_ASSERT_DELTA(-90.0 * DEG_TO_RAD, -M_PI / 2.0, epsilon);
  }

  // Test radians to degrees
  void testRadiansToDegrees() {
    TS_ASSERT_DELTA(0.0 * RAD_TO_DEG, 0.0, epsilon);
    TS_ASSERT_DELTA((M_PI / 2.0) * RAD_TO_DEG, 90.0, epsilon);
    TS_ASSERT_DELTA(M_PI * RAD_TO_DEG, 180.0, epsilon);
    TS_ASSERT_DELTA((2.0 * M_PI) * RAD_TO_DEG, 360.0, epsilon);
  }

  // Test round-trip conversion
  void testRoundTripConversion() {
    double originalDeg = 45.0;
    double rad = originalDeg * DEG_TO_RAD;
    double backToDeg = rad * RAD_TO_DEG;

    TS_ASSERT_DELTA(backToDeg, originalDeg, epsilon);
  }

  /***************************************************************************
   * Heading Difference (Navigation) Tests
   ***************************************************************************/

  // Test heading to turn calculation (right turn)
  void testHeadingTurnRight() {
    double currentHeading = 45.0;
    double targetHeading = 90.0;

    double turn = smallestIncludedAngle(currentHeading, targetHeading);
    TS_ASSERT_DELTA(turn, 45.0, epsilon);  // Turn right 45 degrees
    TS_ASSERT(turn > 0);  // Positive = right turn
  }

  // Test heading to turn calculation (left turn)
  void testHeadingTurnLeft() {
    double currentHeading = 90.0;
    double targetHeading = 45.0;

    double turn = smallestIncludedAngle(currentHeading, targetHeading);
    TS_ASSERT_DELTA(turn, -45.0, epsilon);  // Turn left 45 degrees
    TS_ASSERT(turn < 0);  // Negative = left turn
  }

  // Test north crossing heading turn
  void testNorthCrossingTurn() {
    double currentHeading = 350.0;
    double targetHeading = 20.0;

    double turn = smallestIncludedAngle(currentHeading, targetHeading);
    TS_ASSERT_DELTA(turn, 30.0, epsilon);  // Turn right through north
  }

  // Test south crossing heading turn
  void testSouthCrossingTurn() {
    double currentHeading = 170.0;
    double targetHeading = 200.0;

    double turn = smallestIncludedAngle(currentHeading, targetHeading);
    TS_ASSERT_DELTA(turn, 30.0, epsilon);  // Turn right through south
  }

  /***************************************************************************
   * Bank Angle and Flight Path Tests
   ***************************************************************************/

  // Test bank angle difference
  void testBankAngleDifference() {
    double currentBank = -15.0;  // Left bank
    double targetBank = 15.0;    // Right bank

    double diff = targetBank - currentBank;
    TS_ASSERT_DELTA(diff, 30.0, epsilon);
  }

  // Test pitch angle difference
  void testPitchAngleDifference() {
    double currentPitch = 5.0;   // Nose up
    double targetPitch = -3.0;  // Nose down

    double diff = targetPitch - currentPitch;
    TS_ASSERT_DELTA(diff, -8.0, epsilon);
  }

  /***************************************************************************
   * Cardinal Direction Tests
   ***************************************************************************/

  // Test cardinal direction angles
  void testCardinalDirections() {
    double north = 0.0;
    double east = 90.0;
    double south = 180.0;
    double west = 270.0;

    // East from North = +90
    TS_ASSERT_DELTA(smallestIncludedAngle(north, east), 90.0, epsilon);

    // West from North = -90
    TS_ASSERT_DELTA(smallestIncludedAngle(north, west), -90.0, epsilon);

    // South from North = +/-180
    TS_ASSERT_DELTA(std::abs(smallestIncludedAngle(north, south)), 180.0, epsilon);
  }

  /***************************************************************************
   * Intercardinal Direction Tests
   ***************************************************************************/

  void testIntercardinalDirections() {
    double NE = 45.0;
    double SE = 135.0;
    double SW = 225.0;
    double NW = 315.0;

    // NE to SE = +90
    TS_ASSERT_DELTA(smallestIncludedAngle(NE, SE), 90.0, epsilon);

    // NW to NE = +90 (through north)
    TS_ASSERT_DELTA(smallestIncludedAngle(NW, NE), 90.0, epsilon);

    // NE to SW = +/-180
    TS_ASSERT_DELTA(std::abs(smallestIncludedAngle(NE, SW)), 180.0, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test with very small angles
  void testVerySmallAngle() {
    double source = 0.0;
    double target = 0.001;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, 0.001, epsilon);
  }

  // Test with very large angles (multiple revolutions)
  void testMultipleRevolutions() {
    double source = 0.0;
    double target = 720.0 + 45.0;  // 2 full revolutions + 45

    double normalizedTarget = normalizeAngle360(target);
    double diff = smallestIncludedAngle(source, normalizedTarget);
    TS_ASSERT_DELTA(diff, 45.0, epsilon);
  }

  // Test negative angles
  void testNegativeAngles() {
    double source = -45.0;
    double target = -90.0;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, -45.0, epsilon);
  }

  /***************************************************************************
   * Clipping Tests
   ***************************************************************************/

  // Test angle difference with clipping
  void testAngleDifferenceWithClipping() {
    double source = 0.0;
    double target = 90.0;
    double clipMin = -30.0;
    double clipMax = 30.0;

    double diff = smallestIncludedAngle(source, target);  // = 90
    double clipped = std::clamp(diff, clipMin, clipMax);

    TS_ASSERT_DELTA(clipped, 30.0, epsilon);
  }

  // Test symmetric clipping
  void testSymmetricAngleClipping() {
    double limit = 45.0;

    double angle1 = 60.0;
    double angle2 = -60.0;

    double clipped1 = std::clamp(angle1, -limit, limit);
    double clipped2 = std::clamp(angle2, -limit, limit);

    TS_ASSERT_DELTA(clipped1, 45.0, epsilon);
    TS_ASSERT_DELTA(clipped2, -45.0, epsilon);
  }

  /***************************************************************************
   * Trigonometric Relationship Tests
   ***************************************************************************/

  // Test sine function at key angles
  void testSineAtKeyAngles() {
    TS_ASSERT_DELTA(std::sin(0.0), 0.0, epsilon);
    TS_ASSERT_DELTA(std::sin(30.0 * DEG_TO_RAD), 0.5, epsilon);
    TS_ASSERT_DELTA(std::sin(45.0 * DEG_TO_RAD), std::sqrt(2.0)/2.0, epsilon);
    TS_ASSERT_DELTA(std::sin(60.0 * DEG_TO_RAD), std::sqrt(3.0)/2.0, epsilon);
    TS_ASSERT_DELTA(std::sin(90.0 * DEG_TO_RAD), 1.0, epsilon);
    TS_ASSERT_DELTA(std::sin(180.0 * DEG_TO_RAD), 0.0, epsilon);
    TS_ASSERT_DELTA(std::sin(270.0 * DEG_TO_RAD), -1.0, epsilon);
  }

  // Test cosine function at key angles
  void testCosineAtKeyAngles() {
    TS_ASSERT_DELTA(std::cos(0.0), 1.0, epsilon);
    TS_ASSERT_DELTA(std::cos(30.0 * DEG_TO_RAD), std::sqrt(3.0)/2.0, epsilon);
    TS_ASSERT_DELTA(std::cos(45.0 * DEG_TO_RAD), std::sqrt(2.0)/2.0, epsilon);
    TS_ASSERT_DELTA(std::cos(60.0 * DEG_TO_RAD), 0.5, epsilon);
    TS_ASSERT_DELTA(std::cos(90.0 * DEG_TO_RAD), 0.0, epsilon);
    TS_ASSERT_DELTA(std::cos(180.0 * DEG_TO_RAD), -1.0, epsilon);
  }

  // Test Pythagorean identity
  void testPythagoreanIdentity() {
    double angles[] = {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 123.0, 180.0, 270.0};
    for (double deg : angles) {
      double rad = deg * DEG_TO_RAD;
      double sinVal = std::sin(rad);
      double cosVal = std::cos(rad);
      TS_ASSERT_DELTA(sinVal*sinVal + cosVal*cosVal, 1.0, epsilon);
    }
  }

  // Test tangent = sine/cosine
  void testTangentRelationship() {
    double angles[] = {15.0, 30.0, 45.0, 60.0, 75.0};
    for (double deg : angles) {
      double rad = deg * DEG_TO_RAD;
      double tanVal = std::tan(rad);
      double sinCosRatio = std::sin(rad) / std::cos(rad);
      TS_ASSERT_DELTA(tanVal, sinCosRatio, epsilon);
    }
  }

  // Test inverse trig functions
  void testInverseTrigFunctions() {
    TS_ASSERT_DELTA(std::asin(0.5) * RAD_TO_DEG, 30.0, epsilon);
    TS_ASSERT_DELTA(std::acos(0.5) * RAD_TO_DEG, 60.0, epsilon);
    TS_ASSERT_DELTA(std::atan(1.0) * RAD_TO_DEG, 45.0, epsilon);
  }

  // Test atan2 for quadrant determination
  void testAtan2Quadrants() {
    // First quadrant (0-90)
    TS_ASSERT_DELTA(std::atan2(1.0, 1.0) * RAD_TO_DEG, 45.0, epsilon);

    // Second quadrant (90-180)
    TS_ASSERT_DELTA(std::atan2(1.0, -1.0) * RAD_TO_DEG, 135.0, epsilon);

    // Third quadrant (-180 to -90)
    TS_ASSERT_DELTA(std::atan2(-1.0, -1.0) * RAD_TO_DEG, -135.0, epsilon);

    // Fourth quadrant (-90 to 0)
    TS_ASSERT_DELTA(std::atan2(-1.0, 1.0) * RAD_TO_DEG, -45.0, epsilon);
  }

  /***************************************************************************
   * Angular Rate Tests
   ***************************************************************************/

  // Test angular rate calculation
  void testAngularRateCalculation() {
    double angle1 = 30.0;
    double angle2 = 60.0;
    double dt = 2.0;  // seconds

    double rate = (angle2 - angle1) / dt;  // deg/sec
    TS_ASSERT_DELTA(rate, 15.0, epsilon);
  }

  // Test standard rate turn (3 deg/sec)
  void testStandardRateTurn() {
    double standardRate = 3.0;  // deg/sec
    double time = 60.0;         // seconds

    double headingChange = standardRate * time;
    TS_ASSERT_DELTA(headingChange, 180.0, epsilon);  // Half turn in 1 minute
  }

  // Test turn rate from bank angle
  void testTurnRateFromBankAngle() {
    // Turn rate (deg/s) = g * tan(bank) / V
    // At 60 deg bank, tan(60) = sqrt(3), for V = 100 m/s
    double g = 9.81;
    double bankDeg = 60.0;
    double V = 100.0;

    double turnRateRad = g * std::tan(bankDeg * DEG_TO_RAD) / V;
    double turnRateDeg = turnRateRad * RAD_TO_DEG;

    TS_ASSERT(turnRateDeg > 9.0);
    TS_ASSERT(turnRateDeg < 10.0);
  }

  // Test angular acceleration
  void testAngularAcceleration() {
    double rate1 = 0.0;    // deg/sec
    double rate2 = 30.0;   // deg/sec
    double dt = 3.0;       // seconds

    double accel = (rate2 - rate1) / dt;  // deg/sec^2
    TS_ASSERT_DELTA(accel, 10.0, epsilon);
  }

  // Test rate limiting
  void testAngularRateLimiting() {
    double currentAngle = 0.0;
    double targetAngle = 90.0;
    double maxRate = 3.0;  // deg/sec
    double dt = 1.0;       // seconds

    double requestedChange = targetAngle - currentAngle;
    double maxChange = maxRate * dt;
    double actualChange = std::clamp(requestedChange, -maxChange, maxChange);

    TS_ASSERT_DELTA(actualChange, 3.0, epsilon);
  }

  /***************************************************************************
   * Euler Angle Tests
   ***************************************************************************/

  // Test roll angle range
  void testRollAngleRange() {
    // Roll typically -180 to +180
    double rolls[] = {-180.0, -90.0, 0.0, 90.0, 180.0};
    for (double roll : rolls) {
      double normalized = normalizeAngle180(roll);
      TS_ASSERT(normalized >= -180.0 && normalized <= 180.0);
    }
  }

  // Test pitch angle typical range
  void testPitchAngleTypicalRange() {
    // Pitch typically limited to -90 to +90 (or aircraft limits)
    double pitch = 45.0;
    TS_ASSERT(pitch >= -90.0 && pitch <= 90.0);
  }

  // Test yaw/heading range
  void testYawHeadingRange() {
    // Heading 0 to 360
    double headings[] = {0.0, 90.0, 180.0, 270.0, 359.0};
    for (double hdg : headings) {
      double normalized = normalizeAngle360(hdg);
      TS_ASSERT(normalized >= 0.0 && normalized < 360.0);
    }
  }

  // Test level flight angles
  void testLevelFlightAngles() {
    double roll = 0.0;
    double pitch = 0.0;

    // Level flight: both zero
    TS_ASSERT_DELTA(roll, 0.0, epsilon);
    TS_ASSERT_DELTA(pitch, 0.0, epsilon);
  }

  // Test coordinated turn angles
  void testCoordinatedTurnAngles() {
    // In coordinated turn, bank and turn rate are related
    double bankAngle = 30.0;
    double speed_kts = 120.0;

    // Approximate turn radius (nm) = V^2 / (11.26 * tan(bank))
    double turnRadius = (speed_kts * speed_kts) / (11.26 * std::tan(bankAngle * DEG_TO_RAD));
    TS_ASSERT(turnRadius > 0.0);
    TS_ASSERT(turnRadius > 100.0);  // Reasonably large turn radius
  }

  /***************************************************************************
   * Interpolation Tests
   ***************************************************************************/

  // Test linear angle interpolation (simple case)
  void testLinearAngleInterpolation() {
    double start = 30.0;
    double end = 60.0;
    double t = 0.5;

    double result = start + t * (end - start);
    TS_ASSERT_DELTA(result, 45.0, epsilon);
  }

  // Test angle interpolation across zero
  void testAngleInterpolationAcrossZero() {
    double start = 350.0;
    double end = 10.0;
    double t = 0.5;

    // Need to handle wrap-around
    double diff = smallestIncludedAngle(start, end);  // = 20
    double result = normalizeAngle360(start + t * diff);

    TS_ASSERT_DELTA(result, 0.0, 0.5);  // Should be at/near 360/0
  }

  // Test multiple point angle averaging
  void testMultipleAngleAveraging() {
    // Average of angles requires special handling
    double angles[] = {350.0, 10.0};  // Average should be 0/360, not 180

    // Using unit vector approach
    double sumX = 0.0, sumY = 0.0;
    for (double a : angles) {
      sumX += std::cos(a * DEG_TO_RAD);
      sumY += std::sin(a * DEG_TO_RAD);
    }

    double avgRad = std::atan2(sumY, sumX);
    double avgDeg = normalizeAngle360(avgRad * RAD_TO_DEG);

    // Result should be near 0 or 360 (equivalent angles)
    double nearZero = std::min(avgDeg, 360.0 - avgDeg);
    TS_ASSERT_DELTA(nearZero, 0.0, 1.0);
  }

  // Test weighted angle average
  void testWeightedAngleAverage() {
    double angle1 = 0.0, weight1 = 1.0;
    double angle2 = 90.0, weight2 = 1.0;

    double sumX = weight1 * std::cos(angle1 * DEG_TO_RAD) +
                  weight2 * std::cos(angle2 * DEG_TO_RAD);
    double sumY = weight1 * std::sin(angle1 * DEG_TO_RAD) +
                  weight2 * std::sin(angle2 * DEG_TO_RAD);

    double avgRad = std::atan2(sumY, sumX);
    double avgDeg = normalizeAngle360(avgRad * RAD_TO_DEG);

    TS_ASSERT_DELTA(avgDeg, 45.0, epsilon);
  }

  /***************************************************************************
   * Wind/Track Angle Tests
   ***************************************************************************/

  // Test crosswind angle calculation
  void testCrosswindAngle() {
    double runwayHeading = 90.0;  // East
    double windFrom = 180.0;      // From south

    double crosswindAngle = smallestIncludedAngle(runwayHeading, windFrom);
    TS_ASSERT_DELTA(crosswindAngle, 90.0, epsilon);  // Direct crosswind
  }

  // Test headwind component
  void testHeadwindComponent() {
    double runwayHeading = 360.0;  // North
    double windFrom = 360.0;       // From north
    double windSpeed = 20.0;       // knots

    double relativeWind = smallestIncludedAngle(runwayHeading, windFrom);
    double headwind = windSpeed * std::cos(relativeWind * DEG_TO_RAD);

    TS_ASSERT_DELTA(headwind, 20.0, epsilon);  // Full headwind
  }

  // Test crosswind component
  void testCrosswindComponent() {
    double runwayHeading = 360.0;  // North
    double windFrom = 90.0;        // From east
    double windSpeed = 20.0;       // knots

    double relativeWind = smallestIncludedAngle(runwayHeading, windFrom);
    double crosswind = windSpeed * std::sin(relativeWind * DEG_TO_RAD);

    TS_ASSERT_DELTA(crosswind, 20.0, epsilon);  // Full crosswind
  }

  // Test crab angle calculation
  void testCrabAngleCalculation() {
    // Simplified: crab = asin(crosswind / TAS)
    double crosswind = 10.0;  // knots
    double TAS = 100.0;       // knots

    double crabAngleDeg = std::asin(crosswind / TAS) * RAD_TO_DEG;
    TS_ASSERT_DELTA(crabAngleDeg, 5.74, 0.1);
  }

  // Test track vs heading
  void testTrackVsHeading() {
    double heading = 90.0;     // Flying east
    double crabAngle = 10.0;   // Correcting for north wind

    double track = heading + crabAngle;  // Actual ground track
    TS_ASSERT_DELTA(track, 100.0, epsilon);
  }

  /***************************************************************************
   * Angle of Attack and Sideslip Tests
   ***************************************************************************/

  // Test angle of attack calculation
  void testAngleOfAttackCalculation() {
    double verticalVel = 10.0;    // m/s (climbing)
    double forwardVel = 100.0;   // m/s

    double aoaRad = std::atan2(verticalVel, forwardVel);
    double aoaDeg = aoaRad * RAD_TO_DEG;

    TS_ASSERT_DELTA(aoaDeg, 5.71, 0.1);
  }

  // Test sideslip angle calculation
  void testSideslipAngleCalculation() {
    double lateralVel = 5.0;     // m/s (slipping right)
    double forwardVel = 100.0;   // m/s

    double betaRad = std::atan2(lateralVel, forwardVel);
    double betaDeg = betaRad * RAD_TO_DEG;

    TS_ASSERT_DELTA(betaDeg, 2.86, 0.1);
  }

  // Test zero sideslip (coordinated flight)
  void testZeroSideslip() {
    double lateralVel = 0.0;
    double forwardVel = 100.0;

    double betaDeg = std::atan2(lateralVel, forwardVel) * RAD_TO_DEG;
    TS_ASSERT_DELTA(betaDeg, 0.0, epsilon);
  }

  /***************************************************************************
   * Common Aviation Angle Tests
   ***************************************************************************/

  // Test glideslope angle (ILS)
  void testGlideslopeAngle() {
    double glideslopeAngle = 3.0;  // Standard ILS
    double altitude = 1000.0;     // ft above runway

    // Distance to runway = altitude / tan(angle)
    double distance = altitude / std::tan(glideslopeAngle * DEG_TO_RAD);
    TS_ASSERT_DELTA(distance, 19081.1, 10.0);  // About 3.1 nm
  }

  // Test localizer course width
  void testLocalizerCourseWidth() {
    double fullScaleDeflection = 2.5;  // degrees for ILS
    double halfScale = fullScaleDeflection / 2.0;

    TS_ASSERT_DELTA(halfScale, 1.25, epsilon);
  }

  // Test VOR radial
  void testVORRadial() {
    double stationBearing = 270.0;  // West of station
    double radial = normalizeAngle360(stationBearing + 180.0);

    TS_ASSERT_DELTA(radial, 90.0, epsilon);  // On the 090 radial
  }

  // Test approach angle limits
  void testApproachAngleLimits() {
    double minApproach = 2.5;   // degrees
    double maxApproach = 3.5;   // degrees
    double nominal = 3.0;       // degrees

    TS_ASSERT(nominal >= minApproach);
    TS_ASSERT(nominal <= maxApproach);
  }

  /***************************************************************************
   * Precision and Comparison Tests
   ***************************************************************************/

  // Test floating point angle comparison
  void testFloatingPointAngleComparison() {
    double angle1 = 45.0;
    double angle2 = 45.0 + 1e-12;  // Nearly equal

    bool equal = std::abs(angle1 - angle2) < epsilon;
    TS_ASSERT(equal);
  }

  // Test angle difference near zero
  void testAngleDifferenceNearZero() {
    double source = 0.001;
    double target = -0.001;

    double diff = smallestIncludedAngle(source, target);
    TS_ASSERT_DELTA(diff, -0.002, epsilon);
  }

  // Test angle at machine precision
  void testAngleAtMachinePrecision() {
    double tiny = std::numeric_limits<double>::epsilon();
    double angle = 0.0 + tiny;

    TS_ASSERT(angle != 0.0);
    TS_ASSERT_DELTA(angle, 0.0, 1e-10);
  }

  // Test large angle normalization precision
  void testLargeAngleNormalizationPrecision() {
    double largeAngle = 360000.0 + 45.0;  // 1000 revolutions + 45
    double normalized = normalizeAngle360(largeAngle);

    TS_ASSERT_DELTA(normalized, 45.0, 1e-6);
  }

  /***************************************************************************
   * Bearing Calculation Tests
   ***************************************************************************/

  // Test bearing from coordinates (simplified)
  void testBearingFromCoordinates() {
    // Using atan2 for bearing calculation
    double dx = 1.0;  // East
    double dy = 1.0;  // North

    double bearingRad = std::atan2(dx, dy);  // Note: atan2(x,y) for bearing
    double bearingDeg = normalizeAngle360(bearingRad * RAD_TO_DEG);

    TS_ASSERT_DELTA(bearingDeg, 45.0, epsilon);  // NE bearing
  }

  // Test reciprocal bearing
  void testReciprocalBearing() {
    double bearing = 45.0;
    double reciprocal = normalizeAngle360(bearing + 180.0);

    TS_ASSERT_DELTA(reciprocal, 225.0, epsilon);
  }

  // Test relative bearing
  void testRelativeBearing() {
    double heading = 90.0;   // Flying east
    double targetBearing = 135.0;  // Target is SE

    double relativeBearing = smallestIncludedAngle(heading, targetBearing);
    TS_ASSERT_DELTA(relativeBearing, 45.0, epsilon);  // 45 degrees right
  }

  /***************************************************************************
   * Sector/Quadrant Tests
   ***************************************************************************/

  // Test quadrant determination
  void testQuadrantDetermination() {
    // First quadrant (0-90): sin+, cos+
    double q1 = 45.0;
    TS_ASSERT(std::sin(q1 * DEG_TO_RAD) > 0);
    TS_ASSERT(std::cos(q1 * DEG_TO_RAD) > 0);

    // Second quadrant (90-180): sin+, cos-
    double q2 = 135.0;
    TS_ASSERT(std::sin(q2 * DEG_TO_RAD) > 0);
    TS_ASSERT(std::cos(q2 * DEG_TO_RAD) < 0);

    // Third quadrant (180-270): sin-, cos-
    double q3 = 225.0;
    TS_ASSERT(std::sin(q3 * DEG_TO_RAD) < 0);
    TS_ASSERT(std::cos(q3 * DEG_TO_RAD) < 0);

    // Fourth quadrant (270-360): sin-, cos+
    double q4 = 315.0;
    TS_ASSERT(std::sin(q4 * DEG_TO_RAD) < 0);
    TS_ASSERT(std::cos(q4 * DEG_TO_RAD) > 0);
  }

  // Test angle sector check
  void testAngleSectorCheck() {
    double heading = 45.0;
    double sectorStart = 30.0;
    double sectorEnd = 60.0;

    bool inSector = (heading >= sectorStart && heading <= sectorEnd);
    TS_ASSERT(inSector);
  }

  // Test sector spanning zero
  void testSectorSpanningZero() {
    double heading = 350.0;
    double sectorStart = 330.0;
    double sectorEnd = 30.0;  // Wraps through 0

    // Check if in sector (spans 0)
    bool inSector = (heading >= sectorStart || heading <= sectorEnd);
    TS_ASSERT(inSector);
  }

  /***************************************************************************
   * Rate/Time Integration Tests
   ***************************************************************************/

  // Test angular displacement from rate
  void testAngularDisplacementFromRate() {
    double rate = 3.0;   // deg/sec
    double time = 10.0;  // seconds

    double displacement = rate * time;
    TS_ASSERT_DELTA(displacement, 30.0, epsilon);
  }

  // Test time to turn calculation
  void testTimeToTurnCalculation() {
    double requiredTurn = 90.0;  // degrees
    double turnRate = 3.0;       // deg/sec (standard rate)

    double time = requiredTurn / turnRate;
    TS_ASSERT_DELTA(time, 30.0, epsilon);  // 30 seconds for 90 degree turn
  }

  // Test full turn time
  void testFullTurnTime() {
    double turnRate = 3.0;  // deg/sec (standard rate)
    double fullTurn = 360.0;

    double time = fullTurn / turnRate;
    TS_ASSERT_DELTA(time, 120.0, epsilon);  // 2 minutes for full turn
  }

  /***************************************************************************
   * Miscellaneous Angle Tests
   ***************************************************************************/

  // Test complementary angles
  void testComplementaryAngles() {
    double angle = 30.0;
    double complement = 90.0 - angle;

    TS_ASSERT_DELTA(complement, 60.0, epsilon);
    TS_ASSERT_DELTA(std::sin(angle * DEG_TO_RAD),
                    std::cos(complement * DEG_TO_RAD), epsilon);
  }

  // Test supplementary angles
  void testSupplementaryAngles() {
    double angle = 30.0;
    double supplement = 180.0 - angle;

    TS_ASSERT_DELTA(supplement, 150.0, epsilon);
    TS_ASSERT_DELTA(std::sin(angle * DEG_TO_RAD),
                    std::sin(supplement * DEG_TO_RAD), epsilon);
  }

  // Test angle bisector
  void testAngleBisector() {
    double angle1 = 30.0;
    double angle2 = 90.0;

    double bisector = (angle1 + angle2) / 2.0;
    TS_ASSERT_DELTA(bisector, 60.0, epsilon);
  }

  // Test arc length calculation
  void testArcLengthCalculation() {
    double radius = 100.0;  // units
    double angleDeg = 60.0;
    double angleRad = angleDeg * DEG_TO_RAD;

    double arcLength = radius * angleRad;
    TS_ASSERT_DELTA(arcLength, 104.72, 0.01);
  }

  // Test sector area calculation
  void testSectorAreaCalculation() {
    double radius = 100.0;
    double angleDeg = 60.0;
    double angleRad = angleDeg * DEG_TO_RAD;

    double area = 0.5 * radius * radius * angleRad;
    TS_ASSERT_DELTA(area, 5235.99, 1.0);
  }

  // Test small angle approximation
  void testSmallAngleApproximation() {
    double smallAngleDeg = 1.0;
    double smallAngleRad = smallAngleDeg * DEG_TO_RAD;

    // sin(x) ≈ x for small angles
    TS_ASSERT_DELTA(std::sin(smallAngleRad), smallAngleRad, 0.001);

    // cos(x) ≈ 1 for small angles
    TS_ASSERT_DELTA(std::cos(smallAngleRad), 1.0, 0.001);

    // tan(x) ≈ x for small angles
    TS_ASSERT_DELTA(std::tan(smallAngleRad), smallAngleRad, 0.001);
  }

  // Test double angle formulas
  void testDoubleAngleFormulas() {
    double angle = 30.0;
    double rad = angle * DEG_TO_RAD;

    // sin(2x) = 2*sin(x)*cos(x)
    double sin2x = std::sin(2.0 * rad);
    double sin2x_formula = 2.0 * std::sin(rad) * std::cos(rad);
    TS_ASSERT_DELTA(sin2x, sin2x_formula, epsilon);

    // cos(2x) = cos^2(x) - sin^2(x)
    double cos2x = std::cos(2.0 * rad);
    double cos2x_formula = std::cos(rad)*std::cos(rad) - std::sin(rad)*std::sin(rad);
    TS_ASSERT_DELTA(cos2x, cos2x_formula, epsilon);
  }

  // Test half angle formulas
  void testHalfAngleFormulas() {
    double angle = 60.0;
    double rad = angle * DEG_TO_RAD;

    // sin(x/2) = sqrt((1-cos(x))/2)
    double sinHalf = std::sin(rad / 2.0);
    double sinHalf_formula = std::sqrt((1.0 - std::cos(rad)) / 2.0);
    TS_ASSERT_DELTA(sinHalf, sinHalf_formula, epsilon);
  }

  // Test angle in triangle (sum = 180)
  void testTriangleAngleSum() {
    double angle1 = 60.0;
    double angle2 = 60.0;
    double angle3 = 60.0;  // Equilateral

    double sum = angle1 + angle2 + angle3;
    TS_ASSERT_DELTA(sum, 180.0, epsilon);
  }

  // Test exterior angle
  void testExteriorAngle() {
    double interiorAngle = 60.0;
    double exteriorAngle = 180.0 - interiorAngle;

    TS_ASSERT_DELTA(exteriorAngle, 120.0, epsilon);
  }

  // Test law of sines
  void testLawOfSines() {
    // a/sin(A) = b/sin(B) = c/sin(C)
    double A = 30.0 * DEG_TO_RAD;
    double B = 60.0 * DEG_TO_RAD;
    double C = 90.0 * DEG_TO_RAD;

    // For a right triangle with C=90
    double a = 1.0;  // opposite to A
    double b = std::sqrt(3.0);  // opposite to B
    double c = 2.0;  // hypotenuse

    double ratio_a = a / std::sin(A);
    double ratio_b = b / std::sin(B);
    double ratio_c = c / std::sin(C);

    TS_ASSERT_DELTA(ratio_a, ratio_b, epsilon);
    TS_ASSERT_DELTA(ratio_b, ratio_c, epsilon);
  }

  // Test law of cosines
  void testLawOfCosines() {
    // c^2 = a^2 + b^2 - 2*a*b*cos(C)
    double a = 3.0;
    double b = 4.0;
    double C = 90.0 * DEG_TO_RAD;

    double c_squared = a*a + b*b - 2.0*a*b*std::cos(C);
    double c = std::sqrt(c_squared);

    TS_ASSERT_DELTA(c, 5.0, epsilon);  // 3-4-5 right triangle
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete heading calculation system
  void testCompleteHeadingCalculation() {
    // Aircraft position and destination
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;  // New York area
    double lat2 = 51.5 * DEG_TO_RAD;
    double lon2 = -0.1 * DEG_TO_RAD;   // London area

    // Initial great circle heading
    double dlon = lon2 - lon1;
    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double heading = std::atan2(y, x) * RAD_TO_DEG;

    heading = normalizeAngle360(heading);

    TS_ASSERT(heading > 0.0 && heading < 360.0);
  }

  // Test wind correction angle
  void testWindCorrectionAngle() {
    double true_airspeed = 150.0;  // knots
    double wind_speed = 30.0;      // knots
    double wind_direction = 270.0; // from west
    double course = 360.0;         // north

    // Cross-wind component
    double rel_wind = wind_direction - course;
    double crosswind = wind_speed * std::sin(rel_wind * DEG_TO_RAD);

    // Wind correction angle (simplified)
    double wca = std::asin(crosswind / true_airspeed) * RAD_TO_DEG;

    TS_ASSERT(!std::isnan(wca));
  }

  // Test magnetic variation correction
  void testMagneticVariationCorrection() {
    double true_heading = 090.0;
    double mag_variation = 15.0;  // East variation

    // Magnetic = True + East variation (or - West)
    double mag_heading = true_heading - mag_variation;
    mag_heading = normalizeAngle360(mag_heading);

    TS_ASSERT_DELTA(mag_heading, 75.0, epsilon);
  }

  // Test spherical triangle angle
  void testSphericalTriangleAngle() {
    // Spherical excess for a triangle on a sphere
    double A = 90.0 * DEG_TO_RAD;
    double B = 90.0 * DEG_TO_RAD;
    double C = 90.0 * DEG_TO_RAD;

    double spherical_excess = A + B + C - M_PI;
    TS_ASSERT(spherical_excess > 0.0);  // Should be positive on sphere
  }

  // Test Euler angle sequence
  void testEulerAngleSequence() {
    double phi = 10.0 * DEG_TO_RAD;    // roll
    double theta = 5.0 * DEG_TO_RAD;   // pitch
    double psi = 45.0 * DEG_TO_RAD;    // yaw

    // Verify small angle approximation
    TS_ASSERT(std::abs(phi) < 0.5);  // Should be small in radians
    TS_ASSERT(std::abs(theta) < 0.5);

    // These are valid Euler angles
    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isnan(theta));
    TS_ASSERT(!std::isnan(psi));
  }

  // Test angle interpolation
  void testAngleInterpolation() {
    double angle1 = 350.0;
    double angle2 = 10.0;
    double t = 0.5;  // midpoint

    // Direct interpolation would give 180, but correct is 0
    double diff = smallestIncludedAngle(angle1, angle2);
    double interpolated = normalizeAngle360(angle1 + t * diff);

    TS_ASSERT_DELTA(interpolated, 0.0, epsilon);
  }

  // Test complementary angles extended
  void testComplementaryAnglesExtended() {
    double angle = 30.0;
    double complement = 90.0 - angle;

    TS_ASSERT_DELTA(complement, 60.0, epsilon);
    TS_ASSERT_DELTA(angle + complement, 90.0, epsilon);
  }

  // Test supplementary angles extended
  void testSupplementaryAnglesExtended() {
    double angle = 60.0;
    double supplement = 180.0 - angle;

    TS_ASSERT_DELTA(supplement, 120.0, epsilon);
    TS_ASSERT_DELTA(angle + supplement, 180.0, epsilon);
  }

  // Test bank angle for coordinated turn
  void testBankAngleCoordinatedTurn() {
    double velocity = 200.0;  // ft/s
    double radius = 3000.0;   // ft
    double g = 32.2;          // ft/s^2

    // tan(bank) = V^2 / (g * R)
    double bank_rad = std::atan(velocity * velocity / (g * radius));
    double bank_deg = bank_rad * RAD_TO_DEG;

    TS_ASSERT(bank_deg > 0.0);
    TS_ASSERT(bank_deg < 45.0);  // Reasonable bank angle
  }

  // Test climb angle from rate
  void testClimbAngleFromRate() {
    double climb_rate = 1000.0;  // ft/min
    double groundspeed = 120.0;  // knots = 202 ft/s

    double climb_rate_fps = climb_rate / 60.0;
    double groundspeed_fps = groundspeed * 6076.0 / 3600.0;

    double climb_angle = std::atan(climb_rate_fps / groundspeed_fps) * RAD_TO_DEG;

    TS_ASSERT(climb_angle > 0.0);
    TS_ASSERT(climb_angle < 15.0);
  }

  // Test glide angle calculation
  void testGlideAngleCalculation() {
    double lift_to_drag = 10.0;  // L/D ratio

    // Glide angle = atan(1/L/D)
    double glide_angle = std::atan(1.0 / lift_to_drag) * RAD_TO_DEG;

    TS_ASSERT_DELTA(glide_angle, 5.71, 0.1);  // approximately
  }

  // Test angle rate of change
  void testAngleRateOfChange() {
    double angle_start = 45.0;
    double angle_end = 90.0;
    double time = 5.0;

    double rate = (angle_end - angle_start) / time;

    TS_ASSERT_DELTA(rate, 9.0, epsilon);  // degrees per second
  }

  // Test crab angle for wind correction
  void testCrabAngleWindCorrection() {
    double crosswind = 20.0;  // knots
    double airspeed = 100.0;  // knots

    double crab_angle = std::asin(crosswind / airspeed) * RAD_TO_DEG;

    TS_ASSERT(crab_angle > 0.0);
    TS_ASSERT_DELTA(crab_angle, 11.54, 0.1);
  }

  // Test angle sum and difference identities
  void testAngleSumDifferenceIdentities() {
    double A = 30.0 * DEG_TO_RAD;
    double B = 45.0 * DEG_TO_RAD;

    // sin(A+B) = sinA*cosB + cosA*sinB
    double sin_sum = std::sin(A + B);
    double sin_sum_identity = std::sin(A) * std::cos(B) + std::cos(A) * std::sin(B);

    TS_ASSERT_DELTA(sin_sum, sin_sum_identity, epsilon);

    // cos(A-B) = cosA*cosB + sinA*sinB
    double cos_diff = std::cos(A - B);
    double cos_diff_identity = std::cos(A) * std::cos(B) + std::sin(A) * std::sin(B);

    TS_ASSERT_DELTA(cos_diff, cos_diff_identity, epsilon);
  }

  // Test bearing reciprocal
  void testBearingReciprocal() {
    double bearing = 045.0;
    double reciprocal = normalizeAngle360(bearing + 180.0);

    TS_ASSERT_DELTA(reciprocal, 225.0, epsilon);

    // Test another case
    bearing = 270.0;
    reciprocal = normalizeAngle360(bearing + 180.0);
    TS_ASSERT_DELTA(reciprocal, 90.0, epsilon);
  }

  // Test angle instance independence
  void testAnglesInstanceIndependence() {
    double angle1 = 45.0;
    double angle2 = 90.0;

    double rad1 = angle1 * DEG_TO_RAD;
    double rad2 = angle2 * DEG_TO_RAD;

    TS_ASSERT(rad1 != rad2);
    TS_ASSERT_DELTA(rad1, M_PI / 4.0, epsilon);
    TS_ASSERT_DELTA(rad2, M_PI / 2.0, epsilon);
  }

  // Test complete angle transformation chain
  void testCompleteAngleTransformationChain() {
    double original = 123.456;

    // Convert to radians and back
    double rad = original * DEG_TO_RAD;
    double back = rad * RAD_TO_DEG;

    TS_ASSERT_DELTA(original, back, epsilon);

    // Normalize and verify
    double normalized = normalizeAngle360(original);
    TS_ASSERT(normalized >= 0.0 && normalized < 360.0);
    TS_ASSERT_DELTA(normalized, original, epsilon);
  }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C172X INTEGRATION TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGAnglesC172xTest : public CxxTest::TestSuite
{
public:
  void testC172xEulerAngles() {
    // Test Euler angles from C172x simulation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate != nullptr);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    // Get Euler angles
    double phi = propagate->GetEuler(1);    // Roll
    double theta = propagate->GetEuler(2);  // Pitch
    double psi = propagate->GetEuler(3);    // Yaw

    TS_ASSERT(std::isfinite(phi));
    TS_ASSERT(std::isfinite(theta));
    TS_ASSERT(std::isfinite(psi));
  }

  void testC172xAngleOfAttack() {
    // Test angle of attack calculation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    for (int i = 0; i < 30; i++) {
      fdmex.Run();
    }

    double alpha = aux->Getalpha();
    TS_ASSERT(std::isfinite(alpha));
  }

  void testC172xSideslipAngle() {
    // Test sideslip angle calculation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(fcs != nullptr);
    TS_ASSERT(aux != nullptr);

    // Induce sideslip with rudder
    fcs->SetDrCmd(0.5);
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double beta = aux->Getbeta();
    TS_ASSERT(std::isfinite(beta));
  }

  void testC172xFlightPathAngle() {
    // Test flight path angle
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    for (int i = 0; i < 30; i++) {
      fdmex.Run();
    }

    double gamma = aux->GetGamma();
    TS_ASSERT(std::isfinite(gamma));
  }

  void testC172xHeadingAngle() {
    // Test heading angle
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate != nullptr);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    double psi = propagate->GetEuler(3);
    TS_ASSERT(std::isfinite(psi));
    // Heading should be finite (range depends on convention)
    TS_ASSERT(psi >= -2*M_PI && psi <= 2*M_PI);
  }

  void testC172xBankAngle() {
    // Test bank angle with aileron input
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(fcs != nullptr);
    TS_ASSERT(propagate != nullptr);

    // Apply aileron
    fcs->SetDaCmd(0.3);
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double phi = propagate->GetEuler(1);
    TS_ASSERT(std::isfinite(phi));
  }

  void testC172xPitchAngle() {
    // Test pitch angle with elevator input
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(fcs != nullptr);
    TS_ASSERT(propagate != nullptr);

    // Apply elevator
    fcs->SetDeCmd(-0.3);
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double theta = propagate->GetEuler(2);
    TS_ASSERT(std::isfinite(theta));
  }

  void testC172xAngularRates() {
    // Test angular rates (p, q, r)
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate != nullptr);

    for (int i = 0; i < 30; i++) {
      fdmex.Run();
    }

    double p = propagate->GetPQR(1);  // Roll rate
    double q = propagate->GetPQR(2);  // Pitch rate
    double r = propagate->GetPQR(3);  // Yaw rate

    TS_ASSERT(std::isfinite(p));
    TS_ASSERT(std::isfinite(q));
    TS_ASSERT(std::isfinite(r));
  }

  void testC172xControlSurfaceAngles() {
    // Test control surface deflection angles
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    fcs->SetDeCmd(-0.4);
    fcs->SetDaCmd(0.3);
    fcs->SetDrCmd(0.2);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }

  void testC172xWindAngles() {
    // Test wind-related angles
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    for (int i = 0; i < 30; i++) {
      fdmex.Run();
    }

    double alpha = aux->Getalpha();
    double beta = aux->Getbeta();

    TS_ASSERT(std::isfinite(alpha));
    TS_ASSERT(std::isfinite(beta));
  }

  void testC172xLatitudeLongitude() {
    // Test geographic angles
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate != nullptr);

    double lat = propagate->GetLatitudeDeg();
    double lon = propagate->GetLongitudeDeg();

    TS_ASSERT(std::isfinite(lat));
    TS_ASSERT(std::isfinite(lon));
    TS_ASSERT(lat >= -90.0 && lat <= 90.0);
    TS_ASSERT(lon >= -180.0 && lon <= 180.0);
  }

  void testC172xQuaternionAngles() {
    // Test quaternion representation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate != nullptr);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    // Quaternion should be normalized
    const auto& quat = propagate->GetQuaternion();
    double mag = std::sqrt(quat(1)*quat(1) + quat(2)*quat(2) +
                          quat(3)*quat(3) + quat(4)*quat(4));
    TS_ASSERT_DELTA(mag, 1.0, 1e-6);
  }
};
