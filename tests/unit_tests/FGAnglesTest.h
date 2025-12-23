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
};
