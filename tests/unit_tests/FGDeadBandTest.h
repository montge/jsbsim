/*******************************************************************************
 * FGDeadBandTest.h - Unit tests for FGDeadBand (deadband logic)
 *
 * Tests the mathematical behavior of deadband components:
 * - Zero output within deadband region
 * - Linear output outside deadband
 * - Symmetric and asymmetric deadbands
 *
 * Note: FGDeadBand requires XML element for construction, so these tests focus
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

class FGDeadBandTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Deadband Helper Functions
   ***************************************************************************/

  // Symmetric deadband: output = 0 for |input| < width, else input - sign(input)*width
  double symmetricDeadband(double input, double width) {
    if (std::abs(input) < width) {
      return 0.0;
    } else if (input > 0) {
      return input - width;
    } else {
      return input + width;
    }
  }

  // Asymmetric deadband
  double asymmetricDeadband(double input, double lowWidth, double highWidth) {
    if (input > highWidth) {
      return input - highWidth;
    } else if (input < -lowWidth) {
      return input + lowWidth;
    } else {
      return 0.0;
    }
  }

  /***************************************************************************
   * Symmetric Deadband Tests
   ***************************************************************************/

  // Test zero output within deadband
  void testSymmetricDeadbandZeroRegion() {
    double width = 0.1;

    TS_ASSERT_DELTA(symmetricDeadband(0.0, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(0.05, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.05, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(0.099, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.099, width), 0.0, epsilon);
  }

  // Test output at deadband boundary
  void testSymmetricDeadbandBoundary() {
    double width = 0.1;

    // At exactly the boundary, should be zero or very close
    double output = symmetricDeadband(width, width);
    TS_ASSERT_DELTA(output, 0.0, epsilon);

    output = symmetricDeadband(-width, width);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test positive output above deadband
  void testSymmetricDeadbandPositive() {
    double width = 0.1;

    // Input 0.2, output should be 0.2 - 0.1 = 0.1
    double output = symmetricDeadband(0.2, width);
    TS_ASSERT_DELTA(output, 0.1, epsilon);

    // Input 1.0, output should be 1.0 - 0.1 = 0.9
    output = symmetricDeadband(1.0, width);
    TS_ASSERT_DELTA(output, 0.9, epsilon);
  }

  // Test negative output below deadband
  void testSymmetricDeadbandNegative() {
    double width = 0.1;

    // Input -0.2, output should be -0.2 + 0.1 = -0.1
    double output = symmetricDeadband(-0.2, width);
    TS_ASSERT_DELTA(output, -0.1, epsilon);

    // Input -1.0, output should be -1.0 + 0.1 = -0.9
    output = symmetricDeadband(-1.0, width);
    TS_ASSERT_DELTA(output, -0.9, epsilon);
  }

  // Test symmetry of deadband
  void testSymmetricDeadbandSymmetry() {
    double width = 0.1;

    // Output magnitude should be same for same magnitude input
    double pos_out = symmetricDeadband(0.5, width);
    double neg_out = symmetricDeadband(-0.5, width);

    TS_ASSERT_DELTA(std::abs(pos_out), std::abs(neg_out), epsilon);
    TS_ASSERT_DELTA(pos_out, -neg_out, epsilon);
  }

  /***************************************************************************
   * Zero Width Deadband
   ***************************************************************************/

  // Test zero width deadband (pass-through)
  void testZeroWidthDeadband() {
    double width = 0.0;

    TS_ASSERT_DELTA(symmetricDeadband(0.0, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(0.5, width), 0.5, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.5, width), -0.5, epsilon);
  }

  /***************************************************************************
   * Large Deadband Tests
   ***************************************************************************/

  // Test large deadband that captures most input range
  void testLargeDeadband() {
    double width = 10.0;

    // Inputs within [-10, 10] should give zero
    TS_ASSERT_DELTA(symmetricDeadband(5.0, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-5.0, width), 0.0, epsilon);

    // Inputs outside should give offset value
    TS_ASSERT_DELTA(symmetricDeadband(15.0, width), 5.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-15.0, width), -5.0, epsilon);
  }

  /***************************************************************************
   * Asymmetric Deadband Tests
   ***************************************************************************/

  // Test asymmetric deadband with different thresholds
  void testAsymmetricDeadband() {
    double lowWidth = 0.1;
    double highWidth = 0.2;

    // Within [-0.1, 0.2] should give zero
    TS_ASSERT_DELTA(asymmetricDeadband(0.0, lowWidth, highWidth), 0.0, epsilon);
    TS_ASSERT_DELTA(asymmetricDeadband(0.15, lowWidth, highWidth), 0.0, epsilon);
    TS_ASSERT_DELTA(asymmetricDeadband(-0.05, lowWidth, highWidth), 0.0, epsilon);

    // Above 0.2
    TS_ASSERT_DELTA(asymmetricDeadband(0.5, lowWidth, highWidth), 0.3, epsilon);

    // Below -0.1
    TS_ASSERT_DELTA(asymmetricDeadband(-0.5, lowWidth, highWidth), -0.4, epsilon);
  }

  /***************************************************************************
   * Gain After Deadband
   ***************************************************************************/

  // Test applying gain after deadband
  void testDeadbandWithGain() {
    double width = 0.1;
    double gain = 2.0;

    double db_output = symmetricDeadband(0.5, width);  // = 0.4
    double final_output = db_output * gain;            // = 0.8

    TS_ASSERT_DELTA(final_output, 0.8, epsilon);
  }

  /***************************************************************************
   * Clipping After Deadband
   ***************************************************************************/

  // Test clipping after deadband
  void testDeadbandWithClipping() {
    double width = 0.1;
    double clipMin = -0.5;
    double clipMax = 0.5;

    double db_output = symmetricDeadband(1.0, width);  // = 0.9
    double clipped = std::clamp(db_output, clipMin, clipMax);

    TS_ASSERT_DELTA(clipped, clipMax, epsilon);
  }

  /***************************************************************************
   * Continuity Tests
   ***************************************************************************/

  // Test output is continuous at deadband boundary
  void testContinuityAtBoundary() {
    double width = 0.1;

    // Just inside and just outside the boundary
    double inside = symmetricDeadband(width - 1e-12, width);
    double outside = symmetricDeadband(width + 1e-12, width);

    // Both should be very close to zero
    TS_ASSERT_DELTA(inside, 0.0, 1e-10);
    TS_ASSERT_DELTA(outside, 0.0, 1e-10);
  }

  // Test slope outside deadband is 1
  void testUnitSlopeOutsideDeadband() {
    double width = 0.1;
    double x1 = 0.5;
    double x2 = 1.0;

    double y1 = symmetricDeadband(x1, width);
    double y2 = symmetricDeadband(x2, width);

    double slope = (y2 - y1) / (x2 - x1);
    TS_ASSERT_DELTA(slope, 1.0, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test with very small inputs
  void testVerySmallInputs() {
    double width = 0.1;
    double smallInput = 1e-15;

    double output = symmetricDeadband(smallInput, width);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test with very large inputs
  void testVeryLargeInputs() {
    double width = 0.1;
    double largeInput = 1e10;

    double output = symmetricDeadband(largeInput, width);
    TS_ASSERT_DELTA(output, largeInput - width, 1.0);  // Allow tolerance for large values
  }

  // Test with infinity
  void testInfinityInput() {
    double width = 0.1;
    double infInput = std::numeric_limits<double>::infinity();

    double output = symmetricDeadband(infInput, width);
    TS_ASSERT(std::isinf(output));
    TS_ASSERT(output > 0);
  }

  // Test with NaN
  void testNaNInput() {
    double width = 0.1;
    double nanInput = std::numeric_limits<double>::quiet_NaN();

    double output = symmetricDeadband(nanInput, width);
    // NaN comparisons are tricky - the function should propagate NaN
    TS_ASSERT(std::isnan(output));
  }

  /***************************************************************************
   * Practical Applications
   ***************************************************************************/

  // Test joystick deadband (centered at zero)
  void testJoystickDeadband() {
    double deadzone = 0.05;  // 5% deadzone

    // Small movements ignored
    TS_ASSERT_DELTA(symmetricDeadband(0.03, deadzone), 0.0, epsilon);

    // Full deflection gives 95% of normalized output
    TS_ASSERT_DELTA(symmetricDeadband(1.0, deadzone), 0.95, epsilon);
  }

  // Test throttle deadband (one-sided)
  void testThrottleDeadband() {
    // Throttle only positive, deadband at low end
    double input = 0.05;
    double deadzone = 0.1;

    // Could use symmetric or just clamp to zero for negative
    double output = std::max(0.0, symmetricDeadband(input, deadzone));
    TS_ASSERT_DELTA(output, 0.0, epsilon);

    input = 0.5;
    output = std::max(0.0, symmetricDeadband(input, deadzone));
    TS_ASSERT_DELTA(output, 0.4, epsilon);
  }
};
