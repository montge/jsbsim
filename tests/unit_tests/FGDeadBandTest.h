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

  /***************************************************************************
   * Additional Asymmetric Deadband Tests
   ***************************************************************************/

  void testAsymmetricDeadbandZeroInput() {
    double lowWidth = 0.2;
    double highWidth = 0.3;

    double output = asymmetricDeadband(0.0, lowWidth, highWidth);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  void testAsymmetricDeadbandBoundaries() {
    double lowWidth = 0.1;
    double highWidth = 0.2;

    // At lower boundary
    double output = asymmetricDeadband(-lowWidth, lowWidth, highWidth);
    TS_ASSERT_DELTA(output, 0.0, epsilon);

    // At upper boundary
    output = asymmetricDeadband(highWidth, lowWidth, highWidth);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  void testAsymmetricDeadbandLargeAsymmetry() {
    double lowWidth = 0.05;
    double highWidth = 0.5;

    // Small negative input outside
    TS_ASSERT_DELTA(asymmetricDeadband(-0.1, lowWidth, highWidth), -0.05, epsilon);

    // Large positive input inside
    TS_ASSERT_DELTA(asymmetricDeadband(0.3, lowWidth, highWidth), 0.0, epsilon);

    // Large positive input outside
    TS_ASSERT_DELTA(asymmetricDeadband(1.0, lowWidth, highWidth), 0.5, epsilon);
  }

  void testAsymmetricDeadbandEqualWidths() {
    // When widths are equal, should behave like symmetric
    double width = 0.15;

    double asymResult = asymmetricDeadband(0.5, width, width);
    double symResult = symmetricDeadband(0.5, width);

    TS_ASSERT_DELTA(asymResult, symResult, epsilon);
  }

  /***************************************************************************
   * Offset Deadband Tests (centered at non-zero)
   ***************************************************************************/

  // Deadband centered at a non-zero value
  double offsetDeadband(double input, double center, double width) {
    double shifted = input - center;
    double result = symmetricDeadband(shifted, width);
    return result;
  }

  void testOffsetDeadbandPositiveCenter() {
    double center = 5.0;
    double width = 0.5;

    // Input at center should give zero
    TS_ASSERT_DELTA(offsetDeadband(5.0, center, width), 0.0, epsilon);

    // Input within deadband
    TS_ASSERT_DELTA(offsetDeadband(5.3, center, width), 0.0, epsilon);
    TS_ASSERT_DELTA(offsetDeadband(4.7, center, width), 0.0, epsilon);

    // Input outside deadband
    TS_ASSERT_DELTA(offsetDeadband(6.0, center, width), 0.5, epsilon);
    TS_ASSERT_DELTA(offsetDeadband(4.0, center, width), -0.5, epsilon);
  }

  void testOffsetDeadbandNegativeCenter() {
    double center = -3.0;
    double width = 0.2;

    // Input at center
    TS_ASSERT_DELTA(offsetDeadband(-3.0, center, width), 0.0, epsilon);

    // Input within deadband
    TS_ASSERT_DELTA(offsetDeadband(-2.9, center, width), 0.0, epsilon);
    TS_ASSERT_DELTA(offsetDeadband(-3.1, center, width), 0.0, epsilon);

    // Input outside deadband
    TS_ASSERT_DELTA(offsetDeadband(-2.5, center, width), 0.3, epsilon);
    TS_ASSERT_DELTA(offsetDeadband(-3.5, center, width), -0.3, epsilon);
  }

  /***************************************************************************
   * Width Variation Tests
   ***************************************************************************/

  void testVerySmallWidth() {
    double width = 1e-10;

    // Very close to pass-through behavior
    TS_ASSERT_DELTA(symmetricDeadband(0.5, width), 0.5, 1e-9);
    TS_ASSERT_DELTA(symmetricDeadband(-0.5, width), -0.5, 1e-9);
  }

  void testVeryLargeWidth() {
    double width = 1e6;

    // Almost everything within deadband
    TS_ASSERT_DELTA(symmetricDeadband(1000.0, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-1000.0, width), 0.0, epsilon);

    // Outside the huge deadband
    TS_ASSERT_DELTA(symmetricDeadband(2e6, width), 1e6, 1.0);
  }

  void testMultipleWidths() {
    double inputs[] = {0.3, 0.5, 0.7, 1.0};
    double widths[] = {0.1, 0.2, 0.3, 0.4};

    for (double input : inputs) {
      for (double width : widths) {
        double output = symmetricDeadband(input, width);
        if (input < width) {
          TS_ASSERT_DELTA(output, 0.0, epsilon);
        } else {
          TS_ASSERT_DELTA(output, input - width, epsilon);
        }
      }
    }
  }

  /***************************************************************************
   * Sequence Tests
   ***************************************************************************/

  void testIncreasingInputSequence() {
    double width = 0.2;
    double prev_output = -1.0;

    for (double input = 0.0; input <= 1.0; input += 0.1) {
      double output = symmetricDeadband(input, width);
      // Output should be non-decreasing for increasing input
      TS_ASSERT(output >= prev_output - epsilon);
      prev_output = output;
    }
  }

  void testDecreasingInputSequence() {
    double width = 0.2;
    double prev_output = 1.0;

    for (double input = 1.0; input >= 0.0; input -= 0.1) {
      double output = symmetricDeadband(input, width);
      // Output should be non-increasing for decreasing input
      TS_ASSERT(output <= prev_output + epsilon);
      prev_output = output;
    }
  }

  void testOscillatingInputs() {
    double width = 0.15;

    // Oscillate around zero within deadband - all should be zero
    double inputs[] = {0.1, -0.1, 0.05, -0.05, 0.14, -0.14};
    for (double input : inputs) {
      TS_ASSERT_DELTA(symmetricDeadband(input, width), 0.0, epsilon);
    }
  }

  void testOscillatingInputsOutsideDeadband() {
    double width = 0.1;

    // Oscillate outside deadband
    double inputs[] = {0.5, -0.5, 0.3, -0.3, 0.8, -0.8};
    double expected[] = {0.4, -0.4, 0.2, -0.2, 0.7, -0.7};

    for (int i = 0; i < 6; i++) {
      TS_ASSERT_DELTA(symmetricDeadband(inputs[i], width), expected[i], epsilon);
    }
  }

  /***************************************************************************
   * Gain Application Tests
   ***************************************************************************/

  void testDeadbandWithVaryingGains() {
    double width = 0.1;
    double input = 0.6;  // deadband output = 0.5
    double gains[] = {0.5, 1.0, 2.0, 5.0, 10.0};
    double expected[] = {0.25, 0.5, 1.0, 2.5, 5.0};

    for (int i = 0; i < 5; i++) {
      double output = symmetricDeadband(input, width) * gains[i];
      TS_ASSERT_DELTA(output, expected[i], epsilon);
    }
  }

  void testDeadbandWithNegativeGain() {
    double width = 0.1;
    double gain = -1.0;

    double output = symmetricDeadband(0.5, width) * gain;
    TS_ASSERT_DELTA(output, -0.4, epsilon);

    output = symmetricDeadband(-0.5, width) * gain;
    TS_ASSERT_DELTA(output, 0.4, epsilon);
  }

  void testDeadbandWithZeroGain() {
    double width = 0.1;
    double gain = 0.0;

    double output = symmetricDeadband(1.0, width) * gain;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  /***************************************************************************
   * Clipping Tests
   ***************************************************************************/

  void testDeadbandWithSymmetricClipping() {
    double width = 0.1;
    double clipLimit = 0.3;

    double inputs[] = {0.2, 0.5, 0.8, 1.0, -0.2, -0.5, -0.8, -1.0};
    double expected[] = {0.1, 0.3, 0.3, 0.3, -0.1, -0.3, -0.3, -0.3};

    for (int i = 0; i < 8; i++) {
      double db_out = symmetricDeadband(inputs[i], width);
      double clipped = std::clamp(db_out, -clipLimit, clipLimit);
      TS_ASSERT_DELTA(clipped, expected[i], epsilon);
    }
  }

  void testDeadbandWithAsymmetricClipping() {
    double width = 0.1;
    double clipMin = -0.2;
    double clipMax = 0.5;

    double inputs[] = {0.0, 0.5, 1.0, -0.2, -0.5};
    double expected[] = {0.0, 0.4, 0.5, -0.1, -0.2};

    for (int i = 0; i < 5; i++) {
      double db_out = symmetricDeadband(inputs[i], width);
      double clipped = std::clamp(db_out, clipMin, clipMax);
      TS_ASSERT_DELTA(clipped, expected[i], epsilon);
    }
  }

  /***************************************************************************
   * Combined Operations
   ***************************************************************************/

  void testDeadbandThenGainThenClip() {
    double width = 0.1;
    double gain = 2.0;
    double clipMax = 1.0;

    double input = 0.8;  // db_out = 0.7, gained = 1.4, clipped = 1.0
    double db_out = symmetricDeadband(input, width);
    double gained = db_out * gain;
    double clipped = std::min(gained, clipMax);

    TS_ASSERT_DELTA(clipped, clipMax, epsilon);
  }

  void testChainedDeadbands() {
    double width1 = 0.1;
    double width2 = 0.05;

    double input = 0.25;  // After first: 0.15, after second: 0.10
    double after_first = symmetricDeadband(input, width1);
    double after_second = symmetricDeadband(after_first, width2);

    TS_ASSERT_DELTA(after_first, 0.15, epsilon);
    TS_ASSERT_DELTA(after_second, 0.10, epsilon);
  }

  /***************************************************************************
   * Monotonicity Tests
   ***************************************************************************/

  void testMonotonicPositive() {
    double width = 0.1;

    // For positive inputs, larger input should give larger or equal output
    for (double x = 0.0; x < 2.0; x += 0.01) {
      double y1 = symmetricDeadband(x, width);
      double y2 = symmetricDeadband(x + 0.01, width);
      TS_ASSERT(y2 >= y1 - epsilon);
    }
  }

  void testMonotonicNegative() {
    double width = 0.1;

    // For negative inputs, more negative should give more negative output
    for (double x = 0.0; x > -2.0; x -= 0.01) {
      double y1 = symmetricDeadband(x, width);
      double y2 = symmetricDeadband(x - 0.01, width);
      TS_ASSERT(y2 <= y1 + epsilon);
    }
  }

  /***************************************************************************
   * Sign Preservation Tests
   ***************************************************************************/

  void testSignPreservationPositive() {
    double width = 0.1;

    // Positive inputs outside deadband should give positive output
    for (double x = 0.2; x <= 2.0; x += 0.1) {
      double y = symmetricDeadband(x, width);
      TS_ASSERT(y > 0);
    }
  }

  void testSignPreservationNegative() {
    double width = 0.1;

    // Negative inputs outside deadband should give negative output
    for (double x = -0.2; x >= -2.0; x -= 0.1) {
      double y = symmetricDeadband(x, width);
      TS_ASSERT(y < 0);
    }
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  void testPrecisionAtBoundary() {
    double width = 0.1;

    // Test many points very close to boundary
    for (double delta = 1e-15; delta < 1e-8; delta *= 10) {
      double justInside = symmetricDeadband(width - delta, width);
      double justOutside = symmetricDeadband(width + delta, width);

      TS_ASSERT_DELTA(justInside, 0.0, delta * 10);
      TS_ASSERT_DELTA(justOutside, delta, delta * 10);
    }
  }

  void testPrecisionSmallValues() {
    double width = 1e-10;

    // Very precise behavior at small scales
    double input = 5e-10;
    double output = symmetricDeadband(input, width);
    TS_ASSERT_DELTA(output, 4e-10, 1e-18);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testManyIterations() {
    double width = 0.1;

    // Run many iterations to check stability
    for (int i = 0; i < 1000; i++) {
      double input = (i % 100) / 50.0 - 1.0;  // Range [-1, 1]
      double output = symmetricDeadband(input, width);

      if (std::abs(input) < width) {
        TS_ASSERT_DELTA(output, 0.0, epsilon);
      } else {
        double expected = input > 0 ? input - width : input + width;
        TS_ASSERT_DELTA(output, expected, epsilon);
      }
    }
  }

  void testRandomLikeSequence() {
    double width = 0.15;

    // Pseudo-random sequence (not truly random for reproducibility)
    double values[] = {0.73, -0.42, 0.11, -0.89, 0.05, 0.67, -0.23, 0.91};

    for (double val : values) {
      double output = symmetricDeadband(val, width);
      if (std::abs(val) < width) {
        TS_ASSERT_DELTA(output, 0.0, epsilon);
      } else {
        double expected = val > 0 ? val - width : val + width;
        TS_ASSERT_DELTA(output, expected, epsilon);
      }
    }
  }

  /***************************************************************************
   * Flight Control Specific Tests
   ***************************************************************************/

  void testAileronDeadband() {
    // Typical aileron deadband to prevent roll oscillation
    double deadzone = 0.02;

    // Small stick movements ignored
    TS_ASSERT_DELTA(symmetricDeadband(0.01, deadzone), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.015, deadzone), 0.0, epsilon);

    // Normal deflection
    TS_ASSERT_DELTA(symmetricDeadband(0.5, deadzone), 0.48, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.5, deadzone), -0.48, epsilon);

    // Full deflection
    TS_ASSERT_DELTA(symmetricDeadband(1.0, deadzone), 0.98, epsilon);
  }

  void testElevatorDeadband() {
    double deadzone = 0.03;

    // Centered (trim)
    TS_ASSERT_DELTA(symmetricDeadband(0.0, deadzone), 0.0, epsilon);

    // Small pitch input
    TS_ASSERT_DELTA(symmetricDeadband(0.02, deadzone), 0.0, epsilon);

    // Normal pitch
    TS_ASSERT_DELTA(symmetricDeadband(0.3, deadzone), 0.27, epsilon);
  }

  void testRudderDeadband() {
    double deadzone = 0.05;

    // Yaw input filtering
    TS_ASSERT_DELTA(symmetricDeadband(0.04, deadzone), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(0.2, deadzone), 0.15, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.2, deadzone), -0.15, epsilon);
  }
};
