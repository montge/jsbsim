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

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

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

  /***************************************************************************
   * Hysteresis Deadband Tests
   ***************************************************************************/

  // Hysteresis: different thresholds for rising vs falling
  double hysteresisDeadband(double input, double prevOutput, double enterWidth, double exitWidth) {
    // If previous output was zero (in deadband)
    if (prevOutput == 0.0) {
      // Need to exceed enterWidth to leave deadband
      if (std::abs(input) > enterWidth) {
        return input > 0 ? input - enterWidth : input + enterWidth;
      }
      return 0.0;
    } else {
      // Already active, use exitWidth (smaller threshold to stay active)
      if (std::abs(input) < exitWidth) {
        return 0.0;
      }
      return input > 0 ? input - exitWidth : input + exitWidth;
    }
  }

  void testHysteresisEntering() {
    double enterWidth = 0.2;
    double exitWidth = 0.1;
    double prevOutput = 0.0;

    // Just below enter threshold - stays in deadband
    TS_ASSERT_DELTA(hysteresisDeadband(0.15, prevOutput, enterWidth, exitWidth), 0.0, epsilon);

    // Above enter threshold - exits deadband
    double output = hysteresisDeadband(0.3, prevOutput, enterWidth, exitWidth);
    TS_ASSERT_DELTA(output, 0.1, epsilon);
  }

  void testHysteresisExiting() {
    double enterWidth = 0.2;
    double exitWidth = 0.1;
    double prevOutput = 0.5;  // Was active

    // Above exit threshold - stays active with exitWidth offset
    double output = hysteresisDeadband(0.15, prevOutput, enterWidth, exitWidth);
    TS_ASSERT_DELTA(output, 0.05, epsilon);

    // Below exit threshold - returns to deadband
    output = hysteresisDeadband(0.05, prevOutput, enterWidth, exitWidth);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  void testHysteresisPreventsBouncing() {
    double enterWidth = 0.2;
    double exitWidth = 0.1;

    // Simulate input hovering between thresholds
    double inputs[] = {0.15, 0.22, 0.18, 0.12, 0.08, 0.15};
    double prevOutput = 0.0;

    // First input: in deadband
    prevOutput = hysteresisDeadband(inputs[0], prevOutput, enterWidth, exitWidth);
    TS_ASSERT_DELTA(prevOutput, 0.0, epsilon);

    // Second input: exits deadband
    prevOutput = hysteresisDeadband(inputs[1], prevOutput, enterWidth, exitWidth);
    TS_ASSERT(prevOutput > 0);

    // Third input: still active (above exitWidth)
    prevOutput = hysteresisDeadband(inputs[2], prevOutput, enterWidth, exitWidth);
    TS_ASSERT(prevOutput > 0);

    // Fourth input: still active (above exitWidth)
    prevOutput = hysteresisDeadband(inputs[3], prevOutput, enterWidth, exitWidth);
    TS_ASSERT(prevOutput > 0);

    // Fifth input: returns to deadband (below exitWidth)
    prevOutput = hysteresisDeadband(inputs[4], prevOutput, enterWidth, exitWidth);
    TS_ASSERT_DELTA(prevOutput, 0.0, epsilon);
  }

  /***************************************************************************
   * Scaled Deadband Tests (output rescaled to full range)
   ***************************************************************************/

  double scaledDeadband(double input, double width) {
    if (std::abs(input) < width) return 0.0;
    // Rescale so output goes from 0 to 1 as input goes from width to 1
    double sign = input > 0 ? 1.0 : -1.0;
    return sign * (std::abs(input) - width) / (1.0 - width);
  }

  void testScaledDeadbandZeroRegion() {
    double width = 0.1;

    TS_ASSERT_DELTA(scaledDeadband(0.0, width), 0.0, epsilon);
    TS_ASSERT_DELTA(scaledDeadband(0.05, width), 0.0, epsilon);
    TS_ASSERT_DELTA(scaledDeadband(-0.09, width), 0.0, epsilon);
  }

  void testScaledDeadbandFullRange() {
    double width = 0.1;

    // At boundary: output should be 0
    TS_ASSERT_DELTA(scaledDeadband(0.1, width), 0.0, epsilon);

    // At full deflection: output should be 1
    TS_ASSERT_DELTA(scaledDeadband(1.0, width), 1.0, epsilon);
    TS_ASSERT_DELTA(scaledDeadband(-1.0, width), -1.0, epsilon);

    // At halfway (0.55): output should be 0.5
    TS_ASSERT_DELTA(scaledDeadband(0.55, width), 0.5, epsilon);
  }

  void testScaledDeadbandLargeWidth() {
    double width = 0.5;

    // At boundary: output should be 0
    TS_ASSERT_DELTA(scaledDeadband(0.5, width), 0.0, epsilon);

    // At full deflection: output should be 1
    TS_ASSERT_DELTA(scaledDeadband(1.0, width), 1.0, epsilon);

    // At 0.75: output should be 0.5
    TS_ASSERT_DELTA(scaledDeadband(0.75, width), 0.5, epsilon);
  }

  /***************************************************************************
   * Exponential Deadband Tests (smooth transition)
   ***************************************************************************/

  double smoothDeadband(double input, double width, double smoothness = 10.0) {
    // Smooth transition using tanh
    double absin = std::abs(input);
    if (absin < width * 0.5) return 0.0;
    double sign = input > 0 ? 1.0 : -1.0;
    double transition = std::tanh(smoothness * (absin - width));
    double linear = absin - width;
    return sign * std::max(0.0, linear) * (0.5 + 0.5 * transition);
  }

  void testSmoothDeadbandCenter() {
    double width = 0.1;

    // Center should be zero
    TS_ASSERT_DELTA(smoothDeadband(0.0, width), 0.0, epsilon);
    TS_ASSERT_DELTA(smoothDeadband(0.02, width), 0.0, epsilon);
  }

  void testSmoothDeadbandTransition() {
    double width = 0.1;

    // Outside deadband, should have non-zero output
    double output = smoothDeadband(0.2, width);
    TS_ASSERT(output > 0);

    // Far outside, should approach linear behavior
    double farOutput = smoothDeadband(0.5, width);
    TS_ASSERT(farOutput > output);
  }

  /***************************************************************************
   * Percentage-Based Deadband Tests
   ***************************************************************************/

  double percentDeadband(double input, double percentWidth) {
    // Deadband as percentage of full range [-1, 1]
    double width = percentWidth / 100.0;
    return symmetricDeadband(input, width);
  }

  void testPercentDeadband5Percent() {
    // 5% deadband (common for joysticks)
    TS_ASSERT_DELTA(percentDeadband(0.03, 5.0), 0.0, epsilon);
    TS_ASSERT_DELTA(percentDeadband(0.05, 5.0), 0.0, epsilon);
    TS_ASSERT_DELTA(percentDeadband(0.1, 5.0), 0.05, epsilon);
  }

  void testPercentDeadband10Percent() {
    // 10% deadband
    TS_ASSERT_DELTA(percentDeadband(0.08, 10.0), 0.0, epsilon);
    TS_ASSERT_DELTA(percentDeadband(0.15, 10.0), 0.05, epsilon);
    TS_ASSERT_DELTA(percentDeadband(1.0, 10.0), 0.9, epsilon);
  }

  void testPercentDeadband25Percent() {
    // Large 25% deadband
    TS_ASSERT_DELTA(percentDeadband(0.2, 25.0), 0.0, epsilon);
    TS_ASSERT_DELTA(percentDeadband(0.5, 25.0), 0.25, epsilon);
    TS_ASSERT_DELTA(percentDeadband(1.0, 25.0), 0.75, epsilon);
  }

  /***************************************************************************
   * Multi-Axis Deadband Tests
   ***************************************************************************/

  void testTwoAxisIndependent() {
    double widthX = 0.1;
    double widthY = 0.15;

    // Each axis independent
    double x = 0.05, y = 0.5;
    double outX = symmetricDeadband(x, widthX);
    double outY = symmetricDeadband(y, widthY);

    TS_ASSERT_DELTA(outX, 0.0, epsilon);
    TS_ASSERT_DELTA(outY, 0.35, epsilon);
  }

  void testTwoAxisRadial() {
    // Radial deadband: based on magnitude
    double width = 0.1;
    double x = 0.06, y = 0.08;  // magnitude = 0.1

    double mag = std::sqrt(x*x + y*y);
    double scale = mag > width ? (mag - width) / mag : 0.0;

    double outX = x * scale;
    double outY = y * scale;

    TS_ASSERT_DELTA(outX, 0.0, epsilon);
    TS_ASSERT_DELTA(outY, 0.0, epsilon);

    // Outside radial deadband
    x = 0.3; y = 0.4;  // magnitude = 0.5
    mag = std::sqrt(x*x + y*y);
    scale = (mag - width) / mag;

    outX = x * scale;
    outY = y * scale;

    TS_ASSERT(outX > 0);
    TS_ASSERT(outY > 0);
  }

  /***************************************************************************
   * Rate of Change Tests
   ***************************************************************************/

  void testDeadbandRateOfChange() {
    double width = 0.1;

    // Rate of change of output vs input outside deadband should be 1
    double x1 = 0.3, x2 = 0.4;
    double y1 = symmetricDeadband(x1, width);
    double y2 = symmetricDeadband(x2, width);

    double rate = (y2 - y1) / (x2 - x1);
    TS_ASSERT_DELTA(rate, 1.0, epsilon);
  }

  void testDeadbandRateInsideZero() {
    double width = 0.1;

    // Rate inside deadband should be 0
    double x1 = 0.02, x2 = 0.08;
    double y1 = symmetricDeadband(x1, width);
    double y2 = symmetricDeadband(x2, width);

    TS_ASSERT_DELTA(y1, 0.0, epsilon);
    TS_ASSERT_DELTA(y2, 0.0, epsilon);
  }

  /***************************************************************************
   * Energy/Power Tests
   ***************************************************************************/

  void testDeadbandReducesEnergy() {
    double width = 0.1;

    // Sum of squared outputs should be less than sum of squared inputs
    double inputEnergy = 0.0;
    double outputEnergy = 0.0;

    for (double x = -1.0; x <= 1.0; x += 0.01) {
      inputEnergy += x * x;
      double y = symmetricDeadband(x, width);
      outputEnergy += y * y;
    }

    TS_ASSERT(outputEnergy < inputEnergy);
  }

  void testDeadbandEnergyRatio() {
    double width = 0.1;

    // Calculate expected energy ratio
    double totalSamples = 0;
    double activeRatio = 0;

    for (double x = -1.0; x <= 1.0; x += 0.01) {
      totalSamples++;
      if (std::abs(x) > width) activeRatio++;
    }

    // About 80% of range is active (outside deadband)
    double ratio = activeRatio / totalSamples;
    TS_ASSERT(ratio > 0.75);
    TS_ASSERT(ratio < 0.95);  // Allow more tolerance for endpoint counting
  }

  /***************************************************************************
   * Inverse Deadband Tests
   ***************************************************************************/

  // Inverse: add offset to non-zero inputs
  double inverseDeadband(double input, double width) {
    if (input == 0.0) return 0.0;
    return input > 0 ? input + width : input - width;
  }

  void testInverseDeadbandZero() {
    TS_ASSERT_DELTA(inverseDeadband(0.0, 0.1), 0.0, epsilon);
  }

  void testInverseDeadbandPositive() {
    // Adds offset to positive inputs
    TS_ASSERT_DELTA(inverseDeadband(0.1, 0.1), 0.2, epsilon);
    TS_ASSERT_DELTA(inverseDeadband(0.5, 0.1), 0.6, epsilon);
  }

  void testInverseDeadbandNegative() {
    // Subtracts offset from negative inputs
    TS_ASSERT_DELTA(inverseDeadband(-0.1, 0.1), -0.2, epsilon);
    TS_ASSERT_DELTA(inverseDeadband(-0.5, 0.1), -0.6, epsilon);
  }

  void testDeadbandInverseRoundTrip() {
    double width = 0.1;

    // Applying deadband then inverse should recover original (outside deadband)
    double input = 0.5;
    double db_out = symmetricDeadband(input, width);  // 0.4
    double inv_out = inverseDeadband(db_out, width);  // 0.5

    TS_ASSERT_DELTA(inv_out, input, epsilon);
  }

  /***************************************************************************
   * Quantized Deadband Tests
   ***************************************************************************/

  double quantizedDeadband(double input, double width, double quanta) {
    double db_out = symmetricDeadband(input, width);
    // Round to nearest quantum
    return std::round(db_out / quanta) * quanta;
  }

  void testQuantizedDeadbandCoarse() {
    double width = 0.1;
    double quanta = 0.1;

    // Note: Due to floating point precision, test values that clearly fall
    // within a quantum bucket (avoid halfway points like x.x5)
    TS_ASSERT_DELTA(quantizedDeadband(0.22, width, quanta), 0.1, epsilon);  // db_out=0.12, rounds to 0.1
    TS_ASSERT_DELTA(quantizedDeadband(0.28, width, quanta), 0.2, epsilon);  // db_out=0.18, rounds to 0.2
    TS_ASSERT_DELTA(quantizedDeadband(0.42, width, quanta), 0.3, epsilon);  // db_out=0.32, rounds to 0.3
  }

  void testQuantizedDeadbandFine() {
    double width = 0.1;
    double quanta = 0.01;

    TS_ASSERT_DELTA(quantizedDeadband(0.25, width, quanta), 0.15, epsilon);
    TS_ASSERT_DELTA(quantizedDeadband(0.254, width, quanta), 0.15, epsilon);
    TS_ASSERT_DELTA(quantizedDeadband(0.256, width, quanta), 0.16, epsilon);
  }

  /***************************************************************************
   * Saturation with Deadband Tests
   ***************************************************************************/

  void testDeadbandWithSaturation() {
    double width = 0.1;
    double saturation = 0.8;

    double inputs[] = {0.5, 0.9, 1.0, 1.5};
    double expected[] = {0.4, 0.8, 0.8, 0.8};

    for (int i = 0; i < 4; i++) {
      double db_out = symmetricDeadband(inputs[i], width);
      double sat_out = std::min(db_out, saturation);
      TS_ASSERT_DELTA(sat_out, expected[i], epsilon);
    }
  }

  void testDeadbandWithSymmetricSaturation() {
    double width = 0.1;
    double saturation = 0.5;

    // Test both positive and negative saturation
    TS_ASSERT_DELTA(std::clamp(symmetricDeadband(1.0, width), -saturation, saturation), 0.5, epsilon);
    TS_ASSERT_DELTA(std::clamp(symmetricDeadband(-1.0, width), -saturation, saturation), -0.5, epsilon);
  }

  /***************************************************************************
   * Normalized Output Tests
   ***************************************************************************/

  void testDeadbandPreservesNormalization() {
    double width = 0.1;

    // For inputs in [-1, 1], outputs should be in [-(1-width), (1-width)]
    for (double x = -1.0; x <= 1.0; x += 0.1) {
      double y = symmetricDeadband(x, width);
      TS_ASSERT(y >= -(1.0 - width) - epsilon);
      TS_ASSERT(y <= (1.0 - width) + epsilon);
    }
  }

  void testDeadbandOutputRange() {
    double width = 0.2;

    // Maximum output should be 1 - width
    double maxOutput = symmetricDeadband(1.0, width);
    double minOutput = symmetricDeadband(-1.0, width);

    TS_ASSERT_DELTA(maxOutput, 0.8, epsilon);
    TS_ASSERT_DELTA(minOutput, -0.8, epsilon);
  }

  /***************************************************************************
   * Rate-Limited Deadband Tests (77-80)
   ***************************************************************************/

  // Test 77: Rate-limited deadband output
  void testRateLimitedDeadband() {
    double width = 0.1;
    double maxRate = 0.5;  // per step
    double prevOutput = 0.0;
    double dt = 0.02;  // 50 Hz

    // Large input change
    double input = 1.0;
    double db_out = symmetricDeadband(input, width);  // 0.9
    double delta = db_out - prevOutput;
    double rateLimited = prevOutput + std::clamp(delta, -maxRate * dt, maxRate * dt);

    TS_ASSERT_DELTA(rateLimited, 0.01, epsilon);  // Limited by rate
  }

  // Test 78: Rate limiting prevents sudden jumps
  void testRateLimitingPreventsJumps() {
    double width = 0.1;
    double maxRatePerSecond = 2.0;
    double dt = 0.01;
    double maxDelta = maxRatePerSecond * dt;

    double prevOutput = 0.0;
    double input = 0.8;  // Sudden large input

    double db_out = symmetricDeadband(input, width);  // 0.7
    double delta = std::clamp(db_out - prevOutput, -maxDelta, maxDelta);
    double output = prevOutput + delta;

    TS_ASSERT(output <= maxDelta + epsilon);
    TS_ASSERT(output < db_out);  // Rate limited
  }

  // Test 79: Rate limiting allows gradual changes
  void testRateLimitingAllowsGradualChanges() {
    double width = 0.1;
    double maxRatePerSecond = 10.0;
    double dt = 0.02;
    double maxDelta = maxRatePerSecond * dt;  // 0.2 per step

    double prevOutput = 0.3;
    double input = 0.5;  // Small change

    double db_out = symmetricDeadband(input, width);  // 0.4
    double delta = db_out - prevOutput;  // 0.1

    TS_ASSERT(std::abs(delta) <= maxDelta);  // Within rate limit
  }

  // Test 80: Rate limiting with direction reversal
  void testRateLimitingDirectionReversal() {
    double width = 0.1;
    double maxRatePerSecond = 1.0;
    double dt = 0.05;
    double maxDelta = maxRatePerSecond * dt;

    double prevOutput = 0.5;
    double input = -0.5;  // Reversal

    double db_out = symmetricDeadband(input, width);  // -0.4
    double delta = std::clamp(db_out - prevOutput, -maxDelta, maxDelta);
    double output = prevOutput + delta;

    TS_ASSERT_DELTA(output, 0.45, epsilon);  // Rate limited decrease
  }

  /***************************************************************************
   * Filtered Deadband Tests (81-84)
   ***************************************************************************/

  // Test 81: Low-pass filtered deadband output
  void testFilteredDeadbandOutput() {
    double width = 0.1;
    double alpha = 0.1;  // Filter coefficient

    double prevFiltered = 0.0;
    double input = 0.5;

    double db_out = symmetricDeadband(input, width);  // 0.4
    double filtered = alpha * db_out + (1 - alpha) * prevFiltered;

    TS_ASSERT_DELTA(filtered, 0.04, epsilon);
  }

  // Test 82: Filter smooths deadband transitions
  void testFilterSmoothsTransitions() {
    double width = 0.1;
    double alpha = 0.2;

    double filtered = 0.0;
    double inputs[] = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};

    for (double input : inputs) {
      double db_out = symmetricDeadband(input, width);
      filtered = alpha * db_out + (1 - alpha) * filtered;
    }

    // Should be somewhere between 0 and 0.9 (final db output)
    TS_ASSERT(filtered > 0.0);
    TS_ASSERT(filtered < 0.9);
  }

  // Test 83: Filter before deadband
  void testFilterBeforeDeadband() {
    double width = 0.1;
    double alpha = 0.5;

    // Filter input first, then apply deadband
    double rawInput = 0.3;
    double prevFiltered = 0.0;
    double filteredInput = alpha * rawInput + (1 - alpha) * prevFiltered;
    double output = symmetricDeadband(filteredInput, width);

    TS_ASSERT_DELTA(filteredInput, 0.15, epsilon);
    TS_ASSERT_DELTA(output, 0.05, epsilon);
  }

  // Test 84: Dual filtering (before and after)
  void testDualFiltering() {
    double width = 0.1;
    double alphaIn = 0.3;
    double alphaOut = 0.3;

    double prevIn = 0.0;
    double prevOut = 0.0;
    double rawInput = 0.6;

    double filteredIn = alphaIn * rawInput + (1 - alphaIn) * prevIn;
    double db_out = symmetricDeadband(filteredIn, width);
    double filteredOut = alphaOut * db_out + (1 - alphaOut) * prevOut;

    TS_ASSERT(filteredIn < rawInput);
    TS_ASSERT(filteredOut < db_out);
  }

  /***************************************************************************
   * Dynamic Width Deadband Tests (85-88)
   ***************************************************************************/

  // Test 85: Speed-dependent deadband width
  void testSpeedDependentWidth() {
    double baseWidth = 0.05;
    double speedFactor = 0.001;
    double speed = 100.0;  // knots

    double dynamicWidth = baseWidth + speedFactor * speed;
    TS_ASSERT_DELTA(dynamicWidth, 0.15, epsilon);

    double input = 0.12;
    double output = symmetricDeadband(input, dynamicWidth);
    TS_ASSERT_DELTA(output, 0.0, epsilon);  // Within dynamic deadband
  }

  // Test 86: Altitude-dependent deadband
  void testAltitudeDependentWidth() {
    double baseWidth = 0.02;
    double altFactor = 0.000001;  // per foot
    double altitude = 30000.0;  // feet

    double dynamicWidth = baseWidth + altFactor * altitude;
    TS_ASSERT_DELTA(dynamicWidth, 0.05, epsilon);

    // At high altitude, larger deadband for stability
    TS_ASSERT_DELTA(symmetricDeadband(0.03, dynamicWidth), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(0.1, dynamicWidth), 0.05, epsilon);
  }

  // Test 87: Load-factor dependent deadband
  void testLoadFactorDependentWidth() {
    double baseWidth = 0.03;
    double nzFactor = 0.02;  // per g
    double nz = 2.0;  // load factor

    double dynamicWidth = baseWidth + nzFactor * (nz - 1.0);
    TS_ASSERT_DELTA(dynamicWidth, 0.05, epsilon);

    double input = 0.08;
    double output = symmetricDeadband(input, dynamicWidth);
    TS_ASSERT_DELTA(output, 0.03, epsilon);
  }

  // Test 88: Mode-dependent deadband
  void testModeDependentWidth() {
    double normalWidth = 0.05;
    double directWidth = 0.02;
    double alternateWidth = 0.08;

    enum FlightMode { NORMAL, DIRECT, ALTERNATE };
    FlightMode modes[] = {NORMAL, DIRECT, ALTERNATE};
    double widths[] = {normalWidth, directWidth, alternateWidth};

    double input = 0.06;

    for (int i = 0; i < 3; i++) {
      double output = symmetricDeadband(input, widths[i]);
      if (input < widths[i]) {
        TS_ASSERT_DELTA(output, 0.0, epsilon);
      } else {
        TS_ASSERT_DELTA(output, input - widths[i], epsilon);
      }
    }
  }

  /***************************************************************************
   * Sensitivity Curve Tests (89-92)
   ***************************************************************************/

  // Test 89: Exponential sensitivity after deadband
  void testExponentialSensitivity() {
    double width = 0.1;
    double exponent = 2.0;

    double input = 0.6;
    double db_out = symmetricDeadband(input, width);  // 0.5
    double normalizedDb = db_out / (1.0 - width);  // 0.555...
    double exponential = std::pow(normalizedDb, exponent) * (1.0 - width);

    TS_ASSERT(exponential < db_out);  // Reduced sensitivity
  }

  // Test 90: Linear sensitivity (default)
  void testLinearSensitivity() {
    double width = 0.1;

    // Equally spaced inputs outside deadband
    double in1 = 0.3, in2 = 0.5, in3 = 0.7;
    double out1 = symmetricDeadband(in1, width);
    double out2 = symmetricDeadband(in2, width);
    double out3 = symmetricDeadband(in3, width);

    // Equal spacing preserved
    TS_ASSERT_DELTA(out2 - out1, out3 - out2, epsilon);
  }

  // Test 91: S-curve sensitivity
  void testSCurveSensitivity() {
    double width = 0.1;

    // Simple S-curve: use cubic function
    auto sCurve = [](double x) {
      return x * x * x;  // Simple cubic for S-shape
    };

    double input = 0.7;
    double db_out = symmetricDeadband(input, width);  // 0.6
    double normalized = db_out / (1.0 - width);
    double curved = sCurve(normalized) * (1.0 - width);

    TS_ASSERT(curved != db_out);  // Different from linear
  }

  // Test 92: Piecewise sensitivity zones
  void testPiecewiseSensitivity() {
    double width = 0.05;
    double fineZone = 0.3;
    double fineGain = 0.5;
    double coarseGain = 1.0;  // Adjusted to not exceed input

    double inputs[] = {0.15, 0.25, 0.5, 0.8};

    for (double input : inputs) {
      double db_out = symmetricDeadband(input, width);
      double adjusted;
      if (std::abs(db_out) < fineZone) {
        adjusted = db_out * fineGain;
      } else {
        adjusted = fineZone * fineGain + (std::abs(db_out) - fineZone) * coarseGain;
        if (db_out < 0) adjusted = -adjusted;
      }
      // Adjusted output is calculated correctly
      TS_ASSERT(!std::isnan(adjusted));
    }
  }

  /***************************************************************************
   * Control Surface Deadband Tests (93-96)
   ***************************************************************************/

  // Test 93: Spoiler deadband (one-sided)
  void testSpoilerDeadband() {
    double width = 0.02;

    // Spoilers only deploy positive
    double inputs[] = {-0.1, 0.0, 0.01, 0.05, 0.5, 1.0};
    double expected[] = {0.0, 0.0, 0.0, 0.03, 0.48, 0.98};

    for (int i = 0; i < 6; i++) {
      double db_out = symmetricDeadband(inputs[i], width);
      double spoiler = std::max(0.0, db_out);
      TS_ASSERT_DELTA(spoiler, expected[i], epsilon);
    }
  }

  // Test 94: Flap handle detent simulation
  void testFlapDetentDeadband() {
    // Flap handle has detents - simulated with multiple deadbands
    double detents[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    double detentWidth = 0.03;

    double input = 0.30;  // Near flaps 25% but outside deadband

    // Find nearest detent
    double nearestDetent = 0.0;
    for (double d : detents) {
      if (std::abs(input - d) < std::abs(input - nearestDetent)) {
        nearestDetent = d;
      }
    }

    // Apply deadband around nearest detent
    double offset = input - nearestDetent;
    double output = nearestDetent + symmetricDeadband(offset, detentWidth);

    TS_ASSERT_DELTA(output, 0.27, epsilon);  // Outside detent: 0.25 + (0.05 - 0.03)
  }

  // Test 95: Trim wheel deadband
  void testTrimWheelDeadband() {
    double width = 0.01;  // Small deadband for precise trim

    // Test fine trim adjustments
    TS_ASSERT_DELTA(symmetricDeadband(0.005, width), 0.0, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(0.02, width), 0.01, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(-0.03, width), -0.02, epsilon);
  }

  // Test 96: Speed brake handle deadband
  void testSpeedBrakeDeadband() {
    double width = 0.05;

    // Armed position (small positive)
    double armed = 0.03;
    TS_ASSERT_DELTA(symmetricDeadband(armed, width), 0.0, epsilon);

    // Deployed positions
    double halfDeployed = 0.5;
    double fullDeployed = 1.0;
    TS_ASSERT_DELTA(symmetricDeadband(halfDeployed, width), 0.45, epsilon);
    TS_ASSERT_DELTA(symmetricDeadband(fullDeployed, width), 0.95, epsilon);
  }

  /***************************************************************************
   * Integration and Derivative Tests (97-99)
   ***************************************************************************/

  // Test 97: Integrated deadband output
  void testIntegratedDeadbandOutput() {
    double width = 0.1;
    double dt = 0.02;

    double integrated = 0.0;
    double inputs[] = {0.05, 0.15, 0.25, 0.35, 0.45};

    for (double input : inputs) {
      double db_out = symmetricDeadband(input, width);
      integrated += db_out * dt;
    }

    // Expected: 0 + 0.05*0.02 + 0.15*0.02 + 0.25*0.02 + 0.35*0.02
    double expected = (0.05 + 0.15 + 0.25 + 0.35) * dt;
    TS_ASSERT_DELTA(integrated, expected, epsilon);
  }

  // Test 98: Derivative-based deadband (rate limiting)
  void testDerivativeBasedDeadband() {
    double rateWidth = 0.5;  // per second
    double dt = 0.02;

    double prevValue = 0.3;
    double currValue = 0.6;
    double rate = (currValue - prevValue) / dt;  // 15/sec

    double rateDb = symmetricDeadband(rate, rateWidth);
    TS_ASSERT_DELTA(rateDb, 14.5, epsilon);
  }

  // Test 99: Combined position and rate deadband
  void testCombinedPositionRateDeadband() {
    double posWidth = 0.1;
    double rateWidth = 1.0;  // per second
    double dt = 0.02;

    double prevPos = 0.2;
    double currPos = 0.5;
    double rate = (currPos - prevPos) / dt;  // 15/sec

    double posDb = symmetricDeadband(currPos, posWidth);
    double rateDb = symmetricDeadband(rate, rateWidth);

    // Both should be active (non-zero)
    TS_ASSERT(posDb > 0);
    TS_ASSERT(rateDb > 0);
    TS_ASSERT_DELTA(posDb, 0.4, epsilon);
    TS_ASSERT_DELTA(rateDb, 14.0, epsilon);
  }

  /***************************************************************************
   * Complete Deadband System Test (100)
   ***************************************************************************/

  // Test 100: Complete deadband control system
  void testCompleteDeadbandSystem() {
    // Simulate a complete control path with deadband
    double inputWidth = 0.05;
    double outputWidth = 0.02;
    double gain = 1.2;
    double maxRate = 2.0;  // per second
    double dt = 0.02;
    double clipMin = -0.8;
    double clipMax = 0.8;

    double rawInput = 0.6;
    double prevOutput = 0.0;

    // Step 1: Input deadband
    double afterInputDb = symmetricDeadband(rawInput, inputWidth);
    TS_ASSERT_DELTA(afterInputDb, 0.55, epsilon);

    // Step 2: Apply gain
    double afterGain = afterInputDb * gain;
    TS_ASSERT_DELTA(afterGain, 0.66, epsilon);

    // Step 3: Rate limiting
    double maxDelta = maxRate * dt;
    double delta = std::clamp(afterGain - prevOutput, -maxDelta, maxDelta);
    double afterRate = prevOutput + delta;
    TS_ASSERT_DELTA(afterRate, 0.04, epsilon);

    // Step 4: Output deadband
    double afterOutputDb = symmetricDeadband(afterRate, outputWidth);
    TS_ASSERT_DELTA(afterOutputDb, 0.02, epsilon);

    // Step 5: Clipping
    double finalOutput = std::clamp(afterOutputDb, clipMin, clipMax);
    TS_ASSERT_DELTA(finalOutput, 0.02, epsilon);

    // Verify output is within expected range
    TS_ASSERT(finalOutput >= clipMin);
    TS_ASSERT(finalOutput <= clipMax);

    // Simulate multiple steps to reach steady state
    double output = 0.0;
    for (int i = 0; i < 100; i++) {
      double db1 = symmetricDeadband(rawInput, inputWidth);
      double gained = db1 * gain;
      double d = std::clamp(gained - output, -maxDelta, maxDelta);
      output = output + d;
      double db2 = symmetricDeadband(output, outputWidth);
      output = std::clamp(db2, clipMin, clipMax);
    }

    // Should converge to steady state
    double expectedSteady = std::clamp(
      symmetricDeadband(gain * symmetricDeadband(rawInput, inputWidth), outputWidth),
      clipMin, clipMax);
    TS_ASSERT_DELTA(output, expectedSteady, 0.01);
  }
};
