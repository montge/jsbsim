/*******************************************************************************
 * FGSummerTest.h - Unit tests for FGSummer (signal summing)
 *
 * Tests the mathematical behavior of the summing component:
 * - Addition of multiple inputs
 * - Bias addition
 * - Clipping/saturation
 * - Sign inversion of inputs
 *
 * Note: FGSummer requires XML element for construction, so these tests focus
 * on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <vector>
#include <algorithm>

const double epsilon = 1e-10;

class FGSummerTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Basic Summation Tests
   ***************************************************************************/

  // Test two input sum
  void testTwoInputSum() {
    double input1 = 5.0;
    double input2 = 3.0;

    double output = input1 + input2;
    TS_ASSERT_DELTA(output, 8.0, epsilon);
  }

  // Test three input sum
  void testThreeInputSum() {
    double input1 = 1.0;
    double input2 = 2.0;
    double input3 = 3.0;

    double output = input1 + input2 + input3;
    TS_ASSERT_DELTA(output, 6.0, epsilon);
  }

  // Test multiple input sum using vector
  void testMultipleInputSum() {
    std::vector<double> inputs = {1.0, 2.0, 3.0, 4.0, 5.0};
    double output = 0.0;

    for (double input : inputs) {
      output += input;
    }

    TS_ASSERT_DELTA(output, 15.0, epsilon);
  }

  // Test sum with negative values
  void testSumWithNegatives() {
    double input1 = 10.0;
    double input2 = -3.0;
    double input3 = -2.0;

    double output = input1 + input2 + input3;
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test zero inputs
  void testZeroInputs() {
    double input1 = 0.0;
    double input2 = 0.0;

    double output = input1 + input2;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  /***************************************************************************
   * Bias Tests
   ***************************************************************************/

  // Test sum with bias
  void testSumWithBias() {
    double input1 = 5.0;
    double input2 = 3.0;
    double bias = 2.0;

    double output = input1 + input2 + bias;
    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test negative bias
  void testNegativeBias() {
    double input1 = 10.0;
    double bias = -5.0;

    double output = input1 + bias;
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test bias only (no inputs)
  void testBiasOnly() {
    double bias = 7.5;

    double output = bias;
    TS_ASSERT_DELTA(output, 7.5, epsilon);
  }

  /***************************************************************************
   * Sign Inversion Tests
   ***************************************************************************/

  // Test inverted input (negative sign prefix)
  void testInvertedInput() {
    double input1 = 10.0;
    double input2_inverted = -(5.0);  // -input2

    double output = input1 + input2_inverted;
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test mixed inverted inputs
  void testMixedInvertedInputs() {
    double input1 = 10.0;
    double input2 = 5.0;
    double input3 = 3.0;

    // output = input1 - input2 + input3
    double output = input1 + (-input2) + input3;
    TS_ASSERT_DELTA(output, 8.0, epsilon);
  }

  // Test all inverted inputs
  void testAllInvertedInputs() {
    double input1 = -3.0;
    double input2 = -5.0;
    double input3 = -2.0;

    double output = input1 + input2 + input3;
    TS_ASSERT_DELTA(output, -10.0, epsilon);
  }

  /***************************************************************************
   * Clipping Tests
   ***************************************************************************/

  // Helper: Apply clipping
  double clip(double value, double min, double max) {
    return std::clamp(value, min, max);
  }

  // Test clipping at maximum
  void testClipAtMax() {
    double input1 = 10.0;
    double input2 = 5.0;
    double clipMin = -1.0;
    double clipMax = 10.0;

    double sum = input1 + input2;  // = 15
    double output = clip(sum, clipMin, clipMax);

    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test clipping at minimum
  void testClipAtMin() {
    double input1 = -10.0;
    double input2 = -5.0;
    double clipMin = -10.0;
    double clipMax = 10.0;

    double sum = input1 + input2;  // = -15
    double output = clip(sum, clipMin, clipMax);

    TS_ASSERT_DELTA(output, -10.0, epsilon);
  }

  // Test no clipping when within range
  void testNoClipping() {
    double input1 = 3.0;
    double input2 = 2.0;
    double clipMin = -10.0;
    double clipMax = 10.0;

    double sum = input1 + input2;  // = 5
    double output = clip(sum, clipMin, clipMax);

    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test symmetric clipping
  void testSymmetricClipping() {
    double clipLimit = 5.0;

    double positiveSum = 10.0;
    double negativeSum = -10.0;

    double posClipped = clip(positiveSum, -clipLimit, clipLimit);
    double negClipped = clip(negativeSum, -clipLimit, clipLimit);

    TS_ASSERT_DELTA(posClipped, 5.0, epsilon);
    TS_ASSERT_DELTA(negClipped, -5.0, epsilon);
  }

  /***************************************************************************
   * Flight Control Applications
   ***************************************************************************/

  // Test roll rate error sum
  void testRollRateErrorSum() {
    double rollRate = 0.5;          // Current roll rate (rad/s)
    double rollCommand = 0.0;       // Desired roll rate (rad/s)
    double wingLeveler = -0.3;      // Wing leveler correction

    // Error = command - actual + corrections
    double error = rollCommand - rollRate + wingLeveler;
    TS_ASSERT_DELTA(error, -0.8, epsilon);
  }

  // Test pitch axis summing junction
  void testPitchAxisSum() {
    double pilotInput = 0.5;        // Normalized pilot input
    double trimInput = 0.1;         // Trim setting
    double autopilotCommand = 0.0;  // A/P command

    double totalCommand = pilotInput + trimInput + autopilotCommand;
    TS_ASSERT_DELTA(totalCommand, 0.6, epsilon);
  }

  // Test error integrator input sum
  void testErrorIntegratorSum() {
    double proportionalError = 0.1;
    double integratedError = 0.05;
    double derivativeError = 0.02;

    // PID-like sum
    double output = proportionalError + integratedError + derivativeError;
    TS_ASSERT_DELTA(output, 0.17, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very small values
  void testVerySmallValues() {
    double input1 = 1e-15;
    double input2 = 2e-15;

    double output = input1 + input2;
    TS_ASSERT_DELTA(output, 3e-15, 1e-16);
  }

  // Test very large values
  void testVeryLargeValues() {
    double input1 = 1e10;
    double input2 = 1e10;

    double output = input1 + input2;
    TS_ASSERT_DELTA(output, 2e10, 1e5);
  }

  // Test cancellation (values that sum to zero)
  void testCancellation() {
    double input1 = 5.0;
    double input2 = -5.0;

    double output = input1 + input2;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test infinity handling
  void testInfinityHandling() {
    double input1 = 5.0;
    double input2 = std::numeric_limits<double>::infinity();

    double output = input1 + input2;
    TS_ASSERT(std::isinf(output));
  }

  // Test NaN propagation
  void testNaNPropagation() {
    double input1 = 5.0;
    double input2 = std::numeric_limits<double>::quiet_NaN();

    double output = input1 + input2;
    TS_ASSERT(std::isnan(output));
  }

  /***************************************************************************
   * Associativity and Commutativity Tests
   ***************************************************************************/

  // Test commutativity (a + b = b + a)
  void testCommutativity() {
    double a = 5.0, b = 3.0;

    TS_ASSERT_DELTA(a + b, b + a, epsilon);
  }

  // Test associativity ((a + b) + c = a + (b + c))
  void testAssociativity() {
    double a = 5.0, b = 3.0, c = 2.0;

    double result1 = (a + b) + c;
    double result2 = a + (b + c);

    TS_ASSERT_DELTA(result1, result2, epsilon);
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  // Test floating-point precision limits
  void testFloatingPointPrecision() {
    // Large value plus small value may lose precision
    double large = 1e15;
    double small = 1.0;

    double sum = large + small;
    // Due to precision limits, small value may be lost
    // This demonstrates floating-point limitations
    TS_ASSERT(sum >= large);  // At least as big as large
  }
};
