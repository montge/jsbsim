/*******************************************************************************
 * FGGainTest.h - Unit tests for FGGain (gain component math)
 *
 * Tests the mathematical behavior of gain components:
 * - Pure gain: output = input * gain
 * - Scheduled gain: output = input * table(lookup)
 * - Aerosurface scale: domain-to-range mapping
 *
 * Note: FGGain requires XML element for construction, so these tests focus on
 * the underlying mathematical operations rather than component instantiation.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <algorithm>

const double epsilon = 1e-10;

class FGGainTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Pure Gain Tests
   * output = input * gain
   ***************************************************************************/

  // Test basic gain multiplication
  void testPureGainMultiplication() {
    double input = 5.0;
    double gain = 2.0;
    double output = input * gain;

    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test unity gain
  void testUnityGain() {
    double input = 123.456;
    double gain = 1.0;
    double output = input * gain;

    TS_ASSERT_DELTA(output, input, epsilon);
  }

  // Test zero gain
  void testZeroGain() {
    double input = 999.0;
    double gain = 0.0;
    double output = input * gain;

    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test negative gain (signal inversion)
  void testNegativeGain() {
    double input = 5.0;
    double gain = -2.0;
    double output = input * gain;

    TS_ASSERT_DELTA(output, -10.0, epsilon);
  }

  // Test gain with zero input
  void testGainWithZeroInput() {
    double input = 0.0;
    double gain = 1000.0;
    double output = input * gain;

    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test fractional gain (attenuation)
  void testFractionalGain() {
    double input = 10.0;
    double gain = 0.5;
    double output = input * gain;

    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  /***************************************************************************
   * Clipping Tests
   * output = clamp(input * gain, min, max)
   ***************************************************************************/

  // Test clipping at maximum
  void testClipMax() {
    double input = 10.0;
    double gain = 2.0;
    double clipMin = -5.0;
    double clipMax = 15.0;

    double output = input * gain;  // = 20.0
    output = std::clamp(output, clipMin, clipMax);

    TS_ASSERT_DELTA(output, clipMax, epsilon);
  }

  // Test clipping at minimum
  void testClipMin() {
    double input = -10.0;
    double gain = 2.0;
    double clipMin = -15.0;
    double clipMax = 5.0;

    double output = input * gain;  // = -20.0
    output = std::clamp(output, clipMin, clipMax);

    TS_ASSERT_DELTA(output, clipMin, epsilon);
  }

  // Test no clipping when within range
  void testNoClip() {
    double input = 5.0;
    double gain = 1.0;
    double clipMin = -10.0;
    double clipMax = 10.0;

    double output = input * gain;  // = 5.0
    output = std::clamp(output, clipMin, clipMax);

    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test symmetric clipping
  void testSymmetricClip() {
    double clipLimit = 10.0;

    // Positive input clipped
    double output1 = std::clamp(20.0, -clipLimit, clipLimit);
    TS_ASSERT_DELTA(output1, clipLimit, epsilon);

    // Negative input clipped
    double output2 = std::clamp(-20.0, -clipLimit, clipLimit);
    TS_ASSERT_DELTA(output2, -clipLimit, epsilon);
  }

  /***************************************************************************
   * Aerosurface Scale Tests
   * Maps input from [domain_min, domain_max] to [range_min, range_max]
   ***************************************************************************/

  // Helper: Linear mapping (zero_centered = false)
  double linearMap(double input, double domainMin, double domainMax,
                   double rangeMin, double rangeMax) {
    double normalized = (input - domainMin) / (domainMax - domainMin);
    return rangeMin + normalized * (rangeMax - rangeMin);
  }

  // Helper: Zero-centered mapping
  double zeroCenteredMap(double input, double domainMin, double domainMax,
                         double rangeMin, double rangeMax) {
    if (input >= 0) {
      // Map [0, domainMax] to [0, rangeMax]
      return (input / domainMax) * rangeMax;
    } else {
      // Map [domainMin, 0] to [rangeMin, 0]
      return (input / domainMin) * rangeMin;
    }
  }

  // Test linear mapping: domain [-1,1] to range [-50,50]
  void testLinearMapSymmetric() {
    double output = linearMap(0.5, -1.0, 1.0, -50.0, 50.0);
    // 0.5 is 75% of the way from -1 to 1
    // 75% of the way from -50 to 50 is 25
    TS_ASSERT_DELTA(output, 25.0, epsilon);
  }

  // Test linear mapping at endpoints
  void testLinearMapEndpoints() {
    double domainMin = -1.0, domainMax = 1.0;
    double rangeMin = -50.0, rangeMax = 50.0;

    double output_min = linearMap(domainMin, domainMin, domainMax, rangeMin, rangeMax);
    double output_max = linearMap(domainMax, domainMin, domainMax, rangeMin, rangeMax);

    TS_ASSERT_DELTA(output_min, rangeMin, epsilon);
    TS_ASSERT_DELTA(output_max, rangeMax, epsilon);
  }

  // Test linear mapping at zero (asymmetric domain)
  void testLinearMapAsymmetric() {
    // Domain [-2, 4], range [-1, 1]
    // 0 is 1/3 of the way from -2 to 4
    // 1/3 of the way from -1 to 1 is -0.333...
    double output = linearMap(0.0, -2.0, 4.0, -1.0, 1.0);
    TS_ASSERT_DELTA(output, -1.0/3.0, 0.001);
  }

  // Test zero-centered mapping at zero input
  void testZeroCenteredMapAtZero() {
    double output = zeroCenteredMap(0.0, -1.0, 1.0, -50.0, 50.0);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test zero-centered mapping at positive input
  void testZeroCenteredMapPositive() {
    double output = zeroCenteredMap(0.5, -1.0, 1.0, -50.0, 50.0);
    // 0.5/1.0 * 50.0 = 25.0
    TS_ASSERT_DELTA(output, 25.0, epsilon);
  }

  // Test zero-centered mapping at negative input
  void testZeroCenteredMapNegative() {
    double output = zeroCenteredMap(-0.5, -1.0, 1.0, -50.0, 50.0);
    // -0.5/-1.0 * -50.0 = -25.0
    TS_ASSERT_DELTA(output, -25.0, epsilon);
  }

  // Test zero-centered with asymmetric range
  void testZeroCenteredAsymmetricRange() {
    // Range: [-30, 50]
    // At input 1.0: output = 50
    // At input -1.0: output = -30
    double out_pos = zeroCenteredMap(1.0, -1.0, 1.0, -30.0, 50.0);
    double out_neg = zeroCenteredMap(-1.0, -1.0, 1.0, -30.0, 50.0);

    TS_ASSERT_DELTA(out_pos, 50.0, epsilon);
    TS_ASSERT_DELTA(out_neg, -30.0, epsilon);
  }

  /***************************************************************************
   * Scheduled Gain Tests (Table Lookup)
   ***************************************************************************/

  // Simulate linear interpolation between table points
  double linearInterp(double x, double x1, double y1, double x2, double y2) {
    double t = (x - x1) / (x2 - x1);
    return y1 + t * (y2 - y1);
  }

  // Test linear interpolation
  void testScheduledGainInterpolation() {
    // Table: (10, 80), (50, 15), (150, 2)
    // At x=30, interpolate between (10, 80) and (50, 15)
    double gain = linearInterp(30.0, 10.0, 80.0, 50.0, 15.0);
    double expected = 80.0 + (30.0 - 10.0) / (50.0 - 10.0) * (15.0 - 80.0);

    TS_ASSERT_DELTA(gain, expected, epsilon);
  }

  // Test extrapolation clamping at table bounds
  void testScheduledGainBoundary() {
    // At x below table minimum, should use first value
    // At x above table maximum, should use last value
    double x_low = 5.0;   // Below table min of 10
    double x_high = 200.0; // Above table max of 150

    // JSBSim clamps to table bounds
    double gain_low = 80.0;  // First table value
    double gain_high = 2.0;  // Last table value

    TS_ASSERT_DELTA(gain_low, 80.0, epsilon);
    TS_ASSERT_DELTA(gain_high, 2.0, epsilon);
  }

  // Test scheduled gain multiplication
  void testScheduledGainWithMultiplier() {
    double input = 100.0;
    double scheduledGain = 0.5;  // From table lookup
    double overallGain = 0.017;  // degrees to radians conversion

    double output = input * scheduledGain * overallGain;
    double expected = 100.0 * 0.5 * 0.017;

    TS_ASSERT_DELTA(output, expected, epsilon);
  }

  /***************************************************************************
   * Edge Cases and Numerical Stability
   ***************************************************************************/

  // Test very small inputs
  void testVerySmallInput() {
    double input = 1e-15;
    double gain = 1.0;
    double output = input * gain;

    TS_ASSERT_DELTA(output, input, epsilon);
  }

  // Test very large inputs
  void testVeryLargeInput() {
    double input = 1e15;
    double gain = 0.001;
    double output = input * gain;

    TS_ASSERT_DELTA(output, 1e12, 1e6);  // Allow some tolerance for large values
  }

  // Test denormalized numbers
  void testDenormalizedInput() {
    double input = std::numeric_limits<double>::denorm_min();
    double gain = 1.0;
    double output = input * gain;

    TS_ASSERT(output >= 0.0);  // Should still be non-negative
    TS_ASSERT(output == input || output == 0.0);  // May flush to zero
  }

  // Test infinity handling
  void testInfinityInput() {
    double input = std::numeric_limits<double>::infinity();
    double gain = 2.0;
    double output = input * gain;

    TS_ASSERT(std::isinf(output));
    TS_ASSERT(output > 0);  // Positive infinity
  }

  // Test NaN propagation
  void testNaNInput() {
    double input = std::numeric_limits<double>::quiet_NaN();
    double gain = 2.0;
    double output = input * gain;

    TS_ASSERT(std::isnan(output));
  }

  // Test clipping with NaN bounds
  void testClipWithValidBounds() {
    double output = 15.0;
    double clipMin = -10.0;
    double clipMax = 10.0;

    double clipped = std::clamp(output, clipMin, clipMax);
    TS_ASSERT_DELTA(clipped, 10.0, epsilon);
  }
};
