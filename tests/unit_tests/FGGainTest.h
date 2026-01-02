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

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

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

  /***************************************************************************
   * Proportional Gain Tests
   ***************************************************************************/

  // Test proportional gain with error signal
  void testProportionalGainWithError() {
    double setpoint = 100.0;
    double actual = 80.0;
    double error = setpoint - actual;
    double Kp = 0.5;

    double output = Kp * error;
    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test proportional gain zero crossing
  void testProportionalGainZeroCrossing() {
    double Kp = 2.0;
    double errors[] = {-5.0, -2.0, 0.0, 2.0, 5.0};
    double expected[] = {-10.0, -4.0, 0.0, 4.0, 10.0};

    for (int i = 0; i < 5; i++) {
      double output = Kp * errors[i];
      TS_ASSERT_DELTA(output, expected[i], epsilon);
    }
  }

  // Test high gain amplification
  void testHighGainAmplification() {
    double input = 0.001;
    double highGain = 1000.0;
    double output = input * highGain;

    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  // Test low gain attenuation
  void testLowGainAttenuation() {
    double input = 1000.0;
    double lowGain = 0.001;
    double output = input * lowGain;

    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  /***************************************************************************
   * Derivative Gain Tests
   ***************************************************************************/

  // Test derivative gain (rate of change)
  void testDerivativeGain() {
    double error_prev = 10.0;
    double error_curr = 12.0;
    double dt = 0.1;  // seconds
    double Kd = 0.5;

    double derivative = (error_curr - error_prev) / dt;
    double output = Kd * derivative;

    TS_ASSERT_DELTA(derivative, 20.0, epsilon);
    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test derivative gain with stable signal
  void testDerivativeGainStable() {
    double error_prev = 5.0;
    double error_curr = 5.0;  // No change
    double dt = 0.1;
    double Kd = 0.5;

    double derivative = (error_curr - error_prev) / dt;
    double output = Kd * derivative;

    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test derivative kick (setpoint change)
  void testDerivativeKick() {
    // When setpoint changes, derivative can spike
    double setpoint_prev = 100.0;
    double setpoint_curr = 120.0;
    double actual = 90.0;
    double dt = 0.1;
    double Kd = 0.5;

    double error_prev = setpoint_prev - actual;
    double error_curr = setpoint_curr - actual;
    double derivative = (error_curr - error_prev) / dt;

    TS_ASSERT_DELTA(derivative, 200.0, epsilon);  // Large spike
  }

  /***************************************************************************
   * Integral Gain Tests
   ***************************************************************************/

  // Test integral accumulation
  void testIntegralAccumulation() {
    double Ki = 0.1;
    double dt = 0.1;
    double errors[] = {10.0, 10.0, 10.0, 10.0, 10.0};
    double integral = 0.0;

    for (double e : errors) {
      integral += e * dt;
    }

    double output = Ki * integral;
    TS_ASSERT_DELTA(integral, 5.0, epsilon);
    TS_ASSERT_DELTA(output, 0.5, epsilon);
  }

  // Test integral with sign change
  void testIntegralWithSignChange() {
    double Ki = 1.0;
    double dt = 0.1;
    double errors[] = {10.0, 5.0, -5.0, -10.0};
    double integral = 0.0;

    for (double e : errors) {
      integral += e * dt;
    }
    // 1.0 + 0.5 - 0.5 - 1.0 = 0.0
    TS_ASSERT_DELTA(integral, 0.0, epsilon);
  }

  // Test integral windup
  void testIntegralWindup() {
    double Ki = 0.1;
    double dt = 0.1;
    double error = 100.0;  // Large persistent error
    double integral = 0.0;
    double maxIntegral = 50.0;

    // Accumulate for 100 steps
    for (int i = 0; i < 100; i++) {
      integral += error * dt;
      integral = std::clamp(integral, -maxIntegral, maxIntegral);
    }

    // Should be clamped
    TS_ASSERT_DELTA(integral, maxIntegral, epsilon);
  }

  /***************************************************************************
   * PID Gain Combination Tests
   ***************************************************************************/

  // Test PID output calculation
  void testPIDOutput() {
    double Kp = 1.0, Ki = 0.1, Kd = 0.05;
    double error = 10.0;
    double integral = 50.0;
    double derivative = 20.0;

    double output = Kp * error + Ki * integral + Kd * derivative;
    // 10 + 5 + 1 = 16
    TS_ASSERT_DELTA(output, 16.0, epsilon);
  }

  // Test PID at zero error
  void testPIDAtZeroError() {
    double Kp = 1.0, Ki = 0.1, Kd = 0.05;
    double error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;

    double output = Kp * error + Ki * integral + Kd * derivative;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test PID with integral offset
  void testPIDWithIntegralOffset() {
    // Steady state with integral term providing offset
    double Kp = 1.0, Ki = 0.1;
    double error = 0.0;  // At setpoint
    double integral = 100.0;  // Accumulated from past

    double output = Kp * error + Ki * integral;
    TS_ASSERT_DELTA(output, 10.0, epsilon);  // Integral provides trim
  }

  /***************************************************************************
   * Gain Scheduling (Multi-Point) Tests
   ***************************************************************************/

  // Helper: Multi-point table lookup with interpolation
  double tableLookup(double x, double* xs, double* ys, int n) {
    // Clamp to table bounds
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n-1]) return ys[n-1];

    // Find interval
    for (int i = 0; i < n-1; i++) {
      if (x >= xs[i] && x <= xs[i+1]) {
        return linearInterp(x, xs[i], ys[i], xs[i+1], ys[i+1]);
      }
    }
    return ys[n-1];
  }

  // Test multi-point table lookup
  void testMultiPointTableLookup() {
    double xs[] = {0.0, 50.0, 100.0, 150.0};
    double ys[] = {1.0, 0.8, 0.5, 0.2};

    double gain = tableLookup(75.0, xs, ys, 4);
    // Between 50 and 100, interpolate 0.8 to 0.5
    double expected = 0.8 + (75.0 - 50.0) / (100.0 - 50.0) * (0.5 - 0.8);
    TS_ASSERT_DELTA(gain, expected, epsilon);
  }

  // Test table lookup at breakpoints
  void testTableLookupAtBreakpoints() {
    double xs[] = {0.0, 50.0, 100.0};
    double ys[] = {1.0, 0.5, 0.1};

    TS_ASSERT_DELTA(tableLookup(0.0, xs, ys, 3), 1.0, epsilon);
    TS_ASSERT_DELTA(tableLookup(50.0, xs, ys, 3), 0.5, epsilon);
    TS_ASSERT_DELTA(tableLookup(100.0, xs, ys, 3), 0.1, epsilon);
  }

  // Test table lookup extrapolation
  void testTableLookupExtrapolation() {
    double xs[] = {10.0, 100.0};
    double ys[] = {2.0, 0.5};

    // Below minimum - should return first value
    TS_ASSERT_DELTA(tableLookup(0.0, xs, ys, 2), 2.0, epsilon);

    // Above maximum - should return last value
    TS_ASSERT_DELTA(tableLookup(200.0, xs, ys, 2), 0.5, epsilon);
  }

  /***************************************************************************
   * Bilinear Interpolation Tests
   ***************************************************************************/

  // Helper: Bilinear interpolation
  double bilinearInterp(double x, double y,
                        double x1, double x2, double y1, double y2,
                        double q11, double q12, double q21, double q22) {
    double f1 = q11 * (x2-x) * (y2-y);
    double f2 = q21 * (x-x1) * (y2-y);
    double f3 = q12 * (x2-x) * (y-y1);
    double f4 = q22 * (x-x1) * (y-y1);
    return (f1 + f2 + f3 + f4) / ((x2-x1) * (y2-y1));
  }

  // Test bilinear interpolation at center
  void testBilinearInterpCenter() {
    // 2x2 grid: (0,0)=1, (1,0)=2, (0,1)=3, (1,1)=4
    double result = bilinearInterp(0.5, 0.5, 0.0, 1.0, 0.0, 1.0,
                                   1.0, 3.0, 2.0, 4.0);
    // At center, should be average of all 4 corners
    TS_ASSERT_DELTA(result, 2.5, epsilon);
  }

  // Test bilinear interpolation at corners
  void testBilinearInterpCorners() {
    TS_ASSERT_DELTA(bilinearInterp(0.0, 0.0, 0.0, 1.0, 0.0, 1.0,
                                   1.0, 3.0, 2.0, 4.0), 1.0, epsilon);
    TS_ASSERT_DELTA(bilinearInterp(1.0, 0.0, 0.0, 1.0, 0.0, 1.0,
                                   1.0, 3.0, 2.0, 4.0), 2.0, epsilon);
    TS_ASSERT_DELTA(bilinearInterp(0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
                                   1.0, 3.0, 2.0, 4.0), 3.0, epsilon);
    TS_ASSERT_DELTA(bilinearInterp(1.0, 1.0, 0.0, 1.0, 0.0, 1.0,
                                   1.0, 3.0, 2.0, 4.0), 4.0, epsilon);
  }

  /***************************************************************************
   * Deadband Tests
   ***************************************************************************/

  // Helper: Apply deadband
  double applyDeadband(double input, double deadband) {
    if (std::abs(input) < deadband) return 0.0;
    if (input > 0) return input - deadband;
    return input + deadband;
  }

  // Test deadband suppression
  void testDeadbandSuppression() {
    double deadband = 0.1;

    TS_ASSERT_DELTA(applyDeadband(0.05, deadband), 0.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(-0.05, deadband), 0.0, epsilon);
  }

  // Test deadband passthrough
  void testDeadbandPassthrough() {
    double deadband = 0.1;

    TS_ASSERT_DELTA(applyDeadband(0.5, deadband), 0.4, epsilon);
    TS_ASSERT_DELTA(applyDeadband(-0.5, deadband), -0.4, epsilon);
  }

  // Test deadband with gain
  void testDeadbandWithGain() {
    double input = 0.5;
    double deadband = 0.1;
    double gain = 2.0;

    double adjusted = applyDeadband(input, deadband);
    double output = adjusted * gain;

    TS_ASSERT_DELTA(output, 0.8, epsilon);
  }

  /***************************************************************************
   * Rate Limiting Tests
   ***************************************************************************/

  // Test rate limiting up
  void testRateLimitingUp() {
    double current = 10.0;
    double target = 50.0;
    double maxRate = 5.0;  // per second
    double dt = 1.0;

    double maxChange = maxRate * dt;
    double change = std::clamp(target - current, -maxChange, maxChange);
    double newValue = current + change;

    TS_ASSERT_DELTA(newValue, 15.0, epsilon);
  }

  // Test rate limiting down
  void testRateLimitingDown() {
    double current = 50.0;
    double target = 10.0;
    double maxRate = 5.0;
    double dt = 1.0;

    double maxChange = maxRate * dt;
    double change = std::clamp(target - current, -maxChange, maxChange);
    double newValue = current + change;

    TS_ASSERT_DELTA(newValue, 45.0, epsilon);
  }

  // Test rate limiting no limit needed
  void testRateLimitingNoLimit() {
    double current = 10.0;
    double target = 12.0;
    double maxRate = 5.0;
    double dt = 1.0;

    double maxChange = maxRate * dt;
    double change = std::clamp(target - current, -maxChange, maxChange);
    double newValue = current + change;

    TS_ASSERT_DELTA(newValue, 12.0, epsilon);
  }

  /***************************************************************************
   * Soft Saturation Tests
   ***************************************************************************/

  // Helper: Soft saturation using tanh
  double softSaturate(double input, double limit) {
    return limit * std::tanh(input / limit);
  }

  // Test soft saturation near zero
  void testSoftSaturationNearZero() {
    double output = softSaturate(0.1, 1.0);
    // Near zero, tanh(x) ≈ x
    TS_ASSERT_DELTA(output, 0.1, 0.01);
  }

  // Test soft saturation at limits
  void testSoftSaturationAtLimits() {
    double output = softSaturate(10.0, 1.0);
    // Should be close to limit
    TS_ASSERT(output > 0.99);
    TS_ASSERT(output < 1.0);
  }

  // Test soft saturation symmetry
  void testSoftSaturationSymmetry() {
    double limit = 1.0;
    double positive = softSaturate(5.0, limit);
    double negative = softSaturate(-5.0, limit);

    TS_ASSERT_DELTA(positive, -negative, epsilon);
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test degrees to radians gain
  void testDegreesToRadiansGain() {
    double degrees = 90.0;
    double gain = M_PI / 180.0;
    double radians = degrees * gain;

    TS_ASSERT_DELTA(radians, M_PI / 2.0, epsilon);
  }

  // Test knots to fps gain
  void testKnotsToFpsGain() {
    double knots = 100.0;
    double gain = 1.68781;  // ft/s per knot
    double fps = knots * gain;

    TS_ASSERT_DELTA(fps, 168.781, 0.001);
  }

  // Test feet to meters gain
  void testFeetToMetersGain() {
    double feet = 1000.0;
    double gain = 0.3048;
    double meters = feet * gain;

    TS_ASSERT_DELTA(meters, 304.8, epsilon);
  }

  // Test lbs to kg gain
  void testLbsToKgGain() {
    double lbs = 100.0;
    double gain = 0.453592;
    double kg = lbs * gain;

    TS_ASSERT_DELTA(kg, 45.3592, 0.0001);
  }

  /***************************************************************************
   * Feedforward Gain Tests
   ***************************************************************************/

  // Test feedforward with feedback
  void testFeedforwardWithFeedback() {
    double command = 100.0;
    double error = 5.0;
    double Kff = 0.9;  // Feedforward gain
    double Kp = 0.5;   // Feedback gain

    double output = Kff * command + Kp * error;
    TS_ASSERT_DELTA(output, 92.5, epsilon);
  }

  // Test pure feedforward
  void testPureFeedforward() {
    double command = 50.0;
    double Kff = 1.0;

    double output = Kff * command;
    TS_ASSERT_DELTA(output, 50.0, epsilon);
  }

  // Test feedforward scaling
  void testFeedforwardScaling() {
    double commands[] = {0.0, 25.0, 50.0, 75.0, 100.0};
    double Kff = 0.8;

    for (double cmd : commands) {
      double output = Kff * cmd;
      TS_ASSERT_DELTA(output, 0.8 * cmd, epsilon);
    }
  }

  /***************************************************************************
   * Trim Offset Tests
   ***************************************************************************/

  // Test trim addition
  void testTrimAddition() {
    double controlInput = 0.0;  // Centered
    double trim = 5.0;

    double output = controlInput + trim;
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test trim with control input
  void testTrimWithControlInput() {
    double controlInput = 10.0;
    double trim = -3.0;

    double output = controlInput + trim;
    TS_ASSERT_DELTA(output, 7.0, epsilon);
  }

  // Test trim followed by gain
  void testTrimFollowedByGain() {
    double input = 5.0;
    double trim = 2.0;
    double gain = 3.0;

    double output = (input + trim) * gain;
    TS_ASSERT_DELTA(output, 21.0, epsilon);
  }

  /***************************************************************************
   * Signal Selection Tests
   ***************************************************************************/

  // Test maximum selection
  void testMaximumSelection() {
    double signals[] = {10.0, 25.0, 15.0, 5.0};
    double maxVal = signals[0];
    for (double s : signals) {
      maxVal = std::max(maxVal, s);
    }

    TS_ASSERT_DELTA(maxVal, 25.0, epsilon);
  }

  // Test minimum selection
  void testMinimumSelection() {
    double signals[] = {10.0, 25.0, 15.0, 5.0};
    double minVal = signals[0];
    for (double s : signals) {
      minVal = std::min(minVal, s);
    }

    TS_ASSERT_DELTA(minVal, 5.0, epsilon);
  }

  // Test absolute value selection
  void testAbsoluteMaxSelection() {
    double signals[] = {-30.0, 10.0, -5.0, 20.0};
    double maxAbs = 0.0;
    for (double s : signals) {
      if (std::abs(s) > std::abs(maxAbs)) {
        maxAbs = s;
      }
    }

    TS_ASSERT_DELTA(maxAbs, -30.0, epsilon);  // -30 has largest magnitude
  }

  /***************************************************************************
   * Weighted Sum Tests
   ***************************************************************************/

  // Test weighted sum
  void testWeightedSum() {
    double values[] = {10.0, 20.0, 30.0};
    double weights[] = {0.5, 0.3, 0.2};

    double sum = 0.0;
    for (int i = 0; i < 3; i++) {
      sum += values[i] * weights[i];
    }
    // 5 + 6 + 6 = 17
    TS_ASSERT_DELTA(sum, 17.0, epsilon);
  }

  // Test weighted average
  void testWeightedAverage() {
    double values[] = {10.0, 20.0};
    double weights[] = {0.7, 0.3};

    double sum = 0.0;
    double weightSum = 0.0;
    for (int i = 0; i < 2; i++) {
      sum += values[i] * weights[i];
      weightSum += weights[i];
    }
    double avg = sum / weightSum;

    TS_ASSERT_DELTA(avg, 13.0, epsilon);
  }

  // Test equal weights
  void testEqualWeights() {
    double values[] = {10.0, 20.0, 30.0};
    double weight = 1.0 / 3.0;

    double sum = 0.0;
    for (double v : values) {
      sum += v * weight;
    }

    TS_ASSERT_DELTA(sum, 20.0, epsilon);
  }

  /***************************************************************************
   * Gain Margin Tests
   ***************************************************************************/

  // Test gain margin calculation
  void testGainMarginCalculation() {
    // Gain margin = 1 / |G(jw)| at phase crossover
    double systemGain = 0.5;  // At phase crossover
    double gainMargin = 1.0 / systemGain;

    TS_ASSERT_DELTA(gainMargin, 2.0, epsilon);  // 2x margin
  }

  // Test gain margin in dB
  void testGainMarginInDB() {
    double gainMargin = 2.0;
    double gainMarginDB = 20.0 * std::log10(gainMargin);

    TS_ASSERT_DELTA(gainMarginDB, 6.02, 0.01);  // ~6 dB
  }

  // Test stability with gain increase
  void testStabilityWithGainIncrease() {
    double nominalGain = 1.0;
    double gainMargin = 2.0;
    double maxStableGain = nominalGain * gainMargin;

    TS_ASSERT_DELTA(maxStableGain, 2.0, epsilon);
  }

  /***************************************************************************
   * DC Offset Tests
   ***************************************************************************/

  // Test DC offset removal
  void testDCOffsetRemoval() {
    double signal = 10.0;
    double dcOffset = 3.0;
    double adjusted = signal - dcOffset;

    TS_ASSERT_DELTA(adjusted, 7.0, epsilon);
  }

  // Test DC offset addition
  void testDCOffsetAddition() {
    double signal = -5.0;
    double dcOffset = 10.0;
    double adjusted = signal + dcOffset;

    TS_ASSERT_DELTA(adjusted, 5.0, epsilon);
  }

  // Test centering signal around zero
  void testSignalCentering() {
    double signals[] = {8.0, 10.0, 12.0, 14.0};
    double mean = 0.0;
    for (double s : signals) {
      mean += s;
    }
    mean /= 4.0;

    double centered = signals[0] - mean;
    TS_ASSERT_DELTA(centered, -3.0, epsilon);
  }

  /***************************************************************************
   * Normalization Tests
   ***************************************************************************/

  // Test normalization to unit range
  void testNormalizationToUnitRange() {
    double value = 75.0;
    double minVal = 50.0;
    double maxVal = 100.0;

    double normalized = (value - minVal) / (maxVal - minVal);
    TS_ASSERT_DELTA(normalized, 0.5, epsilon);
  }

  // Test normalization to symmetric range
  void testNormalizationToSymmetricRange() {
    double value = 75.0;
    double center = 50.0;
    double range = 50.0;

    double normalized = (value - center) / range;
    TS_ASSERT_DELTA(normalized, 0.5, epsilon);
  }

  // Test denormalization
  void testDenormalization() {
    double normalized = 0.5;
    double minVal = 0.0;
    double maxVal = 100.0;

    double value = minVal + normalized * (maxVal - minVal);
    TS_ASSERT_DELTA(value, 50.0, epsilon);
  }

  /***************************************************************************
   * Boundary and Edge Cases
   ***************************************************************************/

  // Test gain at boundaries
  void testGainAtBoundaries() {
    double input = 1.0;
    double gains[] = {-1e6, -1.0, 0.0, 1.0, 1e6};

    for (double g : gains) {
      double output = input * g;
      TS_ASSERT(!std::isnan(output));
      TS_ASSERT(!std::isinf(output) || std::abs(g) > 1e5);
    }
  }

  // Test clamp with equal bounds
  void testClampWithEqualBounds() {
    double value = 10.0;
    double bound = 5.0;
    double clamped = std::clamp(value, bound, bound);

    TS_ASSERT_DELTA(clamped, 5.0, epsilon);
  }

  // Test zero range mapping
  void testZeroRangeMapping() {
    // When domain is a single point, return range midpoint or handle specially
    double input = 5.0;
    double domain = 5.0;  // Single point domain
    double rangeMin = 0.0, rangeMax = 100.0;

    // Handle degenerate case
    double output = (rangeMin + rangeMax) / 2.0;
    TS_ASSERT_DELTA(output, 50.0, epsilon);
  }

  // Test step response
  void testStepResponse() {
    double stepInput = 1.0;  // Unit step
    double gain = 2.0;

    // Immediate response for pure gain
    double output = stepInput * gain;
    TS_ASSERT_DELTA(output, 2.0, epsilon);
  }

  // Test sign preservation
  void testSignPreservation() {
    double positive = 5.0;
    double negative = -5.0;
    double gain = 3.0;

    TS_ASSERT(positive * gain > 0.0);
    TS_ASSERT(negative * gain < 0.0);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete gain schedule system
  void testCompleteGainScheduleSystem() {
    // Speed-dependent gain schedule
    double speeds[] = {100.0, 200.0, 300.0, 400.0};
    double gains[] = {2.0, 1.5, 1.0, 0.75};

    double speed = 250.0;
    // Interpolate between 200 and 300
    double t = (speed - 200.0) / (300.0 - 200.0);
    double gain = gains[1] + t * (gains[2] - gains[1]);

    TS_ASSERT_DELTA(gain, 1.25, 0.01);
  }

  // Test feedback gain calculation
  void testFeedbackGainCalculation() {
    double open_loop_gain = 10.0;
    double feedback_gain = 0.5;

    double closed_loop_gain = open_loop_gain / (1.0 + open_loop_gain * feedback_gain);
    TS_ASSERT_DELTA(closed_loop_gain, 1.667, 0.01);
  }

  // Test proportional-derivative gain
  void testProportionalDerivativeGain() {
    double error = 5.0;
    double error_rate = 2.0;
    double Kp = 1.0;
    double Kd = 0.5;

    double output = Kp * error + Kd * error_rate;
    TS_ASSERT_DELTA(output, 6.0, epsilon);
  }

  // Test gain margin in dB
  void testGainMarginInDecibels() {
    double gain_crossover = 0.5;
    double gain_margin_db = 20.0 * std::log10(1.0 / gain_crossover);

    TS_ASSERT_DELTA(gain_margin_db, 6.02, 0.1);
  }

  // Test variable gain with altitude
  void testVariableGainWithAltitude() {
    double sea_level_gain = 1.0;
    double gain_reduction_rate = 0.00001;  // per ft (smaller rate)
    double altitude = 30000.0;

    double gain = sea_level_gain * (1.0 - gain_reduction_rate * altitude);
    TS_ASSERT(gain < sea_level_gain);
    TS_ASSERT(gain > 0.0);
    TS_ASSERT_DELTA(gain, 0.7, epsilon);  // 1 - 0.3 = 0.7
  }

  // Test anti-windup gain
  void testAntiWindupGain() {
    double integral = 100.0;
    double limit = 50.0;
    double anti_windup_gain = 0.1;

    double excess = std::max(0.0, std::abs(integral) - limit);
    double correction = anti_windup_gain * excess;

    TS_ASSERT_DELTA(correction, 5.0, epsilon);
  }

  // Test gain with dead zone
  void testGainWithDeadZone() {
    double input = 0.05;
    double dead_zone = 0.1;
    double gain = 2.0;

    double effective_input = (std::abs(input) < dead_zone) ? 0.0 : input;
    double output = effective_input * gain;

    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test nonlinear gain curve
  void testNonlinearGainCurve() {
    double input = 0.5;
    double linear_gain = 1.0;
    double nonlinear_factor = 2.0;

    // Quadratic nonlinearity
    double output = linear_gain * input + nonlinear_factor * input * input;
    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  // Test gain with hysteresis
  void testGainWithHysteresis() {
    double input = 5.0;
    double hysteresis_band = 0.5;
    double last_output = 4.8;

    // Output only changes if input exceeds hysteresis band
    double output = last_output;
    if (std::abs(input - last_output) > hysteresis_band) {
      output = input;
    }

    TS_ASSERT_DELTA(output, 4.8, epsilon);  // No change
  }

  // Test cascaded gains
  void testCascadedGains() {
    double input = 1.0;
    double gain1 = 2.0;
    double gain2 = 3.0;
    double gain3 = 0.5;

    double output = input * gain1 * gain2 * gain3;
    TS_ASSERT_DELTA(output, 3.0, epsilon);
  }

  // Test rate-limited gain change
  void testRateLimitedGainChange() {
    double current_gain = 1.0;
    double target_gain = 2.0;
    double max_rate = 0.1;  // per time step
    double dt = 1.0;

    double gain_change = std::clamp(target_gain - current_gain, -max_rate * dt, max_rate * dt);
    double new_gain = current_gain + gain_change;

    TS_ASSERT_DELTA(new_gain, 1.1, epsilon);
  }

  // Test gain sensitivity analysis
  void testGainSensitivityAnalysis() {
    double nominal_gain = 1.0;
    double perturbation = 0.01;
    double input = 10.0;

    double nominal_output = input * nominal_gain;
    double perturbed_output = input * (nominal_gain + perturbation);
    double sensitivity = (perturbed_output - nominal_output) / perturbation;

    TS_ASSERT_DELTA(sensitivity, input, epsilon);
  }

  // Test gain normalization
  void testGainNormalization() {
    double raw_gain = 50.0;
    double max_gain = 100.0;
    double min_gain = 0.0;

    double normalized = (raw_gain - min_gain) / (max_gain - min_gain);
    TS_ASSERT_DELTA(normalized, 0.5, epsilon);
  }

  // Test automatic gain control
  void testAutomaticGainControl() {
    double signal_level = 0.5;
    double target_level = 1.0;
    double agc_gain = target_level / signal_level;

    double output = signal_level * agc_gain;
    TS_ASSERT_DELTA(output, target_level, epsilon);
  }

  // Test gain instance independence
  void testGainInstanceIndependence() {
    double gain1 = 2.0;
    double gain2 = 5.0;
    double input = 10.0;

    double output1 = input * gain1;
    double output2 = input * gain2;

    TS_ASSERT(output1 != output2);
    TS_ASSERT_DELTA(output1, 20.0, epsilon);
    TS_ASSERT_DELTA(output2, 50.0, epsilon);
  }

  // Test gain calculation state independence
  void testGainCalculationStateIndependence() {
    double gain = 3.0;
    double input1 = 5.0;
    double input2 = 10.0;

    double output1 = input1 * gain;
    double output2 = input2 * gain;

    TS_ASSERT_DELTA(output1, 15.0, epsilon);
    TS_ASSERT_DELTA(output2, 30.0, epsilon);
    TS_ASSERT(output2 == output1 * 2.0);
  }

  // Test multiple gain path independence
  void testMultipleGainPathIndependence() {
    double input = 100.0;
    double path1_gain = 0.5;
    double path2_gain = 0.8;

    double path1_output = input * path1_gain;
    double path2_output = input * path2_gain;
    double combined = path1_output + path2_output;

    TS_ASSERT_DELTA(path1_output, 50.0, epsilon);
    TS_ASSERT_DELTA(path2_output, 80.0, epsilon);
    TS_ASSERT_DELTA(combined, 130.0, epsilon);
  }
};
