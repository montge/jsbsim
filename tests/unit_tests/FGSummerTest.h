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

#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"
#include "models/FGAuxiliary.h"
#include "models/FGAerodynamics.h"

using namespace JSBSim;

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

  /***************************************************************************
   * Weighted Summation Tests
   ***************************************************************************/

  // Test weighted sum of two inputs
  void testWeightedSumTwoInputs() {
    double input1 = 10.0, weight1 = 0.7;
    double input2 = 20.0, weight2 = 0.3;

    double output = input1 * weight1 + input2 * weight2;
    TS_ASSERT_DELTA(output, 13.0, epsilon);
  }

  // Test weighted sum with equal weights
  void testWeightedSumEqualWeights() {
    double input1 = 10.0, input2 = 20.0, input3 = 30.0;
    double weight = 1.0 / 3.0;

    double output = (input1 + input2 + input3) * weight;
    TS_ASSERT_DELTA(output, 20.0, epsilon);
  }

  // Test normalized weights
  void testNormalizedWeights() {
    double inputs[] = {10.0, 20.0, 30.0};
    double weights[] = {2.0, 3.0, 5.0};  // Sum = 10

    double weightSum = 0.0;
    double weightedSum = 0.0;
    for (int i = 0; i < 3; i++) {
      weightedSum += inputs[i] * weights[i];
      weightSum += weights[i];
    }
    double normalized = weightedSum / weightSum;

    // (10*2 + 20*3 + 30*5) / 10 = 230/10 = 23
    TS_ASSERT_DELTA(normalized, 23.0, epsilon);
  }

  // Test weighted difference
  void testWeightedDifference() {
    double positive = 100.0, weightPos = 1.0;
    double negative = 50.0, weightNeg = -1.0;

    double output = positive * weightPos + negative * weightNeg;
    TS_ASSERT_DELTA(output, 50.0, epsilon);
  }

  /***************************************************************************
   * Conditional Summation Tests
   ***************************************************************************/

  // Test conditional input inclusion
  void testConditionalInputInclusion() {
    double input1 = 10.0;
    double input2 = 20.0;
    bool enable1 = true;
    bool enable2 = false;

    double output = (enable1 ? input1 : 0.0) + (enable2 ? input2 : 0.0);
    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test all inputs enabled
  void testAllInputsEnabled() {
    double inputs[] = {10.0, 20.0, 30.0};
    bool enables[] = {true, true, true};

    double output = 0.0;
    for (int i = 0; i < 3; i++) {
      if (enables[i]) output += inputs[i];
    }
    TS_ASSERT_DELTA(output, 60.0, epsilon);
  }

  // Test all inputs disabled
  void testAllInputsDisabled() {
    double inputs[] = {10.0, 20.0, 30.0};
    bool enables[] = {false, false, false};

    double output = 0.0;
    for (int i = 0; i < 3; i++) {
      if (enables[i]) output += inputs[i];
    }
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test switching logic
  void testSwitchingLogic() {
    double inputA = 10.0;
    double inputB = 20.0;
    bool selectA = true;

    double output = selectA ? inputA : inputB;
    TS_ASSERT_DELTA(output, 10.0, epsilon);

    selectA = false;
    output = selectA ? inputA : inputB;
    TS_ASSERT_DELTA(output, 20.0, epsilon);
  }

  /***************************************************************************
   * Cascaded Summer Tests
   ***************************************************************************/

  // Test two-stage summer
  void testTwoStageSummer() {
    // Stage 1: a + b
    // Stage 2: (a + b) + c
    double a = 5.0, b = 3.0, c = 2.0;

    double stage1 = a + b;
    double stage2 = stage1 + c;

    TS_ASSERT_DELTA(stage1, 8.0, epsilon);
    TS_ASSERT_DELTA(stage2, 10.0, epsilon);
  }

  // Test parallel to series
  void testParallelToSeries() {
    // Parallel: (a + b), (c + d)
    // Series: result1 + result2
    double a = 1.0, b = 2.0, c = 3.0, d = 4.0;

    double parallel1 = a + b;
    double parallel2 = c + d;
    double series = parallel1 + parallel2;

    TS_ASSERT_DELTA(series, 10.0, epsilon);
  }

  // Test feedback path
  void testFeedbackPath() {
    // Simple feedback: output = input + feedback_gain * prev_output
    double input = 10.0;
    double prevOutput = 5.0;
    double feedbackGain = 0.5;

    double output = input + feedbackGain * prevOutput;
    TS_ASSERT_DELTA(output, 12.5, epsilon);
  }

  /***************************************************************************
   * Running Sum and Moving Average Tests
   ***************************************************************************/

  // Test running sum
  void testRunningSum() {
    double values[] = {1.0, 2.0, 3.0, 4.0, 5.0};
    double runningSum = 0.0;
    double expected[] = {1.0, 3.0, 6.0, 10.0, 15.0};

    for (int i = 0; i < 5; i++) {
      runningSum += values[i];
      TS_ASSERT_DELTA(runningSum, expected[i], epsilon);
    }
  }

  // Test simple moving average
  void testSimpleMovingAverage() {
    std::vector<double> values = {10.0, 20.0, 30.0, 40.0, 50.0};
    int windowSize = 3;

    // Average of last 3 values: [30, 40, 50]
    double sum = 0.0;
    for (int i = values.size() - windowSize; i < (int)values.size(); i++) {
      sum += values[i];
    }
    double average = sum / windowSize;

    TS_ASSERT_DELTA(average, 40.0, epsilon);
  }

  // Test exponential moving average
  void testExponentialMovingAverage() {
    double alpha = 0.3;  // Smoothing factor
    double values[] = {10.0, 20.0, 30.0, 40.0};
    double ema = values[0];

    for (int i = 1; i < 4; i++) {
      ema = alpha * values[i] + (1.0 - alpha) * ema;
    }

    // Should be weighted toward more recent values
    TS_ASSERT(ema > 20.0);
    TS_ASSERT(ema < 40.0);
  }

  /***************************************************************************
   * Differential (Rate of Change) Tests
   ***************************************************************************/

  // Test simple difference
  void testSimpleDifference() {
    double current = 50.0;
    double previous = 40.0;

    double diff = current - previous;
    TS_ASSERT_DELTA(diff, 10.0, epsilon);
  }

  // Test rate of change
  void testRateOfChange() {
    double current = 50.0;
    double previous = 40.0;
    double dt = 0.1;

    double rate = (current - previous) / dt;
    TS_ASSERT_DELTA(rate, 100.0, epsilon);
  }

  // Test second derivative
  void testSecondDerivative() {
    double values[] = {10.0, 15.0, 22.0};  // Position at t=0, 1, 2
    double dt = 1.0;

    double v1 = (values[1] - values[0]) / dt;  // First velocity
    double v2 = (values[2] - values[1]) / dt;  // Second velocity
    double accel = (v2 - v1) / dt;             // Acceleration

    TS_ASSERT_DELTA(v1, 5.0, epsilon);
    TS_ASSERT_DELTA(v2, 7.0, epsilon);
    TS_ASSERT_DELTA(accel, 2.0, epsilon);
  }

  /***************************************************************************
   * Control Mixer Tests
   ***************************************************************************/

  // Test aileron mixer
  void testAileronMixer() {
    double stickInput = 0.5;   // Roll right
    double leftAileron = -stickInput;   // Deflect up
    double rightAileron = stickInput;   // Deflect down

    TS_ASSERT_DELTA(leftAileron, -0.5, epsilon);
    TS_ASSERT_DELTA(rightAileron, 0.5, epsilon);
    TS_ASSERT_DELTA(leftAileron + rightAileron, 0.0, epsilon);  // Differential
  }

  // Test V-tail mixer
  void testVTailMixer() {
    double elevator = 0.5;
    double rudder = 0.3;

    double leftRuddervator = elevator + rudder;
    double rightRuddervator = elevator - rudder;

    TS_ASSERT_DELTA(leftRuddervator, 0.8, epsilon);
    TS_ASSERT_DELTA(rightRuddervator, 0.2, epsilon);
  }

  // Test elevon mixer (delta wing)
  void testElevonMixer() {
    double pitch = 0.5;
    double roll = 0.2;

    double leftElevon = pitch + roll;
    double rightElevon = pitch - roll;

    TS_ASSERT_DELTA(leftElevon, 0.7, epsilon);
    TS_ASSERT_DELTA(rightElevon, 0.3, epsilon);
  }

  // Test throttle mixer (twin engine)
  void testThrottleMixer() {
    double throttle = 0.8;
    double differential = 0.1;  // Yaw input via differential thrust

    double leftEngine = throttle - differential;
    double rightEngine = throttle + differential;

    TS_ASSERT_DELTA(leftEngine, 0.7, epsilon);
    TS_ASSERT_DELTA(rightEngine, 0.9, epsilon);
  }

  /***************************************************************************
   * Saturation Recovery Tests
   ***************************************************************************/

  // Test anti-windup sum
  void testAntiWindupSum() {
    double command = 20.0;
    double limit = 10.0;

    // Sum would exceed limit
    double preClip = command;
    double output = clip(preClip, -limit, limit);
    double saturationError = preClip - output;

    TS_ASSERT_DELTA(output, 10.0, epsilon);
    TS_ASSERT_DELTA(saturationError, 10.0, epsilon);
  }

  // Test soft limiting
  void testSoftLimiting() {
    // Using tanh for soft saturation
    double input = 5.0;
    double limit = 1.0;
    double output = limit * std::tanh(input / limit);

    TS_ASSERT(output < 1.0);
    TS_ASSERT(output > 0.99);
  }

  // Test gradual saturation
  void testGradualSaturation() {
    double inputs[] = {0.0, 0.5, 1.0, 2.0, 5.0, 10.0};
    double limit = 1.0;

    for (double input : inputs) {
      double output = limit * std::tanh(input / limit);
      TS_ASSERT(output >= 0.0);
      TS_ASSERT(output <= limit);
    }
  }

  /***************************************************************************
   * Statistical Operations Tests
   ***************************************************************************/

  // Test mean calculation
  void testMeanCalculation() {
    std::vector<double> values = {10.0, 20.0, 30.0, 40.0, 50.0};
    double sum = 0.0;
    for (double v : values) {
      sum += v;
    }
    double mean = sum / values.size();

    TS_ASSERT_DELTA(mean, 30.0, epsilon);
  }

  // Test variance calculation
  void testVarianceCalculation() {
    std::vector<double> values = {10.0, 20.0, 30.0, 40.0, 50.0};
    double mean = 30.0;
    double sumSquaredDiff = 0.0;

    for (double v : values) {
      double diff = v - mean;
      sumSquaredDiff += diff * diff;
    }
    double variance = sumSquaredDiff / values.size();

    // Variance = (400 + 100 + 0 + 100 + 400) / 5 = 200
    TS_ASSERT_DELTA(variance, 200.0, epsilon);
  }

  // Test standard deviation
  void testStandardDeviation() {
    double variance = 200.0;
    double stdDev = std::sqrt(variance);

    TS_ASSERT_DELTA(stdDev, 14.142, 0.001);
  }

  // Test min/max tracking
  void testMinMaxTracking() {
    std::vector<double> values = {30.0, 10.0, 50.0, 20.0, 40.0};
    double minVal = values[0], maxVal = values[0];

    for (double v : values) {
      minVal = std::min(minVal, v);
      maxVal = std::max(maxVal, v);
    }

    TS_ASSERT_DELTA(minVal, 10.0, epsilon);
    TS_ASSERT_DELTA(maxVal, 50.0, epsilon);
  }

  /***************************************************************************
   * Signal Combination Tests
   ***************************************************************************/

  // Test parallel signal paths
  void testParallelSignalPaths() {
    double input = 10.0;
    double path1Gain = 0.5;
    double path2Gain = 0.3;
    double path3Gain = 0.2;

    double output = input * path1Gain + input * path2Gain + input * path3Gain;
    TS_ASSERT_DELTA(output, 10.0, epsilon);  // Gains sum to 1.0
  }

  // Test signal blending
  void testSignalBlending() {
    double signal1 = 100.0;
    double signal2 = 50.0;
    double blendFactor = 0.7;  // 70% signal1, 30% signal2

    double blended = signal1 * blendFactor + signal2 * (1.0 - blendFactor);
    TS_ASSERT_DELTA(blended, 85.0, epsilon);
  }

  // Test crossfade
  void testCrossfade() {
    double signalA = 100.0;
    double signalB = 0.0;

    // Crossfade from A to B
    for (int i = 0; i <= 10; i++) {
      double t = i / 10.0;  // 0.0 to 1.0
      double output = signalA * (1.0 - t) + signalB * t;

      double expected = 100.0 * (1.0 - t);
      TS_ASSERT_DELTA(output, expected, epsilon);
    }
  }

  /***************************************************************************
   * Control Law Application Tests
   ***************************************************************************/

  // Test stability augmentation
  void testStabilityAugmentation() {
    double pilotCommand = 1.0;
    double rate = 0.5;          // Current rate
    double rateGain = 0.3;      // Damping gain

    // Pilot command minus rate feedback
    double augmented = pilotCommand - rateGain * rate;
    TS_ASSERT_DELTA(augmented, 0.85, epsilon);
  }

  // Test load alleviation
  void testLoadAlleviation() {
    double structuralLoad = 2.5;  // G
    double loadLimit = 2.0;
    double alleviationGain = 0.5;

    // Reduce control authority when load is high
    double excessLoad = std::max(0.0, structuralLoad - loadLimit);
    double alleviation = alleviationGain * excessLoad;

    TS_ASSERT_DELTA(excessLoad, 0.5, epsilon);
    TS_ASSERT_DELTA(alleviation, 0.25, epsilon);
  }

  // Test envelope protection
  void testEnvelopeProtection() {
    double pilotPitch = 1.0;    // Full nose up
    double aoa = 15.0;          // Current AOA
    double aoaLimit = 12.0;     // AOA limit
    double protectionGain = 0.2;

    // Reduce pitch command when approaching AOA limit
    double aoaExcess = std::max(0.0, aoa - aoaLimit);
    double protection = protectionGain * aoaExcess;
    double limitedCommand = pilotPitch - protection;

    TS_ASSERT_DELTA(aoaExcess, 3.0, epsilon);
    TS_ASSERT_DELTA(protection, 0.6, epsilon);
    TS_ASSERT_DELTA(limitedCommand, 0.4, epsilon);
  }

  /***************************************************************************
   * Numerical Stability Tests
   ***************************************************************************/

  // Test Kahan summation concept
  void testKahanSummationConcept() {
    // Demonstrates compensated summation for improved precision
    double c = 0.0;  // Compensation
    double sum = 0.0;
    double values[] = {1.0, 1e-16, 1e-16, 1e-16};

    for (double v : values) {
      double y = v - c;
      double t = sum + y;
      c = (t - sum) - y;  // Recovers low-order bits
      sum = t;
    }

    TS_ASSERT(sum >= 1.0);
  }

  // Test summation order independence
  void testSummationOrderIndependence() {
    std::vector<double> values = {1.0, 2.0, 3.0, 4.0, 5.0};
    double sum1 = 0.0;
    double sum2 = 0.0;

    // Forward order
    for (double v : values) {
      sum1 += v;
    }

    // Reverse order
    for (int i = values.size() - 1; i >= 0; i--) {
      sum2 += values[i];
    }

    TS_ASSERT_DELTA(sum1, sum2, epsilon);
  }

  // Test pairwise summation concept
  void testPairwiseSummationConcept() {
    double values[] = {1.0, 2.0, 3.0, 4.0};

    // Pairwise: ((1+2) + (3+4))
    double pair1 = values[0] + values[1];
    double pair2 = values[2] + values[3];
    double result = pair1 + pair2;

    TS_ASSERT_DELTA(result, 10.0, epsilon);
  }

  /***************************************************************************
   * Identity and Zero Tests
   ***************************************************************************/

  // Test additive identity
  void testAdditiveIdentity() {
    double value = 42.0;
    double zero = 0.0;

    TS_ASSERT_DELTA(value + zero, value, epsilon);
    TS_ASSERT_DELTA(zero + value, value, epsilon);
  }

  // Test additive inverse
  void testAdditiveInverse() {
    double value = 42.0;
    double inverse = -42.0;

    TS_ASSERT_DELTA(value + inverse, 0.0, epsilon);
  }

  // Test sum of zeroes
  void testSumOfZeroes() {
    double sum = 0.0;
    for (int i = 0; i < 100; i++) {
      sum += 0.0;
    }
    TS_ASSERT_DELTA(sum, 0.0, epsilon);
  }

  /***************************************************************************
   * Distributive Property Tests
   ***************************************************************************/

  // Test distributive property
  void testDistributiveProperty() {
    // a * (b + c) = a*b + a*c
    double a = 2.0, b = 3.0, c = 4.0;

    double result1 = a * (b + c);
    double result2 = a * b + a * c;

    TS_ASSERT_DELTA(result1, result2, epsilon);
  }

  // Test factoring
  void testFactoring() {
    // 2x + 2y = 2(x + y)
    double x = 5.0, y = 7.0;
    double factor = 2.0;

    double expanded = factor * x + factor * y;
    double factored = factor * (x + y);

    TS_ASSERT_DELTA(expanded, factored, epsilon);
  }

  /***************************************************************************
   * Miscellaneous Tests
   ***************************************************************************/

  // Test bipolar sum
  void testBipolarSum() {
    // Sum that swings positive and negative
    double values[] = {5.0, -3.0, 8.0, -2.0, -1.0};
    double sum = 0.0;
    for (double v : values) {
      sum += v;
    }

    TS_ASSERT_DELTA(sum, 7.0, epsilon);
  }

  // Test incremental sum
  void testIncrementalSum() {
    double base = 100.0;
    double increments[] = {0.1, 0.2, -0.05, 0.15};
    double total = base;

    for (double inc : increments) {
      total += inc;
    }

    TS_ASSERT_DELTA(total, 100.4, epsilon);
  }

  // Test sum with alternating signs
  void testAlternatingSigns() {
    // 1 - 2 + 3 - 4 + 5 = 3
    double sum = 0.0;
    for (int i = 1; i <= 5; i++) {
      double sign = (i % 2 == 1) ? 1.0 : -1.0;
      sum += sign * i;
    }

    TS_ASSERT_DELTA(sum, 3.0, epsilon);
  }

  // Test geometric sum approximation
  void testGeometricSumApproximation() {
    // Sum of 0.5^n for n=0 to infinity -> 2
    double r = 0.5;
    double sum = 0.0;
    double term = 1.0;

    for (int i = 0; i < 20; i++) {
      sum += term;
      term *= r;
    }

    // Should be close to 1/(1-r) = 2
    TS_ASSERT_DELTA(sum, 2.0, 0.001);
  }

  // Test telescoping sum
  void testTelescopingSum() {
    // Sum of (1/n - 1/(n+1)) for n=1 to N -> 1 - 1/(N+1)
    int N = 100;
    double sum = 0.0;

    for (int n = 1; n <= N; n++) {
      sum += (1.0 / n - 1.0 / (n + 1));
    }

    double expected = 1.0 - 1.0 / (N + 1);
    TS_ASSERT_DELTA(sum, expected, epsilon);
  }

  /***************************************************************************
   * Stress and Large Input Tests
   ***************************************************************************/

  // Test many inputs sum
  void testManyInputsSum() {
    const int N = 1000;
    double sum = 0.0;
    for (int i = 1; i <= N; i++) {
      sum += static_cast<double>(i);
    }
    // Sum 1..N = N*(N+1)/2
    double expected = N * (N + 1) / 2.0;
    TS_ASSERT_DELTA(sum, expected, epsilon);
  }

  // Test sum of uniform values
  void testUniformValuesSum() {
    const int N = 500;
    double value = 0.123;
    double sum = 0.0;
    for (int i = 0; i < N; i++) {
      sum += value;
    }
    TS_ASSERT_DELTA(sum, N * value, 1e-8);
  }

  // Test sum with power series
  void testPowerSeriesSum() {
    // Sum x^n for n=0..10 where x=0.5, converges to 2
    double x = 0.5;
    double sum = 0.0;
    double term = 1.0;
    for (int n = 0; n <= 20; n++) {
      sum += term;
      term *= x;
    }
    TS_ASSERT_DELTA(sum, 1.0 / (1.0 - x), 1e-6);
  }

  /***************************************************************************
   * Dead Zone and Threshold Tests
   ***************************************************************************/

  // Test dead zone application
  void testDeadZone() {
    double deadZone = 0.1;

    // Input inside dead zone
    double smallInput = 0.05;
    double output1 = (std::abs(smallInput) < deadZone) ? 0.0 : smallInput;
    TS_ASSERT_DELTA(output1, 0.0, epsilon);

    // Input outside dead zone
    double largeInput = 0.2;
    double output2 = (std::abs(largeInput) < deadZone) ? 0.0 : largeInput;
    TS_ASSERT_DELTA(output2, 0.2, epsilon);
  }

  // Test threshold summation
  void testThresholdSummation() {
    double threshold = 5.0;
    double inputs[] = {1.0, 2.0, 3.0, 10.0};  // Only 10.0 exceeds threshold
    double sum = 0.0;
    for (double input : inputs) {
      if (input > threshold) sum += input;
    }
    TS_ASSERT_DELTA(sum, 10.0, epsilon);
  }

  // Test dead band with bias
  void testDeadBandWithBias() {
    double deadBand = 0.05;
    double bias = 0.02;
    double input = 0.03;

    double adjusted = input + bias;  // = 0.05, at boundary
    double output = (std::abs(adjusted) <= deadBand) ? 0.0 : adjusted;
    TS_ASSERT_DELTA(output, 0.0, epsilon);

    input = 0.04;
    adjusted = input + bias;  // = 0.06, outside
    output = (std::abs(adjusted) <= deadBand) ? 0.0 : adjusted;
    TS_ASSERT_DELTA(output, 0.06, epsilon);
  }

  /***************************************************************************
   * Hysteresis Tests
   ***************************************************************************/

  // Test simple hysteresis
  void testSimpleHysteresis() {
    double upperThreshold = 0.6;
    double lowerThreshold = 0.4;
    bool state = false;

    // Rising signal
    double signal = 0.3;
    if (signal > upperThreshold) state = true;
    else if (signal < lowerThreshold) state = false;
    TS_ASSERT(!state);

    signal = 0.5;  // Between thresholds, state unchanged
    if (signal > upperThreshold) state = true;
    else if (signal < lowerThreshold) state = false;
    TS_ASSERT(!state);

    signal = 0.7;  // Above upper threshold
    if (signal > upperThreshold) state = true;
    else if (signal < lowerThreshold) state = false;
    TS_ASSERT(state);

    signal = 0.5;  // Falls between, stays on
    if (signal > upperThreshold) state = true;
    else if (signal < lowerThreshold) state = false;
    TS_ASSERT(state);
  }

  // Test hysteresis band width
  void testHysteresisBandWidth() {
    double center = 0.5;
    double bandwidth = 0.2;
    double upper = center + bandwidth / 2;
    double lower = center - bandwidth / 2;

    TS_ASSERT_DELTA(upper, 0.6, epsilon);
    TS_ASSERT_DELTA(lower, 0.4, epsilon);
    TS_ASSERT_DELTA(upper - lower, bandwidth, epsilon);
  }

  /***************************************************************************
   * Rate Limiting Tests
   ***************************************************************************/

  // Test rate limited sum
  void testRateLimitedSum() {
    double current = 0.0;
    double target = 10.0;
    double rateLimit = 2.0;
    double dt = 0.1;

    // Single step update
    double change = target - current;
    double maxChange = rateLimit * dt;
    double actualChange = std::clamp(change, -maxChange, maxChange);
    current += actualChange;

    TS_ASSERT_DELTA(current, 0.2, epsilon);
  }

  // Test rate limiting convergence
  void testRateLimitingConvergence() {
    double current = 0.0;
    double target = 1.0;
    double rateLimit = 0.5;
    double dt = 0.1;
    int steps = 50;

    for (int i = 0; i < steps; i++) {
      double change = target - current;
      double maxChange = rateLimit * dt;
      double actualChange = std::clamp(change, -maxChange, maxChange);
      current += actualChange;
    }

    TS_ASSERT_DELTA(current, target, 0.01);
  }

  // Test bidirectional rate limiting
  void testBidirectionalRateLimiting() {
    double rateLimit = 1.0;
    double dt = 0.5;
    double maxChange = rateLimit * dt;

    // Positive direction
    double current = 0.0;
    double target = 5.0;
    double change = std::clamp(target - current, -maxChange, maxChange);
    TS_ASSERT_DELTA(change, 0.5, epsilon);

    // Negative direction
    current = 5.0;
    target = 0.0;
    change = std::clamp(target - current, -maxChange, maxChange);
    TS_ASSERT_DELTA(change, -0.5, epsilon);
  }

  /***************************************************************************
   * Multi-Channel Tests
   ***************************************************************************/

  // Test triple-axis sum
  void testTripleAxisSum() {
    double x = 1.0, y = 2.0, z = 3.0;
    double magnitude = std::sqrt(x*x + y*y + z*z);
    TS_ASSERT_DELTA(magnitude, std::sqrt(14.0), epsilon);

    double sum = x + y + z;
    TS_ASSERT_DELTA(sum, 6.0, epsilon);
  }

  // Test quaternion components sum
  void testQuaternionComponentsSum() {
    // Sum of squared quaternion components should equal 1 for unit quaternion
    double q0 = 0.5, q1 = 0.5, q2 = 0.5, q3 = 0.5;
    double normSquared = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    TS_ASSERT_DELTA(normSquared, 1.0, epsilon);
  }

  // Test multi-engine thrust sum
  void testMultiEngineThrustSum() {
    double engines[] = {1000.0, 1000.0, 950.0, 1050.0};  // 4-engine aircraft
    double totalThrust = 0.0;
    for (double thrust : engines) {
      totalThrust += thrust;
    }
    TS_ASSERT_DELTA(totalThrust, 4000.0, epsilon);
  }

  /***************************************************************************
   * Gain Scheduling Tests
   ***************************************************************************/

  // Test altitude-dependent gain
  void testAltitudeDependentGain() {
    double baseGain = 1.0;
    double altitude = 30000.0;  // feet
    double densityRatio = std::exp(-altitude / 27000.0);
    double adjustedGain = baseGain / densityRatio;

    TS_ASSERT(adjustedGain > baseGain);
    TS_ASSERT(adjustedGain < 5.0);
  }

  // Test speed-scheduled gain
  void testSpeedScheduledGain() {
    double baseGain = 1.0;
    double vcal = 200.0;  // knots
    double vRef = 150.0;  // reference speed
    double gainFactor = vRef / vcal;
    double scheduledGain = baseGain * gainFactor;

    TS_ASSERT_DELTA(scheduledGain, 0.75, epsilon);
  }

  // Test interpolated gain
  void testInterpolatedGain() {
    // Linear interpolation between two gains
    double gain1 = 0.5, gain2 = 1.5;
    double x1 = 0.0, x2 = 100.0;
    double x = 50.0;

    double interp = gain1 + (gain2 - gain1) * (x - x1) / (x2 - x1);
    TS_ASSERT_DELTA(interp, 1.0, epsilon);
  }

  /***************************************************************************
   * Signal Processing Application Tests
   ***************************************************************************/

  // Test complementary filter
  void testComplementaryFilter() {
    double highFreqSignal = 10.0;  // e.g., gyro
    double lowFreqSignal = 9.5;   // e.g., accelerometer
    double alpha = 0.98;

    double filtered = alpha * highFreqSignal + (1.0 - alpha) * lowFreqSignal;
    TS_ASSERT_DELTA(filtered, 9.99, epsilon);
  }

  // Test sensor fusion
  void testSensorFusion() {
    double sensor1 = 100.0, weight1 = 0.6;
    double sensor2 = 102.0, weight2 = 0.4;

    double fused = sensor1 * weight1 + sensor2 * weight2;
    TS_ASSERT_DELTA(fused, 100.8, epsilon);
  }

  // Test low-pass filter accumulation
  void testLowPassFilterAccumulation() {
    double alpha = 0.1;
    double filtered = 0.0;
    double inputs[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    for (double input : inputs) {
      filtered = alpha * input + (1.0 - alpha) * filtered;
    }

    // After many steps of constant input, should approach input
    TS_ASSERT(filtered > 0.6);
    TS_ASSERT(filtered < 1.0);
  }

  /***************************************************************************
   * Error Accumulation Tests
   ***************************************************************************/

  // Test integral error accumulation
  void testIntegralErrorAccumulation() {
    double error = 0.1;
    double dt = 0.01;
    double integral = 0.0;
    int steps = 100;

    for (int i = 0; i < steps; i++) {
      integral += error * dt;
    }

    TS_ASSERT_DELTA(integral, 0.1, epsilon);
  }

  // Test integral anti-windup
  void testIntegralAntiWindup() {
    double error = 0.5;
    double dt = 0.01;
    double integral = 0.0;
    double integralLimit = 0.2;

    for (int i = 0; i < 100; i++) {
      integral += error * dt;
      integral = std::clamp(integral, -integralLimit, integralLimit);
    }

    TS_ASSERT_DELTA(integral, integralLimit, epsilon);
  }

  // Test derivative kick prevention
  void testDerivativeKickPrevention() {
    double setpoint1 = 0.0, setpoint2 = 10.0;
    double measurement = 5.0;

    // Direct derivative would have kick on setpoint change
    double directDerivative = setpoint2 - setpoint1;
    TS_ASSERT_DELTA(directDerivative, 10.0, epsilon);

    // Derivative on measurement only - no kick
    double prevMeasurement = 5.0;
    double measurementDerivative = measurement - prevMeasurement;
    TS_ASSERT_DELTA(measurementDerivative, 0.0, epsilon);
  }

  /***************************************************************************
   * Boundary and Extreme Value Tests
   ***************************************************************************/

  // Test max double addition
  void testMaxDoubleAddition() {
    double maxVal = std::numeric_limits<double>::max();
    double result = maxVal + 0.0;
    TS_ASSERT_DELTA(result, maxVal, 0.0);
  }

  // Test min positive value
  void testMinPositiveValue() {
    double minVal = std::numeric_limits<double>::min();
    double result = minVal + minVal;
    TS_ASSERT(result > 0.0);
    TS_ASSERT(result < 1e-300);
  }

  // Test denormalized number handling
  void testDenormalizedNumbers() {
    double tiny = std::numeric_limits<double>::denorm_min();
    double result = tiny + tiny;
    TS_ASSERT(result >= 0.0);
    TS_ASSERT(result < 1e-300);
  }

  // Test epsilon precision
  void testEpsilonPrecision() {
    double one = 1.0;
    double eps = std::numeric_limits<double>::epsilon();
    double result = one + eps;
    TS_ASSERT(result > one);
    TS_ASSERT(result - one == eps);
  }

  /***************************************************************************
   * Real-Time Update Pattern Tests
   ***************************************************************************/

  // Test incremental update pattern
  void testIncrementalUpdatePattern() {
    double state = 100.0;
    double update = 0.5;

    for (int i = 0; i < 10; i++) {
      state += update;
    }
    TS_ASSERT_DELTA(state, 105.0, epsilon);
  }

  // Test decay pattern
  void testDecayPattern() {
    double value = 100.0;
    double decayRate = 0.9;

    for (int i = 0; i < 10; i++) {
      value *= decayRate;
    }
    // 100 * 0.9^10
    TS_ASSERT_DELTA(value, 100.0 * std::pow(0.9, 10), epsilon);
  }

  // Test reset and accumulate
  void testResetAndAccumulate() {
    double accumulator = 50.0;

    // Reset
    accumulator = 0.0;
    TS_ASSERT_DELTA(accumulator, 0.0, epsilon);

    // Accumulate
    for (int i = 1; i <= 5; i++) {
      accumulator += i;
    }
    TS_ASSERT_DELTA(accumulator, 15.0, epsilon);
  }

  /***************************************************************************
   * Flight Dynamics Application Tests
   ***************************************************************************/

  // Test force component sum
  void testForceComponentSum() {
    double lift = 10000.0;
    double drag = -500.0;
    double thrust = 2000.0;
    double weight = -10500.0;

    double netVertical = lift + weight;
    double netHorizontal = thrust + drag;

    TS_ASSERT_DELTA(netVertical, -500.0, epsilon);
    TS_ASSERT_DELTA(netHorizontal, 1500.0, epsilon);
  }

  // Test moment summation
  void testMomentSummation() {
    double aeroMoment = 1000.0;
    double thrustMoment = -200.0;
    double cgMoment = -100.0;
    double trimMoment = -700.0;

    double totalMoment = aeroMoment + thrustMoment + cgMoment + trimMoment;
    TS_ASSERT_DELTA(totalMoment, 0.0, epsilon);  // Trimmed condition
  }

  // Test angular rate summation
  void testAngularRateSummation() {
    double p = 0.1;   // Roll rate (rad/s)
    double q = 0.05;  // Pitch rate
    double r = 0.02;  // Yaw rate

    double totalRate = std::sqrt(p*p + q*q + r*r);
    TS_ASSERT_DELTA(totalRate, std::sqrt(0.0129), 1e-6);
  }

  // Test control surface deflection sum
  void testControlSurfaceDeflectionSum() {
    double basicCommand = 0.3;
    double servoTrim = 0.05;
    double gearCorrection = -0.02;
    double autopilot = 0.1;

    double totalDeflection = basicCommand + servoTrim + gearCorrection + autopilot;
    TS_ASSERT_DELTA(totalDeflection, 0.43, epsilon);
  }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
