/*******************************************************************************
 * FGGyroTest.h - Unit tests for FGGyro (rate gyroscope sensor)
 *
 * Tests the mathematical behavior of rate gyroscopes:
 * - Angular rate measurements in body axes
 * - Bias and drift modeling
 * - Noise and quantization
 * - Lag filter response
 * - Scale factor errors
 *
 * Note: FGGyro requires XML element for construction, so these tests focus
 * on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

class FGGyroTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Basic Angular Rate Measurement Tests
   ***************************************************************************/

  // Test angular rate measurement in roll axis
  void testRollRateMeasurement() {
    double p = 10.0 * DEG_TO_RAD;  // Roll rate (rad/s)
    double measured = p * RAD_TO_DEG;  // Convert to deg/s

    TS_ASSERT_DELTA(measured, 10.0, epsilon);
  }

  // Test angular rate measurement in pitch axis
  void testPitchRateMeasurement() {
    double q = 5.0 * DEG_TO_RAD;
    double measured = q * RAD_TO_DEG;

    TS_ASSERT_DELTA(measured, 5.0, epsilon);
  }

  // Test angular rate measurement in yaw axis
  void testYawRateMeasurement() {
    double r = 3.0 * DEG_TO_RAD;
    double measured = r * RAD_TO_DEG;

    TS_ASSERT_DELTA(measured, 3.0, epsilon);
  }

  // Test combined angular rates
  void testCombinedRates() {
    double p = 10.0 * DEG_TO_RAD;
    double q = 5.0 * DEG_TO_RAD;
    double r = 3.0 * DEG_TO_RAD;

    double totalRate = std::sqrt(p*p + q*q + r*r) * RAD_TO_DEG;
    double expected = std::sqrt(10.0*10.0 + 5.0*5.0 + 3.0*3.0);

    TS_ASSERT_DELTA(totalRate, expected, epsilon);
  }

  /***************************************************************************
   * Bias Modeling Tests
   ***************************************************************************/

  // Test constant bias
  void testConstantBias() {
    double trueRate = 10.0;  // deg/s
    double bias = 0.5;       // deg/s

    double measured = trueRate + bias;
    TS_ASSERT_DELTA(measured, 10.5, epsilon);
  }

  // Test bias subtraction for calibration
  void testBiasCalibration() {
    double measured = 10.5;  // With bias
    double bias = 0.5;

    double calibrated = measured - bias;
    TS_ASSERT_DELTA(calibrated, 10.0, epsilon);
  }

  // Test temperature-dependent bias drift
  void testTemperatureBiasDrift() {
    double baseBias = 0.1;  // deg/s at reference temp
    double tempCoeff = 0.01;  // deg/s per degree C
    double refTemp = 25.0;    // C
    double actualTemp = 45.0;

    double bias = baseBias + tempCoeff * (actualTemp - refTemp);
    TS_ASSERT_DELTA(bias, 0.3, epsilon);  // 0.1 + 0.01 * 20
  }

  // Test bias stability over time
  void testBiasStability() {
    double initialBias = 0.1;
    double driftRate = 0.001;  // deg/s per hour
    double hours = 10.0;

    double finalBias = initialBias + driftRate * hours;
    TS_ASSERT_DELTA(finalBias, 0.11, epsilon);
  }

  /***************************************************************************
   * Scale Factor Tests
   ***************************************************************************/

  // Test scale factor error
  void testScaleFactorError() {
    double trueRate = 100.0;  // deg/s
    double scaleFactor = 0.98;  // 2% scale factor error

    double measured = trueRate * scaleFactor;
    TS_ASSERT_DELTA(measured, 98.0, epsilon);
  }

  // Test scale factor correction
  void testScaleFactorCorrection() {
    double measured = 98.0;
    double scaleFactor = 0.98;

    double corrected = measured / scaleFactor;
    TS_ASSERT_DELTA(corrected, 100.0, epsilon);
  }

  // Test non-linear scale factor
  void testNonLinearScaleFactor() {
    double trueRate = 100.0;
    double linearSF = 0.98;
    double nonLinearSF = 0.0001;  // Second-order term

    double measured = trueRate * linearSF + trueRate * trueRate * nonLinearSF;
    TS_ASSERT_DELTA(measured, 99.0, epsilon);  // 98 + 1
  }

  /***************************************************************************
   * Noise Modeling Tests
   ***************************************************************************/

  // Test noise RMS calculation
  void testNoiseRMS() {
    std::mt19937 gen(42);
    std::normal_distribution<> noise(0.0, 0.1);  // 0.1 deg/s std dev

    double sumSquares = 0.0;
    int N = 10000;
    for (int i = 0; i < N; i++) {
      double n = noise(gen);
      sumSquares += n * n;
    }

    double rms = std::sqrt(sumSquares / N);
    TS_ASSERT_DELTA(rms, 0.1, 0.01);  // Should be close to 0.1
  }

  // Test noise has zero mean
  void testNoiseZeroMean() {
    std::mt19937 gen(42);
    std::normal_distribution<> noise(0.0, 0.1);

    double sum = 0.0;
    int N = 10000;
    for (int i = 0; i < N; i++) {
      sum += noise(gen);
    }

    double mean = sum / N;
    TS_ASSERT_DELTA(mean, 0.0, 0.01);  // Should be near zero
  }

  // Test angular random walk
  void testAngularRandomWalk() {
    // ARW accumulates over time: sigma_angle = ARW * sqrt(t)
    double ARW = 0.01;  // deg/sqrt(hr)
    double hours = 1.0;

    double angleError = ARW * std::sqrt(hours);
    TS_ASSERT_DELTA(angleError, 0.01, epsilon);  // 0.01 degrees after 1 hour
  }

  /***************************************************************************
   * Quantization Tests
   ***************************************************************************/

  // Test quantization
  void testQuantization() {
    double resolution = 0.01;  // deg/s
    double trueRate = 10.015;

    double quantized = std::round(trueRate / resolution) * resolution;
    TS_ASSERT_DELTA(quantized, 10.02, epsilon);
  }

  // Test quantization with different resolution
  void testQuantizationFineResolution() {
    double resolution = 0.001;
    double trueRate = 10.0154;

    double quantized = std::round(trueRate / resolution) * resolution;
    TS_ASSERT_DELTA(quantized, 10.015, epsilon);
  }

  /***************************************************************************
   * Lag Filter Tests
   ***************************************************************************/

  // First-order lag filter
  double lagFilter(double input, double prevOutput, double timeConstant, double dt) {
    if (timeConstant <= 0.0) return input;
    double alpha = dt / (timeConstant + dt);
    return prevOutput + alpha * (input - prevOutput);
  }

  // Test lag filter step response
  void testLagFilterStep() {
    double output = 0.0;
    double input = 10.0;
    double tau = 0.1;  // 100ms time constant
    double dt = 0.001;  // 1ms

    // After one time constant
    int steps = static_cast<int>(tau / dt);
    for (int i = 0; i < steps; i++) {
      output = lagFilter(input, output, tau, dt);
    }

    // Should be at ~63.2%
    TS_ASSERT_DELTA(output, input * (1.0 - std::exp(-1.0)), 0.5);
  }

  // Test lag filter convergence
  void testLagFilterConvergence() {
    double output = 0.0;
    double input = 10.0;
    double tau = 0.1;
    double dt = 0.001;

    // After 5 time constants (99.3%)
    int steps = static_cast<int>(5 * tau / dt);
    for (int i = 0; i < steps; i++) {
      output = lagFilter(input, output, tau, dt);
    }

    TS_ASSERT_DELTA(output, input, 0.1);
  }

  // Test no lag
  void testNoLag() {
    double output = lagFilter(10.0, 0.0, 0.0, 0.001);
    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  /***************************************************************************
   * Axis Transformation Tests
   ***************************************************************************/

  // Test sensor axis alignment (sensor tilted 1 degree)
  void testSensorAxisMisalignment() {
    double trueRateX = 100.0;  // deg/s
    double trueRateY = 0.0;
    double misalignment = 1.0 * DEG_TO_RAD;  // 1 degree

    // Misaligned sensor sees some Y rate
    double measuredX = trueRateX * std::cos(misalignment);
    double measuredY = trueRateX * std::sin(misalignment);

    TS_ASSERT_DELTA(measuredX, 99.985, 0.001);  // Slightly reduced
    TS_ASSERT_DELTA(measuredY, 1.745, 0.001);   // Cross-axis coupling
  }

  // Test orthogonality error
  void testOrthogonalityError() {
    double rateX = 10.0;
    double rateY = 10.0;
    double orthoError = 0.01;  // 0.01 rad (about 0.57 deg)

    // Cross-axis coupling due to non-orthogonality
    double measuredX = rateX + rateY * orthoError;
    double measuredY = rateY + rateX * orthoError;

    TS_ASSERT_DELTA(measuredX, 10.1, epsilon);
    TS_ASSERT_DELTA(measuredY, 10.1, epsilon);
  }

  /***************************************************************************
   * Saturation Tests
   ***************************************************************************/

  // Test rate saturation (clipping)
  void testRateSaturation() {
    double maxRate = 300.0;  // deg/s typical MEMS gyro
    double trueRate = 400.0;

    double measured = std::min(std::abs(trueRate), maxRate) * (trueRate > 0 ? 1 : -1);
    TS_ASSERT_DELTA(measured, 300.0, epsilon);  // Clipped
  }

  // Test negative saturation
  void testNegativeSaturation() {
    double maxRate = 300.0;
    double trueRate = -400.0;

    double measured = std::max(-maxRate, std::min(maxRate, trueRate));
    TS_ASSERT_DELTA(measured, -300.0, epsilon);
  }

  // Test within range
  void testWithinRange() {
    double maxRate = 300.0;
    double trueRate = 200.0;

    double measured = std::max(-maxRate, std::min(maxRate, trueRate));
    TS_ASSERT_DELTA(measured, 200.0, epsilon);
  }

  /***************************************************************************
   * Integration Tests (Rate to Angle)
   ***************************************************************************/

  // Test rate integration to angle
  void testRateIntegration() {
    double rate = 10.0;  // deg/s
    double dt = 0.01;    // 10ms
    double angle = 0.0;

    // Integrate for 1 second
    for (int i = 0; i < 100; i++) {
      angle += rate * dt;
    }

    TS_ASSERT_DELTA(angle, 10.0, epsilon);  // 10 deg/s * 1s = 10 deg
  }

  // Test varying rate integration
  void testVaryingRateIntegration() {
    double angle = 0.0;
    double dt = 0.01;

    // Integrate linearly increasing rate
    for (int i = 0; i < 100; i++) {
      double rate = static_cast<double>(i);  // 0 to 99 deg/s
      angle += rate * dt;
    }

    // Sum of 0+1+2+...+99 = 4950, times dt=0.01 = 49.5 degrees
    TS_ASSERT_DELTA(angle, 49.5, epsilon);
  }

  /***************************************************************************
   * Gyro-specific Application Tests
   ***************************************************************************/

  // Test turn coordinator calculation
  void testTurnCoordinator() {
    double yawRate = 3.0 * DEG_TO_RAD;  // Standard rate turn
    double rollRate = 0.0;

    // Turn coordinator shows mostly yaw rate
    double tcInput = yawRate;
    double tcOutput = tcInput * RAD_TO_DEG;

    TS_ASSERT_DELTA(tcOutput, 3.0, epsilon);  // 3 deg/s = standard rate
  }

  // Test attitude indicator (simplified rate-based)
  void testRateBasedAttitude() {
    double rollRate = 10.0;  // deg/s
    double dt = 0.1;         // 100ms
    double roll = 0.0;

    // Integrate roll rate
    for (int i = 0; i < 30; i++) {
      roll += rollRate * dt;
    }

    TS_ASSERT_DELTA(roll, 30.0, epsilon);  // 30 degrees of roll
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero rate
  void testZeroRate() {
    double bias = 0.01;  // Small bias
    double trueRate = 0.0;

    double measured = trueRate + bias;
    TS_ASSERT_DELTA(measured, 0.01, epsilon);  // Only see bias
  }

  // Test very high rate
  void testVeryHighRate() {
    double rate = 1000.0;  // deg/s (high but realistic for aerobatic)
    double maxRate = 2000.0;

    bool withinRange = (std::abs(rate) <= maxRate);
    TS_ASSERT(withinRange);
  }

  // Test sign change
  void testSignChange() {
    double rate1 = 10.0;
    double rate2 = -10.0;

    TS_ASSERT(rate1 * rate2 < 0);  // Opposite signs
    TS_ASSERT_DELTA(std::abs(rate1), std::abs(rate2), epsilon);  // Same magnitude
  }
};
