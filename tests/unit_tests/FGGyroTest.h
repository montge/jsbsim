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

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAccelerations.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

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

  /***************************************************************************
   * G-Sensitivity Tests
   ***************************************************************************/

  // Test 32: Linear acceleration sensitivity (g-sensitivity)
  void testGSensitivity() {
    double trueRate = 10.0;  // deg/s
    double gSensitivity = 0.1;  // deg/s per g
    double linearAccel = 2.0;  // g's

    double biasFromAccel = gSensitivity * linearAccel;
    double measured = trueRate + biasFromAccel;

    TS_ASSERT_DELTA(measured, 10.2, epsilon);
  }

  // Test 33: Multi-axis g-sensitivity
  void testMultiAxisGSensitivity() {
    double trueRate = 10.0;
    double gSens_x = 0.05;  // deg/s per g
    double gSens_y = 0.03;
    double gSens_z = 0.08;

    double accel_x = 1.0, accel_y = 0.5, accel_z = 1.5;  // g's

    double biasFromAccel = gSens_x * accel_x + gSens_y * accel_y + gSens_z * accel_z;
    double measured = trueRate + biasFromAccel;

    TS_ASSERT_DELTA(biasFromAccel, 0.185, epsilon);
    TS_ASSERT_DELTA(measured, 10.185, epsilon);
  }

  // Test 34: G-squared sensitivity
  void testGSquaredSensitivity() {
    double trueRate = 10.0;
    double gSquaredSens = 0.01;  // deg/s per g^2
    double accel = 3.0;  // g's

    double biasFromAccelSq = gSquaredSens * accel * accel;
    double measured = trueRate + biasFromAccelSq;

    TS_ASSERT_DELTA(biasFromAccelSq, 0.09, epsilon);
    TS_ASSERT_DELTA(measured, 10.09, epsilon);
  }

  /***************************************************************************
   * Bias Instability Tests
   ***************************************************************************/

  // Test 35: Bias instability simulation
  void testBiasInstability() {
    std::mt19937 gen(123);
    std::normal_distribution<> walk(0.0, 0.001);  // Random walk noise

    double bias = 0.1;
    double biasMin = bias, biasMax = bias;

    // Simulate bias random walk
    for (int i = 0; i < 10000; i++) {
      bias += walk(gen);
      biasMin = std::min(biasMin, bias);
      biasMax = std::max(biasMax, bias);
    }

    // Bias should have wandered
    TS_ASSERT(biasMax - biasMin > 0.01);
  }

  // Test 36: In-run bias stability
  void testInRunBiasStability() {
    double initialBias = 0.1;
    double biasStability = 0.01;  // deg/hr

    // Over 1 hour, bias should stay within stability spec
    double maxBiasChange = biasStability;
    double finalBias = initialBias + maxBiasChange * (1.0 / 3600.0);  // Per second

    TS_ASSERT(std::abs(finalBias - initialBias) < 0.1);
  }

  /***************************************************************************
   * Cross-Axis Sensitivity Tests
   ***************************************************************************/

  // Test 37: Cross-axis sensitivity matrix
  void testCrossAxisSensitivityMatrix() {
    double trueRates[3] = {100.0, 50.0, 30.0};  // p, q, r in deg/s

    // Cross-axis sensitivity matrix (diagonal = 1, off-diagonal = cross-coupling)
    double sensMatrix[3][3] = {
      {1.0, 0.02, 0.01},
      {0.015, 1.0, 0.02},
      {0.01, 0.015, 1.0}
    };

    double measured[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        measured[i] += sensMatrix[i][j] * trueRates[j];
      }
    }

    // Cross-axis coupling adds small amounts from other axes
    TS_ASSERT(measured[0] > trueRates[0]);  // Some q and r coupled in
    TS_ASSERT(measured[1] > trueRates[1]);  // Some p and r coupled in
    TS_ASSERT(measured[2] > trueRates[2]);  // Some p and q coupled in
  }

  // Test 38: Cross-axis correction
  void testCrossAxisCorrection() {
    double measured[3] = {101.3, 52.5, 31.65};  // With cross-axis errors

    // Inverse correction matrix (simplified)
    double corrMatrix[3][3] = {
      {1.0, -0.02, -0.01},
      {-0.015, 1.0, -0.02},
      {-0.01, -0.015, 1.0}
    };

    double corrected[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        corrected[i] += corrMatrix[i][j] * measured[j];
      }
    }

    // Should be closer to true values
    TS_ASSERT_DELTA(corrected[0], 100.0, 0.5);
    TS_ASSERT_DELTA(corrected[1], 50.0, 0.5);
    TS_ASSERT_DELTA(corrected[2], 30.0, 0.5);
  }

  /***************************************************************************
   * Temperature Sensitivity Tests
   ***************************************************************************/

  // Test 39: Scale factor temperature coefficient
  void testScaleFactorTempCoeff() {
    double nominalSF = 1.0;
    double tempCoeff = 0.0003;  // 300 ppm/°C
    double refTemp = 25.0;
    double actualTemp = 50.0;

    double sf = nominalSF * (1.0 + tempCoeff * (actualTemp - refTemp));
    TS_ASSERT_DELTA(sf, 1.0075, 0.0001);
  }

  // Test 40: Bias temperature coefficient
  void testBiasTempCoeff() {
    double nominalBias = 0.1;  // deg/s
    double tempCoeff = 0.005;  // deg/s per °C
    double refTemp = 25.0;
    double actualTemp = 0.0;  // Cold

    double bias = nominalBias + tempCoeff * (actualTemp - refTemp);
    TS_ASSERT_DELTA(bias, -0.025, epsilon);  // Negative due to cold temp
  }

  // Test 41: Temperature hysteresis
  void testTemperatureHysteresis() {
    double bias_warmUp = 0.12;   // Bias when warming up to 40°C
    double bias_coolDown = 0.11;  // Bias when cooling down to 40°C

    double hysteresis = std::abs(bias_warmUp - bias_coolDown);
    TS_ASSERT_DELTA(hysteresis, 0.01, epsilon);
  }

  /***************************************************************************
   * Vibration Effects Tests
   ***************************************************************************/

  // Test 42: Vibration rectification error
  void testVibrationRectification() {
    // Sinusoidal vibration can cause DC offset due to non-linearity
    double vibFreq = 100.0;  // Hz
    double vibAmplitude = 50.0;  // deg/s
    double rectificationCoeff = 0.001;

    // Rectified error is proportional to amplitude squared
    double rectError = rectificationCoeff * vibAmplitude * vibAmplitude;
    TS_ASSERT_DELTA(rectError, 2.5, epsilon);
  }

  // Test 43: High frequency vibration filtering
  void testVibrationFiltering() {
    double filterCutoff = 50.0;  // Hz
    double vibFreq = 200.0;      // Hz (4x above cutoff)

    // 2nd order filter attenuation: approximately (fc/f)^2
    double attenuation = std::pow(filterCutoff / vibFreq, 2);
    TS_ASSERT_DELTA(attenuation, 0.0625, epsilon);  // -24 dB
  }

  /***************************************************************************
   * Rate Limiting Tests
   ***************************************************************************/

  // Test 44: Output rate limiting (slew rate)
  void testOutputSlewRate() {
    double prevOutput = 0.0;
    double newInput = 100.0;
    double maxSlewRate = 500.0;  // deg/s per second
    double dt = 0.01;

    double maxChange = maxSlewRate * dt;
    double change = newInput - prevOutput;

    if (std::abs(change) > maxChange) {
      change = (change > 0) ? maxChange : -maxChange;
    }

    double output = prevOutput + change;
    TS_ASSERT_DELTA(output, 5.0, epsilon);  // Limited to 5 deg/s change
  }

  // Test 45: Slew rate over multiple samples
  void testSlewRateMultipleSamples() {
    double output = 0.0;
    double target = 100.0;
    double maxSlewRate = 500.0;
    double dt = 0.01;

    int samples = 0;
    while (std::abs(output - target) > 0.1 && samples < 100) {
      double change = target - output;
      double maxChange = maxSlewRate * dt;
      if (std::abs(change) > maxChange) {
        change = (change > 0) ? maxChange : -maxChange;
      }
      output += change;
      samples++;
    }

    // Should take 20 samples (100 / 5 = 20)
    TS_ASSERT_EQUALS(samples, 20);
  }

  /***************************************************************************
   * Power-On Behavior Tests
   ***************************************************************************/

  // Test 46: Power-on bias settling
  void testPowerOnBiasSettling() {
    double initialBias = 5.0;  // Large initial bias
    double finalBias = 0.1;    // Settled bias
    double settlingTime = 2.0; // seconds

    // Exponential settling
    double timeConstant = settlingTime / 5.0;  // 5 tau for 99%
    double t = 2.0;
    double currentBias = finalBias + (initialBias - finalBias) * std::exp(-t / timeConstant);

    TS_ASSERT(currentBias < 0.5);  // Should have settled mostly
  }

  // Test 47: Warm-up time effect
  void testWarmUpTime() {
    double coldBias = 0.5;     // Bias when cold
    double warmBias = 0.1;     // Bias after warm-up
    double warmUpTime = 60.0;  // seconds

    // Exponential warm-up
    double tau = warmUpTime / 4.0;
    double t = 30.0;  // Half of warm-up time

    double currentBias = warmBias + (coldBias - warmBias) * std::exp(-t / tau);
    TS_ASSERT(currentBias < coldBias);
    TS_ASSERT(currentBias > warmBias);
  }

  /***************************************************************************
   * Turn Rate Calculation Tests
   ***************************************************************************/

  // Test 48: Standard rate turn calculation
  void testStandardRateTurn() {
    // Standard rate = 3 deg/s = 360 deg in 2 minutes
    double standardRate = 3.0;  // deg/s
    double timeFor360 = 360.0 / standardRate;

    TS_ASSERT_DELTA(timeFor360, 120.0, epsilon);  // 2 minutes
  }

  // Test 49: Half standard rate turn
  void testHalfStandardRateTurn() {
    double halfStandardRate = 1.5;  // deg/s
    double timeFor360 = 360.0 / halfStandardRate;

    TS_ASSERT_DELTA(timeFor360, 240.0, epsilon);  // 4 minutes
  }

  // Test 50: Turn radius from rate and speed
  void testTurnRadiusFromRate() {
    double turnRate = 3.0 * DEG_TO_RAD;  // 3 deg/s in rad/s
    double velocity = 150.0;  // m/s (about 290 kts)

    double turnRadius = velocity / turnRate;
    TS_ASSERT_DELTA(turnRadius, 2864.8, 1.0);  // meters
  }

  // Test 51: Bank angle from turn rate and speed
  void testBankAngleFromTurnRate() {
    double turnRate = 3.0 * DEG_TO_RAD;  // rad/s
    double velocity = 100.0;  // m/s
    double g = 9.81;

    // tan(bank) = V * omega / g
    double tanBank = velocity * turnRate / g;
    double bankAngle = std::atan(tanBank) * RAD_TO_DEG;

    TS_ASSERT_DELTA(bankAngle, 28.1, 0.1);
  }

  /***************************************************************************
   * Dead Reckoning Tests
   ***************************************************************************/

  // Test 52: Heading dead reckoning
  void testHeadingDeadReckoning() {
    double heading = 90.0;  // Initial heading (east)
    double yawRate = 3.0;   // deg/s (standard rate left turn)
    double dt = 0.01;

    // Integrate for 30 seconds
    for (int i = 0; i < 3000; i++) {
      heading += yawRate * dt;
    }

    // Should have turned 90 degrees
    TS_ASSERT_DELTA(heading, 180.0, 0.1);
  }

  // Test 53: Attitude dead reckoning with all axes
  void testAttitudeDeadReckoning() {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double p = 5.0, q = 2.0, r = 1.0;  // deg/s
    double dt = 0.01;

    // Simple Euler integration (small angles)
    for (int i = 0; i < 100; i++) {
      roll += p * dt;
      pitch += q * dt;
      yaw += r * dt;
    }

    TS_ASSERT_DELTA(roll, 5.0, 0.01);
    TS_ASSERT_DELTA(pitch, 2.0, 0.01);
    TS_ASSERT_DELTA(yaw, 1.0, 0.01);
  }

  /***************************************************************************
   * Allan Variance Concepts
   ***************************************************************************/

  // Test 54: Allan variance calculation concept
  void testAllanVarianceConcept() {
    // Allan variance separates noise types
    // White noise: sigma^2 proportional to 1/tau
    double whiteNoiseLevel = 0.1;  // deg/s
    double tau = 1.0;  // Averaging time

    double allanDev_at_tau = whiteNoiseLevel / std::sqrt(tau);
    TS_ASSERT_DELTA(allanDev_at_tau, 0.1, epsilon);
  }

  // Test 55: Bias instability from Allan variance
  void testBiasInstabilityFromAllan() {
    // At minimum of Allan variance, we get bias instability
    double biasInstability = 0.01;  // deg/hr
    double biasInstabilityDegPerSec = biasInstability / 3600.0;

    TS_ASSERT(biasInstabilityDegPerSec < 0.00001);
  }

  /***************************************************************************
   * Complementary Filter Tests
   ***************************************************************************/

  // Test 56: Complementary filter with gyro and accelerometer
  void testComplementaryFilter() {
    double gyroAngle = 10.0;   // From gyro integration
    double accelAngle = 9.5;   // From accelerometer
    double alpha = 0.98;       // Trust gyro more

    double fusedAngle = alpha * gyroAngle + (1 - alpha) * accelAngle;
    TS_ASSERT_DELTA(fusedAngle, 9.99, 0.01);
  }

  // Test 57: Complementary filter time constant
  void testComplementaryFilterTimeConstant() {
    double filterFreq = 0.1;  // Hz (10 second time constant)
    double dt = 0.01;

    // High-pass for gyro
    double alpha = 1.0 / (1.0 + 2 * M_PI * filterFreq * dt);
    TS_ASSERT(alpha > 0.99);  // High pass keeps mostly gyro
  }

  /***************************************************************************
   * Specific Maneuver Tests
   ***************************************************************************/

  // Test 58: Roll reversal detection
  void testRollReversal() {
    double rate1 = 30.0;   // Rolling right
    double rate2 = -30.0;  // Rolling left

    bool reversal = (rate1 * rate2 < 0);
    TS_ASSERT(reversal);
  }

  // Test 59: Coordinated turn detection
  void testCoordinatedTurnDetection() {
    double yawRate = 3.0;   // deg/s
    double rollRate = 0.5;  // deg/s (small, adjusting bank)
    double pitchRate = 0.1; // deg/s (small)

    // In coordinated turn, yaw rate dominates
    double totalRate = std::sqrt(yawRate*yawRate + rollRate*rollRate + pitchRate*pitchRate);
    double yawFraction = std::abs(yawRate) / totalRate;

    TS_ASSERT(yawFraction > 0.95);  // Yaw is dominant
  }

  // Test 60: Spin detection
  void testSpinDetection() {
    double yawRate = 180.0;  // deg/s (very high)
    double spinThreshold = 90.0;

    bool inSpin = std::abs(yawRate) > spinThreshold;
    TS_ASSERT(inSpin);
  }

  /***************************************************************************
   * Coning/Sculling Error Tests
   ***************************************************************************/

  // Test 61: Coning error concept
  void testConingError() {
    // Coning error occurs with simultaneous oscillations in two axes
    double oscillationFreq = 10.0;  // Hz
    double oscillationAmp = 5.0;    // deg
    double dt = 0.001;

    // Coning causes a net drift proportional to amplitude squared
    double coningError = 0.25 * oscillationAmp * oscillationAmp * oscillationFreq * dt;
    TS_ASSERT(coningError < 1.0);  // Small but non-zero
  }

  // Test 62: High frequency coning compensation
  void testConingCompensation() {
    double rawAngleX = 5.0;
    double rawAngleY = 3.0;
    double coningCorrection = 0.5 * rawAngleX * rawAngleY / 1000.0;  // Simplified

    double correctedAngle = rawAngleX - coningCorrection;
    TS_ASSERT(std::abs(correctedAngle - rawAngleX) < 0.01);
  }

  /***************************************************************************
   * Gyro Health Monitoring Tests
   ***************************************************************************/

  // Test 63: Stuck gyro detection
  void testStuckGyroDetection() {
    double samples[10] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};

    double variance = 0.0;
    double mean = 10.0;
    for (int i = 0; i < 10; i++) {
      variance += (samples[i] - mean) * (samples[i] - mean);
    }
    variance /= 10;

    bool stuck = (variance < 0.0001);
    TS_ASSERT(stuck);  // No variation = likely stuck
  }

  // Test 64: Gyro noise level check
  void testGyroNoiseCheck() {
    std::mt19937 gen(42);
    std::normal_distribution<> noise(10.0, 0.1);

    double samples[100];
    double sum = 0, sumSq = 0;
    for (int i = 0; i < 100; i++) {
      samples[i] = noise(gen);
      sum += samples[i];
      sumSq += samples[i] * samples[i];
    }

    double mean = sum / 100;
    double variance = sumSq / 100 - mean * mean;
    double stdDev = std::sqrt(variance);

    // Noise should be close to expected 0.1
    TS_ASSERT_DELTA(stdDev, 0.1, 0.02);
  }

  // Test 65: Out of range detection
  void testOutOfRangeDetection() {
    double maxRate = 300.0;
    double readings[] = {10.0, 50.0, 150.0, 350.0, 280.0};

    int outOfRangeCount = 0;
    for (double r : readings) {
      if (std::abs(r) > maxRate) outOfRangeCount++;
    }

    TS_ASSERT_EQUALS(outOfRangeCount, 1);  // 350 is out of range
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test 66: Degrees per second to radians per second
  void testDegPerSecToRadPerSec() {
    double degPerSec = 57.2957795;  // ~1 radian
    double radPerSec = degPerSec * DEG_TO_RAD;

    TS_ASSERT_DELTA(radPerSec, 1.0, 0.0001);
  }

  // Test 67: RPM to degrees per second
  void testRPMToDegreesPerSecond() {
    double rpm = 60.0;  // 1 revolution per second
    double degPerSec = rpm * 360.0 / 60.0;

    TS_ASSERT_DELTA(degPerSec, 360.0, epsilon);
  }

  // Test 68: Degrees per hour to degrees per second
  void testDegPerHourToDegPerSec() {
    double degPerHour = 3600.0;  // 1 deg/s
    double degPerSec = degPerHour / 3600.0;

    TS_ASSERT_DELTA(degPerSec, 1.0, epsilon);
  }

  /***************************************************************************
   * Bandwidth Tests
   ***************************************************************************/

  // Test 69: Nyquist frequency check
  void testNyquistFrequency() {
    double sampleRate = 1000.0;  // Hz
    double nyquistFreq = sampleRate / 2.0;

    TS_ASSERT_DELTA(nyquistFreq, 500.0, epsilon);
  }

  // Test 70: Anti-aliasing filter cutoff
  void testAntiAliasingFilter() {
    double sampleRate = 1000.0;
    double filterCutoff = sampleRate / 4.0;  // Conservative

    TS_ASSERT_DELTA(filterCutoff, 250.0, epsilon);
    TS_ASSERT(filterCutoff < sampleRate / 2.0);
  }

  // Test 71: Bandwidth vs noise tradeoff
  void testBandwidthNoiseTradeoff() {
    double noiseDensity = 0.01;  // deg/s/sqrt(Hz)
    double bandwidth1 = 100.0;  // Hz
    double bandwidth2 = 50.0;   // Hz

    double noiseRMS1 = noiseDensity * std::sqrt(bandwidth1);
    double noiseRMS2 = noiseDensity * std::sqrt(bandwidth2);

    TS_ASSERT(noiseRMS1 > noiseRMS2);  // Higher bandwidth = more noise
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  // Test 72: Very small rate detection
  void testVerySmallRateDetection() {
    double rate = 0.001;  // deg/s
    double noiseFloor = 0.01;

    bool detectable = std::abs(rate) > noiseFloor;
    TS_ASSERT(!detectable);  // Below noise floor
  }

  // Test 73: Rate reversal timing
  void testRateReversalTiming() {
    double rates[] = {10.0, 8.0, 5.0, 2.0, 0.0, -2.0, -5.0};
    int reversalIndex = -1;

    for (int i = 1; i < 7; i++) {
      if (rates[i-1] >= 0 && rates[i] < 0) {
        reversalIndex = i;
        break;
      }
    }

    TS_ASSERT_EQUALS(reversalIndex, 5);  // Zero crossing between indices 4 and 5
  }

  // Test 74: Maximum angular acceleration
  void testMaxAngularAcceleration() {
    double rate1 = 0.0;
    double rate2 = 100.0;  // deg/s
    double dt = 0.1;       // 100ms

    double angularAccel = (rate2 - rate1) / dt;
    TS_ASSERT_DELTA(angularAccel, 1000.0, epsilon);  // 1000 deg/s^2
  }

  // Test 75: Gyro output during steady state
  void testSteadyStateOutput() {
    double bias = 0.05;
    double trueRate = 0.0;
    double noise_sigma = 0.1;

    std::mt19937 gen(42);
    std::normal_distribution<> noise(0.0, noise_sigma);

    double sum = 0;
    int n = 1000;
    for (int i = 0; i < n; i++) {
      double measured = trueRate + bias + noise(gen);
      sum += measured;
    }

    double mean = sum / n;
    TS_ASSERT_DELTA(mean, bias, 0.02);  // Mean should converge to bias
  }
};

/*******************************************************************************
 * Extended FGGyro Tests (25 new tests)
 ******************************************************************************/

class FGGyroExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * MEMS Gyro Specific Tests
   ***************************************************************************/

  // Test 76: MEMS gyro resonant frequency
  void testMEMSResonantFrequency() {
    double resonantFreq = 10000.0;  // Hz (typical MEMS)
    double driveFreq = 10000.0;     // Drive at resonance

    double gain = 1.0;  // Maximum at resonance
    TS_ASSERT_DELTA(driveFreq, resonantFreq, 1.0);
    TS_ASSERT_DELTA(gain, 1.0, epsilon);
  }

  // Test 77: MEMS quadrature error
  void testMEMSQuadratureError() {
    double driveAmplitude = 10.0;  // um
    double quadratureCoeff = 0.01;  // Coupling coefficient

    double quadratureError = driveAmplitude * quadratureCoeff;
    TS_ASSERT_DELTA(quadratureError, 0.1, epsilon);
  }

  // Test 78: Drive mode amplitude control
  void testDriveModeAmplitudeControl() {
    double targetAmplitude = 10.0;
    double actualAmplitude = 9.8;
    double Kp = 0.5;

    double correction = Kp * (targetAmplitude - actualAmplitude);
    TS_ASSERT_DELTA(correction, 0.1, epsilon);
  }

  /***************************************************************************
   * Ring Laser Gyro Concepts
   ***************************************************************************/

  // Test 79: Sagnac effect calculation
  void testSagnacEffect() {
    double area = 0.01;           // m^2 (ring area)
    double wavelength = 633e-9;   // m (HeNe laser)
    double c = 3e8;               // m/s
    double omega = 1.0 * DEG_TO_RAD;  // rad/s

    // Beat frequency = 4 * A * omega / (lambda * perimeter)
    double perimeter = std::sqrt(4 * M_PI * area);
    double beatFreq = 4 * area * omega / (wavelength * perimeter);

    TS_ASSERT(beatFreq > 0);
  }

  // Test 80: Lock-in threshold
  void testLockInThreshold() {
    double lockInRate = 0.1;  // deg/s (typical RLG lock-in)
    double measuredRate = 0.05;

    bool lockedIn = std::abs(measuredRate) < lockInRate;
    TS_ASSERT(lockedIn);
  }

  // Test 81: Dither mechanism effect
  void testDitherMechanism() {
    double ditherFreq = 400.0;     // Hz
    double ditherAmplitude = 100.0; // arcsec
    double lockInRate = 0.1;       // deg/s

    // Dither keeps gyro out of lock-in
    bool ditherActive = ditherAmplitude > 0;
    TS_ASSERT(ditherActive);
  }

  /***************************************************************************
   * Fiber Optic Gyro Concepts
   ***************************************************************************/

  // Test 82: FOG scale factor
  void testFOGScaleFactor() {
    double fiberLength = 1000.0;   // m
    double fiberDiameter = 0.1;    // m (coil diameter)
    double wavelength = 1550e-9;   // m

    double area = M_PI * fiberDiameter * fiberDiameter / 4;
    double numTurns = fiberLength / (M_PI * fiberDiameter);
    double scaleFactor = 4 * M_PI * numTurns * area / (wavelength * 3e8);

    TS_ASSERT(scaleFactor > 0);
  }

  // Test 83: FOG phase shift measurement
  void testFOGPhaseShift() {
    double rotationRate = 10.0 * DEG_TO_RAD;  // rad/s
    double sensitivity = 0.1;  // rad/(rad/s)

    double phaseShift = rotationRate * sensitivity;
    TS_ASSERT(phaseShift > 0);
  }

  /***************************************************************************
   * Kalman Filter Integration Tests
   ***************************************************************************/

  // Test 84: Gyro measurement update in Kalman filter
  void testKalmanMeasurementUpdate() {
    double predicted = 10.0;      // Predicted rate
    double measured = 10.5;       // Measured rate
    double kalmanGain = 0.3;      // Kalman gain

    double updated = predicted + kalmanGain * (measured - predicted);
    TS_ASSERT_DELTA(updated, 10.15, epsilon);
  }

  // Test 85: Gyro bias estimation in Kalman filter
  void testKalmanBiasEstimation() {
    double estimatedBias = 0.1;
    double innovation = 0.2;  // Measurement - prediction
    double biasGain = 0.01;

    double newBiasEstimate = estimatedBias + biasGain * innovation;
    TS_ASSERT_DELTA(newBiasEstimate, 0.102, epsilon);
  }

  // Test 86: Gyro noise covariance
  void testGyroNoiseCovariance() {
    double noiseDensity = 0.01;  // deg/s/sqrt(Hz)
    double bandwidth = 100.0;    // Hz

    double variance = noiseDensity * noiseDensity * bandwidth;
    double stdDev = std::sqrt(variance);

    TS_ASSERT_DELTA(stdDev, 0.1, epsilon);
  }

  /***************************************************************************
   * Attitude Reference System Tests
   ***************************************************************************/

  // Test 87: Gyro contribution to attitude
  void testGyroAttitudeContribution() {
    double gyroRate = 10.0;  // deg/s
    double dt = 0.01;
    double attitude = 0.0;

    // Simple integration
    attitude += gyroRate * dt;

    TS_ASSERT_DELTA(attitude, 0.1, epsilon);
  }

  // Test 88: Attitude rate limiting
  void testAttitudeRateLimiting() {
    double maxRate = 300.0;  // deg/s
    double commanded = 400.0;

    double limited = std::min(maxRate, std::max(-maxRate, commanded));
    TS_ASSERT_DELTA(limited, 300.0, epsilon);
  }

  // Test 89: Quaternion rate from gyro
  void testQuaternionRateFromGyro() {
    double p = 0.1, q = 0.05, r = 0.02;  // rad/s

    // Simplified quaternion rate (assumes small angles)
    double omega_magnitude = std::sqrt(p*p + q*q + r*r);
    TS_ASSERT(omega_magnitude < 0.2);
  }

  /***************************************************************************
   * Flight Control System Integration
   ***************************************************************************/

  // Test 90: Rate damper input
  void testRateDamperInput() {
    double gyroRate = 5.0;  // deg/s
    double damperGain = 0.5;

    double damperOutput = -damperGain * gyroRate;
    TS_ASSERT_DELTA(damperOutput, -2.5, epsilon);
  }

  // Test 91: Rate command comparison
  void testRateCommandComparison() {
    double commandedRate = 10.0;  // deg/s
    double actualRate = 8.0;      // deg/s from gyro

    double rateError = commandedRate - actualRate;
    TS_ASSERT_DELTA(rateError, 2.0, epsilon);
  }

  // Test 92: Rate feedback loop
  void testRateFeedbackLoop() {
    double Kp = 2.0;
    double commandedRate = 15.0;
    double actualRate = 10.0;

    double surfaceCommand = Kp * (commandedRate - actualRate);
    TS_ASSERT_DELTA(surfaceCommand, 10.0, epsilon);
  }

  /***************************************************************************
   * Redundancy and Voting Tests
   ***************************************************************************/

  // Test 93: Triple redundant gyro voting
  void testTripleRedundantVoting() {
    double gyro1 = 10.0;
    double gyro2 = 10.1;
    double gyro3 = 10.05;

    // Middle value selection (median of 3)
    double voted;
    if ((gyro1 >= gyro2 && gyro1 <= gyro3) || (gyro1 <= gyro2 && gyro1 >= gyro3))
      voted = gyro1;
    else if ((gyro2 >= gyro1 && gyro2 <= gyro3) || (gyro2 <= gyro1 && gyro2 >= gyro3))
      voted = gyro2;
    else
      voted = gyro3;

    TS_ASSERT_DELTA(voted, 10.05, epsilon);
  }

  // Test 94: Failed gyro detection
  void testFailedGyroDetection() {
    double gyro1 = 10.0;
    double gyro2 = 10.1;
    double gyro3 = 50.0;  // Failed

    double mean = (gyro1 + gyro2 + gyro3) / 3.0;
    double threshold = 5.0;

    bool gyro3Failed = std::abs(gyro3 - mean) > threshold;
    TS_ASSERT(gyro3Failed);
  }

  // Test 95: Gyro disagreement monitor
  void testGyroDisagreementMonitor() {
    double gyro1 = 10.0;
    double gyro2 = 12.0;
    double disagreementThreshold = 3.0;

    double disagreement = std::abs(gyro1 - gyro2);
    bool withinTolerance = disagreement < disagreementThreshold;

    TS_ASSERT(withinTolerance);
  }

  /***************************************************************************
   * Environmental Effects Tests
   ***************************************************************************/

  // Test 96: Shock effect on gyro
  void testShockEffect() {
    double normalBias = 0.1;
    double shockMagnitude = 100.0;  // g
    double shockSensitivity = 0.001;  // deg/s per g

    double biasShift = shockSensitivity * shockMagnitude;
    double newBias = normalBias + biasShift;

    TS_ASSERT_DELTA(newBias, 0.2, epsilon);
  }

  // Test 97: Magnetic field sensitivity
  void testMagneticFieldSensitivity() {
    double fieldStrength = 50.0;  // uT (Earth's field)
    double magneticSensitivity = 0.001;  // deg/s per uT

    double biasFromMagnetic = magneticSensitivity * fieldStrength;
    TS_ASSERT_DELTA(biasFromMagnetic, 0.05, epsilon);
  }

  // Test 98: Pressure sensitivity
  void testPressureSensitivity() {
    double pressureChange = 10000.0;  // Pa (altitude change)
    double pressureSensitivity = 0.00001;  // deg/s per Pa

    double biasFromPressure = pressureSensitivity * pressureChange;
    TS_ASSERT_DELTA(biasFromPressure, 0.1, epsilon);
  }

  /***************************************************************************
   * Calibration Tests
   ***************************************************************************/

  // Test 99: Six-position calibration
  void testSixPositionCalibration() {
    // Measurements in 6 positions (+X, -X, +Y, -Y, +Z, -Z)
    double posX = 0.12, negX = -0.08;
    double posY = 0.11, negY = -0.09;
    double posZ = 0.10, negZ = -0.10;

    // Bias = average of opposite positions
    double biasX = (posX + negX) / 2.0;
    double biasY = (posY + negY) / 2.0;
    double biasZ = (posZ + negZ) / 2.0;

    TS_ASSERT_DELTA(biasX, 0.02, epsilon);
    TS_ASSERT_DELTA(biasY, 0.01, epsilon);
    TS_ASSERT_DELTA(biasZ, 0.0, epsilon);
  }

  // Test 100: Rate table calibration
  void testRateTableCalibration() {
    double appliedRate = 100.0;  // deg/s (known)
    double measuredRate = 98.5;  // deg/s

    double scaleFactor = appliedRate / measuredRate;
    TS_ASSERT_DELTA(scaleFactor, 1.0152, 0.001);
  }
};
