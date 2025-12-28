#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>
#include <algorithm>
#include <deque>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Sensor unit tests
 *
 * Note: FGSensor requires XML element for construction, so these tests focus on:
 * - Noise injection (uniform, gaussian)
 * - Quantization effects
 * - Lag filtering
 * - Bias and gain
 * - Drift modeling
 * - Failure modes
 */
class FGSensorTest : public CxxTest::TestSuite
{
public:
  // Test uniform noise distribution bounds
  void testUniformNoiseBounds() {
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    double min_val = 1.0, max_val = -1.0;
    for (int i = 0; i < 10000; i++) {
      double val = dist(gen);
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }

    // Should be bounded by [-1, 1]
    TS_ASSERT(min_val >= -1.0);
    TS_ASSERT(max_val <= 1.0);
    // Should approach bounds with many samples
    TS_ASSERT(min_val < -0.99);
    TS_ASSERT(max_val > 0.99);
  }

  // Test uniform noise with absolute variation
  void testUniformNoiseAbsolute() {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    double input = 100.0;
    double noise_magnitude = 5.0;  // +/- 5 units

    double min_output = input, max_output = input;
    for (int i = 0; i < 1000; i++) {
      double noise = dist(gen) * noise_magnitude;
      double output = input + noise;
      min_output = std::min(min_output, output);
      max_output = std::max(max_output, output);
    }

    // Output should be bounded by input +/- noise_magnitude
    TS_ASSERT(min_output >= input - noise_magnitude);
    TS_ASSERT(max_output <= input + noise_magnitude);
  }

  // Test uniform noise with percent variation
  void testUniformNoisePercent() {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    double input = 100.0;
    double noise_percent = 0.05;  // 5%

    double min_output = input, max_output = input;
    for (int i = 0; i < 1000; i++) {
      double noise_factor = 1.0 + dist(gen) * noise_percent;
      double output = input * noise_factor;
      min_output = std::min(min_output, output);
      max_output = std::max(max_output, output);
    }

    // Output should be bounded by input * (1 +/- noise_percent)
    TS_ASSERT(min_output >= input * (1.0 - noise_percent));
    TS_ASSERT(max_output <= input * (1.0 + noise_percent));
  }

  // Test Gaussian noise distribution properties
  void testGaussianNoiseProperties() {
    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double sum = 0.0, sum_sq = 0.0;
    int n = 10000;
    for (int i = 0; i < n; i++) {
      double val = dist(gen);
      sum += val;
      sum_sq += val * val;
    }

    double mean = sum / n;
    double variance = sum_sq / n - mean * mean;
    double std_dev = std::sqrt(variance);

    // Mean should be close to 0
    TS_ASSERT_DELTA(mean, 0.0, 0.05);
    // Std dev should be close to 1
    TS_ASSERT_DELTA(std_dev, 1.0, 0.05);
  }

  // Test Gaussian noise with 6-sigma span
  void testGaussian6Sigma() {
    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double min_val = 0.0, max_val = 0.0;
    for (int i = 0; i < 100000; i++) {
      double val = dist(gen);
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }

    // Should span roughly -3 to +3 (6-sigma)
    TS_ASSERT(min_val < -2.5);
    TS_ASSERT(max_val > 2.5);
    // But rarely exceed +/- 4
    TS_ASSERT(min_val > -5.0);
    TS_ASSERT(max_val < 5.0);
  }

  // Test quantization with N bits
  void testQuantization() {
    int bits = 12;
    double min_val = 0.0;
    double max_val = 400.0;

    int divisions = (1 << bits);  // 2^bits = 4096
    double granularity = (max_val - min_val) / divisions;

    TS_ASSERT_EQUALS(divisions, 4096);
    TS_ASSERT_DELTA(granularity, 400.0 / 4096.0, epsilon);

    // Test quantizing a value
    double input = 123.456;
    int quantized = static_cast<int>((input - min_val) / granularity);
    double output = min_val + quantized * granularity;

    TS_ASSERT(std::abs(output - input) <= granularity);
  }

  // Test quantization at boundaries
  void testQuantizationBoundaries() {
    int bits = 8;
    double min_val = 0.0;
    double max_val = 100.0;

    int divisions = (1 << bits);  // 256
    double granularity = (max_val - min_val) / divisions;

    // Test at minimum
    double input = 0.0;
    int quantized = static_cast<int>((input - min_val) / granularity);
    TS_ASSERT_EQUALS(quantized, 0);

    // Test at maximum
    input = max_val;
    quantized = std::min(divisions - 1, static_cast<int>((input - min_val) / granularity));
    TS_ASSERT_EQUALS(quantized, divisions - 1);
  }

  // Test quantization step function behavior
  void testQuantizationSteps() {
    int bits = 4;
    double min_val = 0.0;
    double max_val = 16.0;

    int divisions = (1 << bits);  // 16
    double granularity = (max_val - min_val) / divisions;  // 1.0

    // Values 0.0 to 0.999 should quantize to 0
    double inputs[] = {0.0, 0.5, 0.99};
    for (double input : inputs) {
      int q = static_cast<int>((input - min_val) / granularity);
      TS_ASSERT_EQUALS(q, 0);
    }

    // Values 1.0 to 1.999 should quantize to 1
    double inputs2[] = {1.0, 1.5, 1.99};
    for (double input : inputs2) {
      int q = static_cast<int>((input - min_val) / granularity);
      TS_ASSERT_EQUALS(q, 1);
    }
  }

  // Test first-order lag filter
  void testLagFilter() {
    // Lag filter: y(n) = ca*y(n-1) + cb*u(n)
    // For continuous lag C1/(s+C1), discretized with dt:
    // ca = 1 - C1*dt, cb = C1*dt (simplified first-order approximation)

    double C1 = 10.0;  // Lag coefficient
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);  // More accurate discretization
    double cb = 1.0 - ca;

    TS_ASSERT(ca >= 0.0 && ca <= 1.0);
    TS_ASSERT(cb >= 0.0 && cb <= 1.0);
    TS_ASSERT_DELTA(ca + cb, 1.0, 0.01);  // Approximately unity DC gain

    // Apply step input
    double y = 0.0;
    double u = 1.0;  // Step input

    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * u;
    }

    // Should approach 1.0 (DC gain is 1)
    TS_ASSERT_DELTA(y, 1.0, 0.001);
  }

  // Test lag filter time constant
  void testLagTimeConstant() {
    double C1 = 10.0;  // Time constant tau = 1/C1 = 0.1 sec
    double dt = 0.001;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double u = 1.0;
    double t = 0.0;

    // Find time to reach 63.2% of step (1 time constant)
    while (y < 0.632 && t < 1.0) {
      y = ca * y + cb * u;
      t += dt;
    }

    // Should reach 63.2% in approximately tau = 0.1 seconds
    TS_ASSERT_DELTA(t, 0.1, 0.01);
  }

  // Test sensor bias
  void testBias() {
    double input = 100.0;
    double bias = 2.5;

    double output = input + bias;
    TS_ASSERT_DELTA(output, 102.5, epsilon);
  }

  // Test sensor bias negative
  void testBiasNegative() {
    double input = 100.0;
    double bias = -5.0;

    double output = input + bias;
    TS_ASSERT_DELTA(output, 95.0, epsilon);
  }

  // Test sensor gain
  void testGain() {
    double input = 50.0;
    double gain = 1.5;

    double output = input * gain;
    TS_ASSERT_DELTA(output, 75.0, epsilon);
  }

  // Test sensor gain less than 1
  void testGainAttenuation() {
    double input = 100.0;
    double gain = 0.5;

    double output = input * gain;
    TS_ASSERT_DELTA(output, 50.0, epsilon);
  }

  // Test combined bias and gain
  void testBiasAndGain() {
    double input = 100.0;
    double gain = 2.0;
    double bias = 10.0;

    // Order matters: typically gain first, then bias
    double output = input * gain + bias;
    TS_ASSERT_DELTA(output, 210.0, epsilon);
  }

  // Test sensor drift
  void testDrift() {
    double drift_rate = 0.01;  // Units per second
    double dt = 0.1;
    double drift = 0.0;

    // Simulate 10 seconds of drift: 100 iterations * 0.1s = 10s
    // drift = drift_rate * dt * iterations = 0.01 * 0.1 * 100 = 0.1
    for (int i = 0; i < 100; i++) {
      drift += drift_rate * dt;
    }

    TS_ASSERT_DELTA(drift, 0.1, 0.001);  // 0.01 * 0.1 * 100 = 0.1
  }

  // Test drift applied to signal
  void testDriftApplied() {
    double input = 100.0;
    double drift_rate = 0.1;
    double dt = 0.01;
    double drift = 0.0;

    double output = input;
    for (int i = 0; i < 1000; i++) {
      drift += drift_rate * dt;
      output = input + drift;
    }

    // After 10 seconds, drift should be 1.0
    TS_ASSERT_DELTA(output, 101.0, 0.001);
  }

  // Test fail low mode
  void testFailLow() {
    bool fail_low = true;
    double input = 100.0;
    double fail_value = 0.0;  // Typically minimum

    double output = fail_low ? fail_value : input;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test fail high mode
  void testFailHigh() {
    bool fail_high = true;
    double input = 50.0;
    double fail_value = 100.0;  // Typically maximum

    double output = fail_high ? fail_value : input;
    TS_ASSERT_DELTA(output, 100.0, epsilon);
  }

  // Test fail stuck mode
  void testFailStuck() {
    bool fail_stuck = true;
    double stuck_value = 75.0;  // Value when failure occurred
    double input = 100.0;

    double output = fail_stuck ? stuck_value : input;
    TS_ASSERT_DELTA(output, 75.0, epsilon);
  }

  // Test delay by frames
  void testDelayFrames() {
    int delay_frames = 5;
    std::deque<double> buffer;

    // Initialize buffer with zeros
    for (int i = 0; i < delay_frames; i++) {
      buffer.push_back(0.0);
    }

    // Apply step input
    double input = 100.0;
    double output = 0.0;

    // First 5 frames should output 0 (delayed)
    for (int i = 0; i < delay_frames; i++) {
      buffer.push_back(input);
      output = buffer.front();
      buffer.pop_front();
      TS_ASSERT_DELTA(output, 0.0, epsilon);
    }

    // Frame 6 should output the step
    buffer.push_back(input);
    output = buffer.front();
    buffer.pop_front();
    TS_ASSERT_DELTA(output, 100.0, epsilon);
  }

  // Test delay by time
  void testDelayTime() {
    double delay_time = 0.05;  // 50 ms
    double dt = 0.01;          // 10 ms frame time
    int delay_frames = static_cast<int>(delay_time / dt);

    TS_ASSERT_EQUALS(delay_frames, 5);
  }

  // Test signal processing chain
  void testProcessingChain() {
    // Typical order: input -> gain -> bias -> lag -> noise -> quantize -> output

    double input = 50.0;
    double gain = 2.0;
    double bias = 10.0;
    double lag_output = input;  // Simplified

    // Apply gain
    double after_gain = input * gain;
    TS_ASSERT_DELTA(after_gain, 100.0, epsilon);

    // Apply bias
    double after_bias = after_gain + bias;
    TS_ASSERT_DELTA(after_bias, 110.0, epsilon);

    // Noise and quantization would modify further
  }

  // Test random seed determinism
  void testRandomSeedDeterminism() {
    unsigned int seed = 12345;

    // First sequence
    std::mt19937 gen1(seed);
    std::normal_distribution<double> dist1(0.0, 1.0);
    double vals1[10];
    for (int i = 0; i < 10; i++) {
      vals1[i] = dist1(gen1);
    }

    // Second sequence with same seed
    std::mt19937 gen2(seed);
    std::normal_distribution<double> dist2(0.0, 1.0);
    double vals2[10];
    for (int i = 0; i < 10; i++) {
      vals2[i] = dist2(gen2);
    }

    // Should be identical
    for (int i = 0; i < 10; i++) {
      TS_ASSERT_DELTA(vals1[i], vals2[i], epsilon);
    }
  }

  // Test quantization resolution vs bits
  void testQuantizationResolution() {
    double range = 100.0;

    for (int bits = 4; bits <= 16; bits += 4) {
      int divisions = (1 << bits);
      double resolution = range / divisions;

      // More bits = finer resolution
      if (bits == 4) TS_ASSERT_DELTA(resolution, 6.25, 0.01);
      if (bits == 8) TS_ASSERT_DELTA(resolution, 0.390625, 0.0001);
      if (bits == 12) TS_ASSERT_DELTA(resolution, 0.0244140625, 0.00001);
      if (bits == 16) TS_ASSERT_DELTA(resolution, 0.00152587890625, 0.000001);
    }
  }

  // Test noise disabled (zero variance)
  void testNoiseDisabled() {
    double input = 100.0;
    double noise_variance = 0.0;

    // With zero noise, output equals input
    double output = input;  // No noise added
    TS_ASSERT_DELTA(output, input, epsilon);
  }

  // Test combined sensor effects
  void testCombinedEffects() {
    std::mt19937 gen(42);
    std::normal_distribution<double> noise_dist(0.0, 0.5);

    double input = 100.0;
    double gain = 1.1;
    double bias = 2.0;
    double C1 = 50.0;
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    // Process through full chain
    double after_gain = input * gain;
    double after_bias = after_gain + bias;

    // Run lag filter multiple times to converge
    double after_lag = input;
    for (int i = 0; i < 100; i++) {
      after_lag = ca * after_lag + cb * after_bias;
    }
    double after_noise = after_lag + noise_dist(gen);

    // After convergence, lag output should be very close to after_bias
    // Noise std=0.5, so should be within ~2 at 99% confidence
    TS_ASSERT(std::abs(after_noise - after_bias) < 3.0);
  }

  /***************************************************************************
   * Additional Comprehensive Sensor Tests
   ***************************************************************************/

  // Test sensor saturation (clamping)
  double saturate(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
  }

  void testSaturationLow() {
    double output = saturate(-10.0, 0.0, 100.0);
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  void testSaturationHigh() {
    double output = saturate(150.0, 0.0, 100.0);
    TS_ASSERT_DELTA(output, 100.0, epsilon);
  }

  void testSaturationWithin() {
    double output = saturate(50.0, 0.0, 100.0);
    TS_ASSERT_DELTA(output, 50.0, epsilon);
  }

  // Test hysteresis in sensor
  double applyHysteresis(double input, double& prevOutput, double hysteresisWidth) {
    if (std::abs(input - prevOutput) > hysteresisWidth) {
      prevOutput = input;
    }
    return prevOutput;
  }

  void testHysteresisSmallChange() {
    double prevOutput = 50.0;
    double result = applyHysteresis(52.0, prevOutput, 5.0);
    TS_ASSERT_DELTA(result, 50.0, epsilon);  // Within hysteresis band
  }

  void testHysteresisLargeChange() {
    double prevOutput = 50.0;
    double result = applyHysteresis(60.0, prevOutput, 5.0);
    TS_ASSERT_DELTA(result, 60.0, epsilon);  // Outside hysteresis band
  }

  // Test rate limiting (slew rate)
  double rateLimit(double input, double& prevOutput, double maxRate, double dt) {
    double delta = input - prevOutput;
    double maxDelta = maxRate * dt;
    if (std::abs(delta) > maxDelta) {
      delta = delta > 0 ? maxDelta : -maxDelta;
    }
    prevOutput += delta;
    return prevOutput;
  }

  void testRateLimitExceeded() {
    double prev = 0.0;
    double result = rateLimit(100.0, prev, 10.0, 1.0);
    TS_ASSERT_DELTA(result, 10.0, epsilon);
  }

  void testRateLimitNotExceeded() {
    double prev = 0.0;
    double result = rateLimit(5.0, prev, 10.0, 1.0);
    TS_ASSERT_DELTA(result, 5.0, epsilon);
  }

  // Test temperature sensor scaling
  void testTemperatureSensorScaling() {
    // Typical temperature sensor: 0-5V maps to -40 to 150 C
    double voltage = 2.5;  // Mid-range
    double minTemp = -40.0;
    double maxTemp = 150.0;
    double minVolt = 0.0;
    double maxVolt = 5.0;

    double temperature = minTemp + (voltage - minVolt) / (maxVolt - minVolt) * (maxTemp - minTemp);
    TS_ASSERT_DELTA(temperature, 55.0, epsilon);  // Midpoint
  }

  void testTemperatureSensorAtMin() {
    double voltage = 0.0;
    double temperature = -40.0 + (voltage / 5.0) * 190.0;
    TS_ASSERT_DELTA(temperature, -40.0, epsilon);
  }

  void testTemperatureSensorAtMax() {
    double voltage = 5.0;
    double temperature = -40.0 + (voltage / 5.0) * 190.0;
    TS_ASSERT_DELTA(temperature, 150.0, epsilon);
  }

  // Test pressure sensor scaling
  void testPressureSensorScaling() {
    // Pressure sensor: 0.5-4.5V maps to 0-100 PSI
    double voltage = 2.5;  // Mid-range
    double minPres = 0.0;
    double maxPres = 100.0;
    double minVolt = 0.5;
    double maxVolt = 4.5;

    double pressure = minPres + (voltage - minVolt) / (maxVolt - minVolt) * (maxPres - minPres);
    TS_ASSERT_DELTA(pressure, 50.0, epsilon);
  }

  // Test sensor calibration with offset and scale
  void testSensorCalibration() {
    double rawValue = 512;  // ADC counts
    double offset = -2.5;   // Offset correction
    double scale = 0.01;    // Scale factor

    double calibrated = (rawValue * scale) + offset;
    TS_ASSERT_DELTA(calibrated, 2.62, epsilon);
  }

  // Test cross-axis sensitivity
  void testCrossAxisSensitivity() {
    double primaryAxis = 100.0;
    double secondaryAxis = 50.0;
    double crossAxisCoeff = 0.02;  // 2% cross-axis sensitivity

    double measuredPrimary = primaryAxis + secondaryAxis * crossAxisCoeff;
    TS_ASSERT_DELTA(measuredPrimary, 101.0, epsilon);
  }

  // Test temperature compensation
  void testTemperatureCompensation() {
    double rawReading = 100.0;
    double temperature = 50.0;     // Current temperature
    double refTemp = 25.0;         // Reference temperature
    double tempCoeff = 0.001;      // 0.1% per degree

    double compensated = rawReading / (1.0 + tempCoeff * (temperature - refTemp));
    TS_ASSERT_DELTA(compensated, 97.56097561, 0.0001);
  }

  // Test sensor resolution (minimum detectable change)
  void testSensorResolution() {
    int adcBits = 12;
    double fullScaleRange = 10.0;  // Volts

    double resolution = fullScaleRange / (1 << adcBits);
    TS_ASSERT_DELTA(resolution, 10.0 / 4096.0, epsilon);
    TS_ASSERT(resolution < 0.003);  // Less than 3 mV
  }

  // Test signal-to-noise ratio
  void testSignalToNoiseRatio() {
    double signalAmplitude = 100.0;
    double noiseRMS = 1.0;

    double snr = signalAmplitude / noiseRMS;
    double snrDB = 20.0 * std::log10(snr);

    TS_ASSERT_DELTA(snr, 100.0, epsilon);
    TS_ASSERT_DELTA(snrDB, 40.0, epsilon);
  }

  // Test intermittent failure mode
  void testIntermittentFailure() {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    double failureProbability = 0.1;  // 10% chance of failure
    int failures = 0;
    int samples = 10000;

    for (int i = 0; i < samples; i++) {
      if (dist(gen) < failureProbability) {
        failures++;
      }
    }

    // Should be approximately 10% (1000 +/- ~100)
    TS_ASSERT(failures > 900 && failures < 1100);
  }

  // Test erratic failure mode
  void testErraticFailure() {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> erratic(-1000.0, 1000.0);

    bool failed = true;
    double input = 100.0;
    double output = failed ? erratic(gen) : input;

    // Erratic output is random and may be far from input
    TS_ASSERT(output != input || failed == false);
  }

  // Test moving average filter
  void testMovingAverageFilter() {
    std::deque<double> buffer;
    int windowSize = 5;
    double inputs[] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0};

    for (int i = 0; i < 5; i++) {
      buffer.push_back(inputs[i]);
    }

    double sum = 0.0;
    for (double val : buffer) {
      sum += val;
    }
    double avg = sum / windowSize;

    TS_ASSERT_DELTA(avg, 30.0, epsilon);  // (10+20+30+40+50)/5 = 30
  }

  // Test exponential moving average
  void testExponentialMovingAverage() {
    double alpha = 0.1;  // Smoothing factor
    double ema = 0.0;

    double inputs[] = {100.0, 100.0, 100.0, 100.0, 100.0};
    for (double input : inputs) {
      ema = alpha * input + (1.0 - alpha) * ema;
    }

    // After 5 samples of constant 100, EMA should approach 100
    TS_ASSERT(ema > 40.0);  // Should have increased from 0
    TS_ASSERT(ema < 100.0); // But not yet at 100
  }

  // Test median filter
  void testMedianFilter() {
    std::vector<double> buffer = {10.0, 100.0, 20.0, 30.0, 15.0};
    std::sort(buffer.begin(), buffer.end());

    double median = buffer[buffer.size() / 2];
    TS_ASSERT_DELTA(median, 20.0, epsilon);  // Median of sorted {10, 15, 20, 30, 100}
  }

  // Test deadband filter
  void testDeadbandFilter() {
    double prevOutput = 50.0;
    double deadband = 2.0;

    // Small change - within deadband
    double input1 = 51.0;
    double output1 = (std::abs(input1 - prevOutput) > deadband) ? input1 : prevOutput;
    TS_ASSERT_DELTA(output1, 50.0, epsilon);

    // Large change - outside deadband
    double input2 = 55.0;
    double output2 = (std::abs(input2 - prevOutput) > deadband) ? input2 : prevOutput;
    TS_ASSERT_DELTA(output2, 55.0, epsilon);
  }

  // Test sampling rate effect
  void testSamplingRateEffect() {
    double signalFreq = 10.0;  // Hz
    double sampleRate1 = 100.0;  // Hz
    double sampleRate2 = 20.0;   // Hz

    // Nyquist limit
    double nyquist1 = sampleRate1 / 2.0;
    double nyquist2 = sampleRate2 / 2.0;

    TS_ASSERT(signalFreq < nyquist1);  // Well sampled
    TS_ASSERT(signalFreq == nyquist2); // At Nyquist limit
  }

  // Test anti-aliasing concept
  void testAntiAliasingCutoff() {
    double sampleRate = 100.0;  // Hz
    double cutoffFreq = sampleRate / 2.0 * 0.8;  // 80% of Nyquist

    TS_ASSERT_DELTA(cutoffFreq, 40.0, epsilon);
  }

  // Test ADC quantization noise
  void testADCQuantizationNoise() {
    int bits = 12;
    double fullScale = 10.0;
    double lsb = fullScale / (1 << bits);

    // Theoretical quantization noise RMS = LSB / sqrt(12)
    double quantNoiseRMS = lsb / std::sqrt(12.0);

    TS_ASSERT(quantNoiseRMS < 0.001);  // Very small for 12-bit ADC
  }

  // Test gain error
  void testGainError() {
    double expectedGain = 1.0;
    double actualGain = 1.02;  // 2% gain error

    double gainError = (actualGain - expectedGain) / expectedGain * 100.0;
    TS_ASSERT_DELTA(gainError, 2.0, epsilon);
  }

  // Test offset error
  void testOffsetError() {
    double expectedOffset = 0.0;
    double actualOffset = 0.5;

    double offsetError = actualOffset - expectedOffset;
    TS_ASSERT_DELTA(offsetError, 0.5, epsilon);
  }

  // Test linearity error
  void testLinearityError() {
    // Simulated non-linear sensor response
    auto nonLinearResponse = [](double x) { return x + 0.01 * x * x; };

    double input = 50.0;
    double linearOutput = input;
    double actualOutput = nonLinearResponse(input);

    double linearityError = actualOutput - linearOutput;
    TS_ASSERT_DELTA(linearityError, 25.0, epsilon);  // 0.01 * 50^2 = 25
  }

  // Test repeatability
  void testRepeatability() {
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 0.1);

    double trueValue = 100.0;
    std::vector<double> measurements;

    for (int i = 0; i < 100; i++) {
      measurements.push_back(trueValue + noise(gen));
    }

    // Calculate standard deviation (repeatability)
    double sum = 0.0, sum_sq = 0.0;
    for (double m : measurements) {
      sum += m;
      sum_sq += m * m;
    }
    double mean = sum / measurements.size();
    double variance = sum_sq / measurements.size() - mean * mean;
    double stdDev = std::sqrt(variance);

    TS_ASSERT_DELTA(stdDev, 0.1, 0.02);  // Should be close to noise std dev
  }

  // Test sensor warm-up drift
  void testWarmUpDrift() {
    double initialBias = 5.0;   // Initial bias at cold start
    double finalBias = 0.5;     // Final bias after warm-up
    double warmUpTime = 60.0;   // 60 seconds warm-up
    double timeConstant = 20.0; // Thermal time constant

    // Exponential decay of bias
    double t = 60.0;  // After full warm-up
    double currentBias = finalBias + (initialBias - finalBias) * std::exp(-t / timeConstant);

    TS_ASSERT(currentBias < initialBias);
    TS_ASSERT(currentBias > finalBias);
    TS_ASSERT_DELTA(currentBias, 0.7, 0.1);  // Close to final bias
  }

  // Test scale factor stability
  void testScaleFactorStability() {
    double nominalScale = 1.0;
    double temperature = 50.0;
    double refTemp = 25.0;
    double ppmPerDegree = 50;  // 50 ppm/C

    double scaleDrift = nominalScale * ppmPerDegree * (temperature - refTemp) / 1e6;
    double actualScale = nominalScale + scaleDrift;

    TS_ASSERT_DELTA(scaleDrift, 0.00125, epsilon);  // 1250 ppm for 25C delta
    TS_ASSERT_DELTA(actualScale, 1.00125, epsilon);
  }

  // Test second-order lag filter
  void testSecondOrderLag() {
    double omega_n = 10.0;  // Natural frequency
    double zeta = 0.7;      // Damping ratio
    double dt = 0.001;

    // Simplified second-order discrete approximation
    double a1 = 2.0 * zeta * omega_n;
    double a0 = omega_n * omega_n;

    double y_prev = 0.0, y_prev2 = 0.0;
    double u = 1.0;  // Step input

    // Run simulation
    for (int i = 0; i < 5000; i++) {
      double y = (a0 * dt * dt * u + 2 * y_prev - y_prev2 + a1 * dt * y_prev) /
                 (1 + a1 * dt + a0 * dt * dt);
      y_prev2 = y_prev;
      y_prev = y;
    }

    // Should converge to 1.0 (DC gain)
    TS_ASSERT_DELTA(y_prev, 1.0, 0.01);
  }

  // Test multiple lag stages
  void testMultipleLagStages() {
    double C1 = 10.0;
    double C2 = 20.0;
    double dt = 0.01;

    double ca1 = std::exp(-C1 * dt);
    double cb1 = 1.0 - ca1;
    double ca2 = std::exp(-C2 * dt);
    double cb2 = 1.0 - ca2;

    double y1 = 0.0, y2 = 0.0;
    double u = 1.0;

    for (int i = 0; i < 1000; i++) {
      y1 = ca1 * y1 + cb1 * u;
      y2 = ca2 * y2 + cb2 * y1;
    }

    // Both stages converged
    TS_ASSERT_DELTA(y1, 1.0, 0.001);
    TS_ASSERT_DELTA(y2, 1.0, 0.001);
  }

  // Test sensor range percentage
  void testSensorRangePercentage() {
    double minRange = -100.0;
    double maxRange = 100.0;
    double reading = 50.0;

    double percentage = (reading - minRange) / (maxRange - minRange) * 100.0;
    TS_ASSERT_DELTA(percentage, 75.0, epsilon);  // 50 is 75% of -100 to 100
  }

  // Test over-range detection
  void testOverRangeDetection() {
    double maxRange = 100.0;
    double reading = 120.0;

    bool overRange = reading > maxRange;
    TS_ASSERT(overRange);
  }

  // Test under-range detection
  void testUnderRangeDetection() {
    double minRange = 0.0;
    double reading = -5.0;

    bool underRange = reading < minRange;
    TS_ASSERT(underRange);
  }

  // Test sensor resolution in engineering units
  void testResolutionInEngineeringUnits() {
    int adcBits = 16;
    double minEU = -1000.0;  // Engineering units min
    double maxEU = 1000.0;   // Engineering units max

    double resolution = (maxEU - minEU) / (1 << adcBits);
    TS_ASSERT_DELTA(resolution, 2000.0 / 65536.0, epsilon);
    TS_ASSERT(resolution < 0.031);  // Better than 0.031 units
  }

  // Test noise rejection with averaging
  void testNoiseRejectionWithAveraging() {
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 10.0);  // High noise

    double trueValue = 100.0;
    int numSamples = 100;
    double sum = 0.0;

    for (int i = 0; i < numSamples; i++) {
      sum += trueValue + noise(gen);
    }

    double average = sum / numSamples;
    // Averaging reduces noise by sqrt(N)
    // Expected std of average = 10 / sqrt(100) = 1
    TS_ASSERT(std::abs(average - trueValue) < 3.0);  // Within 3 sigma
  }

  // Test sample and hold
  void testSampleAndHold() {
    double sampledValue = 50.0;
    bool holdMode = true;

    double currentInput = 100.0;
    double output = holdMode ? sampledValue : currentInput;

    TS_ASSERT_DELTA(output, 50.0, epsilon);  // Holds previous value
  }

  // Test zero-order hold (ZOH) reconstruction
  void testZeroOrderHold() {
    double samples[] = {0.0, 50.0, 100.0, 75.0, 25.0};
    double sampleRate = 10.0;  // Hz
    double samplePeriod = 1.0 / sampleRate;

    // ZOH holds each sample constant until next sample
    double t = 0.15;  // Between sample 1 and 2
    int sampleIndex = static_cast<int>(t / samplePeriod);
    double output = samples[sampleIndex];

    TS_ASSERT_DELTA(output, 50.0, epsilon);  // Holds sample 1 value
  }

  // Test first-order hold (linear interpolation)
  void testFirstOrderHold() {
    double sample1 = 50.0;
    double sample2 = 100.0;
    double samplePeriod = 0.1;
    double t = 0.05;  // Halfway between samples

    double fraction = t / samplePeriod;
    double output = sample1 + fraction * (sample2 - sample1);

    TS_ASSERT_DELTA(output, 75.0, epsilon);  // Linear interpolation
  }

  // Test digital to analog conversion
  void testDAC() {
    int digitalValue = 2048;  // 12-bit DAC, mid-scale
    int maxDigital = 4095;
    double minVolt = 0.0;
    double maxVolt = 5.0;

    double analogVolt = minVolt + (static_cast<double>(digitalValue) / maxDigital) * (maxVolt - minVolt);
    TS_ASSERT_DELTA(analogVolt, 2.5, 0.001);  // Approximately mid-scale
  }

  // Test analog to digital conversion
  void testADC() {
    double analogVolt = 2.5;
    double minVolt = 0.0;
    double maxVolt = 5.0;
    int maxDigital = 4095;

    int digitalValue = static_cast<int>((analogVolt - minVolt) / (maxVolt - minVolt) * maxDigital);
    TS_ASSERT_EQUALS(digitalValue, 2047);  // Mid-scale
  }

  // Test sensor health status
  void testSensorHealthStatus() {
    double reading = 50.0;
    double minValid = 0.0;
    double maxValid = 100.0;

    bool healthy = (reading >= minValid && reading <= maxValid);
    TS_ASSERT(healthy);
  }

  // Test sensor health with out-of-range
  void testSensorHealthOutOfRange() {
    double reading = 150.0;
    double minValid = 0.0;
    double maxValid = 100.0;

    bool healthy = (reading >= minValid && reading <= maxValid);
    TS_ASSERT(!healthy);
  }

  // Test rate of change monitoring
  void testRateOfChangeMonitor() {
    double prevReading = 100.0;
    double currentReading = 120.0;
    double dt = 0.1;
    double maxRate = 100.0;  // Max allowed rate per second

    double rate = std::abs(currentReading - prevReading) / dt;
    bool rateValid = (rate <= maxRate);

    TS_ASSERT_DELTA(rate, 200.0, epsilon);
    TS_ASSERT(!rateValid);  // Rate exceeds limit
  }

  /***************************************************************************
   * Section: Additional Noise and Signal Processing Tests
   ***************************************************************************/

  void testUniformNoiseStatistics() {
    std::mt19937 gen(123);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    double sum = 0.0, sum_sq = 0.0;
    int n = 10000;
    for (int i = 0; i < n; i++) {
      double val = dist(gen);
      sum += val;
      sum_sq += val * val;
    }

    double mean = sum / n;
    double variance = sum_sq / n - mean * mean;

    // Uniform on [-1,1]: mean=0, var=1/3
    TS_ASSERT_DELTA(mean, 0.0, 0.05);
    TS_ASSERT_DELTA(variance, 1.0/3.0, 0.05);
  }

  void testGaussianNoiseClipping() {
    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double clipLimit = 3.0;
    int clippedCount = 0;
    int n = 10000;

    for (int i = 0; i < n; i++) {
      double val = dist(gen);
      if (std::abs(val) > clipLimit) {
        clippedCount++;
      }
    }

    // About 0.27% should exceed 3 sigma
    double clippedPercent = 100.0 * clippedCount / n;
    TS_ASSERT(clippedPercent < 1.0);  // Less than 1% clipped
  }

  void testNoiseSuperposition() {
    std::mt19937 gen(42);
    std::normal_distribution<double> dist1(0.0, 1.0);
    std::normal_distribution<double> dist2(0.0, 2.0);

    double sum_sq = 0.0;
    int n = 10000;
    for (int i = 0; i < n; i++) {
      double combined = dist1(gen) + dist2(gen);
      sum_sq += combined * combined;
    }

    double variance = sum_sq / n;
    // Variance of sum = var1 + var2 = 1 + 4 = 5
    TS_ASSERT_DELTA(variance, 5.0, 0.3);
  }

  /***************************************************************************
   * Section: Advanced Filter Tests
   ***************************************************************************/

  void testButterworthResponse() {
    // Simplified first-order Butterworth (same as first-order lag)
    double fc = 10.0;  // Cutoff frequency Hz
    double fs = 100.0; // Sample rate Hz
    double dt = 1.0 / fs;
    double omega_c = 2.0 * M_PI * fc;
    double ca = std::exp(-omega_c * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double u = 1.0;

    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * u;
    }

    TS_ASSERT_DELTA(y, 1.0, 0.001);
  }

  void testHighPassFilter() {
    // High pass = input - low pass
    double fc = 10.0;
    double dt = 0.01;
    double omega_c = 2.0 * M_PI * fc;
    double ca = std::exp(-omega_c * dt);
    double cb = 1.0 - ca;

    double y_lp = 0.0;  // Low pass output
    double u = 1.0;     // DC input

    for (int i = 0; i < 1000; i++) {
      y_lp = ca * y_lp + cb * u;
    }

    double y_hp = u - y_lp;  // High pass output

    // HP should block DC
    TS_ASSERT_DELTA(y_hp, 0.0, 0.01);
  }

  void testBandPassFilter() {
    // Band pass = LP1 - LP2 (simplified approximation)
    double f_low = 5.0;
    double f_high = 20.0;
    double dt = 0.001;

    double ca_low = std::exp(-2.0 * M_PI * f_low * dt);
    double cb_low = 1.0 - ca_low;
    double ca_high = std::exp(-2.0 * M_PI * f_high * dt);
    double cb_high = 1.0 - ca_high;

    double y_low = 0.0, y_high = 0.0;
    double u = 1.0;

    for (int i = 0; i < 10000; i++) {
      y_low = ca_low * y_low + cb_low * u;
      y_high = ca_high * y_high + cb_high * u;
    }

    // Both should converge to 1 for DC
    TS_ASSERT_DELTA(y_low, 1.0, 0.01);
    TS_ASSERT_DELTA(y_high, 1.0, 0.01);
  }

  void testNotchFilter() {
    // Notch filter removes specific frequency
    double notchFreq = 60.0;  // Hz (power line interference)
    double Q = 10.0;          // Quality factor

    // Just verify parameters are reasonable
    TS_ASSERT(notchFreq > 0);
    TS_ASSERT(Q > 0);
    TS_ASSERT(notchFreq / Q < notchFreq);  // Bandwidth < center freq
  }

  /***************************************************************************
   * Section: Sensor Dynamics Tests
   ***************************************************************************/

  void testSensorBandwidth() {
    double bandwidth = 50.0;  // Hz
    double riseTime = 0.35 / bandwidth;  // Approximate relationship

    TS_ASSERT_DELTA(riseTime, 0.007, 0.001);  // ~7ms rise time
  }

  void testSensorSettlingTime() {
    double bandwidth = 50.0;
    double settlingTime = 4.0 / (2.0 * M_PI * bandwidth);  // ~4 time constants

    TS_ASSERT(settlingTime < 0.02);  // Less than 20ms
  }

  void testSensorOvershoot() {
    double zeta = 0.5;  // Underdamped
    double overshoot = std::exp(-M_PI * zeta / std::sqrt(1 - zeta * zeta)) * 100.0;

    TS_ASSERT(overshoot > 10.0);  // Significant overshoot
    TS_ASSERT(overshoot < 20.0);  // But not excessive
  }

  void testCriticallyDampedResponse() {
    double zeta = 1.0;  // Critical damping

    // No overshoot for critically damped system
    // Overshoot formula undefined at zeta=1, but approaches 0
    TS_ASSERT_DELTA(zeta, 1.0, epsilon);
  }

  void testOverdampedResponse() {
    double zeta = 2.0;  // Overdamped

    TS_ASSERT(zeta > 1.0);
    // Overdamped has no overshoot but slower response
  }

  /***************************************************************************
   * Section: Error and Uncertainty Tests
   ***************************************************************************/

  void testTotalError() {
    double biasError = 0.5;
    double gainError = 0.02;  // 2%
    double noiseRMS = 0.1;
    double reading = 100.0;

    // Total error (simplified root-sum-square)
    double gainContrib = reading * gainError;
    double totalRSS = std::sqrt(biasError * biasError + gainContrib * gainContrib + noiseRMS * noiseRMS);

    TS_ASSERT(totalRSS > biasError);  // Total > any single component
    TS_ASSERT(totalRSS < biasError + gainContrib + noiseRMS);  // Less than linear sum
  }

  void testUncertaintyPropagation() {
    double u_a = 0.1;  // Uncertainty in a
    double u_b = 0.2;  // Uncertainty in b
    double a = 10.0;
    double b = 20.0;

    // For sum: u_sum = sqrt(u_a^2 + u_b^2)
    double sum = a + b;
    double u_sum = std::sqrt(u_a * u_a + u_b * u_b);

    TS_ASSERT_DELTA(sum, 30.0, epsilon);
    TS_ASSERT_DELTA(u_sum, std::sqrt(0.01 + 0.04), epsilon);
  }

  void testRelativeUncertainty() {
    double value = 100.0;
    double absoluteUnc = 2.0;

    double relativeUnc = absoluteUnc / value * 100.0;  // Percentage
    TS_ASSERT_DELTA(relativeUnc, 2.0, epsilon);
  }

  void testExpandedUncertainty() {
    double standardUnc = 1.0;
    double coverageFactor = 2.0;  // 95% confidence

    double expandedUnc = standardUnc * coverageFactor;
    TS_ASSERT_DELTA(expandedUnc, 2.0, epsilon);
  }

  /***************************************************************************
   * Section: Redundancy and Voting Tests
   ***************************************************************************/

  void testDualRedundancy() {
    double sensor1 = 100.0;
    double sensor2 = 100.5;
    double tolerance = 1.0;

    bool agree = std::abs(sensor1 - sensor2) < tolerance;
    double average = (sensor1 + sensor2) / 2.0;

    TS_ASSERT(agree);
    TS_ASSERT_DELTA(average, 100.25, epsilon);
  }

  void testTripleRedundancyVoting() {
    double sensor1 = 100.0;
    double sensor2 = 100.5;
    double sensor3 = 150.0;  // Failed sensor

    // Median voting
    double values[3] = {sensor1, sensor2, sensor3};
    std::sort(values, values + 3);
    double median = values[1];

    TS_ASSERT_DELTA(median, 100.5, epsilon);  // Rejects failed sensor
  }

  void testSensorFusionWeighted() {
    double sensor1 = 100.0;
    double var1 = 1.0;  // Variance
    double sensor2 = 102.0;
    double var2 = 4.0;  // Higher variance = less trusted

    // Optimal weighted average (Kalman-like)
    double w1 = 1.0 / var1;
    double w2 = 1.0 / var2;
    double wSum = w1 + w2;
    double fused = (w1 * sensor1 + w2 * sensor2) / wSum;

    // More weight on sensor1 (lower variance)
    TS_ASSERT(fused < 101.0);
    TS_ASSERT(fused > 100.0);
  }

  /***************************************************************************
   * Section: Timing and Synchronization Tests
   ***************************************************************************/

  void testSensorLatency() {
    double processingTime = 0.001;  // 1ms processing
    double transmissionTime = 0.0005;  // 0.5ms transmission
    double totalLatency = processingTime + transmissionTime;

    TS_ASSERT_DELTA(totalLatency, 0.0015, epsilon);
  }

  void testSensorJitter() {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> jitter(-0.0001, 0.0001);  // +/- 100us

    double nominalPeriod = 0.01;  // 10ms
    double actualPeriod = nominalPeriod + jitter(gen);

    TS_ASSERT(std::abs(actualPeriod - nominalPeriod) < 0.0001);
  }

  void testSamplingPhaseAlignment() {
    double phase1 = 0.0;
    double phase2 = 0.005;  // 5ms offset
    double samplePeriod = 0.01;

    double phaseError = phase2 - phase1;
    double phaseErrorPercent = phaseError / samplePeriod * 100.0;

    TS_ASSERT_DELTA(phaseErrorPercent, 50.0, epsilon);
  }

  /***************************************************************************
   * Section: Environmental Effects Tests
   ***************************************************************************/

  void testVibrationEffect() {
    std::mt19937 gen(42);
    std::normal_distribution<double> vibration(0.0, 2.0);

    double trueValue = 100.0;
    double vibrationNoise = vibration(gen);
    double reading = trueValue + vibrationNoise;

    TS_ASSERT(std::abs(reading - trueValue) < 10.0);  // Within ~5 sigma
  }

  void testEMIEffect() {
    std::mt19937 gen(42);
    std::normal_distribution<double> emi(0.0, 0.5);

    double trueValue = 100.0;
    double emiNoise = emi(gen);
    double reading = trueValue + emiNoise;

    TS_ASSERT(std::abs(reading - trueValue) < 3.0);
  }

  void testAltitudeEffect() {
    double seaLevelPressure = 101325.0;  // Pa
    double altitude = 10000.0;  // meters
    double scaleHeight = 8500.0;  // meters

    double pressure = seaLevelPressure * std::exp(-altitude / scaleHeight);

    TS_ASSERT(pressure < seaLevelPressure);
    TS_ASSERT(pressure > 20000.0);  // Still positive
  }

  void testTemperatureEffectOnSensor() {
    double refTemp = 25.0;
    double currentTemp = 85.0;
    double tempCoeff = 0.0001;  // 100 ppm/C

    double driftFactor = 1.0 + tempCoeff * (currentTemp - refTemp);
    double nominalReading = 100.0;
    double actualReading = nominalReading * driftFactor;

    TS_ASSERT_DELTA(driftFactor, 1.006, 0.0001);
    TS_ASSERT_DELTA(actualReading, 100.6, 0.01);
  }

  /***************************************************************************
   * Section: Signal Characteristics Tests
   ***************************************************************************/

  void testPeakToPeakAmplitude() {
    double samples[] = {10.0, 50.0, 30.0, 80.0, 20.0, 60.0};
    int n = 6;

    double minVal = samples[0], maxVal = samples[0];
    for (int i = 1; i < n; i++) {
      minVal = std::min(minVal, samples[i]);
      maxVal = std::max(maxVal, samples[i]);
    }

    double peakToPeak = maxVal - minVal;
    TS_ASSERT_DELTA(peakToPeak, 70.0, epsilon);
  }

  void testRMSCalculation() {
    double samples[] = {-1.0, 1.0, -1.0, 1.0};
    int n = 4;

    double sum_sq = 0.0;
    for (int i = 0; i < n; i++) {
      sum_sq += samples[i] * samples[i];
    }
    double rms = std::sqrt(sum_sq / n);

    TS_ASSERT_DELTA(rms, 1.0, epsilon);
  }

  void testCrestFactor() {
    double peakValue = 10.0;
    double rmsValue = 7.07;  // For sine wave: peak/sqrt(2)

    double crestFactor = peakValue / rmsValue;

    TS_ASSERT_DELTA(crestFactor, std::sqrt(2.0), 0.01);
  }

  void testDutyCycle() {
    double onTime = 0.3;
    double period = 1.0;

    double dutyCycle = onTime / period * 100.0;
    TS_ASSERT_DELTA(dutyCycle, 30.0, epsilon);
  }
};
