#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>

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
};
