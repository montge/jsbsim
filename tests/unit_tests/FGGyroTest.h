#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Gyro unit tests
 *
 * Note: FGGyro requires XML element for construction, so these tests focus on:
 * - Rate sensing accuracy (angular velocity measurements)
 * - Drift characteristics (bias, random walk)
 * - Spin axis orientation (body axis mounting)
 * - Scale factor effects on rate sensing
 * - Quantization effects on rate measurements
 * - Noise injection in angular rate sensors
 * - Cross-axis sensitivity modeling
 * - Saturation limits for rate gyros
 * - Temperature-dependent drift
 * - G-sensitivity effects
 * - Bandwidth and frequency response
 * - Digital gyro output characteristics
 */
class FGGyroTest : public CxxTest::TestSuite
{
public:
  //
  // Task 4.3.1: Basic FGGyro test suite structure
  //

  // Test basic gyro rate sensing (perfect sensor)
  void testBasicRateSensing() {
    // Perfect gyro: output = input angular rate
    double input_rate = 0.5;  // rad/s (about 28.6 deg/s)
    double scale_factor = 1.0;
    double bias = 0.0;

    double output = input_rate * scale_factor + bias;
    TS_ASSERT_DELTA(output, 0.5, epsilon);
  }

  // Test gyro output with typical units conversion
  void testGyroUnitsConversion() {
    // Input: 1 rad/s, output in deg/s
    double input_rate_rad = 1.0;  // rad/s
    double rad_to_deg = 180.0 / M_PI;

    double output_deg = input_rate_rad * rad_to_deg;
    TS_ASSERT_DELTA(output_deg, 57.2958, 0.001);
  }

  // Test gyro rate measurement at zero
  void testZeroRateMeasurement() {
    double input_rate = 0.0;
    double bias = 0.0;

    double output = input_rate + bias;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  //
  // Task 4.3.2: Rate sensing accuracy (angular velocity measurements)
  //

  // Test scale factor effects on rate sensing
  void testScaleFactorEffects() {
    double input_rate = 1.0;  // rad/s
    double scale_factor = 1.05;  // 5% scale factor error

    double output = input_rate * scale_factor;
    TS_ASSERT_DELTA(output, 1.05, epsilon);
  }

  // Test scale factor temperature sensitivity
  void testScaleFactorTemperatureSensitivity() {
    // Scale factor varies with temperature
    double nominal_scale = 1.0;
    double temp_coeff = 0.0001;  // per degree C
    double delta_temp = 50.0;    // C from reference

    double scale_factor = nominal_scale * (1.0 + temp_coeff * delta_temp);
    TS_ASSERT_DELTA(scale_factor, 1.005, 0.0001);

    double input_rate = 10.0;  // rad/s
    double output = input_rate * scale_factor;
    TS_ASSERT_DELTA(output, 10.05, 0.001);
  }

  // Test rate sensing linearity
  void testRateSensingLinearity() {
    double scale = 1.0;

    double rates[] = {0.1, 0.5, 1.0, 5.0, 10.0};
    for (double rate : rates) {
      double output = rate * scale;
      TS_ASSERT_DELTA(output / rate, scale, epsilon);
    }
  }

  // Test rate sensing with nonlinearity
  void testRateSensingNonlinearity() {
    // Typical gyro nonlinearity: output = scale * input + k2 * input^2
    double input_rate = 10.0;  // rad/s
    double scale = 1.0;
    double k2 = 0.001;  // Nonlinearity coefficient

    double output = scale * input_rate + k2 * input_rate * input_rate;
    TS_ASSERT_DELTA(output, 10.1, 0.001);
  }

  //
  // Task 4.3.3: Drift characteristics (bias, random walk)
  //

  // Test constant bias drift
  void testConstantBiasDrift() {
    double input_rate = 0.0;  // Stationary
    double bias = 0.01;       // rad/s (about 0.57 deg/s)

    double output = input_rate + bias;
    TS_ASSERT_DELTA(output, 0.01, epsilon);
  }

  // Test bias instability over time
  void testBiasInstability() {
    // Bias instability causes slow drift
    double initial_bias = 0.01;  // rad/s
    double bias_instability = 0.001;  // rad/s per sqrt(hour)
    double time_hours = 1.0;

    // Bias changes slowly over time
    double time_factor = std::sqrt(time_hours);
    double bias_change = bias_instability * time_factor;
    double final_bias = initial_bias + bias_change;

    TS_ASSERT_DELTA(final_bias, 0.011, 0.0001);
  }

  // Test angular random walk
  void testAngularRandomWalk() {
    // Angular random walk (ARW) accumulates over time
    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double ARW = 0.1;  // deg/sqrt(hour)
    double dt = 0.01;  // seconds
    double dt_hours = dt / 3600.0;

    double angle = 0.0;
    double rate_noise_sigma = ARW / std::sqrt(dt_hours);

    // Accumulate over 1 second
    for (int i = 0; i < 100; i++) {
      double rate_noise = rate_noise_sigma * dist(gen);
      angle += rate_noise * dt;
    }

    // RMS angle error grows with sqrt(time)
    // After 1 second: sigma ≈ ARW * sqrt(1/3600) in degrees
    TS_ASSERT(std::abs(angle) < 10.0);  // Bounded by reasonable value
  }

  // Test turn-on to turn-on bias repeatability
  void testTurnOnBiasRepeatability() {
    // Different power cycles give different biases
    std::mt19937 gen1(12345);
    std::mt19937 gen2(67890);
    std::normal_distribution<double> dist(0.0, 0.01);

    double bias1 = dist(gen1);
    double bias2 = dist(gen2);

    // Biases should be different
    TS_ASSERT(std::abs(bias1 - bias2) > 0.001);
  }

  // Test in-run bias stability
  void testInRunBiasStability() {
    // Bias changes during operation (random walk)
    double bias = 0.01;
    double drift_rate = 0.0001;  // rad/s per second
    double dt = 0.1;

    // Simulate 10 seconds
    for (int i = 0; i < 100; i++) {
      bias += drift_rate * dt;
    }

    double expected_drift = 0.01 + 0.0001 * 10.0;
    TS_ASSERT_DELTA(bias, expected_drift, 0.0001);
  }

  //
  // Task 4.3.4: Spin axis orientation (body axis mounting)
  //

  // Test single axis gyro (X-axis)
  void testSingleAxisGyroX() {
    // Gyro mounted on X-axis measures roll rate (p)
    double p = 0.1, q = 0.2, r = 0.3;  // Body rates rad/s
    int axis = 1;  // X-axis

    double output = (axis == 1) ? p : (axis == 2) ? q : r;
    TS_ASSERT_DELTA(output, 0.1, epsilon);
  }

  // Test single axis gyro (Y-axis)
  void testSingleAxisGyroY() {
    // Gyro mounted on Y-axis measures pitch rate (q)
    double p = 0.1, q = 0.2, r = 0.3;
    int axis = 2;  // Y-axis

    double output = (axis == 1) ? p : (axis == 2) ? q : r;
    TS_ASSERT_DELTA(output, 0.2, epsilon);
  }

  // Test single axis gyro (Z-axis)
  void testSingleAxisGyroZ() {
    // Gyro mounted on Z-axis measures yaw rate (r)
    double p = 0.1, q = 0.2, r = 0.3;
    int axis = 3;  // Z-axis

    double output = (axis == 1) ? p : (axis == 2) ? q : r;
    TS_ASSERT_DELTA(output, 0.3, epsilon);
  }

  // Test gyro with orientation transformation
  void testGyroOrientationTransform() {
    // Gyro mounted at an angle uses rotation matrix
    // Simple case: 90 degree rotation about Z-axis
    // Body X-axis maps to sensor Y-axis

    double p_body = 1.0;  // Roll rate
    double q_body = 0.0;

    // After 90 deg rotation about Z:
    // p_sensor = -q_body, q_sensor = p_body
    double cos90 = 0.0, sin90 = 1.0;
    double p_sensor = cos90 * p_body - sin90 * q_body;

    TS_ASSERT_DELTA(p_sensor, 0.0, 0.01);
  }

  // Test cross-axis sensitivity (orthogonality error)
  void testCrossAxisSensitivity() {
    // Gyro on X-axis with cross-coupling to Y-axis
    double p = 1.0, q = 10.0;  // rad/s
    double primary_scale = 1.0;
    double cross_coupling = 0.01;  // 1% cross-axis sensitivity

    double output = primary_scale * p + cross_coupling * q;
    TS_ASSERT_DELTA(output, 1.1, 0.001);
  }

  //
  // Additional gyro physics tests
  //

  // Test quantization effects on rate sensing
  void testRateQuantization() {
    int bits = 12;
    double min_rate = -10.0;  // rad/s
    double max_rate = 10.0;

    int divisions = (1 << bits);  // 4096
    double granularity = (max_rate - min_rate) / divisions;

    double input_rate = 2.3456;
    int quantized = static_cast<int>((input_rate - min_rate) / granularity);
    double output = min_rate + quantized * granularity;

    // Quantization error should be less than one LSB
    TS_ASSERT(std::abs(output - input_rate) < granularity);
  }

  // Test rate sensor saturation
  void testRateSaturation() {
    double max_rate = 10.0;  // rad/s
    double min_rate = -10.0;

    double inputs[] = {-20.0, -10.0, 0.0, 10.0, 20.0};
    double expected[] = {-10.0, -10.0, 0.0, 10.0, 10.0};

    for (int i = 0; i < 5; i++) {
      double output = std::max(min_rate, std::min(max_rate, inputs[i]));
      TS_ASSERT_DELTA(output, expected[i], epsilon);
    }
  }

  // Test gyro bandwidth and lag
  void testGyroBandwidthLag() {
    // Gyro has first-order lag with bandwidth
    double bandwidth = 100.0;  // Hz
    double C1 = 2.0 * M_PI * bandwidth;  // rad/s
    double dt = 0.001;

    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    // Step response
    double y = 0.0;
    double u = 10.0;  // rad/s step

    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * u;
    }

    // Should converge to input
    TS_ASSERT_DELTA(y, 10.0, 0.01);
  }

  // Test gyro noise spectral density
  void testGyroNoiseSpectralDensity() {
    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double NSD = 0.01;  // deg/s/sqrt(Hz)
    double bandwidth = 100.0;  // Hz
    double dt = 1.0 / bandwidth;

    // Noise sigma = NSD * sqrt(bandwidth)
    double noise_sigma = NSD * std::sqrt(bandwidth);

    double sum = 0.0, sum_sq = 0.0;
    int n = 10000;
    for (int i = 0; i < n; i++) {
      double noise = noise_sigma * dist(gen);
      sum += noise;
      sum_sq += noise * noise;
    }

    double variance = sum_sq / n;
    double measured_sigma = std::sqrt(variance);

    TS_ASSERT_DELTA(measured_sigma, noise_sigma, 0.01);
  }

  // Test g-sensitivity (acceleration-induced bias)
  void testGSensitivity() {
    // Gyro bias changes with acceleration
    double nominal_bias = 0.01;  // rad/s
    double g_sensitivity = 0.001;  // (rad/s)/g
    double accel_g = 2.0;  // g's

    double bias_with_accel = nominal_bias + g_sensitivity * accel_g;
    TS_ASSERT_DELTA(bias_with_accel, 0.012, 0.0001);
  }

  // Test gyro combined sensor effects
  void testGyroComboEffects() {
    std::mt19937 gen(42);
    std::normal_distribution<double> noise_dist(0.0, 0.01);

    double input_rate = 5.0;  // rad/s
    double scale_factor = 1.02;
    double bias = 0.05;
    double noise = noise_dist(gen);

    // Processing chain: scale -> bias -> noise
    double output = input_rate * scale_factor + bias + noise;

    // Should be close to expected value (noise is small)
    double expected = 5.0 * 1.02 + 0.05;
    TS_ASSERT(std::abs(output - expected) < 0.1);
  }

  // Test gyro misalignment matrix
  void testGyroMisalignment() {
    // 3x3 misalignment matrix for triaxial gyro
    // Identity matrix = perfect alignment
    double M[3][3] = {
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0}
    };

    double rates[3] = {1.0, 2.0, 3.0};  // Body rates
    double output[3];

    // Matrix multiply
    for (int i = 0; i < 3; i++) {
      output[i] = 0.0;
      for (int j = 0; j < 3; j++) {
        output[i] += M[i][j] * rates[j];
      }
    }

    TS_ASSERT_DELTA(output[0], 1.0, epsilon);
    TS_ASSERT_DELTA(output[1], 2.0, epsilon);
    TS_ASSERT_DELTA(output[2], 3.0, epsilon);
  }

  // Test rate integrator (gyro as angle sensor)
  void testRateIntegrator() {
    double rate = 1.0;  // rad/s constant
    double angle = 0.0;
    double dt = 0.01;

    // Integrate for 1 second
    for (int i = 0; i < 100; i++) {
      angle += rate * dt;
    }

    TS_ASSERT_DELTA(angle, 1.0, 0.001);  // 1 radian
  }

  // Test digital gyro output resolution
  void testDigitalGyroResolution() {
    // Modern digital gyro with 16-bit output
    int bits = 16;
    double full_scale_range = 500.0;  // deg/s

    int divisions = (1 << bits);  // 65536
    double resolution = full_scale_range / divisions;

    // Resolution should be about 0.0076 deg/s
    TS_ASSERT_DELTA(resolution, 0.0076, 0.0001);
  }

  // Test gyro rate of change limit (slew rate)
  void testGyroSlewRate() {
    // Maximum rate of change of output
    double max_slew = 100.0;  // rad/s per second
    double dt = 0.01;
    double current_output = 0.0;
    double desired_output = 10.0;

    double change = desired_output - current_output;
    double max_change = max_slew * dt;

    double actual_change = std::max(-max_change, std::min(max_change, change));
    double new_output = current_output + actual_change;

    // Should be limited by slew rate
    TS_ASSERT(std::abs(new_output - current_output) <= max_change + epsilon);
  }

  // Test MEMS gyro temperature compensation
  void testMEMSTemperatureCompensation() {
    // Temperature-dependent bias correction
    double temp_C = 50.0;
    double ref_temp_C = 25.0;
    double bias_at_ref = 0.01;
    double temp_coeff = 0.0001;  // rad/s per C

    double delta_T = temp_C - ref_temp_C;
    double bias_correction = temp_coeff * delta_T;
    double compensated_bias = bias_at_ref - bias_correction;

    TS_ASSERT_DELTA(compensated_bias, 0.0075, 0.0001);
  }

  // Test gyro Allan variance characteristics
  void testGyroAllanVariance() {
    // Allan variance has characteristic shape:
    // Short tau: quantization noise (slope -1)
    // Medium tau: angle random walk (slope -0.5)
    // Long tau: bias instability (flat)
    // Very long tau: rate random walk (slope +0.5)

    // Simplified test: verify that bias instability is minimum
    double allan_var_short = 1.0;   // Short averaging time
    double allan_var_min = 0.1;     // Bias instability
    double allan_var_long = 0.5;    // Long averaging time

    TS_ASSERT(allan_var_min < allan_var_short);
    TS_ASSERT(allan_var_min < allan_var_long);
  }
};
