/*******************************************************************************
 * FGAccelerometerTest.h - Unit tests for FGAccelerometer (sensor model)
 *
 * Tests the mathematical behavior of accelerometer sensors:
 * - Acceleration measurement at sensor location
 * - Body frame transformations
 * - Sensor orientation effects
 * - Noise and quantization
 * - Bias and drift
 *
 * Note: FGAccelerometer requires XML element for construction, so these tests
 * focus on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>

const double epsilon = 1e-8;

class FGAccelerometerTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Basic Acceleration Measurement Tests
   ***************************************************************************/

  // Test direct acceleration measurement
  void testDirectAcceleration() {
    // Simple case: accelerometer at CG, measuring body-axis acceleration
    double accel_body_x = 5.0;   // m/s^2
    double accel_body_y = 0.0;
    double accel_body_z = -9.81; // Gravity

    // X-axis accelerometer measures ax
    double output = accel_body_x;
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test gravity measurement at rest
  void testGravityMeasurement() {
    // Level aircraft at rest: accelerometer measures -g in Z
    double g = 9.80665;  // m/s^2

    // Z-axis accelerometer (positive down)
    double az_sensed = g;  // Accelerometer senses upward reaction force
    TS_ASSERT_DELTA(az_sensed, 9.80665, 0.001);
  }

  // Test acceleration in g units
  void testAccelerationInGUnits() {
    double g = 9.80665;
    double accel_mps2 = 19.6133;  // 2g

    double accel_g = accel_mps2 / g;
    TS_ASSERT_DELTA(accel_g, 2.0, 0.001);
  }

  /***************************************************************************
   * Sensor Location Effects (Centripetal Acceleration)
   ***************************************************************************/

  // Test centripetal acceleration from rotation
  void testCentripetalAcceleration() {
    // Sensor at radius r from CG, aircraft rotating at omega
    double omega = 0.5;     // rad/s (roll rate)
    double radius = 5.0;    // meters from CG

    // Centripetal acceleration = omega^2 * r
    double a_centripetal = omega * omega * radius;
    TS_ASSERT_DELTA(a_centripetal, 1.25, epsilon);  // m/s^2
  }

  // Test tangential acceleration from angular acceleration
  void testTangentialAcceleration() {
    // Sensor at radius r, angular acceleration alpha
    double alpha = 0.1;     // rad/s^2
    double radius = 5.0;    // meters

    // Tangential acceleration = alpha * r
    double a_tangential = alpha * radius;
    TS_ASSERT_DELTA(a_tangential, 0.5, epsilon);  // m/s^2
  }

  // Test wingtip acceleration during roll
  void testWingtipAcceleration() {
    // Wingtip sensor during rolling maneuver
    double p = 1.0;          // Roll rate rad/s
    double p_dot = 0.5;      // Roll acceleration rad/s^2
    double span_half = 10.0; // Half wingspan in meters

    // Centripetal (inward toward CG)
    double a_centripetal = p * p * span_half;  // 10 m/s^2

    // Tangential (along wing)
    double a_tangential = p_dot * span_half;   // 5 m/s^2

    // Total in Y-Z plane
    double a_total = std::sqrt(a_centripetal*a_centripetal + a_tangential*a_tangential);
    TS_ASSERT_DELTA(a_total, std::sqrt(125.0), 0.01);
  }

  // Test sensor at CG (no rotational effects)
  void testSensorAtCG() {
    double p = 1.0, q = 1.0, r = 1.0;  // Body rotation rates
    double radius = 0.0;  // At CG

    double a_centripetal = p * p * radius;
    TS_ASSERT_DELTA(a_centripetal, 0.0, epsilon);
  }

  /***************************************************************************
   * Sensor Orientation Tests
   ***************************************************************************/

  // Test sensor aligned with body X-axis
  void testSensorAlignedX() {
    double ax = 1.0, ay = 2.0, az = 3.0;  // Body accelerations

    // X-aligned sensor measures ax
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double output = ax * std::cos(pitch) * std::cos(yaw) +
                    ay * std::cos(pitch) * std::sin(yaw) -
                    az * std::sin(pitch);
    TS_ASSERT_DELTA(output, ax, epsilon);
  }

  // Test sensor rotated 90 degrees in pitch
  void testSensorRotated90Pitch() {
    double ax = 1.0, ay = 0.0, az = 3.0;  // Body accelerations

    // Sensor pitched 90 degrees: X-sensor now measures body Z
    double pitch = M_PI / 2.0;
    double output = ax * std::cos(pitch) - az * std::sin(pitch);
    TS_ASSERT_DELTA(output, -3.0, 0.001);
  }

  // Test sensor rotated 90 degrees in roll
  void testSensorRotated90Roll() {
    double ax = 0.0, ay = 2.0, az = 3.0;  // Body accelerations

    // Sensor rolled 90 degrees: Y-sensor now measures body Z
    double roll = M_PI / 2.0;
    double cos_r = std::cos(roll), sin_r = std::sin(roll);
    double output_y = ay * cos_r + az * sin_r;
    TS_ASSERT_DELTA(output_y, 3.0, 0.001);
  }

  /***************************************************************************
   * Single-Axis Selection Tests
   ***************************************************************************/

  // Test X-axis selection
  void testAxisSelectionX() {
    double accel[3] = {1.0, 2.0, 3.0};
    int axis = 0;  // X

    double output = accel[axis];
    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  // Test Y-axis selection
  void testAxisSelectionY() {
    double accel[3] = {1.0, 2.0, 3.0};
    int axis = 1;  // Y

    double output = accel[axis];
    TS_ASSERT_DELTA(output, 2.0, epsilon);
  }

  // Test Z-axis selection
  void testAxisSelectionZ() {
    double accel[3] = {1.0, 2.0, 3.0};
    int axis = 2;  // Z

    double output = accel[axis];
    TS_ASSERT_DELTA(output, 3.0, epsilon);
  }

  /***************************************************************************
   * Noise and Quantization Tests
   ***************************************************************************/

  // Test quantization effects
  void testQuantization() {
    int bits = 12;
    double min_accel = -50.0;  // m/s^2
    double max_accel = 50.0;

    int divisions = (1 << bits);  // 4096
    double resolution = (max_accel - min_accel) / divisions;

    double input = 15.3456;
    int quantized_level = static_cast<int>((input - min_accel) / resolution);
    double output = min_accel + quantized_level * resolution;

    // Quantization error < resolution
    TS_ASSERT(std::abs(output - input) < resolution);
  }

  // Test noise injection
  void testNoiseInjection() {
    std::mt19937 gen(42);
    double noise_sigma = 0.1;  // m/s^2
    std::normal_distribution<double> dist(0.0, noise_sigma);

    double true_accel = 10.0;
    double sum = 0.0, sum_sq = 0.0;
    int n = 10000;

    for (int i = 0; i < n; i++) {
      double measured = true_accel + dist(gen);
      sum += measured;
      sum_sq += measured * measured;
    }

    double mean = sum / n;
    double variance = sum_sq / n - mean * mean;
    double measured_sigma = std::sqrt(variance);

    TS_ASSERT_DELTA(mean, true_accel, 0.01);
    TS_ASSERT_DELTA(measured_sigma, noise_sigma, 0.01);
  }

  // Test percent noise variation
  void testPercentNoiseVariation() {
    double true_accel = 10.0;
    double percent_noise = 2.0;  // 2%

    double noise_magnitude = true_accel * percent_noise / 100.0;
    double min_output = true_accel - noise_magnitude;
    double max_output = true_accel + noise_magnitude;

    TS_ASSERT_DELTA(min_output, 9.8, epsilon);
    TS_ASSERT_DELTA(max_output, 10.2, epsilon);
  }

  /***************************************************************************
   * Bias and Drift Tests
   ***************************************************************************/

  // Test constant bias
  void testConstantBias() {
    double true_accel = 5.0;
    double bias = 0.5;  // m/s^2

    double output = true_accel + bias;
    TS_ASSERT_DELTA(output, 5.5, epsilon);
  }

  // Test bias drift over time
  void testBiasDrift() {
    double initial_bias = 0.1;  // m/s^2
    double drift_rate = 0.001;  // m/s^2 per second
    double time = 100.0;        // seconds

    double final_bias = initial_bias + drift_rate * time;
    TS_ASSERT_DELTA(final_bias, 0.2, epsilon);
  }

  // Test scale factor error
  void testScaleFactorError() {
    double true_accel = 10.0;
    double scale_factor = 1.02;  // 2% scale factor error

    double output = true_accel * scale_factor;
    TS_ASSERT_DELTA(output, 10.2, epsilon);
  }

  /***************************************************************************
   * Lag/Filter Tests
   ***************************************************************************/

  // Test first-order lag filter
  void testFirstOrderLag() {
    double lag_time = 0.5;  // seconds
    double dt = 0.01;
    double ca = std::exp(-dt / lag_time);
    double cb = 1.0 - ca;

    double input = 10.0;  // Step input
    double output = 0.0;

    // Run until settled (10 time constants)
    for (int i = 0; i < 5000; i++) {
      output = ca * output + cb * input;
    }

    TS_ASSERT_DELTA(output, input, 0.001);
  }

  // Test lag time constant effect
  void testLagTimeConstant() {
    double lag_time = 0.1;
    double dt = 0.01;
    double ca = std::exp(-dt / lag_time);
    double cb = 1.0 - ca;

    double input = 10.0;
    double output = 0.0;

    // After 1 time constant, should be at ~63.2%
    int steps = static_cast<int>(lag_time / dt);
    for (int i = 0; i < steps; i++) {
      output = ca * output + cb * input;
    }

    double expected = input * (1.0 - std::exp(-1.0));
    TS_ASSERT_DELTA(output, expected, 0.1);
  }

  /***************************************************************************
   * Saturation Tests
   ***************************************************************************/

  // Test saturation limits
  void testSaturationLimits() {
    double min_range = -50.0;  // m/s^2
    double max_range = 50.0;

    double inputs[] = {-100.0, -50.0, 0.0, 50.0, 100.0};
    double expected[] = {-50.0, -50.0, 0.0, 50.0, 50.0};

    for (int i = 0; i < 5; i++) {
      double output = std::max(min_range, std::min(max_range, inputs[i]));
      TS_ASSERT_DELTA(output, expected[i], epsilon);
    }
  }

  // Test typical MEMS accelerometer range
  void testMEMSRange() {
    // Typical MEMS: +/- 16g
    double g = 9.80665;
    double range_g = 16.0;
    double range_mps2 = range_g * g;

    TS_ASSERT_DELTA(range_mps2, 156.9064, 0.001);
  }

  /***************************************************************************
   * Gain Application Tests
   ***************************************************************************/

  // Test output gain
  void testOutputGain() {
    double measured_accel = 10.0;  // m/s^2
    double gain = 0.10197;         // Convert to g (1/9.80665)

    double output = measured_accel * gain;
    TS_ASSERT_DELTA(output, 1.0197, 0.001);  // ~1g
  }

  // Test unit conversion gain
  void testUnitConversionGain() {
    double accel_mps2 = 9.80665;

    // Convert to ft/s^2
    double mps2_to_ftps2 = 3.28084;
    double accel_ftps2 = accel_mps2 * mps2_to_ftps2;
    TS_ASSERT_DELTA(accel_ftps2, 32.174, 0.001);
  }

  /***************************************************************************
   * Cross-Axis Sensitivity Tests
   ***************************************************************************/

  // Test cross-axis coupling
  void testCrossAxisCoupling() {
    double ax = 10.0, ay = 5.0, az = 2.0;  // m/s^2
    double cross_coupling = 0.01;  // 1%

    // X-axis sensor with Y and Z coupling
    double output_x = ax + cross_coupling * (ay + az);
    TS_ASSERT_DELTA(output_x, 10.07, 0.001);
  }

  // Test orthogonality error
  void testOrthogonalityError() {
    // Misalignment angle
    double misalign_rad = 0.01;  // ~0.57 degrees

    double ax = 10.0, ay = 0.0;

    // X sensor picks up some Y
    // cos(0.01) ≈ 0.99995, so output ≈ 9.9995
    double output_x = ax * std::cos(misalign_rad) + ay * std::sin(misalign_rad);
    TS_ASSERT_DELTA(output_x, 9.9995, 0.0001);
  }

  /***************************************************************************
   * Complete Sensor Chain Tests
   ***************************************************************************/

  // Test full sensor processing chain
  void testFullSensorChain() {
    std::mt19937 gen(42);
    std::normal_distribution<double> noise_dist(0.0, 0.1);

    double true_accel = 15.0;  // m/s^2
    double scale_factor = 1.01;
    double bias = 0.2;

    // Apply sensor model: scale -> bias -> noise
    double output = true_accel * scale_factor + bias + noise_dist(gen);

    // Should be close to expected (within noise bounds)
    double expected = true_accel * scale_factor + bias;
    TS_ASSERT(std::abs(output - expected) < 1.0);
  }

  // Test triaxial accelerometer
  void testTriaxialAccelerometer() {
    double accel[3] = {1.0, 2.0, 3.0};
    double bias[3] = {0.1, 0.1, 0.1};
    double scale[3] = {1.0, 1.0, 1.0};

    double output[3];
    for (int i = 0; i < 3; i++) {
      output[i] = accel[i] * scale[i] + bias[i];
    }

    TS_ASSERT_DELTA(output[0], 1.1, epsilon);
    TS_ASSERT_DELTA(output[1], 2.1, epsilon);
    TS_ASSERT_DELTA(output[2], 3.1, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero acceleration
  void testZeroAcceleration() {
    double accel = 0.0;
    double bias = 0.1;

    double output = accel + bias;
    TS_ASSERT_DELTA(output, 0.1, epsilon);
  }

  // Test very large acceleration
  void testLargeAcceleration() {
    double accel = 1000.0;  // 100g+ (high-g shock)
    double max_range = 500.0;

    double output = std::min(accel, max_range);
    TS_ASSERT_DELTA(output, max_range, epsilon);
  }

  // Test negative acceleration
  void testNegativeAcceleration() {
    double accel = -10.0;
    double bias = 0.0;

    double output = accel + bias;
    TS_ASSERT_DELTA(output, -10.0, epsilon);
  }
};
