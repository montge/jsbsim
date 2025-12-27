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
#include <algorithm>

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

  /***************************************************************************
   * Vibration Effects Tests
   ***************************************************************************/

  // Test vibration rectification error
  void testVibrationRectificationError() {
    // Vibration creates DC offset due to nonlinearity
    double vibration_amplitude = 5.0;  // m/s^2
    double nonlinearity_factor = 0.001;  // 0.1%

    // Rectification error ≈ nonlinearity * amplitude^2 / 2
    double rectification_error = nonlinearity_factor * vibration_amplitude * vibration_amplitude / 2.0;
    TS_ASSERT_DELTA(rectification_error, 0.0125, 0.001);
  }

  // Test vibration filtering
  void testVibrationFiltering() {
    // Anti-vibration filter cutoff
    double cutoff_freq = 100.0;  // Hz
    double vibration_freq = 500.0;  // Hz (engine vibration)

    // Filter attenuation at vibration frequency (simple 1st order)
    double attenuation = cutoff_freq / vibration_freq;
    TS_ASSERT(attenuation < 0.5);
  }

  // Test resonance frequency avoidance
  void testResonanceFrequencyAvoidance() {
    // MEMS accelerometer resonance
    double resonance_freq = 5000.0;  // Hz
    double operating_bandwidth = 200.0;  // Hz

    // Resonance should be well above operating bandwidth
    double margin = resonance_freq / operating_bandwidth;
    TS_ASSERT(margin > 10.0);
  }

  /***************************************************************************
   * Temperature Sensitivity Tests
   ***************************************************************************/

  // Test bias temperature coefficient
  void testBiasTempCoefficient() {
    double bias_25C = 0.1;  // m/s^2 at 25°C
    double temp_coeff = 0.001;  // m/s^2 per °C
    double temp = 45.0;  // °C

    double bias_at_temp = bias_25C + temp_coeff * (temp - 25.0);
    TS_ASSERT_DELTA(bias_at_temp, 0.12, 0.001);
  }

  // Test scale factor temperature coefficient
  void testScaleFactorTempCoefficient() {
    double scale_factor_25C = 1.0;
    double temp_coeff_ppm = 100;  // ppm/°C
    double temp = 55.0;

    double scale_factor = scale_factor_25C * (1.0 + temp_coeff_ppm * 1e-6 * (temp - 25.0));
    TS_ASSERT_DELTA(scale_factor, 1.003, 0.0001);
  }

  // Test operating temperature range
  void testOperatingTempRange() {
    double min_temp = -40.0;  // °C
    double max_temp = 85.0;   // °C
    double test_temp = 25.0;

    bool within_range = (test_temp >= min_temp) && (test_temp <= max_temp);
    TS_ASSERT(within_range);
  }

  /***************************************************************************
   * Turn Coordination Tests
   ***************************************************************************/

  // Test load factor in level turn
  void testLoadFactorLevelTurn() {
    double bank_angle_deg = 30.0;
    double bank_angle_rad = bank_angle_deg * M_PI / 180.0;

    // Load factor in level turn = 1 / cos(bank)
    double load_factor = 1.0 / std::cos(bank_angle_rad);
    TS_ASSERT_DELTA(load_factor, 1.1547, 0.001);
  }

  // Test lateral acceleration in coordinated turn
  void testLateralAccelCoordinatedTurn() {
    // In coordinated turn, lateral accelerometer reads zero
    double lateral_accel = 0.0;  // Coordinated
    TS_ASSERT_DELTA(lateral_accel, 0.0, 0.1);
  }

  // Test slip/skid detection
  void testSlipSkidDetection() {
    double lateral_accel = 0.5;  // m/s^2 (skidding)
    double threshold = 0.1;

    bool skidding = std::abs(lateral_accel) > threshold;
    TS_ASSERT(skidding);
  }

  // Test inclinometer equivalent
  void testInclinometerEquivalent() {
    // Lateral accel / (g * sin(bank)) gives slip angle
    double g = 9.80665;
    double lateral_accel = 0.5;
    double bank_angle_rad = 30.0 * M_PI / 180.0;

    // In coordinated turn, ratio should be 0
    // With slip, ratio indicates slip magnitude
    double expected_lateral = g * std::tan(bank_angle_rad);
    double slip_indication = (expected_lateral - lateral_accel) / g;
    TS_ASSERT(std::abs(slip_indication) < 1.0);
  }

  /***************************************************************************
   * Specific Force Tests
   ***************************************************************************/

  // Test specific force calculation
  void testSpecificForce() {
    // Specific force = total acceleration - gravity
    double total_accel_z = 19.6;  // m/s^2 (2g climb)
    double g = 9.80665;

    double specific_force = total_accel_z - g;
    TS_ASSERT_DELTA(specific_force, 9.79335, 0.01);
  }

  // Test weightlessness detection
  void testWeightlessness() {
    // In free fall, accelerometer reads zero
    double accel_freefall = 0.0;
    double threshold = 0.5;

    bool microgravity = std::abs(accel_freefall) < threshold;
    TS_ASSERT(microgravity);
  }

  // Test 2g pullup
  void testTwoGPullup() {
    double g = 9.80665;
    double load_factor = 2.0;

    double z_accel = g * load_factor;
    TS_ASSERT_DELTA(z_accel, 19.6133, 0.001);
  }

  /***************************************************************************
   * Coriolis Effect Tests
   ***************************************************************************/

  // Test Coriolis acceleration
  void testCoriolisAcceleration() {
    // a_coriolis = 2 * omega x v
    double omega = 0.5;  // rad/s (body rotation rate)
    double velocity = 10.0;  // m/s (linear velocity)

    double a_coriolis = 2.0 * omega * velocity;
    TS_ASSERT_DELTA(a_coriolis, 10.0, 0.001);
  }

  // Test navigation-grade Coriolis compensation
  void testCoriolisCompensation() {
    double earth_rate = 7.292e-5;  // rad/s
    double velocity_east = 100.0;  // m/s
    double latitude = 45.0 * M_PI / 180.0;

    // Vertical Coriolis = 2 * omega * v_east * cos(lat)
    double coriolis_vertical = 2.0 * earth_rate * velocity_east * std::cos(latitude);
    TS_ASSERT(std::abs(coriolis_vertical) < 0.02);  // Small but measurable
  }

  /***************************************************************************
   * High-G Response Tests
   ***************************************************************************/

  // Test high-g linearity
  void testHighGLinearity() {
    double inputs[] = {1.0, 5.0, 10.0, 20.0, 50.0};  // g
    double scale = 1.0;
    double nonlinearity = 0.0001;  // 0.01% per g

    for (double input : inputs) {
      double error = nonlinearity * input * input;
      TS_ASSERT(error < 0.3);  // Less than 0.3g error
    }
  }

  // Test recovery after high-g
  void testHighGRecovery() {
    // After 50g shock, bias should return to normal
    double pre_shock_bias = 0.1;
    double shock_induced_shift = 0.02;
    double recovery_time = 0.1;  // seconds

    // Simple exponential recovery model
    double post_shock_bias = pre_shock_bias + shock_induced_shift * std::exp(-recovery_time / 0.01);
    TS_ASSERT(std::abs(post_shock_bias - pre_shock_bias) < 0.05);
  }

  // Test g-switch threshold
  void testGSwitchThreshold() {
    double threshold_g = 4.0;  // 4g
    double g = 9.80665;
    double threshold_mps2 = threshold_g * g;

    double measured = 45.0;  // m/s^2
    bool triggered = measured > threshold_mps2;
    TS_ASSERT(triggered);
  }

  /***************************************************************************
   * Sample Rate and Aliasing Tests
   ***************************************************************************/

  // Test Nyquist frequency
  void testNyquistFrequency() {
    double sample_rate = 1000.0;  // Hz
    double nyquist = sample_rate / 2.0;

    TS_ASSERT_DELTA(nyquist, 500.0, 0.001);
  }

  // Test anti-aliasing filter requirement
  void testAntiAliasingFilter() {
    double sample_rate = 1000.0;
    double max_signal_freq = 100.0;
    double filter_cutoff = 200.0;

    // Filter cutoff should be < Nyquist
    bool properly_filtered = filter_cutoff < (sample_rate / 2.0);
    TS_ASSERT(properly_filtered);
  }

  // Test decimation filter
  void testDecimationFilter() {
    double raw_sample_rate = 5000.0;
    double output_sample_rate = 100.0;
    int decimation_factor = static_cast<int>(raw_sample_rate / output_sample_rate);

    TS_ASSERT_EQUALS(decimation_factor, 50);
  }

  /***************************************************************************
   * Integration for Velocity Tests
   ***************************************************************************/

  // Test velocity from integration
  void testVelocityIntegration() {
    double accel = 2.0;  // m/s^2
    double dt = 0.01;  // seconds
    double duration = 10.0;  // seconds

    double velocity = 0.0;
    for (double t = 0.0; t < duration; t += dt) {
      velocity += accel * dt;
    }

    TS_ASSERT_DELTA(velocity, 20.0, 0.05);  // v = a * t (small integration error)
  }

  // Test bias effect on integrated velocity
  void testBiasEffectOnIntegration() {
    double accel = 0.0;  // True acceleration
    double bias = 0.001;  // m/s^2 (1 milli-g)
    double duration = 60.0;  // seconds

    double velocity_error = bias * duration;
    TS_ASSERT_DELTA(velocity_error, 0.06, 0.001);  // 6 cm/s error
  }

  // Test trapezoidal integration
  void testTrapezoidalIntegration() {
    double accel_prev = 1.0;
    double accel_curr = 2.0;
    double dt = 0.01;

    double delta_v = (accel_prev + accel_curr) / 2.0 * dt;
    TS_ASSERT_DELTA(delta_v, 0.015, 0.0001);
  }

  /***************************************************************************
   * MEMS-Specific Tests
   ***************************************************************************/

  // Test MEMS proof mass
  void testMEMSProofMass() {
    // F = m * a, displacement x = F / k
    double mass = 1e-9;  // kg (nanogram range)
    double spring_constant = 1.0;  // N/m
    double accel = 10.0;  // m/s^2

    double displacement = mass * accel / spring_constant;
    TS_ASSERT_DELTA(displacement, 1e-8, 1e-10);  // 10 nm
  }

  // Test capacitive sensing
  void testCapacitiveSensing() {
    // Capacitance changes with gap
    double epsilon_0 = 8.854e-12;  // F/m
    double area = 1e-6;  // m^2
    double gap = 2e-6;  // m

    double capacitance = epsilon_0 * area / gap;
    TS_ASSERT(capacitance > 0.0);
  }

  // Test in-run bias stability
  void testInRunBiasStability() {
    // MEMS bias stability specification
    double bias_stability_1sigma = 0.05;  // m/s^2 (typical MEMS)
    double test_bias_variation = 0.03;

    bool within_spec = test_bias_variation < bias_stability_1sigma;
    TS_ASSERT(within_spec);
  }

  /***************************************************************************
   * Multi-Axis Accelerometer Tests
   ***************************************************************************/

  // Test total acceleration magnitude
  void testTotalAccelerationMagnitude() {
    double ax = 3.0, ay = 4.0, az = 0.0;

    double magnitude = std::sqrt(ax*ax + ay*ay + az*az);
    TS_ASSERT_DELTA(magnitude, 5.0, epsilon);
  }

  // Test gravity vector decomposition
  void testGravityVectorDecomposition() {
    double g = 9.80665;
    double pitch = 30.0 * M_PI / 180.0;  // 30 degree pitch

    double gx = g * std::sin(pitch);
    double gz = g * std::cos(pitch);

    TS_ASSERT_DELTA(gx, 4.903, 0.01);
    TS_ASSERT_DELTA(gz, 8.493, 0.01);
  }

  // Test attitude estimation from accelerometer
  void testAttitudeFromAccelerometer() {
    double ax = 0.0;
    double ay = 0.0;
    double az = -9.80665;

    // Pitch and roll from gravity
    double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    double roll = std::atan2(ay, -az);

    TS_ASSERT_DELTA(pitch, 0.0, 0.01);
    TS_ASSERT_DELTA(roll, 0.0, 0.01);
  }

  /***************************************************************************
   * Error Budget Tests
   ***************************************************************************/

  // Test RSS error combination
  void testRSSErrorCombination() {
    double bias_error = 0.1;
    double scale_error = 0.05;
    double noise_error = 0.02;

    double total_error = std::sqrt(bias_error*bias_error +
                                   scale_error*scale_error +
                                   noise_error*noise_error);
    TS_ASSERT_DELTA(total_error, 0.1137, 0.001);
  }

  // Test navigation-grade requirements
  void testNavigationGradeRequirements() {
    // Navigation grade: < 1 mg bias stability
    double bias_stability_mg = 1.0;
    double bias_stability_mps2 = bias_stability_mg * 9.80665e-3;

    TS_ASSERT(bias_stability_mps2 < 0.01);  // < 0.01 m/s^2
  }

  // Test tactical-grade requirements
  void testTacticalGradeRequirements() {
    // Tactical grade: < 1 mg bias repeatability
    double bias_repeatability_mg = 1.0;
    double scale_factor_accuracy_ppm = 300.0;

    TS_ASSERT(bias_repeatability_mg < 5.0);
    TS_ASSERT(scale_factor_accuracy_ppm < 500.0);
  }

  /***************************************************************************
   * Dynamic Range Tests
   ***************************************************************************/

  // Test dynamic range calculation
  void testDynamicRangeCalculation() {
    double max_accel = 50.0;  // g
    double noise_floor = 0.001;  // g

    double dynamic_range_db = 20.0 * std::log10(max_accel / noise_floor);
    TS_ASSERT_DELTA(dynamic_range_db, 94.0, 1.0);  // ~94 dB
  }

  // Test resolution vs range tradeoff
  void testResolutionRangeTradeoff() {
    int bits = 16;
    double range = 50.0;  // g

    double resolution = (2.0 * range) / (1 << bits);
    TS_ASSERT_DELTA(resolution, 0.00153, 0.0001);  // g
  }

  /***************************************************************************
   * Flight Dynamics Correlation Tests
   ***************************************************************************/

  // Test stall warning from acceleration
  void testStallWarningFromAcceleration() {
    // Sudden decrease in load factor indicates impending stall
    double load_factor_cruise = 1.0;
    double load_factor_stall = 0.7;  // Load factor drop at stall

    double load_factor_drop = load_factor_cruise - load_factor_stall;
    TS_ASSERT(load_factor_drop > 0.2);  // Significant drop
  }

  // Test buffet detection
  void testBuffetDetection() {
    // High-frequency acceleration indicates buffet
    double buffet_amplitude = 0.5;  // g peak-to-peak
    double buffet_threshold = 0.2;

    bool buffet_detected = buffet_amplitude > buffet_threshold;
    TS_ASSERT(buffet_detected);
  }

  // Test landing impact measurement
  void testLandingImpactMeasurement() {
    double impact_g = 2.5;  // Hard landing
    double normal_landing_g = 1.2;
    double hard_landing_threshold = 2.0;

    bool hard_landing = impact_g > hard_landing_threshold;
    TS_ASSERT(hard_landing);
  }

  /***************************************************************************
   * Power Spectral Density Tests
   ***************************************************************************/

  // Test RMS noise calculation
  void testRMSNoiseCalculation() {
    // RMS = sqrt(PSD * bandwidth)
    double psd = 0.0001;  // (m/s^2)^2/Hz
    double bandwidth = 100.0;  // Hz

    double rms_noise = std::sqrt(psd * bandwidth);
    TS_ASSERT_DELTA(rms_noise, 0.1, 0.001);
  }

  // Test noise density specification
  void testNoiseDensitySpec() {
    // Typical MEMS: 100-500 ug/sqrt(Hz)
    double noise_density_ug = 200.0;  // ug/sqrt(Hz)
    double noise_density_g = noise_density_ug * 1e-6;
    double bandwidth = 100.0;  // Hz

    double rms_noise_g = noise_density_g * std::sqrt(bandwidth);
    TS_ASSERT_DELTA(rms_noise_g, 0.002, 0.0001);  // 2 milli-g
  }

  // Test velocity random walk
  void testVelocityRandomWalk() {
    // VRW = noise_density * sqrt(time)
    double noise_density = 0.0001;  // m/s^2/sqrt(Hz)
    double time = 100.0;  // seconds

    double velocity_error = noise_density * std::sqrt(time);
    TS_ASSERT_DELTA(velocity_error, 0.001, 0.0001);
  }

  /***************************************************************************
   * Shock and Vibration Survival Tests
   ***************************************************************************/

  // Test shock survival
  void testShockSurvival() {
    double shock_level_g = 500.0;
    double max_survival = 1000.0;

    bool survives = shock_level_g < max_survival;
    TS_ASSERT(survives);
  }

  // Test vibration fatigue
  void testVibrationFatigue() {
    double vibration_amplitude = 5.0;  // g
    double cycles = 1e7;
    double fatigue_limit_cycles = 1e8;

    bool within_limit = cycles < fatigue_limit_cycles;
    TS_ASSERT(within_limit);
  }

  // Test random vibration response
  void testRandomVibrationResponse() {
    // GRMS = sqrt(integral of PSD)
    double grms_input = 10.0;
    double q_factor = 10.0;  // Resonance amplification

    double grms_response = grms_input * q_factor;
    TS_ASSERT_DELTA(grms_response, 100.0, epsilon);
  }

  /***************************************************************************
   * Calibration Tests
   ***************************************************************************/

  // Test multi-point calibration
  void testMultiPointCalibration() {
    // 6-point tumble calibration
    double measurements[] = {9.81, -9.81, 0.0, 0.0, 0.0, 0.0};  // +X, -X, +Y, -Y, +Z, -Z (simplified)

    double scale = (measurements[0] - measurements[1]) / (2.0 * 9.81);
    double bias = (measurements[0] + measurements[1]) / 2.0;

    TS_ASSERT_DELTA(scale, 1.0, 0.01);
    TS_ASSERT_DELTA(bias, 0.0, 0.1);
  }

  // Test in-situ calibration check
  void testInSituCalibration() {
    double g = 9.80665;
    double ax = 0.1, ay = 0.1, az = -9.8;  // Slight misalignment

    double measured_g = std::sqrt(ax*ax + ay*ay + az*az);
    double g_error = std::abs(measured_g - g) / g * 100.0;

    TS_ASSERT(g_error < 1.0);  // Less than 1% error
  }

  // Test cross-axis alignment calibration
  void testCrossAxisAlignment() {
    // Misalignment matrix (small angles)
    double theta_xy = 0.001;  // rad
    double theta_xz = 0.001;
    double theta_yz = 0.001;

    // Cross-coupling error
    double ax_true = 10.0, ay_true = 0.0, az_true = 0.0;
    double ax_measured = ax_true + theta_xy * ay_true + theta_xz * az_true;

    TS_ASSERT_DELTA(ax_measured, 10.0, 0.01);
  }

  /***************************************************************************
   * AHRS Integration Tests
   ***************************************************************************/

  // Test complementary filter
  void testComplementaryFilter() {
    double accel_pitch = 5.0;  // degrees from accelerometer
    double gyro_pitch = 5.5;   // degrees from gyro integration
    double alpha = 0.98;       // Gyro weight

    double filtered_pitch = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch;
    TS_ASSERT_DELTA(filtered_pitch, 5.49, 0.01);
  }

  // Test gravity vector estimation
  void testGravityVectorEstimation() {
    double ax = -2.0, ay = 0.0, az = -9.6;  // Pitched up
    double g = 9.80665;

    // Gravity in body frame indicates attitude
    double pitch_est = std::asin(-ax / g) * 180.0 / M_PI;
    TS_ASSERT_DELTA(pitch_est, 11.77, 0.1);  // About 12 degrees nose up
  }

  // Test acceleration rejection
  void testAccelerationRejection() {
    // Detect non-gravity acceleration
    double ax = 5.0, ay = 0.0, az = -9.8;
    double g = 9.80665;

    double total_accel = std::sqrt(ax*ax + ay*ay + az*az);
    bool under_acceleration = std::abs(total_accel - g) > 0.5;

    TS_ASSERT(under_acceleration);
  }

  /***************************************************************************
   * Aerospace Application Tests
   ***************************************************************************/

  // Test strapdown navigation acceleration
  void testStrapdownNavAcceleration() {
    // Transform body acceleration to nav frame
    double ax_body = 1.0, ay_body = 0.0, az_body = -10.0;
    double pitch = 10.0 * M_PI / 180.0;  // 10 degrees nose up

    double ax_nav = ax_body * std::cos(pitch) + az_body * std::sin(pitch);
    double az_nav = -ax_body * std::sin(pitch) + az_body * std::cos(pitch);

    TS_ASSERT_DELTA(ax_nav, -0.75, 0.1);  // Mostly deceleration in nav frame
  }

  // Test sculling error
  void testScullingError() {
    // Sculling occurs from correlated vibration in accel and gyro
    double accel_amplitude = 1.0;  // g
    double rotation_rate = 0.1;    // rad/s
    double frequency = 50.0;       // Hz

    // Sculling error magnitude (simplified)
    double sculling_error = accel_amplitude * rotation_rate / (4.0 * frequency);
    TS_ASSERT(sculling_error < 0.01);  // Should be small
  }

  // Test coning error detection
  void testConingErrorDetection() {
    double cone_half_angle = 5.0 * M_PI / 180.0;  // rad
    double cone_frequency = 10.0;  // Hz

    // Coning error scales with cone angle squared
    double coning_error = cone_half_angle * cone_half_angle / 2.0;
    TS_ASSERT(coning_error < 0.01);
  }

  /***************************************************************************
   * Structural Health Monitoring Tests
   ***************************************************************************/

  // Test modal frequency detection
  void testModalFrequencyDetection() {
    double sample_rate = 1000.0;  // Hz
    double modal_frequency = 25.0;  // Hz (wing flutter)

    // Check Nyquist
    bool can_detect = modal_frequency < sample_rate / 2.0;
    TS_ASSERT(can_detect);
  }

  // Test structural damping estimation
  void testStructuralDampingEstimation() {
    // Logarithmic decrement method
    double amplitude_1 = 10.0;
    double amplitude_n = 5.0;
    int n_cycles = 10;

    double log_dec = std::log(amplitude_1 / amplitude_n) / n_cycles;
    double damping_ratio = log_dec / (2.0 * M_PI);

    TS_ASSERT_DELTA(damping_ratio, 0.011, 0.001);
  }

  // Test flutter boundary detection
  void testFlutterBoundaryDetection() {
    double current_frequency = 15.0;  // Hz
    double flutter_frequency = 18.0;  // Hz
    double margin = 0.9;  // 90% warning threshold

    bool flutter_warning = current_frequency > flutter_frequency * margin;
    TS_ASSERT(!flutter_warning);
  }

  /***************************************************************************
   * Accelerometer Fusion Tests
   ***************************************************************************/

  // Test redundant sensor voting
  void testRedundantSensorVoting() {
    double sensor1 = 10.0, sensor2 = 10.1, sensor3 = 10.05;
    double tolerance = 0.5;

    // Mid-value selection
    double values[] = {sensor1, sensor2, sensor3};
    std::sort(values, values + 3);
    double voted_value = values[1];  // Median

    TS_ASSERT_DELTA(voted_value, 10.05, epsilon);
  }

  // Test sensor failure detection
  void testSensorFailureDetection() {
    double sensor1 = 10.0, sensor2 = 10.1, sensor3 = 50.0;  // sensor3 failed
    double mean = (sensor1 + sensor2) / 2.0;
    double threshold = 5.0;

    bool sensor3_failed = std::abs(sensor3 - mean) > threshold;
    TS_ASSERT(sensor3_failed);
  }

  // Test weighted average fusion
  void testWeightedAverageFusion() {
    double sensor1 = 10.0, sigma1 = 0.1;
    double sensor2 = 10.2, sigma2 = 0.2;

    // Optimal weighted average
    double w1 = 1.0 / (sigma1 * sigma1);
    double w2 = 1.0 / (sigma2 * sigma2);
    double fused = (w1 * sensor1 + w2 * sensor2) / (w1 + w2);

    TS_ASSERT_DELTA(fused, 10.04, 0.01);
  }

  /***************************************************************************
   * Digital Signal Processing Tests
   ***************************************************************************/

  // Test moving average filter
  void testMovingAverageFilter() {
    double samples[] = {10.0, 11.0, 10.0, 11.0, 10.0};
    int window = 5;
    double sum = 0.0;

    for (int i = 0; i < window; i++) {
      sum += samples[i];
    }
    double average = sum / window;

    TS_ASSERT_DELTA(average, 10.4, epsilon);
  }

  // Test IIR filter implementation
  void testIIRFilterImplementation() {
    // Simple first-order IIR: y[n] = a * x[n] + (1-a) * y[n-1]
    double a = 0.2;
    double y_prev = 0.0;
    double x = 10.0;

    double y = a * x + (1.0 - a) * y_prev;
    TS_ASSERT_DELTA(y, 2.0, epsilon);

    // After many iterations with constant input
    for (int i = 0; i < 50; i++) {
      y = a * x + (1.0 - a) * y;
    }
    TS_ASSERT_DELTA(y, 10.0, 0.01);
  }

  // Test high-pass filter for DC removal
  void testHighPassDCRemoval() {
    double dc_offset = 0.5;
    double signal = 10.0;
    double measured = signal + dc_offset;

    // After high-pass filter, DC should be removed
    double ac_component = measured - dc_offset;
    TS_ASSERT_DELTA(ac_component, 10.0, epsilon);
  }
};
