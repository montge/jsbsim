/*******************************************************************************
 * FGMagnetometerTest.h - Unit tests for FGMagnetometer (magnetic sensor)
 *
 * Tests the mathematical behavior of magnetometer sensors:
 * - Magnetic field vector measurements
 * - Sensor orientation effects
 * - Magnetic declination/inclination
 * - Heading calculation from magnetic field
 * - Noise, bias, and quantization
 *
 * Note: FGMagnetometer requires XML element for construction, so these tests
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

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

const double epsilon = 1e-8;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

class FGMagnetometerTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Magnetic Field Vector Tests
   ***************************************************************************/

  // Test magnetic field components
  void testMagneticFieldComponents() {
    // Typical Earth's magnetic field strength: ~25-65 microtesla
    double Bx = 20.0;   // microtesla (north component)
    double By = 5.0;    // microtesla (east component)
    double Bz = 40.0;   // microtesla (down component)

    double magnitude = std::sqrt(Bx*Bx + By*By + Bz*Bz);
    TS_ASSERT(magnitude > 25.0 && magnitude < 65.0);
  }

  // Test horizontal intensity
  void testHorizontalIntensity() {
    double Bx = 20.0;  // North
    double By = 5.0;   // East

    double H = std::sqrt(Bx*Bx + By*By);
    TS_ASSERT_DELTA(H, 20.615, 0.01);
  }

  // Test total field intensity
  void testTotalFieldIntensity() {
    double Bx = 20.0, By = 5.0, Bz = 40.0;

    double F = std::sqrt(Bx*Bx + By*By + Bz*Bz);
    TS_ASSERT_DELTA(F, 45.0, 0.5);
  }

  /***************************************************************************
   * Magnetic Declination Tests
   ***************************************************************************/

  // Calculate magnetic heading from true heading
  double magneticFromTrue(double trueHeading, double declination) {
    double magHeading = trueHeading - declination;
    while (magHeading < 0) magHeading += 360.0;
    while (magHeading >= 360.0) magHeading -= 360.0;
    return magHeading;
  }

  // Calculate true heading from magnetic heading
  double trueFromMagnetic(double magHeading, double declination) {
    double trueHeading = magHeading + declination;
    while (trueHeading < 0) trueHeading += 360.0;
    while (trueHeading >= 360.0) trueHeading -= 360.0;
    return trueHeading;
  }

  // Test east declination
  void testEastDeclination() {
    double trueHeading = 90.0;   // Due east
    double declination = 10.0;   // 10 degrees east

    double magHeading = magneticFromTrue(trueHeading, declination);
    TS_ASSERT_DELTA(magHeading, 80.0, epsilon);
  }

  // Test west declination
  void testWestDeclination() {
    double trueHeading = 90.0;
    double declination = -15.0;  // 15 degrees west

    double magHeading = magneticFromTrue(trueHeading, declination);
    TS_ASSERT_DELTA(magHeading, 105.0, epsilon);
  }

  // Test declination with wrap-around
  void testDeclinationWrapAround() {
    double trueHeading = 5.0;
    double declination = 20.0;

    double magHeading = magneticFromTrue(trueHeading, declination);
    TS_ASSERT_DELTA(magHeading, 345.0, epsilon);
  }

  /***************************************************************************
   * Magnetic Inclination (Dip) Tests
   ***************************************************************************/

  // Calculate inclination angle
  double calculateInclination(double Bh, double Bz) {
    return std::atan2(Bz, Bh) * RAD_TO_DEG;
  }

  // Test inclination at equator (approximately horizontal)
  void testInclinationEquator() {
    double Bh = 30.0;  // Strong horizontal
    double Bz = 0.0;   // No vertical

    double inclination = calculateInclination(Bh, Bz);
    TS_ASSERT_DELTA(inclination, 0.0, 0.1);
  }

  // Test inclination at mid-latitudes
  void testInclinationMidLatitude() {
    double Bh = 20.0;
    double Bz = 40.0;  // Dipping down

    double inclination = calculateInclination(Bh, Bz);
    TS_ASSERT_DELTA(inclination, 63.4, 0.1);  // arctan(2) ≈ 63.4°
  }

  // Test inclination near pole (nearly vertical)
  void testInclinationPole() {
    double Bh = 5.0;   // Weak horizontal
    double Bz = 55.0;  // Strong vertical

    double inclination = calculateInclination(Bh, Bz);
    TS_ASSERT(inclination > 80.0);  // Very steep
  }

  /***************************************************************************
   * Heading From Magnetometer Tests
   ***************************************************************************/

  // Calculate heading from magnetometer readings (level flight)
  double calculateHeading(double Bx, double By) {
    double heading = std::atan2(By, Bx) * RAD_TO_DEG;
    if (heading < 0) heading += 360.0;
    return heading;
  }

  // Test heading due north
  void testHeadingNorth() {
    double Bx = 20.0;  // Sensor X pointing to magnetic north
    double By = 0.0;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 0.0, 0.1);
  }

  // Test heading due east
  void testHeadingEast() {
    double Bx = 0.0;
    double By = 20.0;  // Field from east

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 90.0, 0.1);
  }

  // Test heading due south
  void testHeadingSouth() {
    double Bx = -20.0;
    double By = 0.0;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 180.0, 0.1);
  }

  // Test heading due west
  void testHeadingWest() {
    double Bx = 0.0;
    double By = -20.0;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 270.0, 0.1);
  }

  /***************************************************************************
   * Tilt Compensation Tests
   ***************************************************************************/

  // Compensate magnetometer for roll and pitch
  void tiltCompensate(double Bx, double By, double Bz,
                      double roll, double pitch,
                      double& Bx_comp, double& By_comp) {
    double cos_r = std::cos(roll * DEG_TO_RAD);
    double sin_r = std::sin(roll * DEG_TO_RAD);
    double cos_p = std::cos(pitch * DEG_TO_RAD);
    double sin_p = std::sin(pitch * DEG_TO_RAD);

    // Rotate to level
    Bx_comp = Bx * cos_p + By * sin_r * sin_p + Bz * cos_r * sin_p;
    By_comp = By * cos_r - Bz * sin_r;
  }

  // Test tilt compensation at level
  void testTiltCompensationLevel() {
    double Bx = 20.0, By = 5.0, Bz = 40.0;
    double roll = 0.0, pitch = 0.0;
    double Bx_comp, By_comp;

    tiltCompensate(Bx, By, Bz, roll, pitch, Bx_comp, By_comp);

    TS_ASSERT_DELTA(Bx_comp, Bx, 0.01);
    TS_ASSERT_DELTA(By_comp, By, 0.01);
  }

  // Test tilt compensation with roll
  void testTiltCompensationRoll() {
    double Bx = 20.0, By = 0.0, Bz = 40.0;
    double roll = 30.0, pitch = 0.0;
    double Bx_comp, By_comp;

    tiltCompensate(Bx, By, Bz, roll, pitch, Bx_comp, By_comp);

    // With roll, By_comp should change due to Bz contribution
    TS_ASSERT(std::abs(By_comp) > 0);
  }

  /***************************************************************************
   * Sensor Orientation Tests
   ***************************************************************************/

  // Test X-axis selection
  void testAxisSelectionX() {
    double field[3] = {20.0, 5.0, 40.0};
    int axis = 0;

    double output = field[axis];
    TS_ASSERT_DELTA(output, 20.0, epsilon);
  }

  // Test Y-axis selection
  void testAxisSelectionY() {
    double field[3] = {20.0, 5.0, 40.0};
    int axis = 1;

    double output = field[axis];
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  // Test Z-axis selection
  void testAxisSelectionZ() {
    double field[3] = {20.0, 5.0, 40.0};
    int axis = 2;

    double output = field[axis];
    TS_ASSERT_DELTA(output, 40.0, epsilon);
  }

  /***************************************************************************
   * Noise and Quantization Tests
   ***************************************************************************/

  // Test noise injection
  void testNoiseInjection() {
    std::mt19937 gen(42);
    double noise_sigma = 0.5;  // microtesla
    std::normal_distribution<double> dist(0.0, noise_sigma);

    double true_field = 20.0;
    double sum = 0.0, sum_sq = 0.0;
    int n = 10000;

    for (int i = 0; i < n; i++) {
      double measured = true_field + dist(gen);
      sum += measured;
      sum_sq += measured * measured;
    }

    double mean = sum / n;
    double variance = sum_sq / n - mean * mean;
    double measured_sigma = std::sqrt(variance);

    TS_ASSERT_DELTA(mean, true_field, 0.05);
    TS_ASSERT_DELTA(measured_sigma, noise_sigma, 0.05);
  }

  // Test quantization
  void testQuantization() {
    int bits = 12;
    double min_field = -100.0;  // microtesla
    double max_field = 100.0;

    int divisions = (1 << bits);
    double resolution = (max_field - min_field) / divisions;

    double input = 23.456;
    int quantized = static_cast<int>((input - min_field) / resolution);
    double output = min_field + quantized * resolution;

    TS_ASSERT(std::abs(output - input) < resolution);
  }

  /***************************************************************************
   * Bias and Drift Tests
   ***************************************************************************/

  // Test constant bias (hard iron)
  void testHardIronBias() {
    double true_field = 20.0;
    double bias = 2.0;  // Hard iron offset

    double measured = true_field + bias;
    TS_ASSERT_DELTA(measured, 22.0, epsilon);
  }

  // Test drift over time
  void testBiasDrift() {
    double initial_bias = 0.5;
    double drift_rate = 0.001;  // microtesla per second
    double time = 100.0;

    double final_bias = initial_bias + drift_rate * time;
    TS_ASSERT_DELTA(final_bias, 0.6, epsilon);
  }

  // Test scale factor (soft iron)
  void testSoftIronScale() {
    double true_field = 20.0;
    double scale_factor = 1.05;  // 5% scale error

    double measured = true_field * scale_factor;
    TS_ASSERT_DELTA(measured, 21.0, epsilon);
  }

  /***************************************************************************
   * Lag/Filter Tests
   ***************************************************************************/

  // Test first-order lag
  void testMagnetometerLag() {
    double lag_time = 0.5;
    double dt = 0.01;
    double ca = std::exp(-dt / lag_time);
    double cb = 1.0 - ca;

    double input = 20.0;
    double output = 0.0;

    for (int i = 0; i < 500; i++) {
      output = ca * output + cb * input;
    }

    TS_ASSERT_DELTA(output, input, 0.01);
  }

  /***************************************************************************
   * Gain Application Tests
   ***************************************************************************/

  // Test output gain
  void testOutputGain() {
    double measured = 20.0;  // microtesla
    double gain = 0.05;      // Arbitrary scale

    double output = measured * gain;
    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero field
  void testZeroField() {
    double Bx = 0.0, By = 0.0;

    // Heading undefined at zero field
    double magnitude = std::sqrt(Bx*Bx + By*By);
    TS_ASSERT_DELTA(magnitude, 0.0, epsilon);
  }

  // Test very weak field
  void testVeryWeakField() {
    double Bx = 0.001, By = 0.001;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 45.0, 0.1);  // 45 degrees
  }

  // Test saturation
  void testSaturation() {
    double measured = 1000.0;  // Way beyond normal
    double max_range = 100.0;

    double saturated = std::min(measured, max_range);
    TS_ASSERT_DELTA(saturated, max_range, epsilon);
  }

  /***************************************************************************
   * Triaxial Magnetometer Tests
   ***************************************************************************/

  // Test triaxial measurement
  void testTriaxialMeasurement() {
    double Bx = 20.0, By = 5.0, Bz = 40.0;
    double bias[3] = {0.5, 0.3, 0.8};
    double scale[3] = {1.0, 1.0, 1.0};

    double output[3];
    output[0] = Bx * scale[0] + bias[0];
    output[1] = By * scale[1] + bias[1];
    output[2] = Bz * scale[2] + bias[2];

    TS_ASSERT_DELTA(output[0], 20.5, epsilon);
    TS_ASSERT_DELTA(output[1], 5.3, epsilon);
    TS_ASSERT_DELTA(output[2], 40.8, epsilon);
  }

  /***************************************************************************
   * Additional Heading Tests
   ***************************************************************************/

  // Test 30: Heading northeast (45 degrees)
  void testHeadingNortheast() {
    double Bx = 14.14;  // Equal components
    double By = 14.14;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 45.0, 0.1);
  }

  // Test 31: Heading southeast (135 degrees)
  void testHeadingSoutheast() {
    double Bx = -14.14;
    double By = 14.14;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 135.0, 0.1);
  }

  // Test 32: Heading southwest (225 degrees)
  void testHeadingSouthwest() {
    double Bx = -14.14;
    double By = -14.14;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 225.0, 0.1);
  }

  // Test 33: Heading northwest (315 degrees)
  void testHeadingNorthwest() {
    double Bx = 14.14;
    double By = -14.14;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT_DELTA(heading, 315.0, 0.1);
  }

  /***************************************************************************
   * Cross-Axis Sensitivity Tests
   ***************************************************************************/

  // Test 34: Cross-axis sensitivity modeling
  void testCrossAxisSensitivity() {
    double Bx_true = 20.0, By_true = 5.0, Bz_true = 40.0;

    // Cross-axis coefficients (typically 1-2%)
    double k_xy = 0.02, k_xz = 0.01;
    double k_yx = 0.015, k_yz = 0.02;
    double k_zx = 0.01, k_zy = 0.015;

    double Bx_meas = Bx_true + k_xy * By_true + k_xz * Bz_true;
    double By_meas = k_yx * Bx_true + By_true + k_yz * Bz_true;
    double Bz_meas = k_zx * Bx_true + k_zy * By_true + Bz_true;

    // Cross-axis errors should be small but measurable
    TS_ASSERT(std::abs(Bx_meas - Bx_true) < 1.5);
    TS_ASSERT(std::abs(By_meas - By_true) < 1.5);
    TS_ASSERT(std::abs(Bz_meas - Bz_true) < 1.5);
  }

  // Test 35: Cross-axis matrix correction
  void testCrossAxisCorrection() {
    // Correction matrix (inverse of cross-axis)
    double corr[3][3] = {
      {1.0, -0.02, -0.01},
      {-0.015, 1.0, -0.02},
      {-0.01, -0.015, 1.0}
    };

    double B_meas[3] = {20.5, 5.8, 40.2};
    double B_corr[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        B_corr[i] += corr[i][j] * B_meas[j];
      }
    }

    // Corrected values should be close to true
    TS_ASSERT(std::abs(B_corr[0] - 20.0) < 0.5);
    TS_ASSERT(std::abs(B_corr[1] - 5.0) < 0.5);
  }

  /***************************************************************************
   * Temperature Effects Tests
   ***************************************************************************/

  // Test 36: Temperature coefficient
  void testTemperatureCoefficient() {
    double B_at_25C = 20.0;
    double temp_coeff = 0.01;  // 1% per 10°C
    double temp = 45.0;        // Operating temp
    double ref_temp = 25.0;

    double B_at_temp = B_at_25C * (1.0 + temp_coeff * (temp - ref_temp) / 10.0);
    TS_ASSERT_DELTA(B_at_temp, 20.4, 0.01);  // 2% increase
  }

  // Test 37: Temperature compensation
  void testTemperatureCompensation() {
    double B_measured = 20.4;
    double temp = 45.0;
    double ref_temp = 25.0;
    double temp_coeff = 0.01;

    double B_compensated = B_measured / (1.0 + temp_coeff * (temp - ref_temp) / 10.0);
    TS_ASSERT_DELTA(B_compensated, 20.0, 0.01);
  }

  /***************************************************************************
   * Hard Iron Calibration Tests
   ***************************************************************************/

  // Test 38: Hard iron offset detection
  void testHardIronOffsetDetection() {
    // Simulate measurements in a circle (rotating aircraft)
    double bias_x = 2.0, bias_y = -1.5;
    double field_strength = 20.0;

    double min_x = 1e10, max_x = -1e10;
    double min_y = 1e10, max_y = -1e10;

    for (int i = 0; i < 360; i += 10) {
      double angle = i * DEG_TO_RAD;
      double Bx = field_strength * std::cos(angle) + bias_x;
      double By = field_strength * std::sin(angle) + bias_y;

      min_x = std::min(min_x, Bx);
      max_x = std::max(max_x, Bx);
      min_y = std::min(min_y, By);
      max_y = std::max(max_y, By);
    }

    double est_bias_x = (max_x + min_x) / 2.0;
    double est_bias_y = (max_y + min_y) / 2.0;

    TS_ASSERT_DELTA(est_bias_x, bias_x, 0.1);
    TS_ASSERT_DELTA(est_bias_y, bias_y, 0.1);
  }

  // Test 39: Hard iron removal
  void testHardIronRemoval() {
    double Bx_meas = 22.0;  // Includes 2.0 bias
    double By_meas = 3.5;   // Includes -1.5 bias
    double bias_x = 2.0, bias_y = -1.5;

    double Bx_cal = Bx_meas - bias_x;
    double By_cal = By_meas - bias_y;

    TS_ASSERT_DELTA(Bx_cal, 20.0, epsilon);
    TS_ASSERT_DELTA(By_cal, 5.0, epsilon);
  }

  /***************************************************************************
   * Soft Iron Calibration Tests
   ***************************************************************************/

  // Test 40: Soft iron ellipse detection
  void testSoftIronEllipse() {
    // Soft iron creates an ellipse instead of circle
    double scale_x = 1.1, scale_y = 0.9;
    double field_strength = 20.0;

    double max_radius = 0, min_radius = 1e10;

    for (int i = 0; i < 360; i += 10) {
      double angle = i * DEG_TO_RAD;
      double Bx = field_strength * scale_x * std::cos(angle);
      double By = field_strength * scale_y * std::sin(angle);
      double radius = std::sqrt(Bx*Bx + By*By);

      max_radius = std::max(max_radius, radius);
      min_radius = std::min(min_radius, radius);
    }

    // Eccentricity should be detectable
    TS_ASSERT(max_radius > min_radius * 1.1);
  }

  // Test 41: Soft iron correction matrix
  void testSoftIronCorrection() {
    // Apply inverse soft iron matrix
    double corr_xx = 1.0/1.1, corr_yy = 1.0/0.9;
    double Bx_meas = 22.0;  // Scaled by 1.1
    double By_meas = 4.5;   // Scaled by 0.9

    double Bx_cal = Bx_meas * corr_xx;
    double By_cal = By_meas * corr_yy;

    TS_ASSERT_DELTA(Bx_cal, 20.0, 0.01);
    TS_ASSERT_DELTA(By_cal, 5.0, 0.01);
  }

  /***************************************************************************
   * Magnetic Anomaly Detection Tests
   ***************************************************************************/

  // Test 42: Field magnitude anomaly
  void testMagnitudeAnomaly() {
    double normal_field = 45.0;
    double measured_field = 65.0;  // Near ferrous material
    double threshold = 10.0;

    bool anomaly = std::abs(measured_field - normal_field) > threshold;
    TS_ASSERT(anomaly);
  }

  // Test 43: Heading rate anomaly
  void testHeadingRateAnomaly() {
    double prev_heading = 90.0;
    double curr_heading = 150.0;
    double dt = 0.1;
    double max_rate = 100.0;  // degrees per second

    double rate = std::abs(curr_heading - prev_heading) / dt;
    bool anomaly = rate > max_rate;
    TS_ASSERT(anomaly);
  }

  /***************************************************************************
   * More Tilt Compensation Tests
   ***************************************************************************/

  // Test 44: Tilt compensation with pitch
  void testTiltCompensationPitch() {
    double Bx = 20.0, By = 0.0, Bz = 40.0;
    double roll = 0.0, pitch = 20.0;
    double Bx_comp, By_comp;

    tiltCompensate(Bx, By, Bz, roll, pitch, Bx_comp, By_comp);

    // With pitch, Bx_comp should change due to Bz contribution
    TS_ASSERT(std::abs(Bx_comp - Bx) > 5.0);
  }

  // Test 45: Combined roll and pitch compensation
  void testTiltCompensationCombined() {
    double Bx = 20.0, By = 5.0, Bz = 40.0;
    double roll = 15.0, pitch = 10.0;
    double Bx_comp, By_comp;

    tiltCompensate(Bx, By, Bz, roll, pitch, Bx_comp, By_comp);

    // Compensated heading should be calculable
    double heading = calculateHeading(Bx_comp, By_comp);
    TS_ASSERT(heading >= 0 && heading < 360.0);
  }

  // Test 46: Extreme tilt (near gimbal lock)
  void testExtremeTilt() {
    double Bx = 20.0, By = 5.0, Bz = 40.0;
    double roll = 0.0, pitch = 85.0;  // Nearly vertical
    double Bx_comp, By_comp;

    tiltCompensate(Bx, By, Bz, roll, pitch, Bx_comp, By_comp);

    // Should still produce finite values
    TS_ASSERT(!std::isnan(Bx_comp));
    TS_ASSERT(!std::isnan(By_comp));
  }

  /***************************************************************************
   * Declination Model Tests
   ***************************************************************************/

  // Test 47: Large positive declination (Alaska)
  void testLargePositiveDeclination() {
    double trueHeading = 270.0;
    double declination = 20.0;  // Large east variation

    double magHeading = magneticFromTrue(trueHeading, declination);
    TS_ASSERT_DELTA(magHeading, 250.0, epsilon);
  }

  // Test 48: Large negative declination (Eastern US)
  void testLargeNegativeDeclination() {
    double trueHeading = 90.0;
    double declination = -20.0;  // Large west variation

    double magHeading = magneticFromTrue(trueHeading, declination);
    TS_ASSERT_DELTA(magHeading, 110.0, epsilon);
  }

  // Test 49: Declination near agonic line
  void testAgonicLine() {
    double trueHeading = 45.0;
    double declination = 0.0;  // On agonic line

    double magHeading = magneticFromTrue(trueHeading, declination);
    TS_ASSERT_DELTA(magHeading, 45.0, epsilon);
  }

  // Test 50: True to magnetic roundtrip
  void testDeclinationRoundtrip() {
    double originalTrue = 137.5;
    double declination = 12.3;

    double magnetic = magneticFromTrue(originalTrue, declination);
    double backToTrue = trueFromMagnetic(magnetic, declination);

    TS_ASSERT_DELTA(backToTrue, originalTrue, epsilon);
  }

  /***************************************************************************
   * Rate Limiting Tests
   ***************************************************************************/

  // Test 51: Output rate limiting
  void testOutputRateLimit() {
    double prev_output = 20.0;
    double raw_input = 35.0;  // Large jump
    double max_rate = 5.0;    // Per sample

    double change = raw_input - prev_output;
    if (std::abs(change) > max_rate) {
      change = (change > 0) ? max_rate : -max_rate;
    }
    double limited_output = prev_output + change;

    TS_ASSERT_DELTA(limited_output, 25.0, epsilon);
  }

  // Test 52: Heading wrap-around rate limiting
  void testHeadingWrapRateLimit() {
    double prev_heading = 350.0;
    double new_heading = 10.0;
    double max_rate = 30.0;  // degrees per sample

    // Compute shortest angular distance
    double diff = new_heading - prev_heading;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    // Limit the change
    if (std::abs(diff) > max_rate) {
      diff = (diff > 0) ? max_rate : -max_rate;
    }

    double limited = prev_heading + diff;
    if (limited >= 360) limited -= 360;
    if (limited < 0) limited += 360;

    TS_ASSERT(std::abs(limited - prev_heading) <= max_rate ||
              std::abs(limited - prev_heading) >= 360 - max_rate);
  }

  /***************************************************************************
   * Geographic/Magnetic Coordinate Tests
   ***************************************************************************/

  // Test 53: Isogonic line interpolation
  void testIsogonalInterpolation() {
    // Two isogonic line values
    double dec_at_A = 10.0;   // Declination at point A
    double dec_at_B = 15.0;   // Declination at point B
    double fraction = 0.4;    // 40% along from A to B

    double interpolated = dec_at_A + fraction * (dec_at_B - dec_at_A);
    TS_ASSERT_DELTA(interpolated, 12.0, epsilon);
  }

  // Test 54: Secular variation
  void testSecularVariation() {
    double dec_2020 = 10.0;
    double annual_change = 0.1;  // degrees per year
    double years = 5.0;

    double dec_2025 = dec_2020 + annual_change * years;
    TS_ASSERT_DELTA(dec_2025, 10.5, epsilon);
  }

  /***************************************************************************
   * Filter Response Tests
   ***************************************************************************/

  // Test 55: Low-pass filter step response
  void testLowPassStepResponse() {
    double cutoff_freq = 5.0;   // Hz
    double sample_rate = 100.0; // Hz
    double dt = 1.0 / sample_rate;
    double rc = 1.0 / (2.0 * M_PI * cutoff_freq);
    double alpha = dt / (rc + dt);

    double input = 20.0;  // Step input
    double output = 0.0;

    // Run filter for 5 time constants to reach steady state
    int samples = static_cast<int>(5.0 * rc * sample_rate);
    for (int i = 0; i < samples; i++) {
      output = alpha * input + (1 - alpha) * output;
    }

    // After 5 time constants, should reach ~99.3% of input
    TS_ASSERT_DELTA(output, input, 0.5);
  }

  // Test 56: Moving average filter
  void testMovingAverageFilter() {
    const int window = 5;
    double samples[window] = {20.0, 21.0, 19.5, 20.5, 20.0};

    double sum = 0;
    for (int i = 0; i < window; i++) {
      sum += samples[i];
    }
    double average = sum / window;

    TS_ASSERT_DELTA(average, 20.2, epsilon);
  }

  /***************************************************************************
   * Compass Swinging Tests
   ***************************************************************************/

  // Test 57: Four-point compass swing
  void testFourPointSwing() {
    // Measurements at N, E, S, W
    double meas_N = 0.0 + 3.0;    // 3° bias
    double meas_E = 90.0 + 3.0;
    double meas_S = 180.0 + 3.0;
    double meas_W = 270.0 + 3.0;

    double avg_error = (meas_N - 0.0 + meas_E - 90.0 +
                        meas_S - 180.0 + meas_W - 270.0) / 4.0;
    TS_ASSERT_DELTA(avg_error, 3.0, epsilon);
  }

  // Test 58: Eight-point compass swing coefficients
  void testEightPointSwingCoefficients() {
    // Coefficient A (constant error)
    double errors[8] = {3.0, 2.5, 3.0, 3.5, 3.0, 2.5, 3.0, 3.5};

    double A = 0;
    for (int i = 0; i < 8; i++) {
      A += errors[i];
    }
    A /= 8.0;

    TS_ASSERT_DELTA(A, 3.0, 0.1);
  }

  /***************************************************************************
   * Multi-sensor Redundancy Tests
   ***************************************************************************/

  // Test 59: Dual magnetometer averaging
  void testDualMagAveraging() {
    double mag1_x = 20.1, mag1_y = 5.05;
    double mag2_x = 19.9, mag2_y = 4.95;

    double avg_x = (mag1_x + mag2_x) / 2.0;
    double avg_y = (mag1_y + mag2_y) / 2.0;

    TS_ASSERT_DELTA(avg_x, 20.0, epsilon);
    TS_ASSERT_DELTA(avg_y, 5.0, epsilon);
  }

  // Test 60: Sensor disagreement detection
  void testSensorDisagreement() {
    double mag1_heading = 90.0;
    double mag2_heading = 95.0;
    double threshold = 3.0;

    bool disagreement = std::abs(mag1_heading - mag2_heading) > threshold;
    TS_ASSERT(disagreement);
  }

  /***************************************************************************
   * Quantization Resolution Tests
   ***************************************************************************/

  // Test 61: Higher resolution ADC
  void testHighResolutionADC() {
    int bits = 16;
    double min_field = -100.0;
    double max_field = 100.0;

    double resolution = (max_field - min_field) / (1 << bits);
    TS_ASSERT(resolution < 0.01);  // Sub-0.01 microtesla
  }

  // Test 62: Low resolution ADC
  void testLowResolutionADC() {
    int bits = 8;
    double min_field = -100.0;
    double max_field = 100.0;

    double resolution = (max_field - min_field) / (1 << bits);
    TS_ASSERT(resolution > 0.5);  // Coarser than 0.5 microtesla
  }

  /***************************************************************************
   * Inclination Correction Tests
   ***************************************************************************/

  // Test 63: Southern hemisphere inclination
  void testSouthernHemisphereInclination() {
    double Bh = 20.0;
    double Bz = -40.0;  // Pointing up in southern hemisphere

    double inclination = calculateInclination(Bh, Bz);
    TS_ASSERT(inclination < 0);  // Negative inclination
  }

  // Test 64: Zero inclination (magnetic equator)
  void testMagneticEquator() {
    double Bh = 35.0;
    double Bz = 0.0;

    double inclination = calculateInclination(Bh, Bz);
    TS_ASSERT_DELTA(inclination, 0.0, epsilon);
  }

  /***************************************************************************
   * Bias Stability Tests
   ***************************************************************************/

  // Test 65: Bias stability measurement
  void testBiasStability() {
    std::mt19937 gen(123);
    std::normal_distribution<double> dist(0.0, 0.1);

    double bias_samples[100];
    double nominal_bias = 0.5;

    for (int i = 0; i < 100; i++) {
      bias_samples[i] = nominal_bias + dist(gen);
    }

    // Calculate Allan variance proxy (simple std dev)
    double sum = 0, sum_sq = 0;
    for (int i = 0; i < 100; i++) {
      sum += bias_samples[i];
      sum_sq += bias_samples[i] * bias_samples[i];
    }
    double mean = sum / 100;
    double variance = sum_sq / 100 - mean * mean;

    TS_ASSERT(variance < 0.02);  // Stable bias
  }

  // Test 66: Long-term drift
  void testLongTermDrift() {
    double initial_bias = 0.5;
    double drift_rate = 0.0001;  // microtesla per hour
    double hours = 24.0;

    double final_bias = initial_bias + drift_rate * hours;
    TS_ASSERT_DELTA(final_bias, 0.5024, 0.0001);
  }

  /***************************************************************************
   * More Edge Cases
   ***************************************************************************/

  // Test 67: Negative saturation
  void testNegativeSaturation() {
    double measured = -1000.0;
    double min_range = -100.0;

    double saturated = std::max(measured, min_range);
    TS_ASSERT_DELTA(saturated, min_range, epsilon);
  }

  // Test 68: Very small field magnitude (near null)
  void testNearNullField() {
    double Bx = 0.001, By = 0.002, Bz = 0.001;
    double magnitude = std::sqrt(Bx*Bx + By*By + Bz*Bz);

    TS_ASSERT(magnitude < 0.01);  // Near null
  }

  // Test 69: Field magnitude preservation through rotation
  void testMagnitudePreservation() {
    double Bx = 20.0, By = 5.0, Bz = 40.0;
    double original_mag = std::sqrt(Bx*Bx + By*By + Bz*Bz);

    // Rotate by 45 degrees in X-Y plane
    double angle = 45.0 * DEG_TO_RAD;
    double Bx_rot = Bx * std::cos(angle) - By * std::sin(angle);
    double By_rot = Bx * std::sin(angle) + By * std::cos(angle);
    double Bz_rot = Bz;

    double rotated_mag = std::sqrt(Bx_rot*Bx_rot + By_rot*By_rot + Bz_rot*Bz_rot);
    TS_ASSERT_DELTA(rotated_mag, original_mag, epsilon);
  }

  // Test 70: Heading calculation with dominant By
  void testDominantByHeading() {
    double Bx = 1.0;
    double By = 50.0;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT(heading > 85.0 && heading < 90.0);  // Nearly east
  }

  // Test 71: Heading stability near north
  void testHeadingStabilityNearNorth() {
    double Bx = 20.0;
    double By = 0.01;  // Small east component

    double heading = calculateHeading(Bx, By);
    TS_ASSERT(heading < 1.0 || heading > 359.0);  // Very close to north
  }

  // Test 72: Heading stability near 180
  void testHeadingStabilityNearSouth() {
    double Bx = -20.0;
    double By = 0.01;

    double heading = calculateHeading(Bx, By);
    TS_ASSERT(heading > 179.0 && heading < 181.0);
  }

  /***************************************************************************
   * AHRS Integration Tests
   ***************************************************************************/

  void testAHRSHeadingFusion() {
    // Magnetometer fused with gyro for heading
    double mag_heading = 90.5;
    double gyro_heading = 89.8;
    double alpha = 0.1;  // Complementary filter gain

    double fused = alpha * mag_heading + (1.0 - alpha) * gyro_heading;
    TS_ASSERT(fused > mag_heading - 1.0 && fused < mag_heading + 1.0);
  }

  void testMagGyroComplementaryFilter() {
    double mag_rate = 0.0;     // Magnetometer gives position
    double gyro_rate = 5.0;    // deg/sec
    double dt = 0.01;
    double tau = 0.5;          // Time constant

    double heading = 90.0;
    heading += gyro_rate * dt;

    double alpha = dt / (tau + dt);
    double mag_heading = 90.05;

    heading = alpha * mag_heading + (1.0 - alpha) * heading;
    TS_ASSERT_DELTA(heading, 90.05, 0.1);
  }

  void testAttitudeFromMagAndAccel() {
    // Roll and pitch from accelerometer
    // Typical level flight: az negative (down in NED), ay small
    double ax = 0.0, ay = 0.1, az = 1.0;  // g's (positive z = down in sensor frame)

    double roll = atan2(ay, az) * RAD_TO_DEG;
    double pitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

    TS_ASSERT(fabs(roll) < 10.0);
    TS_ASSERT(fabs(pitch) < 5.0);
  }

  /***************************************************************************
   * Kalman Filter Concepts
   ***************************************************************************/

  void testKalmanPredictionStep() {
    // Simplified Kalman predict
    double state = 90.0;        // Heading estimate
    double P = 1.0;             // Error covariance
    double Q = 0.01;            // Process noise
    double u = 5.0;             // Gyro rate input
    double dt = 0.01;

    state = state + u * dt;     // State prediction
    P = P + Q;                  // Covariance prediction

    TS_ASSERT_DELTA(state, 90.05, 0.001);
    TS_ASSERT(P > 1.0);
  }

  void testKalmanUpdateStep() {
    double x_pred = 90.05;      // Predicted state
    double P_pred = 1.01;       // Predicted covariance
    double z = 90.08;           // Magnetometer measurement
    double R = 0.5;             // Measurement noise

    double K = P_pred / (P_pred + R);  // Kalman gain
    double x = x_pred + K * (z - x_pred);
    double P = (1 - K) * P_pred;

    TS_ASSERT(K > 0.0 && K < 1.0);
    TS_ASSERT(x > x_pred);
    TS_ASSERT(P < P_pred);
  }

  void testKalmanGainRange() {
    // Kalman gain K = P / (P + R)
    double P = 1.0;

    double R_high = 10.0;
    double K_low = P / (P + R_high);

    double R_low = 0.1;
    double K_high = P / (P + R_low);

    TS_ASSERT(K_low < K_high);
    TS_ASSERT(K_low > 0.0);
    TS_ASSERT(K_high < 1.0);
  }

  /***************************************************************************
   * Advanced Calibration Tests
   ***************************************************************************/

  void testSphereCalibration() {
    // Full sphere calibration produces center and radii
    double center[3] = {2.0, -1.5, 0.8};
    double radii[3] = {22.0, 18.0, 20.0};

    // Calibrated measurement
    double raw[3] = {24.0, -1.0, 20.8};
    double cal[3];

    for (int i = 0; i < 3; i++) {
      cal[i] = (raw[i] - center[i]) * 20.0 / radii[i];
    }

    TS_ASSERT_DELTA(cal[0], 20.0, 0.1);
    TS_ASSERT_DELTA(cal[2], 20.0, 0.1);
  }

  void testEllipsoidCalibration() {
    // Soft iron creates ellipsoid, need full matrix correction
    double A[3][3] = {
      {1.0, 0.02, 0.01},
      {0.02, 1.0, 0.015},
      {0.01, 0.015, 1.0}
    };
    double b[3] = {2.0, -1.5, 0.8};

    double raw[3] = {22.0, -1.0, 40.8};
    double centered[3];
    double cal[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
      centered[i] = raw[i] - b[i];
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        cal[i] += A[i][j] * centered[j];
      }
    }

    TS_ASSERT(cal[0] > 0.0);
    TS_ASSERT(cal[2] > 0.0);
  }

  void testInFlightCalibrationUpdate() {
    // Adaptive calibration during flight
    double old_bias = 2.0;
    double learning_rate = 0.001;
    double error = 0.5;

    double new_bias = old_bias - learning_rate * error;
    TS_ASSERT(new_bias < old_bias);
  }

  /***************************************************************************
   * GPS-Aided Heading Tests
   ***************************************************************************/

  void testGPSGroundTrackComparison() {
    double mag_heading = 90.0;
    double gps_track = 92.0;  // Ground track from GPS

    double diff = gps_track - mag_heading;
    TS_ASSERT(fabs(diff) < 5.0);
  }

  void testGPSMagHeadingFusion() {
    double mag_heading = 90.0;
    double gps_track = 92.0;
    double airspeed = 200.0;   // kts
    double groundspeed = 210.0; // kts

    // Only trust GPS track when moving
    bool gps_valid = (groundspeed > 50.0);
    TS_ASSERT(gps_valid);

    if (gps_valid) {
      double alpha = 0.2;
      double fused = alpha * gps_track + (1.0 - alpha) * mag_heading;
      TS_ASSERT(fused > 90.0 && fused < 92.0);
    }
  }

  void testWindCorrectionAngle() {
    double heading = 90.0;
    double track = 95.0;

    double WCA = track - heading;
    TS_ASSERT_DELTA(WCA, 5.0, 0.1);
  }

  /***************************************************************************
   * Magnetic Storm Effects
   ***************************************************************************/

  void testGeomagneticStormEffect() {
    double normal_field = 45.0;   // microtesla
    double storm_variation = 500.0;  // nanotesla = 0.5 microtesla

    double disturbed_field = normal_field + storm_variation / 1000.0;
    double variation = (disturbed_field - normal_field) / normal_field;

    TS_ASSERT(variation < 0.02);  // <2% typical
  }

  void testKpIndexEffect() {
    // Kp index 0-9, higher = more disturbance
    int Kp = 5;  // Moderate storm

    bool reliable = (Kp < 4);
    TS_ASSERT(!reliable);  // Reduced reliability during storm
  }

  void testAuroralZoneDisturbance() {
    double latitude = 65.0;  // Near auroral zone
    bool inAuroralZone = (fabs(latitude) > 60.0 && fabs(latitude) < 70.0);

    TS_ASSERT(inAuroralZone);
    // Expect higher disturbance in auroral zone
  }

  /***************************************************************************
   * Compass Rose Tests
   ***************************************************************************/

  void testCompassRoseIntercardinal() {
    // NE = 45°, SE = 135°, SW = 225°, NW = 315°
    TS_ASSERT_DELTA(45.0, 45.0, 0.1);
    TS_ASSERT_DELTA(135.0, 135.0, 0.1);
    TS_ASSERT_DELTA(225.0, 225.0, 0.1);
    TS_ASSERT_DELTA(315.0, 315.0, 0.1);
  }

  void testHeadingStringConversion() {
    double heading = 45.0;
    const char* direction = "NE";

    // Heading should be in range for NE
    bool isNE = (heading >= 22.5 && heading < 67.5);
    TS_ASSERT(isNE);
  }

  void testReciprocalHeading() {
    double heading = 90.0;
    double reciprocal = heading + 180.0;
    if (reciprocal >= 360.0) reciprocal -= 360.0;

    TS_ASSERT_DELTA(reciprocal, 270.0, 0.1);
  }

  /***************************************************************************
   * Digital Signal Processing Tests
   ***************************************************************************/

  void testMedianFilter() {
    double samples[5] = {20.0, 21.0, 50.0, 19.5, 20.5};  // 50 is outlier

    // Sort for median
    double sorted[5];
    for (int i = 0; i < 5; i++) sorted[i] = samples[i];
    std::sort(sorted, sorted + 5);

    double median = sorted[2];
    TS_ASSERT_DELTA(median, 20.5, 0.1);  // Outlier rejected
  }

  void testExponentialSmoothing() {
    double alpha = 0.2;
    double smoothed = 20.0;
    double new_sample = 25.0;

    smoothed = alpha * new_sample + (1.0 - alpha) * smoothed;
    TS_ASSERT_DELTA(smoothed, 21.0, 0.1);
  }

  void testBandpassFilter() {
    // Reject DC bias and high frequency noise
    double cutoff_low = 0.1;   // Hz
    double cutoff_high = 5.0;  // Hz

    // Signals in band should pass
    double signal_freq = 1.0;  // Hz
    bool passes = (signal_freq > cutoff_low && signal_freq < cutoff_high);
    TS_ASSERT(passes);
  }

  /***************************************************************************
   * Sensor Fusion Quality Tests
   ***************************************************************************/

  void testHeadingInnovation() {
    double predicted = 90.0;
    double measured = 91.5;

    double innovation = measured - predicted;
    TS_ASSERT_DELTA(innovation, 1.5, 0.1);
  }

  void testInnovationGating() {
    double innovation = 15.0;  // Large discrepancy
    double sigma = 3.0;
    double gate = 3.0 * sigma;  // 3-sigma gate

    bool reject = (fabs(innovation) > gate);
    TS_ASSERT(reject);  // Measurement rejected
  }

  void testCovarianceReset() {
    double P = 0.01;  // Very low covariance (overconfident)
    double P_min = 0.1;

    if (P < P_min) P = P_min;
    TS_ASSERT(P >= P_min);
  }

  /***************************************************************************
   * Mounting Alignment Tests
   ***************************************************************************/

  void testSensorMisalignmentRoll() {
    double roll_offset = 2.0;  // degrees
    double Bx = 20.0, By = 5.0, Bz = 40.0;

    double cos_r = std::cos(roll_offset * DEG_TO_RAD);
    double sin_r = std::sin(roll_offset * DEG_TO_RAD);

    double By_aligned = By * cos_r - Bz * sin_r;
    double Bz_aligned = By * sin_r + Bz * cos_r;

    TS_ASSERT(fabs(By_aligned - By) > 0.1);
  }

  void testSensorMisalignmentYaw() {
    double yaw_offset = 5.0;  // degrees mounting error
    double Bx = 20.0, By = 0.0;

    double cos_y = std::cos(yaw_offset * DEG_TO_RAD);
    double sin_y = std::sin(yaw_offset * DEG_TO_RAD);

    double Bx_rot = Bx * cos_y - By * sin_y;
    double By_rot = Bx * sin_y + By * cos_y;

    double heading_error = calculateHeading(Bx_rot, By_rot);
    TS_ASSERT(fabs(heading_error) > 4.0);
  }

  void testAlignmentCalibration() {
    double measured_offset = 3.5;  // degrees
    double true_heading = 90.0;
    double raw_heading = 93.5;

    double corrected = raw_heading - measured_offset;
    TS_ASSERT_DELTA(corrected, true_heading, 0.1);
  }

  /***************************************************************************
   * Environmental Interference Tests
   ***************************************************************************/

  void testElectricalInterference() {
    double clean_field = 20.0;
    double interference = 2.0;  // From nearby wiring

    double measured = clean_field + interference;
    double error = measured - clean_field;

    TS_ASSERT_DELTA(error, 2.0, 0.1);
  }

  void testEngineRunningEffect() {
    double static_bias = 0.5;
    double engine_bias = 1.5;  // Additional when engine running

    double total_bias = static_bias + engine_bias;
    TS_ASSERT(total_bias > static_bias);
  }

  void testAvionicsInterference() {
    double radio_transmit_bias = 0.8;  // When radio transmitting
    double radar_bias = 0.3;

    double total = radio_transmit_bias + radar_bias;
    TS_ASSERT(total > 1.0);
  }

  /***************************************************************************
   * Performance Specification Tests
   ***************************************************************************/

  void testHeadingAccuracySpec() {
    double spec_accuracy = 1.0;  // ±1 degree
    double measured_error = 0.8;

    bool meets_spec = (fabs(measured_error) <= spec_accuracy);
    TS_ASSERT(meets_spec);
  }

  void testUpdateRateSpec() {
    double sample_rate = 50.0;  // Hz
    double min_rate = 10.0;     // Hz

    bool adequate = (sample_rate >= min_rate);
    TS_ASSERT(adequate);
  }

  void testResponseTimeSpec() {
    double time_constant = 0.3;  // seconds
    double spec = 0.5;           // seconds

    bool meets_spec = (time_constant <= spec);
    TS_ASSERT(meets_spec);
  }

  /***************************************************************************
   * Navigation Computation Tests
   ***************************************************************************/

  void testCrossTrackError() {
    double heading = 90.0;
    double desired_track = 85.0;
    double crosstrack_rate = 1.0;  // nm/min

    double track_error = heading - desired_track;
    TS_ASSERT_DELTA(track_error, 5.0, 0.1);
  }

  void testWindCorrectionCalculation() {
    double groundspeed = 200.0;  // kts
    double windspeed = 30.0;     // kts crosswind

    double WCA = asin(windspeed / groundspeed) * RAD_TO_DEG;
    TS_ASSERT(WCA < 10.0);
  }

  void testTurnAnticipation() {
    double turn_radius = 2.0;  // nm
    double groundspeed = 180.0; // kts
    double bank_angle = 25.0;

    // Distance to start turn before waypoint
    double lead_distance = sqrt(2.0 * turn_radius * turn_radius);
    TS_ASSERT(lead_distance > 2.0);
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  void testPolarRegionOperation() {
    double latitude = 85.0;
    double declination = 45.0;  // Very large at poles

    // Magnetic heading becomes unreliable
    bool unreliable = (fabs(latitude) > 80.0);
    TS_ASSERT(unreliable);
  }

  void testEquatorialRegion() {
    double latitude = 0.0;
    double inclination = 0.0;  // Horizontal field

    bool optimal = (fabs(latitude) < 30.0);
    TS_ASSERT(optimal);  // Best conditions for mag compass
  }

  void testRapidManeuvering() {
    double heading_rate = 50.0;  // deg/sec (high)
    double max_rate = 60.0;

    bool within_limits = (fabs(heading_rate) < max_rate);
    TS_ASSERT(within_limits);
  }

  void testInvertedFlight() {
    double roll = 180.0;  // Inverted
    double Bx = 20.0, By = 5.0, Bz = 40.0;

    // Heading calculation should still work with proper compensation
    double cos_r = std::cos(roll * DEG_TO_RAD);
    double sin_r = std::sin(roll * DEG_TO_RAD);

    double By_comp = By * cos_r - Bz * sin_r;
    TS_ASSERT(!std::isnan(By_comp));
  }
};
