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
};
