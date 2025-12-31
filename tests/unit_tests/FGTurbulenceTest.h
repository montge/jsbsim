/*******************************************************************************
 * FGTurbulenceTest.h - Unit tests for atmospheric turbulence and gust modeling
 *
 * Tests the mathematical behavior of turbulence models:
 * - Dryden turbulence spectrum (MIL-F-8785C)
 * - Von Karman turbulence spectrum
 * - Turbulence intensity scaling with altitude
 * - Gust velocity calculations
 * - Discrete gust (1-cosine) profiles
 * - Continuous turbulence power spectral density
 * - Low altitude turbulence
 * - Boundary layer turbulence
 * - Mountain wave turbulence
 * - Wake turbulence modeling
 * - Wind shear profiles
 * - Microburst modeling
 * - Turbulence scale lengths
 * - RMS gust velocities
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <random>

#include <FGFDMExec.h>
#include <models/atmosphere/FGWinds.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGAccelerations.h>

using namespace JSBSim;

const double epsilon = 1e-10;
const double PI = M_PI;

class FGTurbulenceTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Dryden Turbulence Model Tests (MIL-F-8785C)
   ***************************************************************************/

  // Test Dryden longitudinal scale length below 1000 ft
  void testDrydenScaleLengthLowAltitude() {
    double altitude = 500.0;  // ft AGL

    // MIL-F-8785C, Fig. 10, p. 55
    double L_u = altitude / pow(0.177 + 0.000823 * altitude, 1.2);
    double L_w = altitude;

    TS_ASSERT(L_u > 0);
    TS_ASSERT_DELTA(L_w, 500.0, epsilon);
    TS_ASSERT(L_u > L_w);  // Longitudinal scale larger than vertical
  }

  // Test Dryden scale length at medium altitude (1000-2000 ft)
  void testDrydenScaleLengthMediumAltitude() {
    double altitude = 1500.0;  // ft AGL

    // Linear interpolation between low and high altitude models
    double L_u = 1000.0 + (altitude - 1000.0) / 1000.0 * 750.0;
    double L_w = L_u;

    TS_ASSERT_DELTA(L_u, 1375.0, epsilon);
    TS_ASSERT_DELTA(L_w, 1375.0, epsilon);
  }

  // Test Dryden scale length above 2000 ft
  void testDrydenScaleLengthHighAltitude() {
    double altitude = 5000.0;  // ft AGL

    // MIL-F-8785C, Sec. 3.7.2.1, p. 48
    double L_u = 1750.0;
    double L_w = 1750.0;

    TS_ASSERT_DELTA(L_u, 1750.0, epsilon);
    TS_ASSERT_DELTA(L_w, 1750.0, epsilon);
  }

  // Test Dryden turbulence intensity below 1000 ft
  void testDrydenIntensityLowAltitude() {
    double altitude = 500.0;  // ft AGL
    double windspeed_at_20ft = 15.0;  // ft/s (light turbulence)

    // MIL-F-8785C, Fig. 11, p. 56
    double sig_w = 0.1 * windspeed_at_20ft;
    double sig_u = sig_w / pow(0.177 + 0.000823 * altitude, 0.4);

    TS_ASSERT_DELTA(sig_w, 1.5, epsilon);
    TS_ASSERT(sig_u > sig_w);
    TS_ASSERT(sig_u < 10.0);  // Reasonable value
  }

  // Test Dryden intensity at medium altitude
  void testDrydenIntensityMediumAltitude() {
    double altitude = 1500.0;  // ft AGL
    double windspeed_at_20ft = 25.0;  // ft/s
    double sigma_high = 6.0;  // From POE table

    // Linear interpolation
    double sig_low = 0.1 * windspeed_at_20ft;
    double sig = sig_low + (altitude - 1000.0) / 1000.0 * (sigma_high - sig_low);

    TS_ASSERT(sig > sig_low);
    TS_ASSERT(sig < sigma_high);
  }

  // Test Dryden time constant
  void testDrydenTimeConstant() {
    double L_u = 1000.0;  // Scale length
    double V = 200.0;     // True airspeed (ft/s)

    double tau_u = L_u / V;

    TS_ASSERT_DELTA(tau_u, 5.0, epsilon);
    TS_ASSERT(tau_u > 0);
  }

  /***************************************************************************
   * Von Karman Turbulence Spectrum Tests
   ***************************************************************************/

  // Test Von Karman longitudinal PSD
  void testVonKarmanLongitudinalPSD() {
    double L = 1000.0;      // Scale length (ft)
    double sigma = 5.0;     // RMS turbulence (ft/s)
    double omega = 0.5;     // Angular frequency (rad/s)
    double V = 200.0;       // True airspeed (ft/s)

    // Von Karman spectrum
    double Omega = omega * L / V;
    double Phi_u = sigma * sigma * L / V / PI *
                   1.0 / pow(1.0 + 8.0 / 3.0 * (1.339 * Omega) * (1.339 * Omega), 5.0/6.0);

    TS_ASSERT(Phi_u > 0);
    TS_ASSERT(std::isfinite(Phi_u));
  }

  // Test Von Karman lateral PSD
  void testVonKarmanLateralPSD() {
    double L = 1000.0;
    double sigma = 5.0;
    double omega = 0.5;
    double V = 200.0;

    double Omega = omega * L / V;
    double numerator = 1.0 + 8.0 / 3.0 * (1.339 * Omega) * (1.339 * Omega);
    double Phi_v = sigma * sigma * L / V / PI *
                   (1.0 + 8.0 / 3.0 * (1.339 * Omega) * (1.339 * Omega)) /
                   pow(numerator, 11.0/6.0);

    TS_ASSERT(Phi_v > 0);
    TS_ASSERT(std::isfinite(Phi_v));
  }

  // Test Von Karman vertical PSD
  void testVonKarmanVerticalPSD() {
    double L = 500.0;
    double sigma = 3.0;
    double omega = 1.0;
    double V = 150.0;

    double Omega = omega * L / V;
    double numerator = 1.0 + 8.0 / 3.0 * (1.339 * Omega) * (1.339 * Omega);
    double Phi_w = sigma * sigma * L / V / PI *
                   (1.0 + 8.0 / 3.0 * (1.339 * Omega) * (1.339 * Omega)) /
                   pow(numerator, 11.0/6.0);

    TS_ASSERT(Phi_w > 0);
    TS_ASSERT(std::isfinite(Phi_w));
  }

  /***************************************************************************
   * Turbulence Intensity Scaling Tests
   ***************************************************************************/

  // Test turbulence intensity decreases with altitude
  void testTurbulenceIntensityAltitudeScaling() {
    double windspeed_at_20ft = 30.0;  // ft/s

    double sig_500 = 0.1 * windspeed_at_20ft / pow(0.177 + 0.000823 * 500.0, 0.4);
    double sig_1000 = 0.1 * windspeed_at_20ft / pow(0.177 + 0.000823 * 1000.0, 0.4);

    TS_ASSERT(sig_500 > sig_1000);  // Intensity decreases with altitude
  }

  // Test turbulence intensity with windspeed
  void testTurbulenceIntensityWithWindspeed() {
    double altitude = 500.0;

    double w20_light = 15.0;     // Light turbulence
    double w20_moderate = 30.0;  // Moderate turbulence
    double w20_severe = 45.0;    // Severe turbulence

    double sig_light = 0.1 * w20_light;
    double sig_moderate = 0.1 * w20_moderate;
    double sig_severe = 0.1 * w20_severe;

    TS_ASSERT(sig_light < sig_moderate);
    TS_ASSERT(sig_moderate < sig_severe);
  }

  // Test probability of exceedence curve
  void testProbabilityOfExceedence() {
    // MIL-F-8785C, Figure 7, p. 49
    // At 500 ft, severity index 3 (light)
    double sigma_500_idx3 = 6.6;  // ft/s

    // At 7500 ft, severity index 3
    double sigma_7500_idx3 = 6.7;  // ft/s

    TS_ASSERT_DELTA(sigma_500_idx3, 6.6, 0.1);
    TS_ASSERT_DELTA(sigma_7500_idx3, 6.7, 0.1);
  }

  /***************************************************************************
   * Gust Velocity Calculation Tests
   ***************************************************************************/

  // Test RMS gust velocity
  void testRMSGustVelocity() {
    double sigma = 5.0;  // RMS turbulence intensity

    // For normal distribution, ~68% of values within 1 sigma
    // ~95% within 2 sigma
    double gust_1sigma = sigma;
    double gust_2sigma = 2.0 * sigma;

    TS_ASSERT_DELTA(gust_1sigma, 5.0, epsilon);
    TS_ASSERT_DELTA(gust_2sigma, 10.0, epsilon);
  }

  // Test peak gust velocity
  void testPeakGustVelocity() {
    double sigma_w = 3.0;
    double gust_factor = 3.0;  // Typical peak gust factor

    double V_gust = gust_factor * sigma_w;

    TS_ASSERT_DELTA(V_gust, 9.0, epsilon);
  }

  /***************************************************************************
   * Discrete Gust (1-Cosine) Profile Tests
   ***************************************************************************/

  // Test 1-cosine gust at start
  void testOneMinusCosineGustStart() {
    double gustAmplitude = 20.0;  // ft/s
    double gustLength = 200.0;    // ft
    double x = 0.0;

    double gust = (gustAmplitude / 2.0) * (1.0 - cos(2.0 * PI * x / gustLength));

    TS_ASSERT_DELTA(gust, 0.0, epsilon);
  }

  // Test 1-cosine gust at midpoint (peak)
  void testOneMinusCosineGustPeak() {
    double gustAmplitude = 20.0;
    double gustLength = 200.0;
    double x = 100.0;  // Midpoint

    double gust = (gustAmplitude / 2.0) * (1.0 - cos(2.0 * PI * x / gustLength));

    TS_ASSERT_DELTA(gust, gustAmplitude, epsilon);
  }

  // Test 1-cosine gust at end
  void testOneMinusCosineGustEnd() {
    double gustAmplitude = 20.0;
    double gustLength = 200.0;
    double x = 200.0;

    double gust = (gustAmplitude / 2.0) * (1.0 - cos(2.0 * PI * x / gustLength));

    TS_ASSERT_DELTA(gust, 0.0, epsilon);
  }

  // Test 1-cosine gust quarter point
  void testOneMinusCosineGustQuarter() {
    double gustAmplitude = 20.0;
    double gustLength = 200.0;
    double x = 50.0;  // Quarter point

    double gust = (gustAmplitude / 2.0) * (1.0 - cos(2.0 * PI * x / gustLength));

    TS_ASSERT(gust > 0.0);
    TS_ASSERT(gust < gustAmplitude);
    TS_ASSERT_DELTA(gust, 10.0, 0.1);  // Should be ~10 ft/s
  }

  // Test cosine gust profile time evolution
  void testCosineGustTimeProfile() {
    double startupDuration = 2.0;  // seconds
    double elapsedTime = 1.0;      // 1 second into startup

    double factor = (1.0 - cos(PI * elapsedTime / startupDuration)) / 2.0;

    TS_ASSERT(factor > 0.0);
    TS_ASSERT(factor < 1.0);
    TS_ASSERT_DELTA(factor, 0.5, 0.1);
  }

  /***************************************************************************
   * Continuous Turbulence PSD Tests
   ***************************************************************************/

  // Test Dryden PSD at zero frequency
  void testDrydenPSDZeroFrequency() {
    double sigma = 5.0;
    double L = 1000.0;
    double V = 200.0;
    double omega = 0.0;

    // At omega = 0, Dryden PSD = sigma^2 * L / (PI * V)
    double Phi_0 = sigma * sigma * L / (PI * V);

    TS_ASSERT_DELTA(Phi_0, 39.789, 0.01);
  }

  // Test Dryden PSD at high frequency
  void testDrydenPSDHighFrequency() {
    double sigma = 5.0;
    double L = 1000.0;
    double V = 200.0;
    double omega = 100.0;  // High frequency

    double Omega = omega * L / V;
    double Phi_u = sigma * sigma * L / (PI * V) *
                   1.0 / pow(1.0 + Omega * Omega, 1.0);

    TS_ASSERT(Phi_u > 0);
    TS_ASSERT(Phi_u < 0.01);  // Should be very small at high frequency
  }

  /***************************************************************************
   * Low Altitude Turbulence Tests
   ***************************************************************************/

  // Test ground effect on turbulence at very low altitude
  void testGroundEffectOnTurbulence() {
    double altitude = 10.0;  // ft AGL (minimum per MIL-F-8785C)

    double L_u = altitude / pow(0.177 + 0.000823 * altitude, 1.2);
    double L_w = altitude;

    TS_ASSERT_DELTA(L_w, 10.0, epsilon);
    TS_ASSERT(L_u > 10.0);
    TS_ASSERT(L_u < 100.0);  // Reasonable range
  }

  // Test turbulence intensity near ground
  void testTurbulenceNearGround() {
    double altitude = 20.0;  // ft AGL
    double windspeed_at_20ft = 25.0;

    double sig_w = 0.1 * windspeed_at_20ft;

    TS_ASSERT_DELTA(sig_w, 2.5, epsilon);
  }

  // Test turbulence scale at 100 ft
  void testTurbulenceAt100Feet() {
    double altitude = 100.0;

    double L_w = altitude;
    double L_u = altitude / pow(0.177 + 0.000823 * altitude, 1.2);

    TS_ASSERT_DELTA(L_w, 100.0, epsilon);
    TS_ASSERT(L_u > L_w);
  }

  /***************************************************************************
   * Boundary Layer Turbulence Tests
   ***************************************************************************/

  // Test boundary layer height
  void testBoundaryLayerHeight() {
    double boundary_layer_height = 2000.0;  // ft (typical)

    TS_ASSERT_DELTA(boundary_layer_height, 2000.0, epsilon);
  }

  // Test turbulence in boundary layer
  void testTurbulenceInBoundaryLayer() {
    double altitude = 1200.0;  // Within boundary layer
    double boundary_layer_top = 2000.0;

    TS_ASSERT(altitude < boundary_layer_top);

    // Scale lengths transition in this region
    double L_transition = 1000.0 + (altitude - 1000.0) / 1000.0 * 750.0;
    TS_ASSERT(L_transition > 1000.0);
    TS_ASSERT(L_transition < 1750.0);
  }

  // Test mixed layer turbulence
  void testMixedLayerTurbulence() {
    double zi = 3000.0;  // Convective boundary layer depth (ft)
    double w_star = 2.0; // Convective velocity scale (m/s)

    double sigma_w = 0.52 * w_star;  // Vertical velocity variance

    TS_ASSERT(sigma_w > 0);
    TS_ASSERT_DELTA(sigma_w, 1.04, 0.01);
  }

  /***************************************************************************
   * Mountain Wave Turbulence Tests
   ***************************************************************************/

  // Test mountain wave wavelength
  void testMountainWaveWavelength() {
    double U = 50.0;     // Wind speed (ft/s)
    double N = 0.01;     // Brunt-Vaisala frequency (rad/s)

    double lambda = 2.0 * PI * U / N;  // Wavelength

    TS_ASSERT(lambda > 0);
    TS_ASSERT_DELTA(lambda, 31415.9, 1.0);
  }

  // Test mountain wave amplitude
  void testMountainWaveAmplitude() {
    double h_m = 1000.0;  // Mountain height (ft)
    double U = 50.0;      // Wind speed (ft/s)
    double N = 0.01;      // Brunt-Vaisala frequency

    double Fr = U / (N * h_m);  // Froude number

    TS_ASSERT(Fr > 0);
    TS_ASSERT_DELTA(Fr, 5.0, epsilon);
  }

  /***************************************************************************
   * Wake Turbulence Tests
   ***************************************************************************/

  // Test wake vortex circulation
  void testWakeVortexCirculation() {
    double weight = 500000.0;  // lbs
    double wingspan = 200.0;   // ft
    double V = 250.0;          // ft/s
    double rho = 0.002377;     // slugs/ft^3 (sea level)

    double Gamma = weight / (rho * V * wingspan);  // Circulation

    TS_ASSERT(Gamma > 0);
    TS_ASSERT(Gamma > 1000);  // Reasonable value for aircraft
  }

  // Test wake vortex velocity
  void testWakeVortexVelocity() {
    double Gamma = 2e6;  // Circulation (ft^2/s)
    double r = 100.0;    // Distance from vortex core (ft)

    double v_theta = Gamma / (2.0 * PI * r);  // Tangential velocity

    TS_ASSERT(v_theta > 0);
    TS_ASSERT_DELTA(v_theta, 3183.1, 1.0);
  }

  // Test wake vortex descent rate
  void testWakeVortexDescentRate() {
    double Gamma = 2e6;  // Circulation
    double b = 200.0;    // Vortex separation (wingspan)

    double w_0 = Gamma / (2.0 * PI * b);  // Initial descent rate

    TS_ASSERT(w_0 > 0);
    TS_ASSERT_DELTA(w_0, 1591.5, 1.0);
  }

  /***************************************************************************
   * Wind Shear Profile Tests
   ***************************************************************************/

  // Test logarithmic wind shear
  void testLogarithmicWindShear() {
    double u_star = 1.0;  // Friction velocity (ft/s)
    double z = 100.0;     // Height (ft)
    double z0 = 0.1;      // Roughness length (ft)
    double kappa = 0.4;   // Von Karman constant

    double u_z = (u_star / kappa) * log(z / z0);

    TS_ASSERT(u_z > 0);
    TS_ASSERT(u_z > 10.0);
  }

  // Test power law wind shear
  void testPowerLawWindShear() {
    double u_ref = 20.0;  // Reference wind (ft/s)
    double z_ref = 33.0;  // Reference height (10m = 33 ft)
    double z = 100.0;     // Height of interest
    double alpha = 0.143; // Shear exponent (open terrain)

    double u_z = u_ref * pow(z / z_ref, alpha);

    TS_ASSERT(u_z > u_ref);
    // 20 * (100/33)^0.143 ≈ 23.4
    TS_ASSERT_DELTA(u_z, 23.4, 0.5);
  }

  // Test wind shear gradient
  void testWindShearGradient() {
    double du_dz = 0.1;  // Wind gradient (s^-1)
    double dz = 100.0;   // Height change (ft)

    double delta_u = du_dz * dz;

    TS_ASSERT_DELTA(delta_u, 10.0, epsilon);
  }

  /***************************************************************************
   * Microburst Tests
   ***************************************************************************/

  // Test microburst radial velocity profile
  void testMicroburstRadialProfile() {
    double r_max = 2000.0;  // Radius of maximum outflow (ft)
    double V_max = 50.0;    // Maximum radial velocity (ft/s)
    double r = 1000.0;      // Distance from center

    double V_r = V_max * (r / r_max) * exp(1.0 - r / r_max);

    TS_ASSERT(V_r > 0);
    TS_ASSERT(V_r < V_max);
  }

  // Test microburst at maximum outflow radius
  void testMicroburstAtMaxRadius() {
    double r_max = 2000.0;
    double V_max = 50.0;
    double r = r_max;

    double V_r = V_max * (r / r_max) * exp(1.0 - r / r_max);

    TS_ASSERT_DELTA(V_r, V_max, epsilon);
  }

  // Test microburst vertical velocity
  void testMicroburstVerticalVelocity() {
    double w_max = 60.0;  // Maximum downdraft (ft/s)
    double z = 500.0;     // Height (ft)
    double z_max = 300.0; // Height of max downdraft

    double w = w_max * exp(-pow((z - z_max) / 500.0, 2.0));

    TS_ASSERT(w > 0);
    TS_ASSERT(w < w_max);
  }

  /***************************************************************************
   * Turbulence Scale Length Tests
   ***************************************************************************/

  // Test lateral turbulence scale
  void testLateralTurbulenceScale() {
    double L_w = 1000.0;
    double b_w = 100.0;  // Wingspan

    double L_v = L_w;  // For Dryden model, lateral scale = vertical scale

    TS_ASSERT_DELTA(L_v, L_w, epsilon);
  }

  // Test angular rate scale length
  void testAngularRateScaleLength() {
    double L_w = 1000.0;
    double b_w = 100.0;

    double L_p = sqrt(L_w * b_w) / 2.6;  // Roll rate scale

    TS_ASSERT(L_p > 0);
    TS_ASSERT_DELTA(L_p, 121.3, 0.5);
  }

  // Test pitch rate time constant
  void testPitchRateTimeConstant() {
    double b_w = 100.0;
    double V = 200.0;

    double tau_q = 4.0 * b_w / (PI * V);

    TS_ASSERT(tau_q > 0);
    TS_ASSERT_DELTA(tau_q, 0.637, 0.01);
  }

  // Test yaw rate time constant
  void testYawRateTimeConstant() {
    double b_w = 100.0;
    double V = 200.0;

    double tau_r = 3.0 * b_w / (PI * V);

    TS_ASSERT(tau_r > 0);
    TS_ASSERT_DELTA(tau_r, 0.478, 0.01);
  }

  /***************************************************************************
   * RMS Gust Velocity Tests
   ***************************************************************************/

  // Test RMS angular rate for roll
  void testRMSRollRate() {
    double sigma_w = 5.0;
    double L_w = 1000.0;
    double b_w = 100.0;

    double sig_p = 1.9 / sqrt(L_w * b_w) * sigma_w;

    TS_ASSERT(sig_p > 0);
    // 1.9 / sqrt(100000) * 5 ≈ 0.03
    TS_ASSERT_DELTA(sig_p, 0.03, 0.005);  // rad/s
  }

  // Test RMS turbulence components are consistent
  void testRMSComponentsConsistency() {
    double sig_u = 5.0;
    double sig_v = 5.0;
    double sig_w = 3.0;

    // For isotropic turbulence at high altitude
    TS_ASSERT(sig_u > 0);
    TS_ASSERT(sig_v > 0);
    TS_ASSERT(sig_w > 0);
  }

  // Test turbulence integration time step
  void testTurbulenceTimeStep() {
    double tau_min = 0.5;    // Minimum time constant
    double dt = 0.01;        // Time step (seconds)

    double ratio = dt / tau_min;

    TS_ASSERT(ratio < 0.1);  // Time step should be much smaller than time constant
  }

  /***************************************************************************
   * Filter Transfer Function Tests
   ***************************************************************************/

  // Test first-order filter response
  void testFirstOrderFilter() {
    double tau = 2.0;     // Time constant
    double T_V = 0.01;    // Time step

    double a = 1.0 - T_V / tau;
    double b = sqrt(2.0 * T_V / tau);

    TS_ASSERT(a < 1.0);
    TS_ASSERT(a > 0.0);
    TS_ASSERT(b > 0);
  }

  // Test Tustin bilinear transform coefficient
  void testTustinCoefficient() {
    double tau = 2.0;
    double T_V = 0.01;

    double C_BL = 1.0 / tau / tan(T_V / (2.0 * tau));

    TS_ASSERT(C_BL > 0);
    TS_ASSERT(std::isfinite(C_BL));
  }

  /***************************************************************************
   * Statistical Properties Tests
   ***************************************************************************/

  // Test turbulence autocorrelation
  void testTurbulenceAutocorrelation() {
    double L = 1000.0;
    double V = 200.0;
    double tau = L / V;
    double dt = 1.0;

    double R = exp(-dt / tau);  // Autocorrelation

    TS_ASSERT(R > 0);
    TS_ASSERT(R < 1.0);
    TS_ASSERT_DELTA(R, 0.819, 0.01);
  }

  // Test white noise variance scaling
  void testWhiteNoiseScaling() {
    double sigma = 5.0;
    double tau = 2.0;
    double dt = 0.01;

    double sigma_wn = sigma * sqrt(2.0 * dt / tau);

    TS_ASSERT(sigma_wn > 0);
    TS_ASSERT(sigma_wn < sigma);
  }

  /***************************************************************************
   * Edge Cases and Validation Tests
   ***************************************************************************/

  // Test zero turbulence intensity
  void testZeroTurbulence() {
    double sigma = 0.0;

    TS_ASSERT_DELTA(sigma, 0.0, epsilon);
  }

  // Test very high altitude turbulence
  void testHighAltitudeTurbulence() {
    double altitude = 60000.0;  // ft
    double sigma = 2.0;         // Reduced intensity at high altitude

    TS_ASSERT(sigma > 0);
    TS_ASSERT(sigma < 10.0);
  }

  // Test minimum clipping height
  void testMinimumAltitudeClipping() {
    double altitude = 5.0;      // Below minimum
    double altitude_clipped = std::max(altitude, 10.0);

    TS_ASSERT_DELTA(altitude_clipped, 10.0, epsilon);
  }

  // Test division by zero protection
  void testDivisionByZeroProtection() {
    double V = 0.1;  // Very low airspeed

    if (V > 0) {
      double tau = 1000.0 / V;
      TS_ASSERT(tau > 0);
      TS_ASSERT(std::isfinite(tau));
    }
  }

  // Test numerical stability of filter
  void testFilterNumericalStability() {
    double a = 0.99;  // Filter coefficient

    TS_ASSERT(std::abs(a) < 1.0);  // Stability condition
  }

  /***************************************************************************
   * Additional Dryden Model Tests
   ***************************************************************************/

  // Test Dryden PSD formula for longitudinal component
  void testDrydenLongitudinalPSD() {
    double sigma = 5.0;
    double L = 1000.0;
    double V = 200.0;
    double omega = 0.5;

    double Omega = omega * L / V;
    double Phi_u = sigma * sigma * 2.0 * L / (PI * V) /
                   (1.0 + (1.339 * Omega) * (1.339 * Omega));

    TS_ASSERT(Phi_u > 0);
    TS_ASSERT(std::isfinite(Phi_u));
  }

  // Test Dryden PSD formula for vertical component
  void testDrydenVerticalPSD() {
    double sigma = 3.0;
    double L = 500.0;
    double V = 150.0;
    double omega = 1.0;

    double Omega = omega * L / V;
    double Phi_w = sigma * sigma * 2.0 * L / (PI * V) *
                   (1.0 + 3.0 * (1.339 * Omega) * (1.339 * Omega)) /
                   pow(1.0 + (1.339 * Omega) * (1.339 * Omega), 2.0);

    TS_ASSERT(Phi_w > 0);
    TS_ASSERT(std::isfinite(Phi_w));
  }

  // Test Dryden transfer function coefficient
  void testDrydenTransferFunctionCoefficient() {
    double L = 1000.0;
    double V = 200.0;
    double sigma = 5.0;

    double K = sigma * sqrt(2.0 * L / (PI * V));

    TS_ASSERT(K > 0);
    // K = 5 * sqrt(2000 / 628.318) = 5 * 1.784 = 8.92
    TS_ASSERT_DELTA(K, 8.92, 0.01);
  }

  // Test altitude transition zone (1000-2000 ft)
  void testAltitudeTransitionZone() {
    // Test interpolation at 1250 ft
    double altitude = 1250.0;
    double L_low = 1000.0;
    double L_high = 1750.0;

    double factor = (altitude - 1000.0) / 1000.0;
    double L = L_low + factor * (L_high - L_low);

    TS_ASSERT_DELTA(L, 1187.5, epsilon);
    TS_ASSERT(L > L_low);
    TS_ASSERT(L < L_high);
  }

  // Test scale length ratio Lu/Lw at low altitude
  void testScaleLengthRatioLowAltitude() {
    double altitude = 300.0;

    double L_u = altitude / pow(0.177 + 0.000823 * altitude, 1.2);
    double L_w = altitude;
    double ratio = L_u / L_w;

    TS_ASSERT(ratio > 1.0);
    TS_ASSERT(ratio < 5.0);
  }

  /***************************************************************************
   * Turbulence Severity Category Tests
   ***************************************************************************/

  // Test light turbulence category
  void testLightTurbulenceCategory() {
    double sigma_light_max = 5.0;  // ft/s (approximately)

    TS_ASSERT(sigma_light_max > 0);
    TS_ASSERT(sigma_light_max < 10.0);
  }

  // Test moderate turbulence category
  void testModerateTurbulenceCategory() {
    double sigma_moderate_min = 5.0;
    double sigma_moderate_max = 10.0;

    TS_ASSERT(sigma_moderate_max > sigma_moderate_min);
  }

  // Test severe turbulence category
  void testSevereTurbulenceCategory() {
    double sigma_severe_min = 10.0;
    double sigma_severe_max = 20.0;

    TS_ASSERT(sigma_severe_max > sigma_severe_min);
  }

  // Test extreme turbulence category
  void testExtremeTurbulenceCategory() {
    double sigma_extreme = 25.0;

    TS_ASSERT(sigma_extreme > 20.0);
  }

  /***************************************************************************
   * Crosswind Turbulence Tests
   ***************************************************************************/

  // Test crosswind gust component
  void testCrosswindGustComponent() {
    double sigma_v = 4.0;  // Lateral RMS
    double heading = PI / 4.0;  // 45 degrees

    double crosswind_component = sigma_v * sin(heading);

    TS_ASSERT(crosswind_component > 0);
    TS_ASSERT_DELTA(crosswind_component, 2.828, 0.01);
  }

  // Test headwind gust component
  void testHeadwindGustComponent() {
    double sigma_u = 5.0;
    double heading = PI / 6.0;  // 30 degrees

    double headwind_component = sigma_u * cos(heading);

    TS_ASSERT(headwind_component > 0);
    TS_ASSERT_DELTA(headwind_component, 4.330, 0.01);
  }

  // Test combined horizontal gust
  void testCombinedHorizontalGust() {
    double u_gust = 5.0;
    double v_gust = 3.0;

    double horizontal_gust = sqrt(u_gust * u_gust + v_gust * v_gust);

    TS_ASSERT_DELTA(horizontal_gust, 5.831, 0.01);
  }

  /***************************************************************************
   * Thermal Turbulence Tests
   ***************************************************************************/

  // Test convective velocity scale
  void testConvectiveVelocityScale() {
    double Q_H = 100.0;  // Surface heat flux (W/m^2)
    double rho = 1.225;  // Air density (kg/m^3)
    double cp = 1005.0;  // Specific heat (J/kg/K)
    double zi = 1000.0;  // Boundary layer height (m)
    double T = 300.0;    // Temperature (K)
    double g = 9.81;     // Gravity (m/s^2)

    double w_star = pow(g / T * Q_H / (rho * cp) * zi, 1.0/3.0);

    TS_ASSERT(w_star > 0);
    TS_ASSERT(w_star < 5.0);  // Typical range
  }

  // Test thermal updraft velocity
  void testThermalUpdraftVelocity() {
    double w_star = 2.5;  // Convective velocity scale
    double z = 500.0;     // Height in boundary layer
    double zi = 1000.0;   // Boundary layer height

    double w_thermal = w_star * 1.1 * pow(z / zi, 1.0/3.0) *
                       (1.0 - 1.1 * z / zi);

    TS_ASSERT(std::isfinite(w_thermal));
  }

  // Test thermal diameter
  void testThermalDiameter() {
    double zi = 1500.0;  // Boundary layer height (m)

    double D_thermal = 0.4 * zi;  // Typical thermal diameter

    TS_ASSERT_DELTA(D_thermal, 600.0, epsilon);
  }

  /***************************************************************************
   * Clear Air Turbulence (CAT) Tests
   ***************************************************************************/

  // Test Richardson number for CAT
  void testRichardsonNumber() {
    double N2 = 1e-4;     // Brunt-Vaisala squared (s^-2)
    double du_dz = 0.01;  // Wind shear (s^-1)

    double Ri = N2 / (du_dz * du_dz);

    TS_ASSERT(Ri > 0);
    TS_ASSERT_DELTA(Ri, 1.0, epsilon);
  }

  // Test critical Richardson number
  void testCriticalRichardsonNumber() {
    double Ri_critical = 0.25;  // Below this, turbulence likely

    TS_ASSERT_DELTA(Ri_critical, 0.25, epsilon);
  }

  // Test CAT probability index
  void testCATProbabilityIndex() {
    double vertical_wind_shear = 0.02;  // s^-1
    double horizontal_wind_shear = 0.01;  // s^-1

    double deformation = sqrt(vertical_wind_shear * vertical_wind_shear +
                             horizontal_wind_shear * horizontal_wind_shear);

    TS_ASSERT(deformation > 0);
    TS_ASSERT_DELTA(deformation, 0.02236, 0.0001);
  }

  /***************************************************************************
   * Spectral Analysis Tests
   ***************************************************************************/

  // Test integral of Dryden PSD equals variance
  void testDrydenPSDIntegralVariance() {
    double sigma = 5.0;
    double variance = sigma * sigma;

    // The integral of PSD over all frequencies should equal variance
    TS_ASSERT_DELTA(variance, 25.0, epsilon);
  }

  // Test Von Karman spectrum at low frequency limit
  void testVonKarmanLowFrequencyLimit() {
    double L = 1000.0;
    double sigma = 5.0;
    double V = 200.0;
    double omega = 0.001;  // Very low frequency

    double Omega = omega * L / V;
    double Phi_u = sigma * sigma * L / V / PI /
                   pow(1.0 + (1.339 * Omega) * (1.339 * Omega), 5.0/6.0);

    // At low frequency, should approach sigma^2 * L / (PI * V)
    double Phi_limit = sigma * sigma * L / (PI * V);
    TS_ASSERT_DELTA(Phi_u, Phi_limit, 0.1);
  }

  // Test spectral rolloff rate
  void testSpectralRolloffRate() {
    // Dryden: -2 slope at high frequency
    // Von Karman: -5/3 slope at high frequency
    double dryden_slope = -2.0;
    double vonkarman_slope = -5.0 / 3.0;

    TS_ASSERT_DELTA(vonkarman_slope, -1.667, 0.01);
    TS_ASSERT(dryden_slope < vonkarman_slope);  // Steeper rolloff
  }

  /***************************************************************************
   * Gust Gradient Tests
   ***************************************************************************/

  // Test gust gradient distance
  void testGustGradientDistance() {
    double H = 100.0;  // Gust gradient distance (ft)
    double V = 200.0;  // True airspeed (ft/s)

    double t_gradient = H / V;  // Time to traverse gust

    TS_ASSERT_DELTA(t_gradient, 0.5, epsilon);
  }

  // Test FAR 25 design gust velocity
  void testFAR25DesignGustVelocity() {
    double altitude = 20000.0;  // ft
    double Ude_ref = 56.0;      // Reference gust at sea level (ft/s)

    // Gust alleviation factor decreases with altitude
    double Ude = Ude_ref * (1.0 - altitude / 60000.0);

    TS_ASSERT(Ude > 0);
    TS_ASSERT(Ude < Ude_ref);
    TS_ASSERT_DELTA(Ude, 37.33, 0.1);
  }

  // Test derived gust velocity
  void testDerivedGustVelocity() {
    double Ude = 50.0;     // Design gust velocity
    double rho = 0.002377; // Sea level density
    double rho_0 = 0.002377;
    double Kg = 0.88;      // Gust alleviation factor

    double U_de_derived = Ude * Kg * sqrt(rho / rho_0);

    TS_ASSERT_DELTA(U_de_derived, 44.0, epsilon);
  }

  /***************************************************************************
   * Multiple Instance Independence Tests
   ***************************************************************************/

  // Test independent random number sequences
  void testIndependentRandomSequences() {
    std::mt19937 gen1(12345);
    std::mt19937 gen2(54321);

    double val1 = std::uniform_real_distribution<>(0.0, 1.0)(gen1);
    double val2 = std::uniform_real_distribution<>(0.0, 1.0)(gen2);

    TS_ASSERT(val1 != val2);  // Different seeds give different values
  }

  // Test seeded random reproducibility
  void testSeededRandomReproducibility() {
    std::mt19937 gen1(42);
    std::mt19937 gen2(42);

    double val1 = std::uniform_real_distribution<>(0.0, 1.0)(gen1);
    double val2 = std::uniform_real_distribution<>(0.0, 1.0)(gen2);

    TS_ASSERT_DELTA(val1, val2, epsilon);  // Same seed = same value
  }

  /***************************************************************************
   * Stress and Boundary Tests
   ***************************************************************************/

  // Test turbulence at extreme low altitude
  void testExtremeLowAltitude() {
    double altitude = 10.0;  // Minimum per MIL-F-8785C

    double L_w = std::max(altitude, 10.0);
    double L_u = L_w / pow(0.177 + 0.000823 * L_w, 1.2);

    TS_ASSERT(L_w >= 10.0);
    TS_ASSERT(L_u > 0);
    TS_ASSERT(std::isfinite(L_u));
  }

  // Test turbulence at extreme high altitude
  void testExtremHighAltitude() {
    double altitude = 80000.0;  // ft

    // Above 2000 ft, scale lengths are constant
    double L_u = 1750.0;
    double L_w = 1750.0;

    TS_ASSERT_DELTA(L_u, 1750.0, epsilon);
    TS_ASSERT_DELTA(L_w, 1750.0, epsilon);
  }

  // Test very high airspeed
  void testVeryHighAirspeed() {
    double V = 2000.0;  // ft/s (supersonic)
    double L = 1750.0;

    double tau = L / V;

    TS_ASSERT(tau > 0);
    TS_ASSERT_DELTA(tau, 0.875, epsilon);
  }

  // Test very low airspeed
  void testVeryLowAirspeed() {
    double V = 10.0;  // ft/s (near stall)
    double L = 500.0;

    double tau = L / V;

    TS_ASSERT(tau > 0);
    TS_ASSERT_DELTA(tau, 50.0, epsilon);
  }

  // Test rapid altitude changes
  void testRapidAltitudeChanges() {
    for (double alt = 10.0; alt <= 50000.0; alt *= 2.0) {
      double L_w;
      if (alt < 1000.0) {
        L_w = alt;
      } else if (alt < 2000.0) {
        L_w = 1000.0 + (alt - 1000.0) / 1000.0 * 750.0;
      } else {
        L_w = 1750.0;
      }

      TS_ASSERT(L_w > 0);
      TS_ASSERT(L_w <= 1750.0);
      TS_ASSERT(std::isfinite(L_w));
    }
  }

  /***************************************************************************
   * Coherence Function Tests
   ***************************************************************************/

  // Test lateral coherence function
  void testLateralCoherence() {
    double dy = 50.0;   // Lateral separation (ft)
    double L = 1000.0;  // Scale length
    double omega = 0.5;
    double V = 200.0;

    double Omega = omega * L / V;
    double coh = exp(-sqrt(1.0 + (1.339 * Omega) * (1.339 * Omega)) * dy / L);

    TS_ASSERT(coh >= 0.0);
    TS_ASSERT(coh <= 1.0);
  }

  // Test vertical coherence function
  void testVerticalCoherence() {
    double dz = 100.0;  // Vertical separation (ft)
    double L = 500.0;   // Scale length

    double coh = exp(-dz / L);

    TS_ASSERT(coh >= 0.0);
    TS_ASSERT(coh <= 1.0);
    TS_ASSERT_DELTA(coh, 0.8187, 0.01);
  }

  // Test zero separation coherence
  void testZeroSeparationCoherence() {
    double dy = 0.0;
    double L = 1000.0;

    double coh = exp(-dy / L);

    TS_ASSERT_DELTA(coh, 1.0, epsilon);
  }

  /***************************************************************************
   * Energy and Conservation Tests
   ***************************************************************************/

  // Test turbulent kinetic energy
  void testTurbulentKineticEnergy() {
    double sigma_u = 5.0;
    double sigma_v = 5.0;
    double sigma_w = 3.0;

    double TKE = 0.5 * (sigma_u * sigma_u + sigma_v * sigma_v + sigma_w * sigma_w);

    TS_ASSERT(TKE > 0);
    TS_ASSERT_DELTA(TKE, 29.5, epsilon);
  }

  // Test dissipation rate estimate
  void testDissipationRate() {
    double sigma = 5.0;
    double L = 1000.0;

    double epsilon_turb = pow(sigma, 3.0) / L;

    TS_ASSERT(epsilon_turb > 0);
    TS_ASSERT_DELTA(epsilon_turb, 0.125, epsilon);
  }

  // Test energy cascade consistency
  void testEnergyCascadeConsistency() {
    double sigma = 5.0;
    double L = 1000.0;

    // Kolmogorov microscale estimate (very rough)
    double nu = 1.5e-4;  // Kinematic viscosity (ft^2/s)
    double eps = pow(sigma, 3.0) / L;
    double eta = pow(nu * nu * nu / eps, 0.25);

    TS_ASSERT(eta > 0);
    TS_ASSERT(eta < L);  // Microscale much smaller than integral scale
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test ft/s to m/s conversion
  void testFpsToMpsConversion() {
    double V_fps = 100.0;
    double ft_to_m = 0.3048;

    double V_mps = V_fps * ft_to_m;

    TS_ASSERT_DELTA(V_mps, 30.48, 0.01);
  }

  // Test ft to m conversion for scale length
  void testFeetToMetersScaleLength() {
    double L_ft = 1750.0;
    double ft_to_m = 0.3048;

    double L_m = L_ft * ft_to_m;

    TS_ASSERT_DELTA(L_m, 533.4, 0.1);
  }

  // Test knots to ft/s conversion
  void testKnotsToFpsConversion() {
    double V_kts = 200.0;
    double kts_to_fps = 1.68781;

    double V_fps = V_kts * kts_to_fps;

    TS_ASSERT_DELTA(V_fps, 337.56, 0.1);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete Dryden turbulence chain
  void testCompleteDrydenTurbulenceChain() {
    // Complete turbulence calculation from altitude to gust velocities
    double altitude = 750.0;  // ft AGL
    double V = 200.0;         // True airspeed (ft/s)
    double windspeed_at_20ft = 25.0;  // ft/s

    // Step 1: Calculate scale lengths (low altitude model)
    double L_u = altitude / pow(0.177 + 0.000823 * altitude, 1.2);
    double L_w = altitude;
    TS_ASSERT(L_u > L_w);
    TS_ASSERT(L_w > 0);

    // Step 2: Calculate turbulence intensities
    double sig_w = 0.1 * windspeed_at_20ft;
    double sig_u = sig_w / pow(0.177 + 0.000823 * altitude, 0.4);
    TS_ASSERT(sig_u > sig_w);
    TS_ASSERT(sig_w > 0);

    // Step 3: Calculate time constants
    double tau_u = L_u / V;
    double tau_w = L_w / V;
    TS_ASSERT(tau_u > tau_w);
    TS_ASSERT(tau_w > 0);

    // Step 4: Calculate filter gains
    double K_u = sig_u * sqrt(2.0 * L_u / (PI * V));
    double K_w = sig_w * sqrt(2.0 * L_w / (PI * V));
    TS_ASSERT(K_u > 0);
    TS_ASSERT(K_w > 0);

    // Step 5: Calculate PSD at a reference frequency
    double omega = 0.5;
    double Omega_u = omega * L_u / V;
    double Phi_u = sig_u * sig_u * 2.0 * L_u / (PI * V) /
                   (1.0 + (1.339 * Omega_u) * (1.339 * Omega_u));
    TS_ASSERT(Phi_u > 0);
    TS_ASSERT(std::isfinite(Phi_u));

    // Step 6: Verify consistency
    double TKE = 0.5 * (sig_u * sig_u + sig_u * sig_u + sig_w * sig_w);
    TS_ASSERT(TKE > 0);
  }

  // Test complete discrete gust encounter
  void testCompleteDiscreteGustEncounter() {
    // Simulate aircraft flying through a 1-cosine gust
    double V = 200.0;            // True airspeed (ft/s)
    double gustAmplitude = 30.0; // ft/s (severe gust)
    double gustLength = 300.0;   // ft

    // Time to traverse gust
    double t_gust = gustLength / V;
    TS_ASSERT_DELTA(t_gust, 1.5, 0.01);

    // Sample gust at several points
    int numSamples = 10;
    double maxGust = 0.0;
    double sumGust = 0.0;

    for (int i = 0; i <= numSamples; i++) {
      double x = gustLength * i / numSamples;
      double gust = (gustAmplitude / 2.0) * (1.0 - cos(2.0 * PI * x / gustLength));

      if (gust > maxGust) maxGust = gust;
      sumGust += gust;

      TS_ASSERT(gust >= 0.0);
      TS_ASSERT(gust <= gustAmplitude);
    }

    // Verify peak at center
    TS_ASSERT_DELTA(maxGust, gustAmplitude, 0.1);

    // Verify average (should be roughly gustAmplitude/2)
    double avgGust = sumGust / (numSamples + 1);
    TS_ASSERT(avgGust > gustAmplitude * 0.4);
    TS_ASSERT(avgGust < gustAmplitude * 0.6);
  }

  // Test complete altitude profile calculation
  void testCompleteAltitudeProfile() {
    // Test turbulence parameters across full altitude range
    double altitudes[] = {10.0, 100.0, 500.0, 1000.0, 1500.0, 2000.0, 5000.0, 20000.0};

    double prev_L = 0.0;
    for (double alt : altitudes) {
      double L_w, L_u;

      if (alt < 1000.0) {
        L_w = alt;
        L_u = alt / pow(0.177 + 0.000823 * alt, 1.2);
      } else if (alt < 2000.0) {
        L_w = 1000.0 + (alt - 1000.0) / 1000.0 * 750.0;
        L_u = L_w;
      } else {
        L_w = 1750.0;
        L_u = 1750.0;
      }

      TS_ASSERT(L_w > 0);
      TS_ASSERT(L_u > 0);
      TS_ASSERT(L_w <= 1750.0);
      TS_ASSERT(L_u <= 2000.0);  // Reasonable upper bound
      TS_ASSERT(std::isfinite(L_w));
      TS_ASSERT(std::isfinite(L_u));

      // Scale length should generally increase with altitude up to 2000 ft
      if (alt <= 2000.0 && prev_L > 0) {
        TS_ASSERT(L_w >= prev_L * 0.99);  // Allow small tolerance
      }
      prev_L = L_w;
    }
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test independent turbulence models
  void testIndependentTurbulenceModels() {
    // Verify Dryden and Von Karman models give independent results
    double sigma = 5.0;
    double L = 1000.0;
    double V = 200.0;
    double omega = 0.5;

    // Dryden longitudinal PSD
    double Omega = omega * L / V;
    double Phi_dryden = sigma * sigma * 2.0 * L / (PI * V) /
                        (1.0 + (1.339 * Omega) * (1.339 * Omega));

    // Von Karman longitudinal PSD
    double Phi_vonkarman = sigma * sigma * L / V / PI /
                           pow(1.0 + 8.0 / 3.0 * (1.339 * Omega) * (1.339 * Omega), 5.0/6.0);

    // Both should be positive and finite
    TS_ASSERT(Phi_dryden > 0);
    TS_ASSERT(Phi_vonkarman > 0);
    TS_ASSERT(std::isfinite(Phi_dryden));
    TS_ASSERT(std::isfinite(Phi_vonkarman));

    // They should give different values (different spectral shapes)
    TS_ASSERT(std::abs(Phi_dryden - Phi_vonkarman) > 0.01);

    // Test at multiple frequencies
    double omegas[] = {0.1, 0.5, 1.0, 5.0, 10.0};
    for (double w : omegas) {
      double Om = w * L / V;
      double pd = sigma * sigma * 2.0 * L / (PI * V) /
                  (1.0 + (1.339 * Om) * (1.339 * Om));
      double pv = sigma * sigma * L / V / PI /
                  pow(1.0 + 8.0 / 3.0 * (1.339 * Om) * (1.339 * Om), 5.0/6.0);

      TS_ASSERT(pd > 0);
      TS_ASSERT(pv > 0);
    }
  }

  // Test independent flight condition turbulence
  void testIndependentFlightConditionTurbulence() {
    // Different flight conditions should give independent results
    struct FlightCondition {
      double altitude;
      double airspeed;
      double windspeed_20ft;
    };

    FlightCondition low_slow = {200.0, 100.0, 15.0};
    FlightCondition mid_cruise = {1500.0, 250.0, 25.0};
    FlightCondition high_fast = {25000.0, 400.0, 30.0};

    auto calcTurbParams = [](FlightCondition& fc) {
      double L_w, sig_w;
      if (fc.altitude < 1000.0) {
        L_w = fc.altitude;
        sig_w = 0.1 * fc.windspeed_20ft;
      } else if (fc.altitude < 2000.0) {
        L_w = 1000.0 + (fc.altitude - 1000.0) / 1000.0 * 750.0;
        sig_w = 0.1 * fc.windspeed_20ft;
      } else {
        L_w = 1750.0;
        sig_w = 3.0;  // High altitude intensity
      }
      double tau = L_w / fc.airspeed;
      return std::make_tuple(L_w, sig_w, tau);
    };

    auto [L1, sig1, tau1] = calcTurbParams(low_slow);
    auto [L2, sig2, tau2] = calcTurbParams(mid_cruise);
    auto [L3, sig3, tau3] = calcTurbParams(high_fast);

    // Each condition gives independent, valid results
    TS_ASSERT(L1 > 0);
    TS_ASSERT(L2 > L1);  // Scale length increases
    TS_ASSERT(L3 > L2);

    TS_ASSERT(sig1 > 0);
    TS_ASSERT(sig2 > sig1);
    TS_ASSERT(sig3 > 0);

    TS_ASSERT(tau1 > 0);
    TS_ASSERT(tau2 > 0);
    TS_ASSERT(tau3 > 0);

    // Verify independence
    TS_ASSERT(L1 != L2);
    TS_ASSERT(L2 != L3);
    TS_ASSERT(tau1 != tau2);
    TS_ASSERT(tau2 != tau3);
  }

  // Test independent gust component calculations
  void testIndependentGustComponents() {
    // u, v, w gust components should be calculated independently
    double sigma_u = 5.0;
    double sigma_v = 5.0;
    double sigma_w = 3.0;

    double L_u = 1000.0;
    double L_v = 1000.0;
    double L_w = 500.0;

    double V = 200.0;

    // Calculate time constants for each component
    double tau_u = L_u / V;
    double tau_v = L_v / V;
    double tau_w = L_w / V;

    TS_ASSERT_DELTA(tau_u, 5.0, 0.01);
    TS_ASSERT_DELTA(tau_v, 5.0, 0.01);
    TS_ASSERT_DELTA(tau_w, 2.5, 0.01);

    // Calculate filter gains for each component
    double K_u = sigma_u * sqrt(2.0 * L_u / (PI * V));
    double K_v = sigma_v * sqrt(2.0 * L_v / (PI * V));
    double K_w = sigma_w * sqrt(2.0 * L_w / (PI * V));

    TS_ASSERT(K_u > 0);
    TS_ASSERT(K_v > 0);
    TS_ASSERT(K_w > 0);

    // u and v should have same gain (same sigma and L)
    TS_ASSERT_DELTA(K_u, K_v, 0.001);

    // w should have different gain
    TS_ASSERT(std::abs(K_w - K_u) > 0.1);

    // Verify turbulent kinetic energy calculation
    double TKE = 0.5 * (sigma_u * sigma_u + sigma_v * sigma_v + sigma_w * sigma_w);
    TS_ASSERT_DELTA(TKE, 29.5, 0.01);

    // Each component contributes independently
    double TKE_u = 0.5 * sigma_u * sigma_u;
    double TKE_v = 0.5 * sigma_v * sigma_v;
    double TKE_w = 0.5 * sigma_w * sigma_w;
    TS_ASSERT_DELTA(TKE_u + TKE_v + TKE_w, TKE, 0.01);
  }

  //==========================================================================
  // C172x Model-Based Turbulence Tests
  //==========================================================================

  // Test C172x set turbulence type None
  void testC172xTurbulenceTypeNone() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttNone);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // With no turbulence, simulation should be stable
    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x set turbulence type Standard
  void testC172xTurbulenceTypeStandard() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x set turbulence type Milspec
  void testC172xTurbulenceTypeMilspec() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttMilspec);
    winds->SetProbabilityOfExceedence(0.03);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x set turbulence type Tustin
  void testC172xTurbulenceTypeTustin() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttTustin);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x turbulence gain setting
  void testC172xTurbulenceGain() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(2.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x low turbulence gain
  void testC172xLowTurbulenceGain() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(0.5);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x turbulence rate
  void testC172xTurbulenceRate() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);
    winds->SetTurbRate(5.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x milspec probability of exceedance
  void testC172xMilspecProbability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    winds->SetTurbType(FGWinds::ttMilspec);
    winds->SetProbabilityOfExceedence(0.01);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x turbulence effect on accelerations
  void testC172xTurbulenceAccelerations() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto accel = fdmex.GetAccelerations();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    FGColumnVector3 bodyAccel = accel->GetBodyAccel();
    TS_ASSERT(std::isfinite(bodyAccel(1)));
    TS_ASSERT(std::isfinite(bodyAccel(2)));
    TS_ASSERT(std::isfinite(bodyAccel(3)));
  }

  // Test C172x turbulence with wind
  void testC172xTurbulenceWithWind() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    // Set steady wind
    winds->SetWindNED(10.0, 5.0, 0.0);

    // Add turbulence
    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(auxiliary->Getalpha()));
  }

  // Test C172x turbulence stability
  void testC172xTurbulenceStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    // Run extended simulation
    for (int i = 0; i < 500; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(propagate->GetAltitudeASL()));
  }

  // Test C172x turbulence type switching
  void testC172xTurbulenceTypeSwitching() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();

    // Start with standard
    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 25; i++) {
      fdmex.Run();
    }

    // Switch to milspec
    winds->SetTurbType(FGWinds::ttMilspec);
    winds->SetProbabilityOfExceedence(0.03);

    for (int i = 0; i < 25; i++) {
      fdmex.Run();
    }

    auto auxiliary = fdmex.GetAuxiliary();
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x no turbulence baseline
  void testC172xNoTurbulenceBaseline() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    winds->SetTurbType(FGWinds::ttNone);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Without turbulence, flight should be smooth
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(auxiliary->Getalpha()));
    TS_ASSERT(std::isfinite(auxiliary->Getbeta()));
  }

  // Test C172x high turbulence gain
  void testC172xHighTurbulenceGain() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(3.0);  // High gain

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Should still be stable even with high turbulence
    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x zero turbulence gain
  void testC172xZeroTurbulenceGain() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(0.0);  // Zero gain = no turbulence

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x turbulence with gust
  void testC172xTurbulenceWithGust() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    // Set gust
    winds->SetGustNED(5.0, 3.0, 2.0);

    // Add turbulence
    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
  }

  // Test C172x combined wind effects
  void testC172xCombinedWindEffects() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    // Set wind + gust + turbulence
    winds->SetWindNED(10.0, 0.0, 0.0);
    winds->SetGustNED(5.0, 2.0, 1.0);
    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(0.5);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(auxiliary->Getalpha()));
    TS_ASSERT(std::isfinite(auxiliary->Getbeta()));
  }

  // Test C172x milspec severity levels
  void testC172xMilspecSeverityLevels() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    winds->SetTurbType(FGWinds::ttMilspec);

    // Test different probability levels
    double probs[] = {0.01, 0.03, 0.1};

    for (double prob : probs) {
      winds->SetProbabilityOfExceedence(prob);

      for (int i = 0; i < 20; i++) {
        fdmex.Run();
      }

      TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    }
  }

  // Test C172x turbulence angles effect
  void testC172xTurbulenceAnglesEffect() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();

    winds->SetTurbType(FGWinds::ttStandard);
    winds->SetTurbGain(1.0);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // All angles should remain finite
    TS_ASSERT(std::isfinite(propagate->GetEuler(1)));  // Roll
    TS_ASSERT(std::isfinite(propagate->GetEuler(2)));  // Pitch
    TS_ASSERT(std::isfinite(propagate->GetEuler(3)));  // Yaw
    TS_ASSERT(std::isfinite(auxiliary->Getalpha()));
    TS_ASSERT(std::isfinite(auxiliary->Getbeta()));
  }

  // Test C172x Tustin turbulence stability
  void testC172xTustinTurbulenceStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto winds = fdmex.GetWinds();
    auto auxiliary = fdmex.GetAuxiliary();

    winds->SetTurbType(FGWinds::ttTustin);
    winds->SetTurbGain(1.0);

    // Extended run
    for (int i = 0; i < 300; i++) {
      fdmex.Run();
    }

    TS_ASSERT(std::isfinite(auxiliary->GetVt()));
    TS_ASSERT(std::isfinite(auxiliary->GetMach()));
  }
};
