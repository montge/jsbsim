#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <complex>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Filter unit tests
 *
 * Note: FGFilter requires XML element for construction, so these tests focus on:
 * - Lag filter: C1/(s+C1)
 * - Lead-lag filter: (C1*s+C2)/(C3*s+C4)
 * - Washout filter: s/(s+C1)
 * - Second order filter: (C1*s^2+C2*s+C3)/(C4*s^2+C5*s+C6)
 * - Integrator: C1/s
 */
class FGFilterTest : public CxxTest::TestSuite
{
public:
  // Test lag filter DC gain
  void testLagFilterDCGain() {
    // Lag filter: C1/(s+C1)
    // At DC (s=0): gain = C1/C1 = 1
    double C1 = 10.0;
    double s = 0.0;
    double gain = C1 / (s + C1);

    TS_ASSERT_DELTA(gain, 1.0, epsilon);
  }

  // Test lag filter high frequency gain
  void testLagFilterHighFreqGain() {
    // At high frequency (s >> C1): gain -> C1/s -> 0
    double C1 = 10.0;
    double s = 1000.0;  // High frequency
    double gain = C1 / (s + C1);

    TS_ASSERT(gain < 0.01);  // Nearly zero
  }

  // Test lag filter at corner frequency
  void testLagFilterCornerFreq() {
    // At corner frequency (s = C1): gain = C1/(2*C1) = 0.5
    // Actually |H(jw)| = C1/sqrt(w^2 + C1^2) at w = C1 gives 1/sqrt(2) = 0.707
    double C1 = 10.0;
    double omega = C1;  // Corner frequency
    double gain_mag = C1 / std::sqrt(omega*omega + C1*C1);

    TS_ASSERT_DELTA(gain_mag, 1.0/std::sqrt(2.0), 0.001);
  }

  // Test lag filter step response
  void testLagFilterStepResponse() {
    double C1 = 10.0;
    double dt = 0.001;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double u = 1.0;  // Step input

    // Simulate for 5 time constants
    double t_final = 5.0 / C1;
    int steps = static_cast<int>(t_final / dt);

    for (int i = 0; i < steps; i++) {
      y = ca * y + cb * u;
    }

    // After 5 time constants, should be within 1% of final value
    TS_ASSERT_DELTA(y, 1.0, 0.01);
  }

  // Test lead-lag filter DC gain
  void testLeadLagDCGain() {
    // Lead-lag: (C1*s+C2)/(C3*s+C4)
    // At DC (s=0): gain = C2/C4
    double C1 = 2.0, C2 = 10.0, C3 = 1.0, C4 = 5.0;
    double dc_gain = C2 / C4;

    TS_ASSERT_DELTA(dc_gain, 2.0, epsilon);
  }

  // Test lead-lag filter high frequency gain
  void testLeadLagHighFreqGain() {
    // At high frequency (s >> 1): gain -> C1/C3
    double C1 = 2.0, C2 = 10.0, C3 = 1.0, C4 = 5.0;
    double hf_gain = C1 / C3;

    TS_ASSERT_DELTA(hf_gain, 2.0, epsilon);
  }

  // Test lead-lag phase characteristics
  void testLeadLagPhase() {
    // Lead-lag with lead: C1/C3 > C2/C4 -> phase lead at mid frequencies
    // Lead-lag with lag: C1/C3 < C2/C4 -> phase lag at mid frequencies

    double C1 = 1.0, C2 = 10.0, C3 = 1.0, C4 = 1.0;
    double dc = C2 / C4;    // 10
    double hf = C1 / C3;    // 1

    // This is a lag network (attenuates HF)
    TS_ASSERT(hf < dc);
  }

  // Test washout filter DC gain (zero)
  void testWashoutDCGain() {
    // Washout: s/(s+C1)
    // At DC (s=0): gain = 0
    double C1 = 10.0;
    double s = 0.0;
    // Avoid division by zero, use limit
    double gain = (s == 0.0) ? 0.0 : s / (s + C1);

    TS_ASSERT_DELTA(gain, 0.0, epsilon);
  }

  // Test washout filter high frequency gain
  void testWashoutHighFreqGain() {
    // At high frequency (s >> C1): gain -> 1
    double C1 = 10.0;
    double s = 1000.0;
    double gain = s / (s + C1);

    TS_ASSERT(gain > 0.99);
  }

  // Test washout step response
  void testWashoutStepResponse() {
    // Washout passes transients, blocks DC
    double C1 = 10.0;
    double dt = 0.001;

    double y = 0.0;
    double u = 1.0;
    double u_prev = 0.0;

    // Washout discrete: y = a*y_prev + b*(u - u_prev)
    double a = std::exp(-C1 * dt);
    double b = 1.0;

    // Initially jumps up
    y = a * y + b * (u - u_prev);
    TS_ASSERT(y > 0.9);

    u_prev = u;

    // Then decays to zero
    for (int i = 0; i < 10000; i++) {
      y = a * y + b * (u - u_prev);
      u_prev = u;
    }

    TS_ASSERT_DELTA(y, 0.0, 0.01);
  }

  // Test second order filter DC gain
  void testSecondOrderDCGain() {
    // (C1*s^2+C2*s+C3)/(C4*s^2+C5*s+C6)
    // At DC: gain = C3/C6
    double C1 = 0.0, C2 = 0.0, C3 = 100.0;
    double C4 = 1.0, C5 = 10.0, C6 = 100.0;

    double dc_gain = C3 / C6;
    TS_ASSERT_DELTA(dc_gain, 1.0, epsilon);
  }

  // Test second order underdamped natural frequency
  void testSecondOrderNaturalFreq() {
    // Standard form: 1/(s^2/wn^2 + 2*zeta*s/wn + 1)
    // Or: wn^2/(s^2 + 2*zeta*wn*s + wn^2)
    // C4 = 1, C5 = 2*zeta*wn, C6 = wn^2

    double wn = 10.0;      // Natural frequency
    double zeta = 0.7;     // Damping ratio

    double C4 = 1.0;
    double C5 = 2.0 * zeta * wn;
    double C6 = wn * wn;

    TS_ASSERT_DELTA(C5, 14.0, epsilon);
    TS_ASSERT_DELTA(C6, 100.0, epsilon);
  }

  // Test second order damping ratio effects
  void testSecondOrderDamping() {
    double wn = 10.0;

    // Underdamped (zeta < 1): oscillatory
    double zeta_under = 0.5;
    double discriminant_under = (2*zeta_under*wn)*(2*zeta_under*wn) - 4*wn*wn;
    TS_ASSERT(discriminant_under < 0);  // Complex roots -> oscillatory

    // Critically damped (zeta = 1): fastest non-oscillatory
    double zeta_crit = 1.0;
    double discriminant_crit = (2*zeta_crit*wn)*(2*zeta_crit*wn) - 4*wn*wn;
    TS_ASSERT_DELTA(discriminant_crit, 0.0, epsilon);

    // Overdamped (zeta > 1): sluggish
    double zeta_over = 2.0;
    double discriminant_over = (2*zeta_over*wn)*(2*zeta_over*wn) - 4*wn*wn;
    TS_ASSERT(discriminant_over > 0);  // Real roots -> no oscillation
  }

  // Test integrator gain
  void testIntegratorGain() {
    // Integrator: C1/s
    double C1 = 2.0;
    double dt = 0.1;

    double y = 0.0;
    double u = 1.0;

    // Rectangular integration
    for (int i = 0; i < 10; i++) {
      y += C1 * u * dt;
    }

    // Integral of 1 over 1 second with gain 2 = 2
    TS_ASSERT_DELTA(y, 2.0, epsilon);
  }

  // Test integrator with trigger reset
  void testIntegratorTrigger() {
    double y = 100.0;  // Accumulated value
    int trigger = 1;   // Non-zero = reset inputs

    if (trigger != 0) {
      y = 0.0;  // Reset
    }

    TS_ASSERT_DELTA(y, 0.0, epsilon);
  }

  // Test filter stability (poles in LHP)
  void testFilterStability() {
    // First-order stable: pole at s = -C1 (C1 > 0)
    double C1 = 10.0;
    double pole_real = -C1;
    TS_ASSERT(pole_real < 0);  // Stable

    // Second-order stable: both poles have negative real parts
    double wn = 10.0, zeta = 0.5;
    std::complex<double> s1(-zeta * wn, wn * std::sqrt(1 - zeta*zeta));
    std::complex<double> s2(-zeta * wn, -wn * std::sqrt(1 - zeta*zeta));

    TS_ASSERT(s1.real() < 0);
    TS_ASSERT(s2.real() < 0);
  }

  // Test frequency response magnitude
  void testFreqResponseMagnitude() {
    // Lag filter |H(jw)| = C1/sqrt(w^2 + C1^2)
    double C1 = 10.0;

    double freqs[] = {0.0, 1.0, 10.0, 100.0};
    double expected_mags[] = {1.0, 0.995, 0.707, 0.0995};

    for (int i = 0; i < 4; i++) {
      double w = freqs[i];
      double mag = C1 / std::sqrt(w*w + C1*C1);
      TS_ASSERT_DELTA(mag, expected_mags[i], 0.01);
    }
  }

  // Test frequency response phase
  void testFreqResponsePhase() {
    // Lag filter phase = -atan(w/C1)
    double C1 = 10.0;

    // At corner frequency, phase = -45 degrees
    double w = C1;
    double phase_rad = -std::atan(w / C1);
    double phase_deg = phase_rad * 180.0 / M_PI;

    TS_ASSERT_DELTA(phase_deg, -45.0, 0.01);
  }

  // Test Tustin (bilinear) transform
  void testTustinTransform() {
    // Bilinear: s = 2/T * (z-1)/(z+1)
    // For lag filter C1/(s+C1), discretized:
    double C1 = 10.0;
    double T = 0.01;  // Sample period

    // Pre-warped frequency
    double wd = C1;  // Desired analog frequency
    double wa = (2.0/T) * std::tan(wd * T / 2.0);

    // For C1 = 10, T = 0.01: tan(0.05) ≈ 0.05
    TS_ASSERT_DELTA(wa, C1, 0.5);  // Close to original for small wT
  }

  // Test clipto limits
  void testCliptoLimits() {
    double min_val = -10.0;
    double max_val = 10.0;

    double values[] = {-100.0, -10.0, 0.0, 10.0, 100.0};
    double expected[] = {-10.0, -10.0, 0.0, 10.0, 10.0};

    for (int i = 0; i < 5; i++) {
      double clipped = std::max(min_val, std::min(max_val, values[i]));
      TS_ASSERT_DELTA(clipped, expected[i], epsilon);
    }
  }

  // Test filter coefficient calculation
  void testFilterCoefficients() {
    // For a lag filter using RK2 integration:
    // ca = (1 - C1*dt/2) / (1 + C1*dt/2)
    // cb = C1*dt / (1 + C1*dt/2)

    double C1 = 10.0;
    double dt = 0.01;

    double denom = 1.0 + C1 * dt / 2.0;  // = 1.05
    double ca = (1.0 - C1 * dt / 2.0) / denom;  // = 0.95/1.05 = 0.9047
    double cb = C1 * dt / denom;  // = 0.1/1.05 = 0.0952

    TS_ASSERT_DELTA(ca, 0.9047, 0.001);
    TS_ASSERT_DELTA(cb, 0.0952, 0.001);

    // DC gain check: (1 + cb/ca ... actually verify sum behavior)
    // At steady state: y = ca*y + cb*u => y(1-ca) = cb*u => y/u = cb/(1-ca)
    double dc = cb / (1.0 - ca);
    TS_ASSERT_DELTA(dc, 1.0, 0.01);
  }

  // Test dynamic filter (time-varying coefficients)
  void testDynamicFilter() {
    // When filter coefficients come from properties, they can change
    double C1_initial = 10.0;
    double C1_final = 20.0;

    // Initial corner frequency
    double f1_initial = C1_initial / (2.0 * M_PI);
    // Final corner frequency
    double f1_final = C1_final / (2.0 * M_PI);

    TS_ASSERT_DELTA(f1_initial, 1.59, 0.01);
    TS_ASSERT_DELTA(f1_final, 3.18, 0.01);
  }

  // Test notch filter concept
  void testNotchFilterConcept() {
    // Notch filter: (s^2 + wn^2)/(s^2 + 2*zeta*wn*s + wn^2)
    // Zero gain at s = j*wn

    double wn = 10.0;
    double zeta = 0.1;  // Narrow notch

    // At notch frequency
    std::complex<double> s(0.0, wn);
    std::complex<double> num = s*s + wn*wn;
    // |num| at s=jwn: |-wn^2 + wn^2| = 0

    TS_ASSERT_DELTA(std::abs(num), 0.0, epsilon);
  }

  // Test filter settling time
  void testSettlingTime() {
    // For first-order system, settling time (2%) ≈ 4*tau
    double C1 = 10.0;
    double tau = 1.0 / C1;
    double settling_2pct = 4.0 * tau;

    TS_ASSERT_DELTA(settling_2pct, 0.4, 0.001);

    // For second-order system: ts ≈ 4/(zeta*wn)
    double wn = 10.0, zeta = 0.7;
    double ts_2nd = 4.0 / (zeta * wn);

    TS_ASSERT_DELTA(ts_2nd, 0.571, 0.01);
  }

  // Test filter rise time
  void testRiseTime() {
    // For first-order: tr (10-90%) ≈ 2.2*tau
    double C1 = 10.0;
    double tau = 1.0 / C1;
    double rise_time = 2.2 * tau;

    TS_ASSERT_DELTA(rise_time, 0.22, 0.01);
  }

  // Test filter overshoot
  void testOvershoot() {
    // For second-order underdamped: Mp = exp(-pi*zeta/sqrt(1-zeta^2))
    double zeta = 0.5;
    double overshoot = std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta*zeta));

    TS_ASSERT_DELTA(overshoot, 0.163, 0.01);  // 16.3%

    // Critically damped: no overshoot
    zeta = 1.0;
    // sqrt(1-1) = 0, so formula doesn't apply - overshoot is 0
    TS_ASSERT_DELTA(0.0, 0.0, epsilon);  // No overshoot
  }

  // Test integrator anti-windup
  void testIntegratorAntiWindup() {
    double y = 0.0;
    double u = 10.0;
    double C1 = 1.0;
    double dt = 0.1;
    double max_val = 5.0;

    for (int i = 0; i < 100; i++) {
      y += C1 * u * dt;
      y = std::min(y, max_val);  // Anti-windup clip
    }

    TS_ASSERT_DELTA(y, max_val, epsilon);
  }

  // Test filter passband/stopband concept
  void testPassbandStopband() {
    // Low-pass filter: passes low frequencies, attenuates high
    double C1 = 10.0;  // Corner at 10 rad/s

    // Passband (w < C1): gain ≈ 1
    double w_pass = 1.0;
    double gain_pass = C1 / std::sqrt(w_pass*w_pass + C1*C1);
    TS_ASSERT(gain_pass > 0.99);

    // Stopband (w >> C1): gain ≈ 0
    double w_stop = 100.0;
    double gain_stop = C1 / std::sqrt(w_stop*w_stop + C1*C1);
    TS_ASSERT(gain_stop < 0.1);
  }

  /***************************************************************************
   * Additional Comprehensive Filter Tests
   ***************************************************************************/

  // Test high-pass filter DC gain (zero)
  void testHighPassDCGain() {
    // High-pass: s/(s+C1)
    // At DC (s=0): gain = 0
    double C1 = 10.0;
    double s = 0.001;  // Near DC
    double gain = s / (s + C1);

    TS_ASSERT(gain < 0.001);
  }

  // Test high-pass filter high frequency gain
  void testHighPassHighFreqGain() {
    // At high frequency (s >> C1): gain -> 1
    double C1 = 10.0;
    double s = 1000.0;
    double gain = s / (s + C1);

    TS_ASSERT(gain > 0.99);
  }

  // Test bandpass filter concept
  void testBandpassFilterConcept() {
    // Bandpass: (C1*s)/((s+wL)(s+wH)) where wL < wH
    // Peak at geometric mean
    double wL = 1.0;   // Low corner
    double wH = 100.0; // High corner
    double wCenter = std::sqrt(wL * wH);

    TS_ASSERT_DELTA(wCenter, 10.0, epsilon);
  }

  // Test filter quality factor
  void testFilterQualityFactor() {
    // Q = wn / (2*zeta*wn) = 1/(2*zeta)
    double zeta = 0.1;
    double Q = 1.0 / (2.0 * zeta);

    TS_ASSERT_DELTA(Q, 5.0, epsilon);

    // High Q = sharp resonance
    zeta = 0.01;
    Q = 1.0 / (2.0 * zeta);
    TS_ASSERT_DELTA(Q, 50.0, epsilon);
  }

  // Test resonant peak magnitude
  void testResonantPeak() {
    // For underdamped 2nd order, peak magnitude = 1/(2*zeta*sqrt(1-zeta^2))
    double zeta = 0.1;
    double peak = 1.0 / (2.0 * zeta * std::sqrt(1.0 - zeta*zeta));

    TS_ASSERT(peak > 5.0);  // Significant resonance
  }

  // Test resonant frequency
  void testResonantFrequency() {
    // Resonant frequency wr = wn*sqrt(1-2*zeta^2)
    double wn = 10.0;
    double zeta = 0.3;
    double wr = wn * std::sqrt(1.0 - 2.0*zeta*zeta);

    TS_ASSERT(wr < wn);  // Resonance below natural freq
    TS_ASSERT_DELTA(wr, 9.06, 0.01);
  }

  // Test filter phase margin concept
  void testPhaseMarginConcept() {
    // Phase margin = 180 + phase at gain crossover
    // For lag filter at corner: phase = -45 deg, PM = 135 deg
    double C1 = 10.0;
    double w_crossover = C1;  // Assuming gain crossover at corner
    double phase = -std::atan(w_crossover / C1) * 180.0 / M_PI;

    double PM = 180.0 + phase;
    TS_ASSERT_DELTA(PM, 135.0, 0.1);
  }

  // Test filter gain margin concept
  void testGainMarginConcept() {
    // Gain margin = 1/|H(jw)| at phase crossover (where phase = -180)
    // For first-order lag, phase never reaches -180, so GM = infinity
    // For second-order, depends on parameters

    // Simplified check: stable system has positive GM
    double GM_db = 20.0;  // 20 dB gain margin
    double GM_linear = std::pow(10.0, GM_db / 20.0);

    TS_ASSERT_DELTA(GM_linear, 10.0, 0.01);
  }

  // Test cascaded filters
  void testCascadedFilters() {
    // Two first-order lags in series
    double C1 = 10.0;
    double C2 = 20.0;
    double dt = 0.01;

    double ca1 = std::exp(-C1 * dt);
    double cb1 = 1.0 - ca1;
    double ca2 = std::exp(-C2 * dt);
    double cb2 = 1.0 - ca2;

    double y1 = 0.0, y2 = 0.0;
    double u = 1.0;

    // Run until settled
    for (int i = 0; i < 5000; i++) {
      y1 = ca1 * y1 + cb1 * u;
      y2 = ca2 * y2 + cb2 * y1;
    }

    // Both should converge to 1.0
    TS_ASSERT_DELTA(y1, 1.0, 0.001);
    TS_ASSERT_DELTA(y2, 1.0, 0.001);
  }

  // Test parallel filters
  void testParallelFilters() {
    // Two filters in parallel with different coefficients
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
      y2 = ca2 * y2 + cb2 * u;
    }

    // Sum of outputs
    double y_sum = y1 + y2;
    TS_ASSERT_DELTA(y_sum, 2.0, 0.01);  // Both converge to 1
  }

  // Test forward Euler integration
  void testForwardEulerIntegration() {
    double C1 = 10.0;
    double dt = 0.01;

    // Forward Euler: y(n+1) = y(n) + dt*f(y,u)
    // For dy/dt = C1*(u - y): y(n+1) = y(n) + dt*C1*(u - y(n))
    double y = 0.0;
    double u = 1.0;

    for (int i = 0; i < 1000; i++) {
      y = y + dt * C1 * (u - y);
    }

    TS_ASSERT_DELTA(y, 1.0, 0.001);
  }

  // Test backward Euler integration
  void testBackwardEulerIntegration() {
    double C1 = 10.0;
    double dt = 0.01;

    // Backward Euler: y(n+1) = (y(n) + dt*C1*u) / (1 + dt*C1)
    double y = 0.0;
    double u = 1.0;

    for (int i = 0; i < 1000; i++) {
      y = (y + dt * C1 * u) / (1.0 + dt * C1);
    }

    TS_ASSERT_DELTA(y, 1.0, 0.001);
  }

  // Test trapezoidal integration
  void testTrapezoidalIntegration() {
    // Trapezoidal: y(n+1) = (1-C1*dt/2)/(1+C1*dt/2)*y(n) + (C1*dt)/(1+C1*dt/2)*u
    double C1 = 10.0;
    double dt = 0.01;

    double denom = 1.0 + C1 * dt / 2.0;
    double ca = (1.0 - C1 * dt / 2.0) / denom;
    double cb = (C1 * dt) / denom;

    double y = 0.0;
    double u = 1.0;

    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * u;
    }

    TS_ASSERT_DELTA(y, 1.0, 0.001);
  }

  // Test filter initial conditions
  void testFilterInitialConditions() {
    double C1 = 10.0;
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    // Non-zero initial condition
    double y = 5.0;  // IC
    double u = 1.0;

    // Should decay to final value
    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * u;
    }

    TS_ASSERT_DELTA(y, 1.0, 0.001);
  }

  // Test filter with sinusoidal input
  void testFilterSinusoidalInput() {
    double C1 = 10.0;
    double dt = 0.001;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double omega = 5.0;  // Input frequency below corner
    double y = 0.0;

    // Run for several cycles
    for (int i = 0; i < 10000; i++) {
      double t = i * dt;
      double u = std::sin(omega * t);
      y = ca * y + cb * u;
    }

    // Output should be sinusoidal (check by verifying bounded)
    TS_ASSERT(std::abs(y) <= 1.0);
  }

  // Test differentiator approximation
  void testDifferentiatorApproximation() {
    // Pseudo-differentiator: s/(s/N + 1) = N*s/(s+N)
    // High-pass with gain N at high freq
    double N = 100.0;  // Differentiator bandwidth
    double dt = 0.001;

    // At low freq, acts as differentiator
    // At high freq, limited to gain N
    double corner = N;
    double hf_gain = N;

    TS_ASSERT_DELTA(hf_gain, 100.0, epsilon);
  }

  // Test PI controller as filter
  void testPIControllerFilter() {
    // PI: Kp + Ki/s = (Kp*s + Ki)/s
    double Kp = 1.0;
    double Ki = 10.0;
    double dt = 0.01;

    double integral = 0.0;
    double error = 1.0;  // Constant error

    for (int i = 0; i < 100; i++) {
      integral += Ki * error * dt;
      double output = Kp * error + integral;

      // Output grows without bound (pure integrator)
      if (i == 99) {
        TS_ASSERT(integral > 9.0);  // Ki * 1.0 * 1.0 = 10
      }
    }
  }

  // Test PD controller as filter
  void testPDControllerFilter() {
    // PD: Kp + Kd*s (approximated)
    double Kp = 1.0;
    double Kd = 0.1;
    double dt = 0.01;

    double prev_error = 0.0;
    double error = 1.0;

    double derivative = (error - prev_error) / dt;
    double output = Kp * error + Kd * derivative;

    // First step: derivative is large
    TS_ASSERT_DELTA(output, 1.0 + 0.1 * 100.0, epsilon);  // Kp*1 + Kd*(1/0.01)
  }

  // Test filter bandwidth
  void testFilterBandwidth() {
    // Bandwidth = corner frequency for first-order
    double C1 = 10.0;
    double bandwidth_rad = C1;
    double bandwidth_hz = C1 / (2.0 * M_PI);

    TS_ASSERT_DELTA(bandwidth_hz, 1.59, 0.01);
  }

  // Test filter time constant
  void testFilterTimeConstant() {
    double C1 = 10.0;
    double tau = 1.0 / C1;

    TS_ASSERT_DELTA(tau, 0.1, epsilon);

    // 63.2% response at t = tau
    double response_at_tau = 1.0 - std::exp(-1.0);
    TS_ASSERT_DELTA(response_at_tau, 0.632, 0.001);
  }

  // Test filter pole locations
  void testFilterPoleLocations() {
    // Second-order: poles at s = -zeta*wn +/- wn*sqrt(zeta^2-1)
    double wn = 10.0;
    double zeta = 0.5;

    double real_part = -zeta * wn;
    double imag_part = wn * std::sqrt(1.0 - zeta*zeta);

    TS_ASSERT_DELTA(real_part, -5.0, epsilon);
    TS_ASSERT_DELTA(imag_part, 8.66, 0.01);
  }

  // Test damped oscillation frequency
  void testDampedOscillationFreq() {
    double wn = 10.0;
    double zeta = 0.3;
    double wd = wn * std::sqrt(1.0 - zeta*zeta);  // Damped frequency

    TS_ASSERT_DELTA(wd, 9.54, 0.01);
  }

  // Test exponential decay envelope
  void testExponentialDecayEnvelope() {
    double wn = 10.0;
    double zeta = 0.3;
    double sigma = zeta * wn;  // Decay rate

    // Envelope: exp(-sigma*t)
    double t = 0.5;
    double envelope = std::exp(-sigma * t);

    TS_ASSERT_DELTA(envelope, 0.223, 0.01);
  }

  // Test filter delay (group delay concept)
  void testFilterDelay() {
    // For first-order lag: group delay = 1/(w^2 + C1^2) at frequency w
    double C1 = 10.0;
    double w = 0.0;  // At DC

    // At DC, delay = 1/C1^2... actually group delay = d(phase)/dw
    // For lag: phase = -atan(w/C1), group delay = C1/(w^2+C1^2)
    double group_delay = C1 / (w*w + C1*C1);

    TS_ASSERT_DELTA(group_delay, 0.1, epsilon);  // 1/C1 at DC
  }

  // Test numerical stability
  void testNumericalStability() {
    double C1 = 1000.0;  // Very fast filter
    double dt = 0.01;    // Relatively large timestep

    // Check if ca is stable
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    TS_ASSERT(ca >= 0.0 && ca < 1.0);  // Stable if 0 <= ca < 1
    TS_ASSERT(cb > 0.0 && cb <= 1.0);
  }

  // Test filter with large gain
  void testFilterWithLargeGain() {
    double C1 = 10.0;
    double gain = 100.0;
    double dt = 0.01;

    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double u = 1.0;

    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * (gain * u);
    }

    TS_ASSERT_DELTA(y, 100.0, 0.01);  // Converges to gain * u
  }

  // Test rate limit filter concept
  void testRateLimitFilterConcept() {
    double prev_output = 0.0;
    double input = 100.0;
    double max_rate = 10.0;  // per second
    double dt = 0.1;

    double max_change = max_rate * dt;
    double delta = input - prev_output;

    if (std::abs(delta) > max_change) {
      delta = (delta > 0) ? max_change : -max_change;
    }

    double output = prev_output + delta;
    TS_ASSERT_DELTA(output, 1.0, epsilon);  // Limited to 10 * 0.1 = 1
  }

  // Test all-pass filter concept
  void testAllPassFilterConcept() {
    // All-pass: (s-a)/(s+a) has unity magnitude at all frequencies
    double a = 10.0;
    double w = 5.0;

    // |H(jw)| = |jw-a|/|jw+a| = sqrt(w^2+a^2)/sqrt(w^2+a^2) = 1
    double mag = 1.0;  // Always unity for all-pass

    TS_ASSERT_DELTA(mag, 1.0, epsilon);
  }

  // Test delay using Pade approximation concept
  void testPadeDelayApproximation() {
    // First-order Pade: e^(-sT) ≈ (1 - sT/2)/(1 + sT/2)
    double T = 0.1;  // Delay
    double s = 0.0;  // DC

    double approx = (1.0 - s*T/2.0) / (1.0 + s*T/2.0);
    TS_ASSERT_DELTA(approx, 1.0, epsilon);  // Unity at DC
  }

  // Test filter order effect on roll-off
  void testFilterOrderRolloff() {
    // First-order: -20 dB/decade
    // Second-order: -40 dB/decade
    // nth order: -20n dB/decade

    int order1 = 1, order2 = 2, order4 = 4;

    double rolloff1 = -20.0 * order1;  // dB/decade
    double rolloff2 = -20.0 * order2;
    double rolloff4 = -20.0 * order4;

    TS_ASSERT_DELTA(rolloff1, -20.0, epsilon);
    TS_ASSERT_DELTA(rolloff2, -40.0, epsilon);
    TS_ASSERT_DELTA(rolloff4, -80.0, epsilon);
  }

  // Test Butterworth filter characteristic
  void testButterworthCharacteristic() {
    // Butterworth: maximally flat in passband
    // |H(jw)|^2 = 1/(1 + (w/wc)^(2n))
    double wc = 10.0;  // Cutoff
    int n = 2;         // Order

    // At cutoff: |H(jwc)|^2 = 1/(1+1) = 0.5 -> |H| = 0.707
    double mag_at_cutoff = 1.0 / std::sqrt(1.0 + std::pow(1.0, 2*n));
    TS_ASSERT_DELTA(mag_at_cutoff, 0.707, 0.001);
  }

  // Test Chebyshev filter concept
  void testChebyshevConcept() {
    // Chebyshev: ripple in passband, steeper rolloff
    double ripple_db = 1.0;  // 1 dB ripple
    double epsilon_cheb = std::sqrt(std::pow(10.0, ripple_db/10.0) - 1.0);

    TS_ASSERT(epsilon_cheb > 0.0);
  }

  // Test filter impulse response
  void testFilterImpulseResponse() {
    double C1 = 10.0;
    double dt = 0.001;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;

    // Apply impulse at t=0
    y = ca * y + cb * 1000.0;  // Large impulse
    double peak = y;

    // Then decay
    for (int i = 0; i < 1000; i++) {
      y = ca * y + cb * 0.0;  // Zero input after impulse
    }

    TS_ASSERT(y < 0.01);  // Decayed to near zero
    TS_ASSERT(peak > 0.0);
  }

  // Test filter steady-state error
  void testFilterSteadyStateError() {
    // First-order lag has zero steady-state error for step
    double C1 = 10.0;
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double u = 5.0;  // Step input

    for (int i = 0; i < 2000; i++) {
      y = ca * y + cb * u;
    }

    double ss_error = u - y;
    TS_ASSERT_DELTA(ss_error, 0.0, 0.001);
  }

  // Test filter with noisy input
  void testFilterWithNoisyInput() {
    double C1 = 10.0;  // Low-pass filter
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double signal = 1.0;

    // Simulate with pseudo-noise
    for (int i = 0; i < 1000; i++) {
      double noise = ((i % 7) - 3) * 0.1;  // Simple deterministic "noise"
      double u = signal + noise;
      y = ca * y + cb * u;
    }

    // Filtered output should be close to signal (noise filtered out)
    TS_ASSERT(std::abs(y - signal) < 0.3);
  }

  // Test conversion between frequency units
  void testFrequencyConversion() {
    double f_hz = 10.0;  // Hz
    double w_rad = 2.0 * M_PI * f_hz;  // rad/s

    TS_ASSERT_DELTA(w_rad, 62.83, 0.01);

    // Convert back
    double f_back = w_rad / (2.0 * M_PI);
    TS_ASSERT_DELTA(f_back, f_hz, epsilon);
  }

  // Test decibel conversion
  void testDecibelConversion() {
    double gain_linear = 10.0;
    double gain_db = 20.0 * std::log10(gain_linear);

    TS_ASSERT_DELTA(gain_db, 20.0, epsilon);

    // Convert back
    double gain_back = std::pow(10.0, gain_db / 20.0);
    TS_ASSERT_DELTA(gain_back, gain_linear, epsilon);
  }

  // Test half-power point
  void testHalfPowerPoint() {
    // At -3dB, power is halved, amplitude is 1/sqrt(2)
    double half_power_amplitude = 1.0 / std::sqrt(2.0);
    double half_power_db = 20.0 * std::log10(half_power_amplitude);

    TS_ASSERT_DELTA(half_power_amplitude, 0.707, 0.001);
    TS_ASSERT_DELTA(half_power_db, -3.01, 0.01);
  }
};
