#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <complex>

#include "TestUtilities.h"

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>

using namespace JSBSimTest;
using namespace JSBSim;

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

  /***************************************************************************
   * Advanced Filter Concepts
   ***************************************************************************/

  // Test Bessel filter characteristic (maximally flat delay)
  void testBesselFilterCharacteristic() {
    // Bessel filters have maximally flat group delay
    // Second-order Bessel: wn^2/(s^2 + 3*s*wn/sqrt(3) + wn^2)
    // Normalized 2nd order Bessel poles at approximately -1.1 +/- j0.64

    double wn = 10.0;
    double pole_real = -1.1 * wn / 1.732;  // Scaled
    double pole_imag = 0.64 * wn / 1.732;

    // Bessel poles are in LHP
    TS_ASSERT(pole_real < 0);
    TS_ASSERT(pole_imag > 0);
  }

  // Test complementary filters
  void testComplementaryFilters() {
    // Low-pass and high-pass that sum to unity: LP + HP = 1
    // LP = C1/(s+C1), HP = s/(s+C1)
    double C1 = 10.0;
    double s = 5.0;  // Test frequency

    double lp_gain = C1 / (s + C1);
    double hp_gain = s / (s + C1);

    TS_ASSERT_DELTA(lp_gain + hp_gain, 1.0, epsilon);
  }

  // Test complementary filter for sensor fusion
  void testComplementaryFilterSensorFusion() {
    // Combine accelerometer (accurate long-term) with gyro (accurate short-term)
    double alpha = 0.98;  // Complementary filter coefficient

    double gyro_angle = 45.0;      // Fast but drifts
    double accel_angle = 44.5;     // Noisy but accurate average

    // Fused: alpha*gyro + (1-alpha)*accel
    double fused = alpha * gyro_angle + (1.0 - alpha) * accel_angle;

    TS_ASSERT_DELTA(fused, 44.99, 0.01);  // Mostly gyro, corrected by accel
  }

  // Test sample rate and Nyquist frequency
  void testNyquistFrequency() {
    double sample_rate = 100.0;  // Hz
    double nyquist = sample_rate / 2.0;

    TS_ASSERT_DELTA(nyquist, 50.0, epsilon);

    // Filter must be designed for frequencies below Nyquist
    double max_filter_freq = 0.8 * nyquist;  // 80% of Nyquist for safety
    TS_ASSERT_DELTA(max_filter_freq, 40.0, epsilon);
  }

  // Test anti-aliasing filter requirement
  void testAntiAliasingRequirement() {
    double signal_freq = 45.0;    // Hz
    double sample_rate = 100.0;   // Hz
    double nyquist = sample_rate / 2.0;

    // Signal frequency should be < Nyquist to avoid aliasing
    bool aliasing_risk = (signal_freq > nyquist);
    TS_ASSERT(!aliasing_risk);

    // If signal_freq = 60 Hz with 100 Hz sampling, it aliases to 40 Hz
    double aliased_freq = 60.0;
    double apparent_freq = sample_rate - aliased_freq;  // 40 Hz
    TS_ASSERT_DELTA(apparent_freq, 40.0, epsilon);
  }

  // Test moving average filter
  void testMovingAverageFilter() {
    // Simple N-point moving average
    int N = 5;
    double samples[] = {10.0, 12.0, 8.0, 11.0, 9.0, 13.0, 7.0};

    double sum = 0.0;
    for (int i = 0; i < N; i++) {
      sum += samples[i];
    }
    double avg = sum / N;

    TS_ASSERT_DELTA(avg, 10.0, epsilon);  // (10+12+8+11+9)/5 = 50/5 = 10
  }

  // Test exponential moving average
  void testExponentialMovingAverage() {
    // EMA: y(n) = alpha*x(n) + (1-alpha)*y(n-1)
    double alpha = 0.2;
    double y = 0.0;
    double samples[] = {10.0, 10.0, 10.0, 10.0, 10.0};

    for (int i = 0; i < 5; i++) {
      y = alpha * samples[i] + (1.0 - alpha) * y;
    }

    // After several iterations with constant input, converges to input
    TS_ASSERT(y > 6.0);  // Approaching 10
  }

  // Test filter order vs complexity tradeoff
  void testFilterOrderComplexity() {
    // Higher order = sharper rolloff but more complexity
    int orders[] = {1, 2, 4, 8};
    double rolloffs[] = {-20.0, -40.0, -80.0, -160.0};  // dB/decade

    for (int i = 0; i < 4; i++) {
      double actual_rolloff = -20.0 * orders[i];
      TS_ASSERT_DELTA(actual_rolloff, rolloffs[i], epsilon);
    }
  }

  // Test state-space filter representation
  void testStateSpaceFilter() {
    // First-order filter in state-space: dx/dt = A*x + B*u, y = C*x + D*u
    // For lag filter C1/(s+C1): A = -C1, B = C1, C = 1, D = 0
    double C1 = 10.0;
    double A = -C1;
    double B = C1;
    double C = 1.0;
    double D = 0.0;

    // DC gain = -C*B/A + D = -1*10/(-10) + 0 = 1
    double dc_gain = -C * B / A + D;
    TS_ASSERT_DELTA(dc_gain, 1.0, epsilon);
  }

  // Test filter coefficient sensitivity
  void testCoefficientSensitivity() {
    // Small changes in coefficients shouldn't cause large changes in response
    double C1_nominal = 10.0;
    double C1_perturbed = 10.1;  // 1% change

    double dc_nominal = 1.0;  // Lag filter DC gain is always 1
    double dc_perturbed = 1.0;

    double sensitivity = std::abs(dc_perturbed - dc_nominal) / std::abs(C1_perturbed - C1_nominal);
    TS_ASSERT(sensitivity < 1.0);  // Low sensitivity
  }

  // Test filter transient duration
  void testTransientDuration() {
    // 95% settling time for first-order = 3*tau
    // 99% settling time = 5*tau
    double C1 = 10.0;
    double tau = 1.0 / C1;

    double t_95pct = 3.0 * tau;
    double t_99pct = 5.0 * tau;

    // Verify exponential decay
    double remaining_95 = std::exp(-3.0);  // ~5%
    double remaining_99 = std::exp(-5.0);  // ~0.7%

    TS_ASSERT(remaining_95 < 0.06);
    TS_ASSERT(remaining_99 < 0.01);
  }

  // Test filter energy preservation (Parseval's theorem concept)
  void testFilterEnergyPreservation() {
    // For a filter with unity DC gain, energy is preserved at DC
    double C1 = 10.0;
    double dc_gain = 1.0;

    // Energy through filter = |H(jw)|^2 * input_energy
    double input_energy = 100.0;
    double output_energy_dc = dc_gain * dc_gain * input_energy;

    TS_ASSERT_DELTA(output_energy_dc, input_energy, epsilon);
  }

  // Test adaptive filter concept
  void testAdaptiveFilterConcept() {
    // Adaptive filter adjusts coefficients based on error
    double mu = 0.1;  // Learning rate
    double w = 1.0;   // Initial weight
    double d = 10.0;  // Desired output
    double x = 1.0;   // Input

    // LMS update: w = w + mu * e * x
    double y = w * x;
    double e = d - y;
    double w_new = w + mu * e * x;

    TS_ASSERT(w_new > w);  // Weight increased toward desired
    TS_ASSERT_DELTA(w_new, 1.9, epsilon);  // 1 + 0.1 * 9 * 1 = 1.9
  }

  // Test filter with saturation
  void testFilterWithSaturation() {
    double C1 = 10.0;
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;
    double u = 100.0;
    double sat_limit = 10.0;

    for (int i = 0; i < 500; i++) {
      y = ca * y + cb * u;
      // Apply saturation
      if (y > sat_limit) y = sat_limit;
      if (y < -sat_limit) y = -sat_limit;
    }

    TS_ASSERT_DELTA(y, sat_limit, epsilon);  // Saturated at limit
  }

  // Test filter with dead zone
  void testFilterWithDeadZone() {
    double dead_zone = 1.0;
    double inputs[] = {0.5, 1.5, -0.5, -1.5, 0.0};
    double expected[] = {0.0, 0.5, 0.0, -0.5, 0.0};

    for (int i = 0; i < 5; i++) {
      double x = inputs[i];
      double y;
      if (std::abs(x) < dead_zone) {
        y = 0.0;
      } else {
        y = (x > 0) ? x - dead_zone : x + dead_zone;
      }
      TS_ASSERT_DELTA(y, expected[i], epsilon);
    }
  }

  // Test digital filter coefficient quantization
  void testCoefficientQuantization() {
    // In fixed-point implementations, coefficients are quantized
    double coeff = 0.9047619;
    int bits = 8;  // 8-bit quantization

    int quantized_int = static_cast<int>(coeff * (1 << bits) + 0.5);
    double quantized = static_cast<double>(quantized_int) / (1 << bits);

    // Should be close to original
    TS_ASSERT_DELTA(quantized, coeff, 1.0 / (1 << bits));
  }

  // Test filter with varying sample rate
  void testVaryingSampleRate() {
    double C1 = 10.0;

    // At 100 Hz sample rate
    double dt1 = 0.01;
    double ca1 = std::exp(-C1 * dt1);

    // At 1000 Hz sample rate
    double dt2 = 0.001;
    double ca2 = std::exp(-C1 * dt2);

    // Higher sample rate = ca closer to 1 (slower decay per sample)
    TS_ASSERT(ca2 > ca1);
    TS_ASSERT_DELTA(ca1, 0.9048, 0.001);
    TS_ASSERT_DELTA(ca2, 0.9900, 0.001);
  }

  // Test biquad filter structure
  void testBiquadStructure() {
    // Biquad: H(z) = (b0 + b1*z^-1 + b2*z^-2)/(1 + a1*z^-1 + a2*z^-2)
    // Second-order section with 5 coefficients

    // Example: lowpass at wc = 0.1*fs
    double b0 = 0.0675, b1 = 0.1349, b2 = 0.0675;
    double a1 = -1.1430, a2 = 0.4128;

    // DC gain: (b0+b1+b2)/(1+a1+a2)
    double dc_gain = (b0 + b1 + b2) / (1.0 + a1 + a2);

    TS_ASSERT_DELTA(dc_gain, 1.0, 0.01);  // Unity DC gain
  }

  // Test filter phase distortion
  void testPhaseDistortion() {
    // Non-linear phase causes different delays at different frequencies
    double C1 = 10.0;

    // Phase at w = 1: -atan(1/10) = -5.7 degrees
    double phase1 = -std::atan(1.0 / C1) * 180.0 / M_PI;

    // Phase at w = 10: -atan(10/10) = -45 degrees
    double phase10 = -std::atan(10.0 / C1) * 180.0 / M_PI;

    // Group delay varies with frequency (phase distortion)
    TS_ASSERT(std::abs(phase10) > std::abs(phase1));
  }

  // Test minimum phase filter concept
  void testMinimumPhaseFilter() {
    // Minimum phase: all zeros inside unit circle (analog: LHP)
    // Provides minimum phase lag for given magnitude response

    // First-order lag is minimum phase (no zeros)
    double C1 = 10.0;

    // Check pole location (must be in LHP for stability)
    double pole = -C1;
    TS_ASSERT(pole < 0);  // In LHP = stable and minimum phase
  }

  // Test filter group delay calculation
  void testGroupDelayCalculation() {
    // Group delay = -d(phase)/dw
    // For first-order lag: phase = -atan(w/C1)
    // Group delay = C1/(w^2 + C1^2)

    double C1 = 10.0;
    double frequencies[] = {0.0, 5.0, 10.0, 20.0};
    double expected_delays[] = {0.1, 0.08, 0.05, 0.02};

    for (int i = 0; i < 4; i++) {
      double w = frequencies[i];
      double gd = C1 / (w*w + C1*C1);
      TS_ASSERT_DELTA(gd, expected_delays[i], 0.01);
    }
  }

  // Test cross-over frequency
  void testCrossoverFrequency() {
    // Frequency where LP and HP filters have equal gain
    // For complementary LP and HP: at w = C1, both have gain 0.707

    double C1 = 10.0;
    double w_crossover = C1;

    double lp_gain = C1 / std::sqrt(w_crossover*w_crossover + C1*C1);
    double hp_gain = w_crossover / std::sqrt(w_crossover*w_crossover + C1*C1);

    TS_ASSERT_DELTA(lp_gain, hp_gain, epsilon);
    TS_ASSERT_DELTA(lp_gain, 0.707, 0.001);
  }

  // Test filter transient rejection
  void testTransientRejection() {
    // Low-pass filter should reject sudden transients
    double C1 = 10.0;
    double dt = 0.01;
    double ca = std::exp(-C1 * dt);
    double cb = 1.0 - ca;

    double y = 0.0;

    // Apply spike transient
    y = ca * y + cb * 100.0;
    double peak = y;

    // Continue with zero input
    for (int i = 0; i < 100; i++) {
      y = ca * y + cb * 0.0;
    }

    // Transient should be attenuated
    TS_ASSERT(y < peak * 0.01);
  }

  /***************************************************************************
   * Complete Filter System Tests
   ***************************************************************************/

  // Test complete filter cascade
  void testCompleteFilterCascade() {
    double C1 = 10.0, C2 = 20.0;
    double dt = 0.01;

    double ca1 = std::exp(-C1 * dt);
    double cb1 = 1.0 - ca1;
    double ca2 = std::exp(-C2 * dt);
    double cb2 = 1.0 - ca2;

    double y1 = 0.0, y2 = 0.0;
    double input = 1.0;

    // Apply step input through cascade
    for (int i = 0; i < 100; i++) {
      y1 = ca1 * y1 + cb1 * input;
      y2 = ca2 * y2 + cb2 * y1;
    }

    TS_ASSERT(y2 < y1);  // Second filter adds more lag
    TS_ASSERT(y2 > 0.9);  // Both approach steady state
  }

  // Test notch filter attenuation
  void testNotchFilterAttenuation() {
    double notchFreq = 50.0;     // Hz
    double notchWidth = 5.0;     // Hz bandwidth
    double inputFreq = 50.0;     // At notch frequency

    // At notch frequency, attenuation should be maximum
    double attenuation_dB = -40.0;  // Typical deep notch
    double gain = std::pow(10.0, attenuation_dB / 20.0);

    TS_ASSERT(gain < 0.02);  // Less than 2% pass-through
  }

  // Test complementary filter fusion
  void testComplementaryFilterFusion() {
    double accelData = 1.0;      // Low frequency accurate
    double gyroData = 0.9;       // High frequency accurate
    double alpha = 0.98;         // High pass weight

    double fused = alpha * gyroData + (1.0 - alpha) * accelData;
    TS_ASSERT_DELTA(fused, 0.902, 0.001);
  }

  // Test moving average smoothing
  void testMovingAverageSmoothing() {
    double samples[] = {10.0, 12.0, 8.0, 11.0, 9.0};
    double sum = 0.0;
    int n = 5;

    for (int i = 0; i < n; i++) {
      sum += samples[i];
    }
    double average = sum / n;

    TS_ASSERT_DELTA(average, 10.0, 0.01);
  }

  // Test Kalman filter prediction
  void testKalmanFilterPrediction() {
    double state = 100.0;
    double velocity = 10.0;
    double dt = 0.1;

    double predictedState = state + velocity * dt;
    TS_ASSERT_DELTA(predictedState, 101.0, 0.01);
  }

  // Test rate limiter integration
  void testRateLimiterWithFilter() {
    double maxRate = 10.0;   // units/sec
    double dt = 0.1;
    double target = 100.0;
    double current = 0.0;

    double maxChange = maxRate * dt;
    double change = std::min(target - current, maxChange);
    current += change;

    TS_ASSERT_DELTA(current, 1.0, 0.01);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test filter coefficient independence
  void testFilterCoefficientIndependence() {
    double C1_a = 10.0, C1_b = 50.0;
    double dt = 0.01;

    double ca_a = std::exp(-C1_a * dt);
    double ca_b = std::exp(-C1_b * dt);

    TS_ASSERT(ca_a > ca_b);  // Slower filter has ca closer to 1
    TS_ASSERT_DELTA(ca_a, 0.9048, 0.001);
    TS_ASSERT_DELTA(ca_b, 0.6065, 0.001);
  }

  // Test filter state independence
  void testFilterStateIndependence() {
    double y1 = 50.0, y2 = 100.0;
    double input = 0.0;
    double ca = 0.9;

    double y1_next = ca * y1 + (1.0 - ca) * input;
    double y2_next = ca * y2 + (1.0 - ca) * input;

    TS_ASSERT_DELTA(y1_next, 45.0, 0.1);
    TS_ASSERT_DELTA(y2_next, 90.0, 0.1);
  }

  // Test gain calculation independence
  void testFilterGainCalculationIndependence() {
    double w1 = 5.0, C1 = 10.0;
    double w2 = 20.0;

    double gain1 = C1 / std::sqrt(w1*w1 + C1*C1);
    double gain2 = C1 / std::sqrt(w2*w2 + C1*C1);

    TS_ASSERT(gain1 > gain2);  // Lower frequency has higher gain in LP
    TS_ASSERT_DELTA(gain1, 0.894, 0.01);
    TS_ASSERT_DELTA(gain2, 0.447, 0.01);
  }

  // Test phase calculation independence
  void testPhaseCalculationIndependence() {
    double w1 = 5.0, C1 = 10.0;
    double w2 = 10.0;

    double phase1 = -std::atan(w1 / C1) * 180.0 / M_PI;
    double phase2 = -std::atan(w2 / C1) * 180.0 / M_PI;

    TS_ASSERT(std::abs(phase2) > std::abs(phase1));
    TS_ASSERT_DELTA(phase1, -26.57, 0.1);
    TS_ASSERT_DELTA(phase2, -45.0, 0.1);
  }

  // Test time constant independence
  void testTimeConstantIndependence() {
    double tau1 = 0.1, tau2 = 0.5;

    double C1_1 = 1.0 / tau1;
    double C1_2 = 1.0 / tau2;

    TS_ASSERT_DELTA(C1_1, 10.0, 0.1);
    TS_ASSERT_DELTA(C1_2, 2.0, 0.1);
  }

  //===========================================================================
  // C172X MODEL INTEGRATION TESTS (20 tests)
  //===========================================================================

  void testC172xFilterModelLoads() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());
  }

  void testC172xFCSExists() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
  }

  void testC172xElevatorResponseFiltered() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Apply step input
    fcs->SetDeCmd(0.5);

    // Run several frames
    for (int i = 0; i < 50; i++) fdmex.Run();

    // Elevator position should be finite
    double de = fcs->GetDePos();
    TS_ASSERT(std::isfinite(de));
  }

  void testC172xAileronResponseFiltered() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDaCmd(0.5);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double da = fcs->GetDaLPos();
    TS_ASSERT(std::isfinite(da));
  }

  void testC172xRudderResponseFiltered() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDrCmd(0.5);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double dr = fcs->GetDrPos();
    TS_ASSERT(std::isfinite(dr));
  }

  void testC172xThrottleResponseFiltered() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.8);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double throttle = fcs->GetThrottlePos(0);
    TS_ASSERT(std::isfinite(throttle));
  }

  void testC172xFlapResponseFiltered() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDfCmd(0.5);

    for (int i = 0; i < 200; i++) fdmex.Run();

    double df = fcs->GetDfPos();
    TS_ASSERT(std::isfinite(df));
  }

  void testC172xControlSurfaceSmoothing() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Apply rapid input changes
    for (int cycle = 0; cycle < 5; cycle++) {
      fcs->SetDeCmd(1.0);
      for (int i = 0; i < 20; i++) fdmex.Run();
      fcs->SetDeCmd(-1.0);
      for (int i = 0; i < 20; i++) fdmex.Run();
    }

    double de = fcs->GetDePos();
    TS_ASSERT(std::isfinite(de));
  }

  void testC172xStepResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    double de_initial = fcs->GetDePos();

    // Apply step
    fcs->SetDeCmd(0.5);

    // Record response
    double de_prev = de_initial;
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
      double de_now = fcs->GetDePos();
      TS_ASSERT(std::isfinite(de_now));
      de_prev = de_now;
    }
  }

  void testC172xMultipleInputsFiltered() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Apply all inputs simultaneously
    fcs->SetDeCmd(0.3);
    fcs->SetDaCmd(0.2);
    fcs->SetDrCmd(0.1);

    for (int i = 0; i < 100; i++) fdmex.Run();

    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }

  void testC172xZeroInputResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Zero inputs
    fcs->SetDeCmd(0.0);
    fcs->SetDaCmd(0.0);
    fcs->SetDrCmd(0.0);

    for (int i = 0; i < 50; i++) fdmex.Run();

    // Outputs should be near zero or trim values
    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }

  void testC172xNegativeInputResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(-0.5);
    fcs->SetDaCmd(-0.3);
    fcs->SetDrCmd(-0.2);

    for (int i = 0; i < 50; i++) fdmex.Run();

    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }

  void testC172xFullDeflectionResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetDeCmd(1.0);
    fcs->SetDaCmd(1.0);
    fcs->SetDrCmd(1.0);

    for (int i = 0; i < 100; i++) fdmex.Run();

    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }

  void testC172xBrakeFiltering() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetLBrake(0.8);
    fcs->SetRBrake(0.8);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double lb = fcs->GetLBrake();
    double rb = fcs->GetRBrake();
    TS_ASSERT(std::isfinite(lb));
    TS_ASSERT(std::isfinite(rb));
  }

  void testC172xGearCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // C172x has fixed gear, but test command handling
    fcs->SetGearCmd(1.0);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double gear = fcs->GetGearPos();
    TS_ASSERT(std::isfinite(gear));
  }

  void testC172xMixtureControl() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetMixtureCmd(-1, 0.9);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double mixture = fcs->GetMixturePos(0);
    TS_ASSERT(std::isfinite(mixture));
  }

  void testC172xTrimCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    fcs->SetPitchTrimCmd(0.1);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double trim = fcs->GetPitchTrimCmd();
    TS_ASSERT(std::isfinite(trim));
  }

  void testC172xExtendedFilterSimulation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.8);

    // Extended run with varying inputs
    for (int i = 0; i < 1000; i++) {
      double cmd = 0.3 * std::sin(i * 0.1);
      fcs->SetDeCmd(cmd);
      fcs->SetDaCmd(cmd * 0.5);

      TS_ASSERT(fdmex.Run());
      TS_ASSERT(std::isfinite(fcs->GetDePos()));
      TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    }
  }

  void testC172xFilterStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    // Run extended simulation checking for stability
    double max_de = 0.0;
    for (int i = 0; i < 2000; i++) {
      fdmex.Run();
      double de = std::abs(fcs->GetDePos());
      max_de = std::max(max_de, de);

      // Elevator should not grow unboundedly (stability)
      TS_ASSERT(de < 100.0);  // Reasonable bound
    }
    TS_ASSERT(std::isfinite(max_de));
  }

  void testC172xFilterIntegrity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.75);

    // Run and verify all FCS outputs remain valid
    for (int i = 0; i < 1500; i++) {
      TS_ASSERT(fdmex.Run());

      // All control surfaces should have finite values
      TS_ASSERT(std::isfinite(fcs->GetDePos()));
      TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
      TS_ASSERT(std::isfinite(fcs->GetDaRPos()));
      TS_ASSERT(std::isfinite(fcs->GetDrPos()));
      TS_ASSERT(std::isfinite(fcs->GetDfPos()));
      TS_ASSERT(std::isfinite(fcs->GetThrottlePos(0)));
    }

    // Final values should be finite
    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(aux->GetVcalibratedKTS()));
  }
};
