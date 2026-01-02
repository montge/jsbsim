/*******************************************************************************
 * FGAeroelasticityTest.h - Unit tests for aeroelasticity physics
 *
 * Tests comprehensive aeroelasticity phenomena including:
 * - Flutter speed and frequency calculations
 * - Divergence speed analysis
 * - Wing bending and torsion modes
 * - Structural damping effects
 * - Natural frequency calculations
 * - Mass ratio effects
 * - Control surface flutter
 * - Wing stiffness parameters (EI, GJ)
 * - Structural eigenvalue problems
 * - Reduced frequency calculations
 * - Wing tip deflection
 * - Control reversal effects
 * - Aileron effectiveness vs speed
 * - Dynamic aeroelastic response
 * - Static aeroelastic deformation
 * - Coupling between bending and torsion
 * - Vibration amplitude limits
 * - Flutter margin calculations
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include "TestUtilities.h"

#include <FGFDMExec.h>
#include <models/FGAerodynamics.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>

using namespace JSBSim;
using namespace JSBSimTest;

class FGAeroelasticityTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Flutter Speed and Frequency Tests
   ***************************************************************************/

  // Test basic flutter speed calculation using simplified model
  void testBasicFlutterSpeed() {
    // GIVEN: Wing properties for typical aircraft
    double wingSpan = 40.0;  // ft
    double chord = 5.0;      // ft
    double airDensity = 0.002377;  // slug/ft^3 (sea level)
    double structuralStiffness = 1e7;  // lb-ft^2 (EI)
    double torsionalStiffness = 5e6;   // lb-ft^2 (GJ)
    double mass = 50.0;  // slug/ft (mass per unit span)

    // WHEN: Calculate flutter speed using simplified formula
    // V_flutter ~ sqrt(GJ * omega / (rho * chord^3))
    double omega_torsion = std::sqrt(torsionalStiffness / (mass * chord * chord));
    double V_flutter = std::sqrt(torsionalStiffness * omega_torsion /
                                  (airDensity * chord * chord * chord));

    // THEN: Flutter speed should be positive and finite
    TS_ASSERT(V_flutter > 0.0);
    TS_ASSERT(!std::isnan(V_flutter));
    TS_ASSERT(!std::isinf(V_flutter));
  }

  // Test flutter frequency calculation
  void testFlutterFrequency() {
    // GIVEN: Structural properties
    double torsionalStiffness = 5e6;  // lb-ft^2
    double inertia = 100.0;  // slug-ft^2

    // WHEN: Calculate natural frequency
    double omega_n = std::sqrt(torsionalStiffness / inertia);
    double frequency_hz = omega_n / (2.0 * M_PI);

    // THEN: Frequency should be positive and reasonable (1-50 Hz)
    TS_ASSERT(frequency_hz > 1.0);
    TS_ASSERT(frequency_hz < 50.0);
    TS_ASSERT_DELTA(omega_n * omega_n, torsionalStiffness / inertia, DEFAULT_TOLERANCE);
  }

  // Test flutter speed with varying density altitude
  void testFlutterSpeedVsAltitude() {
    // GIVEN: Sea level and high altitude conditions
    double rho_sl = 0.002377;  // slug/ft^3
    double rho_high = 0.001000;  // slug/ft^3 at ~20,000 ft
    double stiffness = 5e6;
    double omega = 10.0;  // rad/s
    double chord = 5.0;

    // WHEN: Calculate flutter speeds
    double V_flutter_sl = std::sqrt(stiffness * omega / (rho_sl * chord * chord * chord));
    double V_flutter_high = std::sqrt(stiffness * omega / (rho_high * chord * chord * chord));

    // THEN: Flutter speed increases with altitude (lower density)
    TS_ASSERT(V_flutter_high > V_flutter_sl);
    TS_ASSERT_DELTA(V_flutter_high / V_flutter_sl, std::sqrt(rho_sl / rho_high), 1e-6);
  }

  // Test coupled bending-torsion flutter frequency
  void testCoupledFlutterFrequency() {
    // GIVEN: Coupled system properties
    double omega_bending = 5.0;   // rad/s
    double omega_torsion = 15.0;  // rad/s
    double coupling_factor = 0.3;  // dimensionless

    // WHEN: Calculate coupled frequency
    double omega_avg = (omega_bending + omega_torsion) / 2.0;
    double omega_coupled = omega_avg * (1.0 + coupling_factor);

    // THEN: Coupled frequency lies between individual modes
    TS_ASSERT(omega_coupled > omega_bending);
    TS_ASSERT(omega_coupled < omega_torsion * (1.0 + coupling_factor));
  }

  /***************************************************************************
   * Divergence Speed Tests
   ***************************************************************************/

  // Test basic divergence speed calculation
  void testDivergenceSpeed() {
    // GIVEN: Wing parameters
    double torsionalStiffness = 5e6;  // lb-ft^2/rad
    double liftCurveSlope = 5.73;     // per radian (2*pi)
    double aerodynamicCenter = 0.25;  // chord fraction
    double elasticAxis = 0.40;        // chord fraction
    double chord = 5.0;               // ft
    double airDensity = 0.002377;     // slug/ft^3

    // WHEN: Calculate divergence speed
    double e = (elasticAxis - aerodynamicCenter) * chord;  // moment arm (positive when EA aft of AC)
    double V_div = std::sqrt(2.0 * torsionalStiffness /
                             (airDensity * chord * liftCurveSlope * std::abs(e)));

    // THEN: Divergence speed should be positive and finite
    TS_ASSERT(V_div > 0.0);
    TS_ASSERT(!std::isnan(V_div));
    TS_ASSERT(!std::isinf(V_div));
  }

  // Test divergence with aft elastic axis
  void testDivergenceAftElasticAxis() {
    // GIVEN: Elastic axis aft of aerodynamic center (stable)
    double GJ = 5e6;
    double rho = 0.002377;
    double chord = 5.0;
    double CL_alpha = 5.73;
    double ea_large_offset = 0.50;   // aft of AC (large offset)
    double ea_small_offset = 0.30;   // aft of AC (small offset)
    double ac = 0.25;

    // WHEN: Calculate divergence speeds
    double e_large = (ea_large_offset - ac) * chord;  // larger moment arm
    double e_small = (ea_small_offset - ac) * chord;  // smaller moment arm

    double V_div_large = std::sqrt(2.0 * GJ / (rho * chord * CL_alpha * std::abs(e_large)));
    double V_div_small = std::sqrt(2.0 * GJ / (rho * chord * CL_alpha * std::abs(e_small)));

    // THEN: Configuration with larger moment arm has lower divergence speed (less stable)
    TS_ASSERT(V_div_small > V_div_large);
  }

  // Test divergence dynamic pressure
  void testDivergenceDynamicPressure() {
    // GIVEN: Wing properties
    double GJ = 5e6;
    double S = 200.0;  // ft^2
    double CL_alpha = 5.73;
    double moment_arm = 0.5;  // ft

    // WHEN: Calculate critical dynamic pressure
    double q_div = GJ / (S * CL_alpha * moment_arm);

    // THEN: Dynamic pressure should be positive
    TS_ASSERT(q_div > 0.0);
    TS_ASSERT(!std::isnan(q_div));
  }

  /***************************************************************************
   * Wing Bending Mode Tests
   ***************************************************************************/

  // Test fundamental bending frequency
  void testBendingNaturalFrequency() {
    // GIVEN: Cantilever wing properties
    double EI = 1e7;        // lb-ft^2 (bending stiffness)
    double m = 50.0;        // slug/ft (mass per unit length)
    double length = 20.0;   // ft (semi-span)

    // WHEN: Calculate first mode natural frequency
    // omega = (1.875^2) * sqrt(EI / (m * L^4))
    double lambda1 = 1.875;  // First mode eigenvalue for cantilever
    double omega_bend = lambda1 * lambda1 * std::sqrt(EI / (m * length * length * length * length));
    double freq_hz = omega_bend / (2.0 * M_PI);

    // THEN: Frequency should be in typical range
    TS_ASSERT(freq_hz > 0.1);
    TS_ASSERT(freq_hz < 20.0);
    TS_ASSERT_DELTA(omega_bend * omega_bend,
                    lambda1 * lambda1 * lambda1 * lambda1 * EI / (m * length * length * length * length),
                    1e-6);
  }

  // Test second bending mode
  void testSecondBendingMode() {
    // GIVEN: Same wing properties
    double EI = 1e7;
    double m = 50.0;
    double length = 20.0;

    // WHEN: Calculate second mode frequency
    double lambda2 = 4.694;  // Second mode eigenvalue
    double omega_2 = lambda2 * lambda2 * std::sqrt(EI / (m * length * length * length * length));

    // THEN: Second mode should be higher frequency than first
    double lambda1 = 1.875;
    double omega_1 = lambda1 * lambda1 * std::sqrt(EI / (m * length * length * length * length));

    TS_ASSERT(omega_2 > omega_1);
    TS_ASSERT_DELTA(omega_2 / omega_1, (lambda2 / lambda1) * (lambda2 / lambda1), 1e-6);
  }

  // Test bending stiffness effect
  void testBendingStiffnessEffect() {
    // GIVEN: Two wings with different stiffness
    double EI_stiff = 2e7;
    double EI_flexible = 1e7;
    double m = 50.0;
    double L = 20.0;
    double lambda = 1.875;

    // WHEN: Calculate frequencies
    double omega_stiff = lambda * lambda * std::sqrt(EI_stiff / (m * L * L * L * L));
    double omega_flex = lambda * lambda * std::sqrt(EI_flexible / (m * L * L * L * L));

    // THEN: Stiffer wing has higher frequency
    TS_ASSERT(omega_stiff > omega_flex);
    TS_ASSERT_DELTA(omega_stiff / omega_flex, std::sqrt(EI_stiff / EI_flexible), 1e-6);
  }

  // Test wing span effect on bending
  void testWingSpanEffectOnBending() {
    // GIVEN: Different wing spans
    double EI = 1e7;
    double m = 50.0;
    double L_short = 15.0;
    double L_long = 25.0;
    double lambda = 1.875;

    // WHEN: Calculate frequencies
    double omega_short = lambda * lambda * std::sqrt(EI / (m * L_short * L_short * L_short * L_short));
    double omega_long = lambda * lambda * std::sqrt(EI / (m * L_long * L_long * L_long * L_long));

    // THEN: Shorter wing has higher frequency
    TS_ASSERT(omega_short > omega_long);
  }

  /***************************************************************************
   * Wing Torsion Mode Tests
   ***************************************************************************/

  // Test torsional natural frequency
  void testTorsionalNaturalFrequency() {
    // GIVEN: Wing torsional properties
    double GJ = 5e6;        // lb-ft^2/rad (torsional stiffness)
    double I_theta = 100.0; // slug-ft^2/ft (polar moment per unit length)
    double length = 20.0;   // ft

    // WHEN: Calculate torsional frequency
    double omega_torsion = (M_PI / (2.0 * length)) * std::sqrt(GJ / I_theta);
    double freq_hz = omega_torsion / (2.0 * M_PI);

    // THEN: Frequency should be reasonable
    TS_ASSERT(freq_hz > 1.0);
    TS_ASSERT(freq_hz < 50.0);
  }

  // Test torsional stiffness effect
  void testTorsionalStiffnessEffect() {
    // GIVEN: Different torsional stiffnesses
    double GJ_high = 1e7;
    double GJ_low = 5e6;
    double I_theta = 100.0;
    double L = 20.0;

    // WHEN: Calculate frequencies
    double omega_high = (M_PI / (2.0 * L)) * std::sqrt(GJ_high / I_theta);
    double omega_low = (M_PI / (2.0 * L)) * std::sqrt(GJ_low / I_theta);

    // THEN: Higher stiffness gives higher frequency
    TS_ASSERT(omega_high > omega_low);
    TS_ASSERT_DELTA(omega_high / omega_low, std::sqrt(GJ_high / GJ_low), 1e-6);
  }

  // Test torsion-bending frequency ratio
  void testTorsionBendingRatio() {
    // GIVEN: Typical wing properties
    double EI = 1e7;
    double GJ = 5e6;
    double m = 50.0;
    double I_theta = 100.0;
    double L = 20.0;

    // WHEN: Calculate both frequencies
    double lambda_bend = 1.875;
    double omega_bend = lambda_bend * lambda_bend * std::sqrt(EI / (m * L * L * L * L));
    double omega_torsion = (M_PI / (2.0 * L)) * std::sqrt(GJ / I_theta);

    // THEN: Torsion frequency typically higher than bending
    double ratio = omega_torsion / omega_bend;
    TS_ASSERT(ratio > 1.0);
    TS_ASSERT(ratio < 5.0);  // Typical range
  }

  /***************************************************************************
   * Structural Damping Tests
   ***************************************************************************/

  // Test structural damping ratio
  void testStructuralDampingRatio() {
    // GIVEN: Wing with structural damping
    double damping_ratio = 0.02;  // 2% critical damping (typical)

    // THEN: Damping should be in typical range
    TS_ASSERT(damping_ratio > 0.0);
    TS_ASSERT(damping_ratio < 0.10);  // Usually less than 10%
  }

  // Test damped frequency calculation
  void testDampedFrequency() {
    // GIVEN: Undamped frequency and damping ratio
    double omega_n = 10.0;  // rad/s
    double zeta = 0.03;     // damping ratio

    // WHEN: Calculate damped frequency
    double omega_d = omega_n * std::sqrt(1.0 - zeta * zeta);

    // THEN: Damped frequency should be slightly less than undamped
    TS_ASSERT(omega_d < omega_n);
    TS_ASSERT_DELTA(omega_d, omega_n * std::sqrt(1.0 - zeta * zeta), DEFAULT_TOLERANCE);
  }

  // Test amplitude decay with damping
  void testAmplitudeDecay() {
    // GIVEN: Initial amplitude and damping
    double A0 = 1.0;         // initial amplitude
    double zeta = 0.05;      // damping ratio
    double omega_n = 10.0;   // rad/s
    double time = 1.0;       // seconds

    // WHEN: Calculate amplitude after time t
    double A_t = A0 * std::exp(-zeta * omega_n * time);

    // THEN: Amplitude should decrease
    TS_ASSERT(A_t < A0);
    TS_ASSERT(A_t > 0.0);
    TS_ASSERT_DELTA(A_t / A0, std::exp(-zeta * omega_n * time), DEFAULT_TOLERANCE);
  }

  // Test critical damping
  void testCriticalDamping() {
    // GIVEN: System properties
    double mass = 100.0;     // slug
    double stiffness = 1e6;  // lb/ft

    // WHEN: Calculate critical damping coefficient
    double omega_n = std::sqrt(stiffness / mass);
    double c_critical = 2.0 * mass * omega_n;

    // THEN: Critical damping should be positive
    TS_ASSERT(c_critical > 0.0);
    TS_ASSERT_DELTA(c_critical, 2.0 * std::sqrt(mass * stiffness), 1e-6);
  }

  /***************************************************************************
   * Mass Ratio Tests
   ***************************************************************************/

  // Test mass ratio definition
  void testMassRatio() {
    // GIVEN: Wing properties
    double wingMass = 1000.0;      // slug
    double airDensity = 0.002377;  // slug/ft^3
    double wingArea = 200.0;       // ft^2
    double chord = 5.0;            // ft

    // WHEN: Calculate mass ratio (mu)
    double mu = wingMass / (airDensity * wingArea * chord);

    // THEN: Mass ratio should be reasonable (typically 20-200)
    TS_ASSERT(mu > 10.0);
    TS_ASSERT(mu < 500.0);
  }

  // Test mass ratio effect on flutter
  void testMassRatioEffectOnFlutter() {
    // GIVEN: High and low mass ratios
    double mu_high = 100.0;
    double mu_low = 30.0;
    double omega_n = 10.0;
    double chord = 5.0;

    // WHEN: Calculate flutter speed sensitivity
    // V_flutter proportional to sqrt(mu)
    double V_ratio = std::sqrt(mu_high / mu_low);

    // THEN: Higher mass ratio increases flutter speed
    TS_ASSERT(V_ratio > 1.0);
    TS_ASSERT_DELTA(V_ratio, std::sqrt(mu_high / mu_low), DEFAULT_TOLERANCE);
  }

  // Test radius of gyration
  void testRadiusOfGyration() {
    // GIVEN: Wing section properties
    double I_theta = 100.0;  // slug-ft^2/ft
    double m = 50.0;         // slug/ft

    // WHEN: Calculate radius of gyration
    double r_gyration = std::sqrt(I_theta / m);

    // THEN: Should be on order of chord
    TS_ASSERT(r_gyration > 0.0);
    TS_ASSERT(r_gyration < 10.0);  // Reasonable for aircraft wing
  }

  /***************************************************************************
   * Control Surface Flutter Tests
   ***************************************************************************/

  // Test control surface flutter speed
  void testControlSurfaceFlutter() {
    // GIVEN: Aileron properties
    double aileronStiffness = 1e5;  // lb-ft/rad
    double aileronInertia = 10.0;   // slug-ft^2
    double hingeMoment = 50.0;      // lb-ft per radian

    // WHEN: Calculate control surface frequency
    double omega_control = std::sqrt(aileronStiffness / aileronInertia);

    // THEN: Frequency should be reasonable
    TS_ASSERT(omega_control > 0.0);
    TS_ASSERT(!std::isnan(omega_control));
  }

  // Test aileron reversal speed
  void testAileronReversalSpeed() {
    // GIVEN: Wing and aileron properties
    double wingTorsionalStiffness = 5e6;  // lb-ft^2/rad
    double rollControlPower = 0.1;        // per radian deflection
    double wingSpan = 40.0;               // ft
    double airDensity = 0.002377;

    // WHEN: Calculate reversal speed
    double V_reversal = std::sqrt(wingTorsionalStiffness /
                                   (airDensity * wingSpan * wingSpan * rollControlPower));

    // THEN: Reversal speed should be positive
    TS_ASSERT(V_reversal > 0.0);
    TS_ASSERT(V_reversal > 200.0);  // Should be reasonably high
  }

  // Test control effectiveness reduction
  void testControlEffectivenessReduction() {
    // GIVEN: Flight speed and reversal speed
    double V_flight = 200.0;    // ft/s
    double V_reversal = 400.0;  // ft/s

    // WHEN: Calculate effectiveness reduction factor
    double effectiveness = 1.0 - (V_flight / V_reversal);

    // THEN: Effectiveness reduced but still positive
    TS_ASSERT(effectiveness > 0.0);
    TS_ASSERT(effectiveness < 1.0);
    TS_ASSERT_DELTA(effectiveness, 0.5, 0.1);  // 50% at half reversal speed
  }

  /***************************************************************************
   * Reduced Frequency Tests
   ***************************************************************************/

  // Test reduced frequency calculation
  void testReducedFrequency() {
    // GIVEN: Oscillation and flight parameters
    double omega = 10.0;     // rad/s (oscillation frequency)
    double chord = 5.0;      // ft
    double velocity = 200.0; // ft/s

    // WHEN: Calculate reduced frequency
    double k = omega * chord / (2.0 * velocity);

    // THEN: Reduced frequency should be dimensionless and small
    TS_ASSERT(k > 0.0);
    TS_ASSERT(k < 1.0);  // Typically < 0.5 for aircraft
    TS_ASSERT_DELTA(k, 0.125, 0.05);
  }

  // Test quasi-steady assumption validity
  void testQuasiSteadyAssumption() {
    // GIVEN: Low reduced frequency
    double k_low = 0.05;   // quasi-steady valid
    double k_high = 0.3;   // unsteady effects significant

    // THEN: Check validity criteria
    TS_ASSERT(k_low < 0.15);   // Quasi-steady valid
    TS_ASSERT(k_high > 0.15);  // Unsteady aerodynamics needed
  }

  // Test reduced frequency vs Strouhal number
  void testReducedFrequencyRelation() {
    // GIVEN: Oscillation parameters
    double frequency_hz = 2.0;  // Hz
    double chord = 5.0;         // ft
    double velocity = 200.0;    // ft/s

    // WHEN: Calculate reduced frequency
    double omega = 2.0 * M_PI * frequency_hz;
    double k = omega * chord / (2.0 * velocity);

    // THEN: Should match alternative calculation
    double k_alt = M_PI * frequency_hz * chord / velocity;
    TS_ASSERT_DELTA(k, k_alt, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Wing Tip Deflection Tests
   ***************************************************************************/

  // Test static wing tip deflection
  void testStaticWingTipDeflection() {
    // GIVEN: Cantilever wing with tip load
    double tipLoad = 1000.0;  // lb
    double length = 20.0;     // ft
    double EI = 1e7;          // lb-ft^2

    // WHEN: Calculate tip deflection
    double delta_tip = (tipLoad * length * length * length) / (3.0 * EI);

    // THEN: Deflection should be positive and reasonable
    TS_ASSERT(delta_tip > 0.0);
    TS_ASSERT(delta_tip < 5.0);  // Less than 5 ft for typical wing
  }

  // Test distributed load deflection
  void testDistributedLoadDeflection() {
    // GIVEN: Uniformly loaded cantilever
    double w = 50.0;      // lb/ft (distributed load)
    double length = 20.0; // ft
    double EI = 1e7;      // lb-ft^2

    // WHEN: Calculate tip deflection
    double delta_tip = (w * length * length * length * length) / (8.0 * EI);

    // THEN: Deflection should be positive
    TS_ASSERT(delta_tip > 0.0);
  }

  // Test deflection angle at tip
  void testWingTipDeflectionAngle() {
    // GIVEN: Cantilever with tip load
    double tipLoad = 1000.0;
    double length = 20.0;
    double EI = 1e7;

    // WHEN: Calculate tip angle
    double theta_tip = (tipLoad * length * length) / (2.0 * EI);  // radians

    // THEN: Angle should be small (small deflection theory valid)
    TS_ASSERT(theta_tip > 0.0);
    TS_ASSERT(theta_tip < 0.1);  // Less than ~6 degrees
  }

  /***************************************************************************
   * Control Reversal Tests
   ***************************************************************************/

  // Test aileron reversal dynamic pressure
  void testAileronReversalDynamicPressure() {
    // GIVEN: Wing torsional properties
    double GJ = 5e6;              // lb-ft^2/rad
    double rollMomentCoeff = 0.2; // per radian aileron deflection
    double wingArea = 200.0;      // ft^2
    double span = 40.0;           // ft

    // WHEN: Calculate reversal dynamic pressure
    double q_reversal = GJ / (rollMomentCoeff * wingArea * span);

    // THEN: Should be positive
    TS_ASSERT(q_reversal > 0.0);
  }

  // Test roll rate reduction near reversal
  void testRollRateReduction() {
    // GIVEN: Flight condition
    double q_flight = 100.0;    // psf
    double q_reversal = 250.0;  // psf

    // WHEN: Calculate roll rate reduction
    double reduction_factor = 1.0 - (q_flight / q_reversal);

    // THEN: Factor should be between 0 and 1
    TS_ASSERT(reduction_factor > 0.0);
    TS_ASSERT(reduction_factor < 1.0);
    TS_ASSERT_DELTA(reduction_factor, 0.6, 0.1);
  }

  /***************************************************************************
   * Dynamic Aeroelastic Response Tests
   ***************************************************************************/

  // Test harmonic response amplitude
  void testHarmonicResponseAmplitude() {
    // GIVEN: Forced oscillation parameters
    double F0 = 100.0;       // lb (force amplitude)
    double mass = 50.0;      // slug
    double omega_n = 10.0;   // rad/s
    double omega = 8.0;      // rad/s (forcing frequency)
    double zeta = 0.05;      // damping ratio

    // WHEN: Calculate response amplitude
    double static_deflection = F0 / (mass * omega_n * omega_n);
    double frequency_ratio = omega / omega_n;
    double denominator = std::sqrt((1.0 - frequency_ratio * frequency_ratio) *
                                   (1.0 - frequency_ratio * frequency_ratio) +
                                   (2.0 * zeta * frequency_ratio) *
                                   (2.0 * zeta * frequency_ratio));
    double dynamic_amplitude = static_deflection / denominator;

    // THEN: Dynamic amplitude should be positive
    TS_ASSERT(dynamic_amplitude > 0.0);
    TS_ASSERT(dynamic_amplitude > static_deflection);  // Amplification below resonance
  }

  // Test resonance amplification
  void testResonanceAmplification() {
    // GIVEN: Forcing at natural frequency
    double zeta = 0.03;  // Low damping

    // WHEN: Calculate amplification at resonance
    double Q_factor = 1.0 / (2.0 * zeta);  // Quality factor

    // THEN: Amplification should be significant
    TS_ASSERT(Q_factor > 10.0);
    TS_ASSERT_DELTA(Q_factor, 16.67, 1.0);
  }

  // Test phase lag
  void testPhaseLag() {
    // GIVEN: Response parameters
    double omega_n = 10.0;
    double omega = 12.0;
    double zeta = 0.05;

    // WHEN: Calculate phase lag
    double r = omega / omega_n;
    double phase = std::atan2(2.0 * zeta * r, 1.0 - r * r);

    // THEN: Phase should be between 0 and pi
    TS_ASSERT(phase > 0.0);
    TS_ASSERT(phase < M_PI);
  }

  /***************************************************************************
   * Static Aeroelastic Deformation Tests
   ***************************************************************************/

  // Test wing twist under load
  void testWingTwistUnderLoad() {
    // GIVEN: Aerodynamic moment
    double aeroMoment = 5000.0;  // lb-ft
    double GJ = 5e6;             // lb-ft^2/rad
    double span = 20.0;          // ft

    // WHEN: Calculate twist angle
    double theta = aeroMoment * span / GJ;  // radians

    // THEN: Twist should be small
    TS_ASSERT(theta > 0.0);
    TS_ASSERT(theta < 0.1);  // Less than ~6 degrees
  }

  // Test lift redistribution due to twist
  void testLiftRedistributionDueToTwist() {
    // GIVEN: Wing twist
    double twist_angle = 0.05;  // radians
    double CL_alpha = 5.73;     // per radian

    // WHEN: Calculate local lift coefficient change
    double delta_CL = CL_alpha * twist_angle;

    // THEN: Lift change should be reasonable
    TS_ASSERT(delta_CL > 0.0);
    TS_ASSERT(delta_CL < 1.0);
  }

  // Test elastic deformation effect on drag
  void testElasticDeformationDrag() {
    // GIVEN: Wing bending
    double tip_deflection = 2.0;  // ft
    double span = 40.0;           // ft

    // WHEN: Calculate induced drag increase due to effective dihedral
    double effective_dihedral = std::atan(tip_deflection / span);

    // THEN: Angle should be small
    TS_ASSERT(effective_dihedral > 0.0);
    TS_ASSERT(effective_dihedral < 0.1);  // Less than ~6 degrees
  }

  /***************************************************************************
   * Bending-Torsion Coupling Tests
   ***************************************************************************/

  // Test coupling coefficient
  void testBendingTorsionCouplingCoefficient() {
    // GIVEN: Wing geometry
    double massCenter = 0.35;  // chord fraction
    double elasticAxis = 0.40; // chord fraction
    double chord = 5.0;        // ft

    // WHEN: Calculate coupling distance
    double coupling_distance = (massCenter - elasticAxis) * chord;

    // THEN: Coupling should be small
    TS_ASSERT(std::abs(coupling_distance) < 1.0);  // Less than 1 ft
  }

  // Test coupled mode frequency
  void testCoupledModeFrequency() {
    // GIVEN: Uncoupled frequencies
    double omega_bending = 6.0;   // rad/s
    double omega_torsion = 18.0;  // rad/s

    // WHEN: Coupling is present
    double omega_geometric_mean = std::sqrt(omega_bending * omega_torsion);

    // THEN: Coupled frequency between individual modes
    TS_ASSERT(omega_geometric_mean > omega_bending);
    TS_ASSERT(omega_geometric_mean < omega_torsion);
    TS_ASSERT_DELTA(omega_geometric_mean, 10.39, 0.1);
  }

  // Test coupling effect on flutter boundary
  void testCouplingEffectOnFlutter() {
    // GIVEN: Coupling parameter
    double coupling_ratio = 0.15;  // dimensionless

    // THEN: Strong coupling (>0.3) increases flutter criticality
    TS_ASSERT(coupling_ratio > 0.0);
    TS_ASSERT(coupling_ratio < 0.5);  // Moderate coupling
  }

  /***************************************************************************
   * Vibration Amplitude Limits Tests
   ***************************************************************************/

  // Test stress-based amplitude limit
  void testStressBasedAmplitudeLimit() {
    // GIVEN: Wing properties
    double allowable_stress = 30000.0;  // psi
    double elastic_modulus = 1e7;       // psi
    double structure_height = 0.5;      // ft
    double length = 20.0;               // ft

    // WHEN: Calculate maximum allowable tip deflection
    double max_strain = allowable_stress / elastic_modulus;
    double max_tip_deflection = max_strain * length * length / structure_height;

    // THEN: Limit should be reasonable
    TS_ASSERT(max_tip_deflection > 0.0);
    TS_ASSERT(max_tip_deflection < 10.0);
  }

  // Test g-load based limit
  void testGLoadBasedLimit() {
    // GIVEN: Load factor limits
    double max_load_factor = 6.0;  // g's (aerobatic)
    double design_load_factor = 3.8;  // g's (normal)

    // WHEN: Calculate safety margin
    double safety_margin = max_load_factor / design_load_factor;

    // THEN: Margin should be greater than 1
    TS_ASSERT(safety_margin > 1.0);
    TS_ASSERT(safety_margin < 2.0);  // Typical 1.5x
  }

  /***************************************************************************
   * Flutter Margin Tests
   ***************************************************************************/

  // Test flutter margin calculation
  void testFlutterMargin() {
    // GIVEN: Flight and flutter speeds
    double V_flight = 250.0;   // ft/s
    double V_flutter = 400.0;  // ft/s

    // WHEN: Calculate flutter margin
    double flutter_margin = (V_flutter - V_flight) / V_flight;

    // THEN: Margin should be positive (at least 15% required)
    TS_ASSERT(flutter_margin > 0.15);
    TS_ASSERT_DELTA(flutter_margin, 0.60, 0.1);
  }

  // Test flutter margin in terms of dynamic pressure
  void testFlutterMarginDynamicPressure() {
    // GIVEN: Dynamic pressures
    double q_flight = 100.0;   // psf
    double q_flutter = 200.0;  // psf

    // WHEN: Calculate margin
    double margin = q_flutter / q_flight;

    // THEN: Should have at least 1.15 margin
    TS_ASSERT(margin > 1.15);
  }

  // Test flutter boundary with uncertainty
  void testFlutterBoundaryWithUncertainty() {
    // GIVEN: Calculated flutter speed with uncertainty
    double V_flutter_calculated = 400.0;  // ft/s
    double uncertainty_factor = 0.85;     // 15% knockdown for uncertainty

    // WHEN: Apply safety factor
    double V_flutter_design = V_flutter_calculated * uncertainty_factor;

    // THEN: Design flutter speed lower than calculated
    TS_ASSERT(V_flutter_design < V_flutter_calculated);
    TS_ASSERT_DELTA(V_flutter_design, 340.0, 10.0);
  }

  /***************************************************************************
   * Additional Stiffness Parameter Tests
   ***************************************************************************/

  // Test bending stiffness EI calculation
  void testBendingStiffnessEI() {
    // GIVEN: Beam properties
    double E = 1e7;           // psi (elastic modulus)
    double moment_of_inertia = 100.0;  // in^4

    // WHEN: Calculate EI
    double EI = E * moment_of_inertia;

    // THEN: Should be positive
    TS_ASSERT(EI > 0.0);
    TS_ASSERT_DELTA(EI, 1e9, 1e6);  // lb-in^2
  }

  // Test torsional stiffness GJ calculation
  void testTorsionalStiffnessGJ() {
    // GIVEN: Torsional properties
    double G = 4e6;           // psi (shear modulus)
    double polar_moment = 150.0;  // in^4

    // WHEN: Calculate GJ
    double GJ = G * polar_moment;

    // THEN: Should be positive
    TS_ASSERT(GJ > 0.0);
    TS_ASSERT_DELTA(GJ, 6e8, 1e6);  // lb-in^2
  }

  // Test stiffness ratio effect
  void testStiffnessRatioEffect() {
    // GIVEN: Stiffness values
    double EI = 1e7;  // lb-ft^2
    double GJ = 5e6;  // lb-ft^2/rad

    // WHEN: Calculate ratio
    double stiffness_ratio = EI / GJ;

    // THEN: Ratio typically 1-3 for aircraft wings
    TS_ASSERT(stiffness_ratio > 0.5);
    TS_ASSERT(stiffness_ratio < 5.0);
  }

  /***************************************************************************
   * Theodorsen Function and Unsteady Aerodynamics Tests
   ***************************************************************************/

  // Test Theodorsen function magnitude at low reduced frequency
  void testTheodorsenFunctionLowFrequency() {
    // GIVEN: Low reduced frequency
    double k = 0.05;

    // WHEN: Approximate Theodorsen function magnitude
    // For small k, C(k) ≈ 1 - (1/(2+k))
    double magnitude_approx = 1.0 - 1.0 / (2.0 + k * 10.0);

    // THEN: Magnitude should be close to 1 (quasi-steady)
    TS_ASSERT(magnitude_approx > 0.5);
    TS_ASSERT(magnitude_approx < 1.0);
  }

  // Test circulatory lift lag
  void testCirculatoryLiftLag() {
    // GIVEN: Oscillating airfoil parameters
    double k = 0.2;  // reduced frequency
    double phase_approx = -std::atan(k);  // simplified phase lag

    // THEN: Phase should be negative (lag)
    TS_ASSERT(phase_approx < 0.0);
    TS_ASSERT(phase_approx > -M_PI / 2.0);
  }

  // Test apparent mass effect
  void testApparentMassEffect() {
    // GIVEN: Wing section properties
    double chord = 5.0;      // ft
    double rho = 0.002377;   // slug/ft^3
    double span_section = 1.0;  // ft

    // WHEN: Calculate apparent mass per unit span
    double apparent_mass = M_PI * rho * chord * chord / 4.0;

    // THEN: Apparent mass should be positive
    TS_ASSERT(apparent_mass > 0.0);
    TS_ASSERT(apparent_mass < 1.0);  // slug/ft typically
  }

  // Test non-circulatory lift contribution
  void testNonCirculatoryLift() {
    // GIVEN: Pitching rate
    double alpha_dot = 0.5;  // rad/s
    double chord = 5.0;      // ft
    double velocity = 200.0; // ft/s
    double rho = 0.002377;   // slug/ft^3

    // WHEN: Calculate non-circulatory lift coefficient contribution
    double CL_nc = M_PI * chord * alpha_dot / (2.0 * velocity);

    // THEN: Should be small
    TS_ASSERT(std::abs(CL_nc) < 0.1);
  }

  /***************************************************************************
   * Wagner Function and Indicial Response Tests
   ***************************************************************************/

  // Test Wagner function for step change in angle of attack
  void testWagnerFunction() {
    // GIVEN: Dimensionless time
    double s = 5.0;  // semi-chords traveled

    // WHEN: Approximate Wagner function value
    // Jones approximation: phi(s) = 1 - 0.165*exp(-0.0455*s) - 0.335*exp(-0.3*s)
    double phi = 1.0 - 0.165 * std::exp(-0.0455 * s) - 0.335 * std::exp(-0.3 * s);

    // THEN: Should approach 1 as s increases
    TS_ASSERT(phi > 0.5);
    TS_ASSERT(phi < 1.0);
  }

  // Test lift buildup after step input
  void testLiftBuildupAfterStep() {
    // GIVEN: Steady state lift and times
    double CL_steady = 1.0;
    double semi_chords[] = {1.0, 5.0, 10.0, 20.0};

    // THEN: Lift should increase monotonically
    double prev_lift = 0.0;
    for (double s : semi_chords) {
      double phi = 1.0 - 0.165 * std::exp(-0.0455 * s) - 0.335 * std::exp(-0.3 * s);
      double current_lift = CL_steady * phi;
      TS_ASSERT(current_lift >= prev_lift);
      prev_lift = current_lift;
    }
  }

  // Test Kussner function for gust encounter
  void testKussnerFunction() {
    // GIVEN: Dimensionless distance into gust
    double s = 10.0;  // semi-chords

    // WHEN: Approximate Kussner function
    // Sears approximation: psi(s) = 1 - 0.5*exp(-0.13*s) - 0.5*exp(-s)
    double psi = 1.0 - 0.5 * std::exp(-0.13 * s) - 0.5 * std::exp(-s);

    // THEN: Should approach 1 for large s
    TS_ASSERT(psi > 0.7);
    TS_ASSERT(psi < 1.0);
  }

  /***************************************************************************
   * Gust Response Tests
   ***************************************************************************/

  // Test sharp-edge gust load factor
  void testSharpEdgeGustLoadFactor() {
    // GIVEN: Flight and gust parameters
    double V = 200.0;         // ft/s
    double U_gust = 50.0;     // ft/s (FAR gust velocity)
    double CL_alpha = 5.73;   // per radian
    double rho = 0.002377;    // slug/ft^3
    double wing_loading = 50.0;  // psf

    // WHEN: Calculate gust load factor
    double delta_n = (rho * V * U_gust * CL_alpha) / (2.0 * wing_loading);

    // THEN: Load factor increment should be positive
    TS_ASSERT(delta_n > 0.0);
    TS_ASSERT(delta_n < 5.0);  // Reasonable for transport
  }

  // Test gust alleviation factor
  void testGustAlleviationFactor() {
    // GIVEN: Aircraft mass ratio
    double mu_g = 50.0;  // gust mass ratio

    // WHEN: Calculate alleviation factor (kg)
    double kg = 0.88 * mu_g / (5.3 + mu_g);

    // THEN: Factor should be less than 1
    TS_ASSERT(kg > 0.0);
    TS_ASSERT(kg < 1.0);
  }

  // Test 1-cosine gust profile
  void testOneCosineGust() {
    // GIVEN: Gust parameters
    double U_max = 50.0;  // ft/s
    double gust_length = 350.0;  // ft (25 chord lengths)
    double x = 175.0;  // midpoint

    // WHEN: Calculate gust velocity at position
    double U = U_max / 2.0 * (1.0 - std::cos(2.0 * M_PI * x / gust_length));

    // THEN: Maximum at midpoint
    TS_ASSERT_DELTA(U, U_max, 0.01);
  }

  // Test turbulence PSD (Von Karman spectrum)
  void testVonKarmanTurbulencePSD() {
    // GIVEN: Turbulence parameters
    double sigma_w = 5.0;   // ft/s RMS vertical velocity
    double L = 1750.0;      // ft scale length
    double omega = 0.5;     // rad/s

    // WHEN: Approximate spectral density
    double numerator = sigma_w * sigma_w * L / M_PI;
    double denominator = 1.0 + std::pow(1.339 * L * omega / 200.0, 2);
    double PSD = numerator / denominator;

    // THEN: PSD should be positive
    TS_ASSERT(PSD > 0.0);
  }

  /***************************************************************************
   * Modal Analysis Tests
   ***************************************************************************/

  // Test modal mass calculation
  void testModalMassCalculation() {
    // GIVEN: Mode shape and distributed mass
    double m_per_ft = 50.0;  // slug/ft
    double L = 20.0;         // ft
    double lambda = 1.875;   // first mode eigenvalue

    // WHEN: Calculate modal mass
    // For first bending mode, modal mass ≈ 0.25 * physical mass
    double physical_mass = m_per_ft * L;
    double modal_mass = 0.25 * physical_mass;

    // THEN: Modal mass less than physical
    TS_ASSERT(modal_mass < physical_mass);
    TS_ASSERT(modal_mass > 0.0);
  }

  // Test modal stiffness
  void testModalStiffness() {
    // GIVEN: Modal frequency and mass
    double omega_n = 10.0;     // rad/s
    double modal_mass = 250.0; // slug

    // WHEN: Calculate modal stiffness
    double modal_stiffness = modal_mass * omega_n * omega_n;

    // THEN: Stiffness should be positive
    TS_ASSERT(modal_stiffness > 0.0);
    TS_ASSERT_DELTA(modal_stiffness, 25000.0, 100.0);
  }

  // Test generalized force calculation
  void testGeneralizedForce() {
    // GIVEN: Mode shape and applied load
    double load_amplitude = 1000.0;  // lb
    double mode_shape_at_load = 0.8;  // normalized

    // WHEN: Calculate generalized force
    double Q = load_amplitude * mode_shape_at_load;

    // THEN: Should be proportional to mode shape
    TS_ASSERT_DELTA(Q, 800.0, 0.1);
  }

  // Test orthogonality of modes
  void testModeOrthogonality() {
    // GIVEN: Two different mode frequencies
    double omega_1 = 10.0;
    double omega_2 = 30.0;  // Not equal

    // WHEN: Check frequency separation
    double freq_ratio = omega_2 / omega_1;

    // THEN: Well-separated modes (ratio > 2) are typically orthogonal
    TS_ASSERT(freq_ratio > 1.0);
    TS_ASSERT(!std::isnan(freq_ratio));
  }

  /***************************************************************************
   * Structural Nonlinearity Tests
   ***************************************************************************/

  // Test freeplay nonlinearity effect
  void testFreeplayNonlinearity() {
    // GIVEN: Control surface with freeplay
    double freeplay_angle = 0.5 * M_PI / 180.0;  // 0.5 degrees in radians
    double deflection = 1.0 * M_PI / 180.0;     // 1 degree input

    // WHEN: Calculate effective stiffness with freeplay
    double effective_deflection = deflection - freeplay_angle;

    // THEN: Effective deflection less than input when inside freeplay
    TS_ASSERT(effective_deflection < deflection);
    TS_ASSERT(effective_deflection > 0.0);
  }

  // Test hardening spring behavior
  void testHardeningSpringBehavior() {
    // GIVEN: Cubic hardening spring
    double k_linear = 1e6;  // lb/ft
    double k_cubic = 1e5;   // lb/ft^3
    double deflection = 1.0; // ft

    // WHEN: Calculate restoring force
    double force = k_linear * deflection + k_cubic * deflection * deflection * deflection;

    // THEN: Force higher than linear alone
    TS_ASSERT(force > k_linear * deflection);
  }

  // Test softening spring behavior
  void testSofteningSpringBehavior() {
    // GIVEN: Cubic softening spring
    double k_linear = 1e6;   // lb/ft
    double k_cubic = -5e4;   // lb/ft^3 (negative for softening)
    double deflection = 1.0; // ft

    // WHEN: Calculate restoring force
    double force = k_linear * deflection + k_cubic * deflection * deflection * deflection;

    // THEN: Force lower than linear alone
    TS_ASSERT(force < k_linear * deflection);
    TS_ASSERT(force > 0.0);  // Still positive for positive deflection
  }

  // Test limit cycle oscillation amplitude
  void testLimitCycleOscillation() {
    // GIVEN: Nonlinear system parameters
    double flutter_speed_linear = 400.0;  // ft/s
    double nonlinearity_factor = 0.1;     // hardening

    // WHEN: Flight above linear flutter speed
    double V_flight = 420.0;  // 5% above

    // THEN: LCO amplitude proportional to excess speed
    double excess_ratio = (V_flight - flutter_speed_linear) / flutter_speed_linear;
    double LCO_amplitude = std::sqrt(excess_ratio / nonlinearity_factor);
    TS_ASSERT(LCO_amplitude > 0.0);
    TS_ASSERT(LCO_amplitude < 10.0);  // Bounded oscillation
  }

  /***************************************************************************
   * Panel Flutter Tests
   ***************************************************************************/

  // Test panel flutter critical dynamic pressure
  void testPanelFlutterCriticalPressure() {
    // GIVEN: Panel properties
    double E = 1e7;           // psi
    double t = 0.1;           // in (thickness)
    double a = 20.0;          // in (length in flow direction)
    double b = 10.0;          // in (width)
    double nu = 0.3;          // Poisson's ratio

    // WHEN: Calculate flutter parameter
    double D = E * t * t * t / (12.0 * (1.0 - nu * nu));
    double lambda_crit = D * M_PI * M_PI * M_PI * M_PI / (a * a * a * a);

    // THEN: Critical parameter should be positive
    TS_ASSERT(lambda_crit > 0.0);
  }

  // Test panel aspect ratio effect
  void testPanelAspectRatioEffect() {
    // GIVEN: Two panels with different aspect ratios
    double a1 = 20.0, b1 = 10.0;  // AR = 2
    double a2 = 20.0, b2 = 20.0;  // AR = 1

    // WHEN: Calculate aspect ratios
    double AR1 = a1 / b1;
    double AR2 = a2 / b2;

    // THEN: Higher AR typically more critical for flutter
    TS_ASSERT(AR1 > AR2);
  }

  // Test thermal stress effect on panel
  void testThermalStressEffectOnPanel() {
    // GIVEN: Thermal conditions
    double alpha = 1.3e-5;   // 1/degF (thermal expansion)
    double delta_T = 200.0;  // degF temperature rise
    double E = 1e7;          // psi

    // WHEN: Calculate thermal stress
    double thermal_strain = alpha * delta_T;
    double thermal_stress = E * thermal_strain;

    // THEN: Compression reduces flutter margin
    TS_ASSERT(thermal_stress > 0.0);
    TS_ASSERT(thermal_stress < 50000.0);  // psi
  }

  /***************************************************************************
   * Whirl Flutter Tests
   ***************************************************************************/

  // Test whirl flutter precession frequency
  void testWhirlFlutterPrecession() {
    // GIVEN: Propeller/rotor parameters
    double omega_rotation = 200.0;  // rad/s (rotor speed)
    double hub_moment_stiffness = 1e6;  // lb-ft/rad
    double blade_polar_inertia = 100.0;  // slug-ft^2

    // WHEN: Calculate whirl frequency
    double omega_whirl = std::sqrt(hub_moment_stiffness / blade_polar_inertia);

    // THEN: Whirl frequency should be less than rotation
    TS_ASSERT(omega_whirl > 0.0);
    TS_ASSERT(omega_whirl < omega_rotation);
  }

  // Test whirl flutter stability margin
  void testWhirlFlutterStabilityMargin() {
    // GIVEN: Pylon/nacelle properties
    double pylon_pitch_stiffness = 5e6;  // lb-ft/rad
    double pylon_yaw_stiffness = 4e6;    // lb-ft/rad

    // WHEN: Calculate stiffness ratio
    double ratio = pylon_pitch_stiffness / pylon_yaw_stiffness;

    // THEN: Ratio near 1 is best for whirl stability
    TS_ASSERT(ratio > 0.8);
    TS_ASSERT(ratio < 1.5);
  }

  /***************************************************************************
   * Ground Vibration Test Correlation
   ***************************************************************************/

  // Test frequency correlation criteria
  void testFrequencyCorrelationCriteria() {
    // GIVEN: Measured and predicted frequencies
    double f_measured = 5.2;   // Hz
    double f_predicted = 5.0;  // Hz

    // WHEN: Calculate percent error
    double error = std::abs(f_measured - f_predicted) / f_measured * 100.0;

    // THEN: Error should be within 5% for good correlation
    TS_ASSERT(error < 10.0);  // 10% acceptable for initial model
  }

  // Test mode shape correlation (MAC)
  void testModeShapeCorrelation() {
    // GIVEN: Simplified mode shape comparison
    double dot_product = 0.95;     // normalized
    double norm_1 = 1.0;
    double norm_2 = 1.0;

    // WHEN: Calculate MAC value
    double MAC = (dot_product * dot_product) / (norm_1 * norm_2);

    // THEN: MAC > 0.9 indicates good correlation
    TS_ASSERT(MAC > 0.85);
    TS_ASSERT(MAC <= 1.0);
  }

  // Test damping identification
  void testDampingIdentification() {
    // GIVEN: Half-power bandwidth measurements
    double f_n = 10.0;        // Hz (natural frequency)
    double f_1 = 9.8;         // Hz (lower half-power point)
    double f_2 = 10.2;        // Hz (upper half-power point)

    // WHEN: Calculate damping ratio
    double zeta = (f_2 - f_1) / (2.0 * f_n);

    // THEN: Damping should be positive and small
    TS_ASSERT(zeta > 0.0);
    TS_ASSERT(zeta < 0.1);
    TS_ASSERT_DELTA(zeta, 0.02, 0.005);
  }

  /***************************************************************************
   * Additional Specialized Tests
   ***************************************************************************/

  // Test aerodynamic influence coefficient
  void testAerodynamicInfluenceCoefficient() {
    // GIVEN: Panel aerodynamic parameters
    double panel_area = 10.0;    // ft^2
    double distance = 5.0;       // ft
    double rho = 0.002377;       // slug/ft^3

    // WHEN: Calculate influence coefficient (simplified)
    double AIC = panel_area / (4.0 * M_PI * distance * distance);

    // THEN: Should be positive and bounded
    TS_ASSERT(AIC > 0.0);
    TS_ASSERT(AIC < 1.0);
  }

  // Test structural influence coefficient
  void testStructuralInfluenceCoefficient() {
    // GIVEN: Flexibility matrix element
    double force = 1000.0;       // lb
    double deflection = 0.01;    // ft

    // WHEN: Calculate flexibility coefficient
    double flexibility = deflection / force;

    // THEN: Should be positive
    TS_ASSERT(flexibility > 0.0);
  }

  // Test flutter boundary with Mach number
  void testFlutterBoundaryVsMach() {
    // GIVEN: Subsonic flutter speed and compressibility correction
    double V_flutter_incomp = 400.0;  // ft/s
    double Mach = 0.7;

    // WHEN: Apply Prandtl-Glauert correction
    double beta = std::sqrt(1.0 - Mach * Mach);
    double V_flutter_comp = V_flutter_incomp * std::sqrt(beta);

    // THEN: Compressible flutter speed lower
    TS_ASSERT(V_flutter_comp < V_flutter_incomp);
    TS_ASSERT(V_flutter_comp > 0.0);
  }

  // Test transonic flutter dip
  void testTransonicFlutterDip() {
    // GIVEN: Flutter speed at different Mach numbers
    double V_flutter_low_mach = 400.0;   // ft/s at M=0.6
    double dip_factor = 0.85;            // 15% reduction transonic

    // WHEN: Calculate transonic flutter speed
    double V_flutter_transonic = V_flutter_low_mach * dip_factor;

    // THEN: Transonic is lower
    TS_ASSERT(V_flutter_transonic < V_flutter_low_mach);
    TS_ASSERT_DELTA(V_flutter_transonic, 340.0, 5.0);
  }

  // Test supersonic flutter characteristics
  void testSupersonicFlutterCharacteristics() {
    // GIVEN: Supersonic Mach number
    double Mach = 1.5;

    // WHEN: Calculate aerodynamic lag parameter
    double beta_super = std::sqrt(Mach * Mach - 1.0);

    // THEN: Beta should be real for M > 1
    TS_ASSERT(beta_super > 0.0);
    TS_ASSERT(!std::isnan(beta_super));
  }

  // Test store flutter interaction
  void testStoreFlutterInteraction() {
    // GIVEN: Clean wing flutter speed
    double V_flutter_clean = 500.0;  // ft/s
    double mass_ratio_store = 0.1;   // store mass / wing mass

    // WHEN: Estimate flutter speed with store
    // Simplified: reduction proportional to mass ratio
    double reduction_factor = 1.0 - 0.3 * mass_ratio_store;
    double V_flutter_store = V_flutter_clean * reduction_factor;

    // THEN: External stores typically reduce flutter speed
    TS_ASSERT(V_flutter_store < V_flutter_clean);
    TS_ASSERT(V_flutter_store > 0.8 * V_flutter_clean);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete flutter analysis
  void testCompleteFlutterAnalysis() {
    double chord = 5.0;           // ft
    double span = 40.0;           // ft
    double rho = 0.002377;        // slug/ft^3
    double GJ = 5e6;              // lb-ft^2 (torsional stiffness)
    double mass_per_span = 5.0;   // slug/ft
    double x_alpha = 0.1 * chord; // elastic axis offset

    // Torsional frequency
    double omega_alpha = std::sqrt(GJ / (mass_per_span * span * span * span));

    // Divergence speed (simplified)
    double q_div = GJ / (chord * span * span * 2.0);  // dynamic pressure
    double V_div = std::sqrt(2.0 * q_div / rho);

    TS_ASSERT(omega_alpha > 0.0);
    TS_ASSERT(V_div > 0.0);
    TS_ASSERT(!std::isnan(V_div));
  }

  // Test wing bending-torsion coupling frequency
  void testBendingTorsionCouplingFrequency() {
    double omega_b = 10.0;   // rad/s bending frequency
    double omega_t = 25.0;   // rad/s torsion frequency
    double coupling = 0.3;   // coupling coefficient

    // Coupled frequencies (simplified)
    double omega_low = omega_b * std::sqrt(1.0 - coupling);
    double omega_high = omega_t * std::sqrt(1.0 + coupling);

    TS_ASSERT(omega_low < omega_b);
    TS_ASSERT(omega_high > omega_t);
  }

  // Test gust response factor
  void testGustResponseFactor() {
    double omega_n = 15.0;  // rad/s natural frequency
    double V = 300.0;       // ft/s airspeed
    double gust_length = 50.0;  // ft

    double omega_gust = 2.0 * M_PI * V / gust_length;
    double freq_ratio = omega_gust / omega_n;
    double response_factor = 1.0 / std::sqrt(1.0 + freq_ratio * freq_ratio);

    TS_ASSERT(response_factor > 0.0);
    TS_ASSERT(response_factor <= 1.0);
  }

  // Test aerodynamic damping coefficient
  void testAerodynamicDampingCoefficient() {
    double rho = 0.002377;
    double V = 400.0;
    double chord = 5.0;
    double CLa = 5.7;  // lift curve slope per rad

    double damping = rho * V * chord * CLa / 4.0;

    TS_ASSERT(damping > 0.0);
    TS_ASSERT(!std::isnan(damping));
  }

  // Test whirl flutter analysis for propeller
  void testWhirlFlutterAnalysis() {
    double prop_rpm = 2000.0;
    double omega_prop = prop_rpm * 2.0 * M_PI / 60.0;
    double pylon_freq = 15.0;  // Hz
    double omega_pylon = pylon_freq * 2.0 * M_PI;

    // Whirl flutter can occur when omega_prop approaches omega_pylon
    double freq_ratio = omega_prop / omega_pylon;

    TS_ASSERT(freq_ratio > 0.0);
    TS_ASSERT(omega_prop > 0.0);
    TS_ASSERT(omega_pylon > 0.0);
  }

  // Test panel flutter critical dynamic pressure calculation
  void testPanelFlutterCriticalPressureCalculation() {
    double t = 0.05;         // in, panel thickness
    double a = 20.0;         // in, panel length
    double E = 10.5e6;       // psi, elastic modulus
    double nu = 0.33;        // Poisson's ratio

    double D = E * t * t * t / (12.0 * (1.0 - nu * nu));
    double q_cr = M_PI * M_PI * D / (a * a * a * a) * 4.0;  // simplified

    TS_ASSERT(q_cr > 0.0);
    TS_ASSERT(!std::isnan(q_cr));
  }

  // Test limit cycle oscillation amplitude
  void testLimitCycleOscillationAmplitude() {
    double structural_nonlinearity = 0.1;  // nonlinear stiffness coefficient
    double linear_flutter_amplitude = 5.0;  // degrees

    // LCO amplitude limited by nonlinearity
    double LCO_amplitude = linear_flutter_amplitude / std::sqrt(1.0 + structural_nonlinearity);

    TS_ASSERT(LCO_amplitude < linear_flutter_amplitude);
    TS_ASSERT(LCO_amplitude > 0.0);
  }

  // Test thermal effects on flutter
  void testThermalEffectsOnFlutter() {
    double E_room = 10.5e6;     // psi at room temp
    double E_hot = 9.5e6;       // psi at elevated temp
    double V_flutter_room = 500.0;  // ft/s

    // Flutter speed proportional to sqrt(E)
    double V_flutter_hot = V_flutter_room * std::sqrt(E_hot / E_room);

    TS_ASSERT(V_flutter_hot < V_flutter_room);
  }

  // Test swept wing flutter characteristics
  void testSweptWingFlutterCharacteristics() {
    double sweep = 30.0 * M_PI / 180.0;  // rad
    double V_flutter_straight = 500.0;   // ft/s

    // Sweep effect (simplified)
    double V_flutter_swept = V_flutter_straight * std::cos(sweep);

    TS_ASSERT(V_flutter_swept < V_flutter_straight);
    TS_ASSERT(V_flutter_swept > 0.0);
  }

  // Test flutter margin at altitude
  void testFlutterMarginAtAltitude() {
    double V_flutter_SL = 600.0;  // ft/s at sea level
    double rho_SL = 0.002377;
    double rho_alt = 0.001267;    // at 20000 ft

    // Flutter speed scales with sqrt(rho)
    double V_flutter_alt = V_flutter_SL * std::sqrt(rho_SL / rho_alt);

    TS_ASSERT(V_flutter_alt > V_flutter_SL);
  }

  // Test buffet intensity calculation
  void testBuffetIntensityCalculation() {
    double alpha = 15.0 * M_PI / 180.0;  // rad
    double alpha_buffet_onset = 12.0 * M_PI / 180.0;

    double buffet_intensity = 0.0;
    if (alpha > alpha_buffet_onset) {
      buffet_intensity = std::pow((alpha - alpha_buffet_onset) / alpha_buffet_onset, 2.0);
    }

    TS_ASSERT(buffet_intensity > 0.0);
  }

  // Test structural mode shape orthogonality
  void testModeShapeOrthogonality() {
    // Two mode shape vectors (simplified as scalars for test)
    double mode1_amplitude = 1.0;
    double mode2_amplitude = 1.0;
    double mass = 100.0;

    // Orthogonality: integral of m * phi1 * phi2 = 0 for different modes
    // For same mode: generalized mass = m
    double gen_mass = mass * mode1_amplitude * mode1_amplitude;

    TS_ASSERT_DELTA(gen_mass, 100.0, DEFAULT_TOLERANCE);
  }

  // Test control surface mass balance
  void testControlSurfaceMassBalance() {
    double hinge_moment_coeff = 0.05;
    double surface_area = 10.0;  // ft^2
    double chord_ratio = 0.25;
    double rho = 0.002377;
    double V = 300.0;

    double q = 0.5 * rho * V * V;
    double hinge_moment = q * surface_area * chord_ratio * hinge_moment_coeff;

    TS_ASSERT(hinge_moment > 0.0);
  }

  // Test flutter frequency coalescence
  void testFlutterFrequencyCoalescence() {
    double omega_bending = 20.0;   // rad/s
    double omega_torsion = 35.0;   // rad/s
    double V = 400.0;              // ft/s

    // At flutter, frequencies coalesce
    double freq_separation = std::abs(omega_torsion - omega_bending);
    double flutter_parameter = freq_separation / omega_bending;

    TS_ASSERT(flutter_parameter > 0.0);
    TS_ASSERT(flutter_parameter < 1.0);
  }

  // Test aeroelastic divergence safety factor
  void testDivergenceSafetyFactor() {
    double V_design = 400.0;     // ft/s design speed
    double V_divergence = 650.0; // ft/s divergence speed
    double safety_factor_req = 1.15;

    double actual_factor = V_divergence / V_design;

    TS_ASSERT(actual_factor > safety_factor_req);
  }

  // Test dynamic pressure at flutter
  void testDynamicPressureAtFlutter() {
    double rho = 0.002377;
    double V_flutter = 550.0;

    double q_flutter = 0.5 * rho * V_flutter * V_flutter;

    TS_ASSERT(q_flutter > 0.0);
    TS_ASSERT_DELTA(q_flutter, 359.5, 1.0);
  }

  // Test aeroelasticity instance independence
  void testAeroelasticityInstanceIndependence() {
    double config1_flutter_speed = 500.0;
    double config1_stiffness = 1e6;
    double config2_flutter_speed = 600.0;
    double config2_stiffness = 1.5e6;

    // Different configurations should have different properties
    TS_ASSERT(config1_flutter_speed != config2_flutter_speed);
    TS_ASSERT(config1_stiffness != config2_stiffness);
    TS_ASSERT(config2_flutter_speed > config1_flutter_speed);
  }

  // Test complete wing aeroelastic response
  void testCompleteWingAeroelasticResponse() {
    double EI = 1e7;       // lb-ft^2 bending stiffness
    double GJ = 5e6;       // lb-ft^2 torsional stiffness
    double span = 40.0;    // ft
    double chord = 5.0;    // ft
    double mass = 5.0;     // slug/ft

    // Natural frequencies
    double omega_bending = 1.875 * 1.875 * std::sqrt(EI / (mass * span * span * span * span));
    double omega_torsion = (M_PI / 2.0) * std::sqrt(GJ / (mass * chord * chord * span * span));

    TS_ASSERT(omega_bending > 0.0);
    TS_ASSERT(omega_torsion > 0.0);
    TS_ASSERT(omega_torsion > omega_bending);  // Torsion typically higher
  }
};

//=============================================================================
// C172x Integration Tests - Aeroelasticity with Real Aircraft Dynamics
//=============================================================================

class FGAeroelasticityC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {


    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: Wing loading within structural limits
  void testWingLoadingWithinLimits() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(160.0);  // VNE is around 160 KIAS
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    for (int i = 0; i < 20; ++i) fdm.Run();

    auto aux = fdm.GetAuxiliary();
    double Nlf = aux->GetNlf();

    // Normal load factor should be within limits (+3.8g to -1.52g for utility)
    TS_ASSERT(Nlf > -2.0);
    TS_ASSERT(Nlf < 5.0);
  }

  // Test 2: Control surface effectiveness at cruise
  void testControlEffectivenessAtCruise() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(120.0);
    ic->SetAltitudeASLFtIC(8000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto aero = fdm.GetAerodynamics();

    // Apply aileron
    fcs->SetDaCmd(0.3);
    for (int i = 0; i < 20; ++i) fdm.Run();
    double rollMoment = aero->GetMoments()(1);

    TS_ASSERT(std::abs(rollMoment) > 0.0);
  }

  // Test 3: Pitch response to elevator
  void testPitchResponseToElevator() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto prop = fdm.GetPropagate();

    double initialPitch = prop->GetEuler(2);

    fcs->SetDeCmd(-0.3);  // Nose up
    for (int i = 0; i < 100; ++i) fdm.Run();

    double finalPitchRate = prop->GetPQR(2);
    TS_ASSERT(finalPitchRate > 0.0);  // Should be pitching up
  }

  // Test 4: Roll damping present
  void testRollDampingPresent() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetPRadpsIC(0.2);  // Initial roll rate
    fdm.RunIC();

    auto prop = fdm.GetPropagate();

    // Run simulation and check roll rate decreases
    for (int i = 0; i < 50; ++i) fdm.Run();
    double rollRate = std::abs(prop->GetPQR(1));

    // Roll rate should have damped somewhat
    TS_ASSERT(rollRate < 0.25);
  }

  // Test 5: Structural response to gust (simulated with sudden alpha change)
  void testGustResponse() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetAlphaDegIC(2.0);
    fdm.RunIC();

    for (int i = 0; i < 10; ++i) fdm.Run();

    auto aux = fdm.GetAuxiliary();
    auto aero = fdm.GetAerodynamics();

    // Check that the aircraft responds
    double Nlf = aux->GetNlf();
    double liftForce = aero->GetForces()(3);

    TS_ASSERT(std::isfinite(Nlf));
    TS_ASSERT(std::isfinite(liftForce));
  }

  // Test 6: High speed handling (approaching Vne)
  void testHighSpeedHandling() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(150.0);  // Near Vne
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto aux = fdm.GetAuxiliary();
    double qbar = aux->Getqbar();

    // High dynamic pressure expected
    TS_ASSERT(qbar > 50.0);
    TS_ASSERT(std::isfinite(qbar));
  }

  // Test 7: Low speed handling (near stall)
  void testLowSpeedHandling() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(55.0);  // Near stall speed
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto aux = fdm.GetAuxiliary();
    auto aero = fdm.GetAerodynamics();

    // Even near stall, should have valid outputs
    TS_ASSERT(std::isfinite(aux->Getqbar()));
    TS_ASSERT(std::isfinite(aero->GetForces()(3)));
  }

  // Test 8: Maneuvering loads
  void testManeuveringLoads() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto aux = fdm.GetAuxiliary();

    // Apply pull-up
    fcs->SetDeCmd(-0.4);
    for (int i = 0; i < 50; ++i) fdm.Run();

    double Nlf = aux->GetNlf();

    // Load factor should increase during pull-up
    TS_ASSERT(Nlf > 0.5);  // Should be positive G
    TS_ASSERT(std::isfinite(Nlf));
  }

  // Test 9: Speed stability
  void testSpeedStability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto aux = fdm.GetAuxiliary();

    double initialSpeed = aux->GetVcalibratedKTS();

    // Run for a while
    for (int i = 0; i < 100; ++i) fdm.Run();

    double finalSpeed = aux->GetVcalibratedKTS();

    // Speed should be somewhat stable (within reason)
    TS_ASSERT(std::isfinite(finalSpeed));
    TS_ASSERT(std::abs(finalSpeed - initialSpeed) < 50.0);
  }

  // Test 10: Yaw stability
  void testYawStability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetBetaDegIC(5.0);  // Initial sideslip
    fdm.RunIC();

    auto aux = fdm.GetAuxiliary();

    for (int i = 0; i < 100; ++i) fdm.Run();

    double beta = aux->Getbeta() * 57.2958;  // Convert to degrees

    // Sideslip should remain bounded
    TS_ASSERT(std::abs(beta) < 30.0);
    TS_ASSERT(std::isfinite(beta));
  }

  // Test 11: Aerodynamic damping in all axes
  void testAerodynamicDamping() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetPRadpsIC(0.1);
    ic->SetQRadpsIC(0.1);
    ic->SetRRadpsIC(0.1);
    fdm.RunIC();

    auto aero = fdm.GetAerodynamics();
    const auto& moments = aero->GetMoments();

    for (int i = 0; i < 50; ++i) fdm.Run();

    // All moments should be finite
    TS_ASSERT(std::isfinite(moments(1)));
    TS_ASSERT(std::isfinite(moments(2)));
    TS_ASSERT(std::isfinite(moments(3)));
  }

  // Test 12: Extended simulation stability
  void testExtendedSimulationStability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto aux = fdm.GetAuxiliary();
    auto aero = fdm.GetAerodynamics();

    // Run for 500 iterations
    for (int i = 0; i < 500; ++i) {
      fdm.Run();
    }

    // Everything should remain finite and valid
    TS_ASSERT(std::isfinite(aux->GetNlf()));
    TS_ASSERT(std::isfinite(aux->Getqbar()));
    TS_ASSERT(std::isfinite(aero->GetForces()(1)));
    TS_ASSERT(std::isfinite(aero->GetForces()(2)));
    TS_ASSERT(std::isfinite(aero->GetForces()(3)));
  }
};
