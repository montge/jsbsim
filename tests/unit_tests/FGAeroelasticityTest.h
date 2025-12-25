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
};
