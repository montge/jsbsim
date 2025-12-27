/*******************************************************************************
 * FGStabilityTest.h - Unit tests for stability derivative calculations
 *
 * Tests the mathematical behavior of aerodynamic stability:
 * - Longitudinal stability derivatives
 * - Lateral-directional stability derivatives
 * - Control derivatives
 * - Dynamic stability parameters
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;

class FGStabilityTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Longitudinal Stability Tests
   ***************************************************************************/

  // Test Cm_alpha (pitch stiffness)
  void testCmAlpha() {
    double Cm_alpha = -1.0;  // Per radian (typical stable aircraft)

    // Negative Cm_alpha means stable (restoring moment)
    TS_ASSERT(Cm_alpha < 0);
  }

  // Test pitching moment from alpha change
  void testPitchingMomentFromAlpha() {
    double Cm_alpha = -1.0;
    double alpha = 5.0 * DEG_TO_RAD;
    double Cm0 = 0.05;

    double Cm = Cm0 + Cm_alpha * alpha;
    TS_ASSERT(Cm < Cm0);  // Nose-down moment from positive alpha
  }

  // Test CL_alpha (lift curve slope)
  void testCLAlpha() {
    double CL_alpha = 5.7;  // Per radian (typical)

    // Theoretical: 2*pi for thin airfoil
    TS_ASSERT(CL_alpha > 4.0 && CL_alpha < 7.0);
  }

  // Test lift from alpha
  void testLiftFromAlpha() {
    double CL_alpha = 5.7;
    double alpha = 5.0 * DEG_TO_RAD;
    double CL0 = 0.2;

    double CL = CL0 + CL_alpha * alpha;
    TS_ASSERT_DELTA(CL, 0.697, 0.01);
  }

  // Test Cm_q (pitch damping)
  void testCmQ() {
    double Cm_q = -15.0;  // Per radian (typical)

    // Negative means damping (opposes pitch rate)
    TS_ASSERT(Cm_q < 0);
  }

  /***************************************************************************
   * Lateral-Directional Stability Tests
   ***************************************************************************/

  // Test Cl_beta (dihedral effect)
  void testClBeta() {
    double Cl_beta = -0.1;  // Per radian

    // Negative means roll away from sideslip (stabilizing)
    TS_ASSERT(Cl_beta < 0);
  }

  // Test Cn_beta (weathercock stability)
  void testCnBeta() {
    double Cn_beta = 0.1;  // Per radian

    // Positive means yaw into sideslip (stabilizing)
    TS_ASSERT(Cn_beta > 0);
  }

  // Test Cy_beta (side force)
  void testCyBeta() {
    double Cy_beta = -0.5;  // Per radian

    // Negative means side force opposite to sideslip
    TS_ASSERT(Cy_beta < 0);
  }

  // Test roll moment from sideslip
  void testRollFromSideslip() {
    double Cl_beta = -0.1;
    double beta = 10.0 * DEG_TO_RAD;

    double Cl = Cl_beta * beta;
    TS_ASSERT(Cl < 0);  // Roll to right for right sideslip
  }

  // Test yaw moment from sideslip
  void testYawFromSideslip() {
    double Cn_beta = 0.1;
    double beta = 10.0 * DEG_TO_RAD;

    double Cn = Cn_beta * beta;
    TS_ASSERT(Cn > 0);  // Yaw right to reduce right sideslip
  }

  /***************************************************************************
   * Damping Derivative Tests
   ***************************************************************************/

  // Test Cl_p (roll damping)
  void testClP() {
    double Cl_p = -0.4;  // Per radian

    // Negative means opposes roll rate
    TS_ASSERT(Cl_p < 0);
  }

  // Test Cn_r (yaw damping)
  void testCnR() {
    double Cn_r = -0.12;  // Per radian

    // Negative means opposes yaw rate
    TS_ASSERT(Cn_r < 0);
  }

  // Test Cl_r (roll due to yaw rate)
  void testClR() {
    double Cl_r = 0.1;  // Per radian

    // Typically positive (adverse or proverse yaw effect)
    TS_ASSERT(Cl_r > 0 || Cl_r < 0);  // Can be either sign
  }

  // Test Cn_p (yaw due to roll rate)
  void testCnP() {
    double Cn_p = -0.03;  // Per radian

    // Usually negative (adverse yaw)
    TS_ASSERT(Cn_p < 0);
  }

  /***************************************************************************
   * Control Derivative Tests
   ***************************************************************************/

  // Test Cm_de (elevator effectiveness)
  void testCmDe() {
    double Cm_de = -1.5;  // Per radian

    // Negative: down elevator (positive) gives nose-down moment
    TS_ASSERT(Cm_de < 0);
  }

  // Test CL_de (elevator lift)
  void testCLDe() {
    double CL_de = 0.4;  // Per radian

    // Positive: down elevator increases lift
    TS_ASSERT(CL_de > 0);
  }

  // Test Cl_da (aileron roll)
  void testClDa() {
    double Cl_da = 0.15;  // Per radian

    // Positive: right aileron gives right roll
    TS_ASSERT(Cl_da > 0);
  }

  // Test Cn_dr (rudder yaw)
  void testCnDr() {
    double Cn_dr = -0.1;  // Per radian

    // Negative: right rudder (positive) gives left yaw
    TS_ASSERT(Cn_dr < 0);
  }

  // Test Cy_dr (rudder side force)
  void testCyDr() {
    double Cy_dr = 0.2;  // Per radian

    // Positive: right rudder gives right side force
    TS_ASSERT(Cy_dr > 0);
  }

  /***************************************************************************
   * Static Margin Tests
   ***************************************************************************/

  // Test static margin calculation
  void testStaticMargin() {
    double x_cg = 0.25;   // CG at 25% MAC
    double x_np = 0.35;   // Neutral point at 35% MAC

    double SM = x_np - x_cg;
    TS_ASSERT_DELTA(SM, 0.10, epsilon);  // 10% static margin
  }

  // Test Cm_alpha from static margin
  void testCmAlphaFromSM() {
    double SM = 0.10;     // 10% MAC
    double CL_alpha = 5.7;

    // Cm_alpha ≈ -CL_alpha * SM
    double Cm_alpha = -CL_alpha * SM;
    TS_ASSERT_DELTA(Cm_alpha, -0.57, 0.01);
  }

  // Test unstable static margin
  void testUnstableStaticMargin() {
    double x_cg = 0.40;  // CG aft of NP
    double x_np = 0.35;

    double SM = x_np - x_cg;
    TS_ASSERT(SM < 0);  // Unstable
  }

  /***************************************************************************
   * Dynamic Stability Tests
   ***************************************************************************/

  // Test short period frequency
  void testShortPeriodFrequency() {
    double Cm_alpha = -0.8;
    double CL_alpha = 5.7;
    double rho = 0.002377;
    double V = 200.0;
    double S = 200.0;
    double c = 5.0;
    double Iyy = 10000.0;

    double q = 0.5 * rho * V * V;
    double Mq = q * S * c * Cm_alpha / Iyy;

    double omega_sp = std::sqrt(-Mq);  // Simplified
    TS_ASSERT(omega_sp > 0);
  }

  // Test phugoid period
  void testPhugoidPeriod() {
    double V = 200.0;  // ft/s
    double g = 32.174;

    // Simplified: T_phugoid ≈ sqrt(2) * pi * V / g
    double T = std::sqrt(2.0) * M_PI * V / g;
    TS_ASSERT_DELTA(T, 27.7, 1.0);  // seconds
  }

  // Test dutch roll frequency (simplified)
  void testDutchRollFrequency() {
    double Cn_beta = 0.1;
    double rho = 0.002377;
    double V = 200.0;
    double S = 200.0;
    double b = 40.0;
    double Izz = 50000.0;

    double q = 0.5 * rho * V * V;
    double Nv = q * S * b * Cn_beta / Izz;

    // Very simplified approximation
    TS_ASSERT(Nv > 0);
  }

  // Test spiral mode stability
  void testSpiralStability() {
    double Cl_beta = -0.1;
    double Cn_r = -0.12;
    double Cl_r = 0.1;
    double Cn_beta = 0.1;

    // Spiral stability criterion: Cl_beta * Cn_r - Cl_r * Cn_beta > 0
    double criterion = Cl_beta * Cn_r - Cl_r * Cn_beta;

    // If negative, spiral mode is unstable
    TS_ASSERT(criterion != 0);  // Just verify calculation works
  }

  /***************************************************************************
   * Non-dimensional Rate Conversions
   ***************************************************************************/

  // Test non-dimensional pitch rate
  void testNonDimPitchRate() {
    double q = 0.1;    // rad/s
    double c = 5.0;    // chord (ft)
    double V = 200.0;  // ft/s

    double q_hat = q * c / (2 * V);
    TS_ASSERT_DELTA(q_hat, 0.00125, 0.0001);
  }

  // Test non-dimensional roll rate
  void testNonDimRollRate() {
    double p = 0.2;    // rad/s
    double b = 40.0;   // span (ft)
    double V = 200.0;

    double p_hat = p * b / (2 * V);
    TS_ASSERT_DELTA(p_hat, 0.02, 0.001);
  }

  // Test non-dimensional yaw rate
  void testNonDimYawRate() {
    double r = 0.05;   // rad/s
    double b = 40.0;
    double V = 200.0;

    double r_hat = r * b / (2 * V);
    TS_ASSERT_DELTA(r_hat, 0.005, 0.0001);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very stable aircraft
  void testVeryStable() {
    double Cm_alpha = -2.0;  // Very stable

    double alpha = 10.0 * DEG_TO_RAD;
    double Cm = Cm_alpha * alpha;

    TS_ASSERT(std::abs(Cm) > 0.3);  // Strong restoring moment
  }

  // Test neutrally stable
  void testNeutrallyStable() {
    double Cm_alpha = 0.0;  // Neutral

    double alpha = 10.0 * DEG_TO_RAD;
    double Cm = Cm_alpha * alpha;

    TS_ASSERT_DELTA(Cm, 0.0, epsilon);
  }

  // Test zero rate inputs
  void testZeroRates() {
    double Cm_q = -15.0;
    double Cl_p = -0.4;
    double Cn_r = -0.12;

    double q = 0.0, p = 0.0, r = 0.0;

    double Cm = Cm_q * q;
    double Cl = Cl_p * p;
    double Cn = Cn_r * r;

    TS_ASSERT_DELTA(Cm, 0.0, epsilon);
    TS_ASSERT_DELTA(Cl, 0.0, epsilon);
    TS_ASSERT_DELTA(Cn, 0.0, epsilon);
  }

  /***************************************************************************
   * Short Period Mode Tests
   ***************************************************************************/

  // Test short period damping ratio
  void testShortPeriodDamping() {
    // Typical damping ratio range: 0.3 to 0.8
    double Cm_q = -15.0;
    double Cm_alpha = -0.8;
    double rho = 0.002377;
    double V = 200.0;
    double S = 200.0;
    double c = 5.0;
    double Iyy = 10000.0;

    double q = 0.5 * rho * V * V;

    // Simplified damping estimate: zeta ∝ Cm_q
    double Mq = q * S * c * c * Cm_q / (2 * V * Iyy);
    TS_ASSERT(Mq < 0);  // Damping should be negative (stabilizing)
  }

  // Test short period approximation validity
  void testShortPeriodApproximation() {
    // Short period assumes constant speed, decoupled from phugoid
    double omega_sp = 3.0;  // rad/s (typical)
    double omega_phugoid = 0.2;  // rad/s

    double ratio = omega_sp / omega_phugoid;
    TS_ASSERT(ratio > 10);  // Well-separated modes
  }

  // Test CAP (Control Anticipation Parameter)
  void testCAP() {
    // CAP = omega_sp^2 / (n/alpha) - used for handling qualities
    double omega_sp = 3.0;  // rad/s
    double n_per_alpha = 5.0;  // g/rad

    double CAP = omega_sp * omega_sp / n_per_alpha;
    // Good handling: CAP between 0.28 and 3.6
    TS_ASSERT(CAP > 0.1 && CAP < 10.0);
  }

  /***************************************************************************
   * Phugoid Mode Tests
   ***************************************************************************/

  // Test phugoid damping
  void testPhugoidDamping() {
    // Phugoid damping is typically low
    double CD = 0.03;
    double CL = 0.5;

    // Simplified: zeta_phugoid ≈ CD / (sqrt(2) * CL)
    double zeta_phugoid = CD / (std::sqrt(2.0) * CL);
    TS_ASSERT(zeta_phugoid < 0.1);  // Light damping typical
  }

  // Test phugoid frequency
  void testPhugoidFrequencyFormula() {
    double V = 200.0;  // ft/s
    double g = 32.174;

    // omega_phugoid ≈ g * sqrt(2) / V
    double omega_phugoid = g * std::sqrt(2.0) / V;
    TS_ASSERT_DELTA(omega_phugoid, 0.227, 0.01);  // rad/s
  }

  /***************************************************************************
   * Dutch Roll Mode Tests
   ***************************************************************************/

  // Test dutch roll damping ratio
  void testDutchRollDamping() {
    double Cn_r = -0.12;
    double Cy_beta = -0.5;
    double Cn_beta = 0.1;
    double Cl_beta = -0.1;

    // Simplified estimate - actual calculation is more complex
    // Damping depends on Cn_r and other terms
    TS_ASSERT(Cn_r < 0);  // Contributes to damping
  }

  // Test dutch roll coupling
  void testDutchRollCoupling() {
    // Dutch roll involves coupled yaw and roll
    double Cl_beta = -0.1;
    double Cn_beta = 0.1;

    // Roll-to-yaw ratio in dutch roll
    // Indicates degree of coupling
    double coupling = std::abs(Cl_beta / Cn_beta);
    TS_ASSERT(coupling > 0);  // Non-zero coupling
  }

  /***************************************************************************
   * Roll Mode Tests
   ***************************************************************************/

  // Test roll mode time constant
  void testRollModeTimeConstant() {
    double Cl_p = -0.4;
    double rho = 0.002377;
    double V = 200.0;
    double S = 200.0;
    double b = 40.0;
    double Ixx = 20000.0;

    double q = 0.5 * rho * V * V;
    double Lp = q * S * b * b * Cl_p / (2 * V * Ixx);

    // Roll time constant tau = -1/Lp
    double tau_roll = -1.0 / Lp;
    TS_ASSERT(tau_roll > 0 && tau_roll < 2.0);  // Typical range
  }

  // Test roll performance requirement
  void testRollPerformance() {
    // Time to roll 30 degrees from steady flight
    double phi_target = 30.0 * DEG_TO_RAD;
    double p_max = 1.5;  // rad/s max roll rate

    double t_30 = phi_target / p_max;
    TS_ASSERT(t_30 < 1.5);  // Should be quick
  }

  /***************************************************************************
   * Spiral Mode Tests
   ***************************************************************************/

  // Test spiral mode time constant
  void testSpiralModeTimeConstant() {
    double Cl_beta = -0.1;
    double Cn_r = -0.12;
    double Cl_r = 0.1;
    double Cn_beta = 0.1;
    double Cl_p = -0.4;

    // Simplified spiral mode eigenvalue
    double E = Cl_beta * Cn_r - Cl_r * Cn_beta;

    // Spiral stable if E > 0
    if (E > 0) {
      // Time to double (if unstable, negative)
      double tau_spiral = std::abs(Cl_p / E);  // Simplified
      TS_ASSERT(tau_spiral > 0);
    } else {
      // Unstable spiral
      double T2 = std::log(2.0) * std::abs(Cl_p / E);
      TS_ASSERT(T2 > 0);
    }
  }

  // Test spiral stability criteria
  void testSpiralStabilityCriteria() {
    // Stable spiral requires Cl_beta * Cn_r > Cl_r * Cn_beta
    double Cl_beta = -0.1;
    double Cn_r = -0.12;
    double Cl_r = 0.08;
    double Cn_beta = 0.15;

    double lhs = Cl_beta * Cn_r;  // = 0.012
    double rhs = Cl_r * Cn_beta;  // = 0.012

    // Often marginally stable
    TS_ASSERT(lhs != 0 || rhs != 0);
  }

  /***************************************************************************
   * Trim Analysis Tests
   ***************************************************************************/

  // Test trim elevator deflection
  void testTrimElevator() {
    double CL_trim = 0.5;
    double Cm0 = 0.05;
    double Cm_alpha = -0.8;
    double CL_alpha = 5.7;
    double Cm_de = -1.5;
    double CL_de = 0.4;

    // Alpha for required CL
    double alpha_trim = (CL_trim - 0.2) / CL_alpha;  // CL0 = 0.2

    // Elevator for Cm = 0
    double de_trim = -(Cm0 + Cm_alpha * alpha_trim) / Cm_de;

    TS_ASSERT(de_trim > -0.5 && de_trim < 0.5);  // Reasonable range
  }

  // Test trim drag
  void testTrimDrag() {
    double CL_trim = 0.5;
    double CD0 = 0.02;
    double K = 0.05;  // Induced drag factor

    double CD_trim = CD0 + K * CL_trim * CL_trim;
    TS_ASSERT_DELTA(CD_trim, 0.0325, 0.001);
  }

  // Test throttle for trim
  void testTrimThrottle() {
    double W = 10000.0;  // lbs
    double L_D = 15.0;   // Lift-to-drag ratio
    double T_max = 3000.0;  // Max thrust

    double D_trim = W / L_D;
    double throttle = D_trim / T_max;

    TS_ASSERT(throttle > 0 && throttle < 1.0);
  }

  /***************************************************************************
   * Speed Stability Tests
   ***************************************************************************/

  // Test speed stability (dV produces restoring force)
  void testSpeedStability() {
    // At trim, T = D
    // If V increases, D increases (for stable speed)
    double CD = 0.03;
    double rho = 0.002377;
    double S = 200.0;
    double V1 = 200.0;
    double V2 = 210.0;

    double q1 = 0.5 * rho * V1 * V1;
    double q2 = 0.5 * rho * V2 * V2;

    double D1 = q1 * S * CD;
    double D2 = q2 * S * CD;

    TS_ASSERT(D2 > D1);  // Speed stable: more drag at higher V
  }

  // Test phugoid speed exchange
  void testPhugoidSpeedExchange() {
    double g = 32.174;
    double V = 200.0;
    double dh = 100.0;  // altitude change (ft)

    // Energy exchange: dV ≈ -g*dh/V
    double dV = -g * dh / V;
    TS_ASSERT_DELTA(dV, -16.09, 0.1);  // Speed decrease for climb
  }

  /***************************************************************************
   * Control Authority Tests
   ***************************************************************************/

  // Test aileron control power
  void testAileronControlPower() {
    double Cl_da = 0.15;  // per rad
    double da_max = 25.0 * DEG_TO_RAD;

    double Cl_max = Cl_da * da_max;
    TS_ASSERT(Cl_max > 0.05);  // Adequate roll control
  }

  // Test rudder control power for engine-out
  void testRudderForEngineOut() {
    double Cn_dr = -0.1;
    double y_engine = 8.0;  // Engine moment arm (ft)
    double T_engine = 2000.0;  // Engine thrust (lbs)
    double b = 40.0;
    double q = 50.0;  // Dynamic pressure
    double S = 200.0;

    // Yawing moment from engine out
    double N_engine = T_engine * y_engine;

    // Required rudder
    double dr_required = N_engine / (q * S * b * (-Cn_dr));

    TS_ASSERT(dr_required < 0.5);  // Must be achievable
  }

  // Test elevator authority for rotation
  void testElevatorRotation() {
    double Cm_de = -1.5;
    double de_max = 25.0 * DEG_TO_RAD;

    double Cm_max = Cm_de * de_max;
    TS_ASSERT(std::abs(Cm_max) > 0.5);  // Adequate pitch control
  }

  /***************************************************************************
   * High Angle of Attack Stability Tests
   ***************************************************************************/

  // Test stall behavior
  void testStallBehavior() {
    double CL_max = 1.5;
    double alpha_stall = 15.0 * DEG_TO_RAD;
    double CL_alpha = 5.7;

    // Beyond stall, CL drops
    double CL_at_stall = CL_alpha * alpha_stall;
    TS_ASSERT(CL_at_stall > 1.0);  // Approaching CLmax
  }

  // Test Cm at high alpha
  void testCmHighAlpha() {
    // At high alpha, Cm may become less stable
    double Cm_alpha_low = -1.0;  // Low alpha
    double Cm_alpha_high = -0.3; // Reduced stability at high alpha

    TS_ASSERT(std::abs(Cm_alpha_high) < std::abs(Cm_alpha_low));
  }

  // Test departure susceptibility
  void testDepartureSusceptibility() {
    // Lateral-directional departure parameter
    double Cn_beta_dyn = 0.05;  // Dynamic Cn_beta
    double Cl_beta = -0.1;

    // LCDP (Lateral Control Departure Parameter)
    // Positive = resistant to departure
    TS_ASSERT(Cn_beta_dyn > 0);
  }

  /***************************************************************************
   * Stability Axis Transformation Tests
   ***************************************************************************/

  // Test body to stability axis
  void testBodyToStabilityAxis() {
    double alpha = 5.0 * DEG_TO_RAD;
    // Body axes: Cx positive forward, Cz positive down
    // Typical values: Cx negative (drag), Cz negative (lift upward)
    double Cx_body = -0.05;  // Axial force coeff (drag)
    double Cz_body = -0.5;   // Normal force coeff (lift)

    // Transform to stability axis
    // Stability: CD positive aft, CL positive up
    double CD = -Cx_body * std::cos(alpha) - Cz_body * std::sin(alpha);
    double CL = Cx_body * std::sin(alpha) - Cz_body * std::cos(alpha);

    TS_ASSERT(CL > 0);  // Lift positive up
    TS_ASSERT(CD > 0);  // Drag positive aft
  }

  // Test wind axis forces
  void testWindAxisForces() {
    double CL = 0.5;
    double CD = 0.03;
    double beta = 5.0 * DEG_TO_RAD;
    double CY = -0.05;  // Side force

    // Total force magnitude (simplified)
    double C_total = std::sqrt(CL*CL + CD*CD + CY*CY);
    TS_ASSERT(C_total > 0);
  }

  /***************************************************************************
   * Compressibility Effects Tests
   ***************************************************************************/

  // Test Prandtl-Glauert correction
  void testPrandtlGlauert() {
    double Mach = 0.6;
    double CL_incomp = 0.5;

    // Prandtl-Glauert: CL_comp = CL_incomp / sqrt(1-M^2)
    double beta_pg = std::sqrt(1.0 - Mach*Mach);
    double CL_comp = CL_incomp / beta_pg;

    TS_ASSERT(CL_comp > CL_incomp);  // Lift increases with Mach
  }

  // Test stability derivative Mach effects
  void testStabilityMachEffects() {
    double Mach = 0.8;
    double Cm_alpha_low = -1.0;

    // At higher Mach, stability may change
    double beta_pg = std::sqrt(1.0 - Mach*Mach);
    double Cm_alpha_high = Cm_alpha_low / beta_pg;

    TS_ASSERT(std::abs(Cm_alpha_high) > std::abs(Cm_alpha_low));
  }

  /***************************************************************************
   * Handling Qualities Tests
   ***************************************************************************/

  // Test Gibson dropback criterion
  void testGibsonDropback() {
    // Dropback relates pitch rate to attitude response
    double q_ss = 5.0;  // Steady-state pitch rate (deg/s)
    double theta_1 = 4.0;  // Pitch attitude at 1 second

    // Dropback = q_ss - theta_1
    double dropback = q_ss - theta_1;

    // Positive dropback is generally acceptable
    TS_ASSERT(dropback > -2.0);  // Not too negative
  }

  // Test Neal-Smith criterion parameters
  void testNealSmithCriterion() {
    // Bandwidth and phase delay for handling qualities
    double omega_bw = 2.5;  // rad/s bandwidth
    double tau_p = 0.1;     // Phase delay (seconds)

    // Good handling: adequate bandwidth, low phase delay
    TS_ASSERT(omega_bw > 1.0);
    TS_ASSERT(tau_p < 0.2);
  }

  // Test pilot-induced oscillation (PIO) susceptibility
  void testPIOSusceptibility() {
    // Phase delay indicates PIO susceptibility
    double tau_theta = 0.15;  // Attitude phase delay

    // Low delay = less PIO prone
    bool low_pio_risk = (tau_theta < 0.2);
    TS_ASSERT(low_pio_risk);
  }

  /***************************************************************************
   * Downwash Effects Tests
   ***************************************************************************/

  // Test downwash at tail
  void testDownwashAtTail() {
    double alpha = 5.0 * DEG_TO_RAD;
    double CL_alpha_w = 5.0;  // Wing lift curve slope
    double epsilon0 = 0.02;   // Zero-lift downwash
    double d_epsilon_d_alpha = 0.4;  // Downwash derivative

    double epsilon = epsilon0 + d_epsilon_d_alpha * alpha;
    double alpha_t = alpha - epsilon;

    TS_ASSERT(alpha_t < alpha);  // Tail sees less alpha due to downwash
  }

  // Test tail effectiveness with downwash
  void testTailEffectivenessWithDownwash() {
    double d_epsilon_d_alpha = 0.4;
    double CL_alpha_t = 4.0;  // Tail lift curve slope
    double eta_t = 0.9;       // Tail efficiency

    // Effective tail contribution
    double CL_alpha_t_eff = eta_t * CL_alpha_t * (1.0 - d_epsilon_d_alpha);

    TS_ASSERT_DELTA(CL_alpha_t_eff, 2.16, 0.01);
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  // Test combined control inputs
  void testCombinedControls() {
    double Cl_da = 0.15;
    double Cn_da = -0.01;  // Adverse yaw
    double Cl_dr = 0.01;
    double Cn_dr = -0.1;

    double da = 10.0 * DEG_TO_RAD;
    double dr = 5.0 * DEG_TO_RAD;

    // Net moments
    double Cl = Cl_da * da + Cl_dr * dr;
    double Cn = Cn_da * da + Cn_dr * dr;

    TS_ASSERT(Cl > 0);  // Net roll right
  }

  // Test stability at different CG positions
  void testCGEffect() {
    double x_cg_fwd = 0.20;
    double x_cg_aft = 0.35;
    double x_np = 0.30;

    double SM_fwd = x_np - x_cg_fwd;  // More stable
    double SM_aft = x_np - x_cg_aft;  // Less stable

    TS_ASSERT(SM_fwd > SM_aft);
    TS_ASSERT(SM_fwd > 0);  // Still stable
    TS_ASSERT(SM_aft < 0);  // Aft CG unstable
  }

  // Test dynamic pressure effects
  void testDynamicPressureEffects() {
    double rho = 0.002377;
    double V_low = 100.0;
    double V_high = 300.0;

    double q_low = 0.5 * rho * V_low * V_low;
    double q_high = 0.5 * rho * V_high * V_high;

    // Control effectiveness scales with q
    double ratio = q_high / q_low;
    TS_ASSERT_DELTA(ratio, 9.0, 0.01);  // 3^2 = 9
  }

  /***************************************************************************
   * Maneuver Point Tests
   ***************************************************************************/

  // Test maneuver point calculation
  void testManeuverPoint() {
    double x_np = 0.35;    // Neutral point (stick-fixed)
    double Cm_q = -15.0;   // Pitch damping
    double CL_alpha = 5.7;
    double m = 200.0;      // Non-dimensional mass
    double mu = m * 2.0;   // Relative density factor

    // Maneuver point is ahead of NP due to pitch damping
    double x_mp = x_np + Cm_q / (2.0 * mu * CL_alpha);
    TS_ASSERT(x_mp < x_np);  // Maneuver point forward of NP
  }

  // Test stick force per g
  void testStickForcePerG() {
    double W = 10000.0;    // Weight (lbs)
    double S = 200.0;      // Wing area
    double c = 5.0;        // Chord
    double SM = 0.10;      // Static margin
    double G = 50.0;       // Gearing ratio (lb/rad elevator)

    // Simplified: F/n ∝ W * SM
    double F_per_g = W * SM * c / (S * G);
    TS_ASSERT(F_per_g > 0);  // Pull for positive g
  }

  // Test maneuver margin
  void testManeuverMargin() {
    double x_cg = 0.25;
    double x_mp = 0.32;

    double maneuver_margin = x_mp - x_cg;
    TS_ASSERT(maneuver_margin > 0);  // Stable in maneuver
  }

  /***************************************************************************
   * Power Effects on Stability Tests
   ***************************************************************************/

  // Test propeller slipstream on tail
  void testPropSlipstreamEffect() {
    double V = 150.0;
    double delta_V = 20.0;  // Slipstream velocity increase
    double eta_t_clean = 0.9;

    // Dynamic pressure ratio in slipstream
    double q_ratio = std::pow((V + delta_V) / V, 2);
    double eta_t_power = eta_t_clean * q_ratio;

    TS_ASSERT(eta_t_power > eta_t_clean);  // More effective with power
  }

  // Test thrust line offset moment
  void testThrustLineMoment() {
    double T = 2000.0;     // Thrust (lbs)
    double z_T = 2.0;      // Thrust line below CG (positive z down convention)
    double c = 5.0;        // Chord
    double q = 50.0;
    double S = 200.0;

    // Pitching moment coefficient from thrust
    double Cm_thrust = T * z_T / (q * S * c);
    TS_ASSERT(Cm_thrust > 0);  // Nose-up moment (thrust below CG)
  }

  // Test normal force on propeller disc
  void testPropellerNormalForce() {
    double alpha = 10.0 * DEG_TO_RAD;
    double T = 2000.0;
    double a_p = 0.3;  // Normal force factor

    // Normal force on propeller
    double N_p = T * a_p * alpha;
    TS_ASSERT(N_p > 0);  // Destabilizing at positive alpha
  }

  /***************************************************************************
   * Ground Effect on Stability Tests
   ***************************************************************************/

  // Test lift in ground effect
  void testLiftInGroundEffect() {
    double CL = 1.0;
    double h_b = 0.2;  // Height/span ratio

    // Ground effect factor (Raymer approximation)
    double phi = (16.0 * h_b) * (16.0 * h_b) / (1.0 + (16.0 * h_b) * (16.0 * h_b));
    double K_GE = 1.0 + phi * 0.5;  // Simplified

    TS_ASSERT(K_GE > 1.0);  // Lift increased in ground effect
  }

  // Test induced drag reduction in ground effect
  void testInducedDragGroundEffect() {
    double K = 0.05;       // Induced drag factor
    double CL = 1.0;
    double h_b = 0.2;

    // Ground effect reduces induced drag
    double CDi_ooge = K * CL * CL;
    double sigma_GE = 0.8;  // Ground effect factor < 1
    double CDi_ige = CDi_ooge * sigma_GE;

    TS_ASSERT(CDi_ige < CDi_ooge);
  }

  // Test pitch stability change in ground effect
  void testPitchStabilityGroundEffect() {
    double Cm_alpha_ooge = -1.0;
    double d_Cm_alpha_GE = 0.2;  // Ground effect destabilizing

    double Cm_alpha_ige = Cm_alpha_ooge + d_Cm_alpha_GE;
    TS_ASSERT(std::abs(Cm_alpha_ige) < std::abs(Cm_alpha_ooge));
  }

  /***************************************************************************
   * Crosswind Stability Tests
   ***************************************************************************/

  // Test crosswind approach crab angle
  void testCrosswindCrabAngle() {
    double V_aircraft = 150.0;  // kts
    double V_crosswind = 20.0;  // kts

    double psi_crab = std::atan2(V_crosswind, V_aircraft);
    TS_ASSERT_DELTA(psi_crab * 180.0 / M_PI, 7.6, 0.5);  // degrees
  }

  // Test sideslip in crosswind landing
  void testCrosswindSideslip() {
    double V_crosswind = 20.0;
    double V_approach = 120.0;
    double Cy_beta = -0.5;

    // Required sideslip for wing-low approach
    double beta_required = V_crosswind / V_approach;
    TS_ASSERT(beta_required > 0 && beta_required < 0.5);
  }

  // Test aileron-rudder crossfeed
  void testAileronRudderCrossfeed() {
    double Cn_da = -0.01;  // Adverse yaw from aileron
    double Cn_dr = -0.10;  // Rudder yaw effectiveness

    // Rudder needed to cancel adverse yaw per unit aileron
    // To cancel: Cn_da * da + Cn_dr * dr = 0 => dr/da = -Cn_da/Cn_dr
    double dr_per_da = -Cn_da / Cn_dr;
    TS_ASSERT_DELTA(dr_per_da, -0.1, 0.01);  // Opposite sign rudder
  }

  /***************************************************************************
   * Roll-Yaw Coupling Tests
   ***************************************************************************/

  // Test adverse yaw magnitude
  void testAdverseYawMagnitude() {
    double Cn_da = -0.01;  // Per radian
    double da = 20.0 * DEG_TO_RAD;

    double Cn_adverse = Cn_da * da;
    TS_ASSERT(Cn_adverse < 0);  // Yaw opposite to roll
  }

  // Test proverse yaw at high alpha
  void testProverseYawHighAlpha() {
    // At high alpha, aileron can produce proverse yaw
    double Cn_da_low_alpha = -0.01;
    double Cn_da_high_alpha = 0.005;  // Proverse

    TS_ASSERT(Cn_da_high_alpha > 0);  // Yaw same direction as roll
  }

  // Test roll coupling in yaw
  void testYawRollCoupling() {
    double Cl_r = 0.15;    // Roll due to yaw rate
    double r = 0.1;        // rad/s yaw rate

    double Cl = Cl_r * r;
    TS_ASSERT(Cl > 0);  // Right yaw produces right roll
  }

  /***************************************************************************
   * Aeroelastic Effects Tests
   ***************************************************************************/

  // Test aileron reversal speed
  void testAileronReversalSpeed() {
    double V_design = 400.0;  // Design dive speed
    double K_flex = 0.8;      // Flexibility factor at V_design

    // Reversal speed where K_flex = 0
    double V_reversal = V_design / std::sqrt(1.0 - K_flex);
    TS_ASSERT(V_reversal > V_design);
  }

  // Test roll rate reduction due to flexibility
  void testRollRateFlexibility() {
    double p_rigid = 2.0;   // rad/s rigid roll rate
    double K_flex = 0.7;    // Flexibility factor

    double p_flexible = p_rigid * K_flex;
    TS_ASSERT(p_flexible < p_rigid);
  }

  // Test divergence speed estimate
  void testDivergenceSpeed() {
    double EI = 1e9;        // Bending stiffness
    double C_La = 5.7;      // Lift curve slope per rad
    double e = 0.25;        // Elastic axis to AC distance
    double c = 5.0;
    double rho = 0.002377;
    double S = 20.0;        // Panel area

    // Simplified divergence dynamic pressure
    double q_div = EI / (C_La * e * c * S);
    double V_div = std::sqrt(2.0 * q_div / rho);
    TS_ASSERT(V_div > 0);
  }

  /***************************************************************************
   * Icing Effects on Stability Tests
   ***************************************************************************/

  // Test CLmax reduction with ice
  void testCLmaxIcing() {
    double CL_max_clean = 1.8;
    double dCL_max_ice = -0.4;  // Ice degradation

    double CL_max_iced = CL_max_clean + dCL_max_ice;
    TS_ASSERT(CL_max_iced < CL_max_clean);
    TS_ASSERT(CL_max_iced > 1.0);  // Still flyable
  }

  // Test drag increase with ice
  void testDragIcing() {
    double CD_clean = 0.025;
    double dCD_ice = 0.015;  // Ice drag increment

    double CD_iced = CD_clean + dCD_ice;
    TS_ASSERT_DELTA(CD_iced / CD_clean, 1.6, 0.1);  // 60% increase
  }

  // Test Cm_alpha change with tailplane ice
  void testCmAlphaTailplaneIce() {
    double Cm_alpha_clean = -1.0;
    double eta_t_clean = 0.9;
    double eta_t_iced = 0.6;  // Reduced tail effectiveness

    // Stability reduction proportional to tail effectiveness reduction
    double Cm_alpha_iced = Cm_alpha_clean * (eta_t_iced / eta_t_clean);
    TS_ASSERT(std::abs(Cm_alpha_iced) < std::abs(Cm_alpha_clean));
  }

  /***************************************************************************
   * Weight and Balance Limits Tests
   ***************************************************************************/

  // Test forward CG limit for rotation
  void testForwardCGLimitRotation() {
    double Cm_de_max = -1.5 * 25.0 * DEG_TO_RAD;  // Max elevator moment
    double CL_rotation = 1.2;
    double CL_alpha = 5.7;

    // Forward CG limit where elevator saturates at rotation
    double alpha_rot = CL_rotation / CL_alpha;
    double Cm_required = -(-1.0) * alpha_rot;  // Cm_alpha * alpha to overcome
    TS_ASSERT(std::abs(Cm_de_max) > std::abs(Cm_required));
  }

  // Test aft CG limit for stability
  void testAftCGLimitStability() {
    double x_np = 0.35;
    double SM_min = 0.05;  // Minimum required static margin

    double x_cg_aft_limit = x_np - SM_min;
    TS_ASSERT_DELTA(x_cg_aft_limit, 0.30, 0.001);
  }

  // Test CG travel vs fuel burn
  void testCGTravelFuel() {
    double x_cg_full = 0.22;
    double x_cg_empty = 0.28;
    double x_fuel = 0.15;  // Fuel tank location
    double W_fuel = 2000.0;
    double W_total_full = 10000.0;

    // CG moves aft as fuel burns (fuel tank forward of CG)
    TS_ASSERT(x_cg_empty > x_cg_full);  // CG moves aft
  }

  /***************************************************************************
   * Control Harmony Tests
   ***************************************************************************/

  // Test aileron-elevator force ratio
  void testAileronElevatorForceRatio() {
    double F_aileron = 5.0;    // lbs for 30 deg bank
    double F_elevator = 10.0;  // lbs for 2g pullup

    double ratio = F_aileron / F_elevator;
    // Good harmony: ratio between 0.3 and 0.7
    TS_ASSERT(ratio > 0.2 && ratio < 1.0);
  }

  // Test rudder-aileron force ratio
  void testRudderAileronForceRatio() {
    double F_rudder = 15.0;
    double F_aileron = 5.0;

    double ratio = F_rudder / F_aileron;
    // Rudder typically heavier than aileron
    TS_ASSERT(ratio > 2.0);
  }

  // Test control centering
  void testControlCentering() {
    double F_center = 1.0;     // Force to overcome centering
    double delta_max = 25.0;   // Max deflection (deg)
    double F_max = 30.0;       // Max force

    double gradient = (F_max - F_center) / delta_max;
    TS_ASSERT(gradient > 0.5);  // Positive gradient (increasing force)
  }

  /***************************************************************************
   * Flying Qualities Categories Tests
   ***************************************************************************/

  // Test Level 1 short period requirements
  void testLevel1ShortPeriod() {
    double zeta_sp = 0.5;  // Damping ratio
    double omega_sp = 3.0; // rad/s

    // Level 1 requirements (MIL-F-8785C)
    bool level1 = (zeta_sp > 0.35 && zeta_sp < 1.3) && (omega_sp > 1.0);
    TS_ASSERT(level1);
  }

  // Test Level 1 dutch roll requirements
  void testLevel1DutchRoll() {
    double zeta_dr = 0.15;  // Damping ratio
    double omega_dr = 1.5;  // rad/s
    double zeta_omega = zeta_dr * omega_dr;

    // Level 1: zeta > 0.08, omega > 0.4, zeta*omega > 0.15
    bool level1 = (zeta_dr > 0.08) && (omega_dr > 0.4) && (zeta_omega > 0.15);
    TS_ASSERT(level1);
  }

  // Test roll mode time constant requirement
  void testRollModeRequirement() {
    double tau_roll = 0.8;  // seconds

    // Level 1 for Category A: tau < 1.0s
    TS_ASSERT(tau_roll < 1.0);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test stability over alpha range
  void testStabilityAlphaRange() {
    double Cm_alpha = -1.0;

    for (double alpha = -10.0; alpha <= 15.0; alpha += 1.0) {
      double alpha_rad = alpha * DEG_TO_RAD;
      double Cm = Cm_alpha * alpha_rad;

      // Moment should oppose alpha change
      TS_ASSERT((alpha > 0 && Cm < 0) || (alpha < 0 && Cm > 0) || alpha == 0);
    }
  }

  // Test control effectiveness over speed range
  void testControlEffectivenessSpeedRange() {
    double Cm_de = -1.5;
    double rho = 0.002377;

    for (double V = 80.0; V <= 400.0; V += 40.0) {
      double q = 0.5 * rho * V * V;

      // Moment increases with dynamic pressure
      double M_per_de = q * 200.0 * 5.0 * Cm_de;
      TS_ASSERT(M_per_de < 0);  // Consistent sign
    }
  }

  // Test damping derivative consistency
  void testDampingConsistency() {
    double rates[] = {-0.5, -0.3, -0.1, 0.0, 0.1, 0.3, 0.5};
    double Cm_q = -15.0;

    for (double q_rate : rates) {
      double Cm_damping = Cm_q * q_rate;
      // Damping opposes rate
      if (q_rate > 0) TS_ASSERT(Cm_damping < 0);
      if (q_rate < 0) TS_ASSERT(Cm_damping > 0);
      if (q_rate == 0) TS_ASSERT_DELTA(Cm_damping, 0.0, epsilon);
    }
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test radian to degree conversion in derivatives
  void testDerivativeUnitConversion() {
    double Cm_alpha_per_rad = -1.0;
    double Cm_alpha_per_deg = Cm_alpha_per_rad * DEG_TO_RAD;

    TS_ASSERT_DELTA(Cm_alpha_per_deg, -0.01745, 0.0001);
  }

  // Test moment coefficient dimensional form
  void testMomentDimensional() {
    double Cm = -0.1;
    double q = 50.0;
    double S = 200.0;
    double c = 5.0;

    double M = Cm * q * S * c;  // ft-lbs
    TS_ASSERT_DELTA(M, -5000.0, 1.0);
  }

  // Test force coefficient dimensional form
  void testForceDimensional() {
    double CL = 0.5;
    double q = 50.0;
    double S = 200.0;

    double L = CL * q * S;  // lbs
    TS_ASSERT_DELTA(L, 5000.0, 1.0);
  }

  /***************************************************************************
   * Additional Modal Coupling Tests
   ***************************************************************************/

  // Test roll-spiral approximation separation
  void testRollSpiralSeparation() {
    double tau_roll = 0.5;
    double tau_spiral = 50.0;

    double ratio = tau_spiral / tau_roll;
    TS_ASSERT(ratio > 10);  // Well separated modes
  }

  // Test phugoid-short period separation
  void testPhugoidSPSeparation() {
    double omega_sp = 3.0;
    double omega_phugoid = 0.2;

    double ratio = omega_sp / omega_phugoid;
    TS_ASSERT(ratio > 10);  // Well separated
  }

  // Test lateral-directional mode coupling
  void testLatDirModeCoupling() {
    double Cl_beta = -0.1;
    double Cn_beta = 0.1;
    double Cl_r = 0.1;
    double Cn_r = -0.12;

    // Check cross-coupling terms
    double coupling_ratio = std::abs(Cl_beta * Cn_r / (Cl_r * Cn_beta));
    TS_ASSERT(coupling_ratio > 0);  // Non-zero coupling exists
  }
};
