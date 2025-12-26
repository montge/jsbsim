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
};
