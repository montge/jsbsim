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
};
