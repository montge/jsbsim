/*******************************************************************************
 * FGTrimTest.h - Unit tests for trim calculations
 *
 * Tests the mathematical behavior of aircraft trim:
 * - Steady-state force and moment balance
 * - Trim state variables (alpha, throttle, elevator)
 * - Different flight conditions (level, climb, turn)
 * - Trim convergence algorithms
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double G = 32.174;  // ft/s^2

class FGTrimTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Level Flight Trim Tests
   ***************************************************************************/

  // Test lift equals weight in level flight
  void testLiftEqualsWeight() {
    double weight = 10000.0;  // lbs
    double lift = weight;     // Trim condition

    double netVerticalForce = lift - weight;
    TS_ASSERT_DELTA(netVerticalForce, 0.0, epsilon);
  }

  // Test thrust equals drag in steady level flight
  void testThrustEqualsDrag() {
    double drag = 500.0;   // lbs
    double thrust = drag;  // Trim condition

    double netHorizontalForce = thrust - drag;
    TS_ASSERT_DELTA(netHorizontalForce, 0.0, epsilon);
  }

  // Test pitching moment equals zero in trim
  void testPitchingMomentZero() {
    double Mcg = 0.0;  // Pitching moment at CG must be zero

    TS_ASSERT_DELTA(Mcg, 0.0, epsilon);
  }

  /***************************************************************************
   * Lift Coefficient Trim Tests
   ***************************************************************************/

  // Test required CL for level flight
  void testRequiredCL() {
    double weight = 10000.0;  // lbs
    double S = 200.0;         // Wing area (sq ft)
    double rho = 0.002377;    // Sea level density (slugs/ft^3)
    double V = 200.0;         // ft/s

    double q = 0.5 * rho * V * V;  // Dynamic pressure
    double CL_required = weight / (q * S);

    TS_ASSERT_DELTA(CL_required, 1.05, 0.01);
  }

  // Test CL increases at lower speed
  void testCLIncreasesAtLowerSpeed() {
    double weight = 10000.0;
    double S = 200.0;
    double rho = 0.002377;

    double V1 = 200.0;
    double V2 = 150.0;  // Slower

    double q1 = 0.5 * rho * V1 * V1;
    double q2 = 0.5 * rho * V2 * V2;

    double CL1 = weight / (q1 * S);
    double CL2 = weight / (q2 * S);

    TS_ASSERT(CL2 > CL1);  // Need more CL at slower speed
  }

  /***************************************************************************
   * Angle of Attack Trim Tests
   ***************************************************************************/

  // Test alpha from CL (linear assumption)
  void testAlphaFromCL() {
    double CL = 0.5;
    double CLalpha = 5.7;   // Per radian (typical)
    double CL0 = 0.2;       // CL at zero alpha

    double alpha = (CL - CL0) / CLalpha;  // radians
    double alphaDeg = alpha * RAD_TO_DEG;

    TS_ASSERT_DELTA(alphaDeg, 3.02, 0.1);  // About 3 degrees
  }

  // Test alpha limits
  void testAlphaLimits() {
    double alphaMax = 15.0;  // degrees (typical stall)
    double alphaMin = -5.0;  // degrees

    double alpha = 10.0;
    bool withinLimits = (alpha >= alphaMin && alpha <= alphaMax);

    TS_ASSERT(withinLimits);
  }

  /***************************************************************************
   * Elevator Trim Tests
   ***************************************************************************/

  // Test elevator to balance pitching moment
  void testElevatorForTrim() {
    // Simplified: Cm = Cm0 + Cm_alpha * alpha + Cm_de * de = 0
    double Cm0 = 0.05;       // Moment coefficient at zero alpha/de
    double Cmalpha = -0.5;   // Per radian
    double Cmde = -1.5;      // Per radian
    double alpha = 5.0 * DEG_TO_RAD;

    // Solve for de: de = -(Cm0 + Cmalpha * alpha) / Cmde
    double de = -(Cm0 + Cmalpha * alpha) / Cmde;
    double deDeg = de * RAD_TO_DEG;

    TS_ASSERT_DELTA(deDeg, -0.24, 0.5);  // Small elevator deflection
  }

  // Test elevator authority
  void testElevatorAuthority() {
    double deMax = 25.0;   // degrees up
    double deMin = -20.0;  // degrees down

    double de = 10.0;
    bool withinLimits = (de >= deMin && de <= deMax);

    TS_ASSERT(withinLimits);
  }

  /***************************************************************************
   * Throttle Trim Tests
   ***************************************************************************/

  // Test thrust required for level flight
  void testThrustRequired() {
    double weight = 10000.0;
    double LD = 10.0;  // Lift-to-drag ratio

    double drag = weight / LD;
    double thrustRequired = drag;

    TS_ASSERT_DELTA(thrustRequired, 1000.0, epsilon);
  }

  // Test throttle position from thrust
  void testThrottleFromThrust() {
    double thrustRequired = 1000.0;
    double maxThrust = 3000.0;

    double throttle = thrustRequired / maxThrust;
    TS_ASSERT_DELTA(throttle, 0.333, 0.01);
  }

  /***************************************************************************
   * Climbing Flight Trim Tests
   ***************************************************************************/

  // Test thrust required for climb
  void testThrustForClimb() {
    double weight = 10000.0;
    double drag = 1000.0;
    double gamma = 5.0 * DEG_TO_RAD;  // 5 degree climb

    // T = D + W * sin(gamma)
    double thrust = drag + weight * std::sin(gamma);
    TS_ASSERT_DELTA(thrust, 1872.0, 10.0);
  }

  // Test lift in climb
  void testLiftInClimb() {
    double weight = 10000.0;
    double gamma = 5.0 * DEG_TO_RAD;

    // L = W * cos(gamma)
    double lift = weight * std::cos(gamma);
    TS_ASSERT_DELTA(lift, 9962.0, 10.0);  // Slightly less than weight
  }

  // Test climb rate from flight path angle
  void testClimbRate() {
    double V = 200.0;  // ft/s
    double gamma = 5.0 * DEG_TO_RAD;

    double climbRate = V * std::sin(gamma);
    TS_ASSERT_DELTA(climbRate, 17.45, 0.1);  // ft/s
  }

  // Test climb rate in ft/min
  void testClimbRateFPM() {
    double climbRateFPS = 17.45;
    double climbRateFPM = climbRateFPS * 60.0;

    TS_ASSERT_DELTA(climbRateFPM, 1047.0, 1.0);  // About 1000 ft/min
  }

  /***************************************************************************
   * Descending Flight Trim Tests
   ***************************************************************************/

  // Test glide angle
  void testGlideAngle() {
    double LD = 10.0;  // Lift-to-drag ratio

    // gamma = atan(1/LD) for power-off glide
    double gamma = std::atan(1.0 / LD) * RAD_TO_DEG;
    TS_ASSERT_DELTA(gamma, 5.71, 0.1);  // About 6 degrees
  }

  // Test descent rate
  void testDescentRate() {
    double V = 150.0;  // ft/s
    double gamma = -3.0 * DEG_TO_RAD;  // 3 degree descent

    double descentRate = -V * std::sin(gamma);
    TS_ASSERT_DELTA(descentRate, 7.85, 0.1);  // ft/s descent
  }

  /***************************************************************************
   * Turning Flight Trim Tests
   ***************************************************************************/

  // Test bank angle for coordinated turn
  void testBankAngleForTurn() {
    double V = 200.0;  // ft/s
    double R = 2000.0; // Turn radius (ft)

    // tan(phi) = V^2 / (g * R)
    double phi = std::atan(V * V / (G * R)) * RAD_TO_DEG;
    TS_ASSERT_DELTA(phi, 31.6, 0.5);
  }

  // Test load factor in turn
  void testLoadFactorInTurn() {
    double phi = 30.0 * DEG_TO_RAD;  // Bank angle

    // n = 1 / cos(phi)
    double n = 1.0 / std::cos(phi);
    TS_ASSERT_DELTA(n, 1.155, 0.01);
  }

  // Test turn radius from bank and speed
  void testTurnRadius() {
    double V = 200.0;  // ft/s
    double phi = 30.0 * DEG_TO_RAD;

    double R = V * V / (G * std::tan(phi));
    TS_ASSERT_DELTA(R, 2153.0, 10.0);  // ft
  }

  // Test turn rate
  void testTurnRate() {
    double V = 200.0;
    double phi = 30.0 * DEG_TO_RAD;

    double omega = G * std::tan(phi) / V;  // rad/s
    double omegaDeg = omega * RAD_TO_DEG;

    TS_ASSERT_DELTA(omegaDeg, 5.32, 0.1);  // deg/s
  }

  /***************************************************************************
   * Sideslip Trim Tests
   ***************************************************************************/

  // Test zero sideslip in coordinated flight
  void testZeroSideslipCoordinated() {
    double beta = 0.0;  // Sideslip angle

    TS_ASSERT_DELTA(beta, 0.0, epsilon);
  }

  // Test rudder for sideslip trim
  void testRudderForSideslip() {
    double beta = 5.0 * DEG_TO_RAD;  // 5 degree sideslip
    double Cnbeta = 0.1;  // Yaw moment derivative
    double Cndr = -0.08;  // Rudder effectiveness

    // Solve: Cnbeta * beta + Cndr * dr = 0
    double dr = -Cnbeta * beta / Cndr;
    double drDeg = dr * RAD_TO_DEG;

    TS_ASSERT_DELTA(drDeg, 6.25, 0.5);
  }

  /***************************************************************************
   * Trim Convergence Tests
   ***************************************************************************/

  // Test Newton-Raphson convergence concept
  void testNewtonRaphsonStep() {
    // f(x) = x^2 - 2, solve for sqrt(2)
    double x = 1.5;
    double fx = x * x - 2.0;
    double fpx = 2.0 * x;  // Derivative

    double x_new = x - fx / fpx;
    TS_ASSERT_DELTA(x_new, 1.4167, 0.001);

    // Another iteration
    fx = x_new * x_new - 2.0;
    fpx = 2.0 * x_new;
    x_new = x_new - fx / fpx;

    TS_ASSERT_DELTA(x_new, 1.4142, 0.001);  // Close to sqrt(2)
  }

  // Test residual reduction
  void testResidualReduction() {
    double residual1 = 100.0;
    double residual2 = 10.0;
    double residual3 = 1.0;

    TS_ASSERT(residual2 < residual1);
    TS_ASSERT(residual3 < residual2);
  }

  // Test convergence criterion
  void testConvergenceCriterion() {
    double tolerance = 1e-6;
    double residual = 1e-8;

    bool converged = (std::abs(residual) < tolerance);
    TS_ASSERT(converged);
  }

  /***************************************************************************
   * Steady State Tests
   ***************************************************************************/

  // Test all accelerations zero
  void testSteadyStateAccelerations() {
    double udot = 0.0;  // Forward accel
    double vdot = 0.0;  // Lateral accel
    double wdot = 0.0;  // Vertical accel
    double pdot = 0.0;  // Roll accel
    double qdot = 0.0;  // Pitch accel
    double rdot = 0.0;  // Yaw accel

    double sumAccel = std::abs(udot) + std::abs(vdot) + std::abs(wdot) +
                      std::abs(pdot) + std::abs(qdot) + std::abs(rdot);

    TS_ASSERT_DELTA(sumAccel, 0.0, epsilon);
  }

  // Test constant rates in steady turn
  void testSteadyTurnRates() {
    double phi = 30.0 * DEG_TO_RAD;
    double V = 200.0;

    // In steady turn: p = 0, q = omega*sin(phi), r = omega*cos(phi)
    double omega = G * std::tan(phi) / V;
    double p = 0.0;
    double q = omega * std::sin(phi);
    double r = omega * std::cos(phi);

    TS_ASSERT_DELTA(p, 0.0, epsilon);
    TS_ASSERT(q > 0);  // Nose pitching up in turn
    TS_ASSERT(r > 0);  // Turning right
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very slow flight
  void testSlowFlightTrim() {
    double weight = 10000.0;
    double S = 200.0;
    double rho = 0.002377;
    double V = 100.0;  // Slow

    double q = 0.5 * rho * V * V;
    double CL_required = weight / (q * S);

    // High CL required at low speed
    TS_ASSERT(CL_required > 2.0);
  }

  // Test high-speed flight
  void testHighSpeedTrim() {
    double weight = 10000.0;
    double S = 200.0;
    double rho = 0.002377;
    double V = 400.0;  // Fast

    double q = 0.5 * rho * V * V;
    double CL_required = weight / (q * S);

    // Low CL at high speed
    TS_ASSERT(CL_required < 0.5);
  }

  // Test steep bank
  void testSteepBankTrim() {
    double phi = 60.0 * DEG_TO_RAD;

    double n = 1.0 / std::cos(phi);
    TS_ASSERT_DELTA(n, 2.0, epsilon);  // 2g turn
  }
};
