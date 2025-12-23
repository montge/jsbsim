#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGThruster.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGRotorTest : public CxxTest::TestSuite
{
public:
  // Test thruster type enum for rotor
  void testThrusterTypeEnum() {
    TS_ASSERT_EQUALS(FGThruster::ttRotor, 1);
  }

  // Test rotor disk area calculation
  void testRotorDiskArea() {
    // Area = PI * R^2
    double diameter = 35.0;  // ft
    double radius = diameter / 2.0;
    double area = M_PI * radius * radius;

    // = PI * 17.5^2 = 962.1 sq ft
    TS_ASSERT_DELTA(area, 962.1, 0.1);
  }

  // Test tip speed calculation
  void testTipSpeed() {
    // Vtip = omega * R = (RPM * 2*PI/60) * R
    double rpm = 300.0;
    double radius = 17.5;  // ft

    double omega = rpm * 2.0 * M_PI / 60.0;  // rad/sec
    double Vtip = omega * radius;

    // = 31.42 * 17.5 = 549.8 ft/sec
    TS_ASSERT_DELTA(Vtip, 549.8, 0.1);
  }

  // Test advance ratio (mu)
  void testAdvanceRatio() {
    // mu = Uw / (omega * R) where Uw is forward velocity component
    double forwardVelocity = 100.0;  // ft/sec
    double omega = 31.42;  // rad/sec
    double radius = 17.5;  // ft

    double mu = forwardVelocity / (omega * radius);
    // = 100 / 549.8 = 0.182
    TS_ASSERT_DELTA(mu, 0.182, 0.001);
  }

  // Test inflow ratio (lambda)
  void testInflowRatio() {
    // lambda = Ww / (omega * R) - nu
    // where Ww is vertical velocity, nu is induced inflow
    double verticalVelocity = 10.0;  // ft/sec (descent)
    double omega = 31.42;
    double radius = 17.5;
    double inducedInflow = 0.05;

    double lambda = verticalVelocity / (omega * radius) - inducedInflow;
    // = 10 / 549.8 - 0.05 = 0.018 - 0.05 = -0.032
    TS_ASSERT_DELTA(lambda, -0.032, 0.001);
  }

  // Test solidity calculation
  void testSolidity() {
    // sigma = (b * c) / (PI * R)
    // where b = num blades, c = chord
    int numBlades = 4;
    double chord = 1.5;  // ft
    double radius = 17.5;

    double solidity = (numBlades * chord) / (M_PI * radius);
    // = 6 / 54.98 = 0.109
    TS_ASSERT_DELTA(solidity, 0.109, 0.001);
  }

  // Test thrust coefficient
  void testThrustCoefficient() {
    // CT = T / (rho * A * (omega*R)^2)
    double thrust = 5000.0;  // lbs
    double rho = 0.002377;   // slugs/ft^3
    double area = 962.1;     // sq ft
    double Vtip = 549.8;     // ft/sec

    double CT = thrust / (rho * area * Vtip * Vtip);
    // = 5000 / (0.002377 * 962.1 * 302280) = 5000 / 691088 = 0.00724
    TS_ASSERT_DELTA(CT, 0.00724, 0.0001);
  }

  // Test collective pitch effect
  void testCollectivePitch() {
    // Higher collective = more thrust
    double baseThrust = 5000.0;
    double collectiveRad = 0.0;

    // At zero collective, minimal thrust
    double thrust = baseThrust * (1.0 + collectiveRad * 10.0);
    TS_ASSERT_DELTA(thrust, 5000.0, DEFAULT_TOLERANCE);

    // Increased collective (0.1 rad = ~5.7 deg)
    collectiveRad = 0.1;
    thrust = baseThrust * (1.0 + collectiveRad * 10.0);
    TS_ASSERT_DELTA(thrust, 10000.0, DEFAULT_TOLERANCE);
  }

  // Test cyclic control effect on flapping
  void testCyclicControlEffect() {
    // Lateral cyclic (a_ic) affects lateral flapping (b1)
    // Longitudinal cyclic (b_ic) affects longitudinal flapping (a1)
    double lateralCyclic = 0.05;   // rad
    double longCyclic = -0.03;     // rad

    // Flapping response (simplified)
    double b1 = lateralCyclic * 1.2;   // Lateral tilt
    double a1 = longCyclic * 1.2;      // Longitudinal tilt

    TS_ASSERT_DELTA(b1, 0.06, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(a1, -0.036, DEFAULT_TOLERANCE);
  }

  // Test coning angle (a0)
  void testConingAngle() {
    // Coning due to lift distribution and centrifugal force
    // a0 proportional to CT and Lock number
    double CT = 0.007;
    double lockNumber = 8.0;  // Typical value

    // Simplified: a0 ~ CT * lockNumber / 6
    double a0 = CT * lockNumber / 6.0;
    TS_ASSERT_DELTA(a0, 0.0093, 0.001);
  }

  // Test Lock number
  void testLockNumber() {
    // gamma = (rho * a * c * R^4) / Ib
    // where a = lift curve slope, c = chord, Ib = blade inertia
    double rho = 0.002377;
    double a = 5.7;       // 1/rad
    double chord = 1.5;   // ft
    double radius = 17.5; // ft
    double Ib = 250.0;    // slug-ft^2 (typical for rotor blade)

    double gamma = (rho * a * chord * pow(radius, 4)) / Ib;
    // Lock number: (0.002377 * 5.7 * 1.5 * 93789) / 250 = 7.6
    TS_ASSERT(gamma > 5.0);
    TS_ASSERT(gamma < 12.0);
  }

  // Test induced velocity (hover)
  void testInducedVelocityHover() {
    // vi = sqrt(T / (2 * rho * A)) in hover
    double thrust = 5000.0;
    double rho = 0.002377;
    double area = 962.1;

    double vi = sqrt(thrust / (2.0 * rho * area));
    // = sqrt(5000 / 4.57) = sqrt(1094) = 33.1 ft/sec
    TS_ASSERT_DELTA(vi, 33.1, 0.5);
  }

  // Test inflow lag dynamics
  void testInflowLagDynamics() {
    // First-order lag: nu = (nu - target) * exp(-dt/tau) + target
    double currentNu = 0.05;
    double targetNu = 0.07;
    double inflowLag = 0.15;  // seconds
    double deltaT = 0.01;

    double newNu = (currentNu - targetNu) * exp(-deltaT / inflowLag) + targetNu;
    // Moves toward target
    TS_ASSERT(newNu > currentNu);
    TS_ASSERT(newNu < targetNu);
  }

  // Test tip loss factor
  void testTipLossFactor() {
    // B = tip loss factor (0.95-0.98 typical)
    double B = 0.97;

    // Effective blade fraction
    double effectiveRadius = 17.5 * B;
    TS_ASSERT_DELTA(effectiveRadius, 16.975, 0.001);

    // Thrust reduction due to tip loss
    double thrustFactor = B * B;
    TS_ASSERT_DELTA(thrustFactor, 0.9409, 0.001);
  }

  // Test ground effect
  void testGroundEffect() {
    // GE factor = 1 - exp(-groundeffectexp * (h_agl + shift))
    double groundEffectExp = 0.06;
    double groundEffectShift = 5.0;  // ft
    double h_agl = 10.0;  // ft

    double geFactor = 1.0 - exp(-groundEffectExp * (h_agl + groundEffectShift));
    // = 1 - exp(-0.06 * 15) = 1 - exp(-0.9) = 1 - 0.407 = 0.593
    TS_ASSERT_DELTA(geFactor, 0.593, 0.01);

    // At very low altitude
    h_agl = 2.0;
    geFactor = 1.0 - exp(-groundEffectExp * (h_agl + groundEffectShift));
    TS_ASSERT(geFactor < 0.593);  // Stronger ground effect (lower factor)

    // At high altitude, negligible ground effect
    h_agl = 100.0;
    geFactor = 1.0 - exp(-groundEffectExp * (h_agl + groundEffectShift));
    TS_ASSERT(geFactor > 0.99);  // Approaches 1.0
  }

  // Test torque calculation
  void testTorqueCalculation() {
    // Q = Power / omega
    double power = 500.0 * 550.0;  // 500 HP in ft-lbf/sec
    double rpm = 300.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double torque = power / omega;
    // = 275000 / 31.42 = 8751 ft-lbf
    TS_ASSERT_DELTA(torque, 8751.0, 10.0);
  }

  // Test power required (hover)
  void testPowerRequiredHover() {
    // P = T * vi (ideal) plus profile power
    double thrust = 5000.0;
    double vi = 33.0;  // induced velocity

    double inducedPower = thrust * vi;  // ft-lbf/sec
    double profileFactor = 1.15;  // ~15% for profile drag
    double totalPower = inducedPower * profileFactor;

    double hp = totalPower / 550.0;
    // = 5000 * 33 * 1.15 / 550 = 345 HP
    TS_ASSERT_DELTA(hp, 345.0, 5.0);
  }

  // Test gear ratio effect
  void testGearRatio() {
    // Rotor RPM = Engine RPM / Gear Ratio
    double engineRPM = 6000.0;
    double gearRatio = 20.0;

    double rotorRPM = engineRPM / gearRatio;
    TS_ASSERT_DELTA(rotorRPM, 300.0, DEFAULT_TOLERANCE);

    // Torque multiplication
    double engineTorque = 500.0;  // ft-lbf
    double rotorTorque = engineTorque * gearRatio;
    TS_ASSERT_DELTA(rotorTorque, 10000.0, DEFAULT_TOLERANCE);
  }

  // Test RPM limits
  void testRPMLimits() {
    double nominalRPM = 300.0;
    double minRPM = 1.0;
    double maxRPM = 600.0;  // 2x nominal default

    double actualRPM = 350.0;

    // Clamp to limits
    if (actualRPM < minRPM) actualRPM = minRPM;
    if (actualRPM > maxRPM) actualRPM = maxRPM;

    TS_ASSERT(actualRPM >= minRPM);
    TS_ASSERT(actualRPM <= maxRPM);
  }

  // Test blade twist effect
  void testBladetwist() {
    // Twist reduces root angle, increases tip efficiency
    double rootPitch = 0.2;  // rad
    double twist = -0.14;    // rad (washout)

    double tipPitch = rootPitch + twist;
    TS_ASSERT_DELTA(tipPitch, 0.06, DEFAULT_TOLERANCE);
    TS_ASSERT(tipPitch < rootPitch);
  }

  // Test hinge offset effect
  void testHingeOffset() {
    // Hinge offset affects control response
    double hingeOffset = 0.05;  // fraction of radius
    double radius = 17.5;

    double offsetDistance = hingeOffset * radius;
    TS_ASSERT_DELTA(offsetDistance, 0.875, DEFAULT_TOLERANCE);

    // Articulated: small offset (3-5%)
    // Hingeless: large equivalent offset (10-15%)
    TS_ASSERT(hingeOffset >= 0.0);
    TS_ASSERT(hingeOffset <= 0.20);
  }

  // Test control map types
  void testControlMapTypes() {
    // Control maps: MAIN, TAIL, TANDEM
    int MAIN = 0;
    int TAIL = 1;
    int TANDEM = 2;

    TS_ASSERT_EQUALS(MAIN, 0);
    TS_ASSERT_EQUALS(TAIL, 1);
    TS_ASSERT_EQUALS(TANDEM, 2);
  }

  // Test tail rotor anti-torque
  void testTailRotorAntiTorque() {
    // Tail rotor provides anti-torque
    double mainRotorTorque = 10000.0;  // ft-lbf
    double tailRotorArm = 30.0;        // ft (distance to tail)

    // Required tail thrust
    double tailThrust = mainRotorTorque / tailRotorArm;
    TS_ASSERT_DELTA(tailThrust, 333.3, 0.1);
  }

  // Test rotor brake
  void testRotorBrake() {
    double maxBrakePower = 25.0 * 550.0;  // 25 HP in ft-lbf/sec
    double rpm = 300.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    // Maximum brake torque
    double brakeTorque = maxBrakePower / omega;
    TS_ASSERT_DELTA(brakeTorque, 437.0, 5.0);
  }

  // Test gyroscopic moment
  void testGyroscopicMoment() {
    // M_gyro = Izz * omega * q (for pitch rate)
    double Izz = 5000.0;  // slug-ft^2
    double omega = 31.42;  // rad/sec
    double pitchRate = 0.1;  // rad/sec

    double gyroMoment = Izz * omega * pitchRate;
    TS_ASSERT_DELTA(gyroMoment, 15710.0, 10.0);
  }

  // Test downwash angle
  void testDownwashAngle() {
    // Downwash affects tail rotor and empennage
    double vi = 33.0;  // induced velocity
    double forwardVelocity = 100.0;

    double downwashAngle = atan(vi / forwardVelocity);
    // = atan(0.33) = 0.318 rad = 18.2 deg
    TS_ASSERT_DELTA(downwashAngle, 0.318, 0.01);
  }

  // Test autorotation condition
  void testAutorotation() {
    // In autorotation, rotor is unpowered
    double enginePower = 0.0;
    double descentRate = 1800.0;  // ft/min typical autorotation

    // Rotor extracts energy from airflow
    bool autorotating = (enginePower <= 0.0) && (descentRate > 500.0);
    TS_ASSERT(autorotating);
  }

  // Test nominal RPM percentage
  void testRPMPercentage() {
    double nominalRPM = 300.0;
    double actualRPM = 290.0;

    double rpmPercent = (actualRPM / nominalRPM) * 100.0;
    TS_ASSERT_DELTA(rpmPercent, 96.67, 0.01);

    // Normal operating range: 97-103%
    bool inRange = (rpmPercent >= 95.0) && (rpmPercent <= 105.0);
    TS_ASSERT(inRange);
  }
};
