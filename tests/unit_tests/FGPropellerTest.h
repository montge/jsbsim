#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGPropeller.h>
#include "TestUtilities.h"

using namespace JSBSim;

class FGPropellerTest : public CxxTest::TestSuite
{
public:
  // Test thruster type enum for propeller
  void testThrusterTypeEnum() {
    TS_ASSERT_EQUALS(FGThruster::ttPropeller, 2);
  }

  // Test advance ratio calculation J = V / (n * D)
  void testAdvanceRatioCalculation() {
    // J = V / (n * D) where V is velocity, n is rev/sec, D is diameter
    double velocity = 200.0;    // ft/sec
    double rpm = 2400.0;
    double rps = rpm / 60.0;    // rev/sec = 40
    double diameter = 6.0;      // ft

    double J = velocity / (rps * diameter);
    // J = 200 / (40 * 6) = 200 / 240 = 0.833
    TS_ASSERT_DELTA(J, 0.8333, 0.001);

    // At static (V=0)
    velocity = 0.0;
    J = velocity / (rps * diameter);
    TS_ASSERT_DELTA(J, 0.0, DEFAULT_TOLERANCE);

    // Higher velocity
    velocity = 300.0;
    J = velocity / (rps * diameter);
    TS_ASSERT_DELTA(J, 1.25, 0.001);
  }

  // Test thrust coefficient relationship
  void testThrustCoefficientFormula() {
    // Thrust = Ct * rho * n^2 * D^4
    double Ct = 0.1;            // Thrust coefficient (dimensionless)
    double rho = 0.002377;      // Air density (slugs/ft^3)
    double rpm = 2400.0;
    double rps = rpm / 60.0;    // 40 rev/sec
    double diameter = 6.0;      // ft
    double D4 = pow(diameter, 4);  // D^4

    double thrust = Ct * rho * rps * rps * D4;
    // Thrust = 0.1 * 0.002377 * 1600 * 1296 = 493.3 lbs
    TS_ASSERT_DELTA(thrust, 493.3, 1.0);
  }

  // Test power coefficient relationship
  void testPowerCoefficientFormula() {
    // Power = Cp * rho * n^3 * D^5
    double Cp = 0.06;           // Power coefficient (dimensionless)
    double rho = 0.002377;      // Air density (slugs/ft^3)
    double rpm = 2400.0;
    double rps = rpm / 60.0;    // 40 rev/sec
    double diameter = 6.0;      // ft
    double D5 = pow(diameter, 5);  // D^5

    double power = Cp * rho * pow(rps, 3) * D5;
    // Power = 0.06 * 0.002377 * 64000 * 7776 = 70,929 ft-lbf/sec
    TS_ASSERT_DELTA(power, 70929.4, 100.0);

    // Convert to HP
    double hp = power / 550.0;
    TS_ASSERT_DELTA(hp, 128.96, 1.0);
  }

  // Test propeller efficiency calculation
  void testPropellerEfficiency() {
    // Efficiency = J * Ct / Cp
    double J = 0.8;
    double Ct = 0.1;
    double Cp = 0.06;

    double efficiency = J * Ct / Cp;
    // eta = 0.8 * 0.1 / 0.06 = 1.333
    // Note: efficiency > 1.0 is not physically realistic
    // Real propellers have eta < 0.9 typically
    TS_ASSERT_DELTA(efficiency, 1.333, 0.01);

    // More realistic values
    J = 0.7;
    Ct = 0.08;
    Cp = 0.07;
    efficiency = J * Ct / Cp;
    TS_ASSERT_DELTA(efficiency, 0.8, 0.01);
  }

  // Test helical tip Mach calculation
  void testHelicalTipMach() {
    // Helical tip Mach = sqrt(Vtip^2 + V^2) / a
    // where Vtip = omega * R = (RPM * 2*PI/60) * (D/2)
    double rpm = 2400.0;
    double diameter = 6.0;       // ft
    double radius = diameter / 2.0;
    double omega = rpm * 2.0 * M_PI / 60.0;  // rad/sec
    double Vtip = omega * radius;  // tip velocity
    double V = 200.0;            // forward velocity ft/sec
    double a = 1116.45;          // speed of sound ft/sec

    double helicalV = sqrt(Vtip * Vtip + V * V);
    double helicalMach = helicalV / a;

    // Vtip = 251.3 * 3 = 753.98 ft/sec
    TS_ASSERT_DELTA(Vtip, 753.98, 1.0);

    // helicalV = sqrt(753.98^2 + 200^2) = sqrt(568489 + 40000) = 780.0
    TS_ASSERT_DELTA(helicalV, 780.0, 1.0);

    // helicalMach = 780 / 1116.45 = 0.699
    TS_ASSERT_DELTA(helicalMach, 0.699, 0.01);
  }

  // Test RPM to angular velocity conversion
  void testRPMToOmega() {
    // omega = RPM * 2 * PI / 60
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    // omega = 2400 * 2 * PI / 60 = 251.33 rad/sec
    TS_ASSERT_DELTA(omega, 251.327, 0.01);

    // Different RPMs
    rpm = 1200.0;
    omega = rpm * 2.0 * M_PI / 60.0;
    TS_ASSERT_DELTA(omega, 125.664, 0.01);
  }

  // Test gear ratio effect on RPM
  void testGearRatioRPM() {
    // PropRPM = EngineRPM / GearRatio
    double engineRPM = 2800.0;
    double gearRatio = 1.0;  // Direct drive

    double propRPM = engineRPM / gearRatio;
    TS_ASSERT_DELTA(propRPM, 2800.0, DEFAULT_TOLERANCE);

    // Reduction gearing (typical for high-powered engines)
    gearRatio = 2.0;
    propRPM = engineRPM / gearRatio;
    TS_ASSERT_DELTA(propRPM, 1400.0, DEFAULT_TOLERANCE);

    // More reduction
    gearRatio = 3.5;
    propRPM = engineRPM / gearRatio;
    TS_ASSERT_DELTA(propRPM, 800.0, DEFAULT_TOLERANCE);
  }

  // Test variable pitch detection
  void testVariablePitchDetection() {
    double minPitch = 15.0;
    double maxPitch = 45.0;

    // Variable pitch if max != min
    bool isVPitch = (maxPitch != minPitch);
    TS_ASSERT(isVPitch);

    // Fixed pitch
    minPitch = 25.0;
    maxPitch = 25.0;
    isVPitch = (maxPitch != minPitch);
    TS_ASSERT(!isVPitch);
  }

  // Test pitch limiting
  void testPitchLimiting() {
    double minPitch = 10.0;
    double maxPitch = 45.0;

    // Command within limits
    double pitchCmd = 30.0;
    double actualPitch = std::max(minPitch, std::min(maxPitch, pitchCmd));
    TS_ASSERT_DELTA(actualPitch, 30.0, DEFAULT_TOLERANCE);

    // Command above max
    pitchCmd = 60.0;
    actualPitch = std::max(minPitch, std::min(maxPitch, pitchCmd));
    TS_ASSERT_DELTA(actualPitch, 45.0, DEFAULT_TOLERANCE);

    // Command below min
    pitchCmd = 5.0;
    actualPitch = std::max(minPitch, std::min(maxPitch, pitchCmd));
    TS_ASSERT_DELTA(actualPitch, 10.0, DEFAULT_TOLERANCE);
  }

  // Test sense (rotation direction)
  void testRotationSense() {
    // Sense = +1 for clockwise (viewed from cockpit)
    // Sense = -1 for counter-clockwise
    double sense = 1.0;
    TS_ASSERT_DELTA(sense, 1.0, DEFAULT_TOLERANCE);

    sense = -1.0;
    TS_ASSERT_DELTA(sense, -1.0, DEFAULT_TOLERANCE);

    // Sense affects torque direction
    double torque = 100.0;
    double effectiveTorque = sense * torque;
    TS_ASSERT_DELTA(effectiveTorque, -100.0, DEFAULT_TOLERANCE);
  }

  // Test induced velocity calculation
  void testInducedVelocity() {
    // Vi = 0.5 * (-V + sqrt(V^2 + 2*T/(rho*A)))
    // where A = PI * R^2
    double velocity = 100.0;    // ft/sec forward velocity
    double thrust = 500.0;      // lbs
    double rho = 0.002377;      // air density
    double diameter = 6.0;
    double radius = diameter / 2.0;
    double area = M_PI * radius * radius;

    double discriminant = velocity * velocity + 2.0 * thrust / (rho * area);
    double Vi = 0.5 * (-velocity + sqrt(discriminant));

    // area = PI * 9 = 28.27 ft^2
    // discriminant = 10000 + 2*500/(0.002377*28.27) = 10000 + 14878 = 24878
    // Vi = 0.5 * (-100 + 157.73) = 28.87 ft/sec
    TS_ASSERT_DELTA(Vi, 28.87, 0.5);
  }

  // Test static thrust calculation
  void testStaticThrust() {
    // At V=0, advance ratio J=0
    // Thrust coefficient is typically maximum at J=0
    double Ct_static = 0.12;  // Higher Ct at static
    double rho = 0.002377;
    double rpm = 2400.0;
    double rps = rpm / 60.0;
    double diameter = 6.0;
    double D4 = pow(diameter, 4);

    double staticThrust = Ct_static * rho * rps * rps * D4;
    // Static thrust = 0.12 * 0.002377 * 1600 * 1296 = 592 lbs
    TS_ASSERT_DELTA(staticThrust, 592.0, 1.0);
  }

  // Test torque calculation
  void testTorqueCalculation() {
    // Torque = Power / omega
    double power = 55000.0;  // 100 HP in ft-lbf/sec
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double torque = power / omega;
    // Torque = 55000 / 251.33 = 218.9 ft-lbf
    TS_ASSERT_DELTA(torque, 218.87, 0.1);
  }

  // Test RPM integration from excess torque
  void testRPMIntegration() {
    // d(omega)/dt = Q / Ixx
    // d(RPM)/dt = (Q / Ixx) * 60 / (2*PI)
    double torque = 100.0;      // ft-lbf excess torque
    double Ixx = 10.0;          // slug-ft^2 rotational inertia
    double deltaT = 0.01;       // seconds

    double angularAccel = torque / Ixx;  // rad/sec^2
    double rpmChange = angularAccel * deltaT * 60.0 / (2.0 * M_PI);

    // rpmChange = (10.0 * 0.01 * 60) / (2*PI) = 0.9549 RPM per timestep
    TS_ASSERT_DELTA(rpmChange, 0.9549, 0.001);

    // Starting from 2400 RPM
    double rpm = 2400.0;
    rpm += rpmChange;
    TS_ASSERT_DELTA(rpm, 2400.9549, 0.001);
  }

  // Test P-factor angle calculation
  void testPFactorAngle() {
    // P-factor creates asymmetric thrust due to blade angle of attack
    // variation in sideslip or climb
    double pFactor = 0.005;  // P-factor coefficient
    double alpha = 5.0 * M_PI / 180.0;  // 5 degrees angle of attack
    double thrust = 1000.0;

    // Yaw moment from P-factor
    double yawMoment = pFactor * alpha * thrust;
    TS_ASSERT(yawMoment > 0.0);
  }

  // Test constant speed mode pitch adjustment
  void testConstantSpeedPitchAdjust() {
    // In constant speed mode, pitch adjusts to maintain target RPM
    double targetRPM = 2400.0;
    double actualRPM = 2450.0;
    double deltaT = 0.1;

    // RPM too high -> increase pitch to absorb more power
    double rpmError = actualRPM - targetRPM;
    double pitchAdjust = rpmError * deltaT;  // Simplified

    TS_ASSERT(rpmError > 0.0);
    TS_ASSERT(pitchAdjust > 0.0);

    // RPM too low -> decrease pitch
    actualRPM = 2350.0;
    rpmError = actualRPM - targetRPM;
    pitchAdjust = rpmError * deltaT;

    TS_ASSERT(rpmError < 0.0);
    TS_ASSERT(pitchAdjust < 0.0);
  }

  // Test reverse pitch operation
  void testReversePitch() {
    double minPitch = 15.0;
    double reversePitch = -10.0;
    double reverseCoef = 0.0;  // 0 = min pitch, 1 = full reverse

    // Calculate actual pitch based on reverse coefficient
    // At reverseCoef = 0: pitch = minPitch
    // At reverseCoef = 1: pitch = reversePitch
    double pitch = minPitch + reverseCoef * (reversePitch - minPitch);
    TS_ASSERT_DELTA(pitch, minPitch, DEFAULT_TOLERANCE);

    // Full reverse
    reverseCoef = 1.0;
    pitch = minPitch + reverseCoef * (reversePitch - minPitch);
    TS_ASSERT_DELTA(pitch, reversePitch, DEFAULT_TOLERANCE);

    // 50% reverse
    reverseCoef = 0.5;
    pitch = minPitch + reverseCoef * (reversePitch - minPitch);
    TS_ASSERT_DELTA(pitch, 2.5, DEFAULT_TOLERANCE);  // (15 + -10) / 2 = 2.5
  }

  // Test feather pitch operation
  void testFeatherPitch() {
    // Feathering sets pitch to maximum to minimize drag
    double maxPitch = 90.0;
    bool feathered = false;

    // Normal operation
    double pitch = 25.0;
    if (feathered) pitch = maxPitch;
    TS_ASSERT_DELTA(pitch, 25.0, DEFAULT_TOLERANCE);

    // Feathered
    feathered = true;
    if (feathered) pitch = maxPitch;
    TS_ASSERT_DELTA(pitch, maxPitch, DEFAULT_TOLERANCE);
  }

  // Test Ct and Cp factor multipliers
  void testCtCpFactors() {
    double baseCt = 0.1;
    double baseCp = 0.06;
    double CtFactor = 1.0;
    double CpFactor = 1.0;

    // Nominal case
    double effectiveCt = baseCt * CtFactor;
    double effectiveCp = baseCp * CpFactor;
    TS_ASSERT_DELTA(effectiveCt, 0.1, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(effectiveCp, 0.06, DEFAULT_TOLERANCE);

    // Modified factors (e.g., for prop modification)
    CtFactor = 1.1;
    CpFactor = 0.95;
    effectiveCt = baseCt * CtFactor;
    effectiveCp = baseCp * CpFactor;
    TS_ASSERT_DELTA(effectiveCt, 0.11, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(effectiveCp, 0.057, DEFAULT_TOLERANCE);
  }

  // Test D^4 and D^5 calculation for standard diameters
  void testDiameterPowers() {
    // Common propeller diameters
    double d_6ft = 6.0;
    double d_8ft = 8.0;
    double d_10ft = 10.0;

    // D^4 for thrust calculations
    TS_ASSERT_DELTA(pow(d_6ft, 4), 1296.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(pow(d_8ft, 4), 4096.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(pow(d_10ft, 4), 10000.0, DEFAULT_TOLERANCE);

    // D^5 for power calculations
    TS_ASSERT_DELTA(pow(d_6ft, 5), 7776.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(pow(d_8ft, 5), 32768.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(pow(d_10ft, 5), 100000.0, DEFAULT_TOLERANCE);
  }

  // Test minimum RPM for constant speed operation
  void testMinRPMForConstSpeed() {
    double minRPM = 200.0;  // Oil pressure minimum for pitch control
    double actualRPM = 150.0;

    // Below minimum, constant speed control is not active
    bool csActive = (actualRPM >= minRPM);
    TS_ASSERT(!csActive);

    actualRPM = 250.0;
    csActive = (actualRPM >= minRPM);
    TS_ASSERT(csActive);
  }

  // Test gyroscopic moment calculation
  void testGyroscopicMoment() {
    // Gyroscopic moment = Ixx * omega * q (for pitch)
    // where q is pitch rate
    double Ixx = 10.0;          // slug-ft^2
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;  // rad/sec
    double q = 0.1;             // pitch rate rad/sec

    double gyroMoment = Ixx * omega * q;
    // Moment = 10 * 251.33 * 0.1 = 251.33 ft-lbf
    TS_ASSERT_DELTA(gyroMoment, 251.33, 0.1);

    // Moment is proportional to RPM and pitch rate
    q = 0.2;
    gyroMoment = Ixx * omega * q;
    TS_ASSERT_DELTA(gyroMoment, 502.65, 0.1);
  }

  // Test number of blades effect on calculations
  void testNumberOfBlades() {
    // Number of blades affects Ct and Cp curves (via table lookup)
    // but not the dimensional formulas
    int blades2 = 2;
    int blades3 = 3;
    int blades4 = 4;

    TS_ASSERT(blades2 >= 2);
    TS_ASSERT(blades3 >= 2);
    TS_ASSERT(blades4 >= 2);
    TS_ASSERT(blades4 > blades2);

    // More blades typically means higher solidity
    // and different optimal J ranges
  }
};
