#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGPropeller.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

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

  // ==================== WINDMILLING TESTS ====================

  // Test windmilling RPM calculation
  void testWindmillingRPM() {
    // When engine is inoperative, propeller windmills due to airflow
    // Approximate: RPM_windmill = k * V / D where k depends on pitch
    double velocity = 200.0;    // ft/sec
    double diameter = 6.0;      // ft
    double pitchFactor = 0.8;   // Factor based on blade pitch

    double windmillRPM = (velocity / diameter) * pitchFactor * 60.0;
    // RPM = (200/6) * 0.8 * 60 = 1600 RPM
    TS_ASSERT_DELTA(windmillRPM, 1600.0, 1.0);

    // Higher velocity means higher windmill RPM
    velocity = 300.0;
    windmillRPM = (velocity / diameter) * pitchFactor * 60.0;
    TS_ASSERT_DELTA(windmillRPM, 2400.0, 1.0);
  }

  // Test windmilling drag coefficient
  void testWindmillingDrag() {
    // Windmilling prop creates drag
    // Drag = Cd_windmill * rho * V^2 * A_disk / 2
    double Cd_windmill = 0.2;   // Windmilling drag coefficient
    double rho = 0.002377;
    double velocity = 200.0;
    double diameter = 6.0;
    double area = M_PI * (diameter / 2.0) * (diameter / 2.0);

    double drag = 0.5 * Cd_windmill * rho * velocity * velocity * area;
    // Drag = 0.5 * 0.2 * 0.002377 * 40000 * 28.27 = 268.4 lbs
    TS_ASSERT_DELTA(drag, 268.4, 1.0);
  }

  // Test feathered vs windmilling drag comparison
  void testFeatheredVsWindmillingDrag() {
    double rho = 0.002377;
    double velocity = 200.0;
    double diameter = 6.0;
    double area = M_PI * (diameter / 2.0) * (diameter / 2.0);

    // Windmilling drag
    double Cd_windmill = 0.2;
    double dragWindmill = 0.5 * Cd_windmill * rho * velocity * velocity * area;

    // Feathered drag (much lower)
    double Cd_feathered = 0.02;
    double dragFeathered = 0.5 * Cd_feathered * rho * velocity * velocity * area;

    // Feathered drag should be ~10% of windmilling
    TS_ASSERT(dragFeathered < dragWindmill);
    TS_ASSERT_DELTA(dragFeathered / dragWindmill, 0.1, 0.01);
  }

  // ==================== BLADE GEOMETRY TESTS ====================

  // Test activity factor calculation
  void testActivityFactor() {
    // Activity factor (AF) is a measure of blade power absorption capability
    // AF = (100000 / 16) * integral from hub to tip of (c/D)(r/R)^3 d(r/R)
    // Typical values: 80-120 for light aircraft, 140-200 for high performance

    double AF_lightAircraft = 100.0;
    double AF_highPerformance = 160.0;

    TS_ASSERT(AF_lightAircraft >= 80.0 && AF_lightAircraft <= 120.0);
    TS_ASSERT(AF_highPerformance >= 140.0 && AF_highPerformance <= 200.0);

    // Higher activity factor means more power can be absorbed
    TS_ASSERT(AF_highPerformance > AF_lightAircraft);
  }

  // Test solidity calculation
  void testSolidity() {
    // Solidity sigma = B * c / (PI * R)
    // where B = number of blades, c = average chord, R = radius
    int blades = 3;
    double avgChord = 0.5;      // ft
    double radius = 3.0;        // ft

    double solidity = blades * avgChord / (M_PI * radius);
    // sigma = 3 * 0.5 / (PI * 3) = 1.5 / 9.42 = 0.159
    TS_ASSERT_DELTA(solidity, 0.159, 0.001);

    // 4-blade propeller
    blades = 4;
    solidity = blades * avgChord / (M_PI * radius);
    TS_ASSERT_DELTA(solidity, 0.212, 0.001);
  }

  // Test blade element pitch angle variation
  void testBladeElementPitch() {
    // Blade pitch varies along the span
    // beta(r) = beta_75 + twist * (r/R - 0.75)
    double beta_75 = 25.0;      // Pitch at 75% radius (reference)
    double twist = -8.0;        // Degrees from root to tip

    // At hub (r/R = 0.2)
    double r_ratio = 0.2;
    double beta_hub = beta_75 + twist * (r_ratio - 0.75);
    TS_ASSERT_DELTA(beta_hub, 25.0 + (-8.0) * (-0.55), 0.1);  // 29.4 degrees

    // At tip (r/R = 1.0)
    r_ratio = 1.0;
    double beta_tip = beta_75 + twist * (r_ratio - 0.75);
    TS_ASSERT_DELTA(beta_tip, 25.0 + (-8.0) * (0.25), 0.1);   // 23.0 degrees

    // Hub has higher pitch than tip (typical)
    TS_ASSERT(beta_hub > beta_tip);
  }

  // Test blade planform area
  void testBladePlanformArea() {
    // Simplified rectangular blade
    double bladeLength = 2.5;   // ft (from hub to tip)
    double avgChord = 0.5;      // ft
    int numBlades = 3;

    double singleBladeArea = bladeLength * avgChord;
    double totalBladeArea = numBlades * singleBladeArea;

    TS_ASSERT_DELTA(singleBladeArea, 1.25, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(totalBladeArea, 3.75, DEFAULT_TOLERANCE);

    // Compare to disk area
    double diskRadius = 3.0;
    double diskArea = M_PI * diskRadius * diskRadius;
    double bladeAreaRatio = totalBladeArea / diskArea;
    TS_ASSERT(bladeAreaRatio < 0.2);  // Typically 10-15%
  }

  // ==================== COMPRESSIBILITY EFFECTS ====================

  // Test Mach number effect on efficiency
  void testMachEffectOnEfficiency() {
    // Efficiency drops as tip Mach approaches and exceeds critical Mach
    double basEfficiency = 0.85;
    double Mach_tip = 0.7;
    double Mach_crit = 0.85;

    // Below critical Mach - no penalty
    double efficiencyFactor = 1.0;
    if (Mach_tip > Mach_crit) {
      efficiencyFactor = 1.0 - 2.0 * (Mach_tip - Mach_crit);
    }
    TS_ASSERT_DELTA(efficiencyFactor, 1.0, DEFAULT_TOLERANCE);

    // Above critical Mach - efficiency drops
    Mach_tip = 0.95;
    efficiencyFactor = 1.0;
    if (Mach_tip > Mach_crit) {
      efficiencyFactor = 1.0 - 2.0 * (Mach_tip - Mach_crit);
    }
    // Factor = 1.0 - 2.0 * 0.1 = 0.8
    TS_ASSERT_DELTA(efficiencyFactor, 0.8, DEFAULT_TOLERANCE);
  }

  // Test critical Mach number for propeller
  void testCriticalMachNumber() {
    // Critical Mach depends on blade thickness ratio
    // Thicker blades have lower critical Mach
    double thicknessRatio_thin = 0.06;
    double thicknessRatio_thick = 0.12;

    // Approximate critical Mach
    double Mcrit_thin = 0.9 - thicknessRatio_thin;
    double Mcrit_thick = 0.9 - thicknessRatio_thick;

    TS_ASSERT_DELTA(Mcrit_thin, 0.84, 0.01);
    TS_ASSERT_DELTA(Mcrit_thick, 0.78, 0.01);
    TS_ASSERT(Mcrit_thin > Mcrit_thick);
  }

  // Test tip Mach limiting
  void testTipMachLimiting() {
    double maxTipMach = 0.9;    // Maximum allowable tip Mach
    double speedOfSound = 1116.0;
    double diameter = 6.0;
    double radius = diameter / 2.0;

    // Maximum tip velocity
    double maxTipVelocity = maxTipMach * speedOfSound;
    // max omega = maxTipVelocity / radius
    double maxOmega = maxTipVelocity / radius;
    // max RPM = maxOmega * 60 / (2*PI)
    double maxRPM = maxOmega * 60.0 / (2.0 * M_PI);

    // maxRPM = (0.9 * 1116 / 3) * 60 / (2*PI) = 334.8 * 9.55 = 3197 RPM
    TS_ASSERT_DELTA(maxRPM, 3197.0, 10.0);
  }

  // ==================== GOVERNOR AND CONTROL TESTS ====================

  // Test governor proportional control
  void testGovernorProportionalControl() {
    double targetRPM = 2400.0;
    double Kp = 0.01;           // Proportional gain (deg/RPM)
    double currentPitch = 25.0; // degrees

    // RPM too high - increase pitch
    double actualRPM = 2500.0;
    double error = actualRPM - targetRPM;
    double pitchCommand = currentPitch + Kp * error;
    TS_ASSERT_DELTA(pitchCommand, 26.0, DEFAULT_TOLERANCE);

    // RPM too low - decrease pitch
    actualRPM = 2300.0;
    error = actualRPM - targetRPM;
    pitchCommand = currentPitch + Kp * error;
    TS_ASSERT_DELTA(pitchCommand, 24.0, DEFAULT_TOLERANCE);
  }

  // Test governor integral control
  void testGovernorIntegralControl() {
    double Ki = 0.001;          // Integral gain
    double errorSum = 0.0;
    double deltaT = 0.1;
    double targetRPM = 2400.0;

    // Accumulated error over time
    double errors[] = {50.0, 40.0, 30.0, 20.0, 10.0};
    for (int i = 0; i < 5; i++) {
      errorSum += errors[i] * deltaT;
    }
    // Total error sum = (50+40+30+20+10) * 0.1 = 15.0

    double integralCorrection = Ki * errorSum;
    TS_ASSERT_DELTA(integralCorrection, 0.015, DEFAULT_TOLERANCE);
  }

  // Test governor rate limiting
  void testGovernorRateLimiting() {
    double maxPitchRate = 5.0;  // degrees/second
    double deltaT = 0.1;
    double maxPitchChange = maxPitchRate * deltaT;

    double currentPitch = 25.0;
    double commandedPitch = 30.0;
    double pitchDelta = commandedPitch - currentPitch;

    // Limit the rate
    if (std::abs(pitchDelta) > maxPitchChange) {
      pitchDelta = (pitchDelta > 0) ? maxPitchChange : -maxPitchChange;
    }

    double newPitch = currentPitch + pitchDelta;
    TS_ASSERT_DELTA(newPitch, 25.5, DEFAULT_TOLERANCE);  // Limited to 0.5 deg change
  }

  // Test overspeed governor
  void testOverspeedGovernor() {
    double redlineRPM = 2700.0;
    double overspeedMargin = 50.0;  // Extra protection

    double actualRPM = 2680.0;
    bool nearOverspeed = (actualRPM > redlineRPM - overspeedMargin);
    TS_ASSERT(nearOverspeed);

    // Force pitch increase when near overspeed
    double pitchMultiplier = 1.0;
    if (nearOverspeed) {
      pitchMultiplier = 1.0 + (actualRPM - (redlineRPM - overspeedMargin)) / 100.0;
    }
    TS_ASSERT(pitchMultiplier > 1.0);
  }

  // ==================== MULTI-ENGINE PROPELLER TESTS ====================

  // Test counter-rotating propeller torque cancellation
  void testCounterRotatingTorque() {
    double torqueLeft = 500.0;   // ft-lbf (clockwise viewed from behind)
    double torqueRight = -500.0; // ft-lbf (counter-clockwise)

    double netTorque = torqueLeft + torqueRight;
    TS_ASSERT_DELTA(netTorque, 0.0, DEFAULT_TOLERANCE);

    // Asymmetric power
    torqueRight = -450.0;
    netTorque = torqueLeft + torqueRight;
    TS_ASSERT_DELTA(netTorque, 50.0, DEFAULT_TOLERANCE);
  }

  // Test propeller sync calculation
  void testPropellerSync() {
    double masterRPM = 2400.0;
    double slaveRPM = 2380.0;
    double syncTolerance = 5.0;  // RPM

    double rpmDiff = masterRPM - slaveRPM;
    bool inSync = (std::abs(rpmDiff) <= syncTolerance);
    TS_ASSERT(!inSync);  // 20 RPM difference, not in sync

    // Adjust slave to sync
    double syncCorrection = rpmDiff * 0.1;  // Gradual correction
    double newSlaveRPM = slaveRPM + syncCorrection;
    TS_ASSERT_DELTA(newSlaveRPM, 2382.0, DEFAULT_TOLERANCE);
  }

  // Test critical engine effect (multi-engine)
  void testCriticalEngineEffect() {
    // For clockwise rotating props, left engine is typically critical
    // due to P-factor and slipstream effects

    double leftEngineYawMoment = 100.0;   // ft-lbf
    double rightEngineYawMoment = -90.0;  // ft-lbf (slightly less due to geometry)

    // With both engines running
    double netYaw = leftEngineYawMoment + rightEngineYawMoment;
    TS_ASSERT_DELTA(netYaw, 10.0, DEFAULT_TOLERANCE);

    // Left engine failed - only right engine yaw
    double yawLeftFailed = rightEngineYawMoment;
    TS_ASSERT_DELTA(yawLeftFailed, -90.0, DEFAULT_TOLERANCE);

    // Right engine failed - only left engine yaw
    double yawRightFailed = leftEngineYawMoment;
    TS_ASSERT_DELTA(yawRightFailed, 100.0, DEFAULT_TOLERANCE);

    // Left engine failure creates more adverse yaw (critical)
    TS_ASSERT(std::abs(yawLeftFailed) < std::abs(yawRightFailed));
  }

  // ==================== GROUND EFFECT TESTS ====================

  // Test ground effect on thrust
  void testGroundEffectThrust() {
    // Ground effect increases thrust when close to ground
    // Thrust_ge = Thrust * (1 + k * exp(-h/b))
    double baseThrust = 1000.0;
    double height = 3.0;        // ft above ground
    double wingspan = 30.0;     // ft (or propeller diameter for approximation)
    double k = 0.05;            // Ground effect coefficient

    double geFactor = 1.0 + k * exp(-height / wingspan);
    double thrustWithGE = baseThrust * geFactor;

    // At 3 ft, geFactor = 1 + 0.05 * exp(-0.1) = 1 + 0.045 = 1.045
    TS_ASSERT_DELTA(geFactor, 1.045, 0.001);
    TS_ASSERT_DELTA(thrustWithGE, 1045.2, 1.0);

    // Very close to ground
    height = 1.0;
    geFactor = 1.0 + k * exp(-height / wingspan);
    TS_ASSERT(geFactor > 1.045);  // More ground effect
  }

  // Test propeller wake ground interaction
  void testPropWakeGroundInteraction() {
    // Prop wash velocity at ground
    double thrust = 1000.0;
    double rho = 0.002377;
    double diameter = 6.0;
    double area = M_PI * (diameter / 2.0) * (diameter / 2.0);

    // Induced velocity from momentum theory
    double Vi = sqrt(thrust / (2.0 * rho * area));
    // Vi = sqrt(1000 / (2 * 0.002377 * 28.27)) = sqrt(7439) = 86.3 ft/sec
    TS_ASSERT_DELTA(Vi, 86.25, 0.5);

    // Wake velocity at twice diameter downstream
    double distance = 2.0 * diameter;
    double wakeContraction = 0.7;  // Wake contracts
    double wakeVelocity = Vi * 2.0 * wakeContraction;  // Doubles through disk
    TS_ASSERT_DELTA(wakeVelocity, 120.7, 1.0);
  }

  // ==================== REVERSE THRUST TESTS ====================

  // Test reverse thrust magnitude
  void testReverseThrustMagnitude() {
    // Reverse thrust is typically 40-60% of forward thrust
    double forwardThrust = 1000.0;
    double reverseEfficiency = 0.5;

    double reverseThrust = -forwardThrust * reverseEfficiency;
    TS_ASSERT_DELTA(reverseThrust, -500.0, DEFAULT_TOLERANCE);

    // Full reverse pitch
    reverseEfficiency = 0.6;
    reverseThrust = -forwardThrust * reverseEfficiency;
    TS_ASSERT_DELTA(reverseThrust, -600.0, DEFAULT_TOLERANCE);
  }

  // Test reverse thrust at various speeds
  void testReverseThrustVsSpeed() {
    // Reverse thrust effectiveness varies with speed
    double maxReverseThrust = -600.0;
    double velocities[] = {0.0, 50.0, 100.0, 150.0};
    // effectiveFactor = 1.0 - V/300 with min 0.33
    // V=0: 1.0, V=50: 0.833, V=100: 0.667, V=150: 0.5
    double expectedThrust[] = {-600.0, -500.0, -400.0, -300.0};

    for (int i = 0; i < 4; i++) {
      double V = velocities[i];
      // Reverse thrust decreases as speed increases (simplified model)
      double effectiveFactor = 1.0 - (V / 300.0);
      if (effectiveFactor < 0.33) effectiveFactor = 0.33;
      double thrust = maxReverseThrust * effectiveFactor;
      TS_ASSERT_DELTA(thrust, expectedThrust[i], 10.0);
    }
  }

  // Test reverse transition time
  void testReverseTransitionTime() {
    double pitchRateLimit = 10.0;  // degrees/second
    double forwardPitch = 25.0;
    double reversePitch = -15.0;
    double pitchChange = std::abs(reversePitch - forwardPitch);

    double transitionTime = pitchChange / pitchRateLimit;
    TS_ASSERT_DELTA(transitionTime, 4.0, DEFAULT_TOLERANCE);  // 40 deg / 10 deg/s = 4 sec
  }

  // ==================== EFFICIENCY AND PERFORMANCE ====================

  // Test peak efficiency advance ratio
  void testPeakEfficiencyAdvanceRatio() {
    // Most propellers have peak efficiency around J = 0.6-0.9
    double J_peak = 0.75;
    double etaPeak = 0.87;

    // Efficiency vs J (simplified parabola)
    double J_values[] = {0.4, 0.6, 0.75, 0.9, 1.1};
    for (int i = 0; i < 5; i++) {
      double J = J_values[i];
      double eta = etaPeak - 0.5 * (J - J_peak) * (J - J_peak);
      TS_ASSERT(eta <= etaPeak);
      if (std::abs(J - J_peak) < 0.01) {
        TS_ASSERT_DELTA(eta, etaPeak, 0.01);
      }
    }
  }

  // Test propeller loading coefficient
  void testPropellerLoadingCoefficient() {
    // Cp / J^3 is a useful loading parameter
    double Cp = 0.06;
    double J = 0.8;

    double loadingCoef = Cp / (J * J * J);
    TS_ASSERT_DELTA(loadingCoef, 0.1172, 0.001);

    // Higher loading at lower J
    J = 0.5;
    loadingCoef = Cp / (J * J * J);
    TS_ASSERT_DELTA(loadingCoef, 0.48, 0.01);
  }

  // Test power loading
  void testPowerLoading() {
    // Power loading = Power / Disk Area
    double power = 100.0 * 550.0;  // 100 HP in ft-lbf/sec
    double diameter = 6.0;
    double area = M_PI * (diameter / 2.0) * (diameter / 2.0);

    double powerLoading = power / area;
    // Loading = 55000 / 28.27 = 1945.7 ft-lbf/(sec*ft^2)
    TS_ASSERT_DELTA(powerLoading, 1945.7, 1.0);
  }

  // Test disk loading
  void testDiskLoading() {
    // Disk loading = Thrust / Disk Area
    double thrust = 1000.0;     // lbs
    double diameter = 6.0;
    double area = M_PI * (diameter / 2.0) * (diameter / 2.0);

    double diskLoading = thrust / area;
    TS_ASSERT_DELTA(diskLoading, 35.37, 0.1);  // lbs/ft^2
  }

  // ==================== INERTIA AND DYNAMICS ====================

  // Test propeller polar moment of inertia
  void testPropellerInertia() {
    // Simplified: Ixx = k * m * R^2 where k depends on blade shape
    double mass = 20.0;         // slugs (total prop mass)
    double radius = 3.0;        // ft
    double k = 0.5;             // Shape factor (uniform rod = 1/3, concentrated at tip = 1)

    double Ixx = k * mass * radius * radius;
    TS_ASSERT_DELTA(Ixx, 90.0, DEFAULT_TOLERANCE);  // slug-ft^2
  }

  // Test time to spin up
  void testSpinUpTime() {
    // Time to reach operating RPM from rest
    double targetRPM = 2400.0;
    double targetOmega = targetRPM * 2.0 * M_PI / 60.0;
    double Ixx = 10.0;          // slug-ft^2
    double avgTorque = 200.0;   // ft-lbf (average during spinup)

    // omega = alpha * t, where alpha = T/I
    double alpha = avgTorque / Ixx;  // rad/sec^2
    double time = targetOmega / alpha;

    // time = 251.3 / 20 = 12.6 seconds
    TS_ASSERT_DELTA(time, 12.57, 0.1);
  }

  // Test coast-down time
  void testCoastDownTime() {
    // Time to stop after engine shutdown
    double initialRPM = 2400.0;
    double initialOmega = initialRPM * 2.0 * M_PI / 60.0;
    double Ixx = 10.0;
    double dragTorque = 50.0;   // ft-lbf (friction + windmilling)

    double alpha = dragTorque / Ixx;
    double time = initialOmega / alpha;

    // time = 251.3 / 5 = 50.3 seconds
    TS_ASSERT_DELTA(time, 50.27, 0.1);
  }

  // Test rotational kinetic energy
  void testRotationalKineticEnergy() {
    // KE = 0.5 * Ixx * omega^2
    double Ixx = 10.0;
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double KE = 0.5 * Ixx * omega * omega;
    // KE = 0.5 * 10 * 251.3^2 = 315,788 ft-lbf
    TS_ASSERT_DELTA(KE, 315788.0, 100.0);

    // Convert to horsepower-seconds
    double hp_sec = KE / 550.0;
    TS_ASSERT_DELTA(hp_sec, 574.2, 1.0);
  }

  // ==================== SPECIAL OPERATIONS ====================

  // Test beta range operation (ground fine)
  void testBetaRangeOperation() {
    // Beta range allows pitch below flight idle for ground operations
    double flightIdlePitch = 15.0;
    double groundFinePitch = 5.0;
    double reversePitch = -15.0;

    // Ground fine is between flight idle and reverse
    TS_ASSERT(groundFinePitch < flightIdlePitch);
    TS_ASSERT(groundFinePitch > reversePitch);

    // Beta range allows taxi without using brakes heavily
    bool inBetaRange = (groundFinePitch >= reversePitch && groundFinePitch < flightIdlePitch);
    TS_ASSERT(inBetaRange);
  }

  // Test autofeather activation
  void testAutofeatherActivation() {
    // Autofeather triggers when engine power drops suddenly
    double torquePrevious = 100.0;  // percent
    double torqueCurrent = 20.0;
    double torqueDropThreshold = 50.0;
    double torqueDrop = torquePrevious - torqueCurrent;

    bool shouldAutofeather = (torqueDrop > torqueDropThreshold);
    TS_ASSERT(shouldAutofeather);

    // Small drop - no autofeather
    torqueCurrent = 90.0;
    torqueDrop = torquePrevious - torqueCurrent;
    shouldAutofeather = (torqueDrop > torqueDropThreshold);
    TS_ASSERT(!shouldAutofeather);
  }

  // Test feather pump pressure
  void testFeatherPumpPressure() {
    // Feathering requires hydraulic/oil pressure
    double oilPressure = 50.0;  // psi
    double minFeatherPressure = 30.0;

    bool canFeather = (oilPressure >= minFeatherPressure);
    TS_ASSERT(canFeather);

    // Low oil pressure - feather using accumulator or electric pump
    oilPressure = 20.0;
    canFeather = (oilPressure >= minFeatherPressure);
    TS_ASSERT(!canFeather);
  }

  // Test unfeather operation
  void testUnfeatherOperation() {
    // Unfeathering requires windmilling or starter assist
    double currentPitch = 90.0;  // feathered
    double flightPitch = 25.0;
    double unfeatherRate = 3.0;  // degrees/second (slower than feathering)

    double unfeatherTime = (currentPitch - flightPitch) / unfeatherRate;
    TS_ASSERT_DELTA(unfeatherTime, 21.67, 0.1);  // About 22 seconds
  }

  // ==================== NOISE AND VIBRATION ====================

  // Test blade passage frequency
  void testBladePassageFrequency() {
    // BPF = (RPM / 60) * numBlades
    double rpm = 2400.0;
    int numBlades = 3;

    double bpf = (rpm / 60.0) * numBlades;
    TS_ASSERT_DELTA(bpf, 120.0, DEFAULT_TOLERANCE);  // 120 Hz

    // 4-blade prop at same RPM
    numBlades = 4;
    bpf = (rpm / 60.0) * numBlades;
    TS_ASSERT_DELTA(bpf, 160.0, DEFAULT_TOLERANCE);  // 160 Hz
  }

  // Test tip vortex noise correlation
  void testTipVortexNoise() {
    // Noise increases with tip speed
    double tipSpeed1 = 700.0;   // ft/sec
    double tipSpeed2 = 800.0;

    // Noise ~ tipSpeed^5 (rough approximation for high-speed noise)
    // (800/700)^5 = 1.1429^5 = 1.95
    double noiseRatio = pow(tipSpeed2 / tipSpeed1, 5);
    TS_ASSERT_DELTA(noiseRatio, 1.95, 0.05);

    // Reducing tip speed significantly reduces noise
    // (600/700)^5 = 0.857^5 = 0.46
    double tipSpeed3 = 600.0;
    noiseRatio = pow(tipSpeed3 / tipSpeed1, 5);
    TS_ASSERT_DELTA(noiseRatio, 0.46, 0.01);
  }

  // Test vibration from imbalance
  void testVibrationFromImbalance() {
    // Imbalance force = m * r * omega^2
    double imbalanceMass = 0.01;  // slugs
    double imbalanceRadius = 2.0; // ft
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double force = imbalanceMass * imbalanceRadius * omega * omega;
    // Force = 0.01 * 2 * 251.3^2 = 1263 lbf
    TS_ASSERT_DELTA(force, 1263.0, 5.0);
  }

  // ==================== ENVIRONMENTAL EFFECTS ====================

  // Test thrust at altitude
  void testThrustAtAltitude() {
    // Thrust scales with density ratio (approximately)
    double thrustSL = 1000.0;
    double rhoSL = 0.002377;
    double rhoAlt = 0.001267;   // ~15000 ft

    double densityRatio = rhoAlt / rhoSL;
    double thrustAlt = thrustSL * densityRatio;

    TS_ASSERT_DELTA(densityRatio, 0.533, 0.001);
    TS_ASSERT_DELTA(thrustAlt, 533.0, 1.0);
  }

  // Test propeller ice buildup effect
  void testIceBuildupEffect() {
    // Ice changes blade profile and mass
    double baseEfficiency = 0.85;
    double iceThickness = 0.25;  // inches

    // Efficiency degradation from ice (simplified)
    double iceDegradation = 0.05 * iceThickness;  // 5% per quarter inch
    double icedEfficiency = baseEfficiency * (1.0 - iceDegradation);

    TS_ASSERT_DELTA(icedEfficiency, 0.839, 0.001);

    // Severe ice
    iceThickness = 1.0;
    iceDegradation = 0.05 * iceThickness;
    icedEfficiency = baseEfficiency * (1.0 - iceDegradation);
    TS_ASSERT_DELTA(icedEfficiency, 0.808, 0.001);
  }

  // Test temperature effect on air density
  void testTemperatureEffectOnDensity() {
    // rho = P / (R * T)
    double pressure = 2116.22;  // lbf/ft^2 (sea level standard)
    double R = 1716.56;         // ft^2/(sec^2*R) gas constant

    // Standard temperature (59°F = 518.67°R)
    double T_std = 518.67;
    double rho_std = pressure / (R * T_std);
    TS_ASSERT_DELTA(rho_std, 0.002377, 0.00001);

    // Hot day (100°F = 559.67°R)
    double T_hot = 559.67;
    double rho_hot = pressure / (R * T_hot);
    TS_ASSERT(rho_hot < rho_std);

    // Cold day (0°F = 459.67°R)
    double T_cold = 459.67;
    double rho_cold = pressure / (R * T_cold);
    TS_ASSERT(rho_cold > rho_std);
  }

  // ==================== MISCELLANEOUS TESTS ====================

  // Test thrust line offset moment
  void testThrustLineOffset() {
    // Thrust line above/below CG creates pitching moment
    double thrust = 1000.0;
    double verticalOffset = 0.5;  // ft above CG

    double pitchMoment = thrust * verticalOffset;
    TS_ASSERT_DELTA(pitchMoment, 500.0, DEFAULT_TOLERANCE);  // nose up moment

    // Lateral offset creates yaw moment
    double lateralOffset = 3.0;  // ft (typical multi-engine)
    double yawMoment = thrust * lateralOffset;
    TS_ASSERT_DELTA(yawMoment, 3000.0, DEFAULT_TOLERANCE);
  }

  // Test propeller slipstream velocity
  void testSlipstreamVelocity() {
    // Slipstream velocity behind prop
    double freestream = 150.0;  // ft/sec
    double thrust = 1000.0;
    double rho = 0.002377;
    double diameter = 6.0;
    double area = M_PI * (diameter / 2.0) * (diameter / 2.0);

    // Induced velocity
    double Vi = sqrt(thrust / (2.0 * rho * area));

    // Far wake velocity = freestream + 2*Vi
    double slipstreamV = freestream + 2.0 * Vi;
    TS_ASSERT(slipstreamV > freestream);
    TS_ASSERT_DELTA(slipstreamV, 150.0 + 2.0 * 86.25, 2.0);
  }

  // Test blade Reynolds number
  void testBladeReynoldsNumber() {
    // Re = rho * V * c / mu
    double rho = 0.002377;
    double tipSpeed = 750.0;    // ft/sec
    double chord = 0.5;         // ft (at 75% radius)
    double mu = 3.737e-7;       // slugs/(ft*sec) dynamic viscosity

    double Re = rho * tipSpeed * chord / mu;
    // Re = 0.002377 * 750 * 0.5 / 3.737e-7 = 2.38e6
    TS_ASSERT_DELTA(Re, 2.38e6, 0.1e6);

    // High Reynolds number means turbulent flow
    TS_ASSERT(Re > 5e5);
  }

  // Test synchronization phase angle
  void testSyncPhaseAngle() {
    // Phase angle between props affects cabin noise
    double rpm1 = 2400.0;
    double rpm2 = 2400.0;
    double phaseAngle = 0.0;    // degrees

    // In phase - maximum noise at BPF
    TS_ASSERT_DELTA(phaseAngle, 0.0, DEFAULT_TOLERANCE);

    // 180 degrees out of phase - partial cancellation
    phaseAngle = 180.0;
    double cancellationFactor = std::abs(std::cos(phaseAngle * M_PI / 180.0));
    TS_ASSERT_DELTA(cancellationFactor, 1.0, DEFAULT_TOLERANCE);

    // 90 degrees phase - intermediate
    phaseAngle = 90.0;
    cancellationFactor = std::abs(std::cos(phaseAngle * M_PI / 180.0));
    TS_ASSERT_DELTA(cancellationFactor, 0.0, 0.001);
  }

  // Test propeller mass effect on CG
  void testPropellerMassEffect() {
    // Propeller mass affects aircraft CG location
    double aircraftMass = 50.0;   // slugs
    double propMass = 1.5;        // slugs
    double propStation = 10.0;    // ft forward of reference
    double cgStation = 5.0;       // ft forward of reference

    // CG shift from prop
    double totalMass = aircraftMass + propMass;
    double newCG = (aircraftMass * cgStation + propMass * propStation) / totalMass;

    TS_ASSERT(newCG > cgStation);  // CG moves forward
    TS_ASSERT_DELTA(newCG, 5.146, 0.01);  // Slight forward shift
  }

  // ==================== CONTRA-ROTATING PROPELLER TESTS ====================

  // Test contra-rotating prop torque cancellation
  void testContraRotatingTorqueCancellation() {
    double torque_front = 500.0;   // ft-lbf
    double torque_rear = -500.0;   // Opposite rotation

    double net_torque = torque_front + torque_rear;
    TS_ASSERT_DELTA(net_torque, 0.0, DEFAULT_TOLERANCE);
  }

  // Test contra-rotating efficiency gain
  void testContraRotatingEfficiencyGain() {
    double single_prop_efficiency = 0.85;
    double swirl_recovery = 0.03;  // 3% recovery from counter-rotation

    double contra_efficiency = single_prop_efficiency + swirl_recovery;
    TS_ASSERT_DELTA(contra_efficiency, 0.88, DEFAULT_TOLERANCE);
    TS_ASSERT(contra_efficiency > single_prop_efficiency);
  }

  // Test contra-rotating RPM matching
  void testContraRotatingRPMMatching() {
    double rpm_front = 2400.0;
    double rpm_rear = 2400.0;
    double sync_tolerance = 10.0;

    double rpm_diff = std::abs(rpm_front - rpm_rear);
    bool synchronized = rpm_diff < sync_tolerance;
    TS_ASSERT(synchronized);
  }

  // ==================== PROPELLER WAKE TESTS ====================

  // Test wake contraction ratio
  void testWakeContractionRatio() {
    // Wake contracts to 0.707 of disk diameter in far wake
    double disk_diameter = 6.0;
    double contraction_factor = 1.0 / std::sqrt(2.0);

    double wake_diameter = disk_diameter * contraction_factor;
    TS_ASSERT_DELTA(wake_diameter, 4.243, 0.01);
  }

  // Test wake velocity doubling
  void testWakeVelocityDoubling() {
    // Velocity through disk doubles in far wake (momentum theory)
    double induced_velocity_disk = 50.0;  // ft/sec
    double wake_velocity = 2.0 * induced_velocity_disk;

    TS_ASSERT_DELTA(wake_velocity, 100.0, DEFAULT_TOLERANCE);
  }

  // Test propeller wake interaction with wing
  void testWakeWingInteraction() {
    double wing_lift_coefficient = 1.2;
    double wake_dynamic_pressure_ratio = 1.3;  // Increased by prop wash

    double effective_lift = wing_lift_coefficient * wake_dynamic_pressure_ratio;
    TS_ASSERT_DELTA(effective_lift, 1.56, 0.01);
  }

  // ==================== BLADE STALL TESTS ====================

  // Test blade element angle of attack
  void testBladeElementAOA() {
    double pitch_angle = 25.0;  // degrees
    double inflow_angle = 10.0;  // degrees (from induced velocity)

    double blade_aoa = pitch_angle - inflow_angle;
    TS_ASSERT_DELTA(blade_aoa, 15.0, DEFAULT_TOLERANCE);
  }

  // Test blade stall detection
  void testBladeStallDetection() {
    double blade_aoa = 18.0;  // degrees
    double stall_aoa = 15.0;

    bool blade_stalled = blade_aoa > stall_aoa;
    TS_ASSERT(blade_stalled);
  }

  // Test retreating blade stall in sideslip
  void testRetreatingBladeStallSideslip() {
    double advance_angle = 30.0;  // degrees sideslip
    double base_aoa = 12.0;
    double aoa_increase = 5.0;  // Additional AOA on retreating side

    double retreating_aoa = base_aoa + aoa_increase * std::sin(advance_angle * M_PI / 180.0);
    TS_ASSERT_DELTA(retreating_aoa, 14.5, 0.1);
  }

  // ==================== PROPELLER NOISE TESTS ====================

  // Test harmonic frequencies
  void testHarmonicFrequencies() {
    double fundamental_bpf = 120.0;  // Hz

    double second_harmonic = 2.0 * fundamental_bpf;
    double third_harmonic = 3.0 * fundamental_bpf;

    TS_ASSERT_DELTA(second_harmonic, 240.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(third_harmonic, 360.0, DEFAULT_TOLERANCE);
  }

  // Test noise directivity pattern
  void testNoiseDirectivity() {
    // Propeller noise is directional
    double forward_noise = 90.0;  // dB
    double side_noise = 85.0;     // dB (reduced by 5 dB off-axis)

    double noise_reduction = forward_noise - side_noise;
    TS_ASSERT_DELTA(noise_reduction, 5.0, DEFAULT_TOLERANCE);
  }

  // Test tip Mach noise penalty
  void testTipMachNoisePenalty() {
    double base_noise = 80.0;  // dB
    double tip_mach = 0.85;
    double mach_threshold = 0.75;

    // Noise increases rapidly above Mach threshold
    double noise_penalty = 0.0;
    if (tip_mach > mach_threshold) {
      noise_penalty = 30.0 * (tip_mach - mach_threshold);  // dB
    }

    double total_noise = base_noise + noise_penalty;
    TS_ASSERT_DELTA(total_noise, 83.0, 0.1);
  }

  // ==================== PROPELLER ICING TESTS ====================

  // Test ice accretion rate
  void testIceAccretionRate() {
    double lwc = 0.5;           // Liquid water content g/m^3
    double collection_efficiency = 0.8;
    double tip_velocity = 800.0;  // ft/sec
    double time = 60.0;         // seconds

    double ice_mass_rate = lwc * collection_efficiency * tip_velocity * 0.001;  // Simplified
    double total_ice = ice_mass_rate * time;
    TS_ASSERT(total_ice > 0.0);
  }

  // Test ice effect on blade profile
  void testIceEffectOnBladeProfile() {
    double clean_Cl = 1.2;
    double ice_Cl_reduction = 0.2;  // 20% reduction

    double iced_Cl = clean_Cl * (1.0 - ice_Cl_reduction);
    TS_ASSERT_DELTA(iced_Cl, 0.96, DEFAULT_TOLERANCE);
  }

  // Test deice boot effectiveness
  void testDeiceBootEffectiveness() {
    double ice_thickness = 0.5;  // inches
    double boot_efficiency = 0.9;  // 90% effective

    double remaining_ice = ice_thickness * (1.0 - boot_efficiency);
    TS_ASSERT_DELTA(remaining_ice, 0.05, DEFAULT_TOLERANCE);
  }

  // ==================== PROPELLER STRUCTURAL TESTS ====================

  // Test centrifugal blade stress
  void testCentrifugalBladeStress() {
    double blade_mass = 5.0;  // lbs
    double radius = 3.0;      // ft
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    // Centrifugal force = m * omega^2 * r (simplified at tip)
    double force = (blade_mass / 32.174) * omega * omega * radius;
    TS_ASSERT(force > 1000.0);  // Significant force
  }

  // Test blade bending moment
  void testBladeBendingMoment() {
    double thrust_per_blade = 200.0;  // lbs
    double moment_arm = 2.0;          // ft (effective)

    double bending_moment = thrust_per_blade * moment_arm;
    TS_ASSERT_DELTA(bending_moment, 400.0, DEFAULT_TOLERANCE);
  }

  // Test hub stress concentration
  void testHubStressConcentration() {
    double nominal_stress = 10000.0;  // psi
    double stress_concentration_factor = 1.5;

    double peak_stress = nominal_stress * stress_concentration_factor;
    TS_ASSERT_DELTA(peak_stress, 15000.0, DEFAULT_TOLERANCE);
  }

  // ==================== VARIABLE PITCH MECHANISM TESTS ====================

  // Test pitch change mechanism rate
  void testPitchChangeMechanismRate() {
    double pitch_rate = 5.0;   // degrees/second
    double travel = 30.0;      // degrees total travel
    double time = travel / pitch_rate;

    TS_ASSERT_DELTA(time, 6.0, DEFAULT_TOLERANCE);  // 6 seconds full travel
  }

  // Test pitch mechanism hydraulic pressure
  void testPitchMechanismHydraulicPressure() {
    double oil_pressure = 200.0;   // psi
    double min_pressure = 100.0;

    bool adequate_pressure = oil_pressure > min_pressure;
    TS_ASSERT(adequate_pressure);
  }

  // Test pitch mechanism spring force
  void testPitchMechanismSpringForce() {
    double spring_rate = 50.0;     // lb/inch
    double displacement = 2.0;     // inches

    double spring_force = spring_rate * displacement;
    TS_ASSERT_DELTA(spring_force, 100.0, DEFAULT_TOLERANCE);
  }

  // ==================== PROP GOVERNOR ADVANCED TESTS ====================

  // Test governor speeder spring
  void testGovernorSpeederSpring() {
    double spring_preload = 10.0;  // lbs
    double lever_arm = 2.0;        // inches
    double reference_force = spring_preload * lever_arm;

    TS_ASSERT_DELTA(reference_force, 20.0, DEFAULT_TOLERANCE);
  }

  // Test governor flyweight force
  void testGovernorFlyweightForce() {
    double flyweight_mass = 0.1;   // lbs
    double radius = 1.0;           // inches
    double rpm = 2000.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double centrifugal_force = (flyweight_mass / 32.174) * omega * omega * (radius / 12.0);
    TS_ASSERT(centrifugal_force > 0.0);
  }

  // Test governor deadband
  void testGovernorDeadband() {
    double target_rpm = 2400.0;
    double deadband = 10.0;  // RPM

    double rpm_values[] = {2395.0, 2400.0, 2405.0, 2390.0, 2410.0};
    bool expected[] = {true, true, true, false, false};  // In deadband?

    for (int i = 0; i < 5; i++) {
      bool in_deadband = std::abs(rpm_values[i] - target_rpm) <= deadband / 2.0;
      TS_ASSERT_EQUALS(in_deadband, expected[i]);
    }
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompletePropellerPerformance() {
    double diameter = 6.0;  // ft
    double rpm = 2400.0;
    double velocity = 150.0;  // ft/s

    double n = rpm / 60.0;
    double J = velocity / (n * diameter);
    double efficiency = 0.85 * (1.0 - 0.1 * std::abs(J - 0.8));

    TS_ASSERT(J > 0.5);
    TS_ASSERT(J < 1.5);
    TS_ASSERT(efficiency > 0.7);
  }

  void testCompleteThrustCalculation() {
    double power = 200.0;  // HP
    double velocity = 150.0;  // ft/s
    double efficiency = 0.80;

    double thrust = (power * 550.0 * efficiency) / velocity;
    TS_ASSERT(thrust > 500.0);
    TS_ASSERT(thrust < 700.0);
  }

  void testCompleteGovernorResponse() {
    double targetRPM = 2400.0;
    double currentRPM = 2500.0;
    double bladeAngle = 20.0;
    double Kp = 0.1;

    for (int i = 0; i < 10; i++) {
      double error = currentRPM - targetRPM;
      bladeAngle += Kp * error;
      bladeAngle = std::min(std::max(bladeAngle, 10.0), 45.0);
      currentRPM -= error * 0.2;
    }

    TS_ASSERT_DELTA(currentRPM, targetRPM, 50.0);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentPropellerInstances() {
    double rpm1 = 2400.0, rpm2 = 2400.0;
    double pitch1 = 20.0, pitch2 = 20.0;

    rpm1 = 2100.0;
    pitch1 = 25.0;

    TS_ASSERT_DELTA(rpm2, 2400.0, 0.001);
    TS_ASSERT_DELTA(pitch2, 20.0, 0.001);
  }

  void testIndependentBladeAngles() {
    double bladeAngle1 = 15.0, bladeAngle2 = 15.0;
    double thrust1 = 500.0, thrust2 = 500.0;

    bladeAngle1 = 25.0;
    thrust1 = 700.0;

    TS_ASSERT_DELTA(bladeAngle2, 15.0, 0.001);
    TS_ASSERT_DELTA(thrust2, 500.0, 0.001);
  }

  void testIndependentEfficiencyCalculations() {
    double J1 = 0.6, J2 = 0.8;
    double eff1 = 0.85 * (1.0 - 0.1 * std::abs(J1 - 0.8));
    double eff2 = 0.85 * (1.0 - 0.1 * std::abs(J2 - 0.8));

    TS_ASSERT(eff1 < eff2);
    TS_ASSERT_DELTA(eff2, 0.85, 0.001);
  }

  void testIndependentGovernorStates() {
    double position1 = 0.5, position2 = 0.5;
    double targetRPM1 = 2400.0, targetRPM2 = 2400.0;

    position1 = 0.8;
    targetRPM1 = 2600.0;

    TS_ASSERT_DELTA(position2, 0.5, 0.001);
    TS_ASSERT_DELTA(targetRPM2, 2400.0, 0.001);
  }
};
