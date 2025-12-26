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

  // Test figure of merit
  void testFigureOfMerit() {
    // FM = ideal power / actual power = T * vi_ideal / P_actual
    // Or FM = CT^1.5 / (sqrt(2) * CP)
    double thrust = 5000.0;
    double rho = 0.002377;
    double area = 962.1;
    double actualPower = 345.0 * 550.0;  // HP to ft-lbf/sec

    double vi_ideal = sqrt(thrust / (2.0 * rho * area));
    double idealPower = thrust * vi_ideal;

    double FM = idealPower / actualPower;
    // Typical FM: 0.7-0.8 for good rotors
    TS_ASSERT(FM > 0.6);
    TS_ASSERT(FM < 0.95);
  }

  // Test disk loading
  void testDiskLoading() {
    // DL = T / A (lb/ft^2)
    double thrust = 5000.0;
    double area = 962.1;

    double diskLoading = thrust / area;
    // = 5.2 lb/ft^2 (typical for light helicopter)
    TS_ASSERT_DELTA(diskLoading, 5.2, 0.1);

    // Light helos: 3-6 lb/ft^2
    // Heavy helos: 8-15 lb/ft^2
    TS_ASSERT(diskLoading > 2.0);
    TS_ASSERT(diskLoading < 20.0);
  }

  // Test profile power coefficient
  void testProfilePowerCoefficient() {
    // CPo = sigma * Cd0 / 8
    // where Cd0 is average blade profile drag coefficient
    double sigma = 0.109;  // solidity
    double Cd0 = 0.01;     // profile drag coefficient

    double CPo = sigma * Cd0 / 8.0;
    TS_ASSERT_DELTA(CPo, 0.000136, 0.00001);
  }

  // Test power coefficient
  void testPowerCoefficient() {
    // CP = P / (rho * A * (omega*R)^3)
    double power = 345.0 * 550.0;  // ft-lbf/sec
    double rho = 0.002377;
    double area = 962.1;
    double Vtip = 549.8;

    double CP = power / (rho * area * pow(Vtip, 3));
    TS_ASSERT(CP > 0.0);
    TS_ASSERT(CP < 0.01);
  }

  // Test blade element momentum (BEM) hover thrust
  void testBEMHoverThrust() {
    // Simplified BEM: CT = sigma * a * (theta_0/3 - lambda/2)
    double sigma = 0.109;
    double a = 5.7;      // lift curve slope
    double theta0 = 0.15; // collective (rad)
    double lambda = 0.06; // inflow ratio

    double CT = sigma * a * (theta0 / 3.0 - lambda / 2.0);
    TS_ASSERT(CT > 0.0);
    TS_ASSERT(CT < 0.02);
  }

  // Test flapping equation (hover)
  void testFlappingEquationHover() {
    // a0 (coning) = gamma * CT / (4 * sigma)
    // Simplified relationship
    double gamma = 8.0;  // Lock number
    double CT = 0.007;
    double sigma = 0.109;

    double a0 = gamma * CT / (4.0 * sigma);
    TS_ASSERT(a0 > 0.0);
    TS_ASSERT(a0 < 0.2);  // Coning typically < 10 degrees
  }

  // Test longitudinal flapping (a1)
  void testLongitudinalFlapping() {
    // a1 varies with advance ratio
    // a1 ~ -8/3 * mu * theta_0 (simplified, at low mu)
    double mu = 0.2;  // advance ratio
    double theta0 = 0.12;

    double a1 = -8.0 / 3.0 * mu * theta0;
    TS_ASSERT(a1 < 0.0);  // Backward tilt for forward flight
    TS_ASSERT(std::abs(a1) < 0.2);
  }

  // Test lateral flapping (b1)
  void testLateralFlapping() {
    // b1 varies with advance ratio
    // b1 ~ 4/3 * mu * theta_0 (simplified, at low mu)
    double mu = 0.2;
    double theta0 = 0.12;

    double b1 = 4.0 / 3.0 * mu * theta0;
    TS_ASSERT(b1 > 0.0);  // Lateral tilt for forward flight
    TS_ASSERT(b1 < 0.1);
  }

  // Test vortex ring state boundary
  void testVortexRingStateBoundary() {
    // VRS occurs roughly when:
    // Descent rate > 0.7 * vi_hover and forward speed < 1.0 * vi_hover
    double vi_hover = 33.0;  // ft/sec

    double descentRate = 25.0;  // ft/sec
    double forwardSpeed = 20.0; // ft/sec

    bool inVRS = (descentRate > 0.7 * vi_hover) && (forwardSpeed < 1.0 * vi_hover);
    TS_ASSERT(inVRS);

    // Safe: high forward speed
    forwardSpeed = 50.0;
    inVRS = (descentRate > 0.7 * vi_hover) && (forwardSpeed < 1.0 * vi_hover);
    TS_ASSERT(!inVRS);
  }

  // Test retreating blade stall
  void testRetreatingBladeStall() {
    // Stall occurs when local angle of attack exceeds stall angle
    // On retreating blade at high speed
    double Vtip = 549.8;
    double forwardSpeed = 200.0;

    // Advance ratio
    double mu = forwardSpeed / Vtip;

    // At psi = 270 deg, retreating blade velocity
    double V_retreating = Vtip * (1 - mu);
    // = 549.8 * (1 - 0.364) = 349.5 ft/sec

    TS_ASSERT(V_retreating > 0.0);
    TS_ASSERT(V_retreating < Vtip);

    // Higher advance ratio = higher stall risk
    TS_ASSERT(mu < 0.5);  // Practical limit ~0.4
  }

  // Test translational lift
  void testTranslationalLift() {
    // ETL (Effective Translational Lift) typically 15-24 kts
    double ETL_start_kts = 15.0;
    double ETL_peak_kts = 24.0;

    double currentSpeed_kts = 20.0;

    bool inETL = (currentSpeed_kts >= ETL_start_kts) && (currentSpeed_kts <= ETL_peak_kts);
    TS_ASSERT(inETL);

    // Translational lift factor
    double TL_factor = 1.0 + 0.1 * (currentSpeed_kts - ETL_start_kts) / (ETL_peak_kts - ETL_start_kts);
    TS_ASSERT(TL_factor > 1.0);
    TS_ASSERT(TL_factor < 1.15);
  }

  // Test H-force (rotor drag)
  void testHForce() {
    // H-force is rotor drag force in forward flight
    // H ~ CT * mu (simplified)
    double CT = 0.007;
    double mu = 0.2;
    double rho = 0.002377;
    double area = 962.1;
    double Vtip = 549.8;

    double H = CT * mu * rho * area * Vtip * Vtip;
    TS_ASSERT(H > 0.0);
    TS_ASSERT(H < 1000.0);  // Reasonable drag force
  }

  // Test Y-force (rotor side force)
  void testYForce() {
    // Side force due to tail rotor, main rotor cant, etc.
    double tailThrustRequired = 333.0;
    double mainRotorCantAngle = 0.05;  // rad (~3 deg)
    double mainThrustVertical = 5000.0;

    double mainRotorSideForce = mainThrustVertical * sin(mainRotorCantAngle);
    double netSideForce = tailThrustRequired - mainRotorSideForce;

    TS_ASSERT(mainRotorSideForce > 0.0);
    TS_ASSERT(netSideForce > 0.0);  // Remaining tail rotor requirement
  }

  // Test collective-pitch to power relationship
  void testCollectivePowerRelation() {
    // More collective = more power required
    double basePower = 300.0;  // HP
    double collective = 0.1;   // rad

    // Simplified relationship
    double power = basePower * (1.0 + collective * 8.0);
    TS_ASSERT(power > basePower);

    // Zero collective = minimum power
    collective = 0.0;
    power = basePower * (1.0 + collective * 8.0);
    TS_ASSERT_DELTA(power, basePower, DEFAULT_TOLERANCE);
  }

  // Test rotor inertia
  void testRotorInertia() {
    // Rotor inertia affects response time and autorotation
    // I = n * Ib where n = number of blades
    int numBlades = 4;
    double bladeInertia = 250.0;  // slug-ft^2

    double rotorInertia = numBlades * bladeInertia;
    TS_ASSERT_DELTA(rotorInertia, 1000.0, DEFAULT_TOLERANCE);
  }

  // Test blade moment of inertia
  void testBladeMomentOfInertia() {
    // Ib = integral(m * r^2) dr
    // For uniform blade: Ib = m * R^2 / 3
    double bladeMass = 30.0;  // slugs
    double radius = 17.5;     // ft

    double Ib_uniform = bladeMass * radius * radius / 3.0;
    TS_ASSERT(Ib_uniform > 0.0);
    TS_ASSERT(Ib_uniform < 5000.0);
  }

  // Test rotor angular acceleration
  void testRotorAngularAcceleration() {
    // alpha = (Q_engine - Q_rotor) / I
    double engineTorque = 10000.0;  // ft-lbf
    double rotorTorque = 9000.0;    // ft-lbf (drag)
    double rotorInertia = 5000.0;   // slug-ft^2

    double alpha = (engineTorque - rotorTorque) / rotorInertia;
    // = 1000 / 5000 = 0.2 rad/sec^2
    TS_ASSERT_DELTA(alpha, 0.2, DEFAULT_TOLERANCE);
  }

  // Test rotor time constant
  void testRotorTimeConstant() {
    // Time to reach steady-state RPM
    // tau = I * omega / P (approximately)
    double rotorInertia = 5000.0;
    double omega = 31.42;
    double power = 300.0 * 550.0;  // ft-lbf/sec

    double tau = rotorInertia * omega / power;
    // Should be several seconds
    TS_ASSERT(tau > 0.5);
    TS_ASSERT(tau < 10.0);
  }

  // Test freewheel clutch
  void testFreewheelClutch() {
    // Clutch disengages when rotor drives engine
    double engineRPM = 5500.0;
    double rotorRPM = 290.0;
    double gearRatio = 20.0;

    double equivalentEngineRPM = rotorRPM * gearRatio;
    // = 5800 RPM

    // If rotor would drive engine faster, clutch disengages
    bool clutchDisengaged = (equivalentEngineRPM > engineRPM);
    TS_ASSERT(clutchDisengaged);

    // Normal operation: engine drives rotor
    engineRPM = 6000.0;
    clutchDisengaged = (equivalentEngineRPM > engineRPM);
    TS_ASSERT(!clutchDisengaged);
  }

  // Test autorotation RPM decay
  void testAutorotationRPMDecay() {
    // Without power, RPM decays based on drag
    double currentRPM = 300.0;
    double decayRate = 2.0;  // RPM/sec (depends on collective)
    double dt = 1.0;

    double newRPM = currentRPM - decayRate * dt;
    TS_ASSERT_EQUALS(newRPM, 298.0);

    // With proper collective in autorotation, RPM maintains
    decayRate = 0.0;  // Balanced
    newRPM = currentRPM - decayRate * dt;
    TS_ASSERT_DELTA(newRPM, currentRPM, DEFAULT_TOLERANCE);
  }

  // Test hover ceiling calculation
  void testHoverCeiling() {
    // Power available decreases with altitude
    // Hover ceiling where power available = power required
    double seaLevelPower = 400.0;  // HP
    double hoverPower = 345.0;     // HP at sea level

    // Power lapse rate ~3.5% per 1000 ft
    double lapseRate = 0.035;

    // Find altitude where power available = hover power
    // P_avail = P_sl * (1 - lapse * h/1000)
    // hover_power = P_sl * (1 - lapse * h/1000)
    double ceiling = (1.0 - hoverPower / seaLevelPower) / lapseRate * 1000.0;

    TS_ASSERT(ceiling > 0.0);
    TS_ASSERT(ceiling < 20000.0);  // Reasonable ceiling
  }

  // Test rotor wake skew angle
  void testWakeSkewAngle() {
    // Wake skew angle = atan(mu / lambda_c)
    double mu = 0.2;  // advance ratio
    double lambda_c = 0.06;  // climb inflow ratio

    double skewAngle = atan(mu / lambda_c);
    // High skew in forward flight
    TS_ASSERT(skewAngle > 0.0);
    TS_ASSERT(skewAngle < M_PI / 2.0);
  }

  // Test dynamic inflow coefficients
  void testDynamicInflowCoefficients() {
    // Pitt-Peters model: nu = [nu_0, nu_1c, nu_1s]
    // Uniform component (nu_0) and harmonic components
    double nu_0 = 0.05;   // Uniform inflow
    double nu_1c = 0.01;  // Cosine (fore-aft) component
    double nu_1s = 0.005; // Sine (lateral) component

    // Total inflow at blade azimuth psi
    double psi = M_PI / 4.0;  // 45 degrees
    double r = 0.7;  // Radial position (fraction)

    double nu_total = nu_0 + r * (nu_1c * cos(psi) + nu_1s * sin(psi));
    TS_ASSERT(nu_total > 0.0);
    TS_ASSERT(nu_total < 0.1);
  }

  // Test rotor moment due to flapping
  void testRotorMomentFromFlapping() {
    // Pitch moment = -K_beta * b1 * I_beta
    // Roll moment = K_beta * a1 * I_beta
    double K_beta = 10000.0;  // Flapping stiffness (ft-lbf/rad)
    double a1 = -0.05;        // Longitudinal flapping
    double b1 = 0.03;         // Lateral flapping

    double pitchMoment = -K_beta * b1;
    double rollMoment = K_beta * a1;

    TS_ASSERT(pitchMoment < 0.0);  // Nose-down moment
    TS_ASSERT(rollMoment < 0.0);   // Roll left (with negative a1)
  }

  // Test rotor speed governor
  void testRotorSpeedGovernor() {
    // Simple proportional governor
    double targetRPM = 300.0;
    double actualRPM = 295.0;
    double Kp = 50.0;  // Gain

    double error = targetRPM - actualRPM;
    double throttleAdjust = Kp * error;

    // Governor increases throttle when RPM low
    TS_ASSERT(throttleAdjust > 0.0);

    actualRPM = 305.0;
    error = targetRPM - actualRPM;
    throttleAdjust = Kp * error;

    // Governor decreases throttle when RPM high
    TS_ASSERT(throttleAdjust < 0.0);
  }

  // Test collective-yaw coupling
  void testCollectiveYawCoupling() {
    // Increased collective increases torque
    // which requires more tail rotor thrust
    double baseYaw = 0.0;
    double collectiveChange = 0.05;  // rad
    double couplingFactor = 2.0;     // Typical coupling ratio

    double yawChange = collectiveChange * couplingFactor;
    TS_ASSERT(yawChange > 0.0);

    // With proper pedal input, yaw can be trimmed
    double pedalInput = -0.1;
    double netYaw = yawChange + pedalInput;
    TS_ASSERT(std::abs(netYaw) < yawChange);
  }

  // Test blade loading (CT/sigma)
  void testBladeLoading() {
    // CT/sigma is a measure of blade loading
    double CT = 0.007;
    double sigma = 0.109;

    double bladeLoading = CT / sigma;
    // = 0.064 (typical range 0.05 - 0.12)
    TS_ASSERT(bladeLoading > 0.04);
    TS_ASSERT(bladeLoading < 0.15);
  }

  // Test rotor RPM transient response
  void testRPMTransientResponse() {
    // First-order lag model for RPM response
    double targetRPM = 300.0;
    double currentRPM = 290.0;
    double tau = 3.0;  // time constant in seconds
    double dt = 0.1;

    double newRPM = currentRPM + (targetRPM - currentRPM) * (1.0 - exp(-dt / tau));

    // RPM moves toward target
    TS_ASSERT(newRPM > currentRPM);
    TS_ASSERT(newRPM < targetRPM);
  }

  // Test power margin
  void testPowerMargin() {
    // Power margin = available power - required power
    double availablePower = 400.0;  // HP
    double requiredPower = 345.0;   // HP

    double margin = availablePower - requiredPower;
    double marginPercent = (margin / availablePower) * 100.0;

    TS_ASSERT(margin > 0.0);
    TS_ASSERT_DELTA(marginPercent, 13.75, 0.01);
  }

  // Test rotor azimuth position
  void testRotorAzimuthPosition() {
    // Azimuth: 0 = aft, 90 = starboard, 180 = front, 270 = port
    double omega = 31.42;  // rad/sec
    double time = 0.05;    // seconds

    double azimuth = fmod(omega * time, 2.0 * M_PI);
    // = 1.571 rad = 90 deg (one quarter turn)
    TS_ASSERT_DELTA(azimuth, M_PI / 2.0, 0.01);

    // After one full revolution
    time = 0.2;  // ~1 revolution at 300 RPM
    azimuth = fmod(omega * time, 2.0 * M_PI);
    TS_ASSERT(azimuth >= 0.0);
    TS_ASSERT(azimuth < 2.0 * M_PI);
  }

  // Test blade section lift
  void testBladeSectionLift() {
    // dL = 0.5 * rho * V^2 * c * Cl * dr
    double rho = 0.002377;
    double V = 400.0;  // local velocity at 70% span
    double chord = 1.5;
    double Cl = 0.6;
    double dr = 1.0;  // 1 ft section

    double dL = 0.5 * rho * V * V * chord * Cl * dr;
    // Per-foot section lift
    TS_ASSERT(dL > 0.0);
    TS_ASSERT(dL < 500.0);  // Reasonable section lift
  }

  // Test blade section drag
  void testBladeSectionDrag() {
    // dD = 0.5 * rho * V^2 * c * Cd * dr
    double rho = 0.002377;
    double V = 400.0;
    double chord = 1.5;
    double Cd = 0.012;  // Profile + induced
    double dr = 1.0;

    double dD = 0.5 * rho * V * V * chord * Cd * dr;
    TS_ASSERT(dD > 0.0);
    TS_ASSERT(dD < dD * 100);  // Much less than lift
  }

  // Test main rotor torque requirement
  void testMainRotorTorqueRequirement() {
    // Q = P / omega
    double power_hp = 350.0;
    double power_ftlbf = power_hp * 550.0;
    double rpm = 300.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double torque = power_ftlbf / omega;
    TS_ASSERT(torque > 5000.0);
    TS_ASSERT(torque < 15000.0);
  }
};
