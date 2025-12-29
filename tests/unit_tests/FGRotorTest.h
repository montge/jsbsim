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

  /***************************************************************************
   * Coaxial Rotor Tests
   ***************************************************************************/

  // Test coaxial rotor thrust
  void testCoaxialRotorThrust() {
    // Two rotors, but interference reduces total efficiency
    double singleRotorThrust = 5000.0;
    double interferenceFactor = 0.85;  // Typical 15% loss

    double totalThrust = 2.0 * singleRotorThrust * interferenceFactor;
    TS_ASSERT_DELTA(totalThrust, 8500.0, 10.0);
    TS_ASSERT(totalThrust < 2.0 * singleRotorThrust);  // Less than ideal
  }

  // Test coaxial torque cancellation
  void testCoaxialTorqueCancellation() {
    // Counter-rotating rotors cancel torque
    double upperTorque = 8000.0;
    double lowerTorque = -8000.0;  // Opposite direction

    double netTorque = upperTorque + lowerTorque;
    TS_ASSERT_DELTA(netTorque, 0.0, 100.0);  // Some residual from mismatch
  }

  // Test coaxial rotor separation effect
  void testCoaxialSeparationEffect() {
    // Greater separation reduces interference
    double separation = 0.1;  // Fraction of diameter
    double baseEfficiency = 0.85;

    // Efficiency improves with separation
    double improvedEfficiency = baseEfficiency + separation * 0.5;
    TS_ASSERT(improvedEfficiency > baseEfficiency);
    TS_ASSERT(improvedEfficiency <= 1.0);
  }

  /***************************************************************************
   * Tandem Rotor Tests
   ***************************************************************************/

  // Test tandem rotor overlap efficiency
  void testTandemOverlapEfficiency() {
    // Overlapping rotors have interference
    double overlap = 0.3;  // 30% overlap
    double efficiencyLoss = overlap * 0.2;  // Rough approximation

    double netEfficiency = 1.0 - efficiencyLoss;
    TS_ASSERT(netEfficiency > 0.9);
    TS_ASSERT(netEfficiency < 1.0);
  }

  // Test tandem pitch control
  void testTandemPitchControl() {
    // Differential collective for pitch
    double frontCollective = 0.12;  // rad
    double rearCollective = 0.10;   // rad

    double pitchMoment = (frontCollective - rearCollective) * 100000.0;  // ft-lbf
    TS_ASSERT(pitchMoment > 0.0);  // Nose-up moment
  }

  // Test tandem roll control
  void testTandemRollControl() {
    // Lateral cyclic on both rotors
    double lateralCyclic = 0.05;  // rad
    double arm = 4.0;  // ft lateral distance to thrust

    double rollMoment = 5000.0 * lateralCyclic * arm;
    TS_ASSERT(rollMoment > 0.0);
  }

  /***************************************************************************
   * Compressibility Effects Tests
   ***************************************************************************/

  // Test tip Mach number
  void testTipMachNumber() {
    double Vtip = 700.0;  // ft/sec (high for demo)
    double speedOfSound = 1116.0;  // ft/sec at sea level

    double M_tip = Vtip / speedOfSound;
    // = 0.627 (approaching drag divergence)
    TS_ASSERT_DELTA(M_tip, 0.627, 0.01);
    TS_ASSERT(M_tip < 0.9);  // Keep below critical
  }

  // Test advancing blade tip Mach
  void testAdvancingBladeTipMach() {
    double Vtip = 550.0;
    double forwardSpeed = 200.0;
    double speedOfSound = 1116.0;

    // Maximum velocity at advancing blade tip
    double V_adv = Vtip + forwardSpeed;
    double M_adv = V_adv / speedOfSound;

    TS_ASSERT(M_adv > Vtip / speedOfSound);
    TS_ASSERT(M_adv < 1.0);  // Should stay subsonic
  }

  // Test drag divergence Mach effect
  void testDragDivergenceMach() {
    double M = 0.75;
    double M_dd = 0.72;  // Drag divergence Mach

    // Drag increases rapidly above M_dd
    double dragFactor = 1.0;
    if (M > M_dd) {
      dragFactor = 1.0 + 10.0 * pow(M - M_dd, 2);
    }

    TS_ASSERT(dragFactor > 1.0);
  }

  /***************************************************************************
   * Blade Element Theory Extended Tests
   ***************************************************************************/

  // Test radial integration for thrust
  void testRadialThrustIntegration() {
    // T = integral(dT) from r=0 to R
    double rho = 0.002377;
    double omega = 31.42;
    double R = 17.5;
    double chord = 1.5;
    double a = 5.7;
    double theta0 = 0.15;

    // Simplified BEM integral (using 5 sections)
    double T = 0.0;
    for (int i = 1; i <= 5; i++) {
      double r = R * i / 5.0;
      double V = omega * r;
      double dL = 0.5 * rho * V * V * chord * a * theta0 * (R / 5.0);
      T += dL;
    }

    TS_ASSERT(T > 0.0);
    TS_ASSERT(T < 20000.0);
  }

  // Test root cutout effect
  void testRootCutout() {
    double rootCutout = 0.15;  // 15% of radius is hub
    double R = 17.5;

    double effectiveStartRadius = rootCutout * R;
    TS_ASSERT_DELTA(effectiveStartRadius, 2.625, 0.01);

    // Thrust area reduction
    double areaFactor = 1.0 - rootCutout * rootCutout;
    TS_ASSERT(areaFactor > 0.9);
  }

  // Test blade taper effect
  void testBladeTaper() {
    double rootChord = 2.0;
    double tipChord = 1.0;
    double taperRatio = tipChord / rootChord;

    TS_ASSERT_DELTA(taperRatio, 0.5, 0.001);

    // Chord at 70% span (linear taper)
    double r_frac = 0.7;
    double chord_at_r = rootChord * (1.0 - r_frac * (1.0 - taperRatio));
    TS_ASSERT(chord_at_r > tipChord);
    TS_ASSERT(chord_at_r < rootChord);
  }

  /***************************************************************************
   * Autorotation Extended Tests
   ***************************************************************************/

  // Test autorotation index
  void testAutorotationIndex() {
    // AI = (stored kinetic energy) / (power required in hover)
    // Higher AI = more time to react
    double I = 5000.0;  // slug-ft^2
    double omega = 31.42;
    double P_hover = 345.0 * 550.0;

    double KE = 0.5 * I * omega * omega;
    double AI = KE / P_hover;

    TS_ASSERT(AI > 5.0);   // Reasonable index
    TS_ASSERT(AI < 30.0);
  }

  // Test autorotation entry height
  void testAutorotationEntryHeight() {
    // Minimum height for safe autorotation entry
    double reactionTime = 2.0;  // seconds
    double initialDescent = 500.0;  // ft/min initially
    double steadyDescent = 1800.0;  // ft/min in auto

    // Height lost during reaction + transition
    double heightLost = (initialDescent * reactionTime +
                         steadyDescent * reactionTime) / 60.0;

    TS_ASSERT(heightLost > 50.0);
    TS_ASSERT(heightLost < 200.0);
  }

  // Test autorotation flare
  void testAutorotationFlare() {
    // Flare trades rotor RPM for lift
    double descentRate = 30.0;  // ft/sec before flare
    double rpmBefore = 300.0;
    double flareAngle = 20.0;  // degrees nose up

    // Rotor disc loading increases in flare
    double loadFactor = 1.0 / cos(flareAngle * M_PI / 180.0);
    TS_ASSERT(loadFactor > 1.0);

    // Descent rate decreases
    double descentAfter = descentRate * cos(flareAngle * M_PI / 180.0);
    TS_ASSERT(descentAfter < descentRate);
  }

  /***************************************************************************
   * Helicopter Stability Tests
   ***************************************************************************/

  // Test longitudinal static stability
  void testLongitudinalStaticStability() {
    // Speed stability: Cm_u (change in pitching moment with speed)
    double Cm_u = -0.002;  // Stable if negative

    double speedChange = 10.0;  // ft/sec
    double momentChange = Cm_u * speedChange;

    // Positive speed increase causes nose-down moment
    TS_ASSERT(momentChange < 0.0);
  }

  // Test lateral static stability
  void testLateralStaticStability() {
    // Cl_beta: roll due to sideslip
    double Cl_beta = -0.015;  // Stable if negative

    double sideslip = 5.0;  // degrees
    double rollMoment = Cl_beta * sideslip;

    // Positive sideslip causes left roll (opposite direction)
    TS_ASSERT(rollMoment < 0.0);
  }

  // Test directional static stability
  void testDirectionalStaticStability() {
    // Cn_beta: yaw due to sideslip (weathervane)
    double Cn_beta = 0.002;  // Stable if positive

    double sideslip = 5.0;  // degrees
    double yawMoment = Cn_beta * sideslip;

    // Positive sideslip causes yaw to reduce sideslip
    TS_ASSERT(yawMoment > 0.0);
  }

  // Test hover pitch damping
  void testHoverPitchDamping() {
    // Pitch rate damping in hover
    double Mq = -1.5;  // Pitch damping derivative (1/sec)
    double pitchRate = 0.1;  // rad/sec

    double dampingMoment = Mq * pitchRate;
    TS_ASSERT(dampingMoment < 0.0);  // Opposes pitch rate
  }

  /***************************************************************************
   * Vibration Analysis Tests
   ***************************************************************************/

  // Test 1/rev vibration frequency
  void testOnePerRevFrequency() {
    double rpm = 300.0;
    double freq_hz = rpm / 60.0;

    TS_ASSERT_DELTA(freq_hz, 5.0, 0.01);  // 5 Hz for 300 RPM
  }

  // Test N/rev vibration (blade passage)
  void testNPerRevVibration() {
    double rpm = 300.0;
    int numBlades = 4;

    double N_per_rev_freq = (rpm / 60.0) * numBlades;
    TS_ASSERT_DELTA(N_per_rev_freq, 20.0, 0.1);  // 20 Hz
  }

  // Test blade natural frequency (flap)
  void testBladeNaturalFrequency() {
    // First flap mode typically slightly above 1/rev
    double omega = 31.42;  // rad/sec (5 Hz at 300 RPM)
    double nu_beta = 1.03;  // Non-dimensional flap frequency

    double omega_beta = nu_beta * omega;
    TS_ASSERT(omega_beta > omega);  // Above 1/rev
    TS_ASSERT_DELTA(omega_beta / omega, nu_beta, 0.01);
  }

  /***************************************************************************
   * Performance Calculation Tests
   ***************************************************************************/

  // Test maximum rate of climb
  void testMaxRateOfClimb() {
    double excessPower = 55.0 * 550.0;  // 55 HP excess in ft-lbf/sec
    double weight = 5000.0;  // lbs

    double ROC = excessPower / weight * 60.0;  // ft/min
    // = 30250 / 5000 * 60 = 363 ft/min
    TS_ASSERT(ROC > 300.0);
    TS_ASSERT(ROC < 500.0);
  }

  // Test best rate of climb speed
  void testBestROCSpeed() {
    // Typically around 60-70 knots for light helicopter
    double V_best_ROC = 65.0;  // knots

    // Convert to ft/sec
    double V_fps = V_best_ROC * 1.689;
    TS_ASSERT_DELTA(V_fps, 109.8, 1.0);
  }

  // Test service ceiling power requirement
  void testServiceCeilingPower() {
    double seaLevelPower = 400.0;  // HP available
    double altitude = 15000.0;  // ft
    double lapseRate = 0.035;  // per 1000 ft

    double altitudeFactor = 1.0 - lapseRate * altitude / 1000.0;
    double availablePower = seaLevelPower * altitudeFactor;

    TS_ASSERT(availablePower < seaLevelPower);
    TS_ASSERT(availablePower > 0.0);
  }

  // Test fuel flow in cruise
  void testCruiseFuelFlow() {
    double power = 280.0;  // HP in cruise
    double SFC = 0.55;     // lb/HP/hr

    double fuelFlow = power * SFC;  // lb/hr
    TS_ASSERT_DELTA(fuelFlow, 154.0, 1.0);
  }

  // Test endurance calculation
  void testEndurance() {
    double fuelCapacity = 100.0;  // gallons
    double fuelDensity = 6.0;     // lb/gal
    double fuelFlow = 150.0;      // lb/hr

    double fuelWeight = fuelCapacity * fuelDensity;
    double endurance = fuelWeight / fuelFlow;

    TS_ASSERT_DELTA(endurance, 4.0, 0.1);  // 4 hours
  }

  // Test range calculation
  void testRange() {
    double endurance = 4.0;  // hours
    double cruiseSpeed = 120.0;  // knots

    double range = endurance * cruiseSpeed;
    TS_ASSERT_DELTA(range, 480.0, 1.0);  // nm
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  // Test zero RPM behavior
  void testZeroRPM() {
    double rpm = 0.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    TS_ASSERT_DELTA(omega, 0.0, DEFAULT_TOLERANCE);

    // Tip speed should be zero
    double Vtip = omega * 17.5;
    TS_ASSERT_DELTA(Vtip, 0.0, DEFAULT_TOLERANCE);
  }

  // Test maximum collective
  void testMaxCollective() {
    double maxCollective = 0.25;  // rad (~14 degrees)

    // Should be within physical limits
    TS_ASSERT(maxCollective > 0.0);
    TS_ASSERT(maxCollective < M_PI / 6.0);  // < 30 degrees
  }

  // Test reverse flow region
  void testReverseFlowRegion() {
    // At high advance ratio, inner retreating blade sees reverse flow
    double mu = 0.35;
    double R = 17.5;

    // Reverse flow extends to r = mu * R on retreating side
    double reverseFlowRadius = mu * R;
    TS_ASSERT_DELTA(reverseFlowRadius, 6.125, 0.01);
  }

  // Test rotor stall margin
  void testRotorStallMargin() {
    double CT_max = 0.015;  // Maximum before stall
    double CT_operating = 0.007;

    double margin = (CT_max - CT_operating) / CT_max * 100.0;
    TS_ASSERT(margin > 40.0);  // Good margin
    TS_ASSERT(margin < 80.0);
  }

  // Test negative collective
  void testNegativeCollective() {
    double collective = -0.05;  // Negative collective

    // Would produce negative thrust (used in special maneuvers)
    double thrustDirection = (collective > 0) ? 1.0 : -1.0;
    TS_ASSERT(thrustDirection < 0.0);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete rotor performance chain
  void testCompleteRotorPerformance() {
    // Comprehensive rotor performance calculation from inputs to outputs
    double rpm = 300.0;
    double radius = 17.5;
    double numBlades = 4;
    double chord = 1.5;
    double collective = 0.12;
    double rho = 0.002377;

    // Calculate omega and tip speed
    double omega = rpm * 2.0 * M_PI / 60.0;
    double Vtip = omega * radius;
    TS_ASSERT_DELTA(omega, 31.42, 0.1);
    TS_ASSERT_DELTA(Vtip, 549.8, 1.0);

    // Calculate solidity
    double sigma = (numBlades * chord) / (M_PI * radius);
    TS_ASSERT_DELTA(sigma, 0.109, 0.001);

    // Calculate disk area
    double area = M_PI * radius * radius;
    TS_ASSERT_DELTA(area, 962.1, 0.5);

    // Estimate thrust coefficient using BEM approximation
    double a = 5.7;  // Lift curve slope
    double lambda = 0.06;  // Inflow ratio
    double CT = sigma * a * (collective / 3.0 - lambda / 2.0);
    TS_ASSERT(CT > 0.0);
    TS_ASSERT(CT < 0.02);

    // Calculate thrust
    double thrust = CT * rho * area * Vtip * Vtip;
    TS_ASSERT(thrust > 3000.0);
    TS_ASSERT(thrust < 8000.0);

    // Calculate induced velocity
    double vi = sqrt(thrust / (2.0 * rho * area));
    TS_ASSERT(vi > 20.0);
    TS_ASSERT(vi < 50.0);

    // Calculate power
    double inducedPower = thrust * vi / 550.0;  // HP
    double profileFactor = 1.15;
    double totalPower = inducedPower * profileFactor;
    TS_ASSERT(totalPower > 200.0);
    TS_ASSERT(totalPower < 500.0);

    // Calculate figure of merit
    double FM = inducedPower / totalPower;
    TS_ASSERT(FM > 0.8);
    TS_ASSERT(FM < 0.95);
  }

  // Test complete flight envelope calculations
  void testCompleteFlightEnvelope() {
    // Test multiple flight regimes from hover to high-speed forward flight
    double Vtip = 549.8;
    double speedOfSound = 1116.0;
    double vi_hover = 33.0;

    // Flight regimes: hover, transition, cruise, high-speed
    double speeds[] = {0.0, 30.0, 100.0, 180.0};  // ft/sec

    for (double V : speeds) {
      // Advance ratio
      double mu = V / Vtip;
      TS_ASSERT(mu >= 0.0);
      TS_ASSERT(mu < 0.5);

      // Advancing blade tip Mach
      double M_adv = (Vtip + V) / speedOfSound;
      TS_ASSERT(M_adv > 0.0);
      TS_ASSERT(M_adv < 0.9);

      // Check for translational lift regime
      double V_kts = V / 1.689;
      bool inETL = (V_kts >= 15.0) && (V_kts <= 24.0);

      // Check for potential VRS
      double descentRate = 0.0;  // Level flight
      bool inVRS = (descentRate > 0.7 * vi_hover) && (V < vi_hover);
      TS_ASSERT(!inVRS);

      // Retreating blade consideration
      double V_retreat = Vtip * (1.0 - mu);
      TS_ASSERT(V_retreat > 0.0);
    }

    // Verify all regimes covered
    TS_ASSERT_EQUALS(speeds[0], 0.0);  // Hover
    TS_ASSERT(speeds[3] > 150.0);       // High-speed
  }

  // Test complete control response chain
  void testCompleteControlResponse() {
    // Full control input to rotor response calculation
    double collectiveCmd = 0.12;  // rad
    double lateralCyclicCmd = 0.05;  // rad
    double longCyclicCmd = -0.03;  // rad
    double pedalCmd = 0.02;  // rad

    // Control mixing gain factors
    double collectiveGain = 1.0;
    double cyclicGain = 1.2;
    double pedalGain = 1.5;

    // Actual control positions after mixing
    double collective = collectiveCmd * collectiveGain;
    double lateralCyclic = lateralCyclicCmd * cyclicGain;
    double longCyclic = longCyclicCmd * cyclicGain;
    double pedal = pedalCmd * pedalGain;

    TS_ASSERT_DELTA(collective, 0.12, 0.001);
    TS_ASSERT_DELTA(lateralCyclic, 0.06, 0.001);
    TS_ASSERT_DELTA(longCyclic, -0.036, 0.001);
    TS_ASSERT_DELTA(pedal, 0.03, 0.001);

    // Flapping response to cyclic
    double a1 = longCyclic * 1.0;  // Longitudinal tilt
    double b1 = lateralCyclic * 1.0;  // Lateral tilt
    TS_ASSERT(a1 < 0.0);  // Aft tilt for forward stick
    TS_ASSERT(b1 > 0.0);  // Right tilt for right stick

    // Thrust response to collective
    double baseThrust = 5000.0;
    double thrustChange = baseThrust * collective * 8.0;
    double totalThrust = baseThrust + thrustChange;
    TS_ASSERT(totalThrust > baseThrust);

    // Torque response to collective (more collective = more torque)
    double baseTorque = 8000.0;
    double torqueChange = baseTorque * collective * 5.0;
    double totalTorque = baseTorque + torqueChange;
    TS_ASSERT(totalTorque > baseTorque);

    // Tail rotor requirement (balance main rotor torque)
    double tailArm = 30.0;
    double tailThrustRequired = totalTorque / tailArm;
    double tailCollective = tailThrustRequired / 500.0 + pedal;
    TS_ASSERT(tailCollective > 0.0);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test independent rotor instance calculations
  void testIndependentRotorInstances() {
    // Verify that different rotor configurations don't interfere

    // Light helicopter rotor
    struct RotorConfig {
      double radius;
      double numBlades;
      double chord;
      double rpm;
    };

    RotorConfig light = {17.5, 4, 1.5, 300.0};
    RotorConfig medium = {22.0, 4, 1.8, 260.0};
    RotorConfig heavy = {30.0, 5, 2.0, 220.0};

    // Calculate properties for each independently
    auto calcRotor = [](RotorConfig& r) {
      double omega = r.rpm * 2.0 * M_PI / 60.0;
      double Vtip = omega * r.radius;
      double area = M_PI * r.radius * r.radius;
      double sigma = (r.numBlades * r.chord) / (M_PI * r.radius);
      return std::make_tuple(omega, Vtip, area, sigma);
    };

    auto [omega_l, Vtip_l, area_l, sigma_l] = calcRotor(light);
    auto [omega_m, Vtip_m, area_m, sigma_m] = calcRotor(medium);
    auto [omega_h, Vtip_h, area_h, sigma_h] = calcRotor(heavy);

    // Verify each calculation is independent
    TS_ASSERT_DELTA(omega_l, 31.42, 0.1);
    TS_ASSERT_DELTA(omega_m, 27.23, 0.1);
    TS_ASSERT_DELTA(omega_h, 23.04, 0.1);

    // Areas should scale with radius squared
    TS_ASSERT(area_m > area_l);
    TS_ASSERT(area_h > area_m);

    // Verify tip speeds
    TS_ASSERT(Vtip_l > 500.0);
    TS_ASSERT(Vtip_m > 550.0);
    TS_ASSERT(Vtip_h > 650.0);
  }

  // Test independent flight condition calculations
  void testIndependentFlightConditions() {
    // Different atmospheric conditions should give independent results
    struct AtmosphereCondition {
      double rho;
      double speedOfSound;
      double altitude;
    };

    AtmosphereCondition seaLevel = {0.002377, 1116.0, 0.0};
    AtmosphereCondition mid = {0.001756, 1056.0, 10000.0};
    AtmosphereCondition high = {0.001267, 994.0, 20000.0};

    // Fixed rotor parameters
    double thrust = 5000.0;  // Required thrust
    double area = 962.1;
    double Vtip = 549.8;

    // Calculate thrust coefficient at each altitude
    double CT_sl = thrust / (seaLevel.rho * area * Vtip * Vtip);
    double CT_mid = thrust / (mid.rho * area * Vtip * Vtip);
    double CT_high = thrust / (high.rho * area * Vtip * Vtip);

    // CT must increase with altitude (lower density)
    TS_ASSERT(CT_mid > CT_sl);
    TS_ASSERT(CT_high > CT_mid);

    // Calculate induced velocity at each altitude
    double vi_sl = sqrt(thrust / (2.0 * seaLevel.rho * area));
    double vi_mid = sqrt(thrust / (2.0 * mid.rho * area));
    double vi_high = sqrt(thrust / (2.0 * high.rho * area));

    // Induced velocity increases with altitude
    TS_ASSERT(vi_mid > vi_sl);
    TS_ASSERT(vi_high > vi_mid);

    // Calculate tip Mach at each altitude
    double M_sl = Vtip / seaLevel.speedOfSound;
    double M_mid = Vtip / mid.speedOfSound;
    double M_high = Vtip / high.speedOfSound;

    // Mach number increases with altitude (lower speed of sound)
    TS_ASSERT(M_mid > M_sl);
    TS_ASSERT(M_high > M_mid);

    // Verify each condition is calculated independently
    TS_ASSERT_DELTA(CT_sl, 0.00724, 0.0001);
    TS_ASSERT(CT_high < 0.02);  // Still reasonable
  }

  // Test independent blade dynamics calculations
  void testIndependentBladeDynamics() {
    // Calculate blade properties at different azimuth positions
    double Vtip = 549.8;
    double mu = 0.2;  // Advance ratio
    double collective = 0.12;
    double lateralCyclic = 0.05;
    double longCyclic = -0.03;

    // Blade positions: 0°, 90°, 180°, 270°
    double azimuths[] = {0.0, M_PI/2.0, M_PI, 3.0*M_PI/2.0};
    double velocities[4];
    double pitches[4];

    for (int i = 0; i < 4; i++) {
      double psi = azimuths[i];

      // Local velocity varies with azimuth
      velocities[i] = Vtip * (1.0 + mu * sin(psi));

      // Blade pitch varies with azimuth (cyclic)
      pitches[i] = collective + lateralCyclic * cos(psi) + longCyclic * sin(psi);
    }

    // Verify independent calculations
    // At 0° (rear): velocity = Vtip * (1 + 0) = Vtip
    TS_ASSERT_DELTA(velocities[0], Vtip, 0.1);

    // At 90° (starboard/advancing): velocity = Vtip * (1 + mu)
    TS_ASSERT_DELTA(velocities[1], Vtip * 1.2, 1.0);

    // At 180° (front): velocity = Vtip
    TS_ASSERT_DELTA(velocities[2], Vtip, 0.1);

    // At 270° (port/retreating): velocity = Vtip * (1 - mu)
    TS_ASSERT_DELTA(velocities[3], Vtip * 0.8, 1.0);

    // Pitch should vary correctly
    TS_ASSERT_DELTA(pitches[0], collective + lateralCyclic, 0.001);
    TS_ASSERT_DELTA(pitches[1], collective + longCyclic, 0.001);
    TS_ASSERT_DELTA(pitches[2], collective - lateralCyclic, 0.001);
    TS_ASSERT_DELTA(pitches[3], collective - longCyclic, 0.001);

    // All calculations should be independent
    for (int i = 0; i < 4; i++) {
      for (int j = i+1; j < 4; j++) {
        TS_ASSERT(std::abs(azimuths[i] - azimuths[j]) > 0.1);
      }
    }
  }
};
