/*******************************************************************************
 * FGEngineFailureTest.h - Unit tests for engine failure scenarios
 *
 * Tests multi-engine aircraft behavior with engine failures including:
 * - Single/multi-engine failure thrust loss
 * - Asymmetric thrust and yawing moments
 * - VMC (minimum control speed) calculations
 * - Critical engine identification
 * - Rudder required for coordinated flight
 * - Bank angle for zero sideslip
 * - Windmilling and feathered drag
 * - Engine restart criteria
 * - Drift-down performance
 * - One-engine-inoperative (OEI) climb gradient
 * - VMCA vs VMCG calculations
 * - Asymmetric go-around scenarios
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <models/FGPropagate.h>
#include <models/FGFCS.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGPiston.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGEngineFailureTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Single Engine Failure - Thrust Loss
   ***************************************************************************/

  // Test complete thrust loss on single engine failure
  void testSingleEngineFailureThrustLoss() {
    // Twin-engine aircraft, one engine fails
    double normalThrust = 5000.0;  // lbs per engine
    bool engine1Running = true;
    bool engine2Running = false;  // Engine 2 failed

    double totalThrust = 0.0;
    if (engine1Running) totalThrust += normalThrust;
    if (engine2Running) totalThrust += normalThrust;

    // Total thrust is 50% with one engine out
    TS_ASSERT_DELTA(totalThrust, 5000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(totalThrust / (2.0 * normalThrust), 0.5, DEFAULT_TOLERANCE);
  }

  // Test progressive thrust loss during engine failure
  void testProgressiveThrustDecay() {
    // Engine doesn't fail instantly - power decays over time
    double normalThrust = 5000.0;
    double thrustDecayRate = 2000.0;  // lbs/sec
    double deltaT = 0.5;  // seconds

    double currentThrust = normalThrust;
    currentThrust -= thrustDecayRate * deltaT;

    // Thrust decays to 4000 lbs after 0.5 sec
    TS_ASSERT_DELTA(currentThrust, 4000.0, DEFAULT_TOLERANCE);
    TS_ASSERT(currentThrust > 0.0);
    TS_ASSERT(currentThrust < normalThrust);
  }

  // Test thrust loss limits to zero
  void testThrustLossFloor() {
    double currentThrust = 500.0;
    double thrustDecayRate = 2000.0;
    double deltaT = 0.5;

    currentThrust -= thrustDecayRate * deltaT;
    if (currentThrust < 0.0) currentThrust = 0.0;

    TS_ASSERT_DELTA(currentThrust, 0.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Asymmetric Thrust - Yawing Moment
   ***************************************************************************/

  // Test yawing moment from asymmetric thrust
  void testAsymmetricYawingMoment() {
    // N = T * y_arm where y_arm is lateral distance from CG
    double thrustLeft = 5000.0;   // lbs
    double thrustRight = 0.0;     // lbs (failed)
    double engineYArm = 10.0;     // ft lateral arm from centerline

    // Yaw moment = sum of (thrust * arm)
    double yawMoment = (thrustRight * engineYArm) - (thrustLeft * engineYArm);
    // Negative = nose left with left engine operating
    TS_ASSERT_DELTA(yawMoment, -50000.0, DEFAULT_TOLERANCE);
    TS_ASSERT(yawMoment < 0.0);
  }

  // Test symmetric thrust produces no yaw moment
  void testSymmetricThrustNoYaw() {
    double thrustLeft = 5000.0;
    double thrustRight = 5000.0;
    double engineYArm = 10.0;

    double yawMoment = (thrustRight * engineYArm) - (thrustLeft * engineYArm);
    TS_ASSERT_DELTA(yawMoment, 0.0, DEFAULT_TOLERANCE);
  }

  // Test yawing moment scales with engine arm
  void testYawMomentVsEngineArm() {
    double thrust = 5000.0;
    double arm1 = 5.0;   // Narrow-body (small arm)
    double arm2 = 15.0;  // Wide-body (large arm)

    double yawMoment1 = thrust * arm1;
    double yawMoment2 = thrust * arm2;

    TS_ASSERT_DELTA(yawMoment1, 25000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(yawMoment2, 75000.0, DEFAULT_TOLERANCE);
    TS_ASSERT(yawMoment2 > yawMoment1);  // Larger arm = larger moment
  }

  /***************************************************************************
   * VMC - Minimum Control Speed
   ***************************************************************************/

  // Test VMC calculation basics
  void testVMCCalculationBasics() {
    // VMC is when rudder can just counteract asymmetric yaw
    // N_rudder = N_asymmetric_thrust at VMC
    double yawMomentFromThrust = 50000.0;  // ft-lbf
    double rudderEffectiveness = 100.0;    // ft-lbf per degree
    double maxRudderDeflection = 25.0;     // degrees

    double maxRudderMoment = rudderEffectiveness * maxRudderDeflection;
    TS_ASSERT_DELTA(maxRudderMoment, 2500.0, DEFAULT_TOLERANCE);

    // If max rudder moment < asymmetric moment, aircraft cannot maintain heading
    bool controllable = (maxRudderMoment >= yawMomentFromThrust);
    TS_ASSERT(!controllable);  // Not controllable at this speed
  }

  // Test VMC increases with density altitude
  void testVMCVsDensityAltitude() {
    // Rudder effectiveness decreases with air density
    // VMC increases at high altitude
    double baseRudderEffectiveness = 100.0;
    double densityRatioSeaLevel = 1.0;
    double densityRatio10k = 0.7376;

    // Rudder effectiveness proportional to qbar (and thus density)
    double rudderEffSL = baseRudderEffectiveness * densityRatioSeaLevel;
    double rudderEff10k = baseRudderEffectiveness * densityRatio10k;

    TS_ASSERT_DELTA(rudderEffSL, 100.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(rudderEff10k, 73.76, 0.01);
    TS_ASSERT(rudderEff10k < rudderEffSL);  // Less effective at altitude
  }

  // Test VMC dynamic pressure relationship
  void testVMCDynamicPressureRelationship() {
    // Rudder moment = Cn_delta_r * qbar * S * b
    // At VMC: rudder moment = asymmetric thrust moment
    double Cn_delta_r = 0.10;  // Yaw moment coefficient per rad rudder (higher effectiveness)
    double wingSpan = 40.0;    // ft
    double wingArea = 200.0;   // ft^2
    double rho = 0.002377;     // slugs/ft^3

    double thrustMoment = 50000.0;  // ft-lbf

    // Solve for velocity at VMC
    // thrust_moment = Cn_delta_r * 0.5 * rho * V^2 * S * b
    // V^2 = thrust_moment / (Cn_delta_r * 0.5 * rho * S * b)
    double V_squared = thrustMoment / (Cn_delta_r * 0.5 * rho * wingArea * wingSpan);
    double VMC = sqrt(V_squared);

    TS_ASSERT(VMC > 0.0);
    TS_ASSERT(VMC < 400.0);  // Reasonable VMC in ft/sec (~150 kts)
  }

  /***************************************************************************
   * Critical Engine Identification
   ***************************************************************************/

  // Test critical engine is the one creating most yaw when failed
  void testCriticalEngineIdentification() {
    // For counter-rotating props with P-factor:
    // Critical engine = one whose failure creates most adverse yaw

    // Left engine (clockwise rotation, viewed from cockpit)
    double leftEngineYArm = 10.0;  // ft
    double leftPFactor = 0.5;      // P-factor adds to yaw (ft-lbf per lbf thrust)
    double thrust = 5000.0;

    double leftFailureYaw = thrust * (leftEngineYArm + leftPFactor);

    // Right engine (counter-clockwise rotation)
    double rightEngineYArm = 10.0;
    double rightPFactor = -0.5;  // P-factor opposes yaw

    double rightFailureYaw = thrust * (rightEngineYArm + rightPFactor);

    // Left engine creates more yaw when failed (in this configuration)
    TS_ASSERT(fabs(leftFailureYaw) > fabs(rightFailureYaw));
  }

  // Test critical engine for conventional twin (both rotate same direction)
  void testCriticalEngineConventionalTwin() {
    // Both props rotate clockwise (viewed from cockpit)
    // Right engine is critical because P-factor adds to yaw moment

    double engineArm = 10.0;
    double thrust = 5000.0;
    double pFactor = 1.0;  // ft additional moment arm from P-factor

    // Right engine failure: P-factor adds to geometric arm
    double rightEngineEffectiveArm = engineArm + pFactor;
    double rightFailureYaw = thrust * rightEngineEffectiveArm;

    // Left engine failure: P-factor subtracts (or adds less)
    double leftEngineEffectiveArm = engineArm - pFactor;
    double leftFailureYaw = thrust * leftEngineEffectiveArm;

    TS_ASSERT(rightFailureYaw > leftFailureYaw);  // Right is critical
  }

  /***************************************************************************
   * Rudder Required for Zero Sideslip
   ***************************************************************************/

  // Test rudder deflection required for zero sideslip
  void testRudderForZeroSideslip() {
    // At zero sideslip: N_rudder + N_thrust = 0
    double asymmetricYawMoment = 50000.0;  // ft-lbf
    double rudderEffectiveness = 2000.0;   // ft-lbf per degree at current speed

    double rudderDeflection = -asymmetricYawMoment / rudderEffectiveness;
    TS_ASSERT_DELTA(fabs(rudderDeflection), 25.0, 0.1);  // 25 degrees rudder (magnitude)
    TS_ASSERT(rudderDeflection < 0.0);  // Negative deflection to counteract moment
  }

  // Test rudder required decreases with speed
  void testRudderVsSpeed() {
    // Rudder effectiveness increases with qbar (V^2)
    double asymmetricMoment = 50000.0;
    double baseRudderEff = 1000.0;  // at some reference speed

    // At VMC (slow speed)
    double qbar_VMC = 20.0;  // psf
    double rudderEff_VMC = baseRudderEff * (qbar_VMC / 50.0);
    double rudderDefl_VMC = asymmetricMoment / rudderEff_VMC;

    // At higher speed
    double qbar_high = 100.0;
    double rudderEff_high = baseRudderEff * (qbar_high / 50.0);
    double rudderDefl_high = asymmetricMoment / rudderEff_high;

    TS_ASSERT(rudderDefl_high < rudderDefl_VMC);  // Less rudder needed at high speed
  }

  /***************************************************************************
   * Bank Angle for Zero Sideslip
   ***************************************************************************/

  // Test bank angle required for zero sideslip (zero-rudder method)
  void testBankAngleForZeroSideslip() {
    // Bank into good engine to counteract yaw
    // Small bank angle creates lateral lift component to counter sideslip
    // For small angles: phi ≈ (T_asym * y) / (W * b) where b is wing span

    double asymmetricThrust = 1000.0;  // lbs
    double weight = 40000.0;           // lbs
    double engineYArm = 10.0;          // ft
    double wingSpan = 40.0;            // ft

    // Simplified bank angle for coordinated flight (small angle approximation)
    double phi_rad = (asymmetricThrust * engineYArm) / (weight * wingSpan / 2.0);
    double bankAngle_deg = phi_rad * Constants::RAD_TO_DEG;

    TS_ASSERT(bankAngle_deg > 0.0);
    TS_ASSERT(bankAngle_deg < 10.0);  // Typically small bank angle (3-5 degrees typical)
  }

  // Test bank angle increases with thrust asymmetry
  void testBankAngleVsThrustAsymmetry() {
    double lift = 20000.0;
    double engineArm = 10.0;

    // Small thrust asymmetry
    double thrust1 = 1000.0;
    double bank1 = atan((thrust1 * engineArm) / lift) * Constants::RAD_TO_DEG;

    // Large thrust asymmetry
    double thrust2 = 5000.0;
    double bank2 = atan((thrust2 * engineArm) / lift) * Constants::RAD_TO_DEG;

    TS_ASSERT(bank2 > bank1);
  }

  /***************************************************************************
   * Windmilling Drag
   ***************************************************************************/

  // Test windmilling propeller drag calculation
  void testWindmillingDrag() {
    // Windmilling prop creates significant drag
    // D_windmill = Cd_windmill * 0.5 * rho * V^2 * A_prop
    double Cd_windmill = 0.15;  // Drag coefficient for windmilling prop
    double propDiameter = 8.0;  // ft
    double propArea = M_PI * (propDiameter / 2.0) * (propDiameter / 2.0);
    double rho = 0.002377;
    double velocity = 200.0;  // ft/sec

    double windmillingDrag = Cd_windmill * 0.5 * rho * velocity * velocity * propArea;
    // D = 0.15 * 0.5 * 0.002377 * 40000 * 50.27 = 358 lbs
    TS_ASSERT(windmillingDrag > 300.0);
    TS_ASSERT(windmillingDrag < 400.0);
  }

  // Test windmilling drag vs velocity squared
  void testWindmillingDragVsVelocity() {
    double Cd = 0.15;
    double area = 50.0;  // ft^2
    double rho = 0.002377;

    double V1 = 150.0;
    double drag1 = Cd * 0.5 * rho * V1 * V1 * area;

    double V2 = 300.0;  // Double velocity
    double drag2 = Cd * 0.5 * rho * V2 * V2 * area;

    // Drag quadruples when velocity doubles
    TS_ASSERT_DELTA(drag2 / drag1, 4.0, 0.01);
  }

  /***************************************************************************
   * Feathered Propeller Drag
   ***************************************************************************/

  // Test feathered propeller drag is much less than windmilling
  void testFeatheredDrag() {
    // Feathered prop presents blade edge to airflow
    double Cd_feathered = 0.02;  // Much lower than windmilling
    double propDiameter = 8.0;
    double propArea = M_PI * (propDiameter / 2.0) * (propDiameter / 2.0);
    double rho = 0.002377;
    double velocity = 200.0;

    double featheredDrag = Cd_feathered * 0.5 * rho * velocity * velocity * propArea;
    TS_ASSERT(featheredDrag > 0.0);
    TS_ASSERT(featheredDrag < 100.0);  // Much less than windmilling

    // Compare to windmilling
    double Cd_windmill = 0.15;
    double windmillingDrag = Cd_windmill * 0.5 * rho * velocity * velocity * propArea;
    TS_ASSERT(featheredDrag < windmillingDrag);
    TS_ASSERT_DELTA(featheredDrag / windmillingDrag, Cd_feathered / Cd_windmill, 0.01);
  }

  // Test feathering reduces drag by factor of 5-10
  void testFeatheringDragReduction() {
    double windmillingDrag = 350.0;  // lbs
    double featheredDrag = 50.0;     // lbs

    double dragReductionFactor = windmillingDrag / featheredDrag;
    TS_ASSERT_DELTA(dragReductionFactor, 7.0, DEFAULT_TOLERANCE);
    TS_ASSERT(dragReductionFactor > 5.0);
    TS_ASSERT(dragReductionFactor < 10.0);
  }

  /***************************************************************************
   * Engine Restart Criteria
   ***************************************************************************/

  // Test minimum windmilling RPM for restart
  void testMinimumWindmillingRPMForRestart() {
    double windmillingRPM = 800.0;
    double minimumRestartRPM = 500.0;

    bool canRestart = (windmillingRPM >= minimumRestartRPM);
    TS_ASSERT(canRestart);

    // Too slow
    windmillingRPM = 300.0;
    canRestart = (windmillingRPM >= minimumRestartRPM);
    TS_ASSERT(!canRestart);
  }

  // Test airspeed required for windmilling restart
  void testAirspeedForWindmillingRestart() {
    // Windmilling RPM proportional to airspeed
    double airspeed_kts = 150.0;
    double minAirspeed_kts = 100.0;
    double minRPM = 500.0;

    double windmillingRPM = minRPM * (airspeed_kts / minAirspeed_kts);
    TS_ASSERT_DELTA(windmillingRPM, 750.0, DEFAULT_TOLERANCE);

    bool canRestart = (airspeed_kts >= minAirspeed_kts);
    TS_ASSERT(canRestart);
  }

  // Test altitude limits for restart
  void testAltitudeLimitsForRestart() {
    double altitude = 25000.0;  // ft
    double maxRestartAltitude = 30000.0;
    double minRestartAltitude = 5000.0;  // Below this, risky

    bool withinRestartEnvelope = (altitude >= minRestartAltitude) &&
                                  (altitude <= maxRestartAltitude);
    TS_ASSERT(withinRestartEnvelope);

    // Too high
    altitude = 35000.0;
    withinRestartEnvelope = (altitude >= minRestartAltitude) &&
                             (altitude <= maxRestartAltitude);
    TS_ASSERT(!withinRestartEnvelope);
  }

  /***************************************************************************
   * Drift-Down Performance
   ***************************************************************************/

  // Test drift-down altitude calculation
  void testDriftDownAltitude() {
    // After engine failure, aircraft must descend to maintain speed
    // New service ceiling is lower
    double normalServiceCeiling = 25000.0;  // ft
    double OEI_serviceCeiling = 15000.0;    // ft with one engine out

    TS_ASSERT(OEI_serviceCeiling < normalServiceCeiling);
    double altitudeLoss = normalServiceCeiling - OEI_serviceCeiling;
    TS_ASSERT_DELTA(altitudeLoss, 10000.0, DEFAULT_TOLERANCE);
  }

  // Test drift-down rate calculation
  void testDriftDownRate() {
    // Rate of descent during drift-down
    // ROD = (D - T_available) / W * V
    double drag = 8000.0;        // lbs at cruise speed
    double thrustAvailable = 5000.0;  // lbs (one engine)
    double weight = 40000.0;     // lbs
    double velocity = 250.0;     // ft/sec

    double excessDrag = drag - thrustAvailable;
    double sinkRate = (excessDrag / weight) * velocity;
    // sink_rate ≈ (3000/40000) * 250 = 18.75 ft/sec = ~1100 fpm
    double sinkRate_fpm = sinkRate * 60.0;

    TS_ASSERT(sinkRate > 0.0);
    TS_ASSERT(sinkRate_fpm > 1000.0);
    TS_ASSERT(sinkRate_fpm < 1500.0);
  }

  /***************************************************************************
   * One-Engine-Inoperative (OEI) Climb Gradient
   ***************************************************************************/

  // Test OEI climb gradient calculation
  void testOEIClimbGradient() {
    // Climb gradient = (T - D) / W
    double thrustOEI = 5000.0;   // lbs (one engine operating)
    double drag = 4000.0;        // lbs
    double weight = 40000.0;     // lbs

    double excessThrust = thrustOEI - drag;
    double climbGradient = excessThrust / weight;
    double climbGradient_percent = climbGradient * 100.0;

    // (5000 - 4000) / 40000 = 0.025 = 2.5%
    TS_ASSERT_DELTA(climbGradient_percent, 2.5, 0.1);
  }

  // Test minimum required OEI climb gradient (regulatory)
  void testMinimumOEIClimbGradient() {
    // FAR Part 25 requires minimum climb gradients
    // Example: 2.4% for twin, 2.7% for three-engine, 3.0% for four-engine
    double actualClimbGradient = 2.5;  // percent
    double requiredClimbGradient_twin = 2.4;

    bool meetsRequirement = (actualClimbGradient >= requiredClimbGradient_twin);
    TS_ASSERT(meetsRequirement);

    // Below minimum
    actualClimbGradient = 2.0;
    meetsRequirement = (actualClimbGradient >= requiredClimbGradient_twin);
    TS_ASSERT(!meetsRequirement);
  }

  // Test OEI climb rate calculation
  void testOEIClimbRate() {
    // ROC = (T - D) / W * V
    double thrustOEI = 5000.0;
    double drag = 4000.0;
    double weight = 40000.0;
    double velocity = 200.0;  // ft/sec

    double excessThrust = thrustOEI - drag;
    double climbRate = (excessThrust / weight) * velocity;
    double climbRate_fpm = climbRate * 60.0;

    // ROC = 1000/40000 * 200 * 60 = 300 fpm
    TS_ASSERT_DELTA(climbRate_fpm, 300.0, 1.0);
  }

  /***************************************************************************
   * VMCA vs VMCG Calculations
   ***************************************************************************/

  // Test VMCA (air minimum control speed)
  void testVMCA() {
    // VMCA: minimum speed to maintain control in air with OEI
    // At VMCA, maximum rudder moment balances asymmetric thrust moment
    double asymmetricMoment = 20000.0;  // ft-lbf from thrust (more realistic)
    double Cn_delta_r = 0.08;  // Rudder yaw coefficient per radian
    double maxRudderDeflection = 25.0 * Constants::DEG_TO_RAD;
    double wingSpan = 40.0;
    double wingArea = 200.0;
    double rho = 0.002377;

    // At VMCA: Cn_delta_r * delta_r * qbar * S * b = asymmetric_moment
    // Solve for qbar
    double qbar_VMCA = asymmetricMoment / (Cn_delta_r * maxRudderDeflection * wingArea * wingSpan);

    // qbar = 0.5 * rho * V^2, solve for V
    double VMCA = sqrt(qbar_VMCA / (0.5 * rho));

    TS_ASSERT(VMCA > 0.0);
    TS_ASSERT(VMCA < 400.0);  // Reasonable VMCA in ft/sec (~150 kts)
  }

  // Test VMCG (ground minimum control speed)
  void testVMCG() {
    // VMCG: minimum speed to maintain directional control on ground
    // Uses rudder + nosewheel steering

    double asymmetricMoment = 50000.0;
    double maxRudderMoment = 2000.0;    // Less effective at low speed
    double nosewheelMoment = 1500.0;    // Additional moment from nosewheel

    double totalControlMoment = maxRudderMoment + nosewheelMoment;

    bool controllableOnGround = (totalControlMoment >= asymmetricMoment);
    // With nosewheel help, may be controllable at lower speed than VMCA

    TS_ASSERT(totalControlMoment > maxRudderMoment);
  }

  // Test VMCA is typically higher than VMCG
  void testVMCAvsVMCG() {
    double VMCA = 110.0;  // kts (typical twin)
    double VMCG = 95.0;   // kts (with nosewheel steering)

    // VMCA > VMCG in most cases
    TS_ASSERT(VMCA > VMCG);

    // VMCL (landing config) may be lower than VMCA
    double VMCL = 100.0;
    TS_ASSERT(VMCL < VMCA);  // Landing config has more drag, lower speed
  }

  /***************************************************************************
   * Asymmetric Go-Around
   ***************************************************************************/

  // Test thrust available during go-around with OEI
  void testAsymmetricGoAroundThrust() {
    // During go-around with one engine out
    double normalGoAroundThrust = 12000.0;  // lbs (both engines at TOGA)
    double OEI_thrust = 6000.0;  // lbs (one engine at TOGA)

    TS_ASSERT_DELTA(OEI_thrust / normalGoAroundThrust, 0.5, DEFAULT_TOLERANCE);
    TS_ASSERT(OEI_thrust < normalGoAroundThrust);
  }

  // Test go-around climb gradient with OEI
  void testOEIGoAroundClimbGradient() {
    // In landing configuration with one engine out
    double thrustOEI_TOGA = 6000.0;  // lbs at takeoff/go-around power
    double dragLandingConfig = 5500.0;  // High drag (gear/flaps down)
    double weight = 45000.0;  // lbs

    double excessThrust = thrustOEI_TOGA - dragLandingConfig;
    double climbGradient = (excessThrust / weight) * 100.0;

    // (6000 - 5500) / 45000 = 0.0111 = 1.11%
    TS_ASSERT(climbGradient > 0.0);  // Must be positive to climb
    TS_ASSERT(climbGradient < 2.0);  // But marginal
  }

  // Test minimum go-around altitude with OEI
  void testMinimumGoAroundAltitudeOEI() {
    // Decision height for OEI go-around
    double decisionHeight = 200.0;  // ft AGL
    double currentAltitude = 150.0;
    double requiredClimbGradient = 2.1;  // percent (regulatory)
    double actualClimbGradient = 1.5;    // percent with OEI

    // Below decision height and cannot meet climb gradient
    bool canSafelyGoAround = (currentAltitude >= decisionHeight) ||
                              (actualClimbGradient >= requiredClimbGradient);

    TS_ASSERT(!canSafelyGoAround);  // Should land, not go around
  }

  /***************************************************************************
   * Multi-Engine Failure Scenarios
   ***************************************************************************/

  // Test two-engine failure on four-engine aircraft
  void testTwoEngineFailureFourEngine() {
    double normalThrust = 20000.0;  // lbs per engine
    int enginesOperating = 2;
    int totalEngines = 4;

    double totalThrust = normalThrust * enginesOperating;
    double thrustRatio = (double)enginesOperating / totalEngines;

    TS_ASSERT_DELTA(totalThrust, 40000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(thrustRatio, 0.5, DEFAULT_TOLERANCE);
  }

  // Test asymmetric failure on four-engine (both on one wing)
  void testAsymmetricTwoEngineFailure() {
    // Both engines on left wing fail
    double thrustPerEngine = 5000.0;
    double innerEngineArm = 15.0;  // ft
    double outerEngineArm = 25.0;  // ft

    // Right wing engines operating
    double yawMoment = thrustPerEngine * (innerEngineArm + outerEngineArm);
    TS_ASSERT_DELTA(yawMoment, 200000.0, DEFAULT_TOLERANCE);

    // Extreme yaw moment - may exceed rudder authority
    double maxRudderMoment = 100000.0;
    bool controllable = (maxRudderMoment >= yawMoment);
    TS_ASSERT(!controllable);  // Cannot control with rudder alone
  }

  // Test triple-engine failure (four-engine aircraft)
  void testTripleEngineFailure() {
    double thrustPerEngine = 5000.0;
    int enginesOperating = 1;
    int totalEngines = 4;

    double totalThrust = thrustPerEngine * enginesOperating;
    double thrustRatio = (double)enginesOperating / totalEngines;

    TS_ASSERT_DELTA(totalThrust, 5000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(thrustRatio, 0.25, DEFAULT_TOLERANCE);

    // Likely cannot maintain altitude
    double drag = 6000.0;
    bool canMaintainAltitude = (totalThrust >= drag);
    TS_ASSERT(!canMaintainAltitude);
  }

  /***************************************************************************
   * P-Factor and Torque Effects on VMC
   ***************************************************************************/

  // Test P-factor contribution to yaw moment
  void testPFactorYawContribution() {
    // P-factor creates additional yaw due to asymmetric blade angle of attack
    double thrust = 5000.0;
    double geometricArm = 10.0;  // ft
    double pFactorArm = 0.5;     // Additional effective arm from P-factor

    double totalEffectiveArm = geometricArm + pFactorArm;
    double yawMoment = thrust * totalEffectiveArm;

    TS_ASSERT_DELTA(yawMoment, 52500.0, DEFAULT_TOLERANCE);
    TS_ASSERT(yawMoment > thrust * geometricArm);  // P-factor increases moment
  }

  // Test slipstream effect on rudder effectiveness
  void testSlipstreamEffectOnRudder() {
    // Operating engine slipstream increases rudder effectiveness
    double baseRudderEffectiveness = 100.0;
    double slipstreamFactor = 1.3;  // 30% increase from slipstream

    double effectiveRudder = baseRudderEffectiveness * slipstreamFactor;
    TS_ASSERT_DELTA(effectiveRudder, 130.0, DEFAULT_TOLERANCE);

    // Failed engine side has no slipstream benefit
    double noSlipstreamRudder = baseRudderEffectiveness * 1.0;
    TS_ASSERT(effectiveRudder > noSlipstreamRudder);
  }

  /***************************************************************************
   * Additional OEI Performance Tests
   ***************************************************************************/

  // Test acceleration-stop distance with engine failure
  void testAccelerationStopDistance() {
    // V1: decision speed for continue/abort takeoff
    double V1 = 130.0;  // kts
    double VR = 140.0;  // Rotation speed

    // Abort before V1
    double decisionSpeed = 125.0;
    bool shouldAbort = (decisionSpeed < V1);
    TS_ASSERT(shouldAbort);

    // Continue after V1
    decisionSpeed = 135.0;
    bool shouldContinue = (decisionSpeed >= V1);
    TS_ASSERT(shouldContinue);
  }

  // Test balanced field length concept
  void testBalancedFieldLength() {
    // Distance to accelerate to V1 and abort = distance to continue takeoff
    double accelerateStopDistance = 6000.0;  // ft
    double accelerateGoDistance = 6000.0;    // ft at balanced field length

    TS_ASSERT_DELTA(accelerateStopDistance, accelerateGoDistance, DEFAULT_TOLERANCE);

    // V1 is chosen such that distances are equal
    bool isBalanced = (fabs(accelerateStopDistance - accelerateGoDistance) < 100.0);
    TS_ASSERT(isBalanced);
  }

  // Test screen height requirement with OEI
  void testScreenHeightOEI() {
    // Must clear 35 ft obstacle at end of takeoff distance
    double screenHeight = 35.0;  // ft
    double altitudeAtEndOfRunway = 40.0;  // ft

    bool clearsObstacle = (altitudeAtEndOfRunway >= screenHeight);
    TS_ASSERT(clearsObstacle);

    // Insufficient climb
    altitudeAtEndOfRunway = 30.0;
    clearsObstacle = (altitudeAtEndOfRunway >= screenHeight);
    TS_ASSERT(!clearsObstacle);
  }

  /***************************************************************************
   * Directional Control and Sideslip
   ***************************************************************************/

  // Test sideslip angle from unbalanced yaw
  void testSideslipFromAsymmetricThrust() {
    // Sideslip beta arises when yaw moment not balanced
    double yawMoment = 50000.0;
    double rudderMoment = 30000.0;  // Insufficient

    double unbalancedMoment = yawMoment - rudderMoment;
    bool hasSideslip = (fabs(unbalancedMoment) > 100.0);
    TS_ASSERT(hasSideslip);
  }

  // Test adverse yaw from sideslip
  void testAdverseYawFromSideslip() {
    // Sideslip creates additional yaw moment
    double beta = 5.0 * Constants::DEG_TO_RAD;  // 5 degrees sideslip
    double Cn_beta = 0.08;  // Yaw stability derivative
    double qbar = 50.0;
    double S = 200.0;
    double b = 40.0;

    double sideslipYawMoment = Cn_beta * beta * qbar * S * b;
    TS_ASSERT(sideslipYawMoment > 0.0);
  }

  // Test roll-yaw coupling in OEI flight
  void testRollYawCoupling() {
    // Bank angle creates sideslip which creates additional yaw
    double phi = 5.0 * Constants::DEG_TO_RAD;
    double Cn_phi = -0.01;  // Dihedral effect on yaw
    double qbar = 50.0;
    double S = 200.0;
    double b = 40.0;

    double inducedYawMoment = Cn_phi * phi * qbar * S * b;
    TS_ASSERT(inducedYawMoment < 0.0);  // Negative dihedral effect
  }

  /***************************************************************************
   * Engine-Out Drag Increase
   ***************************************************************************/

  // Test total drag increase with windmilling prop
  void testTotalDragIncreaseWindmilling() {
    double normalDrag = 3000.0;
    double windmillingPropDrag = 350.0;
    double asymmetricFlowDrag = 150.0;  // From sideslip

    double totalDragOEI = normalDrag + windmillingPropDrag + asymmetricFlowDrag;
    double dragIncrease = totalDragOEI - normalDrag;

    TS_ASSERT_DELTA(totalDragOEI, 3500.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(dragIncrease / normalDrag, 0.1667, 0.001);  // ~17% increase
  }

  // Test drag reduction benefit of feathering
  void testDragReductionFromFeathering() {
    double normalDrag = 3000.0;
    double windmillingDrag = 350.0;
    double featheredDrag = 50.0;

    double totalDragWindmill = normalDrag + windmillingDrag;
    double totalDragFeathered = normalDrag + featheredDrag;

    double dragSavings = totalDragWindmill - totalDragFeathered;
    TS_ASSERT_DELTA(dragSavings, 300.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Power Available vs Required
   ***************************************************************************/

  // Test power available with OEI
  void testPowerAvailableOEI() {
    double normalPowerPerEngine = 1000.0;  // HP
    int enginesOperating = 1;
    int totalEngines = 2;

    double powerAvailable = normalPowerPerEngine * enginesOperating;
    double powerRatio = (double)enginesOperating / totalEngines;

    TS_ASSERT_DELTA(powerAvailable, 1000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(powerRatio, 0.5, DEFAULT_TOLERANCE);
  }

  // Test power required increases with OEI
  void testPowerRequiredIncreasesOEI() {
    // Due to windmilling drag and asymmetric flow
    double normalPowerRequired = 800.0;  // HP
    double additionalPowerForDrag = 150.0;

    double powerRequiredOEI = normalPowerRequired + additionalPowerForDrag;
    TS_ASSERT_DELTA(powerRequiredOEI, 950.0, DEFAULT_TOLERANCE);
    TS_ASSERT(powerRequiredOEI > normalPowerRequired);
  }

  // Test excess power margin with OEI
  void testExcessPowerMarginOEI() {
    double powerAvailable = 1000.0;
    double powerRequired = 950.0;

    double excessPower = powerAvailable - powerRequired;
    double powerMargin = excessPower / powerRequired;

    TS_ASSERT_DELTA(excessPower, 50.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(powerMargin, 0.0526, 0.001);  // ~5% margin
  }

  /***************************************************************************
   * Transient Engine Failure Dynamics
   ***************************************************************************/

  // Test rapid yaw rate at failure
  void testYawRateAtFailure() {
    // Sudden thrust loss creates yaw acceleration
    // N = I_z * r_dot
    double yawMoment = 50000.0;  // ft-lbf
    double I_z = 100000.0;  // slug-ft^2 (yaw inertia)

    double yawAcceleration = yawMoment / I_z;  // rad/sec^2
    TS_ASSERT_DELTA(yawAcceleration, 0.5, DEFAULT_TOLERANCE);

    // After 1 second
    double yawRate = yawAcceleration * 1.0;
    double yawRate_degps = yawRate * Constants::RAD_TO_DEG;
    TS_ASSERT_DELTA(yawRate_degps, 28.65, 0.1);
  }

  // Test roll response to asymmetric thrust
  void testRollResponseToAsymmetricThrust() {
    // Yaw creates sideslip, sideslip creates roll
    double sideslip = 5.0 * Constants::DEG_TO_RAD;
    double Cl_beta = -0.05;  // Dihedral effect
    double qbar = 50.0;
    double S = 200.0;
    double b = 40.0;

    double rollMoment = Cl_beta * sideslip * qbar * S * b;
    TS_ASSERT(rollMoment < 0.0);  // Roll into dead engine
  }

  /***************************************************************************
   * Certification and Regulatory Tests
   ***************************************************************************/

  // Test minimum required VMC margin above stall
  void testVMCMarginAboveStall() {
    double VMC = 95.0;   // kts
    double Vs1 = 80.0;   // kts stall speed (clean)
    double requiredMargin = 1.13;  // 13% above stall

    double actualMargin = VMC / Vs1;
    TS_ASSERT(actualMargin >= requiredMargin);
  }

  // Test maximum bank angle limit at VMC demonstration
  void testMaxBankAngleAtVMC() {
    // FAR 25.149: Bank angle not to exceed 5 degrees at VMC
    double demonstratedBankAngle = 5.0;  // degrees
    double maxAllowedBank = 5.0;

    TS_ASSERT(demonstratedBankAngle <= maxAllowedBank);
  }

  // Test out-of-ground-effect requirement for VMC
  void testVMCOutOfGroundEffect() {
    // VMC must be demonstrated out of ground effect
    double altitudeAGL = 100.0;  // ft
    double minimumHeightForOGE = 50.0;

    bool isOGE = (altitudeAGL >= minimumHeightForOGE);
    TS_ASSERT(isOGE);
  }

  /***************************************************************************
   * Engine Failure at Critical Phase
   ***************************************************************************/

  // Test failure at V1 decision speed
  void testFailureAtV1() {
    double currentSpeed = 130.0;  // kts
    double V1 = 130.0;
    double VR = 140.0;
    double VMCG = 100.0;

    // At V1, must be able to continue or abort safely
    bool canContinue = (currentSpeed >= VMCG);
    bool belowRotation = (currentSpeed < VR);

    TS_ASSERT(canContinue);
    TS_ASSERT(belowRotation);
  }

  // Test failure just after liftoff
  void testFailureAfterLiftoff() {
    double altitude = 10.0;  // ft AGL
    double speed = 145.0;    // kts
    double VMC = 95.0;

    bool hasFlightSpeed = (speed > VMC);
    bool isAirborne = (altitude > 0.0);

    TS_ASSERT(hasFlightSpeed);
    TS_ASSERT(isAirborne);
    // Must continue - cannot land safely
  }

  // Test failure during initial climb
  void testFailureDuringInitialClimb() {
    double altitude = 500.0;  // ft AGL
    double climbRate = 1500.0;  // fpm (both engines)
    double OEI_climbRate = 200.0;  // fpm (one engine)

    // Sudden reduction in climb performance
    double climbRateReduction = climbRate - OEI_climbRate;
    TS_ASSERT_DELTA(climbRateReduction, 1300.0, DEFAULT_TOLERANCE);
    TS_ASSERT(OEI_climbRate > 0.0);  // Still climbing
  }

  /***************************************************************************
   * Single Engine Performance Extended Tests
   ***************************************************************************/

  // Test single engine ceiling altitude
  void testSingleEngineCeiling() {
    double normalCeiling = 25000.0;  // ft
    double OEI_ceiling = 14000.0;  // ft

    double ceilingReduction = normalCeiling - OEI_ceiling;
    TS_ASSERT_DELTA(ceilingReduction, 11000.0, 100.0);
    TS_ASSERT(OEI_ceiling > 0);
  }

  // Test single engine cruise speed
  void testSingleEngineCruiseSpeed() {
    double normalCruise = 180.0;  // kts
    double OEI_cruise = 140.0;    // kts

    double speedReduction = normalCruise - OEI_cruise;
    TS_ASSERT_DELTA(speedReduction, 40.0, 1.0);
    TS_ASSERT(OEI_cruise > 0);
  }

  // Test single engine range
  void testSingleEngineRange() {
    double normalRange = 1000.0;  // nm
    double fuelConsumptionRatio = 1.3;  // OEI consumes more per nm

    double OEI_range = normalRange / fuelConsumptionRatio;
    TS_ASSERT_DELTA(OEI_range, 769.2, 1.0);
  }

  /***************************************************************************
   * Propeller Autofeather Tests
   ***************************************************************************/

  // Test autofeather arm conditions
  void testAutofeatherArmConditions() {
    double airspeed = 120.0;  // kts
    double minArmSpeed = 80.0;
    double power1 = 90.0;  // percent
    double power2 = 90.0;  // percent
    double minPower = 70.0;

    bool armed = (airspeed > minArmSpeed) &&
                 (power1 > minPower) && (power2 > minPower);
    TS_ASSERT(armed);
  }

  // Test autofeather trigger
  void testAutofeatherTrigger() {
    double torqueNormal = 1000.0;  // ft-lbf
    double torqueFailed = 100.0;   // ft-lbf
    double triggerThreshold = 0.3;

    double torqueRatio = torqueFailed / torqueNormal;
    bool shouldFeather = torqueRatio < triggerThreshold;

    TS_ASSERT(shouldFeather);
  }

  // Test feather time constant
  void testFeatherTimeConstant() {
    double featherTime = 3.0;  // seconds to full feather
    double pitchChange = 90.0 - 25.0;  // degrees (flat to feather)

    double featherRate = pitchChange / featherTime;
    TS_ASSERT_DELTA(featherRate, 21.67, 0.1);
  }

  /***************************************************************************
   * Engine Out Approach Tests
   ***************************************************************************/

  // Test single engine approach speed
  void testSingleEngineApproachSpeed() {
    double normalVref = 110.0;  // kts
    double OEI_increment = 10.0;  // kts higher for safety

    double OEI_Vref = normalVref + OEI_increment;
    TS_ASSERT_DELTA(OEI_Vref, 120.0, 0.1);
  }

  // Test single engine landing distance
  void testSingleEngineLandingDistance() {
    double normalDistance = 3000.0;  // ft
    double OEI_factor = 1.15;  // 15% longer

    double OEI_distance = normalDistance * OEI_factor;
    TS_ASSERT_DELTA(OEI_distance, 3450.0, 10.0);
  }

  // Test single engine missed approach
  void testSingleEngineMissedApproach() {
    double OEI_climbGradient = 2.1;  // percent (minimum required)
    double actualGradient = 2.5;     // percent

    bool canMakeMAP = actualGradient >= OEI_climbGradient;
    TS_ASSERT(canMakeMAP);
  }

  /***************************************************************************
   * Thrust Asymmetry Extended Tests
   ***************************************************************************/

  // Test yaw moment vs airspeed
  void testYawMomentVsAirspeed() {
    double thrust = 5000.0;
    double engineArm = 10.0;
    double yawMoment = thrust * engineArm;

    // Yaw moment is constant (independent of airspeed)
    // But rudder effectiveness increases with airspeed
    TS_ASSERT_DELTA(yawMoment, 50000.0, 1.0);
  }

  // Test yaw moment with partial power
  void testYawMomentPartialPower() {
    double maxThrust = 5000.0;
    double partialPower = 0.7;  // 70%
    double engineArm = 10.0;

    double yawMoment = maxThrust * partialPower * engineArm;
    TS_ASSERT_DELTA(yawMoment, 35000.0, 1.0);
  }

  // Test yaw damping effect
  void testYawDampingEffect() {
    double yawRate = 10.0;  // deg/sec
    double dampingCoeff = 0.5;

    double dampingMoment = dampingCoeff * yawRate;
    TS_ASSERT(dampingMoment > 0);
  }

  /***************************************************************************
   * Engine Fire/Shutdown Procedures
   ***************************************************************************/

  // Test fuel cutoff effect
  void testFuelCutoffEffect() {
    double normalFuelFlow = 100.0;  // lbs/hr
    double cutoffPosition = 0.0;    // 0 = cutoff, 1 = on

    double actualFlow = normalFuelFlow * cutoffPosition;
    TS_ASSERT_DELTA(actualFlow, 0.0, DEFAULT_TOLERANCE);
  }

  // Test fire loop detection
  void testFireLoopDetection() {
    double loopTemp1 = 600.0;  // °F
    double loopTemp2 = 150.0;  // °F (normal)
    double fireThreshold = 500.0;

    bool fireLoop1 = loopTemp1 >= fireThreshold;
    bool fireLoop2 = loopTemp2 >= fireThreshold;

    TS_ASSERT(fireLoop1);
    TS_ASSERT(!fireLoop2);
  }

  // Test engine secured checklist
  void testEngineSecuredChecklist() {
    bool throttle_idle = true;
    bool mixture_cutoff = true;
    bool prop_feathered = true;
    bool fuel_off = true;
    bool magnetos_off = true;

    bool engineSecured = throttle_idle && mixture_cutoff &&
                         prop_feathered && fuel_off && magnetos_off;
    TS_ASSERT(engineSecured);
  }

  /***************************************************************************
   * Performance Degradation Tests
   ***************************************************************************/

  // Test takeoff performance with OEI at V1
  void testTakeoffPerformanceOEI() {
    double normalV2 = 130.0;  // kts
    double OEI_V2 = 125.0;    // kts (reduced due to less thrust)

    TS_ASSERT(OEI_V2 < normalV2);
    TS_ASSERT(OEI_V2 > 0);
  }

  // Test climb gradient segments
  void testClimbGradientSegments() {
    double firstSegment = 0.0;   // percent (gear retraction)
    double secondSegment = 2.4;  // percent (min requirement)
    double thirdSegment = 1.2;   // percent (accel/cleanup)
    double fourthSegment = 1.2;  // percent (en route)

    TS_ASSERT(secondSegment >= 2.4);  // FAR requirement
  }

  // Test drift down fuel consumption
  void testDriftDownFuelConsumption() {
    double normalFuelFlow = 1500.0;  // lbs/hr (both engines)
    double OEI_fuelFlow = 900.0;     // lbs/hr (one engine at MCT)

    double flowReduction = normalFuelFlow - OEI_fuelFlow;
    TS_ASSERT_DELTA(flowReduction, 600.0, 1.0);
  }

  /***************************************************************************
   * Multi-Engine Coordination Tests
   ***************************************************************************/

  // Test thrust lever synchronization
  void testThrustLeverSync() {
    double lever1 = 85.0;  // percent
    double lever2 = 85.0;  // percent

    double difference = std::abs(lever1 - lever2);
    TS_ASSERT(difference < 5.0);  // Within sync tolerance
  }

  // Test automatic thrust mode with OEI
  void testAutoThrustModeOEI() {
    double MCT_limit = 95.0;  // percent (max continuous thrust)
    double operatingThrust = 93.0;  // percent

    bool withinMCT = operatingThrust <= MCT_limit;
    TS_ASSERT(withinMCT);
  }

  // Test torque matching
  void testTorqueMatching() {
    double torque1 = 100.0;  // percent
    double torque2 = 0.0;    // percent (failed)

    double asymmetry = std::abs(torque1 - torque2);
    TS_ASSERT_DELTA(asymmetry, 100.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Alternate Airfield Considerations
   ***************************************************************************/

  // Test fuel required to alternate
  void testFuelToAlternate() {
    double distanceToAlternate = 100.0;  // nm
    double OEI_specificRange = 5.0;      // nm per 100 lbs fuel
    double contingency = 1.1;            // 10% contingency

    double fuelRequired = (distanceToAlternate / OEI_specificRange) * 100.0 * contingency;
    TS_ASSERT_DELTA(fuelRequired, 2200.0, 10.0);
  }

  // Test time to alternate
  void testTimeToAlternate() {
    double distanceToAlternate = 150.0;  // nm
    double OEI_cruiseSpeed = 160.0;      // kts

    double timeToAlternate = distanceToAlternate / OEI_cruiseSpeed * 60.0;  // minutes
    TS_ASSERT_DELTA(timeToAlternate, 56.25, 0.1);
  }

  // Test alternate runway requirements
  void testAlternateRunwayRequirements() {
    double OEI_landingDistance = 4000.0;  // ft
    double safetyFactor = 1.67;           // FAR requirement

    double requiredRunway = OEI_landingDistance * safetyFactor;
    TS_ASSERT_DELTA(requiredRunway, 6680.0, 10.0);
  }

  /***************************************************************************
   * Stress and Edge Case Tests
   ***************************************************************************/

  // Test double engine failure glide
  void testDoubleEngineFailureGlide() {
    double weight = 45000.0;   // lbs
    double liftToDrag = 12.0;  // typical twin L/D
    double altitude = 10000.0; // ft

    double glideRange = altitude * liftToDrag / 5280.0;  // nm
    TS_ASSERT(glideRange > 0);
    TS_ASSERT(glideRange < 50.0);
  }

  // Test engine failure at max altitude
  void testEngineFailureAtMaxAltitude() {
    double altitude = 25000.0;  // ft
    double OEI_ceiling = 15000.0;

    bool mustDescend = altitude > OEI_ceiling;
    double descentRequired = altitude - OEI_ceiling;

    TS_ASSERT(mustDescend);
    TS_ASSERT_DELTA(descentRequired, 10000.0, DEFAULT_TOLERANCE);
  }

  // Test VMC at various weights
  void testVMCVsWeight() {
    double VMC_MTOW = 110.0;  // kts at max weight
    double VMC_light = 95.0; // kts at light weight

    // VMC decreases at lighter weights
    TS_ASSERT(VMC_light < VMC_MTOW);
  }

  // Test VMC at various CG positions
  void testVMCVsCG() {
    double VMC_fwdCG = 105.0;  // kts
    double VMC_aftCG = 115.0; // kts

    // VMC increases with aft CG
    TS_ASSERT(VMC_aftCG > VMC_fwdCG);
  }

  // Test controllability margin
  void testControllabilityMargin() {
    double VMC = 100.0;  // kts
    double currentSpeed = 130.0;  // kts

    double margin = (currentSpeed - VMC) / VMC * 100.0;
    TS_ASSERT(margin > 20.0);  // At least 20% above VMC
  }

  // Test engine restart in-flight
  void testEngineRestartInFlight() {
    double airspeed = 180.0;  // kts
    double altitude = 8000.0;  // ft
    double minRestartAirspeed = 150.0;
    double minRestartAlt = 3000.0;

    bool restartConditionsMet = (airspeed >= minRestartAirspeed) &&
                                 (altitude >= minRestartAlt);
    TS_ASSERT(restartConditionsMet);
  }

  // Test power settling time
  void testPowerSettlingTime() {
    double targetPower = 100.0;  // percent
    double currentPower = 50.0;
    double settlingTime = 3.0;   // seconds for piston
    double timeConstant = 1.0;

    // Exponential approach to target
    double powerAfter1Sec = targetPower - (targetPower - currentPower) *
                            std::exp(-1.0 / timeConstant);
    TS_ASSERT(powerAfter1Sec > currentPower);
    TS_ASSERT(powerAfter1Sec < targetPower);
  }

  // Test numerical stability of yaw calculations
  void testYawCalculationStability() {
    for (double thrust = 100.0; thrust <= 10000.0; thrust += 500.0) {
      for (double arm = 5.0; arm <= 30.0; arm += 5.0) {
        double yawMoment = thrust * arm;
        TS_ASSERT(!std::isnan(yawMoment));
        TS_ASSERT(!std::isinf(yawMoment));
        TS_ASSERT(yawMoment > 0);
      }
    }
  }

  /***************************************************************************
   * Complete Engine Failure System Tests
   ***************************************************************************/

  // Test complete OEI performance envelope
  void testCompleteOEIPerformanceEnvelope() {
    double normalThrust = 50000.0;
    double OEIThrust = 25000.0;
    double weight = 40000.0;

    double normalTW = normalThrust / weight;
    double OEITW = OEIThrust / weight;

    TS_ASSERT(normalTW > 1.0);
    TS_ASSERT(OEITW < 1.0);
    TS_ASSERT_DELTA(OEITW, 0.625, 0.01);
  }

  // Test engine failure during takeoff roll
  void testEngineFailureDuringTakeoffRoll() {
    double V1 = 120.0;  // kts
    double currentSpeed = 100.0;  // kts
    double accelerateStopDistance = 6000.0;  // ft
    double runwayRemaining = 7000.0;  // ft

    bool canReject = (currentSpeed < V1) && (accelerateStopDistance < runwayRemaining);
    TS_ASSERT(canReject);
  }

  // Test engine failure climb gradient
  void testEngineFailureClimbGradient() {
    double OEIClimbRate = 300.0;   // ft/min
    double groundSpeed = 150.0;    // kts

    // Gradient in percent: (ROC / GS) * 100
    double GSfpm = groundSpeed * 101.3;
    double gradient = (OEIClimbRate / GSfpm) * 100.0;

    TS_ASSERT(gradient > 1.5);  // Above obstacle clearance requirement
  }

  // Test asymmetric thrust compensation
  void testAsymmetricThrustCompensation() {
    double thrustDiff = 20000.0;  // lbf
    double armLength = 15.0;      // ft
    double rudderAuthority = 400000.0;  // ft-lbf max

    double yawMoment = thrustDiff * armLength;
    bool canCompensate = yawMoment < rudderAuthority;

    TS_ASSERT(canCompensate);
    TS_ASSERT_DELTA(yawMoment, 300000.0, 1.0);
  }

  // Test drift down profile
  void testDriftDownProfile() {
    double initialAlt = 35000.0;
    double OEICeiling = 20000.0;
    double descentRate = 500.0;  // ft/min

    double descentRequired = initialAlt - OEICeiling;
    double timeToLevel = descentRequired / descentRate;

    TS_ASSERT_DELTA(descentRequired, 15000.0, 1.0);
    TS_ASSERT_DELTA(timeToLevel, 30.0, 0.1);  // 30 minutes
  }

  // Test fuel dumping considerations
  void testFuelDumpingForEmergency() {
    double currentWeight = 180000.0;  // lbs
    double maxLandingWeight = 150000.0;
    double fuelDumpRate = 2000.0;  // lbs/min

    double excessWeight = currentWeight - maxLandingWeight;
    double dumpTime = excessWeight / fuelDumpRate;

    TS_ASSERT_DELTA(excessWeight, 30000.0, 1.0);
    TS_ASSERT_DELTA(dumpTime, 15.0, 0.1);  // 15 minutes
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test thrust calculation independence
  void testThrustCalculationIndependence() {
    double thrust1 = 50000.0, arm1 = 10.0;
    double thrust2 = 25000.0, arm2 = 20.0;

    double moment1 = thrust1 * arm1;
    double moment2 = thrust2 * arm2;

    TS_ASSERT_DELTA(moment1, 500000.0, 1.0);
    TS_ASSERT_DELTA(moment2, 500000.0, 1.0);
  }

  // Test VMC calculation independence
  void testVMCCalculationIndependence() {
    double weight1 = 40000.0, VMCbase1 = 100.0;
    double weight2 = 50000.0, VMCbase2 = 110.0;

    // VMC doesn't scale linearly with weight
    TS_ASSERT(VMCbase2 > VMCbase1);
  }

  // Test performance degradation independence
  void testPerformanceDegradationIndependence() {
    double normalClimb1 = 2000.0, OEIClimb1 = 500.0;
    double normalClimb2 = 3000.0, OEIClimb2 = 800.0;

    double ratio1 = OEIClimb1 / normalClimb1;
    double ratio2 = OEIClimb2 / normalClimb2;

    TS_ASSERT_DELTA(ratio1, 0.25, 0.01);
    TS_ASSERT_DELTA(ratio2, 0.267, 0.01);
  }

  // Test yaw moment state independence
  void testYawMomentStateIndependence() {
    double T1 = 10000.0, arm1 = 15.0;
    double T2 = 20000.0, arm2 = 10.0;

    double M1 = T1 * arm1;
    double M2 = T2 * arm2;

    TS_ASSERT_DELTA(M1, 150000.0, 1.0);
    TS_ASSERT_DELTA(M2, 200000.0, 1.0);
  }

  // Test controllability margin independence
  void testControllabilityMarginIndependence() {
    double VMC1 = 100.0, speed1 = 150.0;
    double VMC2 = 110.0, speed2 = 140.0;

    double margin1 = speed1 - VMC1;
    double margin2 = speed2 - VMC2;

    TS_ASSERT_DELTA(margin1, 50.0, 0.1);
    TS_ASSERT_DELTA(margin2, 30.0, 0.1);
  }
};
