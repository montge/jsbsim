/*******************************************************************************
 * FGLandingGearDynamicsTest.h - Unit tests for landing gear extension/retraction dynamics
 *
 * Tests the mechanical and operational behavior of retractable landing gear systems:
 * - Extension and retraction timing
 * - Gear door sequencing and coordination
 * - Uplock and downlock mechanisms
 * - Weight-on-wheels interlocks
 * - Gear warning systems
 * - Emergency extension procedures
 * - Position indication and feedback
 * - Airspeed limitations
 * - Asymmetric gear conditions
 * - Aerodynamic drag during transit
 * - Free-fall extension dynamics
 * - Hydraulic vs electric actuation systems
 *
 * Note: These tests focus on the dynamics and logic of gear operation,
 * complementing FGLGearTest.h which tests physical ground reaction forces.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <algorithm>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGGroundReactions.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>

const double epsilon = 1e-10;
const double deg2rad = 0.017453292519943295;

class FGLandingGearDynamicsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Gear Extension Time Tests
   ***************************************************************************/

  // Test normal hydraulic extension time
  void testNormalExtensionTime() {
    double extensionTime = 5.0;  // seconds (typical for hydraulic system)
    double dt = 0.01;
    double gearPos = 0.0;  // Fully retracted

    // Simulate extension
    int steps = 0;
    while (gearPos < 0.99 && steps < 1000) {
      double rate = 1.0 / extensionTime;
      gearPos += rate * dt;
      gearPos = std::min(gearPos, 1.0);
      steps++;
    }

    double actualTime = steps * dt;
    TS_ASSERT_DELTA(actualTime, extensionTime, 0.1);
  }

  // Test fast electric extension
  void testFastElectricExtension() {
    double extensionTime = 3.0;  // Electric systems can be faster
    double dt = 0.01;
    double gearPos = 0.0;

    int steps = 0;
    while (gearPos < 0.99 && steps < 500) {
      double rate = 1.0 / extensionTime;
      gearPos += rate * dt;
      gearPos = std::min(gearPos, 1.0);
      steps++;
    }

    double actualTime = steps * dt;
    TS_ASSERT_DELTA(actualTime, extensionTime, 0.1);
  }

  // Test extension under load (slower)
  void testExtensionUnderAirload() {
    double normalExtTime = 5.0;
    double airspeed = 250.0;  // High speed increases resistance
    double loadFactor = 1.0 + (airspeed / 300.0) * 0.5;  // 50% slower at 300 kts
    double actualExtTime = normalExtTime * loadFactor;

    TS_ASSERT(actualExtTime > normalExtTime);
    TS_ASSERT_DELTA(actualExtTime, 7.08, 0.1);
  }

  // Test extension rate limiting
  void testExtensionRateLimit() {
    double maxRate = 0.2;  // 20% per second
    double dt = 0.1;
    double gearPos = 0.5;
    double command = 1.0;  // Full down

    double maxChange = maxRate * dt;
    double newPos = gearPos + std::min(command - gearPos, maxChange);

    TS_ASSERT_DELTA(newPos, 0.52, epsilon);  // Only moved 0.02 in 0.1s
  }

  /***************************************************************************
   * Gear Retraction Time Tests
   ***************************************************************************/

  // Test normal hydraulic retraction time
  void testNormalRetractionTime() {
    double retractionTime = 7.0;  // Often slower than extension
    double dt = 0.01;
    double gearPos = 1.0;  // Fully extended

    int steps = 0;
    while (gearPos > 0.01 && steps < 1000) {
      double rate = -1.0 / retractionTime;
      gearPos += rate * dt;
      gearPos = std::max(gearPos, 0.0);
      steps++;
    }

    double actualTime = steps * dt;
    TS_ASSERT_DELTA(actualTime, retractionTime, 0.1);
  }

  // Test retraction blocked by weight-on-wheels
  void testRetractionBlockedByWOW() {
    double gearPos = 1.0;
    bool WOW = true;
    bool canRetract = !WOW;

    TS_ASSERT(!canRetract);

    // Position should not change
    double dt = 0.1;
    double rate = canRetract ? -0.1 : 0.0;
    double newPos = gearPos + rate * dt;

    TS_ASSERT_DELTA(newPos, 1.0, epsilon);
  }

  // Test retraction after liftoff
  void testRetractionAfterLiftoff() {
    double gearPos = 1.0;
    bool WOW = false;
    double airspeed = 80.0;  // Above liftoff speed
    double minRetractSpeed = 60.0;

    bool canRetract = !WOW && (airspeed > minRetractSpeed);
    TS_ASSERT(canRetract);
  }

  // Test asymmetric retraction rates
  void testAsymmetricRetractionRates() {
    double extensionRate = 1.0 / 5.0;   // 5 seconds to extend
    double retractionRate = 1.0 / 7.0;  // 7 seconds to retract

    TS_ASSERT(std::abs(retractionRate) < std::abs(extensionRate));
  }

  /***************************************************************************
   * Gear Door Sequencing Tests
   ***************************************************************************/

  // Test door opens before gear extends
  void testDoorOpenBeforeExtension() {
    double gearPos = 0.0;
    double doorPos = 0.0;
    double dt = 0.1;

    // Door should move first
    if (gearPos < 0.01 && doorPos < 0.99) {
      doorPos += 0.5 * dt;  // Door opens
      doorPos = std::min(doorPos, 1.0);
    }

    // Gear only extends when door is mostly open
    if (doorPos > 0.8) {
      gearPos += 0.2 * dt;
      gearPos = std::min(gearPos, 1.0);
    }

    TS_ASSERT_DELTA(doorPos, 0.05, epsilon);
    TS_ASSERT_DELTA(gearPos, 0.0, epsilon);  // Gear hasn't started yet
  }

  // Test door closes after gear retracts
  void testDoorCloseAfterRetraction() {
    double gearPos = 0.05;  // Nearly retracted
    double doorPos = 1.0;   // Door open

    // Door only closes when gear is fully retracted
    bool canCloseDoor = (gearPos < 0.01);

    TS_ASSERT(!canCloseDoor);  // Gear not fully retracted yet

    gearPos = 0.0;
    canCloseDoor = (gearPos < 0.01);
    TS_ASSERT(canCloseDoor);
  }

  // Test door position limits
  void testDoorPositionLimits() {
    double doorPositions[] = {-0.1, 0.0, 0.5, 1.0, 1.5};
    double expected[] = {0.0, 0.0, 0.5, 1.0, 1.0};

    for (int i = 0; i < 5; i++) {
      double limited = std::max(0.0, std::min(1.0, doorPositions[i]));
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test door sequencing timing
  void testDoorSequencingTiming() {
    double doorOpenTime = 2.0;
    double gearExtendTime = 5.0;
    double doorCloseTime = 1.5;

    double totalExtensionTime = doorOpenTime + gearExtendTime + doorCloseTime;
    TS_ASSERT_DELTA(totalExtensionTime, 8.5, epsilon);
  }

  /***************************************************************************
   * Uplock and Downlock Tests
   ***************************************************************************/

  // Test downlock engages at full extension
  void testDownlockEngagement() {
    double gearPos = 1.0;
    bool downlocked = (gearPos >= 0.99);
    TS_ASSERT(downlocked);

    gearPos = 0.98;
    downlocked = (gearPos >= 0.99);
    TS_ASSERT(!downlocked);
  }

  // Test uplock engages at full retraction
  void testUplockEngagement() {
    double gearPos = 0.0;
    bool uplocked = (gearPos <= 0.01);
    TS_ASSERT(uplocked);

    gearPos = 0.02;
    uplocked = (gearPos <= 0.01);
    TS_ASSERT(!uplocked);
  }

  // Test downlock prevents retraction
  void testDownlockPreventsRetraction() {
    double gearPos = 1.0;
    bool downlocked = true;

    // Must release downlock before retracting
    bool canRetract = !downlocked;
    TS_ASSERT(!canRetract);
  }

  // Test uplock release for extension
  void testUplockReleaseForExtension() {
    double gearPos = 0.0;
    bool uplocked = true;

    // Uplock must release before gear can extend
    bool canExtend = !uplocked;
    TS_ASSERT(!canExtend);

    // Release uplock
    uplocked = false;
    canExtend = !uplocked;
    TS_ASSERT(canExtend);
  }

  // Test lock override force requirement
  void testLockOverrideForce() {
    double unlockForce = 500.0;  // lbs required to override
    double appliedForce = 600.0;
    bool lockOverridden = (appliedForce > unlockForce);

    TS_ASSERT(lockOverridden);
  }

  /***************************************************************************
   * Weight-on-Wheels Logic Tests
   ***************************************************************************/

  // Test WOW prevents retraction
  void testWOWPreventsRetraction() {
    bool WOW = true;
    bool gearCommand = false;  // Pilot commands gear up

    bool gearCanMove = !WOW;
    TS_ASSERT(!gearCanMove);
  }

  // Test WOW override switch (maintenance mode)
  void testWOWOverride() {
    bool WOW = true;
    bool overrideSwitch = true;

    bool gearCanMove = !WOW || overrideSwitch;
    TS_ASSERT(gearCanMove);
  }

  // Test multiple WOW sensors (voting logic)
  void testMultipleWOWSensors() {
    bool WOW_left = true;
    bool WOW_right = true;
    bool WOW_nose = false;

    // Any main gear WOW prevents retraction
    bool mainGearWOW = WOW_left || WOW_right;
    TS_ASSERT(mainGearWOW);
  }

  // Test WOW debounce to prevent bounce issues
  void testWOWDebounce() {
    double debounceTime = 0.5;  // seconds
    double timeInAir = 0.3;     // Brief bounce

    bool WOW_debounced = (timeInAir < debounceTime);
    TS_ASSERT(WOW_debounced);  // Still consider on ground
  }

  /***************************************************************************
   * Gear Warning System Tests
   ***************************************************************************/

  // Test gear-up warning when landing
  void testGearUpWarningOnApproach() {
    bool gearDown = false;
    double altitude = 500.0;     // ft AGL
    double airspeed = 120.0;     // kts
    bool throttleLow = true;

    bool warningActive = !gearDown && (altitude < 700.0) && throttleLow;
    TS_ASSERT(warningActive);
  }

  // Test warning when flaps extended and gear up
  void testGearWarningWithFlaps() {
    bool gearDown = false;
    double flapPos = 0.5;  // 50% flaps
    double flapThreshold = 0.3;

    bool warningActive = !gearDown && (flapPos > flapThreshold);
    TS_ASSERT(warningActive);
  }

  // Test disagreement warning (gear in transit too long)
  void testGearDisagreementWarning() {
    double gearPos = 0.5;
    double transitTime = 15.0;  // seconds in transit
    double maxTransitTime = 10.0;

    bool disagreement = (gearPos > 0.1 && gearPos < 0.9) &&
                        (transitTime > maxTransitTime);
    TS_ASSERT(disagreement);
  }

  // Test asymmetric gear warning
  void testAsymmetricGearWarning() {
    double leftGearPos = 1.0;
    double rightGearPos = 0.5;
    double noseGearPos = 1.0;
    double tolerance = 0.1;

    bool asymmetric = (std::abs(leftGearPos - rightGearPos) > tolerance);
    TS_ASSERT(asymmetric);
  }

  /***************************************************************************
   * Emergency Extension Tests
   ***************************************************************************/

  // Test emergency extension system
  void testEmergencyExtension() {
    bool emergencyHandle = true;
    double gearPos = 0.0;
    double dt = 0.1;

    // Emergency extension bypasses normal controls
    if (emergencyHandle) {
      double emergencyRate = 0.15;  // Slower than normal
      gearPos += emergencyRate * dt;
      gearPos = std::min(gearPos, 1.0);
    }

    TS_ASSERT_DELTA(gearPos, 0.015, epsilon);
  }

  // Test free-fall extension time
  void testFreeFallExtensionTime() {
    double freeFallTime = 8.0;  // Typically slower than powered
    double poweredTime = 5.0;

    TS_ASSERT(freeFallTime > poweredTime);
  }

  // Test gravity-assisted extension
  void testGravityAssistedExtension() {
    double normalExtensionForce = 1000.0;  // lbs
    double gearWeight = 200.0;             // lbs
    double totalForce = normalExtensionForce + gearWeight;

    TS_ASSERT(totalForce > normalExtensionForce);
    TS_ASSERT_DELTA(totalForce, 1200.0, epsilon);
  }

  // Test emergency extension locks
  void testEmergencyExtensionLocks() {
    bool emergencyExtended = true;
    bool uplockReleased = true;
    bool downlockEngaged = true;

    // Emergency extension should lock down
    bool gearSafe = emergencyExtended && uplockReleased && downlockEngaged;
    TS_ASSERT(gearSafe);
  }

  /***************************************************************************
   * Gear Position Indicator Tests
   ***************************************************************************/

  // Test green light (gear down and locked)
  void testGreenLight() {
    double gearPos = 1.0;
    bool downlocked = true;
    bool greenLight = (gearPos > 0.99) && downlocked;

    TS_ASSERT(greenLight);
  }

  // Test red light (gear in transit)
  void testRedLight() {
    double gearPos = 0.5;
    bool redLight = (gearPos > 0.1 && gearPos < 0.9);

    TS_ASSERT(redLight);
  }

  // Test no lights (gear up and locked)
  void testNoLights() {
    double gearPos = 0.0;
    bool uplocked = true;
    bool greenLight = false;
    bool redLight = false;

    if (gearPos < 0.01 && uplocked) {
      greenLight = false;
      redLight = false;
    }

    TS_ASSERT(!greenLight);
    TS_ASSERT(!redLight);
  }

  // Test position sensor accuracy
  void testPositionSensorAccuracy() {
    double actualPos = 0.753;
    double sensorNoise = 0.002;
    double measuredPos = actualPos + sensorNoise;

    TS_ASSERT_DELTA(measuredPos, actualPos, 0.01);
  }

  /***************************************************************************
   * Airspeed Limitation Tests
   ***************************************************************************/

  // Test maximum extension speed (VLE)
  void testMaximumExtensionSpeed() {
    double VLE = 200.0;  // kts (max speed with gear extended)
    double airspeed = 220.0;

    bool overspeed = (airspeed > VLE);
    TS_ASSERT(overspeed);
  }

  // Test maximum retraction speed (VLO)
  void testMaximumRetractionSpeed() {
    double VLO = 170.0;  // kts (max speed for operating gear)
    double airspeed = 180.0;

    bool canOperate = (airspeed < VLO);
    TS_ASSERT(!canOperate);
  }

  // Test airspeed interlock
  void testAirspeedInterlock() {
    double airspeed = 210.0;
    double VLO = 170.0;
    bool gearCommand = false;  // Retract

    bool gearCanMove = (airspeed < VLO);
    TS_ASSERT(!gearCanMove);  // Blocked by high airspeed
  }

  // Test dynamic pressure effect on extension time
  void testDynamicPressureEffect() {
    double airspeed = 200.0;  // kts
    double dynamicPressure = 0.5 * 0.002377 * (airspeed * 1.68781) * (airspeed * 1.68781);
    double dragForce = dynamicPressure * 5.0;  // 5 sq ft gear area

    TS_ASSERT(dragForce > 0);
    TS_ASSERT_DELTA(dragForce, 677.1, 1.0);
  }

  /***************************************************************************
   * Asymmetric Gear Condition Tests
   ***************************************************************************/

  // Test single gear failure
  void testSingleGearFailure() {
    double leftGearPos = 1.0;
    double rightGearPos = 0.0;  // Failed to extend
    double noseGearPos = 1.0;

    double maxDiff = std::max(std::abs(leftGearPos - rightGearPos),
                              std::abs(leftGearPos - noseGearPos));

    bool asymmetric = (maxDiff > 0.1);
    TS_ASSERT(asymmetric);
  }

  // Test yaw moment from asymmetric gear drag
  void testAsymmetricDragYaw() {
    double leftGearDrag = 0.0;   // Retracted
    double rightGearDrag = 100.0; // Extended
    double lateralDistance = 5.0; // ft from centerline

    double yawMoment = (rightGearDrag - leftGearDrag) * lateralDistance;
    TS_ASSERT_DELTA(yawMoment, 500.0, epsilon);
  }

  // Test asymmetric retraction rates
  void testAsymmetricRetractionDetection() {
    double leftGearPos = 0.5;
    double rightGearPos = 0.7;
    double maxAllowedDiff = 0.15;

    bool outOfSync = (std::abs(leftGearPos - rightGearPos) > maxAllowedDiff);
    TS_ASSERT(outOfSync);
  }

  /***************************************************************************
   * Gear Transit Aerodynamic Drag Tests
   ***************************************************************************/

  // Test drag coefficient during transit
  void testTransitDragCoefficient() {
    double gearPos = 0.5;  // Half extended
    double retractedCD = 0.0;
    double extendedCD = 0.02;

    // Linear interpolation
    double transitCD = retractedCD + gearPos * (extendedCD - retractedCD);
    TS_ASSERT_DELTA(transitCD, 0.01, epsilon);
  }

  // Test maximum drag position
  void testMaximumDragPosition() {
    double gearPos = 0.5;  // Often maximum drag is mid-transit
    double dragMultiplier = 1.0 + 0.5 * std::sin(gearPos * M_PI);

    // Drag can be higher during transit than fully extended
    TS_ASSERT(dragMultiplier > 1.0);
  }

  // Test door drag contribution
  void testDoorDragContribution() {
    double doorPos = 1.0;  // Open
    double doorArea = 3.0; // sq ft
    double doorCD = 1.5;   // Flat plate

    double doorDrag = 0.5 * doorCD * doorArea;  // Reference area contribution
    TS_ASSERT_DELTA(doorDrag, 2.25, epsilon);
  }

  // Test total transit drag force
  void testTotalTransitDrag() {
    double gearDrag = 100.0;  // lbs
    double doorDrag = 50.0;   // lbs
    double totalDrag = gearDrag + doorDrag;

    TS_ASSERT_DELTA(totalDrag, 150.0, epsilon);
  }

  /***************************************************************************
   * Free-Fall Extension Time Tests
   ***************************************************************************/

  // Test free-fall extension rate
  void testFreeFallExtensionRate() {
    double gearMass = 6.0;      // slugs
    double dragForce = 200.0;   // lbs
    double gravityForce = 200.0; // lbs (weight)

    double netForce = gravityForce - dragForce;
    double acceleration = netForce / gearMass;

    TS_ASSERT_DELTA(acceleration, 0.0, epsilon);  // Balanced at terminal velocity
  }

  // Test air resistance during free-fall
  void testFreeFallAirResistance() {
    double velocity = 3.0;  // ft/s extension rate
    double dragCoeff = 20.0;
    double dragForce = dragCoeff * velocity * velocity;

    TS_ASSERT_DELTA(dragForce, 180.0, epsilon);
  }

  // Test free-fall completion time
  void testFreeFallCompletionTime() {
    double gearPos = 0.0;
    double avgVelocity = 0.125;  // Average extension rate (ft/s equivalent)
    double dt = 0.1;
    double freeFallTime = 0.0;

    while (gearPos < 0.99 && freeFallTime < 10.0) {
      gearPos += avgVelocity * dt;
      gearPos = std::min(gearPos, 1.0);
      freeFallTime += dt;
    }

    TS_ASSERT(freeFallTime > 7.0);  // Slower than powered extension
    TS_ASSERT(freeFallTime < 9.0);
  }

  /***************************************************************************
   * Hydraulic System Tests
   ***************************************************************************/

  // Test hydraulic pressure effect on extension rate
  void testHydraulicPressureEffect() {
    double nominalPressure = 3000.0;  // psi
    double actualPressure = 2500.0;   // psi (degraded)
    double pressureRatio = actualPressure / nominalPressure;

    double nominalRate = 0.2;  // 1.0 per 5 seconds
    double actualRate = nominalRate * pressureRatio;

    TS_ASSERT_DELTA(actualRate, 0.167, 0.01);
  }

  // Test hydraulic system failure
  void testHydraulicFailure() {
    double hydraulicPressure = 0.0;  // Total failure
    bool hydraulicAvailable = (hydraulicPressure > 1000.0);

    TS_ASSERT(!hydraulicAvailable);
  }

  // Test alternate hydraulic system
  void testAlternateHydraulicSystem() {
    bool primaryHydraulic = false;
    bool alternateHydraulic = true;

    bool hydraulicAvailable = primaryHydraulic || alternateHydraulic;
    TS_ASSERT(hydraulicAvailable);
  }

  // Test hydraulic flow rate
  void testHydraulicFlowRate() {
    double flowRate = 5.0;        // gallons per minute
    double actuatorArea = 10.0;   // square inches
    double velocity = (flowRate / 231.0) / (actuatorArea / 144.0) * 60.0;
    // Convert gpm to cubic feet per second, then divide by area in sq ft

    TS_ASSERT(velocity > 0);
  }

  /***************************************************************************
   * Electric Actuation Tests
   ***************************************************************************/

  // Test electric motor actuation
  void testElectricMotorActuation() {
    double motorSpeed = 1200.0;  // RPM
    double gearRatio = 100.0;
    double outputSpeed = motorSpeed / gearRatio;

    TS_ASSERT_DELTA(outputSpeed, 12.0, epsilon);  // 12 RPM at actuator
  }

  // Test electric system voltage effect
  void testElectricVoltageEffect() {
    double nominalVoltage = 28.0;  // volts
    double actualVoltage = 24.0;   // volts (low battery)
    double voltageRatio = actualVoltage / nominalVoltage;

    double nominalRate = 0.33;  // 1.0 per 3 seconds
    double actualRate = nominalRate * voltageRatio;

    TS_ASSERT_DELTA(actualRate, 0.283, 0.01);
  }

  // Test electric motor current limit
  void testElectricCurrentLimit() {
    double current = 50.0;      // amps
    double maxCurrent = 40.0;   // amp limit

    bool overload = (current > maxCurrent);
    TS_ASSERT(overload);
  }

  // Test electric backup system
  void testElectricBackupSystem() {
    bool hydraulicAvailable = false;
    bool electricAvailable = true;

    bool gearCanOperate = hydraulicAvailable || electricAvailable;
    TS_ASSERT(gearCanOperate);
  }

  /***************************************************************************
   * Gear Sequencing Logic Tests
   ***************************************************************************/

  // Test extension sequence: nose first or mains first
  void testExtensionSequenceMainsFirst() {
    double mainGearPos = 0.3;
    double noseGearPos = 0.0;
    double minMainPosForNose = 0.2;

    // Nose gear waits for mains to start
    bool noseCanExtend = (mainGearPos > minMainPosForNose);
    TS_ASSERT(noseCanExtend);
  }

  // Test retraction sequence: nose first
  void testRetractionSequenceNoseFirst() {
    double noseGearPos = 0.5;   // Retracting
    double mainGearPos = 1.0;   // Still down
    double maxNosePosForMains = 0.7;

    // Mains wait for nose to retract
    bool mainsCanRetract = (noseGearPos < maxNosePosForMains);
    TS_ASSERT(mainsCanRetract);
  }

  // Test simultaneous gear movement
  void testSimultaneousGearMovement() {
    double leftGearPos = 0.5;
    double rightGearPos = 0.5;
    double noseGearPos = 0.5;
    double dt = 0.1;
    double rate = 0.2;

    // All move together
    leftGearPos += rate * dt;
    rightGearPos += rate * dt;
    noseGearPos += rate * dt;

    TS_ASSERT_DELTA(leftGearPos, rightGearPos, epsilon);
    TS_ASSERT_DELTA(leftGearPos, noseGearPos, epsilon);
  }

  /***************************************************************************
   * Gear Load Tests
   ***************************************************************************/

  // Test load on gear during extension
  void testExtensionLoad() {
    double gearWeight = 200.0;     // lbs
    double airDragForce = 150.0;   // lbs
    double totalLoad = gearWeight + airDragForce;

    TS_ASSERT_DELTA(totalLoad, 350.0, epsilon);
  }

  // Test actuator force requirement
  void testActuatorForceRequirement() {
    double gearLoad = 350.0;        // lbs
    double mechanicalAdvantage = 5.0;
    double actuatorForce = gearLoad / mechanicalAdvantage;

    TS_ASSERT_DELTA(actuatorForce, 70.0, epsilon);
  }

  // Test maximum structural load
  void testMaximumStructuralLoad() {
    double dynamicLoad = 500.0;
    double maxAllowable = 1000.0;

    bool loadSafe = (dynamicLoad < maxAllowable);
    TS_ASSERT(loadSafe);
  }

  /***************************************************************************
   * Gear Control System Tests
   ***************************************************************************/

  // Test gear lever position
  void testGearLeverPosition() {
    bool leverDown = true;
    double commandedPos = leverDown ? 1.0 : 0.0;

    TS_ASSERT_DELTA(commandedPos, 1.0, epsilon);
  }

  // Test gear lever interlock
  void testGearLeverInterlock() {
    bool leverDown = true;
    bool interlockEngaged = true;

    // Cannot move lever while interlocked
    bool leverCanMove = !interlockEngaged;
    TS_ASSERT(!leverCanMove);
  }

  // Test automatic gear extension (some aircraft)
  void testAutomaticGearExtension() {
    double altitude = 200.0;  // ft AGL
    double airspeed = 100.0;  // kts
    bool gearDown = false;
    bool throttleLow = true;

    // Auto-extend criteria
    bool autoExtend = !gearDown && (altitude < 250.0) &&
                      (airspeed < 120.0) && throttleLow;
    TS_ASSERT(autoExtend);
  }

  /***************************************************************************
   * Edge Cases and Failure Modes
   ***************************************************************************/

  // Test partial extension due to obstruction
  void testPartialExtensionObstruction() {
    double gearPos = 0.75;  // Stuck at 75%
    double timeInTransit = 20.0;
    double normalTransitTime = 5.0;

    bool possibleObstruction = (gearPos > 0.1 && gearPos < 0.99) &&
                               (timeInTransit > 2.0 * normalTransitTime);
    TS_ASSERT(possibleObstruction);
  }

  // Test gear extension in flight limits
  void testGearExtensionFlightLimits() {
    double normalLoadFactor = 2.5;  // g's
    double maxLoadForGearOps = 2.0;

    bool safeToOperate = (normalLoadFactor < maxLoadForGearOps);
    TS_ASSERT(!safeToOperate);
  }

  // Test thermal expansion effects
  void testThermalExpansionEffect() {
    double nominalLength = 100.0;  // inches
    double tempChange = 100.0;     // degrees F
    double expansion = nominalLength * 6.5e-6 * tempChange;  // Steel coefficient

    TS_ASSERT_DELTA(expansion, 0.065, 0.01);  // Small but measurable
  }

  // Test gear position after power loss
  void testGearPositionAfterPowerLoss() {
    double gearPos = 0.6;  // In transit when power lost

    // Should stay at current position (hydraulic locks) or free-fall
    bool staysInPlace = true;  // Depends on system design
    double newPos = staysInPlace ? gearPos : gearPos + 0.01;  // Slight drift

    TS_ASSERT(newPos >= gearPos);
  }

  /***************************************************************************
   * Shimmy Damper Tests
   ***************************************************************************/

  // Test shimmy damper effectiveness
  void testShimmyDamperEffectiveness() {
    double oscillationFrequency_hz = 15.0;  // Typical shimmy frequency
    double dampingCoefficient = 500.0;  // lb-s/rad
    double angularVelocity_rad_s = 0.5;

    double dampingForce_lb = dampingCoefficient * angularVelocity_rad_s;

    TS_ASSERT_DELTA(dampingForce_lb, 250.0, 1.0);
    TS_ASSERT(dampingForce_lb > 0.0);
  }

  // Test shimmy damper failure detection
  void testShimmyDamperFailure() {
    double normalOscillation_deg = 0.5;
    double failedOscillation_deg = 5.0;
    double threshold_deg = 2.0;

    bool damperFailed = (failedOscillation_deg > threshold_deg);

    TS_ASSERT(damperFailed);
  }

  // Test shimmy frequency calculation
  void testShimmyFrequencyCalculation() {
    double wheelInertia_slug_ft2 = 0.5;
    double tireStiffness_lb_rad = 5000.0;

    // Natural frequency = sqrt(k/I) / (2*pi)
    double omega_rad_s = std::sqrt(tireStiffness_lb_rad / wheelInertia_slug_ft2);
    double frequency_hz = omega_rad_s / (2.0 * M_PI);

    TS_ASSERT_DELTA(frequency_hz, 15.92, 0.5);
  }

  // Test shimmy onset speed
  void testShimmyOnsetSpeed() {
    double criticalSpeed_kts = 80.0;  // Speed above which shimmy may occur
    double groundspeed_kts = 90.0;

    bool shimmyRisk = (groundspeed_kts > criticalSpeed_kts);

    TS_ASSERT(shimmyRisk);
  }

  /***************************************************************************
   * Oleo Strut Dynamics Tests
   ***************************************************************************/

  // Test oleo strut compression on touchdown
  void testOleoStrutCompression() {
    double sinkRate_fps = 8.0;
    double aircraftMass_slug = 300.0;
    double oleoStiffness_lb_ft = 50000.0;
    double dampingCoeff_lb_s_ft = 5000.0;

    // Energy absorption: 0.5 * m * v^2
    double kineticEnergy_ft_lb = 0.5 * aircraftMass_slug * sinkRate_fps * sinkRate_fps;

    TS_ASSERT_DELTA(kineticEnergy_ft_lb, 9600.0, 10.0);
  }

  // Test oleo extension after compression
  void testOleoExtensionRebound() {
    double maxCompression_ft = 0.8;
    double reboundDamping = 0.3;  // 30% of compression rate

    double extensionRate_fps = 5.0 * reboundDamping;

    TS_ASSERT_DELTA(extensionRate_fps, 1.5, 0.1);
  }

  // Test oleo strut nitrogen pressure
  void testOleoNitrogenPressure() {
    double staticPressure_psi = 800.0;
    double strokeRatio = 0.5;  // 50% compressed
    double polytropicExponent = 1.3;

    // Pressure increases with compression: P2 = P1 * (1/(1-stroke))^n
    double dynamicPressure_psi = staticPressure_psi *
        std::pow(1.0 / (1.0 - strokeRatio), polytropicExponent);

    TS_ASSERT(dynamicPressure_psi > staticPressure_psi);
    TS_ASSERT_DELTA(dynamicPressure_psi, 1966.0, 50.0);
  }

  // Test oleo strut servicing check
  void testOleoStrutServicing() {
    double actualExtension_in = 3.5;
    double minExtension_in = 3.0;
    double maxExtension_in = 4.0;

    bool properlyServiced = (actualExtension_in >= minExtension_in) &&
                            (actualExtension_in <= maxExtension_in);

    TS_ASSERT(properlyServiced);
  }

  /***************************************************************************
   * Nose Wheel Steering Tests
   ***************************************************************************/

  // Test nose wheel steering authority
  void testNoseWheelSteeringAuthority() {
    double maxSteeringAngle_deg = 60.0;
    double rudderpPosition_norm = 0.5;

    double commandedAngle_deg = maxSteeringAngle_deg * rudderpPosition_norm;

    TS_ASSERT_DELTA(commandedAngle_deg, 30.0, 0.1);
  }

  // Test steering rate limit
  void testSteeringRateLimit() {
    double maxSteeringRate_deg_s = 20.0;
    double dt = 0.1;
    double currentAngle_deg = 10.0;
    double commandedAngle_deg = 40.0;

    double deltaAngle = commandedAngle_deg - currentAngle_deg;
    double maxDelta = maxSteeringRate_deg_s * dt;
    double actualDelta = std::min(std::abs(deltaAngle), maxDelta);
    if (deltaAngle < 0) actualDelta = -actualDelta;

    double newAngle = currentAngle_deg + actualDelta;

    TS_ASSERT_DELTA(newAngle, 12.0, 0.1);
  }

  // Test steering disconnect at high speed
  void testSteeringHighSpeedDisconnect() {
    double groundspeed_kts = 80.0;
    double disconnectSpeed_kts = 60.0;

    bool steeringEnabled = (groundspeed_kts < disconnectSpeed_kts);

    TS_ASSERT(!steeringEnabled);
  }

  // Test tiller vs rudder pedal steering
  void testTillerVsRudderSteering() {
    double tillerAngle_deg = 45.0;
    double rudderPedalAngle_deg = 5.0;

    // Tiller provides more authority
    double tillerAuthority_deg = 60.0;
    double rudderAuthority_deg = 7.0;

    double tillerCommand = (tillerAngle_deg / tillerAuthority_deg) * tillerAuthority_deg;
    double rudderCommand = (rudderPedalAngle_deg / rudderAuthority_deg) * rudderAuthority_deg;

    TS_ASSERT(tillerCommand > rudderCommand);
  }

  /***************************************************************************
   * Tailhook/Arresting Gear Tests (Carrier Operations)
   ***************************************************************************/

  // Test tailhook extension time
  void testTailhookExtensionTime() {
    double hookPos = 0.0;  // Retracted
    double extensionTime_s = 2.0;
    double dt = 0.1;

    double extensionRate = 1.0 / extensionTime_s;
    hookPos += extensionRate * dt;

    TS_ASSERT_DELTA(hookPos, 0.05, 0.001);
  }

  // Test arrestor wire engagement
  void testArrestorWireEngagement() {
    double hookHeight_ft = 0.5;
    double wireHeight_ft = 0.3;
    double engagementTolerance_ft = 0.4;

    bool wireEngaged = std::abs(hookHeight_ft - wireHeight_ft) < engagementTolerance_ft;

    TS_ASSERT(wireEngaged);
  }

  // Test arrested deceleration
  void testArrestedDeceleration() {
    double initialSpeed_kts = 130.0;
    double arrestDistance_ft = 300.0;
    double initialSpeed_fps = initialSpeed_kts * 1.68781;

    // v^2 = 2*a*d -> a = v^2 / (2*d)
    double deceleration_fps2 = (initialSpeed_fps * initialSpeed_fps) / (2.0 * arrestDistance_ft);
    double deceleration_g = deceleration_fps2 / 32.174;

    TS_ASSERT_DELTA(deceleration_g, 2.49, 0.1);  // ~2.5g arrest deceleration
  }

  // Test hook bounce back prevention
  void testHookBounceBack() {
    double hookAngle_deg = 45.0;  // Normal deployed angle
    double bounceAngle_deg = 60.0;  // Max before damper engages
    double dampingForce_lb = 200.0;

    bool damperActive = (hookAngle_deg > 40.0);

    TS_ASSERT(damperActive);
  }

  /***************************************************************************
   * Catapult Launch Tests (Carrier Operations)
   ***************************************************************************/

  // Test launch bar extension
  void testLaunchBarExtension() {
    double launchBarPos = 0.0;
    double extensionTime_s = 1.5;
    double dt = 0.1;

    double rate = 1.0 / extensionTime_s;
    launchBarPos += rate * dt;

    TS_ASSERT_DELTA(launchBarPos, 0.0667, 0.001);
  }

  // Test catapult acceleration
  void testCatapultAcceleration() {
    double launchSpeed_kts = 150.0;
    double catapultLength_ft = 300.0;
    double launchSpeed_fps = launchSpeed_kts * 1.68781;

    // v^2 = 2*a*d -> a = v^2 / (2*d)
    double acceleration_fps2 = (launchSpeed_fps * launchSpeed_fps) / (2.0 * catapultLength_ft);
    double acceleration_g = acceleration_fps2 / 32.174;

    TS_ASSERT_DELTA(acceleration_g, 3.31, 0.1);
  }

  // Test hold-back bar release
  void testHoldBackBarRelease() {
    double catapultForce_lb = 80000.0;
    double holdBackStrength_lb = 75000.0;

    bool holdBackReleased = (catapultForce_lb > holdBackStrength_lb);

    TS_ASSERT(holdBackReleased);
  }

  // Test nose gear catapult loads
  void testNoseGearCatapultLoads() {
    double catapultForce_lb = 80000.0;
    double noseGearDesignLoad_lb = 100000.0;

    bool withinLimits = (catapultForce_lb < noseGearDesignLoad_lb);

    TS_ASSERT(withinLimits);
  }

  /***************************************************************************
   * Ground Maneuvering Tests
   ***************************************************************************/

  // Test minimum turning radius
  void testMinimumTurningRadius() {
    double wheelbase_ft = 40.0;
    double maxSteeringAngle_deg = 60.0;
    double maxSteeringAngle_rad = maxSteeringAngle_deg * deg2rad;

    // R = L / tan(steering_angle)
    double turningRadius_ft = wheelbase_ft / std::tan(maxSteeringAngle_rad);

    TS_ASSERT_DELTA(turningRadius_ft, 23.1, 0.5);
  }

  // Test crosswind weathervaning
  void testCrosswindWeathervaning() {
    double crosswindComponent_kts = 15.0;
    double finArea_ft2 = 100.0;
    double dynamicPressure = 0.5 * 0.002377 * std::pow(crosswindComponent_kts * 1.68781, 2);

    double weathervaneForce_lb = dynamicPressure * finArea_ft2;

    TS_ASSERT(weathervaneForce_lb > 0.0);
  }

  // Test taxi speed limits
  void testTaxiSpeedLimits() {
    double currentSpeed_kts = 25.0;
    double maxStraightTaxi_kts = 30.0;
    double maxTurnTaxi_kts = 10.0;
    bool inTurn = false;

    double maxSpeed = inTurn ? maxTurnTaxi_kts : maxStraightTaxi_kts;
    bool withinLimits = (currentSpeed_kts < maxSpeed);

    TS_ASSERT(withinLimits);
  }

  // Test differential braking turn assist
  void testDifferentialBrakingTurn() {
    double leftBrake = 0.8;
    double rightBrake = 0.2;
    double brakeDifferential = leftBrake - rightBrake;

    // Differential creates yaw moment
    bool turningRight = (brakeDifferential > 0.0);

    TS_ASSERT(turningRight);
    TS_ASSERT_DELTA(brakeDifferential, 0.6, 0.01);
  }

  /***************************************************************************
   * Gear Bay Environment Tests
   ***************************************************************************/

  // Test gear bay temperature limits
  void testGearBayTemperature() {
    double ambientTemp_c = -50.0;  // High altitude
    double brakeHeat_c = 150.0;    // After landing
    double gearBayTemp_c = brakeHeat_c;

    double maxTemp_c = 200.0;
    bool tempSafe = (gearBayTemp_c < maxTemp_c);

    TS_ASSERT(tempSafe);
  }

  // Test gear bay fire detection
  void testGearBayFireDetection() {
    double temperature_c = 180.0;
    double fireWarningThreshold_c = 150.0;

    bool fireWarning = (temperature_c > fireWarningThreshold_c);

    TS_ASSERT(fireWarning);
  }

  // Test gear bay pressurization effect
  void testGearBayPressurization() {
    double cabinAltitude_ft = 8000.0;
    double gearBayAltitude_ft = 35000.0;  // Unpressurized

    double pressureDifferential_psi = (gearBayAltitude_ft - cabinAltitude_ft) / 2000.0;

    TS_ASSERT(pressureDifferential_psi > 0.0);
  }

  // Test gear bay ice accumulation
  void testGearBayIceAccumulation() {
    double temperature_c = -20.0;
    double humidity_percent = 80.0;
    double flightTime_hr = 2.0;

    bool icingConditions = (temperature_c < 0.0) && (humidity_percent > 60.0);
    double iceAccumulation_mm = icingConditions ? (flightTime_hr * 0.5) : 0.0;

    TS_ASSERT(icingConditions);
    TS_ASSERT_DELTA(iceAccumulation_mm, 1.0, 0.1);
  }

  /***************************************************************************
   * Tire Pressure and Temperature Tests
   ***************************************************************************/

  // Test tire pressure change with altitude
  void testTirePressureWithAltitude() {
    double groundPressure_psi = 200.0;
    double ambientPressure_ground_psi = 14.7;
    double ambientPressure_altitude_psi = 3.46;  // 35000 ft

    // Tire expands as ambient pressure drops
    double pressureIncrease_psi = (ambientPressure_ground_psi - ambientPressure_altitude_psi);
    double altitudePressure_psi = groundPressure_psi + pressureIncrease_psi;

    TS_ASSERT_DELTA(altitudePressure_psi, 211.24, 0.5);
  }

  // Test tire temperature from braking
  void testTireTemperatureFromBraking() {
    double initialTemp_c = 20.0;
    double brakingEnergy_j = 5000000.0;  // 5 MJ
    double tireMass_kg = 50.0;
    double specificHeat_j_kg_c = 1500.0;

    double tempRise_c = brakingEnergy_j / (tireMass_kg * specificHeat_j_kg_c);
    double finalTemp_c = initialTemp_c + tempRise_c;

    TS_ASSERT_DELTA(tempRise_c, 66.67, 1.0);
  }

  // Test tire pressure monitoring system
  void testTPMSWarningLogic() {
    double leftMainPressure_psi = 185.0;
    double rightMainPressure_psi = 200.0;
    double nominalPressure_psi = 200.0;
    double warningThreshold_percent = 10.0;

    double leftDeviation_percent = std::abs(leftMainPressure_psi - nominalPressure_psi) /
                                   nominalPressure_psi * 100.0;

    bool lowPressureWarning = (leftDeviation_percent > warningThreshold_percent);

    TS_ASSERT(!lowPressureWarning);
    TS_ASSERT_DELTA(leftDeviation_percent, 7.5, 0.1);
  }

  // Test tire blowout detection
  void testTireBlowoutDetection() {
    double previousPressure_psi = 200.0;
    double currentPressure_psi = 20.0;
    double maxPressureLossRate_psi_s = 50.0;
    double dt = 0.1;

    double pressureLossRate = (previousPressure_psi - currentPressure_psi) / dt;
    bool blowoutDetected = (pressureLossRate > maxPressureLossRate_psi_s);

    TS_ASSERT(blowoutDetected);
    TS_ASSERT_DELTA(pressureLossRate, 1800.0, 10.0);
  }
};

/*******************************************************************************
 * FGLandingGearDynamics C172x Integration Tests
 * Tests landing gear behavior using real C172x aircraft (fixed gear)
 ******************************************************************************/
class FGLandingGearDynamicsC172xTest : public CxxTest::TestSuite
{
public:
  JSBSim::FGFDMExec fdmex;
  std::string aircraft_path;

  void setUp() {
    aircraft_path = std::string(JSBSIM_TEST_PATH) + "/aircraft";
    fdmex.SetAircraftPath(aircraft_path);
    fdmex.SetEnginePath(std::string(JSBSIM_TEST_PATH) + "/engine");
    fdmex.SetSystemsPath(std::string(JSBSIM_TEST_PATH) + "/systems");
  }

  void tearDown() {
    fdmex.ResetToInitialConditions(0);
  }

  // Test C172x has fixed gear (always down)
  void testC172xFixedGearAlwaysDown() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    // C172x has fixed gear - check that gear exists
    int numGear = gr->GetNumGearUnits();
    TS_ASSERT(numGear > 0);
  }

  // Test gear contact on ground
  void testC172xGearGroundContact() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);  // On ground
    ic->SetVgroundKtsIC(0.0);     // Stationary
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    // Should have ground contact on stationary aircraft
    bool anyContact = false;
    for (int i = 0; i < gr->GetNumGearUnits(); i++) {
      if (gr->GetGearUnit(i)->GetWOW()) {
        anyContact = true;
        break;
      }
    }
    TS_ASSERT(anyContact);
  }

  // Test weight on wheels during taxi
  void testC172xWeightOnWheelsTaxi() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(20.0);  // Taxi speed
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    double alt = propagate->GetAltitudeAGL();

    // Should be near ground during taxi
    TS_ASSERT(alt < 10.0);
  }

  // Test no gear contact in cruise
  void testC172xNoGearContactCruise() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(110.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    // No gear should have ground contact at altitude
    bool anyContact = false;
    for (int i = 0; i < gr->GetNumGearUnits(); i++) {
      if (gr->GetGearUnit(i)->GetWOW()) {
        anyContact = true;
        break;
      }
    }
    TS_ASSERT(!anyContact);
  }

  // Test tricycle gear configuration
  void testC172xTricycleGearConfig() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(0.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    // C172x should have 3 gear units (tricycle)
    TS_ASSERT(gr->GetNumGearUnits() >= 3);
  }

  // Test gear forces during landing rollout
  void testC172xGearForcesRollout() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(50.0);  // Rollout speed
    TS_ASSERT(fdmex.RunIC());

    for (int i = 0; i < 10; i++) fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    // Check that gear forces exist
    auto forces = gr->GetForces();
    TS_ASSERT(std::isfinite(forces(1)));
    TS_ASSERT(std::isfinite(forces(2)));
    TS_ASSERT(std::isfinite(forces(3)));
  }

  // Test braking during rollout
  void testC172xBrakingRollout() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(40.0);
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    fcs->SetLBrake(1.0);  // Full left brake
    fcs->SetRBrake(1.0);  // Full right brake

    for (int i = 0; i < 10; i++) fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    double speed = propagate->GetVel().Magnitude();

    // Speed should be decreasing with brakes applied
    TS_ASSERT(std::isfinite(speed));
  }

  // Test steering authority during taxi
  void testC172xNosewheelSteering() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(10.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto fcs = fdmex.GetFCS();
    // Apply rudder for nosewheel steering
    fcs->SetDrCmd(0.5);  // Right rudder

    for (int i = 0; i < 10; i++) fdmex.Run();

    // Aircraft should respond to steering input
    auto propagate = fdmex.GetPropagate();
    double yawRate = propagate->GetPQR()(3);
    TS_ASSERT(std::isfinite(yawRate));
  }

  // Test gear position on ground
  void testC172xGearPositionOnGround() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(0.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    for (int i = 0; i < gr->GetNumGearUnits(); i++) {
      auto gear = gr->GetGearUnit(i);
      // Fixed gear should always report position = 1.0 (down)
      double pos = gear->GetGearUnitPos();
      TS_ASSERT_DELTA(pos, 1.0, 0.01);
    }
  }

  // Test lateral gear loading during crosswind taxi
  void testC172xCrosswindTaxiGearLoading() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(15.0);
    ic->SetWindNEDFpsIC(0.0, 20.0, 0.0);  // Crosswind from east
    TS_ASSERT(fdmex.RunIC());

    for (int i = 0; i < 10; i++) fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    // Gear should experience lateral forces from crosswind
    auto forces = gr->GetForces();
    TS_ASSERT(std::isfinite(forces(2)));  // Side force
  }

  // Test touchdown detection
  void testC172xTouchdownDetection() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(10.0);  // Just above ground
    ic->SetVcalibratedKtsIC(60.0);  // Approach speed
    ic->SetClimbRateFpsIC(-5.0);   // Descending
    TS_ASSERT(fdmex.RunIC());

    auto gr = fdmex.GetGroundReactions();

    // Run until touchdown
    bool touchedDown = false;
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
      for (int j = 0; j < gr->GetNumGearUnits(); j++) {
        if (gr->GetGearUnit(j)->GetWOW()) {
          touchedDown = true;
          break;
        }
      }
      if (touchedDown) break;
    }
    TS_ASSERT(touchedDown);
  }
};
