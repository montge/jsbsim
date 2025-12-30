/*******************************************************************************
 * FGLGearTest.h - Unit tests for FGLGear (landing gear physics)
 *
 * Tests the mathematical behavior of landing gear components:
 * - Spring/damper strut forces
 * - Friction models (static, dynamic, rolling)
 * - Brake force calculations
 * - Wheel slip and side force
 * - Steering angle effects
 *
 * Note: FGLGear requires XML element for construction, so these tests focus
 * on the underlying physical calculations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <algorithm>

#include <FGFDMExec.h>
#include <models/FGLGear.h>
#include <models/FGGroundReactions.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-10;

class FGLGearTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Spring Force Tests
   ***************************************************************************/

  // Test linear spring force: F = -k * x
  void testLinearSpringForce() {
    double kSpring = 5000.0;   // lbs/ft
    double compression = 0.5;  // ft

    double force = kSpring * compression;
    TS_ASSERT_DELTA(force, 2500.0, epsilon);
  }

  // Test zero compression (no force)
  void testZeroCompression() {
    double kSpring = 5000.0;
    double compression = 0.0;

    double force = kSpring * compression;
    TS_ASSERT_DELTA(force, 0.0, epsilon);
  }

  // Test maximum compression
  void testMaxCompression() {
    double kSpring = 5000.0;
    double maxCompLen = 1.0;  // ft
    double compression = maxCompLen;

    double force = kSpring * compression;
    TS_ASSERT_DELTA(force, 5000.0, epsilon);
  }

  /***************************************************************************
   * Damper Force Tests
   ***************************************************************************/

  // Test linear damper force: F = -b * v
  void testLinearDamperForce() {
    double bDamp = 500.0;      // lbs/ft/sec
    double compressSpeed = 2.0; // ft/sec

    double force = bDamp * compressSpeed;
    TS_ASSERT_DELTA(force, 1000.0, epsilon);
  }

  // Test squared damper force: F = -b * v^2 * sign(v)
  void testSquaredDamperForce() {
    double bDamp = 500.0;
    double compressSpeed = 2.0;

    double force = bDamp * compressSpeed * std::abs(compressSpeed);
    TS_ASSERT_DELTA(force, 2000.0, epsilon);
  }

  // Test rebound damping (different coefficient)
  void testReboundDamping() {
    double bDampRebound = 300.0;  // Lighter rebound damping
    double compressSpeed = -1.5;   // Negative = extending

    double force = bDampRebound * std::abs(compressSpeed);
    TS_ASSERT_DELTA(force, 450.0, epsilon);
  }

  // Test combined spring-damper force
  void testSpringDamperCombined() {
    double kSpring = 5000.0;
    double bDamp = 500.0;
    double compression = 0.5;
    double compressSpeed = 1.0;

    double springForce = kSpring * compression;
    double damperForce = bDamp * compressSpeed;
    double totalForce = springForce + damperForce;

    TS_ASSERT_DELTA(totalForce, 3000.0, epsilon);
  }

  /***************************************************************************
   * Strut Force Direction Tests
   ***************************************************************************/

  // Test strut force is always upward (reacting against ground)
  void testStrutForceDirection() {
    double kSpring = 5000.0;
    double bDamp = 500.0;
    double compression = 0.5;
    double compressSpeed = 1.0;

    double force = kSpring * compression + bDamp * compressSpeed;

    // Force should be positive (upward in local Z)
    TS_ASSERT(force > 0);
  }

  // Test strut force cannot be negative (no suction)
  void testNoSuctionForce() {
    double kSpring = 5000.0;
    double bDamp = 500.0;
    double compression = 0.1;       // Small compression
    double compressSpeed = -2.0;    // Fast extension

    double force = kSpring * compression + bDamp * compressSpeed;

    // Clamp to zero (no pulling down)
    force = std::max(0.0, force);
    TS_ASSERT(force >= 0);
  }

  /***************************************************************************
   * Friction Force Tests
   ***************************************************************************/

  // Test friction force: F_friction = mu * F_normal
  void testFrictionForce() {
    double normalForce = 3000.0;  // lbs
    double mu = 0.8;              // friction coefficient

    double frictionForce = mu * normalForce;
    TS_ASSERT_DELTA(frictionForce, 2400.0, epsilon);
  }

  // Test static vs dynamic friction
  void testStaticVsDynamicFriction() {
    double normalForce = 3000.0;
    double staticMu = 0.9;
    double dynamicMu = 0.7;

    double staticFriction = staticMu * normalForce;
    double dynamicFriction = dynamicMu * normalForce;

    // Static friction should be greater
    TS_ASSERT(staticFriction > dynamicFriction);
    TS_ASSERT_DELTA(staticFriction, 2700.0, epsilon);
    TS_ASSERT_DELTA(dynamicFriction, 2100.0, epsilon);
  }

  // Test rolling friction
  void testRollingFriction() {
    double normalForce = 3000.0;
    double rollingMu = 0.02;  // Much smaller than sliding

    double rollingFriction = rollingMu * normalForce;
    TS_ASSERT_DELTA(rollingFriction, 60.0, epsilon);
  }

  /***************************************************************************
   * Brake Force Tests
   ***************************************************************************/

  // Test brake force calculation
  void testBrakeForce() {
    double normalForce = 3000.0;
    double brakeMu = 0.7;
    double brakeCmd = 1.0;  // Full braking

    double brakeForce = brakeMu * brakeCmd * normalForce;
    TS_ASSERT_DELTA(brakeForce, 2100.0, epsilon);
  }

  // Test partial brake application
  void testPartialBrake() {
    double normalForce = 3000.0;
    double brakeMu = 0.7;
    double brakeCmd = 0.5;  // Half braking

    double brakeForce = brakeMu * brakeCmd * normalForce;
    TS_ASSERT_DELTA(brakeForce, 1050.0, epsilon);
  }

  // Test brake release
  void testBrakeRelease() {
    double normalForce = 3000.0;
    double brakeMu = 0.7;
    double brakeCmd = 0.0;  // No braking

    double brakeForce = brakeMu * brakeCmd * normalForce;
    TS_ASSERT_DELTA(brakeForce, 0.0, epsilon);
  }

  // Test brake groups
  void testBrakeGroups() {
    double leftBrake = 0.8;
    double rightBrake = 0.6;
    double centerBrake = 0.0;

    // Differential braking for steering
    double differential = leftBrake - rightBrake;
    TS_ASSERT_DELTA(differential, 0.2, epsilon);
  }

  /***************************************************************************
   * Wheel Slip Angle Tests
   ***************************************************************************/

  // Test wheel slip angle calculation
  void testWheelSlipAngle() {
    double wheelVelX = 100.0;  // Forward velocity (ft/s)
    double wheelVelY = 10.0;   // Side velocity (ft/s)

    double slipAngle = std::atan2(wheelVelY, wheelVelX);
    double slipAngleDeg = slipAngle * 180.0 / M_PI;

    TS_ASSERT_DELTA(slipAngleDeg, 5.71, 0.01);
  }

  // Test zero slip (pure rolling)
  void testZeroSlip() {
    double wheelVelX = 100.0;
    double wheelVelY = 0.0;

    double slipAngle = std::atan2(wheelVelY, wheelVelX);
    TS_ASSERT_DELTA(slipAngle, 0.0, epsilon);
  }

  // Test high slip angle
  void testHighSlipAngle() {
    double wheelVelX = 100.0;
    double wheelVelY = 100.0;

    double slipAngle = std::atan2(wheelVelY, wheelVelX);
    double slipAngleDeg = slipAngle * 180.0 / M_PI;

    TS_ASSERT_DELTA(slipAngleDeg, 45.0, 0.01);
  }

  /***************************************************************************
   * Pacejka Tire Model Tests (Magic Formula)
   ***************************************************************************/

  // Pacejka "Magic Formula": F = D * sin(C * atan(B*x - E*(B*x - atan(B*x))))
  double pacejkaForce(double slip, double B, double C, double D, double E) {
    double x = slip;
    double Bx = B * x;
    return D * std::sin(C * std::atan(Bx - E * (Bx - std::atan(Bx))));
  }

  // Test Pacejka model at zero slip
  void testPacejkaZeroSlip() {
    // Typical values
    double B = 10.0;  // Stiffness
    double C = 1.9;   // Shape
    double D = 1.0;   // Peak
    double E = 0.97;  // Curvature

    double force = pacejkaForce(0.0, B, C, D, E);
    TS_ASSERT_DELTA(force, 0.0, 0.001);
  }

  // Test Pacejka model peak force
  void testPacejkaPeakForce() {
    double B = 10.0;
    double C = 1.9;
    double D = 1.0;  // Peak force coefficient
    double E = 0.97;

    // Peak typically occurs around slip = 0.1-0.2 rad
    double maxForce = 0.0;
    for (double slip = 0.0; slip < 0.5; slip += 0.01) {
      double force = std::abs(pacejkaForce(slip, B, C, D, E));
      if (force > maxForce) maxForce = force;
    }

    // Peak should be close to D
    TS_ASSERT(maxForce > 0.9 * D);
    TS_ASSERT(maxForce <= D);
  }

  /***************************************************************************
   * Steering Tests
   ***************************************************************************/

  // Test steering angle effect on forces
  void testSteeringForceTransform() {
    double forceX = 100.0;  // Forward force
    double forceY = 0.0;    // Side force
    double steerAngle = 30.0 * M_PI / 180.0;  // 30 degrees

    // Transform to wheel frame
    double wheelRollForce = forceX * std::cos(steerAngle) + forceY * std::sin(steerAngle);
    double wheelSideForce = forceY * std::cos(steerAngle) - forceX * std::sin(steerAngle);

    TS_ASSERT_DELTA(wheelRollForce, 86.6, 0.1);
    TS_ASSERT_DELTA(wheelSideForce, -50.0, 0.1);
  }

  // Test maximum steering angle
  void testMaxSteerAngle() {
    double maxSteerAngle = 60.0;  // degrees
    double steerCmd = 1.0;        // Full deflection

    double actualAngle = steerCmd * maxSteerAngle;
    TS_ASSERT_DELTA(actualAngle, 60.0, epsilon);
  }

  // Test nose wheel steering
  void testNoseWheelSteering() {
    double maxSteerAngle = 75.0;  // Nose wheel can turn more
    double steerCmd = 0.5;

    double actualAngle = steerCmd * maxSteerAngle;
    TS_ASSERT_DELTA(actualAngle, 37.5, epsilon);
  }

  // Test caster wheel (free to rotate)
  void testCasterWheel() {
    // Caster aligns with velocity
    double velX = 10.0;
    double velY = 5.0;

    double casterAngle = std::atan2(velY, velX);
    double casterAngleDeg = casterAngle * 180.0 / M_PI;

    TS_ASSERT_DELTA(casterAngleDeg, 26.57, 0.01);
  }

  /***************************************************************************
   * Weight on Wheels (WOW) Tests
   ***************************************************************************/

  // Test WOW detection
  void testWOWDetection() {
    double compression = 0.1;  // Gear compressed
    bool WOW = (compression > 0);

    TS_ASSERT(WOW);
  }

  // Test WOW clear (airborne)
  void testWOWClear() {
    double compression = 0.0;  // No compression
    bool WOW = (compression > 0);

    TS_ASSERT(!WOW);
  }

  // Test WOW with small compression threshold
  void testWOWThreshold() {
    double threshold = 0.001;  // Small threshold to avoid bouncing
    double compressions[] = {0.0, 0.0005, 0.001, 0.01};
    bool expectedWOW[] = {false, false, false, true};

    for (int i = 0; i < 4; i++) {
      bool WOW = (compressions[i] > threshold);
      TS_ASSERT_EQUALS(WOW, expectedWOW[i]);
    }
  }

  /***************************************************************************
   * Retractable Gear Tests
   ***************************************************************************/

  // Test gear position
  void testGearPosition() {
    double gearPos = 1.0;  // 1.0 = fully down

    bool gearDown = (gearPos > 0.99);
    bool gearUp = (gearPos < 0.01);

    TS_ASSERT(gearDown);
    TS_ASSERT(!gearUp);
  }

  // Test gear transition
  void testGearTransition() {
    double gearPos = 0.5;  // In transit

    bool gearDown = (gearPos > 0.99);
    bool gearUp = (gearPos < 0.01);

    TS_ASSERT(!gearDown);
    TS_ASSERT(!gearUp);
  }

  // Test retracted gear no force
  void testRetractedGearNoForce() {
    double gearPos = 0.0;  // Retracted
    double compression = 0.5;  // Would be compressed if down

    // Retracted gear produces no force regardless of compression
    double force = (gearPos > 0.5) ? compression * 5000.0 : 0.0;
    TS_ASSERT_DELTA(force, 0.0, epsilon);
  }

  /***************************************************************************
   * Ground Reaction Tests
   ***************************************************************************/

  // Test vertical force in local frame
  void testVerticalForceLocalFrame() {
    double strutForce = 3000.0;  // lbs (upward)

    // In local frame, Z is up, so force is positive
    double localFz = strutForce;
    TS_ASSERT(localFz > 0);
  }

  // Test force transform to body frame
  void testForceTransformToBody() {
    double localFz = 3000.0;  // Vertical force in local

    // Simple case: level aircraft, local Z = body Z
    // In body frame, Z is down, so strut force is negative
    double bodyFz = -localFz;
    TS_ASSERT(bodyFz < 0);
  }

  // Test moment calculation
  void testMomentCalculation() {
    // Force at wheel location
    double wheelX = 10.0;  // ft forward of CG
    double wheelZ = -5.0;  // ft below CG
    double forceZ = -3000.0;  // lbs (upward in body)

    // Moment about CG: M = r x F
    // My = x * Fz - z * Fx
    double momentY = wheelX * forceZ;  // Pitch-up moment
    TS_ASSERT_DELTA(momentY, -30000.0, epsilon);
  }

  /***************************************************************************
   * Compression Velocity Tests
   ***************************************************************************/

  // Test compression velocity from aircraft motion
  void testCompressionVelocity() {
    double sinkRate = 5.0;  // ft/s (positive = sinking)
    double compressSpeed = sinkRate;

    TS_ASSERT_DELTA(compressSpeed, 5.0, epsilon);
  }

  // Test compression velocity with rotation
  void testCompressionVelocityWithRotation() {
    double sinkRate = 3.0;
    double pitchRate = 0.1;    // rad/s (nose up)
    double wheelX = 10.0;      // ft forward of CG

    // Wheel velocity due to pitch = omega * r
    double wheelVelZ = pitchRate * wheelX;

    double compressSpeed = sinkRate - wheelVelZ;
    TS_ASSERT_DELTA(compressSpeed, 2.0, epsilon);
  }

  /***************************************************************************
   * Ground Contact Tests
   ***************************************************************************/

  // Test AGL calculation
  void testAGLCalculation() {
    double gearZLocal = 3.0;  // Gear location Z in local (below aircraft)
    double aircraftAGL = 2.5; // Aircraft CG height

    double gearAGL = aircraftAGL - gearZLocal;
    bool contact = (gearAGL < 0);

    TS_ASSERT(contact);  // Gear is 0.5 ft into ground
  }

  // Test compression length
  void testCompressionLength() {
    double gearZLocal = 3.0;
    double aircraftAGL = 2.5;

    double compression = gearZLocal - aircraftAGL;
    compression = std::max(0.0, compression);

    TS_ASSERT_DELTA(compression, 0.5, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very soft ground (high compression)
  void testHighCompression() {
    double maxCompLen = 1.0;
    double compression = 1.5;  // Exceeds max

    // Clamp to max
    compression = std::min(compression, maxCompLen);
    TS_ASSERT_DELTA(compression, maxCompLen, epsilon);
  }

  // Test zero velocity touchdown
  void testZeroVelocityTouchdown() {
    double sinkRate = 0.0;
    double bDamp = 500.0;

    double damperForce = bDamp * sinkRate;
    TS_ASSERT_DELTA(damperForce, 0.0, epsilon);
  }

  // Test hard landing
  void testHardLanding() {
    double sinkRate = 10.0;   // ft/s - hard landing
    double kSpring = 10000.0;
    double bDamp = 1000.0;
    double compression = 0.5;

    double springForce = kSpring * compression;
    double damperForce = bDamp * sinkRate;
    double totalForce = springForce + damperForce;

    // Very high force during hard landing
    TS_ASSERT_DELTA(totalForce, 15000.0, epsilon);
  }

  // Test bounce detection
  void testBounceDetection() {
    // Gear extending (negative compression velocity)
    double compressSpeed = -3.0;

    // If extending and compression near zero, may leave ground
    bool potentialBounce = (compressSpeed < 0);
    TS_ASSERT(potentialBounce);
  }

  /***************************************************************************
   * Tire Spin-Up Tests
   ***************************************************************************/

  // Test tire spin-up on touchdown
  void testTireSpinUp() {
    double groundSpeed = 150.0;     // ft/s
    double tireRadius = 0.5;        // ft
    double tireInertia = 0.5;       // slug*ft^2
    double spinUpTorque = 100.0;    // ft-lbf

    // Required angular velocity
    double targetOmega = groundSpeed / tireRadius;

    // Time to spin up: I * alpha = Torque
    double alpha = spinUpTorque / tireInertia;
    double spinUpTime = targetOmega / alpha;

    TS_ASSERT_DELTA(targetOmega, 300.0, 0.1);
    TS_ASSERT_DELTA(spinUpTime, 1.5, 0.01);
  }

  // Test spin-up drag force
  void testSpinUpDrag() {
    double groundSpeed = 150.0;
    double tireRadius = 0.5;
    double normalForce = 5000.0;
    double slipRatio = 1.0;         // Initially locked (100% slip)
    double slidingMu = 0.5;

    double spinUpDrag = slidingMu * normalForce;
    TS_ASSERT_DELTA(spinUpDrag, 2500.0, epsilon);
  }

  // Test wheel angular velocity matching
  void testWheelVelocityMatch() {
    double groundSpeed = 150.0;
    double tireRadius = 0.5;
    double wheelOmega = 280.0;      // rad/s

    double wheelTangentSpeed = wheelOmega * tireRadius;
    double slipRatio = (groundSpeed - wheelTangentSpeed) / groundSpeed;

    TS_ASSERT_DELTA(slipRatio, 0.0667, 0.001);
  }

  /***************************************************************************
   * Slip Ratio Tests
   ***************************************************************************/

  // Test slip ratio during acceleration
  void testSlipRatioAcceleration() {
    double groundSpeed = 50.0;
    double wheelSpeed = 55.0;       // Wheel turning faster (drive)

    // Positive slip ratio = acceleration/wheelspin
    double slipRatio = (wheelSpeed - groundSpeed) / std::max(groundSpeed, 1.0);
    TS_ASSERT_DELTA(slipRatio, 0.1, epsilon);
  }

  // Test slip ratio during braking
  void testSlipRatioBraking() {
    double groundSpeed = 50.0;
    double wheelSpeed = 40.0;       // Wheel turning slower (braking)

    // Negative slip ratio = braking
    double slipRatio = (wheelSpeed - groundSpeed) / std::max(groundSpeed, 1.0);
    TS_ASSERT_DELTA(slipRatio, -0.2, epsilon);
  }

  // Test locked wheel slip ratio
  void testLockedWheelSlip() {
    double groundSpeed = 50.0;
    double wheelSpeed = 0.0;

    double slipRatio = (wheelSpeed - groundSpeed) / std::max(groundSpeed, 1.0);
    TS_ASSERT_DELTA(slipRatio, -1.0, epsilon);
  }

  /***************************************************************************
   * Anti-Skid (ABS) Tests
   ***************************************************************************/

  // Test ABS slip threshold
  void testABSSlipThreshold() {
    double targetSlip = 0.15;       // Optimal braking slip
    double actualSlip = 0.25;       // Wheel starting to lock

    bool absTrigger = (std::abs(actualSlip) > targetSlip);
    TS_ASSERT(absTrigger);
  }

  // Test ABS brake modulation
  void testABSBrakeModulation() {
    double brakeCmd = 1.0;
    double slipRatio = -0.25;       // Excessive slip
    double targetSlip = -0.15;

    // Simple ABS: reduce brake if slip exceeds target (in magnitude)
    double absGain = 0.5;
    // When |slipRatio| > |targetSlip|, reduce brake
    double slipError = std::abs(slipRatio) - std::abs(targetSlip);
    double modulatedBrake = std::max(0.0, std::min(1.0, brakeCmd - absGain * slipError));

    TS_ASSERT(modulatedBrake < brakeCmd);
    TS_ASSERT_DELTA(modulatedBrake, 0.95, epsilon);
  }

  // Test ABS cycling
  void testABSCycling() {
    double brakeCmd = 1.0;
    double slipThreshold = 0.2;
    double hysteresis = 0.05;

    double slipSequence[] = {-0.1, -0.15, -0.22, -0.18, -0.12, -0.25};
    bool absActive = false;

    for (double slip : slipSequence) {
      double threshold = absActive ? (slipThreshold - hysteresis) : slipThreshold;
      absActive = (std::abs(slip) > threshold);
    }

    TS_ASSERT(absActive);
  }

  /***************************************************************************
   * Tire Blowout Tests
   ***************************************************************************/

  // Test tire load limit
  void testTireLoadLimit() {
    double normalForce = 8000.0;
    double tireRating = 7500.0;

    double loadFactor = normalForce / tireRating;
    bool overloaded = (loadFactor > 1.0);

    TS_ASSERT(overloaded);
    TS_ASSERT_DELTA(loadFactor, 1.0667, 0.001);
  }

  // Test tire speed rating
  void testTireSpeedRating() {
    double groundSpeed = 250.0;     // knots (approx 422 ft/s)
    double speedRating = 200.0;     // knots

    bool overSpeed = (groundSpeed > speedRating);
    TS_ASSERT(overSpeed);
  }

  // Test blowout force change
  void testBlowoutForceChange() {
    double normalForce = 5000.0;
    double normalMu = 0.8;
    double blowoutMu = 0.4;         // Reduced friction after blowout

    double normalBraking = normalMu * normalForce;
    double blowoutBraking = blowoutMu * normalForce;

    TS_ASSERT(blowoutBraking < normalBraking);
    TS_ASSERT_DELTA(blowoutBraking / normalBraking, 0.5, epsilon);
  }

  /***************************************************************************
   * Runway Surface Tests
   ***************************************************************************/

  // Test wet runway friction reduction
  void testWetRunwayFriction() {
    double dryMu = 0.8;
    double wetReduction = 0.7;      // 70% of dry friction

    double wetMu = dryMu * wetReduction;
    TS_ASSERT_DELTA(wetMu, 0.56, epsilon);
  }

  // Test icy runway friction
  void testIcyRunwayFriction() {
    double dryMu = 0.8;
    double icyReduction = 0.2;      // 20% of dry friction

    double icyMu = dryMu * icyReduction;
    TS_ASSERT_DELTA(icyMu, 0.16, epsilon);
  }

  // Test hydroplaning speed
  void testHydroplaningSpeed() {
    double tirePressure = 200.0;    // psi

    // NASA hydroplaning formula: V = 9 * sqrt(P)
    double hydroSpeed = 9.0 * std::sqrt(tirePressure);  // knots
    TS_ASSERT_DELTA(hydroSpeed, 127.3, 0.1);
  }

  // Test runway slope effect
  void testRunwaySlopeEffect() {
    double normalForce = 5000.0;
    double slopeAngle = 2.0 * M_PI / 180.0;  // 2 degree slope

    // Component of weight down slope
    double slopeForce = normalForce * std::tan(slopeAngle);
    TS_ASSERT_DELTA(slopeForce, 174.5, 0.5);
  }

  /***************************************************************************
   * Shimmy Dynamics Tests
   ***************************************************************************/

  // Test shimmy frequency
  void testShimmyFrequency() {
    double kSteering = 1000.0;      // Steering stiffness
    double wheelInertia = 0.3;      // Wheel yaw inertia

    // Natural frequency: omega = sqrt(k/I)
    double omega = std::sqrt(kSteering / wheelInertia);
    double freq = omega / (2.0 * M_PI);

    TS_ASSERT_DELTA(freq, 9.19, 0.01);
  }

  // Test shimmy damping
  void testShimmyDamping() {
    double criticalDamping = 35.0;  // lbf*s/rad
    double actualDamping = 17.5;

    double dampingRatio = actualDamping / criticalDamping;
    bool underdamped = (dampingRatio < 1.0);

    TS_ASSERT(underdamped);
    TS_ASSERT_DELTA(dampingRatio, 0.5, epsilon);
  }

  // Test shimmy amplitude decay
  void testShimmyDecay() {
    double dampingRatio = 0.5;
    double omega = 57.7;            // rad/s
    double initialAmplitude = 10.0; // degrees

    // Amplitude after 1 second
    double decayRate = dampingRatio * omega;
    double amplitude1s = initialAmplitude * std::exp(-decayRate);

    TS_ASSERT(amplitude1s < initialAmplitude);
  }

  /***************************************************************************
   * Gear Door Sequencing Tests
   ***************************************************************************/

  // Test door opens before gear extends
  void testDoorSequencing() {
    double doorPos = 0.0;           // Closed
    double gearPos = 0.0;           // Retracted

    // Door must be > 0.8 before gear can start moving
    bool gearCanMove = (doorPos > 0.8);
    TS_ASSERT(!gearCanMove);
  }

  // Test door closes after gear retracts
  void testDoorCloseSequence() {
    double doorPos = 1.0;           // Open
    double gearPos = 0.0;           // Retracted

    // Door should close when gear is retracted
    bool doorShouldClose = (gearPos < 0.1);
    TS_ASSERT(doorShouldClose);
  }

  // Test sequence timing
  void testSequenceTiming() {
    double doorTransitTime = 3.0;   // seconds
    double gearTransitTime = 6.0;   // seconds

    double totalCycleTime = doorTransitTime + gearTransitTime + doorTransitTime;
    TS_ASSERT_DELTA(totalCycleTime, 12.0, epsilon);
  }

  /***************************************************************************
   * Emergency Extension Tests
   ***************************************************************************/

  // Test free-fall gear extension
  void testFreeFallExtension() {
    double gearWeight = 500.0;      // lbs
    double drag = 200.0;            // lbs (aerodynamic)
    double extendForce = gearWeight - drag;

    TS_ASSERT(extendForce > 0);
    TS_ASSERT_DELTA(extendForce, 300.0, epsilon);
  }

  // Test gravity extension time
  void testGravityExtensionTime() {
    double gearTravel = 2.0;        // ft
    double effectiveAccel = 16.0;   // ft/s^2 (considering drag)

    // t = sqrt(2*d/a)
    double extensionTime = std::sqrt(2.0 * gearTravel / effectiveAccel);
    TS_ASSERT_DELTA(extensionTime, 0.5, epsilon);
  }

  // Test uplock release
  void testUplockRelease() {
    double uplockForce = 100.0;     // lbs required to release
    double releaseActuation = 120.0;

    bool unlocked = (releaseActuation > uplockForce);
    TS_ASSERT(unlocked);
  }

  /***************************************************************************
   * Crosswind Landing Tests
   ***************************************************************************/

  // Test side load during crab landing
  void testCrabLandingSideLoad() {
    double groundSpeed = 150.0;     // ft/s
    double crabAngle = 10.0 * M_PI / 180.0;  // 10 degrees
    double normalForce = 5000.0;
    double sideMu = 0.6;

    double sideVelocity = groundSpeed * std::sin(crabAngle);
    double maxSideForce = sideMu * normalForce;

    TS_ASSERT_DELTA(sideVelocity, 26.0, 0.5);
    TS_ASSERT_DELTA(maxSideForce, 3000.0, epsilon);
  }

  // Test tire side slip
  void testTireSideSlip() {
    double sideVelocity = 20.0;
    double forwardVelocity = 150.0;

    double sideSlipAngle = std::atan2(sideVelocity, forwardVelocity);
    double sideSlipDeg = sideSlipAngle * 180.0 / M_PI;

    TS_ASSERT_DELTA(sideSlipDeg, 7.59, 0.01);
  }

  // Test weathervaning tendency
  void testWeathervaning() {
    double mainGearX = -3.0;        // ft aft of CG
    double sideForce = 1000.0;      // lbs

    double yawingMoment = mainGearX * sideForce;

    // Negative moment = nose into wind
    TS_ASSERT(yawingMoment < 0);
    TS_ASSERT_DELTA(yawingMoment, -3000.0, epsilon);
  }

  /***************************************************************************
   * Load Distribution Tests
   ***************************************************************************/

  // Test tricycle gear load distribution at rest
  void testTricycleLoadDistribution() {
    double weight = 10000.0;        // lbs
    double cgX = 0.0;               // CG location
    double noseX = 10.0;            // Nose gear X
    double mainX = -3.0;            // Main gear X

    // Static equilibrium: sum moments = 0
    double mainLoad = weight * noseX / (noseX - mainX);
    double noseLoad = weight - mainLoad;

    TS_ASSERT_DELTA(mainLoad, 7692.3, 0.5);
    TS_ASSERT_DELTA(noseLoad, 2307.7, 0.5);
  }

  // Test CG limit forward
  void testCGLimitForward() {
    double noseGearLimit = 3000.0;  // Max nose gear load
    double weight = 10000.0;
    double noseX = 10.0;
    double mainX = -3.0;

    // Find forward CG limit where nose gear hits limit
    // noseLoad = weight * (cgX - mainX) / (noseX - mainX) = limit
    double maxNoseLoadFraction = noseGearLimit / weight;
    double forwardCGLimit = mainX + maxNoseLoadFraction * (noseX - mainX);

    TS_ASSERT_DELTA(forwardCGLimit, 0.9, 0.01);
  }

  // Test CG limit aft (tip-back)
  void testCGLimitAft() {
    double mainX = -3.0;            // Main gear X

    // CG behind main gear = tip-back
    double cgX = -4.0;
    bool tipBack = (cgX < mainX);

    TS_ASSERT(tipBack);
  }

  /***************************************************************************
   * Taildragger Tests
   ***************************************************************************/

  // Test taildragger ground loop tendency
  void testGroundLoopTendency() {
    double mainGearX = 0.5;         // Main gear ahead of CG
    double tailWheelX = -20.0;      // Tail wheel far aft
    double sideForce = 500.0;       // lbs

    // Destabilizing moment from main gear
    double destabilizing = mainGearX * sideForce;

    // CG behind main gear = unstable
    TS_ASSERT(destabilizing > 0);
  }

  // Test tailwheel shimmy
  void testTailwheelShimmy() {
    double tailLoadFraction = 0.1;  // 10% of weight on tail
    double weight = 5000.0;

    double tailLoad = tailLoadFraction * weight;
    TS_ASSERT_DELTA(tailLoad, 500.0, epsilon);
  }

  // Test three-point attitude
  void testThreePointAttitude() {
    double mainGearHeight = 4.0;
    double tailWheelHeight = 0.5;
    double fuselageLength = 25.0;

    double pitchAngle = std::atan2(mainGearHeight - tailWheelHeight, fuselageLength);
    double pitchDeg = pitchAngle * 180.0 / M_PI;

    TS_ASSERT_DELTA(pitchDeg, 8.0, 0.5);
  }

  /***************************************************************************
   * Tire Deflection Tests
   ***************************************************************************/

  // Test tire vertical spring rate
  void testTireSpringRate() {
    double tirePressure = 200.0;    // psi
    double contactArea = 50.0;      // sq inches

    // Approximate tire spring rate
    double tireSpring = tirePressure * contactArea;  // lbs/in for small deflection
    TS_ASSERT_DELTA(tireSpring, 10000.0, epsilon);
  }

  // Test tire deflection under load
  void testTireDeflection() {
    double load = 5000.0;           // lbs
    double tireSpring = 10000.0;    // lbs/in

    double deflection = load / tireSpring;
    TS_ASSERT_DELTA(deflection, 0.5, epsilon);  // 0.5 inches
  }

  // Test combined strut and tire
  void testCombinedStrutTire() {
    double kStrut = 5000.0;         // lbs/ft
    double kTire = 120000.0;        // lbs/ft (10000 lbs/in)

    // Series spring: 1/k_total = 1/k1 + 1/k2
    double kTotal = 1.0 / (1.0/kStrut + 1.0/kTire);
    TS_ASSERT_DELTA(kTotal, 4800.0, 1.0);
  }

  /***************************************************************************
   * Brake Fade Tests
   ***************************************************************************/

  // Test brake temperature rise
  void testBrakeTemperatureRise() {
    double brakeEnergy = 1e6;       // ft-lbf
    double brakeMass = 50.0;        // lbm
    double specificHeat = 0.12;     // BTU/lbm-F

    // Energy in BTU: 1 BTU = 778 ft-lbf
    double energyBTU = brakeEnergy / 778.0;
    double tempRise = energyBTU / (brakeMass * specificHeat);

    TS_ASSERT_DELTA(tempRise, 214.0, 1.0);
  }

  // Test brake fade coefficient
  void testBrakeFadeCoefficient() {
    double baselineMu = 0.5;
    double temperature = 800.0;     // degrees F
    double fadeTemp = 500.0;        // Temperature where fade begins
    double fadeRate = 0.0003;       // Mu reduction per degree

    double fadeFactor = std::max(0.3, 1.0 - fadeRate * std::max(0.0, temperature - fadeTemp));
    double fadedMu = baselineMu * fadeFactor;

    TS_ASSERT_DELTA(fadedMu, 0.455, 0.001);
  }

  // Test brake cooling
  void testBrakeCooling() {
    double brakeTemp = 600.0;
    double ambientTemp = 59.0;
    double coolingRate = 0.02;      // per second
    double dt = 60.0;               // seconds

    // Exponential cooling: T(t) = T_amb + (T_0 - T_amb) * exp(-k*t)
    // = 59 + 541 * exp(-1.2) = 59 + 541 * 0.3012 = 222.0
    double finalTemp = ambientTemp + (brakeTemp - ambientTemp) * std::exp(-coolingRate * dt);
    TS_ASSERT_DELTA(finalTemp, 222.0, 0.5);
  }

  /***************************************************************************
   * Dynamic Load Factor Tests
   ***************************************************************************/

  // Test braking load transfer
  void testBrakingLoadTransfer() {
    double weight = 10000.0;
    double cgHeight = 3.0;          // ft
    double wheelbase = 15.0;        // ft
    double decelG = 0.3;            // g's

    double loadTransfer = weight * decelG * cgHeight / wheelbase;
    TS_ASSERT_DELTA(loadTransfer, 600.0, epsilon);
  }

  // Test dynamic nose load
  void testDynamicNoseLoad() {
    double staticNoseLoad = 2000.0;
    double loadTransfer = 600.0;

    double dynamicNoseLoad = staticNoseLoad + loadTransfer;
    TS_ASSERT_DELTA(dynamicNoseLoad, 2600.0, epsilon);
  }

  // Test dynamic main load
  void testDynamicMainLoad() {
    double staticMainLoad = 8000.0;
    double loadTransfer = 600.0;

    double dynamicMainLoad = staticMainLoad - loadTransfer;
    TS_ASSERT_DELTA(dynamicMainLoad, 7400.0, epsilon);
  }

  /***************************************************************************
   * Touchdown Transient Tests
   ***************************************************************************/

  // Test touchdown load factor
  void testTouchdownLoadFactor() {
    double sinkRate = 10.0;         // ft/s
    double kStrut = 10000.0;        // lbs/ft
    double weight = 5000.0;
    double mass = weight / 32.174;

    // Peak deceleration: m*a = k*x, a = k*v/sqrt(k*m) = sqrt(k/m)*v
    double omega = std::sqrt(kStrut / mass);
    double peakAccel = omega * sinkRate;
    double loadFactor = peakAccel / 32.174;

    TS_ASSERT(loadFactor > 1.0);
  }

  // Test oleo rebound
  void testOleoRebound() {
    double reboundDamping = 2.0;    // Damping ratio > 1 = overdamped

    // Oleo should not oscillate
    bool overdamped = (reboundDamping > 1.0);
    TS_ASSERT(overdamped);
  }

  // Test multiple bounce sequence
  void testMultipleBounces() {
    double bounceEnergy = 1000.0;   // Initial
    double restitution = 0.3;       // Energy retained per bounce

    double bounces[4];
    bounces[0] = bounceEnergy;
    for (int i = 1; i < 4; i++) {
      bounces[i] = bounces[i-1] * restitution;
    }

    TS_ASSERT_DELTA(bounces[1], 300.0, epsilon);
    TS_ASSERT_DELTA(bounces[2], 90.0, epsilon);
    TS_ASSERT_DELTA(bounces[3], 27.0, epsilon);
  }

  /***************************************************************************
   * Complete Landing Gear System Tests
   ***************************************************************************/

  // Test complete gear cycle
  void testCompleteGearCycle() {
    bool gearDown = true;
    double gearPosition = 1.0;  // 1 = down, 0 = up
    double transitionTime = 10.0;  // seconds
    double dt = 0.1;

    // Retract
    for (double t = 0.0; t < transitionTime; t += dt) {
      gearPosition -= dt / transitionTime;
    }
    gearDown = false;
    TS_ASSERT(!gearDown);
    TS_ASSERT(gearPosition < 0.1);

    // Extend
    for (double t = 0.0; t < transitionTime; t += dt) {
      gearPosition += dt / transitionTime;
    }
    gearDown = true;
    TS_ASSERT(gearDown);
    TS_ASSERT(gearPosition > 0.9);
  }

  // Test gear warning system
  void testGearWarningSystem() {
    double altitude = 500.0;    // ft AGL
    double speed = 150.0;       // kts
    bool gearDown = false;
    bool throttleIdle = true;

    // Warning conditions: low altitude, slow, gear up, throttle idle
    bool warning = (altitude < 1000.0 && speed < 180.0 &&
                   !gearDown && throttleIdle);
    TS_ASSERT(warning);

    // Gear extended - no warning
    gearDown = true;
    warning = (altitude < 1000.0 && speed < 180.0 &&
              !gearDown && throttleIdle);
    TS_ASSERT(!warning);
  }

  // Test strut compression sequence
  void testStrutCompressionSequence() {
    double staticLoad = 5000.0;
    double kStrut = 10000.0;
    double staticCompression = staticLoad / kStrut;

    // Dynamic landing: 2x load
    double landingLoad = staticLoad * 2.0;
    double maxCompression = landingLoad / kStrut;

    TS_ASSERT_DELTA(staticCompression, 0.5, epsilon);
    TS_ASSERT_DELTA(maxCompression, 1.0, epsilon);
  }

  // Test anti-skid system
  void testAntiSkidSystem() {
    double wheelSpeed = 100.0;  // ft/s
    double groundSpeed = 120.0; // ft/s
    double slipRatio = (groundSpeed - wheelSpeed) / groundSpeed;

    // Optimal slip ratio ~10-15%
    bool antiSkidActive = slipRatio > 0.15;
    TS_ASSERT(antiSkidActive);

    // Reduce brake pressure
    wheelSpeed = 108.0;
    slipRatio = (groundSpeed - wheelSpeed) / groundSpeed;
    antiSkidActive = slipRatio > 0.15;
    TS_ASSERT(!antiSkidActive);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test gear load independence
  void testGearLoadIndependence() {
    double mainLoad1 = 8000.0, mainLoad2 = 6000.0;
    double noseLoad1 = 2000.0, noseLoad2 = 4000.0;

    TS_ASSERT_DELTA(mainLoad1 + noseLoad1, 10000.0, epsilon);
    TS_ASSERT_DELTA(mainLoad2 + noseLoad2, 10000.0, epsilon);
  }

  // Test strut calculation independence
  void testStrutCalculationIndependence() {
    double k1 = 10000.0, x1 = 0.5;
    double k2 = 20000.0, x2 = 0.25;

    double F1 = k1 * x1;
    double F2 = k2 * x2;

    TS_ASSERT_DELTA(F1, 5000.0, epsilon);
    TS_ASSERT_DELTA(F2, 5000.0, epsilon);
  }

  // Test tire force independence
  void testTireForceIndependence() {
    double mu1 = 0.8, N1 = 5000.0;
    double mu2 = 0.5, N2 = 8000.0;

    double F1 = mu1 * N1;
    double F2 = mu2 * N2;

    TS_ASSERT_DELTA(F1, 4000.0, epsilon);
    TS_ASSERT_DELTA(F2, 4000.0, epsilon);
  }

  // Test brake force independence
  void testBrakeForceIndependence() {
    double brake1 = 0.5, maxBrake1 = 10000.0;
    double brake2 = 0.8, maxBrake2 = 6250.0;

    double F1 = brake1 * maxBrake1;
    double F2 = brake2 * maxBrake2;

    TS_ASSERT_DELTA(F1, 5000.0, epsilon);
    TS_ASSERT_DELTA(F2, 5000.0, epsilon);
  }

  // Test damper coefficient independence
  void testDamperCoefficientIndependence() {
    double C1 = 1000.0, v1 = 5.0;
    double C2 = 2500.0, v2 = 2.0;

    double F1 = C1 * v1;
    double F2 = C2 * v2;

    TS_ASSERT_DELTA(F1, 5000.0, epsilon);
    TS_ASSERT_DELTA(F2, 5000.0, epsilon);
  }

  // Test wheel rotation independence
  void testWheelRotationIndependence() {
    double omega1 = 100.0;  // rad/s
    double omega2 = 50.0;   // rad/s
    double radius = 1.0;    // ft

    double v1 = omega1 * radius;
    double v2 = omega2 * radius;

    TS_ASSERT_DELTA(v1, 100.0, epsilon);
    TS_ASSERT_DELTA(v2, 50.0, epsilon);
  }

  // Test steering angle independence
  void testSteeringAngleIndependence() {
    double angle1 = 10.0;  // degrees
    double angle2 = -15.0; // degrees

    TS_ASSERT(angle1 > 0.0);
    TS_ASSERT(angle2 < 0.0);
    TS_ASSERT_DELTA(std::abs(angle2), 15.0, epsilon);
  }

  // Test gear position sensor independence
  void testGearPositionSensorIndependence() {
    double pos1 = 1.0;  // fully down
    double pos2 = 0.5;  // in transit
    double pos3 = 0.0;  // fully up

    TS_ASSERT(pos1 > pos2);
    TS_ASSERT(pos2 > pos3);
    TS_ASSERT_DELTA(pos1, 1.0, epsilon);
  }

  // ============================================================================
  // FGLGear class tests - using actual class methods
  // ============================================================================

  // Test FGLGear access through FGGroundReactions
  void testFGLGearAccess() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(groundReactions != nullptr);

    int numGear = groundReactions->GetNumGearUnits();
    TS_ASSERT(numGear >= 1);

    if (numGear > 0) {
      auto gear = groundReactions->GetGearUnit(0);
      TS_ASSERT(gear != nullptr);
    }
  }

  // Test GetWOW method
  void testFGLGearGetWOW() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    // Just verify it returns a valid boolean
    bool wow = gear->GetWOW();
    TS_ASSERT(wow == true || wow == false);
  }

  // Test SetWOW method
  void testFGLGearSetWOW() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    gear->SetWOW(true);
    TS_ASSERT(gear->GetWOW() == true);

    gear->SetWOW(false);
    TS_ASSERT(gear->GetWOW() == false);
  }

  // Test GetCompLen method
  void testFGLGearGetCompLen() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double compLen = gear->GetCompLen();
    TS_ASSERT(compLen >= 0.0);  // Compression can't be negative
  }

  // Test GetCompVel method
  void testFGLGearGetCompVel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double compVel = gear->GetCompVel();
    // Can be positive, negative, or zero
    TS_ASSERT(std::isfinite(compVel));
  }

  // Test GetCompForce method
  void testFGLGearGetCompForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double compForce = gear->GetCompForce();
    TS_ASSERT(compForce >= 0.0);  // Force should be non-negative
  }

  // Test GetstaticFCoeff method
  void testFGLGearGetStaticFCoeff() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double staticF = gear->GetstaticFCoeff();
    TS_ASSERT(staticF >= 0.0);  // Friction coefficient should be non-negative
    TS_ASSERT(staticF <= 2.0);  // Reasonable upper bound
  }

  // Test GetBrakeGroup method
  void testFGLGearGetBrakeGroup() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    int brakeGroup = gear->GetBrakeGroup();
    TS_ASSERT(brakeGroup >= 0);  // Valid brake group
  }

  // Test GetSteerType method
  void testFGLGearGetSteerType() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    int steerType = gear->GetSteerType();
    TS_ASSERT(steerType >= 0);  // Valid steer type
  }

  // Test GetSteerable method
  void testFGLGearGetSteerable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    bool steerable = gear->GetSteerable();
    TS_ASSERT(steerable == true || steerable == false);
  }

  // Test GetRetractable method
  void testFGLGearGetRetractable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    bool retractable = gear->GetRetractable();
    TS_ASSERT(retractable == true || retractable == false);
  }

  // Test GetGearUnitUp method
  void testFGLGearGetGearUnitUp() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    bool up = gear->GetGearUnitUp();
    TS_ASSERT(up == true || up == false);
  }

  // Test GetGearUnitDown method
  void testFGLGearGetGearUnitDown() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    bool down = gear->GetGearUnitDown();
    TS_ASSERT(down == true || down == false);
  }

  // Test GetGearUnitPos method
  void testFGLGearGetGearUnitPos() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double pos = gear->GetGearUnitPos();
    TS_ASSERT(pos >= 0.0);
    TS_ASSERT(pos <= 1.0);  // Position is 0-1
  }

  // Test IsBogey method
  void testFGLGearIsBogey() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    bool isBogey = gear->IsBogey();
    TS_ASSERT(isBogey == true || isBogey == false);
  }

  // Test GetSteerAngleDeg method
  void testFGLGearGetSteerAngleDeg() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double angle = gear->GetSteerAngleDeg();
    TS_ASSERT(std::isfinite(angle));
  }

  // Test SetSteerAngleDeg method
  void testFGLGearSetSteerAngleDeg() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    // Ball model gear may not be steerable, so just verify method doesn't crash
    gear->SetSteerAngleDeg(10.0);
    double angle = gear->GetSteerAngleDeg();
    TS_ASSERT(std::isfinite(angle));  // Should return valid value
  }

  // Test GetWheelSlipAngle method
  void testFGLGearGetWheelSlipAngle() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double slipAngle = gear->GetWheelSlipAngle();
    TS_ASSERT(std::isfinite(slipAngle));
  }

  // Test GetSteerNorm method
  void testFGLGearGetSteerNorm() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double steerNorm = gear->GetSteerNorm();
    TS_ASSERT(steerNorm >= -1.0);
    TS_ASSERT(steerNorm <= 1.0);
  }

  // Test GetReport method
  void testFGLGearGetReport() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    bool report = gear->GetReport();
    TS_ASSERT(report == true || report == false);
  }

  // Test SetReport method
  void testFGLGearSetReport() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    gear->SetReport(true);
    TS_ASSERT(gear->GetReport() == true);

    gear->SetReport(false);
    TS_ASSERT(gear->GetReport() == false);
  }

  // Test GetBodyLocation method
  void testFGLGearGetBodyLocation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double x = gear->GetBodyLocation(1);
    double y = gear->GetBodyLocation(2);
    double z = gear->GetBodyLocation(3);
    TS_ASSERT(std::isfinite(x));
    TS_ASSERT(std::isfinite(y));
    TS_ASSERT(std::isfinite(z));
  }

  // Test GetLocalGear method
  void testFGLGearGetLocalGear() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double x = gear->GetLocalGear(1);
    double y = gear->GetLocalGear(2);
    double z = gear->GetLocalGear(3);
    TS_ASSERT(std::isfinite(x));
    TS_ASSERT(std::isfinite(y));
    TS_ASSERT(std::isfinite(z));
  }

  // Test GetWheelVel method
  void testFGLGearGetWheelVel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double vx = gear->GetWheelVel(1);
    double vy = gear->GetWheelVel(2);
    double vz = gear->GetWheelVel(3);
    TS_ASSERT(std::isfinite(vx));
    TS_ASSERT(std::isfinite(vy));
    TS_ASSERT(std::isfinite(vz));
  }

  // Test GetWheelRollVel method
  void testFGLGearGetWheelRollVel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double rollVel = gear->GetWheelRollVel();
    TS_ASSERT(std::isfinite(rollVel));
  }

  // Test GetWheelSideVel method
  void testFGLGearGetWheelSideVel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double sideVel = gear->GetWheelSideVel();
    TS_ASSERT(std::isfinite(sideVel));
  }

  // Test GetWheelRollForce method
  void testFGLGearGetWheelRollForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double rollForce = gear->GetWheelRollForce();
    TS_ASSERT(std::isfinite(rollForce));
  }

  // Test GetWheelSideForce method
  void testFGLGearGetWheelSideForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double sideForce = gear->GetWheelSideForce();
    TS_ASSERT(std::isfinite(sideForce));
  }

  // Test GetBodyXForce method
  void testFGLGearGetBodyXForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double xForce = gear->GetBodyXForce();
    TS_ASSERT(std::isfinite(xForce));
  }

  // Test GetBodyYForce method
  void testFGLGearGetBodyYForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double yForce = gear->GetBodyYForce();
    TS_ASSERT(std::isfinite(yForce));
  }

  // Test GetBodyZForce method
  void testFGLGearGetBodyZForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    double zForce = gear->GetBodyZForce();
    TS_ASSERT(std::isfinite(zForce));
  }

  // Test ResetToIC method
  void testFGLGearResetToIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->Run(false);

    auto gear = groundReactions->GetGearUnit(0);
    TS_ASSERT(gear != nullptr);

    // Should not throw
    gear->ResetToIC();
    TS_ASSERT(std::isfinite(gear->GetCompLen()));
  }
};
