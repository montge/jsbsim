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
};
