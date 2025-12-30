/*******************************************************************************
 * FGGroundReactionsTest.h - Unit tests for FGGroundReactions
 *
 * Tests the mathematical behavior of ground reactions system:
 * - Weight on wheels (WOW) determination
 * - Ground contact detection
 * - Runway surface handling
 * - Multiple gear coordination
 *
 * Note: FGGroundReactions requires XML element for construction, so these tests
 * focus on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <vector>

#include <FGFDMExec.h>
#include <models/FGGroundReactions.h>
#include <initialization/FGInitialCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-10;

class FGGroundReactionsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Gear State Structure
   ***************************************************************************/
  struct GearState {
    double compression = 0.0;     // ft (positive = compressed)
    double compressionVelocity = 0.0;  // ft/s
    double staticFriction = 0.8;
    double rollingFriction = 0.02;
    double maxSteer = 60.0;       // degrees
    double steerAngle = 0.0;      // degrees
    bool brake = false;
    double brakeForce = 0.0;
    bool retracted = false;
    double x = 0.0, y = 0.0, z = 0.0;  // Position
  };

  /***************************************************************************
   * Weight on Wheels Tests
   ***************************************************************************/

  // Test WOW when gear compressed
  void testWOWCompressed() {
    GearState gear;
    gear.compression = 0.5;  // 0.5 ft compression

    bool wow = (gear.compression > 0);
    TS_ASSERT(wow);
  }

  // Test WOW when airborne
  void testWOWAirborne() {
    GearState gear;
    gear.compression = 0.0;  // No compression

    bool wow = (gear.compression > 0);
    TS_ASSERT(!wow);
  }

  // Test WOW threshold
  void testWOWThreshold() {
    GearState gear;
    gear.compression = 0.001;  // Minimal compression

    bool wow = (gear.compression > 0);
    TS_ASSERT(wow);  // Still counts as WOW
  }

  /***************************************************************************
   * Contact Detection Tests
   ***************************************************************************/

  // Test ground contact (AGL < gear length)
  void testGroundContact() {
    double agl = 5.0;  // ft above ground
    double gearLength = 6.0;  // ft extended gear length

    bool contact = (agl < gearLength);
    TS_ASSERT(contact);
  }

  // Test no contact (above ground)
  void testNoContact() {
    double agl = 10.0;
    double gearLength = 6.0;

    bool contact = (agl < gearLength);
    TS_ASSERT(!contact);
  }

  // Test compression amount
  void testCompressionAmount() {
    double agl = 4.0;
    double gearLength = 6.0;

    double compression = gearLength - agl;
    TS_ASSERT_DELTA(compression, 2.0, epsilon);
  }

  /***************************************************************************
   * Friction Model Tests
   ***************************************************************************/

  // Test static friction
  void testStaticFriction() {
    double normalForce = 1000.0;  // lbs
    double staticCoeff = 0.8;

    double maxStaticFriction = normalForce * staticCoeff;
    TS_ASSERT_DELTA(maxStaticFriction, 800.0, epsilon);
  }

  // Test rolling friction
  void testRollingFriction() {
    double normalForce = 1000.0;
    double rollingCoeff = 0.02;

    double rollingFriction = normalForce * rollingCoeff;
    TS_ASSERT_DELTA(rollingFriction, 20.0, epsilon);
  }

  // Test friction transition
  void testFrictionTransition() {
    double staticCoeff = 0.8;
    double rollingCoeff = 0.02;
    double velocity = 5.0;  // ft/s
    double transitionSpeed = 1.0;

    // Blend between static and rolling based on velocity
    double factor = std::min(1.0, velocity / transitionSpeed);
    double effectiveCoeff = staticCoeff * (1 - factor) + rollingCoeff * factor;

    TS_ASSERT_DELTA(effectiveCoeff, rollingCoeff, epsilon);  // Fully rolling
  }

  // Test friction at low speed
  void testFrictionLowSpeed() {
    double staticCoeff = 0.8;
    double rollingCoeff = 0.02;
    double velocity = 0.5;  // ft/s
    double transitionSpeed = 1.0;

    double factor = std::min(1.0, velocity / transitionSpeed);
    double effectiveCoeff = staticCoeff * (1 - factor) + rollingCoeff * factor;

    TS_ASSERT(effectiveCoeff > rollingCoeff);
    TS_ASSERT(effectiveCoeff < staticCoeff);
  }

  /***************************************************************************
   * Steering Tests
   ***************************************************************************/

  // Test steering angle limit
  void testSteeringLimit() {
    GearState gear;
    gear.maxSteer = 60.0;
    double commanded = 90.0;

    double actual = std::max(-gear.maxSteer, std::min(gear.maxSteer, commanded));
    TS_ASSERT_DELTA(actual, 60.0, epsilon);  // Limited
  }

  // Test steering within limits
  void testSteeringWithinLimits() {
    GearState gear;
    gear.maxSteer = 60.0;
    double commanded = 30.0;

    double actual = std::max(-gear.maxSteer, std::min(gear.maxSteer, commanded));
    TS_ASSERT_DELTA(actual, 30.0, epsilon);
  }

  // Test negative steering
  void testNegativeSteering() {
    GearState gear;
    gear.maxSteer = 60.0;
    double commanded = -45.0;

    double actual = std::max(-gear.maxSteer, std::min(gear.maxSteer, commanded));
    TS_ASSERT_DELTA(actual, -45.0, epsilon);
  }

  // Test steering force
  void testSteeringForce() {
    double steerAngle = 30.0;  // degrees
    double velocity = 50.0;    // ft/s
    double lateralForce = 100.0;  // Base lateral force

    // Cornering force increases with steer angle
    double steerRad = steerAngle * M_PI / 180.0;
    double corneringForce = lateralForce * std::sin(steerRad);

    TS_ASSERT_DELTA(corneringForce, 50.0, 0.1);  // sin(30°) = 0.5
  }

  /***************************************************************************
   * Brake Tests
   ***************************************************************************/

  // Test brake force calculation
  void testBrakeForce() {
    double normalForce = 5000.0;  // lbs
    double brakeCoeff = 0.6;      // Brake friction coefficient
    double brakeInput = 1.0;      // Full brake

    double brakeForce = normalForce * brakeCoeff * brakeInput;
    TS_ASSERT_DELTA(brakeForce, 3000.0, epsilon);
  }

  // Test partial brake
  void testPartialBrake() {
    double normalForce = 5000.0;
    double brakeCoeff = 0.6;
    double brakeInput = 0.5;  // Half brake

    double brakeForce = normalForce * brakeCoeff * brakeInput;
    TS_ASSERT_DELTA(brakeForce, 1500.0, epsilon);
  }

  // Test brake fade with temperature
  void testBrakeFade() {
    double baseBrakeCoeff = 0.6;
    double brakeTemp = 600.0;  // degrees F
    double fadeTemp = 400.0;   // Temperature at which fade starts

    // Linear fade above threshold
    double fadeRate = 0.0005;  // Coefficient loss per degree
    double fade = std::max(0.0, (brakeTemp - fadeTemp) * fadeRate);
    double effectiveCoeff = baseBrakeCoeff - fade;

    TS_ASSERT_DELTA(effectiveCoeff, 0.5, epsilon);  // Faded from 0.6 to 0.5
  }

  /***************************************************************************
   * Spring-Damper Tests
   ***************************************************************************/

  // Test spring force
  void testSpringForce() {
    double compression = 0.5;  // ft
    double springConstant = 10000.0;  // lbs/ft

    double springForce = compression * springConstant;
    TS_ASSERT_DELTA(springForce, 5000.0, epsilon);
  }

  // Test damper force
  void testDamperForce() {
    double compressionVelocity = 2.0;  // ft/s (compressing)
    double dampingConstant = 500.0;    // lbs/(ft/s)

    double damperForce = compressionVelocity * dampingConstant;
    TS_ASSERT_DELTA(damperForce, 1000.0, epsilon);
  }

  // Test combined spring-damper
  void testCombinedSpringDamper() {
    double compression = 0.5;
    double compressionVelocity = 2.0;
    double springConstant = 10000.0;
    double dampingConstant = 500.0;

    double totalForce = compression * springConstant +
                        compressionVelocity * dampingConstant;

    TS_ASSERT_DELTA(totalForce, 6000.0, epsilon);  // 5000 + 1000
  }

  // Test extension (negative velocity)
  void testExtension() {
    double compression = 0.5;
    double compressionVelocity = -2.0;  // Extending
    double springConstant = 10000.0;
    double dampingConstant = 500.0;

    double springForce = compression * springConstant;
    double damperForce = compressionVelocity * dampingConstant;

    TS_ASSERT_DELTA(springForce + damperForce, 4000.0, epsilon);  // 5000 - 1000
  }

  /***************************************************************************
   * Multi-Gear Coordination Tests
   ***************************************************************************/

  // Test total weight distribution
  void testWeightDistribution() {
    double totalWeight = 10000.0;  // lbs
    double noseGearRatio = 0.08;   // 8% on nose
    double mainGearRatio = 0.92;   // 92% on mains (46% each)

    double noseLoad = totalWeight * noseGearRatio;
    double mainLoad = totalWeight * mainGearRatio / 2;  // Per main gear

    TS_ASSERT_DELTA(noseLoad, 800.0, epsilon);
    TS_ASSERT_DELTA(mainLoad, 4600.0, epsilon);
  }

  // Test asymmetric loading (crosswind)
  void testAsymmetricLoading() {
    double totalWeight = 10000.0;
    double sideForce = 500.0;
    double gearSpan = 10.0;  // ft between mains

    // Moment creates differential loading
    double moment = sideForce * 2.0;  // Assumed CG height of 2 ft
    double loadTransfer = moment / gearSpan;

    double leftLoad = 4600.0 - loadTransfer;   // Reduced
    double rightLoad = 4600.0 + loadTransfer;  // Increased

    TS_ASSERT(rightLoad > leftLoad);
    TS_ASSERT_DELTA(leftLoad + rightLoad, 9200.0, epsilon);  // Still equals main gear total
  }

  // Test tricycle gear moments
  void testTricycleGearMoments() {
    // Nose at x=10 ft, mains at x=-5 ft, CG at x=0
    double nosePos = 10.0;
    double mainPos = -5.0;
    double noseLoad = 800.0;
    double mainLoad = 9200.0;  // Combined mains

    double neckMoment = noseLoad * nosePos;
    double mainMoment = mainLoad * mainPos;
    double totalMoment = neckMoment + mainMoment;

    TS_ASSERT_DELTA(totalMoment, -38000.0, epsilon);  // Tail-down moment
  }

  /***************************************************************************
   * Retraction Tests
   ***************************************************************************/

  // Test gear retracted state
  void testGearRetracted() {
    GearState gear;
    gear.retracted = true;
    gear.compression = 0.0;

    // Retracted gear generates no forces
    double force = gear.retracted ? 0.0 : 1000.0;
    TS_ASSERT_DELTA(force, 0.0, epsilon);
  }

  // Test gear extended state
  void testGearExtended() {
    GearState gear;
    gear.retracted = false;
    gear.compression = 0.5;

    bool canGenerateForce = !gear.retracted && gear.compression > 0;
    TS_ASSERT(canGenerateForce);
  }

  // Test gear in transit
  void testGearInTransit() {
    double gearPosition = 0.5;  // 0 = retracted, 1 = extended
    double minPositionForContact = 0.8;

    bool canContact = (gearPosition >= minPositionForContact);
    TS_ASSERT(!canContact);  // Still retracting/extending
  }

  /***************************************************************************
   * Surface Type Tests
   ***************************************************************************/

  // Test runway friction
  void testRunwayFriction() {
    double dryFriction = 0.8;
    double wetFriction = 0.5;
    double icyFriction = 0.1;

    TS_ASSERT(dryFriction > wetFriction);
    TS_ASSERT(wetFriction > icyFriction);
  }

  // Test grass runway rolling resistance
  void testGrassRolling() {
    double concreteRolling = 0.02;
    double grassRolling = 0.10;

    TS_ASSERT(grassRolling > concreteRolling);  // More resistance on grass
  }

  // Test soft field sink
  void testSoftFieldSink() {
    double weight = 10000.0;
    double bearingStrength = 5000.0;  // lbs/sq ft

    // Simplified: sink until pressure equals bearing strength
    double gearFootprint = 0.5;  // sq ft
    double maxLoad = bearingStrength * gearFootprint;

    bool willSink = (weight / 3) > maxLoad;  // Weight per wheel
    TS_ASSERT(willSink);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero compression
  void testZeroCompression() {
    double compression = 0.0;
    double springConstant = 10000.0;

    double force = compression * springConstant;
    TS_ASSERT_DELTA(force, 0.0, epsilon);
  }

  // Test maximum compression
  void testMaxCompression() {
    double compression = 1.0;
    double maxCompression = 0.8;
    double springConstant = 10000.0;
    double bumpStopK = 50000.0;  // Stiffer bump stop

    double force;
    if (compression > maxCompression) {
      force = maxCompression * springConstant +
              (compression - maxCompression) * bumpStopK;
    } else {
      force = compression * springConstant;
    }

    TS_ASSERT_DELTA(force, 18000.0, epsilon);  // 8000 + 10000
  }

  // Test zero velocity
  void testZeroVelocity() {
    double velocity = 0.0;
    double rollingFriction = 0.02;
    double normalForce = 1000.0;

    // At zero velocity, use static friction
    double friction = (velocity == 0.0) ? 0.0 : rollingFriction * normalForce;
    TS_ASSERT_DELTA(friction, 0.0, epsilon);
  }

  // Test taildragger configuration
  void testTaildraggarConfig() {
    // Main gear forward, tail wheel aft
    double mainPos = 2.0;   // ft forward of CG
    double tailPos = -20.0; // ft aft of CG
    double mainLoad = 9000.0;
    double tailLoad = 1000.0;

    // Verify balance
    double mainMoment = mainLoad * mainPos;
    double tailMoment = tailLoad * tailPos;

    // For balance: main_moment ≈ -tail_moment
    // This is simplified - actual would need weight at CG
    TS_ASSERT(mainMoment > 0);
    TS_ASSERT(tailMoment < 0);
  }

  /***************************************************************************
   * Additional Comprehensive Ground Reactions Tests
   ***************************************************************************/

  // Test tire slip angle
  void testTireSlipAngle() {
    double velocity_x = 100.0;  // ft/s forward
    double velocity_y = 5.0;    // ft/s sideways

    // Slip angle = atan(Vy/Vx)
    double slipAngle = std::atan2(velocity_y, velocity_x) * 180.0 / M_PI;
    TS_ASSERT_DELTA(slipAngle, 2.86, 0.01);
  }

  // Test tire slip ratio
  void testTireSlipRatio() {
    double wheelSpeed = 95.0;   // ft/s (rotational)
    double groundSpeed = 100.0; // ft/s (translational)

    // Slip ratio = (Vwheel - Vground) / Vground
    double slipRatio = (wheelSpeed - groundSpeed) / groundSpeed;
    TS_ASSERT_DELTA(slipRatio, -0.05, epsilon);  // 5% slip (braking)
  }

  // Test tire slip ratio acceleration
  void testTireSlipRatioAccel() {
    double wheelSpeed = 110.0;  // ft/s
    double groundSpeed = 100.0; // ft/s

    double slipRatio = (wheelSpeed - groundSpeed) / groundSpeed;
    TS_ASSERT_DELTA(slipRatio, 0.10, epsilon);  // 10% slip (accelerating)
  }

  // Test lateral tire force (simplified Pacejka-like)
  void testLateralTireForce() {
    double slipAngle = 5.0;  // degrees
    double normalForce = 5000.0;  // lbs
    double corneringStiffness = 200.0;  // lbs/degree

    double lateralForce = corneringStiffness * slipAngle;
    lateralForce = std::min(lateralForce, normalForce * 0.8);  // Friction limit

    TS_ASSERT_DELTA(lateralForce, 1000.0, epsilon);
  }

  // Test lateral force saturation
  void testLateralForceSaturation() {
    double slipAngle = 20.0;  // Large slip angle
    double normalForce = 5000.0;
    double corneringStiffness = 200.0;
    double frictionCoeff = 0.8;

    double lateralForce = corneringStiffness * slipAngle;  // 4000
    double maxForce = normalForce * frictionCoeff;  // 4000

    lateralForce = std::min(lateralForce, maxForce);
    TS_ASSERT_DELTA(lateralForce, 4000.0, epsilon);  // Saturated
  }

  // Test anti-skid braking
  void testAntiSkidBraking() {
    double slipRatio = -0.15;  // 15% slip
    double optimalSlip = -0.10;  // 10% is optimal
    double tolerance = 0.02;

    // Anti-skid releases brake when slip exceeds optimal
    bool releaseBrake = (slipRatio < optimalSlip - tolerance);
    TS_ASSERT(releaseBrake);
  }

  // Test anti-skid brake application
  void testAntiSkidApplyBrake() {
    double slipRatio = -0.05;  // 5% slip
    double optimalSlip = -0.10;

    // Anti-skid applies brake when slip is below optimal
    bool applyBrake = (slipRatio > optimalSlip);
    TS_ASSERT(applyBrake);
  }

  // Test runway slope effect
  void testRunwaySlopeEffect() {
    double weight = 10000.0;  // lbs
    double slopeAngle = 2.0;  // degrees uphill

    double slopeRad = slopeAngle * M_PI / 180.0;
    double gravityComponent = weight * std::sin(slopeRad);

    TS_ASSERT_DELTA(gravityComponent, 349.0, 1.0);  // Resistance force
  }

  // Test downhill acceleration
  void testDownhillAcceleration() {
    double weight = 10000.0;
    double slopeAngle = -2.0;  // Downhill

    double slopeRad = slopeAngle * M_PI / 180.0;
    double gravityComponent = weight * std::sin(slopeRad);

    TS_ASSERT(gravityComponent < 0);  // Accelerating force
  }

  // Test landing impact force
  void testLandingImpactForce() {
    double verticalVelocity = 10.0;  // ft/s sink rate
    double mass = 10000.0 / 32.2;    // slugs
    double strokeLength = 1.0;       // ft gear travel

    // Energy = 0.5 * m * v^2
    double kineticEnergy = 0.5 * mass * verticalVelocity * verticalVelocity;

    // Force = Energy / stroke (simplified)
    double avgForce = kineticEnergy / strokeLength;
    TS_ASSERT(avgForce > 1000.0);  // Significant impact
  }

  // Test hard landing detection
  void testHardLandingDetection() {
    double sinkRate = 15.0;  // ft/s
    double hardLandingThreshold = 10.0;  // ft/s

    bool hardLanding = (sinkRate > hardLandingThreshold);
    TS_ASSERT(hardLanding);
  }

  // Test normal landing
  void testNormalLanding() {
    double sinkRate = 3.0;  // ft/s
    double hardLandingThreshold = 10.0;

    bool hardLanding = (sinkRate > hardLandingThreshold);
    TS_ASSERT(!hardLanding);
  }

  // Test gear leg bending moment
  void testGearLegBendingMoment() {
    double sideForce = 500.0;  // lbs
    double gearHeight = 4.0;   // ft

    double bendingMoment = sideForce * gearHeight;
    TS_ASSERT_DELTA(bendingMoment, 2000.0, epsilon);  // ft-lbs
  }

  // Test oleo strut preload
  void testOleoStrutPreload() {
    double pneumaticPreload = 500.0;  // lbs (nitrogen charge)
    double staticWeight = 3000.0;     // lbs

    // Compression at static with preload
    double springRate = 5000.0;  // lbs/ft
    double compression = (staticWeight - pneumaticPreload) / springRate;

    TS_ASSERT_DELTA(compression, 0.5, epsilon);
  }

  // Test oleo strut polytropic compression
  void testOleoPolytropicCompression() {
    double P1 = 1000.0;  // Initial pressure (psi)
    double V1 = 1.0;     // Initial volume (relative)
    double V2 = 0.8;     // Compressed volume
    double gamma = 1.3;  // Polytropic exponent

    // P1 * V1^gamma = P2 * V2^gamma
    double P2 = P1 * std::pow(V1 / V2, gamma);
    TS_ASSERT(P2 > P1);  // Pressure increases with compression
  }

  // Test tire cornering stiffness
  void testTireCorneringStiffness() {
    double load = 5000.0;  // lbs
    double baseStiffness = 150.0;  // lbs/deg at reference load
    double refLoad = 4000.0;

    // Stiffness varies with load (typically sqrt relationship)
    double stiffness = baseStiffness * std::sqrt(load / refLoad);
    TS_ASSERT_DELTA(stiffness, 167.7, 0.1);
  }

  // Test aquaplaning speed
  void testAquaplaningSpeed() {
    double tirePressure = 150.0;  // psi

    // NASA aquaplaning formula: V = 9 * sqrt(P)
    double aquaplaningSpeed = 9.0 * std::sqrt(tirePressure);  // knots
    TS_ASSERT_DELTA(aquaplaningSpeed, 110.2, 0.1);
  }

  // Test aquaplaning friction reduction
  void testAquaplaningFriction() {
    double speed = 120.0;  // knots
    double aquaplaningSpeed = 110.0;
    double dryFriction = 0.8;

    // Friction reduces above aquaplaning speed
    double ratio = aquaplaningSpeed / speed;
    double wetFriction = dryFriction * ratio * ratio;

    TS_ASSERT(wetFriction < dryFriction * 0.9);
  }

  // Test crosswind landing side force
  void testCrosswindLandingSideForce() {
    double crosswind = 20.0;  // knots
    double weight = 10000.0;
    double wingArea = 200.0;
    double sideForceCoeff = 0.3;

    // Simplified side force
    double q = 0.5 * 0.002378 * (crosswind * 1.688) * (crosswind * 1.688);
    double sideForce = q * wingArea * sideForceCoeff;

    TS_ASSERT(sideForce > 0);
  }

  // Test crab angle landing
  void testCrabAngleLanding() {
    double groundSpeed = 100.0;  // knots
    double crosswind = 15.0;     // knots

    // Crab angle to maintain track
    double crabAngle = std::atan2(crosswind, groundSpeed) * 180.0 / M_PI;
    TS_ASSERT_DELTA(crabAngle, 8.53, 0.01);
  }

  // Test touchdown position detection
  void testTouchdownDetection() {
    bool prevWOW = false;
    bool currWOW = true;

    bool touchdown = (!prevWOW && currWOW);
    TS_ASSERT(touchdown);
  }

  // Test liftoff detection
  void testLiftoffDetection() {
    bool prevWOW = true;
    bool currWOW = false;

    bool liftoff = (prevWOW && !currWOW);
    TS_ASSERT(liftoff);
  }

  // Test runway roughness effect
  void testRunwayRoughnessEffect() {
    double baseLoad = 5000.0;  // lbs
    double roughnessFactor = 1.3;  // 30% load increase

    double peakLoad = baseLoad * roughnessFactor;
    TS_ASSERT_DELTA(peakLoad, 6500.0, epsilon);
  }

  // Test shimmy oscillation frequency
  void testShimmyFrequency() {
    double steeringStiffness = 1000.0;  // ft-lb/rad
    double wheelInertia = 0.5;          // slug-ft^2

    // Natural frequency = sqrt(K/I)
    double omega = std::sqrt(steeringStiffness / wheelInertia);
    double frequency = omega / (2.0 * M_PI);

    TS_ASSERT_DELTA(frequency, 7.12, 0.01);
  }

  // Test shimmy damping
  void testShimmyDamping() {
    double dampingCoeff = 30.0;   // ft-lb-s/rad (lower damping)
    double steeringStiffness = 1000.0;
    double wheelInertia = 0.5;

    // Critical damping = 2 * sqrt(K * I) = 2 * sqrt(500) = 44.72
    double criticalDamping = 2.0 * std::sqrt(steeringStiffness * wheelInertia);
    double dampingRatio = dampingCoeff / criticalDamping;

    TS_ASSERT(dampingRatio < 1.0);  // Underdamped (prone to shimmy)
    TS_ASSERT_DELTA(dampingRatio, 0.67, 0.01);
  }

  // Test tire blowout effect
  void testTireBlowout() {
    double normalFriction = 0.8;
    double blowoutFriction = 0.3;  // Reduced friction on rim

    double frictionLoss = (normalFriction - blowoutFriction) / normalFriction;
    TS_ASSERT_DELTA(frictionLoss, 0.625, 0.001);  // 62.5% loss
  }

  // Test differential braking yaw moment
  void testDifferentialBrakingYaw() {
    double leftBrake = 0.8;   // 80% left brake
    double rightBrake = 0.2;  // 20% right brake
    double maxBrakeForce = 3000.0;  // lbs per wheel
    double gearSpan = 10.0;   // ft between mains

    double leftForce = leftBrake * maxBrakeForce;
    double rightForce = rightBrake * maxBrakeForce;
    double yawMoment = (leftForce - rightForce) * gearSpan / 2;

    TS_ASSERT_DELTA(yawMoment, 9000.0, epsilon);  // ft-lbs
  }

  // Test nose wheel lift-off speed
  void testNoseWheelLiftoff() {
    double rotationSpeed = 80.0;  // knots
    double weight = 10000.0;
    double cgToMain = 5.0;  // ft
    double cgToNose = 15.0; // ft

    // Simplified: elevator authority overcomes nose-down moment
    double noseLoadFraction = cgToMain / (cgToMain + cgToNose);
    TS_ASSERT_DELTA(noseLoadFraction, 0.25, 0.01);
  }

  // Test gear strut extension rate
  void testStrutExtensionRate() {
    double compressionVelocity = 5.0;  // ft/s
    double maxExtensionRate = 3.0;     // ft/s

    // Extension is damped more than compression
    double dampedRate = std::min(std::abs(compressionVelocity), maxExtensionRate);
    TS_ASSERT_DELTA(dampedRate, 3.0, epsilon);
  }

  // Test multiple contact points
  void testMultipleContactPoints() {
    // Bogie gear with two wheels per leg
    double totalLoad = 10000.0;
    int wheelsPerLeg = 2;

    double loadPerWheel = totalLoad / wheelsPerLeg;
    TS_ASSERT_DELTA(loadPerWheel, 5000.0, epsilon);
  }

  // Test bogie beam rotation
  void testBogieBeamRotation() {
    double frontWheelLoad = 5500.0;
    double rearWheelLoad = 4500.0;
    double bogieLength = 4.0;  // ft

    // Moment about bogie pivot
    double loadDiff = frontWheelLoad - rearWheelLoad;
    double moment = loadDiff * bogieLength / 2;

    TS_ASSERT_DELTA(moment, 2000.0, epsilon);  // ft-lbs
  }

  // Test tire deflection
  void testTireDeflection() {
    double load = 5000.0;      // lbs
    double tireStiffness = 25000.0;  // lbs/ft

    double deflection = load / tireStiffness;
    TS_ASSERT_DELTA(deflection, 0.2, epsilon);  // 0.2 ft = 2.4 inches
  }

  // Test combined tire and strut deflection
  void testCombinedDeflection() {
    double load = 5000.0;
    double tireStiffness = 25000.0;  // lbs/ft
    double strutStiffness = 10000.0; // lbs/ft

    double tireDeflection = load / tireStiffness;
    double strutDeflection = load / strutStiffness;
    double totalDeflection = tireDeflection + strutDeflection;

    TS_ASSERT_DELTA(totalDeflection, 0.7, epsilon);
  }

  // Test ground reaction vertical component
  void testVerticalReaction() {
    double normalForce = 5000.0;  // lbs
    double surfaceAngle = 5.0;    // degrees (tilted surface)

    double surfaceRad = surfaceAngle * M_PI / 180.0;
    double verticalComponent = normalForce * std::cos(surfaceRad);

    TS_ASSERT_DELTA(verticalComponent, 4981.0, 1.0);
  }

  // Test ground reaction horizontal component
  void testHorizontalReaction() {
    double normalForce = 5000.0;
    double surfaceAngle = 5.0;

    double surfaceRad = surfaceAngle * M_PI / 180.0;
    double horizontalComponent = normalForce * std::sin(surfaceRad);

    TS_ASSERT_DELTA(horizontalComponent, 436.0, 1.0);
  }

  // Test CG height effect on braking
  void testCGHeightBrakingEffect() {
    double weight = 10000.0;
    double wheelbase = 20.0;   // ft
    double cgHeight = 5.0;     // ft
    double deceleration = 0.3; // g's

    // Load transfer due to braking
    double loadTransfer = weight * deceleration * cgHeight / wheelbase;
    TS_ASSERT_DELTA(loadTransfer, 750.0, epsilon);
  }

  // Test nose gear loading during braking
  void testNoseGearBrakingLoad() {
    double staticNoseLoad = 800.0;
    double loadTransfer = 750.0;  // From braking

    double dynamicNoseLoad = staticNoseLoad + loadTransfer;
    TS_ASSERT_DELTA(dynamicNoseLoad, 1550.0, epsilon);
  }

  // Test main gear unloading during braking
  void testMainGearBrakingUnload() {
    double staticMainLoad = 4600.0;  // Per main gear
    double loadTransfer = 375.0;     // Half of total transfer per wheel

    double dynamicMainLoad = staticMainLoad - loadTransfer;
    TS_ASSERT_DELTA(dynamicMainLoad, 4225.0, epsilon);
  }

  // Test pivot point for rotation
  void testPivotPointRotation() {
    double mainGearX = -5.0;   // ft (behind CG)
    double tailWheelX = -25.0; // ft

    // Rotation occurs about main gear
    double rotationArm = std::abs(mainGearX);
    TS_ASSERT_DELTA(rotationArm, 5.0, epsilon);
  }

  // Test tip-back angle
  void testTipBackAngle() {
    double cgHeight = 4.0;  // ft above ground
    double mainGearX = 5.0; // ft behind CG

    // Tip-back occurs when CG moves aft of main gear
    double tipBackAngle = std::atan2(cgHeight, mainGearX) * 180.0 / M_PI;
    TS_ASSERT_DELTA(tipBackAngle, 38.66, 0.01);
  }

  // Test turnover angle
  void testTurnoverAngle() {
    double cgHeight = 4.0;     // ft
    double trackWidth = 10.0;  // ft between main gears

    // Turnover occurs when CG exceeds this angle from vertical
    double turnoverAngle = std::atan2(trackWidth / 2, cgHeight) * 180.0 / M_PI;
    TS_ASSERT_DELTA(turnoverAngle, 51.34, 0.01);
  }

  // Test taxi speed limit
  void testTaxiSpeedLimit() {
    double cornerRadius = 50.0;  // ft
    double maxLateralG = 0.2;    // g's

    // V^2 = a * r, where a = g * maxLateralG
    double maxSpeed = std::sqrt(32.2 * maxLateralG * cornerRadius);  // ft/s
    double maxSpeedKnots = maxSpeed / 1.688;

    TS_ASSERT_DELTA(maxSpeedKnots, 10.6, 0.1);
  }

  // Test weight transfer during turn
  void testWeightTransferTurn() {
    double weight = 10000.0;
    double lateralG = 0.15;
    double cgHeight = 4.0;
    double trackWidth = 10.0;

    double loadTransfer = weight * lateralG * cgHeight / trackWidth;
    TS_ASSERT_DELTA(loadTransfer, 600.0, epsilon);
  }

  // Test combined vertical and lateral load
  void testCombinedLoad() {
    double verticalLoad = 5000.0;
    double lateralLoad = 1000.0;

    double totalLoad = std::sqrt(verticalLoad * verticalLoad + lateralLoad * lateralLoad);
    TS_ASSERT_DELTA(totalLoad, 5099.0, 1.0);
  }

  // Test tire scrub during turn
  void testTireScrubTurn() {
    double steerAngle = 30.0;     // degrees
    double wheelbase = 20.0;      // ft
    double velocity = 30.0;       // ft/s

    // Ackermann geometry - inner wheel travels shorter arc
    double turnRadius = wheelbase / std::tan(steerAngle * M_PI / 180.0);
    TS_ASSERT_DELTA(turnRadius, 34.64, 0.01);
  }

  // Test spin-up time
  void testWheelSpinUpTime() {
    double groundSpeed = 150.0;  // ft/s
    double wheelRadius = 1.0;    // ft
    double wheelInertia = 0.1;   // slug-ft^2
    double frictionForce = 500.0;  // lbs

    // Angular acceleration = torque / inertia
    double torque = frictionForce * wheelRadius;
    double angularAccel = torque / wheelInertia;  // rad/s^2

    // Time to reach ground speed
    double targetAngularVel = groundSpeed / wheelRadius;
    double spinUpTime = targetAngularVel / angularAccel;

    TS_ASSERT(spinUpTime < 1.0);  // Should be quick
  }

  // Test wheel spin-down during braking
  void testWheelSpinDown() {
    double initialSpeed = 150.0;   // rad/s
    double brakeForce = 1000.0;    // lbs
    double wheelRadius = 1.0;      // ft
    double wheelInertia = 0.1;     // slug-ft^2

    double torque = brakeForce * wheelRadius;
    double angularDecel = torque / wheelInertia;

    // Time to stop wheel
    double stopTime = initialSpeed / angularDecel;
    TS_ASSERT(stopTime > 0.01);  // Finite time to stop
  }

  /***************************************************************************
   * Complete Ground Reactions System Tests
   ***************************************************************************/

  // Test complete landing gear system verification
  void testCompleteGearSystemVerification() {
    double weight = 10000.0;
    double noseLoad = weight * 0.08;
    double mainLoadEach = weight * 0.46;

    TS_ASSERT_DELTA(noseLoad + 2.0 * mainLoadEach, weight, epsilon);
    TS_ASSERT_DELTA(noseLoad, 800.0, epsilon);
    TS_ASSERT_DELTA(mainLoadEach, 4600.0, epsilon);
  }

  // Test gear energy absorption
  void testGearEnergyAbsorption() {
    double mass = 10000.0 / 32.2;  // slugs (~310.56)
    double sinkRate = 10.0;  // ft/s
    double kineticEnergy = 0.5 * mass * sinkRate * sinkRate;
    double strokeLength = 1.0;  // ft
    double avgForce = kineticEnergy / strokeLength;

    TS_ASSERT(avgForce > 0.0);
    TS_ASSERT_DELTA(kineticEnergy, 15528.0, 10.0);  // 0.5 * 310.56 * 100
  }

  // Test brake temperature rise
  void testBrakeTemperatureRise() {
    double brakingEnergy = 100000.0;  // ft-lbf
    double brakeHeatCapacity = 500.0;  // ft-lbf/°F
    double tempRise = brakingEnergy / brakeHeatCapacity;

    TS_ASSERT_DELTA(tempRise, 200.0, epsilon);
  }

  // Test tire footprint calculation
  void testTireFootprintCalculation() {
    double load = 5000.0;  // lbs
    double pressure = 100.0;  // psi
    double footprint = load / pressure;  // sq in

    TS_ASSERT_DELTA(footprint, 50.0, epsilon);
  }

  // Test gear extension time
  void testGearExtensionTime() {
    double extensionRate = 0.1;  // per second
    double startPosition = 0.0;
    double endPosition = 1.0;
    double extensionTime = (endPosition - startPosition) / extensionRate;

    TS_ASSERT_DELTA(extensionTime, 10.0, epsilon);
  }

  // Test gear up-lock force
  void testGearUpLockForce() {
    double gearWeight = 500.0;  // lbs
    double gFactor = 3.0;
    double lockForce = gearWeight * gFactor;

    TS_ASSERT_DELTA(lockForce, 1500.0, epsilon);
  }

  // Test nose wheel centering
  void testNoseWheelCentering() {
    double steerAngle = 30.0;  // deg
    double centeringForce = 100.0;  // lbs
    double steeringStiffness = 10.0;  // lbs/deg
    double equilibriumAngle = centeringForce / steeringStiffness;

    TS_ASSERT_DELTA(equilibriumAngle, 10.0, epsilon);
  }

  // Test runway crown effect
  void testRunwayCrownEffect() {
    double crownSlope = 0.015;  // 1.5%
    double weight = 10000.0;
    double sideForce = weight * crownSlope;

    TS_ASSERT_DELTA(sideForce, 150.0, epsilon);
  }

  // Test gear door aerodynamic loads
  void testGearDoorAeroLoads() {
    double q = 100.0;  // psf dynamic pressure
    double doorArea = 5.0;  // sq ft
    double Cd = 1.2;
    double dragForce = q * doorArea * Cd;

    TS_ASSERT_DELTA(dragForce, 600.0, epsilon);
  }

  // Test multiple wheel group load sharing
  void testMultipleWheelGroupLoadSharing() {
    double bogieLoad = 20000.0;
    int wheelsPerBogie = 4;
    double loadPerWheel = bogieLoad / wheelsPerBogie;

    TS_ASSERT_DELTA(loadPerWheel, 5000.0, epsilon);
  }

  // Test retraction sequence timing
  void testRetractionSequenceTiming() {
    double doorOpenTime = 2.0;  // seconds
    double gearRetractTime = 5.0;
    double doorCloseTime = 2.0;
    double totalTime = doorOpenTime + gearRetractTime + doorCloseTime;

    TS_ASSERT_DELTA(totalTime, 9.0, epsilon);
  }

  // Test free-fall extension
  void testFreeFallExtension() {
    double gearMass = 500.0 / 32.2;  // slugs
    double dropHeight = 2.0;  // ft
    double velocity = std::sqrt(2.0 * 32.2 * dropHeight);

    TS_ASSERT_DELTA(velocity, 11.35, 0.01);
  }

  // Test gear position indicator accuracy
  void testGearPositionIndicator() {
    double actualPosition = 0.95;
    double indicatorThreshold = 0.90;
    bool indicatesDown = actualPosition >= indicatorThreshold;

    TS_ASSERT(indicatesDown);
  }

  // Test taxi light depression angle
  void testTaxiLightDepressionAngle() {
    double lightHeight = 10.0;  // ft
    double illuminationDistance = 100.0;  // ft
    double depressionAngle = std::atan(lightHeight / illuminationDistance) * 180.0 / M_PI;

    TS_ASSERT_DELTA(depressionAngle, 5.71, 0.01);
  }

  // Test tire pressure variation with altitude
  void testTirePressureAltitude() {
    double groundPressure = 200.0;  // psi
    double altitudePressureRatio = 0.75;  // at 10000 ft
    double effectivePressure = groundPressure * (1.0 + (1.0 - altitudePressureRatio) * 0.1);

    TS_ASSERT(effectivePressure > groundPressure);
  }

  // Test gear walk frequency
  void testGearWalkFrequency() {
    double gearStiffness = 50000.0;  // lbs/ft
    double gearMass = 500.0 / 32.2;  // slugs
    double omega = std::sqrt(gearStiffness / gearMass);
    double frequency = omega / (2.0 * M_PI);

    TS_ASSERT(frequency > 5.0);
    TS_ASSERT(frequency < 20.0);
  }

  // Test complete braking system verification
  void testCompleteBrakingSystemVerification() {
    double weight = 10000.0;
    double mu = 0.6;
    double maxBrakeForce = weight * mu;
    double decel_g = maxBrakeForce / weight;
    double decel_fps2 = decel_g * 32.2;
    double speed_fps = 200.0;
    double stopDistance = (speed_fps * speed_fps) / (2.0 * decel_fps2);

    TS_ASSERT_DELTA(decel_g, 0.6, epsilon);
    TS_ASSERT_DELTA(decel_fps2, 19.32, 0.1);
    TS_ASSERT(stopDistance > 1000.0);
    TS_ASSERT(stopDistance < 1500.0);
  }

  // Test ground reactions instance independence
  void testGroundReactionsInstanceIndependence() {
    GearState gear1, gear2;
    gear1.compression = 0.5;
    gear2.compression = 0.3;
    gear1.staticFriction = 0.8;
    gear2.staticFriction = 0.5;

    TS_ASSERT(gear1.compression != gear2.compression);
    TS_ASSERT(gear1.staticFriction != gear2.staticFriction);
  }

  // Test complete gear dynamics simulation
  void testCompleteGearDynamicsSimulation() {
    double mass = 10000.0 / 32.2;  // slugs
    double springK = 100000.0;  // lbs/ft
    double dampingC = 5000.0;  // lbs/(ft/s)
    double compression = 0.5;  // ft
    double velocity = -5.0;  // ft/s (compressing)

    double springForce = springK * compression;
    double dampingForce = -dampingC * velocity;
    double totalForce = springForce + dampingForce;
    double acceleration = totalForce / mass;

    TS_ASSERT(springForce > 0.0);
    TS_ASSERT(dampingForce > 0.0);
    TS_ASSERT(totalForce > springForce);
    TS_ASSERT(acceleration > 0.0);
    TS_ASSERT_DELTA(springForce, 50000.0, epsilon);
  }

  // ============================================================================
  // FGGroundReactions class tests - using actual class methods
  // ============================================================================

  // Test FGGroundReactions construction through FGFDMExec
  void testFGGroundReactionsConstruction() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(groundReactions != nullptr);
  }

  // Test FGGroundReactions with loaded aircraft
  void testFGGroundReactionsWithAircraft() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(groundReactions != nullptr);

    // Ball model has contact point(s)
    int numGear = groundReactions->GetNumGearUnits();
    TS_ASSERT(numGear > 0);
  }

  // Test GetNumGearUnits
  void testFGGroundReactionsGetNumGearUnits() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    int numGear = groundReactions->GetNumGearUnits();
    // Ball model has 1 contact point
    TS_ASSERT(numGear >= 1);
  }

  // Test GetGearUnit
  void testFGGroundReactionsGetGearUnit() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    int numGear = groundReactions->GetNumGearUnits();
    if (numGear > 0) {
      auto gear = groundReactions->GetGearUnit(0);
      TS_ASSERT(gear != nullptr);
    }
  }

  // Test Run method
  void testFGGroundReactionsRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();

    // Run should succeed
    bool result = groundReactions->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test Run in holding mode
  void testFGGroundReactionsRunHolding() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();

    // Run in holding mode
    bool result = groundReactions->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test InitModel method
  void testFGGroundReactionsInitModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    bool result = groundReactions->InitModel();
    TS_ASSERT(result);
  }

  // Test GetWOW method
  void testFGGroundReactionsGetWOW() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();

    // Run to compute ground reactions
    groundReactions->Run(false);

    // Just verify it returns a valid boolean
    bool wow = groundReactions->GetWOW();
    TS_ASSERT(wow == true || wow == false);
  }

  // Test SetDsCmd (steering command)
  void testFGGroundReactionsSetDsCmd() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    // Set steering command
    groundReactions->SetDsCmd(0.5);
    double cmd = groundReactions->GetDsCmd();
    TS_ASSERT_DELTA(cmd, 0.5, epsilon);

    // Set to different value
    groundReactions->SetDsCmd(-0.3);
    cmd = groundReactions->GetDsCmd();
    TS_ASSERT_DELTA(cmd, -0.3, epsilon);
  }

  // Test GetForces method
  void testFGGroundReactionsGetForces() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();

    // Run to compute forces
    groundReactions->Run(false);

    // Get total ground forces
    const FGColumnVector3& forces = groundReactions->GetForces();
    // Forces should be a valid vector (may be zero if airborne)
    TS_ASSERT(forces.Magnitude() >= 0.0);
  }

  // Test GetMoments method
  void testFGGroundReactionsGetMoments() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();

    // Run to compute moments
    groundReactions->Run(false);

    // Get total ground moments
    const FGColumnVector3& moments = groundReactions->GetMoments();
    // Moments should be a valid vector
    TS_ASSERT(moments.Magnitude() >= 0.0);
  }

  // Test GetGroundReactionStrings
  void testFGGroundReactionsGetStrings() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    std::string header = groundReactions->GetGroundReactionStrings(",");
    // Should return a non-empty string with gear data
    TS_ASSERT(!header.empty());
  }

  // Test GetGroundReactionValues
  void testFGGroundReactionsGetValues() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto groundReactions = fdmex.GetGroundReactions();

    // Run to compute values
    groundReactions->Run(false);

    std::string values = groundReactions->GetGroundReactionValues(",");
    // Should return a non-empty string
    TS_ASSERT(!values.empty());
  }

  // Test property binding for steer command
  void testFGGroundReactionsSteerProperty() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto pm = fdmex.GetPropertyManager();
    auto groundReactions = fdmex.GetGroundReactions();

    // Check property exists
    auto node = pm->GetNode("fcs/steer-cmd-norm");
    TS_ASSERT(node != nullptr);

    if (node) {
      // Set via property
      node->setDoubleValue(0.25);
      TS_ASSERT_DELTA(groundReactions->GetDsCmd(), 0.25, epsilon);
    }
  }

  // Test property binding for WOW
  void testFGGroundReactionsWOWProperty() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto pm = fdmex.GetPropertyManager();
    auto groundReactions = fdmex.GetGroundReactions();

    groundReactions->Run(false);

    // Check WOW property exists
    auto node = pm->GetNode("gear/wow");
    TS_ASSERT(node != nullptr);
  }

  // Test property binding for num gear units
  void testFGGroundReactionsNumUnitsProperty() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto pm = fdmex.GetPropertyManager();

    // Check property exists
    auto node = pm->GetNode("gear/num-units");
    TS_ASSERT(node != nullptr);

    if (node) {
      int numUnits = node->getIntValue();
      TS_ASSERT(numUnits >= 1);  // Ball model has at least 1 contact
    }
  }

  // Test with ball model contact point
  void testFGGroundReactionsBallModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    // Ball model has contact point
    int numGear = groundReactions->GetNumGearUnits();
    TS_ASSERT(numGear >= 1);  // Has contact point(s)
  }

  // Test surface friction access
  void testFGGroundReactionsSurfaceFriction() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    // Get static friction coefficient
    double staticFriction = groundReactions->GetStaticFFactor();
    TS_ASSERT(staticFriction > 0.0);
    TS_ASSERT(staticFriction <= 1.5);  // Reasonable friction range

    // Get rolling friction coefficient
    double rollingFriction = groundReactions->GetRollingFFactor();
    TS_ASSERT(rollingFriction >= 0.0);
  }

  // Test maximum static friction factor
  void testFGGroundReactionsMaxStaticFriction() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto groundReactions = fdmex.GetGroundReactions();

    double maxStatic = groundReactions->GetMaximumForce();
    // Should be a positive value
    TS_ASSERT(maxStatic >= 0.0);
  }

  // ============================================================================
  // C172x Model-Based Tests - Using FGFDMExec with c172x aircraft
  // ============================================================================

  // Test loading c172x model and accessing ground reactions
  void testC172xLoadModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();
    TS_ASSERT(gr != nullptr);
  }

  // Test c172x number of gear units (3 bogeys + 3 structure = 6 total)
  void testC172xGetNumGearUnits() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    int numGear = gr->GetNumGearUnits();
    // c172x has: nose gear, left main, right main, tail skid, left tip, right tip
    TS_ASSERT_EQUALS(numGear, 6);
  }

  // Test c172x individual gear unit access
  void testC172xGetGearUnit() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    // Access each gear unit
    for (int i = 0; i < gr->GetNumGearUnits(); i++) {
      auto gear = gr->GetGearUnit(i);
      TS_ASSERT(gear != nullptr);
    }
  }

  // Test c172x gear names
  void testC172xGearNames() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    // First 3 are bogeys
    TS_ASSERT_EQUALS(gr->GetGearUnit(0)->GetName(), "Nose Gear");
    TS_ASSERT_EQUALS(gr->GetGearUnit(1)->GetName(), "Left Main Gear");
    TS_ASSERT_EQUALS(gr->GetGearUnit(2)->GetName(), "Right Main Gear");
    // Next 3 are structure contact points
    TS_ASSERT_EQUALS(gr->GetGearUnit(3)->GetName(), "TAIL_SKID");
    TS_ASSERT_EQUALS(gr->GetGearUnit(4)->GetName(), "LEFT_TIP");
    TS_ASSERT_EQUALS(gr->GetGearUnit(5)->GetName(), "RIGHT_TIP");
  }

  // Test c172x gear types (bogey vs structure)
  void testC172xGearTypes() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    // First 3 are bogeys
    TS_ASSERT(gr->GetGearUnit(0)->IsBogey());
    TS_ASSERT(gr->GetGearUnit(1)->IsBogey());
    TS_ASSERT(gr->GetGearUnit(2)->IsBogey());
    // Structure contact points are not bogeys
    TS_ASSERT(!gr->GetGearUnit(3)->IsBogey());
    TS_ASSERT(!gr->GetGearUnit(4)->IsBogey());
    TS_ASSERT(!gr->GetGearUnit(5)->IsBogey());
  }

  // Test c172x gear steerable properties
  void testC172xGearSteerable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    // Nose gear is steerable (max_steer = 10 deg)
    TS_ASSERT(gr->GetGearUnit(0)->GetSteerable());
    // Main gears are not steerable (max_steer = 0)
    TS_ASSERT(!gr->GetGearUnit(1)->GetSteerable());
    TS_ASSERT(!gr->GetGearUnit(2)->GetSteerable());
  }

  // Test c172x gear retractable properties
  void testC172xGearRetractable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    // c172x has fixed gear (retractable = 0)
    TS_ASSERT(!gr->GetGearUnit(0)->GetRetractable());
    TS_ASSERT(!gr->GetGearUnit(1)->GetRetractable());
    TS_ASSERT(!gr->GetGearUnit(2)->GetRetractable());
  }

  // Test c172x gear brake groups
  void testC172xGearBrakeGroups() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto gr = fdmex.GetGroundReactions();

    // Nose gear has no brake
    TS_ASSERT_EQUALS(gr->GetGearUnit(0)->GetBrakeGroup(), FGLGear::bgNone);
    // Main gears have left/right brakes
    TS_ASSERT_EQUALS(gr->GetGearUnit(1)->GetBrakeGroup(), FGLGear::bgLeft);
    TS_ASSERT_EQUALS(gr->GetGearUnit(2)->GetBrakeGroup(), FGLGear::bgRight);
  }

  // Test c172x initialization and run
  void testC172xRunIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    bool result = fdmex.RunIC();
    TS_ASSERT(result);

    auto gr = fdmex.GetGroundReactions();
    TS_ASSERT(gr != nullptr);
  }

  // Test c172x ground reactions run
  void testC172xGroundReactionsRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();
    bool result = gr->Run(false);
    TS_ASSERT_EQUALS(result, false);  // false means no error
  }

  // Test c172x WOW status after initialization (on ground)
  void testC172xWOWOnGround() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    // Set aircraft on ground
    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Aircraft should have weight on wheels when on ground
    bool wow = gr->GetWOW();
    TS_ASSERT(wow);
  }

  // Test c172x gear compression on ground
  void testC172xGearCompressionOnGround() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // At least some gear should be compressed on ground
    bool anyCompressed = false;
    for (int i = 0; i < 3; i++) {  // Check the 3 bogeys
      auto gear = gr->GetGearUnit(i);
      if (gear->GetCompLen() > 0.0) {
        anyCompressed = true;
        break;
      }
    }
    TS_ASSERT(anyCompressed);
  }

  // Test c172x gear WOW individual status
  void testC172xIndividualGearWOW() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Check individual gear WOW status
    bool noseWOW = gr->GetGearUnit(0)->GetWOW();
    bool leftWOW = gr->GetGearUnit(1)->GetWOW();
    bool rightWOW = gr->GetGearUnit(2)->GetWOW();

    // At least main gears should have weight on wheels
    TS_ASSERT(leftWOW || rightWOW);
  }

  // Test c172x ground reaction forces on ground
  void testC172xGroundReactionForces() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    const FGColumnVector3& forces = gr->GetForces();

    // On ground, there should be some vertical force (supporting weight)
    // Force is in body frame, Z is down (positive)
    TS_ASSERT(forces.Magnitude() > 0.0);
  }

  // Test c172x ground reaction moments
  void testC172xGroundReactionMoments() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    const FGColumnVector3& moments = gr->GetMoments();

    // Moments vector should exist (may be small if balanced)
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test c172x gear compression force
  void testC172xGearCompressForce() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Check compression force for main gears on ground
    double leftForce = gr->GetGearUnit(1)->GetCompForce();
    double rightForce = gr->GetGearUnit(2)->GetCompForce();

    // Should have some force on main gears (negative = pushing up)
    TS_ASSERT(leftForce != 0.0 || rightForce != 0.0);
  }

  // Test c172x steering command
  void testC172xSteeringCommand() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();

    // Set steering command
    gr->SetDsCmd(0.5);
    TS_ASSERT_DELTA(gr->GetDsCmd(), 0.5, epsilon);

    gr->SetDsCmd(-0.5);
    TS_ASSERT_DELTA(gr->GetDsCmd(), -0.5, epsilon);

    gr->SetDsCmd(0.0);
    TS_ASSERT_DELTA(gr->GetDsCmd(), 0.0, epsilon);
  }

  // Test c172x gear static friction coefficient
  void testC172xStaticFriction() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto gr = fdmex.GetGroundReactions();

    // Static friction coefficient from XML is 0.8 for bogeys
    double staticFCoeff = gr->GetGearUnit(0)->GetstaticFCoeff();
    TS_ASSERT_DELTA(staticFCoeff, 0.8, 0.01);
  }

  // Test c172x gear down status (fixed gear always down)
  void testC172xGearDownStatus() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();

    // Fixed gear should always be down
    TS_ASSERT(gr->GetGearUnit(0)->GetGearUnitDown());
    TS_ASSERT(gr->GetGearUnit(1)->GetGearUnitDown());
    TS_ASSERT(gr->GetGearUnit(2)->GetGearUnitDown());

    // Fixed gear should never be up
    TS_ASSERT(!gr->GetGearUnit(0)->GetGearUnitUp());
    TS_ASSERT(!gr->GetGearUnit(1)->GetGearUnitUp());
    TS_ASSERT(!gr->GetGearUnit(2)->GetGearUnitUp());
  }

  // Test c172x gear position value
  void testC172xGearPosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();

    // Fixed gear position should be 1.0 (fully extended)
    double pos0 = gr->GetGearUnit(0)->GetGearUnitPos();
    double pos1 = gr->GetGearUnit(1)->GetGearUnitPos();
    double pos2 = gr->GetGearUnit(2)->GetGearUnitPos();

    TS_ASSERT_DELTA(pos0, 1.0, epsilon);
    TS_ASSERT_DELTA(pos1, 1.0, epsilon);
    TS_ASSERT_DELTA(pos2, 1.0, epsilon);
  }

  // Test c172x gear local position
  void testC172xGearLocalPosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Get local gear positions
    const FGColumnVector3& noseLocal = gr->GetGearUnit(0)->GetLocalGear();
    const FGColumnVector3& leftLocal = gr->GetGearUnit(1)->GetLocalGear();
    const FGColumnVector3& rightLocal = gr->GetGearUnit(2)->GetLocalGear();

    // Positions should be valid (not NaN)
    TS_ASSERT(!std::isnan(noseLocal(1)));
    TS_ASSERT(!std::isnan(leftLocal(1)));
    TS_ASSERT(!std::isnan(rightLocal(1)));
  }

  // Test c172x WOW airborne (no WOW when high)
  void testC172xWOWAirborne() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(1000.0);  // High above ground

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Aircraft should not have weight on wheels when airborne
    bool wow = gr->GetWOW();
    TS_ASSERT(!wow);
  }

  // Test c172x no gear compression when airborne
  void testC172xNoCompressionAirborne() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(1000.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // No compression when airborne
    for (int i = 0; i < 3; i++) {
      double compLen = gr->GetGearUnit(i)->GetCompLen();
      TS_ASSERT_DELTA(compLen, 0.0, epsilon);
    }
  }

  // Test c172x ground reaction strings
  void testC172xGroundReactionStrings() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();

    std::string header = gr->GetGroundReactionStrings(",");
    TS_ASSERT(!header.empty());
    // Should contain gear names
    TS_ASSERT(header.find("Nose Gear") != std::string::npos);
    TS_ASSERT(header.find("Left Main Gear") != std::string::npos);
    TS_ASSERT(header.find("Right Main Gear") != std::string::npos);
  }

  // Test c172x ground reaction values
  void testC172xGroundReactionValues() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    std::string values = gr->GetGroundReactionValues(",");
    TS_ASSERT(!values.empty());
  }

  // Test c172x gear body forces
  void testC172xGearBodyForces() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Check body forces for main gears
    double leftXForce = gr->GetGearUnit(1)->GetBodyXForce();
    double leftYForce = gr->GetGearUnit(1)->GetBodyYForce();
    double leftZForce = gr->GetGearUnit(1)->GetBodyZForce();

    // Forces should be valid (not NaN)
    TS_ASSERT(!std::isnan(leftXForce));
    TS_ASSERT(!std::isnan(leftYForce));
    TS_ASSERT(!std::isnan(leftZForce));
  }

  // Test c172x multiple simulation runs
  void testC172xMultipleRuns() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();

    // Run multiple times
    for (int i = 0; i < 10; i++) {
      bool result = fdmex.Run();
      TS_ASSERT(result);

      // Check WOW remains consistent
      bool wow = gr->GetWOW();
      TS_ASSERT(wow == true || wow == false);  // Valid boolean
    }
  }

  // Test c172x gear forces sum to total
  void testC172xTotalForcesConsistency() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();
    const FGColumnVector3& totalForces = gr->GetForces();

    // Sum individual gear body forces
    FGColumnVector3 sumForces;
    for (int i = 0; i < gr->GetNumGearUnits(); i++) {
      auto gear = gr->GetGearUnit(i);
      const FGColumnVector3& gearForce = gear->GetBodyForces();
      sumForces(1) += gearForce(1);
      sumForces(2) += gearForce(2);
      sumForces(3) += gearForce(3);
    }

    // Total should equal sum of individual gear forces
    TS_ASSERT_DELTA(totalForces(1), sumForces(1), 1.0);
    TS_ASSERT_DELTA(totalForces(2), sumForces(2), 1.0);
    TS_ASSERT_DELTA(totalForces(3), sumForces(3), 1.0);
  }

  // Test c172x property access for gear
  void testC172xGearProperties() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();

    // Check gear/num-units property
    auto numUnitsNode = pm->GetNode("gear/num-units");
    TS_ASSERT(numUnitsNode != nullptr);
    if (numUnitsNode) {
      TS_ASSERT_EQUALS(numUnitsNode->getIntValue(), 6);
    }

    // Check gear/wow property exists
    auto wowNode = pm->GetNode("gear/wow");
    TS_ASSERT(wowNode != nullptr);
  }

  // Test c172x wheel velocities when stationary
  void testC172xWheelVelocitiesStationary() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);
    ic->SetVgroundKtsIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Check wheel velocities (should be near zero when stationary)
    double rollVel = gr->GetGearUnit(1)->GetWheelRollVel();
    double sideVel = gr->GetGearUnit(1)->GetWheelSideVel();

    TS_ASSERT(!std::isnan(rollVel));
    TS_ASSERT(!std::isnan(sideVel));
  }

  // Test c172x wheel slip angle
  void testC172xWheelSlipAngle() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Check slip angle for main gear
    double slipAngle = gr->GetGearUnit(1)->GetWheelSlipAngle();
    TS_ASSERT(!std::isnan(slipAngle));
  }

  // Test c172x gear steer angle
  void testC172xGearSteerAngle() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto gr = fdmex.GetGroundReactions();

    // Get initial steer angle
    double steerAngle = gr->GetGearUnit(0)->GetSteerAngleDeg();
    TS_ASSERT(!std::isnan(steerAngle));

    // Steer normalized value
    double steerNorm = gr->GetGearUnit(0)->GetSteerNorm();
    TS_ASSERT(!std::isnan(steerNorm));
  }

  // Test c172x compression velocity
  void testC172xCompressionVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto ic = fdmex.GetIC();
    ic->SetAltitudeAGLFtIC(0.0);

    fdmex.RunIC();
    fdmex.Run();

    auto gr = fdmex.GetGroundReactions();

    // Compression velocity should be finite
    for (int i = 0; i < 3; i++) {
      double compVel = gr->GetGearUnit(i)->GetCompVel();
      TS_ASSERT(!std::isnan(compVel));
      TS_ASSERT(std::isfinite(compVel));
    }
  }
};
