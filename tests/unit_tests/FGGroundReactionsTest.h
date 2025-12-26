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
};
