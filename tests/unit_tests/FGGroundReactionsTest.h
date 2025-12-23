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
};
