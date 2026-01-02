/*******************************************************************************
 * FGTireBrakeTest.h - Unit tests for tire and brake dynamics calculations
 *
 * Tests the mathematical behavior of tire and brake physics:
 * - Tire friction coefficients (dry/wet/icy conditions)
 * - Tire slip ratio calculations
 * - Brake torque vs pressure relationships
 * - Anti-skid system logic
 * - Brake fade with temperature
 * - Tire cornering forces
 * - Tire load capacity and deflection
 * - Hydroplaning speed calculations
 * - Rolling resistance
 * - Brake energy absorption and cooling
 * - Maximum braking deceleration
 * - Cross-wind tire forces
 *
 * Note: These tests focus on the underlying physics calculations that would be
 * used in a comprehensive tire and brake model.
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
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>

const double epsilon = 1e-10;

class FGTireBrakeTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Tire Friction Coefficient Tests
   ***************************************************************************/

  // Test dry runway friction coefficient
  void testDryRunwayFriction() {
    double dryMu = 0.8;  // Typical dry concrete/asphalt

    TS_ASSERT(dryMu > 0.7);
    TS_ASSERT(dryMu <= 1.0);
  }

  // Test wet runway friction coefficient
  void testWetRunwayFriction() {
    double wetMu = 0.5;  // Wet surface
    double dryMu = 0.8;

    TS_ASSERT(wetMu < dryMu);
    TS_ASSERT_DELTA(wetMu / dryMu, 0.625, 0.05);
  }

  // Test icy runway friction coefficient
  void testIcyRunwayFriction() {
    double icyMu = 0.1;  // Ice covered
    double wetMu = 0.5;

    TS_ASSERT(icyMu < wetMu);
    TS_ASSERT(icyMu > 0.0);
    TS_ASSERT(icyMu < 0.2);
  }

  // Test friction coefficient degradation with contamination
  void testFrictionDegradation() {
    double dryMu = 0.8;
    double contaminationDepth = 0.25;  // inches of water/snow
    double degradationFactor = 1.0 - std::min(0.6, contaminationDepth * 0.4);

    double degradedMu = dryMu * degradationFactor;
    TS_ASSERT(degradedMu < dryMu);
    TS_ASSERT_DELTA(degradedMu, 0.72, 0.01);
  }

  // Test temperature effect on friction
  void testTemperatureEffectOnFriction() {
    double baseMu = 0.8;
    double temp = 100.0;  // degrees F
    double refTemp = 70.0;
    double tempCoeff = 0.0005;  // Coefficient per degree

    double muAdjusted = baseMu * (1.0 - tempCoeff * (temp - refTemp));
    TS_ASSERT_DELTA(muAdjusted, 0.788, 0.001);
  }

  /***************************************************************************
   * Tire Slip Ratio Tests
   ***************************************************************************/

  // Test longitudinal slip ratio: s = (V_wheel - V_tire) / V_wheel
  void testLongitudinalSlipRatio() {
    double wheelSpeed = 100.0;  // ft/s
    double tireSpeed = 90.0;    // ft/s (tire slipping)

    double slipRatio = (wheelSpeed - tireSpeed) / wheelSpeed;
    TS_ASSERT_DELTA(slipRatio, 0.1, epsilon);
  }

  // Test zero slip (pure rolling)
  void testZeroSlip() {
    double wheelSpeed = 100.0;
    double tireSpeed = 100.0;  // No slip

    double slipRatio = (wheelSpeed - tireSpeed) / wheelSpeed;
    TS_ASSERT_DELTA(slipRatio, 0.0, epsilon);
  }

  // Test full lock-up (wheel locked)
  void testFullLockup() {
    double wheelSpeed = 100.0;
    double tireSpeed = 0.0;  // Wheel locked

    double slipRatio = (wheelSpeed - tireSpeed) / wheelSpeed;
    TS_ASSERT_DELTA(slipRatio, 1.0, epsilon);
  }

  // Test slip ratio at optimal braking (peak friction)
  void testOptimalSlipRatio() {
    double optimalSlip = 0.15;  // Typical optimal slip for maximum friction

    TS_ASSERT(optimalSlip > 0.1);
    TS_ASSERT(optimalSlip < 0.2);
  }

  // Test slip ratio calculation with low speed protection
  void testSlipRatioLowSpeed() {
    double wheelSpeed = 0.5;   // Very low speed
    double tireSpeed = 0.25;
    double minSpeed = 1.0;     // Minimum speed to avoid division issues

    double effectiveSpeed = std::max(wheelSpeed, minSpeed);
    double slipRatio = (effectiveSpeed - tireSpeed) / effectiveSpeed;

    TS_ASSERT(slipRatio < 1.0);
  }

  /***************************************************************************
   * Brake Torque vs Pressure Tests
   ***************************************************************************/

  // Test brake torque calculation
  void testBrakeTorque() {
    double brakePressure = 1000.0;  // psi
    double effectiveRadius = 0.5;   // ft
    double pistonArea = 3.0;        // sq in
    double padMu = 0.35;            // Brake pad friction

    double normalForce = brakePressure * pistonArea;  // lbf
    double torque = normalForce * padMu * effectiveRadius;  // ft-lbf

    TS_ASSERT_DELTA(torque, 525.0, epsilon);
  }

  // Test linear pressure-torque relationship
  void testLinearPressureTorque() {
    double pressure1 = 500.0;   // psi
    double pressure2 = 1000.0;  // psi
    double torqueGain = 0.5;    // ft-lbf per psi

    double torque1 = pressure1 * torqueGain;
    double torque2 = pressure2 * torqueGain;

    TS_ASSERT_DELTA(torque1, 250.0, epsilon);
    TS_ASSERT_DELTA(torque2, 500.0, epsilon);
    TS_ASSERT_DELTA(torque2 / torque1, 2.0, epsilon);
  }

  // Test maximum brake pressure limit
  void testMaxBrakePressure() {
    double commandedPressure = 2000.0;  // psi
    double maxPressure = 1500.0;        // psi (system limit)

    double actualPressure = std::min(commandedPressure, maxPressure);
    TS_ASSERT_DELTA(actualPressure, maxPressure, epsilon);
  }

  // Test brake pressure accumulator dynamics
  void testBrakePressureAccumulator() {
    double currentPressure = 800.0;   // psi
    double targetPressure = 1200.0;   // psi
    double timeConstant = 0.2;        // seconds
    double dt = 0.05;                 // seconds

    // First-order lag
    double alpha = dt / (timeConstant + dt);
    double newPressure = currentPressure + alpha * (targetPressure - currentPressure);

    TS_ASSERT(newPressure > currentPressure);
    TS_ASSERT(newPressure < targetPressure);
    TS_ASSERT_DELTA(newPressure, 880.0, 1.0);
  }

  /***************************************************************************
   * Anti-Skid System Logic Tests
   ***************************************************************************/

  // Test anti-skid slip detection
  void testAntiSkidSlipDetection() {
    double slipRatio = 0.25;       // Current slip
    double slipThreshold = 0.20;   // Anti-skid activates above this

    bool antiSkidActive = (slipRatio > slipThreshold);
    TS_ASSERT(antiSkidActive);
  }

  // Test anti-skid pressure reduction
  void testAntiSkidPressureReduction() {
    double currentPressure = 1000.0;  // psi
    double slipRatio = 0.3;           // Excessive slip
    double targetSlip = 0.15;         // Optimal slip
    double gainFactor = 500.0;        // psi per slip ratio

    double pressureReduction = (slipRatio - targetSlip) * gainFactor;
    double correctedPressure = currentPressure - pressureReduction;

    TS_ASSERT_DELTA(pressureReduction, 75.0, epsilon);
    TS_ASSERT_DELTA(correctedPressure, 925.0, epsilon);
  }

  // Test anti-skid release/apply cycle
  void testAntiSkidCycle() {
    double slipRatio = 0.35;
    double threshold = 0.2;

    // Release when slip exceeds threshold
    bool release = (slipRatio > threshold);
    TS_ASSERT(release);

    // Reapply when slip drops below threshold
    slipRatio = 0.15;
    bool apply = (slipRatio < threshold);
    TS_ASSERT(apply);
  }

  // Test anti-skid disabled at low speed
  void testAntiSkidLowSpeedDisable() {
    double groundSpeed = 15.0;  // kts
    double minSpeed = 20.0;     // kts (anti-skid disabled below this)

    bool antiSkidEnabled = (groundSpeed >= minSpeed);
    TS_ASSERT(!antiSkidEnabled);
  }

  // Test wheel deceleration rate monitoring
  void testWheelDecelerationRate() {
    double wheelSpeed = 100.0;     // ft/s
    double prevWheelSpeed = 110.0; // ft/s
    double dt = 0.05;              // seconds

    double deceleration = (prevWheelSpeed - wheelSpeed) / dt;
    TS_ASSERT_DELTA(deceleration, 200.0, epsilon);  // ft/s^2

    // Check for excessive deceleration (possible lockup)
    double maxDecel = 250.0;  // ft/s^2
    bool excessiveDecel = (deceleration > maxDecel);
    TS_ASSERT(!excessiveDecel);
  }

  /***************************************************************************
   * Brake Fade with Temperature Tests
   ***************************************************************************/

  // Test brake temperature rise from energy absorption
  void testBrakeTemperatureRise() {
    double initialTemp = 70.0;      // degrees F
    double energyAbsorbed = 5000.0; // ft-lbf
    double brakeMass = 50.0;        // lbm
    double specificHeat = 0.11;     // Btu/(lbm·F) for steel

    // Convert energy: 1 Btu = 778.16 ft-lbf
    double energyBtu = energyAbsorbed / 778.16;
    double tempRise = energyBtu / (brakeMass * specificHeat);
    double finalTemp = initialTemp + tempRise;

    TS_ASSERT(finalTemp > initialTemp);
    TS_ASSERT_DELTA(tempRise, 1.17, 0.01);
  }

  // Test brake fade factor with temperature
  void testBrakeFadeFactor() {
    double brakeTemp = 800.0;     // degrees F
    double nominalTemp = 200.0;   // degrees F
    double fadeTemp = 600.0;      // Temperature at which fade begins

    double fadeFactor = 1.0;
    if (brakeTemp > fadeTemp) {
      double fadeRate = 0.0008;  // Per degree F
      fadeFactor = 1.0 - (brakeTemp - fadeTemp) * fadeRate;
      fadeFactor = std::max(0.5, fadeFactor);  // Minimum 50% effectiveness
    }

    TS_ASSERT_DELTA(fadeFactor, 0.84, 0.01);
  }

  // Test effective brake torque with fade
  void testEffectiveTorqueWithFade() {
    double nominalTorque = 500.0;  // ft-lbf
    double fadeFactor = 0.75;      // 25% fade

    double effectiveTorque = nominalTorque * fadeFactor;
    TS_ASSERT_DELTA(effectiveTorque, 375.0, epsilon);
  }

  /***************************************************************************
   * Brake Cooling Rate Tests
   ***************************************************************************/

  // Test convective cooling
  void testConvectiveCooling() {
    double brakeTemp = 500.0;       // degrees F
    double ambientTemp = 70.0;      // degrees F
    double coolingCoeff = 0.05;     // Per second
    double dt = 1.0;                // seconds

    double tempDrop = coolingCoeff * (brakeTemp - ambientTemp) * dt;
    double newTemp = brakeTemp - tempDrop;

    TS_ASSERT_DELTA(tempDrop, 21.5, 0.1);
    TS_ASSERT_DELTA(newTemp, 478.5, 0.1);
  }

  // Test cooling with airflow (velocity dependent)
  void testAirflowCooling() {
    double brakeTemp = 600.0;
    double ambientTemp = 70.0;
    double velocity = 100.0;         // ft/s
    double baseCoolingCoeff = 0.02;

    // Cooling increases with velocity
    double velocityFactor = 1.0 + velocity / 200.0;
    double effectiveCooling = baseCoolingCoeff * velocityFactor;

    double tempDrop = effectiveCooling * (brakeTemp - ambientTemp);
    TS_ASSERT_DELTA(tempDrop, 15.9, 0.1);
  }

  // Test brake thermal equilibrium
  void testBrakeThermalEquilibrium() {
    double heatingRate = 10.0;   // degrees F per second
    double coolingRate = 10.0;   // degrees F per second

    double netRate = heatingRate - coolingRate;
    TS_ASSERT_DELTA(netRate, 0.0, epsilon);  // Equilibrium
  }

  /***************************************************************************
   * Tire Cornering Force Tests
   ***************************************************************************/

  // Test cornering stiffness coefficient
  void testCorneringStiffness() {
    double slipAngle = 5.0 * M_PI / 180.0;  // radians
    double corneringStiffness = 1000.0;     // lbf per radian

    double lateralForce = corneringStiffness * slipAngle;
    TS_ASSERT_DELTA(lateralForce, 87.27, 0.1);
  }

  // Test cornering force with load
  void testCorneringForceWithLoad() {
    double normalLoad = 3000.0;  // lbs
    double slipAngle = 3.0 * M_PI / 180.0;  // radians
    double loadSensitivity = 0.3;  // Cornering force per unit load and slip

    double lateralForce = loadSensitivity * normalLoad * slipAngle;
    TS_ASSERT_DELTA(lateralForce, 47.12, 0.1);
  }

  // Test cornering force saturation
  void testCorneringForceSaturation() {
    double slipAngle = 20.0 * M_PI / 180.0;  // High slip angle
    double normalLoad = 3000.0;
    double mu = 0.8;

    // Maximum lateral force limited by friction
    double maxLateralForce = mu * normalLoad;

    // Linear estimate (cornering stiffness approach)
    double corneringStiffness = 800.0;  // lbf/rad
    double linearEstimate = corneringStiffness * slipAngle;
    double actualForce = std::min(linearEstimate, maxLateralForce);

    // At 20 degrees, linear estimate is ~279 lbs, well under max
    TS_ASSERT(actualForce < maxLateralForce);
    TS_ASSERT_DELTA(actualForce, 279.25, 1.0);
  }

  // Test combined slip (longitudinal + lateral)
  void testCombinedSlip() {
    double longitudinalSlip = 0.1;
    double lateralSlipAngle = 5.0 * M_PI / 180.0;

    // Combined slip magnitude
    double combinedSlip = std::sqrt(longitudinalSlip * longitudinalSlip +
                                    lateralSlipAngle * lateralSlipAngle);

    TS_ASSERT(combinedSlip > longitudinalSlip);
    TS_ASSERT(combinedSlip > lateralSlipAngle);
    TS_ASSERT_DELTA(combinedSlip, 0.1327, 0.001);
  }

  /***************************************************************************
   * Tire Load Capacity Tests
   ***************************************************************************/

  // Test rated load capacity
  void testRatedLoadCapacity() {
    double ratedLoad = 5000.0;   // lbs at rated pressure
    double actualLoad = 4000.0;  // lbs

    double loadFactor = actualLoad / ratedLoad;
    TS_ASSERT_DELTA(loadFactor, 0.8, epsilon);
  }

  // Test load vs pressure relationship
  void testLoadPressureRelationship() {
    double tireArea = 50.0;       // sq in contact patch
    double tirePressure = 100.0;  // psi

    double supportedLoad = tireArea * tirePressure;
    TS_ASSERT_DELTA(supportedLoad, 5000.0, epsilon);
  }

  // Test overload condition
  void testOverloadCondition() {
    double actualLoad = 6000.0;
    double ratedLoad = 5000.0;
    double overloadThreshold = 1.1;  // 110% of rated

    bool overloaded = (actualLoad > ratedLoad * overloadThreshold);
    TS_ASSERT(overloaded);
  }

  // Test load distribution (multiple wheels)
  void testLoadDistribution() {
    double totalLoad = 10000.0;
    double numWheels = 4.0;

    double loadPerWheel = totalLoad / numWheels;
    TS_ASSERT_DELTA(loadPerWheel, 2500.0, epsilon);
  }

  /***************************************************************************
   * Tire Deflection Tests
   ***************************************************************************/

  // Test tire deflection under load
  void testTireDeflection() {
    double load = 3000.0;          // lbs
    double tireStiffness = 5000.0; // lbs/ft

    double deflection = load / tireStiffness;
    TS_ASSERT_DELTA(deflection, 0.6, epsilon);  // ft
  }

  // Test deflection with pressure variation
  void testDeflectionPressureVariation() {
    double load = 3000.0;
    double nominalPressure = 100.0;  // psi
    double actualPressure = 80.0;    // psi (underinflated)
    double nominalStiffness = 5000.0;

    // Stiffness approximately proportional to pressure
    double actualStiffness = nominalStiffness * (actualPressure / nominalPressure);
    double deflection = load / actualStiffness;

    TS_ASSERT(deflection > load / nominalStiffness);
    TS_ASSERT_DELTA(deflection, 0.75, 0.01);
  }

  // Test contact patch area
  void testContactPatchArea() {
    double load = 3000.0;
    double tirePressure = 100.0;  // psi

    // Contact patch area approximation
    double contactArea = load / tirePressure;  // sq in
    TS_ASSERT_DELTA(contactArea, 30.0, epsilon);
  }

  /***************************************************************************
   * Hydroplaning Speed Tests
   ***************************************************************************/

  // Test hydroplaning speed calculation (Horne's formula)
  void testHydroplaningSpeed() {
    double tirePressure = 100.0;  // psi

    // V_hydroplane = 8.6 * sqrt(P) in knots
    double hydroplaneSpeed = 8.6 * std::sqrt(tirePressure);
    TS_ASSERT_DELTA(hydroplaneSpeed, 86.0, 0.1);  // knots
  }

  // Test hydroplaning with low pressure
  void testHydroplaningLowPressure() {
    double tirePressure = 50.0;  // psi (low)

    double hydroplaneSpeed = 8.6 * std::sqrt(tirePressure);
    TS_ASSERT_DELTA(hydroplaneSpeed, 60.81, 0.1);  // knots
  }

  // Test friction reduction near hydroplaning
  void testFrictionNearHydroplaning() {
    double groundSpeed = 80.0;       // knots
    double hydroplaneSpeed = 86.0;   // knots
    double baseMu = 0.5;             // Wet friction

    // Friction reduces as speed approaches hydroplaning
    double speedRatio = groundSpeed / hydroplaneSpeed;
    double muReduction = std::pow(speedRatio, 3.0);  // Cubic reduction
    double effectiveMu = baseMu * (1.0 - 0.5 * muReduction);

    TS_ASSERT(effectiveMu < baseMu);
    TS_ASSERT_DELTA(effectiveMu, 0.299, 0.01);
  }

  // Test water depth effect on hydroplaning
  void testWaterDepthEffect() {
    double baseHydroSpeed = 86.0;   // knots
    double waterDepth = 0.5;        // inches
    double referenceDepth = 0.1;    // inches

    // Hydroplaning speed decreases with water depth
    double depthFactor = std::sqrt(referenceDepth / std::max(waterDepth, referenceDepth));
    double adjustedSpeed = baseHydroSpeed * depthFactor;

    TS_ASSERT(adjustedSpeed < baseHydroSpeed);
    TS_ASSERT_DELTA(adjustedSpeed, 38.43, 0.1);
  }

  /***************************************************************************
   * Rolling Resistance Tests
   ***************************************************************************/

  // Test rolling resistance coefficient
  void testRollingResistanceCoefficient() {
    double normalLoad = 3000.0;  // lbs
    double Crr = 0.015;          // Rolling resistance coefficient (concrete)

    double rollingResistance = Crr * normalLoad;
    TS_ASSERT_DELTA(rollingResistance, 45.0, epsilon);
  }

  // Test rolling resistance on different surfaces
  void testRollingResistanceSurfaces() {
    double load = 3000.0;
    double CrrConcrete = 0.012;
    double CrrAsphalt = 0.015;
    double CrrGrass = 0.10;
    double CrrSand = 0.30;

    TS_ASSERT(CrrConcrete < CrrAsphalt);
    TS_ASSERT(CrrAsphalt < CrrGrass);
    TS_ASSERT(CrrGrass < CrrSand);

    double resistanceSand = CrrSand * load;
    TS_ASSERT_DELTA(resistanceSand, 900.0, epsilon);
  }

  // Test rolling resistance with speed
  void testRollingResistanceSpeed() {
    double load = 3000.0;
    double baseCrr = 0.015;
    double velocity = 100.0;  // ft/s

    // Rolling resistance increases slightly with speed
    double speedFactor = 1.0 + velocity / 10000.0;
    double effectiveCrr = baseCrr * speedFactor;

    TS_ASSERT(effectiveCrr > baseCrr);
    TS_ASSERT_DELTA(effectiveCrr, 0.0151, 0.001);
  }

  /***************************************************************************
   * Brake Energy Absorption Tests
   ***************************************************************************/

  // Test kinetic energy dissipation
  void testKineticEnergyDissipation() {
    double mass = 10000.0 / 32.174;  // slugs (10000 lbs weight)
    double initialSpeed = 150.0;     // ft/s
    double finalSpeed = 50.0;        // ft/s

    double KEinitial = 0.5 * mass * initialSpeed * initialSpeed;
    double KEfinal = 0.5 * mass * finalSpeed * finalSpeed;
    double energyDissipated = KEinitial - KEfinal;

    TS_ASSERT(energyDissipated > 0);
    TS_ASSERT_DELTA(energyDissipated, 3108100.0, 100.0);  // ft-lbf
  }

  // Test brake energy per wheel
  void testBrakeEnergyPerWheel() {
    double totalEnergy = 2000000.0;  // ft-lbf
    double numBrakes = 4;
    double brakeDistribution = 0.9;  // 90% on main gear (10% aerodynamic)

    double energyPerBrake = (totalEnergy * brakeDistribution) / numBrakes;
    TS_ASSERT_DELTA(energyPerBrake, 450000.0, epsilon);
  }

  // Test specific energy absorption
  void testSpecificEnergyAbsorption() {
    double energy = 450000.0;     // ft-lbf
    double brakeMass = 50.0;      // lbm

    double specificEnergy = energy / brakeMass;  // ft-lbf per lbm
    TS_ASSERT_DELTA(specificEnergy, 9000.0, epsilon);
  }

  /***************************************************************************
   * Maximum Braking Deceleration Tests
   ***************************************************************************/

  // Test maximum deceleration on dry surface
  void testMaxDecelerationDry() {
    double mu = 0.8;              // Dry friction
    double g = 32.174;            // ft/s^2

    double maxDecel = mu * g;
    TS_ASSERT_DELTA(maxDecel, 25.74, 0.1);  // ft/s^2
  }

  // Test maximum deceleration on wet surface
  void testMaxDecelerationWet() {
    double mu = 0.5;
    double g = 32.174;

    double maxDecel = mu * g;
    TS_ASSERT_DELTA(maxDecel, 16.09, 0.1);  // ft/s^2
  }

  // Test stopping distance calculation
  void testStoppingDistance() {
    double initialSpeed = 150.0;  // ft/s (about 88 kts)
    double deceleration = 20.0;   // ft/s^2

    // d = v^2 / (2 * a)
    double stoppingDistance = (initialSpeed * initialSpeed) / (2.0 * deceleration);
    TS_ASSERT_DELTA(stoppingDistance, 562.5, 0.1);  // ft
  }

  // Test stopping distance with reaction time
  void testStoppingDistanceWithReaction() {
    double initialSpeed = 150.0;   // ft/s
    double reactionTime = 1.5;     // seconds
    double deceleration = 20.0;    // ft/s^2

    double reactionDistance = initialSpeed * reactionTime;
    double brakingDistance = (initialSpeed * initialSpeed) / (2.0 * deceleration);
    double totalDistance = reactionDistance + brakingDistance;

    TS_ASSERT_DELTA(totalDistance, 787.5, 0.1);  // ft
  }

  // Test deceleration limited by anti-skid
  void testDecelerationAntiSkidLimit() {
    double mu = 0.8;
    double g = 32.174;
    double theoreticalMax = mu * g;  // 25.74 ft/s^2

    // Anti-skid typically achieves 85-95% of theoretical max
    double antiSkidEfficiency = 0.90;
    double actualDecel = theoreticalMax * antiSkidEfficiency;

    TS_ASSERT_DELTA(actualDecel, 23.17, 0.1);
  }

  /***************************************************************************
   * Cross-Wind Tire Forces Tests
   ***************************************************************************/

  // Test cross-wind tire side force
  void testCrossWindSideForce() {
    double crossWindVelocity = 20.0;  // ft/s
    double groundSpeed = 100.0;       // ft/s

    double driftAngle = std::atan2(crossWindVelocity, groundSpeed);
    TS_ASSERT_DELTA(driftAngle, 0.197, 0.001);  // radians (~11.3 deg)

    double corneringStiffness = 1000.0;  // lbf/rad
    double sideForce = corneringStiffness * driftAngle;
    TS_ASSERT_DELTA(sideForce, 197.4, 1.0);
  }

  // Test cross-wind moment about vertical axis
  void testCrossWindMoment() {
    double noseGearSideForce = 100.0;   // lbs
    double mainGearSideForce = 300.0;   // lbs (total)
    double noseToMainDistance = 15.0;   // ft
    double cgToMainDistance = 5.0;      // ft (CG forward of main gear)

    // Yaw moment about CG
    double moment = noseGearSideForce * (noseToMainDistance - cgToMainDistance) -
                    mainGearSideForce * cgToMainDistance;

    TS_ASSERT_DELTA(moment, -500.0, epsilon);  // ft-lbf (nose left)
  }

  // Test tire side force saturation
  void testSideForceSaturation() {
    double normalLoad = 3000.0;
    double mu = 0.8;
    double maxSideForce = mu * normalLoad;  // 2400 lbs

    double driftAngle = 15.0 * M_PI / 180.0;  // Large drift angle
    double corneringStiffness = 1000.0;
    double linearSideForce = corneringStiffness * driftAngle;  // Would be ~262 lbs

    double actualSideForce = std::min(linearSideForce, maxSideForce);
    TS_ASSERT_DELTA(actualSideForce, 261.8, 1.0);  // Not saturated yet

    // Test with extreme drift
    driftAngle = 30.0 * M_PI / 180.0;
    linearSideForce = corneringStiffness * driftAngle;  // Would be ~524 lbs
    actualSideForce = std::min(linearSideForce, maxSideForce);
    TS_ASSERT_DELTA(actualSideForce, 523.6, 1.0);  // Still not saturated
  }

  // Test asymmetric tire loading in crosswind
  void testAsymmetricTireLoading() {
    double totalWeight = 10000.0;
    double baseLoadPerSide = 5000.0;
    double sideForce = 500.0;
    double cgHeight = 5.0;          // ft
    double trackWidth = 10.0;       // ft

    // Rolling moment creates load transfer
    double rollingMoment = sideForce * cgHeight;
    double loadTransfer = rollingMoment / trackWidth;

    double windwardLoad = baseLoadPerSide - loadTransfer;
    double leewardLoad = baseLoadPerSide + loadTransfer;

    TS_ASSERT_DELTA(loadTransfer, 250.0, epsilon);
    TS_ASSERT_DELTA(windwardLoad, 4750.0, epsilon);
    TS_ASSERT_DELTA(leewardLoad, 5250.0, epsilon);
    TS_ASSERT_DELTA(windwardLoad + leewardLoad, totalWeight, epsilon);
  }

  /***************************************************************************
   * Edge Cases and Integration Tests
   ***************************************************************************/

  // Test brake application at zero speed
  void testBrakeAtZeroSpeed() {
    double speed = 0.0;
    double brakeTorque = 500.0;

    // No braking force when stationary
    double brakingForce = (speed > 0.1) ? brakeTorque : 0.0;
    TS_ASSERT_DELTA(brakingForce, 0.0, epsilon);
  }

  // Test tire force with zero normal load (airborne)
  void testTireForceAirborne() {
    double normalLoad = 0.0;
    double mu = 0.8;

    double maxForce = mu * normalLoad;
    TS_ASSERT_DELTA(maxForce, 0.0, epsilon);
  }

  // Test combined braking and cornering
  void testCombinedBrakingCornering() {
    double normalLoad = 3000.0;
    double mu = 0.8;
    double maxTotalForce = mu * normalLoad;  // 2400 lbs

    double brakingForce = 1500.0;  // lbs
    double corneringForce = 800.0;  // lbs

    // Combined force (friction ellipse approximation)
    double combinedForce = std::sqrt(brakingForce * brakingForce +
                                     corneringForce * corneringForce);

    TS_ASSERT_DELTA(combinedForce, 1700.0, 1.0);
    TS_ASSERT(combinedForce < maxTotalForce);  // Within friction limit
  }

  // Test friction ellipse saturation
  void testFrictionEllipseSaturation() {
    double normalLoad = 3000.0;
    double mu = 0.8;
    double maxForce = mu * normalLoad;  // 2400 lbs

    double brakingForce = 2000.0;   // lbs
    double corneringForce = 1500.0;  // lbs

    double combinedForce = std::sqrt(brakingForce * brakingForce +
                                     corneringForce * corneringForce);

    TS_ASSERT(combinedForce > maxForce);  // Exceeds friction limit

    // Scale forces back to friction limit
    double scaleFactor = maxForce / combinedForce;
    double limitedBraking = brakingForce * scaleFactor;
    double limitedCornering = corneringForce * scaleFactor;

    TS_ASSERT(limitedBraking < brakingForce);
    TS_ASSERT(limitedCornering < corneringForce);

    double limitedCombined = std::sqrt(limitedBraking * limitedBraking +
                                       limitedCornering * limitedCornering);
    TS_ASSERT_DELTA(limitedCombined, maxForce, 1.0);
  }

  // Test differential braking for steering
  void testDifferentialBraking() {
    double leftBrakeTorque = 600.0;   // ft-lbf
    double rightBrakeTorque = 300.0;  // ft-lbf
    double trackWidth = 10.0;         // ft
    double wheelRadius = 1.5;         // ft

    double leftForce = leftBrakeTorque / wheelRadius;
    double rightForce = rightBrakeTorque / wheelRadius;
    double differential = leftForce - rightForce;

    double yawMoment = differential * trackWidth / 2.0;
    TS_ASSERT_DELTA(yawMoment, 1000.0, epsilon);  // ft-lbf (yaw right)
  }

  // Test brake thermal runaway detection
  void testBrakeThermalRunaway() {
    double brakeTemp = 1200.0;      // degrees F (very hot)
    double criticalTemp = 1000.0;   // degrees F
    double fadeFactor = 0.3;        // Severe fade

    bool thermalRunaway = (brakeTemp > criticalTemp && fadeFactor < 0.5);
    TS_ASSERT(thermalRunaway);
  }

  // Test tire blowout threshold
  void testTireBlowoutThreshold() {
    double tireTemp = 300.0;        // degrees F
    double criticalTemp = 250.0;    // degrees F
    double deflection = 0.8;        // ft
    double maxDeflection = 0.7;     // ft

    bool overheated = (tireTemp > criticalTemp);
    bool overDeflected = (deflection > maxDeflection);
    bool blowoutRisk = (overheated || overDeflected);

    TS_ASSERT(blowoutRisk);
  }

  /***************************************************************************
   * Autobrake System Tests
   ***************************************************************************/

  // Test autobrake deceleration setting
  void testAutobrakeSetting() {
    double lowDecel = 4.0;    // ft/s^2
    double medDecel = 7.0;    // ft/s^2
    double hiDecel = 12.0;    // ft/s^2
    double maxDecel = 16.0;   // ft/s^2

    TS_ASSERT(lowDecel < medDecel);
    TS_ASSERT(medDecel < hiDecel);
    TS_ASSERT(hiDecel < maxDecel);
  }

  // Test autobrake pressure modulation
  void testAutobrakePressureModulation() {
    double targetDecel = 8.0;    // ft/s^2
    double actualDecel = 6.0;    // ft/s^2
    double Kp = 100.0;           // Gain

    double error = targetDecel - actualDecel;
    double pressureAdjust = Kp * error;

    TS_ASSERT(pressureAdjust > 0);  // Increase pressure
    TS_ASSERT_DELTA(pressureAdjust, 200.0, epsilon);
  }

  // Test autobrake disarm on manual brake
  void testAutobrakeDisarmManual() {
    bool autobrakeArmed = true;
    double manualBrakePressure = 500.0;  // psi
    double disarmThreshold = 100.0;      // psi

    if (manualBrakePressure > disarmThreshold) {
      autobrakeArmed = false;
    }

    TS_ASSERT(!autobrakeArmed);
  }

  // Test autobrake RTO mode
  void testAutobrakeRTO() {
    double groundSpeed = 80.0;   // kts
    double rtoThreshold = 60.0;  // kts
    bool throttleIdle = true;

    bool rtoActive = (groundSpeed > rtoThreshold) && throttleIdle;
    TS_ASSERT(rtoActive);
  }

  /***************************************************************************
   * Carbon Brake Characteristics Tests
   ***************************************************************************/

  // Test carbon brake higher operating temperature
  void testCarbonBrakeTemperature() {
    double steelMaxTemp = 800.0;    // degrees F
    double carbonMaxTemp = 1800.0;  // degrees F

    TS_ASSERT(carbonMaxTemp > steelMaxTemp);
    TS_ASSERT_DELTA(carbonMaxTemp / steelMaxTemp, 2.25, 0.01);
  }

  // Test carbon brake wear rate
  void testCarbonBrakeWearRate() {
    double energyAbsorbed = 1000000.0;  // ft-lbf
    double wearCoefficient = 1e-9;       // Wear per ft-lbf

    double wear = energyAbsorbed * wearCoefficient;
    TS_ASSERT_DELTA(wear, 0.001, epsilon);  // 0.1% wear
  }

  // Test carbon brake cold performance
  void testCarbonBrakeColdPerformance() {
    double coldMu = 0.25;   // Lower at cold
    double hotMu = 0.35;    // Normal at hot
    double temp = 100.0;    // degrees F

    // Carbon brakes need warmup
    double transitionTemp = 300.0;
    double effectiveMu = (temp < transitionTemp) ? coldMu : hotMu;

    TS_ASSERT_DELTA(effectiveMu, 0.25, epsilon);
  }

  // Test carbon brake moisture sensitivity
  void testCarbonBrakeMoisture() {
    double dryMu = 0.35;
    double wetMu = 0.20;    // Significant reduction
    bool wetBrakes = true;

    double effectiveMu = wetBrakes ? wetMu : dryMu;
    TS_ASSERT_DELTA(effectiveMu, 0.20, epsilon);
  }

  /***************************************************************************
   * Tire Wear Modeling Tests
   ***************************************************************************/

  // Test tire wear from braking
  void testTireWearBraking() {
    double slipDistance = 100.0;   // ft
    double wearRate = 0.001;       // Wear per ft of slip

    double wear = slipDistance * wearRate;
    TS_ASSERT_DELTA(wear, 0.1, epsilon);
  }

  // Test tire wear from cornering
  void testTireWearCornering() {
    double slipAngle = 5.0 * M_PI / 180.0;
    double distanceTraveled = 1000.0;  // ft
    double lateralSlip = distanceTraveled * std::tan(slipAngle);
    double wearRate = 0.0001;

    double wear = lateralSlip * wearRate;
    TS_ASSERT(wear > 0);
  }

  // Test tire tread depth
  void testTireTreadDepth() {
    double newTreadDepth = 0.5;     // inches
    double minTreadDepth = 0.125;   // inches
    double currentDepth = 0.2;

    bool needsReplacement = (currentDepth < minTreadDepth);
    TS_ASSERT(!needsReplacement);

    currentDepth = 0.1;
    needsReplacement = (currentDepth < minTreadDepth);
    TS_ASSERT(needsReplacement);
  }

  // Test tire pressure loss rate
  void testTirePressureLoss() {
    double initialPressure = 200.0;  // psi
    double lossRate = 0.5;           // psi per day
    double days = 7.0;

    double finalPressure = initialPressure - (lossRate * days);
    TS_ASSERT_DELTA(finalPressure, 196.5, epsilon);
  }

  /***************************************************************************
   * Emergency Braking Tests
   ***************************************************************************/

  // Test emergency brake system
  void testEmergencyBrakeSystem() {
    bool hydraulicFailed = true;
    bool emergencyAccumulator = true;
    double accumulatorPressure = 2500.0;  // psi

    bool canBrake = emergencyAccumulator && (accumulatorPressure > 1000.0);
    TS_ASSERT(canBrake);
  }

  // Test parking brake force
  void testParkingBrakeForce() {
    double springForce = 1500.0;    // lbs
    double padMu = 0.35;
    double effectiveRadius = 0.5;   // ft

    double torque = springForce * padMu * effectiveRadius;
    TS_ASSERT_DELTA(torque, 262.5, epsilon);
  }

  // Test brake by wire backup
  void testBrakeByWireBackup() {
    bool primarySystem = false;
    bool backupSystem = true;

    bool brakingAvailable = primarySystem || backupSystem;
    TS_ASSERT(brakingAvailable);
  }

  // Test accumulator brake cycles
  void testAccumulatorBrakeCycles() {
    double accumulatorVolume = 50.0;   // cubic inches
    double volumePerCycle = 5.0;       // cubic inches

    int maxCycles = static_cast<int>(accumulatorVolume / volumePerCycle);
    TS_ASSERT_EQUALS(maxCycles, 10);
  }

  /***************************************************************************
   * Tire Pressure Monitoring Tests
   ***************************************************************************/

  // Test TPMS warning threshold
  void testTPMSWarning() {
    double nominalPressure = 200.0;  // psi
    double currentPressure = 165.0;  // psi
    double warningThreshold = 0.15;  // 15% low

    double deviation = (nominalPressure - currentPressure) / nominalPressure;
    bool warning = (deviation > warningThreshold);

    TS_ASSERT(warning);  // 17.5% > 15%
  }

  // Test TPMS temperature compensation
  void testTPMSTempCompensation() {
    double measuredPressure = 220.0;  // psi
    double ambientTemp = 100.0;       // degrees F
    double refTemp = 70.0;
    double tempCoeff = 0.002;         // psi per degree F

    double correctedPressure = measuredPressure - tempCoeff * (ambientTemp - refTemp) * measuredPressure;
    TS_ASSERT(correctedPressure < measuredPressure);
  }

  // Test dual sensor redundancy
  void testDualSensorRedundancy() {
    double sensor1 = 198.0;  // psi
    double sensor2 = 202.0;  // psi
    double maxDisagreement = 10.0;

    double disagreement = std::abs(sensor1 - sensor2);
    bool sensorsValid = (disagreement < maxDisagreement);

    TS_ASSERT(sensorsValid);
  }

  /***************************************************************************
   * Brake Hydraulic System Tests
   ***************************************************************************/

  // Test brake line pressure drop
  void testBrakeLinePressureDrop() {
    double masterCylinderPressure = 1500.0;  // psi
    double lineLength = 20.0;                // ft
    double frictionFactor = 1.0;             // psi per ft

    double pressureDrop = lineLength * frictionFactor;
    double wheelPressure = masterCylinderPressure - pressureDrop;

    TS_ASSERT_DELTA(wheelPressure, 1480.0, epsilon);
  }

  // Test brake fluid boiling point
  void testBrakeFluidBoiling() {
    double fluidTemp = 400.0;          // degrees F
    double boilingPoint = 450.0;       // degrees F (DOT 4)
    double wetBoilingPoint = 311.0;    // degrees F

    bool dryFluidSafe = (fluidTemp < boilingPoint);
    bool wetFluidSafe = (fluidTemp < wetBoilingPoint);

    TS_ASSERT(dryFluidSafe);
    TS_ASSERT(!wetFluidSafe);
  }

  // Test brake pedal feel simulation
  void testBrakePedalFeel() {
    double pedalForce = 50.0;        // lbs
    double pedalRatio = 5.0;
    double masterCylinderArea = 1.0; // sq in

    double hydraulicPressure = (pedalForce * pedalRatio) / masterCylinderArea;
    TS_ASSERT_DELTA(hydraulicPressure, 250.0, epsilon);
  }

  /***************************************************************************
   * Runway Surface Detection Tests
   ***************************************************************************/

  // Test runway contamination depth
  void testContaminationDepth() {
    double waterDepth = 0.25;        // inches
    double slushDepth = 0.5;         // inches
    double snowDepth = 2.0;          // inches

    // Friction degradation increases with depth
    double waterMuFactor = 1.0 - 0.3 * waterDepth;
    double slushMuFactor = 1.0 - 0.4 * slushDepth;
    double snowMuFactor = 1.0 - 0.1 * snowDepth;

    TS_ASSERT_DELTA(waterMuFactor, 0.925, 0.01);
    TS_ASSERT_DELTA(slushMuFactor, 0.80, 0.01);
    TS_ASSERT_DELTA(snowMuFactor, 0.80, 0.01);
  }

  // Test grooved runway friction
  void testGroovedRunwayFriction() {
    double smoothMu = 0.5;   // Wet smooth
    double groovedMu = 0.7;  // Wet grooved

    TS_ASSERT(groovedMu > smoothMu);
    double improvement = (groovedMu - smoothMu) / smoothMu;
    TS_ASSERT_DELTA(improvement, 0.4, 0.01);  // 40% improvement
  }

  // Test rubber deposit effect
  void testRubberDepositEffect() {
    double cleanMu = 0.8;
    double rubberMu = 0.6;   // Rubber contaminated

    double degradation = (cleanMu - rubberMu) / cleanMu;
    TS_ASSERT_DELTA(degradation, 0.25, 0.01);  // 25% reduction
  }

  /***************************************************************************
   * Advanced Tire Physics Tests
   ***************************************************************************/

  // Test Pacejka magic formula
  void testPacejkaMagicFormula() {
    double B = 10.0;   // Stiffness factor
    double C = 1.9;    // Shape factor
    double D = 1.0;    // Peak value
    double E = 0.97;   // Curvature factor
    double slip = 0.1; // 10% slip

    // Simplified Pacejka formula: F = D * sin(C * atan(B*slip - E*(B*slip - atan(B*slip))))
    double Bslip = B * slip;
    double F = D * std::sin(C * std::atan(Bslip - E * (Bslip - std::atan(Bslip))));

    TS_ASSERT(F > 0.0);
    TS_ASSERT(F <= D);  // Cannot exceed peak
  }

  // Test load sensitivity
  void testLoadSensitivity() {
    double nominalLoad = 3000.0;    // lbs
    double nominalMu = 0.8;
    double loadSensitivity = 0.1;   // Mu reduction per 1000 lbs over nominal

    double actualLoad = 5000.0;     // lbs
    double loadExcess = (actualLoad - nominalLoad) / 1000.0;
    double actualMu = nominalMu - loadSensitivity * loadExcess;

    TS_ASSERT(actualMu < nominalMu);
    TS_ASSERT_DELTA(actualMu, 0.6, 0.01);
  }

  // Test tire speed rating
  void testTireSpeedRating() {
    double maxRatedSpeed = 225.0;   // mph
    double actualSpeed = 200.0;     // mph

    bool withinRating = (actualSpeed <= maxRatedSpeed);
    TS_ASSERT(withinRating);

    actualSpeed = 250.0;
    withinRating = (actualSpeed <= maxRatedSpeed);
    TS_ASSERT(!withinRating);
  }

  // Test tire heat generation
  void testTireHeatGeneration() {
    double rollingResistanceForce = 45.0;  // lbs
    double velocity = 100.0;                // ft/s

    double heatRate = rollingResistanceForce * velocity;  // ft-lbf/s
    double heatRateBTU = heatRate / 778.16;               // BTU/s

    TS_ASSERT(heatRateBTU > 0);
    TS_ASSERT_DELTA(heatRateBTU, 5.78, 0.1);
  }
};
