/*******************************************************************************
 * FGTankTest.h - Unit tests for FGTank (fuel tank modeling)
 *
 * Tests the mathematical behavior of fuel tank simulation:
 * - Fuel quantity and capacity
 * - Fuel consumption and transfer
 * - Tank center of gravity effects
 * - Temperature and density effects
 * - Tank pressurization
 *
 * Note: FGTank requires XML element for construction, so these tests focus
 * on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double LBS_TO_GAL = 1.0 / 6.02;  // Approximate for jet fuel
const double GAL_TO_LBS = 6.02;

class FGTankTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Tank State Structure
   ***************************************************************************/
  struct TankState {
    double contents = 0.0;    // lbs
    double capacity = 1000.0; // lbs
    double unusable = 5.0;    // lbs (unusable fuel)
    double density = 6.02;    // lbs/gal (Jet-A)
    double temperature = 288.15;  // K
    double x = 0.0;           // Tank CG location
    double y = 0.0;
    double z = 0.0;
    bool selected = true;
    double priority = 1.0;
  };

  /***************************************************************************
   * Basic Quantity Tests
   ***************************************************************************/

  // Test fuel contents
  void testFuelContents() {
    TankState tank;
    tank.contents = 500.0;  // lbs

    TS_ASSERT_DELTA(tank.contents, 500.0, epsilon);
  }

  // Test fuel in gallons
  void testFuelInGallons() {
    TankState tank;
    tank.contents = 602.0;  // lbs
    tank.density = 6.02;    // lbs/gal

    double gallons = tank.contents / tank.density;
    TS_ASSERT_DELTA(gallons, 100.0, epsilon);
  }

  // Test capacity check
  void testCapacityCheck() {
    TankState tank;
    tank.capacity = 1000.0;
    tank.contents = 800.0;

    double fillPercent = (tank.contents / tank.capacity) * 100.0;
    TS_ASSERT_DELTA(fillPercent, 80.0, epsilon);
  }

  // Test usable fuel
  void testUsableFuel() {
    TankState tank;
    tank.contents = 100.0;
    tank.unusable = 5.0;

    double usable = std::max(0.0, tank.contents - tank.unusable);
    TS_ASSERT_DELTA(usable, 95.0, epsilon);
  }

  // Test unusable fuel threshold
  void testUnusableFuelThreshold() {
    TankState tank;
    tank.contents = 3.0;   // Below unusable threshold
    tank.unusable = 5.0;

    double usable = std::max(0.0, tank.contents - tank.unusable);
    TS_ASSERT_DELTA(usable, 0.0, epsilon);  // No usable fuel
  }

  /***************************************************************************
   * Fuel Consumption Tests
   ***************************************************************************/

  // Test fuel consumption
  void testFuelConsumption() {
    TankState tank;
    tank.contents = 500.0;

    double fuelFlow = 100.0;  // lbs/hr
    double dt = 0.1;          // hours (6 minutes)

    tank.contents -= fuelFlow * dt;
    TS_ASSERT_DELTA(tank.contents, 490.0, epsilon);
  }

  // Test fuel exhaustion
  void testFuelExhaustion() {
    TankState tank;
    tank.contents = 10.0;
    tank.unusable = 5.0;

    double fuelFlow = 100.0;  // lbs/hr
    double dt = 0.1;          // 6 minutes

    // Would consume 10 lbs, but only 5 usable
    double consumption = std::min(fuelFlow * dt, tank.contents - tank.unusable);
    consumption = std::max(0.0, consumption);
    tank.contents -= consumption;

    TS_ASSERT_DELTA(tank.contents, 5.0, epsilon);  // Left with unusable only
  }

  // Test consumption cannot go negative
  void testContentsNonNegative() {
    TankState tank;
    tank.contents = 50.0;

    double consumption = 100.0;  // More than available
    tank.contents = std::max(0.0, tank.contents - consumption);

    TS_ASSERT_DELTA(tank.contents, 0.0, epsilon);
  }

  // Test time to empty
  void testTimeToEmpty() {
    TankState tank;
    tank.contents = 1000.0;
    tank.unusable = 50.0;

    double fuelFlow = 200.0;  // lbs/hr
    double usable = tank.contents - tank.unusable;
    double hoursRemaining = usable / fuelFlow;

    TS_ASSERT_DELTA(hoursRemaining, 4.75, epsilon);  // 950/200 = 4.75 hrs
  }

  /***************************************************************************
   * Fuel Transfer Tests
   ***************************************************************************/

  // Test fuel transfer between tanks
  void testFuelTransfer() {
    TankState tankFrom, tankTo;
    tankFrom.contents = 500.0;
    tankTo.contents = 300.0;

    double transferRate = 50.0;  // lbs/hr
    double dt = 0.1;             // hours

    double transferred = transferRate * dt;
    tankFrom.contents -= transferred;
    tankTo.contents += transferred;

    TS_ASSERT_DELTA(tankFrom.contents, 495.0, epsilon);
    TS_ASSERT_DELTA(tankTo.contents, 305.0, epsilon);
  }

  // Test transfer limited by source
  void testTransferLimitedBySource() {
    TankState tankFrom, tankTo;
    tankFrom.contents = 10.0;
    tankFrom.unusable = 5.0;
    tankTo.contents = 300.0;

    double transferRate = 50.0;
    double dt = 0.5;  // Would transfer 25 lbs

    double available = tankFrom.contents - tankFrom.unusable;
    double transferred = std::min(transferRate * dt, available);
    tankFrom.contents -= transferred;
    tankTo.contents += transferred;

    TS_ASSERT_DELTA(tankFrom.contents, 5.0, epsilon);  // Down to unusable
    TS_ASSERT_DELTA(tankTo.contents, 305.0, epsilon);  // Received 5 lbs
  }

  // Test transfer limited by destination capacity
  void testTransferLimitedByDestination() {
    TankState tankFrom, tankTo;
    tankFrom.contents = 500.0;
    tankTo.contents = 990.0;
    tankTo.capacity = 1000.0;

    double transferRate = 50.0;
    double dt = 0.5;  // Would transfer 25 lbs

    double roomInDest = tankTo.capacity - tankTo.contents;
    double transferred = std::min(transferRate * dt, roomInDest);
    tankFrom.contents -= transferred;
    tankTo.contents += transferred;

    TS_ASSERT_DELTA(tankFrom.contents, 490.0, epsilon);
    TS_ASSERT_DELTA(tankTo.contents, 1000.0, epsilon);  // Full
  }

  /***************************************************************************
   * Center of Gravity Tests
   ***************************************************************************/

  // Test CG contribution from single tank
  void testSingleTankCG() {
    TankState tank;
    tank.contents = 100.0;
    tank.x = 10.0;  // Tank CG at x=10 ft

    double momentX = tank.contents * tank.x;
    TS_ASSERT_DELTA(momentX, 1000.0, epsilon);  // 100 lbs * 10 ft
  }

  // Test CG from multiple tanks
  void testMultipleTanksCG() {
    TankState tank1, tank2;
    tank1.contents = 100.0;
    tank1.x = 5.0;
    tank2.contents = 200.0;
    tank2.x = 15.0;

    double totalWeight = tank1.contents + tank2.contents;
    double momentX = tank1.contents * tank1.x + tank2.contents * tank2.x;
    double cgX = momentX / totalWeight;

    // CG = (100*5 + 200*15) / 300 = 3500/300 = 11.667
    TS_ASSERT_DELTA(cgX, 11.667, 0.001);
  }

  // Test CG shift during fuel burn
  void testCGShiftDuringBurn() {
    TankState fwdTank, aftTank;
    fwdTank.contents = 200.0;
    fwdTank.x = 5.0;
    aftTank.contents = 200.0;
    aftTank.x = 15.0;

    // Initial CG
    double totalWeight1 = fwdTank.contents + aftTank.contents;
    double cgX1 = (fwdTank.contents * fwdTank.x + aftTank.contents * aftTank.x) / totalWeight1;
    TS_ASSERT_DELTA(cgX1, 10.0, epsilon);  // Centered

    // Burn from forward tank only
    fwdTank.contents -= 100.0;

    // New CG
    double totalWeight2 = fwdTank.contents + aftTank.contents;
    double cgX2 = (fwdTank.contents * fwdTank.x + aftTank.contents * aftTank.x) / totalWeight2;
    // (100*5 + 200*15) / 300 = 3500/300 = 11.667
    TS_ASSERT_DELTA(cgX2, 11.667, 0.001);  // CG moved aft
  }

  // Test lateral CG from wing tanks
  void testLateralCG() {
    TankState leftWing, rightWing;
    leftWing.contents = 150.0;
    leftWing.y = -10.0;  // Left of centerline
    rightWing.contents = 100.0;
    rightWing.y = 10.0;   // Right of centerline

    double totalWeight = leftWing.contents + rightWing.contents;
    double momentY = leftWing.contents * leftWing.y + rightWing.contents * rightWing.y;
    double cgY = momentY / totalWeight;

    // (150*-10 + 100*10) / 250 = -500/250 = -2.0
    TS_ASSERT_DELTA(cgY, -2.0, epsilon);  // CG is left of centerline
  }

  /***************************************************************************
   * Temperature and Density Tests
   ***************************************************************************/

  // Test fuel density variation with temperature
  void testFuelDensityTemperature() {
    double baseDensity = 6.02;  // lbs/gal at 15°C (288K)
    double tempCoeff = -0.003;  // Density change per K (approx)
    double baseTemp = 288.15;
    double actualTemp = 308.15;  // 35°C

    double density = baseDensity + tempCoeff * (actualTemp - baseTemp);
    TS_ASSERT_DELTA(density, 5.96, 0.01);  // Slightly less dense when warm
  }

  // Test volume calculation
  void testVolumeCalculation() {
    TankState tank;
    tank.contents = 602.0;   // lbs
    tank.density = 6.02;     // lbs/gal

    double volume = tank.contents / tank.density;
    TS_ASSERT_DELTA(volume, 100.0, epsilon);  // gallons
  }

  // Test volume expansion with temperature
  void testVolumeExpansion() {
    double mass = 602.0;  // lbs (constant)
    double coldDensity = 6.10;
    double hotDensity = 5.95;

    double coldVolume = mass / coldDensity;
    double hotVolume = mass / hotDensity;

    TS_ASSERT(hotVolume > coldVolume);  // Volume increases with temperature
  }

  /***************************************************************************
   * Tank Selection and Priority Tests
   ***************************************************************************/

  // Test tank selection
  void testTankSelection() {
    TankState tank1, tank2;
    tank1.selected = true;
    tank1.contents = 100.0;
    tank2.selected = false;
    tank2.contents = 200.0;

    double drawFromTank1 = tank1.selected ? 50.0 : 0.0;
    double drawFromTank2 = tank2.selected ? 50.0 : 0.0;

    TS_ASSERT_DELTA(drawFromTank1, 50.0, epsilon);
    TS_ASSERT_DELTA(drawFromTank2, 0.0, epsilon);
  }

  // Test priority-based consumption
  void testPriorityConsumption() {
    TankState tanks[3];
    tanks[0].priority = 1.0;
    tanks[0].contents = 100.0;
    tanks[1].priority = 2.0;
    tanks[1].contents = 100.0;
    tanks[2].priority = 3.0;
    tanks[2].contents = 100.0;

    // Find highest priority (lowest number) tank with fuel
    int selectedTank = -1;
    double highestPriority = 999.0;
    for (int i = 0; i < 3; i++) {
      if (tanks[i].contents > 0 && tanks[i].priority < highestPriority) {
        highestPriority = tanks[i].priority;
        selectedTank = i;
      }
    }

    TS_ASSERT_EQUALS(selectedTank, 0);  // Tank 0 has highest priority
  }

  /***************************************************************************
   * Fuel Types Tests
   ***************************************************************************/

  // Test different fuel densities
  void testFuelTypeDensities() {
    double jetADensity = 6.02;    // lbs/gal
    double avgasDensity = 6.00;   // lbs/gal
    double jp4Density = 6.36;     // lbs/gal

    TS_ASSERT(jp4Density > jetADensity);
    TS_ASSERT(jetADensity > avgasDensity);
  }

  // Test fuel weight for fixed volume
  void testFuelWeightFixedVolume() {
    double volume = 100.0;  // gallons
    double jetADensity = 6.02;
    double avgasDensity = 6.00;

    double jetAWeight = volume * jetADensity;
    double avgasWeight = volume * avgasDensity;

    TS_ASSERT_DELTA(jetAWeight, 602.0, epsilon);
    TS_ASSERT_DELTA(avgasWeight, 600.0, epsilon);
  }

  /***************************************************************************
   * Pressurization Tests
   ***************************************************************************/

  // Test tank pressure (simplified)
  void testTankPressure() {
    double altitude = 35000.0;  // ft
    double seaLevelPressure = 14.7;  // psi
    double ambientPressure = seaLevelPressure * std::exp(-altitude / 27000.0);

    // Tank pressurized to maintain differential
    double differential = 2.0;  // psi
    double tankPressure = ambientPressure + differential;

    TS_ASSERT(tankPressure > ambientPressure);
  }

  // Test fuel boiling point vs pressure
  void testFuelBoilingCheck() {
    double fuelTemp = 320.0;  // K
    double boilingPoint = 373.0;  // K at sea level pressure

    bool isSafe = fuelTemp < boilingPoint;
    TS_ASSERT(isSafe);
  }

  /***************************************************************************
   * Inertia Tests
   ***************************************************************************/

  // Test moment of inertia contribution
  void testMomentOfInertia() {
    TankState tank;
    tank.contents = 100.0;  // lbs
    tank.x = 10.0;          // ft from reference

    // Contribution to Ixx (roll) assuming tank is at distance r from roll axis
    double r = 5.0;  // ft from roll axis
    double Ixx = tank.contents * r * r;

    TS_ASSERT_DELTA(Ixx, 2500.0, epsilon);  // 100 * 5^2 = 2500 slug-ft^2
  }

  // Test inertia change during fuel burn
  void testInertiaChange() {
    TankState tank;
    tank.contents = 200.0;
    double r = 10.0;

    double I1 = tank.contents * r * r;

    tank.contents = 100.0;
    double I2 = tank.contents * r * r;

    TS_ASSERT(I2 < I1);  // Inertia decreases with fuel burn
    TS_ASSERT_DELTA(I2, I1 / 2.0, epsilon);  // Half the fuel, half the inertia
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test empty tank
  void testEmptyTank() {
    TankState tank;
    tank.contents = 0.0;
    tank.unusable = 5.0;

    double usable = std::max(0.0, tank.contents - tank.unusable);
    TS_ASSERT_DELTA(usable, 0.0, epsilon);
  }

  // Test full tank
  void testFullTank() {
    TankState tank;
    tank.contents = 1000.0;
    tank.capacity = 1000.0;

    bool isFull = (tank.contents >= tank.capacity);
    TS_ASSERT(isFull);
  }

  // Test overfill protection
  void testOverfillProtection() {
    TankState tank;
    tank.contents = 950.0;
    tank.capacity = 1000.0;

    double fuelToAdd = 100.0;
    double actualAdded = std::min(fuelToAdd, tank.capacity - tank.contents);
    tank.contents += actualAdded;

    TS_ASSERT_DELTA(tank.contents, 1000.0, epsilon);  // Capped at capacity
    TS_ASSERT_DELTA(actualAdded, 50.0, epsilon);      // Only added 50
  }

  // Test very small fuel quantity
  void testSmallFuelQuantity() {
    TankState tank;
    tank.contents = 0.001;  // Very small amount
    tank.unusable = 5.0;

    double usable = std::max(0.0, tank.contents - tank.unusable);
    TS_ASSERT_DELTA(usable, 0.0, epsilon);  // Below unusable threshold
  }

  /***************************************************************************
   * Fuel Slosh and Dynamics Tests
   ***************************************************************************/

  // Test 32: Fuel slosh due to longitudinal acceleration
  void testFuelSloshLongitudinal() {
    double fuelMass = 500.0;  // lbs
    double tankLength = 4.0;  // ft
    double accelG = 0.3;      // 0.3g forward acceleration

    // Simplified model: fuel shifts toward rear under forward accel
    // CG shift proportional to acceleration
    double cgShift = accelG * tankLength * 0.1;  // 10% of tank length per g
    TS_ASSERT_DELTA(cgShift, 0.12, 0.01);  // ft aft shift
  }

  // Test 33: Fuel slosh due to lateral acceleration
  void testFuelSloshLateral() {
    double fuelMass = 300.0;  // lbs
    double tankWidth = 2.0;   // ft
    double lateralG = 0.5;    // 0.5g in turn

    double cgShift = lateralG * tankWidth * 0.1;
    TS_ASSERT_DELTA(cgShift, 0.1, 0.01);  // ft lateral shift
  }

  // Test 34: Fuel slosh damping over time
  void testFuelSloshDamping() {
    double initialSlosh = 1.0;  // ft displacement
    double dampingFactor = 0.95;  // Per timestep
    double slosh = initialSlosh;

    // Damping over 50 timesteps
    for (int i = 0; i < 50; i++) {
      slosh *= dampingFactor;
    }

    TS_ASSERT(slosh < 0.1);  // Slosh should damp out
  }

  // Test 35: Fuel surge during rapid deceleration
  void testFuelSurgeDeceleration() {
    double accelG = -0.5;  // Braking
    double tankLength = 3.0;

    // Forward surge during braking
    double surge = std::abs(accelG) * tankLength * 0.15;
    TS_ASSERT_DELTA(surge, 0.225, 0.01);  // ft forward
  }

  // Test 36: Centrifugal effect on wing tanks in turn
  void testCentrifugalWingTank() {
    double bankAngle = 30.0 * M_PI / 180.0;
    double loadFactor = 1.0 / std::cos(bankAngle);

    // Fuel experiences increased load factor in turn
    TS_ASSERT_DELTA(loadFactor, 1.155, 0.001);
  }

  /***************************************************************************
   * Fuel Venting and Pressurization Tests
   ***************************************************************************/

  // Test 37: Tank vent valve operation
  void testVentValve() {
    double tankPressure = 3.0;  // psi gauge
    double ventOpenPressure = 2.5;  // psi
    double ventClosePressure = 2.0;  // psi

    bool ventOpen = (tankPressure > ventOpenPressure);
    TS_ASSERT(ventOpen);
  }

  // Test 38: Pressure relief during climb
  void testPressureReliefClimb() {
    double seaLevelPressure = 14.7;  // psi
    double altitude = 10000.0;  // ft
    double ambientPressure = seaLevelPressure * std::exp(-altitude / 27000.0);
    double differentialMax = 3.0;  // psi max differential

    double maxTankPressure = ambientPressure + differentialMax;
    TS_ASSERT(maxTankPressure < seaLevelPressure);  // Less than sea level
  }

  // Test 39: Negative pressure protection during descent
  void testNegativePressureProtection() {
    double tankPressure = 10.0;  // psi
    double ambientPressure = 12.0;  // Rising ambient during descent
    double minDifferential = -0.5;  // Minimum differential

    double actualDifferential = tankPressure - ambientPressure;
    bool needsVenting = actualDifferential < minDifferential;
    TS_ASSERT(needsVenting);
  }

  // Test 40: Ullage volume calculation
  void testUllageVolume() {
    double tankCapacityGal = 200.0;
    double fuelGallons = 150.0;
    double ullagePercent = (tankCapacityGal - fuelGallons) / tankCapacityGal * 100.0;

    TS_ASSERT_DELTA(ullagePercent, 25.0, epsilon);
  }

  /***************************************************************************
   * Boost Pump Tests
   ***************************************************************************/

  // Test 41: Boost pump pressure contribution
  void testBoostPumpPressure() {
    double gravityHead = 2.0;  // psi from tank height
    double boostPumpPressure = 15.0;  // psi from pump

    double totalFeedPressure = gravityHead + boostPumpPressure;
    TS_ASSERT_DELTA(totalFeedPressure, 17.0, epsilon);
  }

  // Test 42: Boost pump failure - gravity feed only
  void testBoostPumpFailure() {
    double tankHeight = 5.0;  // ft above engine
    double fuelDensity = 6.02;  // lbs/gal
    double gravityHeadPsi = (tankHeight * fuelDensity / 231.0) * 0.433;

    // Gravity head alone - verify it's positive
    TS_ASSERT(gravityHeadPsi > 0);
  }

  // Test 43: Minimum fuel pressure for engine
  void testMinimumFuelPressure() {
    double feedPressure = 12.0;  // psi
    double minRequired = 8.0;    // psi minimum for engine

    bool pressureOk = feedPressure >= minRequired;
    TS_ASSERT(pressureOk);
  }

  // Test 44: High altitude boost pump requirement
  void testHighAltitudeBoostRequirement() {
    double altitude = 35000.0;
    double ambientPressurePsi = 14.7 * std::exp(-altitude / 27000.0);

    // At high altitude, boost pump prevents vapor lock
    double minBoostPressure = 14.7 - ambientPressurePsi + 5.0;  // Maintain margin
    TS_ASSERT(minBoostPressure > 5.0);
  }

  /***************************************************************************
   * Fuel Freeze Tests
   ***************************************************************************/

  // Test 45: Jet A fuel freeze point check
  void testJetAFreezePoint() {
    double fuelTemp = 233.0;  // -40°C in K
    double freezePointJetA = 233.0;  // -40°C

    bool nearFreeze = fuelTemp <= freezePointJetA + 3.0;  // 3K margin
    TS_ASSERT(nearFreeze);
  }

  // Test 46: Fuel temperature drop rate
  void testFuelTempDropRate() {
    double initialTemp = 288.0;  // 15°C
    double outsideAirTemp = 218.0;  // -55°C at altitude
    double thermalTimeConstant = 3600.0;  // seconds
    double dt = 600.0;  // 10 minutes

    // Exponential cooling
    double tempDrop = (initialTemp - outsideAirTemp) * (1.0 - std::exp(-dt / thermalTimeConstant));
    double newTemp = initialTemp - tempDrop;

    TS_ASSERT(newTemp < initialTemp);
    TS_ASSERT(newTemp > outsideAirTemp);
  }

  // Test 47: Fuel heating from recirculation
  void testFuelRecirculationHeating() {
    double fuelTemp = 250.0;  // K
    double returnFuelTemp = 280.0;  // K (heated by engine)
    double returnFlow = 0.2;  // fraction of total

    double mixedTemp = fuelTemp * (1.0 - returnFlow) + returnFuelTemp * returnFlow;
    TS_ASSERT_DELTA(mixedTemp, 256.0, 0.1);
  }

  // Test 48: Ice crystal formation risk
  void testIceCrystalRisk() {
    double fuelTemp = 235.0;  // K
    double waterContentPPM = 30.0;  // parts per million
    double freezeThreshold = 273.15;  // 0°C for water

    bool iceRisk = (fuelTemp < freezeThreshold) && (waterContentPPM > 20.0);
    TS_ASSERT(iceRisk);
  }

  /***************************************************************************
   * Fuel Jettison Tests
   ***************************************************************************/

  // Test 49: Jettison rate calculation
  void testJettisonRate() {
    double jettisonRateLbsMin = 2500.0;  // lbs/min
    double fuelToJettison = 10000.0;  // lbs

    double jettisonTime = fuelToJettison / jettisonRateLbsMin;
    TS_ASSERT_DELTA(jettisonTime, 4.0, epsilon);  // 4 minutes
  }

  // Test 50: Jettison to minimum landing weight
  void testJettisonToMinLandingWeight() {
    double currentWeight = 150000.0;  // lbs
    double maxLandingWeight = 130000.0;  // lbs
    double emptyWeight = 80000.0;
    double payload = 20000.0;

    double fuelOnBoard = currentWeight - emptyWeight - payload;
    double maxLandingFuel = maxLandingWeight - emptyWeight - payload;
    double fuelToJettison = std::max(0.0, fuelOnBoard - maxLandingFuel);

    TS_ASSERT_DELTA(fuelToJettison, 20000.0, epsilon);
  }

  // Test 51: Jettison nozzle flow rate
  void testJettisonNozzleFlow() {
    double pressureDiff = 10.0;  // psi
    double nozzleArea = 2.0;     // sq inches
    double flowCoeff = 0.65;

    // Simplified flow: Q = C * A * sqrt(2 * dP / rho)
    double flowRate = flowCoeff * nozzleArea * std::sqrt(2.0 * pressureDiff);
    TS_ASSERT(flowRate > 0);
  }

  // Test 52: Jettison cutoff at minimum fuel
  void testJettisonMinimumFuel() {
    TankState tank;
    tank.contents = 3000.0;
    double minFuelReserve = 2000.0;  // lbs reserve

    double jettisonable = std::max(0.0, tank.contents - minFuelReserve);
    TS_ASSERT_DELTA(jettisonable, 1000.0, epsilon);
  }

  /***************************************************************************
   * Refueling Tests
   ***************************************************************************/

  // Test 53: Pressure refuel rate
  void testPressureRefuelRate() {
    double refuelRateGPM = 400.0;  // gallons per minute
    double density = 6.02;

    double refuelRateLbsMin = refuelRateGPM * density;
    TS_ASSERT_DELTA(refuelRateLbsMin, 2408.0, 1.0);
  }

  // Test 54: Gravity refuel (overwing)
  void testGravityRefuelRate() {
    double gravityRefuelGPM = 30.0;  // Much slower than pressure
    double pressureRefuelGPM = 400.0;

    TS_ASSERT(gravityRefuelGPM < pressureRefuelGPM * 0.1);
  }

  // Test 55: Tank fill sequence
  void testTankFillSequence() {
    TankState tanks[3];
    tanks[0].capacity = 500.0;
    tanks[0].contents = 500.0;  // Full
    tanks[1].capacity = 1000.0;
    tanks[1].contents = 600.0;  // Partially full
    tanks[2].capacity = 500.0;
    tanks[2].contents = 0.0;    // Empty

    // Find first non-full tank
    int tankToFill = -1;
    for (int i = 0; i < 3; i++) {
      if (tanks[i].contents < tanks[i].capacity) {
        tankToFill = i;
        break;
      }
    }

    TS_ASSERT_EQUALS(tankToFill, 1);  // Tank 1 is first non-full
  }

  // Test 56: Defuel operation
  void testDefuelOperation() {
    TankState tank;
    tank.contents = 800.0;
    double defuelRate = 300.0;  // lbs/min
    double defuelTime = 2.0;    // minutes

    tank.contents -= defuelRate * defuelTime;
    TS_ASSERT_DELTA(tank.contents, 200.0, epsilon);
  }

  /***************************************************************************
   * Range and Endurance Tests
   ***************************************************************************/

  // Test 57: Specific range calculation
  void testSpecificRange() {
    double groundSpeed = 450.0;  // knots
    double fuelFlow = 2500.0;    // lbs/hr

    double specificRange = groundSpeed / fuelFlow;  // nm/lb
    TS_ASSERT_DELTA(specificRange, 0.18, 0.001);
  }

  // Test 58: Maximum range fuel
  void testMaxRangeFuel() {
    double totalFuel = 10000.0;
    double reserveFuel = 2000.0;  // 45 min reserve
    double taxiFuel = 200.0;

    double tripFuel = totalFuel - reserveFuel - taxiFuel;
    TS_ASSERT_DELTA(tripFuel, 7800.0, epsilon);
  }

  // Test 59: Endurance calculation
  void testEnduranceCalculation() {
    double usableFuel = 5000.0;  // lbs
    double fuelFlow = 800.0;     // lbs/hr

    double endurance = usableFuel / fuelFlow;
    TS_ASSERT_DELTA(endurance, 6.25, epsilon);  // hours
  }

  // Test 60: Range vs payload tradeoff
  void testRangePayloadTradeoff() {
    double maxTakeoffWeight = 100000.0;
    double emptyWeight = 60000.0;
    double maxFuel = 30000.0;
    double maxPayload = 25000.0;

    // With max fuel, payload limited
    double payloadWithMaxFuel = maxTakeoffWeight - emptyWeight - maxFuel;
    TS_ASSERT_DELTA(payloadWithMaxFuel, 10000.0, epsilon);
    TS_ASSERT(payloadWithMaxFuel < maxPayload);
  }

  /***************************************************************************
   * Multi-Tank Balancing Tests
   ***************************************************************************/

  // Test 61: Automatic tank balancing
  void testAutomaticBalancing() {
    TankState leftWing, rightWing;
    leftWing.contents = 600.0;
    leftWing.y = -10.0;
    rightWing.contents = 400.0;
    rightWing.y = 10.0;

    double imbalance = leftWing.contents - rightWing.contents;
    double transferNeeded = imbalance / 2.0;

    // Transfer to balance
    leftWing.contents -= transferNeeded;
    rightWing.contents += transferNeeded;

    TS_ASSERT_DELTA(leftWing.contents, rightWing.contents, epsilon);
  }

  // Test 62: Imbalance limit check
  void testImbalanceLimit() {
    double leftFuel = 550.0;
    double rightFuel = 400.0;
    double maxImbalance = 200.0;

    double imbalance = std::abs(leftFuel - rightFuel);
    bool withinLimits = imbalance <= maxImbalance;

    TS_ASSERT(withinLimits);
  }

  // Test 63: CG-based fuel sequencing
  void testCGBasedSequencing() {
    TankState fwdTank, aftTank, centerTank;
    fwdTank.contents = 200.0;
    fwdTank.x = 5.0;
    aftTank.contents = 200.0;
    aftTank.x = 25.0;
    centerTank.contents = 400.0;
    centerTank.x = 15.0;

    // Current CG
    double totalFuel = fwdTank.contents + aftTank.contents + centerTank.contents;
    double cgX = (fwdTank.contents * fwdTank.x +
                  aftTank.contents * aftTank.x +
                  centerTank.contents * centerTank.x) / totalFuel;

    double targetCG = 15.0;
    double cgError = cgX - targetCG;

    TS_ASSERT_DELTA(cgX, 15.0, 0.1);  // Should be near target
  }

  // Test 64: Crossfeed valve operation
  void testCrossfeedValve() {
    TankState leftTank, rightTank;
    leftTank.contents = 500.0;
    rightTank.contents = 200.0;
    bool crossfeedOpen = true;

    // With crossfeed open, engines can draw from either side
    double leftDraw = crossfeedOpen ? 0.0 : 50.0;  // Left engine can draw from right
    double rightDraw = crossfeedOpen ? 50.0 : 0.0;  // Both draw from right

    if (crossfeedOpen) {
      // Single engine feeding from low tank
      rightTank.contents -= 50.0;
    }

    TS_ASSERT_DELTA(rightTank.contents, 150.0, epsilon);
  }

  /***************************************************************************
   * Fuel Quantity Indication Tests
   ***************************************************************************/

  // Test 65: Fuel quantity sensor accuracy
  void testFuelQuantitySensorAccuracy() {
    double actualFuel = 1000.0;
    double sensorAccuracy = 0.02;  // 2% accuracy

    double maxError = actualFuel * sensorAccuracy;
    double indicatedFuel = actualFuel + maxError;  // Worst case high

    TS_ASSERT_DELTA(indicatedFuel, 1020.0, epsilon);
  }

  // Test 66: Low fuel warning threshold
  void testLowFuelWarning() {
    TankState tank;
    tank.contents = 150.0;
    double lowFuelWarningThreshold = 200.0;

    bool lowFuelWarning = tank.contents < lowFuelWarningThreshold;
    TS_ASSERT(lowFuelWarning);
  }

  // Test 67: Fuel quantity totalizer
  void testFuelTotalizer() {
    double fuelAtStart = 5000.0;
    double fuelFlowIntegrated = 1250.0;  // From flow sensors

    double computedRemaining = fuelAtStart - fuelFlowIntegrated;
    TS_ASSERT_DELTA(computedRemaining, 3750.0, epsilon);
  }

  // Test 68: Fuel density compensation
  void testFuelDensityCompensation() {
    double volumeGallons = 100.0;
    double standardDensity = 6.02;
    double actualDensity = 5.95;  // Warm fuel

    double standardWeight = volumeGallons * standardDensity;
    double actualWeight = volumeGallons * actualDensity;
    double error = standardWeight - actualWeight;

    TS_ASSERT_DELTA(error, 7.0, 0.1);  // 7 lbs overestimate
  }

  /***************************************************************************
   * Fuel System Failure Tests
   ***************************************************************************/

  // Test 69: Single point fuel leak
  void testFuelLeakDetection() {
    double fuelAtTime0 = 5000.0;
    double fuelAtTime1 = 4900.0;  // 10 minutes later
    double expectedBurn = 80.0;   // Normal consumption

    double actualLoss = fuelAtTime0 - fuelAtTime1;
    double unexplainedLoss = actualLoss - expectedBurn;

    bool possibleLeak = unexplainedLoss > 10.0;  // 10 lbs threshold
    TS_ASSERT(possibleLeak);
  }

  // Test 70: Blocked fuel line
  void testBlockedFuelLine() {
    double requestedFlow = 500.0;  // lbs/hr
    double actualFlow = 100.0;     // Restricted

    double flowEfficiency = actualFlow / requestedFlow;
    bool lineBlocked = flowEfficiency < 0.5;

    TS_ASSERT(lineBlocked);
  }

  // Test 71: Water contamination detection
  void testWaterContamination() {
    double fuelDensity = 6.02;
    double sampleDensity = 6.15;  // Higher than expected
    double waterDensity = 8.34;

    // Water is denser than fuel - sinks to bottom
    bool possibleWater = sampleDensity > fuelDensity * 1.01;
    TS_ASSERT(possibleWater);
  }

  /***************************************************************************
   * Advanced Fuel Management Tests
   ***************************************************************************/

  // Test 72: Fuel temperature stratification
  void testFuelStratification() {
    double bottomTemp = 250.0;  // K (colder, denser)
    double topTemp = 260.0;     // K (warmer)
    double avgTemp = (bottomTemp + topTemp) / 2.0;

    TS_ASSERT_DELTA(avgTemp, 255.0, epsilon);
    TS_ASSERT(bottomTemp < topTemp);  // Cold fuel sinks
  }

  // Test 73: Fuel heating for viscosity control
  void testFuelViscosityControl() {
    double coldViscosity = 8.0;   // cSt at -20°C
    double warmViscosity = 2.5;   // cSt at 20°C
    double maxAllowedViscosity = 12.0;

    bool viscosityOk = coldViscosity < maxAllowedViscosity;
    TS_ASSERT(viscosityOk);
    TS_ASSERT(warmViscosity < coldViscosity);
  }

  // Test 74: Fuel system priming
  void testFuelSystemPriming() {
    double lineVolume = 2.0;  // gallons
    double primeFlowRate = 5.0;  // gal/min
    double primeTime = lineVolume / primeFlowRate;

    TS_ASSERT_DELTA(primeTime, 0.4, epsilon);  // 0.4 minutes = 24 seconds
  }

  // Test 75: Fuel trim tank operation
  void testTrimTankOperation() {
    TankState mainTanks, trimTank;
    mainTanks.contents = 8000.0;
    mainTanks.x = 15.0;
    trimTank.contents = 500.0;
    trimTank.x = 50.0;  // Far aft

    // CG with trim fuel
    double totalFuel = mainTanks.contents + trimTank.contents;
    double cgWithTrim = (mainTanks.contents * mainTanks.x +
                         trimTank.contents * trimTank.x) / totalFuel;

    // CG without trim fuel
    double cgWithoutTrim = mainTanks.x;

    TS_ASSERT(cgWithTrim > cgWithoutTrim);  // Trim moves CG aft
  }
};
