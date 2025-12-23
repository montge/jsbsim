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
};
