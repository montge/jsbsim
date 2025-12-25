/*******************************************************************************
 * FGFuelSystemTest.h - Unit tests for fuel system calculations
 *
 * Tests the mathematical and physical behavior of complete fuel systems:
 * - Fuel flow rate calculations
 * - Tank capacity and contents
 * - Fuel transfer operations
 * - Fuel consumption vs power settings
 * - Center of gravity effects from fuel burn
 * - Fuel density variations with temperature
 * - Unusable fuel calculations
 * - Cross-feed operations
 * - Fuel quantity gauging
 * - Fuel pump pressure dynamics
 * - Temperature effects on flow
 * - Multi-engine fuel balancing
 * - Gravity feed vs pressure feed
 * - Fuel jettison calculations
 *
 * This test suite focuses on the fluid mechanics and mass balance aspects
 * of integrated fuel systems, building upon FGTank and FGEngine tests.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <vector>
#include <algorithm>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-10;

class FGFuelSystemTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Support Structures
   ***************************************************************************/

  struct Tank {
    double contents = 0.0;      // lbs
    double capacity = 1000.0;   // lbs
    double unusable = 5.0;      // lbs
    double density = 6.02;      // lbs/gal (Jet-A)
    double temperature = 288.15; // K
    double pressure = 14.7;     // psi
    double x = 0.0;             // ft - CG location
    double y = 0.0;             // ft
    double z = 0.0;             // ft
    bool selected = true;
    int priority = 1;
    std::string name;
  };

  struct Engine {
    double fuelFlowRate = 0.0;  // lbs/sec
    double powerSetting = 0.0;  // 0.0 to 1.0
    double maxFuelFlow = 1.0;   // lbs/sec at max power
    bool running = false;
    bool starved = false;
    int tankIndex = 0;          // Primary tank
  };

  struct FuelPump {
    double flowRate = 0.0;      // lbs/sec
    double pressure = 0.0;      // psi
    double maxPressure = 50.0;  // psi
    bool active = false;
    double efficiency = 0.95;
  };

  /***************************************************************************
   * Fuel Flow Rate Calculations
   ***************************************************************************/

  // Test basic fuel flow rate at full power
  void testFuelFlowRateFullPower() {
    Engine engine;
    engine.maxFuelFlow = 2.0;    // lbs/sec
    engine.powerSetting = 1.0;   // 100%
    engine.running = true;

    engine.fuelFlowRate = engine.maxFuelFlow * engine.powerSetting;
    TS_ASSERT_DELTA(engine.fuelFlowRate, 2.0, epsilon);
  }

  // Test fuel flow rate at partial power
  void testFuelFlowRatePartialPower() {
    Engine engine;
    engine.maxFuelFlow = 2.0;
    engine.powerSetting = 0.75;
    engine.running = true;

    engine.fuelFlowRate = engine.maxFuelFlow * engine.powerSetting;
    TS_ASSERT_DELTA(engine.fuelFlowRate, 1.5, epsilon);
  }

  // Test fuel flow rate at idle
  void testFuelFlowRateIdle() {
    Engine engine;
    engine.maxFuelFlow = 2.0;
    engine.powerSetting = 0.15;  // Idle ~15%
    engine.running = true;

    engine.fuelFlowRate = engine.maxFuelFlow * engine.powerSetting;
    TS_ASSERT_DELTA(engine.fuelFlowRate, 0.3, epsilon);
  }

  // Test zero flow when engine stopped
  void testZeroFlowEngineStopped() {
    Engine engine;
    engine.running = false;
    engine.fuelFlowRate = 0.0;

    TS_ASSERT_DELTA(engine.fuelFlowRate, 0.0, epsilon);
  }

  // Test fuel flow in pounds per hour
  void testFuelFlowPoundsPerHour() {
    double fuelFlowRate = 1.5;  // lbs/sec
    double fuelFlowPPH = fuelFlowRate * 3600.0;  // lbs/hr

    TS_ASSERT_DELTA(fuelFlowPPH, 5400.0, epsilon);
  }

  // Test fuel flow in gallons per hour
  void testFuelFlowGallonsPerHour() {
    double fuelFlowRate = 1.5;  // lbs/sec
    double density = 6.02;      // lbs/gal
    double fuelFlowGPH = (fuelFlowRate * 3600.0) / density;

    TS_ASSERT_DELTA(fuelFlowGPH, 897.01, 0.01);
  }

  /***************************************************************************
   * Tank Capacity and Contents
   ***************************************************************************/

  // Test tank fill percentage
  void testTankFillPercentage() {
    Tank tank;
    tank.contents = 750.0;
    tank.capacity = 1000.0;

    double fillPct = (tank.contents / tank.capacity) * 100.0;
    TS_ASSERT_DELTA(fillPct, 75.0, epsilon);
  }

  // Test multiple tank total capacity
  void testMultipleTankTotalCapacity() {
    std::vector<Tank> tanks(3);
    tanks[0].capacity = 1000.0;
    tanks[1].capacity = 1500.0;
    tanks[2].capacity = 800.0;

    double totalCapacity = 0.0;
    for (const auto& tank : tanks) {
      totalCapacity += tank.capacity;
    }

    TS_ASSERT_DELTA(totalCapacity, 3300.0, epsilon);
  }

  // Test total fuel on board
  void testTotalFuelOnBoard() {
    std::vector<Tank> tanks(3);
    tanks[0].contents = 800.0;
    tanks[1].contents = 1200.0;
    tanks[2].contents = 600.0;

    double totalFuel = 0.0;
    for (const auto& tank : tanks) {
      totalFuel += tank.contents;
    }

    TS_ASSERT_DELTA(totalFuel, 2600.0, epsilon);
  }

  // Test usable fuel calculation
  void testUsableFuelCalculation() {
    Tank tank;
    tank.contents = 150.0;
    tank.unusable = 10.0;

    double usable = std::max(0.0, tank.contents - tank.unusable);
    TS_ASSERT_DELTA(usable, 140.0, epsilon);
  }

  // Test total usable fuel across multiple tanks
  void testTotalUsableFuel() {
    std::vector<Tank> tanks(2);
    tanks[0].contents = 500.0;
    tanks[0].unusable = 10.0;
    tanks[1].contents = 800.0;
    tanks[1].unusable = 15.0;

    double totalUsable = 0.0;
    for (const auto& tank : tanks) {
      totalUsable += std::max(0.0, tank.contents - tank.unusable);
    }

    // (500-10) + (800-15) = 490 + 785 = 1275
    TS_ASSERT_DELTA(totalUsable, 1275.0, epsilon);
  }

  /***************************************************************************
   * Fuel Transfer Between Tanks
   ***************************************************************************/

  // Test simple fuel transfer
  void testSimpleFuelTransfer() {
    Tank source, dest;
    source.contents = 500.0;
    dest.contents = 300.0;

    double transferRate = 100.0;  // lbs/min
    double dt = 0.5;              // minutes

    double transferred = transferRate * dt;
    source.contents -= transferred;
    dest.contents += transferred;

    TS_ASSERT_DELTA(source.contents, 450.0, epsilon);
    TS_ASSERT_DELTA(dest.contents, 350.0, epsilon);
  }

  // Test transfer limited by source unusable fuel
  void testTransferLimitedByUnusable() {
    Tank source, dest;
    source.contents = 60.0;
    source.unusable = 10.0;
    dest.contents = 200.0;

    double transferRate = 100.0;
    double dt = 1.0;  // Would transfer 100 lbs

    double available = source.contents - source.unusable;
    double transferred = std::min(transferRate * dt, available);

    source.contents -= transferred;
    dest.contents += transferred;

    TS_ASSERT_DELTA(source.contents, 10.0, epsilon);
    TS_ASSERT_DELTA(dest.contents, 250.0, epsilon);
  }

  // Test transfer limited by destination capacity
  void testTransferLimitedByCapacity() {
    Tank source, dest;
    source.contents = 1000.0;
    dest.contents = 950.0;
    dest.capacity = 1000.0;

    double transferRate = 100.0;
    double dt = 1.0;

    double room = dest.capacity - dest.contents;
    double transferred = std::min(transferRate * dt, room);

    source.contents -= transferred;
    dest.contents += transferred;

    TS_ASSERT_DELTA(source.contents, 950.0, epsilon);
    TS_ASSERT_DELTA(dest.contents, 1000.0, epsilon);
  }

  // Test bidirectional transfer capability
  void testBidirectionalTransfer() {
    Tank tank1, tank2;
    tank1.contents = 600.0;
    tank2.contents = 400.0;

    // Transfer from tank1 to tank2
    double transfer1to2 = 50.0;
    tank1.contents -= transfer1to2;
    tank2.contents += transfer1to2;

    TS_ASSERT_DELTA(tank1.contents, 550.0, epsilon);
    TS_ASSERT_DELTA(tank2.contents, 450.0, epsilon);

    // Transfer back from tank2 to tank1
    double transfer2to1 = 30.0;
    tank2.contents -= transfer2to1;
    tank1.contents += transfer2to1;

    TS_ASSERT_DELTA(tank1.contents, 580.0, epsilon);
    TS_ASSERT_DELTA(tank2.contents, 420.0, epsilon);
  }

  /***************************************************************************
   * Fuel Consumption vs Power Setting
   ***************************************************************************/

  // Test fuel consumption at cruise power
  void testFuelConsumptionCruise() {
    Engine engine;
    engine.maxFuelFlow = 2.0;     // lbs/sec at max
    engine.powerSetting = 0.65;   // 65% cruise
    engine.running = true;

    engine.fuelFlowRate = engine.maxFuelFlow * engine.powerSetting;

    double dt = 60.0;  // seconds
    double consumed = engine.fuelFlowRate * dt;

    TS_ASSERT_DELTA(consumed, 78.0, epsilon);
  }

  // Test fuel consumption during climb
  void testFuelConsumptionClimb() {
    Engine engine;
    engine.maxFuelFlow = 2.0;
    engine.powerSetting = 0.90;   // 90% climb power
    engine.running = true;

    engine.fuelFlowRate = engine.maxFuelFlow * engine.powerSetting;
    TS_ASSERT_DELTA(engine.fuelFlowRate, 1.8, epsilon);
  }

  // Test specific range calculation
  void testSpecificRange() {
    // Specific range = TAS / fuel flow rate
    double tas = 450.0;         // kts
    double fuelFlowPPH = 5000.0; // lbs/hr

    double specificRange = tas / fuelFlowPPH;  // nm/lb
    TS_ASSERT_DELTA(specificRange, 0.09, 0.001);

    // Or nautical miles per pound
    double nmPerLb = specificRange;
    TS_ASSERT(nmPerLb > 0.0);
  }

  // Test endurance calculation
  void testEnduranceCalculation() {
    double totalFuel = 10000.0;   // lbs
    double fuelFlowPPH = 4000.0;  // lbs/hr

    double endurance = totalFuel / fuelFlowPPH;  // hours
    TS_ASSERT_DELTA(endurance, 2.5, epsilon);
  }

  /***************************************************************************
   * Center of Gravity Shift from Fuel Burn
   ***************************************************************************/

  // Test CG calculation with fuel
  void testCGWithFuel() {
    Tank fwdTank, aftTank;
    fwdTank.contents = 300.0;
    fwdTank.x = 10.0;
    aftTank.contents = 300.0;
    aftTank.x = 30.0;

    double totalMass = fwdTank.contents + aftTank.contents;
    double moment = fwdTank.contents * fwdTank.x + aftTank.contents * aftTank.x;
    double cgX = moment / totalMass;

    TS_ASSERT_DELTA(cgX, 20.0, epsilon);  // Centered
  }

  // Test CG shift after forward tank burn
  void testCGShiftForwardBurn() {
    Tank fwdTank, aftTank;
    fwdTank.contents = 400.0;
    fwdTank.x = 15.0;
    aftTank.contents = 400.0;
    aftTank.x = 25.0;

    double cgInitial = (fwdTank.contents * fwdTank.x + aftTank.contents * aftTank.x) /
                       (fwdTank.contents + aftTank.contents);
    TS_ASSERT_DELTA(cgInitial, 20.0, epsilon);

    // Burn 200 lbs from forward tank
    fwdTank.contents -= 200.0;

    double cgFinal = (fwdTank.contents * fwdTank.x + aftTank.contents * aftTank.x) /
                     (fwdTank.contents + aftTank.contents);

    // CG moves aft
    TS_ASSERT(cgFinal > cgInitial);
    TS_ASSERT_DELTA(cgFinal, 21.667, 0.001);
  }

  // Test lateral CG from asymmetric fuel
  void testLateralCGAsymmetric() {
    Tank leftWing, rightWing;
    leftWing.contents = 500.0;
    leftWing.y = -15.0;
    rightWing.contents = 300.0;
    rightWing.y = 15.0;

    double totalMass = leftWing.contents + rightWing.contents;
    double momentY = leftWing.contents * leftWing.y + rightWing.contents * rightWing.y;
    double cgY = momentY / totalMass;

    TS_ASSERT_DELTA(cgY, -3.75, epsilon);  // Left of centerline
  }

  // Test 3D CG calculation
  void test3DCGCalculation() {
    std::vector<Tank> tanks(3);
    tanks[0].contents = 200.0; tanks[0].x = 10.0; tanks[0].y = 0.0; tanks[0].z = -5.0;
    tanks[1].contents = 300.0; tanks[1].x = 15.0; tanks[1].y = -10.0; tanks[1].z = -5.0;
    tanks[2].contents = 300.0; tanks[2].x = 15.0; tanks[2].y = 10.0; tanks[2].z = -5.0;

    double totalMass = 0.0;
    double momentX = 0.0, momentY = 0.0, momentZ = 0.0;

    for (const auto& tank : tanks) {
      totalMass += tank.contents;
      momentX += tank.contents * tank.x;
      momentY += tank.contents * tank.y;
      momentZ += tank.contents * tank.z;
    }

    double cgX = momentX / totalMass;
    double cgY = momentY / totalMass;
    double cgZ = momentZ / totalMass;

    TS_ASSERT_DELTA(cgX, 13.75, 0.01);
    TS_ASSERT_DELTA(cgY, 0.0, epsilon);  // Symmetric
    TS_ASSERT_DELTA(cgZ, -5.0, epsilon);
  }

  /***************************************************************************
   * Fuel Density Variations with Temperature
   ***************************************************************************/

  // Test fuel density at standard temperature
  void testFuelDensityStandard() {
    Tank tank;
    tank.density = 6.02;  // Jet-A at 15°C

    TS_ASSERT_DELTA(tank.density, 6.02, epsilon);
  }

  // Test fuel density temperature correction
  void testFuelDensityTemperatureCorrection() {
    double baseDensity = 6.02;    // lbs/gal at 288K
    double baseTemp = 288.15;     // K
    double alpha = -0.00055;      // Thermal expansion coefficient

    // At higher temperature (308K)
    double actualTemp = 308.15;
    double densityHot = baseDensity * (1.0 + alpha * (actualTemp - baseTemp));

    TS_ASSERT(densityHot < baseDensity);
    TS_ASSERT_DELTA(densityHot, 5.953, 0.01);
  }

  // Test fuel volume change with temperature
  void testFuelVolumeExpansion() {
    double mass = 6020.0;  // lbs (constant)
    double coldDensity = 6.10;  // lbs/gal at cold temp
    double hotDensity = 5.95;   // lbs/gal at hot temp

    double volumeCold = mass / coldDensity;
    double volumeHot = mass / hotDensity;

    TS_ASSERT(volumeHot > volumeCold);
    TS_ASSERT_DELTA(volumeCold, 986.89, 0.1);
    TS_ASSERT_DELTA(volumeHot, 1011.76, 0.1);
  }

  // Test mass invariance with temperature
  void testMassInvarianceTemperature() {
    double volume = 1000.0;  // gallons
    double coldDensity = 6.10;
    double hotDensity = 5.95;

    double massCold = volume * coldDensity;
    double massHot = volume * hotDensity;

    // Same volume, different mass
    TS_ASSERT_DELTA(massCold, 6100.0, epsilon);
    TS_ASSERT_DELTA(massHot, 5950.0, epsilon);
  }

  /***************************************************************************
   * Unusable Fuel Calculations
   ***************************************************************************/

  // Test unusable fuel threshold
  void testUnusableFuelThreshold() {
    Tank tank;
    tank.contents = 8.0;
    tank.unusable = 10.0;

    double usable = std::max(0.0, tank.contents - tank.unusable);
    TS_ASSERT_DELTA(usable, 0.0, epsilon);
  }

  // Test unusable fuel percentage
  void testUnusableFuelPercentage() {
    Tank tank;
    tank.capacity = 1000.0;
    tank.unusable = 15.0;

    double unusablePct = (tank.unusable / tank.capacity) * 100.0;
    TS_ASSERT_DELTA(unusablePct, 1.5, epsilon);
  }

  // Test flow stops at unusable level
  void testFlowStopsAtUnusable() {
    Tank tank;
    tank.contents = 12.0;
    tank.unusable = 10.0;

    Engine engine;
    engine.fuelFlowRate = 0.1;  // lbs/sec
    double dt = 30.0;  // seconds

    double requested = engine.fuelFlowRate * dt;  // 3.0 lbs
    double available = std::max(0.0, tank.contents - tank.unusable);  // 2.0 lbs
    double actual = std::min(requested, available);

    tank.contents -= actual;

    TS_ASSERT_DELTA(tank.contents, 10.0, epsilon);
    TS_ASSERT_DELTA(actual, 2.0, epsilon);
  }

  /***************************************************************************
   * Cross-feed Operations
   ***************************************************************************/

  // Test cross-feed configuration
  void testCrossFeedConfiguration() {
    std::vector<Engine> engines(2);
    std::vector<Tank> tanks(2);

    tanks[0].contents = 500.0;
    tanks[1].contents = 500.0;

    // Normal: Engine 0 feeds from Tank 0, Engine 1 feeds from Tank 1
    engines[0].tankIndex = 0;
    engines[1].tankIndex = 1;

    TS_ASSERT_EQUALS(engines[0].tankIndex, 0);
    TS_ASSERT_EQUALS(engines[1].tankIndex, 1);

    // Cross-feed: Both engines feed from Tank 0
    engines[1].tankIndex = 0;
    TS_ASSERT_EQUALS(engines[1].tankIndex, 0);
  }

  // Test cross-feed fuel consumption
  void testCrossFeedConsumption() {
    Tank leftTank, rightTank;
    leftTank.contents = 600.0;
    rightTank.contents = 200.0;

    Engine leftEngine, rightEngine;
    leftEngine.fuelFlowRate = 1.0;  // lbs/sec
    rightEngine.fuelFlowRate = 1.0;

    double dt = 60.0;  // seconds

    // Cross-feed: both engines draw from left tank
    double leftEngineConsumption = leftEngine.fuelFlowRate * dt;
    double rightEngineConsumption = rightEngine.fuelFlowRate * dt;

    leftTank.contents -= (leftEngineConsumption + rightEngineConsumption);

    TS_ASSERT_DELTA(leftTank.contents, 480.0, epsilon);
    TS_ASSERT_DELTA(rightTank.contents, 200.0, epsilon);  // Unchanged
  }

  // Test automatic cross-feed on low fuel
  void testAutoCrossFeedLowFuel() {
    Tank leftTank, rightTank;
    leftTank.contents = 50.0;
    leftTank.unusable = 10.0;
    rightTank.contents = 500.0;

    Engine engine;
    engine.tankIndex = 0;  // Normally feeds from left

    // Check if primary tank low
    double usableLeft = leftTank.contents - leftTank.unusable;
    if (usableLeft < 100.0) {
      engine.tankIndex = 1;  // Switch to right tank
    }

    TS_ASSERT_EQUALS(engine.tankIndex, 1);
  }

  /***************************************************************************
   * Fuel Quantity Gauging
   ***************************************************************************/

  // Test fuel quantity in pounds
  void testFuelQuantityPounds() {
    Tank tank;
    tank.contents = 1234.5;

    TS_ASSERT_DELTA(tank.contents, 1234.5, epsilon);
  }

  // Test fuel quantity in gallons
  void testFuelQuantityGallons() {
    Tank tank;
    tank.contents = 602.0;  // lbs
    tank.density = 6.02;    // lbs/gal

    double gallons = tank.contents / tank.density;
    TS_ASSERT_DELTA(gallons, 100.0, epsilon);
  }

  // Test fuel quantity sensor accuracy
  void testFuelQuantitySensorAccuracy() {
    double actualFuel = 500.0;
    double sensorError = 0.02;  // ±2% accuracy

    double minReading = actualFuel * (1.0 - sensorError);
    double maxReading = actualFuel * (1.0 + sensorError);

    TS_ASSERT_DELTA(minReading, 490.0, epsilon);
    TS_ASSERT_DELTA(maxReading, 510.0, epsilon);
    TS_ASSERT(actualFuel >= minReading);
    TS_ASSERT(actualFuel <= maxReading);
  }

  // Test totalizer accuracy
  void testFuelTotalizerAccuracy() {
    double fuelUsed = 0.0;
    double fuelFlow = 100.0;  // lbs/hr
    double dt = 0.1;          // hours

    // Accumulate over time
    for (int i = 0; i < 10; i++) {
      fuelUsed += fuelFlow * dt;
    }

    TS_ASSERT_DELTA(fuelUsed, 100.0, epsilon);
  }

  /***************************************************************************
   * Fuel Pump Pressure
   ***************************************************************************/

  // Test fuel pump pressure at idle
  void testFuelPumpPressureIdle() {
    FuelPump pump;
    pump.active = true;
    pump.maxPressure = 50.0;
    double pumpSpeed = 0.3;  // 30% idle

    pump.pressure = pump.maxPressure * pumpSpeed;
    TS_ASSERT_DELTA(pump.pressure, 15.0, epsilon);
  }

  // Test fuel pump pressure at max
  void testFuelPumpPressureMax() {
    FuelPump pump;
    pump.active = true;
    pump.maxPressure = 50.0;
    double pumpSpeed = 1.0;

    pump.pressure = pump.maxPressure * pumpSpeed;
    TS_ASSERT_DELTA(pump.pressure, 50.0, epsilon);
  }

  // Test gravity feed pressure
  void testGravityFeedPressure() {
    double tankHeight = 10.0;  // ft above engine
    double fuelDensity = 6.02; // lbs/gal
    double densitySlugs = fuelDensity / 32.174 / 7.48;  // slugs/ft^3

    // Pressure = rho * g * h
    double pressure = densitySlugs * 32.174 * tankHeight;  // lbf/ft^2
    double pressurePSI = pressure / 144.0;  // Convert to psi

    TS_ASSERT(pressurePSI > 0.0);
    TS_ASSERT(pressurePSI < 10.0);  // Typical gravity feed
  }

  // Test pump efficiency effect on flow
  void testPumpEfficiencyFlow() {
    FuelPump pump;
    pump.flowRate = 2.0;      // lbs/sec theoretical
    pump.efficiency = 0.90;   // 90% efficient

    double actualFlow = pump.flowRate * pump.efficiency;
    TS_ASSERT_DELTA(actualFlow, 1.8, epsilon);
  }

  /***************************************************************************
   * Fuel Temperature Effects on Flow
   ***************************************************************************/

  // Test viscosity change with temperature
  void testViscosityTemperature() {
    // Viscosity decreases with temperature
    double viscosity15C = 1.8e-5;  // Pa·s at 15°C
    double viscosity35C = 1.2e-5;  // Pa·s at 35°C

    TS_ASSERT(viscosity35C < viscosity15C);
  }

  // Test flow rate change with temperature
  void testFlowRateTemperature() {
    // Higher temp = lower viscosity = higher flow (for same pressure)
    double baseFuelFlow = 100.0;  // lbs/min at 15°C
    double tempCorrection = 1.15; // 15% increase at higher temp

    double hotFuelFlow = baseFuelFlow * tempCorrection;
    TS_ASSERT_DELTA(hotFuelFlow, 115.0, epsilon);
  }

  // Test fuel freezing point check
  void testFuelFreezingPoint() {
    double fuelTemp = 233.15;  // K (-40°C)
    double freezePoint = 228.15;  // K (-45°C for Jet-A)

    bool frozen = (fuelTemp < freezePoint);
    TS_ASSERT(!frozen);  // Not frozen

    // At freezing point
    fuelTemp = 225.0;
    frozen = (fuelTemp < freezePoint);
    TS_ASSERT(frozen);
  }

  /***************************************************************************
   * Multi-Engine Fuel Balancing
   ***************************************************************************/

  // Test balanced fuel consumption
  void testBalancedFuelConsumption() {
    std::vector<Engine> engines(2);
    std::vector<Tank> tanks(2);

    tanks[0].contents = 1000.0;
    tanks[1].contents = 1000.0;

    engines[0].fuelFlowRate = 1.5;
    engines[0].tankIndex = 0;
    engines[1].fuelFlowRate = 1.5;
    engines[1].tankIndex = 1;

    double dt = 60.0;  // seconds

    tanks[0].contents -= engines[0].fuelFlowRate * dt;
    tanks[1].contents -= engines[1].fuelFlowRate * dt;

    TS_ASSERT_DELTA(tanks[0].contents, 910.0, epsilon);
    TS_ASSERT_DELTA(tanks[1].contents, 910.0, epsilon);
  }

  // Test fuel imbalance detection
  void testFuelImbalanceDetection() {
    Tank leftTank, rightTank;
    leftTank.contents = 800.0;
    rightTank.contents = 600.0;

    double imbalance = std::abs(leftTank.contents - rightTank.contents);
    double imbalanceThreshold = 100.0;

    bool imbalanced = (imbalance > imbalanceThreshold);
    TS_ASSERT(imbalanced);
    TS_ASSERT_DELTA(imbalance, 200.0, epsilon);
  }

  // Test fuel balancing transfer
  void testFuelBalancingTransfer() {
    Tank leftTank, rightTank;
    leftTank.contents = 900.0;
    rightTank.contents = 500.0;

    double imbalance = leftTank.contents - rightTank.contents;
    double transferAmount = imbalance / 2.0;

    leftTank.contents -= transferAmount;
    rightTank.contents += transferAmount;

    TS_ASSERT_DELTA(leftTank.contents, 700.0, epsilon);
    TS_ASSERT_DELTA(rightTank.contents, 700.0, epsilon);
  }

  /***************************************************************************
   * Gravity Feed vs Pressure Feed
   ***************************************************************************/

  // Test gravity feed from high wing
  void testGravityFeedHighWing() {
    Tank wingTank;
    wingTank.contents = 500.0;
    wingTank.z = 8.0;  // 8 ft above engine

    Engine engine;
    engine.fuelFlowRate = 1.0;

    // Gravity feed adequate if tank above engine
    bool gravityFeedOK = (wingTank.z > 0.0);
    TS_ASSERT(gravityFeedOK);
  }

  // Test pressure feed requirement for low wing
  void testPressureFeedLowWing() {
    Tank wingTank;
    wingTank.contents = 500.0;
    wingTank.z = -2.0;  // Below engine

    FuelPump pump;
    pump.active = true;
    pump.pressure = 25.0;

    // Pressure feed required if tank below engine
    bool pumpRequired = (wingTank.z < 0.0);
    TS_ASSERT(pumpRequired);
    TS_ASSERT(pump.active);
  }

  // Test pump failure scenario
  void testPumpFailureScenario() {
    FuelPump pump;
    pump.active = false;  // Pump failed
    pump.pressure = 0.0;

    Tank tank;
    tank.z = -3.0;  // Below engine, needs pump

    // Engine will starve without pump
    bool canFeed = pump.active || (tank.z > 0.0);
    TS_ASSERT(!canFeed);
  }

  /***************************************************************************
   * Fuel Jettison Calculations
   ***************************************************************************/

  // Test fuel jettison rate
  void testFuelJettisonRate() {
    Tank tank;
    tank.contents = 5000.0;

    double jettisonRate = 200.0;  // lbs/min
    double dt = 1.0;  // minute

    tank.contents -= jettisonRate * dt;
    TS_ASSERT_DELTA(tank.contents, 4800.0, epsilon);
  }

  // Test jettison to target weight
  void testJettisonToTargetWeight() {
    double currentWeight = 80000.0;  // lbs
    double targetWeight = 70000.0;   // lbs
    double fuelWeight = 12000.0;

    double excessWeight = currentWeight - targetWeight;
    double fuelToJettison = std::min(excessWeight, fuelWeight);

    TS_ASSERT_DELTA(fuelToJettison, 10000.0, epsilon);
  }

  // Test jettison time calculation
  void testJettisonTimeCalculation() {
    double fuelToJettison = 8000.0;  // lbs
    double jettisonRate = 400.0;     // lbs/min

    double timeRequired = fuelToJettison / jettisonRate;  // minutes
    TS_ASSERT_DELTA(timeRequired, 20.0, epsilon);
  }

  // Test selective tank jettison
  void testSelectiveTankJettison() {
    std::vector<Tank> tanks(3);
    tanks[0].contents = 2000.0; tanks[0].name = "Center";
    tanks[1].contents = 1500.0; tanks[1].name = "Left Wing";
    tanks[2].contents = 1500.0; tanks[2].name = "Right Wing";

    // Jettison from center tank only
    double jettisonAmount = 1000.0;
    tanks[0].contents -= jettisonAmount;

    TS_ASSERT_DELTA(tanks[0].contents, 1000.0, epsilon);
    TS_ASSERT_DELTA(tanks[1].contents, 1500.0, epsilon);
    TS_ASSERT_DELTA(tanks[2].contents, 1500.0, epsilon);
  }

  /***************************************************************************
   * Integrated System Tests
   ***************************************************************************/

  // Test complete fuel system cycle
  void testCompleteFuelSystemCycle() {
    // Initialize system
    std::vector<Tank> tanks(2);
    tanks[0].contents = 1000.0; tanks[0].x = 15.0;
    tanks[1].contents = 1000.0; tanks[1].x = 25.0;

    Engine engine;
    engine.fuelFlowRate = 1.0;
    engine.tankIndex = 0;

    // Initial CG
    double totalMass = tanks[0].contents + tanks[1].contents;
    double cgInitial = (tanks[0].contents * tanks[0].x + tanks[1].contents * tanks[1].x) / totalMass;
    TS_ASSERT_DELTA(cgInitial, 20.0, epsilon);

    // Consume fuel for 100 seconds
    double dt = 100.0;
    double consumed = engine.fuelFlowRate * dt;
    tanks[engine.tankIndex].contents -= consumed;

    // Final CG
    totalMass = tanks[0].contents + tanks[1].contents;
    double cgFinal = (tanks[0].contents * tanks[0].x + tanks[1].contents * tanks[1].x) / totalMass;

    TS_ASSERT(cgFinal > cgInitial);  // CG moved aft
    TS_ASSERT_DELTA(tanks[0].contents, 900.0, epsilon);
  }

  // Test multi-engine multi-tank system
  void testMultiEngineMultiTank() {
    std::vector<Tank> tanks(4);
    tanks[0].contents = 500.0;  // Left wing
    tanks[1].contents = 500.0;  // Right wing
    tanks[2].contents = 800.0;  // Center 1
    tanks[3].contents = 800.0;  // Center 2

    std::vector<Engine> engines(2);
    engines[0].fuelFlowRate = 1.2;
    engines[0].tankIndex = 0;
    engines[1].fuelFlowRate = 1.2;
    engines[1].tankIndex = 1;

    double dt = 60.0;

    for (size_t i = 0; i < engines.size(); i++) {
      double consumed = engines[i].fuelFlowRate * dt;
      tanks[engines[i].tankIndex].contents -= consumed;
    }

    TS_ASSERT_DELTA(tanks[0].contents, 428.0, epsilon);
    TS_ASSERT_DELTA(tanks[1].contents, 428.0, epsilon);
    TS_ASSERT_DELTA(tanks[2].contents, 800.0, epsilon);
    TS_ASSERT_DELTA(tanks[3].contents, 800.0, epsilon);
  }

  // Test fuel starvation scenario
  void testFuelStarvationScenario() {
    Tank tank;
    tank.contents = 20.0;
    tank.unusable = 10.0;

    Engine engine;
    engine.fuelFlowRate = 0.5;
    engine.running = true;

    double dt = 30.0;  // seconds
    double requested = engine.fuelFlowRate * dt;  // 15 lbs
    double available = std::max(0.0, tank.contents - tank.unusable);  // 10 lbs

    if (requested > available) {
      engine.starved = true;
      engine.running = false;
    }

    TS_ASSERT(engine.starved);
    TS_ASSERT(!engine.running);
  }

  // Test priority-based fuel sequencing
  void testPriorityFuelSequencing() {
    std::vector<Tank> tanks(3);
    tanks[0].contents = 500.0; tanks[0].priority = 1;
    tanks[1].contents = 800.0; tanks[1].priority = 2;
    tanks[2].contents = 600.0; tanks[2].priority = 3;

    // Find highest priority tank with fuel
    int selectedTank = -1;
    int highestPriority = 999;
    for (size_t i = 0; i < tanks.size(); i++) {
      if (tanks[i].contents > tanks[i].unusable && tanks[i].priority < highestPriority) {
        highestPriority = tanks[i].priority;
        selectedTank = static_cast<int>(i);
      }
    }

    TS_ASSERT_EQUALS(selectedTank, 0);  // Tank with priority 1
  }

  /***************************************************************************
   * Edge Cases and Boundary Conditions
   ***************************************************************************/

  // Test zero fuel condition
  void testZeroFuelCondition() {
    Tank tank;
    tank.contents = 0.0;

    Engine engine;
    engine.fuelFlowRate = 1.0;
    double dt = 10.0;

    double available = tank.contents;
    double consumed = std::min(engine.fuelFlowRate * dt, available);

    TS_ASSERT_DELTA(consumed, 0.0, epsilon);
  }

  // Test maximum fuel condition
  void testMaximumFuelCondition() {
    Tank tank;
    tank.contents = 10000.0;
    tank.capacity = 10000.0;

    bool full = (tank.contents >= tank.capacity);
    TS_ASSERT(full);
  }

  // Test very small time step
  void testVerySmallTimeStep() {
    Engine engine;
    engine.fuelFlowRate = 1.5;
    double dt = 0.001;  // 1 millisecond

    double consumed = engine.fuelFlowRate * dt;
    TS_ASSERT_DELTA(consumed, 0.0015, epsilon);
  }

  // Test numerical stability over long integration
  void testNumericalStability() {
    Tank tank;
    tank.contents = 1000.0;

    Engine engine;
    engine.fuelFlowRate = 0.1;

    // Integrate for 1000 steps
    double dt = 1.0;
    double totalConsumed = 0.0;
    for (int i = 0; i < 1000; i++) {
      double consumed = engine.fuelFlowRate * dt;
      tank.contents -= consumed;
      totalConsumed += consumed;
    }

    TS_ASSERT_DELTA(totalConsumed, 100.0, 0.001);
    TS_ASSERT_DELTA(tank.contents, 900.0, 0.001);
  }
};
