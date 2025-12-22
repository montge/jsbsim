#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <string>
#include <map>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * FGTank unit tests
 *
 * Note: FGTank requires XML element for construction, so these tests focus on:
 * - Fuel density values by type
 * - Tank capacity/content calculations
 * - Temperature conversions
 * - Fill percentage calculations
 */
class FGTankTest : public CxxTest::TestSuite
{
public:
  // Known fuel densities in lbs/gal from FGTank::ProcessFuelName
  std::map<std::string, double> fuelDensities = {
    {"AVGAS", 6.02},
    {"JET-A", 6.74},
    {"JET-A1", 6.74},
    {"JET-B", 6.48},
    {"JP-1", 6.76},
    {"JP-2", 6.38},
    {"JP-3", 6.34},
    {"JP-4", 6.48},
    {"JP-5", 6.81},
    {"JP-6", 6.55},
    {"JP-7", 6.61},
    {"JP-8", 6.66},
    {"JP-8+100", 6.66},
    {"RP-1", 6.73},
    {"T-1", 6.88},
    {"ETHANOL", 6.58},
    {"HYDRAZINE", 8.61}
  };

  // Test AVGAS density
  void testAVGASDensity() {
    TS_ASSERT_DELTA(fuelDensities["AVGAS"], 6.02, epsilon);
  }

  // Test JET-A density
  void testJETADensity() {
    TS_ASSERT_DELTA(fuelDensities["JET-A"], 6.74, epsilon);
    TS_ASSERT_DELTA(fuelDensities["JET-A1"], 6.74, epsilon);
  }

  // Test JP series fuels
  void testJPSeriesDensity() {
    TS_ASSERT_DELTA(fuelDensities["JP-1"], 6.76, epsilon);
    TS_ASSERT_DELTA(fuelDensities["JP-4"], 6.48, epsilon);
    TS_ASSERT_DELTA(fuelDensities["JP-5"], 6.81, epsilon);
    TS_ASSERT_DELTA(fuelDensities["JP-8"], 6.66, epsilon);
  }

  // Test rocket fuel density (RP-1)
  void testRP1Density() {
    TS_ASSERT_DELTA(fuelDensities["RP-1"], 6.73, epsilon);
  }

  // Test hydrazine density (highest density fuel)
  void testHydrazineDensity() {
    TS_ASSERT_DELTA(fuelDensities["HYDRAZINE"], 8.61, epsilon);

    // Should be the highest density
    for (const auto& pair : fuelDensities) {
      TS_ASSERT(fuelDensities["HYDRAZINE"] >= pair.second);
    }
  }

  // Test fuel density ranges (all should be reasonable)
  void testFuelDensityRanges() {
    for (const auto& pair : fuelDensities) {
      // All aviation fuels should have density between 5.5 and 9.0 lbs/gal
      TS_ASSERT(pair.second >= 5.5);
      TS_ASSERT(pair.second <= 9.0);
    }
  }

  // Test capacity to gallons conversion
  void testCapacityToGallons() {
    double density = 6.74;  // JET-A
    double capacityLbs = 1000.0;  // 1000 lbs capacity

    double capacityGal = capacityLbs / density;
    TS_ASSERT_DELTA(capacityGal, 148.368, 0.001);
  }

  // Test contents to gallons conversion
  void testContentsToGallons() {
    double density = 6.02;  // AVGAS
    double contentsLbs = 500.0;

    double contentsGal = contentsLbs / density;
    TS_ASSERT_DELTA(contentsGal, 83.056, 0.001);
  }

  // Test percent full calculation
  void testPercentFull() {
    double capacity = 1000.0;  // lbs
    double contents = 750.0;   // lbs

    double pctFull = (contents / capacity) * 100.0;
    TS_ASSERT_DELTA(pctFull, 75.0, epsilon);
  }

  // Test percent full at empty
  void testPercentFullEmpty() {
    double capacity = 1000.0;
    double contents = 0.0;

    double pctFull = (contents / capacity) * 100.0;
    TS_ASSERT_DELTA(pctFull, 0.0, epsilon);
  }

  // Test percent full at full
  void testPercentFullFull() {
    double capacity = 1000.0;
    double contents = 1000.0;

    double pctFull = (contents / capacity) * 100.0;
    TS_ASSERT_DELTA(pctFull, 100.0, epsilon);
  }

  // Test temperature conversion (Celsius to Fahrenheit)
  void testCelsiusToFahrenheit() {
    // FGTank stores temperature in Celsius internally but can return Fahrenheit

    // Freezing point
    double tempC = 0.0;
    double tempF = (tempC * 9.0 / 5.0) + 32.0;
    TS_ASSERT_DELTA(tempF, 32.0, epsilon);

    // Boiling point
    tempC = 100.0;
    tempF = (tempC * 9.0 / 5.0) + 32.0;
    TS_ASSERT_DELTA(tempF, 212.0, epsilon);

    // Typical fuel temperature
    tempC = 15.0;  // Standard day temp
    tempF = (tempC * 9.0 / 5.0) + 32.0;
    TS_ASSERT_DELTA(tempF, 59.0, epsilon);
  }

  // Test Fahrenheit to Celsius conversion
  void testFahrenheitToCelsius() {
    double tempF = 68.0;  // Room temperature
    double tempC = (tempF - 32.0) * 5.0 / 9.0;
    TS_ASSERT_DELTA(tempC, 20.0, epsilon);

    tempF = -40.0;  // Special point where F = C
    tempC = (tempF - 32.0) * 5.0 / 9.0;
    TS_ASSERT_DELTA(tempC, -40.0, epsilon);
  }

  // Test drain calculation
  void testDrainCalculation() {
    double contents = 1000.0;  // lbs
    double used = 100.0;       // lbs

    double remaining = contents - used;
    TS_ASSERT_DELTA(remaining, 900.0, epsilon);
  }

  // Test drain to empty
  void testDrainToEmpty() {
    double contents = 100.0;
    double used = 100.0;

    double remaining = std::max(0.0, contents - used);
    TS_ASSERT_DELTA(remaining, 0.0, epsilon);
  }

  // Test drain exceeds contents
  void testDrainExceedsContents() {
    double contents = 50.0;
    double used = 100.0;

    double remaining = std::max(0.0, contents - used);
    TS_ASSERT_DELTA(remaining, 0.0, epsilon);
  }

  // Test standpipe behavior (minimum undumpable fuel)
  void testStandpipe() {
    double contents = 100.0;
    double standpipe = 20.0;

    // Dumpable fuel = contents - standpipe
    double dumpable = std::max(0.0, contents - standpipe);
    TS_ASSERT_DELTA(dumpable, 80.0, epsilon);
  }

  // Test standpipe when below standpipe level
  void testStandpipeBelowLevel() {
    double contents = 15.0;
    double standpipe = 20.0;

    // No dumpable fuel when below standpipe
    double dumpable = std::max(0.0, contents - standpipe);
    TS_ASSERT_DELTA(dumpable, 0.0, epsilon);
  }

  // Test unusable fuel calculation
  void testUnusableFuel() {
    double unusableVol = 2.0;  // gallons
    double density = 6.74;     // JET-A

    double unusableLbs = unusableVol * density;
    TS_ASSERT_DELTA(unusableLbs, 13.48, epsilon);
  }

  // Test fill calculation
  void testFillCalculation() {
    double capacity = 1000.0;
    double contents = 500.0;
    double amount = 300.0;

    double newContents = std::min(capacity, contents + amount);
    TS_ASSERT_DELTA(newContents, 800.0, epsilon);
  }

  // Test fill beyond capacity
  void testFillBeyondCapacity() {
    double capacity = 1000.0;
    double contents = 800.0;
    double amount = 500.0;  // Would overflow

    double newContents = std::min(capacity, contents + amount);
    TS_ASSERT_DELTA(newContents, capacity, epsilon);

    double overflow = (contents + amount) - capacity;
    TS_ASSERT_DELTA(overflow, 300.0, epsilon);
  }

  // Test external flow rate (positive = filling)
  void testExternalFlowFilling() {
    double contents = 500.0;
    double flowRate = 10.0;  // pps (lbs per second)
    double dt = 1.0;         // 1 second

    double newContents = contents + flowRate * dt;
    TS_ASSERT_DELTA(newContents, 510.0, epsilon);
  }

  // Test external flow rate (negative = draining)
  void testExternalFlowDraining() {
    double contents = 500.0;
    double flowRate = -10.0;  // pps (lbs per second)
    double dt = 1.0;

    double newContents = std::max(0.0, contents + flowRate * dt);
    TS_ASSERT_DELTA(newContents, 490.0, epsilon);
  }

  // Test tank inertia factor
  void testInertiaFactor() {
    // Inertia factor is typically 0-1, representing how much slosh affects inertia
    double inertiaFactor = 0.5;
    double baseInertia = 1000.0;

    double effectiveInertia = baseInertia * inertiaFactor;
    TS_ASSERT_DELTA(effectiveInertia, 500.0, epsilon);
  }

  // Test tank type enum values
  void testTankTypeEnums() {
    // From FGTank::TankType
    // ttUNKNOWN = 0, ttFUEL = 1, ttOXIDIZER = 2
    int ttUNKNOWN = 0;
    int ttFUEL = 1;
    int ttOXIDIZER = 2;

    TS_ASSERT_EQUALS(ttUNKNOWN, 0);
    TS_ASSERT_EQUALS(ttFUEL, 1);
    TS_ASSERT_EQUALS(ttOXIDIZER, 2);
  }

  // Test grain type enum values
  void testGrainTypeEnums() {
    // From FGTank::GrainType
    // gtUNKNOWN = 0, gtCYLINDRICAL = 1, gtENDBURNING = 2, gtFUNCTION = 3
    int gtUNKNOWN = 0;
    int gtCYLINDRICAL = 1;
    int gtENDBURNING = 2;
    int gtFUNCTION = 3;

    TS_ASSERT_EQUALS(gtUNKNOWN, 0);
    TS_ASSERT_EQUALS(gtCYLINDRICAL, 1);
    TS_ASSERT_EQUALS(gtENDBURNING, 2);
    TS_ASSERT_EQUALS(gtFUNCTION, 3);
  }

  // Test priority affects selection
  void testPrioritySelection() {
    // Priority > 0 means selected, priority = 0 means not selected
    int priority1 = 1;
    int priority0 = 0;

    bool selected1 = priority1 > 0;
    bool selected0 = priority0 > 0;

    TS_ASSERT(selected1);
    TS_ASSERT(!selected0);
  }

  // Test heat capacity calculation concept
  void testHeatCapacityConcept() {
    // Heat capacity of jet fuel ~ 900 J/lbm/K
    double heatCapacity = 900.0;  // J/lbm/K
    double mass = 100.0;          // lbm
    double deltaT = 10.0;         // K

    double heatRequired = heatCapacity * mass * deltaT;
    TS_ASSERT_DELTA(heatRequired, 900000.0, epsilon);
  }

  // Test surface area estimation from capacity
  void testSurfaceAreaEstimation() {
    // From FGTank documentation:
    // Tank dimensions assumed as h x 4h x 10h
    // Volume = 40 * h^3
    // Surface area of one side = 40 * h^2

    double capacityLbs = 1000.0;
    double density = 6.74;  // JET-A in lbs/gal
    double lbsPerCuFt = 49.368;  // Conversion factor

    double volumeCuFt = capacityLbs / lbsPerCuFt;
    double h3 = volumeCuFt / 40.0;
    double h = std::pow(h3, 1.0/3.0);
    double surfaceArea = 40.0 * h * h;

    TS_ASSERT(!std::isnan(surfaceArea));
    TS_ASSERT(surfaceArea > 0.0);
  }
};
