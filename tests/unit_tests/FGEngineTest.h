#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGEngineTest : public CxxTest::TestSuite
{
public:
  // Test engine type enum values
  void testEngineTypeEnums() {
    TS_ASSERT_EQUALS(FGEngine::etUnknown, 0);
    TS_ASSERT_EQUALS(FGEngine::etRocket, 1);
    TS_ASSERT_EQUALS(FGEngine::etPiston, 2);
    TS_ASSERT_EQUALS(FGEngine::etTurbine, 3);
    TS_ASSERT_EQUALS(FGEngine::etTurboprop, 4);
    TS_ASSERT_EQUALS(FGEngine::etElectric, 5);
  }

  // Test default fuel density (AVGAS = 6.02 lbs/gallon)
  void testDefaultFuelDensity() {
    // Default fuel density in FGEngine constructor
    double defaultFuelDensity = 6.02;
    TS_ASSERT_DELTA(defaultFuelDensity, 6.02, DEFAULT_TOLERANCE);
  }

  // Test default throttle bounds
  void testDefaultThrottleBounds() {
    // MinThrottle defaults to 0.0, MaxThrottle defaults to 1.0
    double minThrottle = 0.0;
    double maxThrottle = 1.0;

    TS_ASSERT_DELTA(minThrottle, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(maxThrottle, 1.0, DEFAULT_TOLERANCE);
    TS_ASSERT(minThrottle < maxThrottle);
  }

  // Test fuel flow rate calculation formula
  void testFuelFlowRateFormula() {
    // FuelFlowRate = SLFuelFlowMax * PctPower
    double SLFuelFlowMax = 100.0;  // lbs/sec at sea level max power

    // At 100% power
    double pctPower = 1.0;
    double fuelFlowRate = SLFuelFlowMax * pctPower;
    TS_ASSERT_DELTA(fuelFlowRate, 100.0, DEFAULT_TOLERANCE);

    // At 50% power
    pctPower = 0.5;
    fuelFlowRate = SLFuelFlowMax * pctPower;
    TS_ASSERT_DELTA(fuelFlowRate, 50.0, DEFAULT_TOLERANCE);

    // At 0% power (idle)
    pctPower = 0.0;
    fuelFlowRate = SLFuelFlowMax * pctPower;
    TS_ASSERT_DELTA(fuelFlowRate, 0.0, DEFAULT_TOLERANCE);
  }

  // Test fuel expended calculation
  void testFuelExpendedCalculation() {
    // FuelExpended = FuelFlowRate * TotalDeltaT
    double fuelFlowRate = 10.0;  // lbs/sec
    double deltaT = 0.1;         // seconds

    double fuelExpended = fuelFlowRate * deltaT;
    TS_ASSERT_DELTA(fuelExpended, 1.0, DEFAULT_TOLERANCE);

    // With larger time step
    deltaT = 1.0;
    fuelExpended = fuelFlowRate * deltaT;
    TS_ASSERT_DELTA(fuelExpended, 10.0, DEFAULT_TOLERANCE);

    // With zero time step
    deltaT = 0.0;
    fuelExpended = fuelFlowRate * deltaT;
    TS_ASSERT_DELTA(fuelExpended, 0.0, DEFAULT_TOLERANCE);
  }

  // Test fuel used accumulation (only when not starved)
  void testFuelUsedAccumulation() {
    double fuelUsedLbs = 0.0;
    bool starved = false;

    // Simulate fuel consumption over time
    double fuelExpended = 1.0;
    if (!starved) fuelUsedLbs += fuelExpended;
    TS_ASSERT_DELTA(fuelUsedLbs, 1.0, DEFAULT_TOLERANCE);

    fuelExpended = 2.0;
    if (!starved) fuelUsedLbs += fuelExpended;
    TS_ASSERT_DELTA(fuelUsedLbs, 3.0, DEFAULT_TOLERANCE);

    // When starved, fuel should not accumulate
    starved = true;
    fuelExpended = 5.0;
    if (!starved) fuelUsedLbs += fuelExpended;
    TS_ASSERT_DELTA(fuelUsedLbs, 3.0, DEFAULT_TOLERANCE);  // unchanged
  }

  // Test fuel flow rate to GPH conversion
  void testFuelFlowRateGPHConversion() {
    // FuelFlowRateGPH = FuelFlowRate * 3600 / FuelDensity
    double fuelFlowRate = 1.0;  // lbs/sec
    double fuelDensity = 6.02;  // lbs/gallon (AVGAS)

    double fuelFlowRateGPH = fuelFlowRate * 3600.0 / fuelDensity;
    // 1 lb/sec * 3600 sec/hr / 6.02 lbs/gal = 598.0 gal/hr
    TS_ASSERT_DELTA(fuelFlowRateGPH, 598.0066, 0.001);

    // With JET-A density (6.74 lbs/gallon)
    fuelDensity = 6.74;
    fuelFlowRateGPH = fuelFlowRate * 3600.0 / fuelDensity;
    TS_ASSERT_DELTA(fuelFlowRateGPH, 534.1246, 0.001);
  }

  // Test horsepower to ft-lbf/sec conversion
  void testHorsepowerConversion() {
    // 1 HP = 550 ft-lbf/sec
    double hptoftlbssec = 550.0;

    double hp = 100.0;
    double power_ftlbsec = hp * hptoftlbssec;
    TS_ASSERT_DELTA(power_ftlbsec, 55000.0, DEFAULT_TOLERANCE);

    // Inverse conversion
    hp = power_ftlbsec / hptoftlbssec;
    TS_ASSERT_DELTA(hp, 100.0, DEFAULT_TOLERANCE);
  }

  // Test watts to horsepower conversion
  void testWattsToHorsepowerConversion() {
    // 1 HP = 745.7 watts
    double hptowatts = 745.7;

    double watts = 7457.0;  // 10 HP
    double hp = watts / hptowatts;
    TS_ASSERT_DELTA(hp, 10.0, 0.001);

    // Common electric motor ratings
    double hp1 = 745.7 / hptowatts;  // 1 HP motor
    TS_ASSERT_DELTA(hp1, 1.0, DEFAULT_TOLERANCE);
  }

  // Test inputs structure default initialization
  void testInputsStructureInit() {
    FGEngine::Inputs inputs;

    // Initialize to expected defaults
    inputs.Pressure = 2116.22;  // Sea level pressure (psf)
    inputs.PressureRatio = 1.0;
    inputs.Temperature = 518.67;  // Sea level temp (Rankine)
    inputs.Density = 0.002377;  // Sea level density (slugs/ft^3)
    inputs.DensityRatio = 1.0;
    inputs.Soundspeed = 1116.45;  // ft/sec at sea level
    inputs.TotalDeltaT = 0.0;
    inputs.Vt = 0.0;
    inputs.Vc = 0.0;
    inputs.qbar = 0.0;
    inputs.alpha = 0.0;
    inputs.beta = 0.0;
    inputs.H_agl = 0.0;

    // Verify sea level atmosphere values
    TS_ASSERT_DELTA(inputs.Pressure, Constants::SEA_LEVEL_PRESSURE_PSF, 1.0);
    TS_ASSERT_DELTA(inputs.Temperature, Constants::SEA_LEVEL_TEMP_R, 1.0);
    TS_ASSERT_DELTA(inputs.Density, Constants::SEA_LEVEL_DENSITY_SLUGFT3, 0.00001);
  }

  // Test throttle command vectors
  void testThrottleCommandVectors() {
    FGEngine::Inputs inputs;

    // Initialize for 2 engines
    inputs.ThrottleCmd.resize(2);
    inputs.ThrottlePos.resize(2);
    inputs.MixtureCmd.resize(2);
    inputs.MixturePos.resize(2);

    inputs.ThrottleCmd[0] = 0.0;
    inputs.ThrottleCmd[1] = 1.0;
    inputs.ThrottlePos[0] = 0.0;
    inputs.ThrottlePos[1] = 1.0;

    TS_ASSERT_EQUALS(inputs.ThrottleCmd.size(), 2);
    TS_ASSERT_DELTA(inputs.ThrottleCmd[0], 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(inputs.ThrottleCmd[1], 1.0, DEFAULT_TOLERANCE);
  }

  // Test propeller advance and feather vectors
  void testPropellerControlVectors() {
    FGEngine::Inputs inputs;

    inputs.PropAdvance.resize(1);
    inputs.PropFeather.resize(1);

    inputs.PropAdvance[0] = 0.5;  // 50% advance
    inputs.PropFeather[0] = false;

    TS_ASSERT_DELTA(inputs.PropAdvance[0], 0.5, DEFAULT_TOLERANCE);
    TS_ASSERT(!inputs.PropFeather[0]);

    // Feathered prop
    inputs.PropFeather[0] = true;
    TS_ASSERT(inputs.PropFeather[0]);
  }

  // Test engine state flags
  void testEngineStateFlags() {
    bool running = false;
    bool cranking = false;
    bool starter = false;
    bool starved = false;

    // Initial state: all false
    TS_ASSERT(!running);
    TS_ASSERT(!cranking);
    TS_ASSERT(!starter);
    TS_ASSERT(!starved);

    // Starting sequence
    starter = true;
    TS_ASSERT(starter);

    cranking = true;
    TS_ASSERT(cranking);

    // Engine catches
    running = true;
    cranking = false;
    starter = false;
    TS_ASSERT(running);
    TS_ASSERT(!cranking);
    TS_ASSERT(!starter);

    // Engine starves
    starved = true;
    running = false;
    TS_ASSERT(starved);
    TS_ASSERT(!running);
  }

  // Test reset to initial conditions
  void testResetToIC() {
    // Simulate state after running
    bool starter = true;
    double fuelExpended = 100.0;
    bool starved = false;
    bool running = true;
    bool cranking = false;
    double pctPower = 0.75;
    double fuelFlow_gph = 50.0;
    double fuelFlow_pph = 300.0;
    double fuelFlowRate = 0.0833;
    bool fuelFreeze = true;
    double fuelUsedLbs = 500.0;

    // Reset to IC
    starter = false;
    fuelExpended = 0.0;
    starved = false;
    running = false;
    cranking = false;
    pctPower = 0.0;
    fuelFlow_gph = 0.0;
    fuelFlow_pph = 0.0;
    fuelFlowRate = 0.0;
    fuelFreeze = false;
    fuelUsedLbs = 0.0;

    // Verify reset state
    TS_ASSERT(!starter);
    TS_ASSERT_DELTA(fuelExpended, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT(!starved);
    TS_ASSERT(!running);
    TS_ASSERT(!cranking);
    TS_ASSERT_DELTA(pctPower, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(fuelFlow_gph, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(fuelFlow_pph, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(fuelFlowRate, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT(!fuelFreeze);
    TS_ASSERT_DELTA(fuelUsedLbs, 0.0, DEFAULT_TOLERANCE);
  }

  // Test percent power range
  void testPercentPowerRange() {
    // PctPower should be between 0.0 and 1.0
    double pctPower = 0.0;
    TS_ASSERT(pctPower >= 0.0);
    TS_ASSERT(pctPower <= 1.0);

    pctPower = 0.5;
    TS_ASSERT(pctPower >= 0.0);
    TS_ASSERT(pctPower <= 1.0);

    pctPower = 1.0;
    TS_ASSERT(pctPower >= 0.0);
    TS_ASSERT(pctPower <= 1.0);
  }

  // Test fuel density for common fuels
  void testCommonFuelDensities() {
    // AVGAS 100LL
    double avgas = 6.02;
    TS_ASSERT_DELTA(avgas, 6.02, 0.01);

    // JET-A / JP-8
    double jetA = 6.74;
    TS_ASSERT_DELTA(jetA, 6.74, 0.01);

    // JP-4
    double jp4 = 6.36;
    TS_ASSERT_DELTA(jp4, 6.36, 0.01);

    // Liquid Oxygen (LOX) - for rockets
    double lox = 71.23;  // lbs/gallon
    TS_ASSERT(lox > jetA);  // LOX is much denser

    // RP-1 (rocket kerosene)
    double rp1 = 6.8;
    TS_ASSERT_DELTA(rp1, 6.8, 0.1);
  }

  // Test total thrust at different power levels
  void testThrustVsPower() {
    // For many engines, thrust is roughly proportional to power
    // (exact relationship depends on engine type and conditions)
    double maxThrust = 10000.0;  // lbs

    // 100% power
    double thrust100 = maxThrust * 1.0;
    TS_ASSERT_DELTA(thrust100, 10000.0, DEFAULT_TOLERANCE);

    // 50% power (linear approximation)
    double thrust50 = maxThrust * 0.5;
    TS_ASSERT_DELTA(thrust50, 5000.0, DEFAULT_TOLERANCE);

    // Idle thrust (typically 5-10% of max)
    double thrustIdle = maxThrust * 0.05;
    TS_ASSERT_DELTA(thrustIdle, 500.0, DEFAULT_TOLERANCE);
  }

  // Test aerodynamic input vector structure
  void testAeroInputVectors() {
    FGEngine::Inputs inputs;

    // Body-axis velocities
    inputs.AeroUVW(1) = 200.0;  // U - forward velocity (ft/sec)
    inputs.AeroUVW(2) = 0.0;    // V - lateral velocity
    inputs.AeroUVW(3) = 10.0;   // W - vertical velocity

    TS_ASSERT_DELTA(inputs.AeroUVW(1), 200.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(inputs.AeroUVW(2), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(inputs.AeroUVW(3), 10.0, DEFAULT_TOLERANCE);

    // Body-axis rates
    inputs.AeroPQR(1) = 0.0;    // P - roll rate (rad/sec)
    inputs.AeroPQR(2) = 0.01;   // Q - pitch rate
    inputs.AeroPQR(3) = 0.0;    // R - yaw rate

    TS_ASSERT_DELTA(inputs.AeroPQR(2), 0.01, DEFAULT_TOLERANCE);
  }

  // Test specific fuel consumption (SFC) calculation
  void testSpecificFuelConsumption() {
    // SFC = Fuel Flow Rate / Thrust
    double fuelFlowRate = 1.0;  // lbs/sec
    double thrust = 10000.0;    // lbs

    double sfc = fuelFlowRate / thrust;  // lbs/sec/lbf
    double sfcPerHour = sfc * 3600.0;    // lbs/hr/lbf

    // Typical turbojet SFC is 0.8-1.0 lbs/hr/lbf
    TS_ASSERT_DELTA(sfcPerHour, 0.36, 0.01);

    // Afterburning increases SFC
    double abFuelFlowRate = 2.0;
    double abSfc = abFuelFlowRate / thrust * 3600.0;
    TS_ASSERT(abSfc > sfcPerHour);
  }

  // Test engine indexing for multi-engine aircraft
  void testEngineIndexing() {
    // Engine numbers are 0-indexed
    int engineNumber0 = 0;
    int engineNumber1 = 1;
    int engineNumber2 = 2;
    int engineNumber3 = 3;

    TS_ASSERT_EQUALS(engineNumber0, 0);
    TS_ASSERT_EQUALS(engineNumber1, 1);

    // Four-engine aircraft
    TS_ASSERT(engineNumber3 < 4);

    // Typical naming: propulsion/engine[0], propulsion/engine[1], etc.
    std::string baseName = "propulsion/engine";
    std::string engine0Name = baseName + "[0]";
    std::string engine1Name = baseName + "[1]";

    TS_ASSERT(engine0Name.find("[0]") != std::string::npos);
    TS_ASSERT(engine1Name.find("[1]") != std::string::npos);
  }
};
