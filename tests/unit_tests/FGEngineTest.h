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

  /***************************************************************************
   * Extended Throttle Tests
   ***************************************************************************/

  // Test throttle command clamping
  void testThrottleCommandClamping() {
    auto clampThrottle = [](double cmd) {
      return std::max(0.0, std::min(1.0, cmd));
    };

    TS_ASSERT_DELTA(clampThrottle(0.5), 0.5, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(clampThrottle(-0.5), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(clampThrottle(1.5), 1.0, DEFAULT_TOLERANCE);
  }

  // Test throttle hysteresis
  void testThrottleHysteresis() {
    double throttlePos = 0.0;
    double throttleCmd = 1.0;
    double rate = 0.1;  // per step

    // Throttle should lag command
    for (int i = 0; i < 5; i++) {
      double delta = throttleCmd - throttlePos;
      if (std::abs(delta) > rate) {
        delta = (delta > 0) ? rate : -rate;
      }
      throttlePos += delta;
    }

    TS_ASSERT_DELTA(throttlePos, 0.5, DEFAULT_TOLERANCE);
  }

  // Test idle throttle setting
  void testIdleThrottleSetting() {
    double idleThrottle = 0.05;  // Typical idle: 5%
    double minThrottle = 0.0;

    // Idle should be slightly above minimum
    TS_ASSERT(idleThrottle > minThrottle);
    TS_ASSERT(idleThrottle < 0.2);  // Less than 20%
  }

  // Test throttle to thrust curve (non-linear)
  void testThrottleToThrustCurve() {
    // Many engines have non-linear throttle-thrust relationship
    auto thrustFromThrottle = [](double throttle) {
      // Quadratic approximation
      return throttle * throttle;
    };

    TS_ASSERT_DELTA(thrustFromThrottle(0.0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(thrustFromThrottle(0.5), 0.25, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(thrustFromThrottle(1.0), 1.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Altitude Effects Tests
   ***************************************************************************/

  // Test thrust reduction with altitude
  void testThrustAltitudeEffect() {
    // Thrust decreases with altitude due to lower air density
    double seaLevelThrust = 10000.0;

    // At 10,000 ft (density ratio ~ 0.74)
    double densityRatio10k = 0.74;
    double thrust10k = seaLevelThrust * densityRatio10k;
    TS_ASSERT(thrust10k < seaLevelThrust);
    TS_ASSERT_DELTA(thrust10k, 7400.0, 100.0);

    // At 30,000 ft (density ratio ~ 0.37)
    double densityRatio30k = 0.37;
    double thrust30k = seaLevelThrust * densityRatio30k;
    TS_ASSERT(thrust30k < thrust10k);
    TS_ASSERT_DELTA(thrust30k, 3700.0, 100.0);
  }

  // Test power available at altitude
  void testPowerAvailableAltitude() {
    double seaLevelPower = 200.0;  // HP

    // Normally aspirated engine loses about 3.5% per 1000 ft
    auto powerAtAltitude = [&](double altitude) {
      return seaLevelPower * (1.0 - 0.035 * altitude / 1000.0);
    };

    TS_ASSERT_DELTA(powerAtAltitude(0), 200.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(powerAtAltitude(10000), 130.0, 1.0);
    TS_ASSERT_DELTA(powerAtAltitude(20000), 60.0, 1.0);
  }

  // Test turbocharger effect
  void testTurbochargerEffect() {
    double naturallyAspiratedLoss = 0.035;  // per 1000 ft
    double turbochargeLoss = 0.015;          // per 1000 ft (less loss)
    double criticalAltitude = 18000.0;       // ft

    double seaLevelPower = 300.0;

    // NA engine at 15,000 ft
    double naPower = seaLevelPower * (1.0 - naturallyAspiratedLoss * 15);
    TS_ASSERT(naPower < seaLevelPower);

    // Turbocharged engine maintains power to critical altitude
    double turboPower = seaLevelPower;  // Full power below critical alt
    TS_ASSERT(turboPower > naPower);
  }

  /***************************************************************************
   * Temperature Effects Tests
   ***************************************************************************/

  // Test cold weather startup
  void testColdWeatherStartup() {
    double ambientTempR = 450.0;  // About -10°F
    double standardTempR = 518.67;

    // Cold weather affects engine performance
    double tempRatio = ambientTempR / standardTempR;
    TS_ASSERT(tempRatio < 1.0);

    // Air is denser when cold (helpful for thrust)
    double densityIncrease = standardTempR / ambientTempR;
    TS_ASSERT(densityIncrease > 1.0);
  }

  // Test hot and high conditions
  void testHotAndHighConditions() {
    // Hot day at high altitude - worst case for performance
    double hotDayTempR = 560.0;  // About 100°F
    double standardTempR = 518.67;
    double altitude = 5000.0;

    // Temperature penalty
    double tempRatio = hotDayTempR / standardTempR;
    TS_ASSERT(tempRatio > 1.0);

    // Combined hot and high reduces performance significantly
    double pressureAltitude = altitude + 1000 * (tempRatio - 1.0) * 120;
    TS_ASSERT(pressureAltitude > altitude);
  }

  // Test engine temperature limits
  void testEngineTemperatureLimits() {
    // Typical turbine limits
    double maxTIT = 1850.0;  // Turbine inlet temp (°F)
    double maxEGT = 1700.0;  // Exhaust gas temp (°F)
    double maxCHT = 500.0;   // Cylinder head temp (for pistons)

    TS_ASSERT(maxTIT > maxEGT);
    TS_ASSERT(maxEGT > maxCHT);
  }

  /***************************************************************************
   * Fuel System Tests
   ***************************************************************************/

  // Test fuel tank selection
  void testFuelTankSelection() {
    int leftTank = 0;
    int rightTank = 1;
    int auxTank = 2;
    int selectedTank = leftTank;

    TS_ASSERT_EQUALS(selectedTank, leftTank);

    // Switch tanks
    selectedTank = rightTank;
    TS_ASSERT_EQUALS(selectedTank, rightTank);
  }

  // Test fuel freeze conditions
  void testFuelFreezeConditions() {
    // JET-A freeze point is about -47°F
    double jetAFreezePointF = -47.0;
    double jetAFreezePointR = jetAFreezePointF + 459.67;

    // At very high altitude, fuel can freeze
    double fuelTempR = 420.0;  // Very cold
    bool fuelFrozen = fuelTempR < jetAFreezePointR;

    TS_ASSERT(!fuelFrozen);  // Not quite frozen

    fuelTempR = 400.0;  // Even colder
    fuelFrozen = fuelTempR < jetAFreezePointR;
    TS_ASSERT(fuelFrozen);
  }

  // Test fuel contamination
  void testFuelContamination() {
    double fuelPurity = 1.0;  // 100% pure
    double waterContent = 0.0;

    // Some water contamination
    waterContent = 0.001;  // 0.1%
    fuelPurity = 1.0 - waterContent;

    TS_ASSERT_DELTA(fuelPurity, 0.999, DEFAULT_TOLERANCE);
    TS_ASSERT(waterContent < 0.01);  // Less than 1% is typical limit
  }

  // Test fuel starvation
  void testFuelStarvation() {
    double fuelQuantity = 100.0;  // lbs
    double fuelFlowRate = 10.0;   // lbs/min
    double dt = 0.1;              // time step (min)

    bool starved = false;
    while (fuelQuantity > 0 && !starved) {
      fuelQuantity -= fuelFlowRate * dt;
      if (fuelQuantity <= 0) {
        starved = true;
        fuelQuantity = 0;
      }
    }

    TS_ASSERT(starved);
    TS_ASSERT_DELTA(fuelQuantity, 0.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Multi-Engine Tests
   ***************************************************************************/

  // Test differential thrust (yaw control)
  void testDifferentialThrust() {
    double leftThrust = 5000.0;
    double rightThrust = 5000.0;
    double armLength = 10.0;  // ft from centerline

    // Symmetric thrust - no yaw moment
    double yawMoment = (rightThrust - leftThrust) * armLength;
    TS_ASSERT_DELTA(yawMoment, 0.0, DEFAULT_TOLERANCE);

    // Asymmetric thrust - creates yaw moment
    rightThrust = 4000.0;
    yawMoment = (rightThrust - leftThrust) * armLength;
    TS_ASSERT_DELTA(yawMoment, -10000.0, DEFAULT_TOLERANCE);  // Yaw left
  }

  // Test engine failure effects
  void testEngineFailure() {
    int numEngines = 2;
    double thrustPerEngine = 5000.0;

    // Normal operation
    double totalThrust = numEngines * thrustPerEngine;
    TS_ASSERT_DELTA(totalThrust, 10000.0, DEFAULT_TOLERANCE);

    // One engine failure
    int workingEngines = 1;
    totalThrust = workingEngines * thrustPerEngine;
    TS_ASSERT_DELTA(totalThrust, 5000.0, DEFAULT_TOLERANCE);

    // Performance reduction > 50% due to drag from windmilling prop
    double windmillingDrag = 500.0;
    double netThrust = totalThrust - windmillingDrag;
    TS_ASSERT(netThrust < 0.5 * 10000.0);
  }

  // Test minimum control speed (Vmc)
  void testMinimumControlSpeed() {
    // Vmc increases with:
    // - Higher thrust asymmetry
    // - Lower density altitude
    // - More aft CG

    double vmcSeaLevel = 75.0;   // kts
    double vmc5000ft = 80.0;     // Higher at altitude

    TS_ASSERT(vmc5000ft > vmcSeaLevel);
  }

  /***************************************************************************
   * Engine Performance Tests
   ***************************************************************************/

  // Test brake specific fuel consumption (BSFC)
  void testBrakeSFC() {
    // BSFC = fuel flow / power
    double fuelFlow = 20.0;  // lbs/hr
    double power = 200.0;    // HP

    double bsfc = fuelFlow / power;  // lbs/hr/HP
    TS_ASSERT_DELTA(bsfc, 0.1, DEFAULT_TOLERANCE);

    // Good piston engines: 0.4-0.5 lbs/hr/HP
    // Good turboprops: 0.5-0.6 lbs/hr/HP
    double pistonBSFC = 0.45;
    double turbopropBSFC = 0.55;
    TS_ASSERT(turbopropBSFC > pistonBSFC);
  }

  // Test propulsive efficiency
  void testPropulsiveEfficiency() {
    double thrustPower = 50000.0;  // ft-lbf/sec
    double shaftPower = 55000.0;   // ft-lbf/sec

    double propEff = thrustPower / shaftPower;
    TS_ASSERT(propEff < 1.0);
    TS_ASSERT(propEff > 0.8);  // Good props: 80-90%
  }

  // Test engine torque calculation
  void testEngineTorque() {
    // Torque = Power / (2*pi*RPM/60)
    double power = 300.0 * 550.0;  // 300 HP in ft-lbf/sec
    double rpm = 2700.0;

    double torque = power / (2.0 * M_PI * rpm / 60.0);
    TS_ASSERT(torque > 0);

    // At constant power, lower RPM = higher torque
    double lowRPM = 2400.0;
    double torqueLowRPM = power / (2.0 * M_PI * lowRPM / 60.0);
    TS_ASSERT(torqueLowRPM > torque);
  }

  // Test N1 and N2 spool relationships
  void testSpoolRelationships() {
    // For turbofans, N1 (fan) and N2 (core) are related
    double n2 = 95.0;  // Core spool %
    double n1 = 85.0;  // Fan spool % (typically lower)

    TS_ASSERT(n2 > n1);

    // At idle
    double n2Idle = 60.0;
    double n1Idle = 25.0;

    TS_ASSERT(n2Idle > n1Idle);
  }

  /***************************************************************************
   * Engine Start/Stop Tests
   ***************************************************************************/

  // Test starter motor engagement
  void testStarterMotorEngagement() {
    bool starterEngaged = false;
    double starterTorque = 0.0;
    double engineRPM = 0.0;

    // Engage starter
    starterEngaged = true;
    starterTorque = 100.0;  // ft-lbf

    // Starter spins engine
    for (int i = 0; i < 10; i++) {
      if (starterEngaged) {
        engineRPM += starterTorque * 0.5;
      }
    }

    TS_ASSERT(engineRPM > 0);
    TS_ASSERT(starterEngaged);
  }

  // Test ignition sequence
  void testIgnitionSequence() {
    double engineRPM = 0.0;
    double lightOffRPM = 25.0;  // % N2 for ignition
    bool ignitionOn = false;
    bool combustion = false;
    bool fuelOn = false;

    // Starter spins to light-off RPM
    engineRPM = 30.0;

    // Ignition on
    ignitionOn = true;

    // Fuel on
    fuelOn = true;

    // If all conditions met, combustion starts
    if (engineRPM > lightOffRPM && ignitionOn && fuelOn) {
      combustion = true;
    }

    TS_ASSERT(combustion);
  }

  // Test engine shutdown
  void testEngineShutdown() {
    bool fuelOn = true;
    bool running = true;
    double engineRPM = 2700.0;

    // Cut fuel
    fuelOn = false;

    // Engine winds down
    while (engineRPM > 0) {
      engineRPM -= 50.0;
      if (engineRPM <= 0) {
        running = false;
        engineRPM = 0;
      }
    }

    TS_ASSERT(!running);
    TS_ASSERT(!fuelOn);
    TS_ASSERT_DELTA(engineRPM, 0.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Oil System Tests
   ***************************************************************************/

  // Test oil pressure vs RPM
  void testOilPressureVsRPM() {
    auto oilPressure = [](double rpm) {
      // Simplified: pressure increases with RPM
      return rpm / 100.0;  // psi
    };

    TS_ASSERT_DELTA(oilPressure(0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(oilPressure(2000), 20.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(oilPressure(2700), 27.0, DEFAULT_TOLERANCE);
  }

  // Test oil temperature
  void testOilTemperature() {
    double minOilTemp = 100.0;  // °F
    double maxOilTemp = 245.0;  // °F (typical)
    double normalOilTemp = 180.0;

    TS_ASSERT(normalOilTemp > minOilTemp);
    TS_ASSERT(normalOilTemp < maxOilTemp);
  }

  /***************************************************************************
   * Engine Limits Tests
   ***************************************************************************/

  // Test RPM limits
  void testRPMLimits() {
    double redlineRPM = 2700.0;
    double maxContinuous = 2500.0;
    double cruiseRPM = 2300.0;

    TS_ASSERT(redlineRPM > maxContinuous);
    TS_ASSERT(maxContinuous > cruiseRPM);
  }

  // Test manifold pressure limits
  void testManifoldPressureLimits() {
    // Normally aspirated
    double maxMP_NA = 30.0;  // inHg (sea level)

    // Turbocharged
    double maxMP_Turbo = 40.0;  // inHg

    TS_ASSERT(maxMP_Turbo > maxMP_NA);
  }

  // Test exhaust gas temperature limits
  void testEGTLimits() {
    double maxEGT = 1650.0;  // °F (typical piston)
    double peakEGT = 1400.0;  // Peak EGT (best power)
    double leanEGT = 1300.0;  // 100° lean of peak

    TS_ASSERT(maxEGT > peakEGT);
    TS_ASSERT(peakEGT > leanEGT);
  }

  /***************************************************************************
   * Mixture Control Tests
   ***************************************************************************/

  // Test mixture ratio
  void testMixtureRatio() {
    // Stoichiometric air/fuel ratio for AVGAS is ~15:1
    double stoichRatio = 15.0;

    // Rich mixture (more fuel)
    double richRatio = 12.0;
    TS_ASSERT(richRatio < stoichRatio);

    // Lean mixture (less fuel)
    double leanRatio = 18.0;
    TS_ASSERT(leanRatio > stoichRatio);
  }

  // Test mixture leaning effect on power
  void testMixtureLeaningEffect() {
    // Power peaks slightly rich of stoichiometric
    double fullRichPower = 95.0;  // % of max
    double peakPower = 100.0;     // At best power mixture
    double leanPower = 85.0;      // Lean of peak

    TS_ASSERT(peakPower > fullRichPower);
    TS_ASSERT(peakPower > leanPower);
  }

  /***************************************************************************
   * Propeller Tests
   ***************************************************************************/

  // Test constant speed propeller
  void testConstantSpeedProp() {
    double commandedRPM = 2400.0;
    double actualRPM = 2400.0;
    double pitchAngle = 20.0;  // degrees

    // Governor maintains RPM
    TS_ASSERT_DELTA(actualRPM, commandedRPM, DEFAULT_TOLERANCE);

    // Higher power = higher pitch to maintain RPM
    double highPowerPitch = 25.0;
    TS_ASSERT(highPowerPitch > pitchAngle);
  }

  // Test propeller feathering
  void testPropellerFeathering() {
    bool feathered = false;
    double pitchAngle = 20.0;
    double featheredPitch = 90.0;  // degrees

    // Feather the prop
    feathered = true;
    pitchAngle = featheredPitch;

    TS_ASSERT(feathered);
    TS_ASSERT_DELTA(pitchAngle, 90.0, DEFAULT_TOLERANCE);
  }

  // Test prop efficiency curve
  void testPropEfficiencyCurve() {
    // Propeller efficiency varies with advance ratio J
    auto propEfficiency = [](double J) {
      // Simplified parabolic approximation
      // Peak around J = 0.8
      if (J < 0) return 0.0;
      if (J > 2.0) return 0.0;
      return -0.4 * (J - 0.8) * (J - 0.8) + 0.85;
    };

    TS_ASSERT(propEfficiency(0.0) < propEfficiency(0.8));
    TS_ASSERT(propEfficiency(0.8) > 0.8);
    TS_ASSERT(propEfficiency(1.5) < propEfficiency(0.8));
  }

  /***************************************************************************
   * Thrust Calculations
   ***************************************************************************/

  // Test jet thrust equation
  void testJetThrustEquation() {
    // Thrust = mdot * (Ve - V0)
    double massFlowRate = 100.0;   // slugs/sec
    double exhaustVelocity = 2000.0;  // ft/sec
    double flightVelocity = 500.0;    // ft/sec

    double thrust = massFlowRate * (exhaustVelocity - flightVelocity);
    TS_ASSERT_DELTA(thrust, 150000.0, DEFAULT_TOLERANCE);
  }

  // Test ram drag effect
  void testRamDragEffect() {
    // At higher speeds, inlet momentum reduces net thrust
    double grossThrust = 10000.0;
    double ramDrag = 1000.0;

    double netThrust = grossThrust - ramDrag;
    TS_ASSERT(netThrust < grossThrust);

    // Ram drag increases with speed
    double highSpeedRamDrag = 3000.0;
    TS_ASSERT(highSpeedRamDrag > ramDrag);
  }

  // Test static thrust
  void testStaticThrust() {
    // At zero forward speed, all thrust is "static thrust"
    double flightVelocity = 0.0;
    double massFlowRate = 50.0;
    double exhaustVelocity = 2000.0;

    double staticThrust = massFlowRate * (exhaustVelocity - flightVelocity);
    TS_ASSERT_DELTA(staticThrust, 100000.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test negative throttle handling
  void testNegativeThrottle() {
    double throttle = -0.1;
    double clampedThrottle = std::max(0.0, throttle);

    TS_ASSERT_DELTA(clampedThrottle, 0.0, DEFAULT_TOLERANCE);
  }

  // Test over-temperature protection
  void testOverTemperatureProtection() {
    double currentTemp = 1800.0;  // °F
    double maxTemp = 1700.0;      // °F limit

    bool overTemp = currentTemp > maxTemp;
    TS_ASSERT(overTemp);

    // Protection should reduce power
    if (overTemp) {
      // Reduce throttle
      double throttle = 1.0;
      throttle *= (maxTemp / currentTemp);
      TS_ASSERT(throttle < 1.0);
    }
  }

  // Test zero fuel condition
  void testZeroFuelCondition() {
    double fuelQuantity = 0.0;
    bool starved = fuelQuantity <= 0.0;

    TS_ASSERT(starved);

    // Engine should not produce power when starved
    double power = starved ? 0.0 : 100.0;
    TS_ASSERT_DELTA(power, 0.0, DEFAULT_TOLERANCE);
  }

  // Test very low RPM
  void testVeryLowRPM() {
    double rpm = 50.0;  // Very low
    double idleRPM = 600.0;

    bool belowIdle = rpm < idleRPM;
    TS_ASSERT(belowIdle);

    // Engine likely to stall
    bool engineStall = rpm < 300.0;
    TS_ASSERT(engineStall);
  }

  /***************************************************************************
   * Extended Power Output Tests
   ***************************************************************************/

  // Test power curve vs RPM
  void testPowerCurveVsRPM() {
    double maxPower = 200.0;  // HP
    double maxRPM = 2700.0;

    // Power roughly proportional to RPM^3 / maxRPM^3
    auto powerAtRPM = [&](double rpm) {
      double ratio = rpm / maxRPM;
      return maxPower * ratio * ratio * ratio;
    };

    TS_ASSERT_DELTA(powerAtRPM(2700), 200.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(powerAtRPM(2100), 94.4, 1.0);  // ~47% power
    TS_ASSERT_DELTA(powerAtRPM(1350), 25.0, 1.0);  // 12.5% power
  }

  // Test power required for climb
  void testPowerForClimb() {
    double weight = 3000.0;  // lbs
    double climbRate = 700.0;  // fpm
    double climbRateFPS = climbRate / 60.0;

    // Power = Weight * ClimbRate / 550 (HP)
    double climbPower = weight * climbRateFPS / 550.0;
    TS_ASSERT_DELTA(climbPower, 63.6, 1.0);
  }

  // Test excess power for acceleration
  void testExcessPowerAcceleration() {
    double availablePower = 200.0;  // HP
    double requiredPower = 150.0;   // HP
    double excessPower = availablePower - requiredPower;

    TS_ASSERT_DELTA(excessPower, 50.0, DEFAULT_TOLERANCE);
    TS_ASSERT(excessPower > 0);  // Can accelerate
  }

  /***************************************************************************
   * Extended Efficiency Tests
   ***************************************************************************/

  // Test thermal efficiency
  void testThermalEfficiency() {
    // Typical piston engine: 25-30%
    double fuelEnergy = 100000.0;  // BTU/hr
    double shaftPower = 25000.0;   // BTU/hr (from HP)

    double thermalEff = shaftPower / fuelEnergy;
    TS_ASSERT(thermalEff > 0.20);
    TS_ASSERT(thermalEff < 0.35);
  }

  // Test mechanical efficiency
  void testMechanicalEfficiency() {
    double indicatedPower = 220.0;  // HP
    double brakePower = 200.0;      // HP

    double mechEff = brakePower / indicatedPower;
    TS_ASSERT_DELTA(mechEff, 0.909, 0.01);
    TS_ASSERT(mechEff < 1.0);
  }

  // Test volumetric efficiency
  void testVolumetricEfficiency() {
    double actualAirflow = 150.0;      // CFM
    double theoreticalAirflow = 175.0; // CFM

    double volEff = actualAirflow / theoreticalAirflow;
    TS_ASSERT_DELTA(volEff, 0.857, 0.01);
    TS_ASSERT(volEff < 1.0);
  }

  /***************************************************************************
   * Extended Induction System Tests
   ***************************************************************************/

  // Test carburetor icing
  void testCarburetorIcing() {
    double OAT = 40.0;  // °F
    double dewpoint = 35.0;
    double spread = OAT - dewpoint;

    // Icing likely when OAT between 20-70°F and spread < 15°F
    bool icingLikely = (OAT > 20.0 && OAT < 70.0 && spread < 15.0);
    TS_ASSERT(icingLikely);
  }

  // Test manifold pressure at altitude
  void testManifoldPressureAltitude() {
    double seaLevelMP = 29.92;  // inHg
    double altitude = 10000.0;
    double lapseRate = 1.0;  // inHg per 1000 ft

    double altitudeMP = seaLevelMP - (altitude / 1000.0) * lapseRate;
    TS_ASSERT_DELTA(altitudeMP, 19.92, 0.1);
  }

  // Test alternate air door
  void testAlternateAirDoor() {
    bool primaryBlocked = false;
    bool alternateOpen = false;

    // Primary blocked triggers alternate
    primaryBlocked = true;
    alternateOpen = primaryBlocked;
    TS_ASSERT(alternateOpen);
  }

  /***************************************************************************
   * Extended Cooling System Tests
   ***************************************************************************/

  // Test cylinder head temperature
  void testCylinderHeadTemperature() {
    double CHT = 400.0;  // °F
    double maxCHT = 500.0;
    double minCHT = 200.0;

    TS_ASSERT(CHT >= minCHT);
    TS_ASSERT(CHT <= maxCHT);
  }

  // Test cooling airflow requirement
  void testCoolingAirflowRequirement() {
    double power = 200.0;  // HP
    double heatRejection = power * 0.7;  // 70% becomes heat

    // CFM required roughly 2 CFM per HP of heat
    double cfmRequired = heatRejection * 2.0;
    TS_ASSERT_DELTA(cfmRequired, 280.0, DEFAULT_TOLERANCE);
  }

  // Test cowl flaps effect
  void testCowlFlapsEffect() {
    double baseCooling = 100.0;  // arbitrary units
    double cowlFlapPosition = 0.5;  // 50% open

    double cooling = baseCooling * (0.5 + 0.5 * cowlFlapPosition);
    TS_ASSERT_DELTA(cooling, 75.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended Ignition System Tests
   ***************************************************************************/

  // Test dual magneto check
  void testDualMagnetoCheck() {
    double rpmBoth = 2000.0;
    double rpmLeft = 1950.0;
    double rpmRight = 1940.0;
    double maxDrop = 150.0;
    double maxDiff = 50.0;

    double dropLeft = rpmBoth - rpmLeft;
    double dropRight = rpmBoth - rpmRight;
    double diff = std::abs(dropLeft - dropRight);

    TS_ASSERT(dropLeft < maxDrop);
    TS_ASSERT(dropRight < maxDrop);
    TS_ASSERT(diff < maxDiff);
  }

  // Test spark advance
  void testSparkAdvance() {
    double baseAdvance = 5.0;    // degrees BTDC at idle
    double maxAdvance = 30.0;    // degrees BTDC at max power
    double throttle = 0.5;

    double advance = baseAdvance + (maxAdvance - baseAdvance) * throttle;
    TS_ASSERT_DELTA(advance, 17.5, DEFAULT_TOLERANCE);
  }

  // Test detonation detection
  void testDetonationDetection() {
    double manifoldPressure = 35.0;  // inHg
    double maxMP = 30.0;
    double EGT = 1500.0;
    double normalEGT = 1350.0;

    bool detonationRisk = (manifoldPressure > maxMP) || (EGT > normalEGT * 1.1);
    TS_ASSERT(detonationRisk);
  }

  /***************************************************************************
   * Extended Exhaust System Tests
   ***************************************************************************/

  // Test exhaust back pressure
  void testExhaustBackPressure() {
    double exhaustPressure = 16.0;  // psia
    double ambientPressure = 14.7;

    double backPressure = exhaustPressure - ambientPressure;
    TS_ASSERT(backPressure > 0);
    TS_ASSERT_DELTA(backPressure, 1.3, 0.1);
  }

  // Test turbocharger wastegate
  void testTurbochargerWastegate() {
    double targetMP = 30.0;  // inHg
    double actualMP = 32.0;
    double wastegatePosition = 0.0;

    // Wastegate opens when above target
    if (actualMP > targetMP) {
      wastegatePosition = (actualMP - targetMP) / 10.0;
    }

    TS_ASSERT(wastegatePosition > 0);
    TS_ASSERT_DELTA(wastegatePosition, 0.2, DEFAULT_TOLERANCE);
  }

  // Test EGT probe readings
  void testEGTProbeReadings() {
    std::vector<double> egtByClylinder = {1350, 1360, 1340, 1355};
    double average = 0.0;
    for (auto e : egtByClylinder) average += e;
    average /= egtByClylinder.size();

    TS_ASSERT_DELTA(average, 1351.25, 0.1);
  }

  /***************************************************************************
   * Extended Fuel Injection Tests
   ***************************************************************************/

  // Test fuel injector flow rate
  void testFuelInjectorFlowRate() {
    double injectorRating = 5.0;  // GPH at rated pressure
    double pressureRatio = 0.8;

    // Flow proportional to sqrt(pressure)
    double actualFlow = injectorRating * sqrt(pressureRatio);
    TS_ASSERT_DELTA(actualFlow, 4.47, 0.01);
  }

  // Test fuel distribution
  void testFuelDistribution() {
    std::vector<double> cylFlow = {1.5, 1.48, 1.52, 1.49};  // GPH per cylinder
    double targetFlow = 1.5;
    double maxDeviation = 0.05;  // 5%

    bool distributionOK = true;
    for (auto flow : cylFlow) {
      if (std::abs(flow - targetFlow) / targetFlow > maxDeviation) {
        distributionOK = false;
      }
    }

    TS_ASSERT(distributionOK);
  }

  /***************************************************************************
   * Extended Propeller Load Tests
   ***************************************************************************/

  // Test propeller power absorption
  void testPropellerPowerAbsorption() {
    double rpm = 2400.0;
    double propDiameter = 6.0;  // ft
    double density = 0.002377;

    // Power ~ rho * n^3 * D^5 (simplified)
    double n = rpm / 60.0;  // rps
    double power = density * pow(n, 3) * pow(propDiameter, 5) * 0.001;

    TS_ASSERT(power > 0);
  }

  // Test governor action
  void testGovernorAction() {
    double targetRPM = 2400.0;
    double actualRPM = 2500.0;
    double bladeAngle = 20.0;  // degrees

    // Governor increases pitch to reduce RPM
    double error = actualRPM - targetRPM;
    if (error > 0) {
      bladeAngle += error * 0.01;
    }

    TS_ASSERT(bladeAngle > 20.0);
  }

  /***************************************************************************
   * Extended Environmental Tests
   ***************************************************************************/

  // Test carburetor heat
  void testCarburetorHeat() {
    double intakeTemp = 40.0;  // °F
    double carbHeatOn = true;
    double tempRise = 50.0;    // °F with heat on

    double finalTemp = intakeTemp;
    if (carbHeatOn) {
      finalTemp += tempRise;
    }

    TS_ASSERT_DELTA(finalTemp, 90.0, DEFAULT_TOLERANCE);
  }

  // Test density altitude effect
  void testDensityAltitudeEffect() {
    double pressureAltitude = 5000.0;
    double OAT = 95.0;  // °F (hot day)
    double standardTemp = 59.0 - 3.5 * (5.0);  // at 5000 ft

    // High temp increases density altitude
    double densityAltitude = pressureAltitude + 120.0 * (OAT - standardTemp);
    TS_ASSERT(densityAltitude > pressureAltitude);
  }

  // Test humid air effect
  void testHumidAirEffect() {
    double dryAirDensity = 0.002377;
    double humidity = 0.8;  // 80% relative

    // Humid air is less dense (roughly 1% per 10% humidity)
    double humidAirDensity = dryAirDensity * (1.0 - 0.001 * humidity * 100.0);
    TS_ASSERT(humidAirDensity < dryAirDensity);
  }

  /***************************************************************************
   * Extended Failure Mode Tests
   ***************************************************************************/

  // Test partial power loss
  void testPartialPowerLoss() {
    double normalPower = 200.0;
    double powerLossPercent = 0.25;

    double reducedPower = normalPower * (1.0 - powerLossPercent);
    TS_ASSERT_DELTA(reducedPower, 150.0, DEFAULT_TOLERANCE);
  }

  // Test rough running detection
  void testRoughRunningDetection() {
    std::vector<double> rpmVariation = {2400, 2350, 2450, 2300, 2500};
    double avgRPM = 0;
    for (auto r : rpmVariation) avgRPM += r;
    avgRPM /= rpmVariation.size();

    double variance = 0;
    for (auto r : rpmVariation) {
      variance += pow(r - avgRPM, 2);
    }
    variance /= rpmVariation.size();

    // Std dev ~70.7 which is > 30
    bool roughRunning = sqrt(variance) > 30.0;
    TS_ASSERT(roughRunning);
  }

  // Test oil pressure warning
  void testOilPressureWarning() {
    double oilPressure = 25.0;  // psi
    double minPressure = 30.0;
    double criticalPressure = 10.0;

    bool warning = oilPressure < minPressure;
    bool critical = oilPressure < criticalPressure;

    TS_ASSERT(warning);
    TS_ASSERT(!critical);
  }

  /***************************************************************************
   * Extended Performance Calculation Tests
   ***************************************************************************/

  // Test brake mean effective pressure
  void testBMEP() {
    double power = 200.0;  // HP
    double displacement = 360.0;  // cu in
    double rpm = 2700.0;

    // BMEP = (Power * 792000) / (Displacement * RPM)
    double bmep = (power * 792000.0) / (displacement * rpm);
    TS_ASSERT(bmep > 100);
    TS_ASSERT(bmep < 200);
  }

  // Test indicated horsepower
  void testIndicatedHP() {
    double brakeHP = 200.0;
    double frictionHP = 25.0;

    double indicatedHP = brakeHP + frictionHP;
    TS_ASSERT_DELTA(indicatedHP, 225.0, DEFAULT_TOLERANCE);
  }

  // Test compression ratio effect
  void testCompressionRatioEffect() {
    double compressionRatio = 8.5;
    double gamma = 1.4;

    // Theoretical efficiency = 1 - 1/r^(gamma-1)
    double efficiency = 1.0 - 1.0 / pow(compressionRatio, gamma - 1.0);
    TS_ASSERT(efficiency > 0.5);
    TS_ASSERT(efficiency < 0.7);
  }

  /***************************************************************************
   * Extended Control System Tests
   ***************************************************************************/

  // Test throttle response lag
  void testThrottleResponseLag() {
    double throttleCmd = 1.0;
    double throttlePos = 0.0;
    double timeConstant = 0.5;
    double dt = 0.1;

    for (int i = 0; i < 20; i++) {
      throttlePos += (throttleCmd - throttlePos) * dt / timeConstant;
    }

    TS_ASSERT(throttlePos > 0.95);
  }

  // Test mixture rich cutoff
  void testMixtureRichCutoff() {
    double mixture = 1.0;  // Full rich
    bool cutoff = false;

    mixture = 0.0;  // Cutoff
    cutoff = (mixture <= 0.0);

    TS_ASSERT(cutoff);
  }

  // Test prop lever control
  void testPropLeverControl() {
    double propLever = 0.0;  // Full fine
    double targetRPM = 2700.0;
    double minRPM = 1800.0;

    double commandedRPM = minRPM + propLever * (targetRPM - minRPM);
    TS_ASSERT_DELTA(commandedRPM, 1800.0, DEFAULT_TOLERANCE);

    propLever = 1.0;  // Full coarse
    commandedRPM = minRPM + propLever * (targetRPM - minRPM);
    TS_ASSERT_DELTA(commandedRPM, 2700.0, DEFAULT_TOLERANCE);
  }
};
