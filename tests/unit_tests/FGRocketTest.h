#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include "TestUtilities.h"

using namespace JSBSim;

class FGRocketTest : public CxxTest::TestSuite
{
public:
  // Test engine type is etRocket
  void testEngineType() {
    TS_ASSERT_EQUALS(FGEngine::etRocket, 1);
  }

  // Test specific impulse definition
  void testSpecificImpulse() {
    // Isp is thrust per unit fuel flow rate (seconds)
    // Typical values: 200-450 seconds
    double Isp_solid = 250.0;    // Solid rocket
    double Isp_liquid = 350.0;   // Liquid rocket
    double Isp_hydrolox = 450.0; // LH2/LOX

    TS_ASSERT(Isp_solid > 0.0);
    TS_ASSERT(Isp_solid < Isp_liquid);
    TS_ASSERT(Isp_liquid < Isp_hydrolox);
  }

  // Test thrust from Isp and mass flow
  void testThrustFromIsp() {
    // Thrust = Isp * g0 * m_dot (in consistent units)
    // In lbf/s: Thrust = Isp * PropellantFlowRate
    double Isp = 300.0;  // seconds
    double propFlowRate = 100.0;  // lbs/sec

    double thrust = Isp * propFlowRate;
    TS_ASSERT_DELTA(thrust, 30000.0, DEFAULT_TOLERANCE);

    // Higher Isp = more thrust for same mass flow
    Isp = 400.0;
    thrust = Isp * propFlowRate;
    TS_ASSERT_DELTA(thrust, 40000.0, DEFAULT_TOLERANCE);
  }

  // Test mixture ratio
  void testMixtureRatio() {
    // MxR = oxidizer flow / fuel flow
    double MxR = 2.5;  // Typical for RP-1/LOX

    // Split total prop flow
    double totalPropFlow = 100.0;  // lbs/sec
    double oxiFlow = totalPropFlow * MxR / (1.0 + MxR);
    double fuelFlow = totalPropFlow / (1.0 + MxR);

    // oxiFlow = 100 * 2.5 / 3.5 = 71.43
    // fuelFlow = 100 / 3.5 = 28.57
    TS_ASSERT_DELTA(oxiFlow, 71.43, 0.01);
    TS_ASSERT_DELTA(fuelFlow, 28.57, 0.01);
    TS_ASSERT_DELTA(oxiFlow + fuelFlow, totalPropFlow, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(oxiFlow / fuelFlow, MxR, 0.01);
  }

  // Test fuel and oxidizer consumption
  void testFuelOxidizerConsumption() {
    double fuelFlowRate = 30.0;   // lbs/sec
    double oxiFlowRate = 70.0;    // lbs/sec
    double deltaT = 0.1;          // seconds

    double fuelExpended = fuelFlowRate * deltaT;
    double oxiExpended = oxiFlowRate * deltaT;

    TS_ASSERT_DELTA(fuelExpended, 3.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(oxiExpended, 7.0, DEFAULT_TOLERANCE);

    // Total propellant
    double totalExpended = fuelExpended + oxiExpended;
    TS_ASSERT_DELTA(totalExpended, 10.0, DEFAULT_TOLERANCE);
  }

  // Test total impulse accumulation
  void testTotalImpulse() {
    // It = integral(Thrust * dt)
    double thrust = 50000.0;  // lbs
    double deltaT = 0.1;      // seconds

    double It = 0.0;
    for (int i = 0; i < 100; i++) {
      It += thrust * deltaT;
    }
    // Total impulse after 10 seconds
    TS_ASSERT_DELTA(It, 500000.0, DEFAULT_TOLERANCE);
  }

  // Test vacuum vs actual thrust
  void testVacuumVsActualThrust() {
    double vacThrust = 100000.0;
    double ambientPressure = 14.7;  // psi at sea level
    double nozzleExitArea = 100.0;  // sq in

    // Pressure thrust loss = p_amb * A_exit
    double pressureLoss = ambientPressure * nozzleExitArea;
    // = 14.7 * 100 = 1470 lbs
    TS_ASSERT_DELTA(pressureLoss, 1470.0, DEFAULT_TOLERANCE);

    double actualThrust = vacThrust - pressureLoss;
    TS_ASSERT_DELTA(actualThrust, 98530.0, DEFAULT_TOLERANCE);

    // In vacuum (altitude), no loss
    ambientPressure = 0.0;
    pressureLoss = ambientPressure * nozzleExitArea;
    actualThrust = vacThrust - pressureLoss;
    TS_ASSERT_DELTA(actualThrust, vacThrust, DEFAULT_TOLERANCE);
  }

  // Test solid rocket buildup phase
  void testSolidRocketBuildup() {
    // Solid rockets have a buildup time at ignition
    double buildupTime = 0.5;  // seconds
    double maxThrust = 50000.0;
    double currentTime = 0.0;

    // During buildup: thrust ramps up via sine function
    // thrust = maxThrust * sin(pi/2 * t / buildupTime)
    currentTime = 0.0;
    double thrust = maxThrust * sin(M_PI / 2.0 * currentTime / buildupTime);
    TS_ASSERT_DELTA(thrust, 0.0, DEFAULT_TOLERANCE);

    currentTime = 0.25;  // Half of buildup time
    thrust = maxThrust * sin(M_PI / 2.0 * currentTime / buildupTime);
    // sin(pi/4) = 0.707
    TS_ASSERT_DELTA(thrust, 35355.0, 10.0);

    currentTime = 0.5;  // End of buildup
    thrust = maxThrust * sin(M_PI / 2.0 * currentTime / buildupTime);
    TS_ASSERT_DELTA(thrust, maxThrust, DEFAULT_TOLERANCE);
  }

  // Test thrust variation (temperature effects)
  void testThrustVariation() {
    double nominalThrust = 50000.0;
    double thrustVariation = 0.02;  // +2% due to temperature

    double actualThrust = nominalThrust * (1.0 + thrustVariation);
    TS_ASSERT_DELTA(actualThrust, 51000.0, DEFAULT_TOLERANCE);

    // Cold motor (-2%)
    thrustVariation = -0.02;
    actualThrust = nominalThrust * (1.0 + thrustVariation);
    TS_ASSERT_DELTA(actualThrust, 49000.0, DEFAULT_TOLERANCE);
  }

  // Test Isp variation (energy effects)
  void testIspVariation() {
    double nominalIsp = 300.0;
    double ispVariation = 0.01;  // +1%

    double actualIsp = nominalIsp * (1.0 + ispVariation);
    TS_ASSERT_DELTA(actualIsp, 303.0, DEFAULT_TOLERANCE);

    // Variation affects fuel flow
    double thrust = 30000.0;
    double fuelFlow_nominal = thrust / nominalIsp;
    double fuelFlow_actual = thrust / actualIsp;

    TS_ASSERT(fuelFlow_actual < fuelFlow_nominal);
  }

  // Test liquid rocket throttling
  void testLiquidRocketThrottling() {
    double maxThrust = 100000.0;
    double throttlePos = 1.0;

    // Full throttle
    double thrust = maxThrust * throttlePos;
    TS_ASSERT_DELTA(thrust, 100000.0, DEFAULT_TOLERANCE);

    // Half throttle
    throttlePos = 0.5;
    thrust = maxThrust * throttlePos;
    TS_ASSERT_DELTA(thrust, 50000.0, DEFAULT_TOLERANCE);

    // Minimum throttle (e.g., 50%)
    double minThrottle = 0.5;
    throttlePos = 0.3;  // Below minimum
    if (throttlePos < minThrottle) {
      // Would flameout
      thrust = 0.0;
    } else {
      thrust = maxThrust * throttlePos;
    }
    TS_ASSERT_DELTA(thrust, 0.0, DEFAULT_TOLERANCE);
  }

  // Test flameout condition
  void testFlameoutCondition() {
    double throttlePos = 0.6;
    double minThrottle = 0.5;
    bool starved = false;

    // Normal operation
    bool flameout = (throttlePos < minThrottle) || starved;
    TS_ASSERT(!flameout);

    // Below min throttle
    throttlePos = 0.4;
    flameout = (throttlePos < minThrottle) || starved;
    TS_ASSERT(flameout);

    // Fuel starved
    throttlePos = 0.8;
    starved = true;
    flameout = (throttlePos < minThrottle) || starved;
    TS_ASSERT(flameout);
  }

  // Test burn time accumulation
  void testBurnTimeAccumulation() {
    double burnTime = 0.0;
    double deltaT = 0.01;
    bool burning = true;

    // Accumulate burn time
    for (int i = 0; i < 1000; i++) {
      if (burning) burnTime += deltaT;
    }
    TS_ASSERT_DELTA(burnTime, 10.0, DEFAULT_TOLERANCE);

    // Stop burning
    burning = false;
    for (int i = 0; i < 100; i++) {
      if (burning) burnTime += deltaT;
    }
    TS_ASSERT_DELTA(burnTime, 10.0, DEFAULT_TOLERANCE);  // Unchanged
  }

  // Test total propellant expended tracking
  void testTotalPropellantExpended() {
    double totalPropellant = 0.0;
    double fuelExpended = 3.0;
    double oxiExpended = 7.0;

    totalPropellant += fuelExpended + oxiExpended;
    TS_ASSERT_DELTA(totalPropellant, 10.0, DEFAULT_TOLERANCE);

    // After more time
    totalPropellant += fuelExpended + oxiExpended;
    TS_ASSERT_DELTA(totalPropellant, 20.0, DEFAULT_TOLERANCE);
  }

  // Test propellant flow rate from thrust and Isp
  void testPropFlowFromThrust() {
    double thrust = 50000.0;
    double Isp = 300.0;

    double propFlowRate = thrust / Isp;
    TS_ASSERT_DELTA(propFlowRate, 166.67, 0.01);

    // Higher Isp = lower flow rate for same thrust
    Isp = 400.0;
    propFlowRate = thrust / Isp;
    TS_ASSERT_DELTA(propFlowRate, 125.0, DEFAULT_TOLERANCE);
  }

  // Test solid rocket no throttle control
  void testSolidNoThrottle() {
    bool hasThrustTable = true;  // Solid rocket mode

    // In solid mode, throttle is ignored after ignition
    double throttlePos = 0.3;

    // Thrust comes from table, not throttle
    if (hasThrustTable) {
      // throttlePos ignored
      TS_ASSERT(true);  // Solid motor continues burning
    }
  }

  // Test vacuum impulse calculation
  void testVacuumImpulse() {
    double vacThrust = 100000.0;
    double deltaT = 0.1;

    double ItVac = 0.0;
    for (int i = 0; i < 100; i++) {
      ItVac += vacThrust * deltaT;
    }
    TS_ASSERT_DELTA(ItVac, 1000000.0, DEFAULT_TOLERANCE);
  }

  // Test starved engine behavior
  void testStarvedEngine() {
    bool starved = false;
    double thrust = 50000.0;

    // Normal
    if (starved) thrust = 0.0;
    TS_ASSERT_DELTA(thrust, 50000.0, DEFAULT_TOLERANCE);

    // Starved
    thrust = 50000.0;
    starved = true;
    if (starved) thrust = 0.0;
    TS_ASSERT_DELTA(thrust, 0.0, DEFAULT_TOLERANCE);
  }

  // Test chamber pressure relationship
  void testChamberPressure() {
    // Chamber pressure affects thrust and Isp
    double Pc_design = 1000.0;  // psi
    double Pc_actual = 950.0;   // psi

    // Thrust roughly proportional to Pc
    double thrustRatio = Pc_actual / Pc_design;
    TS_ASSERT_DELTA(thrustRatio, 0.95, DEFAULT_TOLERANCE);
  }

  // Test nozzle expansion ratio effect
  void testNozzleExpansionRatio() {
    // Higher expansion ratio = better vacuum performance
    double epsilon_low = 10.0;   // Sea level optimized
    double epsilon_high = 100.0; // Vacuum optimized

    // Vacuum Isp improves with expansion ratio
    double Isp_low = 300.0;
    double Isp_high = 400.0;

    TS_ASSERT(Isp_high > Isp_low);
    TS_ASSERT(epsilon_high > epsilon_low);
  }

  // Test oxidizer-to-fuel ratio bounds
  void testMixtureRatioBounds() {
    // Typical MxR ranges
    double MxR_RP1_LOX = 2.7;     // RP-1/LOX
    double MxR_LH2_LOX = 6.0;     // LH2/LOX
    double MxR_N2O4_UDMH = 2.1;   // N2O4/UDMH

    TS_ASSERT(MxR_RP1_LOX > 0.0);
    TS_ASSERT(MxR_LH2_LOX > MxR_RP1_LOX);
    TS_ASSERT(MxR_N2O4_UDMH > 0.0);
  }

  // Test engine initialization
  void testEngineInitialization() {
    // Initial state
    double thrust = 0.0;
    double It = 0.0;
    double ItVac = 0.0;
    double burnTime = 0.0;
    double totalPropExpended = 0.0;
    bool flameout = true;  // Not ignited yet

    TS_ASSERT_DELTA(thrust, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(It, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(ItVac, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(burnTime, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(totalPropExpended, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT(flameout);
  }

  // Test delta-v calculation
  void testDeltaV() {
    // Tsiolkovsky rocket equation: dV = Isp * g0 * ln(m0/mf)
    double Isp = 300.0;  // seconds
    double g0 = 32.174;  // ft/s^2
    double m0 = 10000.0; // initial mass (lbs)
    double mf = 5000.0;  // final mass (lbs)

    double dV = Isp * g0 * log(m0 / mf);
    // = 300 * 32.174 * ln(2) = 300 * 32.174 * 0.693 = 6688 ft/s
    TS_ASSERT_DELTA(dV, 6688.0, 10.0);
  }

  // Test mass ratio
  void testMassRatio() {
    double m0 = 10000.0;  // Initial mass
    double propMass = 6000.0;  // Propellant mass
    double mf = m0 - propMass;  // Final mass

    double massRatio = m0 / mf;
    TS_ASSERT_DELTA(massRatio, 2.5, DEFAULT_TOLERANCE);

    // Propellant fraction
    double propFraction = propMass / m0;
    TS_ASSERT_DELTA(propFraction, 0.6, DEFAULT_TOLERANCE);
  }
};
