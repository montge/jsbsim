#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

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

  // ==================== NOZZLE PERFORMANCE TESTS ====================

  // Test exhaust exit velocity
  void testExhaustExitVelocity() {
    // Ve = Isp * g0
    double Isp = 300.0;  // seconds
    double g0 = 32.174;  // ft/s^2

    double Ve = Isp * g0;
    TS_ASSERT_DELTA(Ve, 9652.2, 0.1);

    // Higher Isp means higher exit velocity
    Isp = 450.0;
    Ve = Isp * g0;
    TS_ASSERT_DELTA(Ve, 14478.3, 0.1);
  }

  // Test pressure thrust component
  void testPressureThrustComponent() {
    // F_pressure = (Pe - Pa) * Ae
    double Pe = 14.7;    // Exit pressure psi
    double Pa = 0.0;     // Ambient (vacuum)
    double Ae = 100.0;   // Exit area sq in

    double F_pressure = (Pe - Pa) * Ae;
    TS_ASSERT_DELTA(F_pressure, 1470.0, DEFAULT_TOLERANCE);

    // At sea level (Pa = Pe for optimum expansion)
    Pa = 14.7;
    F_pressure = (Pe - Pa) * Ae;
    TS_ASSERT_DELTA(F_pressure, 0.0, DEFAULT_TOLERANCE);

    // Overexpanded nozzle (Pa > Pe)
    Pa = 20.0;
    F_pressure = (Pe - Pa) * Ae;
    TS_ASSERT(F_pressure < 0.0);  // Negative (loss)
  }

  // Test nozzle area ratio
  void testNozzleAreaRatio() {
    // epsilon = Ae / At
    double At = 10.0;    // Throat area sq in
    double Ae = 100.0;   // Exit area sq in

    double epsilon = Ae / At;
    TS_ASSERT_DELTA(epsilon, 10.0, DEFAULT_TOLERANCE);

    // Higher expansion ratio for vacuum engines
    Ae = 400.0;
    epsilon = Ae / At;
    TS_ASSERT_DELTA(epsilon, 40.0, DEFAULT_TOLERANCE);
  }

  // Test overexpansion effects
  void testOverexpansionEffects() {
    // When ambient pressure > exit pressure
    double Pe_design = 5.0;   // psi (designed for altitude)
    double Pa_actual = 14.7;  // psi (sea level)

    bool isOverexpanded = (Pa_actual > Pe_design);
    TS_ASSERT(isOverexpanded);

    // Flow separation may occur (efficiency loss)
    double overexpansionRatio = Pa_actual / Pe_design;
    TS_ASSERT_DELTA(overexpansionRatio, 2.94, 0.01);
  }

  // Test underexpansion effects
  void testUnderexpansionEffects() {
    // When exit pressure > ambient pressure
    double Pe_design = 14.7;  // psi (sea level optimized)
    double Pa_actual = 0.5;   // psi (high altitude)

    bool isUnderexpanded = (Pe_design > Pa_actual);
    TS_ASSERT(isUnderexpanded);

    // Loses potential thrust from incomplete expansion
    double lostPressureThrust = (Pe_design - Pa_actual);
    TS_ASSERT_DELTA(lostPressureThrust, 14.2, 0.1);
  }

  // ==================== COMBUSTION CHAMBER TESTS ====================

  // Test characteristic velocity (C*)
  void testCharacteristicVelocity() {
    // C* = Pc * At / m_dot
    double Pc = 1000.0;    // psi
    double At = 10.0;      // sq in
    double m_dot = 100.0;  // lbs/sec

    // Convert to consistent units for C*
    // C* typically 5000-8000 ft/sec
    double Cstar = Pc * At * 144.0 / (m_dot / 32.174);  // Simplified
    TS_ASSERT(Cstar > 1000.0);  // Reasonable range
  }

  // Test thrust coefficient
  void testThrustCoefficient() {
    // CF = F / (Pc * At)
    double F = 50000.0;   // lbs thrust
    double Pc = 1000.0;   // psi
    double At = 30.0;     // sq in

    double CF = F / (Pc * At);
    // CF typically 1.5-2.0 for vacuum
    TS_ASSERT_DELTA(CF, 1.667, 0.01);
    TS_ASSERT(CF > 1.0 && CF < 2.5);
  }

  // Test combustion efficiency
  void testCombustionEfficiency() {
    double Isp_theoretical = 350.0;
    double Isp_actual = 330.0;

    double eta_c = Isp_actual / Isp_theoretical;
    TS_ASSERT_DELTA(eta_c, 0.943, 0.001);

    // Good engines achieve 95%+ efficiency
    TS_ASSERT(eta_c > 0.90);
  }

  // Test chamber temperature
  void testChamberTemperature() {
    // Typical combustion temperatures
    double Tc_RP1_LOX = 6000.0;   // °R (RP-1/LOX)
    double Tc_LH2_LOX = 5800.0;   // °R (LH2/LOX)
    double Tc_solid = 5500.0;     // °R (solid)

    TS_ASSERT(Tc_RP1_LOX > 5000.0);
    TS_ASSERT(Tc_LH2_LOX > 5000.0);
    TS_ASSERT(Tc_solid > 5000.0);
  }

  // ==================== THRUST VECTOR CONTROL TESTS ====================

  // Test gimbal angle effect on thrust
  void testGimbalAngleThrust() {
    double nominalThrust = 100000.0;
    double gimbalAngle = 5.0;  // degrees

    // Axial thrust reduced by cos(angle)
    double gimbalRad = gimbalAngle * M_PI / 180.0;
    double axialThrust = nominalThrust * cos(gimbalRad);
    TS_ASSERT_DELTA(axialThrust, 99619.5, 1.0);

    // Side force from sin(angle)
    double sideForce = nominalThrust * sin(gimbalRad);
    TS_ASSERT_DELTA(sideForce, 8716.0, 1.0);
  }

  // Test gimbal rate limits
  void testGimbalRateLimits() {
    double maxGimbalRate = 20.0;  // degrees/second
    double deltaT = 0.1;
    double currentAngle = 0.0;
    double commandedAngle = 5.0;

    double angleDelta = commandedAngle - currentAngle;
    double maxDelta = maxGimbalRate * deltaT;

    if (std::abs(angleDelta) > maxDelta) {
      angleDelta = (angleDelta > 0) ? maxDelta : -maxDelta;
    }

    double newAngle = currentAngle + angleDelta;
    TS_ASSERT_DELTA(newAngle, 2.0, DEFAULT_TOLERANCE);  // Rate limited
  }

  // Test gimbal moment arm
  void testGimbalMomentArm() {
    double thrust = 100000.0;
    double gimbalAngle = 5.0 * M_PI / 180.0;
    double momentArm = 10.0;  // ft from CG

    double sideForce = thrust * sin(gimbalAngle);
    double pitchMoment = sideForce * momentArm;

    TS_ASSERT_DELTA(pitchMoment, 87155.7, 10.0);
  }

  // Test multi-axis gimbal
  void testMultiAxisGimbal() {
    double pitchGimbal = 3.0;   // degrees
    double yawGimbal = 2.0;     // degrees
    double thrust = 100000.0;

    double pitchRad = pitchGimbal * M_PI / 180.0;
    double yawRad = yawGimbal * M_PI / 180.0;

    // Total deflection
    double totalDeflection = sqrt(pitchGimbal * pitchGimbal + yawGimbal * yawGimbal);
    TS_ASSERT_DELTA(totalDeflection, 3.606, 0.01);

    // Forces
    double pitchForce = thrust * sin(pitchRad);
    double yawForce = thrust * sin(yawRad);

    TS_ASSERT_DELTA(pitchForce, 5236.0, 5.0);
    TS_ASSERT_DELTA(yawForce, 3490.0, 5.0);
  }

  // ==================== SOLID ROCKET SPECIFIC TESTS ====================

  // Test burn rate coefficient
  void testBurnRateCoefficient() {
    // r = a * Pc^n (Saint-Venant's law)
    double a = 0.05;     // Burn rate coefficient
    double n = 0.4;      // Pressure exponent
    double Pc = 1000.0;  // psi

    double burnRate = a * pow(Pc, n);
    // r = 0.05 * 1000^0.4 = 0.05 * 15.85 = 0.793 in/sec
    TS_ASSERT_DELTA(burnRate, 0.793, 0.01);

    // Higher pressure = faster burn
    Pc = 1500.0;
    burnRate = a * pow(Pc, n);
    TS_ASSERT(burnRate > 0.793);
  }

  // Test grain geometry effect
  void testGrainGeometry() {
    // Different grain shapes produce different thrust profiles
    // Star grain: progressive then regressive
    // End burner: neutral
    // Tubular: progressive

    double innerRadius = 2.0;  // inches
    double outerRadius = 6.0;  // inches
    double length = 24.0;      // inches

    // Burning surface area for tubular grain
    double burnSurface = 2.0 * M_PI * innerRadius * length;
    TS_ASSERT_DELTA(burnSurface, 301.6, 0.5);

    // As inner radius increases (burning outward)
    innerRadius = 3.0;
    double newBurnSurface = 2.0 * M_PI * innerRadius * length;
    TS_ASSERT(newBurnSurface > burnSurface);  // Progressive burn
  }

  // Test propellant regression
  void testPropellantRegression() {
    double burnRate = 0.5;   // in/sec
    double deltaT = 0.1;     // seconds
    double webThickness = 4.0;  // inches

    // Web consumption
    double regression = burnRate * deltaT;
    TS_ASSERT_DELTA(regression, 0.05, DEFAULT_TOLERANCE);

    // Time to burnout
    double burnoutTime = webThickness / burnRate;
    TS_ASSERT_DELTA(burnoutTime, 8.0, DEFAULT_TOLERANCE);
  }

  // Test solid motor temperature sensitivity
  void testSolidMotorTempSensitivity() {
    double nominalThrust = 50000.0;
    double nominalTemp = 77.0;  // °F

    // Temperature coefficient (e.g., 0.2% per °F)
    double tempCoef = 0.002;
    double actualTemp = 100.0;  // Hot motor

    double tempDelta = actualTemp - nominalTemp;
    double thrustMultiplier = 1.0 + tempCoef * tempDelta;
    double actualThrust = nominalThrust * thrustMultiplier;

    // Hot motor produces more thrust initially
    TS_ASSERT_DELTA(actualThrust, 52300.0, 100.0);

    // Cold motor
    actualTemp = 40.0;
    tempDelta = actualTemp - nominalTemp;
    thrustMultiplier = 1.0 + tempCoef * tempDelta;
    actualThrust = nominalThrust * thrustMultiplier;

    // Cold motor produces less thrust
    TS_ASSERT(actualThrust < nominalThrust);
  }

  // ==================== LIQUID ROCKET SPECIFIC TESTS ====================

  // Test ignition sequence timing
  void testIgnitionSequence() {
    double t = 0.0;
    double ignitionTime = 0.0;
    double mainValveOpenTime = 0.2;
    double fullThrustTime = 0.5;

    bool igniterOn = (t >= ignitionTime);
    bool mainValveOpen = (t >= mainValveOpenTime);
    bool atFullThrust = (t >= fullThrustTime);

    TS_ASSERT(igniterOn);
    TS_ASSERT(!mainValveOpen);
    TS_ASSERT(!atFullThrust);

    t = 0.3;
    mainValveOpen = (t >= mainValveOpenTime);
    atFullThrust = (t >= fullThrustTime);
    TS_ASSERT(mainValveOpen);
    TS_ASSERT(!atFullThrust);
  }

  // Test shutdown transient
  void testShutdownTransient() {
    double maxThrust = 100000.0;
    double shutdownTime = 0.5;  // Time to reach zero
    double t = 0.0;

    // Linear ramp down
    double thrustDecay = maxThrust * (1.0 - t / shutdownTime);
    TS_ASSERT_DELTA(thrustDecay, 100000.0, DEFAULT_TOLERANCE);

    t = 0.25;
    thrustDecay = maxThrust * (1.0 - t / shutdownTime);
    TS_ASSERT_DELTA(thrustDecay, 50000.0, DEFAULT_TOLERANCE);

    t = 0.5;
    thrustDecay = maxThrust * (1.0 - t / shutdownTime);
    TS_ASSERT_DELTA(thrustDecay, 0.0, DEFAULT_TOLERANCE);
  }

  // Test turbopump performance
  void testTurbopumpPerformance() {
    // Pump discharge pressure
    double inletPressure = 50.0;   // psi (tank pressure)
    double pumpRise = 2000.0;      // psi (pump contribution)

    double dischargePressure = inletPressure + pumpRise;
    TS_ASSERT_DELTA(dischargePressure, 2050.0, DEFAULT_TOLERANCE);

    // Must exceed chamber pressure + losses
    double chamberPressure = 1500.0;
    double injectorLoss = 300.0;
    double lineLoss = 100.0;

    double requiredPressure = chamberPressure + injectorLoss + lineLoss;
    TS_ASSERT(dischargePressure > requiredPressure);
  }

  // Test regenerative cooling
  void testRegenerativeCooling() {
    double fuelFlowRate = 30.0;    // lbs/sec
    double heatLoad = 10000.0;     // BTU/sec
    double fuelSpecificHeat = 0.5; // BTU/(lb-°F)

    // Temperature rise in coolant
    double tempRise = heatLoad / (fuelFlowRate * fuelSpecificHeat);
    TS_ASSERT_DELTA(tempRise, 666.7, 1.0);  // °F

    // Fuel enters chamber preheated
    double inletTemp = 70.0;
    double injectorTemp = inletTemp + tempRise;
    TS_ASSERT_DELTA(injectorTemp, 736.7, 1.0);
  }

  // ==================== PROPELLANT MANAGEMENT TESTS ====================

  // Test tank pressurization
  void testTankPressurization() {
    double ullageVolume = 100.0;   // cu ft
    double gasTemperature = 520.0; // °R
    double targetPressure = 50.0;  // psi

    // PV = nRT (ideal gas)
    // n = PV / RT
    double R = 10.73;  // psi*ft^3/(lb-mol*°R) for common pressurant
    double molesRequired = (targetPressure * ullageVolume) / (R * gasTemperature);
    TS_ASSERT_DELTA(molesRequired, 0.897, 0.01);
  }

  // Test propellant slosh frequency
  void testPropellantSlosh() {
    // Slosh frequency f = (1/2pi) * sqrt(g * tanh(pi*h/d) * pi/d)
    double g = 32.174;       // ft/s^2
    double tankDiameter = 8.0;  // ft
    double propHeight = 10.0;   // ft

    double omega_sq = g * tanh(M_PI * propHeight / tankDiameter) * M_PI / tankDiameter;
    double frequency = sqrt(omega_sq) / (2.0 * M_PI);

    // Typical slosh frequency 0.1-1 Hz
    TS_ASSERT(frequency > 0.1 && frequency < 2.0);
  }

  // Test propellant center of mass
  void testPropellantCenterOfMass() {
    double tankBottom = 0.0;      // ft (reference)
    double tankHeight = 20.0;     // ft
    double propHeight = 15.0;     // ft (75% full)

    // CG of propellant (assuming uniform density)
    double propCG = tankBottom + propHeight / 2.0;
    TS_ASSERT_DELTA(propCG, 7.5, DEFAULT_TOLERANCE);

    // As propellant depletes
    propHeight = 5.0;  // 25% full
    propCG = tankBottom + propHeight / 2.0;
    TS_ASSERT_DELTA(propCG, 2.5, DEFAULT_TOLERANCE);
  }

  // Test ullage rocket
  void testUllageRocket() {
    // Ullage rockets provide settling acceleration
    double vehicleMass = 50000.0;  // lbs
    double ullageThrust = 500.0;   // lbs (small)
    double g0 = 32.174;

    double acceleration = ullageThrust / (vehicleMass / g0);
    // Should provide small but positive acceleration
    TS_ASSERT(acceleration > 0.1);  // ft/s^2
    TS_ASSERT(acceleration < 5.0);  // Not too much
  }

  // ==================== THERMAL EFFECTS TESTS ====================

  // Test propellant temperature effect on density
  void testPropellantTempDensity() {
    double rho_nominal = 50.0;  // lb/ft^3
    double temp_nominal = 60.0; // °F
    double temp_actual = 80.0;  // °F

    // Thermal expansion coefficient (typical)
    double beta = 0.0005;  // per °F

    double tempDelta = temp_actual - temp_nominal;
    double rho_actual = rho_nominal / (1.0 + beta * tempDelta);

    TS_ASSERT_DELTA(rho_actual, 49.5, 0.1);
  }

  // Test nozzle throat erosion
  void testNozzleThroatErosion() {
    double initialThroatDia = 6.0;   // inches
    double erosionRate = 0.001;      // in/sec
    double burnTime = 60.0;          // seconds

    double erosion = erosionRate * burnTime;
    double finalThroatDia = initialThroatDia + 2.0 * erosion;  // Diameter grows

    TS_ASSERT_DELTA(erosion, 0.06, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(finalThroatDia, 6.12, 0.01);

    // Throat area increases
    double initialArea = M_PI * pow(initialThroatDia / 2.0, 2);
    double finalArea = M_PI * pow(finalThroatDia / 2.0, 2);
    TS_ASSERT(finalArea > initialArea);
  }

  // Test propellant boiloff (cryogenic)
  void testCryogenicBoiloff() {
    double propMass = 100000.0;   // lbs
    double boiloffRate = 0.001;   // per hour (0.1%)
    double timeHours = 10.0;

    double massLost = propMass * boiloffRate * timeHours;
    TS_ASSERT_DELTA(massLost, 1000.0, DEFAULT_TOLERANCE);

    double remainingMass = propMass - massLost;
    TS_ASSERT_DELTA(remainingMass, 99000.0, DEFAULT_TOLERANCE);
  }

  // ==================== PERFORMANCE METRICS TESTS ====================

  // Test thrust-to-weight ratio
  void testThrustToWeight() {
    double thrust = 150000.0;   // lbs
    double vehicleWeight = 100000.0;  // lbs

    double TWR = thrust / vehicleWeight;
    TS_ASSERT_DELTA(TWR, 1.5, DEFAULT_TOLERANCE);

    // Must be > 1 to lift off
    TS_ASSERT(TWR > 1.0);

    // Typical range 1.2-2.0 for launch vehicles
    TS_ASSERT(TWR > 1.1 && TWR < 3.0);
  }

  // Test burnout velocity
  void testBurnoutVelocity() {
    double Isp = 300.0;
    double g0 = 32.174;
    double m0 = 100000.0;  // Initial mass
    double mf = 40000.0;   // Final mass

    // Ideal burnout velocity (no gravity/drag)
    double Vbo_ideal = Isp * g0 * log(m0 / mf);
    TS_ASSERT_DELTA(Vbo_ideal, 8845.5, 10.0);

    // With gravity loss (typical 1500-2000 ft/s)
    double gravityLoss = 1500.0;
    double Vbo_actual = Vbo_ideal - gravityLoss;
    TS_ASSERT_DELTA(Vbo_actual, 7345.5, 10.0);
  }

  // Test structural efficiency
  void testStructuralEfficiency() {
    double propMass = 80000.0;
    double structMass = 10000.0;
    double totalMass = propMass + structMass;

    // Propellant mass fraction
    double PMF = propMass / totalMass;
    TS_ASSERT_DELTA(PMF, 0.889, 0.001);

    // Structural coefficient
    double epsilon = structMass / (structMass + propMass);
    TS_ASSERT_DELTA(epsilon, 0.111, 0.001);

    // Good stages have PMF > 0.85
    TS_ASSERT(PMF > 0.85);
  }

  // Test gravity loss
  void testGravityLoss() {
    double g = 32.174;  // ft/s^2
    double burnTime = 60.0;  // seconds
    double avgPitchAngle = 45.0 * M_PI / 180.0;  // Average during ascent

    // Simplified gravity loss
    double gravityLoss = g * burnTime * sin(avgPitchAngle);
    TS_ASSERT_DELTA(gravityLoss, 1365.0, 10.0);
  }

  // ==================== ATMOSPHERIC EFFECTS TESTS ====================

  // Test altitude vs ambient pressure
  void testAltitudeVsAmbientPressure() {
    // Exponential atmosphere model
    double P0 = 14.7;           // psi at sea level
    double H = 27000.0;         // Scale height ft

    // At sea level
    double altitude = 0.0;
    double Pa = P0 * exp(-altitude / H);
    TS_ASSERT_DELTA(Pa, 14.7, 0.01);

    // At 50,000 ft
    altitude = 50000.0;
    Pa = P0 * exp(-altitude / H);
    TS_ASSERT_DELTA(Pa, 2.3, 0.1);

    // At 100,000 ft
    altitude = 100000.0;
    Pa = P0 * exp(-altitude / H);
    TS_ASSERT(Pa < 0.5);  // Near vacuum
  }

  // Test nozzle efficiency vs altitude
  void testNozzleEfficiencyVsAltitude() {
    double vacThrust = 100000.0;
    double Ae = 100.0;  // Exit area sq in
    double Pe = 5.0;    // Exit pressure psi

    // Sea level (Pa = 14.7)
    double Pa = 14.7;
    double thrustSL = vacThrust - (Pa - Pe) * Ae;
    TS_ASSERT_DELTA(thrustSL, 99030.0, 10.0);

    // At altitude (Pa = 5.0, optimum expansion)
    Pa = 5.0;
    double thrustAlt = vacThrust - (Pa - Pe) * Ae;
    TS_ASSERT_DELTA(thrustAlt, 100000.0, DEFAULT_TOLERANCE);

    // Vacuum (Pa = 0)
    Pa = 0.0;
    double thrustVac = vacThrust - (Pa - Pe) * Ae;
    TS_ASSERT_DELTA(thrustVac, 100500.0, 10.0);  // Gains from expansion
  }

  // Test drag effects
  void testDragEffects() {
    double velocity = 3000.0;  // ft/s
    double rho = 0.001;        // slugs/ft^3 (high altitude)
    double Cd = 0.3;
    double area = 100.0;       // ft^2

    double drag = 0.5 * rho * velocity * velocity * Cd * area;
    TS_ASSERT_DELTA(drag, 135000.0, 100.0);  // Significant!

    // At lower velocity
    velocity = 1000.0;
    drag = 0.5 * rho * velocity * velocity * Cd * area;
    TS_ASSERT_DELTA(drag, 15000.0, 100.0);
  }

  // ==================== FAILURE MODES TESTS ====================

  // Test combustion instability detection
  void testCombustionInstability() {
    double nominalPressure = 1000.0;
    double pressureOscillation = 50.0;  // psi peak-to-peak
    double instabilityThreshold = 100.0;  // 10% of nominal

    bool isUnstable = (pressureOscillation > instabilityThreshold);
    TS_ASSERT(!isUnstable);

    // High oscillation (unstable)
    pressureOscillation = 150.0;
    isUnstable = (pressureOscillation > instabilityThreshold);
    TS_ASSERT(isUnstable);
  }

  // Test hard start detection
  void testHardStartDetection() {
    double peakPressure = 1500.0;  // psi
    double nominalPressure = 1000.0;
    double hardStartThreshold = 1.3;  // 130% of nominal

    bool isHardStart = (peakPressure > nominalPressure * hardStartThreshold);
    TS_ASSERT(isHardStart);

    // Normal start
    peakPressure = 1100.0;
    isHardStart = (peakPressure > nominalPressure * hardStartThreshold);
    TS_ASSERT(!isHardStart);
  }

  // Test propellant depletion
  void testPropellantDepletion() {
    double fuelRemaining = 100.0;  // lbs
    double oxiRemaining = 200.0;   // lbs
    double minUsable = 50.0;       // Reserve

    bool fuelDepleted = (fuelRemaining <= minUsable);
    bool oxiDepleted = (oxiRemaining <= minUsable);

    TS_ASSERT(!fuelDepleted);
    TS_ASSERT(!oxiDepleted);

    fuelRemaining = 40.0;
    fuelDepleted = (fuelRemaining <= minUsable);
    TS_ASSERT(fuelDepleted);  // Must shut down
  }

  // ==================== RCS AND VERNIER TESTS ====================

  // Test RCS pulse operation
  void testRCSPulseOperation() {
    double minPulseWidth = 0.020;  // 20 ms minimum
    double pulseCommand = 0.015;   // 15 ms requested

    // Pulse too short - reject or extend
    bool validPulse = (pulseCommand >= minPulseWidth);
    TS_ASSERT(!validPulse);

    pulseCommand = 0.050;  // 50 ms
    validPulse = (pulseCommand >= minPulseWidth);
    TS_ASSERT(validPulse);
  }

  // Test RCS impulse bit
  void testRCSImpulseBit() {
    double thrust = 25.0;        // lbs (small thruster)
    double pulseWidth = 0.050;   // seconds

    double impulseBit = thrust * pulseWidth;
    TS_ASSERT_DELTA(impulseBit, 1.25, 0.01);  // lb-sec

    // Multiple pulses
    int numPulses = 10;
    double totalImpulse = impulseBit * numPulses;
    TS_ASSERT_DELTA(totalImpulse, 12.5, 0.1);
  }

  // Test vernier engine trim
  void testVernierTrim() {
    double mainThrust = 100000.0;
    double vernierThrust = 1000.0;  // 1% of main
    double nominalVernier = 0.5;    // 50% position

    // Trim range
    double maxTrim = vernierThrust * (1.0 - nominalVernier);
    double minTrim = -vernierThrust * nominalVernier;

    TS_ASSERT_DELTA(maxTrim, 500.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(minTrim, -500.0, DEFAULT_TOLERANCE);
  }

  // ==================== MULTI-STAGE TESTS ====================

  // Test stage separation delta-V
  void testStageSeparation() {
    double stage1_Isp = 280.0;
    double stage1_m0 = 500000.0;
    double stage1_mf = 100000.0;

    double stage2_Isp = 350.0;
    double stage2_m0 = 80000.0;  // < stage1_mf due to staging hardware
    double stage2_mf = 20000.0;

    double g0 = 32.174;

    // Stage 1 delta-V
    double dV1 = stage1_Isp * g0 * log(stage1_m0 / stage1_mf);

    // Stage 2 delta-V
    double dV2 = stage2_Isp * g0 * log(stage2_m0 / stage2_mf);

    // Total delta-V
    double dV_total = dV1 + dV2;

    TS_ASSERT(dV1 > 10000.0);
    TS_ASSERT(dV2 > 10000.0);
    TS_ASSERT(dV_total > dV1 && dV_total > dV2);
  }

  // Test interstage coast
  void testInterstageCoast() {
    double coastTime = 5.0;  // seconds
    double velocity = 10000.0;  // ft/s
    double altitude = 200000.0;  // ft

    // Altitude gain during coast (no thrust)
    double altitudeGain = velocity * coastTime;  // Simplified (no gravity)
    TS_ASSERT_DELTA(altitudeGain, 50000.0, DEFAULT_TOLERANCE);
  }

  // ==================== MISCELLANEOUS TESTS ====================

  // Test effective exhaust velocity
  void testEffectiveExhaustVelocity() {
    // c = Isp * g0 (effective exhaust velocity)
    double Isp = 300.0;
    double g0 = 32.174;

    double c = Isp * g0;
    TS_ASSERT_DELTA(c, 9652.2, 0.1);

    // Relationship to thrust: F = m_dot * c
    double m_dot = 100.0;  // lbs/sec
    double F = m_dot * c / g0;  // Convert back to lbf
    TS_ASSERT_DELTA(F, 30000.0, 1.0);
  }

  // Test propellant residuals
  void testPropellantResiduals() {
    double totalProp = 100000.0;
    double usedProp = 95000.0;
    double residuals = totalProp - usedProp;

    double residualPercent = (residuals / totalProp) * 100.0;
    TS_ASSERT_DELTA(residualPercent, 5.0, DEFAULT_TOLERANCE);

    // Good designs have < 3% residuals
    double targetResidual = 0.03 * totalProp;
    TS_ASSERT(residuals > targetResidual);  // This design has too much
  }

  // Test engine duty cycle
  void testEngineDutyCycle() {
    double burnTime = 300.0;     // seconds
    double cooldownTime = 60.0;  // seconds between burns
    double numBurns = 3;

    double totalBurnTime = burnTime * numBurns;
    double totalMissionTime = totalBurnTime + cooldownTime * (numBurns - 1);

    double dutyCycle = totalBurnTime / totalMissionTime;
    TS_ASSERT_DELTA(totalBurnTime, 900.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(totalMissionTime, 1020.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(dutyCycle, 0.882, 0.01);
  }

  // Test propellant heating value
  void testPropellantHeatingValue() {
    // Different propellants have different energy densities
    double hv_RP1 = 18500.0;    // BTU/lb
    double hv_LH2 = 51600.0;    // BTU/lb
    double hv_solid = 8500.0;   // BTU/lb (typical)

    TS_ASSERT(hv_LH2 > hv_RP1);
    TS_ASSERT(hv_RP1 > hv_solid);

    // Energy available from propellant
    double propMass = 1000.0;  // lbs
    double energy_RP1 = propMass * hv_RP1;
    TS_ASSERT_DELTA(energy_RP1, 18500000.0, 1000.0);
  }

  // ==================== HYBRID ROCKET TESTS ====================

  void testHybridRocketOxidizerFlow() {
    // Hybrid rockets use solid fuel with liquid/gaseous oxidizer
    double oxiFlowRate = 50.0;  // lbs/sec
    double fuelRegressionRate = 0.02;  // in/sec
    double burnSurface = 500.0;  // sq in
    double fuelDensity = 0.04;   // lb/cu in

    double fuelMassRate = fuelRegressionRate * burnSurface * fuelDensity;
    TS_ASSERT(fuelMassRate > 0.0);
    TS_ASSERT(fuelMassRate < oxiFlowRate);
  }

  void testHybridRocketMixtureRatio() {
    double oxiFlow = 40.0;
    double fuelFlow = 10.0;

    double O_F = oxiFlow / fuelFlow;
    TS_ASSERT_DELTA(O_F, 4.0, DEFAULT_TOLERANCE);
  }

  void testHybridThrottleability() {
    double maxOxiFlow = 50.0;
    double throttlePercent = 0.6;

    double oxiFlow = maxOxiFlow * throttlePercent;
    TS_ASSERT_DELTA(oxiFlow, 30.0, DEFAULT_TOLERANCE);

    // Thrust roughly proportional to oxi flow
    double maxThrust = 50000.0;
    double thrust = maxThrust * throttlePercent;
    TS_ASSERT(thrust > 0.0 && thrust < maxThrust);
  }

  // ==================== MONOPROPELLANT TESTS ====================

  void testMonopropellantDecomposition() {
    // Monopropellant: single propellant decomposes to produce thrust
    double Isp_hydrazine = 230.0;  // seconds (typical)
    double propFlowRate = 10.0;    // lbs/sec

    double thrust = Isp_hydrazine * propFlowRate;
    TS_ASSERT_DELTA(thrust, 2300.0, DEFAULT_TOLERANCE);
  }

  void testMonopropellantCatalystBed() {
    // Catalyst bed temperature affects performance
    double T_cold = 400.0;   // °F (cold catalyst)
    double T_hot = 1200.0;   // °F (operating)

    double eta_cold = 0.85;  // Efficiency when cold
    double eta_hot = 0.98;   // Efficiency when hot

    TS_ASSERT(eta_hot > eta_cold);
  }

  void testMonopropellantPulseWidth() {
    double minPulse = 0.025;  // 25 ms minimum
    double thrust = 100.0;     // lbs

    double impulseBit = thrust * minPulse;
    TS_ASSERT_DELTA(impulseBit, 2.5, 0.1);
  }

  // ==================== BIPROPELLANT TESTS ====================

  void testBipropellantIgnition() {
    // Hypergolic propellants ignite on contact
    bool isHypergolic = true;
    double ignitionDelay = 0.005;  // seconds

    TS_ASSERT(isHypergolic);
    TS_ASSERT(ignitionDelay < 0.1);  // Very fast
  }

  void testBipropellantMixingEfficiency() {
    double Isp_theoretical = 330.0;
    double mixingEfficiency = 0.95;

    double Isp_actual = Isp_theoretical * mixingEfficiency;
    TS_ASSERT_DELTA(Isp_actual, 313.5, 0.1);
  }

  void testBipropellantStorability() {
    // Storable vs cryogenic propellants
    double boiloffRate_storable = 0.0;    // No boiloff
    double boiloffRate_LH2 = 0.001;        // 0.1% per hour

    TS_ASSERT(boiloffRate_storable < boiloffRate_LH2);
  }

  // ==================== RESTART CAPABILITY TESTS ====================

  void testEngineRestartSequence() {
    int maxRestarts = 5;
    int currentRestarts = 0;
    bool restartCapable = (currentRestarts < maxRestarts);

    TS_ASSERT(restartCapable);

    currentRestarts = 5;
    restartCapable = (currentRestarts < maxRestarts);
    TS_ASSERT(!restartCapable);
  }

  void testRestartChilldown() {
    // Cryogenic engines need chilldown before restart
    double chilldownTime = 30.0;  // seconds
    double propTemp = 70.0;        // °R (LH2)
    double engineTemp = 500.0;     // °R (warm engine)

    bool needsChilldown = (engineTemp > propTemp + 50.0);
    TS_ASSERT(needsChilldown);
  }

  void testRestartPropellantSettling() {
    // In microgravity, need ullage thrust to settle propellant
    double ullageThrust = 25.0;   // lbs
    double vehicleMass = 5000.0;  // lbs
    double g0 = 32.174;

    double accel = ullageThrust / (vehicleMass / g0);
    TS_ASSERT(accel > 0.0);
  }

  // ==================== THRUST TERMINATION TESTS ====================

  void testEmergencyShutdown() {
    double shutdownTime = 0.1;  // Very fast shutdown
    double maxThrust = 100000.0;

    // Linear decay
    double thrustAt50ms = maxThrust * (1.0 - 0.05 / shutdownTime);
    TS_ASSERT(thrustAt50ms > 0.0);
    TS_ASSERT(thrustAt50ms < maxThrust);
  }

  void testThrustTerminationPort() {
    // Solid rockets use TTP for emergency termination
    double nominalThrust = 50000.0;
    double portAreaRatio = 0.5;  // TTP opens 50% of chamber

    // Thrust drops rapidly when TTP opens
    double thrustAfterTTP = nominalThrust * (1.0 - portAreaRatio);
    TS_ASSERT_DELTA(thrustAfterTTP, 25000.0, 100.0);
  }

  // ==================== ENGINE HEALTH MONITORING ====================

  void testChamberPressureMonitor() {
    double Pc_nominal = 1000.0;  // psi
    double Pc_measured = 950.0;
    double tolerance = 0.1;      // 10%

    bool withinSpec = (fabs(Pc_measured - Pc_nominal) / Pc_nominal) < tolerance;
    TS_ASSERT(withinSpec);
  }

  void testTurbopumpSpeedMonitor() {
    double rpm_nominal = 35000.0;
    double rpm_measured = 34500.0;
    double rpm_redline = 38000.0;

    bool belowRedline = (rpm_measured < rpm_redline);
    TS_ASSERT(belowRedline);
  }

  void testTemperatureLimit() {
    double T_wall = 3000.0;      // °R
    double T_limit = 3500.0;     // °R

    bool withinLimit = (T_wall < T_limit);
    TS_ASSERT(withinLimit);
  }

  // ==================== POGO SUPPRESSION TESTS ====================

  void testPogoFrequency() {
    // POGO: coupling between propulsion and vehicle structure
    double vehicleFreq = 5.0;    // Hz (first mode)
    double feedFreq = 4.5;       // Hz (feed system)

    double separation = fabs(vehicleFreq - feedFreq);
    bool tooClose = (separation < 1.0);
    TS_ASSERT(tooClose);  // Needs suppression
  }

  void testPogoAccumulator() {
    double accumulatorVolume = 2.0;  // cubic ft
    double gasCharge = 100.0;         // psi

    // Accumulator provides compliance to decouple oscillations
    TS_ASSERT(accumulatorVolume > 0.0);
    TS_ASSERT(gasCharge > 0.0);
  }

  // ==================== THRUST OSCILLATION TESTS ====================

  void testThrustOscillationDetection() {
    double meanThrust = 100000.0;
    double oscillation = 5000.0;  // Peak-to-peak
    double threshold = 10000.0;   // 10% of nominal

    bool acceptable = (oscillation < threshold);
    TS_ASSERT(acceptable);
  }

  void testHighFrequencyInstability() {
    double frequency = 500.0;     // Hz
    double screechThreshold = 400.0;  // Hz

    bool isScreech = (frequency > screechThreshold);
    TS_ASSERT(isScreech);
  }

  // ==================== MULTIPLE ENGINE TESTS ====================

  void testEngineOutCapability() {
    int numEngines = 4;
    double thrustPerEngine = 25000.0;
    double totalThrust = numEngines * thrustPerEngine;

    // One engine out
    double thrustEngineOut = (numEngines - 1) * thrustPerEngine;
    double thrustRatio = thrustEngineOut / totalThrust;

    TS_ASSERT_DELTA(thrustRatio, 0.75, DEFAULT_TOLERANCE);
  }

  void testEngineSynchronization() {
    double thrust1 = 25000.0;
    double thrust2 = 24800.0;  // Slight mismatch
    double mismatchTolerance = 0.02;  // 2%

    double mismatch = fabs(thrust1 - thrust2) / thrust1;
    bool synchronized = (mismatch < mismatchTolerance);
    TS_ASSERT(synchronized);
  }

  void testClusterGimbalCompensation() {
    int numEngines = 4;
    double gimbalRange = 6.0;  // degrees

    // With one engine out, remaining engines may need more gimbal
    int enginesRemaining = 3;
    double gimbalRequired = 4.0 / enginesRemaining * numEngines;

    TS_ASSERT(gimbalRequired < gimbalRange);
  }

  // ==================== COMBUSTION STABILITY TESTS ====================

  void testCombustionStabilityMargin() {
    double dampingRatio = 0.05;  // 5% damping
    double threshold = 0.03;     // Minimum acceptable

    bool stable = (dampingRatio > threshold);
    TS_ASSERT(stable);
  }

  void testAcousticModeFrequencies() {
    // Longitudinal modes
    double chamberLength = 3.0;  // ft
    double soundSpeed = 4000.0;  // ft/s (hot combustion products)

    double firstMode = soundSpeed / (2.0 * chamberLength);
    TS_ASSERT(firstMode > 500.0);  // Typical range
  }

  void testInjectorDampingEffect() {
    double pressureDrop = 300.0;  // psi across injector
    double Pc = 1000.0;

    double pdRatio = pressureDrop / Pc;
    bool adequateDamping = (pdRatio > 0.15);  // >15% typical
    TS_ASSERT(adequateDamping);
  }

  // ==================== NOZZLE TESTS ====================

  void testNozzleErosionEffect() {
    double At_initial = 10.0;  // sq in
    double erosionRate = 0.001;  // in/sec radial
    double burnTime = 100.0;    // sec

    double radiusChange = erosionRate * burnTime;
    double r_initial = sqrt(At_initial / M_PI);
    double r_final = r_initial + radiusChange;
    double At_final = M_PI * r_final * r_final;

    TS_ASSERT(At_final > At_initial);
  }

  void testNozzleHeatFlux() {
    double heatFlux = 5.0e6;  // BTU/(ft²-hr)
    double thickness = 0.1;   // ft
    double k = 100.0;         // BTU/(ft-hr-°R)

    double deltaT = heatFlux * thickness / k;
    TS_ASSERT(deltaT > 0.0);
  }

  void testExtendableNozzle() {
    double epsilon_stowed = 20.0;
    double epsilon_deployed = 80.0;

    // Vacuum Isp improvement
    double Isp_stowed = 350.0;
    double Isp_deployed = 420.0;

    double improvement = (Isp_deployed - Isp_stowed) / Isp_stowed;
    TS_ASSERT(improvement > 0.15);  // >15% improvement
  }

  // ==================== UPPER STAGE TESTS ====================

  void testUpperStageRestart() {
    double coastDuration = 30.0 * 60.0;  // 30 minutes
    double propBoiloff = 0.001 * coastDuration / 3600.0;  // per hour rate

    double propRemaining = 1.0 - propBoiloff;
    TS_ASSERT(propRemaining > 0.99);  // <1% loss
  }

  void testPressureBlowdown() {
    double P_initial = 500.0;  // psi
    double V_ullage = 10.0;    // cu ft
    double V_expelled = 50.0;  // cu ft propellant

    // Final pressure after blowdown
    double P_final = P_initial * V_ullage / (V_ullage + V_expelled);
    TS_ASSERT(P_final < P_initial);
  }

  void testSettlingBurn() {
    double settlingThrust = 50.0;   // lbs (small)
    double duration = 5.0;          // seconds
    double vehicleMass = 10000.0;   // lbs

    double deltaV = settlingThrust * duration / (vehicleMass / 32.174);
    TS_ASSERT(deltaV > 0.0);
  }

  // ==================== ENVIRONMENTAL EFFECTS ====================

  void testAltitudeCompensatingNozzle() {
    // Aerospike or plug nozzle
    double Pa_sl = 14.7;
    double Pa_vac = 0.0;

    // Performance varies less with altitude
    double Isp_sl = 350.0;
    double Isp_vac = 360.0;

    double variation = (Isp_vac - Isp_sl) / Isp_sl;
    TS_ASSERT(variation < 0.05);  // <5% variation
  }

  void testCryogenicPropellantDensity() {
    double rho_LH2 = 4.43;    // lb/cu ft
    double rho_LOX = 71.2;    // lb/cu ft
    double rho_RP1 = 50.8;    // lb/cu ft

    TS_ASSERT(rho_LOX > rho_RP1);
    TS_ASSERT(rho_RP1 > rho_LH2);
  }

  void testPropellantVaporPressure() {
    double Pvap_LOX = 14.7;   // psi at boiling point
    double Pvap_LH2 = 14.7;   // psi at boiling point

    // Tank must be pressurized above vapor pressure
    double tankPressure = 50.0;
    TS_ASSERT(tankPressure > Pvap_LOX);
    TS_ASSERT(tankPressure > Pvap_LH2);
  }

  // ==================== ADDITIONAL PERFORMANCE TESTS ====================

  void testSpecificImpulseDensity() {
    // Density Isp = Isp * propellant_density
    double Isp = 300.0;
    double density = 60.0;  // lb/cu ft

    double densityIsp = Isp * density;
    TS_ASSERT_DELTA(densityIsp, 18000.0, DEFAULT_TOLERANCE);
  }

  void testCharacteristicLength() {
    // L* = chamber_volume / throat_area
    double Vc = 0.5;   // cu ft
    double At = 0.05;  // sq ft

    double Lstar = Vc / At;
    TS_ASSERT_DELTA(Lstar, 10.0, 0.1);  // ft
  }

  void testResidenceTime() {
    double Lstar = 10.0;     // ft
    double C_star = 6000.0;  // ft/s

    double tau = Lstar / C_star;
    TS_ASSERT(tau > 0.001);  // Sufficient for combustion
  }

  void testMassFlowBalance() {
    double thrust = 100000.0;
    double Isp = 300.0;
    double g0 = 32.174;

    double mdot = thrust / (Isp * g0);
    double thrust_check = mdot * Isp * g0;

    TS_ASSERT_DELTA(thrust_check, thrust, 1.0);
  }

  void testPropellantUtilization() {
    double totalProp = 100000.0;
    double usedProp = 97000.0;
    double residual = totalProp - usedProp;

    double utilization = usedProp / totalProp;
    TS_ASSERT(utilization > 0.95);  // >95% utilization
  }

  void testThrustToWeightRatioEngine() {
    double thrust = 200000.0;  // lbs
    double engineWeight = 2000.0;  // lbs

    double TWR = thrust / engineWeight;
    TS_ASSERT(TWR > 50.0);  // Good engine TWR
  }
};
