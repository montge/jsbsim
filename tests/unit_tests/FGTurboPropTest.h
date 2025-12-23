/*******************************************************************************
 * FGTurboPropTest.h - Unit tests for FGTurboProp (turboprop engine)
 *
 * Tests the mathematical behavior of turboprop engine model:
 * - N1 (gas generator speed) calculations
 * - Power output and fuel consumption
 * - ITT (Inter-Turbine Temperature) modeling
 * - Beta range and reverse thrust
 * - Engine phase transitions
 *
 * Note: FGTurboProp requires XML element for construction, so these tests
 * focus on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-8;
const double HP_TO_FTLBS = 550.0;  // 1 HP = 550 ft-lb/s

class FGTurboPropTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * N1 (Gas Generator Speed) Tests
   ***************************************************************************/

  // Exponential seek function (used for N1 spool-up/down)
  double expSeek(double current, double target, double accel, double decel, double dt) {
    double diff = target - current;
    double rate = (diff > 0) ? accel : decel;
    return current + diff * (1.0 - std::exp(-rate * dt));
  }

  // Linear seek function
  double linearSeek(double current, double target, double rate, double dt) {
    double diff = target - current;
    double maxChange = rate * dt;
    if (std::abs(diff) <= maxChange) {
      return target;
    }
    return current + (diff > 0 ? maxChange : -maxChange);
  }

  // Test N1 spool-up
  void testN1SpoolUp() {
    double N1 = 60.0;  // Current N1 (%)
    double targetN1 = 100.0;
    double accel = 2.0;  // Acceleration rate
    double decel = 1.0;
    double dt = 0.1;

    double newN1 = expSeek(N1, targetN1, accel, decel, dt);
    TS_ASSERT(newN1 > N1);
    TS_ASSERT(newN1 < targetN1);
  }

  // Test N1 spool-down
  void testN1SpoolDown() {
    double N1 = 100.0;
    double targetN1 = 60.0;
    double accel = 2.0;
    double decel = 1.0;
    double dt = 0.1;

    double newN1 = expSeek(N1, targetN1, accel, decel, dt);
    TS_ASSERT(newN1 < N1);
    TS_ASSERT(newN1 > targetN1);
  }

  // Test N1 convergence
  void testN1Convergence() {
    double N1 = 60.0;
    double targetN1 = 100.0;
    double accel = 2.0;
    double decel = 1.0;
    double dt = 0.1;

    for (int i = 0; i < 100; i++) {
      N1 = expSeek(N1, targetN1, accel, decel, dt);
    }

    TS_ASSERT_DELTA(N1, targetN1, 0.1);
  }

  // Test idle N1
  void testIdleN1() {
    double idleN1 = 60.0;  // Typical idle ~60%
    double throttle = 0.0;
    double maxN1 = 100.0;

    double targetN1 = idleN1 + (maxN1 - idleN1) * throttle;
    TS_ASSERT_DELTA(targetN1, idleN1, epsilon);
  }

  /***************************************************************************
   * Power Output Tests
   ***************************************************************************/

  // Test power from N1
  void testPowerFromN1() {
    double maxPower = 1000.0;  // HP
    double N1 = 100.0;         // %
    double N1_100 = 100.0;     // N1 at 100% power

    // Simplified: power proportional to N1 squared
    double power = maxPower * (N1 / N1_100) * (N1 / N1_100);
    TS_ASSERT_DELTA(power, 1000.0, epsilon);
  }

  // Test power at idle
  void testPowerAtIdle() {
    double maxPower = 1000.0;
    double N1 = 60.0;
    double N1_100 = 100.0;

    double power = maxPower * (N1 / N1_100) * (N1 / N1_100);
    TS_ASSERT_DELTA(power, 360.0, epsilon);  // 0.6^2 * 1000
  }

  // Test shaft horsepower to thrust conversion
  void testSHPToThrust() {
    double SHP = 500.0;  // Shaft horsepower
    double propEfficiency = 0.85;
    double velocity = 200.0;  // ft/s

    // Power = Thrust * Velocity
    // Thrust = Power / Velocity
    double powerFtLbS = SHP * HP_TO_FTLBS;
    double thrust = (powerFtLbS * propEfficiency) / velocity;

    TS_ASSERT_DELTA(thrust, 1168.75, 0.1);  // lbs
  }

  // Test static thrust (zero velocity)
  void testStaticThrust() {
    double SHP = 500.0;
    double propEfficiency = 0.85;

    // At zero velocity, use propeller disk loading formula
    // Simplified: static thrust ≈ K * sqrt(SHP)
    double K = 10.0;  // Empirical constant
    double staticThrust = K * std::sqrt(SHP);

    TS_ASSERT(staticThrust > 0);
  }

  /***************************************************************************
   * Fuel Consumption Tests
   ***************************************************************************/

  // Test power-specific fuel consumption
  void testPSFC() {
    double PSFC = 0.5;  // lb/(HP*hr) - typical turboprop
    double power = 500.0;  // HP
    double hours = 1.0;

    double fuelBurned = PSFC * power * hours;
    TS_ASSERT_DELTA(fuelBurned, 250.0, epsilon);  // lbs
  }

  // Test fuel flow rate
  void testFuelFlowRate() {
    double PSFC = 0.5;  // lb/(HP*hr)
    double power = 500.0;

    double fuelFlowPPH = PSFC * power;  // lb/hr
    double fuelFlowPPS = fuelFlowPPH / 3600.0;  // lb/s

    TS_ASSERT_DELTA(fuelFlowPPH, 250.0, epsilon);
    TS_ASSERT_DELTA(fuelFlowPPS, 0.0694, 0.001);
  }

  // Test fuel consumption with efficiency
  void testFuelConsumptionWithEfficiency() {
    double PSFC_best = 0.45;  // At optimal N1
    double efficiency = 0.9;  // 90% of optimal

    double actualPSFC = PSFC_best / efficiency;
    TS_ASSERT_DELTA(actualPSFC, 0.5, 0.01);
  }

  /***************************************************************************
   * ITT (Inter-Turbine Temperature) Tests
   ***************************************************************************/

  // Test ITT from N1
  void testITTFromN1() {
    // ITT increases with N1 (simplified linear model)
    double N1 = 80.0;  // %
    double ITT_idle = 400.0;  // °C at idle
    double ITT_max = 800.0;   // °C at max N1
    double N1_idle = 60.0;
    double N1_max = 100.0;

    double ITT = ITT_idle + (ITT_max - ITT_idle) * (N1 - N1_idle) / (N1_max - N1_idle);
    TS_ASSERT_DELTA(ITT, 600.0, epsilon);
  }

  // Test ITT lag response
  void testITTLag() {
    double ITT = 400.0;
    double targetITT = 700.0;
    double ITT_delay = 5.0;  // Time constant
    double dt = 0.1;

    double ca = std::exp(-dt / ITT_delay);
    double cb = 1.0 - ca;
    double newITT = ca * ITT + cb * targetITT;

    TS_ASSERT(newITT > ITT);
    TS_ASSERT(newITT < targetITT);
  }

  // Test ITT limits
  void testITTLimits() {
    double ITT = 850.0;  // Above limit
    double ITT_redline = 800.0;

    bool overtemp = (ITT > ITT_redline);
    TS_ASSERT(overtemp);
  }

  /***************************************************************************
   * Beta Range and Reverse Thrust Tests
   ***************************************************************************/

  // Test beta range detection
  void testBetaRangeDetection() {
    double betaRangeEnd = 0.1;  // 10% throttle

    double throttle1 = 0.05;  // In beta range
    double throttle2 = 0.5;   // Out of beta range

    bool inBeta1 = (throttle1 < betaRangeEnd);
    bool inBeta2 = (throttle2 < betaRangeEnd);

    TS_ASSERT(inBeta1);
    TS_ASSERT(!inBeta2);
  }

  // Test reverse thrust
  void testReverseThrust() {
    double normalThrust = 1000.0;
    double reverseMaxPower = 0.4;  // 40% max reverse

    double reverseThrust = -normalThrust * reverseMaxPower;
    TS_ASSERT_DELTA(reverseThrust, -400.0, epsilon);
  }

  // Test propeller pitch in beta range
  void testPropPitchBetaRange() {
    double throttle = 0.05;
    double minPitch = -15.0;  // Reverse pitch
    double zeroPitch = 0.0;
    double betaEnd = 0.1;

    // Linear interpolation in beta range
    double pitch = minPitch + (zeroPitch - minPitch) * (throttle / betaEnd);
    TS_ASSERT_DELTA(pitch, -7.5, epsilon);
  }

  /***************************************************************************
   * Engine Phase Tests
   ***************************************************************************/

  enum PhaseType { tpOff, tpRun, tpSpinUp, tpStart, tpTrim };

  // Test phase transition: Off to SpinUp
  void testPhaseOffToSpinUp() {
    PhaseType phase = tpOff;
    bool starterEngaged = true;

    if (phase == tpOff && starterEngaged) {
      phase = tpSpinUp;
    }

    TS_ASSERT_EQUALS(phase, tpSpinUp);
  }

  // Test phase transition: SpinUp to Start
  void testPhaseSpinUpToStart() {
    PhaseType phase = tpSpinUp;
    double N1 = 25.0;
    double starterN1 = 20.0;  // N1 where ignition can occur

    if (phase == tpSpinUp && N1 > starterN1) {
      phase = tpStart;
    }

    TS_ASSERT_EQUALS(phase, tpStart);
  }

  // Test phase transition: Start to Run
  void testPhaseStartToRun() {
    PhaseType phase = tpStart;
    double N1 = 62.0;
    double idleN1 = 60.0;

    if (phase == tpStart && N1 > idleN1) {
      phase = tpRun;
    }

    TS_ASSERT_EQUALS(phase, tpRun);
  }

  /***************************************************************************
   * IELU (Integrated Electronic Limiter Unit) Tests
   ***************************************************************************/

  // Test torque limiting
  void testTorqueLimiting() {
    double torque = 1200.0;  // lb-ft
    double maxTorque = 1000.0;

    bool ieluIntervent = (torque > maxTorque);
    TS_ASSERT(ieluIntervent);
  }

  // Test throttle reduction for torque limiting
  void testThrottleReductionForTorque() {
    double currentThrottle = 1.0;
    double torque = 1200.0;
    double maxTorque = 1000.0;

    // Reduce throttle proportionally
    double throttleReduction = (torque - maxTorque) / maxTorque;
    double newThrottle = currentThrottle * (1.0 - throttleReduction * 0.1);
    newThrottle = std::max(0.0, newThrottle);

    TS_ASSERT(newThrottle < currentThrottle);
  }

  /***************************************************************************
   * Oil System Tests
   ***************************************************************************/

  // Test oil pressure from N1
  void testOilPressure() {
    double N1 = 80.0;
    double oilPressureIdle = 30.0;  // psi
    double oilPressureMax = 90.0;
    double N1_idle = 60.0;
    double N1_max = 100.0;

    double oilPressure = oilPressureIdle +
      (oilPressureMax - oilPressureIdle) * (N1 - N1_idle) / (N1_max - N1_idle);

    TS_ASSERT_DELTA(oilPressure, 60.0, epsilon);
  }

  // Test oil temperature
  void testOilTemperature() {
    double ambientTemp = 288.15;  // K (15°C)
    double heatRise = 50.0;       // K at full power
    double N1 = 80.0;
    double N1_idle = 60.0;
    double N1_max = 100.0;

    double powerFraction = (N1 - N1_idle) / (N1_max - N1_idle);
    double oilTemp = ambientTemp + heatRise * powerFraction;

    TS_ASSERT_DELTA(oilTemp, 313.15, epsilon);  // 40°C
  }

  /***************************************************************************
   * Starting Time Tests
   ***************************************************************************/

  // Test maximum starting time check
  void testMaxStartingTime() {
    double startTime = 35.0;  // seconds
    double maxStartTime = 30.0;

    bool startFailed = (startTime > maxStartTime);
    TS_ASSERT(startFailed);
  }

  // Test successful start within time
  void testSuccessfulStart() {
    double startTime = 20.0;
    double maxStartTime = 30.0;
    double N1 = 62.0;
    double idleN1 = 60.0;

    bool started = (N1 > idleN1 && startTime < maxStartTime);
    TS_ASSERT(started);
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test Kelvin to Fahrenheit
  void testKelvinToFahrenheit() {
    double kelvin = 373.15;  // 100°C
    double fahrenheit = (kelvin - 273.15) * 9.0/5.0 + 32.0;

    TS_ASSERT_DELTA(fahrenheit, 212.0, 0.01);
  }

  // Test horsepower to ft-lb/s
  void testHPToFtLbS() {
    double hp = 1.0;
    double ftlbs = hp * HP_TO_FTLBS;

    TS_ASSERT_DELTA(ftlbs, 550.0, epsilon);
  }
};
