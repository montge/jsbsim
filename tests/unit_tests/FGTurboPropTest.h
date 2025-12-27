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

  /***************************************************************************
   * Altitude Effects Tests
   ***************************************************************************/

  // Test power lapse with altitude
  void testPowerLapseWithAltitude() {
    double seaLevelPower = 1000.0;  // HP
    double altitude = 10000.0;      // ft
    double lapseRate = 0.035;       // ~3.5% per 1000 ft

    double availablePower = seaLevelPower * (1.0 - lapseRate * altitude / 1000.0);
    TS_ASSERT_DELTA(availablePower, 650.0, 0.1);
  }

  // Test density altitude effect
  void testDensityAltitudeEffect() {
    double pressureAltitude = 5000.0;
    double tempDeviation = 20.0;  // °C above ISA

    // Density altitude ≈ PA + 120 * temp deviation
    double densityAltitude = pressureAltitude + 120.0 * tempDeviation;
    TS_ASSERT_DELTA(densityAltitude, 7400.0, 0.1);
  }

  // Test ram air recovery
  void testRamAirRecovery() {
    double staticPressure = 2116.0;  // lb/ft^2 (sea level)
    double mach = 0.4;
    double gamma = 1.4;

    // Ram pressure ratio = (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1))
    double term = 1.0 + (gamma - 1.0) / 2.0 * mach * mach;
    double ramRatio = std::pow(term, gamma / (gamma - 1.0));

    TS_ASSERT(ramRatio > 1.0);
    TS_ASSERT(ramRatio < 1.2);
  }

  // Test inlet temperature rise
  void testInletTemperatureRise() {
    double ambientTemp = 288.15;  // K (ISA sea level)
    double mach = 0.4;
    double gamma = 1.4;

    // T_total / T_static = 1 + (gamma-1)/2 * M^2
    double tempRatio = 1.0 + (gamma - 1.0) / 2.0 * mach * mach;
    double totalTemp = ambientTemp * tempRatio;

    TS_ASSERT(totalTemp > ambientTemp);
    TS_ASSERT_DELTA(totalTemp, 297.4, 0.5);
  }

  /***************************************************************************
   * Propeller Governor Tests
   ***************************************************************************/

  // Test prop governor RPM control
  void testPropGovernorRPM() {
    double targetRPM = 1700.0;
    double actualRPM = 1680.0;
    double Kp = 0.1;

    double pitchAdjust = Kp * (targetRPM - actualRPM);
    // Positive adjustment = increase pitch to load prop
    TS_ASSERT(pitchAdjust > 0.0);
  }

  // Test prop governor at overspeed
  void testPropGovernorOverspeed() {
    double targetRPM = 1700.0;
    double actualRPM = 1750.0;
    double Kp = 0.1;

    double pitchAdjust = Kp * (targetRPM - actualRPM);
    // Negative adjustment = reduce pitch
    TS_ASSERT(pitchAdjust < 0.0);
  }

  // Test prop feather position
  void testPropFeatherPosition() {
    double featherPitch = 85.0;  // degrees
    double normalMaxPitch = 35.0;

    TS_ASSERT(featherPitch > normalMaxPitch);
    TS_ASSERT(featherPitch < 90.0);
  }

  // Test prop RPM overspeed protection
  void testRPMOverspeedProtection() {
    double propRPM = 1850.0;
    double maxRPM = 1800.0;
    double overspeedLimit = 1.03;  // 103% of max

    bool overSpeed = (propRPM > maxRPM * overspeedLimit);
    TS_ASSERT(!overSpeed);

    propRPM = 1900.0;
    overSpeed = (propRPM > maxRPM * overspeedLimit);
    TS_ASSERT(overSpeed);
  }

  /***************************************************************************
   * N2 (Power Turbine) Tests
   ***************************************************************************/

  // Test N2 from torque demand
  void testN2FromTorqueDemand() {
    double N2_idle = 95.0;  // %
    double N2_max = 100.0;
    double torqueDemand = 0.7;  // 70% of max

    double N2 = N2_idle + (N2_max - N2_idle) * torqueDemand;
    TS_ASSERT_DELTA(N2, 98.5, 0.1);
  }

  // Test N2 droop with load
  void testN2DroopWithLoad() {
    double N2_noLoad = 100.0;
    double droopRate = 4.0;  // % droop at full load
    double loadFraction = 0.8;

    double N2 = N2_noLoad - droopRate * loadFraction;
    TS_ASSERT_DELTA(N2, 96.8, 0.1);
  }

  /***************************************************************************
   * Compressor Tests
   ***************************************************************************/

  // Test compressor pressure ratio
  void testCompressorPressureRatio() {
    double N1 = 90.0;  // %
    double maxPressureRatio = 10.0;
    double idleN1 = 60.0;
    double maxN1 = 100.0;

    // Simplified: PR increases with N1 squared
    double N1_frac = (N1 - idleN1) / (maxN1 - idleN1);
    double pressureRatio = 1.0 + (maxPressureRatio - 1.0) * N1_frac * N1_frac;

    TS_ASSERT(pressureRatio > 1.0);
    TS_ASSERT(pressureRatio < maxPressureRatio);
  }

  // Test compressor stall margin
  void testCompressorStallMargin() {
    double operatingPR = 8.0;  // Pressure ratio
    double stallPR = 10.0;     // PR at stall line

    double stallMargin = (stallPR - operatingPR) / operatingPR * 100.0;
    TS_ASSERT_DELTA(stallMargin, 25.0, 0.1);  // 25% margin
  }

  // Test bleed air extraction effect
  void testBleedAirExtractionEffect() {
    double basePower = 1000.0;
    double bleedFraction = 0.05;  // 5% bleed
    double bleedPowerLoss = 0.02; // 2% power loss per 1% bleed

    double powerWithBleed = basePower * (1.0 - bleedFraction * bleedPowerLoss / 0.01);
    TS_ASSERT(powerWithBleed < basePower);
    TS_ASSERT_DELTA(powerWithBleed, 900.0, 0.1);
  }

  /***************************************************************************
   * Condition Lever Tests
   ***************************************************************************/

  // Test condition lever fuel cutoff
  void testConditionLeverCutoff() {
    double conditionLever = 0.0;  // Cutoff position
    double cutoffThreshold = 0.1;

    bool fuelCutoff = (conditionLever < cutoffThreshold);
    TS_ASSERT(fuelCutoff);
  }

  // Test condition lever low idle
  void testConditionLeverLowIdle() {
    double conditionLever = 0.3;
    double lowIdlePosition = 0.25;
    double highIdlePosition = 0.75;

    bool atLowIdle = (conditionLever >= lowIdlePosition && conditionLever < highIdlePosition);
    TS_ASSERT(atLowIdle);
  }

  // Test condition lever high idle
  void testConditionLeverHighIdle() {
    double conditionLever = 0.8;
    double highIdlePosition = 0.75;

    bool atHighIdle = (conditionLever >= highIdlePosition);
    TS_ASSERT(atHighIdle);
  }

  /***************************************************************************
   * Autofeather System Tests
   ***************************************************************************/

  // Test autofeather arm condition
  void testAutofeatherArm() {
    double torque = 800.0;
    double armThreshold = 500.0;
    bool autofeatherSwitch = true;

    bool armed = autofeatherSwitch && (torque > armThreshold);
    TS_ASSERT(armed);
  }

  // Test autofeather trigger
  void testAutofeatherTrigger() {
    double torque = 100.0;  // Low torque (engine failure)
    double triggerThreshold = 200.0;
    bool armed = true;

    bool featherTriggered = armed && (torque < triggerThreshold);
    TS_ASSERT(featherTriggered);
  }

  /***************************************************************************
   * Start Anomaly Tests
   ***************************************************************************/

  // Test hot start detection
  void testHotStartDetection() {
    double ITT = 900.0;          // °C
    double ITT_startLimit = 800.0;
    double N1 = 40.0;            // Still spooling
    double N1_idle = 60.0;

    bool hotStart = (ITT > ITT_startLimit) && (N1 < N1_idle);
    TS_ASSERT(hotStart);
  }

  // Test hung start detection
  void testHungStartDetection() {
    double N1 = 35.0;            // Stuck below idle
    double N1_threshold = 40.0;  // Should be above this by now
    double startTime = 40.0;     // seconds
    double maxStartTime = 30.0;

    bool hungStart = (N1 < N1_threshold) && (startTime > maxStartTime);
    TS_ASSERT(hungStart);
  }

  // Test abort start conditions
  void testAbortStartConditions() {
    double ITT = 750.0;
    double ITT_limit = 800.0;
    double N1 = 45.0;
    double N1_expected = 50.0;
    double startTime = 25.0;
    double timeout = 30.0;

    // Abort if ITT too high or N1 too low for time elapsed
    bool shouldAbort = (ITT > ITT_limit * 0.95) ||
                       (startTime > timeout * 0.8 && N1 < N1_expected);
    TS_ASSERT(shouldAbort);
  }

  /***************************************************************************
   * Torque System Tests
   ***************************************************************************/

  // Test torque from power and RPM
  void testTorqueFromPowerAndRPM() {
    double power = 500.0;  // HP
    double propRPM = 1700.0;

    // Torque = Power * 5252 / RPM (ft-lbf)
    double torque = power * 5252.0 / propRPM;
    TS_ASSERT_DELTA(torque, 1544.7, 0.1);
  }

  // Test negative torque sensing
  void testNegativeTorqueSensing() {
    double torque = -50.0;  // Negative (windmilling)
    double NTS_threshold = 0.0;

    bool negativeTorque = (torque < NTS_threshold);
    TS_ASSERT(negativeTorque);
  }

  // Test torque meter scaling
  void testTorqueMeterScaling() {
    double actualTorque = 1500.0;  // ft-lbf
    double maxTorque = 2000.0;

    double torquePercent = (actualTorque / maxTorque) * 100.0;
    TS_ASSERT_DELTA(torquePercent, 75.0, epsilon);
  }

  /***************************************************************************
   * Engine Windmilling Tests
   ***************************************************************************/

  // Test windmill N1 from airspeed
  void testWindmillN1() {
    double airspeed = 150.0;  // knots
    double windmillFactor = 0.2;  // N1% per knot

    double windmillN1 = airspeed * windmillFactor;
    TS_ASSERT_DELTA(windmillN1, 30.0, 0.1);
  }

  // Test windmill restart capability
  void testWindmillRestartCapability() {
    double windmillN1 = 25.0;
    double minRestartN1 = 20.0;

    bool canRestart = (windmillN1 >= minRestartN1);
    TS_ASSERT(canRestart);
  }

  /***************************************************************************
   * Fuel System Tests
   ***************************************************************************/

  // Test fuel metering unit output
  void testFuelMeteringUnit() {
    double N1_demand = 80.0;
    double baseFlow = 100.0;    // PPH at idle
    double maxFlow = 500.0;     // PPH at max
    double N1_idle = 60.0;
    double N1_max = 100.0;

    double flowFraction = (N1_demand - N1_idle) / (N1_max - N1_idle);
    double fuelFlow = baseFlow + (maxFlow - baseFlow) * flowFraction;

    TS_ASSERT(fuelFlow > baseFlow);
    TS_ASSERT(fuelFlow < maxFlow);
  }

  // Test fuel temperature effect on flow
  void testFuelTemperatureEffect() {
    double baseFlow = 300.0;  // PPH
    double fuelTemp = 40.0;   // °C
    double refTemp = 15.0;    // °C reference
    double tempCoeff = 0.001; // per °C

    // Warmer fuel = less dense = slightly higher volume flow
    double correctedFlow = baseFlow * (1.0 + tempCoeff * (fuelTemp - refTemp));
    TS_ASSERT(correctedFlow > baseFlow);
  }

  /***************************************************************************
   * Temperature Compensation Tests
   ***************************************************************************/

  // Test N1 temperature correction
  void testN1TempCorrection() {
    double N1_actual = 95.0;
    double ambientTemp = 303.15;  // K (30°C)
    double refTemp = 288.15;      // K (15°C ISA)

    // Corrected N1 = actual * sqrt(refTemp / ambientTemp)
    double N1_corrected = N1_actual * std::sqrt(refTemp / ambientTemp);
    TS_ASSERT(N1_corrected < N1_actual);
  }

  // Test ITT temperature correction
  void testITTTempCorrection() {
    double ITT_actual = 700.0;    // °C
    double ambientTemp = 303.15;  // K
    double refTemp = 288.15;      // K

    double ITT_corrected = ITT_actual * (refTemp / ambientTemp);
    TS_ASSERT(ITT_corrected < ITT_actual);
  }

  /***************************************************************************
   * Engine Thermal Cycling Tests
   ***************************************************************************/

  // Test thermal soak time
  void testThermalSoakTime() {
    double ITT_current = 200.0;  // °C (cooling down)
    double ITT_ambient = 25.0;   // °C
    double coolingRate = 0.5;    // °C/sec

    double timeToSoak = (ITT_current - ITT_ambient) / coolingRate;
    TS_ASSERT_DELTA(timeToSoak, 350.0, 0.1);  // ~6 minutes
  }

  // Test hot section life factor
  void testHotSectionLifeFactor() {
    double ITT_operating = 750.0;
    double ITT_design = 700.0;

    // Life decreases exponentially with temperature above design
    double overTemp = ITT_operating - ITT_design;
    double lifeFactor = std::exp(-overTemp / 100.0);

    TS_ASSERT(lifeFactor < 1.0);
    TS_ASSERT(lifeFactor > 0.5);
  }

  /***************************************************************************
   * Starter System Tests
   ***************************************************************************/

  // Test starter torque vs N1
  void testStarterTorqueVsN1() {
    double N1 = 15.0;
    double starterMaxTorque = 100.0;  // ft-lbf
    double cutoffN1 = 50.0;

    // Starter torque decreases as N1 increases
    double starterTorque = starterMaxTorque * (1.0 - N1 / cutoffN1);
    TS_ASSERT(starterTorque > 0.0);
    TS_ASSERT(starterTorque < starterMaxTorque);
  }

  // Test starter duty cycle
  void testStarterDutyCycle() {
    double cranckTime = 40.0;      // seconds
    double maxCrankTime = 30.0;
    double cooldownTime = 60.0;    // required rest

    bool needsCooldown = (cranckTime > maxCrankTime);
    TS_ASSERT(needsCooldown);
  }

  // Test ignition timing
  void testIgnitionTiming() {
    double N1 = 22.0;
    double ignitionOnN1 = 15.0;
    double ignitionOffN1 = 50.0;

    bool ignitionActive = (N1 >= ignitionOnN1) && (N1 <= ignitionOffN1);
    TS_ASSERT(ignitionActive);
  }

  /***************************************************************************
   * Performance Calculation Tests
   ***************************************************************************/

  // Test equivalent shaft horsepower
  void testEquivalentSHP() {
    double SHP = 800.0;        // Shaft HP
    double jetThrust = 100.0;  // lb residual jet thrust
    double velocity = 200.0;   // ft/s

    // ESHP = SHP + (thrust * velocity / 550)
    double ESHP = SHP + (jetThrust * velocity / HP_TO_FTLBS);
    TS_ASSERT_DELTA(ESHP, 836.4, 0.1);
  }

  // Test propulsive efficiency
  void testPropulsiveEfficiency() {
    double thrust = 1000.0;     // lb
    double velocity = 300.0;    // ft/s
    double fuelFlow = 0.1;      // lb/s
    double fuelEnergy = 18500.0; // BTU/lb

    double propPower = thrust * velocity;             // ft-lb/s
    double fuelPower = fuelFlow * fuelEnergy * 778.0; // ft-lb/s

    double efficiency = propPower / fuelPower;
    TS_ASSERT(efficiency > 0.0);
    TS_ASSERT(efficiency < 0.5);  // Typical max ~40%
  }

  // Test specific range
  void testSpecificRange() {
    double trueAirspeed = 250.0;  // knots
    double fuelFlow = 300.0;      // PPH

    double specificRange = trueAirspeed / fuelFlow;  // nm/lb
    TS_ASSERT_DELTA(specificRange, 0.833, 0.001);
  }

  /***************************************************************************
   * Extended N1 Dynamics Tests
   ***************************************************************************/

  // Test N1 acceleration rate
  void testN1AccelerationRate() {
    double N1 = 60.0;
    double targetN1 = 100.0;
    double accel = 3.0;  // Faster acceleration
    double decel = 1.0;
    double dt = 0.1;

    double newN1 = expSeek(N1, targetN1, accel, decel, dt);
    double rate = (newN1 - N1) / dt;

    TS_ASSERT(rate > 0.0);
    TS_ASSERT(rate < (targetN1 - N1) / dt);  // Rate limited
  }

  // Test N1 deceleration rate
  void testN1DecelerationRate() {
    double N1 = 100.0;
    double targetN1 = 60.0;
    double accel = 2.0;
    double decel = 0.5;  // Slower deceleration
    double dt = 0.1;

    double newN1 = expSeek(N1, targetN1, accel, decel, dt);
    double rate = (N1 - newN1) / dt;

    TS_ASSERT(rate > 0.0);
    TS_ASSERT(rate < (N1 - targetN1) / dt);
  }

  // Test N1 overshoot prevention
  void testN1OvershootPrevention() {
    double N1 = 99.0;
    double targetN1 = 100.0;
    double accel = 5.0;
    double decel = 1.0;
    double dt = 1.0;  // Large time step

    double newN1 = expSeek(N1, targetN1, accel, decel, dt);
    TS_ASSERT(newN1 <= targetN1);  // No overshoot
  }

  // Test N1 undershoot prevention
  void testN1UndershootPrevention() {
    double N1 = 61.0;
    double targetN1 = 60.0;
    double accel = 2.0;
    double decel = 5.0;
    double dt = 1.0;

    double newN1 = expSeek(N1, targetN1, accel, decel, dt);
    TS_ASSERT(newN1 >= targetN1);  // No undershoot
  }

  /***************************************************************************
   * Extended Power Calculation Tests
   ***************************************************************************/

  // Test power curve nonlinearity
  void testPowerCurveNonlinearity() {
    double maxPower = 1000.0;
    double N1_100 = 100.0;

    // Power at 50% N1
    double N1_50 = 50.0;
    double power50 = maxPower * (N1_50 / N1_100) * (N1_50 / N1_100);

    // Power at 70% N1
    double N1_70 = 70.0;
    double power70 = maxPower * (N1_70 / N1_100) * (N1_70 / N1_100);

    TS_ASSERT_DELTA(power50, 250.0, epsilon);
    TS_ASSERT_DELTA(power70, 490.0, epsilon);

    // Verify non-linear relationship
    TS_ASSERT(power70 / power50 > 70.0 / 50.0);
  }

  // Test power available at altitude
  void testPowerAvailableAtAltitude() {
    double seaLevelPower = 1000.0;
    double altitudes[] = {0.0, 5000.0, 10000.0, 15000.0, 20000.0};
    double lapseRate = 0.035;

    double prevPower = seaLevelPower;
    for (double alt : altitudes) {
      double power = seaLevelPower * (1.0 - lapseRate * alt / 1000.0);
      TS_ASSERT(power <= prevPower);
      prevPower = power;
    }
  }

  // Test residual jet thrust contribution
  void testResidualJetThrustContribution() {
    double SHP = 800.0;
    double jetThrust[] = {0.0, 50.0, 100.0, 150.0};
    double velocity = 200.0;

    double baseESHP = SHP;
    for (double jet : jetThrust) {
      double ESHP = SHP + (jet * velocity / HP_TO_FTLBS);
      TS_ASSERT(ESHP >= baseESHP);
      baseESHP = ESHP;
    }
  }

  /***************************************************************************
   * Extended ITT Modeling Tests
   ***************************************************************************/

  // Test ITT rate of change
  void testITTRateOfChange() {
    double ITT = 400.0;
    double targetITT = 700.0;
    double ITT_delay = 5.0;
    double dt = 0.1;

    double ca = std::exp(-dt / ITT_delay);
    double cb = 1.0 - ca;

    // Track ITT over multiple timesteps (500 iterations = 50 seconds, ~10 time constants)
    for (int i = 0; i < 500; i++) {
      double prevITT = ITT;
      ITT = ca * ITT + cb * targetITT;
      double rate = (ITT - prevITT) / dt;
      TS_ASSERT(rate >= 0.0);  // Always warming
    }
    TS_ASSERT_DELTA(ITT, targetITT, 1.0);
  }

  // Test ITT cooling
  void testITTCooling() {
    double ITT = 700.0;
    double targetITT = 400.0;
    double ITT_delay = 10.0;  // Slower cooling
    double dt = 0.1;

    double ca = std::exp(-dt / ITT_delay);
    double cb = 1.0 - ca;

    for (int i = 0; i < 1000; i++) {  // 100 seconds = 10 time constants
      ITT = ca * ITT + cb * targetITT;
    }
    TS_ASSERT_DELTA(ITT, targetITT, 1.0);
  }

  // Test ITT exceedance margins
  void testITTExceedanceMargins() {
    double ITT_redline = 800.0;
    double ITT_cautionZone = ITT_redline * 0.95;  // 95% is caution = 760
    double ITT_normalMax = ITT_redline * 0.90;    // 90% is normal max = 720

    double ITT_test1 = 700.0;   // Below normal max
    double ITT_test2 = 770.0;   // Above normal max, below redline
    double ITT_test3 = 810.0;   // Above redline

    TS_ASSERT(ITT_test1 < ITT_normalMax);      // Normal
    TS_ASSERT(ITT_test2 >= ITT_normalMax);     // In caution zone
    TS_ASSERT(ITT_test2 < ITT_redline);
    TS_ASSERT(ITT_test3 > ITT_redline);        // Exceedance
  }

  /***************************************************************************
   * Extended Propeller Tests
   ***************************************************************************/

  // Test propeller efficiency curve
  void testPropellerEfficiencyCurve() {
    // Propeller efficiency varies with advance ratio
    double propDiam = 10.0;  // feet
    double propRPM = 1700.0;
    double velocities[] = {0.0, 100.0, 200.0, 300.0};

    for (double V : velocities) {
      double n = propRPM / 60.0;  // rev/s
      double J = V / (n * propDiam);  // Advance ratio
      TS_ASSERT(J >= 0.0);
      TS_ASSERT(!std::isnan(J));
    }
  }

  // Test propeller pitch range
  void testPropellerPitchRange() {
    double featherPitch = 85.0;
    double maxPitch = 35.0;
    double finePitch = 15.0;
    double reversePitch = -15.0;

    TS_ASSERT(reversePitch < 0.0);
    TS_ASSERT(finePitch < maxPitch);
    TS_ASSERT(maxPitch < featherPitch);
    TS_ASSERT(featherPitch < 90.0);
  }

  // Test blade angle response
  void testBladeAngleResponse() {
    double currentPitch = 20.0;
    double targetPitch = 30.0;
    double rate = 5.0;  // deg/s
    double dt = 0.1;

    double newPitch = linearSeek(currentPitch, targetPitch, rate, dt);
    TS_ASSERT_DELTA(newPitch, currentPitch + rate * dt, epsilon);
    TS_ASSERT(newPitch < targetPitch);
  }

  /***************************************************************************
   * Extended Fuel System Tests
   ***************************************************************************/

  // Test fuel consumption at different power settings
  void testFuelConsumptionAtDifferentPower() {
    double PSFC = 0.5;
    double powers[] = {100.0, 250.0, 500.0, 750.0, 1000.0};

    for (double power : powers) {
      double fuelFlow = PSFC * power;
      TS_ASSERT_DELTA(fuelFlow, power * 0.5, epsilon);
    }
  }

  // Test fuel remaining calculation
  void testFuelRemainingCalculation() {
    double fuelCapacity = 1000.0;  // lbs
    double fuelBurned = 0.0;
    double fuelFlow = 300.0;  // PPH
    double flightTime = 2.0;  // hours

    fuelBurned = fuelFlow * flightTime;
    double fuelRemaining = fuelCapacity - fuelBurned;

    TS_ASSERT_DELTA(fuelRemaining, 400.0, epsilon);
  }

  // Test fuel flow limiting
  void testFuelFlowLimiting() {
    double maxFuelFlow = 500.0;  // PPH
    double demandedFlow = 600.0;

    double actualFlow = std::min(demandedFlow, maxFuelFlow);
    TS_ASSERT_DELTA(actualFlow, maxFuelFlow, epsilon);
  }

  /***************************************************************************
   * Extended Starting System Tests
   ***************************************************************************/

  // Test starter motor torque curve
  void testStarterMotorTorqueCurve() {
    double maxTorque = 100.0;
    double cutoffN1 = 50.0;
    double N1_values[] = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};

    double prevTorque = maxTorque;
    for (double N1 : N1_values) {
      double torque = maxTorque * (1.0 - N1 / cutoffN1);
      TS_ASSERT(torque <= prevTorque);
      TS_ASSERT(torque >= 0.0);
      prevTorque = torque;
    }
  }

  // Test ignition window
  void testIgnitionWindow() {
    double ignitionOnN1 = 15.0;
    double ignitionOffN1 = 50.0;

    double N1_values[] = {10.0, 15.0, 30.0, 50.0, 60.0};
    bool expected[] = {false, true, true, true, false};

    for (size_t i = 0; i < 5; i++) {
      bool ignition = (N1_values[i] >= ignitionOnN1) && (N1_values[i] <= ignitionOffN1);
      TS_ASSERT_EQUALS(ignition, expected[i]);
    }
  }

  // Test light-off sequence
  void testLightOffSequence() {
    double N1 = 20.0;
    double fuelFlow = 100.0;  // Present
    double ignitionActive = true;
    double combustorTemp = 300.0;  // K
    double minLightOffTemp = 250.0;

    bool lightOff = ignitionActive && (fuelFlow > 0) && (combustorTemp > minLightOffTemp);
    TS_ASSERT(lightOff);
  }

  /***************************************************************************
   * Extended Torque System Tests
   ***************************************************************************/

  // Test torque limiting with exceedance
  void testTorqueLimitingWithExceedance() {
    double maxTorque = 2000.0;
    double measuredTorque = 2200.0;
    double limitReduction = 0.1;  // 10% reduction per exceedance

    bool limited = (measuredTorque > maxTorque);
    TS_ASSERT(limited);

    double fuelReduction = (measuredTorque - maxTorque) / maxTorque * limitReduction;
    TS_ASSERT(fuelReduction > 0.0);
  }

  // Test torque indication accuracy
  void testTorqueIndicationAccuracy() {
    double power = 500.0;  // HP
    double RPM = 1700.0;

    double torque = power * 5252.0 / RPM;
    double indicatedTorque = torque * 1.02;  // 2% overboosted indicator

    TS_ASSERT(indicatedTorque > torque);
    TS_ASSERT_DELTA(indicatedTorque / torque, 1.02, 0.001);
  }

  // Test negative torque protection
  void testNegativeTorqueProtection() {
    double torque = -100.0;
    double NTS_threshold = 0.0;

    bool ntsActive = (torque < NTS_threshold);
    TS_ASSERT(ntsActive);

    // Protection should increase blade angle
    double pitchAdjust = ntsActive ? 5.0 : 0.0;
    TS_ASSERT(pitchAdjust > 0.0);
  }

  /***************************************************************************
   * Extended Environmental Effect Tests
   ***************************************************************************/

  // Test cold weather starting
  void testColdWeatherStarting() {
    double ambientTemp = 250.0;  // K (-23°C)
    double refTemp = 288.15;     // K (15°C)
    double coldStartPenalty = (refTemp - ambientTemp) / 50.0;  // Extra seconds

    double normalStartTime = 20.0;
    double coldStartTime = normalStartTime + coldStartPenalty;

    TS_ASSERT(coldStartTime > normalStartTime);
    TS_ASSERT_DELTA(coldStartPenalty, 0.763, 0.01);
  }

  // Test hot weather power loss
  void testHotWeatherPowerLoss() {
    double seaLevelPower = 1000.0;
    double ambientTemp = 40.0;  // °C
    double refTemp = 15.0;      // °C ISA
    double tempDerateRate = 0.01;  // 1% per °C above ISA

    double derate = (ambientTemp - refTemp) * tempDerateRate;
    double availablePower = seaLevelPower * (1.0 - derate);

    TS_ASSERT(availablePower < seaLevelPower);
    TS_ASSERT_DELTA(availablePower, 750.0, epsilon);
  }

  // Test icing effect on power
  void testIcingEffectOnPower() {
    double normalPower = 1000.0;
    double icingLoss = 0.05;  // 5% power loss with inlet icing

    double icedPower = normalPower * (1.0 - icingLoss);
    TS_ASSERT_DELTA(icedPower, 950.0, epsilon);
  }

  /***************************************************************************
   * Extended Autofeather and Safety Tests
   ***************************************************************************/

  // Test autofeather conditions
  void testAutofeatherConditions() {
    double torque = 100.0;
    double armThreshold = 500.0;
    double triggerThreshold = 200.0;
    bool autofeatherSwitch = true;
    bool inFlight = true;

    bool armed = autofeatherSwitch && inFlight;
    bool triggered = armed && (torque < triggerThreshold);

    TS_ASSERT(armed);
    TS_ASSERT(triggered);
  }

  // Test autofeather time delay
  void testAutofeatherTimeDelay() {
    double triggerTime = 0.0;
    double featherDelay = 1.5;  // seconds
    double currentTime = 0.0;
    double simTime = 0.0;
    bool triggered = false;

    // Simulate trigger at t=0.5
    for (int i = 0; i < 30; i++) {
      simTime = i * 0.1;
      if (simTime >= 0.5 && !triggered) {
        triggered = true;
        triggerTime = simTime;
      }

      if (triggered && (simTime - triggerTime) >= featherDelay) {
        bool featherCommanded = true;
        TS_ASSERT(featherCommanded);
        break;
      }
    }
  }

  // Test emergency shutdown sequence
  void testEmergencyShutdownSequence() {
    double N1 = 90.0;
    double fuelCutoff = true;
    double decelRate = 20.0;  // N1%/sec (rapid decel rate for emergency)
    double dt = 0.1;

    // 200 iterations at 0.1s = 20 seconds of spooldown
    for (int i = 0; i < 200; i++) {
      if (fuelCutoff) {
        N1 = N1 * (1.0 - decelRate * dt / 100.0);
      }
    }

    TS_ASSERT(N1 < 10.0);  // Engine spooled down
  }

  /***************************************************************************
   * Extended Performance Calculation Tests
   ***************************************************************************/

  // Test cruise efficiency
  void testCruiseEfficiency() {
    double thrust = 800.0;      // lb
    double fuelFlow = 250.0;    // PPH
    double velocity = 300.0;    // ft/s

    double TSFC = fuelFlow / thrust;  // lb/hr/lb
    TS_ASSERT(TSFC < 1.0);
    TS_ASSERT(TSFC > 0.2);
  }

  // Test power management for climb
  void testPowerManagementClimb() {
    double maxContinuousPower = 1000.0;
    double climbPowerSetting = 0.95;
    double climbPower = maxContinuousPower * climbPowerSetting;

    TS_ASSERT_DELTA(climbPower, 950.0, epsilon);
  }

  // Test cruise power optimization
  void testCruisePowerOptimization() {
    double cruisePowerSettings[] = {0.6, 0.65, 0.7, 0.75, 0.8};
    double optimalSetting = 0.7;  // Best range at 70% power

    for (double setting : cruisePowerSettings) {
      TS_ASSERT(setting >= 0.0);
      TS_ASSERT(setting <= 1.0);
    }
    TS_ASSERT(optimalSetting > 0.5);
    TS_ASSERT(optimalSetting < 0.9);
  }
};
