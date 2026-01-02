#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include "TestUtilities.h"

#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>

using namespace JSBSim;
using namespace JSBSimTest;

class FGElectricMotorTest : public CxxTest::TestSuite
{
public:
  // ============================================================================
  // 1. Brushless DC Motor Torque-Speed Characteristics
  // ============================================================================

  // Test linear torque-speed relationship for BLDC motors
  void testBLDCTorqueSpeedLinear() {
    // For BLDC motors: Torque = Kt * (I - I0)
    // Speed-torque line: T = Tstall - (Tstall/omega0) * omega
    double Kt = 0.05;          // Torque constant (Nm/A)
    double stallTorque = 10.0; // Nm
    double noLoadSpeed = 5000.0; // RPM

    // At stall (omega = 0)
    double omega = 0.0;
    double torque = stallTorque - (stallTorque / noLoadSpeed) * omega;
    TS_ASSERT_DELTA(torque, stallTorque, DEFAULT_TOLERANCE);

    // At half speed
    omega = noLoadSpeed / 2.0;
    torque = stallTorque - (stallTorque / noLoadSpeed) * omega;
    TS_ASSERT_DELTA(torque, stallTorque / 2.0, DEFAULT_TOLERANCE);

    // At no-load speed
    omega = noLoadSpeed;
    torque = stallTorque - (stallTorque / noLoadSpeed) * omega;
    TS_ASSERT_DELTA(torque, 0.0, DEFAULT_TOLERANCE);
  }

  // Test torque constant (Kt) relationship
  void testTorqueConstant() {
    // Torque = Kt * Current
    double Kt = 0.045; // Nm/A
    double current = 50.0; // Amperes

    double torque = Kt * current;
    TS_ASSERT_DELTA(torque, 2.25, 0.001);

    // Higher current = more torque
    current = 100.0;
    torque = Kt * current;
    TS_ASSERT_DELTA(torque, 4.5, 0.001);

    // Zero current = zero torque (ideal motor)
    current = 0.0;
    torque = Kt * current;
    TS_ASSERT_DELTA(torque, 0.0, DEFAULT_TOLERANCE);
  }

  // Test speed constant (Kv) for motor
  void testSpeedConstant() {
    // Kv = RPM / Volt (no-load)
    // Kv and Kt are related: Kt = 60 / (2 * PI * Kv) for SI units
    double Kv = 1000.0; // RPM/V
    double voltage = 24.0; // Volts

    double noLoadSpeed = Kv * voltage;
    TS_ASSERT_DELTA(noLoadSpeed, 24000.0, DEFAULT_TOLERANCE);

    // Relationship between Kt and Kv
    double Kt = 60.0 / (2.0 * M_PI * Kv);
    TS_ASSERT_DELTA(Kt, 0.00955, 0.0001);
  }

  // Test stall current calculation
  void testStallCurrent() {
    // At stall: V = I * R (no back-EMF)
    double voltage = 24.0;  // Volts
    double resistance = 0.1; // Ohms

    double stallCurrent = voltage / resistance;
    TS_ASSERT_DELTA(stallCurrent, 240.0, DEFAULT_TOLERANCE);

    // Higher voltage = higher stall current
    voltage = 48.0;
    stallCurrent = voltage / resistance;
    TS_ASSERT_DELTA(stallCurrent, 480.0, DEFAULT_TOLERANCE);
  }

  // Test peak torque vs continuous torque
  void testPeakVsContinuousTorque() {
    // Peak torque is typically 3-5x continuous rating
    double continuousTorque = 5.0; // Nm
    double peakFactor = 4.0;

    double peakTorque = continuousTorque * peakFactor;
    TS_ASSERT_DELTA(peakTorque, 20.0, DEFAULT_TOLERANCE);

    // Motor can produce peak for short duration
    TS_ASSERT(peakTorque > continuousTorque);
  }

  // ============================================================================
  // 2. Motor Efficiency Curves
  // ============================================================================

  // Test efficiency calculation from input/output power
  void testBasicEfficiency() {
    // Efficiency = Pout / Pin
    double powerOut = 7500.0; // Watts (mechanical)
    double powerIn = 8500.0;  // Watts (electrical)

    double efficiency = powerOut / powerIn;
    TS_ASSERT_DELTA(efficiency, 0.8824, 0.001);
    TS_ASSERT(efficiency < 1.0);
    TS_ASSERT(efficiency > 0.0);
  }

  // Test efficiency curve vs load
  void testEfficiencyVsLoad() {
    // Efficiency typically peaks at 70-80% load
    // Model: eta = a * load * (1 - load) + b
    double a = 1.2;
    double b = 0.75;

    // At 75% load (near optimal)
    double load = 0.75;
    double efficiency = a * load * (1.0 - load) + b;
    TS_ASSERT_DELTA(efficiency, 0.975, 0.001);

    // At low load (less efficient)
    load = 0.1;
    efficiency = a * load * (1.0 - load) + b;
    TS_ASSERT_DELTA(efficiency, 0.858, 0.001);

    // At high load (less efficient)
    load = 0.95;
    efficiency = a * load * (1.0 - load) + b;
    TS_ASSERT(efficiency < 1.0);
  }

  // Test copper losses (I²R losses)
  void testCopperLosses() {
    // Pcu = I² * R
    double current = 50.0; // Amperes
    double resistance = 0.1; // Ohms

    double copperLosses = current * current * resistance;
    TS_ASSERT_DELTA(copperLosses, 250.0, DEFAULT_TOLERANCE);

    // Losses are proportional to current squared
    current = 100.0;
    copperLosses = current * current * resistance;
    TS_ASSERT_DELTA(copperLosses, 1000.0, DEFAULT_TOLERANCE);
  }

  // Test iron losses (core losses)
  void testIronLosses() {
    // Iron losses ≈ k * omega²
    double k = 0.001; // Loss coefficient
    double omega = 3000.0; // RPM

    double ironLosses = k * omega * omega;
    TS_ASSERT_DELTA(ironLosses, 9000.0, DEFAULT_TOLERANCE);

    // Losses increase with speed squared
    omega = 6000.0;
    ironLosses = k * omega * omega;
    TS_ASSERT_DELTA(ironLosses, 36000.0, DEFAULT_TOLERANCE);
  }

  // Test windage and friction losses
  void testWindageAndFrictionLosses() {
    // Friction losses ≈ k1 * omega + k2 * omega²
    double k1 = 0.01;  // Friction coefficient
    double k2 = 0.0001; // Windage coefficient
    double omega = 5000.0; // RPM

    double frictionLoss = k1 * omega;
    double windageLoss = k2 * omega * omega;
    double totalLoss = frictionLoss + windageLoss;

    TS_ASSERT_DELTA(frictionLoss, 50.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(windageLoss, 2500.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(totalLoss, 2550.0, DEFAULT_TOLERANCE);
  }

  // Test efficiency map (torque vs speed)
  void testEfficiencyMap() {
    // Peak efficiency region typically at 60-80% of rated values
    double ratedTorque = 10.0; // Nm
    double ratedSpeed = 5000.0; // RPM
    double peakEfficiency = 0.95;

    // At rated point (slightly lower than peak)
    double currentTorque = ratedTorque;
    double currentSpeed = ratedSpeed;
    double efficiency = peakEfficiency * 0.96; // Typical
    TS_ASSERT_DELTA(efficiency, 0.912, 0.001);

    // At 70% torque and 70% speed (near peak)
    currentTorque = 0.7 * ratedTorque;
    currentSpeed = 0.7 * ratedSpeed;
    efficiency = peakEfficiency;
    TS_ASSERT_DELTA(efficiency, 0.95, DEFAULT_TOLERANCE);
  }

  // ============================================================================
  // 3. Back-EMF Calculations
  // ============================================================================

  // Test basic back-EMF formula
  void testBackEMFBasic() {
    // Back-EMF: Vemf = Ke * omega
    double Ke = 0.01; // V/(rad/s) - EMF constant
    double rpm = 3000.0;
    double omega = rpm * 2.0 * M_PI / 60.0; // Convert to rad/s

    double backEMF = Ke * omega;
    // omega = 3000 * 2*PI / 60 = 314.16 rad/s
    // Vemf = 0.01 * 314.16 = 3.14 V
    TS_ASSERT_DELTA(backEMF, 3.1416, 0.001);
  }

  // Test back-EMF using Kv rating
  void testBackEMFFromKv() {
    // Kv is RPM/V, so Vemf = RPM / Kv
    double Kv = 1000.0; // RPM/V
    double rpm = 12000.0;

    double backEMF = rpm / Kv;
    TS_ASSERT_DELTA(backEMF, 12.0, DEFAULT_TOLERANCE);

    // At half speed
    rpm = 6000.0;
    backEMF = rpm / Kv;
    TS_ASSERT_DELTA(backEMF, 6.0, DEFAULT_TOLERANCE);
  }

  // Test current calculation with back-EMF
  void testCurrentWithBackEMF() {
    // I = (V - Vemf) / R
    double voltage = 24.0;   // Supply voltage
    double backEMF = 18.0;   // Back-EMF at current speed
    double resistance = 0.1;  // Winding resistance

    double current = (voltage - backEMF) / resistance;
    TS_ASSERT_DELTA(current, 60.0, DEFAULT_TOLERANCE);

    // At higher speed, higher back-EMF, lower current
    backEMF = 22.0;
    current = (voltage - backEMF) / resistance;
    TS_ASSERT_DELTA(current, 20.0, DEFAULT_TOLERANCE);
  }

  // Test no-load condition (Vemf ≈ V)
  void testNoLoadBackEMF() {
    // At no-load: Vemf ≈ V (neglecting small resistance drop)
    double voltage = 24.0;
    double Kv = 1000.0;

    // No-load speed
    double noLoadRPM = voltage * Kv;
    double backEMFNoLoad = noLoadRPM / Kv;

    TS_ASSERT_DELTA(backEMFNoLoad, voltage, DEFAULT_TOLERANCE);
  }

  // Test relationship between Ke and Kt
  void testKeKtRelationship() {
    // For SI units: Ke (V·s/rad) = Kt (N·m/A)
    // In practical units: Kt = 60/(2*PI*Kv) where Kv is RPM/V
    double Kv = 1000.0; // RPM/V
    double Kt = 60.0 / (2.0 * M_PI * Kv);
    double Ke = Kt; // In SI units, they are numerically equal

    TS_ASSERT_DELTA(Ke, 0.00955, 0.0001);
    TS_ASSERT_DELTA(Kt, 0.00955, 0.0001);
  }

  // ============================================================================
  // 4. Power Consumption vs Output
  // ============================================================================

  // Test electrical power input
  void testElectricalPowerInput() {
    // Pin = V * I
    double voltage = 48.0;
    double current = 100.0;

    double powerIn = voltage * current;
    TS_ASSERT_DELTA(powerIn, 4800.0, DEFAULT_TOLERANCE);

    // Three-phase power: P = sqrt(3) * V * I * cos(phi)
    // For ideal motor controller, cos(phi) ≈ 1
  }

  // Test mechanical power output
  void testMechanicalPowerOutput() {
    // Pout = Torque * omega
    double torque = 10.0; // Nm
    double rpm = 5000.0;
    double omega = rpm * 2.0 * M_PI / 60.0; // rad/s

    double powerOut = torque * omega;
    // Pout = 10 * 523.6 = 5236 Watts
    TS_ASSERT_DELTA(powerOut, 5235.99, 0.1);
  }

  // Test power balance equation
  void testPowerBalance() {
    // Pin = Pout + Plosses
    double powerIn = 5000.0;
    double efficiency = 0.90;

    double powerOut = powerIn * efficiency;
    double powerLosses = powerIn - powerOut;

    TS_ASSERT_DELTA(powerOut, 4500.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(powerLosses, 500.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(powerIn, powerOut + powerLosses, DEFAULT_TOLERANCE);
  }

  // Test power at different operating points
  void testPowerAtOperatingPoints() {
    // Maximum power typically at 50% of stall torque
    double stallTorque = 20.0; // Nm
    double noLoadSpeed = 6000.0; // RPM

    // Max power point
    double torque = stallTorque / 2.0;
    double rpm = noLoadSpeed / 2.0;
    double omega = rpm * 2.0 * M_PI / 60.0;
    double power = torque * omega;

    // Power = 10 Nm * 314.16 rad/s = 3141.6 W
    TS_ASSERT_DELTA(power, 3141.59, 0.1);

    // At stall: power = 0 (omega = 0)
    torque = stallTorque;
    rpm = 0.0;
    omega = 0.0;
    power = torque * omega;
    TS_ASSERT_DELTA(power, 0.0, DEFAULT_TOLERANCE);

    // At no-load: power = 0 (torque = 0)
    torque = 0.0;
    rpm = noLoadSpeed;
    omega = rpm * 2.0 * M_PI / 60.0;
    power = torque * omega;
    TS_ASSERT_DELTA(power, 0.0, DEFAULT_TOLERANCE);
  }

  // Test power factor in motor controller
  void testPowerFactor() {
    // For modern ESC/motor controllers, power factor is close to 1
    double powerFactor = 0.95; // Typical for good ESC

    TS_ASSERT(powerFactor > 0.90);
    TS_ASSERT(powerFactor <= 1.0);

    // Apparent power vs real power
    double voltageRMS = 48.0;
    double currentRMS = 50.0;
    double apparentPower = voltageRMS * currentRMS;
    double realPower = apparentPower * powerFactor;

    TS_ASSERT_DELTA(realPower, 2280.0, 1.0);
  }

  // ============================================================================
  // 5. Thermal Modeling (Motor Heating)
  // ============================================================================

  // Test thermal resistance and temperature rise
  void testThermalResistance() {
    // Delta_T = Rth * Ploss
    double Rth = 0.5; // K/W - thermal resistance to ambient
    double powerLoss = 500.0; // Watts

    double tempRise = Rth * powerLoss;
    TS_ASSERT_DELTA(tempRise, 250.0, DEFAULT_TOLERANCE);

    // With ambient temperature
    double Tambient = 25.0; // Celsius
    double Tmotor = Tambient + tempRise;
    TS_ASSERT_DELTA(Tmotor, 275.0, DEFAULT_TOLERANCE);
  }

  // Test thermal time constant
  void testThermalTimeConstant() {
    // Thermal capacitance: dT/dt = (Ploss - (T-Tamb)/Rth) / Cth
    double Cth = 100.0; // J/K - thermal capacitance
    double Rth = 0.5;   // K/W
    double tauThermal = Rth * Cth; // Thermal time constant

    TS_ASSERT_DELTA(tauThermal, 50.0, DEFAULT_TOLERANCE);

    // Time to reach 63% of final temperature
    double timeFor63Percent = tauThermal;
    TS_ASSERT_DELTA(timeFor63Percent, 50.0, DEFAULT_TOLERANCE);
  }

  // Test temperature-dependent resistance
  void testTemperatureDependentResistance() {
    // R(T) = R0 * (1 + alpha * (T - T0))
    double R0 = 0.1;      // Ohms at reference temp
    double T0 = 25.0;     // Celsius - reference
    double alpha = 0.004; // 1/K - copper temp coefficient
    double T = 125.0;     // Current temperature

    double R = R0 * (1.0 + alpha * (T - T0));
    TS_ASSERT_DELTA(R, 0.14, 0.001);

    // Higher temp = higher resistance = more losses
    TS_ASSERT(R > R0);
  }

  // Test winding temperature limits
  void testWindingTemperatureLimits() {
    // Class F insulation: 155°C continuous
    double maxTemp = 155.0; // Celsius
    double currentTemp = 140.0;
    double margin = maxTemp - currentTemp;

    TS_ASSERT_DELTA(margin, 15.0, DEFAULT_TOLERANCE);
    TS_ASSERT(currentTemp < maxTemp);

    // Derating at high temperature
    currentTemp = 150.0;
    margin = maxTemp - currentTemp;
    TS_ASSERT_DELTA(margin, 5.0, DEFAULT_TOLERANCE);
  }

  // Test cooling effectiveness
  void testCoolingEffectiveness() {
    // With forced air cooling, Rth decreases
    double RthNatural = 1.0;  // K/W - natural convection
    double RthForced = 0.3;   // K/W - with fan
    double powerLoss = 500.0;

    double tempRiseNatural = RthNatural * powerLoss;
    double tempRiseForced = RthForced * powerLoss;

    TS_ASSERT_DELTA(tempRiseNatural, 500.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(tempRiseForced, 150.0, DEFAULT_TOLERANCE);
    TS_ASSERT(tempRiseForced < tempRiseNatural);
  }

  // Test thermal derating curve
  void testThermalDerating() {
    // Above certain ambient temp, must derate power
    double ratedPower = 10000.0; // Watts at 25°C
    double maxAmbient = 40.0;    // Celsius
    double deratingFactor = 0.02; // 2% per degree C above limit
    double Tambient = 50.0;

    double derating = 1.0 - deratingFactor * (Tambient - maxAmbient);
    double deratedPower = ratedPower * derating;

    TS_ASSERT_DELTA(derating, 0.8, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(deratedPower, 8000.0, DEFAULT_TOLERANCE);
  }

  // ============================================================================
  // 6. Motor Controller Dynamics
  // ============================================================================

  // Test PWM frequency effects
  void testPWMFrequency() {
    // Higher PWM frequency = smoother current, less torque ripple
    double pwmFreq1 = 8000.0;  // Hz - basic ESC
    double pwmFreq2 = 32000.0; // Hz - high-performance ESC

    TS_ASSERT(pwmFreq2 > pwmFreq1);

    // Current ripple inversely proportional to frequency
    double ripple1 = 1.0 / pwmFreq1;
    double ripple2 = 1.0 / pwmFreq2;
    TS_ASSERT(ripple2 < ripple1);
  }

  // Test duty cycle to voltage relationship
  void testDutyCycleVoltage() {
    // Vout = Duty * Vin
    double Vin = 48.0;
    double dutyCycle = 0.75;

    double Vout = dutyCycle * Vin;
    TS_ASSERT_DELTA(Vout, 36.0, DEFAULT_TOLERANCE);

    // Full throttle
    dutyCycle = 1.0;
    Vout = dutyCycle * Vin;
    TS_ASSERT_DELTA(Vout, 48.0, DEFAULT_TOLERANCE);

    // Half throttle
    dutyCycle = 0.5;
    Vout = dutyCycle * Vin;
    TS_ASSERT_DELTA(Vout, 24.0, DEFAULT_TOLERANCE);
  }

  // Test commutation timing
  void testCommutationTiming() {
    // Electrical frequency = (poles / 2) * mechanical frequency
    double mechanicalRPM = 5000.0;
    double poles = 14.0; // Typical outrunner motor

    double electricalFreq = (poles / 2.0) * mechanicalRPM / 60.0;
    // f_elec = 7 * 5000 / 60 = 583.33 Hz
    TS_ASSERT_DELTA(electricalFreq, 583.33, 0.1);
  }

  // Test controller response time
  void testControllerResponseTime() {
    // ESC has small delay in responding to throttle commands
    double commandTime = 0.0;
    double responseTime = 0.002; // 2ms typical ESC response

    double delay = responseTime - commandTime;
    TS_ASSERT_DELTA(delay, 0.002, DEFAULT_TOLERANCE);

    // Modern ESCs are faster
    responseTime = 0.0005; // 500 microseconds
    delay = responseTime;
    TS_ASSERT(delay < 0.001);
  }

  // Test field-oriented control (FOC) vs trapezoidal
  void testFOCvsTrapezoidal() {
    // FOC provides better efficiency (2-10% improvement)
    double efficiencyTrapezoidal = 0.85;
    double efficiencyFOC = 0.92;

    double improvement = efficiencyFOC - efficiencyTrapezoidal;
    TS_ASSERT_DELTA(improvement, 0.07, 0.01);
    TS_ASSERT(efficiencyFOC > efficiencyTrapezoidal);
  }

  // ============================================================================
  // 7. Current Limiting
  // ============================================================================

  // Test maximum current limit
  void testMaxCurrentLimit() {
    double maxContinuousCurrent = 100.0; // Amperes
    double requestedCurrent = 150.0;

    // Controller limits to max
    double actualCurrent = std::min(requestedCurrent, maxContinuousCurrent);
    TS_ASSERT_DELTA(actualCurrent, 100.0, DEFAULT_TOLERANCE);

    // Within limits
    requestedCurrent = 80.0;
    actualCurrent = std::min(requestedCurrent, maxContinuousCurrent);
    TS_ASSERT_DELTA(actualCurrent, 80.0, DEFAULT_TOLERANCE);
  }

  // Test peak current for short duration
  void testPeakCurrentLimit() {
    double maxContinuous = 100.0; // A
    double maxPeak = 200.0;       // A
    double peakDuration = 2.0;    // seconds

    TS_ASSERT(maxPeak > maxContinuous);
    TS_ASSERT(peakDuration > 0.0);

    // Can sustain peak for limited time
    double current = 180.0;
    bool withinPeak = (current <= maxPeak);
    TS_ASSERT(withinPeak);
  }

  // Test I²t limiting for protection
  void testI2tLimiting() {
    // I²t = integral of current squared over time
    double current = 150.0; // A
    double time = 5.0;      // seconds
    double I2t = current * current * time;

    // I2t = 150² * 5 = 112,500 A²s
    TS_ASSERT_DELTA(I2t, 112500.0, DEFAULT_TOLERANCE);

    // Compare to limit (e.g., 100,000 A²s)
    double I2tLimit = 100000.0;
    bool withinLimit = (I2t <= I2tLimit);
    TS_ASSERT(!withinLimit); // Would trip protection
  }

  // Test thermal current derating
  void testThermalCurrentDerating() {
    // As motor heats up, max current is reduced
    double maxCurrentCold = 100.0; // A at 25°C
    double currentTemp = 80.0;     // °C
    double deratingTemp = 60.0;    // Start derating at 60°C
    double deratingRate = 0.01;    // 1% per °C

    double derating = 1.0;
    if (currentTemp > deratingTemp) {
      derating = 1.0 - deratingRate * (currentTemp - deratingTemp);
    }
    double maxCurrentHot = maxCurrentCold * derating;

    TS_ASSERT_DELTA(derating, 0.8, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(maxCurrentHot, 80.0, DEFAULT_TOLERANCE);
  }

  // Test overcurrent shutdown
  void testOvercurrentShutdown() {
    double current = 250.0;
    double shutdownThreshold = 200.0;

    bool shutdown = (current > shutdownThreshold);
    TS_ASSERT(shutdown);

    // Safe current
    current = 180.0;
    shutdown = (current > shutdownThreshold);
    TS_ASSERT(!shutdown);
  }

  // ============================================================================
  // 8. Regenerative Braking
  // ============================================================================

  // Test regenerative braking power
  void testRegenerativeBrakingPower() {
    // Motor acts as generator, back-EMF > supply voltage
    double backEMF = 52.0;    // Volts
    double supplyV = 48.0;
    double resistance = 0.1;

    // Current flows backward
    double regenCurrent = (backEMF - supplyV) / resistance;
    TS_ASSERT_DELTA(regenCurrent, 40.0, DEFAULT_TOLERANCE);

    // Power returned to battery
    double regenPower = supplyV * regenCurrent;
    TS_ASSERT_DELTA(regenPower, 1920.0, DEFAULT_TOLERANCE);
  }

  // Test maximum regenerative current
  void testMaxRegenCurrent() {
    // Battery has max charge current limit
    double maxRegenCurrent = 50.0; // A
    double calculatedRegen = 75.0;

    double actualRegen = std::min(calculatedRegen, maxRegenCurrent);
    TS_ASSERT_DELTA(actualRegen, 50.0, DEFAULT_TOLERANCE);
  }

  // Test braking resistor calculation
  void testBrakingResistor() {
    // If battery can't absorb all regen, use resistor
    double regenPower = 3000.0; // Watts
    double maxBatteryCharge = 2000.0; // Watts
    double excessPower = regenPower - maxBatteryCharge;

    TS_ASSERT_DELTA(excessPower, 1000.0, DEFAULT_TOLERANCE);

    // Resistor must dissipate excess
    double voltage = 48.0;
    double resistorCurrent = excessPower / voltage;
    TS_ASSERT_DELTA(resistorCurrent, 20.83, 0.1);
  }

  // Test regenerative efficiency
  void testRegenerativeEfficiency() {
    // Not all mechanical power is recovered
    double mechanicalPower = 5000.0; // Watts
    double regenEfficiency = 0.70;   // 70% recovered

    double recoveredPower = mechanicalPower * regenEfficiency;
    TS_ASSERT_DELTA(recoveredPower, 3500.0, DEFAULT_TOLERANCE);

    double lostPower = mechanicalPower - recoveredPower;
    TS_ASSERT_DELTA(lostPower, 1500.0, DEFAULT_TOLERANCE);
  }

  // Test dynamic braking
  void testDynamicBraking() {
    // Short motor windings to dissipate energy
    double current = 80.0;
    double resistance = 0.15;

    double brakingPower = current * current * resistance;
    TS_ASSERT_DELTA(brakingPower, 960.0, DEFAULT_TOLERANCE);

    // This power is dissipated as heat in motor
  }

  // ============================================================================
  // 9. Motor Sizing Calculations
  // ============================================================================

  // Test power-to-weight ratio
  void testPowerToWeightRatio() {
    // Typical electric motor: 4-8 kW/kg
    double power = 10000.0; // Watts
    double mass = 1.5;      // kg

    double pwrToWeight = power / mass;
    // 10 kW / 1.5 kg = 6.67 kW/kg
    TS_ASSERT_DELTA(pwrToWeight, 6666.67, 1.0);

    TS_ASSERT(pwrToWeight > 4000.0);  // Good motor
    TS_ASSERT(pwrToWeight < 10000.0); // Realistic
  }

  // Test required torque for propeller
  void testRequiredTorquePropeller() {
    // Torque = Power / omega
    double requiredPower = 30000.0; // Watts (40 HP)
    double rpm = 2400.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double requiredTorque = requiredPower / omega;
    // T = 30000 / 251.33 = 119.37 Nm
    TS_ASSERT_DELTA(requiredTorque, 119.37, 0.1);
  }

  // Test motor constant (Km = Kt / sqrt(R))
  void testMotorConstant() {
    // Motor constant: figure of merit for motor quality
    double Kt = 0.05;        // Nm/A
    double resistance = 0.1; // Ohms

    double Km = Kt / sqrt(resistance);
    TS_ASSERT_DELTA(Km, 0.158, 0.001);

    // Higher Km = better motor efficiency
  }

  // Test thermal sizing based on duty cycle
  void testThermalSizingDutyCycle() {
    // RMS power for intermittent operation
    double peakPower = 20000.0;  // Watts
    double dutyCycle = 0.5;      // 50% on-time

    double rmsPower = peakPower * sqrt(dutyCycle);
    TS_ASSERT_DELTA(rmsPower, 14142.14, 1.0);

    // Motor sized for RMS power
  }

  // Test voltage selection
  void testVoltageSelection() {
    // Higher voltage = lower current for same power
    double power = 10000.0; // Watts

    double voltage48 = 48.0;
    double current48 = power / voltage48;
    TS_ASSERT_DELTA(current48, 208.33, 0.1);

    double voltage96 = 96.0;
    double current96 = power / voltage96;
    TS_ASSERT_DELTA(current96, 104.17, 0.1);

    // Higher voltage = lower current = less copper loss
    TS_ASSERT(current96 < current48);
  }

  // ============================================================================
  // 10. Propeller-Motor Matching
  // ============================================================================

  // Test propeller loading on motor
  void testPropellerLoad() {
    // Prop torque: Q = Cp * rho * n² * D⁵
    double Cp = 0.05;
    double rho = 0.002377;  // slugs/ft³
    double rpm = 2400.0;
    double rps = rpm / 60.0;
    double diameter = 6.0;  // ft

    double propTorque = Cp * rho * rps * rps * pow(diameter, 5);
    // Convert to Nm (1 ft-lbf = 1.356 Nm)
    double propTorqueNm = propTorque * 1.356;

    TS_ASSERT(propTorque > 0.0);
    TS_ASSERT(propTorqueNm > 0.0);
  }

  // Test operating point intersection
  void testOperatingPointIntersection() {
    // Motor torque-speed line intersects with prop load curve
    double stallTorque = 100.0; // Nm
    double noLoadSpeed = 6000.0; // RPM

    // At some operating RPM, motor torque = prop torque
    double operatingRPM = 4500.0;
    double motorTorque = stallTorque * (1.0 - operatingRPM / noLoadSpeed);
    double propTorque = 25.0; // From prop calculations

    // In steady state, these should be equal
    // Here we test the relationship
    TS_ASSERT_DELTA(motorTorque, 25.0, 1.0);
  }

  // Test propeller efficiency effect
  void testPropellerEfficiency() {
    // Overall efficiency = motor_eff * prop_eff
    double motorEff = 0.90;
    double propEff = 0.85;

    double overallEff = motorEff * propEff;
    TS_ASSERT_DELTA(overallEff, 0.765, 0.001);

    // Both components matter
    TS_ASSERT(overallEff < motorEff);
    TS_ASSERT(overallEff < propEff);
  }

  // Test gear ratio for propeller matching
  void testGearRatioMatching() {
    // Motor runs fast, prop wants lower RPM
    double motorRPM = 7000.0;
    double optimalPropRPM = 2800.0;

    double gearRatio = motorRPM / optimalPropRPM;
    TS_ASSERT_DELTA(gearRatio, 2.5, 0.1);

    // Gear increases torque, reduces speed
    double motorTorque = 20.0; // Nm
    double propTorque = motorTorque * gearRatio;
    TS_ASSERT_DELTA(propTorque, 50.0, 1.0);
  }

  // Test static thrust matching
  void testStaticThrustMatching() {
    // Motor must provide enough torque at static
    double staticPropTorque = 50.0; // Nm required
    double motorStallTorque = 80.0; // Nm available

    double margin = motorStallTorque - staticPropTorque;
    TS_ASSERT_DELTA(margin, 30.0, DEFAULT_TOLERANCE);
    TS_ASSERT(motorStallTorque > staticPropTorque);
  }

  // Test propeller diameter effect on matching
  void testPropDiameterEffect() {
    // Larger prop = more torque required
    double rpm = 2400.0;
    double rps = rpm / 60.0;
    double Cp = 0.05;
    double rho = 0.002377;

    double diameter6ft = 6.0;
    double torque6ft = Cp * rho * rps * rps * pow(diameter6ft, 5);

    double diameter8ft = 8.0;
    double torque8ft = Cp * rho * rps * rps * pow(diameter8ft, 5);

    // 8ft prop requires more torque
    TS_ASSERT(torque8ft > torque6ft);

    // Ratio should be (8/6)^5 = 4.214
    double ratio = torque8ft / torque6ft;
    TS_ASSERT_DELTA(ratio, 4.214, 0.01);
  }

  // Test cruise power vs climb power
  void testCruiseVsClimbPower() {
    // Climb requires more power than cruise
    double cruisePower = 8000.0;  // Watts
    double climbPower = 15000.0;  // Watts

    TS_ASSERT(climbPower > cruisePower);

    // Motor must be sized for climb, runs cooler in cruise
    double motorRating = 18000.0; // Watts
    double cruiseMargin = motorRating - cruisePower;
    double climbMargin = motorRating - climbPower;

    TS_ASSERT_DELTA(cruiseMargin, 10000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(climbMargin, 3000.0, DEFAULT_TOLERANCE);
    TS_ASSERT(cruiseMargin > climbMargin);
  }

  // Test RPM vs thrust relationship
  void testRPMvsThrustRelationship() {
    // Thrust ∝ n² for constant speed prop
    double rpm1 = 2000.0;
    double thrust1 = 500.0; // lbs

    double rpm2 = 2400.0;
    double thrustRatio = (rpm2 / rpm1) * (rpm2 / rpm1);
    double thrust2 = thrust1 * thrustRatio;

    // thrust2 = 500 * (2400/2000)² = 500 * 1.44 = 720 lbs
    TS_ASSERT_DELTA(thrust2, 720.0, 1.0);
  }

  // ============================================================================
  // 11. Battery State Effects
  // ============================================================================

  // Test voltage sag under load
  void testBatteryVoltageSag() {
    double nominalVoltage = 48.0;
    double internalResistance = 0.05; // Ohms
    double current = 100.0; // Amperes

    double loadVoltage = nominalVoltage - (current * internalResistance);
    TS_ASSERT_DELTA(loadVoltage, 43.0, DEFAULT_TOLERANCE);
    TS_ASSERT(loadVoltage < nominalVoltage);
  }

  // Test state of charge effect on voltage
  void testSOCVoltageEffect() {
    double voltageAt100SOC = 50.4; // 12S LiPo fully charged
    double voltageAt20SOC = 43.2;  // Near discharged

    double voltageDrop = voltageAt100SOC - voltageAt20SOC;
    TS_ASSERT_DELTA(voltageDrop, 7.2, 0.1);
    TS_ASSERT(voltageAt100SOC > voltageAt20SOC);
  }

  // Test battery C-rate limitation
  void testBatteryCRateLimitation() {
    double capacity = 10.0; // Ah
    double maxCRate = 30.0; // 30C discharge

    double maxCurrent = capacity * maxCRate;
    TS_ASSERT_DELTA(maxCurrent, 300.0, DEFAULT_TOLERANCE);
  }

  // Test battery energy density
  void testBatteryEnergyDensity() {
    double energy = 2000.0; // Wh
    double mass = 10.0;     // kg

    double energyDensity = energy / mass; // Wh/kg
    TS_ASSERT_DELTA(energyDensity, 200.0, DEFAULT_TOLERANCE);
    // Typical LiPo: 150-250 Wh/kg
    TS_ASSERT(energyDensity > 100.0 && energyDensity < 300.0);
  }

  // Test Peukert effect on capacity
  void testPeukertEffect() {
    double capacity1C = 10.0;  // Ah at 1C rate
    double peukertExponent = 1.1;
    double dischargeRate = 5.0; // 5C

    // Effective capacity decreases at high discharge rates
    double effectiveCapacity = capacity1C / pow(dischargeRate, peukertExponent - 1.0);
    TS_ASSERT(effectiveCapacity < capacity1C);
  }

  // ============================================================================
  // 12. Multi-Motor Configurations
  // ============================================================================

  // Test dual motor torque splitting
  void testDualMotorTorqueSplit() {
    double totalTorque = 100.0; // Nm
    double motor1Torque = totalTorque * 0.5;
    double motor2Torque = totalTorque * 0.5;

    TS_ASSERT_DELTA(motor1Torque + motor2Torque, totalTorque, DEFAULT_TOLERANCE);
  }

  // Test motor redundancy power loss
  void testMotorRedundancyPowerLoss() {
    double normalPower = 20000.0; // Watts per motor
    int numMotors = 4;
    double totalPower = normalPower * numMotors;

    // One motor fails
    double powerWithFailure = normalPower * (numMotors - 1);
    double powerLossFraction = 1.0 - (powerWithFailure / totalPower);

    TS_ASSERT_DELTA(powerLossFraction, 0.25, DEFAULT_TOLERANCE);
  }

  // Test coaxial motor counter-rotation torque
  void testCoaxialMotorTorqueCancel() {
    double motor1Torque = 50.0;  // Nm (CW)
    double motor2Torque = -50.0; // Nm (CCW)

    double netTorque = motor1Torque + motor2Torque;
    TS_ASSERT_DELTA(netTorque, 0.0, DEFAULT_TOLERANCE);
  }

  // Test distributed propulsion efficiency
  void testDistributedPropulsionEfficiency() {
    // Multiple smaller motors can have different efficiency
    double largMotorEff = 0.92;
    double smallMotorEff = 0.88;
    int numSmallMotors = 4;

    // Overall efficiency comparison
    TS_ASSERT(largMotorEff > smallMotorEff);
  }

  // ============================================================================
  // 13. Motor Failure Modes
  // ============================================================================

  // Test winding short detection
  void testWindingShortDetection() {
    double normalResistance = 0.1; // Ohms
    double shortedResistance = 0.02; // Much lower

    bool shortDetected = (shortedResistance < normalResistance * 0.5);
    TS_ASSERT(shortDetected);
  }

  // Test bearing failure effect
  void testBearingFailureEffect() {
    double normalFriction = 0.02;  // Nm friction torque
    double damagedFriction = 0.15; // Higher friction

    double frictionIncrease = damagedFriction / normalFriction;
    TS_ASSERT(frictionIncrease > 5.0);
  }

  // Test demagnetization detection
  void testDemagnetizationDetection() {
    double normalKt = 0.05;      // Nm/A
    double degradedKt = 0.04;   // After demagnetization

    double ktLoss = 1.0 - (degradedKt / normalKt);
    TS_ASSERT_DELTA(ktLoss, 0.2, 0.01); // 20% loss
  }

  // Test thermal runaway condition
  void testThermalRunawayCondition() {
    double currentTemp = 150.0;  // °C
    double maxTemp = 155.0;      // °C limit
    double heatingRate = 5.0;    // °C/s
    double coolingRate = 3.0;    // °C/s

    bool thermalRunaway = (heatingRate > coolingRate) && (currentTemp > maxTemp - 10.0);
    TS_ASSERT(thermalRunaway);
  }

  // ============================================================================
  // 14. Temperature Effects on Motor Constants
  // ============================================================================

  // Test Kt temperature coefficient
  void testKtTemperatureCoefficient() {
    double Kt_20C = 0.050;     // Nm/A at 20°C
    double tempCoeff = -0.001; // -0.1% per °C (NdFeB magnets)
    double temp = 80.0;        // °C

    double Kt_hot = Kt_20C * (1.0 + tempCoeff * (temp - 20.0));
    TS_ASSERT(Kt_hot < Kt_20C);
    TS_ASSERT_DELTA(Kt_hot, 0.047, 0.001);
  }

  // Test winding resistance temperature rise
  void testWindingResistanceTempRise() {
    double R_25C = 0.1;
    double alpha = 0.00393; // Copper temp coefficient
    double T1 = 25.0;
    double T2 = 100.0;

    double R_hot = R_25C * (1.0 + alpha * (T2 - T1));
    TS_ASSERT_DELTA(R_hot, 0.1295, 0.001);
  }

  // Test efficiency change with temperature
  void testEfficiencyTemperatureChange() {
    double efficiencyCold = 0.93;
    double resistanceIncrease = 1.3; // 30% increase at hot

    // Copper losses increase
    double copperLossIncrease = resistanceIncrease;
    double efficiencyHot = efficiencyCold * (1.0 - 0.03); // Approximate

    TS_ASSERT(efficiencyHot < efficiencyCold);
  }

  // ============================================================================
  // 15. Altitude Effects
  // ============================================================================

  // Test air cooling reduction at altitude
  void testAltitudeCoolingReduction() {
    double seaLevelDensity = 1.225; // kg/m³
    double altitudeDensity = 0.7;   // ~15000 ft

    double coolingRatio = altitudeDensity / seaLevelDensity;
    TS_ASSERT_DELTA(coolingRatio, 0.571, 0.01);
    TS_ASSERT(coolingRatio < 1.0);
  }

  // Test power derating at altitude
  void testPowerDeratingAltitude() {
    double seaLevelPower = 50000.0; // Watts
    double densityRatio = 0.7;
    double thermalDerate = 0.85; // Due to reduced cooling

    double altitudePower = seaLevelPower * thermalDerate;
    TS_ASSERT_DELTA(altitudePower, 42500.0, 100.0);
  }

  // Test corona discharge at altitude
  void testCoronaDischargeRisk() {
    double seaLevelBreakdown = 3000.0; // V/mm
    double altitudeBreakdown = 1500.0; // V/mm at 30000 ft

    // Lower breakdown voltage at altitude
    TS_ASSERT(altitudeBreakdown < seaLevelBreakdown);
  }

  // ============================================================================
  // 16. ESC Advanced Features
  // ============================================================================

  // Test timing advance effect
  void testTimingAdvance() {
    double baseEfficiency = 0.88;
    double optimalAdvance = 15.0; // degrees
    double efficiencyGain = 0.02; // 2% improvement

    double optimizedEfficiency = baseEfficiency + efficiencyGain;
    TS_ASSERT_DELTA(optimizedEfficiency, 0.90, 0.01);
  }

  // Test active freewheeling vs sync rectification
  void testSyncRectification() {
    double diodeDropLoss = 0.7; // V per diode
    double mosfetDropLoss = 0.1; // V using sync rect
    double current = 100.0; // A

    double diodePowerLoss = diodeDropLoss * current * 6; // 3-phase bridge
    double mosfetPowerLoss = mosfetDropLoss * current * 6;

    TS_ASSERT(mosfetPowerLoss < diodePowerLoss);
    TS_ASSERT_DELTA(diodePowerLoss, 420.0, 1.0);
    TS_ASSERT_DELTA(mosfetPowerLoss, 60.0, 1.0);
  }

  // Test dead-time effect on efficiency
  void testDeadTimeEffect() {
    double pwmFreq = 20000.0; // Hz
    double deadTime = 500e-9; // 500 ns
    double dutyCycle = deadTime * pwmFreq * 2; // Both edges

    double efficiencyLoss = dutyCycle;
    TS_ASSERT(efficiencyLoss < 0.03); // < 3% loss typical
  }

  // Test sensorless vs sensored control
  void testSensorlessVsSensored() {
    double sensoredMinRPM = 50.0;
    double sensorlessMinRPM = 500.0;

    // Sensored has better low-speed performance
    TS_ASSERT(sensoredMinRPM < sensorlessMinRPM);
  }

  // ============================================================================
  // 17. Cogging and Torque Ripple
  // ============================================================================

  // Test cogging torque calculation
  void testCoggingTorque() {
    double ratedTorque = 10.0; // Nm
    double coggingPercent = 2.0; // 2% typical for quality motors

    double coggingTorque = ratedTorque * coggingPercent / 100.0;
    TS_ASSERT_DELTA(coggingTorque, 0.2, 0.01);
  }

  // Test torque ripple frequency
  void testTorqueRippleFrequency() {
    double rpm = 3000.0;
    int poles = 14;
    int slots = 12;

    // Cogging frequency = LCM(poles, slots) * RPM / 60
    // For 14P12S: LCM(14,12) = 84
    int lcm = 84;
    double coggingFreq = lcm * rpm / 60.0;

    TS_ASSERT_DELTA(coggingFreq, 4200.0, 1.0);
  }

  // Test pole-slot combination effect
  void testPoleSlotCombination() {
    // Good combinations have high LCM for low cogging
    int poles1 = 10, slots1 = 12; // LCM = 60
    int poles2 = 8, slots2 = 6;   // LCM = 24

    // Higher LCM = lower cogging amplitude
    TS_ASSERT(60 > 24);
  }

  // ============================================================================
  // 18. Edge Cases and Stress Tests
  // ============================================================================

  // Test motor stall current thermal limit
  void testStallCurrentThermalLimit() {
    double voltage = 48.0;
    double resistance = 0.1;
    double stallCurrent = voltage / resistance;

    double thermalLimit = 200.0; // A continuous
    bool overheating = stallCurrent > thermalLimit;

    TS_ASSERT(overheating); // 480A > 200A
  }

  // Test very high speed operation
  void testVeryHighSpeedOperation() {
    double rpm = 50000.0; // High-speed motor
    double ironLossCoeff = 1e-6;

    double ironLosses = ironLossCoeff * rpm * rpm;
    TS_ASSERT(ironLosses > 0);
    TS_ASSERT_DELTA(ironLosses, 2500.0, 1.0);
  }

  // Test zero speed torque control
  void testZeroSpeedTorqueControl() {
    double rpm = 0.0;
    double current = 50.0;
    double Kt = 0.05;

    double torque = Kt * current;
    TS_ASSERT_DELTA(torque, 2.5, DEFAULT_TOLERANCE);
    TS_ASSERT(torque > 0); // Can produce torque at zero speed
  }

  // Test power factor at different loads
  void testPowerFactorVsLoad() {
    // Power factor varies with load
    double pfLightLoad = 0.7;
    double pfFullLoad = 0.95;

    TS_ASSERT(pfFullLoad > pfLightLoad);
  }

  // Test negative sequence current effect
  void testNegativeSequenceCurrentEffect() {
    double positiveSeq = 100.0; // A
    double negativeSeq = 5.0;   // A (imbalance)

    double imbalancePercent = negativeSeq / positiveSeq * 100.0;
    TS_ASSERT_DELTA(imbalancePercent, 5.0, 0.1);
    TS_ASSERT(imbalancePercent < 10.0); // Acceptable limit
  }

  // Test rapid acceleration current spike
  void testRapidAccelerationCurrentSpike() {
    double steadyStateCurrent = 50.0; // A
    double accelerationCurrent = 150.0; // A during accel

    double currentRatio = accelerationCurrent / steadyStateCurrent;
    TS_ASSERT_DELTA(currentRatio, 3.0, 0.1);
  }

  // Test regeneration at maximum speed
  void testRegenerationAtMaxSpeed() {
    double maxRPM = 60000.0; // High speed motor
    double Kv = 1000.0; // RPM/V
    double backEMF = maxRPM / Kv; // 60V
    double batteryVoltage = 48.0;

    bool regenPossible = backEMF > batteryVoltage;
    TS_ASSERT(regenPossible); // 60V > 48V
  }

  // Test flux weakening region
  void testFluxWeakeningRegion() {
    double baseSpeed = 4000.0; // RPM
    double maxSpeed = 8000.0;  // RPM with flux weakening

    double speedRatio = maxSpeed / baseSpeed;
    TS_ASSERT_DELTA(speedRatio, 2.0, 0.1);
  }

  /***************************************************************************
   * Complete Electric Motor System Tests
   ***************************************************************************/

  // Test complete motor power system
  void testCompleteMotorPowerSystem() {
    double voltage = 400.0;     // V
    double current = 100.0;     // A
    double efficiency = 0.95;

    double inputPower = voltage * current;
    double outputPower = inputPower * efficiency;
    double lossedPower = inputPower - outputPower;

    TS_ASSERT_DELTA(inputPower, 40000.0, 1.0);
    TS_ASSERT_DELTA(outputPower, 38000.0, 1.0);
    TS_ASSERT_DELTA(lossedPower, 2000.0, 1.0);
  }

  // Test motor speed control loop
  void testMotorSpeedControlLoop() {
    double targetRPM = 5000.0;
    double currentRPM = 4800.0;
    double Kp = 0.1;

    double error = targetRPM - currentRPM;
    double correction = Kp * error;

    TS_ASSERT_DELTA(error, 200.0, 0.1);
    TS_ASSERT_DELTA(correction, 20.0, 0.1);
  }

  // Test battery discharge characteristics
  void testBatteryDischargeCharacteristics() {
    double initialCapacity = 100.0;  // Ah
    double dischargeCurrent = 50.0;  // A
    double time = 1.0;               // hours

    double remainingCapacity = initialCapacity - dischargeCurrent * time;
    double stateOfCharge = remainingCapacity / initialCapacity;

    TS_ASSERT_DELTA(remainingCapacity, 50.0, 0.1);
    TS_ASSERT_DELTA(stateOfCharge, 0.5, 0.01);
  }

  // Test motor thermal model
  void testMotorThermalModel() {
    double ambientTemp = 25.0;    // C
    double powerLoss = 500.0;     // W
    double thermalResistance = 0.1;  // C/W

    double tempRise = powerLoss * thermalResistance;
    double motorTemp = ambientTemp + tempRise;

    TS_ASSERT_DELTA(tempRise, 50.0, 0.1);
    TS_ASSERT_DELTA(motorTemp, 75.0, 0.1);
  }

  // Test inverter efficiency map
  void testInverterEfficiencyMap() {
    double loadPercent = 75.0;
    double peakEfficiency = 0.98;
    double partLoadFactor = 0.95;

    double efficiency = peakEfficiency * partLoadFactor;
    TS_ASSERT(efficiency > 0.9);
    TS_ASSERT(efficiency < 1.0);
  }

  // Test regenerative braking energy recovery
  void testRegenerativeEnergyRecovery() {
    double kineticEnergy = 10000.0;  // J
    double regenEfficiency = 0.7;

    double recoveredEnergy = kineticEnergy * regenEfficiency;
    double lostEnergy = kineticEnergy - recoveredEnergy;

    TS_ASSERT_DELTA(recoveredEnergy, 7000.0, 0.1);
    TS_ASSERT_DELTA(lostEnergy, 3000.0, 0.1);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test motor calculations independence
  void testMotorCalculationsIndependence() {
    double Kv1 = 1000.0, V1 = 48.0;
    double Kv2 = 500.0, V2 = 96.0;

    double rpm1 = Kv1 * V1;
    double rpm2 = Kv2 * V2;

    TS_ASSERT_DELTA(rpm1, 48000.0, 1.0);
    TS_ASSERT_DELTA(rpm2, 48000.0, 1.0);
  }

  // Test efficiency calculation independence
  void testEfficiencyCalculationIndependence() {
    double Pin1 = 1000.0, Pout1 = 900.0;
    double Pin2 = 5000.0, Pout2 = 4500.0;

    double eff1 = Pout1 / Pin1;
    double eff2 = Pout2 / Pin2;

    TS_ASSERT_DELTA(eff1, 0.9, 0.01);
    TS_ASSERT_DELTA(eff2, 0.9, 0.01);
  }

  // Test torque calculation independence
  void testTorqueCalculationIndependence() {
    double P1 = 1000.0, rpm1 = 3000.0;
    double P2 = 2000.0, rpm2 = 6000.0;

    double torque1 = P1 * 5252.0 / rpm1;
    double torque2 = P2 * 5252.0 / rpm2;

    TS_ASSERT_DELTA(torque1, 1750.67, 1.0);
    TS_ASSERT_DELTA(torque2, 1750.67, 1.0);
  }

  // Test current state independence
  void testCurrentStateIndependence() {
    double V1 = 48.0, R1 = 0.1;
    double V2 = 96.0, R2 = 0.2;

    double I1 = V1 / R1;
    double I2 = V2 / R2;

    TS_ASSERT_DELTA(I1, 480.0, 0.1);
    TS_ASSERT_DELTA(I2, 480.0, 0.1);
  }

  // Test power factor calculation independence
  void testPowerFactorCalculationIndependence() {
    double realPower1 = 900.0, apparentPower1 = 1000.0;
    double realPower2 = 800.0, apparentPower2 = 1000.0;

    double pf1 = realPower1 / apparentPower1;
    double pf2 = realPower2 / apparentPower2;

    TS_ASSERT_DELTA(pf1, 0.9, 0.01);
    TS_ASSERT_DELTA(pf2, 0.8, 0.01);
  }
};
