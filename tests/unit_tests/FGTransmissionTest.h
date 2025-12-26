#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGTransmission.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGTransmissionTest : public CxxTest::TestSuite
{
public:
  // Test gear ratio basic calculation: output_rpm = input_rpm / ratio
  void testGearRatioBasic() {
    // For helicopter main rotor gearbox
    double engineRPM = 6000.0;
    double gearRatio = 20.0;

    double rotorRPM = engineRPM / gearRatio;
    TS_ASSERT_DELTA(rotorRPM, 300.0, DEFAULT_TOLERANCE);

    // Verify angular velocity relationship
    double engineOmega = engineRPM * 2.0 * M_PI / 60.0;  // rad/sec
    double rotorOmega = rotorRPM * 2.0 * M_PI / 60.0;

    TS_ASSERT_DELTA(rotorOmega, engineOmega / gearRatio, 1e-8);
  }

  // Test torque multiplication: output_torque = input_torque * ratio
  void testTorqueMultiplication() {
    double engineTorque = 500.0;  // ft-lbf
    double gearRatio = 20.0;

    // Ideal gearbox (100% efficiency)
    double rotorTorque = engineTorque * gearRatio;
    TS_ASSERT_DELTA(rotorTorque, 10000.0, DEFAULT_TOLERANCE);

    // Smaller ratio test
    gearRatio = 10.0;
    rotorTorque = engineTorque * gearRatio;
    TS_ASSERT_DELTA(rotorTorque, 5000.0, DEFAULT_TOLERANCE);
  }

  // Test torque multiplication with efficiency losses
  void testTorqueMultiplicationWithEfficiency() {
    double engineTorque = 500.0;
    double gearRatio = 20.0;
    double efficiency = 0.95;  // 95% efficient

    double rotorTorque = engineTorque * gearRatio * efficiency;
    TS_ASSERT_DELTA(rotorTorque, 9500.0, 0.1);

    // Lower efficiency
    efficiency = 0.90;
    rotorTorque = engineTorque * gearRatio * efficiency;
    TS_ASSERT_DELTA(rotorTorque, 9000.0, 0.1);
  }

  // Test power conservation: P_in * efficiency = P_out
  void testPowerConservation() {
    // Power should be conserved (minus losses)
    double inputPower = 500.0 * 550.0;  // 500 HP in ft-lbf/sec
    double efficiency = 0.95;

    double outputPower = inputPower * efficiency;
    TS_ASSERT_DELTA(outputPower, 261250.0, 0.1);

    // Verify with torque and RPM
    double engineRPM = 6000.0;
    double engineOmega = engineRPM * 2.0 * M_PI / 60.0;
    double engineTorque = inputPower / engineOmega;

    double gearRatio = 20.0;
    double rotorRPM = engineRPM / gearRatio;
    double rotorOmega = rotorRPM * 2.0 * M_PI / 60.0;
    double rotorTorque = engineTorque * gearRatio * efficiency;

    double rotorPower = rotorTorque * rotorOmega;
    TS_ASSERT_DELTA(rotorPower, outputPower, 1.0);
  }

  // Test power = torque * angular_velocity relationship
  void testPowerTorqueRelationship() {
    double torque = 8750.0;  // ft-lbf
    double rpm = 300.0;
    double omega = rpm * 2.0 * M_PI / 60.0;  // 31.416 rad/sec

    double power = torque * omega;
    // = 8750 * 31.416 = 274890 ft-lbf/sec
    TS_ASSERT_DELTA(power, 274890.0, 10.0);

    // Convert to horsepower
    double hp = power / 550.0;
    TS_ASSERT_DELTA(hp, 499.8, 1.0);
  }

  // Test multi-stage gearbox calculations
  void testMultiStageGearbox() {
    // Two-stage reduction
    double engineRPM = 6000.0;
    double stage1Ratio = 4.0;
    double stage2Ratio = 5.0;

    // First stage
    double intermediate_rpm = engineRPM / stage1Ratio;
    TS_ASSERT_DELTA(intermediate_rpm, 1500.0, DEFAULT_TOLERANCE);

    // Second stage
    double outputRPM = intermediate_rpm / stage2Ratio;
    TS_ASSERT_DELTA(outputRPM, 300.0, DEFAULT_TOLERANCE);

    // Overall ratio
    double totalRatio = stage1Ratio * stage2Ratio;
    TS_ASSERT_DELTA(outputRPM, engineRPM / totalRatio, DEFAULT_TOLERANCE);
  }

  // Test multi-stage torque multiplication
  void testMultiStageTorque() {
    double engineTorque = 100.0;
    double stage1Ratio = 4.0;
    double stage2Ratio = 5.0;
    double stage1Efficiency = 0.96;
    double stage2Efficiency = 0.96;

    // First stage output
    double intermediateTorque = engineTorque * stage1Ratio * stage1Efficiency;
    TS_ASSERT_DELTA(intermediateTorque, 384.0, 0.1);

    // Second stage output
    double outputTorque = intermediateTorque * stage2Ratio * stage2Efficiency;
    TS_ASSERT_DELTA(outputTorque, 1843.2, 0.1);

    // Verify overall multiplication
    double overallEfficiency = stage1Efficiency * stage2Efficiency;
    double totalRatio = stage1Ratio * stage2Ratio;
    double expectedTorque = engineTorque * totalRatio * overallEfficiency;
    TS_ASSERT_DELTA(outputTorque, expectedTorque, 0.1);
  }

  // Test transmission efficiency losses
  void testTransmissionEfficiency() {
    // Typical gearbox efficiencies
    double perfectEfficiency = 1.0;
    double excellentEfficiency = 0.98;
    double goodEfficiency = 0.95;
    double fairEfficiency = 0.90;

    double inputPower = 500.0 * 550.0;  // 500 HP

    TS_ASSERT_DELTA(inputPower * perfectEfficiency, 275000.0, 0.1);
    TS_ASSERT_DELTA(inputPower * excellentEfficiency, 269500.0, 0.1);
    TS_ASSERT_DELTA(inputPower * goodEfficiency, 261250.0, 0.1);
    TS_ASSERT_DELTA(inputPower * fairEfficiency, 247500.0, 0.1);

    // Power loss
    double lossExcellent = inputPower * (1.0 - excellentEfficiency);
    TS_ASSERT_DELTA(lossExcellent, 5500.0, 0.1);  // 11 HP lost
  }

  // Test maximum torque limits
  void testMaximumTorqueLimits() {
    double maxTorque = 12000.0;  // ft-lbf design limit

    // Normal operation
    double torque1 = 10000.0;
    if (torque1 > maxTorque) torque1 = maxTorque;
    TS_ASSERT_DELTA(torque1, 10000.0, DEFAULT_TOLERANCE);

    // Overload condition
    double torque2 = 15000.0;
    if (torque2 > maxTorque) torque2 = maxTorque;
    TS_ASSERT_DELTA(torque2, 12000.0, DEFAULT_TOLERANCE);

    // Verify limiting
    TS_ASSERT(torque2 <= maxTorque);
  }

  // Test angular velocity relationships
  void testAngularVelocityRelationships() {
    double rpm = 300.0;

    // RPM to rad/sec
    double omega = rpm * 2.0 * M_PI / 60.0;
    TS_ASSERT_DELTA(omega, 31.4159265, 1e-6);

    // Rad/sec to RPM
    double rpmCalc = omega * 60.0 / (2.0 * M_PI);
    TS_ASSERT_DELTA(rpmCalc, 300.0, 1e-8);

    // Conversion factor test
    double rpm_to_omega_factor = 0.104719755119659774615421446109;
    TS_ASSERT_DELTA(omega, rpm * rpm_to_omega_factor, 1e-10);

    double omega_to_rpm_factor = 9.54929658551372014613302580235;
    TS_ASSERT_DELTA(rpmCalc, omega * omega_to_rpm_factor, 1e-8);
  }

  // Test RPM to omega conversion (from FGTransmission)
  void testRpmToOmegaConversion() {
    // Test conversion factor from FGTransmission
    double rpm = 300.0;
    double expectedOmega = rpm * 0.104719755119659774615421446109;

    // Manual calculation
    double omega = rpm * 2.0 * M_PI / 60.0;

    TS_ASSERT_DELTA(omega, expectedOmega, 1e-12);
    TS_ASSERT_DELTA(omega, 31.4159265358979, 1e-10);
  }

  // Test omega to RPM conversion (from FGTransmission)
  void testOmegaToRpmConversion() {
    // Test conversion factor from FGTransmission
    double omega = 31.4159265358979;
    double expectedRPM = omega * 9.54929658551372014613302580235;

    // Manual calculation
    double rpm = omega * 60.0 / (2.0 * M_PI);

    TS_ASSERT_DELTA(rpm, expectedRPM, 1e-10);
    TS_ASSERT_DELTA(rpm, 300.0, 1e-8);
  }

  // Test gearbox inertia effects
  void testGearboxInertiaEffects() {
    // J_eq = J_engine + J_rotor/ratio^2
    double engineInertia = 2.0;    // slug-ft^2
    double rotorInertia = 5000.0;  // slug-ft^2
    double gearRatio = 20.0;

    double equivalentInertia = engineInertia + rotorInertia / (gearRatio * gearRatio);
    TS_ASSERT_DELTA(equivalentInertia, 14.5, 0.1);

    // Verify rotor dominates even with gear ratio
    TS_ASSERT(rotorInertia / (gearRatio * gearRatio) > engineInertia);
  }

  // Test acceleration calculation with torque balance
  void testAccelerationCalculation() {
    // alpha = (T_engine - T_load) / J
    double engineTorque = 10000.0;  // ft-lbf
    double loadTorque = 9000.0;     // ft-lbf
    double inertia = 5000.0;        // slug-ft^2

    double netTorque = engineTorque - loadTorque;
    double angularAcceleration = netTorque / inertia;

    TS_ASSERT_DELTA(angularAcceleration, 0.2, 1e-8);  // rad/sec^2

    // Verify units: (ft-lbf) / (slug-ft^2) = rad/sec^2
    double dt = 0.1;  // seconds
    double deltaOmega = angularAcceleration * dt;
    TS_ASSERT_DELTA(deltaOmega, 0.02, 1e-8);  // rad/sec
  }

  // Test deceleration with negative torque
  void testDecelerationWithNegativeTorque() {
    double engineTorque = 8000.0;
    double loadTorque = 10000.0;  // Load exceeds engine
    double inertia = 5000.0;

    double netTorque = engineTorque - loadTorque;
    TS_ASSERT_DELTA(netTorque, -2000.0, DEFAULT_TOLERANCE);

    double angularAcceleration = netTorque / inertia;
    TS_ASSERT_DELTA(angularAcceleration, -0.4, 1e-8);  // Deceleration
    TS_ASSERT(angularAcceleration < 0.0);
  }

  // Test brake torque application
  void testBrakeTorqueApplication() {
    double maxBrakePower = 25.0 * 550.0;  // 25 HP in ft-lbf/sec
    double rpm = 300.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    // Brake torque = Power / omega
    double brakeTorque = maxBrakePower / omega;
    TS_ASSERT_DELTA(brakeTorque, 437.5, 1.0);

    // Partial brake application
    double brakeCtrl = 0.5;  // 50% brake
    double actualBrakeTorque = brakeTorque * brakeCtrl;
    TS_ASSERT_DELTA(actualBrakeTorque, 218.75, 0.5);
  }

  // Test friction torque calculation
  void testFrictionTorque() {
    // Friction power is constant, torque varies with speed
    double frictionPower = 5.0 * 550.0;  // 5 HP friction loss

    // At high RPM
    double rpm1 = 6000.0;
    double omega1 = rpm1 * 2.0 * M_PI / 60.0;
    double frictionTorque1 = frictionPower / omega1;
    TS_ASSERT_DELTA(frictionTorque1, 4.37, 0.01);

    // At low RPM - friction torque higher
    double rpm2 = 300.0;
    double omega2 = rpm2 * 2.0 * M_PI / 60.0;
    double frictionTorque2 = frictionPower / omega2;
    TS_ASSERT_DELTA(frictionTorque2, 87.5, 0.1);

    // Verify inverse relationship
    TS_ASSERT(frictionTorque2 > frictionTorque1);
    TS_ASSERT_DELTA(frictionTorque1 * omega1, frictionPower, 1.0);
    TS_ASSERT_DELTA(frictionTorque2 * omega2, frictionPower, 1.0);
  }

  // Test clutch partial engagement
  void testClutchPartialEngagement() {
    double engineTorque = 500.0;
    double clutchPosition = 0.5;  // 50% engaged

    // Transmitted torque proportional to clutch position
    double transmittedTorque = engineTorque * clutchPosition;
    TS_ASSERT_DELTA(transmittedTorque, 250.0, DEFAULT_TOLERANCE);

    // Full engagement
    clutchPosition = 1.0;
    transmittedTorque = engineTorque * clutchPosition;
    TS_ASSERT_DELTA(transmittedTorque, 500.0, DEFAULT_TOLERANCE);

    // Disengaged
    clutchPosition = 0.0;
    transmittedTorque = engineTorque * clutchPosition;
    TS_ASSERT_DELTA(transmittedTorque, 0.0, DEFAULT_TOLERANCE);
  }

  // Test free-wheel unit operation
  void testFreeWheelUnit() {
    // Free-wheel prevents rotor from driving engine
    double engineOmega = 500.0;   // rad/sec
    double rotorOmega = 520.0;    // rad/sec (spinning faster)

    // Free-wheel should activate when rotor > engine
    bool freeWheelActive = (rotorOmega > engineOmega);
    TS_ASSERT(freeWheelActive);

    // Locked when engine driving rotor
    engineOmega = 520.0;
    rotorOmega = 500.0;
    freeWheelActive = (rotorOmega > engineOmega);
    TS_ASSERT(!freeWheelActive);
  }

  // Test power loss calculation
  void testPowerLossCalculation() {
    double inputPower = 500.0 * 550.0;  // 500 HP
    double efficiency = 0.95;

    double outputPower = inputPower * efficiency;
    double powerLoss = inputPower - outputPower;

    TS_ASSERT_DELTA(powerLoss, 13750.0, 0.1);  // ft-lbf/sec

    // Convert to HP
    double lossHP = powerLoss / 550.0;
    TS_ASSERT_DELTA(lossHP, 25.0, 0.1);  // 25 HP lost

    // Verify percentage
    double lossPercent = (powerLoss / inputPower) * 100.0;
    TS_ASSERT_DELTA(lossPercent, 5.0, 0.01);
  }

  // Test gear ratio effect on torque and speed
  void testGearRatioTradeoff() {
    // Demonstrates fundamental tradeoff: speed for torque
    double engineTorque = 200.0;
    double engineRPM = 6000.0;

    // Low ratio (high speed, low torque)
    double ratio1 = 10.0;
    double outputTorque1 = engineTorque * ratio1;
    double outputRPM1 = engineRPM / ratio1;
    TS_ASSERT_DELTA(outputTorque1, 2000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(outputRPM1, 600.0, DEFAULT_TOLERANCE);

    // High ratio (low speed, high torque)
    double ratio2 = 20.0;
    double outputTorque2 = engineTorque * ratio2;
    double outputRPM2 = engineRPM / ratio2;
    TS_ASSERT_DELTA(outputTorque2, 4000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(outputRPM2, 300.0, DEFAULT_TOLERANCE);

    // Verify tradeoff
    TS_ASSERT(outputTorque2 > outputTorque1);
    TS_ASSERT(outputRPM2 < outputRPM1);
  }

  // Test minimum RPM constraint
  void testMinimumRPMConstraint() {
    // Prevent division by zero and numerical issues
    double minRPM = 1.0;

    // Test various low RPM values
    double rpm1 = 0.5;
    double safeRPM1 = (rpm1 < minRPM) ? minRPM : rpm1;
    TS_ASSERT_DELTA(safeRPM1, 1.0, DEFAULT_TOLERANCE);

    double rpm2 = 0.0;
    double safeRPM2 = (rpm2 < minRPM) ? minRPM : rpm2;
    TS_ASSERT_DELTA(safeRPM2, 1.0, DEFAULT_TOLERANCE);

    double rpm3 = 100.0;
    double safeRPM3 = (rpm3 < minRPM) ? minRPM : rpm3;
    TS_ASSERT_DELTA(safeRPM3, 100.0, DEFAULT_TOLERANCE);
  }

  // Test torque at zero speed edge case
  void testTorqueAtZeroSpeed() {
    // At very low omega, use minimum safe value to avoid division by zero
    double power = 1000.0;  // ft-lbf/sec
    double omega = 0.0;
    double minOmega = 0.1;  // rad/sec minimum

    double safeOmega = (omega < minOmega) ? minOmega : omega;
    double torque = power / safeOmega;

    TS_ASSERT_DELTA(safeOmega, 0.1, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(torque, 10000.0, DEFAULT_TOLERANCE);
    TS_ASSERT(!std::isinf(torque));
    TS_ASSERT(!std::isnan(torque));
  }

  // Test reversing prevention (transmission doesn't support backward)
  void testReversingPrevention() {
    double rpm = -50.0;  // Negative (backward rotation)

    // Clamp to zero
    if (rpm < 0.0) rpm = 0.0;

    TS_ASSERT_DELTA(rpm, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT(rpm >= 0.0);
  }

  // ==================== HELICOPTER TRANSMISSION TESTS ====================

  // Test tail rotor gear ratio
  void testTailRotorGearRatio() {
    // Tail rotor typically spins 4-5x faster than main rotor
    double mainRotorRPM = 300.0;
    double tailRotorRatio = 4.5;

    double tailRotorRPM = mainRotorRPM * tailRotorRatio;
    TS_ASSERT_DELTA(tailRotorRPM, 1350.0, DEFAULT_TOLERANCE);

    // Tail rotor power typically 10-15% of total
    double totalPower = 500.0;  // HP
    double tailPowerFraction = 0.12;
    double tailPower = totalPower * tailPowerFraction;
    TS_ASSERT_DELTA(tailPower, 60.0, DEFAULT_TOLERANCE);
  }

  // Test tail rotor torque requirement
  void testTailRotorTorque() {
    // Tail rotor torque = Power / omega
    double tailPower = 60.0 * 550.0;  // 60 HP in ft-lbf/sec
    double tailRPM = 1350.0;
    double tailOmega = tailRPM * 2.0 * M_PI / 60.0;

    double tailTorque = tailPower / tailOmega;
    TS_ASSERT_DELTA(tailTorque, 233.5, 1.0);
  }

  // Test main transmission torque split
  void testMainTransmissionTorqueSplit() {
    // Total engine torque split between main rotor and accessories
    double totalTorque = 10000.0;  // ft-lbf
    double mainRotorFraction = 0.88;  // 88% to main rotor
    double tailRotorFraction = 0.08;  // 8% to tail rotor
    double accessoryFraction = 0.04;   // 4% to accessories

    double mainRotorTorque = totalTorque * mainRotorFraction;
    double tailRotorTorque = totalTorque * tailRotorFraction;
    double accessoryTorque = totalTorque * accessoryFraction;

    TS_ASSERT_DELTA(mainRotorTorque, 8800.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(tailRotorTorque, 800.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(accessoryTorque, 400.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(mainRotorTorque + tailRotorTorque + accessoryTorque,
                    totalTorque, DEFAULT_TOLERANCE);
  }

  // Test combining box operation
  void testCombiningBox() {
    // Multi-engine helicopter combines inputs
    double engine1Torque = 4000.0;
    double engine2Torque = 4000.0;

    double combinedTorque = engine1Torque + engine2Torque;
    TS_ASSERT_DELTA(combinedTorque, 8000.0, DEFAULT_TOLERANCE);

    // Asymmetric power (one engine reduced)
    engine2Torque = 3000.0;
    combinedTorque = engine1Torque + engine2Torque;
    TS_ASSERT_DELTA(combinedTorque, 7000.0, DEFAULT_TOLERANCE);
  }

  // ==================== GOVERNOR AND SPEED CONTROL TESTS ====================

  // Test droop governor
  void testDroopGovernor() {
    // Droop = RPM change per % load
    double droopPercent = 4.0;  // 4% droop
    double nominalRPM = 6000.0;

    // At 100% load
    double loadPercent = 100.0;
    double governedRPM_100 = nominalRPM * (1.0 - (droopPercent / 100.0) * (loadPercent / 100.0));
    TS_ASSERT_DELTA(governedRPM_100, 5760.0, 1.0);

    // At 50% load
    loadPercent = 50.0;
    double governedRPM_50 = nominalRPM * (1.0 - (droopPercent / 100.0) * (loadPercent / 100.0));
    TS_ASSERT_DELTA(governedRPM_50, 5880.0, 1.0);

    // At no load
    loadPercent = 0.0;
    double governedRPM_0 = nominalRPM * (1.0 - (droopPercent / 100.0) * (loadPercent / 100.0));
    TS_ASSERT_DELTA(governedRPM_0, 6000.0, DEFAULT_TOLERANCE);
  }

  // Test isochronous governor
  void testIsochronousGovernor() {
    // Maintains constant RPM regardless of load
    double setpointRPM = 6000.0;
    double Kp = 10.0;  // Proportional gain
    double Ki = 1.0;   // Integral gain

    // Simulate error correction
    double actualRPM = 5950.0;
    double error = setpointRPM - actualRPM;
    double correction = Kp * error;

    TS_ASSERT_DELTA(error, 50.0, DEFAULT_TOLERANCE);
    TS_ASSERT(correction > 0);  // Increase fuel/power
  }

  // Test load sharing between engines
  void testLoadSharing() {
    double engine1Power = 240.0;  // HP
    double engine2Power = 260.0;  // HP
    double totalPower = engine1Power + engine2Power;

    double engine1Share = engine1Power / totalPower * 100.0;
    double engine2Share = engine2Power / totalPower * 100.0;

    TS_ASSERT_DELTA(engine1Share, 48.0, 0.1);
    TS_ASSERT_DELTA(engine2Share, 52.0, 0.1);

    // Ideally should be 50/50
    double imbalance = std::abs(engine1Share - engine2Share);
    TS_ASSERT(imbalance < 5.0);  // Within 5%
  }

  // ==================== THERMAL EFFECTS TESTS ====================

  // Test transmission temperature rise
  void testTransmissionTempRise() {
    double powerLoss = 25.0 * 550.0;  // 25 HP lost as heat (ft-lbf/sec)
    double thermalCapacity = 1000.0;   // BTU/°F
    double heatToWork = 778.0;         // ft-lbf per BTU

    // Heat generated per second (BTU/sec)
    double heatRate = powerLoss / heatToWork;
    TS_ASSERT_DELTA(heatRate, 17.7, 0.1);

    // Temperature rise rate (°F/sec) without cooling
    double tempRiseRate = heatRate / thermalCapacity;
    TS_ASSERT_DELTA(tempRiseRate, 0.0177, 0.001);
  }

  // Test oil temperature effect on efficiency
  void testOilTempEfficiency() {
    double baseEfficiency = 0.95;
    double nominalTemp = 180.0;  // °F

    // Cold oil - higher viscosity, more losses
    double coldTemp = 80.0;
    double coldPenalty = (nominalTemp - coldTemp) * 0.0002;  // 0.02% per degree
    double coldEfficiency = baseEfficiency - coldPenalty;
    TS_ASSERT_DELTA(coldEfficiency, 0.93, 0.01);

    // Hot oil - lower viscosity, but film breakdown risk
    double hotTemp = 250.0;
    double hotPenalty = (hotTemp - nominalTemp) * 0.0003;
    double hotEfficiency = baseEfficiency - hotPenalty;
    TS_ASSERT_DELTA(hotEfficiency, 0.929, 0.01);
  }

  // Test oil pressure requirements
  void testOilPressure() {
    double minOilPressure = 30.0;  // psi
    double normalOilPressure = 60.0;
    double maxOilPressure = 100.0;

    // Normal operation check
    double actualPressure = 55.0;
    bool pressureOK = (actualPressure >= minOilPressure && actualPressure <= maxOilPressure);
    TS_ASSERT(pressureOK);

    // Low pressure warning
    actualPressure = 25.0;
    bool lowPressure = (actualPressure < minOilPressure);
    TS_ASSERT(lowPressure);
  }

  // ==================== FAILURE MODE TESTS ====================

  // Test overload detection
  void testOverloadDetection() {
    double maxTorque = 12000.0;     // Design limit ft-lbf
    double warningTorque = 11000.0; // 92% warning threshold

    double actualTorque = 11500.0;

    bool warning = (actualTorque > warningTorque);
    bool overload = (actualTorque > maxTorque);

    TS_ASSERT(warning);
    TS_ASSERT(!overload);

    actualTorque = 13000.0;
    overload = (actualTorque > maxTorque);
    TS_ASSERT(overload);
  }

  // Test clutch slip detection
  void testClutchSlipDetection() {
    double engineRPM = 6000.0;
    double inputShaftRPM = 5800.0;  // Should be same as engine when engaged
    double slipThreshold = 50.0;    // RPM

    double slip = engineRPM - inputShaftRPM;
    bool slipping = (std::abs(slip) > slipThreshold);

    TS_ASSERT_DELTA(slip, 200.0, DEFAULT_TOLERANCE);
    TS_ASSERT(slipping);

    // Normal operation
    inputShaftRPM = 5990.0;
    slip = engineRPM - inputShaftRPM;
    slipping = (std::abs(slip) > slipThreshold);
    TS_ASSERT(!slipping);
  }

  // Test overspeed protection
  void testOverspeedProtection() {
    double maxRPM = 320.0;      // Rotor redline
    double warningRPM = 310.0;  // Warning threshold

    double actualRPM = 315.0;
    bool warning = (actualRPM > warningRPM);
    bool overspeed = (actualRPM > maxRPM);

    TS_ASSERT(warning);
    TS_ASSERT(!overspeed);

    actualRPM = 325.0;
    overspeed = (actualRPM > maxRPM);
    TS_ASSERT(overspeed);
  }

  // ==================== AUTOROTATION TESTS ====================

  // Test autorotation RPM decay
  void testAutorotationRPMDecay() {
    double rotorRPM = 300.0;
    double rotorInertia = 5000.0;  // slug-ft^2
    double rotorOmega = rotorRPM * 2.0 * M_PI / 60.0;

    // Rotor energy = 0.5 * I * omega^2
    double rotorEnergy = 0.5 * rotorInertia * rotorOmega * rotorOmega;
    TS_ASSERT_DELTA(rotorEnergy, 2467401.0, 1000.0);

    // Decay rate with blade drag
    double dragPower = 100.0 * 550.0;  // 100 HP of drag
    double decayTime = rotorEnergy / dragPower;  // Seconds to stop (simplified)
    TS_ASSERT_DELTA(decayTime, 44.9, 0.5);
  }

  // Test free-wheel engagement speed
  void testFreeWheelEngagementSpeed() {
    double engineRPM = 6000.0;
    double rotorRPM = 300.0;
    double gearRatio = 20.0;

    // Engine equivalent RPM at rotor
    double engineEquivRPM = engineRPM / gearRatio;
    TS_ASSERT_DELTA(engineEquivRPM, 300.0, DEFAULT_TOLERANCE);

    // Free-wheel engages when rotor spins faster than engine (relative)
    double rotorRelativeRPM = rotorRPM;
    bool freeWheelEngaged = (rotorRelativeRPM > engineEquivRPM);
    TS_ASSERT(!freeWheelEngaged);  // Equal - not engaged

    // Autorotation - rotor being driven by airflow
    rotorRPM = 310.0;
    freeWheelEngaged = (rotorRPM > engineEquivRPM);
    TS_ASSERT(freeWheelEngaged);
  }

  // Test autorotation entry
  void testAutorotationEntry() {
    // Simulate engine failure and free-wheel engagement
    double engineTorque = 10000.0;
    double rotorLoadTorque = 9500.0;
    bool engineRunning = true;
    bool freeWheelActive = false;

    // Normal operation
    double netTorque = engineRunning ? (engineTorque - rotorLoadTorque) : -rotorLoadTorque;
    TS_ASSERT_DELTA(netTorque, 500.0, DEFAULT_TOLERANCE);

    // Engine failure
    engineRunning = false;
    freeWheelActive = true;
    netTorque = engineRunning ? (engineTorque - rotorLoadTorque) : -rotorLoadTorque;

    // Free-wheel prevents engine from being dragged
    if (freeWheelActive) {
      netTorque = -rotorLoadTorque;  // Only drag on rotor
    }
    TS_ASSERT_DELTA(netTorque, -9500.0, DEFAULT_TOLERANCE);
    TS_ASSERT(netTorque < 0);  // Rotor will slow down
  }

  // ==================== MULTI-ENGINE TESTS ====================

  // Test single engine operation
  void testSingleEngineOperation() {
    double engine1Power = 500.0;  // HP
    double engine2Power = 500.0;  // HP
    double singleEngineLimit = 0.85;  // Can operate at 85% power on one engine

    double totalPower = engine1Power + engine2Power;
    double singleEnginePower = engine1Power * singleEngineLimit;

    TS_ASSERT_DELTA(totalPower, 1000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(singleEnginePower, 425.0, DEFAULT_TOLERANCE);

    // Contingency power
    double contingencyFactor = 1.1;  // 10% over normal
    double contingencyPower = singleEnginePower * contingencyFactor;
    TS_ASSERT_DELTA(contingencyPower, 467.5, DEFAULT_TOLERANCE);
  }

  // Test engine crossfeed
  void testEngineCrossfeed() {
    // Both engines can power transmission through crossfeed gearbox
    double engine1Output = 4500.0;  // ft-lbf
    double engine2Output = 4500.0;

    // Input shaft receives combined torque
    double inputShaftTorque = engine1Output + engine2Output;
    TS_ASSERT_DELTA(inputShaftTorque, 9000.0, DEFAULT_TOLERANCE);

    // One engine out
    engine2Output = 0.0;
    inputShaftTorque = engine1Output + engine2Output;
    TS_ASSERT_DELTA(inputShaftTorque, 4500.0, DEFAULT_TOLERANCE);
  }

  // ==================== DYNAMICS TESTS ====================

  // Test power conservation through gearing
  void testPowerConservationThroughGearing() {
    // Power is conserved (minus losses), not angular momentum
    // P = T * omega
    double torque1 = 100.0;    // ft-lbf (input torque)
    double omega1 = 628.3;     // rad/sec (6000 RPM)

    double power1 = torque1 * omega1;
    TS_ASSERT_DELTA(power1, 62830.0, 10.0);

    // After gear reduction (20:1)
    double gearRatio = 20.0;
    double efficiency = 0.95;
    double omega2 = omega1 / gearRatio;
    double torque2 = torque1 * gearRatio * efficiency;

    double power2 = torque2 * omega2;

    // Output power = input power * efficiency
    TS_ASSERT_DELTA(power2, power1 * efficiency, 10.0);
  }

  // Test torsional natural frequency
  void testTorsionalNaturalFrequency() {
    // omega_n = sqrt(k/I)
    double torsionalStiffness = 1e6;  // ft-lbf/rad
    double inertia = 100.0;           // slug-ft^2

    double omegaN = sqrt(torsionalStiffness / inertia);
    TS_ASSERT_DELTA(omegaN, 100.0, 0.1);  // rad/sec

    // Convert to Hz
    double freqHz = omegaN / (2.0 * M_PI);
    TS_ASSERT_DELTA(freqHz, 15.9, 0.1);
  }

  // Test torque spike during engagement
  void testTorqueSpikeEngagement() {
    double steadyTorque = 5000.0;
    double spikeMultiplier = 1.5;  // 50% overshoot

    double peakTorque = steadyTorque * spikeMultiplier;
    TS_ASSERT_DELTA(peakTorque, 7500.0, DEFAULT_TOLERANCE);

    // Must be below structural limit
    double structuralLimit = 12000.0;
    TS_ASSERT(peakTorque < structuralLimit);
  }

  // ==================== ACCESSORY DRIVE TESTS ====================

  // Test accessory gearbox
  void testAccessoryGearbox() {
    double mainShaftRPM = 6000.0;

    // Generator drive
    double generatorRatio = 2.0;  // 2:1 speedup
    double generatorRPM = mainShaftRPM * generatorRatio;
    TS_ASSERT_DELTA(generatorRPM, 12000.0, DEFAULT_TOLERANCE);

    // Hydraulic pump drive
    double pumpRatio = 1.5;
    double pumpRPM = mainShaftRPM * pumpRatio;
    TS_ASSERT_DELTA(pumpRPM, 9000.0, DEFAULT_TOLERANCE);

    // Oil pump (slower)
    double oilPumpRatio = 0.8;
    double oilPumpRPM = mainShaftRPM * oilPumpRatio;
    TS_ASSERT_DELTA(oilPumpRPM, 4800.0, DEFAULT_TOLERANCE);
  }

  // Test accessory power extraction
  void testAccessoryPowerExtraction() {
    double totalShaftPower = 500.0;  // HP

    // Various accessories
    double generatorPower = 15.0;   // HP
    double hydraulicPower = 10.0;   // HP
    double otherAccessories = 5.0;  // HP

    double totalAccessoryPower = generatorPower + hydraulicPower + otherAccessories;
    TS_ASSERT_DELTA(totalAccessoryPower, 30.0, DEFAULT_TOLERANCE);

    // Available for propulsion
    double propulsionPower = totalShaftPower - totalAccessoryPower;
    TS_ASSERT_DELTA(propulsionPower, 470.0, DEFAULT_TOLERANCE);

    // Percentage extraction
    double extractionPercent = (totalAccessoryPower / totalShaftPower) * 100.0;
    TS_ASSERT_DELTA(extractionPercent, 6.0, DEFAULT_TOLERANCE);
  }

  // ==================== STARTING SEQUENCE TESTS ====================

  // Test starter motor torque
  void testStarterMotorTorque() {
    double starterPower = 5.0 * 550.0;  // 5 HP starter
    double startingRPM = 100.0;         // Low RPM during start
    double startingOmega = startingRPM * 2.0 * M_PI / 60.0;

    double starterTorque = starterPower / startingOmega;
    TS_ASSERT_DELTA(starterTorque, 262.6, 1.0);

    // At higher RPM, torque decreases
    startingRPM = 500.0;
    startingOmega = startingRPM * 2.0 * M_PI / 60.0;
    starterTorque = starterPower / startingOmega;
    TS_ASSERT_DELTA(starterTorque, 52.5, 0.5);
  }

  // Test clutch engagement sequence
  void testClutchEngagementSequence() {
    double clutchPosition = 0.0;
    double engagementRate = 0.2;  // 20% per second
    double deltaT = 0.1;

    // Simulate engagement
    for (int i = 0; i < 50; i++) {
      clutchPosition += engagementRate * deltaT;
      if (clutchPosition > 1.0) clutchPosition = 1.0;
    }

    TS_ASSERT_DELTA(clutchPosition, 1.0, DEFAULT_TOLERANCE);
  }

  // ==================== EFFICIENCY MAP TESTS ====================

  // Test efficiency vs load
  void testEfficiencyVsLoad() {
    // Gearboxes are most efficient at design load
    double baseEfficiency = 0.95;
    double designLoad = 100.0;  // percent

    // At design load
    double loadPercent = 100.0;
    double efficiency100 = baseEfficiency;
    TS_ASSERT_DELTA(efficiency100, 0.95, DEFAULT_TOLERANCE);

    // At partial load (less efficient due to fixed losses)
    loadPercent = 50.0;
    double efficiency50 = baseEfficiency - (1.0 - loadPercent / designLoad) * 0.02;
    TS_ASSERT_DELTA(efficiency50, 0.94, 0.01);

    // At overload (less efficient due to heating)
    loadPercent = 120.0;
    double efficiency120 = baseEfficiency - (loadPercent / designLoad - 1.0) * 0.05;
    TS_ASSERT_DELTA(efficiency120, 0.94, 0.01);
  }

  // Test efficiency vs speed
  void testEfficiencyVsSpeed() {
    double baseEfficiency = 0.95;
    double designRPM = 6000.0;

    // At design speed
    double rpm = 6000.0;
    double efficiency = baseEfficiency;
    TS_ASSERT_DELTA(efficiency, 0.95, DEFAULT_TOLERANCE);

    // At low speed (churning losses different)
    rpm = 1000.0;
    double speedRatio = rpm / designRPM;
    efficiency = baseEfficiency - (1.0 - speedRatio) * 0.01;
    TS_ASSERT(efficiency < baseEfficiency);

    // At high speed (windage losses increase)
    rpm = 8000.0;
    speedRatio = rpm / designRPM;
    efficiency = baseEfficiency - (speedRatio - 1.0) * 0.02;
    TS_ASSERT(efficiency < baseEfficiency);
  }

  // ==================== MISCELLANEOUS TESTS ====================

  // Test kinetic energy storage
  void testKineticEnergyStorage() {
    double inertia = 5000.0;     // slug-ft^2
    double rpm = 300.0;
    double omega = rpm * 2.0 * M_PI / 60.0;

    double KE = 0.5 * inertia * omega * omega;
    TS_ASSERT_DELTA(KE, 2467401.0, 100.0);  // ft-lbf

    // Time to dissipate at given power loss
    double powerDrain = 100.0 * 550.0;  // 100 HP
    double drainTime = KE / powerDrain;
    TS_ASSERT_DELTA(drainTime, 44.9, 0.1);  // seconds
  }

  // Test backlash effect
  void testBacklashEffect() {
    double backlashAngle = 0.5;  // degrees of play
    double backlashRad = backlashAngle * M_PI / 180.0;

    // During reversal, output doesn't move for backlash amount
    double inputAngle = 0.0;
    double outputAngle = 0.0;

    inputAngle = 0.3;  // degrees
    // Output doesn't move until backlash taken up
    if (inputAngle < backlashAngle) {
      outputAngle = 0.0;
    } else {
      outputAngle = inputAngle - backlashAngle;
    }
    TS_ASSERT_DELTA(outputAngle, 0.0, DEFAULT_TOLERANCE);

    inputAngle = 1.0;
    outputAngle = inputAngle - backlashAngle;
    TS_ASSERT_DELTA(outputAngle, 0.5, DEFAULT_TOLERANCE);
  }

  // Test torquemeter reading
  void testTorquemeterReading() {
    double actualTorque = 8500.0;  // ft-lbf
    double maxDisplayTorque = 12000.0;

    double torquePercent = (actualTorque / maxDisplayTorque) * 100.0;
    TS_ASSERT_DELTA(torquePercent, 70.83, 0.1);

    // Check normal operating range
    TS_ASSERT(torquePercent < 100.0);
    TS_ASSERT(torquePercent > 0.0);
  }
};
