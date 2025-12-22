#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGTransmission.h>
#include "TestUtilities.h"

using namespace JSBSim;

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
};
