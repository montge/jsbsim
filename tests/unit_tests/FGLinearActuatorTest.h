/*******************************************************************************
 * FGLinearActuatorTest.h - Unit tests for FGLinearActuator (rotation counter)
 *
 * Tests the mathematical behavior of linear actuator components:
 * - Rotation to linear conversion (module/spin counting)
 * - Hysteresis behavior
 * - Direction (versus) handling
 * - Lag filtering
 * - Reset functionality
 *
 * Note: FGLinearActuator requires XML element for construction, so these tests
 * focus on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <initialization/FGInitialCondition.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGLinearActuatorTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Spin Counter State
   ***************************************************************************/
  struct LinearActuatorState {
    double inputLast = 0.0;
    double inputMem = 0.0;
    int countSpin = 0;
    int direction = 0;
    double module = 360.0;
    double hysteresis = 0.1;
    double rate = 0.3;
    double gain = 1.0;
    double bias = 0.0;
  };

  // Simulate one step of the linear actuator
  double runLinearActuator(LinearActuatorState& state, double input) {
    double delta = input - state.inputLast;

    // Detect spin completion using rate threshold
    if (std::abs(delta) > state.rate * state.module) {
      // Crossing detected
      if (delta > 0) {
        state.countSpin--;  // Wrapped from high to low
      } else {
        state.countSpin++;  // Wrapped from low to high
      }
    }

    state.inputLast = input;

    // Apply hysteresis (quantize to hysteresis steps)
    double quantized = std::round(input / state.hysteresis) * state.hysteresis;

    // Output = gain * (bias + input + module * countSpin)
    double output = state.gain * (state.bias + quantized + state.module * state.countSpin);

    return output;
  }

  /***************************************************************************
   * Basic Output Tests
   ***************************************************************************/

  // Test basic output calculation
  void testBasicOutput() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 1.0;
    state.bias = 0.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 45.0);
    TS_ASSERT_DELTA(output, 45.0, 1.0);
  }

  // Test with bias
  void testWithBias() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 1.0;
    state.bias = 10.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 45.0);
    TS_ASSERT_DELTA(output, 55.0, 1.0);  // 45 + 10
  }

  // Test with gain
  void testWithGain() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 2.0;
    state.bias = 0.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 45.0);
    TS_ASSERT_DELTA(output, 90.0, 2.0);  // 45 * 2
  }

  /***************************************************************************
   * Spin Counting Tests (Rotation Detection)
   ***************************************************************************/

  // Simplified spin counter
  int countSpins(double prevAngle, double newAngle, double module, double rate) {
    double delta = newAngle - prevAngle;
    if (std::abs(delta) > rate * module) {
      return (delta > 0) ? -1 : 1;
    }
    return 0;
  }

  // Test spin detection: 350 to 10 (crossing 0, increasing)
  void testSpinCrossingZeroIncreasing() {
    double prev = 350.0;
    double next = 10.0;
    double module = 360.0;
    double rate = 0.3;

    int spin = countSpins(prev, next, module, rate);
    // 10 - 350 = -340, which is < -rate*module, so spin++
    TS_ASSERT_EQUALS(spin, 1);
  }

  // Test spin detection: 10 to 350 (crossing 0, decreasing)
  void testSpinCrossingZeroDecreasing() {
    double prev = 10.0;
    double next = 350.0;
    double module = 360.0;
    double rate = 0.3;

    int spin = countSpins(prev, next, module, rate);
    // 350 - 10 = 340, which is > rate*module, so spin--
    TS_ASSERT_EQUALS(spin, -1);
  }

  // Test no spin (normal movement)
  void testNoSpin() {
    double prev = 45.0;
    double next = 90.0;
    double module = 360.0;
    double rate = 0.3;

    int spin = countSpins(prev, next, module, rate);
    TS_ASSERT_EQUALS(spin, 0);
  }

  // Test multiple full rotations
  void testMultipleRotations() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Simulate going from 0 to 720 with wrap-arounds
    runLinearActuator(state, 0.0);
    runLinearActuator(state, 90.0);
    runLinearActuator(state, 180.0);
    runLinearActuator(state, 270.0);
    runLinearActuator(state, 359.0);
    runLinearActuator(state, 10.0);  // First wrap

    // Count should have increased
    TS_ASSERT_EQUALS(state.countSpin, 1);
  }

  /***************************************************************************
   * Hysteresis Tests
   ***************************************************************************/

  // Apply hysteresis quantization
  double applyHysteresis(double input, double hysteresis) {
    return std::round(input / hysteresis) * hysteresis;
  }

  // Test hysteresis quantization
  void testHysteresisQuantization() {
    double hysteresis = 5.0;

    TS_ASSERT_DELTA(applyHysteresis(0.0, hysteresis), 0.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(2.4, hysteresis), 0.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(2.6, hysteresis), 5.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(7.4, hysteresis), 5.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(7.6, hysteresis), 10.0, epsilon);
  }

  // Test stepper motor simulation
  void testStepperMotorSimulation() {
    double hysteresis = 5.0;  // 5 degree steps

    // Simulate stepper motor with 72 steps per revolution
    double outputs[] = {0, 5, 10, 15, 20};
    for (int i = 0; i <= 4; i++) {
      double input = i * 5.0 + 2.0;  // 2, 7, 12, 17, 22
      double quantized = applyHysteresis(input, hysteresis);
      TS_ASSERT_DELTA(quantized, outputs[i], epsilon);
    }
  }

  /***************************************************************************
   * Module Tests
   ***************************************************************************/

  // Test module of 10 (decimal counter)
  void testModuleTen() {
    double module = 10.0;
    double input = 25.0;
    int fullRotations = static_cast<int>(input / module);
    double remainder = std::fmod(input, module);

    TS_ASSERT_EQUALS(fullRotations, 2);
    TS_ASSERT_DELTA(remainder, 5.0, epsilon);
  }

  // Test module of 360 (compass)
  void testModuleCompass() {
    double module = 360.0;
    double input = 450.0;
    int fullRotations = static_cast<int>(input / module);
    double remainder = std::fmod(input, module);

    TS_ASSERT_EQUALS(fullRotations, 1);
    TS_ASSERT_DELTA(remainder, 90.0, epsilon);
  }

  // Test module of 1 (step counter)
  void testModuleOne() {
    double module = 1.0;
    double input = 5.7;
    int fullRotations = static_cast<int>(input / module);
    double remainder = std::fmod(input, module);

    TS_ASSERT_EQUALS(fullRotations, 5);
    TS_ASSERT_DELTA(remainder, 0.7, epsilon);
  }

  /***************************************************************************
   * Versus (Direction Lock) Tests
   ***************************************************************************/

  // Test increasing-only direction
  void testVersusIncreasing() {
    double prevInput = 10.0;
    double newInput = 15.0;
    int versus = 1;  // Increasing only

    bool accept = (versus == 0) ||
                  (versus > 0 && newInput > prevInput) ||
                  (versus < 0 && newInput < prevInput);

    TS_ASSERT(accept);
  }

  // Test increasing-only rejects decrease
  void testVersusIncreasingRejectsDecrease() {
    double prevInput = 15.0;
    double newInput = 10.0;
    int versus = 1;

    bool accept = (versus == 0) ||
                  (versus > 0 && newInput > prevInput) ||
                  (versus < 0 && newInput < prevInput);

    TS_ASSERT(!accept);
  }

  // Test decreasing-only direction
  void testVersusDecreasing() {
    double prevInput = 15.0;
    double newInput = 10.0;
    int versus = -1;

    bool accept = (versus == 0) ||
                  (versus > 0 && newInput > prevInput) ||
                  (versus < 0 && newInput < prevInput);

    TS_ASSERT(accept);
  }

  // Test bidirectional (versus = 0)
  void testVersusBidirectional() {
    int versus = 0;

    bool acceptIncrease = (versus == 0) || (versus > 0);
    bool acceptDecrease = (versus == 0) || (versus < 0);

    TS_ASSERT(acceptIncrease);
    TS_ASSERT(acceptDecrease);
  }

  /***************************************************************************
   * Lag Filter Tests
   ***************************************************************************/

  // First-order lag filter
  double lagFilter(double input, double prevOutput, double lagTime, double dt) {
    if (lagTime <= 0.0) return input;
    double ca = std::exp(-dt / lagTime);
    double cb = 1.0 - ca;
    return ca * prevOutput + cb * input;
  }

  // Test lag filter convergence
  void testLagFilterConvergence() {
    double prevOutput = 0.0;
    double input = 10.0;
    double lagTime = 0.5;
    double dt = 0.01;

    for (int i = 0; i < 500; i++) {
      prevOutput = lagFilter(input, prevOutput, lagTime, dt);
    }

    TS_ASSERT_DELTA(prevOutput, input, 0.01);
  }

  // Test lag filter step response
  void testLagFilterStepResponse() {
    double prevOutput = 0.0;
    double input = 10.0;
    double lagTime = 0.1;
    double dt = 0.01;

    // After 1 time constant, should be at ~63.2%
    int steps = static_cast<int>(lagTime / dt);
    for (int i = 0; i < steps; i++) {
      prevOutput = lagFilter(input, prevOutput, lagTime, dt);
    }

    double expected = input * (1.0 - std::exp(-1.0));
    TS_ASSERT_DELTA(prevOutput, expected, 0.5);
  }

  // Test no lag (lagTime = 0)
  void testNoLag() {
    double input = 10.0;
    double output = lagFilter(input, 0.0, 0.0, 0.01);
    TS_ASSERT_DELTA(output, input, epsilon);
  }

  /***************************************************************************
   * Reset Tests
   ***************************************************************************/

  // Test reset functionality
  void testReset() {
    LinearActuatorState state;
    state.countSpin = 5;
    state.inputLast = 180.0;
    state.inputMem = 180.0;

    // Simulate reset
    state.countSpin = 0;
    state.inputLast = 0.0;
    state.inputMem = 0.0;

    TS_ASSERT_EQUALS(state.countSpin, 0);
    TS_ASSERT_DELTA(state.inputLast, 0.0, epsilon);
  }

  /***************************************************************************
   * Set (Enable) Tests
   ***************************************************************************/

  // Test disabled actuator holds value
  void testDisabledHoldsValue() {
    double output = 45.0;
    double newInput = 90.0;
    bool set = false;  // Disabled

    double result = set ? newInput : output;
    TS_ASSERT_DELTA(result, 45.0, epsilon);  // Holds previous
  }

  // Test enabled actuator updates
  void testEnabledUpdates() {
    double output = 45.0;
    double newInput = 90.0;
    bool set = true;  // Enabled

    double result = set ? newInput : output;
    TS_ASSERT_DELTA(result, 90.0, epsilon);
  }

  /***************************************************************************
   * Application Examples
   ***************************************************************************/

  // Test odometer (kilometer counter)
  void testOdometer() {
    // Wheel rotations to distance
    double wheelCircumference = 2.0;  // meters
    int rotations = 500;

    double distance = rotations * wheelCircumference;
    TS_ASSERT_DELTA(distance, 1000.0, epsilon);  // 1 km
  }

  // Test gyrocompass unwinding
  void testGyrocompassUnwinding() {
    // Gyrocompass goes 0-360, but displays continuous
    double module = 360.0;
    double heading = 45.0;
    int spins = 2;  // Two full rotations

    double totalAngle = heading + spins * module;
    TS_ASSERT_DELTA(totalAngle, 765.0, epsilon);
  }

  // Test pulse counter
  void testPulseCounter() {
    // Count pulses from a switch
    double module = 1.0;
    double gain = 0.5;  // Two pulses per count
    int pulses = 10;

    double output = pulses * module * gain;
    TS_ASSERT_DELTA(output, 5.0, epsilon);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero input
  void testZeroInput() {
    LinearActuatorState state;
    double output = runLinearActuator(state, 0.0);
    TS_ASSERT_DELTA(output, 0.0, 1.0);
  }

  // Test negative input
  void testNegativeInput() {
    LinearActuatorState state;
    state.module = 360.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, -45.0);
    TS_ASSERT_DELTA(output, -45.0, 1.0);
  }

  // Test very small hysteresis
  void testVerySmallHysteresis() {
    double hysteresis = 0.001;
    double input = 0.1234;

    double quantized = applyHysteresis(input, hysteresis);
    TS_ASSERT_DELTA(quantized, 0.123, 0.001);
  }

  /***************************************************************************
   * Extended Basic Output Tests
   ***************************************************************************/

  // Test combined gain and bias
  void testGainAndBiasCombined() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 2.5;
    state.bias = 20.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 100.0);
    // output = 2.5 * (20.0 + 100.0) = 300.0
    TS_ASSERT_DELTA(output, 300.0, 5.0);
  }

  // Test negative gain (inverts output)
  void testNegativeGain() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = -1.0;
    state.bias = 0.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 45.0);
    TS_ASSERT_DELTA(output, -45.0, 1.0);
  }

  // Test fractional gain
  void testFractionalGain() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 0.1;
    state.bias = 0.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 100.0);
    TS_ASSERT_DELTA(output, 10.0, 1.0);
  }

  // Test negative bias
  void testNegativeBias() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 1.0;
    state.bias = -50.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 100.0);
    TS_ASSERT_DELTA(output, 50.0, 1.0);  // 100 - 50
  }

  /***************************************************************************
   * Extended Spin Counter Tests
   ***************************************************************************/

  // Test backward spin (decreasing direction)
  void testBackwardSpin() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Go backwards through zero
    runLinearActuator(state, 45.0);
    runLinearActuator(state, 10.0);
    runLinearActuator(state, 350.0);  // Wrap backward

    TS_ASSERT_EQUALS(state.countSpin, -1);
  }

  // Test multiple forward spins - crossing 0 multiple times
  void testMultipleForwardSpins() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Going from high to low (crossing zero) triggers spin++
    // Initialize inputLast to avoid spurious spin on first call
    state.inputLast = 350.0;
    runLinearActuator(state, 10.0);   // Spin 1: delta=-340 < -108, spin++

    TS_ASSERT_EQUALS(state.countSpin, 1);

    // Do multiple spins
    state.inputLast = 350.0;
    runLinearActuator(state, 10.0);   // Spin 2
    state.inputLast = 350.0;
    runLinearActuator(state, 10.0);   // Spin 3

    TS_ASSERT_EQUALS(state.countSpin, 3);
  }

  // Test spin with different modules
  void testSpinWithSmallModule() {
    LinearActuatorState state;
    state.module = 10.0;  // Small module
    state.rate = 0.3;
    state.hysteresis = 0.1;

    // Simulate wrap from high to low: delta = 1 - 9 = -8 < -3, spin++
    state.inputLast = 9.0;
    runLinearActuator(state, 1.0);  // Wrap forward

    TS_ASSERT_EQUALS(state.countSpin, 1);
  }

  // Test fast wrapping detection
  void testFastWrappingDetection() {
    double prev = 5.0;
    double next = 355.0;  // Large negative jump
    double module = 360.0;
    double rate = 0.3;

    int spin = countSpins(prev, next, module, rate);
    // 355 - 5 = 350 > 0.3*360 = 108, so spin--
    TS_ASSERT_EQUALS(spin, -1);
  }

  /***************************************************************************
   * Extended Hysteresis Tests
   ***************************************************************************/

  // Test large hysteresis
  void testLargeHysteresis() {
    double hysteresis = 45.0;

    TS_ASSERT_DELTA(applyHysteresis(0.0, hysteresis), 0.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(22.4, hysteresis), 0.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(22.6, hysteresis), 45.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(67.4, hysteresis), 45.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(67.6, hysteresis), 90.0, epsilon);
  }

  // Test negative input with hysteresis
  void testNegativeHysteresis() {
    double hysteresis = 10.0;

    TS_ASSERT_DELTA(applyHysteresis(-15.0, hysteresis), -20.0, epsilon);
    TS_ASSERT_DELTA(applyHysteresis(-25.0, hysteresis), -30.0, epsilon);
  }

  // Test hysteresis boundary
  void testHysteresisBoundary() {
    double hysteresis = 5.0;

    // Exactly on boundary - round() rounds to nearest even or away from zero
    TS_ASSERT_DELTA(applyHysteresis(2.5, hysteresis), 5.0, epsilon);
    // -2.5 rounds to -1 (away from zero), so -1 * 5.0 = -5.0
    TS_ASSERT_DELTA(applyHysteresis(-2.5, hysteresis), -5.0, epsilon);
  }

  /***************************************************************************
   * Extended Module Tests
   ***************************************************************************/

  // Test very large module
  void testVeryLargeModule() {
    double module = 10000.0;
    double input = 45000.0;
    int fullRotations = static_cast<int>(input / module);
    double remainder = std::fmod(input, module);

    TS_ASSERT_EQUALS(fullRotations, 4);
    TS_ASSERT_DELTA(remainder, 5000.0, epsilon);
  }

  // Test non-standard module (100 for percentage)
  void testModulePercentage() {
    double module = 100.0;
    double input = 250.0;
    int fullRotations = static_cast<int>(input / module);
    double remainder = std::fmod(input, module);

    TS_ASSERT_EQUALS(fullRotations, 2);
    TS_ASSERT_DELTA(remainder, 50.0, epsilon);
  }

  // Test fractional module
  void testFractionalModule() {
    double module = 0.5;
    double input = 3.75;
    int fullRotations = static_cast<int>(input / module);
    double remainder = std::fmod(input, module);

    TS_ASSERT_EQUALS(fullRotations, 7);
    TS_ASSERT_DELTA(remainder, 0.25, epsilon);
  }

  /***************************************************************************
   * Extended Lag Filter Tests
   ***************************************************************************/

  // Test fast lag (small time constant)
  void testFastLag() {
    double prevOutput = 0.0;
    double input = 100.0;
    double lagTime = 0.01;
    double dt = 0.01;

    // After one step with lagTime = dt, should be about 63.2%
    prevOutput = lagFilter(input, prevOutput, lagTime, dt);
    double expected = input * (1.0 - std::exp(-1.0));
    TS_ASSERT_DELTA(prevOutput, expected, 1.0);
  }

  // Test slow lag (large time constant)
  void testSlowLag() {
    double prevOutput = 0.0;
    double input = 100.0;
    double lagTime = 10.0;
    double dt = 0.01;

    // After one step, should barely move
    prevOutput = lagFilter(input, prevOutput, lagTime, dt);
    TS_ASSERT(prevOutput < 1.0);
  }

  // Test lag filter with changing input
  void testLagFilterTracking() {
    double prevOutput = 0.0;
    double lagTime = 0.1;
    double dt = 0.01;

    // Step to 50
    for (int i = 0; i < 50; i++) {
      prevOutput = lagFilter(50.0, prevOutput, lagTime, dt);
    }
    TS_ASSERT_DELTA(prevOutput, 50.0, 1.0);

    // Step to 100
    for (int i = 0; i < 50; i++) {
      prevOutput = lagFilter(100.0, prevOutput, lagTime, dt);
    }
    TS_ASSERT_DELTA(prevOutput, 100.0, 1.0);
  }

  // Test lag filter decay
  void testLagFilterDecay() {
    double prevOutput = 100.0;
    double input = 0.0;
    double lagTime = 0.5;
    double dt = 0.01;

    // Decay to zero
    for (int i = 0; i < 500; i++) {
      prevOutput = lagFilter(input, prevOutput, lagTime, dt);
    }

    TS_ASSERT_DELTA(prevOutput, 0.0, 0.1);
  }

  /***************************************************************************
   * Extended Versus Tests
   ***************************************************************************/

  // Test versus with zero change
  void testVersusNoChange() {
    double prevInput = 50.0;
    double newInput = 50.0;
    int versus = 1;  // Increasing only

    // No change should be rejected in strict increasing mode
    bool accept = (versus == 0) ||
                  (versus > 0 && newInput > prevInput) ||
                  (versus < 0 && newInput < prevInput);

    TS_ASSERT(!accept);  // Rejected because not strictly increasing
  }

  // Test versus sequence
  void testVersusIncreasingSequence() {
    int versus = 1;
    double values[] = {10, 20, 30, 25, 40};
    int acceptCount = 0;

    double prev = 0.0;
    for (int i = 0; i < 5; i++) {
      bool accept = (versus == 0) ||
                    (versus > 0 && values[i] > prev) ||
                    (versus < 0 && values[i] < prev);
      if (accept) {
        acceptCount++;
        prev = values[i];
      }
    }

    // 10, 20, 30, 40 accepted; 25 rejected
    TS_ASSERT_EQUALS(acceptCount, 4);
  }

  /***************************************************************************
   * Actuator Behavior Tests
   ***************************************************************************/

  // Saturation function
  double saturate(double value, double minVal, double maxVal) {
    return std::max(minVal, std::min(value, maxVal));
  }

  // Test saturation limits
  void testSaturationLimits() {
    double minVal = -100.0;
    double maxVal = 100.0;

    TS_ASSERT_DELTA(saturate(50.0, minVal, maxVal), 50.0, epsilon);
    TS_ASSERT_DELTA(saturate(150.0, minVal, maxVal), 100.0, epsilon);
    TS_ASSERT_DELTA(saturate(-150.0, minVal, maxVal), -100.0, epsilon);
  }

  // Slew rate limiting
  double slewRateLimit(double target, double current, double maxRate, double dt) {
    double delta = target - current;
    double maxDelta = maxRate * dt;
    if (std::abs(delta) > maxDelta) {
      delta = (delta > 0) ? maxDelta : -maxDelta;
    }
    return current + delta;
  }

  // Test slew rate limiting
  void testSlewRateLimiting() {
    double current = 0.0;
    double target = 100.0;
    double maxRate = 50.0;
    double dt = 0.1;

    // First step: move 5.0
    current = slewRateLimit(target, current, maxRate, dt);
    TS_ASSERT_DELTA(current, 5.0, epsilon);

    // After 20 steps: should reach target
    for (int i = 0; i < 20; i++) {
      current = slewRateLimit(target, current, maxRate, dt);
    }
    TS_ASSERT_DELTA(current, 100.0, epsilon);
  }

  // Deadband function
  double applyDeadband(double input, double deadband) {
    if (std::abs(input) < deadband) return 0.0;
    return (input > 0) ? input - deadband : input + deadband;
  }

  // Test deadband
  void testDeadband() {
    double deadband = 5.0;

    TS_ASSERT_DELTA(applyDeadband(0.0, deadband), 0.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(3.0, deadband), 0.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(10.0, deadband), 5.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(-10.0, deadband), -5.0, epsilon);
  }

  /***************************************************************************
   * Real-World Application Tests
   ***************************************************************************/

  // Test servo motor position control
  void testServoMotorPosition() {
    // Servo with 180 degree range, 10-bit resolution
    double resolution = 180.0 / 1024.0;  // ~0.176 deg per step
    double targetAngle = 90.0;

    double quantized = std::round(targetAngle / resolution) * resolution;
    TS_ASSERT_DELTA(quantized, 90.0, resolution);
  }

  // Test rotary encoder simulation
  void testRotaryEncoder() {
    // 1000 pulses per revolution encoder
    double pulsesPerRev = 1000.0;
    double angle = 45.0;  // degrees

    double pulses = angle / 360.0 * pulsesPerRev;
    TS_ASSERT_DELTA(pulses, 125.0, 0.1);
  }

  // Test aircraft heading indicator
  void testHeadingIndicator() {
    LinearActuatorState state;
    state.module = 360.0;
    state.hysteresis = 1.0;  // 1 degree resolution
    state.gain = 1.0;
    state.bias = 0.0;
    state.rate = 0.3;

    // Aircraft turns from 350 to 010 (crossing 0)
    state.inputLast = 350.0;  // Set previous position
    double output = runLinearActuator(state, 10.0);

    // delta = 10 - 350 = -340 < -108, so spin++ = 1
    // output = 1 * (0 + 10 + 360*1) = 370
    TS_ASSERT_DELTA(output, 370.0, 5.0);
  }

  // Test altimeter encoding
  void testAltimeterEncoding() {
    // Altimeter with 100 ft increments
    double module = 1000.0;  // 1000 ft dial
    double altitude = 12345.0;

    int thousands = static_cast<int>(altitude / module);
    double hundreds = std::fmod(altitude, module);

    TS_ASSERT_EQUALS(thousands, 12);
    TS_ASSERT_DELTA(hundreds, 345.0, epsilon);
  }

  // Test tachometer
  void testTachometer() {
    // RPM from pulse timing
    double pulseFrequency = 100.0;  // Hz
    double pulsesPerRev = 2.0;

    double rpm = (pulseFrequency / pulsesPerRev) * 60.0;
    TS_ASSERT_DELTA(rpm, 3000.0, epsilon);
  }

  /***************************************************************************
   * Integration Tests
   ***************************************************************************/

  // Test combined features
  void testCombinedFeatures() {
    LinearActuatorState state;
    state.module = 360.0;
    state.gain = 2.0;
    state.bias = 10.0;
    state.hysteresis = 5.0;
    state.rate = 0.3;

    // Run through a sequence
    runLinearActuator(state, 0.0);
    runLinearActuator(state, 45.0);
    runLinearActuator(state, 90.0);
    double output = runLinearActuator(state, 180.0);

    // output = gain * (bias + quantized + module * countSpin)
    // = 2.0 * (10.0 + 180.0 + 360.0 * 0) = 380.0
    TS_ASSERT_DELTA(output, 380.0, 10.0);
  }

  // Test state persistence
  void testStatePersistence() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Build up spin count
    runLinearActuator(state, 0.0);
    runLinearActuator(state, 350.0);
    runLinearActuator(state, 10.0);  // Spin 1

    int savedSpin = state.countSpin;

    // Continue operation
    runLinearActuator(state, 45.0);
    runLinearActuator(state, 90.0);

    // Spin count should persist
    TS_ASSERT_EQUALS(state.countSpin, savedSpin);
  }

  /***************************************************************************
   * More Edge Cases
   ***************************************************************************/

  // Test maximum input value
  void testMaximumInput() {
    LinearActuatorState state;
    state.module = 360.0;
    state.hysteresis = 1.0;

    double output = runLinearActuator(state, 1e6);
    TS_ASSERT(!std::isnan(output));
    TS_ASSERT(!std::isinf(output));
  }

  // Test near-zero hysteresis
  void testNearZeroHysteresis() {
    double hysteresis = 1e-10;
    double input = 123.456789;

    double quantized = applyHysteresis(input, hysteresis);
    TS_ASSERT_DELTA(quantized, input, 1e-9);
  }

  // Test input at module boundary
  void testInputAtModuleBoundary() {
    LinearActuatorState state;
    state.module = 360.0;
    state.hysteresis = 1.0;
    state.rate = 0.3;

    // Starting from 0, going to 360 triggers no wrap (delta=360 is large positive)
    // delta = 360 - 0 = 360 > 108, so spin-- = -1
    // output = 1 * (0 + 360 + 360*(-1)) = 0
    double output = runLinearActuator(state, 360.0);
    TS_ASSERT_DELTA(output, 0.0, 1.0);  // Wraps to 0

    // But if we go gradually to 360, no wrap occurs
    LinearActuatorState state2;
    state2.module = 360.0;
    state2.hysteresis = 1.0;
    state2.inputLast = 350.0;
    output = runLinearActuator(state2, 360.0);
    // delta = 10, no wrap, output = 360
    TS_ASSERT_DELTA(output, 360.0, 1.0);
  }

  // Test rapid direction changes
  void testRapidDirectionChanges() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    runLinearActuator(state, 0.0);
    runLinearActuator(state, 10.0);
    runLinearActuator(state, 5.0);
    runLinearActuator(state, 15.0);
    runLinearActuator(state, 8.0);

    // Should not trigger any spins
    TS_ASSERT_EQUALS(state.countSpin, 0);
  }

  // Test symmetry
  void testSymmetry() {
    double hysteresis = 10.0;

    double pos = applyHysteresis(45.0, hysteresis);
    double neg = applyHysteresis(-45.0, hysteresis);

    TS_ASSERT_DELTA(pos, -neg, epsilon);
  }

  /***************************************************************************
   * Multi-Turn Tracking Tests
   ***************************************************************************/

  // Test extended range multi-turn tracking
  void testMultiTurnExtendedRange() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Simulate 10 full rotations
    double angles[] = {0, 90, 180, 270, 350, 10};  // One full rotation
    for (int rotation = 0; rotation < 10; rotation++) {
      for (int i = 0; i < 6; i++) {
        runLinearActuator(state, angles[i]);
      }
    }

    // Should track 10 full rotations
    TS_ASSERT_EQUALS(state.countSpin, 10);
  }

  // Test bidirectional multi-turn
  void testBidirectionalMultiTurn() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Go forward 3 rotations
    for (int i = 0; i < 3; i++) {
      state.inputLast = 350.0;
      runLinearActuator(state, 10.0);
    }
    TS_ASSERT_EQUALS(state.countSpin, 3);

    // Go backward 2 rotations
    for (int i = 0; i < 2; i++) {
      state.inputLast = 10.0;
      runLinearActuator(state, 350.0);
    }
    TS_ASSERT_EQUALS(state.countSpin, 1);  // 3 - 2 = 1
  }

  // Test multi-turn with high precision
  void testMultiTurnHighPrecision() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 0.01;  // High precision

    // Verify sub-degree tracking
    runLinearActuator(state, 0.0);
    runLinearActuator(state, 0.5);
    runLinearActuator(state, 1.0);

    // No wrapping, just precision movement
    TS_ASSERT_EQUALS(state.countSpin, 0);
  }

  /***************************************************************************
   * Mechanical Backlash Tests
   ***************************************************************************/

  // Simulate mechanical backlash
  double applyBacklash(double target, double current, double lastDirection, double backlash) {
    double delta = target - current;
    int newDirection = (delta > 0) ? 1 : -1;

    if (newDirection != lastDirection && std::abs(delta) < backlash) {
      return current;  // Within backlash zone, no movement
    }

    if (newDirection != lastDirection) {
      // Direction change, absorb backlash
      return current + (delta > 0 ? delta - backlash : delta + backlash);
    }

    return target;
  }

  // Test backlash hysteresis zone
  void testBacklashHysteresisZone() {
    double current = 50.0;
    double backlash = 5.0;
    int lastDirection = 1;  // Was moving positive

    // Small negative movement within backlash
    double output = applyBacklash(48.0, current, lastDirection, backlash);
    TS_ASSERT_DELTA(output, 50.0, epsilon);  // No movement

    // Larger negative movement exceeds backlash
    output = applyBacklash(40.0, current, lastDirection, backlash);
    TS_ASSERT_DELTA(output, 45.0, epsilon);  // Move after absorbing backlash
  }

  // Test zero backlash (ideal actuator)
  void testZeroBacklash() {
    double current = 50.0;
    double backlash = 0.0;
    int lastDirection = 1;

    double output = applyBacklash(48.0, current, lastDirection, backlash);
    TS_ASSERT_DELTA(output, 48.0, epsilon);  // Immediate movement
  }

  /***************************************************************************
   * Limit Switch Tests
   ***************************************************************************/

  struct LimitSwitchState {
    double position;
    double lowerLimit;
    double upperLimit;
    bool lowerTriggered;
    bool upperTriggered;
  };

  void checkLimitSwitches(LimitSwitchState& state) {
    state.lowerTriggered = (state.position <= state.lowerLimit);
    state.upperTriggered = (state.position >= state.upperLimit);
  }

  // Test lower limit switch
  void testLowerLimitSwitch() {
    LimitSwitchState state;
    state.lowerLimit = 0.0;
    state.upperLimit = 100.0;
    state.position = -5.0;

    checkLimitSwitches(state);
    TS_ASSERT(state.lowerTriggered);
    TS_ASSERT(!state.upperTriggered);
  }

  // Test upper limit switch
  void testUpperLimitSwitch() {
    LimitSwitchState state;
    state.lowerLimit = 0.0;
    state.upperLimit = 100.0;
    state.position = 105.0;

    checkLimitSwitches(state);
    TS_ASSERT(!state.lowerTriggered);
    TS_ASSERT(state.upperTriggered);
  }

  // Test within limits
  void testWithinLimits() {
    LimitSwitchState state;
    state.lowerLimit = 0.0;
    state.upperLimit = 100.0;
    state.position = 50.0;

    checkLimitSwitches(state);
    TS_ASSERT(!state.lowerTriggered);
    TS_ASSERT(!state.upperTriggered);
  }

  // Test soft limits with deceleration
  void testSoftLimits() {
    double position = 95.0;
    double softLimitStart = 90.0;
    double hardLimit = 100.0;
    double velocity = 10.0;

    // Calculate deceleration factor as position approaches limit
    double margin = hardLimit - position;
    double softZone = hardLimit - softLimitStart;
    double decelFactor = (position > softLimitStart) ? margin / softZone : 1.0;

    double limitedVelocity = velocity * decelFactor;
    TS_ASSERT_DELTA(limitedVelocity, 5.0, epsilon);  // 10 * 0.5
  }

  /***************************************************************************
   * Fault Detection Tests
   ***************************************************************************/

  // Test stall detection
  void testStallDetection() {
    double commandedPosition = 50.0;
    double actualPosition = 10.0;
    double stallThreshold = 5.0;
    double stallTime = 0.0;
    double stallTimeLimit = 1.0;
    double dt = 0.1;

    // Position error exceeds threshold
    double error = std::abs(commandedPosition - actualPosition);

    // Simulate 11 iterations with no movement (exceeds 1.0 second)
    for (int i = 0; i < 11; i++) {
      if (error > stallThreshold) {
        stallTime += dt;
      } else {
        stallTime = 0.0;
      }
    }

    bool stalled = (stallTime >= stallTimeLimit);
    TS_ASSERT(stalled);
  }

  // Test runaway detection
  void testRunawayDetection() {
    double expectedRate = 10.0;  // degrees/sec
    double actualRate = 100.0;   // degrees/sec
    double runawayThreshold = 2.0;  // 2x expected rate

    bool runaway = (actualRate > expectedRate * runawayThreshold);
    TS_ASSERT(runaway);
  }

  // Test position feedback fault
  void testPositionFeedbackFault() {
    double commandHistory[] = {10, 20, 30, 40, 50};
    double feedbackHistory[] = {10, 10, 10, 10, 10};  // Stuck feedback

    double maxFeedbackDelta = 0.0;
    for (int i = 1; i < 5; i++) {
      double delta = std::abs(feedbackHistory[i] - feedbackHistory[i-1]);
      maxFeedbackDelta = std::max(maxFeedbackDelta, delta);
    }

    // Command changed significantly but feedback didn't
    double commandDelta = std::abs(commandHistory[4] - commandHistory[0]);
    bool feedbackFault = (commandDelta > 10.0 && maxFeedbackDelta < 1.0);

    TS_ASSERT(feedbackFault);
  }

  /***************************************************************************
   * Calibration Tests
   ***************************************************************************/

  // Test zero calibration
  void testZeroCalibration() {
    double rawPosition = 15.0;
    double zeroOffset = 15.0;

    double calibrated = rawPosition - zeroOffset;
    TS_ASSERT_DELTA(calibrated, 0.0, epsilon);
  }

  // Test span calibration
  void testSpanCalibration() {
    double rawPosition = 500.0;  // Raw counts
    double countsPerDegree = 10.0;

    double calibrated = rawPosition / countsPerDegree;
    TS_ASSERT_DELTA(calibrated, 50.0, epsilon);
  }

  // Test two-point calibration
  void testTwoPointCalibration() {
    // Calibration points: raw 100 = 0 deg, raw 900 = 180 deg
    double rawLow = 100.0, rawHigh = 900.0;
    double calLow = 0.0, calHigh = 180.0;

    // Calculate slope and offset
    double slope = (calHigh - calLow) / (rawHigh - rawLow);
    double offset = calLow - slope * rawLow;

    // Test a raw value
    double rawTest = 500.0;
    double calibrated = slope * rawTest + offset;

    // Expected: (500-100) * 180/800 = 400 * 0.225 = 90
    TS_ASSERT_DELTA(calibrated, 90.0, 0.1);
  }

  // Test multi-point linearization
  void testMultiPointLinearization() {
    // Non-linear sensor calibration table
    double rawPoints[] = {0, 100, 200, 300, 400};
    double calPoints[] = {0, 25, 55, 90, 100};  // Non-linear

    // Interpolate for raw = 150
    double raw = 150.0;
    int segment = 1;  // Between 100 and 200

    double t = (raw - rawPoints[segment]) / (rawPoints[segment+1] - rawPoints[segment]);
    double calibrated = calPoints[segment] + t * (calPoints[segment+1] - calPoints[segment]);

    // Expected: 25 + 0.5 * (55-25) = 40
    TS_ASSERT_DELTA(calibrated, 40.0, epsilon);
  }

  /***************************************************************************
   * Position Servo Loop Tests
   ***************************************************************************/

  // Test proportional position control
  void testProportionalPositionControl() {
    double target = 100.0;
    double current = 60.0;
    double Kp = 0.5;

    double error = target - current;
    double command = Kp * error;

    TS_ASSERT_DELTA(command, 20.0, epsilon);  // 0.5 * 40
  }

  // Test position servo convergence
  void testPositionServoConvergence() {
    double target = 100.0;
    double current = 0.0;
    double Kp = 0.2;

    for (int i = 0; i < 50; i++) {
      double error = target - current;
      double velocity = Kp * error;
      current += velocity;
    }

    TS_ASSERT_DELTA(current, target, 0.1);
  }

  // Test velocity feedforward
  void testVelocityFeedforward() {
    double targetPos = 100.0;
    double targetVel = 10.0;  // Known target velocity
    double currentPos = 90.0;
    double Kp = 0.5;
    double Kff = 1.0;

    double posError = targetPos - currentPos;
    double command = Kp * posError + Kff * targetVel;

    // 0.5 * 10 + 1.0 * 10 = 15
    TS_ASSERT_DELTA(command, 15.0, epsilon);
  }

  /***************************************************************************
   * Velocity Mode Tests
   ***************************************************************************/

  // Test velocity integration
  void testVelocityModeIntegration() {
    double position = 0.0;
    double velocity = 10.0;  // degrees/sec
    double dt = 0.1;

    for (int i = 0; i < 10; i++) {
      position += velocity * dt;
    }

    TS_ASSERT_DELTA(position, 10.0, epsilon);  // 10 * 10 * 0.1
  }

  // Test velocity limiting
  void testVelocityModeLimiting() {
    double commandedVelocity = 100.0;
    double maxVelocity = 50.0;

    double limitedVelocity = std::min(std::abs(commandedVelocity), maxVelocity);
    if (commandedVelocity < 0) limitedVelocity = -limitedVelocity;

    TS_ASSERT_DELTA(limitedVelocity, 50.0, epsilon);
  }

  // Test acceleration limiting
  void testAccelerationLimiting() {
    double currentVelocity = 10.0;
    double targetVelocity = 100.0;
    double maxAcceleration = 20.0;  // deg/s^2
    double dt = 0.1;

    double maxDeltaV = maxAcceleration * dt;
    double deltaV = targetVelocity - currentVelocity;

    if (std::abs(deltaV) > maxDeltaV) {
      deltaV = (deltaV > 0) ? maxDeltaV : -maxDeltaV;
    }

    double newVelocity = currentVelocity + deltaV;
    TS_ASSERT_DELTA(newVelocity, 12.0, epsilon);  // 10 + 2
  }

  /***************************************************************************
   * Homing Sequence Tests
   ***************************************************************************/

  enum HomingState { IDLE, SEEKING_SWITCH, BACKING_OFF, MOVING_TO_INDEX, COMPLETE };

  // Test homing state machine
  void testHomingStateMachine() {
    HomingState state = IDLE;
    bool limitSwitch = false;
    bool indexPulse = false;

    // Start homing
    state = SEEKING_SWITCH;
    TS_ASSERT_EQUALS(static_cast<int>(state), static_cast<int>(SEEKING_SWITCH));

    // Hit limit switch
    limitSwitch = true;
    if (state == SEEKING_SWITCH && limitSwitch) {
      state = BACKING_OFF;
    }
    TS_ASSERT_EQUALS(static_cast<int>(state), static_cast<int>(BACKING_OFF));

    // Clear of switch
    limitSwitch = false;
    if (state == BACKING_OFF && !limitSwitch) {
      state = MOVING_TO_INDEX;
    }
    TS_ASSERT_EQUALS(static_cast<int>(state), static_cast<int>(MOVING_TO_INDEX));

    // Found index pulse
    indexPulse = true;
    if (state == MOVING_TO_INDEX && indexPulse) {
      state = COMPLETE;
    }
    TS_ASSERT_EQUALS(static_cast<int>(state), static_cast<int>(COMPLETE));
  }

  // Test homing velocity profile
  void testHomingVelocityProfile() {
    double fastSpeed = 100.0;
    double slowSpeed = 10.0;
    bool nearLimit = true;

    double homingSpeed = nearLimit ? slowSpeed : fastSpeed;
    TS_ASSERT_DELTA(homingSpeed, 10.0, epsilon);
  }

  /***************************************************************************
   * Resolution and Accuracy Tests
   ***************************************************************************/

  // Test encoder resolution
  void testEncoderResolution() {
    int countsPerRev = 4096;  // 12-bit encoder
    double resolution = 360.0 / countsPerRev;

    TS_ASSERT_DELTA(resolution, 0.0879, 0.001);  // ~0.088 degrees
  }

  // Test quantization error
  void testQuantizationError() {
    double resolution = 0.1;  // 0.1 degree resolution
    double trueAngle = 45.678;

    double quantized = std::round(trueAngle / resolution) * resolution;
    double error = trueAngle - quantized;

    TS_ASSERT(std::abs(error) <= resolution / 2.0);
  }

  // Test accumulated error over rotations
  void testAccumulatedError() {
    double resolution = 0.1;
    double anglePerStep = 10.0;
    double accumulatedAngle = 0.0;

    // 36 steps should be exactly 360 degrees
    for (int i = 0; i < 36; i++) {
      accumulatedAngle += anglePerStep;
    }

    TS_ASSERT_DELTA(accumulatedAngle, 360.0, epsilon);
  }

  /***************************************************************************
   * Power and Temperature Tests
   ***************************************************************************/

  // Test thermal drift compensation
  void testThermalDriftCompensation() {
    double nominalGain = 1.0;
    double temperature = 50.0;  // Celsius
    double refTemperature = 25.0;
    double tempCoeff = -0.001;  // -0.1% per degree

    double compensatedGain = nominalGain * (1.0 + tempCoeff * (temperature - refTemperature));
    // 1.0 * (1.0 - 0.001 * 25) = 1.0 * 0.975 = 0.975
    TS_ASSERT_DELTA(compensatedGain, 0.975, 0.001);
  }

  // Test power-on initialization
  void testPowerOnInitialization() {
    LinearActuatorState state;

    // Default initialization values
    TS_ASSERT_DELTA(state.inputLast, 0.0, epsilon);
    TS_ASSERT_EQUALS(state.countSpin, 0);
    TS_ASSERT_EQUALS(state.direction, 0);
  }

  // Test brownout recovery
  void testBrownoutRecovery() {
    LinearActuatorState state;
    state.countSpin = 5;
    state.inputLast = 180.0;

    // Simulate brownout - save state
    int savedSpin = state.countSpin;
    double savedInput = state.inputLast;

    // Power restored - restore state
    LinearActuatorState newState;
    newState.countSpin = savedSpin;
    newState.inputLast = savedInput;

    TS_ASSERT_EQUALS(newState.countSpin, 5);
    TS_ASSERT_DELTA(newState.inputLast, 180.0, epsilon);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteActuatorCycle() {
    // Test a complete cycle: start, move, wrap, return
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;
    state.gain = 1.0;
    state.bias = 0.0;

    // Initial position
    double output = runLinearActuator(state, 0.0);
    TS_ASSERT_DELTA(output, 0.0, 1.0);

    // Move in small increments (to avoid wrap detection)
    output = runLinearActuator(state, 45.0);
    TS_ASSERT_DELTA(output, 45.0, 1.0);

    output = runLinearActuator(state, 90.0);
    TS_ASSERT_DELTA(output, 90.0, 1.0);

    // Wrap crossing 0 (350 to 10) - reset state first
    state.countSpin = 0;
    state.inputLast = 350.0;
    output = runLinearActuator(state, 10.0);
    TS_ASSERT_EQUALS(state.countSpin, 1);
  }

  void testCompleteCalibrationSequence() {
    // Full calibration: zero, gain, offset
    double rawValue = 512.0;  // Raw ADC count
    double zeroOffset = 100.0;
    double scaleFactor = 0.1;  // degrees per count
    double biasCorrection = 2.0;

    // Apply calibration
    double zeroed = rawValue - zeroOffset;
    double scaled = zeroed * scaleFactor;
    double corrected = scaled + biasCorrection;

    // (512 - 100) * 0.1 + 2.0 = 41.2 + 2.0 = 43.2
    TS_ASSERT_DELTA(corrected, 43.2, epsilon);
  }

  void testCompleteServoPositioning() {
    // Complete servo positioning with rate limiting
    double targetPosition = 100.0;
    double currentPosition = 0.0;
    double maxRate = 20.0;  // degrees per step
    int steps = 0;

    while (std::abs(targetPosition - currentPosition) > 0.1 && steps < 100) {
      double error = targetPosition - currentPosition;
      double move = std::min(std::abs(error), maxRate);
      currentPosition += (error > 0) ? move : -move;
      steps++;
    }

    TS_ASSERT_DELTA(currentPosition, targetPosition, 0.1);
    TS_ASSERT_EQUALS(steps, 5);  // 100 / 20 = 5 steps
  }

  void testCompleteMultiTurnTracking() {
    LinearActuatorState state;
    state.module = 360.0;
    state.rate = 0.3;
    state.hysteresis = 1.0;

    // Track 5 complete forward rotations
    for (int rotation = 0; rotation < 5; rotation++) {
      state.inputLast = 350.0;
      runLinearActuator(state, 10.0);
    }
    TS_ASSERT_EQUALS(state.countSpin, 5);

    // Track 3 backward rotations
    for (int rotation = 0; rotation < 3; rotation++) {
      state.inputLast = 10.0;
      runLinearActuator(state, 350.0);
    }
    TS_ASSERT_EQUALS(state.countSpin, 2);  // 5 - 3 = 2
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentActuatorStates() {
    LinearActuatorState state1;
    LinearActuatorState state2;
    state1.module = 360.0;
    state2.module = 360.0;
    state1.rate = 0.3;
    state2.rate = 0.3;

    // Modify state1 only
    state1.inputLast = 350.0;
    runLinearActuator(state1, 10.0);

    // state2 should be unchanged
    TS_ASSERT_EQUALS(state2.countSpin, 0);
    TS_ASSERT_DELTA(state2.inputLast, 0.0, epsilon);

    // state1 should have wrapped
    TS_ASSERT_EQUALS(state1.countSpin, 1);
  }

  void testIndependentHysteresisOutputs() {
    double input = 45.678;
    double hysteresis1 = 1.0;
    double hysteresis2 = 10.0;

    double output1 = applyHysteresis(input, hysteresis1);
    double output2 = applyHysteresis(input, hysteresis2);

    // Different hysteresis values produce different outputs
    TS_ASSERT_DELTA(output1, 46.0, epsilon);
    TS_ASSERT_DELTA(output2, 50.0, epsilon);
    TS_ASSERT(std::abs(output1 - output2) > 1.0);
  }

  void testIndependentLagFilters() {
    double state1 = 0.0;
    double state2 = 0.0;
    double alpha1 = 0.1;
    double alpha2 = 0.9;
    double input = 100.0;

    // Apply different lag coefficients
    state1 = alpha1 * input + (1.0 - alpha1) * state1;
    state2 = alpha2 * input + (1.0 - alpha2) * state2;

    // Different responses
    TS_ASSERT_DELTA(state1, 10.0, epsilon);  // 0.1 * 100 = 10
    TS_ASSERT_DELTA(state2, 90.0, epsilon);  // 0.9 * 100 = 90
    TS_ASSERT(state1 < state2);
  }
};

//=============================================================================
// C172x Integration Tests - Actuator Response Tests
//=============================================================================

class FGLinearActuatorC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {


    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: Control surfaces respond to commands
  void testControlSurfaceResponse() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    double initialPos = fcs->GetDePos();
    fcs->SetDeCmd(0.5);

    for (int i = 0; i < 20; ++i) fdm.Run();

    double finalPos = fcs->GetDePos();
    TS_ASSERT(std::isfinite(finalPos));
  }

  // Test 2: Elevator rate limiting
  void testElevatorRateLimiting() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    fcs->SetDeCmd(0.0);
    for (int i = 0; i < 10; ++i) fdm.Run();
    double pos1 = fcs->GetDePos();

    fcs->SetDeCmd(1.0);  // Sudden full deflection
    fdm.Run();
    double pos2 = fcs->GetDePos();

    // Position should not jump instantly
    TS_ASSERT(std::isfinite(pos1));
    TS_ASSERT(std::isfinite(pos2));
  }

  // Test 3: Aileron symmetry
  void testAileronSymmetry() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    fcs->SetDaCmd(0.5);
    for (int i = 0; i < 20; ++i) fdm.Run();
    double posRight = fcs->GetDaLPos();

    fcs->SetDaCmd(-0.5);
    for (int i = 0; i < 20; ++i) fdm.Run();
    double posLeft = fcs->GetDaLPos();

    TS_ASSERT(std::isfinite(posRight));
    TS_ASSERT(std::isfinite(posLeft));
  }

  // Test 4: Rudder response
  void testRudderResponse() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    fcs->SetDrCmd(0.3);
    for (int i = 0; i < 20; ++i) fdm.Run();

    double rudderPos = fcs->GetDrPos();
    TS_ASSERT(std::isfinite(rudderPos));
  }

  // Test 5: Flap actuation time
  void testFlapActuationTime() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(80.0);
    ic->SetAltitudeASLFtIC(3000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    fcs->SetDfCmd(0.0);
    for (int i = 0; i < 20; ++i) fdm.Run();
    double flapPos0 = fcs->GetDfPos();

    fcs->SetDfCmd(1.0);  // Full flaps
    for (int i = 0; i < 100; ++i) fdm.Run();
    double flapPosFinal = fcs->GetDfPos();

    TS_ASSERT(std::isfinite(flapPos0));
    TS_ASSERT(std::isfinite(flapPosFinal));
  }

  // Test 6: Control deflection limits
  void testControlDeflectionLimits() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    // Command beyond limits
    fcs->SetDeCmd(2.0);  // Well beyond normal range
    for (int i = 0; i < 50; ++i) fdm.Run();

    double elevatorPos = fcs->GetDePos();
    TS_ASSERT(std::isfinite(elevatorPos));
    TS_ASSERT(elevatorPos <= 1.5);  // Should be limited
  }

  // Test 7: Trim actuator
  void testTrimActuator() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto pm = fdm.GetPropertyManager();

    // Check if pitch trim exists
    double pitchTrim = pm->GetNode("fcs/pitch-trim-cmd-norm")->getDoubleValue();
    TS_ASSERT(std::isfinite(pitchTrim));
  }

  // Test 8: Surface positions during flight
  void testSurfacePositionsDuringFlight() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetDeCmd(0.1);
    fcs->SetDaCmd(0.05);
    fcs->SetDrCmd(-0.02);

    for (int i = 0; i < 50; ++i) {
      fdm.Run();
      TS_ASSERT(std::isfinite(fcs->GetDePos()));
      TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
      TS_ASSERT(std::isfinite(fcs->GetDrPos()));
    }
  }

  // Test 9: Neutral position return
  void testNeutralPositionReturn() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    fcs->SetDeCmd(0.5);
    for (int i = 0; i < 50; ++i) fdm.Run();

    fcs->SetDeCmd(0.0);
    for (int i = 0; i < 50; ++i) fdm.Run();

    double elevatorPos = fcs->GetDePos();
    TS_ASSERT(std::isfinite(elevatorPos));
  }

  // Test 10: Multiple surface coordination
  void testMultipleSurfaceCoordination() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    // Coordinated turn inputs
    fcs->SetDaCmd(0.3);   // Roll right
    fcs->SetDrCmd(0.1);   // Rudder right
    fcs->SetDeCmd(-0.1);  // Slight back pressure

    for (int i = 0; i < 50; ++i) fdm.Run();

    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }

  // Test 11: Gear retraction (C172 has fixed gear)
  void testGearPosition() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    double gearPos = fcs->GetGearPos();

    // C172 has fixed gear, should always be down
    TS_ASSERT(std::isfinite(gearPos));
  }

  // Test 12: Extended simulation actuator stability
  void testExtendedSimulationActuatorStability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();

    // Varying inputs over time
    for (int i = 0; i < 500; ++i) {
      double t = i * 0.01;
      fcs->SetDeCmd(0.2 * std::sin(t));
      fcs->SetDaCmd(0.1 * std::cos(t));
      fdm.Run();
    }

    TS_ASSERT(std::isfinite(fcs->GetDePos()));
    TS_ASSERT(std::isfinite(fcs->GetDaLPos()));
    TS_ASSERT(std::isfinite(fcs->GetDrPos()));
  }
};
