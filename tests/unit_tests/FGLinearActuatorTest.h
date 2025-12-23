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
};
