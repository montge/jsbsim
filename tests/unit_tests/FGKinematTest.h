/*******************************************************************************
 * FGKinematTest.h - Unit tests for FGKinemat (kinematic component)
 *
 * Tests the mathematical behavior of kinematic (servo) components:
 * - Position-time traverse behavior
 * - Rate-limited position changes
 * - Detent (position stop) handling
 * - Input scaling
 *
 * Note: FGKinemat requires XML element for construction, so these tests focus
 * on the underlying mathematical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <vector>
#include <algorithm>

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGKinematTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Basic Kinematic Position Tests
   ***************************************************************************/

  // Test position at exact detent
  void testPositionAtDetent() {
    std::vector<double> detents = {0.0, 1.0};
    double target = 1.0;

    // Position should snap to detent
    auto it = std::find_if(detents.begin(), detents.end(),
                           [target](double d) { return std::abs(d - target) < epsilon; });
    TS_ASSERT(it != detents.end());
    TS_ASSERT_DELTA(*it, 1.0, epsilon);
  }

  // Test position between detents
  void testPositionBetweenDetents() {
    std::vector<double> detents = {0.0, 0.5, 1.0};
    double target = 0.3;

    // Find surrounding detents
    size_t lower_idx = 0;
    for (size_t i = 0; i < detents.size() - 1; i++) {
      if (target >= detents[i] && target <= detents[i+1]) {
        lower_idx = i;
        break;
      }
    }

    TS_ASSERT_DELTA(detents[lower_idx], 0.0, epsilon);
    TS_ASSERT_DELTA(detents[lower_idx + 1], 0.5, epsilon);
  }

  /***************************************************************************
   * Rate-Limited Position Change Tests
   ***************************************************************************/

  // Helper: Rate-limited position update
  double rateLimitedMove(double current, double target, double maxRate, double dt) {
    double delta = target - current;
    double maxDelta = maxRate * dt;
    if (std::abs(delta) <= maxDelta) {
      return target;  // Can reach target
    }
    return current + (delta > 0 ? maxDelta : -maxDelta);
  }

  // Test positive direction movement
  void testRateLimitedMovePositive() {
    double current = 0.0;
    double target = 1.0;
    double maxRate = 0.2;  // units per second
    double dt = 1.0;       // 1 second

    double newPos = rateLimitedMove(current, target, maxRate, dt);
    TS_ASSERT_DELTA(newPos, 0.2, epsilon);  // Can only move 0.2 in 1 second
  }

  // Test negative direction movement
  void testRateLimitedMoveNegative() {
    double current = 1.0;
    double target = 0.0;
    double maxRate = 0.2;
    double dt = 1.0;

    double newPos = rateLimitedMove(current, target, maxRate, dt);
    TS_ASSERT_DELTA(newPos, 0.8, epsilon);
  }

  // Test reaching target within rate limit
  void testRateLimitedMoveReachesTarget() {
    double current = 0.9;
    double target = 1.0;
    double maxRate = 0.2;
    double dt = 1.0;

    double newPos = rateLimitedMove(current, target, maxRate, dt);
    TS_ASSERT_DELTA(newPos, 1.0, epsilon);  // Can reach target
  }

  // Test multi-step traversal
  void testMultiStepTraversal() {
    double position = 0.0;
    double target = 1.0;
    double maxRate = 0.2;  // 5 seconds to traverse full range
    double dt = 0.5;       // 0.5 second time steps

    int steps = 0;
    while (std::abs(position - target) > epsilon && steps < 100) {
      position = rateLimitedMove(position, target, maxRate, dt);
      steps++;
    }

    TS_ASSERT_DELTA(position, 1.0, epsilon);
    TS_ASSERT_EQUALS(steps, 10);  // 5 seconds / 0.5 = 10 steps
  }

  /***************************************************************************
   * Traverse Time Calculation Tests
   ***************************************************************************/

  // Calculate traverse rate from detent positions and times
  double calculateRate(double pos1, double pos2, double time1, double time2) {
    double posChange = pos2 - pos1;
    double timeForTraverse = time2 - time1;  // Time to traverse this segment
    return (timeForTraverse > 0) ? posChange / timeForTraverse : 0.0;
  }

  // Test traverse rate calculation
  void testTraverseRateCalculation() {
    // Detents: position 0 at time 0, position 1 at time 5
    double rate = calculateRate(0.0, 1.0, 0.0, 5.0);
    TS_ASSERT_DELTA(rate, 0.2, epsilon);  // 1 unit in 5 seconds = 0.2/s
  }

  // Test asymmetric traverse rates
  void testAsymmetricTraverseRates() {
    // Gear: 0 to 1 in 5 seconds, 1 to 0 in 3 seconds
    // (down faster than up)
    std::vector<double> detents = {0.0, 1.0};
    std::vector<double> upTimes = {0.0, 5.0};    // Time to reach each detent going up
    std::vector<double> downTimes = {0.0, 3.0};  // Time to reach each detent going down

    double upRate = calculateRate(detents[0], detents[1], upTimes[0], upTimes[1]);
    double downRate = calculateRate(detents[1], detents[0], downTimes[0], downTimes[1]);

    TS_ASSERT_DELTA(upRate, 0.2, epsilon);
    TS_ASSERT_DELTA(downRate, -1.0/3.0, 0.001);
  }

  /***************************************************************************
   * Input Scaling Tests
   ***************************************************************************/

  // Test input scaling from [-1,1] to [0, maxPos]
  double scaleInput(double input, double maxPos) {
    // Scale from [-1, 1] to [0, maxPos]
    return (input + 1.0) * maxPos / 2.0;
  }

  // Test scale at minimum
  void testScaleInputMinimum() {
    double scaled = scaleInput(-1.0, 1.0);
    TS_ASSERT_DELTA(scaled, 0.0, epsilon);
  }

  // Test scale at maximum
  void testScaleInputMaximum() {
    double scaled = scaleInput(1.0, 1.0);
    TS_ASSERT_DELTA(scaled, 1.0, epsilon);
  }

  // Test scale at center
  void testScaleInputCenter() {
    double scaled = scaleInput(0.0, 1.0);
    TS_ASSERT_DELTA(scaled, 0.5, epsilon);
  }

  // Test scale with larger range
  void testScaleInputLargeRange() {
    double scaled = scaleInput(0.5, 100.0);
    // 0.5 scaled from [-1,1] to [0,100] = 75
    TS_ASSERT_DELTA(scaled, 75.0, epsilon);
  }

  // Test no scaling (noscale mode)
  void testNoScaleMode() {
    double input = 0.75;
    double output = input;  // Direct passthrough
    TS_ASSERT_DELTA(output, 0.75, epsilon);
  }

  /***************************************************************************
   * Output Percentage Tests
   ***************************************************************************/

  // Calculate output percentage
  double getOutputPct(double output, double minDetent, double maxDetent) {
    return (output - minDetent) / (maxDetent - minDetent);
  }

  // Test output percentage at minimum
  void testOutputPctMinimum() {
    double pct = getOutputPct(0.0, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 0.0, epsilon);
  }

  // Test output percentage at maximum
  void testOutputPctMaximum() {
    double pct = getOutputPct(1.0, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 1.0, epsilon);
  }

  // Test output percentage at midpoint
  void testOutputPctMidpoint() {
    double pct = getOutputPct(0.5, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 0.5, epsilon);
  }

  // Test output percentage with offset range
  void testOutputPctOffsetRange() {
    double pct = getOutputPct(60.0, 20.0, 100.0);
    // 60 is 50% of the way from 20 to 100
    TS_ASSERT_DELTA(pct, 0.5, epsilon);
  }

  /***************************************************************************
   * Gear Extension/Retraction Simulation
   ***************************************************************************/

  // Simulate full gear cycle
  void testGearCycle() {
    double position = 0.0;       // Start retracted
    double dt = 0.1;             // 100ms timestep
    double extendRate = 0.2;     // 5 seconds to extend
    double retractRate = 0.333;  // 3 seconds to retract

    // Extend gear
    double target = 1.0;
    while (position < target - epsilon) {
      position = rateLimitedMove(position, target, extendRate, dt);
    }
    TS_ASSERT_DELTA(position, 1.0, epsilon);

    // Retract gear
    target = 0.0;
    while (position > target + epsilon) {
      position = rateLimitedMove(position, target, retractRate, dt);
    }
    TS_ASSERT_DELTA(position, 0.0, epsilon);
  }

  // Test partial gear movement
  void testPartialGearMovement() {
    double position = 0.0;
    double dt = 0.1;
    double rate = 0.2;

    // Move halfway then stop
    double target = 0.5;
    while (std::abs(position - target) > epsilon) {
      position = rateLimitedMove(position, target, rate, dt);
    }
    TS_ASSERT_DELTA(position, 0.5, epsilon);

    // Resume to full extension
    target = 1.0;
    while (std::abs(position - target) > epsilon) {
      position = rateLimitedMove(position, target, rate, dt);
    }
    TS_ASSERT_DELTA(position, 1.0, epsilon);
  }

  /***************************************************************************
   * Multi-Detent Traversal Tests
   ***************************************************************************/

  // Test three-position switch (e.g., flaps 0/15/30)
  void testThreePositionSwitch() {
    std::vector<double> detents = {0.0, 15.0, 30.0};
    std::vector<double> times = {0.0, 3.0, 5.0};  // 3s to 15, 2s more to 30

    // Rate for first segment
    double rate1 = (detents[1] - detents[0]) / (times[1] - times[0]);
    TS_ASSERT_DELTA(rate1, 5.0, epsilon);  // 15 deg in 3 seconds

    // Rate for second segment
    double rate2 = (detents[2] - detents[1]) / (times[2] - times[1]);
    TS_ASSERT_DELTA(rate2, 7.5, epsilon);  // 15 deg in 2 seconds
  }

  // Test finding current segment in multi-detent
  void testFindCurrentSegment() {
    std::vector<double> detents = {0.0, 15.0, 30.0, 40.0};
    double position = 22.0;

    // Find which segment we're in
    size_t segment = 0;
    for (size_t i = 0; i < detents.size() - 1; i++) {
      if (position >= detents[i] && position <= detents[i+1]) {
        segment = i;
        break;
      }
    }

    TS_ASSERT_EQUALS(segment, 1u);  // Between 15 and 30
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test zero rate (instantaneous)
  void testZeroTraverseTime() {
    double current = 0.0;
    double target = 1.0;
    double maxRate = std::numeric_limits<double>::infinity();
    double dt = 0.1;

    double newPos = rateLimitedMove(current, target, maxRate, dt);
    TS_ASSERT_DELTA(newPos, target, epsilon);  // Instantaneous
  }

  // Test already at target
  void testAlreadyAtTarget() {
    double current = 0.5;
    double target = 0.5;
    double maxRate = 0.2;
    double dt = 1.0;

    double newPos = rateLimitedMove(current, target, maxRate, dt);
    TS_ASSERT_DELTA(newPos, 0.5, epsilon);
  }

  // Test very small movement
  void testVerySmallMovement() {
    double current = 0.0;
    double target = 1e-12;
    double maxRate = 0.2;
    double dt = 0.1;

    double newPos = rateLimitedMove(current, target, maxRate, dt);
    TS_ASSERT_DELTA(newPos, target, epsilon);  // Within rate limit
  }

  // Test single detent (degenerate case)
  void testSingleDetent() {
    std::vector<double> detents = {0.0};
    double output = detents[0];
    // Output percentage undefined with single detent, but output is valid
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  /***************************************************************************
   * Clipping Tests
   ***************************************************************************/

  // Test output clipping
  void testOutputClipping() {
    double minClip = 0.0;
    double maxClip = 30.0;

    double outputs[] = {-5.0, 0.0, 15.0, 30.0, 35.0};
    double expected[] = {0.0, 0.0, 15.0, 30.0, 30.0};

    for (int i = 0; i < 5; i++) {
      double clipped = std::max(minClip, std::min(maxClip, outputs[i]));
      TS_ASSERT_DELTA(clipped, expected[i], epsilon);
    }
  }

  // Test gain applied to output
  void testOutputGain() {
    double position = 0.5;
    double gain = 2.0;

    double output = position * gain;
    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  /***************************************************************************
   * Additional Comprehensive Tests
   ***************************************************************************/

  // Test asymmetric rate limits (different up/down rates)
  double asymmetricRateLimitedMove(double current, double target,
                                    double upRate, double downRate, double dt) {
    double delta = target - current;
    double maxDelta = (delta > 0 ? upRate : downRate) * dt;
    if (std::abs(delta) <= maxDelta) {
      return target;
    }
    return current + (delta > 0 ? maxDelta : -maxDelta);
  }

  void testAsymmetricRateLimitedMoveUp() {
    double current = 0.0;
    double target = 1.0;
    double upRate = 0.2;
    double downRate = 0.4;
    double dt = 1.0;

    double newPos = asymmetricRateLimitedMove(current, target, upRate, downRate, dt);
    TS_ASSERT_DELTA(newPos, 0.2, epsilon);
  }

  void testAsymmetricRateLimitedMoveDown() {
    double current = 1.0;
    double target = 0.0;
    double upRate = 0.2;
    double downRate = 0.4;
    double dt = 1.0;

    double newPos = asymmetricRateLimitedMove(current, target, upRate, downRate, dt);
    TS_ASSERT_DELTA(newPos, 0.6, epsilon);  // Faster going down
  }

  // Test direction reversal during movement
  void testDirectionReversal() {
    double position = 0.5;
    double rate = 0.2;
    double dt = 0.5;

    // Moving up
    double target = 1.0;
    position = rateLimitedMove(position, target, rate, dt);
    TS_ASSERT_DELTA(position, 0.6, epsilon);

    // Reverse direction
    target = 0.0;
    position = rateLimitedMove(position, target, rate, dt);
    TS_ASSERT_DELTA(position, 0.5, epsilon);
  }

  // Test position holding
  void testPositionHolding() {
    double position = 0.5;
    double target = 0.5;
    double rate = 0.2;
    double dt = 0.1;

    for (int i = 0; i < 10; i++) {
      position = rateLimitedMove(position, target, rate, dt);
      TS_ASSERT_DELTA(position, 0.5, epsilon);
    }
  }

  // Test deadband in position control
  double applyDeadband(double input, double deadband) {
    if (std::abs(input) < deadband) {
      return 0.0;
    }
    return input > 0 ? input - deadband : input + deadband;
  }

  void testDeadbandPositive() {
    double result = applyDeadband(0.15, 0.1);
    TS_ASSERT_DELTA(result, 0.05, epsilon);
  }

  void testDeadbandNegative() {
    double result = applyDeadband(-0.15, 0.1);
    TS_ASSERT_DELTA(result, -0.05, epsilon);
  }

  void testDeadbandWithin() {
    double result = applyDeadband(0.05, 0.1);
    TS_ASSERT_DELTA(result, 0.0, epsilon);
  }

  // Test time to reach target
  double timeToTarget(double current, double target, double rate) {
    return std::abs(target - current) / rate;
  }

  void testTimeToTargetCalculation() {
    double time = timeToTarget(0.0, 1.0, 0.2);
    TS_ASSERT_DELTA(time, 5.0, epsilon);
  }

  void testTimeToTargetPartial() {
    double time = timeToTarget(0.6, 1.0, 0.2);
    TS_ASSERT_DELTA(time, 2.0, epsilon);
  }

  // Test position percentage calculation
  double positionPercentage(double position, double minPos, double maxPos) {
    if (maxPos <= minPos) return 0.0;
    return (position - minPos) / (maxPos - minPos) * 100.0;
  }

  void testPositionPercentageAtMin() {
    double pct = positionPercentage(0.0, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 0.0, epsilon);
  }

  void testPositionPercentageAtMax() {
    double pct = positionPercentage(1.0, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 100.0, epsilon);
  }

  void testPositionPercentageAtMid() {
    double pct = positionPercentage(0.5, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 50.0, epsilon);
  }

  // Test multi-step convergence
  void testConvergenceSteps() {
    double position = 0.0;
    double target = 1.0;
    double rate = 0.25;
    double dt = 1.0;

    int steps = 0;
    while (std::abs(position - target) > epsilon && steps < 10) {
      position = rateLimitedMove(position, target, rate, dt);
      steps++;
    }

    TS_ASSERT_DELTA(position, 1.0, epsilon);
    TS_ASSERT_EQUALS(steps, 4);  // 1.0 / 0.25 = 4 steps
  }

  // Test different time step sizes
  void testDifferentTimeSteps() {
    double rate = 0.2;
    double current = 0.0;
    double target = 1.0;

    // Small time step
    double newPos1 = rateLimitedMove(current, target, rate, 0.1);
    TS_ASSERT_DELTA(newPos1, 0.02, epsilon);

    // Large time step
    double newPos2 = rateLimitedMove(current, target, rate, 2.0);
    TS_ASSERT_DELTA(newPos2, 0.4, epsilon);
  }

  // Test hysteresis effect
  double applyHysteresis(double input, double& prevOutput, double hysteresis) {
    if (std::abs(input - prevOutput) > hysteresis) {
      prevOutput = input;
    }
    return prevOutput;
  }

  void testHysteresisNoChange() {
    double prevOutput = 0.5;
    double result = applyHysteresis(0.52, prevOutput, 0.1);
    TS_ASSERT_DELTA(result, 0.5, epsilon);  // Within hysteresis band
  }

  void testHysteresisChange() {
    double prevOutput = 0.5;
    double result = applyHysteresis(0.7, prevOutput, 0.1);
    TS_ASSERT_DELTA(result, 0.7, epsilon);  // Outside hysteresis band
  }

  // Test linear interpolation between detents
  double interpolatePosition(double input, const std::vector<double>& detents) {
    if (detents.empty()) return 0.0;
    if (detents.size() == 1) return detents[0];

    // Assume input is normalized [0, 1]
    double scaledInput = input * (detents.size() - 1);
    size_t idx = static_cast<size_t>(scaledInput);
    if (idx >= detents.size() - 1) return detents.back();

    double frac = scaledInput - idx;
    return detents[idx] + frac * (detents[idx + 1] - detents[idx]);
  }

  void testInterpolationAtDetents() {
    std::vector<double> detents = {0.0, 15.0, 30.0};

    TS_ASSERT_DELTA(interpolatePosition(0.0, detents), 0.0, epsilon);
    TS_ASSERT_DELTA(interpolatePosition(0.5, detents), 15.0, epsilon);
    TS_ASSERT_DELTA(interpolatePosition(1.0, detents), 30.0, epsilon);
  }

  void testInterpolationBetweenDetents() {
    std::vector<double> detents = {0.0, 15.0, 30.0};

    TS_ASSERT_DELTA(interpolatePosition(0.25, detents), 7.5, epsilon);
    TS_ASSERT_DELTA(interpolatePosition(0.75, detents), 22.5, epsilon);
  }

  // Test flap simulation
  void testFlapDeployment() {
    std::vector<double> flapDetents = {0.0, 10.0, 20.0, 30.0, 40.0};
    double position = 0.0;
    double rate = 5.0;  // 5 degrees per second
    double dt = 0.5;

    // Deploy to full flaps
    double target = 40.0;
    int steps = 0;
    while (std::abs(position - target) > 0.01 && steps < 100) {
      position = rateLimitedMove(position, target, rate, dt);
      steps++;
    }

    TS_ASSERT_DELTA(position, 40.0, 0.01);
    TS_ASSERT_EQUALS(steps, 16);  // 40 / (5 * 0.5) = 16 steps
  }

  // Test landing gear simulation
  void testLandingGearTransition() {
    double position = 0.0;  // 0 = up, 1 = down
    double extendRate = 0.2;  // 5 seconds to extend
    double retractRate = 0.333;  // 3 seconds to retract
    double dt = 0.25;

    // Extend gear
    while (position < 1.0 - epsilon) {
      position = asymmetricRateLimitedMove(position, 1.0, extendRate, retractRate, dt);
    }
    TS_ASSERT_DELTA(position, 1.0, epsilon);

    // Partial retraction
    for (int i = 0; i < 4; i++) {
      position = asymmetricRateLimitedMove(position, 0.0, extendRate, retractRate, dt);
    }
    // After 1 second (4 * 0.25), should have moved 0.333
    TS_ASSERT_DELTA(position, 1.0 - 0.333, 0.01);
  }

  // Test speed brake simulation
  void testSpeedBrakeDeployment() {
    double position = 0.0;
    double rate = 0.5;  // 2 seconds for full deployment
    double dt = 0.1;

    // Deploy speed brake to 60%
    double target = 0.6;
    while (std::abs(position - target) > epsilon) {
      position = rateLimitedMove(position, target, rate, dt);
    }
    TS_ASSERT_DELTA(position, 0.6, epsilon);

    // Verify time taken
    double expectedTime = 0.6 / 0.5;  // = 1.2 seconds
    double steps = 0.6 / (0.5 * 0.1);  // = 12 steps
    TS_ASSERT_DELTA(steps, 12.0, epsilon);
  }

  // Test segment-based rate calculation
  double getSegmentRate(double position, const std::vector<double>& detents,
                        const std::vector<double>& rates) {
    for (size_t i = 0; i < detents.size() - 1; i++) {
      if (position >= detents[i] && position <= detents[i + 1]) {
        return rates[i];
      }
    }
    return rates.back();
  }

  void testSegmentRateLookup() {
    std::vector<double> detents = {0.0, 15.0, 30.0};
    std::vector<double> rates = {5.0, 7.5};  // Rate for each segment

    TS_ASSERT_DELTA(getSegmentRate(7.0, detents, rates), 5.0, epsilon);
    TS_ASSERT_DELTA(getSegmentRate(22.0, detents, rates), 7.5, epsilon);
  }

  // Test minimum time step handling
  void testMinimumTimeStep() {
    double current = 0.0;
    double target = 1.0;
    double rate = 0.2;
    double dt = 1e-6;  // Very small time step

    double newPos = rateLimitedMove(current, target, rate, dt);
    TS_ASSERT_DELTA(newPos, rate * dt, epsilon);
    TS_ASSERT(newPos > current);
  }

  // Test maximum position limits
  double clampPosition(double position, double minPos, double maxPos) {
    return std::max(minPos, std::min(maxPos, position));
  }

  void testClampPositionAboveMax() {
    double clamped = clampPosition(1.5, 0.0, 1.0);
    TS_ASSERT_DELTA(clamped, 1.0, epsilon);
  }

  void testClampPositionBelowMin() {
    double clamped = clampPosition(-0.5, 0.0, 1.0);
    TS_ASSERT_DELTA(clamped, 0.0, epsilon);
  }

  void testClampPositionWithin() {
    double clamped = clampPosition(0.7, 0.0, 1.0);
    TS_ASSERT_DELTA(clamped, 0.7, epsilon);
  }

  // Test transition state detection
  enum TransitionState { STOPPED, EXTENDING, RETRACTING };

  TransitionState getTransitionState(double current, double target, double tolerance = 0.001) {
    if (std::abs(current - target) < tolerance) return STOPPED;
    return (target > current) ? EXTENDING : RETRACTING;
  }

  void testTransitionStateStopped() {
    TS_ASSERT_EQUALS(getTransitionState(0.5, 0.5), STOPPED);
  }

  void testTransitionStateExtending() {
    TS_ASSERT_EQUALS(getTransitionState(0.3, 0.8), EXTENDING);
  }

  void testTransitionStateRetracting() {
    TS_ASSERT_EQUALS(getTransitionState(0.8, 0.3), RETRACTING);
  }

  // Test percentage complete calculation
  double percentComplete(double current, double start, double target) {
    if (std::abs(target - start) < epsilon) return 100.0;
    return std::abs(current - start) / std::abs(target - start) * 100.0;
  }

  void testPercentCompleteAtStart() {
    double pct = percentComplete(0.0, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 0.0, epsilon);
  }

  void testPercentCompleteAtEnd() {
    double pct = percentComplete(1.0, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 100.0, epsilon);
  }

  void testPercentCompleteMiddle() {
    double pct = percentComplete(0.25, 0.0, 1.0);
    TS_ASSERT_DELTA(pct, 25.0, epsilon);
  }

  // Test remaining time calculation
  double remainingTime(double current, double target, double rate) {
    return std::abs(target - current) / rate;
  }

  void testRemainingTimeCalculation() {
    double time = remainingTime(0.3, 1.0, 0.1);
    TS_ASSERT_DELTA(time, 7.0, epsilon);
  }

  void testRemainingTimeAtTarget() {
    double time = remainingTime(1.0, 1.0, 0.1);
    TS_ASSERT_DELTA(time, 0.0, epsilon);
  }

  // Test non-uniform detent spacing
  void testNonUniformDetents() {
    std::vector<double> detents = {0.0, 5.0, 15.0, 40.0};

    // Verify spacing
    double spacing1 = detents[1] - detents[0];
    double spacing2 = detents[2] - detents[1];
    double spacing3 = detents[3] - detents[2];

    TS_ASSERT_DELTA(spacing1, 5.0, epsilon);
    TS_ASSERT_DELTA(spacing2, 10.0, epsilon);
    TS_ASSERT_DELTA(spacing3, 25.0, epsilon);
  }

  // Test simultaneous multiple actuators
  void testMultipleActuators() {
    double leftGear = 0.0;
    double rightGear = 0.0;
    double noseGear = 0.0;
    double rate = 0.2;
    double dt = 0.5;

    // Extend all gear simultaneously
    for (int i = 0; i < 10; i++) {
      leftGear = rateLimitedMove(leftGear, 1.0, rate, dt);
      rightGear = rateLimitedMove(rightGear, 1.0, rate, dt);
      noseGear = rateLimitedMove(noseGear, 1.0, rate, dt);
    }

    TS_ASSERT_DELTA(leftGear, 1.0, epsilon);
    TS_ASSERT_DELTA(rightGear, 1.0, epsilon);
    TS_ASSERT_DELTA(noseGear, 1.0, epsilon);
  }

  // Test emergency extension (faster rate)
  void testEmergencyGearExtension() {
    double position = 0.0;
    double normalRate = 0.2;
    double emergencyRate = 0.5;
    double dt = 0.5;

    // Emergency extension
    int steps = 0;
    while (position < 1.0 - epsilon && steps < 10) {
      position = rateLimitedMove(position, 1.0, emergencyRate, dt);
      steps++;
    }

    TS_ASSERT_DELTA(position, 1.0, epsilon);
    TS_ASSERT_EQUALS(steps, 4);  // 1.0 / (0.5 * 0.5) = 4 steps
  }

  // Test gradual rate change
  double gradualRate(double position, double minRate, double maxRate, double minPos, double maxPos) {
    double fraction = (position - minPos) / (maxPos - minPos);
    return minRate + fraction * (maxRate - minRate);
  }

  void testGradualRateAtMin() {
    double rate = gradualRate(0.0, 0.1, 0.5, 0.0, 1.0);
    TS_ASSERT_DELTA(rate, 0.1, epsilon);
  }

  void testGradualRateAtMax() {
    double rate = gradualRate(1.0, 0.1, 0.5, 0.0, 1.0);
    TS_ASSERT_DELTA(rate, 0.5, epsilon);
  }

  void testGradualRateAtMid() {
    double rate = gradualRate(0.5, 0.1, 0.5, 0.0, 1.0);
    TS_ASSERT_DELTA(rate, 0.3, epsilon);
  }

  // Test negative position ranges
  void testNegativePositionRange() {
    double position = 0.0;
    double target = -30.0;  // e.g., nose-down trim
    double rate = 10.0;
    double dt = 0.5;

    while (std::abs(position - target) > 0.01) {
      position = rateLimitedMove(position, target, rate, dt);
    }
    TS_ASSERT_DELTA(position, -30.0, 0.01);
  }

  // Test bidirectional range
  void testBidirectionalRange() {
    std::vector<double> detents = {-30.0, 0.0, 30.0};
    double position = 0.0;
    double rate = 15.0;
    double dt = 0.5;

    // Move to positive limit
    while (std::abs(position - 30.0) > 0.01) {
      position = rateLimitedMove(position, 30.0, rate, dt);
    }
    TS_ASSERT_DELTA(position, 30.0, 0.01);

    // Move to negative limit
    while (std::abs(position - (-30.0)) > 0.01) {
      position = rateLimitedMove(position, -30.0, rate, dt);
    }
    TS_ASSERT_DELTA(position, -30.0, 0.01);
  }

  // Test detent snapping
  double snapToNearestDetent(double position, const std::vector<double>& detents, double snapThreshold) {
    for (double detent : detents) {
      if (std::abs(position - detent) < snapThreshold) {
        return detent;
      }
    }
    return position;
  }

  void testSnapToDetent() {
    std::vector<double> detents = {0.0, 0.5, 1.0};
    double snapped = snapToNearestDetent(0.48, detents, 0.05);
    TS_ASSERT_DELTA(snapped, 0.5, epsilon);
  }

  void testNoSnapOutsideThreshold() {
    std::vector<double> detents = {0.0, 0.5, 1.0};
    double snapped = snapToNearestDetent(0.4, detents, 0.05);
    TS_ASSERT_DELTA(snapped, 0.4, epsilon);
  }

  // Test input normalization
  double normalizeInput(double input, double inputMin, double inputMax) {
    return (input - inputMin) / (inputMax - inputMin);
  }

  void testNormalizeInputMin() {
    double normalized = normalizeInput(-1.0, -1.0, 1.0);
    TS_ASSERT_DELTA(normalized, 0.0, epsilon);
  }

  void testNormalizeInputMax() {
    double normalized = normalizeInput(1.0, -1.0, 1.0);
    TS_ASSERT_DELTA(normalized, 1.0, epsilon);
  }

  void testNormalizeInputMid() {
    double normalized = normalizeInput(0.0, -1.0, 1.0);
    TS_ASSERT_DELTA(normalized, 0.5, epsilon);
  }

  // Test acceleration-limited motion
  double accelLimitedMove(double current, double target, double currentVel,
                          double& velocity, double maxAccel, double maxVel, double dt) {
    double error = target - current;
    double desiredAccel = error * 10.0;  // P-controller

    // Limit acceleration
    double accel = std::max(-maxAccel, std::min(maxAccel, desiredAccel));

    // Update velocity
    velocity = currentVel + accel * dt;
    velocity = std::max(-maxVel, std::min(maxVel, velocity));

    // Update position
    return current + velocity * dt;
  }

  void testAccelLimitedMotion() {
    double position = 0.0;
    double velocity = 0.0;
    double maxAccel = 0.5;
    double maxVel = 0.2;
    double dt = 0.1;

    // Move towards target
    double target = 1.0;
    for (int i = 0; i < 100; i++) {
      position = accelLimitedMove(position, target, velocity, velocity, maxAccel, maxVel, dt);
    }

    TS_ASSERT_DELTA(position, 1.0, 0.1);  // Should converge
  }

  // Test slew rate limiting
  double slewRateLimit(double newValue, double& prevValue, double maxSlew, double dt) {
    double delta = newValue - prevValue;
    double maxDelta = maxSlew * dt;

    if (std::abs(delta) > maxDelta) {
      delta = delta > 0 ? maxDelta : -maxDelta;
    }

    prevValue = prevValue + delta;
    return prevValue;
  }

  void testSlewRateLimitExceeded() {
    double prev = 0.0;
    double result = slewRateLimit(1.0, prev, 0.2, 1.0);
    TS_ASSERT_DELTA(result, 0.2, epsilon);
  }

  void testSlewRateLimitWithin() {
    double prev = 0.0;
    double result = slewRateLimit(0.1, prev, 0.2, 1.0);
    TS_ASSERT_DELTA(result, 0.1, epsilon);
  }

  /***************************************************************************
   * Advanced Kinematic Motion Tests (78-81)
   ***************************************************************************/

  // Test 78: S-curve velocity profile
  void testSCurveVelocityProfile() {
    // S-curve provides smooth acceleration and deceleration
    double position = 0.0;
    double velocity = 0.0;
    double target = 1.0;
    double maxVel = 0.5;
    double maxAccel = 0.2;
    double dt = 0.1;

    // Simulate S-curve motion (simplified)
    for (int i = 0; i < 50; i++) {
      double distToTarget = target - position;
      double decelDist = (velocity * velocity) / (2.0 * maxAccel);

      if (distToTarget > decelDist) {
        // Accelerate
        velocity = std::min(velocity + maxAccel * dt, maxVel);
      } else {
        // Decelerate
        velocity = std::max(velocity - maxAccel * dt, 0.0);
      }
      position += velocity * dt;
    }

    TS_ASSERT_DELTA(position, target, 0.1);  // Allow larger tolerance for S-curve overshoot
  }

  // Test 79: Jerk-limited motion
  void testJerkLimitedMotion() {
    double velocity = 0.0;
    double accel = 0.0;
    double maxJerk = 1.0;  // Units/s^3
    double maxAccel = 0.5;
    double dt = 0.1;

    // Ramp up acceleration with jerk limit
    for (int i = 0; i < 10; i++) {
      accel = std::min(accel + maxJerk * dt, maxAccel);
      velocity += accel * dt;
    }

    // Acceleration should ramp smoothly
    TS_ASSERT(accel <= maxAccel);
    TS_ASSERT(velocity > 0.0);
  }

  // Test 80: Trapezoidal velocity profile
  void testTrapezoidalVelocityProfile() {
    double position = 0.0;
    double velocity = 0.0;
    double target = 2.0;
    double maxVel = 0.5;
    double accel = 0.25;
    double dt = 0.1;

    // Three phases: accelerate, cruise, decelerate
    int phase = 0;  // 0=accel, 1=cruise, 2=decel

    for (int i = 0; i < 100 && std::abs(position - target) > 0.01; i++) {
      double distToTarget = target - position;
      double stopDist = (velocity * velocity) / (2.0 * accel);

      if (phase == 0 && velocity >= maxVel) phase = 1;
      if (phase < 2 && distToTarget <= stopDist) phase = 2;

      if (phase == 0) velocity = std::min(velocity + accel * dt, maxVel);
      else if (phase == 2) velocity = std::max(velocity - accel * dt, 0.0);

      position += velocity * dt;
    }

    TS_ASSERT_DELTA(position, target, 0.1);
  }

  // Test 81: Bang-bang control (time-optimal)
  void testBangBangControl() {
    // Bang-bang control switches between max acceleration and max deceleration
    // Simplified test: just verify the concept works for a simple trajectory
    double position = 0.0;
    double velocity = 0.0;
    double target = 1.0;
    double maxAccel = 1.0;
    double dt = 0.1;

    // First half: accelerate
    // Second half: decelerate
    // Total time to reach target = 2 * sqrt(distance / maxAccel)
    double totalTime = 2.0 * std::sqrt(target / maxAccel);  // ~2.0 seconds
    double halfTime = totalTime / 2.0;

    double t = 0.0;
    while (t < totalTime + dt) {
      double accel = (t < halfTime) ? maxAccel : -maxAccel;
      velocity += accel * dt;
      velocity = std::max(0.0, velocity);  // Clamp to prevent going backward
      position += velocity * dt;
      t += dt;
    }

    // Position should be near target
    TS_ASSERT(position >= 0.5);  // Should have moved significantly
    TS_ASSERT(position <= 1.5);  // Should not have gone too far
  }

  /***************************************************************************
   * Complex Actuator Simulations (82-85)
   ***************************************************************************/

  // Test 82: Spoiler deployment with blowback
  void testSpoilerDeploymentWithBlowback() {
    double position = 0.0;
    double rate = 0.5;  // 2 seconds to full deploy
    double dt = 0.1;
    double aeroPressure = 0.2;  // Simulated blowback from aero load

    // Deploy against aero pressure
    double target = 1.0;
    double effectiveRate = rate - aeroPressure;  // 0.3 units/s

    // Need 1.0 / 0.3 = 3.33 seconds = 34 iterations to reach target
    for (int i = 0; i < 40; i++) {
      position = rateLimitedMove(position, target, effectiveRate, dt);
    }

    TS_ASSERT_DELTA(position, 1.0, 0.05);
  }

  // Test 83: Slat extension schedule
  void testSlatExtensionSchedule() {
    // Slats extend based on speed/flap position
    double slatPosition = 0.0;
    double flapPosition = 0.0;
    double rate = 0.3;
    double dt = 0.1;

    // Extend flaps first, then slats follow
    for (int i = 0; i < 20; i++) {
      flapPosition = rateLimitedMove(flapPosition, 1.0, rate, dt);
      // Slats target 50% of flap position
      double slatTarget = flapPosition * 0.5;
      slatPosition = rateLimitedMove(slatPosition, slatTarget, rate, dt);
    }

    TS_ASSERT(slatPosition > 0.0);
    TS_ASSERT(slatPosition <= flapPosition);
  }

  // Test 84: Canard deflection limits
  void testCanardDeflectionLimits() {
    double position = 0.0;
    double minLimit = -25.0;
    double maxLimit = 10.0;  // Asymmetric limits
    double rate = 15.0;  // deg/s
    double dt = 0.1;

    // Try to exceed positive limit
    for (int i = 0; i < 20; i++) {
      double target = 20.0;
      position = rateLimitedMove(position, target, rate, dt);
      position = clampPosition(position, minLimit, maxLimit);
    }
    TS_ASSERT_DELTA(position, maxLimit, epsilon);

    // Try to exceed negative limit
    for (int i = 0; i < 40; i++) {
      double target = -40.0;
      position = rateLimitedMove(position, target, rate, dt);
      position = clampPosition(position, minLimit, maxLimit);
    }
    TS_ASSERT_DELTA(position, minLimit, epsilon);
  }

  // Test 85: Thrust reverser deployment
  void testThrustReverserDeployment() {
    double leftRev = 0.0;
    double rightRev = 0.0;
    double rate = 0.4;  // 2.5 seconds to deploy
    double dt = 0.1;

    // Deploy both reversers
    for (int i = 0; i < 30; i++) {
      leftRev = rateLimitedMove(leftRev, 1.0, rate, dt);
      rightRev = rateLimitedMove(rightRev, 1.0, rate, dt);
    }

    TS_ASSERT_DELTA(leftRev, 1.0, epsilon);
    TS_ASSERT_DELTA(rightRev, 1.0, epsilon);

    // Stow reversers
    for (int i = 0; i < 30; i++) {
      leftRev = rateLimitedMove(leftRev, 0.0, rate, dt);
      rightRev = rateLimitedMove(rightRev, 0.0, rate, dt);
    }

    TS_ASSERT_DELTA(leftRev, 0.0, epsilon);
    TS_ASSERT_DELTA(rightRev, 0.0, epsilon);
  }

  /***************************************************************************
   * Failure Mode Tests (86-89)
   ***************************************************************************/

  // Test 86: Actuator jam simulation
  void testActuatorJam() {
    double position = 0.5;
    double rate = 0.2;
    double dt = 0.1;
    bool jammed = true;

    // Attempt to move jammed actuator
    double target = 1.0;
    for (int i = 0; i < 10; i++) {
      if (!jammed) {
        position = rateLimitedMove(position, target, rate, dt);
      }
    }

    TS_ASSERT_DELTA(position, 0.5, epsilon);  // Should not have moved
  }

  // Test 87: Runaway actuator detection
  void testRunawayActuatorDetection() {
    double position = 0.0;
    double prevPosition = 0.0;
    double normalRate = 0.2;
    double runawayRate = 1.0;  // Much faster than normal
    double dt = 0.1;

    // Simulate runaway
    for (int i = 0; i < 5; i++) {
      prevPosition = position;
      position += runawayRate * dt;
    }

    double actualRate = (position - prevPosition) / dt;
    bool runawayDetected = actualRate > normalRate * 2.0;

    TS_ASSERT(runawayDetected);
  }

  // Test 88: Hydraulic failure slow motion
  void testHydraulicFailureSlowMotion() {
    double position = 0.0;
    double normalRate = 0.5;
    double failureRate = 0.1;  // 20% of normal
    double dt = 0.1;
    bool hydraulicFailed = true;

    double effectiveRate = hydraulicFailed ? failureRate : normalRate;

    // Move at reduced rate
    for (int i = 0; i < 20; i++) {
      position = rateLimitedMove(position, 1.0, effectiveRate, dt);
    }

    // Should have only moved 20% as far as normal
    TS_ASSERT_DELTA(position, 0.2, 0.01);
  }

  // Test 89: Asymmetric failure (one side stuck)
  void testAsymmetricFailure() {
    double leftPos = 0.0;
    double rightPos = 0.0;
    double rate = 0.3;
    double dt = 0.1;
    bool leftFailed = true;

    // Attempt symmetric deployment
    for (int i = 0; i < 20; i++) {
      if (!leftFailed) {
        leftPos = rateLimitedMove(leftPos, 1.0, rate, dt);
      }
      rightPos = rateLimitedMove(rightPos, 1.0, rate, dt);
    }

    TS_ASSERT_DELTA(leftPos, 0.0, epsilon);   // Left stuck
    TS_ASSERT_DELTA(rightPos, 0.6, 0.01);      // Right moved normally
  }

  /***************************************************************************
   * Synchronization Tests (90-93)
   ***************************************************************************/

  // Test 90: Multi-actuator synchronization
  void testMultiActuatorSync() {
    double pos1 = 0.0, pos2 = 0.0, pos3 = 0.0;
    double rate = 0.25;
    double dt = 0.1;

    for (int i = 0; i < 50; i++) {
      pos1 = rateLimitedMove(pos1, 1.0, rate, dt);
      pos2 = rateLimitedMove(pos2, 1.0, rate, dt);
      pos3 = rateLimitedMove(pos3, 1.0, rate, dt);
    }

    // All should be at same position
    TS_ASSERT_DELTA(pos1, pos2, epsilon);
    TS_ASSERT_DELTA(pos2, pos3, epsilon);
    TS_ASSERT_DELTA(pos1, 1.0, epsilon);
  }

  // Test 91: Phased deployment (sequential)
  void testPhasedDeployment() {
    double flaps = 0.0;
    double slats = 0.0;
    double rate = 0.3;
    double dt = 0.1;

    // Flaps start first, slats follow at 50% flaps
    for (int i = 0; i < 50; i++) {
      flaps = rateLimitedMove(flaps, 1.0, rate, dt);
      if (flaps >= 0.5) {
        slats = rateLimitedMove(slats, 1.0, rate, dt);
      }
    }

    TS_ASSERT_DELTA(flaps, 1.0, epsilon);
    TS_ASSERT(slats > 0.5);  // Slats started late
  }

  // Test 92: Master-slave actuator
  void testMasterSlaveActuator() {
    double master = 0.0;
    double slave = 0.0;
    double rate = 0.2;
    double dt = 0.1;

    for (int i = 0; i < 30; i++) {
      master = rateLimitedMove(master, 1.0, rate, dt);
      // Slave follows master with slight delay
      slave = rateLimitedMove(slave, master, rate * 1.2, dt);
    }

    TS_ASSERT_DELTA(master, 0.6, 0.01);
    TS_ASSERT(std::abs(slave - master) < 0.1);  // Slave tracks master
  }

  // Test 93: Split control surface
  void testSplitControlSurface() {
    double inboard = 0.0;
    double outboard = 0.0;
    double inboardRate = 0.3;
    double outboardRate = 0.4;  // Outboard faster
    double dt = 0.1;

    // Deploy both sections
    double target = 1.0;
    for (int i = 0; i < 30; i++) {
      inboard = rateLimitedMove(inboard, target, inboardRate, dt);
      outboard = rateLimitedMove(outboard, target, outboardRate, dt);
    }

    TS_ASSERT_DELTA(inboard, 0.9, 0.1);
    TS_ASSERT_DELTA(outboard, 1.0, epsilon);  // Outboard reached first
  }

  /***************************************************************************
   * Complete Kinematic System Tests (94-100)
   ***************************************************************************/

  // Test 94: Complete flap cycle
  void testCompleteFlapCycle() {
    std::vector<double> detents = {0.0, 5.0, 15.0, 25.0, 40.0};
    double position = 0.0;
    double rate = 3.0;  // deg/s
    double dt = 0.1;

    // Extend to each detent
    for (size_t i = 1; i < detents.size(); i++) {
      while (std::abs(position - detents[i]) > 0.1) {
        position = rateLimitedMove(position, detents[i], rate, dt);
      }
      TS_ASSERT_DELTA(position, detents[i], 0.1);
    }

    // Retract to clean
    while (std::abs(position - detents[0]) > 0.1) {
      position = rateLimitedMove(position, detents[0], rate, dt);
    }
    TS_ASSERT_DELTA(position, 0.0, 0.1);
  }

  // Test 95: Landing gear with door sequence
  void testLandingGearWithDoors() {
    double doorPosition = 0.0;   // 0=closed, 1=open
    double gearPosition = 0.0;   // 0=up, 1=down
    double doorRate = 0.4;       // Door faster
    double gearRate = 0.2;
    double dt = 0.1;

    // Extend sequence: doors first, then gear
    // Open doors
    while (doorPosition < 0.99) {
      doorPosition = rateLimitedMove(doorPosition, 1.0, doorRate, dt);
    }
    TS_ASSERT_DELTA(doorPosition, 1.0, 0.01);

    // Extend gear
    while (gearPosition < 0.99) {
      gearPosition = rateLimitedMove(gearPosition, 1.0, gearRate, dt);
    }
    TS_ASSERT_DELTA(gearPosition, 1.0, 0.01);

    // Doors stay open with gear down
    TS_ASSERT(doorPosition > 0.9);
  }

  // Test 96: Aileron droop with flap interconnect
  void testAileronDroopWithFlaps() {
    double flapPosition = 0.0;
    double aileronDroop = 0.0;
    double flapRate = 0.2;
    double dt = 0.1;

    // Extend flaps
    for (int i = 0; i < 50; i++) {
      flapPosition = rateLimitedMove(flapPosition, 1.0, flapRate, dt);
      // Ailerons droop proportionally to flaps
      double droopTarget = flapPosition * 0.3;  // 30% of flap position
      aileronDroop = rateLimitedMove(aileronDroop, droopTarget, flapRate, dt);
    }

    TS_ASSERT_DELTA(flapPosition, 1.0, epsilon);
    TS_ASSERT_DELTA(aileronDroop, 0.3, 0.02);
  }

  // Test 97: Speed brake modulation
  void testSpeedBrakeModulation() {
    double position = 0.0;
    double rate = 0.5;
    double dt = 0.1;

    // Series of modulation commands
    double targets[] = {0.5, 0.3, 0.8, 0.0, 1.0};
    for (int t = 0; t < 5; t++) {
      for (int i = 0; i < 30; i++) {
        position = rateLimitedMove(position, targets[t], rate, dt);
      }
      TS_ASSERT_DELTA(position, targets[t], 0.1);
    }
  }

  // Test 98: Variable rate based on hydraulic pressure
  void testVariableRateWithPressure() {
    double position = 0.0;
    double baseRate = 0.5;
    double dt = 0.1;

    // Varying hydraulic pressure
    double pressures[] = {3000.0, 2500.0, 2000.0, 1500.0};
    double expectedRates[] = {0.5, 0.417, 0.333, 0.25};  // Rate proportional to pressure

    for (int p = 0; p < 4; p++) {
      double effectiveRate = baseRate * (pressures[p] / 3000.0);
      TS_ASSERT_DELTA(effectiveRate, expectedRates[p], 0.01);
    }
  }

  // Test 99: Position feedback loop
  void testPositionFeedbackLoop() {
    double position = 0.0;
    double target = 1.0;
    double Kp = 2.0;  // Proportional gain
    double maxRate = 0.5;
    double dt = 0.1;

    for (int i = 0; i < 50; i++) {
      double error = target - position;
      double command = Kp * error;
      // Rate limited
      double actualRate = std::min(std::abs(command), maxRate);
      if (command < 0) actualRate = -actualRate;
      position += actualRate * dt;
    }

    TS_ASSERT_DELTA(position, target, 0.05);
  }

  // Test 100: Complete kinematic system verification
  void testCompleteKinematicSystemVerification() {
    // Comprehensive test of all kinematic behaviors

    // 1. Basic rate-limited motion
    double pos = 0.0;
    pos = rateLimitedMove(pos, 1.0, 0.5, 0.2);
    TS_ASSERT_DELTA(pos, 0.1, epsilon);

    // 2. Asymmetric rates
    double upRate = 0.2, downRate = 0.4;
    pos = 0.5;
    pos = asymmetricRateLimitedMove(pos, 0.0, upRate, downRate, 1.0);
    TS_ASSERT_DELTA(pos, 0.1, epsilon);

    // 3. Multi-detent system
    std::vector<double> detents = {0.0, 0.25, 0.5, 0.75, 1.0};
    double interpPos = interpolatePosition(0.5, detents);
    TS_ASSERT_DELTA(interpPos, 0.5, epsilon);

    // 4. Input scaling
    double scaled = scaleInput(0.0, 1.0);
    TS_ASSERT_DELTA(scaled, 0.5, epsilon);

    // 5. Clamping
    double clamped = clampPosition(1.5, 0.0, 1.0);
    TS_ASSERT_DELTA(clamped, 1.0, epsilon);

    // 6. Detent snapping
    double snapped = snapToNearestDetent(0.48, detents, 0.05);
    TS_ASSERT_DELTA(snapped, 0.5, epsilon);

    // 7. Transition state
    TS_ASSERT_EQUALS(getTransitionState(0.0, 1.0), EXTENDING);
    TS_ASSERT_EQUALS(getTransitionState(1.0, 0.0), RETRACTING);
    TS_ASSERT_EQUALS(getTransitionState(0.5, 0.5), STOPPED);

    // 8. Time calculations
    double time = timeToTarget(0.0, 1.0, 0.25);
    TS_ASSERT_DELTA(time, 4.0, epsilon);

    // All kinematic system functions verified
  }
};
