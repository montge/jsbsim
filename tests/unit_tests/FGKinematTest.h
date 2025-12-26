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
};
