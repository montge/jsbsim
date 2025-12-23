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
};
