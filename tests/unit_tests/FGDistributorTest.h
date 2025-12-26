/*******************************************************************************
 * FGDistributorTest.h - Unit tests for FGDistributor (conditional logic)
 *
 * Tests the mathematical behavior of distributor components:
 * - Exclusive case evaluation (first match wins)
 * - Inclusive case evaluation (all matching cases)
 * - Default case handling
 * - Complex conditional logic
 *
 * Note: FGDistributor requires XML element for construction, so these tests
 * focus on the underlying conditional logic.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <vector>
#include <functional>

const double epsilon = 1e-10;

class FGDistributorTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Exclusive Case Evaluation Tests
   * (First matching case wins, stop evaluating)
   ***************************************************************************/

  // Simulate exclusive distributor
  double exclusiveDistributor(double input,
                              const std::vector<std::pair<std::function<bool(double)>, double>>& cases,
                              double defaultValue) {
    for (const auto& c : cases) {
      if (c.first(input)) {
        return c.second;
      }
    }
    return defaultValue;
  }

  // Test first case matches
  void testExclusiveFirstCaseMatches() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x < 0; }, -1.0},
      {[](double x) { return x >= 0; }, 1.0}
    };

    double result = exclusiveDistributor(-5.0, cases, 0.0);
    TS_ASSERT_DELTA(result, -1.0, epsilon);
  }

  // Test second case matches
  void testExclusiveSecondCaseMatches() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x < 0; }, -1.0},
      {[](double x) { return x >= 0; }, 1.0}
    };

    double result = exclusiveDistributor(5.0, cases, 0.0);
    TS_ASSERT_DELTA(result, 1.0, epsilon);
  }

  // Test exclusive stops at first match
  void testExclusiveStopsAtFirstMatch() {
    // Both conditions would match for x=5, but exclusive stops at first
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x > 0; }, 1.0},
      {[](double x) { return x > 3; }, 2.0}
    };

    double result = exclusiveDistributor(5.0, cases, 0.0);
    TS_ASSERT_DELTA(result, 1.0, epsilon);  // First match wins
  }

  // Test default case when no match
  void testExclusiveDefaultCase() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x < -10; }, -1.0},
      {[](double x) { return x > 10; }, 1.0}
    };

    double result = exclusiveDistributor(0.0, cases, 999.0);
    TS_ASSERT_DELTA(result, 999.0, epsilon);  // Default
  }

  /***************************************************************************
   * Inclusive Case Evaluation Tests
   * (All matching cases are evaluated)
   ***************************************************************************/

  // Simulate inclusive distributor (returns sum of matching values)
  std::vector<double> inclusiveDistributor(double input,
                                           const std::vector<std::pair<std::function<bool(double)>, double>>& cases) {
    std::vector<double> results;
    for (const auto& c : cases) {
      if (c.first(input)) {
        results.push_back(c.second);
      }
    }
    return results;
  }

  // Test multiple cases match
  void testInclusiveMultipleMatches() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x > 0; }, 1.0},
      {[](double x) { return x > 3; }, 2.0},
      {[](double x) { return x > 7; }, 3.0}
    };

    auto results = inclusiveDistributor(5.0, cases);
    TS_ASSERT_EQUALS(results.size(), 2u);  // Two matches: >0 and >3
  }

  // Test all cases match
  void testInclusiveAllMatch() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x > 0; }, 1.0},
      {[](double x) { return x > 3; }, 2.0},
      {[](double x) { return x > 7; }, 3.0}
    };

    auto results = inclusiveDistributor(10.0, cases);
    TS_ASSERT_EQUALS(results.size(), 3u);  // All three match
  }

  // Test no cases match
  void testInclusiveNoMatch() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x > 10; }, 1.0},
      {[](double x) { return x < -10; }, 2.0}
    };

    auto results = inclusiveDistributor(0.0, cases);
    TS_ASSERT_EQUALS(results.size(), 0u);
  }

  /***************************************************************************
   * Condition Tests
   ***************************************************************************/

  // Test less than
  void testConditionLT() {
    auto lt = [](double a, double b) { return a < b; };
    TS_ASSERT(lt(5.0, 10.0));
    TS_ASSERT(!lt(10.0, 5.0));
    TS_ASSERT(!lt(5.0, 5.0));
  }

  // Test greater than
  void testConditionGT() {
    auto gt = [](double a, double b) { return a > b; };
    TS_ASSERT(gt(10.0, 5.0));
    TS_ASSERT(!gt(5.0, 10.0));
    TS_ASSERT(!gt(5.0, 5.0));
  }

  // Test less than or equal
  void testConditionLE() {
    auto le = [](double a, double b) { return a <= b; };
    TS_ASSERT(le(5.0, 10.0));
    TS_ASSERT(!le(10.0, 5.0));
    TS_ASSERT(le(5.0, 5.0));
  }

  // Test greater than or equal
  void testConditionGE() {
    auto ge = [](double a, double b) { return a >= b; };
    TS_ASSERT(ge(10.0, 5.0));
    TS_ASSERT(!ge(5.0, 10.0));
    TS_ASSERT(ge(5.0, 5.0));
  }

  // Test equality
  void testConditionEQ() {
    auto eq = [](double a, double b) { return std::abs(a - b) < 1e-10; };
    TS_ASSERT(eq(5.0, 5.0));
    TS_ASSERT(!eq(5.0, 5.1));
  }

  // Test not equal
  void testConditionNE() {
    auto ne = [](double a, double b) { return std::abs(a - b) >= 1e-10; };
    TS_ASSERT(!ne(5.0, 5.0));
    TS_ASSERT(ne(5.0, 5.1));
  }

  /***************************************************************************
   * Compound Condition Tests
   ***************************************************************************/

  // Test AND logic
  void testCompoundAND() {
    auto andLogic = [](bool a, bool b) { return a && b; };

    TS_ASSERT(andLogic(true, true));
    TS_ASSERT(!andLogic(true, false));
    TS_ASSERT(!andLogic(false, true));
    TS_ASSERT(!andLogic(false, false));
  }

  // Test OR logic
  void testCompoundOR() {
    auto orLogic = [](bool a, bool b) { return a || b; };

    TS_ASSERT(orLogic(true, true));
    TS_ASSERT(orLogic(true, false));
    TS_ASSERT(orLogic(false, true));
    TS_ASSERT(!orLogic(false, false));
  }

  // Test complex compound condition
  void testComplexCondition() {
    // (x > 0 AND x < 10) OR x == -5
    auto complex = [](double x) {
      return (x > 0 && x < 10) || std::abs(x - (-5)) < epsilon;
    };

    TS_ASSERT(complex(5.0));    // In range
    TS_ASSERT(complex(-5.0));   // Equals -5
    TS_ASSERT(!complex(15.0));  // Out of range
    TS_ASSERT(!complex(-10.0)); // Not -5, not in range
  }

  /***************************************************************************
   * Sign Determination Example
   ***************************************************************************/

  // Replicate the sign determination example from docs
  void testSignDetermination() {
    auto determineSign = [](double x) -> double {
      if (x < 0.0) return -1.0;
      if (x >= 0.0) return 1.0;
      return 0.0;  // Default (never reached)
    };

    TS_ASSERT_DELTA(determineSign(-5.0), -1.0, epsilon);
    TS_ASSERT_DELTA(determineSign(5.0), 1.0, epsilon);
    TS_ASSERT_DELTA(determineSign(0.0), 1.0, epsilon);  // >= 0
  }

  /***************************************************************************
   * Range Mapping Example
   ***************************************************************************/

  // Map value to range
  void testRangeMapping() {
    auto mapToRange = [](double x) -> int {
      if (x < 0) return 0;
      if (x < 10) return 1;
      if (x < 20) return 2;
      if (x < 30) return 3;
      return 4;  // >= 30
    };

    TS_ASSERT_EQUALS(mapToRange(-5.0), 0);
    TS_ASSERT_EQUALS(mapToRange(5.0), 1);
    TS_ASSERT_EQUALS(mapToRange(15.0), 2);
    TS_ASSERT_EQUALS(mapToRange(25.0), 3);
    TS_ASSERT_EQUALS(mapToRange(35.0), 4);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test boundary conditions
  void testBoundaryConditions() {
    auto boundary = [](double x) -> double {
      if (x < 10.0) return 0.0;
      return 1.0;
    };

    TS_ASSERT_DELTA(boundary(9.999999), 0.0, epsilon);
    TS_ASSERT_DELTA(boundary(10.0), 1.0, epsilon);
    TS_ASSERT_DELTA(boundary(10.000001), 1.0, epsilon);
  }

  // Test with NaN
  void testWithNaN() {
    double nan = std::numeric_limits<double>::quiet_NaN();

    // NaN comparisons are always false
    TS_ASSERT(!(nan < 0));
    TS_ASSERT(!(nan > 0));
    TS_ASSERT(!(nan == 0));
  }

  // Test with infinity
  void testWithInfinity() {
    double inf = std::numeric_limits<double>::infinity();
    double neg_inf = -std::numeric_limits<double>::infinity();

    TS_ASSERT(inf > 1e308);
    TS_ASSERT(neg_inf < -1e308);
    TS_ASSERT(1000.0 < inf);
    TS_ASSERT(-1000.0 > neg_inf);
  }

  /***************************************************************************
   * Property Value Setting
   ***************************************************************************/

  // Test property value assignment
  void testPropertyValueAssignment() {
    double property = 0.0;
    double value = 42.0;

    property = value;
    TS_ASSERT_DELTA(property, 42.0, epsilon);
  }

  // Test conditional property update
  void testConditionalPropertyUpdate() {
    double property = 0.0;
    double input = 5.0;

    if (input > 0) {
      property = 1.0;
    }

    TS_ASSERT_DELTA(property, 1.0, epsilon);
  }

  /***************************************************************************
   * Extended Exclusive Case Tests
   ***************************************************************************/

  // Test with many cases
  void testExclusiveManyCases() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x < -100; }, -3.0},
      {[](double x) { return x < -10; }, -2.0},
      {[](double x) { return x < 0; }, -1.0},
      {[](double x) { return x == 0; }, 0.0},
      {[](double x) { return x < 10; }, 1.0},
      {[](double x) { return x < 100; }, 2.0},
      {[](double x) { return x >= 100; }, 3.0}
    };

    TS_ASSERT_DELTA(exclusiveDistributor(-150.0, cases, 999.0), -3.0, epsilon);
    TS_ASSERT_DELTA(exclusiveDistributor(-50.0, cases, 999.0), -2.0, epsilon);
    TS_ASSERT_DELTA(exclusiveDistributor(-5.0, cases, 999.0), -1.0, epsilon);
    TS_ASSERT_DELTA(exclusiveDistributor(5.0, cases, 999.0), 1.0, epsilon);
    TS_ASSERT_DELTA(exclusiveDistributor(50.0, cases, 999.0), 2.0, epsilon);
    TS_ASSERT_DELTA(exclusiveDistributor(150.0, cases, 999.0), 3.0, epsilon);
  }

  // Test with empty cases list
  void testExclusiveEmptyCases() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases;

    double result = exclusiveDistributor(5.0, cases, 42.0);
    TS_ASSERT_DELTA(result, 42.0, epsilon);  // Default
  }

  // Test with single case that matches
  void testExclusiveSingleCaseMatch() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return true; }, 100.0}
    };

    double result = exclusiveDistributor(5.0, cases, 0.0);
    TS_ASSERT_DELTA(result, 100.0, epsilon);
  }

  // Test with single case that doesn't match
  void testExclusiveSingleCaseNoMatch() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return false; }, 100.0}
    };

    double result = exclusiveDistributor(5.0, cases, 0.0);
    TS_ASSERT_DELTA(result, 0.0, epsilon);
  }

  // Test order matters in exclusive
  void testExclusiveOrderMatters() {
    // Case A: More specific first
    std::vector<std::pair<std::function<bool(double)>, double>> casesA = {
      {[](double x) { return x > 5 && x < 10; }, 1.0},  // Specific
      {[](double x) { return x > 0; }, 2.0}              // General
    };

    // Case B: General first
    std::vector<std::pair<std::function<bool(double)>, double>> casesB = {
      {[](double x) { return x > 0; }, 2.0},             // General
      {[](double x) { return x > 5 && x < 10; }, 1.0}   // Specific
    };

    // x=7 is in both ranges
    double resultA = exclusiveDistributor(7.0, casesA, 0.0);
    double resultB = exclusiveDistributor(7.0, casesB, 0.0);

    TS_ASSERT_DELTA(resultA, 1.0, epsilon);  // Specific wins when first
    TS_ASSERT_DELTA(resultB, 2.0, epsilon);  // General wins when first
  }

  /***************************************************************************
   * Extended Inclusive Case Tests
   ***************************************************************************/

  // Test inclusive with overlapping ranges
  void testInclusiveOverlappingRanges() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x >= 0 && x <= 10; }, 1.0},
      {[](double x) { return x >= 5 && x <= 15; }, 2.0},
      {[](double x) { return x >= 10 && x <= 20; }, 3.0}
    };

    auto results = inclusiveDistributor(10.0, cases);
    TS_ASSERT_EQUALS(results.size(), 3u);  // 10 is in all three ranges
  }

  // Test inclusive order preserved
  void testInclusiveOrderPreserved() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return true; }, 1.0},
      {[](double x) { return true; }, 2.0},
      {[](double x) { return true; }, 3.0}
    };

    auto results = inclusiveDistributor(5.0, cases);
    TS_ASSERT_EQUALS(results.size(), 3u);
    TS_ASSERT_DELTA(results[0], 1.0, epsilon);
    TS_ASSERT_DELTA(results[1], 2.0, epsilon);
    TS_ASSERT_DELTA(results[2], 3.0, epsilon);
  }

  // Test sum of inclusive results
  void testInclusiveSum() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x > 0; }, 10.0},
      {[](double x) { return x > 5; }, 20.0},
      {[](double x) { return x > 10; }, 30.0}
    };

    auto results = inclusiveDistributor(15.0, cases);
    double sum = 0.0;
    for (double r : results) sum += r;

    TS_ASSERT_DELTA(sum, 60.0, epsilon);  // All three match
  }

  /***************************************************************************
   * Flight Control Distribution Tests
   ***************************************************************************/

  // Test flap schedule distribution
  void testFlapSchedule() {
    auto flapPosition = [](double airspeed, double handle) -> double {
      // Simplified flap schedule based on airspeed
      if (airspeed > 200.0) return 0.0;       // No flaps at high speed
      if (airspeed > 150.0) return handle * 0.5;  // Partial flaps
      return handle;  // Full commanded flaps
    };

    TS_ASSERT_DELTA(flapPosition(250.0, 30.0), 0.0, epsilon);
    TS_ASSERT_DELTA(flapPosition(175.0, 30.0), 15.0, epsilon);
    TS_ASSERT_DELTA(flapPosition(100.0, 30.0), 30.0, epsilon);
  }

  // Test landing gear distribution
  void testLandingGearLogic() {
    auto gearCommand = [](double handlePos, double altitude, double airspeed) -> double {
      // Only allow gear down below 10000 ft and 250 kts
      if (handlePos > 0.5 && altitude < 10000.0 && airspeed < 250.0) {
        return 1.0;  // Gear down
      }
      if (handlePos < 0.5) {
        return 0.0;  // Gear up
      }
      return 0.5;  // Transit/blocked
    };

    TS_ASSERT_DELTA(gearCommand(1.0, 5000.0, 200.0), 1.0, epsilon);
    TS_ASSERT_DELTA(gearCommand(1.0, 15000.0, 200.0), 0.5, epsilon);
    TS_ASSERT_DELTA(gearCommand(0.0, 5000.0, 200.0), 0.0, epsilon);
  }

  // Test spoiler distribution
  void testSpoilerDistribution() {
    auto spoilerDeploy = [](bool onGround, double throttle, double speedbrakeHandle) -> double {
      // Ground spoilers: full deploy on landing
      if (onGround && throttle < 0.1) return 60.0;
      // Speed brakes: proportional to handle in flight
      if (!onGround && speedbrakeHandle > 0) return speedbrakeHandle * 45.0;
      return 0.0;
    };

    TS_ASSERT_DELTA(spoilerDeploy(true, 0.0, 0.0), 60.0, epsilon);
    TS_ASSERT_DELTA(spoilerDeploy(false, 0.5, 1.0), 45.0, epsilon);
    TS_ASSERT_DELTA(spoilerDeploy(false, 0.5, 0.5), 22.5, epsilon);
    TS_ASSERT_DELTA(spoilerDeploy(false, 0.5, 0.0), 0.0, epsilon);
  }

  // Test autopilot mode selection
  void testAutopilotModeSelection() {
    enum APMode { OFF = 0, ALT_HOLD = 1, VS_MODE = 2, ILS_APPROACH = 3 };

    auto selectMode = [](bool apEngaged, bool altCapture, bool ilsArmed, bool ilsCapture) -> int {
      if (!apEngaged) return OFF;
      if (ilsCapture) return ILS_APPROACH;
      if (ilsArmed && !altCapture) return VS_MODE;
      if (altCapture) return ALT_HOLD;
      return VS_MODE;
    };

    TS_ASSERT_EQUALS(selectMode(false, false, false, false), OFF);
    TS_ASSERT_EQUALS(selectMode(true, true, false, false), ALT_HOLD);
    TS_ASSERT_EQUALS(selectMode(true, false, true, true), ILS_APPROACH);
    TS_ASSERT_EQUALS(selectMode(true, false, true, false), VS_MODE);
  }

  /***************************************************************************
   * Priority Logic Tests
   ***************************************************************************/

  // Test priority-based selection
  void testPrioritySelection() {
    auto selectByPriority = [](bool emergency, bool warning, bool caution, bool advisory) -> int {
      if (emergency) return 4;
      if (warning) return 3;
      if (caution) return 2;
      if (advisory) return 1;
      return 0;
    };

    TS_ASSERT_EQUALS(selectByPriority(true, true, true, true), 4);
    TS_ASSERT_EQUALS(selectByPriority(false, true, true, true), 3);
    TS_ASSERT_EQUALS(selectByPriority(false, false, true, true), 2);
    TS_ASSERT_EQUALS(selectByPriority(false, false, false, true), 1);
    TS_ASSERT_EQUALS(selectByPriority(false, false, false, false), 0);
  }

  // Test weighted priority
  void testWeightedPriority() {
    auto weightedSelect = [](double p1, double w1, double p2, double w2, double p3, double w3) {
      double sumWeights = w1 + w2 + w3;
      return (p1 * w1 + p2 * w2 + p3 * w3) / sumWeights;
    };

    double result = weightedSelect(10.0, 1.0, 20.0, 2.0, 30.0, 3.0);
    // (10*1 + 20*2 + 30*3) / 6 = (10 + 40 + 90) / 6 = 140/6 = 23.33...
    TS_ASSERT_DELTA(result, 23.333, 0.01);
  }

  /***************************************************************************
   * State Machine Logic Tests
   ***************************************************************************/

  // Test state transition logic
  void testStateTransition() {
    enum State { IDLE, RUNNING, PAUSED, STOPPED };

    auto transition = [](int currentState, const std::string& event) -> int {
      if (currentState == IDLE && event == "start") return RUNNING;
      if (currentState == RUNNING && event == "pause") return PAUSED;
      if (currentState == RUNNING && event == "stop") return STOPPED;
      if (currentState == PAUSED && event == "resume") return RUNNING;
      if (currentState == PAUSED && event == "stop") return STOPPED;
      return currentState;  // No valid transition
    };

    TS_ASSERT_EQUALS(transition(IDLE, "start"), RUNNING);
    TS_ASSERT_EQUALS(transition(RUNNING, "pause"), PAUSED);
    TS_ASSERT_EQUALS(transition(PAUSED, "resume"), RUNNING);
    TS_ASSERT_EQUALS(transition(RUNNING, "stop"), STOPPED);
    TS_ASSERT_EQUALS(transition(IDLE, "stop"), IDLE);  // Invalid, no change
  }

  // Test multi-state machine
  void testMultiStateMachine() {
    // Engine state: 0=off, 1=starting, 2=running, 3=shutdown
    auto engineTransition = [](int state, bool startCmd, bool stopCmd) -> int {
      if (state == 0 && startCmd) return 1;
      if (state == 1) return 2;  // Auto-transition
      if (state == 2 && stopCmd) return 3;
      if (state == 3) return 0;  // Auto-transition
      return state;
    };

    TS_ASSERT_EQUALS(engineTransition(0, true, false), 1);
    TS_ASSERT_EQUALS(engineTransition(1, false, false), 2);
    TS_ASSERT_EQUALS(engineTransition(2, false, true), 3);
    TS_ASSERT_EQUALS(engineTransition(3, false, false), 0);
  }

  /***************************************************************************
   * Hysteresis Tests
   ***************************************************************************/

  // Test hysteresis in threshold
  void testHysteresis() {
    bool state = false;
    double highThreshold = 10.0;
    double lowThreshold = 5.0;

    auto hysteresisCheck = [&](double value) {
      if (state && value < lowThreshold) state = false;
      if (!state && value > highThreshold) state = true;
      return state;
    };

    // Rising
    TS_ASSERT(!hysteresisCheck(3.0));
    TS_ASSERT(!hysteresisCheck(7.0));  // Between thresholds, stays false
    TS_ASSERT(hysteresisCheck(12.0));  // Above high, turns on

    // Falling
    TS_ASSERT(hysteresisCheck(7.0));   // Between thresholds, stays true
    TS_ASSERT(!hysteresisCheck(3.0));  // Below low, turns off
  }

  // Test deadband
  void testDeadband() {
    auto applyDeadband = [](double input, double deadband) -> double {
      if (std::abs(input) < deadband) return 0.0;
      if (input > 0) return input - deadband;
      return input + deadband;
    };

    TS_ASSERT_DELTA(applyDeadband(0.5, 1.0), 0.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(2.0, 1.0), 1.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(-2.0, 1.0), -1.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(0.0, 1.0), 0.0, epsilon);
  }

  /***************************************************************************
   * Lookup Table Distribution Tests
   ***************************************************************************/

  // Test simple lookup
  void testSimpleLookup() {
    auto lookup = [](int index) -> double {
      double table[] = {0.0, 10.0, 20.0, 30.0, 40.0};
      if (index < 0 || index > 4) return 0.0;
      return table[index];
    };

    TS_ASSERT_DELTA(lookup(0), 0.0, epsilon);
    TS_ASSERT_DELTA(lookup(2), 20.0, epsilon);
    TS_ASSERT_DELTA(lookup(4), 40.0, epsilon);
    TS_ASSERT_DELTA(lookup(-1), 0.0, epsilon);  // Out of bounds
  }

  // Test interpolated lookup
  void testInterpolatedLookup() {
    auto interpolate = [](double x) -> double {
      // Simple linear interpolation between points (0,0), (10,100)
      if (x <= 0) return 0.0;
      if (x >= 10) return 100.0;
      return x * 10.0;
    };

    TS_ASSERT_DELTA(interpolate(0.0), 0.0, epsilon);
    TS_ASSERT_DELTA(interpolate(5.0), 50.0, epsilon);
    TS_ASSERT_DELTA(interpolate(10.0), 100.0, epsilon);
    TS_ASSERT_DELTA(interpolate(7.5), 75.0, epsilon);
  }

  /***************************************************************************
   * Error Handling Tests
   ***************************************************************************/

  // Test with extreme values
  void testExtremeValues() {
    double maxVal = std::numeric_limits<double>::max();
    double minVal = std::numeric_limits<double>::lowest();

    auto classify = [](double x) -> int {
      if (x < 0) return -1;
      if (x > 0) return 1;
      return 0;
    };

    TS_ASSERT_EQUALS(classify(maxVal), 1);
    TS_ASSERT_EQUALS(classify(minVal), -1);
  }

  // Test with very small values
  void testVerySmallValues() {
    double tiny = 1e-300;

    auto isPositive = [](double x) { return x > 0; };

    TS_ASSERT(isPositive(tiny));
    TS_ASSERT(!isPositive(-tiny));
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many evaluations
  void testStressManyEvaluations() {
    std::vector<std::pair<std::function<bool(double)>, double>> cases = {
      {[](double x) { return x < 0; }, -1.0},
      {[](double x) { return x >= 0 && x < 50; }, 0.0},
      {[](double x) { return x >= 50; }, 1.0}
    };

    for (int i = 0; i < 1000; i++) {
      double input = (i % 100) - 25;  // Range: -25 to 74
      double result = exclusiveDistributor(input, cases, 999.0);
      TS_ASSERT(result >= -1.0 && result <= 1.0);
    }
  }

  // Test rapid switching
  void testStressRapidSwitching() {
    bool state = false;

    for (int i = 0; i < 1000; i++) {
      state = (i % 2 == 0);
      TS_ASSERT(state == (i % 2 == 0));
    }
  }

  // Test complex nested conditions
  void testStressNestedConditions() {
    auto deepNested = [](double x, double y, double z) -> int {
      if (x > 0) {
        if (y > 0) {
          if (z > 0) return 7;
          return 6;
        } else {
          if (z > 0) return 5;
          return 4;
        }
      } else {
        if (y > 0) {
          if (z > 0) return 3;
          return 2;
        } else {
          if (z > 0) return 1;
          return 0;
        }
      }
    };

    TS_ASSERT_EQUALS(deepNested(1, 1, 1), 7);
    TS_ASSERT_EQUALS(deepNested(1, 1, -1), 6);
    TS_ASSERT_EQUALS(deepNested(-1, -1, -1), 0);
    TS_ASSERT_EQUALS(deepNested(-1, 1, 1), 3);
  }

  /***************************************************************************
   * Additional Logic Tests
   ***************************************************************************/

  // Test XOR behavior
  void testXORLogic() {
    auto xorFunc = [](bool a, bool b) { return a != b; };

    TS_ASSERT(!xorFunc(true, true));
    TS_ASSERT(xorFunc(true, false));
    TS_ASSERT(xorFunc(false, true));
    TS_ASSERT(!xorFunc(false, false));
  }

  // Test NAND behavior
  void testNANDLogic() {
    auto nandFunc = [](bool a, bool b) { return !(a && b); };

    TS_ASSERT(!nandFunc(true, true));
    TS_ASSERT(nandFunc(true, false));
    TS_ASSERT(nandFunc(false, true));
    TS_ASSERT(nandFunc(false, false));
  }

  // Test NOR behavior
  void testNORLogic() {
    auto norFunc = [](bool a, bool b) { return !(a || b); };

    TS_ASSERT(!norFunc(true, true));
    TS_ASSERT(!norFunc(true, false));
    TS_ASSERT(!norFunc(false, true));
    TS_ASSERT(norFunc(false, false));
  }

  // Test XNOR behavior
  void testXNORLogic() {
    auto xnorFunc = [](bool a, bool b) { return a == b; };

    TS_ASSERT(xnorFunc(true, true));
    TS_ASSERT(!xnorFunc(true, false));
    TS_ASSERT(!xnorFunc(false, true));
    TS_ASSERT(xnorFunc(false, false));
  }

  // Test multi-input AND
  void testMultiInputAND() {
    auto and3 = [](bool a, bool b, bool c) { return a && b && c; };
    auto and4 = [](bool a, bool b, bool c, bool d) { return a && b && c && d; };

    TS_ASSERT(and3(true, true, true));
    TS_ASSERT(!and3(true, true, false));
    TS_ASSERT(and4(true, true, true, true));
    TS_ASSERT(!and4(true, true, true, false));
  }

  // Test multi-input OR
  void testMultiInputOR() {
    auto or3 = [](bool a, bool b, bool c) { return a || b || c; };
    auto or4 = [](bool a, bool b, bool c, bool d) { return a || b || c || d; };

    TS_ASSERT(!or3(false, false, false));
    TS_ASSERT(or3(false, false, true));
    TS_ASSERT(!or4(false, false, false, false));
    TS_ASSERT(or4(false, false, false, true));
  }

  // Test majority voting
  void testMajorityVoting() {
    auto majorityOf3 = [](bool a, bool b, bool c) {
      int count = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
      return count >= 2;
    };

    TS_ASSERT(!majorityOf3(false, false, false));
    TS_ASSERT(!majorityOf3(true, false, false));
    TS_ASSERT(majorityOf3(true, true, false));
    TS_ASSERT(majorityOf3(true, true, true));
  }
};
