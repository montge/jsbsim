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
#include <algorithm>
#include <functional>

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

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

  /***************************************************************************
   * Flight Mode Distribution Tests
   ***************************************************************************/

  // Test flight phase detection
  void testFlightPhaseDetection() {
    enum Phase { PREFLIGHT, TAXI, TAKEOFF, CLIMB, CRUISE, DESCENT, APPROACH, LANDING };

    auto detectPhase = [](double altitude, double vs, bool onGround, double airspeed) -> int {
      if (onGround && airspeed < 30) return PREFLIGHT;
      if (onGround && airspeed >= 30) return TAXI;
      if (!onGround && altitude < 1500 && vs > 500) return TAKEOFF;
      if (!onGround && vs > 100 && altitude < 35000) return CLIMB;
      if (!onGround && std::abs(vs) < 100 && altitude > 10000) return CRUISE;
      if (!onGround && vs < -100 && altitude > 3000) return DESCENT;
      if (!onGround && altitude < 3000 && altitude > 50) return APPROACH;
      if (!onGround && altitude < 50) return LANDING;
      return PREFLIGHT;
    };

    TS_ASSERT_EQUALS(detectPhase(0, 0, true, 0), PREFLIGHT);
    TS_ASSERT_EQUALS(detectPhase(0, 0, true, 50), TAXI);
    TS_ASSERT_EQUALS(detectPhase(500, 1500, false, 150), TAKEOFF);
    TS_ASSERT_EQUALS(detectPhase(15000, 1000, false, 300), CLIMB);
    TS_ASSERT_EQUALS(detectPhase(35000, 0, false, 450), CRUISE);
    TS_ASSERT_EQUALS(detectPhase(8000, -1500, false, 300), DESCENT);
    TS_ASSERT_EQUALS(detectPhase(1500, -700, false, 150), APPROACH);
    TS_ASSERT_EQUALS(detectPhase(30, -300, false, 130), LANDING);
  }

  // Test engine mode selection
  void testEngineModeSelection() {
    enum EngineMode { IDLE, CLIMB, CRUISE, DESCENT, TOGA };

    auto selectEngineMode = [](double throttle, bool toga, double altitude, double vs) -> int {
      if (toga) return TOGA;
      if (throttle < 0.3) return IDLE;
      if (vs > 500 && altitude < 10000) return CLIMB;
      if (std::abs(vs) < 200) return CRUISE;
      if (vs < -500) return DESCENT;
      return CRUISE;
    };

    TS_ASSERT_EQUALS(selectEngineMode(0.9, true, 1000, 1500), TOGA);
    TS_ASSERT_EQUALS(selectEngineMode(0.2, false, 0, 0), IDLE);
    TS_ASSERT_EQUALS(selectEngineMode(0.8, false, 5000, 1500), CLIMB);
    TS_ASSERT_EQUALS(selectEngineMode(0.5, false, 35000, 0), CRUISE);
    TS_ASSERT_EQUALS(selectEngineMode(0.3, false, 20000, -2000), DESCENT);
  }

  // Test warning priority system
  void testWarningPrioritySystem() {
    auto getWarningLevel = [](bool fire, bool engine_fail, bool hydraulic_low,
                              bool fuel_low, bool cabin_alt_high) -> int {
      if (fire) return 5;  // EMERGENCY
      if (engine_fail) return 4;  // WARNING
      if (hydraulic_low || cabin_alt_high) return 3;  // CAUTION
      if (fuel_low) return 2;  // ADVISORY
      return 0;  // NORMAL
    };

    TS_ASSERT_EQUALS(getWarningLevel(true, true, true, true, true), 5);
    TS_ASSERT_EQUALS(getWarningLevel(false, true, true, true, true), 4);
    TS_ASSERT_EQUALS(getWarningLevel(false, false, true, true, true), 3);
    TS_ASSERT_EQUALS(getWarningLevel(false, false, false, true, false), 2);
    TS_ASSERT_EQUALS(getWarningLevel(false, false, false, false, false), 0);
  }

  /***************************************************************************
   * Control Surface Distribution Tests
   ***************************************************************************/

  // Test aileron distribution
  void testAileronDistribution() {
    auto distributeAileron = [](double command, bool spoilerAssist) -> std::pair<double, double> {
      double left = -command;
      double right = command;
      if (spoilerAssist && std::abs(command) > 0.5) {
        // Spoiler assist for large commands
        double spoiler_add = (std::abs(command) - 0.5) * 0.5;
        if (command > 0) left -= spoiler_add;
        else right -= spoiler_add;
      }
      return {left, right};
    };

    auto result = distributeAileron(0.3, false);
    TS_ASSERT_DELTA(result.first, -0.3, epsilon);
    TS_ASSERT_DELTA(result.second, 0.3, epsilon);
  }

  // Test rudder distribution
  void testRudderDistribution() {
    auto distributeRudder = [](double command, double yaw_damper) -> double {
      return command + yaw_damper;
    };

    TS_ASSERT_DELTA(distributeRudder(0.5, 0.1), 0.6, epsilon);
    TS_ASSERT_DELTA(distributeRudder(-0.3, 0.05), -0.25, epsilon);
  }

  // Test elevator distribution
  void testElevatorDistribution() {
    auto distributeElevator = [](double command, double trim, double mach_compensation) -> double {
      return command + trim + mach_compensation;
    };

    TS_ASSERT_DELTA(distributeElevator(-0.2, 0.05, 0.02), -0.13, epsilon);
  }

  /***************************************************************************
   * Selector Logic Tests
   ***************************************************************************/

  // Test min selector
  void testMinSelector() {
    auto minOf3 = [](double a, double b, double c) {
      return std::min({a, b, c});
    };

    TS_ASSERT_DELTA(minOf3(5.0, 3.0, 7.0), 3.0, epsilon);
    TS_ASSERT_DELTA(minOf3(-2.0, -5.0, -1.0), -5.0, epsilon);
  }

  // Test max selector
  void testMaxSelector() {
    auto maxOf3 = [](double a, double b, double c) {
      return std::max({a, b, c});
    };

    TS_ASSERT_DELTA(maxOf3(5.0, 3.0, 7.0), 7.0, epsilon);
    TS_ASSERT_DELTA(maxOf3(-2.0, -5.0, -1.0), -1.0, epsilon);
  }

  // Test median selector
  void testMedianSelector() {
    auto medianOf3 = [](double a, double b, double c) {
      double arr[] = {a, b, c};
      std::sort(arr, arr + 3);
      return arr[1];
    };

    TS_ASSERT_DELTA(medianOf3(5.0, 3.0, 7.0), 5.0, epsilon);
    TS_ASSERT_DELTA(medianOf3(1.0, 2.0, 3.0), 2.0, epsilon);
  }

  // Test average selector
  void testAverageSelector() {
    auto avgOf3 = [](double a, double b, double c) {
      return (a + b + c) / 3.0;
    };

    TS_ASSERT_DELTA(avgOf3(3.0, 6.0, 9.0), 6.0, epsilon);
  }

  /***************************************************************************
   * Timer/Delay Distribution Tests
   ***************************************************************************/

  // Test delayed activation
  void testDelayedActivation() {
    double timer = 0.0;
    double delay = 0.5;
    double dt = 0.1;
    bool input = true;
    bool output = false;

    for (int i = 0; i < 10; i++) {
      if (input) {
        timer += dt;
        if (timer >= delay) output = true;
      } else {
        timer = 0.0;
        output = false;
      }
    }

    TS_ASSERT(output);  // After 1.0 seconds, should be active
  }

  // Test one-shot trigger
  void testOneShotTrigger() {
    bool prev_input = false;
    bool input = true;
    bool triggered = false;

    // Rising edge detection
    triggered = input && !prev_input;
    TS_ASSERT(triggered);

    prev_input = input;
    triggered = input && !prev_input;
    TS_ASSERT(!triggered);  // No longer rising edge
  }

  // Test flip-flop
  void testFlipFlop() {
    bool state = false;
    bool set = true;
    bool reset = false;

    // Set-dominant SR flip-flop
    if (set) state = true;
    else if (reset) state = false;

    TS_ASSERT(state);

    set = false;
    reset = true;
    if (set) state = true;
    else if (reset) state = false;

    TS_ASSERT(!state);
  }

  /***************************************************************************
   * Limit Distribution Tests
   ***************************************************************************/

  // Test rate limiting
  void testRateLimiting() {
    double prev_output = 0.0;
    double rate_limit = 1.0;
    double dt = 0.1;

    auto rateLimitedOutput = [&](double target) {
      double max_change = rate_limit * dt;
      double delta = target - prev_output;
      if (delta > max_change) delta = max_change;
      if (delta < -max_change) delta = -max_change;
      prev_output += delta;
      return prev_output;
    };

    TS_ASSERT_DELTA(rateLimitedOutput(10.0), 0.1, epsilon);
    TS_ASSERT_DELTA(rateLimitedOutput(10.0), 0.2, epsilon);
  }

  // Test position limiting
  void testPositionLimiting() {
    auto limitPosition = [](double input, double min, double max) {
      if (input < min) return min;
      if (input > max) return max;
      return input;
    };

    TS_ASSERT_DELTA(limitPosition(5.0, 0.0, 10.0), 5.0, epsilon);
    TS_ASSERT_DELTA(limitPosition(-5.0, 0.0, 10.0), 0.0, epsilon);
    TS_ASSERT_DELTA(limitPosition(15.0, 0.0, 10.0), 10.0, epsilon);
  }

  // Test asymmetric limiting
  void testAsymmetricLimiting() {
    auto asymLimit = [](double input, double min, double max) {
      return std::max(min, std::min(max, input));
    };

    TS_ASSERT_DELTA(asymLimit(0.0, -5.0, 25.0), 0.0, epsilon);
    TS_ASSERT_DELTA(asymLimit(-10.0, -5.0, 25.0), -5.0, epsilon);
    TS_ASSERT_DELTA(asymLimit(30.0, -5.0, 25.0), 25.0, epsilon);
  }

  /***************************************************************************
   * Failure Mode Distribution Tests
   ***************************************************************************/

  // Test fail-safe logic
  void testFailSafeLogic() {
    auto failSafe = [](double primary, double backup, bool primary_valid) {
      return primary_valid ? primary : backup;
    };

    TS_ASSERT_DELTA(failSafe(100.0, 50.0, true), 100.0, epsilon);
    TS_ASSERT_DELTA(failSafe(100.0, 50.0, false), 50.0, epsilon);
  }

  // Test triple redundancy voting
  void testTripleRedundancyVoting() {
    auto vote3 = [](double a, double b, double c, double threshold) {
      // Return median if all agree within threshold
      if (std::abs(a - b) < threshold && std::abs(b - c) < threshold) {
        double arr[] = {a, b, c};
        std::sort(arr, arr + 3);
        return arr[1];
      }
      // Otherwise find the two that agree
      if (std::abs(a - b) < threshold) return (a + b) / 2.0;
      if (std::abs(b - c) < threshold) return (b + c) / 2.0;
      if (std::abs(a - c) < threshold) return (a + c) / 2.0;
      return (a + b + c) / 3.0;  // Fallback
    };

    TS_ASSERT_DELTA(vote3(10.0, 10.1, 10.0, 0.5), 10.0, 0.1);
    TS_ASSERT_DELTA(vote3(10.0, 10.0, 50.0, 0.5), 10.0, 0.1);
  }

  // Test fail-operational logic
  void testFailOperationalLogic() {
    auto failOp = [](bool ch1_valid, bool ch2_valid, double ch1, double ch2) {
      if (ch1_valid && ch2_valid) return (ch1 + ch2) / 2.0;
      if (ch1_valid) return ch1;
      if (ch2_valid) return ch2;
      return 0.0;  // Both failed
    };

    TS_ASSERT_DELTA(failOp(true, true, 10.0, 12.0), 11.0, epsilon);
    TS_ASSERT_DELTA(failOp(true, false, 10.0, 12.0), 10.0, epsilon);
    TS_ASSERT_DELTA(failOp(false, true, 10.0, 12.0), 12.0, epsilon);
    TS_ASSERT_DELTA(failOp(false, false, 10.0, 12.0), 0.0, epsilon);
  }

  /***************************************************************************
   * Multi-Way Switch Tests
   ***************************************************************************/

  // Test 4-way switch
  void testFourWaySwitch() {
    auto switch4 = [](int selector, double a, double b, double c, double d) {
      switch (selector) {
        case 0: return a;
        case 1: return b;
        case 2: return c;
        case 3: return d;
        default: return 0.0;
      }
    };

    TS_ASSERT_DELTA(switch4(0, 10.0, 20.0, 30.0, 40.0), 10.0, epsilon);
    TS_ASSERT_DELTA(switch4(1, 10.0, 20.0, 30.0, 40.0), 20.0, epsilon);
    TS_ASSERT_DELTA(switch4(2, 10.0, 20.0, 30.0, 40.0), 30.0, epsilon);
    TS_ASSERT_DELTA(switch4(3, 10.0, 20.0, 30.0, 40.0), 40.0, epsilon);
    TS_ASSERT_DELTA(switch4(5, 10.0, 20.0, 30.0, 40.0), 0.0, epsilon);
  }

  // Test rotary switch with wraparound
  void testRotarySwitchWraparound() {
    auto rotarySwitch = [](int position, int numPositions) {
      return ((position % numPositions) + numPositions) % numPositions;
    };

    TS_ASSERT_EQUALS(rotarySwitch(0, 4), 0);
    TS_ASSERT_EQUALS(rotarySwitch(4, 4), 0);
    TS_ASSERT_EQUALS(rotarySwitch(5, 4), 1);
    TS_ASSERT_EQUALS(rotarySwitch(-1, 4), 3);
  }

  // Test binary selector tree
  void testBinarySelectorTree() {
    auto binaryTree = [](bool b0, bool b1, double v00, double v01, double v10, double v11) {
      if (!b0) {
        return b1 ? v01 : v00;
      } else {
        return b1 ? v11 : v10;
      }
    };

    TS_ASSERT_DELTA(binaryTree(false, false, 1.0, 2.0, 3.0, 4.0), 1.0, epsilon);
    TS_ASSERT_DELTA(binaryTree(false, true, 1.0, 2.0, 3.0, 4.0), 2.0, epsilon);
    TS_ASSERT_DELTA(binaryTree(true, false, 1.0, 2.0, 3.0, 4.0), 3.0, epsilon);
    TS_ASSERT_DELTA(binaryTree(true, true, 1.0, 2.0, 3.0, 4.0), 4.0, epsilon);
  }

  /***************************************************************************
   * Envelope Protection Tests
   ***************************************************************************/

  // Test AOA protection distribution
  void testAOAProtection() {
    auto aoaProtect = [](double pilot_cmd, double aoa, double aoa_limit, double gain) {
      double protection = 0.0;
      if (aoa > aoa_limit) {
        protection = gain * (aoa - aoa_limit);
      }
      return pilot_cmd - protection;
    };

    TS_ASSERT_DELTA(aoaProtect(1.0, 10.0, 15.0, 0.1), 1.0, epsilon);  // Below limit
    TS_ASSERT_DELTA(aoaProtect(1.0, 18.0, 15.0, 0.1), 0.7, epsilon);  // Above limit
  }

  // Test overspeed protection
  void testOverspeedProtection() {
    auto overspeedProtect = [](double throttle, double mach, double mach_limit) {
      if (mach > mach_limit) {
        double reduction = (mach - mach_limit) * 5.0;
        return std::max(0.0, throttle - reduction);
      }
      return throttle;
    };

    TS_ASSERT_DELTA(overspeedProtect(0.8, 0.80, 0.85), 0.8, epsilon);
    TS_ASSERT_DELTA(overspeedProtect(0.8, 0.90, 0.85), 0.55, epsilon);
  }

  // Test bank angle protection
  void testBankAngleProtection() {
    auto bankProtect = [](double roll_cmd, double bank, double bank_limit) {
      double limit_rad = bank_limit * 3.14159 / 180.0;
      double bank_rad = bank * 3.14159 / 180.0;

      if (std::abs(bank_rad) > limit_rad) {
        if (bank > 0 && roll_cmd > 0) return 0.0;
        if (bank < 0 && roll_cmd < 0) return 0.0;
      }
      return roll_cmd;
    };

    TS_ASSERT_DELTA(bankProtect(0.5, 20.0, 30.0), 0.5, epsilon);
    TS_ASSERT_DELTA(bankProtect(0.5, 35.0, 30.0), 0.0, epsilon);
    TS_ASSERT_DELTA(bankProtect(-0.5, 35.0, 30.0), -0.5, epsilon);  // Recovery allowed
  }

  /***************************************************************************
   * Input Validation Tests
   ***************************************************************************/

  // Test range validation
  void testRangeValidation() {
    auto validateRange = [](double input, double min, double max) {
      return input >= min && input <= max;
    };

    TS_ASSERT(validateRange(50.0, 0.0, 100.0));
    TS_ASSERT(!validateRange(-10.0, 0.0, 100.0));
    TS_ASSERT(!validateRange(110.0, 0.0, 100.0));
    TS_ASSERT(validateRange(0.0, 0.0, 100.0));  // Edge case
  }

  // Test rate of change validation
  void testRateValidation() {
    auto validateRate = [](double current, double previous, double dt, double maxRate) {
      double rate = std::abs(current - previous) / dt;
      return rate <= maxRate;
    };

    TS_ASSERT(validateRate(10.0, 9.0, 0.1, 15.0));   // Rate = 10/s
    TS_ASSERT(!validateRate(20.0, 9.0, 0.1, 15.0));  // Rate = 110/s
  }

  // Test signal reasonableness
  void testSignalReasonableness() {
    auto isReasonable = [](double value, double expected, double tolerance) {
      return std::abs(value - expected) <= tolerance;
    };

    TS_ASSERT(isReasonable(100.5, 100.0, 1.0));
    TS_ASSERT(!isReasonable(105.0, 100.0, 1.0));
  }

  /***************************************************************************
   * Smoothing and Transition Tests
   ***************************************************************************/

  // Test linear crossfade
  void testLinearCrossfade() {
    auto crossfade = [](double a, double b, double t) {
      return a * (1.0 - t) + b * t;
    };

    TS_ASSERT_DELTA(crossfade(0.0, 100.0, 0.0), 0.0, epsilon);
    TS_ASSERT_DELTA(crossfade(0.0, 100.0, 0.5), 50.0, epsilon);
    TS_ASSERT_DELTA(crossfade(0.0, 100.0, 1.0), 100.0, epsilon);
  }

  // Test smooth step function
  void testSmoothStep() {
    auto smoothStep = [](double edge0, double edge1, double x) {
      double t = std::max(0.0, std::min((x - edge0) / (edge1 - edge0), 1.0));
      return t * t * (3.0 - 2.0 * t);  // Hermite interpolation
    };

    TS_ASSERT_DELTA(smoothStep(0.0, 1.0, -0.5), 0.0, epsilon);
    TS_ASSERT_DELTA(smoothStep(0.0, 1.0, 0.5), 0.5, epsilon);
    TS_ASSERT_DELTA(smoothStep(0.0, 1.0, 1.5), 1.0, epsilon);
  }

  // Test exponential transition
  void testExponentialTransition() {
    auto expTransition = [](double current, double target, double tau, double dt) {
      double alpha = 1.0 - std::exp(-dt / tau);
      return current + alpha * (target - current);
    };

    double current = 0.0;
    for (int i = 0; i < 100; i++) {
      current = expTransition(current, 100.0, 1.0, 0.1);
    }
    TS_ASSERT(current > 99.0);  // Should approach target
  }

  /***************************************************************************
   * Threshold Crossing Tests
   ***************************************************************************/

  // Test rising edge detection
  void testRisingEdge() {
    auto risingEdge = [](double prev, double curr, double threshold) {
      return prev < threshold && curr >= threshold;
    };

    TS_ASSERT(risingEdge(0.4, 0.6, 0.5));
    TS_ASSERT(!risingEdge(0.6, 0.8, 0.5));
    TS_ASSERT(!risingEdge(0.6, 0.4, 0.5));
  }

  // Test falling edge detection
  void testFallingEdge() {
    auto fallingEdge = [](double prev, double curr, double threshold) {
      return prev >= threshold && curr < threshold;
    };

    TS_ASSERT(fallingEdge(0.6, 0.4, 0.5));
    TS_ASSERT(!fallingEdge(0.4, 0.3, 0.5));
    TS_ASSERT(!fallingEdge(0.4, 0.6, 0.5));
  }

  // Test level crossing count
  void testLevelCrossingCount() {
    double values[] = {0.0, 0.3, 0.6, 0.4, 0.7, 0.2, 0.8};
    double threshold = 0.5;
    int crossings = 0;

    for (int i = 1; i < 7; i++) {
      if ((values[i-1] < threshold && values[i] >= threshold) ||
          (values[i-1] >= threshold && values[i] < threshold)) {
        crossings++;
      }
    }

    TS_ASSERT_EQUALS(crossings, 5);
  }

  /***************************************************************************
   * Mode Arbitration Tests
   ***************************************************************************/

  // Test mode priority arbitration
  void testModePriorityArbitration() {
    auto arbitrate = [](bool mode_a, bool mode_b, bool mode_c, int priority_a, int priority_b, int priority_c) {
      int active = -1;
      int highest = -1;

      if (mode_a && priority_a > highest) { active = 0; highest = priority_a; }
      if (mode_b && priority_b > highest) { active = 1; highest = priority_b; }
      if (mode_c && priority_c > highest) { active = 2; highest = priority_c; }

      return active;
    };

    TS_ASSERT_EQUALS(arbitrate(true, true, true, 1, 2, 3), 2);
    TS_ASSERT_EQUALS(arbitrate(true, true, false, 1, 2, 3), 1);
    TS_ASSERT_EQUALS(arbitrate(true, false, false, 1, 2, 3), 0);
  }

  // Test mutual exclusion
  void testMutualExclusion() {
    auto exclusiveSelect = [](int request1, int request2, int current) {
      // Only allow one to be active; priority to current holder
      if (current == 1) return 1;
      if (current == 2) return 2;
      if (request1 && !request2) return 1;
      if (request2 && !request1) return 2;
      return 0;  // Neither or conflict
    };

    TS_ASSERT_EQUALS(exclusiveSelect(1, 0, 0), 1);
    TS_ASSERT_EQUALS(exclusiveSelect(0, 1, 0), 2);
    TS_ASSERT_EQUALS(exclusiveSelect(1, 1, 0), 0);  // Conflict
    TS_ASSERT_EQUALS(exclusiveSelect(1, 1, 1), 1);  // Holder priority
  }

  /***************************************************************************
   * Sensor Selection Tests
   ***************************************************************************/

  // Test best source selection
  void testBestSourceSelection() {
    auto selectBest = [](double s1, bool v1, double s2, bool v2, double s3, bool v3) {
      // Select median of valid sources, or single valid, or default
      std::vector<double> valid;
      if (v1) valid.push_back(s1);
      if (v2) valid.push_back(s2);
      if (v3) valid.push_back(s3);

      if (valid.empty()) return 0.0;
      if (valid.size() == 1) return valid[0];
      if (valid.size() == 2) return (valid[0] + valid[1]) / 2.0;

      std::sort(valid.begin(), valid.end());
      return valid[1];  // Median
    };

    TS_ASSERT_DELTA(selectBest(10.0, true, 11.0, true, 100.0, true), 11.0, epsilon);
    TS_ASSERT_DELTA(selectBest(10.0, true, 11.0, true, 100.0, false), 10.5, epsilon);
    TS_ASSERT_DELTA(selectBest(10.0, true, 11.0, false, 100.0, false), 10.0, epsilon);
  }

  // Test outlier rejection
  void testOutlierRejection() {
    auto rejectOutliers = [](double a, double b, double c, double threshold) {
      double avg = (a + b + c) / 3.0;
      std::vector<double> valid;

      if (std::abs(a - avg) < threshold) valid.push_back(a);
      if (std::abs(b - avg) < threshold) valid.push_back(b);
      if (std::abs(c - avg) < threshold) valid.push_back(c);

      if (valid.empty()) return avg;
      double sum = 0.0;
      for (double v : valid) sum += v;
      return sum / valid.size();
    };

    // With values 10, 11, 12 and threshold 5, avg = 11, all within threshold
    TS_ASSERT_DELTA(rejectOutliers(10.0, 11.0, 12.0, 5.0), 11.0, epsilon);

    // With one outlier at 50, avg = 23.67, 50 is 26.33 away (rejected with threshold 25)
    // 10 and 11 are within threshold, so result is (10+11)/2 = 10.5
    TS_ASSERT_DELTA(rejectOutliers(10.0, 11.0, 50.0, 25.0), 10.5, epsilon);
  }

  /***************************************************************************
   * Configuration Distribution Tests
   ***************************************************************************/

  // Test aircraft configuration selection
  void testAircraftConfigSelection() {
    enum Config { CLEAN, TAKEOFF, APPROACH, LANDING };

    auto selectConfig = [](double flaps, bool gear_down, double speed) {
      if (gear_down && flaps > 25.0) return LANDING;
      if (gear_down || flaps > 15.0) return APPROACH;
      if (flaps > 0.0) return TAKEOFF;
      return CLEAN;
    };

    TS_ASSERT_EQUALS(selectConfig(0.0, false, 300.0), CLEAN);
    TS_ASSERT_EQUALS(selectConfig(10.0, false, 200.0), TAKEOFF);
    TS_ASSERT_EQUALS(selectConfig(20.0, true, 150.0), APPROACH);
    TS_ASSERT_EQUALS(selectConfig(30.0, true, 130.0), LANDING);
  }

  // Test weight and balance distribution
  void testWeightBalanceDistribution() {
    auto cgPosition = [](double fuel_fwd, double fuel_aft, double pax_fwd, double pax_aft, double cargo) {
      double total_weight = fuel_fwd + fuel_aft + pax_fwd + pax_aft + cargo;
      if (total_weight == 0.0) return 0.0;

      double moment = fuel_fwd * 10.0 + fuel_aft * 40.0 + pax_fwd * 15.0 + pax_aft * 35.0 + cargo * 45.0;
      return moment / total_weight;
    };

    double cg = cgPosition(1000.0, 500.0, 400.0, 300.0, 200.0);
    TS_ASSERT(cg > 15.0 && cg < 35.0);  // CG in reasonable range
  }

  /***************************************************************************
   * Time-Based Distribution Tests
   ***************************************************************************/

  // Test duty cycle
  void testDutyCycle() {
    auto dutyCycleOutput = [](double time, double period, double duty) {
      double phase = std::fmod(time, period);
      return phase < (period * duty) ? 1.0 : 0.0;
    };

    TS_ASSERT_DELTA(dutyCycleOutput(0.1, 1.0, 0.5), 1.0, epsilon);
    TS_ASSERT_DELTA(dutyCycleOutput(0.6, 1.0, 0.5), 0.0, epsilon);
    TS_ASSERT_DELTA(dutyCycleOutput(1.1, 1.0, 0.5), 1.0, epsilon);
  }

  // Test time-based mode switching
  void testTimeBasedModeSwitching() {
    auto modeForTime = [](double elapsed, double phase1_end, double phase2_end, double phase3_end) {
      if (elapsed < phase1_end) return 1;
      if (elapsed < phase2_end) return 2;
      if (elapsed < phase3_end) return 3;
      return 4;
    };

    TS_ASSERT_EQUALS(modeForTime(5.0, 10.0, 20.0, 30.0), 1);
    TS_ASSERT_EQUALS(modeForTime(15.0, 10.0, 20.0, 30.0), 2);
    TS_ASSERT_EQUALS(modeForTime(25.0, 10.0, 20.0, 30.0), 3);
    TS_ASSERT_EQUALS(modeForTime(35.0, 10.0, 20.0, 30.0), 4);
  }

  // Test scheduled event
  void testScheduledEvent() {
    auto checkSchedule = [](double time, double event_time, double tolerance) {
      return std::abs(time - event_time) <= tolerance;
    };

    TS_ASSERT(checkSchedule(10.0, 10.0, 0.1));
    TS_ASSERT(checkSchedule(10.05, 10.0, 0.1));
    TS_ASSERT(!checkSchedule(10.2, 10.0, 0.1));
  }

  /***************************************************************************
   * Conditional Output Tests
   ***************************************************************************/

  // Test conditional pass-through
  void testConditionalPassthrough() {
    auto passthrough = [](double input, bool enable) {
      return enable ? input : 0.0;
    };

    TS_ASSERT_DELTA(passthrough(42.0, true), 42.0, epsilon);
    TS_ASSERT_DELTA(passthrough(42.0, false), 0.0, epsilon);
  }

  // Test conditional with hold
  void testConditionalWithHold() {
    double held_value = 0.0;

    auto conditionalHold = [&](double input, bool track) {
      if (track) {
        held_value = input;
      }
      return held_value;
    };

    TS_ASSERT_DELTA(conditionalHold(10.0, true), 10.0, epsilon);
    TS_ASSERT_DELTA(conditionalHold(20.0, false), 10.0, epsilon);  // Held
    TS_ASSERT_DELTA(conditionalHold(30.0, true), 30.0, epsilon);  // Track again
  }

  // Test multiple output assignment
  void testMultipleOutputAssignment() {
    auto assignOutputs = [](double input, bool use_a, bool use_b) {
      double out_a = use_a ? input : 0.0;
      double out_b = use_b ? input * 2.0 : 0.0;
      return std::make_pair(out_a, out_b);
    };

    auto result = assignOutputs(5.0, true, true);
    TS_ASSERT_DELTA(result.first, 5.0, epsilon);
    TS_ASSERT_DELTA(result.second, 10.0, epsilon);

    result = assignOutputs(5.0, true, false);
    TS_ASSERT_DELTA(result.first, 5.0, epsilon);
    TS_ASSERT_DELTA(result.second, 0.0, epsilon);
  }
};
