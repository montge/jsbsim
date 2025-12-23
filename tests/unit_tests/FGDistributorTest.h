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
};
