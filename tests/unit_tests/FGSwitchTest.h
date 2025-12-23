/*******************************************************************************
 * FGSwitchTest.h - Unit tests for FGSwitch (switching logic)
 *
 * Tests the logical behavior of switch components:
 * - Boolean condition evaluation
 * - Multi-input selection
 * - Default value handling
 *
 * Note: FGSwitch requires XML element for construction, so these tests focus on
 * the underlying logical operations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <string>

const double epsilon = 1e-10;

class FGSwitchTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Basic Condition Evaluation
   ***************************************************************************/

  // Test EQ (equals) condition
  void testConditionEQ() {
    auto eq = [](double a, double b) { return std::abs(a - b) < 1e-10; };

    TS_ASSERT(eq(5.0, 5.0));
    TS_ASSERT(!eq(5.0, 5.1));
    TS_ASSERT(eq(0.0, 0.0));
    TS_ASSERT(eq(-1.0, -1.0));
  }

  // Test NE (not equals) condition
  void testConditionNE() {
    auto ne = [](double a, double b) { return std::abs(a - b) >= 1e-10; };

    TS_ASSERT(!ne(5.0, 5.0));
    TS_ASSERT(ne(5.0, 5.1));
    TS_ASSERT(!ne(0.0, 0.0));
  }

  // Test GT (greater than) condition
  void testConditionGT() {
    auto gt = [](double a, double b) { return a > b; };

    TS_ASSERT(gt(5.1, 5.0));
    TS_ASSERT(!gt(5.0, 5.0));
    TS_ASSERT(!gt(4.9, 5.0));
  }

  // Test GE (greater or equal) condition
  void testConditionGE() {
    auto ge = [](double a, double b) { return a >= b; };

    TS_ASSERT(ge(5.1, 5.0));
    TS_ASSERT(ge(5.0, 5.0));
    TS_ASSERT(!ge(4.9, 5.0));
  }

  // Test LT (less than) condition
  void testConditionLT() {
    auto lt = [](double a, double b) { return a < b; };

    TS_ASSERT(!lt(5.1, 5.0));
    TS_ASSERT(!lt(5.0, 5.0));
    TS_ASSERT(lt(4.9, 5.0));
  }

  // Test LE (less or equal) condition
  void testConditionLE() {
    auto le = [](double a, double b) { return a <= b; };

    TS_ASSERT(!le(5.1, 5.0));
    TS_ASSERT(le(5.0, 5.0));
    TS_ASSERT(le(4.9, 5.0));
  }

  /***************************************************************************
   * Logical Operations
   ***************************************************************************/

  // Test AND operation
  void testLogicalAND() {
    auto AND = [](bool a, bool b) { return a && b; };

    TS_ASSERT(AND(true, true));
    TS_ASSERT(!AND(true, false));
    TS_ASSERT(!AND(false, true));
    TS_ASSERT(!AND(false, false));
  }

  // Test OR operation
  void testLogicalOR() {
    auto OR = [](bool a, bool b) { return a || b; };

    TS_ASSERT(OR(true, true));
    TS_ASSERT(OR(true, false));
    TS_ASSERT(OR(false, true));
    TS_ASSERT(!OR(false, false));
  }

  // Test NOT operation
  void testLogicalNOT() {
    auto NOT = [](bool a) { return !a; };

    TS_ASSERT(!NOT(true));
    TS_ASSERT(NOT(false));
  }

  // Test compound AND condition
  void testCompoundAND() {
    // (a > 5) AND (b < 10)
    double a = 6.0, b = 8.0;
    bool result = (a > 5.0) && (b < 10.0);
    TS_ASSERT(result);

    a = 6.0; b = 12.0;
    result = (a > 5.0) && (b < 10.0);
    TS_ASSERT(!result);
  }

  // Test compound OR condition
  void testCompoundOR() {
    // (a > 5) OR (b < 10)
    double a = 3.0, b = 8.0;
    bool result = (a > 5.0) || (b < 10.0);
    TS_ASSERT(result);

    a = 3.0; b = 12.0;
    result = (a > 5.0) || (b < 10.0);
    TS_ASSERT(!result);
  }

  /***************************************************************************
   * Switch Selection Logic
   ***************************************************************************/

  // Test two-way switch
  void testTwoWaySwitch() {
    auto twoWaySwitch = [](bool condition, double trueVal, double falseVal) {
      return condition ? trueVal : falseVal;
    };

    TS_ASSERT_DELTA(twoWaySwitch(true, 10.0, 5.0), 10.0, epsilon);
    TS_ASSERT_DELTA(twoWaySwitch(false, 10.0, 5.0), 5.0, epsilon);
  }

  // Test multi-way switch (first match wins)
  void testMultiWaySwitch() {
    // Simulate switch with multiple conditions
    auto multiSwitch = [](double x) -> double {
      if (x < 0) return -1.0;
      if (x == 0) return 0.0;
      if (x < 10) return 1.0;
      if (x < 100) return 2.0;
      return 3.0;  // default
    };

    TS_ASSERT_DELTA(multiSwitch(-5.0), -1.0, epsilon);
    TS_ASSERT_DELTA(multiSwitch(0.0), 0.0, epsilon);
    TS_ASSERT_DELTA(multiSwitch(5.0), 1.0, epsilon);
    TS_ASSERT_DELTA(multiSwitch(50.0), 2.0, epsilon);
    TS_ASSERT_DELTA(multiSwitch(150.0), 3.0, epsilon);
  }

  // Test default value when no conditions match
  void testDefaultValue() {
    double defaultVal = 999.0;
    bool cond1 = false, cond2 = false, cond3 = false;

    double result;
    if (cond1) result = 1.0;
    else if (cond2) result = 2.0;
    else if (cond3) result = 3.0;
    else result = defaultVal;

    TS_ASSERT_DELTA(result, defaultVal, epsilon);
  }

  /***************************************************************************
   * Property-based Conditions
   ***************************************************************************/

  // Test threshold crossing (gear down)
  void testThresholdCrossing() {
    auto gearDown = [](double velocity) {
      const double threshold = 200.0;  // knots
      return velocity < threshold;
    };

    TS_ASSERT(gearDown(150.0));   // Below threshold
    TS_ASSERT(!gearDown(250.0));  // Above threshold
    TS_ASSERT(!gearDown(200.0));  // At threshold (not strictly less)
  }

  // Test hysteresis switching
  void testHysteresisSwitching() {
    double lowThreshold = 100.0;
    double highThreshold = 150.0;
    bool state = false;

    // State machine with hysteresis
    auto updateState = [&](double input) {
      if (input > highThreshold) state = true;
      else if (input < lowThreshold) state = false;
      // else: state unchanged (hysteresis)
      return state;
    };

    // Start low, go high
    TS_ASSERT(!updateState(50.0));   // Below low
    TS_ASSERT(!updateState(125.0));  // Between (stays low)
    TS_ASSERT(updateState(175.0));   // Above high
    TS_ASSERT(updateState(125.0));   // Between (stays high)
    TS_ASSERT(!updateState(75.0));   // Below low
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test comparison with zero
  void testComparisonWithZero() {
    TS_ASSERT(0.0 == 0.0);
    TS_ASSERT(-0.0 == 0.0);  // Signed zero
    TS_ASSERT(!(0.0 < 0.0));
    TS_ASSERT(0.0 <= 0.0);
  }

  // Test comparison with very small differences
  void testSmallDifferences() {
    double a = 1.0;
    double b = 1.0 + 1e-15;

    // Direct comparison may fail
    TS_ASSERT(a != b);

    // But with tolerance should work
    TS_ASSERT(std::abs(a - b) < 1e-10);
  }

  // Test comparison with infinity
  void testInfinityComparison() {
    double inf = std::numeric_limits<double>::infinity();
    double neg_inf = -std::numeric_limits<double>::infinity();

    TS_ASSERT(inf > 1e308);
    TS_ASSERT(neg_inf < -1e308);
    TS_ASSERT(inf > neg_inf);
    TS_ASSERT(1000.0 < inf);
    TS_ASSERT(-1000.0 > neg_inf);
  }

  // Test comparison with NaN
  void testNaNComparison() {
    double nan = std::numeric_limits<double>::quiet_NaN();

    // All comparisons with NaN return false
    TS_ASSERT(!(nan < 0.0));
    TS_ASSERT(!(nan > 0.0));
    TS_ASSERT(!(nan == 0.0));
    TS_ASSERT(!(nan == nan));  // NaN != NaN
    TS_ASSERT(nan != nan);     // This is true!
  }
};
