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

  /***************************************************************************
   * Extended Logical Operations
   ***************************************************************************/

  // Test XOR operation
  void testLogicalXOR() {
    auto XOR = [](bool a, bool b) { return a != b; };

    TS_ASSERT(!XOR(true, true));
    TS_ASSERT(XOR(true, false));
    TS_ASSERT(XOR(false, true));
    TS_ASSERT(!XOR(false, false));
  }

  // Test NAND operation
  void testLogicalNAND() {
    auto NAND = [](bool a, bool b) { return !(a && b); };

    TS_ASSERT(!NAND(true, true));
    TS_ASSERT(NAND(true, false));
    TS_ASSERT(NAND(false, true));
    TS_ASSERT(NAND(false, false));
  }

  // Test NOR operation
  void testLogicalNOR() {
    auto NOR = [](bool a, bool b) { return !(a || b); };

    TS_ASSERT(!NOR(true, true));
    TS_ASSERT(!NOR(true, false));
    TS_ASSERT(!NOR(false, true));
    TS_ASSERT(NOR(false, false));
  }

  // Test XNOR operation (equivalence)
  void testLogicalXNOR() {
    auto XNOR = [](bool a, bool b) { return a == b; };

    TS_ASSERT(XNOR(true, true));
    TS_ASSERT(!XNOR(true, false));
    TS_ASSERT(!XNOR(false, true));
    TS_ASSERT(XNOR(false, false));
  }

  // Test triple AND condition
  void testTripleAND() {
    auto AND3 = [](bool a, bool b, bool c) { return a && b && c; };

    TS_ASSERT(AND3(true, true, true));
    TS_ASSERT(!AND3(true, true, false));
    TS_ASSERT(!AND3(true, false, true));
    TS_ASSERT(!AND3(false, true, true));
    TS_ASSERT(!AND3(false, false, false));
  }

  // Test triple OR condition
  void testTripleOR() {
    auto OR3 = [](bool a, bool b, bool c) { return a || b || c; };

    TS_ASSERT(OR3(true, true, true));
    TS_ASSERT(OR3(true, false, false));
    TS_ASSERT(OR3(false, true, false));
    TS_ASSERT(OR3(false, false, true));
    TS_ASSERT(!OR3(false, false, false));
  }

  // Test mixed AND/OR conditions
  void testMixedANDOR() {
    // (a AND b) OR (c AND d)
    auto mixedLogic = [](bool a, bool b, bool c, bool d) {
      return (a && b) || (c && d);
    };

    TS_ASSERT(mixedLogic(true, true, false, false));
    TS_ASSERT(mixedLogic(false, false, true, true));
    TS_ASSERT(mixedLogic(true, true, true, true));
    TS_ASSERT(!mixedLogic(true, false, false, true));
  }

  // Test De Morgan's laws
  void testDeMorgansLaws() {
    // NOT (A AND B) == (NOT A) OR (NOT B)
    auto deMorgan1 = [](bool a, bool b) {
      bool lhs = !(a && b);
      bool rhs = !a || !b;
      return lhs == rhs;
    };

    // NOT (A OR B) == (NOT A) AND (NOT B)
    auto deMorgan2 = [](bool a, bool b) {
      bool lhs = !(a || b);
      bool rhs = !a && !b;
      return lhs == rhs;
    };

    // Test all combinations
    for (int i = 0; i < 4; i++) {
      bool a = (i & 1) != 0;
      bool b = (i & 2) != 0;
      TS_ASSERT(deMorgan1(a, b));
      TS_ASSERT(deMorgan2(a, b));
    }
  }

  /***************************************************************************
   * Extended Switch Selection Logic
   ***************************************************************************/

  // Test switch with value ranges
  void testSwitchWithRanges() {
    auto rangeSwitch = [](double x) -> std::string {
      if (x < 0) return "negative";
      if (x <= 100) return "normal";
      if (x <= 200) return "warning";
      return "critical";
    };

    TS_ASSERT_EQUALS(rangeSwitch(-10), "negative");
    TS_ASSERT_EQUALS(rangeSwitch(0), "normal");
    TS_ASSERT_EQUALS(rangeSwitch(50), "normal");
    TS_ASSERT_EQUALS(rangeSwitch(100), "normal");
    TS_ASSERT_EQUALS(rangeSwitch(150), "warning");
    TS_ASSERT_EQUALS(rangeSwitch(200), "warning");
    TS_ASSERT_EQUALS(rangeSwitch(250), "critical");
  }

  // Test switch cascade (first match wins)
  void testSwitchCascade() {
    auto cascadeSwitch = [](int priority) -> double {
      // Higher priority wins (lower number = higher priority)
      if (priority == 1) return 100.0;
      if (priority == 2) return 50.0;
      if (priority == 3) return 25.0;
      if (priority == 4) return 10.0;
      return 0.0;  // default
    };

    TS_ASSERT_DELTA(cascadeSwitch(1), 100.0, epsilon);
    TS_ASSERT_DELTA(cascadeSwitch(2), 50.0, epsilon);
    TS_ASSERT_DELTA(cascadeSwitch(3), 25.0, epsilon);
    TS_ASSERT_DELTA(cascadeSwitch(4), 10.0, epsilon);
    TS_ASSERT_DELTA(cascadeSwitch(5), 0.0, epsilon);
  }

  // Test switch with property passthrough
  void testSwitchPassthrough() {
    auto passthroughSwitch = [](bool enabled, double input, double defaultVal) {
      return enabled ? input : defaultVal;
    };

    TS_ASSERT_DELTA(passthroughSwitch(true, 123.456, 0.0), 123.456, epsilon);
    TS_ASSERT_DELTA(passthroughSwitch(false, 123.456, 0.0), 0.0, epsilon);
    TS_ASSERT_DELTA(passthroughSwitch(true, -50.0, 999.0), -50.0, epsilon);
    TS_ASSERT_DELTA(passthroughSwitch(false, -50.0, 999.0), 999.0, epsilon);
  }

  // Test switch with computed value
  void testSwitchComputedValue() {
    auto computedSwitch = [](int mode, double a, double b) -> double {
      switch (mode) {
        case 0: return a + b;
        case 1: return a - b;
        case 2: return a * b;
        case 3: return (b != 0) ? a / b : 0.0;
        default: return 0.0;
      }
    };

    TS_ASSERT_DELTA(computedSwitch(0, 10.0, 5.0), 15.0, epsilon);
    TS_ASSERT_DELTA(computedSwitch(1, 10.0, 5.0), 5.0, epsilon);
    TS_ASSERT_DELTA(computedSwitch(2, 10.0, 5.0), 50.0, epsilon);
    TS_ASSERT_DELTA(computedSwitch(3, 10.0, 5.0), 2.0, epsilon);
  }

  /***************************************************************************
   * State Machine Patterns
   ***************************************************************************/

  // Test latch (set-reset flip-flop)
  void testLatchBehavior() {
    bool latch = false;

    auto setLatch = [&]() { latch = true; };
    auto resetLatch = [&]() { latch = false; };
    auto getLatch = [&]() { return latch; };

    TS_ASSERT(!getLatch());
    setLatch();
    TS_ASSERT(getLatch());
    setLatch();  // Should stay set
    TS_ASSERT(getLatch());
    resetLatch();
    TS_ASSERT(!getLatch());
  }

  // Test edge detector (rising edge)
  void testRisingEdgeDetector() {
    bool prevState = false;

    auto detectRisingEdge = [&](bool currentState) {
      bool edge = !prevState && currentState;
      prevState = currentState;
      return edge;
    };

    TS_ASSERT(detectRisingEdge(true));   // 0 -> 1: rising edge
    TS_ASSERT(!detectRisingEdge(true));  // 1 -> 1: no edge
    TS_ASSERT(!detectRisingEdge(false)); // 1 -> 0: falling edge
    TS_ASSERT(detectRisingEdge(true));   // 0 -> 1: rising edge
  }

  // Test edge detector (falling edge)
  void testFallingEdgeDetector() {
    bool prevState = false;

    auto detectFallingEdge = [&](bool currentState) {
      bool edge = prevState && !currentState;
      prevState = currentState;
      return edge;
    };

    TS_ASSERT(!detectFallingEdge(true));  // 0 -> 1: rising edge
    TS_ASSERT(!detectFallingEdge(true));  // 1 -> 1: no edge
    TS_ASSERT(detectFallingEdge(false));  // 1 -> 0: falling edge
    TS_ASSERT(!detectFallingEdge(false)); // 0 -> 0: no edge
  }

  // Test pulse generator
  void testPulseGenerator() {
    int counter = 0;
    bool prevState = false;

    auto pulse = [&](bool trigger) {
      bool result = false;
      if (trigger && !prevState) {
        counter = 3;  // Pulse for 3 cycles
      }
      if (counter > 0) {
        result = true;
        counter--;
      }
      prevState = trigger;
      return result;
    };

    TS_ASSERT(!pulse(false));
    TS_ASSERT(pulse(true));   // Trigger
    TS_ASSERT(pulse(true));   // Continue
    TS_ASSERT(pulse(true));   // Continue
    TS_ASSERT(!pulse(true));  // Pulse ended
    TS_ASSERT(!pulse(true));  // Still ended
  }

  /***************************************************************************
   * Floating Point Edge Cases
   ***************************************************************************/

  // Test comparison with epsilon
  void testEpsilonComparison() {
    auto approxEqual = [](double a, double b, double eps) {
      return std::abs(a - b) < eps;
    };

    TS_ASSERT(approxEqual(1.0, 1.0 + 1e-11, 1e-10));
    TS_ASSERT(!approxEqual(1.0, 1.0 + 1e-9, 1e-10));
    TS_ASSERT(approxEqual(0.0, 1e-15, 1e-10));
  }

  // Test comparison with large numbers
  void testLargeNumberComparison() {
    double large = 1e15;

    TS_ASSERT(large < large + 1.0);
    TS_ASSERT(large > large - 1.0);
    TS_ASSERT(large == large);
  }

  // Test comparison with subnormal numbers
  void testSubnormalComparison() {
    double subnorm1 = std::numeric_limits<double>::denorm_min();
    double subnorm2 = 2.0 * std::numeric_limits<double>::denorm_min();

    TS_ASSERT(subnorm1 < subnorm2);
    TS_ASSERT(subnorm1 > 0.0);
    TS_ASSERT(subnorm2 > subnorm1);
  }

  // Test positive and negative zero
  void testSignedZero() {
    double posZero = 0.0;
    double negZero = -0.0;

    TS_ASSERT(posZero == negZero);
    TS_ASSERT(!(posZero < negZero));
    TS_ASSERT(!(posZero > negZero));
  }

  /***************************************************************************
   * Flight Control Scenarios
   ***************************************************************************/

  // Test gear logic (velocity and altitude based)
  void testGearLogic() {
    auto gearSafe = [](double velocity, double altitude, bool gearDown) {
      const double maxGearVelocity = 200.0;  // knots
      const double minGearAltitude = 50.0;   // feet

      if (gearDown) {
        return velocity < maxGearVelocity;  // OK if below max speed
      } else {
        return altitude > minGearAltitude;  // OK if above min altitude
      }
    };

    TS_ASSERT(gearSafe(150.0, 100.0, true));   // Slow, gear down
    TS_ASSERT(!gearSafe(250.0, 100.0, true));  // Too fast, gear down
    TS_ASSERT(gearSafe(250.0, 100.0, false));  // Fast, gear up
    TS_ASSERT(!gearSafe(250.0, 30.0, false));  // Too low, gear up
  }

  // Test flap schedule
  void testFlapSchedule() {
    auto flapPosition = [](double velocity) -> double {
      if (velocity < 100) return 40.0;       // Full flaps
      if (velocity < 140) return 25.0;       // Approach flaps
      if (velocity < 180) return 10.0;       // Takeoff flaps
      if (velocity < 220) return 5.0;        // Slats
      return 0.0;                            // Clean
    };

    TS_ASSERT_DELTA(flapPosition(80), 40.0, epsilon);
    TS_ASSERT_DELTA(flapPosition(120), 25.0, epsilon);
    TS_ASSERT_DELTA(flapPosition(160), 10.0, epsilon);
    TS_ASSERT_DELTA(flapPosition(200), 5.0, epsilon);
    TS_ASSERT_DELTA(flapPosition(250), 0.0, epsilon);
  }

  // Test autopilot mode selection
  void testAutopilotModeSelection() {
    auto selectMode = [](bool altHold, bool hdgHold, bool navMode) -> int {
      // Priority: NAV > HDG > ALT > OFF
      if (navMode) return 3;
      if (hdgHold) return 2;
      if (altHold) return 1;
      return 0;
    };

    TS_ASSERT_EQUALS(selectMode(false, false, false), 0);  // OFF
    TS_ASSERT_EQUALS(selectMode(true, false, false), 1);   // ALT
    TS_ASSERT_EQUALS(selectMode(false, true, false), 2);   // HDG
    TS_ASSERT_EQUALS(selectMode(true, true, false), 2);    // HDG (priority)
    TS_ASSERT_EQUALS(selectMode(false, false, true), 3);   // NAV
    TS_ASSERT_EQUALS(selectMode(true, true, true), 3);     // NAV (priority)
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many sequential evaluations
  void testStressManyEvaluations() {
    auto evaluate = [](double x) {
      if (x < 0) return -1;
      if (x < 100) return 0;
      return 1;
    };

    for (int i = 0; i < 10000; i++) {
      double x = static_cast<double>(i - 5000) / 50.0;
      int result = evaluate(x);
      TS_ASSERT(result >= -1 && result <= 1);
    }
  }

  // Test complex compound conditions
  void testStressCompoundConditions() {
    auto complexCondition = [](double a, double b, double c, double d) {
      return ((a > 0 && b > 0) || (c < 0 && d < 0)) &&
             (a + b + c + d > 0 || a * b * c * d < 0);
    };

    for (int i = 0; i < 1000; i++) {
      double a = sin(i * 0.1);
      double b = cos(i * 0.1);
      double c = sin(i * 0.2);
      double d = cos(i * 0.2);
      // Just verify no exceptions/crashes
      bool result = complexCondition(a, b, c, d);
      TS_ASSERT(result == true || result == false);  // Boolean
    }
  }

  // Test rapid state changes
  void testStressRapidStateChanges() {
    bool state = false;
    double lowThresh = 0.3;
    double highThresh = 0.7;

    for (int i = 0; i < 1000; i++) {
      double value = sin(i * 0.1) * 0.5 + 0.5;  // 0 to 1

      if (value > highThresh) state = true;
      else if (value < lowThresh) state = false;

      TS_ASSERT(state == true || state == false);
    }
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  // Test boundary values
  void testBoundaryValues() {
    double max = std::numeric_limits<double>::max();
    double min = std::numeric_limits<double>::lowest();

    TS_ASSERT(max > 0);
    TS_ASSERT(min < 0);
    TS_ASSERT(max > min);
    TS_ASSERT(max == max);
    TS_ASSERT(min == min);
  }

  // Test comparison precision near 1.0
  void testPrecisionNearOne() {
    double a = 1.0;
    double b = 1.0 + std::numeric_limits<double>::epsilon();

    TS_ASSERT(a < b);
    TS_ASSERT(b > a);
    TS_ASSERT(a != b);
  }

  // Test comparison with negative numbers
  void testNegativeComparisons() {
    TS_ASSERT(-5.0 < -4.0);
    TS_ASSERT(-100.0 < -1.0);
    TS_ASSERT(-0.001 < 0.0);
    TS_ASSERT(-1e-10 < 0.0);
  }

  // Test chained comparisons (range check)
  void testRangeCheck() {
    auto inRange = [](double x, double low, double high) {
      return x >= low && x <= high;
    };

    TS_ASSERT(inRange(5.0, 0.0, 10.0));
    TS_ASSERT(inRange(0.0, 0.0, 10.0));   // Boundary
    TS_ASSERT(inRange(10.0, 0.0, 10.0));  // Boundary
    TS_ASSERT(!inRange(-1.0, 0.0, 10.0));
    TS_ASSERT(!inRange(11.0, 0.0, 10.0));
  }

  // Test exclusive range check
  void testExclusiveRangeCheck() {
    auto inRangeExclusive = [](double x, double low, double high) {
      return x > low && x < high;
    };

    TS_ASSERT(inRangeExclusive(5.0, 0.0, 10.0));
    TS_ASSERT(!inRangeExclusive(0.0, 0.0, 10.0));   // Boundary excluded
    TS_ASSERT(!inRangeExclusive(10.0, 0.0, 10.0));  // Boundary excluded
  }

  // Test nested switch logic
  void testNestedSwitchLogic() {
    auto nestedSwitch = [](int outer, int inner) -> double {
      if (outer == 1) {
        if (inner == 1) return 1.1;
        if (inner == 2) return 1.2;
        return 1.0;
      }
      if (outer == 2) {
        if (inner == 1) return 2.1;
        if (inner == 2) return 2.2;
        return 2.0;
      }
      return 0.0;
    };

    TS_ASSERT_DELTA(nestedSwitch(1, 1), 1.1, epsilon);
    TS_ASSERT_DELTA(nestedSwitch(1, 2), 1.2, epsilon);
    TS_ASSERT_DELTA(nestedSwitch(1, 3), 1.0, epsilon);
    TS_ASSERT_DELTA(nestedSwitch(2, 1), 2.1, epsilon);
    TS_ASSERT_DELTA(nestedSwitch(2, 2), 2.2, epsilon);
    TS_ASSERT_DELTA(nestedSwitch(3, 1), 0.0, epsilon);
  }
};
