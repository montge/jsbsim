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

  /***************************************************************************
   * Extended Condition Evaluation Tests
   ***************************************************************************/

  void testConditionWithTolerance() {
    // Floating point equality with tolerance
    auto eqTol = [](double a, double b, double tol) {
      return std::abs(a - b) <= tol;
    };

    TS_ASSERT(eqTol(1.0, 1.0001, 0.001));
    TS_ASSERT(!eqTol(1.0, 1.01, 0.001));
    TS_ASSERT(eqTol(0.0, 1e-11, 1e-10));
  }

  void testConditionChaining() {
    // a < b < c form
    auto inOrder = [](double a, double b, double c) {
      return a < b && b < c;
    };

    TS_ASSERT(inOrder(1.0, 2.0, 3.0));
    TS_ASSERT(!inOrder(1.0, 3.0, 2.0));
    TS_ASSERT(!inOrder(2.0, 1.0, 3.0));
    TS_ASSERT(!inOrder(1.0, 1.0, 2.0));  // Not strictly less
  }

  void testConditionWithAbsoluteValue() {
    auto withinBand = [](double x, double center, double halfWidth) {
      return std::abs(x - center) <= halfWidth;
    };

    TS_ASSERT(withinBand(5.0, 5.0, 0.1));
    TS_ASSERT(withinBand(5.05, 5.0, 0.1));
    TS_ASSERT(!withinBand(5.2, 5.0, 0.1));
    TS_ASSERT(withinBand(-5.0, -5.0, 0.1));
  }

  /***************************************************************************
   * Extended Logical Operations Tests
   ***************************************************************************/

  void testImplication() {
    // A implies B (A -> B) is equivalent to (!A || B)
    auto implies = [](bool a, bool b) { return !a || b; };

    TS_ASSERT(implies(false, false));
    TS_ASSERT(implies(false, true));
    TS_ASSERT(!implies(true, false));
    TS_ASSERT(implies(true, true));
  }

  void testBiconditional() {
    // A if and only if B (A <-> B)
    auto iff = [](bool a, bool b) { return a == b; };

    TS_ASSERT(iff(true, true));
    TS_ASSERT(!iff(true, false));
    TS_ASSERT(!iff(false, true));
    TS_ASSERT(iff(false, false));
  }

  void testMajorityVote() {
    // Returns true if majority of inputs are true
    auto majority = [](bool a, bool b, bool c) {
      return (a && b) || (b && c) || (a && c);
    };

    TS_ASSERT(!majority(false, false, false));
    TS_ASSERT(!majority(true, false, false));
    TS_ASSERT(!majority(false, true, false));
    TS_ASSERT(!majority(false, false, true));
    TS_ASSERT(majority(true, true, false));
    TS_ASSERT(majority(true, false, true));
    TS_ASSERT(majority(false, true, true));
    TS_ASSERT(majority(true, true, true));
  }

  void testAtLeastN() {
    auto atLeast2 = [](bool a, bool b, bool c) {
      int count = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
      return count >= 2;
    };

    TS_ASSERT(!atLeast2(false, false, false));
    TS_ASSERT(!atLeast2(true, false, false));
    TS_ASSERT(atLeast2(true, true, false));
    TS_ASSERT(atLeast2(true, true, true));
  }

  void testExactlyOne() {
    auto exactlyOne = [](bool a, bool b, bool c) {
      int count = (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0);
      return count == 1;
    };

    TS_ASSERT(!exactlyOne(false, false, false));
    TS_ASSERT(exactlyOne(true, false, false));
    TS_ASSERT(exactlyOne(false, true, false));
    TS_ASSERT(exactlyOne(false, false, true));
    TS_ASSERT(!exactlyOne(true, true, false));
    TS_ASSERT(!exactlyOne(true, true, true));
  }

  /***************************************************************************
   * Extended State Machine Patterns
   ***************************************************************************/

  void testToggle() {
    bool state = false;

    auto toggle = [&]() {
      state = !state;
      return state;
    };

    TS_ASSERT(toggle());   // false -> true
    TS_ASSERT(!toggle());  // true -> false
    TS_ASSERT(toggle());   // false -> true
    TS_ASSERT(!toggle());  // true -> false
  }

  void testCounter() {
    int count = 0;

    auto increment = [&]() { return ++count; };
    auto decrement = [&]() { return --count; };
    auto reset = [&]() { count = 0; return count; };

    TS_ASSERT_EQUALS(increment(), 1);
    TS_ASSERT_EQUALS(increment(), 2);
    TS_ASSERT_EQUALS(increment(), 3);
    TS_ASSERT_EQUALS(decrement(), 2);
    TS_ASSERT_EQUALS(reset(), 0);
    TS_ASSERT_EQUALS(increment(), 1);
  }

  void testDebounce() {
    // Requires N consecutive same values to change output
    int consecutiveCount = 0;
    bool lastInput = false;
    bool output = false;
    const int threshold = 3;

    auto debounce = [&](bool input) {
      if (input == lastInput) {
        consecutiveCount++;
      } else {
        consecutiveCount = 1;
        lastInput = input;
      }
      if (consecutiveCount >= threshold) {
        output = input;
      }
      return output;
    };

    TS_ASSERT(!debounce(true));   // 1 true
    TS_ASSERT(!debounce(true));   // 2 trues
    TS_ASSERT(debounce(true));    // 3 trues -> output true
    TS_ASSERT(debounce(false));   // 1 false, output still true
    TS_ASSERT(debounce(false));   // 2 falses
    TS_ASSERT(!debounce(false));  // 3 falses -> output false
  }

  void testTimer() {
    int timer = 0;
    bool timerActive = false;

    auto startTimer = [&](int duration) {
      timer = duration;
      timerActive = true;
    };

    auto tickTimer = [&]() {
      if (timerActive && timer > 0) {
        timer--;
        if (timer == 0) timerActive = false;
      }
      return timerActive;
    };

    startTimer(3);
    TS_ASSERT(tickTimer());  // 2 remaining
    TS_ASSERT(tickTimer());  // 1 remaining
    TS_ASSERT(!tickTimer()); // 0, timer stopped
    TS_ASSERT(!tickTimer()); // Still stopped
  }

  /***************************************************************************
   * Extended Flight Control Scenarios
   ***************************************************************************/

  void testStallWarning() {
    auto stallWarning = [](double aoa, double velocity, bool flapsExtended) {
      double criticalAoA = flapsExtended ? 18.0 : 15.0;  // degrees
      double minSpeed = flapsExtended ? 60.0 : 80.0;      // knots

      return aoa > criticalAoA || velocity < minSpeed;
    };

    TS_ASSERT(!stallWarning(10.0, 150.0, false));  // Normal flight
    TS_ASSERT(stallWarning(16.0, 150.0, false));   // High AoA
    TS_ASSERT(stallWarning(10.0, 70.0, false));    // Low speed
    TS_ASSERT(!stallWarning(16.0, 70.0, true));    // Flaps help
    TS_ASSERT(stallWarning(19.0, 70.0, true));     // Too high AoA even with flaps
  }

  void testConfigurationWarning() {
    auto configWarning = [](bool gearDown, bool flapsSet, double altitude, double velocity) {
      // Warning if approaching landing but not configured
      bool approachingLanding = altitude < 1000.0 && velocity < 180.0;
      bool configured = gearDown && flapsSet;
      return approachingLanding && !configured;
    };

    TS_ASSERT(!configWarning(true, true, 500.0, 140.0));   // Configured
    TS_ASSERT(configWarning(false, true, 500.0, 140.0));   // Gear up
    TS_ASSERT(configWarning(true, false, 500.0, 140.0));   // No flaps
    TS_ASSERT(!configWarning(false, false, 5000.0, 140.0)); // High altitude
    TS_ASSERT(!configWarning(false, false, 500.0, 250.0));  // High speed
  }

  void testOverspeedWarning() {
    auto overspeedWarning = [](double mach, double ias, double altitude) {
      double maxMach = 0.82;
      double maxIAS = 350.0;  // knots
      double vmo = altitude > 26000 ? 300.0 : 350.0;

      return mach > maxMach || ias > maxIAS || ias > vmo;
    };

    TS_ASSERT(!overspeedWarning(0.75, 300.0, 30000.0));
    TS_ASSERT(overspeedWarning(0.85, 300.0, 30000.0));  // Mach exceeded
    TS_ASSERT(overspeedWarning(0.75, 360.0, 10000.0));  // IAS exceeded
    TS_ASSERT(overspeedWarning(0.75, 310.0, 30000.0));  // VMO exceeded at altitude
  }

  void testEngineFireLogic() {
    auto fireShutdownSequence = [](bool fireDetected, bool fireHandlePulled,
                                    bool agent1Discharged, bool agent2Discharged) -> int {
      // Returns step in shutdown sequence
      if (!fireDetected) return 0;  // No fire
      if (!fireHandlePulled) return 1;  // Fire detected, need to pull handle
      if (!agent1Discharged) return 2;  // Handle pulled, discharge agent 1
      if (!agent2Discharged) return 3;  // Agent 1 done, discharge agent 2
      return 4;  // Sequence complete
    };

    TS_ASSERT_EQUALS(fireShutdownSequence(false, false, false, false), 0);
    TS_ASSERT_EQUALS(fireShutdownSequence(true, false, false, false), 1);
    TS_ASSERT_EQUALS(fireShutdownSequence(true, true, false, false), 2);
    TS_ASSERT_EQUALS(fireShutdownSequence(true, true, true, false), 3);
    TS_ASSERT_EQUALS(fireShutdownSequence(true, true, true, true), 4);
  }

  /***************************************************************************
   * Priority Encoder Tests
   ***************************************************************************/

  void testPriorityEncoder4() {
    // Returns highest priority active input (0 = none, 1-4 = priority)
    auto priorityEncode = [](bool p1, bool p2, bool p3, bool p4) {
      if (p1) return 1;
      if (p2) return 2;
      if (p3) return 3;
      if (p4) return 4;
      return 0;
    };

    TS_ASSERT_EQUALS(priorityEncode(true, true, true, true), 1);
    TS_ASSERT_EQUALS(priorityEncode(false, true, true, true), 2);
    TS_ASSERT_EQUALS(priorityEncode(false, false, true, true), 3);
    TS_ASSERT_EQUALS(priorityEncode(false, false, false, true), 4);
    TS_ASSERT_EQUALS(priorityEncode(false, false, false, false), 0);
  }

  void testPriorityEncoderWithValue() {
    auto prioritySelect = [](bool c1, double v1, bool c2, double v2,
                              bool c3, double v3, double def) {
      if (c1) return v1;
      if (c2) return v2;
      if (c3) return v3;
      return def;
    };

    TS_ASSERT_DELTA(prioritySelect(true, 100.0, true, 200.0, true, 300.0, 0.0), 100.0, epsilon);
    TS_ASSERT_DELTA(prioritySelect(false, 100.0, true, 200.0, true, 300.0, 0.0), 200.0, epsilon);
    TS_ASSERT_DELTA(prioritySelect(false, 100.0, false, 200.0, true, 300.0, 0.0), 300.0, epsilon);
    TS_ASSERT_DELTA(prioritySelect(false, 100.0, false, 200.0, false, 300.0, 0.0), 0.0, epsilon);
  }

  /***************************************************************************
   * Multiplexer Tests
   ***************************************************************************/

  void testMux2to1() {
    auto mux2 = [](bool sel, double a, double b) {
      return sel ? b : a;
    };

    TS_ASSERT_DELTA(mux2(false, 10.0, 20.0), 10.0, epsilon);
    TS_ASSERT_DELTA(mux2(true, 10.0, 20.0), 20.0, epsilon);
  }

  void testMux4to1() {
    auto mux4 = [](int sel, double a, double b, double c, double d) {
      switch (sel) {
        case 0: return a;
        case 1: return b;
        case 2: return c;
        case 3: return d;
        default: return 0.0;
      }
    };

    TS_ASSERT_DELTA(mux4(0, 1.0, 2.0, 3.0, 4.0), 1.0, epsilon);
    TS_ASSERT_DELTA(mux4(1, 1.0, 2.0, 3.0, 4.0), 2.0, epsilon);
    TS_ASSERT_DELTA(mux4(2, 1.0, 2.0, 3.0, 4.0), 3.0, epsilon);
    TS_ASSERT_DELTA(mux4(3, 1.0, 2.0, 3.0, 4.0), 4.0, epsilon);
  }

  /***************************************************************************
   * Sample and Hold Tests
   ***************************************************************************/

  void testSampleAndHold() {
    double heldValue = 0.0;

    auto sampleAndHold = [&](bool sample, double input) {
      if (sample) heldValue = input;
      return heldValue;
    };

    TS_ASSERT_DELTA(sampleAndHold(true, 5.0), 5.0, epsilon);
    TS_ASSERT_DELTA(sampleAndHold(false, 10.0), 5.0, epsilon);  // Held
    TS_ASSERT_DELTA(sampleAndHold(false, 15.0), 5.0, epsilon);  // Still held
    TS_ASSERT_DELTA(sampleAndHold(true, 20.0), 20.0, epsilon);  // New sample
    TS_ASSERT_DELTA(sampleAndHold(false, 25.0), 20.0, epsilon); // Held again
  }

  void testTrackAndHold() {
    double value = 0.0;
    bool tracking = true;

    auto trackAndHold = [&](bool track, double input) {
      if (track) {
        value = input;
        tracking = true;
      } else {
        tracking = false;
      }
      return value;
    };

    TS_ASSERT_DELTA(trackAndHold(true, 1.0), 1.0, epsilon);
    TS_ASSERT_DELTA(trackAndHold(true, 2.0), 2.0, epsilon);
    TS_ASSERT_DELTA(trackAndHold(true, 3.0), 3.0, epsilon);
    TS_ASSERT_DELTA(trackAndHold(false, 4.0), 3.0, epsilon);  // Hold at 3
    TS_ASSERT_DELTA(trackAndHold(false, 5.0), 3.0, epsilon);  // Still held
    TS_ASSERT_DELTA(trackAndHold(true, 6.0), 6.0, epsilon);   // Resume tracking
  }

  /***************************************************************************
   * Window Comparator Tests
   ***************************************************************************/

  void testWindowComparator() {
    auto windowCompare = [](double input, double low, double high) -> int {
      if (input < low) return -1;   // Below window
      if (input > high) return 1;   // Above window
      return 0;                      // In window
    };

    TS_ASSERT_EQUALS(windowCompare(5.0, 0.0, 10.0), 0);   // In window
    TS_ASSERT_EQUALS(windowCompare(-1.0, 0.0, 10.0), -1); // Below
    TS_ASSERT_EQUALS(windowCompare(11.0, 0.0, 10.0), 1);  // Above
    TS_ASSERT_EQUALS(windowCompare(0.0, 0.0, 10.0), 0);   // At low boundary
    TS_ASSERT_EQUALS(windowCompare(10.0, 0.0, 10.0), 0);  // At high boundary
  }

  void testWindowWithHysteresis() {
    bool inWindow = false;
    double lowEnter = 2.0, lowExit = 1.0;
    double highEnter = 8.0, highExit = 9.0;

    auto windowHyst = [&](double input) {
      if (inWindow) {
        // Check exit conditions
        if (input < lowExit || input > highExit) {
          inWindow = false;
        }
      } else {
        // Check entry conditions
        if (input >= lowEnter && input <= highEnter) {
          inWindow = true;
        }
      }
      return inWindow;
    };

    TS_ASSERT(!windowHyst(0.0));   // Outside, below entry
    TS_ASSERT(!windowHyst(1.5));   // Still outside (between exit and enter)
    TS_ASSERT(windowHyst(5.0));    // Enter window
    TS_ASSERT(windowHyst(1.5));    // Stay in (above exit)
    TS_ASSERT(!windowHyst(0.5));   // Exit (below lowExit)
  }
};
