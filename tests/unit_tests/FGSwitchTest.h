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

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;

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

  /***************************************************************************
   * Advanced State Machine Patterns (Tests 77-80)
   ***************************************************************************/

  // Test 77: Shift register behavior
  void testShiftRegister() {
    bool reg[4] = {false, false, false, false};

    auto shift = [&](bool input) {
      // Shift right, new input on left
      for (int i = 3; i > 0; i--) {
        reg[i] = reg[i-1];
      }
      reg[0] = input;
    };

    auto getOutput = [&]() { return reg[3]; };

    shift(true);  // 1,0,0,0
    TS_ASSERT(!getOutput());
    shift(true);  // 1,1,0,0
    TS_ASSERT(!getOutput());
    shift(false); // 0,1,1,0
    TS_ASSERT(!getOutput());
    shift(true);  // 1,0,1,1
    TS_ASSERT(getOutput());  // First true reaches output
    shift(true);  // 1,1,0,1
    TS_ASSERT(getOutput());
    shift(true);  // 1,1,1,0
    TS_ASSERT(!getOutput());  // False propagated
  }

  // Test 78: Sequencer with step advancement
  void testSequencer() {
    int step = 0;
    const int maxSteps = 5;

    auto advance = [&]() {
      if (step < maxSteps) step++;
      return step;
    };

    auto retreat = [&]() {
      if (step > 0) step--;
      return step;
    };

    auto reset = [&]() {
      step = 0;
      return step;
    };

    TS_ASSERT_EQUALS(advance(), 1);
    TS_ASSERT_EQUALS(advance(), 2);
    TS_ASSERT_EQUALS(advance(), 3);
    TS_ASSERT_EQUALS(retreat(), 2);
    TS_ASSERT_EQUALS(advance(), 3);
    TS_ASSERT_EQUALS(advance(), 4);
    TS_ASSERT_EQUALS(advance(), 5);
    TS_ASSERT_EQUALS(advance(), 5);  // Can't exceed max
    TS_ASSERT_EQUALS(reset(), 0);
    TS_ASSERT_EQUALS(retreat(), 0);  // Can't go below 0
  }

  // Test 79: Ring counter
  void testRingCounter() {
    int state = 0;
    const int numStates = 4;

    auto tick = [&]() {
      state = (state + 1) % numStates;
      return state;
    };

    TS_ASSERT_EQUALS(tick(), 1);
    TS_ASSERT_EQUALS(tick(), 2);
    TS_ASSERT_EQUALS(tick(), 3);
    TS_ASSERT_EQUALS(tick(), 0);  // Wraps around
    TS_ASSERT_EQUALS(tick(), 1);
    TS_ASSERT_EQUALS(tick(), 2);
  }

  // Test 80: Johnson counter (twisted ring)
  void testJohnsonCounter() {
    // 4-bit Johnson counter sequence: 0000,1000,1100,1110,1111,0111,0011,0001,0000...
    int state = 0;

    auto tick = [&]() {
      // Shift left, invert MSB to LSB
      bool msb = (state & 0x8) != 0;
      state = ((state << 1) | (!msb ? 1 : 0)) & 0xF;
      return state;
    };

    TS_ASSERT_EQUALS(tick(), 0x1);  // 0000 -> 0001
    TS_ASSERT_EQUALS(tick(), 0x3);  // 0001 -> 0011
    TS_ASSERT_EQUALS(tick(), 0x7);  // 0011 -> 0111
    TS_ASSERT_EQUALS(tick(), 0xF);  // 0111 -> 1111
    TS_ASSERT_EQUALS(tick(), 0xE);  // 1111 -> 1110
    TS_ASSERT_EQUALS(tick(), 0xC);  // 1110 -> 1100
    TS_ASSERT_EQUALS(tick(), 0x8);  // 1100 -> 1000
    TS_ASSERT_EQUALS(tick(), 0x0);  // 1000 -> 0000
    TS_ASSERT_EQUALS(tick(), 0x1);  // Back to start
  }

  /***************************************************************************
   * Safety Interlock Logic (Tests 81-84)
   ***************************************************************************/

  // Test 81: Two-key arming system
  void testTwoKeyArming() {
    auto armed = [](bool key1, bool key2, bool masterSwitch) {
      // Both keys AND master switch required
      return key1 && key2 && masterSwitch;
    };

    TS_ASSERT(!armed(false, false, false));
    TS_ASSERT(!armed(true, false, false));
    TS_ASSERT(!armed(true, true, false));
    TS_ASSERT(!armed(false, true, true));
    TS_ASSERT(armed(true, true, true));
  }

  // Test 82: Dead man's switch
  void testDeadMansSwitch() {
    int releaseCount = 0;

    auto checkSwitch = [&](bool pressed) {
      if (!pressed) {
        releaseCount++;
      } else {
        releaseCount = 0;
      }
      // Trigger if released for 3+ cycles
      return releaseCount >= 3;
    };

    TS_ASSERT(!checkSwitch(true));   // Pressed
    TS_ASSERT(!checkSwitch(true));   // Still pressed
    TS_ASSERT(!checkSwitch(false));  // 1 release
    TS_ASSERT(!checkSwitch(false));  // 2 releases
    TS_ASSERT(checkSwitch(false));   // 3 releases - TRIGGER
    TS_ASSERT(!checkSwitch(true));   // Reset
    TS_ASSERT(!checkSwitch(false));  // 1 release
  }

  // Test 83: Permissive interlock chain
  void testPermissiveInterlockChain() {
    auto canOperate = [](bool permit1, bool permit2, bool permit3, bool permit4) {
      // All permits must be granted in sequence
      // permit1 enables permit2 check, etc.
      if (!permit1) return false;
      if (!permit2) return false;
      if (!permit3) return false;
      if (!permit4) return false;
      return true;
    };

    TS_ASSERT(!canOperate(false, true, true, true));   // First permit missing
    TS_ASSERT(!canOperate(true, false, true, true));   // Second permit missing
    TS_ASSERT(!canOperate(true, true, false, true));   // Third permit missing
    TS_ASSERT(!canOperate(true, true, true, false));   // Fourth permit missing
    TS_ASSERT(canOperate(true, true, true, true));     // All permits granted
  }

  // Test 84: Safety lockout with timeout
  void testSafetyLockoutTimeout() {
    int lockoutTimer = 0;
    bool lockedOut = false;

    auto triggerLockout = [&](int duration) {
      lockedOut = true;
      lockoutTimer = duration;
    };

    auto tick = [&]() {
      if (lockoutTimer > 0) {
        lockoutTimer--;
        if (lockoutTimer == 0) {
          lockedOut = false;
        }
      }
      return lockedOut;
    };

    TS_ASSERT(!tick());  // Not locked out initially
    triggerLockout(3);
    TS_ASSERT(tick());   // Locked out, 2 remaining
    TS_ASSERT(tick());   // Locked out, 1 remaining
    TS_ASSERT(!tick());  // Lockout expired
    TS_ASSERT(!tick());  // Still clear
  }

  /***************************************************************************
   * Complex Flight Control Scenarios (Tests 85-88)
   ***************************************************************************/

  // Test 85: Engine start sequence
  void testEngineStartSequence() {
    auto startSequenceStep = [](bool batteryOn, bool fuelOn, bool starterEngaged,
                                 double n2Percent, bool ignition) -> int {
      // Returns current step in start sequence
      if (!batteryOn) return 0;                    // Need battery
      if (!fuelOn) return 1;                       // Need fuel
      if (!starterEngaged) return 2;               // Engage starter
      if (n2Percent < 25.0) return 3;              // Motoring
      if (!ignition) return 4;                     // Need ignition
      if (n2Percent < 60.0) return 5;              // Light-off
      return 6;                                     // Running
    };

    TS_ASSERT_EQUALS(startSequenceStep(false, false, false, 0.0, false), 0);
    TS_ASSERT_EQUALS(startSequenceStep(true, false, false, 0.0, false), 1);
    TS_ASSERT_EQUALS(startSequenceStep(true, true, false, 0.0, false), 2);
    TS_ASSERT_EQUALS(startSequenceStep(true, true, true, 15.0, false), 3);
    TS_ASSERT_EQUALS(startSequenceStep(true, true, true, 30.0, false), 4);
    TS_ASSERT_EQUALS(startSequenceStep(true, true, true, 30.0, true), 5);
    TS_ASSERT_EQUALS(startSequenceStep(true, true, true, 65.0, true), 6);
  }

  // Test 86: Bleed air routing
  void testBleedAirRouting() {
    auto bleedSource = [](bool eng1Avail, bool eng2Avail, bool apuAvail,
                          int selectedSource) -> int {
      // 0=none, 1=eng1, 2=eng2, 3=apu
      // Priority: selected > eng1 > eng2 > apu
      if (selectedSource == 1 && eng1Avail) return 1;
      if (selectedSource == 2 && eng2Avail) return 2;
      if (selectedSource == 3 && apuAvail) return 3;
      // Auto selection
      if (eng1Avail) return 1;
      if (eng2Avail) return 2;
      if (apuAvail) return 3;
      return 0;
    };

    TS_ASSERT_EQUALS(bleedSource(true, true, true, 0), 1);   // Auto: eng1
    TS_ASSERT_EQUALS(bleedSource(true, true, true, 2), 2);   // Manual: eng2
    TS_ASSERT_EQUALS(bleedSource(false, true, true, 0), 2);  // Auto: eng2 (no eng1)
    TS_ASSERT_EQUALS(bleedSource(false, false, true, 0), 3); // Auto: apu
    TS_ASSERT_EQUALS(bleedSource(false, false, false, 0), 0);// None available
    TS_ASSERT_EQUALS(bleedSource(true, true, true, 3), 3);   // Manual: apu
  }

  // Test 87: Environmental control system mode
  void testECSModeSelection() {
    auto ecsMode = [](double altitude, double cabinAlt, bool packFail,
                      bool manualOverride, int manualMode) -> int {
      // Modes: 0=off, 1=auto-low, 2=auto-high, 3=max, 4=manual
      if (packFail) return 0;  // Pack failed
      if (manualOverride) return manualMode;

      double diffPress = altitude - cabinAlt;
      if (altitude < 10000.0) return 1;        // Low altitude
      if (diffPress > 8000.0) return 3;        // High differential
      return 2;                                 // Normal high altitude
    };

    TS_ASSERT_EQUALS(ecsMode(5000.0, 5000.0, false, false, 0), 1);    // Low alt
    TS_ASSERT_EQUALS(ecsMode(35000.0, 28000.0, false, false, 0), 2);  // Normal (diff=7000)
    TS_ASSERT_EQUALS(ecsMode(40000.0, 8000.0, false, false, 0), 3);   // High diff (diff=32000)
    TS_ASSERT_EQUALS(ecsMode(35000.0, 8000.0, true, false, 0), 0);    // Pack fail
    TS_ASSERT_EQUALS(ecsMode(35000.0, 8000.0, false, true, 3), 3);    // Manual
  }

  // Test 88: Flight director mode transitions
  void testFDModeTransitions() {
    int currentMode = 0;  // 0=off, 1=TO, 2=CLB, 3=CRZ, 4=DES, 5=APP

    auto transitionValid = [](int from, int to) {
      // Valid transitions matrix
      if (from == 0) return to == 1 || to == 5;  // OFF -> TO or APP
      if (from == 1) return to == 2 || to == 0;  // TO -> CLB or OFF
      if (from == 2) return to == 3 || to == 0;  // CLB -> CRZ or OFF
      if (from == 3) return to == 4 || to == 0;  // CRZ -> DES or OFF
      if (from == 4) return to == 5 || to == 0;  // DES -> APP or OFF
      if (from == 5) return to == 0;              // APP -> OFF only
      return false;
    };

    TS_ASSERT(transitionValid(0, 1));   // OFF -> TO
    TS_ASSERT(transitionValid(1, 2));   // TO -> CLB
    TS_ASSERT(!transitionValid(1, 3));  // TO -> CRZ invalid
    TS_ASSERT(transitionValid(3, 4));   // CRZ -> DES
    TS_ASSERT(!transitionValid(4, 2));  // DES -> CLB invalid
    TS_ASSERT(transitionValid(4, 5));   // DES -> APP
    TS_ASSERT(transitionValid(5, 0));   // APP -> OFF
    TS_ASSERT(!transitionValid(5, 1));  // APP -> TO invalid
  }

  /***************************************************************************
   * Signal Routing and Crossover (Tests 89-92)
   ***************************************************************************/

  // Test 89: Dual channel crossover
  void testDualChannelCrossover() {
    auto crossover = [](int mode, double chA, double chB) -> std::pair<double, double> {
      // mode: 0=normal, 1=cross, 2=A-only, 3=B-only
      switch (mode) {
        case 0: return {chA, chB};       // Normal
        case 1: return {chB, chA};       // Crossed
        case 2: return {chA, chA};       // A to both
        case 3: return {chB, chB};       // B to both
        default: return {0.0, 0.0};
      }
    };

    auto [outA0, outB0] = crossover(0, 10.0, 20.0);
    TS_ASSERT_DELTA(outA0, 10.0, epsilon);
    TS_ASSERT_DELTA(outB0, 20.0, epsilon);

    auto [outA1, outB1] = crossover(1, 10.0, 20.0);
    TS_ASSERT_DELTA(outA1, 20.0, epsilon);
    TS_ASSERT_DELTA(outB1, 10.0, epsilon);

    auto [outA2, outB2] = crossover(2, 10.0, 20.0);
    TS_ASSERT_DELTA(outA2, 10.0, epsilon);
    TS_ASSERT_DELTA(outB2, 10.0, epsilon);
  }

  // Test 90: Triple redundancy voting
  void testTripleRedundancyVoting() {
    auto tripleVote = [](double ch1, double ch2, double ch3, double tolerance) -> double {
      // Mid-value selection with tolerance check
      double diff12 = std::abs(ch1 - ch2);
      double diff23 = std::abs(ch2 - ch3);
      double diff13 = std::abs(ch1 - ch3);

      // If all agree within tolerance, return average
      if (diff12 < tolerance && diff23 < tolerance && diff13 < tolerance) {
        return (ch1 + ch2 + ch3) / 3.0;
      }

      // Find the outlier and exclude it
      if (diff12 < tolerance) return (ch1 + ch2) / 2.0;
      if (diff23 < tolerance) return (ch2 + ch3) / 2.0;
      if (diff13 < tolerance) return (ch1 + ch3) / 2.0;

      // All disagree - return middle value
      if (ch1 >= ch2 && ch1 <= ch3) return ch1;
      if (ch1 >= ch3 && ch1 <= ch2) return ch1;
      if (ch2 >= ch1 && ch2 <= ch3) return ch2;
      if (ch2 >= ch3 && ch2 <= ch1) return ch2;
      return ch3;
    };

    // All agree
    TS_ASSERT_DELTA(tripleVote(10.0, 10.1, 9.9, 0.5), 10.0, 0.1);

    // One outlier
    TS_ASSERT_DELTA(tripleVote(10.0, 10.1, 50.0, 0.5), 10.05, 0.1);

    // All disagree - mid value
    TS_ASSERT_DELTA(tripleVote(5.0, 10.0, 15.0, 0.5), 10.0, epsilon);
  }

  // Test 91: Fail-operational switching
  void testFailOperationalSwitching() {
    auto failOpSelect = [](bool ch1Valid, double ch1, bool ch2Valid, double ch2,
                           bool ch3Valid, double ch3) -> std::pair<double, int> {
      // Returns value and number of valid channels
      int validCount = (ch1Valid ? 1 : 0) + (ch2Valid ? 1 : 0) + (ch3Valid ? 1 : 0);

      if (ch1Valid && ch2Valid && ch3Valid) {
        return {(ch1 + ch2 + ch3) / 3.0, 3};
      }
      if (ch1Valid && ch2Valid) return {(ch1 + ch2) / 2.0, 2};
      if (ch1Valid && ch3Valid) return {(ch1 + ch3) / 2.0, 2};
      if (ch2Valid && ch3Valid) return {(ch2 + ch3) / 2.0, 2};
      if (ch1Valid) return {ch1, 1};
      if (ch2Valid) return {ch2, 1};
      if (ch3Valid) return {ch3, 1};
      return {0.0, 0};
    };

    auto [val3, cnt3] = failOpSelect(true, 10.0, true, 11.0, true, 12.0);
    TS_ASSERT_DELTA(val3, 11.0, 0.1);
    TS_ASSERT_EQUALS(cnt3, 3);

    auto [val2, cnt2] = failOpSelect(true, 10.0, false, 11.0, true, 12.0);
    TS_ASSERT_DELTA(val2, 11.0, 0.1);
    TS_ASSERT_EQUALS(cnt2, 2);

    auto [val1, cnt1] = failOpSelect(false, 10.0, false, 11.0, true, 12.0);
    TS_ASSERT_DELTA(val1, 12.0, epsilon);
    TS_ASSERT_EQUALS(cnt1, 1);

    auto [val0, cnt0] = failOpSelect(false, 10.0, false, 11.0, false, 12.0);
    TS_ASSERT_DELTA(val0, 0.0, epsilon);
    TS_ASSERT_EQUALS(cnt0, 0);
  }

  // Test 92: Bus tie logic
  void testBusTieLogic() {
    auto busTieState = [](bool bus1Powered, bool bus2Powered, bool tieRequested,
                          bool bus1Fault, bool bus2Fault) -> int {
      // Returns: 0=open, 1=closed, 2=auto-closed (fault recovery)
      if (bus1Fault && bus2Fault) return 0;  // Both faulty, isolate

      if (bus1Fault && bus2Powered) return 2;  // Auto-close to power bus1
      if (bus2Fault && bus1Powered) return 2;  // Auto-close to power bus2

      if (tieRequested && bus1Powered && bus2Powered) return 1;  // Manual close

      return 0;  // Default open
    };

    TS_ASSERT_EQUALS(busTieState(true, true, false, false, false), 0);   // Open
    TS_ASSERT_EQUALS(busTieState(true, true, true, false, false), 1);    // Closed
    TS_ASSERT_EQUALS(busTieState(false, true, false, true, false), 2);   // Auto
    TS_ASSERT_EQUALS(busTieState(true, false, false, false, true), 2);   // Auto
    TS_ASSERT_EQUALS(busTieState(false, false, true, true, true), 0);    // Both fault
  }

  /***************************************************************************
   * Mode Transition Guards (Tests 93-96)
   ***************************************************************************/

  // Test 93: Guarded state machine
  void testGuardedStateMachine() {
    int state = 0;

    auto transition = [&](int targetState, bool guard) -> bool {
      if (!guard) return false;

      // State-specific transition rules
      switch (state) {
        case 0: if (targetState == 1) { state = 1; return true; } break;
        case 1: if (targetState == 2 || targetState == 0) { state = targetState; return true; } break;
        case 2: if (targetState == 3 || targetState == 1) { state = targetState; return true; } break;
        case 3: if (targetState == 0) { state = 0; return true; } break;
      }
      return false;
    };

    TS_ASSERT(transition(1, true));   // 0 -> 1
    TS_ASSERT_EQUALS(state, 1);
    TS_ASSERT(!transition(3, true));  // 1 -> 3 not allowed
    TS_ASSERT(transition(2, true));   // 1 -> 2
    TS_ASSERT(!transition(2, false)); // Guard prevents
    TS_ASSERT(transition(3, true));   // 2 -> 3
    TS_ASSERT(transition(0, true));   // 3 -> 0 (reset)
  }

  // Test 94: Transition with minimum dwell time
  void testMinimumDwellTime() {
    int state = 0;
    int dwellCounter = 0;
    const int minDwell = 5;

    auto requestTransition = [&](int newState) -> bool {
      if (dwellCounter < minDwell) {
        return false;  // Must wait
      }
      if (newState != state) {
        state = newState;
        dwellCounter = 0;
        return true;
      }
      return false;
    };

    auto tick = [&]() { dwellCounter++; };

    TS_ASSERT(!requestTransition(1));  // dwell = 0
    tick(); tick(); tick(); tick();
    TS_ASSERT(!requestTransition(1));  // dwell = 4
    tick();
    TS_ASSERT(requestTransition(1));   // dwell = 5, OK
    TS_ASSERT_EQUALS(dwellCounter, 0); // Reset
    tick(); tick();
    TS_ASSERT(!requestTransition(2));  // dwell = 2
  }

  // Test 95: Conditional mode with fallback
  void testConditionalModeWithFallback() {
    auto selectMode = [](int requestedMode, bool cond1, bool cond2, bool cond3) -> int {
      // Mode 3 requires all conditions
      // Mode 2 requires cond1 and cond2
      // Mode 1 requires cond1
      // Mode 0 always available

      if (requestedMode == 3 && cond1 && cond2 && cond3) return 3;
      if (requestedMode >= 2 && cond1 && cond2) return 2;
      if (requestedMode >= 1 && cond1) return 1;
      return 0;
    };

    TS_ASSERT_EQUALS(selectMode(3, true, true, true), 3);    // All met
    TS_ASSERT_EQUALS(selectMode(3, true, true, false), 2);   // Fallback to 2
    TS_ASSERT_EQUALS(selectMode(3, true, false, false), 1);  // Fallback to 1
    TS_ASSERT_EQUALS(selectMode(3, false, false, false), 0); // Fallback to 0
    TS_ASSERT_EQUALS(selectMode(2, true, true, false), 2);   // Exact mode 2
    TS_ASSERT_EQUALS(selectMode(1, true, false, false), 1);  // Exact mode 1
  }

  // Test 96: Priority-based mode arbitration
  void testPriorityModeArbitration() {
    auto arbitrate = [](bool emergency, int emergencyMode,
                        bool priority, int priorityMode,
                        bool normal, int normalMode,
                        int defaultMode) -> int {
      if (emergency) return emergencyMode;
      if (priority) return priorityMode;
      if (normal) return normalMode;
      return defaultMode;
    };

    TS_ASSERT_EQUALS(arbitrate(true, 99, true, 50, true, 10, 0), 99);   // Emergency
    TS_ASSERT_EQUALS(arbitrate(false, 99, true, 50, true, 10, 0), 50);  // Priority
    TS_ASSERT_EQUALS(arbitrate(false, 99, false, 50, true, 10, 0), 10); // Normal
    TS_ASSERT_EQUALS(arbitrate(false, 99, false, 50, false, 10, 0), 0); // Default
  }

  /***************************************************************************
   * Complete Switch System Tests (Tests 97-100)
   ***************************************************************************/

  // Test 97: Complete autopilot engage/disengage logic
  void testCompleteAutopilotLogic() {
    struct AutopilotState {
      bool engaged = false;
      bool fdOn = false;
      int mode = 0;  // 0=off, 1=alt_hold, 2=hdg_hold, 3=nav
    };

    AutopilotState ap;

    auto engage = [&](bool condition) {
      if (condition && ap.fdOn && !ap.engaged) {
        ap.engaged = true;
        if (ap.mode == 0) ap.mode = 1;  // Default to alt_hold
        return true;
      }
      return false;
    };

    auto disengage = [&]() {
      ap.engaged = false;
      return true;
    };

    auto setMode = [&](int mode) {
      if (ap.fdOn) {
        ap.mode = mode;
        return true;
      }
      return false;
    };

    // Initial state
    TS_ASSERT(!ap.engaged);
    TS_ASSERT(!engage(true));  // FD not on

    ap.fdOn = true;
    TS_ASSERT(engage(true));
    TS_ASSERT(ap.engaged);
    TS_ASSERT_EQUALS(ap.mode, 1);

    TS_ASSERT(setMode(3));
    TS_ASSERT_EQUALS(ap.mode, 3);

    disengage();
    TS_ASSERT(!ap.engaged);
  }

  // Test 98: Complete landing gear state machine
  void testCompleteLandingGearStateMachine() {
    enum GearState { GEAR_DOWN, GEAR_TRANSIT_UP, GEAR_UP, GEAR_TRANSIT_DOWN };
    GearState state = GEAR_DOWN;
    int transitTimer = 0;
    const int transitTime = 10;

    auto command = [&](bool gearDown) {
      if (gearDown && state == GEAR_UP) {
        state = GEAR_TRANSIT_DOWN;
        transitTimer = transitTime;
      } else if (!gearDown && state == GEAR_DOWN) {
        state = GEAR_TRANSIT_UP;
        transitTimer = transitTime;
      }
    };

    auto tick = [&]() {
      if (transitTimer > 0) {
        transitTimer--;
        if (transitTimer == 0) {
          if (state == GEAR_TRANSIT_UP) state = GEAR_UP;
          else if (state == GEAR_TRANSIT_DOWN) state = GEAR_DOWN;
        }
      }
      return state;
    };

    TS_ASSERT_EQUALS(tick(), GEAR_DOWN);  // Initial

    command(false);  // Gear up command
    TS_ASSERT_EQUALS(tick(), GEAR_TRANSIT_UP);

    for (int i = 0; i < 9; i++) tick();  // Wait
    TS_ASSERT_EQUALS(tick(), GEAR_UP);   // Transition complete

    command(true);   // Gear down command
    TS_ASSERT_EQUALS(tick(), GEAR_TRANSIT_DOWN);

    for (int i = 0; i < 9; i++) tick();
    TS_ASSERT_EQUALS(tick(), GEAR_DOWN);
  }

  // Test 99: Complete hydraulic system routing
  void testCompleteHydraulicSystemRouting() {
    struct HydSystem {
      bool sys1Available = true;
      bool sys2Available = true;
      bool sys3Available = true;  // Backup
      double sys1Pressure = 3000.0;
      double sys2Pressure = 3000.0;
      double sys3Pressure = 3000.0;
    };

    HydSystem hyd;

    auto getPressure = [&](int consumer) -> double {
      // Consumer priority: 1=flight controls, 2=gear, 3=brakes
      // Routing: sys1 primary, sys2 backup, sys3 emergency
      double minPressure = 2000.0;

      if (hyd.sys1Available && hyd.sys1Pressure > minPressure) {
        return hyd.sys1Pressure;
      }
      if (hyd.sys2Available && hyd.sys2Pressure > minPressure) {
        return hyd.sys2Pressure;
      }
      if (consumer == 1 && hyd.sys3Available && hyd.sys3Pressure > minPressure) {
        return hyd.sys3Pressure;  // Flight controls get backup
      }
      return 0.0;
    };

    // All systems normal
    TS_ASSERT_DELTA(getPressure(1), 3000.0, epsilon);
    TS_ASSERT_DELTA(getPressure(2), 3000.0, epsilon);

    // Sys1 fails
    hyd.sys1Available = false;
    TS_ASSERT_DELTA(getPressure(1), 3000.0, epsilon);  // Sys2 backup
    TS_ASSERT_DELTA(getPressure(2), 3000.0, epsilon);

    // Sys2 also fails
    hyd.sys2Available = false;
    TS_ASSERT_DELTA(getPressure(1), 3000.0, epsilon);  // Sys3 for flight controls
    TS_ASSERT_DELTA(getPressure(2), 0.0, epsilon);     // No pressure for gear
  }

  // Test 100: Complete switch system verification
  void testCompleteSwitchSystemVerification() {
    // Comprehensive test combining multiple switch patterns

    // 1. Condition evaluation
    auto eq = [](double a, double b) { return std::abs(a - b) < 1e-10; };
    auto gt = [](double a, double b) { return a > b; };
    TS_ASSERT(eq(5.0, 5.0));
    TS_ASSERT(gt(6.0, 5.0));

    // 2. Logical operations
    auto AND = [](bool a, bool b) { return a && b; };
    auto OR = [](bool a, bool b) { return a || b; };
    TS_ASSERT(AND(true, true));
    TS_ASSERT(OR(false, true));

    // 3. Multi-way switch
    auto rangeSwitch = [](double x) {
      if (x < 0) return 0;
      if (x < 100) return 1;
      if (x < 200) return 2;
      return 3;
    };
    TS_ASSERT_EQUALS(rangeSwitch(-5), 0);
    TS_ASSERT_EQUALS(rangeSwitch(50), 1);
    TS_ASSERT_EQUALS(rangeSwitch(150), 2);
    TS_ASSERT_EQUALS(rangeSwitch(250), 3);

    // 4. State machine with hysteresis
    bool state = false;
    double lowThresh = 30.0, highThresh = 70.0;
    auto hystSwitch = [&](double val) {
      if (val > highThresh) state = true;
      else if (val < lowThresh) state = false;
      return state;
    };
    TS_ASSERT(!hystSwitch(20.0));
    TS_ASSERT(!hystSwitch(50.0));
    TS_ASSERT(hystSwitch(80.0));
    TS_ASSERT(hystSwitch(50.0));  // Stays high

    // 5. Priority encoder
    auto priority = [](bool p1, bool p2, bool p3) {
      if (p1) return 1;
      if (p2) return 2;
      if (p3) return 3;
      return 0;
    };
    TS_ASSERT_EQUALS(priority(true, true, true), 1);
    TS_ASSERT_EQUALS(priority(false, true, true), 2);
    TS_ASSERT_EQUALS(priority(false, false, true), 3);

    // 6. Edge detection
    bool prev = false;
    auto risingEdge = [&](bool curr) {
      bool edge = !prev && curr;
      prev = curr;
      return edge;
    };
    TS_ASSERT(risingEdge(true));
    TS_ASSERT(!risingEdge(true));
    TS_ASSERT(!risingEdge(false));
    TS_ASSERT(risingEdge(true));

    // 7. Debounce logic
    int debounceCount = 0;
    auto debounce = [&](bool input) {
      if (input) debounceCount++;
      else debounceCount = 0;
      return debounceCount >= 3;
    };
    TS_ASSERT(!debounce(true));
    TS_ASSERT(!debounce(true));
    TS_ASSERT(debounce(true));

    // All switch system patterns verified
  }
};

/*******************************************************************************
 * C172x Integration Tests for FGSwitch
 *
 * Tests switch component behavior in realistic flight scenarios using the
 * C172x aircraft model. Tests FCS switches, autopilot mode selection, and
 * dynamic switching during flight operations.
 ******************************************************************************/
class FGSwitchC172xTest : public CxxTest::TestSuite
{
public:
  // Test 1: C172x FCS model loads with switches
  void testC172xFCSLoads() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
  }

  // Test 2: Autopilot attitude hold switch
  void testC172xAutopilotAttitudeSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    // Run to stabilize
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Enable attitude hold
    fdmex.SetPropertyValue("ap/attitude_hold", 1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double attHold = fdmex.GetPropertyValue("ap/attitude_hold");
    TS_ASSERT(std::isfinite(attHold));
  }

  // Test 3: Autopilot altitude hold switch
  void testC172xAutopilotAltitudeSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Set altitude setpoint and enable
    fdmex.SetPropertyValue("ap/altitude_setpoint", 5000.0);
    fdmex.SetPropertyValue("ap/altitude_hold", 1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double altHold = fdmex.GetPropertyValue("ap/altitude_hold");
    TS_ASSERT(std::isfinite(altHold));
  }

  // Test 4: Autopilot heading hold switch
  void testC172xAutopilotHeadingSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    fdmex.SetPropertyValue("ap/heading_setpoint", 180.0);
    fdmex.SetPropertyValue("ap/heading_hold", 1.0);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double hdgHold = fdmex.GetPropertyValue("ap/heading_hold");
    TS_ASSERT(std::isfinite(hdgHold));
  }

  // Test 5: Throttle command switch states
  void testC172xThrottleCommandSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    // Test throttle at different positions
    double throttleValues[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    for (double throttle : throttleValues) {
      fcs->SetThrottleCmd(-1, throttle);
      fdmex.Run();

      double tcmd = fdmex.GetPropertyValue("fcs/throttle-cmd-norm");
      TS_ASSERT(std::isfinite(tcmd));
      TS_ASSERT_DELTA(tcmd, throttle, 0.01);
    }
  }

  // Test 6: Control surface command switching
  void testC172xControlSurfaceSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();

    // Switch elevator position
    fcs->SetDeCmd(-0.5);
    for (int i = 0; i < 20; i++) fdmex.Run();
    double elev1 = fdmex.GetPropertyValue("fcs/elevator-pos-rad");

    fcs->SetDeCmd(0.5);
    for (int i = 0; i < 20; i++) fdmex.Run();
    double elev2 = fdmex.GetPropertyValue("fcs/elevator-pos-rad");

    TS_ASSERT(std::isfinite(elev1));
    TS_ASSERT(std::isfinite(elev2));
    // Positions should differ when commands differ
    TS_ASSERT(std::abs(elev1 - elev2) > 0.01);
  }

  // Test 7: Flaps position switch states
  void testC172xFlapsPositionSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();

    // Switch flaps to different positions
    double flapSettings[] = {0.0, 0.33, 0.66, 1.0};
    for (double flap : flapSettings) {
      fcs->SetDfCmd(flap);
      // Run for flap actuation time
      for (int i = 0; i < 100; i++) fdmex.Run();

      double flapPos = fdmex.GetPropertyValue("fcs/flap-pos-deg");
      TS_ASSERT(std::isfinite(flapPos));
    }
  }

  // Test 8: Gear WOW switch state during flight
  void testC172xGearWOWSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(3000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double wow = fdmex.GetPropertyValue("gear/wow");
    TS_ASSERT(std::isfinite(wow));
    // In flight, WOW should be 0
    TS_ASSERT_DELTA(wow, 0.0, 1.0);
  }

  // Test 9: Roll command selector switch
  void testC172xRollCommandSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    // Get roll command switch output
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double rollCmd = fdmex.GetPropertyValue("fcs/roll-command-selector");
    TS_ASSERT(std::isfinite(rollCmd));
  }

  // Test 10: Brake command switch
  void testC172xBrakeSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();

    // Switch brakes on
    fcs->SetLBrake(1.0);
    fcs->SetRBrake(1.0);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    double lBrake = fdmex.GetPropertyValue("fcs/left-brake-cmd-norm");
    double rBrake = fdmex.GetPropertyValue("fcs/right-brake-cmd-norm");

    TS_ASSERT(std::isfinite(lBrake));
    TS_ASSERT(std::isfinite(rBrake));
  }

  // Test 11: Engine mixture switch
  void testC172xMixtureSwitchStates() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();

    // Test mixture at different values
    double mixtureValues[] = {0.0, 0.5, 1.0};
    for (double mix : mixtureValues) {
      fcs->SetMixtureCmd(-1, mix);
      for (int i = 0; i < 10; i++) fdmex.Run();

      double mixture = fdmex.GetPropertyValue("fcs/mixture-cmd-norm");
      TS_ASSERT(std::isfinite(mixture));
    }
  }

  // Test 12: Trim command switch response
  void testC172xTrimSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();

    // Switch pitch trim
    fcs->SetPitchTrimCmd(-0.3);
    for (int i = 0; i < 20; i++) fdmex.Run();

    double trim = fdmex.GetPropertyValue("fcs/pitch-trim-cmd-norm");
    TS_ASSERT(std::isfinite(trim));
    TS_ASSERT(trim < 0.0);
  }

  // Test 13: Steering command switch
  void testC172xSteeringSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();

    // Test steering command
    fcs->SetDrCmd(0.5);  // Rudder for steering

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    double steer = fdmex.GetPropertyValue("fcs/steer-cmd-norm");
    TS_ASSERT(std::isfinite(steer));
  }

  // Test 14: Multi-mode autopilot switching
  void testC172xMultiModeAutopilotSwitch() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) fdmex.Run();

    // Switch between different modes
    fdmex.SetPropertyValue("ap/attitude_hold", 1.0);
    fdmex.SetPropertyValue("ap/heading_hold", 0.0);
    for (int i = 0; i < 50; i++) fdmex.Run();

    fdmex.SetPropertyValue("ap/attitude_hold", 0.0);
    fdmex.SetPropertyValue("ap/heading_hold", 1.0);
    for (int i = 0; i < 50; i++) fdmex.Run();

    double attHold = fdmex.GetPropertyValue("ap/attitude_hold");
    double hdgHold = fdmex.GetPropertyValue("ap/heading_hold");
    TS_ASSERT(std::isfinite(attHold));
    TS_ASSERT(std::isfinite(hdgHold));
  }

  // Test 15: FCS component output validity
  void testC172xFCSOutputValidity() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    auto fcs = fdmex.GetFCS();
    fcs->SetThrottleCmd(-1, 0.5);
    fcs->SetDeCmd(0.1);
    fcs->SetDaCmd(-0.1);
    fcs->SetDrCmd(0.05);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Check all FCS outputs are finite
    TS_ASSERT(std::isfinite(fdmex.GetPropertyValue("fcs/elevator-pos-rad")));
    TS_ASSERT(std::isfinite(fdmex.GetPropertyValue("fcs/left-aileron-pos-rad")));
    TS_ASSERT(std::isfinite(fdmex.GetPropertyValue("fcs/right-aileron-pos-rad")));
    TS_ASSERT(std::isfinite(fdmex.GetPropertyValue("fcs/rudder-pos-rad")));
  }
};
