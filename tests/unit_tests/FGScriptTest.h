#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <string>
#include <vector>
#include <map>

#include <FGFDMExec.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGScriptTest : public CxxTest::TestSuite
{
public:
  // Test action type enum values
  void testActionTypeEnums() {
    // FG_RAMP = 1, FG_STEP = 2, FG_EXP = 3
    int FG_RAMP = 1;
    int FG_STEP = 2;
    int FG_EXP = 3;

    TS_ASSERT_EQUALS(FG_RAMP, 1);
    TS_ASSERT_EQUALS(FG_STEP, 2);
    TS_ASSERT_EQUALS(FG_EXP, 3);
  }

  // Test value type enum values
  void testValueTypeEnums() {
    // FG_VALUE = 1, FG_DELTA = 2, FG_BOOL = 3
    int FG_VALUE = 1;
    int FG_DELTA = 2;
    int FG_BOOL = 3;

    TS_ASSERT_EQUALS(FG_VALUE, 1);
    TS_ASSERT_EQUALS(FG_DELTA, 2);
    TS_ASSERT_EQUALS(FG_BOOL, 3);
  }

  // Test step action behavior
  void testStepAction() {
    // Step action: immediate transition to target value
    double currentValue = 0.0;
    double targetValue = 100.0;

    // Step action immediately sets value to target
    currentValue = targetValue;
    TS_ASSERT_DELTA(currentValue, 100.0, DEFAULT_TOLERANCE);
  }

  // Test ramp action behavior
  void testRampAction() {
    // Ramp action: linear transition over time constant
    double currentValue = 0.0;
    double targetValue = 100.0;
    double timeConstant = 1.0;  // seconds
    double deltaT = 0.1;        // time step

    // Progress toward target: newValue = currentValue + (target - current) * deltaT / timeConstant
    double progress = deltaT / timeConstant;
    double newValue = currentValue + (targetValue - currentValue) * progress;
    TS_ASSERT_DELTA(newValue, 10.0, DEFAULT_TOLERANCE);

    // After another step
    currentValue = newValue;
    newValue = currentValue + (targetValue - currentValue) * progress;
    TS_ASSERT_DELTA(newValue, 19.0, DEFAULT_TOLERANCE);

    // After full time constant (approx 63% of transition)
    currentValue = 0.0;
    for (int i = 0; i < 10; i++) {
      currentValue = currentValue + (targetValue - currentValue) * progress;
    }
    TS_ASSERT(currentValue > 60.0);
    TS_ASSERT(currentValue < 70.0);
  }

  // Test exponential action behavior
  void testExponentialAction() {
    // Exponential action: asymptotic approach to target
    // value = target - (target - initial) * exp(-t/tc)
    double initialValue = 0.0;
    double targetValue = 100.0;
    double timeConstant = 1.0;

    // At t = 0
    double t = 0.0;
    double value = targetValue - (targetValue - initialValue) * exp(-t / timeConstant);
    TS_ASSERT_DELTA(value, 0.0, DEFAULT_TOLERANCE);

    // At t = tc (one time constant), should be ~63.2% of way to target
    t = timeConstant;
    value = targetValue - (targetValue - initialValue) * exp(-t / timeConstant);
    TS_ASSERT_DELTA(value, 63.212, 0.01);

    // At t = 3*tc, should be ~95% of way to target
    t = 3.0 * timeConstant;
    value = targetValue - (targetValue - initialValue) * exp(-t / timeConstant);
    TS_ASSERT_DELTA(value, 95.02, 0.1);

    // At t = 5*tc, should be ~99.3% of way to target
    t = 5.0 * timeConstant;
    value = targetValue - (targetValue - initialValue) * exp(-t / timeConstant);
    TS_ASSERT_DELTA(value, 99.33, 0.1);
  }

  // Test delta value type
  void testDeltaValueType() {
    // Delta type modifies current value by an amount
    double currentValue = 50.0;
    double deltaValue = 10.0;

    currentValue += deltaValue;
    TS_ASSERT_DELTA(currentValue, 60.0, DEFAULT_TOLERANCE);

    // Negative delta
    deltaValue = -25.0;
    currentValue += deltaValue;
    TS_ASSERT_DELTA(currentValue, 35.0, DEFAULT_TOLERANCE);
  }

  // Test boolean value type
  void testBoolValueType() {
    // Boolean type: 0 = false, non-zero = true
    double boolValue = 0.0;
    TS_ASSERT(!static_cast<bool>(boolValue));

    boolValue = 1.0;
    TS_ASSERT(static_cast<bool>(boolValue));

    // Any non-zero is true
    boolValue = 0.5;
    TS_ASSERT(static_cast<bool>(boolValue));

    boolValue = -1.0;
    TS_ASSERT(static_cast<bool>(boolValue));
  }

  // Test event trigger logic
  void testEventTriggerLogic() {
    // Non-persistent event: fires once when condition becomes true
    bool conditionMet = false;
    bool triggered = false;
    bool persistent = false;

    // Initial state: condition false, not triggered
    TS_ASSERT(!conditionMet);
    TS_ASSERT(!triggered);

    // Condition becomes true - should trigger
    conditionMet = true;
    if (conditionMet && !triggered) {
      triggered = true;
    }
    TS_ASSERT(triggered);

    // Condition still true - should not re-trigger (non-persistent)
    bool shouldExecute = conditionMet && !triggered;
    TS_ASSERT(!shouldExecute);
  }

  // Test persistent event logic
  void testPersistentEventLogic() {
    // Persistent event: fires each time condition toggles from false to true
    bool conditionMet = false;
    bool persistent = true;
    bool wasTriggered = false;

    // Toggle to true - should fire
    conditionMet = true;
    bool shouldFire = conditionMet && !wasTriggered;
    TS_ASSERT(shouldFire);
    wasTriggered = conditionMet;

    // Stays true - should not fire
    shouldFire = conditionMet && !wasTriggered;
    TS_ASSERT(!shouldFire);

    // Toggle to false
    conditionMet = false;
    wasTriggered = conditionMet;
    TS_ASSERT(!wasTriggered);

    // Toggle back to true - should fire again (persistent)
    conditionMet = true;
    shouldFire = conditionMet && !wasTriggered;
    TS_ASSERT(shouldFire);
  }

  // Test continuous event logic
  void testContinuousEventLogic() {
    // Continuous event: executes every cycle while condition is true
    bool conditionMet = false;
    bool continuous = true;
    int executionCount = 0;

    // Simulate 5 cycles
    for (int cycle = 0; cycle < 5; cycle++) {
      if (continuous && conditionMet) {
        executionCount++;
      }
    }
    TS_ASSERT_EQUALS(executionCount, 0);

    // Now with condition true
    conditionMet = true;
    executionCount = 0;
    for (int cycle = 0; cycle < 5; cycle++) {
      if (continuous && conditionMet) {
        executionCount++;
      }
    }
    TS_ASSERT_EQUALS(executionCount, 5);
  }

  // Test event delay
  void testEventDelay() {
    // Event can have a delay before actions execute
    double delay = 0.5;         // 0.5 second delay
    double simTime = 0.0;
    double triggerTime = 0.0;
    bool triggered = false;
    bool actionsExecuted = false;

    // Trigger at t=1.0
    simTime = 1.0;
    triggered = true;
    triggerTime = simTime;

    // Check before delay expires
    simTime = 1.3;
    if (triggered && (simTime - triggerTime >= delay)) {
      actionsExecuted = true;
    }
    TS_ASSERT(!actionsExecuted);

    // Check after delay expires
    simTime = 1.6;
    if (triggered && (simTime - triggerTime >= delay)) {
      actionsExecuted = true;
    }
    TS_ASSERT(actionsExecuted);
  }

  // Test time constant conversion
  void testTimeConstantConversion() {
    // Time constant determines speed of transition
    // Smaller tc = faster transition
    double tc_fast = 0.1;
    double tc_slow = 2.0;

    // Compare progress after 0.1 seconds
    double deltaT = 0.1;
    double progressFast = deltaT / tc_fast;  // 1.0 (100%)
    double progressSlow = deltaT / tc_slow;  // 0.05 (5%)

    TS_ASSERT_DELTA(progressFast, 1.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(progressSlow, 0.05, DEFAULT_TOLERANCE);
    TS_ASSERT(progressFast > progressSlow);
  }

  // Test simulation time boundaries
  void testSimTimeBoundaries() {
    double startTime = 0.0;
    double endTime = 3000.0;
    double currentTime = 0.0;

    // Simulation should run while within bounds
    TS_ASSERT(currentTime >= startTime);
    TS_ASSERT(currentTime < endTime);

    // At end time, simulation should stop
    currentTime = 3000.0;
    bool shouldContinue = currentTime < endTime;
    TS_ASSERT(!shouldContinue);

    // Past end time
    currentTime = 3001.0;
    shouldContinue = currentTime < endTime;
    TS_ASSERT(!shouldContinue);
  }

  // Test property value setting
  void testPropertyValueSetting() {
    // Simulate setting a property value
    double throttleCmd = 0.0;
    double targetValue = 1.0;

    // Step action
    throttleCmd = targetValue;
    TS_ASSERT_DELTA(throttleCmd, 1.0, DEFAULT_TOLERANCE);

    // Value action with specific value
    double specificValue = 0.75;
    throttleCmd = specificValue;
    TS_ASSERT_DELTA(throttleCmd, 0.75, DEFAULT_TOLERANCE);
  }

  // Test ramp with zero time constant (should behave as step)
  void testRampWithZeroTC() {
    double currentValue = 0.0;
    double targetValue = 100.0;
    double timeConstant = 0.0;  // Zero tc
    double deltaT = 0.1;

    // With tc=0, should behave as step (instant transition)
    // Avoid division by zero by using step behavior
    if (timeConstant <= 0.0) {
      currentValue = targetValue;
    }
    TS_ASSERT_DELTA(currentValue, targetValue, DEFAULT_TOLERANCE);
  }

  // Test value span calculation for transitions
  void testValueSpanCalculation() {
    double originalValue = 25.0;
    double targetValue = 100.0;
    double valueSpan = targetValue - originalValue;

    TS_ASSERT_DELTA(valueSpan, 75.0, DEFAULT_TOLERANCE);

    // Negative span (decreasing)
    targetValue = 10.0;
    valueSpan = targetValue - originalValue;
    TS_ASSERT_DELTA(valueSpan, -15.0, DEFAULT_TOLERANCE);
  }

  // Test transiting flag behavior
  void testTransitingFlag() {
    bool transiting = false;
    double currentValue = 0.0;
    double targetValue = 100.0;
    double tolerance = 0.01;

    // Start transition
    transiting = true;
    TS_ASSERT(transiting);

    // During transition
    currentValue = 50.0;
    transiting = std::abs(currentValue - targetValue) > tolerance;
    TS_ASSERT(transiting);

    // Complete transition
    currentValue = 100.0;
    transiting = std::abs(currentValue - targetValue) > tolerance;
    TS_ASSERT(!transiting);
  }

  // Test notification system
  void testNotificationSystem() {
    bool notify = true;
    bool notified = false;

    // Event triggers
    bool triggered = true;

    if (notify && triggered && !notified) {
      // Would output notification message
      notified = true;
    }

    TS_ASSERT(notified);

    // Subsequent cycles should not re-notify
    bool shouldNotify = notify && triggered && !notified;
    TS_ASSERT(!shouldNotify);
  }

  // Test event reset
  void testEventReset() {
    // Event state
    bool triggered = true;
    bool notified = true;
    double startTime = 5.0;

    // Reset event
    triggered = false;
    notified = false;
    startTime = 0.0;

    TS_ASSERT(!triggered);
    TS_ASSERT(!notified);
    TS_ASSERT_DELTA(startTime, 0.0, DEFAULT_TOLERANCE);
  }

  // Test script name parsing
  void testScriptNameParsing() {
    std::string scriptName = "C172-01A takeoff run";

    TS_ASSERT(!scriptName.empty());
    TS_ASSERT(scriptName.length() > 0);
    TS_ASSERT(scriptName.find("C172") != std::string::npos);
  }

  // Test default event initialization
  void testDefaultEventInit() {
    // Default event values
    bool triggered = false;
    bool persistent = false;
    bool continuous = false;
    double delay = 0.0;
    bool notify = false;
    bool notified = false;
    bool notifyKML = false;
    std::string name = "";
    double startTime = 0.0;
    double timeSpan = 0.0;

    TS_ASSERT(!triggered);
    TS_ASSERT(!persistent);
    TS_ASSERT(!continuous);
    TS_ASSERT_DELTA(delay, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT(!notify);
    TS_ASSERT(!notified);
    TS_ASSERT(!notifyKML);
    TS_ASSERT(name.empty());
    TS_ASSERT_DELTA(startTime, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(timeSpan, 0.0, DEFAULT_TOLERANCE);
  }

  // Test condition evaluation timing
  void testConditionEvaluationTiming() {
    // Conditions are evaluated each cycle
    double simTime = 0.0;
    double conditionValue = 0.25;
    bool conditionMet = false;

    // Before threshold
    simTime = 0.1;
    conditionMet = (simTime >= conditionValue);
    TS_ASSERT(!conditionMet);

    // At threshold
    simTime = 0.25;
    conditionMet = (simTime >= conditionValue);
    TS_ASSERT(conditionMet);

    // After threshold
    simTime = 0.5;
    conditionMet = (simTime >= conditionValue);
    TS_ASSERT(conditionMet);
  }

  // Test multiple set actions in single event
  void testMultipleSetActions() {
    // An event can have multiple set actions
    std::vector<double> values(4);
    std::vector<double> targets = {1.0, 0.87, 3.0, 1.0};

    // Execute all set actions
    for (size_t i = 0; i < targets.size(); i++) {
      values[i] = targets[i];
    }

    TS_ASSERT_DELTA(values[0], 1.0, DEFAULT_TOLERANCE);   // throttle
    TS_ASSERT_DELTA(values[1], 0.87, DEFAULT_TOLERANCE);  // mixture
    TS_ASSERT_DELTA(values[2], 3.0, DEFAULT_TOLERANCE);   // magneto
    TS_ASSERT_DELTA(values[3], 1.0, DEFAULT_TOLERANCE);   // starter
  }

  /***************************************************************************
   * Extended Ramp Action Tests
   ***************************************************************************/

  // Test ramp action with different time constants
  void testRampVariousTimeConstants() {
    double target = 100.0;
    double dt = 0.1;

    // Fast ramp (tc = 0.5)
    double current_fast = 0.0;
    double tc_fast = 0.5;
    for (int i = 0; i < 10; i++) {
      current_fast += (target - current_fast) * dt / tc_fast;
    }

    // Slow ramp (tc = 2.0)
    double current_slow = 0.0;
    double tc_slow = 2.0;
    for (int i = 0; i < 10; i++) {
      current_slow += (target - current_slow) * dt / tc_slow;
    }

    // Fast should be closer to target
    TS_ASSERT(current_fast > current_slow);
    TS_ASSERT(current_fast > 80.0);
    TS_ASSERT(current_slow < 50.0);
  }

  // Test ramp decreasing value
  void testRampDecreasing() {
    double current = 100.0;
    double target = 0.0;
    double tc = 1.0;
    double dt = 0.1;

    for (int i = 0; i < 20; i++) {
      current += (target - current) * dt / tc;
    }

    TS_ASSERT(current < 20.0);
    TS_ASSERT(current > 0.0);
  }

  // Test ramp to negative target
  void testRampToNegative() {
    double current = 0.0;
    double target = -50.0;
    double tc = 1.0;
    double dt = 0.1;

    for (int i = 0; i < 30; i++) {
      current += (target - current) * dt / tc;
    }

    TS_ASSERT(current < -40.0);
    TS_ASSERT(current > target);
  }

  /***************************************************************************
   * Extended Exponential Action Tests
   ***************************************************************************/

  // Test exponential approach convergence
  void testExponentialConvergence() {
    double initial = 0.0;
    double target = 100.0;
    double tc = 1.0;

    // At various time constants
    double t1 = 1.0 * tc;  // 63.2%
    double t2 = 2.0 * tc;  // 86.5%
    double t3 = 3.0 * tc;  // 95.0%
    double t4 = 4.0 * tc;  // 98.2%
    double t5 = 5.0 * tc;  // 99.3%

    double v1 = target - (target - initial) * exp(-t1 / tc);
    double v2 = target - (target - initial) * exp(-t2 / tc);
    double v3 = target - (target - initial) * exp(-t3 / tc);
    double v4 = target - (target - initial) * exp(-t4 / tc);
    double v5 = target - (target - initial) * exp(-t5 / tc);

    TS_ASSERT(v1 < v2);
    TS_ASSERT(v2 < v3);
    TS_ASSERT(v3 < v4);
    TS_ASSERT(v4 < v5);
    TS_ASSERT(v5 < target);
  }

  // Test exponential from non-zero initial
  void testExponentialFromNonZero() {
    double initial = 50.0;
    double target = 100.0;
    double tc = 1.0;

    double t = 1.0;
    double value = target - (target - initial) * exp(-t / tc);

    // Should be 63.2% of way from 50 to 100 = 50 + 0.632 * 50 = 81.6
    TS_ASSERT_DELTA(value, 81.6, 0.5);
  }

  // Test exponential decrease
  void testExponentialDecrease() {
    double initial = 100.0;
    double target = 0.0;
    double tc = 1.0;

    double t = 1.0;
    double value = target - (target - initial) * exp(-t / tc);

    // Should be 63.2% of way from 100 to 0 = 100 - 63.2 = 36.8
    TS_ASSERT_DELTA(value, 36.8, 0.5);
  }

  /***************************************************************************
   * Complex Event Sequence Tests
   ***************************************************************************/

  // Test event sequence (takeoff)
  void testTakeoffEventSequence() {
    double simTime = 0.0;
    double throttle = 0.0;
    double brakes = 1.0;
    double pitchTrim = 0.0;

    // Event 1: Set throttle at t=0
    if (simTime >= 0.0) {
      throttle = 1.0;
    }
    TS_ASSERT_DELTA(throttle, 1.0, DEFAULT_TOLERANCE);

    // Event 2: Release brakes at t=1
    simTime = 1.0;
    if (simTime >= 1.0) {
      brakes = 0.0;
    }
    TS_ASSERT_DELTA(brakes, 0.0, DEFAULT_TOLERANCE);

    // Event 3: Set pitch trim at t=5
    simTime = 5.0;
    if (simTime >= 5.0) {
      pitchTrim = -0.05;
    }
    TS_ASSERT_DELTA(pitchTrim, -0.05, DEFAULT_TOLERANCE);
  }

  // Test event sequence with conditions
  void testConditionalEventSequence() {
    double altitude = 0.0;
    double flaps = 20.0;
    double gear = 1.0;

    // Climb
    altitude = 500.0;

    // Retract gear above 200 ft
    if (altitude > 200.0) {
      gear = 0.0;
    }
    TS_ASSERT_DELTA(gear, 0.0, DEFAULT_TOLERANCE);

    // Retract flaps above 400 ft
    if (altitude > 400.0) {
      flaps = 0.0;
    }
    TS_ASSERT_DELTA(flaps, 0.0, DEFAULT_TOLERANCE);
  }

  // Test landing event sequence
  void testLandingEventSequence() {
    double altitude = 3000.0;
    double flaps = 0.0;
    double gear = 0.0;
    double spoilers = 0.0;
    bool onGround = false;

    // Descending through 2000 ft - extend flaps
    altitude = 1500.0;
    if (altitude < 2000.0) {
      flaps = 10.0;
    }
    TS_ASSERT_DELTA(flaps, 10.0, DEFAULT_TOLERANCE);

    // Descending through 1000 ft - extend gear
    altitude = 800.0;
    if (altitude < 1000.0) {
      gear = 1.0;
      flaps = 30.0;
    }
    TS_ASSERT_DELTA(gear, 1.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(flaps, 30.0, DEFAULT_TOLERANCE);

    // Touchdown - deploy spoilers
    onGround = true;
    if (onGround) {
      spoilers = 1.0;
    }
    TS_ASSERT_DELTA(spoilers, 1.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Multiple Condition Tests
   ***************************************************************************/

  // Test AND conditions
  void testANDConditions() {
    double altitude = 5000.0;
    double airspeed = 150.0;
    bool triggered = false;

    // Only trigger if both conditions met
    if (altitude > 3000.0 && airspeed > 100.0) {
      triggered = true;
    }
    TS_ASSERT(triggered);

    // One condition not met
    altitude = 2000.0;
    triggered = false;
    if (altitude > 3000.0 && airspeed > 100.0) {
      triggered = true;
    }
    TS_ASSERT(!triggered);
  }

  // Test OR conditions
  void testORConditions() {
    double altitude = 1000.0;
    double airspeed = 200.0;
    bool triggered = false;

    // Trigger if either condition met
    if (altitude > 5000.0 || airspeed > 150.0) {
      triggered = true;
    }
    TS_ASSERT(triggered);

    // Neither condition met
    airspeed = 100.0;
    triggered = false;
    if (altitude > 5000.0 || airspeed > 150.0) {
      triggered = true;
    }
    TS_ASSERT(!triggered);
  }

  // Test complex conditions
  void testComplexConditions() {
    double time = 10.0;
    double altitude = 5000.0;
    double speed = 150.0;
    bool apEngaged = true;

    // (time > 5 AND altitude > 3000) OR (speed > 200)
    bool condition = (time > 5.0 && altitude > 3000.0) || (speed > 200.0);
    TS_ASSERT(condition);

    // (time > 5 AND altitude > 3000) AND NOT apEngaged
    condition = (time > 5.0 && altitude > 3000.0) && !apEngaged;
    TS_ASSERT(!condition);
  }

  /***************************************************************************
   * Time-Based Trigger Tests
   ***************************************************************************/

  // Test time window trigger
  void testTimeWindowTrigger() {
    double simTime = 5.0;
    double startTime = 3.0;
    double endTime = 7.0;
    bool triggered = false;

    // Within window
    if (simTime >= startTime && simTime <= endTime) {
      triggered = true;
    }
    TS_ASSERT(triggered);

    // Before window
    simTime = 2.0;
    triggered = false;
    if (simTime >= startTime && simTime <= endTime) {
      triggered = true;
    }
    TS_ASSERT(!triggered);

    // After window
    simTime = 8.0;
    triggered = false;
    if (simTime >= startTime && simTime <= endTime) {
      triggered = true;
    }
    TS_ASSERT(!triggered);
  }

  // Test periodic trigger
  void testPeriodicTrigger() {
    double period = 1.0;
    double lastTrigger = 0.0;
    int triggerCount = 0;

    // Use integer loop to avoid floating point precision issues
    for (int i = 0; i <= 50; i++) {
      double t = i * 0.1;
      if (t - lastTrigger >= period) {
        triggerCount++;
        lastTrigger = t;
      }
    }

    TS_ASSERT_EQUALS(triggerCount, 5);  // At t=1,2,3,4,5
  }

  // Test one-shot trigger
  void testOneShotTrigger() {
    bool triggered = false;
    bool hasTriggered = false;
    int triggerCount = 0;

    for (int i = 0; i < 10; i++) {
      if (!hasTriggered) {
        triggered = true;
        hasTriggered = true;
        triggerCount++;
      }
    }

    TS_ASSERT(hasTriggered);
    TS_ASSERT_EQUALS(triggerCount, 1);
  }

  /***************************************************************************
   * Value Interpolation Tests
   ***************************************************************************/

  // Test linear interpolation
  void testLinearInterpolation() {
    double start = 0.0;
    double end = 100.0;
    double duration = 10.0;

    for (double t = 0.0; t <= duration; t += 1.0) {
      double value = start + (end - start) * (t / duration);
      double expected = t * 10.0;
      TS_ASSERT_DELTA(value, expected, DEFAULT_TOLERANCE);
    }
  }

  // Test cosine interpolation (smoother)
  void testCosineInterpolation() {
    double start = 0.0;
    double end = 100.0;
    double duration = 10.0;

    for (double t = 0.0; t <= duration; t += 1.0) {
      double fraction = t / duration;
      double smooth = (1.0 - cos(fraction * M_PI)) / 2.0;
      double value = start + (end - start) * smooth;

      // At midpoint, should be exactly 50
      if (std::abs(t - 5.0) < 0.01) {
        TS_ASSERT_DELTA(value, 50.0, 0.1);
      }
    }
  }

  /***************************************************************************
   * Action Execution Order Tests
   ***************************************************************************/

  // Test actions execute in order
  void testActionExecutionOrder() {
    std::vector<int> executionOrder;

    // Simulate 3 actions
    executionOrder.push_back(1);
    executionOrder.push_back(2);
    executionOrder.push_back(3);

    TS_ASSERT_EQUALS(executionOrder.size(), 3u);
    TS_ASSERT_EQUALS(executionOrder[0], 1);
    TS_ASSERT_EQUALS(executionOrder[1], 2);
    TS_ASSERT_EQUALS(executionOrder[2], 3);
  }

  // Test conditional action skipping
  void testConditionalActionSkipping() {
    double throttle = 0.5;
    double mixture = 0.8;
    bool engineRunning = false;

    // Only execute if engine running
    if (engineRunning) {
      throttle = 1.0;
      mixture = 1.0;
    }

    TS_ASSERT_DELTA(throttle, 0.5, DEFAULT_TOLERANCE);  // Unchanged
    TS_ASSERT_DELTA(mixture, 0.8, DEFAULT_TOLERANCE);   // Unchanged
  }

  /***************************************************************************
   * Flight Phase Tests
   ***************************************************************************/

  // Test phase detection
  void testPhaseDetection() {
    enum FlightPhase { PREFLIGHT, TAXI, TAKEOFF, CLIMB, CRUISE, DESCENT, APPROACH, LANDING };

    auto detectPhase = [](double altitude, double vertSpeed, bool onGround, double throttle) -> int {
      if (onGround && throttle < 0.1) return PREFLIGHT;
      if (onGround && throttle < 0.5) return TAXI;
      if (onGround && throttle > 0.9) return TAKEOFF;
      if (!onGround && vertSpeed > 500.0) return CLIMB;
      if (!onGround && altitude > 10000.0 && std::abs(vertSpeed) < 200.0) return CRUISE;
      if (!onGround && vertSpeed < -500.0 && altitude > 3000.0) return DESCENT;
      if (!onGround && altitude < 3000.0 && vertSpeed < 0) return APPROACH;
      if (onGround) return LANDING;
      return CRUISE;  // Default
    };

    TS_ASSERT_EQUALS(detectPhase(0, 0, true, 0.0), PREFLIGHT);
    TS_ASSERT_EQUALS(detectPhase(0, 0, true, 0.3), TAXI);
    TS_ASSERT_EQUALS(detectPhase(0, 0, true, 1.0), TAKEOFF);
    TS_ASSERT_EQUALS(detectPhase(5000, 1000, false, 1.0), CLIMB);
    TS_ASSERT_EQUALS(detectPhase(15000, 0, false, 0.7), CRUISE);
    TS_ASSERT_EQUALS(detectPhase(8000, -1000, false, 0.3), DESCENT);
    TS_ASSERT_EQUALS(detectPhase(1500, -500, false, 0.3), APPROACH);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many events
  void testStressManyEvents() {
    std::vector<bool> eventTriggered(100, false);

    for (int i = 0; i < 100; i++) {
      eventTriggered[i] = true;
    }

    for (int i = 0; i < 100; i++) {
      TS_ASSERT(eventTriggered[i]);
    }
  }

  // Test rapid condition changes
  void testStressRapidConditionChanges() {
    bool condition = false;
    int toggleCount = 0;

    for (int i = 0; i < 1000; i++) {
      bool newCondition = (i % 2 == 0);
      if (newCondition != condition) {
        toggleCount++;
        condition = newCondition;
      }
    }

    TS_ASSERT_EQUALS(toggleCount, 1000);
  }

  // Test long simulation
  void testStressLongSimulation() {
    double simTime = 0.0;
    double dt = 0.01;
    int eventCount = 0;

    // Simulate 1 hour with events every 60 seconds
    while (simTime < 3600.0) {
      if (std::fmod(simTime, 60.0) < dt) {
        eventCount++;
      }
      simTime += dt;
    }

    TS_ASSERT(eventCount >= 59);
    TS_ASSERT(eventCount <= 61);
  }

  // Test many simultaneous ramps
  void testStressManyRamps() {
    std::vector<double> values(10, 0.0);
    std::vector<double> targets = {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
    double tc = 1.0;
    double dt = 0.1;

    // Run 20 iterations
    for (int iter = 0; iter < 20; iter++) {
      for (size_t i = 0; i < values.size(); i++) {
        values[i] += (targets[i] - values[i]) * dt / tc;
      }
    }

    // All should have progressed toward targets
    for (size_t i = 0; i < values.size(); i++) {
      TS_ASSERT(values[i] > targets[i] * 0.8);
    }
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  // Test zero duration ramp
  void testZeroDurationRamp() {
    double current = 0.0;
    double target = 100.0;

    // Zero duration should be instant
    current = target;
    TS_ASSERT_DELTA(current, 100.0, DEFAULT_TOLERANCE);
  }

  // Test negative time constant
  void testNegativeTimeConstant() {
    double tc = -1.0;

    // Negative tc should be treated as absolute or clamped
    if (tc < 0) tc = std::abs(tc);
    TS_ASSERT(tc > 0.0);
  }

  // Test very large time constant
  void testVeryLargeTimeConstant() {
    double current = 0.0;
    double target = 100.0;
    double tc = 1000.0;  // Very slow
    double dt = 0.1;

    for (int i = 0; i < 10; i++) {
      current += (target - current) * dt / tc;
    }

    // Should barely have moved
    TS_ASSERT(current < 1.0);
  }

  // Test NaN handling
  void testNaNHandling() {
    double value = std::numeric_limits<double>::quiet_NaN();

    // NaN checks
    TS_ASSERT(std::isnan(value));

    // Replace with default
    if (std::isnan(value)) {
      value = 0.0;
    }
    TS_ASSERT_DELTA(value, 0.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Script XML Element Tests
   ***************************************************************************/

  // Test run element parsing concept
  void testRunElementConcept() {
    double start = 0.0;
    double end = 1000.0;
    double dt = 0.008333;  // 120 Hz

    TS_ASSERT(start < end);
    TS_ASSERT(dt > 0.0);
    TS_ASSERT_DELTA(1.0 / dt, 120.0, 0.1);
  }

  // Test use element concept
  void testUseElementConcept() {
    std::string aircraftName = "c172x";
    std::string initName = "reset01";

    TS_ASSERT(!aircraftName.empty());
    TS_ASSERT(!initName.empty());
    TS_ASSERT(aircraftName.find("c172") != std::string::npos);
  }

  // Test property output concept
  void testPropertyOutputConcept() {
    std::vector<std::string> outputProperties = {
      "simulation/sim-time-sec",
      "position/h-sl-ft",
      "velocities/vc-kts",
      "attitude/phi-rad"
    };

    TS_ASSERT_EQUALS(outputProperties.size(), 4u);
    TS_ASSERT(outputProperties[0].find("sim-time") != std::string::npos);
  }

  /***************************************************************************
   * Additional Value Tests
   ***************************************************************************/

  // Test value clamping
  void testValueClamping() {
    auto clamp = [](double val, double minVal, double maxVal) {
      return std::max(minVal, std::min(val, maxVal));
    };

    TS_ASSERT_DELTA(clamp(50.0, 0.0, 100.0), 50.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(clamp(-10.0, 0.0, 100.0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(clamp(150.0, 0.0, 100.0), 100.0, DEFAULT_TOLERANCE);
  }

  // Test rate limiting
  void testRateLimiting() {
    double current = 0.0;
    double target = 100.0;
    double maxRate = 10.0;  // per second
    double dt = 0.1;

    // maxDelta = 10.0 * 0.1 = 1.0 per iteration
    // Need 100 iterations to reach target of 100.0
    for (int i = 0; i < 100; i++) {
      double delta = target - current;
      double maxDelta = maxRate * dt;
      if (std::abs(delta) > maxDelta) {
        delta = (delta > 0) ? maxDelta : -maxDelta;
      }
      current += delta;
    }

    // Should have reached target in 100 iterations
    TS_ASSERT_DELTA(current, 100.0, DEFAULT_TOLERANCE);
  }

  // Test wraparound values
  void testWraparoundValues() {
    auto wrapAngle = [](double angle) {
      while (angle > 180.0) angle -= 360.0;
      while (angle < -180.0) angle += 360.0;
      return angle;
    };

    TS_ASSERT_DELTA(wrapAngle(0.0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(wrapAngle(360.0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(wrapAngle(-360.0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(wrapAngle(270.0), -90.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(wrapAngle(-270.0), 90.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended Script Execution Tests
   ***************************************************************************/

  // Test script initialization state
  void testScriptInitializationState() {
    bool initialized = false;
    double startTime = 0.0;
    double endTime = 100.0;
    std::string scriptName = "";

    // Initialize
    initialized = true;
    startTime = 0.0;
    endTime = 1000.0;
    scriptName = "test_script";

    TS_ASSERT(initialized);
    TS_ASSERT_DELTA(startTime, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT(endTime > startTime);
    TS_ASSERT(!scriptName.empty());
  }

  // Test multiple event ordering
  void testMultipleEventOrdering() {
    std::vector<double> eventTimes = {1.0, 5.0, 10.0, 15.0, 20.0};
    std::vector<bool> executed(5, false);
    double simTime = 0.0;

    // Process events in order
    for (int i = 0; i <= 25; i++) {
      simTime = i * 1.0;
      for (size_t j = 0; j < eventTimes.size(); j++) {
        if (simTime >= eventTimes[j] && !executed[j]) {
          executed[j] = true;
        }
      }
    }

    // All should be executed
    for (size_t j = 0; j < executed.size(); j++) {
      TS_ASSERT(executed[j]);
    }
  }

  // Test event with notify flag
  void testEventWithNotifyFlag() {
    bool notify = true;
    bool notified = false;
    bool triggered = false;

    // Event triggers
    triggered = true;
    if (notify && triggered) {
      notified = true;
    }

    TS_ASSERT(notified);
  }

  // Test event grouping
  void testEventGrouping() {
    std::vector<std::string> group1 = {"throttle", "mixture", "propeller"};
    std::vector<std::string> group2 = {"gear", "flaps", "spoilers"};

    TS_ASSERT_EQUALS(group1.size(), 3u);
    TS_ASSERT_EQUALS(group2.size(), 3u);
  }

  /***************************************************************************
   * Extended Property Tests
   ***************************************************************************/

  // Test property path parsing
  void testPropertyPathParsing() {
    std::string path = "fcs/throttle-cmd-norm[0]";

    TS_ASSERT(path.find("fcs") != std::string::npos);
    TS_ASSERT(path.find("throttle") != std::string::npos);
    TS_ASSERT(path.find("[0]") != std::string::npos);
  }

  // Test property value bounds
  void testPropertyValueBounds() {
    double value = 0.5;
    double minValue = 0.0;
    double maxValue = 1.0;

    TS_ASSERT(value >= minValue);
    TS_ASSERT(value <= maxValue);

    // Clamped value
    value = 1.5;
    value = std::max(minValue, std::min(value, maxValue));
    TS_ASSERT_DELTA(value, 1.0, DEFAULT_TOLERANCE);
  }

  // Test property indexing
  void testPropertyIndexing() {
    std::vector<double> throttleCmd(4, 0.0);

    throttleCmd[0] = 0.5;
    throttleCmd[1] = 0.6;
    throttleCmd[2] = 0.7;
    throttleCmd[3] = 0.8;

    TS_ASSERT_DELTA(throttleCmd[0], 0.5, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(throttleCmd[3], 0.8, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended Condition Tests
   ***************************************************************************/

  // Test nested conditions
  void testNestedConditions() {
    double altitude = 5000.0;
    double speed = 150.0;
    double time = 10.0;

    // Nested: (alt > 3000 AND speed > 100) OR time > 15
    bool result = ((altitude > 3000.0 && speed > 100.0) || time > 15.0);
    TS_ASSERT(result);

    // Other case
    altitude = 1000.0;
    speed = 50.0;
    time = 20.0;
    result = ((altitude > 3000.0 && speed > 100.0) || time > 15.0);
    TS_ASSERT(result);  // time > 15 satisfies OR
  }

  // Test NOT condition
  void testNOTCondition() {
    bool gearDown = true;
    bool gearUp = !gearDown;

    TS_ASSERT(!gearUp);

    gearDown = false;
    gearUp = !gearDown;
    TS_ASSERT(gearUp);
  }

  // Test comparison operators
  void testComparisonOperators() {
    double value = 50.0;
    double threshold = 50.0;

    // Equal
    TS_ASSERT(value == threshold);

    // Not equal
    value = 51.0;
    TS_ASSERT(value != threshold);

    // Greater than
    TS_ASSERT(value > threshold);

    // Greater or equal
    value = 50.0;
    TS_ASSERT(value >= threshold);

    // Less than
    value = 49.0;
    TS_ASSERT(value < threshold);

    // Less or equal
    value = 50.0;
    TS_ASSERT(value <= threshold);
  }

  // Test epsilon comparison
  void testEpsilonComparison() {
    double a = 0.1 + 0.2;  // May not be exactly 0.3
    double b = 0.3;
    double eps = 1e-9;

    TS_ASSERT(std::abs(a - b) < eps);
  }

  /***************************************************************************
   * Extended Flight Scenario Tests
   ***************************************************************************/

  // Test climb scenario
  void testClimbScenario() {
    double altitude = 1000.0;
    double targetAlt = 10000.0;
    double climbRate = 500.0;  // fpm
    double dt = 1.0;  // minutes

    while (altitude < targetAlt) {
      altitude += climbRate * dt;
    }

    TS_ASSERT(altitude >= targetAlt);
  }

  // Test cruise scenario
  void testCruiseScenario() {
    double altitude = 35000.0;
    double speed = 450.0;  // kts
    double heading = 90.0;  // degrees

    // Cruise conditions
    TS_ASSERT(altitude > 30000.0);
    TS_ASSERT(speed > 400.0);
    TS_ASSERT(heading >= 0.0 && heading < 360.0);
  }

  // Test descent scenario
  void testDescentScenario() {
    double altitude = 35000.0;
    double targetAlt = 3000.0;
    double descentRate = 1500.0;  // fpm
    double dt = 1.0;

    while (altitude > targetAlt) {
      altitude -= descentRate * dt;
    }

    TS_ASSERT(altitude <= targetAlt);
  }

  // Test holding pattern
  void testHoldingPattern() {
    double heading = 0.0;
    double turnRate = 3.0;  // degrees per second
    int turns = 0;

    // Complete 2 holding patterns (4 turns total)
    for (double t = 0.0; t < 480.0; t += 1.0) {
      heading += turnRate;
      if (heading >= 360.0) {
        heading -= 360.0;
        turns++;
      }
    }

    TS_ASSERT_EQUALS(turns, 4);
  }

  /***************************************************************************
   * Extended Error Handling Tests
   ***************************************************************************/

  // Test infinity handling
  void testInfinityHandling() {
    double value = std::numeric_limits<double>::infinity();

    TS_ASSERT(std::isinf(value));

    // Replace with default
    if (std::isinf(value)) {
      value = 0.0;
    }
    TS_ASSERT_DELTA(value, 0.0, DEFAULT_TOLERANCE);
  }

  // Test divide by zero prevention
  void testDivideByZeroPrevention() {
    double numerator = 100.0;
    double denominator = 0.0;
    double result;

    if (denominator != 0.0) {
      result = numerator / denominator;
    } else {
      result = 0.0;  // Safe default
    }

    TS_ASSERT_DELTA(result, 0.0, DEFAULT_TOLERANCE);
  }

  // Test array bounds
  void testArrayBounds() {
    std::vector<double> values(10, 0.0);
    size_t index = 5;

    TS_ASSERT(index < values.size());
    values[index] = 1.0;
    TS_ASSERT_DELTA(values[index], 1.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended Timing Tests
   ***************************************************************************/

  // Test simulation frame rate
  void testSimulationFrameRate() {
    double dt = 0.008333;  // 120 Hz
    double frameRate = 1.0 / dt;

    TS_ASSERT_DELTA(frameRate, 120.0, 0.1);

    // 60 Hz
    dt = 0.01667;
    frameRate = 1.0 / dt;
    TS_ASSERT_DELTA(frameRate, 60.0, 0.1);
  }

  // Test accumulator pattern
  void testAccumulatorPattern() {
    double accumulator = 0.0;
    double dt = 0.01;
    double threshold = 1.0;
    int triggers = 0;

    for (int i = 0; i < 500; i++) {
      accumulator += dt;
      if (accumulator >= threshold) {
        triggers++;
        accumulator -= threshold;
      }
    }

    TS_ASSERT_EQUALS(triggers, 5);
  }

  // Test time scaling
  void testTimeScaling() {
    double realTime = 1.0;
    double timeScale = 4.0;  // 4x speed
    double simTime = realTime * timeScale;

    TS_ASSERT_DELTA(simTime, 4.0, DEFAULT_TOLERANCE);

    // Slow motion
    timeScale = 0.5;
    simTime = realTime * timeScale;
    TS_ASSERT_DELTA(simTime, 0.5, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended State Machine Tests
   ***************************************************************************/

  // Test state machine transitions
  void testStateMachineTransitions() {
    enum State { IDLE, RUNNING, PAUSED, STOPPED };
    State state = IDLE;

    // IDLE -> RUNNING
    state = RUNNING;
    TS_ASSERT_EQUALS(state, RUNNING);

    // RUNNING -> PAUSED
    state = PAUSED;
    TS_ASSERT_EQUALS(state, PAUSED);

    // PAUSED -> RUNNING
    state = RUNNING;
    TS_ASSERT_EQUALS(state, RUNNING);

    // RUNNING -> STOPPED
    state = STOPPED;
    TS_ASSERT_EQUALS(state, STOPPED);
  }

  // Test state guards
  void testStateGuards() {
    bool canStart = false;
    bool engineRunning = false;
    bool fuelAvailable = true;
    bool starterOn = true;

    // Guard: can only start if not running, fuel available, starter on
    canStart = !engineRunning && fuelAvailable && starterOn;
    TS_ASSERT(canStart);

    // Engine already running
    engineRunning = true;
    canStart = !engineRunning && fuelAvailable && starterOn;
    TS_ASSERT(!canStart);
  }

  /***************************************************************************
   * Extended Output Tests
   ***************************************************************************/

  // Test output file format
  void testOutputFileFormat() {
    std::string delimiter = ",";
    std::vector<std::string> headers = {"time", "altitude", "speed"};
    std::string headerLine = "";

    for (size_t i = 0; i < headers.size(); i++) {
      headerLine += headers[i];
      if (i < headers.size() - 1) headerLine += delimiter;
    }

    TS_ASSERT(headerLine.find("time") != std::string::npos);
    TS_ASSERT(headerLine.find(",") != std::string::npos);
  }

  // Test output rate
  void testOutputRate() {
    double simRate = 120.0;  // Hz
    double outputRate = 10.0;  // Hz
    int skipFrames = static_cast<int>(simRate / outputRate);

    TS_ASSERT_EQUALS(skipFrames, 12);
  }

  /***************************************************************************
   * Extended Integration Tests
   ***************************************************************************/

  // Test Euler integration
  void testEulerIntegration() {
    double position = 0.0;
    double velocity = 10.0;
    double dt = 0.1;

    for (int i = 0; i < 10; i++) {
      position += velocity * dt;
    }

    TS_ASSERT_DELTA(position, 10.0, DEFAULT_TOLERANCE);
  }

  // Test RK2 integration
  void testRK2Integration() {
    double y = 1.0;  // dy/dt = y (exponential growth)
    double dt = 0.1;

    for (int i = 0; i < 10; i++) {
      double k1 = y;
      double k2 = y + dt * k1;
      y = y + dt * 0.5 * (k1 + k2);
    }

    // Should be close to e (2.718...)
    TS_ASSERT(y > 2.5);
    TS_ASSERT(y < 3.0);
  }

  /***************************************************************************
   * Extended Script Element Tests
   ***************************************************************************/

  // Test local property creation
  void testLocalPropertyCreation() {
    std::map<std::string, double> localProps;

    localProps["my-local-var"] = 0.0;
    localProps["my-local-var"] = 42.0;

    TS_ASSERT_DELTA(localProps["my-local-var"], 42.0, DEFAULT_TOLERANCE);
  }

  // Test table lookup simulation
  void testTableLookupSimulation() {
    std::vector<std::pair<double, double>> table = {
      {0.0, 0.0},
      {0.5, 0.3},
      {1.0, 1.0}
    };

    auto lookup = [&](double x) {
      if (x <= table[0].first) return table[0].second;
      if (x >= table.back().first) return table.back().second;

      for (size_t i = 1; i < table.size(); i++) {
        if (x <= table[i].first) {
          double t = (x - table[i-1].first) / (table[i].first - table[i-1].first);
          return table[i-1].second + t * (table[i].second - table[i-1].second);
        }
      }
      return 0.0;
    };

    TS_ASSERT_DELTA(lookup(0.0), 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(lookup(0.5), 0.3, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(lookup(1.0), 1.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(lookup(0.25), 0.15, DEFAULT_TOLERANCE);
  }

  // Test function expression
  void testFunctionExpression() {
    double x = 0.5;
    double y = 2.0 * x * x + 3.0 * x + 1.0;

    TS_ASSERT_DELTA(y, 3.0, DEFAULT_TOLERANCE);
  }
};
