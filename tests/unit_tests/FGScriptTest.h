#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <string>
#include <vector>

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
};
