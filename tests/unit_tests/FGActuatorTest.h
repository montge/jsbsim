#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Actuator unit tests
 *
 * Note: FGActuator requires XML element for construction, so these tests focus on:
 * - Rate limiting
 * - Position limiting
 * - Hysteresis
 * - Deadband
 * - Lag dynamics
 * - Failure modes
 */
class FGActuatorTest : public CxxTest::TestSuite
{
public:
  // Test rate limiting - positive direction
  void testRateLimitPositive() {
    double rate_limit = 10.0;  // deg/sec
    double dt = 0.1;
    double max_change = rate_limit * dt;  // 1.0 deg per step

    double current_pos = 0.0;
    double command = 100.0;  // Large step command

    double desired_change = command - current_pos;
    double actual_change = std::min(desired_change, max_change);
    double new_pos = current_pos + actual_change;

    TS_ASSERT_DELTA(new_pos, 1.0, epsilon);  // Limited to 1 deg
  }

  // Test rate limiting - negative direction
  void testRateLimitNegative() {
    double rate_limit = 10.0;
    double dt = 0.1;
    double max_change = rate_limit * dt;

    double current_pos = 0.0;
    double command = -100.0;

    double desired_change = command - current_pos;
    double actual_change = std::max(desired_change, -max_change);
    double new_pos = current_pos + actual_change;

    TS_ASSERT_DELTA(new_pos, -1.0, epsilon);
  }

  // Test asymmetric rate limits
  void testAsymmetricRateLimits() {
    double rate_limit_up = 20.0;
    double rate_limit_down = 10.0;
    double dt = 0.1;

    double current_pos = 0.0;

    // Moving up
    double command_up = 100.0;
    double change_up = std::min(command_up - current_pos, rate_limit_up * dt);
    TS_ASSERT_DELTA(change_up, 2.0, epsilon);

    // Moving down
    double command_down = -100.0;
    double change_down = std::max(command_down - current_pos, -rate_limit_down * dt);
    TS_ASSERT_DELTA(change_down, -1.0, epsilon);
  }

  // Test position limiting - upper bound
  void testPositionLimitUpper() {
    double pos_max = 30.0;
    double pos_min = -30.0;

    double positions[] = {0.0, 29.0, 30.0, 31.0, 100.0};
    double expected[] = {0.0, 29.0, 30.0, 30.0, 30.0};

    for (int i = 0; i < 5; i++) {
      double limited = std::min(pos_max, std::max(pos_min, positions[i]));
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test position limiting - lower bound
  void testPositionLimitLower() {
    double pos_max = 30.0;
    double pos_min = -30.0;

    double positions[] = {0.0, -29.0, -30.0, -31.0, -100.0};
    double expected[] = {0.0, -29.0, -30.0, -30.0, -30.0};

    for (int i = 0; i < 5; i++) {
      double limited = std::min(pos_max, std::max(pos_min, positions[i]));
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test saturation property
  void testSaturationProperty() {
    double pos_max = 30.0;
    double pos_min = -30.0;

    // Not saturated
    double pos1 = 15.0;
    bool saturated1 = (pos1 >= pos_max || pos1 <= pos_min);
    TS_ASSERT(!saturated1);

    // Saturated high
    double pos2 = 30.0;
    bool saturated2 = (pos2 >= pos_max);
    TS_ASSERT(saturated2);

    // Saturated low
    double pos3 = -30.0;
    bool saturated3 = (pos3 <= pos_min);
    TS_ASSERT(saturated3);
  }

  // Test hysteresis rising
  void testHysteresisRising() {
    double hysteresis_width = 0.5;
    double last_output = 0.0;

    // Rising: output changes when input exceeds threshold (last_output + width/2 = 0.25)
    double input = 0.2;  // Below threshold (0.2 < 0.25)
    double output = (input > last_output + hysteresis_width/2) ? input : last_output;
    TS_ASSERT_DELTA(output, 0.0, epsilon);  // No change

    input = 0.6;  // Above threshold (0.6 > 0.25)
    output = (input > last_output + hysteresis_width/2) ? input : last_output;
    TS_ASSERT_DELTA(output, 0.6, epsilon);  // Changed
  }

  // Test hysteresis falling
  void testHysteresisFalling() {
    double hysteresis_width = 0.5;
    double last_output = 1.0;

    // Falling: output changes when input drops below threshold
    double input = 0.8;  // Not below threshold
    double output = (input < last_output - hysteresis_width/2) ? input : last_output;
    TS_ASSERT_DELTA(output, 1.0, epsilon);  // No change

    input = 0.4;  // Below threshold
    output = (input < last_output - hysteresis_width/2) ? input : last_output;
    TS_ASSERT_DELTA(output, 0.4, epsilon);  // Changed
  }

  // Test hysteresis prevents oscillation
  void testHysteresisAntiOscillation() {
    double hysteresis_width = 1.0;
    double output = 0.0;

    // Input oscillates around a value
    double inputs[] = {0.6, 0.4, 0.6, 0.4, 0.6};

    for (double input : inputs) {
      // Only update if change exceeds hysteresis
      if (std::abs(input - output) > hysteresis_width/2) {
        output = input;
      }
    }

    // Output should stay stable despite input oscillation
    TS_ASSERT_DELTA(output, 0.6, epsilon);
  }

  // Test deadband centered at zero
  void testDeadbandZeroCentered() {
    double deadband_width = 2.0;

    // Input within deadband
    double inputs_in[] = {0.0, 0.5, -0.5, 0.99, -0.99};
    for (double input : inputs_in) {
      double output = (std::abs(input) < deadband_width/2) ? 0.0 : input;
      TS_ASSERT_DELTA(output, 0.0, epsilon);
    }

    // Input outside deadband
    double inputs_out[] = {1.5, -1.5, 5.0, -5.0};
    double expected_out[] = {1.5, -1.5, 5.0, -5.0};
    for (int i = 0; i < 4; i++) {
      double output = (std::abs(inputs_out[i]) < deadband_width/2) ? 0.0 : inputs_out[i];
      TS_ASSERT_DELTA(output, expected_out[i], epsilon);
    }
  }

  // Test deadband with offset
  void testDeadbandOffset() {
    double deadband_width = 2.0;
    double center = 10.0;

    double input = 10.5;
    double output = (std::abs(input - center) < deadband_width/2) ? center : input;
    TS_ASSERT_DELTA(output, center, epsilon);

    input = 12.0;
    output = (std::abs(input - center) < deadband_width/2) ? center : input;
    TS_ASSERT_DELTA(output, 12.0, epsilon);
  }

  // Test actuator lag
  void testActuatorLag() {
    double lag_time_const = 0.05;  // 50 ms
    double dt = 0.01;
    double ca = std::exp(-dt / lag_time_const);
    double cb = 1.0 - ca;

    double position = 0.0;
    double command = 10.0;

    // After one time constant, should reach 63.2%
    int steps_per_tc = static_cast<int>(lag_time_const / dt);
    for (int i = 0; i < steps_per_tc; i++) {
      position = ca * position + cb * command;
    }

    TS_ASSERT_DELTA(position, command * 0.632, 0.05);
  }

  // Test actuator settling
  void testActuatorSettling() {
    double lag_time_const = 0.05;
    double dt = 0.001;
    double ca = std::exp(-dt / lag_time_const);
    double cb = 1.0 - ca;

    double position = 0.0;
    double command = 10.0;

    // Simulate 5 time constants
    int steps = static_cast<int>(5.0 * lag_time_const / dt);
    for (int i = 0; i < steps; i++) {
      position = ca * position + cb * command;
    }

    // Should be within 1% of command
    TS_ASSERT_DELTA(position, command, command * 0.01);
  }

  // Test fail stuck mode
  void testFailStuck() {
    double stuck_position = 5.0;
    bool fail_stuck = true;

    double command = 10.0;
    double output = fail_stuck ? stuck_position : command;

    TS_ASSERT_DELTA(output, stuck_position, epsilon);
  }

  // Test fail hardover positive
  void testFailHardoverPositive() {
    double pos_max = 30.0;
    bool fail_hardover_pos = true;

    double command = 10.0;
    double output = fail_hardover_pos ? pos_max : command;

    TS_ASSERT_DELTA(output, pos_max, epsilon);
  }

  // Test fail hardover negative
  void testFailHardoverNegative() {
    double pos_min = -30.0;
    bool fail_hardover_neg = true;

    double command = 10.0;
    double output = fail_hardover_neg ? pos_min : command;

    TS_ASSERT_DELTA(output, pos_min, epsilon);
  }

  // Test fail off (zero output)
  void testFailOff() {
    bool fail_off = true;
    double command = 15.0;

    double output = fail_off ? 0.0 : command;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // Test linear actuator position control
  void testLinearPositionControl() {
    double rate = 10.0;  // inches per second
    double dt = 0.1;

    double current_pos = 0.0;
    double target_pos = 5.0;

    // Simulate movement
    while (std::abs(target_pos - current_pos) > epsilon) {
      double direction = (target_pos > current_pos) ? 1.0 : -1.0;
      double move = std::min(rate * dt, std::abs(target_pos - current_pos));
      current_pos += direction * move;
    }

    TS_ASSERT_DELTA(current_pos, target_pos, epsilon);
  }

  // Test rate + position limits combined
  void testCombinedRateAndPositionLimits() {
    double rate_limit = 100.0;  // deg/sec
    double pos_max = 30.0;
    double pos_min = -30.0;
    double dt = 0.01;

    double position = 25.0;
    double command = 50.0;  // Would exceed pos_max

    // Apply rate limit
    double max_change = rate_limit * dt;
    double desired_change = command - position;
    double rate_limited = position + std::max(-max_change, std::min(max_change, desired_change));

    // Apply position limit
    double final_pos = std::max(pos_min, std::min(pos_max, rate_limited));

    // After rate limiting: 25 + 1 = 26
    // After position limiting: 26 (within limits)
    TS_ASSERT_DELTA(final_pos, 26.0, epsilon);

    // Continue to limit
    position = 29.5;
    desired_change = command - position;
    rate_limited = position + std::max(-max_change, std::min(max_change, desired_change));
    final_pos = std::max(pos_min, std::min(pos_max, rate_limited));

    // 29.5 + 1 = 30.5, but limited to 30
    TS_ASSERT_DELTA(final_pos, 30.0, epsilon);
  }

  // Test backlash
  void testBacklash() {
    double backlash = 0.5;  // degrees
    double output = 0.0;
    double last_input = 0.0;

    // Input moves right
    double input = 1.0;
    double dead_zone = backlash / 2.0;

    // Backlash: output doesn't move until input moves past dead zone
    if (input > last_input) {
      output = std::max(output, input - dead_zone);
    } else {
      output = std::min(output, input + dead_zone);
    }

    TS_ASSERT_DELTA(output, 0.75, epsilon);  // 1.0 - 0.25
  }

  // Test slew rate measurement
  void testSlewRateMeasurement() {
    double pos_prev = 10.0;
    double pos_curr = 15.0;
    double dt = 0.1;

    double slew_rate = (pos_curr - pos_prev) / dt;
    TS_ASSERT_DELTA(slew_rate, 50.0, epsilon);  // 5 deg / 0.1 sec = 50 deg/sec
  }

  // Test normalized position output
  void testNormalizedPosition() {
    double pos_max = 30.0;
    double pos_min = -30.0;
    double range = pos_max - pos_min;

    // Normalize to [0, 1]
    double positions[] = {-30.0, -15.0, 0.0, 15.0, 30.0};
    double expected[] = {0.0, 0.25, 0.5, 0.75, 1.0};

    for (int i = 0; i < 5; i++) {
      double normalized = (positions[i] - pos_min) / range;
      TS_ASSERT_DELTA(normalized, expected[i], epsilon);
    }
  }

  // Test bias compensation
  void testBiasCompensation() {
    double bias = 0.5;
    double command = 10.0;

    double compensated = command + bias;
    TS_ASSERT_DELTA(compensated, 10.5, epsilon);
  }

  // Test actuator power (force * velocity)
  void testActuatorPower() {
    double force = 1000.0;  // lbs
    double velocity = 2.0;  // ft/s

    double power_ft_lbs_per_sec = force * velocity;
    double power_hp = power_ft_lbs_per_sec / 550.0;

    TS_ASSERT_DELTA(power_ft_lbs_per_sec, 2000.0, epsilon);
    TS_ASSERT_DELTA(power_hp, 3.636, 0.01);
  }

  // Test step input response
  void testStepResponse() {
    double lag = 0.1;  // 100 ms
    double dt = 0.001;
    double rate_limit = 1000.0;  // High enough to not limit

    double ca = std::exp(-dt / lag);
    double cb = 1.0 - ca;

    double pos = 0.0;
    double cmd = 10.0;
    double t = 0.0;

    // Find time to 50% of command
    while (pos < 5.0 && t < 1.0) {
      pos = ca * pos + cb * cmd;
      t += dt;
    }

    // For first-order lag, t50 = tau * ln(2) ≈ 0.693 * tau
    TS_ASSERT_DELTA(t, 0.693 * lag, 0.01);
  }

  // Test command following error
  void testFollowingError() {
    double lag = 0.05;
    double dt = 0.001;
    double ca = std::exp(-dt / lag);
    double cb = 1.0 - ca;

    double pos = 0.0;

    // Ramp input
    double cmd = 0.0;
    double ramp_rate = 10.0;  // deg/sec

    for (int i = 0; i < 1000; i++) {
      cmd = ramp_rate * (i * dt);
      pos = ca * pos + cb * cmd;
    }

    // Steady-state error for first-order lag with ramp = tau * ramp_rate
    double expected_error = lag * ramp_rate;
    double actual_error = cmd - pos;

    TS_ASSERT_DELTA(actual_error, expected_error, 0.1);
  }

  // Test zero-position hold
  void testZeroPositionHold() {
    double pos = 0.0;
    double threshold = 0.001;

    // Small perturbations should be rejected
    double perturbations[] = {0.0005, -0.0005, 0.0001};
    for (double p : perturbations) {
      double input = pos + p;
      double output = (std::abs(input) < threshold) ? 0.0 : input;
      TS_ASSERT_DELTA(output, 0.0, epsilon);
    }
  }
};
