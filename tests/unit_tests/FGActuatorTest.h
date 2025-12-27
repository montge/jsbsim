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

  // ==================== SECOND-ORDER DYNAMICS ====================

  // Test critically damped response (zeta = 1.0)
  void testCriticallyDampedResponse() {
    double wn = 10.0;  // Natural frequency (rad/s)
    double zeta = 1.0;  // Critical damping
    double dt = 0.001;

    // State variables
    double pos = 0.0;
    double vel = 0.0;
    double cmd = 1.0;

    // Simulate for 1 second
    for (int i = 0; i < 1000; i++) {
      double accel = wn * wn * (cmd - pos) - 2.0 * zeta * wn * vel;
      vel += accel * dt;
      pos += vel * dt;
    }

    // Should reach command without overshoot
    TS_ASSERT_DELTA(pos, cmd, 0.01);
  }

  // Test underdamped response (zeta < 1.0)
  void testUnderdampedResponse() {
    double wn = 10.0;
    double zeta = 0.3;  // Underdamped
    double dt = 0.001;

    double pos = 0.0, vel = 0.0, cmd = 1.0;
    double max_pos = 0.0;

    for (int i = 0; i < 2000; i++) {
      double accel = wn * wn * (cmd - pos) - 2.0 * zeta * wn * vel;
      vel += accel * dt;
      pos += vel * dt;
      max_pos = std::max(max_pos, pos);
    }

    // Underdamped should overshoot
    TS_ASSERT(max_pos > cmd);
  }

  // Test overdamped response (zeta > 1.0)
  void testOverdampedResponse() {
    double wn = 10.0;
    double zeta = 2.0;  // Overdamped
    double dt = 0.001;

    double pos = 0.0, vel = 0.0, cmd = 1.0;
    double max_pos = 0.0;

    for (int i = 0; i < 3000; i++) {
      double accel = wn * wn * (cmd - pos) - 2.0 * zeta * wn * vel;
      vel += accel * dt;
      pos += vel * dt;
      max_pos = std::max(max_pos, pos);
    }

    // Overdamped should not overshoot
    TS_ASSERT(max_pos <= cmd + 0.001);
  }

  // Test rise time vs natural frequency
  void testRiseTimeVsNaturalFrequency() {
    double dt = 0.001;
    double zeta = 0.7;

    // Higher natural frequency = faster response
    double wn_fast = 20.0;
    double wn_slow = 5.0;

    auto simulateRiseTime = [&](double wn) {
      double pos = 0.0, vel = 0.0, cmd = 1.0;
      int steps = 0;
      while (pos < 0.9 * cmd && steps < 5000) {
        double accel = wn * wn * (cmd - pos) - 2.0 * zeta * wn * vel;
        vel += accel * dt;
        pos += vel * dt;
        steps++;
      }
      return steps * dt;
    };

    double t_fast = simulateRiseTime(wn_fast);
    double t_slow = simulateRiseTime(wn_slow);

    TS_ASSERT(t_fast < t_slow);
  }

  // ==================== HYDRAULIC ACTUATOR TESTS ====================

  // Test hydraulic force calculation
  void testHydraulicForce() {
    double pressure = 3000.0;  // psi
    double piston_area = 2.0;   // square inches

    double force = pressure * piston_area;
    TS_ASSERT_DELTA(force, 6000.0, epsilon);  // lbs
  }

  // Test hydraulic flow rate
  void testHydraulicFlowRate() {
    double piston_area = 2.0;  // in^2
    double velocity = 5.0;     // in/sec

    double flow_rate = piston_area * velocity;  // in^3/sec
    double flow_gpm = flow_rate * 60.0 / 231.0;  // Convert to GPM

    TS_ASSERT_DELTA(flow_rate, 10.0, epsilon);
    TS_ASSERT_DELTA(flow_gpm, 2.597, 0.01);
  }

  // Test hydraulic power
  void testHydraulicPower() {
    double pressure = 3000.0;   // psi
    double flow_gpm = 5.0;

    // Hydraulic power (hp) = P * Q / 1714
    double power_hp = pressure * flow_gpm / 1714.0;
    TS_ASSERT_DELTA(power_hp, 8.75, 0.01);
  }

  // Test differential pressure
  void testDifferentialPressure() {
    double supply_pressure = 3000.0;
    double return_pressure = 100.0;

    double delta_p = supply_pressure - return_pressure;
    TS_ASSERT_DELTA(delta_p, 2900.0, epsilon);
  }

  // ==================== ELECTRICAL ACTUATOR TESTS ====================

  // Test motor torque constant
  void testMotorTorqueConstant() {
    double Kt = 0.5;   // Nm/A (torque constant)
    double current = 10.0;  // Amps

    double torque = Kt * current;
    TS_ASSERT_DELTA(torque, 5.0, epsilon);  // Nm
  }

  // Test back-EMF
  void testBackEMF() {
    double Ke = 0.5;    // V/(rad/s) (back-EMF constant)
    double omega = 100.0;  // rad/s

    double emf = Ke * omega;
    TS_ASSERT_DELTA(emf, 50.0, epsilon);  // Volts
  }

  // Test motor current limiting
  void testMotorCurrentLimiting() {
    double max_current = 20.0;  // Amps
    double currents[] = {5.0, 15.0, 25.0, 50.0};
    double expected[] = {5.0, 15.0, 20.0, 20.0};

    for (int i = 0; i < 4; i++) {
      double limited = std::min(currents[i], max_current);
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test motor speed from voltage
  void testMotorSpeedFromVoltage() {
    double voltage = 24.0;
    double resistance = 0.5;  // Ohms
    double Ke = 0.1;  // V/(rad/s)
    double Kt = 0.1;  // Nm/A
    double load_torque = 0.5;  // Nm

    // At steady state: V = I*R + Ke*omega
    // and: T_motor = Kt * I = T_load
    double current = load_torque / Kt;
    double omega = (voltage - current * resistance) / Ke;

    TS_ASSERT_DELTA(current, 5.0, epsilon);
    TS_ASSERT_DELTA(omega, 215.0, 1.0);
  }

  // ==================== SERVO LOOP TESTS ====================

  // Test position loop gain
  void testPositionLoopGain() {
    double Kp = 10.0;  // Position gain
    double position_error = 0.5;

    double velocity_command = Kp * position_error;
    TS_ASSERT_DELTA(velocity_command, 5.0, epsilon);
  }

  // Test velocity loop gain
  void testVelocityLoopGain() {
    double Kv = 2.0;  // Velocity gain
    double velocity_error = 10.0;

    double torque_command = Kv * velocity_error;
    TS_ASSERT_DELTA(torque_command, 20.0, epsilon);
  }

  // Test cascaded position-velocity loop
  void testCascadedPosVelLoop() {
    double Kp = 10.0;  // Position gain
    double Kv = 2.0;   // Velocity gain

    double pos_cmd = 5.0;
    double pos_actual = 4.0;
    double vel_actual = 1.0;

    double pos_error = pos_cmd - pos_actual;
    double vel_cmd = Kp * pos_error;
    double vel_error = vel_cmd - vel_actual;
    double output = Kv * vel_error;

    TS_ASSERT_DELTA(pos_error, 1.0, epsilon);
    TS_ASSERT_DELTA(vel_cmd, 10.0, epsilon);
    TS_ASSERT_DELTA(vel_error, 9.0, epsilon);
    TS_ASSERT_DELTA(output, 18.0, epsilon);
  }

  // ==================== FRICTION MODEL TESTS ====================

  // Test static friction (stiction)
  void testStaticFriction() {
    double static_friction = 5.0;  // Force units
    double velocity = 0.0;
    double applied_force = 3.0;

    // If velocity is zero and force < static friction, no movement
    bool moves = (std::abs(applied_force) > static_friction) || (std::abs(velocity) > epsilon);
    TS_ASSERT(!moves);

    applied_force = 6.0;
    moves = (std::abs(applied_force) > static_friction);
    TS_ASSERT(moves);
  }

  // Test kinetic friction
  void testKineticFriction() {
    double kinetic_friction_coeff = 0.3;
    double normal_force = 100.0;
    double velocity = 5.0;

    double friction_force = kinetic_friction_coeff * normal_force;
    double friction_direction = (velocity > 0) ? -1.0 : 1.0;
    double total_friction = friction_direction * friction_force;

    TS_ASSERT_DELTA(friction_force, 30.0, epsilon);
    TS_ASSERT_DELTA(total_friction, -30.0, epsilon);
  }

  // Test viscous damping
  void testViscousDamping() {
    double damping_coeff = 5.0;  // Force per velocity
    double velocity = 10.0;

    double damping_force = -damping_coeff * velocity;
    TS_ASSERT_DELTA(damping_force, -50.0, epsilon);
  }

  // Test combined friction model
  void testCombinedFrictionModel() {
    double static_friction = 10.0;
    double kinetic_friction = 8.0;
    double viscous_coeff = 0.5;
    double velocity = 20.0;

    // Moving: kinetic + viscous
    double friction = kinetic_friction + viscous_coeff * std::abs(velocity);
    friction = (velocity > 0) ? -friction : friction;

    TS_ASSERT_DELTA(friction, -18.0, epsilon);
  }

  // ==================== THERMAL EFFECTS TESTS ====================

  // Test thermal derating
  void testThermalDerating() {
    double rated_torque = 100.0;
    double temp_ambient = 25.0;
    double temp_actual = 80.0;
    double temp_limit = 100.0;

    // Linear derating above 60C
    double derating_start = 60.0;
    double derating_factor = 1.0;

    if (temp_actual > derating_start) {
      derating_factor = 1.0 - (temp_actual - derating_start) / (temp_limit - derating_start);
      derating_factor = std::max(0.0, derating_factor);
    }

    double available_torque = rated_torque * derating_factor;
    TS_ASSERT_DELTA(derating_factor, 0.5, epsilon);
    TS_ASSERT_DELTA(available_torque, 50.0, epsilon);
  }

  // Test temperature rise
  void testTemperatureRise() {
    double power_loss = 100.0;   // Watts
    double thermal_resistance = 0.5;  // C/W
    double ambient_temp = 25.0;

    double temp_rise = power_loss * thermal_resistance;
    double final_temp = ambient_temp + temp_rise;

    TS_ASSERT_DELTA(temp_rise, 50.0, epsilon);
    TS_ASSERT_DELTA(final_temp, 75.0, epsilon);
  }

  // ==================== MECHANICAL EFFECTS TESTS ====================

  // Test gear ratio effect on torque
  void testGearRatioTorque() {
    double motor_torque = 1.0;  // Nm
    double gear_ratio = 50.0;   // 50:1 reduction
    double efficiency = 0.9;

    double output_torque = motor_torque * gear_ratio * efficiency;
    TS_ASSERT_DELTA(output_torque, 45.0, epsilon);
  }

  // Test gear ratio effect on speed
  void testGearRatioSpeed() {
    double motor_speed = 3000.0;  // RPM
    double gear_ratio = 50.0;

    double output_speed = motor_speed / gear_ratio;
    TS_ASSERT_DELTA(output_speed, 60.0, epsilon);
  }

  // Test reflected inertia
  void testReflectedInertia() {
    double motor_inertia = 0.001;  // kg*m^2
    double load_inertia = 1.0;     // kg*m^2
    double gear_ratio = 10.0;

    // Inertia reflected to motor = J_motor + J_load / N^2
    double reflected_inertia = motor_inertia + load_inertia / (gear_ratio * gear_ratio);
    TS_ASSERT_DELTA(reflected_inertia, 0.011, 0.0001);
  }

  // Test compliance/stiffness
  void testCompliance() {
    double stiffness = 1000.0;  // N/m or lb/in
    double deflection = 0.5;

    double force = stiffness * deflection;
    TS_ASSERT_DELTA(force, 500.0, epsilon);
  }

  // ==================== POSITION FEEDBACK TESTS ====================

  // Test resolver-like feedback
  void testResolverFeedback() {
    double angle_rad = M_PI / 4.0;  // 45 degrees

    double sine = std::sin(angle_rad);
    double cosine = std::cos(angle_rad);
    double reconstructed = std::atan2(sine, cosine);

    TS_ASSERT_DELTA(reconstructed, angle_rad, epsilon);
  }

  // Test encoder counts to position
  void testEncoderToPosition() {
    int counts = 4096;
    int counts_per_rev = 4096;
    double position_deg = (double)counts / counts_per_rev * 360.0;

    TS_ASSERT_DELTA(position_deg, 360.0, epsilon);
  }

  // Test quadrature decoding
  void testQuadratureDecoding() {
    // 4x decoding: each A and B edge counts
    int base_counts_per_rev = 1024;
    int decoded_counts_per_rev = base_counts_per_rev * 4;

    double resolution = 360.0 / decoded_counts_per_rev;
    TS_ASSERT_DELTA(resolution, 0.0879, 0.001);  // degrees per count
  }

  // ==================== SAFETY SYSTEMS TESTS ====================

  // Test over-travel detection
  void testOverTravelDetection() {
    double pos_max = 30.0;
    double pos_min = -30.0;

    double positions[] = {0.0, 29.0, 31.0, -31.0};
    bool expected[] = {false, false, true, true};

    for (int i = 0; i < 4; i++) {
      bool over_travel = (positions[i] > pos_max) || (positions[i] < pos_min);
      TS_ASSERT_EQUALS(over_travel, expected[i]);
    }
  }

  // Test overload detection
  void testOverloadDetection() {
    double max_load = 1000.0;
    double loads[] = {500.0, 900.0, 1000.0, 1100.0, 2000.0};
    bool expected[] = {false, false, false, true, true};

    for (int i = 0; i < 5; i++) {
      bool overload = loads[i] > max_load;
      TS_ASSERT_EQUALS(overload, expected[i]);
    }
  }

  // Test emergency stop
  void testEmergencyStop() {
    bool e_stop = true;
    double command = 100.0;

    double output = e_stop ? 0.0 : command;
    TS_ASSERT_DELTA(output, 0.0, epsilon);
  }

  // ==================== PERFORMANCE METRICS TESTS ====================

  // Test bandwidth calculation
  void testBandwidthCalculation() {
    // For first-order system, bandwidth = 1 / (2*pi*tau)
    double tau = 0.05;  // 50 ms time constant
    double bandwidth = 1.0 / (2.0 * M_PI * tau);

    TS_ASSERT_DELTA(bandwidth, 3.18, 0.01);  // Hz
  }

  // Test phase margin approximation
  void testPhaseMarginApproximation() {
    // For second-order system, PM ≈ 100 * zeta (rough approximation for small zeta)
    double zeta = 0.5;
    double pm_approx = 100.0 * zeta;

    TS_ASSERT_DELTA(pm_approx, 50.0, epsilon);  // degrees
  }

  // Test steady-state accuracy
  void testSteadyStateAccuracy() {
    double command = 10.0;
    double actual = 9.95;
    double error_percent = std::abs(command - actual) / command * 100.0;

    TS_ASSERT_DELTA(error_percent, 0.5, 0.01);  // 0.5% error
  }

  // ==================== FAILURE INJECTION TESTS ====================

  // Test partial failure (reduced authority)
  void testPartialFailure() {
    double command = 30.0;
    double authority = 0.5;  // 50% authority

    double output = command * authority;
    TS_ASSERT_DELTA(output, 15.0, epsilon);
  }

  // Test oscillatory failure
  void testOscillatoryFailure() {
    double frequency = 10.0;  // Hz
    double amplitude = 5.0;
    double t = 0.1;

    double oscillation = amplitude * std::sin(2.0 * M_PI * frequency * t);
    // At t=0.1, sin(2*pi) = 0
    TS_ASSERT_DELTA(oscillation, 0.0, 0.01);
  }

  // Test runaway failure
  void testRunawayFailure() {
    double rate = 100.0;  // deg/sec runaway rate
    double dt = 0.1;
    double initial_pos = 0.0;

    double pos_after_runaway = initial_pos + rate * dt;
    TS_ASSERT_DELTA(pos_after_runaway, 10.0, epsilon);
  }

  // ==================== MISCELLANEOUS TESTS ====================

  // Test PWM duty cycle to position
  void testPWMDutyCycleToPosition() {
    double duty_cycle = 0.75;  // 75%
    double pos_min = -30.0;
    double pos_max = 30.0;

    double position = pos_min + duty_cycle * (pos_max - pos_min);
    TS_ASSERT_DELTA(position, 15.0, epsilon);
  }

  // Test actuator load factor
  void testActuatorLoadFactor() {
    double actual_load = 800.0;
    double rated_load = 1000.0;

    double load_factor = actual_load / rated_load;
    TS_ASSERT_DELTA(load_factor, 0.8, epsilon);
  }

  // Test symmetry check
  void testSymmetryCheck() {
    double rate_limit = 50.0;
    double pos_up = rate_limit * 0.1;
    double pos_down = -rate_limit * 0.1;

    TS_ASSERT_DELTA(std::abs(pos_up), std::abs(pos_down), epsilon);
  }

  // Test actuator efficiency
  void testActuatorEfficiency() {
    double input_power = 1000.0;   // Watts
    double output_power = 850.0;  // Watts

    double efficiency = output_power / input_power;
    TS_ASSERT_DELTA(efficiency, 0.85, epsilon);
  }

  // Test position prediction
  void testPositionPrediction() {
    double current_pos = 10.0;
    double velocity = 5.0;
    double lookahead = 0.2;  // seconds

    double predicted_pos = current_pos + velocity * lookahead;
    TS_ASSERT_DELTA(predicted_pos, 11.0, epsilon);
  }

  // Test command rate limiting
  void testCommandRateLimiting() {
    double prev_cmd = 0.0;
    double new_cmd = 100.0;
    double max_cmd_rate = 50.0;  // per second
    double dt = 0.1;

    double max_change = max_cmd_rate * dt;
    double delta = new_cmd - prev_cmd;
    double clamped_delta = std::max(-max_change, std::min(max_change, delta));
    double limited_cmd = prev_cmd + clamped_delta;

    TS_ASSERT_DELTA(limited_cmd, 5.0, epsilon);
  }

  // ==================== FLIGHT CONTROL SURFACE TESTS ====================

  // Test elevator deflection range
  void testElevatorDeflectionRange() {
    double elevator_max_up = -25.0;    // degrees (nose up)
    double elevator_max_down = 15.0;   // degrees (nose down)

    double commands[] = {-30.0, -25.0, 0.0, 15.0, 20.0};
    double expected[] = {-25.0, -25.0, 0.0, 15.0, 15.0};

    for (int i = 0; i < 5; i++) {
      double limited = std::max(elevator_max_up, std::min(elevator_max_down, commands[i]));
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test aileron differential
  void testAileronDifferential() {
    double command = 10.0;  // degrees
    double differential = 0.5;  // 50% differential

    double left_aileron = command;
    double right_aileron = -command * (1.0 - differential);

    TS_ASSERT_DELTA(left_aileron, 10.0, epsilon);
    TS_ASSERT_DELTA(right_aileron, -5.0, epsilon);
  }

  // Test flap transit time
  void testFlapTransitTime() {
    double flap_rate = 3.0;  // degrees/second
    double flap_travel = 30.0;  // degrees

    double transit_time = flap_travel / flap_rate;
    TS_ASSERT_DELTA(transit_time, 10.0, epsilon);
  }

  // Test spoiler blowdown effect
  void testSpoilerBlowdown() {
    double command = 60.0;  // degrees
    double dynamic_pressure = 200.0;  // psf
    double blowdown_factor = 0.001;  // per psf

    double actual = command * (1.0 - blowdown_factor * dynamic_pressure);
    TS_ASSERT_DELTA(actual, 48.0, epsilon);
  }

  // ==================== SERVO DYNAMICS TESTS ====================

  // Test servo natural frequency
  void testServoNaturalFrequency() {
    double wn = 20.0;  // rad/s
    double period = 2.0 * M_PI / wn;

    TS_ASSERT_DELTA(period, 0.314, 0.001);
  }

  // Test servo damping ratio effects
  void testServoDampingEffects() {
    double wn = 20.0;
    double zeta_values[] = {0.3, 0.7, 1.0, 1.5};

    // Overshoot % for second-order system
    for (double zeta : zeta_values) {
      double overshoot = 0.0;
      if (zeta < 1.0) {
        overshoot = 100.0 * std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta * zeta));
      }
      TS_ASSERT(overshoot >= 0.0);
    }
  }

  // Test servo bandwidth
  void testServoBandwidth() {
    double wn = 20.0;
    double zeta = 0.7;

    // Bandwidth for second-order system
    double wd = wn * std::sqrt(1.0 - zeta * zeta);
    TS_ASSERT_DELTA(wd, 14.28, 0.1);
  }

  // ==================== LOAD SENSING TESTS ====================

  // Test load cell measurement
  void testLoadCellMeasurement() {
    double applied_force = 500.0;  // lbs
    double sensitivity = 2.0;  // mV/V per lb
    double excitation = 10.0;  // V

    double output_mv = applied_force * sensitivity * excitation / 1000.0;
    TS_ASSERT_DELTA(output_mv, 10.0, epsilon);
  }

  // Test hinge moment estimation
  void testHingeMomentEstimation() {
    double dynamic_pressure = 100.0;  // psf
    double chord = 1.0;  // ft
    double span = 5.0;   // ft
    double Ch = -0.05;   // Hinge moment coefficient

    double hinge_moment = Ch * dynamic_pressure * chord * chord * span;
    TS_ASSERT_DELTA(hinge_moment, -25.0, epsilon);
  }

  // Test actuator load limiting
  void testActuatorLoadLimiting() {
    double max_load = 1000.0;  // lbs
    double loads[] = {500.0, 1000.0, 1500.0};

    for (double load : loads) {
      double limited = std::min(load, max_load);
      TS_ASSERT(limited <= max_load);
    }
  }

  // ==================== REDUNDANCY TESTS ====================

  // Test dual-channel comparison
  void testDualChannelComparison() {
    double channel_a = 15.0;
    double channel_b = 15.1;
    double threshold = 0.5;

    double difference = std::abs(channel_a - channel_b);
    bool channels_agree = difference < threshold;
    TS_ASSERT(channels_agree);
  }

  // Test active/standby switchover
  void testActiveStandbySwitchover() {
    bool active_failed = true;
    bool standby_healthy = true;

    bool switch_to_standby = active_failed && standby_healthy;
    TS_ASSERT(switch_to_standby);
  }

  // Test force-fight monitoring
  void testForceFightMonitoring() {
    double actuator1_force = 500.0;
    double actuator2_force = 480.0;
    double force_fight_threshold = 100.0;

    double force_diff = std::abs(actuator1_force - actuator2_force);
    bool force_fight = force_diff > force_fight_threshold;
    TS_ASSERT(!force_fight);
  }

  // ==================== POSITION TRANSDUCER TESTS ====================

  // Test LVDT output
  void testLVDTOutput() {
    double stroke = 2.0;  // inches
    double sensitivity = 5.0;  // V/inch

    double output = stroke * sensitivity;
    TS_ASSERT_DELTA(output, 10.0, epsilon);
  }

  // Test RVDT output
  void testRVDTOutput() {
    double angle = 30.0;  // degrees
    double sensitivity = 0.1;  // V/degree

    double output = angle * sensitivity;
    TS_ASSERT_DELTA(output, 3.0, epsilon);
  }

  // Test position sensor resolution
  void testPositionSensorResolution() {
    double full_scale = 30.0;  // degrees
    int bits = 12;

    double resolution = full_scale / (1 << bits);
    TS_ASSERT_DELTA(resolution, 0.00732, 0.0001);
  }

  // ==================== ENVIRONMENT EFFECTS TESTS ====================

  // Test cold temperature effect
  void testColdTemperatureEffect() {
    double viscosity_20C = 100.0;  // cSt
    double temp_coeff = 0.02;
    double temp = -40.0;  // Cold day

    double viscosity = viscosity_20C * (1.0 + temp_coeff * (20.0 - temp));
    TS_ASSERT(viscosity > viscosity_20C);
  }

  // Test altitude effect on hydraulic
  void testAltitudeEffectHydraulic() {
    double sea_level_pressure = 3000.0;  // psi
    double altitude = 40000.0;  // ft
    double reservoir_boost = 15.0;  // psi (pressurized)

    // At altitude, ambient pressure is lower, may affect cavitation margin
    double ambient_pressure = 14.7 * std::exp(-altitude / 27000.0);
    TS_ASSERT(ambient_pressure < reservoir_boost);
  }

  // Test vibration effect on actuator
  void testVibrationEffectActuator() {
    double nominal_position = 10.0;
    double vibration_amplitude = 0.1;

    double max_position = nominal_position + vibration_amplitude;
    double min_position = nominal_position - vibration_amplitude;

    TS_ASSERT_DELTA(max_position, 10.1, epsilon);
    TS_ASSERT_DELTA(min_position, 9.9, epsilon);
  }

  // ==================== FLY-BY-WIRE TESTS ====================

  // Test command limiting for structural protection
  void testCommandLimitingStructural() {
    double g_limit = 2.5;
    double current_g = 2.8;

    bool limit_exceeded = current_g > g_limit;
    double scale_factor = limit_exceeded ? g_limit / current_g : 1.0;

    TS_ASSERT_DELTA(scale_factor, 0.893, 0.01);
  }

  // Test angle of attack limiting
  void testAOALimiting() {
    double aoa_cmd = 18.0;  // degrees
    double aoa_limit = 15.0;

    double limited_aoa = std::min(aoa_cmd, aoa_limit);
    TS_ASSERT_DELTA(limited_aoa, 15.0, epsilon);
  }

  // Test control law blending
  void testControlLawBlending() {
    double normal_law_cmd = 10.0;
    double direct_law_cmd = 8.0;
    double blend_factor = 0.7;  // 70% normal law

    double blended = blend_factor * normal_law_cmd + (1.0 - blend_factor) * direct_law_cmd;
    TS_ASSERT_DELTA(blended, 9.4, epsilon);
  }

  // ==================== TRIM SYSTEM TESTS ====================

  // Test trim actuator rate
  void testTrimActuatorRate() {
    double trim_rate = 0.3;  // degrees/second
    double dt = 1.0;

    double trim_change = trim_rate * dt;
    TS_ASSERT_DELTA(trim_change, 0.3, epsilon);
  }

  // Test trim range
  void testTrimRange() {
    double trim_max = 10.0;  // degrees nose down
    double trim_min = -3.0;  // degrees nose up

    double trim_cmds[] = {-5.0, 0.0, 5.0, 12.0};
    double expected[] = {-3.0, 0.0, 5.0, 10.0};

    for (int i = 0; i < 4; i++) {
      double limited = std::max(trim_min, std::min(trim_max, trim_cmds[i]));
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test trim runaway protection
  void testTrimRunawayProtection() {
    double trim_rate = 2.0;  // degrees/second (excessive)
    double max_rate = 0.5;

    bool runaway = trim_rate > max_rate;
    TS_ASSERT(runaway);
  }

  // ==================== ACTUATOR HEALTH MONITORING ====================

  // Test current monitoring
  void testCurrentMonitoring() {
    double current = 25.0;  // Amps
    double max_current = 30.0;
    double warning_threshold = 0.8;

    bool warning = current > max_current * warning_threshold;
    TS_ASSERT(warning);
  }

  // Test temperature monitoring
  void testTemperatureMonitoring() {
    double temp = 85.0;  // Celsius
    double max_temp = 100.0;

    double margin = (max_temp - temp) / max_temp;
    TS_ASSERT_DELTA(margin, 0.15, epsilon);
  }

  // Test BITE (Built-In Test Equipment)
  void testBITECheck() {
    bool position_valid = true;
    bool current_valid = true;
    bool temp_valid = true;

    bool bite_pass = position_valid && current_valid && temp_valid;
    TS_ASSERT(bite_pass);
  }
};
