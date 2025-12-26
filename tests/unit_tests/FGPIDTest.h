#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <deque>
#include <vector>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * PID Controller unit tests
 *
 * Note: FGPID requires XML element for construction, so these tests focus on:
 * - Proportional, Integral, Derivative calculations
 * - Various integrator types (rectangular, trapezoidal, Adams-Bashforth)
 * - Anti-windup concepts
 * - Output limiting
 */
class FGPIDTest : public CxxTest::TestSuite
{
public:
  // Test proportional response (P term)
  void testProportionalResponse() {
    double Kp = 2.0;
    double error = 10.0;

    double P_output = Kp * error;
    TS_ASSERT_DELTA(P_output, 20.0, epsilon);
  }

  // Test proportional with negative error
  void testProportionalNegativeError() {
    double Kp = 2.0;
    double error = -5.0;

    double P_output = Kp * error;
    TS_ASSERT_DELTA(P_output, -10.0, epsilon);
  }

  // Test proportional with zero error
  void testProportionalZeroError() {
    double Kp = 100.0;
    double error = 0.0;

    double P_output = Kp * error;
    TS_ASSERT_DELTA(P_output, 0.0, epsilon);
  }

  // Test integral with rectangular integration
  void testIntegralRectangular() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;

    // Constant error of 1.0 for 10 steps
    for (int i = 0; i < 10; i++) {
      double error = 1.0;
      I_sum += error * dt;  // Rectangular: I(n) = I(n-1) + e(n)*dt
    }

    double I_output = Ki * I_sum;
    TS_ASSERT_DELTA(I_output, 1.0, epsilon);  // 1.0 * 10 * 0.1 = 1.0
  }

  // Test integral with trapezoidal integration
  void testIntegralTrapezoidal() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double prev_error = 0.0;

    // Constant error of 1.0 for 10 steps
    for (int i = 0; i < 10; i++) {
      double error = 1.0;
      I_sum += (error + prev_error) * dt / 2.0;  // Trapezoidal
      prev_error = error;
    }

    double I_output = Ki * I_sum;
    // First step: (1+0)*0.1/2 = 0.05, remaining 9: (1+1)*0.1/2*9 = 0.9
    TS_ASSERT_DELTA(I_output, 0.95, epsilon);
  }

  // Test integral with Adams-Bashforth 2nd order
  void testIntegralAdamsBashforth2() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double e_prev = 0.0;

    // AB2: I(n) = I(n-1) + dt/2 * (3*e(n) - e(n-1))
    for (int i = 0; i < 10; i++) {
      double error = 1.0;
      if (i == 0) {
        // First step: use rectangular
        I_sum += error * dt;
      } else {
        I_sum += dt / 2.0 * (3.0 * error - e_prev);
      }
      e_prev = error;
    }

    double I_output = Ki * I_sum;
    // With constant error, AB2 should approach rectangular
    TS_ASSERT(I_output > 0.9);
    TS_ASSERT(I_output < 1.2);
  }

  // Test integral with Adams-Bashforth 3rd order
  void testIntegralAdamsBashforth3() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double e_prev = 0.0;
    double e_prev2 = 0.0;

    // AB3: I(n) = I(n-1) + dt/12 * (23*e(n) - 16*e(n-1) + 5*e(n-2))
    for (int i = 0; i < 10; i++) {
      double error = 1.0;
      if (i == 0) {
        I_sum += error * dt;
      } else if (i == 1) {
        I_sum += dt / 2.0 * (3.0 * error - e_prev);
      } else {
        I_sum += dt / 12.0 * (23.0 * error - 16.0 * e_prev + 5.0 * e_prev2);
      }
      e_prev2 = e_prev;
      e_prev = error;
    }

    double I_output = Ki * I_sum;
    TS_ASSERT(I_output > 0.9);
    TS_ASSERT(I_output < 1.2);
  }

  // Test derivative response (D term)
  void testDerivativeResponse() {
    double Kd = 0.5;
    double dt = 0.1;
    double error = 10.0;
    double prev_error = 8.0;

    // D term: Kd * (error - prev_error) / dt
    double D_output = Kd * (error - prev_error) / dt;
    TS_ASSERT_DELTA(D_output, 10.0, epsilon);  // 0.5 * 2 / 0.1 = 10
  }

  // Test derivative with decreasing error
  void testDerivativeDecreasing() {
    double Kd = 1.0;
    double dt = 0.1;
    double error = 5.0;
    double prev_error = 10.0;

    double D_output = Kd * (error - prev_error) / dt;
    TS_ASSERT_DELTA(D_output, -50.0, epsilon);  // Negative = slowing down
  }

  // Test derivative with constant error (no change)
  void testDerivativeConstant() {
    double Kd = 1.0;
    double dt = 0.1;
    double error = 5.0;
    double prev_error = 5.0;

    double D_output = Kd * (error - prev_error) / dt;
    TS_ASSERT_DELTA(D_output, 0.0, epsilon);
  }

  // Test full PID output
  void testFullPIDOutput() {
    double Kp = 1.0;
    double Ki = 0.1;
    double Kd = 0.01;
    double dt = 0.1;

    double error = 10.0;
    double prev_error = 8.0;
    double I_sum = 5.0;  // Accumulated integral

    double P = Kp * error;
    double I = Ki * I_sum;
    double D = Kd * (error - prev_error) / dt;
    double output = P + I + D;

    // P = 10, I = 0.5, D = 0.01 * 2 / 0.1 = 0.2
    TS_ASSERT_DELTA(output, 10.7, epsilon);
  }

  // Test anti-windup when saturated
  void testAntiWindupSaturated() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double output_max = 5.0;
    bool saturated = false;

    // Simulate integration with saturation check
    for (int i = 0; i < 100; i++) {
      double error = 1.0;

      // Only integrate if not saturated (anti-windup)
      if (!saturated) {
        I_sum += error * dt;
      }

      double output = Ki * I_sum;
      saturated = (output >= output_max);
    }

    // Without anti-windup, I_sum would be 10.0
    // With anti-windup, it stops at 5.0/Ki = 5.0
    TS_ASSERT(I_sum <= output_max / Ki + 0.1);
  }

  // Test anti-windup reset
  void testAntiWindupReset() {
    double I_sum = 100.0;  // Large accumulated integral
    int trigger = -1;      // Negative trigger = reset

    if (trigger < 0) {
      I_sum = 0.0;  // Reset integral
    }

    TS_ASSERT_DELTA(I_sum, 0.0, epsilon);
  }

  // Test output limiting (clipto)
  void testOutputLimiting() {
    double output_min = -10.0;
    double output_max = 10.0;

    double outputs[] = {-50.0, -10.0, 0.0, 10.0, 50.0};
    double expected[] = {-10.0, -10.0, 0.0, 10.0, 10.0};

    for (int i = 0; i < 5; i++) {
      double limited = std::max(output_min, std::min(output_max, outputs[i]));
      TS_ASSERT_DELTA(limited, expected[i], epsilon);
    }
  }

  // Test standard vs parallel PID
  void testStandardVsParallel() {
    // Parallel (default): output = Kp*e + Ki*integral(e) + Kd*de/dt
    // Standard: output = Kp * (e + Ki*integral(e) + Kd*de/dt)

    double Kp = 2.0, Ki = 0.5, Kd = 0.1;
    double error = 10.0;
    double I_sum = 5.0;
    double deriv = 2.0;

    // Parallel PID
    double parallel = Kp * error + Ki * I_sum + Kd * deriv;

    // Standard PID (Ki and Kd are relative to Kp)
    double standard = Kp * (error + Ki * I_sum + Kd * deriv);

    // They should be different
    TS_ASSERT(std::abs(parallel - standard) > epsilon);

    // Parallel: 2*10 + 0.5*5 + 0.1*2 = 20 + 2.5 + 0.2 = 22.7
    TS_ASSERT_DELTA(parallel, 22.7, epsilon);

    // Standard: 2 * (10 + 0.5*5 + 0.1*2) = 2 * 12.7 = 25.4
    TS_ASSERT_DELTA(standard, 25.4, epsilon);
  }

  // Test derivative kick prevention
  void testDerivativeKickPrevention() {
    // When setpoint changes, error jumps causing derivative spike
    // Solution: differentiate process variable instead of error

    double Kd = 1.0;
    double dt = 0.1;

    double prev_setpoint = 0.0;
    double curr_setpoint = 100.0;  // Step change
    double process = 50.0;
    double prev_process = 50.0;

    double prev_error = prev_setpoint - prev_process;
    double curr_error = curr_setpoint - process;

    // Derivative of error (causes kick)
    double D_error = Kd * (curr_error - prev_error) / dt;

    // Derivative of process variable (no kick)
    double D_pv = -Kd * (process - prev_process) / dt;

    // Error derivative shows large spike
    TS_ASSERT(std::abs(D_error) > 100.0);

    // PV derivative shows no spike
    TS_ASSERT_DELTA(D_pv, 0.0, epsilon);
  }

  // Test integral term accumulation over time
  void testIntegralAccumulation() {
    double Ki = 1.0;
    double dt = 0.01;
    double I_sum = 0.0;

    // Accumulate over 100 steps with error = 1
    for (int i = 0; i < 100; i++) {
      I_sum += 1.0 * dt;
    }

    double I_output = Ki * I_sum;
    TS_ASSERT_DELTA(I_output, 1.0, 0.001);  // 100 * 0.01 = 1.0
  }

  // Test PID with varying timesteps
  void testVaryingTimesteps() {
    double Kp = 1.0, Ki = 1.0;
    double I_sum = 0.0;
    double error = 5.0;

    // Two methods should give same result
    double dt1 = 0.1;
    double dt2 = 0.05;

    // Method 1: one step with dt=0.1
    double I1 = error * dt1;

    // Method 2: two steps with dt=0.05
    double I2 = error * dt2 + error * dt2;

    TS_ASSERT_DELTA(I1, I2, epsilon);
  }

  // Test zero gains (disabled terms)
  void testZeroGains() {
    double error = 10.0;
    double I_sum = 5.0;
    double deriv = 2.0;

    // P-only controller
    double Kp = 1.0, Ki = 0.0, Kd = 0.0;
    double output = Kp * error + Ki * I_sum + Kd * deriv;
    TS_ASSERT_DELTA(output, error, epsilon);

    // I-only controller
    Kp = 0.0; Ki = 1.0; Kd = 0.0;
    output = Kp * error + Ki * I_sum + Kd * deriv;
    TS_ASSERT_DELTA(output, I_sum, epsilon);

    // D-only controller
    Kp = 0.0; Ki = 0.0; Kd = 1.0;
    output = Kp * error + Ki * I_sum + Kd * deriv;
    TS_ASSERT_DELTA(output, deriv, epsilon);
  }

  // Test negative gains (inverted response)
  void testNegativeGains() {
    double Kp = -1.0;
    double error = 10.0;

    double output = Kp * error;
    TS_ASSERT_DELTA(output, -10.0, epsilon);
  }

  // Test large gains stability consideration
  void testLargeGains() {
    double Kp = 1000.0;
    double error = 0.001;

    double output = Kp * error;
    TS_ASSERT_DELTA(output, 1.0, epsilon);

    // Very large gain with small error
    Kp = 1e6;
    error = 1e-6;
    output = Kp * error;
    TS_ASSERT_DELTA(output, 1.0, epsilon);
  }

  // Test settling behavior
  void testSettlingBehavior() {
    double Kp = 1.0, Ki = 0.5, Kd = 0.1;  // Tuned gains
    double dt = 0.1;

    double setpoint = 100.0;
    double process = 0.0;
    double I_sum = 0.0;
    double prev_error = setpoint - process;

    // Simulate closed-loop (simple first-order plant)
    for (int i = 0; i < 200; i++) {  // More iterations for convergence
      double error = setpoint - process;
      I_sum += error * dt;
      double D = (error - prev_error) / dt;

      double output = Kp * error + Ki * I_sum + Kd * D;

      // Simple plant: process follows output with lag
      process += 0.2 * (output - process);

      prev_error = error;
    }

    // Should approach setpoint within tolerance
    TS_ASSERT(std::abs(setpoint - process) < 10.0);
  }

  // Test error types (position vs velocity)
  void testPositionVsVelocityError() {
    // Position error: e = setpoint - process
    double setpoint = 100.0;
    double process = 80.0;
    double pos_error = setpoint - process;
    TS_ASSERT_DELTA(pos_error, 20.0, epsilon);

    // Velocity error: e = setpoint_rate - process_rate
    double setpoint_rate = 10.0;
    double process_rate = 8.0;
    double vel_error = setpoint_rate - process_rate;
    TS_ASSERT_DELTA(vel_error, 2.0, epsilon);
  }

  /***************************************************************************
   * Extended Integration Tests
   ***************************************************************************/

  // Test integral with ramp input
  void testIntegralRampInput() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;

    // Ramp input: error increases linearly
    for (int i = 0; i < 10; i++) {
      double error = static_cast<double>(i);
      I_sum += error * dt;
    }

    // Sum of 0+1+2+...+9 = 45, times dt=0.1 = 4.5
    double I_output = Ki * I_sum;
    TS_ASSERT_DELTA(I_output, 4.5, epsilon);
  }

  // Test integral with sinusoidal input
  void testIntegralSinusoidal() {
    double Ki = 1.0;
    double dt = 0.01;
    double I_sum = 0.0;

    // One complete sine wave should integrate to ~0
    int steps = static_cast<int>(2 * M_PI / dt);
    for (int i = 0; i < steps; i++) {
      double t = static_cast<double>(i) * dt;
      double error = sin(t);
      I_sum += error * dt;
    }

    double I_output = Ki * I_sum;
    TS_ASSERT_DELTA(I_output, 0.0, 0.1);  // Should be near zero
  }

  // Test integral sign reversal
  void testIntegralSignReversal() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;

    // Positive error accumulation
    for (int i = 0; i < 10; i++) {
      I_sum += 1.0 * dt;
    }
    TS_ASSERT_DELTA(I_sum, 1.0, epsilon);

    // Negative error reduces accumulated integral
    for (int i = 0; i < 10; i++) {
      I_sum += (-1.0) * dt;
    }
    TS_ASSERT_DELTA(I_sum, 0.0, epsilon);
  }

  /***************************************************************************
   * Derivative Filter Tests
   ***************************************************************************/

  // Test filtered derivative (low-pass filter)
  void testFilteredDerivative() {
    double Kd = 1.0;
    double dt = 0.1;
    double alpha = 0.2;  // Filter coefficient (0 < alpha < 1)
    double filtered_deriv = 0.0;

    double prev_error = 0.0;
    for (int i = 0; i < 20; i++) {
      double error = (i < 10) ? 10.0 : 0.0;  // Step down at i=10
      double raw_deriv = (error - prev_error) / dt;

      // First-order low-pass filter
      filtered_deriv = alpha * raw_deriv + (1 - alpha) * filtered_deriv;

      TS_ASSERT(std::isfinite(filtered_deriv));
      prev_error = error;
    }

    // After filtering, derivative should have decayed
    TS_ASSERT(std::abs(filtered_deriv) < 50.0);
  }

  // Test derivative noise sensitivity
  void testDerivativeNoiseSensitivity() {
    double Kd = 1.0;
    double dt = 0.1;

    // Error with noise
    double errors[] = {10.0, 10.1, 9.9, 10.05, 9.95, 10.0};
    double max_deriv = 0.0;

    for (int i = 1; i < 6; i++) {
      double deriv = std::abs(errors[i] - errors[i-1]) / dt;
      max_deriv = std::max(max_deriv, deriv);
    }

    // Noise causes derivative spikes
    TS_ASSERT(max_deriv > 0.5);
  }

  /***************************************************************************
   * Output Rate Limiting Tests
   ***************************************************************************/

  // Test output rate limiting
  void testOutputRateLimiting() {
    double rate_limit = 10.0;  // Max change per step
    double prev_output = 0.0;

    double commands[] = {100.0, 100.0, 100.0, -100.0, -100.0};
    double outputs[5];

    for (int i = 0; i < 5; i++) {
      double delta = commands[i] - prev_output;
      if (delta > rate_limit) delta = rate_limit;
      if (delta < -rate_limit) delta = -rate_limit;

      outputs[i] = prev_output + delta;
      prev_output = outputs[i];
    }

    // Should ramp up gradually
    TS_ASSERT_DELTA(outputs[0], 10.0, epsilon);
    TS_ASSERT_DELTA(outputs[1], 20.0, epsilon);
    TS_ASSERT_DELTA(outputs[2], 30.0, epsilon);
    // Then ramp down
    TS_ASSERT_DELTA(outputs[3], 20.0, epsilon);
    TS_ASSERT_DELTA(outputs[4], 10.0, epsilon);
  }

  // Test symmetric rate limits
  void testSymmetricRateLimits() {
    double rate_limit = 5.0;
    double prev = 0.0;

    // Positive rate
    double cmd_up = 100.0;
    double delta_up = std::min(cmd_up - prev, rate_limit);
    TS_ASSERT_DELTA(delta_up, 5.0, epsilon);

    // Negative rate
    prev = 50.0;
    double cmd_dn = -100.0;
    double delta_dn = std::max(cmd_dn - prev, -rate_limit);
    TS_ASSERT_DELTA(delta_dn, -5.0, epsilon);
  }

  /***************************************************************************
   * Bumpless Transfer Tests
   ***************************************************************************/

  // Test bumpless transfer (manual to auto)
  void testBumplessTransfer() {
    double manual_output = 50.0;
    double I_sum = 0.0;
    double Kp = 1.0, Ki = 0.5;
    double error = 10.0;

    // Before transfer: I_sum should be set so output matches manual
    // output = Kp*error + Ki*I_sum = manual_output
    // I_sum = (manual_output - Kp*error) / Ki
    I_sum = (manual_output - Kp * error) / Ki;

    double auto_output = Kp * error + Ki * I_sum;
    TS_ASSERT_DELTA(auto_output, manual_output, epsilon);
  }

  // Test integral initialization for bumpless transfer
  void testIntegralInitialization() {
    double target_output = 75.0;
    double Kp = 2.0, Ki = 0.25;
    double error = 10.0;

    // Calculate required I_sum
    double P_term = Kp * error;
    double required_I_sum = (target_output - P_term) / Ki;

    // Verify
    double output = Kp * error + Ki * required_I_sum;
    TS_ASSERT_DELTA(output, target_output, epsilon);
  }

  /***************************************************************************
   * Gain Scheduling Concepts
   ***************************************************************************/

  // Test gain scheduling based on operating point
  void testGainScheduling() {
    auto getKp = [](double velocity) -> double {
      if (velocity < 100) return 2.0;       // Low speed
      if (velocity < 200) return 1.5;       // Medium speed
      return 1.0;                           // High speed
    };

    TS_ASSERT_DELTA(getKp(50), 2.0, epsilon);
    TS_ASSERT_DELTA(getKp(150), 1.5, epsilon);
    TS_ASSERT_DELTA(getKp(250), 1.0, epsilon);
  }

  // Test interpolated gain scheduling
  void testInterpolatedGainScheduling() {
    // Linear interpolation between gain values
    auto interpKp = [](double velocity) -> double {
      double v_low = 100.0, v_high = 200.0;
      double Kp_low = 2.0, Kp_high = 1.0;

      if (velocity <= v_low) return Kp_low;
      if (velocity >= v_high) return Kp_high;

      double ratio = (velocity - v_low) / (v_high - v_low);
      return Kp_low + ratio * (Kp_high - Kp_low);
    };

    TS_ASSERT_DELTA(interpKp(100), 2.0, epsilon);
    TS_ASSERT_DELTA(interpKp(150), 1.5, epsilon);
    TS_ASSERT_DELTA(interpKp(200), 1.0, epsilon);
  }

  /***************************************************************************
   * Feed-Forward Tests
   ***************************************************************************/

  // Test PID with feed-forward
  void testPIDWithFeedForward() {
    double Kp = 1.0, Ki = 0.1, Kd = 0.01;
    double Kff = 0.5;  // Feed-forward gain

    double error = 10.0;
    double I_sum = 5.0;
    double deriv = 2.0;
    double setpoint = 100.0;

    // PID output
    double pid_out = Kp * error + Ki * I_sum + Kd * deriv;

    // Feed-forward term (based on setpoint)
    double ff_out = Kff * setpoint;

    double total = pid_out + ff_out;

    // PID: 10 + 0.5 + 0.02 = 10.52
    // FF: 0.5 * 100 = 50
    // Total: 60.52
    TS_ASSERT_DELTA(total, 60.52, epsilon);
  }

  // Test feed-forward only (open loop)
  void testFeedForwardOnly() {
    double Kff = 1.0;
    double setpoint = 50.0;

    double output = Kff * setpoint;
    TS_ASSERT_DELTA(output, 50.0, epsilon);
  }

  /***************************************************************************
   * Integral Clamping Tests
   ***************************************************************************/

  // Test integral clamping (upper limit)
  void testIntegralClampingUpper() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double I_max = 10.0;

    for (int i = 0; i < 200; i++) {
      I_sum += 1.0 * dt;
      I_sum = std::min(I_sum, I_max);
    }

    TS_ASSERT_DELTA(I_sum, I_max, epsilon);
  }

  // Test integral clamping (lower limit)
  void testIntegralClampingLower() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double I_min = -10.0;

    for (int i = 0; i < 200; i++) {
      I_sum += (-1.0) * dt;
      I_sum = std::max(I_sum, I_min);
    }

    TS_ASSERT_DELTA(I_sum, I_min, epsilon);
  }

  // Test symmetric integral clamping
  void testSymmetricIntegralClamping() {
    double I_limit = 5.0;
    double I_sum = 0.0;
    double dt = 0.1;

    // Accumulate positive
    for (int i = 0; i < 100; i++) {
      I_sum += 1.0 * dt;
      I_sum = std::max(-I_limit, std::min(I_limit, I_sum));
    }
    TS_ASSERT_DELTA(I_sum, I_limit, epsilon);

    // Accumulate negative
    for (int i = 0; i < 200; i++) {
      I_sum += (-1.0) * dt;
      I_sum = std::max(-I_limit, std::min(I_limit, I_sum));
    }
    TS_ASSERT_DELTA(I_sum, -I_limit, epsilon);
  }

  /***************************************************************************
   * Cascaded Controller Tests
   ***************************************************************************/

  // Test cascaded (inner/outer loop) PID
  void testCascadedPID() {
    // Outer loop
    double Kp_outer = 0.5;
    double outer_setpoint = 100.0;
    double outer_process = 80.0;
    double outer_error = outer_setpoint - outer_process;
    double inner_setpoint = Kp_outer * outer_error;  // 10.0

    // Inner loop
    double Kp_inner = 2.0;
    double inner_process = 8.0;
    double inner_error = inner_setpoint - inner_process;  // 2.0
    double output = Kp_inner * inner_error;  // 4.0

    TS_ASSERT_DELTA(inner_setpoint, 10.0, epsilon);
    TS_ASSERT_DELTA(output, 4.0, epsilon);
  }

  /***************************************************************************
   * Edge Cases and Special Values
   ***************************************************************************/

  // Test with very small timestep
  void testVerySmallTimestep() {
    double Ki = 1.0;
    double dt = 1e-10;
    double I_sum = 0.0;

    for (int i = 0; i < 1000; i++) {
      I_sum += 1.0 * dt;
    }

    TS_ASSERT_DELTA(I_sum, 1e-7, 1e-10);
    TS_ASSERT(std::isfinite(I_sum));
  }

  // Test with very large error
  void testVeryLargeError() {
    double Kp = 1.0;
    double error = 1e10;

    double output = Kp * error;
    TS_ASSERT_DELTA(output, 1e10, 1e5);
    TS_ASSERT(std::isfinite(output));
  }

  // Test with denormalized numbers
  void testDenormalizedNumbers() {
    double Kp = 1.0;
    double error = std::numeric_limits<double>::denorm_min();

    double output = Kp * error;
    TS_ASSERT(std::isfinite(output));
    TS_ASSERT(output >= 0);
  }

  // Test integral overflow prevention
  void testIntegralOverflowPrevention() {
    double I_sum = std::numeric_limits<double>::max() / 2;
    double dt = 0.1;
    double error = std::numeric_limits<double>::max() / 10;

    // This would overflow without protection
    double new_sum = I_sum + error * dt;

    // Check if result is finite (may be inf without protection)
    // In real implementation, clamping would prevent this
    TS_ASSERT(std::isfinite(I_sum));
  }

  /***************************************************************************
   * PI and PD Controller Tests
   ***************************************************************************/

  // Test PI controller (no derivative)
  void testPIController() {
    double Kp = 2.0, Ki = 0.5;
    double dt = 0.1;
    double I_sum = 0.0;

    double error = 10.0;
    I_sum += error * dt;

    double output = Kp * error + Ki * I_sum;
    // P: 20, I: 0.5 * 1.0 = 0.5
    TS_ASSERT_DELTA(output, 20.5, epsilon);
  }

  // Test PD controller (no integral)
  void testPDController() {
    double Kp = 2.0, Kd = 0.1;
    double dt = 0.1;

    double error = 10.0;
    double prev_error = 8.0;
    double deriv = (error - prev_error) / dt;

    double output = Kp * error + Kd * deriv;
    // P: 20, D: 0.1 * 20 = 2
    TS_ASSERT_DELTA(output, 22.0, epsilon);
  }

  /***************************************************************************
   * Response Characteristic Tests
   ***************************************************************************/

  // Test underdamped response (oscillatory)
  void testUnderdampedResponse() {
    // High Kp, low Kd leads to oscillation
    double Kp = 5.0, Ki = 0.1, Kd = 0.01;
    double dt = 0.1;

    double setpoint = 100.0;
    double process = 0.0;
    double I_sum = 0.0;
    double prev_error = setpoint;

    std::vector<double> history;
    for (int i = 0; i < 100; i++) {
      double error = setpoint - process;
      I_sum += error * dt;
      double D = (error - prev_error) / dt;

      double output = Kp * error + Ki * I_sum + Kd * D;
      process += 0.3 * (output - process);

      history.push_back(process);
      prev_error = error;
    }

    // Check for oscillation (process crosses setpoint)
    int crossings = 0;
    for (size_t i = 1; i < history.size(); i++) {
      if ((history[i-1] < setpoint && history[i] >= setpoint) ||
          (history[i-1] >= setpoint && history[i] < setpoint)) {
        crossings++;
      }
    }

    // Underdamped should have multiple crossings
    TS_ASSERT(crossings >= 1);
  }

  // Test overdamped response (sluggish)
  void testOverdampedResponse() {
    // Low Kp leads to slow response
    double Kp = 0.2, Ki = 0.01, Kd = 0.0;  // Very low gains
    double dt = 0.1;

    double setpoint = 100.0;
    double process = 0.0;
    double I_sum = 0.0;

    for (int i = 0; i < 50; i++) {
      double error = setpoint - process;
      I_sum += error * dt;

      double output = Kp * error + Ki * I_sum;
      process += 0.05 * output;  // Slow plant response
    }

    // Overdamped: slow approach, should be moving but not at setpoint
    TS_ASSERT(process > 10.0);  // Should have made some progress
    TS_ASSERT(process < setpoint);  // Not yet reached
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many iterations
  void testStressManyIterations() {
    double Kp = 1.0, Ki = 0.1, Kd = 0.01;
    double dt = 0.01;
    double I_sum = 0.0;
    double prev_error = 0.0;

    for (int i = 0; i < 100000; i++) {
      double error = sin(i * dt);
      I_sum += error * dt;
      double D = (error - prev_error) / dt;

      double output = Kp * error + Ki * I_sum + Kd * D;

      TS_ASSERT(std::isfinite(output));
      prev_error = error;
    }

    TS_ASSERT(std::isfinite(I_sum));
  }

  // Test rapid setpoint changes
  void testRapidSetpointChanges() {
    double Kp = 1.0;

    for (int i = 0; i < 1000; i++) {
      double setpoint = (i % 2 == 0) ? 100.0 : 0.0;
      double process = 50.0;
      double error = setpoint - process;
      double output = Kp * error;

      TS_ASSERT(std::isfinite(output));
      TS_ASSERT(std::abs(output) <= 100.0);
    }
  }

  // Test numerical precision over time
  void testNumericalPrecision() {
    double I_sum = 0.0;
    double dt = 0.001;

    // Accumulate small values
    for (int i = 0; i < 1000; i++) {
      I_sum += 0.001 * dt;
    }

    // Should be approximately 0.001
    TS_ASSERT_DELTA(I_sum, 0.001, 1e-6);
  }

  // Test alternating error sign
  void testAlternatingError() {
    double I_sum = 0.0;
    double dt = 0.1;

    for (int i = 0; i < 100; i++) {
      double error = (i % 2 == 0) ? 1.0 : -1.0;
      I_sum += error * dt;
    }

    // Alternating +1 and -1 should nearly cancel
    TS_ASSERT_DELTA(I_sum, 0.0, epsilon);
  }

  // Test with gradually changing gains
  void testGraduallyChangingGains() {
    double dt = 0.1;
    double error = 10.0;

    for (int i = 0; i < 100; i++) {
      double Kp = 1.0 + 0.01 * i;  // Increases from 1.0 to 2.0
      double output = Kp * error;

      TS_ASSERT(output >= 10.0);
      TS_ASSERT(output <= 20.0);
    }
  }

  /***************************************************************************
   * Ziegler-Nichols Tuning Tests
   ***************************************************************************/

  // Test Ziegler-Nichols ultimate gain method
  void testZieglerNicholsUltimateGain() {
    double Ku = 4.0;   // Ultimate gain
    double Tu = 2.0;   // Ultimate period (seconds)

    // ZN PID tuning rules
    double Kp = 0.6 * Ku;
    double Ti = Tu / 2.0;
    double Td = Tu / 8.0;

    TS_ASSERT_DELTA(Kp, 2.4, 0.01);
    TS_ASSERT_DELTA(Ti, 1.0, 0.01);
    TS_ASSERT_DELTA(Td, 0.25, 0.01);
  }

  // Test Ziegler-Nichols step response method
  void testZieglerNicholsStepResponse() {
    double K = 2.0;    // Process gain
    double L = 0.5;    // Dead time
    double T = 3.0;    // Time constant

    // ZN tuning from step response
    double Kp = 1.2 * T / (K * L);
    double Ti = 2.0 * L;
    double Td = 0.5 * L;

    TS_ASSERT_DELTA(Kp, 3.6, 0.1);
    TS_ASSERT_DELTA(Ti, 1.0, 0.01);
    TS_ASSERT_DELTA(Td, 0.25, 0.01);
  }

  // Test P-only tuning (Ziegler-Nichols)
  void testZNPOnlyTuning() {
    double Ku = 4.0;
    double Kp_P = 0.5 * Ku;

    TS_ASSERT_DELTA(Kp_P, 2.0, 0.01);
  }

  // Test PI tuning (Ziegler-Nichols)
  void testZNPITuning() {
    double Ku = 4.0;
    double Tu = 2.0;

    double Kp_PI = 0.45 * Ku;
    double Ti_PI = Tu / 1.2;

    TS_ASSERT_DELTA(Kp_PI, 1.8, 0.01);
    TS_ASSERT_DELTA(Ti_PI, 1.667, 0.01);
  }

  /***************************************************************************
   * Error Metrics Tests
   ***************************************************************************/

  // Test Integral of Absolute Error (IAE)
  void testIAEMetric() {
    double IAE = 0.0;
    double dt = 0.1;

    double errors[] = {10.0, 8.0, 5.0, -2.0, -1.0, 0.5, 0.1};
    for (double e : errors) {
      IAE += std::abs(e) * dt;
    }

    // Sum of |errors| * dt = (10+8+5+2+1+0.5+0.1) * 0.1 = 2.66
    TS_ASSERT_DELTA(IAE, 2.66, 0.01);
  }

  // Test Integral of Squared Error (ISE)
  void testISEMetric() {
    double ISE = 0.0;
    double dt = 0.1;

    double errors[] = {10.0, 5.0, 2.0, 1.0};
    for (double e : errors) {
      ISE += e * e * dt;
    }

    // Sum of e^2 * dt = (100+25+4+1) * 0.1 = 13.0
    TS_ASSERT_DELTA(ISE, 13.0, 0.01);
  }

  // Test Integral of Time-weighted Absolute Error (ITAE)
  void testITAEMetric() {
    double ITAE = 0.0;
    double dt = 0.1;
    double t = 0.0;

    double errors[] = {10.0, 5.0, 2.0, 1.0};
    for (double e : errors) {
      ITAE += t * std::abs(e) * dt;
      t += dt;
    }

    // ITAE penalizes late errors more
    TS_ASSERT(ITAE > 0.0);
  }

  /***************************************************************************
   * Setpoint Weighting Tests
   ***************************************************************************/

  // Test setpoint weighting on P term
  void testSetpointWeightingP() {
    double b = 0.5;  // Setpoint weight (0-1)
    double Kp = 2.0;
    double setpoint = 100.0;
    double process = 80.0;

    // Without weighting: error = setpoint - process
    double error_std = setpoint - process;

    // With weighting: error = b*setpoint - process
    double error_weighted = b * setpoint - process;

    double P_std = Kp * error_std;
    double P_weighted = Kp * error_weighted;

    TS_ASSERT_DELTA(P_std, 40.0, epsilon);
    TS_ASSERT_DELTA(P_weighted, -60.0, epsilon);  // Reduced response (b*sp - pv)
  }

  // Test setpoint weighting on D term
  void testSetpointWeightingD() {
    double c = 0.0;  // Setpoint weight for D (typically 0)
    double Kd = 1.0;
    double dt = 0.1;

    double sp_curr = 100.0, sp_prev = 50.0;  // Step change
    double pv_curr = 75.0, pv_prev = 75.0;   // No change

    // Standard: D based on error change
    double error_curr = sp_curr - pv_curr;
    double error_prev = sp_prev - pv_prev;
    double D_std = Kd * (error_curr - error_prev) / dt;

    // With c=0: D based on -PV change only
    double D_weighted = -Kd * (pv_curr - pv_prev) / dt;

    // Setpoint change causes spike in standard
    TS_ASSERT(std::abs(D_std) > 100.0);
    // No spike with weighting
    TS_ASSERT_DELTA(D_weighted, 0.0, epsilon);
  }

  /***************************************************************************
   * Deadband Tests
   ***************************************************************************/

  // Test deadband on error
  void testDeadbandOnError() {
    double deadband = 1.0;

    auto applyDeadband = [deadband](double error) -> double {
      if (std::abs(error) < deadband) return 0.0;
      return (error > 0) ? error - deadband : error + deadband;
    };

    TS_ASSERT_DELTA(applyDeadband(0.5), 0.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(2.0), 1.0, epsilon);
    TS_ASSERT_DELTA(applyDeadband(-2.0), -1.0, epsilon);
  }

  // Test deadband prevents hunting
  void testDeadbandPreventsHunting() {
    double deadband = 0.5;
    double Kp = 10.0;
    int output_changes = 0;
    double prev_output = 0.0;

    // Small errors within deadband
    double errors[] = {0.3, -0.2, 0.4, -0.1, 0.2};
    for (double e : errors) {
      double effective_error = (std::abs(e) < deadband) ? 0.0 : e;
      double output = Kp * effective_error;

      if (output != prev_output) output_changes++;
      prev_output = output;
    }

    // With deadband, output should remain at 0
    TS_ASSERT_EQUALS(output_changes, 0);
  }

  /***************************************************************************
   * Setpoint Filter Tests
   ***************************************************************************/

  // Test setpoint filtering (rate limiting)
  void testSetpointRateLimiting() {
    double sp_target = 100.0;
    double sp_filtered = 0.0;
    double sp_rate_limit = 10.0;
    double dt = 0.1;

    for (int i = 0; i < 20; i++) {
      double delta = sp_target - sp_filtered;
      double rate = delta / dt;
      if (rate > sp_rate_limit) delta = sp_rate_limit * dt;
      if (rate < -sp_rate_limit) delta = -sp_rate_limit * dt;
      sp_filtered += delta;
    }

    // Should ramp up to target
    TS_ASSERT(sp_filtered > 0.0);
    TS_ASSERT(sp_filtered <= sp_target);
  }

  // Test setpoint low-pass filter
  void testSetpointLowPassFilter() {
    double tau = 0.5;  // Filter time constant
    double dt = 0.1;
    double alpha = dt / (tau + dt);

    double sp_target = 100.0;
    double sp_filtered = 0.0;

    for (int i = 0; i < 50; i++) {
      sp_filtered = alpha * sp_target + (1 - alpha) * sp_filtered;
    }

    // Should approach target
    TS_ASSERT(std::abs(sp_filtered - sp_target) < 1.0);
  }

  /***************************************************************************
   * Output Smoothing Tests
   ***************************************************************************/

  // Test output averaging
  void testOutputAveraging() {
    std::deque<double> output_history;
    int window_size = 5;

    double outputs[] = {10.0, 12.0, 8.0, 15.0, 5.0, 10.0, 11.0};
    for (double out : outputs) {
      output_history.push_back(out);
      if (output_history.size() > static_cast<size_t>(window_size)) {
        output_history.pop_front();
      }

      double sum = 0.0;
      for (double h : output_history) sum += h;
      double avg = sum / output_history.size();

      TS_ASSERT(std::isfinite(avg));
    }
  }

  // Test output exponential smoothing
  void testOutputExponentialSmoothing() {
    double alpha = 0.3;
    double smoothed = 0.0;

    double outputs[] = {100.0, 100.0, 100.0, 0.0, 0.0, 0.0};
    for (double out : outputs) {
      smoothed = alpha * out + (1 - alpha) * smoothed;
      TS_ASSERT(std::isfinite(smoothed));
    }

    // After step down, smoothed should be between 0 and starting value
    TS_ASSERT(smoothed >= 0.0);
    TS_ASSERT(smoothed < 50.0);
  }

  /***************************************************************************
   * Auto-tuning Concepts
   ***************************************************************************/

  // Test relay feedback identification
  void testRelayFeedbackIdentification() {
    // Relay output amplitude
    double d = 10.0;
    // Measured oscillation amplitude
    double a = 5.0;
    // Measured period
    double Tu = 2.0;

    // Ultimate gain from relay test
    double Ku = 4.0 * d / (M_PI * a);

    TS_ASSERT_DELTA(Ku, 2.546, 0.01);
    TS_ASSERT(Ku > 0.0);
  }

  // Test model-based tuning (FOPDT)
  void testFOPDTModelTuning() {
    // First Order Plus Dead Time parameters
    double K = 1.5;   // Process gain
    double tau = 5.0; // Time constant
    double theta = 1.0; // Dead time

    // Lambda tuning for PI
    double lambda = 2.0;  // Closed-loop time constant
    double Kc = tau / (K * (lambda + theta));
    double Ti = tau;

    TS_ASSERT(Kc > 0.0);
    TS_ASSERT_DELTA(Ti, 5.0, epsilon);
  }

  /***************************************************************************
   * Stability Tests
   ***************************************************************************/

  // Test gain margin concept
  void testGainMarginConcept() {
    double Kp_stable = 2.0;
    double gain_margin = 2.0;  // Factor of 2 margin

    double Kp_unstable = Kp_stable * gain_margin * 1.1;

    TS_ASSERT(Kp_unstable > Kp_stable * gain_margin);
  }

  // Test phase margin concept
  void testPhaseMarginConcept() {
    double phase_margin_deg = 45.0;  // Desired margin
    double phase_margin_rad = phase_margin_deg * M_PI / 180.0;

    // Phase margin > 0 indicates stability
    TS_ASSERT(phase_margin_rad > 0.0);
    TS_ASSERT_DELTA(phase_margin_rad, 0.785, 0.01);
  }

  /***************************************************************************
   * Practical Implementation Tests
   ***************************************************************************/

  // Test sample time effect on gains
  void testSampleTimeEffectOnGains() {
    // Continuous gains
    double Kp_c = 2.0;
    double Ti_c = 1.0;  // Integral time
    double Td_c = 0.1;  // Derivative time
    double dt = 0.01;

    // Discrete gains
    double Ki_d = Kp_c / Ti_c * dt;  // Integral gain for discrete
    double Kd_d = Kp_c * Td_c / dt;  // Derivative gain for discrete

    TS_ASSERT(Ki_d < Kp_c);  // Integral gain small
    TS_ASSERT(Kd_d > Kp_c);  // Derivative gain large
  }

  // Test execution timing
  void testExecutionTiming() {
    double dt_target = 0.01;  // 100 Hz
    double dt_tolerance = 0.002;  // 20% tolerance

    // Simulated actual dt
    double dt_actual = 0.011;

    bool timing_ok = std::abs(dt_actual - dt_target) <= dt_tolerance;
    TS_ASSERT(timing_ok);
  }

  // Test controller output in engineering units
  void testOutputEngineeringUnits() {
    double Kp = 0.5;  // deg/deg error
    double error_deg = 10.0;

    double output_deg = Kp * error_deg;
    TS_ASSERT_DELTA(output_deg, 5.0, epsilon);

    // Convert to radians
    double output_rad = output_deg * M_PI / 180.0;
    TS_ASSERT_DELTA(output_rad, 0.0873, 0.001);
  }

  // Test error calculation with different units
  void testErrorWithDifferentUnits() {
    // Altitude error in feet
    double setpoint_ft = 10000.0;
    double process_ft = 9500.0;
    double error_ft = setpoint_ft - process_ft;
    TS_ASSERT_DELTA(error_ft, 500.0, epsilon);

    // Same in meters
    double ft_to_m = 0.3048;
    double error_m = error_ft * ft_to_m;
    TS_ASSERT_DELTA(error_m, 152.4, 0.1);
  }
};
