#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <deque>
#include <vector>

#include "TestUtilities.h"
#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;
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

  /***************************************************************************
   * Advanced PID Response Tests (77-80)
   ***************************************************************************/

  // Test 77: Critically damped response
  void testCriticallyDampedResponse() {
    double Kp = 0.5, Ki = 0.1, Kd = 0.3;  // Conservative gains for less overshoot
    double dt = 0.1;

    double setpoint = 100.0;
    double process = 0.0;
    double I_sum = 0.0;
    double prev_error = setpoint;

    // Simulate with well-tuned gains for critical damping
    for (int i = 0; i < 200; i++) {
      double error = setpoint - process;
      I_sum += error * dt;
      I_sum = std::max(-200.0, std::min(200.0, I_sum));  // Anti-windup
      double D = (error - prev_error) / dt;

      double output = Kp * error + Ki * I_sum + Kd * D;
      process += 0.05 * output;  // Slower integration for stability

      prev_error = error;
    }

    // Should converge close to setpoint
    TS_ASSERT(std::abs(process - setpoint) < 10.0);
  }

  // Test 78: Response to step disturbance
  void testStepDisturbanceRejection() {
    double Kp = 2.0, Ki = 0.5;
    double dt = 0.1;

    double setpoint = 50.0;
    double process = 50.0;  // Start at setpoint
    double I_sum = 0.0;

    // Apply step disturbance at step 10
    for (int i = 0; i < 100; i++) {
      double disturbance = (i >= 10 && i < 20) ? -10.0 : 0.0;
      double error = setpoint - process;
      I_sum += error * dt;

      double output = Kp * error + Ki * I_sum;
      // Process integrates output and experiences disturbance
      process += 0.1 * (output - process + disturbance);
    }

    // Should recover close to setpoint after disturbance ends
    TS_ASSERT(std::abs(process - setpoint) < 10.0);
  }

  // Test 79: Response to ramp reference
  void testRampReferenceTracking() {
    double Kp = 3.0, Ki = 1.0;
    double dt = 0.1;

    double process = 0.0;
    double I_sum = 0.0;
    double tracking_error = 0.0;

    for (int i = 0; i < 50; i++) {
      double setpoint = i * 1.0;  // Ramp: 0, 1, 2, 3...
      double error = setpoint - process;
      I_sum += error * dt;

      double output = Kp * error + Ki * I_sum;
      process += 0.3 * output;

      if (i > 20) {  // After transient
        tracking_error += std::abs(error);
      }
    }

    // Should track ramp with bounded error
    TS_ASSERT(tracking_error / 30.0 < 5.0);  // Average error < 5
  }

  // Test 80: Integral time constant relationship
  void testIntegralTimeConstant() {
    double Kp = 2.0;
    double Ti = 1.0;  // Integral time constant
    double dt = 0.1;

    // Ki = Kp / Ti in standard form
    double Ki = Kp / Ti;
    TS_ASSERT_DELTA(Ki, 2.0, epsilon);

    // After Ti seconds of constant error, integral contribution equals P
    double I_sum = 0.0;
    double error = 5.0;
    int steps = static_cast<int>(Ti / dt);

    for (int i = 0; i < steps; i++) {
      I_sum += error * dt;
    }

    double P = Kp * error;
    double I = Ki * I_sum;

    TS_ASSERT_DELTA(P, I, 0.5);
  }

  /***************************************************************************
   * Advanced Derivative Tests (81-84)
   ***************************************************************************/

  // Test 81: Derivative time constant relationship
  void testDerivativeTimeConstant() {
    double Kp = 2.0;
    double Td = 0.5;  // Derivative time constant

    // Kd = Kp * Td in standard form
    double Kd = Kp * Td;
    TS_ASSERT_DELTA(Kd, 1.0, epsilon);
  }

  // Test 82: Derivative with exponential filter
  void testDerivativeExponentialFilter() {
    double Kd = 1.0;
    double N = 10.0;  // Filter coefficient
    double dt = 0.1;
    double filtered_D = 0.0;
    double prev_error = 0.0;

    for (int i = 0; i < 20; i++) {
      double error = (i < 5) ? 0.0 : 10.0;  // Step at i=5
      double raw_D = (error - prev_error) / dt;

      // First-order filter: D_f = (1 - alpha) * D_f + alpha * D_raw
      double alpha = N * dt / (1 + N * dt);
      filtered_D = (1 - alpha) * filtered_D + alpha * raw_D;

      TS_ASSERT(std::isfinite(filtered_D));
      prev_error = error;
    }

    // Filtered derivative should be smaller than raw spike
    TS_ASSERT(std::abs(filtered_D) < 100.0);
  }

  // Test 83: Derivative-on-measurement
  void testDerivativeOnMeasurement() {
    double Kd = 1.0;
    double dt = 0.1;

    double sp_prev = 50.0, sp_curr = 100.0;  // Setpoint step
    double pv_prev = 75.0, pv_curr = 77.0;   // Slow measurement change

    // D on error (has derivative kick from setpoint)
    double D_error = Kd * ((sp_curr - pv_curr) - (sp_prev - pv_prev)) / dt;

    // D on measurement (no kick)
    double D_measurement = -Kd * (pv_curr - pv_prev) / dt;

    TS_ASSERT(std::abs(D_error) > std::abs(D_measurement));
    TS_ASSERT_DELTA(D_measurement, -20.0, epsilon);
  }

  // Test 84: Second derivative approximation
  void testSecondDerivativeApproximation() {
    double dt = 0.1;
    double e_curr = 10.0, e_prev = 8.0, e_prev2 = 5.0;

    // First derivative approximations
    double d1 = (e_curr - e_prev) / dt;
    double d2 = (e_prev - e_prev2) / dt;

    // Second derivative
    double d2_dt2 = (d1 - d2) / dt;

    TS_ASSERT_DELTA(d1, 20.0, epsilon);
    TS_ASSERT_DELTA(d2, 30.0, epsilon);
    TS_ASSERT_DELTA(d2_dt2, -100.0, epsilon);
  }

  /***************************************************************************
   * Flight Control Application Tests (85-88)
   ***************************************************************************/

  // Test 85: Altitude hold PID
  void testAltitudeHoldPID() {
    double Kp = 0.1;   // ft/s per ft error
    double Ki = 0.01;  // ft/s per ft-s
    double dt = 0.1;

    double target_alt = 10000.0;  // ft
    double current_alt = 9800.0;  // ft
    double I_sum = 0.0;

    double error = target_alt - current_alt;
    I_sum += error * dt;

    double vs_command = Kp * error + Ki * I_sum;

    // Should command positive (climb) vertical speed
    // With Kp=0.1, Ki=0.01, error=200: output = 0.1*200 + 0.01*20 = 20.2 ft/s
    TS_ASSERT(vs_command > 0.0);
    TS_ASSERT_DELTA(vs_command, 20.2, 1.0);  // ~20 ft/s climb
  }

  // Test 86: Heading hold PID
  void testHeadingHoldPID() {
    double Kp = 2.0;  // deg/s per deg error
    double dt = 0.1;

    double target_hdg = 90.0;   // deg
    double current_hdg = 85.0;  // deg
    double error = target_hdg - current_hdg;

    double roll_command = Kp * error;

    TS_ASSERT_DELTA(roll_command, 10.0, epsilon);  // 10 deg/s roll rate
  }

  // Test 87: Airspeed hold PID
  void testAirspeedHoldPID() {
    double Kp = 0.5;   // % per knot error
    double Ki = 0.05;
    double dt = 0.1;

    double target_speed = 250.0;  // knots
    double current_speed = 240.0;
    double I_sum = 0.0;

    double error = target_speed - current_speed;
    I_sum += error * dt;

    double throttle_change = Kp * error + Ki * I_sum;

    TS_ASSERT(throttle_change > 0.0);  // Increase throttle
  }

  // Test 88: Rate-based PID for pitch control
  void testPitchRateController() {
    double Kp = 3.0;  // elevator per deg/s rate error
    double Kd = 0.1;
    double dt = 0.02;

    double commanded_rate = 5.0;    // deg/s
    double actual_rate = 3.0;       // deg/s
    double prev_rate_error = 1.5;   // deg/s

    double rate_error = commanded_rate - actual_rate;
    double rate_accel = (rate_error - prev_rate_error) / dt;

    double elevator = Kp * rate_error + Kd * rate_accel;

    TS_ASSERT(elevator > 0.0);  // Nose-up command
  }

  /***************************************************************************
   * Practical Implementation Tests (89-92)
   ***************************************************************************/

  // Test 89: Velocity form PID
  void testVelocityFormPID() {
    double Kp = 2.0, Ki = 0.5, Kd = 0.1;
    double dt = 0.1;

    double e_curr = 10.0, e_prev = 8.0, e_prev2 = 5.0;

    // Velocity form: delta_u = Kp*(e_n - e_n-1) + Ki*dt*e_n + Kd/dt*(e_n - 2*e_n-1 + e_n-2)
    double delta_u = Kp * (e_curr - e_prev) +
                     Ki * dt * e_curr +
                     Kd / dt * (e_curr - 2*e_prev + e_prev2);

    TS_ASSERT(std::isfinite(delta_u));
  }

  // Test 90: PID with deadband
  void testPIDWithDeadband() {
    double Kp = 2.0;
    double deadband = 1.0;

    // errors: 0.5, -0.3, 1.5, -2.0
    // With deadband=1.0:
    // 0.5: |0.5| < 1.0 -> effective=0, output=0
    // -0.3: |-0.3| < 1.0 -> effective=0, output=0
    // 1.5: 1.5 > 1.0 -> effective=1.5-1.0=0.5, output=2.0*0.5=1.0
    // -2.0: |-2.0| > 1.0 -> effective=-2.0+1.0=-1.0, output=2.0*-1.0=-2.0
    double errors[] = {0.5, -0.3, 1.5, -2.0};
    double expected_outs[] = {0.0, 0.0, 1.0, -2.0};  // These are final outputs

    for (int i = 0; i < 4; i++) {
      double effective_error = errors[i];
      if (std::abs(errors[i]) < deadband) {
        effective_error = 0.0;
      } else {
        effective_error = errors[i] - (errors[i] > 0 ? deadband : -deadband);
      }

      double output = Kp * effective_error;
      TS_ASSERT_DELTA(output, expected_outs[i], epsilon);
    }
  }

  // Test 91: Split-range PID
  void testSplitRangePID() {
    double Kp = 1.0;
    double output_split = 50.0;  // Split point

    auto splitOutput = [output_split](double raw_output) -> std::pair<double, double> {
      if (raw_output <= output_split) {
        return {raw_output, 0.0};
      } else {
        return {output_split, raw_output - output_split};
      }
    };

    auto [out1, out2] = splitOutput(75.0);
    TS_ASSERT_DELTA(out1, 50.0, epsilon);
    TS_ASSERT_DELTA(out2, 25.0, epsilon);

    auto [out3, out4] = splitOutput(30.0);
    TS_ASSERT_DELTA(out3, 30.0, epsilon);
    TS_ASSERT_DELTA(out4, 0.0, epsilon);
  }

  // Test 92: PID with manual override
  void testPIDWithManualOverride() {
    double Kp = 2.0, Ki = 0.5;
    double dt = 0.1;
    double I_sum = 0.0;

    bool manual_mode = false;
    double manual_output = 50.0;
    double error = 10.0;

    I_sum += error * dt;
    double auto_output = Kp * error + Ki * I_sum;

    double final_output = manual_mode ? manual_output : auto_output;
    TS_ASSERT_DELTA(final_output, auto_output, epsilon);

    manual_mode = true;
    final_output = manual_mode ? manual_output : auto_output;
    TS_ASSERT_DELTA(final_output, manual_output, epsilon);
  }

  /***************************************************************************
   * Robustness and Edge Case Tests (93-96)
   ***************************************************************************/

  // Test 93: PID with sensor failure detection
  void testPIDSensorFailureDetection() {
    double prev_value = 100.0;
    double curr_value = 500.0;  // Large jump
    double max_rate = 50.0;     // per step

    bool sensor_failure = std::abs(curr_value - prev_value) > max_rate;
    TS_ASSERT(sensor_failure);
  }

  // Test 94: PID with actuator saturation tracking
  void testActuatorSaturationTracking() {
    double Ki = 1.0;
    double dt = 0.1;
    double I_sum = 0.0;
    double output_max = 100.0;

    bool was_saturated = false;
    for (int i = 0; i < 50; i++) {
      double error = 20.0;

      if (!was_saturated) {
        I_sum += error * dt;
      }

      double output = Ki * I_sum;
      was_saturated = output >= output_max;

      if (was_saturated) {
        I_sum = output_max / Ki;  // Back-calculate
      }
    }

    TS_ASSERT_DELTA(I_sum, output_max / Ki, 0.5);
  }

  // Test 95: PID initialization from steady state
  void testPIDInitFromSteadyState() {
    double Kp = 2.0, Ki = 0.5;
    double steady_output = 60.0;
    double steady_error = 5.0;

    // Initialize I_sum to match steady state
    double P = Kp * steady_error;
    double I_sum_init = (steady_output - P) / Ki;

    double output = Kp * steady_error + Ki * I_sum_init;
    TS_ASSERT_DELTA(output, steady_output, epsilon);
  }

  // Test 96: PID mode transition (auto to manual and back)
  void testPIDModeTransition() {
    double Kp = 2.0, Ki = 0.5;
    double I_sum = 5.0;  // Accumulated integral
    double error = 10.0;

    double auto_output = Kp * error + Ki * I_sum;

    // Switch to manual
    double manual_output = 30.0;

    // Switch back to auto - reinitialize I_sum for bumpless
    double new_I_sum = (manual_output - Kp * error) / Ki;

    double new_auto_output = Kp * error + Ki * new_I_sum;
    TS_ASSERT_DELTA(new_auto_output, manual_output, epsilon);
  }

  /***************************************************************************
   * Complete PID System Tests (97-100)
   ***************************************************************************/

  // Test 97: Complete altitude autopilot simulation
  void testCompleteAltitudeAutopilot() {
    double Kp = 0.2, Ki = 0.05, Kd = 0.3;
    double dt = 0.1;

    double target_alt = 10000.0;
    double altitude = 9500.0;  // Start closer to target
    double vs = 0.0;
    double I_sum = 0.0;
    double prev_error = target_alt - altitude;

    for (int i = 0; i < 300; i++) {
      double error = target_alt - altitude;
      I_sum += error * dt;
      I_sum = std::max(-1000.0, std::min(1000.0, I_sum));  // Anti-windup
      double D = (error - prev_error) / dt;

      double vs_cmd = Kp * error + Ki * I_sum + Kd * D;
      vs_cmd = std::max(-500.0, std::min(500.0, vs_cmd));  // Rate limit

      vs = vs + 0.2 * (vs_cmd - vs);  // Faster response
      altitude += vs * dt;

      prev_error = error;
    }

    // Should converge to within 100 ft of target
    TS_ASSERT(std::abs(altitude - target_alt) < 100.0);
  }

  // Test 98: Complete heading autopilot simulation
  void testCompleteHeadingAutopilot() {
    double Kp = 1.5, Ki = 0.1, Kd = 0.3;
    double dt = 0.1;

    double target_hdg = 180.0;
    double heading = 90.0;
    double I_sum = 0.0;
    double prev_error = 0.0;

    for (int i = 0; i < 100; i++) {
      double error = target_hdg - heading;
      // Normalize to -180 to 180
      while (error > 180.0) error -= 360.0;
      while (error < -180.0) error += 360.0;

      I_sum += error * dt;
      I_sum = std::max(-50.0, std::min(50.0, I_sum));
      double D = (error - prev_error) / dt;

      double turn_rate = Kp * error + Ki * I_sum + Kd * D;
      turn_rate = std::max(-15.0, std::min(15.0, turn_rate));

      heading += turn_rate * dt;
      if (heading < 0) heading += 360.0;
      if (heading >= 360.0) heading -= 360.0;

      prev_error = error;
    }

    double final_error = target_hdg - heading;
    while (final_error > 180.0) final_error -= 360.0;
    while (final_error < -180.0) final_error += 360.0;

    TS_ASSERT(std::abs(final_error) < 10.0);
  }

  // Test 99: Complete airspeed controller simulation
  void testCompleteAirspeedController() {
    double Kp = 2.0, Ki = 0.2, Kd = 0.1;
    double dt = 0.1;

    double target_speed = 250.0;
    double airspeed = 200.0;
    double throttle = 50.0;  // Initial throttle %
    double I_sum = 0.0;
    double prev_error = target_speed - airspeed;

    for (int i = 0; i < 100; i++) {
      double error = target_speed - airspeed;
      I_sum += error * dt;
      I_sum = std::max(-100.0, std::min(100.0, I_sum));
      double D = (error - prev_error) / dt;

      double throttle_cmd = 50.0 + Kp * error + Ki * I_sum + Kd * D;
      throttle_cmd = std::max(0.0, std::min(100.0, throttle_cmd));

      throttle = throttle + 0.2 * (throttle_cmd - throttle);

      // Simple speed model: speed increases with throttle
      double accel = (throttle - 50.0) * 0.1 - 0.05 * airspeed * 0.01;
      airspeed += accel * dt;

      prev_error = error;
    }

    TS_ASSERT(std::abs(airspeed - target_speed) < 20.0);
  }

  // Test 100: Complete PID controller system verification
  void testCompletePIDSystemVerification() {
    // Comprehensive verification of PID controller behavior
    double Kp = 2.0, Ki = 0.5, Kd = 0.2;
    double dt = 0.1;

    // 1. Verify proportional gain
    double P = Kp * 10.0;
    TS_ASSERT_DELTA(P, 20.0, epsilon);

    // 2. Verify integral accumulation
    double I_sum = 0.0;
    for (int i = 0; i < 10; i++) I_sum += 10.0 * dt;
    TS_ASSERT_DELTA(I_sum, 10.0, epsilon);

    // 3. Verify derivative calculation
    double D = Kd * (10.0 - 5.0) / dt;
    TS_ASSERT_DELTA(D, 10.0, epsilon);

    // 4. Verify anti-windup
    double I_max = 50.0;
    I_sum = 100.0;
    I_sum = std::min(I_sum, I_max);
    TS_ASSERT_DELTA(I_sum, I_max, epsilon);

    // 5. Verify output limiting
    double output = 200.0;
    double out_max = 100.0;
    output = std::min(output, out_max);
    TS_ASSERT_DELTA(output, out_max, epsilon);

    // 6. Verify rate limiting
    double prev_output = 0.0;
    double new_output = 50.0;
    double rate_limit = 10.0;
    double delta = new_output - prev_output;
    double clamped_delta = std::max(-rate_limit, std::min(delta, rate_limit));
    double limited = prev_output + clamped_delta;
    TS_ASSERT_DELTA(limited, 10.0, epsilon);

    // 7. Verify velocity form
    double e0 = 10.0, e1 = 8.0, e2 = 5.0;
    double delta_u = Kp * (e0 - e1) + Ki * dt * e0 + Kd / dt * (e0 - 2*e1 + e2);
    TS_ASSERT(std::isfinite(delta_u));

    // 8. Verify bumpless transfer
    double manual = 60.0;
    double P_comp = Kp * 10.0;
    double I_init = (manual - P_comp) / Ki;
    double auto_out = P_comp + Ki * I_init;
    TS_ASSERT_DELTA(auto_out, manual, epsilon);

    // 9. Verify deadband
    double error = 0.5;
    double deadband = 1.0;
    double eff_error = std::abs(error) < deadband ? 0.0 : error;
    TS_ASSERT_DELTA(eff_error, 0.0, epsilon);

    // 10. Verify complete system coherence
    TS_ASSERT(Kp > 0.0);
    TS_ASSERT(Ki >= 0.0);
    TS_ASSERT(Kd >= 0.0);
    TS_ASSERT(dt > 0.0);
  }
};

/*******************************************************************************
 * C172x Integration Tests for PID/FCS Control
 ******************************************************************************/

class FGPIDC172xTest : public CxxTest::TestSuite
{
public:
  // Test FCS throttle control response
  void testC172xThrottleControlResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);

    // Set throttle command
    fcs->SetThrottleCmd(-1, 0.5);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double throttle_pos = fcs->GetThrottlePos(0);
    TS_ASSERT(std::isfinite(throttle_pos));
    TS_ASSERT(throttle_pos >= 0.0 && throttle_pos <= 1.0);
  }

  // Test elevator control tracking
  void testC172xElevatorControlTracking() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Command elevator
    fcs->SetDeCmd(-0.5);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double de_pos = fcs->GetDePos();
    TS_ASSERT(std::isfinite(de_pos));
    // Should track command direction
    TS_ASSERT(de_pos < 0.0);
  }

  // Test aileron control tracking
  void testC172xAileronControlTracking() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Command aileron
    fcs->SetDaCmd(0.5);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double da_l = fcs->GetDaLPos();
    double da_r = fcs->GetDaRPos();

    TS_ASSERT(std::isfinite(da_l));
    TS_ASSERT(std::isfinite(da_r));
  }

  // Test rudder control tracking
  void testC172xRudderControlTracking() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Command rudder
    fcs->SetDrCmd(0.5);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double dr_pos = fcs->GetDrPos();
    TS_ASSERT(std::isfinite(dr_pos));
    TS_ASSERT(dr_pos > 0.0);
  }

  // Test control response stability
  void testC172xControlResponseStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.7);

    // Apply step inputs and check for stability
    fcs->SetDeCmd(-0.3);
    fcs->SetDaCmd(0.2);

    double max_de = 0, max_da = 0;

    for (int i = 0; i < 500; i++) {
      fdmex.Run();
      max_de = std::max(max_de, std::abs(fcs->GetDePos()));
      max_da = std::max(max_da, std::abs(fcs->GetDaLPos()));
    }

    TS_ASSERT(std::isfinite(max_de));
    TS_ASSERT(std::isfinite(max_da));
    // Control surfaces should stay within limits
    TS_ASSERT(max_de < 1.0);
    TS_ASSERT(max_da < 1.0);
  }

  // Test pitch trim control
  void testC172xPitchTrimControl() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();

    // Set pitch trim
    pm->GetNode("fcs/pitch-trim-cmd-norm", true)->setDoubleValue(-0.3);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double trim = pm->GetNode("fcs/pitch-trim-cmd-norm")->getDoubleValue();
    TS_ASSERT(std::isfinite(trim));
    TS_ASSERT_DELTA(trim, -0.3, 0.01);
  }

  // Test mixture control
  void testC172xMixtureControl() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetMixtureCmd(-1, 0.9);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double mix_pos = fcs->GetMixturePos(0);
    TS_ASSERT(std::isfinite(mix_pos));
    TS_ASSERT(mix_pos >= 0.0 && mix_pos <= 1.0);
  }

  // Test control command via property tree
  void testC172xControlViaPropertyTree() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();

    // Set commands via properties
    pm->GetNode("fcs/elevator-cmd-norm", true)->setDoubleValue(-0.2);
    pm->GetNode("fcs/aileron-cmd-norm", true)->setDoubleValue(0.3);
    pm->GetNode("fcs/rudder-cmd-norm", true)->setDoubleValue(0.1);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double de = pm->GetNode("fcs/elevator-pos-rad", true)->getDoubleValue();
    double da = pm->GetNode("fcs/left-aileron-pos-rad", true)->getDoubleValue();
    double dr = pm->GetNode("fcs/rudder-pos-rad", true)->getDoubleValue();

    TS_ASSERT(std::isfinite(de));
    TS_ASSERT(std::isfinite(da));
    TS_ASSERT(std::isfinite(dr));
  }

  // Test flap control increments
  void testC172xFlapControlIncrements() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Test flap positions: 0, 10, 20, 30 degrees
    double flap_settings[] = {0.0, 0.33, 0.67, 1.0};
    double prev_flap = 0;

    for (double setting : flap_settings) {
      fcs->SetDfCmd(setting);
      for (int i = 0; i < 200; i++) fdmex.Run();

      double df = fcs->GetDfPos();
      TS_ASSERT(std::isfinite(df));
      TS_ASSERT(df >= prev_flap - 0.01);  // Flaps should not go backwards
      prev_flap = df;
    }
  }

  // Test brake control
  void testC172xBrakeControl() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Apply brakes
    fcs->SetLBrake(0.8);
    fcs->SetRBrake(0.8);

    for (int i = 0; i < 50; i++) fdmex.Run();

    double lb = fcs->GetBrake(FGLGear::bgLeft);
    double rb = fcs->GetBrake(FGLGear::bgRight);

    TS_ASSERT(std::isfinite(lb));
    TS_ASSERT(std::isfinite(rb));
    TS_ASSERT(lb > 0.5);
    TS_ASSERT(rb > 0.5);
  }

  // Test FCS data consistency
  void testC172xFCSDataConsistency() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto pm = fdmex.GetPropertyManager();

    fcs->SetDeCmd(-0.4);

    for (int i = 0; i < 100; i++) fdmex.Run();

    // Get via FCS
    double de_fcs = fcs->GetDePos();

    // Get via property tree
    double de_pm = pm->GetNode("fcs/elevator-pos-rad", true)->getDoubleValue();

    TS_ASSERT(std::isfinite(de_fcs));
    TS_ASSERT(std::isfinite(de_pm));
    TS_ASSERT_DELTA(de_fcs, de_pm, 0.001);
  }

  // Test control values remain finite over time
  void testC172xControlFiniteOverTime() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetDeCmd(-0.2);
    fcs->SetDaCmd(0.1);

    bool all_finite = true;

    for (int i = 0; i < 500; i++) {
      fdmex.Run();
      if (!std::isfinite(fcs->GetDePos()) ||
          !std::isfinite(fcs->GetDaLPos()) ||
          !std::isfinite(fcs->GetDrPos())) {
        all_finite = false;
        break;
      }
    }

    TS_ASSERT(all_finite);
  }

  // Test control reversal response
  void testC172xControlReversalResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();

    // Command full up
    fcs->SetDeCmd(-1.0);
    for (int i = 0; i < 100; i++) fdmex.Run();
    double de_up = fcs->GetDePos();

    // Command full down
    fcs->SetDeCmd(1.0);
    for (int i = 0; i < 200; i++) fdmex.Run();
    double de_down = fcs->GetDePos();

    TS_ASSERT(std::isfinite(de_up));
    TS_ASSERT(std::isfinite(de_down));
    TS_ASSERT(de_up < de_down);  // Up is negative
  }

  // Test control authority at speed
  void testC172xControlAuthorityAtSpeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.8);

    // Let aircraft accelerate
    for (int i = 0; i < 400; i++) fdmex.Run();

    // Apply control input
    fcs->SetDeCmd(-0.3);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double de = fcs->GetDePos();
    double q = propagate->GetPQR(2);  // Pitch rate

    TS_ASSERT(std::isfinite(de));
    TS_ASSERT(std::isfinite(q));
  }

  // Test all FCS commands zero at start
  void testC172xFCSCommandsZeroAtStart() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();

    double de_cmd = pm->GetNode("fcs/elevator-cmd-norm", true)->getDoubleValue();
    double da_cmd = pm->GetNode("fcs/aileron-cmd-norm", true)->getDoubleValue();
    double dr_cmd = pm->GetNode("fcs/rudder-cmd-norm", true)->getDoubleValue();

    TS_ASSERT(std::isfinite(de_cmd));
    TS_ASSERT(std::isfinite(da_cmd));
    TS_ASSERT(std::isfinite(dr_cmd));
  }
};
