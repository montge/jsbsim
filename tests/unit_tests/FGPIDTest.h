#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <deque>

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
};
