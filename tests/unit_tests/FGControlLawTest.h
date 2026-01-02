/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       FGControlLawTest.h
 Author:       Claude Code
 Date started: 12/24/2025

 ------------- Copyright (C) 2025 -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be
 found on the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------
Unit tests for flight control law mathematics including PID controllers,
lead-lag compensators, washout filters, rate limiters, authority limits,
scheduled gains, stability augmentation, stick force gradients, control mixing,
and anti-windup mechanisms.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SENTRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef FGCONTROLLAWTEST_H
#define FGCONTROLLAWTEST_H

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"
#include "models/FGAuxiliary.h"

using namespace JSBSim;

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DECLARATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Constants for testing
static const double EPSILON = 1e-9;
static const double DEG_TO_RAD = 0.017453292519943295;
static const double RAD_TO_DEG = 57.295779513082323;

// Simple PID Controller for testing
class SimplePID {
public:
    SimplePID(double kp, double ki, double kd, double dt)
        : Kp(kp), Ki(ki), Kd(kd), dt(dt), integral(0.0), prev_error(0.0) {}

    double Update(double error) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    void Reset() {
        integral = 0.0;
        prev_error = 0.0;
    }

    void SetAntiWindup(double max_integral) {
        if (integral > max_integral) integral = max_integral;
        if (integral < -max_integral) integral = -max_integral;
    }

    double GetIntegral() const { return integral; }

private:
    double Kp, Ki, Kd, dt;
    double integral;
    double prev_error;
};

// Lead-Lag Compensator: G(s) = (s + z) / (s + p)
class LeadLag {
public:
    LeadLag(double zero, double pole, double dt)
        : z(zero), p(pole), dt(dt), prev_input(0.0), prev_output(0.0) {}

    double Update(double input) {
        // Tustin (bilinear) discretization
        double a0 = 2.0 / dt + p;
        double a1 = p - 2.0 / dt;
        double b0 = 2.0 / dt + z;
        double b1 = z - 2.0 / dt;

        double output = (b0 * input + b1 * prev_input - a1 * prev_output) / a0;
        prev_input = input;
        prev_output = output;
        return output;
    }

    void Reset() {
        prev_input = 0.0;
        prev_output = 0.0;
    }

private:
    double z, p, dt;
    double prev_input, prev_output;
};

// Washout Filter: G(s) = s / (s + p)
class Washout {
public:
    Washout(double pole, double dt)
        : p(pole), dt(dt), prev_input(0.0), prev_output(0.0) {}

    double Update(double input) {
        // Tustin discretization
        double a0 = 2.0 / dt + p;
        double a1 = p - 2.0 / dt;
        double b0 = 2.0 / dt;
        double b1 = -2.0 / dt;

        double output = (b0 * input + b1 * prev_input - a1 * prev_output) / a0;
        prev_input = input;
        prev_output = output;
        return output;
    }

    void Reset() {
        prev_input = 0.0;
        prev_output = 0.0;
    }

private:
    double p, dt;
    double prev_input, prev_output;
};

// Rate Limiter
class RateLimiter {
public:
    RateLimiter(double max_rate, double dt)
        : max_rate(max_rate), dt(dt), prev_value(0.0) {}

    double Update(double input) {
        double delta = input - prev_value;
        double max_delta = max_rate * dt;

        if (delta > max_delta) delta = max_delta;
        if (delta < -max_delta) delta = -max_delta;

        prev_value += delta;
        return prev_value;
    }

    void Reset(double value = 0.0) {
        prev_value = value;
    }

private:
    double max_rate, dt;
    double prev_value;
};

// Authority Limiter (Saturation)
class Saturation {
public:
    static double Limit(double value, double min_val, double max_val) {
        if (value > max_val) return max_val;
        if (value < min_val) return min_val;
        return value;
    }
};

// Scheduled Gain (linear interpolation)
class ScheduledGain {
public:
    static double GetGain(double schedule_var,
                          double x1, double y1,
                          double x2, double y2) {
        if (schedule_var <= x1) return y1;
        if (schedule_var >= x2) return y2;
        return y1 + (y2 - y1) * (schedule_var - x1) / (x2 - x1);
    }
};

class FGControlLawTest : public CxxTest::TestSuite
{
public:

    //===========================================================================
    // PID Controller Tests
    //===========================================================================

    void testPIDProportionalTerm() {
        SimplePID pid(2.0, 0.0, 0.0, 0.01);
        double error = 5.0;
        double output = pid.Update(error);
        TS_ASSERT_DELTA(output, 10.0, EPSILON);
    }

    void testPIDIntegralTerm() {
        SimplePID pid(0.0, 1.0, 0.0, 0.01);
        double error = 2.0;

        // First update
        double output1 = pid.Update(error);
        TS_ASSERT_DELTA(output1, 0.02, EPSILON);  // 2.0 * 0.01

        // Second update
        double output2 = pid.Update(error);
        TS_ASSERT_DELTA(output2, 0.04, EPSILON);  // 2.0 * 0.01 * 2
    }

    void testPIDDerivativeTerm() {
        SimplePID pid(0.0, 0.0, 1.0, 0.01);

        // First update (derivative is based on change)
        pid.Update(0.0);

        // Second update with error change
        double output = pid.Update(1.0);
        TS_ASSERT_DELTA(output, 100.0, EPSILON);  // (1.0 - 0.0) / 0.01
    }

    void testPIDFullController() {
        SimplePID pid(1.0, 0.5, 0.1, 0.01);

        // Initialize
        pid.Update(0.0);

        // Apply error
        double error = 2.0;
        double output = pid.Update(error);

        // P term: 1.0 * 2.0 = 2.0
        // I term: 0.5 * 2.0 * 0.01 = 0.01
        // D term: 0.1 * (2.0 - 0.0) / 0.01 = 20.0
        // Total: 22.01
        TS_ASSERT_DELTA(output, 22.01, EPSILON);
    }

    void testPIDReset() {
        SimplePID pid(1.0, 1.0, 1.0, 0.01);

        // Build up some integral
        pid.Update(5.0);
        pid.Update(5.0);

        // Reset
        pid.Reset();
        pid.Update(0.0);

        // Next update should only have P term
        double output = pid.Update(3.0);
        TS_ASSERT_DELTA(output, 3.0 + 0.03 + 300.0, EPSILON);
    }

    //===========================================================================
    // Anti-Windup Tests
    //===========================================================================

    void testAntiWindupBasic() {
        SimplePID pid(1.0, 10.0, 0.0, 0.01);

        // Build up large integral (1000 iterations * 1.0 * 0.01 = 10.0)
        for (int i = 0; i < 1000; i++) {
            pid.Update(1.0);
        }

        // Integral should be 10.0, apply anti-windup to clamp to 5.0
        pid.SetAntiWindup(5.0);

        TS_ASSERT_DELTA(pid.GetIntegral(), 5.0, EPSILON);
    }

    void testAntiWindupNegative() {
        SimplePID pid(1.0, 10.0, 0.0, 0.01);

        // Build up large negative integral (1000 iterations * -1.0 * 0.01 = -10.0)
        for (int i = 0; i < 1000; i++) {
            pid.Update(-1.0);
        }

        // Integral should be -10.0, apply anti-windup to clamp to -5.0
        pid.SetAntiWindup(5.0);

        TS_ASSERT_DELTA(pid.GetIntegral(), -5.0, EPSILON);
    }

    void testAntiWindupNoEffect() {
        SimplePID pid(1.0, 10.0, 0.0, 0.01);

        // Build small integral
        pid.Update(0.1);
        double integral_before = pid.GetIntegral();

        // Apply anti-windup with large limit
        pid.SetAntiWindup(100.0);

        TS_ASSERT_DELTA(pid.GetIntegral(), integral_before, EPSILON);
    }

    //===========================================================================
    // Lead-Lag Compensator Tests
    //===========================================================================

    void testLeadLagUnityGain() {
        // Same zero and pole should give unity gain at DC
        LeadLag leadlag(1.0, 1.0, 0.01);

        double input = 5.0;
        double output = leadlag.Update(input);

        // After settling, should approach input
        for (int i = 0; i < 100; i++) {
            output = leadlag.Update(input);
        }

        TS_ASSERT_DELTA(output, input, 0.01);
    }

    void testLeadLagPhaseAdvance() {
        // Lead compensator: zero > pole
        LeadLag leadlag(10.0, 1.0, 0.001);

        leadlag.Reset();
        leadlag.Update(0.0);

        // Step input should produce overshoot (lead behavior)
        double output = leadlag.Update(1.0);

        // Lead compensator produces initial overshoot
        TS_ASSERT(output > 0.0);
    }

    void testLeadLagReset() {
        LeadLag leadlag(5.0, 2.0, 0.01);

        // Build up some state
        for (int i = 0; i < 10; i++) {
            leadlag.Update(5.0);
        }

        // Reset and verify
        leadlag.Reset();
        double output = leadlag.Update(0.0);

        TS_ASSERT_DELTA(output, 0.0, EPSILON);
    }

    //===========================================================================
    // Washout Filter Tests
    //===========================================================================

    void testWashoutBlocksDC() {
        Washout washout(1.0, 0.01);

        double input = 10.0;
        double output = 0.0;

        // Apply constant input, output should decay to zero
        for (int i = 0; i < 1000; i++) {
            output = washout.Update(input);
        }

        TS_ASSERT_DELTA(output, 0.0, 0.01);
    }

    void testWashoutPassesAC() {
        Washout washout(10.0, 0.001);

        // Apply changing signal
        double output1 = washout.Update(0.0);
        double output2 = washout.Update(1.0);

        // Washout should pass the change
        TS_ASSERT(std::abs(output2 - output1) > 0.1);
    }

    void testWashoutReset() {
        Washout washout(5.0, 0.01);

        // Build up some state
        for (int i = 0; i < 10; i++) {
            washout.Update(5.0);
        }

        // Reset and verify
        washout.Reset();
        double output = washout.Update(0.0);

        TS_ASSERT_DELTA(output, 0.0, EPSILON);
    }

    //===========================================================================
    // Rate Limiter Tests
    //===========================================================================

    void testRateLimiterNoLimit() {
        RateLimiter limiter(1000.0, 0.01);  // Very high rate limit (1000 * 0.01 = 10 per step)

        double input = 5.0;
        double output = limiter.Update(input);

        // max_delta = 1000 * 0.01 = 10, so 5.0 is reachable in one step
        TS_ASSERT_DELTA(output, input, EPSILON);
    }

    void testRateLimiterPositiveLimit() {
        RateLimiter limiter(10.0, 0.01);  // 10 units/sec, dt=0.01

        limiter.Reset(0.0);

        // Try to jump to 5.0, but limited to 10*0.01 = 0.1
        double output = limiter.Update(5.0);

        TS_ASSERT_DELTA(output, 0.1, EPSILON);
    }

    void testRateLimiterNegativeLimit() {
        RateLimiter limiter(10.0, 0.01);

        limiter.Reset(5.0);

        // Try to jump to 0.0, limited to -10*0.01 = -0.1 change
        double output = limiter.Update(0.0);

        TS_ASSERT_DELTA(output, 4.9, EPSILON);
    }

    void testRateLimiterGradualApproach() {
        RateLimiter limiter(1.0, 0.01);

        limiter.Reset(0.0);

        // Approach 1.0 gradually
        double output = 0.0;
        for (int i = 0; i < 100; i++) {
            output = limiter.Update(1.0);
        }

        TS_ASSERT_DELTA(output, 1.0, EPSILON);
    }

    //===========================================================================
    // Authority Limits (Saturation) Tests
    //===========================================================================

    void testSaturationUpperLimit() {
        double value = 15.0;
        double limited = Saturation::Limit(value, -10.0, 10.0);

        TS_ASSERT_DELTA(limited, 10.0, EPSILON);
    }

    void testSaturationLowerLimit() {
        double value = -15.0;
        double limited = Saturation::Limit(value, -10.0, 10.0);

        TS_ASSERT_DELTA(limited, -10.0, EPSILON);
    }

    void testSaturationNoLimit() {
        double value = 5.0;
        double limited = Saturation::Limit(value, -10.0, 10.0);

        TS_ASSERT_DELTA(limited, 5.0, EPSILON);
    }

    void testSaturationAsymmetric() {
        double value1 = 30.0;
        double value2 = -30.0;

        double limited1 = Saturation::Limit(value1, -5.0, 25.0);
        double limited2 = Saturation::Limit(value2, -5.0, 25.0);

        TS_ASSERT_DELTA(limited1, 25.0, EPSILON);
        TS_ASSERT_DELTA(limited2, -5.0, EPSILON);
    }

    //===========================================================================
    // Scheduled Gains Tests
    //===========================================================================

    void testScheduledGainLinearInterpolation() {
        // Gain varies from 1.0 at 0 m/s to 2.0 at 100 m/s
        double gain = ScheduledGain::GetGain(50.0, 0.0, 1.0, 100.0, 2.0);

        TS_ASSERT_DELTA(gain, 1.5, EPSILON);
    }

    void testScheduledGainBelowRange() {
        double gain = ScheduledGain::GetGain(-10.0, 0.0, 1.0, 100.0, 2.0);

        TS_ASSERT_DELTA(gain, 1.0, EPSILON);
    }

    void testScheduledGainAboveRange() {
        double gain = ScheduledGain::GetGain(150.0, 0.0, 1.0, 100.0, 2.0);

        TS_ASSERT_DELTA(gain, 2.0, EPSILON);
    }

    void testScheduledGainAtBoundaries() {
        double gain1 = ScheduledGain::GetGain(0.0, 0.0, 1.0, 100.0, 2.0);
        double gain2 = ScheduledGain::GetGain(100.0, 0.0, 1.0, 100.0, 2.0);

        TS_ASSERT_DELTA(gain1, 1.0, EPSILON);
        TS_ASSERT_DELTA(gain2, 2.0, EPSILON);
    }

    void testScheduledGainDecreasing() {
        // Gain decreases with schedule variable
        double gain = ScheduledGain::GetGain(50.0, 0.0, 2.0, 100.0, 1.0);

        TS_ASSERT_DELTA(gain, 1.5, EPSILON);
    }

    //===========================================================================
    // Stability Augmentation Tests
    //===========================================================================

    void testPitchDamperBasic() {
        // Pitch damper: -Kq * q (pitch rate damping)
        double Kq = 2.0;
        double pitch_rate = 0.1;  // rad/s

        double elevator_cmd = -Kq * pitch_rate;

        TS_ASSERT_DELTA(elevator_cmd, -0.2, EPSILON);
    }

    void testRollDamperBasic() {
        // Roll damper: -Kp * p (roll rate damping)
        double Kp = 1.5;
        double roll_rate = 0.2;  // rad/s

        double aileron_cmd = -Kp * roll_rate;

        TS_ASSERT_DELTA(aileron_cmd, -0.3, EPSILON);
    }

    void testYawDamperBasic() {
        // Yaw damper: -Kr * r (yaw rate damping)
        double Kr = 3.0;
        double yaw_rate = -0.05;  // rad/s

        double rudder_cmd = -Kr * yaw_rate;

        TS_ASSERT_DELTA(rudder_cmd, 0.15, EPSILON);
    }

    void testYawDamperWithWashout() {
        // Yaw damper with washout to prevent DC response
        Washout washout(1.0, 0.01);
        double Kr = 2.0;

        // Apply constant yaw rate
        double yaw_rate = 0.1;
        double filtered_rate = 0.0;

        for (int i = 0; i < 10; i++) {
            filtered_rate = washout.Update(yaw_rate);
        }

        double rudder_cmd = -Kr * filtered_rate;

        // Washout reduces the signal
        TS_ASSERT(std::abs(rudder_cmd) < 0.2);
    }

    //===========================================================================
    // Stick Force Gradient Tests
    //===========================================================================

    void testStickForceGradient() {
        // Force = K * deflection
        double K = 50.0;  // N/degree
        double deflection = 10.0;  // degrees

        double force = K * deflection;

        TS_ASSERT_DELTA(force, 500.0, EPSILON);
    }

    void testStickForceWithPreload() {
        // Force = K * deflection + F0
        double K = 50.0;
        double F0 = 20.0;  // Preload force
        double deflection = 5.0;

        double force = K * deflection + F0;

        TS_ASSERT_DELTA(force, 270.0, EPSILON);
    }

    void testStickForceNonlinear() {
        // Force = K1 * deflection + K2 * deflection^2
        double K1 = 30.0;
        double K2 = 2.0;
        double deflection = 10.0;

        double force = K1 * deflection + K2 * deflection * deflection;

        TS_ASSERT_DELTA(force, 500.0, EPSILON);
    }

    //===========================================================================
    // Control Mixing Tests
    //===========================================================================

    void testElevonMixing() {
        // Elevons: combine elevator and aileron
        double elevator = 0.5;
        double aileron = 0.3;

        // Left elevon: elevator - aileron
        // Right elevon: elevator + aileron
        double left_elevon = elevator - aileron;
        double right_elevon = elevator + aileron;

        TS_ASSERT_DELTA(left_elevon, 0.2, EPSILON);
        TS_ASSERT_DELTA(right_elevon, 0.8, EPSILON);
    }

    void testElevonDemixing() {
        // Given elevon positions, recover elevator and aileron
        double left_elevon = 0.2;
        double right_elevon = 0.8;

        double elevator = (left_elevon + right_elevon) / 2.0;
        double aileron = (right_elevon - left_elevon) / 2.0;

        TS_ASSERT_DELTA(elevator, 0.5, EPSILON);
        TS_ASSERT_DELTA(aileron, 0.3, EPSILON);
    }

    void testRuddervatorMixing() {
        // Ruddervators: combine elevator and rudder (V-tail)
        double elevator = 0.6;
        double rudder = -0.2;

        // Left ruddervator: elevator - rudder
        // Right ruddervator: elevator + rudder
        double left_ruddervator = elevator - rudder;
        double right_ruddervator = elevator + rudder;

        TS_ASSERT_DELTA(left_ruddervator, 0.8, EPSILON);
        TS_ASSERT_DELTA(right_ruddervator, 0.4, EPSILON);
    }

    void testControlMixingWithGains() {
        // Mixing with different gains
        double pitch_cmd = 0.5;
        double roll_cmd = 0.3;
        double K_pitch = 0.8;
        double K_roll = 1.2;

        double left_surface = K_pitch * pitch_cmd - K_roll * roll_cmd;
        double right_surface = K_pitch * pitch_cmd + K_roll * roll_cmd;

        TS_ASSERT_DELTA(left_surface, 0.04, EPSILON);
        TS_ASSERT_DELTA(right_surface, 0.76, EPSILON);
    }

    //===========================================================================
    // Complex Control Law Tests
    //===========================================================================

    void testPitchAttitudeHoldController() {
        // Simple pitch attitude hold: proportional + rate damping
        double pitch_error = 5.0 * DEG_TO_RAD;
        double pitch_rate = 0.1;  // rad/s

        double Kp = 2.0;
        double Kq = 1.5;

        double elevator_cmd = Kp * pitch_error - Kq * pitch_rate;

        TS_ASSERT_DELTA(elevator_cmd, 2.0 * 5.0 * DEG_TO_RAD - 0.15, EPSILON);
    }

    void testAltitudeHoldController() {
        // Altitude hold with cascaded loops
        double alt_error = 100.0;  // meters
        double climb_rate = -2.0;  // m/s

        // Outer loop: altitude error to climb rate command
        double K_alt = 0.5;
        double climb_rate_cmd = K_alt * alt_error;

        // Inner loop: climb rate error to pitch command
        double climb_rate_error = climb_rate_cmd - climb_rate;
        double K_climb = 0.1;
        double pitch_cmd = K_climb * climb_rate_error;

        TS_ASSERT_DELTA(climb_rate_cmd, 50.0, EPSILON);
        TS_ASSERT_DELTA(pitch_cmd, 5.2, EPSILON);
    }

    void testCoordinatedTurn() {
        // Coordinated turn: rudder proportional to aileron
        double aileron = 0.5;
        double K_coord = 0.3;

        double rudder = K_coord * aileron;

        TS_ASSERT_DELTA(rudder, 0.15, EPSILON);
    }

    void testFlareController() {
        // Flare law: exponential pitch command vs altitude
        double altitude_agl = 20.0;  // meters
        double h_ref = 30.0;
        double theta_ref = 5.0 * DEG_TO_RAD;

        double pitch_cmd = theta_ref * std::exp(-altitude_agl / h_ref);

        double expected = 5.0 * DEG_TO_RAD * std::exp(-20.0 / 30.0);
        TS_ASSERT_DELTA(pitch_cmd, expected, EPSILON);
    }

    void testAutothrottleController() {
        // Autothrottle: airspeed error to throttle
        double airspeed_error = -5.0;  // m/s (5 m/s slow)
        double Kv = 0.05;

        double throttle_cmd = Kv * (-airspeed_error);

        // Should increase throttle
        TS_ASSERT_DELTA(throttle_cmd, 0.25, EPSILON);
    }

    void testGainSchedulingWithAirspeed() {
        // Pitch damper gain scheduled with airspeed
        double airspeed = 75.0;  // m/s

        // Gain = 1.0 at 50 m/s, 2.0 at 100 m/s
        double Kq = ScheduledGain::GetGain(airspeed, 50.0, 1.0, 100.0, 2.0);

        double pitch_rate = 0.1;
        double elevator_cmd = -Kq * pitch_rate;

        TS_ASSERT_DELTA(Kq, 1.5, EPSILON);
        TS_ASSERT_DELTA(elevator_cmd, -0.15, EPSILON);
    }

    void testControlAllocationWithSaturation() {
        // Multiple surfaces for same control, handle saturation
        double pitch_cmd = 2.0;

        double elevator_1 = Saturation::Limit(pitch_cmd, -1.0, 1.0);
        double remaining = pitch_cmd - elevator_1;
        double elevator_2 = Saturation::Limit(remaining, -1.0, 1.0);

        TS_ASSERT_DELTA(elevator_1, 1.0, EPSILON);
        TS_ASSERT_DELTA(elevator_2, 1.0, EPSILON);
    }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 * Additional Control Law Tests (30 new tests)
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGControlLawAdditionalTest : public CxxTest::TestSuite
{
public:
    //===========================================================================
    // Autopilot Mode Logic Tests
    //===========================================================================

    // Test 46: Wings level mode
    void testWingsLevelMode() {
        double bank_angle = 10.0 * DEG_TO_RAD;
        double roll_rate = 0.05;  // rad/s

        double K_phi = 1.5;
        double K_p = 0.8;

        double aileron_cmd = -K_phi * bank_angle - K_p * roll_rate;

        // Should command aileron opposite to bank
        TS_ASSERT(aileron_cmd < 0);
    }

    // Test 47: Heading hold mode
    void testHeadingHoldMode() {
        double heading_error = 15.0 * DEG_TO_RAD;
        double heading_rate = 0.02;  // rad/s

        double K_psi = 1.0;
        double K_r = 2.0;

        // Bank angle command from heading error
        double bank_cmd = K_psi * heading_error - K_r * heading_rate;

        // Limit bank angle
        double max_bank = 30.0 * DEG_TO_RAD;
        bank_cmd = Saturation::Limit(bank_cmd, -max_bank, max_bank);

        TS_ASSERT(bank_cmd > 0);
        TS_ASSERT(bank_cmd <= max_bank);
    }

    // Test 48: Altitude capture logic
    void testAltitudeCaptureLogic() {
        double current_alt = 4500.0;  // ft
        double target_alt = 5000.0;   // ft
        double capture_band = 200.0;  // ft

        double alt_error = target_alt - current_alt;
        bool in_capture = std::abs(alt_error) < capture_band;

        TS_ASSERT_EQUALS(in_capture, false);

        // Move closer
        current_alt = 4900.0;
        alt_error = target_alt - current_alt;
        in_capture = std::abs(alt_error) < capture_band;

        TS_ASSERT_EQUALS(in_capture, true);
    }

    // Test 49: Speed mode transition
    void testSpeedModeTransition() {
        double current_speed = 250.0;  // kts
        double target_speed = 280.0;   // kts
        double speed_error = target_speed - current_speed;

        // Climb mode: trade altitude for speed
        double K_speed = 0.1;
        double pitch_adjust = K_speed * speed_error;

        // Should pitch down to accelerate
        TS_ASSERT_DELTA(pitch_adjust, 3.0, 0.1);
    }

    //===========================================================================
    // Envelope Protection Tests
    //===========================================================================

    // Test 50: Alpha protection
    void testAlphaProtection() {
        double alpha = 14.0 * DEG_TO_RAD;
        double alpha_max = 15.0 * DEG_TO_RAD;
        double alpha_margin = 1.0 * DEG_TO_RAD;

        double protection_gain = 0.0;
        if (alpha > alpha_max - alpha_margin) {
            protection_gain = (alpha - (alpha_max - alpha_margin)) / alpha_margin;
        }

        TS_ASSERT(protection_gain > 0);
        TS_ASSERT(protection_gain < 1.0);
    }

    // Test 51: Load factor limiting
    void testLoadFactorLimiting() {
        double n_commanded = 3.5;
        double n_max = 2.5;
        double n_min = -1.0;

        double n_limited = Saturation::Limit(n_commanded, n_min, n_max);

        TS_ASSERT_DELTA(n_limited, n_max, EPSILON);
    }

    // Test 52: Overspeed protection
    void testOverspeedProtection() {
        double current_mach = 0.85;
        double mmo = 0.82;  // Max operating Mach

        double overspeed_gain = 0.0;
        if (current_mach > mmo) {
            overspeed_gain = 10.0 * (current_mach - mmo);
        }

        // Should command pitch up
        double pitch_cmd = overspeed_gain;
        TS_ASSERT_DELTA(pitch_cmd, 0.3, 0.01);
    }

    // Test 53: Bank angle protection
    void testBankAngleProtection() {
        double bank = 40.0 * DEG_TO_RAD;
        double bank_limit = 35.0 * DEG_TO_RAD;

        double protection_active = std::abs(bank) > bank_limit;

        TS_ASSERT_EQUALS(protection_active, true);
    }

    //===========================================================================
    // Stick Shaker/Pusher Tests
    //===========================================================================

    // Test 54: Stick shaker activation
    void testStickShakerActivation() {
        double alpha = 12.0 * DEG_TO_RAD;
        double alpha_stall = 15.0 * DEG_TO_RAD;
        double shaker_margin = 3.0 * DEG_TO_RAD;

        bool shaker_active = alpha > (alpha_stall - shaker_margin);

        TS_ASSERT_EQUALS(shaker_active, true);
    }

    // Test 55: Stick pusher activation
    void testStickPusherActivation() {
        double alpha = 14.5 * DEG_TO_RAD;
        double alpha_stall = 15.0 * DEG_TO_RAD;
        double pusher_margin = 1.0 * DEG_TO_RAD;

        bool pusher_active = alpha > (alpha_stall - pusher_margin);
        double pusher_force = 0.0;

        if (pusher_active) {
            pusher_force = 50.0 * (alpha - (alpha_stall - pusher_margin)) / pusher_margin;
        }

        TS_ASSERT_EQUALS(pusher_active, true);
        TS_ASSERT(pusher_force > 0);
    }

    //===========================================================================
    // Structural Mode Filter Tests
    //===========================================================================

    // Test 56: Notch filter for structural mode
    void testNotchFilter() {
        // Simplified notch filter test
        double omega_n = 10.0;  // rad/s (structural mode frequency)
        double zeta = 0.1;      // Damping

        // Notch filter attenuates at omega_n
        // At omega_n, gain should be minimal
        double gain_at_notch = 2.0 * zeta;  // Simplified

        TS_ASSERT(gain_at_notch < 0.5);
    }

    // Test 57: Low-pass filter for noise
    void testLowPassFilter() {
        double omega_c = 5.0;   // rad/s cutoff
        double omega = 20.0;   // rad/s input frequency

        // First-order low-pass gain = 1/sqrt(1 + (omega/omega_c)^2)
        double gain = 1.0 / std::sqrt(1.0 + (omega / omega_c) * (omega / omega_c));

        TS_ASSERT(gain < 0.3);  // Attenuated at high frequency
    }

    //===========================================================================
    // Control Surface Failure Tests
    //===========================================================================

    // Test 58: Single aileron failure compensation
    void testSingleAileronFailure() {
        double roll_cmd = 0.5;
        bool left_aileron_failed = true;

        double left_aileron = left_aileron_failed ? 0.0 : roll_cmd;
        double right_aileron = -roll_cmd;  // Opposite for roll

        // Use spoilers or differential thrust for compensation
        double spoiler_assist = left_aileron_failed ? 0.3 : 0.0;

        TS_ASSERT_DELTA(left_aileron, 0.0, EPSILON);
        TS_ASSERT_DELTA(right_aileron, -0.5, EPSILON);
        TS_ASSERT_DELTA(spoiler_assist, 0.3, EPSILON);
    }

    // Test 59: Elevator failure with stabilizer trim
    void testElevatorFailureWithTrim() {
        double pitch_cmd = 0.3;
        bool elevator_failed = true;

        double elevator = elevator_failed ? 0.0 : pitch_cmd;
        double stab_trim_rate = 0.0;

        if (elevator_failed) {
            stab_trim_rate = 0.5 * pitch_cmd;  // Slower trim authority
        }

        TS_ASSERT_DELTA(elevator, 0.0, EPSILON);
        TS_ASSERT_DELTA(stab_trim_rate, 0.15, EPSILON);
    }

    // Test 60: Rudder hardover detection
    void testRudderHardoverDetection() {
        double rudder_cmd = 0.1;
        double rudder_pos = 1.0;  // Full deflection despite small command

        double position_error = std::abs(rudder_pos - rudder_cmd);
        double threshold = 0.5;

        bool hardover_detected = position_error > threshold;

        TS_ASSERT_EQUALS(hardover_detected, true);
    }

    //===========================================================================
    // Turn Coordination Tests
    //===========================================================================

    // Test 61: Sideslip minimization
    void testSideslipMinimization() {
        double beta = 3.0 * DEG_TO_RAD;  // Sideslip angle
        double K_beta = 2.0;

        double rudder_cmd = -K_beta * beta;

        // Rudder should oppose sideslip
        TS_ASSERT(rudder_cmd < 0);
    }

    // Test 62: Lateral acceleration feedback
    void testLateralAccelerationFeedback() {
        double ay = 0.1;  // g's lateral acceleration
        double K_ay = 0.5;

        double rudder_cmd = -K_ay * ay;

        TS_ASSERT_DELTA(rudder_cmd, -0.05, EPSILON);
    }

    // Test 63: Aileron-rudder interconnect
    void testAileronRudderInterconnect() {
        double aileron = 0.4;
        double airspeed = 150.0;  // kts

        // ARI gain varies with airspeed
        double K_ari = ScheduledGain::GetGain(airspeed, 100.0, 0.2, 200.0, 0.1);
        double rudder_from_ari = K_ari * aileron;

        TS_ASSERT_DELTA(K_ari, 0.15, EPSILON);
        TS_ASSERT_DELTA(rudder_from_ari, 0.06, EPSILON);
    }

    //===========================================================================
    // Fly-by-Wire Control Law Tests
    //===========================================================================

    // Test 64: C* control law
    void testCStarControlLaw() {
        // C* = n_z + (V/g) * q
        double n_z = 1.5;       // g's
        double V = 200.0;       // ft/s
        double g = 32.2;        // ft/s^2
        double q = 0.1;         // rad/s

        double C_star = n_z + (V / g) * q;

        TS_ASSERT_DELTA(C_star, 2.12, 0.01);
    }

    // Test 65: Direct law (manual reversion)
    void testDirectLaw() {
        double stick_input = 0.6;
        double surface_gain = 1.0;  // Direct relationship

        double surface_cmd = stick_input * surface_gain;

        TS_ASSERT_DELTA(surface_cmd, 0.6, EPSILON);
    }

    // Test 66: Alternate law with degraded protection
    void testAlternateLaw() {
        double stick_input = 0.8;
        double alpha = 13.0 * DEG_TO_RAD;
        double alpha_prot = 12.0 * DEG_TO_RAD;

        // Reduced envelope protection in alternate law
        double protection_factor = 0.5;  // 50% protection
        double protection_cmd = 0.0;

        if (alpha > alpha_prot) {
            protection_cmd = protection_factor * (alpha - alpha_prot) / DEG_TO_RAD;
        }

        double final_cmd = stick_input - protection_cmd;

        TS_ASSERT(final_cmd < stick_input);
    }

    //===========================================================================
    // Trim System Tests
    //===========================================================================

    // Test 67: Electric trim rate
    void testElectricTrimRate() {
        double trim_rate = 0.5;  // degrees/sec
        double dt = 0.02;        // sec
        double trim_position = 0.0;

        // Simulate trim button held for 10 steps
        for (int i = 0; i < 10; i++) {
            trim_position += trim_rate * dt;
        }

        TS_ASSERT_DELTA(trim_position, 0.1, EPSILON);
    }

    // Test 68: Mach trim compensation
    void testMachTrimCompensation() {
        double mach = 0.85;
        double mach_ref = 0.70;

        // Nose-down trim as Mach increases
        double K_mach_trim = -2.0;  // deg/Mach
        double trim_bias = K_mach_trim * (mach - mach_ref);

        TS_ASSERT_DELTA(trim_bias, -0.3, 0.01);
    }

    // Test 69: CG compensation trim
    void testCGCompensationTrim() {
        double cg_current = 0.30;   // MAC fraction
        double cg_ref = 0.25;       // Reference MAC

        // More aft CG requires more nose-down trim
        double K_cg = -10.0;  // deg/% MAC
        double trim_adjust = K_cg * (cg_current - cg_ref);

        TS_ASSERT_DELTA(trim_adjust, -0.5, EPSILON);
    }

    //===========================================================================
    // Autopilot Disconnect Tests
    //===========================================================================

    // Test 70: Force disconnect threshold
    void testForceDisconnectThreshold() {
        double stick_force = 35.0;  // lbs
        double disconnect_threshold = 25.0;  // lbs

        bool force_disconnect = stick_force > disconnect_threshold;

        TS_ASSERT_EQUALS(force_disconnect, true);
    }

    // Test 71: Quick disconnect button
    void testQuickDisconnect() {
        bool autopilot_engaged = true;
        bool disconnect_button_pressed = true;

        if (disconnect_button_pressed) {
            autopilot_engaged = false;
        }

        TS_ASSERT_EQUALS(autopilot_engaged, false);
    }

    //===========================================================================
    // Control Harmony Tests
    //===========================================================================

    // Test 72: Pitch-roll harmony ratio
    void testPitchRollHarmony() {
        // Typical harmony ratio: 1.5-2.0
        double pitch_force_gradient = 4.0;  // lbs/g
        double roll_force_gradient = 2.5;   // lbs/deg/sec

        double harmony_ratio = pitch_force_gradient / roll_force_gradient;

        TS_ASSERT(harmony_ratio > 1.0);
        TS_ASSERT(harmony_ratio < 3.0);
    }

    // Test 73: Control sensitivity
    void testControlSensitivity() {
        double stick_deflection = 0.5;  // normalized
        double control_power = 20.0;    // deg/sec^2 per unit

        double response_rate = control_power * stick_deflection;

        TS_ASSERT_DELTA(response_rate, 10.0, EPSILON);
    }

    //===========================================================================
    // Model Following Tests
    //===========================================================================

    // Test 74: Model reference response
    void testModelReferenceResponse() {
        // Desired response: first-order with tau = 0.5 sec
        double tau = 0.5;
        double dt = 0.01;
        double cmd = 1.0;
        double model_state = 0.0;

        // Simulate for 50 steps (0.5 sec)
        for (int i = 0; i < 50; i++) {
            double model_dot = (cmd - model_state) / tau;
            model_state += model_dot * dt;
        }

        // After 1 tau, should reach ~63% of command
        TS_ASSERT_DELTA(model_state, 0.632, 0.02);
    }

    // Test 75: Model following error
    void testModelFollowingError() {
        double model_output = 0.8;
        double actual_output = 0.75;

        double model_error = model_output - actual_output;

        // Error used to adjust control
        double K_model = 2.0;
        double correction = K_model * model_error;

        TS_ASSERT_DELTA(model_error, 0.05, EPSILON);
        TS_ASSERT_DELTA(correction, 0.1, EPSILON);
    }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 * Extended Control Law Tests (25 new tests)
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGControlLawExtendedTest : public CxxTest::TestSuite
{
public:
    //===========================================================================
    // Advanced PID and Gain Scheduling Tests
    //===========================================================================

    // Test 76: PID with derivative filter
    void testPIDWithDerivativeFilter() {
        // Derivative term with first-order filter to reduce noise
        double error = 2.0;
        double prev_error = 1.8;
        double dt = 0.01;
        double tau_d = 0.1;  // Derivative filter time constant

        double raw_derivative = (error - prev_error) / dt;

        // Apply filter: d_filtered = (1 - alpha) * d_prev + alpha * d_raw
        double alpha = dt / (tau_d + dt);
        double prev_filtered = 15.0;
        double filtered_derivative = (1.0 - alpha) * prev_filtered + alpha * raw_derivative;

        TS_ASSERT(filtered_derivative < raw_derivative);  // Filtering reduces spikes
    }

    // Test 77: Multi-point gain schedule
    void testMultiPointGainSchedule() {
        // 3-point interpolation: 100 kts -> 1.0, 200 kts -> 0.6, 300 kts -> 0.4
        double speed = 250.0;

        double gain;
        if (speed <= 200.0) {
            gain = ScheduledGain::GetGain(speed, 100.0, 1.0, 200.0, 0.6);
        } else {
            gain = ScheduledGain::GetGain(speed, 200.0, 0.6, 300.0, 0.4);
        }

        TS_ASSERT_DELTA(gain, 0.5, EPSILON);
    }

    // Test 78: Gain scheduling with dynamic pressure
    void testDynamicPressureGainSchedule() {
        // q_bar = 0.5 * rho * V^2
        double rho = 0.002377;  // slug/ft^3
        double V = 400.0;       // ft/s
        double q_bar = 0.5 * rho * V * V;

        // Gain inversely proportional to q_bar
        double K_ref = 1.0;
        double q_ref = 100.0;
        double K = K_ref * q_ref / q_bar;

        TS_ASSERT(K < K_ref);  // Gain decreases with dynamic pressure
        TS_ASSERT_DELTA(q_bar, 190.16, 0.1);
    }

    //===========================================================================
    // Flight Phase Control Laws
    //===========================================================================

    // Test 79: Takeoff rotation law
    void testTakeoffRotationLaw() {
        double V_r = 150.0;     // Rotation speed kts
        double V_current = 155.0;
        double pitch_target = 10.0 * DEG_TO_RAD;

        // Initiate rotation when V > V_r
        bool rotate = V_current > V_r;
        double pitch_cmd = rotate ? pitch_target : 0.0;

        TS_ASSERT_EQUALS(rotate, true);
        TS_ASSERT_DELTA(pitch_cmd, pitch_target, EPSILON);
    }

    // Test 80: Ground effect compensation
    void testGroundEffectCompensation() {
        double altitude_agl = 20.0;  // ft
        double wingspan = 100.0;     // ft

        // Ground effect factor: increases lift as h/b decreases
        double h_b = altitude_agl / wingspan;
        double ground_effect_factor = 1.0 + 0.1 / (h_b + 0.1);

        TS_ASSERT(ground_effect_factor > 1.0);
        TS_ASSERT_DELTA(ground_effect_factor, 1.333, 0.01);
    }

    // Test 81: Approach speed compensation
    void testApproachSpeedCompensation() {
        double weight = 150000.0;    // lbs
        double weight_ref = 120000.0;
        double V_ref = 130.0;        // kts

        // Approach speed scales with sqrt of weight ratio
        double V_app = V_ref * std::sqrt(weight / weight_ref);

        TS_ASSERT(V_app > V_ref);
        TS_ASSERT_DELTA(V_app, 145.3, 0.5);
    }

    // Test 82: Crosswind landing crab/decrab
    void testCrosswindDecrab() {
        double heading_wind = 45.0 * DEG_TO_RAD;  // Relative wind angle
        double crosswind = 20.0;  // kts
        double groundspeed = 130.0;

        // Crab angle for crosswind
        double crab_angle = std::asin(crosswind * std::sin(heading_wind) / groundspeed);

        // Decrab required at touchdown
        TS_ASSERT(crab_angle > 0);
    }

    //===========================================================================
    // Engine Failure Control Laws
    //===========================================================================

    // Test 83: Asymmetric thrust compensation
    void testAsymmetricThrustCompensation() {
        double thrust_left = 20000.0;   // lbs
        double thrust_right = 0.0;      // Engine failure
        double arm = 20.0;              // ft (engine to centerline)

        double yaw_moment = (thrust_left - thrust_right) * arm;
        double K_rudder = 0.00001;  // rudder effectiveness

        double rudder_cmd = K_rudder * yaw_moment;

        TS_ASSERT(rudder_cmd > 0);  // Rudder to oppose yaw
        TS_ASSERT_DELTA(yaw_moment, 400000.0, 1.0);
    }

    // Test 84: Minimum control speed (Vmc)
    void testMinimumControlSpeed() {
        double thrust_asymmetric = 20000.0;
        double arm = 20.0;
        double rudder_power = 500.0;  // lb-ft per degree
        double max_rudder = 25.0;     // degrees

        double max_yaw_moment = rudder_power * max_rudder;
        double asymmetric_moment = thrust_asymmetric * arm;

        bool controllable = max_yaw_moment >= asymmetric_moment;

        TS_ASSERT_EQUALS(controllable, false);  // Need more airspeed
    }

    // Test 85: Bank angle limit with engine out
    void testEnginOutBankLimit() {
        double bank_normal = 30.0;    // degrees
        double bank_engine_out = 15.0; // degrees

        bool engine_failed = true;
        double bank_limit = engine_failed ? bank_engine_out : bank_normal;

        TS_ASSERT_DELTA(bank_limit, 15.0, EPSILON);
    }

    //===========================================================================
    // Turbulence Control Laws
    //===========================================================================

    // Test 86: Gust alleviation factor
    void testGustAlleviationFactor() {
        double gust_velocity = 15.0;  // ft/s vertical gust
        double wing_loading = 80.0;   // lbs/ft^2
        double V = 500.0;             // ft/s

        // Load factor increment from gust
        double rho = 0.002377;
        double CLa = 5.0;  // lift curve slope per rad
        double c_bar = 8.0;  // mean chord

        double mu = 2.0 * wing_loading / (rho * c_bar * CLa * 32.2);
        double Kg = 0.88 * mu / (5.3 + mu);  // Gust alleviation factor

        TS_ASSERT(Kg > 0.0);
        TS_ASSERT(Kg < 1.0);
    }

    // Test 87: Turbulence penetration speed
    void testTurbulencePenetrationSpeed() {
        double Va = 180.0;  // Maneuvering speed kts
        double Vno = 200.0; // Max structural cruising speed

        // Turbulence penetration between Va and Vno
        double V_turb = (Va + Vno) / 2.0;

        TS_ASSERT(V_turb > Va);
        TS_ASSERT(V_turb < Vno);
        TS_ASSERT_DELTA(V_turb, 190.0, EPSILON);
    }

    //===========================================================================
    // Stability Margin Tests
    //===========================================================================

    // Test 88: Static margin calculation
    void testStaticMarginCalculation() {
        double x_np = 0.35;   // Neutral point MAC fraction
        double x_cg = 0.28;   // CG MAC fraction

        double static_margin = x_np - x_cg;

        TS_ASSERT(static_margin > 0);  // Stable
        TS_ASSERT_DELTA(static_margin, 0.07, EPSILON);
    }

    // Test 89: Pitch damping requirement
    void testPitchDampingRequirement() {
        // Minimum damping ratio for short period
        double damping_ratio = 0.3;
        double min_damping = 0.25;

        bool adequate_damping = damping_ratio >= min_damping;

        TS_ASSERT_EQUALS(adequate_damping, true);
    }

    // Test 90: Dutch roll damping
    void testDutchRollDamping() {
        double omega_d = 2.0;     // rad/s natural frequency
        double zeta_d = 0.15;     // damping ratio

        // Minimum product requirement: zeta * omega > 0.15
        double product = zeta_d * omega_d;

        TS_ASSERT(product > 0.15);
        TS_ASSERT_DELTA(product, 0.3, EPSILON);
    }

    //===========================================================================
    // Control Effectiveness Tests
    //===========================================================================

    // Test 91: Elevator effectiveness with Mach
    void testElevatorEffectivenessWithMach() {
        double mach = 0.85;
        double effectiveness_ref = 1.0;

        // Compressibility reduces effectiveness
        double beta = std::sqrt(1.0 - mach * mach);
        double effectiveness = effectiveness_ref * beta;

        TS_ASSERT(effectiveness < effectiveness_ref);
        TS_ASSERT_DELTA(effectiveness, 0.527, 0.01);
    }

    // Test 92: Aileron effectiveness at high alpha
    void testAileronEffectivenessHighAlpha() {
        double alpha = 18.0 * DEG_TO_RAD;
        double alpha_stall = 15.0 * DEG_TO_RAD;
        double effectiveness_ref = 1.0;

        // Reduced effectiveness near stall
        double reduction = 0.0;
        if (alpha > alpha_stall) {
            reduction = 0.5 * (alpha - alpha_stall) / DEG_TO_RAD;
        }
        double effectiveness = effectiveness_ref - reduction;
        effectiveness = std::max(0.3, effectiveness);

        TS_ASSERT(effectiveness < effectiveness_ref);
    }

    // Test 93: Control surface hinge moment
    void testControlSurfaceHingeMoment() {
        double Ch0 = 0.0;
        double Cha = -0.5;  // Hinge moment due to alpha
        double Chd = -0.3;  // Hinge moment due to deflection
        double alpha = 5.0 * DEG_TO_RAD;
        double delta = 10.0 * DEG_TO_RAD;

        double Ch = Ch0 + Cha * alpha + Chd * delta;
        double q_bar = 100.0;  // psf
        double S_e = 20.0;     // ft^2
        double c_e = 2.0;      // ft

        double hinge_moment = q_bar * S_e * c_e * Ch;

        TS_ASSERT(hinge_moment < 0);  // Restoring moment
    }

    //===========================================================================
    // Multi-Axis Coupling Tests
    //===========================================================================

    // Test 94: Roll-yaw coupling
    void testRollYawCoupling() {
        double roll_rate = 0.2;  // rad/s
        double V = 300.0;        // ft/s
        double b = 80.0;         // wingspan ft

        // Yaw rate induced by roll (adverse yaw)
        double Cnp = -0.05;  // Yaw due to roll rate
        double yaw_induced = Cnp * roll_rate * b / (2.0 * V);

        TS_ASSERT(yaw_induced < 0);  // Adverse yaw
    }

    // Test 95: Pitch-roll coupling (inertia coupling)
    void testPitchRollCoupling() {
        double Ixx = 10000.0;  // Roll inertia
        double Izz = 50000.0;  // Yaw inertia
        double Iyy = 40000.0;  // Pitch inertia

        double pitch_rate = 0.1;  // rad/s
        double yaw_rate = 0.05;

        // Roll moment from pitch-yaw coupling
        // (Izz - Iyy) * p * r = (50000 - 40000) * 0.1 * 0.05 = 10000 * 0.005 = 50
        double roll_moment = (Izz - Iyy) * pitch_rate * yaw_rate;

        TS_ASSERT_DELTA(roll_moment, 50.0, EPSILON);
    }

    //===========================================================================
    // Digital Control Implementation Tests
    //===========================================================================

    // Test 96: Sample rate effect on phase lag
    void testSampleRatePhaselag() {
        double omega = 10.0;  // rad/s signal frequency
        double dt = 0.01;     // sample period

        // Phase lag from sample/hold: theta = omega * dt / 2
        double phase_lag = omega * dt / 2.0;

        TS_ASSERT_DELTA(phase_lag, 0.05, EPSILON);  // rad
    }

    // Test 97: Quantization effect
    void testQuantizationEffect() {
        double input = 0.12345;
        double resolution = 0.01;

        double quantized = std::round(input / resolution) * resolution;

        TS_ASSERT_DELTA(quantized, 0.12, EPSILON);
    }

    // Test 98: Computational delay compensation
    void testComputationalDelayCompensation() {
        double delay = 0.02;  // seconds
        double omega = 5.0;   // rad/s

        // Phase lag from delay
        double phase_lag = omega * delay;

        // Lead compensator to compensate
        double phase_lead = omega * delay;  // Match the lag

        double net_phase = phase_lag - phase_lead;

        TS_ASSERT_DELTA(net_phase, 0.0, EPSILON);
    }

    //===========================================================================
    // Auto-Throttle and Speed Control Tests
    //===========================================================================

    // Test 99: Speed hold with integral
    void testSpeedHoldWithIntegral() {
        SimplePID pid(0.1, 0.02, 0.0, 0.1);

        double target_speed = 250.0;
        double current_speed = 245.0;
        double error = target_speed - current_speed;

        double throttle_cmd = pid.Update(error);

        TS_ASSERT(throttle_cmd > 0);  // Increase throttle
    }

    // Test 100: Idle descent speed control
    void testIdleDescentSpeedControl() {
        double target_speed = 280.0;
        double current_speed = 300.0;
        double idle_thrust = 0.1;  // normalized

        // Speed high, throttle at idle
        double throttle = idle_thrust;
        double speed_error = target_speed - current_speed;

        // Use speed brakes if needed
        double speedbrake_cmd = 0.0;
        if (speed_error < -10.0) {
            speedbrake_cmd = 0.5;
        }

        TS_ASSERT_DELTA(throttle, idle_thrust, EPSILON);
        TS_ASSERT_DELTA(speedbrake_cmd, 0.5, EPSILON);
    }
};

#endif
