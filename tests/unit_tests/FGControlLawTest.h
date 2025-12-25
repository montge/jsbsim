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

#endif  // FGCONTROLLAWTEST_H
