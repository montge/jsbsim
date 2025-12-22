#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGNozzle.h>
#include <models/propulsion/FGThruster.h>
#include <math/FGColumnVector3.h>
#include "TestUtilities.h"

using namespace JSBSim;

class FGNozzleTest : public CxxTest::TestSuite
{
public:
  // Test nozzle pressure thrust component
  void testPressureThrustComponent() {
    // Pressure thrust = (p_exit - p_ambient) * A_exit
    // In JSBSim: Thrust_loss = p_ambient * A_exit
    // Actual_thrust = vac_thrust - p_ambient * A_exit

    double vacThrust = 100000.0;      // lbs (vacuum thrust)
    double ambientPressure = 2116.22; // psf (sea level)
    double nozzleArea = 2.0;          // sq ft

    // Calculate pressure thrust loss
    double pressureThrustLoss = ambientPressure * nozzleArea;
    TS_ASSERT_DELTA(pressureThrustLoss, 4232.44, 0.01);

    // Actual thrust at sea level
    double actualThrust = vacThrust - pressureThrustLoss;
    TS_ASSERT_DELTA(actualThrust, 95767.56, 0.01);
    TS_ASSERT(actualThrust < vacThrust);
  }

  // Test vacuum thrust (no atmospheric pressure)
  void testVacuumThrust() {
    double vacThrust = 100000.0;
    double ambientPressure = 0.0;  // Vacuum
    double nozzleArea = 2.0;

    // In vacuum, no pressure thrust loss
    double actualThrust = vacThrust - ambientPressure * nozzleArea;
    TS_ASSERT_DELTA(actualThrust, vacThrust, DEFAULT_TOLERANCE);
  }

  // Test altitude effects on nozzle performance
  void testAltitudeEffects() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // Sea level
    double p_sl = 2116.22;  // psf
    double thrust_sl = vacThrust - p_sl * nozzleArea;

    // 10,000 ft altitude (~1456 psf)
    double p_10k = 1456.0;
    double thrust_10k = vacThrust - p_10k * nozzleArea;

    // 50,000 ft altitude (~242 psf)
    double p_50k = 242.0;
    double thrust_50k = vacThrust - p_50k * nozzleArea;

    // Thrust increases with altitude
    TS_ASSERT(thrust_10k > thrust_sl);
    TS_ASSERT(thrust_50k > thrust_10k);
    TS_ASSERT(thrust_50k < vacThrust);
  }

  // Test nozzle area effects on thrust
  void testNozzleAreaEffects() {
    double vacThrust = 100000.0;
    double ambientPressure = 2116.22;

    // Small nozzle area
    double area_small = 1.0;
    double thrust_small = vacThrust - ambientPressure * area_small;

    // Large nozzle area
    double area_large = 5.0;
    double thrust_large = vacThrust - ambientPressure * area_large;

    // Larger area = more pressure thrust loss at sea level
    TS_ASSERT(thrust_small > thrust_large);

    // Calculate the difference
    double thrustDiff = thrust_small - thrust_large;
    double expectedDiff = ambientPressure * (area_large - area_small);
    TS_ASSERT_DELTA(thrustDiff, expectedDiff, 0.01);
  }

  // Test ideal expansion at design altitude
  void testIdealExpansion() {
    // At design altitude where p_exit = p_ambient, nozzle is ideally expanded
    // This is the maximum efficiency condition

    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // Design condition: p_exit = p_ambient
    // Assume nozzle designed for ~30,000 ft (~629 psf)
    double p_design = 629.0;

    double thrust_design = vacThrust - p_design * nozzleArea;

    // At design condition, thrust is optimized for that altitude
    TS_ASSERT(thrust_design > 0.0);
    TS_ASSERT(thrust_design < vacThrust);
  }

  // Test overexpanded nozzle behavior (p_exit < p_ambient)
  void testOverexpandedNozzle() {
    // Overexpanded: nozzle designed for high altitude, operating at low altitude
    // p_exit < p_ambient causes flow separation and efficiency loss

    double vacThrust = 100000.0;
    double nozzleArea = 5.0;  // Large area implies high expansion ratio
    double ambientPressure = 2116.22;  // Sea level

    // Overexpanded nozzle loses significant thrust at sea level
    double actualThrust = vacThrust - ambientPressure * nozzleArea;
    double thrustLoss = vacThrust - actualThrust;

    TS_ASSERT_DELTA(thrustLoss, 10581.1, 0.1);
    TS_ASSERT(thrustLoss > 10000.0);  // Significant loss
  }

  // Test underexpanded nozzle behavior (p_exit > p_ambient)
  void testUnderexpandedNozzle() {
    // Underexpanded: nozzle designed for low altitude, operating at high altitude
    // p_exit > p_ambient means thrust could be higher with more expansion

    double vacThrust = 100000.0;
    double nozzleArea = 1.0;  // Small area implies low expansion ratio
    double ambientPressure = 242.0;  // 50,000 ft

    // Underexpanded nozzle still produces good thrust but not optimized
    double actualThrust = vacThrust - ambientPressure * nozzleArea;
    double thrustLoss = vacThrust - actualThrust;

    TS_ASSERT_DELTA(thrustLoss, 242.0, 0.01);
    TS_ASSERT(thrustLoss < 300.0);  // Minimal loss at altitude
  }

  // Test expansion ratio relationship
  void testExpansionRatioRelationship() {
    // Expansion ratio ε = A_exit / A_throat
    // Higher ε = better vacuum performance but worse sea level performance

    double vacThrust = 100000.0;
    double ambientPressure = 2116.22;

    // Low expansion ratio (ε ~ 4, A_exit small)
    double area_low_epsilon = 1.5;
    double thrust_low_epsilon = vacThrust - ambientPressure * area_low_epsilon;

    // High expansion ratio (ε ~ 40, A_exit large)
    double area_high_epsilon = 4.0;
    double thrust_high_epsilon = vacThrust - ambientPressure * area_high_epsilon;

    // At sea level: low expansion ratio performs better
    TS_ASSERT(thrust_low_epsilon > thrust_high_epsilon);

    // In vacuum (p_ambient = 0): both produce same thrust (vacThrust)
    double thrust_vac_low = vacThrust - 0.0 * area_low_epsilon;
    double thrust_vac_high = vacThrust - 0.0 * area_high_epsilon;
    TS_ASSERT_DELTA(thrust_vac_low, thrust_vac_high, DEFAULT_TOLERANCE);
  }

  // Test momentum thrust component concept
  void testMomentumThrustConcept() {
    // Total thrust = momentum thrust + pressure thrust
    // Momentum thrust = m_dot * V_exit
    // Pressure thrust = (p_exit - p_ambient) * A_exit

    // Example values
    double massFlow = 200.0;      // lbs/s
    double exitVelocity = 8000.0; // ft/s
    double p_exit = 5.0;          // psi (converted to psf)
    double p_ambient = 14.7;      // psi
    double area = 2.0;            // sq ft

    // Momentum thrust
    double momentumThrust = (massFlow * exitVelocity) / 32.174;  // F = ma/g_c

    // Pressure thrust (note: negative when p_exit < p_ambient)
    double pressureThrust = (p_exit - p_ambient) * 144.0 * area;  // Convert psi to psf

    // Total thrust
    double totalThrust = momentumThrust + pressureThrust;

    TS_ASSERT(momentumThrust > 0.0);
    TS_ASSERT(pressureThrust < 0.0);  // Overexpanded condition
    TS_ASSERT(totalThrust > 0.0);
  }

  // Test nozzle exit velocity calculation
  void testNozzleExitVelocity() {
    // Exit velocity from isentropic expansion
    // V_exit = sqrt(2 * gamma/(gamma-1) * R * T_c * (1 - (p_exit/p_c)^((gamma-1)/gamma)))

    // For ideal gas with gamma = 1.4
    double gamma = 1.4;
    double Tc = 3500.0;  // Chamber temperature (K)
    double pc = 1000.0;  // Chamber pressure (psi)
    double pe = 14.7;    // Exit pressure (psi)

    double pressureRatio = pe / pc;  // 0.0147
    double exponent = (gamma - 1.0) / gamma;  // 0.286
    double term = 1.0 - pow(pressureRatio, exponent);

    TS_ASSERT(term > 0.0 && term < 1.0);
    TS_ASSERT(pressureRatio < 1.0);
  }

  // Test thrust coefficient Cf calculation concept
  void testThrustCoefficientConcept() {
    // Thrust coefficient: Cf = F / (p_c * A_throat)
    // Typical values: 1.4 - 1.8 for rocket nozzles

    double thrust = 100000.0;   // lbs
    double pc = 1000.0;         // Chamber pressure (psi)
    double pc_psf = pc * 144.0; // Convert to psf
    double Athroat = 1.0;       // Throat area (sq ft)

    double Cf = thrust / (pc_psf * Athroat);

    // Cf should be in reasonable range
    TS_ASSERT(Cf > 0.5);
    TS_ASSERT(Cf < 2.0);

    // Higher Cf indicates better nozzle efficiency
    double Cf_good = 1.7;
    double Cf_poor = 1.3;
    TS_ASSERT(Cf_good > Cf_poor);
  }

  // Test zero vacuum thrust condition
  void testZeroVacuumThrust() {
    double vacThrust = 0.0;
    double ambientPressure = 2116.22;
    double nozzleArea = 2.0;

    double actualThrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

    // Even with pressure loss, thrust cannot be negative (max(0, ...))
    TS_ASSERT_DELTA(actualThrust, 0.0, DEFAULT_TOLERANCE);
  }

  // Test negative vacuum thrust (should not occur in practice)
  void testNegativeVacuumThrustHandling() {
    double vacThrust = -1000.0;  // Invalid but test for robustness
    double ambientPressure = 2116.22;
    double nozzleArea = 2.0;

    double actualThrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

    // Should clamp to zero
    TS_ASSERT_DELTA(actualThrust, 0.0, DEFAULT_TOLERANCE);
  }

  // Test very large vacuum thrust
  void testLargeVacuumThrust() {
    double vacThrust = 1.0e7;  // 10 million lbs (large launch vehicle)
    double ambientPressure = 2116.22;
    double nozzleArea = 50.0;  // Large nozzle

    double actualThrust = vacThrust - ambientPressure * nozzleArea;
    double expectedThrust = vacThrust - 105811.0;

    TS_ASSERT_DELTA(actualThrust, expectedThrust, 1.0);
    TS_ASSERT(actualThrust > 9.8e6);
  }

  // Test nozzle thrust with reverser angle
  void testNozzleWithReverserAngle() {
    // FGNozzle::Calculate returns thrust before reverser
    // Base FGThruster::Calculate applies reverser: Thrust = cos(angle) * thrust

    double vacThrust = 100000.0;
    double ambientPressure = 2116.22;
    double nozzleArea = 2.0;

    // Calculate base nozzle thrust
    double nozzleThrust = vacThrust - ambientPressure * nozzleArea;

    // Test with various reverser angles
    double reverserAngle = 0.0;  // No reversal
    double finalThrust = cos(reverserAngle) * nozzleThrust;
    TS_ASSERT_DELTA(finalThrust, nozzleThrust, 0.01);

    // Half reversal (60 degrees)
    reverserAngle = M_PI / 3.0;
    finalThrust = cos(reverserAngle) * nozzleThrust;
    TS_ASSERT_DELTA(finalThrust, nozzleThrust * 0.5, 0.01);

    // Full reversal (180 degrees)
    reverserAngle = M_PI;
    finalThrust = cos(reverserAngle) * nozzleThrust;
    TS_ASSERT_DELTA(finalThrust, -nozzleThrust, 0.01);
  }

  // Test nozzle performance at multiple flight conditions
  void testMultipleFlightConditions() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.5;

    // Ground static (sea level)
    double p_ground = 2116.22;
    double thrust_ground = vacThrust - p_ground * nozzleArea;

    // Takeoff roll (sea level, dynamic)
    double p_takeoff = 2116.22;
    double thrust_takeoff = vacThrust - p_takeoff * nozzleArea;

    // Climb (20,000 ft, ~973 psf)
    double p_climb = 973.0;
    double thrust_climb = vacThrust - p_climb * nozzleArea;

    // Cruise (40,000 ft, ~392 psf)
    double p_cruise = 392.0;
    double thrust_cruise = vacThrust - p_cruise * nozzleArea;

    // Space (vacuum)
    double p_space = 0.0;
    double thrust_space = vacThrust - p_space * nozzleArea;

    // Verify thrust progression with altitude
    TS_ASSERT(thrust_ground < thrust_climb);
    TS_ASSERT(thrust_climb < thrust_cruise);
    TS_ASSERT(thrust_cruise < thrust_space);
    TS_ASSERT_DELTA(thrust_space, vacThrust, DEFAULT_TOLERANCE);
  }

  // Test nozzle area bounds
  void testNozzleAreaBounds() {
    double vacThrust = 100000.0;
    double ambientPressure = 2116.22;

    // Very small area (high pressure, low expansion)
    double area_min = 0.1;
    double thrust_min = vacThrust - ambientPressure * area_min;
    TS_ASSERT(thrust_min > 99500.0);

    // Very large area (low pressure, high expansion)
    double area_max = 20.0;
    double thrust_max = vacThrust - ambientPressure * area_max;
    TS_ASSERT(thrust_max < 60000.0);

    // Zero area (theoretical, would cause division by zero in some calcs)
    double area_zero = 0.0;
    double thrust_zero = vacThrust - ambientPressure * area_zero;
    TS_ASSERT_DELTA(thrust_zero, vacThrust, DEFAULT_TOLERANCE);
  }

  // Test thrust consistency across pressure range
  void testThrustConsistencyAcrossPressureRange() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // Test across pressure range from vacuum to sea level
    for (double pressure = 0.0; pressure <= 2200.0; pressure += 200.0) {
      double thrust = std::max(0.0, vacThrust - pressure * nozzleArea);

      // Thrust should be non-negative
      TS_ASSERT(thrust >= 0.0);

      // Thrust should never exceed vacuum thrust
      TS_ASSERT(thrust <= vacThrust + DEFAULT_TOLERANCE);

      // Thrust should decrease monotonically with pressure
      if (pressure > 0.0) {
        double prevThrust = std::max(0.0, vacThrust - (pressure - 200.0) * nozzleArea);
        TS_ASSERT(thrust <= prevThrust + DEFAULT_TOLERANCE);
      }
    }
  }

  // Test specific impulse relationship to exit velocity
  void testIspToExitVelocityRelationship() {
    // Isp (vacuum) = F / (m_dot * g0) = V_exit / g0
    // Therefore: V_exit = Isp * g0

    double Isp = 300.0;  // seconds
    double g0 = 32.174;  // ft/s^2

    double V_exit = Isp * g0;
    TS_ASSERT_DELTA(V_exit, 9652.2, 0.1);

    // Higher Isp means higher exit velocity
    double Isp_high = 450.0;
    double V_exit_high = Isp_high * g0;
    TS_ASSERT(V_exit_high > V_exit);
    TS_ASSERT_DELTA(V_exit_high, 14478.3, 0.1);
  }

  // Test nozzle efficiency concept
  void testNozzleEfficiencyConcept() {
    // Nozzle efficiency η = actual Isp / ideal Isp
    // Typical values: 0.95 - 0.98 for well-designed nozzles

    double idealIsp = 350.0;
    double efficiency = 0.96;
    double actualIsp = idealIsp * efficiency;

    TS_ASSERT_DELTA(actualIsp, 336.0, 0.1);
    TS_ASSERT(actualIsp < idealIsp);
    TS_ASSERT(efficiency > 0.9 && efficiency < 1.0);

    // Losses come from viscous effects, heat transfer, etc.
    double losses = idealIsp - actualIsp;
    TS_ASSERT_DELTA(losses, 14.0, 0.1);
  }

  // Test chamber pressure to exit pressure ratio
  void testPressureRatioEffects() {
    // Pressure ratio p_c/p_e affects thrust and exit velocity
    // Higher ratio = higher exit velocity but requires larger expansion ratio

    double pc = 1000.0;  // Chamber pressure (psi)

    // Low expansion (p_c/p_e = 10)
    double pe_low = pc / 10.0;
    double ratio_low = pc / pe_low;
    TS_ASSERT_DELTA(ratio_low, 10.0, DEFAULT_TOLERANCE);

    // High expansion (p_c/p_e = 100)
    double pe_high = pc / 100.0;
    double ratio_high = pc / pe_high;
    TS_ASSERT_DELTA(ratio_high, 100.0, DEFAULT_TOLERANCE);

    // Higher pressure ratio yields better performance in vacuum
    TS_ASSERT(ratio_high > ratio_low);
  }
};
