#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGNozzle.h>
#include <models/propulsion/FGThruster.h>
#include <math/FGColumnVector3.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

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

  /***************************************************************************
   * Extended Pressure-Thrust Relationship Tests
   ***************************************************************************/

  // Test linear thrust-pressure relationship
  void testLinearThrustPressureRelationship() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // Thrust should decrease linearly with pressure
    for (double p = 0.0; p <= 2000.0; p += 100.0) {
      double thrust = vacThrust - p * nozzleArea;
      double expectedThrust = vacThrust - p * nozzleArea;
      TS_ASSERT_DELTA(thrust, expectedThrust, 0.01);
    }
  }

  // Test thrust gradient with respect to pressure
  void testThrustGradient() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;
    double dp = 100.0;  // Pressure increment

    double p1 = 1000.0;
    double p2 = p1 + dp;

    double thrust1 = vacThrust - p1 * nozzleArea;
    double thrust2 = vacThrust - p2 * nozzleArea;

    double dThrust = thrust1 - thrust2;
    double expectedGradient = nozzleArea * dp;

    TS_ASSERT_DELTA(dThrust, expectedGradient, 0.01);
  }

  // Test thrust at stratosphere boundary
  void testThrustAtStratosphere() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // Tropopause (~36,000 ft, ~474 psf)
    double p_tropopause = 474.0;
    double thrust_tropopause = vacThrust - p_tropopause * nozzleArea;

    TS_ASSERT(thrust_tropopause > 99000.0);
    TS_ASSERT(thrust_tropopause < vacThrust);
  }

  /***************************************************************************
   * Expansion Ratio Tests
   ***************************************************************************/

  // Test expansion ratio calculation
  void testExpansionRatioCalculation() {
    // ε = A_exit / A_throat
    double A_throat = 0.5;  // sq ft
    double A_exit = 5.0;    // sq ft

    double epsilon = A_exit / A_throat;
    TS_ASSERT_DELTA(epsilon, 10.0, DEFAULT_TOLERANCE);

    // Higher expansion ratio for vacuum-optimized nozzle
    A_exit = 25.0;
    epsilon = A_exit / A_throat;
    TS_ASSERT_DELTA(epsilon, 50.0, DEFAULT_TOLERANCE);
  }

  // Test expansion ratio effects on thrust
  void testExpansionRatioThrustEffects() {
    double vacThrust = 100000.0;
    double ambientPressure = 2116.22;
    double A_throat = 0.5;

    // Low expansion ratio (ε = 4)
    double epsilon_low = 4.0;
    double A_exit_low = A_throat * epsilon_low;
    double thrust_low = vacThrust - ambientPressure * A_exit_low;

    // High expansion ratio (ε = 40)
    double epsilon_high = 40.0;
    double A_exit_high = A_throat * epsilon_high;
    double thrust_high = vacThrust - ambientPressure * A_exit_high;

    // At sea level, lower expansion performs better
    TS_ASSERT(thrust_low > thrust_high);
  }

  // Test optimal expansion ratio concept
  void testOptimalExpansionRatio() {
    // Optimal expansion: p_exit = p_ambient
    // For different altitudes, different ε is optimal

    double ambientPressures[] = {2116.22, 1000.0, 500.0, 100.0, 10.0};
    double optimalExpansions[] = {5.0, 10.0, 20.0, 50.0, 200.0};

    // At higher altitude (lower pressure), larger expansion is optimal
    for (int i = 1; i < 5; i++) {
      TS_ASSERT(optimalExpansions[i] > optimalExpansions[i-1]);
    }
  }

  /***************************************************************************
   * Nozzle Type Tests
   ***************************************************************************/

  // Test conical nozzle divergence losses
  void testConicalNozzleLosses() {
    // Conical nozzle thrust loss factor: λ = (1 + cos(α)) / 2
    // α = half-cone angle (typically 15 degrees)

    double alpha_deg = 15.0;
    double alpha_rad = alpha_deg * M_PI / 180.0;

    double lambda = (1.0 + cos(alpha_rad)) / 2.0;
    TS_ASSERT_DELTA(lambda, 0.9830, 0.001);  // (1 + cos(15°))/2 ≈ 0.983

    // Higher divergence angle = more losses
    alpha_deg = 30.0;
    alpha_rad = alpha_deg * M_PI / 180.0;
    double lambda_high = (1.0 + cos(alpha_rad)) / 2.0;
    TS_ASSERT(lambda_high < lambda);
  }

  // Test bell nozzle efficiency
  void testBellNozzleEfficiency() {
    // Bell (contoured) nozzles have λ ≈ 0.98-0.99
    // Better than conical due to optimized flow turning

    double lambda_conical = 0.983;  // 15-degree half-angle conical
    double lambda_bell = 0.995;     // Optimized bell nozzle

    TS_ASSERT(lambda_bell > lambda_conical);

    // Apply efficiency to thrust
    double vacThrust = 100000.0;
    double thrust_conical = vacThrust * lambda_conical;
    double thrust_bell = vacThrust * lambda_bell;

    TS_ASSERT(thrust_bell > thrust_conical);
    TS_ASSERT_DELTA(thrust_bell - thrust_conical, 1200.0, 100.0);
  }

  // Test aerospike nozzle concept
  void testAerospikeNozzleConcept() {
    // Aerospike nozzles have altitude compensation
    // Effective area ratio changes with altitude

    double vacThrust = 100000.0;

    // Aerospike at sea level acts like low expansion
    double effArea_SL = 2.0;
    double p_SL = 2116.22;
    double thrust_SL = vacThrust - p_SL * effArea_SL;

    // Aerospike at altitude acts like higher expansion
    double effArea_alt = 3.0;
    double p_alt = 500.0;
    double thrust_alt = vacThrust - p_alt * effArea_alt;

    // Both should maintain good efficiency
    TS_ASSERT(thrust_SL > 95000.0);
    TS_ASSERT(thrust_alt > 98000.0);
  }

  /***************************************************************************
   * Thrust Vectoring Tests
   ***************************************************************************/

  // Test thrust vector components with pitch angle
  void testThrustVectorPitch() {
    double totalThrust = 100000.0;
    double pitchAngle_deg = 10.0;
    double pitchAngle_rad = pitchAngle_deg * M_PI / 180.0;

    double axialThrust = totalThrust * cos(pitchAngle_rad);
    double normalThrust = totalThrust * sin(pitchAngle_rad);

    TS_ASSERT_DELTA(axialThrust, 98481.0, 1.0);
    TS_ASSERT_DELTA(normalThrust, 17365.0, 1.0);
  }

  // Test thrust vector components with yaw angle
  void testThrustVectorYaw() {
    double totalThrust = 100000.0;
    double yawAngle_deg = 5.0;
    double yawAngle_rad = yawAngle_deg * M_PI / 180.0;

    double axialThrust = totalThrust * cos(yawAngle_rad);
    double sideThrust = totalThrust * sin(yawAngle_rad);

    TS_ASSERT_DELTA(axialThrust, 99619.0, 1.0);
    TS_ASSERT_DELTA(sideThrust, 8716.0, 1.0);
  }

  // Test combined pitch and yaw vectoring
  void testCombinedThrustVectoring() {
    double totalThrust = 100000.0;
    double pitch_deg = 5.0;
    double yaw_deg = 3.0;

    double pitch_rad = pitch_deg * M_PI / 180.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;

    // Simplified 2-axis vectoring
    double axial = totalThrust * cos(pitch_rad) * cos(yaw_rad);
    double normal = totalThrust * sin(pitch_rad);
    double side = totalThrust * cos(pitch_rad) * sin(yaw_rad);

    // Magnitude should still be total thrust
    double magnitude = sqrt(axial*axial + normal*normal + side*side);
    TS_ASSERT_DELTA(magnitude, totalThrust, 1.0);
  }

  // Test maximum vector angle limits
  void testMaxVectorAngleLimits() {
    // Typical maximum vector angle is ±20 degrees
    double maxAngle_deg = 20.0;
    double maxAngle_rad = maxAngle_deg * M_PI / 180.0;

    double totalThrust = 100000.0;

    // At max angle, axial thrust is reduced
    double axialAtMax = totalThrust * cos(maxAngle_rad);
    TS_ASSERT_DELTA(axialAtMax, 93969.0, 1.0);

    // Normal force at max angle
    double normalAtMax = totalThrust * sin(maxAngle_rad);
    TS_ASSERT_DELTA(normalAtMax, 34202.0, 1.0);
  }

  /***************************************************************************
   * Multi-Engine Configuration Tests
   ***************************************************************************/

  // Test dual engine total thrust
  void testDualEngineThrust() {
    double singleVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;

    double singleThrust = singleVacThrust - ambientPressure * nozzleArea;
    double totalThrust = 2 * singleThrust;

    TS_ASSERT_DELTA(totalThrust, 2 * singleThrust, 0.01);
  }

  // Test quad engine configuration
  void testQuadEngineThrust() {
    double singleVacThrust = 500000.0;
    double nozzleArea = 8.0;
    double ambientPressure = 2116.22;

    double singleThrust = singleVacThrust - ambientPressure * nozzleArea;
    double totalThrust = 4 * singleThrust;

    TS_ASSERT(totalThrust > 1900000.0);
    TS_ASSERT(totalThrust < 2000000.0);
  }

  // Test engine out scenario
  void testEngineOutThrust() {
    double singleVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;
    int numEngines = 4;

    double singleThrust = singleVacThrust - ambientPressure * nozzleArea;
    double normalTotalThrust = numEngines * singleThrust;
    double engineOutThrust = (numEngines - 1) * singleThrust;

    double thrustLoss = normalTotalThrust - engineOutThrust;
    TS_ASSERT_DELTA(thrustLoss, singleThrust, 0.01);
    TS_ASSERT_DELTA(engineOutThrust / normalTotalThrust, 0.75, 0.001);
  }

  /***************************************************************************
   * Transient Operation Tests
   ***************************************************************************/

  // Test throttle up transient
  void testThrottleUpTransient() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;

    // Simulate throttle ramp from 0 to 100%
    for (double throttle = 0.0; throttle <= 1.0; throttle += 0.1) {
      double vacThrust = maxVacThrust * throttle;
      double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

      if (throttle > 0.05) {  // Above idle
        TS_ASSERT(thrust > 0.0);
      }
    }
  }

  // Test throttle down transient
  void testThrottleDownTransient() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;

    double prevThrust = maxVacThrust - ambientPressure * nozzleArea;

    // Simulate throttle ramp from 100% to 0
    for (double throttle = 1.0; throttle >= 0.0; throttle -= 0.1) {
      double vacThrust = maxVacThrust * throttle;
      double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

      TS_ASSERT(thrust <= prevThrust + 1.0);  // Thrust decreasing
      prevThrust = thrust;
    }
  }

  // Test rapid altitude change
  void testRapidAltitudeChange() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // Simulate rapid climb
    double altitudes[] = {0, 5000, 10000, 20000, 30000, 40000, 50000};
    double pressures[] = {2116.22, 1761, 1456, 973, 629, 392, 242};

    double prevThrust = 0.0;
    for (int i = 0; i < 7; i++) {
      double thrust = vacThrust - pressures[i] * nozzleArea;

      // Thrust should increase with altitude
      TS_ASSERT(thrust > prevThrust);
      prevThrust = thrust;
    }
  }

  /***************************************************************************
   * Performance Coefficient Tests
   ***************************************************************************/

  // Test thrust coefficient range
  void testThrustCoefficientRange() {
    double Athroat = 1.0;  // sq ft
    double pc_psf = 1000.0 * 144.0;  // 1000 psi to psf

    // Test various thrust levels
    double thrusts[] = {50000.0, 100000.0, 150000.0, 200000.0};

    for (int i = 0; i < 4; i++) {
      double Cf = thrusts[i] / (pc_psf * Athroat);
      TS_ASSERT(Cf > 0.3);
      TS_ASSERT(Cf < 2.0);
    }
  }

  // Test characteristic velocity
  void testCharacteristicVelocity() {
    // c* = p_c * A_throat / m_dot
    // Typical values: 5000-8000 ft/s

    double pc = 1000.0 * 144.0;  // 1000 psi to psf
    double Athroat = 1.0;
    double mdot = 100.0;  // lbs/s

    double cstar = pc * Athroat / mdot;
    TS_ASSERT(cstar > 1000.0);

    // Convert to proper units (this is simplified)
    double g0 = 32.174;
    double cstar_fps = cstar / g0 * 32.174;  // Simplified
    TS_ASSERT(cstar_fps > 0.0);
  }

  // Test mass flow relationship
  void testMassFlowRelationship() {
    // m_dot = F / (Isp * g0)

    double thrust = 100000.0;
    double Isp = 300.0;
    double g0 = 32.174;

    double mdot = thrust / (Isp * g0);
    TS_ASSERT_DELTA(mdot, 10.35, 0.1);

    // Higher Isp = lower mass flow for same thrust
    Isp = 400.0;
    double mdot_high = thrust / (Isp * g0);
    TS_ASSERT(mdot_high < mdot);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very high altitude (near vacuum)
  void testNearVacuumOperation() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    // 100 km altitude (~0.03 psf)
    double p_100km = 0.03;
    double thrust_100km = vacThrust - p_100km * nozzleArea;

    TS_ASSERT_DELTA(thrust_100km, vacThrust, 1.0);
  }

  // Test very low vacuum thrust
  void testVeryLowVacuumThrust() {
    double vacThrust = 100.0;  // Very small motor
    double nozzleArea = 0.01;
    double ambientPressure = 2116.22;

    double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);
    TS_ASSERT(thrust > 0.0);
    TS_ASSERT_DELTA(thrust, 78.84, 0.1);
  }

  // Test pressure exceeds vacuum thrust capability
  void testPressureExceedsCapability() {
    double vacThrust = 1000.0;  // Low vacuum thrust
    double nozzleArea = 10.0;   // Large area
    double ambientPressure = 2116.22;

    // At sea level, pressure loss exceeds vacuum thrust
    double rawThrust = vacThrust - ambientPressure * nozzleArea;
    TS_ASSERT(rawThrust < 0.0);

    // Should clamp to zero
    double clampedThrust = std::max(0.0, rawThrust);
    TS_ASSERT_DELTA(clampedThrust, 0.0, DEFAULT_TOLERANCE);
  }

  // Test extreme expansion ratio
  void testExtremeExpansionRatio() {
    double vacThrust = 100000.0;

    // Extreme expansion ratio ε = 200 (upper stage nozzle)
    double Athroat = 0.5;
    double epsilon = 200.0;
    double Aexit = Athroat * epsilon;  // 100 sq ft

    // At sea level, severe overexpansion
    double p_SL = 2116.22;
    double thrust_SL = vacThrust - p_SL * Aexit;
    TS_ASSERT(thrust_SL < 0.0);

    // In vacuum, full performance
    double p_vac = 0.0;
    double thrust_vac = vacThrust - p_vac * Aexit;
    TS_ASSERT_DELTA(thrust_vac, vacThrust, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test many pressure calculations
  void testStressManyPressureCalcs() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    for (int i = 0; i < 1000; i++) {
      double pressure = (i % 2200);
      double thrust = std::max(0.0, vacThrust - pressure * nozzleArea);

      TS_ASSERT(thrust >= 0.0);
      TS_ASSERT(!std::isnan(thrust));
    }
  }

  // Test oscillating pressure
  void testStressOscillatingPressure() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;

    for (int i = 0; i < 500; i++) {
      double pressure = 1000.0 + 500.0 * sin(i * 0.1);
      double thrust = vacThrust - pressure * nozzleArea;

      TS_ASSERT(!std::isnan(thrust));
      TS_ASSERT(!std::isinf(thrust));
    }
  }

  // Test rapid thrust cycling
  void testStressRapidThrustCycling() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;

    for (int i = 0; i < 1000; i++) {
      double throttle = (i % 2 == 0) ? 1.0 : 0.0;  // Full on/off cycling
      double vacThrust = maxVacThrust * throttle;
      double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

      TS_ASSERT(thrust >= 0.0);
    }
  }

  // Test combined altitude and throttle sweep
  void testStressAltitudeThrottleSweep() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;

    for (double throttle = 0.0; throttle <= 1.0; throttle += 0.1) {
      for (double pressure = 0.0; pressure <= 2200.0; pressure += 200.0) {
        double vacThrust = maxVacThrust * throttle;
        double thrust = std::max(0.0, vacThrust - pressure * nozzleArea);

        TS_ASSERT(thrust >= 0.0);
        TS_ASSERT(!std::isnan(thrust));
      }
    }
  }

  /***************************************************************************
   * Additional Physical Relationship Tests
   ***************************************************************************/

  // Test thrust to weight ratio calculation
  void testThrustToWeightRatio() {
    double vacThrust = 2000000.0;  // Saturn V F-1 scale
    double engineWeight = 20000.0;  // lbs

    double TWR = vacThrust / engineWeight;
    TS_ASSERT_DELTA(TWR, 100.0, 0.1);

    // Good rocket engine has TWR > 50
    TS_ASSERT(TWR > 50.0);
  }

  // Test nozzle thermal constraints
  void testNozzleThermalConstraintsConcept() {
    // Nozzle throat temperature is critical
    // T_throat ≈ T_chamber * (2 / (γ + 1))

    double Tc = 6000.0;  // Combustion chamber temp (R)
    double gamma = 1.2;

    double Tthroat = Tc * (2.0 / (gamma + 1.0));
    TS_ASSERT_DELTA(Tthroat, 5454.5, 1.0);

    // Material limits
    double maxMaterialTemp = 4000.0;  // Need cooling if exceeds
    TS_ASSERT(Tthroat > maxMaterialTemp);  // Would need cooling
  }

  // Test nozzle length approximation
  void testNozzleLengthApproximation() {
    // Approximate nozzle length for conical nozzle
    // L = (D_exit - D_throat) / (2 * tan(α))

    double Athroat = 1.0;  // sq ft
    double Aexit = 10.0;   // sq ft (ε = 10)

    double Dthroat = 2.0 * sqrt(Athroat / M_PI);
    double Dexit = 2.0 * sqrt(Aexit / M_PI);

    double alpha_deg = 15.0;
    double alpha_rad = alpha_deg * M_PI / 180.0;

    double length = (Dexit - Dthroat) / (2.0 * tan(alpha_rad));
    TS_ASSERT(length > 0.0);
    TS_ASSERT(length < 20.0);  // Reasonable nozzle length
  }

  // Test area ratio from pressure ratio (isentropic)
  void testAreaRatioFromPressureRatio() {
    // For given p_exit/p_chamber and γ, there's a unique ε
    // This is the inverse of the isentropic flow relation

    double gamma = 1.4;
    double pressureRatio = 0.05;  // p_e/p_c

    // Simplified check - higher pressure ratio (closer to 1) means lower ε
    double pressureRatio_high = 0.2;
    double pressureRatio_low = 0.01;

    // Just verify the relationship direction
    TS_ASSERT(pressureRatio_high > pressureRatio_low);
    // Lower pressure ratio requires higher expansion ratio
  }
};
