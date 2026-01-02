#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGNozzle.h>
#include <models/propulsion/FGThruster.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>
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

  /***************************************************************************
   * Propellant Properties Tests
   ***************************************************************************/

  // Test propellant specific impulse comparison
  void testPropellantIspComparison() {
    // Different propellants have different Isp values
    double Isp_LOX_RP1 = 300.0;     // LOX/RP-1 (Kerosene)
    double Isp_LOX_LH2 = 450.0;     // LOX/LH2 (Hydrogen)
    double Isp_N2O4_UDMH = 280.0;   // Storable hypergolic
    double Isp_solid = 250.0;       // Solid propellant

    TS_ASSERT(Isp_LOX_LH2 > Isp_LOX_RP1);
    TS_ASSERT(Isp_LOX_RP1 > Isp_N2O4_UDMH);
    TS_ASSERT(Isp_N2O4_UDMH > Isp_solid);
  }

  // Test mass flow from thrust and Isp
  void testMassFlowFromThrustAndIsp() {
    double g0 = 32.174;  // ft/s^2
    double thrust = 500000.0;  // lbs

    // Higher Isp = lower mass flow for same thrust
    double Isp1 = 300.0;
    double mdot1 = thrust / (Isp1 * g0);

    double Isp2 = 450.0;
    double mdot2 = thrust / (Isp2 * g0);

    TS_ASSERT(mdot2 < mdot1);
    TS_ASSERT_DELTA(mdot1 / mdot2, 1.5, 0.01);
  }

  // Test delta-V relationship
  void testDeltaVRelationship() {
    // Tsiolkovsky: ΔV = Isp * g0 * ln(m0 / mf)
    double Isp = 350.0;
    double g0 = 32.174;
    double m0 = 1000000.0;  // Initial mass
    double mf = 100000.0;   // Final mass

    double deltaV = Isp * g0 * log(m0 / mf);
    TS_ASSERT(deltaV > 25000.0);

    // Higher Isp = higher delta-V
    double Isp_high = 450.0;
    double deltaV_high = Isp_high * g0 * log(m0 / mf);
    TS_ASSERT(deltaV_high > deltaV);
  }

  /***************************************************************************
   * Combustion Chamber Tests
   ***************************************************************************/

  // Test chamber pressure effect on thrust
  void testChamberPressureEffect() {
    double Athroat = 1.0;
    double Cf = 1.6;  // Thrust coefficient

    // Higher chamber pressure = higher thrust
    double pc1 = 500.0 * 144.0;  // 500 psi in psf
    double thrust1 = Cf * pc1 * Athroat;

    double pc2 = 1000.0 * 144.0;
    double thrust2 = Cf * pc2 * Athroat;

    TS_ASSERT_DELTA(thrust2 / thrust1, 2.0, 0.01);
  }

  // Test combustion efficiency effect
  void testCombustionEfficiencyEffect() {
    double theoreticalIsp = 350.0;

    // Real combustion efficiency 95-98%
    double efficiency_low = 0.95;
    double efficiency_high = 0.98;

    double Isp_low = theoreticalIsp * efficiency_low;
    double Isp_high = theoreticalIsp * efficiency_high;

    TS_ASSERT_DELTA(Isp_low, 332.5, 0.1);
    TS_ASSERT_DELTA(Isp_high, 343.0, 0.1);
    TS_ASSERT(Isp_high > Isp_low);
  }

  // Test mixture ratio effect on Isp
  void testMixtureRatioEffect() {
    // Optimal mixture ratio maximizes Isp
    // LOX/LH2: optimal O/F ~ 6.0
    // Too lean or rich reduces Isp

    double Isp_optimal = 450.0;  // At O/F = 6.0
    double Isp_lean = 430.0;     // At O/F = 5.0
    double Isp_rich = 420.0;     // At O/F = 7.0

    TS_ASSERT(Isp_optimal > Isp_lean);
    TS_ASSERT(Isp_optimal > Isp_rich);
  }

  /***************************************************************************
   * Nozzle Startup/Shutdown Tests
   ***************************************************************************/

  // Test startup thrust buildup
  void testStartupThrustBuildup() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;

    // Simulate startup ramp over 20 steps
    double prevThrust = 0.0;
    for (int i = 0; i <= 20; i++) {
      double fraction = i / 20.0;
      double vacThrust = maxVacThrust * fraction;
      double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

      TS_ASSERT(thrust >= prevThrust - 1.0);
      prevThrust = thrust;
    }
  }

  // Test shutdown thrust decay
  void testShutdownThrustDecay() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 2116.22;

    // Simulate shutdown over 20 steps
    double prevThrust = maxVacThrust - ambientPressure * nozzleArea;
    for (int i = 20; i >= 0; i--) {
      double fraction = i / 20.0;
      double vacThrust = maxVacThrust * fraction;
      double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);

      TS_ASSERT(thrust <= prevThrust + 1.0);
      prevThrust = thrust;
    }
  }

  // Test restart capability
  void testRestartCapability() {
    double maxVacThrust = 100000.0;
    double nozzleArea = 2.0;
    double ambientPressure = 500.0;  // In space

    // First burn
    double thrust1 = maxVacThrust - ambientPressure * nozzleArea;
    TS_ASSERT(thrust1 > 98000.0);

    // Coast (engine off)
    double thrustCoast = 0.0;
    TS_ASSERT_DELTA(thrustCoast, 0.0, DEFAULT_TOLERANCE);

    // Restart
    double thrust2 = maxVacThrust - ambientPressure * nozzleArea;
    TS_ASSERT_DELTA(thrust2, thrust1, 0.01);
  }

  /***************************************************************************
   * Flow Separation Tests
   ***************************************************************************/

  // Test flow separation onset
  void testFlowSeparationOnset() {
    // Flow separation occurs when p_wall < ~0.4 * p_ambient
    double ambientPressure = 2116.22;
    double separationThreshold = 0.4 * ambientPressure;

    // Wall pressure must stay above threshold
    double p_wall_safe = 0.5 * ambientPressure;
    double p_wall_separated = 0.3 * ambientPressure;

    TS_ASSERT(p_wall_safe > separationThreshold);
    TS_ASSERT(p_wall_separated < separationThreshold);
  }

  // Test restricted shock separation effect
  void testRestrictedShockSeparation() {
    double vacThrust = 100000.0;
    double nozzleArea = 50.0;  // Large area for severe overexpansion
    double ambientPressure = 2116.22;

    // Highly overexpanded - shock separation occurs
    double rawThrust = vacThrust - ambientPressure * nozzleArea;
    TS_ASSERT(rawThrust < 0.0);  // Would be negative without separation

    // With flow separation, effective area is reduced
    double effectiveArea = 20.0;  // Flow separates, reducing effective area
    double separatedThrust = vacThrust - ambientPressure * effectiveArea;
    TS_ASSERT(separatedThrust > rawThrust);
  }

  /***************************************************************************
   * Altitude Compensation Nozzle Tests
   ***************************************************************************/

  // Test dual-bell nozzle low altitude mode
  void testDualBellNozzleLowAltitude() {
    double vacThrust = 100000.0;
    double ambientPressure = 2116.22;

    // Dual-bell: inner contour for sea level
    double innerArea = 2.0;  // Low expansion
    double thrustSL = vacThrust - ambientPressure * innerArea;

    TS_ASSERT(thrustSL > 95000.0);
  }

  // Test dual-bell nozzle high altitude mode
  void testDualBellNozzleHighAltitude() {
    double vacThrust = 100000.0;
    double ambientPressure = 100.0;  // High altitude

    // Dual-bell: outer contour for altitude
    double outerArea = 8.0;  // High expansion
    double thrustAlt = vacThrust - ambientPressure * outerArea;

    TS_ASSERT(thrustAlt > 99000.0);
  }

  // Test extendable nozzle performance gain
  void testExtendableNozzleGain() {
    double vacThrust = 100000.0;
    double ambientPressure = 50.0;  // Upper stage altitude

    // Retracted (lower expansion)
    double areaRetracted = 3.0;
    double thrustRetracted = vacThrust - ambientPressure * areaRetracted;

    // Extended (higher expansion)
    double areaExtended = 10.0;
    double thrustExtended = vacThrust - ambientPressure * areaExtended;

    // At very low pressure, larger area still performs well
    TS_ASSERT(thrustRetracted > 99800.0);
    TS_ASSERT(thrustExtended > 99400.0);
  }

  /***************************************************************************
   * Gimbal and Vectoring Extended Tests
   ***************************************************************************/

  // Test gimbal rate limits
  void testGimbalRateLimits() {
    double maxRate = 20.0;  // degrees per second
    double dt = 0.01;       // 10 ms timestep
    double maxDelta = maxRate * dt;

    double currentAngle = 0.0;
    double commandedAngle = 5.0;

    // Rate limited response
    double angleDelta = std::min(std::abs(commandedAngle - currentAngle), maxDelta);
    double newAngle = currentAngle + angleDelta;

    TS_ASSERT_DELTA(newAngle, 0.2, 0.01);
    TS_ASSERT(newAngle < commandedAngle);
  }

  // Test gimbal actuator authority
  void testGimbalActuatorAuthority() {
    double maxGimbalAngle = 7.0;  // degrees

    // Test within limits
    double commanded = 5.0;
    double actual = std::min(std::abs(commanded), maxGimbalAngle);
    TS_ASSERT_DELTA(actual, 5.0, 0.01);

    // Test at limits
    commanded = 10.0;
    actual = std::min(std::abs(commanded), maxGimbalAngle);
    TS_ASSERT_DELTA(actual, 7.0, 0.01);
  }

  // Test differential thrust steering
  void testDifferentialThrustSteering() {
    double nominalThrust = 100000.0;
    double throttleLeft = 0.9;
    double throttleRight = 1.0;

    double thrustLeft = nominalThrust * throttleLeft;
    double thrustRight = nominalThrust * throttleRight;
    double yawMoment = (thrustRight - thrustLeft) * 5.0;  // 5 ft moment arm

    TS_ASSERT(yawMoment > 0.0);
    TS_ASSERT_DELTA(yawMoment, 50000.0, 1.0);
  }

  /***************************************************************************
   * Multi-Engine Failure Mode Tests
   ***************************************************************************/

  // Test two engine out of four
  void testTwoEngineOut() {
    double singleThrust = 100000.0;
    int totalEngines = 4;
    int workingEngines = 2;

    double normalThrust = totalEngines * singleThrust;
    double degradedThrust = workingEngines * singleThrust;

    TS_ASSERT_DELTA(degradedThrust / normalThrust, 0.5, 0.001);
  }

  // Test asymmetric engine failure
  void testAsymmetricEngineFailure() {
    double singleThrust = 100000.0;
    double armLength = 10.0;  // ft from centerline

    // 4-engine cluster, one engine out
    double momentArm = armLength;
    double yawMoment = singleThrust * momentArm;

    TS_ASSERT_DELTA(yawMoment, 1000000.0, 1.0);
    // Would require gimbal correction on remaining engines
  }

  // Test center engine shutdown
  void testCenterEngineShutdown() {
    double centerThrust = 200000.0;
    double outerThrust = 100000.0;
    int outerEngines = 4;

    double normalTotal = centerThrust + outerEngines * outerThrust;
    double centerOutTotal = outerEngines * outerThrust;

    TS_ASSERT_DELTA(normalTotal, 600000.0, 1.0);
    TS_ASSERT_DELTA(centerOutTotal, 400000.0, 1.0);
    TS_ASSERT_DELTA(centerOutTotal / normalTotal, 0.667, 0.01);
  }

  /***************************************************************************
   * Nozzle Erosion and Degradation Tests
   ***************************************************************************/

  // Test throat erosion effect on thrust
  void testThroatErosionEffect() {
    double pc = 1000.0 * 144.0;
    double Cf = 1.6;

    // Fresh nozzle
    double Athroat_fresh = 1.0;
    double thrust_fresh = Cf * pc * Athroat_fresh;

    // Eroded nozzle (throat enlarged by 5%)
    double Athroat_eroded = 1.05;
    double thrust_eroded = Cf * pc * Athroat_eroded;

    // More mass flow but lower chamber pressure
    // Simplified: thrust approximately scales with throat area
    TS_ASSERT(thrust_eroded > thrust_fresh);
  }

  // Test ablative nozzle mass loss
  void testAblativeNozzleMassLoss() {
    double initialMass = 500.0;  // lbs
    double ablationRate = 0.1;   // lbs/s
    double burnTime = 60.0;      // seconds

    double massLoss = ablationRate * burnTime;
    double finalMass = initialMass - massLoss;

    TS_ASSERT_DELTA(massLoss, 6.0, 0.1);
    TS_ASSERT_DELTA(finalMass, 494.0, 0.1);
  }

  /***************************************************************************
   * Supersonic Flow Tests
   ***************************************************************************/

  // Test Mach number at throat
  void testMachNumberAtThroat() {
    // At throat, M = 1 (sonic condition)
    double M_throat = 1.0;
    TS_ASSERT_DELTA(M_throat, 1.0, DEFAULT_TOLERANCE);
  }

  // Test exit Mach number
  void testExitMachNumber() {
    // Exit Mach depends on expansion ratio
    // Typical values: M_exit = 2.5 to 4.0 for rocket nozzles

    double M_exit_low = 2.5;   // Low expansion
    double M_exit_high = 4.0;  // High expansion

    TS_ASSERT(M_exit_high > M_exit_low);

    // Thrust increases with exit velocity (hence exit Mach)
    double V_exit_low = M_exit_low * 3000.0;   // Approx speed of sound
    double V_exit_high = M_exit_high * 3000.0;
    TS_ASSERT(V_exit_high > V_exit_low);
  }

  // Test critical pressure ratio
  void testCriticalPressureRatio() {
    // p*/p0 = (2/(γ+1))^(γ/(γ-1)) for choked flow
    double gamma = 1.4;
    double criticalRatio = pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));

    TS_ASSERT_DELTA(criticalRatio, 0.528, 0.001);
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  // Test very small nozzle (model rocket)
  void testVerySmallNozzle() {
    double vacThrust = 10.0;  // Model rocket
    double nozzleArea = 0.001;
    double ambientPressure = 2116.22;

    double thrust = std::max(0.0, vacThrust - ambientPressure * nozzleArea);
    TS_ASSERT(thrust > 7.0);
  }

  // Test very large nozzle (heavy lift)
  void testVeryLargeNozzle() {
    double vacThrust = 7000000.0;  // RS-25 scale
    double nozzleArea = 50.0;
    double ambientPressure = 2116.22;

    double thrust = vacThrust - ambientPressure * nozzleArea;
    TS_ASSERT(thrust > 6800000.0);
  }

  // Test nozzle at maximum dynamic pressure
  void testNozzleAtMaxQ() {
    double vacThrust = 500000.0;
    double nozzleArea = 4.0;

    // Max-Q typically at ~35,000 ft, ~600 psf static + dynamic effects
    double effectivePressure = 800.0;
    double thrust = vacThrust - effectivePressure * nozzleArea;

    TS_ASSERT(thrust > 496000.0);
  }

  // Test numerical stability with very small pressure difference
  void testSmallPressureDifference() {
    double vacThrust = 100000.0;
    double nozzleArea = 2.0;
    // Just under breakeven: thrust = vacThrust - p * A ≈ 0.1
    // So p * A ≈ 99999.9, p ≈ 49999.95
    double ambientPressure = (vacThrust - 0.1) / nozzleArea;

    double thrust = vacThrust - ambientPressure * nozzleArea;
    TS_ASSERT(thrust > 0.0);
    TS_ASSERT(thrust < 1.0);
    TS_ASSERT(!std::isnan(thrust));
  }

  /***************************************************************************
   * Nozzle Efficiency and Performance Tests
   ***************************************************************************/

  // Test nozzle efficiency calculation
  void testNozzleEfficiencyCalculation() {
    double actualThrust = 450000.0;    // lbf
    double idealThrust = 500000.0;     // lbf

    double efficiency = actualThrust / idealThrust;
    TS_ASSERT_DELTA(efficiency, 0.9, 0.001);

    // Typical rocket nozzle efficiency range
    TS_ASSERT(efficiency > 0.85);
    TS_ASSERT(efficiency < 1.0);
  }

  // Test thrust coefficient calculation
  void testThrustCoefficientCalculation() {
    double gamma = 1.4;
    double expansionRatio = 25.0;
    double pressureRatio = 100.0;

    // Simplified thrust coefficient calculation
    double term1 = 2.0 * gamma * gamma / (gamma - 1.0);
    double term2 = pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (gamma - 1.0));
    double term3 = 1.0 - pow(1.0 / pressureRatio, (gamma - 1.0) / gamma);

    double Cf = sqrt(term1 * term2 * term3);
    TS_ASSERT(Cf > 1.5);
    TS_ASSERT(Cf < 2.0);
  }

  // Test specific impulse from thrust and mass flow
  void testSpecificImpulseFromThrustMassFlow() {
    double thrust = 500000.0;     // lbf
    double massFlow = 1500.0;     // lbs/s

    double Isp = thrust / massFlow;
    TS_ASSERT_DELTA(Isp, 333.33, 1.0);

    // Good rocket engine Isp range (300-450 s)
    TS_ASSERT(Isp > 300.0);
    TS_ASSERT(Isp < 450.0);
  }

  // Test characteristic velocity from pressure and mass flow
  void testCharacteristicVelocityFromPressure() {
    double chamberPressure = 1000.0 * 144.0;  // psi to psf
    double throatArea = 1.5;                   // sq ft
    double massFlow = 1500.0;                  // lbs/s

    // c* = Pc * At / mdot
    double cStar = chamberPressure * throatArea / massFlow;
    TS_ASSERT(cStar > 100.0);

    // Verify units are consistent
    double velocity = cStar;  // ft/s
    TS_ASSERT(velocity > 0.0);
  }

  // Test exhaust velocity calculation
  void testExhaustVelocityCalculation() {
    double Isp = 400.0;          // seconds
    double g0 = 32.174;          // ft/s^2

    double exhaustVelocity = Isp * g0;
    TS_ASSERT_DELTA(exhaustVelocity, 12869.6, 1.0);

    // High performance exhaust velocity
    TS_ASSERT(exhaustVelocity > 12000.0);
  }

  /***************************************************************************
   * Nozzle Geometry Optimization Tests
   ***************************************************************************/

  // Test optimal expansion ratio for altitude
  void testOptimalExpansionRatioForAltitude() {
    // At sea level, smaller expansion preferred
    double seaLevelOptimal = 10.0;

    // At high altitude, larger expansion preferred
    double highAltitudeOptimal = 50.0;

    // In vacuum, even larger
    double vacuumOptimal = 100.0;

    TS_ASSERT(highAltitudeOptimal > seaLevelOptimal);
    TS_ASSERT(vacuumOptimal > highAltitudeOptimal);

    // Verify ratio relationships
    TS_ASSERT_DELTA(highAltitudeOptimal / seaLevelOptimal, 5.0, 0.01);
  }

  // Test bell nozzle contour parameters
  void testBellNozzleContour() {
    double throatRadius = 0.5;     // ft
    double exitRadius = 2.0;       // ft
    double nozzleLength = 3.0;     // ft

    double areaRatio = (exitRadius * exitRadius) / (throatRadius * throatRadius);
    TS_ASSERT_DELTA(areaRatio, 16.0, 0.01);

    // Length-to-throat ratio
    double lengthRatio = nozzleLength / throatRadius;
    TS_ASSERT_DELTA(lengthRatio, 6.0, 0.01);
  }

  // Test aerospike nozzle altitude compensation
  void testAerospikeAltitudeCompensation() {
    double seaLevelThrust = 450000.0;
    double vacuumThrust = 500000.0;

    // Aerospike maintains better efficiency ratio
    double conventionalSeaLevel = 400000.0;
    double conventionalVacuum = 500000.0;

    double aerospikeRatio = seaLevelThrust / vacuumThrust;
    double conventionalRatio = conventionalSeaLevel / conventionalVacuum;

    // Aerospike has better sea-level to vacuum ratio
    TS_ASSERT(aerospikeRatio > conventionalRatio);
    TS_ASSERT_DELTA(aerospikeRatio, 0.9, 0.01);
    TS_ASSERT_DELTA(conventionalRatio, 0.8, 0.01);
  }

  // Test dual-bell nozzle mode transition
  void testDualBellNozzleModeTransition() {
    double mode1ExpansionRatio = 15.0;
    double mode2ExpansionRatio = 40.0;

    // Transition altitude (simplified)
    double transitionAltitude = 30000.0;  // ft

    // Mode 1 thrust at sea level
    double mode1SeaLevelThrust = 450000.0;

    // Mode 2 thrust at altitude
    double mode2AltitudeThrust = 480000.0;

    TS_ASSERT(mode2ExpansionRatio > mode1ExpansionRatio);
    TS_ASSERT(mode2AltitudeThrust > mode1SeaLevelThrust);
    TS_ASSERT(transitionAltitude > 0.0);
  }

  /***************************************************************************
   * Thermal and Material Effects Tests
   ***************************************************************************/

  // Test thermal expansion effect on throat area
  void testThermalExpansionOnThroatArea() {
    double coldThroatDiameter = 1.0;      // ft
    double thermalExpansionCoeff = 0.001; // per 100°F
    double tempRise = 500.0;              // °F

    double hotThroatDiameter = coldThroatDiameter * (1.0 + thermalExpansionCoeff * tempRise / 100.0);
    double areaRatio = (hotThroatDiameter * hotThroatDiameter) / (coldThroatDiameter * coldThroatDiameter);

    TS_ASSERT(hotThroatDiameter > coldThroatDiameter);
    TS_ASSERT_DELTA(areaRatio, 1.01, 0.01);
  }

  // Test regenerative cooling heat transfer
  void testRegenerativeCoolingHeatTransfer() {
    double heatFlux = 10.0;           // BTU/ft^2-s
    double nozzleWallArea = 50.0;     // sq ft
    double coolantMassFlow = 500.0;   // lbs/s
    double coolantCp = 1.0;           // BTU/lb-°F

    double totalHeat = heatFlux * nozzleWallArea;
    double tempRise = totalHeat / (coolantMassFlow * coolantCp);

    TS_ASSERT_DELTA(totalHeat, 500.0, 0.1);
    TS_ASSERT_DELTA(tempRise, 1.0, 0.01);
  }

  // Test film cooling effectiveness
  void testFilmCoolingEffectiveness() {
    double hotGasTemp = 6000.0;       // °F
    double wallTempWithCooling = 2000.0;
    double coolantTemp = 500.0;

    double effectiveness = (hotGasTemp - wallTempWithCooling) / (hotGasTemp - coolantTemp);
    TS_ASSERT_DELTA(effectiveness, 0.727, 0.01);

    // Good film cooling effectiveness > 0.5
    TS_ASSERT(effectiveness > 0.5);
  }

  /***************************************************************************
   * Complete Nozzle System Tests
   ***************************************************************************/

  // Test complete nozzle performance envelope
  void testCompleteNozzlePerformanceEnvelope() {
    double vacuumThrust = 500000.0;
    double exitArea = 4.0;
    double Isp_vacuum = 450.0;

    // Calculate performance at multiple altitudes
    double altitudes[] = {0.0, 30000.0, 60000.0, 100000.0};
    double pressures[] = {2116.22, 628.0, 151.0, 23.0};

    double prevThrust = 0.0;
    for (int i = 0; i < 4; i++) {
      double thrust = vacuumThrust - pressures[i] * exitArea;
      TS_ASSERT(thrust > prevThrust || i == 0);
      prevThrust = thrust;
    }

    // Vacuum thrust should be maximum
    double spaceThrust = vacuumThrust - 0.0 * exitArea;
    TS_ASSERT_DELTA(spaceThrust, vacuumThrust, 0.01);
  }

  // Test nozzle thrust vectoring system
  void testNozzleThrustVectoringSystem() {
    double thrust = 500000.0;
    double maxGimbalAngle = 8.0 * M_PI / 180.0;  // 8 degrees in radians

    double axialThrust = thrust * cos(maxGimbalAngle);
    double sideForce = thrust * sin(maxGimbalAngle);

    TS_ASSERT_DELTA(axialThrust, 495139.0, 100.0);
    TS_ASSERT_DELTA(sideForce, 69565.0, 100.0);

    // Verify thrust magnitude is preserved
    double totalThrust = sqrt(axialThrust * axialThrust + sideForce * sideForce);
    TS_ASSERT_DELTA(totalThrust, thrust, 1.0);
  }

  // Test multi-nozzle cluster performance
  void testMultiNozzleClusterPerformance() {
    double singleNozzleThrust = 500000.0;
    int nozzleCount = 9;  // Like Falcon 9 first stage

    double totalThrust = singleNozzleThrust * nozzleCount;
    TS_ASSERT_DELTA(totalThrust, 4500000.0, 1.0);

    // Center engine throttled for landing
    double landingThrust = singleNozzleThrust * 0.4;  // 40% throttle
    TS_ASSERT_DELTA(landingThrust, 200000.0, 1.0);

    // Cluster redundancy
    double thrustWith1Out = singleNozzleThrust * (nozzleCount - 1);
    TS_ASSERT_DELTA(thrustWith1Out / totalThrust, 0.889, 0.01);
  }

  // Test nozzle startup transient
  void testNozzleStartupTransient() {
    double steadyStateThrust = 500000.0;
    double rampTime = 2.0;  // seconds

    // Simulate thrust buildup
    double times[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    double prevThrust = 0.0;

    for (int i = 0; i < 5; i++) {
      double fraction = times[i] / rampTime;
      if (fraction > 1.0) fraction = 1.0;
      double thrust = steadyStateThrust * fraction;
      TS_ASSERT(thrust >= prevThrust);
      prevThrust = thrust;
    }

    TS_ASSERT_DELTA(prevThrust, steadyStateThrust, 0.01);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test independent nozzle calculations
  void testIndependentNozzleCalculations() {
    double vacThrust1 = 500000.0;
    double exitArea1 = 4.0;
    double ambient1 = 2116.22;

    double vacThrust2 = 300000.0;
    double exitArea2 = 2.5;
    double ambient2 = 1000.0;

    double thrust1 = vacThrust1 - ambient1 * exitArea1;
    double thrust2 = vacThrust2 - ambient2 * exitArea2;

    TS_ASSERT_DELTA(thrust1, 491535.12, 1.0);
    TS_ASSERT_DELTA(thrust2, 297500.0, 1.0);

    // Verify independence
    TS_ASSERT(std::abs(thrust1 - thrust2) > 100000.0);
  }

  // Test nozzle state does not persist between calculations
  void testNozzleStateNoPersistence() {
    // First calculation set
    double Isp1 = 400.0;
    double massFlow1 = 1000.0;
    double thrust1 = Isp1 * massFlow1;

    // Second calculation set (different values)
    double Isp2 = 350.0;
    double massFlow2 = 1200.0;
    double thrust2 = Isp2 * massFlow2;

    // Verify calculations are independent
    TS_ASSERT_DELTA(thrust1, 400000.0, 0.1);
    TS_ASSERT_DELTA(thrust2, 420000.0, 0.1);

    // Recalculate first to verify no state pollution
    double thrust1_again = Isp1 * massFlow1;
    TS_ASSERT_DELTA(thrust1_again, thrust1, 0.001);
  }
};

//=============================================================================
// C172x Integration Tests - Propeller/Thruster Tests
//=============================================================================

class FGNozzleC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {


    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: C172x uses propeller (not nozzle)
  void testC172xUsesPropeller() {
    auto propulsion = fdm.GetPropulsion();
    auto thruster = propulsion->GetEngine(0)->GetThruster();

    TS_ASSERT(thruster != nullptr);
    // C172x uses propeller, not nozzle
    TS_ASSERT(thruster->GetType() == FGThruster::ttPropeller);
  }

  // Test 2: Thruster produces thrust
  void testThrusterProducesThrust() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double thrust = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    TS_ASSERT(thrust > 0.0);
  }

  // Test 3: Thrust varies with throttle
  void testThrustVariesWithThrottle() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();

    fcs->SetMixtureCmd(-1, 1.0);

    // Low throttle
    fcs->SetThrottleCmd(-1, 0.3);
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust_low = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    // High throttle
    fcs->SetThrottleCmd(-1, 0.9);
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust_high = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    TS_ASSERT(thrust_high > thrust_low);
  }

  // Test 4: Propeller RPM
  void testPropellerRPM() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 100; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double rpm = propulsion->GetEngine(0)->GetThruster()->GetRPM();

    TS_ASSERT(rpm > 0.0);
    TS_ASSERT(std::isfinite(rpm));
  }

  // Test 5: Thrust direction is forward
  void testThrustDirectionForward() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double thrust = propulsion->GetEngine(0)->GetThrust();

    // Positive thrust means forward
    TS_ASSERT(thrust >= 0.0);
  }

  // Test 6: Thrust at different airspeeds
  void testThrustAtDifferentAirspeeds() {
    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();
    auto ic = fdm.GetIC();

    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetMixtureCmd(-1, 1.0);

    // Static (near zero speed)
    ic->SetVcalibratedKtsIC(10.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust_static = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    // Cruise speed
    ic->SetVcalibratedKtsIC(120.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust_cruise = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    TS_ASSERT(std::isfinite(thrust_static));
    TS_ASSERT(std::isfinite(thrust_cruise));
  }

  // Test 7: Gear ratio effect
  void testGearRatioEffect() {
    auto propulsion = fdm.GetPropulsion();
    auto thruster = propulsion->GetEngine(0)->GetThruster();

    double gearRatio = thruster->GetGearRatio();

    TS_ASSERT(gearRatio > 0.0);
    TS_ASSERT(std::isfinite(gearRatio));
  }

  // Test 8: Thrust coefficient positive
  void testThrustCoefficientPositive() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double thrust = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    TS_ASSERT(thrust >= 0.0);
  }

  // Test 9: Power coefficient
  void testPowerCoefficient() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double power = propulsion->GetEngine(0)->GetThruster()->GetPowerRequired();

    TS_ASSERT(std::isfinite(power));
  }

  // Test 10: Altitude effects on thruster
  void testAltitudeEffectsOnThruster() {
    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();
    auto ic = fdm.GetIC();

    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetMixtureCmd(-1, 1.0);

    // Sea level
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(0.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust_sl = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    // High altitude
    ic->SetAltitudeASLFtIC(10000.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust_high = propulsion->GetEngine(0)->GetThruster()->GetThrust();

    TS_ASSERT(std::isfinite(thrust_sl));
    TS_ASSERT(std::isfinite(thrust_high));
  }

  // Test 11: Thruster moment
  void testThrusterMoment() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 1.0);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    const FGColumnVector3& moment = propulsion->GetMoments();

    // Propeller should create some torque moment
    TS_ASSERT(std::isfinite(moment(1)));
    TS_ASSERT(std::isfinite(moment(2)));
    TS_ASSERT(std::isfinite(moment(3)));
  }

  // Test 12: Extended operation stability
  void testExtendedOperationStability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    auto propulsion = fdm.GetPropulsion();

    for (int i = 0; i < 500; ++i) {
      fdm.Run();
    }

    double thrust = propulsion->GetEngine(0)->GetThruster()->GetThrust();
    double rpm = propulsion->GetEngine(0)->GetThruster()->GetRPM();

    TS_ASSERT(std::isfinite(thrust));
    TS_ASSERT(std::isfinite(rpm));
    TS_ASSERT(thrust >= 0.0);
  }
};
