#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include "TestUtilities.h"

using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * Buoyant Forces unit tests
 *
 * Tests for buoyancy physics concepts including:
 * - Archimedes principle (buoyancy = weight of displaced fluid)
 * - Gas properties (hydrogen, helium, hot air)
 * - Pressure effects on gas volume
 * - Temperature effects on gas density
 * - Ballonet operation
 * - Valve release calculations
 */
class FGBuoyantForcesTest : public CxxTest::TestSuite
{
public:
  // Physical constants
  static constexpr double R_universal = 8314.32;   // J/(kmol*K)
  static constexpr double M_air = 28.97;           // kg/kmol
  static constexpr double M_hydrogen = 2.016;      // kg/kmol
  static constexpr double M_helium = 4.003;        // kg/kmol
  static constexpr double rho_air_SL = 1.225;      // kg/m^3 at sea level
  static constexpr double g = 9.80665;             // m/s^2
  static constexpr double slugs_per_kg = 0.0685218;
  static constexpr double lbs_per_N = 0.224809;
  static constexpr double ft3_per_m3 = 35.3147;

  // Test Archimedes principle
  void testArchimedesPrinciple() {
    // Buoyancy = weight of displaced air
    double volume_m3 = 1000.0;  // 1000 cubic meters
    double rho_air = 1.225;     // kg/m^3

    double displaced_mass = rho_air * volume_m3;  // kg
    double buoyancy_N = displaced_mass * g;       // Newtons

    TS_ASSERT_DELTA(displaced_mass, 1225.0, 0.1);
    TS_ASSERT_DELTA(buoyancy_N, 12012.65, 1.0);
  }

  // Test net lift for hydrogen
  void testHydrogenNetLift() {
    double volume_m3 = 1000.0;
    double T_kelvin = 288.15;   // Standard temp (15C)
    double P_pascal = 101325.0; // Standard pressure

    // Air density
    double rho_air = P_pascal * M_air / (R_universal * T_kelvin);

    // Hydrogen density
    double rho_h2 = P_pascal * M_hydrogen / (R_universal * T_kelvin);

    // Net lift = (rho_air - rho_gas) * V * g
    double net_lift_N = (rho_air - rho_h2) * volume_m3 * g;
    double net_lift_kg = net_lift_N / g;

    // Hydrogen provides ~1.1 kg lift per m^3
    TS_ASSERT_DELTA(net_lift_kg, 1100.0, 50.0);
  }

  // Test net lift for helium
  void testHeliumNetLift() {
    double volume_m3 = 1000.0;
    double T_kelvin = 288.15;
    double P_pascal = 101325.0;

    double rho_air = P_pascal * M_air / (R_universal * T_kelvin);
    double rho_he = P_pascal * M_helium / (R_universal * T_kelvin);

    double net_lift_N = (rho_air - rho_he) * volume_m3 * g;
    double net_lift_kg = net_lift_N / g;

    // Helium provides ~1.0 kg lift per m^3 (less than hydrogen)
    TS_ASSERT_DELTA(net_lift_kg, 1050.0, 50.0);
    TS_ASSERT(net_lift_kg < 1100.0);  // Less than hydrogen
  }

  // Test hydrogen vs helium lift comparison
  void testHydrogenVsHeliumLift() {
    double volume_m3 = 1000.0;
    double T_kelvin = 288.15;
    double P_pascal = 101325.0;

    double rho_air = P_pascal * M_air / (R_universal * T_kelvin);
    double rho_h2 = P_pascal * M_hydrogen / (R_universal * T_kelvin);
    double rho_he = P_pascal * M_helium / (R_universal * T_kelvin);

    double lift_h2 = (rho_air - rho_h2) * volume_m3;
    double lift_he = (rho_air - rho_he) * volume_m3;

    // Hydrogen provides about 8% more lift than helium
    double ratio = lift_h2 / lift_he;
    TS_ASSERT_DELTA(ratio, 1.08, 0.02);
  }

  // Test ideal gas law
  void testIdealGasLaw() {
    // PV = nRT => rho = PM/(RT)
    double P = 101325.0;  // Pa
    double T = 288.15;    // K
    double M = 28.97;     // kg/kmol for air

    double rho = P * M / (R_universal * T);
    TS_ASSERT_DELTA(rho, 1.225, 0.01);  // Should match standard air density
  }

  // Test pressure effect on gas volume
  void testPressureVolumeRelation() {
    // PV = constant at constant T (Boyle's law)
    double P1 = 101325.0;
    double V1 = 1000.0;

    double P2 = 2 * P1;  // Double pressure
    double V2 = (P1 * V1) / P2;

    TS_ASSERT_DELTA(V2, 500.0, epsilon);  // Half volume
  }

  // Test temperature effect on gas volume
  void testTemperatureVolumeRelation() {
    // V/T = constant at constant P (Charles's law)
    double T1 = 288.15;  // 15C
    double V1 = 1000.0;

    double T2 = 303.15;  // 30C
    double V2 = V1 * (T2 / T1);

    TS_ASSERT_DELTA(V2, 1052.1, 0.5);  // Volume increases with temperature
  }

  // Test superheat effect on hot air balloon
  void testHotAirBalloon() {
    double volume_m3 = 2800.0;  // Typical hot air balloon
    double P_pascal = 101325.0;
    double T_ambient = 288.15;  // 15C
    double T_heated = 373.15;   // 100C

    // Air densities using ideal gas law
    double rho_cold = P_pascal * M_air / (R_universal * T_ambient);
    double rho_hot = P_pascal * M_air / (R_universal * T_heated);

    // Net lift = (rho_cold - rho_hot) * volume
    // With these values: (1.225 - 0.946) * 2800 ≈ 781 kg
    double net_lift_kg = (rho_cold - rho_hot) * volume_m3;

    // Hot air balloon provides significant lift
    TS_ASSERT(net_lift_kg > 700.0);
    TS_ASSERT(net_lift_kg < 900.0);
  }

  // Test altitude effect on air density
  void testAltitudeAirDensity() {
    // Approximate: rho = rho_0 * exp(-h/H) where H ≈ 8500m
    double H = 8500.0;  // Scale height
    double altitudes[] = {0, 1000, 5000, 10000};
    double expected_ratios[] = {1.0, 0.889, 0.558, 0.311};

    for (int i = 0; i < 4; i++) {
      double ratio = std::exp(-altitudes[i] / H);
      TS_ASSERT_DELTA(ratio, expected_ratios[i], 0.02);
    }
  }

  // Test gas cell overpressure
  void testGasCellOverpressure() {
    // When internal pressure exceeds external + max_overpressure, valve opens
    double P_external = 101325.0;
    double max_overpressure = 340.0;  // Pa
    double valve_threshold = P_external + max_overpressure;

    TS_ASSERT_DELTA(valve_threshold, 101665.0, epsilon);

    // Test valve states
    double P_internal_low = 101400.0;  // Below threshold
    double P_internal_high = 102000.0; // Above threshold

    bool valve_open_low = (P_internal_low > valve_threshold);
    bool valve_open_high = (P_internal_high > valve_threshold);

    TS_ASSERT(!valve_open_low);
    TS_ASSERT(valve_open_high);
  }

  // Test valve mass flow rate
  void testValveMassFlowRate() {
    // Mass flow rate = valve_coeff * sqrt(delta_P * rho)
    double valve_coeff = 0.015;  // M4*SEC/KG
    double delta_P = 100.0;      // Pa
    double rho_gas = 0.0899;     // kg/m^3 (hydrogen)

    double mass_flow = valve_coeff * std::sqrt(delta_P * rho_gas);
    TS_ASSERT(mass_flow > 0.0);
    TS_ASSERT(mass_flow < 1.0);  // Reasonable range
  }

  // Test ballonet operation concept
  void testBallonetOperation() {
    // Ballonet maintains envelope shape as altitude changes
    double total_volume = 10000.0;  // m^3
    double gas_volume_low = 9000.0; // At low altitude
    double ballonet_volume_low = total_volume - gas_volume_low;

    TS_ASSERT_DELTA(ballonet_volume_low, 1000.0, epsilon);

    // At high altitude, gas expands, ballonet compresses
    double gas_volume_high = 9800.0;
    double ballonet_volume_high = total_volume - gas_volume_high;

    TS_ASSERT_DELTA(ballonet_volume_high, 200.0, epsilon);
    TS_ASSERT(ballonet_volume_high < ballonet_volume_low);
  }

  // Test gas mass calculation
  void testGasMassCalculation() {
    double volume_m3 = 1000.0;
    double T_kelvin = 288.15;
    double P_pascal = 101325.0;

    // Hydrogen mass
    double rho_h2 = P_pascal * M_hydrogen / (R_universal * T_kelvin);
    double mass_h2 = rho_h2 * volume_m3;

    // Helium mass
    double rho_he = P_pascal * M_helium / (R_universal * T_kelvin);
    double mass_he = rho_he * volume_m3;

    // Helium is about twice as heavy as hydrogen
    TS_ASSERT_DELTA(mass_he / mass_h2, 2.0, 0.1);
    TS_ASSERT(mass_h2 < 100.0);  // Hydrogen is very light
    TS_ASSERT(mass_he < 200.0);  // Helium is still light
  }

  // Test ellipsoid volume calculation
  void testEllipsoidVolume() {
    // V = 4/3 * pi * a * b * c for ellipsoid with semi-axes a, b, c
    double a = 22.86;  // x-radius in meters
    double b = 4.55;   // y-radius
    double c = 4.55;   // z-radius

    double volume = (4.0/3.0) * M_PI * a * b * c;

    // LZ-120 type envelope
    TS_ASSERT(volume > 1900.0);
    TS_ASSERT(volume < 2100.0);
  }

  // Test moment from gas cell
  void testGasCellMoment() {
    // Moment = force × arm
    double buoyancy_N = 10000.0;
    double arm_m = 2.0;  // Offset from CG

    double moment_Nm = buoyancy_N * arm_m;
    TS_ASSERT_DELTA(moment_Nm, 20000.0, epsilon);
  }

  // Test center of buoyancy shift
  void testCenterOfBuoyancyShift() {
    // CB moves as gas redistributes
    double volume = 1000.0;
    double x_initial = 10.0;
    double x_final = 12.0;

    double shift = x_final - x_initial;
    TS_ASSERT_DELTA(shift, 2.0, epsilon);
  }

  // Test pressure altitude relation
  void testPressureAltitude() {
    // Standard atmosphere: P = P0 * (1 - 0.0000226h)^5.256
    double P0 = 101325.0;
    double h = 1000.0;  // meters

    double P = P0 * std::pow(1.0 - 0.0000226 * h, 5.256);
    TS_ASSERT_DELTA(P, 89874.0, 100.0);  // Pressure at 1000m
  }

  // Test gas density ratio
  void testGasDensityRatio() {
    // Density ratio = M_gas / M_air
    double ratio_h2 = M_hydrogen / M_air;
    double ratio_he = M_helium / M_air;

    TS_ASSERT_DELTA(ratio_h2, 0.0696, 0.001);  // Hydrogen is ~7% of air
    TS_ASSERT_DELTA(ratio_he, 0.138, 0.001);   // Helium is ~14% of air
  }

  // Test superpressure
  void testSuperpressure() {
    // Superpressure keeps balloon inflated at altitude
    double external_P = 70000.0;  // At altitude
    double internal_P = 70500.0;  // Slightly higher
    double superpressure = internal_P - external_P;

    TS_ASSERT_DELTA(superpressure, 500.0, epsilon);
  }

  // Test weight vs buoyancy equilibrium
  void testEquilibrium() {
    double buoyancy = 10000.0;  // lbs
    double weight = 9500.0;     // lbs

    double net_lift = buoyancy - weight;
    bool ascending = net_lift > 0;

    TS_ASSERT(ascending);
    TS_ASSERT_DELTA(net_lift, 500.0, epsilon);
  }

  // Test lift coefficient
  void testLiftCoefficient() {
    // Gross lift per unit volume
    double volume_m3 = 1000.0;
    double gross_lift_N = 11000.0;

    double lift_per_m3 = gross_lift_N / volume_m3;
    TS_ASSERT_DELTA(lift_per_m3, 11.0, epsilon);
  }

  // Test temperature sensitivity
  void testTemperatureSensitivity() {
    double volume = 1000.0;
    double P = 101325.0;
    double T1 = 288.15;
    double T2 = 278.15;  // 10K colder

    double rho1 = P * M_helium / (R_universal * T1);
    double rho2 = P * M_helium / (R_universal * T2);

    // Colder gas is denser
    TS_ASSERT(rho2 > rho1);
    double ratio = rho2 / rho1;
    TS_ASSERT_DELTA(ratio, 1.036, 0.01);  // About 3.6% denser
  }

  // Test unit conversions
  void testUnitConversions() {
    double volume_m3 = 28316.85;  // 1 million cubic feet
    double volume_ft3 = volume_m3 * ft3_per_m3;
    TS_ASSERT_DELTA(volume_ft3, 1000000.0, 100.0);

    double mass_kg = 100.0;
    double mass_slugs = mass_kg * slugs_per_kg;
    TS_ASSERT_DELTA(mass_slugs, 6.852, 0.01);
  }

  // Test multiple gas cell summation
  void testMultipleGasCells() {
    // Total forces are sum of individual cells
    double forces[3] = {1000.0, 1500.0, 2000.0};
    double total = 0.0;
    for (double f : forces) {
      total += f;
    }
    TS_ASSERT_DELTA(total, 4500.0, epsilon);
  }

  // Test inertia from gas mass
  void testGasInertia() {
    // Simple approximation: I = m * r^2
    double mass = 100.0;   // slugs
    double radius = 10.0;  // ft

    double I = mass * radius * radius;
    TS_ASSERT_DELTA(I, 10000.0, epsilon);  // slug*ft^2
  }
};
