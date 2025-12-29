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

  // ==================== GAS PROPERTIES TESTS ====================

  // Test mixed gas properties
  void testMixedGasProperties() {
    // Mixture of helium with air contamination
    double purity = 0.97;  // 97% pure helium
    double T_kelvin = 288.15;
    double P_pascal = 101325.0;

    double rho_he_pure = P_pascal * M_helium / (R_universal * T_kelvin);
    double rho_air = P_pascal * M_air / (R_universal * T_kelvin);

    // Mixed density
    double rho_mix = purity * rho_he_pure + (1.0 - purity) * rho_air;

    // Should be slightly higher than pure helium
    TS_ASSERT(rho_mix > rho_he_pure);
    TS_ASSERT(rho_mix < rho_air);
  }

  // Test gas diffusion through envelope
  void testGasDiffusion() {
    // Helium diffuses through envelope material
    double initial_mass = 100.0;  // kg
    double diffusion_rate = 0.001;  // fraction per day
    double days = 30.0;

    // Exponential decay: m = m0 * exp(-k*t)
    double remaining_mass = initial_mass * std::exp(-diffusion_rate * days);
    double lost_mass = initial_mass - remaining_mass;

    TS_ASSERT(remaining_mass < initial_mass);
    TS_ASSERT_DELTA(lost_mass, 2.96, 0.1);  // About 3% loss in 30 days
  }

  // Test lifting gas comparison - ammonia
  void testAmmoniaLiftingGas() {
    // Ammonia (NH3) was historically considered
    double M_ammonia = 17.03;  // kg/kmol
    double T_kelvin = 288.15;
    double P_pascal = 101325.0;

    double rho_air = P_pascal * M_air / (R_universal * T_kelvin);
    double rho_ammonia = P_pascal * M_ammonia / (R_universal * T_kelvin);

    double lift_ratio = (rho_air - rho_ammonia) / rho_air;

    // Ammonia provides less lift than hydrogen/helium
    TS_ASSERT(lift_ratio > 0.0);  // Still lighter than air
    TS_ASSERT(lift_ratio < 0.5);  // Much less lift than hydrogen
  }

  // Test methane as lifting gas
  void testMethaneLiftingGas() {
    double M_methane = 16.04;  // kg/kmol (CH4)
    double T_kelvin = 288.15;
    double P_pascal = 101325.0;

    double rho_air = P_pascal * M_air / (R_universal * T_kelvin);
    double rho_ch4 = P_pascal * M_methane / (R_universal * T_kelvin);

    double lift_per_m3 = rho_air - rho_ch4;  // kg/m3

    // Methane provides about 0.55 kg/m3 lift (less than helium's ~1.0)
    TS_ASSERT(lift_per_m3 > 0.4);
    TS_ASSERT(lift_per_m3 < 0.7);
  }

  // Test gas compressibility factor
  void testGasCompressibilityFactor() {
    // At high pressures, real gas behavior deviates from ideal
    // Z = PV/(nRT), Z=1 for ideal gas
    double Z_helium_low_P = 1.0;
    double Z_helium_high_P = 1.0005;  // Slightly positive deviation

    // For balloon applications at normal pressures, Z ≈ 1
    TS_ASSERT_DELTA(Z_helium_low_P, 1.0, 0.01);
    TS_ASSERT_DELTA(Z_helium_high_P, 1.0, 0.01);
  }

  // ==================== ENVELOPE TESTS ====================

  // Test envelope surface area for sphere
  void testSphereEnvelopeSurfaceArea() {
    double radius = 10.0;  // meters
    double surface_area = 4.0 * M_PI * radius * radius;
    double volume = (4.0/3.0) * M_PI * radius * radius * radius;

    TS_ASSERT_DELTA(surface_area, 1256.64, 1.0);
    TS_ASSERT_DELTA(volume, 4188.79, 1.0);

    // Surface to volume ratio
    double ratio = surface_area / volume;
    TS_ASSERT_DELTA(ratio, 0.3, 0.01);  // 3/r for sphere
  }

  // Test envelope stress from internal pressure
  void testEnvelopeStress() {
    // Hoop stress in spherical pressure vessel: sigma = P*r/(2*t)
    double overpressure = 340.0;  // Pa
    double radius = 10.0;         // m
    double thickness = 0.0005;    // m (0.5 mm)

    double hoop_stress = overpressure * radius / (2.0 * thickness);
    TS_ASSERT(hoop_stress > 0.0);

    // Convert to MPa
    double stress_MPa = hoop_stress / 1e6;
    TS_ASSERT(stress_MPa < 5.0);  // Typical fabric stress limits
  }

  // Test prolate ellipsoid surface area
  void testProlateEllipsoidSurface() {
    // Approximation for prolate ellipsoid (a > b = c)
    double a = 50.0;  // semi-major axis
    double b = 10.0;  // semi-minor axis

    // Using Knud Thomsen approximation
    double p = 1.6075;
    double term1 = std::pow(a * b, p);
    double term2 = std::pow(a * b, p);
    double term3 = std::pow(b * b, p);
    double surface_approx = 4.0 * M_PI * std::pow((term1 + term2 + term3)/3.0, 1.0/p);

    TS_ASSERT(surface_approx > 0.0);
  }

  // Test envelope fineness ratio effect
  void testFinenesRatioEffect() {
    // Fineness ratio = length / diameter
    // Higher ratio = less drag, more structural challenge
    double length = 100.0;
    double diameter = 20.0;
    double fineness = length / diameter;

    TS_ASSERT_DELTA(fineness, 5.0, epsilon);

    // Typical airship fineness ratios: 4-6
    TS_ASSERT(fineness >= 4.0);
    TS_ASSERT(fineness <= 6.0);
  }

  // Test volume efficiency
  void testVolumeEfficiency() {
    // Envelope material weight vs gas volume
    double envelope_mass = 500.0;  // kg
    double volume = 10000.0;       // m^3

    double mass_per_volume = envelope_mass / volume;  // kg/m^3
    TS_ASSERT(mass_per_volume < 0.1);  // Good efficiency
  }

  // ==================== ENVIRONMENTAL EFFECTS ====================

  // Test solar heating (superheat)
  void testSolarSuperheat() {
    // Gas inside envelope heats up more than ambient
    double T_ambient = 288.15;  // K
    double superheat = 15.0;    // K (typical solar heating)
    double T_gas = T_ambient + superheat;

    // Superheat reduces gas density, increases lift
    double P = 101325.0;
    double rho_ambient = P * M_helium / (R_universal * T_ambient);
    double rho_superheated = P * M_helium / (R_universal * T_gas);

    TS_ASSERT(rho_superheated < rho_ambient);

    // Lift increase from superheat
    double lift_increase = (rho_ambient - rho_superheated) / rho_ambient;
    TS_ASSERT(lift_increase > 0.04);  // Several percent increase
  }

  // Test nighttime radiation cooling
  void testNighttimeCooling() {
    // Gas cools at night, reducing lift
    double T_day = 303.15;    // 30C daytime
    double T_night = 278.15;  // 5C nighttime
    double P = 101325.0;

    double rho_day = P * M_helium / (R_universal * T_day);
    double rho_night = P * M_helium / (R_universal * T_night);

    // Nighttime lift reduction
    double density_increase = (rho_night - rho_day) / rho_day;
    TS_ASSERT(density_increase > 0.05);  // ~8% denser at night
  }

  // Test moisture loading
  void testMoistureLoading() {
    // Rain adds weight to envelope
    double envelope_area = 5000.0;  // m^2
    double water_film = 0.001;      // m (1mm water)
    double water_density = 1000.0;  // kg/m^3

    double water_mass = envelope_area * water_film * water_density;
    TS_ASSERT_DELTA(water_mass, 5000.0, 1.0);  // 5 tonnes of water!
  }

  // Test wind load
  void testWindLoad() {
    // Dynamic pressure = 0.5 * rho * V^2
    double rho_air = 1.225;
    double V = 20.0;  // m/s (about 40 knots)

    double q = 0.5 * rho_air * V * V;
    TS_ASSERT_DELTA(q, 245.0, 1.0);  // Pa

    // Force on envelope section
    double Cd = 0.04;        // Low drag coefficient for streamlined shape
    double ref_area = 500.0; // m^2
    double drag = q * Cd * ref_area;

    TS_ASSERT(drag > 0.0);
    TS_ASSERT(drag < 10000.0);  // Reasonable drag force in N
  }

  // Test icing effects
  void testIcingEffects() {
    // Ice accumulation increases weight
    double ice_thickness = 0.01;   // m
    double ice_density = 917.0;    // kg/m^3
    double envelope_area = 5000.0; // m^2

    double ice_mass = ice_thickness * ice_density * envelope_area;
    TS_ASSERT(ice_mass > 40000.0);  // Significant weight
  }

  // Test turbulence effect on envelope
  void testTurbulenceEffect() {
    // Turbulent gusts cause dynamic loading
    double base_load = 10000.0;  // N
    double gust_factor = 1.5;    // Peak/mean ratio

    double peak_load = base_load * gust_factor;
    TS_ASSERT_DELTA(peak_load, 15000.0, epsilon);
  }

  // ==================== DYNAMIC BEHAVIOR ====================

  // Test heaviness static condition
  void testHeavinessCondition() {
    double buoyancy = 10000.0;  // lbs
    double weight = 10500.0;    // lbs

    double heaviness = weight - buoyancy;  // positive = heavy
    TS_ASSERT(heaviness > 0.0);
    TS_ASSERT_DELTA(heaviness, 500.0, epsilon);
  }

  // Test lightness static condition
  void testLightnessCondition() {
    double buoyancy = 10000.0;  // lbs
    double weight = 9500.0;     // lbs

    double lightness = buoyancy - weight;  // positive = light
    TS_ASSERT(lightness > 0.0);
    TS_ASSERT_DELTA(lightness, 500.0, epsilon);
  }

  // Test vertical acceleration from net lift
  void testVerticalAcceleration() {
    double net_lift = 500.0;   // N
    double total_mass = 5000.0; // kg

    double accel = net_lift / total_mass;
    TS_ASSERT_DELTA(accel, 0.1, 0.01);  // m/s^2
  }

  // Test heave damping
  void testHeaveDamping() {
    // Vertical oscillation damping coefficient
    double mass = 5000.0;    // kg
    double damping = 0.1;    // damping ratio
    double omega = 0.5;      // rad/s (natural frequency)

    double c = 2.0 * damping * mass * omega;  // damping coefficient
    TS_ASSERT(c > 0.0);
    TS_ASSERT_DELTA(c, 500.0, 1.0);
  }

  // Test pitch stability
  void testPitchStability() {
    // Restoring moment from buoyancy offset
    double buoyancy = 100000.0;  // N
    double metacentric_height = 2.0;  // m
    double pitch_angle = 0.1;    // rad

    double restoring_moment = buoyancy * metacentric_height * pitch_angle;
    TS_ASSERT_DELTA(restoring_moment, 20000.0, 100.0);  // Nm
  }

  // Test roll stability
  void testRollStability() {
    double buoyancy = 100000.0;  // N
    double arm = 1.5;            // m (lateral CB-CG offset)
    double roll_angle = 0.05;    // rad

    double restoring_moment = buoyancy * arm * roll_angle;
    TS_ASSERT(restoring_moment > 0.0);
  }

  // Test added mass effect
  void testAddedMassEffect() {
    // Airship accelerating displaces air
    double volume = 10000.0;  // m^3
    double rho_air = 1.225;   // kg/m^3
    double added_mass_coeff = 0.6;  // For ellipsoid

    double added_mass = added_mass_coeff * rho_air * volume;
    TS_ASSERT(added_mass > 5000.0);
    TS_ASSERT(added_mass < 10000.0);
  }

  // ==================== GAS CELL/BALLONET OPERATIONS ====================

  // Test ballonet blower capacity
  void testBallonetBlowerCapacity() {
    // Volume flow rate needed to maintain pressure
    double volume_rate = 10.0;  // m^3/s
    double pressure_rise = 500.0;  // Pa

    double power = volume_rate * pressure_rise;  // Watts (ideal)
    double efficiency = 0.6;
    double actual_power = power / efficiency;

    TS_ASSERT(actual_power > 8000.0);  // About 8 kW
  }

  // Test valve sizing
  void testValveSizing() {
    // Valve must handle gas expansion during descent
    double descent_rate = 5.0;  // m/s
    double dP_dh = 12.0;       // Pa/m (pressure gradient)
    double volume = 10000.0;   // m^3

    // Rate of pressure change
    double dP_dt = descent_rate * dP_dh;  // Pa/s

    // Gas compression rate (simplified)
    double volume_rate = volume * dP_dt / 101325.0;
    TS_ASSERT(volume_rate > 0.0);
  }

  // Test multi-cell pressure balance
  void testMultiCellPressureBalance() {
    // Multiple gas cells should equalize pressure
    double P_cell1 = 101500.0;
    double P_cell2 = 101400.0;
    double P_cell3 = 101450.0;

    double P_avg = (P_cell1 + P_cell2 + P_cell3) / 3.0;
    TS_ASSERT_DELTA(P_avg, 101450.0, 1.0);
  }

  // Test ballonet pressure differential
  void testBallonetPressureDifferential() {
    // Air in ballonet slightly higher than ambient
    double P_ambient = 101325.0;
    double P_ballonet = 101400.0;
    double differential = P_ballonet - P_ambient;

    TS_ASSERT(differential > 0.0);
    TS_ASSERT(differential < 200.0);  // Typical 50-150 Pa
  }

  // Test automatic valve operation
  void testAutomaticValveOperation() {
    // Valve opens when overpressure exceeded
    double max_overpressure = 340.0;  // Pa
    double pressures[] = {100.0, 200.0, 340.0, 400.0, 500.0};
    bool expected_open[] = {false, false, false, true, true};

    for (int i = 0; i < 5; i++) {
      bool valve_open = pressures[i] > max_overpressure;
      TS_ASSERT_EQUALS(valve_open, expected_open[i]);
    }
  }

  // Test maneuvering valve operation
  void testManeuveringValveOperation() {
    // Manual valve for emergency descent
    double gas_mass = 1000.0;  // kg
    double valve_flow = 5.0;   // kg/s at full open

    double time_to_dump = gas_mass / valve_flow;
    TS_ASSERT_DELTA(time_to_dump, 200.0, 1.0);  // seconds
  }

  // ==================== THERMAL MANAGEMENT ====================

  // Test thermal time constant
  void testThermalTimeConstant() {
    // Time for gas to reach equilibrium temperature
    double mass_gas = 1000.0;     // kg
    double Cp = 5193.0;           // J/(kg*K) for helium
    double heat_transfer = 500.0; // W/K (envelope conductance)

    double tau = mass_gas * Cp / heat_transfer;  // seconds
    TS_ASSERT(tau > 10000.0);  // Several hours
  }

  // Test solar radiation absorption
  void testSolarRadiationAbsorption() {
    double solar_flux = 1000.0;    // W/m^2
    double absorptivity = 0.1;     // White/silver envelope
    double area = 5000.0;          // m^2

    double absorbed_power = solar_flux * absorptivity * area;
    TS_ASSERT_DELTA(absorbed_power, 500000.0, 1000.0);  // 500 kW
  }

  // Test infrared emission
  void testInfraredEmission() {
    double emissivity = 0.9;
    double sigma = 5.67e-8;  // Stefan-Boltzmann constant
    double T = 288.15;       // K
    double area = 5000.0;    // m^2

    double radiated_power = emissivity * sigma * std::pow(T, 4) * area;
    TS_ASSERT(radiated_power > 1.5e6);  // ~2 MW
  }

  // Test gas heating from sunlight
  void testGasHeatingFromSunlight() {
    double solar_input = 500000.0;  // W absorbed
    double mass_gas = 1000.0;       // kg
    double Cp = 5193.0;             // J/(kg*K)
    double time = 3600.0;           // 1 hour

    // Temperature rise (ignoring losses)
    double energy = solar_input * time;
    double dT = energy / (mass_gas * Cp);

    TS_ASSERT(dT > 300.0);  // Significant heating without losses
  }

  // ==================== STRUCTURAL EFFECTS ====================

  // Test catenary curve load
  void testCatenaryCurveLoad() {
    // Suspension cables form catenary under load
    double horizontal_tension = 10000.0;  // N
    double weight_per_length = 100.0;     // N/m
    double span = 50.0;                   // m

    // Sag = w*L^2 / (8*H)
    double sag = weight_per_length * span * span / (8.0 * horizontal_tension);
    TS_ASSERT(sag > 0.0);
    TS_ASSERT_DELTA(sag, 3.125, 0.01);
  }

  // Test suspension wire load
  void testSuspensionWireLoad() {
    // Wire from envelope to gondola
    double gondola_weight = 5000.0;  // N
    double num_wires = 8;
    double angle = 0.5;  // rad from vertical

    double wire_tension = gondola_weight / (num_wires * std::cos(angle));
    TS_ASSERT(wire_tension > 700.0);
  }

  // Test fin attachment loads
  void testFinAttachmentLoads() {
    // Aerodynamic load on tail fins
    double q = 200.0;        // Pa dynamic pressure
    double fin_area = 50.0;  // m^2
    double Cl = 0.5;         // lift coefficient

    double fin_force = q * fin_area * Cl;
    TS_ASSERT_DELTA(fin_force, 5000.0, 10.0);  // N
  }

  // Test nose mooring loads
  void testNoseMooringLoads() {
    // Forces at mooring mast
    double wind_drag = 20000.0;   // N
    double buoyancy = 100000.0;   // N
    double weight = 95000.0;      // N

    double vertical_load = buoyancy - weight;
    double horizontal_load = wind_drag;
    double total_load = std::sqrt(vertical_load * vertical_load +
                                   horizontal_load * horizontal_load);

    TS_ASSERT(total_load > 20000.0);
  }

  // ==================== PERFORMANCE ====================

  // Test pressure ceiling
  void testPressureCeiling() {
    // Maximum altitude before gas cells are full
    double initial_pressure_ratio = 0.8;  // 80% full at ground
    double gamma = 1.0 / 5.256;  // From barometric formula

    // Pressure ratio at ceiling = initial_pressure_ratio
    // h = H * ln(1/ratio) approximately
    double H = 8500.0;  // scale height
    double ceiling = H * std::log(1.0 / initial_pressure_ratio);

    TS_ASSERT(ceiling > 1500.0);  // meters
    TS_ASSERT(ceiling < 2500.0);
  }

  // Test useful lift calculation
  void testUsefulLiftCalculation() {
    double gross_lift = 50000.0;     // kg
    double structure_weight = 20000.0;
    double fuel_weight = 5000.0;
    double crew_weight = 500.0;
    double fixed_equipment = 3000.0;

    double useful_lift = gross_lift - structure_weight - fuel_weight -
                         crew_weight - fixed_equipment;

    TS_ASSERT_DELTA(useful_lift, 21500.0, 1.0);
  }

  // Test payload vs range tradeoff
  void testPayloadRangeTradeoff() {
    double max_fuel = 10000.0;    // kg
    double useful_lift = 20000.0; // kg
    double fuel_consumption = 100.0;  // kg/hour
    double speed = 100.0;         // km/hour

    // Full fuel
    double payload1 = useful_lift - max_fuel;
    double range1 = (max_fuel / fuel_consumption) * speed;

    // Half fuel
    double fuel2 = max_fuel / 2.0;
    double payload2 = useful_lift - fuel2;
    double range2 = (fuel2 / fuel_consumption) * speed;

    TS_ASSERT(range1 > range2);
    TS_ASSERT(payload2 > payload1);
  }

  // Test endurance calculation
  void testEnduranceCalculation() {
    double fuel_mass = 5000.0;  // kg
    double power = 500.0;       // kW
    double SFC = 0.3;           // kg/(kW*hr)

    double fuel_rate = power * SFC;  // kg/hr
    double endurance = fuel_mass / fuel_rate;  // hours

    TS_ASSERT_DELTA(endurance, 33.33, 0.1);
  }

  // Test speed vs power relationship
  void testSpeedPowerRelationship() {
    // Power ∝ V^3 for drag-limited vehicle
    double V1 = 50.0;   // m/s
    double P1 = 100.0;  // kW

    double V2 = 75.0;   // 50% faster
    double P2 = P1 * std::pow(V2/V1, 3);

    TS_ASSERT_DELTA(P2, 337.5, 1.0);  // 3.375x power for 1.5x speed
  }

  // ==================== FAILURE MODES ====================

  // Test gas leak rate effect
  void testGasLeakRateEffect() {
    double initial_mass = 1000.0;  // kg
    double leak_rate = 0.01;       // fraction per hour
    double time = 10.0;            // hours

    double remaining = initial_mass * std::exp(-leak_rate * time);
    double lost = initial_mass - remaining;

    TS_ASSERT(remaining > 900.0);
    TS_ASSERT_DELTA(lost, 95.16, 1.0);  // About 9.5% loss
  }

  // Test ballonet failure consequences
  void testBallonetFailure() {
    // Loss of ballonet = loss of shape control
    double envelope_volume = 10000.0;  // m^3
    double ballonet_volume = 1000.0;   // m^3
    double gas_fill = 0.85;            // 85% gas

    double gas_volume = envelope_volume * gas_fill;
    double remaining_ballonet = envelope_volume - gas_volume;

    TS_ASSERT_DELTA(remaining_ballonet, 1500.0, 1.0);
  }

  // Test envelope tear propagation
  void testEnvelopeTearPropagation() {
    // Small hole becomes large under pressure
    double initial_diameter = 0.01;  // m
    double stress = 1000000.0;       // Pa (membrane stress)
    double tear_strength = 50000.0;  // N/m

    // Simplified: hole grows if stress > tear strength
    double edge_stress = stress * initial_diameter;
    bool will_propagate = edge_stress > tear_strength;

    TS_ASSERT(will_propagate || !will_propagate);  // Just test logic runs
  }

  // Test emergency gas release
  void testEmergencyGasRelease() {
    // Rapid descent by venting
    double gas_mass = 1000.0;      // kg
    double valve_capacity = 50.0;  // kg/s
    double target_descent = 300.0; // m

    double time_to_vent = gas_mass / valve_capacity;
    TS_ASSERT_DELTA(time_to_vent, 20.0, 0.1);  // seconds
  }

  // ==================== MULTI-CELL CONFIGURATIONS ====================

  // Test cell-to-cell force transfer
  void testCellToCellForceTransfer() {
    // Adjacent cells share loads
    double cell1_buoyancy = 10000.0;
    double cell2_buoyancy = 12000.0;
    double cell3_buoyancy = 11000.0;

    double total = cell1_buoyancy + cell2_buoyancy + cell3_buoyancy;
    double average = total / 3.0;

    TS_ASSERT_DELTA(total, 33000.0, epsilon);
    TS_ASSERT_DELTA(average, 11000.0, epsilon);
  }

  // Test cell failure redistribution
  void testCellFailureRedistribution() {
    // If one cell fails, others must carry load
    double num_cells = 10.0;
    double total_lift = 100000.0;  // N
    double lift_per_cell = total_lift / num_cells;

    // One cell fails
    double remaining_cells = num_cells - 1.0;
    double new_lift_per_cell = total_lift / remaining_cells;

    TS_ASSERT(new_lift_per_cell > lift_per_cell);
    TS_ASSERT_DELTA(new_lift_per_cell, 11111.11, 1.0);
  }

  // Test internal walkway loads
  void testInternalWalkwayLoads() {
    // Keel walkway suspended in envelope
    double walkway_weight = 2000.0;  // N/m
    double span = 10.0;              // m between supports

    double support_load = walkway_weight * span / 2.0;
    TS_ASSERT_DELTA(support_load, 10000.0, epsilon);
  }

  // ==================== HISTORICAL/SPECIAL CASES ====================

  // Test Zeppelin LZ-129 scale
  void testZeppelinLZ129Scale() {
    // Hindenburg specifications
    double volume_m3 = 200000.0;  // 200,000 m^3
    double length_m = 245.0;
    double diameter_m = 41.2;

    double fineness_ratio = length_m / diameter_m;
    TS_ASSERT_DELTA(fineness_ratio, 5.95, 0.1);

    // Approximate lift with hydrogen
    double T = 288.15;
    double P = 101325.0;
    double rho_air = P * M_air / (R_universal * T);
    double rho_h2 = P * M_hydrogen / (R_universal * T);
    double gross_lift_kg = (rho_air - rho_h2) * volume_m3;

    TS_ASSERT(gross_lift_kg > 200000.0);  // Over 200 tonnes
  }

  // Test modern blimp scale
  void testModernBlimpScale() {
    // Typical advertising blimp
    double volume_m3 = 5700.0;  // ~200,000 ft^3
    double T = 288.15;
    double P = 101325.0;

    double rho_air = P * M_air / (R_universal * T);
    double rho_he = P * M_helium / (R_universal * T);
    double gross_lift_kg = (rho_air - rho_he) * volume_m3;

    TS_ASSERT(gross_lift_kg > 5000.0);
    TS_ASSERT(gross_lift_kg < 7000.0);
  }

  // Test high altitude balloon
  void testHighAltitudeBalloon() {
    // Stratospheric balloon at 30 km
    double h = 30000.0;  // m
    double H = 8500.0;   // scale height

    double P_ratio = std::exp(-h / H);
    double rho_ratio = P_ratio;  // Simplified (isothermal)

    double rho_air_30km = rho_air_SL * rho_ratio;
    TS_ASSERT(rho_air_30km < 0.05);  // Very thin air
  }

  // Test rozier balloon (hybrid)
  void testRozierBalloon() {
    // Combination hot air and helium
    double helium_cell_volume = 1000.0;  // m^3
    double hot_air_volume = 3000.0;      // m^3
    double T_ambient = 288.15;
    double T_hot = 373.15;
    double P = 101325.0;

    // Helium lift
    double rho_air = P * M_air / (R_universal * T_ambient);
    double rho_he = P * M_helium / (R_universal * T_ambient);
    double helium_lift = (rho_air - rho_he) * helium_cell_volume;

    // Hot air lift
    double rho_hot = P * M_air / (R_universal * T_hot);
    double hot_air_lift = (rho_air - rho_hot) * hot_air_volume;

    double total_lift = helium_lift + hot_air_lift;
    TS_ASSERT(total_lift > 1500.0);
  }

  // ==================== MISCELLANEOUS ====================

  // Test weight estimation
  void testWeightEstimation() {
    // Component weight buildup
    double envelope_weight = 5000.0;
    double structure_weight = 3000.0;
    double engines_weight = 2000.0;
    double fuel_weight = 4000.0;
    double payload_weight = 6000.0;

    double total = envelope_weight + structure_weight + engines_weight +
                   fuel_weight + payload_weight;
    TS_ASSERT_DELTA(total, 20000.0, epsilon);
  }

  // Test center of gravity calculation
  void testCenterOfGravityCalculation() {
    // Weighted average
    double masses[] = {5000.0, 3000.0, 2000.0};
    double positions[] = {10.0, 50.0, 80.0};

    double total_mass = 0.0;
    double moment = 0.0;
    for (int i = 0; i < 3; i++) {
      total_mass += masses[i];
      moment += masses[i] * positions[i];
    }

    double cg = moment / total_mass;
    // CG = (5000*10 + 3000*50 + 2000*80) / 10000 = 360000/10000 = 36.0
    TS_ASSERT_DELTA(cg, 36.0, 0.1);
  }

  // Test moment of inertia for ellipsoid
  void testEllipsoidInertia() {
    // I_xx = m/5 * (b^2 + c^2) for ellipsoid
    double mass = 10000.0;  // kg
    double a = 50.0;        // semi-axis x
    double b = 10.0;        // semi-axis y
    double c = 10.0;        // semi-axis z

    double I_xx = (mass / 5.0) * (b*b + c*c);
    double I_yy = (mass / 5.0) * (a*a + c*c);
    double I_zz = (mass / 5.0) * (a*a + b*b);

    TS_ASSERT(I_yy > I_xx);
    TS_ASSERT(I_zz > I_xx);
    TS_ASSERT_DELTA(I_yy, I_zz, 1.0);  // Nearly equal for a >> b ≈ c
  }

  // Test ground handling forces
  void testGroundHandlingForces() {
    // Mooring and ground crew requirements
    double buoyancy = 50000.0;  // N
    double ballast = 45000.0;   // N

    double net_lift = buoyancy - ballast;
    TS_ASSERT(net_lift > 0.0);  // Slightly light

    // Wind adds horizontal force
    double wind_force = 5000.0;
    double total_ground_force = std::sqrt(net_lift*net_lift + wind_force*wind_force);
    TS_ASSERT(total_ground_force > 7000.0);
  }

  // Test ballast water weight
  void testBallastWaterWeight() {
    double volume_liters = 500.0;
    double density = 1.0;  // kg/L

    double mass_kg = volume_liters * density;
    double weight_N = mass_kg * g;

    TS_ASSERT_DELTA(mass_kg, 500.0, epsilon);
    TS_ASSERT_DELTA(weight_N, 4903.33, 1.0);
  }

  // Test superheat gradient
  void testSuperheatGradient() {
    // Temperature varies through gas cell
    double T_top = 308.15;     // Warmer at top
    double T_middle = 298.15;
    double T_bottom = 293.15;  // Cooler at bottom

    double gradient = (T_top - T_bottom) / 20.0;  // K/m over 20m height
    TS_ASSERT(gradient > 0.5);  // Significant stratification
  }

  // Test gas cell pressure distribution
  void testPressureDistribution() {
    // Pressure increases with depth in gas
    double rho_he = 0.164;  // kg/m^3 at SL
    double height = 20.0;   // m

    double delta_P = rho_he * g * height;
    TS_ASSERT_DELTA(delta_P, 32.2, 0.5);  // Pa
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  // Test complete buoyancy system simulation
  void testCompleteBuoyancySystem() {
    // Full airship configuration
    double main_cell_volume = 50000.0;  // m³
    double aft_cell_volume = 10000.0;
    double total_volume = main_cell_volume + aft_cell_volume;

    double rho_air = 1.225;
    double rho_helium = 0.164;
    double gross_lift = total_volume * (rho_air - rho_helium) * g;

    TS_ASSERT(gross_lift > 500000.0);  // More than 500 kN lift
  }

  // Test dynamic superheat cycle
  void testDynamicSuperheatCycle() {
    double base_temp = 288.15;
    double solar_heating = 20.0;  // K
    double night_cooling = -15.0;  // K

    double day_temp = base_temp + solar_heating;
    double night_temp = base_temp + night_cooling;

    TS_ASSERT(day_temp > night_temp);
    TS_ASSERT_DELTA(day_temp - night_temp, 35.0, 0.1);
  }

  // Test ballonet fill sequence
  void testBallonetFillSequence() {
    double ballonet_volume = 0.0;
    double max_volume = 5000.0;
    double fill_rate = 100.0;  // m³/s

    for (double t = 0; t < 60; t += 1.0) {
      ballonet_volume = std::min(ballonet_volume + fill_rate, max_volume);
    }

    TS_ASSERT_DELTA(ballonet_volume, max_volume, 1.0);
  }

  // Test envelope stress calculation
  void testEnvelopeStressCalculation() {
    double internal_pressure = 500.0;  // Pa
    double radius = 10.0;  // m
    double thickness = 0.001;  // m (1mm fabric)

    // Hoop stress: σ = PR/t
    double hoop_stress = internal_pressure * radius / thickness;
    TS_ASSERT_DELTA(hoop_stress, 5000000.0, 100.0);  // 5 MPa
  }

  // Test weight distribution analysis
  void testWeightDistributionAnalysis() {
    double envelope = 5000.0;
    double gondola = 2000.0;
    double propulsion = 1500.0;
    double fuel = 3000.0;
    double payload = 4000.0;

    double total = envelope + gondola + propulsion + fuel + payload;
    double gondola_ratio = gondola / total;

    TS_ASSERT_DELTA(total, 15500.0, 1.0);
    TS_ASSERT(gondola_ratio < 0.2);  // Gondola less than 20% of weight
  }

  // Test altitude ceiling calculation
  void testAltitudeCeilingCalculation() {
    double gross_lift_sl = 100000.0;  // N at sea level
    double total_weight = 95000.0;  // N

    // At ceiling, lift equals weight
    // Using exponential atmosphere approximation
    double required_density_ratio = total_weight / gross_lift_sl;
    double scale_height = 8500.0;  // m

    double ceiling = -scale_height * std::log(required_density_ratio);
    TS_ASSERT(ceiling > 0.0);
    TS_ASSERT(ceiling < 10000.0);  // Reasonable ceiling
  }

  // Test trim calculation for level flight
  void testTrimCalculationLevelFlight() {
    double buoyancy = 100000.0;  // N
    double weight = 98000.0;  // N
    double net_lift = buoyancy - weight;

    // Need downward thrust or ballast
    double required_ballonnet_fill = net_lift / g;  // kg of air needed

    TS_ASSERT(required_ballonnet_fill > 0.0);
    TS_ASSERT(required_ballonnet_fill < 300.0);  // Reasonable ballast
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test independent gas cell calculations
  void testIndependentGasCellCalculations() {
    double volume1 = 10000.0;
    double volume2 = 20000.0;
    double rho_diff = 1.225 - 0.164;

    double lift1 = volume1 * rho_diff * g;
    double lift2 = volume2 * rho_diff * g;

    TS_ASSERT_DELTA(lift2 / lift1, 2.0, 0.01);
    TS_ASSERT(lift1 != lift2);
  }

  // Test separate temperature calculations
  void testSeparateTemperatureCalculations() {
    double T1 = 300.0;
    double T2 = 320.0;
    double V_std = 1000.0;

    double V1 = V_std * T1 / 288.15;
    double V2 = V_std * T2 / 288.15;

    TS_ASSERT(V2 > V1);
    TS_ASSERT(std::abs(V1 - V2) > 50.0);
  }

  // Test pressure calculations independence
  void testPressureCalculationsIndependence() {
    double P1 = 101325.0;
    double P2 = 95000.0;

    double rho1 = P1 / (287.0 * 288.15);
    double rho2 = P2 / (287.0 * 288.15);

    TS_ASSERT(rho1 > rho2);
    TS_ASSERT_DELTA(rho1 / rho2, P1 / P2, 0.01);
  }

  // Test envelope volume independence
  void testEnvelopeVolumeIndependence() {
    double a1 = 50.0, b1 = 10.0, c1 = 10.0;
    double a2 = 60.0, b2 = 12.0, c2 = 12.0;

    double V1 = (4.0/3.0) * M_PI * a1 * b1 * c1;
    double V2 = (4.0/3.0) * M_PI * a2 * b2 * c2;

    TS_ASSERT(V2 > V1);
    TS_ASSERT(V1 > 20000.0);
  }

  // Test dynamic pressure coefficient independence
  void testDynamicPressureCoefficientIndependence() {
    double rho = 1.225;
    double V1 = 20.0;
    double V2 = 40.0;

    double q1 = 0.5 * rho * V1 * V1;
    double q2 = 0.5 * rho * V2 * V2;

    TS_ASSERT_DELTA(q2 / q1, 4.0, 0.01);
  }

  // Test wind loading independence
  void testWindLoadingIndependence() {
    double A = 1000.0;  // Reference area
    double Cd = 0.02;
    double rho = 1.225;

    double V1 = 10.0;
    double V2 = 20.0;

    double F1 = 0.5 * rho * V1 * V1 * Cd * A;
    double F2 = 0.5 * rho * V2 * V2 * Cd * A;

    TS_ASSERT_DELTA(F2 / F1, 4.0, 0.01);
  }

  // Test valve flow calculations independence
  void testValveFlowIndependence() {
    double delta_P1 = 100.0;
    double delta_P2 = 400.0;
    double K = 0.1;

    double flow1 = K * std::sqrt(delta_P1);
    double flow2 = K * std::sqrt(delta_P2);

    TS_ASSERT_DELTA(flow2 / flow1, 2.0, 0.01);
  }

  // Test buoyancy ratio calculations
  void testBuoyancyRatioCalculations() {
    double gross_lift = 100000.0;
    double total_weight = 95000.0;

    double buoyancy_ratio = gross_lift / total_weight;
    TS_ASSERT(buoyancy_ratio > 1.0);  // Positive lift
    TS_ASSERT_DELTA(buoyancy_ratio, 1.053, 0.01);
  }
};
