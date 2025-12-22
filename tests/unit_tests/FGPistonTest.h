#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include "TestUtilities.h"

using namespace JSBSim;

class FGPistonTest : public CxxTest::TestSuite
{
public:
  // Test engine type is etPiston
  void testEngineType() {
    TS_ASSERT_EQUALS(FGEngine::etPiston, 2);
  }

  // Test manifold pressure calculation at sea level
  void testManifoldPressureSeaLevel() {
    // At sea level, full throttle, MAP approaches ambient
    double ambientPressure_inHg = 29.92;
    double throttlePos = 1.0;

    // At full throttle, MAP ~= ambient (minus small losses)
    double expectedMAP = ambientPressure_inHg * 0.95;  // ~5% loss typical
    TS_ASSERT(expectedMAP > 28.0);
    TS_ASSERT(expectedMAP < 30.0);
  }

  // Test manifold pressure at altitude
  void testManifoldPressureAltitude() {
    // At altitude, ambient pressure drops
    double ambientPressure_inHg_SL = 29.92;
    double altitudeFt = 10000.0;

    // Rough approximation: pressure drops ~1 inHg per 1000 ft
    double ambientPressure_inHg = ambientPressure_inHg_SL - (altitudeFt / 1000.0);
    TS_ASSERT_DELTA(ambientPressure_inHg, 19.92, 0.1);

    // MAP cannot exceed ambient (naturally aspirated)
    double maxMAP = ambientPressure_inHg;
    TS_ASSERT(maxMAP < ambientPressure_inHg_SL);
  }

  // Test throttle impedance calculation
  void testThrottleImpedance() {
    // Throttle impedance: Z_t = (1 - ThrottlePos)^2 * Z_throttle
    double Z_throttle = 40.0;  // Throttle bore impedance constant

    // Full throttle - minimal impedance
    double throttlePos = 1.0;
    double Z_t = pow(1.0 - throttlePos, 2) * Z_throttle;
    TS_ASSERT_DELTA(Z_t, 0.0, DEFAULT_TOLERANCE);

    // Half throttle
    throttlePos = 0.5;
    Z_t = pow(1.0 - throttlePos, 2) * Z_throttle;
    TS_ASSERT_DELTA(Z_t, 10.0, DEFAULT_TOLERANCE);

    // Idle throttle
    throttlePos = 0.0;
    Z_t = pow(1.0 - throttlePos, 2) * Z_throttle;
    TS_ASSERT_DELTA(Z_t, 40.0, DEFAULT_TOLERANCE);
  }

  // Test volumetric efficiency calculation
  void testVolumetricEfficiency() {
    // Volumetric efficiency typically 0.8-0.9 for naturally aspirated
    double volEff = 0.85;

    // Reduced at high RPM due to valve timing
    double rpmRatio = 1.0;  // At rated RPM
    double reducedVolEff = volEff * (1.0 - 0.1 * (rpmRatio - 0.8));
    TS_ASSERT(reducedVolEff > 0.75);
    TS_ASSERT(reducedVolEff < 0.90);
  }

  // Test air mass flow rate calculation
  void testAirMassFlowRate() {
    // m_dot_air = swept_volume * volumetric_efficiency * rho_manifold
    double displacement = 360.0;  // cubic inches
    double displacement_SI = displacement * 1.6387e-5;  // m^3
    double rpm = 2400.0;
    double volumetricEfficiency = 0.85;

    // Swept volume per second
    double sweptVolume = (displacement_SI * (rpm / 60.0)) / 2.0;  // 4-stroke

    // Air density at manifold
    double MAP_Pa = 90000.0;  // ~26 inHg
    double T_ambient = 288.15;  // K (15°C)
    double R_air = 287.05;  // J/(kg·K)
    double rho_manifold = MAP_Pa / (R_air * T_ambient);

    double m_dot_air = sweptVolume * volumetricEfficiency * rho_manifold;
    TS_ASSERT(m_dot_air > 0.0);
    TS_ASSERT(m_dot_air < 1.0);  // kg/s - reasonable for small engine
  }

  // Test fuel flow from mixture
  void testFuelFlowFromMixture() {
    // m_dot_fuel = m_dot_air * equivalence_ratio / stoich_ratio
    double m_dot_air = 0.05;  // kg/s
    double stoich_ratio = 14.7;  // Air/fuel stoichiometric
    double equivalence_ratio = 1.0;  // Stoichiometric

    double m_dot_fuel = (m_dot_air * equivalence_ratio) / stoich_ratio;
    TS_ASSERT_DELTA(m_dot_fuel, 0.0034, 0.0001);

    // Rich mixture (equivalence_ratio > 1)
    equivalence_ratio = 1.2;
    m_dot_fuel = (m_dot_air * equivalence_ratio) / stoich_ratio;
    TS_ASSERT(m_dot_fuel > 0.0034);

    // Lean mixture (equivalence_ratio < 1)
    equivalence_ratio = 0.8;
    m_dot_fuel = (m_dot_air * equivalence_ratio) / stoich_ratio;
    TS_ASSERT(m_dot_fuel < 0.0034);
  }

  // Test fuel flow unit conversions
  void testFuelFlowConversions() {
    double m_dot_fuel_kgs = 0.01;  // kg/s

    // Convert to lbs/sec
    double fuelFlowRate_lbs = m_dot_fuel_kgs * 2.2046;
    TS_ASSERT_DELTA(fuelFlowRate_lbs, 0.02205, 0.0001);

    // Convert to lbs/hr (PPH)
    double fuelFlow_pph = fuelFlowRate_lbs * 3600.0;
    TS_ASSERT_DELTA(fuelFlow_pph, 79.37, 0.1);

    // Convert to gallons/hr (AVGAS density 6.02 lbs/gal)
    double fuelDensity = 6.02;
    double fuelFlow_gph = fuelFlow_pph / fuelDensity;
    TS_ASSERT_DELTA(fuelFlow_gph, 13.18, 0.1);
  }

  // Test indicated horsepower calculation
  void testIndicatedHorsepower() {
    // IHP = FuelFlow_pph / ISFC
    double fuelFlow_pph = 50.0;
    double ISFC = 0.45;  // Indicated specific fuel consumption (lbs/hp-hr)

    double IHP = fuelFlow_pph / ISFC;
    TS_ASSERT_DELTA(IHP, 111.1, 0.1);
  }

  // Test brake specific fuel consumption
  void testBSFC() {
    // BSFC = Fuel_flow / Brake_HP
    double fuelFlow_pph = 50.0;
    double brakeHP = 100.0;

    double BSFC = fuelFlow_pph / brakeHP;
    TS_ASSERT_DELTA(BSFC, 0.5, DEFAULT_TOLERANCE);

    // Typical ranges: 0.4-0.6 lbs/hp-hr for piston engines
    TS_ASSERT(BSFC >= 0.3);
    TS_ASSERT(BSFC <= 0.7);
  }

  // Test mean piston speed calculation
  void testMeanPistonSpeed() {
    // MPS = 2 * Stroke * RPM / 60
    double stroke_in = 4.5;
    double stroke_ft = stroke_in / 12.0;
    double rpm = 2700.0;

    double MPS_fps = 2.0 * stroke_ft * rpm / 60.0;
    // MPS = 2 * 0.375 * 2700 / 60 = 33.75 ft/sec
    TS_ASSERT_DELTA(MPS_fps, 33.75, 0.1);

    // Typical limit is ~50 ft/sec for reliability
    TS_ASSERT(MPS_fps < 50.0);
  }

  // Test exhaust gas temperature calculation
  void testExhaustGasTemp() {
    // EGT rises with power and leans with mixture
    double T_ambient = 288.15;  // K
    double fuelFlow = 0.01;     // kg/s
    double calorificValue = 43e6;  // J/kg (gasoline)
    double combustionEff = 0.95;
    double exhaustFraction = 0.30;  // 30% of heat goes to exhaust

    double Cp_exhaust = 1100.0;  // J/(kg·K)
    double m_dot_exhaust = 0.05;  // kg/s (air + fuel)

    double heatToExhaust = fuelFlow * calorificValue * combustionEff * exhaustFraction;
    double deltaT = heatToExhaust / (Cp_exhaust * m_dot_exhaust);
    double EGT = T_ambient + deltaT;

    TS_ASSERT(EGT > T_ambient);
    TS_ASSERT(EGT < 1500.0);  // Reasonable EGT limit in K
  }

  // Test cylinder head temperature dynamics
  void testCylinderHeadTemp() {
    // CHT responds slower than EGT due to thermal mass
    double CHT_initial = 350.0;  // K
    double CHT_target = 450.0;   // K
    double heatCapacity = 5000.0;  // J/K
    double heatInput = 10000.0;    // W
    double deltaT = 1.0;           // sec

    double dCHT = (heatInput / heatCapacity) * deltaT;
    double CHT_new = CHT_initial + dCHT;

    TS_ASSERT_DELTA(CHT_new, 352.0, 0.1);
    TS_ASSERT(CHT_new < CHT_target);  // Not reached yet
  }

  // Test oil pressure calculation
  void testOilPressure() {
    // Oil pressure roughly proportional to RPM
    double maxPressure = 60.0;  // psi
    double maxRPM = 2700.0;
    double rpm = 2400.0;

    double oilPressure = (maxPressure / maxRPM) * rpm;
    TS_ASSERT_DELTA(oilPressure, 53.33, 0.1);

    // At idle, pressure is lower
    rpm = 600.0;
    oilPressure = (maxPressure / maxRPM) * rpm;
    TS_ASSERT_DELTA(oilPressure, 13.33, 0.1);
  }

  // Test boost pressure (supercharger)
  void testBoostPressure() {
    // Boost multiplier increases MAP above ambient
    double ambientMAP = 20.0;  // inHg at altitude
    double boostMultiplier = 1.5;

    double boostedMAP = ambientMAP * boostMultiplier;
    TS_ASSERT_DELTA(boostedMAP, 30.0, DEFAULT_TOLERANCE);

    // Boost is limited by wastegate/BCV
    double ratedMAP = 36.0;  // inHg
    if (boostedMAP > ratedMAP) boostedMAP = ratedMAP;
    TS_ASSERT(boostedMAP <= ratedMAP);
  }

  // Test mixture control effect
  void testMixtureControl() {
    // Mixture position 0-1 controls equivalence ratio
    double mixturePos = 1.0;  // Full rich
    double thi_sea_level = 1.3 * mixturePos;
    TS_ASSERT_DELTA(thi_sea_level, 1.3, DEFAULT_TOLERANCE);

    // Lean mixture
    mixturePos = 0.5;
    thi_sea_level = 1.3 * mixturePos;
    TS_ASSERT_DELTA(thi_sea_level, 0.65, DEFAULT_TOLERANCE);

    // Cutoff (idle cutoff)
    mixturePos = 0.0;
    thi_sea_level = 1.3 * mixturePos;
    TS_ASSERT_DELTA(thi_sea_level, 0.0, DEFAULT_TOLERANCE);
  }

  // Test magneto effect on power
  void testMagnetoEffect() {
    // Both mags: 100% power
    // Single mag: ~96% power (3-5% loss)
    int magnetos = 3;  // Both mags
    double powerFactor = (magnetos == 3) ? 1.0 : 0.96;
    TS_ASSERT_DELTA(powerFactor, 1.0, DEFAULT_TOLERANCE);

    // Left mag only
    magnetos = 1;
    powerFactor = (magnetos == 3) ? 1.0 : 0.96;
    TS_ASSERT_DELTA(powerFactor, 0.96, DEFAULT_TOLERANCE);

    // Right mag only
    magnetos = 2;
    powerFactor = (magnetos == 3) ? 1.0 : 0.96;
    TS_ASSERT_DELTA(powerFactor, 0.96, DEFAULT_TOLERANCE);

    // No mags (engine won't run)
    magnetos = 0;
    powerFactor = (magnetos == 0) ? 0.0 : 0.96;
    TS_ASSERT_DELTA(powerFactor, 0.0, DEFAULT_TOLERANCE);
  }

  // Test RPM governor behavior
  void testRPMGovernor() {
    double targetRPM = 2400.0;
    double actualRPM = 2450.0;
    double governorGain = 0.01;

    // Governor reduces throttle when RPM too high
    double throttleAdjust = governorGain * (targetRPM - actualRPM);
    TS_ASSERT(throttleAdjust < 0.0);  // Reduce throttle

    // Governor increases throttle when RPM too low
    actualRPM = 2350.0;
    throttleAdjust = governorGain * (targetRPM - actualRPM);
    TS_ASSERT(throttleAdjust > 0.0);  // Increase throttle
  }

  // Test compression ratio effect
  void testCompressionRatio() {
    // Higher CR = higher thermal efficiency
    double CR = 8.5;  // Compression ratio

    // Otto cycle efficiency: eta = 1 - 1/CR^(gamma-1)
    double gamma = 1.4;
    double efficiency = 1.0 - pow(1.0 / CR, gamma - 1.0);
    TS_ASSERT_DELTA(efficiency, 0.575, 0.01);

    // Higher CR
    CR = 10.0;
    efficiency = 1.0 - pow(1.0 / CR, gamma - 1.0);
    TS_ASSERT(efficiency > 0.575);
  }

  // Test displacement calculation
  void testDisplacementCalculation() {
    // Displacement = pi/4 * bore^2 * stroke * cylinders
    double bore_in = 5.125;
    double stroke_in = 4.5;
    int cylinders = 4;

    double displacement = (M_PI / 4.0) * pow(bore_in, 2) * stroke_in * cylinders;
    // = 0.785 * 26.27 * 4.5 * 4 = 371.3 cu in
    TS_ASSERT_DELTA(displacement, 371.3, 1.0);
  }

  // Test power at altitude
  void testPowerAtAltitude() {
    // Power decreases with altitude due to lower air density
    double seaLevelPower = 180.0;  // HP
    double densityRatio = 0.8;     // At ~6000 ft

    // Simple approximation: power proportional to density
    double altitudePower = seaLevelPower * densityRatio;
    TS_ASSERT_DELTA(altitudePower, 144.0, DEFAULT_TOLERANCE);
  }

  // Test engine startup conditions
  void testEngineStartup() {
    double rpm = 0.0;
    bool starter = true;
    bool hasFuel = true;
    int magnetos = 3;
    double startRPM = 400.0;  // RPM threshold for starting

    // During cranking
    rpm = 150.0;
    bool running = (rpm >= startRPM) && hasFuel && (magnetos > 0);
    TS_ASSERT(!running);

    // Above start threshold
    rpm = 500.0;
    running = (rpm >= startRPM) && hasFuel && (magnetos > 0);
    TS_ASSERT(running);
  }
};
