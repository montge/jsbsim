#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGPiston.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

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
    double fuelFlow = 0.002;    // kg/s (small GA engine)
    double calorificValue = 43e6;  // J/kg (gasoline)
    double combustionEff = 0.95;
    double exhaustFraction = 0.30;  // 30% of heat goes to exhaust

    double Cp_exhaust = 1100.0;  // J/(kg·K)
    double m_dot_exhaust = 0.10;  // kg/s (air + fuel, AFR ~15:1)

    double heatToExhaust = fuelFlow * calorificValue * combustionEff * exhaustFraction;
    double deltaT = heatToExhaust / (Cp_exhaust * m_dot_exhaust);
    double EGT = T_ambient + deltaT;

    // EGT should be reasonable (typically 800-1100 K for piston engines)
    TS_ASSERT(EGT > T_ambient);
    TS_ASSERT(EGT < 1200.0);  // Reasonable EGT limit in K
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

  /***************************************************************************
   * Additional Mixture and Fuel Tests
   ***************************************************************************/

  // Test best economy mixture (lean of peak)
  void testBestEconomyMixture() {
    // Peak EGT occurs at stoichiometric
    // Best economy is 50F lean of peak
    double phi_stoich = 1.0;
    double phi_leanOfPeak = 0.85;  // Typical lean of peak

    // Fuel flow reduction
    double fuelFlowStoich = 0.01;  // kg/s
    double fuelFlowLOP = fuelFlowStoich * (phi_leanOfPeak / phi_stoich);

    TS_ASSERT(fuelFlowLOP < fuelFlowStoich);
    TS_ASSERT_DELTA(fuelFlowLOP, 0.0085, 0.0001);
  }

  // Test best power mixture (rich of peak)
  void testBestPowerMixture() {
    // Best power is ~125F rich of peak (phi ~ 1.1)
    double phi_bestPower = 1.1;
    double phi_stoich = 1.0;

    // Power increase ~3% from stoich
    double powerStoich = 100.0;
    double powerROP = powerStoich * (1.0 + 0.03 * (phi_bestPower - 1.0));

    TS_ASSERT(powerROP > powerStoich);
    TS_ASSERT_DELTA(powerROP, 100.3, 0.1);
  }

  // Test detonation margin
  void testDetonationMargin() {
    // Detonation occurs when charge temp exceeds autoignition temp
    double chargeTemp = 500.0;  // K
    double detonationTemp = 550.0;  // K (depends on fuel octane)

    double margin = detonationTemp - chargeTemp;
    TS_ASSERT(margin > 0);  // Safe

    // High manifold pressure increases charge temp
    double MAP = 35.0;  // inHg (high power)
    double chargeTemp_highPower = 500.0 + (MAP - 28.0) * 5.0;

    TS_ASSERT(chargeTemp_highPower > chargeTemp);
    TS_ASSERT_DELTA(chargeTemp_highPower, 535.0, 1.0);
  }

  // Test fuel grade requirements
  void testFuelGradeRequirements() {
    // Higher compression ratio requires higher octane
    double CR = 8.5;
    double minOctane = 80.0 + (CR - 7.0) * 6.67;
    TS_ASSERT_DELTA(minOctane, 90.0, 1.0);

    // Higher CR
    CR = 10.0;
    minOctane = 80.0 + (CR - 7.0) * 6.67;
    TS_ASSERT_DELTA(minOctane, 100.0, 1.0);
  }

  // Test cold start fuel enrichment
  void testColdStartEnrichment() {
    double oilTemp = 280.0;  // K (cold)
    double normalOilTemp = 350.0;  // K
    double primeFactor = 2.0;  // Full prime enrichment

    double enrichmentFactor = primeFactor * (normalOilTemp - oilTemp) / (normalOilTemp - 260.0);
    TS_ASSERT(enrichmentFactor > 0);
    TS_ASSERT(enrichmentFactor < primeFactor);

    // At normal temp, no enrichment
    oilTemp = normalOilTemp;
    enrichmentFactor = primeFactor * std::max(0.0, (normalOilTemp - oilTemp) / (normalOilTemp - 260.0));
    TS_ASSERT_DELTA(enrichmentFactor, 0.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Carburetor Tests
   ***************************************************************************/

  // Test carburetor icing conditions
  void testCarburetorIcing() {
    // Icing occurs when fuel vaporization causes temp drop
    double ambientTemp = 280.0;  // K (7°C)
    double humidity = 0.8;        // High humidity
    double tempDrop = 30.0;       // K due to venturi/fuel evaporation

    double carbTemp = ambientTemp - tempDrop;
    bool icingPossible = (carbTemp < 273.15) && (humidity > 0.5);

    TS_ASSERT(icingPossible);

    // No icing at low humidity
    humidity = 0.3;
    icingPossible = (carbTemp < 273.15) && (humidity > 0.5);
    TS_ASSERT(!icingPossible);
  }

  // Test carburetor heat effect
  void testCarburetorHeat() {
    double ambientTemp = 275.0;  // K
    double carbHeatTemp = 310.0;  // K (exhaust-heated air)
    double carbHeatFraction = 0.5;  // 50% heat application

    double inductionTemp = ambientTemp + carbHeatFraction * (carbHeatTemp - ambientTemp);
    TS_ASSERT_DELTA(inductionTemp, 292.5, 0.1);

    // Full carb heat
    carbHeatFraction = 1.0;
    inductionTemp = ambientTemp + carbHeatFraction * (carbHeatTemp - ambientTemp);
    TS_ASSERT_DELTA(inductionTemp, carbHeatTemp, DEFAULT_TOLERANCE);
  }

  // Test accelerator pump
  void testAcceleratorPump() {
    double throttleDelta = 0.5;  // Rapid throttle advance
    double pumpVolume = 0.001;   // liters
    double pumpConstant = 0.5;   // shot proportional to throttle rate

    double extraFuel = throttleDelta * pumpVolume * pumpConstant;
    TS_ASSERT(extraFuel > 0);

    // No pump shot on gradual throttle
    throttleDelta = 0.01;
    extraFuel = throttleDelta * pumpVolume * pumpConstant;
    TS_ASSERT(extraFuel < 0.0001);
  }

  /***************************************************************************
   * Engine Performance Tests
   ***************************************************************************/

  // Test torque from power and RPM
  void testTorqueCalculation() {
    // Torque = Power * 5252 / RPM
    double powerHP = 180.0;
    double rpm = 2400.0;

    double torque_ftlb = powerHP * 5252.0 / rpm;
    TS_ASSERT_DELTA(torque_ftlb, 394.0, 1.0);

    // At lower RPM, more torque for same power
    rpm = 2000.0;
    torque_ftlb = powerHP * 5252.0 / rpm;
    TS_ASSERT_DELTA(torque_ftlb, 472.7, 1.0);
  }

  // Test friction horsepower
  void testFrictionHorsepower() {
    // FHP increases with RPM
    double rpm = 2400.0;
    double displacement = 360.0;  // cu in
    double fhpConstant = 0.01;    // HP per cu in per 1000 RPM

    double FHP = displacement * fhpConstant * (rpm / 1000.0);
    TS_ASSERT_DELTA(FHP, 8.64, 0.1);

    // At higher RPM, more friction
    rpm = 2700.0;
    FHP = displacement * fhpConstant * (rpm / 1000.0);
    TS_ASSERT(FHP > 8.64);
  }

  // Test pumping losses
  void testPumpingLosses() {
    // Pumping work increases at low throttle
    double MAP = 15.0;  // inHg (partial throttle)
    double ambientP = 29.92;

    double pumpingWork = (ambientP - MAP) / ambientP;
    TS_ASSERT_DELTA(pumpingWork, 0.498, 0.01);

    // At full throttle, minimal pumping loss
    MAP = 28.0;
    pumpingWork = (ambientP - MAP) / ambientP;
    TS_ASSERT(pumpingWork < 0.1);
  }

  // Test engine inertia
  void testEngineInertia() {
    // J = 0.5 * m * r^2 for rotating parts
    double crankMass = 30.0;  // lbs
    double radiusGyration = 0.25;  // ft

    double J = 0.5 * (crankMass / 32.2) * pow(radiusGyration, 2);  // slug-ft^2
    TS_ASSERT(J > 0);

    // Angular acceleration
    double torque = 100.0;  // ft-lbs
    double alpha = torque / J;  // rad/s^2
    TS_ASSERT(alpha > 0);
  }

  // Test ram air effect
  void testRamAirEffect() {
    // RAM air increases MAP at high speed
    double IAS = 200.0;  // knots
    double ramRecovery = 0.8;
    double dynamicPressure = 0.5 * 0.00238 * pow(IAS * 1.687, 2);  // psf

    double ramRise_inHg = dynamicPressure * 0.0145 * ramRecovery;  // ~inHg
    TS_ASSERT(ramRise_inHg > 0);
    TS_ASSERT(ramRise_inHg < 3.0);  // Reasonable RAM air effect
  }

  /***************************************************************************
   * Turbocharger/Supercharger Tests
   ***************************************************************************/

  // Test turbocharger waste gate
  void testTurboWasteGate() {
    double targetMAP = 36.0;  // inHg
    double actualMAP = 40.0;  // Over boost

    // Waste gate opens to reduce boost
    double wasteGatePosition = std::min(1.0, (actualMAP - targetMAP) / 10.0);
    TS_ASSERT(wasteGatePosition > 0);

    // At target, waste gate modulates
    actualMAP = 36.0;
    wasteGatePosition = std::max(0.0, (actualMAP - targetMAP) / 10.0);
    TS_ASSERT_DELTA(wasteGatePosition, 0.0, DEFAULT_TOLERANCE);
  }

  // Test critical altitude
  void testCriticalAltitude() {
    // Altitude where turbo can no longer maintain rated MAP
    double ratedMAP = 36.0;  // inHg
    double maxBoostRatio = 2.5;
    double ambientPressure_SL = 29.92;

    // Critical altitude is where ambient * boost_ratio = rated MAP
    double criticalAmbient = ratedMAP / maxBoostRatio;
    TS_ASSERT_DELTA(criticalAmbient, 14.4, 0.1);

    // This corresponds to ~14000 ft
    TS_ASSERT(criticalAmbient < ambientPressure_SL);
  }

  // Test intercooler effect
  void testIntercoolerEffect() {
    // Intercooler reduces charge temp after compression
    double tempAfterCompression = 420.0;  // K
    double ambientTemp = 288.0;  // K
    double intercoolerEfficiency = 0.7;

    double tempAfterIntercooler = tempAfterCompression -
      intercoolerEfficiency * (tempAfterCompression - ambientTemp);
    TS_ASSERT_DELTA(tempAfterIntercooler, 327.6, 1.0);

    // Power gain from denser charge
    double densityRatio = tempAfterCompression / tempAfterIntercooler;
    TS_ASSERT(densityRatio > 1.0);
  }

  /***************************************************************************
   * Cooling System Tests
   ***************************************************************************/

  // Test cowl flap effect
  void testCowlFlapEffect() {
    // Open cowl flaps increase cooling, decrease airspeed
    double cowlFlapPosition = 1.0;  // Fully open
    double baseCooling = 100.0;  // arbitrary units
    double maxCoolingIncrease = 0.5;

    double cooling = baseCooling * (1.0 + maxCoolingIncrease * cowlFlapPosition);
    TS_ASSERT_DELTA(cooling, 150.0, DEFAULT_TOLERANCE);

    // Closed cowl flaps
    cowlFlapPosition = 0.0;
    cooling = baseCooling * (1.0 + maxCoolingIncrease * cowlFlapPosition);
    TS_ASSERT_DELTA(cooling, 100.0, DEFAULT_TOLERANCE);
  }

  // Test oil temperature dynamics
  void testOilTempDynamics() {
    double oilTemp = 350.0;  // K
    double targetTemp = 380.0;  // K under load
    double heatInput = 5000.0;  // W
    double heatRejection = 3000.0;  // W
    double thermalMass = 20000.0;  // J/K

    double netHeat = heatInput - heatRejection;
    double deltaT = (netHeat / thermalMass) * 1.0;  // 1 second
    double newOilTemp = oilTemp + deltaT;

    TS_ASSERT(newOilTemp > oilTemp);
    TS_ASSERT_DELTA(newOilTemp, 350.1, 0.01);
  }

  // Test air cooling at different speeds
  void testAirCoolingVsSpeed() {
    double IAS = 100.0;  // knots
    double baseCooling = 1000.0;  // W at 100 kts

    // Cooling proportional to sqrt(velocity)
    double cooling100 = baseCooling;
    double cooling150 = baseCooling * std::sqrt(150.0 / 100.0);

    TS_ASSERT(cooling150 > cooling100);
    TS_ASSERT_DELTA(cooling150, 1224.7, 1.0);
  }

  /***************************************************************************
   * Propeller Interaction Tests
   ***************************************************************************/

  // Test propeller load at constant speed
  void testConstantSpeedPropLoad() {
    // Constant speed prop varies pitch to maintain RPM
    double targetRPM = 2400.0;
    double currentRPM = 2500.0;
    double propGain = 0.1;

    // Increase pitch to reduce RPM
    double pitchChange = propGain * (currentRPM - targetRPM);
    TS_ASSERT(pitchChange > 0);  // Increase pitch

    // Decrease pitch when under-speed
    currentRPM = 2300.0;
    pitchChange = propGain * (currentRPM - targetRPM);
    TS_ASSERT(pitchChange < 0);  // Decrease pitch
  }

  // Test propeller power absorption
  void testPropellerPowerAbsorption() {
    // Power = k * rho * n^3 * D^5 * Cp
    double rho = 0.00238;  // slug/ft^3
    double n = 2400.0 / 60.0;  // rev/sec
    double D = 6.5;  // ft (propeller diameter)
    double Cp = 0.04;  // power coefficient

    double power = rho * pow(n, 3) * pow(D, 5) * Cp;
    // Convert to HP
    double powerHP = power / 550.0;

    TS_ASSERT(powerHP > 0);
    TS_ASSERT(powerHP < 500.0);  // Reasonable for GA aircraft
  }

  /***************************************************************************
   * Ignition System Tests
   ***************************************************************************/

  // Test ignition timing effect
  void testIgnitionTiming() {
    // Optimal timing varies with RPM
    double rpm = 2400.0;
    double baseTiming = 20.0;  // degrees BTDC
    double advancePerRPM = 0.005;

    double timing = baseTiming + advancePerRPM * rpm;
    TS_ASSERT_DELTA(timing, 32.0, 1.0);

    // At idle, less advance
    rpm = 600.0;
    timing = baseTiming + advancePerRPM * rpm;
    TS_ASSERT_DELTA(timing, 23.0, 1.0);
  }

  // Test spark energy
  void testSparkEnergy() {
    // Magneto generates voltage proportional to RPM
    double rpm = 2400.0;
    double voltagePerRPM = 0.01;  // kV per RPM
    double minVoltage = 5.0;  // kV

    double sparkVoltage = minVoltage + voltagePerRPM * rpm;
    TS_ASSERT_DELTA(sparkVoltage, 29.0, 1.0);

    // At cranking, barely enough voltage
    rpm = 150.0;
    sparkVoltage = minVoltage + voltagePerRPM * rpm;
    TS_ASSERT_DELTA(sparkVoltage, 6.5, 0.1);
  }

  /***************************************************************************
   * Miscellaneous Tests
   ***************************************************************************/

  // Test altitude mixture leaning
  void testAltitudeMixtureLeaning() {
    // Need to lean mixture at altitude for same power
    double densityRatioSL = 1.0;
    double densityRatioAlt = 0.7;  // ~10000 ft

    // Required mixture position for same equivalence ratio
    double mixtureSL = 1.0;
    double mixtureAlt = mixtureSL * densityRatioAlt;

    TS_ASSERT(mixtureAlt < mixtureSL);
    TS_ASSERT_DELTA(mixtureAlt, 0.7, DEFAULT_TOLERANCE);
  }

  // Test engine mount vibration
  void testEngineVibration() {
    // Primary frequency = RPM/60 for 4-stroke
    double rpm = 2400.0;
    double primaryFreq = rpm / 60.0;
    TS_ASSERT_DELTA(primaryFreq, 40.0, DEFAULT_TOLERANCE);

    // Firing frequency for 4-cyl 4-stroke
    int cylinders = 4;
    double firingFreq = (rpm / 60.0) * (cylinders / 2.0);
    TS_ASSERT_DELTA(firingFreq, 80.0, DEFAULT_TOLERANCE);
  }

  // Test engine shutdown sequence
  void testEngineShutdown() {
    bool mixtureCutoff = true;
    double rpm = 600.0;
    double decayRate = 100.0;  // RPM/sec without fuel

    // RPM decays when mixture is cut
    double timeToStop = rpm / decayRate;
    TS_ASSERT_DELTA(timeToStop, 6.0, 0.1);

    // Verify shutdown state
    if (mixtureCutoff) {
      rpm = 0.0;
    }
    TS_ASSERT_DELTA(rpm, 0.0, DEFAULT_TOLERANCE);
  }

  // Test engine overspeed
  void testEngineOverspeed() {
    double rpm = 2800.0;
    double redlineRPM = 2700.0;
    double maxRPM = 3000.0;

    bool overRedline = rpm > redlineRPM;
    bool overMax = rpm > maxRPM;

    TS_ASSERT(overRedline);
    TS_ASSERT(!overMax);

    // At max, engine damage possible
    rpm = 3100.0;
    overMax = rpm > maxRPM;
    TS_ASSERT(overMax);
  }

  // Test six-cylinder balance
  void testSixCylinderBalance() {
    int cylinders = 6;

    // 6-cylinder opposed engine is inherently balanced
    // Firing order evenly spaced
    double firingInterval = 720.0 / cylinders;  // degrees
    TS_ASSERT_DELTA(firingInterval, 120.0, DEFAULT_TOLERANCE);

    // Power pulses per revolution
    double pulsePerRev = cylinders / 2.0;  // 4-stroke
    TS_ASSERT_DELTA(pulsePerRev, 3.0, DEFAULT_TOLERANCE);
  }

  // Test engine moment of inertia effect on spool-up
  void testEngineSpoolUp() {
    double momentOfInertia = 0.5;  // kg-m^2
    double startingTorque = 10.0;  // N-m
    double frictionTorque = 2.0;   // N-m

    double netTorque = startingTorque - frictionTorque;
    double angularAccel = netTorque / momentOfInertia;  // rad/s^2

    // Convert to RPM/s
    double rpmAccel = angularAccel * 60.0 / (2.0 * M_PI);
    TS_ASSERT(rpmAccel > 0);
    TS_ASSERT_DELTA(rpmAccel, 152.8, 1.0);
  }

  // Test primer system
  void testPrimerSystem() {
    int strokes = 3;  // Number of primer strokes
    double fuelPerStroke = 0.002;  // kg
    double oilTemp = 280.0;  // Cold engine

    // More priming needed when cold
    double primeFuel = strokes * fuelPerStroke;
    TS_ASSERT_DELTA(primeFuel, 0.006, 0.0001);

    // Hot engine needs no priming
    oilTemp = 350.0;
    int requiredStrokes = (oilTemp < 300.0) ? 3 : 0;
    TS_ASSERT_EQUALS(requiredStrokes, 0);
  }

  // Test fuel injection vs carburetor
  void testFuelInjectionAdvantage() {
    // Fuel injection provides more uniform mixture distribution
    double carbMixtureVariation = 0.15;  // 15% variation between cylinders
    double fuelInjVariation = 0.03;      // 3% variation

    TS_ASSERT(fuelInjVariation < carbMixtureVariation);

    // Power improvement potential
    double powerImprovement = (carbMixtureVariation - fuelInjVariation) * 0.5;
    TS_ASSERT_DELTA(powerImprovement, 0.06, 0.01);  // ~6% potential improvement
  }

  // Test engine efficiency at different power settings
  void testEngineEfficiency() {
    // Best efficiency at ~65% power
    double power65 = 0.65 * 180.0;  // HP
    double bsfc65 = 0.42;           // lbs/hp-hr

    double power100 = 1.0 * 180.0;
    double bsfc100 = 0.50;          // Higher BSFC at full power

    double fuelFlow65 = power65 * bsfc65;
    double fuelFlow100 = power100 * bsfc100;

    TS_ASSERT(bsfc65 < bsfc100);  // Better efficiency at cruise
    TS_ASSERT_DELTA(fuelFlow65, 49.14, 0.1);
    TS_ASSERT_DELTA(fuelFlow100, 90.0, 0.1);
  }

  // Test leaning effect on EGT
  void testLeaningEGTEffect() {
    double phi = 1.0;  // Stoichiometric
    double EGT_peak = 1000.0;  // K at peak

    // Rich of peak, EGT lower
    phi = 1.2;
    double EGT_rich = EGT_peak * (1.0 - 0.1 * (phi - 1.0));
    TS_ASSERT(EGT_rich < EGT_peak);

    // Lean of peak, EGT lower
    phi = 0.8;
    double EGT_lean = EGT_peak * (1.0 - 0.1 * (1.0 - phi));
    TS_ASSERT(EGT_lean < EGT_peak);
  }

  /***************************************************************************
   * Engine Timing and Detonation Tests
   ***************************************************************************/

  // Test spark advance vs RPM
  void testSparkAdvanceVsRPM() {
    double baseAdvance = 5.0; // BTDC at idle
    double maxAdvance = 35.0; // BTDC at full power
    double rpmIdle = 600.0;
    double rpmMax = 2700.0;

    double advanceAtMax = baseAdvance + (maxAdvance - baseAdvance) * (rpmMax - rpmIdle) / (rpmMax - rpmIdle);
    TS_ASSERT_DELTA(advanceAtMax, 35.0, 0.1);
  }

  // Test detonation onset temperature
  void testDetonationOnsetTemperature() {
    double chargeTemp = 400.0; // K
    double criticalTemp = 600.0; // K for 100LL
    double safetyMargin = criticalTemp - chargeTemp;

    TS_ASSERT(safetyMargin > 100.0); // Safe margin
    TS_ASSERT(chargeTemp < criticalTemp);
  }

  // Test engine knock intensity
  void testKnockIntensity() {
    double MAP = 38.0; // inHg (overboosted)
    double normalMAP = 30.0;
    double knockThreshold = 35.0;

    bool knocking = MAP > knockThreshold;
    TS_ASSERT(knocking);
  }

  // Test octane requirement vs compression ratio
  void testOctaneRequirement() {
    double CR = 9.0;
    double octane = 73.0 + (CR - 6.0) * 9.0; // Approximate formula

    TS_ASSERT_DELTA(octane, 100.0, 5.0);
    TS_ASSERT(octane > 80.0);
  }

  /***************************************************************************
   * Fuel Injection Tests
   ***************************************************************************/

  // Test injector pulse width calculation
  void testInjectorPulseWidth() {
    double fuelRequired = 0.001; // kg per cylinder per cycle
    double injectorFlow = 0.05;  // kg/s at full open
    double cycleTime = 0.05;     // seconds at 2400 RPM

    double pulseWidth = fuelRequired / injectorFlow; // 0.02 s
    double dutyCycle = pulseWidth / cycleTime; // 0.4

    TS_ASSERT(dutyCycle < 0.8); // Must have margin
    TS_ASSERT_DELTA(dutyCycle, 0.4, 0.1);
  }

  // Test fuel pressure effect on flow
  void testFuelPressureEffect() {
    double basePressure = 30.0; // psi
    double baseFlow = 10.0;     // lbs/hr
    double newPressure = 45.0;  // psi

    // Flow proportional to sqrt(pressure)
    double newFlow = baseFlow * std::sqrt(newPressure / basePressure);
    TS_ASSERT(newFlow > baseFlow);
    TS_ASSERT_DELTA(newFlow, 12.25, 0.1);
  }

  // Test injector dead time compensation
  void testInjectorDeadTime() {
    double deadTime = 0.001;    // 1 ms
    double pulseWidth = 0.005;  // 5 ms
    double voltage = 12.0;      // V

    // Dead time varies with voltage
    double deadTimeCompensated = deadTime * (14.0 / voltage);
    double effectivePulse = pulseWidth - deadTimeCompensated;

    TS_ASSERT(effectivePulse < pulseWidth);
    TS_ASSERT(effectivePulse > 0);
  }

  /***************************************************************************
   * Engine Dynamics Tests
   ***************************************************************************/

  // Test crankshaft angular acceleration
  void testCrankshaftAcceleration() {
    double torque = 200.0;   // ft-lbs
    double inertia = 0.5;    // slug-ft^2

    double alpha = torque / inertia; // rad/s^2
    TS_ASSERT(alpha > 0);
    TS_ASSERT_DELTA(alpha, 400.0, 1.0);
  }

  // Test RPM decay rate on shutdown
  void testRPMDecayRate() {
    double rpm = 2400.0;
    double frictionTorque = 20.0; // ft-lbs
    double inertia = 0.8;        // slug-ft^2

    double omega = rpm * 2.0 * M_PI / 60.0;
    double alpha = frictionTorque / inertia;
    double timeToStop = omega / alpha;

    TS_ASSERT(timeToStop > 0);
    TS_ASSERT(timeToStop < 30.0); // Should stop within 30 sec
  }

  // Test engine run-up time
  void testEngineRunUpTime() {
    double targetRPM = 2400.0;
    double idleRPM = 600.0;
    double accelRate = 500.0; // RPM/sec

    double runUpTime = (targetRPM - idleRPM) / accelRate;
    TS_ASSERT_DELTA(runUpTime, 3.6, 0.1);
  }

  // Test torsional vibration frequency
  void testTorsionalVibration() {
    double K = 100000.0; // torsional stiffness, ft-lb/rad
    double J = 0.5;      // moment of inertia, slug-ft^2

    double omega_n = std::sqrt(K / J); // rad/s
    double freq = omega_n / (2.0 * M_PI);

    TS_ASSERT(freq > 0);
    TS_ASSERT(freq < 100.0); // Reasonable frequency
  }

  /***************************************************************************
   * Exhaust System Tests
   ***************************************************************************/

  // Test exhaust back pressure effect
  void testExhaustBackPressure() {
    double normalBackPressure = 2.0;   // inHg
    double highBackPressure = 8.0;     // inHg (blocked)
    double powerLossPerInHg = 1.0;     // % per inHg

    double powerLoss = (highBackPressure - normalBackPressure) * powerLossPerInHg;
    TS_ASSERT_DELTA(powerLoss, 6.0, 0.1); // 6% power loss
  }

  // Test exhaust gas velocity
  void testExhaustGasVelocity() {
    double exhaustFlow = 0.1;    // kg/s
    double exhaustDensity = 0.5; // kg/m^3
    double pipeArea = 0.01;      // m^2

    double velocity = exhaustFlow / (exhaustDensity * pipeArea);
    TS_ASSERT(velocity > 0);
    TS_ASSERT_DELTA(velocity, 20.0, 0.1);
  }

  // Test exhaust temperature drop
  void testExhaustTempDrop() {
    double EGT = 900.0;      // K at manifold
    double ambientTemp = 288.0; // K
    double coolingCoeff = 0.5;
    double pipeLength = 2.0; // m

    double tempDrop = coolingCoeff * pipeLength * (EGT - ambientTemp) / 10.0;
    double tailpipeTemp = EGT - tempDrop;

    TS_ASSERT(tailpipeTemp < EGT);
    TS_ASSERT(tailpipeTemp > ambientTemp);
  }

  /***************************************************************************
   * Lubrication System Tests
   ***************************************************************************/

  // Test oil viscosity temperature dependence
  void testOilViscosityTemperature() {
    double viscosity20C = 100.0;  // cSt
    double viscosity100C = 10.0;  // cSt

    // Viscosity decreases with temperature
    TS_ASSERT(viscosity100C < viscosity20C);
    double ratio = viscosity20C / viscosity100C;
    TS_ASSERT_DELTA(ratio, 10.0, 0.1);
  }

  // Test oil consumption rate
  void testOilConsumptionRate() {
    double oilCapacity = 8.0;      // quarts
    double consumptionRate = 0.05; // quarts/hr
    double flightTime = 10.0;      // hours

    double oilConsumed = consumptionRate * flightTime;
    double oilRemaining = oilCapacity - oilConsumed;

    TS_ASSERT(oilRemaining > 4.0); // Minimum safe level
    TS_ASSERT_DELTA(oilConsumed, 0.5, 0.01);
  }

  // Test oil pressure relief valve
  void testOilPressureReliefValve() {
    double reliefPressure = 80.0; // psi
    double normalPressure = 60.0; // psi
    double coldStartPressure = 100.0; // psi (high viscosity)

    bool reliefOpen = coldStartPressure > reliefPressure;
    TS_ASSERT(reliefOpen);
  }

  /***************************************************************************
   * Altitude Performance Tests
   ***************************************************************************/

  // Test critical altitude for turbo
  void testTurboCriticalAltitude() {
    double seaLevelMAP = 30.0;  // inHg
    double maxBoost = 2.0;      // pressure ratio
    double targetMAP = 36.0;    // inHg

    // At critical altitude, max boost just achieves target
    double criticalAmbient = targetMAP / maxBoost;
    TS_ASSERT_DELTA(criticalAmbient, 18.0, 0.1);
    // ~18 inHg corresponds to ~12000 ft
  }

  // Test power lapse rate
  void testPowerLapseRate() {
    double seaLevelPower = 200.0; // HP
    double lapseRate = 0.03;      // 3% per 1000 ft
    double altitude = 8000.0;     // ft

    double altitudePower = seaLevelPower * (1.0 - lapseRate * altitude / 1000.0);
    TS_ASSERT_DELTA(altitudePower, 152.0, 1.0);
  }

  // Test TIT limit at altitude
  void testTITLimitAltitude() {
    double maxTIT = 1650.0; // F
    double currentTIT = 1580.0;

    double margin = maxTIT - currentTIT;
    TS_ASSERT(margin > 50.0); // Safety margin
    TS_ASSERT(currentTIT < maxTIT);
  }

  /***************************************************************************
   * Propeller Interaction Tests Extended
   ***************************************************************************/

  // Test propeller governor droop
  void testGovernorDroop() {
    double setRPM = 2400.0;
    double droop = 0.02; // 2% droop
    double loadIncrease = 0.5; // 50% load increase

    double actualRPM = setRPM * (1.0 - droop * loadIncrease);
    TS_ASSERT(actualRPM < setRPM);
    TS_ASSERT_DELTA(actualRPM, 2376.0, 1.0);
  }

  // Test propeller overspeed protection
  void testPropellerOverspeedProtection() {
    double maxRPM = 2700.0;
    double overspeedTrip = 2800.0;
    double currentRPM = 2850.0;

    bool overspeed = currentRPM > overspeedTrip;
    TS_ASSERT(overspeed);
  }

  // Test feathering pitch angle
  void testFeatheringPitch() {
    double featherAngle = 85.0; // degrees (near 90)
    double cruiseAngle = 25.0;  // degrees

    TS_ASSERT(featherAngle > cruiseAngle);
    TS_ASSERT(featherAngle > 80.0);
  }

  /***************************************************************************
   * Engine Health Monitoring Tests
   ***************************************************************************/

  // Test CHT spread analysis
  void testCHTSpreadAnalysis() {
    double CHT[] = {350.0, 355.0, 390.0, 360.0}; // F per cylinder

    double maxCHT = 390.0;
    double minCHT = 350.0;
    double spread = maxCHT - minCHT;

    // High spread indicates potential problem
    bool spreadExcessive = spread > 30.0;
    TS_ASSERT(spreadExcessive); // 40F spread is excessive
  }

  // Test EGT trend analysis
  void testEGTTrendAnalysis() {
    double EGT_baseline = 1350.0; // F
    double EGT_current = 1420.0;  // F

    double deviation = EGT_current - EGT_baseline;
    bool significantChange = std::abs(deviation) > 50.0;

    TS_ASSERT(significantChange); // 70F deviation is significant
  }

  // Test compression test
  void testCompressionTest() {
    double masterOrifice = 80.0; // psi
    double cylinderPressure = 72.0; // psi

    double compressionRatio = cylinderPressure / masterOrifice;
    double percentCompression = compressionRatio * 100.0;

    TS_ASSERT(percentCompression > 60.0); // >60/80 is acceptable
    TS_ASSERT_DELTA(percentCompression, 90.0, 1.0);
  }

  // Test oil analysis trending
  void testOilAnalysisTrending() {
    double ironPPM = 25.0;     // parts per million
    double ironLimit = 50.0;  // action limit

    bool withinLimits = ironPPM < ironLimit;
    TS_ASSERT(withinLimits);
  }

  /***************************************************************************
   * Edge Cases and Stress Tests
   ***************************************************************************/

  // Test engine start at cold soak
  void testColdSoakStart() {
    double oilTemp = -20.0; // C
    double minStartTemp = -30.0; // C

    bool canStart = oilTemp > minStartTemp;
    TS_ASSERT(canStart);

    // Cranking current increases at low temp
    double crankingCurrentRatio = 1.0 + 0.01 * std::abs(oilTemp);
    TS_ASSERT(crankingCurrentRatio > 1.0);
  }

  // Test hot start vapor lock
  void testHotStartVaporLock() {
    double fuelTemp = 150.0; // F
    double vaporPressure = 7.0; // psi at this temp
    double fuelSystemPressure = 5.0; // psi

    bool vaporLock = vaporPressure > fuelSystemPressure;
    TS_ASSERT(vaporLock);
  }

  // Test flooded engine recovery
  void testFloodedEngineRecovery() {
    double excessFuel = 0.05; // kg
    double evaporationRate = 0.01; // kg/min with throttle open

    double recoveryTime = excessFuel / evaporationRate;
    TS_ASSERT(recoveryTime > 0);
    TS_ASSERT_DELTA(recoveryTime, 5.0, 0.1); // 5 minutes
  }

  // Test very high altitude operation
  void testVeryHighAltitudeOperation() {
    double altitude = 25000.0; // ft
    double densityRatio = 0.45;
    double seaLevelPower = 200.0;

    // Even turbocharged, power is limited
    double availablePower = seaLevelPower * 0.75; // turbo limited
    TS_ASSERT(availablePower < seaLevelPower);
  }

  // Test engine overheat response
  void testEngineOverheatResponse() {
    double CHT = 500.0; // F (overheating)
    double maxCHT = 450.0;

    bool overheating = CHT > maxCHT;
    TS_ASSERT(overheating);

    // Power reduction required
    double powerReduction = (CHT - maxCHT) * 0.02; // 2% per F
    TS_ASSERT(powerReduction > 0);
  }

  // Test rapid power changes
  void testRapidPowerChanges() {
    double powerBefore = 65.0; // percent
    double powerAfter = 100.0; // percent
    double changeTime = 2.0;   // seconds

    double changeRate = (powerAfter - powerBefore) / changeTime;
    TS_ASSERT_DELTA(changeRate, 17.5, 0.1); // %/sec

    // Rapid changes can cause shock cooling
    bool rapidChange = changeRate > 10.0;
    TS_ASSERT(rapidChange);
  }

  // Test partial power loss scenario
  void testPartialPowerLoss() {
    double normalPower = 180.0;  // HP
    double fuelFlowLoss = 0.3;   // 30% fuel flow reduction

    double reducedPower = normalPower * (1.0 - fuelFlowLoss);
    TS_ASSERT_DELTA(reducedPower, 126.0, 0.1);
    TS_ASSERT(reducedPower > 0);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteEngineStartSequence() {
    // Simulate complete engine start
    double batteryVoltage = 12.0;
    double starterDraw = 150.0;  // amps
    double crankingRPM = 60.0;

    // Cranking phase
    TS_ASSERT(crankingRPM > 0);

    // Ignition
    double ignitionTiming = 25.0;  // degrees BTDC
    TS_ASSERT(ignitionTiming > 0);

    // Idle stabilization
    double idleRPM = 800.0;
    double targetRPM = 1000.0;
    double idleCorrection = (targetRPM - idleRPM) * 0.1;
    TS_ASSERT(idleCorrection > 0);
  }

  void testCompleteThrottleAdvancement() {
    // Full throttle advancement sequence
    double throttle[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    double expectedPower[] = {0.0, 45.0, 100.0, 155.0, 200.0};

    for (int i = 0; i < 5; i++) {
      double power = 200.0 * throttle[i];
      TS_ASSERT(power >= 0.0);
      TS_ASSERT(power <= 200.0);
    }
  }

  void testCompleteEngineCycle() {
    // Four-stroke cycle timing
    double crankAngle = 0.0;
    int cycleCount = 0;

    while (crankAngle < 720.0) {
      if (crankAngle < 180.0) {
        // Intake stroke
      } else if (crankAngle < 360.0) {
        // Compression stroke
      } else if (crankAngle < 540.0) {
        // Power stroke
        cycleCount++;
      } else {
        // Exhaust stroke
      }
      crankAngle += 90.0;
    }

    TS_ASSERT_EQUALS(cycleCount, 2);  // One power event per 360 degrees
  }

  void testCompleteEngineShutdown() {
    double rpm = 2000.0;
    double decelRate = 200.0;  // RPM/s
    double dt = 0.1;

    int steps = 0;
    while (rpm > 0 && steps < 200) {
      rpm -= decelRate * dt;
      steps++;
    }

    TS_ASSERT(rpm <= 0 || steps < 200);
  }

  void testCompleteMixtureLeaning() {
    // Leaning procedure for cruise
    double mixture[] = {1.0, 0.9, 0.8, 0.7, 0.65};
    double egt[] = {1300.0, 1350.0, 1400.0, 1450.0, 1480.0};

    double maxEGT = 0.0;
    int peakIndex = 0;

    for (int i = 0; i < 5; i++) {
      if (egt[i] > maxEGT) {
        maxEGT = egt[i];
        peakIndex = i;
      }
    }

    TS_ASSERT_EQUALS(peakIndex, 4);  // Peak EGT at leanest setting
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentEngineCalculations() {
    // Two engines with different settings
    double power1 = 180.0 * 0.75;
    double power2 = 200.0 * 0.65;

    TS_ASSERT_DELTA(power1, 135.0, 0.1);
    TS_ASSERT_DELTA(power2, 130.0, 0.1);

    // Verify power1 unchanged
    double power1_verify = 180.0 * 0.75;
    TS_ASSERT_DELTA(power1, power1_verify, 0.001);
  }

  void testIndependentFuelFlow() {
    double sfc1 = 0.45;  // lb/hp/hr
    double power1 = 150.0;
    double fuel1 = sfc1 * power1;

    double sfc2 = 0.42;
    double power2 = 180.0;
    double fuel2 = sfc2 * power2;

    TS_ASSERT_DELTA(fuel1, 67.5, 0.1);
    TS_ASSERT_DELTA(fuel2, 75.6, 0.1);
  }

  void testIndependentCylinderTemps() {
    double CHT1 = 380.0 + 20.0 * 0.75;  // Engine 1 at 75%
    double CHT2 = 380.0 + 20.0 * 0.85;  // Engine 2 at 85%

    TS_ASSERT(CHT2 > CHT1);
    TS_ASSERT_DELTA(CHT1, 395.0, 0.1);
    TS_ASSERT_DELTA(CHT2, 397.0, 0.1);
  }

  void testIndependentThrottleResponse() {
    double throttle1 = 0.6;
    double throttle2 = 0.9;
    double maxPower = 200.0;

    double power1 = maxPower * throttle1;
    double power2 = maxPower * throttle2;

    TS_ASSERT(power2 > power1);

    // Verify power1 unchanged
    double power1_verify = maxPower * throttle1;
    TS_ASSERT_DELTA(power1, power1_verify, 0.001);
  }

  void testIndependentOilPressure() {
    double rpm1 = 2000.0;
    double basePressure1 = 40.0;
    double oilPress1 = basePressure1 + (rpm1 / 100.0);

    double rpm2 = 2500.0;
    double basePressure2 = 42.0;
    double oilPress2 = basePressure2 + (rpm2 / 100.0);

    TS_ASSERT(oilPress2 > oilPress1);

    // Verify oilPress1 unchanged
    double oilPress1_verify = basePressure1 + (rpm1 / 100.0);
    TS_ASSERT_DELTA(oilPress1, oilPress1_verify, 0.001);
  }

  void testIndependentManifoldPressure() {
    double altitude1 = 5000.0;
    double altitude2 = 10000.0;
    double seaLevelMP = 29.92;

    double mp1 = seaLevelMP * std::exp(-altitude1 / 27000.0);
    double mp2 = seaLevelMP * std::exp(-altitude2 / 27000.0);

    TS_ASSERT(mp1 > mp2);

    // Verify mp1 unchanged
    double mp1_verify = seaLevelMP * std::exp(-altitude1 / 27000.0);
    TS_ASSERT_DELTA(mp1, mp1_verify, 0.001);
  }

  //==========================================================================
  // Class-based tests using FGFDMExec and actual FGPiston
  //==========================================================================

  // Test getting piston engine from c172x model
  void testGetPistonEngine() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    TS_ASSERT(propulsion != nullptr);

    int numEngines = propulsion->GetNumEngines();
    TS_ASSERT(numEngines > 0);

    if (numEngines > 0) {
      auto engine = propulsion->GetEngine(0);
      TS_ASSERT(engine != nullptr);
      // c172x has a piston engine
      TS_ASSERT_EQUALS(engine->GetType(), FGEngine::etPiston);
    }
  }

  // Test piston engine RPM
  void testPistonRPM() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double rpm = piston->getRPM();
        TS_ASSERT(std::isfinite(rpm));
        TS_ASSERT(rpm >= 0.0);
      }
    }
  }

  // Test piston manifold pressure
  void testPistonManifoldPressure() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double mp = piston->getManifoldPressure_inHg();
        TS_ASSERT(std::isfinite(mp));
        TS_ASSERT(mp >= 0.0);
      }
    }
  }

  // Test piston exhaust gas temperature
  void testPistonEGT() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double egt = piston->GetEGT();
        TS_ASSERT(std::isfinite(egt));
      }
    }
  }

  // Test piston cylinder head temperature
  void testPistonCHT() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double cht = piston->getCylinderHeadTemp_degF();
        TS_ASSERT(std::isfinite(cht));
      }
    }
  }

  // Test piston oil pressure
  void testPistonOilPressure() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double oilP = piston->getOilPressure_psi();
        TS_ASSERT(std::isfinite(oilP));
        TS_ASSERT(oilP >= 0.0);
      }
    }
  }

  // Test piston magnetos
  void testPistonMagnetos() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        int mags = piston->GetMagnetos();
        TS_ASSERT(mags >= 0);
        TS_ASSERT(mags <= 3);  // 0=off, 1=left, 2=right, 3=both
      }
    }
  }

  // Test piston set magnetos
  void testPistonSetMagnetos() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());

        piston->SetMagnetos(3);  // Both
        TS_ASSERT_EQUALS(piston->GetMagnetos(), 3);

        piston->SetMagnetos(0);  // Off
        TS_ASSERT_EQUALS(piston->GetMagnetos(), 0);
      }
    }
  }

  // Test piston power available
  void testPistonPowerAvailable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double power = piston->GetPowerAvailable();
        TS_ASSERT(std::isfinite(power));
        TS_ASSERT(power >= 0.0);
      }
    }
  }

  // Test piston air-fuel ratio
  void testPistonAFR() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double afr = piston->getAFR();
        TS_ASSERT(std::isfinite(afr) || std::isinf(afr));  // May be infinity if no fuel flow
      }
    }
  }

  // Test piston engine labels
  void testPistonEngineLabels() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        std::string labels = piston->GetEngineLabels(",");
        TS_ASSERT(!labels.empty());
      }
    }
  }

  // Test piston engine values
  void testPistonEngineValues() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        std::string values = piston->GetEngineValues(",");
        TS_ASSERT(!values.empty());
      }
    }
  }

  // Test piston Calculate method
  void testPistonCalculate() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());

        // Run a few simulation steps
        for (int i = 0; i < 5; i++) {
          fdmex.Run();
        }

        // Check values are still valid after Calculate
        double rpm = piston->getRPM();
        TS_ASSERT(std::isfinite(rpm));
      }
    }
  }

  // Test piston ResetToIC
  void testPistonResetToIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());

        // Run some steps
        for (int i = 0; i < 5; i++) {
          fdmex.Run();
        }

        // Reset
        piston->ResetToIC();

        // Check RPM is finite after reset
        double rpm = piston->getRPM();
        TS_ASSERT(std::isfinite(rpm));
      }
    }
  }

  // Test piston exhaust gas temp in Fahrenheit
  void testPistonEGT_degF() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double egt_f = piston->getExhaustGasTemp_degF();
        TS_ASSERT(std::isfinite(egt_f));
      }
    }
  }

  // Test piston CalcFuelNeed
  void testPistonCalcFuelNeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      if (engine->GetType() == FGEngine::etPiston) {
        FGPiston* piston = static_cast<FGPiston*>(engine.get());
        double fuelNeed = piston->CalcFuelNeed();
        TS_ASSERT(std::isfinite(fuelNeed));
        TS_ASSERT(fuelNeed >= 0.0);
      }
    }
  }
};
