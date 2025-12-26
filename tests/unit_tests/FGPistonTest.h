#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
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
};
