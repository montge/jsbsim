/*******************************************************************************
 * FGWeatherEffectsTest.h - Unit tests for weather effects on aircraft performance
 *
 * Tests mathematical modeling of environmental weather effects on aircraft:
 * - Rain effects on aerodynamics (drag, lift)
 * - Rain effects on engine performance
 * - Hail damage modeling
 * - Lightning strike effects
 * - Volcanic ash engine degradation
 * - Sand/dust ingestion
 * - Snow accumulation effects
 * - Frost effects on wing surfaces
 * - Temperature inversion effects
 * - Density altitude calculations
 * - Humidity effects on performance
 * - Visibility degradation modeling
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double tolerance = 1e-6;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double FT_TO_M = 0.3048;
const double M_TO_FT = 3.28084;
const double SLUGFT3_TO_KGCUM = 515.379;

class FGWeatherEffectsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Rain Effects on Drag
   ***************************************************************************/

  // Test rain drag coefficient increase
  void testRainDragIncrease() {
    double baseDrag = 0.025;
    double rainfall = 25.0;  // mm/hr (moderate rain)

    // Rain increases drag coefficient by approximately 5-10% per 25mm/hr
    double rainDragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    double totalDrag = baseDrag * rainDragFactor;

    TS_ASSERT(totalDrag > baseDrag);
    TS_ASSERT_DELTA(totalDrag, 0.02688, 0.001);
  }

  // Test light rain drag effect
  void testLightRainDrag() {
    double baseDrag = 0.025;
    double rainfall = 5.0;  // mm/hr (light rain)

    double rainDragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    double totalDrag = baseDrag * rainDragFactor;

    TS_ASSERT_DELTA(totalDrag, 0.02538, 0.001);
  }

  // Test heavy rain drag effect
  void testHeavyRainDrag() {
    double baseDrag = 0.025;
    double rainfall = 100.0;  // mm/hr (heavy rain)

    double rainDragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    double totalDrag = baseDrag * rainDragFactor;

    TS_ASSERT_DELTA(totalDrag, 0.0325, 0.001);
  }

  // Test no rain condition
  void testNoRainDrag() {
    double baseDrag = 0.025;
    double rainfall = 0.0;

    double rainDragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    double totalDrag = baseDrag * rainDragFactor;

    TS_ASSERT_DELTA(totalDrag, baseDrag, epsilon);
  }

  /***************************************************************************
   * Rain Effects on Lift
   ***************************************************************************/

  // Test rain lift coefficient reduction
  void testRainLiftReduction() {
    double baseLift = 0.45;
    double rainfall = 25.0;  // mm/hr

    // Rain reduces lift coefficient by approximately 3-5% per 25mm/hr
    double rainLiftFactor = 1.0 - 0.04 * (rainfall / 25.0);
    double totalLift = baseLift * rainLiftFactor;

    TS_ASSERT(totalLift < baseLift);
    TS_ASSERT_DELTA(totalLift, 0.432, 0.001);
  }

  // Test light rain lift reduction
  void testLightRainLift() {
    double baseLift = 0.45;
    double rainfall = 10.0;  // mm/hr

    double rainLiftFactor = 1.0 - 0.04 * (rainfall / 25.0);
    double totalLift = baseLift * rainLiftFactor;

    TS_ASSERT_DELTA(totalLift, 0.4428, 0.001);
  }

  // Test heavy rain lift reduction
  void testHeavyRainLift() {
    double baseLift = 0.45;
    double rainfall = 50.0;  // mm/hr

    double rainLiftFactor = 1.0 - 0.04 * (rainfall / 25.0);
    double totalLift = baseLift * rainLiftFactor;

    TS_ASSERT_DELTA(totalLift, 0.414, 0.001);
  }

  // Test wing surface roughness from rain
  void testRainSurfaceRoughness() {
    double dryRoughness = 0.0001;  // ft
    double rainfall = 25.0;         // mm/hr

    // Rain film increases effective surface roughness
    double wetRoughness = dryRoughness * (1.0 + rainfall / 100.0);

    TS_ASSERT(wetRoughness > dryRoughness);
    TS_ASSERT_DELTA(wetRoughness, 0.000125, tolerance);
  }

  /***************************************************************************
   * Heavy Rain Engine Power Loss
   ***************************************************************************/

  // Test engine power reduction in rain
  void testRainEnginePowerLoss() {
    double basePower = 200.0;  // hp
    double rainfall = 25.0;    // mm/hr

    // Heavy rain can reduce engine power by 2-5%
    double powerLossFactor = 1.0 - 0.03 * (rainfall / 25.0);
    double effectivePower = basePower * powerLossFactor;

    TS_ASSERT(effectivePower < basePower);
    TS_ASSERT_DELTA(effectivePower, 194.0, 0.1);
  }

  // Test water ingestion power loss
  void testWaterIngestionPowerLoss() {
    double basePower = 200.0;
    double waterIngestionRate = 0.5;  // lb/s

    // Water ingestion causes power loss
    double powerLoss = waterIngestionRate * 10.0;  // hp loss
    double effectivePower = basePower - powerLoss;

    TS_ASSERT_DELTA(effectivePower, 195.0, epsilon);
  }

  // Test jet engine flameout risk
  void testJetEngineFlameoutRisk() {
    double rainfall = 100.0;  // mm/hr (very heavy)
    double airspeed = 150.0;  // kts

    // Risk factor based on water flux
    double waterFlux = rainfall * airspeed / 1000.0;
    double flameoutRisk = std::min(1.0, waterFlux / 20.0);

    TS_ASSERT(flameoutRisk > 0);
    TS_ASSERT(flameoutRisk <= 1.0);
    TS_ASSERT_DELTA(flameoutRisk, 0.75, 0.01);
  }

  // Test compressor efficiency in rain
  void testCompressorEfficiencyRain() {
    double baseEfficiency = 0.85;
    double rainfall = 50.0;  // mm/hr

    // Rain reduces compressor efficiency
    double efficiencyFactor = 1.0 - 0.05 * (rainfall / 50.0);
    double wetEfficiency = baseEfficiency * efficiencyFactor;

    TS_ASSERT_DELTA(wetEfficiency, 0.8075, 0.001);
  }

  /***************************************************************************
   * Hail Damage Modeling
   ***************************************************************************/

  // Test hail impact force
  void testHailImpactForce() {
    double hailMass = 0.01;     // kg (1cm diameter)
    double airspeed = 250.0;    // kts
    double velocity = airspeed * 0.514444;  // m/s

    // Impact energy = 0.5 * mass * velocity^2
    double impactEnergy = 0.5 * hailMass * velocity * velocity;

    TS_ASSERT(impactEnergy > 0);
    TS_ASSERT_DELTA(impactEnergy, 82.7, 1.0);
  }

  // Test hail damage to leading edge
  void testHailLeadingEdgeDamage() {
    double hailSize = 2.0;    // cm diameter
    double airspeed = 200.0;  // kts

    // Damage severity index
    double damageSeverity = (hailSize / 5.0) * (airspeed / 250.0);

    TS_ASSERT(damageSeverity > 0);
    TS_ASSERT_DELTA(damageSeverity, 0.32, 0.01);
  }

  // Test hail aerodynamic degradation
  void testHailAerodynamicDegradation() {
    double baseDrag = 0.025;
    double hailDamageIndex = 0.5;  // 0 to 1 scale

    // Hail damage increases drag
    double damagedDrag = baseDrag * (1.0 + 0.2 * hailDamageIndex);

    TS_ASSERT(damagedDrag > baseDrag);
    TS_ASSERT_DELTA(damagedDrag, 0.0275, epsilon);
  }

  // Test windshield impact resistance
  void testWindshieldHailImpact() {
    double hailKineticEnergy = 100.0;  // Joules
    double glassStrength = 150.0;      // Joules threshold

    bool windshieldIntact = (hailKineticEnergy < glassStrength);

    TS_ASSERT(windshieldIntact);
  }

  /***************************************************************************
   * Lightning Strike Effects
   ***************************************************************************/

  // Test lightning current magnitude
  void testLightningCurrentMagnitude() {
    double peakCurrent = 30000.0;  // Amperes (typical)
    double duration = 0.0002;      // seconds

    // Charge transfer
    double charge = peakCurrent * duration;

    TS_ASSERT(charge > 0);
    TS_ASSERT_DELTA(charge, 6.0, epsilon);  // Coulombs
  }

  // Test electromagnetic interference
  void testLightningEMI() {
    double current = 30000.0;  // A
    double distance = 10.0;    // m

    // Magnetic field strength (simplified)
    double magneticField = current / (2.0 * M_PI * distance);

    TS_ASSERT(magneticField > 0);
    TS_ASSERT_DELTA(magneticField, 477.46, 1.0);  // A/m
  }

  // Test avionics disruption probability
  void testAvionicsDisruption() {
    double strikeIntensity = 0.8;  // 0 to 1 scale
    double shielding = 0.6;        // 0 to 1 scale

    double disruptionProbability = strikeIntensity * (1.0 - shielding);

    TS_ASSERT_DELTA(disruptionProbability, 0.32, epsilon);
  }

  // Test composite material damage
  void testCompositeLightningDamage() {
    double strikeEnergy = 1e6;  // Joules
    double conductivity = 0.1;   // Relative to metal

    // Damage inversely proportional to conductivity
    double damageIndex = strikeEnergy * (1.0 - conductivity) / 1e6;

    TS_ASSERT_DELTA(damageIndex, 0.9, epsilon);
  }

  /***************************************************************************
   * Volcanic Ash Effects on Engines
   ***************************************************************************/

  // Test ash particle ingestion
  void testAshParticleIngestion() {
    double ashConcentration = 2.0;  // mg/m³
    double airflowRate = 100.0;     // kg/s

    // Ash ingestion rate
    double ashRate = (ashConcentration / 1e6) * airflowRate;

    TS_ASSERT(ashRate > 0);
    TS_ASSERT_DELTA(ashRate, 0.0002, tolerance);  // kg/s
  }

  // Test turbine blade erosion
  void testTurbineBladeErosion() {
    double ashExposureTime = 600.0;  // seconds (10 min)
    double ashConcentration = 4.0;   // mg/m³

    // Erosion depth (simplified)
    double erosionRate = 0.001;  // mm per mg/m³ per hour
    double erosionDepth = erosionRate * ashConcentration * (ashExposureTime / 3600.0);

    TS_ASSERT_DELTA(erosionDepth, 0.00067, 0.0001);  // mm
  }

  // Test engine temperature rise from ash
  void testAshEngineTemperatureRise() {
    double baseEGT = 1200.0;  // °F
    double ashConcentration = 5.0;  // mg/m³

    // Ash deposition increases EGT
    double tempRise = ashConcentration * 20.0;  // °F per mg/m³
    double effectiveEGT = baseEGT + tempRise;

    TS_ASSERT_DELTA(effectiveEGT, 1300.0, epsilon);
  }

  // Test compressor stall risk
  void testAshCompressorStall() {
    double ashDeposit = 0.5;    // kg accumulated
    double flowReduction = ashDeposit * 0.1;  // 10% per kg

    double stallRisk = std::min(1.0, flowReduction);

    TS_ASSERT_DELTA(stallRisk, 0.05, epsilon);
  }

  // Test ash melting temperature
  void testAshMeltingPoint() {
    double ashSilicaContent = 0.7;  // 70% silica

    // Melting point varies with composition (°C)
    double meltingPoint = 1000.0 + ashSilicaContent * 200.0;

    TS_ASSERT_DELTA(meltingPoint, 1140.0, epsilon);
  }

  /***************************************************************************
   * Sand/Dust Ingestion
   ***************************************************************************/

  // Test sand particle erosion
  void testSandParticleErosion() {
    double sandConcentration = 10.0;  // mg/m³
    double particleSize = 50.0;       // microns

    // Erosion severity
    double erosionIndex = (sandConcentration / 10.0) * (particleSize / 100.0);

    TS_ASSERT_DELTA(erosionIndex, 0.5, epsilon);
  }

  // Test air filter clogging
  void testAirFilterClogging() {
    double dustAccumulation = 500.0;  // grams
    double filterCapacity = 1000.0;   // grams

    double cloggingFraction = dustAccumulation / filterCapacity;
    double flowReduction = cloggingFraction * 0.5;  // 50% at full capacity

    TS_ASSERT_DELTA(flowReduction, 0.25, epsilon);
  }

  // Test engine power loss from sand
  void testSandEnginePowerLoss() {
    double basePower = 250.0;  // hp
    double sandExposure = 0.3;  // 0 to 1 scale

    double powerLoss = basePower * sandExposure * 0.15;  // Up to 15% loss
    double effectivePower = basePower - powerLoss;

    TS_ASSERT_DELTA(effectivePower, 238.75, 0.01);
  }

  // Test bearing wear from dust
  void testBearingWearDust() {
    double operatingHours = 100.0;
    double dustConcentration = 5.0;  // mg/m³

    // Wear index
    double wearIndex = operatingHours * (dustConcentration / 10.0);

    TS_ASSERT_DELTA(wearIndex, 50.0, epsilon);
  }

  /***************************************************************************
   * Snow Accumulation Effects
   ***************************************************************************/

  // Test snow weight accumulation
  void testSnowWeightAccumulation() {
    double wingArea = 200.0;     // ft²
    double snowDepth = 0.25;     // inches
    double snowDensity = 6.0;    // lb/ft³ (wet snow)

    // Snow weight
    double snowWeight = wingArea * (snowDepth / 12.0) * snowDensity;

    TS_ASSERT_DELTA(snowWeight, 25.0, epsilon);  // lbs
  }

  // Test snow drag penalty
  void testSnowDragPenalty() {
    double baseDrag = 0.025;
    double snowRoughness = 0.5;  // inches

    // Snow increases drag significantly
    double dragIncrease = 1.0 + (snowRoughness * 0.3);
    double totalDrag = baseDrag * dragIncrease;

    TS_ASSERT_DELTA(totalDrag, 0.02875, epsilon);
  }

  // Test snow shedding rate
  void testSnowSheddingRate() {
    double snowMass = 50.0;      // lbs
    double airspeed = 120.0;     // kts
    double dynamicPressure = 0.5 * 0.002377 * (airspeed * 1.68781) * (airspeed * 1.68781);

    // Shedding rate proportional to dynamic pressure
    double sheddingRate = dynamicPressure * 0.01;  // lb/s

    TS_ASSERT(sheddingRate > 0);
    TS_ASSERT_DELTA(sheddingRate, 0.4875, 0.01);
  }

  // Test pitot tube blockage
  void testPitotTubeSnowBlockage() {
    double snowAccumulation = 0.1;  // inches
    double pitotDiameter = 0.25;    // inches

    bool blocked = (snowAccumulation > pitotDiameter * 0.3);

    TS_ASSERT(blocked);
  }

  /***************************************************************************
   * Frost Effects on Wing
   ***************************************************************************/

  // Test frost lift degradation
  void testFrostLiftDegradation() {
    double baseLift = 0.45;
    double frostThickness = 0.02;  // inches

    // Frost can reduce lift by 30% even with thin layer
    double liftReduction = std::min(0.3, frostThickness * 5.0);
    double frostyLift = baseLift * (1.0 - liftReduction);

    TS_ASSERT(frostyLift < baseLift);
    TS_ASSERT_DELTA(frostyLift, 0.405, 0.001);
  }

  // Test frost drag increase
  void testFrostDragIncrease() {
    double baseDrag = 0.025;
    double frostThickness = 0.02;  // inches

    double dragIncrease = 1.0 + frostThickness * 2.0;
    double frostyDrag = baseDrag * dragIncrease;

    TS_ASSERT_DELTA(frostyDrag, 0.026, epsilon);
  }

  // Test frost formation rate
  void testFrostFormationRate() {
    double temperature = -5.0;  // °C
    double dewPoint = -2.0;     // °C
    double humidity = 0.8;      // 80%

    // Frost forms when temp < dewpoint and below freezing
    bool frostForming = (temperature < 0.0) && (temperature < dewPoint);

    TS_ASSERT(frostForming);
  }

  // Test de-icing effectiveness
  void testDeIcingEffectiveness() {
    double frostMass = 10.0;     // lbs
    double heatFlux = 50.0;      // BTU/min
    double duration = 5.0;       // minutes

    // Heat of fusion for ice: ~144 BTU/lb
    double heatApplied = heatFlux * duration;
    double massMelted = heatApplied / 144.0;

    TS_ASSERT_DELTA(massMelted, 1.736, 0.001);
  }

  /***************************************************************************
   * Temperature Inversion Effects
   ***************************************************************************/

  // Test temperature inversion gradient
  void testTemperatureInversion() {
    double tempAtBase = 50.0;   // °F at 1000 ft
    double tempAtTop = 60.0;    // °F at 3000 ft
    double altDifference = 2000.0;  // ft

    // Positive lapse rate (inversion)
    double lapseRate = (tempAtTop - tempAtBase) / altDifference;

    TS_ASSERT(lapseRate > 0);  // Inverted (temp increases with altitude)
    TS_ASSERT_DELTA(lapseRate, 0.005, epsilon);  // °F per ft
  }

  // Test inversion layer thickness
  void testInversionLayerThickness() {
    double baseAltitude = 1000.0;  // ft
    double topAltitude = 3000.0;   // ft

    double thickness = topAltitude - baseAltitude;

    TS_ASSERT_DELTA(thickness, 2000.0, epsilon);
  }

  // Test trapped pollution effects
  void testTrappedPollution() {
    double visibility = 5.0;      // miles (normal 10)
    double inversionStrength = 0.7;  // 0 to 1

    double reducedVisibility = visibility * (1.0 - inversionStrength * 0.4);

    TS_ASSERT_DELTA(reducedVisibility, 3.6, epsilon);
  }

  /***************************************************************************
   * Density Altitude Effects
   ***************************************************************************/

  // Test density altitude calculation
  void testDensityAltitudeCalculation() {
    double pressureAltitude = 3000.0;  // ft
    double temperature = 95.0;          // °F
    double standardTemp = 59.0 - (3000.0 * 0.00356);  // ISA at 3000 ft

    // Density altitude approximation
    double densityAltitude = pressureAltitude +
                            118.8 * (temperature - standardTemp);

    TS_ASSERT(densityAltitude > pressureAltitude);
    TS_ASSERT_DELTA(densityAltitude, 8545.6, 1.0);
  }

  // Test high density altitude performance
  void testHighDensityAltitudePerformance() {
    double seaLevelPower = 200.0;  // hp
    double densityAltitude = 8000.0;  // ft

    // Power loss approximately 3% per 1000 ft
    double powerLossFactor = 1.0 - (densityAltitude / 1000.0) * 0.03;
    double effectivePower = seaLevelPower * powerLossFactor;

    TS_ASSERT(effectivePower < seaLevelPower);
    TS_ASSERT_DELTA(effectivePower, 152.0, 0.1);
  }

  // Test density ratio
  void testDensityRatio() {
    double actualDensity = 0.002;     // slugs/ft³
    double seaLevelDensity = 0.002377;  // slugs/ft³

    double densityRatio = actualDensity / seaLevelDensity;

    TS_ASSERT(densityRatio < 1.0);
    TS_ASSERT_DELTA(densityRatio, 0.8415, 0.001);
  }

  // Test takeoff distance increase
  void testTakeoffDistanceIncrease() {
    double seaLevelDistance = 1000.0;  // ft
    double densityRatio = 0.85;

    // Takeoff distance inversely proportional to density ratio
    double adjustedDistance = seaLevelDistance / densityRatio;

    TS_ASSERT_DELTA(adjustedDistance, 1176.47, 0.1);
  }

  /***************************************************************************
   * Humidity Effects on Performance
   ***************************************************************************/

  // Test humid air density
  void testHumidAirDensity() {
    double dryAirDensity = 0.002377;  // slugs/ft³
    double relativeHumidity = 0.8;     // 80%

    // Humid air is less dense (water vapor lighter than air)
    double humidityFactor = 1.0 - relativeHumidity * 0.01;
    double humidDensity = dryAirDensity * humidityFactor;

    TS_ASSERT(humidDensity < dryAirDensity);
    TS_ASSERT_DELTA(humidDensity, 0.002358, tolerance);
  }

  // Test humidity effect on engine power
  void testHumidityEnginePower() {
    double dryPower = 200.0;       // hp
    double relativeHumidity = 0.9;  // 90%

    // High humidity reduces power slightly (1-2%)
    double powerReduction = 1.0 - relativeHumidity * 0.015;
    double humidPower = dryPower * powerReduction;

    TS_ASSERT_DELTA(humidPower, 197.3, 0.1);
  }

  // Test water vapor partial pressure
  void testWaterVaporPressure() {
    double saturationPressure = 0.5;  // psi at given temp
    double relativeHumidity = 0.7;    // 70%

    double vaporPressure = saturationPressure * relativeHumidity;

    TS_ASSERT_DELTA(vaporPressure, 0.35, epsilon);
  }

  // Test virtual temperature
  void testVirtualTemperature() {
    double actualTemp = 300.0;  // K
    double mixingRatio = 0.01;  // kg water / kg dry air

    // Virtual temperature (temp dry air would have to have same density)
    double virtualTemp = actualTemp * (1.0 + 0.61 * mixingRatio);

    TS_ASSERT(virtualTemp > actualTemp);
    TS_ASSERT_DELTA(virtualTemp, 301.83, 0.01);
  }

  /***************************************************************************
   * Visibility Degradation
   ***************************************************************************/

  // Test fog visibility reduction
  void testFogVisibility() {
    double baseVisibility = 10.0;  // statute miles
    double fogDensity = 0.7;       // 0 to 1 scale

    double visibility = baseVisibility * (1.0 - fogDensity);

    TS_ASSERT_DELTA(visibility, 3.0, epsilon);
  }

  // Test smoke visibility
  void testSmokeVisibility() {
    double particleConcentration = 100.0;  // μg/m³
    double extinctionCoefficient = 4.0;     // m⁻¹ per 100 μg/m³

    // Visibility = 3.912 / extinction coefficient
    double visibility = 3.912 / (extinctionCoefficient * particleConcentration / 100.0);

    TS_ASSERT_DELTA(visibility, 0.978, 0.001);  // km
  }

  // Test rain visibility reduction
  void testRainVisibilityReduction() {
    double clearVisibility = 10.0;  // miles
    double rainfall = 50.0;         // mm/hr

    // Heavy rain reduces visibility significantly
    double visibilityFactor = 1.0 / (1.0 + rainfall / 100.0);
    double rainVisibility = clearVisibility * visibilityFactor;

    TS_ASSERT_DELTA(rainVisibility, 6.667, 0.01);
  }

  // Test contrast threshold
  void testContrastThreshold() {
    double objectLuminance = 100.0;
    double backgroundLuminance = 90.0;

    double contrast = std::abs(objectLuminance - backgroundLuminance) /
                     backgroundLuminance;

    TS_ASSERT_DELTA(contrast, 0.1111, 0.001);
  }

  // Test slant range visibility
  void testSlantRangeVisibility() {
    double horizontalVisibility = 5.0;  // miles
    double viewAngle = 30.0 * DEG_TO_RAD;  // degrees from horizontal

    // Slant range visibility often less than horizontal
    double slantVisibility = horizontalVisibility * std::cos(viewAngle);

    TS_ASSERT_DELTA(slantVisibility, 4.33, 0.01);
  }

  /***************************************************************************
   * Combined Effects Tests
   ***************************************************************************/

  // Test combined rain and wind effects
  void testCombinedRainWind() {
    double baseDrag = 0.025;
    double rainfall = 25.0;
    double gustFactor = 1.3;

    double rainDragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    double totalDrag = baseDrag * rainDragFactor;
    double effectiveDrag = totalDrag * gustFactor;

    TS_ASSERT_DELTA(effectiveDrag, 0.03494, 0.001);
  }

  // Test icing in clouds
  void testIcingInClouds() {
    double temperature = -10.0;  // °C
    double liquidWaterContent = 0.5;  // g/m³

    // Icing severity
    bool icingConditions = (temperature < 0.0) && (temperature > -40.0) &&
                          (liquidWaterContent > 0.1);

    TS_ASSERT(icingConditions);
  }

  // Test thunderstorm hazard index
  void testThunderstormHazardIndex() {
    double turbulence = 0.8;    // 0-1 scale
    double lightning = 0.6;     // 0-1 scale
    double hail = 0.4;          // 0-1 scale
    double rainfall = 0.9;      // 0-1 scale

    // Combined hazard index
    double hazardIndex = (turbulence + lightning + hail + rainfall) / 4.0;

    TS_ASSERT_DELTA(hazardIndex, 0.675, epsilon);
  }

  /***************************************************************************
   * Wind Shear Effects
   ***************************************************************************/

  // Test microburst wind shear
  void testMicroburstWindShear() {
    double altitude = 500.0;  // ft AGL
    double headwindAtAltitude = 30.0;  // kts
    double tailwindAtSurface = -20.0;  // kts (reverse)

    double shearGradient = (headwindAtAltitude - tailwindAtSurface) / altitude;
    TS_ASSERT_DELTA(shearGradient, 0.1, 0.001);  // 0.1 kts/ft
  }

  // Test gust front wind change
  void testGustFrontWindChange() {
    double preGustWind = 10.0;  // kts
    double postGustWind = 40.0;  // kts
    double timeToPass = 30.0;  // seconds

    double windChangeRate = (postGustWind - preGustWind) / timeToPass;
    TS_ASSERT_DELTA(windChangeRate, 1.0, 0.01);  // 1 kt/sec
  }

  // Test wind shear altitude profile
  void testWindShearAltitudeProfile() {
    double surfaceWind = 5.0;  // kts
    double wind1000ft = 25.0;  // kts
    double wind2000ft = 40.0;  // kts

    // Low-level shear is stronger
    double lowShear = (wind1000ft - surfaceWind) / 1000.0;
    double highShear = (wind2000ft - wind1000ft) / 1000.0;

    TS_ASSERT(lowShear > highShear);
  }

  /***************************************************************************
   * Icing Intensity Categories
   ***************************************************************************/

  // Test trace icing accumulation
  void testTraceIcing() {
    double LWC = 0.1;  // g/m³ (liquid water content)
    double temperature = -5.0;  // °C
    double accumulationRate = 0.01;  // inches/hr

    bool isTraceIcing = (accumulationRate < 0.1) && (LWC < 0.2);
    TS_ASSERT(isTraceIcing);
  }

  // Test light icing accumulation
  void testLightIcing() {
    double accumulationRate = 0.3;  // inches/hr

    bool isLightIcing = (accumulationRate >= 0.1) && (accumulationRate < 0.5);
    TS_ASSERT(isLightIcing);
  }

  // Test moderate icing accumulation
  void testModerateIcing() {
    double accumulationRate = 0.8;  // inches/hr

    bool isModerateIcing = (accumulationRate >= 0.5) && (accumulationRate < 1.0);
    TS_ASSERT(isModerateIcing);
  }

  // Test severe icing accumulation
  void testSevereIcing() {
    double accumulationRate = 1.5;  // inches/hr

    bool isSevereIcing = (accumulationRate >= 1.0);
    TS_ASSERT(isSevereIcing);
  }

  /***************************************************************************
   * Carburetor Icing
   ***************************************************************************/

  // Test carburetor icing temperature range
  void testCarbIcingTempRange() {
    double oat = 15.0;  // °C (outside air temp)
    double dewPoint = 12.0;  // °C

    // Carb ice most likely between -5°C and 25°C with high humidity
    bool inIcingRange = (oat >= -5.0) && (oat <= 25.0);
    bool highHumidity = (oat - dewPoint) < 5.0;
    bool carbIceRisk = inIcingRange && highHumidity;

    TS_ASSERT(carbIceRisk);
  }

  // Test carb heat effectiveness
  void testCarbHeatEffectiveness() {
    double intakeTemp = 10.0;  // °C without heat
    double tempRise = 40.0;  // °C typical carb heat rise

    double heatedTemp = intakeTemp + tempRise;
    bool aboveFreezing = heatedTemp > 0.0;

    TS_ASSERT_DELTA(heatedTemp, 50.0, epsilon);
    TS_ASSERT(aboveFreezing);
  }

  /***************************************************************************
   * Supercooled Large Droplets (SLD)
   ***************************************************************************/

  // Test SLD detection conditions
  void testSLDConditions() {
    double dropletSize = 50.0;  // microns (>40 = SLD)
    double temperature = -10.0;  // °C
    double LWC = 0.5;  // g/m³

    bool isSLD = (dropletSize > 40.0) && (temperature < 0.0);
    TS_ASSERT(isSLD);
  }

  // Test SLD runback ice formation
  void testSLDRunbackIce() {
    double dropletSize = 100.0;  // microns (large)
    double temperature = -5.0;  // °C (warm enough for runback)

    // Large droplets run back before freezing
    bool runbackRisk = (dropletSize > 50.0) && (temperature > -10.0);
    TS_ASSERT(runbackRisk);
  }

  /***************************************************************************
   * Convective Weather
   ***************************************************************************/

  // Test convective SIGMET criteria
  void testConvectiveSIGMETCriteria() {
    double radarReflectivity = 45.0;  // dBZ
    double cloudTops = 35000.0;  // ft
    double visibility = 1.0;  // miles

    bool meetsCriteria = (radarReflectivity >= 40.0) ||
                         (cloudTops >= 30000.0) ||
                         (visibility < 3.0);
    TS_ASSERT(meetsCriteria);
  }

  // Test hail probability vs reflectivity
  void testHailProbabilityVsReflectivity() {
    double reflectivity = 55.0;  // dBZ

    // Approximate probability based on reflectivity
    double hailProb = 0.0;
    if (reflectivity >= 60.0) hailProb = 0.9;
    else if (reflectivity >= 50.0) hailProb = 0.5;
    else if (reflectivity >= 40.0) hailProb = 0.2;

    TS_ASSERT_DELTA(hailProb, 0.5, epsilon);
  }

  // Test severe turbulence proximity to convection
  void testTurbulenceNearConvection() {
    double distanceFromCell = 10.0;  // nm
    double severeTurbRange = 20.0;  // nm

    bool severeTurbRisk = distanceFromCell <= severeTurbRange;
    TS_ASSERT(severeTurbRisk);
  }

  /***************************************************************************
   * Mountain Wave Effects
   ***************************************************************************/

  // Test mountain wave amplitude
  void testMountainWaveAmplitude() {
    double windSpeed = 40.0;  // kts
    double mountainHeight = 10000.0;  // ft
    double stabilityFactor = 0.5;

    // Wave amplitude proportional to wind and terrain
    double amplitude = stabilityFactor * windSpeed * (mountainHeight / 10000.0);
    TS_ASSERT_DELTA(amplitude, 20.0, epsilon);
  }

  // Test rotor turbulence below mountain wave
  void testRotorTurbulence() {
    double waveStrength = 0.8;  // 0-1 scale
    double altitudeAGL = 3000.0;  // ft
    double ridgeHeight = 8000.0;  // ft

    // Rotor typically found below ridge height
    bool rotorAltitude = altitudeAGL < ridgeHeight;
    double rotorIntensity = waveStrength * (1.0 - altitudeAGL / ridgeHeight);

    TS_ASSERT(rotorAltitude);
    TS_ASSERT(rotorIntensity > 0.0);
  }

  // Test lenticular cloud indication
  void testLenticularCloudIndication() {
    double windSpeed = 35.0;  // kts perpendicular to ridge
    double stability = 0.7;  // high stability

    // Lenticular clouds indicate mountain waves
    bool lenticularConditions = (windSpeed > 25.0) && (stability > 0.5);
    TS_ASSERT(lenticularConditions);
  }

  /***************************************************************************
   * Dust Devil and Whirlwind Effects
   ***************************************************************************/

  // Test dust devil formation conditions
  void testDustDevilConditions() {
    double surfaceTemp = 45.0;  // °C (hot surface)
    double windSpeed = 5.0;  // kts (light wind)
    double solarRadiation = 0.9;  // 0-1 scale

    bool dustDevilConditions = (surfaceTemp > 35.0) &&
                               (windSpeed < 10.0) &&
                               (solarRadiation > 0.7);
    TS_ASSERT(dustDevilConditions);
  }

  // Test dust devil wind speeds
  void testDustDevilWindSpeeds() {
    double rotationalSpeed = 40.0;  // kts (typical)
    double maxSpeed = 70.0;  // kts (strong dust devil)

    TS_ASSERT(rotationalSpeed > 30.0);
    TS_ASSERT(rotationalSpeed < maxSpeed);
  }

  /***************************************************************************
   * Sea Breeze and Land Breeze Effects
   ***************************************************************************/

  // Test sea breeze development
  void testSeaBreezeDevelopment() {
    double landTemp = 30.0;  // °C
    double seaTemp = 22.0;  // °C
    double tempDifferential = landTemp - seaTemp;

    bool seaBreezeConditions = tempDifferential > 5.0;
    TS_ASSERT(seaBreezeConditions);
  }

  // Test sea breeze front wind shear
  void testSeaBreezeFrontShear() {
    double onshoreWind = 15.0;  // kts (sea breeze)
    double offshoreWind = -5.0;  // kts (pre-front)

    double windChange = onshoreWind - offshoreWind;
    TS_ASSERT_DELTA(windChange, 20.0, epsilon);
  }

  // Test sea breeze penetration distance
  void testSeaBreezePenetration() {
    double tempDiff = 10.0;  // °C
    double timeOfDay = 14.0;  // hours (afternoon peak)

    // Penetration distance proportional to temp diff
    double penetration = tempDiff * 3.0;  // nm (simplified)
    TS_ASSERT_DELTA(penetration, 30.0, epsilon);
  }

  /***************************************************************************
   * Wake Turbulence in Weather
   ***************************************************************************/

  // Test wake vortex decay in rain
  void testWakeVortexDecayRain() {
    double normalDecayTime = 120.0;  // seconds
    double rainfall = 25.0;  // mm/hr

    // Rain accelerates vortex decay
    double decayFactor = 1.0 + (rainfall / 50.0);
    double acceleratedDecay = normalDecayTime / decayFactor;

    TS_ASSERT(acceleratedDecay < normalDecayTime);
  }

  // Test crosswind effect on wake transport
  void testCrosswindWakeTransport() {
    double crosswind = 8.0;  // kts
    double time = 60.0;  // seconds

    // Wake moves downwind
    double lateralTransport = crosswind * (time / 3600.0) * 6076.0;  // ft
    TS_ASSERT(lateralTransport > 0.0);
  }

  /***************************************************************************
   * Additional Visibility Effects
   ***************************************************************************/

  // Test mist vs fog classification
  void testMistVsFogClassification() {
    double visibility1 = 0.8;  // km (fog)
    double visibility2 = 3.0;  // km (mist)

    bool isFog = visibility1 < 1.0;
    bool isMist = visibility2 >= 1.0 && visibility2 < 5.0;

    TS_ASSERT(isFog);
    TS_ASSERT(isMist);
  }

  // Test haze effect on visibility
  void testHazeVisibilityEffect() {
    double clearVisibility = 50.0;  // km
    double hazeConcentration = 0.5;  // 0-1 scale

    double hazyVisibility = clearVisibility * (1.0 - hazeConcentration * 0.8);
    TS_ASSERT(hazyVisibility < clearVisibility);
    TS_ASSERT_DELTA(hazyVisibility, 30.0, epsilon);
  }

  // Test blowing snow visibility reduction
  void testBlowingSnowVisibility() {
    double windSpeed = 25.0;  // kts
    double snowOnGround = 6.0;  // inches

    // Visibility decreases with wind speed
    double visibilityMiles = 10.0 / (windSpeed / 10.0);
    TS_ASSERT_DELTA(visibilityMiles, 4.0, 0.1);
  }

  /***************************************************************************
   * Temperature Effects on Aircraft Performance
   ***************************************************************************/

  // Test cold weather oil viscosity
  void testColdWeatherOilViscosity() {
    double oat = -20.0;  // °C
    double baseViscosity = 100.0;  // cSt at 40°C

    // Viscosity increases exponentially with decreasing temp
    double coldViscosity = baseViscosity * std::exp(-0.02 * oat);
    TS_ASSERT(coldViscosity > baseViscosity);
  }

  // Test hot weather vapor lock risk
  void testHotWeatherVaporLock() {
    double oat = 40.0;  // °C
    double fuelTemp = oat + 15.0;  // Fuel in wing absorbs heat
    double vaporLockTemp = 60.0;  // °C threshold

    bool vaporLockRisk = fuelTemp >= vaporLockTemp - 5.0;
    TS_ASSERT(vaporLockRisk);
  }

  // Test cold weather battery performance
  void testColdWeatherBatteryPerformance() {
    double oat = -25.0;  // °C
    double batteryCapacityAt25C = 100.0;  // percent

    // Capacity decreases ~1% per degree below 0°C
    double capacityReduction = std::max(0.0, -oat) * 1.0;
    double actualCapacity = batteryCapacityAt25C - capacityReduction;

    TS_ASSERT(actualCapacity < batteryCapacityAt25C);
    TS_ASSERT_DELTA(actualCapacity, 75.0, epsilon);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test extreme rain scenario
  void testExtremeRainScenario() {
    double rainfall = 500.0;  // mm/hr (extreme monsoon)
    double baseDrag = 0.025;

    double rainDragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    double totalDrag = baseDrag * rainDragFactor;

    TS_ASSERT(totalDrag > baseDrag * 2.0);
    TS_ASSERT(!std::isnan(totalDrag));
  }

  // Test multiple simultaneous weather effects
  void testMultipleWeatherEffects() {
    double baseDrag = 0.025;
    double baseLift = 0.45;

    // Rain + frost + turbulence
    double rainFactor = 1.05;
    double frostFactor = 1.08;
    double turbulenceGustFactor = 1.15;

    double totalDrag = baseDrag * rainFactor * frostFactor * turbulenceGustFactor;
    double totalLift = baseLift * 0.96 * 0.90;  // Reductions

    TS_ASSERT(totalDrag > baseDrag);
    TS_ASSERT(totalLift < baseLift);
    TS_ASSERT(!std::isnan(totalDrag));
    TS_ASSERT(!std::isnan(totalLift));
  }

  // Test weather effects at boundary conditions
  void testWeatherBoundaryConditions() {
    // Zero rainfall
    double rainfall = 0.0;
    double dragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    TS_ASSERT_DELTA(dragFactor, 1.0, epsilon);

    // Maximum realistic rainfall
    rainfall = 200.0;
    dragFactor = 1.0 + 0.075 * (rainfall / 25.0);
    TS_ASSERT(dragFactor > 1.0);
    TS_ASSERT(!std::isinf(dragFactor));
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteWeatherScenario() {
    // Complete weather scenario: rain + wind + temperature
    double rainfall = 15.0;  // mm/hr
    double windSpeed = 25.0; // kts
    double temperature = 5.0; // C

    // Rain effect on drag
    double dragIncrease = 0.03 * (rainfall / 25.0);

    // Crosswind effect
    double crosswindFactor = windSpeed / 30.0;

    // Temperature effect on density
    double densityFactor = 1.0 + 0.001 * (15.0 - temperature);

    // Combined effect
    double totalEffect = (1.0 + dragIncrease) * densityFactor;
    TS_ASSERT(totalEffect > 1.0);
    TS_ASSERT(totalEffect < 1.5);
  }

  void testCompleteIcingEncounter() {
    // Complete icing scenario
    double temp = -5.0;  // C
    double humidity = 0.85;
    double lwc = 0.5;  // g/m^3 liquid water content

    // Icing severity
    double icingRate = lwc * (humidity) * std::exp(-0.1 * temp);
    TS_ASSERT(icingRate > 0.0);

    // Performance degradation
    double liftLoss = 0.15 * icingRate;
    double dragIncrease = 0.25 * icingRate;

    TS_ASSERT(liftLoss > 0.0);
    TS_ASSERT(dragIncrease > 0.0);
  }

  void testCompleteThunderstormPenetration() {
    // Thunderstorm effects
    double verticalGust = 50.0;  // ft/s
    double lateralGust = 30.0;   // ft/s
    double rainfall = 100.0;     // mm/hr

    // Combined gust magnitude
    double totalGust = std::sqrt(verticalGust * verticalGust + lateralGust * lateralGust);
    TS_ASSERT(totalGust > verticalGust);

    // Visibility reduction
    double visibility = 10.0 / (1.0 + rainfall / 50.0);  // km
    TS_ASSERT(visibility < 10.0);
    TS_ASSERT(visibility > 0.0);
  }

  void testCompleteTemperatureInversion() {
    // Temperature inversion scenario
    double altitudes[] = {0.0, 1000.0, 2000.0, 3000.0};
    double temps[] = {15.0, 12.0, 18.0, 10.0};  // Inversion at 2000ft

    double maxTemp = temps[0];
    int inversionLevel = 0;

    for (int i = 1; i < 4; i++) {
      if (temps[i] > temps[i-1]) {
        inversionLevel = i;
      }
      if (temps[i] > maxTemp) maxTemp = temps[i];
    }

    TS_ASSERT_EQUALS(inversionLevel, 2);  // Inversion detected
  }

  void testCompleteWindShearRecovery() {
    // Wind shear encounter and recovery
    double airspeed = 140.0;  // kts
    double shearMagnitude = 30.0;  // kts headwind loss

    double newAirspeed = airspeed - shearMagnitude;
    TS_ASSERT(newAirspeed < airspeed);

    // Recovery with pitch and power
    double pitchIncrease = 10.0;  // degrees
    double powerIncrease = 0.3;   // 30% more thrust

    double recoverySpeed = newAirspeed + powerIncrease * 50.0;
    TS_ASSERT(recoverySpeed > newAirspeed);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentWeatherZones() {
    // Two independent weather zones
    double rain1 = 10.0;
    double wind1 = 15.0;
    double drag1 = 1.0 + 0.03 * (rain1 / 25.0);

    double rain2 = 25.0;
    double wind2 = 30.0;
    double drag2 = 1.0 + 0.03 * (rain2 / 25.0);

    TS_ASSERT(drag2 > drag1);

    // Verify drag1 unchanged
    double drag1_verify = 1.0 + 0.03 * (rain1 / 25.0);
    TS_ASSERT_DELTA(drag1, drag1_verify, 0.0001);
  }

  void testIndependentIcingCalculations() {
    double temp1 = -3.0;
    double icing1 = 0.5 * std::exp(-0.1 * temp1);

    double temp2 = -10.0;
    double icing2 = 0.5 * std::exp(-0.1 * temp2);

    TS_ASSERT(icing2 > icing1);  // Colder = more icing

    // Verify icing1 unchanged
    double icing1_verify = 0.5 * std::exp(-0.1 * temp1);
    TS_ASSERT_DELTA(icing1, icing1_verify, 0.0001);
  }

  void testIndependentVisibilityCalculations() {
    double fog1 = 0.3;  // fog density
    double vis1 = 10.0 / (1.0 + 20.0 * fog1);

    double fog2 = 0.7;
    double vis2 = 10.0 / (1.0 + 20.0 * fog2);

    TS_ASSERT(vis1 > vis2);

    // Verify vis1 unchanged
    double vis1_verify = 10.0 / (1.0 + 20.0 * fog1);
    TS_ASSERT_DELTA(vis1, vis1_verify, 0.0001);
  }

  void testIndependentWindCalculations() {
    double headwind1 = 20.0;
    double crosswind1 = 10.0;
    double total1 = std::sqrt(headwind1*headwind1 + crosswind1*crosswind1);

    double headwind2 = 15.0;
    double crosswind2 = 25.0;
    double total2 = std::sqrt(headwind2*headwind2 + crosswind2*crosswind2);

    TS_ASSERT(total2 > total1);

    // Verify total1 unchanged
    double total1_verify = std::sqrt(headwind1*headwind1 + crosswind1*crosswind1);
    TS_ASSERT_DELTA(total1, total1_verify, 0.0001);
  }

  void testIndependentDensityCalculations() {
    double temp1 = 25.0;  // C
    double density1 = 1.225 * (273.15 / (273.15 + temp1));

    double temp2 = -10.0;
    double density2 = 1.225 * (273.15 / (273.15 + temp2));

    TS_ASSERT(density2 > density1);  // Colder = denser

    // Verify density1 unchanged
    double density1_verify = 1.225 * (273.15 / (273.15 + temp1));
    TS_ASSERT_DELTA(density1, density1_verify, 0.0001);
  }

  void testIndependentTurbulenceEffects() {
    double intensity1 = 5.0;  // ft/s RMS
    double load1 = 0.1 * intensity1;

    double intensity2 = 15.0;
    double load2 = 0.1 * intensity2;

    TS_ASSERT(load2 > load1);

    // Verify load1 unchanged
    double load1_verify = 0.1 * intensity1;
    TS_ASSERT_DELTA(load1, load1_verify, 0.0001);
  }
};
