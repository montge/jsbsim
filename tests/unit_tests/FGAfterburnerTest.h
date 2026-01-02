#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGTurbine.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGAfterburnerTest : public CxxTest::TestSuite
{
public:
  // Test afterburner thrust augmentation ratio
  void testAfterburnerAugmentationRatio() {
    double milThrust = 15000.0;  // Military (dry) thrust
    double maxThrust = 24000.0;  // Maximum (wet) thrust with AB

    // Augmentation ratio = maxThrust / milThrust
    double augRatio = maxThrust / milThrust;
    TS_ASSERT_DELTA(augRatio, 1.6, DEFAULT_TOLERANCE);

    // Typical fighter aircraft AB provides 40-70% more thrust
    TS_ASSERT(augRatio >= 1.4);
    TS_ASSERT(augRatio <= 1.8);
  }

  // Test afterburner thrust augmentation ratio for various engine types
  void testAugmentationRatioByEngineType() {
    // Low bypass turbofan (F-15, F-16) - 50-60% augmentation
    double lowBypassMil = 17800.0;
    double lowBypassMax = 26400.0;
    double lowBypassRatio = lowBypassMax / lowBypassMil;
    TS_ASSERT_DELTA(lowBypassRatio, 1.483, 0.01);

    // Turbojet (older designs) - 60-70% augmentation
    double turbojetMil = 10000.0;
    double turbojetMax = 16500.0;
    double turbojetRatio = turbojetMax / turbojetMil;
    TS_ASSERT_DELTA(turbojetRatio, 1.65, DEFAULT_TOLERANCE);

    // Modern low bypass ratio should be less than turbojet
    TS_ASSERT(lowBypassRatio < turbojetRatio);
  }

  // Test fuel flow in afterburner
  void testAfterburnerFuelFlow() {
    double dryThrust = 15000.0;
    double wetThrust = 24000.0;
    double dryTSFC = 0.85;   // lbs/hr/lbf (dry)
    double wetATSFC = 1.9;   // lbs/hr/lbf (wet)

    // Dry fuel flow
    double fuelFlow_dry = dryThrust * dryTSFC;
    TS_ASSERT_DELTA(fuelFlow_dry, 12750.0, 0.1);

    // Wet fuel flow
    double fuelFlow_wet = wetThrust * wetATSFC;
    TS_ASSERT_DELTA(fuelFlow_wet, 45600.0, 0.1);

    // Afterburner dramatically increases fuel consumption
    TS_ASSERT(fuelFlow_wet > 3.0 * fuelFlow_dry);
  }

  // Test afterburner fuel flow ratio
  void testAfterburnerFuelFlowRatio() {
    // AB fuel flow is typically 2.5-4x dry fuel flow
    double dryFF = 12000.0;  // lbs/hr
    double wetFF = 38000.0;  // lbs/hr

    double ffRatio = wetFF / dryFF;
    TS_ASSERT_DELTA(ffRatio, 3.167, 0.01);
    TS_ASSERT(ffRatio >= 2.5);
    TS_ASSERT(ffRatio <= 4.5);
  }

  // Test exhaust gas temperature with reheat
  void testExhaustGasTempWithReheat() {
    double TAT = 25.0;  // Total air temp (°C)

    // Dry EGT at full military power
    double throttlePos = 1.0;
    double EGT_dry = TAT + 363.1 + throttlePos * 357.1;
    TS_ASSERT_DELTA(EGT_dry, 745.2, 0.1);

    // Wet EGT with afterburner (typically 100-300°C higher)
    double EGT_wet = EGT_dry + 200.0;  // Additional 200°C with AB
    TS_ASSERT_DELTA(EGT_wet, 945.2, 0.1);

    // AB EGT should not exceed material limits (~1200°C)
    TS_ASSERT(EGT_wet < 1200.0);
  }

  // Test EGT rise across afterburner
  void testEGTRiseAcrossAfterburner() {
    // Temperature rise depends on fuel/air ratio in AB
    double T_turbineExit = 900.0;  // K
    double fuelAirRatio_AB = 0.015; // Additional fuel in AB
    double fuelHeatValue = 18400.0; // BTU/lb
    double cp_gas = 0.295;  // BTU/lb-R

    // Simplified delta-T calculation
    double deltaT = (fuelAirRatio_AB * fuelHeatValue) / (cp_gas * (1 + fuelAirRatio_AB));

    // Temperature rise should be substantial (100-300K)
    TS_ASSERT(deltaT > 100.0);
    TS_ASSERT(deltaT < 1000.0);

    double T_ABExit = T_turbineExit + deltaT;
    TS_ASSERT(T_ABExit > T_turbineExit);
  }

  // Test nozzle area scheduling with afterburner
  void testNozzleAreaScheduling() {
    // Nozzle position: 0 = fully closed, 1 = fully open
    double N2norm = 0.8;

    // Dry operation: nozzle partially closed
    double nozzlePos_dry = 1.0 - N2norm;
    TS_ASSERT_DELTA(nozzlePos_dry, 0.2, DEFAULT_TOLERANCE);

    // Wet operation: nozzle fully open to accommodate higher mass flow
    double nozzlePos_wet = 1.0;
    TS_ASSERT_DELTA(nozzlePos_wet, 1.0, DEFAULT_TOLERANCE);

    // Nozzle must open when AB is active
    TS_ASSERT(nozzlePos_wet > nozzlePos_dry);
  }

  // Test nozzle area ratio for afterburner operation
  void testNozzleAreaRatio() {
    // Typical variable nozzle area ratio: 1.5:1 to 2.0:1
    double A_min = 1.0;  // Minimum area (dry)
    double A_max = 1.7;  // Maximum area (wet)

    double areaRatio = A_max / A_min;
    TS_ASSERT_DELTA(areaRatio, 1.7, DEFAULT_TOLERANCE);
    TS_ASSERT(areaRatio >= 1.4);
    TS_ASSERT(areaRatio <= 2.2);
  }

  // Test afterburner light-off conditions
  void testAfterburnerLightOffConditions() {
    // AB light-off requires sufficient core RPM and throttle position
    double N2_min = 97.0;  // Minimum N2 for AB light-off
    double throttlePos = 1.0;
    double N2 = 98.5;

    bool canLightOff = (N2 > N2_min) && (throttlePos > 0.99);
    TS_ASSERT(canLightOff);

    // Insufficient N2
    N2 = 90.0;
    canLightOff = (N2 > N2_min) && (throttlePos > 0.99);
    TS_ASSERT(!canLightOff);
  }

  // Test afterburner light-off delay
  void testAfterburnerLightOffDelay() {
    // AB light-off is not instantaneous
    double lightOffTime = 0.5;  // seconds
    double timeElapsed = 0.0;
    bool abActive = false;

    // Request AB
    double dt = 0.1;  // time step
    timeElapsed += dt;
    if (timeElapsed < lightOffTime) {
      abActive = false;
    }
    TS_ASSERT(!abActive);

    // After light-off delay
    timeElapsed = 0.6;
    if (timeElapsed >= lightOffTime) {
      abActive = true;
    }
    TS_ASSERT(abActive);
  }

  // Test afterburner zones/stages (3-stage AB)
  void testAfterburnerZonesStages() {
    // Many fighter aircraft have staged afterburners
    double abCommand = 0.0;  // 0 to 1.0

    // Stage 1: Minimum AB (abCommand = 0.33)
    abCommand = 0.33;
    int stage = static_cast<int>(abCommand * 3.0) + 1;
    TS_ASSERT_EQUALS(stage, 1);

    // Stage 2: Intermediate AB (abCommand = 0.66)
    abCommand = 0.66;
    stage = static_cast<int>(abCommand * 3.0) + 1;
    TS_ASSERT_EQUALS(stage, 2);

    // Stage 3: Maximum AB (abCommand = 1.0)
    abCommand = 1.0;
    stage = std::min(3, static_cast<int>(abCommand * 3.0) + 1);
    TS_ASSERT_EQUALS(stage, 3);
  }

  // Test thrust vs AB stage
  void testThrustVsABStage() {
    double milThrust = 15000.0;
    double maxThrust = 24000.0;
    double abCommand = 0.0;

    // Dry
    abCommand = 0.0;
    double thrust = milThrust + (maxThrust - milThrust) * abCommand;
    TS_ASSERT_DELTA(thrust, 15000.0, 0.1);

    // Partial AB (50%)
    abCommand = 0.5;
    thrust = milThrust + (maxThrust - milThrust) * abCommand;
    TS_ASSERT_DELTA(thrust, 19500.0, 0.1);

    // Full AB
    abCommand = 1.0;
    thrust = milThrust + (maxThrust - milThrust) * abCommand;
    TS_ASSERT_DELTA(thrust, 24000.0, 0.1);
  }

  // Test thrust specific fuel consumption with AB
  void testTSFCWithAfterburner() {
    // Dry TSFC
    double TSFC_dry = 0.8;  // lbs/hr/lbf

    // Wet ATSFC (typically 2.0-2.5x dry TSFC)
    double ATSFC_wet = 1.9;

    // ATSFC should be significantly higher
    double tsfcRatio = ATSFC_wet / TSFC_dry;
    TS_ASSERT_DELTA(tsfcRatio, 2.375, 0.01);
    TS_ASSERT(ATSFC_wet > 1.5 * TSFC_dry);
    TS_ASSERT(ATSFC_wet < 3.0 * TSFC_dry);
  }

  // Test TSFC vs AB stage
  void testTSFCvsABStage() {
    double TSFC_dry = 0.85;
    double ATSFC_max = 2.0;
    double abCommand = 0.0;

    // Interpolate TSFC based on AB command
    double TSFC = TSFC_dry + (ATSFC_max - TSFC_dry) * abCommand;

    // Dry
    abCommand = 0.0;
    TSFC = TSFC_dry + (ATSFC_max - TSFC_dry) * abCommand;
    TS_ASSERT_DELTA(TSFC, 0.85, DEFAULT_TOLERANCE);

    // Partial AB
    abCommand = 0.5;
    TSFC = TSFC_dry + (ATSFC_max - TSFC_dry) * abCommand;
    TS_ASSERT_DELTA(TSFC, 1.425, 0.01);

    // Full AB
    abCommand = 1.0;
    TSFC = TSFC_dry + (ATSFC_max - TSFC_dry) * abCommand;
    TS_ASSERT_DELTA(TSFC, 2.0, DEFAULT_TOLERANCE);
  }

  // Test afterburner blowout limits - low speed
  void testAfterburnerBlowoutLowSpeed() {
    // AB may blow out at low speeds due to insufficient ram pressure
    double airspeed = 100.0;  // kts
    double minAirspeed = 150.0;  // Minimum for stable AB

    bool abStable = (airspeed >= minAirspeed);
    TS_ASSERT(!abStable);

    // Above minimum speed
    airspeed = 200.0;
    abStable = (airspeed >= minAirspeed);
    TS_ASSERT(abStable);
  }

  // Test afterburner blowout limits - altitude
  void testAfterburnerBlowoutAltitude() {
    // AB effectiveness decreases with altitude
    double altitude = 50000.0;  // ft
    double maxABaltitude = 60000.0;  // Maximum altitude for AB

    bool abAvailable = (altitude < maxABaltitude);
    TS_ASSERT(abAvailable);

    // Above maximum altitude
    altitude = 70000.0;
    abAvailable = (altitude < maxABaltitude);
    TS_ASSERT(!abAvailable);
  }

  // Test afterburner blowout limits - dynamic pressure
  void testAfterburnerBlowoutDynamicPressure() {
    // AB stability depends on dynamic pressure (q)
    double qbar = 150.0;  // psf
    double minQbar = 100.0;  // Minimum for stable AB

    bool abStable = (qbar >= minQbar);
    TS_ASSERT(abStable);

    // Below minimum q
    qbar = 50.0;
    abStable = (qbar >= minQbar);
    TS_ASSERT(!abStable);
  }

  // Test pressure ratio across afterburner
  void testPressureRatioAcrossAfterburner() {
    // Afterburner has small pressure loss (typically 2-5%)
    double P_turbineExit = 14.7;  // psi
    double pressureLoss = 0.03;  // 3% loss

    double P_ABExit = P_turbineExit * (1.0 - pressureLoss);
    TS_ASSERT_DELTA(P_ABExit, 14.259, 0.01);

    // Pressure ratio across AB (should be < 1.0)
    double pressureRatio = P_ABExit / P_turbineExit;
    TS_ASSERT_DELTA(pressureRatio, 0.97, DEFAULT_TOLERANCE);
    TS_ASSERT(pressureRatio < 1.0);
    TS_ASSERT(pressureRatio > 0.90);
  }

  // Test afterburner pressure loss
  void testAfterburnerPressureLoss() {
    // Total pressure loss in AB (flameholders, mixing, friction)
    double Pt_in = 100.0;  // Total pressure in (psi)
    double lossCoeff = 0.04;  // 4% total pressure loss

    double Pt_out = Pt_in * (1.0 - lossCoeff);
    TS_ASSERT_DELTA(Pt_out, 96.0, DEFAULT_TOLERANCE);

    double deltaP = Pt_in - Pt_out;
    TS_ASSERT_DELTA(deltaP, 4.0, DEFAULT_TOLERANCE);
  }

  // Test afterburner efficiency
  void testAfterburnerEfficiency() {
    // Combustion efficiency in afterburner (typically 85-95%)
    double eta_AB = 0.90;

    TS_ASSERT(eta_AB >= 0.85);
    TS_ASSERT(eta_AB <= 0.98);

    // Energy added vs theoretical
    double Q_theoretical = 1000.0;  // BTU
    double Q_actual = Q_theoretical * eta_AB;
    TS_ASSERT_DELTA(Q_actual, 900.0, DEFAULT_TOLERANCE);
  }

  // Test afterburner efficiency vs fuel-air ratio
  void testAfterburnerEfficiencyVsFAR() {
    // Efficiency decreases at very rich or lean conditions
    double FAR_stoich = 0.067;  // Stoichiometric fuel-air ratio

    // Lean condition (FAR = 0.01)
    double FAR_lean = 0.01;
    double phi_lean = FAR_lean / FAR_stoich;  // Equivalence ratio
    TS_ASSERT(phi_lean < 1.0);

    // Rich condition (FAR = 0.02) - typical for AB
    double FAR_rich = 0.02;
    double phi_rich = FAR_rich / FAR_stoich;
    TS_ASSERT(phi_rich < 1.0);  // Still lean, but closer to stoichiometric

    // AB operates fuel-lean for temperature control
    TS_ASSERT(FAR_rich < FAR_stoich);
  }

  // Test inlet/nozzle matching
  void testInletNozzleMatching() {
    // Inlet recovery and nozzle expansion must be matched
    double inletRecovery = 0.95;  // 95% total pressure recovery
    double P0_freestream = 2116.22;  // psf

    double P0_inlet = P0_freestream * inletRecovery;
    TS_ASSERT_DELTA(P0_inlet, 2010.41, 0.1);

    // Nozzle throat area must accommodate mass flow
    // A_throat * P0 = constant for choked flow
    double A_throat_dry = 1.0;
    double P0_dry = 2000.0;
    double massFlowParam_dry = A_throat_dry * P0_dry;

    double A_throat_wet = 1.5;
    double P0_wet = 1980.0;  // Slightly lower due to AB pressure loss
    double massFlowParam_wet = A_throat_wet * P0_wet;

    // Higher mass flow parameter with AB
    TS_ASSERT(massFlowParam_wet > massFlowParam_dry);
  }

  // Test inlet pressure recovery with AB
  void testInletPressureRecoveryWithAB() {
    // Inlet recovery depends on Mach number
    double mach = 1.5;

    // Simplified recovery (decreases with Mach number)
    double recovery = 1.0 - 0.075 * (mach - 1.0);
    TS_ASSERT_DELTA(recovery, 0.9625, 0.001);

    // High Mach number
    mach = 2.0;
    recovery = 1.0 - 0.075 * (mach - 1.0);
    TS_ASSERT_DELTA(recovery, 0.925, 0.001);
    TS_ASSERT(recovery < 1.0);
  }

  // Test screech limits (high frequency combustion instability)
  void testScreechLimits() {
    // Screech occurs at certain fuel flow and pressure combinations
    double fuelFlow = 50000.0;  // lbs/hr
    double chamberPressure = 20.0;  // psi

    // Screech boundary (simplified)
    double screechParameter = fuelFlow / chamberPressure;
    double screechLimit = 3000.0;

    bool screechRisk = (screechParameter > screechLimit);
    TS_ASSERT(!screechRisk);

    // High fuel flow condition
    fuelFlow = 70000.0;
    screechParameter = fuelFlow / chamberPressure;
    screechRisk = (screechParameter > screechLimit);
    TS_ASSERT(screechRisk);
  }

  // Test rumble limits (low frequency combustion instability)
  void testRumbleLimits() {
    // Rumble occurs at low pressures and high fuel flows
    double P_AB = 8.0;  // psi
    double minPressure = 10.0;  // Minimum pressure to avoid rumble

    bool rumbleRisk = (P_AB < minPressure);
    TS_ASSERT(rumbleRisk);

    // Higher pressure
    P_AB = 15.0;
    rumbleRisk = (P_AB < minPressure);
    TS_ASSERT(!rumbleRisk);
  }

  // Test AB thrust vs altitude (sea level)
  void testABThrustAtSeaLevel() {
    double seaLevelThrust_dry = 15000.0;  // lbs
    double seaLevelThrust_wet = 24000.0;  // lbs
    double densityRatio = 1.0;  // Sea level

    // Thrust at sea level
    double thrust_dry = seaLevelThrust_dry * densityRatio;
    double thrust_wet = seaLevelThrust_wet * densityRatio;

    TS_ASSERT_DELTA(thrust_dry, 15000.0, 0.1);
    TS_ASSERT_DELTA(thrust_wet, 24000.0, 0.1);

    // Augmentation ratio at sea level
    double augRatio = thrust_wet / thrust_dry;
    TS_ASSERT_DELTA(augRatio, 1.6, DEFAULT_TOLERANCE);
  }

  // Test AB thrust vs altitude (30,000 ft)
  void testABThrustAt30kFeet() {
    double seaLevelThrust_dry = 15000.0;
    double seaLevelThrust_wet = 24000.0;
    double densityRatio = 0.374;  // ~30,000 ft

    // Thrust at altitude (roughly proportional to density)
    double thrust_dry = seaLevelThrust_dry * densityRatio;
    double thrust_wet = seaLevelThrust_wet * densityRatio;

    TS_ASSERT_DELTA(thrust_dry, 5610.0, 10.0);
    TS_ASSERT_DELTA(thrust_wet, 8976.0, 10.0);

    // Augmentation ratio remains approximately constant
    double augRatio = thrust_wet / thrust_dry;
    TS_ASSERT_DELTA(augRatio, 1.6, 0.01);
  }

  // Test AB thrust lapse with altitude
  void testABThrustLapseWithAltitude() {
    double seaLevelThrust = 24000.0;

    // Density ratios at various altitudes
    double sigma_SL = 1.0;
    double sigma_10k = 0.738;
    double sigma_30k = 0.374;
    double sigma_50k = 0.154;

    // Thrust at various altitudes
    double thrust_SL = seaLevelThrust * sigma_SL;
    double thrust_10k = seaLevelThrust * sigma_10k;
    double thrust_30k = seaLevelThrust * sigma_30k;
    double thrust_50k = seaLevelThrust * sigma_50k;

    // Verify thrust decreases with altitude
    TS_ASSERT(thrust_SL > thrust_10k);
    TS_ASSERT(thrust_10k > thrust_30k);
    TS_ASSERT(thrust_30k > thrust_50k);

    TS_ASSERT_DELTA(thrust_50k, 3696.0, 1.0);
  }

  // Test AB thrust vs Mach number
  void testABThrustVsMachNumber() {
    double staticThrust = 24000.0;
    double mach = 0.0;

    // Static thrust
    mach = 0.0;
    double ramEffect = 1.0;  // No ram effect at M=0
    double thrust = staticThrust * ramEffect;
    TS_ASSERT_DELTA(thrust, 24000.0, 0.1);

    // Subsonic (M=0.9) - slight ram effect
    mach = 0.9;
    ramEffect = 1.0 + 0.1 * mach;  // Simplified
    thrust = staticThrust * ramEffect;
    TS_ASSERT(thrust > staticThrust);

    // Supersonic (M=1.5) - significant ram effect
    mach = 1.5;
    ramEffect = 1.0 + 0.1 * mach;
    thrust = staticThrust * ramEffect;
    TS_ASSERT(thrust > 1.1 * staticThrust);
  }

  // Test AB activation method 0 (property-based)
  void testABActivationMethod0() {
    int augMethod = 0;
    bool augmentation = false;

    // Augmentation controlled by property
    augmentation = true;
    TS_ASSERT(augmentation);

    augmentation = false;
    TS_ASSERT(!augmentation);
  }

  // Test AB activation method 1 (throttle > 99%)
  void testABActivationMethod1() {
    int augMethod = 1;
    double throttlePos = 1.0;
    double N2 = 98.5;
    bool augmentation = false;

    // Throttle > 99% and N2 > 97%
    if ((throttlePos > 0.99) && (N2 > 97.0)) {
      augmentation = true;
    }
    TS_ASSERT(augmentation);

    // Throttle below threshold
    throttlePos = 0.98;
    augmentation = false;
    if ((throttlePos > 0.99) && (N2 > 97.0)) {
      augmentation = true;
    }
    TS_ASSERT(!augmentation);
  }

  // Test AB activation method 2 (throttle > 1.0)
  void testABActivationMethod2() {
    int augMethod = 2;
    double throttleCmd = 1.5;  // Throttle extended beyond 1.0
    double augmentCmd = 0.0;

    // Extract AB command
    if (throttleCmd > 1.0) {
      augmentCmd = throttleCmd - 1.0;
    }

    TS_ASSERT_DELTA(augmentCmd, 0.5, DEFAULT_TOLERANCE);
    TS_ASSERT(augmentCmd > 0.0);
  }

  // Test AB spool-up time
  void testABSpoolUpTime() {
    // AB light-off and thrust increase takes time
    double thrust_dry = 15000.0;
    double thrust_wet_target = 24000.0;
    double thrust_current = thrust_dry;
    double abSpoolRate = 10000.0;  // lbs/sec
    double dt = 0.1;  // time step

    // One time step
    if (thrust_current < thrust_wet_target) {
      thrust_current += abSpoolRate * dt;
    }
    TS_ASSERT_DELTA(thrust_current, 16000.0, 0.1);

    // Continue to target
    thrust_current = thrust_dry;
    double time = 0.0;
    while (thrust_current < thrust_wet_target && time < 2.0) {
      thrust_current += abSpoolRate * dt;
      time += dt;
    }
    TS_ASSERT(thrust_current >= thrust_wet_target - 1.0);
  }

  // Test AB shutdown time
  void testABShutdownTime() {
    // AB shutdown is faster than spool-up
    double thrust_wet = 24000.0;
    double thrust_dry_target = 15000.0;
    double thrust_current = thrust_wet;
    double abShutdownRate = 15000.0;  // lbs/sec (faster than spool-up)
    double dt = 0.1;

    // One time step
    if (thrust_current > thrust_dry_target) {
      thrust_current -= abShutdownRate * dt;
    }
    TS_ASSERT_DELTA(thrust_current, 22500.0, 0.1);

    // Shutdown rate faster than spool-up
    double abSpoolRate = 10000.0;
    TS_ASSERT(abShutdownRate > abSpoolRate);
  }

  // Test maximum AB usage time limit
  void testMaxABUsageTimeLimit() {
    // Some aircraft limit continuous AB usage
    double abTimer = 0.0;
    double maxABTime = 300.0;  // 5 minutes max continuous AB
    bool abAllowed = true;

    // Within limit
    abTimer = 120.0;
    abAllowed = (abTimer < maxABTime);
    TS_ASSERT(abAllowed);

    // Exceeds limit
    abTimer = 350.0;
    abAllowed = (abTimer < maxABTime);
    TS_ASSERT(!abAllowed);
  }

  // Test AB thrust variation with temperature
  void testABThrustVsTemperature() {
    double baseThrust = 24000.0;
    double T_std = 288.15;  // K (ISA sea level)

    // Standard day
    double T = 288.15;
    double tempRatio = T / T_std;
    double thrust = baseThrust / sqrt(tempRatio);
    TS_ASSERT_DELTA(thrust, 24000.0, 1.0);

    // Hot day (T = 310 K)
    T = 310.0;
    tempRatio = T / T_std;
    thrust = baseThrust / sqrt(tempRatio);
    TS_ASSERT(thrust < 24000.0);  // Thrust decreases

    // Cold day (T = 260 K)
    T = 260.0;
    tempRatio = T / T_std;
    thrust = baseThrust / sqrt(tempRatio);
    TS_ASSERT(thrust > 24000.0);  // Thrust increases
  }

  // Test flameholder pressure loss
  void testFlameHolderPressureLoss() {
    // Flameholders cause pressure loss
    double velocity = 500.0;  // ft/sec
    double density = 0.002;   // slugs/ft^3
    double dragCoeff = 1.5;   // Flameholder drag coefficient

    // Dynamic pressure
    double q = 0.5 * density * velocity * velocity;
    TS_ASSERT_DELTA(q, 250.0, 0.1);

    // Pressure loss
    double deltaP = dragCoeff * q;
    TS_ASSERT_DELTA(deltaP, 375.0, 0.1);
  }

  // Test AB spray bar fuel distribution
  void testABSprayBarFuelDistribution() {
    // Fuel spray bars distribute fuel across AB cross-section
    int numOrifices = 12;
    double totalFuelFlow = 36000.0;  // lbs/hr

    double fuelFlowPerOrifice = totalFuelFlow / numOrifices;
    TS_ASSERT_DELTA(fuelFlowPerOrifice, 3000.0, 0.1);

    // Each orifice should have equal flow (ideally)
    TS_ASSERT(fuelFlowPerOrifice > 0.0);
    TS_ASSERT(fuelFlowPerOrifice * numOrifices == totalFuelFlow);
  }

  // Test AB zone fuel injection pattern
  void testABZoneFuelInjectionPattern() {
    // Multi-zone AB has fuel injected in stages
    int numZones = 3;
    bool zone1Active = false;
    bool zone2Active = false;
    bool zone3Active = false;
    double abCommand = 0.0;

    // Zone 1 only (Min AB)
    abCommand = 0.35;
    zone1Active = (abCommand > 0.0);
    zone2Active = (abCommand > 0.5);
    zone3Active = (abCommand > 0.8);
    TS_ASSERT(zone1Active);
    TS_ASSERT(!zone2Active);
    TS_ASSERT(!zone3Active);

    // Zones 1 and 2 (Intermediate AB)
    abCommand = 0.65;
    zone1Active = (abCommand > 0.0);
    zone2Active = (abCommand > 0.5);
    zone3Active = (abCommand > 0.8);
    TS_ASSERT(zone1Active);
    TS_ASSERT(zone2Active);
    TS_ASSERT(!zone3Active);

    // All zones (Max AB)
    abCommand = 1.0;
    zone1Active = (abCommand > 0.0);
    zone2Active = (abCommand > 0.5);
    zone3Active = (abCommand > 0.8);
    TS_ASSERT(zone1Active);
    TS_ASSERT(zone2Active);
    TS_ASSERT(zone3Active);
  }

  // Test AB mixer efficiency
  void testABMixerEfficiency() {
    // Mixing efficiency affects combustion completeness
    double mixingEfficiency = 0.88;  // 88% mixing efficiency

    TS_ASSERT(mixingEfficiency > 0.80);
    TS_ASSERT(mixingEfficiency < 0.95);

    // Poor mixing reduces combustion efficiency
    double combustionEff_goodMixing = 0.95;
    double combustionEff_poorMixing = combustionEff_goodMixing * mixingEfficiency;
    TS_ASSERT(combustionEff_poorMixing < combustionEff_goodMixing);
    TS_ASSERT_DELTA(combustionEff_poorMixing, 0.836, 0.01);
  }

  // Test AB thrust coefficient
  void testABThrustCoefficient() {
    // Thrust coefficient relates thrust to flow and pressure
    double massFlow = 250.0;  // lbs/sec
    double exitVelocity = 5000.0;  // ft/sec
    double g0 = 32.174;  // ft/sec^2

    // Thrust = massFlow * exitVelocity / g0
    double thrust = (massFlow * exitVelocity) / g0;
    TS_ASSERT_DELTA(thrust, 38851.2, 1.0);

    // With AB, exit velocity increases
    double exitVelocity_AB = 6500.0;
    double thrust_AB = (massFlow * exitVelocity_AB) / g0;
    TS_ASSERT(thrust_AB > thrust);
    TS_ASSERT_DELTA(thrust_AB, 50506.6, 1.0);
  }

  // Test AB igniter energy requirements
  void testABIgniterEnergyRequirements() {
    // AB requires continuous ignition energy
    double igniterPower = 5000.0;  // Watts
    double igniterVoltage = 28.0;  // Volts (aircraft electrical system)

    double igniterCurrent = igniterPower / igniterVoltage;
    TS_ASSERT_DELTA(igniterCurrent, 178.6, 0.1);

    // High current draw
    TS_ASSERT(igniterCurrent > 100.0);
  }

  // Test AB fuel atomization quality
  void testABFuelAtomizationQuality() {
    // Fuel droplet size affects combustion efficiency
    double meanDropletSize = 50.0;  // microns
    double maxDropletSize = 100.0;  // microns for good combustion

    bool goodAtomization = (meanDropletSize < maxDropletSize);
    TS_ASSERT(goodAtomization);

    // Poor atomization
    meanDropletSize = 150.0;
    goodAtomization = (meanDropletSize < maxDropletSize);
    TS_ASSERT(!goodAtomization);
  }

  // Test AB residence time
  void testABResidenceTime() {
    // Fuel must have sufficient residence time in AB for complete combustion
    double ABLength = 10.0;  // feet
    double gasVelocity = 800.0;  // ft/sec

    double residenceTime = ABLength / gasVelocity;  // seconds
    TS_ASSERT_DELTA(residenceTime, 0.0125, 0.0001);

    // Minimum residence time (typically > 5 milliseconds)
    double minResidenceTime = 0.005;  // seconds
    TS_ASSERT(residenceTime > minResidenceTime);
  }

  // Test AB exhaust plume visibility
  void testABExhaustPlumeVisibility() {
    // Afterburner produces visible exhaust plume
    bool abActive = false;
    bool visiblePlume = false;

    // Without AB
    abActive = false;
    visiblePlume = abActive;
    TS_ASSERT(!visiblePlume);

    // With AB - highly visible infrared signature
    abActive = true;
    visiblePlume = abActive;
    TS_ASSERT(visiblePlume);
  }

  // Test AB thermal signature
  void testABThermalSignature() {
    // AB dramatically increases infrared signature
    double EGT_dry = 750.0;  // °C
    double EGT_wet = 1050.0;  // °C

    // Stefan-Boltzmann: radiant intensity ~ T^4
    double T_dry_K = EGT_dry + 273.15;
    double T_wet_K = EGT_wet + 273.15;

    double radianceRatio = pow(T_wet_K / T_dry_K, 4.0);

    // AB increases IR signature by factor of ~2.5-3
    TS_ASSERT(radianceRatio > 2.5);
    TS_ASSERT(radianceRatio < 3.5);
    TS_ASSERT_DELTA(radianceRatio, 2.797, 0.01);
  }

  // Test AB acoustic signature
  void testABAcousticSignature() {
    // Afterburner significantly increases noise
    double noiseLevel_dry = 140.0;  // dB
    double noiseLevel_wet = 165.0;  // dB

    double noiseIncrease = noiseLevel_wet - noiseLevel_dry;
    TS_ASSERT_DELTA(noiseIncrease, 25.0, 0.1);

    // AB adds 20-30 dB to exhaust noise
    TS_ASSERT(noiseIncrease > 15.0);
    TS_ASSERT(noiseIncrease < 35.0);
  }

  // Test AB specific thrust calculation
  void testABSpecificThrust() {
    // Specific thrust = Thrust / (mass flow rate)
    double thrust = 24000.0;  // lbs
    double massFlow = 200.0;  // lbs/sec

    double specificThrust = thrust / massFlow;
    TS_ASSERT_DELTA(specificThrust, 120.0, 0.1);  // lbs/(lbs/sec) = sec

    // Higher with AB due to higher exhaust velocity
    double thrust_dry = 15000.0;
    double specificThrust_dry = thrust_dry / massFlow;
    TS_ASSERT(specificThrust > specificThrust_dry);
  }

  // Test nozzle pressure ratio
  void testNozzlePressureRatio() {
    double P0_nozzle = 50.0;  // psia
    double P_ambient = 14.7;  // psia

    double NPR = P0_nozzle / P_ambient;
    TS_ASSERT_DELTA(NPR, 3.40, 0.01);

    // Choked flow occurs at NPR > ~1.89
    bool chokedFlow = (NPR > 1.89);
    TS_ASSERT(chokedFlow);
  }

  // Test convergent-divergent nozzle expansion
  void testCDNozzleExpansion() {
    // Area ratio for supersonic expansion
    double exitMach = 2.0;
    double gamma = 1.35;

    // Simplified area ratio calculation
    double term1 = 1.0 + (gamma - 1.0) / 2.0 * exitMach * exitMach;
    double term2 = (gamma + 1.0) / 2.0;
    double areaRatio = pow(term1 / term2, (gamma + 1.0) / (2.0 * (gamma - 1.0))) / exitMach;

    TS_ASSERT(areaRatio > 1.0);  // Exit area > throat area
  }

  // Test AB fuel-air ratio limits
  void testABFuelAirRatioLimits() {
    double FAR_min = 0.008;   // Lean limit for stable combustion
    double FAR_max = 0.035;   // Rich limit before efficiency drop
    double FAR_operating = 0.020;

    bool withinLimits = (FAR_operating >= FAR_min) && (FAR_operating <= FAR_max);
    TS_ASSERT(withinLimits);
  }

  // Test AB combustion stability margin
  void testABCombustionStabilityMargin() {
    double velocityRatio = 1.5;  // Flame speed / flow velocity
    double stabilityMargin = velocityRatio - 1.0;

    // Positive margin means stable combustion
    TS_ASSERT(stabilityMargin > 0.0);
    TS_ASSERT_DELTA(stabilityMargin, 0.5, 0.01);
  }

  // Test AB liner cooling effectiveness
  void testABLinerCoolingEffectiveness() {
    double T_gas = 1800.0;     // K (gas temperature)
    double T_coolant = 600.0;  // K (cooling air temperature)
    double T_wall = 1000.0;    // K (wall temperature)

    double coolingEffectiveness = (T_gas - T_wall) / (T_gas - T_coolant);
    TS_ASSERT(coolingEffectiveness > 0.5);
    TS_ASSERT_DELTA(coolingEffectiveness, 0.667, 0.01);
  }

  // Test AB exit area control
  void testABExitAreaControl() {
    // Variable area nozzle position based on AB command
    double abCommand = 0.0;
    double A_min = 1.0;
    double A_max = 1.8;

    double A_exit = A_min + abCommand * (A_max - A_min);
    TS_ASSERT_DELTA(A_exit, 1.0, 0.01);  // Dry

    abCommand = 1.0;
    A_exit = A_min + abCommand * (A_max - A_min);
    TS_ASSERT_DELTA(A_exit, 1.8, 0.01);  // Full AB
  }

  // Test exhaust velocity with AB
  void testExhaustVelocityWithAB() {
    double Tt = 1500.0;  // K (total temperature)
    double gamma = 1.3;
    double R = 287.0;    // J/kg-K
    double NPR = 4.0;

    // Ideal exhaust velocity
    double V_exit = sqrt(2.0 * gamma / (gamma - 1.0) * R * Tt *
                        (1.0 - pow(1.0 / NPR, (gamma - 1.0) / gamma)));

    TS_ASSERT(V_exit > 500.0);  // m/s
    TS_ASSERT(V_exit < 2000.0);
  }

  // Test nozzle thrust coefficient
  void testNozzleThrustCoefficient() {
    double Cf_ideal = 1.8;
    double Cf_loss = 0.02;

    double Cf_actual = Cf_ideal * (1.0 - Cf_loss);
    TS_ASSERT_DELTA(Cf_actual, 1.764, 0.01);
  }

  // Test AB cycle time from dry to wet
  void testABCycleTimeDryToWet() {
    double dryToWetTime = 1.5;  // seconds
    double wetToDryTime = 0.8;  // seconds

    // Wet-to-dry faster than dry-to-wet
    TS_ASSERT(wetToDryTime < dryToWetTime);
  }

  // Test AB fuel control valve response
  void testABFuelControlValveResponse() {
    double valveTimeConstant = 0.1;  // seconds
    double stepResponse_1tau = 1.0 - std::exp(-1.0);

    TS_ASSERT_DELTA(stepResponse_1tau, 0.632, 0.01);
  }

  // Test AB temperature limiting
  void testABTemperatureLimiting() {
    double T_max = 1900.0;  // K (material limit)
    double T_current = 1850.0;

    bool limitActive = T_current > 0.95 * T_max;
    TS_ASSERT(limitActive);
  }

  // Test partial AB thrust
  void testPartialABThrust() {
    double milThrust = 15000.0;
    double maxThrust = 24000.0;
    double abCommands[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    double expectedThrusts[] = {15000.0, 17250.0, 19500.0, 21750.0, 24000.0};

    for (int i = 0; i < 5; i++) {
      double thrust = milThrust + abCommands[i] * (maxThrust - milThrust);
      TS_ASSERT_DELTA(thrust, expectedThrusts[i], 0.1);
    }
  }

  // Test AB mass flow augmentation
  void testABMassFlowAugmentation() {
    double massFlow_dry = 200.0;  // lbs/sec
    double fuelFlow_AB = 30000.0 / 3600.0;  // lbs/sec (from lbs/hr)

    double massFlow_wet = massFlow_dry + fuelFlow_AB;
    TS_ASSERT(massFlow_wet > massFlow_dry);
  }

  // Test AB inlet compatibility
  void testABInletCompatibility() {
    // Inlet capture ratio at supersonic speeds
    double A_capture = 5.0;  // ft^2
    double A_inlet = 4.5;    // ft^2

    double captureRatio = A_inlet / A_capture;
    TS_ASSERT(captureRatio < 1.0);  // Some spillage
  }

  // Test AB minimum fuel flow
  void testABMinimumFuelFlow() {
    double minFuelFlow = 5000.0;  // lbs/hr for stable combustion
    double currentFuelFlow = 8000.0;

    bool stableCombustion = currentFuelFlow >= minFuelFlow;
    TS_ASSERT(stableCombustion);
  }

  // Test AB screech margin
  void testABScreechMargin() {
    double frequency = 500.0;  // Hz (screech frequency)
    double operatingBand_low = 200.0;
    double operatingBand_high = 400.0;

    bool outOfBand = (frequency < operatingBand_low) || (frequency > operatingBand_high);
    TS_ASSERT(outOfBand);  // Should avoid screech frequency
  }

  // Test nozzle area scheduling with Mach
  void testNozzleAreaSchedulingWithMach() {
    double mach = 0.0;
    double A_base = 1.0;

    // Area increases with Mach for constant pressure operation
    mach = 1.5;
    double A_supersonic = A_base * (1.0 + 0.15 * mach);
    TS_ASSERT(A_supersonic > A_base);
  }

  // Test AB bleed air reduction
  void testABBleedAirReduction() {
    // Some aircraft reduce bleed air during AB operation
    double bleed_normal = 5.0;  // % of core flow
    double bleed_AB = 2.0;      // Reduced during AB

    TS_ASSERT(bleed_AB < bleed_normal);
  }

  // Test nozzle divergence losses
  void testNozzleDivergenceLosses() {
    double halfAngle = 15.0 * M_PI / 180.0;  // radians

    // Divergence loss factor
    double divergenceFactor = (1.0 + std::cos(halfAngle)) / 2.0;
    TS_ASSERT(divergenceFactor < 1.0);
    TS_ASSERT(divergenceFactor > 0.95);
  }

  // Test AB thrust-to-weight ratio impact
  void testABThrustToWeightImpact() {
    double weight = 30000.0;  // lbs
    double thrust_dry = 15000.0;
    double thrust_wet = 24000.0;

    double TW_dry = thrust_dry / weight;
    double TW_wet = thrust_wet / weight;

    TS_ASSERT_DELTA(TW_dry, 0.5, 0.01);
    TS_ASSERT_DELTA(TW_wet, 0.8, 0.01);
    TS_ASSERT(TW_wet > TW_dry);
  }

  // Test AB acceleration capability
  void testABAccelerationCapability() {
    double mass = 30000.0 / 32.174;  // slugs
    double thrust_dry = 15000.0;
    double thrust_wet = 24000.0;
    double drag = 5000.0;

    double accel_dry = (thrust_dry - drag) / mass;
    double accel_wet = (thrust_wet - drag) / mass;

    TS_ASSERT(accel_wet > accel_dry);
    TS_ASSERT_DELTA(accel_wet / accel_dry, 1.9, 0.1);
  }

  // Test AB climb rate improvement
  void testABClimbRateImprovement() {
    double weight = 30000.0;
    double excessThrust_dry = 10000.0;
    double excessThrust_wet = 19000.0;

    double ROC_dry = excessThrust_dry / weight * 60.0 * 101.3;  // fpm
    double ROC_wet = excessThrust_wet / weight * 60.0 * 101.3;

    TS_ASSERT(ROC_wet > ROC_dry);
  }

  // Test AB service ceiling impact
  void testABServiceCeilingImpact() {
    // AB extends service ceiling
    double ceiling_dry = 50000.0;  // ft
    double ceiling_wet = 60000.0;  // ft with AB

    double ceilingIncrease = ceiling_wet - ceiling_dry;
    TS_ASSERT(ceilingIncrease > 5000.0);
  }

  // Test AB turn rate improvement
  void testABTurnRateImprovement() {
    double loadFactor = 4.0;  // g's
    double velocity = 500.0;  // fps
    double g0 = 32.174;

    // Turn rate = g * n / V
    double turnRate = g0 * loadFactor / velocity;  // rad/s
    double turnRate_degps = turnRate * 180.0 / M_PI;

    TS_ASSERT(turnRate_degps > 10.0);
  }

  // Test AB range penalty
  void testABRangePenalty() {
    double SFC_dry = 0.85;
    double SFC_wet = 2.0;
    double SFC_ratio = SFC_wet / SFC_dry;

    // Range roughly inversely proportional to SFC
    double rangeFactor = 1.0 / SFC_ratio;
    TS_ASSERT(rangeFactor < 0.5);  // Significant range reduction
  }

  // Test AB usage in combat
  void testABUsageInCombat() {
    // Typical combat AB usage time
    double combatDuration = 300.0;  // seconds
    double abUsagePercent = 0.30;   // 30% of combat time

    double abTime = combatDuration * abUsagePercent;
    TS_ASSERT_DELTA(abTime, 90.0, 0.1);  // 90 seconds of AB
  }

  // Test AB for takeoff
  void testABForTakeoff() {
    double thrust_dry = 15000.0;
    double thrust_wet = 24000.0;
    double weight = 45000.0;

    // Takeoff thrust-to-weight
    double TW_wet = thrust_wet / weight;
    TS_ASSERT(TW_wet > 0.5);  // Adequate for takeoff
  }

  // Test minimum AB engagement speed
  void testMinABEngagementSpeed() {
    double minSpeed = 200.0;  // knots
    double currentSpeed = 250.0;

    bool canEngage = currentSpeed >= minSpeed;
    TS_ASSERT(canEngage);
  }

  /***************************************************************************
   * Extended Afterburner Tests (77-100)
   ***************************************************************************/

  // Test 77: AB fuel manifold pressure
  void testABFuelManifoldPressure() {
    double pumpPressure = 800.0;  // psi
    double lineDrop = 50.0;       // psi
    double manifoldPressure = pumpPressure - lineDrop;

    TS_ASSERT_DELTA(manifoldPressure, 750.0, 0.1);
    TS_ASSERT(manifoldPressure > 500.0);  // Minimum for atomization
  }

  // Test 78: AB nozzle exit velocity ratio
  void testNozzleExitVelocityRatio() {
    double V_dry = 1500.0;   // ft/s
    double V_wet = 2200.0;   // ft/s

    double velRatio = V_wet / V_dry;
    TS_ASSERT_DELTA(velRatio, 1.467, 0.01);
    TS_ASSERT(velRatio > 1.3);
  }

  // Test 79: AB combustion chamber length
  void testCombustionChamberLength() {
    double residenceTime = 0.01;  // seconds
    double gasVelocity = 800.0;   // ft/s

    double length = residenceTime * gasVelocity;
    TS_ASSERT_DELTA(length, 8.0, 0.1);  // feet
  }

  // Test 80: AB flame stability parameter
  void testFlameStabilityParameter() {
    double velocity = 600.0;      // ft/s
    double flameSpeed = 100.0;    // ft/s
    double stabRatio = velocity / flameSpeed;

    TS_ASSERT(stabRatio < 10.0);  // Flame stable if ratio < 10
  }

  // Test 81: AB plume length estimation
  void testPlumeLengthEstimation() {
    double exitDiameter = 2.0;    // ft
    double exitMach = 1.5;

    double plumeLength = exitDiameter * (3.0 + 2.0 * exitMach);
    TS_ASSERT_DELTA(plumeLength, 12.0, 0.1);
  }

  // Test 82: AB fuel pump power requirement
  void testFuelPumpPower() {
    double fuelFlow = 40000.0 / 3600.0;  // lbs/s
    double pressureRise = 800.0;          // psi
    double efficiency = 0.75;

    double power = (fuelFlow * pressureRise * 144.0) / (550.0 * efficiency);
    TS_ASSERT(power > 0);
    TS_ASSERT(power < 5000);  // Reasonable HP range for high-flow pump
  }

  // Test 83: AB nozzle divergence angle
  void testNozzleDivergenceAngle() {
    double exitRadius = 1.5;   // ft
    double throatRadius = 1.0; // ft
    double length = 2.0;       // ft

    double angle = std::atan((exitRadius - throatRadius) / length) * 180.0 / M_PI;
    TS_ASSERT_DELTA(angle, 14.04, 0.1);  // degrees
  }

  // Test 84: AB cooling air requirement
  void testCoolingAirRequirement() {
    double T_gas = 2000.0;     // K
    double T_wall_max = 1200.0; // K
    double T_cooling = 700.0;   // K

    double coolingEffectiveness = (T_gas - T_wall_max) / (T_gas - T_cooling);
    TS_ASSERT_DELTA(coolingEffectiveness, 0.615, 0.01);
  }

  // Test 85: AB fuel distribution uniformity
  void testFuelDistributionUniformity() {
    double flows[] = {3000.0, 3100.0, 2900.0, 3050.0, 2950.0, 3000.0};
    double sum = 0, sumSq = 0;
    int n = 6;

    for (int i = 0; i < n; i++) sum += flows[i];
    double mean = sum / n;

    for (int i = 0; i < n; i++) sumSq += (flows[i] - mean) * (flows[i] - mean);
    double stdDev = std::sqrt(sumSq / n);
    double cv = stdDev / mean * 100.0;  // Coefficient of variation

    TS_ASSERT(cv < 5.0);  // Less than 5% variation
  }

  // Test 86: AB bypass duct mixing
  void testBypassDuctMixing() {
    double T_core = 1000.0;   // K
    double T_bypass = 350.0;  // K
    double BPR = 0.5;         // Low bypass ratio

    double T_mixed = (T_core + BPR * T_bypass) / (1.0 + BPR);
    TS_ASSERT_DELTA(T_mixed, 783.3, 1.0);
  }

  // Test 87: AB thrust coefficient variation
  void testThrustCoefficientVariation() {
    double Cf_ideal = 1.8;
    double NPR = 4.0;
    double gamma = 1.3;

    double Cf_actual = Cf_ideal * (1.0 - 0.02 * (NPR - 3.0));
    TS_ASSERT(Cf_actual > 1.7);
    TS_ASSERT(Cf_actual < 1.9);
  }

  // Test 88: AB igniter spark rate
  void testIgniterSparkRate() {
    double sparkRate = 4.0;  // sparks per second
    double minRate = 2.0;

    TS_ASSERT(sparkRate >= minRate);
    TS_ASSERT(sparkRate <= 10.0);
  }

  // Test 89: AB fuel vaporization rate
  void testFuelVaporizationRate() {
    double dropletDiameter = 50.0e-6;  // m
    double T_gas = 1500.0;             // K
    double vaporizationTime = dropletDiameter * dropletDiameter / (0.001 * T_gas);

    TS_ASSERT(vaporizationTime < 0.01);  // Complete in < 10ms
  }

  // Test 90: AB nozzle area margin
  void testNozzleAreaMargin() {
    double A_required = 1.5;  // sq ft
    double A_max = 1.8;       // sq ft
    double margin = (A_max - A_required) / A_required * 100.0;

    TS_ASSERT_DELTA(margin, 20.0, 0.1);
    TS_ASSERT(margin >= 15.0);  // At least 15% margin
  }

  // Test 91: AB altitude thrust ratio
  void testAltitudeThrustRatio() {
    double thrust_SL = 24000.0;
    double thrust_36k = thrust_SL * 0.30;  // ~30% at 36,000 ft

    double ratio = thrust_36k / thrust_SL;
    TS_ASSERT_DELTA(ratio, 0.30, 0.01);
  }

  // Test 92: AB supersonic operation
  void testSupersonicOperation() {
    double mach = 1.8;
    double ramRatio = 1.0 + 0.2 * mach * mach;

    TS_ASSERT_DELTA(ramRatio, 1.648, 0.01);
    TS_ASSERT(ramRatio > 1.5);
  }

  // Test 93: AB transient response time
  void testTransientResponseTime() {
    double timeConstant = 0.3;  // seconds
    double settlingTime = 4.0 * timeConstant;

    TS_ASSERT_DELTA(settlingTime, 1.2, 0.01);
    TS_ASSERT(settlingTime < 2.0);
  }

  // Test 94: AB fuel control accuracy
  void testFuelControlAccuracy() {
    double commanded = 35000.0;  // lbs/hr
    double actual = 35350.0;     // lbs/hr
    double error = std::abs(actual - commanded) / commanded * 100.0;

    TS_ASSERT(error < 2.0);  // Within 2%
  }

  // Test 95: AB thermal expansion
  void testThermalExpansion() {
    double length_cold = 10.0;  // ft
    double deltaT = 1500.0;     // K
    double alpha = 12e-6;       // per K

    double expansion = length_cold * alpha * deltaT * 12.0;  // inches
    TS_ASSERT_DELTA(expansion, 2.16, 0.1);
  }

  // Test 96: AB material creep limit
  void testMaterialCreepLimit() {
    double T_operating = 1100.0;  // K
    double T_creep_limit = 1200.0; // K

    double margin = (T_creep_limit - T_operating) / T_creep_limit * 100.0;
    TS_ASSERT(margin > 5.0);  // At least 5% margin
  }

  // Test 97: AB acoustic instability frequency
  void testAcousticInstabilityFrequency() {
    double length = 8.0;      // ft
    double soundSpeed = 2000.0;  // ft/s in hot gas

    double freq = soundSpeed / (2.0 * length);  // First mode
    TS_ASSERT_DELTA(freq, 125.0, 1.0);  // Hz
  }

  // Test 98: AB fuel-air equivalence ratio
  void testEquivalenceRatio() {
    double FAR_actual = 0.020;
    double FAR_stoich = 0.068;
    double phi = FAR_actual / FAR_stoich;

    TS_ASSERT_DELTA(phi, 0.294, 0.01);
    TS_ASSERT(phi < 1.0);  // Lean operation
  }

  // Test 99: AB exit plane pressure ratio
  void testExitPlanePressureRatio() {
    double P_exit = 30.0;     // psia
    double P_ambient = 14.7;  // psia

    double ratio = P_exit / P_ambient;
    TS_ASSERT_DELTA(ratio, 2.04, 0.01);
  }

  // Test 100: Complete AB system state
  void testCompleteABSystemState() {
    // Define AB operating point
    double N2 = 98.5;           // percent
    double throttle = 1.0;
    double mach = 0.9;
    double altitude = 20000.0;  // ft

    // AB light-off conditions
    bool canLightOff = (N2 > 97.0) && (throttle > 0.99);
    TS_ASSERT(canLightOff);

    // Thrust calculation
    double milThrust = 15000.0;
    double maxThrust = 24000.0;
    double sigma = std::exp(-altitude / 27000.0);
    double ramEffect = 1.0 + 0.1 * mach;

    double thrust_dry = milThrust * sigma * ramEffect;
    double thrust_wet = maxThrust * sigma * ramEffect;

    TS_ASSERT(thrust_wet > thrust_dry);
    TS_ASSERT(thrust_wet / thrust_dry > 1.5);

    // Fuel flow
    double TSFC_dry = 0.85;
    double ATSFC_wet = 2.0;
    double fuelFlow_dry = thrust_dry * TSFC_dry;
    double fuelFlow_wet = thrust_wet * ATSFC_wet;

    TS_ASSERT(fuelFlow_wet > 3.0 * fuelFlow_dry);

    // Nozzle area
    double A_dry = 1.0;
    double A_wet = 1.7;
    TS_ASSERT(A_wet > A_dry);
  }
};
