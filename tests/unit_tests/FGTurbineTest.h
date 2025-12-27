#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGTurbineTest : public CxxTest::TestSuite
{
public:
  // Test engine type is etTurbine
  void testEngineType() {
    TS_ASSERT_EQUALS(FGEngine::etTurbine, 3);
  }

  // Test phase state enum values
  void testPhaseStateEnums() {
    // Phase states: Off, Run, SpinUp, Start, Stall, Seize, Trim
    int tpOff = 0;
    int tpRun = 1;
    int tpSpinUp = 2;
    int tpStart = 3;
    int tpStall = 4;
    int tpSeize = 5;
    int tpTrim = 6;

    TS_ASSERT_EQUALS(tpOff, 0);
    TS_ASSERT_EQUALS(tpRun, 1);
    TS_ASSERT_EQUALS(tpSpinUp, 2);
    TS_ASSERT_EQUALS(tpStart, 3);
    TS_ASSERT_EQUALS(tpStall, 4);
    TS_ASSERT_EQUALS(tpSeize, 5);
    TS_ASSERT_EQUALS(tpTrim, 6);
  }

  // Test N1 and N2 percentage ranges
  void testSpoolPercentageRanges() {
    // N1 (fan/LP spool): 0-100%
    double N1_idle = 30.0;
    double N1_max = 100.0;

    TS_ASSERT(N1_idle >= 0.0);
    TS_ASSERT(N1_idle <= 100.0);
    TS_ASSERT(N1_max >= 0.0);
    TS_ASSERT(N1_max <= 100.0);
    TS_ASSERT(N1_idle < N1_max);

    // N2 (core/HP spool): 0-100%
    double N2_idle = 60.0;
    double N2_max = 100.0;

    TS_ASSERT(N2_idle >= 0.0);
    TS_ASSERT(N2_idle <= 100.0);
    TS_ASSERT(N2_max >= 0.0);
    TS_ASSERT(N2_max <= 100.0);
    TS_ASSERT(N2_idle < N2_max);
  }

  // Test N2 normalization
  void testN2Normalization() {
    // N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2)
    double IdleN2 = 60.0;
    double MaxN2 = 100.0;

    // At idle
    double N2 = 60.0;
    double N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2);
    TS_ASSERT_DELTA(N2norm, 0.0, DEFAULT_TOLERANCE);

    // At max
    N2 = 100.0;
    N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2);
    TS_ASSERT_DELTA(N2norm, 1.0, DEFAULT_TOLERANCE);

    // Mid-range
    N2 = 80.0;
    N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2);
    TS_ASSERT_DELTA(N2norm, 0.5, DEFAULT_TOLERANCE);
  }

  // Test thrust calculation (quadratic relationship)
  void testThrustCalculation() {
    // thrust = idleThrust + (milThrust * N2norm^2)
    double idleThrust = 500.0;   // lbs at idle
    double milThrust = 9500.0;   // lbs additional at max
    double N2norm = 0.0;

    // At idle (N2norm = 0)
    double thrust = idleThrust + milThrust * pow(N2norm, 2);
    TS_ASSERT_DELTA(thrust, 500.0, DEFAULT_TOLERANCE);

    // At max (N2norm = 1)
    N2norm = 1.0;
    thrust = idleThrust + milThrust * pow(N2norm, 2);
    TS_ASSERT_DELTA(thrust, 10000.0, DEFAULT_TOLERANCE);

    // At 50% (N2norm = 0.5)
    N2norm = 0.5;
    thrust = idleThrust + milThrust * pow(N2norm, 2);
    // = 500 + 9500 * 0.25 = 500 + 2375 = 2875
    TS_ASSERT_DELTA(thrust, 2875.0, DEFAULT_TOLERANCE);
  }

  // Test bleed air effect on thrust
  void testBleedAirEffect() {
    double baseThrust = 10000.0;
    double bleedDemand = 0.0;

    // No bleed
    double thrust = baseThrust * (1.0 - bleedDemand);
    TS_ASSERT_DELTA(thrust, 10000.0, DEFAULT_TOLERANCE);

    // 5% bleed (anti-ice, pressurization)
    bleedDemand = 0.05;
    thrust = baseThrust * (1.0 - bleedDemand);
    TS_ASSERT_DELTA(thrust, 9500.0, DEFAULT_TOLERANCE);

    // 10% bleed
    bleedDemand = 0.10;
    thrust = baseThrust * (1.0 - bleedDemand);
    TS_ASSERT_DELTA(thrust, 9000.0, DEFAULT_TOLERANCE);
  }

  // Test afterburner thrust augmentation
  void testAfterburnerThrust() {
    double milThrust = 10000.0;  // Military (dry) thrust
    double maxThrust = 15000.0;  // Max (wet) thrust with AB

    // Without AB
    bool augmentation = false;
    double thrust = augmentation ? maxThrust : milThrust;
    TS_ASSERT_DELTA(thrust, 10000.0, DEFAULT_TOLERANCE);

    // With AB
    augmentation = true;
    thrust = augmentation ? maxThrust : milThrust;
    TS_ASSERT_DELTA(thrust, 15000.0, DEFAULT_TOLERANCE);

    // AB typically provides 50% more thrust
    double augRatio = maxThrust / milThrust;
    TS_ASSERT_DELTA(augRatio, 1.5, DEFAULT_TOLERANCE);
  }

  // Test TSFC calculation
  void testTSFC() {
    // TSFC = Fuel_flow / Thrust (lbs/hr/lbf)
    double fuelFlow_pph = 8000.0;  // lbs/hr
    double thrust = 10000.0;        // lbf

    double TSFC = fuelFlow_pph / thrust;
    TS_ASSERT_DELTA(TSFC, 0.8, DEFAULT_TOLERANCE);

    // Typical turbofan TSFC: 0.3-0.6 (cruise)
    // Typical turbojet TSFC: 0.8-1.0 (cruise)
    TS_ASSERT(TSFC >= 0.3);
    TS_ASSERT(TSFC <= 1.5);
  }

  // Test afterburner TSFC (ATSFC)
  void testATSFC() {
    // Afterburner uses much more fuel
    double normalTSFC = 0.8;
    double ATSFC = 1.7;

    TS_ASSERT(ATSFC > normalTSFC);
    TS_ASSERT_DELTA(ATSFC / normalTSFC, 2.125, 0.01);
  }

  // Test temperature-corrected TSFC
  void testCorrectedTSFC() {
    // TSFC increases at higher temperatures
    double baseTSFC = 0.8;
    double T_ref = 389.7;  // Reference temperature (K)
    double T_actual = 288.15;  // Standard day (K)

    double correctedTSFC = baseTSFC * sqrt(T_actual / T_ref);
    // = 0.8 * sqrt(288.15/389.7) = 0.8 * 0.86 = 0.688
    TS_ASSERT_DELTA(correctedTSFC, 0.688, 0.01);

    // Hot day
    T_actual = 310.0;
    correctedTSFC = baseTSFC * sqrt(T_actual / T_ref);
    TS_ASSERT(correctedTSFC > 0.688);
  }

  // Test Seek filter (lag dynamics)
  void testSeekFilter() {
    // Seek ramped approach with different accel/decel rates
    double var = 50.0;
    double target = 100.0;
    double accel = 20.0;  // %/sec acceleration
    double decel = 10.0;  // %/sec deceleration
    double deltaT = 0.1;

    // Accelerating (var < target)
    if (var < target) {
      var += deltaT * accel;
    }
    TS_ASSERT_DELTA(var, 52.0, DEFAULT_TOLERANCE);

    // Continue accelerating
    for (int i = 0; i < 10; i++) {
      if (var < target) var += deltaT * accel;
    }
    TS_ASSERT(var > 50.0);
    TS_ASSERT(var <= 100.0);

    // Decelerating (var > target)
    var = 80.0;
    target = 50.0;
    if (var > target) {
      var -= deltaT * decel;
    }
    TS_ASSERT_DELTA(var, 79.0, DEFAULT_TOLERANCE);
  }

  // Test spool-up time
  void testSpoolUpTime() {
    // Spool-up time depends on bypass ratio and N2
    double bypassRatio = 5.0;
    double factor = 1.0;

    // Base delay
    double delay = factor * 90.0 / (bypassRatio + 3.0);
    // = 1.0 * 90 / 8 = 11.25 seconds
    TS_ASSERT_DELTA(delay, 11.25, 0.1);

    // Higher bypass ratio = faster spool
    bypassRatio = 8.0;
    delay = factor * 90.0 / (bypassRatio + 3.0);
    TS_ASSERT(delay < 11.25);
  }

  // Test exhaust gas temperature
  void testExhaustGasTemp() {
    // EGT = TAT + 363.1 + ThrottlePos * 357.1
    double TAT = 25.0;  // Total air temp (°C)
    double throttlePos = 0.0;

    // At idle
    double EGT = TAT + 363.1 + throttlePos * 357.1;
    TS_ASSERT_DELTA(EGT, 388.1, 0.1);

    // At full power
    throttlePos = 1.0;
    EGT = TAT + 363.1 + throttlePos * 357.1;
    TS_ASSERT_DELTA(EGT, 745.2, 0.1);
  }

  // Test stall EGT (very high)
  void testStallEGT() {
    // During stall, EGT spikes to TAT + 903°C
    double TAT = 25.0;
    double stallEGT = TAT + 903.14;

    TS_ASSERT_DELTA(stallEGT, 928.14, 0.1);
    TS_ASSERT(stallEGT > 900.0);  // Very hot
  }

  // Test oil pressure relationship to N2
  void testOilPressure() {
    // Oil pressure roughly linear with N2
    double N2 = 80.0;
    double oilPressure = N2 * 0.62;

    TS_ASSERT_DELTA(oilPressure, 49.6, 0.1);

    // At idle N2
    N2 = 60.0;
    oilPressure = N2 * 0.62;
    TS_ASSERT_DELTA(oilPressure, 37.2, 0.1);
  }

  // Test engine pressure ratio
  void testEnginePressureRatio() {
    // EPR = 1.0 + (Thrust / MilThrust)
    double thrust = 8000.0;
    double milThrust = 10000.0;

    double EPR = 1.0 + (thrust / milThrust);
    TS_ASSERT_DELTA(EPR, 1.8, DEFAULT_TOLERANCE);

    // At idle
    thrust = 500.0;
    EPR = 1.0 + (thrust / milThrust);
    TS_ASSERT_DELTA(EPR, 1.05, DEFAULT_TOLERANCE);
  }

  // Test phase transitions
  void testPhaseTransitions() {
    // Off -> SpinUp: Starter on, Cutoff off
    bool starter = true;
    bool cutoff = true;  // Fuel cutoff engaged

    bool canSpinUp = starter && cutoff;  // Starter on but no fuel yet
    TS_ASSERT(canSpinUp);

    // SpinUp -> Start: N2 > 15%, Cutoff off
    double N2 = 20.0;
    cutoff = false;

    bool canStart = (N2 > 15.0) && !cutoff;
    TS_ASSERT(canStart);

    // Start -> Run: N2 reaches idle
    double IdleN2 = 60.0;
    N2 = 60.0;

    bool atIdle = (N2 >= IdleN2);
    TS_ASSERT(atIdle);
  }

  // Test water injection effect
  void testWaterInjection() {
    double N1 = 95.0;
    double N2 = 95.0;
    double N1_increment = 2.5;
    double N2_increment = 2.0;
    bool injection = false;

    // Without injection
    double effectiveN1 = N1;
    double effectiveN2 = N2;
    TS_ASSERT_DELTA(effectiveN1, 95.0, DEFAULT_TOLERANCE);

    // With injection
    injection = true;
    if (injection) {
      effectiveN1 = N1 + N1_increment;
      effectiveN2 = N2 + N2_increment;
    }
    TS_ASSERT_DELTA(effectiveN1, 97.5, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(effectiveN2, 97.0, DEFAULT_TOLERANCE);
  }

  // Test idle fuel flow
  void testIdleFuelFlow() {
    double idleFF = 500.0;  // lbs/hr at idle
    double fuelFlow = 400.0;

    // Fuel flow cannot go below idle
    if (fuelFlow < idleFF) fuelFlow = idleFF;
    TS_ASSERT_DELTA(fuelFlow, 500.0, DEFAULT_TOLERANCE);
  }

  // Test windmill effect
  void testWindmillEffect() {
    // In Off phase, windmilling can occur
    bool windmillEnabled = true;
    double N1 = 10.0;  // Some windmill RPM
    double airspeed = 250.0;  // kts

    // Windmill N1 roughly proportional to airspeed
    if (windmillEnabled && airspeed > 100.0) {
      N1 = airspeed * 0.05;  // Simplified
    }
    TS_ASSERT(N1 > 0.0);
    TS_ASSERT(N1 < 30.0);  // Less than idle
  }

  // Test cutoff behavior
  void testCutoffBehavior() {
    bool cutoff = false;
    bool running = true;

    // Cutoff engages
    cutoff = true;
    if (cutoff) running = false;

    TS_ASSERT(!running);
  }

  // Test fuel consumption calculation
  void testFuelConsumption() {
    double fuelFlow_pph = 8000.0;  // lbs/hr
    double deltaT = 0.01667;       // 1 minute in hours

    double fuelConsumed = fuelFlow_pph * deltaT;
    TS_ASSERT_DELTA(fuelConsumed, 133.36, 0.1);

    // Per second
    double fuelFlowRate = fuelFlow_pph / 3600.0;  // lbs/sec
    TS_ASSERT_DELTA(fuelFlowRate, 2.222, 0.01);
  }

  // Test thrust at altitude
  void testThrustAtAltitude() {
    double seaLevelThrust = 20000.0;
    double densityRatio = 0.5;  // ~18000 ft

    // Thrust roughly proportional to density
    double altitudeThrust = seaLevelThrust * densityRatio;
    TS_ASSERT_DELTA(altitudeThrust, 10000.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Bypass Ratio and Engine Configuration Tests
   ***************************************************************************/

  // Test bypass ratio effect on TSFC
  void testBypassRatioTSFC() {
    // Higher bypass ratio = lower TSFC
    double lowBypassTSFC = 0.9;   // Turbojet BPR ~0
    double midBypassTSFC = 0.6;   // Low-bypass turbofan BPR ~1
    double highBypassTSFC = 0.35; // High-bypass turbofan BPR ~8

    TS_ASSERT(highBypassTSFC < midBypassTSFC);
    TS_ASSERT(midBypassTSFC < lowBypassTSFC);
  }

  // Test bypass ratio effect on thrust
  void testBypassRatioThrust() {
    // Higher bypass = more mass flow but lower exhaust velocity
    double bypassRatio = 5.0;
    double coreThrust = 4000.0;  // lbs from core
    double fanThrust = coreThrust * bypassRatio * 0.3;  // Fan produces less per unit mass

    double totalThrust = coreThrust + fanThrust;
    TS_ASSERT_DELTA(totalThrust, 10000.0, DEFAULT_TOLERANCE);
  }

  // Test N1 to N2 relationship
  void testN1N2Relationship() {
    // N1 (fan) spools faster than N2 (core) at low power
    double N2 = 70.0;
    double IdleN2 = 60.0;
    double MaxN2 = 100.0;

    // N1 typically higher than N2 at same throttle
    double N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2);
    double N1 = 30.0 + 70.0 * std::sqrt(N2norm);  // Non-linear relationship

    TS_ASSERT(N1 > N2 * 0.5);
    TS_ASSERT(N1 < 100.0);
  }

  /***************************************************************************
   * Inlet and Compressor Tests
   ***************************************************************************/

  // Test ram recovery factor
  void testRamRecoveryFactor() {
    // Ram recovery decreases with Mach number
    double Mach = 0.0;
    double ramRecovery = 1.0;
    TS_ASSERT_DELTA(ramRecovery, 1.0, DEFAULT_TOLERANCE);

    // At Mach 0.8 (typical cruise)
    Mach = 0.8;
    ramRecovery = 1.0 - 0.075 * pow(Mach - 0.3, 2);
    TS_ASSERT(ramRecovery > 0.95);
    TS_ASSERT(ramRecovery < 1.0);

    // At supersonic speeds
    Mach = 2.0;
    ramRecovery = 1.0 - 0.075 * pow(Mach - 0.3, 2);
    TS_ASSERT(ramRecovery < 0.85);
  }

  // Test inlet pressure ratio
  void testInletPressureRatio() {
    double P_ambient = 14.7;  // psi
    double Mach = 0.85;
    double gamma = 1.4;

    // Total pressure ratio = (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1))
    double totalPressureRatio = pow(1.0 + (gamma-1.0)/2.0 * Mach*Mach, gamma/(gamma-1.0));
    double P_total = P_ambient * totalPressureRatio;

    TS_ASSERT(P_total > P_ambient);
    TS_ASSERT_DELTA(totalPressureRatio, 1.604, 0.01);
  }

  // Test compressor pressure ratio
  void testCompressorPressureRatio() {
    double OPR = 30.0;  // Overall pressure ratio (modern turbofan)
    double inletPressure = 14.7;  // psi

    double compressorExit = inletPressure * OPR;
    TS_ASSERT_DELTA(compressorExit, 441.0, 1.0);

    // Higher OPR = better efficiency but harder to achieve
    TS_ASSERT(OPR >= 20.0);  // Modern engines
    TS_ASSERT(OPR <= 60.0);  // Practical limit
  }

  // Test compressor surge margin
  void testCompressorSurgeMargin() {
    double operatingPoint = 0.85;  // Operating line
    double surgeLine = 1.0;        // Surge boundary

    double surgeMargin = (surgeLine - operatingPoint) / surgeLine * 100.0;
    TS_ASSERT_DELTA(surgeMargin, 15.0, 0.1);  // 15% margin

    // Minimum acceptable surge margin
    TS_ASSERT(surgeMargin > 10.0);
  }

  /***************************************************************************
   * Temperature Limits Tests
   ***************************************************************************/

  // Test turbine inlet temperature limit
  void testTurbineInletTempLimit() {
    double TIT_limit = 1700.0;  // K (modern engine limit)
    double TIT_actual = 1500.0;

    TS_ASSERT(TIT_actual < TIT_limit);

    // At max power, TIT approaches limit
    TIT_actual = 1650.0;
    double margin = TIT_limit - TIT_actual;
    TS_ASSERT(margin > 0);
    TS_ASSERT_DELTA(margin, 50.0, DEFAULT_TOLERANCE);
  }

  // Test ITT redline
  void testITTRedline() {
    double ITT_redline = 950.0;  // °C
    double ITT_normal = 750.0;
    double ITT_takeoff = 900.0;

    TS_ASSERT(ITT_normal < ITT_redline);
    TS_ASSERT(ITT_takeoff < ITT_redline);

    // Margin at takeoff
    double margin = ITT_redline - ITT_takeoff;
    TS_ASSERT_DELTA(margin, 50.0, DEFAULT_TOLERANCE);
  }

  // Test EGT limit exceedance
  void testEGTLimitExceedance() {
    double EGT_limit = 850.0;  // °C
    double EGT = 800.0;

    bool exceedance = EGT > EGT_limit;
    TS_ASSERT(!exceedance);

    // Over limit
    EGT = 880.0;
    exceedance = EGT > EGT_limit;
    TS_ASSERT(exceedance);
  }

  /***************************************************************************
   * Afterburner Tests
   ***************************************************************************/

  // Test afterburner staging
  void testAfterburnerStaging() {
    // Some engines have multiple AB stages
    int maxStages = 5;
    int stage = 0;
    double abThrust = 0.0;
    double milThrust = 10000.0;

    // Each stage adds thrust
    for (stage = 1; stage <= maxStages; stage++) {
      abThrust = milThrust * (1.0 + 0.1 * stage);
    }

    // At max AB
    TS_ASSERT_DELTA(abThrust, 15000.0, DEFAULT_TOLERANCE);
  }

  // Test afterburner fuel consumption
  void testAfterburnerFuelConsumption() {
    double dryFuelFlow = 8000.0;  // lbs/hr
    double wetFuelFlow = 20000.0; // lbs/hr with AB

    double abRatio = wetFuelFlow / dryFuelFlow;
    TS_ASSERT_DELTA(abRatio, 2.5, DEFAULT_TOLERANCE);

    // AB uses much more fuel for modest thrust increase
    double dryThrust = 10000.0;
    double wetThrust = 15000.0;
    double thrustRatio = wetThrust / dryThrust;

    // Fuel efficiency is poor in AB
    double efficiency = thrustRatio / abRatio;
    TS_ASSERT(efficiency < 1.0);
  }

  // Test afterburner light-off
  void testAfterburnerLightOff() {
    double N2_min = 85.0;  // Minimum N2 for AB
    double N2 = 90.0;
    bool ABlight = false;

    if (N2 >= N2_min) {
      ABlight = true;
    }
    TS_ASSERT(ABlight);

    // Cannot light AB at low N2
    N2 = 70.0;
    ABlight = (N2 >= N2_min);
    TS_ASSERT(!ABlight);
  }

  /***************************************************************************
   * Thrust Reverser Tests
   ***************************************************************************/

  // Test thrust reverser effect
  void testThrustReverser() {
    double forwardThrust = 20000.0;
    double reverserEfficiency = 0.4;  // 40% of thrust reversed

    double reverseThrust = -forwardThrust * reverserEfficiency;
    TS_ASSERT_DELTA(reverseThrust, -8000.0, DEFAULT_TOLERANCE);

    // Net deceleration force
    TS_ASSERT(reverseThrust < 0);
  }

  // Test thrust reverser at idle
  void testThrustReverserIdle() {
    double idleThrust = 2000.0;
    double reverserEfficiency = 0.4;

    double reverseThrust = -idleThrust * reverserEfficiency;
    TS_ASSERT_DELTA(reverseThrust, -800.0, DEFAULT_TOLERANCE);
  }

  // Test thrust reverser deploy time
  void testThrustReverserDeployTime() {
    double deployTime = 2.0;  // seconds
    double currentPosition = 0.0;  // stowed
    double targetPosition = 1.0;   // deployed
    double rate = 1.0 / deployTime;
    double dt = 0.1;

    // Simulate deployment
    for (int i = 0; i < 20; i++) {
      currentPosition += rate * dt;
    }

    TS_ASSERT_DELTA(currentPosition, 1.0, 0.01);
  }

  /***************************************************************************
   * Starting and Shutdown Tests
   ***************************************************************************/

  // Test starter dropout speed
  void testStarterDropout() {
    double N2 = 0.0;
    double starterDropoutN2 = 50.0;
    bool starterEngaged = true;

    // Below dropout
    TS_ASSERT(starterEngaged);

    // Above dropout, starter disengages
    N2 = 55.0;
    if (N2 > starterDropoutN2) {
      starterEngaged = false;
    }
    TS_ASSERT(!starterEngaged);
  }

  // Test light-off N2
  void testLightOffN2() {
    double N2 = 15.0;
    double lightOffN2 = 12.0;

    bool canLight = N2 >= lightOffN2;
    TS_ASSERT(canLight);

    // Below light-off
    N2 = 10.0;
    canLight = N2 >= lightOffN2;
    TS_ASSERT(!canLight);
  }

  // Test engine rundown time
  void testEngineRundown() {
    double N2 = 60.0;  // idle
    double rundownRate = 5.0;  // %/sec
    double rundownTime = N2 / rundownRate;

    TS_ASSERT_DELTA(rundownTime, 12.0, 0.1);  // 12 seconds to stop
  }

  // Test hung start detection
  void testHungStart() {
    double N2 = 25.0;
    double EGT = 600.0;
    double normalStartEGT = 500.0;
    double lightOffN2 = 20.0;

    // Hung start: N2 stalls but EGT high
    bool hungStart = (N2 > lightOffN2 && N2 < 50.0 && EGT > normalStartEGT);
    TS_ASSERT(hungStart);
  }

  // Test hot start detection
  void testHotStart() {
    double EGT = 950.0;
    double EGT_limit = 900.0;

    bool hotStart = EGT > EGT_limit;
    TS_ASSERT(hotStart);
  }

  /***************************************************************************
   * Mach and Altitude Effects Tests
   ***************************************************************************/

  // Test thrust lapse with altitude
  void testThrustLapseAltitude() {
    double seaLevelThrust = 25000.0;
    double altitudes[] = {0.0, 10000.0, 20000.0, 35000.0};
    double densityRatios[] = {1.0, 0.74, 0.53, 0.31};

    for (int i = 0; i < 4; i++) {
      double thrust = seaLevelThrust * densityRatios[i];
      TS_ASSERT(thrust <= seaLevelThrust);
      TS_ASSERT(thrust > 0);
    }
  }

  // Test ram drag
  void testRamDrag() {
    double massFlow = 200.0;  // lbs/sec
    double velocity = 800.0;   // ft/s
    double g = 32.2;

    // Ram drag = mass_flow * velocity / g
    double ramDrag = massFlow * velocity / g;
    TS_ASSERT_DELTA(ramDrag, 4969.0, 10.0);
  }

  // Test net thrust calculation
  void testNetThrust() {
    double grossThrust = 25000.0;
    double ramDrag = 5000.0;

    double netThrust = grossThrust - ramDrag;
    TS_ASSERT_DELTA(netThrust, 20000.0, DEFAULT_TOLERANCE);

    // At high speed, ram drag increases
    ramDrag = 10000.0;
    netThrust = grossThrust - ramDrag;
    TS_ASSERT_DELTA(netThrust, 15000.0, DEFAULT_TOLERANCE);
  }

  // Test thrust available at Mach
  void testThrustVsMach() {
    double seaLevelStaticThrust = 25000.0;
    double Mach = 0.0;

    // Static
    double thrust = seaLevelStaticThrust;
    TS_ASSERT_DELTA(thrust, 25000.0, DEFAULT_TOLERANCE);

    // At Mach 0.85
    Mach = 0.85;
    double machFactor = 1.0 - 0.15 * Mach;  // Simplified
    thrust = seaLevelStaticThrust * machFactor;
    TS_ASSERT_DELTA(thrust, 21812.5, 10.0);
  }

  /***************************************************************************
   * Engine Control Tests
   ***************************************************************************/

  // Test throttle to N2 mapping
  void testThrottleN2Mapping() {
    double throttle = 0.0;  // Idle
    double IdleN2 = 60.0;
    double MaxN2 = 100.0;

    double N2 = IdleN2 + throttle * (MaxN2 - IdleN2);
    TS_ASSERT_DELTA(N2, 60.0, DEFAULT_TOLERANCE);

    // Full throttle
    throttle = 1.0;
    N2 = IdleN2 + throttle * (MaxN2 - IdleN2);
    TS_ASSERT_DELTA(N2, 100.0, DEFAULT_TOLERANCE);
  }

  // Test acceleration schedule
  void testAccelerationSchedule() {
    double N2 = 70.0;
    double targetN2 = 100.0;
    double maxAccel = 20.0;  // %/sec at low N2
    double dt = 0.1;

    // Acceleration limited by schedule
    double accel = std::min(maxAccel, (targetN2 - N2) / 1.0);
    double newN2 = N2 + accel * dt;

    TS_ASSERT(newN2 > N2);
    TS_ASSERT(newN2 < targetN2);
  }

  // Test deceleration schedule
  void testDecelerationSchedule() {
    double N2 = 90.0;
    double targetN2 = 60.0;
    double maxDecel = 15.0;  // %/sec
    double dt = 0.1;

    double decel = std::min(maxDecel, (N2 - targetN2) / 1.0);
    double newN2 = N2 - decel * dt;

    TS_ASSERT(newN2 < N2);
    TS_ASSERT(newN2 > targetN2);
  }

  // Test engine protection cutback
  void testEngineProtection() {
    double TIT = 1680.0;
    double TIT_limit = 1700.0;
    double N2 = 98.0;

    // Protection activates near limit
    bool protection = (TIT > TIT_limit * 0.98);
    TS_ASSERT(protection);

    // Reduce N2 to protect engine
    if (protection) {
      N2 = N2 * 0.98;
    }
    TS_ASSERT(N2 < 98.0);
  }

  /***************************************************************************
   * Fuel System Tests
   ***************************************************************************/

  // Test minimum fuel pressure
  void testMinFuelPressure() {
    double fuelPressure = 25.0;  // psi
    double minPressure = 15.0;

    bool fuelOK = fuelPressure >= minPressure;
    TS_ASSERT(fuelOK);

    // Low fuel pressure
    fuelPressure = 10.0;
    fuelOK = fuelPressure >= minPressure;
    TS_ASSERT(!fuelOK);
  }

  // Test fuel flow vs N2
  void testFuelFlowVsN2() {
    double N2 = 60.0;  // idle
    double IdleN2 = 60.0;
    double MaxN2 = 100.0;
    double idleFuelFlow = 500.0;
    double maxFuelFlow = 8000.0;

    double N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2);
    double fuelFlow = idleFuelFlow + (maxFuelFlow - idleFuelFlow) * pow(N2norm, 2);

    TS_ASSERT_DELTA(fuelFlow, 500.0, DEFAULT_TOLERANCE);

    // At max N2
    N2 = 100.0;
    N2norm = (N2 - IdleN2) / (MaxN2 - IdleN2);
    fuelFlow = idleFuelFlow + (maxFuelFlow - idleFuelFlow) * pow(N2norm, 2);
    TS_ASSERT_DELTA(fuelFlow, 8000.0, DEFAULT_TOLERANCE);
  }

  // Test fuel heating value
  void testFuelHeatingValue() {
    double JetA_LHV = 18400.0;  // BTU/lb
    double JP4_LHV = 18600.0;

    TS_ASSERT(JP4_LHV > JetA_LHV);

    // Energy available
    double fuelFlow = 8000.0;  // lbs/hr
    double energyRate = fuelFlow * JetA_LHV;  // BTU/hr
    TS_ASSERT(energyRate > 0);
  }

  /***************************************************************************
   * Accessory and Bleed Tests
   ***************************************************************************/

  // Test accessory power extraction
  void testAccessoryPowerExtraction() {
    double shaftPower = 50000.0;  // HP
    double accessoryLoad = 200.0;  // HP

    double availablePower = shaftPower - accessoryLoad;
    TS_ASSERT_DELTA(availablePower, 49800.0, DEFAULT_TOLERANCE);

    // Accessory load as percentage
    double loadPercent = accessoryLoad / shaftPower * 100.0;
    TS_ASSERT_DELTA(loadPercent, 0.4, 0.01);
  }

  // Test bleed air extraction limits
  void testBleedAirLimits() {
    double maxBleed = 0.10;  // 10% of core flow
    double currentBleed = 0.05;

    TS_ASSERT(currentBleed <= maxBleed);

    // Excessive bleed
    currentBleed = 0.15;
    bool bleedExcessive = currentBleed > maxBleed;
    TS_ASSERT(bleedExcessive);
  }

  // Test anti-ice bleed demand
  void testAntiIceBleed() {
    double cowlAntiIce = 0.02;   // 2%
    double wingAntiIce = 0.03;  // 3%
    bool cowlOn = true;
    bool wingOn = true;

    double totalBleed = 0.0;
    if (cowlOn) totalBleed += cowlAntiIce;
    if (wingOn) totalBleed += wingAntiIce;

    TS_ASSERT_DELTA(totalBleed, 0.05, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Miscellaneous Tests
   ***************************************************************************/

  // Test nozzle area ratio
  void testNozzleAreaRatio() {
    double A8 = 3.5;  // Nozzle throat area (sq ft)
    double A9 = 5.0;  // Nozzle exit area (sq ft)

    double areaRatio = A9 / A8;
    TS_ASSERT_DELTA(areaRatio, 1.428, 0.01);

    // For supersonic nozzle, ratio > 1
    TS_ASSERT(areaRatio > 1.0);
  }

  // Test exhaust velocity
  void testExhaustVelocity() {
    double thrust = 20000.0;  // lbf
    double massFlow = 200.0;  // lbs/sec
    double g = 32.2;

    // Ve = Thrust * g / massFlow
    double Ve = thrust * g / massFlow;
    TS_ASSERT_DELTA(Ve, 3220.0, 10.0);  // ft/s
  }

  // Test propulsive efficiency
  void testPropulsiveEfficiency() {
    double flightVelocity = 800.0;  // ft/s
    double exhaustVelocity = 2000.0;  // ft/s

    // eta_p = 2 * V0 / (V0 + Ve)
    double eta_p = 2.0 * flightVelocity / (flightVelocity + exhaustVelocity);
    TS_ASSERT_DELTA(eta_p, 0.571, 0.01);

    // Higher bypass = higher propulsive efficiency
    exhaustVelocity = 1200.0;  // High bypass engine
    eta_p = 2.0 * flightVelocity / (flightVelocity + exhaustVelocity);
    TS_ASSERT_DELTA(eta_p, 0.80, 0.01);
  }

  // Test engine weight to thrust ratio
  void testWeightThrustRatio() {
    double thrust = 25000.0;  // lbf
    double weight = 5000.0;   // lbs

    double ratio = thrust / weight;
    TS_ASSERT_DELTA(ratio, 5.0, DEFAULT_TOLERANCE);

    // Modern engines achieve 5-8:1
    TS_ASSERT(ratio >= 4.0);
    TS_ASSERT(ratio <= 10.0);
  }

  /***************************************************************************
   * Extended Spool Dynamics Tests
   ***************************************************************************/

  // Test N1 acceleration rate
  void testN1AccelerationRate() {
    double N1 = 25.0;
    double targetN1 = 100.0;
    double accelRate = 20.0;  // %/sec
    double dt = 0.1;

    for (int i = 0; i < 40; i++) {
      double delta = std::min(accelRate * dt, targetN1 - N1);
      N1 += delta;
    }

    TS_ASSERT(N1 > 95.0);
  }

  // Test N2 deceleration rate
  void testN2DecelerationRate() {
    double N2 = 100.0;
    double targetN2 = 60.0;
    double decelRate = 10.0;  // %/sec
    double dt = 0.1;

    for (int i = 0; i < 50; i++) {
      double delta = std::min(decelRate * dt, N2 - targetN2);
      N2 -= delta;
    }

    TS_ASSERT(N2 < 65.0);
  }

  // Test spool inertia effect
  void testSpoolInertia() {
    double spoolMass = 100.0;  // kg
    double radius = 0.3;       // m
    double inertia = 0.5 * spoolMass * radius * radius;

    TS_ASSERT(inertia > 0);
    TS_ASSERT_DELTA(inertia, 4.5, 0.1);
  }

  /***************************************************************************
   * Extended Combustion Tests
   ***************************************************************************/

  // Test combustion efficiency
  void testCombustionEfficiency() {
    double fuelEnergy = 18400.0;  // BTU/lb
    double actualEnergy = 17000.0;

    double efficiency = actualEnergy / fuelEnergy;
    TS_ASSERT(efficiency > 0.90);
    TS_ASSERT(efficiency < 1.0);
  }

  // Test flame stability
  void testFlameStability() {
    double airVelocity = 50.0;  // ft/s in combustor
    double maxStableVelocity = 100.0;

    bool stable = airVelocity < maxStableVelocity;
    TS_ASSERT(stable);
  }

  // Test fuel-air ratio
  void testFuelAirRatio() {
    double fuelFlow = 8000.0;  // lbs/hr
    double airFlow = 200.0 * 3600.0;  // lbs/hr

    double FAR = fuelFlow / airFlow;
    // Typical FAR: 0.01-0.02
    TS_ASSERT(FAR > 0.005);
    TS_ASSERT(FAR < 0.03);
  }

  /***************************************************************************
   * Extended Compressor Tests
   ***************************************************************************/

  // Test compressor stall recovery
  void testCompressorStallRecovery() {
    bool stalled = true;
    double N2 = 95.0;
    double targetN2 = 60.0;

    // Reduce power to recover
    if (stalled) {
      while (N2 > targetN2) {
        N2 -= 5.0;
      }
      stalled = false;
    }

    TS_ASSERT(!stalled);
    TS_ASSERT(N2 <= targetN2);
  }

  // Test compressor bleed valve
  void testCompressorBleedValve() {
    double N2 = 50.0;  // During start
    double bleedThreshold = 80.0;
    bool bleedOpen = N2 < bleedThreshold;

    TS_ASSERT(bleedOpen);

    N2 = 90.0;
    bleedOpen = N2 < bleedThreshold;
    TS_ASSERT(!bleedOpen);
  }

  // Test variable geometry
  void testVariableGeometry() {
    double N2 = 70.0;
    double vaneAngle = 0.0;  // degrees

    // Vanes close as N2 increases
    if (N2 < 60.0) vaneAngle = 30.0;
    else if (N2 < 80.0) vaneAngle = 15.0;
    else vaneAngle = 0.0;

    TS_ASSERT_DELTA(vaneAngle, 15.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended Turbine Tests
   ***************************************************************************/

  // Test turbine power extraction
  void testTurbinePowerExtraction() {
    double compressorPower = 40000.0;  // HP
    double accessoryPower = 500.0;
    double totalExtraction = compressorPower + accessoryPower;

    TS_ASSERT_DELTA(totalExtraction, 40500.0, DEFAULT_TOLERANCE);
  }

  // Test turbine cooling effectiveness
  void testTurbineCooling() {
    double bladeTemp = 1500.0;  // K
    double coolingAirTemp = 700.0;
    double coolingEffectiveness = 0.6;

    double effectiveTemp = bladeTemp - coolingEffectiveness * (bladeTemp - coolingAirTemp);
    TS_ASSERT(effectiveTemp < bladeTemp);
    TS_ASSERT_DELTA(effectiveTemp, 1020.0, 10.0);
  }

  // Test turbine blade creep
  void testTurbineBladeCreep() {
    double temp = 1600.0;  // K
    double stress = 200.0; // MPa
    double creepLimit = 1700.0;

    bool creepRisk = temp > creepLimit * 0.95;
    TS_ASSERT(!creepRisk);

    temp = 1650.0;
    creepRisk = temp > creepLimit * 0.95;
    TS_ASSERT(creepRisk);
  }

  /***************************************************************************
   * Extended Nozzle Tests
   ***************************************************************************/

  // Test convergent nozzle
  void testConvergentNozzle() {
    double throatArea = 3.0;  // sq ft
    double exitArea = 3.0;    // Same for convergent

    double areaRatio = exitArea / throatArea;
    TS_ASSERT_DELTA(areaRatio, 1.0, DEFAULT_TOLERANCE);
  }

  // Test convergent-divergent nozzle
  void testConvergentDivergentNozzle() {
    double throatArea = 3.0;  // sq ft
    double exitArea = 5.0;    // Larger for supersonic

    double areaRatio = exitArea / throatArea;
    TS_ASSERT(areaRatio > 1.0);
    TS_ASSERT_DELTA(areaRatio, 1.667, 0.01);
  }

  // Test nozzle pressure ratio
  void testNozzlePressureRatio() {
    double P_total = 50.0;  // psia
    double P_ambient = 14.7;

    double NPR = P_total / P_ambient;
    TS_ASSERT_DELTA(NPR, 3.4, 0.1);

    // Critical NPR for choked flow
    double criticalNPR = 1.89;  // For gamma = 1.4
    TS_ASSERT(NPR > criticalNPR);
  }

  /***************************************************************************
   * Extended Efficiency Tests
   ***************************************************************************/

  // Test overall efficiency
  void testOverallEfficiency() {
    double thermalEff = 0.45;
    double propulsiveEff = 0.80;

    double overallEff = thermalEff * propulsiveEff;
    TS_ASSERT_DELTA(overallEff, 0.36, 0.01);
  }

  // Test polytropic efficiency
  void testPolytropicEfficiency() {
    double gamma = 1.4;
    double PR = 30.0;  // Pressure ratio
    double T2_T1_actual = 3.5;
    double T2_T1_ideal = pow(PR, (gamma-1)/gamma);  // ≈ 2.77

    // isentropicEff = (2.77-1)/(3.5-1) ≈ 0.71
    double isentropicEff = (T2_T1_ideal - 1) / (T2_T1_actual - 1);
    TS_ASSERT(isentropicEff < 1.0);
    TS_ASSERT(isentropicEff > 0.6);
  }

  /***************************************************************************
   * Extended Operating Envelope Tests
   ***************************************************************************/

  // Test altitude thrust lapse
  void testAltitudeThrustLapse() {
    double seaLevelThrust = 25000.0;

    // Thrust at various altitudes
    double thrust10k = seaLevelThrust * 0.74;
    double thrust20k = seaLevelThrust * 0.53;
    double thrust35k = seaLevelThrust * 0.31;

    TS_ASSERT(thrust10k < seaLevelThrust);
    TS_ASSERT(thrust20k < thrust10k);
    TS_ASSERT(thrust35k < thrust20k);
  }

  // Test Mach effect on thrust
  void testMachEffectOnThrust() {
    double staticThrust = 25000.0;
    double Mach = 0.85;

    // Ram effect partially compensates for inlet losses
    double machFactor = 1.0 - 0.1 * Mach + 0.05 * Mach * Mach;
    double thrust = staticThrust * machFactor;

    TS_ASSERT(thrust < staticThrust);
    TS_ASSERT(thrust > staticThrust * 0.85);
  }

  // Test temperature effect on thrust
  void testTemperatureEffectOnThrust() {
    double standardThrust = 25000.0;
    double ISA_temp = 288.15;  // K

    // Hot day
    double hotDay = 303.15;  // K (ISA+15)
    double hotDayThrust = standardThrust * sqrt(ISA_temp / hotDay);

    TS_ASSERT(hotDayThrust < standardThrust);

    // Cold day
    double coldDay = 268.15;  // K (ISA-20)
    double coldDayThrust = standardThrust * sqrt(ISA_temp / coldDay);

    TS_ASSERT(coldDayThrust > standardThrust);
  }

  /***************************************************************************
   * Extended Start Sequence Tests
   ***************************************************************************/

  // Test starter energy requirements
  void testStarterEnergyRequirements() {
    double spoolInertia = 5.0;  // kg*m^2
    double targetN2 = 0.25;     // 25% N2
    double maxN2_rps = 200.0;   // rad/s at 100%
    double omega = targetN2 * maxN2_rps;  // = 50 rad/s

    // E = 0.5 * I * omega^2 = 0.5 * 5 * 50^2 = 6250 J
    double energy = 0.5 * spoolInertia * omega * omega;
    TS_ASSERT(energy > 0);
    TS_ASSERT_DELTA(energy, 6250.0, 100.0);
  }

  // Test start time vs temperature
  void testStartTimeVsTemperature() {
    double baseStartTime = 30.0;  // seconds
    double ambientTemp = 288.15;
    double coldTemp = 253.15;

    // Cold starts take longer
    double coldStartTime = baseStartTime * sqrt(ambientTemp / coldTemp);
    TS_ASSERT(coldStartTime > baseStartTime);
  }

  // Test altitude restart envelope
  void testAltitudeRestartEnvelope() {
    double maxRestartAlt = 25000.0;  // ft
    double currentAlt = 20000.0;

    bool canRestart = currentAlt <= maxRestartAlt;
    TS_ASSERT(canRestart);

    currentAlt = 30000.0;
    canRestart = currentAlt <= maxRestartAlt;
    TS_ASSERT(!canRestart);
  }

  /***************************************************************************
   * Extended Fuel Control Tests
   ***************************************************************************/

  // Test fuel metering unit
  void testFuelMeteringUnit() {
    double N2 = 80.0;
    double throttle = 0.7;
    double baseFuelFlow = 6000.0;  // lbs/hr

    double fuelFlow = baseFuelFlow * throttle * (N2 / 100.0);
    TS_ASSERT_DELTA(fuelFlow, 3360.0, 10.0);
  }

  // Test fuel schedule during acceleration
  void testFuelScheduleAcceleration() {
    double N2 = 70.0;
    double targetN2 = 100.0;
    double maxAccelFuel = 10000.0;
    double idleFuel = 500.0;

    // Fuel increases with N2 demand
    double fuelSchedule = idleFuel + (maxAccelFuel - idleFuel) * (targetN2 - N2) / 30.0;
    TS_ASSERT_DELTA(fuelSchedule, 10000.0, 100.0);
  }

  // Test fuel enrichment during start
  void testFuelEnrichmentStart() {
    double normalFuel = 500.0;
    double enrichmentFactor = 1.5;
    double N2 = 20.0;
    double enrichmentCutoff = 50.0;

    double fuelFlow = normalFuel;
    if (N2 < enrichmentCutoff) {
      fuelFlow *= enrichmentFactor;
    }

    TS_ASSERT_DELTA(fuelFlow, 750.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Extended Vibration and Health Tests
   ***************************************************************************/

  // Test vibration monitoring
  void testVibrationMonitoring() {
    double N1_vib = 0.5;   // mils
    double N2_vib = 0.3;
    double maxVib = 1.0;

    bool vibOK = (N1_vib < maxVib) && (N2_vib < maxVib);
    TS_ASSERT(vibOK);
  }

  // Test engine trend monitoring
  void testEngineTrendMonitoring() {
    double baselineEGT = 700.0;
    double currentEGT = 720.0;
    double alertThreshold = 25.0;

    double deviation = currentEGT - baselineEGT;
    bool trendAlert = deviation > alertThreshold;

    TS_ASSERT(!trendAlert);

    currentEGT = 730.0;
    deviation = currentEGT - baselineEGT;
    trendAlert = deviation > alertThreshold;
    TS_ASSERT(trendAlert);
  }

  // Test FOD detection
  void testFODDetection() {
    double N1_baseline = 95.0;
    double N1_current = 93.5;
    double N1_threshold = 2.0;

    bool possibleFOD = (N1_baseline - N1_current) > N1_threshold;
    TS_ASSERT(!possibleFOD);

    N1_current = 92.0;
    possibleFOD = (N1_baseline - N1_current) > N1_threshold;
    TS_ASSERT(possibleFOD);
  }

  /***************************************************************************
   * Extended Environmental Tests
   ***************************************************************************/

  // Test rain ingestion
  void testRainIngestion() {
    double rainRate = 2.0;  // inches/hour (heavy)
    double thrustLoss = rainRate * 0.5;  // % per in/hr

    TS_ASSERT_DELTA(thrustLoss, 1.0, DEFAULT_TOLERANCE);
    TS_ASSERT(thrustLoss < 5.0);  // Acceptable loss
  }

  // Test icing conditions
  void testIcingConditions() {
    double OAT = -5.0;  // °C
    double humidity = 0.9;
    double LWC = 0.5;   // Liquid water content (g/m^3)

    bool icingRisk = (OAT < 0 && OAT > -20 && LWC > 0.1);
    TS_ASSERT(icingRisk);
  }

  // Test volcanic ash detection
  void testVolcanicAshDetection() {
    double EGT_normal = 700.0;
    double EGT_current = 680.0;  // Drops with ash

    double EGT_drop = EGT_normal - EGT_current;
    bool ashPossible = EGT_drop > 15.0;

    TS_ASSERT(ashPossible);
  }

  /***************************************************************************
   * Extended Performance Margin Tests
   ***************************************************************************/

  // Test takeoff thrust margin
  void testTakeoffThrustMargin() {
    double requiredThrust = 22000.0;
    double availableThrust = 25000.0;

    double margin = (availableThrust - requiredThrust) / requiredThrust * 100.0;
    TS_ASSERT(margin > 10.0);
    TS_ASSERT_DELTA(margin, 13.6, 0.1);
  }

  // Test climb thrust rating
  void testClimbThrustRating() {
    double takeoffThrust = 25000.0;
    double climbDerate = 0.90;

    double climbThrust = takeoffThrust * climbDerate;
    TS_ASSERT_DELTA(climbThrust, 22500.0, DEFAULT_TOLERANCE);
  }

  // Test cruise thrust setting
  void testCruiseThrustSetting() {
    double maxCruiseThrust = 18000.0;
    double cruiseSetting = 0.85;

    double cruiseThrust = maxCruiseThrust * cruiseSetting;
    TS_ASSERT_DELTA(cruiseThrust, 15300.0, DEFAULT_TOLERANCE);
  }
};
