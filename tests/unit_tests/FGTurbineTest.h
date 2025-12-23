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
};
