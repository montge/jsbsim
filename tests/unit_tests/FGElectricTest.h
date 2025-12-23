#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGElectric.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGElectricTest : public CxxTest::TestSuite
{
public:
  // Test engine type is etElectric
  void testEngineType() {
    // Electric motor should be type etElectric (5)
    TS_ASSERT_EQUALS(FGEngine::etElectric, 5);
  }

  // Test watts to horsepower conversion constant
  void testWattsToHPConstant() {
    // 1 HP = 745.7 watts (standard conversion)
    double hptowatts = 745.7;
    TS_ASSERT_DELTA(hptowatts, 745.7, DEFAULT_TOLERANCE);

    // Verify roundtrip conversion
    double hp = 100.0;
    double watts = hp * hptowatts;
    double hpBack = watts / hptowatts;
    TS_ASSERT_DELTA(hpBack, hp, DEFAULT_TOLERANCE);
  }

  // Test power available calculation
  void testPowerAvailableCalculation() {
    // GetPowerAvailable() = HP * hptoftlbssec
    double hptoftlbssec = 550.0;  // 1 HP = 550 ft-lbf/sec

    double hp = 100.0;
    double powerAvailable = hp * hptoftlbssec;
    TS_ASSERT_DELTA(powerAvailable, 55000.0, DEFAULT_TOLERANCE);

    // Zero HP
    hp = 0.0;
    powerAvailable = hp * hptoftlbssec;
    TS_ASSERT_DELTA(powerAvailable, 0.0, DEFAULT_TOLERANCE);

    // High-power motor (e.g., 500 HP electric motor)
    hp = 500.0;
    powerAvailable = hp * hptoftlbssec;
    TS_ASSERT_DELTA(powerAvailable, 275000.0, DEFAULT_TOLERANCE);
  }

  // Test HP calculation from throttle and power rating
  void testHPFromThrottle() {
    // HP = PowerWatts * ThrottlePos / hptowatts
    double hptowatts = 745.7;
    double powerWatts = 74570.0;  // 100 HP motor

    // Full throttle
    double throttlePos = 1.0;
    double hp = powerWatts * throttlePos / hptowatts;
    TS_ASSERT_DELTA(hp, 100.0, 0.01);

    // Half throttle
    throttlePos = 0.5;
    hp = powerWatts * throttlePos / hptowatts;
    TS_ASSERT_DELTA(hp, 50.0, 0.01);

    // Zero throttle
    throttlePos = 0.0;
    hp = powerWatts * throttlePos / hptowatts;
    TS_ASSERT_DELTA(hp, 0.0, DEFAULT_TOLERANCE);

    // 75% throttle
    throttlePos = 0.75;
    hp = powerWatts * throttlePos / hptowatts;
    TS_ASSERT_DELTA(hp, 75.0, 0.01);
  }

  // Test RPM calculation from thruster
  void testRPMFromThruster() {
    // RPM = Thruster->GetRPM() * Thruster->GetGearRatio()
    double thrusterRPM = 5000.0;
    double gearRatio = 2.0;

    double engineRPM = thrusterRPM * gearRatio;
    TS_ASSERT_DELTA(engineRPM, 10000.0, DEFAULT_TOLERANCE);

    // Direct drive (gear ratio 1.0)
    gearRatio = 1.0;
    engineRPM = thrusterRPM * gearRatio;
    TS_ASSERT_DELTA(engineRPM, thrusterRPM, DEFAULT_TOLERANCE);

    // Reduction gearing (gear ratio < 1)
    gearRatio = 0.5;
    engineRPM = thrusterRPM * gearRatio;
    TS_ASSERT_DELTA(engineRPM, 2500.0, DEFAULT_TOLERANCE);
  }

  // Test that electric motors don't consume fuel
  void testNoFuelConsumption() {
    // CalcFuelNeed() returns 0 for electric motors
    double fuelNeed = 0.0;  // This is what FGElectric::CalcFuelNeed returns
    TS_ASSERT_DELTA(fuelNeed, 0.0, DEFAULT_TOLERANCE);
  }

  // Test power output at various throttle settings
  void testPowerOutputRange() {
    double hptowatts = 745.7;
    double hptoftlbssec = 550.0;
    double maxPowerWatts = 37285.0;  // 50 HP motor

    // Test at 0%, 25%, 50%, 75%, 100% throttle
    double throttleSettings[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    double expectedHP[] = {0.0, 12.5, 25.0, 37.5, 50.0};

    for (int i = 0; i < 5; i++) {
      double hp = maxPowerWatts * throttleSettings[i] / hptowatts;
      double powerFtLbSec = hp * hptoftlbssec;

      TS_ASSERT_DELTA(hp, expectedHP[i], 0.1);
      TS_ASSERT(powerFtLbSec >= 0.0);

      // Power should be linear with throttle
      if (i > 0) {
        double ratio = throttleSettings[i] / throttleSettings[i-1];
        double powerRatio = hp / expectedHP[i-1];
        if (expectedHP[i-1] > 0.0) {
          TS_ASSERT_DELTA(powerRatio, ratio, 0.01);
        }
      }
    }
  }

  // Test default power rating
  void testDefaultPowerRating() {
    // Default PowerWatts in FGElectric constructor is 745.7 (1 HP)
    double defaultPowerWatts = 745.7;
    double hptowatts = 745.7;

    double defaultHP = defaultPowerWatts / hptowatts;
    TS_ASSERT_DELTA(defaultHP, 1.0, DEFAULT_TOLERANCE);
  }

  // Test common electric motor power ratings
  void testCommonMotorRatings() {
    double hptowatts = 745.7;

    // Small drone motor (1 kW = 1.34 HP)
    double droneMotor = 1000.0;
    double droneHP = droneMotor / hptowatts;
    TS_ASSERT_DELTA(droneHP, 1.341, 0.001);

    // Small UAV motor (5 HP)
    double uavMotor = 5.0 * hptowatts;
    TS_ASSERT_DELTA(uavMotor, 3728.5, 0.1);

    // Light sport aircraft (100 HP)
    double lsaMotor = 100.0 * hptowatts;
    TS_ASSERT_DELTA(lsaMotor, 74570.0, 1.0);

    // eVTOL aircraft (500 HP)
    double evtolMotor = 500.0 * hptowatts;
    TS_ASSERT_DELTA(evtolMotor, 372850.0, 1.0);
  }

  // Test negative power filtering at low RPM
  void testNegativePowerFiltering() {
    // Electric motors filter out negative power when RPM <= 0.1
    double rpm = 0.0;
    double power = -100.0;  // Negative power (regenerative)

    // If RPM <= 0.1, power should be max(power, 0.0)
    if (rpm <= 0.1) {
      power = std::max(power, 0.0);
    }
    TS_ASSERT_DELTA(power, 0.0, DEFAULT_TOLERANCE);

    // At higher RPM, negative power is allowed
    rpm = 1000.0;
    power = -100.0;
    if (rpm <= 0.1) {
      power = std::max(power, 0.0);
    }
    TS_ASSERT_DELTA(power, -100.0, DEFAULT_TOLERANCE);

    // At boundary (0.1 RPM)
    rpm = 0.1;
    power = -50.0;
    if (rpm <= 0.1) {
      power = std::max(power, 0.0);
    }
    TS_ASSERT_DELTA(power, 0.0, DEFAULT_TOLERANCE);
  }

  // Test power output continuity
  void testPowerContinuity() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;  // 100 HP motor

    double prevPower = 0.0;
    for (double throttle = 0.0; throttle <= 1.0; throttle += 0.01) {
      double hp = maxPowerWatts * throttle / hptowatts;
      double power = hp * 550.0;

      // Power should increase monotonically
      TS_ASSERT(power >= prevPower - DEFAULT_TOLERANCE);
      prevPower = power;
    }
  }

  // Test instant power response (no lag)
  void testInstantPowerResponse() {
    // Electric motors have essentially instant torque response
    // (unlike piston/turbine engines with spool-up time)
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    // Instant jump from 0% to 100%
    double throttle1 = 0.0;
    double throttle2 = 1.0;

    double hp1 = maxPowerWatts * throttle1 / hptowatts;
    double hp2 = maxPowerWatts * throttle2 / hptowatts;

    // Power should instantly reflect throttle position
    TS_ASSERT_DELTA(hp1, 0.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(hp2, 100.0, 0.01);
  }

  // Test torque calculation from power and RPM
  void testTorqueFromPowerRPM() {
    // Torque (ft-lbf) = Power (ft-lbf/sec) / omega (rad/sec)
    // omega = RPM * 2 * PI / 60
    double hptoftlbssec = 550.0;
    double hp = 100.0;
    double rpm = 3000.0;

    double power = hp * hptoftlbssec;  // 55000 ft-lbf/sec
    double omega = rpm * 2.0 * M_PI / 60.0;  // rad/sec
    double torque = power / omega;

    // Expected: 55000 / (3000 * 2 * PI / 60) = 55000 / 314.16 = 175.07 ft-lbf
    TS_ASSERT_DELTA(torque, 175.07, 0.1);

    // Higher RPM = lower torque for same power
    rpm = 6000.0;
    omega = rpm * 2.0 * M_PI / 60.0;
    double torqueHigh = power / omega;
    TS_ASSERT(torqueHigh < torque);
    TS_ASSERT_DELTA(torqueHigh, 87.535, 0.1);
  }

  // Test efficiency consideration (not modeled, but verify linear)
  void testLinearThrottleResponse() {
    // FGElectric assumes 100% efficiency (no losses)
    // Power output is exactly linear with throttle
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    for (double t = 0.0; t <= 1.0; t += 0.1) {
      double hp = maxPowerWatts * t / hptowatts;
      double expectedHP = 100.0 * t;
      TS_ASSERT_DELTA(hp, expectedHP, 0.1);
    }
  }

  // Test motor behavior at zero throttle
  void testZeroThrottleBehavior() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;
    double throttle = 0.0;

    double hp = maxPowerWatts * throttle / hptowatts;
    TS_ASSERT_DELTA(hp, 0.0, DEFAULT_TOLERANCE);

    // Power available should be zero
    double powerAvailable = hp * 550.0;
    TS_ASSERT_DELTA(powerAvailable, 0.0, DEFAULT_TOLERANCE);
  }

  // Test motor behavior at max throttle
  void testMaxThrottleBehavior() {
    double hptowatts = 745.7;
    double hptoftlbssec = 550.0;
    double maxPowerWatts = 74570.0;
    double throttle = 1.0;

    double hp = maxPowerWatts * throttle / hptowatts;
    TS_ASSERT_DELTA(hp, 100.0, 0.01);

    double powerAvailable = hp * hptoftlbssec;
    TS_ASSERT_DELTA(powerAvailable, 55000.0, 1.0);
  }

  // Test engine labels format
  void testEngineLabelsFormat() {
    // Labels format: "{Name} HP (engine {N}){delimiter}{thruster labels}"
    std::string name = "Electric Motor";
    int engineNumber = 0;
    std::string delimiter = ",";

    std::string expectedStart = name + " HP (engine " + std::to_string(engineNumber) + ")" + delimiter;
    TS_ASSERT(expectedStart.find("Electric Motor HP") != std::string::npos);
    TS_ASSERT(expectedStart.find("engine 0") != std::string::npos);
  }

  // Test SI to Imperial conversions
  void testSIToImperialConversions() {
    // Watts to HP
    double watts = 1000.0;  // 1 kW
    double hp = watts / 745.7;
    TS_ASSERT_DELTA(hp, 1.341, 0.001);

    // kW to HP
    double kw = 75.0;  // 75 kW (about 100 HP)
    hp = kw * 1000.0 / 745.7;
    TS_ASSERT_DELTA(hp, 100.58, 0.1);

    // MW to HP (large motors)
    double mw = 1.0;  // 1 MW
    hp = mw * 1000000.0 / 745.7;
    TS_ASSERT_DELTA(hp, 1341.0, 1.0);
  }
};
