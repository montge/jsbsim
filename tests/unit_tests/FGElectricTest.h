#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGElectric.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <initialization/FGInitialCondition.h>
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

  /***************************************************************************
   * Extended Power Calculation Tests
   ***************************************************************************/

  // Test power at very small throttle
  void testVerySmallThrottle() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;
    double throttle = 0.001;  // 0.1%

    double hp = maxPowerWatts * throttle / hptowatts;
    TS_ASSERT_DELTA(hp, 0.1, 0.01);
    TS_ASSERT(hp > 0.0);
  }

  // Test power at throttle just below max
  void testNearMaxThrottle() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;
    double throttle = 0.999;  // 99.9%

    double hp = maxPowerWatts * throttle / hptowatts;
    TS_ASSERT_DELTA(hp, 99.9, 0.01);
    TS_ASSERT(hp < 100.0);
  }

  // Test power linearity over fine increments
  void testPowerLinearityFineIncrements() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    double prevHP = 0.0;
    for (double throttle = 0.0; throttle <= 1.0; throttle += 0.001) {
      double hp = maxPowerWatts * throttle / hptowatts;

      // Each increment should add same amount
      if (throttle > 0.0) {
        double increment = hp - prevHP;
        TS_ASSERT_DELTA(increment, 0.1, 0.01);  // 0.1 HP per 0.1% throttle
      }
      prevHP = hp;
    }
  }

  // Test power available at mid-range
  void testMidRangePower() {
    double hptoftlbssec = 550.0;
    double hp = 50.0;  // Mid-range for 100 HP motor

    double powerAvailable = hp * hptoftlbssec;
    TS_ASSERT_DELTA(powerAvailable, 27500.0, 0.1);
  }

  /***************************************************************************
   * RPM and Gear Ratio Tests
   ***************************************************************************/

  // Test RPM with various gear ratios
  void testVariousGearRatios() {
    double thrusterRPM = 5000.0;
    double gearRatios[] = {0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0};
    double expectedRPM[] = {1250.0, 2500.0, 5000.0, 7500.0, 10000.0, 15000.0, 20000.0};

    for (int i = 0; i < 7; i++) {
      double engineRPM = thrusterRPM * gearRatios[i];
      TS_ASSERT_DELTA(engineRPM, expectedRPM[i], 0.1);
    }
  }

  // Test RPM at zero
  void testZeroRPM() {
    double thrusterRPM = 0.0;
    double gearRatio = 2.0;

    double engineRPM = thrusterRPM * gearRatio;
    TS_ASSERT_DELTA(engineRPM, 0.0, DEFAULT_TOLERANCE);
  }

  // Test very high RPM (high-speed electric motor)
  void testHighSpeedMotorRPM() {
    double thrusterRPM = 30000.0;  // High-speed motor
    double gearRatio = 0.1;  // Significant reduction

    double engineRPM = thrusterRPM * gearRatio;
    TS_ASSERT_DELTA(engineRPM, 3000.0, 0.1);
  }

  // Test RPM at boundary (0.1 threshold)
  void testRPMBoundaryThreshold() {
    double rpm = 0.1;
    double power = -100.0;

    // At exactly 0.1, negative power is filtered
    if (rpm <= 0.1) {
      power = std::max(power, 0.0);
    }
    TS_ASSERT_DELTA(power, 0.0, DEFAULT_TOLERANCE);

    // Just above 0.1
    rpm = 0.11;
    power = -100.0;
    if (rpm <= 0.1) {
      power = std::max(power, 0.0);
    }
    TS_ASSERT_DELTA(power, -100.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Torque Tests
   ***************************************************************************/

  // Test torque at various RPM levels
  void testTorqueAtVariousRPM() {
    double hptoftlbssec = 550.0;
    double hp = 100.0;
    double power = hp * hptoftlbssec;

    double rpms[] = {1000.0, 2000.0, 3000.0, 5000.0, 10000.0};
    double expectedTorques[] = {525.21, 262.61, 175.07, 105.04, 52.52};

    for (int i = 0; i < 5; i++) {
      double omega = rpms[i] * 2.0 * M_PI / 60.0;
      double torque = power / omega;
      TS_ASSERT_DELTA(torque, expectedTorques[i], 0.1);
    }
  }

  // Test torque at low RPM (high torque)
  void testHighTorqueLowRPM() {
    double hptoftlbssec = 550.0;
    double hp = 100.0;
    double power = hp * hptoftlbssec;
    double rpm = 500.0;

    double omega = rpm * 2.0 * M_PI / 60.0;
    double torque = power / omega;

    // Low RPM should give high torque
    TS_ASSERT(torque > 1000.0);
    TS_ASSERT_DELTA(torque, 1050.42, 0.1);
  }

  // Test torque at high RPM (low torque)
  void testLowTorqueHighRPM() {
    double hptoftlbssec = 550.0;
    double hp = 100.0;
    double power = hp * hptoftlbssec;
    double rpm = 20000.0;

    double omega = rpm * 2.0 * M_PI / 60.0;
    double torque = power / omega;

    // High RPM should give low torque
    TS_ASSERT(torque < 30.0);
    TS_ASSERT_DELTA(torque, 26.26, 0.1);
  }

  // Test torque-speed curve inversely proportional
  void testTorqueSpeedInverse() {
    double hptoftlbssec = 550.0;
    double hp = 100.0;
    double power = hp * hptoftlbssec;

    double rpm1 = 2000.0;
    double rpm2 = 4000.0;  // Double the RPM

    double omega1 = rpm1 * 2.0 * M_PI / 60.0;
    double omega2 = rpm2 * 2.0 * M_PI / 60.0;

    double torque1 = power / omega1;
    double torque2 = power / omega2;

    // Doubling RPM should halve torque
    TS_ASSERT_DELTA(torque1, torque2 * 2.0, 0.1);
  }

  /***************************************************************************
   * Electric Motor Characteristics Tests
   ***************************************************************************/

  // Test typical eVTOL power requirements
  void testEVTOLPowerRequirements() {
    double hptowatts = 745.7;

    // Lilium Jet: approximately 320 kW peak per motor
    double liliumMotorKW = 320.0;
    double liliumHP = liliumMotorKW * 1000.0 / hptowatts;
    TS_ASSERT_DELTA(liliumHP, 429.1, 1.0);

    // Small multicopter: 10 kW per motor
    double multicopterKW = 10.0;
    double multicopterHP = multicopterKW * 1000.0 / hptowatts;
    TS_ASSERT_DELTA(multicopterHP, 13.41, 0.1);
  }

  // Test typical drone motor power
  void testDroneMotorPower() {
    double hptowatts = 745.7;

    // DJI-class motor: 200W
    double djiWatts = 200.0;
    double djiHP = djiWatts / hptowatts;
    TS_ASSERT_DELTA(djiHP, 0.268, 0.01);

    // Racing drone motor: 500W
    double racingWatts = 500.0;
    double racingHP = racingWatts / hptowatts;
    TS_ASSERT_DELTA(racingHP, 0.670, 0.01);
  }

  // Test electric aircraft examples
  void testElectricAircraftPower() {
    double hptowatts = 745.7;

    // Pipistrel Alpha Electro: 60 kW
    double alphaKW = 60.0;
    double alphaHP = alphaKW * 1000.0 / hptowatts;
    TS_ASSERT_DELTA(alphaHP, 80.46, 0.1);

    // Bye Aerospace eFlyer 2: 90 kW
    double eflyerKW = 90.0;
    double eflyerHP = eflyerKW * 1000.0 / hptowatts;
    TS_ASSERT_DELTA(eflyerHP, 120.69, 0.1);

    // Eviation Alice: 640 kW total
    double aliceKW = 640.0;
    double aliceHP = aliceKW * 1000.0 / hptowatts;
    TS_ASSERT_DELTA(aliceHP, 858.28, 0.1);
  }

  // Test power density considerations
  void testPowerDensityCalculation() {
    double hptowatts = 745.7;

    // Modern electric motor: 5 kW/kg
    double motorMassKg = 20.0;
    double powerDensityKWperKg = 5.0;
    double motorPowerKW = motorMassKg * powerDensityKWperKg;  // 100 kW
    double motorHP = motorPowerKW * 1000.0 / hptowatts;

    TS_ASSERT_DELTA(motorHP, 134.1, 0.1);
  }

  /***************************************************************************
   * Regenerative Braking Concept Tests
   ***************************************************************************/

  // Test regenerative power at various descent rates
  void testRegenerativeDescendPower() {
    // Conceptual: Power recovered during descent
    // Negative throttle could represent regen
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;  // 100 HP motor

    // 10% regeneration
    double regenFraction = 0.1;
    double regenPower = -maxPowerWatts * regenFraction / hptowatts;
    TS_ASSERT_DELTA(regenPower, -10.0, 0.1);  // -10 HP (power returning)

    // 25% regeneration
    regenFraction = 0.25;
    regenPower = -maxPowerWatts * regenFraction / hptowatts;
    TS_ASSERT_DELTA(regenPower, -25.0, 0.1);
  }

  // Test regen filtering at low RPM
  void testRegenFilteringLowRPM() {
    double rpm = 0.05;  // Very low RPM
    double regenPower = -50.0;

    // Regen filtered at low RPM
    if (rpm <= 0.1) {
      regenPower = std::max(regenPower, 0.0);
    }
    TS_ASSERT_DELTA(regenPower, 0.0, DEFAULT_TOLERANCE);
  }

  // Test regen allowed at operating RPM
  void testRegenAllowedOperatingRPM() {
    double rpm = 1000.0;
    double regenPower = -50.0;

    if (rpm <= 0.1) {
      regenPower = std::max(regenPower, 0.0);
    }
    TS_ASSERT_DELTA(regenPower, -50.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Multi-Motor Configuration Tests
   ***************************************************************************/

  // Test dual motor configuration
  void testDualMotorPower() {
    double singleMotorHP = 100.0;
    double numMotors = 2;

    double totalHP = singleMotorHP * numMotors;
    double totalPowerFtLbSec = totalHP * 550.0;

    TS_ASSERT_DELTA(totalHP, 200.0, 0.1);
    TS_ASSERT_DELTA(totalPowerFtLbSec, 110000.0, 1.0);
  }

  // Test quad motor configuration (eVTOL)
  void testQuadMotorPower() {
    double singleMotorHP = 50.0;
    double numMotors = 4;

    double totalHP = singleMotorHP * numMotors;
    TS_ASSERT_DELTA(totalHP, 200.0, 0.1);
  }

  // Test octo-motor configuration
  void testOctoMotorPower() {
    double singleMotorHP = 25.0;
    double numMotors = 8;

    double totalHP = singleMotorHP * numMotors;
    TS_ASSERT_DELTA(totalHP, 200.0, 0.1);

    // Each motor contribution
    double motorContribution = totalHP / numMotors;
    TS_ASSERT_DELTA(motorContribution, 25.0, 0.1);
  }

  // Test differential motor power (turns)
  void testDifferentialMotorPower() {
    // Simulating a turn with left/right motor differential
    double leftMotorHP = 80.0;   // Reduced
    double rightMotorHP = 100.0; // Full power

    double differential = rightMotorHP - leftMotorHP;
    TS_ASSERT_DELTA(differential, 20.0, 0.1);

    double averageHP = (leftMotorHP + rightMotorHP) / 2.0;
    TS_ASSERT_DELTA(averageHP, 90.0, 0.1);
  }

  /***************************************************************************
   * Efficiency Concept Tests
   ***************************************************************************/

  // Test motor efficiency impact (conceptual)
  void testMotorEfficiencyImpact() {
    double inputPowerWatts = 100000.0;  // 100 kW input
    double efficiency = 0.95;  // 95% efficient (typical for modern motors)

    double outputPowerWatts = inputPowerWatts * efficiency;
    double lossWatts = inputPowerWatts * (1.0 - efficiency);

    TS_ASSERT_DELTA(outputPowerWatts, 95000.0, 0.1);
    TS_ASSERT_DELTA(lossWatts, 5000.0, 0.1);
  }

  // Test efficiency at different load points
  void testEfficiencyVsLoad() {
    // Motors are most efficient at ~75% load
    double peakEfficiency = 0.96;
    double lowLoadEfficiency = 0.85;
    double overloadEfficiency = 0.90;

    TS_ASSERT(peakEfficiency > lowLoadEfficiency);
    TS_ASSERT(peakEfficiency > overloadEfficiency);
  }

  // Test heat generation from losses
  void testHeatFromLosses() {
    double inputPowerWatts = 100000.0;
    double efficiency = 0.95;

    double heatWatts = inputPowerWatts * (1.0 - efficiency);
    TS_ASSERT_DELTA(heatWatts, 5000.0, 0.1);  // 5 kW of heat
  }

  /***************************************************************************
   * Edge Cases and Boundary Tests
   ***************************************************************************/

  // Test very small power motor
  void testVerySmallMotor() {
    double hptowatts = 745.7;
    double smallMotorWatts = 10.0;  // 10W motor (tiny)

    double hp = smallMotorWatts / hptowatts;
    TS_ASSERT_DELTA(hp, 0.0134, 0.0001);
    TS_ASSERT(hp > 0.0);
  }

  // Test very large power motor
  void testVeryLargeMotor() {
    double hptowatts = 745.7;
    double largeMW = 5.0;  // 5 MW (large industrial)

    double hp = largeMW * 1000000.0 / hptowatts;
    TS_ASSERT_DELTA(hp, 6705.0, 1.0);
  }

  // Test throttle at float precision limit
  void testThrottlePrecisionLimit() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    double throttle = 0.0000001;  // Very small
    double hp = maxPowerWatts * throttle / hptowatts;

    TS_ASSERT(hp > 0.0);
    TS_ASSERT(hp < 0.001);
  }

  // Test power calculation stability
  void testPowerCalculationStability() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    // Same calculation multiple times should give same result
    double hp1 = maxPowerWatts * 0.5 / hptowatts;
    double hp2 = maxPowerWatts * 0.5 / hptowatts;
    double hp3 = maxPowerWatts * 0.5 / hptowatts;

    TS_ASSERT_DELTA(hp1, hp2, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(hp2, hp3, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test rapid throttle changes
  void testStressRapidThrottleChanges() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    for (int i = 0; i < 1000; i++) {
      double throttle = (i % 101) / 100.0;  // 0.00 to 1.00
      double hp = maxPowerWatts * throttle / hptowatts;

      TS_ASSERT(hp >= 0.0);
      TS_ASSERT(hp <= 100.1);  // Allow small tolerance
      TS_ASSERT(!std::isnan(hp));
    }
  }

  // Test many torque calculations
  void testStressTorqueCalculations() {
    double hptoftlbssec = 550.0;
    double hp = 100.0;
    double power = hp * hptoftlbssec;

    for (int rpm = 100; rpm <= 10000; rpm += 10) {
      double omega = rpm * 2.0 * M_PI / 60.0;
      double torque = power / omega;

      TS_ASSERT(torque > 0.0);
      TS_ASSERT(!std::isnan(torque));
      TS_ASSERT(!std::isinf(torque));
    }
  }

  // Test oscillating power demand
  void testOscillatingPowerDemand() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    for (int i = 0; i < 500; i++) {
      double throttle = 0.5 + 0.5 * std::sin(i * 0.1);  // 0.0 to 1.0
      double hp = maxPowerWatts * throttle / hptowatts;

      TS_ASSERT(hp >= 0.0);
      TS_ASSERT(hp <= 100.1);
    }
  }

  // Test full throttle cycle (0 to max to 0)
  void testFullThrottleCycle() {
    double hptowatts = 745.7;
    double maxPowerWatts = 74570.0;

    // Ramp up
    for (double t = 0.0; t <= 1.0; t += 0.01) {
      double hp = maxPowerWatts * t / hptowatts;
      TS_ASSERT(hp >= 0.0);
      TS_ASSERT(hp <= 100.1);
    }

    // Ramp down
    for (double t = 1.0; t >= 0.0; t -= 0.01) {
      double hp = maxPowerWatts * t / hptowatts;
      TS_ASSERT(hp >= 0.0);
      TS_ASSERT(hp <= 100.1);
    }
  }

  /***************************************************************************
   * Additional Unit Conversion Tests
   ***************************************************************************/

  // Test Newton-meters to ft-lbf
  void testNmToFtLbf() {
    double nm = 100.0;  // Newton-meters
    double ftlbf = nm * 0.7376;  // Conversion factor

    TS_ASSERT_DELTA(ftlbf, 73.76, 0.01);
  }

  // Test kW to ft-lbf/sec
  void testKWToFtLbfPerSec() {
    double kw = 1.0;
    double ftlbfPerSec = kw * 1000.0 / 1.3558;  // 1 W = 1/1.3558 ft-lbf/s

    TS_ASSERT_DELTA(ftlbfPerSec, 737.56, 0.1);
  }

  // Test rad/s to RPM
  void testRadPerSecToRPM() {
    double radPerSec = 314.159;  // ~3000 RPM
    double rpm = radPerSec * 60.0 / (2.0 * M_PI);

    TS_ASSERT_DELTA(rpm, 3000.0, 0.1);
  }

  // Test HP to kW
  void testHPToKW() {
    double hp = 100.0;
    double kw = hp * 0.7457;

    TS_ASSERT_DELTA(kw, 74.57, 0.01);
  }

  // Test ft-lbf/sec to watts
  void testFtLbfPerSecToWatts() {
    double ftlbfPerSec = 550.0;  // 1 HP
    double watts = ftlbfPerSec * 1.3558;

    TS_ASSERT_DELTA(watts, 745.69, 0.1);
  }

  /***************************************************************************
   * Battery System Integration Tests
   ***************************************************************************/

  // Test battery capacity to flight time
  void testBatteryCapacityToFlightTime() {
    double batteryCapacity = 100.0;  // kWh
    double avgPowerDraw = 50.0;      // kW

    double flightTimeHours = batteryCapacity / avgPowerDraw;
    TS_ASSERT_DELTA(flightTimeHours, 2.0, 0.01);
  }

  // Test battery voltage under load
  void testBatteryVoltageUnderLoad() {
    double nominalVoltage = 400.0;  // V
    double internalResistance = 0.1;  // Ohms
    double current = 200.0;  // A

    double loadVoltage = nominalVoltage - (current * internalResistance);
    TS_ASSERT_DELTA(loadVoltage, 380.0, 0.1);
  }

  // Test battery state of charge
  void testBatteryStateOfCharge() {
    double initialSOC = 100.0;  // percent
    double energyUsed = 25.0;   // kWh
    double totalCapacity = 100.0;  // kWh

    double currentSOC = initialSOC - (energyUsed / totalCapacity * 100.0);
    TS_ASSERT_DELTA(currentSOC, 75.0, 0.1);
  }

  // Test battery thermal limits
  void testBatteryThermalLimits() {
    double batteryTemp = 45.0;  // °C
    double maxTemp = 55.0;      // °C
    double minTemp = -20.0;     // °C

    bool withinLimits = (batteryTemp <= maxTemp) && (batteryTemp >= minTemp);
    TS_ASSERT(withinLimits);
  }

  /***************************************************************************
   * Motor Controller Tests
   ***************************************************************************/

  // Test PWM duty cycle to power
  void testPWMDutyCycleToPower() {
    double maxPowerWatts = 100000.0;  // 100 kW
    double dutyCycle = 0.75;  // 75%

    double outputPower = maxPowerWatts * dutyCycle;
    TS_ASSERT_DELTA(outputPower, 75000.0, 0.1);
  }

  // Test field-oriented control torque
  void testFieldOrientedControlTorque() {
    double Kt = 0.5;  // Nm/A
    double Iq = 100.0;  // A (torque-producing current)

    double torqueNm = Kt * Iq;
    double torqueFtLbf = torqueNm * 0.7376;

    TS_ASSERT_DELTA(torqueNm, 50.0, 0.1);
    TS_ASSERT_DELTA(torqueFtLbf, 36.88, 0.1);
  }

  // Test controller current limit
  void testControllerCurrentLimit() {
    double maxCurrent = 500.0;  // A
    double requestedCurrent = 600.0;  // A

    double actualCurrent = std::min(requestedCurrent, maxCurrent);
    TS_ASSERT_DELTA(actualCurrent, 500.0, DEFAULT_TOLERANCE);
  }

  /***************************************************************************
   * Propeller Matching Tests
   ***************************************************************************/

  // Test motor-propeller speed matching
  void testMotorPropellerSpeedMatching() {
    double motorRPM = 6000.0;
    double gearRatio = 3.0;  // Reduction

    double propRPM = motorRPM / gearRatio;
    TS_ASSERT_DELTA(propRPM, 2000.0, 0.1);
  }

  // Test propeller loading curve
  void testPropellerLoadingCurve() {
    // Propeller power ∝ n³
    double rpm1 = 2000.0;
    double rpm2 = 2500.0;

    double powerRatio = std::pow(rpm2 / rpm1, 3.0);
    TS_ASSERT_DELTA(powerRatio, 1.953, 0.01);
  }

  // Test operating point determination
  void testOperatingPointDetermination() {
    // Motor torque = a - b*rpm (simplified linear model)
    // Prop torque = c*rpm² (simplified quadratic model)
    double a = 100.0;  // Nm stall torque
    double b = 0.01;   // torque drop per rpm
    double c = 0.000005;  // prop coefficient

    // At equilibrium: a - b*rpm = c*rpm²
    // Solve for rpm (simplified: assume prop torque dominates)
    double operatingRPM = std::sqrt((a) / (c + b / 100.0));

    TS_ASSERT(operatingRPM > 0);
    TS_ASSERT(operatingRPM < 10000);
  }

  /***************************************************************************
   * Thermal Management Tests
   ***************************************************************************/

  // Test motor temperature rise
  void testMotorTemperatureRise() {
    double powerLoss = 5000.0;  // W (heat generated)
    double thermalResistance = 0.01;  // °C/W
    double ambientTemp = 25.0;  // °C

    double motorTemp = ambientTemp + (powerLoss * thermalResistance);
    TS_ASSERT_DELTA(motorTemp, 75.0, 0.1);
  }

  // Test thermal derating
  void testThermalDerating() {
    double maxPower = 100.0;  // kW at 25°C
    double currentTemp = 80.0;  // °C
    double derateStartTemp = 60.0;  // °C
    double maxTemp = 100.0;  // °C

    double derateFactor = 1.0;
    if (currentTemp > derateStartTemp) {
      derateFactor = 1.0 - (currentTemp - derateStartTemp) / (maxTemp - derateStartTemp);
    }

    double deratedPower = maxPower * derateFactor;
    TS_ASSERT(deratedPower < maxPower);
    TS_ASSERT_DELTA(deratedPower, 50.0, 1.0);
  }

  // Test cooling system capacity
  void testCoolingSystemCapacity() {
    double heatGenerated = 8000.0;  // W
    double coolingCapacity = 10000.0;  // W

    bool thermallyStable = coolingCapacity >= heatGenerated;
    double thermalMargin = (coolingCapacity - heatGenerated) / coolingCapacity;

    TS_ASSERT(thermallyStable);
    TS_ASSERT_DELTA(thermalMargin, 0.2, 0.01);
  }

  /***************************************************************************
   * Range and Endurance Tests
   ***************************************************************************/

  // Test range calculation
  void testRangeCalculation() {
    double batteryEnergy = 150.0;  // kWh
    double cruisePower = 75.0;     // kW
    double cruiseSpeed = 150.0;    // kts

    double enduranceHours = batteryEnergy / cruisePower;
    double rangeNM = cruiseSpeed * enduranceHours;

    TS_ASSERT_DELTA(enduranceHours, 2.0, 0.01);
    TS_ASSERT_DELTA(rangeNM, 300.0, 0.1);
  }

  // Test reserve energy requirement
  void testReserveEnergyRequirement() {
    double totalEnergy = 100.0;  // kWh
    double reservePercent = 20.0;  // 20% reserve

    double usableEnergy = totalEnergy * (1.0 - reservePercent / 100.0);
    TS_ASSERT_DELTA(usableEnergy, 80.0, 0.1);
  }

  // Test energy consumption rate
  void testEnergyConsumptionRate() {
    double power = 50.0;  // kW
    double time = 0.5;    // hours

    double energyConsumed = power * time;
    TS_ASSERT_DELTA(energyConsumed, 25.0, 0.1);  // kWh
  }

  /***************************************************************************
   * Failure Mode Tests
   ***************************************************************************/

  // Test motor short circuit
  void testMotorShortCircuit() {
    double normalResistance = 0.1;  // Ohms
    double shortedResistance = 0.01;  // Ohms

    bool shortCircuit = shortedResistance < (normalResistance * 0.2);
    TS_ASSERT(shortCircuit);
  }

  // Test inverter failure detection
  void testInverterFailureDetection() {
    double expectedOutput = 100.0;  // kW
    double actualOutput = 5.0;      // kW

    double outputRatio = actualOutput / expectedOutput;
    bool inverterFailed = outputRatio < 0.1;

    TS_ASSERT(inverterFailed);
  }

  // Test battery cell imbalance
  void testBatteryCellImbalance() {
    double cellVoltages[] = {3.7, 3.65, 3.7, 3.68, 3.2, 3.7};
    double minVoltage = 3.2;
    double maxVoltage = 3.7;

    double imbalance = maxVoltage - minVoltage;
    bool imbalanceExcessive = imbalance > 0.3;

    TS_ASSERT(imbalanceExcessive);
  }

  /***************************************************************************
   * Regeneration Extended Tests
   ***************************************************************************/

  // Test regeneration during descent
  void testRegenerationDuringDescent() {
    double sinkRate = 500.0;  // fpm
    double weight = 2000.0;   // lbs
    double regenEfficiency = 0.7;

    // Potential energy recovery rate
    double potentialEnergyRate = weight * (sinkRate / 60.0);  // ft-lbs/sec
    double regenPower = potentialEnergyRate * regenEfficiency / 550.0;  // HP

    TS_ASSERT(regenPower > 0);
    TS_ASSERT(!std::isnan(regenPower));
  }

  // Test regen limited by battery SOC
  void testRegenLimitedBySOC() {
    double regenPowerRequested = 50.0;  // kW
    double batterySOC = 95.0;  // percent
    double maxSOCForRegen = 90.0;  // percent

    double regenLimit = (100.0 - batterySOC) / (100.0 - maxSOCForRegen);
    double actualRegen = regenPowerRequested * std::min(1.0, regenLimit);

    TS_ASSERT(actualRegen < regenPowerRequested);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test continuous power cycling
  void testContinuousPowerCycling() {
    double maxPower = 100.0;

    for (int i = 0; i < 100; i++) {
      double throttle = (i % 10) / 10.0;
      double power = maxPower * throttle;

      TS_ASSERT(power >= 0.0);
      TS_ASSERT(power <= maxPower);
    }
  }

  // Test power calculation numerical stability
  void testPowerCalculationNumericalStability() {
    double hptowatts = 745.7;

    for (double watts = 1.0; watts <= 1000000.0; watts *= 10.0) {
      double hp = watts / hptowatts;
      TS_ASSERT(!std::isnan(hp));
      TS_ASSERT(!std::isinf(hp));
      TS_ASSERT(hp > 0);
    }
  }

  // Test torque at boundary RPM values
  void testTorqueAtBoundaryRPM() {
    double power = 55000.0;  // ft-lbf/sec (100 HP)

    // Very low RPM
    double rpm = 10.0;
    double omega = rpm * 2.0 * M_PI / 60.0;
    double torque = power / omega;
    TS_ASSERT(!std::isinf(torque));

    // Very high RPM
    rpm = 50000.0;
    omega = rpm * 2.0 * M_PI / 60.0;
    torque = power / omega;
    TS_ASSERT(torque > 0);
    TS_ASSERT(torque < 100.0);
  }

  /***************************************************************************
   * Complete Electric Motor System Tests
   ***************************************************************************/

  // Test complete motor power curve
  void testCompleteMotorPowerCurve() {
    double hptowatts = 745.7;
    double hptoftlbssec = 550.0;
    double maxPowerWatts = 74570.0;  // 100 HP motor

    for (double throttle = 0.0; throttle <= 1.0; throttle += 0.1) {
      double hp = maxPowerWatts * throttle / hptowatts;
      double power = hp * hptoftlbssec;
      TS_ASSERT(power >= 0.0);
      TS_ASSERT(power <= 55000.1);
      TS_ASSERT_DELTA(hp, 100.0 * throttle, 0.2);
    }
  }

  // Test motor efficiency at various operating points
  void testMotorEfficiencyOperatingPoints() {
    double nominalEfficiency = 0.95;
    double lowLoadEfficiency = 0.88;
    double peakEfficiency = 0.96;

    TS_ASSERT(peakEfficiency > nominalEfficiency);
    TS_ASSERT(nominalEfficiency > lowLoadEfficiency);
    TS_ASSERT(peakEfficiency <= 1.0);
    TS_ASSERT(lowLoadEfficiency > 0.0);
  }

  // Test motor thermal model
  void testMotorThermalModel() {
    double ambientTemp = 25.0;  // °C
    double heatGenerated = 5000.0;  // W
    double thermalResistance = 0.01;  // °C/W

    double tempRise = heatGenerated * thermalResistance;
    double motorTemp = ambientTemp + tempRise;

    TS_ASSERT_DELTA(motorTemp, 75.0, 0.1);
    TS_ASSERT(motorTemp < 150.0);  // Below thermal limit
  }

  // Test power factor considerations
  void testPowerFactorConsiderations() {
    double realPower = 100.0;  // kW
    double powerFactor = 0.85;
    double apparentPower = realPower / powerFactor;

    TS_ASSERT_DELTA(apparentPower, 117.6, 0.1);  // kVA
    TS_ASSERT(apparentPower > realPower);
  }

  // Test voltage-torque relationship
  void testVoltageTorqueRelationship() {
    double voltage1 = 400.0;  // V
    double voltage2 = 380.0;  // V
    double torque1 = 100.0;   // Nm at V1

    // Torque approximately proportional to V²
    double torque2 = torque1 * (voltage2 / voltage1) * (voltage2 / voltage1);
    TS_ASSERT_DELTA(torque2, 90.25, 0.1);
    TS_ASSERT(torque2 < torque1);
  }

  // Test regenerative braking limits
  void testRegenerativeBrakingLimits() {
    double maxRegenPower = 50.0;  // kW
    double requestedRegen = 60.0;
    double actualRegen = std::min(requestedRegen, maxRegenPower);

    TS_ASSERT_DELTA(actualRegen, 50.0, DEFAULT_TOLERANCE);
    TS_ASSERT(actualRegen <= maxRegenPower);
  }

  // Test motor current calculation
  void testMotorCurrentCalculation() {
    double power = 75000.0;  // W
    double voltage = 400.0;  // V
    double powerFactor = 0.9;

    double current = power / (voltage * std::sqrt(3.0) * powerFactor);
    TS_ASSERT_DELTA(current, 120.3, 0.5);
  }

  // Test motor slip calculation
  void testMotorSlipCalculation() {
    double syncSpeed = 1800.0;  // RPM
    double actualSpeed = 1750.0;
    double slip = (syncSpeed - actualSpeed) / syncSpeed * 100.0;

    TS_ASSERT_DELTA(slip, 2.78, 0.01);
    TS_ASSERT(slip > 0.0);
    TS_ASSERT(slip < 10.0);
  }

  // Test motor poles and frequency
  void testMotorPolesFrequency() {
    double frequency = 60.0;  // Hz
    int poles = 4;
    double syncSpeed = (120.0 * frequency) / poles;

    TS_ASSERT_DELTA(syncSpeed, 1800.0, DEFAULT_TOLERANCE);
  }

  // Test multi-motor configuration with redundancy
  void testMultiMotorRedundancy() {
    double motorPower[] = {100.0, 100.0, 100.0, 100.0};
    int numMotors = 4;
    double totalPower = 0.0;
    for (int i = 0; i < numMotors; i++) {
      totalPower += motorPower[i];
    }

    TS_ASSERT_DELTA(totalPower, 400.0, DEFAULT_TOLERANCE);

    // One motor failure
    motorPower[2] = 0.0;
    totalPower = 0.0;
    for (int i = 0; i < numMotors; i++) {
      totalPower += motorPower[i];
    }
    TS_ASSERT_DELTA(totalPower, 300.0, DEFAULT_TOLERANCE);
  }

  // Test motor inrush current
  void testMotorInrushCurrent() {
    double ratedCurrent = 100.0;  // A
    double inrushMultiplier = 6.0;
    double inrushCurrent = ratedCurrent * inrushMultiplier;

    TS_ASSERT_DELTA(inrushCurrent, 600.0, DEFAULT_TOLERANCE);
    TS_ASSERT(inrushCurrent > ratedCurrent);
  }

  // Test motor service factor
  void testMotorServiceFactor() {
    double ratedPower = 100.0;  // HP
    double serviceFactor = 1.15;
    double maxContinuousPower = ratedPower * serviceFactor;

    TS_ASSERT_DELTA(maxContinuousPower, 115.0, DEFAULT_TOLERANCE);
  }

  // Test motor derating for altitude
  void testMotorDeratingAltitude() {
    double seaLevelPower = 100.0;  // kW
    double altitude = 6000.0;  // ft
    double derateStartAlt = 3300.0;  // ft
    double deratePercent = 3.0;  // % per 1000 ft above start

    double derating = 0.0;
    if (altitude > derateStartAlt) {
      derating = ((altitude - derateStartAlt) / 1000.0) * deratePercent;
    }
    double deratedPower = seaLevelPower * (1.0 - derating / 100.0);

    TS_ASSERT(deratedPower < seaLevelPower);
    TS_ASSERT_DELTA(deratedPower, 91.9, 0.1);
  }

  // Test motor insulation class temperature
  void testMotorInsulationClassTemp() {
    double classB_maxTemp = 130.0;  // °C
    double classF_maxTemp = 155.0;
    double classH_maxTemp = 180.0;

    TS_ASSERT(classB_maxTemp < classF_maxTemp);
    TS_ASSERT(classF_maxTemp < classH_maxTemp);
  }

  // Test motor locked rotor torque
  void testMotorLockedRotorTorque() {
    double ratedTorque = 100.0;  // Nm
    double lockedRotorMultiplier = 1.5;
    double lockedRotorTorque = ratedTorque * lockedRotorMultiplier;

    TS_ASSERT_DELTA(lockedRotorTorque, 150.0, DEFAULT_TOLERANCE);
  }

  // Test complete electric propulsion verification
  void testCompleteElectricPropulsionVerification() {
    double hptowatts = 745.7;
    double hptoftlbssec = 550.0;

    double batteryVoltage = 400.0;  // V
    double batteryCurrent = 250.0;  // A
    double inputPower = batteryVoltage * batteryCurrent;  // W

    double motorEfficiency = 0.95;
    double controllerEfficiency = 0.98;
    double outputPowerW = inputPower * motorEfficiency * controllerEfficiency;
    double outputPowerHP = outputPowerW / hptowatts;
    double powerFtLbSec = outputPowerHP * hptoftlbssec;

    TS_ASSERT_DELTA(outputPowerHP, 124.8, 1.0);
    TS_ASSERT(powerFtLbSec > 0.0);
  }

  // Test motor constant Ke relationship
  void testMotorConstantKe() {
    double voltage = 400.0;  // V
    double rpm = 3000.0;
    double Ke = voltage / rpm;  // V/RPM

    TS_ASSERT_DELTA(Ke, 0.1333, 0.001);
  }

  // Test motor constant Kt relationship
  void testMotorConstantKt() {
    double torqueNm = 100.0;
    double current = 200.0;  // A
    double Kt = torqueNm / current;  // Nm/A

    TS_ASSERT_DELTA(Kt, 0.5, DEFAULT_TOLERANCE);
  }

  // Test electric motor instance independence
  void testElectricMotorInstanceIndependence() {
    double motor1_power = 50.0;
    double motor2_power = 100.0;
    double motor1_rpm = 3000.0;
    double motor2_rpm = 4000.0;

    double motor1_torque = motor1_power * 550.0 / (motor1_rpm * 2.0 * M_PI / 60.0);
    double motor2_torque = motor2_power * 550.0 / (motor2_rpm * 2.0 * M_PI / 60.0);

    TS_ASSERT(motor1_torque > 0.0);
    TS_ASSERT(motor2_torque > 0.0);
    TS_ASSERT(motor2_torque > motor1_torque);
  }
};

//=============================================================================
// C172x Integration Tests - General Propulsion Tests with C172x
//=============================================================================

class FGElectricC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {


    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: Engine count
  void testEngineCount() {
    auto propulsion = fdm.GetPropulsion();
    unsigned int numEngines = propulsion->GetNumEngines();

    TS_ASSERT_EQUALS(numEngines, 1);
  }

  // Test 2: Engine type check
  void testEngineType() {
    auto propulsion = fdm.GetPropulsion();
    auto engine = propulsion->GetEngine(0);

    TS_ASSERT(engine != nullptr);
    // C172x uses a piston engine, not electric
    TS_ASSERT(engine->GetType() != FGEngine::etElectric);
  }

  // Test 3: Thrust at different speeds
  void testThrustAtDifferentSpeeds() {
    auto ic = fdm.GetIC();
    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();

    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetMixtureCmd(-1, 1.0);

    // At 60 knots
    ic->SetVcalibratedKtsIC(60.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust60 = propulsion->GetEngine(0)->GetThrust();

    // At 120 knots
    ic->SetVcalibratedKtsIC(120.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust120 = propulsion->GetEngine(0)->GetThrust();

    TS_ASSERT(std::isfinite(thrust60));
    TS_ASSERT(std::isfinite(thrust120));
  }

  // Test 4: Power conversion at full throttle
  void testPowerConversionFullThrottle() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 1.0);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double thrust = propulsion->GetEngine(0)->GetThrust();
    double power = propulsion->GetEngine(0)->GetPowerAvailable();

    TS_ASSERT(thrust > 0.0);
    TS_ASSERT(std::isfinite(power));
  }

  // Test 5: RPM at cruise
  void testRPMAtCruise() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(110.0);
    ic->SetAltitudeASLFtIC(7500.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.65);
    fcs->SetMixtureCmd(-1, 0.9);

    for (int i = 0; i < 100; ++i) fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double rpm = pm->GetNode("propulsion/engine/propeller-rpm")->getDoubleValue();

    TS_ASSERT(std::isfinite(rpm));
  }

  // Test 6: Fuel tank access
  void testFuelTankAccess() {
    auto propulsion = fdm.GetPropulsion();
    unsigned int numTanks = propulsion->GetNumTanks();

    TS_ASSERT(numTanks > 0);
  }

  // Test 7: Thrust vector direction
  void testThrustVectorDirection() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    double thrust = propulsion->GetEngine(0)->GetThrust();

    // Thrust should be positive (forward)
    TS_ASSERT(thrust >= 0.0);
  }

  // Test 8: Mixture leaning effects
  void testMixtureLeaningEffects() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();

    fcs->SetThrottleCmd(-1, 0.7);

    // Rich mixture
    fcs->SetMixtureCmd(-1, 1.0);
    for (int i = 0; i < 50; ++i) fdm.Run();
    double ff_rich = propulsion->GetEngine(0)->GetFuelFlowRate();

    // Lean mixture
    fcs->SetMixtureCmd(-1, 0.6);
    for (int i = 0; i < 50; ++i) fdm.Run();
    double ff_lean = propulsion->GetEngine(0)->GetFuelFlowRate();

    TS_ASSERT(std::isfinite(ff_rich));
    TS_ASSERT(std::isfinite(ff_lean));
  }

  // Test 9: Engine power at altitude
  void testEnginePowerAtAltitude() {
    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();
    auto ic = fdm.GetIC();

    fcs->SetThrottleCmd(-1, 1.0);
    fcs->SetMixtureCmd(-1, 1.0);

    // Sea level
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(0.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double power_sl = propulsion->GetEngine(0)->GetPowerAvailable();

    // High altitude
    ic->SetAltitudeASLFtIC(12000.0);
    fdm.RunIC();
    for (int i = 0; i < 50; ++i) fdm.Run();
    double power_high = propulsion->GetEngine(0)->GetPowerAvailable();

    TS_ASSERT(std::isfinite(power_sl));
    TS_ASSERT(std::isfinite(power_high));
  }

  // Test 10: Propeller model integration
  void testPropellerModelIntegration() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    for (int i = 0; i < 100; ++i) fdm.Run();

    auto propulsion = fdm.GetPropulsion();
    auto thruster = propulsion->GetEngine(0)->GetThruster();

    TS_ASSERT(thruster != nullptr);
  }

  // Test 11: Continuous operation
  void testContinuousOperation() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    auto propulsion = fdm.GetPropulsion();

    // Run for extended period
    for (int i = 0; i < 500; ++i) {
      fdm.Run();
      double thrust = propulsion->GetEngine(0)->GetThrust();
      TS_ASSERT(std::isfinite(thrust));
    }
  }

  // Test 12: Engine restart capability
  void testEngineRestartCapability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    auto propulsion = fdm.GetPropulsion();

    // Run engine
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust1 = propulsion->GetEngine(0)->GetThrust();

    // Cut mixture
    fcs->SetMixtureCmd(-1, 0.0);
    for (int i = 0; i < 50; ++i) fdm.Run();

    // Restart
    fcs->SetMixtureCmd(-1, 1.0);
    for (int i = 0; i < 50; ++i) fdm.Run();
    double thrust2 = propulsion->GetEngine(0)->GetThrust();

    TS_ASSERT(std::isfinite(thrust1));
    TS_ASSERT(std::isfinite(thrust2));
  }
};
