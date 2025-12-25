/*******************************************************************************
 * FGFlightPathTest.h - Unit tests for flight path calculations
 *
 * Tests the mathematical behavior of flight path computations:
 * - Flight path angle (gamma)
 * - Ground track calculations
 * - Range and endurance
 * - Performance parameters
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

const double epsilon = 1e-10;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double G = 32.174;  // ft/s^2
const double KTS_TO_FPS = 1.68781;

class FGFlightPathTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Flight Path Angle Tests
   ***************************************************************************/

  // Test flight path angle from velocity components
  void testFlightPathAngle() {
    double Vx = 200.0;  // Horizontal velocity (ft/s)
    double Vz = -20.0;  // Vertical velocity (ft/s, down positive)

    double gamma = std::atan2(-Vz, Vx) * RAD_TO_DEG;
    TS_ASSERT_DELTA(gamma, 5.71, 0.1);  // Climbing
  }

  // Test level flight gamma
  void testLevelFlightGamma() {
    double Vx = 200.0;
    double Vz = 0.0;

    double gamma = std::atan2(-Vz, Vx) * RAD_TO_DEG;
    TS_ASSERT_DELTA(gamma, 0.0, epsilon);
  }

  // Test descending gamma
  void testDescendingGamma() {
    double Vx = 200.0;
    double Vz = 20.0;  // Descending

    double gamma = std::atan2(-Vz, Vx) * RAD_TO_DEG;
    TS_ASSERT_DELTA(gamma, -5.71, 0.1);
  }

  // Test gamma from climb rate and airspeed
  void testGammaFromClimbRate() {
    double climbRate = 1000.0 / 60.0;  // 1000 ft/min in ft/s
    double TAS = 150.0 * KTS_TO_FPS;   // 150 kts in ft/s

    double gamma = std::asin(climbRate / TAS) * RAD_TO_DEG;
    TS_ASSERT_DELTA(gamma, 3.78, 0.1);
  }

  /***************************************************************************
   * Ground Track Tests
   ***************************************************************************/

  // Test ground speed from TAS and wind
  void testGroundSpeed() {
    double TAS = 200.0;      // ft/s
    double headwind = 30.0;  // ft/s

    double groundSpeed = TAS - headwind;
    TS_ASSERT_DELTA(groundSpeed, 170.0, epsilon);
  }

  // Test ground speed with crosswind
  void testGroundSpeedCrosswind() {
    double TAS = 200.0;
    double headwind = 20.0;
    double crosswind = 30.0;

    // Simplified (assumes small angles)
    double groundSpeed = std::sqrt((TAS - headwind) * (TAS - headwind) -
                                    crosswind * crosswind);
    TS_ASSERT(groundSpeed < TAS - headwind);
  }

  // Test ground track angle (heading + drift)
  void testGroundTrack() {
    double heading = 90.0;   // Heading east
    double drift = 5.0;      // Wind drift angle

    double track = heading + drift;
    TS_ASSERT_DELTA(track, 95.0, epsilon);
  }

  // Test drift angle calculation
  void testDriftAngle() {
    double crosswind = 20.0;  // ft/s
    double TAS = 200.0;       // ft/s

    double drift = std::asin(crosswind / TAS) * RAD_TO_DEG;
    TS_ASSERT_DELTA(drift, 5.74, 0.1);
  }

  /***************************************************************************
   * Range Calculations
   ***************************************************************************/

  // Test still air range
  void testStillAirRange() {
    double fuel = 1000.0;       // lbs
    double fuelFlow = 200.0;    // lbs/hr
    double groundSpeed = 200.0; // kts

    double endurance = fuel / fuelFlow;  // hours
    double range = groundSpeed * endurance;  // nm

    TS_ASSERT_DELTA(range, 1000.0, epsilon);
  }

  // Test range with headwind
  void testRangeHeadwind() {
    double fuel = 1000.0;
    double fuelFlow = 200.0;
    double TAS = 200.0;
    double headwind = 40.0;  // kts

    double groundSpeed = TAS - headwind;
    double endurance = fuel / fuelFlow;
    double range = groundSpeed * endurance;

    TS_ASSERT_DELTA(range, 800.0, epsilon);  // Reduced range
  }

  // Test range with tailwind
  void testRangeTailwind() {
    double fuel = 1000.0;
    double fuelFlow = 200.0;
    double TAS = 200.0;
    double tailwind = 40.0;  // kts

    double groundSpeed = TAS + tailwind;
    double endurance = fuel / fuelFlow;
    double range = groundSpeed * endurance;

    TS_ASSERT_DELTA(range, 1200.0, epsilon);  // Increased range
  }

  /***************************************************************************
   * Endurance Calculations
   ***************************************************************************/

  // Test endurance from fuel and flow
  void testEndurance() {
    double fuel = 500.0;      // lbs
    double fuelFlow = 100.0;  // lbs/hr

    double endurance = fuel / fuelFlow;
    TS_ASSERT_DELTA(endurance, 5.0, epsilon);  // hours
  }

  // Test endurance in minutes
  void testEnduranceMinutes() {
    double fuel = 500.0;
    double fuelFlow = 100.0;

    double enduranceHrs = fuel / fuelFlow;
    double enduranceMin = enduranceHrs * 60.0;

    TS_ASSERT_DELTA(enduranceMin, 300.0, epsilon);
  }

  /***************************************************************************
   * Specific Range/Endurance Tests
   ***************************************************************************/

  // Test specific range (nm/lb)
  void testSpecificRange() {
    double groundSpeed = 200.0;  // kts
    double fuelFlow = 200.0;     // lbs/hr

    double specificRange = groundSpeed / fuelFlow;
    TS_ASSERT_DELTA(specificRange, 1.0, epsilon);  // nm/lb
  }

  // Test specific endurance (hr/lb)
  void testSpecificEndurance() {
    double fuelFlow = 100.0;  // lbs/hr

    double specificEndurance = 1.0 / fuelFlow;
    TS_ASSERT_DELTA(specificEndurance, 0.01, epsilon);  // hr/lb
  }

  /***************************************************************************
   * Rate of Climb Tests
   ***************************************************************************/

  // Test rate of climb from excess thrust
  void testROCFromExcessThrust() {
    double thrust = 2000.0;   // lbs
    double drag = 1000.0;     // lbs
    double weight = 10000.0;  // lbs
    double V = 200.0;         // ft/s

    // Excess power = (T - D) * V
    // ROC = excess power / W
    double excessPower = (thrust - drag) * V;
    double ROC = excessPower / weight;

    TS_ASSERT_DELTA(ROC, 20.0, epsilon);  // ft/s
  }

  // Test ROC in ft/min
  void testROCFtPerMin() {
    double ROC_fps = 20.0;  // ft/s
    double ROC_fpm = ROC_fps * 60.0;

    TS_ASSERT_DELTA(ROC_fpm, 1200.0, epsilon);
  }

  // Test service ceiling (ROC = 100 ft/min)
  void testServiceCeilingConcept() {
    double serviceCeilingROC = 100.0;  // ft/min
    double currentROC = 500.0;         // ft/min at current altitude

    bool atServiceCeiling = (currentROC <= serviceCeilingROC);
    TS_ASSERT(!atServiceCeiling);
  }

  /***************************************************************************
   * Glide Performance Tests
   ***************************************************************************/

  // Test best glide ratio
  void testBestGlideRatio() {
    double LD_max = 15.0;  // Max L/D ratio

    // Glide ratio = L/D for power-off glide
    TS_ASSERT_DELTA(LD_max, 15.0, epsilon);
  }

  // Test glide distance
  void testGlideDistance() {
    double altitude = 10000.0;  // ft AGL
    double glideRatio = 15.0;

    double glideDistance = altitude * glideRatio;
    TS_ASSERT_DELTA(glideDistance, 150000.0, epsilon);  // ft
  }

  // Test glide distance in nm
  void testGlideDistanceNM() {
    double altitude = 10000.0;
    double glideRatio = 15.0;

    double glideDistanceFt = altitude * glideRatio;
    double glideDistanceNM = glideDistanceFt / 6076.12;

    TS_ASSERT_DELTA(glideDistanceNM, 24.69, 0.1);
  }

  // Test sink rate at best glide
  void testSinkRate() {
    double V = 150.0;      // ft/s (best glide speed)
    double glideRatio = 15.0;

    double sinkRate = V / glideRatio;
    TS_ASSERT_DELTA(sinkRate, 10.0, epsilon);  // ft/s
  }

  /***************************************************************************
   * Energy Height Tests
   ***************************************************************************/

  // Test specific energy (energy height)
  void testEnergyHeight() {
    double altitude = 10000.0;  // ft
    double V = 300.0;           // ft/s

    // Energy height = h + V²/(2g)
    double energyHeight = altitude + V * V / (2.0 * G);
    TS_ASSERT_DELTA(energyHeight, 11398.0, 10.0);
  }

  // Test energy trade-off (zoom climb)
  void testEnergyTradeOff() {
    double h1 = 10000.0;
    double V1 = 400.0;
    double V2 = 300.0;  // After zoom

    // Kinetic energy lost = potential energy gained (ignoring drag)
    double KE_lost = (V1 * V1 - V2 * V2) / (2.0 * G);
    double h2 = h1 + KE_lost;

    TS_ASSERT(h2 > h1);
  }

  /***************************************************************************
   * Turn Performance Tests
   ***************************************************************************/

  // Test minimum turn radius
  void testMinTurnRadius() {
    double V = 200.0;         // ft/s
    double n_max = 3.0;       // Max load factor
    double phi_max = std::acos(1.0 / n_max);

    double R_min = V * V / (G * std::tan(phi_max));
    TS_ASSERT(R_min > 0);
  }

  // Test time to turn 180 degrees
  void testTimeToTurn180() {
    double V = 200.0;
    double phi = 45.0 * DEG_TO_RAD;

    double omega = G * std::tan(phi) / V;  // rad/s
    double time180 = M_PI / omega;

    TS_ASSERT_DELTA(time180, 19.57, 0.5);  // seconds
  }

  /***************************************************************************
   * Speed Definitions Tests
   ***************************************************************************/

  // Test IAS to TAS conversion
  void testIASToTAS() {
    double IAS = 200.0;       // kts
    double rho = 0.001756;    // slugs/ft³ at 10000 ft
    double rho0 = 0.002377;   // Sea level density

    double TAS = IAS * std::sqrt(rho0 / rho);
    TS_ASSERT_DELTA(TAS, 232.6, 0.5);  // kts
  }

  // Test Mach number
  void testMachNumber() {
    double TAS = 500.0;   // ft/s
    double a = 1000.0;    // Speed of sound (ft/s)

    double Mach = TAS / a;
    TS_ASSERT_DELTA(Mach, 0.5, epsilon);
  }

  // Test dynamic pressure
  void testDynamicPressure() {
    double rho = 0.002377;
    double V = 200.0;

    double q = 0.5 * rho * V * V;
    TS_ASSERT_DELTA(q, 47.54, 0.1);  // lb/ft²
  }

  /***************************************************************************
   * Distance/Time Calculations
   ***************************************************************************/

  // Test time to travel distance
  void testTimeToTravel() {
    double distance = 100.0;     // nm
    double groundSpeed = 200.0;  // kts

    double time = distance / groundSpeed;  // hours
    double timeMin = time * 60.0;

    TS_ASSERT_DELTA(timeMin, 30.0, epsilon);
  }

  // Test distance in given time
  void testDistanceInTime() {
    double time = 0.5;           // hours
    double groundSpeed = 200.0;  // kts

    double distance = groundSpeed * time;
    TS_ASSERT_DELTA(distance, 100.0, epsilon);  // nm
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test vertical climb (gamma = 90)
  void testVerticalClimb() {
    double Vx = 0.0;
    double Vz = -100.0;  // Climbing vertically

    // Need to handle Vx = 0 case
    double gamma;
    if (std::abs(Vx) < 0.001) {
      gamma = (Vz < 0) ? 90.0 : -90.0;
    } else {
      gamma = std::atan2(-Vz, Vx) * RAD_TO_DEG;
    }

    TS_ASSERT_DELTA(gamma, 90.0, epsilon);
  }

  // Test zero ground speed
  void testZeroGroundSpeed() {
    double TAS = 50.0;
    double headwind = 50.0;

    double groundSpeed = TAS - headwind;
    TS_ASSERT_DELTA(groundSpeed, 0.0, epsilon);  // Hovering
  }

  // Test negative ground speed (flying backwards)
  void testNegativeGroundSpeed() {
    double TAS = 40.0;
    double headwind = 60.0;

    double groundSpeed = TAS - headwind;
    TS_ASSERT(groundSpeed < 0);  // Flying backwards relative to ground
  }
};
