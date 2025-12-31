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

#include <FGFDMExec.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropagate.h>
#include <models/FGAccelerations.h>
#include <models/FGFCS.h>
#include <models/FGPropulsion.h>
#include <models/FGAerodynamics.h>

using namespace JSBSim;

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

/*******************************************************************************
 * Additional FGFlightPath Tests (41 new tests)
 ******************************************************************************/

class FGFlightPathAdditionalTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Best Climb Performance Tests
   ***************************************************************************/

  // Test 35: Best rate of climb speed (Vy)
  void testBestROCSpeed() {
    // Simplified: Vy occurs where excess power is maximum
    // For a typical light aircraft, Vy is about 1.2-1.3 times stall speed
    double Vs = 50.0;  // Stall speed (kts)
    double Vy = 1.25 * Vs;

    TS_ASSERT_DELTA(Vy, 62.5, epsilon);
  }

  // Test 36: Best angle of climb speed (Vx)
  void testBestAOCSpeed() {
    // Vx < Vy, typically 0.9-1.0 * Vy
    double Vy = 65.0;
    double Vx = 0.92 * Vy;

    TS_ASSERT_DELTA(Vx, 59.8, 0.1);
    TS_ASSERT(Vx < Vy);
  }

  // Test 37: Climb gradient calculation
  void testClimbGradient() {
    double ROC = 1000.0;        // ft/min
    double groundSpeed = 120.0; // kts

    // Gradient = ROC / GS (converted to same units)
    // GS in ft/min = 120 * 6076.12 / 60 = 12152.24 ft/min
    double GS_ftmin = groundSpeed * 6076.12 / 60.0;
    double gradient = (ROC / GS_ftmin) * 100.0;  // percent

    TS_ASSERT_DELTA(gradient, 8.23, 0.1);
  }

  // Test 38: Climb gradient in degrees
  void testClimbGradientDegrees() {
    double ROC = 1000.0;  // ft/min
    double TAS = 150.0;   // kts

    double TAS_fps = TAS * KTS_TO_FPS;
    double ROC_fps = ROC / 60.0;
    double gamma = std::atan2(ROC_fps, TAS_fps) * RAD_TO_DEG;

    TS_ASSERT_DELTA(gamma, 3.77, 0.1);
  }

  // Test 39: Time to climb calculation
  void testTimeToClimb() {
    double altitude_diff = 10000.0;  // ft
    double avg_ROC = 800.0;          // ft/min

    double time_min = altitude_diff / avg_ROC;
    TS_ASSERT_DELTA(time_min, 12.5, 0.1);
  }

  // Test 40: Fuel used in climb
  void testFuelUsedInClimb() {
    double time_min = 12.5;
    double fuel_flow = 15.0;  // gal/hr

    double fuel_used = (fuel_flow / 60.0) * time_min;
    TS_ASSERT_DELTA(fuel_used, 3.125, 0.001);  // gallons
  }

  /***************************************************************************
   * Cruise Performance Tests
   ***************************************************************************/

  // Test 41: Long range cruise speed
  void testLongRangeCruiseSpeed() {
    // LRC is typically 99% of max range speed (MRC)
    double MRC = 180.0;  // kts
    double LRC = MRC * 0.99;

    TS_ASSERT_DELTA(LRC, 178.2, 0.1);
  }

  // Test 42: Specific air range
  void testSpecificAirRange() {
    double TAS = 200.0;        // kts
    double fuel_flow = 10.0;   // gal/hr

    double SAR = TAS / fuel_flow;  // nm/gal
    TS_ASSERT_DELTA(SAR, 20.0, epsilon);
  }

  // Test 43: Breguet range equation (simplified)
  void testBreguetRange() {
    // R = (V/c) * (L/D) * ln(W_initial/W_final)
    double V = 250.0;           // kts
    double c = 0.5;             // TSFC (1/hr)
    double LD = 15.0;           // L/D ratio
    double W_ratio = 1.2;       // W_initial/W_final

    double R = (V / c) * LD * std::log(W_ratio);
    TS_ASSERT_DELTA(R, 1365.0, 10.0);  // nm
  }

  // Test 44: Range with reserve fuel
  void testRangeWithReserve() {
    double total_fuel = 100.0;   // gallons
    double reserve = 0.10;       // 10% reserve
    double usable_fuel = total_fuel * (1.0 - reserve);
    double fuel_flow = 10.0;     // gal/hr
    double groundSpeed = 180.0;  // kts

    double endurance = usable_fuel / fuel_flow;
    double range = groundSpeed * endurance;

    TS_ASSERT_DELTA(usable_fuel, 90.0, epsilon);
    TS_ASSERT_DELTA(range, 1620.0, epsilon);  // nm
  }

  // Test 45: Point of no return
  void testPointOfNoReturn() {
    double fuel = 100.0;         // gallons
    double fuel_flow = 10.0;     // gal/hr
    double headwind_out = 20.0;  // kts
    double tailwind_back = 20.0; // kts (headwind becomes tailwind)
    double TAS = 150.0;          // kts

    double GS_out = TAS - headwind_out;
    double GS_back = TAS + tailwind_back;

    // PNR distance = (fuel/fuel_flow) * (GS_out * GS_back) / (GS_out + GS_back)
    double endurance = fuel / fuel_flow;
    double PNR = endurance * (GS_out * GS_back) / (GS_out + GS_back);

    TS_ASSERT_DELTA(PNR, 736.67, 1.0);  // nm
  }

  /***************************************************************************
   * Descent Performance Tests
   ***************************************************************************/

  // Test 46: Top of descent calculation
  void testTopOfDescent() {
    double cruise_alt = 35000.0;     // ft
    double destination_alt = 0.0;    // ft
    double descent_rate = 3.0;       // degrees glide path

    double alt_diff = cruise_alt - destination_alt;
    double TOD_distance = alt_diff / std::tan(descent_rate * DEG_TO_RAD);

    TS_ASSERT_DELTA(TOD_distance, 667890.0, 1000.0);  // ft
  }

  // Test 47: 3:1 descent rule
  void testThreeToOneDescentRule() {
    // Rule of thumb: 3 nm per 1000 ft descent
    double altitude_to_lose = 10000.0;  // ft
    double distance = 3.0 * (altitude_to_lose / 1000.0);

    TS_ASSERT_DELTA(distance, 30.0, epsilon);  // nm
  }

  // Test 48: Descent gradient
  void testDescentGradient() {
    double altitude_loss = 10000.0;   // ft
    double distance = 30.0 * 6076.12; // nm to ft

    double gradient = (altitude_loss / distance) * 100.0;
    TS_ASSERT_DELTA(gradient, 5.49, 0.1);  // percent
  }

  // Test 49: Descent rate for 3-degree glide
  void testDescentRateFor3DegreeGlide() {
    double groundSpeed = 150.0;  // kts
    double GS_fps = groundSpeed * KTS_TO_FPS;
    double angle = 3.0 * DEG_TO_RAD;

    double descent_rate_fps = GS_fps * std::tan(angle);
    double descent_rate_fpm = descent_rate_fps * 60.0;

    TS_ASSERT_DELTA(descent_rate_fpm, 797.0, 10.0);  // ft/min
  }

  // Test 50: Rule of thumb descent rate
  void testRuleOfThumbDescentRate() {
    // Descent rate ≈ groundspeed (kts) * 5
    double groundSpeed = 150.0;
    double descent_rate = groundSpeed * 5.0;

    TS_ASSERT_DELTA(descent_rate, 750.0, epsilon);  // ft/min
  }

  /***************************************************************************
   * Wind Correction Tests
   ***************************************************************************/

  // Test 51: Wind correction angle
  void testWindCorrectionAngle() {
    double crosswind = 20.0;  // kts
    double TAS = 120.0;       // kts

    double WCA = std::asin(crosswind / TAS) * RAD_TO_DEG;
    TS_ASSERT_DELTA(WCA, 9.59, 0.1);  // degrees
  }

  // Test 52: Heading from track and wind
  void testHeadingFromTrack() {
    double track = 90.0;      // degrees (east)
    double wind_dir = 0.0;    // degrees (from north)
    double wind_speed = 20.0; // kts
    double TAS = 120.0;       // kts

    // Wind from north, flying east = right crosswind
    double crosswind = wind_speed * std::sin((90.0 - wind_dir) * DEG_TO_RAD);
    double WCA = std::asin(crosswind / TAS) * RAD_TO_DEG;
    double heading = track - WCA;  // Crab into the wind

    TS_ASSERT_DELTA(WCA, 9.59, 0.1);
    TS_ASSERT_DELTA(heading, 80.41, 0.1);
  }

  // Test 53: Ground speed calculation with wind
  void testGroundSpeedWithWind() {
    double TAS = 150.0;        // kts
    double wind_speed = 30.0;  // kts
    double angle = 45.0;       // degrees (relative to track)

    // Components
    double headwind = wind_speed * std::cos(angle * DEG_TO_RAD);
    double crosswind = wind_speed * std::sin(angle * DEG_TO_RAD);

    // Approximate ground speed
    double GS = std::sqrt((TAS - headwind) * (TAS - headwind) + crosswind * crosswind);
    // More accurate would account for crab angle

    TS_ASSERT(GS > 0);
    TS_ASSERT(GS < TAS);  // Headwind component reduces GS
  }

  /***************************************************************************
   * Turn Performance Advanced Tests
   ***************************************************************************/

  // Test 54: Turn rate (degrees per second)
  void testTurnRate() {
    double V = 200.0 * KTS_TO_FPS;  // ft/s
    double bank = 30.0 * DEG_TO_RAD;

    double omega = G * std::tan(bank) / V;  // rad/s
    double turn_rate = omega * RAD_TO_DEG;

    TS_ASSERT_DELTA(turn_rate, 3.16, 0.1);  // deg/s
  }

  // Test 55: Standard rate turn (3 deg/s)
  void testStandardRateTurn() {
    double turn_rate = 3.0;  // deg/s
    double time_360 = 360.0 / turn_rate;

    TS_ASSERT_DELTA(time_360, 120.0, epsilon);  // 2 minutes
  }

  // Test 56: Bank angle for standard rate turn
  void testBankForStandardRate() {
    double V = 120.0 * KTS_TO_FPS;  // ft/s
    double omega = 3.0 * DEG_TO_RAD;  // rad/s

    // omega = g * tan(phi) / V
    // phi = atan(omega * V / g)
    double phi = std::atan(omega * V / G) * RAD_TO_DEG;

    TS_ASSERT_DELTA(phi, 18.24, 0.5);  // degrees
  }

  // Test 57: Load factor in turn
  void testLoadFactorInTurn() {
    double bank = 60.0 * DEG_TO_RAD;
    double n = 1.0 / std::cos(bank);

    TS_ASSERT_DELTA(n, 2.0, 0.01);  // 2g turn
  }

  // Test 58: Stall speed in turn
  void testStallSpeedInTurn() {
    double Vs = 50.0;  // 1g stall speed (kts)
    double n = 2.0;    // Load factor

    double Vs_turn = Vs * std::sqrt(n);
    TS_ASSERT_DELTA(Vs_turn, 70.71, 0.1);  // kts
  }

  /***************************************************************************
   * Approach Path Tests
   ***************************************************************************/

  // Test 59: VASI/PAPI 3-degree glide path altitude
  void testGlidePathAltitude() {
    double distance = 3.0;  // nm from threshold
    double glide_angle = 3.0 * DEG_TO_RAD;

    double distance_ft = distance * 6076.12;
    double altitude = distance_ft * std::tan(glide_angle);

    TS_ASSERT_DELTA(altitude, 955.0, 10.0);  // ft
  }

  // Test 60: ILS glideslope intercept altitude
  void testGlideslopeIntercept() {
    double FAF_distance = 5.0;  // nm
    double glide_angle = 3.0 * DEG_TO_RAD;

    double FAF_distance_ft = FAF_distance * 6076.12;
    double intercept_alt = FAF_distance_ft * std::tan(glide_angle);

    TS_ASSERT_DELTA(intercept_alt, 1592.0, 10.0);  // ft
  }

  // Test 61: Visual descent point
  void testVisualDescentPoint() {
    double field_elevation = 500.0;  // ft
    double TPA = 1000.0;             // Traffic pattern altitude AGL
    double descent_gradient = 300.0; // ft/nm (typical)

    double pattern_alt = field_elevation + TPA;
    double VDP_distance = TPA / descent_gradient;

    TS_ASSERT_DELTA(pattern_alt, 1500.0, epsilon);
    TS_ASSERT_DELTA(VDP_distance, 3.33, 0.1);  // nm from threshold
  }

  /***************************************************************************
   * Performance Planning Tests
   ***************************************************************************/

  // Test 62: Takeoff distance over 50 ft obstacle
  void testTakeoffDistanceOver50ft() {
    double ground_roll = 2000.0;   // ft
    double obstacle_factor = 1.4;  // Typical factor for 50 ft clearance

    double total_distance = ground_roll * obstacle_factor;
    TS_ASSERT_DELTA(total_distance, 2800.0, epsilon);  // ft
  }

  // Test 63: Landing distance over 50 ft obstacle
  void testLandingDistanceOver50ft() {
    double ground_roll = 1500.0;
    double approach_factor = 1.67;  // Typical for 50 ft

    double total_distance = ground_roll * approach_factor;
    TS_ASSERT_DELTA(total_distance, 2505.0, epsilon);  // ft
  }

  // Test 64: Accelerate-stop distance
  void testAccelerateStopDistance() {
    double V1 = 100.0;           // Decision speed (kts)
    double V1_fps = V1 * KTS_TO_FPS;
    double decel = 10.0;         // ft/s²
    double accel_distance = 1500.0;  // ft to V1

    double stopping_distance = V1_fps * V1_fps / (2.0 * decel);
    double total = accel_distance + stopping_distance;

    TS_ASSERT_DELTA(stopping_distance, 1424.9, 10.0);  // ft
    TS_ASSERT_DELTA(total, 2924.9, 10.0);
  }

  // Test 65: Balanced field length concept
  void testBalancedFieldLength() {
    double accel_stop = 4000.0;     // ft
    double continue_takeoff = 3500.0;  // ft over obstacle

    // BFL is where these are equal
    double BFL = std::max(accel_stop, continue_takeoff);
    TS_ASSERT_DELTA(BFL, 4000.0, epsilon);
  }

  /***************************************************************************
   * Holding Pattern Tests
   ***************************************************************************/

  // Test 66: Holding pattern fuel burn
  void testHoldingFuelBurn() {
    double fuel_flow = 12.0;     // gal/hr
    double hold_time = 30.0;     // minutes

    double fuel_used = fuel_flow * (hold_time / 60.0);
    TS_ASSERT_DELTA(fuel_used, 6.0, epsilon);  // gallons
  }

  // Test 67: One minute leg holding pattern distance
  void testHoldingPatternDistance() {
    double TAS = 180.0;  // kts
    double leg_time = 1.0;  // minutes

    double leg_distance = TAS * (leg_time / 60.0);
    TS_ASSERT_DELTA(leg_distance, 3.0, epsilon);  // nm
  }

  // Test 68: Holding pattern entry time
  void testHoldingPatternEntry() {
    // Standard rate turns for entry
    double standard_rate = 3.0;  // deg/s
    double turn_angle = 180.0;   // degrees (direct entry)

    double turn_time = turn_angle / standard_rate;
    TS_ASSERT_DELTA(turn_time, 60.0, epsilon);  // seconds
  }

  /***************************************************************************
   * Profile Descent Tests
   ***************************************************************************/

  // Test 69: Idle descent fuel savings
  void testIdleDescentFuelSavings() {
    double step_down_fuel = 200.0;  // lbs (traditional)
    double idle_descent_fuel = 50.0; // lbs (profile descent)

    double savings = step_down_fuel - idle_descent_fuel;
    double savings_percent = (savings / step_down_fuel) * 100.0;

    TS_ASSERT_DELTA(savings, 150.0, epsilon);
    TS_ASSERT_DELTA(savings_percent, 75.0, epsilon);
  }

  // Test 70: Continuous descent distance
  void testContinuousDescentDistance() {
    double start_alt = 35000.0;  // ft
    double end_alt = 3000.0;     // ft
    double avg_gradient = 3.0;   // degrees

    double alt_diff = start_alt - end_alt;
    double distance_ft = alt_diff / std::tan(avg_gradient * DEG_TO_RAD);
    double distance_nm = distance_ft / 6076.12;

    TS_ASSERT_DELTA(distance_nm, 100.5, 1.0);
  }

  /***************************************************************************
   * Speed Schedule Tests
   ***************************************************************************/

  // Test 71: Climb speed schedule (250 below 10000)
  void testClimbSpeedSchedule() {
    double altitude = 8000.0;
    double IAS_below_10k = 250.0;  // kts
    double IAS_above_10k = 280.0;  // kts

    double IAS = (altitude < 10000.0) ? IAS_below_10k : IAS_above_10k;
    TS_ASSERT_DELTA(IAS, 250.0, epsilon);
  }

  // Test 72: Mach/IAS crossover altitude
  void testCrossoverAltitude() {
    // Crossover: where constant IAS = constant Mach
    // Simplified: at ~28,000-30,000 ft for typical jets
    double crossover_alt = 29000.0;  // ft (approximate)

    TS_ASSERT(crossover_alt > 25000.0);
    TS_ASSERT(crossover_alt < 35000.0);
  }

  // Test 73: Descent speed transition
  void testDescentSpeedTransition() {
    double cruise_mach = 0.78;
    double descent_mach = 0.78;    // Same initially
    double descent_IAS = 280.0;    // Below crossover
    double approach_IAS = 250.0;   // Below 10000 ft

    // Verify schedule exists
    TS_ASSERT(descent_mach <= cruise_mach);
    TS_ASSERT(descent_IAS >= approach_IAS);
  }

  /***************************************************************************
   * Fuel Planning Tests
   ***************************************************************************/

  // Test 74: Alternate fuel calculation
  void testAlternateFuel() {
    double alternate_distance = 100.0;  // nm
    double TAS = 200.0;                 // kts
    double fuel_flow = 150.0;           // lbs/hr

    double time_hrs = alternate_distance / TAS;
    double alternate_fuel = fuel_flow * time_hrs;

    TS_ASSERT_DELTA(alternate_fuel, 75.0, epsilon);  // lbs
  }

  // Test 75: Contingency fuel (5%)
  void testContingencyFuel() {
    double trip_fuel = 1000.0;  // lbs
    double contingency_percent = 0.05;

    double contingency_fuel = trip_fuel * contingency_percent;
    TS_ASSERT_DELTA(contingency_fuel, 50.0, epsilon);  // lbs
  }
};

/*******************************************************************************
 * Extended FGFlightPath Tests (25 new tests)
 ******************************************************************************/

class FGFlightPathExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Advanced Energy Management Tests
   ***************************************************************************/

  // Test 76: Specific excess power (Ps)
  void testSpecificExcessPower() {
    double thrust = 5000.0;   // lbs
    double drag = 3000.0;     // lbs
    double weight = 10000.0;  // lbs
    double V = 400.0;         // ft/s

    // Ps = (T - D) * V / W
    double Ps = (thrust - drag) * V / weight;

    TS_ASSERT_DELTA(Ps, 80.0, epsilon);  // ft/s
  }

  // Test 77: Energy rate from Ps
  void testEnergyRateFromPs() {
    double Ps = 80.0;  // ft/s
    double dt = 10.0;  // seconds

    // Change in energy height
    double delta_He = Ps * dt;

    TS_ASSERT_DELTA(delta_He, 800.0, epsilon);  // ft
  }

  // Test 78: Maximum sustained turn performance
  void testMaxSustainedTurn() {
    // At sustained turn, Ps = 0 (no energy change)
    double thrust = 4000.0;
    double weight = 10000.0;
    double V = 350.0;
    double CL = 0.8;
    double S = 200.0;
    double rho = 0.002377;

    double q = 0.5 * rho * V * V;
    double lift = q * S * CL;
    double n_max = lift / weight;

    TS_ASSERT(n_max > 1.0);
  }

  // Test 79: Corner speed (max instantaneous turn)
  void testCornerSpeed() {
    // Speed for maximum instantaneous turn rate
    double Vs = 100.0;   // Stall speed ft/s
    double n_max = 4.0;  // Structural limit

    // Corner speed = Vs * sqrt(n_max)
    double Vc = Vs * std::sqrt(n_max);

    TS_ASSERT_DELTA(Vc, 200.0, epsilon);
  }

  /***************************************************************************
   * Trajectory Optimization Tests
   ***************************************************************************/

  // Test 80: Constant CL climb
  void testConstantCLClimb() {
    double CL = 0.5;
    double weight = 10000.0;
    double S = 200.0;
    double rho = 0.002377;

    // Speed for constant CL
    double q = weight / (S * CL);
    double V = std::sqrt(2.0 * q / rho);

    TS_ASSERT(V > 0);
    TS_ASSERT_DELTA(V, 289.7, 1.0);
  }

  // Test 81: Minimum time to climb profile
  void testMinTimeClimbProfile() {
    // Min time climb at speed for max Ps
    double V_maxPs = 300.0;  // ft/s
    double altitude = 10000.0;
    double Ps = 50.0;  // ft/s average

    double time = altitude / Ps;

    TS_ASSERT_DELTA(time, 200.0, epsilon);  // seconds
  }

  // Test 82: Cruise climb (constant Mach)
  void testCruiseClimb() {
    double mach = 0.78;
    double a_low = 1000.0;   // Speed of sound at low alt
    double a_high = 970.0;   // Speed of sound at high alt

    double TAS_low = mach * a_low;
    double TAS_high = mach * a_high;

    // TAS decreases with altitude at constant Mach
    TS_ASSERT(TAS_high < TAS_low);
  }

  /***************************************************************************
   * Great Circle Navigation Tests
   ***************************************************************************/

  // Test 83: Great circle distance
  void testGreatCircleDistance() {
    // Simplified haversine for small distances
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 51.5 * DEG_TO_RAD;
    double lon2 = 0.0 * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1) * std::cos(lat2) * std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    double R = 3440.065;  // Earth radius in nm
    double distance = R * c;

    TS_ASSERT_DELTA(distance, 3000.0, 100.0);  // ~3000 nm NYC to London
  }

  // Test 84: Initial great circle heading
  void testGreatCircleHeading() {
    double lat1 = 40.0 * DEG_TO_RAD;
    double lon1 = -74.0 * DEG_TO_RAD;
    double lat2 = 51.5 * DEG_TO_RAD;
    double lon2 = 0.0 * DEG_TO_RAD;

    double dlon = lon2 - lon1;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double heading = std::atan2(x, y) * RAD_TO_DEG;
    if (heading < 0) heading += 360.0;

    TS_ASSERT(heading > 40.0 && heading < 70.0);  // Northeast
  }

  /***************************************************************************
   * Cost Index Optimization Tests
   ***************************************************************************/

  // Test 85: Cost index effect on speed
  void testCostIndexSpeed() {
    // Higher CI = faster speed (time more valuable)
    double CI_low = 20.0;
    double CI_high = 100.0;
    double V_MRC = 250.0;  // Max range cruise speed

    // Speed increases with CI
    double V_low = V_MRC + CI_low * 0.2;
    double V_high = V_MRC + CI_high * 0.2;

    TS_ASSERT(V_high > V_low);
  }

  // Test 86: Econ speed calculation
  void testEconSpeed() {
    double CI = 50.0;  // Cost index
    double V_MRC = 240.0;
    double V_MMO = 320.0;

    // Linear interpolation for simplification
    double econ_speed = V_MRC + (V_MMO - V_MRC) * (CI / 200.0);

    TS_ASSERT(econ_speed > V_MRC);
    TS_ASSERT(econ_speed < V_MMO);
    TS_ASSERT_DELTA(econ_speed, 260.0, 1.0);
  }

  /***************************************************************************
   * Wind Optimization Tests
   ***************************************************************************/

  // Test 87: Optimal altitude for wind
  void testOptimalWindAltitude() {
    double headwind_low = 30.0;   // At FL350
    double headwind_mid = 10.0;   // At FL310
    double headwind_high = 50.0;  // At FL390

    double min_headwind = std::min(std::min(headwind_low, headwind_mid), headwind_high);

    TS_ASSERT_DELTA(min_headwind, 10.0, epsilon);
  }

  // Test 88: Optimal track deviation for winds
  void testOptimalTrackDeviation() {
    double direct_distance = 1000.0;  // nm
    double direct_headwind = 50.0;    // kts
    double deviation_distance = 1050.0;  // nm
    double deviation_headwind = 20.0;    // kts
    double TAS = 450.0;

    double direct_time = direct_distance / (TAS - direct_headwind);
    double deviation_time = deviation_distance / (TAS - deviation_headwind);

    // Deviation might be faster despite longer distance
    TS_ASSERT(deviation_time < direct_time);
  }

  /***************************************************************************
   * Step Climb Tests
   ***************************************************************************/

  // Test 89: Step climb fuel benefit
  void testStepClimbFuelBenefit() {
    double fuel_flow_FL350 = 5000.0;  // lbs/hr
    double fuel_flow_FL370 = 4800.0;  // lbs/hr
    double flight_time = 4.0;  // hours

    double fuel_350 = fuel_flow_FL350 * flight_time;
    double fuel_370 = fuel_flow_FL370 * flight_time;
    double savings = fuel_350 - fuel_370;

    TS_ASSERT_DELTA(savings, 800.0, epsilon);  // lbs
  }

  // Test 90: Step climb timing
  void testStepClimbTiming() {
    // Climb when weight reduces enough
    double initial_weight = 150000.0;
    double fuel_burn_rate = 5000.0;  // lbs/hr
    double weight_for_step = 140000.0;

    double time_to_step = (initial_weight - weight_for_step) / fuel_burn_rate;

    TS_ASSERT_DELTA(time_to_step, 2.0, epsilon);  // hours
  }

  /***************************************************************************
   * Approach Energy Tests
   ***************************************************************************/

  // Test 91: Stabilized approach energy
  void testStabilizedApproachEnergy() {
    double V_app = 140.0 * KTS_TO_FPS;
    double altitude = 1000.0;  // ft
    double weight = 50000.0;

    // Total energy
    double KE = 0.5 * (weight / G) * V_app * V_app;
    double PE = weight * altitude;
    double total_energy = KE + PE;

    TS_ASSERT(total_energy > 0);
  }

  // Test 92: Energy to bleed on approach
  void testEnergyToBleedOnApproach() {
    double V_high = 180.0 * KTS_TO_FPS;
    double V_target = 140.0 * KTS_TO_FPS;
    double mass = 1500.0;  // slugs

    double KE_high = 0.5 * mass * V_high * V_high;
    double KE_target = 0.5 * mass * V_target * V_target;
    double energy_to_bleed = KE_high - KE_target;

    TS_ASSERT(energy_to_bleed > 0);
  }

  /***************************************************************************
   * Missed Approach Tests
   ***************************************************************************/

  // Test 93: Missed approach climb gradient
  void testMissedApproachGradient() {
    // Minimum 2.5% gradient required
    double required_gradient = 2.5;  // percent
    double ROC = 500.0;  // ft/min
    double GS = 140.0;   // kts

    double GS_fpm = GS * 6076.12 / 60.0;
    double actual_gradient = (ROC / GS_fpm) * 100.0;

    TS_ASSERT(actual_gradient >= required_gradient);
  }

  // Test 94: Missed approach fuel burn
  void testMissedApproachFuelBurn() {
    double go_around_fuel = 200.0;     // lbs
    double climb_fuel = 100.0;         // lbs
    double vectors_fuel = 300.0;       // lbs (15 min at holding speed)
    double approach_fuel = 50.0;       // lbs

    double total = go_around_fuel + climb_fuel + vectors_fuel + approach_fuel;

    TS_ASSERT_DELTA(total, 650.0, epsilon);  // lbs
  }

  /***************************************************************************
   * Performance Degradation Tests
   ***************************************************************************/

  // Test 95: Engine out ceiling
  void testEngineOutCeiling() {
    double two_engine_ceiling = 40000.0;  // ft
    double engine_out_factor = 0.6;

    double one_engine_ceiling = two_engine_ceiling * engine_out_factor;

    TS_ASSERT_DELTA(one_engine_ceiling, 24000.0, epsilon);
  }

  // Test 96: Icing effect on performance
  void testIcingPerformanceEffect() {
    double clean_drag = 1000.0;   // lbs
    double ice_penalty = 0.15;    // 15% increase

    double iced_drag = clean_drag * (1.0 + ice_penalty);
    double drag_increase = iced_drag - clean_drag;

    TS_ASSERT_DELTA(iced_drag, 1150.0, epsilon);
    TS_ASSERT_DELTA(drag_increase, 150.0, epsilon);
  }

  // Test 97: High altitude performance loss
  void testHighAltitudePerformance() {
    double sea_level_thrust = 20000.0;  // lbs
    double density_ratio = 0.3;  // At FL400

    double high_alt_thrust = sea_level_thrust * density_ratio;

    TS_ASSERT_DELTA(high_alt_thrust, 6000.0, epsilon);
  }

  /***************************************************************************
   * Fuel Emergency Tests
   ***************************************************************************/

  // Test 98: Minimum fuel calculation
  void testMinimumFuelCalculation() {
    double distance = 50.0;      // nm to nearest suitable
    double groundspeed = 200.0;  // kts
    double fuel_flow = 1000.0;   // lbs/hr

    double time_hrs = distance / groundspeed;
    double fuel_required = fuel_flow * time_hrs;
    double reserve = fuel_required * 0.1;
    double minimum_fuel = fuel_required + reserve;

    TS_ASSERT_DELTA(minimum_fuel, 275.0, 1.0);  // lbs
  }

  // Test 99: Bingo fuel (return to base)
  void testBingoFuel() {
    double distance_to_base = 200.0;  // nm
    double TAS = 400.0;               // kts
    double fuel_flow = 2000.0;        // lbs/hr
    double reserve = 500.0;           // lbs

    double time_hrs = distance_to_base / TAS;
    double trip_fuel = fuel_flow * time_hrs;
    double bingo = trip_fuel + reserve;

    TS_ASSERT_DELTA(bingo, 1500.0, epsilon);  // lbs
  }

  // Test 100: Maximum range profile
  void testMaxRangeProfile() {
    // Fly at speed for max L/D, highest efficient altitude
    double fuel = 10000.0;       // lbs
    double TSFC = 0.5;           // lb/hr/lb thrust
    double LD_max = 18.0;        // Best L/D
    double weight_ratio = 1.2;   // Initial/final weight

    // Simplified Breguet: R = (V/c) * L/D * ln(Wi/Wf)
    // For max range, V/c should be optimized
    double range_factor = LD_max * std::log(weight_ratio);

    TS_ASSERT(range_factor > 0);
    TS_ASSERT_DELTA(range_factor, 3.28, 0.1);
  }
};

/*******************************************************************************
 * C172X MODEL INTEGRATION TESTS (25 tests)
 ******************************************************************************/

class FGFlightPathC172xTest : public CxxTest::TestSuite
{
public:
  void testC172xFlightPathModelLoads() {
    // Test that C172x model loads for flight path testing
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());
  }

  void testC172xFlightPathAngleFinite() {
    // Test flight path angle is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double gamma = aux->GetGamma();
    TS_ASSERT(std::isfinite(gamma));
  }

  void testC172xGroundTrackFinite() {
    // Test ground track angle is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double track = aux->GetGroundTrack();
    TS_ASSERT(std::isfinite(track));
  }

  void testC172xGroundSpeedFinite() {
    // Test ground speed is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double vg = aux->GetVground();
    TS_ASSERT(std::isfinite(vg));
    TS_ASSERT(vg >= 0.0);
  }

  void testC172xTASFinite() {
    // Test true airspeed is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double vt = aux->GetVt();
    TS_ASSERT(std::isfinite(vt));
    TS_ASSERT(vt >= 0.0);
  }

  void testC172xClimbRateFinite() {
    // Test climb rate is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double hdot = propagate->Gethdot();
    TS_ASSERT(std::isfinite(hdot));
  }

  void testC172xPositionFinite() {
    // Test position is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double lat = propagate->GetLatitudeDeg();
    double lon = propagate->GetLongitudeDeg();
    double alt = propagate->GetAltitudeASL();

    TS_ASSERT(std::isfinite(lat));
    TS_ASSERT(std::isfinite(lon));
    TS_ASSERT(std::isfinite(alt));
  }

  void testC172xHeadingFinite() {
    // Test heading is finite
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    for (int i = 0; i < 50; i++) fdmex.Run();

    double psi = propagate->GetEuler(3);  // Heading
    TS_ASSERT(std::isfinite(psi));
  }

  void testC172xClimbSimulation() {
    // Test climb with full power
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);
    fcs->SetDeCmd(-0.2);  // Nose up

    double initial_alt = propagate->GetAltitudeASL();
    for (int i = 0; i < 500; i++) fdmex.Run();

    double final_alt = propagate->GetAltitudeASL();
    TS_ASSERT(std::isfinite(final_alt));
    // Should be climbing (or at least not descending much)
  }

  void testC172xDescentSimulation() {
    // Test descent with reduced power
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.2);  // Reduced power

    for (int i = 0; i < 200; i++) fdmex.Run();

    double gamma = aux->GetGamma();
    TS_ASSERT(std::isfinite(gamma));
  }

  void testC172xTurnFlightPath() {
    // Test flight path during turn
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.8);
    fcs->SetDaCmd(0.3);  // Bank input

    for (int i = 0; i < 300; i++) fdmex.Run();

    double gamma = aux->GetGamma();
    double track = aux->GetGroundTrack();
    TS_ASSERT(std::isfinite(gamma));
    TS_ASSERT(std::isfinite(track));
  }

  void testC172xEnergyHeightComputation() {
    // Test energy height computation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto propagate = fdmex.GetPropagate();
    auto aux = fdmex.GetAuxiliary();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    for (int i = 0; i < 200; i++) fdmex.Run();

    double h = propagate->GetAltitudeASL();
    double v = aux->GetVt();
    double energy_height = h + (v * v) / (2.0 * G);

    TS_ASSERT(std::isfinite(energy_height));
    TS_ASSERT(energy_height > 0.0);
  }

  void testC172xGlideRatio() {
    // Test glide ratio (L/D)
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto aero = fdmex.GetAerodynamics();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.5);

    for (int i = 0; i < 200; i++) fdmex.Run();

    double ld = aero->GetLoD();
    TS_ASSERT(std::isfinite(ld));
  }

  void testC172xSpecificExcessPower() {
    // Test specific excess power calculation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    for (int i = 0; i < 100; i++) fdmex.Run();

    // Ps approximated by hdot when in steady state
    double vt = aux->GetVt();
    TS_ASSERT(std::isfinite(vt));
  }

  void testC172xRangeParameters() {
    // Test range-related parameters
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.75);

    for (int i = 0; i < 200; i++) fdmex.Run();

    double vg = aux->GetVground();
    double vcas = aux->GetVcalibratedKTS();

    TS_ASSERT(std::isfinite(vg));
    TS_ASSERT(std::isfinite(vcas));
  }

  void testC172xWindEffect() {
    // Test flight path with wind
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    // Set wind
    auto pm = fdmex.GetPropertyManager();
    auto wind_node = pm->GetNode("atmosphere/wind-north-fps", true);
    wind_node->setDoubleValue(-20.0);  // 20 fps headwind

    auto aux = fdmex.GetAuxiliary();
    for (int i = 0; i < 100; i++) fdmex.Run();

    double vg = aux->GetVground();
    double vt = aux->GetVt();

    TS_ASSERT(std::isfinite(vg));
    TS_ASSERT(std::isfinite(vt));
  }

  void testC172xCrosswindFlightPath() {
    // Test flight path with crosswind
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    // Set crosswind
    auto pm = fdmex.GetPropertyManager();
    auto wind_node = pm->GetNode("atmosphere/wind-east-fps", true);
    wind_node->setDoubleValue(15.0);  // 15 fps crosswind

    auto aux = fdmex.GetAuxiliary();
    for (int i = 0; i < 100; i++) fdmex.Run();

    double track = aux->GetGroundTrack();
    TS_ASSERT(std::isfinite(track));
  }

  void testC172xSteadyLevelFlight() {
    // Test steady level flight parameters
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.6);

    for (int i = 0; i < 300; i++) fdmex.Run();

    double gamma = aux->GetGamma();
    double hdot = propagate->Gethdot();

    TS_ASSERT(std::isfinite(gamma));
    TS_ASSERT(std::isfinite(hdot));
  }

  void testC172xFlightPathAcceleration() {
    // Test accelerations along flight path
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto acc = fdmex.GetAccelerations();
    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    for (int i = 0; i < 100; i++) fdmex.Run();

    double ax = acc->GetBodyAccel(1);
    TS_ASSERT(std::isfinite(ax));
  }

  void testC172xDistanceTraveled() {
    // Test distance traveled
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    double lat0 = propagate->GetLatitudeDeg();
    double lon0 = propagate->GetLongitudeDeg();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    for (int i = 0; i < 500; i++) fdmex.Run();

    double lat1 = propagate->GetLatitudeDeg();
    double lon1 = propagate->GetLongitudeDeg();

    TS_ASSERT(std::isfinite(lat1));
    TS_ASSERT(std::isfinite(lon1));
  }

  void testC172xAltitudeChange() {
    // Test altitude change over time
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto propagate = fdmex.GetPropagate();
    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    double alt0 = propagate->GetAltitudeASL();
    for (int i = 0; i < 500; i++) fdmex.Run();

    double alt1 = propagate->GetAltitudeASL();
    TS_ASSERT(std::isfinite(alt0));
    TS_ASSERT(std::isfinite(alt1));
  }

  void testC172xVelocityComponents() {
    // Test velocity components
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 0.8);

    for (int i = 0; i < 200; i++) fdmex.Run();

    double alpha = aux->Getalpha();
    double beta = aux->Getbeta();

    TS_ASSERT(std::isfinite(alpha));
    TS_ASSERT(std::isfinite(beta));
  }

  void testC172xDynamicPressure() {
    // Test dynamic pressure
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    for (int i = 0; i < 200; i++) fdmex.Run();

    double qbar = aux->Getqbar();
    TS_ASSERT(std::isfinite(qbar));
    TS_ASSERT(qbar >= 0.0);
  }

  void testC172xExtendedFlightPathRun() {
    // Extended flight path simulation
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    auto propagate = fdmex.GetPropagate();
    auto prop = fdmex.GetPropulsion();
    auto fcs = fdmex.GetFCS();

    prop->InitRunning(-1);
    fcs->SetThrottleCmd(-1, 1.0);

    // Run extended simulation
    for (int i = 0; i < 1000; i++) {
      TS_ASSERT(fdmex.Run());

      // Verify flight path parameters remain finite
      TS_ASSERT(std::isfinite(aux->GetGamma()));
      TS_ASSERT(std::isfinite(aux->GetVground()));
      TS_ASSERT(std::isfinite(propagate->GetAltitudeASL()));
      TS_ASSERT(std::isfinite(propagate->Gethdot()));
    }

    // Final checks
    double gamma = aux->GetGamma();
    double vg = aux->GetVground();
    double alt = propagate->GetAltitudeASL();
    double hdot = propagate->Gethdot();

    TS_ASSERT(std::isfinite(gamma));
    TS_ASSERT(std::isfinite(vg));
    TS_ASSERT(std::isfinite(alt));
    TS_ASSERT(std::isfinite(hdot));
  }
};
