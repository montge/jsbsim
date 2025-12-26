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
