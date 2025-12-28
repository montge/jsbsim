/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       FGAtmosphericModelTest.h
 Author:       Claude Code
 Date started: 12/24/2025

 %%%%%%%%%%%%%%%%%%% DOCUMENTATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 FUNCTIONAL TEST for ISA atmospheric model calculations

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 SENTRY
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

constexpr double epsilon = 100.0 * std::numeric_limits<double>::epsilon();

// ISA Standard Atmosphere Constants
constexpr double ISA_SEA_LEVEL_TEMP_K = 288.15;      // K (15°C)
constexpr double ISA_SEA_LEVEL_TEMP_C = 15.0;        // °C
constexpr double ISA_SEA_LEVEL_PRESSURE = 101325.0;  // Pa
constexpr double ISA_SEA_LEVEL_DENSITY = 1.225;      // kg/m³
constexpr double ISA_LAPSE_RATE = -0.0065;           // K/m (-6.5°C/km)
constexpr double TROPOPAUSE_ALTITUDE = 11000.0;      // m
constexpr double TROPOPAUSE_TEMP = 216.65;           // K (-56.5°C)
constexpr double GAS_CONSTANT_AIR = 287.05287;       // J/(kg·K)
constexpr double GRAVITY_SI = 9.80665;               // m/s²
constexpr double GAMMA_AIR = 1.4;                    // Ratio of specific heats
constexpr double SUTHERLAND_CONSTANT = 110.4;        // K
constexpr double DYNAMIC_VISCOSITY_REF = 1.7894e-5;  // Pa·s at 288.15K

class FGAtmosphericModelTest : public CxxTest::TestSuite
{
public:
  // Test 1: ISA temperature at sea level
  void testISATemperatureSeaLevel() {
    double temp = ISA_SEA_LEVEL_TEMP_K;
    TS_ASSERT_DELTA(temp, 288.15, epsilon);
    TS_ASSERT_DELTA(temp - 273.15, 15.0, epsilon);
  }

  // Test 2: ISA temperature lapse rate in troposphere (at 5000m)
  void testISATemperatureLapseRate5000m() {
    double altitude = 5000.0; // meters
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double expected = 288.15 - 32.5; // 255.65 K
    TS_ASSERT_DELTA(temp, expected, 1e-10);
    TS_ASSERT_DELTA(temp, 255.65, epsilon);
  }

  // Test 3: ISA temperature at tropopause (11,000m)
  void testISATemperatureAtTropopause() {
    double altitude = TROPOPAUSE_ALTITUDE;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    TS_ASSERT_DELTA(temp, TROPOPAUSE_TEMP, 1e-10);
    TS_ASSERT_DELTA(temp, 216.65, epsilon);
  }

  // Test 4: Temperature ratio (theta) at altitude
  void testTemperatureRatio() {
    double altitude = 5000.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double theta = temp / ISA_SEA_LEVEL_TEMP_K;
    double expected = 255.65 / 288.15;
    TS_ASSERT_DELTA(theta, expected, epsilon);
    TS_ASSERT(theta < 1.0); // Temperature decreases with altitude
  }

  // Test 5: Pressure ratio (delta) in troposphere
  void testPressureRatioTroposphere() {
    double altitude = 5000.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double delta = std::pow(temp / ISA_SEA_LEVEL_TEMP_K, exponent);
    
    // At 5000m, pressure ratio should be approximately 0.5334
    TS_ASSERT_DELTA(delta, 0.5334, 0.001);
    TS_ASSERT(delta < 1.0); // Pressure decreases with altitude
  }

  // Test 6: Pressure altitude calculation from pressure
  void testPressureAltitudeFromPressure() {
    // Given a pressure, calculate the altitude in troposphere
    // Use the inverse of the pressure formula: p = p0 * (T/T0)^(-g/(L*R))
    double pressure = 54048.0; // Pa (approximately 5000m)
    double pressure_ratio = pressure / ISA_SEA_LEVEL_PRESSURE;
    double exponent = (ISA_LAPSE_RATE * GAS_CONSTANT_AIR) / (-GRAVITY_SI);
    double temp_ratio = std::pow(pressure_ratio, exponent);
    double altitude = (ISA_SEA_LEVEL_TEMP_K * (temp_ratio - 1.0)) / ISA_LAPSE_RATE;

    TS_ASSERT_DELTA(altitude, 5000.0, 10.0); // Within 10m
  }

  // Test 7: Density ratio (sigma) from temperature and pressure ratios
  void testDensityRatio() {
    double altitude = 5000.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double theta = temp / ISA_SEA_LEVEL_TEMP_K;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double delta = std::pow(theta, exponent);
    double sigma = delta / theta; // rho/rho0 = (P/P0) / (T/T0)
    
    // At 5000m, density ratio should be approximately 0.6012
    TS_ASSERT_DELTA(sigma, 0.6012, 0.001);
    TS_ASSERT(sigma < 1.0); // Density decreases with altitude
  }

  // Test 8: Density calculation from ideal gas law
  void testDensityFromIdealGasLaw() {
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double temp = ISA_SEA_LEVEL_TEMP_K;
    double density = pressure / (GAS_CONSTANT_AIR * temp);
    
    TS_ASSERT_DELTA(density, ISA_SEA_LEVEL_DENSITY, 0.001);
  }

  // Test 9: Speed of sound at sea level
  void testSpeedOfSoundSeaLevel() {
    double temp = ISA_SEA_LEVEL_TEMP_K;
    double speed_of_sound = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * temp);
    
    // Speed of sound at 15°C should be approximately 340.3 m/s
    TS_ASSERT_DELTA(speed_of_sound, 340.294, 0.01);
  }

  // Test 10: Speed of sound at altitude
  void testSpeedOfSoundAtAltitude() {
    double altitude = 10000.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double speed_of_sound = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * temp);
    
    // At 10000m, temperature is 223.15K, speed of sound ~299.5 m/s
    TS_ASSERT_DELTA(speed_of_sound, 299.5, 1.0);
    TS_ASSERT(speed_of_sound < 340.3); // Lower than sea level
  }

  // Test 11: Mach number calculation
  void testMachNumberCalculation() {
    double true_airspeed = 250.0; // m/s
    double temp = ISA_SEA_LEVEL_TEMP_K;
    double speed_of_sound = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * temp);
    double mach = true_airspeed / speed_of_sound;
    
    TS_ASSERT_DELTA(mach, 0.7347, 0.001);
  }

  // Test 12: Dynamic pressure calculation
  void testDynamicPressure() {
    double density = ISA_SEA_LEVEL_DENSITY;
    double velocity = 100.0; // m/s
    double q = 0.5 * density * velocity * velocity;
    
    TS_ASSERT_DELTA(q, 6125.0, 0.1);
  }

  // Test 13: Dynamic viscosity using Sutherland's formula
  void testDynamicViscositySutherland() {
    double temp = ISA_SEA_LEVEL_TEMP_K;
    double ref_temp = 288.15;
    double mu = DYNAMIC_VISCOSITY_REF * std::pow(temp / ref_temp, 1.5) * 
                (ref_temp + SUTHERLAND_CONSTANT) / (temp + SUTHERLAND_CONSTANT);
    
    TS_ASSERT_DELTA(mu, DYNAMIC_VISCOSITY_REF, 1e-10);
  }

  // Test 14: Dynamic viscosity at different temperature
  void testDynamicViscosityAtAltitude() {
    double temp = 250.0; // K (cold temperature)
    double ref_temp = 288.15;
    double mu = DYNAMIC_VISCOSITY_REF * std::pow(temp / ref_temp, 1.5) * 
                (ref_temp + SUTHERLAND_CONSTANT) / (temp + SUTHERLAND_CONSTANT);
    
    // Viscosity decreases with temperature
    TS_ASSERT(mu < DYNAMIC_VISCOSITY_REF);
    TS_ASSERT_DELTA(mu, 1.598e-5, 1e-7);
  }

  // Test 15: Kinematic viscosity calculation
  void testKinematicViscosity() {
    double dynamic_visc = DYNAMIC_VISCOSITY_REF;
    double density = ISA_SEA_LEVEL_DENSITY;
    double kinematic_visc = dynamic_visc / density;
    
    // At sea level, kinematic viscosity ~1.46e-5 m²/s
    TS_ASSERT_DELTA(kinematic_visc, 1.461e-5, 1e-7);
  }

  // Test 16: Stratosphere temperature (isothermal above tropopause)
  void testStratosphereIsothermal() {
    double altitude1 = 15000.0; // Above tropopause
    double altitude2 = 20000.0; // Also above tropopause
    
    // In simple ISA model, stratosphere is isothermal at -56.5°C
    double temp1 = TROPOPAUSE_TEMP;
    double temp2 = TROPOPAUSE_TEMP;
    
    TS_ASSERT_DELTA(temp1, temp2, epsilon);
    TS_ASSERT_DELTA(temp1, 216.65, epsilon);
  }

  // Test 17: Pressure in stratosphere (exponential decay)
  void testStratospherePressure() {
    double altitude = 15000.0; // 4000m above tropopause
    double altitude_above_tropo = altitude - TROPOPAUSE_ALTITUDE;

    // First get pressure at tropopause
    double exponent_tropo = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double pressure_tropo = ISA_SEA_LEVEL_PRESSURE *
                           std::pow(TROPOPAUSE_TEMP / ISA_SEA_LEVEL_TEMP_K, exponent_tropo);

    // Then exponential decay in stratosphere
    double pressure = pressure_tropo *
                     std::exp(-GRAVITY_SI * altitude_above_tropo / (GAS_CONSTANT_AIR * TROPOPAUSE_TEMP));

    // At 15000m, pressure should be approximately 12045 Pa
    TS_ASSERT_DELTA(pressure, 12045.0, 10.0);
  }

  // Test 18: Density altitude concept (hot day)
  void testDensityAltitudeHotDay() {
    // On a hot day (30°C at sea level), density is lower
    double temp_hot = 273.15 + 30.0; // 303.15 K
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double density_hot = pressure / (GAS_CONSTANT_AIR * temp_hot);
    
    // Density should be less than standard
    TS_ASSERT(density_hot < ISA_SEA_LEVEL_DENSITY);
    TS_ASSERT_DELTA(density_hot, 1.164, 0.001);
  }

  // Test 19: Density altitude concept (cold day)
  void testDensityAltitudeColdDay() {
    // On a cold day (0°C at sea level), density is higher
    double temp_cold = 273.15; // 273.15 K
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double density_cold = pressure / (GAS_CONSTANT_AIR * temp_cold);
    
    // Density should be greater than standard
    TS_ASSERT(density_cold > ISA_SEA_LEVEL_DENSITY);
    TS_ASSERT_DELTA(density_cold, 1.292, 0.001);
  }

  // Test 20: Temperature deviation effects on pressure altitude
  void testTemperatureDeviationPressureAltitude() {
    // Same geometric altitude, different temperatures
    double altitude = 5000.0;
    double temp_std = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double temp_hot = temp_std + 15.0; // 15K warmer
    
    // Pressure altitude differs from geometric altitude on non-standard day
    double theta_std = temp_std / ISA_SEA_LEVEL_TEMP_K;
    double theta_hot = temp_hot / ISA_SEA_LEVEL_TEMP_K;
    
    TS_ASSERT(theta_hot > theta_std);
  }

  // Test 21: Indicated airspeed to true airspeed (no compressibility)
  void testIAStoTASIncompressible() {
    // At low speeds, IAS to TAS conversion uses density ratio
    double ias = 100.0; // m/s indicated
    double density_ratio = 0.8; // At some altitude
    double tas = ias / std::sqrt(density_ratio);
    
    TS_ASSERT_DELTA(tas, 111.803, 0.01);
    TS_ASSERT(tas > ias); // TAS > IAS at altitude
  }

  // Test 22: Calibrated airspeed equals indicated (no instrument error)
  void testCASEqualsIASNoError() {
    // In ideal case with no instrument/position error
    double ias = 150.0; // m/s
    double cas = ias; // No corrections
    
    TS_ASSERT_DELTA(cas, ias, epsilon);
  }

  // Test 23: Compressibility correction factor
  void testCompressibilityCorrectionFactor() {
    // At higher Mach numbers, compressibility affects airspeed
    double mach = 0.5;
    double impact_pressure_ratio = std::pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0;
    
    // Should be non-zero for M > 0
    TS_ASSERT(impact_pressure_ratio > 0.0);
    TS_ASSERT_DELTA(impact_pressure_ratio, 0.1860, 0.001);
  }

  // Test 24: Reynolds number calculation
  void testReynoldsNumber() {
    double density = ISA_SEA_LEVEL_DENSITY;
    double velocity = 50.0; // m/s
    double chord = 2.0; // m
    double dynamic_visc = DYNAMIC_VISCOSITY_REF;
    
    double reynolds = (density * velocity * chord) / dynamic_visc;
    
    TS_ASSERT_DELTA(reynolds, 6.845e6, 1e4);
  }

  // Test 25: Pressure coefficient calculation
  void testPressureCoefficient() {
    double static_pressure = 95000.0; // Pa
    double freestream_pressure = ISA_SEA_LEVEL_PRESSURE;
    double dynamic_pressure = 6000.0; // Pa
    
    double cp = (static_pressure - freestream_pressure) / dynamic_pressure;
    
    TS_ASSERT_DELTA(cp, -1.054, 0.001);
  }

  // Test 26: Geopotential altitude vs geometric altitude
  void testGeopotentialAltitude() {
    double geometric_altitude = 10000.0; // m
    double earth_radius = 6356766.0; // m (mean radius)
    double geopotential_altitude = (earth_radius * geometric_altitude) / 
                                   (earth_radius + geometric_altitude);
    
    // Geopotential altitude is slightly less than geometric
    TS_ASSERT(geopotential_altitude < geometric_altitude);
    TS_ASSERT_DELTA(geopotential_altitude, 9984.3, 0.1);
  }

  // Test 27: Humidity effect on density (approximate)
  void testHumidityEffectOnDensity() {
    // Moist air is less dense than dry air at same pressure and temperature
    // Using simple approximation: partial pressure of water vapor
    double temp = ISA_SEA_LEVEL_TEMP_K;
    double pressure_total = ISA_SEA_LEVEL_PRESSURE;
    double pressure_water_vapor = 1000.0; // Pa (some humidity)
    double pressure_dry_air = pressure_total - pressure_water_vapor;
    
    // Dry air density
    double density_dry_part = pressure_dry_air / (GAS_CONSTANT_AIR * temp);
    
    // Water vapor has different gas constant (461.5 J/(kg·K))
    double gas_constant_water = 461.5;
    double density_vapor_part = pressure_water_vapor / (gas_constant_water * temp);
    
    double density_moist = density_dry_part + density_vapor_part;
    
    // Moist air should be less dense than dry air
    double density_dry = pressure_total / (GAS_CONSTANT_AIR * temp);
    TS_ASSERT(density_moist < density_dry);
  }

  // Test 28: Equivalent airspeed calculation
  void testEquivalentAirspeed() {
    double true_airspeed = 200.0; // m/s
    double density_ratio = 0.7; // At altitude
    double equivalent_airspeed = true_airspeed * std::sqrt(density_ratio);
    
    TS_ASSERT_DELTA(equivalent_airspeed, 167.33, 0.01);
    TS_ASSERT(equivalent_airspeed < true_airspeed);
  }

  // Test 29: Standard atmosphere consistency check
  void testStandardAtmosphereConsistency() {
    // Verify that P = ρRT holds for standard conditions
    double pressure_calculated = ISA_SEA_LEVEL_DENSITY * GAS_CONSTANT_AIR * ISA_SEA_LEVEL_TEMP_K;
    
    TS_ASSERT_DELTA(pressure_calculated, ISA_SEA_LEVEL_PRESSURE, 1.0);
  }

  // Test 30: Lapse rate in different units
  void testLapseRateUnits() {
    // Verify -6.5°C/km = -0.0065 K/m
    double lapse_rate_per_km = -6.5; // °C/km or K/km
    double lapse_rate_per_m = lapse_rate_per_km / 1000.0;

    TS_ASSERT_DELTA(lapse_rate_per_m, ISA_LAPSE_RATE, epsilon);
  }

  /***************************************************************************
   * Extended Altitude Layer Tests
   ***************************************************************************/

  // Test 31: Temperature at base of stratosphere (20km)
  void testStratosphereBase20km() {
    // Still isothermal at 20km
    double temp = TROPOPAUSE_TEMP;  // -56.5°C
    TS_ASSERT_DELTA(temp, 216.65, epsilon);
  }

  // Test 32: Temperature profile upper stratosphere (above 20km)
  void testUpperStratosphereLapseRate() {
    // Above 20km, temperature increases again
    // Stratosphere 2 lapse rate: +0.001 K/m (from 20km to 32km)
    double base_altitude = 20000.0;
    double temp_at_20km = 216.65;
    double lapse_rate_strat2 = 0.001;  // K/m positive

    double altitude = 25000.0;
    double temp = temp_at_20km + lapse_rate_strat2 * (altitude - base_altitude);
    TS_ASSERT_DELTA(temp, 221.65, epsilon);
  }

  // Test 33: Mesosphere temperature profile (50-80km)
  void testMesosphereTemperature() {
    // In mesosphere, temperature decreases again
    // Peak at stratopause (~47km) around 270K
    // Decreases in mesosphere with lapse rate ~-0.0028 K/m
    double stratopause_temp = 270.65;
    double mesosphere_lapse = -0.0028;
    double altitude_above_stratopause = 10000.0;  // 10km above stratopause

    double temp = stratopause_temp + mesosphere_lapse * altitude_above_stratopause;
    TS_ASSERT_DELTA(temp, 242.65, 0.1);
  }

  /***************************************************************************
   * Non-Standard Day Tests
   ***************************************************************************/

  // Test 34: Hot day pressure altitude
  void testHotDayPressureAltitude() {
    // On ISA+20 day, pressure altitude is higher than geometric
    double geometric_alt = 5000.0;
    double temp_std = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * geometric_alt;
    double temp_hot = temp_std + 20.0;  // ISA+20

    // Pressure at geometric altitude is same, but temperature is higher
    // This means density is lower = higher density altitude
    double density_std = 1.0;  // Reference
    double density_hot = temp_std / temp_hot;  // Ratio

    TS_ASSERT(density_hot < density_std);
  }

  // Test 35: Cold day performance
  void testColdDayPerformance() {
    // On ISA-20 day at sea level
    double temp_cold = ISA_SEA_LEVEL_TEMP_K - 20.0;  // 268.15 K
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double density_cold = pressure / (GAS_CONSTANT_AIR * temp_cold);

    // Higher density = better performance
    TS_ASSERT(density_cold > ISA_SEA_LEVEL_DENSITY);
    TS_ASSERT_DELTA(density_cold, 1.317, 0.001);
  }

  // Test 36: High altitude airport (Denver)
  void testHighAltitudeAirport() {
    double field_elevation = 1609.0;  // Denver ~5280 ft = 1609 m
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * field_elevation;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double delta = std::pow(temp / ISA_SEA_LEVEL_TEMP_K, exponent);
    double pressure = ISA_SEA_LEVEL_PRESSURE * delta;

    TS_ASSERT_DELTA(pressure, 83431.0, 100.0);  // ~83.4 kPa
  }

  // Test 37: ISA deviation calculation
  void testISADeviation() {
    double actual_temp = 293.15;  // 20°C at sea level
    double isa_temp = ISA_SEA_LEVEL_TEMP_K;  // 288.15 K = 15°C
    double deviation = actual_temp - isa_temp;

    TS_ASSERT_DELTA(deviation, 5.0, epsilon);  // ISA+5
  }

  /***************************************************************************
   * Pressure Altitude vs Density Altitude Tests
   ***************************************************************************/

  // Test 38: Density altitude greater than pressure altitude (hot day)
  void testDensityAltitudeGreaterThanPressureAlt() {
    // Standard day: pressure alt = density alt = geometric alt
    // Hot day: density alt > pressure alt (because density is lower)
    double pressure_alt = 5000.0;  // m
    double temp_deviation = 15.0;  // ISA+15

    // Density altitude approximation
    double density_alt = pressure_alt + 120.0 * temp_deviation;  // Rule of thumb

    TS_ASSERT(density_alt > pressure_alt);
    TS_ASSERT_DELTA(density_alt, 6800.0, 100.0);
  }

  // Test 39: Density altitude less than pressure altitude (cold day)
  void testDensityAltitudeLessThanPressureAlt() {
    double pressure_alt = 5000.0;
    double temp_deviation = -10.0;  // ISA-10

    double density_alt = pressure_alt + 120.0 * temp_deviation;

    TS_ASSERT(density_alt < pressure_alt);
    TS_ASSERT_DELTA(density_alt, 3800.0, 100.0);
  }

  /***************************************************************************
   * More Airspeed Conversion Tests
   ***************************************************************************/

  // Test 40: TAS to EAS conversion
  void testTASToEAS() {
    double tas = 200.0;  // m/s
    double sigma = 0.65;  // density ratio at altitude
    double eas = tas * std::sqrt(sigma);

    TS_ASSERT_DELTA(eas, 161.24, 0.1);
  }

  // Test 41: Mach to TAS conversion
  void testMachToTAS() {
    double mach = 0.8;
    double temp = 223.15;  // At ~10km altitude
    double sound_speed = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * temp);
    double tas = mach * sound_speed;

    TS_ASSERT_DELTA(tas, 239.63, 0.1);
  }

  // Test 42: Ground speed from TAS and wind
  void testGroundSpeedWithWind() {
    double tas = 150.0;  // m/s
    double headwind = 20.0;  // m/s
    double groundspeed = tas - headwind;

    TS_ASSERT_DELTA(groundspeed, 130.0, epsilon);

    // Tailwind
    double tailwind = 20.0;
    groundspeed = tas + tailwind;
    TS_ASSERT_DELTA(groundspeed, 170.0, epsilon);
  }

  // Test 43: Crosswind component
  void testCrosswindComponent() {
    double wind_speed = 30.0;  // m/s
    double wind_angle = 30.0 * M_PI / 180.0;  // 30 degrees off runway
    double crosswind = wind_speed * std::sin(wind_angle);
    double headwind = wind_speed * std::cos(wind_angle);

    TS_ASSERT_DELTA(crosswind, 15.0, 0.1);
    TS_ASSERT_DELTA(headwind, 25.98, 0.1);
  }

  /***************************************************************************
   * Stagnation/Total Temperature Tests
   ***************************************************************************/

  // Test 44: Total temperature (stagnation)
  void testTotalTemperature() {
    double static_temp = 223.15;  // K at altitude
    double mach = 0.8;
    double total_temp = static_temp * (1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach * mach);

    TS_ASSERT_DELTA(total_temp, 251.77, 0.1);
  }

  // Test 45: Recovery factor for temperature probe
  void testRecoveryFactor() {
    double static_temp = 223.15;
    double mach = 0.8;
    double recovery_factor = 0.98;  // Typical for TAT probe

    double total_temp = static_temp * (1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach * mach);
    double measured_temp = static_temp +
                          recovery_factor * (total_temp - static_temp);

    TS_ASSERT(measured_temp < total_temp);
    TS_ASSERT(measured_temp > static_temp);
  }

  /***************************************************************************
   * Pitot-Static Tests
   ***************************************************************************/

  // Test 46: Impact pressure (subsonic)
  void testImpactPressureSubsonic() {
    double mach = 0.5;
    double static_pressure = ISA_SEA_LEVEL_PRESSURE;

    // Subsonic: qc = P0 * ((1 + 0.2*M^2)^3.5 - 1)
    double qc = static_pressure * (std::pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0);

    TS_ASSERT_DELTA(qc, 18868.0, 50.0);
  }

  // Test 47: Total pressure from Mach
  void testTotalPressureFromMach() {
    double mach = 0.6;
    double static_pressure = 80000.0;  // Pa at altitude

    double total_pressure = static_pressure *
                           std::pow(1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach * mach,
                                   GAMMA_AIR / (GAMMA_AIR - 1.0));

    TS_ASSERT_DELTA(total_pressure, 102040.0, 50.0);
  }

  // Test 48: Static pressure from pitot and total
  void testStaticPressureRecovery() {
    double total_pressure = 105000.0;
    double impact_pressure = 4000.0;
    double static_pressure = total_pressure - impact_pressure;

    TS_ASSERT_DELTA(static_pressure, 101000.0, epsilon);
  }

  /***************************************************************************
   * Reynolds Number Extended Tests
   ***************************************************************************/

  // Test 49: Reynolds number at altitude
  void testReynoldsNumberAtAltitude() {
    double altitude = 10000.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double theta = temp / ISA_SEA_LEVEL_TEMP_K;
    double delta = std::pow(theta, -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR));
    double sigma = delta / theta;

    double density = ISA_SEA_LEVEL_DENSITY * sigma;
    double velocity = 250.0;  // m/s TAS
    double chord = 3.0;  // m
    double mu = DYNAMIC_VISCOSITY_REF * std::pow(theta, 1.5) *
               (ISA_SEA_LEVEL_TEMP_K + SUTHERLAND_CONSTANT) /
               (temp + SUTHERLAND_CONSTANT);

    double reynolds = (density * velocity * chord) / mu;

    // Reynolds number at altitude is lower due to lower density
    TS_ASSERT(reynolds < 1e8);
    TS_ASSERT(reynolds > 1e7);
  }

  // Test 50: Critical Reynolds number for transition
  void testCriticalReynoldsNumber() {
    // Transition from laminar to turbulent typically 3e5 to 5e5
    double re_critical = 5e5;
    double velocity = 50.0;  // m/s
    double x_transition = (re_critical * DYNAMIC_VISCOSITY_REF) /
                         (ISA_SEA_LEVEL_DENSITY * velocity);

    // x = Re * mu / (rho * v) = 5e5 * 1.79e-5 / (1.225 * 50) = 0.146m
    TS_ASSERT_DELTA(x_transition, 0.146, 0.01);
  }

  /***************************************************************************
   * Altitude Performance Tests
   ***************************************************************************/

  // Test 51: Thrust reduction with altitude (jet engine)
  void testThrustReductionWithAltitude() {
    // Jet thrust roughly proportional to density ratio
    double sigma = 0.65;  // At ~15000 ft
    double sea_level_thrust = 25000.0;  // N
    double altitude_thrust = sea_level_thrust * sigma;

    TS_ASSERT_DELTA(altitude_thrust, 16250.0, 1.0);
  }

  // Test 52: Power required vs altitude
  void testPowerRequiredVsAltitude() {
    // For same TAS, induced drag is same but parasite drag lower
    double tas = 100.0;  // m/s
    double cd_parasite = 0.02;
    double s = 30.0;  // wing area m^2

    double rho_sl = ISA_SEA_LEVEL_DENSITY;
    double rho_alt = 0.9 * rho_sl;

    double drag_sl = 0.5 * rho_sl * tas * tas * s * cd_parasite;
    double drag_alt = 0.5 * rho_alt * tas * tas * s * cd_parasite;

    TS_ASSERT(drag_alt < drag_sl);
  }

  // Test 53: Rate of climb calculation
  void testRateOfClimb() {
    double excess_power = 50000.0;  // Watts
    double weight = 10000.0;  // N
    double roc = excess_power / weight;  // m/s

    TS_ASSERT_DELTA(roc, 5.0, epsilon);  // 5 m/s = 984 fpm
  }

  // Test 54: Service ceiling (ROC = 0.5 m/s)
  void testServiceCeiling() {
    // Service ceiling defined as altitude where ROC = 100 ft/min = 0.508 m/s
    double roc_service = 0.508;

    // At service ceiling, excess thrust barely exceeds drag
    TS_ASSERT_DELTA(roc_service, 0.508, 0.01);
  }

  /***************************************************************************
   * Specific Energy Tests
   ***************************************************************************/

  // Test 55: Specific energy (energy height)
  void testSpecificEnergy() {
    double altitude = 5000.0;  // m
    double velocity = 200.0;   // m/s
    double g = GRAVITY_SI;

    double specific_energy = altitude + (velocity * velocity) / (2.0 * g);
    double energy_height = specific_energy;

    TS_ASSERT_DELTA(energy_height, 7039.4, 0.5);
  }

  // Test 56: Energy exchange (zoom climb)
  void testEnergyExchangeZoomClimb() {
    double initial_alt = 5000.0;
    double initial_vel = 250.0;
    double g = GRAVITY_SI;

    double initial_energy = initial_alt + (initial_vel * initial_vel) / (2.0 * g);

    // Zoom climb: trade speed for altitude
    double final_vel = 100.0;
    double final_energy = initial_energy;  // Conservation
    double final_alt = final_energy - (final_vel * final_vel) / (2.0 * g);

    TS_ASSERT(final_alt > initial_alt);
    TS_ASSERT_DELTA(final_alt, 7675.0, 10.0);
  }

  /***************************************************************************
   * Extreme Condition Tests
   ***************************************************************************/

  // Test 57: Arctic conditions (-50°C at sea level)
  void testArcticConditions() {
    double temp = 273.15 - 50.0;  // 223.15 K
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double density = pressure / (GAS_CONSTANT_AIR * temp);

    TS_ASSERT_DELTA(density, 1.582, 0.001);  // Much denser than standard
  }

  // Test 58: Tropical conditions (+40°C at sea level)
  void testTropicalConditions() {
    double temp = 273.15 + 40.0;  // 313.15 K
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double density = pressure / (GAS_CONSTANT_AIR * temp);

    TS_ASSERT_DELTA(density, 1.127, 0.001);  // Less dense than standard
  }

  // Test 59: High altitude cruise (FL410)
  void testHighAltitudeCruise() {
    double altitude = 12497.0;  // FL410 = 41000 ft = 12497 m
    // Above tropopause, so isothermal

    double altitude_above_tropo = altitude - TROPOPAUSE_ALTITUDE;
    double exponent_tropo = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double pressure_tropo = ISA_SEA_LEVEL_PRESSURE *
                           std::pow(TROPOPAUSE_TEMP / ISA_SEA_LEVEL_TEMP_K, exponent_tropo);

    double pressure = pressure_tropo *
                     std::exp(-GRAVITY_SI * altitude_above_tropo /
                             (GAS_CONSTANT_AIR * TROPOPAUSE_TEMP));

    TS_ASSERT_DELTA(pressure, 17873.0, 100.0);  // ~179 hPa
  }

  // Test 60: Space boundary (Karman line 100km)
  void testKarmanLine() {
    double altitude = 100000.0;  // m
    // At Karman line, air is extremely thin

    // Rough approximation using barometric formula
    double scale_height = 8500.0;  // Approximate
    double pressure_ratio = std::exp(-altitude / scale_height);

    TS_ASSERT(pressure_ratio < 1e-5);  // Very low pressure
  }

  /***************************************************************************
   * Humidity and Water Vapor Tests
   ***************************************************************************/

  // Test 61: Saturation vapor pressure (Magnus formula)
  void testSaturationVaporPressure() {
    double temp_c = 20.0;  // Celsius
    double es = 610.78 * std::exp((17.27 * temp_c) / (temp_c + 237.3));

    TS_ASSERT_DELTA(es, 2339.0, 10.0);  // ~23.4 hPa
  }

  // Test 62: Relative humidity effect on density
  void testRelativeHumidityEffect() {
    double temp = 293.15;  // 20°C
    double pressure = ISA_SEA_LEVEL_PRESSURE;
    double rh = 0.8;  // 80% RH

    double temp_c = temp - 273.15;
    double es = 610.78 * std::exp((17.27 * temp_c) / (temp_c + 237.3));
    double e = rh * es;  // Vapor pressure

    // Virtual temperature accounts for humidity
    double tv = temp / (1.0 - (e / pressure) * (1.0 - 0.622));
    double density_moist = pressure / (GAS_CONSTANT_AIR * tv);

    // Moist air less dense
    double density_dry = pressure / (GAS_CONSTANT_AIR * temp);
    TS_ASSERT(density_moist < density_dry);
  }

  // Test 63: Dew point calculation
  void testDewPointCalculation() {
    double temp = 25.0;  // Celsius
    double rh = 0.6;  // 60% RH

    // Magnus approximation for dew point
    double a = 17.27, b = 237.3;
    double alpha = ((a * temp) / (b + temp)) + std::log(rh);
    double dew_point = (b * alpha) / (a - alpha);

    TS_ASSERT_DELTA(dew_point, 16.7, 0.5);
  }

  /***************************************************************************
   * Instrument Error Tests
   ***************************************************************************/

  // Test 64: Altimeter error with non-standard pressure
  void testAltimeterError() {
    // Altimeter set to 1013.25 hPa but actual pressure is 1000 hPa
    double set_pressure = 101325.0;  // Pa
    double actual_pressure = 100000.0;  // Pa

    // Each 1 hPa difference = ~27 ft = ~8.2 m
    double pressure_diff = (set_pressure - actual_pressure) / 100.0;  // in hPa
    double altitude_error = pressure_diff * 8.23;  // m

    TS_ASSERT_DELTA(altitude_error, 109.3, 1.0);
  }

  // Test 65: Temperature error in altimeter
  void testAltimeterTempError() {
    // On cold day, true altitude is LOWER than indicated
    // "High to low, look out below"
    double indicated_alt = 5000.0;
    double isa_temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * indicated_alt;
    double actual_temp = isa_temp - 20.0;  // 20K colder

    // Approximately 4% per 10°C error
    double error_percent = 0.04 * 20.0 / 10.0;
    double true_alt = indicated_alt * (1.0 - error_percent);

    TS_ASSERT(true_alt < indicated_alt);
  }

  /***************************************************************************
   * More Pressure Calculations
   ***************************************************************************/

  // Test 66: QNH calculation
  void testQNHCalculation() {
    double field_pressure = 99000.0;  // Pa at field
    double field_elevation = 500.0;   // m

    // QNH reduces field pressure to sea level using ISA lapse rate
    // p_sl = p_field / delta, where delta = (T/T0)^exponent
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * field_elevation;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double delta = std::pow(temp / ISA_SEA_LEVEL_TEMP_K, exponent);
    double qnh = field_pressure / delta;

    // With field_pressure=99000 Pa, delta~0.942, QNH~105kPa
    TS_ASSERT(qnh > 100000.0);  // QNH should be higher than field pressure
  }

  // Test 67: Flight level pressure
  void testFlightLevelPressure() {
    // FL350 = 35000 ft = 10668 m
    // Using standard atmosphere
    double altitude = 10668.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;

    // Clamp to tropopause
    if (altitude > TROPOPAUSE_ALTITUDE) {
      temp = TROPOPAUSE_TEMP;
    }

    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double pressure = ISA_SEA_LEVEL_PRESSURE *
                     std::pow(TROPOPAUSE_TEMP / ISA_SEA_LEVEL_TEMP_K, exponent);

    // Multiply by exponential for above tropopause
    double altitude_above = altitude - TROPOPAUSE_ALTITUDE;
    if (altitude_above > 0) {
      pressure *= std::exp(-GRAVITY_SI * altitude_above /
                          (GAS_CONSTANT_AIR * TROPOPAUSE_TEMP));
    }

    TS_ASSERT(pressure < 30000.0);  // Less than 300 hPa
  }

  // Test 68: Standard atmosphere table value verification
  void testStandardAtmosphereTable() {
    // At 1000m: P = 89875 Pa, T = 281.65 K, rho = 1.112 kg/m³
    double altitude = 1000.0;
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    TS_ASSERT_DELTA(temp, 281.65, 0.01);

    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double pressure = ISA_SEA_LEVEL_PRESSURE *
                     std::pow(temp / ISA_SEA_LEVEL_TEMP_K, exponent);
    TS_ASSERT_DELTA(pressure, 89875.0, 50.0);

    double density = pressure / (GAS_CONSTANT_AIR * temp);
    TS_ASSERT_DELTA(density, 1.112, 0.001);
  }

  // Test 69: Pressure doubling altitude
  void testPressureDoublingAltitude() {
    // Pressure halves roughly every 5500m
    double half_pressure_alt = 5500.0;  // Approximate

    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * half_pressure_alt;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double pressure_ratio = std::pow(temp / ISA_SEA_LEVEL_TEMP_K, exponent);

    TS_ASSERT_DELTA(pressure_ratio, 0.5, 0.02);
  }

  // Test 70: Pressure at different reference temperatures
  void testPressureAtDifferentTemps() {
    double altitude = 3000.0;

    // ISA
    double temp_isa = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double delta_isa = std::pow(temp_isa / ISA_SEA_LEVEL_TEMP_K, exponent);

    // ISA+20
    double temp_hot = temp_isa + 20.0;
    double delta_hot = std::pow(temp_hot / (ISA_SEA_LEVEL_TEMP_K + 20.0), exponent);

    // Both should give reasonable pressure ratios
    TS_ASSERT(delta_isa > 0.5 && delta_isa < 0.9);
    TS_ASSERT(delta_hot > 0.5 && delta_hot < 0.9);
  }

  // Test 71: Speed of sound variation with altitude
  void testSpeedOfSoundVariation() {
    // Speed of sound at different altitudes
    double a_sl = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * ISA_SEA_LEVEL_TEMP_K);
    double a_5k = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * (ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * 5000));
    double a_tropo = std::sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * TROPOPAUSE_TEMP);

    TS_ASSERT(a_sl > a_5k);
    TS_ASSERT(a_5k > a_tropo);
    TS_ASSERT_DELTA(a_sl, 340.3, 0.1);
    TS_ASSERT_DELTA(a_tropo, 295.1, 0.1);
  }

  // Test 72: Maximum operating altitude (coffin corner concept)
  void testCoffinCornerConcept() {
    // At high altitude, Mmo and stall speed converge
    // This is the aerodynamic ceiling
    double mmo = 0.82;
    double a = 295.0;  // Speed of sound at altitude
    double max_speed = mmo * a;

    // Stall speed increases with altitude (same EAS = higher TAS)
    double stall_eas = 80.0;  // m/s
    double sigma = 0.3;  // Very high altitude
    double stall_tas = stall_eas / std::sqrt(sigma);

    // At coffin corner, these converge
    TS_ASSERT(stall_tas > 100.0);  // Much higher than at sea level
  }

  // Test 73: Atmospheric scale height
  void testAtmosphericScaleHeight() {
    // Scale height H = RT/g
    double scale_height = (GAS_CONSTANT_AIR * ISA_SEA_LEVEL_TEMP_K) / GRAVITY_SI;

    TS_ASSERT_DELTA(scale_height, 8434.5, 1.0);  // ~8.4 km
  }

  // Test 74: Air mass calculation
  void testAirMassCalculation() {
    // Column of air above a point
    double surface_pressure = ISA_SEA_LEVEL_PRESSURE;  // Pa = N/m^2
    double area = 1.0;  // m^2
    double force = surface_pressure * area;
    double mass = force / GRAVITY_SI;  // kg

    TS_ASSERT_DELTA(mass, 10332.0, 10.0);  // ~10.3 tonnes per m^2
  }

  // Test 75: Virtual temperature calculation
  void testVirtualTemperature() {
    double temp = 288.15;
    double mixing_ratio = 0.01;  // kg/kg (1% moisture)

    // Virtual temperature is higher due to water vapor
    double tv = temp * (1.0 + 0.608 * mixing_ratio);

    TS_ASSERT(tv > temp);
    TS_ASSERT_DELTA(tv, 289.9, 0.1);
  }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DOCUMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/** Extended test suite for advanced atmospheric calculations
 */

class FGAtmosphericModelExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Temperature Inversion and Stability Tests
   ***************************************************************************/

  // Test 76: Temperature inversion layer detection
  void testTemperatureInversion() {
    // Normal: temperature decreases with altitude
    // Inversion: temperature increases with altitude
    double temp_low = 280.0;   // K at lower altitude
    double temp_high = 285.0;  // K at higher altitude (inversion)

    // Inversion exists when temp increases with altitude
    bool inversion = temp_high > temp_low;
    TS_ASSERT(inversion);

    double inversion_strength = temp_high - temp_low;
    TS_ASSERT_DELTA(inversion_strength, 5.0, epsilon);
  }

  // Test 77: Atmospheric stability (Brunt-Väisälä frequency)
  void testBruntVaisalaFrequency() {
    // N = sqrt((g/theta) * d(theta)/dz)
    double g = GRAVITY_SI;
    double theta = 300.0;  // Potential temperature K
    double dtheta_dz = 0.003;  // K/m (stable atmosphere)

    double N_squared = (g / theta) * dtheta_dz;
    double N = std::sqrt(N_squared);  // Brunt-Väisälä frequency

    TS_ASSERT(N > 0.0);  // Positive = stable
    TS_ASSERT_DELTA(N, 0.01, 0.001);
  }

  // Test 78: Potential temperature calculation
  void testPotentialTemperature() {
    // theta = T * (p0/p)^(R/cp)
    double temp = 250.0;  // K at altitude
    double pressure = 50000.0;  // Pa
    double R_over_cp = GAS_CONSTANT_AIR / 1005.0;  // ~0.286

    double theta = temp * std::pow(ISA_SEA_LEVEL_PRESSURE / pressure, R_over_cp);

    TS_ASSERT(theta > temp);  // Potential temp > actual temp at altitude
    TS_ASSERT_DELTA(theta, 305.9, 0.5);
  }

  // Test 79: Lapse rate vs adiabatic lapse rate
  void testLapseRateComparison() {
    // Dry adiabatic lapse rate ~9.8 K/km
    double dry_adiabatic = -9.8 / 1000.0;  // K/m

    // ISA lapse rate is less steep (atmosphere is stable)
    TS_ASSERT(ISA_LAPSE_RATE > dry_adiabatic);  // Less negative
    TS_ASSERT_DELTA(ISA_LAPSE_RATE, -0.0065, epsilon);
  }

  /***************************************************************************
   * Icing Condition Tests
   ***************************************************************************/

  // Test 80: Icing temperature range
  void testIcingTemperatureRange() {
    // Icing typically occurs between 0°C and -20°C
    double temp_upper = 273.15;  // 0°C
    double temp_lower = 253.15;  // -20°C

    double test_temp = 263.15;  // -10°C - in icing range

    bool in_icing_range = (test_temp <= temp_upper) && (test_temp >= temp_lower);
    TS_ASSERT(in_icing_range);
  }

  // Test 81: Supercooled water droplet region
  void testSupercooledWater() {
    // Supercooled large droplets (SLD) can exist below 0°C
    double temp = 263.15;  // -10°C
    double temp_celsius = temp - 273.15;

    // SLD freezes on contact with aircraft
    bool supercooled = temp_celsius < 0.0;
    TS_ASSERT(supercooled);
  }

  /***************************************************************************
   * Contrail Formation Tests
   ***************************************************************************/

  // Test 82: Contrail formation (Schmidt-Appleman criterion)
  void testContrailFormation() {
    // Contrails form when exhaust mixes with cold, humid air
    double ambient_temp = 220.0;  // K (-53°C typical at cruise)
    double threshold_temp = 233.0;  // K approximate threshold

    // Contrails likely when ambient temp < threshold
    bool contrails_likely = ambient_temp < threshold_temp;
    TS_ASSERT(contrails_likely);
  }

  // Test 83: Contrail persistence (humidity dependent)
  void testContrailPersistence() {
    // Persistent contrails form in ice-supersaturated air
    double relative_humidity_ice = 1.1;  // 110% RH with respect to ice

    bool persistent = relative_humidity_ice > 1.0;
    TS_ASSERT(persistent);
  }

  /***************************************************************************
   * Wind and Shear Tests
   ***************************************************************************/

  // Test 84: Wind shear calculation
  void testWindShear() {
    double wind_upper = 50.0;  // m/s at upper level
    double wind_lower = 20.0;  // m/s at lower level
    double altitude_diff = 1000.0;  // m

    double wind_shear = (wind_upper - wind_lower) / altitude_diff;  // per second

    TS_ASSERT_DELTA(wind_shear, 0.03, 0.001);
  }

  // Test 85: Jet stream core speed
  void testJetStreamCore() {
    // Typical jet stream speeds
    double jet_core_speed = 75.0;  // m/s (~150 kt)
    double jet_width = 200000.0;   // m (200 km width)
    double jet_depth = 3000.0;     // m

    TS_ASSERT(jet_core_speed > 25.0);  // Definition: > 25 m/s
    TS_ASSERT_DELTA(jet_core_speed, 75.0, epsilon);
  }

  // Test 86: Thermal wind equation
  void testThermalWind() {
    // Thermal wind is proportional to horizontal temperature gradient
    double delta_T = 10.0;  // K temperature difference
    double delta_x = 500000.0;  // m (500 km)
    double temp_gradient = delta_T / delta_x;

    // f = 2 * Omega * sin(lat) ~ 1e-4 at mid-latitudes
    double f = 1.0e-4;

    // Thermal wind shear: dV/dz ~ (g/f*T) * dT/dx
    double g = GRAVITY_SI;
    double T = 250.0;  // Mean temp
    double thermal_wind_shear = (g / (f * T)) * temp_gradient;

    TS_ASSERT(thermal_wind_shear > 0.0);
  }

  /***************************************************************************
   * Isentropic Flow Relations
   ***************************************************************************/

  // Test 87: Isentropic pressure ratio
  void testIsentropicPressureRatio() {
    double mach = 0.8;
    double gamma = GAMMA_AIR;

    // P0/P = (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1))
    double pressure_ratio = std::pow(1.0 + 0.5 * (gamma - 1.0) * mach * mach,
                                     gamma / (gamma - 1.0));

    TS_ASSERT_DELTA(pressure_ratio, 1.524, 0.001);
  }

  // Test 88: Isentropic temperature ratio
  void testIsentropicTemperatureRatio() {
    double mach = 0.8;
    double gamma = GAMMA_AIR;

    // T0/T = 1 + (gamma-1)/2 * M^2
    double temp_ratio = 1.0 + 0.5 * (gamma - 1.0) * mach * mach;

    TS_ASSERT_DELTA(temp_ratio, 1.128, 0.001);
  }

  // Test 89: Isentropic density ratio
  void testIsentropicDensityRatio() {
    double mach = 0.8;
    double gamma = GAMMA_AIR;

    // rho0/rho = (1 + (gamma-1)/2 * M^2)^(1/(gamma-1))
    double density_ratio = std::pow(1.0 + 0.5 * (gamma - 1.0) * mach * mach,
                                    1.0 / (gamma - 1.0));

    TS_ASSERT_DELTA(density_ratio, 1.351, 0.001);
  }

  /***************************************************************************
   * High-Speed Aerodynamic Heating Tests
   ***************************************************************************/

  // Test 90: Stagnation temperature at supersonic speed
  void testStagnationTemperatureSupersonic() {
    double mach = 2.0;
    double static_temp = 216.65;  // K at tropopause

    double T0 = static_temp * (1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach * mach);

    TS_ASSERT_DELTA(T0, 390.0, 1.0);  // Significant heating at M=2
  }

  // Test 91: Skin friction heating
  void testSkinFrictionHeating() {
    double mach = 0.85;
    double recovery_factor = 0.9;  // For turbulent boundary layer
    double static_temp = 220.0;

    double total_temp = static_temp * (1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach * mach);
    double adiabatic_wall_temp = static_temp + recovery_factor * (total_temp - static_temp);

    TS_ASSERT(adiabatic_wall_temp > static_temp);
    TS_ASSERT(adiabatic_wall_temp < total_temp);
  }

  /***************************************************************************
   * Flight Planning Atmospheric Tests
   ***************************************************************************/

  // Test 92: Standard temperature at various flight levels
  void testStandardTempFlightLevels() {
    // FL100 = 10000 ft = 3048 m
    double alt_fl100 = 3048.0;
    double temp_fl100 = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * alt_fl100;
    TS_ASSERT_DELTA(temp_fl100, 268.3, 0.5);

    // FL250 = 25000 ft = 7620 m
    double alt_fl250 = 7620.0;
    double temp_fl250 = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * alt_fl250;
    TS_ASSERT_DELTA(temp_fl250, 238.6, 0.5);
  }

  // Test 93: ICAO standard atmosphere verification
  void testICAOStandardAtmosphere() {
    // Verify standard values at key altitudes
    // At 5000m: T = 255.65 K
    double temp_5k = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * 5000.0;
    TS_ASSERT_DELTA(temp_5k, 255.65, 0.01);

    // At 10000m: T = 223.15 K
    double temp_10k = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * 10000.0;
    TS_ASSERT_DELTA(temp_10k, 223.15, 0.01);
  }

  // Test 94: True altitude correction
  void testTrueAltitudeCorrection() {
    // True altitude = indicated + correction for temperature deviation
    double indicated_alt = 10000.0;  // ft
    double isa_dev = 10.0;  // ISA+10

    // Correction factor: ~4 ft per degree per 1000 ft
    double correction = (4.0 * isa_dev * indicated_alt) / 1000.0;
    double true_alt = indicated_alt + correction;

    TS_ASSERT(true_alt > indicated_alt);
    TS_ASSERT_DELTA(true_alt, 10400.0, 10.0);
  }

  /***************************************************************************
   * Atmospheric Composition Tests
   ***************************************************************************/

  // Test 95: Oxygen partial pressure at altitude
  void testOxygenPartialPressure() {
    double altitude = 8000.0;  // m
    double temp = ISA_SEA_LEVEL_TEMP_K + ISA_LAPSE_RATE * altitude;
    double exponent = -GRAVITY_SI / (ISA_LAPSE_RATE * GAS_CONSTANT_AIR);
    double pressure = ISA_SEA_LEVEL_PRESSURE * std::pow(temp / ISA_SEA_LEVEL_TEMP_K, exponent);

    // Oxygen is 21% of air
    double O2_fraction = 0.21;
    double O2_partial = pressure * O2_fraction;

    // At sea level, O2 partial pressure ~21.3 kPa
    double O2_sea_level = ISA_SEA_LEVEL_PRESSURE * O2_fraction;

    TS_ASSERT(O2_partial < O2_sea_level);
    TS_ASSERT_DELTA(O2_sea_level, 21278.0, 10.0);
  }

  // Test 96: Cabin altitude from cabin pressure
  void testCabinAltitude() {
    // Aircraft pressurized to equivalent of 8000 ft = 2438 m
    double cabin_pressure = 75250.0;  // Pa (~752.5 hPa)

    // Calculate equivalent altitude
    double pressure_ratio = cabin_pressure / ISA_SEA_LEVEL_PRESSURE;
    double exponent = (ISA_LAPSE_RATE * GAS_CONSTANT_AIR) / (-GRAVITY_SI);
    double temp_ratio = std::pow(pressure_ratio, exponent);
    double cabin_altitude = (ISA_SEA_LEVEL_TEMP_K * (temp_ratio - 1.0)) / ISA_LAPSE_RATE;

    TS_ASSERT_DELTA(cabin_altitude, 2438.0, 50.0);
  }

  /***************************************************************************
   * Turbulence Intensity Tests
   ***************************************************************************/

  // Test 97: Turbulence intensity classification
  void testTurbulenceIntensity() {
    // Light: < 0.5 g
    // Moderate: 0.5-1.0 g
    // Severe: > 1.0 g
    double acceleration = 0.7;  // g

    bool light = acceleration < 0.5;
    bool moderate = (acceleration >= 0.5) && (acceleration <= 1.0);
    bool severe = acceleration > 1.0;

    TS_ASSERT(!light);
    TS_ASSERT(moderate);
    TS_ASSERT(!severe);
  }

  // Test 98: Convective available potential energy (CAPE)
  void testCAPE() {
    // CAPE indicates thunderstorm potential
    // CAPE > 2500 J/kg = severe thunderstorm potential
    double cape = 3000.0;  // J/kg

    bool severe_potential = cape > 2500.0;
    TS_ASSERT(severe_potential);

    // Estimated updraft velocity: W = sqrt(2 * CAPE)
    double max_updraft = std::sqrt(2.0 * cape);
    TS_ASSERT_DELTA(max_updraft, 77.5, 0.5);  // m/s
  }

  /***************************************************************************
   * Atmospheric Boundary Layer Tests
   ***************************************************************************/

  // Test 99: Planetary boundary layer height
  void testPBLHeight() {
    // Daytime convective boundary layer ~1-2 km
    double pbl_height = 1500.0;  // m typical daytime

    // At night, stable boundary layer is much thinner
    double sbl_height = 200.0;  // m

    TS_ASSERT(pbl_height > sbl_height);
    TS_ASSERT(pbl_height > 1000.0);
  }

  // Test 100: Wind profile power law
  void testWindProfilePowerLaw() {
    // Wind speed increases with height: V(z) = V_ref * (z/z_ref)^alpha
    double v_ref = 10.0;  // m/s at reference height
    double z_ref = 10.0;  // m reference height
    double z = 100.0;     // m height of interest
    double alpha = 0.14;  // Power law exponent (neutral conditions)

    double v_z = v_ref * std::pow(z / z_ref, alpha);

    TS_ASSERT(v_z > v_ref);
    TS_ASSERT_DELTA(v_z, 13.8, 0.1);
  }
};
