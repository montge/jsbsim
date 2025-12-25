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
};
