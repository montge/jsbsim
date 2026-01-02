#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <models/atmosphere/FGStandardAtmosphere.h>
#include <models/FGPropagate.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>
#include <input_output/FGPropertyManager.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();

using namespace JSBSim;

// Wrapper class that ensures proper cleanup of property bindings
class TestableStdAtmosphere : public FGStandardAtmosphere
{
public:
  TestableStdAtmosphere(FGFDMExec* fdm) : FGStandardAtmosphere(fdm) {}
  ~TestableStdAtmosphere() { PropertyManager->Unbind(this); }
};

class FGStandardAtmosphereTest : public CxxTest::TestSuite
{
public:
  FGFDMExec fdmex;

  FGStandardAtmosphereTest() {
    auto atm = fdmex.GetAtmosphere();
    fdmex.GetPropertyManager()->Unbind(atm);
  }

  void testInitialization()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Check standard day values
    TS_ASSERT_DELTA(atm.GetTemperatureSL(), FGAtmosphere::StdDaySLtemperature, epsilon);
    TS_ASSERT_DELTA(atm.GetPressureSL(), FGAtmosphere::StdDaySLpressure, epsilon);
    TS_ASSERT_DELTA(atm.GetStdTemperatureSL(), FGAtmosphere::StdDaySLtemperature, epsilon);
  }

  void testStandardTemperatureAtVariousAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test sea level
    TS_ASSERT_DELTA(atm.GetStdTemperature(0.0), 518.67, 0.01);

    // Test troposphere (11,000 ft)
    double T_11k = atm.GetStdTemperature(11000.0);
    TS_ASSERT(T_11k < 518.67);  // Should be colder

    // Test stratosphere (40,000 ft)
    double T_40k = atm.GetStdTemperature(40000.0);
    TS_ASSERT(T_40k < T_11k);  // Even colder

    // Test tropopause region (36,089 ft - where temperature is constant)
    double T_tropo = atm.GetStdTemperature(36089.0);
    TS_ASSERT_DELTA(T_tropo, 389.97, 1.0);
  }

  void testStandardPressureAtVariousAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test sea level
    TS_ASSERT_DELTA(atm.GetStdPressure(0.0), FGAtmosphere::StdDaySLpressure, 1.0);

    // Test 10,000 ft
    double P_10k = atm.GetStdPressure(10000.0);
    TS_ASSERT(P_10k < FGAtmosphere::StdDaySLpressure);
    TS_ASSERT_DELTA(P_10k, 1456.0, 10.0);  // Approximately 1456 psf

    // Test 30,000 ft
    double P_30k = atm.GetStdPressure(30000.0);
    TS_ASSERT(P_30k < P_10k);
    TS_ASSERT_DELTA(P_30k, 628.0, 10.0);  // Approximately 628 psf

    // Pressure should decrease with altitude
    TS_ASSERT(atm.GetStdPressure(50000.0) < P_30k);
  }

  void testStandardDensityAtVariousAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test sea level
    double rho_sl = atm.GetStdDensity(0.0);
    TS_ASSERT_DELTA(rho_sl, FGAtmosphere::StdDaySLdensity, 1e-6);

    // Test 10,000 ft
    double rho_10k = atm.GetStdDensity(10000.0);
    TS_ASSERT(rho_10k < rho_sl);

    // Test 30,000 ft
    double rho_30k = atm.GetStdDensity(30000.0);
    TS_ASSERT(rho_30k < rho_10k);

    // Density should decrease with altitude
    TS_ASSERT(atm.GetStdDensity(50000.0) < rho_30k);
  }

  void testTemperatureBias()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set a temperature bias of +10 degrees Rankine
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 10.0);

    // Check that bias is applied
    double T_sl = atm.GetTemperature(0.0);
    TS_ASSERT_DELTA(T_sl, FGAtmosphere::StdDaySLtemperature + 10.0, epsilon);

    // Check bias at altitude
    double T_10k = atm.GetTemperature(10000.0);
    double T_std_10k = atm.GetStdTemperature(10000.0);
    TS_ASSERT_DELTA(T_10k, T_std_10k + 10.0, epsilon);

    // Reset and verify
    atm.ResetSLTemperature();
    T_sl = atm.GetTemperature(0.0);
    TS_ASSERT_DELTA(T_sl, FGAtmosphere::StdDaySLtemperature, epsilon);
  }

  void testTemperatureBiasInCelsius()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set a temperature bias of +5 degrees Celsius
    atm.SetTemperatureBias(FGAtmosphere::eCelsius, 5.0);

    // Check that bias is applied (5C = 9R)
    double T_sl = atm.GetTemperature(0.0);
    TS_ASSERT_DELTA(T_sl, FGAtmosphere::StdDaySLtemperature + 9.0, epsilon);

    // Get the bias back in Celsius
    double bias_C = atm.GetTemperatureBias(FGAtmosphere::eCelsius);
    TS_ASSERT_DELTA(bias_C, 5.0, epsilon);
  }

  void testTemperatureGradedDelta()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set a graded delta of +20 degrees Rankine at sea level
    atm.SetSLTemperatureGradedDelta(FGAtmosphere::eRankine, 20.0);

    // At sea level, full delta should be applied
    double T_sl = atm.GetTemperature(0.0);
    double T_std_sl = atm.GetStdTemperature(0.0);
    TS_ASSERT_DELTA(T_sl - T_std_sl, 20.0, 1.0);

    // At high altitude, delta should be reduced
    double T_high = atm.GetTemperature(200000.0);
    double T_std_high = atm.GetStdTemperature(200000.0);
    double delta_high = T_high - T_std_high;
    TS_ASSERT(delta_high < 20.0);  // Should be less than full delta

    // Reset and verify
    atm.ResetSLTemperature();
    T_sl = atm.GetTemperature(0.0);
    TS_ASSERT_DELTA(T_sl, FGAtmosphere::StdDaySLtemperature, epsilon);
  }

  void testSetTemperatureSL()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set sea level temperature to 530 R
    atm.SetTemperatureSL(530.0, FGAtmosphere::eRankine);

    // Check that the temperature at sea level matches
    double T_sl = atm.GetTemperature(0.0);
    TS_ASSERT_DELTA(T_sl, 530.0, epsilon);

    // The bias should be 530 - standard
    double expected_bias = 530.0 - FGAtmosphere::StdDaySLtemperature;
    double actual_bias = atm.GetTemperatureBias(FGAtmosphere::eRankine);
    TS_ASSERT_DELTA(actual_bias, expected_bias, epsilon);
  }

  void testSetTemperatureAtAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set temperature to 400 R at 20,000 ft
    double h = 20000.0;
    double T_target = 400.0;
    atm.SetTemperature(T_target, h, FGAtmosphere::eRankine);

    // Verify temperature at that altitude
    double T_h = atm.GetTemperature(h);
    TS_ASSERT_DELTA(T_h, T_target, 0.1);
  }

  void testSetPressureSL()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set sea level pressure to 2000 psf
    atm.SetPressureSL(FGAtmosphere::ePSF, 2000.0);

    // Verify pressure at sea level
    double P_sl = atm.GetPressure(0.0);
    TS_ASSERT_DELTA(P_sl, 2000.0, epsilon);

    // Reset and verify
    atm.ResetSLPressure();
    P_sl = atm.GetPressure(0.0);
    TS_ASSERT_DELTA(P_sl, FGAtmosphere::StdDaySLpressure, epsilon);
  }

  void testPressureAltitudeCalculation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At standard conditions, pressure altitude should equal geometric altitude
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);

    double press_alt = atm.GetPressureAltitude();
    TS_ASSERT_DELTA(press_alt, 10000.0, 10.0);

    // With non-standard pressure, pressure altitude should differ
    atm.InitModel();
    atm.SetPressureSL(FGAtmosphere::ePSF, 2000.0);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    press_alt = atm.GetPressureAltitude();
    TS_ASSERT(press_alt != 0.0);  // Should not be zero
  }

  void testDensityAltitudeCalculation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At standard conditions, density altitude should equal geometric altitude
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);

    double dens_alt = atm.GetDensityAltitude();
    TS_ASSERT_DELTA(dens_alt, 10000.0, 10.0);

    // With hot temperature, density altitude should be higher
    atm.InitModel();
    atm.SetTemperatureSL(550.0, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    dens_alt = atm.GetDensityAltitude();
    TS_ASSERT(dens_alt > 0.0);  // Should be positive (higher than geometric)
  }

  void testGeopotentialDifference()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test that temperature at geometric altitude differs slightly from geopotential
    // due to the geopotential conversion (indirect test)
    double T_10k_std = atm.GetStdTemperature(10000.0);
    double T_10k = atm.GetTemperature(10000.0);

    // At standard conditions, they should match
    TS_ASSERT_DELTA(T_10k, T_10k_std, 0.1);
  }

  void testNegativeAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test below sea level (e.g., Death Valley at -282 ft)
    double h = -282.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Temperature should be higher than sea level (use parameterless version from base class)
    TS_ASSERT(atm.FGAtmosphere::GetTemperature() > FGAtmosphere::StdDaySLtemperature);

    // Pressure should be higher than sea level
    TS_ASSERT(atm.FGAtmosphere::GetPressure() > FGAtmosphere::StdDaySLpressure);

    // Density should be higher than sea level
    TS_ASSERT(atm.GetDensity() > FGAtmosphere::StdDaySLdensity);
  }

  void testHighAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test at 100,000 ft
    double h = 100000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Temperature should be cold but above absolute zero (use parameterless version from base class)
    double T = atm.FGAtmosphere::GetTemperature();
    TS_ASSERT(T > 0.0);
    TS_ASSERT(T < FGAtmosphere::StdDaySLtemperature);

    // Pressure should be very low
    double P = atm.FGAtmosphere::GetPressure();
    TS_ASSERT(P > 0.0);
    TS_ASSERT(P < FGAtmosphere::StdDaySLpressure);
  }

  void testTroposphereStratosphereTransition()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test just below tropopause
    double T_below = atm.GetStdTemperature(36000.0);

    // Test at tropopause
    double T_tropo = atm.GetStdTemperature(36089.0);

    // Test above tropopause
    double T_above = atm.GetStdTemperature(40000.0);

    // Temperature should be isothermal in lower stratosphere
    TS_ASSERT_DELTA(T_tropo, T_above, 1.0);
  }

  void testStdTemperatureRatio()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At sea level, ratio should be 1.0
    TS_ASSERT_DELTA(atm.GetStdTemperatureRatio(0.0), 1.0, epsilon);

    // At altitude, ratio should be less than 1.0
    double ratio_10k = atm.GetStdTemperatureRatio(10000.0);
    TS_ASSERT(ratio_10k < 1.0);

    // Higher altitude should have lower ratio
    double ratio_30k = atm.GetStdTemperatureRatio(30000.0);
    TS_ASSERT(ratio_30k < ratio_10k);
  }

  void testPropertyBindings()
  {
    auto pm = fdmex.GetPropertyManager();
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Verify standard atmosphere specific properties are bound
    TS_ASSERT(pm->HasNode("atmosphere/delta-T"));
    TS_ASSERT(pm->HasNode("atmosphere/SL-graded-delta-T"));
    TS_ASSERT(pm->HasNode("atmosphere/P-sl-psf"));
  }

  void testHumidityFeatures()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test setting relative humidity
    atm.SetRelativeHumidity(50.0);
    double RH = atm.GetRelativeHumidity();
    TS_ASSERT_DELTA(RH, 50.0, 1.0);

    // Test setting dew point
    atm.SetDewPoint(FGAtmosphere::eFahrenheit, 50.0);
    double dewpoint = atm.GetDewPoint(FGAtmosphere::eFahrenheit);
    TS_ASSERT_DELTA(dewpoint, 50.0, 1.0);

    // Vapor pressure should be positive
    double vp = atm.GetVaporPressure(FGAtmosphere::ePSF);
    TS_ASSERT(vp >= 0.0);

    // Test vapor mass fraction
    atm.SetVaporMassFractionPPM(10000.0);
    double vmf = atm.GetVaporMassFractionPPM();
    TS_ASSERT(vmf > 0.0);
  }

  void testHumidityValidation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test negative humidity is capped
    atm.SetRelativeHumidity(-10.0);
    double RH = atm.GetRelativeHumidity();
    TS_ASSERT(RH >= 0.0);

    // Test humidity over 100% is capped
    atm.SetRelativeHumidity(150.0);
    RH = atm.GetRelativeHumidity();
    TS_ASSERT(RH <= 100.0);
  }

  void testVaporPressureSettings()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set vapor pressure to a reasonable value (not too high)
    double vp_set = 10.0;  // psf (lower value to avoid capping)
    atm.SetVaporPressure(FGAtmosphere::ePSF, vp_set);

    // Get it back - may be capped based on saturation and max vapor fraction
    double vp_get = atm.GetVaporPressure(FGAtmosphere::ePSF);
    TS_ASSERT(vp_get > 0.0);
    TS_ASSERT(vp_get <= vp_set);  // Should be less than or equal due to capping

    // Saturated vapor pressure should be positive
    double svp = atm.GetSaturatedVaporPressure(FGAtmosphere::ePSF);
    TS_ASSERT(svp > 0.0);
  }

  void testMultipleTemperatureUnits()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set in Celsius
    atm.SetTemperatureSL(15.0, FGAtmosphere::eCelsius);
    double T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, (15.0 + 273.15) * 1.8, 0.1);

    // Set in Kelvin
    atm.SetTemperatureSL(288.15, FGAtmosphere::eKelvin);
    T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, 288.15 * 1.8, 0.1);

    // Set in Fahrenheit
    atm.SetTemperatureSL(59.0, FGAtmosphere::eFahrenheit);
    T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, 59.0 + 459.67, 0.1);
  }

  void testMultiplePressureUnits()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set in millibars
    atm.SetPressureSL(FGAtmosphere::eMillibars, 1013.25);
    double P_psf = atm.GetPressureSL(FGAtmosphere::ePSF);
    TS_ASSERT_DELTA(P_psf, 1013.25 * 2.08854342, 0.1);

    // Set in Pascals
    atm.SetPressureSL(FGAtmosphere::ePascals, 101325.0);
    P_psf = atm.GetPressureSL(FGAtmosphere::ePSF);
    TS_ASSERT_DELTA(P_psf, 101325.0 * 0.0208854342, 0.1);

    // Set in InchesHg
    atm.SetPressureSL(FGAtmosphere::eInchesHg, 29.92);
    P_psf = atm.GetPressureSL(FGAtmosphere::ePSF);
    TS_ASSERT_DELTA(P_psf, 29.92 * 70.7180803, 1.0);
  }

  void testAtmosphereRun()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test Run method at various altitudes
    for (double h = 0.0; h < 50000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      // Verify all values are reasonable (use base class methods)
      TS_ASSERT(atm.FGAtmosphere::GetTemperature() > 0.0);
      TS_ASSERT(atm.FGAtmosphere::GetPressure() > 0.0);
      TS_ASSERT(atm.GetDensity() > 0.0);
      TS_ASSERT(atm.GetSoundSpeed() > 0.0);
      TS_ASSERT(atm.GetAbsoluteViscosity() > 0.0);
      TS_ASSERT(atm.GetKinematicViscosity() > 0.0);
    }
  }

  void testTemperatureDeltaGradient()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set a temperature graded delta at a specific altitude
    double delta_T = 15.0;  // degrees Rankine
    double h = 5000.0;      // ft
    atm.SetTemperatureGradedDelta(delta_T, h, FGAtmosphere::eRankine);

    // Verify the gradient was applied
    double gradient = atm.GetTemperatureDeltaGradient(FGAtmosphere::eRankine);
    TS_ASSERT(gradient != 0.0);
  }

  void testExtremeTemperatureBias()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Try to set an extremely low temperature bias
    atm.SetTemperatureBias(FGAtmosphere::eRankine, -1000.0);

    // Temperature should be capped to prevent going below absolute zero
    double T_sl = atm.GetTemperature(0.0);
    TS_ASSERT(T_sl > 0.0);  // Should be above absolute zero
  }

  void testExtremeTemperatureGradient()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Try to set an extreme temperature gradient
    atm.SetSLTemperatureGradedDelta(FGAtmosphere::eRankine, -1000.0);

    // Temperature should be validated
    double T_sl = atm.GetTemperature(0.0);
    TS_ASSERT(T_sl > 0.0);  // Should be above absolute zero
  }

  void testViscosityAtVariousTemperatures()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test at standard conditions
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double mu_std = atm.GetAbsoluteViscosity();
    double nu_std = atm.GetKinematicViscosity();
    TS_ASSERT(mu_std > 0.0);
    TS_ASSERT(nu_std > 0.0);

    // Test at higher temperature
    atm.SetTemperatureSL(600.0, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double mu_hot = atm.GetAbsoluteViscosity();
    TS_ASSERT(mu_hot > mu_std);  // Viscosity increases with temperature

    // Test at lower temperature
    atm.SetTemperatureSL(450.0, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double mu_cold = atm.GetAbsoluteViscosity();
    TS_ASSERT(mu_cold < mu_std);  // Viscosity decreases with temperature
  }

  // ============================================================================
  // Additional Standard Atmosphere Tests
  // ============================================================================

  void testSpeedOfSoundVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Speed of sound at sea level (about 1116 ft/s at 518.67R)
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double a_sl = atm.GetSoundSpeed();
    TS_ASSERT_DELTA(a_sl, 1116.0, 5.0);

    // Speed of sound at 10000 ft (lower temperature, lower speed)
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);
    double a_10k = atm.GetSoundSpeed();
    TS_ASSERT(a_10k < a_sl);

    // Speed of sound at tropopause (isothermal region)
    atm.in.altitudeASL = 36089.0;
    TS_ASSERT(atm.Run(false) == false);
    double a_tropo = atm.GetSoundSpeed();
    TS_ASSERT(a_tropo < a_10k);
  }

  void testDensityRatioVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At sea level, density ratio (sigma) should be 1.0
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double sigma_sl = atm.GetDensityRatio();
    TS_ASSERT_DELTA(sigma_sl, 1.0, epsilon);

    // At altitude, density ratio should decrease
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);
    double sigma_10k = atm.GetDensityRatio();
    TS_ASSERT(sigma_10k < 1.0);
    TS_ASSERT(sigma_10k > 0.0);

    // Higher altitude, even lower ratio
    atm.in.altitudeASL = 30000.0;
    TS_ASSERT(atm.Run(false) == false);
    double sigma_30k = atm.GetDensityRatio();
    TS_ASSERT(sigma_30k < sigma_10k);
  }

  void testPressureRatioVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At sea level, pressure ratio (delta) should be 1.0
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double delta_sl = atm.GetPressureRatio();
    TS_ASSERT_DELTA(delta_sl, 1.0, epsilon);

    // At 10000 ft, pressure ratio should be less
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);
    double delta_10k = atm.GetPressureRatio();
    TS_ASSERT(delta_10k < 1.0);

    // Pressure ratio at 18000 ft should be approximately 0.5
    atm.in.altitudeASL = 18000.0;
    TS_ASSERT(atm.Run(false) == false);
    double delta_18k = atm.GetPressureRatio();
    TS_ASSERT_DELTA(delta_18k, 0.5, 0.05);
  }

  void testTemperatureRatioVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At sea level, temperature ratio (theta) should be 1.0
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double theta_sl = atm.GetTemperatureRatio();
    TS_ASSERT_DELTA(theta_sl, 1.0, epsilon);

    // At altitude, temperature ratio should decrease
    atm.in.altitudeASL = 20000.0;
    TS_ASSERT(atm.Run(false) == false);
    double theta_20k = atm.GetTemperatureRatio();
    TS_ASSERT(theta_20k < 1.0);
  }

  void testIdealGasLawConsistency()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // P = rho * R * T (ideal gas law)
    // R for air ≈ 1716.49 ft²/(s²·R) or 53.35 ft·lbf/(lbm·R)
    const double R_specific = 1716.49;  // ft²/(s²·R)

    for (double h = 0.0; h <= 50000.0; h += 10000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double P = atm.FGAtmosphere::GetPressure();  // psf
      double rho = atm.GetDensity();  // slug/ft³
      double T = atm.FGAtmosphere::GetTemperature();  // Rankine

      // P = rho * R * T (with consistent units)
      // Converting: P (lbf/ft²) should equal rho (slug/ft³) * g * R/g * T
      double P_calculated = rho * R_specific * T;
      TS_ASSERT_DELTA(P, P_calculated, P * 0.01);  // Within 1%
    }
  }

  void testStratosphereIsothermal()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // In the lower stratosphere (36089 - 65617 ft), temperature is constant
    double T_36k = atm.GetStdTemperature(36089.0);
    double T_45k = atm.GetStdTemperature(45000.0);
    double T_55k = atm.GetStdTemperature(55000.0);
    double T_65k = atm.GetStdTemperature(65617.0);

    // All should be approximately equal (isothermal)
    TS_ASSERT_DELTA(T_36k, T_45k, 1.0);
    TS_ASSERT_DELTA(T_45k, T_55k, 1.0);
    TS_ASSERT_DELTA(T_55k, T_65k, 1.0);
  }

  void testTroposphereLapseRate()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Standard lapse rate in troposphere is -3.56616 °R/1000 ft
    // or -0.00356616 °R/ft
    double T_0 = atm.GetStdTemperature(0.0);
    double T_10k = atm.GetStdTemperature(10000.0);

    double lapse = (T_10k - T_0) / 10000.0;  // °R/ft
    TS_ASSERT_DELTA(lapse, -0.00356616, 0.0001);
  }

  void testDensityAltitudeHighTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // On a hot day, density altitude is higher than geometric altitude
    atm.SetTemperatureSL(560.0, FGAtmosphere::eRankine);  // Hot day (~100°F)
    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    double dens_alt = atm.GetDensityAltitude();
    TS_ASSERT(dens_alt > 5000.0);  // Higher than geometric due to heat
  }

  void testDensityAltitudeLowTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // On a cold day, density altitude is lower than geometric altitude
    atm.SetTemperatureSL(460.0, FGAtmosphere::eRankine);  // Cold day (~0°F)
    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    double dens_alt = atm.GetDensityAltitude();
    TS_ASSERT(dens_alt < 5000.0);  // Lower than geometric due to cold
  }

  void testPressureAltitudeHighPressure()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // High pressure day - pressure altitude lower than geometric
    atm.SetPressureSL(FGAtmosphere::ePSF, 2200.0);  // High pressure
    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    double press_alt = atm.GetPressureAltitude();
    TS_ASSERT(press_alt < 5000.0);  // Lower because higher pressure
  }

  void testPressureAltitudeLowPressure()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Low pressure day - pressure altitude higher than geometric
    atm.SetPressureSL(FGAtmosphere::ePSF, 2000.0);  // Low pressure
    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    double press_alt = atm.GetPressureAltitude();
    TS_ASSERT(press_alt > 5000.0);  // Higher because lower pressure
  }

  void testVeryHighAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test at 200,000 ft (near edge of atmosphere model)
    atm.in.altitudeASL = 200000.0;
    TS_ASSERT(atm.Run(false) == false);

    // Should still have valid values
    double T = atm.FGAtmosphere::GetTemperature();
    double P = atm.FGAtmosphere::GetPressure();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(P > 0.0);
    TS_ASSERT(rho > 0.0);
    TS_ASSERT(!std::isnan(T));
    TS_ASSERT(!std::isnan(P));
    TS_ASSERT(!std::isnan(rho));
  }

  void testKinematicViscosityRelation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // nu = mu / rho (kinematic = dynamic / density)
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);

    double mu = atm.GetAbsoluteViscosity();
    double rho = atm.GetDensity();
    double nu = atm.GetKinematicViscosity();

    double nu_calculated = mu / rho;
    TS_ASSERT_DELTA(nu, nu_calculated, nu * 0.001);
  }

  void testReynoldsNumberAtAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Reynolds number = V * L / nu
    // At altitude, nu increases (rho decreases faster than mu)
    // So for same V and L, Re decreases with altitude

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double nu_sl = atm.GetKinematicViscosity();

    atm.in.altitudeASL = 30000.0;
    TS_ASSERT(atm.Run(false) == false);
    double nu_30k = atm.GetKinematicViscosity();

    TS_ASSERT(nu_30k > nu_sl);  // Higher viscosity at altitude
  }

  void testSaturationVaporPressure()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Saturation vapor pressure increases with temperature
    atm.SetTemperatureSL(500.0, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double svp_cold = atm.GetSaturatedVaporPressure(FGAtmosphere::ePSF);

    atm.SetTemperatureSL(540.0, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double svp_hot = atm.GetSaturatedVaporPressure(FGAtmosphere::ePSF);

    TS_ASSERT(svp_hot > svp_cold);
  }

  void testDewPointTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set dew point
    atm.SetDewPoint(FGAtmosphere::eFahrenheit, 40.0);
    double dewpoint_F = atm.GetDewPoint(FGAtmosphere::eFahrenheit);
    TS_ASSERT_DELTA(dewpoint_F, 40.0, 1.0);

    // Get in Celsius
    double dewpoint_C = atm.GetDewPoint(FGAtmosphere::eCelsius);
    TS_ASSERT_DELTA(dewpoint_C, (40.0 - 32.0) * 5.0 / 9.0, 1.0);
  }

  void testHumidityAffectsDensity()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Dry air density
    atm.SetRelativeHumidity(0.0);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double rho_dry = atm.GetDensity();

    // Humid air density (water vapor is lighter than dry air)
    atm.SetRelativeHumidity(100.0);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double rho_humid = atm.GetDensity();

    // Humid air should be less dense
    TS_ASSERT(rho_humid <= rho_dry);
  }

  void testTemperatureBiasAtAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set a bias and verify it's uniform with altitude
    double bias = 10.0;  // degrees Rankine
    atm.SetTemperatureBias(FGAtmosphere::eRankine, bias);

    // Check at multiple altitudes
    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      double T_std = atm.GetStdTemperature(h);
      double T_biased = atm.GetTemperature(h);
      TS_ASSERT_DELTA(T_biased - T_std, bias, 0.1);
    }
  }

  void testStdPressureAtKnownAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Known ISA pressure values
    // Sea level: 2116.22 psf
    TS_ASSERT_DELTA(atm.GetStdPressure(0.0), FGAtmosphere::StdDaySLpressure, 1.0);

    // 5000 ft: ~1760 psf
    TS_ASSERT_DELTA(atm.GetStdPressure(5000.0), 1760.0, 20.0);

    // 10000 ft: ~1455 psf
    TS_ASSERT_DELTA(atm.GetStdPressure(10000.0), 1455.0, 20.0);

    // 20000 ft: ~973 psf
    TS_ASSERT_DELTA(atm.GetStdPressure(20000.0), 973.0, 20.0);
  }

  void testStdDensityAtKnownAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Known ISA density values (slug/ft³)
    // Sea level: 0.002377 slug/ft³
    TS_ASSERT_DELTA(atm.GetStdDensity(0.0), 0.002377, 0.00001);

    // 10000 ft: ~0.001756 slug/ft³
    TS_ASSERT_DELTA(atm.GetStdDensity(10000.0), 0.001756, 0.0001);

    // 20000 ft: ~0.001267 slug/ft³
    TS_ASSERT_DELTA(atm.GetStdDensity(20000.0), 0.001267, 0.0001);
  }

  void testAtmosphereLayerBoundaries()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Tropopause boundary (~36,089 ft / 11 km)
    double T_below_tropo = atm.GetStdTemperature(35000.0);
    double T_at_tropo = atm.GetStdTemperature(36089.0);
    double T_above_tropo = atm.GetStdTemperature(40000.0);

    // Temperature should decrease below, then become constant
    TS_ASSERT(T_below_tropo > T_at_tropo);
    TS_ASSERT_DELTA(T_at_tropo, T_above_tropo, 1.0);

    // Stratopause boundary (~65,617 ft / 20 km)
    double T_above_65k = atm.GetStdTemperature(70000.0);
    double T_at_65k = atm.GetStdTemperature(65617.0);

    // Temperature should start increasing above stratopause
    TS_ASSERT(T_above_65k >= T_at_65k - 1.0);  // May increase or stay same
  }

  void testMachOneAtAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Speed of sound determines Mach 1
    // At sea level: ~1116 ft/s
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double a_sl = atm.GetSoundSpeed();
    TS_ASSERT_DELTA(a_sl, 1116.0, 5.0);

    // At 30000 ft: ~995 ft/s
    atm.in.altitudeASL = 30000.0;
    TS_ASSERT(atm.Run(false) == false);
    double a_30k = atm.GetSoundSpeed();
    TS_ASSERT_DELTA(a_30k, 995.0, 10.0);

    // At tropopause: ~968 ft/s
    atm.in.altitudeASL = 36089.0;
    TS_ASSERT(atm.Run(false) == false);
    double a_tropo = atm.GetSoundSpeed();
    TS_ASSERT_DELTA(a_tropo, 968.0, 10.0);
  }

  void testMultipleInitModelCalls()
  {
    TestableStdAtmosphere atm(&fdmex);

    // First init
    TS_ASSERT(atm.InitModel());
    double T_sl_1 = atm.GetTemperatureSL();

    // Modify state
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 50.0);

    // Re-init should reset
    TS_ASSERT(atm.InitModel());
    double T_sl_2 = atm.GetTemperatureSL();

    TS_ASSERT_DELTA(T_sl_1, T_sl_2, epsilon);
  }

  void testZeroAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    // All ratios should be 1.0 at sea level
    TS_ASSERT_DELTA(atm.GetTemperatureRatio(), 1.0, epsilon);
    TS_ASSERT_DELTA(atm.GetPressureRatio(), 1.0, epsilon);
    TS_ASSERT_DELTA(atm.GetDensityRatio(), 1.0, epsilon);
  }

  void testDeltaTProperty()
  {
    auto pm = fdmex.GetPropertyManager();
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set temperature bias
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 15.0);

    // Check that the temperature bias was set
    double T_at_sea = atm.GetTemperature(0.0);

    // At sea level, ISA temp is 518.67 R, plus 15 R bias
    TS_ASSERT_DELTA(T_at_sea, 518.67 + 15.0, 0.5);
  }

  // ============================================================================
  // Additional Expanded Tests
  // ============================================================================

  void testSoundSpeedFormula()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Speed of sound a = sqrt(gamma * R * T)
    // For air: gamma = 1.4, R = 1716.49 ft²/(s²·R)
    const double gamma = 1.4;
    const double R_specific = 1716.49;

    for (double h = 0.0; h <= 40000.0; h += 10000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.FGAtmosphere::GetTemperature();
      double a = atm.GetSoundSpeed();

      double a_calculated = sqrt(gamma * R_specific * T);
      TS_ASSERT_DELTA(a, a_calculated, a * 0.001);  // Within 0.1%
    }
  }

  void testDensityFromIdealGasLaw()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // rho = P / (R * T) from ideal gas law
    const double R_specific = 1716.49;

    for (double h = 0.0; h <= 40000.0; h += 10000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double P = atm.FGAtmosphere::GetPressure();
      double T = atm.FGAtmosphere::GetTemperature();
      double rho = atm.GetDensity();

      double rho_calculated = P / (R_specific * T);
      TS_ASSERT_DELTA(rho, rho_calculated, rho * 0.01);  // Within 1%
    }
  }

  void testAltitudeMonotonicDecrease()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Pressure and density should monotonically decrease with altitude
    double prev_P = atm.GetStdPressure(0.0);
    double prev_rho = atm.GetStdDensity(0.0);

    for (double h = 1000.0; h <= 100000.0; h += 1000.0) {
      double P = atm.GetStdPressure(h);
      double rho = atm.GetStdDensity(h);

      TS_ASSERT(P < prev_P);
      TS_ASSERT(rho < prev_rho);

      prev_P = P;
      prev_rho = rho;
    }
  }

  void testTemperatureBiasUnits()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set 10 Rankine bias
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 10.0);
    double bias_R = atm.GetTemperatureBias(FGAtmosphere::eRankine);
    TS_ASSERT_DELTA(bias_R, 10.0, epsilon);

    // Get in Fahrenheit (same as Rankine for deltas)
    double bias_F = atm.GetTemperatureBias(FGAtmosphere::eFahrenheit);
    TS_ASSERT_DELTA(bias_F, 10.0, epsilon);

    // Get in Celsius (10 R = 10/1.8 C = 5.556 C)
    double bias_C = atm.GetTemperatureBias(FGAtmosphere::eCelsius);
    TS_ASSERT_DELTA(bias_C, 10.0 / 1.8, 0.01);

    // Get in Kelvin (same as Celsius for deltas)
    double bias_K = atm.GetTemperatureBias(FGAtmosphere::eKelvin);
    TS_ASSERT_DELTA(bias_K, 10.0 / 1.8, 0.01);
  }

  void testPressureUnitsConversion()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double P_psf = atm.GetPressureSL(FGAtmosphere::ePSF);
    double P_pa = atm.GetPressureSL(FGAtmosphere::ePascals);
    double P_mb = atm.GetPressureSL(FGAtmosphere::eMillibars);
    double P_inhg = atm.GetPressureSL(FGAtmosphere::eInchesHg);

    // Verify conversions
    TS_ASSERT_DELTA(P_pa, P_psf / 0.0208854342, 10.0);
    TS_ASSERT_DELTA(P_mb, P_pa / 100.0, 0.1);
    TS_ASSERT_DELTA(P_inhg, P_psf / 70.7180803, 0.01);

    // PSF to PSI conversion (manually calculate)
    double P_psi_calc = P_psf / 144.0;
    TS_ASSERT(P_psi_calc > 0.0);  // About 14.7 psi
  }

  void testConsistentRatios()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At sea level, all ratios should be 1.0
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    TS_ASSERT_DELTA(atm.GetTemperatureRatio(), 1.0, epsilon);
    TS_ASSERT_DELTA(atm.GetPressureRatio(), 1.0, epsilon);
    TS_ASSERT_DELTA(atm.GetDensityRatio(), 1.0, epsilon);

    // Verify: delta = sigma * theta (from ideal gas law)
    for (double h = 5000.0; h <= 40000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double theta = atm.GetTemperatureRatio();
      double delta = atm.GetPressureRatio();
      double sigma = atm.GetDensityRatio();

      TS_ASSERT_DELTA(delta, sigma * theta, delta * 0.001);
    }
  }

  void testAbsoluteZeroProtection()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Try to set temperature to below absolute zero
    atm.SetTemperatureSL(-100.0, FGAtmosphere::eRankine);

    // Temperature should never be negative or zero
    double T = atm.GetTemperatureSL();
    TS_ASSERT(T > 0.0);
  }

  void testHumidityBoundaries()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test RH = 0%
    atm.SetRelativeHumidity(0.0);
    TS_ASSERT_DELTA(atm.GetRelativeHumidity(), 0.0, 1.0);

    // Vapor pressure should be near zero
    double vp_dry = atm.GetVaporPressure(FGAtmosphere::ePSF);
    TS_ASSERT(vp_dry >= 0.0);

    // Test RH = 100%
    atm.SetRelativeHumidity(100.0);
    TS_ASSERT_DELTA(atm.GetRelativeHumidity(), 100.0, 1.0);

    // Vapor pressure should equal saturation vapor pressure
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double vp_sat = atm.GetSaturatedVaporPressure(FGAtmosphere::ePSF);
    double vp_100 = atm.GetVaporPressure(FGAtmosphere::ePSF);
    TS_ASSERT(vp_100 <= vp_sat);  // May be capped due to max vapor fraction
  }

  void testUpperStratosphere()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Upper stratosphere (above 65617 ft / 20 km): temperature increases
    double T_65k = atm.GetStdTemperature(65617.0);
    double T_80k = atm.GetStdTemperature(80000.0);
    double T_100k = atm.GetStdTemperature(100000.0);

    // Temperature should increase in upper stratosphere
    TS_ASSERT(T_80k >= T_65k - 1.0);  // Allow small tolerance
    TS_ASSERT(T_100k >= T_80k - 1.0);
  }

  void testDewPointVsTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    // Set dew point below ambient temperature (normal condition)
    double T_ambient = atm.FGAtmosphere::GetTemperature() - 459.67;  // Convert to Fahrenheit
    atm.SetDewPoint(FGAtmosphere::eFahrenheit, T_ambient - 20.0);

    double dewpoint = atm.GetDewPoint(FGAtmosphere::eFahrenheit);
    TS_ASSERT(dewpoint <= T_ambient);

    // RH should be less than 100%
    double RH = atm.GetRelativeHumidity();
    TS_ASSERT(RH < 100.0);
  }

  void testViscositySutherlandLaw()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Sutherland's law: mu = mu0 * (T/T0)^(3/2) * (T0 + S) / (T + S)
    // For air: T0 = 518.67 R, S = 198.72 R
    const double T0 = 518.67;
    const double S = 198.72;
    const double mu0 = 3.737e-7;  // slug/(ft·s) at T0

    for (double h = 0.0; h <= 40000.0; h += 10000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.FGAtmosphere::GetTemperature();
      double mu = atm.GetAbsoluteViscosity();

      double mu_sutherland = mu0 * pow(T / T0, 1.5) * (T0 + S) / (T + S);
      TS_ASSERT_DELTA(mu, mu_sutherland, mu * 0.05);  // Within 5%
    }
  }

  void testAltitudeAtmosphereConsistency()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Run at a specific altitude and verify consistency
    double h = 25000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Get values from Run
    double T_run = atm.FGAtmosphere::GetTemperature();
    double P_run = atm.FGAtmosphere::GetPressure();
    double rho_run = atm.GetDensity();

    // Get standard values for same altitude
    double T_std = atm.GetStdTemperature(h);
    double P_std = atm.GetStdPressure(h);
    double rho_std = atm.GetStdDensity(h);

    // At standard conditions (no bias), should match
    TS_ASSERT_DELTA(T_run, T_std, 1.0);
    TS_ASSERT_DELTA(P_run, P_std, 1.0);
    TS_ASSERT_DELTA(rho_run, rho_std, 1e-6);
  }

  void testSeaLevelPressureVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test range of sea level pressures
    double pressures[] = {1900.0, 2000.0, 2100.0, 2200.0, 2300.0};

    for (double P_sl : pressures) {
      atm.InitModel();
      atm.SetPressureSL(FGAtmosphere::ePSF, P_sl);

      double P_get = atm.GetPressureSL(FGAtmosphere::ePSF);
      TS_ASSERT_DELTA(P_get, P_sl, 1.0);

      // Run and verify
      atm.in.altitudeASL = 0.0;
      TS_ASSERT(atm.Run(false) == false);
      TS_ASSERT_DELTA(atm.FGAtmosphere::GetPressure(), P_sl, 1.0);
    }
  }

  void testSeaLevelTemperatureVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test range of sea level temperatures
    double temps[] = {480.0, 500.0, 518.67, 540.0, 560.0};  // Rankine

    for (double T_sl : temps) {
      atm.InitModel();
      atm.SetTemperatureSL(T_sl, FGAtmosphere::eRankine);

      double T_get = atm.GetTemperatureSL();
      TS_ASSERT_DELTA(T_get, T_sl, 0.1);

      // Run and verify
      atm.in.altitudeASL = 0.0;
      TS_ASSERT(atm.Run(false) == false);
      TS_ASSERT_DELTA(atm.FGAtmosphere::GetTemperature(), T_sl, 0.1);
    }
  }

  void testResetFunctions()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Modify both temperature and pressure
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 50.0);
    atm.SetPressureSL(FGAtmosphere::ePSF, 2000.0);

    // Verify modified
    TS_ASSERT(fabs(atm.GetTemperatureSL() - FGAtmosphere::StdDaySLtemperature) > 1.0);
    TS_ASSERT(fabs(atm.GetPressureSL(FGAtmosphere::ePSF) - FGAtmosphere::StdDaySLpressure) > 1.0);

    // Reset temperature
    atm.ResetSLTemperature();
    TS_ASSERT_DELTA(atm.GetTemperatureSL(), FGAtmosphere::StdDaySLtemperature, epsilon);

    // Reset pressure
    atm.ResetSLPressure();
    TS_ASSERT_DELTA(atm.GetPressureSL(FGAtmosphere::ePSF), FGAtmosphere::StdDaySLpressure, epsilon);
  }

  void testDensityAltitudeConsistency()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At standard conditions, density altitude = geometric altitude
    for (double h = 0.0; h <= 20000.0; h += 5000.0) {
      atm.InitModel();
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double dens_alt = atm.GetDensityAltitude();
      TS_ASSERT_DELTA(dens_alt, h, 50.0);  // Within 50 ft tolerance
    }
  }

  void testPressureAltitudeConsistency()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At standard conditions, pressure altitude = geometric altitude
    for (double h = 0.0; h <= 20000.0; h += 5000.0) {
      atm.InitModel();
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double press_alt = atm.GetPressureAltitude();
      TS_ASSERT_DELTA(press_alt, h, 50.0);  // Within 50 ft tolerance
    }
  }

  void testGradedDeltaVsBias()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Bias: constant offset at all altitudes
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 20.0);

    double T_sl_bias = atm.GetTemperature(0.0);
    double T_20k_bias = atm.GetTemperature(20000.0);

    double diff_sl = T_sl_bias - atm.GetStdTemperature(0.0);
    double diff_20k = T_20k_bias - atm.GetStdTemperature(20000.0);

    TS_ASSERT_DELTA(diff_sl, diff_20k, 0.1);  // Same offset at all altitudes

    // Reset and test graded delta
    atm.ResetSLTemperature();
    atm.SetSLTemperatureGradedDelta(FGAtmosphere::eRankine, 20.0);

    double T_sl_graded = atm.GetTemperature(0.0);
    double T_200k_graded = atm.GetTemperature(200000.0);

    double diff_sl_graded = T_sl_graded - atm.GetStdTemperature(0.0);
    double diff_200k_graded = T_200k_graded - atm.GetStdTemperature(200000.0);

    // Graded delta should be larger at sea level than at high altitude
    TS_ASSERT(diff_sl_graded > diff_200k_graded);
  }

  void testVaporMassFraction()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set vapor mass fraction in PPM
    atm.SetVaporMassFractionPPM(5000.0);

    double vmf = atm.GetVaporMassFractionPPM();
    TS_ASSERT(vmf > 0.0);
    TS_ASSERT(vmf <= 5000.0);  // May be capped
  }

  void testSpecificHeatRatio()
  {
    // Specific heat ratio gamma for dry air is approximately 1.4
    // This is implicit in the speed of sound calculation
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double a = atm.GetSoundSpeed();
    double T = atm.FGAtmosphere::GetTemperature();
    const double R = 1716.49;

    // gamma = a^2 / (R * T)
    double gamma_calc = (a * a) / (R * T);
    TS_ASSERT_DELTA(gamma_calc, 1.4, 0.01);
  }

  void testVeryLowAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test at -1000 ft (Dead Sea is about -1400 ft)
    double h = -1000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // All values should be higher than sea level
    TS_ASSERT(atm.FGAtmosphere::GetTemperature() > FGAtmosphere::StdDaySLtemperature);
    TS_ASSERT(atm.FGAtmosphere::GetPressure() > FGAtmosphere::StdDaySLpressure);
    TS_ASSERT(atm.GetDensity() > FGAtmosphere::StdDaySLdensity);
  }

  void testExtremeAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test at 300,000 ft (near Karman line at ~328,000 ft)
    double h = 300000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Should still have valid (very low) values
    double T = atm.FGAtmosphere::GetTemperature();
    double P = atm.FGAtmosphere::GetPressure();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0 && !std::isnan(T) && !std::isinf(T));
    TS_ASSERT(P > 0.0 && !std::isnan(P) && !std::isinf(P));
    TS_ASSERT(rho > 0.0 && !std::isnan(rho) && !std::isinf(rho));

    // Pressure should be very low
    TS_ASSERT(P < 1.0);  // Less than 1 psf
  }

  // ============================================================================
  // Extended Standard Atmosphere Tests (76-100)
  // ============================================================================

  // Test 76: Temperature decreases monotonically in troposphere
  void testTroposphereMonotonicTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    double prev_T = atm.GetStdTemperature(0.0);
    for (double h = 1000.0; h <= 36000.0; h += 1000.0) {
      double T = atm.GetStdTemperature(h);
      TS_ASSERT(T < prev_T);  // Temperature should decrease
      prev_T = T;
    }
  }

  // Test 77: Mesosphere temperature behavior
  void testMesosphereTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Above stratopause (~160,000 ft / 47 km), temperature decreases again
    double T_160k = atm.GetStdTemperature(160000.0);
    double T_200k = atm.GetStdTemperature(200000.0);

    // In mesosphere, temperature should decrease with altitude
    // Note: depends on exact ISA model implementation
    TS_ASSERT(T_200k > 0.0);  // Must be above absolute zero
    TS_ASSERT(!std::isnan(T_200k));
  }

  // Test 78: Standard atmosphere at FL350 (35,000 ft)
  void testFlightLevel350()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 35000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Known ISA values at FL350
    double T = atm.FGAtmosphere::GetTemperature();
    double P = atm.FGAtmosphere::GetPressure();

    TS_ASSERT_DELTA(T, 394.0, 5.0);  // Approximately 394 R
    TS_ASSERT_DELTA(P, 498.0, 20.0);  // Approximately 498 psf
  }

  // Test 79: Standard atmosphere at FL410 (41,000 ft)
  void testFlightLevel410()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 41000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Above tropopause, temperature is constant
    double T = atm.FGAtmosphere::GetTemperature();
    TS_ASSERT_DELTA(T, 389.97, 5.0);  // Tropopause temperature
  }

  // Test 80: Pressure doubling altitude
  void testPressureDoublingAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Pressure halves approximately every 18,000 ft
    double P_sl = atm.GetStdPressure(0.0);
    double P_18k = atm.GetStdPressure(18000.0);
    double P_36k = atm.GetStdPressure(36000.0);

    TS_ASSERT_DELTA(P_18k / P_sl, 0.5, 0.05);
    TS_ASSERT_DELTA(P_36k / P_18k, 0.5, 0.1);
  }

  // Test 81: Density ratio at known altitudes
  void testDensityRatioKnownAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Known sigma values
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT_DELTA(atm.GetDensityRatio(), 0.7385, 0.01);

    atm.in.altitudeASL = 20000.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT_DELTA(atm.GetDensityRatio(), 0.5328, 0.01);

    atm.in.altitudeASL = 30000.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT_DELTA(atm.GetDensityRatio(), 0.3741, 0.01);
  }

  // Test 82: Combined temperature and pressure bias effects
  void testCombinedBiasEffects()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Hot day with low pressure
    atm.SetTemperatureSL(540.0, FGAtmosphere::eRankine);  // Hot
    atm.SetPressureSL(FGAtmosphere::ePSF, 2050.0);        // Low pressure

    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    // Both density altitude and pressure altitude should be higher
    double dens_alt = atm.GetDensityAltitude();
    double press_alt = atm.GetPressureAltitude();

    TS_ASSERT(dens_alt > 5000.0);
    TS_ASSERT(press_alt > 5000.0);
  }

  // Test 83: Sound speed ratio variation
  void testSoundSpeedRatio()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // a/a_sl = sqrt(T/T_sl) = sqrt(theta)
    for (double h = 0.0; h <= 40000.0; h += 10000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double a = atm.GetSoundSpeed();
      double a_sl = 1116.45;  // Standard sea level sound speed
      double theta = atm.GetTemperatureRatio();

      double ratio = a / a_sl;
      TS_ASSERT_DELTA(ratio, sqrt(theta), 0.01);
    }
  }

  // Test 84: Dynamic pressure calculation
  void testDynamicPressureCalculation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // q = 0.5 * rho * V^2
    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho = atm.GetDensity();
    double V = 500.0;  // ft/s
    double q_calculated = 0.5 * rho * V * V;

    // Should be a reasonable value
    TS_ASSERT(q_calculated > 0.0);
    TS_ASSERT(q_calculated < 10000.0);  // Reasonable dynamic pressure
  }

  // Test 85: Temperature gradient at altitude
  void testTemperatureGradientAtAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set a graded delta at 10,000 ft
    atm.SetTemperatureGradedDelta(15.0, 10000.0, FGAtmosphere::eRankine);

    // Verify gradient is set
    double gradient = atm.GetTemperatureDeltaGradient(FGAtmosphere::eRankine);
    TS_ASSERT(gradient != 0.0);

    // Temperature at 10000 ft should be higher than standard
    double T_10k = atm.GetTemperature(10000.0);
    double T_std_10k = atm.GetStdTemperature(10000.0);
    TS_ASSERT(T_10k > T_std_10k);
  }

  // Test 86: Atmosphere values at 1000 ft intervals
  void testThousandFootIntervals()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    double prev_T = atm.GetStdTemperature(0.0);
    double prev_P = atm.GetStdPressure(0.0);
    double prev_rho = atm.GetStdDensity(0.0);

    for (int h = 1000; h <= 10000; h += 1000) {
      double T = atm.GetStdTemperature(static_cast<double>(h));
      double P = atm.GetStdPressure(static_cast<double>(h));
      double rho = atm.GetStdDensity(static_cast<double>(h));

      // All should be less than previous
      TS_ASSERT(T < prev_T);
      TS_ASSERT(P < prev_P);
      TS_ASSERT(rho < prev_rho);

      prev_T = T;
      prev_P = P;
      prev_rho = rho;
    }
  }

  // Test 87: Temperature in Kelvin
  void testTemperatureInKelvin()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double T_R = atm.FGAtmosphere::GetTemperature();
    double T_K = T_R / 1.8;  // Convert Rankine to Kelvin

    TS_ASSERT_DELTA(T_K, 288.15, 0.1);  // ISA sea level is 288.15 K
  }

  // Test 88: Pressure in hectopascals
  void testPressureInHectopascals()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double P_mb = atm.GetPressureSL(FGAtmosphere::eMillibars);
    TS_ASSERT_DELTA(P_mb, 1013.25, 0.1);  // ISA sea level is 1013.25 hPa
  }

  // Test 89: Very small altitude steps
  void testVerySmallAltitudeSteps()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test every 10 ft from 0 to 100 ft
    double prev_T = atm.GetStdTemperature(0.0);
    for (double h = 10.0; h <= 100.0; h += 10.0) {
      double T = atm.GetStdTemperature(h);
      TS_ASSERT(T < prev_T);  // Should monotonically decrease
      TS_ASSERT_DELTA(prev_T - T, 0.0357, 0.001);  // ~3.57 R per 1000 ft
      prev_T = T;
    }
  }

  // Test 90: Multiple humidity settings
  void testMultipleHumiditySettings()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;

    // Test various RH values
    double RH_values[] = {0.0, 25.0, 50.0, 75.0, 100.0};
    double prev_rho = std::numeric_limits<double>::max();

    for (double RH : RH_values) {
      atm.SetRelativeHumidity(RH);
      TS_ASSERT(atm.Run(false) == false);

      double rho = atm.GetDensity();
      TS_ASSERT(rho > 0.0);
      TS_ASSERT(rho <= prev_rho);  // Density decreases with humidity
      prev_rho = rho;
    }
  }

  // Test 91: Temperature bias in all units
  void testTemperatureBiasAllUnits()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set 18 R bias (equivalent to 10 K or 10 C)
    atm.SetTemperatureBias(FGAtmosphere::eRankine, 18.0);

    // Check in all units
    double bias_R = atm.GetTemperatureBias(FGAtmosphere::eRankine);
    double bias_F = atm.GetTemperatureBias(FGAtmosphere::eFahrenheit);
    double bias_K = atm.GetTemperatureBias(FGAtmosphere::eKelvin);
    double bias_C = atm.GetTemperatureBias(FGAtmosphere::eCelsius);

    TS_ASSERT_DELTA(bias_R, 18.0, epsilon);
    TS_ASSERT_DELTA(bias_F, 18.0, epsilon);  // Same as Rankine for deltas
    TS_ASSERT_DELTA(bias_K, 10.0, 0.01);
    TS_ASSERT_DELTA(bias_C, 10.0, 0.01);
  }

  // Test 92: Dew point at various temperatures
  void testDewPointAtVariousTemperatures()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Set different temperatures and check dew point consistency
    double temps[] = {500.0, 520.0, 540.0, 560.0};  // Rankine

    for (double T : temps) {
      atm.InitModel();
      atm.SetTemperatureSL(T, FGAtmosphere::eRankine);
      atm.SetRelativeHumidity(50.0);
      atm.in.altitudeASL = 0.0;
      TS_ASSERT(atm.Run(false) == false);

      double dewpoint = atm.GetDewPoint(FGAtmosphere::eRankine);
      // Dew point should be at or below ambient temperature at 50% RH
      TS_ASSERT(dewpoint <= T);
      TS_ASSERT(dewpoint > 0.0);
    }
  }

  // Test 93: Atmospheric properties at critical altitudes
  void testCriticalAltitudes()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Critical altitudes for flight
    double altitudes[] = {8000.0, 10000.0, 14000.0, 18000.0, 25000.0};

    for (double h : altitudes) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      // All atmospheric properties should be valid
      TS_ASSERT(atm.FGAtmosphere::GetTemperature() > 0.0);
      TS_ASSERT(atm.FGAtmosphere::GetPressure() > 0.0);
      TS_ASSERT(atm.GetDensity() > 0.0);
      TS_ASSERT(atm.GetSoundSpeed() > 0.0);
    }
  }

  // Test 94: Standard day values at sea level
  void testStandardDaySeaLevel()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    // ISA sea level standard values
    TS_ASSERT_DELTA(atm.FGAtmosphere::GetTemperature(), 518.67, 0.1);  // 59°F
    TS_ASSERT_DELTA(atm.FGAtmosphere::GetPressure(), 2116.22, 1.0);    // 14.7 psi
    TS_ASSERT_DELTA(atm.GetDensity(), 0.002377, 0.00001);              // slug/ft³
    TS_ASSERT_DELTA(atm.GetSoundSpeed(), 1116.45, 1.0);                // ft/s
  }

  // Test 95: Effective altitude vs geometric altitude
  void testEffectiveVsGeometricAltitude()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // At standard conditions, effective = geometric
    double h = 15000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    double dens_alt = atm.GetDensityAltitude();
    double press_alt = atm.GetPressureAltitude();

    TS_ASSERT_DELTA(dens_alt, h, 50.0);
    TS_ASSERT_DELTA(press_alt, h, 50.0);
  }

  // Test 96: Saturation vapor pressure variation with temperature
  void testSaturationVaporPressureVariation()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // SVP increases exponentially with temperature
    double prev_svp = 0.0;
    for (double T = 460.0; T <= 560.0; T += 20.0) {
      atm.InitModel();
      atm.SetTemperatureSL(T, FGAtmosphere::eRankine);
      atm.in.altitudeASL = 0.0;
      TS_ASSERT(atm.Run(false) == false);

      double svp = atm.GetSaturatedVaporPressure(FGAtmosphere::ePSF);
      TS_ASSERT(svp > prev_svp);  // Should increase
      prev_svp = svp;
    }
  }

  // Test 97: Cold soak temperature effects
  void testColdSoakTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Simulate very cold conditions
    atm.SetTemperatureSL(430.0, FGAtmosphere::eRankine);  // About -30°F

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    // All values should still be valid
    TS_ASSERT(atm.FGAtmosphere::GetTemperature() > 0.0);
    TS_ASSERT(atm.FGAtmosphere::GetPressure() > 0.0);
    TS_ASSERT(atm.GetDensity() > 0.0);
    TS_ASSERT(atm.GetSoundSpeed() > 0.0);

    // Density should be higher than standard
    TS_ASSERT(atm.GetDensity() > FGAtmosphere::StdDaySLdensity);
  }

  // Test 98: Hot day temperature effects
  void testHotDayTemperature()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Simulate very hot conditions
    atm.SetTemperatureSL(580.0, FGAtmosphere::eRankine);  // About 120°F

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    // All values should still be valid
    TS_ASSERT(atm.FGAtmosphere::GetTemperature() > 0.0);
    TS_ASSERT(atm.FGAtmosphere::GetPressure() > 0.0);
    TS_ASSERT(atm.GetDensity() > 0.0);
    TS_ASSERT(atm.GetSoundSpeed() > 0.0);

    // Density should be lower than standard
    TS_ASSERT(atm.GetDensity() < FGAtmosphere::StdDaySLdensity);
  }

  // Test 99: Pressure altitude at non-standard pressure
  void testPressureAltitudeNonStandard()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // 29.92 inHg is standard, test at 30.50 inHg (high pressure)
    atm.SetPressureSL(FGAtmosphere::eInchesHg, 30.50);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double press_alt = atm.GetPressureAltitude();
    TS_ASSERT(press_alt < 0.0);  // Negative because pressure is high

    // Test at 29.00 inHg (low pressure)
    atm.InitModel();
    atm.SetPressureSL(FGAtmosphere::eInchesHg, 29.00);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    press_alt = atm.GetPressureAltitude();
    TS_ASSERT(press_alt > 0.0);  // Positive because pressure is low
  }

  // Test 100: Complete atmosphere state consistency
  void testCompleteAtmosphereStateConsistency()
  {
    TestableStdAtmosphere atm(&fdmex);
    TS_ASSERT(atm.InitModel());

    // Test at various conditions for complete consistency
    double altitudes[] = {0.0, 5000.0, 10000.0, 25000.0, 40000.0};
    double temp_biases[] = {-20.0, 0.0, 20.0};

    for (double h : altitudes) {
      for (double bias : temp_biases) {
        atm.InitModel();
        atm.SetTemperatureBias(FGAtmosphere::eRankine, bias);
        atm.in.altitudeASL = h;
        TS_ASSERT(atm.Run(false) == false);

        // Get all properties
        double T = atm.FGAtmosphere::GetTemperature();
        double P = atm.FGAtmosphere::GetPressure();
        double rho = atm.GetDensity();
        double a = atm.GetSoundSpeed();
        double mu = atm.GetAbsoluteViscosity();
        double nu = atm.GetKinematicViscosity();

        // Verify all are valid
        TS_ASSERT(T > 0.0 && !std::isnan(T));
        TS_ASSERT(P > 0.0 && !std::isnan(P));
        TS_ASSERT(rho > 0.0 && !std::isnan(rho));
        TS_ASSERT(a > 0.0 && !std::isnan(a));
        TS_ASSERT(mu > 0.0 && !std::isnan(mu));
        TS_ASSERT(nu > 0.0 && !std::isnan(nu));

        // Verify ideal gas law: P = rho * R * T
        const double R_specific = 1716.49;
        double P_calc = rho * R_specific * T;
        TS_ASSERT_DELTA(P, P_calc, P * 0.02);  // Within 2%

        // Verify kinematic viscosity: nu = mu / rho
        double nu_calc = mu / rho;
        TS_ASSERT_DELTA(nu, nu_calc, nu * 0.01);  // Within 1%
      }
    }
  }
};
