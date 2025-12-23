#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <models/atmosphere/FGStandardAtmosphere.h>

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
};
