#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <models/FGAtmosphere.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();

using namespace JSBSim;

class DummyAtmosphere : public FGAtmosphere
{
public:
  DummyAtmosphere(FGFDMExec* fdm, double t_lapse_rate, double p_lapse_rate)
    : FGAtmosphere(fdm), a_t(t_lapse_rate), a_p(p_lapse_rate)
  {}

  ~DummyAtmosphere() { PropertyManager->Unbind(this); }

  using FGAtmosphere::GetTemperature;
  using FGAtmosphere::GetPressure;

  double GetTemperature(double altitude) const override
  {
    return ValidateTemperature(SLtemperature+a_t*altitude, "", true);
  }
  void SetTemperature(double t, double h, eTemperature unit) override
  {
    SetTemperatureSL(ConvertToRankine(t, unit)-a_t*h, eRankine);
  }
  double GetPressure(double altitude) const override
  {
    return ValidatePressure(SLpressure+a_p*altitude, "", true);
  }
  // Getters for the protected members
  static constexpr double GetR(void) { return Reng0; }
  static constexpr double GetBeta(void) { return Beta; }
  static constexpr double GetSutherlandConstant(void) { return SutherlandConstant; }
  static constexpr double GetPSFtoPa(void) { return psftopa; }
  static constexpr double GetPSFtoInHg(void) { return psftoinhg; }
private:
  double a_t, a_p;
};

constexpr double R = DummyAtmosphere::GetR();
constexpr double gama = FGAtmosphere::SHRatio;
constexpr double beta = DummyAtmosphere::GetBeta();
constexpr double k = DummyAtmosphere::GetSutherlandConstant();
constexpr double psftopa = DummyAtmosphere::GetPSFtoPa();
constexpr double psftombar = psftopa/100.;
constexpr double psftoinhg = DummyAtmosphere::GetPSFtoInHg();

class FGAtmosphereTest : public CxxTest::TestSuite
{
public:
  FGFDMExec fdmex;

  FGAtmosphereTest() {
    auto atm = fdmex.GetAtmosphere();
    fdmex.GetPropertyManager()->Unbind(atm);
  }

  void testDefaultValuesBeforeInit()
  {
    FGJSBBase::debug_lvl = 2;
    auto atm = DummyAtmosphere(&fdmex, 1.0, 1.0);

    TS_ASSERT_EQUALS(atm.GetTemperatureSL(), 1.8);
    TS_ASSERT_EQUALS(atm.GetTemperature(), 1.8);
    TS_ASSERT_EQUALS(atm.GetTemperature(0.0), 1.8);
    TS_ASSERT_EQUALS(atm.GetTemperatureRatio(), 1.0);
    TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);

    TS_ASSERT_EQUALS(atm.GetPressureSL(), 1.0);
    TS_ASSERT_EQUALS(atm.GetPressure(), 0.0);
    TS_ASSERT_EQUALS(atm.GetPressure(0.0), 1.0);
    TS_ASSERT_EQUALS(atm.GetPressureRatio(), 0.0);

    const double rho = 1.0/(R*1.8);
    TS_ASSERT_EQUALS(atm.GetDensitySL(), 1.0);
    TS_ASSERT_EQUALS(atm.GetDensity(), 0.0);
    TS_ASSERT_EQUALS(atm.GetDensity(0.0), rho);
    TS_ASSERT_EQUALS(atm.GetDensityRatio(), 0.0);

    const double a = sqrt(gama*R*1.8);
    TS_ASSERT_EQUALS(atm.GetSoundSpeedSL(), 1.0);
    TS_ASSERT_EQUALS(atm.GetSoundSpeed(), 0.0);
    TS_ASSERT_EQUALS(atm.GetSoundSpeed(0.0), a);
    TS_ASSERT_EQUALS(atm.GetSoundSpeedRatio(), 0.0);

    TS_ASSERT_EQUALS(atm.GetDensityAltitude(), 0.0);
    TS_ASSERT_EQUALS(atm.GetPressureAltitude(), 0.0);

    TS_ASSERT_EQUALS(atm.GetAbsoluteViscosity(), 0.0);
    TS_ASSERT_EQUALS(atm.GetKinematicViscosity(), 0.0);

    FGJSBBase::debug_lvl = 0;
  }

  void testDefaultValuesAfterInit()
  {
    auto atm = DummyAtmosphere(&fdmex, 1.0, 1.0);

    TS_ASSERT(atm.InitModel());

    const double T0 = FGAtmosphere::StdDaySLtemperature;
    const double P0 = FGAtmosphere::StdDaySLpressure;

    TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
    TS_ASSERT_EQUALS(atm.GetTemperature(), T0);
    TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
    TS_ASSERT_EQUALS(atm.GetTemperatureRatio(), 1.0);
    TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
    TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
    TS_ASSERT_EQUALS(atm.GetPressure(), P0);
    TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
    TS_ASSERT_EQUALS(atm.GetPressureRatio(), 1.0);

    const double SLdensity = P0/(R*T0);
    TS_ASSERT_EQUALS(atm.GetDensity(), SLdensity);
    TS_ASSERT_EQUALS(atm.GetDensity(0.0), SLdensity);
    TS_ASSERT_EQUALS(atm.GetDensitySL(), SLdensity);
    TS_ASSERT_EQUALS(atm.GetDensityRatio(), 1.0);

    const double SLsoundspeed = sqrt(gama*R*T0);
    TS_ASSERT_EQUALS(atm.GetSoundSpeed(), SLsoundspeed);
    TS_ASSERT_EQUALS(atm.GetSoundSpeed(0.0), SLsoundspeed);
    TS_ASSERT_EQUALS(atm.GetSoundSpeedSL(), SLsoundspeed);
    TS_ASSERT_EQUALS(atm.GetSoundSpeedRatio(), 1.0);

    TS_ASSERT_EQUALS(atm.GetDensityAltitude(), 0.0);
    TS_ASSERT_EQUALS(atm.GetPressureAltitude(), 0.0);

    const double mu = beta*T0*sqrt(T0)/(k+T0);
    const double nu = mu/SLdensity;
    TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
    TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
  }

  void testGetAltitudeParameters()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);
    const double mu0 = beta*T0*sqrt(T0)/(k+T0);
    const double nu0 = mu0/rho0;

    for(double h=-1000.0; h<10000; h+= 1000) {
      double T = T0 + 0.1*h;
      double P = P0 + 1.0*h;

      TS_ASSERT_DELTA(atm.GetTemperature(h), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);

      double rho = P/(R*T);
      TS_ASSERT_DELTA(atm.GetDensity(h), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);

      // Local values must remain unchanged
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_EQUALS(atm.GetTemperature(), T0);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(), 1.0);
      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_EQUALS(atm.GetPressure(), P0);
      TS_ASSERT_EQUALS(atm.GetPressureRatio(), 1.0);
      TS_ASSERT_DELTA(atm.GetDensity(), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), 1.0);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_EQUALS(atm.GetSoundSpeedRatio(), 1.0);
      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), 0.0);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), 0.0);
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu0, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu0, epsilon);
    }
  }

  void testRun()
  {
    auto pm = fdmex.GetPropertyManager();
    auto T_node = pm->GetNode("atmosphere/T-R");
    auto rho_node = pm->GetNode("atmosphere/rho-slugs_ft3");
    auto P_node = pm->GetNode("atmosphere/P-psf");
    auto a_node = pm->GetNode("atmosphere/a-fps");
    auto T0_node = pm->GetNode("atmosphere/T-sl-R");
    auto rho0_node = pm->GetNode("atmosphere/rho-sl-slugs_ft3");
    auto a0_node = pm->GetNode("atmosphere/a-sl-fps");
    auto theta_node = pm->GetNode("atmosphere/theta");
    auto sigma_node = pm->GetNode("atmosphere/sigma");
    auto delta_node = pm->GetNode("atmosphere/delta");
    auto a_ratio_node = pm->GetNode("atmosphere/a-ratio");
    auto density_altitude_node = pm->GetNode("atmosphere/density-altitude");
    auto pressure_altitude_node = pm->GetNode("atmosphere/pressure-altitude");

    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);

    for(double h=-1000.0; h<10000; h+= 1000) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = T0 + 0.1*h;
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_DELTA(T0_node->getDoubleValue(), T0, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperature(), T, epsilon);
      TS_ASSERT_DELTA(T_node->getDoubleValue(), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(h), T, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), T/T0, epsilon);
      TS_ASSERT_DELTA(theta_node->getDoubleValue(), T/T0, epsilon);

      double P = P0 + 1.0*h;
      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_DELTA(atm.GetPressure(), P, epsilon);
      TS_ASSERT_DELTA(P_node->getDoubleValue(), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P, epsilon);
      TS_ASSERT_DELTA(atm.GetPressureRatio(), P/P0, epsilon);
      TS_ASSERT_DELTA(delta_node->getDoubleValue(), P/P0, epsilon);

      double rho = P/(R*T);
      TS_ASSERT_DELTA(atm.GetDensity(), rho, epsilon);
      TS_ASSERT_DELTA(rho_node->getDoubleValue(), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(h), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_DELTA(rho0_node->getDoubleValue(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), rho/rho0);
      TS_ASSERT_DELTA(sigma_node->getDoubleValue(), rho/rho0, epsilon);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a, epsilon);
      TS_ASSERT_DELTA(a_node->getDoubleValue(), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_DELTA(a0_node->getDoubleValue(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a/a0, epsilon);
      TS_ASSERT_DELTA(a_ratio_node->getDoubleValue(), a/a0, epsilon);

      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), h);
      TS_ASSERT_EQUALS(density_altitude_node->getDoubleValue(), h);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), h);
      TS_ASSERT_EQUALS(pressure_altitude_node->getDoubleValue(), h);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
    }
  }

  void testTemperatureOverride()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);

    auto t_node = pm->GetNode("atmosphere/override/temperature", true);
    const double T = 300.0;
    t_node->setDoubleValue(T);

    for(double h=-1000.0; h<10000; h+= 1000) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double Tz = T0+0.1*h;
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(h), Tz, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), 1.0+0.1*h/T0, epsilon);

      double P = P0 + 1.0*h;
      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_DELTA(atm.GetPressure(), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P, epsilon);
      TS_ASSERT_DELTA(atm.GetPressureRatio(), P/P0, epsilon);

      double rho = P/(R*T);
      TS_ASSERT_DELTA(atm.GetDensity(), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(h), P/(R*Tz), epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), rho/rho0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), sqrt(gama*R*Tz), epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a/a0, epsilon);

      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), h);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), h);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
    }

    // Detach the property atmosphere/override/temperature
    auto parent = t_node->getParent();
    parent->removeChild(t_node);
  }

  void testPressureOverride()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);

    auto p_node = pm->GetNode("atmosphere/override/pressure", true);
    const double P = 3000.0;
    p_node->setDoubleValue(P);

    for(double h=-1000.0; h<10000; h+= 1000) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = T0 + 0.1*h;
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(h), T, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), T/T0, epsilon);

      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_DELTA(atm.GetPressure(), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P0+h, epsilon);
      TS_ASSERT_DELTA(atm.GetPressureRatio(), P/P0, epsilon);

      double rho = P/(R*T);
      TS_ASSERT_DELTA(atm.GetDensity(), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(h), (P0+h)/(R*T), epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), rho/rho0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a/a0, epsilon);

      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), h);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), h);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
    }

    // Detach the property atmosphere/override/pressure
    auto parent = p_node->getParent();
    parent->removeChild(p_node);
  }

  void testDensityOverride()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);

    auto rho_node = pm->GetNode("atmosphere/override/density", true);
    const double rho = 3000.0;
    rho_node->setDoubleValue(rho);

    for(double h=-1000.0; h<10000; h+= 1000) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = T0 + 0.1*h;
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(h), T, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), T/T0, epsilon);

      double P = P0 + 1.0*h;
      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_DELTA(atm.GetPressure(), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P, epsilon);
      TS_ASSERT_DELTA(atm.GetPressureRatio(), P/P0, epsilon);

      TS_ASSERT_DELTA(atm.GetDensity(), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(h), P/(R*T), epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), rho/rho0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a/a0, epsilon);

      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), h);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), h);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
    }

    // Detach the property atmosphere/override/density
    auto parent = rho_node->getParent();
    parent->removeChild(rho_node);
  }

  void testSetTemperatureSL()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = 300.0;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);

    atm.SetTemperatureSL(T0, FGAtmosphere::eRankine);

    for(double h=-1000.0; h<10000; h+= 1000) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = T0+0.1*h;
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(h), T, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), T/T0, epsilon);

      double P = P0 + 1.0*h;
      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_DELTA(atm.GetPressure(), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P, epsilon);
      TS_ASSERT_DELTA(atm.GetPressureRatio(), P/P0, epsilon);

      double rho = P/(R*T);
      TS_ASSERT_DELTA(atm.GetDensity(), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(h), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), rho/rho0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a/a0, epsilon);

      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), h);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), h);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
    }
  }

  void testSetPressureSL()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = 3000.0;
    constexpr double rho0 = P0/(R*T0);
    const double a0 = sqrt(gama*R*T0);

    atm.SetPressureSL(FGAtmosphere::ePSF, P0);

    for(double h=-1000.0; h<10000; h+= 1000) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = T0+0.1*h;
      TS_ASSERT_EQUALS(atm.GetTemperatureSL(), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(), T, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperature(0.0), T0);
      TS_ASSERT_DELTA(atm.GetTemperature(h), T, epsilon);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T/T0, epsilon);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(0.0), 1.0);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), T/T0, epsilon);

      double P = P0 + 1.0*h;
      TS_ASSERT_EQUALS(atm.GetPressureSL(), P0);
      TS_ASSERT_DELTA(atm.GetPressure(), P, epsilon);
      TS_ASSERT_EQUALS(atm.GetPressure(0.0), P0);
      TS_ASSERT_DELTA(atm.GetPressure(h), P, epsilon);
      TS_ASSERT_DELTA(atm.GetPressureRatio(), P/P0, epsilon);

      double rho = P/(R*T);
      TS_ASSERT_DELTA(atm.GetDensity(), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(0.0), rho0, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(h), rho, epsilon);
      TS_ASSERT_DELTA(atm.GetDensitySL(), rho0, epsilon);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), rho/rho0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(0.0), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedSL(), a0, epsilon);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a/a0, epsilon);

      TS_ASSERT_EQUALS(atm.GetDensityAltitude(), h);
      TS_ASSERT_EQUALS(atm.GetPressureAltitude(), h);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);
    }
  }

  void testPressureConversion()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    double P0 = 900.0;  // mbar
    atm.SetPressureSL(FGAtmosphere::eMillibars, P0);
    TS_ASSERT_DELTA(atm.GetPressureSL()*psftombar / P0, 1.0, 1e-5);
    TS_ASSERT_DELTA(atm.GetPressureSL(FGAtmosphere::eMillibars) / P0, 1.0, 1e-5);

    P0 *= 100.0;  // Pa
    atm.SetPressureSL(FGAtmosphere::ePascals, P0);
    TS_ASSERT_DELTA(atm.GetPressureSL()*psftopa / P0, 1.0, 1e-5);
    TS_ASSERT_DELTA(atm.GetPressureSL(FGAtmosphere::ePascals) / P0, 1.0, 1e-5);

    P0 = 25.0;  // inHg
    atm.SetPressureSL(FGAtmosphere::eInchesHg, P0);
    TS_ASSERT_DELTA(atm.GetPressureSL()*psftoinhg / P0, 1.0, 1e-3);
    TS_ASSERT_DELTA(atm.GetPressureSL(FGAtmosphere::eInchesHg) / P0, 1.0, 1e-3);

    // Illegal units
    TS_ASSERT_THROWS(atm.SetPressureSL(FGAtmosphere::eNoPressUnit, P0), BaseException&);
    TS_ASSERT_THROWS(atm.GetPressureSL(FGAtmosphere::eNoPressUnit), BaseException&);
  }

  void testTemperatureConversion()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    double T0 = 250.0;  // K
    atm.SetTemperatureSL(T0, FGAtmosphere::eKelvin);
    TS_ASSERT_DELTA(atm.GetTemperatureSL()*5.0/9.0, T0, epsilon);

    T0 = -30.0;  // Celsius
    atm.SetTemperatureSL(T0, FGAtmosphere::eCelsius);
    TS_ASSERT_DELTA(atm.GetTemperatureSL()*5.0/9.0-273.15, T0, epsilon);

    T0 = 10.0;  // Fahrenheit
    atm.SetTemperatureSL(T0, FGAtmosphere::eFahrenheit);
    TS_ASSERT_DELTA(atm.GetTemperatureSL()-459.67, T0, epsilon);

    // Illegal units
    TS_ASSERT_THROWS(atm.SetTemperatureSL(T0, FGAtmosphere::eNoTempUnit), BaseException&);
  }

  void testAltitudeParametersValidation()
  {
    auto atm = DummyAtmosphere(&fdmex, -1.0, -100.0);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 1000;
    TS_ASSERT(atm.Run(false) == false);

    TS_ASSERT_EQUALS(atm.GetTemperature(), 1.8);
    TS_ASSERT_DELTA(atm.GetPressure()*psftopa*1e15, 1.0, 1e-5);
  }

  void testSeaLevelParametersValidation()
  {
    auto atm = DummyAtmosphere(&fdmex, -1.0, -100.0);
    TS_ASSERT(atm.InitModel());

    atm.SetTemperatureSL(0.0, FGAtmosphere::eKelvin);
    TS_ASSERT_EQUALS(atm.GetTemperatureSL(), 1.8);

    atm.SetPressureSL(FGAtmosphere::ePascals, 0.0);
    TS_ASSERT_DELTA(atm.GetPressureSL()*psftopa*1e15, 1.0, 1e-5);
  }

  void testProbeAtADifferentAltitude()
  {
    auto atm = DummyAtmosphere(&fdmex, -1.0, -100.0);
    TS_ASSERT(atm.InitModel());

    TS_ASSERT_EQUALS(atm.GetTemperature(1000.), 1.8);
    TS_ASSERT_DELTA(atm.GetPressure(1000.)*psftopa*1e15, 1.0, 1e-5);
  }

  void testConversionMethods()
  {
    // Test conversion methods through SetTemperatureSL and GetTemperatureSL
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Test Fahrenheit conversion
    double T_F = 59.0;
    atm.SetTemperatureSL(T_F, FGAtmosphere::eFahrenheit);
    double T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, T_F + 459.67, epsilon);

    // Test Celsius conversion
    double T_C = 15.0;
    atm.SetTemperatureSL(T_C, FGAtmosphere::eCelsius);
    T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, (T_C + 273.15) * 1.8, epsilon);

    // Test Kelvin conversion
    double T_K = 288.15;
    atm.SetTemperatureSL(T_K, FGAtmosphere::eKelvin);
    T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, T_K * 1.8, epsilon);
  }

  void testMultiplePressureUnits()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Test setting and getting pressure in PSF
    double P_psf = 2000.0;
    atm.SetPressureSL(FGAtmosphere::ePSF, P_psf);
    TS_ASSERT_DELTA(atm.GetPressureSL(FGAtmosphere::ePSF), P_psf, epsilon);

    // Test getting pressure in millibars
    double P_mbar = atm.GetPressureSL(FGAtmosphere::eMillibars);
    TS_ASSERT_DELTA(P_mbar, P_psf / 2.08854342, 1e-5);

    // Test getting pressure in Pascals
    double P_pa = atm.GetPressureSL(FGAtmosphere::ePascals);
    TS_ASSERT_DELTA(P_pa, P_psf / 0.0208854342, 1e-5);

    // Test getting pressure in InchesHg
    double P_inhg = atm.GetPressureSL(FGAtmosphere::eInchesHg);
    TS_ASSERT_DELTA(P_inhg, P_psf / 70.7180803, 1e-3);
  }

  void testDensityAtVariousAltitudes()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003566, 0.0);  // Standard lapse rate
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // Test density calculation at sea level
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    double rho0 = P0 / (R * T0);
    TS_ASSERT_DELTA(atm.GetDensity(), rho0, epsilon);

    // Test density at 10,000 ft
    double h = 10000.0;
    double T_h = T0 - 0.003566 * h;
    double rho_h = P0 / (R * T_h);
    TS_ASSERT_DELTA(atm.GetDensity(h), rho_h, epsilon);
  }

  void testSoundSpeedAtVariousAltitudes()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003566, 0.0);  // Standard lapse rate
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    const double a0 = sqrt(gama * R * T0);

    // Test sound speed at sea level
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT_DELTA(atm.GetSoundSpeed(), a0, epsilon);

    // Test sound speed at altitude
    double h = 5000.0;
    double T_h = T0 - 0.003566 * h;
    double a_h = sqrt(gama * R * T_h);
    TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a_h, epsilon);
  }

  void testViscosityCalculations()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    const double rho0 = P0 / (R * T0);

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    // Test absolute viscosity (Sutherland's formula)
    double mu = beta * T0 * sqrt(T0) / (k + T0);
    TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, epsilon);

    // Test kinematic viscosity
    double nu = mu / rho0;
    TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu, epsilon);

    // Test at higher temperature
    double T_hot = 600.0;
    atm.SetTemperatureSL(T_hot, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho_hot = P0 / (R * T_hot);
    double mu_hot = beta * T_hot * sqrt(T_hot) / (k + T_hot);
    double nu_hot = mu_hot / rho_hot;

    TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu_hot, epsilon);
    TS_ASSERT_DELTA(atm.GetKinematicViscosity(), nu_hot, epsilon);
  }

  void testTemperatureUnitConsistency()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Set temperature in Fahrenheit and verify consistency
    double T_F = 59.0;
    atm.SetTemperatureSL(T_F, FGAtmosphere::eFahrenheit);
    double T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, T_F + 459.67, epsilon);

    // Set temperature in Celsius and verify consistency
    double T_C = 15.0;
    atm.SetTemperatureSL(T_C, FGAtmosphere::eCelsius);
    T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, (T_C + 273.15) * 1.8, epsilon);

    // Set temperature in Kelvin and verify consistency
    double T_K = 288.15;
    atm.SetTemperatureSL(T_K, FGAtmosphere::eKelvin);
    T_R = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_R, T_K * 1.8, epsilon);
  }

  void testSetTemperatureAtAltitude()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    // Set temperature at 5000 ft
    double h = 5000.0;
    double T_target = 520.0;  // Rankine (a safe, high value)

    atm.SetTemperature(T_target, h, FGAtmosphere::eRankine);

    // With lapse rate of 0.1, at altitude h:
    // T(h) = SLtemperature + 0.1*h
    // SetTemperature sets SLtemperature = T_target - 0.1*h
    // So T(h) should equal T_target
    TS_ASSERT_DELTA(atm.GetTemperature(h), T_target, 0.1);
  }

  void testNegativeAltitudes()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // Test below sea level (e.g., Death Valley)
    double h = -282.0;  // Below sea level

    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    double T_h = T0 + 0.1 * h;
    double P_h = P0 + 1.0 * h;

    TS_ASSERT_DELTA(atm.GetTemperature(), T_h, epsilon);
    TS_ASSERT_DELTA(atm.GetPressure(), P_h, epsilon);

    // Verify density and sound speed
    double rho_h = P_h / (R * T_h);
    double a_h = sqrt(gama * R * T_h);

    TS_ASSERT_DELTA(atm.GetDensity(), rho_h, epsilon);
    TS_ASSERT_DELTA(atm.GetSoundSpeed(), a_h, epsilon);
  }

  void testRatioMethods()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    const double rho0 = P0 / (R * T0);
    const double a0 = sqrt(gama * R * T0);

    double h = 8000.0;
    double T_h = T0 + 0.1 * h;
    double P_h = P0 + 1.0 * h;
    double rho_h = P_h / (R * T_h);
    double a_h = sqrt(gama * R * T_h);

    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    // Test temperature ratio
    TS_ASSERT_DELTA(atm.GetTemperatureRatio(), T_h / T0, epsilon);

    // Test pressure ratio
    TS_ASSERT_DELTA(atm.GetPressureRatio(), P_h / P0, epsilon);

    // Test density ratio
    TS_ASSERT_DELTA(atm.GetDensityRatio(), rho_h / rho0, epsilon);

    // Test sound speed ratio
    TS_ASSERT_DELTA(atm.GetSoundSpeedRatio(), a_h / a0, epsilon);
  }

  void testPropertyBindings()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Verify key properties are bound
    TS_ASSERT(pm->HasNode("atmosphere/T-R"));
    TS_ASSERT(pm->HasNode("atmosphere/rho-slugs_ft3"));
    TS_ASSERT(pm->HasNode("atmosphere/P-psf"));
    TS_ASSERT(pm->HasNode("atmosphere/a-fps"));
    TS_ASSERT(pm->HasNode("atmosphere/T-sl-R"));
    TS_ASSERT(pm->HasNode("atmosphere/rho-sl-slugs_ft3"));
    TS_ASSERT(pm->HasNode("atmosphere/a-sl-fps"));
    TS_ASSERT(pm->HasNode("atmosphere/theta"));
    TS_ASSERT(pm->HasNode("atmosphere/sigma"));
    TS_ASSERT(pm->HasNode("atmosphere/delta"));
    TS_ASSERT(pm->HasNode("atmosphere/a-ratio"));
    TS_ASSERT(pm->HasNode("atmosphere/density-altitude"));
    TS_ASSERT(pm->HasNode("atmosphere/pressure-altitude"));
  }

  void testHighAltitude()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.001, 0.0);
    TS_ASSERT(atm.InitModel());

    // Test at high altitude (50,000 ft)
    double h = 50000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    double T_h = T0 - 0.001 * h;

    TS_ASSERT_DELTA(atm.GetTemperature(), T_h, epsilon);

    // Verify temperature is validated to minimum
    double min_validated_temp = atm.GetTemperature();
    TS_ASSERT(min_validated_temp > 0.0);  // Should be above absolute zero
  }

  void testVeryLowTemperatureValidation()
  {
    auto atm = DummyAtmosphere(&fdmex, -1.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Try to set a temperature below absolute zero
    atm.SetTemperatureSL(0.0, FGAtmosphere::eKelvin);

    // Should be capped to minimum
    double T_SL = atm.GetTemperatureSL();
    TS_ASSERT(T_SL >= 1.8);  // Minimum temperature (1K in Rankine)
  }

  void testTemperatureInDifferentUnits()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Set temperature in Celsius
    double T_C = 20.0;
    atm.SetTemperatureSL(T_C, FGAtmosphere::eCelsius);
    double T_SL = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_SL, (T_C + 273.15) * 1.8, epsilon);

    // Set temperature in Kelvin
    double T_K = 300.0;
    atm.SetTemperatureSL(T_K, FGAtmosphere::eKelvin);
    T_SL = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_SL, T_K * 1.8, epsilon);

    // Set temperature in Fahrenheit
    double T_F = 70.0;
    atm.SetTemperatureSL(T_F, FGAtmosphere::eFahrenheit);
    T_SL = atm.GetTemperatureSL();
    TS_ASSERT_DELTA(T_SL, T_F + 459.67, epsilon);
  }

  // Additional comprehensive atmosphere tests

  void testDynamicPressureCalculation()
  {
    // Use lapse rates that ensure density decreases with altitude
    // With T decreasing and P decreasing faster, density will decrease
    auto atm = DummyAtmosphere(&fdmex, -0.001, -0.3);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // At sea level
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho0 = atm.GetDensity();
    double velocity = 200.0;  // ft/s
    double q = 0.5 * rho0 * velocity * velocity;
    TS_ASSERT(q > 0.0);

    // At altitude with proper lapse rates, density should be lower
    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho_alt = atm.GetDensity();
    double q_alt = 0.5 * rho_alt * velocity * velocity;

    // Verify dynamic pressure relationship with density
    TS_ASSERT(rho_alt > 0.0);
    TS_ASSERT(q_alt > 0.0);
    TS_ASSERT_DELTA(q_alt / q, rho_alt / rho0, epsilon);
  }

  void testMachNumberRelation()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003566, -0.001);
    TS_ASSERT(atm.InitModel());

    // At sea level
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double a0 = atm.GetSoundSpeed();
    double velocity = 600.0;  // ft/s
    double mach0 = velocity / a0;

    // At altitude, sound speed is lower (colder temperature)
    atm.in.altitudeASL = 30000.0;
    TS_ASSERT(atm.Run(false) == false);

    double a_alt = atm.GetSoundSpeed();
    double mach_alt = velocity / a_alt;

    // Same TAS but lower temperature means higher Mach
    TS_ASSERT(a_alt < a0);
    TS_ASSERT(mach_alt > mach0);
  }

  void testReynoldsNumberComponents()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003566, -0.001);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho = atm.GetDensity();
    double mu = atm.GetAbsoluteViscosity();
    double nu = atm.GetKinematicViscosity();

    // Kinematic viscosity = absolute viscosity / density
    TS_ASSERT_DELTA(nu, mu / rho, epsilon);

    // Reynolds number = V * L / nu
    double velocity = 500.0;  // ft/s
    double length = 10.0;     // ft (characteristic length)
    double Re = velocity * length / nu;
    TS_ASSERT(Re > 1e6);  // Typical aircraft Reynolds numbers
  }

  void testISADeviation()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003566, -0.001);
    TS_ASSERT(atm.InitModel());

    constexpr double T_std = FGAtmosphere::StdDaySLtemperature;

    // Standard day
    atm.SetTemperatureSL(T_std, FGAtmosphere::eRankine);
    double rho_std = atm.GetDensity();

    // Hot day (+20 Rankine deviation)
    double T_hot = T_std + 20.0;
    atm.SetTemperatureSL(T_hot, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho_hot = atm.GetDensity();
    TS_ASSERT(rho_hot < rho_std);  // Hot air is less dense

    // Cold day (-20 Rankine deviation)
    double T_cold = T_std - 20.0;
    atm.SetTemperatureSL(T_cold, FGAtmosphere::eRankine);
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double rho_cold = atm.GetDensity();
    TS_ASSERT(rho_cold > rho_std);  // Cold air is more dense
  }

  void testPressureAltitudeVsDensityAltitude()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // At sea level on a standard day
    atm.in.altitudeASL = 0.0;
    TS_ASSERT(atm.Run(false) == false);

    double pa = atm.GetPressureAltitude();
    double da = atm.GetDensityAltitude();

    // On standard day at sea level, both should be 0
    TS_ASSERT_EQUALS(pa, 0.0);
    TS_ASSERT_EQUALS(da, 0.0);
  }

  void testAltitudeEffectOnDensity()
  {
    // Use lapse rates where pressure drops faster than temperature
    // to ensure density decreases with altitude
    auto atm = DummyAtmosphere(&fdmex, -0.001, -0.5);
    TS_ASSERT(atm.InitModel());

    // Verify density is positive at various altitudes
    for (double h = 0.0; h <= 10000.0; h += 2000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double current_density = atm.GetDensity();
      TS_ASSERT(current_density > 0.0);

      // Verify ideal gas law holds
      double T = atm.GetTemperature();
      double P = atm.GetPressure();
      TS_ASSERT_DELTA(current_density, P / (R * T), epsilon);
    }
  }

  void testTemperatureLapseRate()
  {
    // Standard tropospheric lapse rate: ~-0.003566 R/ft (-1.98 K/1000ft)
    double lapse_rate = -0.003566;
    auto atm = DummyAtmosphere(&fdmex, lapse_rate, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    // Temperature should decrease with altitude in troposphere
    for (double h = 0.0; h <= 35000.0; h += 5000.0) {
      double expected_T = T0 + lapse_rate * h;
      TS_ASSERT_DELTA(atm.GetTemperature(h), expected_T, epsilon);
    }
  }

  void testSoundSpeedRatioConsistency()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    for (double h = 0.0; h <= 20000.0; h += 2000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double a = atm.GetSoundSpeed();
      double a0 = atm.GetSoundSpeedSL();
      double ratio = atm.GetSoundSpeedRatio();

      TS_ASSERT_DELTA(ratio, a / a0, epsilon);
    }
  }

  void testDensityRatioConsistency()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    for (double h = 0.0; h <= 20000.0; h += 2000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double rho = atm.GetDensity();
      double rho0 = atm.GetDensitySL();
      double sigma = atm.GetDensityRatio();

      TS_ASSERT_EQUALS(sigma, rho / rho0);
    }
  }

  void testPressureRatioConsistency()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, -0.2);
    TS_ASSERT(atm.InitModel());

    for (double h = 0.0; h <= 20000.0; h += 2000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double P = atm.GetPressure();
      double P0 = atm.GetPressureSL();
      double delta = atm.GetPressureRatio();

      TS_ASSERT_DELTA(delta, P / P0, epsilon);
    }
  }

  void testIdealGasLaw()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.15);
    TS_ASSERT(atm.InitModel());

    // P = rho * R * T (ideal gas law)
    for (double h = 0.0; h <= 30000.0; h += 3000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double P = atm.GetPressure();
      double rho = atm.GetDensity();
      double T = atm.GetTemperature();

      // P = rho * R * T
      double P_calc = rho * R * T;
      TS_ASSERT_DELTA(P, P_calc, epsilon * P);
    }
  }

  void testSutherlandViscosityFormula()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    // Test Sutherland's formula: mu = beta * T^1.5 / (k + T)
    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double mu_expected = beta * T * sqrt(T) / (k + T);
      double mu_actual = atm.GetAbsoluteViscosity();

      TS_ASSERT_DELTA(mu_actual, mu_expected, epsilon);
    }
  }

  void testSoundSpeedFormula()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    // a = sqrt(gamma * R * T)
    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double a_expected = sqrt(gama * R * T);
      double a_actual = atm.GetSoundSpeed();

      TS_ASSERT_DELTA(a_actual, a_expected, epsilon);
    }
  }

  void testOverridePropertyRemoval()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    // Set temperature override
    auto t_node = pm->GetNode("atmosphere/override/temperature", true);
    t_node->setDoubleValue(500.0);

    atm.in.altitudeASL = 1000.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT_DELTA(atm.GetTemperature(), 500.0, epsilon);

    // Remove override
    auto parent = t_node->getParent();
    parent->removeChild(t_node);

    // Now temperature should revert to normal
    TS_ASSERT(atm.Run(false) == false);
    double expected_T = T0 + 0.1 * 1000.0;
    TS_ASSERT_DELTA(atm.GetTemperature(), expected_T, epsilon);
  }

  void testMultipleOverridesSimultaneously()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, 0.1, 1.0);
    TS_ASSERT(atm.InitModel());

    // Create temperature and pressure overrides
    auto t_node = pm->GetNode("atmosphere/override/temperature", true);
    auto p_node = pm->GetNode("atmosphere/override/pressure", true);

    double T_override = 400.0;
    double P_override = 1500.0;

    t_node->setDoubleValue(T_override);
    p_node->setDoubleValue(P_override);

    atm.in.altitudeASL = 5000.0;
    TS_ASSERT(atm.Run(false) == false);

    // Both overrides should be active
    TS_ASSERT_DELTA(atm.GetTemperature(), T_override, epsilon);
    TS_ASSERT_DELTA(atm.GetPressure(), P_override, epsilon);

    // Density should be calculated from overridden values
    double rho_expected = P_override / (R * T_override);
    TS_ASSERT_DELTA(atm.GetDensity(), rho_expected, epsilon);

    // Cleanup
    auto parent = t_node->getParent();
    parent->removeChild(t_node);
    parent->removeChild(p_node);
  }

  void testTemperatureRatioAtAltitude()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    for (double h = 0.0; h <= 40000.0; h += 5000.0) {
      double T_h = T0 - 0.003 * h;
      double theta_expected = T_h / T0;

      TS_ASSERT_DELTA(atm.GetTemperatureRatio(h), theta_expected, epsilon);
    }
  }

  void testAtmospherePropertyValues()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 10000.0;
    TS_ASSERT(atm.Run(false) == false);

    // Verify property values match method values
    auto T_node = pm->GetNode("atmosphere/T-R");
    auto P_node = pm->GetNode("atmosphere/P-psf");
    auto rho_node = pm->GetNode("atmosphere/rho-slugs_ft3");
    auto a_node = pm->GetNode("atmosphere/a-fps");

    TS_ASSERT_DELTA(T_node->getDoubleValue(), atm.GetTemperature(), epsilon);
    TS_ASSERT_DELTA(P_node->getDoubleValue(), atm.GetPressure(), epsilon);
    TS_ASSERT_DELTA(rho_node->getDoubleValue(), atm.GetDensity(), epsilon);
    TS_ASSERT_DELTA(a_node->getDoubleValue(), atm.GetSoundSpeed(), epsilon);
  }

  void testSeaLevelPropertyValues()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    atm.in.altitudeASL = 20000.0;
    TS_ASSERT(atm.Run(false) == false);

    // SL values should remain constant regardless of altitude
    auto T0_node = pm->GetNode("atmosphere/T-sl-R");
    auto rho0_node = pm->GetNode("atmosphere/rho-sl-slugs_ft3");
    auto a0_node = pm->GetNode("atmosphere/a-sl-fps");

    TS_ASSERT_DELTA(T0_node->getDoubleValue(), atm.GetTemperatureSL(), epsilon);
    TS_ASSERT_DELTA(rho0_node->getDoubleValue(), atm.GetDensitySL(), epsilon);
    TS_ASSERT_DELTA(a0_node->getDoubleValue(), atm.GetSoundSpeedSL(), epsilon);
  }

  void testAltitudePropertyValues()
  {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    double h = 15000.0;
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    auto da_node = pm->GetNode("atmosphere/density-altitude");
    auto pa_node = pm->GetNode("atmosphere/pressure-altitude");

    TS_ASSERT_EQUALS(da_node->getDoubleValue(), atm.GetDensityAltitude());
    TS_ASSERT_EQUALS(pa_node->getDoubleValue(), atm.GetPressureAltitude());
  }

  void testZeroLapseRate()
  {
    // Test with zero temperature lapse rate (isothermal)
    auto atm = DummyAtmosphere(&fdmex, 0.0, -0.1);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    // Temperature should remain constant at all altitudes
    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      TS_ASSERT_DELTA(atm.GetTemperature(h), T0, epsilon);
    }
  }

  void testZeroPressureLapseRate()
  {
    // Test with zero pressure lapse rate (constant pressure)
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // Pressure should remain constant at all altitudes
    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      TS_ASSERT_DELTA(atm.GetPressure(h), P0, epsilon);
    }
  }

  void testPositiveLapseRate()
  {
    // Test with positive temperature lapse rate (temperature increases with altitude)
    auto atm = DummyAtmosphere(&fdmex, 0.002, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    // Temperature should increase with altitude
    double prev_T = T0;
    for (double h = 1000.0; h <= 30000.0; h += 5000.0) {
      double current_T = atm.GetTemperature(h);
      TS_ASSERT(current_T > prev_T);
      prev_T = current_T;
    }
  }

  void testDensityDecreasesWithPressureDrop()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, -0.2);  // Constant temp, decreasing pressure
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    // With constant temperature and decreasing pressure, density should follow P/(R*T)
    for (double h = 0.0; h <= 5000.0; h += 1000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double current_rho = atm.GetDensity();
      double P = atm.GetPressure();
      double T = atm.GetTemperature();

      TS_ASSERT(current_rho > 0.0);
      // Verify ideal gas law
      TS_ASSERT_DELTA(current_rho, P / (R * T), epsilon);
      // Temperature should remain constant
      TS_ASSERT_DELTA(T, T0, epsilon);
    }
  }

  void testViscosityIncreasesWithTemperature()
  {
    // Viscosity increases with temperature in gases
    auto atm = DummyAtmosphere(&fdmex, 0.005, 0.0);  // Temperature increases with altitude
    TS_ASSERT(atm.InitModel());

    double prev_mu = 0.0;
    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double current_mu = atm.GetAbsoluteViscosity();
      TS_ASSERT(current_mu > prev_mu);
      prev_mu = current_mu;
    }
  }

  void testKinematicViscosityRelation()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double mu = atm.GetAbsoluteViscosity();
      double rho = atm.GetDensity();
      double nu = atm.GetKinematicViscosity();

      TS_ASSERT_DELTA(nu, mu / rho, epsilon * nu);
    }
  }

  void testSoundSpeedProportionalToSqrtTemp()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    for (double h = 0.0; h <= 30000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double a = atm.GetSoundSpeed();

      // a^2 = gamma * R * T
      double a_squared = a * a;
      double expected = gama * R * T;

      TS_ASSERT_DELTA(a_squared, expected, epsilon * a_squared);
    }
  }

  void testExtremePressureValues()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Test very high pressure
    double P_high = 5000.0;  // psf
    atm.SetPressureSL(FGAtmosphere::ePSF, P_high);
    TS_ASSERT_DELTA(atm.GetPressureSL(), P_high, epsilon);

    // Test low pressure (should be validated)
    double P_low = 100.0;  // psf
    atm.SetPressureSL(FGAtmosphere::ePSF, P_low);
    TS_ASSERT_DELTA(atm.GetPressureSL(), P_low, epsilon);
  }

  void testExtremeTemperatureValues()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Test very hot temperature
    double T_hot = 700.0;  // Rankine (about 240°F)
    atm.SetTemperatureSL(T_hot, FGAtmosphere::eRankine);
    TS_ASSERT_DELTA(atm.GetTemperatureSL(), T_hot, epsilon);

    // Test cold temperature
    double T_cold = 400.0;  // Rankine (about -60°F)
    atm.SetTemperatureSL(T_cold, FGAtmosphere::eRankine);
    TS_ASSERT_DELTA(atm.GetTemperatureSL(), T_cold, epsilon);
  }

  void testAltitudeLoopStability()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    // Run multiple times at same altitude - results should be stable
    atm.in.altitudeASL = 10000.0;

    TS_ASSERT(atm.Run(false) == false);
    double T1 = atm.GetTemperature();
    double P1 = atm.GetPressure();
    double rho1 = atm.GetDensity();

    for (int i = 0; i < 10; ++i) {
      TS_ASSERT(atm.Run(false) == false);
      TS_ASSERT_DELTA(atm.GetTemperature(), T1, epsilon);
      TS_ASSERT_DELTA(atm.GetPressure(), P1, epsilon);
      TS_ASSERT_DELTA(atm.GetDensity(), rho1, epsilon);
    }
  }

  void testAltitudeTransitionSmooth()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // Values should change smoothly with altitude following lapse rates
    for (double h = 0.0; h <= 10000.0; h += 1000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      // Verify temperature follows lapse rate
      double expected_T = T0 - 0.003 * h;
      TS_ASSERT_DELTA(atm.GetTemperature(), expected_T, 0.01);

      // Verify pressure follows lapse rate
      double expected_P = P0 - 0.1 * h;
      TS_ASSERT_DELTA(atm.GetPressure(), expected_P, 0.01);
    }
  }

  void testDensityVsTemperatureInverse()
  {
    // At constant pressure, density is inversely proportional to temperature
    auto atm = DummyAtmosphere(&fdmex, 0.005, 0.0);  // Increasing temp, constant pressure
    TS_ASSERT(atm.InitModel());

    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    for (double h = 0.0; h <= 20000.0; h += 5000.0) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double rho = atm.GetDensity();

      // rho = P / (R * T)
      double rho_expected = P0 / (R * T);
      TS_ASSERT_DELTA(rho, rho_expected, epsilon);
    }
  }

  void testAllPressureUnits()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, 0.0);
    TS_ASSERT(atm.InitModel());

    // Set known pressure in PSF
    double P_psf = 2116.22;  // Standard atmosphere pressure
    atm.SetPressureSL(FGAtmosphere::ePSF, P_psf);

    // Get in all units
    double P_mb = atm.GetPressureSL(FGAtmosphere::eMillibars);
    double P_pa = atm.GetPressureSL(FGAtmosphere::ePascals);
    double P_inhg = atm.GetPressureSL(FGAtmosphere::eInchesHg);
    double P_psf_back = atm.GetPressureSL(FGAtmosphere::ePSF);

    // Verify round-trip
    TS_ASSERT_DELTA(P_psf_back, P_psf, epsilon);

    // Verify conversions are consistent
    TS_ASSERT_DELTA(P_pa, P_mb * 100.0, 1.0);  // 100 Pa per mbar
  }

  void testTemperatureAtAltitudeMethod()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003566, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;

    // GetTemperature(h) should not affect the current state
    double T_10000 = atm.GetTemperature(10000.0);
    double T_20000 = atm.GetTemperature(20000.0);

    // These are just queries, current temp should still be T0
    TS_ASSERT_DELTA(atm.GetTemperature(), T0, epsilon);

    // Verify altitude-dependent temperatures
    TS_ASSERT_DELTA(T_10000, T0 - 0.003566 * 10000.0, epsilon);
    TS_ASSERT_DELTA(T_20000, T0 - 0.003566 * 20000.0, epsilon);
  }

  void testPressureAtAltitudeMethod()
  {
    auto atm = DummyAtmosphere(&fdmex, 0.0, -0.1);
    TS_ASSERT(atm.InitModel());

    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // GetPressure(h) should not affect the current state
    double P_10000 = atm.GetPressure(10000.0);
    double P_20000 = atm.GetPressure(20000.0);

    // Current pressure should still be P0
    TS_ASSERT_DELTA(atm.GetPressure(), P0, epsilon);

    // Verify altitude-dependent pressures
    TS_ASSERT_DELTA(P_10000, P0 - 0.1 * 10000.0, epsilon);
    TS_ASSERT_DELTA(P_20000, P0 - 0.1 * 20000.0, epsilon);
  }

  void testDensityAtAltitudeMethod()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;
    const double rho0 = P0 / (R * T0);

    // GetDensity(h) should calculate density at that altitude
    double h = 15000.0;
    double T_h = T0 - 0.003 * h;
    double P_h = P0 - 0.1 * h;
    double rho_h_expected = P_h / (R * T_h);

    TS_ASSERT_DELTA(atm.GetDensity(h), rho_h_expected, epsilon);

    // Current density should still be at SL
    TS_ASSERT_DELTA(atm.GetDensity(), rho0, epsilon);
  }

  void testSoundSpeedAtAltitudeMethod()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, 0.0);
    TS_ASSERT(atm.InitModel());

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    const double a0 = sqrt(gama * R * T0);

    // GetSoundSpeed(h) should calculate sound speed at that altitude
    double h = 25000.0;
    double T_h = T0 - 0.003 * h;
    double a_h_expected = sqrt(gama * R * T_h);

    TS_ASSERT_DELTA(atm.GetSoundSpeed(h), a_h_expected, epsilon);

    // Current sound speed should still be at SL
    TS_ASSERT_DELTA(atm.GetSoundSpeed(), a0, epsilon);
  }

  void testTropopauseLikeConditions()
  {
    // Simulate tropopause-like conditions (constant temperature above 36,000 ft)
    // Using a small lapse rate that doesn't go below minimum
    auto atm = DummyAtmosphere(&fdmex, -0.001, -0.05);
    TS_ASSERT(atm.InitModel());

    // At high altitude
    atm.in.altitudeASL = 40000.0;
    TS_ASSERT(atm.Run(false) == false);

    // Verify values are still valid
    TS_ASSERT(atm.GetTemperature() > 0.0);
    TS_ASSERT(atm.GetPressure() > 0.0);
    TS_ASSERT(atm.GetDensity() > 0.0);
    TS_ASSERT(atm.GetSoundSpeed() > 0.0);
  }

  void testVeryHighAltitude()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.0001, -0.01);  // Very small lapse rates
    TS_ASSERT(atm.InitModel());

    // Test at very high altitude (100,000 ft)
    atm.in.altitudeASL = 100000.0;
    TS_ASSERT(atm.Run(false) == false);

    // All values should remain physically valid
    TS_ASSERT(atm.GetTemperature() > 0.0);
    TS_ASSERT(atm.GetPressure() >= 0.0);
    TS_ASSERT(atm.GetDensity() >= 0.0);
    TS_ASSERT(atm.GetSoundSpeed() > 0.0);
    TS_ASSERT(atm.GetAbsoluteViscosity() > 0.0);
  }

  void testDeepNegativeAltitude()
  {
    // Use negative lapse rates (like standard atmosphere)
    // Temperature decreases with altitude, so increases below SL
    // Pressure decreases with altitude, so increases below SL
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    // Test at negative altitude (below sea level)
    double h = -500.0;  // 500 ft below sea level
    atm.in.altitudeASL = h;
    TS_ASSERT(atm.Run(false) == false);

    constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
    constexpr double P0 = FGAtmosphere::StdDaySLpressure;

    // With negative lapse rates:
    // T(h) = T0 + (-0.003) * (-500) = T0 + 1.5 (warmer below SL)
    // P(h) = P0 + (-0.1) * (-500) = P0 + 50 (higher pressure below SL)
    TS_ASSERT_DELTA(atm.GetTemperature(), T0 + 1.5, 0.01);
    TS_ASSERT_DELTA(atm.GetPressure(), P0 + 50.0, 0.01);
  }

  void testInitModelResets()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    // Change sea level values
    atm.SetTemperatureSL(600.0, FGAtmosphere::eRankine);
    atm.SetPressureSL(FGAtmosphere::ePSF, 3000.0);

    // Re-init should reset to standard values
    TS_ASSERT(atm.InitModel());

    TS_ASSERT_DELTA(atm.GetTemperatureSL(), FGAtmosphere::StdDaySLtemperature, epsilon);
    TS_ASSERT_DELTA(atm.GetPressureSL(), FGAtmosphere::StdDaySLpressure, epsilon);
  }

  void testConsecutiveRuns()
  {
    auto atm = DummyAtmosphere(&fdmex, -0.003, -0.1);
    TS_ASSERT(atm.InitModel());

    // Run at different altitudes sequentially
    double altitudes[] = {0.0, 5000.0, 10000.0, 15000.0, 20000.0};

    for (double h : altitudes) {
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      // Verify results match expected
      constexpr double T0 = FGAtmosphere::StdDaySLtemperature;
      constexpr double P0 = FGAtmosphere::StdDaySLpressure;

      double T_expected = T0 - 0.003 * h;
      double P_expected = P0 - 0.1 * h;

      TS_ASSERT_DELTA(atm.GetTemperature(), T_expected, epsilon);
      TS_ASSERT_DELTA(atm.GetPressure(), P_expected, epsilon);
    }
  }
};
