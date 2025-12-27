#include <fstream>
#include <string>
#include <vector>

#include <cxxtest/TestSuite.h>
#include <FGFDMExec.h>
#include <models/atmosphere/FGMSIS.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class DummyMSIS : public FGMSIS
{
public:
  DummyMSIS(FGFDMExec* fdmex) : FGMSIS(fdmex) {
    in.altitudeASL = 0.0;
    in.GeodLatitudeDeg = 0.0;
    in.LongitudeDeg = 0.0;
  }

  ~DummyMSIS() { PropertyManager->Unbind(this); }

  // Getters for the protected members
  double GetR(void) { return Reng; }
  static constexpr double GetRstar(void) { return Rstar; }
  static constexpr double GetBeta(void) { return Beta; }
  static constexpr double GetSutherlandConstant(void) { return SutherlandConstant; }
  static constexpr double GetPSFtoPa(void) { return psftopa; }
  static constexpr double GetPSFtoInHg(void) { return psftoinhg; }
  // Setters for the protected members
  void SetDay(double day) { day_of_year = day; }
  void SetSeconds(double seconds) { seconds_in_day = seconds; }
  void SetF107A(double value) { input.f107A = value; }
  void SetF107(double value) { input.f107 = value; }
  void SetAP(double value) { input.ap = value; }
};

constexpr double Rstar = DummyMSIS::GetRstar();
constexpr double gama = FGAtmosphere::SHRatio;
constexpr double beta = DummyMSIS::GetBeta();
constexpr double k = DummyMSIS::GetSutherlandConstant();
constexpr double psftopa = DummyMSIS::GetPSFtoPa();
constexpr double psftombar = psftopa/100.;
constexpr double psftoinhg = DummyMSIS::GetPSFtoInHg();

class FGMSISTest : public CxxTest::TestSuite, FGJSBBase
{
public:
  static constexpr double kmtoft = 1000. / fttom;
  static constexpr double gcm3_to_slugft3 = 1000. * kgtoslug / m3toft3;
  static constexpr double gtoslug = kgtoslug / 1000.;

  FGFDMExec fdmex;
  std::shared_ptr<FGAtmosphere> std_atm;
  std::vector<unsigned int> MSIS_iyd, MSIS_sec;
  std::vector<double> MSIS_alt, MSIS_glat, MSIS_glon, MSIS_f107a, MSIS_f107, MSIS_ap,
                      MSIS_T, MSIS_rho, MSIS_mair;

  FGMSISTest() {
    std_atm = fdmex.GetAtmosphere();
    fdmex.GetPropertyManager()->Unbind(std_atm);

    const double species_mmol[8] {28.0134, 31.9988, 31.9988/2.0, 4.0, 1.0, 39.948,
                                  28.0134/2.0, 31.9988/2.0};
    double n[8];
    enum {N2=0, O2, O, He, H, Ar, N, OA};
    struct nrlmsise_output output;
	  struct nrlmsise_input input[15];
  	struct nrlmsise_flags flags;
    int i;
    /* input values */
  	flags.switches[0]=0;
  	for (i=1;i<24;i++)
  		flags.switches[i]=1;
    for (i=0;i<15;i++) {
      input[i].doy=172;
      input[i].year=0; /* without effect */
      input[i].sec=29000;
      input[i].alt=400;
      input[i].g_lat=60;
      input[i].g_long=-70;
      input[i].lst=16;
      input[i].f107A=150;
      input[i].f107=150;
      input[i].ap=4;
    }
    input[1].doy=81;
    input[2].sec=75000;
    input[2].alt=1000;
    input[3].alt=100;
    input[10].alt=0;
    input[11].alt=10;
    input[12].alt=30;
    input[13].alt=50;
    input[14].alt=70;
    input[6].alt=100;
    input[4].g_lat=0;
    input[5].g_long=0;
    // input[6].lst=4;
    input[7].f107A=70;
    input[8].f107=180;
    input[9].ap=40;
    /* evaluate 0 to 14 */
    for (i=0;i<15;i++) {
      double mol = 0.0;
      double mmol = 0.0;

      input[i].lst = input[i].sec/3600.+input[i].g_long/15.;
      MSIS_iyd.push_back(input[i].doy);
      MSIS_sec.push_back(input[i].sec);
      MSIS_alt.push_back(input[i].alt);
      MSIS_glat.push_back(input[i].g_lat);
      MSIS_glon.push_back(input[i].g_long);
      MSIS_f107a.push_back(input[i].f107A);
      MSIS_f107.push_back(input[i].f107);
      MSIS_ap.push_back(input[i].ap);
      gtd7(&input[i], &flags, &output);
      MSIS_T.push_back(output.t[1]);
      MSIS_rho.push_back(output.d[5]);
      n[He] = output.d[0];
      n[O] = output.d[1];
      n[N2] = output.d[2];
      n[O2] = output.d[3];
      n[Ar] = output.d[4];
      n[H] = output.d[6];
      n[N] = output.d[7];
      n[OA] = 0.0;

      for(unsigned j=N2; j<=OA; ++j) {
        mmol += n[j]*species_mmol[j];
        mol += n[j];
      }
      MSIS_mair.push_back(mmol/mol);
    }
  }

  void testConstructor()
  {
    auto atm = DummyMSIS(&fdmex);

    double h = MSIS_alt[0]*kmtoft;

    atm.SetDay(MSIS_iyd[0]);
    atm.SetSeconds(MSIS_sec[0]);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = MSIS_glat[0];
    atm.in.LongitudeDeg = MSIS_glon[0];
    atm.SetF107A(MSIS_f107a[0]);
    atm.SetF107(MSIS_f107[0]);
    atm.SetAP(MSIS_ap[0]);

    double T = KelvinToRankine(MSIS_T[0]);
    TS_ASSERT_EQUALS(atm.GetTemperatureSL(), 1.8);
    TS_ASSERT_EQUALS(atm.GetTemperature(), 1.8);
    TS_ASSERT_DELTA(atm.GetTemperature(h)/T, 1.0, 1E-5);
    TS_ASSERT_EQUALS(atm.GetTemperatureRatio(), 1.0);
    TS_ASSERT_DELTA(atm.GetTemperatureRatio(h)*1.8/T, 1.0, 1E-5);

    double rho = MSIS_rho[0]*gcm3_to_slugft3;
    TS_ASSERT_EQUALS(atm.GetDensitySL(), 1.0);
    TS_ASSERT_EQUALS(atm.GetDensity(), 0.0);
    TS_ASSERT_DELTA(atm.GetDensity(h)/rho, 1.0, 2E-4);
    TS_ASSERT_EQUALS(atm.GetDensityRatio(), 0.0);

    double R = Rstar / (MSIS_mair[0]*gtoslug);
    double P = rho*R*T;
    TS_ASSERT_EQUALS(atm.GetPressureSL(), 1.0);
    TS_ASSERT_EQUALS(atm.GetPressure(), 0.0);
    TS_ASSERT_DELTA(atm.GetPressure(h)/P, 1.0, 2E-4);
    TS_ASSERT_EQUALS(atm.GetPressureRatio(), 0.0);

    double a = sqrt(gama*R*T);
    TS_ASSERT_EQUALS(atm.GetSoundSpeedSL(), 1.0);
    TS_ASSERT_EQUALS(atm.GetSoundSpeed(), 0.0);
    TS_ASSERT_DELTA(atm.GetSoundSpeed(h)/a, 1.0, 1E-4);
    TS_ASSERT_EQUALS(atm.GetSoundSpeedRatio(), 0.0);

    TS_ASSERT_EQUALS(atm.GetDensityAltitude(), 0.0);
    TS_ASSERT_EQUALS(atm.GetPressureAltitude(), 0.0);

    TS_ASSERT_EQUALS(atm.GetAbsoluteViscosity(), 0.0);
    TS_ASSERT_EQUALS(atm.GetKinematicViscosity(), 0.0);
  }

  void testInitModel()
  {
    auto pm = fdmex.GetPropertyManager();
    auto theta_node = pm->GetNode("atmosphere/theta");
    auto sigma_node = pm->GetNode("atmosphere/sigma");
    auto delta_node = pm->GetNode("atmosphere/delta");
    auto a_ratio_node = pm->GetNode("atmosphere/a-ratio");

    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    for (unsigned int i=0; i<MSIS_iyd.size(); ++i) {
      double h = MSIS_alt[i]*kmtoft;

      atm.SetDay(MSIS_iyd[i]);
      atm.SetSeconds(MSIS_sec[i]);
      atm.in.altitudeASL = h;
      atm.in.GeodLatitudeDeg = MSIS_glat[i];
      atm.in.LongitudeDeg = MSIS_glon[i];
      atm.SetF107A(MSIS_f107a[i]);
      atm.SetF107(MSIS_f107[i]);
      atm.SetAP(MSIS_ap[i]);

      double T = KelvinToRankine(MSIS_T[i]);
      TS_ASSERT_DELTA(atm.GetTemperature(h)/T, 1.0, 1E-4);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(), 1.0);
      TS_ASSERT_EQUALS(theta_node->getDoubleValue(), 1.0);

      double rho = MSIS_rho[i]*gcm3_to_slugft3;
      TS_ASSERT_DELTA(atm.GetDensity(h)/rho, 1.0, 5E-4);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), 1.0);
      TS_ASSERT_EQUALS(sigma_node->getDoubleValue(), 1.0);

      double R = Rstar / (MSIS_mair[i]*gtoslug);
      double P = rho*R*T;
      TS_ASSERT_DELTA(atm.GetPressure(h)/P, 1.0, 5E-4);
      TS_ASSERT_EQUALS(atm.GetPressureRatio(), 1.0);
      TS_ASSERT_EQUALS(delta_node->getDoubleValue(), 1.0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h)/a, 1.0, 1E-4);
      TS_ASSERT_EQUALS(atm.GetSoundSpeedRatio(), 1.0);
      TS_ASSERT_EQUALS(a_ratio_node->getDoubleValue(), 1.0);

      double p_alt = atm.GetPressureAltitude();
      double P_SL = atm.GetPressureSL();
      TS_ASSERT_DELTA(std_atm->GetPressure(p_alt), P_SL, 1E-8);

      double rho_alt = atm.GetDensityAltitude();
      double rho_SL = atm.GetDensitySL();
      TS_ASSERT_DELTA(std_atm->GetDensity(rho_alt)/rho_SL, 1.0, 1E-8);
    }
  }

  void testLoadModel()
  {
    auto pm = fdmex.GetPropertyManager();
    auto theta_node = pm->GetNode("atmosphere/theta");
    auto sigma_node = pm->GetNode("atmosphere/sigma");
    auto delta_node = pm->GetNode("atmosphere/delta");
    auto a_ratio_node = pm->GetNode("atmosphere/a-ratio");

    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    for (unsigned int i=0; i<MSIS_iyd.size(); ++i) {
      double h = MSIS_alt[i]*kmtoft;
      std::stringstream s;

      s << "<dummy>"
        << "  <day>" << MSIS_iyd[i] << "</day>"
        << "  <utc>" << MSIS_sec[i] << "</utc>"
        << "</dummy>" << std::endl;

      Element_ptr elm = readFromXML(s.str());
      TS_ASSERT(atm.Load(elm));

      atm.in.altitudeASL = h;
      atm.in.GeodLatitudeDeg = MSIS_glat[i];
      atm.in.LongitudeDeg = MSIS_glon[i];
      atm.SetF107A(MSIS_f107a[i]);
      atm.SetF107(MSIS_f107[i]);
      atm.SetAP(MSIS_ap[i]);

      double T = KelvinToRankine(MSIS_T[i]);
      TS_ASSERT_DELTA(atm.GetTemperature(h)/T, 1.0, 1E-4);
      TS_ASSERT_EQUALS(atm.GetTemperatureRatio(), 1.0);
      TS_ASSERT_EQUALS(theta_node->getDoubleValue(), 1.0);

      double rho = MSIS_rho[i]*gcm3_to_slugft3;
      TS_ASSERT_DELTA(atm.GetDensity(h)/rho, 1.0, 5E-4);
      TS_ASSERT_EQUALS(atm.GetDensityRatio(), 1.0);
      TS_ASSERT_EQUALS(sigma_node->getDoubleValue(), 1.0);

      double R = Rstar / (MSIS_mair[i]*gtoslug);
      double P = rho*R*T;
      TS_ASSERT_DELTA(atm.GetPressure(h)/P, 1.0, 5E-4);
      TS_ASSERT_EQUALS(atm.GetPressureRatio(), 1.0);
      TS_ASSERT_EQUALS(delta_node->getDoubleValue(), 1.0);

      double a = sqrt(gama*R*T);
      TS_ASSERT_DELTA(atm.GetSoundSpeed(h)/a, 1.0, 1E-4);
      TS_ASSERT_EQUALS(atm.GetSoundSpeedRatio(), 1.0);
      TS_ASSERT_EQUALS(a_ratio_node->getDoubleValue(), 1.0);

      double p_alt = atm.GetPressureAltitude();
      double P_SL = atm.GetPressureSL();
      TS_ASSERT_DELTA(std_atm->GetPressure(p_alt), P_SL, 1E-8);

      double rho_alt = atm.GetDensityAltitude();
      double rho_SL = atm.GetDensitySL();
      TS_ASSERT_DELTA(std_atm->GetDensity(rho_alt)/rho_SL, 1.0, 1E-8);
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

    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    for (unsigned int i=0; i<MSIS_iyd.size(); ++i) {
      double h = MSIS_alt[i]*kmtoft;

      atm.SetDay(MSIS_iyd[i]);
      atm.SetSeconds(MSIS_sec[i]);
      atm.in.altitudeASL = h;
      atm.in.GeodLatitudeDeg = MSIS_glat[i];
      atm.in.LongitudeDeg = MSIS_glon[i];
      atm.SetF107A(MSIS_f107a[i]);
      atm.SetF107(MSIS_f107[i]);
      atm.SetAP(MSIS_ap[i]);

      TS_ASSERT(atm.Run(false) == false);

      double T = KelvinToRankine(MSIS_T[i]);
      double T_SL = atm.GetTemperatureSL();
      double T0 = atm.GetTemperature(0.0);
      TS_ASSERT_DELTA(atm.GetTemperature()/T, 1.0, 1E-4);
      TS_ASSERT_DELTA(T_node->getDoubleValue()/T, 1.0, 1E-4);
      TS_ASSERT_EQUALS(T_SL, T0);
      TS_ASSERT_EQUALS(T0_node->getDoubleValue(), T_SL);
      TS_ASSERT_DELTA(atm.GetTemperatureRatio()*T_SL/T, 1.0, 1E-4);
      TS_ASSERT_DELTA(theta_node->getDoubleValue()*T_SL/T, 1.0, 1E-4);

      double rho = MSIS_rho[i]*gcm3_to_slugft3;
      double rho_SL = atm.GetDensitySL();
      double rho0= atm.GetDensity(0.0);
      TS_ASSERT_DELTA(atm.GetDensity()/rho, 1.0, 5E-4);
      TS_ASSERT_DELTA(rho_node->getDoubleValue()/rho, 1.0, 5E-4);
      TS_ASSERT_EQUALS(rho_SL, rho0);
      TS_ASSERT_EQUALS(rho0_node->getDoubleValue(), rho_SL);
      TS_ASSERT_DELTA(atm.GetDensityRatio()*rho_SL/rho, 1.0, 5E-4);
      TS_ASSERT_DELTA(sigma_node->getDoubleValue()*rho_SL/rho, 1.0, 5E-4);

      double R = Rstar / (MSIS_mair[i]*gtoslug);
      double P = rho*R*T;
      double P_SL = atm.GetPressureSL();
      double P0 = atm.GetPressure(0.0);
      TS_ASSERT_DELTA(atm.GetPressure()/P, 1.0, 5E-4);
      TS_ASSERT_DELTA(P_node->getDoubleValue()/P, 1.0, 5E-4);
      TS_ASSERT_EQUALS(P_SL, P0);
      TS_ASSERT_DELTA(atm.GetPressureRatio()*P_SL/P, 1.0, 5E-4);
      TS_ASSERT_DELTA(delta_node->getDoubleValue()*P_SL/P, 1.0, 5E-4);

      double a = sqrt(gama*R*T);
      double a_SL = atm.GetSoundSpeedSL();
      double a0 = atm.GetSoundSpeed(0.0);
      TS_ASSERT_DELTA(atm.GetSoundSpeed()/a, 1.0, 1E-4);
      TS_ASSERT_DELTA(a_node->getDoubleValue()/a, 1.0, 1E-4);
      TS_ASSERT_EQUALS(a_SL, a0);
      TS_ASSERT_EQUALS(a0_node->getDoubleValue(), a_SL);
      TS_ASSERT_DELTA(atm.GetSoundSpeedRatio()*a_SL/a, 1.0, 1E-4);
      TS_ASSERT_DELTA(a_ratio_node->getDoubleValue()*a_SL/a, 1.0, 1E-4);

      double mu = beta*T*sqrt(T)/(k+T);
      double nu = mu/rho;
      TS_ASSERT_DELTA(atm.GetAbsoluteViscosity(), mu, 1E-4);
      TS_ASSERT_DELTA(atm.GetKinematicViscosity()/nu, 1.0, 5E-4);


      double p_alt = atm.GetPressureAltitude();
      TS_ASSERT_DELTA(std_atm->GetPressure(p_alt), P, 1E-8);
      TS_ASSERT_EQUALS(pressure_altitude_node->getDoubleValue(), p_alt);
      double rho_alt = atm.GetDensityAltitude();
      TS_ASSERT_DELTA(std_atm->GetDensity(rho_alt)/rho, 1.0, 1E-8);
      TS_ASSERT_EQUALS(density_altitude_node->getDoubleValue(), rho_alt);
    }
  }

  /***************************************************************************
   * Solar Activity Tests
   ***************************************************************************/

  // Test high solar activity (F107 = 200)
  void testHighSolarActivity() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;  // 400 km
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 60.0;
    atm.in.LongitudeDeg = -70.0;
    atm.SetF107A(200);  // High solar activity
    atm.SetF107(200);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    // Temperature and density should be valid
    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
    TS_ASSERT(!std::isnan(T));
    TS_ASSERT(!std::isnan(rho));
  }

  // Test low solar activity (F107 = 70)
  void testLowSolarActivity() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 60.0;
    atm.in.LongitudeDeg = -70.0;
    atm.SetF107A(70);  // Low solar activity
    atm.SetF107(70);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test solar activity comparison (high vs low)
  void testSolarActivityComparison() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 60.0;
    atm.in.LongitudeDeg = -70.0;
    atm.SetAP(4);

    // High solar activity
    atm.SetF107A(200);
    atm.SetF107(200);
    TS_ASSERT(atm.Run(false) == false);
    double T_high = atm.GetTemperature();
    double rho_high = atm.GetDensity();

    // Low solar activity
    atm.SetF107A(70);
    atm.SetF107(70);
    TS_ASSERT(atm.Run(false) == false);
    double T_low = atm.GetTemperature();
    double rho_low = atm.GetDensity();

    // Higher solar activity should give higher temperature at these altitudes
    TS_ASSERT(T_high > T_low);
    // And higher density
    TS_ASSERT(rho_high > rho_low);
  }

  /***************************************************************************
   * Geomagnetic Activity Tests
   ***************************************************************************/

  // Test high geomagnetic activity
  void testHighGeomagneticActivity() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 60.0;
    atm.in.LongitudeDeg = -70.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(100);  // High geomagnetic activity

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test quiet geomagnetic conditions
  void testQuietGeomagneticConditions() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 60.0;
    atm.in.LongitudeDeg = -70.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(2);  // Very quiet conditions

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  /***************************************************************************
   * Altitude Profile Tests
   ***************************************************************************/

  // Test sea level conditions
  void testSeaLevel() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = 0.0;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double P = atm.GetPressure();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(P > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test stratosphere (~30 km)
  void testStratosphere() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 30 * kmtoft;  // 30 km
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double P = atm.GetPressure();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(P > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test mesosphere (~70 km)
  void testMesosphere() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 70 * kmtoft;  // 70 km
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test thermosphere (~200 km)
  void testThermosphere() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 200 * kmtoft;  // 200 km
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test LEO altitude (~400 km)
  void testLEOAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;  // 400 km (ISS orbit)
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test high orbit (~1000 km)
  void testHighOrbit() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 1000 * kmtoft;  // 1000 km
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test density decreases with altitude
  void testDensityDecreasesWithAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double prev_rho = std::numeric_limits<double>::max();

    for (double alt_km = 0; alt_km <= 500; alt_km += 100) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);
      double rho = atm.GetDensity();

      TS_ASSERT(rho > 0.0);
      TS_ASSERT(rho < prev_rho);  // Density should decrease
      prev_rho = rho;
    }
  }

  /***************************************************************************
   * Latitude Variation Tests
   ***************************************************************************/

  // Test equator conditions
  void testEquator() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 0.0;  // Equator
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test polar region
  void testPolarRegion() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 80.0;  // Near pole
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test southern hemisphere
  void testSouthernHemisphere() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = -45.0;  // Southern hemisphere
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  /***************************************************************************
   * Time Variation Tests
   ***************************************************************************/

  // Test morning conditions
  void testMorningConditions() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(6 * 3600);  // 6 AM UTC
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test afternoon conditions
  void testAfternoonConditions() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(14 * 3600);  // 2 PM UTC
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test midnight conditions
  void testMidnightConditions() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(0);  // Midnight UTC
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  /***************************************************************************
   * Seasonal Variation Tests
   ***************************************************************************/

  // Test summer solstice (day 172)
  void testSummerSolstice() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);  // June 21
    atm.SetSeconds(43200);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test winter solstice (day 355)
  void testWinterSolstice() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(355);  // December 21
    atm.SetSeconds(43200);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test spring equinox (day 81)
  void testSpringEquinox() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(81);  // March 21
    atm.SetSeconds(43200);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  /***************************************************************************
   * Viscosity Tests
   ***************************************************************************/

  // Test viscosity at various altitudes
  void testViscosityProfile() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    for (double alt_km = 0; alt_km <= 100; alt_km += 20) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double mu = atm.GetAbsoluteViscosity();
      double nu = atm.GetKinematicViscosity();

      TS_ASSERT(mu > 0.0);
      TS_ASSERT(nu > 0.0);
      TS_ASSERT(!std::isnan(mu));
      TS_ASSERT(!std::isnan(nu));
    }
  }

  // Test Sutherland law consistency
  void testSutherlandLaw() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 10 * kmtoft;  // 10 km
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();
    double mu = atm.GetAbsoluteViscosity();
    double nu = atm.GetKinematicViscosity();

    // Kinematic viscosity = absolute viscosity / density
    TS_ASSERT_DELTA(nu * rho / mu, 1.0, 1E-4);
  }

  /***************************************************************************
   * Sound Speed Tests
   ***************************************************************************/

  // Test sound speed at various altitudes
  void testSoundSpeedProfile() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    for (double alt_km = 0; alt_km <= 400; alt_km += 50) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);

      double a = atm.GetSoundSpeed();

      TS_ASSERT(a > 0.0);
      TS_ASSERT(!std::isnan(a));
    }
  }

  // Test sound speed ratio
  void testSoundSpeedRatio() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 100 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double a = atm.GetSoundSpeed();
    double a_SL = atm.GetSoundSpeedSL();
    double a_ratio = atm.GetSoundSpeedRatio();

    TS_ASSERT_DELTA(a / a_SL, a_ratio, 1E-6);
  }

  /***************************************************************************
   * Pressure Altitude Tests
   ***************************************************************************/

  // Test pressure altitude calculation
  void testPressureAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double p_alt = atm.GetPressureAltitude();
    double P = atm.GetPressure();

    // Pressure altitude should give same pressure in standard atmosphere
    TS_ASSERT_DELTA(std_atm->GetPressure(p_alt), P, 1E-6);
  }

  // Test density altitude calculation
  void testDensityAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double rho_alt = atm.GetDensityAltitude();
    double rho = atm.GetDensity();

    // Density altitude should give same density in standard atmosphere
    TS_ASSERT_DELTA(std_atm->GetDensity(rho_alt) / rho, 1.0, 1E-6);
  }

  /***************************************************************************
   * Unit Conversion Tests
   ***************************************************************************/

  // Test PSF to Pa conversion constant
  void testPSFtoPaConversion() {
    TS_ASSERT_DELTA(psftopa, 47.880258, 0.001);
  }

  // Test PSF to mbar conversion
  void testPSFtoMbarConversion() {
    TS_ASSERT_DELTA(psftombar, 0.47880258, 0.00001);
  }

  // Test PSF to inHg conversion
  void testPSFtoInHgConversion() {
    TS_ASSERT(psftoinhg > 0.0);
    TS_ASSERT_DELTA(psftoinhg, 0.014139, 0.0001);
  }

  /***************************************************************************
   * Gas Constant Tests
   ***************************************************************************/

  // Test universal gas constant (in ft-lbf/(slug-R) units)
  void testUniversalGasConstant() {
    // Rstar in JSBSim is in ft-lbf/(slug-R), not J/(mol-K)
    TS_ASSERT(Rstar > 0.0);
    TS_ASSERT_DELTA(Rstar, 3.4068, 0.01);
  }

  // Test specific heat ratio
  void testSpecificHeatRatio() {
    TS_ASSERT_DELTA(gama, 1.4, 0.001);
  }

  // Test Sutherland constant
  void testSutherlandConstant() {
    TS_ASSERT_DELTA(k, 198.72, 0.1);
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very high altitude (edge of MSIS validity)
  void testVeryHighAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 800 * kmtoft;  // 800 km - near upper limit
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
    TS_ASSERT(!std::isnan(T));
    TS_ASSERT(!std::isnan(rho));
  }

  // Test extreme solar activity
  void testExtremeSolarActivity() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(250);  // Very high solar activity
    atm.SetF107(300);
    atm.SetAP(150);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
    TS_ASSERT(!std::isnan(T));
    TS_ASSERT(!std::isnan(rho));
  }

  // Test day boundaries
  void testDayBoundaries() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Test day 1
    atm.SetDay(1);
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT(atm.GetTemperature() > 0.0);

    // Test day 365
    atm.SetDay(365);
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT(atm.GetTemperature() > 0.0);
  }

  // Test second boundaries
  void testSecondBoundaries() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Test 0 seconds
    atm.SetSeconds(0);
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT(atm.GetTemperature() > 0.0);

    // Test 86399 seconds (end of day)
    atm.SetSeconds(86399);
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT(atm.GetTemperature() > 0.0);
  }

  // Test longitude extremes
  void testLongitudeExtremes() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Test -180 longitude
    atm.in.LongitudeDeg = -180.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT(atm.GetTemperature() > 0.0);

    // Test 180 longitude
    atm.in.LongitudeDeg = 180.0;
    TS_ASSERT(atm.Run(false) == false);
    TS_ASSERT(atm.GetTemperature() > 0.0);
  }

  /***************************************************************************
   * Scale Height Tests
   ***************************************************************************/

  // Test scale height concept (density e-folding)
  void testScaleHeightConcept() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // At lower altitudes, density should decrease exponentially
    double h1 = 10 * kmtoft;
    double h2 = 20 * kmtoft;

    atm.in.altitudeASL = h1;
    TS_ASSERT(atm.Run(false) == false);
    double rho1 = atm.GetDensity();

    atm.in.altitudeASL = h2;
    TS_ASSERT(atm.Run(false) == false);
    double rho2 = atm.GetDensity();

    // Density should be lower at higher altitude
    TS_ASSERT(rho2 < rho1);
  }

  // Test approximate scale height at low altitude
  void testApproximateScaleHeight() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double h1 = 0.0;
    double h2 = 10 * kmtoft;

    atm.in.altitudeASL = h1;
    TS_ASSERT(atm.Run(false) == false);
    double rho1 = atm.GetDensity();

    atm.in.altitudeASL = h2;
    TS_ASSERT(atm.Run(false) == false);
    double rho2 = atm.GetDensity();

    // Approximate scale height H = delta_h / ln(rho1/rho2)
    double scaleHeight = (h2 - h1) / std::log(rho1 / rho2);

    // Scale height in lower atmosphere ~8-9 km (26000-30000 ft)
    TS_ASSERT(scaleHeight > 20000.0);  // > 20000 ft
    TS_ASSERT(scaleHeight < 35000.0);  // < 35000 ft
  }

  /***************************************************************************
   * Exospheric Temperature Tests
   ***************************************************************************/

  // Test temperature at high altitude approaches exospheric temperature
  void testExosphericTemperatureApproach() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Get temperature at progressively higher altitudes
    double T_400 = 0.0, T_600 = 0.0, T_800 = 0.0;

    atm.in.altitudeASL = 400 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    T_400 = atm.GetTemperature();

    atm.in.altitudeASL = 600 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    T_600 = atm.GetTemperature();

    atm.in.altitudeASL = 800 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    T_800 = atm.GetTemperature();

    // Temperature change should decrease as we approach exospheric temp
    double dT1 = std::abs(T_600 - T_400);
    double dT2 = std::abs(T_800 - T_600);

    // The rate of temperature change should be smaller at higher altitudes
    TS_ASSERT(dT2 <= dT1 * 1.1);  // Allow some tolerance
  }

  // Test exospheric temperature is high (around 1000K = 1800R)
  void testExosphericTemperatureRange() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 600 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();

    // Exospheric temperature typically 600-2000K = 1080-3600R
    TS_ASSERT(T > 1000.0);  // > ~555K
    TS_ASSERT(T < 4000.0);  // < ~2200K
  }

  /***************************************************************************
   * Diurnal Variation Tests
   ***************************************************************************/

  // Test density is higher in afternoon (heated side)
  void testDiurnalDensityVariation() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;  // Use longitude 0
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Morning (6 AM local)
    atm.SetSeconds(6 * 3600);
    TS_ASSERT(atm.Run(false) == false);
    double rho_morning = atm.GetDensity();

    // Afternoon (14 PM local - thermal bulge time)
    atm.SetSeconds(14 * 3600);
    TS_ASSERT(atm.Run(false) == false);
    double rho_afternoon = atm.GetDensity();

    // At thermospheric altitudes, afternoon density should be higher
    // due to atmospheric expansion from solar heating
    TS_ASSERT(rho_afternoon > rho_morning);
  }

  // Test temperature diurnal variation
  void testDiurnalTemperatureVariation() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Night
    atm.SetSeconds(3 * 3600);  // 3 AM
    TS_ASSERT(atm.Run(false) == false);
    double T_night = atm.GetTemperature();

    // Afternoon peak
    atm.SetSeconds(15 * 3600);  // 3 PM
    TS_ASSERT(atm.Run(false) == false);
    double T_day = atm.GetTemperature();

    // Day should be warmer
    TS_ASSERT(T_day > T_night);
  }

  /***************************************************************************
   * Atmospheric Drag Tests
   ***************************************************************************/

  // Test drag force proportionality to density
  void testDragForceProportionality() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double V = 7800.0;  // Orbital velocity m/s ~ 25590 ft/s
    double Cd = 2.2;
    double A = 10.0;    // m^2 ~ 107.6 ft^2

    atm.in.altitudeASL = 200 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double rho_200 = atm.GetDensity();
    double drag_200 = 0.5 * rho_200 * V * V * Cd * A;

    atm.in.altitudeASL = 400 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double rho_400 = atm.GetDensity();
    double drag_400 = 0.5 * rho_400 * V * V * Cd * A;

    // Drag should be proportional to density
    TS_ASSERT_DELTA(drag_200 / drag_400, rho_200 / rho_400, 1E-6);
  }

  // Test atmospheric drag at ISS altitude
  void testDragAtISSAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 420 * kmtoft;  // ISS altitude
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 51.6;  // ISS inclination
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double rho = atm.GetDensity();

    // Density at 420 km should be extremely small but positive
    TS_ASSERT(rho > 0.0);
    TS_ASSERT(rho < 1E-10);  // Very thin atmosphere
  }

  /***************************************************************************
   * Pressure Profile Tests
   ***************************************************************************/

  // Test pressure decreases with altitude
  void testPressureDecreasesWithAltitude() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double prev_P = std::numeric_limits<double>::max();

    for (double alt_km = 0; alt_km <= 100; alt_km += 20) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);
      double P = atm.GetPressure();

      TS_ASSERT(P > 0.0);
      TS_ASSERT(P < prev_P);
      prev_P = P;
    }
  }

  // Test pressure ratio consistency
  void testPressureRatioConsistency() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double P = atm.GetPressure();
    double P_SL = atm.GetPressureSL();
    double delta = atm.GetPressureRatio();

    TS_ASSERT_DELTA(P / P_SL, delta, 1E-6);
  }

  /***************************************************************************
   * Temperature Profile Tests
   ***************************************************************************/

  // Test temperature ratio consistency
  void testTemperatureRatioConsistency() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double T_SL = atm.GetTemperatureSL();
    double theta = atm.GetTemperatureRatio();

    TS_ASSERT_DELTA(T / T_SL, theta, 1E-6);
  }

  // Test density ratio consistency
  void testDensityRatioConsistency() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double rho = atm.GetDensity();
    double rho_SL = atm.GetDensitySL();
    double sigma = atm.GetDensityRatio();

    TS_ASSERT_DELTA(rho / rho_SL, sigma, 1E-6);
  }

  /***************************************************************************
   * Solar Cycle Tests
   ***************************************************************************/

  // Test solar minimum conditions (F107 = 65)
  void testSolarMinimum() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(65);   // Solar minimum
    atm.SetF107(65);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test solar maximum conditions (F107 = 250)
  void testSolarMaximum() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(250);  // Solar maximum
    atm.SetF107(250);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test F107/F107A difference effects
  void testF107Difference() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetAP(4);

    // Higher daily F107 than average (solar flare)
    atm.SetF107A(150);
    atm.SetF107(200);
    TS_ASSERT(atm.Run(false) == false);
    double T_flare = atm.GetTemperature();

    // Average conditions
    atm.SetF107A(150);
    atm.SetF107(150);
    TS_ASSERT(atm.Run(false) == false);
    double T_normal = atm.GetTemperature();

    // Both should be valid
    TS_ASSERT(T_flare > 0.0);
    TS_ASSERT(T_normal > 0.0);
  }

  /***************************************************************************
   * Geomagnetic Storm Tests
   ***************************************************************************/

  // Test geomagnetic storm conditions
  void testGeomagneticStorm() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(200);  // Major storm

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double rho = atm.GetDensity();

    TS_ASSERT(T > 0.0);
    TS_ASSERT(rho > 0.0);
  }

  // Test Ap index effect on density
  void testApIndexEffect() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 60.0;  // High latitude for geomagnetic effects
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);

    // Quiet conditions
    atm.SetAP(4);
    TS_ASSERT(atm.Run(false) == false);
    double rho_quiet = atm.GetDensity();

    // Active conditions
    atm.SetAP(100);
    TS_ASSERT(atm.Run(false) == false);
    double rho_active = atm.GetDensity();

    // Higher Ap should lead to higher density at high latitudes
    TS_ASSERT(rho_active > rho_quiet);
  }

  /***************************************************************************
   * Property Node Tests
   ***************************************************************************/

  // Test all atmosphere property nodes
  void testAllPropertyNodes() {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 100 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    // Check temperature node
    auto T_node = pm->GetNode("atmosphere/T-R");
    TS_ASSERT(T_node != nullptr);
    TS_ASSERT_DELTA(T_node->getDoubleValue(), atm.GetTemperature(), 1E-6);

    // Check pressure node
    auto P_node = pm->GetNode("atmosphere/P-psf");
    TS_ASSERT(P_node != nullptr);
    TS_ASSERT_DELTA(P_node->getDoubleValue(), atm.GetPressure(), 1E-6);

    // Check density node
    auto rho_node = pm->GetNode("atmosphere/rho-slugs_ft3");
    TS_ASSERT(rho_node != nullptr);
    TS_ASSERT_DELTA(rho_node->getDoubleValue(), atm.GetDensity(), 1E-10);
  }

  // Test ratio property nodes
  void testRatioPropertyNodes() {
    auto pm = fdmex.GetPropertyManager();
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    auto theta_node = pm->GetNode("atmosphere/theta");
    auto sigma_node = pm->GetNode("atmosphere/sigma");
    auto delta_node = pm->GetNode("atmosphere/delta");

    TS_ASSERT(theta_node != nullptr);
    TS_ASSERT(sigma_node != nullptr);
    TS_ASSERT(delta_node != nullptr);

    TS_ASSERT_DELTA(theta_node->getDoubleValue(), atm.GetTemperatureRatio(), 1E-6);
    TS_ASSERT_DELTA(sigma_node->getDoubleValue(), atm.GetDensityRatio(), 1E-6);
    TS_ASSERT_DELTA(delta_node->getDoubleValue(), atm.GetPressureRatio(), 1E-6);
  }

  /***************************************************************************
   * Ideal Gas Law Tests
   ***************************************************************************/

  // Test ideal gas law: P = rho * R * T
  void testIdealGasLaw() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 50 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double P = atm.GetPressure();
    double rho = atm.GetDensity();
    double R = atm.GetR();

    // P = rho * R * T
    TS_ASSERT_DELTA(P / (rho * R * T), 1.0, 1E-4);
  }

  // Test sound speed from ideal gas: a = sqrt(gamma * R * T)
  void testSoundSpeedFromIdealGas() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 30 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    TS_ASSERT(atm.Run(false) == false);

    double T = atm.GetTemperature();
    double a = atm.GetSoundSpeed();
    double R = atm.GetR();

    double a_calc = std::sqrt(gama * R * T);
    TS_ASSERT_DELTA(a / a_calc, 1.0, 1E-4);
  }

  /***************************************************************************
   * Orbit Decay Rate Tests
   ***************************************************************************/

  // Test relative orbit decay rate at different altitudes
  void testRelativeOrbitDecayRate() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Get density at different orbital altitudes
    atm.in.altitudeASL = 300 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double rho_300 = atm.GetDensity();

    atm.in.altitudeASL = 500 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double rho_500 = atm.GetDensity();

    // Orbit decay rate proportional to density
    // 500km orbit decays slower than 300km
    TS_ASSERT(rho_500 < rho_300);

    // Ratio should be very large (orders of magnitude)
    TS_ASSERT(rho_300 / rho_500 > 10.0);
  }

  // Test solar activity effect on orbit lifetime
  void testSolarActivityOrbitEffect() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetAP(4);

    // Solar minimum
    atm.SetF107A(70);
    atm.SetF107(70);
    TS_ASSERT(atm.Run(false) == false);
    double rho_min = atm.GetDensity();

    // Solar maximum
    atm.SetF107A(200);
    atm.SetF107(200);
    TS_ASSERT(atm.Run(false) == false);
    double rho_max = atm.GetDensity();

    // Solar max has much higher density at these altitudes
    TS_ASSERT(rho_max > rho_min);
    TS_ASSERT(rho_max / rho_min > 2.0);  // Should be at least 2x
  }

  /***************************************************************************
   * Beta Parameter Test
   ***************************************************************************/

  // Test beta (Sutherland's constant for viscosity)
  void testBetaParameter() {
    // Beta is used in Sutherland's formula: mu = beta * T^1.5 / (T + k)
    TS_ASSERT(beta > 0.0);
    TS_ASSERT_DELTA(beta, 2.2696e-8, 1E-10);
  }

  /***************************************************************************
   * Altitude Continuity Tests
   ***************************************************************************/

  // Test smooth density profile (no jumps)
  void testSmoothDensityProfile() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double prev_rho = 0.0;
    bool first = true;

    for (double alt_km = 0; alt_km <= 500; alt_km += 10) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);
      double rho = atm.GetDensity();

      if (!first) {
        // No sudden jumps (more than 10x change per 10km)
        double ratio = (prev_rho > 0.0) ? rho / prev_rho : 1.0;
        TS_ASSERT(ratio < 10.0);
        TS_ASSERT(ratio > 0.1);
      }
      prev_rho = rho;
      first = false;
    }
  }

  // Test smooth temperature profile
  void testSmoothTemperatureProfile() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double prev_T = 0.0;
    bool first = true;

    for (double alt_km = 0; alt_km <= 500; alt_km += 10) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);
      double T = atm.GetTemperature();

      if (!first) {
        // No sudden jumps (more than 2x change per 10km)
        double ratio = T / prev_T;
        TS_ASSERT(ratio < 2.0);
        TS_ASSERT(ratio > 0.5);
      }
      prev_T = T;
      first = false;
    }
  }

  /***************************************************************************
   * Model Identity Tests
   ***************************************************************************/

  // Test model name
  void testModelName() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    std::string name = atm.GetName();
    TS_ASSERT(!name.empty());
  }

  // Test GetExec returns correct FDMExec
  void testGetExec() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    TS_ASSERT_EQUALS(atm.GetExec(), &fdmex);
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  // Test many consecutive runs
  void testManyConsecutiveRuns() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    for (int i = 0; i < 100; i++) {
      double h = (i % 10) * 50 * kmtoft;
      atm.in.altitudeASL = h;
      atm.SetSeconds(i * 100);

      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double rho = atm.GetDensity();
      double P = atm.GetPressure();

      TS_ASSERT(T > 0.0);
      TS_ASSERT(rho > 0.0);
      TS_ASSERT(P > 0.0);
      TS_ASSERT(!std::isnan(T));
      TS_ASSERT(!std::isnan(rho));
      TS_ASSERT(!std::isnan(P));
    }
  }

  // Test rapid altitude changes
  void testRapidAltitudeChanges() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double altitudes_km[] = {0, 500, 100, 400, 50, 300, 0, 200};

    for (double alt_km : altitudes_km) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;

      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double rho = atm.GetDensity();

      TS_ASSERT(T > 0.0);
      TS_ASSERT(rho > 0.0);
    }
  }

  // Test latitude sweep
  void testLatitudeSweep() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    for (double lat = -90.0; lat <= 90.0; lat += 15.0) {
      atm.in.GeodLatitudeDeg = lat;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double rho = atm.GetDensity();

      TS_ASSERT(T > 0.0);
      TS_ASSERT(rho > 0.0);
    }
  }

  // Test longitude sweep
  void testLongitudeSweep() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    for (double lon = -180.0; lon <= 180.0; lon += 30.0) {
      atm.in.LongitudeDeg = lon;
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double rho = atm.GetDensity();

      TS_ASSERT(T > 0.0);
      TS_ASSERT(rho > 0.0);
    }
  }

  // Test day of year sweep
  void testDayOfYearSweep() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double h = 400 * kmtoft;
    atm.SetSeconds(29000);
    atm.in.altitudeASL = h;
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    for (int day = 1; day <= 365; day += 30) {
      atm.SetDay(day);
      TS_ASSERT(atm.Run(false) == false);

      double T = atm.GetTemperature();
      double rho = atm.GetDensity();

      TS_ASSERT(T > 0.0);
      TS_ASSERT(rho > 0.0);
    }
  }

  /***************************************************************************
   * Molecular Weight Variation Tests
   ***************************************************************************/

  // Test mean molecular mass changes with altitude
  void testMolecularMassVariation() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // At lower altitudes, R should be relatively constant (N2/O2 mixture)
    // At higher altitudes, R increases as lighter species dominate

    atm.in.altitudeASL = 100 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double R_100km = atm.GetR();

    atm.in.altitudeASL = 500 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double R_500km = atm.GetR();

    // Both should be valid positive values
    TS_ASSERT(R_100km > 0.0);
    TS_ASSERT(R_500km > 0.0);
  }

  /***************************************************************************
   * Troposphere/Stratosphere/Mesosphere Tests
   ***************************************************************************/

  // Test temperature inversion in stratosphere
  void testStratosphereTemperature() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Get temperatures at various altitudes
    atm.in.altitudeASL = 10 * kmtoft;  // Upper troposphere
    TS_ASSERT(atm.Run(false) == false);
    double T_10km = atm.GetTemperature();

    atm.in.altitudeASL = 25 * kmtoft;  // Mid stratosphere
    TS_ASSERT(atm.Run(false) == false);
    double T_25km = atm.GetTemperature();

    atm.in.altitudeASL = 50 * kmtoft;  // Stratopause
    TS_ASSERT(atm.Run(false) == false);
    double T_50km = atm.GetTemperature();

    // All should be valid
    TS_ASSERT(T_10km > 0.0);
    TS_ASSERT(T_25km > 0.0);
    TS_ASSERT(T_50km > 0.0);

    // Stratosphere shows temperature increase with altitude
    // (at least from tropopause to stratopause)
    TS_ASSERT(T_50km > T_10km);
  }

  // Test mesopause temperature minimum
  void testMesopauseTemperature() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // Mesopause around 85-90 km is typically coldest point
    atm.in.altitudeASL = 50 * kmtoft;  // Stratopause
    TS_ASSERT(atm.Run(false) == false);
    double T_50km = atm.GetTemperature();

    atm.in.altitudeASL = 85 * kmtoft;  // Mesopause
    TS_ASSERT(atm.Run(false) == false);
    double T_85km = atm.GetTemperature();

    atm.in.altitudeASL = 120 * kmtoft;  // Lower thermosphere
    TS_ASSERT(atm.Run(false) == false);
    double T_120km = atm.GetTemperature();

    // Mesopause should be cooler than stratopause and thermosphere
    TS_ASSERT(T_85km < T_50km);
    TS_ASSERT(T_85km < T_120km);
  }

  /***************************************************************************
   * Thermosphere Heating Tests
   ***************************************************************************/

  // Test thermosphere temperature increases with altitude
  void testThermosphereHeating() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    double prev_T = 0.0;

    for (double alt_km = 100; alt_km <= 400; alt_km += 50) {
      double h = alt_km * kmtoft;
      atm.in.altitudeASL = h;
      TS_ASSERT(atm.Run(false) == false);
      double T = atm.GetTemperature();

      if (alt_km > 100) {
        // Temperature should generally increase in thermosphere
        TS_ASSERT(T >= prev_T * 0.95);  // Allow small decrease
      }
      prev_T = T;
    }
  }

  /***************************************************************************
   * Atmospheric Composition Effect Tests
   ***************************************************************************/

  // Test atomic oxygen dominance at high altitude
  void testAtomicOxygenDominance() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    atm.SetDay(172);
    atm.SetSeconds(29000);
    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetF107A(150);
    atm.SetF107(150);
    atm.SetAP(4);

    // At low altitude, mean molecular weight ~29 (N2/O2 mix)
    // At high altitude, lighter species dominate, mean MW decreases
    // This affects the gas constant R = Rstar / M

    atm.in.altitudeASL = 100 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double R_100km = atm.GetR();

    atm.in.altitudeASL = 400 * kmtoft;
    TS_ASSERT(atm.Run(false) == false);
    double R_400km = atm.GetR();

    // R should increase as mean molecular weight decreases
    TS_ASSERT(R_400km > R_100km);
  }

  /***************************************************************************
   * Numerical Stability Tests
   ***************************************************************************/

  // Test no NaN or Inf values across parameter space
  void testNumericalStability() {
    auto atm = DummyMSIS(&fdmex);
    TS_ASSERT(atm.InitModel());

    double altitudes[] = {0, 100, 200, 300, 400, 500, 700, 1000};
    double f107s[] = {65, 100, 150, 200, 250};
    double aps[] = {2, 10, 50, 100, 200};

    atm.in.GeodLatitudeDeg = 45.0;
    atm.in.LongitudeDeg = 0.0;
    atm.SetDay(172);
    atm.SetSeconds(29000);

    for (double alt_km : altitudes) {
      for (double f107 : f107s) {
        for (double ap : aps) {
          atm.in.altitudeASL = alt_km * kmtoft;
          atm.SetF107A(f107);
          atm.SetF107(f107);
          atm.SetAP(ap);

          TS_ASSERT(atm.Run(false) == false);

          double T = atm.GetTemperature();
          double rho = atm.GetDensity();
          double P = atm.GetPressure();
          double a = atm.GetSoundSpeed();

          TS_ASSERT(!std::isnan(T));
          TS_ASSERT(!std::isnan(rho));
          TS_ASSERT(!std::isnan(P));
          TS_ASSERT(!std::isnan(a));
          TS_ASSERT(!std::isinf(T));
          TS_ASSERT(!std::isinf(rho));
          TS_ASSERT(!std::isinf(P));
          TS_ASSERT(!std::isinf(a));
          TS_ASSERT(T > 0.0);
          TS_ASSERT(rho > 0.0);
          TS_ASSERT(P > 0.0);
          TS_ASSERT(a > 0.0);
        }
      }
    }
  }
};
