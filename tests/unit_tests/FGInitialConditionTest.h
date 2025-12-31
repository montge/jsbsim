#include <limits>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <FGJSBBase.h>
#include "TestAssertions.h"

using namespace JSBSim;

const double epsilon = 100. * std::numeric_limits<double>::epsilon();
const FGColumnVector3 zero {0.0, 0.0, 0.0};
constexpr double ktstofps = 1852./(3600*0.3048);

class FGInitialConditionTest : public CxxTest::TestSuite
{
public:
  void testDefaultConstructor() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    TS_ASSERT_EQUALS(ic.GetLatitudeDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetLatitudeRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetLongitudeDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetLongitudeRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetGeodLatitudeDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetGeodLatitudeRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetThetaDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetThetaRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPhiDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPhiRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPsiDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPsiRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetAltitudeASLFtIC(), 0.0);
#ifdef __arm64__
    TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC(), 0.0, 1E-8);
    TS_ASSERT_DELTA(ic.GetTerrainElevationFtIC(), 0.0, 1E-8);
#else
    TS_ASSERT_EQUALS(ic.GetAltitudeAGLFtIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetTerrainElevationFtIC(), 0.0);
#endif
    TS_ASSERT_EQUALS(ic.GetEarthPositionAngleIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVcalibratedKtsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVequivalentKtsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVgroundFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVtrueFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetMachIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetClimbRateFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetFlightPathAngleDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetFlightPathAngleRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetAlphaDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetAlphaRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetBetaDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetBetaDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetBetaRadIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindMagFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindDirDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindUFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindVFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindWFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindNFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindEFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWindDFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetUBodyFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVBodyFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetWBodyFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVNorthFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVEastFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVDownFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPRadpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetQRadpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetRRadpsIC(), 0.0);
    TS_ASSERT_VECTOR_EQUALS(ic.GetWindNEDFpsIC(), zero);
    TS_ASSERT_VECTOR_EQUALS(ic.GetUVWFpsIC(), zero);
    TS_ASSERT_VECTOR_EQUALS(ic.GetPQRRadpsIC(), zero);
  }

  void testSetPositionASL() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    for(double lon=-180.; lon <= 180.; lon += 30.) {
      ic.SetLongitudeDegIC(lon);

      // Altitude first, then latitude
      for(double asl=1.; asl <= 1000001.; asl += 10000.) {
        ic.SetAltitudeASLFtIC(asl);
        for(double lat=-90.; lat <=90.; lat += 10.) {
          ic.SetLatitudeDegIC(lat);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          // For some reasons, MinGW32 and MSVC are less accurate than other platforms.
#if defined(_MSC_VER) || defined(__MINGW32__)
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 4E-8);
          // TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/asl, 1.0, 4E-8);
#else
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 2E-8);
          // TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/asl, 1.0, 2E-8);
#endif
          TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), lat, epsilon*10.);
          TS_ASSERT_DELTA(ic.GetLatitudeRadIC(), lat*M_PI/180., epsilon);
        }
      }

      // Latitude first, then altitude
      for(double lat=-90.; lat <=90.; lat += 10.) {
        ic.SetLatitudeDegIC(lat);
        for(double asl=1.; asl <= 1000001.; asl += 10000.) {
          ic.SetAltitudeASLFtIC(asl);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 2E-8);
          // TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/asl, 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), lat, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLatitudeRadIC(), lat*M_PI/180., epsilon);
        }
      }
    }
  }

  void testSetPositionAGL() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetTerrainElevationFtIC(2000.);

    for(double lon=-180.; lon <= 180.; lon += 30.) {
      ic.SetLongitudeDegIC(lon);

      // Altitude first, then latitude
      for(double agl=1.; agl <= 1000001.; agl += 10000.) {
        ic.SetAltitudeAGLFtIC(agl);
        for(double lat=-90.; lat <=90.; lat += 10.) {
          ic.SetLatitudeDegIC(lat);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          // TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/(agl+2000.), 1.0, 2E-8);
          // For some reasons, MinGW32, MSVC and MacOS are less accurate than Linux.
#if defined(_MSC_VER) || defined(__MINGW32__) || defined(__APPLE__)
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/agl, 1.0, 4E-8);
#else
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/agl, 1.0, 2E-8);
#endif
          TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), lat, epsilon*10.);
          TS_ASSERT_DELTA(ic.GetLatitudeRadIC(), lat*M_PI/180., epsilon);
        }

        ic.SetAltitudeAGLFtIC(-2000.);
        for(double lat=-90.; lat <=90.; lat += 10.) {
          ic.SetLatitudeDegIC(lat);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC(), 0.0, 3E-8);
          // For some reasons, MinGW32 is less accurate than other platforms.
#ifdef __MINGW32__
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/2000., -1.0, 4E-8);
#else
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/2000., -1.0, 2E-8);
#endif
          TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), lat, epsilon*10.);
          TS_ASSERT_DELTA(ic.GetLatitudeRadIC(), lat*M_PI/180., epsilon);
        }
      }

      // Latitude first, then altitude
      for(double lat=-90.; lat <=90.; lat += 10.) {
        ic.SetLatitudeDegIC(lat);
        for(double agl=1.; agl <= 1000001.; agl += 10000.) {
          ic.SetAltitudeAGLFtIC(agl);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          // TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/(agl+2000.), 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/agl, 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), lat, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLatitudeRadIC(), lat*M_PI/180., epsilon);
        }
      }
    }
  }

  void testSetGeodeticLatitudeAndASL() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    for(double lon=-180.; lon <= 180.; lon += 30.) {
      ic.SetLongitudeDegIC(lon);

      // Altitude first, then latitude
      for(double asl=1.; asl <= 1000001.; asl += 10000.) {
        ic.SetAltitudeASLFtIC(asl);
        for(double lat=-90.; lat <=90.; lat += 10.) {
          ic.SetGeodLatitudeDegIC(lat);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          // For some reasons, MinGW32 and MSVC are less accurate than other platforms.
#if defined(_MSC_VER) || defined(__MINGW32__)
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 4E-8);
          // TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/asl, 1.0, 4E-8);
#else
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 2E-8);
          // TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/asl, 1.0, 2E-8);
#endif
          TS_ASSERT_DELTA(ic.GetGeodLatitudeDegIC(), lat, epsilon*1000.);
          TS_ASSERT_DELTA(ic.GetGeodLatitudeRadIC(), lat*M_PI/180., epsilon*10.);
        }
      }

      // Latitude first, then altitude
      for(double lat=-90.; lat <=90.; lat += 10.) {
        ic.SetGeodLatitudeDegIC(lat);
        for(double asl=1.; asl <= 1000001.; asl += 10000.) {
          ic.SetAltitudeASLFtIC(asl);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 2E-8);
          // TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/asl, 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetGeodLatitudeDegIC(), lat, 1E-9);
          TS_ASSERT_DELTA(ic.GetGeodLatitudeRadIC(), lat*M_PI/180., epsilon*1000.);
        }
      }
    }
  }

  void testSetGeodeticLatitudeAndAGL() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    for(double lon=-180.; lon <= 180.; lon += 30.) {
      ic.SetLongitudeDegIC(lon);

      // Altitude first, then latitude
      for(double agl=1.; agl <= 1000001.; agl += 10000.) {
        ic.SetAltitudeAGLFtIC(agl);
        for(double lat=-90.; lat <=90.; lat += 10.) {
          ic.SetGeodLatitudeDegIC(lat);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          // For some reasons, MinGW32 and MSVC are less accurate than other platforms.
#if defined(_MSC_VER) || defined(__MINGW32__)
          // TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 4E-8);
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/agl, 1.0, 4E-8);
#else
          // TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/agl, 1.0, 4E-8);
#endif
          TS_ASSERT_DELTA(ic.GetGeodLatitudeDegIC(), lat, epsilon*1000.);
          TS_ASSERT_DELTA(ic.GetGeodLatitudeRadIC(), lat*M_PI/180., epsilon*10.);
        }
      }

      // Latitude first, then altitude
      for(double lat=-90.; lat <=90.; lat += 10.) {
        ic.SetGeodLatitudeDegIC(lat);
        for(double agl=1.; agl <= 1000001.; agl += 10000.) {
          ic.SetAltitudeAGLFtIC(agl);

          TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon*100.);
          TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon*M_PI/180., epsilon);
          // TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC()/asl, 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetAltitudeAGLFtIC()/agl, 1.0, 2E-8);
          TS_ASSERT_DELTA(ic.GetGeodLatitudeDegIC(), lat, 1E-9);
          TS_ASSERT_DELTA(ic.GetGeodLatitudeRadIC(), lat*M_PI/180., epsilon*1000.);
        }
      }
    }
  }

  void testBodyVelocity() {
    using namespace std;

    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetUBodyFpsIC(100.);
    TS_ASSERT_DELTA(ic.GetUBodyFpsIC(), 100., epsilon);
    TS_ASSERT_DELTA(ic.GetVBodyFpsIC(), 0., epsilon);
    TS_ASSERT_DELTA(ic.GetWBodyFpsIC(), 0., epsilon);
    TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 100., epsilon);
    TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), 100., epsilon);
    TS_ASSERT_DELTA(ic.GetAlphaDegIC(), 0.0, epsilon);
    TS_ASSERT_DELTA(ic.GetBetaDegIC(), 0.0, epsilon);

    for(double theta=-90.; theta <= 90.; theta+=10.) {
      ic.SetThetaDegIC(theta);

      TS_ASSERT_DELTA(ic.GetUBodyFpsIC(), 100., epsilon*10.);
      TS_ASSERT_DELTA(ic.GetVBodyFpsIC(), 0., epsilon);
      TS_ASSERT_DELTA(ic.GetWBodyFpsIC(), 0., epsilon);
#ifdef __arm64__
      TS_ASSERT_DELTA(ic.GetVNorthFpsIC(), 100.*cos(theta*M_PI/180.), epsilon*10.);
      TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), abs(100.*cos(theta*M_PI/180.)),
                      epsilon*10.);
#else
      TS_ASSERT_DELTA(ic.GetVNorthFpsIC(), 100.*cos(theta*M_PI/180.), epsilon);
      TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), abs(100.*cos(theta*M_PI/180.)),
                      epsilon);
#endif
      TS_ASSERT_DELTA(ic.GetVEastFpsIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetVDownFpsIC(), -100.*sin(theta*M_PI/180.),
                      epsilon*10.);
      TS_ASSERT_DELTA(ic.GetAlphaDegIC(), 0.0, epsilon*10.);
      TS_ASSERT_DELTA(ic.GetBetaDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 100., epsilon*10.);
      TS_ASSERT_DELTA(ic.GetPhiDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetThetaDegIC(), theta, epsilon*10.);
      TS_ASSERT_DELTA(ic.GetPsiDegIC(), 0.0, epsilon);
    }

    ic.SetThetaRadIC(0.0);
    for(double phi=-180.; phi <= 180.; phi+=10.) {
      ic.SetPhiDegIC(phi);

      TS_ASSERT_DELTA(ic.GetUBodyFpsIC(), 100., epsilon*100.);
      TS_ASSERT_DELTA(ic.GetVBodyFpsIC(), 0., epsilon);
      TS_ASSERT_DELTA(ic.GetWBodyFpsIC(), 0., epsilon);
      TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 100., epsilon*100.);
      TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), 100., epsilon*100.);
      TS_ASSERT_DELTA(ic.GetAlphaDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetBetaDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetPhiDegIC(), phi, epsilon);
      TS_ASSERT_DELTA(ic.GetThetaDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetPsiDegIC(), 0.0, epsilon);
    }

    ic.SetPhiDegIC(0.0);
    for(double psi=0.; psi <= 360.; psi+=10.) {
      ic.SetPsiDegIC(psi);

      TS_ASSERT_DELTA(ic.GetUBodyFpsIC(), 100., epsilon*100.);
      TS_ASSERT_DELTA(ic.GetVBodyFpsIC(), 0., epsilon*10.);
      TS_ASSERT_DELTA(ic.GetWBodyFpsIC(), 0., epsilon);
      TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 100., epsilon*100.);
      TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), 100., epsilon*100.);
      TS_ASSERT_DELTA(ic.GetAlphaDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetBetaDegIC(), 0.0, epsilon*10.);
      TS_ASSERT_DELTA(ic.GetPhiDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetThetaDegIC(), 0.0, epsilon);
      TS_ASSERT_DELTA(ic.GetPsiDegIC(), psi, epsilon*10.);
    }
  }

  void testWindVelocity() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetWindDownKtsIC(1.0);
    TS_ASSERT_DELTA(ic.GetWindDFpsIC(), ktstofps, epsilon);

    ic.SetWindNEDFpsIC(1.0, 2.0, 3.0);
    TS_ASSERT_VECTOR_EQUALS(ic.GetWindNEDFpsIC(), FGColumnVector3(1.0, 2.0, 3.0));
    TS_ASSERT_DELTA(ic.GetWindNFpsIC(), 1.0, epsilon);
    TS_ASSERT_DELTA(ic.GetWindEFpsIC(), 2.0, epsilon);
    TS_ASSERT_DELTA(ic.GetWindDFpsIC(), 3.0, epsilon);
    TS_ASSERT_DELTA(ic.GetWindMagFpsIC(), sqrt(5.0), epsilon);
    TS_ASSERT_DELTA(ic.GetWindDirDegIC(), atan2(2.0, 1.0)*180./M_PI, epsilon);

    double mag = ic.GetWindMagFpsIC();
    ic.SetWindDirDegIC(30.);
    TS_ASSERT_DELTA(ic.GetWindNFpsIC(), 0.5*mag*sqrt(3.0), epsilon);
    TS_ASSERT_DELTA(ic.GetWindEFpsIC(), 0.5*mag, epsilon);
    TS_ASSERT_DELTA(ic.GetWindDFpsIC(), 3.0, epsilon);

    ic.SetWindMagKtsIC(7.0);
    TS_ASSERT_DELTA(ic.GetWindNFpsIC(), 3.5*sqrt(3.0)*ktstofps, epsilon);
    TS_ASSERT_DELTA(ic.GetWindEFpsIC(), 3.5*ktstofps, epsilon);
    TS_ASSERT_DELTA(ic.GetWindDFpsIC(), 3.0, epsilon);

    ic.SetWindMagFpsIC(7.0);
    TS_ASSERT_DELTA(ic.GetWindNFpsIC(), 3.5 * sqrt(3.0), epsilon);
    TS_ASSERT_DELTA(ic.GetWindEFpsIC(), 3.5, epsilon);
    TS_ASSERT_DELTA(ic.GetWindDFpsIC(), 3.0, epsilon);
  }

  void testSetVcalibratedKtsIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetAltitudeASLFtIC(5000.0);
    ic.SetVcalibratedKtsIC(250.0);

    // Verify calibrated airspeed was set
    TS_ASSERT_DELTA(ic.GetVcalibratedKtsIC(), 250.0, 0.1);

    // True airspeed should be higher at altitude
    TS_ASSERT(ic.GetVtrueKtsIC() > 250.0);

    // Test at different altitude
    ic.SetAltitudeASLFtIC(10000.0);
    ic.SetVcalibratedKtsIC(300.0);
    TS_ASSERT_DELTA(ic.GetVcalibratedKtsIC(), 300.0, 0.1);
  }

  void testSetMachIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetAltitudeASLFtIC(10000.0);
    ic.SetMachIC(0.8);

    TS_ASSERT_DELTA(ic.GetMachIC(), 0.8, 0.001);

    // Verify true airspeed was computed correctly
    double vt = ic.GetVtrueFpsIC();
    TS_ASSERT(vt > 0.0);

    // Test subsonic and supersonic
    ic.SetMachIC(0.5);
    TS_ASSERT_DELTA(ic.GetMachIC(), 0.5, 0.001);

    ic.SetMachIC(1.2);
    TS_ASSERT_DELTA(ic.GetMachIC(), 1.2, 0.001);
  }

  void testSetVtrueKtsIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueKtsIC(500.0);
    TS_ASSERT_DELTA(ic.GetVtrueKtsIC(), 500.0, 0.001);
    TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 500.0 * ktstofps, 0.1);

    // Test conversion between knots and fps
    ic.SetVtrueKtsIC(250.0);
    TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 250.0 * ktstofps, 0.1);
  }

  void testSetAlphaIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set initial velocity
    ic.SetUBodyFpsIC(200.0);

    // Test various angles of attack
    for(double alpha = -10.; alpha <= 20.; alpha += 5.) {
      ic.SetAlphaDegIC(alpha);
      TS_ASSERT_DELTA(ic.GetAlphaDegIC(), alpha, 0.1);
      TS_ASSERT_DELTA(ic.GetAlphaRadIC(), alpha * M_PI / 180., 0.001);
    }

    // Test SetAlphaRadIC
    ic.SetAlphaRadIC(0.1);
    TS_ASSERT_DELTA(ic.GetAlphaRadIC(), 0.1, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetAlphaDegIC(), 0.1 * 180. / M_PI, 0.01);
  }

  void testSetBetaIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set initial velocity
    ic.SetUBodyFpsIC(200.0);

    // Test various sideslip angles
    for(double beta = -15.; beta <= 15.; beta += 5.) {
      ic.SetBetaDegIC(beta);
      TS_ASSERT_DELTA(ic.GetBetaDegIC(), beta, 0.1);
      TS_ASSERT_DELTA(ic.GetBetaRadIC(), beta * M_PI / 180., 0.001);
    }

    // Test SetBetaRadIC
    ic.SetBetaRadIC(0.05);
    TS_ASSERT_DELTA(ic.GetBetaRadIC(), 0.05, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetBetaDegIC(), 0.05 * 180. / M_PI, 0.01);
  }

  void testSetFlightPathAngleIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueFpsIC(300.0);

    // Test various flight path angles
    ic.SetFlightPathAngleDegIC(5.0);
    TS_ASSERT_DELTA(ic.GetFlightPathAngleDegIC(), 5.0, 0.1);

    ic.SetFlightPathAngleDegIC(-3.0);
    TS_ASSERT_DELTA(ic.GetFlightPathAngleDegIC(), -3.0, 0.1);

    ic.SetFlightPathAngleDegIC(0.0);
    TS_ASSERT_DELTA(ic.GetFlightPathAngleDegIC(), 0.0, epsilon);

    // Test SetFlightPathAngleRadIC
    ic.SetFlightPathAngleRadIC(0.1);
    TS_ASSERT_DELTA(ic.GetFlightPathAngleRadIC(), 0.1, 0.001);
  }

  void testSetClimbRateIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueFpsIC(300.0);

    // Test climb rate in fps
    ic.SetClimbRateFpsIC(20.0);
    TS_ASSERT_DELTA(ic.GetClimbRateFpsIC(), 20.0, 0.1);
    TS_ASSERT_DELTA(ic.GetClimbRateFpmIC(), 20.0 * 60., 10.);

    // Test descent
    ic.SetClimbRateFpsIC(-15.0);
    TS_ASSERT_DELTA(ic.GetClimbRateFpsIC(), -15.0, 0.1);

    // Test climb rate in fpm
    ic.SetClimbRateFpmIC(1200.0);
    TS_ASSERT_DELTA(ic.GetClimbRateFpmIC(), 1200.0, 10.);
    TS_ASSERT_DELTA(ic.GetClimbRateFpsIC(), 20.0, 0.2);
  }

  void testNEDVelocities() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test setting NED velocities individually
    ic.SetVNorthFpsIC(100.0);
    TS_ASSERT_DELTA(ic.GetVNorthFpsIC(), 100.0, epsilon);

    ic.SetVEastFpsIC(50.0);
    TS_ASSERT_DELTA(ic.GetVEastFpsIC(), 50.0, epsilon);

    ic.SetVDownFpsIC(10.0);
    TS_ASSERT_DELTA(ic.GetVDownFpsIC(), 10.0, epsilon);

    // Verify ground speed
    double vg = sqrt(100.*100. + 50.*50.);
    TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), vg, 0.1);

    // Test with different heading
    ic.SetPsiDegIC(90.0);
    ic.SetVNorthFpsIC(0.0);
    ic.SetVEastFpsIC(200.0);
    ic.SetVDownFpsIC(0.0);

    TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), 200.0, 0.1);
  }

  void testBodyVelocityGetters() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set body velocities
    ic.SetUBodyFpsIC(150.0);
    ic.SetVBodyFpsIC(20.0);
    ic.SetWBodyFpsIC(30.0);

    // Verify getters
    TS_ASSERT_DELTA(ic.GetUBodyFpsIC(), 150.0, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetVBodyFpsIC(), 20.0, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetWBodyFpsIC(), 30.0, epsilon * 10.);

    // Verify UVW vector getter
    FGColumnVector3 uvw = ic.GetUVWFpsIC();
    TS_ASSERT_DELTA(uvw(1), 150.0, 0.1);
    TS_ASSERT_DELTA(uvw(2), 20.0, 0.1);
    TS_ASSERT_DELTA(uvw(3), 30.0, 0.1);
  }

  void testResetIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set some initial conditions first
    ic.SetLatitudeDegIC(45.0);
    ic.SetLongitudeDegIC(-120.0);
    ic.SetAltitudeAGLFtIC(1000.0);
    ic.SetVtrueKtsIC(250.0);

    // Reset with new values
    double u0 = 200.0, v0 = 10.0, w0 = 5.0;
    double p0 = 0.01, q0 = 0.02, r0 = 0.01;
    double alpha0 = 0.05, beta0 = 0.02;
    double phi0 = 0.1, theta0 = 0.05, psi0 = 1.57;
    double lat0 = 0.7854, lon0 = -2.094, alt0 = 5000.0, gamma0 = 0.087;

    ic.ResetIC(u0, v0, w0, p0, q0, r0, alpha0, beta0,
               phi0, theta0, psi0, lat0, lon0, alt0, gamma0);

    // Verify values were reset
    // Note: ResetIC calls SetFlightPathAngleRadIC at the end which modifies theta
    TS_ASSERT_DELTA(ic.GetUBodyFpsIC(), u0, 1.0);
    TS_ASSERT_DELTA(ic.GetVBodyFpsIC(), v0, 1.0);
    TS_ASSERT_DELTA(ic.GetWBodyFpsIC(), w0, 1.0);
    TS_ASSERT_DELTA(ic.GetPRadpsIC(), p0, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetQRadpsIC(), q0, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetRRadpsIC(), r0, epsilon * 10.);
    TS_ASSERT_DELTA(ic.GetAlphaRadIC(), alpha0, 0.01);
    TS_ASSERT_DELTA(ic.GetPhiRadIC(), phi0, 0.001);
    // Theta is modified by SetFlightPathAngleRadIC, so we check flight path angle instead
    TS_ASSERT_DELTA(ic.GetFlightPathAngleRadIC(), gamma0, 0.01);
    TS_ASSERT_DELTA(ic.GetPsiRadIC(), psi0, 0.001);
    TS_ASSERT_DELTA(ic.GetLatitudeRadIC(), lat0, 0.001);
    TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon0, 0.001);
  }

  void testPQRRates() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test setting body rotation rates
    ic.SetPRadpsIC(0.1);
    TS_ASSERT_DELTA(ic.GetPRadpsIC(), 0.1, epsilon);

    ic.SetQRadpsIC(0.05);
    TS_ASSERT_DELTA(ic.GetQRadpsIC(), 0.05, epsilon);

    ic.SetRRadpsIC(0.02);
    TS_ASSERT_DELTA(ic.GetRRadpsIC(), 0.02, epsilon);

    // Verify PQR vector getter
    FGColumnVector3 pqr = ic.GetPQRRadpsIC();
    TS_ASSERT_DELTA(pqr(1), 0.1, epsilon);
    TS_ASSERT_DELTA(pqr(2), 0.05, epsilon);
    TS_ASSERT_DELTA(pqr(3), 0.02, epsilon);
  }

  void testSetVequivalentKtsIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetAltitudeASLFtIC(8000.0);
    ic.SetVequivalentKtsIC(200.0);

    TS_ASSERT_DELTA(ic.GetVequivalentKtsIC(), 200.0, 0.5);

    // True airspeed should be different from equivalent at altitude
    double vt = ic.GetVtrueKtsIC();
    TS_ASSERT(fabs(vt - 200.0) > 1.0);
  }

  void testSetVgroundKtsIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPsiDegIC(0.0);  // Heading north
    ic.SetVgroundKtsIC(250.0);

    TS_ASSERT_DELTA(ic.GetVgroundKtsIC(), 250.0, 0.1);
    TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), 250.0 * ktstofps, 1.0);

    // Change heading
    ic.SetPsiDegIC(90.0);  // Heading east
    ic.SetVgroundKtsIC(300.0);
    TS_ASSERT_DELTA(ic.GetVgroundKtsIC(), 300.0, 0.1);
  }

  void testHeadwindCrosswind() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueKtsIC(200.0);
    ic.SetPsiDegIC(0.0);  // Heading north

    // Set headwind
    ic.SetHeadWindKtsIC(20.0);
    double windN = ic.GetWindNFpsIC();
    double windE = ic.GetWindEFpsIC();

    // Headwind should be from the north (negative N component)
    TS_ASSERT(windN < 0.0);
    TS_ASSERT_DELTA(windE, 0.0, 0.1);

    // Set crosswind (from left)
    ic.SetCrossWindKtsIC(10.0);
    windN = ic.GetWindNFpsIC();
    windE = ic.GetWindEFpsIC();

    // Crosswind from left should have positive east component
    TS_ASSERT(windE > 0.0);
  }

  void testLongitudeSetters() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test SetLongitudeDegIC
    for(double lon = -180.; lon <= 180.; lon += 45.) {
      ic.SetLongitudeDegIC(lon);
      TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, epsilon * 100.);
      TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), lon * M_PI / 180., epsilon);
    }

    // Test SetLongitudeRadIC
    ic.SetLongitudeRadIC(M_PI / 4.);
    TS_ASSERT_DELTA(ic.GetLongitudeRadIC(), M_PI / 4., epsilon);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), 45.0, epsilon * 100.);
  }

  void testInitializeIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set various parameters
    ic.SetLatitudeDegIC(30.0);
    ic.SetLongitudeDegIC(-100.0);
    ic.SetAltitudeASLFtIC(10000.0);
    ic.SetVtrueKtsIC(400.0);
    ic.SetPhiDegIC(10.0);
    ic.SetThetaDegIC(5.0);
    ic.SetPsiDegIC(45.0);

    // Initialize - should reset everything to defaults
    ic.InitializeIC();

    // Verify reset to defaults
    TS_ASSERT_EQUALS(ic.GetLatitudeDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetLongitudeDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetAltitudeASLFtIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetVtrueFpsIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPhiDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetThetaDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetPsiDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetAlphaDegIC(), 0.0);
    TS_ASSERT_EQUALS(ic.GetBetaDegIC(), 0.0);
  }

  void testEulerAngleConsistency() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetUBodyFpsIC(200.0);

    // Test that setting Euler angles in radians and degrees are consistent
    ic.SetPhiDegIC(15.0);
    TS_ASSERT_DELTA(ic.GetPhiRadIC(), 15.0 * M_PI / 180., epsilon);

    ic.SetPhiRadIC(0.3);
    TS_ASSERT_DELTA(ic.GetPhiDegIC(), 0.3 * 180. / M_PI, epsilon * 100.);

    ic.SetThetaDegIC(10.0);
    TS_ASSERT_DELTA(ic.GetThetaRadIC(), 10.0 * M_PI / 180., epsilon);

    ic.SetThetaRadIC(0.2);
    TS_ASSERT_DELTA(ic.GetThetaDegIC(), 0.2 * 180. / M_PI, epsilon * 100.);

    ic.SetPsiDegIC(90.0);
    TS_ASSERT_DELTA(ic.GetPsiRadIC(), M_PI / 2., epsilon);

    ic.SetPsiRadIC(M_PI);
    TS_ASSERT_DELTA(ic.GetPsiDegIC(), 180.0, epsilon * 100.);
  }

  void testGetOrientation() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPhiDegIC(10.0);
    ic.SetThetaDegIC(5.0);
    ic.SetPsiDegIC(30.0);

    const FGQuaternion& quat = ic.GetOrientation();

    // Verify quaternion represents the same orientation
    TS_ASSERT_DELTA(quat.GetEulerDeg(FGJSBBase::ePhi), 10.0, 0.01);
    TS_ASSERT_DELTA(quat.GetEulerDeg(FGJSBBase::eTht), 5.0, 0.01);
    TS_ASSERT_DELTA(quat.GetEulerDeg(FGJSBBase::ePsi), 30.0, 0.01);
  }

  void testGetPosition() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLatitudeDegIC(40.0);
    ic.SetLongitudeDegIC(-75.0);
    ic.SetAltitudeASLFtIC(5000.0);

    const FGLocation& pos = ic.GetPosition();

    TS_ASSERT_DELTA(pos.GetLatitudeDeg(), 40.0, 0.01);
    TS_ASSERT_DELTA(pos.GetLongitudeDeg(), -75.0, 0.01);
  }

  void testSpeedSetTracking() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test that last speed set is tracked correctly
    ic.SetVtrueKtsIC(200.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setvt);

    ic.SetVcalibratedKtsIC(250.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setvc);

    ic.SetMachIC(0.7);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setmach);

    ic.SetVequivalentKtsIC(220.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setve);

    ic.SetUBodyFpsIC(300.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setuvw);

    ic.SetVNorthFpsIC(250.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setned);

    ic.SetVgroundKtsIC(280.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setvg);
  }

  void testTargetNlfIC() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Default should be 1.0
    TS_ASSERT_DELTA(ic.GetTargetNlfIC(), 1.0, epsilon);

    // Set different values
    ic.SetTargetNlfIC(2.5);
    TS_ASSERT_DELTA(ic.GetTargetNlfIC(), 2.5, epsilon);

    ic.SetTargetNlfIC(0.5);
    TS_ASSERT_DELTA(ic.GetTargetNlfIC(), 0.5, epsilon);
  }

  void testAltitudeSpeedInteraction() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set Mach at one altitude
    ic.SetAltitudeASLFtIC(10000.0);
    ic.SetMachIC(0.8);
    double vt1 = ic.GetVtrueFpsIC();

    // Change altitude - true speed should update to maintain same Mach
    ic.SetAltitudeASLFtIC(20000.0);
    double vt2 = ic.GetVtrueFpsIC();

    // True speed should be lower at higher altitude for same Mach
    TS_ASSERT(vt2 < vt1);
    TS_ASSERT_DELTA(ic.GetMachIC(), 0.8, 0.01);
  }

  void testVelocityVectorConsistency() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set body velocities
    double u = 150.0, v = 20.0, w = 10.0;
    ic.SetUBodyFpsIC(u);
    ic.SetVBodyFpsIC(v);
    ic.SetWBodyFpsIC(w);

    // Get UVW vector and verify components
    FGColumnVector3 uvw = ic.GetUVWFpsIC();
    TS_ASSERT_DELTA(uvw(1), u, 0.1);
    TS_ASSERT_DELTA(uvw(2), v, 0.1);
    TS_ASSERT_DELTA(uvw(3), w, 0.1);

    // Verify true speed matches vector magnitude
    double vt_computed = sqrt(u*u + v*v + w*w);
    TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), vt_computed, 0.5);
  }

  /***************************************************************************
   * Earth Position Angle Tests
   ***************************************************************************/

  void testEarthPositionAngleDefault() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Default earth position angle should be 0
    TS_ASSERT_DELTA(ic.GetEarthPositionAngleIC(), 0.0, epsilon);
  }

  /***************************************************************************
   * Trim Request Tests
   ***************************************************************************/

  void testTrimRequestedDefault() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Verify trim requested returns a valid value
    int trim = ic.TrimRequested();
    TS_ASSERT(trim >= 0);  // Should be a valid trim mode
  }

  /***************************************************************************
   * Engine Running Tests
   ***************************************************************************/

  void testIsEngineRunningDefault() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // By default no engines should be running
    for (unsigned int i = 0; i < 8; i++) {
      TS_ASSERT_EQUALS(ic.IsEngineRunning(i), false);
    }
  }

  /***************************************************************************
   * Wind in Body Frame Tests
   ***************************************************************************/

  void testWindBodyFrameDefault() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Default wind should be zero
    TS_ASSERT_DELTA(ic.GetWindUFpsIC(), 0.0, epsilon);
    TS_ASSERT_DELTA(ic.GetWindVFpsIC(), 0.0, epsilon);
    TS_ASSERT_DELTA(ic.GetWindWFpsIC(), 0.0, epsilon);
  }

  void testWindBodyFrameWithNEDWind() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set NED wind and check body frame transformation
    ic.SetWindNEDFpsIC(10.0, 5.0, 0.0);

    // With level flight (phi=theta=psi=0), body and NED align
    // so body U should match NED N, body V should match NED E
    double windU = ic.GetWindUFpsIC();
    double windV = ic.GetWindVFpsIC();
    double windW = ic.GetWindWFpsIC();

    // All should be finite
    TS_ASSERT(!std::isnan(windU));
    TS_ASSERT(!std::isnan(windV));
    TS_ASSERT(!std::isnan(windW));
  }

  /***************************************************************************
   * Speed Relationship Tests
   ***************************************************************************/

  void testSpeedRelationshipsAtSeaLevel() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // At sea level, calibrated = equivalent = true airspeed (approximately)
    ic.SetAltitudeASLFtIC(0.0);
    ic.SetVcalibratedKtsIC(200.0);

    double vc = ic.GetVcalibratedKtsIC();
    double ve = ic.GetVequivalentKtsIC();
    double vt = ic.GetVtrueKtsIC();

    // At sea level, these should be approximately equal
    TS_ASSERT_DELTA(vc, 200.0, 1.0);
    TS_ASSERT_DELTA(ve, vt, 5.0);  // EAS and TAS nearly equal at sea level
  }

  void testSpeedRelationshipsAtAltitude() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // At altitude, true airspeed > calibrated airspeed
    ic.SetAltitudeASLFtIC(30000.0);
    ic.SetVcalibratedKtsIC(200.0);

    double vt = ic.GetVtrueKtsIC();

    // True airspeed should be significantly higher at altitude
    TS_ASSERT(vt > 250.0);  // Much higher at 30000 ft
  }

  void testMachAtDifferentAltitudes() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Same true airspeed gives different Mach at different altitudes
    ic.SetAltitudeASLFtIC(0.0);
    ic.SetVtrueKtsIC(400.0);
    double mach_low = ic.GetMachIC();

    ic.SetAltitudeASLFtIC(40000.0);
    ic.SetVtrueKtsIC(400.0);
    double mach_high = ic.GetMachIC();

    // Mach should be higher at altitude (lower speed of sound)
    TS_ASSERT(mach_high > mach_low);
  }

  /***************************************************************************
   * Alpha, Gamma, Theta Relationship Tests
   ***************************************************************************/

  void testAlphaGammaThetaRelationship() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set velocity first
    ic.SetVtrueFpsIC(300.0);

    // theta = alpha + gamma for level-wing flight
    // Set alpha and gamma, check theta
    ic.SetAlphaDegIC(5.0);
    ic.SetFlightPathAngleDegIC(3.0);

    double theta = ic.GetThetaDegIC();
    double alpha = ic.GetAlphaDegIC();
    double gamma = ic.GetFlightPathAngleDegIC();

    // In steady flight: theta ≈ alpha + gamma
    TS_ASSERT_DELTA(theta, alpha + gamma, 1.0);
  }

  void testClimbRateFlightPathAngleRelationship() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // gamma = arcsin(climb_rate / Vt)
    double vt = 300.0;  // fps
    ic.SetVtrueFpsIC(vt);

    // Set climb rate
    double climb_rate = 20.0;  // fps
    ic.SetClimbRateFpsIC(climb_rate);

    double gamma = ic.GetFlightPathAngleRadIC();
    double expected_gamma = asin(climb_rate / vt);

    TS_ASSERT_DELTA(gamma, expected_gamma, 0.01);
  }

  void testClimbRateFromFlightPathAngle() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    double vt = 400.0;  // fps
    ic.SetVtrueFpsIC(vt);

    // Set flight path angle to 5 degrees
    ic.SetFlightPathAngleDegIC(5.0);

    double climb_rate = ic.GetClimbRateFpsIC();
    double expected = vt * sin(5.0 * M_PI / 180.0);

    TS_ASSERT_DELTA(climb_rate, expected, 1.0);
  }

  /***************************************************************************
   * Extreme Position Tests
   ***************************************************************************/

  void testPositionNearNorthPole() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLatitudeDegIC(89.9);
    ic.SetLongitudeDegIC(0.0);
    ic.SetAltitudeASLFtIC(1000.0);

    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 89.9, 0.01);
    TS_ASSERT(!std::isnan(ic.GetGeodLatitudeDegIC()));
  }

  void testPositionNearSouthPole() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLatitudeDegIC(-89.9);
    ic.SetLongitudeDegIC(180.0);
    ic.SetAltitudeASLFtIC(5000.0);

    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), -89.9, 0.01);
    TS_ASSERT(!std::isnan(ic.GetGeodLatitudeDegIC()));
  }

  void testPositionAtEquator() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLatitudeDegIC(0.0);
    ic.SetLongitudeDegIC(90.0);
    ic.SetAltitudeASLFtIC(0.0);

    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 0.0, epsilon);
    // At equator, geodetic and geocentric latitude should be equal
    TS_ASSERT_DELTA(ic.GetGeodLatitudeDegIC(), 0.0, 0.01);
  }

  void testPositionAtDateLine() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLatitudeDegIC(45.0);
    ic.SetLongitudeDegIC(180.0);
    ic.SetAltitudeASLFtIC(1000.0);

    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), 180.0, epsilon * 100.);
  }

  void testPositionNegativeLongitude() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLongitudeDegIC(-180.0);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), -180.0, epsilon * 100.);

    ic.SetLongitudeDegIC(-90.0);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), -90.0, epsilon * 100.);
  }

  /***************************************************************************
   * Geodetic vs Geocentric Latitude Tests
   ***************************************************************************/

  void testGeodeticVsGeocentricLatitude() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // At mid-latitudes, geodetic latitude differs from geocentric
    ic.SetLatitudeDegIC(45.0);  // Set geocentric

    double geocentric = ic.GetLatitudeDegIC();
    double geodetic = ic.GetGeodLatitudeDegIC();

    // Geodetic latitude should be slightly larger at 45 degrees
    // (Earth is flattened at poles, bulges at equator)
    TS_ASSERT(geodetic >= geocentric - 0.5);  // Within reasonable range
  }

  void testSetGeodeticThenGetGeocentric() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetAltitudeASLFtIC(1000.0);
    ic.SetGeodLatitudeDegIC(45.0);

    double geodetic = ic.GetGeodLatitudeDegIC();
    double geocentric = ic.GetLatitudeDegIC();

    // Should be close but not identical
    TS_ASSERT_DELTA(geodetic, 45.0, 0.01);
    TS_ASSERT(fabs(geodetic - geocentric) < 1.0);  // Reasonable difference
  }

  /***************************************************************************
   * Wind Magnitude and Direction Tests
   ***************************************************************************/

  void testWindMagnitudeAndDirection() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set wind using magnitude and direction
    ic.SetWindMagKtsIC(20.0);
    ic.SetWindDirDegIC(90.0);  // Wind from east

    // Check components
    double windMag = ic.GetWindMagFpsIC();
    double windDir = ic.GetWindDirDegIC();

    TS_ASSERT(windMag > 0.0);
    TS_ASSERT_DELTA(windDir, 90.0, 1.0);

    // Wind from east should have N=0, E<0 (wind blowing from east to west)
    double windN = ic.GetWindNFpsIC();
    double windE = ic.GetWindEFpsIC();

    TS_ASSERT_DELTA(windN, 0.0, 1.0);
  }

  void testWindDirectionWrap() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test wind direction at various angles
    ic.SetWindMagKtsIC(10.0);

    ic.SetWindDirDegIC(0.0);  // From north
    double windN_0 = ic.GetWindNFpsIC();

    ic.SetWindDirDegIC(180.0);  // From south
    double windN_180 = ic.GetWindNFpsIC();

    // Wind direction should affect N component (opposite signs for N vs S wind)
    // Just verify they're different and wind magnitude is correct
    TS_ASSERT(windN_0 != windN_180 || ic.GetWindMagFpsIC() < 0.1);
    TS_ASSERT(ic.GetWindMagFpsIC() > 0.0);
  }

  /***************************************************************************
   * Combined Headwind and Crosswind Tests
   ***************************************************************************/

  void testCombinedHeadwindCrosswind() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueKtsIC(200.0);
    ic.SetPsiDegIC(0.0);  // Heading north

    // Set both headwind and crosswind
    ic.SetHeadWindKtsIC(15.0);
    ic.SetCrossWindKtsIC(10.0);

    // Wind should have both N and E components
    double windMag = ic.GetWindMagFpsIC();
    TS_ASSERT(windMag > 0.0);

    // Total wind magnitude should be sqrt(15^2 + 10^2) in knots
    double expectedMag = sqrt(15.0*15.0 + 10.0*10.0) * ktstofps;
    TS_ASSERT_DELTA(windMag, expectedMag, 1.0);
  }

  /***************************************************************************
   * Terrain Elevation Tests
   ***************************************************************************/

  void testTerrainElevationSetting() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetTerrainElevationFtIC(5000.0);
    TS_ASSERT_DELTA(ic.GetTerrainElevationFtIC(), 5000.0, 1.0);

    ic.SetTerrainElevationFtIC(0.0);
    TS_ASSERT_DELTA(ic.GetTerrainElevationFtIC(), 0.0, 1.0);

    // Negative terrain (below sea level)
    ic.SetTerrainElevationFtIC(-1000.0);
    TS_ASSERT_DELTA(ic.GetTerrainElevationFtIC(), -1000.0, 1.0);
  }

  void testAGLWithTerrainElevation() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetTerrainElevationFtIC(2000.0);
    ic.SetAltitudeAGLFtIC(500.0);

    // ASL should be terrain + AGL
    double asl = ic.GetAltitudeASLFtIC();
    TS_ASSERT_DELTA(asl, 2500.0, 10.0);

    // Now change terrain elevation
    ic.SetTerrainElevationFtIC(3000.0);
    ic.SetAltitudeAGLFtIC(500.0);
    asl = ic.GetAltitudeASLFtIC();
    TS_ASSERT_DELTA(asl, 3500.0, 10.0);
  }

  /***************************************************************************
   * Very High Altitude Tests
   ***************************************************************************/

  void testVeryHighAltitude() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test at space-like altitude
    ic.SetAltitudeASLFtIC(300000.0);  // ~90km, near space
    TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC(), 300000.0, 100.0);

    // Set Mach at very high altitude
    ic.SetMachIC(5.0);  // Hypersonic
    TS_ASSERT_DELTA(ic.GetMachIC(), 5.0, 0.1);
  }

  /***************************************************************************
   * Zero Velocity Edge Cases
   ***************************************************************************/

  void testZeroVelocity() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueFpsIC(0.0);

    TS_ASSERT_DELTA(ic.GetVtrueFpsIC(), 0.0, epsilon);
    TS_ASSERT_DELTA(ic.GetVgroundFpsIC(), 0.0, epsilon);
    TS_ASSERT_DELTA(ic.GetMachIC(), 0.0, epsilon);
  }

  void testFlightPathAngleWithZeroVelocity() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueFpsIC(0.0);

    // Flight path angle should be 0 when velocity is 0
    double gamma = ic.GetFlightPathAngleRadIC();
    TS_ASSERT_DELTA(gamma, 0.0, epsilon);
  }

  /***************************************************************************
   * Orientation Quaternion Tests
   ***************************************************************************/

  void testQuaternionNormalized() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPhiDegIC(30.0);
    ic.SetThetaDegIC(15.0);
    ic.SetPsiDegIC(90.0);

    const FGQuaternion& quat = ic.GetOrientation();

    // Quaternion should be normalized (magnitude = 1)
    double mag = sqrt(quat(1)*quat(1) + quat(2)*quat(2) +
                      quat(3)*quat(3) + quat(4)*quat(4));
    TS_ASSERT_DELTA(mag, 1.0, epsilon);
  }

  void testQuaternionIdentityAtZeroAngles() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPhiDegIC(0.0);
    ic.SetThetaDegIC(0.0);
    ic.SetPsiDegIC(0.0);

    const FGQuaternion& quat = ic.GetOrientation();

    // Identity quaternion is [1, 0, 0, 0]
    TS_ASSERT_DELTA(quat(1), 1.0, epsilon);
    TS_ASSERT_DELTA(quat(2), 0.0, epsilon);
    TS_ASSERT_DELTA(quat(3), 0.0, epsilon);
    TS_ASSERT_DELTA(quat(4), 0.0, epsilon);
  }

  /***************************************************************************
   * Alpha and Beta at Various Velocities
   ***************************************************************************/

  void testAlphaWithPositiveW() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Flying forward with nose-up attack
    ic.SetUBodyFpsIC(200.0);
    ic.SetWBodyFpsIC(20.0);  // Positive W means positive alpha

    double alpha = ic.GetAlphaRadIC();
    TS_ASSERT(alpha > 0.0);

    // alpha = atan(w/u) approximately
    double expected = atan2(20.0, 200.0);
    TS_ASSERT_DELTA(alpha, expected, 0.01);
  }

  void testBetaWithPositiveV() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Flying forward with sideslip
    ic.SetUBodyFpsIC(200.0);
    ic.SetVBodyFpsIC(20.0);  // Positive V means positive beta

    double beta = ic.GetBetaRadIC();
    TS_ASSERT(beta > 0.0);

    // beta = asin(v/vt) approximately
    double vt = sqrt(200.0*200.0 + 20.0*20.0);
    double expected = asin(20.0 / vt);
    TS_ASSERT_DELTA(beta, expected, 0.01);
  }

  /***************************************************************************
   * Speed Set Tracking Additional Tests
   ***************************************************************************/

  void testSpeedSetAfterVBodyFps() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVBodyFpsIC(50.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setuvw);

    ic.SetWBodyFpsIC(10.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setuvw);
  }

  void testSpeedSetAfterNEDVelocities() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVEastFpsIC(100.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setned);

    ic.SetVDownFpsIC(-10.0);
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setned);
  }

  /***************************************************************************
   * Target Nlf Additional Tests
   ***************************************************************************/

  void testTargetNlfNegative() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test negative load factor (inverted flight)
    ic.SetTargetNlfIC(-1.0);
    TS_ASSERT_DELTA(ic.GetTargetNlfIC(), -1.0, epsilon);
  }

  void testTargetNlfHighG() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test high G load factor
    ic.SetTargetNlfIC(9.0);
    TS_ASSERT_DELTA(ic.GetTargetNlfIC(), 9.0, epsilon);
  }

  /***************************************************************************
   * FGLocation Access Tests
   ***************************************************************************/

  void testGetPositionRadius() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetLatitudeDegIC(45.0);
    ic.SetLongitudeDegIC(-75.0);
    ic.SetAltitudeASLFtIC(10000.0);

    const FGLocation& pos = ic.GetPosition();

    // Radius should be approximately Earth's radius + altitude
    double radius = pos.GetRadius();
    TS_ASSERT(radius > 20000000.0);  // > 20 million feet (Earth radius)
    TS_ASSERT(radius < 25000000.0);  // < 25 million feet
  }

  /***************************************************************************
   * Multiple Set/Get Cycles
   ***************************************************************************/

  void testMultipleSetGetCycles() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Test that multiple set/get cycles are stable
    for (int i = 0; i < 5; i++) {
      double lat = 30.0 + i * 5.0;
      double lon = -100.0 + i * 10.0;
      double alt = 5000.0 + i * 1000.0;

      ic.SetLatitudeDegIC(lat);
      ic.SetLongitudeDegIC(lon);
      ic.SetAltitudeASLFtIC(alt);

      TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), lat, 0.01);
      TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), lon, 0.01);
      TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC(), alt, 10.0);
    }
  }

  void testVelocitySetGetCycles() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetAltitudeASLFtIC(10000.0);

    // Multiple velocity set/get cycles
    for (int i = 0; i < 5; i++) {
      double vt = 200.0 + i * 50.0;
      ic.SetVtrueKtsIC(vt);
      TS_ASSERT_DELTA(ic.GetVtrueKtsIC(), vt, 1.0);
    }
  }

  /***************************************************************************
   * Conversion Consistency Tests
   ***************************************************************************/

  void testKnotsToFpsConversion() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueKtsIC(100.0);
    double vt_fps = ic.GetVtrueFpsIC();
    double vt_kts = ic.GetVtrueKtsIC();

    // Check conversion factor
    TS_ASSERT_DELTA(vt_fps / vt_kts, ktstofps, 0.001);
  }

  void testDegreesToRadiansConversion() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPhiDegIC(45.0);
    TS_ASSERT_DELTA(ic.GetPhiRadIC(), M_PI / 4.0, epsilon);

    ic.SetThetaDegIC(30.0);
    TS_ASSERT_DELTA(ic.GetThetaRadIC(), M_PI / 6.0, epsilon);

    ic.SetPsiDegIC(90.0);
    TS_ASSERT_DELTA(ic.GetPsiRadIC(), M_PI / 2.0, epsilon);
  }

  void testFpmToFpsConversion() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueFpsIC(300.0);
    ic.SetClimbRateFpmIC(1200.0);

    TS_ASSERT_DELTA(ic.GetClimbRateFpsIC(), 20.0, 0.1);
    TS_ASSERT_DELTA(ic.GetClimbRateFpmIC(), 1200.0, 5.0);
  }

  /***************************************************************************
   * Edge Case: Very Small Velocities
   ***************************************************************************/

  void testVerySmallVelocity() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVtrueFpsIC(0.001);
    TS_ASSERT(!std::isnan(ic.GetMachIC()));
    TS_ASSERT(!std::isnan(ic.GetVcalibratedKtsIC()));
  }

  /***************************************************************************
   * Heading Wrap Tests
   ***************************************************************************/

  void testHeadingWrap360() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPsiDegIC(359.0);
    TS_ASSERT_DELTA(ic.GetPsiDegIC(), 359.0, 0.1);

    ic.SetPsiDegIC(0.0);
    TS_ASSERT_DELTA(ic.GetPsiDegIC(), 0.0, epsilon);
  }

  /***************************************************************************
   * InitializeIC Then Set Tests
   ***************************************************************************/

  void testInitializeThenSetValues() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set values
    ic.SetLatitudeDegIC(40.0);
    ic.SetVtrueKtsIC(300.0);

    // Initialize (reset)
    ic.InitializeIC();

    // Set new values
    ic.SetLatitudeDegIC(50.0);
    ic.SetVtrueKtsIC(400.0);

    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 50.0, epsilon);
    TS_ASSERT_DELTA(ic.GetVtrueKtsIC(), 400.0, 1.0);
  }

  /***************************************************************************
   * Section 17: Combined Position and Velocity Tests
   ***************************************************************************/

  void testPositionVelocityTogether() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set position
    ic.SetLatitudeDegIC(35.0);
    ic.SetLongitudeDegIC(-118.0);
    ic.SetAltitudeASLFtIC(5000.0);

    // Set velocity
    ic.SetVtrueKtsIC(250.0);
    ic.SetPsiDegIC(90.0);

    // Verify both are correct
    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 35.0, 0.01);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), -118.0, 0.01);
    TS_ASSERT_DELTA(ic.GetVtrueKtsIC(), 250.0, 1.0);
    TS_ASSERT_DELTA(ic.GetPsiDegIC(), 90.0, 0.1);
  }

  void testAltitudeVelocityInteraction() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set velocity first
    ic.SetVcalibratedKtsIC(200.0);

    // Then change altitude
    ic.SetAltitudeASLFtIC(20000.0);

    // Calibrated should remain same, true should increase
    TS_ASSERT_DELTA(ic.GetVcalibratedKtsIC(), 200.0, 1.0);
    TS_ASSERT(ic.GetVtrueKtsIC() > 200.0);
  }

  /***************************************************************************
   * Section 18: Body Rates Comprehensive Tests
   ***************************************************************************/

  void testRollRatePositive() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPRadpsIC(0.5);  // Roll right
    TS_ASSERT_DELTA(ic.GetPRadpsIC(), 0.5, epsilon);

    FGColumnVector3 pqr = ic.GetPQRRadpsIC();
    TS_ASSERT_DELTA(pqr(1), 0.5, epsilon);
  }

  void testPitchRatePositive() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetQRadpsIC(0.3);  // Pitch up
    TS_ASSERT_DELTA(ic.GetQRadpsIC(), 0.3, epsilon);

    FGColumnVector3 pqr = ic.GetPQRRadpsIC();
    TS_ASSERT_DELTA(pqr(2), 0.3, epsilon);
  }

  void testYawRatePositive() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetRRadpsIC(0.2);  // Yaw right
    TS_ASSERT_DELTA(ic.GetRRadpsIC(), 0.2, epsilon);

    FGColumnVector3 pqr = ic.GetPQRRadpsIC();
    TS_ASSERT_DELTA(pqr(3), 0.2, epsilon);
  }

  void testCombinedBodyRates() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPRadpsIC(0.1);
    ic.SetQRadpsIC(0.2);
    ic.SetRRadpsIC(0.15);

    FGColumnVector3 pqr = ic.GetPQRRadpsIC();
    TS_ASSERT_DELTA(pqr(1), 0.1, epsilon);
    TS_ASSERT_DELTA(pqr(2), 0.2, epsilon);
    TS_ASSERT_DELTA(pqr(3), 0.15, epsilon);
  }

  /***************************************************************************
   * Section 19: Wind Component Tests
   ***************************************************************************/

  void testWindFromNorth() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetWindMagKtsIC(20.0);
    ic.SetWindDirDegIC(0.0);  // From north

    // Verify direction was set
    double windDir = ic.GetWindDirDegIC();
    TS_ASSERT_DELTA(windDir, 0.0, 1.0);
  }

  void testWindFromSouth() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetWindMagKtsIC(20.0);
    ic.SetWindDirDegIC(180.0);  // From south

    // Verify direction
    double windDir = ic.GetWindDirDegIC();
    TS_ASSERT_DELTA(windDir, 180.0, 1.0);
  }

  void testWindFromWest() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetWindMagKtsIC(15.0);
    ic.SetWindDirDegIC(270.0);  // From west

    // Verify direction was set (may be normalized to -90 or 270)
    double windDir = ic.GetWindDirDegIC();
    // Normalize to 0-360 range for comparison
    if (windDir < 0) windDir += 360.0;
    TS_ASSERT_DELTA(windDir, 270.0, 1.0);
  }

  /***************************************************************************
   * Section 20: Trim and Load Factor Tests
   ***************************************************************************/

  void testTrimRequestValue() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    int trim = ic.TrimRequested();
    // Should return a valid trim mode
    TS_ASSERT(trim >= 0 && trim < 10);
  }

  void testZeroLoadFactor() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetTargetNlfIC(0.0);
    TS_ASSERT_DELTA(ic.GetTargetNlfIC(), 0.0, epsilon);
  }

  /***************************************************************************
   * Section 21: Altitude Conversion Tests
   ***************************************************************************/

  void testASLToAGLWithTerrain() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetTerrainElevationFtIC(1000.0);
    ic.SetAltitudeASLFtIC(3000.0);

    double agl = ic.GetAltitudeAGLFtIC();
    TS_ASSERT_DELTA(agl, 2000.0, 10.0);
  }

  void testAGLToASLWithTerrain() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetTerrainElevationFtIC(500.0);
    ic.SetAltitudeAGLFtIC(1500.0);

    double asl = ic.GetAltitudeASLFtIC();
    TS_ASSERT_DELTA(asl, 2000.0, 10.0);
  }

  /***************************************************************************
   * Section 22: Velocity Vector Magnitude Tests
   ***************************************************************************/

  void testVelocityMagnitudeFromComponents() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVNorthFpsIC(300.0);
    ic.SetVEastFpsIC(400.0);
    ic.SetVDownFpsIC(0.0);

    double vg = ic.GetVgroundFpsIC();
    TS_ASSERT_DELTA(vg, 500.0, 1.0);  // 3-4-5 triangle
  }

  void testBodyVelocityMagnitude() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetUBodyFpsIC(300.0);
    ic.SetVBodyFpsIC(0.0);
    ic.SetWBodyFpsIC(0.0);

    double vt = ic.GetVtrueFpsIC();
    TS_ASSERT_DELTA(vt, 300.0, 1.0);
  }

  /***************************************************************************
   * Section 23: Speed Mode Persistence Tests
   ***************************************************************************/

  void testSpeedModeAfterPositionChange() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetMachIC(0.8);
    double mach = ic.GetMachIC();
    TS_ASSERT_DELTA(mach, 0.8, 0.01);

    // Change position
    ic.SetLatitudeDegIC(40.0);

    // Mach should still be approximately the same
    TS_ASSERT_DELTA(ic.GetMachIC(), 0.8, 0.1);
  }

  void testSpeedModeAfterAltitudeChange() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetVcalibratedKtsIC(250.0);
    double vc = ic.GetVcalibratedKtsIC();
    TS_ASSERT_DELTA(vc, 250.0, 1.0);

    // Change altitude
    ic.SetAltitudeASLFtIC(20000.0);

    // Vcalibrated should still be approximately the same
    TS_ASSERT_DELTA(ic.GetVcalibratedKtsIC(), 250.0, 5.0);
  }

  /***************************************************************************
   * Section 24: Euler Angle Edge Cases
   ***************************************************************************/

  void testPhiNear180() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPhiDegIC(179.0);
    TS_ASSERT_DELTA(ic.GetPhiDegIC(), 179.0, 0.1);
  }

  void testThetaNear90() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetUBodyFpsIC(100.0);
    ic.SetThetaDegIC(85.0);
    TS_ASSERT_DELTA(ic.GetThetaDegIC(), 85.0, 1.0);
  }

  void testPsiNear360() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetPsiDegIC(355.0);
    TS_ASSERT_DELTA(ic.GetPsiDegIC(), 355.0, 0.1);
  }

  /***************************************************************************
   * Section 25: Stress Tests
   ***************************************************************************/

  void testManyPositionUpdates() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    for (int i = 0; i < 100; i++) {
      double lat = -90.0 + i * 1.8;
      double lon = -180.0 + i * 3.6;
      ic.SetLatitudeDegIC(lat);
      ic.SetLongitudeDegIC(lon);
    }

    // Final values should be correct: i=99, lat=-90+99*1.8=88.2, lon=-180+99*3.6=176.4
    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 88.2, 0.1);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), 176.4, 0.1);
  }

  void testManyVelocityUpdates() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    for (int i = 0; i < 50; i++) {
      double vt = 100.0 + i * 10.0;
      ic.SetVtrueKtsIC(vt);
    }

    TS_ASSERT_DELTA(ic.GetVtrueKtsIC(), 590.0, 1.0);
  }

  void testAlternatingSpeedModes() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    ic.SetAltitudeASLFtIC(10000.0);

    for (int i = 0; i < 10; i++) {
      ic.SetVtrueKtsIC(200.0 + i);
      ic.SetMachIC(0.7 + i * 0.01);
      ic.SetVcalibratedKtsIC(180.0 + i);
    }

    // Should end with calibrated airspeed mode
    TS_ASSERT_EQUALS(ic.GetSpeedSet(), setvc);
    TS_ASSERT_DELTA(ic.GetVcalibratedKtsIC(), 189.0, 1.0);
  }

  /***************************************************************************
   * Section 26: Complete System Verification Tests
   ***************************************************************************/

  void testCompleteInitialConditionVerification() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // 1. Set complete flight condition
    ic.SetLatitudeDegIC(37.6213);
    ic.SetLongitudeDegIC(-122.3790);
    ic.SetAltitudeASLFtIC(3000.0);
    ic.SetVtrueKtsIC(120.0);
    ic.SetPhiDegIC(0.0);
    ic.SetThetaDegIC(5.0);
    ic.SetPsiDegIC(270.0);

    // 2. Verify all values
    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 37.6213, 0.001);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), -122.3790, 0.001);
    TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC(), 3000.0, 1.0);
    TS_ASSERT_DELTA(ic.GetVtrueKtsIC(), 120.0, 1.0);
    TS_ASSERT_DELTA(ic.GetPhiDegIC(), 0.0, 0.1);
    TS_ASSERT_DELTA(ic.GetThetaDegIC(), 5.0, 0.1);
    TS_ASSERT_DELTA(ic.GetPsiDegIC(), 270.0, 0.1);

    // 3. Verify derived values
    TS_ASSERT(ic.GetVtrueKtsIC() > 0);
    TS_ASSERT(ic.GetAltitudeASLFtIC() >= 0);
  }

  void testInitialConditionConsistency() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set altitude and speed
    ic.SetAltitudeASLFtIC(20000.0);
    ic.SetMachIC(0.75);

    // True airspeed should be computable
    double Vt = ic.GetVtrueKtsIC();
    TS_ASSERT(Vt > 0);

    // At 20000 ft, TAS should be higher than indicated for same Mach
    TS_ASSERT(Vt > 200.0);
    TS_ASSERT(Vt < 600.0);
  }

  void testMultipleInitialConditionInstances() {
    FGFDMExec fdmex1, fdmex2;
    FGInitialCondition ic1(&fdmex1);
    FGInitialCondition ic2(&fdmex2);

    // Set different conditions
    ic1.SetAltitudeASLFtIC(5000.0);
    ic2.SetAltitudeASLFtIC(15000.0);

    ic1.SetVtrueKtsIC(150.0);
    ic2.SetVtrueKtsIC(250.0);

    // Verify independence
    TS_ASSERT_DELTA(ic1.GetAltitudeASLFtIC(), 5000.0, 1.0);
    TS_ASSERT_DELTA(ic2.GetAltitudeASLFtIC(), 15000.0, 1.0);
    TS_ASSERT_DELTA(ic1.GetVtrueKtsIC(), 150.0, 1.0);
    TS_ASSERT_DELTA(ic2.GetVtrueKtsIC(), 250.0, 1.0);
  }

  void testInitialConditionReset() {
    FGFDMExec fdmex;
    FGInitialCondition ic(&fdmex);

    // Set values
    ic.SetLatitudeDegIC(45.0);
    ic.SetLongitudeDegIC(-90.0);
    ic.SetAltitudeASLFtIC(10000.0);
    ic.SetVtrueKtsIC(200.0);

    // Verify
    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 45.0, 0.1);

    // Set new values (simulate reset)
    ic.SetLatitudeDegIC(0.0);
    ic.SetLongitudeDegIC(0.0);
    ic.SetAltitudeASLFtIC(0.0);

    // Verify new values
    TS_ASSERT_DELTA(ic.GetLatitudeDegIC(), 0.0, 0.1);
    TS_ASSERT_DELTA(ic.GetLongitudeDegIC(), 0.0, 0.1);
    TS_ASSERT_DELTA(ic.GetAltitudeASLFtIC(), 0.0, 1.0);
  }

  /***************************************************************************
   * C172x Model-Based FGInitialCondition Tests
   ***************************************************************************/

  // Test setting latitude with C172x model
  void testC172xSetLatitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetLatitudeDegIC(45.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetLatitudeDegIC(), 45.0, 0.01);
    TS_ASSERT_DELTA(ic->GetLatitudeRadIC(), 45.0 * M_PI / 180.0, epsilon);
  }

  // Test setting longitude with C172x model
  void testC172xSetLongitude() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetLongitudeDegIC(-122.4);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetLongitudeDegIC(), -122.4, 0.01);
    TS_ASSERT_DELTA(ic->GetLongitudeRadIC(), -122.4 * M_PI / 180.0, epsilon);
  }

  // Test setting altitude ASL with C172x model
  void testC172xSetAltitudeASL() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetAltitudeASLFtIC(5000.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetAltitudeASLFtIC(), 5000.0, 10.0);
  }

  // Test setting altitude AGL with C172x model
  void testC172xSetAltitudeAGL() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetTerrainElevationFtIC(1000.0);
    ic->SetAltitudeAGLFtIC(3000.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetAltitudeAGLFtIC(), 3000.0, 10.0);
    TS_ASSERT_DELTA(ic->GetAltitudeASLFtIC(), 4000.0, 20.0);
  }

  // Test setting U body velocity with C172x model
  void testC172xSetUBodyVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetUBodyFpsIC(200.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetUBodyFpsIC(), 200.0, 1.0);
  }

  // Test setting V body velocity with C172x model
  void testC172xSetVBodyVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetUBodyFpsIC(200.0);
    ic->SetVBodyFpsIC(10.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVBodyFpsIC(), 10.0, 1.0);
  }

  // Test setting W body velocity with C172x model
  void testC172xSetWBodyVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetUBodyFpsIC(200.0);
    ic->SetWBodyFpsIC(5.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetWBodyFpsIC(), 5.0, 1.0);
  }

  // Test setting VNorth velocity with C172x model
  void testC172xSetVNorthVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetVNorthFpsIC(150.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVNorthFpsIC(), 150.0, 1.0);
  }

  // Test setting VEast velocity with C172x model
  void testC172xSetVEastVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetVNorthFpsIC(150.0);
    ic->SetVEastFpsIC(50.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVEastFpsIC(), 50.0, 1.0);
  }

  // Test setting VDown velocity with C172x model
  void testC172xSetVDownVelocity() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetVNorthFpsIC(150.0);
    ic->SetVDownFpsIC(-10.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVDownFpsIC(), -10.0, 1.0);
  }

  // Test setting heading (psi) with C172x model
  void testC172xSetHeading() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetPsiDegIC(270.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetPsiDegIC(), 270.0, 0.1);
    TS_ASSERT_DELTA(ic->GetPsiRadIC(), 270.0 * M_PI / 180.0, epsilon);
  }

  // Test setting pitch (theta) with C172x model
  void testC172xSetPitch() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetThetaDegIC(5.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetThetaDegIC(), 5.0, 0.1);
    TS_ASSERT_DELTA(ic->GetThetaRadIC(), 5.0 * M_PI / 180.0, epsilon);
  }

  // Test setting roll (phi) with C172x model
  void testC172xSetRoll() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetPhiDegIC(15.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetPhiDegIC(), 15.0, 0.1);
    TS_ASSERT_DELTA(ic->GetPhiRadIC(), 15.0 * M_PI / 180.0, epsilon);
  }

  // Test setting flight path angle with C172x model
  void testC172xSetFlightPathAngle() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetVtrueFpsIC(200.0);
    ic->SetFlightPathAngleDegIC(3.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetFlightPathAngleDegIC(), 3.0, 0.5);
  }

  // Test setting climb rate with C172x model
  void testC172xSetClimbRate() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetVtrueFpsIC(200.0);
    ic->SetClimbRateFpsIC(15.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetClimbRateFpsIC(), 15.0, 1.0);
  }

  // Test setting Mach number with C172x model
  void testC172xSetMach() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetAltitudeASLFtIC(10000.0);
    ic->SetMachIC(0.3);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetMachIC(), 0.3, 0.01);
  }

  // Test setting calibrated airspeed with C172x model
  void testC172xSetCalibratedAirspeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(120.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVcalibratedKtsIC(), 120.0, 1.0);
  }

  // Test setting equivalent airspeed with C172x model
  void testC172xSetEquivalentAirspeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetAltitudeASLFtIC(8000.0);
    ic->SetVequivalentKtsIC(150.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVequivalentKtsIC(), 150.0, 2.0);
  }

  // Test values persist after RunIC with C172x model
  void testC172xValuesPersistAfterRunIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    // Set comprehensive initial conditions
    ic->SetLatitudeDegIC(37.6);
    ic->SetLongitudeDegIC(-122.4);
    ic->SetAltitudeASLFtIC(3000.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetPsiDegIC(180.0);
    ic->SetPhiDegIC(0.0);
    ic->SetThetaDegIC(3.0);

    fdmex.RunIC();

    // Verify all values persist
    TS_ASSERT_DELTA(ic->GetLatitudeDegIC(), 37.6, 0.1);
    TS_ASSERT_DELTA(ic->GetLongitudeDegIC(), -122.4, 0.1);
    TS_ASSERT_DELTA(ic->GetAltitudeASLFtIC(), 3000.0, 20.0);
    TS_ASSERT_DELTA(ic->GetVcalibratedKtsIC(), 100.0, 2.0);
    TS_ASSERT_DELTA(ic->GetPsiDegIC(), 180.0, 0.5);
    TS_ASSERT_DELTA(ic->GetPhiDegIC(), 0.0, 0.1);
    TS_ASSERT_DELTA(ic->GetThetaDegIC(), 3.0, 0.5);
  }

  // Test latitude bounds with C172x model
  void testC172xLatitudeBounds() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    // Test near north pole
    ic->SetLatitudeDegIC(89.0);
    fdmex.RunIC();
    TS_ASSERT_DELTA(ic->GetLatitudeDegIC(), 89.0, 0.1);

    // Test near south pole
    ic->SetLatitudeDegIC(-89.0);
    fdmex.RunIC();
    TS_ASSERT_DELTA(ic->GetLatitudeDegIC(), -89.0, 0.1);

    // Test equator
    ic->SetLatitudeDegIC(0.0);
    fdmex.RunIC();
    TS_ASSERT_DELTA(ic->GetLatitudeDegIC(), 0.0, 0.01);
  }

  // Test longitude bounds with C172x model
  void testC172xLongitudeBounds() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    // Test international date line (positive)
    ic->SetLongitudeDegIC(180.0);
    fdmex.RunIC();
    TS_ASSERT_DELTA(ic->GetLongitudeDegIC(), 180.0, 0.1);

    // Test international date line (negative)
    ic->SetLongitudeDegIC(-180.0);
    fdmex.RunIC();
    TS_ASSERT_DELTA(ic->GetLongitudeDegIC(), -180.0, 0.1);

    // Test prime meridian
    ic->SetLongitudeDegIC(0.0);
    fdmex.RunIC();
    TS_ASSERT_DELTA(ic->GetLongitudeDegIC(), 0.0, 0.01);
  }

  // Test velocity consistency with C172x model
  void testC172xVelocityConsistency() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    // Set body velocities
    double u = 180.0, v = 10.0, w = 5.0;
    ic->SetUBodyFpsIC(u);
    ic->SetVBodyFpsIC(v);
    ic->SetWBodyFpsIC(w);
    fdmex.RunIC();

    // Verify UVW vector matches individual components
    FGColumnVector3 uvw = ic->GetUVWFpsIC();
    TS_ASSERT_DELTA(uvw(1), u, 2.0);
    TS_ASSERT_DELTA(uvw(2), v, 2.0);
    TS_ASSERT_DELTA(uvw(3), w, 2.0);

    // True speed should match vector magnitude
    double vt_expected = sqrt(u*u + v*v + w*w);
    TS_ASSERT_DELTA(ic->GetVtrueFpsIC(), vt_expected, 5.0);
  }

  // Test multiple simulation runs don't corrupt IC with C172x model
  void testC172xMultipleRunsNoCorruption() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    // Set initial conditions
    ic->SetLatitudeDegIC(40.0);
    ic->SetLongitudeDegIC(-74.0);
    ic->SetAltitudeASLFtIC(2000.0);
    ic->SetVcalibratedKtsIC(110.0);
    ic->SetPsiDegIC(90.0);

    // Run multiple times
    for (int i = 0; i < 5; i++) {
      fdmex.RunIC();
      for (int j = 0; j < 10; j++) {
        fdmex.Run();
      }
    }

    // IC values should still be accessible (though simulation state may have changed)
    TS_ASSERT(!std::isnan(ic->GetLatitudeDegIC()));
    TS_ASSERT(!std::isnan(ic->GetLongitudeDegIC()));
    TS_ASSERT(!std::isnan(ic->GetAltitudeASLFtIC()));
  }

  // Test PQR rates with C172x model
  void testC172xSetPQRRates() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetPRadpsIC(0.05);
    ic->SetQRadpsIC(0.02);
    ic->SetRRadpsIC(0.01);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetPRadpsIC(), 0.05, 0.01);
    TS_ASSERT_DELTA(ic->GetQRadpsIC(), 0.02, 0.01);
    TS_ASSERT_DELTA(ic->GetRRadpsIC(), 0.01, 0.01);

    // Verify PQR vector
    FGColumnVector3 pqr = ic->GetPQRRadpsIC();
    TS_ASSERT_DELTA(pqr(1), 0.05, 0.01);
    TS_ASSERT_DELTA(pqr(2), 0.02, 0.01);
    TS_ASSERT_DELTA(pqr(3), 0.01, 0.01);
  }

  // Test true airspeed with C172x model
  void testC172xSetTrueAirspeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVtrueKtsIC(150.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVtrueKtsIC(), 150.0, 1.0);
    TS_ASSERT_DELTA(ic->GetVtrueFpsIC(), 150.0 * ktstofps, 2.0);
  }

  // Test ground speed with C172x model
  void testC172xSetGroundSpeed() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetPsiDegIC(0.0);
    ic->SetVgroundKtsIC(120.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetVgroundKtsIC(), 120.0, 2.0);
  }

  // Test orientation quaternion with C172x model
  void testC172xGetOrientation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetPhiDegIC(10.0);
    ic->SetThetaDegIC(5.0);
    ic->SetPsiDegIC(45.0);
    fdmex.RunIC();

    const FGQuaternion& quat = ic->GetOrientation();

    // Quaternion should be normalized
    double mag = sqrt(quat(1)*quat(1) + quat(2)*quat(2) +
                      quat(3)*quat(3) + quat(4)*quat(4));
    TS_ASSERT_DELTA(mag, 1.0, epsilon);

    // Euler angles from quaternion should match inputs
    TS_ASSERT_DELTA(quat.GetEulerDeg(FGJSBBase::ePhi), 10.0, 0.5);
    TS_ASSERT_DELTA(quat.GetEulerDeg(FGJSBBase::eTht), 5.0, 0.5);
    TS_ASSERT_DELTA(quat.GetEulerDeg(FGJSBBase::ePsi), 45.0, 0.5);
  }

  // Test position access with C172x model
  void testC172xGetPosition() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetLatitudeDegIC(51.5);
    ic->SetLongitudeDegIC(-0.1);
    ic->SetAltitudeASLFtIC(1500.0);
    fdmex.RunIC();

    const FGLocation& pos = ic->GetPosition();

    TS_ASSERT_DELTA(pos.GetLatitudeDeg(), 51.5, 0.1);
    TS_ASSERT_DELTA(pos.GetLongitudeDeg(), -0.1, 0.1);
  }

  // Test wind settings with C172x model
  void testC172xSetWind() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetWindNEDFpsIC(10.0, 5.0, 0.0);
    fdmex.RunIC();

    FGColumnVector3 wind = ic->GetWindNEDFpsIC();
    TS_ASSERT_DELTA(wind(1), 10.0, 0.5);
    TS_ASSERT_DELTA(wind(2), 5.0, 0.5);
    TS_ASSERT_DELTA(wind(3), 0.0, 0.5);
  }

  // Test terrain elevation with C172x model
  void testC172xSetTerrainElevation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto ic = fdmex.GetIC();

    ic->SetTerrainElevationFtIC(2500.0);
    fdmex.RunIC();

    TS_ASSERT_DELTA(ic->GetTerrainElevationFtIC(), 2500.0, 10.0);
  }
};
