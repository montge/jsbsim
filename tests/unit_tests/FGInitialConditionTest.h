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
};
