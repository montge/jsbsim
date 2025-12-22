#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <math/FGColumnVector3.h>
#include <math/FGQuaternion.h>
#include <math/FGLocation.h>
#include "TestAssertions.h"
#include "TestUtilities.h"

using namespace JSBSim;

const double epsilon = 1e-8;

class FGPropagateTest : public CxxTest::TestSuite
{
public:
  // Test default construction
  void testConstruction() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    TS_ASSERT(propagate != nullptr);
  }

  // Test InitModel
  void testInitModel() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    bool result = propagate->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  // Test body velocity getters
  void testBodyVelocity() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 uvw = propagate->GetUVW();
    TS_ASSERT(!std::isnan(uvw(1)));
    TS_ASSERT(!std::isnan(uvw(2)));
    TS_ASSERT(!std::isnan(uvw(3)));

    // Test indexed accessor
    TS_ASSERT_DELTA(propagate->GetUVW(1), uvw(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetUVW(2), uvw(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetUVW(3), uvw(3), epsilon);
  }

  // Test local velocity getters
  void testLocalVelocity() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 vel = propagate->GetVel();
    TS_ASSERT(!std::isnan(vel(1)));
    TS_ASSERT(!std::isnan(vel(2)));
    TS_ASSERT(!std::isnan(vel(3)));

    // Test indexed accessor
    TS_ASSERT_DELTA(propagate->GetVel(1), vel(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetVel(2), vel(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetVel(3), vel(3), epsilon);
  }

  // Test angular rate getters (PQR relative to ECEF)
  void testAngularRates() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 pqr = propagate->GetPQR();
    TS_ASSERT(!std::isnan(pqr(1)));
    TS_ASSERT(!std::isnan(pqr(2)));
    TS_ASSERT(!std::isnan(pqr(3)));

    // Test indexed accessor
    TS_ASSERT_DELTA(propagate->GetPQR(1), pqr(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetPQR(2), pqr(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetPQR(3), pqr(3), epsilon);
  }

  // Test angular rate getters (PQRi relative to ECI)
  void testInertialAngularRates() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 pqri = propagate->GetPQRi();
    TS_ASSERT(!std::isnan(pqri(1)));
    TS_ASSERT(!std::isnan(pqri(2)));
    TS_ASSERT(!std::isnan(pqri(3)));

    // Test indexed accessor
    TS_ASSERT_DELTA(propagate->GetPQRi(1), pqri(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetPQRi(2), pqri(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetPQRi(3), pqri(3), epsilon);
  }

  // Test Euler angle getters
  void testEulerAngles() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 euler = propagate->GetEuler();
    FGColumnVector3 eulerDeg = propagate->GetEulerDeg();

    TS_ASSERT(!std::isnan(euler(1)));
    TS_ASSERT(!std::isnan(euler(2)));
    TS_ASSERT(!std::isnan(euler(3)));

    // Verify radians to degrees conversion
    TS_ASSERT_DELTA(eulerDeg(1), euler(1) * 180.0 / M_PI, 1e-6);
    TS_ASSERT_DELTA(eulerDeg(2), euler(2) * 180.0 / M_PI, 1e-6);
    TS_ASSERT_DELTA(eulerDeg(3), euler(3) * 180.0 / M_PI, 1e-6);
  }

  // Test altitude getters
  void testAltitude() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double altFt = propagate->GetAltitudeASL();
    double altM = propagate->GetAltitudeASLmeters();

    TS_ASSERT(!std::isnan(altFt));
    TS_ASSERT(!std::isnan(altM));

    // Verify feet to meters conversion (1 ft = 0.3048 m)
    TS_ASSERT_DELTA(altM, altFt * 0.3048, 0.01);
  }

  // Test inertial velocity
  void testInertialVelocity() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 vInertial = propagate->GetInertialVelocity();
    double vMag = propagate->GetInertialVelocityMagnitude();

    TS_ASSERT(!std::isnan(vInertial(1)));
    TS_ASSERT(!std::isnan(vInertial(2)));
    TS_ASSERT(!std::isnan(vInertial(3)));
    TS_ASSERT(!std::isnan(vMag));

    // Magnitude should equal sqrt(sum of squares)
    double expectedMag = sqrt(vInertial(1)*vInertial(1) +
                              vInertial(2)*vInertial(2) +
                              vInertial(3)*vInertial(3));
    TS_ASSERT_DELTA(vMag, expectedMag, epsilon);
  }

  // Test inertial position
  void testInertialPosition() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 posInertial = propagate->GetInertialPosition();

    TS_ASSERT(!std::isnan(posInertial(1)));
    TS_ASSERT(!std::isnan(posInertial(2)));
    TS_ASSERT(!std::isnan(posInertial(3)));

    // Test indexed accessor
    TS_ASSERT_DELTA(propagate->GetInertialPosition(1), posInertial(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetInertialPosition(2), posInertial(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetInertialPosition(3), posInertial(3), epsilon);
  }

  // Test ECEF velocity
  void testECEFVelocity() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 vECEF = propagate->GetECEFVelocity();

    TS_ASSERT(!std::isnan(vECEF(1)));
    TS_ASSERT(!std::isnan(vECEF(2)));
    TS_ASSERT(!std::isnan(vECEF(3)));

    // Test indexed accessor
    TS_ASSERT_DELTA(propagate->GetECEFVelocity(1), vECEF(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetECEFVelocity(2), vECEF(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetECEFVelocity(3), vECEF(3), epsilon);
  }

  // Test NED velocity magnitude
  void testNEDVelocityMagnitude() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double nedMag = propagate->GetNEDVelocityMagnitude();
    TS_ASSERT(!std::isnan(nedMag));
    TS_ASSERT(nedMag >= 0.0);
  }

  // Test quaternion derivative getter
  void testQuaternionDerivative() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGQuaternion& qdot = propagate->GetQuaterniondot();

    TS_ASSERT(!std::isnan(qdot(1)));
    TS_ASSERT(!std::isnan(qdot(2)));
    TS_ASSERT(!std::isnan(qdot(3)));
    TS_ASSERT(!std::isnan(qdot(4)));
  }

  // Test Run returns false (no error)
  void testRun() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    bool result = propagate->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test Run with holding
  void testRunHolding() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    bool result = propagate->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test multiple Run calls maintain valid state
  void testMultipleRuns() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    for (int i = 0; i < 10; i++) {
      propagate->Run(false);

      // State should remain valid
      FGColumnVector3 uvw = propagate->GetUVW();
      TS_ASSERT(!std::isnan(uvw(1)));
      TS_ASSERT(!std::isnan(uvw(2)));
      TS_ASSERT(!std::isnan(uvw(3)));

      FGColumnVector3 euler = propagate->GetEuler();
      TS_ASSERT(!std::isnan(euler(1)));
      TS_ASSERT(!std::isnan(euler(2)));
      TS_ASSERT(!std::isnan(euler(3)));
    }
  }

  // Test location getter
  void testLocation() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGLocation& loc = propagate->GetLocation();

    // Location should be valid
    double lat = loc.GetLatitude();
    double lon = loc.GetLongitude();
    double radius = loc.GetRadius();

    TS_ASSERT(!std::isnan(lat));
    TS_ASSERT(!std::isnan(lon));
    TS_ASSERT(!std::isnan(radius));

    // Latitude should be in [-pi/2, pi/2]
    TS_ASSERT(lat >= -M_PI/2.0 && lat <= M_PI/2.0);
    // Longitude should be in [-pi, pi]
    TS_ASSERT(lon >= -M_PI && lon <= M_PI);
    // Radius should be positive (Earth radius ~20 million ft)
    TS_ASSERT(radius > 0.0);
  }

  // Test altitude above ground level
  void testAltitudeAGL() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double agl = propagate->GetDistanceAGL();
    TS_ASSERT(!std::isnan(agl));
  }

  // Test geodetic altitude
  void testGeodeticAltitude() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double geoAlt = propagate->GetGeodeticAltitude();
    TS_ASSERT(!std::isnan(geoAlt));
  }

  // Test latitude/longitude in degrees
  void testLatLonDegrees() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double latDeg = propagate->GetLatitudeDeg();
    double lonDeg = propagate->GetLongitudeDeg();

    TS_ASSERT(!std::isnan(latDeg));
    TS_ASSERT(!std::isnan(lonDeg));

    // Latitude in degrees should be in [-90, 90]
    TS_ASSERT(latDeg >= -90.0 && latDeg <= 90.0);
    // Longitude in degrees should be in [-180, 180]
    TS_ASSERT(lonDeg >= -180.0 && lonDeg <= 180.0);
  }

  // Test terrain elevation getter
  void testTerrainElevation() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double terrainElev = propagate->GetTerrainElevation();
    TS_ASSERT(!std::isnan(terrainElev));
  }

  // Test transformation matrices
  void testTransformationMatrices() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Get local-to-body transformation
    const FGMatrix33& Tl2b = propagate->GetTl2b();
    const FGMatrix33& Tb2l = propagate->GetTb2l();

    // These should be inverses of each other
    FGMatrix33 product = Tl2b * Tb2l;
    TS_ASSERT_DELTA(product(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(3,3), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(1,2), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(1,3), 0.0, 1e-6);
    TS_ASSERT_DELTA(product(2,3), 0.0, 1e-6);
  }

  // Test ECI-to-body transformation matrices
  void testECITransformationMatrices() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Ti2b = propagate->GetTi2b();
    const FGMatrix33& Tb2i = propagate->GetTb2i();

    // These should be inverses
    FGMatrix33 product = Ti2b * Tb2i;
    TS_ASSERT_DELTA(product(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(3,3), 1.0, 1e-6);
  }

  // Test ECEF-to-body transformation matrices
  void testECEFTransformationMatrices() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Tec2b = propagate->GetTec2b();
    const FGMatrix33& Tb2ec = propagate->GetTb2ec();

    // These should be inverses
    FGMatrix33 product = Tec2b * Tb2ec;
    TS_ASSERT_DELTA(product(1,1), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(2,2), 1.0, 1e-6);
    TS_ASSERT_DELTA(product(3,3), 1.0, 1e-6);
  }

  // Test quaternion getters
  void testQuaternionGetters() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGQuaternion& qLocal = propagate->GetQuaternion();
    const FGQuaternion& qECI = propagate->GetQuaternionECI();

    // Quaternions should be unit quaternions (magnitude = 1)
    double magLocal = sqrt(qLocal(1)*qLocal(1) + qLocal(2)*qLocal(2) +
                           qLocal(3)*qLocal(3) + qLocal(4)*qLocal(4));
    double magECI = sqrt(qECI(1)*qECI(1) + qECI(2)*qECI(2) +
                         qECI(3)*qECI(3) + qECI(4)*qECI(4));

    TS_ASSERT_DELTA(magLocal, 1.0, 1e-6);
    TS_ASSERT_DELTA(magECI, 1.0, 1e-6);
  }

  // Test simulation time
  void testSimulationTime() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double simTime = propagate->GetSimTime();
    TS_ASSERT(!std::isnan(simTime));
    TS_ASSERT(simTime >= 0.0);
  }

  // Test cosine/sine of Euler angles
  void testEulerTrigFunctions() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double cPhi = propagate->GetCosEuler(1);
    double sPhi = propagate->GetSinEuler(1);
    double cTheta = propagate->GetCosEuler(2);
    double sTheta = propagate->GetSinEuler(2);
    double cPsi = propagate->GetCosEuler(3);
    double sPsi = propagate->GetSinEuler(3);

    // cos^2 + sin^2 = 1
    TS_ASSERT_DELTA(cPhi*cPhi + sPhi*sPhi, 1.0, 1e-10);
    TS_ASSERT_DELTA(cTheta*cTheta + sTheta*sTheta, 1.0, 1e-10);
    TS_ASSERT_DELTA(cPsi*cPsi + sPsi*sPsi, 1.0, 1e-10);
  }

  // Test integrator type enum values
  void testIntegratorEnums() {
    // Verify enum values are as expected
    TS_ASSERT_EQUALS(FGPropagate::eNone, 0);
    TS_ASSERT_EQUALS(FGPropagate::eRectEuler, 1);
    TS_ASSERT_EQUALS(FGPropagate::eTrapezoidal, 2);
    TS_ASSERT_EQUALS(FGPropagate::eAdamsBashforth2, 3);
    TS_ASSERT_EQUALS(FGPropagate::eAdamsBashforth3, 4);
    TS_ASSERT_EQUALS(FGPropagate::eAdamsBashforth4, 5);
  }
};
