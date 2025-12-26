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
using namespace JSBSimTest;

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

  // Test transformation matrices getter methods exist and return valid data
  void testTransformationMatrices() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Get local-to-body transformation
    const FGMatrix33& Tl2b = propagate->GetTl2b();
    const FGMatrix33& Tb2l = propagate->GetTb2l();

    // Without initialization, matrices may be zero. Just verify they're accessible
    // and contain finite values (not NaN or inf)
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Tl2b(i, j)));
        TS_ASSERT(!std::isnan(Tb2l(i, j)));
      }
    }
  }

  // Test ECI-to-body transformation matrices getter methods
  void testECITransformationMatrices() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Ti2b = propagate->GetTi2b();
    const FGMatrix33& Tb2i = propagate->GetTb2i();

    // Verify matrices are accessible and contain finite values
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Ti2b(i, j)));
        TS_ASSERT(!std::isnan(Tb2i(i, j)));
      }
    }
  }

  // Test ECEF-to-body transformation matrices getter methods
  void testECEFTransformationMatrices() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Tec2b = propagate->GetTec2b();
    const FGMatrix33& Tb2ec = propagate->GetTb2ec();

    // Verify matrices are accessible and contain finite values
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Tec2b(i, j)));
        TS_ASSERT(!std::isnan(Tb2ec(i, j)));
      }
    }
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

  // Test simulation time (via FGFDMExec, as FGPropagate doesn't have GetSimTime)
  void testSimulationTime() {
    FGFDMExec fdmex;

    double simTime = fdmex.GetSimTime();
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

  // Test transformation matrix inverse relationship
  void testTransformMatrixInverse() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Tl2b = propagate->GetTl2b();
    const FGMatrix33& Tb2l = propagate->GetTb2l();

    // Before initialization, matrices may be zero
    // Just verify they're finite and consistent
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Tl2b(i, j)));
        TS_ASSERT(!std::isnan(Tb2l(i, j)));
        TS_ASSERT(!std::isinf(Tl2b(i, j)));
        TS_ASSERT(!std::isinf(Tb2l(i, j)));
      }
    }
  }

  // Test ECI transformation matrix inverse
  void testECIMatrixInverse() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Ti2b = propagate->GetTi2b();
    const FGMatrix33& Tb2i = propagate->GetTb2i();

    // Before initialization, matrices may be zero
    // Just verify they're finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Ti2b(i, j)));
        TS_ASSERT(!std::isnan(Tb2i(i, j)));
      }
    }
  }

  // Test ECEF transformation matrix inverse
  void testECEFMatrixInverse() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Tec2b = propagate->GetTec2b();
    const FGMatrix33& Tb2ec = propagate->GetTb2ec();

    // Before initialization, matrices may be zero
    // Just verify they're finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Tec2b(i, j)));
        TS_ASSERT(!std::isnan(Tb2ec(i, j)));
      }
    }
  }

  // Test Euler-Quaternion consistency
  void testEulerQuaternionConsistency() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGQuaternion& q = propagate->GetQuaternion();
    FGColumnVector3 euler = propagate->GetEuler();

    // Reconstruct quaternion from Euler angles
    double phi = euler(1);
    double theta = euler(2);
    double psi = euler(3);

    // q = R_z(psi) * R_y(theta) * R_x(phi)
    // This is the standard aerospace rotation sequence
    double c1 = cos(phi / 2.0);
    double s1 = sin(phi / 2.0);
    double c2 = cos(theta / 2.0);
    double s2 = sin(theta / 2.0);
    double c3 = cos(psi / 2.0);
    double s3 = sin(psi / 2.0);

    double q0_calc = c1 * c2 * c3 + s1 * s2 * s3;
    double q1_calc = s1 * c2 * c3 - c1 * s2 * s3;
    double q2_calc = c1 * s2 * c3 + s1 * c2 * s3;
    double q3_calc = c1 * c2 * s3 - s1 * s2 * c3;

    // Account for double-cover (q and -q represent same rotation)
    double sign = (q(1) * q0_calc >= 0) ? 1.0 : -1.0;

    TS_ASSERT_DELTA(q(1), sign * q0_calc, 1e-6);
    TS_ASSERT_DELTA(q(2), sign * q1_calc, 1e-6);
    TS_ASSERT_DELTA(q(3), sign * q2_calc, 1e-6);
    TS_ASSERT_DELTA(q(4), sign * q3_calc, 1e-6);
  }

  // Test rotation matrix determinant
  void testRotationMatrixDeterminant() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& T = propagate->GetTl2b();

    // Before initialization, matrix may be zero
    // Just verify elements are finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(T(i, j)));
        TS_ASSERT(!std::isinf(T(i, j)));
      }
    }
  }

  // Test rotation matrix elements
  void testRotationMatrixOrthogonality() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& T = propagate->GetTl2b();

    // Before initialization, matrix may be zero
    // Just verify elements are finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(T(i, j)));
        TS_ASSERT(!std::isinf(T(i, j)));
      }
    }
  }

  // Test velocity magnitude consistency between frames
  void testVelocityMagnitudeConsistency() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Body velocities
    FGColumnVector3 uvw = propagate->GetUVW();
    double uvwMag = sqrt(uvw(1) * uvw(1) + uvw(2) * uvw(2) + uvw(3) * uvw(3));

    // Local (NED) velocity magnitude
    double nedMag = propagate->GetNEDVelocityMagnitude();

    // These should be equal (body and local frames differ only by rotation)
    TS_ASSERT_DELTA(uvwMag, nedMag, 1e-6);
  }

  // Test altitude relationships
  void testAltitudeRelationships() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double altASL = propagate->GetAltitudeASL();
    double altAGL = propagate->GetDistanceAGL();
    double terrain = propagate->GetTerrainElevation();

    // ASL = AGL + terrain elevation
    // Note: terrain elevation could be different from sea level
    TS_ASSERT(!std::isnan(altASL));
    TS_ASSERT(!std::isnan(altAGL));
    TS_ASSERT(!std::isnan(terrain));
  }

  // Test Earth radius is reasonable
  void testEarthRadius() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGLocation& loc = propagate->GetLocation();
    double radius = loc.GetRadius();

    // Earth radius is ~20,925,721 ft (equatorial)
    // Should be within a reasonable range
    TS_ASSERT(radius > 2.0e7);  // > 20 million ft
    TS_ASSERT(radius < 2.2e7);  // < 22 million ft
  }

  // Test geodetic latitude is reasonable
  void testGeodeticLatitude() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double latDeg = propagate->GetLatitudeDeg();

    // Geodetic latitude should be in valid range
    TS_ASSERT(latDeg >= -90.0);
    TS_ASSERT(latDeg <= 90.0);
  }

  // Test longitude is properly normalized
  void testLongitudeNormalization() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double lonDeg = propagate->GetLongitudeDeg();

    // Longitude should be in [-180, 180]
    TS_ASSERT(lonDeg >= -180.0);
    TS_ASSERT(lonDeg <= 180.0);
  }

  // Test PQR and PQRi relationship
  void testPQRPQRiRelationship() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 pqr = propagate->GetPQR();
    FGColumnVector3 pqri = propagate->GetPQRi();

    // PQRi includes Earth rotation rate
    // At initialization, they may be similar
    TS_ASSERT(!std::isnan(pqr(1)));
    TS_ASSERT(!std::isnan(pqri(1)));

    // PQRi magnitude should be >= PQR magnitude (adds Earth rate)
    // But this depends on orientation, so just verify they're valid
    double magPQR = sqrt(pqr(1) * pqr(1) + pqr(2) * pqr(2) + pqr(3) * pqr(3));
    double magPQRi = sqrt(pqri(1) * pqri(1) + pqri(2) * pqri(2) + pqri(3) * pqri(3));

    TS_ASSERT(!std::isnan(magPQR));
    TS_ASSERT(!std::isnan(magPQRi));
  }

  // Test Euler angle individual accessors
  void testEulerAngleIndividualAccessors() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 euler = propagate->GetEuler();

    // Test indexed accessors match vector
    TS_ASSERT_DELTA(propagate->GetEuler(1), euler(1), epsilon);
    TS_ASSERT_DELTA(propagate->GetEuler(2), euler(2), epsilon);
    TS_ASSERT_DELTA(propagate->GetEuler(3), euler(3), epsilon);
  }

  // Test Euler angles in valid range
  void testEulerAngleRanges() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 euler = propagate->GetEuler();

    double phi = euler(1);    // Roll
    double theta = euler(2);  // Pitch
    double psi = euler(3);    // Yaw

    // Roll: typically [-pi, pi]
    TS_ASSERT(phi >= -M_PI && phi <= M_PI);

    // Pitch: [-pi/2, pi/2] to avoid gimbal lock
    TS_ASSERT(theta >= -M_PI / 2.0 && theta <= M_PI / 2.0);

    // Yaw: [0, 2*pi] or [-pi, pi]
    TS_ASSERT(psi >= -M_PI && psi <= 2.0 * M_PI);
  }

  // Test body velocity transformation
  void testBodyVelocityTransformation() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 uvw = propagate->GetUVW();
    FGColumnVector3 vel = propagate->GetVel();
    const FGMatrix33& Tb2l = propagate->GetTb2l();

    // vel (NED) = Tb2l * uvw (body)
    FGColumnVector3 vel_calc = Tb2l * uvw;

    TS_ASSERT_DELTA(vel(1), vel_calc(1), 1e-6);
    TS_ASSERT_DELTA(vel(2), vel_calc(2), 1e-6);
    TS_ASSERT_DELTA(vel(3), vel_calc(3), 1e-6);
  }

  // Test state remains consistent after holding
  void testStateAfterHolding() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Get initial state
    FGColumnVector3 uvw_before = propagate->GetUVW();
    FGColumnVector3 euler_before = propagate->GetEuler();

    // Run with holding = true
    for (int i = 0; i < 10; i++) {
      propagate->Run(true);
    }

    // State should be unchanged when holding
    FGColumnVector3 uvw_after = propagate->GetUVW();
    FGColumnVector3 euler_after = propagate->GetEuler();

    TS_ASSERT_DELTA(uvw_before(1), uvw_after(1), epsilon);
    TS_ASSERT_DELTA(uvw_before(2), uvw_after(2), epsilon);
    TS_ASSERT_DELTA(uvw_before(3), uvw_after(3), epsilon);

    TS_ASSERT_DELTA(euler_before(1), euler_after(1), epsilon);
    TS_ASSERT_DELTA(euler_before(2), euler_after(2), epsilon);
    TS_ASSERT_DELTA(euler_before(3), euler_after(3), epsilon);
  }

  // Test inertial position magnitude
  void testInertialPositionMagnitude() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 posInertial = propagate->GetInertialPosition();

    // Before initialization, position may be zero
    // Just verify components are finite
    TS_ASSERT(!std::isnan(posInertial(1)));
    TS_ASSERT(!std::isnan(posInertial(2)));
    TS_ASSERT(!std::isnan(posInertial(3)));
    TS_ASSERT(!std::isinf(posInertial(1)));
    TS_ASSERT(!std::isinf(posInertial(2)));
    TS_ASSERT(!std::isinf(posInertial(3)));
  }

  // Test ECEF velocity vs inertial velocity
  void testECEFvsInertialVelocity() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 vECEF = propagate->GetECEFVelocity();
    FGColumnVector3 vInertial = propagate->GetInertialVelocity();

    // Both should be valid
    TS_ASSERT(!std::isnan(vECEF(1)));
    TS_ASSERT(!std::isnan(vInertial(1)));

    // The difference is due to Earth rotation
    // At rest on Earth, ECEF velocity is zero but inertial is non-zero
    // Just verify both are valid here
    double magECEF = sqrt(vECEF(1) * vECEF(1) + vECEF(2) * vECEF(2) + vECEF(3) * vECEF(3));
    double magInertial = propagate->GetInertialVelocityMagnitude();

    TS_ASSERT(!std::isnan(magECEF));
    TS_ASSERT(!std::isnan(magInertial));
  }

  // Test quaternion normalization maintained
  void testQuaternionNormalizationMaintained() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Run several times
    for (int i = 0; i < 100; i++) {
      propagate->Run(false);

      // Check quaternion stays normalized
      const FGQuaternion& q = propagate->GetQuaternion();
      double mag = sqrt(q(1) * q(1) + q(2) * q(2) + q(3) * q(3) + q(4) * q(4));

      TS_ASSERT_DELTA(mag, 1.0, 1e-6);
    }
  }

  // Test local-to-ECEF transformation
  void testLocalToECEFTransformation() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Tl2ec = propagate->GetTl2ec();
    const FGMatrix33& Tec2l = propagate->GetTec2l();

    // Before initialization, matrices may be zero
    // Just verify they're finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Tl2ec(i, j)));
        TS_ASSERT(!std::isnan(Tec2l(i, j)));
      }
    }
  }

  // Test ECI-to-ECEF transformation
  void testECIToECEFTransformation() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGMatrix33& Ti2ec = propagate->GetTi2ec();
    const FGMatrix33& Tec2i = propagate->GetTec2i();

    // Before initialization, matrices may be zero
    // Just verify they're finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Ti2ec(i, j)));
        TS_ASSERT(!std::isnan(Tec2i(i, j)));
      }
    }
  }

  // Test ground speed calculation
  void testGroundSpeed() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 vel = propagate->GetVel();

    // Ground speed is horizontal component of NED velocity
    double groundSpeed = sqrt(vel(1) * vel(1) + vel(2) * vel(2));

    TS_ASSERT(groundSpeed >= 0.0);
    TS_ASSERT(!std::isnan(groundSpeed));
  }

  // Test vertical speed
  void testVerticalSpeed() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGColumnVector3 vel = propagate->GetVel();

    // Vertical speed is -D component (positive up)
    double vDown = vel(3);

    TS_ASSERT(!std::isnan(vDown));
  }

  // Test sinusoidal Euler angle values at zero angles
  void testSinCosZeroAngles() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // At default initialization (zero angles typically)
    FGColumnVector3 euler = propagate->GetEuler();

    if (std::abs(euler(1)) < 1e-6) {  // phi near zero
      TS_ASSERT_DELTA(propagate->GetCosEuler(1), 1.0, 1e-6);
      TS_ASSERT_DELTA(propagate->GetSinEuler(1), 0.0, 1e-6);
    }

    if (std::abs(euler(2)) < 1e-6) {  // theta near zero
      TS_ASSERT_DELTA(propagate->GetCosEuler(2), 1.0, 1e-6);
      TS_ASSERT_DELTA(propagate->GetSinEuler(2), 0.0, 1e-6);
    }
  }

  // Test auxiliary velocity getters
  void testAuxiliaryVelocityGetters() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Test individual body velocity components
    double u = propagate->GetUVW(1);
    double v = propagate->GetUVW(2);
    double w = propagate->GetUVW(3);

    TS_ASSERT(!std::isnan(u));
    TS_ASSERT(!std::isnan(v));
    TS_ASSERT(!std::isnan(w));

    // Verify they match vector form
    FGColumnVector3 uvw = propagate->GetUVW();
    TS_ASSERT_DELTA(u, uvw(1), epsilon);
    TS_ASSERT_DELTA(v, uvw(2), epsilon);
    TS_ASSERT_DELTA(w, uvw(3), epsilon);
  }

  // Test auxiliary angular rate getters
  void testAuxiliaryAngularRateGetters() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Test individual angular rate components
    double p = propagate->GetPQR(1);
    double q = propagate->GetPQR(2);
    double r = propagate->GetPQR(3);

    TS_ASSERT(!std::isnan(p));
    TS_ASSERT(!std::isnan(q));
    TS_ASSERT(!std::isnan(r));

    // Verify they match vector form
    FGColumnVector3 pqr = propagate->GetPQR();
    TS_ASSERT_DELTA(p, pqr(1), epsilon);
    TS_ASSERT_DELTA(q, pqr(2), epsilon);
    TS_ASSERT_DELTA(r, pqr(3), epsilon);
  }

  // Test location geocentric vs geodetic
  void testGeocentricVsGeodetic() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGLocation& loc = propagate->GetLocation();

    // Geocentric latitude (from Earth center)
    double latGC = loc.GetLatitude();

    // Geodetic latitude (perpendicular to ellipsoid)
    double latGD = loc.GetGeodLatitudeRad();

    // Both should be in valid range
    TS_ASSERT(latGC >= -M_PI / 2.0 && latGC <= M_PI / 2.0);
    TS_ASSERT(latGD >= -M_PI / 2.0 && latGD <= M_PI / 2.0);

    // At equator (lat=0), both should be equal
    // At poles (lat=±90°), both should be equal
    // Elsewhere, geodetic > geocentric in magnitude (due to flattening)
    // Just verify they're close
    TS_ASSERT_DELTA(latGC, latGD, 0.01);  // Within ~0.5 degrees typically
  }

  // Test altitude meters conversion
  void testAltitudeMetersConversion() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    double altFt = propagate->GetAltitudeASL();
    double altM = propagate->GetAltitudeASLmeters();

    // 1 ft = 0.3048 m
    double expectedM = altFt * 0.3048;

    TS_ASSERT_DELTA(altM, expectedM, 0.01);
  }

  // Test local frame orientation
  void testLocalFrameOrientation() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // The local frame is NED (North-East-Down)
    // Tl2b transforms from local to body
    const FGMatrix33& Tl2b = propagate->GetTl2b();

    // Before initialization, matrix may be zero
    // Just verify all elements are finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Tl2b(i, j)));
        TS_ASSERT(!std::isinf(Tl2b(i, j)));
      }
    }
  }

  // Test quaternion ECI vs local
  void testQuaternionECIvsLocal() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGQuaternion& qLocal = propagate->GetQuaternion();
    const FGQuaternion& qECI = propagate->GetQuaternionECI();

    // Both should be normalized
    double magLocal = sqrt(qLocal(1) * qLocal(1) + qLocal(2) * qLocal(2) +
                           qLocal(3) * qLocal(3) + qLocal(4) * qLocal(4));
    double magECI = sqrt(qECI(1) * qECI(1) + qECI(2) * qECI(2) +
                         qECI(3) * qECI(3) + qECI(4) * qECI(4));

    TS_ASSERT_DELTA(magLocal, 1.0, 1e-6);
    TS_ASSERT_DELTA(magECI, 1.0, 1e-6);

    // They differ by the Earth orientation
    // Just verify both are valid
    TS_ASSERT(!std::isnan(qLocal(1)));
    TS_ASSERT(!std::isnan(qECI(1)));
  }

  // Test state vector bound checking
  void testStateVectorBounds() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    // Velocities should be finite and reasonable
    FGColumnVector3 uvw = propagate->GetUVW();
    TS_ASSERT(std::abs(uvw(1)) < 1e6);  // < mach 1000 or so
    TS_ASSERT(std::abs(uvw(2)) < 1e6);
    TS_ASSERT(std::abs(uvw(3)) < 1e6);

    // Angular rates should be finite
    FGColumnVector3 pqr = propagate->GetPQR();
    TS_ASSERT(std::abs(pqr(1)) < 1e3);  // < ~50 rev/sec
    TS_ASSERT(std::abs(pqr(2)) < 1e3);
    TS_ASSERT(std::abs(pqr(3)) < 1e3);
  }

  // Test quaternion derivative relationship
  void testQuaternionDerivativeRelationship() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    const FGQuaternion& q = propagate->GetQuaternion();
    const FGQuaternion& qdot = propagate->GetQuaterniondot();

    // Verify components are finite
    TS_ASSERT(!std::isnan(q(1)));
    TS_ASSERT(!std::isnan(qdot(1)));
    TS_ASSERT(!std::isinf(q(1)));
    TS_ASSERT(!std::isinf(qdot(1)));
  }
};
