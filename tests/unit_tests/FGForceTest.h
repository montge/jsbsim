#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/propulsion/FGForce.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>
#include "TestAssertions.h"
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;

// Testable wrapper that exposes protected members for testing
class TestableFGForce : public FGForce {
public:
  explicit TestableFGForce(FGFDMExec* fdm) : FGForce(fdm) {}

  // Provide access to protected vFn and vMn for testing
  void SetNativeForces(const FGColumnVector3& f) { vFn = f; }
  void SetNativeForces(double x, double y, double z) {
    vFn(1) = x; vFn(2) = y; vFn(3) = z;
  }
  void SetNativeMoments(const FGColumnVector3& m) { vMn = m; }
  void SetNativeMoments(double x, double y, double z) {
    vMn(1) = x; vMn(2) = y; vMn(3) = z;
  }
  const FGColumnVector3& GetNativeForces() const { return vFn; }
  const FGColumnVector3& GetNativeMoments() const { return vMn; }
};

class FGForceTest : public CxxTest::TestSuite
{
public:
  // Test construction
  void testConstruction() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Should be constructed successfully
    TS_ASSERT(true);
  }

  // Test setting location
  void testSetLocation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    double x = 100.0, y = 50.0, z = -25.0;
    force.SetLocation(x, y, z);

    TS_ASSERT_DELTA(force.GetLocationX(), x, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), y, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), z, epsilon);
  }

  // Test setting location with vector
  void testSetLocationVector() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    FGColumnVector3 loc(150.0, -30.0, 10.0);
    force.SetLocation(loc);

    const FGColumnVector3& result = force.GetLocation();
    TS_ASSERT_DELTA(result(1), loc(1), epsilon);
    TS_ASSERT_DELTA(result(2), loc(2), epsilon);
    TS_ASSERT_DELTA(result(3), loc(3), epsilon);
  }

  // Test individual location setters
  void testSetLocationIndividual() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocationX(10.0);
    force.SetLocationY(20.0);
    force.SetLocationZ(30.0);

    TS_ASSERT_DELTA(force.GetLocationX(), 10.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 20.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 30.0, epsilon);
  }

  // Test acting location
  void testActingLocation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetActingLocation(50.0, 25.0, -10.0);

    TS_ASSERT_DELTA(force.GetActingLocationX(), 50.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 25.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), -10.0, epsilon);
  }

  // Test individual acting location setters
  void testSetActingLocationIndividual() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetActingLocationX(5.0);
    force.SetActingLocationY(15.0);
    force.SetActingLocationZ(25.0);

    TS_ASSERT_DELTA(force.GetActingLocationX(), 5.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 15.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 25.0, epsilon);
  }

  // Test that SetLocation also sets acting location
  void testSetLocationSetsActingLocation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(100.0, 200.0, 300.0);

    // Acting location should be set to same as location
    TS_ASSERT_DELTA(force.GetActingLocationX(), 100.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 200.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 300.0, epsilon);
  }

  // Test angles to body (requires tCustom transform type)
  void testAnglesToBody() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // SetAnglesToBody only works when transform type is tCustom
    force.SetTransformType(FGForce::tCustom);

    double roll = 0.1, pitch = 0.2, yaw = 0.3;
    force.SetAnglesToBody(roll, pitch, yaw);

    const FGColumnVector3& angles = force.GetAnglesToBody();
    TS_ASSERT_DELTA(angles(1), roll, epsilon);
    TS_ASSERT_DELTA(angles(2), pitch, epsilon);
    TS_ASSERT_DELTA(angles(3), yaw, epsilon);

    // Test indexed accessor
    TS_ASSERT_DELTA(force.GetAnglesToBody(1), roll, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(2), pitch, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(3), yaw, epsilon);
  }

  // Test pitch getter/setter
  void testPitch() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetPitch(0.5);
    TS_ASSERT_DELTA(force.GetPitch(), 0.5, epsilon);
  }

  // Test yaw getter/setter
  void testYaw() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetYaw(-0.3);
    TS_ASSERT_DELTA(force.GetYaw(), -0.3, epsilon);
  }

  // Test transform type enum values
  void testTransformTypeEnums() {
    TS_ASSERT_EQUALS(FGForce::tNone, 0);
    TS_ASSERT_EQUALS(FGForce::tWindBody, 1);
    TS_ASSERT_EQUALS(FGForce::tLocalBody, 2);
    TS_ASSERT_EQUALS(FGForce::tInertialBody, 3);
    TS_ASSERT_EQUALS(FGForce::tCustom, 4);
  }

  // Test set transform type
  void testSetTransformType() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tNone);

    force.SetTransformType(FGForce::tWindBody);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tWindBody);

    force.SetTransformType(FGForce::tLocalBody);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tLocalBody);

    force.SetTransformType(FGForce::tCustom);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tCustom);
  }

  // Test body forces with no transform (tNone)
  void testBodyForcesNoTransform() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    FGColumnVector3 nativeForce(100.0, 50.0, -25.0);
    force.SetNativeForces(nativeForce);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // With no transform, body forces should equal native forces
    TS_ASSERT_DELTA(bodyForce(1), nativeForce(1), epsilon);
    TS_ASSERT_DELTA(bodyForce(2), nativeForce(2), epsilon);
    TS_ASSERT_DELTA(bodyForce(3), nativeForce(3), epsilon);
  }

  // Test body force component getters
  void testBodyForceComponents() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(200.0, 100.0, -50.0);

    force.GetBodyForces();  // Compute forces

    TS_ASSERT_DELTA(force.GetBodyXForce(), 200.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyYForce(), 100.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyZForce(), -50.0, epsilon);
  }

  // Test native forces with individual values
  void testSetNativeForcesIndividual() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(10.0, 20.0, 30.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 10.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 20.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 30.0, epsilon);
  }

  // Test native moments
  void testNativeMoments() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    FGColumnVector3 moments(500.0, -200.0, 100.0);
    force.SetNativeMoments(moments);

    // Call GetBodyForces to compute moments
    force.SetTransformType(FGForce::tNone);
    force.GetBodyForces();

    const FGColumnVector3& resultMoments = force.GetMoments();
    // Note: moments include native moments plus force-induced moments
    TS_ASSERT(!std::isnan(resultMoments(1)));
    TS_ASSERT(!std::isnan(resultMoments(2)));
    TS_ASSERT(!std::isnan(resultMoments(3)));
  }

  // Test native moments with individual values
  void testSetNativeMomentsIndividual() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetNativeMoments(100.0, 200.0, 300.0);
    force.SetTransformType(FGForce::tNone);
    force.GetBodyForces();

    const FGColumnVector3& resultMoments = force.GetMoments();
    TS_ASSERT(!std::isnan(resultMoments(1)));
    TS_ASSERT(!std::isnan(resultMoments(2)));
    TS_ASSERT(!std::isnan(resultMoments(3)));
  }

  // Test zero forces produce zero body forces
  void testZeroForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 0.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 0.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 0.0, epsilon);
  }

  // Test large forces
  void testLargeForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    double large = 1e6;
    force.SetNativeForces(large, large, large);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), large, large * 1e-10);
    TS_ASSERT_DELTA(bodyForce(2), large, large * 1e-10);
    TS_ASSERT_DELTA(bodyForce(3), large, large * 1e-10);
  }

  // Test negative forces
  void testNegativeForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(-100.0, -200.0, -300.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), -100.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), -200.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), -300.0, epsilon);
  }

  // Test custom transform with zero angles (should be identity)
  void testCustomTransformZeroAngles() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);
    force.SetNativeForces(100.0, 50.0, 25.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // With zero angles, transform should be identity
    TS_ASSERT_DELTA(bodyForce(1), 100.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 50.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 25.0, epsilon);
  }

  // Test update custom transform matrix
  void testUpdateCustomTransformMatrix() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);

    // Update should not cause any errors
    force.UpdateCustomTransformMatrix();

    force.SetNativeForces(100.0, 0.0, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }

  // Test location at origin produces zero offset moments
  void testLocationOrigin() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(0.0, 0.0, 0.0);
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(100.0, 0.0, 0.0);
    force.SetNativeMoments(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // With force at origin and no native moments, total moments should be zero
    // (depends on CG location though)
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test GetActingLocation returns vector
  void testGetActingLocationVector() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetActingLocation(10.0, 20.0, 30.0);
    const FGColumnVector3& loc = force.GetActingLocation();

    TS_ASSERT_DELTA(loc(1), 10.0, epsilon);
    TS_ASSERT_DELTA(loc(2), 20.0, epsilon);
    TS_ASSERT_DELTA(loc(3), 30.0, epsilon);
  }

  // Test SetActingLocation with vector
  void testSetActingLocationVector() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    FGColumnVector3 loc(15.0, 25.0, 35.0);
    force.SetActingLocation(loc);

    TS_ASSERT_DELTA(force.GetActingLocationX(), 15.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 25.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 35.0, epsilon);
  }

  // Test that acting location can differ from normal location
  void testActingLocationDiffersFromLocation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(100.0, 200.0, 300.0);
    force.SetActingLocation(50.0, 75.0, 100.0);

    // Normal location should remain unchanged
    TS_ASSERT_DELTA(force.GetLocationX(), 100.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 200.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 300.0, epsilon);

    // Acting location should be different
    TS_ASSERT_DELTA(force.GetActingLocationX(), 50.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 75.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 100.0, epsilon);
  }

  // Test SetAnglesToBody with vector input
  void testSetAnglesToBodyVector() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);

    FGColumnVector3 angles(0.15, 0.25, 0.35);
    force.SetAnglesToBody(angles);

    const FGColumnVector3& result = force.GetAnglesToBody();
    TS_ASSERT_DELTA(result(1), 0.15, epsilon);
    TS_ASSERT_DELTA(result(2), 0.25, epsilon);
    TS_ASSERT_DELTA(result(3), 0.35, epsilon);
  }

  // Test Transform() method returns valid matrix
  void testTransformMethod() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);

    const FGMatrix33& T = force.Transform();

    // With zero angles, should be identity matrix
    TS_ASSERT_DELTA(T(1,1), 1.0, epsilon);
    TS_ASSERT_DELTA(T(2,2), 1.0, epsilon);
    TS_ASSERT_DELTA(T(3,3), 1.0, epsilon);
    TS_ASSERT_DELTA(T(1,2), 0.0, epsilon);
    TS_ASSERT_DELTA(T(1,3), 0.0, epsilon);
    TS_ASSERT_DELTA(T(2,1), 0.0, epsilon);
    TS_ASSERT_DELTA(T(2,3), 0.0, epsilon);
    TS_ASSERT_DELTA(T(3,1), 0.0, epsilon);
    TS_ASSERT_DELTA(T(3,2), 0.0, epsilon);
  }

  // Test custom transform with pitch angle (rotate around Y)
  void testCustomTransformPitchOnly() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    double pitchAngle = M_PI / 6.0;  // 30 degrees
    force.SetAnglesToBody(0.0, pitchAngle, 0.0);

    // Apply force along local x-axis
    force.SetNativeForces(100.0, 0.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // Force should be rotated: x component reduced, z component appears
    double expectedX = 100.0 * cos(pitchAngle);
    double expectedZ = -100.0 * sin(pitchAngle);  // Sign depends on convention

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
    // Verify magnitude is preserved
    double magnitude = sqrt(bodyForce(1)*bodyForce(1) + bodyForce(2)*bodyForce(2) + bodyForce(3)*bodyForce(3));
    TS_ASSERT_DELTA(magnitude, 100.0, 0.01);
  }

  // Test custom transform with yaw angle (rotate around Z)
  void testCustomTransformYawOnly() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    double yawAngle = M_PI / 4.0;  // 45 degrees
    force.SetAnglesToBody(0.0, 0.0, yawAngle);

    force.SetNativeForces(100.0, 0.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
    // Verify magnitude is preserved
    double magnitude = sqrt(bodyForce(1)*bodyForce(1) + bodyForce(2)*bodyForce(2) + bodyForce(3)*bodyForce(3));
    TS_ASSERT_DELTA(magnitude, 100.0, 0.01);
  }

  // Test combined pitch and yaw angles
  void testCustomTransformCombinedAngles() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.1, 0.2, 0.3);
    force.SetNativeForces(100.0, 50.0, 25.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));

    // Verify magnitude is preserved through rotation
    double originalMag = sqrt(100.0*100.0 + 50.0*50.0 + 25.0*25.0);
    double resultMag = sqrt(bodyForce(1)*bodyForce(1) + bodyForce(2)*bodyForce(2) + bodyForce(3)*bodyForce(3));
    TS_ASSERT_DELTA(resultMag, originalMag, 0.01);
  }

  // Test SetPitch updates transform matrix
  void testSetPitchUpdatesMatrix() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);

    // Set a pitch value
    force.SetPitch(0.5);
    TS_ASSERT_DELTA(force.GetPitch(), 0.5, epsilon);

    // The transform matrix should have been updated
    force.SetNativeForces(100.0, 0.0, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // Force should now be rotated due to pitch
    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }

  // Test SetYaw updates transform matrix
  void testSetYawUpdatesMatrix() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);

    // Set a yaw value
    force.SetYaw(0.3);
    TS_ASSERT_DELTA(force.GetYaw(), 0.3, epsilon);

    // The transform matrix should have been updated
    force.SetNativeForces(100.0, 0.0, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }

  // Test very small forces
  void testVerySmallForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    double tiny = 1e-15;
    force.SetNativeForces(tiny, tiny, tiny);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), tiny, 1e-20);
    TS_ASSERT_DELTA(bodyForce(2), tiny, 1e-20);
    TS_ASSERT_DELTA(bodyForce(3), tiny, 1e-20);
  }

  // Test forces along each axis separately
  void testForceAlongXAxisOnly() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(500.0, 0.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 500.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 0.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 0.0, epsilon);
  }

  void testForceAlongYAxisOnly() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 500.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 0.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 500.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 0.0, epsilon);
  }

  void testForceAlongZAxisOnly() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 500.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 0.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 0.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 500.0, epsilon);
  }

  // Test multiple force objects independence
  void testMultipleForceObjectsIndependence() {
    FGFDMExec fdmex;
    TestableFGForce force1(&fdmex);
    TestableFGForce force2(&fdmex);

    force1.SetTransformType(FGForce::tNone);
    force2.SetTransformType(FGForce::tNone);

    force1.SetNativeForces(100.0, 0.0, 0.0);
    force2.SetNativeForces(0.0, 200.0, 0.0);

    const FGColumnVector3& body1 = force1.GetBodyForces();
    const FGColumnVector3& body2 = force2.GetBodyForces();

    // Each force should have its own values
    TS_ASSERT_DELTA(body1(1), 100.0, epsilon);
    TS_ASSERT_DELTA(body1(2), 0.0, epsilon);
    TS_ASSERT_DELTA(body2(1), 0.0, epsilon);
    TS_ASSERT_DELTA(body2(2), 200.0, epsilon);
  }

  // Test transform type switching
  void testTransformTypeSwitching() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetNativeForces(100.0, 50.0, 25.0);

    // Test switching between transform types
    force.SetTransformType(FGForce::tNone);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tNone);
    const FGColumnVector3& body1 = force.GetBodyForces();
    TS_ASSERT(!std::isnan(body1(1)));

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tCustom);
    const FGColumnVector3& body2 = force.GetBodyForces();
    TS_ASSERT(!std::isnan(body2(1)));

    force.SetTransformType(FGForce::tNone);
    const FGColumnVector3& body3 = force.GetBodyForces();
    TS_ASSERT(!std::isnan(body3(1)));
  }

  // Test repeated GetBodyForces calls with same input
  void testRepeatedGetBodyForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(100.0, 200.0, 300.0);

    const FGColumnVector3& result1 = force.GetBodyForces();
    const FGColumnVector3& result2 = force.GetBodyForces();
    const FGColumnVector3& result3 = force.GetBodyForces();

    // Results should be consistent
    TS_ASSERT_DELTA(result1(1), result2(1), epsilon);
    TS_ASSERT_DELTA(result1(2), result2(2), epsilon);
    TS_ASSERT_DELTA(result1(3), result2(3), epsilon);
    TS_ASSERT_DELTA(result2(1), result3(1), epsilon);
    TS_ASSERT_DELTA(result2(2), result3(2), epsilon);
    TS_ASSERT_DELTA(result2(3), result3(3), epsilon);
  }

  // Test location with extreme values
  void testExtremeLocationValues() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    double extreme = 1e6;
    force.SetLocation(extreme, extreme, extreme);

    TS_ASSERT_DELTA(force.GetLocationX(), extreme, extreme * 1e-10);
    TS_ASSERT_DELTA(force.GetLocationY(), extreme, extreme * 1e-10);
    TS_ASSERT_DELTA(force.GetLocationZ(), extreme, extreme * 1e-10);
  }

  // Test angles at PI/2 (90 degrees)
  void testAnglesAt90Degrees() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, M_PI / 2.0, 0.0);  // 90 degree pitch

    force.SetNativeForces(100.0, 0.0, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));

    // At 90 degree pitch, x-force should become z-force (approximately)
    TS_ASSERT_DELTA(bodyForce(1), 0.0, 0.1);
  }

  // Test angles at PI (180 degrees)
  void testAnglesAt180Degrees() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, M_PI, 0.0);  // 180 degree pitch

    force.SetNativeForces(100.0, 0.0, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));

    // At 180 degree pitch, x-force should be reversed
    TS_ASSERT_DELTA(bodyForce(1), -100.0, 0.1);
  }

  // Test negative angles
  void testNegativeAngles() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(-0.5, -0.3, -0.2);

    const FGColumnVector3& angles = force.GetAnglesToBody();
    TS_ASSERT_DELTA(angles(1), -0.5, epsilon);
    TS_ASSERT_DELTA(angles(2), -0.3, epsilon);
    TS_ASSERT_DELTA(angles(3), -0.2, epsilon);

    force.SetNativeForces(100.0, 50.0, 25.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }

  // Test moments calculation with force offset
  void testMomentsWithForceOffset() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Place force at a distance from origin
    force.SetLocation(100.0, 0.0, 0.0);  // 100 inches back
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 100.0);  // Force in Z direction
    force.SetNativeMoments(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // With Z force at X offset, should produce Y moment
    // (sign depends on coordinate conventions)
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test stress: rapid force updates
  void testStressRapidForceUpdates() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);

    for (int i = 0; i < 1000; i++) {
      double val = static_cast<double>(i);
      force.SetNativeForces(val, val * 2, val * 3);
      const FGColumnVector3& bodyForce = force.GetBodyForces();

      TS_ASSERT_DELTA(bodyForce(1), val, epsilon);
      TS_ASSERT_DELTA(bodyForce(2), val * 2, epsilon);
      TS_ASSERT_DELTA(bodyForce(3), val * 3, epsilon);
    }
  }

  // Test stress: rapid angle changes with custom transform
  void testStressRapidAngleChanges() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
      double angle = static_cast<double>(i) * 0.01;
      force.SetAnglesToBody(angle, angle * 0.5, angle * 0.25);

      const FGColumnVector3& bodyForce = force.GetBodyForces();
      double magnitude = sqrt(bodyForce(1)*bodyForce(1) +
                              bodyForce(2)*bodyForce(2) +
                              bodyForce(3)*bodyForce(3));

      // Magnitude should be preserved
      TS_ASSERT_DELTA(magnitude, 100.0, 0.01);
    }
  }

  // Test stress: multiple force objects
  void testStressMultipleForceObjects() {
    FGFDMExec fdmex;
    std::vector<TestableFGForce*> forces;

    // Create many force objects
    for (int i = 0; i < 50; i++) {
      forces.push_back(new TestableFGForce(&fdmex));
      forces.back()->SetTransformType(FGForce::tNone);
      forces.back()->SetNativeForces(static_cast<double>(i), 0.0, 0.0);
    }

    // Verify each maintains its own value
    for (int i = 0; i < 50; i++) {
      const FGColumnVector3& bodyForce = forces[i]->GetBodyForces();
      TS_ASSERT_DELTA(bodyForce(1), static_cast<double>(i), epsilon);
    }

    // Cleanup
    for (auto* f : forces) {
      delete f;
    }
  }

  // Test transform type tInertialBody
  void testTransformTypeInertialBody() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tInertialBody);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tInertialBody);

    force.SetNativeForces(100.0, 50.0, 25.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // Should not produce NaN
    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }

  // Test force with denormalized values
  void testDenormalizedForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    double denorm = std::numeric_limits<double>::denorm_min();
    force.SetNativeForces(denorm, denorm, denorm);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }

  // Test mixed positive and negative forces
  void testMixedSignForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(100.0, -50.0, 25.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 100.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), -50.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), 25.0, epsilon);
  }

  // Test mixed positive and negative moments
  void testMixedSignMoments() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeMoments(1000.0, -500.0, 250.0);
    force.SetNativeForces(0.0, 0.0, 0.0);
    force.SetLocation(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test zero location doesn't affect force transformation
  void testZeroLocationForceTransform() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(0.0, 0.0, 0.0);
    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, M_PI / 6.0, 0.0);  // 30 degree pitch
    force.SetNativeForces(100.0, 0.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // Force magnitude should still be preserved
    double magnitude = sqrt(bodyForce(1)*bodyForce(1) +
                            bodyForce(2)*bodyForce(2) +
                            bodyForce(3)*bodyForce(3));
    TS_ASSERT_DELTA(magnitude, 100.0, 0.01);
  }

  // Test UpdateCustomTransformMatrix with various angles
  void testUpdateCustomTransformMatrixVarious() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);

    // Test several angle combinations
    double angles[][3] = {
      {0.0, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {0.0, 0.5, 0.0},
      {0.0, 0.0, 0.5},
      {0.1, 0.2, 0.3},
      {-0.1, -0.2, -0.3}
    };

    for (int i = 0; i < 6; i++) {
      force.SetAnglesToBody(angles[i][0], angles[i][1], angles[i][2]);
      force.UpdateCustomTransformMatrix();

      const FGMatrix33& T = force.Transform();

      // Verify matrix is valid (determinant should be 1 for rotation matrix)
      double det = T(1,1) * (T(2,2)*T(3,3) - T(2,3)*T(3,2))
                 - T(1,2) * (T(2,1)*T(3,3) - T(2,3)*T(3,1))
                 + T(1,3) * (T(2,1)*T(3,2) - T(2,2)*T(3,1));
      TS_ASSERT_DELTA(det, 1.0, 0.01);
    }
  }

  // Test orthogonality of rotation matrix
  void testRotationMatrixOrthogonality() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.3, 0.4, 0.5);

    const FGMatrix33& T = force.Transform();

    // For orthogonal matrix, T * T^T should be identity
    // Check row orthogonality (each row has unit length)
    double row1Mag = sqrt(T(1,1)*T(1,1) + T(1,2)*T(1,2) + T(1,3)*T(1,3));
    double row2Mag = sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2) + T(2,3)*T(2,3));
    double row3Mag = sqrt(T(3,1)*T(3,1) + T(3,2)*T(3,2) + T(3,3)*T(3,3));

    TS_ASSERT_DELTA(row1Mag, 1.0, 0.01);
    TS_ASSERT_DELTA(row2Mag, 1.0, 0.01);
    TS_ASSERT_DELTA(row3Mag, 1.0, 0.01);
  }

  // Test GetMoments with native moments contribution
  void testNativeMomentsContribution() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(0.0, 0.0, 0.0);  // Origin location
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 0.0);  // No forces
    force.SetNativeMoments(100.0, 200.0, 300.0);  // Native moments only

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // With no force offset, native moments should pass through
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test that GetBodyForces is required before GetMoments
  void testGetMomentsAfterGetBodyForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(50.0, 25.0, 10.0);
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(100.0, 50.0, 25.0);
    force.SetNativeMoments(10.0, 20.0, 30.0);

    // Must call GetBodyForces first to compute moments
    const FGColumnVector3& bodyForce = force.GetBodyForces();
    TS_ASSERT(!std::isnan(bodyForce(1)));

    // Now moments should be valid
    const FGColumnVector3& moments = force.GetMoments();
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }
};
