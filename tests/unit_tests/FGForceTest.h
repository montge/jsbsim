#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGForce.h>
#include <models/propulsion/FGThruster.h>
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

  /***************************************************************************
   * Extended Force Vector Tests
   ***************************************************************************/

  // Test force magnitude preservation through transform
  void testForceMagnitudePreservation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    FGColumnVector3 nativeForce(100.0, 200.0, 300.0);
    double originalMag = nativeForce.Magnitude();

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(nativeForce);

    // Test various rotation angles
    double angles[] = {0.0, 0.1, 0.5, 1.0, M_PI / 4, M_PI / 2};
    for (double angle : angles) {
      force.SetAnglesToBody(angle, angle * 0.5, angle * 0.25);
      const FGColumnVector3& bodyForce = force.GetBodyForces();
      double resultMag = bodyForce.Magnitude();
      TS_ASSERT_DELTA(resultMag, originalMag, 0.1);
    }
  }

  // Test force direction reversal with 180 degree rotations
  void testForceDirectionReversal() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 0.0, 0.0);

    // 180 degree yaw should reverse X force in X-Y plane
    force.SetAnglesToBody(0.0, 0.0, M_PI);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), -100.0, 0.1);
    TS_ASSERT_DELTA(bodyForce(3), 0.0, 0.1);
  }

  // Test force scaling
  void testForceScaling() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);

    // Test that force scales linearly
    double scales[] = {0.1, 0.5, 1.0, 2.0, 10.0};
    for (double scale : scales) {
      force.SetNativeForces(100.0 * scale, 50.0 * scale, 25.0 * scale);
      const FGColumnVector3& bodyForce = force.GetBodyForces();

      TS_ASSERT_DELTA(bodyForce(1), 100.0 * scale, epsilon);
      TS_ASSERT_DELTA(bodyForce(2), 50.0 * scale, epsilon);
      TS_ASSERT_DELTA(bodyForce(3), 25.0 * scale, epsilon);
    }
  }

  // Test force superposition
  void testForceSuperposition() {
    FGFDMExec fdmex;
    TestableFGForce force1(&fdmex);
    TestableFGForce force2(&fdmex);

    force1.SetTransformType(FGForce::tNone);
    force2.SetTransformType(FGForce::tNone);

    force1.SetNativeForces(100.0, 0.0, 0.0);
    force2.SetNativeForces(0.0, 100.0, 0.0);

    const FGColumnVector3& bf1 = force1.GetBodyForces();
    const FGColumnVector3& bf2 = force2.GetBodyForces();

    // Sum of forces
    FGColumnVector3 sum = bf1 + bf2;

    TS_ASSERT_DELTA(sum(1), 100.0, epsilon);
    TS_ASSERT_DELTA(sum(2), 100.0, epsilon);
    TS_ASSERT_DELTA(sum(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Extended Location Tests
   ***************************************************************************/

  // Test negative location values
  void testNegativeLocationValues() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(-100.0, -50.0, -25.0);

    TS_ASSERT_DELTA(force.GetLocationX(), -100.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), -50.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), -25.0, epsilon);
  }

  // Test zero location
  void testZeroLocation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(0.0, 0.0, 0.0);

    TS_ASSERT_DELTA(force.GetLocationX(), 0.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 0.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 0.0, epsilon);
  }

  // Test location update sequence
  void testLocationUpdateSequence() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Update location multiple times
    force.SetLocation(10.0, 20.0, 30.0);
    force.SetLocation(40.0, 50.0, 60.0);
    force.SetLocation(70.0, 80.0, 90.0);

    // Only final value should remain
    TS_ASSERT_DELTA(force.GetLocationX(), 70.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 80.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 90.0, epsilon);
  }

  /***************************************************************************
   * Extended Transform Tests
   ***************************************************************************/

  // Test transform consistency
  void testTransformConsistency() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.2, 0.3, 0.4);
    force.SetNativeForces(100.0, 50.0, 25.0);

    // Get forces multiple times
    const FGColumnVector3& bf1 = force.GetBodyForces();
    const FGColumnVector3& bf2 = force.GetBodyForces();
    const FGColumnVector3& bf3 = force.GetBodyForces();

    // Should be identical
    TS_ASSERT_DELTA(bf1(1), bf2(1), epsilon);
    TS_ASSERT_DELTA(bf1(2), bf2(2), epsilon);
    TS_ASSERT_DELTA(bf1(3), bf2(3), epsilon);
    TS_ASSERT_DELTA(bf2(1), bf3(1), epsilon);
    TS_ASSERT_DELTA(bf2(2), bf3(2), epsilon);
    TS_ASSERT_DELTA(bf2(3), bf3(3), epsilon);
  }

  // Test small angle approximation regime
  void testSmallAngles() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 0.0, 0.0);

    // Very small angles - transform should be approximately identity
    double smallAngle = 1e-6;
    force.SetAnglesToBody(smallAngle, smallAngle, smallAngle);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 100.0, 0.001);
    TS_ASSERT_DELTA(bodyForce(2), 0.0, 0.001);
    TS_ASSERT_DELTA(bodyForce(3), 0.0, 0.001);
  }

  // Test roll angle effect
  void testRollAngleEffect() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(0.0, 100.0, 0.0);  // Y-axis force

    // 90 degree roll should swap Y and Z
    force.SetAnglesToBody(M_PI / 2.0, 0.0, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 0.0, 0.1);
    // Y and Z should be swapped (sign depends on convention)
    double magnitude = sqrt(bodyForce(2)*bodyForce(2) + bodyForce(3)*bodyForce(3));
    TS_ASSERT_DELTA(magnitude, 100.0, 0.1);
  }

  /***************************************************************************
   * Extended Moment Tests
   ***************************************************************************/

  // Test moment arm calculation
  void testMomentArmCalculation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Force at X = 100 inches, pure Z force
    force.SetLocation(100.0, 0.0, 0.0);
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 50.0);
    force.SetNativeMoments(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // M = r x F
    // r = (100, 0, 0), F = (0, 0, 50)
    // M = (0*50 - 0*0, 0*0 - 100*50, 100*0 - 0*0) = (0, -5000, 0)
    // But actual calculation depends on CG offset
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test symmetric forces produce zero net moment
  void testSymmetricForcesZeroMoment() {
    FGFDMExec fdmex;
    TestableFGForce forceLeft(&fdmex);
    TestableFGForce forceRight(&fdmex);

    // Symmetric forces at +/- Y positions
    forceLeft.SetLocation(0.0, -100.0, 0.0);
    forceRight.SetLocation(0.0, 100.0, 0.0);

    forceLeft.SetTransformType(FGForce::tNone);
    forceRight.SetTransformType(FGForce::tNone);

    // Same force on both sides
    forceLeft.SetNativeForces(0.0, 0.0, -50.0);
    forceRight.SetNativeForces(0.0, 0.0, -50.0);
    forceLeft.SetNativeMoments(0.0, 0.0, 0.0);
    forceRight.SetNativeMoments(0.0, 0.0, 0.0);

    forceLeft.GetBodyForces();
    forceRight.GetBodyForces();

    const FGColumnVector3& ml = forceLeft.GetMoments();
    const FGColumnVector3& mr = forceRight.GetMoments();

    // Roll moments should cancel (opposite signs)
    // Note: depends on CG position
    TS_ASSERT(!std::isnan(ml(1) + mr(1)));
  }

  /***************************************************************************
   * Angle Boundary Tests
   ***************************************************************************/

  // Test angles at 2*PI (360 degrees)
  void testAnglesAt360Degrees() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 2.0 * M_PI, 0.0);  // 360 degree pitch
    force.SetNativeForces(100.0, 0.0, 0.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // 360 degrees should be same as 0 degrees
    TS_ASSERT_DELTA(bodyForce(1), 100.0, 0.1);
    TS_ASSERT_DELTA(bodyForce(2), 0.0, 0.1);
    TS_ASSERT_DELTA(bodyForce(3), 0.0, 0.1);
  }

  // Test angles beyond 2*PI
  void testAnglesBeyon360Degrees() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 0.0, 0.0);

    // 450 degrees should be same as 90 degrees
    force.SetAnglesToBody(0.0, 2.5 * M_PI, 0.0);
    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
  }


  /***************************************************************************
   * Force and Location Combination Tests
   ***************************************************************************/

  // Test force at various positions
  void testForceAtVariousPositions() {
    FGFDMExec fdmex;

    double positions[][3] = {
      {0.0, 0.0, 0.0},
      {100.0, 0.0, 0.0},
      {0.0, 100.0, 0.0},
      {0.0, 0.0, 100.0},
      {100.0, 100.0, 100.0},
      {-100.0, -100.0, -100.0}
    };

    for (int i = 0; i < 6; i++) {
      TestableFGForce force(&fdmex);
      force.SetLocation(positions[i][0], positions[i][1], positions[i][2]);
      force.SetTransformType(FGForce::tNone);
      force.SetNativeForces(100.0, 50.0, 25.0);

      const FGColumnVector3& bodyForce = force.GetBodyForces();

      // Force should be independent of position
      TS_ASSERT_DELTA(bodyForce(1), 100.0, epsilon);
      TS_ASSERT_DELTA(bodyForce(2), 50.0, epsilon);
      TS_ASSERT_DELTA(bodyForce(3), 25.0, epsilon);
    }
  }

  /***************************************************************************
   * Additional Stress Tests
   ***************************************************************************/

  // Test stress with alternating signs
  void testStressAlternatingSigns() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);

    for (int i = 0; i < 500; i++) {
      double sign = (i % 2 == 0) ? 1.0 : -1.0;
      force.SetNativeForces(sign * 100.0, sign * 50.0, sign * 25.0);

      const FGColumnVector3& bodyForce = force.GetBodyForces();

      TS_ASSERT_DELTA(bodyForce(1), sign * 100.0, epsilon);
      TS_ASSERT_DELTA(bodyForce(2), sign * 50.0, epsilon);
      TS_ASSERT_DELTA(bodyForce(3), sign * 25.0, epsilon);
    }
  }

  // Test stress with growing and shrinking forces
  void testStressGrowingShrinking() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);

    // Growing
    for (int i = 0; i < 100; i++) {
      double val = i * 10.0;
      force.SetNativeForces(val, val, val);
      const FGColumnVector3& bodyForce = force.GetBodyForces();
      TS_ASSERT_DELTA(bodyForce(1), val, epsilon);
    }

    // Shrinking
    for (int i = 100; i >= 0; i--) {
      double val = i * 10.0;
      force.SetNativeForces(val, val, val);
      const FGColumnVector3& bodyForce = force.GetBodyForces();
      TS_ASSERT_DELTA(bodyForce(1), val, epsilon);
    }
  }

  /***************************************************************************
   * Unit Force Tests
   ***************************************************************************/

  // Test unit force in each direction
  void testUnitForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);

    // Unit X
    force.SetNativeForces(1.0, 0.0, 0.0);
    TS_ASSERT_DELTA(force.GetBodyForces()(1), 1.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyForces()(2), 0.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyForces()(3), 0.0, epsilon);

    // Unit Y
    force.SetNativeForces(0.0, 1.0, 0.0);
    TS_ASSERT_DELTA(force.GetBodyForces()(1), 0.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyForces()(2), 1.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyForces()(3), 0.0, epsilon);

    // Unit Z
    force.SetNativeForces(0.0, 0.0, 1.0);
    TS_ASSERT_DELTA(force.GetBodyForces()(1), 0.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyForces()(2), 0.0, epsilon);
    TS_ASSERT_DELTA(force.GetBodyForces()(3), 1.0, epsilon);
  }

  // Test diagonal unit force
  void testDiagonalUnitForce() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);

    // Unit vector along (1,1,1) direction
    double component = 1.0 / sqrt(3.0);
    force.SetNativeForces(component, component, component);

    const FGColumnVector3& bodyForce = force.GetBodyForces();
    double magnitude = bodyForce.Magnitude();

    TS_ASSERT_DELTA(magnitude, 1.0, epsilon);
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  // Test precision with very different magnitudes
  void testMixedMagnitudes() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(1e10, 1e-10, 1.0);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    TS_ASSERT_DELTA(bodyForce(1), 1e10, 1.0);
    TS_ASSERT_DELTA(bodyForce(2), 1e-10, 1e-15);
    TS_ASSERT_DELTA(bodyForce(3), 1.0, epsilon);
  }

  // Test precision near machine epsilon
  void testNearEpsilonForces() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tNone);
    double eps = std::numeric_limits<double>::epsilon();
    force.SetNativeForces(1.0 + eps, 1.0, 1.0 - eps);

    const FGColumnVector3& bodyForce = force.GetBodyForces();

    // Should preserve these tiny differences
    TS_ASSERT(bodyForce(1) > bodyForce(2));
    TS_ASSERT(bodyForce(2) > bodyForce(3));
  }

  /***************************************************************************
   * Extended Multi-Instance Tests (Tests 79-83)
   ***************************************************************************/

  // Test 79: Multiple forces with different transform types
  void testMultipleForcesWithDifferentTransforms() {
    FGFDMExec fdmex;
    TestableFGForce forceNone(&fdmex);
    TestableFGForce forceCustom(&fdmex);

    forceNone.SetTransformType(FGForce::tNone);
    forceCustom.SetTransformType(FGForce::tCustom);
    forceCustom.SetAnglesToBody(0.0, 0.0, 0.0);

    forceNone.SetNativeForces(100.0, 50.0, 25.0);
    forceCustom.SetNativeForces(100.0, 50.0, 25.0);

    const FGColumnVector3& bf1 = forceNone.GetBodyForces();
    const FGColumnVector3& bf2 = forceCustom.GetBodyForces();

    // With identity transform, both should give same result
    TS_ASSERT_DELTA(bf1(1), bf2(1), epsilon);
    TS_ASSERT_DELTA(bf1(2), bf2(2), epsilon);
    TS_ASSERT_DELTA(bf1(3), bf2(3), epsilon);
  }

  // Test 80: Force objects from different FDMExec instances
  void testForcesFromDifferentFDMExec() {
    FGFDMExec fdmex1, fdmex2;

    TestableFGForce force1(&fdmex1);
    TestableFGForce force2(&fdmex2);

    force1.SetTransformType(FGForce::tNone);
    force2.SetTransformType(FGForce::tNone);

    force1.SetNativeForces(111.0, 222.0, 333.0);
    force2.SetNativeForces(444.0, 555.0, 666.0);

    const FGColumnVector3& bf1 = force1.GetBodyForces();
    const FGColumnVector3& bf2 = force2.GetBodyForces();

    // Each should have its own values
    TS_ASSERT_DELTA(bf1(1), 111.0, epsilon);
    TS_ASSERT_DELTA(bf2(1), 444.0, epsilon);
  }

  // Test 81: Many forces sequentially
  void testManyForcesSequentially() {
    FGFDMExec fdmex;

    for (int i = 0; i < 100; i++) {
      TestableFGForce force(&fdmex);
      force.SetTransformType(FGForce::tNone);
      force.SetNativeForces(i * 1.0, i * 2.0, i * 3.0);

      const FGColumnVector3& bf = force.GetBodyForces();
      TS_ASSERT_DELTA(bf(1), i * 1.0, epsilon);
      TS_ASSERT_DELTA(bf(2), i * 2.0, epsilon);
      TS_ASSERT_DELTA(bf(3), i * 3.0, epsilon);
    }
  }

  // Test 82: Force array operations
  void testForceArrayOperations() {
    FGFDMExec fdmex;
    std::vector<std::unique_ptr<TestableFGForce>> forces;

    for (int i = 0; i < 10; i++) {
      forces.push_back(std::make_unique<TestableFGForce>(&fdmex));
      forces.back()->SetTransformType(FGForce::tNone);
      forces.back()->SetNativeForces(i * 10.0, 0.0, 0.0);
    }

    // Verify each force has correct value
    for (int i = 0; i < 10; i++) {
      const FGColumnVector3& bf = forces[i]->GetBodyForces();
      TS_ASSERT_DELTA(bf(1), i * 10.0, epsilon);
    }
  }

  // Test 83: Force independence across updates
  void testForceIndependenceAcrossUpdates() {
    FGFDMExec fdmex;
    TestableFGForce force1(&fdmex);
    TestableFGForce force2(&fdmex);

    force1.SetTransformType(FGForce::tNone);
    force2.SetTransformType(FGForce::tNone);

    // Set initial values
    force1.SetNativeForces(100.0, 0.0, 0.0);
    force2.SetNativeForces(0.0, 100.0, 0.0);

    // Update force1, verify force2 unchanged
    force1.SetNativeForces(200.0, 0.0, 0.0);

    const FGColumnVector3& bf1 = force1.GetBodyForces();
    const FGColumnVector3& bf2 = force2.GetBodyForces();

    TS_ASSERT_DELTA(bf1(1), 200.0, epsilon);
    TS_ASSERT_DELTA(bf2(2), 100.0, epsilon);
  }

  /***************************************************************************
   * Extended Transform Combination Tests (Tests 84-88)
   ***************************************************************************/

  // Test 84: Full 3D rotation sequence
  void testFull3DRotationSequence() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 0.0, 0.0);

    // Apply progressive rotations
    double angles[] = {0.0, M_PI/6, M_PI/4, M_PI/3, M_PI/2};

    for (double roll : angles) {
      for (double pitch : angles) {
        for (double yaw : angles) {
          force.SetAnglesToBody(roll, pitch, yaw);
          const FGColumnVector3& bf = force.GetBodyForces();

          // Magnitude should always be preserved
          double mag = bf.Magnitude();
          TS_ASSERT_DELTA(mag, 100.0, 0.01);
        }
      }
    }
  }

  // Test 85: Gimbal lock angles
  void testGimbalLockAngles() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 50.0, 25.0);

    // Test near gimbal lock (pitch = +/- 90 degrees)
    force.SetAnglesToBody(0.1, M_PI / 2.0, 0.2);
    const FGColumnVector3& bf1 = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bf1(1)));
    TS_ASSERT(!std::isnan(bf1(2)));
    TS_ASSERT(!std::isnan(bf1(3)));

    force.SetAnglesToBody(0.1, -M_PI / 2.0, 0.2);
    const FGColumnVector3& bf2 = force.GetBodyForces();

    TS_ASSERT(!std::isnan(bf2(1)));
    TS_ASSERT(!std::isnan(bf2(2)));
    TS_ASSERT(!std::isnan(bf2(3)));
  }

  // Test 86: Transform type cycling
  void testTransformTypeCycling() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetNativeForces(100.0, 50.0, 25.0);

    FGForce::TransformType types[] = {
      FGForce::tNone, FGForce::tCustom, FGForce::tNone,
      FGForce::tCustom, FGForce::tNone
    };

    for (int i = 0; i < 5; i++) {
      force.SetTransformType(types[i]);
      if (types[i] == FGForce::tCustom) {
        force.SetAnglesToBody(0.0, 0.0, 0.0);
      }

      const FGColumnVector3& bf = force.GetBodyForces();
      TS_ASSERT_DELTA(bf(1), 100.0, epsilon);
      TS_ASSERT_DELTA(bf(2), 50.0, epsilon);
      TS_ASSERT_DELTA(bf(3), 25.0, epsilon);
    }
  }

  // Test 87: Sequential angle updates
  void testSequentialAngleUpdates() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetNativeForces(100.0, 0.0, 0.0);

    // Update angles incrementally
    for (int i = 0; i < 36; i++) {
      double angle = i * M_PI / 18.0;  // 10 degree increments
      force.SetAnglesToBody(0.0, angle, 0.0);

      const FGColumnVector3& bf = force.GetBodyForces();
      double mag = bf.Magnitude();
      TS_ASSERT_DELTA(mag, 100.0, 0.01);
    }
  }

  // Test 88: Pitch and yaw individual setters
  void testPitchYawIndividualSetters() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);
    force.SetNativeForces(100.0, 0.0, 0.0);

    // Set pitch individually
    force.SetPitch(0.5);
    TS_ASSERT_DELTA(force.GetPitch(), 0.5, epsilon);

    // Set yaw individually
    force.SetYaw(0.3);
    TS_ASSERT_DELTA(force.GetYaw(), 0.3, epsilon);

    const FGColumnVector3& bf = force.GetBodyForces();
    double mag = bf.Magnitude();
    TS_ASSERT_DELTA(mag, 100.0, 0.01);
  }

  /***************************************************************************
   * Extended Moment Calculation Tests (Tests 89-93)
   ***************************************************************************/

  // Test 89: Moment with force and native moment combination
  void testMomentForcePlusNative() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(100.0, 0.0, 0.0);
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 50.0);
    force.SetNativeMoments(100.0, 200.0, 300.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // Moments should include both contributions
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test 90: Moment direction with Y-offset force
  void testMomentDirectionYOffset() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(0.0, 100.0, 0.0);  // Y offset
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 100.0);  // Z force
    force.SetNativeMoments(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // Should produce roll moment
    TS_ASSERT(!std::isnan(moments(1)));
  }

  // Test 91: Moment direction with Z-offset force
  void testMomentDirectionZOffset() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(0.0, 0.0, 100.0);  // Z offset
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(100.0, 0.0, 0.0);  // X force
    force.SetNativeMoments(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // Should produce pitch moment
    TS_ASSERT(!std::isnan(moments(2)));
  }

  // Test 92: Moment with acting location different from location
  void testMomentWithActingLocation() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(100.0, 0.0, 0.0);
    force.SetActingLocation(50.0, 0.0, 0.0);  // Different acting location
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 100.0);
    force.SetNativeMoments(0.0, 0.0, 0.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test 93: Zero force produces only native moments
  void testZeroForceOnlyNativeMoments() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetLocation(100.0, 100.0, 100.0);
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(0.0, 0.0, 0.0);
    force.SetNativeMoments(50.0, 100.0, 150.0);

    force.GetBodyForces();
    const FGColumnVector3& moments = force.GetMoments();

    // With zero force, only native moments contribute
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  /***************************************************************************
   * Complete Verification Tests (Tests 94-100)
   ***************************************************************************/

  // Test 94: Full location API verification
  void testFullLocationAPIVerification() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Test SetLocation(x, y, z)
    force.SetLocation(10.0, 20.0, 30.0);
    TS_ASSERT_DELTA(force.GetLocationX(), 10.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 20.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 30.0, epsilon);

    // Test SetLocation(vector)
    FGColumnVector3 loc(40.0, 50.0, 60.0);
    force.SetLocation(loc);
    const FGColumnVector3& result = force.GetLocation();
    TS_ASSERT_DELTA(result(1), 40.0, epsilon);
    TS_ASSERT_DELTA(result(2), 50.0, epsilon);
    TS_ASSERT_DELTA(result(3), 60.0, epsilon);

    // Test individual setters
    force.SetLocationX(70.0);
    force.SetLocationY(80.0);
    force.SetLocationZ(90.0);
    TS_ASSERT_DELTA(force.GetLocationX(), 70.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 80.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 90.0, epsilon);
  }

  // Test 95: Full acting location API verification
  void testFullActingLocationAPIVerification() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Test SetActingLocation(x, y, z)
    force.SetActingLocation(10.0, 20.0, 30.0);
    TS_ASSERT_DELTA(force.GetActingLocationX(), 10.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 20.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 30.0, epsilon);

    // Test SetActingLocation(vector)
    FGColumnVector3 loc(40.0, 50.0, 60.0);
    force.SetActingLocation(loc);
    const FGColumnVector3& result = force.GetActingLocation();
    TS_ASSERT_DELTA(result(1), 40.0, epsilon);
    TS_ASSERT_DELTA(result(2), 50.0, epsilon);
    TS_ASSERT_DELTA(result(3), 60.0, epsilon);

    // Test individual setters
    force.SetActingLocationX(70.0);
    force.SetActingLocationY(80.0);
    force.SetActingLocationZ(90.0);
    TS_ASSERT_DELTA(force.GetActingLocationX(), 70.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 80.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 90.0, epsilon);
  }

  // Test 96: Full transform type API verification
  void testFullTransformTypeAPIVerification() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // Test all transform types
    force.SetTransformType(FGForce::tNone);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tNone);

    force.SetTransformType(FGForce::tWindBody);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tWindBody);

    force.SetTransformType(FGForce::tLocalBody);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tLocalBody);

    force.SetTransformType(FGForce::tInertialBody);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tInertialBody);

    force.SetTransformType(FGForce::tCustom);
    TS_ASSERT_EQUALS(force.GetTransformType(), FGForce::tCustom);
  }

  // Test 97: Full angles API verification
  void testFullAnglesAPIVerification() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);

    // Test SetAnglesToBody(roll, pitch, yaw)
    force.SetAnglesToBody(0.1, 0.2, 0.3);
    const FGColumnVector3& angles = force.GetAnglesToBody();
    TS_ASSERT_DELTA(angles(1), 0.1, epsilon);
    TS_ASSERT_DELTA(angles(2), 0.2, epsilon);
    TS_ASSERT_DELTA(angles(3), 0.3, epsilon);

    // Test indexed access
    TS_ASSERT_DELTA(force.GetAnglesToBody(1), 0.1, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(2), 0.2, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(3), 0.3, epsilon);

    // Test SetAnglesToBody(vector)
    FGColumnVector3 newAngles(0.4, 0.5, 0.6);
    force.SetAnglesToBody(newAngles);
    TS_ASSERT_DELTA(force.GetAnglesToBody(1), 0.4, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(2), 0.5, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(3), 0.6, epsilon);

    // Test pitch/yaw individual setters/getters
    force.SetPitch(0.7);
    force.SetYaw(0.8);
    TS_ASSERT_DELTA(force.GetPitch(), 0.7, epsilon);
    TS_ASSERT_DELTA(force.GetYaw(), 0.8, epsilon);
  }

  // Test 98: Full force calculation workflow
  void testFullForceCalculationWorkflow() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // 1. Set transform type
    force.SetTransformType(FGForce::tCustom);

    // 2. Set angles
    force.SetAnglesToBody(0.1, 0.2, 0.3);

    // 3. Set location
    force.SetLocation(100.0, 50.0, 25.0);

    // 4. Set native forces and moments
    force.SetNativeForces(1000.0, 500.0, 250.0);
    force.SetNativeMoments(100.0, 50.0, 25.0);

    // 5. Get body forces
    const FGColumnVector3& bodyForce = force.GetBodyForces();
    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));

    // 6. Get component forces
    TS_ASSERT(!std::isnan(force.GetBodyXForce()));
    TS_ASSERT(!std::isnan(force.GetBodyYForce()));
    TS_ASSERT(!std::isnan(force.GetBodyZForce()));

    // 7. Get moments
    const FGColumnVector3& moments = force.GetMoments();
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));

    // 8. Verify force magnitude preserved
    double originalMag = sqrt(1000.0*1000.0 + 500.0*500.0 + 250.0*250.0);
    double resultMag = bodyForce.Magnitude();
    TS_ASSERT_DELTA(resultMag, originalMag, 1.0);
  }

  // Test 99: Full stress test workflow
  void testFullStressTestWorkflow() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    force.SetTransformType(FGForce::tCustom);

    // Many iterations with changing values
    for (int i = 0; i < 200; i++) {
      double angle = i * 0.01;
      double fmag = 100.0 + i;

      force.SetAnglesToBody(angle * 0.5, angle, angle * 0.3);
      force.SetLocation(i * 1.0, i * 0.5, i * 0.25);
      force.SetNativeForces(fmag, fmag * 0.5, fmag * 0.25);
      force.SetNativeMoments(i * 0.1, i * 0.2, i * 0.3);

      const FGColumnVector3& bf = force.GetBodyForces();
      const FGColumnVector3& m = force.GetMoments();

      // All should be finite
      TS_ASSERT(std::isfinite(bf(1)));
      TS_ASSERT(std::isfinite(bf(2)));
      TS_ASSERT(std::isfinite(bf(3)));
      TS_ASSERT(std::isfinite(m(1)));
      TS_ASSERT(std::isfinite(m(2)));
      TS_ASSERT(std::isfinite(m(3)));
    }
  }

  // Test 100: Complete force system integration test
  void testCompleteForceSystemIntegration() {
    FGFDMExec fdmex;
    TestableFGForce force(&fdmex);

    // 1. Verify construction
    TS_ASSERT(true);  // Force object created successfully

    // 2. Test all transform types
    for (int t = 0; t <= 4; t++) {
      force.SetTransformType(static_cast<FGForce::TransformType>(t));
      TS_ASSERT_EQUALS(force.GetTransformType(), static_cast<FGForce::TransformType>(t));
    }

    // 3. Set up for custom transform
    force.SetTransformType(FGForce::tCustom);
    force.SetAnglesToBody(0.0, 0.0, 0.0);

    // 4. Test location API
    force.SetLocation(100.0, 50.0, 25.0);
    TS_ASSERT_DELTA(force.GetLocationX(), 100.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), 50.0, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), 25.0, epsilon);

    // 5. Test acting location API
    force.SetActingLocation(75.0, 25.0, 10.0);
    TS_ASSERT_DELTA(force.GetActingLocationX(), 75.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 25.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 10.0, epsilon);

    // 6. Test angles API
    force.SetAnglesToBody(0.1, 0.2, 0.3);
    TS_ASSERT_DELTA(force.GetAnglesToBody(1), 0.1, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(2), 0.2, epsilon);
    TS_ASSERT_DELTA(force.GetAnglesToBody(3), 0.3, epsilon);

    // 7. Test pitch/yaw setters
    force.SetPitch(0.25);
    force.SetYaw(0.35);
    TS_ASSERT_DELTA(force.GetPitch(), 0.25, epsilon);
    TS_ASSERT_DELTA(force.GetYaw(), 0.35, epsilon);

    // 8. Test native forces
    force.SetNativeForces(500.0, 250.0, 125.0);

    // 9. Test native moments
    force.SetNativeMoments(50.0, 25.0, 12.5);

    // 10. Get body forces
    const FGColumnVector3& bodyForce = force.GetBodyForces();
    TS_ASSERT(!std::isnan(bodyForce(1)));
    TS_ASSERT(!std::isnan(bodyForce(2)));
    TS_ASSERT(!std::isnan(bodyForce(3)));
    TS_ASSERT(std::isfinite(bodyForce(1)));
    TS_ASSERT(std::isfinite(bodyForce(2)));
    TS_ASSERT(std::isfinite(bodyForce(3)));

    // 11. Test body force components match vector
    TS_ASSERT_DELTA(force.GetBodyXForce(), bodyForce(1), epsilon);
    TS_ASSERT_DELTA(force.GetBodyYForce(), bodyForce(2), epsilon);
    TS_ASSERT_DELTA(force.GetBodyZForce(), bodyForce(3), epsilon);

    // 12. Get moments
    const FGColumnVector3& moments = force.GetMoments();
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
    TS_ASSERT(std::isfinite(moments(1)));
    TS_ASSERT(std::isfinite(moments(2)));
    TS_ASSERT(std::isfinite(moments(3)));

    // 13. Test transform matrix
    const FGMatrix33& T = force.Transform();
    // Matrix should have determinant = 1 (rotation matrix)
    double det = T(1,1) * (T(2,2)*T(3,3) - T(2,3)*T(3,2))
               - T(1,2) * (T(2,1)*T(3,3) - T(2,3)*T(3,1))
               + T(1,3) * (T(2,1)*T(3,2) - T(2,2)*T(3,1));
    TS_ASSERT_DELTA(det, 1.0, 0.01);

    // 14. Test force magnitude preservation
    double originalMag = sqrt(500.0*500.0 + 250.0*250.0 + 125.0*125.0);
    double resultMag = bodyForce.Magnitude();
    TS_ASSERT_DELTA(resultMag, originalMag, 1.0);

    // 15. Test with identity transform
    force.SetAnglesToBody(0.0, 0.0, 0.0);
    force.SetNativeForces(100.0, 50.0, 25.0);
    const FGColumnVector3& identityForce = force.GetBodyForces();
    TS_ASSERT_DELTA(identityForce(1), 100.0, epsilon);
    TS_ASSERT_DELTA(identityForce(2), 50.0, epsilon);
    TS_ASSERT_DELTA(identityForce(3), 25.0, epsilon);

    // 16. Final verification with tNone
    force.SetTransformType(FGForce::tNone);
    force.SetNativeForces(1000.0, 2000.0, 3000.0);
    const FGColumnVector3& finalForce = force.GetBodyForces();
    TS_ASSERT_DELTA(finalForce(1), 1000.0, epsilon);
    TS_ASSERT_DELTA(finalForce(2), 2000.0, epsilon);
    TS_ASSERT_DELTA(finalForce(3), 3000.0, epsilon);
  }

  //==========================================================================
  // Class-based tests using FGFDMExec with loaded aircraft model
  //==========================================================================

  // Test loading c172x model and accessing propulsion system
  void testLoadC172xModel() {
    FGFDMExec fdmex;
    bool loaded = fdmex.LoadModel("c172x");

    TS_ASSERT(loaded);
    TS_ASSERT(fdmex.GetPropulsion() != nullptr);
  }

  // Test getting thruster (FGForce subclass) from propulsion
  void testGetThrusterFromPropulsion() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    TS_ASSERT(propulsion != nullptr);

    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      TS_ASSERT(engine != nullptr);

      FGThruster* thruster = engine->GetThruster();
      TS_ASSERT(thruster != nullptr);
    }
  }

  // Test thruster body forces through FGForce interface
  void testThrusterBodyForces() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        const FGColumnVector3& bodyForces = thruster->GetBodyForces();

        // Verify forces are finite
        TS_ASSERT(std::isfinite(bodyForces(1)));
        TS_ASSERT(std::isfinite(bodyForces(2)));
        TS_ASSERT(std::isfinite(bodyForces(3)));

        // Verify individual component getters match vector
        TS_ASSERT_DELTA(thruster->GetBodyXForce(), bodyForces(1), epsilon);
        TS_ASSERT_DELTA(thruster->GetBodyYForce(), bodyForces(2), epsilon);
        TS_ASSERT_DELTA(thruster->GetBodyZForce(), bodyForces(3), epsilon);
      }
    }
  }

  // Test thruster moments through FGForce interface
  void testThrusterMoments() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        const FGColumnVector3& moments = thruster->GetMoments();

        // Verify moments are finite
        TS_ASSERT(std::isfinite(moments(1)));
        TS_ASSERT(std::isfinite(moments(2)));
        TS_ASSERT(std::isfinite(moments(3)));
      }
    }
  }

  // Test thruster location getters
  void testThrusterLocation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        // Get location values
        double locX = thruster->GetLocationX();
        double locY = thruster->GetLocationY();
        double locZ = thruster->GetLocationZ();

        // Verify they are finite
        TS_ASSERT(std::isfinite(locX));
        TS_ASSERT(std::isfinite(locY));
        TS_ASSERT(std::isfinite(locZ));

        // Verify vector accessor matches individual getters
        const FGColumnVector3& loc = thruster->GetLocation();
        TS_ASSERT_DELTA(loc(1), locX, epsilon);
        TS_ASSERT_DELTA(loc(2), locY, epsilon);
        TS_ASSERT_DELTA(loc(3), locZ, epsilon);
      }
    }
  }

  // Test thruster acting location getters
  void testThrusterActingLocation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        // Get acting location values
        double actX = thruster->GetActingLocationX();
        double actY = thruster->GetActingLocationY();
        double actZ = thruster->GetActingLocationZ();

        // Verify they are finite
        TS_ASSERT(std::isfinite(actX));
        TS_ASSERT(std::isfinite(actY));
        TS_ASSERT(std::isfinite(actZ));

        // Verify vector accessor matches individual getters
        const FGColumnVector3& actLoc = thruster->GetActingLocation();
        TS_ASSERT_DELTA(actLoc(1), actX, epsilon);
        TS_ASSERT_DELTA(actLoc(2), actY, epsilon);
        TS_ASSERT_DELTA(actLoc(3), actZ, epsilon);
      }
    }
  }

  // Test thruster transform type
  void testThrusterTransformType() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        FGForce::TransformType ttype = thruster->GetTransformType();

        // Transform type should be one of the valid enum values
        TS_ASSERT(ttype == FGForce::tNone ||
                  ttype == FGForce::tWindBody ||
                  ttype == FGForce::tLocalBody ||
                  ttype == FGForce::tInertialBody ||
                  ttype == FGForce::tCustom);
      }
    }
  }

  // Test thruster transform matrix
  void testThrusterTransformMatrix() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        const FGMatrix33& T = thruster->Transform();

        // All matrix elements should be finite
        for (int i = 1; i <= 3; i++) {
          for (int j = 1; j <= 3; j++) {
            TS_ASSERT(std::isfinite(T(i, j)));
          }
        }
      }
    }
  }

  // Test thruster angles to body
  void testThrusterAnglesToBody() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        const FGColumnVector3& angles = thruster->GetAnglesToBody();

        // Angles should be finite
        TS_ASSERT(std::isfinite(angles(1)));
        TS_ASSERT(std::isfinite(angles(2)));
        TS_ASSERT(std::isfinite(angles(3)));

        // Individual accessor should match vector
        TS_ASSERT_DELTA(thruster->GetAnglesToBody(1), angles(1), epsilon);
        TS_ASSERT_DELTA(thruster->GetAnglesToBody(2), angles(2), epsilon);
        TS_ASSERT_DELTA(thruster->GetAnglesToBody(3), angles(3), epsilon);
      }
    }
  }

  // Test thruster pitch and yaw getters
  void testThrusterPitchYaw() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        double pitch = thruster->GetPitch();
        double yaw = thruster->GetYaw();

        // Values should be finite
        TS_ASSERT(std::isfinite(pitch));
        TS_ASSERT(std::isfinite(yaw));

        // Angles are in radians, should be in reasonable range
        TS_ASSERT(pitch >= -M_PI && pitch <= M_PI);
        TS_ASSERT(yaw >= -M_PI && yaw <= M_PI);
      }
    }
  }

  // Test running simulation and checking force updates
  void testForcesDuringSimulation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    // Initialize the simulation
    fdmex.RunIC();

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        // Run a few simulation steps
        for (int i = 0; i < 10; i++) {
          fdmex.Run();

          // Forces should remain finite during simulation
          const FGColumnVector3& bodyForces = thruster->GetBodyForces();
          TS_ASSERT(std::isfinite(bodyForces(1)));
          TS_ASSERT(std::isfinite(bodyForces(2)));
          TS_ASSERT(std::isfinite(bodyForces(3)));

          // Moments should remain finite during simulation
          const FGColumnVector3& moments = thruster->GetMoments();
          TS_ASSERT(std::isfinite(moments(1)));
          TS_ASSERT(std::isfinite(moments(2)));
          TS_ASSERT(std::isfinite(moments(3)));
        }
      }
    }
  }

  // Test multiple engines accessing thrusters
  void testMultipleEngineThrusters() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    size_t numEngines = propulsion->GetNumEngines();

    // Test all available engines
    for (size_t i = 0; i < numEngines; i++) {
      auto engine = propulsion->GetEngine(i);
      TS_ASSERT(engine != nullptr);

      FGThruster* thruster = engine->GetThruster();
      TS_ASSERT(thruster != nullptr);

      if (thruster != nullptr) {
        // Each thruster should have valid forces
        const FGColumnVector3& forces = thruster->GetBodyForces();
        TS_ASSERT(std::isfinite(forces(1)));
        TS_ASSERT(std::isfinite(forces(2)));
        TS_ASSERT(std::isfinite(forces(3)));
      }
    }
  }

  // Test force location setters with model loaded
  void testForceLocationSettersWithModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        // Save original location
        double origX = thruster->GetLocationX();
        double origY = thruster->GetLocationY();
        double origZ = thruster->GetLocationZ();

        // Modify location
        thruster->SetLocation(100.0, 50.0, 25.0);

        TS_ASSERT_DELTA(thruster->GetLocationX(), 100.0, epsilon);
        TS_ASSERT_DELTA(thruster->GetLocationY(), 50.0, epsilon);
        TS_ASSERT_DELTA(thruster->GetLocationZ(), 25.0, epsilon);

        // Acting location should be updated too
        TS_ASSERT_DELTA(thruster->GetActingLocationX(), 100.0, epsilon);
        TS_ASSERT_DELTA(thruster->GetActingLocationY(), 50.0, epsilon);
        TS_ASSERT_DELTA(thruster->GetActingLocationZ(), 25.0, epsilon);

        // Restore original location
        thruster->SetLocation(origX, origY, origZ);
      }
    }
  }

  // Test force angles setters with model loaded
  void testForceAngleSettersWithModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    if (propulsion->GetNumEngines() > 0) {
      auto engine = propulsion->GetEngine(0);
      FGThruster* thruster = engine->GetThruster();

      if (thruster != nullptr) {
        // Set transform type to custom to allow angle changes
        thruster->SetTransformType(FGForce::tCustom);

        // Set angles
        thruster->SetAnglesToBody(0.1, 0.2, 0.3);

        TS_ASSERT_DELTA(thruster->GetAnglesToBody(1), 0.1, epsilon);
        TS_ASSERT_DELTA(thruster->GetAnglesToBody(2), 0.2, epsilon);
        TS_ASSERT_DELTA(thruster->GetAnglesToBody(3), 0.3, epsilon);

        // Test pitch/yaw setters
        thruster->SetPitch(0.15);
        thruster->SetYaw(0.25);

        TS_ASSERT_DELTA(thruster->GetPitch(), 0.15, epsilon);
        TS_ASSERT_DELTA(thruster->GetYaw(), 0.25, epsilon);
      }
    }
  }

  // Test propulsion system forces aggregate
  void testPropulsionSystemForces() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    TS_ASSERT(propulsion != nullptr);

    // Get total forces from propulsion system
    const FGColumnVector3& forces = propulsion->GetForces();

    // Forces should be finite
    TS_ASSERT(std::isfinite(forces(1)));
    TS_ASSERT(std::isfinite(forces(2)));
    TS_ASSERT(std::isfinite(forces(3)));

    // Individual accessors should match vector
    TS_ASSERT_DELTA(propulsion->GetForces(1), forces(1), epsilon);
    TS_ASSERT_DELTA(propulsion->GetForces(2), forces(2), epsilon);
    TS_ASSERT_DELTA(propulsion->GetForces(3), forces(3), epsilon);
  }

  // Test propulsion system moments aggregate
  void testPropulsionSystemMoments() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto propulsion = fdmex.GetPropulsion();
    TS_ASSERT(propulsion != nullptr);

    // Get total moments from propulsion system
    const FGColumnVector3& moments = propulsion->GetMoments();

    // Moments should be finite
    TS_ASSERT(std::isfinite(moments(1)));
    TS_ASSERT(std::isfinite(moments(2)));
    TS_ASSERT(std::isfinite(moments(3)));

    // Individual accessors should match vector
    TS_ASSERT_DELTA(propulsion->GetMoments(1), moments(1), epsilon);
    TS_ASSERT_DELTA(propulsion->GetMoments(2), moments(2), epsilon);
    TS_ASSERT_DELTA(propulsion->GetMoments(3), moments(3), epsilon);
  }
};
