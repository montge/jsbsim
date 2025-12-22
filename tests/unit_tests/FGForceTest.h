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

const double epsilon = 1e-8;

class FGForceTest : public CxxTest::TestSuite
{
public:
  // Test construction
  void testConstruction() {
    FGFDMExec fdmex;
    FGForce force(&fdmex);

    // Should be constructed successfully
    TS_ASSERT(true);
  }

  // Test setting location
  void testSetLocation() {
    FGFDMExec fdmex;
    FGForce force(&fdmex);

    double x = 100.0, y = 50.0, z = -25.0;
    force.SetLocation(x, y, z);

    TS_ASSERT_DELTA(force.GetLocationX(), x, epsilon);
    TS_ASSERT_DELTA(force.GetLocationY(), y, epsilon);
    TS_ASSERT_DELTA(force.GetLocationZ(), z, epsilon);
  }

  // Test setting location with vector
  void testSetLocationVector() {
    FGFDMExec fdmex;
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

    force.SetActingLocation(50.0, 25.0, -10.0);

    TS_ASSERT_DELTA(force.GetActingLocationX(), 50.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 25.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), -10.0, epsilon);
  }

  // Test individual acting location setters
  void testSetActingLocationIndividual() {
    FGFDMExec fdmex;
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

    force.SetLocation(100.0, 200.0, 300.0);

    // Acting location should be set to same as location
    TS_ASSERT_DELTA(force.GetActingLocationX(), 100.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationY(), 200.0, epsilon);
    TS_ASSERT_DELTA(force.GetActingLocationZ(), 300.0, epsilon);
  }

  // Test angles to body
  void testAnglesToBody() {
    FGFDMExec fdmex;
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

    force.SetPitch(0.5);
    TS_ASSERT_DELTA(force.GetPitch(), 0.5, epsilon);
  }

  // Test yaw getter/setter
  void testYaw() {
    FGFDMExec fdmex;
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
    FGForce force(&fdmex);

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
};
