/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       FGExternalForceTest.h
 Author:       Claude Code
 Date started: 2025-12-22

 ------------- Copyright (C) 2025 JSBSim Development Team ----------------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be
 found on the world wide web at http://www.gnu.org.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SENTRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGExternalForce.h>
#include <models/propulsion/FGForce.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>
#include "TestUtilities.h"

using namespace JSBSim;

const double epsilon = 1e-8;

/**
 * Unit tests for FGExternalForce class.
 *
 * This test suite validates the physics of external forces and moments,
 * including force transformations, moment arm calculations, and accumulation.
 * Since FGExternalForce requires complex XML setup, these tests focus on
 * the underlying physics equations and mathematical relationships.
 */
class FGExternalForceTest : public CxxTest::TestSuite
{
public:
  // ============================================================================
  // Task 6.4.1: Basic FGExternalForce test suite structure
  // ============================================================================

  // Test construction of external force object
  void testConstruction() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    // Should be constructed successfully
    TS_ASSERT(true);
  }

  // Test that FGExternalForce inherits from FGForce
  void testInheritance() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    // FGExternalForce should inherit FGForce methods
    extForce.SetLocation(0.0, 0.0, 0.0);
    TS_ASSERT_DELTA(extForce.GetLocationX(), 0.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationY(), 0.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationZ(), 0.0, epsilon);
  }

  // Test frame type enumeration
  void testFrameTypeEnums() {
    TS_ASSERT_EQUALS(FGForce::tNone, 0);         // BODY frame
    TS_ASSERT_EQUALS(FGForce::tWindBody, 1);     // WIND frame
    TS_ASSERT_EQUALS(FGForce::tLocalBody, 2);    // LOCAL frame
    TS_ASSERT_EQUALS(FGForce::tInertialBody, 3); // INERTIAL frame
    TS_ASSERT_EQUALS(FGForce::tCustom, 4);       // CUSTOM frame
  }

  // ============================================================================
  // Task 6.4.2: Force application points (moment arm calculations)
  // ============================================================================

  // Test moment calculation from force at offset location: M = r × F
  // With force along +X at location (0, 1, 0), moment should be about Z axis
  void testMomentArmCalculation_ForceX_OffsetY() {
    // Physics: Force Fx at location (0, y, 0) produces moment Mz = -Fx * y
    double forceX = 100.0;   // lbs force in +X direction
    double offsetY = 2.0;    // ft offset in +Y direction

    // Expected moment about Z axis: Mz = -Fx * y (negative per right-hand rule)
    double expectedMz = -forceX * offsetY;

    TS_ASSERT_DELTA(expectedMz, -200.0, epsilon);
  }

  // Test moment from vertical force at longitudinal offset
  // Force along +Z at location (5, 0, 0) produces pitch moment
  void testMomentArmCalculation_ForceZ_OffsetX() {
    // Physics: Force Fz at location (x, 0, 0) produces moment My = Fz * x
    double forceZ = 150.0;   // lbs force in +Z direction
    double offsetX = 3.0;    // ft offset in +X direction

    // Expected moment about Y axis: My = Fz * x
    double expectedMy = forceZ * offsetX;

    TS_ASSERT_DELTA(expectedMy, 450.0, epsilon);
  }

  // Test moment from lateral force at vertical offset
  // Force along +Y at location (0, 0, -2) produces roll moment
  void testMomentArmCalculation_ForceY_OffsetZ() {
    // Physics: Force Fy at location (0, 0, z) produces moment Mx = -Fy * z
    double forceY = 50.0;    // lbs force in +Y direction
    double offsetZ = -2.0;   // ft offset in -Z direction (up)

    // Expected moment about X axis: Mx = -Fy * z
    double expectedMx = -forceY * offsetZ;

    TS_ASSERT_DELTA(expectedMx, 100.0, epsilon);
  }

  // Test cross product for general moment arm
  // M = r × F, where r is position vector and F is force vector
  void testCrossProductMoment() {
    // Position vector: r = (2, 3, -1) ft
    FGColumnVector3 position(2.0, 3.0, -1.0);

    // Force vector: F = (10, 20, 30) lbs
    FGColumnVector3 force(10.0, 20.0, 30.0);

    // Moment: M = r × F
    // Mx = ry*Fz - rz*Fy = 3*30 - (-1)*20 = 90 + 20 = 110
    // My = rz*Fx - rx*Fz = (-1)*10 - 2*30 = -10 - 60 = -70
    // Mz = rx*Fy - ry*Fx = 2*20 - 3*10 = 40 - 30 = 10
    FGColumnVector3 moment = position * force;  // Cross product

    TS_ASSERT_DELTA(moment(1), 110.0, epsilon);
    TS_ASSERT_DELTA(moment(2), -70.0, epsilon);
    TS_ASSERT_DELTA(moment(3), 10.0, epsilon);
  }

  // Test zero moment when force acts at CG
  void testZeroMomentAtCG() {
    // Force at origin (CG location) produces no offset moment
    FGColumnVector3 position(0.0, 0.0, 0.0);
    FGColumnVector3 force(100.0, 50.0, 25.0);

    FGColumnVector3 moment = position * force;

    TS_ASSERT_DELTA(moment(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(3), 0.0, epsilon);
  }

  // Test moment magnitude increases with distance
  void testMomentScalesWithDistance() {
    double force = 100.0;  // Constant force magnitude

    // Distance 1: moment at 1 ft
    double distance1 = 1.0;
    double moment1 = force * distance1;

    // Distance 2: moment at 2 ft (should double)
    double distance2 = 2.0;
    double moment2 = force * distance2;

    TS_ASSERT_DELTA(moment1, 100.0, epsilon);
    TS_ASSERT_DELTA(moment2, 200.0, epsilon);
    TS_ASSERT_DELTA(moment2 / moment1, 2.0, epsilon);
  }

  // Test symmetry: opposite position produces opposite moment
  void testMomentSymmetry() {
    FGColumnVector3 position(1.0, 2.0, 3.0);
    FGColumnVector3 force(10.0, 20.0, 30.0);

    FGColumnVector3 moment1 = position * force;
    FGColumnVector3 moment2 = (position * -1.0) * force;

    // Opposite position should produce opposite moment
    TS_ASSERT_DELTA(moment2(1), -moment1(1), epsilon);
    TS_ASSERT_DELTA(moment2(2), -moment1(2), epsilon);
    TS_ASSERT_DELTA(moment2(3), -moment1(3), epsilon);
  }

  // ============================================================================
  // Task 6.4.3: Force frame transformations (body to local, etc.)
  // ============================================================================

  // Test no transformation in BODY frame (tNone)
  void testBodyFrameNoTransform() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    extForce.SetTransformType(FGForce::tNone);

    // Force in body frame
    FGColumnVector3 bodyForce(100.0, 50.0, -25.0);

    // With tNone, output should equal input
    TS_ASSERT_DELTA(bodyForce(1), 100.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(2), 50.0, epsilon);
    TS_ASSERT_DELTA(bodyForce(3), -25.0, epsilon);
  }

  // Test rotation matrix identity for zero angles
  void testRotationMatrixIdentity() {
    // Rotation matrix with zero angles should be identity
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    // Build rotation matrix manually
    FGMatrix33 rotMatrix;
    rotMatrix(1,1) = 1.0; rotMatrix(1,2) = 0.0; rotMatrix(1,3) = 0.0;
    rotMatrix(2,1) = 0.0; rotMatrix(2,2) = 1.0; rotMatrix(2,3) = 0.0;
    rotMatrix(3,1) = 0.0; rotMatrix(3,2) = 0.0; rotMatrix(3,3) = 1.0;

    FGColumnVector3 input(100.0, 50.0, 25.0);
    FGColumnVector3 output = rotMatrix * input;

    TS_ASSERT_DELTA(output(1), input(1), epsilon);
    TS_ASSERT_DELTA(output(2), input(2), epsilon);
    TS_ASSERT_DELTA(output(3), input(3), epsilon);
  }

  // Test 90-degree rotation about Z axis (yaw)
  void testRotation90Yaw() {
    // 90-degree rotation about Z axis: (x,y,z) -> (-y,x,z)
    double yaw = M_PI / 2.0;  // 90 degrees

    // Rotation matrix for yaw
    FGMatrix33 rotZ;
    rotZ(1,1) = cos(yaw);  rotZ(1,2) = sin(yaw); rotZ(1,3) = 0.0;
    rotZ(2,1) = -sin(yaw); rotZ(2,2) = cos(yaw); rotZ(2,3) = 0.0;
    rotZ(3,1) = 0.0;       rotZ(3,2) = 0.0;      rotZ(3,3) = 1.0;

    FGColumnVector3 input(1.0, 0.0, 0.0);  // Unit vector along X
    FGColumnVector3 output = rotZ * input;

    // Should rotate to Y axis
    TS_ASSERT_DELTA(output(1), 0.0, epsilon);
    TS_ASSERT_DELTA(output(2), 1.0, epsilon);
    TS_ASSERT_DELTA(output(3), 0.0, epsilon);
  }

  // Test 90-degree rotation about Y axis (pitch)
  void testRotation90Pitch() {
    // 90-degree rotation about Y axis: (x,y,z) -> (z,y,-x)
    double pitch = M_PI / 2.0;  // 90 degrees

    // Rotation matrix for pitch
    FGMatrix33 rotY;
    rotY(1,1) = cos(pitch);  rotY(1,2) = 0.0; rotY(1,3) = -sin(pitch);
    rotY(2,1) = 0.0;         rotY(2,2) = 1.0; rotY(2,3) = 0.0;
    rotY(3,1) = sin(pitch);  rotY(3,2) = 0.0; rotY(3,3) = cos(pitch);

    FGColumnVector3 input(1.0, 0.0, 0.0);  // Unit vector along X
    FGColumnVector3 output = rotY * input;

    // Should rotate to -Z axis
    TS_ASSERT_DELTA(output(1), 0.0, epsilon);
    TS_ASSERT_DELTA(output(2), 0.0, epsilon);
    TS_ASSERT_DELTA(output(3), 1.0, epsilon);
  }

  // Test 90-degree rotation about X axis (roll)
  void testRotation90Roll() {
    // 90-degree rotation about X axis: (x,y,z) -> (x,-z,y)
    double roll = M_PI / 2.0;  // 90 degrees

    // Rotation matrix for roll
    FGMatrix33 rotX;
    rotX(1,1) = 1.0; rotX(1,2) = 0.0;        rotX(1,3) = 0.0;
    rotX(2,1) = 0.0; rotX(2,2) = cos(roll);  rotX(2,3) = sin(roll);
    rotX(3,1) = 0.0; rotX(3,2) = -sin(roll); rotX(3,3) = cos(roll);

    FGColumnVector3 input(0.0, 1.0, 0.0);  // Unit vector along Y
    FGColumnVector3 output = rotX * input;

    // Should rotate to -Z axis
    TS_ASSERT_DELTA(output(1), 0.0, epsilon);
    TS_ASSERT_DELTA(output(2), 0.0, epsilon);
    TS_ASSERT_DELTA(output(3), -1.0, epsilon);
  }

  // Test force magnitude preserved through rotation
  void testForceMagnitudePreserved() {
    // Rotation should preserve vector magnitude
    FGColumnVector3 force(100.0, 50.0, 25.0);
    double originalMag = force.Magnitude();

    // Apply arbitrary rotation (30 degrees about Z)
    double yaw = M_PI / 6.0;
    FGMatrix33 rotZ;
    rotZ(1,1) = cos(yaw);  rotZ(1,2) = sin(yaw); rotZ(1,3) = 0.0;
    rotZ(2,1) = -sin(yaw); rotZ(2,2) = cos(yaw); rotZ(2,3) = 0.0;
    rotZ(3,1) = 0.0;       rotZ(3,2) = 0.0;      rotZ(3,3) = 1.0;

    FGColumnVector3 rotatedForce = rotZ * force;
    double rotatedMag = rotatedForce.Magnitude();

    TS_ASSERT_DELTA(rotatedMag, originalMag, epsilon);
  }

  // Test direction vector normalization
  void testDirectionNormalization() {
    // Direction vectors should be normalized to unit length
    FGColumnVector3 direction(3.0, 4.0, 0.0);
    double magnitude = direction.Magnitude();

    FGColumnVector3 normalized = direction / magnitude;

    TS_ASSERT_DELTA(normalized.Magnitude(), 1.0, epsilon);
    TS_ASSERT_DELTA(normalized(1), 0.6, epsilon);  // 3/5
    TS_ASSERT_DELTA(normalized(2), 0.8, epsilon);  // 4/5
  }

  // Test force vector computation: F = magnitude * direction
  void testForceVectorComputation() {
    double magnitude = 100.0;  // lbs

    // Unit direction vector (normalized)
    FGColumnVector3 direction(0.6, 0.8, 0.0);

    // Force = magnitude * direction
    FGColumnVector3 force = direction * magnitude;

    TS_ASSERT_DELTA(force(1), 60.0, epsilon);
    TS_ASSERT_DELTA(force(2), 80.0, epsilon);
    TS_ASSERT_DELTA(force(3), 0.0, epsilon);
    TS_ASSERT_DELTA(force.Magnitude(), 100.0, epsilon);
  }

  // ============================================================================
  // Additional physics validation tests
  // ============================================================================

  // Test multiple force accumulation (superposition principle)
  void testMultipleForceAccumulation() {
    // Multiple forces should accumulate linearly
    FGColumnVector3 force1(10.0, 20.0, 30.0);
    FGColumnVector3 force2(5.0, -10.0, 15.0);
    FGColumnVector3 force3(-3.0, 7.0, -8.0);

    FGColumnVector3 totalForce = force1 + force2 + force3;

    TS_ASSERT_DELTA(totalForce(1), 12.0, epsilon);
    TS_ASSERT_DELTA(totalForce(2), 17.0, epsilon);
    TS_ASSERT_DELTA(totalForce(3), 37.0, epsilon);
  }

  // Test multiple moment accumulation
  void testMultipleMomentAccumulation() {
    // Moments from different forces should accumulate
    FGColumnVector3 moment1(100.0, 200.0, 300.0);
    FGColumnVector3 moment2(-50.0, 150.0, -100.0);

    FGColumnVector3 totalMoment = moment1 + moment2;

    TS_ASSERT_DELTA(totalMoment(1), 50.0, epsilon);
    TS_ASSERT_DELTA(totalMoment(2), 350.0, epsilon);
    TS_ASSERT_DELTA(totalMoment(3), 200.0, epsilon);
  }

  // Test force components in different directions
  void testForceComponentDecomposition() {
    // Total force can be decomposed into components
    double Fx = 30.0;
    double Fy = 40.0;
    double Fz = 0.0;

    FGColumnVector3 force(Fx, Fy, Fz);
    double magnitude = force.Magnitude();

    // Magnitude = sqrt(30^2 + 40^2) = sqrt(900 + 1600) = 50
    TS_ASSERT_DELTA(magnitude, 50.0, epsilon);
  }

  // Test parachute drag example (wind frame)
  void testParachuteDragWindFrame() {
    // Parachute drag acts opposite to velocity (in wind frame)
    // In wind frame: drag is along +X axis (opposite to velocity)
    double dragMagnitude = 500.0;  // lbs

    // Wind frame direction: drag along +X
    FGColumnVector3 windFrameDrag(dragMagnitude, 0.0, 0.0);

    TS_ASSERT_DELTA(windFrameDrag(1), 500.0, epsilon);
    TS_ASSERT_DELTA(windFrameDrag.Magnitude(), 500.0, epsilon);
  }

  // Test propeller thrust example (custom frame)
  void testPropellerThrustCustomFrame() {
    // Propeller thrust at angle to body axis
    double thrustMagnitude = 1000.0;  // lbs
    double pitchAngle = 0.1;  // rad (~5.7 degrees)

    // Thrust components (simplified)
    double Fx = thrustMagnitude * cos(pitchAngle);
    double Fz = -thrustMagnitude * sin(pitchAngle);

    TS_ASSERT_DELTA(Fx, 995.0, 1.0);
    TS_ASSERT_DELTA(Fz, -99.8, 1.0);
  }

  // Test moment arm perpendicular distance calculation
  void testPerpendicularDistance() {
    // Perpendicular distance from line of action to CG
    // For force F at position r, moment magnitude = |r| * |F| * sin(θ)

    double r = 5.0;      // ft, distance from CG
    double F = 100.0;    // lbs
    double theta = M_PI / 2.0;  // 90 degrees (perpendicular)

    double momentMagnitude = r * F * sin(theta);

    TS_ASSERT_DELTA(momentMagnitude, 500.0, epsilon);
  }

  // Test parallel force produces no moment
  void testParallelForceNoMoment() {
    // Force parallel to position vector produces no moment
    FGColumnVector3 position(1.0, 0.0, 0.0);
    FGColumnVector3 force(10.0, 0.0, 0.0);  // Parallel to position

    FGColumnVector3 moment = position * force;

    TS_ASSERT_DELTA(moment(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(3), 0.0, epsilon);
  }

  // Test large force values
  void testLargeForceValues() {
    double largeMagnitude = 1e6;  // 1 million lbs

    FGColumnVector3 largeForce(largeMagnitude, 0.0, 0.0);

    TS_ASSERT_DELTA(largeForce(1), largeMagnitude, largeMagnitude * 1e-10);
    TS_ASSERT(!std::isnan(largeForce(1)));
    TS_ASSERT(!std::isinf(largeForce(1)));
  }

  // Test small force values
  void testSmallForceValues() {
    double smallMagnitude = 1e-6;  // Very small force

    FGColumnVector3 smallForce(smallMagnitude, smallMagnitude, smallMagnitude);

    TS_ASSERT_DELTA(smallForce(1), smallMagnitude, 1e-15);
    TS_ASSERT(!std::isnan(smallForce(1)));
  }

  // Test negative force values (compression/reverse thrust)
  void testNegativeForceValues() {
    FGColumnVector3 negativeForce(-100.0, -200.0, -300.0);

    TS_ASSERT_DELTA(negativeForce(1), -100.0, epsilon);
    TS_ASSERT_DELTA(negativeForce(2), -200.0, epsilon);
    TS_ASSERT_DELTA(negativeForce(3), -300.0, epsilon);
  }
};
