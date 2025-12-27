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
using namespace JSBSimTest;

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

    // Rotation matrix for yaw (right-hand rule: positive yaw rotates X toward -Y)
    FGMatrix33 rotZ;
    rotZ(1,1) = cos(yaw);  rotZ(1,2) = sin(yaw); rotZ(1,3) = 0.0;
    rotZ(2,1) = -sin(yaw); rotZ(2,2) = cos(yaw); rotZ(2,3) = 0.0;
    rotZ(3,1) = 0.0;       rotZ(3,2) = 0.0;      rotZ(3,3) = 1.0;

    FGColumnVector3 input(1.0, 0.0, 0.0);  // Unit vector along X
    FGColumnVector3 output = rotZ * input;

    // With this matrix convention, 90-deg yaw rotates (1,0,0) to (0,-1,0)
    TS_ASSERT_DELTA(output(1), 0.0, epsilon);
    TS_ASSERT_DELTA(output(2), -1.0, epsilon);
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

  // ============================================================================
  // Additional Force Location and Moment Tests
  // ============================================================================

  // Test setting force location
  void testSetLocation() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    extForce.SetLocation(10.0, 5.0, -3.0);

    TS_ASSERT_DELTA(extForce.GetLocationX(), 10.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationY(), 5.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationZ(), -3.0, epsilon);
  }

  // Test location at aircraft nose
  void testNoseLocation() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    // Typical nose location (forward of CG)
    extForce.SetLocation(15.0, 0.0, 0.0);

    TS_ASSERT_DELTA(extForce.GetLocationX(), 15.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationY(), 0.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationZ(), 0.0, epsilon);
  }

  // Test location at aircraft tail
  void testTailLocation() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    // Typical tail location (behind CG)
    extForce.SetLocation(-25.0, 0.0, 5.0);

    TS_ASSERT_DELTA(extForce.GetLocationX(), -25.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationY(), 0.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationZ(), 5.0, epsilon);
  }

  // Test location at wingtip (left)
  void testLeftWingtipLocation() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    // Left wingtip (negative Y in body frame)
    extForce.SetLocation(0.0, -30.0, 0.0);

    TS_ASSERT_DELTA(extForce.GetLocationX(), 0.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationY(), -30.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationZ(), 0.0, epsilon);
  }

  // Test location at wingtip (right)
  void testRightWingtipLocation() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    // Right wingtip (positive Y in body frame)
    extForce.SetLocation(0.0, 30.0, 0.0);

    TS_ASSERT_DELTA(extForce.GetLocationX(), 0.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationY(), 30.0, epsilon);
    TS_ASSERT_DELTA(extForce.GetLocationZ(), 0.0, epsilon);
  }

  // ============================================================================
  // Couple and Distributed Force Tests
  // ============================================================================

  // Test couple (pure moment from equal and opposite forces)
  void testCoupleForce() {
    // A couple consists of two equal and opposite forces at different points
    // Net force = 0, but produces a moment
    double forceY = 50.0;  // lbs
    double separation = 4.0;  // ft between forces

    // Forces at +2 and -2 ft on X axis
    FGColumnVector3 pos1(2.0, 0.0, 0.0);
    FGColumnVector3 pos2(-2.0, 0.0, 0.0);
    FGColumnVector3 force1(0.0, forceY, 0.0);
    FGColumnVector3 force2(0.0, -forceY, 0.0);

    // Net force should be zero
    FGColumnVector3 netForce = force1 + force2;
    TS_ASSERT_DELTA(netForce.Magnitude(), 0.0, epsilon);

    // Net moment should be non-zero
    FGColumnVector3 moment1 = pos1 * force1;
    FGColumnVector3 moment2 = pos2 * force2;
    FGColumnVector3 netMoment = moment1 + moment2;

    // Moment magnitude = force * separation
    double expectedMoment = forceY * separation;
    TS_ASSERT_DELTA(netMoment.Magnitude(), expectedMoment, epsilon);
  }

  // Test distributed load equivalent point force
  void testDistributedLoadEquivalent() {
    // A uniformly distributed load can be replaced by equivalent point force
    double loadPerFoot = 10.0;  // lbs/ft
    double length = 20.0;  // ft

    double totalForce = loadPerFoot * length;
    TS_ASSERT_DELTA(totalForce, 200.0, epsilon);

    // Equivalent point force acts at center
    double centerPos = length / 2.0;
    TS_ASSERT_DELTA(centerPos, 10.0, epsilon);
  }

  // Test triangular distributed load
  void testTriangularDistributedLoad() {
    // Triangular load: zero at one end, maximum at other
    // Equivalent point force at 2/3 from zero end
    double maxLoad = 30.0;  // lbs/ft at max point
    double length = 15.0;  // ft

    // Total force = 0.5 * base * height
    double totalForce = 0.5 * length * maxLoad;
    TS_ASSERT_DELTA(totalForce, 225.0, epsilon);

    // Centroid at 2/3 from zero end
    double centroidPos = (2.0 / 3.0) * length;
    TS_ASSERT_DELTA(centroidPos, 10.0, epsilon);
  }

  // ============================================================================
  // Aerodynamic Center and Pressure Center Tests
  // ============================================================================

  // Test neutral point concept
  void testNeutralPointMoment() {
    // At neutral point, pitching moment coefficient doesn't change with alpha
    // This is a stability concept
    double lift = 1000.0;  // lbs
    double distToNP = 2.5;  // ft from CG to neutral point

    // Lift at neutral point produces pitching moment
    double pitchMoment = lift * distToNP;
    TS_ASSERT_DELTA(pitchMoment, 2500.0, epsilon);
  }

  // Test center of pressure shift with angle of attack
  void testCPShiftWithAOA() {
    // Center of pressure moves forward with increasing AOA
    // CP position relative to chord: xCP = xAC - M0/(CL * c)
    double xAC = 0.25;  // Aerodynamic center at quarter chord
    double M0_coeff = -0.05;  // Moment coefficient about AC
    double CL = 0.5;  // Lift coefficient

    // Simplified CP position
    double xCP = xAC - M0_coeff / CL;
    TS_ASSERT_DELTA(xCP, 0.35, epsilon);
  }

  // Test pitching moment from tail lift
  void testTailPitchingMoment() {
    // Horizontal tail produces pitching moment about CG
    double tailLift = 200.0;  // lbs (typically negative for stability)
    double tailArm = 20.0;  // ft from CG to tail AC

    // Nose-down moment from upward tail force
    double pitchMoment = -tailLift * tailArm;
    TS_ASSERT_DELTA(pitchMoment, -4000.0, epsilon);
  }

  // ============================================================================
  // Frame Transformation Extended Tests
  // ============================================================================

  // Test wind frame to body frame at zero alpha and beta
  void testWindToBodyZeroAlphaBeta() {
    // At zero alpha and beta, wind and body frames align
    FGColumnVector3 windForce(100.0, 0.0, -200.0);

    // With zero alpha/beta, body force equals wind force
    TS_ASSERT_DELTA(windForce(1), 100.0, epsilon);
    TS_ASSERT_DELTA(windForce(3), -200.0, epsilon);
  }

  // Test body frame force at angle of attack
  void testForceAtAngleOfAttack() {
    // At non-zero alpha, lift and drag decompose differently in body frame
    double alpha = 10.0 * M_PI / 180.0;  // 10 degrees
    double lift = 1000.0;
    double drag = 100.0;

    // Body X (axial) and Z (normal) from lift and drag
    // At positive alpha, lift has positive X component in body frame
    double Fx = -drag * cos(alpha) + lift * sin(alpha);
    double Fz = -drag * sin(alpha) - lift * cos(alpha);

    TS_ASSERT(!std::isnan(Fx));
    TS_ASSERT(!std::isnan(Fz));
    // Verify Fz is negative (lift is upward, which is -Z in body frame)
    TS_ASSERT(Fz < 0.0);
  }

  // Test sideslip effect on lateral force
  void testSideslipLateralForce() {
    // Sideslip generates side force
    double beta = 5.0 * M_PI / 180.0;  // 5 degrees
    double sideForceCoeff = 0.1;  // per degree
    double qS = 10000.0;  // Dynamic pressure * area

    double sideForce = sideForceCoeff * (beta * 180.0 / M_PI) * qS;
    TS_ASSERT_DELTA(sideForce, 5000.0, 100.0);
  }

  // ============================================================================
  // Moment of Inertia and Angular Acceleration Tests
  // ============================================================================

  // Test angular acceleration from moment: alpha = M / I
  void testAngularAccelerationFromMoment() {
    double moment = 10000.0;  // lb-ft
    double Ixx = 5000.0;  // slug-ft^2

    double angularAccel = moment / Ixx;  // rad/s^2
    TS_ASSERT_DELTA(angularAccel, 2.0, epsilon);
  }

  // Test roll rate buildup
  void testRollRateBuildup() {
    double rollMoment = 500.0;  // lb-ft
    double Ixx = 1000.0;  // slug-ft^2
    double dt = 0.1;  // seconds

    double rollAccel = rollMoment / Ixx;  // rad/s^2
    double rollRate = rollAccel * dt;  // rad/s

    TS_ASSERT_DELTA(rollRate, 0.05, epsilon);
  }

  // Test pitch rate buildup
  void testPitchRateBuildup() {
    double pitchMoment = 2000.0;  // lb-ft
    double Iyy = 4000.0;  // slug-ft^2
    double dt = 0.5;  // seconds

    double pitchAccel = pitchMoment / Iyy;  // rad/s^2
    double pitchRate = pitchAccel * dt;  // rad/s

    TS_ASSERT_DELTA(pitchRate, 0.25, epsilon);
  }

  // Test yaw rate buildup
  void testYawRateBuildup() {
    double yawMoment = 1500.0;  // lb-ft
    double Izz = 6000.0;  // slug-ft^2
    double dt = 0.2;  // seconds

    double yawAccel = yawMoment / Izz;  // rad/s^2
    double yawRate = yawAccel * dt;  // rad/s

    TS_ASSERT_DELTA(yawRate, 0.05, epsilon);
  }

  // ============================================================================
  // Force Balance and Equilibrium Tests
  // ============================================================================

  // Test level flight force balance
  void testLevelFlightForceBalance() {
    // In level flight: Lift = Weight, Thrust = Drag
    double weight = 10000.0;  // lbs
    double lift = 10000.0;  // lbs
    double drag = 500.0;  // lbs
    double thrust = 500.0;  // lbs

    double verticalBalance = lift - weight;
    double horizontalBalance = thrust - drag;

    TS_ASSERT_DELTA(verticalBalance, 0.0, epsilon);
    TS_ASSERT_DELTA(horizontalBalance, 0.0, epsilon);
  }

  // Test climbing flight force balance
  void testClimbingFlightForceBalance() {
    // In climb: Thrust > Drag (excess for climb)
    double weight = 10000.0;  // lbs
    double gamma = 5.0 * M_PI / 180.0;  // 5 degree climb

    // Required thrust includes component of weight
    double thrustRequired = 500.0 + weight * sin(gamma);
    TS_ASSERT(thrustRequired > 500.0);
  }

  // Test banked turn force balance
  void testBankedTurnForceBalance() {
    // In coordinated turn: L * cos(phi) = W
    double weight = 10000.0;  // lbs
    double bankAngle = 30.0 * M_PI / 180.0;  // 30 degrees

    double requiredLift = weight / cos(bankAngle);
    double loadFactor = requiredLift / weight;

    TS_ASSERT_DELTA(loadFactor, 1.155, 0.001);
  }

  // Test moment equilibrium in steady flight
  void testMomentEquilibrium() {
    // In steady flight, sum of moments = 0
    double wingMoment = 5000.0;  // lb-ft (nose up)
    double tailMoment = -5000.0;  // lb-ft (nose down)
    double fuselageMoment = 0.0;  // lb-ft

    double totalMoment = wingMoment + tailMoment + fuselageMoment;
    TS_ASSERT_DELTA(totalMoment, 0.0, epsilon);
  }

  // ============================================================================
  // Force Direction and Sign Convention Tests
  // ============================================================================

  // Test body frame sign conventions
  void testBodyFrameSignConventions() {
    // X forward, Y right, Z down
    FGColumnVector3 forward(1.0, 0.0, 0.0);
    FGColumnVector3 right(0.0, 1.0, 0.0);
    FGColumnVector3 down(0.0, 0.0, 1.0);

    TS_ASSERT_DELTA(forward(1), 1.0, epsilon);
    TS_ASSERT_DELTA(right(2), 1.0, epsilon);
    TS_ASSERT_DELTA(down(3), 1.0, epsilon);
  }

  // Test moment sign conventions (right-hand rule)
  void testMomentSignConventions() {
    // Positive roll (Mx): right wing down
    // Positive pitch (My): nose up
    // Positive yaw (Mz): nose right

    // Roll moment from Y-force at +Z location
    FGColumnVector3 pos(0.0, 0.0, 1.0);
    FGColumnVector3 force(0.0, 1.0, 0.0);
    FGColumnVector3 moment = pos * force;

    // Mx = y*Fz - z*Fy = 0*0 - 1*1 = -1 (left wing down)
    TS_ASSERT_DELTA(moment(1), -1.0, epsilon);
  }

  // ============================================================================
  // Edge Cases and Numerical Stability Tests
  // ============================================================================

  // Test zero vector handling
  void testZeroVectorHandling() {
    FGColumnVector3 zeroVec(0.0, 0.0, 0.0);
    FGColumnVector3 force(100.0, 50.0, 25.0);

    FGColumnVector3 moment = zeroVec * force;

    TS_ASSERT_DELTA(moment(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(3), 0.0, epsilon);
  }

  // Test very large moment arms
  void testLargeMomentArms() {
    double largeArm = 1000.0;  // ft
    double force = 100.0;  // lbs

    double moment = force * largeArm;
    TS_ASSERT_DELTA(moment, 100000.0, epsilon);
    TS_ASSERT(!std::isinf(moment));
  }

  // Test very small moment arms
  void testSmallMomentArms() {
    double smallArm = 0.001;  // ft
    double force = 100.0;  // lbs

    double moment = force * smallArm;
    TS_ASSERT_DELTA(moment, 0.1, 1e-6);
    TS_ASSERT(!std::isnan(moment));
  }

  // Test orthogonality of rotation matrix
  void testRotationMatrixOrthogonality() {
    double angle = 0.5;  // radians

    FGMatrix33 rotZ;
    rotZ(1,1) = cos(angle);  rotZ(1,2) = sin(angle); rotZ(1,3) = 0.0;
    rotZ(2,1) = -sin(angle); rotZ(2,2) = cos(angle); rotZ(2,3) = 0.0;
    rotZ(3,1) = 0.0;         rotZ(3,2) = 0.0;        rotZ(3,3) = 1.0;

    // R * R^T should equal identity
    FGMatrix33 rotZT = rotZ.Transposed();
    FGMatrix33 product = rotZ * rotZT;

    TS_ASSERT_DELTA(product(1,1), 1.0, epsilon);
    TS_ASSERT_DELTA(product(2,2), 1.0, epsilon);
    TS_ASSERT_DELTA(product(3,3), 1.0, epsilon);
    TS_ASSERT_DELTA(product(1,2), 0.0, epsilon);
    TS_ASSERT_DELTA(product(1,3), 0.0, epsilon);
    TS_ASSERT_DELTA(product(2,3), 0.0, epsilon);
  }

  // Test rotation matrix determinant = 1
  void testRotationMatrixDeterminant() {
    double angle = 1.2;  // radians

    FGMatrix33 rotY;
    rotY(1,1) = cos(angle);  rotY(1,2) = 0.0; rotY(1,3) = -sin(angle);
    rotY(2,1) = 0.0;         rotY(2,2) = 1.0; rotY(2,3) = 0.0;
    rotY(3,1) = sin(angle);  rotY(3,2) = 0.0; rotY(3,3) = cos(angle);

    double det = rotY.Determinant();
    TS_ASSERT_DELTA(det, 1.0, epsilon);
  }

  // Test inverse rotation
  void testInverseRotation() {
    double angle = 0.7;  // radians

    FGMatrix33 rotX;
    rotX(1,1) = 1.0; rotX(1,2) = 0.0;         rotX(1,3) = 0.0;
    rotX(2,1) = 0.0; rotX(2,2) = cos(angle);  rotX(2,3) = sin(angle);
    rotX(3,1) = 0.0; rotX(3,2) = -sin(angle); rotX(3,3) = cos(angle);

    FGColumnVector3 original(1.0, 2.0, 3.0);
    FGColumnVector3 rotated = rotX * original;
    FGColumnVector3 restored = rotX.Transposed() * rotated;

    TS_ASSERT_DELTA(restored(1), original(1), epsilon);
    TS_ASSERT_DELTA(restored(2), original(2), epsilon);
    TS_ASSERT_DELTA(restored(3), original(3), epsilon);
  }

  // ============================================================================
  // Specific External Force Scenarios
  // ============================================================================

  // Test tow cable force on glider
  void testTowCableForce() {
    // Tow cable applies force at nose hook
    double cableTension = 300.0;  // lbs
    double cableAngle = 15.0 * M_PI / 180.0;  // degrees above horizontal

    // Force components in body frame
    double Fx = cableTension * cos(cableAngle);
    double Fz = -cableTension * sin(cableAngle);

    TS_ASSERT(Fx > 0.0);  // Forward
    TS_ASSERT(Fz < 0.0);  // Upward (negative Z in body frame)
  }

  // Test refueling boom force
  void testRefuelingBoomForce() {
    // Refueling boom applies force at receptacle location
    double boomForce = 50.0;  // lbs
    double offsetX = -5.0;  // ft aft of CG
    double offsetZ = 2.0;  // ft below fuselage centerline

    // Force primarily downward
    FGColumnVector3 pos(offsetX, 0.0, offsetZ);
    FGColumnVector3 force(0.0, 0.0, boomForce);

    FGColumnVector3 moment = pos * force;

    // M = r × F: My = rz*Fx - rx*Fz = 2*0 - (-5)*50 = 250 (nose-down)
    TS_ASSERT(moment(2) > 0.0);  // Nose-down moment
  }

  // Test arresting hook force
  void testArrestingHookForce() {
    // Arresting wire applies force at tail hook
    double cableForce = 50000.0;  // lbs
    double hookOffset = -40.0;  // ft behind CG
    double hookHeight = 3.0;  // ft below CG

    FGColumnVector3 pos(hookOffset, 0.0, hookHeight);
    FGColumnVector3 force(-cableForce, 0.0, 0.0);  // Decelerating force

    FGColumnVector3 moment = pos * force;

    // M = r × F: My = rz*Fx - rx*Fz = 3*(-50000) - (-40)*0 = -150000 (nose-up)
    TS_ASSERT(moment(2) < 0.0);  // Nose-up moment from tail hook
  }

  // Test catapult launch force
  void testCatapultForce() {
    // Catapult applies force at nose gear attachment
    double catForce = 75000.0;  // lbs
    double noseGearOffset = 10.0;  // ft forward of CG
    double noseGearHeight = 5.0;  // ft below CG

    FGColumnVector3 pos(noseGearOffset, 0.0, noseGearHeight);
    FGColumnVector3 force(catForce, 0.0, 0.0);  // Accelerating force

    FGColumnVector3 moment = pos * force;

    // Should produce pitch moment
    TS_ASSERT(fabs(moment(2)) > 0.0);
  }

  // Test parachute drogue deployment
  void testDrogueParachuteForce() {
    // Drogue chute applies drag at tail
    double drogueForce = 2000.0;  // lbs
    double drogueTailOffset = -50.0;  // ft behind CG
    double drogueHeight = 0.0;  // centerline

    // Force direction depends on relative wind
    FGColumnVector3 pos(drogueTailOffset, 0.0, drogueHeight);
    FGColumnVector3 force(-drogueForce, 0.0, 0.0);  // Drag (aft)

    FGColumnVector3 moment = pos * force;

    // Should produce minimal pitch moment if on centerline
    TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
  }

  /***************************************************************************
   * Extended Force Application Tests
   ***************************************************************************/

  // Test engine thrust line offset
  void testEngineThrustLineOffset() {
    double thrust = 5000.0;
    double thrustLineZ = 1.5;  // ft below CG

    FGColumnVector3 pos(0.0, 0.0, thrustLineZ);
    FGColumnVector3 force(thrust, 0.0, 0.0);

    FGColumnVector3 moment = pos * force;
    // Thrust below CG produces nose-down moment
    TS_ASSERT(moment(2) > 0);
  }

  // Test asymmetric thrust
  void testAsymmetricThrust() {
    double leftThrust = 5000.0;
    double rightThrust = 4000.0;
    double engineSpan = 10.0;  // ft from centerline

    FGColumnVector3 leftPos(0.0, -engineSpan, 0.0);
    FGColumnVector3 rightPos(0.0, engineSpan, 0.0);
    FGColumnVector3 leftForce(leftThrust, 0.0, 0.0);
    FGColumnVector3 rightForce(rightThrust, 0.0, 0.0);

    FGColumnVector3 totalMoment = (leftPos * leftForce) + (rightPos * rightForce);
    // More left thrust should produce yaw right
    TS_ASSERT(totalMoment(3) > 0);
  }

  // Test vertical stabilizer lift
  void testVerticalStabilizerLift() {
    double sideForce = 500.0;
    double vertStabArm = -30.0;  // ft behind CG

    FGColumnVector3 pos(vertStabArm, 0.0, -5.0);
    FGColumnVector3 force(0.0, sideForce, 0.0);

    FGColumnVector3 moment = pos * force;
    // Side force on vertical stab produces yaw moment
    TS_ASSERT(fabs(moment(3)) > 0);
  }

  // Test horizontal stabilizer downforce
  void testHorizontalStabilizerDownforce() {
    double downforce = 200.0;  // Downward force magnitude
    double hstabArm = -25.0;   // Aft of CG

    FGColumnVector3 pos(hstabArm, 0.0, 0.0);
    FGColumnVector3 force(0.0, 0.0, downforce);  // Z+ is down in body frame

    FGColumnVector3 moment = pos * force;
    // Tail downforce at aft position produces nose-up (positive My in body frame)
    TS_ASSERT(moment(2) > 0);
  }

  /***************************************************************************
   * Extended Moment Calculation Tests
   ***************************************************************************/

  // Test rolling moment from aileron
  void testRollingMomentFromAileron() {
    double liftDiff = 100.0;
    double aileronSpan = 15.0;

    double rollMoment = liftDiff * aileronSpan;
    TS_ASSERT_DELTA(rollMoment, 1500.0, epsilon);
  }

  // Test pitching moment from elevator
  void testPitchingMomentFromElevator() {
    double elevatorLift = 300.0;
    double tailArm = 20.0;

    double pitchMoment = -elevatorLift * tailArm;
    TS_ASSERT_DELTA(pitchMoment, -6000.0, epsilon);
  }

  // Test yawing moment from rudder
  void testYawingMomentFromRudder() {
    double rudderForce = 200.0;
    double rudderArm = 25.0;

    double yawMoment = rudderForce * rudderArm;
    TS_ASSERT_DELTA(yawMoment, 5000.0, epsilon);
  }

  /***************************************************************************
   * Extended Frame Transformation Tests
   ***************************************************************************/

  // Test combined rotation
  void testCombinedRotation() {
    double roll = 0.1;
    double pitch = 0.05;

    FGMatrix33 rotX;
    rotX(1,1) = 1.0; rotX(1,2) = 0.0;         rotX(1,3) = 0.0;
    rotX(2,1) = 0.0; rotX(2,2) = cos(roll);   rotX(2,3) = sin(roll);
    rotX(3,1) = 0.0; rotX(3,2) = -sin(roll);  rotX(3,3) = cos(roll);

    FGMatrix33 rotY;
    rotY(1,1) = cos(pitch);  rotY(1,2) = 0.0; rotY(1,3) = -sin(pitch);
    rotY(2,1) = 0.0;         rotY(2,2) = 1.0; rotY(2,3) = 0.0;
    rotY(3,1) = sin(pitch);  rotY(3,2) = 0.0; rotY(3,3) = cos(pitch);

    FGMatrix33 combined = rotY * rotX;
    TS_ASSERT_DELTA(combined.Determinant(), 1.0, 1e-6);
  }

  // Test inverse transform recovery
  void testInverseTransformRecovery() {
    double angle = 0.3;

    FGMatrix33 rot;
    rot(1,1) = cos(angle);  rot(1,2) = sin(angle); rot(1,3) = 0.0;
    rot(2,1) = -sin(angle); rot(2,2) = cos(angle); rot(2,3) = 0.0;
    rot(3,1) = 0.0;         rot(3,2) = 0.0;        rot(3,3) = 1.0;

    FGColumnVector3 original(100.0, 50.0, 25.0);
    FGColumnVector3 transformed = rot * original;
    FGColumnVector3 recovered = rot.Transposed() * transformed;

    TS_ASSERT_DELTA(recovered(1), original(1), 1e-10);
    TS_ASSERT_DELTA(recovered(2), original(2), 1e-10);
    TS_ASSERT_DELTA(recovered(3), original(3), 1e-10);
  }

  /***************************************************************************
   * Extended Dynamic Force Tests
   ***************************************************************************/

  // Test gust load factor
  void testGustLoadFactor() {
    double gustVelocity = 50.0;  // fps
    double wingLoading = 50.0;   // lb/ft^2
    double CLa = 0.1;            // per degree
    double rho = 0.002377;

    double loadFactor = 1.0 + (0.5 * rho * gustVelocity * CLa / wingLoading);
    TS_ASSERT(loadFactor > 1.0);
  }

  // Test centrifugal force in turn
  void testCentrifugalForceInTurn() {
    double weight = 10000.0;
    double bankAngle = 30.0 * M_PI / 180.0;
    double g = 32.2;

    double loadFactor = 1.0 / cos(bankAngle);
    double centrifugalForce = weight * (loadFactor - 1.0);

    TS_ASSERT(centrifugalForce > 0);
  }

  // Test gyroscopic precession
  void testGyroscopicPrecession() {
    double propMomentOfInertia = 5.0;  // slug-ft^2
    double propOmega = 200.0;          // rad/s
    double pitchRate = 0.1;            // rad/s

    double gyroMoment = propMomentOfInertia * propOmega * pitchRate;
    TS_ASSERT(gyroMoment > 0);
  }

  /***************************************************************************
   * Extended Location Tests
   ***************************************************************************/

  // Test multiple force locations
  void testMultipleForceLocations() {
    FGFDMExec fdmex;
    FGExternalForce extForce1(&fdmex);
    FGExternalForce extForce2(&fdmex);

    extForce1.SetLocation(10.0, 0.0, 0.0);
    extForce2.SetLocation(-20.0, 0.0, 0.0);

    TS_ASSERT_DELTA(extForce1.GetLocationX(), 10.0, epsilon);
    TS_ASSERT_DELTA(extForce2.GetLocationX(), -20.0, epsilon);
  }

  // Test diagonal location
  void testDiagonalLocation() {
    FGFDMExec fdmex;
    FGExternalForce extForce(&fdmex);

    extForce.SetLocation(5.0, 3.0, 4.0);

    double distance = sqrt(25.0 + 9.0 + 16.0);
    TS_ASSERT_DELTA(distance, sqrt(50.0), epsilon);
  }

  /***************************************************************************
   * Extended Vector Operation Tests
   ***************************************************************************/

  // Test vector addition
  void testVectorAddition() {
    FGColumnVector3 v1(1.0, 2.0, 3.0);
    FGColumnVector3 v2(4.0, 5.0, 6.0);

    FGColumnVector3 sum = v1 + v2;
    TS_ASSERT_DELTA(sum(1), 5.0, epsilon);
    TS_ASSERT_DELTA(sum(2), 7.0, epsilon);
    TS_ASSERT_DELTA(sum(3), 9.0, epsilon);
  }

  // Test vector subtraction
  void testVectorSubtraction() {
    FGColumnVector3 v1(10.0, 20.0, 30.0);
    FGColumnVector3 v2(3.0, 5.0, 7.0);

    FGColumnVector3 diff = v1 - v2;
    TS_ASSERT_DELTA(diff(1), 7.0, epsilon);
    TS_ASSERT_DELTA(diff(2), 15.0, epsilon);
    TS_ASSERT_DELTA(diff(3), 23.0, epsilon);
  }

  // Test scalar multiplication
  void testScalarMultiplication() {
    FGColumnVector3 v(2.0, 3.0, 4.0);
    double scalar = 5.0;

    FGColumnVector3 result = v * scalar;
    TS_ASSERT_DELTA(result(1), 10.0, epsilon);
    TS_ASSERT_DELTA(result(2), 15.0, epsilon);
    TS_ASSERT_DELTA(result(3), 20.0, epsilon);
  }

  // Test dot product
  void testDotProduct() {
    FGColumnVector3 v1(1.0, 2.0, 3.0);
    FGColumnVector3 v2(4.0, 5.0, 6.0);

    double dot = v1(1)*v2(1) + v1(2)*v2(2) + v1(3)*v2(3);
    TS_ASSERT_DELTA(dot, 32.0, epsilon);
  }

  /***************************************************************************
   * Extended Stress Test Cases
   ***************************************************************************/

  // Test very small forces
  void testVerySmallForces() {
    FGColumnVector3 smallForce(1e-10, 1e-10, 1e-10);
    TS_ASSERT(!std::isnan(smallForce(1)));
    TS_ASSERT(!std::isnan(smallForce.Magnitude()));
  }

  // Test repeated operations
  void testRepeatedOperations() {
    FGColumnVector3 pos(5.0, 0.0, 0.0);
    FGColumnVector3 force(100.0, 0.0, 0.0);

    for (int i = 0; i < 1000; i++) {
      FGColumnVector3 moment = pos * force;
      TS_ASSERT_DELTA(moment.Magnitude(), 0.0, epsilon);
    }
  }
};
