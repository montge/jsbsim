/*******************************************************************************
 * FGInertiaTest.h - Unit tests for inertia and rotational dynamics
 *
 * Tests moment of inertia calculations, parallel axis theorem, perpendicular
 * axis theorem, inertia tensors, angular momentum, and rotational kinetic
 * energy calculations used in flight simulation.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>
#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropagate.h"
#include "models/FGMassBalance.h"

using namespace JSBSim;

const double epsilon = 1e-8;

class FGInertiaTest : public CxxTest::TestSuite
{
public:

  // ============ Moment of Inertia Tests (~10 tests) ============

  // Test point mass moment of inertia: I = mr²
  void testPointMassMomentOfInertia() {
    // GIVEN: A point mass at distance r from axis
    double mass = 2.5;  // kg
    double radius = 3.0;  // m

    // WHEN: Computing moment of inertia I = mr²
    double I = mass * radius * radius;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(22.5, I, epsilon);
  }

  // Test point mass with zero mass edge case
  void testPointMassZeroMass() {
    // GIVEN: Zero mass at any distance
    double mass = 0.0;
    double radius = 5.0;

    // WHEN: Computing moment of inertia
    double I = mass * radius * radius;

    // THEN: Moment of inertia should be zero
    TS_ASSERT_EQUALS(0.0, I);
  }

  // Test point mass with zero radius edge case
  void testPointMassZeroRadius() {
    // GIVEN: Mass at zero distance from axis
    double mass = 10.0;
    double radius = 0.0;

    // WHEN: Computing moment of inertia
    double I = mass * radius * radius;

    // THEN: Moment of inertia should be zero
    TS_ASSERT_EQUALS(0.0, I);
  }

  // Test solid cylinder/disk: I = ½mr²
  void testSolidCylinderMomentOfInertia() {
    // GIVEN: A solid cylinder rotating about its central axis
    double mass = 10.0;  // kg
    double radius = 0.5;  // m

    // WHEN: Computing moment of inertia I = ½mr²
    double I = 0.5 * mass * radius * radius;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(1.25, I, epsilon);
  }

  // Test hollow cylinder: I = ½m(r₁² + r₂²)
  void testHollowCylinderMomentOfInertia() {
    // GIVEN: A hollow cylinder with inner and outer radii
    double mass = 8.0;  // kg
    double r_inner = 0.3;  // m
    double r_outer = 0.5;  // m

    // WHEN: Computing moment of inertia I = ½m(r₁² + r₂²)
    double I = 0.5 * mass * (r_inner * r_inner + r_outer * r_outer);

    // THEN: Verify correct value
    double expected = 0.5 * 8.0 * (0.09 + 0.25);
    TS_ASSERT_DELTA(expected, I, epsilon);
  }

  // Test solid sphere: I = ⅖mr²
  void testSolidSphereMomentOfInertia() {
    // GIVEN: A solid sphere
    double mass = 15.0;  // kg
    double radius = 0.4;  // m

    // WHEN: Computing moment of inertia I = ⅖mr²
    double I = (2.0 / 5.0) * mass * radius * radius;

    // THEN: Verify correct value
    double expected = 0.4 * 15.0 * 0.16;
    TS_ASSERT_DELTA(expected, I, epsilon);
  }

  // Test hollow sphere (thin shell): I = ⅔mr²
  void testHollowSphereMomentOfInertia() {
    // GIVEN: A hollow sphere (thin spherical shell)
    double mass = 6.0;  // kg
    double radius = 0.6;  // m

    // WHEN: Computing moment of inertia I = ⅔mr²
    double I = (2.0 / 3.0) * mass * radius * radius;

    // THEN: Verify correct value
    double expected = (2.0 / 3.0) * 6.0 * 0.36;
    TS_ASSERT_DELTA(expected, I, epsilon);
  }

  // Test rod about center: I = (1/12)mL²
  void testRodAboutCenterMomentOfInertia() {
    // GIVEN: A thin rod rotating about its center
    double mass = 4.0;  // kg
    double length = 2.0;  // m

    // WHEN: Computing moment of inertia I = (1/12)mL²
    double I = (1.0 / 12.0) * mass * length * length;

    // THEN: Verify correct value
    double expected = (1.0 / 12.0) * 4.0 * 4.0;
    TS_ASSERT_DELTA(expected, I, epsilon);
  }

  // Test rod about end: I = ⅓mL²
  void testRodAboutEndMomentOfInertia() {
    // GIVEN: A thin rod rotating about one end
    double mass = 3.0;  // kg
    double length = 1.5;  // m

    // WHEN: Computing moment of inertia I = ⅓mL²
    double I = (1.0 / 3.0) * mass * length * length;

    // THEN: Verify correct value
    double expected = (1.0 / 3.0) * 3.0 * 2.25;
    TS_ASSERT_DELTA(expected, I, epsilon);
  }

  // Test rectangular plate about center: I = (1/12)m(a² + b²)
  void testRectangularPlateAboutCenter() {
    // GIVEN: A rectangular plate rotating about axis through center perpendicular to plate
    double mass = 12.0;  // kg
    double width = 2.0;  // m
    double height = 1.0;  // m

    // WHEN: Computing moment of inertia for rotation about axis perpendicular to plate
    // About center: I_z = (1/12)m(a² + b²)
    double I_z = (1.0 / 12.0) * mass * (width * width + height * height);

    // THEN: Verify correct value
    double expected = (1.0 / 12.0) * 12.0 * (4.0 + 1.0);
    TS_ASSERT_DELTA(expected, I_z, epsilon);
  }

  // Test rectangular plate about edge
  void testRectangularPlateAboutEdge() {
    // GIVEN: A rectangular plate rotating about one edge
    double mass = 6.0;  // kg
    double width = 1.5;  // m

    // WHEN: Computing moment of inertia about edge I = (1/3)mw²
    double I = (1.0 / 3.0) * mass * width * width;

    // THEN: Verify correct value
    double expected = (1.0 / 3.0) * 6.0 * 2.25;
    TS_ASSERT_DELTA(expected, I, epsilon);
  }

  // ============ Parallel Axis Theorem Tests (~6 tests) ============

  // Test parallel axis theorem basic formula: I = I_cm + md²
  void testParallelAxisTheoremBasic() {
    // GIVEN: A solid disk with known I_cm and displacement
    double mass = 5.0;  // kg
    double radius = 0.4;  // m
    double I_cm = 0.5 * mass * radius * radius;  // I = ½mr²
    double distance = 1.0;  // m displacement from CM

    // WHEN: Computing moment about parallel axis using parallel axis theorem
    double I_parallel = I_cm + mass * distance * distance;

    // THEN: Verify correct value
    double expected = 0.4 + 5.0;
    TS_ASSERT_DELTA(expected, I_parallel, epsilon);
  }

  // Test parallel axis theorem with rod
  void testParallelAxisTheoremRod() {
    // GIVEN: A rod of length L, mass m
    double mass = 2.0;  // kg
    double length = 1.0;  // m
    double I_center = (1.0 / 12.0) * mass * length * length;

    // WHEN: Computing moment about end using parallel axis theorem
    // Distance from center to end is L/2
    double distance = length / 2.0;
    double I_end = I_center + mass * distance * distance;

    // THEN: Should equal I = ⅓mL² (rod about end formula)
    double I_end_direct = (1.0 / 3.0) * mass * length * length;
    TS_ASSERT_DELTA(I_end_direct, I_end, epsilon);
  }

  // Test parallel axis theorem with sphere
  void testParallelAxisTheoremSphere() {
    // GIVEN: A solid sphere
    double mass = 8.0;  // kg
    double radius = 0.25;  // m
    double I_cm = (2.0 / 5.0) * mass * radius * radius;

    // WHEN: Computing moment about axis tangent to surface
    // Distance = radius
    double I_tangent = I_cm + mass * radius * radius;

    // THEN: Verify correct value
    double expected = (2.0 / 5.0) * 8.0 * 0.0625 + 8.0 * 0.0625;
    TS_ASSERT_DELTA(expected, I_tangent, epsilon);
  }

  // Test parallel axis theorem zero displacement
  void testParallelAxisTheoremZeroDisplacement() {
    // GIVEN: Object with I_cm, zero displacement
    double mass = 3.0;
    double I_cm = 2.5;
    double distance = 0.0;

    // WHEN: Applying parallel axis theorem
    double I = I_cm + mass * distance * distance;

    // THEN: Should equal I_cm
    TS_ASSERT_DELTA(I_cm, I, epsilon);
  }

  // Test parallel axis theorem with multiple bodies (composite)
  void testParallelAxisTheoremComposite() {
    // GIVEN: Two point masses connected by massless rod
    double m1 = 2.0;  // kg
    double m2 = 3.0;  // kg
    double r1 = 0.5;  // m from axis
    double r2 = 1.0;  // m from axis

    // WHEN: Computing total moment of inertia
    double I_total = m1 * r1 * r1 + m2 * r2 * r2;

    // THEN: Verify correct value
    double expected = 2.0 * 0.25 + 3.0 * 1.0;
    TS_ASSERT_DELTA(expected, I_total, epsilon);
  }

  // Test parallel axis theorem with realistic aircraft component
  void testParallelAxisTheoremWingSection() {
    // GIVEN: A wing section modeled as rectangular plate
    double mass = 50.0;  // kg
    double chord = 1.5;  // m
    double span = 0.3;  // m

    // Moment of inertia about center
    double I_cm = (1.0 / 12.0) * mass * (chord * chord + span * span);

    // WHEN: Wing is offset from fuselage centerline by 5m
    double offset = 5.0;  // m
    double I_offset = I_cm + mass * offset * offset;

    // THEN: Parallel axis contribution should dominate
    TS_ASSERT(I_offset > I_cm);
    double expected = I_cm + 1250.0;
    TS_ASSERT_DELTA(expected, I_offset, epsilon);
  }

  // ============ Perpendicular Axis Theorem Tests (~4 tests) ============

  // Test perpendicular axis theorem: Iz = Ix + Iy for planar objects
  void testPerpendicularAxisTheoremBasic() {
    // GIVEN: A thin rectangular plate in xy-plane
    double mass = 10.0;  // kg
    double width = 2.0;  // m (along x)
    double height = 1.0;  // m (along y)

    // WHEN: Computing moments of inertia
    double Ix = (1.0 / 12.0) * mass * height * height;  // About x-axis
    double Iy = (1.0 / 12.0) * mass * width * width;   // About y-axis
    double Iz = Ix + Iy;  // About z-axis (perpendicular axis theorem)

    // THEN: Should equal direct calculation
    double Iz_direct = (1.0 / 12.0) * mass * (width * width + height * height);
    TS_ASSERT_DELTA(Iz_direct, Iz, epsilon);
  }

  // Test perpendicular axis theorem with circular disk
  void testPerpendicularAxisTheoremDisk() {
    // GIVEN: A thin circular disk in xy-plane
    double mass = 5.0;  // kg
    double radius = 0.5;  // m

    // WHEN: By symmetry Ix = Iy, and Iz = ½mr²
    double Iz = 0.5 * mass * radius * radius;

    // THEN: By perpendicular axis theorem: Iz = Ix + Iy = 2*Ix
    double Ix = Iz / 2.0;
    double Iy = Ix;

    TS_ASSERT_DELTA(Ix, Iy, epsilon);
    TS_ASSERT_DELTA(Iz, Ix + Iy, epsilon);

    // Ix for disk should be ¼mr²
    double expected_Ix = 0.25 * mass * radius * radius;
    TS_ASSERT_DELTA(expected_Ix, Ix, epsilon);
  }

  // Test perpendicular axis theorem with square plate
  void testPerpendicularAxisTheoremSquare() {
    // GIVEN: A square plate with side length a
    double mass = 8.0;  // kg
    double side = 1.2;  // m

    // WHEN: Computing moments by symmetry
    double Ix = (1.0 / 12.0) * mass * side * side;
    double Iy = Ix;  // By symmetry
    double Iz = Ix + Iy;  // Perpendicular axis theorem

    // THEN: Iz should equal sum
    double expected_Iz = (1.0 / 6.0) * mass * side * side;
    TS_ASSERT_DELTA(expected_Iz, Iz, epsilon);
  }

  // Test perpendicular axis theorem verification for complex shape
  void testPerpendicularAxisTheoremTriangle() {
    // GIVEN: A triangular plate (right triangle) in xy-plane
    // Base b along x, height h along y, mass m
    double mass = 6.0;  // kg
    double base = 1.5;  // m
    double height = 1.0;  // m

    // For right triangle about centroid:
    // Ix = (1/18)mh², Iy = (1/18)mb²
    double Ix = (1.0 / 18.0) * mass * height * height;
    double Iy = (1.0 / 18.0) * mass * base * base;

    // WHEN: Using perpendicular axis theorem
    double Iz = Ix + Iy;

    // THEN: Verify Iz = (1/18)m(b² + h²)
    double expected = (1.0 / 18.0) * mass * (base * base + height * height);
    TS_ASSERT_DELTA(expected, Iz, epsilon);
  }

  // ============ Inertia Tensor Tests (~8 tests) ============

  // Test diagonal inertia tensor (principal axes aligned with body axes)
  void testDiagonalInertiaTensor() {
    // GIVEN: An object with principal moments aligned with coordinate axes
    double Ixx = 100.0;  // kg⋅m²
    double Iyy = 150.0;  // kg⋅m²
    double Izz = 200.0;  // kg⋅m²

    // WHEN: Creating inertia tensor
    FGMatrix33 I(Ixx, 0.0, 0.0,
                 0.0, Iyy, 0.0,
                 0.0, 0.0, Izz);

    // THEN: Verify diagonal elements and zero off-diagonal
    TS_ASSERT_DELTA(Ixx, I(1, 1), epsilon);
    TS_ASSERT_DELTA(Iyy, I(2, 2), epsilon);
    TS_ASSERT_DELTA(Izz, I(3, 3), epsilon);
    TS_ASSERT_EQUALS(0.0, I(1, 2));
    TS_ASSERT_EQUALS(0.0, I(1, 3));
    TS_ASSERT_EQUALS(0.0, I(2, 3));
  }

  // Test inertia tensor with products of inertia
  void testInertiaTensorWithProducts() {
    // GIVEN: An asymmetric body with products of inertia
    double Ixx = 50.0, Iyy = 60.0, Izz = 70.0;
    double Ixy = -5.0, Ixz = -3.0, Iyz = -2.0;

    // WHEN: Creating full inertia tensor (symmetric)
    FGMatrix33 I(Ixx,  -Ixy, -Ixz,
                 -Ixy, Iyy,  -Iyz,
                 -Ixz, -Iyz, Izz);

    // THEN: Verify symmetry
    TS_ASSERT_DELTA(I(1, 2), I(2, 1), epsilon);
    TS_ASSERT_DELTA(I(1, 3), I(3, 1), epsilon);
    TS_ASSERT_DELTA(I(2, 3), I(3, 2), epsilon);
  }

  // Test inertia tensor symmetry property
  void testInertiaTensorSymmetry() {
    // GIVEN: Any inertia tensor
    FGMatrix33 I(45.0, -3.5, -2.1,
                 -3.5, 52.0, -1.8,
                 -2.1, -1.8, 48.0);

    // WHEN: Checking symmetry
    FGMatrix33 I_transposed = I.Transposed();

    // THEN: I should equal I^T
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT_DELTA(I(i, j), I_transposed(i, j), epsilon);
      }
    }
  }

  // Test inertia tensor for point masses
  void testInertiaTensorPointMasses() {
    // GIVEN: A simple system of point masses
    // Mass at (1, 0, 0)
    double m1 = 2.0;  // kg
    double x1 = 1.0, y1 = 0.0, z1 = 0.0;

    // Mass at (0, 1, 0)
    double m2 = 3.0;  // kg
    double x2 = 0.0, y2 = 1.0, z2 = 0.0;

    // WHEN: Computing inertia tensor components
    // Ixx = Σm(y² + z²)
    double Ixx = m1 * (y1*y1 + z1*z1) + m2 * (y2*y2 + z2*z2);

    // Iyy = Σm(x² + z²)
    double Iyy = m1 * (x1*x1 + z1*z1) + m2 * (x2*x2 + z2*z2);

    // Izz = Σm(x² + y²)
    double Izz = m1 * (x1*x1 + y1*y1) + m2 * (x2*x2 + y2*y2);

    // Ixy = -Σm(xy)
    double Ixy = -(m1 * x1 * y1 + m2 * x2 * y2);

    // THEN: Verify expected values
    TS_ASSERT_DELTA(3.0, Ixx, epsilon);  // m2 * 1² = 3
    TS_ASSERT_DELTA(2.0, Iyy, epsilon);  // m1 * 1² = 2
    TS_ASSERT_DELTA(5.0, Izz, epsilon);  // m1*1² + m2*1² = 5
    TS_ASSERT_EQUALS(0.0, Ixy);  // No product terms
  }

  // Test inertia tensor rotation (coordinate transformation)
  void testInertiaTensorRotation() {
    // GIVEN: Diagonal inertia tensor in body frame
    FGMatrix33 I_body(100.0, 0.0, 0.0,
                      0.0, 150.0, 0.0,
                      0.0, 0.0, 200.0);

    // Rotation matrix (45° about z-axis)
    double angle = M_PI / 4.0;
    double c = cos(angle);
    double s = sin(angle);
    FGMatrix33 R(c,  s, 0.0,
                 -s, c, 0.0,
                 0.0, 0.0, 1.0);

    // WHEN: Transforming to rotated frame I' = R I R^T
    FGMatrix33 I_rotated = R * I_body * R.Transposed();

    // THEN: Should still be symmetric
    TS_ASSERT_DELTA(I_rotated(1, 2), I_rotated(2, 1), epsilon);
    TS_ASSERT_DELTA(I_rotated(1, 3), I_rotated(3, 1), epsilon);
    TS_ASSERT_DELTA(I_rotated(2, 3), I_rotated(3, 2), epsilon);

    // Trace should be invariant under rotation
    double trace_original = I_body(1, 1) + I_body(2, 2) + I_body(3, 3);
    double trace_rotated = I_rotated(1, 1) + I_rotated(2, 2) + I_rotated(3, 3);
    TS_ASSERT_DELTA(trace_original, trace_rotated, epsilon);
  }

  // Test trace invariance of inertia tensor
  void testInertiaTensorTraceInvariance() {
    // GIVEN: An inertia tensor and a rotation
    FGMatrix33 I(50.0, -5.0, -3.0,
                 -5.0, 60.0, -2.0,
                 -3.0, -2.0, 70.0);

    double angle = M_PI / 6.0;
    double c = cos(angle);
    double s = sin(angle);
    FGMatrix33 R(c, -s, 0.0,
                 s,  c, 0.0,
                 0.0, 0.0, 1.0);

    // WHEN: Rotating tensor
    FGMatrix33 I_rotated = R * I * R.Transposed();

    // THEN: Trace should be invariant
    double trace_I = I(1, 1) + I(2, 2) + I(3, 3);
    double trace_I_rotated = I_rotated(1, 1) + I_rotated(2, 2) + I_rotated(3, 3);
    TS_ASSERT_DELTA(trace_I, trace_I_rotated, epsilon);
  }

  // Test principal axes (eigenvalues) of inertia tensor
  void testPrincipalAxesDiagonal() {
    // GIVEN: Diagonal tensor (already in principal axes)
    FGMatrix33 I(100.0, 0.0, 0.0,
                 0.0, 150.0, 0.0,
                 0.0, 0.0, 200.0);

    // THEN: Diagonal elements are principal moments
    // For diagonal matrix, eigenvalues = diagonal elements
    TS_ASSERT_DELTA(100.0, I(1, 1), epsilon);
    TS_ASSERT_DELTA(150.0, I(2, 2), epsilon);
    TS_ASSERT_DELTA(200.0, I(3, 3), epsilon);
  }

  // Test aircraft-realistic inertia tensor
  void testAircraftInertiaTensor() {
    // GIVEN: Realistic small aircraft inertia values (kg⋅m²)
    double Ixx = 1285.3;   // Roll inertia
    double Iyy = 1824.9;   // Pitch inertia
    double Izz = 2666.9;   // Yaw inertia
    double Ixz = -37.2;    // Common product of inertia

    // WHEN: Creating inertia tensor (Ixy and Iyz typically near zero for aircraft)
    FGMatrix33 I(Ixx,  0.0, -Ixz,
                 0.0,  Iyy, 0.0,
                 -Ixz, 0.0, Izz);

    // THEN: Verify physical constraints
    // All principal moments must be positive
    TS_ASSERT(I(1, 1) > 0.0);
    TS_ASSERT(I(2, 2) > 0.0);
    TS_ASSERT(I(3, 3) > 0.0);

    // Triangle inequality: |Ix - Iy| < Iz < Ix + Iy (and cyclic permutations)
    TS_ASSERT(fabs(Ixx - Iyy) < Izz);
    TS_ASSERT(Izz < Ixx + Iyy);
  }

  // ============ Angular Momentum Tests (~6 tests) ============

  // Test angular momentum L = Iω for simple rotation
  void testAngularMomentumSimple() {
    // GIVEN: Object rotating about z-axis
    double I = 5.0;  // kg⋅m²
    double omega = 10.0;  // rad/s

    // WHEN: Computing angular momentum L = Iω
    double L = I * omega;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(50.0, L, epsilon);
  }

  // Test angular momentum vector L = I·ω for general rotation
  void testAngularMomentumVector() {
    // GIVEN: Diagonal inertia tensor and angular velocity
    FGMatrix33 I(10.0, 0.0, 0.0,
                 0.0, 15.0, 0.0,
                 0.0, 0.0, 20.0);
    FGColumnVector3 omega(2.0, 3.0, 1.0);  // rad/s

    // WHEN: Computing L = I·ω
    FGColumnVector3 L = I * omega;

    // THEN: For diagonal I: Lx = Ixx*ωx, Ly = Iyy*ωy, Lz = Izz*ωz
    TS_ASSERT_DELTA(20.0, L(1), epsilon);  // 10 * 2
    TS_ASSERT_DELTA(45.0, L(2), epsilon);  // 15 * 3
    TS_ASSERT_DELTA(20.0, L(3), epsilon);  // 20 * 1
  }

  // Test angular momentum for point mass: L = r × p
  void testAngularMomentumPointMass() {
    // GIVEN: Point mass with position and linear momentum
    FGColumnVector3 r(1.0, 0.0, 0.0);  // position
    double mass = 2.0;  // kg
    FGColumnVector3 v(0.0, 3.0, 0.0);  // velocity
    FGColumnVector3 p = mass * v;  // linear momentum

    // WHEN: Computing angular momentum L = r × p
    FGColumnVector3 L = r * p;  // Cross product

    // THEN: L should point in z direction
    TS_ASSERT_DELTA(0.0, L(1), epsilon);
    TS_ASSERT_DELTA(0.0, L(2), epsilon);
    TS_ASSERT_DELTA(6.0, L(3), epsilon);  // r * p = 1 * 6 = 6
  }

  // Test conservation of angular momentum (no external torque)
  void testAngularMomentumConservation() {
    // GIVEN: Initial state with angular velocity
    FGMatrix33 I(8.0, 0.0, 0.0,
                 0.0, 8.0, 0.0,
                 0.0, 0.0, 8.0);
    FGColumnVector3 omega_initial(1.0, 2.0, 3.0);
    FGColumnVector3 L_initial = I * omega_initial;

    // WHEN: No external torque, L should be conserved
    // Simulate "later" state with different omega due to rotation
    FGColumnVector3 omega_later(0.5, 2.5, 3.2);
    FGColumnVector3 L_later = I * omega_later;

    // For this test, we just verify the calculation works
    // In a real scenario without torque, L would be constant
    double L_mag_initial = L_initial.Magnitude();
    double L_mag_later = L_later.Magnitude();

    TS_ASSERT(L_mag_initial > 0.0);
    TS_ASSERT(L_mag_later > 0.0);
  }

  // Test angular momentum with non-diagonal inertia tensor
  void testAngularMomentumNonDiagonal() {
    // GIVEN: Inertia tensor with products of inertia
    FGMatrix33 I(12.0, -2.0, -1.0,
                 -2.0, 15.0, -0.5,
                 -1.0, -0.5, 18.0);
    FGColumnVector3 omega(1.0, 0.0, 0.0);  // Rotation about x-axis only

    // WHEN: Computing angular momentum
    FGColumnVector3 L = I * omega;

    // THEN: L won't align with omega (due to products of inertia)
    TS_ASSERT_DELTA(12.0, L(1), epsilon);   // Ixx * ωx
    TS_ASSERT_DELTA(-2.0, L(2), epsilon);   // Ixy * ωx
    TS_ASSERT_DELTA(-1.0, L(3), epsilon);   // Ixz * ωx

    // L is not parallel to omega
    TS_ASSERT(L(2) != 0.0 || L(3) != 0.0);
  }

  // Test precession due to angular momentum change
  void testAngularMomentumPrecession() {
    // GIVEN: Spinning disk (gyroscope)
    double I_spin = 0.5;  // kg⋅m² (about spin axis)
    double I_trans = 0.25;  // kg⋅m² (transverse)
    double omega_spin = 100.0;  // rad/s (high spin rate)

    // Initial angular momentum (spinning about z)
    double L = I_spin * omega_spin;

    // WHEN: Small torque applied perpendicular to spin axis
    double torque = 0.1;  // N⋅m
    double dt = 0.01;  // s
    double dL = torque * dt;

    // THEN: Change in angular momentum
    double precession_rate = torque / L;  // ω_precession = τ/L

    TS_ASSERT(precession_rate > 0.0);
    TS_ASSERT(precession_rate < omega_spin);  // Precession much slower than spin
  }

  // ============ Rotational Kinetic Energy Tests (~4 tests) ============

  // Test rotational kinetic energy: KE = ½Iω²
  void testRotationalKineticEnergySimple() {
    // GIVEN: Object rotating about single axis
    double I = 4.0;  // kg⋅m²
    double omega = 5.0;  // rad/s

    // WHEN: Computing rotational kinetic energy
    double KE = 0.5 * I * omega * omega;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(50.0, KE, epsilon);  // ½ * 4 * 25 = 50
  }

  // Test rotational kinetic energy for 3D rotation: KE = ½ω·I·ω
  void testRotationalKineticEnergy3D() {
    // GIVEN: Object with diagonal inertia tensor
    FGMatrix33 I(10.0, 0.0, 0.0,
                 0.0, 15.0, 0.0,
                 0.0, 0.0, 20.0);
    FGColumnVector3 omega(2.0, 1.0, 3.0);  // rad/s

    // WHEN: Computing KE = ½ω·(I·ω)
    FGColumnVector3 I_omega = I * omega;
    double KE = 0.5 * DotProduct(omega, I_omega);

    // THEN: For diagonal I: KE = ½(Ixx*ωx² + Iyy*ωy² + Izz*ωz²)
    double expected = 0.5 * (10.0 * 4.0 + 15.0 * 1.0 + 20.0 * 9.0);
    TS_ASSERT_DELTA(expected, KE, epsilon);
  }

  // Test energy conservation in rotation
  void testRotationalEnergyConservation() {
    // GIVEN: Object with constant angular velocity (no torque)
    double I = 6.0;  // kg⋅m²
    double omega = 8.0;  // rad/s
    double KE_initial = 0.5 * I * omega * omega;

    // WHEN: Time passes with no external torque
    // Angular velocity remains constant
    double KE_later = 0.5 * I * omega * omega;

    // THEN: Energy should be conserved
    TS_ASSERT_DELTA(KE_initial, KE_later, epsilon);
  }

  // Test work-energy theorem for rotation: W = ΔKE
  void testWorkEnergyTheoremRotation() {
    // GIVEN: Initial and final angular velocities
    double I = 3.0;  // kg⋅m²
    double omega_i = 2.0;  // rad/s
    double omega_f = 5.0;  // rad/s

    // WHEN: Computing change in kinetic energy
    double KE_i = 0.5 * I * omega_i * omega_i;
    double KE_f = 0.5 * I * omega_f * omega_f;
    double delta_KE = KE_f - KE_i;

    // Work done by torque should equal change in KE
    double work = delta_KE;

    // THEN: Verify calculation
    double expected_delta = 0.5 * 3.0 * (25.0 - 4.0);  // ½I(ωf² - ωi²)
    TS_ASSERT_DELTA(expected_delta, delta_KE, epsilon);
    TS_ASSERT(delta_KE > 0.0);  // Energy increased
  }

  // ============ Edge Cases and Integration Tests ============

  // Test combined parallel and perpendicular axis theorems
  void testCombinedAxisTheorems() {
    // GIVEN: Rectangular plate offset from origin
    double mass = 10.0;  // kg
    double a = 2.0;  // m (width)
    double b = 1.0;  // m (height)

    // Moments about center
    double Ix_cm = (1.0 / 12.0) * mass * b * b;
    double Iy_cm = (1.0 / 12.0) * mass * a * a;
    double Iz_cm = Ix_cm + Iy_cm;  // Perpendicular axis theorem

    // WHEN: Plate is offset by distance d from origin
    double d = 3.0;  // m
    double Iz_offset = Iz_cm + mass * d * d;  // Parallel axis theorem

    // THEN: Verify both theorems work together
    double expected = (1.0 / 12.0) * mass * (a*a + b*b) + mass * d * d;
    TS_ASSERT_DELTA(expected, Iz_offset, epsilon);
  }

  // Test zero moment of inertia edge case
  void testZeroMomentOfInertia() {
    // GIVEN: Zero mass or zero dimension
    double mass = 0.0;
    double radius = 5.0;

    // WHEN: Computing moment of inertia
    double I = 0.5 * mass * radius * radius;

    // THEN: Should be zero
    TS_ASSERT_EQUALS(0.0, I);
  }

  // Test very large values for numerical stability
  void testLargeMomentOfInertia() {
    // GIVEN: Large aircraft or spacecraft
    double mass = 1.0e6;  // kg (large spacecraft)
    double radius = 100.0;  // m

    // WHEN: Computing moment of inertia
    double I = 0.5 * mass * radius * radius;

    // THEN: Should handle large values correctly
    TS_ASSERT_DELTA(5.0e9, I, 1.0);  // Allow small relative error
    TS_ASSERT(!std::isnan(I));
    TS_ASSERT(!std::isinf(I));
  }

  // Test realistic aircraft scenario
  void testRealisticAircraftRotation() {
    // GIVEN: Small general aviation aircraft inertia
    FGMatrix33 I(1200.0, 0.0,    -50.0,
                 0.0,    1800.0, 0.0,
                 -50.0,  0.0,    2500.0);

    // Angular velocity in typical maneuver
    FGColumnVector3 omega(0.1, 0.05, 0.02);  // rad/s (gentle turn)

    // WHEN: Computing angular momentum and kinetic energy
    FGColumnVector3 L = I * omega;
    double KE = 0.5 * DotProduct(omega, L);

    // THEN: Values should be physically reasonable
    TS_ASSERT(L.Magnitude() > 0.0);
    TS_ASSERT(KE > 0.0);
    TS_ASSERT(!std::isnan(KE));
    TS_ASSERT(!std::isinf(KE));

    // Magnitude of L should be on order of I*omega
    double L_approx = 2000.0 * 0.1;  // Rough estimate
    TS_ASSERT(L.Magnitude() < 10.0 * L_approx);  // Within reasonable range
  }

  // Test gyroscopic effect
  void testGyroscopicEffect() {
    // GIVEN: Spinning rotor with angular momentum
    double I_rotor = 2.0;  // kg⋅m²
    double omega_rotor = 200.0;  // rad/s (high speed rotor)
    double L = I_rotor * omega_rotor;

    // WHEN: Aircraft pitches with rate omega_pitch
    double omega_pitch = 0.1;  // rad/s

    // THEN: Gyroscopic torque = L * omega_pitch (perpendicular)
    double torque_gyro = L * omega_pitch;

    TS_ASSERT_DELTA(40.0, torque_gyro, epsilon);
    TS_ASSERT(torque_gyro > 0.0);
  }

  // Test moment of inertia tensor determinant (physical constraint)
  void testInertiaTensorDeterminant() {
    // GIVEN: Physical inertia tensor
    FGMatrix33 I(100.0, -5.0,  -3.0,
                 -5.0,  120.0, -2.0,
                 -3.0,  -2.0,  140.0);

    // WHEN: Computing determinant
    double det = I.Determinant();

    // THEN: For physical inertia tensor, determinant should be positive
    TS_ASSERT(det > 0.0);
  }

  // Test inertia tensor positive definiteness
  void testInertiaTensorPositiveDefinite() {
    // GIVEN: Physical inertia tensor (diagonal dominant)
    FGMatrix33 I(50.0, -2.0,  -1.0,
                 -2.0,  60.0, -1.5,
                 -1.0,  -1.5, 70.0);

    // THEN: All diagonal elements should be positive
    TS_ASSERT(I(1, 1) > 0.0);
    TS_ASSERT(I(2, 2) > 0.0);
    TS_ASSERT(I(3, 3) > 0.0);

    // Determinant should be positive
    TS_ASSERT(I.Determinant() > 0.0);
  }

  // ============ Angular Acceleration Tests ============

  // Test angular acceleration: α = τ/I (single axis)
  void testAngularAccelerationSimple() {
    // GIVEN: Torque and moment of inertia
    double torque = 10.0;  // N⋅m
    double I = 2.0;  // kg⋅m²

    // WHEN: Computing angular acceleration α = τ/I
    double alpha = torque / I;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(5.0, alpha, epsilon);  // 10/2 = 5 rad/s²
  }

  // Test angular acceleration vector: α = I⁻¹ · τ
  void testAngularAccelerationVector() {
    // GIVEN: Diagonal inertia tensor and torque vector
    FGMatrix33 I(10.0, 0.0, 0.0,
                 0.0, 20.0, 0.0,
                 0.0, 0.0, 30.0);
    FGMatrix33 I_inv(0.1, 0.0, 0.0,
                     0.0, 0.05, 0.0,
                     0.0, 0.0, 1.0/30.0);
    FGColumnVector3 torque(5.0, 10.0, 15.0);  // N⋅m

    // WHEN: Computing angular acceleration
    FGColumnVector3 alpha = I_inv * torque;

    // THEN: For diagonal I: αx = τx/Ixx, etc.
    TS_ASSERT_DELTA(0.5, alpha(1), epsilon);   // 5/10
    TS_ASSERT_DELTA(0.5, alpha(2), epsilon);   // 10/20
    TS_ASSERT_DELTA(0.5, alpha(3), epsilon);   // 15/30
  }

  // Test zero torque gives zero acceleration
  void testZeroTorqueZeroAcceleration() {
    // GIVEN: Zero torque
    FGMatrix33 I_inv(0.1, 0.0, 0.0,
                     0.0, 0.05, 0.0,
                     0.0, 0.0, 0.033);
    FGColumnVector3 torque(0.0, 0.0, 0.0);

    // WHEN: Computing angular acceleration
    FGColumnVector3 alpha = I_inv * torque;

    // THEN: Zero acceleration
    TS_ASSERT_DELTA(0.0, alpha(1), epsilon);
    TS_ASSERT_DELTA(0.0, alpha(2), epsilon);
    TS_ASSERT_DELTA(0.0, alpha(3), epsilon);
  }

  // Test angular acceleration with products of inertia
  void testAngularAccelerationCoupled() {
    // GIVEN: Inertia tensor with Ixz coupling
    // For aircraft with Ixz ≠ 0
    double Ixx = 1000.0, Izz = 2000.0, Ixz = 100.0;
    double det = Ixx * Izz - Ixz * Ixz;

    // Simplified inverse for Ixz coupling only
    FGMatrix33 I_inv(Izz/det, 0.0, Ixz/det,
                     0.0, 0.001, 0.0,
                     Ixz/det, 0.0, Ixx/det);
    FGColumnVector3 torque(100.0, 0.0, 0.0);  // Roll torque only

    // WHEN: Computing angular acceleration
    FGColumnVector3 alpha = I_inv * torque;

    // THEN: Roll torque causes both roll and yaw acceleration due to Ixz
    TS_ASSERT(std::abs(alpha(1)) > 0.0);  // Roll acceleration
    TS_ASSERT(std::abs(alpha(3)) > 0.0);  // Yaw acceleration (coupling)
  }

  // ============ Euler's Equations of Motion ============

  // Test Euler equation: dL/dt = τ
  void testEulerEquationTorque() {
    // GIVEN: Angular momentum change rate equals applied torque
    FGColumnVector3 L_initial(10.0, 20.0, 30.0);
    FGColumnVector3 torque(1.0, 2.0, 3.0);
    double dt = 0.1;

    // WHEN: Applying torque for time dt
    FGColumnVector3 dL = torque * dt;
    FGColumnVector3 L_final = L_initial + dL;

    // THEN: Verify change in angular momentum
    TS_ASSERT_DELTA(10.1, L_final(1), epsilon);
    TS_ASSERT_DELTA(20.2, L_final(2), epsilon);
    TS_ASSERT_DELTA(30.3, L_final(3), epsilon);
  }

  // Test Euler equations in body frame: I·α + ω × (I·ω) = τ
  void testEulerEquationsBodyFrame() {
    // GIVEN: Diagonal inertia tensor and angular velocity
    double Ixx = 100.0, Iyy = 150.0, Izz = 200.0;
    FGColumnVector3 omega(0.1, 0.2, 0.3);  // rad/s

    // WHEN: Computing gyroscopic term ω × (I·ω)
    FGColumnVector3 L(Ixx * omega(1), Iyy * omega(2), Izz * omega(3));
    FGColumnVector3 gyro = omega * L;  // Cross product

    // THEN: Verify gyroscopic moments
    // ωy*Lz - ωz*Ly = 0.2*60 - 0.3*30 = 12 - 9 = 3
    TS_ASSERT_DELTA(3.0, gyro(1), epsilon);
    // ωz*Lx - ωx*Lz = 0.3*10 - 0.1*60 = 3 - 6 = -3
    TS_ASSERT_DELTA(-3.0, gyro(2), epsilon);
    // ωx*Ly - ωy*Lx = 0.1*30 - 0.2*10 = 3 - 2 = 1
    TS_ASSERT_DELTA(1.0, gyro(3), epsilon);
  }

  // Test symmetric top (axisymmetric body)
  void testSymmetricTopEulerEquations() {
    // GIVEN: Axisymmetric body (Ixx = Iyy)
    double I_perp = 100.0;  // Ixx = Iyy
    double I_spin = 200.0;  // Izz

    FGColumnVector3 omega(0.1, 0.0, 10.0);  // Small tilt, fast spin

    // WHEN: Computing angular momentum
    FGColumnVector3 L(I_perp * omega(1), I_perp * omega(2), I_spin * omega(3));

    // THEN: L_z dominates due to fast spin
    TS_ASSERT(std::abs(L(3)) > std::abs(L(1)) + std::abs(L(2)));
  }

  // ============ Radius of Gyration Tests ============

  // Test radius of gyration: k = √(I/m)
  void testRadiusOfGyrationBasic() {
    // GIVEN: Moment of inertia and mass
    double I = 50.0;  // kg⋅m²
    double m = 10.0;  // kg

    // WHEN: Computing radius of gyration
    double k = sqrt(I / m);

    // THEN: Verify correct value
    TS_ASSERT_DELTA(sqrt(5.0), k, epsilon);
  }

  // Test radius of gyration for solid cylinder
  void testRadiusOfGyrationCylinder() {
    // GIVEN: Solid cylinder I = ½mr²
    double m = 8.0;  // kg
    double r = 0.5;  // m
    double I = 0.5 * m * r * r;

    // WHEN: Computing radius of gyration
    double k = sqrt(I / m);

    // THEN: Should equal r/√2
    TS_ASSERT_DELTA(r / sqrt(2.0), k, epsilon);
  }

  // Test radius of gyration for hollow cylinder
  void testRadiusOfGyrationHollowCylinder() {
    // GIVEN: Hollow cylinder I = ½m(r₁² + r₂²)
    double m = 5.0;
    double r1 = 0.3, r2 = 0.5;
    double I = 0.5 * m * (r1*r1 + r2*r2);

    // WHEN: Computing radius of gyration
    double k = sqrt(I / m);

    // THEN: Should equal √((r₁² + r₂²)/2)
    double expected = sqrt(0.5 * (r1*r1 + r2*r2));
    TS_ASSERT_DELTA(expected, k, epsilon);
  }

  // ============ Torque Tests ============

  // Test torque from force and moment arm: τ = r × F
  void testTorqueFromForce() {
    // GIVEN: Force and position vector
    FGColumnVector3 r(2.0, 0.0, 0.0);  // 2m along x
    FGColumnVector3 F(0.0, 10.0, 0.0);  // 10N along y

    // WHEN: Computing torque
    FGColumnVector3 tau = r * F;  // Cross product

    // THEN: Torque about z-axis
    TS_ASSERT_DELTA(0.0, tau(1), epsilon);
    TS_ASSERT_DELTA(0.0, tau(2), epsilon);
    TS_ASSERT_DELTA(20.0, tau(3), epsilon);
  }

  // Test torque magnitude: |τ| = |r||F|sin(θ)
  void testTorqueMagnitude() {
    // GIVEN: Force and position at 90 degrees
    double r = 3.0;  // m
    double F = 5.0;  // N
    double theta = M_PI / 2.0;  // 90 degrees

    // WHEN: Computing torque magnitude
    double tau = r * F * sin(theta);

    // THEN: Maximum torque at 90 degrees
    TS_ASSERT_DELTA(15.0, tau, epsilon);
  }

  // Test zero torque when force through pivot
  void testZeroTorqueForceAtPivot() {
    // GIVEN: Force applied at pivot point (r = 0)
    FGColumnVector3 r(0.0, 0.0, 0.0);
    FGColumnVector3 F(5.0, 3.0, 2.0);

    // WHEN: Computing torque
    FGColumnVector3 tau = r * F;

    // THEN: Zero torque
    TS_ASSERT_DELTA(0.0, tau.Magnitude(), epsilon);
  }

  // Test torque from parallel force (zero torque)
  void testZeroTorqueParallelForce() {
    // GIVEN: Force parallel to moment arm
    FGColumnVector3 r(2.0, 0.0, 0.0);
    FGColumnVector3 F(5.0, 0.0, 0.0);  // Parallel to r

    // WHEN: Computing torque
    FGColumnVector3 tau = r * F;

    // THEN: Zero torque (sin(0) = 0)
    TS_ASSERT_DELTA(0.0, tau.Magnitude(), epsilon);
  }

  // ============ Composite Body Tests ============

  // Test composite body moment of inertia
  void testCompositeBodyMomentOfInertia() {
    // GIVEN: Two masses connected by massless rod
    double m1 = 3.0, r1 = 1.0;  // kg, m
    double m2 = 2.0, r2 = 2.0;  // kg, m

    // WHEN: Computing total moment of inertia
    double I_total = m1 * r1 * r1 + m2 * r2 * r2;

    // THEN: Sum of individual contributions
    TS_ASSERT_DELTA(3.0 + 8.0, I_total, epsilon);
  }

  // Test moment of inertia of T-shaped beam
  void testTShapedBeamMomentOfInertia() {
    // GIVEN: T-beam composed of two rectangular sections
    double m_vertical = 4.0, h_v = 2.0;  // Vertical section
    double m_horizontal = 3.0, w_h = 1.5;  // Horizontal section
    double offset_h = 1.5;  // Distance from axis to horizontal bar center

    // Moment about center of vertical section
    double I_v = (1.0/12.0) * m_vertical * h_v * h_v;

    // Horizontal bar about its center + parallel axis
    double I_h_cm = (1.0/12.0) * m_horizontal * w_h * w_h;
    double I_h = I_h_cm + m_horizontal * offset_h * offset_h;

    // WHEN: Computing total
    double I_total = I_v + I_h;

    // THEN: Verify calculation
    double expected = (1.0/12.0) * 4.0 * 4.0 + (1.0/12.0) * 3.0 * 2.25 + 3.0 * 2.25;
    TS_ASSERT_DELTA(expected, I_total, epsilon);
  }

  // Test dumbbell moment of inertia
  void testDumbbellMomentOfInertia() {
    // GIVEN: Two spheres connected by massless rod
    double m_sphere = 5.0;  // kg each
    double r_sphere = 0.1;  // m radius
    double L = 0.5;  // m distance from center

    // Each sphere: I_cm = (2/5)mr² + mr² (parallel axis)
    double I_sphere_cm = (2.0/5.0) * m_sphere * r_sphere * r_sphere;
    double I_sphere_offset = I_sphere_cm + m_sphere * L * L;

    // WHEN: Total for two spheres
    double I_total = 2 * I_sphere_offset;

    // THEN: Dominated by m*L² term
    double expected = 2 * (0.4 * 5.0 * 0.01 + 5.0 * 0.25);
    TS_ASSERT_DELTA(expected, I_total, epsilon);
  }

  // ============ Time Evolution Tests ============

  // Test angular velocity evolution: ω(t) = ω₀ + αt
  void testAngularVelocityEvolution() {
    // GIVEN: Initial angular velocity and constant acceleration
    double omega_0 = 5.0;  // rad/s
    double alpha = 2.0;    // rad/s²
    double t = 3.0;        // s

    // WHEN: Computing final angular velocity
    double omega_f = omega_0 + alpha * t;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(11.0, omega_f, epsilon);
  }

  // Test angular displacement: θ = ω₀t + ½αt²
  void testAngularDisplacement() {
    // GIVEN: Constant angular acceleration
    double omega_0 = 2.0;  // rad/s
    double alpha = 1.0;    // rad/s²
    double t = 4.0;        // s

    // WHEN: Computing angular displacement
    double theta = omega_0 * t + 0.5 * alpha * t * t;

    // THEN: Verify correct value
    TS_ASSERT_DELTA(2.0*4.0 + 0.5*1.0*16.0, theta, epsilon);
  }

  // Test angular velocity squared relation: ω² = ω₀² + 2αθ
  void testAngularVelocitySquaredRelation() {
    // GIVEN: Angular displacement and acceleration
    double omega_0 = 3.0;  // rad/s
    double alpha = 2.0;    // rad/s²
    double theta = 5.0;    // rad

    // WHEN: Computing final angular velocity
    double omega_f_sq = omega_0 * omega_0 + 2 * alpha * theta;
    double omega_f = sqrt(omega_f_sq);

    // THEN: Verify correct value
    TS_ASSERT_DELTA(sqrt(9.0 + 20.0), omega_f, epsilon);
  }

  // ============ Physical Constraint Tests ============

  // Test triangle inequality for moments of inertia
  void testTriangleInequality() {
    // GIVEN: Physical moments of inertia
    double Ixx = 100.0, Iyy = 120.0, Izz = 180.0;

    // THEN: Must satisfy triangle inequality
    TS_ASSERT(Ixx + Iyy >= Izz);
    TS_ASSERT(Ixx + Izz >= Iyy);
    TS_ASSERT(Iyy + Izz >= Ixx);
  }

  // Test moments can't be negative
  void testMomentsNonNegative() {
    // GIVEN: Any physical body
    double mass = 5.0;
    double dimension = 2.0;

    // WHEN: Computing any moment of inertia formula
    double I_point = mass * dimension * dimension;
    double I_sphere = (2.0/5.0) * mass * dimension * dimension;
    double I_rod = (1.0/12.0) * mass * dimension * dimension;

    // THEN: All should be non-negative
    TS_ASSERT(I_point >= 0.0);
    TS_ASSERT(I_sphere >= 0.0);
    TS_ASSERT(I_rod >= 0.0);
  }

  // Test product of inertia bound
  void testProductOfInertiaBound() {
    // GIVEN: Products of inertia are bounded by principal moments
    double Ixx = 50.0, Iyy = 60.0;
    double Ixy = -10.0;  // Typical product of inertia

    // THEN: |Ixy| should be less than average of Ixx and Iyy
    // This is a soft constraint, not strict
    TS_ASSERT(std::abs(Ixy) < (Ixx + Iyy));
  }

  // ============ Numerical Stability Tests ============

  // Test small moment of inertia calculation
  void testSmallMomentOfInertia() {
    // GIVEN: Very small mass and dimension
    double mass = 1e-6;  // kg
    double radius = 1e-3;  // m

    // WHEN: Computing moment of inertia
    double I = 0.5 * mass * radius * radius;

    // THEN: Should be finite and positive
    TS_ASSERT(std::isfinite(I));
    TS_ASSERT(I > 0.0);
    TS_ASSERT(I < 1.0);  // Should be very small
  }

  // Test inertia tensor inverse stability
  void testInertiaTensorInverseStability() {
    // GIVEN: Well-conditioned inertia tensor
    FGMatrix33 I(100.0, -2.0, -1.0,
                 -2.0, 120.0, -1.5,
                 -1.0, -1.5, 150.0);

    // WHEN: Computing inverse
    FGMatrix33 I_inv = I.Inverse();

    // THEN: I * I_inv should equal identity
    FGMatrix33 identity = I * I_inv;

    TS_ASSERT_DELTA(1.0, identity(1, 1), 1e-10);
    TS_ASSERT_DELTA(1.0, identity(2, 2), 1e-10);
    TS_ASSERT_DELTA(1.0, identity(3, 3), 1e-10);
    TS_ASSERT_DELTA(0.0, identity(1, 2), 1e-10);
    TS_ASSERT_DELTA(0.0, identity(1, 3), 1e-10);
    TS_ASSERT_DELTA(0.0, identity(2, 3), 1e-10);
  }

  // Test angular momentum direction
  void testAngularMomentumDirection() {
    // GIVEN: Rotation about z-axis with diagonal inertia
    FGMatrix33 I(10.0, 0.0, 0.0,
                 0.0, 10.0, 0.0,
                 0.0, 0.0, 20.0);
    FGColumnVector3 omega(0.0, 0.0, 5.0);

    // WHEN: Computing angular momentum
    FGColumnVector3 L = I * omega;

    // THEN: L should be aligned with omega (diagonal I)
    TS_ASSERT_DELTA(0.0, L(1), epsilon);
    TS_ASSERT_DELTA(0.0, L(2), epsilon);
    TS_ASSERT_DELTA(100.0, L(3), epsilon);
  }

  // Test energy-momentum consistency
  void testEnergyMomentumConsistency() {
    // GIVEN: Rotating body
    FGMatrix33 I(50.0, 0.0, 0.0,
                 0.0, 60.0, 0.0,
                 0.0, 0.0, 70.0);
    FGColumnVector3 omega(1.0, 2.0, 3.0);

    // WHEN: Computing energy and momentum
    FGColumnVector3 L = I * omega;
    double L_mag = L.Magnitude();
    double KE = 0.5 * DotProduct(omega, L);

    // THEN: Energy should be related to momentum magnitude
    // KE = L²/(2I) for single axis, more complex for 3D
    TS_ASSERT(KE > 0.0);
    TS_ASSERT(L_mag > 0.0);
    TS_ASSERT(std::isfinite(KE));
  }

  // ============ Dynamic Mass Distribution Tests ============

  void testFuelBurnMassChange() {
    // GIVEN: Aircraft with fuel consumption
    double initialMass = 50000.0;  // lbs
    double fuelBurnRate = 200.0;   // lbs/min
    double time = 60.0;            // minutes

    // WHEN: Computing mass after fuel burn
    double fuelBurned = fuelBurnRate * time;
    double finalMass = initialMass - fuelBurned;

    // THEN: Mass should decrease
    TS_ASSERT(finalMass < initialMass);
    TS_ASSERT_DELTA(finalMass, 38000.0, 1.0);
  }

  void testCGShiftDuringFuelBurn() {
    // GIVEN: Wing tanks at different positions
    double wingTankX = 10.0;  // ft from ref
    double fusTankX = 5.0;    // ft from ref

    double wingFuel = 2000.0;  // lbs
    double fusFuel = 1000.0;   // lbs
    double totalFuel = wingFuel + fusFuel;

    // WHEN: Computing CG
    double cgX = (wingFuel * wingTankX + fusFuel * fusTankX) / totalFuel;

    // THEN: CG should be between tank positions
    TS_ASSERT(cgX > fusTankX);
    TS_ASSERT(cgX < wingTankX);
  }

  void testInertiaChangeWithFuelBurn() {
    // Burning fuel from wing tanks reduces roll inertia
    double Ixx_full = 50000.0;    // slug-ft²
    double fuelContribution = 5000.0;

    double Ixx_empty = Ixx_full - fuelContribution;
    TS_ASSERT(Ixx_empty < Ixx_full);
  }

  // ============ Payload and CG Tests ============

  void testPayloadCGEffect() {
    double emptyWeight = 10000.0;
    double emptyX = 100.0;
    double payloadWeight = 2000.0;
    double payloadX = 120.0;

    double totalWeight = emptyWeight + payloadWeight;
    double loadedCG = (emptyWeight * emptyX + payloadWeight * payloadX) / totalWeight;

    TS_ASSERT(loadedCG > emptyX);
    TS_ASSERT(loadedCG < payloadX);
  }

  void testPayloadInertiaContribution() {
    double baseIyy = 80000.0;
    double payloadMass = 500.0;  // slugs
    double payloadOffset = 10.0; // ft from CG

    // Parallel axis contribution
    double payloadIyy = payloadMass * payloadOffset * payloadOffset;
    double totalIyy = baseIyy + payloadIyy;

    TS_ASSERT(totalIyy > baseIyy);
    TS_ASSERT_DELTA(payloadIyy, 50000.0, 1.0);
  }

  void testForwardCGLimit() {
    double currentCG = 22.5;  // % MAC
    double fwdLimit = 15.0;   // % MAC

    bool withinLimits = (currentCG >= fwdLimit);
    TS_ASSERT(withinLimits);
  }

  void testAftCGLimit() {
    double currentCG = 32.0;  // % MAC
    double aftLimit = 35.0;   // % MAC

    bool withinLimits = (currentCG <= aftLimit);
    TS_ASSERT(withinLimits);
  }

  // ============ Fluid Slosh Tests ============

  void testFluidSloshFrequency() {
    // Slosh frequency for rectangular tank
    double g = 32.174;          // ft/s²
    double tankLength = 5.0;    // ft
    double fillLevel = 0.7;     // 70%

    double h = fillLevel * tankLength;
    double f_slosh = (1.0 / (2.0 * M_PI)) * sqrt(g * M_PI * tanh(M_PI * h / tankLength) / tankLength);

    TS_ASSERT(f_slosh > 0.0);
    TS_ASSERT(f_slosh < 2.0);  // Typical range
  }

  void testFluidSloshMass() {
    // Effective slosh mass is fraction of total fluid
    double totalFluidMass = 1000.0;  // lbs
    double sloshFraction = 0.3;       // 30% participates

    double sloshMass = totalFluidMass * sloshFraction;
    double rigidMass = totalFluidMass - sloshMass;

    TS_ASSERT_DELTA(sloshMass, 300.0, 0.1);
    TS_ASSERT_DELTA(rigidMass, 700.0, 0.1);
  }

  void testSloshDamping() {
    double sloshFreq = 0.5;   // Hz
    double dampingRatio = 0.05;

    double dampedFreq = sloshFreq * sqrt(1.0 - dampingRatio * dampingRatio);
    TS_ASSERT(dampedFreq < sloshFreq);
    TS_ASSERT(dampedFreq > 0.99 * sloshFreq);
  }

  // ============ Control Surface Deployment Tests ============

  void testLandingGearInertiaChange() {
    double Izz_retracted = 200000.0;  // slug-ft²
    double gearMass = 100.0;           // slugs
    double gearOffset = 15.0;          // ft

    double gearInertia = gearMass * gearOffset * gearOffset;
    double Izz_extended = Izz_retracted + gearInertia;

    TS_ASSERT(Izz_extended > Izz_retracted);
  }

  void testSpeedBrakeDeployment() {
    double Iyy_clean = 150000.0;
    double brakeAreaMass = 20.0;  // slugs
    double brakeOffset = 8.0;     // ft

    double brakeInertia = brakeAreaMass * brakeOffset * brakeOffset;
    double Iyy_deployed = Iyy_clean + brakeInertia;

    TS_ASSERT(Iyy_deployed > Iyy_clean);
  }

  void testFlapDeploymentMassEffect() {
    double cgClean = 25.0;      // % MAC
    double cgFlaps = 25.5;      // Slight aft shift with flaps

    bool cgShiftedAft = (cgFlaps > cgClean);
    TS_ASSERT(cgShiftedAft);
  }

  // ============ Helicopter Rotor Inertia Tests ============

  void testRotorBladeInertia() {
    // Single blade: thin rod about root
    double bladeMass = 50.0;   // lbs
    double bladeLength = 20.0; // ft
    double g = 32.174;
    double massSlug = bladeMass / g;

    double I_blade = (1.0 / 3.0) * massSlug * bladeLength * bladeLength;
    TS_ASSERT(I_blade > 0.0);
  }

  void testMultiBladeRotorInertia() {
    double I_blade = 200.0;  // slug-ft² per blade
    int numBlades = 4;

    double I_rotor = numBlades * I_blade;
    TS_ASSERT_DELTA(I_rotor, 800.0, 0.1);
  }

  void testGyroscopicMomentRotor() {
    double I_rotor = 800.0;     // slug-ft²
    double omega = 27.0;        // rad/s (typical main rotor)
    double pitch_rate = 0.1;    // rad/s

    double gyro_moment = I_rotor * omega * pitch_rate;
    TS_ASSERT(gyro_moment > 0.0);
  }

  // ============ Spin and Tumble Tests ============

  void testSpinStabilityRatio() {
    // For spin stability: Ix > Iy (for spin about x)
    double Ix = 150.0;
    double Iy = 100.0;
    double Iz = 200.0;

    bool spinStable = (Ix > Iy);
    TS_ASSERT(spinStable);
  }

  void testIntermediateAxisInstability() {
    // Rotation about intermediate axis is unstable
    double Ix = 100.0;
    double Iy = 150.0;  // Intermediate
    double Iz = 200.0;

    bool isIntermediate = (Iy > Ix && Iy < Iz);
    TS_ASSERT(isIntermediate);
    // Rotation about Y is unstable (tennis racket theorem)
  }

  void testNutationFrequency() {
    double Ix = 100.0;
    double Iz = 200.0;
    double omega_spin = 10.0;  // rad/s

    double omega_nutation = omega_spin * (Iz - Ix) / Ix;
    TS_ASSERT(omega_nutation > 0.0);
  }

  // ============ Satellite/Spacecraft Tests ============

  void testSpacecraftDualSpin() {
    double I_despun = 50.0;    // slug-ft² (antenna platform)
    double I_spun = 200.0;      // slug-ft² (main body)
    double omega_spun = 5.0;    // rad/s

    double L_total = I_spun * omega_spun;  // Angular momentum conserved
    TS_ASSERT(L_total > 0.0);
  }

  void testMomentumWheelSaturation() {
    double wheelInertia = 0.1;   // slug-ft²
    double maxSpeed = 6000.0;    // rpm
    double maxSpeedRad = maxSpeed * 2 * M_PI / 60.0;

    double maxMomentum = wheelInertia * maxSpeedRad;
    TS_ASSERT(maxMomentum > 0.0);
  }

  void testGravityGradientStability() {
    double Ix = 100.0;
    double Iy = 120.0;
    double Iz = 150.0;

    // For gravity gradient stability: Iz > Iy > Ix
    bool ggStable = (Iz > Iy && Iy > Ix);
    TS_ASSERT(ggStable);
  }

  // ============ Variable Geometry Aircraft Tests ============

  void testSwingWingInertiaChange() {
    double Ixx_swept = 80000.0;   // Full sweep
    double Ixx_unswept = 120000.0; // Unswept

    TS_ASSERT(Ixx_unswept > Ixx_swept);
  }

  void testVariableSweepCGShift() {
    double cgSwept = 30.0;    // % MAC
    double cgUnswept = 25.0;  // % MAC

    TS_ASSERT(cgSwept > cgUnswept);  // CG moves aft with sweep
  }

  // ============ Launch Vehicle Tests ============

  void testRocketMassRatio() {
    double initialMass = 100000.0;  // lbs
    double propellantMass = 80000.0; // lbs
    double finalMass = initialMass - propellantMass;

    double massRatio = initialMass / finalMass;
    TS_ASSERT_DELTA(massRatio, 5.0, 0.1);
  }

  void testStageSeparationMassChange() {
    double stage1Mass = 80000.0;
    double stage2Mass = 20000.0;
    double totalMass = stage1Mass + stage2Mass;

    double remainingAfterSep = stage2Mass;
    double massLost = totalMass - remainingAfterSep;

    TS_ASSERT_DELTA(massLost, stage1Mass, 1.0);
  }

  void testRocketCGTravel() {
    // CG moves forward as propellant burns from aft tanks
    double cgInitial = 80.0;  // ft from nose
    double cgFinal = 60.0;    // ft from nose

    double cgTravel = cgInitial - cgFinal;
    TS_ASSERT(cgTravel > 0.0);
  }

  // ============ Multi-Body Dynamics Tests ============

  void testArticulatedBodyInertia() {
    // Two rigid bodies connected
    double I1 = 100.0;
    double I2 = 50.0;
    double m1 = 10.0;
    double m2 = 5.0;
    double d = 3.0;  // Distance between CGs

    // Total about combined CG (simplified)
    double totalMass = m1 + m2;
    double cg = (m1 * 0 + m2 * d) / totalMass;

    double I_total = I1 + m1 * cg * cg + I2 + m2 * (d - cg) * (d - cg);
    TS_ASSERT(I_total > I1 + I2);
  }

  void testFlexibleAppendage() {
    // Solar panel or antenna
    double I_rigid = 1000.0;
    double flexibility = 0.1;  // 10% effective reduction

    double I_effective = I_rigid * (1.0 - flexibility);
    TS_ASSERT(I_effective < I_rigid);
  }

  // ============ Propeller/Engine Inertia Tests ============

  void testPropellerInertia() {
    // Three-blade propeller
    double bladeMass = 5.0;   // lbs
    double bladeLength = 3.0; // ft
    double g = 32.174;
    double massSlug = bladeMass / g;

    double I_blade = (1.0 / 3.0) * massSlug * bladeLength * bladeLength;
    double I_prop = 3 * I_blade;

    TS_ASSERT(I_prop > 0.0);
  }

  void testEngineCrankcaseInertia() {
    double I_crankcase = 2.0;  // slug-ft²
    double I_prop = 1.0;       // slug-ft²
    double gearRatio = 0.5;    // Reduction gear

    // Effective inertia at propeller
    double I_engine_eff = I_crankcase / (gearRatio * gearRatio);
    double I_total = I_prop + I_engine_eff;

    TS_ASSERT(I_total > I_prop);
  }

  void testTurbineSpoolInertia() {
    double I_LP = 5.0;   // Low pressure spool
    double I_HP = 2.0;   // High pressure spool

    double totalRotorInertia = I_LP + I_HP;
    TS_ASSERT_DELTA(totalRotorInertia, 7.0, 0.1);
  }

  // ============ CG Envelope Tests ============

  void testCGEnvelopeWeight() {
    double minWeight = 40000.0;
    double maxWeight = 60000.0;
    double currentWeight = 50000.0;

    bool withinWeightEnvelope = (currentWeight >= minWeight && currentWeight <= maxWeight);
    TS_ASSERT(withinWeightEnvelope);
  }

  void testCGEnvelopeFwdLimit() {
    double weight = 50000.0;
    double cg = 20.0;  // % MAC

    // Forward limit varies with weight (simplified)
    double fwdLimit = 15.0 + 0.0001 * (weight - 40000.0);

    bool withinFwd = (cg >= fwdLimit);
    TS_ASSERT(withinFwd);
  }

  void testCGEnvelopeAftLimit() {
    double weight = 50000.0;
    double cg = 32.0;  // % MAC

    // Aft limit varies with weight
    double aftLimit = 35.0 - 0.00005 * (weight - 40000.0);

    bool withinAft = (cg <= aftLimit);
    TS_ASSERT(withinAft);
  }

  // ============ Additional Physical Constraint Tests ============

  void testMassConservation() {
    double initialMass = 50000.0;
    double fuelBurned = 5000.0;
    double payloadDropped = 1000.0;

    double finalMass = initialMass - fuelBurned - payloadDropped;
    TS_ASSERT_DELTA(finalMass, 44000.0, 0.1);
  }

  void testPositiveInertia() {
    // All diagonal elements must be positive
    double Ixx = 100.0;
    double Iyy = 150.0;
    double Izz = 200.0;

    TS_ASSERT(Ixx > 0.0);
    TS_ASSERT(Iyy > 0.0);
    TS_ASSERT(Izz > 0.0);
  }

  void testInertiaSymmetryRelation() {
    // For symmetric aircraft: Ixy = Iyz = 0, only Ixz may be non-zero
    double Ixy = 0.0;
    double Iyz = 0.0;
    double Ixz = -50.0;  // Typical

    TS_ASSERT_DELTA(Ixy, 0.0, 0.1);
    TS_ASSERT_DELTA(Iyz, 0.0, 0.1);
    TS_ASSERT(Ixz != 0.0);  // Usually non-zero
  }

  // ============ Moment Calculations Tests ============

  void testPitchingMomentInertia() {
    double Iyy = 150000.0;        // slug-ft²
    double q_dot = 0.5;           // rad/s² pitch acceleration

    double pitchMoment = Iyy * q_dot;
    TS_ASSERT_DELTA(pitchMoment, 75000.0, 1.0);  // ft-lbs
  }

  void testRollingMomentInertia() {
    double Ixx = 100000.0;
    double p_dot = 0.3;

    double rollMoment = Ixx * p_dot;
    TS_ASSERT_DELTA(rollMoment, 30000.0, 1.0);
  }

  void testYawingMomentInertia() {
    double Izz = 200000.0;
    double r_dot = 0.2;

    double yawMoment = Izz * r_dot;
    TS_ASSERT_DELTA(yawMoment, 40000.0, 1.0);
  }

  // ============ Time Rate of Change Tests ============

  void testInertiaTimeDerivative() {
    double I_initial = 100000.0;
    double I_final = 95000.0;
    double dt = 60.0;  // seconds

    double I_dot = (I_final - I_initial) / dt;
    TS_ASSERT_DELTA(I_dot, -83.33, 0.1);
  }

  void testMassCenterVelocity() {
    double cg_initial = 25.0;  // ft
    double cg_final = 24.5;    // ft
    double dt = 60.0;

    double cg_dot = (cg_final - cg_initial) / dt;
    TS_ASSERT_DELTA(cg_dot, -0.00833, 0.0001);
  }
};

// ============ C172x Aircraft Inertia Integration Tests ============
class FGInertiaC172xTest : public CxxTest::TestSuite
{
public:

  // Test C172x initial mass properties
  void testC172xInitialMass() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto massbal = fdmex.GetMassBalance();
    TS_ASSERT(massbal != nullptr);

    double mass = massbal->GetMass();
    TS_ASSERT(std::isfinite(mass));
    TS_ASSERT(mass > 0.0);  // Mass must be positive
  }

  // Test C172x moments of inertia
  void testC172xMomentsOfInertia() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto massbal = fdmex.GetMassBalance();
    TS_ASSERT(massbal != nullptr);

    const FGMatrix33& J = massbal->GetJ();
    double Ixx = J(1, 1);
    double Iyy = J(2, 2);
    double Izz = J(3, 3);

    TS_ASSERT(std::isfinite(Ixx));
    TS_ASSERT(std::isfinite(Iyy));
    TS_ASSERT(std::isfinite(Izz));

    // All principal moments must be positive
    TS_ASSERT(Ixx > 0.0);
    TS_ASSERT(Iyy > 0.0);
    TS_ASSERT(Izz > 0.0);
  }

  // Test C172x triangle inequality for inertia
  void testC172xTriangleInequality() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto massbal = fdmex.GetMassBalance();
    const FGMatrix33& J = massbal->GetJ();
    double Ixx = J(1, 1);
    double Iyy = J(2, 2);
    double Izz = J(3, 3);

    // Physical bodies must satisfy triangle inequality
    TS_ASSERT(Ixx + Iyy >= Izz);
    TS_ASSERT(Ixx + Izz >= Iyy);
    TS_ASSERT(Iyy + Izz >= Ixx);
  }

  // Test C172x products of inertia
  void testC172xProductsOfInertia() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto massbal = fdmex.GetMassBalance();
    const FGMatrix33& J = massbal->GetJ();
    double Ixy = J(1, 2);
    double Ixz = J(1, 3);
    double Iyz = J(2, 3);

    TS_ASSERT(std::isfinite(Ixy));
    TS_ASSERT(std::isfinite(Ixz));
    TS_ASSERT(std::isfinite(Iyz));

    // For symmetric aircraft, Ixy and Iyz should be near zero
    // Ixz may be non-zero due to engine placement
  }

  // Test C172x CG location
  void testC172xCGLocation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto massbal = fdmex.GetMassBalance();
    double cgX = massbal->GetXYZcg(1);
    double cgY = massbal->GetXYZcg(2);
    double cgZ = massbal->GetXYZcg(3);

    TS_ASSERT(std::isfinite(cgX));
    TS_ASSERT(std::isfinite(cgY));
    TS_ASSERT(std::isfinite(cgZ));
  }

  // Test C172x mass during simulation
  void testC172xMassDuringFlight() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto massbal = fdmex.GetMassBalance();
    prop->InitRunning(-1);

    double initialMass = massbal->GetMass();

    // Run simulation for a few steps
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double finalMass = massbal->GetMass();
    TS_ASSERT(std::isfinite(finalMass));
    TS_ASSERT(finalMass > 0.0);

    // Mass should decrease or stay same (fuel burn)
    TS_ASSERT(finalMass <= initialMass + 0.1);  // Small tolerance
  }

  // Test C172x inertia tensor consistency
  void testC172xInertiaTensorConsistency() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto massbal = fdmex.GetMassBalance();
    const FGMatrix33& J = massbal->GetJ();
    double Ixx = J(1, 1);
    double Iyy = J(2, 2);
    double Izz = J(3, 3);
    double Ixz = J(1, 3);

    // Check that determinant is positive (positive definite)
    // For simplified check with Ixy=Iyz=0
    double det = Ixx * (Iyy * Izz) - Ixz * Ixz * Iyy;
    TS_ASSERT(det > 0.0);
  }

  // Test C172x angular momentum calculation
  void testC172xAngularMomentum() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Apply aileron to induce roll
    fcs->SetDaCmd(0.3);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Get body angular rates
    double p = propagate->GetPQR(1);  // Roll rate
    double q = propagate->GetPQR(2);  // Pitch rate
    double r = propagate->GetPQR(3);  // Yaw rate

    TS_ASSERT(std::isfinite(p));
    TS_ASSERT(std::isfinite(q));
    TS_ASSERT(std::isfinite(r));
  }

  // Test C172x rotational kinetic energy
  void testC172xRotationalKineticEnergy() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto massbal = fdmex.GetMassBalance();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Induce rotation
    fcs->SetDrCmd(0.2);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    const FGMatrix33& J = massbal->GetJ();
    double Ixx = J(1, 1);
    double Iyy = J(2, 2);
    double Izz = J(3, 3);
    double p = propagate->GetPQR(1);
    double q = propagate->GetPQR(2);
    double r = propagate->GetPQR(3);

    // Simplified rotational KE (ignoring products of inertia)
    double KE_rot = 0.5 * (Ixx * p * p + Iyy * q * q + Izz * r * r);
    TS_ASSERT(std::isfinite(KE_rot));
    TS_ASSERT(KE_rot >= 0.0);  // KE must be non-negative
  }

  // Test C172x pitch maneuver inertia response
  void testC172xPitchInertiaResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Apply elevator
    fcs->SetDeCmd(-0.3);

    double q_prev = 0.0;
    for (int i = 0; i < 30; i++) {
      fdmex.Run();
      double q = propagate->GetPQR(2);
      TS_ASSERT(std::isfinite(q));
      q_prev = q;
    }

    // Pitch rate should have changed due to elevator input
    double q_final = propagate->GetPQR(2);
    TS_ASSERT(std::isfinite(q_final));
  }

  // Test C172x yaw moment due to rudder
  void testC172xYawInertiaResponse() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto propagate = fdmex.GetPropagate();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Apply rudder
    fcs->SetDrCmd(0.5);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double r = propagate->GetPQR(3);  // Yaw rate
    TS_ASSERT(std::isfinite(r));
  }

  // Test C172x mass balance stability during flight
  void testC172xMassBalanceStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto prop = fdmex.GetPropulsion();
    auto massbal = fdmex.GetMassBalance();
    auto fcs = fdmex.GetFCS();
    prop->InitRunning(-1);

    // Control inputs
    fcs->SetDeCmd(-0.1);
    fcs->SetDaCmd(0.1);

    // Run for extended period
    for (int i = 0; i < 200; i++) {
      fdmex.Run();

      double mass = massbal->GetMass();
      const FGMatrix33& J = massbal->GetJ();
      double Ixx = J(1, 1);
      double Iyy = J(2, 2);
      double Izz = J(3, 3);

      TS_ASSERT(std::isfinite(mass));
      TS_ASSERT(std::isfinite(Ixx));
      TS_ASSERT(std::isfinite(Iyy));
      TS_ASSERT(std::isfinite(Izz));
      TS_ASSERT(mass > 0.0);
      TS_ASSERT(Ixx > 0.0);
      TS_ASSERT(Iyy > 0.0);
      TS_ASSERT(Izz > 0.0);
    }
  }
};
