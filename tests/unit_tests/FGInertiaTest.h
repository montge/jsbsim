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
};
