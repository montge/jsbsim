#include <limits>
#include <cxxtest/TestSuite.h>
#include "TestAssertions.h"
#include <math/FGQuaternion.h>
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();

class FGQuaternionTest : public CxxTest::TestSuite
{
public:
  void testConstructors() {
    double angle = 10. * M_PI / 180.;
    double ca = cos(angle), sa = sin(angle);
    double ca_half = cos(0.5*angle), sa_half = sin(0.5*angle);
    // Default constructor
    JSBSim::FGQuaternion q;
    TS_ASSERT_EQUALS(1.0, q(1));
    TS_ASSERT_EQUALS(0.0, q(2));
    TS_ASSERT_EQUALS(0.0, q(3));
    TS_ASSERT_EQUALS(0.0, q(4));

    // Several ways to build a quaternion representing a rotation of an angle
    // 'angle' around the X axis
    q = JSBSim::FGQuaternion(1, angle);
    TS_ASSERT_DELTA(ca_half, q(1), epsilon);
    TS_ASSERT_DELTA(sa_half ,q(2), epsilon);
    TS_ASSERT_EQUALS(0.0, q(3));
    TS_ASSERT_EQUALS(0.0, q(4));
    JSBSim::FGQuaternion q2(angle, 0.0, 0.0);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    JSBSim::FGColumnVector3 v(angle, 0.0, 0.0);
    q2 = JSBSim::FGQuaternion(v);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0, 0.0, ca, sa, 0.0, -sa, ca);
    q2 = JSBSim::FGQuaternion(m);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    JSBSim::FGColumnVector3 euler = q2.GetEuler();
    TS_ASSERT_DELTA(v(1), euler(1), epsilon);
    TS_ASSERT_DELTA(v(2), euler(2), epsilon);
    TS_ASSERT_DELTA(v(3), euler(3), epsilon);
    JSBSim::FGMatrix33 mT = q2.GetT();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(i,j), epsilon);
    mT = JSBSim::FGMatrix33(q2);
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(i,j), epsilon);
    mT = q2.GetTInv();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(j,i), epsilon);

    // around the Y axis
    q = JSBSim::FGQuaternion(2, angle);
    TS_ASSERT_DELTA(ca_half, q(1), epsilon);
    TS_ASSERT_EQUALS(0.0, q(2));
    TS_ASSERT_DELTA(sa_half, q(3), epsilon);
    TS_ASSERT_EQUALS(0.0, q(4));
    q2 = JSBSim::FGQuaternion(0.0, angle, 0.0);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    v.InitMatrix(0.0, angle, 0.0);
    q2 = JSBSim::FGQuaternion(v);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    m.InitMatrix(ca, 0.0, -sa, 0.0, 1.0, 0.0, sa, 0.0, ca);
    q2 = JSBSim::FGQuaternion(m);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    euler = q2.GetEuler();
    TS_ASSERT_DELTA(v(1), euler(1), epsilon);
    TS_ASSERT_DELTA(v(2), euler(2), epsilon);
    TS_ASSERT_DELTA(v(3), euler(3), epsilon);
    mT = q2.GetT();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(i,j), epsilon);
    mT = JSBSim::FGMatrix33(q2);
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(i,j), epsilon);
    mT = q2.GetTInv();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(j,i), epsilon);

    // around the Z axis
    q = JSBSim::FGQuaternion(3, angle);
    TS_ASSERT_DELTA(ca_half, q(1), epsilon);
    TS_ASSERT_DELTA(0.0, q(2), epsilon);
    TS_ASSERT_DELTA(0.0, q(3), epsilon);
    TS_ASSERT_DELTA(sa_half, q(4), epsilon);
    q2 = JSBSim::FGQuaternion(0.0, 0.0, angle);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    v.InitMatrix(0.0, 0.0, angle);
    q2 = JSBSim::FGQuaternion(v);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    m.InitMatrix(ca, sa, 0.0, -sa, ca, 0.0, 0.0, 0.0, 1.0);
    q2 = JSBSim::FGQuaternion(m);
    TS_ASSERT_DELTA(q(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q(4), q2(4), epsilon);
    euler = q2.GetEuler();
    TS_ASSERT_DELTA(v(1), euler(1), epsilon);
    TS_ASSERT_DELTA(v(2), euler(2), epsilon);
    TS_ASSERT_DELTA(v(3), euler(3), epsilon);
    mT = q2.GetT();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(i,j), epsilon);
    mT = JSBSim::FGMatrix33(q2);
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(i,j), epsilon);
    mT = q2.GetTInv();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++)
        TS_ASSERT_DELTA(m(i,j), mT(j,i), epsilon);

    // Constructor with an angle and an axis of rotation.
    v.InitMatrix(1.0, 2.0, -0.5);
    q2 = JSBSim::FGQuaternion(angle, v);
    v.Normalize();
    TS_ASSERT_DELTA(q2(1), ca_half, epsilon);
    TS_ASSERT_DELTA(q2(2), sa_half*v(1), epsilon);
    TS_ASSERT_DELTA(q2(3), sa_half*v(2), epsilon);
    TS_ASSERT_DELTA(q2(4), sa_half*v(3), epsilon);

    // Initializes to zero.
    q2 = JSBSim::FGQuaternion::zero();
    TS_ASSERT_EQUALS(0.0, q2.Entry(1));
    TS_ASSERT_EQUALS(0.0, q2.Entry(2));
    TS_ASSERT_EQUALS(0.0, q2.Entry(3));
    TS_ASSERT_EQUALS(0.0, q2.Entry(4));
  }

  void testComponentWise() {
    JSBSim::FGQuaternion q(0.5, 1.0, -0.75);
    double x = q(1);
    double y = q(2);
    double z = q(3);
    double w = q(4);
    q.Entry(1) = x + 1.0;
    TS_ASSERT_EQUALS(q.Entry(1), x + 1.0);
    // Check there are no side effects on other components
    TS_ASSERT_EQUALS(q.Entry(2), y);
    TS_ASSERT_EQUALS(q.Entry(3), z);
    TS_ASSERT_EQUALS(q.Entry(4), w);
  }

  void testCopyConstructor() {
    JSBSim::FGQuaternion q0(0.5, 1.0, -0.75);
    JSBSim::FGQuaternion q1(q0); // Copy before updating the cache

    // First make sure that q0 and q1 are identical
    TS_ASSERT_DELTA(q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q1(4), epsilon);

    // Second, make sure that q0 and q1 are distinct copies
    // i.e. that q0 and q1 does not point to the same memory location
    double z = q0.Entry(2);
    q1.Entry(2) = 5.0;
    TS_ASSERT_DELTA(z, q0.Entry(2), epsilon); // q0[2] must remain unchanged
    TS_ASSERT_DELTA(5.0, q1.Entry(2), epsilon); // q1[2] must now contain 5.0

    // Force the cache update
    TS_ASSERT_DELTA(0.5, q0.GetEuler(1), epsilon);

    JSBSim::FGQuaternion q2(q0);

    // First make sure that q0 and q2 are identical
    TS_ASSERT_DELTA(q0(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q2(4), epsilon);

    // Second, make sure that q0 and q2 are distinct copies
    // i.e. that q0 and q2 does not point to the same memory location
    z = q0.Entry(2);
    q2.Entry(2) = 5.0;
    TS_ASSERT_DELTA(z, q0.Entry(2), epsilon); // q0[2] must remain unchanged
    TS_ASSERT_DELTA(5.0, q2.Entry(2), epsilon); // q2[2] must now contain 5.0
  }

  void testEquality() {
    JSBSim::FGQuaternion q0(0.5, 1.0, -0.75);
    JSBSim::FGQuaternion q1(q0);
    TS_ASSERT_EQUALS(q0, q1);
    q1(1) += 0.1;
    TS_ASSERT(!(q0 == q1));
    TS_ASSERT_DIFFERS(q0, q1);
    q1(1) = q0(1);
    q1(2) += 0.1;
    TS_ASSERT(!(q0 == q1));
    TS_ASSERT_DIFFERS(q0, q1);
    q1(2) = q0(2);
    q1(3) += 0.1;
    TS_ASSERT(!(q0 == q1));
    TS_ASSERT_DIFFERS(q0, q1);
    q1(3) = q0(3);
    q1(4) += 0.1;
    TS_ASSERT(!(q0 == q1));
    TS_ASSERT_DIFFERS(q0, q1);
  }

  void testAssignment() {
    JSBSim::FGQuaternion q0(0.5, 1.0, -0.75);
    JSBSim::FGQuaternion q1 = q0; // Copy before updating the cache

    // First make sure that q0 and q1 are identical
    TS_ASSERT_DELTA(q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q1(4), epsilon);

    // Second, make sure that q0 and q1 are distinct copies
    // i.e. that q0 and q1 does not point to the same memory location
    double z = q0.Entry(2);
    q1.Entry(2) = 5.0;
    TS_ASSERT_DELTA(z, q0.Entry(2), epsilon); // q0[2] must remain unchanged
    TS_ASSERT_DELTA(5.0, q1.Entry(2), epsilon); // q1[2] must now contain 5.0

    const JSBSim::FGQuaternion q2 = q0;

    // First make sure that q0 and q2 are identical
    TS_ASSERT_DELTA(q0(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q2(4), epsilon);

    // Second, make sure that q0 and q2 are distinct copies
    // i.e. that q0 and q2 does not point to the same memory location
    z = q2.Entry(2);
    q0.Entry(2) = 5.0;
    TS_ASSERT_DELTA(z, q2.Entry(2), epsilon); // q2[2] must remain unchanged
    TS_ASSERT_DELTA(5.0, q0.Entry(2), epsilon); // q0[2] must now contain 5.0

    // Test the assignment of a quaternion with a valid cache.
    q0(3) = -1.5;
    JSBSim::FGMatrix33 m = q0.GetT();
    JSBSim::FGColumnVector3 v = q0.GetEuler();
    q1 = q0;
    TS_ASSERT_DELTA(q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q1(4), epsilon);
    TS_ASSERT_VECTOR_EQUALS(v, q1.GetEuler());
    TS_ASSERT_MATRIX_EQUALS(m, q1.GetT());
    TS_ASSERT_MATRIX_EQUALS(m.Transposed(), q1.GetTInv());
  }

  void testEulerAngles() {
    JSBSim::FGQuaternion q0(0.5, 1.0, -0.75);

    // Euler angles in radians
    double x = q0.GetEuler(1);
    double y = q0.GetEuler(2);
    double z = q0.GetEuler(3);
    x = x > M_PI ? x - 2.*M_PI : x;
    x = x < -M_PI ? x + 2.*M_PI : x;
    y = y > M_PI ? y - 2.*M_PI : y;
    y = y < -M_PI ? y + 2.*M_PI : y;
    z = z > M_PI ? z - 2.*M_PI : z;
    z = z < -M_PI ? z + 2.*M_PI : z;
    TS_ASSERT_DELTA(0.5, x, epsilon);
    TS_ASSERT_DELTA(1.0, y, epsilon);
    TS_ASSERT_DELTA(-0.75, z, epsilon);

    JSBSim::FGColumnVector3 euler = q0.GetEuler();
    x = euler(1);
    y = euler(2);
    z = euler(3);
    x = x > M_PI ? x - 2.*M_PI : x;
    x = x < -M_PI ? x + 2.*M_PI : x;
    y = y > M_PI ? y - 2.*M_PI : y;
    y = y < -M_PI ? y + 2.*M_PI : y;
    z = z > M_PI ? z - 2.*M_PI : z;
    z = z < -M_PI ? z + 2.*M_PI : z;
    TS_ASSERT_DELTA(0.5, x, epsilon);
    TS_ASSERT_DELTA(1.0, y, epsilon);
    TS_ASSERT_DELTA(-0.75, z, epsilon);

    // Euler angles in degrees
    q0 = JSBSim::FGQuaternion(M_PI / 3.0, 0.25 * M_PI, -M_PI / 6.0);

    x = q0.GetEulerDeg(1);
    y = q0.GetEulerDeg(2);
    z = q0.GetEulerDeg(3);
    x = x > 180.0 ? x - 360. : x;
    x = x < -180.0 ? x + 360. : x;
    y = y > 180.0 ? y - 360. : y;
    y = y < -180.0 ? y + 360. : y;
    z = z > 180.0 ? z - 360. : z;
    z = z < -180.0 ? z + 360. : z;
    TS_ASSERT_DELTA(60., x, epsilon);
    TS_ASSERT_DELTA(45., y, epsilon);
#ifdef __arm64__
    TS_ASSERT_DELTA(-30., z, epsilon*10.);
#else
    TS_ASSERT_DELTA(-30., z, epsilon);
#endif

    euler = q0.GetEulerDeg();
    x = euler(1);
    y = euler(2);
    z = euler(3);
    x = x > 180.0 ? x - 360. : x;
    x = x < -180.0 ? x + 360. : x;
    y = y > 180.0 ? y - 360. : y;
    y = y < -180.0 ? y + 360. : y;
    z = z > 180.0 ? z - 360. : z;
    z = z < -180.0 ? z + 360. : z;
    TS_ASSERT_DELTA(60., x, epsilon);
    TS_ASSERT_DELTA(45., y, epsilon);
#ifdef __arm64__
    TS_ASSERT_DELTA(-30., z, epsilon*10.);
#else
    TS_ASSERT_DELTA(-30., z, epsilon);
#endif

    // Euler angles sin
    TS_ASSERT_DELTA(0.5*sqrt(3), q0.GetSinEuler(1), epsilon);
    TS_ASSERT_DELTA(0.5*sqrt(2), q0.GetSinEuler(2), epsilon);
    TS_ASSERT_DELTA(-0.5, q0.GetSinEuler(3), epsilon);

    // Euler angles cos
    TS_ASSERT_DELTA(0.5, q0.GetCosEuler(1), epsilon);
    TS_ASSERT_DELTA(0.5*sqrt(2), q0.GetCosEuler(2), epsilon);
    TS_ASSERT_DELTA(0.5*sqrt(3), q0.GetCosEuler(3), epsilon);

    JSBSim::FGColumnVector3 v = q0.GetEulerDeg();
    TS_ASSERT_DELTA(v(1), euler(1), epsilon);
    TS_ASSERT_DELTA(v(2), euler(2), epsilon);
    TS_ASSERT_DELTA(v(3), euler(3), epsilon);
}

  void testOperations() {
    double angle = 10. * M_PI / 180.;
    const JSBSim::FGQuaternion q0(0.5, 1.0, -0.75);
    const JSBSim::FGQuaternion unit;
    JSBSim::FGQuaternion q1 = q0, zero;

    q1 *= 2.0;
    TS_ASSERT_DELTA(q1(1), 2.0 * q0(1), epsilon);
    TS_ASSERT_DELTA(q1(2), 2.0 * q0(2), epsilon);
    TS_ASSERT_DELTA(q1(3), 2.0 * q0(3), epsilon);
    TS_ASSERT_DELTA(q1(4), 2.0 * q0(4), epsilon);

    q1 = 2.0 * q0;
    TS_ASSERT_DELTA(q1(1), 2.0 * q0(1), epsilon);
    TS_ASSERT_DELTA(q1(2), 2.0 * q0(2), epsilon);
    TS_ASSERT_DELTA(q1(3), 2.0 * q0(3), epsilon);
    TS_ASSERT_DELTA(q1(4), 2.0 * q0(4), epsilon);

    q1 /= 2.0;
    TS_ASSERT_DELTA(q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q1(4), epsilon);

    q1 = q0;
    q1 += q0;
    TS_ASSERT_DELTA(q1(1), 2.0 * q0(1), epsilon);
    TS_ASSERT_DELTA(q1(2), 2.0 * q0(2), epsilon);
    TS_ASSERT_DELTA(q1(3), 2.0 * q0(3), epsilon);
    TS_ASSERT_DELTA(q1(4), 2.0 * q0(4), epsilon);

    q1 -= q0;
    TS_ASSERT_DELTA(q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q1(4), epsilon);

    q1 = q0 + q0;
    TS_ASSERT_DELTA(q1(1), 2.0 * q0(1), epsilon);
    TS_ASSERT_DELTA(q1(2), 2.0 * q0(2), epsilon);
    TS_ASSERT_DELTA(q1(3), 2.0 * q0(3), epsilon);
    TS_ASSERT_DELTA(q1(4), 2.0 * q0(4), epsilon);

    q1 = q1 - q0;
    TS_ASSERT_DELTA(q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q1(4), epsilon);

    q1 = q0.Conjugate();
    TS_ASSERT_DELTA(q1(1), q0(1), epsilon);
    TS_ASSERT_DELTA(q1(2), -q0(2), epsilon);
    TS_ASSERT_DELTA(q1(3), -q0(3), epsilon);
    TS_ASSERT_DELTA(q1(4), -q0(4), epsilon);

    q1 *= q0;
    TS_ASSERT_DELTA(q0.SqrMagnitude(), q1(1), epsilon);
    TS_ASSERT_DELTA(0.0, q1(2), epsilon);
    TS_ASSERT_DELTA(0.0, q1(3), epsilon);
    TS_ASSERT_DELTA(0.0, q1(4), epsilon);

    q1 = q0.Inverse();
    q1 *= q0;
    TS_ASSERT_DELTA(unit(1), q1(1), epsilon);
    TS_ASSERT_DELTA(unit(2), q1(2), epsilon);
    TS_ASSERT_DELTA(unit(3), q1(3), epsilon);
    TS_ASSERT_DELTA(unit(4), q1(4), epsilon);

    //Check the inverse of a null quaternion
    zero = q1 - q1;
    TS_ASSERT_EQUALS(0.0, zero(1));
    TS_ASSERT_EQUALS(0.0, zero(2));
    TS_ASSERT_EQUALS(0.0, zero(3));
    TS_ASSERT_EQUALS(0.0, zero(4));
    q1 = zero.Inverse();
    TS_ASSERT_EQUALS(q1, zero);

    q1 = JSBSim::FGQuaternion(1, angle);
    q1 = q1.Conjugate();
    JSBSim::FGColumnVector3 euler = q1.GetEuler();
    TS_ASSERT_DELTA(-angle, euler(1), epsilon);
    TS_ASSERT_DELTA(0.0, euler(2), epsilon);
    TS_ASSERT_DELTA(0.0, euler(3), epsilon);

    q1 = q0 * JSBSim::FGQuaternion(1, angle);
    euler = q1.GetEuler();
    double z = euler(3);
    z = z > M_PI ? z - 2.0 * M_PI : z;
    z = z < -M_PI ? z + 2.0 * M_PI : z;
    TS_ASSERT_DELTA(0.5 + angle, euler(1), epsilon);
    TS_ASSERT_DELTA(1.0, euler(2), epsilon);
    TS_ASSERT_DELTA(-0.75, z, epsilon);

    q1 = JSBSim::FGQuaternion(3, angle) * q0;
    euler = q1.GetEuler();
    z = euler(3);
    z = z > M_PI ? z - 2.0 * M_PI : z;
    z = z < -M_PI ? z + 2.0 * M_PI : z;
    TS_ASSERT_DELTA(0.5, euler(1), epsilon);
    TS_ASSERT_DELTA(1.0, euler(2), epsilon);
    TS_ASSERT_DELTA(angle-0.75, z, epsilon);
  }

  void testFunctions() {
    JSBSim::FGColumnVector3 omega(3., 4., 0.);
    omega.Normalize();
    omega *= M_PI / 6.0;
    JSBSim::FGQuaternion q1 = QExp(omega);
    TS_ASSERT_DELTA(0.5 * sqrt(3.0), q1(1), epsilon);
    TS_ASSERT_DELTA(0.3, q1(2), epsilon);
    TS_ASSERT_DELTA(0.4, q1(3), epsilon);
    TS_ASSERT_EQUALS(0.0, q1(4));

    omega.InitMatrix();
    q1 = QExp(omega);
    TS_ASSERT_EQUALS(1.0, q1(1));
    TS_ASSERT_EQUALS(0.0, q1(2));
    TS_ASSERT_EQUALS(0.0, q1(3));
    TS_ASSERT_EQUALS(0.0, q1(4));

    omega(3) = -1.0;
    JSBSim::FGQuaternion q2 = q1.GetQDot(omega);
    JSBSim::FGQuaternion q_omega, qref;
    q_omega(1) = 0.0;
    q_omega(2) = 0.5 * omega(1);
    q_omega(3) = 0.5 * omega(2);
    q_omega(4) = 0.5 * omega(3);
    qref = q_omega * q1;
    TS_ASSERT_EQUALS(qref, q2);
  }

  void testNormalize() {
    JSBSim::FGQuaternion q0, q1, zero;
    q1.Normalize();
    TS_ASSERT_EQUALS(q0, q1);

    zero = q0 - q1;
    // Check that 'zero' is null
    TS_ASSERT_EQUALS(0.0, zero(1));
    TS_ASSERT_EQUALS(0.0, zero(2));
    TS_ASSERT_EQUALS(0.0, zero(3));
    TS_ASSERT_EQUALS(0.0, zero(4));
    // Check that Normalize is a no-op on null quaternions
    zero.Normalize();
    TS_ASSERT_EQUALS(0.0, zero(1));
    TS_ASSERT_EQUALS(0.0, zero(2));
    TS_ASSERT_EQUALS(0.0, zero(3));
    TS_ASSERT_EQUALS(0.0, zero(4));

    // Test the normalization of quaternion which magnitude is neither zero nor
    // unity.
    JSBSim::FGColumnVector3 v(1.0, 2.0, -0.5);
    q0 = JSBSim::FGQuaternion(0.4, v);
    double x = q0(1);
    double y = q0(2);
    double z = q0(3);
    double w = q0(4);
    q0 *= 2.0;
    TS_ASSERT_DELTA(q0(1), 2.0*x, epsilon);
    TS_ASSERT_DELTA(q0(2), 2.0*y, epsilon);
    TS_ASSERT_DELTA(q0(3), 2.0*z, epsilon);
    TS_ASSERT_DELTA(q0(4), 2.0*w, epsilon);
    q0.Normalize();
    TS_ASSERT_DELTA(q0(1), x, epsilon);
    TS_ASSERT_DELTA(q0(2), y, epsilon);
    TS_ASSERT_DELTA(q0(3), z, epsilon);
    TS_ASSERT_DELTA(q0(4), w, epsilon);
  }

  void testOutput() {
    JSBSim::FGQuaternion q;
    std::string s = q.Dump(" , ");
    TS_ASSERT_EQUALS(std::string("1 , 0 , 0 , 0"), s);
    std::ostringstream os;
    os << q;
    TS_ASSERT_EQUALS(std::string("1 , 0 , 0 , 0"), os.str());
  }

  void testMagnitude() {
    // Test magnitude of unit quaternion
    JSBSim::FGQuaternion q0;
    TS_ASSERT_DELTA(1.0, q0.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, q0.SqrMagnitude(), epsilon);

    // Test magnitude of quaternion from Euler angles
    JSBSim::FGQuaternion q1(0.5, 1.0, -0.75);
    double mag = q1.Magnitude();
    TS_ASSERT_DELTA(1.0, mag, epsilon);
    TS_ASSERT_DELTA(1.0, q1.SqrMagnitude(), epsilon);

    // Test magnitude of scaled quaternion
    JSBSim::FGQuaternion q2 = 2.0 * q1;
    TS_ASSERT_DELTA(2.0, q2.Magnitude(), epsilon);
    TS_ASSERT_DELTA(4.0, q2.SqrMagnitude(), epsilon);

    // Test magnitude of arbitrary quaternion
    JSBSim::FGQuaternion q3 = JSBSim::FGQuaternion::zero();
    q3.Entry(1) = 3.0;
    q3.Entry(2) = 4.0;
    q3.Entry(3) = 0.0;
    q3.Entry(4) = 0.0;
    TS_ASSERT_DELTA(5.0, q3.Magnitude(), epsilon);
    TS_ASSERT_DELTA(25.0, q3.SqrMagnitude(), epsilon);

    // Test magnitude of zero quaternion
    JSBSim::FGQuaternion zero = JSBSim::FGQuaternion::zero();
    TS_ASSERT_EQUALS(0.0, zero.Magnitude());
    TS_ASSERT_EQUALS(0.0, zero.SqrMagnitude());
  }

  void testScalarMultiplication() {
    JSBSim::FGQuaternion q0(0.5, 1.0, -0.75);

    // Test non-member scalar multiplication (scalar * quaternion)
    JSBSim::FGQuaternion q1 = 3.0 * q0;
    TS_ASSERT_DELTA(3.0 * q0(1), q1(1), epsilon);
    TS_ASSERT_DELTA(3.0 * q0(2), q1(2), epsilon);
    TS_ASSERT_DELTA(3.0 * q0(3), q1(3), epsilon);
    TS_ASSERT_DELTA(3.0 * q0(4), q1(4), epsilon);

    // Test division using /= operator
    JSBSim::FGQuaternion q2 = q1;
    q2 /= 3.0;
    TS_ASSERT_DELTA(q0(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q0(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q0(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q0(4), q2(4), epsilon);
  }

  void testQuaternionMultiplication() {
    // Test that quaternion multiplication represents successive rotations
    double angle1 = M_PI / 4.0; // 45 degrees
    double angle2 = M_PI / 3.0; // 60 degrees

    // Two rotations around Z axis should add
    JSBSim::FGQuaternion q1(3, angle1);
    JSBSim::FGQuaternion q2(3, angle2);
    JSBSim::FGQuaternion q3 = q2 * q1;
    JSBSim::FGQuaternion q_expected(3, angle1 + angle2);

    TS_ASSERT_DELTA(q_expected(1), q3(1), epsilon);
    TS_ASSERT_DELTA(q_expected(2), q3(2), epsilon);
    TS_ASSERT_DELTA(q_expected(3), q3(3), epsilon);
    TS_ASSERT_DELTA(q_expected(4), q3(4), epsilon);

    // Test non-commutativity: q1 * q2 != q2 * q1 in general
    JSBSim::FGQuaternion qx(1, M_PI / 6.0);
    JSBSim::FGQuaternion qy(2, M_PI / 4.0);
    JSBSim::FGQuaternion qxy = qy * qx;
    JSBSim::FGQuaternion qyx = qx * qy;

    // These should be different
    TS_ASSERT(!(qxy == qyx));

    // Test that unit quaternions produce unit quaternions
    TS_ASSERT_DELTA(1.0, q3.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, qxy.Magnitude(), epsilon);
  }

  void testTransformationMatrixProperties() {
    double angle = M_PI / 6.0; // 30 degrees

    // Test that T and TInv are inverses
    JSBSim::FGQuaternion q(0.3, 0.4, -0.5);
    JSBSim::FGMatrix33 T = q.GetT();
    JSBSim::FGMatrix33 TInv = q.GetTInv();
    JSBSim::FGMatrix33 product = T * TInv;
    TS_ASSERT_MATRIX_IS_IDENTITY(product);

    // Test orthogonality: T^T = T^-1
    JSBSim::FGMatrix33 TT = T.Transposed();
    TS_ASSERT_MATRIX_EQUALS(TT, TInv);

    // Test determinant is 1 (proper rotation)
    double det = T.Determinant();
    TS_ASSERT_DELTA(1.0, det, epsilon);
  }

  void testSpecialAngles() {
    // Test 90 degree rotation around X
    JSBSim::FGQuaternion q90x(1, M_PI / 2.0);
    TS_ASSERT_DELTA(sqrt(2.0) / 2.0, q90x(1), epsilon);
    TS_ASSERT_DELTA(sqrt(2.0) / 2.0, q90x(2), epsilon);
    TS_ASSERT_DELTA(0.0, q90x(3), epsilon);
    TS_ASSERT_DELTA(0.0, q90x(4), epsilon);

    // Test 180 degree rotation around Y
    JSBSim::FGQuaternion q180y(2, M_PI);
    TS_ASSERT_DELTA(0.0, q180y(1), epsilon);
    TS_ASSERT_DELTA(0.0, q180y(2), epsilon);
    TS_ASSERT_DELTA(1.0, q180y(3), epsilon);
    TS_ASSERT_DELTA(0.0, q180y(4), epsilon);

    // Test 360 degree rotation (should be identity)
    JSBSim::FGQuaternion q360(3, 2.0 * M_PI);
    TS_ASSERT_DELTA(1.0, fabs(q360(1)), epsilon);
    TS_ASSERT_DELTA(0.0, q360(2), epsilon);
    TS_ASSERT_DELTA(0.0, q360(3), epsilon);
    TS_ASSERT_DELTA(0.0, fabs(q360(4)), epsilon);
  }

  void testGimbalLock() {
    // Test gimbal lock condition (theta = +/- 90 degrees)
    double phi = 0.3;
    double psi = 0.5;

    // Positive gimbal lock
    JSBSim::FGQuaternion q_pos(phi, M_PI / 2.0, psi);
    JSBSim::FGColumnVector3 euler_pos = q_pos.GetEuler();
    // At gimbal lock, phi and psi are not uniquely defined
    // but theta should be pi/2
    TS_ASSERT_DELTA(M_PI / 2.0, euler_pos(2), 1e-6);

    // Negative gimbal lock
    JSBSim::FGQuaternion q_neg(phi, -M_PI / 2.0, psi);
    JSBSim::FGColumnVector3 euler_neg = q_neg.GetEuler();
    TS_ASSERT_DELTA(-M_PI / 2.0, euler_neg(2), 1e-6);
  }

  void testQuaternionDerivative() {
    // Test GetQDot with various angular velocities
    JSBSim::FGQuaternion q(0.1, 0.2, 0.3);
    JSBSim::FGColumnVector3 omega(1.0, 2.0, 3.0);

    JSBSim::FGQuaternion qDot = q.GetQDot(omega);

    // Verify that GetQDot produces reasonable results
    // The derivative should have similar magnitude to q * |omega|
    double omega_mag = omega.Magnitude();
    double qDot_mag = qDot.Magnitude();
    double q_mag = q.Magnitude();

    // Derivative magnitude should be on order of q_mag * omega_mag / 2
    TS_ASSERT(qDot_mag > 0.0);
    TS_ASSERT(qDot_mag < q_mag * omega_mag);

    // Test with zero angular velocity
    JSBSim::FGColumnVector3 omega_zero(0.0, 0.0, 0.0);
    JSBSim::FGQuaternion qDot_zero = q.GetQDot(omega_zero);
    TS_ASSERT_EQUALS(0.0, qDot_zero(1));
    TS_ASSERT_EQUALS(0.0, qDot_zero(2));
    TS_ASSERT_EQUALS(0.0, qDot_zero(3));
    TS_ASSERT_EQUALS(0.0, qDot_zero(4));

    // Test derivative for unit quaternion with simple rotation
    JSBSim::FGQuaternion q_unit(1, M_PI / 6.0);  // Roll 30 degrees
    JSBSim::FGColumnVector3 omega_x(1.0, 0.0, 0.0);  // Rotating around X
    JSBSim::FGQuaternion qDot_unit = q_unit.GetQDot(omega_x);

    // For a unit quaternion rotating around X, derivative should have non-zero components
    TS_ASSERT(fabs(qDot_unit(1)) > 0.0 || fabs(qDot_unit(2)) > 0.0);
  }

  void testInverseProperties() {
    // Test that q * q^-1 = identity
    JSBSim::FGQuaternion q(0.5, 1.0, -0.75);
    JSBSim::FGQuaternion qInv = q.Inverse();
    JSBSim::FGQuaternion product = q * qInv;

    JSBSim::FGQuaternion identity;
    TS_ASSERT_DELTA(identity(1), product(1), epsilon);
    TS_ASSERT_DELTA(identity(2), product(2), epsilon);
    TS_ASSERT_DELTA(identity(3), product(3), epsilon);
    TS_ASSERT_DELTA(identity(4), product(4), epsilon);

    // Test that q^-1 * q = identity
    product = qInv * q;
    TS_ASSERT_DELTA(identity(1), product(1), epsilon);
    TS_ASSERT_DELTA(identity(2), product(2), epsilon);
    TS_ASSERT_DELTA(identity(3), product(3), epsilon);
    TS_ASSERT_DELTA(identity(4), product(4), epsilon);

    // Test inverse of inverse
    JSBSim::FGQuaternion qInvInv = qInv.Inverse();
    TS_ASSERT_DELTA(q(1), qInvInv(1), epsilon);
    TS_ASSERT_DELTA(q(2), qInvInv(2), epsilon);
    TS_ASSERT_DELTA(q(3), qInvInv(3), epsilon);
    TS_ASSERT_DELTA(q(4), qInvInv(4), epsilon);
  }

  void testConjugateProperties() {
    // For unit quaternions, conjugate equals inverse
    JSBSim::FGQuaternion q(0.5, 1.0, -0.75);
    JSBSim::FGQuaternion qConj = q.Conjugate();
    JSBSim::FGQuaternion qInv = q.Inverse();

    TS_ASSERT_DELTA(qInv(1), qConj(1), epsilon);
    TS_ASSERT_DELTA(qInv(2), qConj(2), epsilon);
    TS_ASSERT_DELTA(qInv(3), qConj(3), epsilon);
    TS_ASSERT_DELTA(qInv(4), qConj(4), epsilon);

    // Test conjugate properties: (q*)* = q
    JSBSim::FGQuaternion qConjConj = qConj.Conjugate();
    TS_ASSERT_DELTA(q(1), qConjConj(1), epsilon);
    TS_ASSERT_DELTA(q(2), qConjConj(2), epsilon);
    TS_ASSERT_DELTA(q(3), qConjConj(3), epsilon);
    TS_ASSERT_DELTA(q(4), qConjConj(4), epsilon);

    // Test (q1 * q2)* = q2* * q1*
    JSBSim::FGQuaternion q1(1, M_PI / 6.0);
    JSBSim::FGQuaternion q2(2, M_PI / 4.0);
    JSBSim::FGQuaternion prod = q1 * q2;
    JSBSim::FGQuaternion prodConj = prod.Conjugate();
    JSBSim::FGQuaternion expected = q2.Conjugate() * q1.Conjugate();

    TS_ASSERT_DELTA(expected(1), prodConj(1), epsilon);
    TS_ASSERT_DELTA(expected(2), prodConj(2), epsilon);
    TS_ASSERT_DELTA(expected(3), prodConj(3), epsilon);
    TS_ASSERT_DELTA(expected(4), prodConj(4), epsilon);
  }

  void testVectorRotation() {
    // Test that quaternion properly rotates vectors using transformation matrix
    double angle = M_PI / 2.0; // 90 degrees

    // Test rotation around Z axis by 90 degrees (should map X to Y)
    // Note: The transformation matrix represents rotation FROM body TO reference frame
    JSBSim::FGQuaternion qz(3, angle);
    JSBSim::FGMatrix33 Tz = qz.GetT();
    JSBSim::FGColumnVector3 x_axis(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 rotated = Tz * x_axis;

    // Verify the rotated vector has magnitude 1
    TS_ASSERT_DELTA(1.0, rotated.Magnitude(), epsilon);

    // Test that rotation matrices are orthogonal and preserve lengths
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGMatrix33 T = q.GetT();
    JSBSim::FGColumnVector3 v(1.5, -2.3, 0.7);
    JSBSim::FGColumnVector3 v_rotated = T * v;

    // Rotation preserves length
    TS_ASSERT_DELTA(v.Magnitude(), v_rotated.Magnitude(), epsilon);

    // Test that inverse transformation returns original vector
    JSBSim::FGMatrix33 TInv = q.GetTInv();
    JSBSim::FGColumnVector3 v_back = TInv * v_rotated;
    TS_ASSERT_DELTA(v(1), v_back(1), epsilon);
    TS_ASSERT_DELTA(v(2), v_back(2), epsilon);
    TS_ASSERT_DELTA(v(3), v_back(3), epsilon);
  }

  void testEulerAngleConsistency() {
    // Test that converting to Euler and back gives the same quaternion
    double phi = 0.3;
    double tht = 0.4;
    double psi = 0.5;

    JSBSim::FGQuaternion q1(phi, tht, psi);
    JSBSim::FGColumnVector3 euler = q1.GetEuler();
    JSBSim::FGQuaternion q2(euler);

    // Quaternions might differ by sign (represent same rotation)
    bool same = (fabs(q1(1) - q2(1)) < epsilon &&
                 fabs(q1(2) - q2(2)) < epsilon &&
                 fabs(q1(3) - q2(3)) < epsilon &&
                 fabs(q1(4) - q2(4)) < epsilon);
    bool opposite = (fabs(q1(1) + q2(1)) < epsilon &&
                     fabs(q1(2) + q2(2)) < epsilon &&
                     fabs(q1(3) + q2(3)) < epsilon &&
                     fabs(q1(4) + q2(4)) < epsilon);

    TS_ASSERT(same || opposite);

    // The transformation matrices should be identical
    TS_ASSERT_MATRIX_EQUALS(q1.GetT(), q2.GetT());
  }

  void testQExpProperties() {
    // Test QExp with zero vector gives identity
    JSBSim::FGColumnVector3 zero(0.0, 0.0, 0.0);
    JSBSim::FGQuaternion q = QExp(zero);
    TS_ASSERT_DELTA(1.0, q(1), epsilon);
    TS_ASSERT_DELTA(0.0, q(2), epsilon);
    TS_ASSERT_DELTA(0.0, q(3), epsilon);
    TS_ASSERT_DELTA(0.0, q(4), epsilon);

    // Test QExp with small rotation
    // For small angle theta, QExp(omega) where |omega| = theta
    // gives quaternion q = (cos(theta), sin(theta)/theta * omega)
    // For very small theta: q ≈ (1, omega/2)
    JSBSim::FGColumnVector3 small(0.001, 0.002, 0.003);
    q = QExp(small);

    TS_ASSERT_DELTA(1.0, q(1), 0.01);
    TS_ASSERT_DELTA(0.5 * small(1), q(2), 0.01);
    TS_ASSERT_DELTA(0.5 * small(2), q(3), 0.01);
    TS_ASSERT_DELTA(0.5 * small(3), q(4), 0.01);

    // Test that QExp produces unit quaternions
    JSBSim::FGColumnVector3 large(1.0, 2.0, -0.5);
    q = QExp(large);
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);
  }

  void testSinCosEulerAngles() {
    // Test consistency between GetSinEuler/GetCosEuler and GetEuler
    double phi = M_PI / 6.0;   // 30 degrees
    double tht = M_PI / 4.0;   // 45 degrees
    double psi = M_PI / 3.0;   // 60 degrees

    JSBSim::FGQuaternion q(phi, tht, psi);

    // Get Euler angles
    JSBSim::FGColumnVector3 euler = q.GetEuler();

    // Compare sin/cos values
    for (int i = 1; i <= 3; i++) {
      TS_ASSERT_DELTA(sin(euler(i)), q.GetSinEuler(i), epsilon);
      TS_ASSERT_DELTA(cos(euler(i)), q.GetCosEuler(i), epsilon);
    }

    // Verify sin^2 + cos^2 = 1
    for (int i = 1; i <= 3; i++) {
      double sinE = q.GetSinEuler(i);
      double cosE = q.GetCosEuler(i);
      TS_ASSERT_DELTA(1.0, sinE * sinE + cosE * cosE, epsilon);
    }
  }

  void testComplexRotationSequences() {
    // Test a complex sequence of rotations
    JSBSim::FGQuaternion q1(1, M_PI / 6.0);  // Roll 30°
    JSBSim::FGQuaternion q2(2, M_PI / 4.0);  // Pitch 45°
    JSBSim::FGQuaternion q3(3, M_PI / 3.0);  // Yaw 60°

    // Combined rotation
    JSBSim::FGQuaternion qTotal = q3 * q2 * q1;

    // Should still be unit quaternion
    TS_ASSERT_DELTA(1.0, qTotal.Magnitude(), epsilon);

    // Test that the transformation matrix is orthogonal
    JSBSim::FGMatrix33 T = qTotal.GetT();
    TS_ASSERT_DELTA(1.0, T.Determinant(), epsilon);

    // Test inverse rotation sequence
    JSBSim::FGQuaternion qInv = qTotal.Inverse();
    JSBSim::FGQuaternion qInv_decomposed = q1.Inverse() * q2.Inverse() * q3.Inverse();

    TS_ASSERT_DELTA(qInv(1), qInv_decomposed(1), epsilon);
    TS_ASSERT_DELTA(qInv(2), qInv_decomposed(2), epsilon);
    TS_ASSERT_DELTA(qInv(3), qInv_decomposed(3), epsilon);
    TS_ASSERT_DELTA(qInv(4), qInv_decomposed(4), epsilon);
  }

  void testEulerDegreeConversions() {
    // Test conversion between radians and degrees
    double phi_deg = 30.0;
    double tht_deg = 45.0;
    double psi_deg = 60.0;

    double phi_rad = phi_deg * M_PI / 180.0;
    double tht_rad = tht_deg * M_PI / 180.0;
    double psi_rad = psi_deg * M_PI / 180.0;

    JSBSim::FGQuaternion q(phi_rad, tht_rad, psi_rad);

    // Test individual angle access in degrees
    TS_ASSERT_DELTA(phi_deg, q.GetEulerDeg(1), epsilon);
    TS_ASSERT_DELTA(tht_deg, q.GetEulerDeg(2), epsilon);
    TS_ASSERT_DELTA(psi_deg, q.GetEulerDeg(3), epsilon);

    // Test vector access in degrees
    JSBSim::FGColumnVector3 euler_deg = q.GetEulerDeg();
    TS_ASSERT_DELTA(phi_deg, euler_deg(1), epsilon);
    TS_ASSERT_DELTA(tht_deg, euler_deg(2), epsilon);
    TS_ASSERT_DELTA(psi_deg, euler_deg(3), epsilon);
  }

  void testMatrixConstructorInverse() {
    // Test that constructing from a matrix and getting it back works
    double ca = cos(M_PI / 6.0);
    double sa = sin(M_PI / 6.0);

    // Create rotation matrix for 30° around X
    JSBSim::FGMatrix33 m_orig(1.0, 0.0, 0.0,
                               0.0, ca, sa,
                               0.0, -sa, ca);

    JSBSim::FGQuaternion q(m_orig);
    JSBSim::FGMatrix33 m_recovered = q.GetT();

    TS_ASSERT_MATRIX_EQUALS(m_orig, m_recovered);
  }

  void testNormalizePreservesOrientation() {
    // Test that normalizing preserves the orientation
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGMatrix33 T_before = q.GetT();

    // Scale and normalize
    q *= 2.5;
    q.Normalize();

    JSBSim::FGMatrix33 T_after = q.GetT();
    TS_ASSERT_MATRIX_EQUALS(T_before, T_after);

    // Check magnitude is 1
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);
  }

  void testArithmeticOperatorsCombined() {
    // Test combinations of arithmetic operations
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);

    // Test (q1 + q2) - q2 == q1
    JSBSim::FGQuaternion result = (q1 + q2) - q2;
    TS_ASSERT_DELTA(q1(1), result(1), epsilon);
    TS_ASSERT_DELTA(q1(2), result(2), epsilon);
    TS_ASSERT_DELTA(q1(3), result(3), epsilon);
    TS_ASSERT_DELTA(q1(4), result(4), epsilon);

    // Test scalar operations: (q * 2) / 2 == q
    result = 2.0 * q1;
    result /= 2.0;
    TS_ASSERT_DELTA(q1(1), result(1), epsilon);
    TS_ASSERT_DELTA(q1(2), result(2), epsilon);
    TS_ASSERT_DELTA(q1(3), result(3), epsilon);
    TS_ASSERT_DELTA(q1(4), result(4), epsilon);
  }

  // ============================================================================
  // Additional Edge Case and Validation Tests
  // ============================================================================

  void testDoubleCoverProperty() {
    // Test that q and -q represent the same rotation
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGQuaternion neg_q = -1.0 * q;

    // Both should produce identical transformation matrices
    JSBSim::FGMatrix33 T1 = q.GetT();
    JSBSim::FGMatrix33 T2 = neg_q.GetT();

    TS_ASSERT_MATRIX_EQUALS(T1, T2);

    // And identical Euler angles (or differ by 2*pi)
    JSBSim::FGColumnVector3 e1 = q.GetEuler();
    JSBSim::FGColumnVector3 e2 = neg_q.GetEuler();

    for (int i = 1; i <= 3; i++) {
      double diff = fabs(e1(i) - e2(i));
      // Either same or differ by 2*pi
      TS_ASSERT(diff < epsilon || fabs(diff - 2.0 * M_PI) < epsilon);
    }
  }

  void testSmallAngleApproximation() {
    // For small angles, sin(theta/2) ≈ theta/2, cos(theta/2) ≈ 1
    double small_angle = 0.001;  // radians

    JSBSim::FGQuaternion q(1, small_angle);  // Roll by small angle

    // q ≈ (1, small_angle/2, 0, 0)
    TS_ASSERT_DELTA(1.0, q(1), small_angle);
    TS_ASSERT_DELTA(small_angle / 2.0, q(2), small_angle * small_angle);
    TS_ASSERT_DELTA(0.0, q(3), epsilon);
    TS_ASSERT_DELTA(0.0, q(4), epsilon);
  }

  void testQuaternionFromArbitraryAxis() {
    // Test rotation around arbitrary axis
    JSBSim::FGColumnVector3 axis(1.0, 1.0, 1.0);  // Diagonal axis
    double angle = M_PI / 4.0;  // 45 degrees

    JSBSim::FGQuaternion q(angle, axis);

    // Should be unit quaternion
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

    // Verify the axis direction is preserved
    axis.Normalize();
    double half_angle = angle / 2.0;
    TS_ASSERT_DELTA(cos(half_angle), q(1), epsilon);
    TS_ASSERT_DELTA(sin(half_angle) * axis(1), q(2), epsilon);
    TS_ASSERT_DELTA(sin(half_angle) * axis(2), q(3), epsilon);
    TS_ASSERT_DELTA(sin(half_angle) * axis(3), q(4), epsilon);
  }

  void testIdentityQuaternionProperties() {
    // Test properties of identity quaternion
    JSBSim::FGQuaternion identity;

    // Should be (1, 0, 0, 0)
    TS_ASSERT_DELTA(1.0, identity(1), epsilon);
    TS_ASSERT_DELTA(0.0, identity(2), epsilon);
    TS_ASSERT_DELTA(0.0, identity(3), epsilon);
    TS_ASSERT_DELTA(0.0, identity(4), epsilon);

    // Identity * q = q
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGQuaternion product = identity * q;

    TS_ASSERT_DELTA(q(1), product(1), epsilon);
    TS_ASSERT_DELTA(q(2), product(2), epsilon);
    TS_ASSERT_DELTA(q(3), product(3), epsilon);
    TS_ASSERT_DELTA(q(4), product(4), epsilon);

    // q * identity = q
    product = q * identity;
    TS_ASSERT_DELTA(q(1), product(1), epsilon);
    TS_ASSERT_DELTA(q(2), product(2), epsilon);
    TS_ASSERT_DELTA(q(3), product(3), epsilon);
    TS_ASSERT_DELTA(q(4), product(4), epsilon);

    // Identity transformation matrix is identity
    JSBSim::FGMatrix33 T = identity.GetT();
    TS_ASSERT_MATRIX_IS_IDENTITY(T);

    // Identity Euler angles are zero
    JSBSim::FGColumnVector3 euler = identity.GetEuler();
    TS_ASSERT_DELTA(0.0, euler(1), epsilon);
    TS_ASSERT_DELTA(0.0, euler(2), epsilon);
    TS_ASSERT_DELTA(0.0, euler(3), epsilon);
  }

  void test180DegreeRotations() {
    // Test 180 degree rotations around each axis
    double angle = M_PI;

    // 180° around X
    JSBSim::FGQuaternion qx(1, angle);
    JSBSim::FGMatrix33 Tx = qx.GetT();
    // Should negate Y and Z
    TS_ASSERT_DELTA(1.0, Tx(1, 1), epsilon);
    TS_ASSERT_DELTA(-1.0, Tx(2, 2), epsilon);
    TS_ASSERT_DELTA(-1.0, Tx(3, 3), epsilon);

    // 180° around Y
    JSBSim::FGQuaternion qy(2, angle);
    JSBSim::FGMatrix33 Ty = qy.GetT();
    // Should negate X and Z
    TS_ASSERT_DELTA(-1.0, Ty(1, 1), epsilon);
    TS_ASSERT_DELTA(1.0, Ty(2, 2), epsilon);
    TS_ASSERT_DELTA(-1.0, Ty(3, 3), epsilon);

    // 180° around Z
    JSBSim::FGQuaternion qz(3, angle);
    JSBSim::FGMatrix33 Tz = qz.GetT();
    // Should negate X and Y
    TS_ASSERT_DELTA(-1.0, Tz(1, 1), epsilon);
    TS_ASSERT_DELTA(-1.0, Tz(2, 2), epsilon);
    TS_ASSERT_DELTA(1.0, Tz(3, 3), epsilon);
  }

  void testNegativeAngles() {
    // Test negative rotation angles
    double angle = -M_PI / 4.0;  // -45 degrees

    JSBSim::FGQuaternion q_neg(1, angle);
    JSBSim::FGQuaternion q_pos(1, -angle);

    // Should be conjugates
    JSBSim::FGQuaternion q_neg_conj = q_neg.Conjugate();
    TS_ASSERT_DELTA(q_pos(1), q_neg_conj(1), epsilon);
    TS_ASSERT_DELTA(q_pos(2), q_neg_conj(2), epsilon);
    TS_ASSERT_DELTA(q_pos(3), q_neg_conj(3), epsilon);
    TS_ASSERT_DELTA(q_pos(4), q_neg_conj(4), epsilon);
  }

  void testQuaternionNorm() {
    // Test that quaternion norm equals magnitude squared
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);

    double sqrMag = q.SqrMagnitude();
    double mag = q.Magnitude();

    TS_ASSERT_DELTA(sqrMag, mag * mag, epsilon);

    // For unit quaternion, norm should be 1
    TS_ASSERT_DELTA(1.0, sqrMag, epsilon);
  }

  void testSuccessiveRotationsEquivalence() {
    // Test that successive rotations match combined Euler angles
    double phi = 0.1;
    double tht = 0.2;
    double psi = 0.3;

    // Method 1: Single quaternion from Euler angles
    JSBSim::FGQuaternion q_euler(phi, tht, psi);

    // Method 2: Successive rotations (Euler sequence: ZYX)
    JSBSim::FGQuaternion qx(1, phi);
    JSBSim::FGQuaternion qy(2, tht);
    JSBSim::FGQuaternion qz(3, psi);
    JSBSim::FGQuaternion q_successive = qz * qy * qx;

    // Should be equivalent
    TS_ASSERT_DELTA(q_euler(1), q_successive(1), epsilon);
    TS_ASSERT_DELTA(q_euler(2), q_successive(2), epsilon);
    TS_ASSERT_DELTA(q_euler(3), q_successive(3), epsilon);
    TS_ASSERT_DELTA(q_euler(4), q_successive(4), epsilon);
  }

  void testMatrixOrthogonality() {
    // Test that transformation matrix is orthogonal for various quaternions
    std::vector<std::tuple<double, double, double>> angles = {
      {0.0, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {0.0, 0.5, 0.0},
      {0.0, 0.0, 0.5},
      {0.3, 0.4, 0.5},
      {M_PI / 2, 0.0, 0.0},
      {0.0, M_PI / 2, 0.0}
    };

    for (const auto& [phi, tht, psi] : angles) {
      JSBSim::FGQuaternion q(phi, tht, psi);
      JSBSim::FGMatrix33 T = q.GetT();
      JSBSim::FGMatrix33 TT = T.Transposed();
      JSBSim::FGMatrix33 product = T * TT;

      TS_ASSERT_MATRIX_IS_IDENTITY(product);
    }
  }

  void testLargeAngles() {
    // Test with angles larger than 2*pi
    double large_angle = 5.0 * M_PI;  // 900 degrees

    JSBSim::FGQuaternion q(1, large_angle);

    // Should still be unit quaternion
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

    // Equivalent to large_angle mod 2*pi
    double equiv_angle = fmod(large_angle, 2.0 * M_PI);
    JSBSim::FGQuaternion q_equiv(1, equiv_angle);

    // Transformation matrices should be equal
    TS_ASSERT_MATRIX_EQUALS(q.GetT(), q_equiv.GetT());
  }

  void testZeroVectorRotation() {
    // Rotating a zero vector should give zero vector
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGMatrix33 T = q.GetT();

    JSBSim::FGColumnVector3 zero(0.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 result = T * zero;

    TS_ASSERT_DELTA(0.0, result(1), epsilon);
    TS_ASSERT_DELTA(0.0, result(2), epsilon);
    TS_ASSERT_DELTA(0.0, result(3), epsilon);
  }

  void testUnitVectorRotationPreservesLength() {
    // Rotating unit vectors should preserve unit length
    JSBSim::FGQuaternion q(0.5, 0.6, 0.7);
    JSBSim::FGMatrix33 T = q.GetT();

    JSBSim::FGColumnVector3 x_unit(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 y_unit(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 z_unit(0.0, 0.0, 1.0);

    JSBSim::FGColumnVector3 x_rot = T * x_unit;
    JSBSim::FGColumnVector3 y_rot = T * y_unit;
    JSBSim::FGColumnVector3 z_rot = T * z_unit;

    TS_ASSERT_DELTA(1.0, x_rot.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, y_rot.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, z_rot.Magnitude(), epsilon);
  }

  void testRotatedVectorsOrthogonality() {
    // Rotating orthogonal vectors should preserve orthogonality
    JSBSim::FGQuaternion q(0.4, 0.5, 0.6);
    JSBSim::FGMatrix33 T = q.GetT();

    JSBSim::FGColumnVector3 x_unit(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 y_unit(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 z_unit(0.0, 0.0, 1.0);

    JSBSim::FGColumnVector3 x_rot = T * x_unit;
    JSBSim::FGColumnVector3 y_rot = T * y_unit;
    JSBSim::FGColumnVector3 z_rot = T * z_unit;

    // Dot products should be zero (orthogonal)
    double xy_dot = x_rot(1) * y_rot(1) + x_rot(2) * y_rot(2) + x_rot(3) * y_rot(3);
    double xz_dot = x_rot(1) * z_rot(1) + x_rot(2) * z_rot(2) + x_rot(3) * z_rot(3);
    double yz_dot = y_rot(1) * z_rot(1) + y_rot(2) * z_rot(2) + y_rot(3) * z_rot(3);

    TS_ASSERT_DELTA(0.0, xy_dot, epsilon);
    TS_ASSERT_DELTA(0.0, xz_dot, epsilon);
    TS_ASSERT_DELTA(0.0, yz_dot, epsilon);
  }

  void testMultiplicationAssociativity() {
    // Test that quaternion multiplication is associative: (q1*q2)*q3 = q1*(q2*q3)
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);
    JSBSim::FGQuaternion q3(0.7, 0.8, 0.9);

    JSBSim::FGQuaternion left = (q1 * q2) * q3;
    JSBSim::FGQuaternion right = q1 * (q2 * q3);

    TS_ASSERT_DELTA(left(1), right(1), epsilon);
    TS_ASSERT_DELTA(left(2), right(2), epsilon);
    TS_ASSERT_DELTA(left(3), right(3), epsilon);
    TS_ASSERT_DELTA(left(4), right(4), epsilon);
  }

  void testMultiplicationDistributivity() {
    // Test (q1 + q2) * q3 vs q1*q3 + q2*q3
    // Note: quaternion addition isn't geometrically meaningful, but algebraically valid
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);
    JSBSim::FGQuaternion q3(0.7, 0.8, 0.9);

    JSBSim::FGQuaternion left = (q1 + q2) * q3;
    JSBSim::FGQuaternion right = q1 * q3 + q2 * q3;

    TS_ASSERT_DELTA(left(1), right(1), epsilon);
    TS_ASSERT_DELTA(left(2), right(2), epsilon);
    TS_ASSERT_DELTA(left(3), right(3), epsilon);
    TS_ASSERT_DELTA(left(4), right(4), epsilon);
  }

  void testQDotIntegration() {
    // Test that integrating QDot gives expected rotation
    JSBSim::FGQuaternion q;  // Identity
    JSBSim::FGColumnVector3 omega(0.1, 0.0, 0.0);  // Small roll rate
    double dt = 0.001;

    // Simple Euler integration
    for (int i = 0; i < 100; i++) {
      JSBSim::FGQuaternion qDot = q.GetQDot(omega);
      q = q + dt * qDot;  // scalar must come first
      q.Normalize();
    }

    // After 100 steps of 0.001s = 0.1s at 0.1 rad/s, should have rotated 0.01 rad
    double expected_angle = 0.1 * 0.1;  // omega * total_time
    JSBSim::FGColumnVector3 euler = q.GetEuler();

    TS_ASSERT_DELTA(expected_angle, euler(1), 0.001);
    TS_ASSERT_DELTA(0.0, euler(2), epsilon);
    TS_ASSERT_DELTA(0.0, euler(3), epsilon);
  }

  void testEntryAccessModification() {
    // Test Entry() access for reading and writing
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    double original1 = q.Entry(1);
    double original2 = q.Entry(2);
    double original3 = q.Entry(3);
    double original4 = q.Entry(4);

    // Modify using Entry()
    q.Entry(1) = 0.9;
    q.Entry(2) = 0.1;
    q.Entry(3) = 0.2;
    q.Entry(4) = 0.3;

    TS_ASSERT_DELTA(0.9, q.Entry(1), epsilon);
    TS_ASSERT_DELTA(0.1, q.Entry(2), epsilon);
    TS_ASSERT_DELTA(0.2, q.Entry(3), epsilon);
    TS_ASSERT_DELTA(0.3, q.Entry(4), epsilon);

    // Verify operator() gives same values
    TS_ASSERT_DELTA(q(1), q.Entry(1), epsilon);
    TS_ASSERT_DELTA(q(2), q.Entry(2), epsilon);
    TS_ASSERT_DELTA(q(3), q.Entry(3), epsilon);
    TS_ASSERT_DELTA(q(4), q.Entry(4), epsilon);
  }

  void testNearGimbalLockStability() {
    // Test numerical stability near gimbal lock (theta ≈ ±90°)
    double near_90 = M_PI / 2.0 - 0.001;

    JSBSim::FGQuaternion q(0.1, near_90, 0.2);

    // Should still produce valid Euler angles
    JSBSim::FGColumnVector3 euler = q.GetEuler();
    TS_ASSERT(!std::isnan(euler(1)));
    TS_ASSERT(!std::isnan(euler(2)));
    TS_ASSERT(!std::isnan(euler(3)));
    TS_ASSERT(!std::isinf(euler(1)));
    TS_ASSERT(!std::isinf(euler(2)));
    TS_ASSERT(!std::isinf(euler(3)));

    // Theta should be close to 90 degrees
    TS_ASSERT_DELTA(near_90, euler(2), 0.01);
  }

  void testInverseMultiplicationOrder() {
    // Test (q1 * q2)^-1 = q2^-1 * q1^-1
    JSBSim::FGQuaternion q1(0.2, 0.3, 0.4);
    JSBSim::FGQuaternion q2(0.5, 0.6, 0.7);

    JSBSim::FGQuaternion product = q1 * q2;
    JSBSim::FGQuaternion productInv = product.Inverse();
    JSBSim::FGQuaternion expected = q2.Inverse() * q1.Inverse();

    TS_ASSERT_DELTA(expected(1), productInv(1), epsilon);
    TS_ASSERT_DELTA(expected(2), productInv(2), epsilon);
    TS_ASSERT_DELTA(expected(3), productInv(3), epsilon);
    TS_ASSERT_DELTA(expected(4), productInv(4), epsilon);
  }

  void testScaledQuaternionInverse() {
    // Test inverse of non-unit quaternion
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGQuaternion q_scaled = 2.0 * q;  // scalar must come first

    JSBSim::FGQuaternion q_scaled_inv = q_scaled.Inverse();
    JSBSim::FGQuaternion product = q_scaled * q_scaled_inv;

    // Should give identity
    JSBSim::FGQuaternion identity;
    TS_ASSERT_DELTA(identity(1), product(1), epsilon);
    TS_ASSERT_DELTA(identity(2), product(2), epsilon);
    TS_ASSERT_DELTA(identity(3), product(3), epsilon);
    TS_ASSERT_DELTA(identity(4), product(4), epsilon);
  }

  // ============================================================================
  // Additional Tests for Extended Coverage
  // ============================================================================

  void testPureRotationAngles() {
    // Test pure rotation angles (single axis only)
    for (int axis = 1; axis <= 3; axis++) {
      for (double angle = -M_PI; angle <= M_PI; angle += M_PI / 6.0) {
        JSBSim::FGQuaternion q(axis, angle);

        // Should be unit quaternion
        TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

        // Components should follow quaternion formula
        double half = angle / 2.0;
        TS_ASSERT_DELTA(cos(half), q(1), epsilon);

        // Only one axis component should be non-zero
        for (int i = 2; i <= 4; i++) {
          if (i == axis + 1) {
            TS_ASSERT_DELTA(sin(half), q(i), epsilon);
          } else {
            TS_ASSERT_DELTA(0.0, q(i), epsilon);
          }
        }
      }
    }
  }

  void testEulerAngleBoundaries() {
    // Test Euler angle boundaries
    // Phi at ±180°
    JSBSim::FGQuaternion q1(M_PI, 0.0, 0.0);
    JSBSim::FGColumnVector3 e1 = q1.GetEuler();
    TS_ASSERT_DELTA(fabs(e1(1)), M_PI, epsilon);

    // Psi at ±180°
    JSBSim::FGQuaternion q2(0.0, 0.0, M_PI);
    JSBSim::FGColumnVector3 e2 = q2.GetEuler();
    TS_ASSERT_DELTA(fabs(e2(3)), M_PI, epsilon);

    // Theta at limits
    JSBSim::FGQuaternion q3(0.0, M_PI / 2.0 - 0.01, 0.0);
    JSBSim::FGColumnVector3 e3 = q3.GetEuler();
    TS_ASSERT_DELTA(M_PI / 2.0 - 0.01, e3(2), 0.1);
  }

  void testQuaternionSquare() {
    // Test q * q = double rotation
    double angle = M_PI / 6.0;  // 30 degrees
    JSBSim::FGQuaternion q(1, angle);
    JSBSim::FGQuaternion q_squared = q * q;

    // Should equal rotation by 2*angle
    JSBSim::FGQuaternion q_double(1, 2.0 * angle);

    TS_ASSERT_DELTA(q_double(1), q_squared(1), epsilon);
    TS_ASSERT_DELTA(q_double(2), q_squared(2), epsilon);
    TS_ASSERT_DELTA(q_double(3), q_squared(3), epsilon);
    TS_ASSERT_DELTA(q_double(4), q_squared(4), epsilon);
  }

  void testHalfAngleRotation() {
    // Creating quaternion for angle theta means half-angle is theta/2
    double angle = M_PI / 3.0;  // 60 degrees

    JSBSim::FGQuaternion q(2, angle);

    // Scalar part should be cos(theta/2)
    TS_ASSERT_DELTA(cos(angle / 2.0), q(1), epsilon);

    // Y component should be sin(theta/2)
    TS_ASSERT_DELTA(sin(angle / 2.0), q(3), epsilon);

    // Other components zero
    TS_ASSERT_DELTA(0.0, q(2), epsilon);
    TS_ASSERT_DELTA(0.0, q(4), epsilon);
  }

  void testMatrixToQuaternionRoundTrip() {
    // Test matrix->quaternion->matrix round trips
    // Avoid angles near gimbal lock (theta ≈ ±90°)
    double angles[] = {0.0, 0.1, 0.3, 0.5, -0.1, -0.3, -0.5};

    for (double phi : angles) {
      for (double tht : angles) {
        for (double psi : angles) {
          JSBSim::FGQuaternion q1(phi, tht, psi);
          JSBSim::FGMatrix33 m = q1.GetT();
          JSBSim::FGQuaternion q2(m);
          JSBSim::FGMatrix33 m2 = q2.GetT();

          // Matrices should match
          TS_ASSERT_MATRIX_EQUALS(m, m2);
        }
      }
    }
  }

  void testQuaternionFromIdentityMatrix() {
    // Identity matrix should give identity quaternion
    JSBSim::FGMatrix33 I;
    I.InitMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    JSBSim::FGQuaternion q(I);

    TS_ASSERT_DELTA(1.0, q(1), epsilon);
    TS_ASSERT_DELTA(0.0, q(2), epsilon);
    TS_ASSERT_DELTA(0.0, q(3), epsilon);
    TS_ASSERT_DELTA(0.0, q(4), epsilon);
  }

  void testVerySmallRotations() {
    // Test very small rotation angles
    double tiny = 1e-10;

    JSBSim::FGQuaternion q(1, tiny);

    // For tiny angle: cos(tiny/2) ≈ 1, sin(tiny/2) ≈ tiny/2
    TS_ASSERT_DELTA(1.0, q(1), 1e-15);
    TS_ASSERT_DELTA(tiny / 2.0, q(2), 1e-15);

    // Euler angles should recover the small rotation
    JSBSim::FGColumnVector3 euler = q.GetEuler();
    TS_ASSERT_DELTA(tiny, euler(1), 1e-10);
  }

  void testQuaternionAdditionCommutative() {
    // Quaternion addition should be commutative
    JSBSim::FGQuaternion q1(0.2, 0.3, 0.4);
    JSBSim::FGQuaternion q2(0.5, 0.6, 0.7);

    JSBSim::FGQuaternion sum1 = q1 + q2;
    JSBSim::FGQuaternion sum2 = q2 + q1;

    TS_ASSERT_DELTA(sum1(1), sum2(1), epsilon);
    TS_ASSERT_DELTA(sum1(2), sum2(2), epsilon);
    TS_ASSERT_DELTA(sum1(3), sum2(3), epsilon);
    TS_ASSERT_DELTA(sum1(4), sum2(4), epsilon);
  }

  void testQuaternionSubtractionProperty() {
    // q - q should give zero quaternion
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGQuaternion diff = q - q;

    TS_ASSERT_DELTA(0.0, diff(1), epsilon);
    TS_ASSERT_DELTA(0.0, diff(2), epsilon);
    TS_ASSERT_DELTA(0.0, diff(3), epsilon);
    TS_ASSERT_DELTA(0.0, diff(4), epsilon);
  }

  void testRotationBy270Degrees() {
    // 270° rotation = -90° rotation (in opposite direction)
    double angle_270 = 3.0 * M_PI / 2.0;
    double angle_neg90 = -M_PI / 2.0;

    JSBSim::FGQuaternion q270(1, angle_270);
    JSBSim::FGQuaternion qneg90(1, angle_neg90);

    // Transformation matrices should be identical
    TS_ASSERT_MATRIX_EQUALS(q270.GetT(), qneg90.GetT());
  }

  void testCombinedAxisRotations() {
    // Test combined rotations around multiple axes
    double angle = M_PI / 4.0;

    JSBSim::FGQuaternion qx(1, angle);
    JSBSim::FGQuaternion qy(2, angle);
    JSBSim::FGQuaternion qz(3, angle);

    // qx * qy * qz should be unit quaternion
    JSBSim::FGQuaternion combined = qz * qy * qx;
    TS_ASSERT_DELTA(1.0, combined.Magnitude(), epsilon);

    // Should produce orthogonal transformation
    JSBSim::FGMatrix33 T = combined.GetT();
    TS_ASSERT_DELTA(1.0, T.Determinant(), epsilon);
  }

  void testRotationPreservesHandedness() {
    // Proper rotations should preserve handedness (det = +1)
    JSBSim::FGQuaternion q1(0.3, 0.4, 0.5);
    JSBSim::FGQuaternion q2(0.6, 0.7, 0.8);
    JSBSim::FGQuaternion q3 = q1 * q2;

    TS_ASSERT_DELTA(1.0, q1.GetT().Determinant(), epsilon);
    TS_ASSERT_DELTA(1.0, q2.GetT().Determinant(), epsilon);
    TS_ASSERT_DELTA(1.0, q3.GetT().Determinant(), epsilon);
  }

  void testQDotWithUnitQuaternion() {
    // QDot for unit quaternion with specific omega
    JSBSim::FGQuaternion q;  // Identity
    JSBSim::FGColumnVector3 omega(1.0, 2.0, 3.0);

    JSBSim::FGQuaternion qDot = q.GetQDot(omega);

    // For identity q, qDot = 0.5 * omega * q
    // qDot(1) = -0.5 * (omega_x*q_x + omega_y*q_y + omega_z*q_z) = 0 (since q_xyz = 0)
    // qDot(2) = 0.5 * (omega_x*q_w) = 0.5 * omega_x
    // qDot(3) = 0.5 * omega_y
    // qDot(4) = 0.5 * omega_z
    TS_ASSERT_DELTA(0.0, qDot(1), epsilon);
    TS_ASSERT_DELTA(0.5 * omega(1), qDot(2), epsilon);
    TS_ASSERT_DELTA(0.5 * omega(2), qDot(3), epsilon);
    TS_ASSERT_DELTA(0.5 * omega(3), qDot(4), epsilon);
  }

  void testConjugateOfProductReversed() {
    // (q1 * q2)* = q2* * q1*
    JSBSim::FGQuaternion q1(0.2, 0.3, 0.4);
    JSBSim::FGQuaternion q2(0.5, 0.6, 0.7);

    JSBSim::FGQuaternion prod = q1 * q2;
    JSBSim::FGQuaternion prodConj = prod.Conjugate();

    JSBSim::FGQuaternion expected = q2.Conjugate() * q1.Conjugate();

    TS_ASSERT_DELTA(expected(1), prodConj(1), epsilon);
    TS_ASSERT_DELTA(expected(2), prodConj(2), epsilon);
    TS_ASSERT_DELTA(expected(3), prodConj(3), epsilon);
    TS_ASSERT_DELTA(expected(4), prodConj(4), epsilon);
  }

  void testEulerSequence321() {
    // Verify the Euler sequence is ZYX (3-2-1)
    double phi = 0.1;   // Roll (X)
    double tht = 0.2;   // Pitch (Y)
    double psi = 0.3;   // Yaw (Z)

    // From Euler angles
    JSBSim::FGQuaternion q_euler(phi, tht, psi);

    // From successive rotations: Z then Y then X
    JSBSim::FGQuaternion qx(1, phi);
    JSBSim::FGQuaternion qy(2, tht);
    JSBSim::FGQuaternion qz(3, psi);
    JSBSim::FGQuaternion q_successive = qz * qy * qx;

    TS_ASSERT_DELTA(q_euler(1), q_successive(1), epsilon);
    TS_ASSERT_DELTA(q_euler(2), q_successive(2), epsilon);
    TS_ASSERT_DELTA(q_euler(3), q_successive(3), epsilon);
    TS_ASSERT_DELTA(q_euler(4), q_successive(4), epsilon);
  }

  void testUnitQuaternionMagnitudeAfterOperations() {
    // Unit quaternions multiplied together stay unit
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);

    // Both should be unit
    TS_ASSERT_DELTA(1.0, q1.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, q2.Magnitude(), epsilon);

    // Product should be unit
    JSBSim::FGQuaternion prod = q1 * q2;
    TS_ASSERT_DELTA(1.0, prod.Magnitude(), epsilon);

    // Inverse should be unit
    JSBSim::FGQuaternion inv = q1.Inverse();
    TS_ASSERT_DELTA(1.0, inv.Magnitude(), epsilon);

    // Conjugate should be unit
    JSBSim::FGQuaternion conj = q1.Conjugate();
    TS_ASSERT_DELTA(1.0, conj.Magnitude(), epsilon);
  }

  void testAxisAngleExtraction() {
    // Test extracting rotation axis and angle from quaternion
    double angle = M_PI / 5.0;
    JSBSim::FGColumnVector3 axis(1.0, 2.0, 3.0);
    axis.Normalize();

    JSBSim::FGQuaternion q(angle, axis);

    // Recover angle from scalar part
    double recovered_half = acos(q(1));
    double recovered_angle = 2.0 * recovered_half;
    TS_ASSERT_DELTA(angle, recovered_angle, epsilon);

    // Recover axis from vector part (normalized)
    double s = sin(recovered_half);
    if (fabs(s) > epsilon) {
      double ax = q(2) / s;
      double ay = q(3) / s;
      double az = q(4) / s;

      TS_ASSERT_DELTA(axis(1), ax, epsilon);
      TS_ASSERT_DELTA(axis(2), ay, epsilon);
      TS_ASSERT_DELTA(axis(3), az, epsilon);
    }
  }

  void testTransformationMatrixRows() {
    // Test that T matrix rows are orthonormal
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGMatrix33 T = q.GetT();

    // Extract rows
    JSBSim::FGColumnVector3 r1(T(1, 1), T(1, 2), T(1, 3));
    JSBSim::FGColumnVector3 r2(T(2, 1), T(2, 2), T(2, 3));
    JSBSim::FGColumnVector3 r3(T(3, 1), T(3, 2), T(3, 3));

    // Rows should be unit length
    TS_ASSERT_DELTA(1.0, r1.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, r2.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, r3.Magnitude(), epsilon);

    // Rows should be mutually orthogonal (dot products = 0)
    double d12 = r1(1) * r2(1) + r1(2) * r2(2) + r1(3) * r2(3);
    double d13 = r1(1) * r3(1) + r1(2) * r3(2) + r1(3) * r3(3);
    double d23 = r2(1) * r3(1) + r2(2) * r3(2) + r2(3) * r3(3);

    TS_ASSERT_DELTA(0.0, d12, epsilon);
    TS_ASSERT_DELTA(0.0, d13, epsilon);
    TS_ASSERT_DELTA(0.0, d23, epsilon);
  }

  void testTransformationMatrixColumns() {
    // Test that T matrix columns are orthonormal
    JSBSim::FGQuaternion q(0.4, 0.5, 0.6);
    JSBSim::FGMatrix33 T = q.GetT();

    // Extract columns
    JSBSim::FGColumnVector3 c1(T(1, 1), T(2, 1), T(3, 1));
    JSBSim::FGColumnVector3 c2(T(1, 2), T(2, 2), T(3, 2));
    JSBSim::FGColumnVector3 c3(T(1, 3), T(2, 3), T(3, 3));

    // Columns should be unit length
    TS_ASSERT_DELTA(1.0, c1.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, c2.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, c3.Magnitude(), epsilon);

    // Columns should be mutually orthogonal
    double d12 = c1(1) * c2(1) + c1(2) * c2(2) + c1(3) * c2(3);
    double d13 = c1(1) * c3(1) + c1(2) * c3(2) + c1(3) * c3(3);
    double d23 = c2(1) * c3(1) + c2(2) * c3(2) + c2(3) * c3(3);

    TS_ASSERT_DELTA(0.0, d12, epsilon);
    TS_ASSERT_DELTA(0.0, d13, epsilon);
    TS_ASSERT_DELTA(0.0, d23, epsilon);
  }

  void testScalarMultiplicationAssociativity() {
    // Test (a * b) * q = a * (b * q)
    double a = 2.0, b = 3.0;
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);

    JSBSim::FGQuaternion left = (a * b) * q;
    JSBSim::FGQuaternion right_inner = b * q;
    JSBSim::FGQuaternion right = a * right_inner;

    TS_ASSERT_DELTA(left(1), right(1), epsilon);
    TS_ASSERT_DELTA(left(2), right(2), epsilon);
    TS_ASSERT_DELTA(left(3), right(3), epsilon);
    TS_ASSERT_DELTA(left(4), right(4), epsilon);
  }

  void testNegativeQuaternionSameRotation() {
    // -q and q represent the same rotation
    JSBSim::FGQuaternion q(0.4, 0.5, 0.6);
    JSBSim::FGQuaternion neg_q = -1.0 * q;

    // Both should give same transformation matrix
    JSBSim::FGMatrix33 T1 = q.GetT();
    JSBSim::FGMatrix33 T2 = neg_q.GetT();

    TS_ASSERT_MATRIX_EQUALS(T1, T2);

    // Same Euler angles (or equivalent)
    JSBSim::FGColumnVector3 e1 = q.GetEuler();
    JSBSim::FGColumnVector3 e2 = neg_q.GetEuler();

    // Angles might differ by 2*pi but should give same rotation
    for (int i = 1; i <= 3; i++) {
      double diff = fabs(e1(i) - e2(i));
      TS_ASSERT(diff < epsilon || fabs(diff - 2.0 * M_PI) < epsilon);
    }
  }

  void testInverseOfIdentity() {
    // Identity quaternion should be its own inverse
    JSBSim::FGQuaternion identity;
    JSBSim::FGQuaternion inv = identity.Inverse();

    TS_ASSERT_DELTA(identity(1), inv(1), epsilon);
    TS_ASSERT_DELTA(identity(2), inv(2), epsilon);
    TS_ASSERT_DELTA(identity(3), inv(3), epsilon);
    TS_ASSERT_DELTA(identity(4), inv(4), epsilon);
  }

  void testConjugateOfIdentity() {
    // Identity quaternion should be its own conjugate
    JSBSim::FGQuaternion identity;
    JSBSim::FGQuaternion conj = identity.Conjugate();

    TS_ASSERT_DELTA(identity(1), conj(1), epsilon);
    TS_ASSERT_DELTA(identity(2), conj(2), epsilon);
    TS_ASSERT_DELTA(identity(3), conj(3), epsilon);
    TS_ASSERT_DELTA(identity(4), conj(4), epsilon);
  }

  void testAngularVelocityDirection() {
    // Test that QDot produces rotation in the correct direction
    JSBSim::FGQuaternion q;  // Identity
    double omega_x = 1.0;
    JSBSim::FGColumnVector3 omega(omega_x, 0.0, 0.0);

    JSBSim::FGQuaternion qDot = q.GetQDot(omega);

    // After small dt, q + dt*qDot should have small positive roll
    double dt = 0.01;
    JSBSim::FGQuaternion q_new = q + dt * qDot;
    q_new.Normalize();

    JSBSim::FGColumnVector3 euler = q_new.GetEuler();

    // Roll should be positive (matching omega_x direction)
    TS_ASSERT(euler(1) > 0.0);
    TS_ASSERT_DELTA(omega_x * dt, euler(1), 0.01);
  }

  void testTripleProduct() {
    // Test q1 * q2 * q3 is valid and produces correct type
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);
    JSBSim::FGQuaternion q3(0.7, 0.8, 0.9);

    JSBSim::FGQuaternion triple = q1 * q2 * q3;

    // Should still be unit quaternion
    TS_ASSERT_DELTA(1.0, triple.Magnitude(), epsilon);

    // Should be associative
    JSBSim::FGQuaternion left = (q1 * q2) * q3;
    JSBSim::FGQuaternion right = q1 * (q2 * q3);

    TS_ASSERT_DELTA(left(1), right(1), epsilon);
    TS_ASSERT_DELTA(left(2), right(2), epsilon);
    TS_ASSERT_DELTA(left(3), right(3), epsilon);
    TS_ASSERT_DELTA(left(4), right(4), epsilon);
  }

  void testDifferentAnglesAroundSameAxis() {
    // Two rotations around the same axis should commute
    double angle1 = M_PI / 6.0;
    double angle2 = M_PI / 4.0;

    JSBSim::FGQuaternion q1(1, angle1);
    JSBSim::FGQuaternion q2(1, angle2);

    JSBSim::FGQuaternion prod12 = q1 * q2;
    JSBSim::FGQuaternion prod21 = q2 * q1;

    // Rotations around same axis commute
    TS_ASSERT_DELTA(prod12(1), prod21(1), epsilon);
    TS_ASSERT_DELTA(prod12(2), prod21(2), epsilon);
    TS_ASSERT_DELTA(prod12(3), prod21(3), epsilon);
    TS_ASSERT_DELTA(prod12(4), prod21(4), epsilon);
  }

  void testNormalizeAfterScaling() {
    // After scaling and normalizing, should get back original unit quaternion
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    double orig1 = q(1), orig2 = q(2), orig3 = q(3), orig4 = q(4);

    // Scale by various factors
    double factors[] = {0.5, 2.0, 0.1, 10.0};
    for (double factor : factors) {
      JSBSim::FGQuaternion scaled = factor * q;
      scaled.Normalize();

      TS_ASSERT_DELTA(orig1, scaled(1), epsilon);
      TS_ASSERT_DELTA(orig2, scaled(2), epsilon);
      TS_ASSERT_DELTA(orig3, scaled(3), epsilon);
      TS_ASSERT_DELTA(orig4, scaled(4), epsilon);
    }
  }

  void testEulerAngleEdgeCasesNearZero() {
    // Test near-zero Euler angles
    double tiny = 1e-10;

    JSBSim::FGQuaternion q(tiny, tiny, tiny);

    JSBSim::FGColumnVector3 euler = q.GetEuler();

    // Should recover tiny values
    TS_ASSERT_DELTA(tiny, euler(1), 1e-8);
    TS_ASSERT_DELTA(tiny, euler(2), 1e-8);
    TS_ASSERT_DELTA(tiny, euler(3), 1e-8);
  }

  void testQuaternionOutputFormat() {
    // Test the Dump() output format
    JSBSim::FGQuaternion q(0.1, 0.2, 0.3);
    std::string s = q.Dump(",");

    // Should contain commas as separator
    TS_ASSERT(s.find(",") != std::string::npos);
  }

  void testZeroConstructor() {
    // Test FGQuaternion::zero()
    JSBSim::FGQuaternion zero = JSBSim::FGQuaternion::zero();

    TS_ASSERT_EQUALS(0.0, zero(1));
    TS_ASSERT_EQUALS(0.0, zero(2));
    TS_ASSERT_EQUALS(0.0, zero(3));
    TS_ASSERT_EQUALS(0.0, zero(4));

    // Zero quaternion has zero magnitude
    TS_ASSERT_EQUALS(0.0, zero.Magnitude());
    TS_ASSERT_EQUALS(0.0, zero.SqrMagnitude());
  }

  void testVectorConstructorEquivalence() {
    // FGQuaternion(phi, tht, psi) should equal FGQuaternion(FGColumnVector3(phi, tht, psi))
    double phi = 0.3, tht = 0.4, psi = 0.5;

    JSBSim::FGQuaternion q1(phi, tht, psi);
    JSBSim::FGColumnVector3 v(phi, tht, psi);
    JSBSim::FGQuaternion q2(v);

    TS_ASSERT_DELTA(q1(1), q2(1), epsilon);
    TS_ASSERT_DELTA(q1(2), q2(2), epsilon);
    TS_ASSERT_DELTA(q1(3), q2(3), epsilon);
    TS_ASSERT_DELTA(q1(4), q2(4), epsilon);
  }

  // ============================================================================
  // Multi-Instance Independence Tests (80-83)
  // ============================================================================

  void testMultipleQuaternionInstancesIndependent() {
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);
    JSBSim::FGQuaternion q3(0.7, 0.8, 0.9);

    // Save original values
    double q1_orig = q1(1);
    double q2_orig = q2(1);

    // Modify q1
    q1.Entry(1) = 0.999;

    // q2 and q3 should be unchanged
    TS_ASSERT_DELTA(q2_orig, q2(1), epsilon);
    TS_ASSERT(q2(1) != q1(1));
    TS_ASSERT(q3(1) != q1(1));
  }

  void testQuaternionOperationsDoNotAffectOthers() {
    JSBSim::FGQuaternion q1(0.2, 0.3, 0.4);
    JSBSim::FGQuaternion q2(0.5, 0.6, 0.7);

    double q2_1 = q2(1);
    double q2_2 = q2(2);
    double q2_3 = q2(3);
    double q2_4 = q2(4);

    // Operations on q1
    q1 *= 2.0;
    q1.Normalize();
    q1 = q1.Conjugate();

    // q2 unchanged
    TS_ASSERT_DELTA(q2_1, q2(1), epsilon);
    TS_ASSERT_DELTA(q2_2, q2(2), epsilon);
    TS_ASSERT_DELTA(q2_3, q2(3), epsilon);
    TS_ASSERT_DELTA(q2_4, q2(4), epsilon);
  }

  void testFourIndependentQuaternionRotations() {
    // Create four independent rotations
    JSBSim::FGQuaternion qx(1, M_PI / 6.0);
    JSBSim::FGQuaternion qy(2, M_PI / 4.0);
    JSBSim::FGQuaternion qz(3, M_PI / 3.0);
    JSBSim::FGQuaternion qxy(M_PI / 6.0, M_PI / 4.0, 0.0);

    // All should be unit quaternions
    TS_ASSERT_DELTA(1.0, qx.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, qy.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, qz.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, qxy.Magnitude(), epsilon);

    // All should produce different rotations
    TS_ASSERT(qx != qy);
    TS_ASSERT(qy != qz);
    TS_ASSERT(qz != qx);
    TS_ASSERT(qxy != qx);
    TS_ASSERT(qxy != qy);
  }

  void testMultipleMatrixConversions() {
    JSBSim::FGQuaternion q1(0.2, 0.3, 0.4);
    JSBSim::FGQuaternion q2(0.5, 0.6, 0.7);

    // Get matrices
    JSBSim::FGMatrix33 T1 = q1.GetT();
    JSBSim::FGMatrix33 T2 = q2.GetT();

    // Matrices should be different
    TS_ASSERT(T1(1, 1) != T2(1, 1) || T1(1, 2) != T2(1, 2));

    // Each should be proper rotation
    TS_ASSERT_DELTA(1.0, T1.Determinant(), epsilon);
    TS_ASSERT_DELTA(1.0, T2.Determinant(), epsilon);
  }

  // ============================================================================
  // State Consistency Tests (84-87)
  // ============================================================================

  void testQuaternionStateAfterMultipleOperations() {
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGMatrix33 T_orig = q.GetT();

    // Apply multiple operations that should cancel out
    q *= 2.0;
    q /= 2.0;

    // Matrix should be unchanged
    JSBSim::FGMatrix33 T_after = q.GetT();
    TS_ASSERT_MATRIX_EQUALS(T_orig, T_after);
  }

  void testConsistentEulerConversions() {
    // Multiple conversions should give same result
    JSBSim::FGQuaternion q(0.25, 0.35, 0.45);

    JSBSim::FGColumnVector3 euler1 = q.GetEuler();
    JSBSim::FGColumnVector3 euler2 = q.GetEuler();
    JSBSim::FGColumnVector3 euler3 = q.GetEuler();

    // All should be identical
    TS_ASSERT_DELTA(euler1(1), euler2(1), epsilon);
    TS_ASSERT_DELTA(euler1(2), euler2(2), epsilon);
    TS_ASSERT_DELTA(euler1(3), euler2(3), epsilon);
    TS_ASSERT_DELTA(euler1(1), euler3(1), epsilon);
    TS_ASSERT_DELTA(euler1(2), euler3(2), epsilon);
    TS_ASSERT_DELTA(euler1(3), euler3(3), epsilon);
  }

  void testTransformMatrixConsistency() {
    JSBSim::FGQuaternion q(0.35, 0.45, 0.55);

    // Get T multiple times
    JSBSim::FGMatrix33 T1 = q.GetT();
    JSBSim::FGMatrix33 T2 = q.GetT();
    JSBSim::FGMatrix33 TInv1 = q.GetTInv();
    JSBSim::FGMatrix33 TInv2 = q.GetTInv();

    // All should match
    TS_ASSERT_MATRIX_EQUALS(T1, T2);
    TS_ASSERT_MATRIX_EQUALS(TInv1, TInv2);

    // T * TInv = I
    JSBSim::FGMatrix33 product = T1 * TInv1;
    TS_ASSERT_MATRIX_IS_IDENTITY(product);
  }

  void testMagnitudeConsistencyAfterNormalize() {
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);

    // Scale up
    q *= 5.0;
    TS_ASSERT_DELTA(5.0, q.Magnitude(), epsilon);

    // Normalize
    q.Normalize();
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

    // Multiple magnitude calls should give same result
    double mag1 = q.Magnitude();
    double mag2 = q.Magnitude();
    double mag3 = q.Magnitude();

    TS_ASSERT_DELTA(mag1, mag2, epsilon);
    TS_ASSERT_DELTA(mag2, mag3, epsilon);
  }

  // ============================================================================
  // Edge Cases and Boundary Tests (88-91)
  // ============================================================================

  void testRotationByExactly360Degrees() {
    // Rotation by 360° should be identity (or -identity)
    JSBSim::FGQuaternion q360x(1, 2.0 * M_PI);
    JSBSim::FGQuaternion q360y(2, 2.0 * M_PI);
    JSBSim::FGQuaternion q360z(3, 2.0 * M_PI);

    // All should produce identity transformation matrix
    JSBSim::FGMatrix33 I;
    I.InitMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    TS_ASSERT_MATRIX_EQUALS(I, q360x.GetT());
    TS_ASSERT_MATRIX_EQUALS(I, q360y.GetT());
    TS_ASSERT_MATRIX_EQUALS(I, q360z.GetT());
  }

  void testRotationByExactly180DegreesAllAxes() {
    // Test 180° rotations produce correct matrices
    JSBSim::FGQuaternion q180x(1, M_PI);
    JSBSim::FGQuaternion q180y(2, M_PI);
    JSBSim::FGQuaternion q180z(3, M_PI);

    // All should be unit quaternions
    TS_ASSERT_DELTA(1.0, q180x.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, q180y.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, q180z.Magnitude(), epsilon);

    // Determinant should be 1
    TS_ASSERT_DELTA(1.0, q180x.GetT().Determinant(), epsilon);
    TS_ASSERT_DELTA(1.0, q180y.GetT().Determinant(), epsilon);
    TS_ASSERT_DELTA(1.0, q180z.GetT().Determinant(), epsilon);
  }

  void testExtremelySmallAngles() {
    // Test numerically small angles
    double tiny = 1e-12;

    JSBSim::FGQuaternion q(1, tiny);

    // Should be close to identity
    TS_ASSERT_DELTA(1.0, q(1), 1e-10);
    TS_ASSERT(fabs(q(2)) < 1e-10);
    TS_ASSERT_DELTA(0.0, q(3), epsilon);
    TS_ASSERT_DELTA(0.0, q(4), epsilon);

    // Transformation should be near identity
    JSBSim::FGMatrix33 T = q.GetT();
    TS_ASSERT_DELTA(1.0, T(1, 1), 1e-8);
    TS_ASSERT_DELTA(1.0, T(2, 2), 1e-8);
    TS_ASSERT_DELTA(1.0, T(3, 3), 1e-8);
  }

  void testNearSingularityAngles() {
    // Test angles near but not at gimbal lock
    double near_90 = M_PI / 2.0 - 1e-6;

    JSBSim::FGQuaternion q(0.0, near_90, 0.0);
    JSBSim::FGColumnVector3 euler = q.GetEuler();

    // Should produce valid (finite) result
    TS_ASSERT(!std::isnan(euler(1)));
    TS_ASSERT(!std::isnan(euler(2)));
    TS_ASSERT(!std::isnan(euler(3)));
    TS_ASSERT(std::isfinite(euler(1)));
    TS_ASSERT(std::isfinite(euler(2)));
    TS_ASSERT(std::isfinite(euler(3)));

    // Pitch should be close to 90°
    TS_ASSERT_DELTA(near_90, euler(2), 1e-4);
  }

  // ============================================================================
  // Stress Tests (92-95)
  // ============================================================================

  void testManyMultiplications() {
    JSBSim::FGQuaternion q(1, M_PI / 100.0);
    JSBSim::FGQuaternion accumulated;  // Identity

    // 100 small rotations
    for (int i = 0; i < 100; i++) {
      accumulated = accumulated * q;
    }

    // Should have rotated by approximately pi radians
    JSBSim::FGColumnVector3 euler = accumulated.GetEuler();
    TS_ASSERT_DELTA(M_PI, fabs(euler(1)), 0.01);

    // Should still be unit quaternion
    TS_ASSERT_DELTA(1.0, accumulated.Magnitude(), epsilon);
  }

  void testManyNormalizations() {
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    double orig1 = q(1), orig2 = q(2), orig3 = q(3), orig4 = q(4);

    // Scale and normalize many times
    for (int i = 0; i < 100; i++) {
      q *= (1.0 + 0.1 * (i % 3));
      q.Normalize();
    }

    // Should be same orientation
    TS_ASSERT_DELTA(orig1, q(1), epsilon);
    TS_ASSERT_DELTA(orig2, q(2), epsilon);
    TS_ASSERT_DELTA(orig3, q(3), epsilon);
    TS_ASSERT_DELTA(orig4, q(4), epsilon);
  }

  void testManyInverseMultiplications() {
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);
    JSBSim::FGQuaternion qInv = q.Inverse();

    // q * qInv many times should stay identity
    JSBSim::FGQuaternion result = q * qInv;
    for (int i = 0; i < 50; i++) {
      result = result * (q * qInv);
      result.Normalize();
    }

    // Should still be identity
    JSBSim::FGQuaternion identity;
    TS_ASSERT_DELTA(identity(1), result(1), 1e-6);
    TS_ASSERT_DELTA(identity(2), result(2), 1e-6);
    TS_ASSERT_DELTA(identity(3), result(3), 1e-6);
    TS_ASSERT_DELTA(identity(4), result(4), 1e-6);
  }

  void testStressQuaternionCreation() {
    // Create many quaternions with different parameters
    for (int i = 0; i < 100; i++) {
      double phi = (i * 0.01) - 0.5;
      double tht = (i * 0.02) - 1.0;
      double psi = (i * 0.015) - 0.75;

      JSBSim::FGQuaternion q(phi, tht, psi);

      // Should always be unit
      TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

      // Should produce valid transformation
      TS_ASSERT_DELTA(1.0, q.GetT().Determinant(), epsilon);
    }
  }

  // ============================================================================
  // Complete System Verification Tests (96-100)
  // ============================================================================

  void testCompleteQuaternionSystemVerification() {
    JSBSim::FGQuaternion q(0.3, 0.4, 0.5);

    // 1. Verify basic properties
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, q.SqrMagnitude(), epsilon);

    // 2. Verify transformation matrix properties
    JSBSim::FGMatrix33 T = q.GetT();
    JSBSim::FGMatrix33 TInv = q.GetTInv();
    TS_ASSERT_DELTA(1.0, T.Determinant(), epsilon);
    TS_ASSERT_MATRIX_EQUALS(T.Transposed(), TInv);
    TS_ASSERT_MATRIX_IS_IDENTITY(T * TInv);

    // 3. Verify Euler angle round-trip
    JSBSim::FGColumnVector3 euler = q.GetEuler();
    JSBSim::FGQuaternion q2(euler);
    TS_ASSERT_MATRIX_EQUALS(T, q2.GetT());

    // 4. Verify inverse property
    JSBSim::FGQuaternion qInv = q.Inverse();
    JSBSim::FGQuaternion product = q * qInv;
    JSBSim::FGQuaternion identity;
    TS_ASSERT_DELTA(identity(1), product(1), epsilon);
  }

  void testCompleteAxisAngleVerification() {
    // Test all three axes
    for (int axis = 1; axis <= 3; axis++) {
      double angle = M_PI / 5.0;
      JSBSim::FGQuaternion q(axis, angle);

      // 1. Unit quaternion
      TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

      // 2. Correct scalar component
      TS_ASSERT_DELTA(cos(angle / 2.0), q(1), epsilon);

      // 3. Correct vector component
      TS_ASSERT_DELTA(sin(angle / 2.0), q(axis + 1), epsilon);

      // 4. Other components zero
      for (int i = 2; i <= 4; i++) {
        if (i != axis + 1) {
          TS_ASSERT_DELTA(0.0, q(i), epsilon);
        }
      }

      // 5. Valid transformation matrix
      TS_ASSERT_DELTA(1.0, q.GetT().Determinant(), epsilon);
    }
  }

  void testCompleteEulerAngleVerification() {
    double phi = M_PI / 6.0;   // 30°
    double tht = M_PI / 4.0;   // 45°
    double psi = M_PI / 3.0;   // 60°

    JSBSim::FGQuaternion q(phi, tht, psi);

    // 1. Verify Euler angle recovery (radians)
    JSBSim::FGColumnVector3 euler = q.GetEuler();
    TS_ASSERT_DELTA(phi, euler(1), epsilon);
    TS_ASSERT_DELTA(tht, euler(2), epsilon);
    TS_ASSERT_DELTA(psi, euler(3), epsilon);

    // 2. Verify Euler angle recovery (degrees)
    JSBSim::FGColumnVector3 eulerDeg = q.GetEulerDeg();
    TS_ASSERT_DELTA(30.0, eulerDeg(1), epsilon);
    TS_ASSERT_DELTA(45.0, eulerDeg(2), epsilon);
    TS_ASSERT_DELTA(60.0, eulerDeg(3), epsilon);

    // 3. Verify sin/cos consistency
    for (int i = 1; i <= 3; i++) {
      double sinE = q.GetSinEuler(i);
      double cosE = q.GetCosEuler(i);
      TS_ASSERT_DELTA(1.0, sinE * sinE + cosE * cosE, epsilon);
    }
  }

  void testCompleteMultiplicationVerification() {
    JSBSim::FGQuaternion q1(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion q2(0.4, 0.5, 0.6);

    // 1. Both inputs unit
    TS_ASSERT_DELTA(1.0, q1.Magnitude(), epsilon);
    TS_ASSERT_DELTA(1.0, q2.Magnitude(), epsilon);

    // 2. Product is unit
    JSBSim::FGQuaternion product = q1 * q2;
    TS_ASSERT_DELTA(1.0, product.Magnitude(), epsilon);

    // 3. Product transformation is composition
    JSBSim::FGMatrix33 T1 = q1.GetT();
    JSBSim::FGMatrix33 T2 = q2.GetT();
    JSBSim::FGMatrix33 Tprod = product.GetT();
    JSBSim::FGMatrix33 Tcomposed = T2 * T1;
    TS_ASSERT_MATRIX_EQUALS(Tcomposed, Tprod);

    // 4. Inverse of product
    JSBSim::FGQuaternion productInv = product.Inverse();
    JSBSim::FGQuaternion expected = q1.Inverse() * q2.Inverse();
    JSBSim::FGQuaternion identity;
    JSBSim::FGQuaternion check = product * productInv;
    TS_ASSERT_DELTA(identity(1), check(1), epsilon);
  }

  void testCompleteQuaternionIntegration() {
    // Test complete quaternion workflow
    // 1. Create from Euler angles
    double phi = 0.2, tht = 0.3, psi = 0.4;
    JSBSim::FGQuaternion q(phi, tht, psi);

    // 2. Verify unit quaternion
    TS_ASSERT_DELTA(1.0, q.Magnitude(), epsilon);

    // 3. Get transformation matrices
    JSBSim::FGMatrix33 T = q.GetT();
    JSBSim::FGMatrix33 TInv = q.GetTInv();
    TS_ASSERT_MATRIX_IS_IDENTITY(T * TInv);

    // 4. Verify Euler round-trip
    JSBSim::FGColumnVector3 euler = q.GetEuler();
    TS_ASSERT_DELTA(phi, euler(1), epsilon);
    TS_ASSERT_DELTA(tht, euler(2), epsilon);
    TS_ASSERT_DELTA(psi, euler(3), epsilon);

    // 5. Reconstruct from matrix
    JSBSim::FGQuaternion q2(T);
    TS_ASSERT_MATRIX_EQUALS(T, q2.GetT());

    // 6. Test conjugate/inverse
    JSBSim::FGQuaternion qConj = q.Conjugate();
    JSBSim::FGQuaternion qInv = q.Inverse();
    TS_ASSERT_DELTA(qConj(1), qInv(1), epsilon);
    TS_ASSERT_DELTA(qConj(2), qInv(2), epsilon);
    TS_ASSERT_DELTA(qConj(3), qInv(3), epsilon);
    TS_ASSERT_DELTA(qConj(4), qInv(4), epsilon);

    // 7. Test q * qInv = identity
    JSBSim::FGQuaternion identity;
    JSBSim::FGQuaternion product = q * qInv;
    TS_ASSERT_DELTA(identity(1), product(1), epsilon);
    TS_ASSERT_DELTA(identity(2), product(2), epsilon);
    TS_ASSERT_DELTA(identity(3), product(3), epsilon);
    TS_ASSERT_DELTA(identity(4), product(4), epsilon);

    // 8. Test QDot
    JSBSim::FGColumnVector3 omega(0.1, 0.2, 0.3);
    JSBSim::FGQuaternion qDot = q.GetQDot(omega);
    TS_ASSERT(!std::isnan(qDot(1)));
    TS_ASSERT(!std::isnan(qDot(2)));
    TS_ASSERT(!std::isnan(qDot(3)));
    TS_ASSERT(!std::isnan(qDot(4)));
  }
};

/*******************************************************************************
 * FGQuaternion C172x Integration Tests
 * Tests quaternion operations using real C172x aircraft attitude data
 ******************************************************************************/
class FGQuaternionC172xTest : public CxxTest::TestSuite
{
public:
  JSBSim::FGFDMExec fdmex;
  std::string aircraft_path;

  void setUp() {
    aircraft_path = "aircraft";
    fdmex.SetAircraftPath(SGPath("aircraft"));
    fdmex.SetEnginePath(SGPath("engine"));
    fdmex.SetSystemsPath(SGPath("systems"));
  }

  void tearDown() {
    fdmex.ResetToInitialConditions(0);
  }

  // Test quaternion from level flight
  void testC172xLevelFlightQuaternion() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetPhiDegIC(0.0);
    ic->SetThetaDegIC(3.0);
    ic->SetPsiDegIC(0.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // Quaternion should be normalized
    TS_ASSERT_DELTA(quat.Magnitude(), 1.0, 1e-12);

    // Euler angles should match initial conditions
    JSBSim::FGColumnVector3 euler = quat.GetEuler();
    TS_ASSERT_DELTA(euler(1), 0.0, 0.1);  // Phi ~0
    TS_ASSERT(euler(2) > 0.0);  // Positive pitch
  }

  // Test quaternion from banked turn
  void testC172xBankedTurnQuaternion() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetPhiDegIC(30.0);  // 30 degree bank
    ic->SetThetaDegIC(5.0);
    ic->SetPsiDegIC(90.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(4000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // Quaternion should be normalized
    TS_ASSERT_DELTA(quat.Magnitude(), 1.0, 1e-12);

    // Euler angles should reflect bank
    JSBSim::FGColumnVector3 euler = quat.GetEuler();
    TS_ASSERT(fabs(euler(1)) > 0.3);  // Non-zero bank
  }

  // Test quaternion to transformation matrix
  void testC172xQuaternionToMatrix() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetPhiDegIC(10.0);
    ic->SetThetaDegIC(5.0);
    ic->SetPsiDegIC(45.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // Get transformation matrix from quaternion
    JSBSim::FGMatrix33 T = quat.GetT();
    JSBSim::FGMatrix33 TInv = quat.GetTInv();

    // Product should be identity
    JSBSim::FGMatrix33 product = T * TInv;
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        double expected = (i == j) ? 1.0 : 0.0;
        TS_ASSERT_DELTA(product(i, j), expected, 1e-10);
      }
    }
  }

  // Test quaternion multiplication
  void testC172xQuaternionMultiplication() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // q * q^-1 should equal identity
    JSBSim::FGQuaternion qInv = quat.Inverse();
    JSBSim::FGQuaternion product = quat * qInv;

    TS_ASSERT_DELTA(product(1), 1.0, 1e-10);
    TS_ASSERT_DELTA(product(2), 0.0, 1e-10);
    TS_ASSERT_DELTA(product(3), 0.0, 1e-10);
    TS_ASSERT_DELTA(product(4), 0.0, 1e-10);
  }

  // Test quaternion conjugate
  void testC172xQuaternionConjugate() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetPhiDegIC(15.0);
    ic->SetThetaDegIC(5.0);
    ic->SetPsiDegIC(90.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();
    JSBSim::FGQuaternion conj = quat.Conjugate();

    // Conjugate has same scalar part, negated vector part
    TS_ASSERT_DELTA(conj(1), quat(1), 1e-12);
    TS_ASSERT_DELTA(conj(2), -quat(2), 1e-12);
    TS_ASSERT_DELTA(conj(3), -quat(3), 1e-12);
    TS_ASSERT_DELTA(conj(4), -quat(4), 1e-12);

    // For unit quaternion, conjugate equals inverse
    JSBSim::FGQuaternion inv = quat.Inverse();
    TS_ASSERT_DELTA(conj(1), inv(1), 1e-12);
    TS_ASSERT_DELTA(conj(2), inv(2), 1e-12);
    TS_ASSERT_DELTA(conj(3), inv(3), 1e-12);
    TS_ASSERT_DELTA(conj(4), inv(4), 1e-12);
  }

  // Test Euler angle extraction
  void testC172xEulerAngleExtraction() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    double phi = 20.0 * M_PI / 180.0;
    double theta = 10.0 * M_PI / 180.0;
    double psi = 135.0 * M_PI / 180.0;
    ic->SetPhiRadIC(phi);
    ic->SetThetaRadIC(theta);
    ic->SetPsiRadIC(psi);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();
    JSBSim::FGColumnVector3 euler = quat.GetEuler();

    // Euler angles should approximately match
    TS_ASSERT_DELTA(euler(1), phi, 0.1);
    TS_ASSERT_DELTA(euler(2), theta, 0.1);
    TS_ASSERT_DELTA(euler(3), psi, 0.1);
  }

  // Test angular rate to quaternion derivative
  void testC172xQDot() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // Get current angular rates
    JSBSim::FGColumnVector3 pqr = propagate->GetPQR();

    // Compute quaternion derivative
    JSBSim::FGQuaternion qDot = quat.GetQDot(pqr);

    // QDot should be finite
    TS_ASSERT(std::isfinite(qDot(1)));
    TS_ASSERT(std::isfinite(qDot(2)));
    TS_ASSERT(std::isfinite(qDot(3)));
    TS_ASSERT(std::isfinite(qDot(4)));
  }

  // Test quaternion sine/cosine functions
  void testC172xQuaternionTrigFunctions() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetPhiDegIC(10.0);
    ic->SetThetaDegIC(5.0);
    ic->SetPsiDegIC(45.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // Get Euler sine/cosine values
    double sinPhi = quat.GetSinEuler(1);
    double cosPhi = quat.GetCosEuler(1);
    double sinTheta = quat.GetSinEuler(2);
    double cosTheta = quat.GetCosEuler(2);
    double sinPsi = quat.GetSinEuler(3);
    double cosPsi = quat.GetCosEuler(3);

    // sin^2 + cos^2 = 1
    TS_ASSERT_DELTA(sinPhi*sinPhi + cosPhi*cosPhi, 1.0, 1e-12);
    TS_ASSERT_DELTA(sinTheta*sinTheta + cosTheta*cosTheta, 1.0, 1e-12);
    TS_ASSERT_DELTA(sinPsi*sinPsi + cosPsi*cosPsi, 1.0, 1e-12);
  }

  // Test quaternion double rotation
  void testC172xDoubleRotation() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetPhiDegIC(15.0);
    ic->SetThetaDegIC(10.0);
    ic->SetPsiDegIC(60.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGQuaternion quat = propagate->GetQuaternion();

    // Apply rotation twice: q * q
    JSBSim::FGQuaternion doubleRot = quat * quat;

    // Result should still be a unit quaternion
    TS_ASSERT_DELTA(doubleRot.Magnitude(), 1.0, 1e-12);
  }

  // Test stability of quaternion over simulation
  void testC172xQuaternionStability() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    // Run for 10 timesteps
    for (int i = 0; i < 10; i++) {
      fdmex.Run();
      auto propagate = fdmex.GetPropagate();
      JSBSim::FGQuaternion quat = propagate->GetQuaternion();

      // Quaternion should stay normalized
      TS_ASSERT_DELTA(quat.Magnitude(), 1.0, 1e-10);
    }
  }
};
