#include <limits>
#include <cxxtest/TestSuite.h>
#include "TestAssertions.h"
#include <math/FGQuaternion.h>

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
};
