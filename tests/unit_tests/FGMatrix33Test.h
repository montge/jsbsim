#include <iomanip>
#include <cxxtest/TestSuite.h>
#include <math/FGMatrix33.h>
#include <math/FGQuaternion.h>

class FGMatrix33Test : public CxxTest::TestSuite
{
public:
  void testConstructors() {
    const JSBSim::FGMatrix33 m0;
    TS_ASSERT_EQUALS(m0.Rows(), 3);
    TS_ASSERT_EQUALS(m0.Cols(), 3);
    TS_ASSERT_EQUALS(m0.Entry(1,1), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(1,2), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(1,3), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(2,1), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(2,2), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(2,3), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(3,1), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(3,2), 0.0);
    TS_ASSERT_EQUALS(m0.Entry(3,3), 0.0);
    TS_ASSERT_EQUALS(m0(1,1), 0.0);
    TS_ASSERT_EQUALS(m0(1,2), 0.0);
    TS_ASSERT_EQUALS(m0(1,3), 0.0);
    TS_ASSERT_EQUALS(m0(2,1), 0.0);
    TS_ASSERT_EQUALS(m0(2,2), 0.0);
    TS_ASSERT_EQUALS(m0(2,3), 0.0);
    TS_ASSERT_EQUALS(m0(3,1), 0.0);
    TS_ASSERT_EQUALS(m0(3,2), 0.0);
    TS_ASSERT_EQUALS(m0(3,3), 0.0);
    JSBSim::FGMatrix33 m = m0;
    TS_ASSERT_EQUALS(m.Entry(1,1), 0.0);
    TS_ASSERT_EQUALS(m.Entry(1,2), 0.0);
    TS_ASSERT_EQUALS(m.Entry(1,3), 0.0);
    TS_ASSERT_EQUALS(m.Entry(2,1), 0.0);
    TS_ASSERT_EQUALS(m.Entry(2,2), 0.0);
    TS_ASSERT_EQUALS(m.Entry(2,3), 0.0);
    TS_ASSERT_EQUALS(m.Entry(3,1), 0.0);
    TS_ASSERT_EQUALS(m.Entry(3,2), 0.0);
    TS_ASSERT_EQUALS(m.Entry(3,3), 0.0);
    TS_ASSERT_EQUALS(m(1,1), 0.0);
    TS_ASSERT_EQUALS(m(1,2), 0.0);
    TS_ASSERT_EQUALS(m(1,3), 0.0);
    TS_ASSERT_EQUALS(m(2,1), 0.0);
    TS_ASSERT_EQUALS(m(2,2), 0.0);
    TS_ASSERT_EQUALS(m(2,3), 0.0);
    TS_ASSERT_EQUALS(m(3,1), 0.0);
    TS_ASSERT_EQUALS(m(3,2), 0.0);
    TS_ASSERT_EQUALS(m(3,3), 0.0);
    m(2,2) = 1.0;
    TS_ASSERT_EQUALS(m0(2,2), 0.0);
    JSBSim::FGMatrix33 m1(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    TS_ASSERT_EQUALS(m1(1,1), 1.0);
    TS_ASSERT_EQUALS(m1(1,2), 2.0);
    TS_ASSERT_EQUALS(m1(1,3), 3.0);
    TS_ASSERT_EQUALS(m1(2,1), 4.0);
    TS_ASSERT_EQUALS(m1(2,2), 5.0);
    TS_ASSERT_EQUALS(m1(2,3), 6.0);
    TS_ASSERT_EQUALS(m1(3,1), 7.0);
    TS_ASSERT_EQUALS(m1(3,2), 8.0);
    TS_ASSERT_EQUALS(m1(3,3), 9.0);
    m1.InitMatrix();
    TS_ASSERT_EQUALS(m1(1,1), 0.0);
    TS_ASSERT_EQUALS(m1(1,2), 0.0);
    TS_ASSERT_EQUALS(m1(1,3), 0.0);
    TS_ASSERT_EQUALS(m1(2,1), 0.0);
    TS_ASSERT_EQUALS(m1(2,2), 0.0);
    TS_ASSERT_EQUALS(m1(2,3), 0.0);
    TS_ASSERT_EQUALS(m1(3,1), 0.0);
    TS_ASSERT_EQUALS(m1(3,2), 0.0);
    TS_ASSERT_EQUALS(m1(3,3), 0.0);
  }

  void testTransposed() {
    const JSBSim::FGMatrix33 m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    JSBSim::FGMatrix33 mT = m.Transposed();
    TS_ASSERT_EQUALS(mT(1,1), 1.0);
    TS_ASSERT_EQUALS(mT(1,2), 4.0);
    TS_ASSERT_EQUALS(mT(1,3), 7.0);
    TS_ASSERT_EQUALS(mT(2,1), 2.0);
    TS_ASSERT_EQUALS(mT(2,2), 5.0);
    TS_ASSERT_EQUALS(mT(2,3), 8.0);
    TS_ASSERT_EQUALS(mT(3,1), 3.0);
    TS_ASSERT_EQUALS(mT(3,2), 6.0);
    TS_ASSERT_EQUALS(mT(3,3), 9.0);
    mT.InitMatrix(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    TS_ASSERT_EQUALS(mT(1,1), 1.0);
    TS_ASSERT_EQUALS(mT(1,2), 2.0);
    TS_ASSERT_EQUALS(mT(1,3), 3.0);
    TS_ASSERT_EQUALS(mT(2,1), 4.0);
    TS_ASSERT_EQUALS(mT(2,2), 5.0);
    TS_ASSERT_EQUALS(mT(2,3), 6.0);
    TS_ASSERT_EQUALS(mT(3,1), 7.0);
    TS_ASSERT_EQUALS(mT(3,2), 8.0);
    TS_ASSERT_EQUALS(mT(3,3), 9.0);
    mT.T();
    TS_ASSERT_EQUALS(mT(1,1), 1.0);
    TS_ASSERT_EQUALS(mT(1,2), 4.0);
    TS_ASSERT_EQUALS(mT(1,3), 7.0);
    TS_ASSERT_EQUALS(mT(2,1), 2.0);
    TS_ASSERT_EQUALS(mT(2,2), 5.0);
    TS_ASSERT_EQUALS(mT(2,3), 8.0);
    TS_ASSERT_EQUALS(mT(3,1), 3.0);
    TS_ASSERT_EQUALS(mT(3,2), 6.0);
    TS_ASSERT_EQUALS(mT(3,3), 9.0);
  }

  void testOperations() {
    JSBSim::FGMatrix33 m0;
    const JSBSim::FGMatrix33 m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    m0 = m;
    TS_ASSERT_EQUALS(m0(1,1), 1.0);
    TS_ASSERT_EQUALS(m0(1,2), 2.0);
    TS_ASSERT_EQUALS(m0(1,3), 3.0);
    TS_ASSERT_EQUALS(m0(2,1), 4.0);
    TS_ASSERT_EQUALS(m0(2,2), 5.0);
    TS_ASSERT_EQUALS(m0(2,3), 6.0);
    TS_ASSERT_EQUALS(m0(3,1), 7.0);
    TS_ASSERT_EQUALS(m0(3,2), 8.0);
    TS_ASSERT_EQUALS(m0(3,3), 9.0);
    m0(2,2) = -10.0;
    TS_ASSERT_EQUALS(m(2,2), 5.0);
    m0 = -1.0 * m;
    TS_ASSERT_EQUALS(m0(1,1), -1.0);
    TS_ASSERT_EQUALS(m0(1,2), -2.0);
    TS_ASSERT_EQUALS(m0(1,3), -3.0);
    TS_ASSERT_EQUALS(m0(2,1), -4.0);
    TS_ASSERT_EQUALS(m0(2,2), -5.0);
    TS_ASSERT_EQUALS(m0(2,3), -6.0);
    TS_ASSERT_EQUALS(m0(3,1), -7.0);
    TS_ASSERT_EQUALS(m0(3,2), -8.0);
    TS_ASSERT_EQUALS(m0(3,3), -9.0);
    const JSBSim::FGMatrix33 m_twice = m * 2.0;
    TS_ASSERT_EQUALS(m_twice(1,1), 2.0);
    TS_ASSERT_EQUALS(m_twice(1,2), 4.0);
    TS_ASSERT_EQUALS(m_twice(1,3), 6.0);
    TS_ASSERT_EQUALS(m_twice(2,1), 8.0);
    TS_ASSERT_EQUALS(m_twice(2,2), 10.0);
    TS_ASSERT_EQUALS(m_twice(2,3), 12.0);
    TS_ASSERT_EQUALS(m_twice(3,1), 14.0);
    TS_ASSERT_EQUALS(m_twice(3,2), 16.0);
    TS_ASSERT_EQUALS(m_twice(3,3), 18.0);
    JSBSim::FGMatrix33 m_res = m_twice - m;
    TS_ASSERT_EQUALS(m_res(1,1), 1.0);
    TS_ASSERT_EQUALS(m_res(1,2), 2.0);
    TS_ASSERT_EQUALS(m_res(1,3), 3.0);
    TS_ASSERT_EQUALS(m_res(2,1), 4.0);
    TS_ASSERT_EQUALS(m_res(2,2), 5.0);
    TS_ASSERT_EQUALS(m_res(2,3), 6.0);
    TS_ASSERT_EQUALS(m_res(3,1), 7.0);
    TS_ASSERT_EQUALS(m_res(3,2), 8.0);
    TS_ASSERT_EQUALS(m_res(3,3), 9.0);
    m_res = m_twice;
    m_res -= m;
    TS_ASSERT_EQUALS(m_res(1,1), 1.0);
    TS_ASSERT_EQUALS(m_res(1,2), 2.0);
    TS_ASSERT_EQUALS(m_res(1,3), 3.0);
    TS_ASSERT_EQUALS(m_res(2,1), 4.0);
    TS_ASSERT_EQUALS(m_res(2,2), 5.0);
    TS_ASSERT_EQUALS(m_res(2,3), 6.0);
    TS_ASSERT_EQUALS(m_res(3,1), 7.0);
    TS_ASSERT_EQUALS(m_res(3,2), 8.0);
    TS_ASSERT_EQUALS(m_res(3,3), 9.0);
    m_res = m_twice + m;
    TS_ASSERT_EQUALS(m_res(1,1), 3.0);
    TS_ASSERT_EQUALS(m_res(1,2), 6.0);
    TS_ASSERT_EQUALS(m_res(1,3), 9.0);
    TS_ASSERT_EQUALS(m_res(2,1), 12.0);
    TS_ASSERT_EQUALS(m_res(2,2), 15.0);
    TS_ASSERT_EQUALS(m_res(2,3), 18.0);
    TS_ASSERT_EQUALS(m_res(3,1), 21.0);
    TS_ASSERT_EQUALS(m_res(3,2), 24.0);
    TS_ASSERT_EQUALS(m_res(3,3), 27.0);
    m_res += m;
    TS_ASSERT_EQUALS(m_res(1,1), 4.0);
    TS_ASSERT_EQUALS(m_res(1,2), 8.0);
    TS_ASSERT_EQUALS(m_res(1,3), 12.0);
    TS_ASSERT_EQUALS(m_res(2,1), 16.0);
    TS_ASSERT_EQUALS(m_res(2,2), 20.0);
    TS_ASSERT_EQUALS(m_res(2,3), 24.0);
    TS_ASSERT_EQUALS(m_res(3,1), 28.0);
    TS_ASSERT_EQUALS(m_res(3,2), 32.0);
    TS_ASSERT_EQUALS(m_res(3,3), 36.0);
    m_res *= 0.25;
    TS_ASSERT_EQUALS(m_res(1,1), 1.0);
    TS_ASSERT_EQUALS(m_res(1,2), 2.0);
    TS_ASSERT_EQUALS(m_res(1,3), 3.0);
    TS_ASSERT_EQUALS(m_res(2,1), 4.0);
    TS_ASSERT_EQUALS(m_res(2,2), 5.0);
    TS_ASSERT_EQUALS(m_res(2,3), 6.0);
    TS_ASSERT_EQUALS(m_res(3,1), 7.0);
    TS_ASSERT_EQUALS(m_res(3,2), 8.0);
    TS_ASSERT_EQUALS(m_res(3,3), 9.0);
    m_res = m_twice / 2.0;
    TS_ASSERT_EQUALS(m_res(1,1), 1.0);
    TS_ASSERT_EQUALS(m_res(1,2), 2.0);
    TS_ASSERT_EQUALS(m_res(1,3), 3.0);
    TS_ASSERT_EQUALS(m_res(2,1), 4.0);
    TS_ASSERT_EQUALS(m_res(2,2), 5.0);
    TS_ASSERT_EQUALS(m_res(2,3), 6.0);
    TS_ASSERT_EQUALS(m_res(3,1), 7.0);
    TS_ASSERT_EQUALS(m_res(3,2), 8.0);
    TS_ASSERT_EQUALS(m_res(3,3), 9.0);
    m_res = m_twice;
    m_res /= 2.0;
    TS_ASSERT_EQUALS(m_res(1,1), 1.0);
    TS_ASSERT_EQUALS(m_res(1,2), 2.0);
    TS_ASSERT_EQUALS(m_res(1,3), 3.0);
    TS_ASSERT_EQUALS(m_res(2,1), 4.0);
    TS_ASSERT_EQUALS(m_res(2,2), 5.0);
    TS_ASSERT_EQUALS(m_res(2,3), 6.0);
    TS_ASSERT_EQUALS(m_res(3,1), 7.0);
    TS_ASSERT_EQUALS(m_res(3,2), 8.0);
    TS_ASSERT_EQUALS(m_res(3,3), 9.0);
    const JSBSim::FGMatrix33 eye(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    const JSBSim::FGColumnVector3 v0(1.0, -2.0, 3.0);
    JSBSim::FGColumnVector3 v = eye * v0;
    TS_ASSERT_EQUALS(v, v0);
    m_res = m_twice * eye;
    TS_ASSERT_EQUALS(m_res(1,1), 2.0);
    TS_ASSERT_EQUALS(m_res(1,2), 4.0);
    TS_ASSERT_EQUALS(m_res(1,3), 6.0);
    TS_ASSERT_EQUALS(m_res(2,1), 8.0);
    TS_ASSERT_EQUALS(m_res(2,2), 10.0);
    TS_ASSERT_EQUALS(m_res(2,3), 12.0);
    TS_ASSERT_EQUALS(m_res(3,1), 14.0);
    TS_ASSERT_EQUALS(m_res(3,2), 16.0);
    TS_ASSERT_EQUALS(m_res(3,3), 18.0);
    m_res = eye * m_twice;
    TS_ASSERT_EQUALS(m_res(1,1), 2.0);
    TS_ASSERT_EQUALS(m_res(1,2), 4.0);
    TS_ASSERT_EQUALS(m_res(1,3), 6.0);
    TS_ASSERT_EQUALS(m_res(2,1), 8.0);
    TS_ASSERT_EQUALS(m_res(2,2), 10.0);
    TS_ASSERT_EQUALS(m_res(2,3), 12.0);
    TS_ASSERT_EQUALS(m_res(3,1), 14.0);
    TS_ASSERT_EQUALS(m_res(3,2), 16.0);
    TS_ASSERT_EQUALS(m_res(3,3), 18.0);
    m_res *= eye;
    TS_ASSERT_EQUALS(m_res(1,1), 2.0);
    TS_ASSERT_EQUALS(m_res(1,2), 4.0);
    TS_ASSERT_EQUALS(m_res(1,3), 6.0);
    TS_ASSERT_EQUALS(m_res(2,1), 8.0);
    TS_ASSERT_EQUALS(m_res(2,2), 10.0);
    TS_ASSERT_EQUALS(m_res(2,3), 12.0);
    TS_ASSERT_EQUALS(m_res(3,1), 14.0);
    TS_ASSERT_EQUALS(m_res(3,2), 16.0);
    TS_ASSERT_EQUALS(m_res(3,3), 18.0);
  }

  void testInversion() {
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    JSBSim::FGMatrix33 m_res;
    TS_ASSERT_EQUALS(m.Determinant(), 1.0);
    TS_ASSERT(m.Invertible());
    m_res = m.Inverse();
    TS_ASSERT_EQUALS(m_res(1,1), 1.0);
    TS_ASSERT_EQUALS(m_res(1,2), 0.0);
    TS_ASSERT_EQUALS(m_res(1,3), 0.0);
    TS_ASSERT_EQUALS(m_res(2,1), 0.0);
    TS_ASSERT_EQUALS(m_res(2,2), 1.0);
    TS_ASSERT_EQUALS(m_res(2,3), 0.0);
    TS_ASSERT_EQUALS(m_res(3,1), 0.0);
    TS_ASSERT_EQUALS(m_res(3,2), 0.0);
    TS_ASSERT_EQUALS(m_res(3,3), 1.0);
    m.InitMatrix();
    m(1,1) = 1.0;
    TS_ASSERT_EQUALS(m.Determinant(), 0.0);
    TS_ASSERT(!m.Invertible());
    m_res = m.Inverse();
    TS_ASSERT_EQUALS(m_res(1,1), 0.0);
    TS_ASSERT_EQUALS(m_res(1,2), 0.0);
    TS_ASSERT_EQUALS(m_res(1,3), 0.0);
    TS_ASSERT_EQUALS(m_res(2,1), 0.0);
    TS_ASSERT_EQUALS(m_res(2,2), 0.0);
    TS_ASSERT_EQUALS(m_res(2,3), 0.0);
    TS_ASSERT_EQUALS(m_res(3,1), 0.0);
    TS_ASSERT_EQUALS(m_res(3,2), 0.0);
    TS_ASSERT_EQUALS(m_res(3,3), 0.0);
  }

  // Check the assignment via an initializer list
  void testAssignmentInitializerList(void) {
    JSBSim::FGMatrix33 m;
    TS_ASSERT_EQUALS(m(1,1), 0.0);
    TS_ASSERT_EQUALS(m(1,2), 0.0);
    TS_ASSERT_EQUALS(m(1,3), 0.0);
    TS_ASSERT_EQUALS(m(2,1), 0.0);
    TS_ASSERT_EQUALS(m(2,2), 0.0);
    TS_ASSERT_EQUALS(m(2,3), 0.0);
    TS_ASSERT_EQUALS(m(3,1), 0.0);
    TS_ASSERT_EQUALS(m(3,2), 0.0);
    TS_ASSERT_EQUALS(m(3,3), 0.0);

    m = { 1.0, 2.0, -3.0,
          4.0, -5.0, 6.0,
          -7.0, 8.0, 9.0};

    TS_ASSERT_EQUALS(m(1,1), 1.0);
    TS_ASSERT_EQUALS(m(1,2), 2.0);
    TS_ASSERT_EQUALS(m(1,3), -3.0);
    TS_ASSERT_EQUALS(m(2,1), 4.0);
    TS_ASSERT_EQUALS(m(2,2), -5.0);
    TS_ASSERT_EQUALS(m(2,3), 6.0);
    TS_ASSERT_EQUALS(m(3,1), -7.0);
    TS_ASSERT_EQUALS(m(3,2), 8.0);
    TS_ASSERT_EQUALS(m(3,3), 9.0);
  }

  void testInputOutput() {
    std::ostringstream os, os_ref;
    JSBSim::FGMatrix33 m;
    std::istringstream values("1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0");

    values >> m;
    TS_ASSERT_EQUALS(m(1,1), 1.0);
    TS_ASSERT_EQUALS(m(1,2), 2.0);
    TS_ASSERT_EQUALS(m(1,3), 3.0);
    TS_ASSERT_EQUALS(m(2,1), 4.0);
    TS_ASSERT_EQUALS(m(2,2), 5.0);
    TS_ASSERT_EQUALS(m(2,3), 6.0);
    TS_ASSERT_EQUALS(m(3,1), 7.0);
    TS_ASSERT_EQUALS(m(3,2), 8.0);
    TS_ASSERT_EQUALS(m(3,3), 9.0);

    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++) {
        os << m(i,j);
        if (i!=3 || j!=3)
          os << ", ";
      }
    os_ref << m;
    TS_ASSERT_EQUALS(os_ref.str(), os.str());

    os.clear();
    os.str("");
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++) {
        os << std::setw(12) << std::setprecision(10) << m(i,j);
        if (i!=3 || j!=3)
          os << ", ";
      }
    TS_ASSERT_EQUALS(m.Dump(", "), os.str());

    os.clear();
    os.str("");
    for (int i=1; i<=3; i++) {
      os << "# ";
      for (int j=1; j<=3; j++) {
        os << std::right << std::fixed << std::setw(9);
        os << std::setprecision(6) << m(i,j);
        if (j != 3)
          os << ", ";
        else {
          if (i != 3)
            os << std::endl;
          else
            os << std::setw(0) << std::left;
        }
      }
    }
    TS_ASSERT_EQUALS(m.Dump(", ", "# "), os.str());
  }

  void testAngles() {
    double phi = 10. * M_PI / 180.;
    double theta = 45. * M_PI / 180.;
    double psi = 265. * M_PI / 180.;
    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta), sth = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    // Check that m is orthogonal
    TS_ASSERT_DELTA(m.Determinant(), 1.0, 1E-8);
    JSBSim::FGMatrix33 mInv = m.Inverse();
    JSBSim::FGMatrix33 mT = m.Transposed();
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++) {
        TS_ASSERT_DELTA(m(i,j), mInv(j,i), 1E-8);
        TS_ASSERT_DELTA(mT(i,j), mInv(i,j), 1E-8);
      }
    JSBSim::FGMatrix33 eye = m * mInv;
    for (int i=1; i<=3; i++)
      for (int j=1; j<=3; j++) {
        if (i == j) {
          TS_ASSERT_DELTA(eye(i,j), 1.0, 1E-8);
        } else {
          TS_ASSERT_DELTA(eye(i,j), 0.0, 1E-8);
        }
      }
    m.InitMatrix(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(2), 0.5*M_PI, 1E-8);
    m.InitMatrix(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(2), -0.5*M_PI, 1E-8);

    JSBSim::FGQuaternion q = m_phi.GetQuaternion();
    TS_ASSERT_DELTA(q(1), cos(0.5*phi), 1E-8);
    TS_ASSERT_DELTA(q(2), sin(0.5*phi), 1E-8);
    TS_ASSERT_DELTA(q(3), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);

    q = m_th.GetQuaternion();
    TS_ASSERT_DELTA(q(1), cos(0.5*theta), 1E-8);
    TS_ASSERT_DELTA(q(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(3), sin(0.5*theta), 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);

    q = m_psi.GetQuaternion();
    TS_ASSERT_DELTA(q(1), cos(0.5*psi), 1E-8);
    TS_ASSERT_DELTA(q(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(3), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(4), sin(0.5*psi), 1E-8);

    // These ones are designed to activate specific branches in
    // FGMatrix33::GetQuaternion()
    phi = 100. * M_PI / 180.;
    cphi = cos(phi); sphi = sin(phi);
    m.InitMatrix(1.0, 0.0, 0.0, 0.0, cphi, sphi, 0.0, -sphi, cphi);
    q = m.GetQuaternion();
    TS_ASSERT_DELTA(q(1), cos(0.5*phi), 1E-8);
    TS_ASSERT_DELTA(q(2), sin(0.5*phi), 1E-8);
    TS_ASSERT_DELTA(q(3), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);
    theta = 100. * M_PI / 180.;
    cth = cos(theta); sth = sin(theta);
    m.InitMatrix(cth, 0.0, -sth, 0.0, 1.0, 0.0, sth, 0.0, cth);
    q = m.GetQuaternion();
    TS_ASSERT_DELTA(q(1), cos(0.5*theta), 1E-8);
    TS_ASSERT_DELTA(q(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(3), sin(0.5*theta), 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);
  }

  void test_angles_psi_270deg()
  {
    double phi = 10. * M_PI / 180.;
    double theta = 45. * M_PI / 180.;
    double psi = 1.5*M_PI;
    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta), sth = sin(theta);
    double cpsi = 0.0, spsi = -1.0;
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
  }

  void test_angles_phi_m90deg()
  {
    double phi = -0.5*M_PI;
    double theta = 10. * M_PI / 180.;
    double psi = 45. * M_PI / 180.;
    double cphi = 0.0, sphi = -1.0;
    double cth = cos(theta), sth = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
  }

  void test_angles_gimbal_lock_up()
  {
    double phi = 28. * M_PI / 180.;
    double theta = 0.5*M_PI;
    double psi = 0.0;
    double cphi = cos(phi), sphi = sin(phi);
    double cth = 0.0, sth = 1.0;
    double cpsi = 1.0, spsi = 0.0;
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
  }

  void test_angles_gimbal_lock_up2()
  {
    double phi = 28. * M_PI / 180.;
    double theta = 0.5*M_PI;
    double psi = 17. * M_PI / 180.;
    double cphi = cos(phi), sphi = sin(phi);
    double cth = 0.0, sth = 1.0;
    double cpsi = cos(psi), spsi = sin(psi);
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi-psi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
  }

  void test_angles_gimbal_lock_down()
  {
    double phi = 28. * M_PI / 180.;
    double theta = -0.5*M_PI;
    double psi = 0.0;
    double cphi = cos(phi), sphi = sin(phi);
    double cth = 0.0, sth = -1.0;
    double cpsi = 1.0, spsi = 0.0;
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
  }

  void test_angles_gimbal_lock_down2()
  {
    double phi = 28. * M_PI / 180.;
    double theta = -0.5*M_PI;
    double psi = 17. * M_PI / 180.;
    double cphi = cos(phi), sphi = sin(phi);
    double cth = 0.0, sth = -1.0;
    double cpsi = cos(psi), spsi = sin(psi);
    const JSBSim::FGMatrix33 m_phi(1.0,   0.0,  0.0,
                                   0.0,  cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0,  0.0,
                                  sth, 0.0,  cth);
    const JSBSim::FGMatrix33 m_psi(cpsi,  spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0,    0.0, 1.0);
    JSBSim::FGColumnVector3 angles = m_phi.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_th.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
    angles = m_psi.GetEuler();
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    angles = m.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi+psi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
  }

  // ============ Edge Case Tests for Singular Matrices ============

  void testSingularZeroMatrix() {
    // GIVEN: A zero matrix (all elements zero)
    JSBSim::FGMatrix33 m;  // Default constructor gives zero matrix

    // THEN: Determinant should be zero, not invertible
    TS_ASSERT_EQUALS(m.Determinant(), 0.0);
    TS_ASSERT(!m.Invertible());

    // Inverse should return zero matrix
    JSBSim::FGMatrix33 inv = m.Inverse();
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(inv(i, j), 0.0);
  }

  void testSingularRank1Matrix() {
    // GIVEN: A rank-1 matrix (all rows are multiples of each other)
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          2.0, 4.0, 6.0,
                          3.0, 6.0, 9.0);

    // THEN: Determinant should be zero, not invertible
    TS_ASSERT_DELTA(m.Determinant(), 0.0, 1E-10);
    TS_ASSERT(!m.Invertible());
  }

  void testSingularRank2Matrix() {
    // GIVEN: A rank-2 matrix (third row is sum of first two)
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          5.0, 7.0, 9.0);  // Row 3 = Row 1 + Row 2

    // THEN: Determinant should be zero, not invertible
    TS_ASSERT_DELTA(m.Determinant(), 0.0, 1E-10);
    TS_ASSERT(!m.Invertible());
  }

  void testSingularColumnDependent() {
    // GIVEN: A matrix where column 3 = column 1 + column 2
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 9.0,
                          7.0, 8.0, 15.0);

    // THEN: Determinant should be zero
    TS_ASSERT_DELTA(m.Determinant(), 0.0, 1E-10);
    TS_ASSERT(!m.Invertible());
  }

  void testDeterminantPositive() {
    // GIVEN: A matrix with positive determinant
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0,
                          0.0, 2.0, 0.0,
                          0.0, 0.0, 3.0);  // Diagonal matrix, det = 6

    // THEN: Determinant should be 6
    TS_ASSERT_EQUALS(m.Determinant(), 6.0);
    TS_ASSERT(m.Invertible());
  }

  void testDeterminantNegative() {
    // GIVEN: A matrix with negative determinant (reflection)
    JSBSim::FGMatrix33 m(-1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0);  // Reflection, det = -1

    // THEN: Determinant should be -1
    TS_ASSERT_EQUALS(m.Determinant(), -1.0);
    TS_ASSERT(m.Invertible());
  }

  void testInverseOfDiagonalMatrix() {
    // GIVEN: A diagonal matrix
    JSBSim::FGMatrix33 m(2.0, 0.0, 0.0,
                          0.0, 4.0, 0.0,
                          0.0, 0.0, 5.0);

    // WHEN: Computing inverse
    JSBSim::FGMatrix33 inv = m.Inverse();

    // THEN: Inverse should have reciprocals on diagonal
    TS_ASSERT_DELTA(inv(1, 1), 0.5, 1E-10);
    TS_ASSERT_DELTA(inv(2, 2), 0.25, 1E-10);
    TS_ASSERT_DELTA(inv(3, 3), 0.2, 1E-10);
    TS_ASSERT_EQUALS(inv(1, 2), 0.0);
    TS_ASSERT_EQUALS(inv(1, 3), 0.0);
    TS_ASSERT_EQUALS(inv(2, 1), 0.0);
    TS_ASSERT_EQUALS(inv(2, 3), 0.0);
    TS_ASSERT_EQUALS(inv(3, 1), 0.0);
    TS_ASSERT_EQUALS(inv(3, 2), 0.0);

    // Verify M * M^-1 = I
    JSBSim::FGMatrix33 product = m * inv;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(product(i, j), (i == j ? 1.0 : 0.0), 1E-10);
  }

  void testInverseSymmetricMatrix() {
    // GIVEN: A symmetric positive definite matrix
    JSBSim::FGMatrix33 m(4.0, 2.0, 1.0,
                          2.0, 5.0, 2.0,
                          1.0, 2.0, 6.0);

    // WHEN: Computing inverse
    JSBSim::FGMatrix33 inv = m.Inverse();

    // THEN: Inverse should also be symmetric
    TS_ASSERT_DELTA(inv(1, 2), inv(2, 1), 1E-10);
    TS_ASSERT_DELTA(inv(1, 3), inv(3, 1), 1E-10);
    TS_ASSERT_DELTA(inv(2, 3), inv(3, 2), 1E-10);

    // Verify M * M^-1 = I
    JSBSim::FGMatrix33 product = m * inv;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(product(i, j), (i == j ? 1.0 : 0.0), 1E-10);
  }

  void testMatrixMultiplicationWithZero() {
    // GIVEN: Any matrix and zero matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    JSBSim::FGMatrix33 zero;

    // WHEN: Multiplying with zero matrix
    JSBSim::FGMatrix33 result1 = m * zero;
    JSBSim::FGMatrix33 result2 = zero * m;

    // THEN: Result should be zero matrix
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT_EQUALS(result1(i, j), 0.0);
        TS_ASSERT_EQUALS(result2(i, j), 0.0);
      }
  }

  void testTransposedTwiceIsOriginal() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

    // WHEN: Transposing twice
    JSBSim::FGMatrix33 mTT = m.Transposed().Transposed();

    // THEN: Should equal original
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(mTT(i, j), m(i, j));
  }

  void testDivisionByZeroProducesInfinity() {
    // GIVEN: A matrix with positive values
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

    // WHEN: Dividing by zero
    JSBSim::FGMatrix33 result = m / 0.0;

    // THEN: Result should be infinity (IEEE 754 behavior)
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT(std::isinf(result(i, j)));
  }

  void testScaleByOne() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

    // WHEN: Scaling by 1
    JSBSim::FGMatrix33 result = m * 1.0;

    // THEN: Result should equal original
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(result(i, j), m(i, j));
  }

  // ============ Additional Tests for Matrix Row/Column Access ============

  void testRowAndColumnAccess() {
    // GIVEN: A matrix with known values
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // THEN: Verify all elements can be read via Entry()
    TS_ASSERT_EQUALS(m.Entry(1, 1), 1.0);
    TS_ASSERT_EQUALS(m.Entry(1, 2), 2.0);
    TS_ASSERT_EQUALS(m.Entry(1, 3), 3.0);
    TS_ASSERT_EQUALS(m.Entry(2, 1), 4.0);
    TS_ASSERT_EQUALS(m.Entry(2, 2), 5.0);
    TS_ASSERT_EQUALS(m.Entry(2, 3), 6.0);
    TS_ASSERT_EQUALS(m.Entry(3, 1), 7.0);
    TS_ASSERT_EQUALS(m.Entry(3, 2), 8.0);
    TS_ASSERT_EQUALS(m.Entry(3, 3), 9.0);

    // Test write access via Entry()
    m.Entry(2, 2) = 99.0;
    TS_ASSERT_EQUALS(m.Entry(2, 2), 99.0);
    TS_ASSERT_EQUALS(m(2, 2), 99.0);
  }

  // ============ Tests for Non-Trivial Matrix Operations ============

  void testNonIdentityMatrixInverse() {
    // GIVEN: A non-identity invertible matrix
    JSBSim::FGMatrix33 m(2.0, 1.0, 0.0,
                          1.0, 3.0, 1.0,
                          0.0, 1.0, 2.0);

    // WHEN: Computing inverse
    JSBSim::FGMatrix33 inv = m.Inverse();

    // THEN: m * inv should equal identity
    JSBSim::FGMatrix33 product = m * inv;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(product(i, j), (i == j ? 1.0 : 0.0), 1E-10);

    // Also verify inv * m equals identity
    product = inv * m;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(product(i, j), (i == j ? 1.0 : 0.0), 1E-10);
  }

  void testComplexMatrixMultiplication() {
    // GIVEN: Two non-trivial matrices
    JSBSim::FGMatrix33 m1(1.0, 2.0, 3.0,
                           4.0, 5.0, 6.0,
                           7.0, 8.0, 9.0);
    JSBSim::FGMatrix33 m2(9.0, 8.0, 7.0,
                           6.0, 5.0, 4.0,
                           3.0, 2.0, 1.0);

    // WHEN: Computing product
    JSBSim::FGMatrix33 product = m1 * m2;

    // THEN: Verify correct matrix multiplication
    // Row 1: [1 2 3] * [[9 8 7], [6 5 4], [3 2 1]]
    TS_ASSERT_EQUALS(product(1, 1), 1.0*9.0 + 2.0*6.0 + 3.0*3.0);  // 30
    TS_ASSERT_EQUALS(product(1, 2), 1.0*8.0 + 2.0*5.0 + 3.0*2.0);  // 24
    TS_ASSERT_EQUALS(product(1, 3), 1.0*7.0 + 2.0*4.0 + 3.0*1.0);  // 18

    // Row 2: [4 5 6] * [[9 8 7], [6 5 4], [3 2 1]]
    TS_ASSERT_EQUALS(product(2, 1), 4.0*9.0 + 5.0*6.0 + 6.0*3.0);  // 84
    TS_ASSERT_EQUALS(product(2, 2), 4.0*8.0 + 5.0*5.0 + 6.0*2.0);  // 69
    TS_ASSERT_EQUALS(product(2, 3), 4.0*7.0 + 5.0*4.0 + 6.0*1.0);  // 54

    // Row 3: [7 8 9] * [[9 8 7], [6 5 4], [3 2 1]]
    TS_ASSERT_EQUALS(product(3, 1), 7.0*9.0 + 8.0*6.0 + 9.0*3.0);  // 138
    TS_ASSERT_EQUALS(product(3, 2), 7.0*8.0 + 8.0*5.0 + 9.0*2.0);  // 114
    TS_ASSERT_EQUALS(product(3, 3), 7.0*7.0 + 8.0*4.0 + 9.0*1.0);  // 90
  }

  void testMatrixVectorMultiplicationNonTrivial() {
    // GIVEN: A rotation matrix (90 deg around Z axis) and a vector
    double angle = M_PI / 2.0;  // 90 degrees
    JSBSim::FGMatrix33 rot_z(cos(angle), sin(angle), 0.0,
                              -sin(angle), cos(angle), 0.0,
                              0.0, 0.0, 1.0);
    JSBSim::FGColumnVector3 v(1.0, 0.0, 0.0);  // Unit X vector

    // WHEN: Rotating vector
    JSBSim::FGColumnVector3 result = rot_z * v;

    // THEN: Should get rotated vector
    // For a 90 degree rotation around Z: (1,0,0) -> (0,-1,0)
    TS_ASSERT_DELTA(result(1), 0.0, 1E-10);
    TS_ASSERT_DELTA(result(2), -1.0, 1E-10);
    TS_ASSERT_DELTA(result(3), 0.0, 1E-10);
  }

  // ============ Tests for Determinant Edge Cases ============

  void testDeterminantComplex() {
    // GIVEN: A matrix with known determinant
    JSBSim::FGMatrix33 m(2.0, 3.0, 1.0,
                          4.0, 1.0, 2.0,
                          1.0, 5.0, 3.0);

    // Compute determinant manually:
    // det = 2*(1*3 - 2*5) - 3*(4*3 - 2*1) + 1*(4*5 - 1*1)
    //     = 2*(3 - 10) - 3*(12 - 2) + 1*(20 - 1)
    //     = 2*(-7) - 3*(10) + 1*(19)
    //     = -14 - 30 + 19 = -25
    double expected_det = -25.0;

    // THEN: Determinant should match
    TS_ASSERT_DELTA(m.Determinant(), expected_det, 1E-10);
  }

  // ============ Tests for Transpose Operations ============

  void testTransposeSymmetricMatrix() {
    // GIVEN: A symmetric matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          2.0, 4.0, 5.0,
                          3.0, 5.0, 6.0);

    // WHEN: Transposing
    JSBSim::FGMatrix33 mT = m.Transposed();

    // THEN: Transpose should equal original (for symmetric matrix)
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(mT(i, j), m(i, j));
  }

  void testInPlaceTranspose() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Calling in-place transpose
    m.T();

    // THEN: Matrix should be transposed
    TS_ASSERT_EQUALS(m(1, 1), 1.0);
    TS_ASSERT_EQUALS(m(1, 2), 4.0);
    TS_ASSERT_EQUALS(m(1, 3), 7.0);
    TS_ASSERT_EQUALS(m(2, 1), 2.0);
    TS_ASSERT_EQUALS(m(2, 2), 5.0);
    TS_ASSERT_EQUALS(m(2, 3), 8.0);
    TS_ASSERT_EQUALS(m(3, 1), 3.0);
    TS_ASSERT_EQUALS(m(3, 2), 6.0);
    TS_ASSERT_EQUALS(m(3, 3), 9.0);

    // Transpose again should give original
    m.T();
    TS_ASSERT_EQUALS(m(1, 1), 1.0);
    TS_ASSERT_EQUALS(m(1, 2), 2.0);
    TS_ASSERT_EQUALS(m(1, 3), 3.0);
    TS_ASSERT_EQUALS(m(2, 1), 4.0);
    TS_ASSERT_EQUALS(m(2, 2), 5.0);
    TS_ASSERT_EQUALS(m(2, 3), 6.0);
    TS_ASSERT_EQUALS(m(3, 1), 7.0);
    TS_ASSERT_EQUALS(m(3, 2), 8.0);
    TS_ASSERT_EQUALS(m(3, 3), 9.0);
  }

  // ============ Tests for Arithmetic Operations ============

  void testMatrixSubtractionNonTrivial() {
    // GIVEN: Two matrices
    JSBSim::FGMatrix33 m1(5.0, 7.0, 9.0,
                           3.0, 2.0, 8.0,
                           1.0, 4.0, 6.0);
    JSBSim::FGMatrix33 m2(1.0, 2.0, 3.0,
                           4.0, 5.0, 6.0,
                           7.0, 8.0, 9.0);

    // WHEN: Subtracting
    JSBSim::FGMatrix33 diff = m1 - m2;

    // THEN: Result should be element-wise difference
    TS_ASSERT_EQUALS(diff(1, 1), 4.0);
    TS_ASSERT_EQUALS(diff(1, 2), 5.0);
    TS_ASSERT_EQUALS(diff(1, 3), 6.0);
    TS_ASSERT_EQUALS(diff(2, 1), -1.0);
    TS_ASSERT_EQUALS(diff(2, 2), -3.0);
    TS_ASSERT_EQUALS(diff(2, 3), 2.0);
    TS_ASSERT_EQUALS(diff(3, 1), -6.0);
    TS_ASSERT_EQUALS(diff(3, 2), -4.0);
    TS_ASSERT_EQUALS(diff(3, 3), -3.0);
  }

  void testMatrixScalarMultiplicationNegative() {
    // GIVEN: A matrix and negative scalar
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Multiplying by negative scalar
    JSBSim::FGMatrix33 result = m * (-2.5);

    // THEN: All elements should be scaled
    TS_ASSERT_EQUALS(result(1, 1), -2.5);
    TS_ASSERT_EQUALS(result(1, 2), -5.0);
    TS_ASSERT_EQUALS(result(1, 3), -7.5);
    TS_ASSERT_EQUALS(result(2, 1), -10.0);
    TS_ASSERT_EQUALS(result(2, 2), -12.5);
    TS_ASSERT_EQUALS(result(2, 3), -15.0);
    TS_ASSERT_EQUALS(result(3, 1), -17.5);
    TS_ASSERT_EQUALS(result(3, 2), -20.0);
    TS_ASSERT_EQUALS(result(3, 3), -22.5);
  }

  void testScalarMatrixMultiplication() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Multiplying scalar on left (uses friend function)
    JSBSim::FGMatrix33 result = 3.0 * m;

    // THEN: Result should be same as m * 3.0
    TS_ASSERT_EQUALS(result(1, 1), 3.0);
    TS_ASSERT_EQUALS(result(1, 2), 6.0);
    TS_ASSERT_EQUALS(result(1, 3), 9.0);
    TS_ASSERT_EQUALS(result(2, 1), 12.0);
    TS_ASSERT_EQUALS(result(2, 2), 15.0);
    TS_ASSERT_EQUALS(result(2, 3), 18.0);
    TS_ASSERT_EQUALS(result(3, 1), 21.0);
    TS_ASSERT_EQUALS(result(3, 2), 24.0);
    TS_ASSERT_EQUALS(result(3, 3), 27.0);
  }

  // ============ Tests for Quaternion Conversion ============

  void testQuaternionConversionIdentity() {
    // GIVEN: Identity matrix
    JSBSim::FGMatrix33 eye(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0);

    // WHEN: Converting to quaternion
    JSBSim::FGQuaternion q = eye.GetQuaternion();

    // THEN: Should get identity quaternion [1, 0, 0, 0]
    TS_ASSERT_DELTA(q(1), 1.0, 1E-8);
    TS_ASSERT_DELTA(q(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(3), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);
  }

  void testQuaternionConversionRotationZ() {
    // GIVEN: 90 degree rotation around Z axis
    double psi = M_PI / 2.0;
    JSBSim::FGMatrix33 m(cos(psi), sin(psi), 0.0,
                          -sin(psi), cos(psi), 0.0,
                          0.0, 0.0, 1.0);

    // WHEN: Converting to quaternion
    JSBSim::FGQuaternion q = m.GetQuaternion();

    // THEN: Should match expected quaternion for Z rotation
    TS_ASSERT_DELTA(q(1), cos(psi/2.0), 1E-8);
    TS_ASSERT_DELTA(q(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(3), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(4), sin(psi/2.0), 1E-8);
  }

  // ============ Tests for Euler Angle Conversion ============

  void testEulerAnglesIdentity() {
    // GIVEN: Identity matrix
    JSBSim::FGMatrix33 eye(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0);

    // WHEN: Getting Euler angles
    JSBSim::FGColumnVector3 angles = eye.GetEuler();

    // THEN: All angles should be zero
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
  }

  void testEulerAnglesSmallRotations() {
    // GIVEN: Small rotation angles
    double phi = 0.1;     // 5.7 degrees
    double theta = 0.2;   // 11.5 degrees
    double psi = 0.3;     // 17.2 degrees

    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta), sth = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);

    const JSBSim::FGMatrix33 m_phi(1.0, 0.0, 0.0,
                                   0.0, cphi, sphi,
                                   0.0, -sphi, cphi);
    const JSBSim::FGMatrix33 m_th(cth, 0.0, -sth,
                                  0.0, 1.0, 0.0,
                                  sth, 0.0, cth);
    const JSBSim::FGMatrix33 m_psi(cpsi, spsi, 0.0,
                                   -spsi, cpsi, 0.0,
                                   0.0, 0.0, 1.0);

    JSBSim::FGMatrix33 m = m_phi * m_th * m_psi;
    JSBSim::FGColumnVector3 angles = m.GetEuler();

    // THEN: Should recover original angles
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
  }

  // ============ Tests for Matrix Norms and Properties ============

  void testOrthogonalMatrixProperties() {
    // GIVEN: An orthogonal matrix (rotation)
    double angle = M_PI / 4.0;  // 45 degrees
    JSBSim::FGMatrix33 rot(cos(angle), sin(angle), 0.0,
                            -sin(angle), cos(angle), 0.0,
                            0.0, 0.0, 1.0);

    // THEN: Determinant should be 1 for proper rotation
    TS_ASSERT_DELTA(rot.Determinant(), 1.0, 1E-10);

    // AND: Transpose should equal inverse
    JSBSim::FGMatrix33 rotT = rot.Transposed();
    JSBSim::FGMatrix33 rotInv = rot.Inverse();

    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(rotT(i, j), rotInv(i, j), 1E-10);
  }

  // ============ Tests for Special Matrix Cases ============

  void testAntiSymmetricMatrix() {
    // GIVEN: An anti-symmetric matrix (A^T = -A)
    JSBSim::FGMatrix33 m(0.0, -3.0, 2.0,
                          3.0, 0.0, -1.0,
                          -2.0, 1.0, 0.0);

    // WHEN: Computing transpose
    JSBSim::FGMatrix33 mT = m.Transposed();

    // THEN: Transpose should equal negative of original
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(mT(i, j), -m(i, j), 1E-10);

    // AND: Determinant should be zero (for 3x3 anti-symmetric)
    TS_ASSERT_DELTA(m.Determinant(), 0.0, 1E-10);
  }

  void testDiagonalMatrixOperations() {
    // GIVEN: Two diagonal matrices
    JSBSim::FGMatrix33 d1(2.0, 0.0, 0.0,
                           0.0, 3.0, 0.0,
                           0.0, 0.0, 4.0);
    JSBSim::FGMatrix33 d2(5.0, 0.0, 0.0,
                           0.0, 6.0, 0.0,
                           0.0, 0.0, 7.0);

    // WHEN: Multiplying
    JSBSim::FGMatrix33 product = d1 * d2;

    // THEN: Result should be diagonal with products
    TS_ASSERT_EQUALS(product(1, 1), 10.0);
    TS_ASSERT_EQUALS(product(2, 2), 18.0);
    TS_ASSERT_EQUALS(product(3, 3), 28.0);
    TS_ASSERT_EQUALS(product(1, 2), 0.0);
    TS_ASSERT_EQUALS(product(1, 3), 0.0);
    TS_ASSERT_EQUALS(product(2, 1), 0.0);
    TS_ASSERT_EQUALS(product(2, 3), 0.0);
    TS_ASSERT_EQUALS(product(3, 1), 0.0);
    TS_ASSERT_EQUALS(product(3, 2), 0.0);
  }

  // ============ Tests for Copy and Assignment ============

  void testCopyConstructor() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m1(1.0, 2.0, 3.0,
                           4.0, 5.0, 6.0,
                           7.0, 8.0, 9.0);

    // WHEN: Copying via copy constructor
    JSBSim::FGMatrix33 m2(m1);

    // THEN: Matrices should be equal
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(m2(i, j), m1(i, j));

    // AND: Modifying copy should not affect original
    m2(2, 2) = 99.0;
    TS_ASSERT_EQUALS(m1(2, 2), 5.0);
    TS_ASSERT_EQUALS(m2(2, 2), 99.0);
  }

  void testInitializerListConstructor() {
    // GIVEN: An initializer list
    JSBSim::FGMatrix33 m = {1.0, 2.0, 3.0,
                             4.0, 5.0, 6.0,
                             7.0, 8.0, 9.0};

    // THEN: Matrix should be initialized correctly
    TS_ASSERT_EQUALS(m(1, 1), 1.0);
    TS_ASSERT_EQUALS(m(1, 2), 2.0);
    TS_ASSERT_EQUALS(m(1, 3), 3.0);
    TS_ASSERT_EQUALS(m(2, 1), 4.0);
    TS_ASSERT_EQUALS(m(2, 2), 5.0);
    TS_ASSERT_EQUALS(m(2, 3), 6.0);
    TS_ASSERT_EQUALS(m(3, 1), 7.0);
    TS_ASSERT_EQUALS(m(3, 2), 8.0);
    TS_ASSERT_EQUALS(m(3, 3), 9.0);
  }

  void testPartialInitializerList() {
    // GIVEN: A partial initializer list (less than 9 elements)
    JSBSim::FGMatrix33 m;
    m = {1.0, 2.0, 3.0, 4.0, 5.0};  // Only 5 elements

    // THEN: First 5 elements should be set, rest unchanged
    TS_ASSERT_EQUALS(m(1, 1), 1.0);
    TS_ASSERT_EQUALS(m(1, 2), 2.0);
    TS_ASSERT_EQUALS(m(1, 3), 3.0);
    TS_ASSERT_EQUALS(m(2, 1), 4.0);
    TS_ASSERT_EQUALS(m(2, 2), 5.0);
  }

  // ============ Tests for Rows and Cols Methods ============

  void testRowsAndCols() {
    // GIVEN: Any matrix
    JSBSim::FGMatrix33 m;

    // THEN: Rows and Cols should always be 3
    TS_ASSERT_EQUALS(m.Rows(), 3);
    TS_ASSERT_EQUALS(m.Cols(), 3);

    // Even after modifications
    m(1, 1) = 99.0;
    TS_ASSERT_EQUALS(m.Rows(), 3);
    TS_ASSERT_EQUALS(m.Cols(), 3);
  }

  // ============ Tests for Complex Determinant Cases ============

  void testDeterminantWithNegativeValues() {
    // GIVEN: Matrix with negative values
    JSBSim::FGMatrix33 m(-2.0, 3.0, -1.0,
                          4.0, -5.0, 6.0,
                          -7.0, 8.0, -9.0);

    // WHEN: Computing determinant
    double det = m.Determinant();

    // Verify it's computed correctly (non-zero)
    TS_ASSERT(det != 0.0);
    TS_ASSERT(m.Invertible());
  }

  // ============ Additional Inverse Tests ============

  void testInverseWithScaling() {
    // GIVEN: A scaled identity matrix
    JSBSim::FGMatrix33 m(5.0, 0.0, 0.0,
                          0.0, 5.0, 0.0,
                          0.0, 0.0, 5.0);

    // WHEN: Computing inverse
    JSBSim::FGMatrix33 inv = m.Inverse();

    // THEN: Inverse should be 1/5 times identity
    TS_ASSERT_DELTA(inv(1, 1), 0.2, 1E-10);
    TS_ASSERT_DELTA(inv(2, 2), 0.2, 1E-10);
    TS_ASSERT_DELTA(inv(3, 3), 0.2, 1E-10);

    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        if (i != j)
          TS_ASSERT_DELTA(inv(i, j), 0.0, 1E-10);
  }

  void testInverseComplexMatrix() {
    // GIVEN: A complex invertible matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 1.0,
                          0.0, 1.0, 2.0,
                          1.0, 1.0, 1.0);

    // WHEN: Computing inverse
    JSBSim::FGMatrix33 inv = m.Inverse();

    // THEN: m * inv = I and inv * m = I
    JSBSim::FGMatrix33 product1 = m * inv;
    JSBSim::FGMatrix33 product2 = inv * m;

    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT_DELTA(product1(i, j), (i == j ? 1.0 : 0.0), 1E-10);
        TS_ASSERT_DELTA(product2(i, j), (i == j ? 1.0 : 0.0), 1E-10);
      }
  }

  // ============ Rotation Composition Tests ============

  void testRotationCompositionXY() {
    // GIVEN: Rotations around X and Y axes
    double phi = M_PI / 6.0;   // 30 degrees
    double theta = M_PI / 4.0; // 45 degrees

    JSBSim::FGMatrix33 Rx(1.0, 0.0, 0.0,
                           0.0, cos(phi), sin(phi),
                           0.0, -sin(phi), cos(phi));
    JSBSim::FGMatrix33 Ry(cos(theta), 0.0, -sin(theta),
                           0.0, 1.0, 0.0,
                           sin(theta), 0.0, cos(theta));

    // WHEN: Composing rotations
    JSBSim::FGMatrix33 Rxy = Rx * Ry;
    JSBSim::FGMatrix33 Ryx = Ry * Rx;

    // THEN: Both should be orthogonal (det = 1)
    TS_ASSERT_DELTA(Rxy.Determinant(), 1.0, 1E-10);
    TS_ASSERT_DELTA(Ryx.Determinant(), 1.0, 1E-10);

    // But order matters (rotation is non-commutative)
    bool matrices_differ = false;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        if (fabs(Rxy(i, j) - Ryx(i, j)) > 1E-10)
          matrices_differ = true;
    TS_ASSERT(matrices_differ);
  }

  void testRotationCompositionAllAxes() {
    // GIVEN: Rotations around all three axes (Euler angles)
    double phi = 0.2;   // Roll
    double theta = 0.3; // Pitch
    double psi = 0.4;   // Yaw

    JSBSim::FGMatrix33 Rx(1.0, 0.0, 0.0,
                           0.0, cos(phi), sin(phi),
                           0.0, -sin(phi), cos(phi));
    JSBSim::FGMatrix33 Ry(cos(theta), 0.0, -sin(theta),
                           0.0, 1.0, 0.0,
                           sin(theta), 0.0, cos(theta));
    JSBSim::FGMatrix33 Rz(cos(psi), sin(psi), 0.0,
                           -sin(psi), cos(psi), 0.0,
                           0.0, 0.0, 1.0);

    // WHEN: Composing all rotations
    JSBSim::FGMatrix33 R = Rx * Ry * Rz;

    // THEN: Should be orthogonal and extract same angles
    TS_ASSERT_DELTA(R.Determinant(), 1.0, 1E-10);

    JSBSim::FGColumnVector3 angles = R.GetEuler();
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), psi, 1E-8);
  }

  void testInverseOfInverse() {
    // GIVEN: An invertible matrix
    JSBSim::FGMatrix33 m(3.0, 1.0, 2.0,
                          1.0, 4.0, 1.0,
                          2.0, 1.0, 5.0);

    // WHEN: Computing inverse of inverse
    JSBSim::FGMatrix33 inv = m.Inverse();
    JSBSim::FGMatrix33 inv_inv = inv.Inverse();

    // THEN: Should equal original
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(inv_inv(i, j), m(i, j), 1E-10);
  }

  void testDeterminantOfProduct() {
    // GIVEN: Two matrices
    JSBSim::FGMatrix33 A(2.0, 1.0, 0.0,
                          1.0, 3.0, 1.0,
                          0.0, 1.0, 2.0);
    JSBSim::FGMatrix33 B(1.0, 2.0, 1.0,
                          0.0, 1.0, 2.0,
                          1.0, 0.0, 1.0);

    // WHEN: Computing determinants
    double detA = A.Determinant();
    double detB = B.Determinant();
    double detAB = (A * B).Determinant();

    // THEN: det(AB) = det(A) * det(B)
    TS_ASSERT_DELTA(detAB, detA * detB, 1E-10);
  }

  void testDeterminantOfTranspose() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(2.0, 3.0, 1.0,
                          4.0, 1.0, 2.0,
                          1.0, 5.0, 3.0);

    // THEN: det(M) = det(M^T)
    TS_ASSERT_DELTA(m.Determinant(), m.Transposed().Determinant(), 1E-10);
  }

  void testDeterminantOfInverse() {
    // GIVEN: An invertible matrix
    JSBSim::FGMatrix33 m(3.0, 1.0, 2.0,
                          1.0, 4.0, 1.0,
                          2.0, 1.0, 5.0);

    // THEN: det(M^-1) = 1/det(M)
    double det = m.Determinant();
    double det_inv = m.Inverse().Determinant();
    TS_ASSERT_DELTA(det_inv, 1.0 / det, 1E-10);
  }

  void testTransposeOfProduct() {
    // GIVEN: Two matrices
    JSBSim::FGMatrix33 A(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);
    JSBSim::FGMatrix33 B(9.0, 8.0, 7.0,
                          6.0, 5.0, 4.0,
                          3.0, 2.0, 1.0);

    // WHEN: Computing transposes
    JSBSim::FGMatrix33 AB_T = (A * B).Transposed();
    JSBSim::FGMatrix33 BT_AT = B.Transposed() * A.Transposed();

    // THEN: (AB)^T = B^T * A^T
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(AB_T(i, j), BT_AT(i, j), 1E-10);
  }

  void testVectorTransformationChain() {
    // GIVEN: Multiple rotation matrices and a vector
    double angle1 = M_PI / 3.0;
    double angle2 = M_PI / 4.0;

    JSBSim::FGMatrix33 R1(cos(angle1), sin(angle1), 0.0,
                           -sin(angle1), cos(angle1), 0.0,
                           0.0, 0.0, 1.0);
    JSBSim::FGMatrix33 R2(cos(angle2), sin(angle2), 0.0,
                           -sin(angle2), cos(angle2), 0.0,
                           0.0, 0.0, 1.0);
    JSBSim::FGColumnVector3 v(1.0, 0.0, 0.0);

    // WHEN: Applying transformations
    JSBSim::FGColumnVector3 v_R1_R2 = R1 * (R2 * v);
    JSBSim::FGColumnVector3 v_R1R2 = (R1 * R2) * v;

    // THEN: Should be equal (associativity)
    TS_ASSERT_DELTA(v_R1_R2(1), v_R1R2(1), 1E-10);
    TS_ASSERT_DELTA(v_R1_R2(2), v_R1R2(2), 1E-10);
    TS_ASSERT_DELTA(v_R1_R2(3), v_R1R2(3), 1E-10);
  }

  void testOrthogonalMatrixInverseEqualsTranspose() {
    // GIVEN: Multiple rotation matrices
    double angles[] = {0.0, M_PI/6, M_PI/4, M_PI/3, M_PI/2};

    for (double angle : angles) {
      JSBSim::FGMatrix33 R(cos(angle), sin(angle), 0.0,
                            -sin(angle), cos(angle), 0.0,
                            0.0, 0.0, 1.0);

      JSBSim::FGMatrix33 RT = R.Transposed();
      JSBSim::FGMatrix33 Rinv = R.Inverse();

      for (int i = 1; i <= 3; i++)
        for (int j = 1; j <= 3; j++)
          TS_ASSERT_DELTA(RT(i, j), Rinv(i, j), 1E-10);
    }
  }

  void testNearSingularMatrix() {
    // GIVEN: A nearly singular matrix (small determinant)
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0001);  // Almost singular

    // THEN: Should still be invertible but close to singular
    double det = m.Determinant();
    TS_ASSERT(fabs(det) < 0.01);  // Very small determinant
    TS_ASSERT(m.Invertible());
  }

  void testMatrixMultiplicationAssociativity() {
    // GIVEN: Three matrices
    JSBSim::FGMatrix33 A(1.0, 2.0, 0.0,
                          0.0, 1.0, 3.0,
                          1.0, 0.0, 1.0);
    JSBSim::FGMatrix33 B(2.0, 0.0, 1.0,
                          1.0, 1.0, 0.0,
                          0.0, 2.0, 1.0);
    JSBSim::FGMatrix33 C(1.0, 1.0, 1.0,
                          2.0, 1.0, 0.0,
                          0.0, 1.0, 2.0);

    // WHEN: Computing products in different orders
    JSBSim::FGMatrix33 AB_C = (A * B) * C;
    JSBSim::FGMatrix33 A_BC = A * (B * C);

    // THEN: (AB)C = A(BC) (associativity)
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(AB_C(i, j), A_BC(i, j), 1E-10);
  }

  void testMatrixAdditionCommutativity() {
    // GIVEN: Two matrices
    JSBSim::FGMatrix33 A(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);
    JSBSim::FGMatrix33 B(9.0, 8.0, 7.0,
                          6.0, 5.0, 4.0,
                          3.0, 2.0, 1.0);

    // WHEN: Computing A+B and B+A
    JSBSim::FGMatrix33 AB = A + B;
    JSBSim::FGMatrix33 BA = B + A;

    // THEN: A+B = B+A (commutativity)
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(AB(i, j), BA(i, j));
  }

  void testMatrixScalarDistributivity() {
    // GIVEN: A matrix and scalars
    JSBSim::FGMatrix33 A(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);
    double a = 3.0, b = 2.0;

    // WHEN: Computing (a+b)*A and a*A + b*A
    JSBSim::FGMatrix33 left = A * (a + b);
    JSBSim::FGMatrix33 right = A * a + A * b;

    // THEN: (a+b)*A = a*A + b*A
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(left(i, j), right(i, j), 1E-10);
  }

  void testRotationPreservesVectorMagnitude() {
    // GIVEN: A rotation matrix and various vectors
    double angle = M_PI / 5.0;
    JSBSim::FGMatrix33 R(cos(angle), sin(angle), 0.0,
                          -sin(angle), cos(angle), 0.0,
                          0.0, 0.0, 1.0);

    JSBSim::FGColumnVector3 vectors[] = {
      JSBSim::FGColumnVector3(1.0, 0.0, 0.0),
      JSBSim::FGColumnVector3(0.0, 1.0, 0.0),
      JSBSim::FGColumnVector3(0.0, 0.0, 1.0),
      JSBSim::FGColumnVector3(1.0, 1.0, 1.0),
      JSBSim::FGColumnVector3(3.0, 4.0, 0.0)
    };

    for (const auto& v : vectors) {
      JSBSim::FGColumnVector3 Rv = R * v;
      double mag_v = v.Magnitude();
      double mag_Rv = Rv.Magnitude();
      TS_ASSERT_DELTA(mag_v, mag_Rv, 1E-10);
    }
  }

  void testReflectionMatrix() {
    // GIVEN: A reflection matrix (det = -1)
    JSBSim::FGMatrix33 reflect(-1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0);

    // THEN: Determinant should be -1
    TS_ASSERT_EQUALS(reflect.Determinant(), -1.0);

    // AND: Applying twice gives identity
    JSBSim::FGMatrix33 reflect_twice = reflect * reflect;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(reflect_twice(i, j), (i == j ? 1.0 : 0.0), 1E-10);
  }

  void testScalingMatrix() {
    // GIVEN: A scaling matrix
    double sx = 2.0, sy = 3.0, sz = 4.0;
    JSBSim::FGMatrix33 scale(sx, 0.0, 0.0,
                              0.0, sy, 0.0,
                              0.0, 0.0, sz);

    JSBSim::FGColumnVector3 v(1.0, 1.0, 1.0);

    // WHEN: Scaling the vector
    JSBSim::FGColumnVector3 sv = scale * v;

    // THEN: Each component should be scaled
    TS_ASSERT_EQUALS(sv(1), sx);
    TS_ASSERT_EQUALS(sv(2), sy);
    TS_ASSERT_EQUALS(sv(3), sz);

    // AND: Determinant should be product of scale factors
    TS_ASSERT_EQUALS(scale.Determinant(), sx * sy * sz);
  }

  void testShearMatrix() {
    // GIVEN: A shear matrix (shear in X based on Y)
    double k = 2.0;
    JSBSim::FGMatrix33 shear(1.0, k, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0);

    JSBSim::FGColumnVector3 v(0.0, 1.0, 0.0);

    // WHEN: Applying shear
    JSBSim::FGColumnVector3 sv = shear * v;

    // THEN: X is sheared by k*Y
    TS_ASSERT_EQUALS(sv(1), k);
    TS_ASSERT_EQUALS(sv(2), 1.0);
    TS_ASSERT_EQUALS(sv(3), 0.0);

    // AND: Determinant should be 1 (volume preserving)
    TS_ASSERT_EQUALS(shear.Determinant(), 1.0);
  }

  void testNegativeMatrix() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Computing negative
    JSBSim::FGMatrix33 neg = -1.0 * m;

    // THEN: Each element should be negated
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(neg(i, j), -m(i, j));
  }

  void testMatrixWithSmallValues() {
    // GIVEN: A matrix with small values
    double eps = 1e-10;
    JSBSim::FGMatrix33 m(eps, 0.0, 0.0,
                          0.0, eps, 0.0,
                          0.0, 0.0, eps);

    // THEN: Determinant should be eps^3
    TS_ASSERT_DELTA(m.Determinant(), eps * eps * eps, 1E-40);

    // AND: Should be invertible
    TS_ASSERT(m.Invertible());

    // Inverse should have 1/eps on diagonal
    JSBSim::FGMatrix33 inv = m.Inverse();
    TS_ASSERT_DELTA(inv(1, 1), 1.0 / eps, 1.0);
    TS_ASSERT_DELTA(inv(2, 2), 1.0 / eps, 1.0);
    TS_ASSERT_DELTA(inv(3, 3), 1.0 / eps, 1.0);
  }

  void testMatrixWithLargeValues() {
    // GIVEN: A matrix with large values
    double big = 1e10;
    JSBSim::FGMatrix33 m(big, 0.0, 0.0,
                          0.0, big, 0.0,
                          0.0, 0.0, big);

    // THEN: Determinant should be big^3
    TS_ASSERT_DELTA(m.Determinant(), big * big * big, 1E20);

    // AND: Inverse should have 1/big on diagonal
    JSBSim::FGMatrix33 inv = m.Inverse();
    TS_ASSERT_DELTA(inv(1, 1), 1.0 / big, 1E-20);
    TS_ASSERT_DELTA(inv(2, 2), 1.0 / big, 1E-20);
    TS_ASSERT_DELTA(inv(3, 3), 1.0 / big, 1E-20);
  }

  void testCrossProductMatrix() {
    // GIVEN: A vector representing angular velocity
    JSBSim::FGColumnVector3 omega(1.0, 2.0, 3.0);

    // Create skew-symmetric cross product matrix [omega]×
    JSBSim::FGMatrix33 omega_cross(0.0, -omega(3), omega(2),
                                    omega(3), 0.0, -omega(1),
                                    -omega(2), omega(1), 0.0);

    // THEN: omega_cross should be skew-symmetric
    TS_ASSERT_DELTA(omega_cross(1, 2), -omega_cross(2, 1), 1E-10);
    TS_ASSERT_DELTA(omega_cross(1, 3), -omega_cross(3, 1), 1E-10);
    TS_ASSERT_DELTA(omega_cross(2, 3), -omega_cross(3, 2), 1E-10);

    // AND: Diagonal should be zero
    TS_ASSERT_EQUALS(omega_cross(1, 1), 0.0);
    TS_ASSERT_EQUALS(omega_cross(2, 2), 0.0);
    TS_ASSERT_EQUALS(omega_cross(3, 3), 0.0);
  }

  void testRotation180Degrees() {
    // GIVEN: 180 degree rotations
    // Rotation around Z by 180 degrees
    JSBSim::FGMatrix33 Rz180(cos(M_PI), sin(M_PI), 0.0,
                              -sin(M_PI), cos(M_PI), 0.0,
                              0.0, 0.0, 1.0);

    // THEN: Should be orthogonal with det = 1
    TS_ASSERT_DELTA(Rz180.Determinant(), 1.0, 1E-10);

    // AND: Rotating twice should give identity
    JSBSim::FGMatrix33 Rz360 = Rz180 * Rz180;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(Rz360(i, j), (i == j ? 1.0 : 0.0), 1E-10);
  }

  void testMatrixTrace() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // Compute trace (sum of diagonal)
    double trace = m(1, 1) + m(2, 2) + m(3, 3);

    // THEN: Trace should be 15
    TS_ASSERT_EQUALS(trace, 15.0);

    // AND: For rotation matrix, trace = 1 + 2*cos(theta)
    double angle = M_PI / 4.0;
    JSBSim::FGMatrix33 R(cos(angle), sin(angle), 0.0,
                          -sin(angle), cos(angle), 0.0,
                          0.0, 0.0, 1.0);
    double trace_R = R(1, 1) + R(2, 2) + R(3, 3);
    TS_ASSERT_DELTA(trace_R, 1.0 + 2.0 * cos(angle), 1E-10);
  }

  void testIdentityMatrixProperties() {
    // GIVEN: Identity matrix
    JSBSim::FGMatrix33 I(1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0);

    // THEN: I * I = I
    JSBSim::FGMatrix33 I2 = I * I;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(I2(i, j), I(i, j));

    // AND: I^-1 = I
    JSBSim::FGMatrix33 Iinv = I.Inverse();
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(Iinv(i, j), I(i, j));

    // AND: det(I) = 1
    TS_ASSERT_EQUALS(I.Determinant(), 1.0);

    // AND: I^T = I
    JSBSim::FGMatrix33 IT = I.Transposed();
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(IT(i, j), I(i, j));
  }

  /***************************************************************************
   * Section: Additional Rotation Tests
   ***************************************************************************/

  void testRotationAroundXAxis() {
    // GIVEN: Rotation around X axis by 60 degrees
    double phi = M_PI / 3.0;
    JSBSim::FGMatrix33 Rx(1.0, 0.0, 0.0,
                           0.0, cos(phi), sin(phi),
                           0.0, -sin(phi), cos(phi));

    // THEN: Should be orthogonal
    TS_ASSERT_DELTA(Rx.Determinant(), 1.0, 1E-10);

    // AND: Transform unit Y to expected position
    JSBSim::FGColumnVector3 ey(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 result = Rx * ey;
    TS_ASSERT_DELTA(result(1), 0.0, 1E-10);
    TS_ASSERT_DELTA(result(2), cos(phi), 1E-10);
    TS_ASSERT_DELTA(result(3), -sin(phi), 1E-10);
  }

  void testRotationAroundYAxis() {
    // GIVEN: Rotation around Y axis by 45 degrees
    double theta = M_PI / 4.0;
    JSBSim::FGMatrix33 Ry(cos(theta), 0.0, -sin(theta),
                           0.0, 1.0, 0.0,
                           sin(theta), 0.0, cos(theta));

    // THEN: Transform unit X to expected position
    JSBSim::FGColumnVector3 ex(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 result = Ry * ex;
    TS_ASSERT_DELTA(result(1), cos(theta), 1E-10);
    TS_ASSERT_DELTA(result(2), 0.0, 1E-10);
    TS_ASSERT_DELTA(result(3), sin(theta), 1E-10);
  }

  void testRotation270DegreesAroundZ() {
    // GIVEN: 270 degree rotation around Z
    double psi = 3.0 * M_PI / 2.0;
    JSBSim::FGMatrix33 Rz(cos(psi), sin(psi), 0.0,
                           -sin(psi), cos(psi), 0.0,
                           0.0, 0.0, 1.0);

    // THEN: Transform unit X should give (0, 1, 0)
    JSBSim::FGColumnVector3 ex(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 result = Rz * ex;
    TS_ASSERT_DELTA(result(1), 0.0, 1E-10);
    TS_ASSERT_DELTA(result(2), 1.0, 1E-10);
    TS_ASSERT_DELTA(result(3), 0.0, 1E-10);
  }

  void testSmallAngleRotation() {
    // GIVEN: Very small rotation angle
    double angle = 1E-6;
    JSBSim::FGMatrix33 R(cos(angle), sin(angle), 0.0,
                          -sin(angle), cos(angle), 0.0,
                          0.0, 0.0, 1.0);

    // THEN: Should be very close to identity
    TS_ASSERT_DELTA(R(1, 1), 1.0, 1E-10);
    TS_ASSERT_DELTA(R(2, 2), 1.0, 1E-10);
    TS_ASSERT_DELTA(R(3, 3), 1.0, 1E-10);

    // AND: Still orthogonal
    TS_ASSERT_DELTA(R.Determinant(), 1.0, 1E-10);
  }

  /***************************************************************************
   * Section: Additional Euler Angle Edge Cases
   ***************************************************************************/

  void testEulerAnglesNegativeRoll() {
    // GIVEN: Negative roll angle
    double phi = -M_PI / 4.0;
    double cphi = cos(phi), sphi = sin(phi);
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0,
                          0.0, cphi, sphi,
                          0.0, -sphi, cphi);

    // WHEN: Getting Euler angles
    JSBSim::FGColumnVector3 angles = m.GetEuler();

    // THEN: Should recover original angle
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
    TS_ASSERT_DELTA(angles(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
  }

  void testEulerAnglesNegativePitch() {
    // GIVEN: Negative pitch angle
    double theta = -M_PI / 6.0;
    double cth = cos(theta), sth = sin(theta);
    JSBSim::FGMatrix33 m(cth, 0.0, -sth,
                          0.0, 1.0, 0.0,
                          sth, 0.0, cth);

    // WHEN: Getting Euler angles
    JSBSim::FGColumnVector3 angles = m.GetEuler();

    // THEN: Should recover original angle
    TS_ASSERT_DELTA(angles(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(angles(2), theta, 1E-8);
    TS_ASSERT_DELTA(angles(3), 0.0, 1E-8);
  }

  void testEulerAnglesLargePhi() {
    // GIVEN: Large roll near 180 degrees
    double phi = 170.0 * M_PI / 180.0;
    double cphi = cos(phi), sphi = sin(phi);
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0,
                          0.0, cphi, sphi,
                          0.0, -sphi, cphi);

    // WHEN: Getting Euler angles
    JSBSim::FGColumnVector3 angles = m.GetEuler();

    // THEN: Should recover original angle
    TS_ASSERT_DELTA(angles(1), phi, 1E-8);
  }

  /***************************************************************************
   * Section: Quaternion Conversion Edge Cases
   ***************************************************************************/

  void testQuaternionConversionRotationX() {
    // GIVEN: 45 degree rotation around X axis
    double phi = M_PI / 4.0;
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0,
                          0.0, cos(phi), sin(phi),
                          0.0, -sin(phi), cos(phi));

    // WHEN: Converting to quaternion
    JSBSim::FGQuaternion q = m.GetQuaternion();

    // THEN: Should match expected quaternion for X rotation
    TS_ASSERT_DELTA(q(1), cos(phi/2.0), 1E-8);
    TS_ASSERT_DELTA(q(2), sin(phi/2.0), 1E-8);
    TS_ASSERT_DELTA(q(3), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);
  }

  void testQuaternionConversionRotationY() {
    // GIVEN: 60 degree rotation around Y axis
    double theta = M_PI / 3.0;
    JSBSim::FGMatrix33 m(cos(theta), 0.0, -sin(theta),
                          0.0, 1.0, 0.0,
                          sin(theta), 0.0, cos(theta));

    // WHEN: Converting to quaternion
    JSBSim::FGQuaternion q = m.GetQuaternion();

    // THEN: Should match expected quaternion for Y rotation
    TS_ASSERT_DELTA(q(1), cos(theta/2.0), 1E-8);
    TS_ASSERT_DELTA(q(2), 0.0, 1E-8);
    TS_ASSERT_DELTA(q(3), sin(theta/2.0), 1E-8);
    TS_ASSERT_DELTA(q(4), 0.0, 1E-8);
  }

  void testQuaternion180DegreeRotation() {
    // GIVEN: 180 degree rotation around Z axis
    JSBSim::FGMatrix33 m(-1.0, 0.0, 0.0,
                          0.0, -1.0, 0.0,
                          0.0, 0.0, 1.0);

    // WHEN: Converting to quaternion
    JSBSim::FGQuaternion q = m.GetQuaternion();

    // THEN: Should be quaternion for 180 deg around Z
    // q = [cos(90), 0, 0, sin(90)] = [0, 0, 0, 1]
    TS_ASSERT_DELTA(q(1), 0.0, 1E-8);
    TS_ASSERT_DELTA(fabs(q(4)), 1.0, 1E-8);
  }

  /***************************************************************************
   * Section: Matrix Algebra Properties
   ***************************************************************************/

  void testDistributivityOverMatrixAddition() {
    // GIVEN: Three matrices
    JSBSim::FGMatrix33 A(1.0, 2.0, 0.0,
                          0.0, 1.0, 3.0,
                          1.0, 0.0, 1.0);
    JSBSim::FGMatrix33 B(2.0, 0.0, 1.0,
                          1.0, 1.0, 0.0,
                          0.0, 2.0, 1.0);
    JSBSim::FGMatrix33 C(1.0, 1.0, 1.0,
                          0.0, 1.0, 2.0,
                          1.0, 0.0, 1.0);

    // WHEN: Computing A * (B + C) and A*B + A*C
    JSBSim::FGMatrix33 left = A * (B + C);
    JSBSim::FGMatrix33 right = A * B + A * C;

    // THEN: Should be equal
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(left(i, j), right(i, j), 1E-10);
  }

  void testRightDistributivity() {
    // GIVEN: Three matrices
    JSBSim::FGMatrix33 A(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);
    JSBSim::FGMatrix33 B(9.0, 8.0, 7.0,
                          6.0, 5.0, 4.0,
                          3.0, 2.0, 1.0);
    JSBSim::FGMatrix33 C(1.0, 0.0, 1.0,
                          0.0, 1.0, 0.0,
                          1.0, 0.0, 1.0);

    // WHEN: Computing (A + B) * C and A*C + B*C
    JSBSim::FGMatrix33 left = (A + B) * C;
    JSBSim::FGMatrix33 right = A * C + B * C;

    // THEN: Should be equal
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(left(i, j), right(i, j), 1E-10);
  }

  void testScalarMultiplicationAssociativity() {
    // GIVEN: A matrix and scalars
    JSBSim::FGMatrix33 A(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);
    double a = 2.0, b = 3.0;

    // WHEN: Computing (a*b)*A and a*(b*A)
    JSBSim::FGMatrix33 left = A * (a * b);
    JSBSim::FGMatrix33 right = (A * b) * a;

    // THEN: Should be equal
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(left(i, j), right(i, j), 1E-10);
  }

  /***************************************************************************
   * Section: Special Transformation Matrices
   ***************************************************************************/

  void testPermutationMatrix() {
    // GIVEN: A permutation matrix (swaps X and Y)
    JSBSim::FGMatrix33 P(0.0, 1.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0);

    // THEN: Determinant should be -1 (odd permutation)
    TS_ASSERT_EQUALS(P.Determinant(), -1.0);

    // AND: P * P = I
    JSBSim::FGMatrix33 P2 = P * P;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(P2(i, j), (i == j ? 1.0 : 0.0), 1E-10);
  }

  void testProjectionMatrix() {
    // GIVEN: Projection onto XY plane
    JSBSim::FGMatrix33 proj(1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0);

    // THEN: proj * proj = proj (idempotent)
    JSBSim::FGMatrix33 proj2 = proj * proj;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(proj2(i, j), proj(i, j));

    // AND: Determinant should be 0
    TS_ASSERT_EQUALS(proj.Determinant(), 0.0);
  }

  void testOuterProductLikeMatrix() {
    // GIVEN: Matrix formed like outer product (rank 1)
    // v = [1, 2, 3], u = [1, 1, 1]
    // M = v * u^T
    JSBSim::FGMatrix33 M(1.0, 1.0, 1.0,
                          2.0, 2.0, 2.0,
                          3.0, 3.0, 3.0);

    // THEN: Should have rank 1 (det = 0)
    TS_ASSERT_DELTA(M.Determinant(), 0.0, 1E-10);
    TS_ASSERT(!M.Invertible());
  }

  /***************************************************************************
   * Section: Numerical Precision Tests
   ***************************************************************************/

  void testVerySmallDeterminant() {
    // GIVEN: Matrix with very small but non-zero determinant
    double eps = 1E-12;
    JSBSim::FGMatrix33 m(1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          eps, 0.0, eps);

    double det = m.Determinant();
    TS_ASSERT(fabs(det) > 0);
    TS_ASSERT(fabs(det) < 1E-10);
  }

  void testMatrixSubtractionToZero() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Subtracting from itself
    JSBSim::FGMatrix33 zero = m - m;

    // THEN: Should be zero matrix
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(zero(i, j), 0.0);
  }

  void testMultiplyByZeroScalar() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Multiplying by zero
    JSBSim::FGMatrix33 result = m * 0.0;

    // THEN: Should be zero matrix
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(result(i, j), 0.0);
  }

  /***************************************************************************
   * Section: Inertia Tensor Operations
   ***************************************************************************/

  void testSymmetricInertiaMatrix() {
    // GIVEN: A symmetric positive definite inertia tensor
    JSBSim::FGMatrix33 I(100.0, -20.0, -10.0,
                          -20.0, 200.0, -30.0,
                          -10.0, -30.0, 150.0);

    // THEN: Should be symmetric
    TS_ASSERT_EQUALS(I(1, 2), I(2, 1));
    TS_ASSERT_EQUALS(I(1, 3), I(3, 1));
    TS_ASSERT_EQUALS(I(2, 3), I(3, 2));

    // AND: Should be invertible (positive definite)
    TS_ASSERT(I.Invertible());
    double det = I.Determinant();
    TS_ASSERT(det > 0);

    // Inverse should also be symmetric
    JSBSim::FGMatrix33 Iinv = I.Inverse();
    TS_ASSERT_DELTA(Iinv(1, 2), Iinv(2, 1), 1E-10);
    TS_ASSERT_DELTA(Iinv(1, 3), Iinv(3, 1), 1E-10);
    TS_ASSERT_DELTA(Iinv(2, 3), Iinv(3, 2), 1E-10);
  }

  void testDiagonalInertiaMatrix() {
    // GIVEN: Diagonal inertia tensor (principal axes aligned)
    JSBSim::FGMatrix33 I(1000.0, 0.0, 0.0,
                          0.0, 2000.0, 0.0,
                          0.0, 0.0, 1500.0);

    // THEN: Inverse should have reciprocals on diagonal
    JSBSim::FGMatrix33 Iinv = I.Inverse();
    TS_ASSERT_DELTA(Iinv(1, 1), 1.0/1000.0, 1E-15);
    TS_ASSERT_DELTA(Iinv(2, 2), 1.0/2000.0, 1E-15);
    TS_ASSERT_DELTA(Iinv(3, 3), 1.0/1500.0, 1E-15);
  }

  /***************************************************************************
   * Section: Combined Transformation Tests
   ***************************************************************************/

  void testRotationThenScaling() {
    // GIVEN: Rotation then scaling
    double angle = M_PI / 6.0;
    JSBSim::FGMatrix33 R(cos(angle), sin(angle), 0.0,
                          -sin(angle), cos(angle), 0.0,
                          0.0, 0.0, 1.0);
    JSBSim::FGMatrix33 S(2.0, 0.0, 0.0,
                          0.0, 3.0, 0.0,
                          0.0, 0.0, 1.0);

    // WHEN: Composing transformations
    JSBSim::FGMatrix33 RS = R * S;
    JSBSim::FGMatrix33 SR = S * R;

    // THEN: det(RS) = det(R) * det(S) = 1 * 6 = 6
    TS_ASSERT_DELTA(RS.Determinant(), 6.0, 1E-10);
    TS_ASSERT_DELTA(SR.Determinant(), 6.0, 1E-10);

    // But the transformations are different
    bool differ = false;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        if (fabs(RS(i, j) - SR(i, j)) > 1E-10)
          differ = true;
    TS_ASSERT(differ);
  }

  void testMultipleRotationsInverse() {
    // GIVEN: Three successive rotations
    double a1 = 0.3, a2 = 0.5, a3 = 0.7;
    JSBSim::FGMatrix33 R1(cos(a1), sin(a1), 0.0,
                           -sin(a1), cos(a1), 0.0,
                           0.0, 0.0, 1.0);
    JSBSim::FGMatrix33 R2(1.0, 0.0, 0.0,
                           0.0, cos(a2), sin(a2),
                           0.0, -sin(a2), cos(a2));
    JSBSim::FGMatrix33 R3(cos(a3), 0.0, -sin(a3),
                           0.0, 1.0, 0.0,
                           sin(a3), 0.0, cos(a3));

    JSBSim::FGMatrix33 R = R1 * R2 * R3;

    // THEN: R^-1 = R3^T * R2^T * R1^T
    JSBSim::FGMatrix33 Rinv = R.Inverse();
    JSBSim::FGMatrix33 Rinv_expected = R3.Transposed() * R2.Transposed() * R1.Transposed();

    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(Rinv(i, j), Rinv_expected(i, j), 1E-10);
  }

  /***************************************************************************
   * Section: Stress Tests
   ***************************************************************************/

  void testManyMatrixMultiplications() {
    // GIVEN: A rotation matrix
    double angle = 0.1;
    JSBSim::FGMatrix33 R(cos(angle), sin(angle), 0.0,
                          -sin(angle), cos(angle), 0.0,
                          0.0, 0.0, 1.0);

    // WHEN: Multiplying many times
    JSBSim::FGMatrix33 result = R;
    for (int i = 0; i < 100; i++) {
      result = result * R;
    }

    // THEN: Should still be orthogonal
    TS_ASSERT_DELTA(result.Determinant(), 1.0, 1E-8);
  }

  void testManyTransposes() {
    // GIVEN: A matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // WHEN: Transposing many times (should alternate)
    JSBSim::FGMatrix33 result = m;
    for (int i = 0; i < 50; i++) {
      result.T();
      result.T();  // Two transposes = identity
    }

    // THEN: Should equal original
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(result(i, j), m(i, j));
  }

  void testManyInverses() {
    // GIVEN: An invertible matrix
    JSBSim::FGMatrix33 m(2.0, 1.0, 0.0,
                          1.0, 3.0, 1.0,
                          0.0, 1.0, 2.0);

    // WHEN: Taking inverse twice (should return to original)
    JSBSim::FGMatrix33 minv = m.Inverse();
    JSBSim::FGMatrix33 minvinv = minv.Inverse();

    // THEN: Should equal original
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(minvinv(i, j), m(i, j), 1E-10);
  }

  void testCompoundAssignmentOperators() {
    // GIVEN: Initial matrix
    JSBSim::FGMatrix33 m(1.0, 2.0, 3.0,
                          4.0, 5.0, 6.0,
                          7.0, 8.0, 9.0);

    // Test *= with scalar
    JSBSim::FGMatrix33 m2 = m;
    m2 *= 2.0;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(m2(i, j), m(i, j) * 2.0);

    // Test /= with scalar
    m2 /= 2.0;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(m2(i, j), m(i, j), 1E-10);

    // Test += with matrix
    JSBSim::FGMatrix33 m3 = m;
    m3 += m;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_EQUALS(m3(i, j), m(i, j) * 2.0);

    // Test -= with matrix
    m3 -= m;
    for (int i = 1; i <= 3; i++)
      for (int j = 1; j <= 3; j++)
        TS_ASSERT_DELTA(m3(i, j), m(i, j), 1E-10);
  }
};
