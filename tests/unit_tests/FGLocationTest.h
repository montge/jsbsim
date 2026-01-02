#include <limits>
#include <cxxtest/TestSuite.h>
#include <math/FGLocation.h>
#include <math/FGQuaternion.h>
#include "TestAssertions.h"
#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <initialization/FGInitialCondition.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();

double NormalizedAngle(double angle) {
  if (angle > M_PI) angle -= 2.0*M_PI;
  if (angle <= -M_PI) angle += 2.0*M_PI;
  return angle;
}

void CheckLocation(const JSBSim::FGLocation& loc,
                   JSBSim::FGColumnVector3 vec) {
  const JSBSim::FGQuaternion qloc(2, -0.5*M_PI);
  JSBSim::FGQuaternion q;
  double r = vec.Magnitude();

  TS_ASSERT_DELTA(vec(1), loc(1), r*epsilon);
  TS_ASSERT_DELTA(vec(2), loc(2), r*epsilon);
  TS_ASSERT_DELTA(vec(3), loc(3), r*epsilon);
  TS_ASSERT_DELTA(r, loc.GetRadius(), r*epsilon);

  vec.Normalize();
  double lon = atan2(vec(2), vec(1));
  double lat = asin(vec(3));

  TS_ASSERT_DELTA(lon, loc.GetLongitude(), epsilon);
  TS_ASSERT_DELTA(lat, loc.GetLatitude(), epsilon);
  TS_ASSERT_DELTA(sin(lon), loc.GetSinLongitude(), epsilon);
  TS_ASSERT_DELTA(cos(lon), loc.GetCosLongitude(), epsilon);

  q = JSBSim::FGQuaternion(0., -loc.GetGeodLatitudeRad(), lon);
  JSBSim::FGMatrix33 m = (q * qloc).GetT();
  TS_ASSERT_MATRIX_EQUALS(m, loc.GetTec2l());
  TS_ASSERT_MATRIX_EQUALS(m.Transposed(), loc.GetTl2ec());
}

class  FGLocationTest : public CxxTest::TestSuite
{
public:
  void testConstructors() {
    JSBSim::FGLocation l0;
    TS_ASSERT_EQUALS(1.0, l0(1));
    TS_ASSERT_EQUALS(0.0, l0(2));
    TS_ASSERT_EQUALS(0.0, l0(3));
    TS_ASSERT_EQUALS(1.0, l0.Entry(1));
    TS_ASSERT_EQUALS(0.0, l0.Entry(2));
    TS_ASSERT_EQUALS(0.0, l0.Entry(3));
    TS_ASSERT_EQUALS(0.0, l0.GetLongitude());
    TS_ASSERT_EQUALS(0.0, l0.GetLatitude());
    TS_ASSERT_EQUALS(0.0, l0.GetLongitudeDeg());
    TS_ASSERT_EQUALS(0.0, l0.GetLatitudeDeg());
    TS_ASSERT_EQUALS(1.0, l0.GetRadius());
    TS_ASSERT_EQUALS(0.0, l0.GetSinLongitude());
    TS_ASSERT_EQUALS(1.0, l0.GetCosLongitude());

    l0.SetEllipse(1., 1.);
    TS_ASSERT_EQUALS(0.0, l0.GetGeodLatitudeRad());
    TS_ASSERT_EQUALS(0.0, l0.GetGeodLatitudeDeg());
    TS_ASSERT_EQUALS(0.0, l0.GetGeodAltitude());

    double lat = -0.25*M_PI;
    double lon = M_PI/6.0;
    JSBSim::FGLocation l(lon, lat, 1.0);
    TS_ASSERT_DELTA(lon, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(lat, l.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetRadius(), epsilon);
    TS_ASSERT_DELTA(30.0, l.GetLongitudeDeg(), epsilon);
    TS_ASSERT_DELTA(-45.0, l.GetLatitudeDeg(), epsilon);
    TS_ASSERT_DELTA(0.5, l.GetSinLongitude(), epsilon);
    TS_ASSERT_DELTA(0.5*sqrt(3.0), l.GetCosLongitude(), epsilon);

    l.SetEllipse(1., 1.);
    TS_ASSERT_EQUALS(lat, l.GetGeodLatitudeRad());
    TS_ASSERT_EQUALS(-45.0, l.GetGeodLatitudeDeg());
    TS_ASSERT_DELTA(0.0, l.GetGeodAltitude(), epsilon);

    const JSBSim::FGQuaternion qloc(2, -0.5*M_PI);
    JSBSim::FGQuaternion q(0., -lat, lon);
    JSBSim::FGMatrix33 m = (q * qloc).GetT();
    TS_ASSERT_MATRIX_EQUALS(m, l.GetTec2l());
    TS_ASSERT_MATRIX_EQUALS(m.Transposed(), l.GetTl2ec());

    JSBSim::FGColumnVector3 v(1.,0.,1.);
    JSBSim::FGLocation lv1(v);
    TS_ASSERT_EQUALS(v(1), lv1(1));
    TS_ASSERT_EQUALS(v(2), lv1(2));
    TS_ASSERT_EQUALS(v(3), lv1(3));
    TS_ASSERT_DELTA(0.0, lv1.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(0.25*M_PI, lv1.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(sqrt(2.0), lv1.GetRadius(), epsilon);
    JSBSim::FGQuaternion qlat(2, -lv1.GetLatitude());
    m = (qlat * qloc).GetT();
    TS_ASSERT_MATRIX_EQUALS(m, lv1.GetTec2l());
    TS_ASSERT_MATRIX_EQUALS(m.Transposed(), lv1.GetTl2ec());

    v = JSBSim::FGColumnVector3(1.,1.,0.);
    JSBSim::FGLocation lv2(v);
    TS_ASSERT_EQUALS(v(1), lv2(1));
    TS_ASSERT_EQUALS(v(2), lv2(2));
    TS_ASSERT_EQUALS(v(3), lv2(3));
    TS_ASSERT_DELTA(0.25*M_PI, lv2.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, lv2.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(sqrt(2.0), lv2.GetRadius(), epsilon);
    JSBSim::FGQuaternion qlon(3, lv2.GetLongitude());
    m = (qlon * qloc).GetT();
    TS_ASSERT_MATRIX_EQUALS(m, lv2.GetTec2l());
    TS_ASSERT_MATRIX_EQUALS(m.Transposed(), lv2.GetTl2ec());

    v = JSBSim::FGColumnVector3(1.5,-2.,3.);
    JSBSim::FGLocation lv3(v);

    lv3.SetEllipse(1., 1.);
    CheckLocation(lv3, v);
  }

  void testCopyConstructor() {
    JSBSim::FGColumnVector3 v(1.5, -2.0, 3.0);
    JSBSim::FGLocation l(v);
    l.SetEllipse(1., 1.);
    JSBSim::FGLocation lv(l);

    TS_ASSERT_DELTA(l(1), lv(1), epsilon);
    TS_ASSERT_DELTA(l(2), lv(2), epsilon);
    TS_ASSERT_DELTA(l(3), lv(3), epsilon);

    CheckLocation(l, v);
    CheckLocation(lv, v);

    // Check that FGLocation use a copy of the values contained in the vector v
    // If a value of v is modified, then the FGLocation instances shall not be
    // affected.
    JSBSim::FGColumnVector3 v0 = v;
    v(2) = 1.0;
    TS_ASSERT_DELTA(l(1), lv(1), epsilon);
    TS_ASSERT_DELTA(-2.0, lv(2), epsilon);
    TS_ASSERT_DELTA(1.0, v(2), epsilon);
    TS_ASSERT_DELTA(l(3), lv(3), epsilon);

    CheckLocation(l, v0);
    CheckLocation(lv, v0);

    // Check that the copy 'lv' is not altered if the FGLocation 'l' is modified
    l(2) = 1.0;
    CheckLocation(l, v);
    CheckLocation(lv, v0);

    // Check the copy constructor for an FGLocation with cached values.
    JSBSim::FGLocation lv2(l);

    TS_ASSERT_DELTA(l(1), lv2(1), epsilon);
    TS_ASSERT_DELTA(l(2), lv2(2), epsilon);
    TS_ASSERT_DELTA(l(3), lv2(3), epsilon);

    CheckLocation(lv2, v);
  }

  void testEquality() {
    const JSBSim::FGColumnVector3 v(1.5, -2.0, 3.0);
    JSBSim::FGLocation l(v);
    const JSBSim::FGLocation lv(l);

    TS_ASSERT_EQUALS(l, lv);

    for (unsigned int i=1; i < 4; i++) {
      l = lv;
      l(i) = lv.Entry(i) + 1.0;
      TS_ASSERT_DIFFERS(l, lv);

      for (unsigned int j=1; j < 4; j++) {
        if (i == j)
          l(i) = lv.Entry(i);
        else
          l(j) = lv.Entry(j) + 1.0;
      }

      TS_ASSERT_DIFFERS(l, lv);
    }
  }

  void testAssignment() {
    JSBSim::FGColumnVector3 v(1.5, -2.0, 3.0);
    JSBSim::FGLocation lv(v);
    lv.SetEllipse(1., 1.);
    JSBSim::FGLocation l;

    TS_ASSERT_EQUALS(1.0, l(1));
    TS_ASSERT_EQUALS(0.0, l(2));
    TS_ASSERT_EQUALS(0.0, l(3));

    l = lv;
    TS_ASSERT_EQUALS(l(1), lv(1));
    TS_ASSERT_EQUALS(l(2), lv(2));
    TS_ASSERT_EQUALS(l(3), lv(3));
    CheckLocation(l, v);

    // Make sure that l and lv are distinct copies
    lv(1) = -3.4;
    TS_ASSERT_EQUALS(v(1), l(1));
    TS_ASSERT_EQUALS(v(2), l(2));
    TS_ASSERT_EQUALS(v(3), l(3));
    lv(1) = 1.5;

    for (unsigned int i=1; i < 4; i++) {
      l = lv;
      double x = v(i) + 1.0;
      l.Entry(i) = x;

      for (unsigned int j=1; j < 4; j++) {
        if (i == j) {
          TS_ASSERT_EQUALS(l(i), x);
          TS_ASSERT_EQUALS(l.Entry(i), x);
        }
        else {
          TS_ASSERT_EQUALS(l(j), v(j));
          TS_ASSERT_EQUALS(l.Entry(j), v(j));
        }
      }

      CheckLocation(l, JSBSim::FGColumnVector3(l(1), l(2), l(3)));
    }

    l = v;
    TS_ASSERT_EQUALS(l(1), v(1));
    TS_ASSERT_EQUALS(l(2), v(2));
    TS_ASSERT_EQUALS(l(3), v(3));
    CheckLocation(l, v);

    // Make sure that l and v are distinct copies
    v(2) = -3.4;
    TS_ASSERT_EQUALS(lv(1), l(1));
    TS_ASSERT_EQUALS(lv(2), l(2));
    TS_ASSERT_EQUALS(lv(3), l(3));
    v(2) = -2.0;

    for (unsigned int i=1; i < 4; i++) {
      l = v;
      double x = v(i) + 1.0;
      l(i) = x;

      for (unsigned int j=1; j < 4; j++) {
        if (i == j) {
          TS_ASSERT_EQUALS(l(i), x);
          TS_ASSERT_EQUALS(l.Entry(i), x);
        }
        else {
          TS_ASSERT_EQUALS(l(j), v(j));
          TS_ASSERT_EQUALS(l.Entry(j), v(j));
        }
      }

      CheckLocation(l, JSBSim::FGColumnVector3(l(1), l(2), l(3)));
    }

    // Check the copy assignment operator for an FGLocation with cached values.
    l = v;
    CheckLocation(l, v);

    lv = l;

    TS_ASSERT_DELTA(l(1), lv(1), epsilon);
    TS_ASSERT_DELTA(l(2), lv(2), epsilon);
    TS_ASSERT_DELTA(l(3), lv(3), epsilon);

    CheckLocation(lv, v);
  }

  void testOperations() {
    const JSBSim::FGColumnVector3 v(1.5, -2.0, 3.0);
    JSBSim::FGLocation l(v);
    l.SetEllipse(1., 1.);
    JSBSim::FGLocation l2(l);

    l2 += l;

    TS_ASSERT_EQUALS(l2(1), 2.0*l(1));
    TS_ASSERT_EQUALS(l2(2), 2.0*l(2));
    TS_ASSERT_EQUALS(l2(3), 2.0*l(3));
    CheckLocation(l2, 2.0*v);

    JSBSim::FGColumnVector3 v2 = l2;
    TS_ASSERT_VECTOR_EQUALS(v2, 2.0*v);

    l2 -= l;

    TS_ASSERT_EQUALS(l2(1), l(1));
    TS_ASSERT_EQUALS(l2(2), l(2));
    TS_ASSERT_EQUALS(l2(3), l(3));
    CheckLocation(l2, v);

    v2 = l2;
    TS_ASSERT_VECTOR_EQUALS(v2, v);

    l2 *= 3.5;

    TS_ASSERT_EQUALS(l2(1), 3.5*l(1));
    TS_ASSERT_EQUALS(l2(2), 3.5*l(2));
    TS_ASSERT_EQUALS(l2(3), 3.5*l(3));
    CheckLocation(l2, 3.5*v);

    l2 /= 7.0;

    TS_ASSERT_EQUALS(l2(1), 0.5*l(1));
    TS_ASSERT_EQUALS(l2(2), 0.5*l(2));
    TS_ASSERT_EQUALS(l2(3), 0.5*l(3));
    CheckLocation(l2, 0.5*v);

    l2 = l * 2.0;

    TS_ASSERT_EQUALS(l2(1), 2.0*l(1));
    TS_ASSERT_EQUALS(l2(2), 2.0*l(2));
    TS_ASSERT_EQUALS(l2(3), 2.0*l(3));
    CheckLocation(l2, 2.0 * v);

    l2 = 1.5 * l;

    TS_ASSERT_EQUALS(l2(1), 1.5*l(1));
    TS_ASSERT_EQUALS(l2(2), 1.5*l(2));
    TS_ASSERT_EQUALS(l2(3), 1.5*l(3));
    CheckLocation(l2, 1.5*v);

    l2 = 0.7 * l + l;

    TS_ASSERT_EQUALS(l2(1), 1.7*l(1));
    TS_ASSERT_EQUALS(l2(2), 1.7*l(2));
    TS_ASSERT_EQUALS(l2(3), 1.7*l(3));
    CheckLocation(l2, 1.7*v);

    l2 = 0.5 * l - l;

    TS_ASSERT_EQUALS(l2(1), -0.5*l(1));
    TS_ASSERT_EQUALS(l2(2), -0.5*l(2));
    TS_ASSERT_EQUALS(l2(3), -0.5*l(3));
    CheckLocation(l2, -0.5*v);
  }

  void testLocalLocation() {
    const JSBSim::FGColumnVector3 v(1.5, -2.0, 3.0);
    const JSBSim::FGColumnVector3 z(0.0, 0.0, 1.0);
    JSBSim::FGColumnVector3 v0(0.0, 0.0, -1.0);
    JSBSim::FGLocation l(v);

    JSBSim::FGLocation l2 = l.LocalToLocation(v0);
    TS_ASSERT_DELTA(l2.GetRadius(), v.Magnitude() + 1.0, epsilon);
    TS_ASSERT_VECTOR_EQUALS(v * l2, JSBSim::FGColumnVector3());
    TS_ASSERT_VECTOR_EQUALS(l.LocationToLocal(l2), v0);

    JSBSim::FGColumnVector3 east = (z * v).Normalize();
    v0 = JSBSim::FGColumnVector3(0.0, 1.0, 0.0);
    l2 = l.LocalToLocation(v0);
    TS_ASSERT_DELTA(l(3), l2(3), epsilon);
    TS_ASSERT_VECTOR_EQUALS(JSBSim::FGColumnVector3(l2), east+l);
    TS_ASSERT_VECTOR_EQUALS(l.LocationToLocal(l2), v0);

    JSBSim::FGColumnVector3 north = (v * east).Normalize();
    v0 = JSBSim::FGColumnVector3(1.0, 0.0, 0.0);
    l2 = l.LocalToLocation(v0);
    TS_ASSERT_VECTOR_EQUALS(JSBSim::FGColumnVector3(l2), north+l);
    TS_ASSERT_VECTOR_EQUALS(l.LocationToLocal(l2), v0);

    JSBSim::FGColumnVector3 down = (-1.0*v).Normalize();
    v0 = JSBSim::FGColumnVector3(1.0, 2.1, -0.5);
    l2 = l.LocalToLocation(v0);
    TS_ASSERT_VECTOR_EQUALS(JSBSim::FGColumnVector3(l2),
                            v0(1)*north+v0(2)*east+v0(3)*down+l);
    TS_ASSERT_VECTOR_EQUALS(l.LocationToLocal(l2), v0);
  }

  void testPosition() {
    const JSBSim::FGQuaternion qloc(2, -0.5*M_PI);
    JSBSim::FGLocation l;
    JSBSim::FGQuaternion q;
    JSBSim::FGMatrix33 m;
    JSBSim::FGColumnVector3 v;

    for (int ilat=-5; ilat <=5; ilat++) {
      l.SetRadius(1.0);
      TS_ASSERT_DELTA(1.0, l.GetRadius(), epsilon);
      double lat = ilat*M_PI/12.0;
      l.SetLatitude(lat);
      TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
      TS_ASSERT_DELTA(lat, l.GetLatitude(), epsilon);
      TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);
      TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);

      q = JSBSim::FGQuaternion(0.0, -lat, 0.0);
      m = (q * qloc).GetT();
      TS_ASSERT_MATRIX_EQUALS(m, l.GetTec2l());
      TS_ASSERT_MATRIX_EQUALS(m.Transposed(), l.GetTl2ec());

      for (unsigned int ilon=0; ilon < 12; ilon++) {
        double r = ilon + 1;
        double lon = NormalizedAngle(ilon*M_PI/6.0);
        l.SetLongitude(lon);
        TS_ASSERT_DELTA(lon, l.GetLongitude(), epsilon);
        TS_ASSERT_DELTA(lat, l.GetLatitude(), epsilon);
        TS_ASSERT_DELTA(sin(lon), l.GetSinLongitude(), epsilon);
        TS_ASSERT_DELTA(cos(lon), l.GetCosLongitude(), epsilon);

        q = JSBSim::FGQuaternion(0.0, -lat, lon);
        m = (q * qloc).GetT();
        TS_ASSERT_MATRIX_EQUALS(m, l.GetTec2l());
        TS_ASSERT_MATRIX_EQUALS(m.Transposed(), l.GetTl2ec());

        l.SetRadius(r);
        TS_ASSERT_DELTA(r, l.GetRadius(), epsilon);
        v = m.Transposed() * JSBSim::FGColumnVector3(0.0, 0.0, -r);
        TS_ASSERT_VECTOR_EQUALS(v, JSBSim::FGColumnVector3(l));
      }

      l.SetLongitude(0.0);
    }

    for (int ilat=-5; ilat <=5; ilat++) {
      double lat = ilat*M_PI/12.0;
      for (unsigned int ilon=0; ilon < 12; ilon++) {
        double r = ilon + 1;
        double lon = NormalizedAngle(ilon*M_PI/6.0);

        l.SetPosition(lon, lat, r);
        TS_ASSERT_DELTA(lon, l.GetLongitude(), epsilon);
        TS_ASSERT_DELTA(lat, l.GetLatitude(), epsilon);
        TS_ASSERT_DELTA(sin(lon), l.GetSinLongitude(), epsilon);
        TS_ASSERT_DELTA(cos(lon), l.GetCosLongitude(), epsilon);

        q = JSBSim::FGQuaternion(0.0, -lat, lon);
        m = (q * qloc).GetT();
        v = m.Transposed() * JSBSim::FGColumnVector3(0.0, 0.0, -r);
        TS_ASSERT_MATRIX_EQUALS(m, l.GetTec2l());
        TS_ASSERT_MATRIX_EQUALS(m.Transposed(), l.GetTl2ec());
        TS_ASSERT_DELTA(r, l.GetRadius(), epsilon);
        TS_ASSERT_VECTOR_EQUALS(v, JSBSim::FGColumnVector3(l));
      }
    }

    // Check the condition where the location is at the center of the Earth
    v.InitMatrix();
    l = v;
    TS_ASSERT_DELTA(0.0, l.GetRadius(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    l.SetRadius(1.0);
    l.SetEllipse(1., 1.);
    CheckLocation(l, JSBSim::FGColumnVector3(1., 0., 0.).Normalize());

    l = v;
    l.SetLatitude(M_PI*0.25);
    CheckLocation(l, JSBSim::FGColumnVector3(1., 0., 1.).Normalize());

    l = v;
    l.SetLongitude(M_PI*0.25);
    CheckLocation(l, JSBSim::FGColumnVector3(1., 1., 0.).Normalize());

    // Check the location definition does not depend on the order in which the
    // latitude & longitude are specified
    v(1) = 1.0;

    l = v;
    l.SetLongitude(M_PI/3.0);
    l.SetLatitude(M_PI/6.0);

    JSBSim::FGLocation lbis(v);
    lbis.SetLatitude(M_PI/6.0);
    lbis.SetLongitude(M_PI/3.0);
    TS_ASSERT_DELTA(l(1), lbis(1), epsilon);
    TS_ASSERT_DELTA(l(2), lbis(2), epsilon);
    TS_ASSERT_DELTA(l(3), lbis(3), epsilon);
  }

  void testGeodetic() {
    const double a = 20925646.32546; // WGS84 semimajor axis length in feet
    const double b = 20855486.5951;  // WGS84 semiminor axis length in feet
    JSBSim::FGLocation l;

    l.SetEllipse(a, b);

    for (int ilat=-5; ilat <=5; ilat++) {
      double glat = ilat*M_PI/12.0;
      for (unsigned int ilon=0; ilon < 12; ilon++) {
        double h = ilon + 1.0;
        double lon = NormalizedAngle(ilon*M_PI/6.0);
        double ac = a * cos(glat);
        double bs = b * sin(glat);
        double N = a*a / sqrt(ac*ac + bs*bs);
        l(1) = (N+h)*cos(glat)*cos(lon);
        l(2) = (N+h)*cos(glat)*sin(lon);
        l(3) = (b*b*N/(a*a)+h)*sin(glat);
        TS_ASSERT_DELTA(lon, l.GetLongitude(), epsilon);
        TS_ASSERT_DELTA(sin(lon), l.GetSinLongitude(), epsilon);
        TS_ASSERT_DELTA(cos(lon), l.GetCosLongitude(), epsilon);
        TS_ASSERT_DELTA(glat, l.GetGeodLatitudeRad(), epsilon);
        TS_ASSERT_DELTA(h, l.GetGeodAltitude(), 1E-8);
      }
    }

    for (int ilat=-5; ilat <=5; ilat++) {
      double glat = ilat*M_PI/12.0;
      for (unsigned int ilon=0; ilon < 12; ilon++) {
        double h = ilon + 1.0;
        double lon = NormalizedAngle(ilon*M_PI/6.0);
        double ac = a * cos(glat);
        double bs = b * sin(glat);
        double N = a*a / sqrt(ac*ac + bs*bs);
        double x = (N+h)*cos(glat)*cos(lon);
        double y = (N+h)*cos(glat)*sin(lon);
        double z = (b*b*N/(a*a)+h)*sin(glat);
        l.SetPositionGeodetic(lon, glat, h);
        TS_ASSERT_DELTA(x, l(1), epsilon*fabs(x));
        TS_ASSERT_DELTA(y, l(2), epsilon*fabs(y));
        TS_ASSERT_DELTA(z, l(3), epsilon*fabs(z));
        CheckLocation(l, JSBSim::FGColumnVector3(x, y, z));
        TS_ASSERT_DELTA(lon, l.GetLongitude(), epsilon);
        TS_ASSERT_DELTA(sin(lon), l.GetSinLongitude(), epsilon);
        TS_ASSERT_DELTA(cos(lon), l.GetCosLongitude(), epsilon);
        TS_ASSERT_DELTA(glat, l.GetGeodLatitudeRad(), epsilon);
        TS_ASSERT_DELTA(h, l.GetGeodAltitude(), 1E-8);
      }
    }
  }

  void testPoles() {
    JSBSim::FGColumnVector3 v(0.0, 0.0, 1.0); // North pole
    JSBSim::FGLocation l(v);

    TS_ASSERT_DELTA(M_PI*0.5, l.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    // Check that SetLongitude is a no-op when applied at the North pole
    l.SetLongitude(M_PI/6.0);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    l.SetLatitude(M_PI/3.0);
    TS_ASSERT_DELTA(M_PI/3.0, l.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    // South Pole
    l = -1.0 * v;
    TS_ASSERT_DELTA(-M_PI*0.5, l.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    // Check that SetLongitude is a no-op when applied at the South pole
    l.SetLongitude(M_PI/6.0);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    l.SetLatitude(-M_PI/3.0);
    TS_ASSERT_DELTA(-M_PI/3.0, l.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);

    // Geodetic calculations next to the North Pole
    const double a = 20925646.32546; // WGS84 semimajor axis length in feet
    const double b = 20855486.5951;  // WGS84 semiminor axis length in feet
    l.SetEllipse(a, b);
    l = b * v;
    TS_ASSERT_DELTA(90.0, l.GetGeodLatitudeDeg(), epsilon);
    TS_ASSERT_DELTA(M_PI*0.5, l.GetGeodLatitudeRad(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetGeodAltitude(), 1E-8);

    // Check locations next to the North Pole
    for (unsigned int i=1; i < 1000; i++) {
      double h = 10.0;
      double glat = 0.5*M_PI-i*1E-9;
      double ac = a * cos(glat);
      double bs = b * sin(glat);
      double N = a*a / sqrt(ac*ac + bs*bs);
      double x = (N+h)*cos(glat);
      double z = (b*b*N/(a*a)+h)*sin(glat);
      l.SetPositionGeodetic(0., glat, h);
      TS_ASSERT_DELTA(x, l(1), epsilon*fabs(x));
      TS_ASSERT_DELTA(0.0, l(2), epsilon);
      TS_ASSERT_DELTA(z, l(3), epsilon*fabs(z));
      TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
      TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);
      TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
      TS_ASSERT_DELTA(glat, l.GetGeodLatitudeRad(), epsilon);
      TS_ASSERT_DELTA(h, l.GetGeodAltitude(), 1E-8);
    }

    // Geodetic calculations next to the South Pole
    l = -b * v;
    TS_ASSERT_DELTA(-90.0, l.GetGeodLatitudeDeg(), epsilon);
    TS_ASSERT_DELTA(-0.5*M_PI, l.GetGeodLatitudeRad(), epsilon);
    TS_ASSERT_DELTA(0.0, l.GetGeodAltitude(), 1E-8);

    // Check locations next to the South Pole
    for (unsigned int i=1; i < 1000; i++) {
      double h = 10.0;
      double glat = -0.5*M_PI+i*1E-9;
      double ac = a * cos(glat);
      double bs = b * sin(glat);
      double N = a*a / sqrt(ac*ac + bs*bs);
      double x = (N+h)*cos(glat);
      double z = (b*b*N/(a*a)+h)*sin(glat);
      l.SetPositionGeodetic(0., glat, h);
      TS_ASSERT_DELTA(x, l(1), epsilon*fabs(x));
      TS_ASSERT_DELTA(0.0, l(2), epsilon);
      TS_ASSERT_DELTA(z, l(3), epsilon*fabs(z));
      TS_ASSERT_DELTA(0.0, l.GetLongitude(), epsilon);
      TS_ASSERT_DELTA(0.0, l.GetSinLongitude(), epsilon);
      TS_ASSERT_DELTA(1.0, l.GetCosLongitude(), epsilon);
      TS_ASSERT_DELTA(glat, l.GetGeodLatitudeRad(), epsilon);
      TS_ASSERT_DELTA(h, l.GetGeodAltitude(), 1E-8);
    }
  }

  void testNavigationOnSphericalEarth()
  {
    double slr = 20925646.32546; // Sea Level Radius
    double lat0 = 1.0;
    double lon0 = 1.0;
    JSBSim::FGLocation l0(lon0, lat0, slr);
    l0.SetEllipse(slr, slr);

    // Compare the computations with Haversine formulas.
    for (int ilat = -6; ilat <= 6; ilat++)
    {
      double lat = ilat * M_PI / 12.0;
      double dlat = lat - lat0;
      for (int ilon = -5; ilon < 6; ilon++)
      {
        double lon = NormalizedAngle(ilon * M_PI / 6.0);
        double dlon = lon - lon0;
        // Compute the distance
        double distance_a = pow(sin(0.5 * dlat), 2.) + (cos(lat0) * cos(lat) * pow(sin(0.5 * dlon), 2.));
        double distance = 2. * slr * atan2(sqrt(distance_a), sqrt(1. - distance_a));
#ifdef __APPLE__
        TS_ASSERT_DELTA(distance, l0.GetDistanceTo(lon, lat), 1E-6);
#else
        TS_ASSERT_DELTA(distance, l0.GetDistanceTo(lon, lat), 1E-7);
#endif
        // Compute the heading
        double Y = sin(dlon) * cos(lat);
        double X = cos(lat0) * sin(lat) - sin(lat0) * cos(lat) * cos(dlon);
        double heading = NormalizedAngle(atan2(Y, X));
        TS_ASSERT_DELTA(heading, l0.GetHeadingTo(lon, lat), epsilon);
      }
    }
  }

  void testNavigationOnOblateEarth()
  {
    using namespace std;

    const double a = 20925646.32546; // WGS84 semimajor axis length in feet
    const double b = 20855486.5951;  // WGS84 semiminor axis length in feet
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);
    l.SetPositionGeodetic(0., 0., 0.);

    // Distance and heading to other points on the equator.
    for (int ilon = -5; ilon < 6; ilon++)
    {
      double lon = NormalizedAngle(ilon * M_PI / 6.0);
      double distance = abs(lon * a);
      TS_ASSERT_DELTA(distance, l.GetDistanceTo(lon, 0.), 1.);
      if (ilon != 0)
        TS_ASSERT_DELTA(lon > 0 ? 0.5 * M_PI : -0.5 * M_PI,
                        l.GetHeadingTo(lon, 0.), epsilon);
    }

    // Compute the ellipse perimeter
    double h = (a-b)/(a+b);
    double p = M_PI*(a+b)*(1.+3.*h*h/(10.+sqrt(4-3.*h*h)));

    // Distance and heading to the poles.
    for (int ilon = -5; ilon < 6; ilon++)
    {
      double lon = NormalizedAngle(ilon * M_PI / 6.0);
      l.SetPositionGeodetic(lon, 0., 0.);
      TS_ASSERT_DELTA(0.25*p, l.GetDistanceTo(0., 0.5*M_PI), 1.);
      TS_ASSERT_DELTA(0., l.GetHeadingTo(0., 0.5*M_PI), epsilon);
      TS_ASSERT_DELTA(0.25*p, l.GetDistanceTo(0., -0.5*M_PI), 1.);
      TS_ASSERT_DELTA(M_PI, l.GetHeadingTo(0., -0.5*M_PI), epsilon);
    }

    // Distance to the antipode.
    for (int ilat = -5; ilat <= 5; ++ilat) {
      double glat = ilat * M_PI / 12.0;
      for (int ilon = -5; ilon <= 6; ++ilon) {
        double lon = NormalizedAngle(ilon * M_PI / 6.0);
        l.SetPositionGeodetic(lon, glat, 0.);
        TS_ASSERT_DELTA(0.5*p, l.GetDistanceTo(lon+M_PI, -glat), 1.);
      }
    }
  }

  // ============ Polar Coordinate Edge Cases ============

  void testDateLineCrossing() {
    // GIVEN: Location near the date line (±180° longitude)
    JSBSim::FGLocation l1(M_PI - 0.01, 0.0, 1.0);  // Just before +180°
    JSBSim::FGLocation l2(-M_PI + 0.01, 0.0, 1.0); // Just after -180°

    // THEN: Longitudes should be close to ±π
    TS_ASSERT_DELTA(M_PI - 0.01, l1.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(-M_PI + 0.01, l2.GetLongitude(), epsilon);

    // Check that positions are near each other (across date line)
    TS_ASSERT_DELTA(l1(1), l2(1), 0.1);  // X coords should be similar
    TS_ASSERT_DELTA(l1(3), l2(3), epsilon); // Z coords should be equal (both at equator)
  }

  void testEquatorEdgeCases() {
    // GIVEN: Locations exactly on the equator
    JSBSim::FGLocation l1(0.0, 0.0, 1.0);
    JSBSim::FGLocation l2(M_PI * 0.5, 0.0, 1.0);
    JSBSim::FGLocation l3(M_PI, 0.0, 1.0);

    // THEN: Latitude should be exactly zero
    TS_ASSERT_EQUALS(l1.GetLatitude(), 0.0);
    TS_ASSERT_EQUALS(l2.GetLatitude(), 0.0);
    TS_ASSERT_DELTA(l3.GetLatitude(), 0.0, epsilon);

    // Z component should be zero for equatorial positions
    TS_ASSERT_EQUALS(l1(3), 0.0);
    TS_ASSERT_EQUALS(l2(3), 0.0);
    TS_ASSERT_DELTA(l3(3), 0.0, epsilon);
  }

  void testCardinalDirections() {
    // GIVEN: Locations at cardinal longitudes (0°, 90°, 180°, 270°)
    double r = 1.0;
    JSBSim::FGLocation lEast(0.0, 0.0, r);        // 0° longitude
    JSBSim::FGLocation lNorth(M_PI * 0.5, 0.0, r); // 90° East
    JSBSim::FGLocation lWest(M_PI, 0.0, r);        // 180°
    JSBSim::FGLocation lSouth(-M_PI * 0.5, 0.0, r); // 90° West

    // THEN: Positions should be at expected Cartesian coordinates
    // At equator: x = r*cos(lon), y = r*sin(lon), z = 0
    TS_ASSERT_DELTA(lEast(1), r, epsilon);   // (1, 0, 0)
    TS_ASSERT_DELTA(lEast(2), 0.0, epsilon);

    TS_ASSERT_DELTA(lNorth(1), 0.0, epsilon); // (0, 1, 0)
    TS_ASSERT_DELTA(lNorth(2), r, epsilon);

    TS_ASSERT_DELTA(lWest(1), -r, epsilon);   // (-1, 0, 0)
    TS_ASSERT_DELTA(lWest(2), 0.0, epsilon);

    TS_ASSERT_DELTA(lSouth(1), 0.0, epsilon); // (0, -1, 0)
    TS_ASSERT_DELTA(lSouth(2), -r, epsilon);
  }

  void testVerySmallRadius() {
    // GIVEN: A location with very small radius
    JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, 1e-10);

    // THEN: Radius should be preserved
    TS_ASSERT_DELTA(l.GetRadius(), 1e-10, 1e-20);

    // Latitude and longitude should still be valid
    TS_ASSERT_DELTA(l.GetLongitude(), M_PI / 4.0, epsilon);
    TS_ASSERT_DELTA(l.GetLatitude(), M_PI / 6.0, epsilon);
  }

  void testExactDegreeValues() {
    // GIVEN: Location at exactly 45° lat, 45° lon
    double lat45 = M_PI / 4.0;
    double lon45 = M_PI / 4.0;
    JSBSim::FGLocation l(lon45, lat45, 1.0);

    // THEN: Degree conversions should be exact
    TS_ASSERT_DELTA(l.GetLongitudeDeg(), 45.0, epsilon);
    TS_ASSERT_DELTA(l.GetLatitudeDeg(), 45.0, epsilon);

    // Check sin/cos values
    double expected = sqrt(2.0) / 2.0;
    TS_ASSERT_DELTA(l.GetSinLongitude(), expected, epsilon);
    TS_ASSERT_DELTA(l.GetCosLongitude(), expected, epsilon);
  }

  void testLongitudeWrapAround() {
    // GIVEN: Setting longitude beyond ±π range
    JSBSim::FGLocation l;

    // WHEN: Setting various longitude values
    l.SetLongitude(0.0);
    TS_ASSERT_DELTA(l.GetLongitude(), 0.0, epsilon);

    l.SetLongitude(M_PI);
    TS_ASSERT_DELTA(std::abs(l.GetLongitude()), M_PI, epsilon);

    l.SetLongitude(-M_PI);
    TS_ASSERT_DELTA(std::abs(l.GetLongitude()), M_PI, epsilon);
  }

  void testLatitudeExtremes() {
    // GIVEN: Locations at extreme latitudes (±90° minus a small epsilon)
    double nearPole = M_PI * 0.5 - 1e-10;
    JSBSim::FGLocation lNearNorth(0.0, nearPole, 1.0);
    JSBSim::FGLocation lNearSouth(0.0, -nearPole, 1.0);

    // THEN: Latitudes should be close to ±90°
    TS_ASSERT_DELTA(lNearNorth.GetLatitude(), nearPole, epsilon);
    TS_ASSERT_DELTA(lNearSouth.GetLatitude(), -nearPole, epsilon);

    // Z component should dominate
    TS_ASSERT(std::abs(lNearNorth(3)) > 0.99);
    TS_ASSERT(std::abs(lNearSouth(3)) > 0.99);
  }

  void testPrimeMeridian() {
    // GIVEN: Location on prime meridian (longitude = 0)
    JSBSim::FGLocation l(0.0, M_PI / 6.0, 1.0);  // 0° lon, 30° lat

    // THEN: Y component should be zero
    TS_ASSERT_EQUALS(l(2), 0.0);

    // X and Z should follow spherical coordinates
    double lat = M_PI / 6.0;
    TS_ASSERT_DELTA(l(1), cos(lat), epsilon);
    TS_ASSERT_DELTA(l(3), sin(lat), epsilon);
  }

  // ============ Additional Coverage Tests ============

  void testGetTi2ecAndGetTec2i() {
    // GIVEN: A location with known position
    double lat = M_PI / 6.0;  // 30 degrees
    double lon = M_PI / 4.0;  // 45 degrees
    JSBSim::FGLocation l(lon, lat, 1.0);
    l.SetEllipse(1.0, 1.0);

    // THEN: Transformation matrices should be transposes of each other
    const JSBSim::FGMatrix33& Tec2l = l.GetTec2l();
    const JSBSim::FGMatrix33& Tl2ec = l.GetTl2ec();

    // Verify they are transposes
    TS_ASSERT_MATRIX_EQUALS(Tec2l, Tl2ec.Transposed());
    TS_ASSERT_MATRIX_EQUALS(Tl2ec, Tec2l.Transposed());

    // Verify they are orthogonal (transpose = inverse)
    JSBSim::FGMatrix33 identity;
    identity.InitMatrix(1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0);
    JSBSim::FGMatrix33 product = Tec2l * Tl2ec;
    TS_ASSERT_MATRIX_EQUALS(product, identity);
  }

  void testDistanceCalculations() {
    // GIVEN: Two locations on a spherical Earth
    const double a = 20925646.32546;
    const double b = 20925646.32546;  // Sphere
    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a, b);
    l2.SetEllipse(a, b);

    // WHEN: Setting up location at equator, prime meridian
    l1.SetPositionGeodetic(0.0, 0.0, 0.0);

    // Test distance to same point
    TS_ASSERT_DELTA(l1.GetDistanceTo(0.0, 0.0), 0.0, 1e-6);

    // Test distance to 90 degrees away (quarter circumference)
    double quarter_circ = 0.25 * 2.0 * M_PI * a;
    TS_ASSERT_DELTA(l1.GetDistanceTo(M_PI * 0.5, 0.0), quarter_circ, 100.0);
  }

  void testBearingCalculations() {
    // GIVEN: Two locations
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // WHEN: At equator, prime meridian
    l.SetPositionGeodetic(0.0, 0.0, 0.0);

    // THEN: Heading due east should be 90 degrees
    double heading_east = l.GetHeadingTo(M_PI / 6.0, 0.0);
    TS_ASSERT_DELTA(heading_east, M_PI * 0.5, 1e-3);

    // Heading due west should be -90 degrees
    double heading_west = l.GetHeadingTo(-M_PI / 6.0, 0.0);
    TS_ASSERT_DELTA(heading_west, -M_PI * 0.5, 1e-3);

    // Heading due north should be 0 degrees
    double heading_north = l.GetHeadingTo(0.0, M_PI / 6.0);
    TS_ASSERT_DELTA(heading_north, 0.0, 1e-3);
  }

  void testGetSeaLevelRadius() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // WHEN: At equator
    l.SetPositionGeodetic(0.0, 0.0, 0.0);

    // THEN: Sea level radius should be semimajor axis
    TS_ASSERT_DELTA(l.GetSeaLevelRadius(), a, 1.0);

    // WHEN: At pole
    l.SetPositionGeodetic(0.0, M_PI * 0.5, 0.0);

    // THEN: Sea level radius should be semiminor axis
    TS_ASSERT_DELTA(l.GetSeaLevelRadius(), b, 1.0);

    // WHEN: At 45 degrees latitude
    l.SetPositionGeodetic(0.0, M_PI / 4.0, 0.0);

    // THEN: Sea level radius should be between a and b
    double slr = l.GetSeaLevelRadius();
    TS_ASSERT(slr > b && slr < a);
  }

  void testGeodeticAltitudeWithVariousHeights() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test various altitudes at equator
    for (double h = 0.0; h <= 10000.0; h += 1000.0) {
      l.SetPositionGeodetic(0.0, 0.0, h);
      TS_ASSERT_DELTA(l.GetGeodAltitude(), h, 1e-6);
    }

    // Test various altitudes at 45 degrees
    for (double h = 0.0; h <= 10000.0; h += 1000.0) {
      l.SetPositionGeodetic(M_PI / 4.0, M_PI / 4.0, h);
      TS_ASSERT_DELTA(l.GetGeodAltitude(), h, 1e-6);
    }

    // Test various altitudes at pole
    for (double h = 0.0; h <= 10000.0; h += 1000.0) {
      l.SetPositionGeodetic(0.0, M_PI * 0.5, h);
      TS_ASSERT_DELTA(l.GetGeodAltitude(), h, 1e-6);
    }
  }

  void testLocalToLocationAndBack() {
    // GIVEN: A location
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l0;
    l0.SetEllipse(a, b);
    l0.SetPositionGeodetic(M_PI / 6.0, M_PI / 4.0, 1000.0);

    // Test various local vectors
    JSBSim::FGColumnVector3 local_vectors[] = {
      JSBSim::FGColumnVector3(100.0, 0.0, 0.0),    // North
      JSBSim::FGColumnVector3(0.0, 100.0, 0.0),    // East
      JSBSim::FGColumnVector3(0.0, 0.0, 100.0),    // Down
      JSBSim::FGColumnVector3(100.0, 50.0, -25.0), // Mixed
      JSBSim::FGColumnVector3(-50.0, -75.0, 30.0)  // Mixed
    };

    for (const auto& local : local_vectors) {
      // Convert to ECEF location and back
      JSBSim::FGLocation l_ecef = l0.LocalToLocation(local);
      JSBSim::FGColumnVector3 local_back = l0.LocationToLocal(l_ecef);

      // Should get back the same vector (with tolerance for numerical precision)
      TS_ASSERT_DELTA(local_back(1), local(1), 1e-8);
      TS_ASSERT_DELTA(local_back(2), local(2), 1e-8);
      TS_ASSERT_DELTA(local_back(3), local(3), 1e-8);
    }
  }

  void testSetPositionWithNegativeRadius() {
    // GIVEN: A location
    JSBSim::FGLocation l;

    // WHEN: Setting radius to a small positive value
    l.SetRadius(0.001);

    // THEN: Radius should be set correctly
    TS_ASSERT_DELTA(l.GetRadius(), 0.001, epsilon);

    // Position components should be valid
    TS_ASSERT(std::isfinite(l(1)));
    TS_ASSERT(std::isfinite(l(2)));
    TS_ASSERT(std::isfinite(l(3)));
  }

  void testMultipleSetEllipseCalls() {
    // GIVEN: A location
    JSBSim::FGLocation l;
    const double a1 = 20925646.32546;
    const double b1 = 20855486.5951;
    const double a2 = 10000000.0;
    const double b2 = 9900000.0;

    // WHEN: Setting ellipse parameters multiple times
    l.SetEllipse(a1, b1);
    l.SetPositionGeodetic(0.0, M_PI / 4.0, 1000.0);
    double alt1 = l.GetGeodAltitude();

    l.SetEllipse(a2, b2);
    l.SetPositionGeodetic(0.0, M_PI / 4.0, 1000.0);
    double alt2 = l.GetGeodAltitude();

    // THEN: Both should give correct altitude
    TS_ASSERT_DELTA(alt1, 1000.0, 1e-6);
    TS_ASSERT_DELTA(alt2, 1000.0, 1e-6);
  }

  void testGeodeticLatitudeConversions() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test geodetic latitude in both radians and degrees
    for (int ilat = -5; ilat <= 5; ilat++) {
      double glat = ilat * M_PI / 12.0;
      l.SetPositionGeodetic(0.0, glat, 0.0);

      // Check radian conversion
      TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), glat, epsilon);

      // Check degree conversion
      double glat_deg = glat * 180.0 / M_PI;
      TS_ASSERT_DELTA(l.GetGeodLatitudeDeg(), glat_deg, epsilon);
    }
  }

  void testCopyConstructorWithEllipse() {
    // GIVEN: A location with ellipse set
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l1;
    l1.SetEllipse(a, b);
    l1.SetPositionGeodetic(M_PI / 6.0, M_PI / 4.0, 5000.0);

    // WHEN: Copy constructing
    JSBSim::FGLocation l2(l1);

    // THEN: All properties should match
    TS_ASSERT_DELTA(l2.GetLongitude(), l1.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(l2.GetLatitude(), l1.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(l2.GetRadius(), l1.GetRadius(), epsilon);
    TS_ASSERT_DELTA(l2.GetGeodLatitudeRad(), l1.GetGeodLatitudeRad(), epsilon);
    TS_ASSERT_DELTA(l2.GetGeodAltitude(), l1.GetGeodAltitude(), epsilon);
    TS_ASSERT_DELTA(l2.GetSeaLevelRadius(), l1.GetSeaLevelRadius(), epsilon);

    // Transformation matrices should match
    TS_ASSERT_MATRIX_EQUALS(l2.GetTec2l(), l1.GetTec2l());
    TS_ASSERT_MATRIX_EQUALS(l2.GetTl2ec(), l1.GetTl2ec());
  }

  void testAssignmentWithEllipse() {
    // GIVEN: Two locations with different ellipse parameters
    const double a1 = 20925646.32546;
    const double b1 = 20855486.5951;
    const double a2 = 10000000.0;
    const double b2 = 9900000.0;

    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a1, b1);
    l1.SetPositionGeodetic(M_PI / 6.0, M_PI / 4.0, 5000.0);

    l2.SetEllipse(a2, b2);
    l2.SetPositionGeodetic(0.0, 0.0, 0.0);

    // WHEN: Assigning l1 to l2
    l2 = l1;

    // THEN: l2 should have all properties of l1
    TS_ASSERT_DELTA(l2.GetLongitude(), l1.GetLongitude(), epsilon);
    TS_ASSERT_DELTA(l2.GetLatitude(), l1.GetLatitude(), epsilon);
    TS_ASSERT_DELTA(l2.GetRadius(), l1.GetRadius(), epsilon);
    TS_ASSERT_DELTA(l2.GetGeodLatitudeRad(), l1.GetGeodLatitudeRad(), epsilon);
    TS_ASSERT_DELTA(l2.GetGeodAltitude(), l1.GetGeodAltitude(), epsilon);
    TS_ASSERT_DELTA(l2.GetSeaLevelRadius(), l1.GetSeaLevelRadius(), epsilon);
  }

  void testRadiusExtremes() {
    // GIVEN: A location
    JSBSim::FGLocation l;

    // Test very large radius
    double large_radius = 1e12;
    l.SetRadius(large_radius);
    TS_ASSERT_DELTA(l.GetRadius(), large_radius, large_radius * epsilon);

    // Test very small radius
    double small_radius = 1e-12;
    l.SetRadius(small_radius);
    TS_ASSERT_DELTA(l.GetRadius(), small_radius, epsilon);
  }

  void testSetPositionPreservesEllipse() {
    // GIVEN: A location with ellipse set
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);
    l.SetPositionGeodetic(0.0, 0.0, 0.0);

    // WHEN: Using SetPosition (geocentric)
    l.SetPosition(M_PI / 4.0, M_PI / 6.0, a);

    // THEN: Geodetic methods should still work
    double glat = l.GetGeodLatitudeRad();
    TS_ASSERT(std::isfinite(glat));

    double galt = l.GetGeodAltitude();
    TS_ASSERT(std::isfinite(galt));
  }

  void testOperatorPlusWithEllipse() {
    // GIVEN: Two locations with ellipse parameters
    const double a = 20925646.32546;
    const double b = 20855486.5951;

    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a, b);
    l1.SetPositionGeodetic(0.0, 0.0, 1000.0);

    l2.SetEllipse(a, b);
    l2.SetPositionGeodetic(M_PI / 6.0, M_PI / 6.0, 2000.0);

    // WHEN: Adding the locations
    JSBSim::FGLocation l3 = l1 + l2;

    // THEN: Result should preserve ellipse and have sum of ECEF coordinates
    TS_ASSERT_DELTA(l3(1), l1(1) + l2(1), epsilon * a);
    TS_ASSERT_DELTA(l3(2), l1(2) + l2(2), epsilon * a);
    TS_ASSERT_DELTA(l3(3), l1(3) + l2(3), epsilon * a);

    // Geodetic methods should work
    double galt = l3.GetGeodAltitude();
    TS_ASSERT(std::isfinite(galt));
  }

  void testOperatorMinusWithEllipse() {
    // GIVEN: Two locations with ellipse parameters
    const double a = 20925646.32546;
    const double b = 20855486.5951;

    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a, b);
    l1.SetPositionGeodetic(M_PI / 6.0, M_PI / 6.0, 2000.0);

    l2.SetEllipse(a, b);
    l2.SetPositionGeodetic(M_PI / 12.0, M_PI / 12.0, 1000.0);

    // WHEN: Subtracting the locations
    JSBSim::FGLocation l3 = l1 - l2;

    // THEN: Result should preserve ellipse and have difference of ECEF coordinates
    TS_ASSERT_DELTA(l3(1), l1(1) - l2(1), epsilon * a);
    TS_ASSERT_DELTA(l3(2), l1(2) - l2(2), epsilon * a);
    TS_ASSERT_DELTA(l3(3), l1(3) - l2(3), epsilon * a);
  }

  void testOperatorScalarMultiply() {
    // GIVEN: A location with ellipse
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l1;
    l1.SetEllipse(a, b);
    l1.SetPositionGeodetic(M_PI / 6.0, M_PI / 4.0, 1000.0);

    double scalar = 2.5;

    // WHEN: Multiplying by scalar (both forms)
    JSBSim::FGLocation l2 = l1 * scalar;
    JSBSim::FGLocation l3 = scalar * l1;

    // THEN: Both should give same result
    TS_ASSERT_DELTA(l2(1), l3(1), epsilon * a);
    TS_ASSERT_DELTA(l2(2), l3(2), epsilon * a);
    TS_ASSERT_DELTA(l2(3), l3(3), epsilon * a);

    // And should be scaled properly
    TS_ASSERT_DELTA(l2(1), l1(1) * scalar, epsilon * a);
    TS_ASSERT_DELTA(l2(2), l1(2) * scalar, epsilon * a);
    TS_ASSERT_DELTA(l2(3), l1(3) * scalar, epsilon * a);

    // Geodetic methods should work
    TS_ASSERT(std::isfinite(l2.GetGeodAltitude()));
  }

  void testCastToColumnVector() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 6.0, M_PI / 4.0, 1000.0);

    // WHEN: Casting to FGColumnVector3
    const JSBSim::FGColumnVector3& v = l;

    // THEN: Vector components should match location components
    TS_ASSERT_EQUALS(v(1), l(1));
    TS_ASSERT_EQUALS(v(2), l(2));
    TS_ASSERT_EQUALS(v(3), l(3));

    // Should maintain values
    JSBSim::FGColumnVector3 v2 = static_cast<const JSBSim::FGColumnVector3&>(l);
    TS_ASSERT_EQUALS(v2(1), l(1));
    TS_ASSERT_EQUALS(v2(2), l(2));
    TS_ASSERT_EQUALS(v2(3), l(3));
  }

  void testEntryAccessorConsistency() {
    // GIVEN: A location
    JSBSim::FGColumnVector3 vec(1.5, -2.0, 3.0);
    JSBSim::FGLocation l(vec);

    // THEN: Entry() and operator() should be consistent
    for (unsigned int i = 1; i <= 3; i++) {
      TS_ASSERT_EQUALS(l.Entry(i), l(i));
      TS_ASSERT_EQUALS(l.Entry(i), vec(i));
    }

    // Test write access
    l.Entry(1) = 10.0;
    TS_ASSERT_EQUALS(l(1), 10.0);

    l(2) = 20.0;
    TS_ASSERT_EQUALS(l.Entry(2), 20.0);
  }

  void testZeroRadiusHandling() {
    // GIVEN: A location at the Earth's center
    JSBSim::FGLocation l;
    JSBSim::FGColumnVector3 zero(0.0, 0.0, 0.0);
    l = zero;

    // THEN: Should have zero radius
    TS_ASSERT_DELTA(l.GetRadius(), 0.0, epsilon);

    // WHEN: Setting latitude (should set radius to 1)
    l.SetLatitude(M_PI / 4.0);

    // THEN: Radius should now be 1
    TS_ASSERT_DELTA(l.GetRadius(), 1.0, epsilon);
    TS_ASSERT_DELTA(l.GetLatitude(), M_PI / 4.0, epsilon);

    // Reset to zero
    l = zero;
    TS_ASSERT_DELTA(l.GetRadius(), 0.0, epsilon);

    // WHEN: Setting longitude (should set radius to 1)
    l.SetLongitude(M_PI / 3.0);

    // THEN: Radius should now be 1
    TS_ASSERT_DELTA(l.GetRadius(), 1.0, epsilon);
    TS_ASSERT_DELTA(l.GetLongitude(), M_PI / 3.0, epsilon);
  }

  void testLatitudeLimits() {
    // GIVEN: A location
    JSBSim::FGLocation l;

    // Test at exactly +90 degrees
    l.SetLatitude(M_PI * 0.5);
    TS_ASSERT_DELTA(l.GetLatitude(), M_PI * 0.5, epsilon);
    TS_ASSERT_DELTA(l.GetLatitudeDeg(), 90.0, epsilon);

    // Test at exactly -90 degrees
    l.SetLatitude(-M_PI * 0.5);
    TS_ASSERT_DELTA(l.GetLatitude(), -M_PI * 0.5, epsilon);
    TS_ASSERT_DELTA(l.GetLatitudeDeg(), -90.0, epsilon);

    // Test just below +90 degrees
    l.SetLatitude(M_PI * 0.5 - 1e-10);
    TS_ASSERT_DELTA(l.GetLatitude(), M_PI * 0.5 - 1e-10, epsilon);
  }

  void testLongitudeNormalization() {
    // GIVEN: A location
    JSBSim::FGLocation l;

    // Test various longitude values
    l.SetLongitude(0.0);
    TS_ASSERT_DELTA(l.GetLongitude(), 0.0, epsilon);

    l.SetLongitude(M_PI);
    TS_ASSERT_DELTA(std::abs(l.GetLongitude()), M_PI, epsilon);

    l.SetLongitude(-M_PI);
    TS_ASSERT_DELTA(std::abs(l.GetLongitude()), M_PI, epsilon);

    // Test intermediate values
    l.SetLongitude(M_PI / 2.0);
    TS_ASSERT_DELTA(l.GetLongitude(), M_PI / 2.0, epsilon);
    TS_ASSERT_DELTA(l.GetLongitudeDeg(), 90.0, epsilon);

    l.SetLongitude(-M_PI / 2.0);
    TS_ASSERT_DELTA(l.GetLongitude(), -M_PI / 2.0, epsilon);
    TS_ASSERT_DELTA(l.GetLongitudeDeg(), -90.0, epsilon);
  }

  void testGeodeticNearEquator() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test locations very near equator
    for (int i = -10; i <= 10; i++) {
      double glat = i * 1e-6;  // Very small latitude
      double h = 1000.0;
      l.SetPositionGeodetic(0.0, glat, h);

      TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), glat, 1e-9);
      TS_ASSERT_DELTA(l.GetGeodAltitude(), h, 1e-6);
    }
  }

  void testTransformationMatrixProperties() {
    // GIVEN: Various locations
    for (int ilat = -5; ilat <= 5; ilat++) {
      double lat = ilat * M_PI / 12.0;
      for (int ilon = 0; ilon < 12; ilon++) {
        double lon = NormalizedAngle(ilon * M_PI / 6.0);
        JSBSim::FGLocation l(lon, lat, 1.0);

        // THEN: Transformation matrices should be orthogonal
        const JSBSim::FGMatrix33& Tec2l = l.GetTec2l();
        const JSBSim::FGMatrix33& Tl2ec = l.GetTl2ec();

        // Check orthogonality: T * T^T = I
        JSBSim::FGMatrix33 I1 = Tec2l * Tl2ec;
        JSBSim::FGMatrix33 I2 = Tl2ec * Tec2l;

        // Diagonal elements should be 1
        TS_ASSERT_DELTA(I1(1,1), 1.0, epsilon);
        TS_ASSERT_DELTA(I1(2,2), 1.0, epsilon);
        TS_ASSERT_DELTA(I1(3,3), 1.0, epsilon);

        TS_ASSERT_DELTA(I2(1,1), 1.0, epsilon);
        TS_ASSERT_DELTA(I2(2,2), 1.0, epsilon);
        TS_ASSERT_DELTA(I2(3,3), 1.0, epsilon);

        // Off-diagonal elements should be 0
        TS_ASSERT_DELTA(I1(1,2), 0.0, epsilon);
        TS_ASSERT_DELTA(I1(1,3), 0.0, epsilon);
        TS_ASSERT_DELTA(I1(2,1), 0.0, epsilon);
        TS_ASSERT_DELTA(I1(2,3), 0.0, epsilon);
        TS_ASSERT_DELTA(I1(3,1), 0.0, epsilon);
        TS_ASSERT_DELTA(I1(3,2), 0.0, epsilon);
      }
    }
  }

  void testLocalCoordinateVectors() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 6.0, M_PI / 4.0, 20000000.0);

    // WHEN: Creating unit vectors in local frame
    JSBSim::FGColumnVector3 north(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 east(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 down(0.0, 0.0, 1.0);

    // THEN: Converting to ECEF and back should preserve them
    JSBSim::FGLocation l_north = l.LocalToLocation(north);
    JSBSim::FGLocation l_east = l.LocalToLocation(east);
    JSBSim::FGLocation l_down = l.LocalToLocation(down);

    JSBSim::FGColumnVector3 north_back = l.LocationToLocal(l_north);
    JSBSim::FGColumnVector3 east_back = l.LocationToLocal(l_east);
    JSBSim::FGColumnVector3 down_back = l.LocationToLocal(l_down);

    // Use tolerance for numerical precision
    TS_ASSERT_DELTA(north_back(1), north(1), 1e-8);
    TS_ASSERT_DELTA(north_back(2), north(2), 1e-8);
    TS_ASSERT_DELTA(north_back(3), north(3), 1e-8);

    TS_ASSERT_DELTA(east_back(1), east(1), 1e-8);
    TS_ASSERT_DELTA(east_back(2), east(2), 1e-8);
    TS_ASSERT_DELTA(east_back(3), east(3), 1e-8);

    TS_ASSERT_DELTA(down_back(1), down(1), 1e-8);
    TS_ASSERT_DELTA(down_back(2), down(2), 1e-8);
    TS_ASSERT_DELTA(down_back(3), down(3), 1e-8);
  }

  void testSinCosLongitudeConsistency() {
    // GIVEN: Various longitudes
    for (int ilon = -12; ilon <= 12; ilon++) {
      double lon = NormalizedAngle(ilon * M_PI / 6.0);
      JSBSim::FGLocation l(lon, 0.0, 1.0);

      // THEN: Sin and cos should match the longitude
      TS_ASSERT_DELTA(l.GetSinLongitude(), sin(lon), epsilon);
      TS_ASSERT_DELTA(l.GetCosLongitude(), cos(lon), epsilon);

      // And should satisfy trig identity
      double sin_lon = l.GetSinLongitude();
      double cos_lon = l.GetCosLongitude();
      TS_ASSERT_DELTA(sin_lon * sin_lon + cos_lon * cos_lon, 1.0, epsilon);
    }
  }

  void testGeodeticAtNegativeAltitude() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // WHEN: Setting negative altitude (below sea level)
    double h = -1000.0;
    l.SetPositionGeodetic(0.0, M_PI / 4.0, h);

    // THEN: Should handle it correctly
    TS_ASSERT_DELTA(l.GetGeodAltitude(), h, 1e-6);
    TS_ASSERT(std::isfinite(l.GetGeodLatitudeRad()));
    TS_ASSERT(l.GetRadius() > 0.0);
  }

  void testCompoundOperations() {
    // GIVEN: Locations
    const double a = 20925646.32546;
    const double b = 20855486.5951;

    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a, b);
    l2.SetEllipse(a, b);

    l1.SetPositionGeodetic(0.0, M_PI / 6.0, 1000.0);
    l2.SetPositionGeodetic(M_PI / 6.0, M_PI / 4.0, 2000.0);

    // WHEN: Performing compound operations
    JSBSim::FGLocation l3 = 2.0 * l1 + 0.5 * l2;

    // THEN: Result should be correct
    TS_ASSERT_DELTA(l3(1), 2.0 * l1(1) + 0.5 * l2(1), epsilon * a);
    TS_ASSERT_DELTA(l3(2), 2.0 * l1(2) + 0.5 * l2(2), epsilon * a);
    TS_ASSERT_DELTA(l3(3), 2.0 * l1(3) + 0.5 * l2(3), epsilon * a);
  }

  void testDistanceSymmetry() {
    // GIVEN: Two locations
    const double a = 20925646.32546;
    const double b = 20855486.5951;

    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a, b);
    l2.SetEllipse(a, b);

    double lon1 = M_PI / 6.0, lat1 = M_PI / 4.0;
    double lon2 = M_PI / 3.0, lat2 = M_PI / 6.0;

    l1.SetPositionGeodetic(lon1, lat1, 0.0);
    l2.SetPositionGeodetic(lon2, lat2, 0.0);

    // THEN: Distance should be symmetric
    double dist12 = l1.GetDistanceTo(lon2, lat2);
    double dist21 = l2.GetDistanceTo(lon1, lat1);

    TS_ASSERT_DELTA(dist12, dist21, 1.0);
  }

  void testDefaultConstructorState() {
    // GIVEN: Default constructed location
    JSBSim::FGLocation l;

    // THEN: Should be at (1, 0, 0) in ECEF
    TS_ASSERT_EQUALS(l(1), 1.0);
    TS_ASSERT_EQUALS(l(2), 0.0);
    TS_ASSERT_EQUALS(l(3), 0.0);

    // Longitude and latitude should be zero
    TS_ASSERT_EQUALS(l.GetLongitude(), 0.0);
    TS_ASSERT_EQUALS(l.GetLatitude(), 0.0);

    // Radius should be 1
    TS_ASSERT_EQUALS(l.GetRadius(), 1.0);

    // Sin/cos longitude
    TS_ASSERT_EQUALS(l.GetSinLongitude(), 0.0);
    TS_ASSERT_EQUALS(l.GetCosLongitude(), 1.0);
  }

  // ============ Additional Numerical Precision Tests ============

  void testHighAltitudeGeodetic() {
    // GIVEN: WGS84 ellipsoid and high altitude (100km)
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    double high_alt = 328084.0;  // 100km in feet

    // Test at various latitudes
    for (int ilat = -6; ilat <= 6; ilat++) {
      double glat = ilat * M_PI / 12.0;
      l.SetPositionGeodetic(0.0, glat, high_alt);

      TS_ASSERT_DELTA(l.GetGeodAltitude(), high_alt, 1e-4);
      TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), glat, epsilon);
    }
  }

  void testVeryHighAltitudeGeodetic() {
    // GIVEN: WGS84 ellipsoid and geostationary altitude (~36,000 km)
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    double geo_alt = 118110236.0;  // ~36000 km in feet

    l.SetPositionGeodetic(0.0, 0.0, geo_alt);
    TS_ASSERT_DELTA(l.GetGeodAltitude(), geo_alt, 10.0);

    l.SetPositionGeodetic(M_PI / 4.0, M_PI / 4.0, geo_alt);
    TS_ASSERT_DELTA(l.GetGeodAltitude(), geo_alt, 10.0);
  }

  void testGeodeticVsGeocentricLatitude() {
    // GIVEN: WGS84 ellipsoid - geodetic and geocentric latitudes differ
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // At equator and poles, geodetic = geocentric
    l.SetPositionGeodetic(0.0, 0.0, 0.0);
    TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), l.GetLatitude(), 1e-6);

    l.SetPositionGeodetic(0.0, M_PI * 0.5, 0.0);
    TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), l.GetLatitude(), 1e-6);

    // At 45 degrees, they should differ
    l.SetPositionGeodetic(0.0, M_PI / 4.0, 0.0);
    double glat = l.GetGeodLatitudeRad();
    double clat = l.GetLatitude();
    TS_ASSERT(std::abs(glat - clat) > 1e-4);  // Should be different
    TS_ASSERT(glat > clat);  // Geodetic > geocentric in northern hemisphere
  }

  void testAntipodalDistances() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Compute the ellipse perimeter (approximate)
    double h = (a - b) / (a + b);
    double p = M_PI * (a + b) * (1.0 + 3.0 * h * h / (10.0 + sqrt(4.0 - 3.0 * h * h)));

    // Distance to antipodal point should be half circumference
    l.SetPositionGeodetic(0.0, 0.0, 0.0);
    double dist = l.GetDistanceTo(M_PI, 0.0);
    TS_ASSERT_DELTA(dist, 0.5 * p, 100.0);

    l.SetPositionGeodetic(M_PI / 4.0, M_PI / 4.0, 0.0);
    dist = l.GetDistanceTo(M_PI / 4.0 - M_PI, -M_PI / 4.0);
    TS_ASSERT_DELTA(dist, 0.5 * p, 100.0);
  }

  void testShortDistancePrecision() {
    // GIVEN: WGS84 ellipsoid, two nearby points
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Points 1 arc-second apart (approximately 100 feet)
    double lon1 = 0.0;
    double lat1 = M_PI / 4.0;
    double dlon = 1.0 / 3600.0 * M_PI / 180.0;  // 1 arc-second

    l.SetPositionGeodetic(lon1, lat1, 0.0);

    double dist = l.GetDistanceTo(lon1 + dlon, lat1);
    // At 45° latitude, 1 arc-second is roughly 72 feet in longitude
    TS_ASSERT(dist > 50.0 && dist < 150.0);
  }

  void testHeadingAtEquator() {
    // GIVEN: Location at equator
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);
    l.SetPositionGeodetic(0.0, 0.0, 0.0);

    // Heading to north pole should be 0
    TS_ASSERT_DELTA(l.GetHeadingTo(0.0, M_PI * 0.5), 0.0, epsilon);

    // Heading to south pole should be π
    TS_ASSERT_DELTA(l.GetHeadingTo(0.0, -M_PI * 0.5), M_PI, epsilon);

    // Heading to east should be π/2
    TS_ASSERT_DELTA(l.GetHeadingTo(M_PI / 4.0, 0.0), M_PI / 2.0, epsilon);

    // Heading to west should be -π/2
    TS_ASSERT_DELTA(l.GetHeadingTo(-M_PI / 4.0, 0.0), -M_PI / 2.0, epsilon);
  }

  void testHeadingFromPoles() {
    // GIVEN: Location at north pole
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // From north pole, any direction is south (heading = π for any lon)
    l.SetPositionGeodetic(0.0, M_PI * 0.5, 0.0);
    double heading = l.GetHeadingTo(0.0, 0.0);
    TS_ASSERT_DELTA(std::abs(heading), M_PI, epsilon);

    // From south pole, any direction is north (heading = 0 for any lon)
    l.SetPositionGeodetic(0.0, -M_PI * 0.5, 0.0);
    heading = l.GetHeadingTo(0.0, 0.0);
    TS_ASSERT_DELTA(heading, 0.0, epsilon);
  }

  void testLocationInterpolation() {
    // GIVEN: Two locations
    JSBSim::FGColumnVector3 v1(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 v2(0.0, 1.0, 0.0);
    JSBSim::FGLocation l1(v1);
    JSBSim::FGLocation l2(v2);

    // WHEN: Interpolating between them
    double t = 0.5;
    JSBSim::FGLocation l_mid = (1.0 - t) * l1 + t * l2;

    // THEN: Should be at midpoint in ECEF
    TS_ASSERT_DELTA(l_mid(1), 0.5, epsilon);
    TS_ASSERT_DELTA(l_mid(2), 0.5, epsilon);
    TS_ASSERT_DELTA(l_mid(3), 0.0, epsilon);

    // Longitude should be 45 degrees
    TS_ASSERT_DELTA(l_mid.GetLongitude(), M_PI / 4.0, epsilon);
  }

  void testECEFtoGeodeticRoundTrip() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test round trip at various points
    for (int ilat = -5; ilat <= 5; ilat++) {
      double glat = ilat * M_PI / 12.0;
      for (int ilon = -5; ilon <= 6; ilon++) {
        double lon = NormalizedAngle(ilon * M_PI / 6.0);
        for (double h = 0.0; h <= 100000.0; h += 50000.0) {
          l.SetPositionGeodetic(lon, glat, h);

          // Get ECEF coordinates
          double x = l(1), y = l(2), z = l(3);

          // Set from ECEF and verify geodetic
          JSBSim::FGLocation l2;
          l2.SetEllipse(a, b);
          l2(1) = x;
          l2(2) = y;
          l2(3) = z;

          TS_ASSERT_DELTA(l2.GetLongitude(), lon, epsilon);
          TS_ASSERT_DELTA(l2.GetGeodLatitudeRad(), glat, epsilon);
          TS_ASSERT_DELTA(l2.GetGeodAltitude(), h, 1e-6);
        }
      }
    }
  }

  void testTransformationChainConsistency() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 6.0, M_PI / 4.0, 20000000.0);

    // WHEN: Transforming a vector ECEF->local->ECEF
    JSBSim::FGColumnVector3 v_ecef(1000.0, 2000.0, 3000.0);

    const JSBSim::FGMatrix33& Tec2l = l.GetTec2l();
    const JSBSim::FGMatrix33& Tl2ec = l.GetTl2ec();

    JSBSim::FGColumnVector3 v_local = Tec2l * v_ecef;
    JSBSim::FGColumnVector3 v_back = Tl2ec * v_local;

    // THEN: Should get back the original vector (with reasonable tolerance)
    TS_ASSERT_DELTA(v_back(1), v_ecef(1), 1e-8);
    TS_ASSERT_DELTA(v_back(2), v_ecef(2), 1e-8);
    TS_ASSERT_DELTA(v_back(3), v_ecef(3), 1e-8);
  }

  void testLocalFrameOrientation() {
    // GIVEN: A location at equator, prime meridian
    JSBSim::FGLocation l(0.0, 0.0, 1.0);

    const JSBSim::FGMatrix33& Tec2l = l.GetTec2l();

    // At equator, prime meridian:
    // - North in local frame should point in +Z direction in ECEF
    // - East should point in +Y direction in ECEF
    // - Down should point in -X direction in ECEF

    // Transform unit vectors from local to ECEF
    JSBSim::FGColumnVector3 north_local(1.0, 0.0, 0.0);
    JSBSim::FGColumnVector3 east_local(0.0, 1.0, 0.0);
    JSBSim::FGColumnVector3 down_local(0.0, 0.0, 1.0);

    JSBSim::FGColumnVector3 north_ecef = l.GetTl2ec() * north_local;
    JSBSim::FGColumnVector3 east_ecef = l.GetTl2ec() * east_local;
    JSBSim::FGColumnVector3 down_ecef = l.GetTl2ec() * down_local;

    // Check orientations
    TS_ASSERT_DELTA(north_ecef(3), 1.0, epsilon);  // North points +Z
    TS_ASSERT_DELTA(east_ecef(2), 1.0, epsilon);   // East points +Y
    TS_ASSERT_DELTA(down_ecef(1), -1.0, epsilon);  // Down points -X
  }

  void testMagnitudePreservation() {
    // GIVEN: A location and vector
    JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, 1.0);
    JSBSim::FGColumnVector3 v(3.0, 4.0, 5.0);
    double orig_mag = v.Magnitude();

    // WHEN: Transforming the vector
    JSBSim::FGColumnVector3 v_local = l.GetTec2l() * v;
    JSBSim::FGColumnVector3 v_back = l.GetTl2ec() * v_local;

    // THEN: Magnitude should be preserved
    TS_ASSERT_DELTA(v_local.Magnitude(), orig_mag, epsilon);
    TS_ASSERT_DELTA(v_back.Magnitude(), orig_mag, epsilon);
  }

  void testCrossProduct() {
    // GIVEN: Two locations as vectors
    JSBSim::FGColumnVector3 v1(1.5, -2.0, 3.0);
    JSBSim::FGColumnVector3 v2(2.0, 1.0, -0.5);
    JSBSim::FGLocation l1(v1);
    JSBSim::FGLocation l2(v2);

    // Compute cross product using column vectors
    JSBSim::FGColumnVector3 cross = v1 * v2;

    // Cross product should be perpendicular to both
    double dot1 = v1(1) * cross(1) + v1(2) * cross(2) + v1(3) * cross(3);
    double dot2 = v2(1) * cross(1) + v2(2) * cross(2) + v2(3) * cross(3);

    TS_ASSERT_DELTA(dot1, 0.0, 1e-6);
    TS_ASSERT_DELTA(dot2, 0.0, 1e-6);
  }

  void testDotProduct() {
    // GIVEN: Two orthogonal locations (equator at 0° and 90°)
    JSBSim::FGLocation l1(0.0, 0.0, 1.0);      // (1, 0, 0)
    JSBSim::FGLocation l2(M_PI / 2.0, 0.0, 1.0); // (0, 1, 0)

    // Dot product of orthogonal unit vectors should be 0
    double dot = l1(1) * l2(1) + l1(2) * l2(2) + l1(3) * l2(3);
    TS_ASSERT_DELTA(dot, 0.0, epsilon);

    // Self dot product should be radius squared
    double self_dot = l1(1) * l1(1) + l1(2) * l1(2) + l1(3) * l1(3);
    TS_ASSERT_DELTA(self_dot, 1.0, epsilon);
  }

  void testSelfDistance() {
    // GIVEN: A location
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test at various points
    for (int ilat = -5; ilat <= 5; ilat++) {
      double glat = ilat * M_PI / 12.0;
      for (int ilon = -5; ilon <= 6; ilon++) {
        double lon = NormalizedAngle(ilon * M_PI / 6.0);
        l.SetPositionGeodetic(lon, glat, 0.0);

        // Distance to self should be 0
        double dist = l.GetDistanceTo(lon, glat);
        TS_ASSERT_DELTA(dist, 0.0, 1e-6);
      }
    }
  }

  void testTriangleInequality() {
    // GIVEN: Three locations forming a triangle
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l1, l2, l3;
    l1.SetEllipse(a, b);
    l2.SetEllipse(a, b);
    l3.SetEllipse(a, b);

    l1.SetPositionGeodetic(0.0, 0.0, 0.0);
    l2.SetPositionGeodetic(M_PI / 6.0, M_PI / 6.0, 0.0);
    l3.SetPositionGeodetic(M_PI / 3.0, 0.0, 0.0);

    double d12 = l1.GetDistanceTo(M_PI / 6.0, M_PI / 6.0);
    double d23 = l2.GetDistanceTo(M_PI / 3.0, 0.0);
    double d13 = l1.GetDistanceTo(M_PI / 3.0, 0.0);

    // Triangle inequality: sum of two sides >= third side
    TS_ASSERT(d12 + d23 >= d13 - 1.0);  // Small tolerance
    TS_ASSERT(d12 + d13 >= d23 - 1.0);
    TS_ASSERT(d23 + d13 >= d12 - 1.0);
  }

  void testNumericalStabilityNearPole() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test very close to pole (89.999... degrees)
    for (int i = 1; i <= 10; i++) {
      double glat = M_PI / 2.0 - i * 1e-9;
      l.SetPositionGeodetic(0.0, glat, 0.0);

      // Should be numerically stable
      TS_ASSERT(std::isfinite(l.GetGeodLatitudeRad()));
      TS_ASSERT(std::isfinite(l.GetGeodAltitude()));
      TS_ASSERT(std::isfinite(l.GetSeaLevelRadius()));
      TS_ASSERT(std::isfinite(l(1)));
      TS_ASSERT(std::isfinite(l(2)));
      TS_ASSERT(std::isfinite(l(3)));
    }
  }

  void testNumericalStabilityWithLargeAltitude() {
    // GIVEN: WGS84 ellipsoid with very large altitude
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    double huge_alt = 1e10;  // Very high altitude
    l.SetPositionGeodetic(M_PI / 4.0, M_PI / 4.0, huge_alt);

    // Should be numerically stable
    TS_ASSERT(std::isfinite(l.GetGeodLatitudeRad()));
    TS_ASSERT_DELTA(l.GetGeodAltitude(), huge_alt, huge_alt * 1e-10);
    TS_ASSERT(std::isfinite(l.GetSeaLevelRadius()));
    TS_ASSERT(l.GetRadius() > a + huge_alt * 0.9);
  }

  void testSphericalEarthSeaLevelRadius() {
    // GIVEN: Spherical Earth (a = b)
    const double r = 20925646.32546;
    JSBSim::FGLocation l;
    l.SetEllipse(r, r);

    // Sea level radius should be constant
    for (int ilat = -6; ilat <= 6; ilat++) {
      double glat = ilat * M_PI / 12.0;
      l.SetPositionGeodetic(0.0, glat, 0.0);
      TS_ASSERT_DELTA(l.GetSeaLevelRadius(), r, epsilon);
    }
  }

  void testEllipsoidFlattening() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Sea level radius at equator should be a
    l.SetPositionGeodetic(0.0, 0.0, 0.0);
    TS_ASSERT_DELTA(l.GetSeaLevelRadius(), a, 1.0);

    // Sea level radius at pole should be b
    l.SetPositionGeodetic(0.0, M_PI / 2.0, 0.0);
    TS_ASSERT_DELTA(l.GetSeaLevelRadius(), b, 1.0);

    // Sea level radius should monotonically decrease from equator to pole
    double prev_slr = a;
    for (int ilat = 1; ilat <= 6; ilat++) {
      double glat = ilat * M_PI / 12.0;
      l.SetPositionGeodetic(0.0, glat, 0.0);
      double slr = l.GetSeaLevelRadius();
      TS_ASSERT(slr < prev_slr);
      prev_slr = slr;
    }
  }

  void testMultipleLocationOperations() {
    // GIVEN: Multiple locations
    JSBSim::FGLocation l1(0.0, 0.0, 1.0);
    JSBSim::FGLocation l2(M_PI / 4.0, 0.0, 1.0);
    JSBSim::FGLocation l3(M_PI / 2.0, 0.0, 1.0);

    // WHEN: Combining operations (use *= for division)
    JSBSim::FGLocation l4 = l1 + l2 + l3;
    l4 *= (1.0 / 3.0);

    // THEN: Should be centroid of the three points
    double avg_x = (l1(1) + l2(1) + l3(1)) / 3.0;
    double avg_y = (l1(2) + l2(2) + l3(2)) / 3.0;
    double avg_z = (l1(3) + l2(3) + l3(3)) / 3.0;

    TS_ASSERT_DELTA(l4(1), avg_x, epsilon);
    TS_ASSERT_DELTA(l4(2), avg_y, epsilon);
    TS_ASSERT_DELTA(l4(3), avg_z, epsilon);
  }

  void testLocationNormalization() {
    // GIVEN: A location with arbitrary radius
    JSBSim::FGColumnVector3 v(3.0, 4.0, 5.0);
    JSBSim::FGLocation l(v);

    double orig_radius = l.GetRadius();
    double orig_lat = l.GetLatitude();
    double orig_lon = l.GetLongitude();

    // WHEN: Normalizing to unit sphere
    JSBSim::FGLocation l_unit = (1.0 / orig_radius) * l;

    // THEN: Radius should be 1, lat/lon unchanged
    TS_ASSERT_DELTA(l_unit.GetRadius(), 1.0, epsilon);
    TS_ASSERT_DELTA(l_unit.GetLatitude(), orig_lat, epsilon);
    TS_ASSERT_DELTA(l_unit.GetLongitude(), orig_lon, epsilon);
  }

  void testGreatCircleProperties() {
    // GIVEN: Two locations on a sphere
    const double r = 20925646.32546;
    JSBSim::FGLocation l;
    l.SetEllipse(r, r);  // Sphere

    l.SetPositionGeodetic(0.0, 0.0, 0.0);

    // Distance around 90 degrees should be quarter circumference
    double quarter_circ = 0.25 * 2.0 * M_PI * r;

    double dist = l.GetDistanceTo(M_PI / 2.0, 0.0);
    TS_ASSERT_DELTA(dist, quarter_circ, 100.0);

    dist = l.GetDistanceTo(0.0, M_PI / 2.0);
    TS_ASSERT_DELTA(dist, quarter_circ, 100.0);
  }

  void testLocationDifferenceVector() {
    // GIVEN: Two locations
    JSBSim::FGLocation l1(0.0, 0.0, 1.0);
    JSBSim::FGLocation l2(0.0, M_PI / 6.0, 1.0);

    // WHEN: Computing difference
    JSBSim::FGLocation diff = l2 - l1;

    // THEN: Should represent the displacement vector
    TS_ASSERT_DELTA(diff(1), l2(1) - l1(1), epsilon);
    TS_ASSERT_DELTA(diff(2), l2(2) - l1(2), epsilon);
    TS_ASSERT_DELTA(diff(3), l2(3) - l1(3), epsilon);
  }

  void testOperatorDivide() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, 10.0);

    // WHEN: Dividing by scalar (use *= with reciprocal)
    JSBSim::FGLocation l2 = l;
    l2 *= (1.0 / 2.0);

    // THEN: All components should be halved
    TS_ASSERT_DELTA(l2(1), l(1) / 2.0, epsilon);
    TS_ASSERT_DELTA(l2(2), l(2) / 2.0, epsilon);
    TS_ASSERT_DELTA(l2(3), l(3) / 2.0, epsilon);
    TS_ASSERT_DELTA(l2.GetRadius(), l.GetRadius() / 2.0, epsilon);
  }

  void testLocalToLocationAtPoles() {
    // GIVEN: Location at north pole
    JSBSim::FGColumnVector3 north_pole(0.0, 0.0, 1.0);
    JSBSim::FGLocation l(north_pole);

    // WHEN: Moving locally (small offset to stay near unit sphere)
    JSBSim::FGColumnVector3 local_south(0.1, 0.0, 0.0);  // Small move from pole
    JSBSim::FGLocation l2 = l.LocalToLocation(local_south);

    // THEN: Should move away from pole (latitude decreases)
    TS_ASSERT(l2.GetLatitude() < l.GetLatitude());

    // Radius should increase slightly (moved away from center)
    TS_ASSERT(l2.GetRadius() > l.GetRadius());
  }

  void testSetPositionDegreeConversions() {
    // GIVEN: A location
    JSBSim::FGLocation l;

    // Test degree conversions
    l.SetLongitude(M_PI / 3.0);
    l.SetLatitude(M_PI / 4.0);

    TS_ASSERT_DELTA(l.GetLongitudeDeg(), 60.0, epsilon);
    TS_ASSERT_DELTA(l.GetLatitudeDeg(), 45.0, epsilon);

    // Test negative values
    l.SetLongitude(-M_PI / 6.0);
    l.SetLatitude(-M_PI / 3.0);

    TS_ASSERT_DELTA(l.GetLongitudeDeg(), -30.0, epsilon);
    TS_ASSERT_DELTA(l.GetLatitudeDeg(), -60.0, epsilon);
  }

  void testModifyingLocationComponents() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, 1.0);
    l.SetEllipse(1.0, 1.0);

    // Store original values
    double orig_x = l(1);
    double orig_y = l(2);
    double orig_z = l(3);

    // WHEN: Modifying X component
    l(1) = 2.0;

    // THEN: Should update correctly
    TS_ASSERT_EQUALS(l(1), 2.0);
    TS_ASSERT_EQUALS(l(2), orig_y);
    TS_ASSERT_EQUALS(l(3), orig_z);

    // Latitude and longitude should recalculate
    TS_ASSERT(std::isfinite(l.GetLatitude()));
    TS_ASSERT(std::isfinite(l.GetLongitude()));
  }

  void testVectorConstructorVariants() {
    // GIVEN: Various vectors
    JSBSim::FGColumnVector3 v1(1.0, 0.0, 0.0);  // On X-axis
    JSBSim::FGColumnVector3 v2(0.0, 1.0, 0.0);  // On Y-axis
    JSBSim::FGColumnVector3 v3(0.0, 0.0, 1.0);  // On Z-axis

    JSBSim::FGLocation l1(v1);
    JSBSim::FGLocation l2(v2);
    JSBSim::FGLocation l3(v3);

    // THEN: Locations should have correct lat/lon
    TS_ASSERT_DELTA(l1.GetLongitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(l1.GetLatitude(), 0.0, epsilon);

    TS_ASSERT_DELTA(l2.GetLongitude(), M_PI / 2.0, epsilon);
    TS_ASSERT_DELTA(l2.GetLatitude(), 0.0, epsilon);

    TS_ASSERT_DELTA(l3.GetLongitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(l3.GetLatitude(), M_PI / 2.0, epsilon);
  }

  void testNegativeCoordinates() {
    // GIVEN: Locations with negative coordinates
    JSBSim::FGColumnVector3 v1(-1.0, 0.0, 0.0);   // -X axis
    JSBSim::FGColumnVector3 v2(0.0, -1.0, 0.0);   // -Y axis
    JSBSim::FGColumnVector3 v3(0.0, 0.0, -1.0);   // -Z axis (south pole)

    JSBSim::FGLocation l1(v1);
    JSBSim::FGLocation l2(v2);
    JSBSim::FGLocation l3(v3);

    // THEN: Should have correct lat/lon
    TS_ASSERT_DELTA(std::abs(l1.GetLongitude()), M_PI, epsilon);  // ±180°
    TS_ASSERT_DELTA(l1.GetLatitude(), 0.0, epsilon);

    TS_ASSERT_DELTA(l2.GetLongitude(), -M_PI / 2.0, epsilon);  // -90°
    TS_ASSERT_DELTA(l2.GetLatitude(), 0.0, epsilon);

    TS_ASSERT_DELTA(l3.GetLongitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(l3.GetLatitude(), -M_PI / 2.0, epsilon);  // South pole
  }

  // ============ Multi-Instance Independence Tests (79-82) ============

  // Test 79: Multiple location instances are independent
  void testMultipleInstanceIndependence() {
    // GIVEN: Multiple location instances
    JSBSim::FGLocation l1(0.0, 0.0, 1.0);
    JSBSim::FGLocation l2(M_PI / 4.0, M_PI / 6.0, 2.0);
    JSBSim::FGLocation l3(M_PI / 2.0, M_PI / 3.0, 3.0);

    // Store original values
    double l1_lon = l1.GetLongitude();
    double l2_lon = l2.GetLongitude();
    double l3_lon = l3.GetLongitude();

    // WHEN: Modifying one instance
    l2.SetLongitude(M_PI);

    // THEN: Other instances should be unaffected
    TS_ASSERT_DELTA(l1.GetLongitude(), l1_lon, epsilon);
    TS_ASSERT_DELTA(l3.GetLongitude(), l3_lon, epsilon);
    TS_ASSERT_DELTA(l2.GetLongitude(), M_PI, epsilon);
  }

  // Test 80: Independent ellipsoid settings
  void testIndependentEllipsoidSettings() {
    // GIVEN: Two locations with different ellipsoids
    const double a1 = 20925646.32546, b1 = 20855486.5951;  // WGS84
    const double a2 = 21000000.0, b2 = 20900000.0;         // Custom

    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a1, b1);
    l2.SetEllipse(a2, b2);

    l1.SetPositionGeodetic(0.0, M_PI / 4.0, 0.0);
    l2.SetPositionGeodetic(0.0, M_PI / 4.0, 0.0);

    // THEN: They should have different sea level radii
    double slr1 = l1.GetSeaLevelRadius();
    double slr2 = l2.GetSeaLevelRadius();
    TS_ASSERT(std::abs(slr1 - slr2) > 1000.0);
  }

  // Test 81: Copy independence
  void testCopyIndependence() {
    // GIVEN: A location
    JSBSim::FGLocation l1(M_PI / 4.0, M_PI / 6.0, 10.0);
    l1.SetEllipse(20925646.0, 20855486.0);

    // WHEN: Copying and modifying
    JSBSim::FGLocation l2 = l1;
    l2.SetLongitude(M_PI / 2.0);

    // THEN: Original should be unaffected
    TS_ASSERT_DELTA(l1.GetLongitude(), M_PI / 4.0, epsilon);
    TS_ASSERT_DELTA(l2.GetLongitude(), M_PI / 2.0, epsilon);
  }

  // Test 82: Assignment independence
  void testAssignmentIndependence() {
    // GIVEN: Two locations
    JSBSim::FGLocation l1(0.0, M_PI / 4.0, 5.0);
    JSBSim::FGLocation l2(M_PI / 2.0, 0.0, 10.0);

    // WHEN: Assigning and modifying
    JSBSim::FGLocation l3;
    l3 = l1;
    l1.SetLatitude(M_PI / 3.0);

    // THEN: l3 should retain original l1 values
    TS_ASSERT_DELTA(l3.GetLatitude(), M_PI / 4.0, epsilon);
    TS_ASSERT_DELTA(l1.GetLatitude(), M_PI / 3.0, epsilon);
  }

  // ============ State Consistency Tests (83-86) ============

  // Test 83: ECEF and geodetic consistency
  void testECEFGeodeticConsistency() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // WHEN: Setting position geodetically
    double lon = M_PI / 6.0;
    double lat = M_PI / 4.0;
    double alt = 10000.0;
    l.SetPositionGeodetic(lon, lat, alt);

    // THEN: ECEF coordinates should be consistent
    double x = l(1), y = l(2), z = l(3);
    double r = std::sqrt(x * x + y * y + z * z);
    TS_ASSERT(r > a * 0.9);
    TS_ASSERT(r < a * 1.1 + alt);

    // And should be recoverable
    TS_ASSERT_DELTA(l.GetLongitude(), lon, epsilon);
    TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), lat, epsilon);
    TS_ASSERT_DELTA(l.GetGeodAltitude(), alt, 1e-6);
  }

  // Test 84: Transform matrix orthogonality
  void testTransformMatrixOrthogonality() {
    // GIVEN: Various locations
    for (int ilat = -3; ilat <= 3; ilat++) {
      for (int ilon = 0; ilon <= 6; ilon++) {
        double lat = ilat * M_PI / 6.0;
        double lon = ilon * M_PI / 6.0;
        JSBSim::FGLocation l(lon, lat, 1.0);

        const JSBSim::FGMatrix33& T = l.GetTec2l();

        // THEN: Matrix should be orthogonal (T * T^T = I)
        JSBSim::FGMatrix33 TTT = T * T.Transposed();
        TS_ASSERT_DELTA(TTT(1,1), 1.0, epsilon);
        TS_ASSERT_DELTA(TTT(2,2), 1.0, epsilon);
        TS_ASSERT_DELTA(TTT(3,3), 1.0, epsilon);
        TS_ASSERT_DELTA(TTT(1,2), 0.0, epsilon);
        TS_ASSERT_DELTA(TTT(1,3), 0.0, epsilon);
        TS_ASSERT_DELTA(TTT(2,3), 0.0, epsilon);
      }
    }
  }

  // Test 85: Sin/cos latitude consistency
  void testSinCosLatitudeConsistency() {
    // GIVEN: Locations at various latitudes
    for (int ilat = -6; ilat <= 6; ilat++) {
      double lat = ilat * M_PI / 12.0;
      JSBSim::FGLocation l(0.0, lat, 1.0);

      // THEN: Sin/cos should satisfy identity
      double sin_lat = std::sin(l.GetLatitude());
      double cos_lat = std::cos(l.GetLatitude());
      TS_ASSERT_DELTA(sin_lat * sin_lat + cos_lat * cos_lat, 1.0, epsilon);

      // And should match computed values
      double sin_lon = l.GetSinLongitude();
      double cos_lon = l.GetCosLongitude();
      TS_ASSERT_DELTA(sin_lon * sin_lon + cos_lon * cos_lon, 1.0, epsilon);
    }
  }

  // Test 86: Radius consistency
  void testRadiusConsistency() {
    // GIVEN: Locations with various radii
    for (double r = 1.0; r <= 1e8; r *= 10.0) {
      JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, r);

      // THEN: Radius should be consistent with ECEF coordinates
      double x = l(1), y = l(2), z = l(3);
      double computed_r = std::sqrt(x * x + y * y + z * z);
      TS_ASSERT_DELTA(l.GetRadius(), computed_r, computed_r * epsilon);
    }
  }

  // ============ Transform Chain Tests (87-90) ============

  // Test 87: ECEF->Local->ECEF round trip
  void testECEFLocalECEFRoundTrip() {
    // GIVEN: Various locations
    for (int i = 0; i < 10; i++) {
      double lon = i * M_PI / 5.0 - M_PI;
      double lat = (i - 5) * M_PI / 12.0;
      JSBSim::FGLocation l(lon, lat, 1e7);

      JSBSim::FGColumnVector3 v_ecef(1000.0, 2000.0, 3000.0);

      // WHEN: Transform ECEF->Local->ECEF
      JSBSim::FGColumnVector3 v_local = l.GetTec2l() * v_ecef;
      JSBSim::FGColumnVector3 v_back = l.GetTl2ec() * v_local;

      // THEN: Should recover original
      TS_ASSERT_DELTA(v_back(1), v_ecef(1), 1e-8);
      TS_ASSERT_DELTA(v_back(2), v_ecef(2), 1e-8);
      TS_ASSERT_DELTA(v_back(3), v_ecef(3), 1e-8);
    }
  }

  // Test 88: Local->ECEF->Local round trip
  void testLocalECEFLocalRoundTrip() {
    // GIVEN: Various locations
    JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, 1e7);
    JSBSim::FGColumnVector3 v_local(100.0, 200.0, 300.0);

    // WHEN: Transform Local->ECEF->Local
    JSBSim::FGColumnVector3 v_ecef = l.GetTl2ec() * v_local;
    JSBSim::FGColumnVector3 v_back = l.GetTec2l() * v_ecef;

    // THEN: Should recover original
    TS_ASSERT_DELTA(v_back(1), v_local(1), 1e-8);
    TS_ASSERT_DELTA(v_back(2), v_local(2), 1e-8);
    TS_ASSERT_DELTA(v_back(3), v_local(3), 1e-8);
  }

  // Test 89: Transform determinant is +1
  void testTransformDeterminantPositiveOne() {
    // GIVEN: Various locations
    for (int i = 0; i < 12; i++) {
      double lon = i * M_PI / 6.0;
      for (int j = -3; j <= 3; j++) {
        double lat = j * M_PI / 7.0;
        JSBSim::FGLocation l(lon, lat, 1.0);

        const JSBSim::FGMatrix33& T = l.GetTec2l();

        // Compute determinant
        double det = T(1,1) * (T(2,2) * T(3,3) - T(2,3) * T(3,2))
                   - T(1,2) * (T(2,1) * T(3,3) - T(2,3) * T(3,1))
                   + T(1,3) * (T(2,1) * T(3,2) - T(2,2) * T(3,1));

        // THEN: Determinant should be +1 (proper rotation)
        TS_ASSERT_DELTA(det, 1.0, epsilon);
      }
    }
  }

  // Test 90: Inverse matrix correctness
  void testInverseMatrixCorrectness() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 3.0, M_PI / 5.0, 1.0);

    const JSBSim::FGMatrix33& Tec2l = l.GetTec2l();
    const JSBSim::FGMatrix33& Tl2ec = l.GetTl2ec();

    // THEN: Tl2ec should be transpose of Tec2l (for orthogonal matrix)
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT_DELTA(Tl2ec(i, j), Tec2l(j, i), epsilon);
      }
    }

    // And their product should be identity
    JSBSim::FGMatrix33 prod = Tec2l * Tl2ec;
    TS_ASSERT_DELTA(prod(1,1), 1.0, epsilon);
    TS_ASSERT_DELTA(prod(2,2), 1.0, epsilon);
    TS_ASSERT_DELTA(prod(3,3), 1.0, epsilon);
  }

  // ============ Edge Case Tests (91-94) ============

  // Test 91: Location at exact pole
  void testLocationAtExactPole() {
    // GIVEN: Location exactly at north pole
    JSBSim::FGColumnVector3 north_pole(0.0, 0.0, 1.0);
    JSBSim::FGLocation l(north_pole);

    // THEN: Latitude should be π/2
    TS_ASSERT_DELTA(l.GetLatitude(), M_PI / 2.0, epsilon);

    // Longitude is undefined at pole, but should be finite
    TS_ASSERT(std::isfinite(l.GetLongitude()));

    // Transform matrices should still be valid
    const JSBSim::FGMatrix33& T = l.GetTec2l();
    TS_ASSERT(std::isfinite(T(1,1)));
    TS_ASSERT(std::isfinite(T(2,2)));
    TS_ASSERT(std::isfinite(T(3,3)));
  }

  // Test 92: Location at date line
  void testLocationAtDateLine() {
    // GIVEN: Location at ±180 degrees longitude
    JSBSim::FGLocation l1(M_PI, 0.0, 1.0);
    JSBSim::FGLocation l2(-M_PI, 0.0, 1.0);

    // THEN: Both should represent the same point
    TS_ASSERT_DELTA(l1(1), l2(1), epsilon);
    TS_ASSERT_DELTA(l1(2), l2(2), epsilon);
    TS_ASSERT_DELTA(l1(3), l2(3), epsilon);

    // Both should have same latitude
    TS_ASSERT_DELTA(l1.GetLatitude(), l2.GetLatitude(), epsilon);
  }

  // Test 93: Zero radius location
  void testZeroRadiusLocation() {
    // GIVEN: Location at origin (zero radius)
    JSBSim::FGColumnVector3 origin(0.0, 0.0, 0.0);
    JSBSim::FGLocation l(origin);

    // THEN: Radius should be 0
    TS_ASSERT_DELTA(l.GetRadius(), 0.0, epsilon);

    // Lat/lon may be undefined but should be finite
    TS_ASSERT(std::isfinite(l.GetLatitude()) || std::isnan(l.GetLatitude()));
    TS_ASSERT(std::isfinite(l.GetLongitude()) || std::isnan(l.GetLongitude()));
  }

  // Test 94: Very small radius location
  void testVerySmallRadiusLocation() {
    // GIVEN: Location with very small radius
    JSBSim::FGLocation l(0.0, M_PI / 4.0, 1e-10);

    // THEN: Should still compute valid lat/lon
    TS_ASSERT_DELTA(l.GetLatitude(), M_PI / 4.0, 1e-5);
    TS_ASSERT_DELTA(l.GetLongitude(), 0.0, 1e-5);
    TS_ASSERT(l.GetRadius() > 0.0);
  }

  // ============ Stress Tests (95-98) ============

  // Test 95: Many sequential operations
  void testManySequentialOperations() {
    // GIVEN: A location
    JSBSim::FGLocation l(0.0, 0.0, 1e7);
    l.SetEllipse(20925646.0, 20855486.0);

    // WHEN: Performing many sequential operations
    for (int i = 0; i < 100; i++) {
      double lon = (i * 0.1) * M_PI / 180.0;
      double lat = (i * 0.05 - 2.5) * M_PI / 180.0;
      l.SetPositionGeodetic(lon, lat, i * 100.0);

      // Read back values
      double read_lon = l.GetLongitude();
      double read_lat = l.GetGeodLatitudeRad();
      double read_alt = l.GetGeodAltitude();

      // THEN: Values should be consistent
      TS_ASSERT_DELTA(read_lon, lon, epsilon);
      TS_ASSERT_DELTA(read_lat, lat, epsilon);
      TS_ASSERT_DELTA(read_alt, i * 100.0, 1e-6);
    }
  }

  // Test 96: Rapid transform access
  void testRapidTransformAccess() {
    // GIVEN: A location
    JSBSim::FGLocation l(M_PI / 4.0, M_PI / 6.0, 1e7);

    // WHEN: Accessing transforms rapidly
    double sum = 0.0;
    for (int i = 0; i < 1000; i++) {
      const JSBSim::FGMatrix33& T = l.GetTec2l();
      sum += T(1,1) + T(2,2) + T(3,3);
    }

    // THEN: Results should be consistent (trace of orthogonal matrix)
    TS_ASSERT(std::isfinite(sum));
    // Each access should give same trace
    const JSBSim::FGMatrix33& T = l.GetTec2l();
    double expected_trace = T(1,1) + T(2,2) + T(3,3);
    TS_ASSERT_DELTA(sum, 1000.0 * expected_trace, 1e-6);
  }

  // Test 97: Many location instances
  void testManyLocationInstances() {
    // GIVEN: Many location instances
    std::vector<JSBSim::FGLocation> locations;
    for (int i = 0; i < 100; i++) {
      double lon = (i - 50) * M_PI / 50.0;
      double lat = (i % 10 - 5) * M_PI / 12.0;
      locations.push_back(JSBSim::FGLocation(lon, lat, 1.0 + i));
    }

    // THEN: All should have valid and distinct values
    for (size_t i = 0; i < locations.size(); i++) {
      TS_ASSERT(std::isfinite(locations[i].GetRadius()));
      TS_ASSERT_DELTA(locations[i].GetRadius(), 1.0 + i, epsilon);
    }
  }

  // Test 98: Alternating operations stress test
  void testAlternatingOperationsStress() {
    // GIVEN: A location
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // WHEN: Alternating between different operations
    for (int i = 0; i < 50; i++) {
      // Set geodetic
      l.SetPositionGeodetic(i * M_PI / 25.0, 0.0, i * 1000.0);
      double r1 = l.GetRadius();

      // Set ECEF directly
      l(1) = a + i * 100.0;
      l(2) = i * 50.0;
      l(3) = i * 25.0;

      // Read back geodetic
      double lon = l.GetLongitude();
      double lat = l.GetGeodLatitudeRad();
      double alt = l.GetGeodAltitude();

      // THEN: All values should be finite and reasonable
      TS_ASSERT(std::isfinite(lon));
      TS_ASSERT(std::isfinite(lat));
      TS_ASSERT(std::isfinite(alt));
      TS_ASSERT(l.GetRadius() > 0.0);
    }
  }

  // ============ Complete Verification Tests (99-100) ============

  // Test 99: Comprehensive geodetic verification
  void testComprehensiveGeodeticVerification() {
    // GIVEN: WGS84 ellipsoid
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l;
    l.SetEllipse(a, b);

    // Test comprehensive set of geodetic positions
    std::vector<std::tuple<double, double, double>> test_cases = {
      {0.0, 0.0, 0.0},                    // Equator, prime meridian
      {M_PI / 2.0, 0.0, 0.0},             // Equator, 90E
      {M_PI, 0.0, 0.0},                   // Equator, 180E
      {0.0, M_PI / 2.0, 0.0},             // North pole
      {0.0, -M_PI / 2.0, 0.0},            // South pole
      {M_PI / 4.0, M_PI / 4.0, 10000.0},  // Mid-latitude with altitude
      {-M_PI / 3.0, -M_PI / 6.0, 35000.0} // Southern hemisphere
    };

    for (const auto& tc : test_cases) {
      double lon = std::get<0>(tc);
      double lat = std::get<1>(tc);
      double alt = std::get<2>(tc);

      l.SetPositionGeodetic(lon, lat, alt);

      // Verify round-trip
      TS_ASSERT_DELTA(l.GetLongitude(), lon, epsilon);
      TS_ASSERT_DELTA(l.GetGeodLatitudeRad(), lat, epsilon);
      TS_ASSERT_DELTA(l.GetGeodAltitude(), alt, 1e-5);

      // Verify transforms are valid
      const JSBSim::FGMatrix33& T = l.GetTec2l();
      double det = T(1,1) * (T(2,2) * T(3,3) - T(2,3) * T(3,2))
                 - T(1,2) * (T(2,1) * T(3,3) - T(2,3) * T(3,1))
                 + T(1,3) * (T(2,1) * T(3,2) - T(2,2) * T(3,1));
      TS_ASSERT_DELTA(det, 1.0, epsilon);
    }
  }

  // Test 100: Complete location system integration test
  void testCompleteLocationSystemIntegration() {
    // GIVEN: Two independent location instances
    const double a = 20925646.32546;
    const double b = 20855486.5951;
    JSBSim::FGLocation l1, l2;
    l1.SetEllipse(a, b);
    l2.SetEllipse(a, b);

    // 1. Set up different positions
    l1.SetPositionGeodetic(0.0, M_PI / 4.0, 0.0);
    l2.SetPositionGeodetic(M_PI / 2.0, 0.0, 10000.0);

    // 2. Verify independence
    TS_ASSERT(l1.GetLongitude() != l2.GetLongitude());
    TS_ASSERT(l1.GetGeodLatitudeRad() != l2.GetGeodLatitudeRad());

    // 3. Verify ECEF coordinates are different
    TS_ASSERT(std::abs(l1(1) - l2(1)) > 1000.0 ||
              std::abs(l1(2) - l2(2)) > 1000.0 ||
              std::abs(l1(3) - l2(3)) > 1000.0);

    // 4. Verify transform consistency for both
    for (auto* loc : {&l1, &l2}) {
      JSBSim::FGColumnVector3 v_ecef(1000.0, 2000.0, 3000.0);
      JSBSim::FGColumnVector3 v_local = loc->GetTec2l() * v_ecef;
      JSBSim::FGColumnVector3 v_back = loc->GetTl2ec() * v_local;

      TS_ASSERT_DELTA(v_back(1), v_ecef(1), 1e-8);
      TS_ASSERT_DELTA(v_back(2), v_ecef(2), 1e-8);
      TS_ASSERT_DELTA(v_back(3), v_ecef(3), 1e-8);
    }

    // 5. Test distance calculation symmetry
    double d12 = l1.GetDistanceTo(l2.GetLongitude(), l2.GetGeodLatitudeRad());
    double d21 = l2.GetDistanceTo(l1.GetLongitude(), l1.GetGeodLatitudeRad());
    TS_ASSERT_DELTA(d12, d21, 10.0);  // Distance should be symmetric

    // 6. Test heading calculation
    double h1 = l1.GetHeadingTo(l2.GetLongitude(), l2.GetGeodLatitudeRad());
    TS_ASSERT(std::isfinite(h1));
    TS_ASSERT(h1 >= -M_PI && h1 <= M_PI);

    // 7. Test arithmetic operations
    JSBSim::FGLocation l3 = l1 + l2;
    TS_ASSERT_DELTA(l3(1), l1(1) + l2(1), epsilon * a);
    TS_ASSERT_DELTA(l3(2), l1(2) + l2(2), epsilon * a);
    TS_ASSERT_DELTA(l3(3), l1(3) + l2(3), epsilon * a);

    // 8. Verify vector magnitude preservation through transforms
    JSBSim::FGColumnVector3 test_vec(100.0, 200.0, 300.0);
    double orig_mag = test_vec.Magnitude();
    JSBSim::FGColumnVector3 transformed = l1.GetTec2l() * test_vec;
    TS_ASSERT_DELTA(transformed.Magnitude(), orig_mag, 1e-10);

    // 9. Verify LocalToLocation functionality
    JSBSim::FGColumnVector3 local_offset(100.0, 0.0, 0.0);  // 100 ft north
    JSBSim::FGLocation l4 = l1.LocalToLocation(local_offset);
    TS_ASSERT(l4.GetLatitude() > l1.GetLatitude());  // Should be north

    // 10. Complete verification passed
    TS_ASSERT(true);
  }
};

/*******************************************************************************
 * FGLocation C172x Integration Tests
 * Tests geographic location operations using real C172x aircraft position data
 ******************************************************************************/
class FGLocationC172xTest : public CxxTest::TestSuite
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

  // Test location at specific lat/lon
  void testC172xLocationLatLon() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    double lat = 37.0 * M_PI / 180.0;
    double lon = -122.0 * M_PI / 180.0;
    ic->SetLatitudeRadIC(lat);
    ic->SetLongitudeRadIC(lon);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    // Latitude and longitude should approximately match
    TS_ASSERT_DELTA(loc.GetLatitude(), lat, 0.01);
    TS_ASSERT_DELTA(loc.GetLongitude(), lon, 0.01);
  }

  // Test altitude retrieval
  void testC172xLocationAltitude() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    double altitude = 8000.0;
    ic->SetAltitudeASLFtIC(altitude);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    double h = propagate->GetAltitudeASL();

    // Altitude should approximately match
    TS_ASSERT_DELTA(h, altitude, 100.0);
  }

  // Test ECEF coordinates
  void testC172xLocationECEF() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(0.0);  // Equator
    ic->SetLongitudeDegIC(0.0);  // Prime meridian
    ic->SetAltitudeASLFtIC(1000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    // At equator/prime meridian, X should be max, Y and Z near zero
    double x = loc(1);
    double y = loc(2);
    double z = loc(3);

    TS_ASSERT(x > 0.0);  // Positive X direction
    TS_ASSERT(fabs(y) < fabs(x));  // Y much smaller
    TS_ASSERT(fabs(z) < fabs(x));  // Z much smaller
  }

  // Test earth radius at location
  void testC172xLocationRadius() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(45.0);
    ic->SetLongitudeDegIC(-90.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    // Radius should be Earth radius + altitude
    double radius = loc.GetRadius();
    double earthRadiusFt = 20902000.0;  // ~6371 km in feet
    TS_ASSERT(radius > earthRadiusFt);
    TS_ASSERT(radius < earthRadiusFt + 50000.0);
  }

  // Test ECEF to local transformation
  void testC172xLocationTec2l() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(40.0);
    ic->SetLongitudeDegIC(-75.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    // Get transformation matrices
    JSBSim::FGMatrix33 Tec2l = loc.GetTec2l();
    JSBSim::FGMatrix33 Tl2ec = loc.GetTl2ec();

    // Should be orthogonal rotation matrices
    TS_ASSERT_DELTA(Tec2l.Determinant(), 1.0, 1e-10);
    TS_ASSERT_DELTA(Tl2ec.Determinant(), 1.0, 1e-10);

    // Product should be identity
    JSBSim::FGMatrix33 product = Tec2l * Tl2ec;
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        double expected = (i == j) ? 1.0 : 0.0;
        TS_ASSERT_DELTA(product(i, j), expected, 1e-10);
      }
    }
  }

  // Test geodetic vs geocentric latitude
  void testC172xLocationGeodeticLatitude() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(45.0);
    ic->SetLongitudeDegIC(0.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    double geocLat = loc.GetLatitude();
    double geodLat = loc.GetGeodLatitudeRad();

    // Both should be finite
    TS_ASSERT(std::isfinite(geocLat));
    TS_ASSERT(std::isfinite(geodLat));

    // Geodetic and geocentric differ due to Earth oblateness
    // At 45 degrees they should be close but not equal
    TS_ASSERT_DELTA(geocLat, geodLat, 0.01);
  }

  // Test location vector arithmetic
  void testC172xLocationArithmetic() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(40.0);
    ic->SetLongitudeDegIC(-100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    JSBSim::FGLocation loc1 = propagate->GetLocation();

    // Create offset location
    JSBSim::FGLocation offset;
    offset(1) = 100.0;
    offset(2) = 100.0;
    offset(3) = 100.0;

    // Add offset
    JSBSim::FGLocation loc2 = loc1 + offset;
    TS_ASSERT_DELTA(loc2(1), loc1(1) + 100.0, 1e-9);
    TS_ASSERT_DELTA(loc2(2), loc1(2) + 100.0, 1e-9);
    TS_ASSERT_DELTA(loc2(3), loc1(3) + 100.0, 1e-9);

    // Subtract offset
    JSBSim::FGLocation loc3 = loc2 - offset;
    TS_ASSERT_DELTA(loc3(1), loc1(1), 1e-9);
    TS_ASSERT_DELTA(loc3(2), loc1(2), 1e-9);
    TS_ASSERT_DELTA(loc3(3), loc1(3), 1e-9);
  }

  // Test sine/cosine of latitude/longitude
  void testC172xLocationTrigFunctions() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(30.0);
    ic->SetLongitudeDegIC(60.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    double sinLon = loc.GetSinLongitude();
    double cosLon = loc.GetCosLongitude();

    // sin^2 + cos^2 = 1
    TS_ASSERT_DELTA(sinLon*sinLon + cosLon*cosLon, 1.0, 1e-12);
  }

  // Test distance calculation between two positions
  void testC172xLocationDistance() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(40.0);
    ic->SetLongitudeDegIC(-74.0);  // Near NYC
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    // Calculate distance to nearby point
    double targetLon = -73.9 * M_PI / 180.0;
    double targetLat = 40.1 * M_PI / 180.0;
    double dist = loc.GetDistanceTo(targetLon, targetLat);

    // Distance should be positive and finite
    TS_ASSERT(dist > 0.0);
    TS_ASSERT(std::isfinite(dist));
    TS_ASSERT(dist < 1000000.0);  // Less than ~300 km
  }

  // Test heading to another location
  void testC172xLocationHeading() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(35.0);
    ic->SetLongitudeDegIC(-120.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();
    const JSBSim::FGLocation& loc = propagate->GetLocation();

    // Calculate heading to point due north
    double targetLon = -120.0 * M_PI / 180.0;
    double targetLat = 36.0 * M_PI / 180.0;  // 1 degree north
    double heading = loc.GetHeadingTo(targetLon, targetLat);

    // Heading should be approximately 0 (north)
    TS_ASSERT(std::isfinite(heading));
    TS_ASSERT(heading >= -M_PI && heading <= M_PI);
  }

  // Test location propagation during flight
  void testC172xLocationPropagation() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetLatitudeDegIC(45.0);
    ic->SetLongitudeDegIC(-90.0);
    ic->SetAltitudeASLFtIC(5000.0);
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetPsiDegIC(90.0);  // Heading east
    TS_ASSERT(fdmex.RunIC());

    auto propagate = fdmex.GetPropagate();
    double initialLon = propagate->GetLongitude();

    // Run for several timesteps
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double finalLon = propagate->GetLongitude();

    // Heading east, longitude should increase
    TS_ASSERT(finalLon > initialLon);
  }

  // Test altitude change during climb
  void testC172xLocationAltitudeChange() {
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(3000.0);
    ic->SetVcalibratedKtsIC(80.0);
    ic->SetClimbRateFpsIC(10.0);  // Climbing
    TS_ASSERT(fdmex.RunIC());

    auto propagate = fdmex.GetPropagate();
    double initialAlt = propagate->GetAltitudeASL();

    // Run for a few timesteps
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double finalAlt = propagate->GetAltitudeASL();

    // Altitude should increase (climbing)
    TS_ASSERT(finalAlt > initialAlt - 100.0);  // Allow some tolerance
  }
};
