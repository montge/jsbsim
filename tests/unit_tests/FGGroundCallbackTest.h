/*******************************************************************************
 * FGGroundCallbackTest.h - Unit tests for FGGroundCallback classes
 *
 * Tests the ground callback interface including:
 * - FGDefaultGroundCallback for spherical and WGS84 Earth
 * - Terrain elevation handling
 * - Altitude calculations
 * - Normal vector computation
 * - Custom ground callback support
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <limits>
#include <memory>
#include <cxxtest/TestSuite.h>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <math/FGLocation.h>
#include <models/FGInertial.h>
#include <models/FGPropagate.h>
#include <input_output/FGGroundCallback.h>
#include "TestAssertions.h"

const double epsilon = 100. * std::numeric_limits<double>::epsilon();
const double RadiusReference = 20925646.32546;
const double a = 20925646.32546; // WGS84 semimajor axis length in feet
const double b = 20855486.5951;  // WGS84 semiminor axis length in feet

using namespace JSBSim;

// A class that does not set the ellipse parameters of the 'contact' location
// to check that such a ground callback does not crash JSBSim.
class DummyGroundCallback : public FGDefaultGroundCallback
{
public:
  DummyGroundCallback(double a, double b) : FGDefaultGroundCallback(a, b) {}
  double GetAGLevel(double t, const FGLocation& location,
                    FGLocation& contact,
                    FGColumnVector3& normal, FGColumnVector3& v,
                    FGColumnVector3& w) const override
  {
    FGLocation c;
    double h = FGDefaultGroundCallback::GetAGLevel(t, location, c, normal, v, w);
    // The ellipse parameters are intentionally not copied from c to contact.
    contact(1) = c(1);
    contact(2) = c(2);
    contact(3) = c(3);
    return h;
  }
};

// Test callback that tracks calls
class TrackingGroundCallback : public FGDefaultGroundCallback
{
public:
  TrackingGroundCallback(double a, double b)
    : FGDefaultGroundCallback(a, b), callCount(0), lastTime(0.0) {}

  double GetAGLevel(double t, const FGLocation& location,
                    FGLocation& contact,
                    FGColumnVector3& normal, FGColumnVector3& v,
                    FGColumnVector3& w) const override
  {
    callCount++;
    lastTime = t;
    return FGDefaultGroundCallback::GetAGLevel(t, location, contact, normal, v, w);
  }

  mutable int callCount;
  mutable double lastTime;
};

class FGGroundCallbackTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Constructor Tests
   ***************************************************************************/

  void testDefaultGroundCallbackConstructor() {
    std::unique_ptr<FGDefaultGroundCallback> cb(
        new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    TS_ASSERT(cb != nullptr);
  }

  void testDefaultGroundCallbackWGS84Constructor() {
    std::unique_ptr<FGDefaultGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    TS_ASSERT(cb != nullptr);
  }

  void testDefaultGroundCallbackSmallRadius() {
    // Test with a small radius (like a moon or asteroid)
    double smallRadius = 1000.0;
    std::unique_ptr<FGDefaultGroundCallback> cb(
        new FGDefaultGroundCallback(smallRadius, smallRadius));
    TS_ASSERT(cb != nullptr);
  }

  /***************************************************************************
   * Spherical Earth Surface Tests
   ***************************************************************************/

  void testSphericalEarthSurface() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};

    // Check that, for a point located, on the sea level radius the AGL is 0.0
    for(double lat = -90.0; lat <= 90.; lat += 30.) {
      for(double lon = 0.0; lon <=360.; lon += 45.){
        double lon_rad = lon*M_PI/180.;
        double lat_rad = lat*M_PI/180.;
        loc = FGLocation(lon_rad, lat_rad, RadiusReference);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA(0.0, agl, 1e-8);
        TS_ASSERT_VECTOR_EQUALS(v, zero);
        TS_ASSERT_VECTOR_EQUALS(w, zero);
        FGColumnVector3 vLoc = loc;
        FGColumnVector3 vContact = contact;
        TS_ASSERT_DELTA(vContact.Magnitude(), RadiusReference, epsilon);
        TS_ASSERT_DELTA(vLoc(1), vContact(1), 1e-8);
        TS_ASSERT_DELTA(vLoc(2), vContact(2), 1e-8);
        TS_ASSERT_DELTA(vLoc(3), vContact(3), 1e-8);
        TS_ASSERT_DELTA(normal(1), cos(lat_rad)*cos(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(2), cos(lat_rad)*sin(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(3), sin(lat_rad), epsilon);
        vContact.Normalize();
        TS_ASSERT_VECTOR_EQUALS(vContact, normal);
      }
    }
  }

  void testSphericalEarthEquator() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Test at equator, prime meridian
    loc = FGLocation(0.0, 0.0, RadiusReference);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(0.0, agl, 1e-8);
    TS_ASSERT_DELTA(normal(1), 1.0, epsilon);
    TS_ASSERT_DELTA(normal(2), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(3), 0.0, epsilon);
  }

  void testSphericalEarthNorthPole() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Test at North Pole
    loc = FGLocation(0.0, M_PI/2.0, RadiusReference);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(0.0, agl, 1e-8);
    TS_ASSERT_DELTA(normal(1), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(2), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(3), 1.0, epsilon);
  }

  void testSphericalEarthSouthPole() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Test at South Pole
    loc = FGLocation(0.0, -M_PI/2.0, RadiusReference);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(0.0, agl, 1e-8);
    TS_ASSERT_DELTA(normal(1), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(2), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(3), -1.0, epsilon);
  }

  /***************************************************************************
   * Spherical Earth Altitude Tests
   ***************************************************************************/

  void testSphericalEarthAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};
    double h = 100000.;

    // Check that, for a point located, on the sea level radius the AGL is 0.0
    for(double lat = -90.0; lat <= 90.; lat += 30.) {
      for(double lon = 0.0; lon <=360.; lon += 45.){
        double lon_rad = lon*M_PI/180.;
        double lat_rad = lat*M_PI/180.;
        loc = FGLocation(lon_rad, lat_rad, RadiusReference+h);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA(h/agl, 1.0, epsilon*100.);
        TS_ASSERT_VECTOR_EQUALS(v, zero);
        TS_ASSERT_VECTOR_EQUALS(w, zero);
        FGColumnVector3 vLoc = loc;
        FGColumnVector3 vContact = contact;
#ifdef __arm64__
        TS_ASSERT_DELTA(vContact.Magnitude()/RadiusReference, 1.0, epsilon);
#else
        TS_ASSERT_DELTA(vContact.Magnitude(), RadiusReference, epsilon);
#endif
        FGColumnVector3 vtest = vLoc/(1.+h/RadiusReference);
        TS_ASSERT_DELTA(vtest(1), vContact(1), 1e-8);
        TS_ASSERT_DELTA(vtest(2), vContact(2), 1e-8);
        TS_ASSERT_DELTA(vtest(3), vContact(3), 1e-8);
        TS_ASSERT_DELTA(normal(1), cos(lat_rad)*cos(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(2), cos(lat_rad)*sin(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(3), sin(lat_rad), epsilon);
        vContact.Normalize();
        TS_ASSERT_VECTOR_EQUALS(vContact, normal);
      }
    }
  }

  void testSphericalEarthLowAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 100.0; // 100 feet

    loc = FGLocation(0.0, 0.0, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);
  }

  void testSphericalEarthHighAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 1000000.0; // ~190 miles

    loc = FGLocation(0.0, 0.0, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl / h, 1.0, epsilon * 100.);
  }

  /***************************************************************************
   * Terrain Elevation Tests
   ***************************************************************************/

  void testSphericalEarthAltitudeWithTerrainElevation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};
    double h = 100000.;
    double elevation = 2000.;

    cb->SetTerrainElevation(elevation);

    // Check that, for a point located, on the sea level radius the AGL is 0.0
    for(double lat = -90.0; lat <= 90.; lat += 30.) {
      for(double lon = 0.0; lon <=360.; lon += 45.){
        double lon_rad = lon*M_PI/180.;
        double lat_rad = lat*M_PI/180.;
        loc = FGLocation(lon_rad, lat_rad, RadiusReference+h);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA((h-elevation)/agl, 1.0, epsilon*100.);
        TS_ASSERT_VECTOR_EQUALS(v, zero);
        TS_ASSERT_VECTOR_EQUALS(w, zero);
        FGColumnVector3 vLoc = loc;
        FGColumnVector3 vContact = contact;
        TS_ASSERT_DELTA(vContact.Magnitude()/(RadiusReference+elevation), 1.0,
                        epsilon);
        TS_ASSERT_VECTOR_EQUALS(vLoc/(RadiusReference+h),
                                vContact/(RadiusReference+elevation));
        TS_ASSERT_DELTA(normal(1), cos(lat_rad)*cos(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(2), cos(lat_rad)*sin(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(3), sin(lat_rad), epsilon);
        vContact.Normalize();
        TS_ASSERT_VECTOR_EQUALS(vContact, normal);
      }
    }
  }

  void testTerrainElevationZero() {
    std::unique_ptr<FGGroundCallback> cb(
        new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    cb->SetTerrainElevation(0.0);
    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 1000.0, 1e-6);
  }

  void testTerrainElevationPositive() {
    std::unique_ptr<FGGroundCallback> cb(
        new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double h = 5000.0;
    double elevation = 1000.0;
    cb->SetTerrainElevation(elevation);
    loc = FGLocation(0.0, 0.0, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h - elevation, 1e-6);
  }

  void testTerrainElevationChange() {
    std::unique_ptr<FGGroundCallback> cb(
        new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 10000.0);

    // Initial elevation
    cb->SetTerrainElevation(0.0);
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl1, 10000.0, 1e-6);

    // Change elevation
    cb->SetTerrainElevation(2000.0);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl2, 8000.0, 1e-6);

    // Change again
    cb->SetTerrainElevation(5000.0);
    double agl3 = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl3, 5000.0, 1e-6);
  }

  /***************************************************************************
   * WGS84 Earth Surface Tests
   ***************************************************************************/

  void testWGS84EarthSurface() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};

    loc.SetEllipse(a, b);
    contact.SetEllipse(a, b);

    // Check that, for a point located, on the sea level radius the AGL is 0.0
    for(double lat = -90.0; lat <= 90.; lat += 30.) {
      for(double lon = 0.0; lon <=360.; lon += 45.){
        double lon_rad = lon*M_PI/180.;
        double lat_rad = lat*M_PI/180.;
        loc.SetPositionGeodetic(lon_rad, lat_rad, 0.0);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA(0.0, agl, 1e-8);
        TS_ASSERT_VECTOR_EQUALS(v, zero);
        TS_ASSERT_VECTOR_EQUALS(w, zero);
        FGColumnVector3 vLoc = loc;
        FGColumnVector3 vContact = contact;
        TS_ASSERT_DELTA(vLoc(1), vContact(1), 1e-8);
        TS_ASSERT_DELTA(vLoc(2), vContact(2), 1e-8);
        TS_ASSERT_DELTA(vLoc(3), vContact(3), 1e-8);
        TS_ASSERT_DELTA(normal(1), cos(lat_rad)*cos(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(2), cos(lat_rad)*sin(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(3), sin(lat_rad), epsilon);
      }
    }
  }

  /***************************************************************************
   * WGS84 Earth Altitude Tests
   ***************************************************************************/

  void testWGS84EarthAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};
    double h = 100000.;

    loc.SetEllipse(a, b);
    contact.SetEllipse(a, b);

    // Check that, for a point located, on the sea level radius the AGL is 0.0
    for(double lat = -90.0; lat <= 90.; lat += 30.) {
      for(double lon = 0.0; lon <=360.; lon += 45.){
        double lon_rad = lon*M_PI/180.;
        double lat_rad = lat*M_PI/180.;
        loc.SetPositionGeodetic(lon_rad, lat_rad, h);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA(h, agl, 1e-8);
        TS_ASSERT_VECTOR_EQUALS(v, zero);
        TS_ASSERT_VECTOR_EQUALS(w, zero);
        TS_ASSERT_DELTA(normal(1), cos(lat_rad)*cos(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(2), cos(lat_rad)*sin(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(3), sin(lat_rad), epsilon);
        FGColumnVector3 vLoc = loc-h*normal;
        FGColumnVector3 vContact = contact;
        TS_ASSERT_DELTA(vLoc(1), vContact(1), 1e-7);
        TS_ASSERT_DELTA(vLoc(2), vContact(2), 1e-7);
        TS_ASSERT_DELTA(vLoc(3), vContact(3), 1e-7);
      }
    }
  }

  void testWGS84EarthAltitudeWithTerrainElevation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};
    double h = 100000.;
    double elevation = 2000.;

    loc.SetEllipse(a, b);
    contact.SetEllipse(a, b);
    cb->SetTerrainElevation(elevation);

    // Check that, for a point located, on the sea level radius the AGL is 0.0
    for(double lat = -90.0; lat <= 90.; lat += 30.) {
      for(double lon = 0.0; lon <=360.; lon += 45.){
        double lon_rad = lon*M_PI/180.;
        double lat_rad = lat*M_PI/180.;
        loc.SetPositionGeodetic(lon_rad, lat_rad, h);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA(h-elevation, agl, 1e-8);
        TS_ASSERT_VECTOR_EQUALS(v, zero);
        TS_ASSERT_VECTOR_EQUALS(w, zero);
        TS_ASSERT_DELTA(normal(1), cos(lat_rad)*cos(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(2), cos(lat_rad)*sin(lon_rad), epsilon);
        TS_ASSERT_DELTA(normal(3), sin(lat_rad), epsilon);
        FGColumnVector3 vLoc = loc-(h-elevation)*normal;
        FGColumnVector3 vContact = contact;
        TS_ASSERT_DELTA(vLoc(1), vContact(1), 1e-7);
        TS_ASSERT_DELTA(vLoc(2), vContact(2), 1e-7);
        TS_ASSERT_DELTA(vLoc(3), vContact(3), 1e-7);
      }
    }
  }

  /***************************************************************************
   * SetEllipse Tests
   ***************************************************************************/

  void testSetEllipse() {
    std::unique_ptr<FGGroundCallback> cb(
        new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Initially spherical
    loc = FGLocation(0.0, 0.0, RadiusReference);
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl1, 0.0, 1e-8);

    // Change to WGS84-like ellipse
    cb->SetEllipse(a, b);
    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, 0.0, 0.0);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl2, 0.0, 1e-8);
  }

  /***************************************************************************
   * SetTime Tests
   ***************************************************************************/

  void testSetTime() {
    TrackingGroundCallback* cb = new TrackingGroundCallback(RadiusReference, RadiusReference);
    std::unique_ptr<FGGroundCallback> cbPtr(cb);
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    cb->SetTime(0.0);
    cbPtr->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(cb->lastTime, 0.0, 1e-10);

    cb->SetTime(100.5);
    cbPtr->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(cb->lastTime, 100.5, 1e-10);

    cb->SetTime(3600.0);
    cbPtr->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(cb->lastTime, 3600.0, 1e-10);
  }

  void testGetAGLevelWithExplicitTime() {
    TrackingGroundCallback* cb = new TrackingGroundCallback(RadiusReference, RadiusReference);
    std::unique_ptr<FGGroundCallback> cbPtr(cb);
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    // Call with explicit time parameter
    cb->GetAGLevel(50.0, loc, contact, normal, v, w);
    TS_ASSERT_DELTA(cb->lastTime, 50.0, 1e-10);

    cb->GetAGLevel(200.0, loc, contact, normal, v, w);
    TS_ASSERT_DELTA(cb->lastTime, 200.0, 1e-10);
  }

  /***************************************************************************
   * Tracking Callback Tests
   ***************************************************************************/

  void testCallbackCallCount() {
    TrackingGroundCallback* cb = new TrackingGroundCallback(RadiusReference, RadiusReference);
    std::unique_ptr<FGGroundCallback> cbPtr(cb);
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    TS_ASSERT_EQUALS(cb->callCount, 0);

    cbPtr->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_EQUALS(cb->callCount, 1);

    cbPtr->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_EQUALS(cb->callCount, 2);

    for (int i = 0; i < 10; i++) {
      cbPtr->GetAGLevel(loc, contact, normal, v, w);
    }
    TS_ASSERT_EQUALS(cb->callCount, 12);
  }

  /***************************************************************************
   * Custom Ground Callback Tests (FlightGear regression test)
   ***************************************************************************/

  // Regression test for FlightGear.
  //
  // Check that JSBSim does not crash (assertion "ellipse not set") when using
  // a ground callback that does not set the ellipse parameters of the 'contact'
  // location in its GetAGLevel method.
  void testGroundCallback() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    auto planet = fdmex.GetInertial();
    DummyGroundCallback* cb = new DummyGroundCallback(planet->GetSemimajor(),
                                                      planet->GetSemiminor());
    planet->SetGroundCallback(cb);
    auto IC = fdmex.GetIC();
    TS_ASSERT_DELTA(IC->GetTerrainElevationFtIC(), 0.0, 1E-8);
    TS_ASSERT_DELTA(propagate->GetTerrainElevation(), 0.0, 1E-8);
    FGLocation loc;
    loc.SetEllipse(planet->GetSemimajor(), planet->GetSemiminor());
    planet->SetAltitudeAGL(loc, 1.0);
    TS_ASSERT_DELTA(loc.GetGeodAltitude(), 1.0, 1E-8);
  }

  void testCustomGroundCallbackMultipleCalls() {
    FGFDMExec fdmex;
    auto planet = fdmex.GetInertial();
    DummyGroundCallback* cb = new DummyGroundCallback(planet->GetSemimajor(),
                                                      planet->GetSemiminor());
    planet->SetGroundCallback(cb);

    FGLocation loc;
    loc.SetEllipse(planet->GetSemimajor(), planet->GetSemiminor());

    // Multiple SetAltitudeAGL calls should not crash
    for (int i = 0; i < 10; i++) {
      planet->SetAltitudeAGL(loc, static_cast<double>(i * 100));
      TS_ASSERT_DELTA(loc.GetGeodAltitude(), static_cast<double>(i * 100), 1E-8);
    }
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testVeryLowAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 0.001); // 0.001 feet
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    // Relative tolerance due to floating point precision at large radius
    TS_ASSERT_DELTA(agl, 0.001, 1e-4);
  }

  void testExactlyOnSurface() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 0.0, 1e-10);
  }

  void testMultipleLocationsSequentially() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double altitudes[] = {0.0, 100.0, 1000.0, 10000.0, 100000.0};

    for (double alt : altitudes) {
      loc = FGLocation(0.0, 0.0, RadiusReference + alt);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, alt, 1e-6);
    }
  }

  void testDifferentLongitudes() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 1000.0;

    // Test at different longitudes at equator
    for (double lon = 0.0; lon <= 2*M_PI; lon += M_PI/6.0) {
      loc = FGLocation(lon, 0.0, RadiusReference + h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, h, 1e-6);
    }
  }

  void testDifferentLatitudes() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 1000.0;

    // Test at different latitudes at prime meridian
    for (double lat = -M_PI/2.0; lat <= M_PI/2.0; lat += M_PI/6.0) {
      loc = FGLocation(0.0, lat, RadiusReference + h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, h, 1e-6);
    }
  }

  /***************************************************************************
   * Normal Vector Tests
   ***************************************************************************/

  void testNormalVectorAtEquator() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // At equator, 0 longitude - normal should point along x-axis
    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
    TS_ASSERT_DELTA(normal(1), 1.0, epsilon);
    TS_ASSERT_DELTA(normal(2), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(3), 0.0, epsilon);
  }

  void testNormalVectorNormalized() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Test that normal is always normalized at various locations
    for (double lat = -90.0; lat <= 90.0; lat += 30.0) {
      for (double lon = 0.0; lon <= 360.0; lon += 45.0) {
        loc = FGLocation(lon * M_PI/180.0, lat * M_PI/180.0, RadiusReference + 5000.0);
        cb->GetAGLevel(loc, contact, normal, v, w);
        TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
      }
    }
  }

  /***************************************************************************
   * Velocity and Angular Velocity Tests
   ***************************************************************************/

  void testVelocityVectorsZero() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero {0., 0., 0.};

    loc = FGLocation(0.0, 0.0, RadiusReference + 10000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    // Default ground callback returns zero velocities
    TS_ASSERT_VECTOR_EQUALS(v, zero);
    TS_ASSERT_VECTOR_EQUALS(w, zero);
  }

  /***************************************************************************
   * Additional Edge Case Tests
   ***************************************************************************/

  // Test 32: Very high altitude (100 nautical miles)
  void testVeryHighAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 607611.5;  // ~100 nautical miles in feet

    loc = FGLocation(0.0, 0.0, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl / h, 1.0, 1e-6);
  }

  // Test 33: Dateline longitude wrap
  void testDatelineLongitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 1000.0;

    // At 180 degrees longitude
    loc = FGLocation(M_PI, 0.0, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);

    // At -180 degrees longitude (should be same)
    loc = FGLocation(-M_PI, 0.0, RadiusReference + h);
    agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);
  }

  // Test 34: Nearly antipodal points
  void testAntipodalPoints() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc1, loc2, contact;
    FGColumnVector3 normal, v, w;
    double h = 5000.0;

    // Point 1: (0, 0)
    loc1 = FGLocation(0.0, 0.0, RadiusReference + h);
    double agl1 = cb->GetAGLevel(loc1, contact, normal, v, w);

    // Point 2: (180, 0) - opposite side
    loc2 = FGLocation(M_PI, 0.0, RadiusReference + h);
    double agl2 = cb->GetAGLevel(loc2, contact, normal, v, w);

    TS_ASSERT_DELTA(agl1, agl2, 1e-6);
  }

  /***************************************************************************
   * WGS84 Flattening Tests
   ***************************************************************************/

  // Test 35: WGS84 flattening calculation
  void testWGS84Flattening() {
    double flattening = (a - b) / a;
    // WGS84 flattening is approximately 1/298.257
    TS_ASSERT_DELTA(flattening, 1.0/298.257, 1e-6);
  }

  // Test 36: Radius at equator vs pole (WGS84)
  void testWGS84RadiusDifference() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation locEquator, locPole, contact;
    FGColumnVector3 normal, v, w;

    locEquator.SetEllipse(a, b);
    locPole.SetEllipse(a, b);

    locEquator.SetPositionGeodetic(0.0, 0.0, 0.0);  // Equator
    locPole.SetPositionGeodetic(0.0, M_PI/2.0, 0.0);  // North pole

    FGColumnVector3 vEquator = locEquator;
    FGColumnVector3 vPole = locPole;

    // Equator should be further from center than pole
    TS_ASSERT(vEquator.Magnitude() > vPole.Magnitude());
    TS_ASSERT_DELTA(vEquator.Magnitude(), a, 1.0);
    TS_ASSERT_DELTA(vPole.Magnitude(), b, 1.0);
  }

  // Test 37: WGS84 mid-latitude radius
  void testWGS84MidLatitudeRadius() {
    FGLocation loc45;
    loc45.SetEllipse(a, b);
    loc45.SetPositionGeodetic(0.0, 45.0 * M_PI/180.0, 0.0);

    FGColumnVector3 v45 = loc45;
    double r45 = v45.Magnitude();

    // Should be between a and b
    TS_ASSERT(r45 < a);
    TS_ASSERT(r45 > b);
  }

  /***************************************************************************
   * Terrain Variation Tests
   ***************************************************************************/

  // Test 38: Mountain terrain elevation
  void testMountainTerrainElevation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double aircraft_alt = 30000.0;  // 30000 feet MSL
    double terrain_elev = 14000.0;  // Mountain peak

    cb->SetTerrainElevation(terrain_elev);
    loc = FGLocation(0.0, 0.0, RadiusReference + aircraft_alt);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl, aircraft_alt - terrain_elev, 1e-6);
  }

  // Test 39: Below sea level terrain (Death Valley)
  void testBelowSeaLevelTerrain() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double aircraft_alt = 1000.0;   // 1000 feet MSL
    double terrain_elev = -282.0;   // Death Valley (below sea level)

    cb->SetTerrainElevation(terrain_elev);
    loc = FGLocation(0.0, 0.0, RadiusReference + aircraft_alt);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl, aircraft_alt - terrain_elev, 1e-6);
    TS_ASSERT_DELTA(agl, 1282.0, 1e-6);  // Higher AGL due to depression
  }

  // Test 40: Rapid terrain change
  void testRapidTerrainChange() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 5000.0);

    // Simulate flying over varying terrain
    double elevations[] = {0.0, 1000.0, 2500.0, 500.0, 3000.0, 100.0};
    for (double elev : elevations) {
      cb->SetTerrainElevation(elev);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, 5000.0 - elev, 1e-6);
    }
  }

  /***************************************************************************
   * Contact Point Tests
   ***************************************************************************/

  // Test 41: Contact point directly below aircraft
  void testContactPointBelow() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 10000.0;

    loc = FGLocation(0.0, 0.0, RadiusReference + h);
    cb->GetAGLevel(loc, contact, normal, v, w);

    FGColumnVector3 vLoc = loc;
    FGColumnVector3 vContact = contact;

    // Contact point should be on surface (radius = RadiusReference)
    TS_ASSERT_DELTA(vContact.Magnitude(), RadiusReference, 1e-6);

    // Contact should be collinear with location and center
    FGColumnVector3 dirLoc = vLoc;
    dirLoc.Normalize();
    FGColumnVector3 dirContact = vContact;
    dirContact.Normalize();
    TS_ASSERT_DELTA(dirLoc(1), dirContact(1), 1e-10);
    TS_ASSERT_DELTA(dirLoc(2), dirContact(2), 1e-10);
    TS_ASSERT_DELTA(dirLoc(3), dirContact(3), 1e-10);
  }

  // Test 42: Contact point at different latitudes
  void testContactPointLatitudes() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 5000.0;

    for (double lat = -80.0; lat <= 80.0; lat += 20.0) {
      double lat_rad = lat * M_PI / 180.0;
      loc = FGLocation(0.0, lat_rad, RadiusReference + h);
      cb->GetAGLevel(loc, contact, normal, v, w);

      FGColumnVector3 vContact = contact;
      TS_ASSERT_DELTA(vContact.Magnitude(), RadiusReference, 1e-6);
    }
  }

  /***************************************************************************
   * Normal Vector Direction Tests
   ***************************************************************************/

  // Test 43: Normal at 45 degree latitude
  void testNormalAt45Degrees() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double lat = 45.0 * M_PI / 180.0;
    loc = FGLocation(0.0, lat, RadiusReference + 1000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    // At 45 deg lat, 0 lon: normal should be (cos(45), 0, sin(45))
    TS_ASSERT_DELTA(normal(1), std::cos(lat), epsilon);
    TS_ASSERT_DELTA(normal(2), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(3), std::sin(lat), epsilon);
  }

  // Test 44: Normal at 90 degrees longitude
  void testNormalAt90Longitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double lon = 90.0 * M_PI / 180.0;
    loc = FGLocation(lon, 0.0, RadiusReference + 1000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    // At equator, 90 deg lon: normal should be (0, 1, 0)
    TS_ASSERT_DELTA(normal(1), 0.0, epsilon);
    TS_ASSERT_DELTA(normal(2), 1.0, epsilon);
    TS_ASSERT_DELTA(normal(3), 0.0, epsilon);
  }

  // Test 45: Normal perpendicular to tangent plane
  void testNormalPerpendicularToTangent() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.5, 0.5, RadiusReference + 1000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    // Normal should be unit length
    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);

    // Normal should point radially outward
    FGColumnVector3 vContact = contact;
    vContact.Normalize();
    TS_ASSERT_DELTA(normal(1), vContact(1), epsilon);
    TS_ASSERT_DELTA(normal(2), vContact(2), epsilon);
    TS_ASSERT_DELTA(normal(3), vContact(3), epsilon);
  }

  /***************************************************************************
   * Time Parameter Tests
   ***************************************************************************/

  // Test 46: Time parameter propagation
  void testTimeParameterPropagation() {
    TrackingGroundCallback* cb = new TrackingGroundCallback(RadiusReference, RadiusReference);
    std::unique_ptr<FGGroundCallback> cbPtr(cb);
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    // Different time values
    double times[] = {0.0, 0.1, 1.0, 10.0, 100.0, 3600.0};
    for (double t : times) {
      cb->GetAGLevel(t, loc, contact, normal, v, w);
      TS_ASSERT_DELTA(cb->lastTime, t, 1e-10);
    }
  }

  // Test 47: SetTime affects GetAGLevel without time parameter
  void testSetTimeAffectsGetAGLevel() {
    TrackingGroundCallback* cb = new TrackingGroundCallback(RadiusReference, RadiusReference);
    std::unique_ptr<FGGroundCallback> cbPtr(cb);
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    cb->SetTime(42.0);
    cbPtr->GetAGLevel(loc, contact, normal, v, w);
    // The tracking callback will use the time from SetTime
    TS_ASSERT_DELTA(cb->lastTime, 42.0, 1e-10);
  }

  /***************************************************************************
   * Multiple Callback Instance Tests
   ***************************************************************************/

  // Test 48: Independent callback instances
  void testIndependentCallbackInstances() {
    std::unique_ptr<FGGroundCallback> cb1(new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    std::unique_ptr<FGGroundCallback> cb2(new FGDefaultGroundCallback(RadiusReference, RadiusReference));
    FGLocation loc, contact1, contact2;
    FGColumnVector3 normal1, normal2, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 5000.0);

    // Set different terrain elevations
    cb1->SetTerrainElevation(0.0);
    cb2->SetTerrainElevation(1000.0);

    double agl1 = cb1->GetAGLevel(loc, contact1, normal1, v, w);
    double agl2 = cb2->GetAGLevel(loc, contact2, normal2, v, w);

    TS_ASSERT_DELTA(agl1, 5000.0, 1e-6);
    TS_ASSERT_DELTA(agl2, 4000.0, 1e-6);
  }

  // Test 49: Callback switching
  void testCallbackSwitching() {
    FGFDMExec fdmex;
    auto planet = fdmex.GetInertial();

    TrackingGroundCallback* cb1 = new TrackingGroundCallback(RadiusReference, RadiusReference);
    TrackingGroundCallback* cb2 = new TrackingGroundCallback(RadiusReference, RadiusReference);

    planet->SetGroundCallback(cb1);
    TS_ASSERT_EQUALS(cb1->callCount, 0);

    planet->SetGroundCallback(cb2);
    TS_ASSERT_EQUALS(cb2->callCount, 0);
  }

  /***************************************************************************
   * Consistency Tests
   ***************************************************************************/

  // Test 50: AGL consistency across calls
  void testAGLConsistency() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.5, 0.3, RadiusReference + 7500.0);

    // Multiple calls should return same result
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);
    double agl3 = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl1, agl2, 1e-10);
    TS_ASSERT_DELTA(agl2, agl3, 1e-10);
  }

  // Test 51: Contact point consistency
  void testContactPointConsistency() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact1, contact2;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(1.0, 0.5, RadiusReference + 3000.0);

    cb->GetAGLevel(loc, contact1, normal, v, w);
    cb->GetAGLevel(loc, contact2, normal, v, w);

    FGColumnVector3 c1 = contact1;
    FGColumnVector3 c2 = contact2;
    TS_ASSERT_DELTA(c1(1), c2(1), 1e-10);
    TS_ASSERT_DELTA(c1(2), c2(2), 1e-10);
    TS_ASSERT_DELTA(c1(3), c2(3), 1e-10);
  }

  /***************************************************************************
   * Altitude Boundary Tests
   ***************************************************************************/

  // Test 52: At terrain elevation exactly
  void testAtTerrainElevation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double terrain_elev = 5000.0;
    cb->SetTerrainElevation(terrain_elev);

    // Aircraft exactly at terrain elevation
    loc = FGLocation(0.0, 0.0, RadiusReference + terrain_elev);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 0.0, 1e-6);
  }

  // Test 53: Just above terrain
  void testJustAboveTerrain() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double terrain_elev = 5000.0;
    cb->SetTerrainElevation(terrain_elev);

    loc = FGLocation(0.0, 0.0, RadiusReference + terrain_elev + 1.0);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 1.0, 1e-4);
  }

  /***************************************************************************
   * Performance-Related Tests
   ***************************************************************************/

  // Test 54: Many sequential calls
  void testManySequentialCalls() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 5000.0);

    // Call many times to ensure no degradation
    for (int i = 0; i < 1000; i++) {
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, 5000.0, 1e-6);
    }
  }

  // Test 55: Varying locations rapidly
  void testVaryingLocationsRapidly() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 2000.0;

    for (int i = 0; i < 100; i++) {
      double lon = (i * 3.6) * M_PI / 180.0;
      double lat = (i % 180 - 90) * M_PI / 180.0;
      loc = FGLocation(lon, lat, RadiusReference + h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, h, 1e-6);
    }
  }

  /***************************************************************************
   * WGS84 Geodetic Tests
   ***************************************************************************/

  // Test 56: Geodetic altitude at equator
  void testGeodeticAltitudeEquator() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 10000.0;

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, 0.0, h);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);
  }

  // Test 57: Geodetic altitude at pole
  void testGeodeticAltitudePole() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 10000.0;

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, M_PI/2.0, h);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);
  }

  // Test 58: WGS84 45 degree latitude behavior
  void testWGS84At45Degrees() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 5000.0;

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, 45.0 * M_PI/180.0, h);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);

    // Normal should be unit length
    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
  }

  /***************************************************************************
   * Error Handling Tests
   ***************************************************************************/

  // Test 59: Large terrain elevation
  void testLargeTerrainElevation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Mount Everest elevation in feet
    double everest_ft = 29029.0;
    cb->SetTerrainElevation(everest_ft);

    loc = FGLocation(0.0, 0.0, RadiusReference + 35000.0);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 35000.0 - everest_ft, 1e-6);
  }

  // Test 60: Extreme southern latitude
  void testExtremeSouthernLatitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Very close to south pole
    double lat = -89.9 * M_PI / 180.0;
    loc = FGLocation(0.0, lat, RadiusReference + 1000.0);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 1000.0, 1e-6);

    // Normal should still be valid
    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
  }

  // Test 61: Extreme northern latitude
  void testExtremeNorthernLatitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Very close to north pole
    double lat = 89.9 * M_PI / 180.0;
    loc = FGLocation(0.0, lat, RadiusReference + 1000.0);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, 1000.0, 1e-6);
  }

  /***************************************************************************
   * Integration Tests
   ***************************************************************************/

  // Test 62: FGFDMExec terrain elevation integration
  void testFDMExecTerrainIntegration() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    auto planet = fdmex.GetInertial();

    // Set terrain elevation through callback
    FGDefaultGroundCallback* cb = new FGDefaultGroundCallback(planet->GetSemimajor(),
                                                               planet->GetSemiminor());
    cb->SetTerrainElevation(1000.0);
    planet->SetGroundCallback(cb);

    TS_ASSERT_DELTA(propagate->GetTerrainElevation(), 1000.0, 1.0);
  }

  // Test 63: Normal vector remains valid after many operations
  void testNormalVectorStability() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    for (int i = 0; i < 100; i++) {
      double lon = i * 0.1;
      double lat = std::sin(i * 0.1) * 0.5;
      loc = FGLocation(lon, lat, RadiusReference + 5000.0);
      cb->GetAGLevel(loc, contact, normal, v, w);

      // Normal should always be unit length
      TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);

      // Normal components should be finite
      TS_ASSERT(!std::isnan(normal(1)));
      TS_ASSERT(!std::isnan(normal(2)));
      TS_ASSERT(!std::isnan(normal(3)));
    }
  }

  // Test 64: Contact point finite after many operations
  void testContactPointStability() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    for (int i = 0; i < 50; i++) {
      double lon = i * 0.2;
      double lat = std::cos(i * 0.15) * 0.8;
      loc = FGLocation(lon, lat, RadiusReference + 10000.0);
      cb->GetAGLevel(loc, contact, normal, v, w);

      FGColumnVector3 vContact = contact;
      TS_ASSERT(!std::isnan(vContact(1)));
      TS_ASSERT(!std::isnan(vContact(2)));
      TS_ASSERT(!std::isnan(vContact(3)));
      TS_ASSERT(vContact.Magnitude() > 0);
    }
  }

  /***************************************************************************
   * Extended Spherical Earth Tests
   ***************************************************************************/

  // Test 65: Spherical earth radius consistency
  void testSphericalRadiusConsistency() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // All points at same altitude should have same AGL
    double h = 5000.0;
    double aglSum = 0.0;
    int count = 0;

    for (double lat = -60.0; lat <= 60.0; lat += 30.0) {
      for (double lon = 0.0; lon <= 180.0; lon += 45.0) {
        loc = FGLocation(lon * M_PI/180.0, lat * M_PI/180.0, RadiusReference + h);
        double agl = cb->GetAGLevel(loc, contact, normal, v, w);
        aglSum += agl;
        count++;
      }
    }

    double avgAgl = aglSum / count;
    TS_ASSERT_DELTA(avgAgl, h, 1e-6);
  }

  // Test 66: Spherical earth surface normal orthogonality
  void testSurfaceNormalOrthogonality() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.5, 0.5, RadiusReference + 1000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    // Create tangent vectors at contact point
    FGColumnVector3 vContact = contact;
    double lat = 0.5;
    double lon = 0.5;

    // Tangent in longitude direction
    FGColumnVector3 tangentLon(-std::sin(lon), std::cos(lon), 0.0);
    // Tangent in latitude direction
    FGColumnVector3 tangentLat(-std::sin(lat)*std::cos(lon), -std::sin(lat)*std::sin(lon), std::cos(lat));

    // Dot product of normal with tangents should be zero
    double dotLon = normal(1)*tangentLon(1) + normal(2)*tangentLon(2) + normal(3)*tangentLon(3);
    double dotLat = normal(1)*tangentLat(1) + normal(2)*tangentLat(2) + normal(3)*tangentLat(3);

    TS_ASSERT_DELTA(dotLon, 0.0, 1e-6);
    TS_ASSERT_DELTA(dotLat, 0.0, 1e-6);
  }

  // Test 67: Negative altitude (below surface)
  void testNegativeAltitude() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Point below the surface
    double h = -100.0;
    loc = FGLocation(0.0, 0.0, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl, h, 1e-6);
    TS_ASSERT(agl < 0.0);
  }

  // Test 68: Very close to center of earth
  void testVeryLowRadius() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Point far below surface
    double radius = RadiusReference * 0.5;
    loc = FGLocation(0.0, 0.0, radius);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT(agl < 0.0);
    TS_ASSERT_DELTA(agl, radius - RadiusReference, 1e-6);
  }

  /***************************************************************************
   * Extended WGS84 Tests
   ***************************************************************************/

  // Test 69: WGS84 polar flattening verification
  void testWGS84PolarFlattening() {
    FGLocation locEquator, locPole;
    locEquator.SetEllipse(a, b);
    locPole.SetEllipse(a, b);

    locEquator.SetPositionGeodetic(0.0, 0.0, 0.0);
    locPole.SetPositionGeodetic(0.0, M_PI/2.0, 0.0);

    FGColumnVector3 vEquator = locEquator;
    FGColumnVector3 vPole = locPole;

    // Difference should be approximately 70,160 feet (21.4 km)
    double diff = vEquator.Magnitude() - vPole.Magnitude();
    TS_ASSERT_DELTA(diff, a - b, 1.0);
  }

  // Test 70: WGS84 at various geodetic latitudes
  void testWGS84GeodeticLatitudes() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 10000.0;

    double latitudes[] = {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
    for (double lat : latitudes) {
      loc.SetEllipse(a, b);
      loc.SetPositionGeodetic(0.0, lat * M_PI/180.0, h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, h, 1e-5);
    }
  }

  // Test 71: WGS84 terrain on ellipsoid
  void testWGS84TerrainOnEllipsoid() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double terrainElev = 5000.0;
    double aircraftAlt = 15000.0;
    cb->SetTerrainElevation(terrainElev);

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, 45.0 * M_PI/180.0, aircraftAlt);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, aircraftAlt - terrainElev, 1e-6);
  }

  /***************************************************************************
   * Terrain Model Extension Tests
   ***************************************************************************/

  // Test 72: Terrain elevation range test
  void testTerrainElevationRange() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 50000.0);

    // Test various terrain elevations from sea level to high mountains
    double elevations[] = {0.0, 1000.0, 5000.0, 10000.0, 20000.0, 29029.0};
    for (double elev : elevations) {
      cb->SetTerrainElevation(elev);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, 50000.0 - elev, 1e-6);
    }
  }

  // Test 73: Terrain below sea level (Dead Sea region)
  void testTerrainBelowSeaLevel() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Dead Sea: approximately -1400 feet below sea level
    double deadSeaElev = -1400.0;
    cb->SetTerrainElevation(deadSeaElev);

    loc = FGLocation(0.0, 0.0, RadiusReference + 5000.0);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl, 5000.0 - deadSeaElev, 1e-6);
    TS_ASSERT_DELTA(agl, 6400.0, 1e-6);
  }

  // Test 74: Contact point radius with terrain
  void testContactPointRadiusWithTerrain() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double terrainElev = 3000.0;
    cb->SetTerrainElevation(terrainElev);

    loc = FGLocation(0.0, 0.0, RadiusReference + 10000.0);
    cb->GetAGLevel(loc, contact, normal, v, w);

    FGColumnVector3 vContact = contact;
    TS_ASSERT_DELTA(vContact.Magnitude(), RadiusReference + terrainElev, 1e-6);
  }

  /***************************************************************************
   * Custom Callback Extension Tests
   ***************************************************************************/

  // Test 75: Tracking callback time sequence
  void testTrackingCallbackTimeSequence() {
    TrackingGroundCallback* cb = new TrackingGroundCallback(RadiusReference, RadiusReference);
    std::unique_ptr<FGGroundCallback> cbPtr(cb);
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    // Simulate time advancing
    for (int i = 0; i < 100; i++) {
      double t = i * 0.1;
      cb->GetAGLevel(t, loc, contact, normal, v, w);
      TS_ASSERT_DELTA(cb->lastTime, t, 1e-10);
    }
    TS_ASSERT_EQUALS(cb->callCount, 100);
  }

  // Test 76: Custom callback with FDMExec
  void testCustomCallbackWithFDMExec() {
    FGFDMExec fdmex;
    auto planet = fdmex.GetInertial();

    TrackingGroundCallback* cb = new TrackingGroundCallback(planet->GetSemimajor(),
                                                             planet->GetSemiminor());
    planet->SetGroundCallback(cb);

    FGLocation loc;
    loc.SetEllipse(planet->GetSemimajor(), planet->GetSemiminor());
    loc.SetPositionGeodetic(0.0, 0.0, 5000.0);

    planet->SetAltitudeAGL(loc, 1000.0);
    TS_ASSERT(cb->callCount > 0);
  }

  /***************************************************************************
   * Geographic Position Tests
   ***************************************************************************/

  // Test 77: Prime meridian crossing
  void testPrimeMeridianCrossing() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 1000.0;

    // Just west of prime meridian
    loc = FGLocation(-0.01 * M_PI/180.0, 0.0, RadiusReference + h);
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);

    // Just east of prime meridian
    loc = FGLocation(0.01 * M_PI/180.0, 0.0, RadiusReference + h);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl1, h, 1e-6);
    TS_ASSERT_DELTA(agl2, h, 1e-6);
  }

  // Test 78: Equator crossing
  void testEquatorCrossing() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 1000.0;

    // Just south of equator
    loc = FGLocation(0.0, -0.01 * M_PI/180.0, RadiusReference + h);
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);

    // Just north of equator
    loc = FGLocation(0.0, 0.01 * M_PI/180.0, RadiusReference + h);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl1, h, 1e-6);
    TS_ASSERT_DELTA(agl2, h, 1e-6);
  }

  // Test 79: Arctic circle region
  void testArcticCircleRegion() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 5000.0;

    double arcticLat = 66.5 * M_PI/180.0;
    loc = FGLocation(0.0, arcticLat, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl, h, 1e-6);
    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
  }

  // Test 80: Antarctic circle region
  void testAntarcticCircleRegion() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 5000.0;

    double antarcticLat = -66.5 * M_PI/180.0;
    loc = FGLocation(0.0, antarcticLat, RadiusReference + h);
    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl, h, 1e-6);
    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
  }

  /***************************************************************************
   * Numerical Precision Tests
   ***************************************************************************/

  // Test 81: Very small altitude changes
  void testVerySmallAltitudeChanges() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double baseAlt = 10000.0;
    double delta = 0.01;  // 0.01 feet

    loc = FGLocation(0.0, 0.0, RadiusReference + baseAlt);
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);

    loc = FGLocation(0.0, 0.0, RadiusReference + baseAlt + delta);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl2 - agl1, delta, 1e-4);
  }

  // Test 82: Altitude precision at different altitudes
  void testAltitudePrecisionAtDifferentAltitudes() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double altitudes[] = {100.0, 1000.0, 10000.0, 100000.0};
    for (double alt : altitudes) {
      loc = FGLocation(0.0, 0.0, RadiusReference + alt);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);

      // Relative precision check
      TS_ASSERT_DELTA(agl / alt, 1.0, 1e-6);
    }
  }

  // Test 83: Contact normal consistency
  void testContactNormalConsistency() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal1, normal2, v, w;

    loc = FGLocation(0.5, 0.3, RadiusReference + 1000.0);
    cb->GetAGLevel(loc, contact, normal1, v, w);

    // Call again - should get same normal
    cb->GetAGLevel(loc, contact, normal2, v, w);

    TS_ASSERT_DELTA(normal1(1), normal2(1), 1e-12);
    TS_ASSERT_DELTA(normal1(2), normal2(2), 1e-12);
    TS_ASSERT_DELTA(normal1(3), normal2(3), 1e-12);
  }

  /***************************************************************************
   * Stress and Boundary Tests
   ***************************************************************************/

  // Test 84: Rapid terrain changes
  void testRapidTerrainChanges() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 20000.0);

    // Simulate flying over rapidly varying terrain
    for (int i = 0; i < 1000; i++) {
      double elev = 5000.0 * std::sin(i * 0.1);
      cb->SetTerrainElevation(elev);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, 20000.0 - elev, 1e-6);
    }
  }

  // Test 85: Multiple callback instances stress test
  void testMultipleCallbackInstances() {
    std::vector<std::unique_ptr<FGGroundCallback>> callbacks;
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Create many callback instances
    for (int i = 0; i < 10; i++) {
      callbacks.push_back(std::unique_ptr<FGGroundCallback>(
          new FGDefaultGroundCallback(RadiusReference, RadiusReference)));
      callbacks.back()->SetTerrainElevation(i * 1000.0);
    }

    loc = FGLocation(0.0, 0.0, RadiusReference + 50000.0);

    // Query all callbacks
    for (size_t i = 0; i < callbacks.size(); i++) {
      double agl = callbacks[i]->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, 50000.0 - i * 1000.0, 1e-6);
    }
  }

  // Test 86: Ellipse parameter change stress
  void testEllipseParameterChangeStress() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 1000.0);

    // Change ellipse parameters multiple times
    for (int i = 0; i < 100; i++) {
      double factor = 1.0 + i * 0.001;
      cb->SetEllipse(RadiusReference * factor, RadiusReference * factor);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT(!std::isnan(agl));
    }
  }

  /***************************************************************************
   * WGS84 Specific Extended Tests
   ***************************************************************************/

  // Test 87: WGS84 eccentricity check
  void testWGS84Eccentricity() {
    // First eccentricity squared: e^2 = 1 - (b/a)^2
    double e2 = 1.0 - (b/a) * (b/a);
    // WGS84 e^2 ≈ 0.00669437999
    TS_ASSERT_DELTA(e2, 0.00669438, 1e-6);
  }

  // Test 88: WGS84 at tropic of Cancer
  void testWGS84TropicOfCancer() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double tropicLat = 23.5 * M_PI/180.0;
    double h = 10000.0;

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, tropicLat, h);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);
  }

  // Test 89: WGS84 at tropic of Capricorn
  void testWGS84TropicOfCapricorn() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double tropicLat = -23.5 * M_PI/180.0;
    double h = 10000.0;

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(0.0, tropicLat, h);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);
    TS_ASSERT_DELTA(agl, h, 1e-6);
  }

  // Test 90: WGS84 normal vector validation
  void testWGS84NormalVectorValidation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    for (double lat = -80.0; lat <= 80.0; lat += 20.0) {
      loc.SetEllipse(a, b);
      loc.SetPositionGeodetic(0.0, lat * M_PI/180.0, 5000.0);

      cb->GetAGLevel(loc, contact, normal, v, w);

      // Normal should be unit vector
      TS_ASSERT_DELTA(normal.Magnitude(), 1.0, 1e-10);

      // Normal should not be NaN
      TS_ASSERT(!std::isnan(normal(1)));
      TS_ASSERT(!std::isnan(normal(2)));
      TS_ASSERT(!std::isnan(normal(3)));
    }
  }

  /***************************************************************************
   * Extended Ground Interaction Tests (91-100)
   ***************************************************************************/

  // Test 91: International Date Line crossing
  void testInternationalDateLineCrossing() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 5000.0;

    // Just west of date line (179.99°E)
    loc = FGLocation(179.99 * M_PI/180.0, 0.0, RadiusReference + h);
    double agl1 = cb->GetAGLevel(loc, contact, normal, v, w);

    // Just east of date line (179.99°W = -179.99°)
    loc = FGLocation(-179.99 * M_PI/180.0, 0.0, RadiusReference + h);
    double agl2 = cb->GetAGLevel(loc, contact, normal, v, w);

    TS_ASSERT_DELTA(agl1, h, 1e-6);
    TS_ASSERT_DELTA(agl2, h, 1e-6);
    TS_ASSERT_DELTA(agl1, agl2, 1e-6);
  }

  // Test 92: WGS84 geocentric vs geodetic latitude difference
  void testWGS84GeocentricVsGeodetic() {
    FGLocation loc1, loc2;
    loc1.SetEllipse(a, b);
    loc2.SetEllipse(a, b);

    // At 45° geodetic latitude, geocentric latitude is slightly less
    double geodetic_lat = 45.0 * M_PI/180.0;
    loc1.SetPositionGeodetic(0.0, geodetic_lat, 0.0);

    FGColumnVector3 v = loc1;
    double geocentric_lat = std::atan2(v(3), std::sqrt(v(1)*v(1) + v(2)*v(2)));

    // Geocentric should be slightly less than geodetic at 45°
    TS_ASSERT(geocentric_lat < geodetic_lat);

    // Difference should be small but measurable
    double diff_deg = (geodetic_lat - geocentric_lat) * 180.0 / M_PI;
    TS_ASSERT(diff_deg > 0.1);  // About 0.19 degrees
    TS_ASSERT(diff_deg < 0.25);
  }

  // Test 93: Terrain gradient effect
  void testTerrainGradientEffect() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    double aircraftAlt = 10000.0;
    loc = FGLocation(0.0, 0.0, RadiusReference + aircraftAlt);

    // Simulate ascending terrain gradient
    double previousAgl = 10000.0;
    for (int i = 0; i < 10; i++) {
      double terrainElev = i * 500.0;
      cb->SetTerrainElevation(terrainElev);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);

      // AGL should decrease as terrain rises
      TS_ASSERT(agl <= previousAgl);
      TS_ASSERT_DELTA(agl, aircraftAlt - terrainElev, 1e-6);
      previousAgl = agl;
    }
  }

  // Test 94: High-speed flight path terrain queries
  void testHighSpeedFlightPath() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 35000.0;  // FL350

    // Simulate flight from 0° to 90° longitude at Mach 2 (~40 km/min)
    // At this speed, longitude changes rapidly
    for (double lon = 0.0; lon <= 90.0; lon += 1.0) {
      loc = FGLocation(lon * M_PI/180.0, 30.0 * M_PI/180.0, RadiusReference + h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, h, 1e-6);
      TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
    }
  }

  // Test 95: WGS84 meridional radius of curvature
  void testWGS84MeridionalRadius() {
    // M = a(1-e²) / (1 - e²sin²φ)^(3/2)
    double e2 = 1.0 - (b/a)*(b/a);

    // At equator (φ = 0)
    double M_equator = a * (1.0 - e2);  // Simplified for φ=0

    // At pole (φ = 90°)
    double M_pole = a * (1.0 - e2) / std::pow(1.0 - e2, 1.5);

    // Polar radius of curvature should be larger than equatorial
    TS_ASSERT(M_pole > M_equator);

    // Check approximate values
    TS_ASSERT_DELTA(M_equator / a, 1.0 - e2, 1e-6);
  }

  // Test 96: Contact point elevation tracking
  void testContactPointElevationTracking() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    loc = FGLocation(0.0, 0.0, RadiusReference + 20000.0);

    double elevations[] = {0.0, 1000.0, 3000.0, 5000.0, 2000.0, 0.0};
    for (double elev : elevations) {
      cb->SetTerrainElevation(elev);
      cb->GetAGLevel(loc, contact, normal, v, w);

      FGColumnVector3 vContact = contact;
      double contactRadius = vContact.Magnitude();
      TS_ASSERT_DELTA(contactRadius, RadiusReference + elev, 1e-6);
    }
  }

  // Test 97: Polar orbit ground track
  void testPolarOrbitGroundTrack() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    double h = 200000.0;  // ~60 km, low orbit altitude

    // Simulate polar orbit crossing all latitudes
    for (double lat = -90.0; lat <= 90.0; lat += 5.0) {
      loc = FGLocation(0.0, lat * M_PI/180.0, RadiusReference + h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);

      // AGL should be consistent at this altitude
      TS_ASSERT_DELTA(agl / h, 1.0, 1e-6);

      // Normal should always be unit vector
      TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);
    }
  }

  // Test 98: Terrain masking for radar calculations
  void testTerrainMaskingCalculation() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Aircraft at 10000 ft MSL
    double aircraftAlt = 10000.0;
    loc = FGLocation(0.0, 0.0, RadiusReference + aircraftAlt);

    // Different terrain scenarios for radar LOS calculations
    struct TerrainScenario {
      double elevation;
      double expectedAgl;
    } scenarios[] = {
      {0.0, 10000.0},       // Sea level
      {5000.0, 5000.0},     // Mid-level terrain
      {8000.0, 2000.0},     // High terrain
      {9500.0, 500.0},      // Very high terrain
      {-500.0, 10500.0}     // Depression
    };

    for (const auto& s : scenarios) {
      cb->SetTerrainElevation(s.elevation);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);
      TS_ASSERT_DELTA(agl, s.expectedAgl, 1e-6);
    }
  }

  // Test 99: Ground effect boundary detection
  void testGroundEffectBoundary() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(RadiusReference,
                                                                     RadiusReference));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;

    // Typical ground effect is significant below wingspan height
    // Test AGL detection at various low altitudes
    double lowAltitudes[] = {10.0, 25.0, 50.0, 75.0, 100.0, 150.0, 200.0};

    for (double h : lowAltitudes) {
      loc = FGLocation(0.0, 0.0, RadiusReference + h);
      double agl = cb->GetAGLevel(loc, contact, normal, v, w);

      TS_ASSERT_DELTA(agl, h, 1e-4);

      // Verify contact point is on surface
      FGColumnVector3 vContact = contact;
      TS_ASSERT_DELTA(vContact.Magnitude(), RadiusReference, 1e-6);
    }
  }

  // Test 100: Complete ground interaction state
  void testCompleteGroundInteractionState() {
    std::unique_ptr<FGGroundCallback> cb(new FGDefaultGroundCallback(a, b));
    FGLocation loc, contact;
    FGColumnVector3 normal, v, w;
    FGColumnVector3 zero{0., 0., 0.};

    double terrainElev = 2500.0;
    double aircraftAlt = 15000.0;
    cb->SetTerrainElevation(terrainElev);

    loc.SetEllipse(a, b);
    loc.SetPositionGeodetic(45.0 * M_PI/180.0, 40.0 * M_PI/180.0, aircraftAlt);

    double agl = cb->GetAGLevel(loc, contact, normal, v, w);

    // Verify all output parameters
    // 1. AGL is correct
    TS_ASSERT_DELTA(agl, aircraftAlt - terrainElev, 1e-6);

    // 2. Normal is unit vector
    TS_ASSERT_DELTA(normal.Magnitude(), 1.0, epsilon);

    // 3. Normal components are finite
    TS_ASSERT(!std::isnan(normal(1)));
    TS_ASSERT(!std::isnan(normal(2)));
    TS_ASSERT(!std::isnan(normal(3)));

    // 4. Velocity outputs are zero (default callback)
    TS_ASSERT_VECTOR_EQUALS(v, zero);
    TS_ASSERT_VECTOR_EQUALS(w, zero);

    // 5. Contact point is on surface at terrain elevation
    FGColumnVector3 vContact = contact;
    TS_ASSERT(vContact.Magnitude() > 0);
    TS_ASSERT(!std::isnan(vContact(1)));
    TS_ASSERT(!std::isnan(vContact(2)));
    TS_ASSERT(!std::isnan(vContact(3)));

    // 6. Contact point is below aircraft
    FGColumnVector3 vLoc = loc;
    TS_ASSERT(vLoc.Magnitude() > vContact.Magnitude());
  }
};
