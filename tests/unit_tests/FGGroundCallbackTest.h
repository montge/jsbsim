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
};
