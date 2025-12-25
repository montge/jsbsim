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
};
