#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGAircraft.h>
#include <math/FGColumnVector3.h>
#include "TestAssertions.h"
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-10;

class FGAircraftTest : public CxxTest::TestSuite
{
public:
  // Test default construction with FGFDMExec
  void testConstruction() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Should be constructed successfully
    TS_ASSERT(aircraft != nullptr);
  }

  // Test wing area getter/setter
  void testWingArea() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double wingArea = 200.0;  // sq ft
    aircraft->SetWingArea(wingArea);

    TS_ASSERT_DELTA(aircraft->GetWingArea(), wingArea, epsilon);
  }

  // Test wing span getter
  void testWingSpan() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // WingSpan is typically set via Load(), test that getter works
    double wingspan = aircraft->GetWingSpan();
    TS_ASSERT(!std::isnan(wingspan));
    TS_ASSERT(wingspan >= 0.0);  // Should be non-negative
  }

  // Test mean aerodynamic chord getter
  void testMeanChord() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double cbar = aircraft->Getcbar();
    TS_ASSERT(!std::isnan(cbar));
    TS_ASSERT(cbar >= 0.0);
  }

  // Test wing incidence getters
  void testWingIncidence() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double incRad = aircraft->GetWingIncidence();
    double incDeg = aircraft->GetWingIncidenceDeg();

    TS_ASSERT(!std::isnan(incRad));
    TS_ASSERT(!std::isnan(incDeg));

    // Verify radians to degrees conversion
    double expectedDeg = incRad * 180.0 / M_PI;
    TS_ASSERT_DELTA(incDeg, expectedDeg, epsilon);
  }

  // Test horizontal tail getters
  void testHTail() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double htailArea = aircraft->GetHTailArea();
    double htailArm = aircraft->GetHTailArm();
    double lbarh = aircraft->Getlbarh();

    TS_ASSERT(!std::isnan(htailArea));
    TS_ASSERT(!std::isnan(htailArm));
    TS_ASSERT(!std::isnan(lbarh));
    TS_ASSERT(htailArea >= 0.0);
  }

  // Test vertical tail getters
  void testVTail() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double vtailArea = aircraft->GetVTailArea();
    double vtailArm = aircraft->GetVTailArm();
    double lbarv = aircraft->Getlbarv();

    TS_ASSERT(!std::isnan(vtailArea));
    TS_ASSERT(!std::isnan(vtailArm));
    TS_ASSERT(!std::isnan(lbarv));
    TS_ASSERT(vtailArea >= 0.0);
  }

  // Test tail volume coefficients
  void testTailVolumeCoefficients() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double vbarh = aircraft->Getvbarh();
    double vbarv = aircraft->Getvbarv();

    TS_ASSERT(!std::isnan(vbarh));
    TS_ASSERT(!std::isnan(vbarv));
  }

  // Test aircraft name getter/setter
  void testAircraftName() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    std::string name = "TestAircraft";
    aircraft->SetAircraftName(name);

    TS_ASSERT_EQUALS(aircraft->GetAircraftName(), name);
  }

  // Test empty aircraft name
  void testEmptyAircraftName() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetAircraftName("");
    TS_ASSERT_EQUALS(aircraft->GetAircraftName(), "");
  }

  // Test reference point (RP) getters/setters
  void testReferencePoint() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Set RP values
    aircraft->SetXYZrp(1, 100.0);
    aircraft->SetXYZrp(2, 0.0);
    aircraft->SetXYZrp(3, 50.0);

    TS_ASSERT_DELTA(aircraft->GetXYZrp(1), 100.0, epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZrp(2), 0.0, epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZrp(3), 50.0, epsilon);

    // Test vector accessor
    FGColumnVector3 rp = aircraft->GetXYZrp();
    TS_ASSERT_DELTA(rp(1), 100.0, epsilon);
    TS_ASSERT_DELTA(rp(2), 0.0, epsilon);
    TS_ASSERT_DELTA(rp(3), 50.0, epsilon);
  }

  // Test visual reference point (VRP) getters
  void testVisualReferencePoint() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // VRP is set via Load(), verify accessor works
    FGColumnVector3 vrp = aircraft->GetXYZvrp();
    TS_ASSERT(!std::isnan(vrp(1)));
    TS_ASSERT(!std::isnan(vrp(2)));
    TS_ASSERT(!std::isnan(vrp(3)));

    // Indexed accessor
    TS_ASSERT_DELTA(aircraft->GetXYZvrp(1), vrp(1), epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZvrp(2), vrp(2), epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZvrp(3), vrp(3), epsilon);
  }

  // Test eyepoint getters
  void testEyepoint() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    FGColumnVector3 ep = aircraft->GetXYZep();
    TS_ASSERT(!std::isnan(ep(1)));
    TS_ASSERT(!std::isnan(ep(2)));
    TS_ASSERT(!std::isnan(ep(3)));

    TS_ASSERT_DELTA(aircraft->GetXYZep(1), ep(1), epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZep(2), ep(2), epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZep(3), ep(3), epsilon);
  }

  // Test force aggregation from subsystems
  void testForceAggregation() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Set forces from different sources
    aircraft->in.AeroForce = FGColumnVector3(100.0, 10.0, -500.0);
    aircraft->in.PropForce = FGColumnVector3(2000.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(50.0, 25.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);

    // Run to aggregate forces
    aircraft->Run(false);

    // Total forces = sum of all component forces
    FGColumnVector3 forces = aircraft->GetForces();
    FGColumnVector3 expected = aircraft->in.AeroForce + aircraft->in.PropForce +
                               aircraft->in.GroundForce + aircraft->in.ExternalForce +
                               aircraft->in.BuoyantForce;

    TS_ASSERT_DELTA(forces(1), expected(1), epsilon);
    TS_ASSERT_DELTA(forces(2), expected(2), epsilon);
    TS_ASSERT_DELTA(forces(3), expected(3), epsilon);

    // Test indexed accessor
    TS_ASSERT_DELTA(aircraft->GetForces(1), expected(1), epsilon);
    TS_ASSERT_DELTA(aircraft->GetForces(2), expected(2), epsilon);
    TS_ASSERT_DELTA(aircraft->GetForces(3), expected(3), epsilon);
  }

  // Test moment aggregation from subsystems
  void testMomentAggregation() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Set moments from different sources
    aircraft->in.AeroMoment = FGColumnVector3(1000.0, 500.0, -200.0);
    aircraft->in.PropMoment = FGColumnVector3(100.0, 0.0, 50.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 100.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    // Run to aggregate moments
    aircraft->Run(false);

    // Total moments = sum of all component moments
    FGColumnVector3 moments = aircraft->GetMoments();
    FGColumnVector3 expected = aircraft->in.AeroMoment + aircraft->in.PropMoment +
                               aircraft->in.GroundMoment + aircraft->in.ExternalMoment +
                               aircraft->in.BuoyantMoment;

    TS_ASSERT_DELTA(moments(1), expected(1), epsilon);
    TS_ASSERT_DELTA(moments(2), expected(2), epsilon);
    TS_ASSERT_DELTA(moments(3), expected(3), epsilon);

    // Test indexed accessor
    TS_ASSERT_DELTA(aircraft->GetMoments(1), expected(1), epsilon);
    TS_ASSERT_DELTA(aircraft->GetMoments(2), expected(2), epsilon);
    TS_ASSERT_DELTA(aircraft->GetMoments(3), expected(3), epsilon);
  }

  // Test zero forces and moments
  void testZeroForcesAndMoments() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Initialize all to zero
    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    FGColumnVector3 moments = aircraft->GetMoments();

    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(3), 0.0, epsilon);
    TS_ASSERT_DELTA(moments(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moments(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moments(3), 0.0, epsilon);
  }

  // Test buoyant forces (for lighter-than-air vehicles)
  void testBuoyantForces() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Set only buoyant forces (simulating a balloon/airship)
    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 5000.0);  // Upward lift
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(100.0, 0.0, 0.0);  // Roll moment

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    FGColumnVector3 moments = aircraft->GetMoments();

    TS_ASSERT_DELTA(forces(3), 5000.0, epsilon);
    TS_ASSERT_DELTA(moments(1), 100.0, epsilon);
  }

  // Test holding mode (simulation paused)
  void testHoldingMode() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->in.AeroForce = FGColumnVector3(100.0, 0.0, 0.0);

    // Run with Holding = true
    bool result = aircraft->Run(true);

    // Run should return false (no error)
    TS_ASSERT_EQUALS(result, false);
  }

  // Test InitModel
  void testInitModel() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    bool result = aircraft->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  // Test large forces
  void testLargeForces() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Large propulsion force (e.g., rocket)
    double largeForce = 1e6;  // 1 million lbs
    aircraft->in.PropForce = FGColumnVector3(largeForce, 0.0, 0.0);
    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(1), largeForce, epsilon);
  }

  // Test negative forces (e.g., drag)
  void testNegativeForces() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Drag is negative in x-direction
    aircraft->in.AeroForce = FGColumnVector3(-500.0, 0.0, -1000.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(1), -500.0, epsilon);
    TS_ASSERT_DELTA(forces(3), -1000.0, epsilon);
  }
};
