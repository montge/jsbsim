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

  /***************************************************************************
   * Aspect Ratio Tests
   ***************************************************************************/

  // Test aspect ratio calculation formula
  void testAspectRatioFormula() {
    // AR = b^2 / S
    double wingspan = 36.0;    // ft
    double wingArea = 174.0;   // sq ft

    double aspectRatio = wingspan * wingspan / wingArea;
    TS_ASSERT_DELTA(aspectRatio, 7.448, 0.01);
  }

  // Test high aspect ratio (glider-like)
  void testHighAspectRatio() {
    double wingspan = 60.0;    // ft
    double wingArea = 200.0;   // sq ft

    double aspectRatio = wingspan * wingspan / wingArea;
    TS_ASSERT(aspectRatio > 15.0);  // Gliders: 15-40
  }

  // Test low aspect ratio (fighter-like)
  void testLowAspectRatio() {
    double wingspan = 30.0;    // ft
    double wingArea = 400.0;   // sq ft

    double aspectRatio = wingspan * wingspan / wingArea;
    TS_ASSERT(aspectRatio < 4.0);  // Fighters: 2-4
  }

  /***************************************************************************
   * Wing Loading Tests
   ***************************************************************************/

  // Test wing loading calculation
  void testWingLoadingFormula() {
    // W/S = Weight / Wing Area
    double weight = 3000.0;    // lbs
    double wingArea = 174.0;   // sq ft

    double wingLoading = weight / wingArea;
    TS_ASSERT_DELTA(wingLoading, 17.24, 0.1);
  }

  // Test light aircraft wing loading
  void testLightAircraftWingLoading() {
    double weight = 2500.0;
    double wingArea = 170.0;

    double wingLoading = weight / wingArea;
    // Light aircraft: 10-20 lbs/sq ft
    TS_ASSERT(wingLoading >= 10.0);
    TS_ASSERT(wingLoading <= 25.0);
  }

  // Test high-performance wing loading
  void testHighPerformanceWingLoading() {
    double weight = 40000.0;
    double wingArea = 400.0;

    double wingLoading = weight / wingArea;
    // High performance: 80-120 lbs/sq ft
    TS_ASSERT(wingLoading > 50.0);
  }

  /***************************************************************************
   * Coordinate System Tests
   ***************************************************************************/

  // Test body axis force directions
  void testBodyAxisForces() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // X-axis: forward positive
    // Y-axis: right positive
    // Z-axis: down positive

    aircraft->in.AeroForce = FGColumnVector3(100.0, 50.0, -200.0);
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
    TS_ASSERT_DELTA(forces(1), 100.0, epsilon);   // Fx
    TS_ASSERT_DELTA(forces(2), 50.0, epsilon);    // Fy
    TS_ASSERT_DELTA(forces(3), -200.0, epsilon);  // Fz (lift is negative)
  }

  // Test body axis moment directions
  void testBodyAxisMoments() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // L: roll moment (positive = right wing down)
    // M: pitch moment (positive = nose up)
    // N: yaw moment (positive = nose right)

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(500.0, -1000.0, 200.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(1), 500.0, epsilon);   // L (roll)
    TS_ASSERT_DELTA(moments(2), -1000.0, epsilon); // M (pitch)
    TS_ASSERT_DELTA(moments(3), 200.0, epsilon);   // N (yaw)
  }

  /***************************************************************************
   * Reference Point Tests
   ***************************************************************************/

  // Test reference point update
  void testReferencePointUpdate() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Set initial RP
    aircraft->SetXYZrp(1, 100.0);
    aircraft->SetXYZrp(2, 0.0);
    aircraft->SetXYZrp(3, 50.0);

    TS_ASSERT_DELTA(aircraft->GetXYZrp(1), 100.0, epsilon);

    // Update RP
    aircraft->SetXYZrp(1, 120.0);
    TS_ASSERT_DELTA(aircraft->GetXYZrp(1), 120.0, epsilon);
  }

  // Test negative reference point coordinates
  void testNegativeReferencePoint() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetXYZrp(1, -50.0);
    aircraft->SetXYZrp(2, -25.0);
    aircraft->SetXYZrp(3, -10.0);

    TS_ASSERT_DELTA(aircraft->GetXYZrp(1), -50.0, epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZrp(2), -25.0, epsilon);
    TS_ASSERT_DELTA(aircraft->GetXYZrp(3), -10.0, epsilon);
  }

  /***************************************************************************
   * Tail Volume Coefficient Tests
   ***************************************************************************/

  // Test horizontal tail volume coefficient formula
  void testHTailVolumeFormula() {
    // Vbar_h = (S_h * l_h) / (S * c)
    double htailArea = 40.0;    // sq ft
    double htailArm = 15.0;     // ft
    double wingArea = 180.0;    // sq ft
    double meanChord = 5.0;     // ft

    double vbarh = (htailArea * htailArm) / (wingArea * meanChord);
    // Typical values: 0.5-1.0
    TS_ASSERT(vbarh > 0.4);
    TS_ASSERT(vbarh < 1.5);
  }

  // Test vertical tail volume coefficient formula
  void testVTailVolumeFormula() {
    // Vbar_v = (S_v * l_v) / (S * b)
    double vtailArea = 25.0;    // sq ft
    double vtailArm = 15.0;     // ft
    double wingArea = 180.0;    // sq ft
    double wingspan = 36.0;     // ft

    double vbarv = (vtailArea * vtailArm) / (wingArea * wingspan);
    // Typical values: 0.02-0.10
    TS_ASSERT(vbarv > 0.02);
    TS_ASSERT(vbarv < 0.15);
  }

  /***************************************************************************
   * Force Combination Tests
   ***************************************************************************/

  // Test thrust exceeds drag
  void testThrustExceedsDrag() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double drag = -500.0;   // Negative x (opposing motion)
    double thrust = 800.0;  // Positive x (forward)

    aircraft->in.AeroForce = FGColumnVector3(drag, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(thrust, 0.0, 0.0);
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
    TS_ASSERT(forces(1) > 0);  // Net forward force (accelerating)
    TS_ASSERT_DELTA(forces(1), 300.0, epsilon);
  }

  // Test lift equals weight
  void testLiftEqualsWeight() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double weight = 3000.0;  // Positive z (down)
    double lift = -3000.0;   // Negative z (up)

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, lift);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, weight);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(3), 0.0, epsilon);  // Level flight
  }

  // Test sideslip force
  void testSideslipForce() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Sideslip creates side force
    double sideForce = 200.0;

    aircraft->in.AeroForce = FGColumnVector3(0.0, sideForce, 0.0);
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
    TS_ASSERT_DELTA(forces(2), sideForce, epsilon);
  }

  /***************************************************************************
   * Moment Combination Tests
   ***************************************************************************/

  // Test pitch trim moment
  void testPitchTrimMoment() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double nosePitchingMoment = -500.0;   // Nose down
    double tailPitchingMoment = 500.0;    // Nose up (trim)

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, nosePitchingMoment, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, tailPitchingMoment, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(2), 0.0, epsilon);  // Trimmed
  }

  // Test roll due to aileron
  void testRollDueToAileron() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double rollMoment = 1000.0;  // Right wing down

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(rollMoment, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(1), rollMoment, epsilon);
  }

  // Test adverse yaw
  void testAdverseYaw() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Roll right creates left yaw (adverse)
    double rollMoment = 500.0;   // Right roll
    double yawMoment = -100.0;   // Left yaw (adverse)

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(rollMoment, 0.0, yawMoment);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT(moments(1) > 0);   // Rolling right
    TS_ASSERT(moments(3) < 0);   // Yawing left (adverse)
  }

  /***************************************************************************
   * Ground Reaction Tests
   ***************************************************************************/

  // Test ground normal force
  void testGroundNormalForce() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // On ground, normal force opposes weight
    double normalForce = -3000.0;  // Up (negative z)

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, normalForce);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(3), normalForce, epsilon);
  }

  // Test braking force
  void testBrakingForce() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double brakeForce = -500.0;  // Opposing motion

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(brakeForce, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(1), brakeForce, epsilon);
  }

  /***************************************************************************
   * Propulsion Moment Tests
   ***************************************************************************/

  // Test propeller torque
  void testPropellerTorque() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Clockwise prop creates left rolling moment
    double propTorque = -200.0;

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(propTorque, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(1), propTorque, epsilon);
  }

  // Test thrust line offset moment
  void testThrustLineOffset() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Thrust below CG creates nose-up pitching moment
    double pitchMoment = 300.0;

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, pitchMoment, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(2), pitchMoment, epsilon);
  }

  /***************************************************************************
   * Aircraft Name Tests
   ***************************************************************************/

  // Test long aircraft name
  void testLongAircraftName() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    std::string longName = "Boeing 747-400 International Passenger Aircraft";
    aircraft->SetAircraftName(longName);

    TS_ASSERT_EQUALS(aircraft->GetAircraftName(), longName);
  }

  // Test special characters in name
  void testSpecialCharactersInName() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    std::string specialName = "F/A-18E/F Super Hornet";
    aircraft->SetAircraftName(specialName);

    TS_ASSERT_EQUALS(aircraft->GetAircraftName(), specialName);
  }

  /***************************************************************************
   * Wing Geometry Tests
   ***************************************************************************/

  // Test taper ratio effect
  void testTaperRatioEffect() {
    // Taper ratio = tip chord / root chord
    double rootChord = 8.0;
    double tipChord = 4.0;

    double taperRatio = tipChord / rootChord;
    TS_ASSERT_DELTA(taperRatio, 0.5, epsilon);
    TS_ASSERT(taperRatio > 0.0);
    TS_ASSERT(taperRatio <= 1.0);
  }

  // Test mean aerodynamic chord calculation
  void testMACCalculation() {
    // Simplified MAC for trapezoidal wing
    double rootChord = 8.0;
    double tipChord = 4.0;
    double taperRatio = tipChord / rootChord;

    double mac = (2.0/3.0) * rootChord * (1 + taperRatio + taperRatio*taperRatio) / (1 + taperRatio);
    TS_ASSERT(mac > tipChord);
    TS_ASSERT(mac < rootChord);
  }

  /***************************************************************************
   * Multiple Force Source Tests
   ***************************************************************************/

  // Test all force sources contributing
  void testAllForceSourcesContributing() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->in.AeroForce = FGColumnVector3(100.0, 10.0, -500.0);
    aircraft->in.PropForce = FGColumnVector3(1000.0, 0.0, 50.0);
    aircraft->in.GroundForce = FGColumnVector3(-50.0, 0.0, -100.0);
    aircraft->in.ExternalForce = FGColumnVector3(25.0, 5.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, -50.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(1), 1075.0, epsilon);   // 100+1000-50+25+0
    TS_ASSERT_DELTA(forces(2), 15.0, epsilon);     // 10+0+0+5+0
    TS_ASSERT_DELTA(forces(3), -600.0, epsilon);   // -500+50-100+0-50
  }

  // Test all moment sources contributing
  void testAllMomentSourcesContributing() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(100.0, 200.0, 50.0);
    aircraft->in.PropMoment = FGColumnVector3(-50.0, 100.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 25.0);
    aircraft->in.ExternalMoment = FGColumnVector3(10.0, -50.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, -25.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(1), 60.0, epsilon);    // 100-50+0+10+0
    TS_ASSERT_DELTA(moments(2), 250.0, epsilon);   // 200+100+0-50+0
    TS_ASSERT_DELTA(moments(3), 50.0, epsilon);    // 50+0+25+0-25
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very large moments
  void testVeryLargeMoments() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double largeMoment = 1e8;

    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(largeMoment, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(1), largeMoment, epsilon);
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isinf(moments(1)));
  }

  // Test very small wing area
  void testVerySmallWingArea() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    double smallArea = 0.001;  // sq ft
    aircraft->SetWingArea(smallArea);

    TS_ASSERT_DELTA(aircraft->GetWingArea(), smallArea, epsilon);
  }

  // Test zero wing area (edge case)
  void testZeroWingArea() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetWingArea(0.0);
    TS_ASSERT_DELTA(aircraft->GetWingArea(), 0.0, epsilon);
  }

  // Test symmetric forces (balanced flight)
  void testSymmetricForces() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->in.AeroForce = FGColumnVector3(-300.0, 0.0, -3000.0);
    aircraft->in.PropForce = FGColumnVector3(300.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 3000.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 forces = aircraft->GetForces();
    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);   // Thrust = Drag
    TS_ASSERT_DELTA(forces(2), 0.0, epsilon);   // No side force
    TS_ASSERT_DELTA(forces(3), 0.0, epsilon);   // Lift = Weight
  }
};
