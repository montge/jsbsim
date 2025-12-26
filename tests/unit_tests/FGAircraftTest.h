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

/*******************************************************************************
 * FGAircraftAdditionalTest - Extended aircraft tests
 ******************************************************************************/
class FGAircraftAdditionalTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Static Margin Tests
   ***************************************************************************/

  // Test static margin formula
  void testStaticMarginFormula() {
    // Static margin = (x_np - x_cg) / MAC
    double x_np = 120.0;   // Neutral point (inches from nose)
    double x_cg = 100.0;   // CG position
    double MAC = 50.0;     // Mean aerodynamic chord

    double staticMargin = (x_np - x_cg) / MAC;

    // Typical static margin: 5-15%
    TS_ASSERT(staticMargin > 0.0);  // Stable
    TS_ASSERT_DELTA(staticMargin, 0.4, 0.01);  // 40% MAC
  }

  // Test negative static margin (unstable)
  void testNegativeStaticMargin() {
    double x_np = 95.0;   // Neutral point
    double x_cg = 100.0;  // CG aft of NP
    double MAC = 50.0;

    double staticMargin = (x_np - x_cg) / MAC;

    // Negative = unstable
    TS_ASSERT(staticMargin < 0.0);
    TS_ASSERT_DELTA(staticMargin, -0.1, 0.01);
  }

  // Test static margin limits
  void testStaticMarginLimits() {
    double MAC = 50.0;
    double x_np = 115.0;
    double x_cg_fwd = 90.0;   // Forward CG limit
    double x_cg_aft = 110.0;  // Aft CG limit

    double sm_fwd = (x_np - x_cg_fwd) / MAC;  // At forward limit
    double sm_aft = (x_np - x_cg_aft) / MAC;  // At aft limit

    TS_ASSERT(sm_fwd > sm_aft);  // More stable at forward CG
    TS_ASSERT(sm_aft > 0);       // Still stable at aft limit
  }

  /***************************************************************************
   * Center of Gravity Tests
   ***************************************************************************/

  // Test CG range calculation
  void testCGRangeCalculation() {
    double MAC = 50.0;
    double cg_fwd_pct = 0.15;  // 15% MAC
    double cg_aft_pct = 0.35;  // 35% MAC
    double wing_le = 80.0;     // Wing LE position

    double cg_fwd = wing_le + cg_fwd_pct * MAC;
    double cg_aft = wing_le + cg_aft_pct * MAC;
    double cg_range = cg_aft - cg_fwd;

    TS_ASSERT(cg_range > 0);
    TS_ASSERT_DELTA(cg_fwd, 87.5, 0.1);
    TS_ASSERT_DELTA(cg_aft, 97.5, 0.1);
    TS_ASSERT_DELTA(cg_range, 10.0, 0.1);
  }

  // Test CG envelope at different weights
  void testCGEnvelopeWeights() {
    // CG envelope typically narrows at higher weights
    double cg_range_light = 12.0;  // inches at light weight
    double cg_range_heavy = 8.0;   // inches at heavy weight

    TS_ASSERT(cg_range_heavy < cg_range_light);
  }

  /***************************************************************************
   * Neutral Point Tests
   ***************************************************************************/

  // Test neutral point from stability derivatives
  void testNeutralPointCalculation() {
    // x_np = x_ac + (Cm_alpha / CL_alpha) * MAC
    double x_ac = 100.0;      // Aerodynamic center
    double Cm_alpha = -1.0;   // Pitch stiffness (stable)
    double CL_alpha = 5.0;    // Lift slope
    double MAC = 50.0;

    double x_np = x_ac - (Cm_alpha / CL_alpha) * MAC;

    TS_ASSERT(x_np > x_ac);  // NP aft of AC for stable aircraft
    TS_ASSERT_DELTA(x_np, 110.0, 0.1);
  }

  // Test stick-free vs stick-fixed neutral point
  void testStickFreeVsFixedNP() {
    double x_np_fixed = 115.0;  // Stick-fixed NP
    double x_np_free = 110.0;   // Stick-free NP (further forward)

    // Stick-free NP is typically forward of stick-fixed
    TS_ASSERT(x_np_free < x_np_fixed);
  }

  /***************************************************************************
   * Load Factor Tests
   ***************************************************************************/

  // Test load factor from lift and weight
  void testLoadFactorCalculation() {
    double lift = 6000.0;   // lbs
    double weight = 3000.0; // lbs

    double n = lift / weight;

    TS_ASSERT_DELTA(n, 2.0, epsilon);  // 2G maneuver
  }

  // Test maximum load factor limits
  void testMaxLoadFactorLimits() {
    // FAR 23 limits for normal category
    double n_pos_max = 3.8;
    double n_neg_max = -1.52;

    TS_ASSERT(n_pos_max > 0);
    TS_ASSERT(n_neg_max < 0);
    TS_ASSERT(std::abs(n_pos_max) > std::abs(n_neg_max));
  }

  // Test gust load factor
  void testGustLoadFactor() {
    // Simplified gust formula: delta_n = (rho * U * V * CL_alpha) / (2 * W/S)
    double rho = 0.002377; // Air density (slug/ft^3)
    double U = 30.0;       // Gust velocity (ft/s)
    double V = 150.0;      // Aircraft velocity (ft/s)
    double CL_alpha = 5.0; // Lift curve slope
    double W_S = 20.0;     // Wing loading (lb/ft^2)

    double delta_n = (rho * U * V * CL_alpha) / (2.0 * W_S);

    TS_ASSERT(delta_n > 0);
    TS_ASSERT_DELTA(delta_n, 1.336, 0.01);  // About 1.3 G increment
  }

  /***************************************************************************
   * Force Resolution Tests
   ***************************************************************************/

  // Test body to stability axis transformation
  void testBodyToStabilityAxis() {
    double alpha = 5.0 * M_PI / 180.0;  // 5 degree AOA

    double Fx_body = -100.0;  // Drag in body
    double Fz_body = -1000.0; // Lift in body

    // Transform to stability axis
    double Fx_stab = Fx_body * cos(alpha) + Fz_body * sin(alpha);
    double Fz_stab = -Fx_body * sin(alpha) + Fz_body * cos(alpha);

    // Drag (stability) should be more negative
    TS_ASSERT(Fx_stab < Fx_body);
    // Lift (stability) should be slightly different
    TS_ASSERT(!std::isnan(Fz_stab));
  }

  // Test wind axis forces
  void testWindAxisForces() {
    double lift = 3000.0;
    double drag = 300.0;
    double sideforce = 50.0;

    // In wind axis, lift is perpendicular to relative wind
    TS_ASSERT(lift > drag);  // L/D > 1 for reasonable flight
    TS_ASSERT(std::abs(sideforce) < lift);  // Side force typically small
  }

  /***************************************************************************
   * Moment Arm Tests
   ***************************************************************************/

  // Test tail moment arm effectiveness
  void testTailMomentArm() {
    double tail_force = 500.0;   // lbs
    double tail_arm = 15.0;      // ft from CG

    double pitching_moment = tail_force * tail_arm;

    TS_ASSERT_DELTA(pitching_moment, 7500.0, epsilon);
  }

  // Test thrust moment arm
  void testThrustMomentArm() {
    double thrust = 2000.0;    // lbs
    double arm = 0.5;          // ft below CG

    double nose_up_moment = thrust * arm;

    TS_ASSERT(nose_up_moment > 0);  // Thrust below CG pitches nose up
    TS_ASSERT_DELTA(nose_up_moment, 1000.0, epsilon);
  }

  /***************************************************************************
   * Dynamic Pressure Tests
   ***************************************************************************/

  // Test dynamic pressure calculation
  void testDynamicPressure() {
    double rho = 0.002377;  // slug/ft^3
    double V = 200.0;       // ft/s

    double q = 0.5 * rho * V * V;

    TS_ASSERT_DELTA(q, 47.54, 0.1);  // psf
  }

  // Test force coefficient to force conversion
  void testCoefficientToForce() {
    double CL = 0.5;
    double q = 50.0;      // psf
    double S = 180.0;     // sq ft

    double lift = CL * q * S;

    TS_ASSERT_DELTA(lift, 4500.0, epsilon);
  }

  // Test moment coefficient to moment conversion
  void testCoefficientToMoment() {
    double Cm = -0.05;
    double q = 50.0;      // psf
    double S = 180.0;     // sq ft
    double c = 5.0;       // MAC, ft

    double moment = Cm * q * S * c;

    TS_ASSERT_DELTA(moment, -2250.0, epsilon);
  }

  /***************************************************************************
   * Aircraft Type Tests
   ***************************************************************************/

  // Test typical fighter configuration
  void testFighterConfiguration() {
    double wingspan = 35.0;
    double wingArea = 350.0;
    double AR = wingspan * wingspan / wingArea;
    double wingLoading = 70.0;  // typical fighter

    TS_ASSERT(AR < 5.0);        // Low aspect ratio
    TS_ASSERT(wingLoading > 50.0); // High wing loading
  }

  // Test typical transport configuration
  void testTransportConfiguration() {
    double wingspan = 120.0;
    double wingArea = 1400.0;
    double AR = wingspan * wingspan / wingArea;
    double wingLoading = 110.0;

    TS_ASSERT(AR > 7.0);        // High aspect ratio
    TS_ASSERT(AR < 12.0);
    TS_ASSERT(wingLoading > 100.0);
  }

  // Test typical glider configuration
  void testGliderConfiguration() {
    double wingspan = 50.0;
    double wingArea = 120.0;
    double AR = wingspan * wingspan / wingArea;
    double wingLoading = 6.0;

    TS_ASSERT(AR > 15.0);       // Very high aspect ratio
    TS_ASSERT(wingLoading < 10.0); // Very low wing loading
  }

  /***************************************************************************
   * Moment Balance Tests
   ***************************************************************************/

  // Test trimmed flight condition
  void testTrimmedCondition() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Set up trimmed condition: all moments sum to zero
    aircraft->in.AeroMoment = FGColumnVector3(0.0, -500.0, 0.0);  // Wing moment
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 500.0, 0.0); // Tail trim
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(2), 0.0, epsilon);  // Pitch trimmed
  }

  // Test roll moment balance
  void testRollMomentBalance() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Aileron creates roll, dihedral effect opposes
    double aileron_roll = 500.0;
    double dihedral_roll = -500.0;

    aircraft->in.AeroMoment = FGColumnVector3(aileron_roll + dihedral_roll, 0.0, 0.0);
    aircraft->in.PropMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantMoment = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.AeroForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.PropForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.GroundForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.ExternalForce = FGColumnVector3(0.0, 0.0, 0.0);
    aircraft->in.BuoyantForce = FGColumnVector3(0.0, 0.0, 0.0);

    aircraft->Run(false);

    FGColumnVector3 moments = aircraft->GetMoments();
    TS_ASSERT_DELTA(moments(1), 0.0, epsilon);  // Roll balanced
  }
};
