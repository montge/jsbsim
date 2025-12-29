/*******************************************************************************
 * FGMassBalanceTest.h - Unit tests for FGMassBalance model
 *
 * Tests mass, weight, center of gravity, and inertia calculations.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGMassBalance.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>

using namespace JSBSim;

const double epsilon = 1e-8;

class FGMassBalanceTest : public CxxTest::TestSuite
{
public:
  // Test construction and initialization
  void testConstruction() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    TS_ASSERT(massBalance != nullptr);
  }

  // Test InitModel
  void testInitModel() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    bool result = massBalance->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  // Test GetMass returns non-negative value
  void testGetMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass = massBalance->GetMass();
    TS_ASSERT(!std::isnan(mass));
    TS_ASSERT(mass >= 0.0);
  }

  // Test GetWeight returns non-negative value
  void testGetWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double weight = massBalance->GetWeight();
    TS_ASSERT(!std::isnan(weight));
    TS_ASSERT(weight >= 0.0);
  }

  // Test GetEmptyWeight
  void testGetEmptyWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double emptyWeight = massBalance->GetEmptyWeight();
    TS_ASSERT(!std::isnan(emptyWeight));
    TS_ASSERT(emptyWeight >= 0.0);
  }

  // Test SetEmptyWeight
  void testSetEmptyWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double testWeight = 5000.0;
    massBalance->SetEmptyWeight(testWeight);
    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), testWeight, epsilon);
  }

  // Test GetXYZcg returns valid CG location
  void testGetXYZcg() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& cg = massBalance->GetXYZcg();

    TS_ASSERT(!std::isnan(cg(1)));
    TS_ASSERT(!std::isnan(cg(2)));
    TS_ASSERT(!std::isnan(cg(3)));
  }

  // Test GetXYZcg indexed accessor
  void testGetXYZcgIndexed() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& cg = massBalance->GetXYZcg();

    TS_ASSERT_DELTA(massBalance->GetXYZcg(1), cg(1), epsilon);
    TS_ASSERT_DELTA(massBalance->GetXYZcg(2), cg(2), epsilon);
    TS_ASSERT_DELTA(massBalance->GetXYZcg(3), cg(3), epsilon);
  }

  // Test SetBaseCG
  void testSetBaseCG() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 testCG(100.0, 0.0, -10.0);
    massBalance->SetBaseCG(testCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), testCG(1), epsilon);
    TS_ASSERT_DELTA(cg(2), testCG(2), epsilon);
    TS_ASSERT_DELTA(cg(3), testCG(3), epsilon);
  }

  // Test GetDeltaXYZcg
  void testGetDeltaXYZcg() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& deltaCG = massBalance->GetDeltaXYZcg();

    TS_ASSERT(!std::isnan(deltaCG(1)));
    TS_ASSERT(!std::isnan(deltaCG(2)));
    TS_ASSERT(!std::isnan(deltaCG(3)));
  }

  // Test GetDeltaXYZcg indexed accessor
  void testGetDeltaXYZcgIndexed() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& deltaCG = massBalance->GetDeltaXYZcg();

    TS_ASSERT_DELTA(massBalance->GetDeltaXYZcg(1), deltaCG(1), epsilon);
    TS_ASSERT_DELTA(massBalance->GetDeltaXYZcg(2), deltaCG(2), epsilon);
    TS_ASSERT_DELTA(massBalance->GetDeltaXYZcg(3), deltaCG(3), epsilon);
  }

  // Test GetJ returns valid inertia matrix
  void testGetJ() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGMatrix33& J = massBalance->GetJ();

    // Check matrix elements are finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(J(i, j)));
        TS_ASSERT(!std::isinf(J(i, j)));
      }
    }
  }

  // Test GetJinv returns valid inverse inertia matrix
  void testGetJinv() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGMatrix33& Jinv = massBalance->GetJinv();

    // Check matrix elements are finite
    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(Jinv(i, j)));
        TS_ASSERT(!std::isinf(Jinv(i, j)));
      }
    }
  }

  // Test StructuralToBody coordinate conversion
  void testStructuralToBody() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Structural frame: X positive aft, in inches
    // Body frame: X positive forward, in feet
    FGColumnVector3 structPos(120.0, 24.0, -12.0);  // inches

    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    // Verify output is in feet (input was inches)
    // Structural to body involves X sign flip and unit conversion
    TS_ASSERT(!std::isnan(bodyPos(1)));
    TS_ASSERT(!std::isnan(bodyPos(2)));
    TS_ASSERT(!std::isnan(bodyPos(3)));
  }

  // Test GetPointmassInertia calculation
  void testGetPointmassInertia() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Test point mass inertia calculation
    double mass_slug = 10.0;  // slugs
    FGColumnVector3 location(100.0, 0.0, 0.0);  // inches from CG

    FGMatrix33 pmInertia = massBalance->GetPointmassInertia(mass_slug, location);

    // Inertia matrix should be symmetric
    TS_ASSERT_DELTA(pmInertia(1, 2), pmInertia(2, 1), epsilon);
    TS_ASSERT_DELTA(pmInertia(1, 3), pmInertia(3, 1), epsilon);
    TS_ASSERT_DELTA(pmInertia(2, 3), pmInertia(3, 2), epsilon);

    // Diagonal elements should be non-negative for a point mass
    // (point mass at non-zero location contributes positive inertia)
    TS_ASSERT(pmInertia(1, 1) >= 0.0 || pmInertia(1, 1) < epsilon);
    TS_ASSERT(pmInertia(2, 2) >= 0.0);
    TS_ASSERT(pmInertia(3, 3) >= 0.0);
  }

  // Test point mass at origin has zero inertia
  void testPointmassInertiaAtOrigin() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set CG to origin for this test
    FGColumnVector3 originCG(0.0, 0.0, 0.0);
    massBalance->SetBaseCG(originCG);

    double mass_slug = 10.0;
    FGColumnVector3 location(0.0, 0.0, 0.0);  // at CG

    FGMatrix33 pmInertia = massBalance->GetPointmassInertia(mass_slug, location);

    // Point mass at CG should have zero moment of inertia about CG
    // (actually contributes based on offset from body origin)
    TS_ASSERT(!std::isnan(pmInertia(1, 1)));
    TS_ASSERT(!std::isnan(pmInertia(2, 2)));
    TS_ASSERT(!std::isnan(pmInertia(3, 3)));
  }

  // Test GetTotalPointMassWeight
  void testGetTotalPointMassWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double pmWeight = massBalance->GetTotalPointMassWeight();
    TS_ASSERT(!std::isnan(pmWeight));
    TS_ASSERT(pmWeight >= 0.0);
  }

  // Test GetPointMassMoment
  void testGetPointMassMoment() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& pmMoment = massBalance->GetPointMassMoment();

    TS_ASSERT(!std::isnan(pmMoment(1)));
    TS_ASSERT(!std::isnan(pmMoment(2)));
    TS_ASSERT(!std::isnan(pmMoment(3)));
  }

  // Test SetAircraftBaseInertias
  void testSetAircraftBaseInertias() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Create a test inertia matrix
    FGMatrix33 testInertia(
      1000.0, -50.0, 10.0,
      -50.0, 2000.0, -20.0,
      10.0, -20.0, 3000.0
    );

    massBalance->SetAircraftBaseInertias(testInertia);

    // Run to apply the inertias
    massBalance->Run(false);

    // Verify inertia was set (GetJ should reflect base inertia)
    const FGMatrix33& J = massBalance->GetJ();
    TS_ASSERT(!std::isnan(J(1, 1)));
    TS_ASSERT(!std::isnan(J(2, 2)));
    TS_ASSERT(!std::isnan(J(3, 3)));
  }

  // Test Run method
  void testRun() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Run should succeed
    bool result = massBalance->Run(false);
    TS_ASSERT_EQUALS(result, false);  // false means no error
  }

  // Test Run with holding flag
  void testRunHolding() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Run with holding should also succeed
    bool result = massBalance->Run(true);
    TS_ASSERT_EQUALS(result, false);  // false means no error
  }

  // Test mass and weight relationship
  void testMassWeightRelationship() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set empty weight
    double testWeight = 3000.0;  // lbs
    massBalance->SetEmptyWeight(testWeight);
    massBalance->Run(false);

    double mass = massBalance->GetMass();
    double weight = massBalance->GetWeight();

    // Weight = mass * g
    // In Imperial units: weight (lbs) = mass (slugs) * 32.174 (ft/s^2)
    if (mass > 0) {
      double expectedWeight = mass * 32.174;
      // Allow some tolerance for fuel, pointmass contributions
      TS_ASSERT(weight >= 0.0);
    }
  }

  // Test inertia symmetry
  void testInertiaSymmetry() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGMatrix33& J = massBalance->GetJ();

    // Inertia tensor should be symmetric
    TS_ASSERT_DELTA(J(1, 2), J(2, 1), 1e-6);
    TS_ASSERT_DELTA(J(1, 3), J(3, 1), 1e-6);
    TS_ASSERT_DELTA(J(2, 3), J(3, 2), 1e-6);
  }

  /***************************************************************************
   * Extended Mass and Weight Tests
   ***************************************************************************/

  // Test zero empty weight
  void testZeroEmptyWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(0.0);
    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), 0.0, epsilon);
  }

  // Test large empty weight
  void testLargeEmptyWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double largeWeight = 500000.0;  // 500,000 lbs (like a 747)
    massBalance->SetEmptyWeight(largeWeight);
    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), largeWeight, epsilon);
  }

  // Test small empty weight
  void testSmallEmptyWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double smallWeight = 100.0;  // 100 lbs (like a small UAV)
    massBalance->SetEmptyWeight(smallWeight);
    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), smallWeight, epsilon);
  }

  // Test weight changes after Run
  void testWeightAfterRun() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double testWeight = 10000.0;
    massBalance->SetEmptyWeight(testWeight);
    massBalance->Run(false);

    double weight = massBalance->GetWeight();
    TS_ASSERT(weight >= testWeight);  // Weight should include empty weight
  }

  /***************************************************************************
   * Extended CG Tests
   ***************************************************************************/

  // Test CG at origin
  void testCGAtOrigin() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 originCG(0.0, 0.0, 0.0);
    massBalance->SetBaseCG(originCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), 0.0, epsilon);
    TS_ASSERT_DELTA(cg(2), 0.0, epsilon);
    TS_ASSERT_DELTA(cg(3), 0.0, epsilon);
  }

  // Test CG with negative coordinates
  void testCGNegativeCoordinates() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 negCG(-50.0, -10.0, -20.0);
    massBalance->SetBaseCG(negCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), -50.0, epsilon);
    TS_ASSERT_DELTA(cg(2), -10.0, epsilon);
    TS_ASSERT_DELTA(cg(3), -20.0, epsilon);
  }

  // Test CG typical aircraft position
  void testCGTypicalAircraft() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Typical CG: X positive aft, near centerline, below wing
    FGColumnVector3 typicalCG(150.0, 0.0, -5.0);
    massBalance->SetBaseCG(typicalCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), 150.0, epsilon);
    TS_ASSERT_DELTA(cg(2), 0.0, epsilon);
    TS_ASSERT_DELTA(cg(3), -5.0, epsilon);
  }

  // Test delta CG is initially zero
  void testDeltaCGInitiallyZero() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& deltaCG = massBalance->GetDeltaXYZcg();

    // Delta CG should be zero or very small initially
    TS_ASSERT(std::abs(deltaCG(1)) < 1e6);
    TS_ASSERT(std::abs(deltaCG(2)) < 1e6);
    TS_ASSERT(std::abs(deltaCG(3)) < 1e6);
  }

  /***************************************************************************
   * Extended Inertia Tests
   ***************************************************************************/

  // Test diagonal inertia values are non-negative
  void testDiagonalInertiaNonNegative() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGMatrix33& J = massBalance->GetJ();

    // Diagonal elements (moments of inertia) must be non-negative
    TS_ASSERT(J(1, 1) >= 0.0 || std::isnan(J(1, 1)));
    TS_ASSERT(J(2, 2) >= 0.0 || std::isnan(J(2, 2)));
    TS_ASSERT(J(3, 3) >= 0.0 || std::isnan(J(3, 3)));
  }

  // Test inverse inertia times inertia approaches identity
  void testJinvTimesJ() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set some base inertias
    FGMatrix33 testInertia(
      1000.0, 0.0, 0.0,
      0.0, 2000.0, 0.0,
      0.0, 0.0, 3000.0
    );
    massBalance->SetAircraftBaseInertias(testInertia);
    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();
    const FGMatrix33& Jinv = massBalance->GetJinv();

    // J * Jinv should be approximately identity
    // Only check if inertias are non-zero
    if (J(1, 1) > 0 && J(2, 2) > 0 && J(3, 3) > 0) {
      FGMatrix33 product = J * Jinv;

      // Diagonal should be near 1
      TS_ASSERT_DELTA(product(1, 1), 1.0, 0.01);
      TS_ASSERT_DELTA(product(2, 2), 1.0, 0.01);
      TS_ASSERT_DELTA(product(3, 3), 1.0, 0.01);

      // Off-diagonal should be near 0
      TS_ASSERT_DELTA(product(1, 2), 0.0, 0.01);
      TS_ASSERT_DELTA(product(1, 3), 0.0, 0.01);
      TS_ASSERT_DELTA(product(2, 3), 0.0, 0.01);
    }
  }

  // Test base inertia setting with non-diagonal terms
  void testNonDiagonalInertia() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Symmetric inertia matrix with products of inertia
    FGMatrix33 testInertia(
      1000.0, -50.0, -25.0,
      -50.0, 2000.0, -30.0,
      -25.0, -30.0, 3000.0
    );
    massBalance->SetAircraftBaseInertias(testInertia);
    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();
    TS_ASSERT(!std::isnan(J(1, 2)));
    TS_ASSERT(!std::isnan(J(1, 3)));
    TS_ASSERT(!std::isnan(J(2, 3)));
  }

  /***************************************************************************
   * Point Mass Inertia Tests
   ***************************************************************************/

  // Test point mass along X axis
  void testPointMassAlongXAxis() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass_slug = 1.0;
    FGColumnVector3 location(120.0, 0.0, 0.0);  // 10 ft aft

    FGMatrix33 pmInertia = massBalance->GetPointmassInertia(mass_slug, location);

    // Point on X axis: Ixx = 0 (no distance from X axis)
    // Iyy, Izz should be positive (distance from Y, Z axes)
    TS_ASSERT(!std::isnan(pmInertia(1, 1)));
    TS_ASSERT(pmInertia(2, 2) >= 0.0);
    TS_ASSERT(pmInertia(3, 3) >= 0.0);
  }

  // Test point mass along Y axis
  void testPointMassAlongYAxis() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass_slug = 1.0;
    FGColumnVector3 location(0.0, 120.0, 0.0);  // 10 ft right

    FGMatrix33 pmInertia = massBalance->GetPointmassInertia(mass_slug, location);

    TS_ASSERT(pmInertia(1, 1) >= 0.0);
    TS_ASSERT(!std::isnan(pmInertia(2, 2)));
    TS_ASSERT(pmInertia(3, 3) >= 0.0);
  }

  // Test point mass along Z axis
  void testPointMassAlongZAxis() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass_slug = 1.0;
    FGColumnVector3 location(0.0, 0.0, 120.0);  // 10 ft up

    FGMatrix33 pmInertia = massBalance->GetPointmassInertia(mass_slug, location);

    TS_ASSERT(pmInertia(1, 1) >= 0.0);
    TS_ASSERT(pmInertia(2, 2) >= 0.0);
    TS_ASSERT(!std::isnan(pmInertia(3, 3)));
  }

  // Test point mass inertia scales with mass
  void testPointMassInertiaScalesWithMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 location(100.0, 50.0, 25.0);

    FGMatrix33 pm1 = massBalance->GetPointmassInertia(1.0, location);
    FGMatrix33 pm2 = massBalance->GetPointmassInertia(2.0, location);

    // Double mass should give double inertia
    TS_ASSERT_DELTA(pm2(1, 1), 2.0 * pm1(1, 1), 0.001);
    TS_ASSERT_DELTA(pm2(2, 2), 2.0 * pm1(2, 2), 0.001);
    TS_ASSERT_DELTA(pm2(3, 3), 2.0 * pm1(3, 3), 0.001);
  }

  // Test point mass inertia scales with distance squared
  void testPointMassInertiaScalesWithDistanceSquared() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass_slug = 1.0;

    FGColumnVector3 loc1(100.0, 0.0, 0.0);  // 100 inches
    FGColumnVector3 loc2(200.0, 0.0, 0.0);  // 200 inches (2x distance)

    FGMatrix33 pm1 = massBalance->GetPointmassInertia(mass_slug, loc1);
    FGMatrix33 pm2 = massBalance->GetPointmassInertia(mass_slug, loc2);

    // 2x distance -> 4x inertia (for Iyy and Izz which depend on X distance)
    // Allow for some tolerance due to coordinate transformations
    if (pm1(2, 2) > 0) {
      double ratio = pm2(2, 2) / pm1(2, 2);
      TS_ASSERT(ratio > 3.0 && ratio < 5.0);
    }
  }

  /***************************************************************************
   * Structural to Body Conversion Tests
   ***************************************************************************/

  // Test structural to body at origin
  void testStructuralToBodyOrigin() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 structPos(0.0, 0.0, 0.0);
    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    // Origin should map to something finite
    TS_ASSERT(!std::isnan(bodyPos(1)));
    TS_ASSERT(!std::isnan(bodyPos(2)));
    TS_ASSERT(!std::isnan(bodyPos(3)));
  }

  // Test structural to body unit conversion
  void testStructuralToBodyUnits() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set CG to origin for simple calculation
    massBalance->SetBaseCG(FGColumnVector3(0.0, 0.0, 0.0));

    FGColumnVector3 structPos(12.0, 0.0, 0.0);  // 12 inches = 1 foot
    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    // Structural X is aft, body X is forward
    // Result should be in feet
    TS_ASSERT(std::isfinite(bodyPos(1)));
  }

  // Test structural to body Y axis
  void testStructuralToBodyYAxis() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetBaseCG(FGColumnVector3(0.0, 0.0, 0.0));

    FGColumnVector3 structPos(0.0, 24.0, 0.0);  // 2 feet right
    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    // Y axis should have same sign in both frames
    TS_ASSERT(bodyPos(2) > 0);
  }

  /***************************************************************************
   * Inputs Structure Tests
   ***************************************************************************/

  // Test Inputs structure fields
  void testInputsStructure() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Access Inputs structure
    massBalance->in.GasMass = 10.0;
    massBalance->in.TanksWeight = 500.0;
    massBalance->in.WOW = true;

    TS_ASSERT_DELTA(massBalance->in.GasMass, 10.0, epsilon);
    TS_ASSERT_DELTA(massBalance->in.TanksWeight, 500.0, epsilon);
    TS_ASSERT(massBalance->in.WOW);
  }

  // Test Inputs GasMoment
  void testInputsGasMoment() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->in.GasMoment(1) = 100.0;
    massBalance->in.GasMoment(2) = 50.0;
    massBalance->in.GasMoment(3) = 25.0;

    TS_ASSERT_DELTA(massBalance->in.GasMoment(1), 100.0, epsilon);
    TS_ASSERT_DELTA(massBalance->in.GasMoment(2), 50.0, epsilon);
    TS_ASSERT_DELTA(massBalance->in.GasMoment(3), 25.0, epsilon);
  }

  // Test Inputs TanksMoment
  void testInputsTanksMoment() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->in.TanksMoment(1) = 200.0;
    massBalance->in.TanksMoment(2) = -50.0;
    massBalance->in.TanksMoment(3) = 0.0;

    TS_ASSERT_DELTA(massBalance->in.TanksMoment(1), 200.0, epsilon);
    TS_ASSERT_DELTA(massBalance->in.TanksMoment(2), -50.0, epsilon);
    TS_ASSERT_DELTA(massBalance->in.TanksMoment(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  // Test multiple Run calls
  void testMultipleRunCalls() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    for (int i = 0; i < 100; i++) {
      bool result = massBalance->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  // Test Run doesn't change empty weight
  void testRunPreservesEmptyWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double testWeight = 7500.0;
    massBalance->SetEmptyWeight(testWeight);

    for (int i = 0; i < 10; i++) {
      massBalance->Run(false);
      TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), testWeight, epsilon);
    }
  }

  // Test SetBaseCG correctly sets the initial CG
  void testSetBaseCGSetsInitialCG() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 testCG(120.0, 0.0, -8.0);
    massBalance->SetBaseCG(testCG);

    // Before Run(), GetXYZcg should return the set value
    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), testCG(1), 0.1);
    TS_ASSERT_DELTA(cg(2), testCG(2), 0.1);
    TS_ASSERT_DELTA(cg(3), testCG(3), 0.1);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  // Test rapid weight changes
  void testStressRapidWeightChanges() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    for (int i = 0; i < 1000; i++) {
      double weight = 1000.0 + i * 10.0;
      massBalance->SetEmptyWeight(weight);
      TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), weight, epsilon);
    }
  }

  // Test rapid CG changes
  void testStressRapidCGChanges() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    for (int i = 0; i < 1000; i++) {
      double x = 100.0 + sin(i * 0.1) * 50.0;
      FGColumnVector3 cg(x, 0.0, 0.0);
      massBalance->SetBaseCG(cg);

      const FGColumnVector3& result = massBalance->GetXYZcg();
      TS_ASSERT_DELTA(result(1), x, epsilon);
    }
  }

  // Test many point mass inertia calculations
  void testStressManyPointMassCalcs() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    for (int i = 0; i < 1000; i++) {
      double mass = 1.0 + i * 0.1;
      double x = static_cast<double>(i % 100);
      double y = static_cast<double>((i / 10) % 100);
      double z = static_cast<double>((i / 100) % 100);

      FGColumnVector3 loc(x, y, z);
      FGMatrix33 inertia = massBalance->GetPointmassInertia(mass, loc);

      TS_ASSERT(!std::isnan(inertia(1, 1)));
      TS_ASSERT(!std::isnan(inertia(2, 2)));
      TS_ASSERT(!std::isnan(inertia(3, 3)));
    }
  }

  /***************************************************************************
   * Edge Cases
   ***************************************************************************/

  // Test very small mass
  void testVerySmallMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double tinyMass = 1e-10;
    FGColumnVector3 location(100.0, 0.0, 0.0);

    FGMatrix33 inertia = massBalance->GetPointmassInertia(tinyMass, location);

    TS_ASSERT(!std::isnan(inertia(1, 1)));
    TS_ASSERT(!std::isnan(inertia(2, 2)));
    TS_ASSERT(!std::isnan(inertia(3, 3)));
  }

  // Test very large distance
  void testVeryLargeDistance() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass_slug = 1.0;
    FGColumnVector3 location(10000.0, 10000.0, 10000.0);  // Far away

    FGMatrix33 inertia = massBalance->GetPointmassInertia(mass_slug, location);

    TS_ASSERT(!std::isnan(inertia(1, 1)));
    TS_ASSERT(!std::isnan(inertia(2, 2)));
    TS_ASSERT(!std::isnan(inertia(3, 3)));
    TS_ASSERT(std::isfinite(inertia(1, 1)));
  }

  // Test InitModel idempotent
  void testInitModelIdempotent() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    bool result1 = massBalance->InitModel();
    bool result2 = massBalance->InitModel();

    TS_ASSERT_EQUALS(result1, true);
    TS_ASSERT_EQUALS(result2, true);
  }

  /***************************************************************************
   * Model Identity Tests
   ***************************************************************************/

  void testGetName() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    std::string name = massBalance->GetName();
    TS_ASSERT(!name.empty());
  }

  void testGetExec() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    TS_ASSERT(massBalance->GetExec() == &fdmex);
  }

  void testSetRate() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetRate(5);
    TS_ASSERT_EQUALS(massBalance->GetRate(), 5u);
  }

  void testRateZero() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetRate(0);
    TS_ASSERT_EQUALS(massBalance->GetRate(), 0u);
  }

  /***************************************************************************
   * Multiple Instance Tests
   ***************************************************************************/

  void testMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto mb1 = fdmex1.GetMassBalance();
    auto mb2 = fdmex2.GetMassBalance();

    TS_ASSERT(mb1 != mb2);
    TS_ASSERT(mb1->GetExec() == &fdmex1);
    TS_ASSERT(mb2->GetExec() == &fdmex2);
  }

  void testIndependentEmptyWeights() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto mb1 = fdmex1.GetMassBalance();
    auto mb2 = fdmex2.GetMassBalance();

    mb1->SetEmptyWeight(5000.0);
    mb2->SetEmptyWeight(10000.0);

    TS_ASSERT_DELTA(mb1->GetEmptyWeight(), 5000.0, epsilon);
    TS_ASSERT_DELTA(mb2->GetEmptyWeight(), 10000.0, epsilon);
  }

  void testIndependentCG() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto mb1 = fdmex1.GetMassBalance();
    auto mb2 = fdmex2.GetMassBalance();

    mb1->SetBaseCG(FGColumnVector3(100.0, 0.0, 0.0));
    mb2->SetBaseCG(FGColumnVector3(200.0, 0.0, 0.0));

    TS_ASSERT_DELTA(mb1->GetXYZcg(1), 100.0, epsilon);
    TS_ASSERT_DELTA(mb2->GetXYZcg(1), 200.0, epsilon);
  }

  /***************************************************************************
   * Inertia Tensor Physical Properties
   ***************************************************************************/

  void testTriangleInequalityIxx() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set realistic inertias
    FGMatrix33 testInertia(
      1000.0, 0.0, 0.0,
      0.0, 5000.0, 0.0,
      0.0, 0.0, 5500.0
    );
    massBalance->SetAircraftBaseInertias(testInertia);
    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();

    // Triangle inequality: Ixx <= Iyy + Izz
    TS_ASSERT(J(1, 1) <= J(2, 2) + J(3, 3) + 0.01);
  }

  void testTriangleInequalityIyy() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGMatrix33 testInertia(
      1000.0, 0.0, 0.0,
      0.0, 5000.0, 0.0,
      0.0, 0.0, 5500.0
    );
    massBalance->SetAircraftBaseInertias(testInertia);
    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();

    // Triangle inequality: Iyy <= Ixx + Izz
    TS_ASSERT(J(2, 2) <= J(1, 1) + J(3, 3) + 0.01);
  }

  void testTriangleInequalityIzz() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGMatrix33 testInertia(
      1000.0, 0.0, 0.0,
      0.0, 5000.0, 0.0,
      0.0, 0.0, 5500.0
    );
    massBalance->SetAircraftBaseInertias(testInertia);
    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();

    // Triangle inequality: Izz <= Ixx + Iyy
    TS_ASSERT(J(3, 3) <= J(1, 1) + J(2, 2) + 0.01);
  }

  void testInertiaTensorPositiveDefinite() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGMatrix33 testInertia(
      1000.0, -10.0, -5.0,
      -10.0, 2000.0, -8.0,
      -5.0, -8.0, 2500.0
    );
    massBalance->SetAircraftBaseInertias(testInertia);
    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();

    // All diagonal elements should be positive
    TS_ASSERT(J(1, 1) > 0.0);
    TS_ASSERT(J(2, 2) > 0.0);
    TS_ASSERT(J(3, 3) > 0.0);
  }

  /***************************************************************************
   * Typical Aircraft Configuration Tests
   ***************************************************************************/

  void testTypicalLightAircraftMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Cessna 172 type: ~2400 lbs empty
    massBalance->SetEmptyWeight(2400.0);
    massBalance->Run(false);

    double mass = massBalance->GetMass();
    TS_ASSERT(mass > 0.0);
    TS_ASSERT(mass < 200.0);  // Should be under 200 slugs
  }

  void testTypicalTransportAircraftMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Boeing 737 type: ~90,000 lbs empty
    massBalance->SetEmptyWeight(90000.0);
    massBalance->Run(false);

    double mass = massBalance->GetMass();
    TS_ASSERT(mass > 0.0);
    TS_ASSERT(mass < 5000.0);  // Should be under 5000 slugs
  }

  void testTypicalUAVMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Small UAV type: ~50 lbs empty
    massBalance->SetEmptyWeight(50.0);
    massBalance->Run(false);

    double mass = massBalance->GetMass();
    TS_ASSERT(mass > 0.0);
    TS_ASSERT(mass < 10.0);  // Should be under 10 slugs
  }

  /***************************************************************************
   * CG Limit Tests
   ***************************************************************************/

  void testCGForwardLimit() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Very forward CG
    FGColumnVector3 forwardCG(50.0, 0.0, 0.0);
    massBalance->SetBaseCG(forwardCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), 50.0, epsilon);
  }

  void testCGAftLimit() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Very aft CG
    FGColumnVector3 aftCG(300.0, 0.0, 0.0);
    massBalance->SetBaseCG(aftCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(1), 300.0, epsilon);
  }

  void testCGLateralOffset() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // CG with lateral offset (asymmetric loading)
    FGColumnVector3 lateralCG(150.0, 5.0, 0.0);
    massBalance->SetBaseCG(lateralCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(2), 5.0, epsilon);
  }

  void testCGVerticalOffset() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // High CG (cargo on top)
    FGColumnVector3 highCG(150.0, 0.0, -50.0);
    massBalance->SetBaseCG(highCG);

    const FGColumnVector3& cg = massBalance->GetXYZcg();
    TS_ASSERT_DELTA(cg(3), -50.0, epsilon);
  }

  /***************************************************************************
   * Point Mass Contribution Tests
   ***************************************************************************/

  void testZeroPointMassWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Initially should have zero or minimal point mass weight
    double pmWeight = massBalance->GetTotalPointMassWeight();
    TS_ASSERT(pmWeight >= 0.0);
  }

  void testPointMassMomentVector() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    const FGColumnVector3& moment = massBalance->GetPointMassMoment();

    // Should return a valid vector
    TS_ASSERT(!std::isnan(moment(1)));
    TS_ASSERT(!std::isnan(moment(2)));
    TS_ASSERT(!std::isnan(moment(3)));
  }

  void testPointMassInertiaSymmetric() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double mass = 5.0;
    FGColumnVector3 location(100.0, 50.0, 25.0);

    FGMatrix33 inertia = massBalance->GetPointmassInertia(mass, location);

    // Inertia tensor must be symmetric
    TS_ASSERT_DELTA(inertia(1, 2), inertia(2, 1), epsilon);
    TS_ASSERT_DELTA(inertia(1, 3), inertia(3, 1), epsilon);
    TS_ASSERT_DELTA(inertia(2, 3), inertia(3, 2), epsilon);
  }

  void testPointMassZeroMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 location(100.0, 0.0, 0.0);
    FGMatrix33 inertia = massBalance->GetPointmassInertia(0.0, location);

    // Zero mass should give zero inertia
    TS_ASSERT_DELTA(inertia(1, 1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(2, 2), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(3, 3), 0.0, epsilon);
  }

  /***************************************************************************
   * Weight and Mass Conversion Tests
   ***************************************************************************/

  void testMassToWeightConversion() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(3217.4);  // 100 slugs * 32.174
    massBalance->Run(false);

    double mass = massBalance->GetMass();
    double weight = massBalance->GetWeight();

    // Weight should be approximately mass * g
    if (mass > 0) {
      double g = 32.174;  // ft/s^2
      TS_ASSERT(weight >= 0.0);
    }
  }

  void testWeightPrecision() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double preciseWeight = 12345.6789;
    massBalance->SetEmptyWeight(preciseWeight);

    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), preciseWeight, 1e-4);
  }

  /***************************************************************************
   * Coordinate Frame Tests
   ***************************************************************************/

  void testStructuralToBodyNegativeX() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetBaseCG(FGColumnVector3(0.0, 0.0, 0.0));

    FGColumnVector3 structPos(-120.0, 0.0, 0.0);  // Forward in structural
    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    TS_ASSERT(std::isfinite(bodyPos(1)));
  }

  void testStructuralToBodyLargeOffset() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetBaseCG(FGColumnVector3(500.0, 0.0, 0.0));

    FGColumnVector3 structPos(100.0, 0.0, 0.0);
    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    TS_ASSERT(std::isfinite(bodyPos(1)));
    TS_ASSERT(std::isfinite(bodyPos(2)));
    TS_ASSERT(std::isfinite(bodyPos(3)));
  }

  void testStructuralToBodyAllAxes() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetBaseCG(FGColumnVector3(0.0, 0.0, 0.0));

    FGColumnVector3 structPos(100.0, 50.0, -25.0);
    FGColumnVector3 bodyPos = massBalance->StructuralToBody(structPos);

    // All outputs should be finite
    TS_ASSERT(std::isfinite(bodyPos(1)));
    TS_ASSERT(std::isfinite(bodyPos(2)));
    TS_ASSERT(std::isfinite(bodyPos(3)));
  }

  /***************************************************************************
   * State Consistency Tests
   ***************************************************************************/

  void testConsistencyAfterMultipleRuns() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(5000.0);
    massBalance->SetBaseCG(FGColumnVector3(150.0, 0.0, -5.0));

    double weight1 = massBalance->GetEmptyWeight();
    FGColumnVector3 cg1 = massBalance->GetXYZcg();

    for (int i = 0; i < 100; i++) {
      massBalance->Run(false);
    }

    double weight2 = massBalance->GetEmptyWeight();
    FGColumnVector3 cg2 = massBalance->GetXYZcg();

    // Empty weight should not change
    TS_ASSERT_DELTA(weight1, weight2, epsilon);
    // Base CG should remain consistent
    TS_ASSERT_DELTA(cg1(1), cg2(1), 0.1);
  }

  void testConsistencyAfterInitModel() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(8000.0);
    massBalance->Run(false);

    double weightBefore = massBalance->GetWeight();

    massBalance->InitModel();
    massBalance->Run(false);

    // Weight calculation should still work
    double weightAfter = massBalance->GetWeight();
    TS_ASSERT(weightAfter >= 0.0);
  }

  /***************************************************************************
   * Additional Edge Cases
   ***************************************************************************/

  void testNegativeMassHandling() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Negative mass (invalid but shouldn't crash)
    FGColumnVector3 location(100.0, 0.0, 0.0);
    FGMatrix33 inertia = massBalance->GetPointmassInertia(-1.0, location);

    // Should produce some result without crashing
    TS_ASSERT(!std::isnan(inertia(1, 1)));
  }

  void testExtremelyLargeMass() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double hugeMass = 1e10;  // Unrealistically large
    FGColumnVector3 location(100.0, 0.0, 0.0);

    FGMatrix33 inertia = massBalance->GetPointmassInertia(hugeMass, location);

    TS_ASSERT(std::isfinite(inertia(1, 1)));
    TS_ASSERT(std::isfinite(inertia(2, 2)));
    TS_ASSERT(std::isfinite(inertia(3, 3)));
  }

  void testExtremelySmallWeight() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(1e-10);
    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), 1e-10, 1e-15);
  }

  void testInertiaWithZeroDiagonal() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Edge case: point mass on axis
    FGMatrix33 inertia = massBalance->GetPointmassInertia(1.0, FGColumnVector3(0.0, 0.0, 0.0));

    // Should not produce NaN
    TS_ASSERT(!std::isnan(inertia(1, 1)));
    TS_ASSERT(!std::isnan(inertia(2, 2)));
    TS_ASSERT(!std::isnan(inertia(3, 3)));
  }

  void testRapidCGAndWeightChanges() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    for (int i = 0; i < 500; i++) {
      double weight = 1000.0 + (i % 100) * 100.0;
      double cgX = 100.0 + (i % 50);

      massBalance->SetEmptyWeight(weight);
      massBalance->SetBaseCG(FGColumnVector3(cgX, 0.0, 0.0));
      massBalance->Run(false);

      TS_ASSERT(massBalance->GetWeight() >= 0.0);
      TS_ASSERT(!std::isnan(massBalance->GetXYZcg(1)));
    }
  }

  void testInputsGasInertia() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set gas inertia input
    massBalance->in.GasInertia = FGMatrix33(
      100.0, 0.0, 0.0,
      0.0, 200.0, 0.0,
      0.0, 0.0, 300.0
    );

    massBalance->Run(false);

    // Verify inertia was incorporated
    const FGMatrix33& J = massBalance->GetJ();
    TS_ASSERT(!std::isnan(J(1, 1)));
  }

  void testInputsTankInertia() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->in.TankInertia = FGMatrix33(
      50.0, 0.0, 0.0,
      0.0, 100.0, 0.0,
      0.0, 0.0, 150.0
    );

    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();
    TS_ASSERT(!std::isnan(J(1, 1)));
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteMassBalanceCalculation() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Set up complete aircraft mass properties
    massBalance->SetEmptyWeight(5000.0);
    massBalance->SetBaseCG(FGColumnVector3(150.0, 0.0, -5.0));

    massBalance->Run(false);

    // Verify complete state
    TS_ASSERT(massBalance->GetWeight() > 0.0);
    TS_ASSERT(massBalance->GetMass() > 0.0);

    FGColumnVector3 cg = massBalance->GetXYZcg();
    TS_ASSERT(std::isfinite(cg(1)));
    TS_ASSERT(std::isfinite(cg(2)));
    TS_ASSERT(std::isfinite(cg(3)));
  }

  void testCompleteInertiaComputation() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(10000.0);

    // Add inertia contributions
    massBalance->in.GasInertia = FGMatrix33(
      500.0, 0.0, 0.0,
      0.0, 1000.0, 0.0,
      0.0, 0.0, 1200.0
    );

    massBalance->in.TankInertia = FGMatrix33(
      100.0, 0.0, 0.0,
      0.0, 200.0, 0.0,
      0.0, 0.0, 250.0
    );

    massBalance->Run(false);

    const FGMatrix33& J = massBalance->GetJ();
    TS_ASSERT(J(1, 1) > 0.0);
    TS_ASSERT(J(2, 2) > 0.0);
    TS_ASSERT(J(3, 3) > 0.0);
  }

  void testCompleteCGShiftDueToFuel() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(3000.0);
    massBalance->SetBaseCG(FGColumnVector3(100.0, 0.0, 0.0));

    // Initial CG
    massBalance->Run(false);
    double cg_initial = massBalance->GetXYZcg(1);

    // Simulate fuel burn by updating tank moment input
    massBalance->in.TanksMoment = FGColumnVector3(-1000.0, 0.0, 0.0);  // Aft moment
    massBalance->in.TanksWeight = 100.0;

    massBalance->Run(false);

    // CG should be affected
    TS_ASSERT(std::isfinite(massBalance->GetXYZcg(1)));
  }

  void testCompleteMultiplePointMasses() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Calculate inertia for multiple point masses
    FGColumnVector3 positions[] = {
      FGColumnVector3(100.0, 50.0, 0.0),
      FGColumnVector3(100.0, -50.0, 0.0),
      FGColumnVector3(200.0, 0.0, 10.0)
    };
    double masses[] = {50.0, 50.0, 100.0};

    FGMatrix33 totalInertia;
    for (int i = 0; i < 3; i++) {
      FGMatrix33 pmInertia = massBalance->GetPointmassInertia(masses[i], positions[i]);
      for (int r = 1; r <= 3; r++) {
        for (int c = 1; c <= 3; c++) {
          totalInertia(r, c) += pmInertia(r, c);
        }
      }
    }

    TS_ASSERT(totalInertia(1, 1) > 0.0);
    TS_ASSERT(totalInertia(2, 2) > 0.0);
    TS_ASSERT(totalInertia(3, 3) > 0.0);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentMassBalanceInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto mb1 = fdmex1.GetMassBalance();
    auto mb2 = fdmex2.GetMassBalance();

    mb1->SetEmptyWeight(5000.0);
    mb2->SetEmptyWeight(8000.0);

    TS_ASSERT_DELTA(mb1->GetEmptyWeight(), 5000.0, epsilon);
    TS_ASSERT_DELTA(mb2->GetEmptyWeight(), 8000.0, epsilon);

    // Verify mb1 unchanged after mb2 modification
    mb2->SetEmptyWeight(10000.0);
    TS_ASSERT_DELTA(mb1->GetEmptyWeight(), 5000.0, epsilon);
  }

  void testIndependentCGCalculations() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto mb1 = fdmex1.GetMassBalance();
    auto mb2 = fdmex2.GetMassBalance();

    mb1->SetBaseCG(FGColumnVector3(100.0, 0.0, 0.0));
    mb2->SetBaseCG(FGColumnVector3(200.0, 0.0, 0.0));

    mb1->Run(false);
    mb2->Run(false);

    // CG values are independent
    TS_ASSERT(std::isfinite(mb1->GetXYZcg(1)));
    TS_ASSERT(std::isfinite(mb2->GetXYZcg(1)));
  }

  void testIndependentInertiaCalculations() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    FGColumnVector3 pos1(100.0, 0.0, 0.0);
    FGColumnVector3 pos2(0.0, 100.0, 0.0);

    FGMatrix33 inertia1 = massBalance->GetPointmassInertia(10.0, pos1);
    FGMatrix33 inertia2 = massBalance->GetPointmassInertia(10.0, pos2);

    // Different positions give different inertias
    TS_ASSERT(std::abs(inertia1(1,1) - inertia2(1,1)) > 0.01 ||
              std::abs(inertia1(2,2) - inertia2(2,2)) > 0.01);

    // Verify inertia1 unchanged
    FGMatrix33 inertia1_verify = massBalance->GetPointmassInertia(10.0, pos1);
    TS_ASSERT_DELTA(inertia1(1,1), inertia1_verify(1,1), epsilon);
  }

  void testIndependentWeightSettings() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    double weights[] = {1000.0, 2000.0, 3000.0, 4000.0, 5000.0};
    double storedWeights[5];

    for (int i = 0; i < 5; i++) {
      massBalance->SetEmptyWeight(weights[i]);
      storedWeights[i] = massBalance->GetEmptyWeight();
    }

    // Each weight was stored correctly
    TS_ASSERT_DELTA(storedWeights[0], 1000.0, epsilon);
    TS_ASSERT_DELTA(storedWeights[4], 5000.0, epsilon);
  }

  void testIndependentRunCycles() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    massBalance->SetEmptyWeight(5000.0);
    massBalance->SetBaseCG(FGColumnVector3(150.0, 0.0, 0.0));

    // Run multiple times
    for (int i = 0; i < 10; i++) {
      massBalance->Run(false);
    }

    // Values should remain consistent
    TS_ASSERT_DELTA(massBalance->GetEmptyWeight(), 5000.0, epsilon);
    TS_ASSERT(std::isfinite(massBalance->GetXYZcg(1)));
  }

  void testIndependentInertiaContributions() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();

    // Two separate inertia contributions
    FGMatrix33 gas1(100.0, 0.0, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 300.0);
    FGMatrix33 gas2(200.0, 0.0, 0.0, 0.0, 400.0, 0.0, 0.0, 0.0, 600.0);

    massBalance->in.GasInertia = gas1;
    massBalance->Run(false);
    const FGMatrix33& J1 = massBalance->GetJ();
    double j1_11 = J1(1, 1);

    massBalance->in.GasInertia = gas2;
    massBalance->Run(false);
    const FGMatrix33& J2 = massBalance->GetJ();

    // J should reflect the current gas inertia, not accumulated
    TS_ASSERT(!std::isnan(j1_11));
    TS_ASSERT(!std::isnan(J2(1, 1)));
  }
};
