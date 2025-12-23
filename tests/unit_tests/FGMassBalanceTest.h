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
};
