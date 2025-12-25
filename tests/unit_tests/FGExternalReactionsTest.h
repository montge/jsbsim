/*******************************************************************************
 * FGExternalReactionsTest.h - Unit tests for External Reactions
 *
 * Tests the FGExternalReactions model including:
 * - Force and moment vector access
 * - Model initialization and run
 * - External force management
 * - Indexed access to forces and moments
 * - Multiple FDMExec instance independence
 * - State consistency
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGExternalReactions.h>
#include <math/FGColumnVector3.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGExternalReactionsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction and Initialization Tests
   ***************************************************************************/

  void testConstruction() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    TS_ASSERT(extReact != nullptr);
  }

  void testConstructionMultipleTimes() {
    for (int i = 0; i < 5; i++) {
      FGFDMExec fdmex;
      auto extReact = fdmex.GetExternalReactions();
      TS_ASSERT(extReact != nullptr);
    }
  }

  void testInitModel() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    bool result = extReact->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  void testInitModelMultipleTimes() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Multiple init calls should not crash
    for (int i = 0; i < 5; i++) {
      bool result = extReact->InitModel();
      TS_ASSERT_EQUALS(result, true);
    }
  }

  void testInitModelResetsState() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);
    extReact->InitModel();

    // After init, forces should still be zero
    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Force Vector Tests
   ***************************************************************************/

  void testGetForcesVector() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& forces = extReact->GetForces();

    TS_ASSERT(!std::isnan(forces(1)));
    TS_ASSERT(!std::isnan(forces(2)));
    TS_ASSERT(!std::isnan(forces(3)));
  }

  void testGetForcesInitiallyZero() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Without external forces defined, should be zero
    const FGColumnVector3& forces = extReact->GetForces();

    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(3), 0.0, epsilon);
  }

  void testGetForcesIndexed() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Test indexed access
    double fx = extReact->GetForces(1);
    double fy = extReact->GetForces(2);
    double fz = extReact->GetForces(3);

    TS_ASSERT_DELTA(fx, 0.0, epsilon);
    TS_ASSERT_DELTA(fy, 0.0, epsilon);
    TS_ASSERT_DELTA(fz, 0.0, epsilon);
  }

  void testGetForcesIndexedMatchesVector() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& forces = extReact->GetForces();

    TS_ASSERT_EQUALS(extReact->GetForces(1), forces(1));
    TS_ASSERT_EQUALS(extReact->GetForces(2), forces(2));
    TS_ASSERT_EQUALS(extReact->GetForces(3), forces(3));
  }

  void testGetForcesNotNaN() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(!std::isnan(extReact->GetForces(i)));
      TS_ASSERT(!std::isinf(extReact->GetForces(i)));
    }
  }

  void testGetForcesFinite() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& forces = extReact->GetForces();

    TS_ASSERT(std::isfinite(forces(1)));
    TS_ASSERT(std::isfinite(forces(2)));
    TS_ASSERT(std::isfinite(forces(3)));
  }

  /***************************************************************************
   * Moment Vector Tests
   ***************************************************************************/

  void testGetMomentsVector() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& moments = extReact->GetMoments();

    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  void testGetMomentsInitiallyZero() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& moments = extReact->GetMoments();

    TS_ASSERT_DELTA(moments(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moments(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moments(3), 0.0, epsilon);
  }

  void testGetMomentsIndexed() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Test indexed access
    double mx = extReact->GetMoments(1);
    double my = extReact->GetMoments(2);
    double mz = extReact->GetMoments(3);

    TS_ASSERT_DELTA(mx, 0.0, epsilon);
    TS_ASSERT_DELTA(my, 0.0, epsilon);
    TS_ASSERT_DELTA(mz, 0.0, epsilon);
  }

  void testGetMomentsIndexedMatchesVector() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& moments = extReact->GetMoments();

    TS_ASSERT_EQUALS(extReact->GetMoments(1), moments(1));
    TS_ASSERT_EQUALS(extReact->GetMoments(2), moments(2));
    TS_ASSERT_EQUALS(extReact->GetMoments(3), moments(3));
  }

  void testGetMomentsNotNaN() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(!std::isnan(extReact->GetMoments(i)));
      TS_ASSERT(!std::isinf(extReact->GetMoments(i)));
    }
  }

  void testGetMomentsFinite() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& moments = extReact->GetMoments();

    TS_ASSERT(std::isfinite(moments(1)));
    TS_ASSERT(std::isfinite(moments(2)));
    TS_ASSERT(std::isfinite(moments(3)));
  }

  /***************************************************************************
   * Run Model Tests
   ***************************************************************************/

  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Run returns true when no external forces are defined (early return)
    bool result = extReact->Run(false);
    TS_ASSERT_EQUALS(result, true);
  }

  void testRunHolding() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // When holding, Run returns false
    bool result = extReact->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  void testRunAlternatingHold() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      bool result = extReact->Run(i % 2 == 0);
      // When holding (odd i), returns false; otherwise true
      TS_ASSERT_EQUALS(result, i % 2 != 0);
    }
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  void testMultipleRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      bool result = extReact->Run(false);
      TS_ASSERT_EQUALS(result, true);  // Returns true when no forces defined
    }
  }

  void testManyRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 100; i++) {
      extReact->Run(false);
    }

    // Forces should still be zero after many runs
    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(forces(3), 0.0, epsilon);
  }

  void testRunAfterInit() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->InitModel();
    bool result = extReact->Run(false);
    TS_ASSERT_EQUALS(result, true);  // Returns true when no forces defined
  }

  void testMultipleInitAndRun() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 5; i++) {
      extReact->InitModel();
      bool result = extReact->Run(false);
      TS_ASSERT_EQUALS(result, true);
    }
  }

  /***************************************************************************
   * Force Consistency Tests
   ***************************************************************************/

  void testForcesConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Without any external forces, values should remain zero
    extReact->Run(false);
    const FGColumnVector3& forces1 = extReact->GetForces();

    extReact->Run(false);
    const FGColumnVector3& forces2 = extReact->GetForces();

    TS_ASSERT_DELTA(forces1(1), forces2(1), epsilon);
    TS_ASSERT_DELTA(forces1(2), forces2(2), epsilon);
    TS_ASSERT_DELTA(forces1(3), forces2(3), epsilon);
  }

  void testMomentsConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);
    const FGColumnVector3& moments1 = extReact->GetMoments();

    extReact->Run(false);
    const FGColumnVector3& moments2 = extReact->GetMoments();

    TS_ASSERT_DELTA(moments1(1), moments2(1), epsilon);
    TS_ASSERT_DELTA(moments1(2), moments2(2), epsilon);
    TS_ASSERT_DELTA(moments1(3), moments2(3), epsilon);
  }

  void testForcesZeroWithoutExternalForces() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Run multiple times and check forces remain zero
    for (int i = 0; i < 10; i++) {
      extReact->Run(false);
      TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(3), 0.0, epsilon);
    }
  }

  void testMomentsZeroWithoutExternalForces() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      extReact->Run(false);
      TS_ASSERT_DELTA(extReact->GetMoments(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(3), 0.0, epsilon);
    }
  }

  /***************************************************************************
   * Multiple FDMExec Instance Tests
   ***************************************************************************/

  void testMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();

    TS_ASSERT(extReact1 != nullptr);
    TS_ASSERT(extReact2 != nullptr);
    TS_ASSERT(extReact1 != extReact2);
  }

  void testIndependentFDMExecForces() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();

    // Running one should not affect the other
    extReact1->Run(false);

    const FGColumnVector3& forces1 = extReact1->GetForces();
    const FGColumnVector3& forces2 = extReact2->GetForces();

    TS_ASSERT_DELTA(forces1(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces2(1), 0.0, epsilon);
  }

  void testIndependentFDMExecMoments() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();

    extReact1->Run(false);
    extReact2->Run(false);

    const FGColumnVector3& moments1 = extReact1->GetMoments();
    const FGColumnVector3& moments2 = extReact2->GetMoments();

    TS_ASSERT_DELTA(moments1(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moments2(1), 0.0, epsilon);
  }

  void testThreeFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;
    FGFDMExec fdmex3;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();
    auto extReact3 = fdmex3.GetExternalReactions();

    TS_ASSERT(extReact1 != extReact2);
    TS_ASSERT(extReact2 != extReact3);
    TS_ASSERT(extReact1 != extReact3);
  }

  /***************************************************************************
   * Vector Magnitude Tests
   ***************************************************************************/

  void testForcesMagnitudeZero() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& forces = extReact->GetForces();
    double magnitude = forces.Magnitude();

    TS_ASSERT_DELTA(magnitude, 0.0, epsilon);
  }

  void testMomentsMagnitudeZero() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& moments = extReact->GetMoments();
    double magnitude = moments.Magnitude();

    TS_ASSERT_DELTA(magnitude, 0.0, epsilon);
  }

  void testForcesMagnitudeAfterRun() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    const FGColumnVector3& forces = extReact->GetForces();
    double magnitude = forces.Magnitude();

    TS_ASSERT_DELTA(magnitude, 0.0, epsilon);
  }

  void testMomentsMagnitudeAfterRun() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    const FGColumnVector3& moments = extReact->GetMoments();
    double magnitude = moments.Magnitude();

    TS_ASSERT_DELTA(magnitude, 0.0, epsilon);
  }

  /***************************************************************************
   * State Tests
   ***************************************************************************/

  void testForcesVectorReference() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& forces1 = extReact->GetForces();
    const FGColumnVector3& forces2 = extReact->GetForces();

    // Should return reference to same vector
    TS_ASSERT_EQUALS(&forces1, &forces2);
  }

  void testMomentsVectorReference() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& moments1 = extReact->GetMoments();
    const FGColumnVector3& moments2 = extReact->GetMoments();

    // Should return reference to same vector
    TS_ASSERT_EQUALS(&moments1, &moments2);
  }

  void testForcesAndMomentsDifferentVectors() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    const FGColumnVector3& forces = extReact->GetForces();
    const FGColumnVector3& moments = extReact->GetMoments();

    // Forces and moments should be different vectors
    TS_ASSERT(&forces != &moments);
  }

  /***************************************************************************
   * Sequence Tests
   ***************************************************************************/

  void testInitRunSequence() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    TS_ASSERT_EQUALS(extReact->InitModel(), true);
    TS_ASSERT_EQUALS(extReact->Run(false), true);
  }

  void testRunInitRunSequence() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);
    extReact->InitModel();
    bool result = extReact->Run(false);

    TS_ASSERT_EQUALS(result, true);
  }

  void testComplexSequence() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Complex sequence of operations
    extReact->InitModel();
    extReact->Run(false);
    extReact->Run(true);  // Holding
    extReact->Run(false);
    extReact->InitModel();
    extReact->Run(false);

    // Should complete without error
    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);
  }

  /***************************************************************************
   * Name Tests
   ***************************************************************************/

  void testGetName() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // GetName() returns the model name - may be empty for ExternalReactions
    std::string name = extReact->GetName();
    // Just verify the call doesn't crash
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testRunWithNoInitialization() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Should work without explicit initialization
    bool result = extReact->Run(false);
    TS_ASSERT_EQUALS(result, true);
  }

  void testForcesAccessBeforeRun() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Accessing forces before any run should work
    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT_DELTA(forces.Magnitude(), 0.0, epsilon);
  }

  void testMomentsAccessBeforeRun() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Accessing moments before any run should work
    const FGColumnVector3& moments = extReact->GetMoments();
    TS_ASSERT_DELTA(moments.Magnitude(), 0.0, epsilon);
  }

  void testInterleavedFDMExecOperations() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();

    // Interleaved operations on two instances
    for (int i = 0; i < 10; i++) {
      extReact1->Run(false);
      extReact2->Run(false);
      extReact1->InitModel();
      extReact2->InitModel();
    }

    const FGColumnVector3& forces1 = extReact1->GetForces();
    const FGColumnVector3& forces2 = extReact2->GetForces();

    TS_ASSERT_DELTA(forces1(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces2(1), 0.0, epsilon);
  }
};
