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
#include <memory>

#include <FGFDMExec.h>
#include <models/FGExternalReactions.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <initialization/FGInitialCondition.h>
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

  /***************************************************************************
   * Extended Indexed Access Tests
   ***************************************************************************/

  void testForcesIndexValidRange() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Test valid indices 1-3
    for (int i = 1; i <= 3; i++) {
      double f = extReact->GetForces(i);
      TS_ASSERT(!std::isnan(f));
      TS_ASSERT(std::isfinite(f));
    }
  }

  void testMomentsIndexValidRange() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Test valid indices 1-3
    for (int i = 1; i <= 3; i++) {
      double m = extReact->GetMoments(i);
      TS_ASSERT(!std::isnan(m));
      TS_ASSERT(std::isfinite(m));
    }
  }

  void testForcesAllIndicesSequentially() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 1; i <= 3; i++) {
      double f = extReact->GetForces(i);
      TS_ASSERT_DELTA(f, 0.0, epsilon);
    }
  }

  void testMomentsAllIndicesSequentially() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 1; i <= 3; i++) {
      double m = extReact->GetMoments(i);
      TS_ASSERT_DELTA(m, 0.0, epsilon);
    }
  }

  void testForcesIndexRepeatedAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Access same index multiple times
    for (int j = 0; j < 100; j++) {
      double fx = extReact->GetForces(1);
      TS_ASSERT_DELTA(fx, 0.0, epsilon);
    }
  }

  void testMomentsIndexRepeatedAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int j = 0; j < 100; j++) {
      double mz = extReact->GetMoments(3);
      TS_ASSERT_DELTA(mz, 0.0, epsilon);
    }
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testStressThousandRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 1000; i++) {
      extReact->Run(false);
    }

    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT_DELTA(forces.Magnitude(), 0.0, epsilon);
  }

  void testStressAlternatingHoldThousandRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 1000; i++) {
      extReact->Run(i % 2 == 0);
    }

    const FGColumnVector3& moments = extReact->GetMoments();
    TS_ASSERT_DELTA(moments.Magnitude(), 0.0, epsilon);
  }

  void testStressRepeatedInitialization() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 500; i++) {
      bool result = extReact->InitModel();
      TS_ASSERT_EQUALS(result, true);
    }
  }

  void testStressInitRunCycle() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 500; i++) {
      extReact->InitModel();
      extReact->Run(false);
    }

    TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(extReact->GetMoments(1), 0.0, epsilon);
  }

  void testStressManyFDMExecInstances() {
    for (int i = 0; i < 50; i++) {
      FGFDMExec fdmex;
      auto extReact = fdmex.GetExternalReactions();
      extReact->Run(false);

      TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
    }
  }

  /***************************************************************************
   * More Multi-FDMExec Tests
   ***************************************************************************/

  void testFiveFDMExecInstances() {
    FGFDMExec fdmex[5];
    std::shared_ptr<FGExternalReactions> extReact[5];

    for (int i = 0; i < 5; i++) {
      extReact[i] = fdmex[i].GetExternalReactions();
      TS_ASSERT(extReact[i] != nullptr);
    }

    // All should be different instances
    for (int i = 0; i < 5; i++) {
      for (int j = i + 1; j < 5; j++) {
        TS_ASSERT(extReact[i].get() != extReact[j].get());
      }
    }
  }

  void testParallelFDMExecOperations() {
    FGFDMExec fdmex1, fdmex2, fdmex3;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();
    auto extReact3 = fdmex3.GetExternalReactions();

    // Run all in sequence
    for (int i = 0; i < 50; i++) {
      extReact1->Run(false);
      extReact2->Run(false);
      extReact3->Run(false);
    }

    // All should have zero forces
    TS_ASSERT_DELTA(extReact1->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(extReact2->GetForces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(extReact3->GetForces(3), 0.0, epsilon);
  }

  void testFDMExecInstanceIndependence() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto extReact1 = fdmex1.GetExternalReactions();
    auto extReact2 = fdmex2.GetExternalReactions();

    // Run first one many times
    for (int i = 0; i < 100; i++) {
      extReact1->Run(false);
    }

    // Second one should still be in initial state
    const FGColumnVector3& forces2 = extReact2->GetForces();
    TS_ASSERT_DELTA(forces2(1), 0.0, epsilon);
    TS_ASSERT_DELTA(forces2(2), 0.0, epsilon);
    TS_ASSERT_DELTA(forces2(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Vector Property Tests
   ***************************************************************************/

  void testForcesVectorMagnitudeNonNegative() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      extReact->Run(false);
      double mag = extReact->GetForces().Magnitude();
      TS_ASSERT(mag >= 0.0);
    }
  }

  void testMomentsVectorMagnitudeNonNegative() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      extReact->Run(false);
      double mag = extReact->GetMoments().Magnitude();
      TS_ASSERT(mag >= 0.0);
    }
  }

  void testForcesComponentsFinite() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(std::isfinite(extReact->GetForces(i)));
    }
  }

  void testMomentsComponentsFinite() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(std::isfinite(extReact->GetMoments(i)));
    }
  }

  /***************************************************************************
   * State Transition Tests
   ***************************************************************************/

  void testTransitionHoldToRun() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(true);   // Holding
    bool result = extReact->Run(false);  // Not holding

    TS_ASSERT_EQUALS(result, true);
  }

  void testTransitionRunToHold() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);  // Not holding
    bool result = extReact->Run(true);   // Holding

    TS_ASSERT_EQUALS(result, false);
  }

  void testMultipleHoldTransitions() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 100; i++) {
      bool holding = (i % 3) == 0;
      bool result = extReact->Run(holding);
      TS_ASSERT_EQUALS(result, !holding);
    }
  }

  void testStateAfterManyTransitions() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 1000; i++) {
      extReact->Run(i % 2 == 0);
    }

    // State should still be valid
    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT(!std::isnan(forces(1)));
    TS_ASSERT(!std::isnan(forces(2)));
    TS_ASSERT(!std::isnan(forces(3)));
  }

  /***************************************************************************
   * Initialization State Tests
   ***************************************************************************/

  void testInitReturnsTrue() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    TS_ASSERT_EQUALS(extReact->InitModel(), true);
  }

  void testInitIdempotent() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    bool result1 = extReact->InitModel();
    bool result2 = extReact->InitModel();

    TS_ASSERT_EQUALS(result1, result2);
  }

  void testForcesZeroAfterMultipleInit() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      extReact->InitModel();
      TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(3), 0.0, epsilon);
    }
  }

  void testMomentsZeroAfterMultipleInit() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      extReact->InitModel();
      TS_ASSERT_DELTA(extReact->GetMoments(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(3), 0.0, epsilon);
    }
  }

  /***************************************************************************
   * Additional Boundary Tests
   ***************************************************************************/

  void testVectorAccessAfterManyOperations() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 500; i++) {
      if (i % 3 == 0) extReact->InitModel();
      extReact->Run(i % 5 == 0);
      extReact->GetForces();
      extReact->GetMoments();
    }

    // Should still be stable
    TS_ASSERT(std::isfinite(extReact->GetForces(1)));
    TS_ASSERT(std::isfinite(extReact->GetMoments(1)));
  }

  void testRepeatedVectorAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int i = 0; i < 1000; i++) {
      const FGColumnVector3& f = extReact->GetForces();
      const FGColumnVector3& m = extReact->GetMoments();
      TS_ASSERT_DELTA(f.Magnitude(), 0.0, epsilon);
      TS_ASSERT_DELTA(m.Magnitude(), 0.0, epsilon);
    }
  }

  void testInterleavedVectorAndIndexedAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 100; i++) {
      extReact->Run(false);

      // Alternate between vector and indexed access
      if (i % 2 == 0) {
        const FGColumnVector3& f = extReact->GetForces();
        TS_ASSERT_DELTA(f(1), 0.0, epsilon);
      } else {
        double fx = extReact->GetForces(1);
        TS_ASSERT_DELTA(fx, 0.0, epsilon);
      }
    }
  }

  void testForcesAndMomentsAlternateAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 100; i++) {
      extReact->Run(false);

      double f = extReact->GetForces(1);
      double m = extReact->GetMoments(1);

      TS_ASSERT_DELTA(f, 0.0, epsilon);
      TS_ASSERT_DELTA(m, 0.0, epsilon);
    }
  }

  void testSequentialIndexAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    // Access all force and moment components in sequence
    for (int cycle = 0; cycle < 50; cycle++) {
      for (int i = 1; i <= 3; i++) {
        TS_ASSERT_DELTA(extReact->GetForces(i), 0.0, epsilon);
        TS_ASSERT_DELTA(extReact->GetMoments(i), 0.0, epsilon);
      }
    }
  }

  /***************************************************************************
   * Extended Multi-Instance Tests (Tests 79-83)
   ***************************************************************************/

  // Test 79: Create and destroy many instances sequentially
  void testManyInstancesSequential() {
    for (int i = 0; i < 100; i++) {
      FGFDMExec fdmex;
      auto extReact = fdmex.GetExternalReactions();
      extReact->InitModel();
      extReact->Run(false);
      TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
    }
  }

  // Test 80: Multiple instances with different run patterns
  void testMultipleInstancesDifferentPatterns() {
    FGFDMExec fdmex1, fdmex2, fdmex3, fdmex4;

    auto ext1 = fdmex1.GetExternalReactions();
    auto ext2 = fdmex2.GetExternalReactions();
    auto ext3 = fdmex3.GetExternalReactions();
    auto ext4 = fdmex4.GetExternalReactions();

    // Different patterns for each
    for (int i = 0; i < 20; i++) {
      ext1->Run(false);
      ext2->Run(true);
      ext3->Run(i % 2 == 0);
      ext4->Run(i % 3 == 0);
    }

    // All should have zero forces
    TS_ASSERT_DELTA(ext1->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(ext2->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(ext3->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(ext4->GetForces(1), 0.0, epsilon);
  }

  // Test 81: Verify instance isolation during init
  void testInstanceIsolationDuringInit() {
    FGFDMExec fdmex1, fdmex2;

    auto ext1 = fdmex1.GetExternalReactions();
    auto ext2 = fdmex2.GetExternalReactions();

    // Run one instance multiple times
    for (int i = 0; i < 50; i++) {
      ext1->Run(false);
    }

    // Init second instance
    ext2->InitModel();

    // Both should still have zero forces
    TS_ASSERT_DELTA(ext1->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(ext2->GetForces(1), 0.0, epsilon);
  }

  // Test 82: Many instances simultaneous operations
  void testManyInstancesSimultaneousOps() {
    std::vector<std::unique_ptr<FGFDMExec>> fdmexList;

    for (int i = 0; i < 10; i++) {
      fdmexList.push_back(std::make_unique<FGFDMExec>());
    }

    // Run all instances
    for (auto& fdmex : fdmexList) {
      auto ext = fdmex->GetExternalReactions();
      ext->InitModel();
      ext->Run(false);
    }

    // Verify all have valid state
    for (auto& fdmex : fdmexList) {
      auto ext = fdmex->GetExternalReactions();
      TS_ASSERT_DELTA(ext->GetForces(1), 0.0, epsilon);
      TS_ASSERT_DELTA(ext->GetMoments(1), 0.0, epsilon);
    }
  }

  // Test 83: Instance destruction order independence
  void testInstanceDestructionOrderIndependence() {
    auto fdmex1 = std::make_unique<FGFDMExec>();
    auto fdmex2 = std::make_unique<FGFDMExec>();
    auto fdmex3 = std::make_unique<FGFDMExec>();

    auto ext1 = fdmex1->GetExternalReactions();
    auto ext2 = fdmex2->GetExternalReactions();
    auto ext3 = fdmex3->GetExternalReactions();

    ext1->Run(false);
    ext2->Run(false);
    ext3->Run(false);

    // Destroy in different order
    fdmex2.reset();
    TS_ASSERT_DELTA(ext1->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(ext3->GetForces(1), 0.0, epsilon);

    fdmex1.reset();
    TS_ASSERT_DELTA(ext3->GetForces(1), 0.0, epsilon);
  }

  /***************************************************************************
   * Extended State Consistency Tests (Tests 84-88)
   ***************************************************************************/

  // Test 84: Forces consistency through init-run cycles
  void testForcesConsistencyThroughCycles() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int cycle = 0; cycle < 20; cycle++) {
      extReact->InitModel();

      double beforeRun = extReact->GetForces(1);

      for (int i = 0; i < 10; i++) {
        extReact->Run(false);
      }

      double afterRun = extReact->GetForces(1);

      TS_ASSERT_DELTA(beforeRun, afterRun, epsilon);
      TS_ASSERT_DELTA(beforeRun, 0.0, epsilon);
    }
  }

  // Test 85: Moments consistency through init-run cycles
  void testMomentsConsistencyThroughCycles() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int cycle = 0; cycle < 20; cycle++) {
      extReact->InitModel();

      for (int i = 0; i < 10; i++) {
        extReact->Run(false);
      }

      for (int j = 1; j <= 3; j++) {
        TS_ASSERT_DELTA(extReact->GetMoments(j), 0.0, epsilon);
      }
    }
  }

  // Test 86: Verify run return values pattern
  void testRunReturnValuesPattern() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Run(false) returns true (no external forces defined)
    // Run(true) returns false (holding)
    for (int i = 0; i < 50; i++) {
      bool holdResult = extReact->Run(true);
      bool runResult = extReact->Run(false);

      TS_ASSERT_EQUALS(holdResult, false);
      TS_ASSERT_EQUALS(runResult, true);
    }
  }

  // Test 87: State invariants through operations
  void testStateInvariantsThroughOperations() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Mix of operations
    for (int i = 0; i < 100; i++) {
      switch (i % 4) {
        case 0: extReact->InitModel(); break;
        case 1: extReact->Run(false); break;
        case 2: extReact->Run(true); break;
        case 3:
          extReact->GetForces();
          extReact->GetMoments();
          break;
      }

      // Invariants should hold
      TS_ASSERT(std::isfinite(extReact->GetForces(1)));
      TS_ASSERT(std::isfinite(extReact->GetMoments(1)));
    }
  }

  // Test 88: All three components zero consistently
  void testAllComponentsZeroConsistently() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int iter = 0; iter < 50; iter++) {
      extReact->Run(false);

      // All force components zero
      TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(3), 0.0, epsilon);

      // All moment components zero
      TS_ASSERT_DELTA(extReact->GetMoments(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(3), 0.0, epsilon);
    }
  }

  /***************************************************************************
   * Vector Access Pattern Tests (Tests 89-93)
   ***************************************************************************/

  // Test 89: Rapid alternation between vector and indexed
  void testRapidAlternationVectorIndexed() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int i = 0; i < 200; i++) {
      if (i % 2 == 0) {
        const FGColumnVector3& f = extReact->GetForces();
        TS_ASSERT_DELTA(f.Magnitude(), 0.0, epsilon);
      } else {
        double fx = extReact->GetForces(1);
        double fy = extReact->GetForces(2);
        double fz = extReact->GetForces(3);
        TS_ASSERT_DELTA(fx, 0.0, epsilon);
        TS_ASSERT_DELTA(fy, 0.0, epsilon);
        TS_ASSERT_DELTA(fz, 0.0, epsilon);
      }
    }
  }

  // Test 90: Moments rapid alternation
  void testMomentsRapidAlternation() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int i = 0; i < 200; i++) {
      if (i % 2 == 0) {
        const FGColumnVector3& m = extReact->GetMoments();
        TS_ASSERT_DELTA(m.Magnitude(), 0.0, epsilon);
      } else {
        for (int j = 1; j <= 3; j++) {
          TS_ASSERT_DELTA(extReact->GetMoments(j), 0.0, epsilon);
        }
      }
    }
  }

  // Test 91: Cross-access forces and moments
  void testCrossAccessForcesMoments() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int i = 0; i < 100; i++) {
      // Access forces vector then moments indexed
      const FGColumnVector3& f = extReact->GetForces();
      double m1 = extReact->GetMoments(1);
      double m2 = extReact->GetMoments(2);

      // Access moments vector then forces indexed
      const FGColumnVector3& m = extReact->GetMoments();
      double f1 = extReact->GetForces(1);
      double f2 = extReact->GetForces(2);

      TS_ASSERT_DELTA(f.Magnitude(), 0.0, epsilon);
      TS_ASSERT_DELTA(m.Magnitude(), 0.0, epsilon);
      TS_ASSERT_DELTA(m1, 0.0, epsilon);
      TS_ASSERT_DELTA(f1, 0.0, epsilon);
    }
  }

  // Test 92: All indices in reverse order
  void testAllIndicesReverseOrder() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    for (int cycle = 0; cycle < 50; cycle++) {
      // Access in reverse order: 3, 2, 1
      for (int i = 3; i >= 1; i--) {
        TS_ASSERT_DELTA(extReact->GetForces(i), 0.0, epsilon);
        TS_ASSERT_DELTA(extReact->GetMoments(i), 0.0, epsilon);
      }
    }
  }

  // Test 93: Specific index repeated access
  void testSpecificIndexRepeatedAccess() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->Run(false);

    // Each index accessed many times
    for (int idx = 1; idx <= 3; idx++) {
      for (int j = 0; j < 100; j++) {
        TS_ASSERT_DELTA(extReact->GetForces(idx), 0.0, epsilon);
        TS_ASSERT_DELTA(extReact->GetMoments(idx), 0.0, epsilon);
      }
    }
  }

  /***************************************************************************
   * Complete Verification Tests (Tests 94-100)
   ***************************************************************************/

  // Test 94: Full API coverage test
  void testFullAPICoverage() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // 1. Construction verified
    TS_ASSERT(extReact != nullptr);

    // 2. InitModel
    TS_ASSERT_EQUALS(extReact->InitModel(), true);

    // 3. Run with both hold states
    TS_ASSERT_EQUALS(extReact->Run(true), false);
    TS_ASSERT_EQUALS(extReact->Run(false), true);

    // 4. Forces vector access
    const FGColumnVector3& forces = extReact->GetForces();
    TS_ASSERT(!std::isnan(forces(1)));
    TS_ASSERT(!std::isnan(forces(2)));
    TS_ASSERT(!std::isnan(forces(3)));

    // 5. Forces indexed access
    TS_ASSERT_DELTA(extReact->GetForces(1), forces(1), epsilon);
    TS_ASSERT_DELTA(extReact->GetForces(2), forces(2), epsilon);
    TS_ASSERT_DELTA(extReact->GetForces(3), forces(3), epsilon);

    // 6. Moments vector access
    const FGColumnVector3& moments = extReact->GetMoments();
    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));

    // 7. Moments indexed access
    TS_ASSERT_DELTA(extReact->GetMoments(1), moments(1), epsilon);
    TS_ASSERT_DELTA(extReact->GetMoments(2), moments(2), epsilon);
    TS_ASSERT_DELTA(extReact->GetMoments(3), moments(3), epsilon);

    // 8. GetName
    std::string name = extReact->GetName();
    // Just verify call succeeds
  }

  // Test 95: Stress test complete workflow
  void testStressCompleteWorkflow() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int cycle = 0; cycle < 100; cycle++) {
      // Init
      extReact->InitModel();

      // Multiple runs
      for (int i = 0; i < 10; i++) {
        extReact->Run(i % 3 == 0);
      }

      // Access all data
      const FGColumnVector3& f = extReact->GetForces();
      const FGColumnVector3& m = extReact->GetMoments();

      for (int j = 1; j <= 3; j++) {
        extReact->GetForces(j);
        extReact->GetMoments(j);
      }

      TS_ASSERT_DELTA(f.Magnitude(), 0.0, epsilon);
      TS_ASSERT_DELTA(m.Magnitude(), 0.0, epsilon);
    }
  }

  // Test 96: Multi-instance stress test
  void testMultiInstanceStress() {
    for (int outer = 0; outer < 20; outer++) {
      FGFDMExec fdmex1, fdmex2, fdmex3;

      auto ext1 = fdmex1.GetExternalReactions();
      auto ext2 = fdmex2.GetExternalReactions();
      auto ext3 = fdmex3.GetExternalReactions();

      for (int i = 0; i < 20; i++) {
        ext1->Run(false);
        ext2->Run(true);
        ext3->Run(i % 2 == 0);
      }

      TS_ASSERT_DELTA(ext1->GetForces(1), 0.0, epsilon);
      TS_ASSERT_DELTA(ext2->GetMoments(2), 0.0, epsilon);
      TS_ASSERT_DELTA(ext3->GetForces(3), 0.0, epsilon);
    }
  }

  // Test 97: Long-running simulation pattern
  void testLongRunningSimulationPattern() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->InitModel();

    // Simulate many timesteps
    for (int timestep = 0; timestep < 2000; timestep++) {
      bool holding = (timestep % 100 == 0);  // Hold every 100 steps
      extReact->Run(holding);

      // Periodic checks
      if (timestep % 200 == 0) {
        TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
        TS_ASSERT_DELTA(extReact->GetMoments(1), 0.0, epsilon);
      }
    }

    // Final verification
    TS_ASSERT_DELTA(extReact->GetForces().Magnitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(extReact->GetMoments().Magnitude(), 0.0, epsilon);
  }

  // Test 98: Verify vector references stable through operations
  void testVectorReferencesStable() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Get reference before operations
    const FGColumnVector3& forcesRef = extReact->GetForces();
    const FGColumnVector3& momentsRef = extReact->GetMoments();

    // Perform many operations
    for (int i = 0; i < 100; i++) {
      extReact->InitModel();
      extReact->Run(i % 2 == 0);
    }

    // References should still be to same objects
    const FGColumnVector3& forcesRef2 = extReact->GetForces();
    const FGColumnVector3& momentsRef2 = extReact->GetMoments();

    TS_ASSERT_EQUALS(&forcesRef, &forcesRef2);
    TS_ASSERT_EQUALS(&momentsRef, &momentsRef2);
  }

  // Test 99: All zero invariant verification
  void testAllZeroInvariantVerification() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    // Without external forces defined, everything should be zero always
    for (int phase = 0; phase < 5; phase++) {
      // Different operation patterns per phase
      switch (phase) {
        case 0:
          extReact->InitModel();
          break;
        case 1:
          for (int i = 0; i < 100; i++) extReact->Run(false);
          break;
        case 2:
          for (int i = 0; i < 100; i++) extReact->Run(true);
          break;
        case 3:
          for (int i = 0; i < 100; i++) {
            extReact->InitModel();
            extReact->Run(false);
          }
          break;
        case 4:
          for (int i = 0; i < 100; i++) extReact->Run(i % 3 != 0);
          break;
      }

      // Verify all zeros after each phase
      TS_ASSERT_DELTA(extReact->GetForces(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetForces(3), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(1), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(2), 0.0, epsilon);
      TS_ASSERT_DELTA(extReact->GetMoments(3), 0.0, epsilon);
    }
  }

  // Test 100: Complete external reactions system verification
  void testCompleteExternalReactionsSystemVerification() {
    // Multi-FDMExec comprehensive test
    FGFDMExec fdmex1, fdmex2;

    auto ext1 = fdmex1.GetExternalReactions();
    auto ext2 = fdmex2.GetExternalReactions();

    // 1. Both should be valid and different
    TS_ASSERT(ext1 != nullptr);
    TS_ASSERT(ext2 != nullptr);
    TS_ASSERT(ext1 != ext2);

    // 2. InitModel returns true for both
    TS_ASSERT_EQUALS(ext1->InitModel(), true);
    TS_ASSERT_EQUALS(ext2->InitModel(), true);

    // 3. Run operations
    for (int i = 0; i < 50; i++) {
      bool hold1 = (i % 3 == 0);
      bool hold2 = (i % 5 == 0);

      bool result1 = ext1->Run(hold1);
      bool result2 = ext2->Run(hold2);

      TS_ASSERT_EQUALS(result1, !hold1);
      TS_ASSERT_EQUALS(result2, !hold2);
    }

    // 4. Forces verification
    const FGColumnVector3& f1 = ext1->GetForces();
    const FGColumnVector3& f2 = ext2->GetForces();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(!std::isnan(f1(i)));
      TS_ASSERT(!std::isnan(f2(i)));
      TS_ASSERT(std::isfinite(f1(i)));
      TS_ASSERT(std::isfinite(f2(i)));
      TS_ASSERT_DELTA(f1(i), 0.0, epsilon);
      TS_ASSERT_DELTA(f2(i), 0.0, epsilon);
      TS_ASSERT_DELTA(ext1->GetForces(i), f1(i), epsilon);
      TS_ASSERT_DELTA(ext2->GetForces(i), f2(i), epsilon);
    }

    // 5. Moments verification
    const FGColumnVector3& m1 = ext1->GetMoments();
    const FGColumnVector3& m2 = ext2->GetMoments();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(!std::isnan(m1(i)));
      TS_ASSERT(!std::isnan(m2(i)));
      TS_ASSERT(std::isfinite(m1(i)));
      TS_ASSERT(std::isfinite(m2(i)));
      TS_ASSERT_DELTA(m1(i), 0.0, epsilon);
      TS_ASSERT_DELTA(m2(i), 0.0, epsilon);
      TS_ASSERT_DELTA(ext1->GetMoments(i), m1(i), epsilon);
      TS_ASSERT_DELTA(ext2->GetMoments(i), m2(i), epsilon);
    }

    // 6. Vector magnitude checks
    TS_ASSERT_DELTA(f1.Magnitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(f2.Magnitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(m1.Magnitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(m2.Magnitude(), 0.0, epsilon);

    // 7. Reference stability
    TS_ASSERT_EQUALS(&ext1->GetForces(), &f1);
    TS_ASSERT_EQUALS(&ext1->GetMoments(), &m1);
    TS_ASSERT_EQUALS(&ext2->GetForces(), &f2);
    TS_ASSERT_EQUALS(&ext2->GetMoments(), &m2);

    // 8. Forces and moments are different vectors
    TS_ASSERT(&f1 != &m1);
    TS_ASSERT(&f2 != &m2);
  }
};
