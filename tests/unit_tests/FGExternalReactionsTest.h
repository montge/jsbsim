/*******************************************************************************
 * FGExternalReactionsTest.h - Unit tests for External Reactions
 *
 * Tests the FGExternalReactions model including:
 * - Force and moment vector access
 * - Model initialization and run
 * - External force management
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

  void testInitModel() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    bool result = extReact->InitModel();
    TS_ASSERT_EQUALS(result, true);
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

  /***************************************************************************
   * Run Model Tests
   ***************************************************************************/

  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    bool result = extReact->Run(false);
    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  void testRunHolding() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    bool result = extReact->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  void testMultipleRuns() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    for (int i = 0; i < 10; i++) {
      bool result = extReact->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  void testRunAfterInit() {
    FGFDMExec fdmex;
    auto extReact = fdmex.GetExternalReactions();

    extReact->InitModel();
    bool result = extReact->Run(false);
    TS_ASSERT_EQUALS(result, false);
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
};
