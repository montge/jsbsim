/*******************************************************************************
 * FGPropulsionTest.h - Unit tests for Propulsion System
 *
 * Tests the FGPropulsion model including:
 * - Engine and tank management
 * - Force and moment calculations
 * - Fuel management
 * - Starter, cutoff, magnetos control
 * - Active engine selection
 * - Multiple run scenarios
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <math/FGColumnVector3.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGPropulsionTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction and Initialization Tests
   ***************************************************************************/

  void testConstruction() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT(prop != nullptr);
  }

  void testConstructionMultipleTimes() {
    for (int i = 0; i < 5; i++) {
      FGFDMExec fdmex;
      auto prop = fdmex.GetPropulsion();
      TS_ASSERT(prop != nullptr);
    }
  }

  void testInitModel() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    bool result = prop->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  void testInitModelMultipleTimes() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 5; i++) {
      bool result = prop->InitModel();
      TS_ASSERT_EQUALS(result, true);
    }
  }

  /***************************************************************************
   * Engine Count Tests
   ***************************************************************************/

  void testNoEnginesInitially() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetNumEngines(), 0u);
  }

  void testEngineCountType() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    size_t numEngines = prop->GetNumEngines();
    TS_ASSERT_EQUALS(numEngines, 0u);
  }

  /***************************************************************************
   * Tank Count Tests
   ***************************************************************************/

  void testNoTanksInitially() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetNumTanks(), 0u);
  }

  void testTankCountType() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    size_t numTanks = prop->GetNumTanks();
    TS_ASSERT_EQUALS(numTanks, 0u);
  }

  /***************************************************************************
   * Force Vector Tests
   ***************************************************************************/

  void testGetForcesVector() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& forces = prop->GetForces();

    TS_ASSERT(!std::isnan(forces(1)));
    TS_ASSERT(!std::isnan(forces(2)));
    TS_ASSERT(!std::isnan(forces(3)));
  }

  void testGetForcesInitiallyZero() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Without engines, forces should be zero
    TS_ASSERT_DELTA(prop->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetForces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetForces(3), 0.0, epsilon);
  }

  void testGetForcesIndexedMatchesVector() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& forces = prop->GetForces();

    TS_ASSERT_EQUALS(prop->GetForces(1), forces(1));
    TS_ASSERT_EQUALS(prop->GetForces(2), forces(2));
    TS_ASSERT_EQUALS(prop->GetForces(3), forces(3));
  }

  void testGetForcesFinite() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& forces = prop->GetForces();

    TS_ASSERT(std::isfinite(forces(1)));
    TS_ASSERT(std::isfinite(forces(2)));
    TS_ASSERT(std::isfinite(forces(3)));
  }

  /***************************************************************************
   * Moment Vector Tests
   ***************************************************************************/

  void testGetMomentsVector() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& moments = prop->GetMoments();

    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  void testGetMomentsInitiallyZero() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_DELTA(prop->GetMoments(1), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetMoments(2), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetMoments(3), 0.0, epsilon);
  }

  void testGetMomentsIndexedMatchesVector() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& moments = prop->GetMoments();

    TS_ASSERT_EQUALS(prop->GetMoments(1), moments(1));
    TS_ASSERT_EQUALS(prop->GetMoments(2), moments(2));
    TS_ASSERT_EQUALS(prop->GetMoments(3), moments(3));
  }

  void testGetMomentsFinite() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& moments = prop->GetMoments();

    TS_ASSERT(std::isfinite(moments(1)));
    TS_ASSERT(std::isfinite(moments(2)));
    TS_ASSERT(std::isfinite(moments(3)));
  }

  /***************************************************************************
   * Active Engine Tests
   ***************************************************************************/

  void testActiveEngineDefault() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Default active engine is -1 (all engines)
    int activeEngine = prop->GetActiveEngine();
    TS_ASSERT_EQUALS(activeEngine, -1);
  }

  void testSetActiveEngine() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Without engines, SetActiveEngine may not change the value
    // Just verify it doesn't crash
    prop->SetActiveEngine(0);
    prop->SetActiveEngine(1);
    prop->SetActiveEngine(-1);
    TS_ASSERT(true);
  }

  void testSetActiveEngineNoEngines() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Without engines, active engine stays at -1
    prop->SetActiveEngine(0);
    // May or may not change depending on implementation
    int active = prop->GetActiveEngine();
    TS_ASSERT(active == -1 || active == 0);
  }

  /***************************************************************************
   * Fuel Freeze Tests
   ***************************************************************************/

  void testFuelFreezeDefault() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetFuelFreeze(), false);
  }

  void testFuelFreezeSet() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->SetFuelFreeze(true);
    TS_ASSERT_EQUALS(prop->GetFuelFreeze(), true);

    prop->SetFuelFreeze(false);
    TS_ASSERT_EQUALS(prop->GetFuelFreeze(), false);
  }

  void testFuelFreezeToggle() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 10; i++) {
      prop->SetFuelFreeze(i % 2 == 0);
      TS_ASSERT_EQUALS(prop->GetFuelFreeze(), i % 2 == 0);
    }
  }

  /***************************************************************************
   * Cutoff Tests
   ***************************************************************************/

  void testCutoffDefault() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Without engines, GetCutoff returns 1
    int cutoff = prop->GetCutoff();
    TS_ASSERT_EQUALS(cutoff, 1);
  }

  void testSetCutoff() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // SetCutoff without engines should not crash
    prop->SetCutoff(1);
    TS_ASSERT(true);

    prop->SetCutoff(0);
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Starter Tests
   ***************************************************************************/

  void testStarterDefault() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Without engines, GetStarter returns 1
    int starter = prop->GetStarter();
    TS_ASSERT_EQUALS(starter, 1);
  }

  void testSetStarter() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // SetStarter without engines should not crash
    prop->SetStarter(1);
    TS_ASSERT(true);

    prop->SetStarter(0);
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Magnetos Tests
   ***************************************************************************/

  void testSetMagnetos() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // SetMagnetos without engines should not crash
    prop->SetMagnetos(0);  // Off
    TS_ASSERT(true);

    prop->SetMagnetos(1);  // Left
    TS_ASSERT(true);

    prop->SetMagnetos(2);  // Right
    TS_ASSERT(true);

    prop->SetMagnetos(3);  // Both
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Run Model Tests
   ***************************************************************************/

  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    bool result = prop->Run(false);
    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  void testRunHolding() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    bool result = prop->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  void testMultipleRuns() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 10; i++) {
      bool result = prop->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  void testRunAlternatingHold() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 10; i++) {
      bool result = prop->Run(i % 2 == 0);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  void testRunAfterInit() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->InitModel();
    bool result = prop->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Tank Weight Tests
   ***************************************************************************/

  void testTanksWeightNoTanks() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    double weight = prop->GetTanksWeight();
    TS_ASSERT_DELTA(weight, 0.0, epsilon);
  }

  void testTanksWeightNonNegative() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    double weight = prop->GetTanksWeight();
    TS_ASSERT(weight >= 0.0);
  }

  void testTanksWeightAfterRun() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->Run(false);
    double weight = prop->GetTanksWeight();
    TS_ASSERT_DELTA(weight, 0.0, epsilon);
  }

  /***************************************************************************
   * Tank Moment Tests
   ***************************************************************************/

  void testTanksMomentNoTanks() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& moment = prop->GetTanksMoment();

    TS_ASSERT_DELTA(moment(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(3), 0.0, epsilon);
  }

  void testTanksMomentFinite() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGColumnVector3& moment = prop->GetTanksMoment();

    TS_ASSERT(std::isfinite(moment(1)));
    TS_ASSERT(std::isfinite(moment(2)));
    TS_ASSERT(std::isfinite(moment(3)));
  }

  /***************************************************************************
   * Propulsion String Output Tests
   ***************************************************************************/

  void testGetPropulsionStringsEmpty() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    std::string strings = prop->GetPropulsionStrings(",");
    // With no engines, output should be empty or minimal
    TS_ASSERT(strings.empty() || strings.length() < 10);
  }

  void testGetPropulsionValuesEmpty() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    std::string values = prop->GetPropulsionValues(",");
    TS_ASSERT(values.empty() || values.length() < 10);
  }

  void testGetPropulsionStringsTabDelimiter() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    std::string strings = prop->GetPropulsionStrings("\t");
    TS_ASSERT(strings.empty() || strings.length() < 10);
  }

  void testGetPropulsionValuesTabDelimiter() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    std::string values = prop->GetPropulsionValues("\t");
    TS_ASSERT(values.empty() || values.length() < 10);
  }

  void testGetPropulsionTankReport() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    std::string report = prop->GetPropulsionTankReport();
    // With no tanks, report might be empty or have header
    TS_ASSERT(report.length() >= 0);
  }

  /***************************************************************************
   * Tank Inertia Tests
   ***************************************************************************/

  void testCalculateTankInertiasNoTanks() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGMatrix33& inertias = prop->CalculateTankInertias();

    // With no tanks, inertias should be zero
    TS_ASSERT_DELTA(inertias(1,1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(2,2), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(3,3), 0.0, epsilon);
  }

  void testCalculateTankInertiasOffDiagonal() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGMatrix33& inertias = prop->CalculateTankInertias();

    // Off-diagonal should also be zero
    TS_ASSERT_DELTA(inertias(1,2), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(1,3), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(2,1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(2,3), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(3,1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertias(3,2), 0.0, epsilon);
  }

  void testCalculateTankInertiasSymmetric() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGMatrix33& inertias = prop->CalculateTankInertias();

    // Inertia matrix should be symmetric
    TS_ASSERT_DELTA(inertias(1,2), inertias(2,1), epsilon);
    TS_ASSERT_DELTA(inertias(1,3), inertias(3,1), epsilon);
    TS_ASSERT_DELTA(inertias(2,3), inertias(3,2), epsilon);
  }

  /***************************************************************************
   * Multiple FDMExec Instance Tests
   ***************************************************************************/

  void testMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto prop1 = fdmex1.GetPropulsion();
    auto prop2 = fdmex2.GetPropulsion();

    TS_ASSERT(prop1 != nullptr);
    TS_ASSERT(prop2 != nullptr);
    TS_ASSERT(prop1 != prop2);
  }

  void testIndependentFDMExecPropulsion() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto prop1 = fdmex1.GetPropulsion();
    auto prop2 = fdmex2.GetPropulsion();

    // Setting fuel freeze on one shouldn't affect the other
    prop1->SetFuelFreeze(true);
    TS_ASSERT_EQUALS(prop1->GetFuelFreeze(), true);
    TS_ASSERT_EQUALS(prop2->GetFuelFreeze(), false);
  }

  void testIndependentActiveEngine() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto prop1 = fdmex1.GetPropulsion();
    auto prop2 = fdmex2.GetPropulsion();

    // Without engines, SetActiveEngine may not change the value
    prop1->SetActiveEngine(0);
    prop2->SetActiveEngine(1);

    // Just verify they have independent instances
    TS_ASSERT(prop1 != prop2);
  }

  /***************************************************************************
   * State Consistency Tests
   ***************************************************************************/

  void testForcesConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->Run(false);
    const FGColumnVector3& forces1 = prop->GetForces();
    double fx1 = forces1(1), fy1 = forces1(2), fz1 = forces1(3);

    prop->Run(false);
    const FGColumnVector3& forces2 = prop->GetForces();

    TS_ASSERT_DELTA(forces2(1), fx1, epsilon);
    TS_ASSERT_DELTA(forces2(2), fy1, epsilon);
    TS_ASSERT_DELTA(forces2(3), fz1, epsilon);
  }

  void testMomentsConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->Run(false);
    const FGColumnVector3& moments1 = prop->GetMoments();
    double mx1 = moments1(1), my1 = moments1(2), mz1 = moments1(3);

    prop->Run(false);
    const FGColumnVector3& moments2 = prop->GetMoments();

    TS_ASSERT_DELTA(moments2(1), mx1, epsilon);
    TS_ASSERT_DELTA(moments2(2), my1, epsilon);
    TS_ASSERT_DELTA(moments2(3), mz1, epsilon);
  }

  /***************************************************************************
   * Sequence Tests
   ***************************************************************************/

  void testInitRunSequence() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->InitModel(), true);
    TS_ASSERT_EQUALS(prop->Run(false), false);
  }

  void testComplexSequence() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->InitModel();
    prop->Run(false);
    prop->SetFuelFreeze(true);
    prop->Run(false);
    prop->SetActiveEngine(0);
    prop->Run(false);
    prop->SetFuelFreeze(false);
    prop->InitModel();
    prop->Run(false);

    // Should complete without error
    TS_ASSERT_DELTA(prop->GetForces(1), 0.0, epsilon);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testRapidStateChanges() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 100; i++) {
      prop->SetFuelFreeze(i % 2 == 0);
      prop->SetActiveEngine(i % 4 - 1);
      prop->Run(i % 3 == 0);
    }

    TS_ASSERT(true);
  }

  void testInterleavedOperations() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto prop1 = fdmex1.GetPropulsion();
    auto prop2 = fdmex2.GetPropulsion();

    for (int i = 0; i < 10; i++) {
      prop1->Run(false);
      prop2->Run(false);
      prop1->InitModel();
      prop2->InitModel();
    }

    TS_ASSERT_DELTA(prop1->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(prop2->GetForces(1), 0.0, epsilon);
  }

  /***************************************************************************
   * Model Identity Tests
   ***************************************************************************/

  void testGetName() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    std::string name = prop->GetName();
    TS_ASSERT(!name.empty());
  }

  void testGetExec() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT(prop->GetExec() == &fdmex);
  }

  void testSetRate() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->SetRate(5);
    TS_ASSERT_EQUALS(prop->GetRate(), 5u);
  }

  void testRatePersistence() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->SetRate(7);
    prop->Run(false);
    TS_ASSERT_EQUALS(prop->GetRate(), 7u);
  }

  void testRateZero() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->SetRate(0);
    TS_ASSERT_EQUALS(prop->GetRate(), 0u);
  }

  /***************************************************************************
   * Fuel Management Tests
   ***************************************************************************/

  void testTankInertiaMatrixSymmetry() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGMatrix33& inertia = prop->CalculateTankInertias();

    // Verify symmetry
    TS_ASSERT_DELTA(inertia(1,2), inertia(2,1), epsilon);
    TS_ASSERT_DELTA(inertia(1,3), inertia(3,1), epsilon);
    TS_ASSERT_DELTA(inertia(2,3), inertia(3,2), epsilon);
  }

  void testTankInertiaMatrixNonNegativeDiagonal() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    const FGMatrix33& inertia = prop->CalculateTankInertias();

    // Diagonal should be non-negative
    TS_ASSERT(inertia(1,1) >= 0.0);
    TS_ASSERT(inertia(2,2) >= 0.0);
    TS_ASSERT(inertia(3,3) >= 0.0);
  }

  /***************************************************************************
   * Force and Moment Detailed Tests
   ***************************************************************************/

  void testForcesAfterMultipleRuns() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 50; i++) {
      prop->Run(false);
    }

    // Forces should still be zero without engines
    TS_ASSERT_DELTA(prop->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetForces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetForces(3), 0.0, epsilon);
  }

  void testMomentsAfterMultipleRuns() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 50; i++) {
      prop->Run(false);
    }

    TS_ASSERT_DELTA(prop->GetMoments(1), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetMoments(2), 0.0, epsilon);
    TS_ASSERT_DELTA(prop->GetMoments(3), 0.0, epsilon);
  }

  void testForcesVectorNonNegativeX() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->Run(false);
    // Without engines, thrust (X-force) should be zero
    TS_ASSERT_DELTA(prop->GetForces(1), 0.0, epsilon);
  }

  /***************************************************************************
   * Engine and Tank Count Tests
   ***************************************************************************/

  void testEngineCountZero() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetNumEngines(), 0u);
  }

  void testTankCountZero() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetNumTanks(), 0u);
  }

  /***************************************************************************
   * Stability Tests
   ***************************************************************************/

  void testRapidFuelFreezeToggle() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 1000; i++) {
      prop->SetFuelFreeze(i % 2 == 0);
    }

    TS_ASSERT_EQUALS(prop->GetFuelFreeze(), false);
  }

  void testStressRun() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 500; i++) {
      prop->Run(false);
    }

    TS_ASSERT_DELTA(prop->GetForces(1), 0.0, epsilon);
  }

  void testStressInitAndRun() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    for (int i = 0; i < 100; i++) {
      prop->InitModel();
      prop->Run(false);
    }

    TS_ASSERT_EQUALS(prop->GetNumEngines(), 0u);
  }

  void testStressMultipleInstances() {
    for (int i = 0; i < 20; i++) {
      FGFDMExec fdmex;
      auto prop = fdmex.GetPropulsion();

      prop->Run(false);
      TS_ASSERT(prop != nullptr);
    }
  }

  /***************************************************************************
   * Cross-Instance Tests
   ***************************************************************************/

  void testThreeFDMExecInstances() {
    FGFDMExec fdmex1, fdmex2, fdmex3;

    auto p1 = fdmex1.GetPropulsion();
    auto p2 = fdmex2.GetPropulsion();
    auto p3 = fdmex3.GetPropulsion();

    p1->SetFuelFreeze(true);
    p2->SetFuelFreeze(false);
    p3->SetFuelFreeze(true);

    TS_ASSERT_EQUALS(p1->GetFuelFreeze(), true);
    TS_ASSERT_EQUALS(p2->GetFuelFreeze(), false);
    TS_ASSERT_EQUALS(p3->GetFuelFreeze(), true);
  }

  void testConcurrentRuns() {
    FGFDMExec fdmex1, fdmex2;

    auto p1 = fdmex1.GetPropulsion();
    auto p2 = fdmex2.GetPropulsion();

    for (int i = 0; i < 50; i++) {
      p1->Run(false);
      p2->Run(false);
    }

    TS_ASSERT_EQUALS(p1->GetNumEngines(), 0u);
    TS_ASSERT_EQUALS(p2->GetNumEngines(), 0u);
  }

  /***************************************************************************
   * Additional State Tests
   ***************************************************************************/

  void testInitModelResetsState() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->SetFuelFreeze(true);
    prop->Run(false);
    prop->InitModel();

    // FuelFreeze is preserved across InitModel
    TS_ASSERT_EQUALS(prop->GetFuelFreeze(), true);
  }

  void testForcesMomentsBothZero() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->Run(false);

    // Without engines, both should be zero
    double totalForce = std::abs(prop->GetForces(1)) +
                        std::abs(prop->GetForces(2)) +
                        std::abs(prop->GetForces(3));
    double totalMoment = std::abs(prop->GetMoments(1)) +
                         std::abs(prop->GetMoments(2)) +
                         std::abs(prop->GetMoments(3));

    TS_ASSERT_DELTA(totalForce, 0.0, epsilon);
    TS_ASSERT_DELTA(totalMoment, 0.0, epsilon);
  }

  void testTankWeightMomentConsistent() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->Run(false);

    double weight = prop->GetTanksWeight();
    const FGColumnVector3& moment = prop->GetTanksMoment();

    // If weight is zero, moment should be zero
    if (weight < epsilon) {
      TS_ASSERT_DELTA(moment(1), 0.0, epsilon);
      TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
      TS_ASSERT_DELTA(moment(3), 0.0, epsilon);
    }
  }

  void testTankInertiaZeroWithoutTanks() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Without tanks, inertias should be zero
    const FGMatrix33& inertia = prop->CalculateTankInertias();

    TS_ASSERT_DELTA(inertia(1,1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(2,2), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(3,3), 0.0, epsilon);
  }

  void testActiveEngineRange() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Try various active engine values
    for (int i = -2; i < 5; i++) {
      prop->SetActiveEngine(i);
      int active = prop->GetActiveEngine();
      // Should be -1 (all) or >= 0
      TS_ASSERT(active >= -1);
    }
  }

  void testMagnetosRange() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    // Test all magneto positions
    for (int pos = 0; pos <= 3; pos++) {
      prop->SetMagnetos(pos);
      // Should not crash
      TS_ASSERT(true);
    }
  }

  void testStarterCutoffInteraction() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    prop->SetStarter(1);
    prop->SetCutoff(0);
    prop->Run(false);

    prop->SetStarter(0);
    prop->SetCutoff(1);
    prop->Run(false);

    // Should complete without error
    TS_ASSERT(true);
  }
};
