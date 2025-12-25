/*******************************************************************************
 * FGPropulsionTest.h - Unit tests for Propulsion System
 *
 * Tests the FGPropulsion model including:
 * - Engine and tank management
 * - Force and moment calculations
 * - Fuel management
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

  void testInitModel() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    bool result = prop->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  /***************************************************************************
   * Engine Count Tests
   ***************************************************************************/

  void testNoEnginesInitially() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetNumEngines(), 0u);
  }

  /***************************************************************************
   * Tank Count Tests
   ***************************************************************************/

  void testNoTanksInitially() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

    TS_ASSERT_EQUALS(prop->GetNumTanks(), 0u);
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

  /***************************************************************************
   * Tank Weight Tests
   ***************************************************************************/

  void testTanksWeightNoTanks() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();

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
};
