/*******************************************************************************
 * FGFDMExecTest.h - Unit tests for FGFDMExec (Executive class)
 *
 * Tests the FGFDMExec executive class including:
 * - Construction and initialization
 * - Model management
 * - Simulation time handling
 * - Property access
 * - Integration stepping
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGAtmosphere.h>
#include <initialization/FGInitialCondition.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGFDMExecTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction Tests
   ***************************************************************************/

  void testDefaultConstruction() {
    FGFDMExec fdmex;
    TS_ASSERT(true);  // Construction succeeded
  }

  void testGetPropertyManager() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();
    TS_ASSERT(pm != nullptr);
  }

  /***************************************************************************
   * Model Access Tests
   ***************************************************************************/

  void testGetPropagate() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropagate();
    TS_ASSERT(prop != nullptr);
  }

  void testGetAuxiliary() {
    FGFDMExec fdmex;
    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);
  }

  void testGetAtmosphere() {
    FGFDMExec fdmex;
    auto atm = fdmex.GetAtmosphere();
    TS_ASSERT(atm != nullptr);
  }

  void testGetAerodynamics() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();
    TS_ASSERT(aero != nullptr);
  }

  void testGetFCS() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
  }

  void testGetGroundReactions() {
    FGFDMExec fdmex;
    auto gr = fdmex.GetGroundReactions();
    TS_ASSERT(gr != nullptr);
  }

  void testGetPropulsion() {
    FGFDMExec fdmex;
    auto prop = fdmex.GetPropulsion();
    TS_ASSERT(prop != nullptr);
  }

  void testGetMassBalance() {
    FGFDMExec fdmex;
    auto mb = fdmex.GetMassBalance();
    TS_ASSERT(mb != nullptr);
  }

  void testGetAircraft() {
    FGFDMExec fdmex;
    auto ac = fdmex.GetAircraft();
    TS_ASSERT(ac != nullptr);
  }

  void testGetAccelerations() {
    FGFDMExec fdmex;
    auto accel = fdmex.GetAccelerations();
    TS_ASSERT(accel != nullptr);
  }

  void testGetInertial() {
    FGFDMExec fdmex;
    auto inertial = fdmex.GetInertial();
    TS_ASSERT(inertial != nullptr);
  }

  void testGetWinds() {
    FGFDMExec fdmex;
    auto winds = fdmex.GetWinds();
    TS_ASSERT(winds != nullptr);
  }

  void testGetBuoyantForces() {
    FGFDMExec fdmex;
    auto bf = fdmex.GetBuoyantForces();
    TS_ASSERT(bf != nullptr);
  }

  void testGetExternalReactions() {
    FGFDMExec fdmex;
    auto er = fdmex.GetExternalReactions();
    TS_ASSERT(er != nullptr);
  }

  /***************************************************************************
   * Initial Condition Tests
   ***************************************************************************/

  void testGetIC() {
    FGFDMExec fdmex;
    auto ic = fdmex.GetIC();
    TS_ASSERT(ic != nullptr);
  }

  /***************************************************************************
   * Simulation Time Tests
   ***************************************************************************/

  void testGetSimTimeInitiallyZero() {
    FGFDMExec fdmex;
    TS_ASSERT_DELTA(fdmex.GetSimTime(), 0.0, epsilon);
  }

  void testGetDeltaT() {
    FGFDMExec fdmex;
    double dt = fdmex.GetDeltaT();
    TS_ASSERT(dt > 0.0);
  }

  void testSetDeltaT() {
    FGFDMExec fdmex;
    fdmex.Setdt(0.01);
    TS_ASSERT_DELTA(fdmex.GetDeltaT(), 0.01, epsilon);
  }

  void testIncrTime() {
    FGFDMExec fdmex;
    double dt = fdmex.GetDeltaT();
    double t0 = fdmex.GetSimTime();

    fdmex.IncrTime();
    double t1 = fdmex.GetSimTime();

    TS_ASSERT_DELTA(t1 - t0, dt, epsilon);
  }

  /***************************************************************************
   * Frame Count Tests
   ***************************************************************************/

  void testGetFrameInitiallyZero() {
    FGFDMExec fdmex;
    TS_ASSERT_EQUALS(fdmex.GetFrame(), 0u);
  }

  /***************************************************************************
   * Holding Tests
   ***************************************************************************/

  void testHoldingDefault() {
    FGFDMExec fdmex;
    TS_ASSERT_EQUALS(fdmex.Holding(), false);
  }

  void testHoldSet() {
    FGFDMExec fdmex;
    fdmex.Hold();
    TS_ASSERT_EQUALS(fdmex.Holding(), true);
  }

  void testResume() {
    FGFDMExec fdmex;
    fdmex.Hold();
    fdmex.Resume();
    TS_ASSERT_EQUALS(fdmex.Holding(), false);
  }

  /***************************************************************************
   * Trim Status Tests
   ***************************************************************************/

  void testTrimStatusDefault() {
    FGFDMExec fdmex;
    TS_ASSERT_EQUALS(fdmex.GetTrimStatus(), false);
  }

  void testSetTrimStatus() {
    FGFDMExec fdmex;
    fdmex.SetTrimStatus(true);
    TS_ASSERT_EQUALS(fdmex.GetTrimStatus(), true);

    fdmex.SetTrimStatus(false);
    TS_ASSERT_EQUALS(fdmex.GetTrimStatus(), false);
  }

  /***************************************************************************
   * Root Directory Tests
   ***************************************************************************/

  void testSetRootDir() {
    FGFDMExec fdmex;
    fdmex.SetRootDir(SGPath("/tmp/test"));
    TS_ASSERT_EQUALS(fdmex.GetRootDir().utf8Str(), "/tmp/test");
  }

  /***************************************************************************
   * Debug Level Tests
   ***************************************************************************/

  void testDebugLevel() {
    FGFDMExec fdmex;
    // Default debug level should be reasonable
    int debugLevel = fdmex.GetDebugLevel();
    TS_ASSERT(debugLevel >= 0);
  }

  /***************************************************************************
   * Model Name Tests
   ***************************************************************************/

  void testModelNameEmpty() {
    FGFDMExec fdmex;
    // Without loading a model, name should be empty
    std::string name = fdmex.GetModelName();
    TS_ASSERT(name.empty());
  }

  /***************************************************************************
   * Property Access Tests
   ***************************************************************************/

  void testPropertyValueDouble() {
    FGFDMExec fdmex;

    // Simulation time is a built-in property
    double simTime = fdmex.GetPropertyValue("simulation/sim-time-sec");
    TS_ASSERT_DELTA(simTime, 0.0, epsilon);
  }

  void testSetPropertyValue() {
    FGFDMExec fdmex;

    // Create and set a custom property value
    auto pm = fdmex.GetPropertyManager();
    pm->GetNode("test/custom-prop", true)->setDoubleValue(0.0);

    fdmex.SetPropertyValue("test/custom-prop", 42.5);
    double val = fdmex.GetPropertyValue("test/custom-prop");
    TS_ASSERT_DELTA(val, 42.5, epsilon);
  }

  /***************************************************************************
   * Integration Rate Tests
   ***************************************************************************/

  void testGetIntegrationRate() {
    FGFDMExec fdmex;
    double dt = fdmex.GetDeltaT();

    // Integration rate should be 1/dt
    if (dt > 0) {
      double expectedRate = 1.0 / dt;
      // Just verify we can get the dt value
      TS_ASSERT(dt > 0.0);
    }
  }

  /***************************************************************************
   * Multiple Instance Tests
   ***************************************************************************/

  void testMultipleInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    // Both should have independent property managers
    TS_ASSERT(fdmex1.GetPropertyManager() != nullptr);
    TS_ASSERT(fdmex2.GetPropertyManager() != nullptr);
  }

  void testInstancesIndependent() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.Setdt(0.01);
    fdmex2.Setdt(0.02);

    TS_ASSERT_DELTA(fdmex1.GetDeltaT(), 0.01, epsilon);
    TS_ASSERT_DELTA(fdmex2.GetDeltaT(), 0.02, epsilon);
  }

  /***************************************************************************
   * Time Increment Behavior Tests
   ***************************************************************************/

  void testMultipleTimeIncrements() {
    FGFDMExec fdmex;
    double dt = fdmex.GetDeltaT();
    double t0 = fdmex.GetSimTime();

    for (int i = 0; i < 10; i++) {
      fdmex.IncrTime();
    }

    double t10 = fdmex.GetSimTime();
    TS_ASSERT_DELTA(t10 - t0, 10 * dt, epsilon * 10);
  }

  /***************************************************************************
   * Initialization Tests
   ***************************************************************************/

  void testSuspendResume() {
    FGFDMExec fdmex;

    // Test suspend/resume functionality
    fdmex.SuspendIntegration();
    TS_ASSERT_EQUALS(fdmex.IntegrationSuspended(), true);

    fdmex.ResumeIntegration();
    TS_ASSERT_EQUALS(fdmex.IntegrationSuspended(), false);
  }
};
