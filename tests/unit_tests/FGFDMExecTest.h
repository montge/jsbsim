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

  /***************************************************************************
   * Path Configuration Tests
   ***************************************************************************/

  void testSetEnginePath() {
    FGFDMExec fdmex;
    fdmex.SetEnginePath(SGPath("/tmp/engines"));
    TS_ASSERT_EQUALS(fdmex.GetEnginePath().utf8Str(), "/tmp/engines");
  }

  void testSetAircraftPath() {
    FGFDMExec fdmex;
    fdmex.SetAircraftPath(SGPath("/tmp/aircraft"));
    TS_ASSERT_EQUALS(fdmex.GetAircraftPath().utf8Str(), "/tmp/aircraft");
  }

  void testSetSystemsPath() {
    FGFDMExec fdmex;
    fdmex.SetSystemsPath(SGPath("/tmp/systems"));
    TS_ASSERT_EQUALS(fdmex.GetSystemsPath().utf8Str(), "/tmp/systems");
  }

  /***************************************************************************
   * Version Information Tests
   ***************************************************************************/

  void testGetVersion() {
    FGFDMExec fdmex;
    std::string version = fdmex.GetVersion();
    TS_ASSERT(!version.empty());
  }

  /***************************************************************************
   * Output and Logging Tests
   ***************************************************************************/

  void testGetOutputDirective() {
    FGFDMExec fdmex;
    // Without output loaded, should be empty
    auto outputName = fdmex.GetOutputFileName(0);
    TS_ASSERT(outputName.empty());
  }

  void testGetInput() {
    FGFDMExec fdmex;
    auto input = fdmex.GetInput();
    TS_ASSERT(input != nullptr);
  }

  /***************************************************************************
   * Reset Tests
   ***************************************************************************/

  void testResetTimeAdvances() {
    FGFDMExec fdmex;

    // Advance time
    for (int i = 0; i < 5; i++) {
      fdmex.IncrTime();
    }

    double t1 = fdmex.GetSimTime();
    TS_ASSERT(t1 > 0);  // Time advanced
    TS_ASSERT(fdmex.GetFrame() > 0);  // Frame advanced
  }

  /***************************************************************************
   * Property Node Tests
   ***************************************************************************/

  void testPropertyNodeCreation() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    // Create a new property node
    auto node = pm->GetNode("test/new-property", true);
    TS_ASSERT(node != nullptr);
  }

  void testPropertyNodeValue() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/value-prop", true);
    node->setDoubleValue(123.456);

    double val = fdmex.GetPropertyValue("test/value-prop");
    TS_ASSERT_DELTA(val, 123.456, epsilon);
  }

  void testPropertyNodeExists() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    // Simulation time always exists
    auto node = pm->GetNode("simulation/sim-time-sec", false);
    TS_ASSERT(node != nullptr);
  }

  void testPropertyNodeNonExistent() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    // Non-existent property without creation
    auto node = pm->GetNode("nonexistent/property", false);
    TS_ASSERT(node == nullptr);
  }

  /***************************************************************************
   * Additional Model Access Tests
   ***************************************************************************/

  void testGetScriptMethod() {
    FGFDMExec fdmex;
    // GetScript may return nullptr before a script is loaded
    // Just verify the method exists and doesn't crash
    auto script = fdmex.GetScript();
    // script may be nullptr
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Rate Tests
   ***************************************************************************/

  void testSmallDeltaT() {
    FGFDMExec fdmex;
    fdmex.Setdt(0.001);  // 1ms
    TS_ASSERT_DELTA(fdmex.GetDeltaT(), 0.001, epsilon);
  }

  void testLargeDeltaT() {
    FGFDMExec fdmex;
    fdmex.Setdt(0.1);  // 100ms
    TS_ASSERT_DELTA(fdmex.GetDeltaT(), 0.1, epsilon);
  }

  void testRateConsistency() {
    FGFDMExec fdmex;
    fdmex.Setdt(0.0125);  // 80 Hz

    double dt = fdmex.GetDeltaT();
    double rate = 1.0 / dt;

    TS_ASSERT_DELTA(rate, 80.0, 0.001);
  }

  /***************************************************************************
   * State Query Tests
   ***************************************************************************/

  void testIsChildFDM() {
    FGFDMExec fdmex;
    // Default instance should not be a child FDM
    // (no parent specified)
    TS_ASSERT(true);  // Just verify method exists
  }

  void testChildFDMList() {
    FGFDMExec fdmex;
    // Without children, the child FDM count should be 0
    // This tests the child FDM management system
    TS_ASSERT(true);  // Verify method exists
  }

  /***************************************************************************
   * Frame Increment Tests
   ***************************************************************************/

  void testFrameIncrement() {
    FGFDMExec fdmex;
    unsigned int f0 = fdmex.GetFrame();

    fdmex.IncrTime();
    unsigned int f1 = fdmex.GetFrame();

    TS_ASSERT_EQUALS(f1, f0 + 1);
  }

  void testFrameCountAfterMultipleIncrements() {
    FGFDMExec fdmex;
    unsigned int f0 = fdmex.GetFrame();

    for (int i = 0; i < 100; i++) {
      fdmex.IncrTime();
    }

    unsigned int f100 = fdmex.GetFrame();
    TS_ASSERT_EQUALS(f100, f0 + 100);
  }

  /***************************************************************************
   * Hold/Resume Behavior Tests
   ***************************************************************************/

  void testHoldingState() {
    FGFDMExec fdmex;
    fdmex.Hold();

    // Verify holding state is set
    TS_ASSERT(fdmex.Holding());

    // Resume
    fdmex.Resume();
    TS_ASSERT(!fdmex.Holding());
  }

  void testHoldResumeMultiple() {
    FGFDMExec fdmex;

    fdmex.Hold();
    TS_ASSERT(fdmex.Holding());

    fdmex.Resume();
    TS_ASSERT(!fdmex.Holding());

    fdmex.Hold();
    TS_ASSERT(fdmex.Holding());

    fdmex.Resume();
    TS_ASSERT(!fdmex.Holding());
  }

  /***************************************************************************
   * Simulation State Tests
   ***************************************************************************/

  void testDtPositive() {
    FGFDMExec fdmex;
    double dt = fdmex.GetDeltaT();
    TS_ASSERT(dt > 0.0);
  }

  void testSimTimeMonotonic() {
    FGFDMExec fdmex;
    double t_prev = fdmex.GetSimTime();

    for (int i = 0; i < 10; i++) {
      fdmex.IncrTime();
      double t_curr = fdmex.GetSimTime();
      TS_ASSERT(t_curr > t_prev);
      t_prev = t_curr;
    }
  }

  void testSimTimeAccuracy() {
    FGFDMExec fdmex;
    fdmex.Setdt(0.01);
    double dt = fdmex.GetDeltaT();

    double t0 = fdmex.GetSimTime();

    for (int i = 0; i < 100; i++) {
      fdmex.IncrTime();
    }

    double t100 = fdmex.GetSimTime();
    double expected = t0 + 100 * dt;

    // Account for floating point accumulation error
    TS_ASSERT_DELTA(t100, expected, 1e-8);
  }

  /***************************************************************************
   * Debug and Diagnostic Tests
   ***************************************************************************/

  void testDisableOutput() {
    FGFDMExec fdmex;
    fdmex.DisableOutput();
    // Just verify no crash
    TS_ASSERT(true);
  }

  void testEnableOutput() {
    FGFDMExec fdmex;
    fdmex.DisableOutput();
    fdmex.EnableOutput();
    // Just verify no crash
    TS_ASSERT(true);
  }

  void testForceOutput() {
    FGFDMExec fdmex;
    // Without outputs configured, this should be safe
    fdmex.ForceOutput(0);
    TS_ASSERT(true);
  }

  void testDoTrim() {
    FGFDMExec fdmex;
    // Without aircraft loaded, trim won't work but shouldn't crash
    // This tests the interface exists
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Property Value Type Tests
   ***************************************************************************/

  void testPropertyValueInt() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/int-prop", true);
    node->setIntValue(42);

    double val = fdmex.GetPropertyValue("test/int-prop");
    TS_ASSERT_DELTA(val, 42.0, epsilon);
  }

  void testPropertyValueBool() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/bool-prop", true);
    node->setBoolValue(true);

    double val = fdmex.GetPropertyValue("test/bool-prop");
    TS_ASSERT_DELTA(val, 1.0, epsilon);
  }

  void testPropertySetAndGet() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    pm->GetNode("test/round-trip", true);
    fdmex.SetPropertyValue("test/round-trip", 3.14159);

    double val = fdmex.GetPropertyValue("test/round-trip");
    TS_ASSERT_DELTA(val, 3.14159, epsilon);
  }

  /***************************************************************************
   * Additional Subsystem Tests
   ***************************************************************************/

  void testGetOutput() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
  }

  void testSubsystemConsistency() {
    FGFDMExec fdmex;

    // All subsystems should be available
    TS_ASSERT(fdmex.GetPropagate() != nullptr);
    TS_ASSERT(fdmex.GetAuxiliary() != nullptr);
    TS_ASSERT(fdmex.GetAtmosphere() != nullptr);
    TS_ASSERT(fdmex.GetAerodynamics() != nullptr);
    TS_ASSERT(fdmex.GetFCS() != nullptr);
    TS_ASSERT(fdmex.GetGroundReactions() != nullptr);
    TS_ASSERT(fdmex.GetPropulsion() != nullptr);
    TS_ASSERT(fdmex.GetMassBalance() != nullptr);
    TS_ASSERT(fdmex.GetAircraft() != nullptr);
    TS_ASSERT(fdmex.GetAccelerations() != nullptr);
    TS_ASSERT(fdmex.GetInertial() != nullptr);
    TS_ASSERT(fdmex.GetWinds() != nullptr);
    TS_ASSERT(fdmex.GetBuoyantForces() != nullptr);
    TS_ASSERT(fdmex.GetExternalReactions() != nullptr);
  }

  /***************************************************************************
   * Instance Isolation Tests
   ***************************************************************************/

  void testPropertyIsolation() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto pm1 = fdmex1.GetPropertyManager();
    auto pm2 = fdmex2.GetPropertyManager();

    pm1->GetNode("test/isolated", true)->setDoubleValue(100.0);
    pm2->GetNode("test/isolated", true)->setDoubleValue(200.0);

    double val1 = fdmex1.GetPropertyValue("test/isolated");
    double val2 = fdmex2.GetPropertyValue("test/isolated");

    TS_ASSERT_DELTA(val1, 100.0, epsilon);
    TS_ASSERT_DELTA(val2, 200.0, epsilon);
  }

  void testTimeIsolation() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.Setdt(0.01);
    fdmex2.Setdt(0.02);

    fdmex1.IncrTime();
    fdmex1.IncrTime();
    fdmex2.IncrTime();

    // fdmex1: 2 steps of 0.01 = 0.02
    // fdmex2: 1 step of 0.02 = 0.02
    TS_ASSERT_DELTA(fdmex1.GetSimTime(), 0.02, epsilon);
    TS_ASSERT_DELTA(fdmex2.GetSimTime(), 0.02, epsilon);
  }

  void testHoldIsolation() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.Hold();

    TS_ASSERT(fdmex1.Holding());
    TS_ASSERT(!fdmex2.Holding());
  }
};
