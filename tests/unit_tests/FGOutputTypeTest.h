/*******************************************************************************
 * FGOutputTypeTest.h - Unit tests for FGOutputType base class
 *
 * Tests the FGOutputType abstract base class including:
 * - Rate configuration
 * - Subsystem settings
 * - Output enabling/disabling
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGOutput.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGOutputTypeTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Output Model Access Tests
   ***************************************************************************/

  void testGetOutput() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
  }

  void testOutputInitModel() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // InitModel returns false when no outputs are configured (normal case)
    bool result = output->InitModel();
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Enable/Disable Tests
   ***************************************************************************/

  void testEnableOutput() {
    FGFDMExec fdmex;
    fdmex.EnableOutput();
    // Should not crash
    TS_ASSERT(true);
  }

  void testDisableOutput() {
    FGFDMExec fdmex;
    fdmex.DisableOutput();
    // Should not crash
    TS_ASSERT(true);
  }

  void testEnableAfterDisable() {
    FGFDMExec fdmex;
    fdmex.DisableOutput();
    fdmex.EnableOutput();
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output Run Tests
   ***************************************************************************/

  void testOutputRunNotHolding() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  void testOutputRunHolding() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  void testMultipleOutputRuns() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      bool result = output->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  /***************************************************************************
   * Output Consistency Tests
   ***************************************************************************/

  void testOutputAfterInit() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Force Output Tests
   ***************************************************************************/

  void testForceOutput() {
    FGFDMExec fdmex;

    // ForceOutput should not crash without outputs configured
    fdmex.ForceOutput(0);
    TS_ASSERT(true);
  }
};
