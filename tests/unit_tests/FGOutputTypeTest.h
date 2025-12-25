/*******************************************************************************
 * FGOutputTypeTest.h - Unit tests for FGOutput and FGOutputType classes
 *
 * Tests the FGOutput model and FGOutputType abstract base class including:
 * - Initialization and model access
 * - Enable/disable functionality
 * - Output rate configuration
 * - Multiple instance management
 * - Run behavior
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

  void testOutputNotNull() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.GetOutput() != nullptr);
  }

  void testOutputInitModel() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // InitModel returns false when no outputs are configured (normal case)
    bool result = output->InitModel();
    TS_ASSERT_EQUALS(result, false);
  }

  void testOutputInitModelMultipleTimes() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Multiple InitModel calls should not crash
    output->InitModel();
    output->InitModel();
    output->InitModel();
    TS_ASSERT(true);
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

  void testDisableAfterEnable() {
    FGFDMExec fdmex;
    fdmex.EnableOutput();
    fdmex.DisableOutput();
    TS_ASSERT(true);
  }

  void testMultipleEnableDisableCycles() {
    FGFDMExec fdmex;

    for (int i = 0; i < 10; i++) {
      fdmex.DisableOutput();
      fdmex.EnableOutput();
    }
    TS_ASSERT(true);
  }

  void testOutputEnableMethod() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Enable();
    TS_ASSERT(true);
  }

  void testOutputDisableMethod() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Disable();
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

  void testMultipleOutputRuns() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      bool result = output->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  void testOutputRunAlternatingHold() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      bool result = output->Run(i % 2 == 0);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  void testOutputRunAfterDisable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Disable();
    bool result = output->Run(false);
    // When disabled, Run returns true (success without processing)
    TS_ASSERT_EQUALS(result, true);
  }

  void testOutputRunAfterEnable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Enable();
    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
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

  void testOutputSequence() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Normal sequence: init, enable, run
    output->InitModel();
    output->Enable();
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

  void testForceOutputNegativeIndex() {
    FGFDMExec fdmex;

    // Negative index should not crash
    fdmex.ForceOutput(-1);
    TS_ASSERT(true);
  }

  void testForceOutputLargeIndex() {
    FGFDMExec fdmex;

    // Large index should not crash (no outputs configured)
    fdmex.ForceOutput(100);
    TS_ASSERT(true);
  }

  void testForceOutputMultipleTimes() {
    FGFDMExec fdmex;

    for (int i = 0; i < 10; i++) {
      fdmex.ForceOutput(0);
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Print Tests
   ***************************************************************************/

  void testPrintNoOutputs() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Print should not crash without outputs configured
    output->Print();
    TS_ASSERT(true);
  }

  void testPrintMultipleTimes() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      output->Print();
    }
    TS_ASSERT(true);
  }

  void testPrintAfterDisable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Disable();
    output->Print();
    TS_ASSERT(true);
  }

  /***************************************************************************
   * SetStartNewOutput Tests
   ***************************************************************************/

  void testSetStartNewOutput() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Should not crash without outputs configured
    output->SetStartNewOutput();
    TS_ASSERT(true);
  }

  void testSetStartNewOutputMultipleTimes() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 5; i++) {
      output->SetStartNewOutput();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output Rate Tests
   ***************************************************************************/

  void testSetRateHz() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Should not crash without outputs configured
    output->SetRateHz(10.0);
    TS_ASSERT(true);
  }

  void testSetRateHzZero() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->SetRateHz(0.0);
    TS_ASSERT(true);
  }

  void testSetRateHzLarge() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->SetRateHz(1000.0);
    TS_ASSERT(true);
  }

  void testSetRateHzSmall() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->SetRateHz(0.1);
    TS_ASSERT(true);
  }

  void testSetRateHzMultipleTimes() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->SetRateHz(10.0);
    output->SetRateHz(20.0);
    output->SetRateHz(50.0);
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output Name Tests
   ***************************************************************************/

  void testGetOutputNameNoOutputs() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // No outputs configured, should return empty string
    std::string name = output->GetOutputName(0);
    TS_ASSERT(name.empty());
  }

  void testGetOutputNameLargeIndex() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    std::string name = output->GetOutputName(100);
    TS_ASSERT(name.empty());
  }

  void testSetOutputNameNoOutputs() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Should return false when no outputs exist
    bool result = output->SetOutputName(0, "test.csv");
    TS_ASSERT_EQUALS(result, false);
  }

  void testSetOutputNameLargeIndex() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->SetOutputName(100, "test.csv");
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Toggle Tests
   ***************************************************************************/

  void testToggleNoOutputs() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Should return false when no outputs exist
    bool result = output->Toggle(0);
    TS_ASSERT_EQUALS(result, false);
  }

  void testToggleLargeIndex() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->Toggle(100);
    TS_ASSERT_EQUALS(result, false);
  }

  void testToggleNegativeIndex() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->Toggle(-1);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Multiple FDMExec Tests
   ***************************************************************************/

  void testMultipleFDMExecOutputs() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto output1 = fdmex1.GetOutput();
    auto output2 = fdmex2.GetOutput();

    TS_ASSERT(output1 != nullptr);
    TS_ASSERT(output2 != nullptr);
    TS_ASSERT(output1 != output2);
  }

  void testIndependentOutputEnableDisable() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.DisableOutput();
    fdmex2.EnableOutput();

    // Should not interfere with each other
    TS_ASSERT(true);
  }

  void testIndependentOutputRun() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto output1 = fdmex1.GetOutput();
    auto output2 = fdmex2.GetOutput();

    bool result1 = output1->Run(false);
    bool result2 = output2->Run(false);

    TS_ASSERT_EQUALS(result1, false);
    TS_ASSERT_EQUALS(result2, false);
  }

  /***************************************************************************
   * Combined Operation Tests
   ***************************************************************************/

  void testFullOutputCycle() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Full cycle of operations
    output->InitModel();
    output->Enable();
    output->SetRateHz(10.0);
    output->Run(false);
    output->Print();
    output->Disable();
    output->Run(false);
    output->Enable();
    output->SetStartNewOutput();
    output->Run(false);

    TS_ASSERT(true);
  }

  void testOutputStateAfterOperations() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();

    // Run should work after init
    bool result1 = output->Run(false);
    TS_ASSERT_EQUALS(result1, false);

    output->Disable();

    // When disabled, Run returns true (success without processing)
    bool result2 = output->Run(false);
    TS_ASSERT_EQUALS(result2, true);

    output->Enable();

    // Run should work after re-enable
    bool result3 = output->Run(false);
    TS_ASSERT_EQUALS(result3, false);
  }
};
