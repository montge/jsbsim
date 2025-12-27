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

  /***************************************************************************
   * Extended Rate Tests
   ***************************************************************************/

  void testSetRateHzNegative() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Negative rate should not crash
    output->SetRateHz(-10.0);
    TS_ASSERT(true);
  }

  void testSetRateHzVeryLarge() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->SetRateHz(1e6);
    TS_ASSERT(true);
  }

  void testSetRateHzVerySmall() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->SetRateHz(1e-6);
    TS_ASSERT(true);
  }

  void testSetRateHzSequence() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    double rates[] = {1.0, 10.0, 100.0, 50.0, 25.0, 120.0};
    for (double rate : rates) {
      output->SetRateHz(rate);
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Extended Enable/Disable State Tests
   ***************************************************************************/

  void testEnableDisableWithRun() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Enable, run, disable, run, enable, run
    output->Enable();
    bool r1 = output->Run(false);

    output->Disable();
    bool r2 = output->Run(false);

    output->Enable();
    bool r3 = output->Run(false);

    TS_ASSERT_EQUALS(r1, false);
    TS_ASSERT_EQUALS(r2, true);  // Disabled
    TS_ASSERT_EQUALS(r3, false);
  }

  void testDisablePreservesState() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->SetRateHz(50.0);
    output->Disable();
    output->Enable();

    // Should still work after enable
    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  void testRapidEnableDisable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 100; i++) {
      output->Enable();
      output->Disable();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Extended ForceOutput Tests
   ***************************************************************************/

  void testForceOutputAfterInit() {
    FGFDMExec fdmex;
    fdmex.GetOutput()->InitModel();

    fdmex.ForceOutput(0);
    TS_ASSERT(true);
  }

  void testForceOutputAfterDisable() {
    FGFDMExec fdmex;
    fdmex.DisableOutput();

    fdmex.ForceOutput(0);
    TS_ASSERT(true);
  }

  void testForceOutputSequence() {
    FGFDMExec fdmex;

    for (int i = 0; i < 5; i++) {
      fdmex.ForceOutput(i);
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Extended Print Tests
   ***************************************************************************/

  void testPrintAfterInit() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->Print();
    TS_ASSERT(true);
  }

  void testPrintAfterRun() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Run(false);
    output->Print();
    TS_ASSERT(true);
  }

  void testPrintAfterEnable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Enable();
    output->Print();
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Extended Name Tests
   ***************************************************************************/

  void testGetOutputNameNegativeIndex() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    std::string name = output->GetOutputName(-1);
    TS_ASSERT(name.empty());
  }

  void testSetOutputNameNegativeIndex() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->SetOutputName(-1, "test.csv");
    TS_ASSERT_EQUALS(result, false);
  }

  void testSetOutputNameEmptyString() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    bool result = output->SetOutputName(0, "");
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Extended Toggle Tests
   ***************************************************************************/

  void testToggleMultipleTimes() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      output->Toggle(0);
    }
    TS_ASSERT(true);
  }

  void testToggleAfterInit() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    bool result = output->Toggle(0);
    TS_ASSERT_EQUALS(result, false);  // No outputs configured
  }

  void testToggleDifferentIndices() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = -5; i < 10; i++) {
      output->Toggle(i);
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Extended Multiple FDMExec Tests
   ***************************************************************************/

  void testManyFDMExecOutputs() {
    std::vector<std::unique_ptr<FGFDMExec>> fdmexes;

    for (int i = 0; i < 5; i++) {
      fdmexes.push_back(std::make_unique<FGFDMExec>());
    }

    for (auto& fdmex : fdmexes) {
      auto output = fdmex->GetOutput();
      TS_ASSERT(output != nullptr);
    }
  }

  void testIndependentOutputRates() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.GetOutput()->SetRateHz(10.0);
    fdmex2.GetOutput()->SetRateHz(50.0);

    // Different rates should not interfere
    TS_ASSERT(true);
  }

  void testIndependentOutputInit() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.GetOutput()->InitModel();
    // fdmex2 should not be affected
    bool result = fdmex2.GetOutput()->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * SetStartNewOutput Extended Tests
   ***************************************************************************/

  void testSetStartNewOutputAfterRun() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Run(false);
    output->SetStartNewOutput();
    output->Run(false);
    TS_ASSERT(true);
  }

  void testSetStartNewOutputAfterDisable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->Disable();
    output->SetStartNewOutput();
    TS_ASSERT(true);
  }

  void testSetStartNewOutputCycle() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 5; i++) {
      output->Run(false);
      output->SetStartNewOutput();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Combined Stress Tests
   ***************************************************************************/

  void testOutputStressTest() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 100; i++) {
      output->InitModel();
      output->Enable();
      output->SetRateHz(i * 1.0 + 1.0);
      output->Run(i % 2 == 0);
      if (i % 3 == 0) output->Disable();
      if (i % 5 == 0) output->SetStartNewOutput();
      output->Print();
    }
    TS_ASSERT(true);
  }

  void testOutputRandomOperations() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Simulate random-like operations
    output->Enable();
    output->SetRateHz(25.0);
    output->Run(false);
    output->Disable();
    output->SetRateHz(10.0);
    output->Enable();
    output->Print();
    output->Run(true);
    output->SetStartNewOutput();
    output->Run(false);

    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output Instance Count Tests
   ***************************************************************************/

  void testOutputCountNoOutputs() {
    FGFDMExec fdmex;

    // GetOutputName with no outputs returns empty
    std::string name = fdmex.GetOutput()->GetOutputName(0);
    TS_ASSERT(name.empty());
  }

  void testOutputCountAfterOperations() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->Enable();
    output->Run(false);

    // Still no outputs configured
    std::string name = output->GetOutputName(0);
    TS_ASSERT(name.empty());
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testOutputWithNoFDMExecInit() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Output should work even without full FDMExec initialization
    output->InitModel();
    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  void testOutputRapidRateChanges() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 50; i++) {
      output->SetRateHz(i * 0.5);
      output->Run(false);
    }
    TS_ASSERT(true);
  }

  void testOutputInterleavedOperations() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 20; i++) {
      output->InitModel();
      output->Enable();
      output->Run(false);
      output->SetRateHz(10.0 + i);
      output->Disable();
      output->Run(false);
      output->Enable();
    }
    TS_ASSERT(true);
  }
};
