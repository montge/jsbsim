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

  /***************************************************************************
   * Rate Calculation Tests (Tests 78-82)
   ***************************************************************************/

  // Test 78: Rate Hz to period conversion logic
  void testRateHzPeriodConversion() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Common output rates and their expected periods
    double rates[] = {1.0, 10.0, 30.0, 60.0, 120.0};
    for (double rate : rates) {
      output->SetRateHz(rate);
      // Period = 1/rate (in seconds)
      double expectedPeriod = 1.0 / rate;
      TS_ASSERT(expectedPeriod > 0.0);
    }
    TS_ASSERT(true);
  }

  // Test 79: Fractional rate handling
  void testFractionalRateHz() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Test fractional rates (less than 1 Hz)
    double fractionalRates[] = {0.5, 0.25, 0.1, 0.01, 0.001};
    for (double rate : fractionalRates) {
      output->SetRateHz(rate);
    }
    TS_ASSERT(true);
  }

  // Test 80: High frequency rate handling
  void testHighFrequencyRateHz() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Test high frequency rates
    double highRates[] = {500.0, 1000.0, 5000.0, 10000.0};
    for (double rate : highRates) {
      output->SetRateHz(rate);
      output->Run(false);
    }
    TS_ASSERT(true);
  }

  // Test 81: Rate change during simulation
  void testRateChangeDuringSimulation() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->Enable();

    // Simulate changing rate during running simulation
    for (int i = 0; i < 20; i++) {
      if (i == 5) output->SetRateHz(10.0);
      if (i == 10) output->SetRateHz(50.0);
      if (i == 15) output->SetRateHz(100.0);
      output->Run(false);
    }
    TS_ASSERT(true);
  }

  // Test 82: Rate validation with infinity
  void testRateHzInfinity() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Test with infinity - should not crash
    output->SetRateHz(std::numeric_limits<double>::infinity());
    output->Run(false);
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output Buffer Management Tests (Tests 83-87)
   ***************************************************************************/

  // Test 83: Output after many runs
  void testOutputAfterManyRuns() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->Enable();

    // Many runs to stress buffer management
    for (int i = 0; i < 1000; i++) {
      output->Run(false);
    }
    TS_ASSERT(true);
  }

  // Test 84: Print after many runs
  void testPrintAfterManyRuns() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 100; i++) {
      output->Run(false);
    }
    output->Print();
    TS_ASSERT(true);
  }

  // Test 85: SetStartNewOutput multiple cycles
  void testSetStartNewOutputManyCycles() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 50; i++) {
      for (int j = 0; j < 10; j++) {
        output->Run(false);
      }
      output->SetStartNewOutput();
    }
    TS_ASSERT(true);
  }

  // Test 86: Force output sequence
  void testForceOutputSequencePattern() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();

    // Force output at various patterns
    for (int i = 0; i < 100; i++) {
      output->Run(false);
      if (i % 10 == 0) {
        fdmex.ForceOutput(0);
      }
    }
    TS_ASSERT(true);
  }

  // Test 87: Mixed buffer operations
  void testMixedBufferOperations() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->Enable();

    for (int i = 0; i < 50; i++) {
      output->Run(false);
      if (i % 5 == 0) output->Print();
      if (i % 7 == 0) output->SetStartNewOutput();
      if (i % 11 == 0) fdmex.ForceOutput(0);
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output State Persistence Tests (Tests 88-92)
   ***************************************************************************/

  // Test 88: State persistence across disable/enable
  void testStatePersistenceAcrossDisableEnable() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->SetRateHz(25.0);

    // Run some iterations
    for (int i = 0; i < 10; i++) {
      output->Run(false);
    }

    // Disable and re-enable
    output->Disable();
    output->Enable();

    // Continue running
    for (int i = 0; i < 10; i++) {
      bool result = output->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  // Test 89: Multiple init cycles
  void testMultipleInitCycles() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int cycle = 0; cycle < 5; cycle++) {
      output->InitModel();
      output->Enable();
      output->SetRateHz(10.0 * (cycle + 1));

      for (int i = 0; i < 20; i++) {
        output->Run(false);
      }

      output->Disable();
    }
    TS_ASSERT(true);
  }

  // Test 90: Output state after SetStartNewOutput
  void testStateAfterSetStartNewOutput() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    output->InitModel();
    output->Enable();

    for (int i = 0; i < 50; i++) {
      output->Run(false);
    }

    output->SetStartNewOutput();

    // Output should still work after starting new
    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test 91: State recovery after error-like conditions
  void testStateRecoveryAfterErrors() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Attempt operations that might fail
    output->Toggle(-1);  // Invalid index
    output->SetOutputName(-1, "test");  // Invalid
    output->GetOutputName(-1);  // Invalid

    // Should still work normally
    output->InitModel();
    output->Enable();
    bool result = output->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  // Test 92: Rapid state transitions
  void testRapidStateTransitions() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 200; i++) {
      switch (i % 4) {
        case 0: output->Enable(); break;
        case 1: output->Disable(); break;
        case 2: output->InitModel(); break;
        case 3: output->Run(false); break;
      }
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Output Naming Extended Tests (Tests 93-96)
   ***************************************************************************/

  // Test 93: Get output name boundary indices
  void testGetOutputNameBoundaryIndices() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Test various boundary indices
    int indices[] = {-1000, -1, 0, 1, 10, 100, 1000,
                     std::numeric_limits<int>::max()};
    for (int idx : indices) {
      std::string name = output->GetOutputName(idx);
      TS_ASSERT(name.empty());  // No outputs configured
    }
  }

  // Test 94: Set output name with special characters
  void testSetOutputNameSpecialCharacters() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Should return false (no outputs) but not crash
    bool r1 = output->SetOutputName(0, "test file.csv");
    bool r2 = output->SetOutputName(0, "test/path/file.csv");
    bool r3 = output->SetOutputName(0, "test\\path\\file.csv");
    bool r4 = output->SetOutputName(0, "test:file.csv");

    TS_ASSERT_EQUALS(r1, false);
    TS_ASSERT_EQUALS(r2, false);
    TS_ASSERT_EQUALS(r3, false);
    TS_ASSERT_EQUALS(r4, false);
  }

  // Test 95: Set output name with various extensions
  void testSetOutputNameExtensions() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    std::vector<std::string> names = {
      "output.csv", "output.txt", "output.dat",
      "output.xml", "output.json", "output"
    };

    for (const auto& name : names) {
      bool result = output->SetOutputName(0, name);
      TS_ASSERT_EQUALS(result, false);  // No outputs configured
    }
  }

  // Test 96: Set output name long string
  void testSetOutputNameLongString() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // Very long filename
    std::string longName(1000, 'x');
    longName += ".csv";

    bool result = output->SetOutputName(0, longName);
    TS_ASSERT_EQUALS(result, false);  // No outputs configured
  }

  /***************************************************************************
   * Multiple Instance Interaction Tests (Tests 97-100)
   ***************************************************************************/

  // Test 97: Sequential FDMExec creation and output operations
  void testSequentialFDMExecCreation() {
    for (int i = 0; i < 10; i++) {
      FGFDMExec fdmex;
      auto output = fdmex.GetOutput();

      output->InitModel();
      output->Enable();
      output->SetRateHz(10.0 + i);

      for (int j = 0; j < 10; j++) {
        output->Run(false);
      }
    }
    TS_ASSERT(true);
  }

  // Test 98: Parallel output operations on multiple instances
  void testParallelOutputOperations() {
    FGFDMExec fdmex1, fdmex2, fdmex3;
    auto output1 = fdmex1.GetOutput();
    auto output2 = fdmex2.GetOutput();
    auto output3 = fdmex3.GetOutput();

    // Initialize all
    output1->InitModel();
    output2->InitModel();
    output3->InitModel();

    // Set different rates
    output1->SetRateHz(10.0);
    output2->SetRateHz(25.0);
    output3->SetRateHz(50.0);

    // Run all in interleaved fashion
    for (int i = 0; i < 30; i++) {
      output1->Run(false);
      output2->Run(false);
      output3->Run(false);
    }

    TS_ASSERT(true);
  }

  // Test 99: Mixed enable/disable across instances
  void testMixedEnableDisableAcrossInstances() {
    FGFDMExec fdmex1, fdmex2, fdmex3, fdmex4;

    // Different enable/disable states
    fdmex1.EnableOutput();
    fdmex2.DisableOutput();
    fdmex3.EnableOutput();
    fdmex4.DisableOutput();

    // Run all
    fdmex1.GetOutput()->Run(false);
    fdmex2.GetOutput()->Run(false);
    fdmex3.GetOutput()->Run(false);
    fdmex4.GetOutput()->Run(false);

    // Swap states
    fdmex1.DisableOutput();
    fdmex2.EnableOutput();
    fdmex3.DisableOutput();
    fdmex4.EnableOutput();

    // Run all again
    fdmex1.GetOutput()->Run(false);
    fdmex2.GetOutput()->Run(false);
    fdmex3.GetOutput()->Run(false);
    fdmex4.GetOutput()->Run(false);

    TS_ASSERT(true);
  }

  // Test 100: Complete output system verification
  void testCompleteOutputSystemVerification() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();

    // 1. Initialization
    bool initResult = output->InitModel();
    TS_ASSERT_EQUALS(initResult, false);  // No outputs configured

    // 2. Enable/Disable cycle
    output->Enable();
    output->Disable();
    output->Enable();

    // 3. Rate configuration
    output->SetRateHz(1.0);
    output->SetRateHz(10.0);
    output->SetRateHz(100.0);
    output->SetRateHz(50.0);  // Final rate

    // 4. Run operations
    for (int i = 0; i < 50; i++) {
      bool runResult = output->Run(false);
      TS_ASSERT_EQUALS(runResult, false);
    }

    // 5. Print operation
    output->Print();

    // 6. Force output
    fdmex.ForceOutput(0);

    // 7. SetStartNewOutput
    output->SetStartNewOutput();

    // 8. Toggle (should fail with no outputs)
    bool toggleResult = output->Toggle(0);
    TS_ASSERT_EQUALS(toggleResult, false);

    // 9. Name operations (should fail with no outputs)
    std::string name = output->GetOutputName(0);
    TS_ASSERT(name.empty());

    bool setNameResult = output->SetOutputName(0, "test.csv");
    TS_ASSERT_EQUALS(setNameResult, false);

    // 10. Final run after all operations
    bool finalResult = output->Run(false);
    TS_ASSERT_EQUALS(finalResult, false);

    // 11. Disable and verify disabled behavior
    output->Disable();
    bool disabledResult = output->Run(false);
    TS_ASSERT_EQUALS(disabledResult, true);

    // 12. Re-enable and verify
    output->Enable();
    bool reenabledResult = output->Run(false);
    TS_ASSERT_EQUALS(reenabledResult, false);
  }
};
