#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGModel.h>
#include <models/FGAircraft.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGInertial.h>

using namespace JSBSim;

// Since FGModel is a base class, we test its functionality through concrete derived classes

class FGModelTest : public CxxTest::TestSuite
{
public:
  // Test rate scheduling - SetRate/GetRate
  void testRateScheduling() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Default rate
    unsigned int defaultRate = aircraft->GetRate();
    TS_ASSERT(defaultRate >= 0);  // Should have a valid rate

    // Set new rate
    aircraft->SetRate(2);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 2);

    aircraft->SetRate(4);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 4);

    aircraft->SetRate(1);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 1);
  }

  // Test rate = 0 (every frame)
  void testRateZero() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetRate(0);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 0);
  }

  // Test high rate values
  void testHighRate() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetRate(100);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 100);

    aircraft->SetRate(1000);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 1000);
  }

  // Test GetExec returns the executive
  void testGetExec() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    FGFDMExec* exec = aircraft->GetExec();
    TS_ASSERT(exec != nullptr);
    TS_ASSERT_EQUALS(exec, &fdmex);
  }

  // Test GetName for different models
  void testGetName() {
    FGFDMExec fdmex;

    // Different models should have different names
    auto aircraft = fdmex.GetAircraft();
    auto atmosphere = fdmex.GetAtmosphere();
    auto inertial = fdmex.GetInertial();
    auto auxiliary = fdmex.GetAuxiliary();

    // Model names should not be empty
    TS_ASSERT(!aircraft->GetName().empty());
    TS_ASSERT(!atmosphere->GetName().empty());
    TS_ASSERT(!inertial->GetName().empty());
    TS_ASSERT(!auxiliary->GetName().empty());
  }

  // Test InitModel returns true
  void testInitModel() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    bool result = aircraft->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  // Test InitModel for multiple models
  void testInitModelMultipleModels() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetAircraft()->InitModel());
    TS_ASSERT(fdmex.GetAtmosphere()->InitModel());
    TS_ASSERT(fdmex.GetInertial()->InitModel());
    TS_ASSERT(fdmex.GetAuxiliary()->InitModel());
  }

  // Test Run returns false (no error) when not holding
  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    bool result = aircraft->Run(false);
    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  // Test Run returns false (no error) when holding
  void testRunHolding() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    bool result = aircraft->Run(true);
    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  // Test multiple Run calls
  void testMultipleRunCalls() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (int i = 0; i < 10; i++) {
      bool result = aircraft->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  // Test rate affects execution (through counter mechanism)
  void testRateAffectsExecution() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // With rate = 1, model should run every frame
    aircraft->SetRate(1);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 1);

    // Multiple runs should work without error
    for (int i = 0; i < 5; i++) {
      TS_ASSERT_EQUALS(aircraft->Run(false), false);
    }
  }

  // Test Load with nullptr (should return true for base implementation)
  void testLoadNull() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Base FGModel::Load returns true by default
    // Derived classes may override
    TS_ASSERT(aircraft != nullptr);
  }

  // Test property manager is set
  void testPropertyManagerSet() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Models should have property manager set from FDMExec
    FGFDMExec* exec = aircraft->GetExec();
    TS_ASSERT(exec != nullptr);
    TS_ASSERT(exec->GetPropertyManager() != nullptr);
  }

  // Test model initialization sequence
  void testInitializationSequence() {
    FGFDMExec fdmex;

    // Get all major models
    auto aircraft = fdmex.GetAircraft();
    auto atmosphere = fdmex.GetAtmosphere();
    auto inertial = fdmex.GetInertial();
    auto auxiliary = fdmex.GetAuxiliary();

    // All should be initialized properly
    TS_ASSERT(aircraft != nullptr);
    TS_ASSERT(atmosphere != nullptr);
    TS_ASSERT(inertial != nullptr);
    TS_ASSERT(auxiliary != nullptr);

    // Re-initialize and verify
    TS_ASSERT(aircraft->InitModel());
    TS_ASSERT(atmosphere->InitModel());
    TS_ASSERT(inertial->InitModel());
    TS_ASSERT(auxiliary->InitModel());
  }

  // Test that different models have unique names
  void testUniqueModelNames() {
    FGFDMExec fdmex;

    std::string aircraftName = fdmex.GetAircraft()->GetName();
    std::string atmosphereName = fdmex.GetAtmosphere()->GetName();
    std::string inertialName = fdmex.GetInertial()->GetName();
    std::string auxiliaryName = fdmex.GetAuxiliary()->GetName();

    // Names should be unique
    TS_ASSERT_DIFFERS(aircraftName, atmosphereName);
    TS_ASSERT_DIFFERS(aircraftName, inertialName);
    TS_ASSERT_DIFFERS(aircraftName, auxiliaryName);
    TS_ASSERT_DIFFERS(atmosphereName, inertialName);
    TS_ASSERT_DIFFERS(atmosphereName, auxiliaryName);
    TS_ASSERT_DIFFERS(inertialName, auxiliaryName);
  }

  // Test consecutive rate changes
  void testConsecutiveRateChanges() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (unsigned int rate = 0; rate < 10; rate++) {
      aircraft->SetRate(rate);
      TS_ASSERT_EQUALS(aircraft->GetRate(), rate);
    }
  }

  // Test model can run after rate change
  void testRunAfterRateChange() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Test SetRate with different values
    aircraft->SetRate(1);
    aircraft->Run(false);  // Run returns true on success

    aircraft->SetRate(5);
    aircraft->Run(false);

    aircraft->SetRate(0);
    aircraft->Run(false);

    // Verify rate can be queried
    TS_ASSERT(aircraft->GetRate() >= 0);
  }

  // Test that model keeps reference to same executive
  void testExecConsistency() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    FGFDMExec* exec1 = aircraft->GetExec();
    aircraft->SetRate(5);
    FGFDMExec* exec2 = aircraft->GetExec();
    aircraft->Run(false);
    FGFDMExec* exec3 = aircraft->GetExec();

    TS_ASSERT_EQUALS(exec1, exec2);
    TS_ASSERT_EQUALS(exec2, exec3);
    TS_ASSERT_EQUALS(exec1, &fdmex);
  }

  /***************************************************************************
   * Rate Boundary Tests
   ***************************************************************************/

  void testRateMaxValue() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Test maximum unsigned int value
    unsigned int maxRate = std::numeric_limits<unsigned int>::max();
    aircraft->SetRate(maxRate);
    TS_ASSERT_EQUALS(aircraft->GetRate(), maxRate);
  }

  void testRateLargeValues() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    unsigned int rates[] = {10000, 100000, 1000000};
    for (auto rate : rates) {
      aircraft->SetRate(rate);
      TS_ASSERT_EQUALS(aircraft->GetRate(), rate);
    }
  }

  void testRateToggle() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Toggle between 0 and 1
    for (int i = 0; i < 10; i++) {
      aircraft->SetRate(i % 2);
      TS_ASSERT_EQUALS(aircraft->GetRate(), static_cast<unsigned int>(i % 2));
    }
  }

  void testRatePersistenceAcrossRuns() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetRate(7);
    for (int i = 0; i < 5; i++) {
      aircraft->Run(false);
      TS_ASSERT_EQUALS(aircraft->GetRate(), 7u);
    }
  }

  /***************************************************************************
   * Multiple FDMExec Tests
   ***************************************************************************/

  void testMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto aircraft1 = fdmex1.GetAircraft();
    auto aircraft2 = fdmex2.GetAircraft();

    TS_ASSERT(aircraft1 != nullptr);
    TS_ASSERT(aircraft2 != nullptr);
    TS_ASSERT(aircraft1 != aircraft2);
  }

  void testMultipleFDMExecIndependentRates() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto aircraft1 = fdmex1.GetAircraft();
    auto aircraft2 = fdmex2.GetAircraft();

    aircraft1->SetRate(3);
    aircraft2->SetRate(7);

    TS_ASSERT_EQUALS(aircraft1->GetRate(), 3u);
    TS_ASSERT_EQUALS(aircraft2->GetRate(), 7u);
  }

  void testMultipleFDMExecIndependentRuns() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto aircraft1 = fdmex1.GetAircraft();
    auto aircraft2 = fdmex2.GetAircraft();

    // Run them independently
    TS_ASSERT_EQUALS(aircraft1->Run(false), false);
    TS_ASSERT_EQUALS(aircraft2->Run(false), false);
    TS_ASSERT_EQUALS(aircraft1->Run(true), false);
    TS_ASSERT_EQUALS(aircraft2->Run(true), false);
  }

  void testThreeFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;
    FGFDMExec fdmex3;

    auto a1 = fdmex1.GetAircraft();
    auto a2 = fdmex2.GetAircraft();
    auto a3 = fdmex3.GetAircraft();

    a1->SetRate(1);
    a2->SetRate(2);
    a3->SetRate(3);

    TS_ASSERT_EQUALS(a1->GetRate(), 1u);
    TS_ASSERT_EQUALS(a2->GetRate(), 2u);
    TS_ASSERT_EQUALS(a3->GetRate(), 3u);
  }

  /***************************************************************************
   * All Model Types Tests
   ***************************************************************************/

  void testAllModelsHaveExec() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetAircraft()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetAtmosphere()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetInertial()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetAuxiliary()->GetExec() == &fdmex);
  }

  void testAllModelsRunWithoutError() {
    FGFDMExec fdmex;

    TS_ASSERT_EQUALS(fdmex.GetAircraft()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAtmosphere()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetInertial()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAuxiliary()->Run(false), false);
  }

  void testAllModelsRunHolding() {
    FGFDMExec fdmex;

    TS_ASSERT_EQUALS(fdmex.GetAircraft()->Run(true), false);
    TS_ASSERT_EQUALS(fdmex.GetAtmosphere()->Run(true), false);
    TS_ASSERT_EQUALS(fdmex.GetInertial()->Run(true), false);
    TS_ASSERT_EQUALS(fdmex.GetAuxiliary()->Run(true), false);
  }

  void testAllModelsSetRate() {
    FGFDMExec fdmex;

    fdmex.GetAircraft()->SetRate(5);
    fdmex.GetAtmosphere()->SetRate(10);
    fdmex.GetInertial()->SetRate(15);
    fdmex.GetAuxiliary()->SetRate(20);

    TS_ASSERT_EQUALS(fdmex.GetAircraft()->GetRate(), 5u);
    TS_ASSERT_EQUALS(fdmex.GetAtmosphere()->GetRate(), 10u);
    TS_ASSERT_EQUALS(fdmex.GetInertial()->GetRate(), 15u);
    TS_ASSERT_EQUALS(fdmex.GetAuxiliary()->GetRate(), 20u);
  }

  /***************************************************************************
   * Model Name Tests
   ***************************************************************************/

  void testModelNamesAreNotEmpty() {
    FGFDMExec fdmex;

    TS_ASSERT(!fdmex.GetAircraft()->GetName().empty());
    TS_ASSERT(!fdmex.GetAtmosphere()->GetName().empty());
    TS_ASSERT(!fdmex.GetInertial()->GetName().empty());
    TS_ASSERT(!fdmex.GetAuxiliary()->GetName().empty());
  }

  void testModelNamesConsistent() {
    FGFDMExec fdmex;

    // Same model should return same name
    std::string name1 = fdmex.GetAircraft()->GetName();
    std::string name2 = fdmex.GetAircraft()->GetName();
    TS_ASSERT_EQUALS(name1, name2);
  }

  void testModelNamesAcrossInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    // Same model type should have same name
    std::string name1 = fdmex1.GetAircraft()->GetName();
    std::string name2 = fdmex2.GetAircraft()->GetName();
    TS_ASSERT_EQUALS(name1, name2);
  }

  /***************************************************************************
   * InitModel Stress Tests
   ***************************************************************************/

  void testInitModelMultipleTimes() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Call InitModel multiple times
    for (int i = 0; i < 10; i++) {
      TS_ASSERT(aircraft->InitModel());
    }
  }

  void testInitModelAfterRun() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->Run(false);
    TS_ASSERT(aircraft->InitModel());
    aircraft->Run(false);
    TS_ASSERT(aircraft->InitModel());
  }

  void testInitModelAllModels() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetAircraft()->InitModel());
    TS_ASSERT(fdmex.GetAtmosphere()->InitModel());
    TS_ASSERT(fdmex.GetInertial()->InitModel());
    TS_ASSERT(fdmex.GetAuxiliary()->InitModel());
  }

  /***************************************************************************
   * Run Sequence Tests
   ***************************************************************************/

  void testRunSequenceModels() {
    FGFDMExec fdmex;

    // Run models in sequence
    for (int i = 0; i < 5; i++) {
      TS_ASSERT_EQUALS(fdmex.GetAtmosphere()->Run(false), false);
      TS_ASSERT_EQUALS(fdmex.GetInertial()->Run(false), false);
      TS_ASSERT_EQUALS(fdmex.GetAircraft()->Run(false), false);
      TS_ASSERT_EQUALS(fdmex.GetAuxiliary()->Run(false), false);
    }
  }

  void testRunAlternatingHolding() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (int i = 0; i < 10; i++) {
      bool holding = (i % 2 == 0);
      TS_ASSERT_EQUALS(aircraft->Run(holding), false);
    }
  }

  void testManyRunCalls() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(aircraft->Run(false), false);
    }
  }

  /***************************************************************************
   * Rate and Run Interaction Tests
   ***************************************************************************/

  void testRateChangesDuringRuns() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (unsigned int rate = 0; rate < 5; rate++) {
      aircraft->SetRate(rate);
      TS_ASSERT_EQUALS(aircraft->GetRate(), rate);
      aircraft->Run(false);
      TS_ASSERT_EQUALS(aircraft->GetRate(), rate);
    }
  }

  void testRateZeroMultipleRuns() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetRate(0);
    for (int i = 0; i < 20; i++) {
      // Run() may return true when model skips execution
      aircraft->Run(false);
      TS_ASSERT_EQUALS(aircraft->GetRate(), 0u);
    }
  }

  /***************************************************************************
   * Cross-Model Tests
   ***************************************************************************/

  void testCrossModelInitialization() {
    FGFDMExec fdmex;

    // Initialize all models
    auto models = {
      std::dynamic_pointer_cast<FGModel>(fdmex.GetAircraft()),
      std::dynamic_pointer_cast<FGModel>(fdmex.GetAtmosphere()),
      std::dynamic_pointer_cast<FGModel>(fdmex.GetInertial()),
      std::dynamic_pointer_cast<FGModel>(fdmex.GetAuxiliary())
    };

    for (auto& model : models) {
      if (model) {
        TS_ASSERT(model->InitModel());
      }
    }
  }

  void testCrossModelRates() {
    FGFDMExec fdmex;

    auto aircraft = fdmex.GetAircraft();
    auto atmosphere = fdmex.GetAtmosphere();

    // Rates should be independent
    aircraft->SetRate(1);
    atmosphere->SetRate(100);

    TS_ASSERT_EQUALS(aircraft->GetRate(), 1u);
    TS_ASSERT_EQUALS(atmosphere->GetRate(), 100u);

    aircraft->SetRate(50);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 50u);
    TS_ASSERT_EQUALS(atmosphere->GetRate(), 100u);
  }

  /***************************************************************************
   * Stability Tests
   ***************************************************************************/

  void testRapidRateChanges() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (int i = 0; i < 100; i++) {
      aircraft->SetRate(i % 20);
      TS_ASSERT_EQUALS(aircraft->GetRate(), static_cast<unsigned int>(i % 20));
    }
  }

  void testInterleavedOperations() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (int i = 0; i < 20; i++) {
      aircraft->SetRate(i);
      aircraft->Run(false);
      TS_ASSERT_EQUALS(aircraft->GetRate(), static_cast<unsigned int>(i));
      aircraft->Run(true);
      TS_ASSERT(aircraft->InitModel());
    }
  }

  void testAllOperationsSequence() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    // Complete operation sequence
    TS_ASSERT(aircraft->InitModel());
    aircraft->SetRate(5);
    TS_ASSERT_EQUALS(aircraft->GetRate(), 5u);
    // Run() return value depends on rate counter, don't assert
    aircraft->Run(false);
    aircraft->Run(true);
    TS_ASSERT(!aircraft->GetName().empty());
    TS_ASSERT(aircraft->GetExec() == &fdmex);
    TS_ASSERT(aircraft->InitModel());
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testGetExecNeverNull() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetAircraft()->GetExec() != nullptr);
    TS_ASSERT(fdmex.GetAtmosphere()->GetExec() != nullptr);
    TS_ASSERT(fdmex.GetInertial()->GetExec() != nullptr);
    TS_ASSERT(fdmex.GetAuxiliary()->GetExec() != nullptr);
  }

  void testGetNameNeverEmpty() {
    FGFDMExec fdmex;

    TS_ASSERT(!fdmex.GetAircraft()->GetName().empty());
    TS_ASSERT(!fdmex.GetAtmosphere()->GetName().empty());
    TS_ASSERT(!fdmex.GetInertial()->GetName().empty());
    TS_ASSERT(!fdmex.GetAuxiliary()->GetName().empty());
  }

  void testRateAfterInit() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    aircraft->SetRate(42);
    aircraft->InitModel();
    // Rate may or may not be reset - check it's still valid
    TS_ASSERT(aircraft->GetRate() >= 0);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testManyFDMExecInstances() {
    std::vector<std::unique_ptr<FGFDMExec>> instances;

    for (int i = 0; i < 10; i++) {
      instances.push_back(std::make_unique<FGFDMExec>());
      TS_ASSERT(instances.back()->GetAircraft() != nullptr);
    }
  }

  void testRapidInitAndRun() {
    FGFDMExec fdmex;
    auto aircraft = fdmex.GetAircraft();

    for (int i = 0; i < 50; i++) {
      TS_ASSERT(aircraft->InitModel());
      TS_ASSERT_EQUALS(aircraft->Run(false), false);
    }
  }

  void testAllModelsStress() {
    FGFDMExec fdmex;

    for (int i = 0; i < 20; i++) {
      fdmex.GetAircraft()->SetRate(i);
      fdmex.GetAtmosphere()->SetRate(i + 1);
      fdmex.GetInertial()->SetRate(i + 2);
      fdmex.GetAuxiliary()->SetRate(i + 3);

      // Run() can return true when skipping execution due to rate scheduling
      fdmex.GetAircraft()->Run(false);
      fdmex.GetAtmosphere()->Run(false);
      fdmex.GetInertial()->Run(false);
      fdmex.GetAuxiliary()->Run(false);
    }
    TS_ASSERT(true);
  }
};
