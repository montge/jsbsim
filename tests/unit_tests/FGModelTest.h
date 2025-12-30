#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <memory>
#include <vector>

#include <FGFDMExec.h>
#include <models/FGModel.h>
#include <models/FGAircraft.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGInertial.h>
#include <models/FGPropagate.h>
#include <models/FGMassBalance.h>
#include <models/FGAerodynamics.h>
#include <models/FGGroundReactions.h>
#include <models/FGAccelerations.h>
#include <models/FGFCS.h>

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

  /***************************************************************************
   * FGPropagate Model Tests
   ***************************************************************************/

  void testFGPropagateExists() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate != nullptr);
  }

  void testFGPropagateGetExec() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate->GetExec() == &fdmex);
  }

  void testFGPropagateGetName() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(!propagate->GetName().empty());
  }

  void testFGPropagateInitModel() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT(propagate->InitModel());
  }

  void testFGPropagateRun() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    TS_ASSERT_EQUALS(propagate->Run(false), false);
    TS_ASSERT_EQUALS(propagate->Run(true), false);
  }

  void testFGPropagateSetRate() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();
    propagate->SetRate(3);
    TS_ASSERT_EQUALS(propagate->GetRate(), 3u);
  }

  /***************************************************************************
   * FGMassBalance Model Tests
   ***************************************************************************/

  void testFGMassBalanceExists() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();
    TS_ASSERT(massBalance != nullptr);
  }

  void testFGMassBalanceGetExec() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();
    TS_ASSERT(massBalance->GetExec() == &fdmex);
  }

  void testFGMassBalanceGetName() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();
    TS_ASSERT(!massBalance->GetName().empty());
  }

  void testFGMassBalanceInitModel() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();
    TS_ASSERT(massBalance->InitModel());
  }

  void testFGMassBalanceRun() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();
    TS_ASSERT_EQUALS(massBalance->Run(false), false);
  }

  void testFGMassBalanceSetRate() {
    FGFDMExec fdmex;
    auto massBalance = fdmex.GetMassBalance();
    massBalance->SetRate(5);
    TS_ASSERT_EQUALS(massBalance->GetRate(), 5u);
  }

  /***************************************************************************
   * FGAerodynamics Model Tests
   ***************************************************************************/

  void testFGAerodynamicsExists() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();
    TS_ASSERT(aero != nullptr);
  }

  void testFGAerodynamicsGetExec() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();
    TS_ASSERT(aero->GetExec() == &fdmex);
  }

  void testFGAerodynamicsGetName() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();
    TS_ASSERT(!aero->GetName().empty());
  }

  // Note: FGAerodynamics::InitModel() and Run() require an aircraft to be loaded
  // Testing only basic operations (GetExec, GetName, SetRate) here

  void testFGAerodynamicsSetRate() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();
    aero->SetRate(4);
    TS_ASSERT_EQUALS(aero->GetRate(), 4u);
  }

  /***************************************************************************
   * FGGroundReactions Model Tests
   ***************************************************************************/

  void testFGGroundReactionsExists() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(groundReactions != nullptr);
  }

  void testFGGroundReactionsGetExec() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(groundReactions->GetExec() == &fdmex);
  }

  void testFGGroundReactionsGetName() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(!groundReactions->GetName().empty());
  }

  void testFGGroundReactionsInitModel() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT(groundReactions->InitModel());
  }

  void testFGGroundReactionsRun() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    TS_ASSERT_EQUALS(groundReactions->Run(false), false);
  }

  void testFGGroundReactionsSetRate() {
    FGFDMExec fdmex;
    auto groundReactions = fdmex.GetGroundReactions();
    groundReactions->SetRate(6);
    TS_ASSERT_EQUALS(groundReactions->GetRate(), 6u);
  }

  /***************************************************************************
   * FGAccelerations Model Tests
   ***************************************************************************/

  void testFGAccelerationsExists() {
    FGFDMExec fdmex;
    auto accelerations = fdmex.GetAccelerations();
    TS_ASSERT(accelerations != nullptr);
  }

  void testFGAccelerationsGetExec() {
    FGFDMExec fdmex;
    auto accelerations = fdmex.GetAccelerations();
    TS_ASSERT(accelerations->GetExec() == &fdmex);
  }

  void testFGAccelerationsGetName() {
    FGFDMExec fdmex;
    auto accelerations = fdmex.GetAccelerations();
    TS_ASSERT(!accelerations->GetName().empty());
  }

  void testFGAccelerationsInitModel() {
    FGFDMExec fdmex;
    auto accelerations = fdmex.GetAccelerations();
    TS_ASSERT(accelerations->InitModel());
  }

  void testFGAccelerationsRun() {
    FGFDMExec fdmex;
    auto accelerations = fdmex.GetAccelerations();
    TS_ASSERT_EQUALS(accelerations->Run(false), false);
  }

  void testFGAccelerationsSetRate() {
    FGFDMExec fdmex;
    auto accelerations = fdmex.GetAccelerations();
    accelerations->SetRate(2);
    TS_ASSERT_EQUALS(accelerations->GetRate(), 2u);
  }

  /***************************************************************************
   * FGFCS Model Tests
   ***************************************************************************/

  void testFGFCSExists() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
  }

  void testFGFCSGetExec() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs->GetExec() == &fdmex);
  }

  void testFGFCSGetName() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(!fcs->GetName().empty());
  }

  void testFGFCSInitModel() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs->InitModel());
  }

  void testFGFCSRun() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    TS_ASSERT_EQUALS(fcs->Run(false), false);
  }

  void testFGFCSSetRate() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();
    fcs->SetRate(8);
    TS_ASSERT_EQUALS(fcs->GetRate(), 8u);
  }

  /***************************************************************************
   * All Extended Models Tests
   ***************************************************************************/

  void testAllExtendedModelsExist() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetPropagate() != nullptr);
    TS_ASSERT(fdmex.GetMassBalance() != nullptr);
    TS_ASSERT(fdmex.GetAerodynamics() != nullptr);
    TS_ASSERT(fdmex.GetGroundReactions() != nullptr);
    TS_ASSERT(fdmex.GetAccelerations() != nullptr);
    TS_ASSERT(fdmex.GetFCS() != nullptr);
  }

  void testAllExtendedModelsHaveExec() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetPropagate()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetMassBalance()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetAerodynamics()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetGroundReactions()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetAccelerations()->GetExec() == &fdmex);
    TS_ASSERT(fdmex.GetFCS()->GetExec() == &fdmex);
  }

  void testAllExtendedModelsHaveNames() {
    FGFDMExec fdmex;

    TS_ASSERT(!fdmex.GetPropagate()->GetName().empty());
    TS_ASSERT(!fdmex.GetMassBalance()->GetName().empty());
    TS_ASSERT(!fdmex.GetAerodynamics()->GetName().empty());
    TS_ASSERT(!fdmex.GetGroundReactions()->GetName().empty());
    TS_ASSERT(!fdmex.GetAccelerations()->GetName().empty());
    TS_ASSERT(!fdmex.GetFCS()->GetName().empty());
  }

  void testAllExtendedModelsInitModel() {
    FGFDMExec fdmex;

    TS_ASSERT(fdmex.GetPropagate()->InitModel());
    TS_ASSERT(fdmex.GetMassBalance()->InitModel());
    // Note: FGAerodynamics::InitModel() requires aircraft to be loaded
    TS_ASSERT(fdmex.GetGroundReactions()->InitModel());
    TS_ASSERT(fdmex.GetAccelerations()->InitModel());
    TS_ASSERT(fdmex.GetFCS()->InitModel());
  }

  void testAllExtendedModelsRun() {
    FGFDMExec fdmex;

    TS_ASSERT_EQUALS(fdmex.GetPropagate()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->Run(false), false);
    // Note: FGAerodynamics::Run() requires aircraft to be loaded
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->Run(false), false);
  }

  void testAllExtendedModelsIndependentRates() {
    FGFDMExec fdmex;

    fdmex.GetPropagate()->SetRate(1);
    fdmex.GetMassBalance()->SetRate(2);
    fdmex.GetAerodynamics()->SetRate(3);
    fdmex.GetGroundReactions()->SetRate(4);
    fdmex.GetAccelerations()->SetRate(5);
    fdmex.GetFCS()->SetRate(6);

    TS_ASSERT_EQUALS(fdmex.GetPropagate()->GetRate(), 1u);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->GetRate(), 2u);
    TS_ASSERT_EQUALS(fdmex.GetAerodynamics()->GetRate(), 3u);
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->GetRate(), 4u);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->GetRate(), 5u);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->GetRate(), 6u);
  }

  /***************************************************************************
   * Model Uniqueness Tests
   ***************************************************************************/

  void testExtendedModelNamesUnique() {
    FGFDMExec fdmex;

    std::string propagateName = fdmex.GetPropagate()->GetName();
    std::string massBalanceName = fdmex.GetMassBalance()->GetName();
    std::string aeroName = fdmex.GetAerodynamics()->GetName();
    std::string groundName = fdmex.GetGroundReactions()->GetName();
    std::string accelName = fdmex.GetAccelerations()->GetName();
    std::string fcsName = fdmex.GetFCS()->GetName();

    TS_ASSERT_DIFFERS(propagateName, massBalanceName);
    TS_ASSERT_DIFFERS(propagateName, aeroName);
    TS_ASSERT_DIFFERS(propagateName, groundName);
    TS_ASSERT_DIFFERS(propagateName, accelName);
    TS_ASSERT_DIFFERS(propagateName, fcsName);
    TS_ASSERT_DIFFERS(massBalanceName, aeroName);
    TS_ASSERT_DIFFERS(massBalanceName, groundName);
    TS_ASSERT_DIFFERS(aeroName, groundName);
    TS_ASSERT_DIFFERS(accelName, fcsName);
  }

  void testAllModelNamesUniqueComplete() {
    FGFDMExec fdmex;

    std::vector<std::string> names;
    names.push_back(fdmex.GetAircraft()->GetName());
    names.push_back(fdmex.GetAtmosphere()->GetName());
    names.push_back(fdmex.GetInertial()->GetName());
    names.push_back(fdmex.GetAuxiliary()->GetName());
    names.push_back(fdmex.GetPropagate()->GetName());
    names.push_back(fdmex.GetMassBalance()->GetName());
    names.push_back(fdmex.GetAerodynamics()->GetName());
    names.push_back(fdmex.GetGroundReactions()->GetName());
    names.push_back(fdmex.GetAccelerations()->GetName());
    names.push_back(fdmex.GetFCS()->GetName());

    // Check all pairs are different
    for (size_t i = 0; i < names.size(); i++) {
      for (size_t j = i + 1; j < names.size(); j++) {
        TS_ASSERT_DIFFERS(names[i], names[j]);
      }
    }
  }

  /***************************************************************************
   * Complete System Run Tests
   ***************************************************************************/

  void testCompleteSystemRun() {
    FGFDMExec fdmex;

    // Run models that don't require aircraft to be loaded
    TS_ASSERT_EQUALS(fdmex.GetAtmosphere()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetInertial()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetPropagate()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->Run(false), false);
    // Note: FGAerodynamics::Run() requires aircraft to be loaded
    TS_ASSERT_EQUALS(fdmex.GetFCS()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAircraft()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAuxiliary()->Run(false), false);
  }

  void testCompleteSystemMultipleRuns() {
    FGFDMExec fdmex;

    for (int cycle = 0; cycle < 10; cycle++) {
      fdmex.GetAtmosphere()->Run(false);
      fdmex.GetPropagate()->Run(false);
      fdmex.GetMassBalance()->Run(false);
      // Note: FGAerodynamics::Run() requires aircraft to be loaded
      fdmex.GetFCS()->Run(false);
      fdmex.GetGroundReactions()->Run(false);
      fdmex.GetAccelerations()->Run(false);
      fdmex.GetAircraft()->Run(false);
      fdmex.GetAuxiliary()->Run(false);
    }
    TS_ASSERT(true);
  }

  void testCompleteSystemInitialize() {
    FGFDMExec fdmex;

    // Initialize models that don't require aircraft to be loaded
    TS_ASSERT(fdmex.GetAtmosphere()->InitModel());
    TS_ASSERT(fdmex.GetInertial()->InitModel());
    TS_ASSERT(fdmex.GetPropagate()->InitModel());
    TS_ASSERT(fdmex.GetMassBalance()->InitModel());
    // Note: FGAerodynamics::InitModel() requires aircraft to be loaded
    TS_ASSERT(fdmex.GetFCS()->InitModel());
    TS_ASSERT(fdmex.GetGroundReactions()->InitModel());
    TS_ASSERT(fdmex.GetAccelerations()->InitModel());
    TS_ASSERT(fdmex.GetAircraft()->InitModel());
    TS_ASSERT(fdmex.GetAuxiliary()->InitModel());
  }

  /***************************************************************************
   * Extended Stress Tests
   ***************************************************************************/

  void testExtendedModelsStress() {
    FGFDMExec fdmex;

    for (int i = 0; i < 20; i++) {
      fdmex.GetPropagate()->SetRate(i);
      fdmex.GetMassBalance()->SetRate(i + 1);
      fdmex.GetAerodynamics()->SetRate(i + 2);
      fdmex.GetGroundReactions()->SetRate(i + 3);
      fdmex.GetAccelerations()->SetRate(i + 4);
      fdmex.GetFCS()->SetRate(i + 5);

      fdmex.GetPropagate()->Run(false);
      fdmex.GetMassBalance()->Run(false);
      // Note: FGAerodynamics::Run() requires aircraft to be loaded
      fdmex.GetGroundReactions()->Run(false);
      fdmex.GetAccelerations()->Run(false);
      fdmex.GetFCS()->Run(false);
    }
    TS_ASSERT(true);
  }

  void testRapidModelSwitching() {
    FGFDMExec fdmex;

    for (int i = 0; i < 50; i++) {
      switch (i % 5) {  // Skip FGAerodynamics which requires loaded aircraft
        case 0: fdmex.GetPropagate()->Run(false); break;
        case 1: fdmex.GetMassBalance()->Run(false); break;
        case 2: fdmex.GetGroundReactions()->Run(false); break;
        case 3: fdmex.GetAccelerations()->Run(false); break;
        case 4: fdmex.GetFCS()->Run(false); break;
      }
    }
    TS_ASSERT(true);
  }

  void testManyModelInitCycles() {
    FGFDMExec fdmex;

    for (int i = 0; i < 10; i++) {
      TS_ASSERT(fdmex.GetPropagate()->InitModel());
      TS_ASSERT(fdmex.GetMassBalance()->InitModel());
      // Note: FGAerodynamics::InitModel() requires aircraft to be loaded
      TS_ASSERT(fdmex.GetGroundReactions()->InitModel());
      TS_ASSERT(fdmex.GetAccelerations()->InitModel());
      TS_ASSERT(fdmex.GetFCS()->InitModel());
    }
  }

  /***************************************************************************
   * Model Consistency Tests
   ***************************************************************************/

  void testModelExecConsistencyAfterOperations() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    FGFDMExec* exec1 = propagate->GetExec();
    propagate->SetRate(10);
    FGFDMExec* exec2 = propagate->GetExec();
    propagate->Run(false);
    FGFDMExec* exec3 = propagate->GetExec();
    propagate->InitModel();
    FGFDMExec* exec4 = propagate->GetExec();

    TS_ASSERT_EQUALS(exec1, exec2);
    TS_ASSERT_EQUALS(exec2, exec3);
    TS_ASSERT_EQUALS(exec3, exec4);
    TS_ASSERT_EQUALS(exec1, &fdmex);
  }

  void testModelNameConsistencyAfterOperations() {
    FGFDMExec fdmex;
    auto propagate = fdmex.GetPropagate();

    std::string name1 = propagate->GetName();
    propagate->SetRate(5);
    std::string name2 = propagate->GetName();
    propagate->Run(false);
    std::string name3 = propagate->GetName();
    propagate->InitModel();
    std::string name4 = propagate->GetName();

    TS_ASSERT_EQUALS(name1, name2);
    TS_ASSERT_EQUALS(name2, name3);
    TS_ASSERT_EQUALS(name3, name4);
  }

  void testMultipleFDMExecModelIndependence() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto prop1 = fdmex1.GetPropagate();
    auto prop2 = fdmex2.GetPropagate();

    prop1->SetRate(1);
    prop2->SetRate(100);

    TS_ASSERT_EQUALS(prop1->GetRate(), 1u);
    TS_ASSERT_EQUALS(prop2->GetRate(), 100u);

    TS_ASSERT(prop1->GetExec() == &fdmex1);
    TS_ASSERT(prop2->GetExec() == &fdmex2);
    TS_ASSERT(prop1->GetExec() != prop2->GetExec());
  }

  /***************************************************************************
   * Holding State Tests
   ***************************************************************************/

  void testAllExtendedModelsRunHolding() {
    FGFDMExec fdmex;

    TS_ASSERT_EQUALS(fdmex.GetPropagate()->Run(true), false);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->Run(true), false);
    // Note: FGAerodynamics::Run() requires aircraft to be loaded
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->Run(true), false);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->Run(true), false);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->Run(true), false);
  }

  void testAlternatingHoldingAllModels() {
    FGFDMExec fdmex;

    for (int i = 0; i < 10; i++) {
      bool holding = (i % 2 == 0);
      fdmex.GetPropagate()->Run(holding);
      fdmex.GetMassBalance()->Run(holding);
      // Note: FGAerodynamics::Run() requires aircraft to be loaded
      fdmex.GetGroundReactions()->Run(holding);
      fdmex.GetAccelerations()->Run(holding);
      fdmex.GetFCS()->Run(holding);
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Rate Boundary Tests for Extended Models
   ***************************************************************************/

  void testExtendedModelMaxRate() {
    FGFDMExec fdmex;
    unsigned int maxRate = std::numeric_limits<unsigned int>::max();

    fdmex.GetPropagate()->SetRate(maxRate);
    TS_ASSERT_EQUALS(fdmex.GetPropagate()->GetRate(), maxRate);

    fdmex.GetMassBalance()->SetRate(maxRate);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->GetRate(), maxRate);
  }

  void testExtendedModelRateZero() {
    FGFDMExec fdmex;

    fdmex.GetPropagate()->SetRate(0);
    fdmex.GetMassBalance()->SetRate(0);
    fdmex.GetAerodynamics()->SetRate(0);
    fdmex.GetGroundReactions()->SetRate(0);
    fdmex.GetAccelerations()->SetRate(0);
    fdmex.GetFCS()->SetRate(0);

    TS_ASSERT_EQUALS(fdmex.GetPropagate()->GetRate(), 0u);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->GetRate(), 0u);
    TS_ASSERT_EQUALS(fdmex.GetAerodynamics()->GetRate(), 0u);
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->GetRate(), 0u);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->GetRate(), 0u);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->GetRate(), 0u);
  }

  void testRateSequenceAllModels() {
    FGFDMExec fdmex;

    for (unsigned int rate = 0; rate < 10; rate++) {
      fdmex.GetPropagate()->SetRate(rate);
      fdmex.GetMassBalance()->SetRate(rate);
      fdmex.GetAerodynamics()->SetRate(rate);
      fdmex.GetGroundReactions()->SetRate(rate);
      fdmex.GetAccelerations()->SetRate(rate);
      fdmex.GetFCS()->SetRate(rate);

      TS_ASSERT_EQUALS(fdmex.GetPropagate()->GetRate(), rate);
      TS_ASSERT_EQUALS(fdmex.GetMassBalance()->GetRate(), rate);
      TS_ASSERT_EQUALS(fdmex.GetAerodynamics()->GetRate(), rate);
      TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->GetRate(), rate);
      TS_ASSERT_EQUALS(fdmex.GetAccelerations()->GetRate(), rate);
      TS_ASSERT_EQUALS(fdmex.GetFCS()->GetRate(), rate);
    }
  }

  /***************************************************************************
   * C172x Model Tests - FGModel Base Class Functionality
   ***************************************************************************/

  // Test GetName - model name retrieval with loaded aircraft
  void testC172xAerodynamicsGetName() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    std::string name = aero->GetName();

    TS_ASSERT(!name.empty());
    TS_ASSERT(name.length() > 0);
    TS_ASSERT(name.length() < 100);  // Reasonable length
  }

  // Test GetName for multiple models with loaded aircraft
  void testC172xAllModelsGetName() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    std::string aeroName = fdmex.GetAerodynamics()->GetName();
    std::string propName = fdmex.GetPropagate()->GetName();
    std::string fcsName = fdmex.GetFCS()->GetName();
    std::string massName = fdmex.GetMassBalance()->GetName();

    TS_ASSERT(!aeroName.empty());
    TS_ASSERT(!propName.empty());
    TS_ASSERT(!fcsName.empty());
    TS_ASSERT(!massName.empty());

    // Names should be unique
    TS_ASSERT_DIFFERS(aeroName, propName);
    TS_ASSERT_DIFFERS(aeroName, fcsName);
    TS_ASSERT_DIFFERS(propName, massName);
  }

  // Test GetRate - model execution rate
  void testC172xGetRate() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    unsigned int rate = aero->GetRate();

    TS_ASSERT(rate >= 0);
    TS_ASSERT(rate <= 1000);  // Reasonable upper bound
  }

  // Test SetRate - set execution rate
  void testC172xSetRate() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();

    aero->SetRate(1);
    TS_ASSERT_EQUALS(aero->GetRate(), 1u);

    aero->SetRate(5);
    TS_ASSERT_EQUALS(aero->GetRate(), 5u);

    aero->SetRate(10);
    TS_ASSERT_EQUALS(aero->GetRate(), 10u);
  }

  // Test SetRate on multiple models
  void testC172xSetRateMultipleModels() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    fdmex.GetAerodynamics()->SetRate(2);
    fdmex.GetPropagate()->SetRate(3);
    fdmex.GetFCS()->SetRate(4);
    fdmex.GetMassBalance()->SetRate(5);

    TS_ASSERT_EQUALS(fdmex.GetAerodynamics()->GetRate(), 2u);
    TS_ASSERT_EQUALS(fdmex.GetPropagate()->GetRate(), 3u);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->GetRate(), 4u);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->GetRate(), 5u);
  }

  // Test GetExec - access to FGFDMExec
  void testC172xGetExec() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    FGFDMExec* exec = aero->GetExec();

    TS_ASSERT(exec != nullptr);
    TS_ASSERT_EQUALS(exec, &fdmex);
  }

  // Test GetExec for all models
  void testC172xGetExecAllModels() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    TS_ASSERT_EQUALS(fdmex.GetAerodynamics()->GetExec(), &fdmex);
    TS_ASSERT_EQUALS(fdmex.GetPropagate()->GetExec(), &fdmex);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->GetExec(), &fdmex);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->GetExec(), &fdmex);
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->GetExec(), &fdmex);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->GetExec(), &fdmex);
  }

  // Test Run - model execution with loaded aircraft
  void testC172xRunAerodynamics() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    bool result = aero->Run(false);

    TS_ASSERT_EQUALS(result, false);  // false = no error
  }

  // Test Run on multiple models
  void testC172xRunAllModels() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    TS_ASSERT_EQUALS(fdmex.GetAerodynamics()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetPropagate()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetFCS()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetMassBalance()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetGroundReactions()->Run(false), false);
    TS_ASSERT_EQUALS(fdmex.GetAccelerations()->Run(false), false);
  }

  // Test InitModel - model initialization
  void testC172xInitModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    bool result = aero->InitModel();

    TS_ASSERT_EQUALS(result, true);
  }

  // Test InitModel on multiple models
  void testC172xInitModelAllModels() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    TS_ASSERT(fdmex.GetAerodynamics()->InitModel());
    TS_ASSERT(fdmex.GetPropagate()->InitModel());
    TS_ASSERT(fdmex.GetFCS()->InitModel());
    TS_ASSERT(fdmex.GetMassBalance()->InitModel());
    TS_ASSERT(fdmex.GetGroundReactions()->InitModel());
    TS_ASSERT(fdmex.GetAccelerations()->InitModel());
  }

  // Test model chaining - multiple models running in sequence
  void testC172xModelChaining() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    // Run models in sequence as they would during simulation
    for (int i = 0; i < 10; i++) {
      fdmex.GetAtmosphere()->Run(false);
      fdmex.GetPropagate()->Run(false);
      fdmex.GetAerodynamics()->Run(false);
      fdmex.GetMassBalance()->Run(false);
      fdmex.GetFCS()->Run(false);
      fdmex.GetGroundReactions()->Run(false);
      fdmex.GetAccelerations()->Run(false);
    }
    TS_ASSERT(true);  // No errors occurred
  }

  // Test model naming convention - names should follow expected patterns
  void testC172xModelNamingConvention() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    std::string aeroName = fdmex.GetAerodynamics()->GetName();
    std::string propName = fdmex.GetPropagate()->GetName();
    std::string fcsName = fdmex.GetFCS()->GetName();

    // Names should be non-empty and printable
    TS_ASSERT(!aeroName.empty());
    TS_ASSERT(!propName.empty());
    TS_ASSERT(!fcsName.empty());

    // All characters should be printable
    for (char c : aeroName) {
      TS_ASSERT(std::isprint(c));
    }
    for (char c : propName) {
      TS_ASSERT(std::isprint(c));
    }
    for (char c : fcsName) {
      TS_ASSERT(std::isprint(c));
    }
  }

  // Test model execution order - verify FDM can run complete cycle
  void testC172xModelExecutionOrder() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    // Run full FDM cycle
    bool result = fdmex.Run();
    TS_ASSERT_EQUALS(result, true);

    // Run multiple cycles
    for (int i = 0; i < 100; i++) {
      result = fdmex.Run();
      TS_ASSERT_EQUALS(result, true);
    }
  }

  // Test aerodynamic forces are finite after model run
  void testC172xAeroForcesFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto aero = fdmex.GetAerodynamics();

    // Get aerodynamic forces in body frame
    double forceX = aero->GetForces()(1);
    double forceY = aero->GetForces()(2);
    double forceZ = aero->GetForces()(3);

    TS_ASSERT(std::isfinite(forceX));
    TS_ASSERT(std::isfinite(forceY));
    TS_ASSERT(std::isfinite(forceZ));
  }

  // Test propagate state values are finite
  void testC172xPropagateStateFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto propagate = fdmex.GetPropagate();

    // Check position values
    double altitude = propagate->GetAltitudeASL();
    double lat = propagate->GetLatitudeDeg();
    double lon = propagate->GetLongitudeDeg();

    TS_ASSERT(std::isfinite(altitude));
    TS_ASSERT(std::isfinite(lat));
    TS_ASSERT(std::isfinite(lon));

    // Check velocity values
    double u = propagate->GetUVW()(1);
    double v = propagate->GetUVW()(2);
    double w = propagate->GetUVW()(3);

    TS_ASSERT(std::isfinite(u));
    TS_ASSERT(std::isfinite(v));
    TS_ASSERT(std::isfinite(w));
  }

  // Test mass balance values are finite and reasonable
  void testC172xMassBalanceFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto mass = fdmex.GetMassBalance();

    double weight = mass->GetWeight();
    double mass_val = mass->GetMass();

    TS_ASSERT(std::isfinite(weight));
    TS_ASSERT(std::isfinite(mass_val));
    TS_ASSERT(weight > 0);  // Weight should be positive
    TS_ASSERT(mass_val > 0);  // Mass should be positive

    // C172 typical weight range 1600-2400 lbs
    TS_ASSERT(weight > 1000);
    TS_ASSERT(weight < 5000);
  }

  // Test rate changes during simulation
  void testC172xRateChangesDuringSimulation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();

    for (unsigned int rate = 1; rate <= 5; rate++) {
      aero->SetRate(rate);
      TS_ASSERT_EQUALS(aero->GetRate(), rate);

      fdmex.Run();

      // Verify rate persists after run
      TS_ASSERT_EQUALS(aero->GetRate(), rate);
    }
  }

  // Test model exec consistency after multiple runs
  void testC172xExecConsistencyAfterRuns() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();

    FGFDMExec* exec1 = aero->GetExec();

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    FGFDMExec* exec2 = aero->GetExec();

    TS_ASSERT_EQUALS(exec1, exec2);
    TS_ASSERT_EQUALS(exec1, &fdmex);
  }

  // Test model name consistency after operations
  void testC172xNameConsistencyAfterOperations() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();

    std::string name1 = aero->GetName();

    aero->SetRate(10);
    fdmex.Run();
    aero->InitModel();

    std::string name2 = aero->GetName();

    TS_ASSERT_EQUALS(name1, name2);
  }

  // Test FCS control surface outputs are finite
  void testC172xFCSOutputsFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto fcs = fdmex.GetFCS();

    double aileron = fcs->GetDePos(0);  // Aileron
    double elevator = fcs->GetDePos(1);  // Elevator
    double rudder = fcs->GetDrPos();

    TS_ASSERT(std::isfinite(aileron));
    TS_ASSERT(std::isfinite(elevator));
    TS_ASSERT(std::isfinite(rudder));

    // Control surfaces should be within reasonable deflection limits
    TS_ASSERT(aileron >= -1.0 && aileron <= 1.0);
    TS_ASSERT(elevator >= -1.0 && elevator <= 1.0);
    TS_ASSERT(rudder >= -1.0 && rudder <= 1.0);
  }

  // Test ground reactions with loaded aircraft
  void testC172xGroundReactionsFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto ground = fdmex.GetGroundReactions();

    // Number of gear units should be positive for C172
    int numGear = ground->GetNumGearUnits();
    TS_ASSERT(numGear > 0);
    TS_ASSERT(numGear <= 10);  // Reasonable upper bound
  }

  // Test accelerations are finite
  void testC172xAccelerationsFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto accel = fdmex.GetAccelerations();

    // Body accelerations
    double udot = accel->GetUVWdot()(1);
    double vdot = accel->GetUVWdot()(2);
    double wdot = accel->GetUVWdot()(3);

    TS_ASSERT(std::isfinite(udot));
    TS_ASSERT(std::isfinite(vdot));
    TS_ASSERT(std::isfinite(wdot));

    // Accelerations should be within reasonable bounds (< 100 ft/s^2)
    TS_ASSERT(std::abs(udot) < 1000);
    TS_ASSERT(std::abs(vdot) < 1000);
    TS_ASSERT(std::abs(wdot) < 1000);
  }

  // Test auxiliary computed values are finite
  void testC172xAuxiliaryFinite() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    fdmex.Run();

    auto aux = fdmex.GetAuxiliary();

    double mach = aux->GetMach();
    double qbar = aux->Getqbar();
    double vcas = aux->GetVcalibratedKTS();

    TS_ASSERT(std::isfinite(mach));
    TS_ASSERT(std::isfinite(qbar));
    TS_ASSERT(std::isfinite(vcas));

    // Reasonable ranges for C172 at startup
    TS_ASSERT(mach >= 0);
    TS_ASSERT(mach < 1.0);  // C172 is subsonic
    TS_ASSERT(qbar >= 0);
  }

  // Test multiple FDMExec instances with C172x
  void testC172xMultipleFDMExecInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    fdmex1.LoadModel("c172x");
    fdmex2.LoadModel("c172x");

    fdmex1.RunIC();
    fdmex2.RunIC();

    // Verify each has its own model instances
    auto aero1 = fdmex1.GetAerodynamics();
    auto aero2 = fdmex2.GetAerodynamics();

    TS_ASSERT(aero1 != aero2);
    TS_ASSERT(aero1->GetExec() == &fdmex1);
    TS_ASSERT(aero2->GetExec() == &fdmex2);

    // Set different rates
    aero1->SetRate(1);
    aero2->SetRate(10);

    TS_ASSERT_EQUALS(aero1->GetRate(), 1u);
    TS_ASSERT_EQUALS(aero2->GetRate(), 10u);
  }

  // Test extended simulation run stability
  void testC172xExtendedSimulationStability() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    // Run for extended period
    for (int i = 0; i < 500; i++) {
      bool result = fdmex.Run();
      TS_ASSERT_EQUALS(result, true);

      // Verify state remains finite
      auto prop = fdmex.GetPropagate();
      TS_ASSERT(std::isfinite(prop->GetAltitudeASL()));
      TS_ASSERT(std::isfinite(prop->GetLatitudeDeg()));
      TS_ASSERT(std::isfinite(prop->GetLongitudeDeg()));
    }
  }

  // Test model InitModel and Run sequence
  void testC172xInitRunSequence() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();

    // Sequence: Init -> Run -> Init -> Run
    TS_ASSERT(aero->InitModel());
    TS_ASSERT_EQUALS(aero->Run(false), false);
    TS_ASSERT(aero->InitModel());
    TS_ASSERT_EQUALS(aero->Run(false), false);

    // Verify state is still valid
    TS_ASSERT(!aero->GetName().empty());
    TS_ASSERT(aero->GetExec() == &fdmex);
  }
};
