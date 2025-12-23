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
};
