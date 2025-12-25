/*******************************************************************************
 * FGGasCellTest.h - Unit tests for Gas Cell
 *
 * Tests the FGGasCell class including:
 * - Gas cell constants and types
 * - Input structure validation
 *
 * Note: FGGasCell requires XML configuration to construct, so these tests
 * focus on verifying the interface contracts and constants.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>

#include <FGFDMExec.h>
#include <models/FGBuoyantForces.h>
#include <math/FGColumnVector3.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGGasCellTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Buoyant Forces Model Tests (parent of gas cells)
   ***************************************************************************/

  void testBuoyantForcesConstruction() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    TS_ASSERT(buoyant != nullptr);
  }

  void testBuoyantForcesInitModel() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    bool result = buoyant->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  /***************************************************************************
   * Buoyant Forces Vector Tests
   ***************************************************************************/

  void testGetForcesVector() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGColumnVector3& forces = buoyant->GetForces();

    TS_ASSERT(!std::isnan(forces(1)));
    TS_ASSERT(!std::isnan(forces(2)));
    TS_ASSERT(!std::isnan(forces(3)));
  }

  void testGetForcesInitiallyZero() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    // Without gas cells, forces should be zero
    TS_ASSERT_DELTA(buoyant->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(buoyant->GetForces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(buoyant->GetForces(3), 0.0, epsilon);
  }

  void testGetMomentsVector() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGColumnVector3& moments = buoyant->GetMoments();

    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  void testGetMomentsInitiallyZero() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    TS_ASSERT_DELTA(buoyant->GetMoments(1), 0.0, epsilon);
    TS_ASSERT_DELTA(buoyant->GetMoments(2), 0.0, epsilon);
    TS_ASSERT_DELTA(buoyant->GetMoments(3), 0.0, epsilon);
  }

  /***************************************************************************
   * Gas Cell Count Tests
   ***************************************************************************/

  void testNoGasCellsInitially() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    TS_ASSERT_EQUALS(buoyant->GetNumGasCells(), 0u);
  }

  /***************************************************************************
   * Run Model Tests
   ***************************************************************************/

  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    bool result = buoyant->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  void testRunHolding() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    bool result = buoyant->Run(true);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Mass and Inertia Tests
   ***************************************************************************/

  void testGetGasMassInitiallyZero() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    double mass = buoyant->GetGasMass();
    TS_ASSERT_DELTA(mass, 0.0, epsilon);
  }

  void testGetGasMassMomentInitiallyZero() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGColumnVector3& moment = buoyant->GetGasMassMoment();

    TS_ASSERT_DELTA(moment(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(2), 0.0, epsilon);
    TS_ASSERT_DELTA(moment(3), 0.0, epsilon);
  }

  void testGetGasMassInertiaInitiallyZero() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGMatrix33& inertia = buoyant->GetGasMassInertia();

    TS_ASSERT_DELTA(inertia(1,1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(2,2), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(3,3), 0.0, epsilon);
  }

  /***************************************************************************
   * Multiple Run Tests
   ***************************************************************************/

  void testMultipleRuns() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 10; i++) {
      bool result = buoyant->Run(false);
      TS_ASSERT_EQUALS(result, false);
    }
  }

  void testRunAfterInit() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    buoyant->InitModel();
    bool result = buoyant->Run(false);
    TS_ASSERT_EQUALS(result, false);
  }

  /***************************************************************************
   * Force Consistency Tests
   ***************************************************************************/

  void testForcesConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    buoyant->Run(false);
    const FGColumnVector3& forces1 = buoyant->GetForces();

    buoyant->Run(false);
    const FGColumnVector3& forces2 = buoyant->GetForces();

    TS_ASSERT_DELTA(forces1(1), forces2(1), epsilon);
    TS_ASSERT_DELTA(forces1(2), forces2(2), epsilon);
    TS_ASSERT_DELTA(forces1(3), forces2(3), epsilon);
  }

  /***************************************************************************
   * Gas Cell Input Structure Tests
   ***************************************************************************/

  void testGasCellInputsStructure() {
    // Test that the Inputs structure has expected fields
    FGGasCell::Inputs inputs;
    inputs.Pressure = 2116.22;    // Standard sea level pressure (psf)
    inputs.Temperature = 518.67;  // Standard temperature (Rankine)
    inputs.Density = 0.002377;    // Standard density (slug/ft^3)
    inputs.gravity = 32.174;      // Standard gravity (ft/s^2)

    TS_ASSERT_DELTA(inputs.Pressure, 2116.22, 0.01);
    TS_ASSERT_DELTA(inputs.Temperature, 518.67, 0.01);
    TS_ASSERT_DELTA(inputs.Density, 0.002377, 1e-6);
    TS_ASSERT_DELTA(inputs.gravity, 32.174, 0.001);
  }

  void testGasCellInputsZeroValues() {
    FGGasCell::Inputs inputs;
    inputs.Pressure = 0.0;
    inputs.Temperature = 0.0;
    inputs.Density = 0.0;
    inputs.gravity = 0.0;

    TS_ASSERT_DELTA(inputs.Pressure, 0.0, epsilon);
    TS_ASSERT_DELTA(inputs.Temperature, 0.0, epsilon);
    TS_ASSERT_DELTA(inputs.Density, 0.0, epsilon);
    TS_ASSERT_DELTA(inputs.gravity, 0.0, epsilon);
  }

  void testGasCellInputsNegativeGravity() {
    // Some configurations might use negative gravity orientation
    FGGasCell::Inputs inputs;
    inputs.gravity = -32.174;

    TS_ASSERT_DELTA(inputs.gravity, -32.174, 0.001);
  }

  void testGasCellInputsHighAltitude() {
    // High altitude atmospheric conditions
    FGGasCell::Inputs inputs;
    inputs.Pressure = 472.68;      // ~35,000 ft pressure
    inputs.Temperature = 394.06;   // ~35,000 ft temperature
    inputs.Density = 0.000738;     // ~35,000 ft density
    inputs.gravity = 32.17;        // Slightly reduced gravity

    TS_ASSERT(inputs.Pressure > 0.0);
    TS_ASSERT(inputs.Temperature > 0.0);
    TS_ASSERT(inputs.Density > 0.0);
    TS_ASSERT(inputs.gravity > 0.0);
  }
};
