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
   * Run Model Tests
   ***************************************************************************/

  void testRunNotHolding() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    // Run returns true when no gas cells are defined (early return)
    bool result = buoyant->Run(false);
    TS_ASSERT_EQUALS(result, true);
  }

  void testRunHolding() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    // When holding, Run returns false
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
      TS_ASSERT_EQUALS(result, true);  // Returns true when no gas cells defined
    }
  }

  void testRunAfterInit() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    buoyant->InitModel();
    bool result = buoyant->Run(false);
    TS_ASSERT_EQUALS(result, true);  // Returns true when no gas cells defined
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

  /***************************************************************************
   * Additional Atmospheric Condition Tests
   ***************************************************************************/

  void testGasCellInputsVeryHighAltitude() {
    // Very high altitude (~60,000 ft)
    FGGasCell::Inputs inputs;
    inputs.Pressure = 151.03;
    inputs.Temperature = 389.97;
    inputs.Density = 0.000224;
    inputs.gravity = 32.15;

    TS_ASSERT(inputs.Pressure > 0.0);
    TS_ASSERT(inputs.Temperature > 0.0);
    TS_ASSERT(inputs.Density > 0.0);
  }

  void testGasCellInputsHotDay() {
    // Hot day conditions at sea level
    FGGasCell::Inputs inputs;
    inputs.Pressure = 2116.22;
    inputs.Temperature = 558.67;  // 99°F
    inputs.Density = 0.00225;     // Slightly less dense
    inputs.gravity = 32.174;

    TS_ASSERT(inputs.Temperature > 518.67);  // Hotter than standard
    TS_ASSERT(inputs.Density < 0.002377);    // Less dense
  }

  void testGasCellInputsColdDay() {
    // Cold day conditions at sea level
    FGGasCell::Inputs inputs;
    inputs.Pressure = 2116.22;
    inputs.Temperature = 459.67;  // 0°F
    inputs.Density = 0.00268;     // Denser
    inputs.gravity = 32.174;

    TS_ASSERT(inputs.Temperature < 518.67);  // Colder than standard
    TS_ASSERT(inputs.Density > 0.002377);    // More dense
  }

  void testGasCellInputsLowPressure() {
    // Low pressure (mountain altitude)
    FGGasCell::Inputs inputs;
    inputs.Pressure = 1455.63;    // ~10,000 ft
    inputs.Temperature = 483.03;  // ~10,000 ft
    inputs.Density = 0.001756;
    inputs.gravity = 32.17;

    TS_ASSERT(inputs.Pressure < 2116.22);
  }

  /***************************************************************************
   * Buoyancy Strings/Values Tests
   ***************************************************************************/

  void testGetBuoyancyStringsComma() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    std::string strings = buoyant->GetBuoyancyStrings(",");
    // Without gas cells, should be empty
    TS_ASSERT(strings.empty());
  }

  void testGetBuoyancyStringsTab() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    std::string strings = buoyant->GetBuoyancyStrings("\t");
    TS_ASSERT(strings.empty());
  }

  void testGetBuoyancyValuesComma() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    std::string values = buoyant->GetBuoyancyValues(",");
    TS_ASSERT(values.empty());
  }

  void testGetBuoyancyValuesTab() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    std::string values = buoyant->GetBuoyancyValues("\t");
    TS_ASSERT(values.empty());
  }

  void testGetBuoyancyStringsSemicolon() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    std::string strings = buoyant->GetBuoyancyStrings(";");
    TS_ASSERT(strings.empty());
  }

  /***************************************************************************
   * Matrix Properties Tests
   ***************************************************************************/

  void testInertiaMatrixSymmetry() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGMatrix33& inertia = buoyant->GetGasMassInertia();

    // Inertia matrix should be symmetric
    TS_ASSERT_DELTA(inertia(1,2), inertia(2,1), epsilon);
    TS_ASSERT_DELTA(inertia(1,3), inertia(3,1), epsilon);
    TS_ASSERT_DELTA(inertia(2,3), inertia(3,2), epsilon);
  }

  void testInertiaMatrixDiagonalsNonNegative() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGMatrix33& inertia = buoyant->GetGasMassInertia();

    // Diagonal elements should be non-negative
    TS_ASSERT(inertia(1,1) >= 0.0);
    TS_ASSERT(inertia(2,2) >= 0.0);
    TS_ASSERT(inertia(3,3) >= 0.0);
  }

  void testInertiaMatrixNotNaN() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGMatrix33& inertia = buoyant->GetGasMassInertia();

    for (int i = 1; i <= 3; i++) {
      for (int j = 1; j <= 3; j++) {
        TS_ASSERT(!std::isnan(inertia(i, j)));
        TS_ASSERT(!std::isinf(inertia(i, j)));
      }
    }
  }

  /***************************************************************************
   * Multiple Instance Tests
   ***************************************************************************/

  void testMultipleFDMExecBuoyantForces() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto buoyant1 = fdmex1.GetBuoyantForces();
    auto buoyant2 = fdmex2.GetBuoyantForces();

    TS_ASSERT(buoyant1 != nullptr);
    TS_ASSERT(buoyant2 != nullptr);
    TS_ASSERT(buoyant1 != buoyant2);
  }

  void testMultipleInstancesIndependentForces() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto buoyant1 = fdmex1.GetBuoyantForces();
    auto buoyant2 = fdmex2.GetBuoyantForces();

    buoyant1->Run(false);
    buoyant2->Run(true);

    // Both should have zero forces without gas cells
    TS_ASSERT_DELTA(buoyant1->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(buoyant2->GetForces(1), 0.0, epsilon);
  }

  void testThreeInstances() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;
    FGFDMExec fdmex3;

    auto b1 = fdmex1.GetBuoyantForces();
    auto b2 = fdmex2.GetBuoyantForces();
    auto b3 = fdmex3.GetBuoyantForces();

    TS_ASSERT(b1 != nullptr);
    TS_ASSERT(b2 != nullptr);
    TS_ASSERT(b3 != nullptr);
    TS_ASSERT(b1 != b2);
    TS_ASSERT(b2 != b3);
    TS_ASSERT(b1 != b3);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testManyRuns() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 100; i++) {
      bool result = buoyant->Run(false);
      TS_ASSERT_EQUALS(result, true);
    }
  }

  void testAlternatingHolding() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 50; i++) {
      bool holding = (i % 2 == 0);
      buoyant->Run(holding);
    }
    TS_ASSERT(true);
  }

  void testManyInitCalls() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 50; i++) {
      bool result = buoyant->InitModel();
      TS_ASSERT_EQUALS(result, true);
    }
  }

  void testInterleavedOperations() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 20; i++) {
      buoyant->InitModel();
      buoyant->Run(false);
      buoyant->GetForces();
      buoyant->GetMoments();
      buoyant->GetGasMass();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Gas Cell Inputs Copy/Assignment Tests
   ***************************************************************************/

  void testInputsCopyConstruction() {
    FGGasCell::Inputs inputs1;
    inputs1.Pressure = 2116.22;
    inputs1.Temperature = 518.67;
    inputs1.Density = 0.002377;
    inputs1.gravity = 32.174;

    FGGasCell::Inputs inputs2 = inputs1;

    TS_ASSERT_DELTA(inputs2.Pressure, inputs1.Pressure, epsilon);
    TS_ASSERT_DELTA(inputs2.Temperature, inputs1.Temperature, epsilon);
    TS_ASSERT_DELTA(inputs2.Density, inputs1.Density, epsilon);
    TS_ASSERT_DELTA(inputs2.gravity, inputs1.gravity, epsilon);
  }

  void testInputsAssignment() {
    FGGasCell::Inputs inputs1;
    inputs1.Pressure = 2116.22;
    inputs1.Temperature = 518.67;
    inputs1.Density = 0.002377;
    inputs1.gravity = 32.174;

    FGGasCell::Inputs inputs2;
    inputs2 = inputs1;

    TS_ASSERT_DELTA(inputs2.Pressure, inputs1.Pressure, epsilon);
    TS_ASSERT_DELTA(inputs2.Temperature, inputs1.Temperature, epsilon);
  }

  void testInputsModificationIndependence() {
    FGGasCell::Inputs inputs1;
    inputs1.Pressure = 2116.22;

    FGGasCell::Inputs inputs2 = inputs1;
    inputs2.Pressure = 1000.0;

    TS_ASSERT_DELTA(inputs1.Pressure, 2116.22, epsilon);
    TS_ASSERT_DELTA(inputs2.Pressure, 1000.0, epsilon);
  }

  /***************************************************************************
   * Moments Consistency Tests
   ***************************************************************************/

  void testMomentsConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    buoyant->Run(false);
    const FGColumnVector3& moments1 = buoyant->GetMoments();

    buoyant->Run(false);
    const FGColumnVector3& moments2 = buoyant->GetMoments();

    TS_ASSERT_DELTA(moments1(1), moments2(1), epsilon);
    TS_ASSERT_DELTA(moments1(2), moments2(2), epsilon);
    TS_ASSERT_DELTA(moments1(3), moments2(3), epsilon);
  }

  void testGasMassConsistentAcrossRuns() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    buoyant->Run(false);
    double mass1 = buoyant->GetGasMass();

    buoyant->Run(false);
    double mass2 = buoyant->GetGasMass();

    TS_ASSERT_DELTA(mass1, mass2, epsilon);
  }

  void testGasMassMomentConsistent() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    buoyant->Run(false);
    const FGColumnVector3& moment1 = buoyant->GetGasMassMoment();

    buoyant->Run(false);
    const FGColumnVector3& moment2 = buoyant->GetGasMassMoment();

    TS_ASSERT_DELTA(moment1(1), moment2(1), epsilon);
    TS_ASSERT_DELTA(moment1(2), moment2(2), epsilon);
    TS_ASSERT_DELTA(moment1(3), moment2(3), epsilon);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testGasCellInputsExtremePressure() {
    FGGasCell::Inputs inputs;
    inputs.Pressure = 1e10;  // Very high pressure
    TS_ASSERT_DELTA(inputs.Pressure, 1e10, 1.0);

    inputs.Pressure = 1e-10;  // Very low pressure
    TS_ASSERT_DELTA(inputs.Pressure, 1e-10, 1e-15);
  }

  void testGasCellInputsExtremeTemperature() {
    FGGasCell::Inputs inputs;
    inputs.Temperature = 10000.0;  // Very high temperature
    TS_ASSERT_DELTA(inputs.Temperature, 10000.0, 0.1);

    inputs.Temperature = 1.0;  // Very low temperature (near absolute zero)
    TS_ASSERT_DELTA(inputs.Temperature, 1.0, epsilon);
  }

  void testGasCellInputsExtremeDensity() {
    FGGasCell::Inputs inputs;
    inputs.Density = 1.0;  // Extremely high density
    TS_ASSERT_DELTA(inputs.Density, 1.0, epsilon);

    inputs.Density = 1e-10;  // Very low density
    TS_ASSERT_DELTA(inputs.Density, 1e-10, 1e-15);
  }

  void testGasCellInputsExtremeGravity() {
    FGGasCell::Inputs inputs;
    inputs.gravity = 100.0;  // High gravity (like Jupiter-like)
    TS_ASSERT_DELTA(inputs.gravity, 100.0, epsilon);

    inputs.gravity = 0.001;  // Very low gravity (like asteroid)
    TS_ASSERT_DELTA(inputs.gravity, 0.001, epsilon);
  }

  /***************************************************************************
   * Vector Access Tests
   ***************************************************************************/

  void testForcesVectorIndexAccess() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    double fx = buoyant->GetForces(1);
    double fy = buoyant->GetForces(2);
    double fz = buoyant->GetForces(3);

    TS_ASSERT(!std::isnan(fx));
    TS_ASSERT(!std::isnan(fy));
    TS_ASSERT(!std::isnan(fz));
  }

  void testMomentsVectorIndexAccess() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    double mx = buoyant->GetMoments(1);
    double my = buoyant->GetMoments(2);
    double mz = buoyant->GetMoments(3);

    TS_ASSERT(!std::isnan(mx));
    TS_ASSERT(!std::isnan(my));
    TS_ASSERT(!std::isnan(mz));
  }

  void testForcesConsistencyBetweenAccessMethods() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGColumnVector3& forces = buoyant->GetForces();

    TS_ASSERT_DELTA(forces(1), buoyant->GetForces(1), epsilon);
    TS_ASSERT_DELTA(forces(2), buoyant->GetForces(2), epsilon);
    TS_ASSERT_DELTA(forces(3), buoyant->GetForces(3), epsilon);
  }

  void testMomentsConsistencyBetweenAccessMethods() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    const FGColumnVector3& moments = buoyant->GetMoments();

    TS_ASSERT_DELTA(moments(1), buoyant->GetMoments(1), epsilon);
    TS_ASSERT_DELTA(moments(2), buoyant->GetMoments(2), epsilon);
    TS_ASSERT_DELTA(moments(3), buoyant->GetMoments(3), epsilon);
  }

  /***************************************************************************
   * Combined Operation Tests
   ***************************************************************************/

  void testFullOperationSequence() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    // Full operation sequence
    TS_ASSERT(buoyant->InitModel());
    buoyant->Run(false);

    const FGColumnVector3& forces = buoyant->GetForces();
    const FGColumnVector3& moments = buoyant->GetMoments();
    double mass = buoyant->GetGasMass();
    const FGColumnVector3& massMoment = buoyant->GetGasMassMoment();
    const FGMatrix33& inertia = buoyant->GetGasMassInertia();
    std::string strings = buoyant->GetBuoyancyStrings(",");
    std::string values = buoyant->GetBuoyancyValues(",");

    // All should be valid (zero without gas cells)
    TS_ASSERT_DELTA(forces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(moments(1), 0.0, epsilon);
    TS_ASSERT_DELTA(mass, 0.0, epsilon);
    TS_ASSERT_DELTA(massMoment(1), 0.0, epsilon);
    TS_ASSERT_DELTA(inertia(1,1), 0.0, epsilon);
    TS_ASSERT(strings.empty());
    TS_ASSERT(values.empty());
  }

  void testRapidPropertyAccess() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 100; i++) {
      buoyant->GetForces();
      buoyant->GetMoments();
      buoyant->GetGasMass();
      buoyant->GetGasMassMoment();
      buoyant->GetGasMassInertia();
    }
    TS_ASSERT(true);
  }
};
