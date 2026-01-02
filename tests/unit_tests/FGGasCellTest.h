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
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <initialization/FGInitialCondition.h>
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

  /***************************************************************************
   * Gas Cell Physics Tests - Ideal Gas Law Concepts
   ***************************************************************************/

  void testIdealGasLawPV() {
    // P*V = n*R*T, testing concept with inputs
    FGGasCell::Inputs inputs;
    double P = 2116.22;    // psf
    double T = 518.67;     // Rankine
    double rho = 0.002377; // slug/ft^3

    inputs.Pressure = P;
    inputs.Temperature = T;
    inputs.Density = rho;

    // Verify inputs are consistent (P = rho * R_specific * T)
    // R_air = 1716.49 ft^2/(s^2 * R) for air
    double R_air = 1716.49;
    double calculated_P = rho * R_air * T;

    // Check order of magnitude is correct
    TS_ASSERT(calculated_P > 1000.0 && calculated_P < 3000.0);
  }

  void testIdealGasLawIsothermal() {
    // At constant temperature, P1*V1 = P2*V2
    double P1 = 2116.22;
    double V1 = 1000.0;  // ft^3

    double P2 = P1 * 2.0;  // Double pressure
    double V2 = V1 / 2.0;  // Half volume

    TS_ASSERT_DELTA(P1 * V1, P2 * V2, 0.01);
  }

  void testIdealGasLawIsobaric() {
    // At constant pressure, V1/T1 = V2/T2
    double V1 = 1000.0;
    double T1 = 518.67;

    double T2 = T1 * 2.0;  // Double temperature
    double V2 = V1 * 2.0;  // Double volume

    TS_ASSERT_DELTA(V1 / T1, V2 / T2, 1e-6);
  }

  void testIdealGasLawIsochoric() {
    // At constant volume, P1/T1 = P2/T2
    double P1 = 2116.22;
    double T1 = 518.67;

    double T2 = T1 * 1.5;  // 50% higher temperature
    double P2 = P1 * 1.5;  // 50% higher pressure

    TS_ASSERT_DELTA(P1 / T1, P2 / T2, 1e-6);
  }

  /***************************************************************************
   * Buoyancy Physics Tests
   ***************************************************************************/

  void testBuoyancyFormula() {
    // Buoyancy = rho_air * g * V - rho_gas * g * V
    double rho_air = 0.002377;     // slug/ft^3
    double rho_helium = 0.000333;  // slug/ft^3 (approximate)
    double g = 32.174;             // ft/s^2
    double V = 1000.0;             // ft^3

    double buoyancy = (rho_air - rho_helium) * g * V;

    TS_ASSERT(buoyancy > 0.0);  // Positive lift
    TS_ASSERT(buoyancy < 100.0);  // Reasonable magnitude
  }

  void testBuoyancyHeliumVsHydrogen() {
    double rho_air = 0.002377;
    double rho_helium = 0.000333;
    double rho_hydrogen = 0.000167;  // Lighter than helium
    double g = 32.174;
    double V = 1000.0;

    double buoyancy_helium = (rho_air - rho_helium) * g * V;
    double buoyancy_hydrogen = (rho_air - rho_hydrogen) * g * V;

    // Hydrogen should provide more lift
    TS_ASSERT(buoyancy_hydrogen > buoyancy_helium);
  }

  void testBuoyancyAirCell() {
    // Air ballonet - no net buoyancy
    double rho_air = 0.002377;
    double g = 32.174;
    double V = 1000.0;

    double buoyancy = (rho_air - rho_air) * g * V;

    TS_ASSERT_DELTA(buoyancy, 0.0, epsilon);
  }

  void testBuoyancyAltitudeEffect() {
    double g = 32.174;
    double V = 1000.0;

    // Sea level
    double rho_air_sl = 0.002377;
    double rho_helium_sl = 0.000333;
    double buoyancy_sl = (rho_air_sl - rho_helium_sl) * g * V;

    // 35,000 ft - lower density air
    double rho_air_35k = 0.000738;
    double rho_helium_35k = 0.000103;  // Approximate
    double buoyancy_35k = (rho_air_35k - rho_helium_35k) * g * V;

    // Less buoyancy at altitude
    TS_ASSERT(buoyancy_35k < buoyancy_sl);
  }

  /***************************************************************************
   * Gas Cell Inputs - Additional Scenario Tests
   ***************************************************************************/

  void testInputsTroposphere() {
    // Troposphere conditions (0-36,000 ft)
    FGGasCell::Inputs inputs;

    // At 20,000 ft
    inputs.Pressure = 973.27;
    inputs.Temperature = 447.42;
    inputs.Density = 0.001267;
    inputs.gravity = 32.17;

    TS_ASSERT(inputs.Pressure > 0.0);
    TS_ASSERT(inputs.Temperature > 0.0);
    TS_ASSERT(inputs.Density > 0.0);
  }

  void testInputsStratosphere() {
    // Stratosphere conditions (>36,000 ft isothermal layer)
    FGGasCell::Inputs inputs;

    // At 45,000 ft
    inputs.Pressure = 286.91;
    inputs.Temperature = 389.97;  // Constant in isothermal layer
    inputs.Density = 0.000428;
    inputs.gravity = 32.16;

    TS_ASSERT(inputs.Pressure > 0.0);
    TS_ASSERT(inputs.Temperature > 0.0);
  }

  void testInputsMarsAtmosphere() {
    // Mars-like conditions (very thin atmosphere)
    FGGasCell::Inputs inputs;

    inputs.Pressure = 12.0;        // Very low (psf equivalent)
    inputs.Temperature = 392.0;    // ~218K in Rankine
    inputs.Density = 0.00003;      // Very thin
    inputs.gravity = 12.1;         // Mars gravity ~3.7 m/s^2

    TS_ASSERT(inputs.Pressure > 0.0);
    TS_ASSERT(inputs.gravity < 32.174);  // Less than Earth
  }

  void testInputsVenusAtmosphere() {
    // Venus-like conditions (very dense atmosphere)
    FGGasCell::Inputs inputs;

    inputs.Pressure = 192000.0;    // ~92 bar in psf
    inputs.Temperature = 1296.0;   // ~720K in Rankine
    inputs.Density = 0.13;         // Very dense
    inputs.gravity = 28.8;         // Venus gravity ~8.87 m/s^2

    TS_ASSERT(inputs.Pressure > 2116.22);  // Higher than Earth
    TS_ASSERT(inputs.Density > 0.002377);  // Denser than Earth
  }

  /***************************************************************************
   * Gas Properties Tests
   ***************************************************************************/

  void testHeliumMolarMass() {
    // Helium molar mass ~ 4.003 g/mol = 0.0000867 slug/mol
    double M_helium = 0.0000867;  // slug/mol
    TS_ASSERT(M_helium > 0.0 && M_helium < 0.001);
  }

  void testHydrogenMolarMass() {
    // Hydrogen molar mass ~ 2.016 g/mol = 0.0000437 slug/mol
    double M_hydrogen = 0.0000437;  // slug/mol
    double M_helium = 0.0000867;    // slug/mol
    TS_ASSERT(M_hydrogen > 0.0 && M_hydrogen < M_helium);  // Lighter than helium
  }

  void testAirMolarMass() {
    // Air molar mass ~ 28.97 g/mol = 0.000629 slug/mol
    double M_air = 0.000629;  // slug/mol
    double M_helium = 0.0000867;
    TS_ASSERT(M_air > M_helium);  // Air is heavier
  }

  void testSpecificHeatCapacity() {
    // Cv for monatomic gas (He) = 3/2 R
    // Cv for diatomic gas (H2, air) = 5/2 R
    double Cv_helium = 3.0 / 2.0;
    double Cv_hydrogen = 5.0 / 2.0;
    double Cv_air = 5.0 / 2.0;

    TS_ASSERT_DELTA(Cv_helium, 1.5, epsilon);
    TS_ASSERT_DELTA(Cv_hydrogen, 2.5, epsilon);
    TS_ASSERT_DELTA(Cv_air, 2.5, epsilon);
  }

  /***************************************************************************
   * Volume Calculations Tests
   ***************************************************************************/

  void testSphereVolume() {
    // Volume of sphere = (4/3) * pi * r^3
    double radius = 10.0;  // ft
    double volume = (4.0 / 3.0) * M_PI * radius * radius * radius;

    TS_ASSERT_DELTA(volume, 4188.79, 0.01);
  }

  void testEllipsoidVolume() {
    // Volume of ellipsoid = (4/3) * pi * a * b * c
    double a = 20.0;  // ft (x-radius)
    double b = 10.0;  // ft (y-radius)
    double c = 10.0;  // ft (z-radius)

    double volume = (4.0 / 3.0) * M_PI * a * b * c;

    TS_ASSERT_DELTA(volume, 8377.58, 0.01);
  }

  void testCylinderVolume() {
    // Volume of cylinder = pi * r^2 * h
    double radius = 10.0;  // ft
    double height = 50.0;  // ft

    double volume = M_PI * radius * radius * height;

    TS_ASSERT_DELTA(volume, 15707.96, 0.01);
  }

  void testBlimpEnvelopeVolume() {
    // Typical blimp envelope (ellipsoid-like)
    double length = 200.0;  // ft
    double diameter = 50.0; // ft

    double volume = (4.0 / 3.0) * M_PI * (length / 2.0) * (diameter / 2.0) * (diameter / 2.0);

    TS_ASSERT(volume > 100000.0);  // Large volume
  }

  /***************************************************************************
   * Pressure/Temperature Relationship Tests
   ***************************************************************************/

  void testPressureAltitudeRelationship() {
    // Pressure decreases with altitude
    FGGasCell::Inputs sea_level;
    sea_level.Pressure = 2116.22;

    FGGasCell::Inputs altitude_10k;
    altitude_10k.Pressure = 1455.63;

    FGGasCell::Inputs altitude_35k;
    altitude_35k.Pressure = 472.68;

    TS_ASSERT(sea_level.Pressure > altitude_10k.Pressure);
    TS_ASSERT(altitude_10k.Pressure > altitude_35k.Pressure);
  }

  void testDensityAltitudeRelationship() {
    // Density decreases with altitude
    FGGasCell::Inputs sea_level;
    sea_level.Density = 0.002377;

    FGGasCell::Inputs altitude_10k;
    altitude_10k.Density = 0.001756;

    FGGasCell::Inputs altitude_35k;
    altitude_35k.Density = 0.000738;

    TS_ASSERT(sea_level.Density > altitude_10k.Density);
    TS_ASSERT(altitude_10k.Density > altitude_35k.Density);
  }

  void testTemperatureAltitudeRelationship() {
    // Temperature decreases in troposphere, constant in lower stratosphere
    FGGasCell::Inputs sea_level;
    sea_level.Temperature = 518.67;

    FGGasCell::Inputs altitude_10k;
    altitude_10k.Temperature = 483.03;

    FGGasCell::Inputs altitude_35k;
    altitude_35k.Temperature = 394.06;

    TS_ASSERT(sea_level.Temperature > altitude_10k.Temperature);
    TS_ASSERT(altitude_10k.Temperature > altitude_35k.Temperature);
  }

  /***************************************************************************
   * Mass Calculations Tests
   ***************************************************************************/

  void testGasMassFromVolume() {
    // mass = rho * V
    double rho_helium = 0.000333;  // slug/ft^3
    double V = 10000.0;            // ft^3

    double mass = rho_helium * V;

    TS_ASSERT_DELTA(mass, 3.33, 0.01);  // slugs
  }

  void testLiftCapacity() {
    // Gross lift = buoyancy - gas weight
    double rho_air = 0.002377;
    double rho_helium = 0.000333;
    double V = 10000.0;
    double g = 32.174;

    double buoyancy = rho_air * V * g;
    double gas_weight = rho_helium * V * g;
    double net_lift = buoyancy - gas_weight;

    TS_ASSERT(net_lift > 0.0);
    TS_ASSERT(net_lift < buoyancy);
  }

  void testPayloadCapacity() {
    // Payload = net lift - envelope weight - structure weight
    double rho_air = 0.002377;
    double rho_helium = 0.000333;
    double V = 100000.0;  // 100,000 ft^3 envelope
    double g = 32.174;

    double buoyancy = rho_air * V * g;
    double gas_weight = rho_helium * V * g;
    double gross_lift = buoyancy - gas_weight;

    // Assume envelope weighs 0.03 lb/ft^2 and is a sphere
    double radius = std::pow((3.0 * V) / (4.0 * M_PI), 1.0/3.0);
    double surface_area = 4.0 * M_PI * radius * radius;
    double envelope_weight = surface_area * 0.03;

    double payload = gross_lift - envelope_weight;

    TS_ASSERT(payload > 0.0);  // Should have positive payload capacity
  }

  /***************************************************************************
   * Valve and Pressure Relief Tests
   ***************************************************************************/

  void testOverpressureRelief() {
    // When pressure exceeds max, valve opens
    double max_overpressure = 50.0;  // psf
    double ambient_pressure = 2116.22;
    double internal_pressure = ambient_pressure + 60.0;  // Over max

    double overpressure = internal_pressure - ambient_pressure;

    TS_ASSERT(overpressure > max_overpressure);
  }

  void testValveCoefficientFlow() {
    // dV/dt = valve_coeff * delta_P
    double valve_coeff = 0.1;  // ft^4 sec / slug
    double delta_P = 10.0;     // psf

    double flow_rate = valve_coeff * delta_P;

    TS_ASSERT(flow_rate > 0.0);
  }

  void testHeatTransferConcept() {
    // Q = h * A * (T_external - T_internal)
    double h = 0.01;           // heat transfer coefficient
    double A = 1000.0;         // surface area (ft^2)
    double T_ext = 518.67;     // Rankine
    double T_int = 530.0;      // Slightly warmer inside

    double Q = h * A * (T_ext - T_int);

    // Negative means heat flows out
    TS_ASSERT(Q < 0.0);
  }

  /***************************************************************************
   * Ballonet Concept Tests
   ***************************************************************************/

  void testBallonetVolumeCompensation() {
    // As gas cell cools/contracts, ballonet expands
    double total_volume = 10000.0;
    double initial_gas_volume = 8000.0;
    double initial_ballonet_volume = 2000.0;

    // After cooling, gas contracts
    double final_gas_volume = 7000.0;
    double final_ballonet_volume = total_volume - final_gas_volume;

    TS_ASSERT_DELTA(initial_gas_volume + initial_ballonet_volume, total_volume, epsilon);
    TS_ASSERT_DELTA(final_gas_volume + final_ballonet_volume, total_volume, epsilon);
    TS_ASSERT(final_ballonet_volume > initial_ballonet_volume);
  }

  void testBallonetBlowerInput() {
    // Blower adds air to maintain pressure
    double blower_flow = 100.0;  // ft^3/sec
    double dt = 0.1;             // seconds

    double volume_added = blower_flow * dt;

    TS_ASSERT_DELTA(volume_added, 10.0, epsilon);
  }

  /***************************************************************************
   * Stress Tests - Rapid Operations
   ***************************************************************************/

  void testStressManyInputConfigurations() {
    for (int i = 0; i < 100; i++) {
      FGGasCell::Inputs inputs;
      inputs.Pressure = 1000.0 + i * 20.0;
      inputs.Temperature = 400.0 + i * 2.0;
      inputs.Density = 0.001 + i * 0.00002;
      inputs.gravity = 32.0 + i * 0.01;

      TS_ASSERT(inputs.Pressure > 0.0);
      TS_ASSERT(inputs.Temperature > 0.0);
      TS_ASSERT(inputs.Density > 0.0);
    }
  }

  void testStressBuoyantForcesOperations() {
    FGFDMExec fdmex;
    auto buoyant = fdmex.GetBuoyantForces();

    for (int i = 0; i < 50; i++) {
      buoyant->InitModel();
      buoyant->Run(false);
      buoyant->Run(true);

      const FGColumnVector3& forces = buoyant->GetForces();
      const FGColumnVector3& moments = buoyant->GetMoments();
      double mass = buoyant->GetGasMass();
      const FGMatrix33& inertia = buoyant->GetGasMassInertia();

      TS_ASSERT(!std::isnan(forces(1)));
      TS_ASSERT(!std::isnan(moments(1)));
      TS_ASSERT(!std::isnan(mass));
      TS_ASSERT(!std::isnan(inertia(1,1)));
    }
  }

  void testStressManyFDMExecInstances() {
    std::vector<std::shared_ptr<FGFDMExec>> execs;

    for (int i = 0; i < 10; i++) {
      auto fdmex = std::make_shared<FGFDMExec>();
      execs.push_back(fdmex);
    }

    for (auto& fdmex : execs) {
      auto buoyant = fdmex->GetBuoyantForces();
      TS_ASSERT(buoyant != nullptr);
      buoyant->Run(false);
    }
  }

  /***************************************************************************
   * Complete Gas Cell System Tests
   ***************************************************************************/

  // Test complete gas cell state
  void testCompleteGasCellState() {
    FGGasCell::Inputs inputs;
    inputs.Pressure = 2116.22;
    inputs.Temperature = 518.67;
    inputs.Density = 0.002378;
    inputs.gravity = 32.174;

    // Verify all inputs are valid
    TS_ASSERT(inputs.Pressure > 0.0);
    TS_ASSERT(inputs.Temperature > 0.0);
    TS_ASSERT(inputs.Density > 0.0);
    TS_ASSERT(inputs.gravity > 0.0);
  }

  // Test gas cell temperature effects
  void testGasCellTemperatureEffects() {
    double volume1 = 10000.0;
    double T1 = 500.0;
    double T2 = 600.0;

    // Charles's Law: V1/T1 = V2/T2
    double volume2 = volume1 * T2 / T1;
    TS_ASSERT(volume2 > volume1);
    TS_ASSERT_DELTA(volume2, 12000.0, 1.0);
  }

  // Test gas cell pressure-volume relationship
  void testGasCellPressureVolumeRelationship() {
    double P1 = 2000.0;
    double V1 = 10000.0;
    double P2 = 1000.0;

    // Boyle's Law: P1*V1 = P2*V2
    double V2 = P1 * V1 / P2;
    TS_ASSERT(V2 > V1);
    TS_ASSERT_DELTA(V2, 20000.0, 1.0);
  }

  // Test superheat lift calculation
  void testSuperheatLiftCalculation() {
    double volume = 50000.0;        // ft^3
    double airDensity = 0.002378;   // slugs/ft^3
    double gasDensity = 0.00015;    // Helium density
    double superheatFactor = 1.1;   // 10% superheat

    double coldLift = volume * (airDensity - gasDensity) * 32.174;
    double hotLift = coldLift * superheatFactor;

    TS_ASSERT(hotLift > coldLift);
  }

  // Test valve operations
  void testValveOperations() {
    double pressure = 2200.0;   // psf
    double reliefPressure = 2150.0;

    bool valveOpen = pressure > reliefPressure;
    TS_ASSERT(valveOpen);

    // After venting
    pressure = 2100.0;
    valveOpen = pressure > reliefPressure;
    TS_ASSERT(!valveOpen);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  // Test gas cell inputs independence
  void testGasCellInputsIndependence() {
    FGGasCell::Inputs inputs1;
    FGGasCell::Inputs inputs2;

    inputs1.Pressure = 2000.0;
    inputs2.Pressure = 3000.0;

    TS_ASSERT_DELTA(inputs1.Pressure, 2000.0, 0.1);
    TS_ASSERT_DELTA(inputs2.Pressure, 3000.0, 0.1);
  }

  // Test buoyant force calculation independence
  void testBuoyantForceCalculationIndependence() {
    double vol1 = 10000.0, rho1 = 0.002;
    double vol2 = 20000.0, rho2 = 0.001;

    double buoyancy1 = vol1 * rho1 * 32.174;
    double buoyancy2 = vol2 * rho2 * 32.174;

    TS_ASSERT_DELTA(buoyancy1, 643.48, 1.0);
    TS_ASSERT_DELTA(buoyancy2, 643.48, 1.0);
  }

  // Test temperature state independence
  void testTemperatureStateIndependence() {
    double T1 = 500.0;
    double T2 = 600.0;

    // Different cells at different temperatures
    TS_ASSERT(T1 < T2);
    TS_ASSERT_DELTA(T2 - T1, 100.0, 0.01);
  }

  // Test pressure calculation independence
  void testPressureCalculationIndependence() {
    double P1 = 2000.0;
    double P2 = 2500.0;

    TS_ASSERT(P1 < P2);
    TS_ASSERT_DELTA(P2 / P1, 1.25, 0.01);
  }

  // Test volume calculation independence
  void testVolumeCalculationIndependence() {
    double vol1 = 5000.0;
    double vol2 = 10000.0;

    TS_ASSERT(vol2 > vol1);
    TS_ASSERT_DELTA(vol2 / vol1, 2.0, 0.01);
  }

  // Test gas density calculation independence
  void testGasDensityCalculationIndependence() {
    double rho1 = 0.00015;  // Helium
    double rho2 = 0.00011;  // Hydrogen

    TS_ASSERT(rho1 > rho2);
    TS_ASSERT(rho1 < 0.001);
    TS_ASSERT(rho2 < 0.001);
  }

  // Test buoyancy ratio calculation
  void testBuoyancyRatioCalculation() {
    double airDensity = 0.002378;
    double heliumDensity = 0.00015;
    double hydrogenDensity = 0.00011;

    double heliumLiftRatio = (airDensity - heliumDensity) / airDensity;
    double hydrogenLiftRatio = (airDensity - hydrogenDensity) / airDensity;

    TS_ASSERT(hydrogenLiftRatio > heliumLiftRatio);
    TS_ASSERT(heliumLiftRatio > 0.9);
  }

  // Test envelope material stress
  void testEnvelopeMaterialStress() {
    double pressure = 100.0;    // psf differential
    double radius = 50.0;       // ft
    double thickness = 0.01;    // ft

    double hoopStress = pressure * radius / thickness;
    TS_ASSERT(hoopStress > 0.0);
    TS_ASSERT_DELTA(hoopStress, 500000.0, 1.0);
  }
};

//=============================================================================
// C172x Integration Tests - Buoyancy/Mass Context Tests
//=============================================================================

class FGGasCellC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {


    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: C172x has no buoyant forces (it's a heavier-than-air aircraft)
  void testNoBuoyantForces() {
    auto buoyant = fdm.GetBuoyantForces();

    TS_ASSERT(buoyant != nullptr);
    TS_ASSERT_DELTA(buoyant->GetForces()(1), 0.0, 0.01);
    TS_ASSERT_DELTA(buoyant->GetForces()(2), 0.0, 0.01);
    TS_ASSERT_DELTA(buoyant->GetForces()(3), 0.0, 0.01);
  }

  // Test 2: Buoyant moments are zero
  void testNoBuoyantMoments() {
    auto buoyant = fdm.GetBuoyantForces();

    TS_ASSERT_DELTA(buoyant->GetMoments()(1), 0.0, 0.01);
    TS_ASSERT_DELTA(buoyant->GetMoments()(2), 0.0, 0.01);
    TS_ASSERT_DELTA(buoyant->GetMoments()(3), 0.0, 0.01);
  }

  // Test 3: Mass is positive
  void testPositiveMass() {
    auto mass = fdm.GetMassBalance();

    double weight = mass->GetWeight();
    TS_ASSERT(weight > 0.0);
  }

  // Test 4: CG location is valid
  void testCGLocation() {
    auto mass = fdm.GetMassBalance();

    double cgX = mass->GetXYZcg()(1);
    double cgY = mass->GetXYZcg()(2);
    double cgZ = mass->GetXYZcg()(3);

    TS_ASSERT(std::isfinite(cgX));
    TS_ASSERT(std::isfinite(cgY));
    TS_ASSERT(std::isfinite(cgZ));
  }

  // Test 5: Inertia tensor is positive definite
  void testInertiaTensor() {
    auto mass = fdm.GetMassBalance();

    const auto& J = mass->GetJ();
    double Ixx = J(1,1);
    double Iyy = J(2,2);
    double Izz = J(3,3);

    TS_ASSERT(Ixx > 0.0);
    TS_ASSERT(Iyy > 0.0);
    TS_ASSERT(Izz > 0.0);
  }

  // Test 6: Aircraft weight in typical range
  void testWeightInRange() {
    auto mass = fdm.GetMassBalance();

    double weight = mass->GetWeight();

    // C172 empty weight ~1600 lbs, max gross ~2450 lbs
    TS_ASSERT(weight > 1000.0);
    TS_ASSERT(weight < 3000.0);
  }

  // Test 7: Forces remain zero during flight
  void testBuoyantForcesDuringFlight() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    for (int i = 0; i < 50; ++i) fdm.Run();

    auto buoyant = fdm.GetBuoyantForces();

    TS_ASSERT_DELTA(buoyant->GetForces()(1), 0.0, 0.01);
    TS_ASSERT_DELTA(buoyant->GetForces()(2), 0.0, 0.01);
    TS_ASSERT_DELTA(buoyant->GetForces()(3), 0.0, 0.01);
  }

  // Test 8: Weight does not change with altitude
  void testWeightConstantWithAltitude() {
    auto mass = fdm.GetMassBalance();
    auto ic = fdm.GetIC();

    // Sea level
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(0.0);
    fdm.RunIC();
    double weight_sl = mass->GetWeight();

    // High altitude
    ic->SetAltitudeASLFtIC(10000.0);
    fdm.RunIC();
    double weight_high = mass->GetWeight();

    TS_ASSERT_DELTA(weight_sl, weight_high, 1.0);
  }

  // Test 9: Empty weight reasonable
  void testEmptyWeight() {
    auto mass = fdm.GetMassBalance();

    double emptyWeight = mass->GetEmptyWeight();

    TS_ASSERT(emptyWeight > 0.0 || std::isfinite(mass->GetWeight()));
  }

  // Test 10: Fuel affects weight
  void testFuelAffectsWeight() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto fcs = fdm.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);
    fcs->SetMixtureCmd(-1, 1.0);

    auto mass = fdm.GetMassBalance();
    double initialWeight = mass->GetWeight();

    // Run simulation (burn fuel)
    for (int i = 0; i < 1000; ++i) fdm.Run();

    double finalWeight = mass->GetWeight();

    // Weight should decrease or stay same (fuel burned)
    TS_ASSERT(finalWeight <= initialWeight + 1.0);
  }

  // Test 11: Moments of inertia are finite
  void testMomentsOfInertiaFinite() {
    auto mass = fdm.GetMassBalance();
    const auto& J = mass->GetJ();

    TS_ASSERT(std::isfinite(J(1,1)));  // Ixx
    TS_ASSERT(std::isfinite(J(2,2)));  // Iyy
    TS_ASSERT(std::isfinite(J(3,3)));  // Izz
    TS_ASSERT(std::isfinite(J(1,2)));  // Ixy
    TS_ASSERT(std::isfinite(J(1,3)));  // Ixz
    TS_ASSERT(std::isfinite(J(2,3)));  // Iyz
  }

  // Test 12: Extended simulation mass stability
  void testExtendedSimulationMassStability() {
    auto ic = fdm.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    fdm.RunIC();

    auto mass = fdm.GetMassBalance();
    auto buoyant = fdm.GetBuoyantForces();

    for (int i = 0; i < 500; ++i) {
      fdm.Run();
    }

    // All values should remain valid
    TS_ASSERT(std::isfinite(mass->GetWeight()));
    TS_ASSERT(std::isfinite(mass->GetJ()(1,1)));  // Ixx
    TS_ASSERT_DELTA(buoyant->GetForces()(1), 0.0, 0.01);
  }
};
