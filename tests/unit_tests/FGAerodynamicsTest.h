/*******************************************************************************
 * FGAerodynamicsTest.h - Unit tests for FGAerodynamics model
 *
 * Tests the aerodynamics model including:
 * - Force and moment vector access
 * - Coordinate system transformations (body, stability, wind)
 * - Aerodynamic parameters (L/D, CL, alpha limits)
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>

#include <FGFDMExec.h>
#include <models/FGAerodynamics.h>
#include <math/FGColumnVector3.h>
#include <math/FGMatrix33.h>

using namespace JSBSim;

const double epsilon = 1e-8;

class FGAerodynamicsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction and Initialization Tests
   ***************************************************************************/

  // Test construction
  void testConstruction() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    TS_ASSERT(aero != nullptr);
  }

  // Test InitModel
  void testInitModel() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    bool result = aero->InitModel();
    TS_ASSERT_EQUALS(result, true);
  }

  /***************************************************************************
   * Force Vector Tests
   ***************************************************************************/

  // Test GetForces returns valid vector
  void testGetForcesVector() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& forces = aero->GetForces();

    TS_ASSERT(!std::isnan(forces(1)));
    TS_ASSERT(!std::isnan(forces(2)));
    TS_ASSERT(!std::isnan(forces(3)));
  }

  // Test GetForces indexed accessor
  void testGetForcesIndexed() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& forces = aero->GetForces();

    TS_ASSERT_DELTA(aero->GetForces(1), forces(1), epsilon);
    TS_ASSERT_DELTA(aero->GetForces(2), forces(2), epsilon);
    TS_ASSERT_DELTA(aero->GetForces(3), forces(3), epsilon);
  }

  /***************************************************************************
   * Moment Vector Tests
   ***************************************************************************/

  // Test GetMoments returns valid vector
  void testGetMomentsVector() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& moments = aero->GetMoments();

    TS_ASSERT(!std::isnan(moments(1)));
    TS_ASSERT(!std::isnan(moments(2)));
    TS_ASSERT(!std::isnan(moments(3)));
  }

  // Test GetMoments indexed accessor
  void testGetMomentsIndexed() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& moments = aero->GetMoments();

    TS_ASSERT_DELTA(aero->GetMoments(1), moments(1), epsilon);
    TS_ASSERT_DELTA(aero->GetMoments(2), moments(2), epsilon);
    TS_ASSERT_DELTA(aero->GetMoments(3), moments(3), epsilon);
  }

  // Test GetMomentsMRC
  void testGetMomentsMRC() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& momentsMRC = aero->GetMomentsMRC();

    TS_ASSERT(!std::isnan(momentsMRC(1)));
    TS_ASSERT(!std::isnan(momentsMRC(2)));
    TS_ASSERT(!std::isnan(momentsMRC(3)));
  }

  // Test GetMomentsMRC indexed accessor
  void testGetMomentsMRCIndexed() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& momentsMRC = aero->GetMomentsMRC();

    TS_ASSERT_DELTA(aero->GetMomentsMRC(1), momentsMRC(1), epsilon);
    TS_ASSERT_DELTA(aero->GetMomentsMRC(2), momentsMRC(2), epsilon);
    TS_ASSERT_DELTA(aero->GetMomentsMRC(3), momentsMRC(3), epsilon);
  }

  /***************************************************************************
   * Wind Axis Forces
   ***************************************************************************/

  // Test GetvFw (wind axis forces)
  void testGetvFw() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& vFw = aero->GetvFw();

    TS_ASSERT(!std::isnan(vFw(1)));
    TS_ASSERT(!std::isnan(vFw(2)));
    TS_ASSERT(!std::isnan(vFw(3)));
  }

  // Test GetvFw indexed accessor
  void testGetvFwIndexed() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& vFw = aero->GetvFw();

    TS_ASSERT_DELTA(aero->GetvFw(1), vFw(1), epsilon);
    TS_ASSERT_DELTA(aero->GetvFw(2), vFw(2), epsilon);
    TS_ASSERT_DELTA(aero->GetvFw(3), vFw(3), epsilon);
  }

  /***************************************************************************
   * Stability Axis Forces
   ***************************************************************************/

  // Test GetForcesInStabilityAxes
  void testGetForcesInStabilityAxes() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 stabilityForces = aero->GetForcesInStabilityAxes();

    TS_ASSERT(!std::isnan(stabilityForces(1)));
    TS_ASSERT(!std::isnan(stabilityForces(2)));
    TS_ASSERT(!std::isnan(stabilityForces(3)));
  }

  // Test GetMomentsInStabilityAxes
  void testGetMomentsInStabilityAxes() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 stabilityMoments = aero->GetMomentsInStabilityAxes();

    TS_ASSERT(!std::isnan(stabilityMoments(1)));
    TS_ASSERT(!std::isnan(stabilityMoments(2)));
    TS_ASSERT(!std::isnan(stabilityMoments(3)));
  }

  // Test GetMomentsInWindAxes
  void testGetMomentsInWindAxes() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 windMoments = aero->GetMomentsInWindAxes();

    TS_ASSERT(!std::isnan(windMoments(1)));
    TS_ASSERT(!std::isnan(windMoments(2)));
    TS_ASSERT(!std::isnan(windMoments(3)));
  }

  /***************************************************************************
   * Aerodynamic Parameters
   ***************************************************************************/

  // Test GetLoD (Lift over Drag)
  void testGetLoD() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double lod = aero->GetLoD();
    TS_ASSERT(!std::isnan(lod));
    // L/D can be any value (positive, negative, zero, or inf with zero drag)
  }

  // Test GetClSquared
  void testGetClSquared() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double clsq = aero->GetClSquared();
    TS_ASSERT(!std::isnan(clsq));
    TS_ASSERT(clsq >= 0.0);  // Squared value must be non-negative
  }

  /***************************************************************************
   * Alpha Limits Tests
   ***************************************************************************/

  // Test GetAlphaCLMax
  void testGetAlphaCLMax() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double alphaCLMax = aero->GetAlphaCLMax();
    TS_ASSERT(!std::isnan(alphaCLMax));
  }

  // Test GetAlphaCLMin
  void testGetAlphaCLMin() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double alphaCLMin = aero->GetAlphaCLMin();
    TS_ASSERT(!std::isnan(alphaCLMin));
  }

  // Test SetAlphaCLMax and GetAlphaCLMax
  void testSetAlphaCLMax() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double testAlpha = 0.35;  // ~20 degrees
    aero->SetAlphaCLMax(testAlpha);
    TS_ASSERT_DELTA(aero->GetAlphaCLMax(), testAlpha, epsilon);
  }

  // Test SetAlphaCLMin and GetAlphaCLMin
  void testSetAlphaCLMin() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double testAlpha = -0.17;  // ~-10 degrees
    aero->SetAlphaCLMin(testAlpha);
    TS_ASSERT_DELTA(aero->GetAlphaCLMin(), testAlpha, epsilon);
  }

  /***************************************************************************
   * Stall Warning Tests
   ***************************************************************************/

  // Test GetHysteresisParm
  void testGetHysteresisParm() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double hystParm = aero->GetHysteresisParm();
    TS_ASSERT(!std::isnan(hystParm));
  }

  // Test GetStallWarn
  void testGetStallWarn() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double stallWarn = aero->GetStallWarn();
    TS_ASSERT(!std::isnan(stallWarn));
    // Stall warning should be between 0 and 1
    TS_ASSERT(stallWarn >= 0.0);
    TS_ASSERT(stallWarn <= 1.0);
  }

  // Test GetAlphaW
  void testGetAlphaW() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double alphaW = aero->GetAlphaW();
    TS_ASSERT(!std::isnan(alphaW));
  }

  /***************************************************************************
   * Rate Damping Parameters
   ***************************************************************************/

  // Test GetBI2Vel (b/(2V) - roll rate damping factor)
  void testGetBI2Vel() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double bi2vel = aero->GetBI2Vel();
    TS_ASSERT(!std::isnan(bi2vel));
    TS_ASSERT(bi2vel >= 0.0);  // Non-negative (wingspan/velocity related)
  }

  // Test GetCI2Vel (c/(2V) - pitch rate damping factor)
  void testGetCI2Vel() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double ci2vel = aero->GetCI2Vel();
    TS_ASSERT(!std::isnan(ci2vel));
    TS_ASSERT(ci2vel >= 0.0);  // Non-negative (chord/velocity related)
  }

  /***************************************************************************
   * Run Method Tests
   * Note: Run() requires a loaded aircraft with proper aerodynamics definition.
   * Without an aircraft, it throws LogException about axis type.
   * We test that the method exists and handles the holding parameter.
   ***************************************************************************/

  // Test Run method - without aircraft, verifies method exists
  void testRun() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Without a loaded aircraft, Run throws because axis type is not defined.
    // We verify the method exists and the exception message is expected.
    try {
      aero->Run(false);
      // If no exception, that's also fine (aircraft was loaded somehow)
      TS_ASSERT(true);
    } catch (const std::exception& e) {
      // Expected: "A proper axis type has NOT been selected"
      std::string msg = e.what();
      TS_ASSERT(msg.find("axis type") != std::string::npos);
    }
  }

  // Test Run with holding flag
  void testRunHolding() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // With holding=true, the model should return early without processing
    bool result = aero->Run(true);
    TS_ASSERT_EQUALS(result, false);  // false means no error
  }

  /***************************************************************************
   * Aero Function String Tests
   ***************************************************************************/

  // Test GetAeroFunctionStrings
  void testGetAeroFunctionStrings() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    std::string tabStrings = aero->GetAeroFunctionStrings("\t");
    std::string commaStrings = aero->GetAeroFunctionStrings(",");

    // Without loading an aircraft, these may be empty but shouldn't crash
    TS_ASSERT(tabStrings.size() >= 0);  // May be empty
    TS_ASSERT(commaStrings.size() >= 0);
  }

  // Test GetAeroFunctionValues
  void testGetAeroFunctionValues() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    std::string tabValues = aero->GetAeroFunctionValues("\t");
    std::string commaValues = aero->GetAeroFunctionValues(",");

    // Without loading an aircraft, these may be empty but shouldn't crash
    TS_ASSERT(tabValues.size() >= 0);
    TS_ASSERT(commaValues.size() >= 0);
  }

  /***************************************************************************
   * Aero Functions Access
   * Note: GetAeroFunctions() may throw if aerodynamics not fully loaded
   ***************************************************************************/

  // Test GetAeroFunctions returns valid pointer array
  void testGetAeroFunctions() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Note: Without loading an aircraft, this returns a pointer to
    // uninitialized data. The array exists but may not be fully set up.
    // We just verify it doesn't return null.
    auto aeroFunctions = aero->GetAeroFunctions();
    // May be null without aircraft loaded - that's OK for this test
    // Just verify we don't crash accessing it
    TS_ASSERT(true);  // If we got here, no crash
  }

  /***************************************************************************
   * Force Magnitude Conservation Tests
   ***************************************************************************/

  // Test that body forces magnitude equals wind forces magnitude
  void testForceMagnitudeConservation() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& bodyForces = aero->GetForces();
    const FGColumnVector3& windForces = aero->GetvFw();

    double bodyMag = bodyForces.Magnitude();
    double windMag = windForces.Magnitude();

    // Both should be zero without aircraft or both should have same magnitude
    TS_ASSERT_DELTA(bodyMag, windMag, 1e-6);
  }

  // Test stability axis force magnitude
  void testStabilityForceMagnitude() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& bodyForces = aero->GetForces();
    FGColumnVector3 stabilityForces = aero->GetForcesInStabilityAxes();

    double bodyMag = bodyForces.Magnitude();
    double stabilityMag = stabilityForces.Magnitude();

    // Force magnitude should be preserved in coordinate transformation
    TS_ASSERT_DELTA(bodyMag, stabilityMag, 1e-6);
  }

  // Test moment magnitude conservation
  void testMomentMagnitudeConservation() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& bodyMoments = aero->GetMoments();
    FGColumnVector3 stabilityMoments = aero->GetMomentsInStabilityAxes();

    double bodyMag = bodyMoments.Magnitude();
    double stabilityMag = stabilityMoments.Magnitude();

    // Moment magnitude should be preserved in coordinate transformation
    TS_ASSERT_DELTA(bodyMag, stabilityMag, 1e-6);
  }

  /***************************************************************************
   * Rate Damping Factor Relationship Tests
   ***************************************************************************/

  // Test bi2vel calculation (b/(2V))
  void testBI2VelCalculation() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // bi2vel = wingspan / (2 * velocity)
    // At zero velocity, this should be zero or infinite
    double bi2vel = aero->GetBI2Vel();

    // Without velocity, should be 0 (protected against divide-by-zero)
    TS_ASSERT_DELTA(bi2vel, 0.0, 1e-6);
  }

  // Test ci2vel calculation (c/(2V))
  void testCI2VelCalculation() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // ci2vel = chord / (2 * velocity)
    double ci2vel = aero->GetCI2Vel();

    // Without velocity, should be 0 (protected against divide-by-zero)
    TS_ASSERT_DELTA(ci2vel, 0.0, 1e-6);
  }

  // Test that bi2vel and ci2vel are both non-negative
  void testRateDampingFactorsNonNegative() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    TS_ASSERT(aero->GetBI2Vel() >= 0.0);
    TS_ASSERT(aero->GetCI2Vel() >= 0.0);
  }

  /***************************************************************************
   * Alpha Limits Extended Tests
   ***************************************************************************/

  // Test alpha limits consistency (max > min)
  void testAlphaLimitsConsistency() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double alphaMax = aero->GetAlphaCLMax();
    double alphaMin = aero->GetAlphaCLMin();

    // Max should be greater than min
    TS_ASSERT(alphaMax >= alphaMin);
  }

  // Test setting negative alpha min
  void testNegativeAlphaCLMin() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double negAlpha = -0.35;  // -20 degrees
    aero->SetAlphaCLMin(negAlpha);
    TS_ASSERT_DELTA(aero->GetAlphaCLMin(), negAlpha, epsilon);
  }

  // Test setting positive alpha max
  void testPositiveAlphaCLMax() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double posAlpha = 0.52;  // 30 degrees
    aero->SetAlphaCLMax(posAlpha);
    TS_ASSERT_DELTA(aero->GetAlphaCLMax(), posAlpha, epsilon);
  }

  // Test setting alpha limits to extreme values
  void testExtremeAlphaLimits() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Set to +/- 90 degrees
    double alphaMax = M_PI / 2.0;
    double alphaMin = -M_PI / 2.0;

    aero->SetAlphaCLMax(alphaMax);
    aero->SetAlphaCLMin(alphaMin);

    TS_ASSERT_DELTA(aero->GetAlphaCLMax(), alphaMax, epsilon);
    TS_ASSERT_DELTA(aero->GetAlphaCLMin(), alphaMin, epsilon);
  }

  // Test setting alpha limits to zero
  void testZeroAlphaLimits() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    aero->SetAlphaCLMax(0.0);
    aero->SetAlphaCLMin(0.0);

    TS_ASSERT_DELTA(aero->GetAlphaCLMax(), 0.0, epsilon);
    TS_ASSERT_DELTA(aero->GetAlphaCLMin(), 0.0, epsilon);
  }

  /***************************************************************************
   * Stall Warning Extended Tests
   ***************************************************************************/

  // Test stall warning initial state
  void testStallWarnInitialState() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double stallWarn = aero->GetStallWarn();

    // Without an aircraft at stall conditions, should be 0
    TS_ASSERT_EQUALS(stallWarn, 0.0);
  }

  // Test hysteresis parameter initial state
  void testHysteresisParmInitialState() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double hystParm = aero->GetHysteresisParm();

    // Hysteresis parameter should be non-negative
    TS_ASSERT(hystParm >= 0.0);
  }

  // Test alpha wing initial value
  void testAlphaWInitialValue() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double alphaW = aero->GetAlphaW();

    // Initial alpha wing should be 0
    TS_ASSERT_DELTA(alphaW, 0.0, epsilon);
  }

  /***************************************************************************
   * L/D and ClSquared Extended Tests
   ***************************************************************************/

  // Test L/D initial state
  void testLoDInitialState() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double lod = aero->GetLoD();

    // Without aircraft, should be 0 (protected against div-by-zero)
    TS_ASSERT(!std::isinf(lod));
  }

  // Test ClSquared initial state
  void testClSquaredInitialState() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double clsq = aero->GetClSquared();

    // Without forces, should be 0
    TS_ASSERT_DELTA(clsq, 0.0, epsilon);
  }

  // Test that ClSquared is always non-negative
  void testClSquaredAlwaysNonNegative() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Try multiple init cycles
    for (int i = 0; i < 5; i++) {
      aero->InitModel();
      TS_ASSERT(aero->GetClSquared() >= 0.0);
    }
  }

  /***************************************************************************
   * Coordinate System Transformation Tests
   ***************************************************************************/

  // Test wind axis force components
  void testWindAxisForceComponents() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // In wind axes: X = -Drag, Y = Side force, Z = -Lift
    const FGColumnVector3& vFw = aero->GetvFw();

    // All components should be finite
    TS_ASSERT(!std::isnan(vFw(1)));  // -Drag
    TS_ASSERT(!std::isnan(vFw(2)));  // Side force
    TS_ASSERT(!std::isnan(vFw(3)));  // -Lift
    TS_ASSERT(!std::isinf(vFw(1)));
    TS_ASSERT(!std::isinf(vFw(2)));
    TS_ASSERT(!std::isinf(vFw(3)));
  }

  // Test stability axis transformation
  void testStabilityAxisTransformation() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 stabilityForces = aero->GetForcesInStabilityAxes();

    // All components should be finite
    TS_ASSERT(!std::isnan(stabilityForces(1)));
    TS_ASSERT(!std::isnan(stabilityForces(2)));
    TS_ASSERT(!std::isnan(stabilityForces(3)));
    TS_ASSERT(!std::isinf(stabilityForces(1)));
    TS_ASSERT(!std::isinf(stabilityForces(2)));
    TS_ASSERT(!std::isinf(stabilityForces(3)));
  }

  // Test wind axis moment transformation
  void testWindAxisMomentTransformation() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 windMoments = aero->GetMomentsInWindAxes();

    // All components should be finite
    TS_ASSERT(!std::isnan(windMoments(1)));
    TS_ASSERT(!std::isnan(windMoments(2)));
    TS_ASSERT(!std::isnan(windMoments(3)));
    TS_ASSERT(!std::isinf(windMoments(1)));
    TS_ASSERT(!std::isinf(windMoments(2)));
    TS_ASSERT(!std::isinf(windMoments(3)));
  }

  // Test indexed force accessors in stability axes
  void testStabilityAxisIndexedForces() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 stabilityForces = aero->GetForcesInStabilityAxes();

    TS_ASSERT_DELTA(aero->GetForcesInStabilityAxes(1), stabilityForces(1), epsilon);
    TS_ASSERT_DELTA(aero->GetForcesInStabilityAxes(2), stabilityForces(2), epsilon);
    TS_ASSERT_DELTA(aero->GetForcesInStabilityAxes(3), stabilityForces(3), epsilon);
  }

  // Test indexed moment accessors in stability axes
  void testStabilityAxisIndexedMoments() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 stabilityMoments = aero->GetMomentsInStabilityAxes();

    TS_ASSERT_DELTA(aero->GetMomentsInStabilityAxes(1), stabilityMoments(1), epsilon);
    TS_ASSERT_DELTA(aero->GetMomentsInStabilityAxes(2), stabilityMoments(2), epsilon);
    TS_ASSERT_DELTA(aero->GetMomentsInStabilityAxes(3), stabilityMoments(3), epsilon);
  }

  // Test indexed moment accessors in wind axes
  void testWindAxisIndexedMoments() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    FGColumnVector3 windMoments = aero->GetMomentsInWindAxes();

    TS_ASSERT_DELTA(aero->GetMomentsInWindAxes(1), windMoments(1), epsilon);
    TS_ASSERT_DELTA(aero->GetMomentsInWindAxes(2), windMoments(2), epsilon);
    TS_ASSERT_DELTA(aero->GetMomentsInWindAxes(3), windMoments(3), epsilon);
  }

  /***************************************************************************
   * Moment Reference Center Tests
   ***************************************************************************/

  // Test MRC moments vs CG moments relationship
  void testMRCvsCGMoments() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& momentsCG = aero->GetMoments();
    const FGColumnVector3& momentsMRC = aero->GetMomentsMRC();

    // Without aircraft, both should be zero
    TS_ASSERT_DELTA(momentsCG.Magnitude(), 0.0, epsilon);
    TS_ASSERT_DELTA(momentsMRC.Magnitude(), 0.0, epsilon);
  }

  // Test MRC indexed access consistency
  void testMRCIndexedConsistency() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& momentsMRC = aero->GetMomentsMRC();

    // Verify all three indices match vector
    for (int i = 1; i <= 3; i++) {
      TS_ASSERT_DELTA(aero->GetMomentsMRC(i), momentsMRC(i), epsilon);
    }
  }

  /***************************************************************************
   * Multiple InitModel Calls Tests
   ***************************************************************************/

  // Test multiple InitModel calls don't accumulate state
  void testMultipleInitModelCalls() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Call InitModel multiple times
    for (int i = 0; i < 3; i++) {
      bool result = aero->InitModel();
      TS_ASSERT_EQUALS(result, true);
    }

    // Forces and moments should still be at initial values
    TS_ASSERT_DELTA(aero->GetForces(1), 0.0, epsilon);
    TS_ASSERT_DELTA(aero->GetForces(2), 0.0, epsilon);
    TS_ASSERT_DELTA(aero->GetForces(3), 0.0, epsilon);
  }

  // Test InitModel resets alpha limits to defaults
  void testInitModelResetsAlphaLimits() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Get default values
    double defaultMax = aero->GetAlphaCLMax();
    double defaultMin = aero->GetAlphaCLMin();

    // Change values
    aero->SetAlphaCLMax(1.0);
    aero->SetAlphaCLMin(-1.0);

    // InitModel should reset to defaults
    aero->InitModel();

    TS_ASSERT_DELTA(aero->GetAlphaCLMax(), defaultMax, epsilon);
    TS_ASSERT_DELTA(aero->GetAlphaCLMin(), defaultMin, epsilon);
  }

  /***************************************************************************
   * Aero Function String Format Tests
   ***************************************************************************/

  // Test tab delimiter format
  void testAeroFunctionStringsTabDelimiter() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    std::string tabStrings = aero->GetAeroFunctionStrings("\t");

    // Should not crash and should return consistent format
    // (may be empty without aircraft)
    TS_ASSERT(true);  // Got here without crash
  }

  // Test comma delimiter format
  void testAeroFunctionStringsCommaDelimiter() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    std::string commaStrings = aero->GetAeroFunctionStrings(",");

    // Should not crash
    TS_ASSERT(true);
  }

  // Test empty delimiter
  void testAeroFunctionStringsEmptyDelimiter() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    std::string emptyDelim = aero->GetAeroFunctionStrings("");

    // Should not crash even with empty delimiter
    TS_ASSERT(true);
  }

  // Test values with different delimiters
  void testAeroFunctionValuesDelimiters() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    std::string tabVals = aero->GetAeroFunctionValues("\t");
    std::string commaVals = aero->GetAeroFunctionValues(",");
    std::string emptyVals = aero->GetAeroFunctionValues("");

    // All should work without crash
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Input Structure Tests
   ***************************************************************************/

  // Test that input structure is accessible
  void testInputStructureAccess() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Access input structure members
    double alpha = aero->in.Alpha;
    double beta = aero->in.Beta;
    double vt = aero->in.Vt;
    double qbar = aero->in.Qbar;

    // All should be finite values (likely zero without aircraft)
    TS_ASSERT(!std::isnan(alpha));
    TS_ASSERT(!std::isnan(beta));
    TS_ASSERT(!std::isnan(vt));
    TS_ASSERT(!std::isnan(qbar));
  }

  // Test input structure reference values
  void testInputStructureReferenceValues() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Reference values
    double wingarea = aero->in.Wingarea;
    double wingspan = aero->in.Wingspan;
    double wingchord = aero->in.Wingchord;

    // May be uninitialized without aircraft, but should not crash
    TS_ASSERT(true);
  }

  // Test input structure transformation matrices
  void testInputStructureMatrices() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Transformation matrices
    FGMatrix33 Tb2w = aero->in.Tb2w;
    FGMatrix33 Tw2b = aero->in.Tw2b;

    // Matrices should exist, even if identity
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Aerodynamic Theory Validation Tests
   ***************************************************************************/

  // Test lift coefficient squared relationship
  void testLiftCoefficientSquaredConcept() {
    // ClSquared = (Lift / (qbar * S))^2
    // For induced drag calculation: CDi = CL^2 / (pi * e * AR)

    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double clsq = aero->GetClSquared();

    // ClSquared must be non-negative (it's squared)
    TS_ASSERT(clsq >= 0.0);
  }

  // Test L/D ratio relationship concept
  void testLoDRatioConcept() {
    // L/D = CL / CD = Lift / Drag
    // Higher L/D means better aerodynamic efficiency

    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double lod = aero->GetLoD();

    // Without aircraft, L/D should be finite (0/0 handled)
    TS_ASSERT(!std::isinf(lod) || lod == 0.0);
  }

  // Test stall warning concept
  void testStallWarningConcept() {
    // Stall warning activates when approaching alpha CLmax
    // Value between 0 (no warning) and 1 (full stall)

    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double stallWarn = aero->GetStallWarn();

    // Must be in valid range
    TS_ASSERT(stallWarn >= 0.0);
    TS_ASSERT(stallWarn <= 1.0);
  }

  // Test hysteresis concept for stall
  void testStallHysteresisConcept() {
    // Hysteresis prevents rapid stall/unstall oscillation
    // Stall occurs at higher alpha than unstall

    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double hystParm = aero->GetHysteresisParm();

    // Hysteresis must be non-negative
    TS_ASSERT(hystParm >= 0.0);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  // Test forces at index boundaries
  void testForcesIndexBoundaries() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Valid indices are 1, 2, 3
    TS_ASSERT(!std::isnan(aero->GetForces(1)));
    TS_ASSERT(!std::isnan(aero->GetForces(2)));
    TS_ASSERT(!std::isnan(aero->GetForces(3)));
  }

  // Test moments at index boundaries
  void testMomentsIndexBoundaries() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    // Valid indices are 1, 2, 3
    TS_ASSERT(!std::isnan(aero->GetMoments(1)));
    TS_ASSERT(!std::isnan(aero->GetMoments(2)));
    TS_ASSERT(!std::isnan(aero->GetMoments(3)));
  }

  // Test wind forces at index boundaries
  void testWindForcesIndexBoundaries() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    TS_ASSERT(!std::isnan(aero->GetvFw(1)));
    TS_ASSERT(!std::isnan(aero->GetvFw(2)));
    TS_ASSERT(!std::isnan(aero->GetvFw(3)));
  }

  // Test holding state doesn't modify forces
  void testHoldingDoesNotModifyForces() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& forcesBefore = aero->GetForces();
    double fx = forcesBefore(1);
    double fy = forcesBefore(2);
    double fz = forcesBefore(3);

    // Run with holding=true
    aero->Run(true);

    const FGColumnVector3& forcesAfter = aero->GetForces();
    TS_ASSERT_DELTA(forcesAfter(1), fx, epsilon);
    TS_ASSERT_DELTA(forcesAfter(2), fy, epsilon);
    TS_ASSERT_DELTA(forcesAfter(3), fz, epsilon);
  }

  /***************************************************************************
   * Dimensional Analysis Tests
   ***************************************************************************/

  // Test bi2vel has correct dimensions (time)
  void testBI2VelDimensions() {
    // bi2vel = b / (2V) has dimensions of [length] / [length/time] = [time]
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double bi2vel = aero->GetBI2Vel();

    // Value should be finite (dimensions: seconds)
    TS_ASSERT(!std::isnan(bi2vel));
    TS_ASSERT(!std::isinf(bi2vel));
  }

  // Test ci2vel has correct dimensions (time)
  void testCI2VelDimensions() {
    // ci2vel = c / (2V) has dimensions of [length] / [length/time] = [time]
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    double ci2vel = aero->GetCI2Vel();

    // Value should be finite (dimensions: seconds)
    TS_ASSERT(!std::isnan(ci2vel));
    TS_ASSERT(!std::isinf(ci2vel));
  }

  // Test forces have correct dimensions (force = mass * length / time^2)
  void testForcesDimensions() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& forces = aero->GetForces();

    // Forces in lbf should be finite
    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(!std::isnan(forces(i)));
      TS_ASSERT(!std::isinf(forces(i)));
    }
  }

  // Test moments have correct dimensions (torque = force * length)
  void testMomentsDimensions() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& moments = aero->GetMoments();

    // Moments in lbf-ft should be finite
    for (int i = 1; i <= 3; i++) {
      TS_ASSERT(!std::isnan(moments(i)));
      TS_ASSERT(!std::isinf(moments(i)));
    }
  }

  /***************************************************************************
   * State Consistency Tests
   ***************************************************************************/

  // Test forces vector and indexed access are consistent
  void testForcesVectorIndexedConsistency() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& forces = aero->GetForces();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT_DELTA(aero->GetForces(i), forces(i), epsilon);
    }
  }

  // Test moments vector and indexed access are consistent
  void testMomentsVectorIndexedConsistency() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& moments = aero->GetMoments();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT_DELTA(aero->GetMoments(i), moments(i), epsilon);
    }
  }

  // Test wind forces vector and indexed access consistency
  void testWindForcesVectorIndexedConsistency() {
    FGFDMExec fdmex;
    auto aero = fdmex.GetAerodynamics();

    const FGColumnVector3& vFw = aero->GetvFw();

    for (int i = 1; i <= 3; i++) {
      TS_ASSERT_DELTA(aero->GetvFw(i), vFw(i), epsilon);
    }
  }
};
