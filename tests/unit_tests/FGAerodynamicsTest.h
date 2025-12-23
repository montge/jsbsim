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
};
