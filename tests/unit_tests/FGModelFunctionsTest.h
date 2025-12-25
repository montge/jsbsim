/*******************************************************************************
 * FGModelFunctionsTest.h - Unit tests for FGModelFunctions
 *
 * Tests the FGModelFunctions class including:
 * - Function string output
 * - Pre/Post function execution
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <string>

#include <FGFDMExec.h>
#include <math/FGModelFunctions.h>

using namespace JSBSim;

class FGModelFunctionsTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction Tests
   ***************************************************************************/

  void testConstruction() {
    FGModelFunctions mf;
    TS_ASSERT(true);  // Construction succeeded
  }

  /***************************************************************************
   * Function String Tests
   ***************************************************************************/

  void testGetFunctionStringsEmpty() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings(",");
    // Without any functions loaded, should be empty
    TS_ASSERT(strings.empty());
  }

  void testGetFunctionStringsTabDelimiter() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings("\t");
    TS_ASSERT(strings.empty());
  }

  /***************************************************************************
   * Function Values Tests
   ***************************************************************************/

  void testGetFunctionValuesEmpty() {
    FGModelFunctions mf;

    std::string values = mf.GetFunctionValues(",");
    TS_ASSERT(values.empty());
  }

  void testGetFunctionValuesTabDelimiter() {
    FGModelFunctions mf;

    std::string values = mf.GetFunctionValues("\t");
    TS_ASSERT(values.empty());
  }

  /***************************************************************************
   * Pre/Post Function Execution Tests
   ***************************************************************************/

  void testRunPreFunctionsEmpty() {
    FGModelFunctions mf;

    // Should not crash with no functions
    mf.RunPreFunctions();
    TS_ASSERT(true);
  }

  void testRunPostFunctionsEmpty() {
    FGModelFunctions mf;

    mf.RunPostFunctions();
    TS_ASSERT(true);
  }

  void testRunPreThenPostFunctions() {
    FGModelFunctions mf;

    mf.RunPreFunctions();
    mf.RunPostFunctions();
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Get Pre Function Tests
   ***************************************************************************/

  void testGetPreFunctionNotFound() {
    FGModelFunctions mf;

    auto func = mf.GetPreFunction("nonexistent");
    TS_ASSERT(func == nullptr);
  }

  void testGetPreFunctionEmptyName() {
    FGModelFunctions mf;

    auto func = mf.GetPreFunction("");
    TS_ASSERT(func == nullptr);
  }

  /***************************************************************************
   * Multiple Execution Tests
   ***************************************************************************/

  void testMultiplePreFunctionRuns() {
    FGModelFunctions mf;

    for (int i = 0; i < 10; i++) {
      mf.RunPreFunctions();
    }
    TS_ASSERT(true);
  }

  void testMultiplePostFunctionRuns() {
    FGModelFunctions mf;

    for (int i = 0; i < 10; i++) {
      mf.RunPostFunctions();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Alternating Pre/Post Tests
   ***************************************************************************/

  void testAlternatingPrePost() {
    FGModelFunctions mf;

    for (int i = 0; i < 5; i++) {
      mf.RunPreFunctions();
      mf.RunPostFunctions();
    }
    TS_ASSERT(true);
  }
};
