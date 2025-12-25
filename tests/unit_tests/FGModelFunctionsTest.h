/*******************************************************************************
 * FGModelFunctionsTest.h - Unit tests for FGModelFunctions
 *
 * Tests the FGModelFunctions class including:
 * - Function string output
 * - Pre/Post function execution
 * - Multiple instances
 * - Different delimiter handling
 * - Initialization behavior
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

  void testMultipleConstruction() {
    for (int i = 0; i < 10; i++) {
      FGModelFunctions mf;
      TS_ASSERT(true);
    }
  }

  void testSequentialConstruction() {
    FGModelFunctions mf1;
    FGModelFunctions mf2;
    FGModelFunctions mf3;
    TS_ASSERT(true);
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

  void testGetFunctionStringsSemicolonDelimiter() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings(";");
    TS_ASSERT(strings.empty());
  }

  void testGetFunctionStringsSpaceDelimiter() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings(" ");
    TS_ASSERT(strings.empty());
  }

  void testGetFunctionStringsEmptyDelimiter() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings("");
    TS_ASSERT(strings.empty());
  }

  void testGetFunctionStringsLongDelimiter() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings("|||");
    TS_ASSERT(strings.empty());
  }

  void testGetFunctionStringsNewlineDelimiter() {
    FGModelFunctions mf;

    std::string strings = mf.GetFunctionStrings("\n");
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

  void testGetFunctionValuesSemicolonDelimiter() {
    FGModelFunctions mf;

    std::string values = mf.GetFunctionValues(";");
    TS_ASSERT(values.empty());
  }

  void testGetFunctionValuesSpaceDelimiter() {
    FGModelFunctions mf;

    std::string values = mf.GetFunctionValues(" ");
    TS_ASSERT(values.empty());
  }

  void testGetFunctionValuesEmptyDelimiter() {
    FGModelFunctions mf;

    std::string values = mf.GetFunctionValues("");
    TS_ASSERT(values.empty());
  }

  void testGetFunctionValuesLongDelimiter() {
    FGModelFunctions mf;

    std::string values = mf.GetFunctionValues("|||");
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

  void testRunPostThenPreFunctions() {
    FGModelFunctions mf;

    mf.RunPostFunctions();
    mf.RunPreFunctions();
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

  void testGetPreFunctionSpecialChars() {
    FGModelFunctions mf;

    auto func = mf.GetPreFunction("func/sub/name");
    TS_ASSERT(func == nullptr);
  }

  void testGetPreFunctionLongName() {
    FGModelFunctions mf;

    auto func = mf.GetPreFunction("this-is-a-very-long-function-name-that-probably-does-not-exist");
    TS_ASSERT(func == nullptr);
  }

  void testGetPreFunctionNumbers() {
    FGModelFunctions mf;

    auto func = mf.GetPreFunction("123");
    TS_ASSERT(func == nullptr);
  }

  void testGetPreFunctionMultipleTimes() {
    FGModelFunctions mf;

    for (int i = 0; i < 10; i++) {
      auto func = mf.GetPreFunction("nonexistent");
      TS_ASSERT(func == nullptr);
    }
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

  void testManyPreFunctionRuns() {
    FGModelFunctions mf;

    for (int i = 0; i < 100; i++) {
      mf.RunPreFunctions();
    }
    TS_ASSERT(true);
  }

  void testManyPostFunctionRuns() {
    FGModelFunctions mf;

    for (int i = 0; i < 100; i++) {
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

  void testAlternatingPostPre() {
    FGModelFunctions mf;

    for (int i = 0; i < 5; i++) {
      mf.RunPostFunctions();
      mf.RunPreFunctions();
    }
    TS_ASSERT(true);
  }

  void testManyAlternatingRuns() {
    FGModelFunctions mf;

    for (int i = 0; i < 50; i++) {
      mf.RunPreFunctions();
      mf.RunPostFunctions();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Multiple Instance Tests
   ***************************************************************************/

  void testTwoInstances() {
    FGModelFunctions mf1;
    FGModelFunctions mf2;

    mf1.RunPreFunctions();
    mf2.RunPreFunctions();

    TS_ASSERT(true);
  }

  void testThreeInstances() {
    FGModelFunctions mf1;
    FGModelFunctions mf2;
    FGModelFunctions mf3;

    mf1.RunPreFunctions();
    mf2.RunPostFunctions();
    mf3.RunPreFunctions();

    TS_ASSERT(true);
  }

  void testInterleavedInstanceOperations() {
    FGModelFunctions mf1;
    FGModelFunctions mf2;

    for (int i = 0; i < 10; i++) {
      mf1.RunPreFunctions();
      mf2.RunPostFunctions();
      mf1.RunPostFunctions();
      mf2.RunPreFunctions();
    }

    TS_ASSERT(true);
  }

  void testMultipleInstanceStrings() {
    FGModelFunctions mf1;
    FGModelFunctions mf2;
    FGModelFunctions mf3;

    std::string s1 = mf1.GetFunctionStrings(",");
    std::string s2 = mf2.GetFunctionStrings(",");
    std::string s3 = mf3.GetFunctionStrings(",");

    TS_ASSERT(s1.empty());
    TS_ASSERT(s2.empty());
    TS_ASSERT(s3.empty());
  }

  void testMultipleInstanceValues() {
    FGModelFunctions mf1;
    FGModelFunctions mf2;
    FGModelFunctions mf3;

    std::string v1 = mf1.GetFunctionValues(",");
    std::string v2 = mf2.GetFunctionValues(",");
    std::string v3 = mf3.GetFunctionValues(",");

    TS_ASSERT(v1.empty());
    TS_ASSERT(v2.empty());
    TS_ASSERT(v3.empty());
  }

  /***************************************************************************
   * Combined Operation Tests
   ***************************************************************************/

  void testCombinedOperations() {
    FGModelFunctions mf;

    mf.RunPreFunctions();
    std::string strings = mf.GetFunctionStrings(",");
    mf.RunPostFunctions();
    std::string values = mf.GetFunctionValues(",");
    auto func = mf.GetPreFunction("test");

    TS_ASSERT(strings.empty());
    TS_ASSERT(values.empty());
    TS_ASSERT(func == nullptr);
  }

  void testFullOperationSequence() {
    FGModelFunctions mf;

    // Run through all operations
    mf.RunPreFunctions();
    mf.RunPostFunctions();
    mf.GetFunctionStrings(",");
    mf.GetFunctionValues("\t");
    mf.GetPreFunction("nonexistent");
    mf.RunPreFunctions();
    mf.RunPostFunctions();

    TS_ASSERT(true);
  }

  void testRepetitiveOperationSequence() {
    FGModelFunctions mf;

    for (int i = 0; i < 10; i++) {
      mf.RunPreFunctions();
      std::string s = mf.GetFunctionStrings(",");
      TS_ASSERT(s.empty());

      mf.RunPostFunctions();
      std::string v = mf.GetFunctionValues(",");
      TS_ASSERT(v.empty());

      auto func = mf.GetPreFunction("func" + std::to_string(i));
      TS_ASSERT(func == nullptr);
    }
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testGetStringsAfterPreRun() {
    FGModelFunctions mf;

    mf.RunPreFunctions();
    std::string strings = mf.GetFunctionStrings(",");

    TS_ASSERT(strings.empty());
  }

  void testGetStringsAfterPostRun() {
    FGModelFunctions mf;

    mf.RunPostFunctions();
    std::string strings = mf.GetFunctionStrings(",");

    TS_ASSERT(strings.empty());
  }

  void testGetValuesAfterPreRun() {
    FGModelFunctions mf;

    mf.RunPreFunctions();
    std::string values = mf.GetFunctionValues(",");

    TS_ASSERT(values.empty());
  }

  void testGetValuesAfterPostRun() {
    FGModelFunctions mf;

    mf.RunPostFunctions();
    std::string values = mf.GetFunctionValues(",");

    TS_ASSERT(values.empty());
  }

  void testGetPreFunctionAfterRuns() {
    FGModelFunctions mf;

    mf.RunPreFunctions();
    mf.RunPostFunctions();

    auto func = mf.GetPreFunction("test");
    TS_ASSERT(func == nullptr);
  }

  void testRapidGetFunctionStrings() {
    FGModelFunctions mf;

    for (int i = 0; i < 100; i++) {
      std::string s = mf.GetFunctionStrings(",");
      TS_ASSERT(s.empty());
    }
  }

  void testRapidGetFunctionValues() {
    FGModelFunctions mf;

    for (int i = 0; i < 100; i++) {
      std::string v = mf.GetFunctionValues(",");
      TS_ASSERT(v.empty());
    }
  }

  void testDifferentDelimitersSequentially() {
    FGModelFunctions mf;

    std::string delimiters[] = {",", "\t", ";", " ", "|", ":::", "\n", ""};

    for (const auto& delim : delimiters) {
      std::string strings = mf.GetFunctionStrings(delim);
      std::string values = mf.GetFunctionValues(delim);
      TS_ASSERT(strings.empty());
      TS_ASSERT(values.empty());
    }
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testManyInstances() {
    std::vector<FGModelFunctions> functions;

    for (int i = 0; i < 20; i++) {
      functions.emplace_back();
    }

    for (auto& mf : functions) {
      mf.RunPreFunctions();
      mf.RunPostFunctions();
    }

    TS_ASSERT(true);
  }

  void testRapidPrePostAlternation() {
    FGModelFunctions mf;

    for (int i = 0; i < 200; i++) {
      if (i % 2 == 0) {
        mf.RunPreFunctions();
      } else {
        mf.RunPostFunctions();
      }
    }

    TS_ASSERT(true);
  }

  void testMixedOperationsStress() {
    FGModelFunctions mf;

    for (int i = 0; i < 50; i++) {
      switch (i % 5) {
        case 0: mf.RunPreFunctions(); break;
        case 1: mf.RunPostFunctions(); break;
        case 2: mf.GetFunctionStrings(","); break;
        case 3: mf.GetFunctionValues("\t"); break;
        case 4: mf.GetPreFunction("test" + std::to_string(i)); break;
      }
    }

    TS_ASSERT(true);
  }
};
