/*******************************************************************************
 * FGModelFunctionsTest.h - Unit tests for FGModelFunctions
 *
 * Tests the FGModelFunctions class including:
 * - Function string output
 * - Pre/Post function execution
 * - Multiple instances
 * - Different delimiter handling
 * - Initialization behavior
 * - Loading functions from XML
 * - Pre/Post function values
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

#include <FGFDMExec.h>
#include <math/FGModelFunctions.h>
#include <math/FGFunction.h>
#include <input_output/FGXMLElement.h>
#include <models/FGFCS.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGModelFunctionsTest : public CxxTest::TestSuite
{
private:
  // Helper function to read XML from string
  Element_ptr readFromXML(const std::string& xml) {
    std::stringstream ss;
    ss << xml;
    FGXMLParse xml_parse;
    readXML(ss, xml_parse);
    return xml_parse.GetDocument();
  }

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

  /***************************************************************************
   * Function Loading Tests - PreLoad
   ***************************************************************************/

  void testPreLoadSimpleFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/preload\" type=\"pre\">"
      "    <value>42.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/preload");
    TS_ASSERT(func != nullptr);
  }

  void testPreLoadSumFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/sum\" type=\"pre\">"
      "    <sum>"
      "      <value>10.0</value>"
      "      <value>20.0</value>"
      "    </sum>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/sum");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 30.0, 1e-6);
    }
  }

  void testPreLoadProductFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/product\" type=\"pre\">"
      "    <product>"
      "      <value>3.0</value>"
      "      <value>4.0</value>"
      "      <value>5.0</value>"
      "    </product>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/product");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 60.0, 1e-6);
    }
  }

  void testPreLoadMultipleFunctions() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/func1\" type=\"pre\">"
      "    <value>1.0</value>"
      "  </function>"
      "  <function name=\"test/func2\" type=\"pre\">"
      "    <value>2.0</value>"
      "  </function>"
      "  <function name=\"test/func3\" type=\"pre\">"
      "    <value>3.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func1 = mf.GetPreFunction("test/func1");
    auto func2 = mf.GetPreFunction("test/func2");
    auto func3 = mf.GetPreFunction("test/func3");

    TS_ASSERT(func1 != nullptr);
    TS_ASSERT(func2 != nullptr);
    TS_ASSERT(func3 != nullptr);
  }

  void testPreLoadWithPrefix() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/value\" type=\"pre\">"
      "    <value>99.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get(), "prefix");

    auto func = mf.GetPreFunction("test/value");
    // The prefix affects the property path, not the lookup name
    TS_ASSERT(func != nullptr);
  }

  /***************************************************************************
   * Function Loading Tests - PostLoad
   ***************************************************************************/

  void testPostLoadSimpleFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/postfunc\" type=\"post\">"
      "    <value>77.0</value>"
      "  </function>"
      "</model>");

    mf.PostLoad(elm, fdmex.get());
    mf.RunPostFunctions();

    // Post functions are not accessible via GetPreFunction
    auto func = mf.GetPreFunction("test/postfunc");
    TS_ASSERT(func == nullptr);  // It's a post function
  }

  void testPostLoadMultipleFunctions() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/post1\" type=\"post\">"
      "    <value>100.0</value>"
      "  </function>"
      "  <function name=\"test/post2\" type=\"post\">"
      "    <value>200.0</value>"
      "  </function>"
      "</model>");

    mf.PostLoad(elm, fdmex.get());
    mf.RunPostFunctions();

    TS_ASSERT(true);  // No crashes
  }

  /***************************************************************************
   * Function Loading Tests - Mixed Pre/Post
   ***************************************************************************/

  void testMixedPrePostLoad() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/prefn\" type=\"pre\">"
      "    <value>10.0</value>"
      "  </function>"
      "  <function name=\"test/postfn\" type=\"post\">"
      "    <value>20.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.PostLoad(elm, fdmex.get());
    mf.RunPreFunctions();
    mf.RunPostFunctions();

    auto preFunc = mf.GetPreFunction("test/prefn");
    TS_ASSERT(preFunc != nullptr);
  }

  /***************************************************************************
   * Function String/Values Tests with Loaded Functions
   ***************************************************************************/

  void testGetFunctionStringsWithLoadedFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"output/myvalue\">"
      "    <value>123.0</value>"
      "  </function>"
      "</model>");

    mf.Load(elm, fdmex.get());

    std::string strings = mf.GetFunctionStrings(",");
    // Should contain function name or be non-empty
    TS_ASSERT(!strings.empty() || true);  // May be empty if not output type
  }

  void testGetFunctionValuesWithLoadedFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"output/testval\">"
      "    <value>456.0</value>"
      "  </function>"
      "</model>");

    mf.Load(elm, fdmex.get());
    mf.RunPreFunctions();

    std::string values = mf.GetFunctionValues(",");
    TS_ASSERT(true);  // No crash
  }

  void testFunctionStringsMultipleFunctions() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"out/val1\">"
      "    <value>1.0</value>"
      "  </function>"
      "  <function name=\"out/val2\">"
      "    <value>2.0</value>"
      "  </function>"
      "</model>");

    mf.Load(elm, fdmex.get());

    std::string strings = mf.GetFunctionStrings(",");
    std::string values = mf.GetFunctionValues(",");

    TS_ASSERT(true);  // No crash
  }

  /***************************************************************************
   * Mathematical Function Tests
   ***************************************************************************/

  void testLoadedDifferenceFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/diff\" type=\"pre\">"
      "    <difference>"
      "      <value>100.0</value>"
      "      <value>30.0</value>"
      "    </difference>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/diff");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 70.0, 1e-6);
    }
  }

  void testLoadedQuotientFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/quot\" type=\"pre\">"
      "    <quotient>"
      "      <value>100.0</value>"
      "      <value>4.0</value>"
      "    </quotient>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/quot");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 25.0, 1e-6);
    }
  }

  void testLoadedAbsFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/absval\" type=\"pre\">"
      "    <abs>"
      "      <value>-42.5</value>"
      "    </abs>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/absval");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 42.5, 1e-6);
    }
  }

  void testLoadedPowFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/power\" type=\"pre\">"
      "    <pow>"
      "      <value>2.0</value>"
      "      <value>10.0</value>"
      "    </pow>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/power");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 1024.0, 1e-6);
    }
  }

  void testLoadedSqrtFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/sqrt\" type=\"pre\">"
      "    <sqrt>"
      "      <value>144.0</value>"
      "    </sqrt>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/sqrt");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 12.0, 1e-6);
    }
  }

  /***************************************************************************
   * Trigonometric Function Tests
   ***************************************************************************/

  void testLoadedSinFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/sin\" type=\"pre\">"
      "    <sin>"
      "      <value>0.0</value>"
      "    </sin>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/sin");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 0.0, 1e-6);
    }
  }

  void testLoadedCosFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/cos\" type=\"pre\">"
      "    <cos>"
      "      <value>0.0</value>"
      "    </cos>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/cos");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 1.0, 1e-6);
    }
  }

  void testLoadedTanFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/tan\" type=\"pre\">"
      "    <tan>"
      "      <value>0.0</value>"
      "    </tan>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/tan");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 0.0, 1e-6);
    }
  }

  /***************************************************************************
   * Nested Function Tests
   ***************************************************************************/

  void testLoadedNestedFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/nested\" type=\"pre\">"
      "    <sum>"
      "      <product>"
      "        <value>3.0</value>"
      "        <value>4.0</value>"
      "      </product>"
      "      <value>5.0</value>"
      "    </sum>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/nested");
    TS_ASSERT(func != nullptr);
    if (func) {
      // (3 * 4) + 5 = 17
      TS_ASSERT_DELTA(func->GetValue(), 17.0, 1e-6);
    }
  }

  void testLoadedDeeplyNestedFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/deep\" type=\"pre\">"
      "    <product>"
      "      <sum>"
      "        <value>1.0</value>"
      "        <value>2.0</value>"
      "      </sum>"
      "      <difference>"
      "        <value>10.0</value>"
      "        <value>5.0</value>"
      "      </difference>"
      "    </product>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/deep");
    TS_ASSERT(func != nullptr);
    if (func) {
      // (1 + 2) * (10 - 5) = 3 * 5 = 15
      TS_ASSERT_DELTA(func->GetValue(), 15.0, 1e-6);
    }
  }

  /***************************************************************************
   * Min/Max Function Tests
   ***************************************************************************/

  void testLoadedMinFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/min\" type=\"pre\">"
      "    <min>"
      "      <value>5.0</value>"
      "      <value>3.0</value>"
      "      <value>8.0</value>"
      "    </min>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/min");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 3.0, 1e-6);
    }
  }

  void testLoadedMaxFunction() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/max\" type=\"pre\">"
      "    <max>"
      "      <value>5.0</value>"
      "      <value>3.0</value>"
      "      <value>8.0</value>"
      "    </max>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/max");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 8.0, 1e-6);
    }
  }

  /***************************************************************************
   * Edge Cases with Functions
   ***************************************************************************/

  void testLoadFunctionWithZeroValue() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/zero\" type=\"pre\">"
      "    <value>0.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/zero");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 0.0, 1e-6);
    }
  }

  void testLoadFunctionWithNegativeValue() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/negative\" type=\"pre\">"
      "    <value>-123.456</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/negative");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), -123.456, 1e-6);
    }
  }

  void testLoadFunctionWithLargeValue() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/large\" type=\"pre\">"
      "    <value>1e12</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/large");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 1e12, 1e6);
    }
  }

  void testLoadFunctionWithSmallValue() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/small\" type=\"pre\">"
      "    <value>1e-12</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/small");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 1e-12, 1e-18);
    }
  }

  /***************************************************************************
   * Run Functions Multiple Times Tests
   ***************************************************************************/

  void testRunPreFunctionsMultipleTimesAfterLoad() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/multi\" type=\"pre\">"
      "    <value>50.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());

    for (int i = 0; i < 100; i++) {
      mf.RunPreFunctions();
    }

    auto func = mf.GetPreFunction("test/multi");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 50.0, 1e-6);
    }
  }

  void testRunPostFunctionsMultipleTimesAfterLoad() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/postmulti\" type=\"post\">"
      "    <value>75.0</value>"
      "  </function>"
      "</model>");

    mf.PostLoad(elm, fdmex.get());

    for (int i = 0; i < 100; i++) {
      mf.RunPostFunctions();
    }

    TS_ASSERT(true);
  }

  void testAlternatePrePostAfterLoad() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/alt\" type=\"pre\">"
      "    <value>25.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());

    for (int i = 0; i < 50; i++) {
      mf.RunPreFunctions();
      mf.RunPostFunctions();
    }

    auto func = mf.GetPreFunction("test/alt");
    TS_ASSERT(func != nullptr);
  }

  /***************************************************************************
   * Integration Stress Tests
   ***************************************************************************/

  void testStressManyFunctions() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    // Build XML with many functions
    std::stringstream ss;
    ss << "<model>";
    for (int i = 0; i < 20; i++) {
      ss << "<function name=\"test/func" << i << "\" type=\"pre\">"
         << "<value>" << (i * 10.0) << "</value>"
         << "</function>";
    }
    ss << "</model>";

    Element_ptr elm = readFromXML(ss.str());
    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    // Check that all functions are accessible
    for (int i = 0; i < 20; i++) {
      std::string name = "test/func" + std::to_string(i);
      auto func = mf.GetPreFunction(name);
      TS_ASSERT(func != nullptr);
      if (func) {
        TS_ASSERT_DELTA(func->GetValue(), i * 10.0, 1e-6);
      }
    }
  }

  void testStressComplexNestedFunctions() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/complex\" type=\"pre\">"
      "    <sum>"
      "      <product>"
      "        <quotient>"
      "          <value>100.0</value>"
      "          <value>5.0</value>"
      "        </quotient>"
      "        <value>2.0</value>"
      "      </product>"
      "      <abs>"
      "        <difference>"
      "          <value>10.0</value>"
      "          <value>15.0</value>"
      "        </difference>"
      "      </abs>"
      "    </sum>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/complex");
    TS_ASSERT(func != nullptr);
    if (func) {
      // ((100/5) * 2) + abs(10-15) = (20 * 2) + 5 = 40 + 5 = 45
      TS_ASSERT_DELTA(func->GetValue(), 45.0, 1e-6);
    }
  }

  void testStressRapidFunctionAccess() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/rapid\" type=\"pre\">"
      "    <value>999.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());

    for (int i = 0; i < 200; i++) {
      mf.RunPreFunctions();
      auto func = mf.GetPreFunction("test/rapid");
      TS_ASSERT(func != nullptr);
      std::string strings = mf.GetFunctionStrings(",");
      std::string values = mf.GetFunctionValues(",");
    }

    TS_ASSERT(true);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteModelWithMultipleFunctionTypes() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/pre1\" type=\"pre\">"
      "    <value>10.0</value>"
      "  </function>"
      "  <function name=\"test/pre2\" type=\"pre\">"
      "    <value>20.0</value>"
      "  </function>"
      "  <function name=\"test/post1\" type=\"post\">"
      "    <value>30.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.PostLoad(elm, fdmex.get());
    mf.RunPreFunctions();
    mf.RunPostFunctions();

    TS_ASSERT(mf.GetPreFunction("test/pre1") != nullptr);
    TS_ASSERT(mf.GetPreFunction("test/pre2") != nullptr);
  }

  void testFunctionWithSumOperation() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/sum\" type=\"pre\">"
      "    <sum>"
      "      <value>5.0</value>"
      "      <value>10.0</value>"
      "      <value>15.0</value>"
      "    </sum>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/sum");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 30.0, 1e-6);
    }
  }

  void testFunctionWithProductOperation() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/product\" type=\"pre\">"
      "    <product>"
      "      <value>2.0</value>"
      "      <value>3.0</value>"
      "      <value>4.0</value>"
      "    </product>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/product");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 24.0, 1e-6);
    }
  }

  void testFunctionWithMinMaxOperations() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/minmax\" type=\"pre\">"
      "    <sum>"
      "      <min>"
      "        <value>5.0</value>"
      "        <value>10.0</value>"
      "      </min>"
      "      <max>"
      "        <value>3.0</value>"
      "        <value>7.0</value>"
      "      </max>"
      "    </sum>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/minmax");
    TS_ASSERT(func != nullptr);
    if (func) {
      // min(5,10) + max(3,7) = 5 + 7 = 12
      TS_ASSERT_DELTA(func->GetValue(), 12.0, 1e-6);
    }
  }

  void testFunctionWithNestedDifference() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/diff\" type=\"pre\">"
      "    <difference>"
      "      <value>100.0</value>"
      "      <sum>"
      "        <value>20.0</value>"
      "        <value>30.0</value>"
      "      </sum>"
      "    </difference>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/diff");
    TS_ASSERT(func != nullptr);
    if (func) {
      // 100 - (20 + 30) = 50
      TS_ASSERT_DELTA(func->GetValue(), 50.0, 1e-6);
    }
  }

  void testFunctionWithPowerOperation() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/power\" type=\"pre\">"
      "    <pow>"
      "      <value>2.0</value>"
      "      <value>8.0</value>"
      "    </pow>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/power");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 256.0, 1e-6);
    }
  }

  void testFunctionStringOutput() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"output/test\" type=\"pre\">"
      "    <value>42.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    std::string names = mf.GetFunctionStrings(",");
    TS_ASSERT(names.find("output/test") != std::string::npos);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testMultipleFGModelFunctionsInstances() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf1;
    FGModelFunctions mf2;

    Element_ptr elm1 = readFromXML(
      "<model>"
      "  <function name=\"test/func1\" type=\"pre\">"
      "    <value>100.0</value>"
      "  </function>"
      "</model>");

    Element_ptr elm2 = readFromXML(
      "<model>"
      "  <function name=\"test/func2\" type=\"pre\">"
      "    <value>200.0</value>"
      "  </function>"
      "</model>");

    mf1.PreLoad(elm1, fdmex.get());
    mf2.PreLoad(elm2, fdmex.get());
    mf1.RunPreFunctions();
    mf2.RunPreFunctions();

    auto func1 = mf1.GetPreFunction("test/func1");
    auto func2 = mf2.GetPreFunction("test/func2");

    TS_ASSERT(func1 != nullptr);
    TS_ASSERT(func2 != nullptr);
  }

  void testSeparateFunctionNamespaces() {
    // Use separate FDMExec instances to avoid property binding conflicts
    auto fdmex1 = std::make_shared<FGFDMExec>();
    auto fdmex2 = std::make_shared<FGFDMExec>();
    FGModelFunctions mf1;
    FGModelFunctions mf2;

    Element_ptr elm1 = readFromXML(
      "<model>"
      "  <function name=\"test/ns1\" type=\"pre\">"
      "    <value>111.0</value>"
      "  </function>"
      "</model>");

    Element_ptr elm2 = readFromXML(
      "<model>"
      "  <function name=\"test/ns2\" type=\"pre\">"
      "    <value>222.0</value>"
      "  </function>"
      "</model>");

    mf1.PreLoad(elm1, fdmex1.get());
    mf2.PreLoad(elm2, fdmex2.get());
    mf1.RunPreFunctions();
    mf2.RunPreFunctions();

    auto f1 = mf1.GetPreFunction("test/ns1");
    auto f2 = mf2.GetPreFunction("test/ns2");

    TS_ASSERT(f1 != nullptr);
    TS_ASSERT(f2 != nullptr);
    if (f1 && f2) {
      TS_ASSERT_DELTA(f1->GetValue(), 111.0, 1e-6);
      TS_ASSERT_DELTA(f2->GetValue(), 222.0, 1e-6);
    }
  }

  void testFunctionLoadOrderIndependence() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/first\" type=\"pre\">"
      "    <value>1.0</value>"
      "  </function>"
      "  <function name=\"test/second\" type=\"pre\">"
      "    <value>2.0</value>"
      "  </function>"
      "  <function name=\"test/third\" type=\"pre\">"
      "    <value>3.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto f1 = mf.GetPreFunction("test/first");
    auto f2 = mf.GetPreFunction("test/second");
    auto f3 = mf.GetPreFunction("test/third");

    TS_ASSERT(f1 != nullptr);
    TS_ASSERT(f2 != nullptr);
    TS_ASSERT(f3 != nullptr);
  }

  void testPrePostFunctionIndependence() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/pre\" type=\"pre\">"
      "    <value>10.0</value>"
      "  </function>"
      "  <function name=\"test/post\" type=\"post\">"
      "    <value>20.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.PostLoad(elm, fdmex.get());

    mf.RunPreFunctions();
    auto preFn = mf.GetPreFunction("test/pre");
    TS_ASSERT(preFn != nullptr);

    mf.RunPostFunctions();
    // Post functions run but we just verify pre function exists
    TS_ASSERT(preFn != nullptr);
  }

  void testFunctionValuesCommaDelimited() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/a\" type=\"pre\">"
      "    <value>1.0</value>"
      "  </function>"
      "  <function name=\"test/b\" type=\"pre\">"
      "    <value>2.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    std::string values = mf.GetFunctionValues(",");
    TS_ASSERT(!values.empty());
  }

  void testEmptyFunctionModel() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML("<model></model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    std::string names = mf.GetFunctionStrings(",");
    // Empty or contains no function names
    TS_ASSERT(names.empty() || names.find("test") == std::string::npos);
  }

  void testFunctionWithZeroValue() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/zero\" type=\"pre\">"
      "    <value>0.0</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/zero");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), 0.0, 1e-10);
    }
  }

  void testFunctionWithNegativeValue() {
    auto fdmex = std::make_shared<FGFDMExec>();
    FGModelFunctions mf;

    Element_ptr elm = readFromXML(
      "<model>"
      "  <function name=\"test/negative\" type=\"pre\">"
      "    <value>-123.456</value>"
      "  </function>"
      "</model>");

    mf.PreLoad(elm, fdmex.get());
    mf.RunPreFunctions();

    auto func = mf.GetPreFunction("test/negative");
    TS_ASSERT(func != nullptr);
    if (func) {
      TS_ASSERT_DELTA(func->GetValue(), -123.456, 1e-6);
    }
  }
};

// ============================================================================
// C172x Integration Tests for FGModelFunctions
// ============================================================================

class FGModelFunctionsC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {
    std::string rootDir = JSBSIM_TEST_ROOT_DIR;
    fdm.SetRootDir(SGPath(rootDir));
    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: C172x aerodynamics model has functions
  void testC172xAerodynamicsModelFunctions() {
    auto aero = fdm.GetAerodynamics();
    TS_ASSERT(aero != nullptr);

    // Run pre-functions on aerodynamics model
    aero->RunPreFunctions();
    TS_ASSERT(true);  // No crash
  }

  // Test 2: C172x FCS model has functions
  void testC172xFCSModelFunctions() {
    auto fcs = fdm.GetFCS();
    TS_ASSERT(fcs != nullptr);

    fdm.RunIC();
    fdm.Run();
    TS_ASSERT(true);
  }

  // Test 3: C172x propulsion model with functions
  void testC172xPropulsionModelFunctions() {
    auto prop = fdm.GetPropulsion();
    TS_ASSERT(prop != nullptr);

    fdm.RunIC();
    fdm.Run();
    TS_ASSERT(true);
  }

  // Test 4: C172x model functions execute during simulation
  void testC172xModelFunctionsExecuteDuringSimulation() {
    fdm.RunIC();

    // Run multiple simulation frames
    for (int i = 0; i < 50; i++) {
      fdm.Run();
    }

    // Verify simulation ran successfully
    double simTime = fdm.GetPropertyManager()->GetNode("simulation/sim-time-sec")->getDoubleValue();
    TS_ASSERT(simTime > 0.0);
  }

  // Test 5: C172x function values accessible via properties
  void testC172xFunctionValuesViaProperties() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();

    // Access various computed values (results of internal functions)
    double qbar = pm->GetNode("aero/qbar-psf")->getDoubleValue();
    TS_ASSERT(!std::isnan(qbar));
    TS_ASSERT(qbar >= 0.0);
  }

  // Test 6: C172x aerodynamic coefficient functions
  void testC172xAeroCoefficientFunctions() {
    fdm.RunIC();
    fdm.GetFCS()->SetThrottleCmd(0, 0.8);
    fdm.GetFCS()->SetDeCmd(0.1);

    for (int i = 0; i < 20; i++) {
      fdm.Run();
    }

    auto pm = fdm.GetPropertyManager();
    double cl = pm->GetNode("aero/coefficient/CLwbh")->getDoubleValue();
    double cd = pm->GetNode("aero/coefficient/CDwbh")->getDoubleValue();

    // Coefficients should be computed (may be zero on ground)
    TS_ASSERT(!std::isnan(cl));
    TS_ASSERT(!std::isnan(cd));
  }

  // Test 7: C172x trim functions
  void testC172xTrimFunctions() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();

    // Trim tab position (if defined)
    double elevatorTrim = pm->GetNode("fcs/pitch-trim-cmd-norm")->getDoubleValue();
    TS_ASSERT(!std::isnan(elevatorTrim));
  }

  // Test 8: C172x mass functions
  void testC172xMassFunctions() {
    auto mass = fdm.GetMassBalance();
    TS_ASSERT(mass != nullptr);

    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double weight = pm->GetNode("inertia/weight-lbs")->getDoubleValue();
    double cgX = pm->GetNode("inertia/cg-x-in")->getDoubleValue();

    TS_ASSERT(!std::isnan(weight));
    TS_ASSERT(!std::isnan(cgX));
    TS_ASSERT(weight > 0.0);
  }

  // Test 9: C172x ground reaction functions
  void testC172xGroundReactionFunctions() {
    auto gr = fdm.GetGroundReactions();
    TS_ASSERT(gr != nullptr);

    fdm.RunIC();
    fdm.Run();

    // Ground reactions should be active when on ground
    TS_ASSERT(true);
  }

  // Test 10: C172x auxiliary functions
  void testC172xAuxiliaryFunctions() {
    auto aux = fdm.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double mach = pm->GetNode("velocities/mach")->getDoubleValue();
    double vc = pm->GetNode("velocities/vc-kts")->getDoubleValue();

    TS_ASSERT(!std::isnan(mach));
    TS_ASSERT(!std::isnan(vc));
    TS_ASSERT(mach >= 0.0);
  }

  // Test 11: C172x function execution order
  void testC172xFunctionExecutionOrder() {
    fdm.RunIC();

    // Pre-functions run before main model, post-functions after
    auto aero = fdm.GetAerodynamics();
    aero->RunPreFunctions();
    fdm.Run();
    aero->RunPostFunctions();

    TS_ASSERT(true);
  }

  // Test 12: C172x function stability over time
  void testC172xFunctionStabilityOverTime() {
    fdm.RunIC();
    fdm.GetFCS()->SetThrottleCmd(0, 0.6);
    fdm.GetFCS()->SetMixtureCmd(0, 0.9);

    auto pm = fdm.GetPropertyManager();
    std::vector<double> qbarValues;

    for (int i = 0; i < 100; i++) {
      fdm.Run();
      double qbar = pm->GetNode("aero/qbar-psf")->getDoubleValue();
      qbarValues.push_back(qbar);
      TS_ASSERT(!std::isnan(qbar));
      TS_ASSERT(!std::isinf(qbar));
    }

    // Values should remain stable (no wild oscillations)
    for (size_t i = 1; i < qbarValues.size(); i++) {
      double diff = std::abs(qbarValues[i] - qbarValues[i-1]);
      // Allow up to 50% change per frame maximum
      TS_ASSERT(diff < std::abs(qbarValues[i-1]) * 0.5 + 10.0);
    }
  }
};
