/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 FGFunctionTest.h - Unit tests for FGFunction class
 Author: Claude Code Assistant
 Date started: December 2025

 ------------- Copyright (C) 2025  JSBSim Development Team ------------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cxxtest/TestSuite.h>
#include <sstream>
#include <limits>
#include <cmath>
#include "math/FGFunction.h"
#include "math/FGRealValue.h"
#include "FGFDMExec.h"
#include "input_output/FGXMLElement.h"
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DOCUMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/** Unit tests for FGFunction class.
    Tests mathematical function operations, nested functions, property binding,
    and edge cases.
    @author Claude Code Assistant
*/

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TEST SUITE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGFunctionTest : public CxxTest::TestSuite
{
private:
  std::shared_ptr<FGFDMExec> fdmex;

  // Helper function to read XML from string
  Element_ptr readFromXML(const std::string& xml) {
    std::stringstream ss;
    ss << xml;
    FGXMLParse xml_parse;
    readXML(ss, xml_parse);
    return xml_parse.GetDocument();
  }

public:
  void setUp() {
    fdmex = std::make_shared<FGFDMExec>();
    fdmex->GetPropertyManager()->GetNode("test/x", true);
    fdmex->GetPropertyManager()->GetNode("test/y", true);
    fdmex->GetPropertyManager()->GetNode("test/z", true);
  }

  void tearDown() {
    fdmex.reset();
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    BASIC FUNCTION TESTS - Sum, Product, Difference, Quotient
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testSumTwoValues() {
    // Test: sum of 3.0 + 5.0 = 8.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sum>"
      "    <value>3.0</value>"
      "    <value>5.0</value>"
      "  </sum>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 8.0, DEFAULT_TOLERANCE);
  }

  void testSumMultipleValues() {
    // Test: sum of 1.0 + 2.0 + 3.0 + 4.0 = 10.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sum>"
      "    <value>1.0</value>"
      "    <value>2.0</value>"
      "    <value>3.0</value>"
      "    <value>4.0</value>"
      "  </sum>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);
  }

  void testProductTwoValues() {
    // Test: product of 4.0 * 5.0 = 20.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>4.0</value>"
      "    <value>5.0</value>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 20.0, DEFAULT_TOLERANCE);
  }

  void testProductMultipleValues() {
    // Test: product of 2.0 * 3.0 * 4.0 = 24.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>2.0</value>"
      "    <value>3.0</value>"
      "    <value>4.0</value>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 24.0, DEFAULT_TOLERANCE);
  }

  void testDifferenceTwoValues() {
    // Test: difference of 10.0 - 3.0 = 7.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <difference>"
      "    <value>10.0</value>"
      "    <value>3.0</value>"
      "  </difference>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 7.0, DEFAULT_TOLERANCE);
  }

  void testDifferenceMultipleValues() {
    // Test: difference of 20.0 - 5.0 - 3.0 = 12.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <difference>"
      "    <value>20.0</value>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "  </difference>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 12.0, DEFAULT_TOLERANCE);
  }

  void testQuotient() {
    // Test: quotient of 20.0 / 4.0 = 5.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <quotient>"
      "    <value>20.0</value>"
      "    <value>4.0</value>"
      "  </quotient>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    POWER AND ROOT OPERATIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testPower() {
    // Test: pow(2.0, 3.0) = 8.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <pow>"
      "    <value>2.0</value>"
      "    <value>3.0</value>"
      "  </pow>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 8.0, DEFAULT_TOLERANCE);
  }

  void testSqrt() {
    // Test: sqrt(25.0) = 5.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sqrt>"
      "    <value>25.0</value>"
      "  </sqrt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    TRIGONOMETRIC FUNCTIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testSin() {
    // Test: sin(pi/2) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sin>"
      "    <value>1.5707963267948966</value>"
      "  </sin>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testCos() {
    // Test: cos(0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <cos>"
      "    <value>0.0</value>"
      "  </cos>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testTan() {
    // Test: tan(pi/4) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <tan>"
      "    <value>0.7853981633974483</value>"
      "  </tan>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testAsin() {
    // Test: asin(0.5) = pi/6 (approx 0.5236)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <asin>"
      "    <value>0.5</value>"
      "  </asin>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_PI/6.0, DEFAULT_TOLERANCE);
  }

  void testAcos() {
    // Test: acos(0.5) = pi/3 (approx 1.0472)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <acos>"
      "    <value>0.5</value>"
      "  </acos>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_PI/3.0, DEFAULT_TOLERANCE);
  }

  void testAtan() {
    // Test: atan(1.0) = pi/4
    Element_ptr elm = readFromXML(
      "<function>"
      "  <atan>"
      "    <value>1.0</value>"
      "  </atan>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_PI/4.0, DEFAULT_TOLERANCE);
  }

  void testAtan2() {
    // Test: atan2(1.0, 1.0) = pi/4
    Element_ptr elm = readFromXML(
      "<function>"
      "  <atan2>"
      "    <value>1.0</value>"
      "    <value>1.0</value>"
      "  </atan2>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_PI/4.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ANGLE CONVERSION FUNCTIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testToRadians() {
    // Test: toradians(180) = pi
    Element_ptr elm = readFromXML(
      "<function>"
      "  <toradians>"
      "    <value>180.0</value>"
      "  </toradians>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_PI, DEFAULT_TOLERANCE);
  }

  void testToDegrees() {
    // Test: todegrees(pi) = 180.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <todegrees>"
      "    <value>3.14159265358979323846</value>"
      "  </todegrees>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 180.0, LOOSE_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    EXPONENTIAL AND LOGARITHMIC FUNCTIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testExp() {
    // Test: exp(1.0) = e (approx 2.71828)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <exp>"
      "    <value>1.0</value>"
      "  </exp>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_E, DEFAULT_TOLERANCE);
  }

  void testLn() {
    // Test: ln(e) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ln>"
      "    <value>2.718281828459045</value>"
      "  </ln>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testLog10() {
    // Test: log10(100) = 2.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <log10>"
      "    <value>100.0</value>"
      "  </log10>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 2.0, DEFAULT_TOLERANCE);
  }

  void testLog2() {
    // Test: log2(128) = 7.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <log2>"
      "    <value>128.0</value>"
      "  </log2>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 7.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ABSOLUTE VALUE AND MIN/MAX FUNCTIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testAbsPositive() {
    // Test: abs(5.0) = 5.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <abs>"
      "    <value>5.0</value>"
      "  </abs>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  void testAbsNegative() {
    // Test: abs(-7.5) = 7.5
    Element_ptr elm = readFromXML(
      "<function>"
      "  <abs>"
      "    <value>-7.5</value>"
      "  </abs>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 7.5, DEFAULT_TOLERANCE);
  }

  void testMin() {
    // Test: min(5.0, 3.0, 8.0, 1.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <min>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "    <value>8.0</value>"
      "    <value>1.0</value>"
      "  </min>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testMax() {
    // Test: max(5.0, 3.0, 8.0, 1.0) = 8.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <max>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "    <value>8.0</value>"
      "    <value>1.0</value>"
      "  </max>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 8.0, DEFAULT_TOLERANCE);
  }

  void testAvg() {
    // Test: avg(2.0, 4.0, 6.0, 8.0) = 5.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <avg>"
      "    <value>2.0</value>"
      "    <value>4.0</value>"
      "    <value>6.0</value>"
      "    <value>8.0</value>"
      "  </avg>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    MODULO AND ROUNDING FUNCTIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testMod() {
    // Test: mod(5, 2) = 1
    Element_ptr elm = readFromXML(
      "<function>"
      "  <mod>"
      "    <value>5</value>"
      "    <value>2</value>"
      "  </mod>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testFmod() {
    // Test: fmod(18.5, 4.2) = 1.7
    Element_ptr elm = readFromXML(
      "<function>"
      "  <fmod>"
      "    <value>18.5</value>"
      "    <value>4.2</value>"
      "  </fmod>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.7, LOOSE_TOLERANCE);
  }

  void testFloorPositive() {
    // Test: floor(2.7) = 2.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <floor>"
      "    <value>2.7</value>"
      "  </floor>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 2.0, DEFAULT_TOLERANCE);
  }

  void testFloorNegative() {
    // Test: floor(-2.3) = -3.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <floor>"
      "    <value>-2.3</value>"
      "  </floor>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), -3.0, DEFAULT_TOLERANCE);
  }

  void testCeilPositive() {
    // Test: ceil(2.3) = 3.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ceil>"
      "    <value>2.3</value>"
      "  </ceil>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 3.0, DEFAULT_TOLERANCE);
  }

  void testCeilNegative() {
    // Test: ceil(-2.7) = -2.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ceil>"
      "    <value>-2.7</value>"
      "  </ceil>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), -2.0, DEFAULT_TOLERANCE);
  }

  void testFraction() {
    // Test: fraction(3.14159) = 0.14159
    Element_ptr elm = readFromXML(
      "<function>"
      "  <fraction>"
      "    <value>3.14159</value>"
      "  </fraction>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.14159, LOOSE_TOLERANCE);
  }

  void testInteger() {
    // Test: integer(3.14159) = 3.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <integer>"
      "    <value>3.14159</value>"
      "  </integer>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 3.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    COMPARISON AND LOGICAL OPERATORS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testLessThanTrue() {
    // Test: lt(3.0, 5.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <lt>"
      "    <value>3.0</value>"
      "    <value>5.0</value>"
      "  </lt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testLessThanFalse() {
    // Test: lt(5.0, 3.0) = 0.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <lt>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "  </lt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  void testLessEqualTrue() {
    // Test: le(3.0, 3.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <le>"
      "    <value>3.0</value>"
      "    <value>3.0</value>"
      "  </le>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testGreaterThanTrue() {
    // Test: gt(5.0, 3.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <gt>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "  </gt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testGreaterEqualTrue() {
    // Test: ge(3.0, 3.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ge>"
      "    <value>3.0</value>"
      "    <value>3.0</value>"
      "  </ge>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testEqualTrue() {
    // Test: eq(5.0, 5.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <eq>"
      "    <value>5.0</value>"
      "    <value>5.0</value>"
      "  </eq>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testNotEqualTrue() {
    // Test: nq(5.0, 3.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <nq>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "  </nq>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testAndTrue() {
    // Test: and(1.0, 1.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <and>"
      "    <value>1.0</value>"
      "    <value>1.0</value>"
      "  </and>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testAndFalse() {
    // Test: and(1.0, 0.0) = 0.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <and>"
      "    <value>1.0</value>"
      "    <value>0.0</value>"
      "  </and>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  void testOrTrue() {
    // Test: or(1.0, 0.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <or>"
      "    <value>1.0</value>"
      "    <value>0.0</value>"
      "  </or>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testOrFalse() {
    // Test: or(0.0, 0.0) = 0.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <or>"
      "    <value>0.0</value>"
      "    <value>0.0</value>"
      "  </or>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  void testNotTrue() {
    // Test: not(0.0) = 1.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <not>"
      "    <value>0.0</value>"
      "  </not>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testNotFalse() {
    // Test: not(1.0) = 0.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <not>"
      "    <value>1.0</value>"
      "  </not>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CONDITIONAL FUNCTIONS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testIfThenTrue() {
    // Test: ifthen(1.0, 10.0, 20.0) = 10.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ifthen>"
      "    <value>1.0</value>"
      "    <value>10.0</value>"
      "    <value>20.0</value>"
      "  </ifthen>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);
  }

  void testIfThenFalse() {
    // Test: ifthen(0.0, 10.0, 20.0) = 20.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ifthen>"
      "    <value>0.0</value>"
      "    <value>10.0</value>"
      "    <value>20.0</value>"
      "  </ifthen>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 20.0, DEFAULT_TOLERANCE);
  }

  void testSwitchIndexZero() {
    // Test: switch(0, 100.0, 200.0, 300.0) = 100.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <switch>"
      "    <value>0</value>"
      "    <value>100.0</value>"
      "    <value>200.0</value>"
      "    <value>300.0</value>"
      "  </switch>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 100.0, DEFAULT_TOLERANCE);
  }

  void testSwitchIndexOne() {
    // Test: switch(1, 100.0, 200.0, 300.0) = 200.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <switch>"
      "    <value>1</value>"
      "    <value>100.0</value>"
      "    <value>200.0</value>"
      "    <value>300.0</value>"
      "  </switch>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 200.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    NESTED FUNCTION TESTS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testNestedProductSum() {
    // Test: (2 + 3) * (4 + 5) = 5 * 9 = 45
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <sum>"
      "      <value>2.0</value>"
      "      <value>3.0</value>"
      "    </sum>"
      "    <sum>"
      "      <value>4.0</value>"
      "      <value>5.0</value>"
      "    </sum>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 45.0, DEFAULT_TOLERANCE);
  }

  void testNestedQuotientProduct() {
    // Test: (2 * 3 * 4) / (2 * 3) = 24 / 6 = 4.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <quotient>"
      "    <product>"
      "      <value>2.0</value>"
      "      <value>3.0</value>"
      "      <value>4.0</value>"
      "    </product>"
      "    <product>"
      "      <value>2.0</value>"
      "      <value>3.0</value>"
      "    </product>"
      "  </quotient>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 4.0, DEFAULT_TOLERANCE);
  }

  void testNestedSqrtSum() {
    // Test: sqrt(9 + 16) = sqrt(25) = 5.0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sqrt>"
      "    <sum>"
      "      <value>9.0</value>"
      "      <value>16.0</value>"
      "    </sum>"
      "  </sqrt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  void testNestedTrigonometric() {
    // Test: sin(asin(0.5)) = 0.5
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sin>"
      "    <asin>"
      "      <value>0.5</value>"
      "    </asin>"
      "  </sin>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.5, DEFAULT_TOLERANCE);
  }

  void testDeeplyNestedFunction() {
    // Test: abs(cos(acos(-0.5))) = abs(-0.5) = 0.5
    Element_ptr elm = readFromXML(
      "<function>"
      "  <abs>"
      "    <cos>"
      "      <acos>"
      "        <value>-0.5</value>"
      "      </acos>"
      "    </cos>"
      "  </abs>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.5, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PROPERTY BINDING TESTS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testPropertySum() {
    // Test: sum of two properties
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(10.0);
    pm->GetNode("test/y")->setDoubleValue(20.0);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <sum>"
      "    <property>test/x</property>"
      "    <property>test/y</property>"
      "  </sum>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 30.0, DEFAULT_TOLERANCE);

    // Verify dynamic update
    pm->GetNode("test/x")->setDoubleValue(15.0);
    TS_ASSERT_DELTA(func.GetValue(), 35.0, DEFAULT_TOLERANCE);
  }

  void testPropertyProduct() {
    // Test: product of property and value
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(5.0);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <property>test/x</property>"
      "    <value>3.0</value>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 15.0, DEFAULT_TOLERANCE);
  }

  void testPropertyInNestedFunction() {
    // Test: sqrt(x^2 + y^2) with properties
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(3.0);
    pm->GetNode("test/y")->setDoubleValue(4.0);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <sqrt>"
      "    <sum>"
      "      <pow>"
      "        <property>test/x</property>"
      "        <value>2.0</value>"
      "      </pow>"
      "      <pow>"
      "        <property>test/y</property>"
      "        <value>2.0</value>"
      "      </pow>"
      "    </sum>"
      "  </sqrt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    EDGE CASE TESTS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testDivisionByZero() {
    // Test: quotient returns HUGE_VAL for division by zero
    Element_ptr elm = readFromXML(
      "<function>"
      "  <quotient>"
      "    <value>10.0</value>"
      "    <value>0.0</value>"
      "  </quotient>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double result = func.GetValue();
    TS_ASSERT(result == HUGE_VAL || std::isinf(result));
  }

  void testSqrtNegative() {
    // Test: sqrt of negative returns -HUGE_VAL
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sqrt>"
      "    <value>-25.0</value>"
      "  </sqrt>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double result = func.GetValue();
    TS_ASSERT_EQUALS(result, -HUGE_VAL);
  }

  void testLnZero() {
    // Test: ln(0) returns -HUGE_VAL
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ln>"
      "    <value>0.0</value>"
      "  </ln>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_EQUALS(func.GetValue(), -HUGE_VAL);
  }

  void testLnNegative() {
    // Test: ln(-1) returns -HUGE_VAL
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ln>"
      "    <value>-1.0</value>"
      "  </ln>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_EQUALS(func.GetValue(), -HUGE_VAL);
  }

  void testLog10Zero() {
    // Test: log10(0) returns -HUGE_VAL
    Element_ptr elm = readFromXML(
      "<function>"
      "  <log10>"
      "    <value>0.0</value>"
      "  </log10>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_EQUALS(func.GetValue(), -HUGE_VAL);
  }

  void testLog2Zero() {
    // Test: log2(0) returns -HUGE_VAL
    Element_ptr elm = readFromXML(
      "<function>"
      "  <log2>"
      "    <value>0.0</value>"
      "  </log2>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_EQUALS(func.GetValue(), -HUGE_VAL);
  }

  void testFmodByZero() {
    // Test: fmod with zero divisor returns HUGE_VAL
    Element_ptr elm = readFromXML(
      "<function>"
      "  <fmod>"
      "    <value>10.0</value>"
      "    <value>0.0</value>"
      "  </fmod>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double result = func.GetValue();
    TS_ASSERT(result == HUGE_VAL || std::isinf(result));
  }

  void testPowerZeroZero() {
    // Test: pow(0, 0) = 1 (by C++ standard)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <pow>"
      "    <value>0.0</value>"
      "    <value>0.0</value>"
      "  </pow>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  void testVeryLargeValue() {
    // Test: Very large value handling
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sum>"
      "    <value>1e300</value>"
      "    <value>1e300</value>"
      "  </sum>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 2e300, 1e290);
  }

  void testVerySmallValue() {
    // Test: Very small value handling
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>1e-150</value>"
      "    <value>1e-150</value>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1e-300, 1e-310);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PI CONSTANT TEST
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testPiConstant() {
    // Test: pi constant returns M_PI
    Element_ptr elm = readFromXML(
      "<function>"
      "  <pi/>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), M_PI, DEFAULT_TOLERANCE);
  }

  void testPiInCalculation() {
    // Test: 2 * pi
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>2.0</value>"
      "    <pi/>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 2.0 * M_PI, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    INTERPOLATE1D TEST
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testInterpolate1D() {
    // Test: interpolate1d with lookup value
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(0.5);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <interpolate1d>"
      "    <property>test/x</property>"
      "    <value>0.0</value> <value>0.0</value>"
      "    <value>1.0</value> <value>10.0</value>"
      "  </interpolate1d>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);

    // Test saturation at lower bound
    pm->GetNode("test/x")->setDoubleValue(-1.0);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);

    // Test saturation at upper bound
    pm->GetNode("test/x")->setDoubleValue(2.0);
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CACHING TESTS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testCacheValue() {
    // Test: caching mechanism
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(5.0);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <property>test/x</property>"
      "    <value>2.0</value>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);

    // Initial value
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);

    // Enable caching
    func.cacheValue(true);
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);

    // Change property - should still return cached value
    pm->GetNode("test/x")->setDoubleValue(7.0);
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);

    // Disable caching - should return updated value
    func.cacheValue(false);
    TS_ASSERT_DELTA(func.GetValue(), 14.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CONSTANT FUNCTION TESTS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  void testIsConstantTrue() {
    // Test: function with only constant values is constant
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sum>"
      "    <value>5.0</value>"
      "    <value>3.0</value>"
      "  </sum>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT(func.IsConstant());
  }

  void testIsConstantFalse() {
    // Test: function with property is not constant
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sum>"
      "    <property>test/x</property>"
      "    <value>3.0</value>"
      "  </sum>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT(!func.IsConstant());
  }
};
