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
#include "models/FGFCS.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGThruster.h"
#include "models/FGAuxiliary.h"
#include "models/FGAerodynamics.h"
#include "math/FGColumnVector3.h"
#include "initialization/FGInitialCondition.h"
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

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ADDITIONAL FUNCTION TESTS (77-80)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 77: Round to multiple
  void testRoundMultiple() {
    // Round 23.7 to nearest multiple of 5 = 25
    Element_ptr elm = readFromXML(
      "<function>"
      "  <roundmultiple>"
      "    <value>23.7</value>"
      "    <value>5.0</value>"
      "  </roundmultiple>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 25.0, DEFAULT_TOLERANCE);
  }

  // Test 78: Round to multiple (smaller value)
  void testRoundMultipleSmall() {
    // Round 0.27 to nearest multiple of 0.1 = 0.3
    Element_ptr elm = readFromXML(
      "<function>"
      "  <roundmultiple>"
      "    <value>0.27</value>"
      "    <value>0.1</value>"
      "  </roundmultiple>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.3, DEFAULT_TOLERANCE);
  }

  // Test 79: Exp(0) = 1
  void testExpZero() {
    Element_ptr elm = readFromXML(
      "<function>"
      "  <exp>"
      "    <value>0.0</value>"
      "  </exp>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  // Test 80: Random function produces value in range
  void testRandomRange() {
    Element_ptr elm = readFromXML(
      "<function>"
      "  <random/>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double value = func.GetValue();
    // Random should produce a value (just check it's a valid number)
    TS_ASSERT(!std::isnan(value));
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SIGN AND CLIPPING TESTS (81-84)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 81: Sign of positive number
  void testSignPositive() {
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sign>"
      "    <value>5.0</value>"
      "  </sign>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  // Test 82: Sign of negative number
  void testSignNegative() {
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sign>"
      "    <value>-5.0</value>"
      "  </sign>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), -1.0, DEFAULT_TOLERANCE);
  }

  // Test 83: Sign of zero (JSBSim treats 0 as positive)
  void testSignZero() {
    Element_ptr elm = readFromXML(
      "<function>"
      "  <sign>"
      "    <value>0.0</value>"
      "  </sign>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);  // JSBSim: 0 counts as positive
  }

  // Test 84: Clipping with min and max
  void testClipMinMax() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(15.0);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <min>"
      "    <max>"
      "      <property>test/x</property>"
      "      <value>0.0</value>"
      "    </max>"
      "    <value>10.0</value>"
      "  </min>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 10.0, DEFAULT_TOLERANCE);

    // Test lower clipping
    pm->GetNode("test/x")->setDoubleValue(-5.0);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);

    // Test pass-through
    pm->GetNode("test/x")->setDoubleValue(5.0);
    TS_ASSERT_DELTA(func.GetValue(), 5.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    COMPLEX EXPRESSION TESTS (85-88)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 85: Quadratic formula discriminant
  void testQuadraticDiscriminant() {
    // b^2 - 4ac with a=1, b=5, c=6: 25 - 24 = 1
    Element_ptr elm = readFromXML(
      "<function>"
      "  <difference>"
      "    <pow>"
      "      <value>5.0</value>"
      "      <value>2.0</value>"
      "    </pow>"
      "    <product>"
      "      <value>4.0</value>"
      "      <value>1.0</value>"
      "      <value>6.0</value>"
      "    </product>"
      "  </difference>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  // Test 86: Polar to Cartesian X
  void testPolarToCartesianX() {
    // x = r * cos(theta), r=5, theta=pi/3
    double theta = M_PI / 3.0;
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>5.0</value>"
      "    <cos>"
      "      <value>1.0471975511965976</value>"
      "    </cos>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0 * std::cos(theta), DEFAULT_TOLERANCE);
  }

  // Test 87: Polar to Cartesian Y
  void testPolarToCartesianY() {
    // y = r * sin(theta), r=5, theta=pi/3
    double theta = M_PI / 3.0;
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>5.0</value>"
      "    <sin>"
      "      <value>1.0471975511965976</value>"
      "    </sin>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 5.0 * std::sin(theta), DEFAULT_TOLERANCE);
  }

  // Test 88: Distance formula
  void testDistanceFormula() {
    // Distance between (0,0) and (3,4) = 5
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
    CONDITIONAL LOGIC TESTS (89-92)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 89: Nested conditional
  void testNestedConditional() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(5.0);

    // if (x > 10) then 100 else (if x > 0 then 50 else 0)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <ifthen>"
      "    <gt>"
      "      <property>test/x</property>"
      "      <value>10.0</value>"
      "    </gt>"
      "    <value>100.0</value>"
      "    <ifthen>"
      "      <gt>"
      "        <property>test/x</property>"
      "        <value>0.0</value>"
      "      </gt>"
      "      <value>50.0</value>"
      "      <value>0.0</value>"
      "    </ifthen>"
      "  </ifthen>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 50.0, DEFAULT_TOLERANCE);

    pm->GetNode("test/x")->setDoubleValue(15.0);
    TS_ASSERT_DELTA(func.GetValue(), 100.0, DEFAULT_TOLERANCE);

    pm->GetNode("test/x")->setDoubleValue(-5.0);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  // Test 90: Complex AND condition
  void testComplexAndCondition() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(5.0);
    pm->GetNode("test/y")->setDoubleValue(3.0);

    // x > 0 AND y > 0 AND x > y
    Element_ptr elm = readFromXML(
      "<function>"
      "  <and>"
      "    <gt>"
      "      <property>test/x</property>"
      "      <value>0.0</value>"
      "    </gt>"
      "    <gt>"
      "      <property>test/y</property>"
      "      <value>0.0</value>"
      "    </gt>"
      "    <gt>"
      "      <property>test/x</property>"
      "      <property>test/y</property>"
      "    </gt>"
      "  </and>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);

    pm->GetNode("test/x")->setDoubleValue(2.0);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  // Test 91: Complex OR condition
  void testComplexOrCondition() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(-5.0);
    pm->GetNode("test/y")->setDoubleValue(-3.0);

    // x > 10 OR y > 10 OR (x + y) < 0
    Element_ptr elm = readFromXML(
      "<function>"
      "  <or>"
      "    <gt>"
      "      <property>test/x</property>"
      "      <value>10.0</value>"
      "    </gt>"
      "    <gt>"
      "      <property>test/y</property>"
      "      <value>10.0</value>"
      "    </gt>"
      "    <lt>"
      "      <sum>"
      "        <property>test/x</property>"
      "        <property>test/y</property>"
      "      </sum>"
      "      <value>0.0</value>"
      "    </lt>"
      "  </or>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);
  }

  // Test 92: XOR-like behavior using AND/OR/NOT
  void testXorBehavior() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(1.0);
    pm->GetNode("test/y")->setDoubleValue(0.0);

    // XOR: (x AND NOT y) OR (NOT x AND y)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <or>"
      "    <and>"
      "      <property>test/x</property>"
      "      <not>"
      "        <property>test/y</property>"
      "      </not>"
      "    </and>"
      "    <and>"
      "      <not>"
      "        <property>test/x</property>"
      "      </not>"
      "      <property>test/y</property>"
      "    </and>"
      "  </or>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 1.0, DEFAULT_TOLERANCE);

    // Both true - XOR should be false
    pm->GetNode("test/y")->setDoubleValue(1.0);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);

    // Both false - XOR should be false
    pm->GetNode("test/x")->setDoubleValue(0.0);
    pm->GetNode("test/y")->setDoubleValue(0.0);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    FLIGHT DYNAMICS FORMULA TESTS (93-96)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 93: Dynamic pressure calculation
  void testDynamicPressure() {
    // q = 0.5 * rho * V^2, with rho=0.002377, V=200
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <value>0.5</value>"
      "    <value>0.002377</value>"
      "    <pow>"
      "      <value>200.0</value>"
      "      <value>2.0</value>"
      "    </pow>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double expected = 0.5 * 0.002377 * 200.0 * 200.0;
    TS_ASSERT_DELTA(func.GetValue(), expected, DEFAULT_TOLERANCE);
  }

  // Test 94: Lift coefficient calculation
  void testLiftCoefficient() {
    // CL = L / (q * S), with L=10000, q=47.54, S=200
    Element_ptr elm = readFromXML(
      "<function>"
      "  <quotient>"
      "    <value>10000.0</value>"
      "    <product>"
      "      <value>47.54</value>"
      "      <value>200.0</value>"
      "    </product>"
      "  </quotient>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double expected = 10000.0 / (47.54 * 200.0);
    TS_ASSERT_DELTA(func.GetValue(), expected, LOOSE_TOLERANCE);
  }

  // Test 95: Mach number calculation
  void testMachNumber() {
    // M = V / a, with V=500, a=1116 (speed of sound in ft/s at sea level)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <quotient>"
      "    <value>500.0</value>"
      "    <value>1116.0</value>"
      "  </quotient>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    double expected = 500.0 / 1116.0;
    TS_ASSERT_DELTA(func.GetValue(), expected, DEFAULT_TOLERANCE);
  }

  // Test 96: Bank angle from load factor
  void testBankAngleFromLoadFactor() {
    // Bank = acos(1/n), with n=2 (60 degree bank)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <todegrees>"
      "    <acos>"
      "      <quotient>"
      "        <value>1.0</value>"
      "        <value>2.0</value>"
      "      </quotient>"
      "    </acos>"
      "  </todegrees>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 60.0, LOOSE_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    INTERPOLATION AND TABLE TESTS (97-99)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 97: Interpolate1D with multiple breakpoints
  void testInterpolate1DMultipleBreakpoints() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(1.5);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <interpolate1d>"
      "    <property>test/x</property>"
      "    <value>0.0</value> <value>0.0</value>"
      "    <value>1.0</value> <value>10.0</value>"
      "    <value>2.0</value> <value>30.0</value>"
      "    <value>3.0</value> <value>60.0</value>"
      "  </interpolate1d>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    // Linear interpolation between 10 and 30 at x=1.5
    TS_ASSERT_DELTA(func.GetValue(), 20.0, DEFAULT_TOLERANCE);

    pm->GetNode("test/x")->setDoubleValue(2.5);
    TS_ASSERT_DELTA(func.GetValue(), 45.0, DEFAULT_TOLERANCE);
  }

  // Test 98: Interpolate1D with negative values
  void testInterpolate1DNegativeValues() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(0.0);

    Element_ptr elm = readFromXML(
      "<function>"
      "  <interpolate1d>"
      "    <property>test/x</property>"
      "    <value>-10.0</value> <value>-100.0</value>"
      "    <value>0.0</value> <value>0.0</value>"
      "    <value>10.0</value> <value>100.0</value>"
      "  </interpolate1d>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    TS_ASSERT_DELTA(func.GetValue(), 0.0, DEFAULT_TOLERANCE);

    pm->GetNode("test/x")->setDoubleValue(-5.0);
    TS_ASSERT_DELTA(func.GetValue(), -50.0, DEFAULT_TOLERANCE);

    pm->GetNode("test/x")->setDoubleValue(5.0);
    TS_ASSERT_DELTA(func.GetValue(), 50.0, DEFAULT_TOLERANCE);
  }

  // Test 99: Interpolate1D used in complex expression
  void testInterpolate1DInExpression() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(0.5);

    // Scale interpolated value by a factor
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <interpolate1d>"
      "      <property>test/x</property>"
      "      <value>0.0</value> <value>1.0</value>"
      "      <value>1.0</value> <value>2.0</value>"
      "    </interpolate1d>"
      "    <value>10.0</value>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);
    // At x=0.5, interpolated value = 1.5, multiplied by 10 = 15
    TS_ASSERT_DELTA(func.GetValue(), 15.0, DEFAULT_TOLERANCE);
  }

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    COMPLETE FUNCTION SYSTEM TEST (100)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  // Test 100: Complete aerodynamic coefficient calculation
  void testCompleteAeroCoefficient() {
    auto pm = fdmex->GetPropertyManager();
    pm->GetNode("test/x")->setDoubleValue(5.0);  // Alpha in degrees
    pm->GetNode("test/y")->setDoubleValue(0.8);  // Mach number
    pm->GetNode("test/z")->setDoubleValue(1.0);  // Delta elevator

    // Complex lift coefficient calculation:
    // CL = CL_alpha * alpha + CL_de * de + CL_mach_correction
    // where CL_alpha = 0.1, CL_de = 0.02
    // CL_mach_correction = 1 / sqrt(1 - M^2) for M < 1 (Prandtl-Glauert)
    Element_ptr elm = readFromXML(
      "<function>"
      "  <product>"
      "    <sum>"
      "      <product>"
      "        <value>0.1</value>"
      "        <property>test/x</property>"
      "      </product>"
      "      <product>"
      "        <value>0.02</value>"
      "        <property>test/z</property>"
      "      </product>"
      "    </sum>"
      "    <quotient>"
      "      <value>1.0</value>"
      "      <sqrt>"
      "        <difference>"
      "          <value>1.0</value>"
      "          <pow>"
      "            <property>test/y</property>"
      "            <value>2.0</value>"
      "          </pow>"
      "        </difference>"
      "      </sqrt>"
      "    </quotient>"
      "  </product>"
      "</function>");

    FGFunction func(fdmex.get(), elm);

    // Manual calculation:
    // CL_base = 0.1 * 5 + 0.02 * 1 = 0.52
    // beta = sqrt(1 - 0.64) = sqrt(0.36) = 0.6
    // CL = 0.52 / 0.6 = 0.8667
    double alpha = 5.0;
    double mach = 0.8;
    double de = 1.0;
    double CL_base = 0.1 * alpha + 0.02 * de;
    double beta = std::sqrt(1.0 - mach * mach);
    double expected = CL_base / beta;

    TS_ASSERT_DELTA(func.GetValue(), expected, LOOSE_TOLERANCE);

    // Verify the function updates with property changes
    pm->GetNode("test/x")->setDoubleValue(10.0);  // Double alpha
    alpha = 10.0;
    CL_base = 0.1 * alpha + 0.02 * de;
    expected = CL_base / beta;
    TS_ASSERT_DELTA(func.GetValue(), expected, LOOSE_TOLERANCE);

    // Verify Mach effect
    pm->GetNode("test/y")->setDoubleValue(0.5);  // Lower Mach
    mach = 0.5;
    beta = std::sqrt(1.0 - mach * mach);
    expected = CL_base / beta;
    TS_ASSERT_DELTA(func.GetValue(), expected, LOOSE_TOLERANCE);
  }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C172X INTEGRATION TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGFunctionC172xTest : public CxxTest::TestSuite
{
public:
  void testC172xAerodynamicFunctions() {
    // Test that aerodynamic functions evaluate correctly with C172x model
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aero = fdmex.GetAerodynamics();
    TS_ASSERT(aero != nullptr);

    // Run a few iterations to stabilize
    for (int i = 0; i < 10; i++) {
      fdmex.Run();
    }

    // Verify aerodynamic forces are finite
    const FGColumnVector3& forces = aero->GetForces();
    TS_ASSERT(std::isfinite(forces(1)));
    TS_ASSERT(std::isfinite(forces(2)));
    TS_ASSERT(std::isfinite(forces(3)));
  }

  void testC172xPropertyFunctions() {
    // Test function evaluation with C172x property tree
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto pm = fdmex.GetPropertyManager();
    TS_ASSERT(pm != nullptr);

    // Check standard properties exist and are finite
    TS_ASSERT(std::isfinite(pm->GetNode("velocities/vc-kts")->getDoubleValue()));
    TS_ASSERT(std::isfinite(pm->GetNode("position/h-sl-ft")->getDoubleValue()));
    TS_ASSERT(std::isfinite(pm->GetNode("attitude/phi-rad")->getDoubleValue()));
  }

  void testC172xThrottleFunction() {
    // Test throttle command affects engine thrust via functions
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto prop = fdmex.GetPropulsion();
    TS_ASSERT(prop != nullptr);
    prop->InitRunning(-1);
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    // Set low throttle
    fcs->SetThrottleCmd(-1, 0.3);
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }
    double lowThrust = prop->GetEngine(0)->GetThrust();

    // Set high throttle
    fcs->SetThrottleCmd(-1, 0.9);
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }
    double highThrust = prop->GetEngine(0)->GetThrust();

    // Verify thrust values are finite and valid
    TS_ASSERT(std::isfinite(lowThrust));
    TS_ASSERT(std::isfinite(highThrust));
    TS_ASSERT(highThrust >= 0.0);
  }

  void testC172xMixtureFunction() {
    // Test mixture affects engine performance
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto prop = fdmex.GetPropulsion();
    TS_ASSERT(prop != nullptr);
    prop->InitRunning(-1);
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    // Full rich mixture
    fcs->SetMixtureCmd(-1, 1.0);
    fcs->SetThrottleCmd(-1, 0.7);
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }
    double thrust = prop->GetEngine(0)->GetThrust();
    TS_ASSERT(std::isfinite(thrust));
    TS_ASSERT(thrust > 0.0);
  }

  void testC172xPropellerFunction() {
    // Test propeller thrust function
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto prop = fdmex.GetPropulsion();
    TS_ASSERT(prop != nullptr);
    prop->InitRunning(-1);
    fdmex.RunIC();

    // Run to stabilize
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Check thruster output
    auto thruster = prop->GetEngine(0)->GetThruster();
    TS_ASSERT(thruster != nullptr);
    double rpm = thruster->GetRPM();
    TS_ASSERT(std::isfinite(rpm));
    TS_ASSERT(rpm > 0.0);
  }

  void testC172xElevatorFunction() {
    // Test elevator command function
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    // Apply elevator input
    fcs->SetDeCmd(-0.5);
    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    double dePos = fcs->GetDePos();
    TS_ASSERT(std::isfinite(dePos));
    TS_ASSERT(dePos != 0.0);
  }

  void testC172xAileronFunction() {
    // Test aileron command function
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    // Apply aileron input
    fcs->SetDaCmd(0.3);
    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    double daLPos = fcs->GetDaLPos();
    double daRPos = fcs->GetDaRPos();
    TS_ASSERT(std::isfinite(daLPos));
    TS_ASSERT(std::isfinite(daRPos));
  }

  void testC172xRudderFunction() {
    // Test rudder command function
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);

    // Apply rudder input
    fcs->SetDrCmd(0.4);
    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    double drPos = fcs->GetDrPos();
    TS_ASSERT(std::isfinite(drPos));
  }

  void testC172xFlightPathFunction() {
    // Test flight path calculations
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto prop = fdmex.GetPropulsion();
    TS_ASSERT(prop != nullptr);
    prop->InitRunning(-1);
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    // Verify flight path angle is finite
    double gamma = aux->GetGamma();
    TS_ASSERT(std::isfinite(gamma));
  }

  void testC172xDynamicPressureFunction() {
    // Test dynamic pressure calculations
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");

    auto prop = fdmex.GetPropulsion();
    TS_ASSERT(prop != nullptr);
    prop->InitRunning(-1);
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double qbar = aux->Getqbar();
    TS_ASSERT(std::isfinite(qbar));
    TS_ASSERT(qbar >= 0.0);
  }

  void testC172xAlphaFunction() {
    // Test angle of attack calculations
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(aux != nullptr);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    double alpha = aux->Getalpha();
    TS_ASSERT(std::isfinite(alpha));
  }

  void testC172xBetaFunction() {
    // Test sideslip angle calculations
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();

    auto fcs = fdmex.GetFCS();
    auto aux = fdmex.GetAuxiliary();
    TS_ASSERT(fcs != nullptr);
    TS_ASSERT(aux != nullptr);

    // Apply rudder to induce sideslip
    fcs->SetDrCmd(0.5);
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    double beta = aux->Getbeta();
    TS_ASSERT(std::isfinite(beta));
  }
};
