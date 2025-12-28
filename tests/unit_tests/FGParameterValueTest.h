#include <cxxtest/TestSuite.h>
#include <math/FGParameterValue.h>
#include "TestUtilities.h"
#include <limits>
#include <cmath>

using namespace JSBSim;
using namespace JSBSimTest;


class FGParameterValueTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * String Constructor Tests - Real Values
   ***************************************************************************/

  void testRealConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("1.2", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetValue(), 1.2);
    TS_ASSERT_EQUALS(x.GetName(), "constant value 1.200000");
  }

  void testRealConstructorNegative() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("-5.5", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetValue(), -5.5);
  }

  void testRealConstructorZero() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("0.0", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
  }

  void testRealConstructorInteger() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("42", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  void testRealConstructorScientificNotation() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("1.5e10", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 1.5e10);
  }

  void testRealConstructorNegativeExponent() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("3.14e-5", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 3.14e-5);
  }

  void testRealConstructorLeadingZero() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("0.123", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 0.123);
  }

  /***************************************************************************
   * String Constructor Tests - Property Values
   ***************************************************************************/

  void testPropertyConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetName(), "x");

    node->setDoubleValue(0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    node->setDoubleValue(1.2);
    TS_ASSERT_EQUALS(x.GetValue(), 1.2);
  }

  void testPropertyConstructorNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("aero/alpha-deg", true);
    FGParameterValue x("aero/alpha-deg", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(!x.IsLateBound());

    node->setDoubleValue(5.5);
    TS_ASSERT_EQUALS(x.GetValue(), 5.5);
  }

  void testPropertyConstructorSigned() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("-x", pm, nullptr);

    TS_ASSERT(!x.IsConstant());

    node->setDoubleValue(10.0);
    TS_ASSERT_EQUALS(x.GetValue(), -10.0);
  }

  /***************************************************************************
   * Late Bound Property Tests
   ***************************************************************************/

  void testLateBoundPropertyConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("x", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetName(), "x");

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT(!x.IsLateBound());
    node->setDoubleValue(1.2);
    TS_ASSERT_EQUALS(x.GetValue(), 1.2);
  }

  void testLateBoundPropertyIllegalAccess() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("x", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetName(), "x");
    TS_ASSERT_THROWS(x.GetValue(), BaseException&);
  }

  void testLateBoundNestedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("fcs/elevator-cmd", pm, nullptr);

    TS_ASSERT(x.IsLateBound());

    auto node = pm->GetNode("fcs/elevator-cmd", true);
    node->setDoubleValue(0.5);
    TS_ASSERT_EQUALS(x.GetValue(), 0.5);
    TS_ASSERT(!x.IsLateBound());
  }

  void testLateBoundSignedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("-throttle", pm, nullptr);

    TS_ASSERT(x.IsLateBound());

    auto node = pm->GetNode("throttle", true);
    node->setDoubleValue(0.8);
    TS_ASSERT_EQUALS(x.GetValue(), -0.8);
    TS_ASSERT(!x.IsLateBound());
  }

  /***************************************************************************
   * XML Constructor Tests - Real Values
   ***************************************************************************/

  void testXMLRealConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy> 1.2 </dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetValue(), 1.2);
    TS_ASSERT_EQUALS(x.GetName(), "constant value 1.200000");
  }

  void testXMLRealConstructorNegative() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>-99.5</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), -99.5);
  }

  void testXMLRealConstructorInteger() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>100</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 100.0);
  }

  void testXMLRealConstructorScientific() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>6.022e23</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 6.022e23);
  }

  /***************************************************************************
   * XML Constructor Tests - Property Values
   ***************************************************************************/

  void testXMLPropertyConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    Element_ptr elm = readFromXML("<dummy> x </dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetName(), "x");

    node->setDoubleValue(0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    node->setDoubleValue(1.2);
    TS_ASSERT_EQUALS(x.GetValue(), 1.2);
  }

  void testXMLPropertyConstructorNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("velocities/vc-kts", true);
    Element_ptr elm = readFromXML("<dummy>velocities/vc-kts</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(!x.IsConstant());

    node->setDoubleValue(250.0);
    TS_ASSERT_EQUALS(x.GetValue(), 250.0);
  }

  void testXMLPropertyConstructorSigned() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("beta", true);
    Element_ptr elm = readFromXML("<dummy>-beta</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(!x.IsConstant());

    node->setDoubleValue(5.0);
    TS_ASSERT_EQUALS(x.GetValue(), -5.0);
  }

  /***************************************************************************
   * XML Constructor Tests - Late Bound
   ***************************************************************************/

  void testXMLLateBoundPropertyConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy> x </dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetName(), "x");

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT(!x.IsLateBound());
    node->setDoubleValue(1.2);
    TS_ASSERT_EQUALS(x.GetValue(), 1.2);
  }

  void testXMLLateBoundPropertyIllegalAccess() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy> x </dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(x.IsLateBound());
    TS_ASSERT_EQUALS(x.GetName(), "x");
    TS_ASSERT_THROWS(x.GetValue(), BaseException&);
  }

  /***************************************************************************
   * XML Constructor Tests - Error Cases
   ***************************************************************************/

  void testXMLEmptyNameConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy/>");
    TS_ASSERT_THROWS(FGParameterValue x(elm, pm), BaseException&);
  }

  void testXMLMultiLinesConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>x\ny</dummy>");
    TS_ASSERT_THROWS(FGParameterValue x(elm, pm), BaseException&);
  }

  void testXMLWhitespaceOnly() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>   </dummy>");
    TS_ASSERT_THROWS(FGParameterValue x(elm, pm), BaseException&);
  }

  /***************************************************************************
   * Value Modification Tests
   ***************************************************************************/

  void testPropertyValueModification() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    for (int i = 0; i < 10; i++) {
      double val = static_cast<double>(i) * 1.5;
      node->setDoubleValue(val);
      TS_ASSERT_EQUALS(x.GetValue(), val);
    }
  }

  void testPropertyValueNegativeRange() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    node->setDoubleValue(-1000.0);
    TS_ASSERT_EQUALS(x.GetValue(), -1000.0);

    node->setDoubleValue(1000.0);
    TS_ASSERT_EQUALS(x.GetValue(), 1000.0);
  }

  void testConstantValueUnchanged() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("3.14159", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    double v1 = x.GetValue();
    double v2 = x.GetValue();
    double v3 = x.GetValue();

    TS_ASSERT_EQUALS(v1, 3.14159);
    TS_ASSERT_EQUALS(v2, 3.14159);
    TS_ASSERT_EQUALS(v3, 3.14159);
  }

  /***************************************************************************
   * Multiple Parameter Tests
   ***************************************************************************/

  void testMultipleParameters() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node_x = pm->GetNode("x", true);
    auto node_y = pm->GetNode("y", true);

    FGParameterValue px("x", pm, nullptr);
    FGParameterValue py("y", pm, nullptr);
    FGParameterValue pconst("100.0", pm, nullptr);

    node_x->setDoubleValue(10.0);
    node_y->setDoubleValue(20.0);

    TS_ASSERT_EQUALS(px.GetValue(), 10.0);
    TS_ASSERT_EQUALS(py.GetValue(), 20.0);
    TS_ASSERT_EQUALS(pconst.GetValue(), 100.0);

    TS_ASSERT(!px.IsConstant());
    TS_ASSERT(!py.IsConstant());
    TS_ASSERT(pconst.IsConstant());
  }

  void testIndependentPropertyManagers() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();

    auto node1 = pm1->GetNode("x", true);
    auto node2 = pm2->GetNode("x", true);

    FGParameterValue p1("x", pm1, nullptr);
    FGParameterValue p2("x", pm2, nullptr);

    node1->setDoubleValue(100.0);
    node2->setDoubleValue(200.0);

    TS_ASSERT_EQUALS(p1.GetValue(), 100.0);
    TS_ASSERT_EQUALS(p2.GetValue(), 200.0);
  }

  /***************************************************************************
   * Edge Case Value Tests
   ***************************************************************************/

  void testPropertyValueInfinity() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    double inf = std::numeric_limits<double>::infinity();
    node->setDoubleValue(inf);
    TS_ASSERT_EQUALS(x.GetValue(), inf);
  }

  void testPropertyValueNaN() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    double nan_val = std::nan("");
    node->setDoubleValue(nan_val);
    TS_ASSERT(std::isnan(x.GetValue()));
  }

  void testPropertyValueLarge() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    node->setDoubleValue(1e100);
    TS_ASSERT_EQUALS(x.GetValue(), 1e100);
  }

  void testPropertyValueSmall() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    node->setDoubleValue(1e-100);
    TS_ASSERT_EQUALS(x.GetValue(), 1e-100);
  }

  /***************************************************************************
   * GetName Tests
   ***************************************************************************/

  void testGetNameConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("42.5", pm, nullptr);

    TS_ASSERT(x.GetName().find("constant value") != std::string::npos);
    TS_ASSERT(x.GetName().find("42.5") != std::string::npos);
  }

  void testGetNameProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("aero/beta-rad", true);
    FGParameterValue x("aero/beta-rad", pm, nullptr);

    // GetName returns the leaf node name
    TS_ASSERT_EQUALS(x.GetName(), "beta-rad");
  }

  void testGetNameLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("nonexistent/prop", pm, nullptr);

    // Late bound properties return full path until resolved
    TS_ASSERT(x.GetName().find("prop") != std::string::npos);
    TS_ASSERT(x.IsLateBound());
  }

  /***************************************************************************
   * Pointer/Reference Tests
   ***************************************************************************/

  void testSharedPointerConstruction() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("x", true);
    SGSharedPtr<FGParameterValue> p(new FGParameterValue("x", pm, nullptr));

    TS_ASSERT(p.valid());
    TS_ASSERT(!p->IsConstant());
  }

  void testFGParameterPtrPolymorphism() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("x", true)->setDoubleValue(50.0);

    FGParameter_ptr p(new FGParameterValue("x", pm, nullptr));

    TS_ASSERT(p.valid());
    TS_ASSERT_EQUALS(p->GetValue(), 50.0);
  }

  /***************************************************************************
   * Additional Real Value Tests
   ***************************************************************************/

  // Test 41: Positive sign prefix
  void testRealConstructorPositiveSign() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("+3.5", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 3.5);
  }

  // Test 42: Scientific notation uppercase E
  void testRealConstructorScientificUpperE() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("2.5E8", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 2.5e8);
  }

  // Test 43: Scientific notation negative with uppercase E
  void testRealConstructorNegativeScientific() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("-1.0E-3", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), -1.0e-3);
  }

  // Test 44: Very large exponent
  void testRealConstructorLargeExponent() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("1e200", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 1e200);
  }

  // Test 45: Very small exponent
  void testRealConstructorSmallExponent() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("1e-200", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 1e-200);
  }

  /***************************************************************************
   * Property Path Variations Tests
   ***************************************************************************/

  // Test 46: Deeply nested property
  void testDeeplyNestedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("a/b/c/d/e", true);
    FGParameterValue x("a/b/c/d/e", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    node->setDoubleValue(42.0);
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  // Test 47: Property with index
  void testPropertyWithIndex() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("propulsion/engine[0]/thrust-lbs", true);
    FGParameterValue x("propulsion/engine[0]/thrust-lbs", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    node->setDoubleValue(5000.0);
    TS_ASSERT_EQUALS(x.GetValue(), 5000.0);
  }

  // Test 48: Property with underscore
  void testPropertyWithUnderscore() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("my_property_name", true);
    FGParameterValue x("my_property_name", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    node->setDoubleValue(100.0);
    TS_ASSERT_EQUALS(x.GetValue(), 100.0);
  }

  // Test 49: Property with dash
  void testPropertyWithDash() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("velocities/ve-fps", true);
    FGParameterValue x("velocities/ve-fps", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    node->setDoubleValue(500.0);
    TS_ASSERT_EQUALS(x.GetValue(), 500.0);
  }

  // Test 50: Single character property name
  void testSingleCharProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("q", true);
    FGParameterValue x("q", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    node->setDoubleValue(0.1);
    TS_ASSERT_EQUALS(x.GetValue(), 0.1);
  }

  /***************************************************************************
   * Sign Handling Tests
   ***************************************************************************/

  // Test 51: Negative sign on zero-valued property
  void testNegativeSignOnZero() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("-x", pm, nullptr);

    node->setDoubleValue(0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);  // -0.0 == 0.0
  }

  // Test 52: Negative sign on negative value
  void testNegativeSignOnNegative() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("-x", pm, nullptr);

    node->setDoubleValue(-5.0);
    TS_ASSERT_EQUALS(x.GetValue(), 5.0);  // Double negative
  }

  // Test 53: Negative sign on large value
  void testNegativeSignOnLarge() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("-x", pm, nullptr);

    node->setDoubleValue(1e50);
    TS_ASSERT_EQUALS(x.GetValue(), -1e50);
  }

  // Test 54: Nested property with negative sign
  void testNegativeNestedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("aero/qbar-psf", true);
    FGParameterValue x("-aero/qbar-psf", pm, nullptr);

    node->setDoubleValue(100.0);
    TS_ASSERT_EQUALS(x.GetValue(), -100.0);
  }

  /***************************************************************************
   * XML Value Tests
   ***************************************************************************/

  // Test 55: XML with leading whitespace
  void testXMLLeadingWhitespace() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>   5.5</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 5.5);
  }

  // Test 56: XML with trailing whitespace
  void testXMLTrailingWhitespace() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>7.7   </dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 7.7);
  }

  // Test 57: XML zero value
  void testXMLZeroValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>0</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
  }

  // Test 58: XML positive sign
  void testXMLPositiveSign() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>+42.0</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  // Test 59: XML signed property
  void testXMLSignedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("alpha", true);
    Element_ptr elm = readFromXML("<dummy>-alpha</dummy>");
    FGParameterValue x(elm, pm);

    TS_ASSERT(!x.IsConstant());
    node->setDoubleValue(10.0);
    TS_ASSERT_EQUALS(x.GetValue(), -10.0);
  }

  /***************************************************************************
   * Precision and Accuracy Tests
   ***************************************************************************/

  // Test 60: High precision constant
  void testHighPrecisionConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("3.14159265358979323846", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT_DELTA(x.GetValue(), M_PI, 1e-14);
  }

  // Test 61: Precision maintained in property
  void testPropertyPrecision() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    double precise = 1.23456789012345;
    node->setDoubleValue(precise);
    TS_ASSERT_DELTA(x.GetValue(), precise, 1e-14);
  }

  // Test 62: Small difference detection
  void testSmallDifferenceDetection() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    node->setDoubleValue(1.0);
    double v1 = x.GetValue();
    node->setDoubleValue(1.0 + 1e-15);
    double v2 = x.GetValue();

    TS_ASSERT(v2 > v1);
  }

  /***************************************************************************
   * Multiple Access Pattern Tests
   ***************************************************************************/

  // Test 63: Multiple accesses same value
  void testMultipleAccessesSameValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    node->setDoubleValue(42.0);
    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(x.GetValue(), 42.0);
    }
  }

  // Test 64: Interleaved property accesses
  void testInterleavedAccesses() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node_a = pm->GetNode("a", true);
    auto node_b = pm->GetNode("b", true);
    FGParameterValue pa("a", pm, nullptr);
    FGParameterValue pb("b", pm, nullptr);

    for (int i = 0; i < 10; i++) {
      node_a->setDoubleValue(static_cast<double>(i));
      node_b->setDoubleValue(static_cast<double>(i * 2));
      TS_ASSERT_EQUALS(pa.GetValue(), static_cast<double>(i));
      TS_ASSERT_EQUALS(pb.GetValue(), static_cast<double>(i * 2));
    }
  }

  // Test 65: Rapid value changes
  void testRapidValueChanges() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    for (int i = 0; i < 1000; i++) {
      double val = static_cast<double>(i) * 0.001;
      node->setDoubleValue(val);
      TS_ASSERT_EQUALS(x.GetValue(), val);
    }
  }

  /***************************************************************************
   * Special Value Handling Tests
   ***************************************************************************/

  // Test 66: Negative infinity
  void testNegativeInfinity() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    double neg_inf = -std::numeric_limits<double>::infinity();
    node->setDoubleValue(neg_inf);
    TS_ASSERT_EQUALS(x.GetValue(), neg_inf);
  }

  // Test 67: Denormalized value
  void testDenormalizedValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    double denorm = std::numeric_limits<double>::denorm_min();
    node->setDoubleValue(denorm);
    TS_ASSERT_EQUALS(x.GetValue(), denorm);
  }

  // Test 68: Max double value
  void testMaxDoubleValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("x", pm, nullptr);

    double max_val = std::numeric_limits<double>::max();
    node->setDoubleValue(max_val);
    TS_ASSERT_EQUALS(x.GetValue(), max_val);
  }

  // Test 69: Negated infinity via sign
  void testNegatedInfinity() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("x", true);
    FGParameterValue x("-x", pm, nullptr);

    double inf = std::numeric_limits<double>::infinity();
    node->setDoubleValue(inf);
    TS_ASSERT_EQUALS(x.GetValue(), -inf);
  }

  /***************************************************************************
   * IsConstant and IsLateBound Interaction Tests
   ***************************************************************************/

  // Test 70: Constant is never late bound
  void testConstantNeverLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("123.456", pm, nullptr);

    TS_ASSERT(x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
  }

  // Test 71: Resolved property is not late bound
  void testResolvedPropertyNotLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("y", true);
    FGParameterValue x("y", pm, nullptr);

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT(!x.IsLateBound());
  }

  // Test 72: Late bound becomes resolved
  void testLateBoundBecomesResolved() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("z", pm, nullptr);

    TS_ASSERT(x.IsLateBound());

    auto node = pm->GetNode("z", true);
    node->setDoubleValue(100.0);
    x.GetValue();  // This resolves the late binding

    TS_ASSERT(!x.IsLateBound());
  }

  /***************************************************************************
   * Name Variations Tests
   ***************************************************************************/

  // Test 73: Property name with multiple indexes
  void testPropertyMultipleIndexes() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("systems/electrical/bus[0]/load[1]", true);
    FGParameterValue x("systems/electrical/bus[0]/load[1]", pm, nullptr);

    node->setDoubleValue(25.0);
    TS_ASSERT_EQUALS(x.GetValue(), 25.0);
  }

  // Test 74: GetName for signed property
  void testGetNameSignedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("throttle", true);
    FGParameterValue x("-throttle", pm, nullptr);

    // The name should not include the sign
    TS_ASSERT(x.GetName().find("throttle") != std::string::npos);
  }

  // Test 75: Very long property path
  void testVeryLongPropertyPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    std::string longPath = "a/b/c/d/e/f/g/h/i/j/k/l/m/n/o/p";
    auto node = pm->GetNode(longPath, true);
    FGParameterValue x(longPath, pm, nullptr);

    node->setDoubleValue(99.9);
    TS_ASSERT_EQUALS(x.GetValue(), 99.9);
  }

  /***************************************************************************
   * Extended Tests (76-100)
   ***************************************************************************/

  void testConstantEpsilon() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("2.220446049250313e-16", pm, nullptr);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT_DELTA(x.GetValue(), std::numeric_limits<double>::epsilon(), 1e-30);
  }

  void testConstantMinPositive() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("2.2250738585072014e-308", pm, nullptr);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT(x.GetValue() > 0.0);
  }

  void testPropertyRapidChanges() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("rapid", true);
    FGParameterValue x("rapid", pm, nullptr);
    for (int i = 0; i < 1000; i++) {
      double val = std::sin(static_cast<double>(i) * 0.01);
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(x.GetValue(), val, 1e-15);
    }
  }

  void testSignedPropertyWithScientificPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("aero/coefficient1e2", true);
    FGParameterValue x("-aero/coefficient1e2", pm, nullptr);
    node->setDoubleValue(50.0);
    TS_ASSERT_EQUALS(x.GetValue(), -50.0);
  }

  void testXMLPropertyWithSign() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("gamma", true);
    Element_ptr elm = readFromXML("<dummy>-gamma</dummy>");
    FGParameterValue x(elm, pm);
    node->setDoubleValue(1.4);
    TS_ASSERT_EQUALS(x.GetValue(), -1.4);
  }

  void testLateBoundMultipleResolve() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x1("shared/val", pm, nullptr);
    FGParameterValue x2("shared/val", pm, nullptr);
    TS_ASSERT(x1.IsLateBound());
    TS_ASSERT(x2.IsLateBound());
    auto node = pm->GetNode("shared/val", true);
    node->setDoubleValue(42.0);
    TS_ASSERT_EQUALS(x1.GetValue(), 42.0);
    TS_ASSERT_EQUALS(x2.GetValue(), 42.0);
    TS_ASSERT(!x1.IsLateBound());
    TS_ASSERT(!x2.IsLateBound());
  }

  void testConstantNegativeZero() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("-0.0", pm, nullptr);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
  }

  void testPropertyAlternatingSign() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("alt", true);
    FGParameterValue x("-alt", pm, nullptr);
    for (int i = 0; i < 10; i++) {
      double val = (i % 2 == 0) ? 100.0 : -100.0;
      node->setDoubleValue(val);
      TS_ASSERT_EQUALS(x.GetValue(), -val);
    }
  }

  void testXMLScientificNegativeExponent() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>5.67e-8</dummy>");
    FGParameterValue x(elm, pm);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT_DELTA(x.GetValue(), 5.67e-8, 1e-15);
  }

  void testPropertySubsequentAccess() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("subseq", true);
    FGParameterValue x("subseq", pm, nullptr);
    node->setDoubleValue(1.0);
    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(x.GetValue(), 1.0);
    }
  }

  void testGetNameForConstantWithManyDigits() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("123.456789012345", pm, nullptr);
    TS_ASSERT(x.GetName().find("constant") != std::string::npos);
  }

  void testSignedPropertyInfinityNegation() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("inf_test", true);
    FGParameterValue x("-inf_test", pm, nullptr);
    node->setDoubleValue(-std::numeric_limits<double>::infinity());
    TS_ASSERT_EQUALS(x.GetValue(), std::numeric_limits<double>::infinity());
  }

  void testMultiplePropertyManagers() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();
    auto pm3 = std::make_shared<FGPropertyManager>();
    pm1->GetNode("x", true)->setDoubleValue(1.0);
    pm2->GetNode("x", true)->setDoubleValue(2.0);
    pm3->GetNode("x", true)->setDoubleValue(3.0);
    FGParameterValue p1("x", pm1, nullptr);
    FGParameterValue p2("x", pm2, nullptr);
    FGParameterValue p3("x", pm3, nullptr);
    TS_ASSERT_EQUALS(p1.GetValue(), 1.0);
    TS_ASSERT_EQUALS(p2.GetValue(), 2.0);
    TS_ASSERT_EQUALS(p3.GetValue(), 3.0);
  }

  void testConstantLargeNegative() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("-1.7976931348623157e+308", pm, nullptr);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT(x.GetValue() < -1e307);
  }

  void testPropertyNameWithNumbers() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("prop123/value456", true);
    FGParameterValue x("prop123/value456", pm, nullptr);
    node->setDoubleValue(789.0);
    TS_ASSERT_EQUALS(x.GetValue(), 789.0);
  }

  void testXMLLateBoundNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>deep/nested/path/value</dummy>");
    FGParameterValue x(elm, pm);
    TS_ASSERT(x.IsLateBound());
    auto node = pm->GetNode("deep/nested/path/value", true);
    node->setDoubleValue(999.0);
    TS_ASSERT_EQUALS(x.GetValue(), 999.0);
    TS_ASSERT(!x.IsLateBound());
  }

  void testPropertyDenormalized() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("denorm", true);
    FGParameterValue x("denorm", pm, nullptr);
    double denorm = std::numeric_limits<double>::denorm_min();
    node->setDoubleValue(denorm);
    TS_ASSERT_EQUALS(x.GetValue(), denorm);
  }

  void testConstantPiApproximation() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("3.141592653589793", pm, nullptr);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT_DELTA(x.GetValue(), M_PI, 1e-15);
  }

  void testSignedLateBoundResolution() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue x("-future/prop", pm, nullptr);
    TS_ASSERT(x.IsLateBound());
    auto node = pm->GetNode("future/prop", true);
    node->setDoubleValue(25.0);
    TS_ASSERT_EQUALS(x.GetValue(), -25.0);
    TS_ASSERT(!x.IsLateBound());
  }

  void testMultipleConstantsIndependent() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGParameterValue c1("100.0", pm, nullptr);
    FGParameterValue c2("200.0", pm, nullptr);
    FGParameterValue c3("300.0", pm, nullptr);
    TS_ASSERT_EQUALS(c1.GetValue(), 100.0);
    TS_ASSERT_EQUALS(c2.GetValue(), 200.0);
    TS_ASSERT_EQUALS(c3.GetValue(), 300.0);
  }

  void testPropertyValueBoundaryPositive() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("boundary", true);
    FGParameterValue x("boundary", pm, nullptr);
    node->setDoubleValue(std::numeric_limits<double>::max());
    TS_ASSERT_EQUALS(x.GetValue(), std::numeric_limits<double>::max());
  }

  void testPropertyValueBoundaryNegative() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("boundary_neg", true);
    FGParameterValue x("boundary_neg", pm, nullptr);
    node->setDoubleValue(std::numeric_limits<double>::lowest());
    TS_ASSERT_EQUALS(x.GetValue(), std::numeric_limits<double>::lowest());
  }

  void testXMLConstantWithSpaces() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>  123.456  </dummy>");
    FGParameterValue x(elm, pm);
    TS_ASSERT(x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 123.456);
  }

  void testPropertyPathWithArrayIndex() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("array[0]/value", true);
    FGParameterValue x("array[0]/value", pm, nullptr);
    node->setDoubleValue(42.0);
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  void testCompleteParameterValueStressTest() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Create various types of FGParameterValue
    FGParameterValue constant("3.14159", pm, nullptr);
    TS_ASSERT(constant.IsConstant());
    TS_ASSERT_DELTA(constant.GetValue(), 3.14159, 1e-10);

    auto node = pm->GetNode("stress/test", true);
    FGParameterValue property("stress/test", pm, nullptr);
    FGParameterValue signedProp("-stress/test", pm, nullptr);

    // Stress test with many value changes
    for (int i = 0; i < 100; i++) {
      double val = std::sin(i * 0.1) * 100.0;
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(property.GetValue(), val, 1e-10);
      TS_ASSERT_DELTA(signedProp.GetValue(), -val, 1e-10);
    }
  }
};
