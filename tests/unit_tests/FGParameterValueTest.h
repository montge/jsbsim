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
};
