#include <cxxtest/TestSuite.h>
#include <math/FGPropertyValue.h>
#include <limits>
#include <cmath>

using namespace JSBSim;

class FGPropertyValueTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Constructor Tests - From Node
   ***************************************************************************/

  void testConstructorFromNode() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetValue(), 0.0);
    TS_ASSERT_EQUALS(property.IsConstant(), false);
    TS_ASSERT_EQUALS(property.IsLateBound(), false);
    TS_ASSERT_EQUALS(property.GetName(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetNameWithSign(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), std::string("/x"));
    TS_ASSERT_EQUALS(property.GetPrintableName(), std::string("x"));
  }

  void testConstructorFromNodeNested() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("level1/level2/prop", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), std::string("prop"));
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), std::string("/level1/level2/prop"));
  }

  void testConstructorFromNodeWithIndex() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("array", 0, true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetValue(), 0.0);
    TS_ASSERT_EQUALS(property.IsLateBound(), false);
  }

  /***************************************************************************
   * SetValue Tests
   ***************************************************************************/

  void testSetValue() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(node->getDoubleValue(), 0.0);
    property.SetValue(1.54);
    TS_ASSERT_EQUALS(property.GetValue(), 1.54);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 1.54);
  }

  void testSetValueNegative() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    property.SetValue(-999.5);
    TS_ASSERT_EQUALS(property.GetValue(), -999.5);
    TS_ASSERT_EQUALS(node->getDoubleValue(), -999.5);
  }

  void testSetValueZero() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    property.SetValue(100.0);
    TS_ASSERT_EQUALS(property.GetValue(), 100.0);

    property.SetValue(0.0);
    TS_ASSERT_EQUALS(property.GetValue(), 0.0);
  }

  void testSetValueMultipleTimes() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    for (int i = 0; i < 10; i++) {
      property.SetValue(static_cast<double>(i));
      TS_ASSERT_EQUALS(property.GetValue(), static_cast<double>(i));
    }
  }

  void testSetValueLargeNumber() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    property.SetValue(1e15);
    TS_ASSERT_EQUALS(property.GetValue(), 1e15);
  }

  void testSetValueSmallNumber() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    property.SetValue(1e-15);
    TS_ASSERT_DELTA(property.GetValue(), 1e-15, 1e-20);
  }

  /***************************************************************************
   * SetNode Tests
   ***************************************************************************/

  void testSetNode() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_x = root.getNode("x", true);
    SGPropertyNode_ptr node_y = root.getNode("y", true);
    FGPropertyValue property(node_x);

    node_y->setDoubleValue(-1.547);
    TS_ASSERT_EQUALS(property.GetValue(), 0.0);
    TS_ASSERT_EQUALS(property.GetName(), "x");
    property.SetNode(node_y);
    TS_ASSERT_EQUALS(property.GetValue(), -1.547);
    TS_ASSERT_EQUALS(property.GetName(), "y");
  }

  void testSetNodeUpdatesFullyQualifiedName() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_a = root.getNode("path/a", true);
    SGPropertyNode_ptr node_b = root.getNode("other/b", true);
    FGPropertyValue property(node_a);

    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/path/a");

    property.SetNode(node_b);
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/other/b");
  }

  void testSetNodePreservesSign() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    SGPropertyNode_ptr node_y = pm->GetNode("y", true);
    node_y->setDoubleValue(10.0);

    // SetNode replaces the node entirely but sign is still applied
    property.SetNode(node_y);
    // Sign is preserved after SetNode
    TS_ASSERT_EQUALS(property.GetValue(), -10.0);
  }

  /***************************************************************************
   * IsConstant Tests
   ***************************************************************************/

  void testConstant_ness() {
    auto pm = std::make_shared<FGPropertyManager>();
    SGPropertyNode_ptr node = pm->GetNode("x", true);
    FGPropertyValue property(node);

    TS_ASSERT(!property.IsConstant());
    node->setAttribute(SGPropertyNode::WRITE, false);
    TS_ASSERT(property.IsConstant());
  }

  void testTiedPropertiesAreNotConstant() {
    // Check that tied properties are not constant even if the underlying
    // property is set to READ ONLY.
    auto pm = std::make_shared<FGPropertyManager>();
    double value = 0.0;
    SGPropertyNode_ptr node = pm->GetNode("x", true);
    FGPropertyValue property(node);

    node->setAttribute(SGPropertyNode::WRITE, false);

    pm->Tie("x", &value);
    TS_ASSERT(!node->getAttribute(SGPropertyNode::WRITE)); // READ ONLY
    TS_ASSERT(!property.IsConstant()); // but not constant.

    // Since the property is declared READ ONLY, calls to
    // SGPropertyNode::setDoubleValue are ignored.
    node->setDoubleValue(1.0);
    TS_ASSERT_EQUALS(property.GetValue(), 0.0);

    // However FGPropertyValue can be modified by altering the variable which
    // it is tied to.
    value = 1.0;
    TS_ASSERT_EQUALS(property.GetValue(), 1.0);
    // And as soon as the property is untied, the FGProperty instance can be
    // made constant again.
    pm->Untie("x");
    node->setAttribute(SGPropertyNode::WRITE, false);
    TS_ASSERT(property.IsConstant());
  }

  void testIsConstantDefaultFalse() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    // By default, properties are not constant (writable)
    TS_ASSERT_EQUALS(property.IsConstant(), false);
  }

  void testIsConstantAfterValueChange() {
    auto pm = std::make_shared<FGPropertyManager>();
    SGPropertyNode_ptr node = pm->GetNode("x", true);
    FGPropertyValue property(node);

    property.SetValue(100.0);
    TS_ASSERT_EQUALS(property.IsConstant(), false);

    node->setAttribute(SGPropertyNode::WRITE, false);
    TS_ASSERT_EQUALS(property.IsConstant(), true);
  }

  /***************************************************************************
   * Late Bound Constructor Tests
   ***************************************************************************/

  void testConstructorLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("x", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);
    TS_ASSERT_EQUALS(property.GetName(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetNameWithSign(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetPrintableName(), std::string("x"));
    TS_ASSERT_EQUALS(property.IsConstant(), false);
    TS_ASSERT_EQUALS(property.IsLateBound(), true);
    // The property manager does not contain the property "x" so GetValue()
    // should throw an exception.
    TS_ASSERT_THROWS(property.GetValue(), BaseException&);
  }

  void testLateBoundWithNestedPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("systems/fcs/aileron-cmd", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);
    TS_ASSERT_EQUALS(property.GetName(), std::string("systems/fcs/aileron-cmd"));
  }

  void testLateBoundThrowsOnSetValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("nonexistent", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);
    // SetValue on late-bound property that doesn't exist should throw
    TS_ASSERT_THROWS(property.SetValue(1.0), BaseException&);
  }

  /***************************************************************************
   * Late Bound Instantiation Tests
   ***************************************************************************/

  void testInstantiateLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("x", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);

    auto node = pm->GetNode("x", true);
    TS_ASSERT_EQUALS(property.GetValue(), 0.0);
    TS_ASSERT_EQUALS(property.IsLateBound(), false);
    TS_ASSERT_EQUALS(property.GetName(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetNameWithSign(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), std::string("/x"));
    TS_ASSERT_EQUALS(property.GetPrintableName(), std::string("x"));

    // Check the link is two-way.
    node->setDoubleValue(1.3574);
    TS_ASSERT_EQUALS(property.GetValue(), 1.3574);

    property.SetValue(-2.01);
    TS_ASSERT_EQUALS(node->getDoubleValue(), -2.01);
  }

  void testInstantiateLateBoundNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("aero/alpha-deg", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);

    auto node = pm->GetNode("aero/alpha-deg", true);
    node->setDoubleValue(5.5);

    TS_ASSERT_EQUALS(property.GetValue(), 5.5);
    TS_ASSERT_EQUALS(property.IsLateBound(), false);
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), std::string("/aero/alpha-deg"));
  }

  void testInstantiateLateBoundThenModify() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("test/prop", pm, nullptr);

    auto node = pm->GetNode("test/prop", true);
    // Access the property to trigger late binding resolution
    double val = property.GetValue();
    TS_ASSERT_EQUALS(val, 0.0);
    TS_ASSERT_EQUALS(property.IsLateBound(), false);

    // Should be able to set value after instantiation
    property.SetValue(42.0);
    TS_ASSERT_EQUALS(property.GetValue(), 42.0);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 42.0);
  }

  /***************************************************************************
   * Signed Property Tests
   ***************************************************************************/

  void testSignedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);
    TS_ASSERT_EQUALS(property.GetName(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetNameWithSign(), std::string("-x"));
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), std::string("x"));
    TS_ASSERT_EQUALS(property.GetPrintableName(), std::string("x"));
    TS_ASSERT_EQUALS(property.IsConstant(), false);
    TS_ASSERT_EQUALS(property.IsLateBound(), true);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(1.234);
    TS_ASSERT_EQUALS(property.GetValue(), -1.234);
  }

  void testSignedPropertyZeroValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(0.0);
    TS_ASSERT_EQUALS(property.GetValue(), 0.0);  // -0.0 == 0.0
  }

  void testSignedPropertyNegativeValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(-5.0);
    TS_ASSERT_EQUALS(property.GetValue(), 5.0);  // -(-5) = 5
  }

  void testSignedPropertyLargeValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(1e10);
    TS_ASSERT_EQUALS(property.GetValue(), -1e10);
  }

  void testUnsignedProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(100.0);
    TS_ASSERT_EQUALS(property.GetValue(), 100.0);  // No sign inversion
    TS_ASSERT_EQUALS(property.GetNameWithSign(), std::string("x"));
  }

  /***************************************************************************
   * GetName Variations Tests
   ***************************************************************************/

  void testGetNameSimple() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("simple", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "simple");
  }

  void testGetNameNested() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("a/b/c/d", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "d");
  }

  void testGetFullyQualifiedNameRoot() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("prop", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/prop");
  }

  void testGetFullyQualifiedNameDeep() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("level1/level2/level3/prop", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/level1/level2/level3/prop");
  }

  void testGetPrintableNameSimple() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("test-prop", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetPrintableName(), "test-prop");
  }

  /***************************************************************************
   * Two-Way Binding Tests
   ***************************************************************************/

  void testTwoWayBindingNodeToProperty() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    node->setDoubleValue(42.0);
    TS_ASSERT_EQUALS(property.GetValue(), 42.0);

    node->setDoubleValue(-100.5);
    TS_ASSERT_EQUALS(property.GetValue(), -100.5);
  }

  void testTwoWayBindingPropertyToNode() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    property.SetValue(99.9);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 99.9);

    property.SetValue(0.001);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 0.001);
  }

  void testTwoWayBindingMultipleProperties() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("shared", true);
    FGPropertyValue prop1(node);
    FGPropertyValue prop2(node);

    prop1.SetValue(50.0);
    TS_ASSERT_EQUALS(prop2.GetValue(), 50.0);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 50.0);

    prop2.SetValue(75.0);
    TS_ASSERT_EQUALS(prop1.GetValue(), 75.0);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 75.0);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testPropertyValueInfinity() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    double inf = std::numeric_limits<double>::infinity();
    property.SetValue(inf);
    TS_ASSERT_EQUALS(property.GetValue(), inf);
  }

  void testPropertyValueNegativeInfinity() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    double neg_inf = -std::numeric_limits<double>::infinity();
    property.SetValue(neg_inf);
    TS_ASSERT_EQUALS(property.GetValue(), neg_inf);
  }

  void testPropertyValueNaN() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    double nan_val = std::nan("");
    property.SetValue(nan_val);
    TS_ASSERT(std::isnan(property.GetValue()));
  }

  void testPropertyValueMaxDouble() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    double max_val = std::numeric_limits<double>::max();
    property.SetValue(max_val);
    TS_ASSERT_EQUALS(property.GetValue(), max_val);
  }

  void testPropertyValueMinDouble() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    double min_val = std::numeric_limits<double>::min();
    property.SetValue(min_val);
    TS_ASSERT_EQUALS(property.GetValue(), min_val);
  }

  /***************************************************************************
   * Multiple Property Manager Tests
   ***************************************************************************/

  void testDifferentPropertyManagers() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();

    FGPropertyValue prop1("x", pm1, nullptr);
    FGPropertyValue prop2("x", pm2, nullptr);

    pm1->GetNode("x", true)->setDoubleValue(100.0);
    pm2->GetNode("x", true)->setDoubleValue(200.0);

    TS_ASSERT_EQUALS(prop1.GetValue(), 100.0);
    TS_ASSERT_EQUALS(prop2.GetValue(), 200.0);
  }

  /***************************************************************************
   * GetValue Consistency Tests
   ***************************************************************************/

  void testGetValueConsistentWithGetDoubleValue() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    node->setDoubleValue(3.14159);
    TS_ASSERT_EQUALS(property.GetValue(), node->getDoubleValue());
  }

  void testGetValueAfterMultipleModifications() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    for (int i = 0; i < 100; i++) {
      double val = static_cast<double>(i) * 1.5;
      node->setDoubleValue(val);
      TS_ASSERT_EQUALS(property.GetValue(), val);
    }
  }
};
