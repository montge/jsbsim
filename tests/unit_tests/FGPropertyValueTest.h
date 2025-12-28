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

/*******************************************************************************
 * Additional FGPropertyValue Tests (32 new tests)
 ******************************************************************************/

class FGPropertyValueAdditionalTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Signed Property Edge Cases
   ***************************************************************************/

  // Test 44: Signed property with infinity
  void testSignedPropertyInfinity() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    double inf = std::numeric_limits<double>::infinity();
    node->setDoubleValue(inf);
    TS_ASSERT_EQUALS(property.GetValue(), -inf);
  }

  // Test 45: Signed property with negative infinity
  void testSignedPropertyNegativeInfinity() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    double neg_inf = -std::numeric_limits<double>::infinity();
    node->setDoubleValue(neg_inf);
    TS_ASSERT_EQUALS(property.GetValue(), std::numeric_limits<double>::infinity());
  }

  // Test 46: Signed property with NaN (NaN sign behavior)
  void testSignedPropertyNaN() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(std::nan(""));
    // -NaN is still NaN
    TS_ASSERT(std::isnan(property.GetValue()));
  }

  // Test 47: Signed property with denormalized value
  void testSignedPropertyDenormalized() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    double denorm = std::numeric_limits<double>::denorm_min();
    node->setDoubleValue(denorm);
    TS_ASSERT_EQUALS(property.GetValue(), -denorm);
  }

  /***************************************************************************
   * Property Path Patterns
   ***************************************************************************/

  // Test 48: Property with numeric suffix
  void testPropertyWithNumericSuffix() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("engine0/rpm", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "rpm");
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/engine0/rpm");
  }

  // Test 49: Property with hyphenated name
  void testPropertyWithHyphenatedName() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("fuel-flow-rate", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "fuel-flow-rate");
  }

  // Test 50: Property with underscore name
  void testPropertyWithUnderscoreName() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("pitch_rate_rad", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "pitch_rate_rad");
  }

  // Test 51: Deeply nested property path
  void testDeeplyNestedPropertyPath() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("a/b/c/d/e/f/g/h", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "h");
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/a/b/c/d/e/f/g/h");
  }

  // Test 52: Property with array index
  void testPropertyWithArrayIndex() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("tank", 2, true);
    FGPropertyValue property(node);

    node->setDoubleValue(1000.0);
    TS_ASSERT_EQUALS(property.GetValue(), 1000.0);
  }

  /***************************************************************************
   * Late Binding Edge Cases
   ***************************************************************************/

  // Test 53: Late bound property with nested path containing hyphen
  void testLateBoundNestedWithHyphen() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("fcs/aileron-cmd-norm", pm, nullptr);

    TS_ASSERT_EQUALS(property.IsLateBound(), true);

    auto node = pm->GetNode("fcs/aileron-cmd-norm", true);
    node->setDoubleValue(0.75);

    TS_ASSERT_EQUALS(property.GetValue(), 0.75);
    TS_ASSERT_EQUALS(property.IsLateBound(), false);
  }

  // Test 54: Late bound signed property with nested path
  void testLateBoundSignedNestedPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-aero/beta-rad", pm, nullptr);

    auto node = pm->GetNode("aero/beta-rad", true);
    node->setDoubleValue(0.05);

    TS_ASSERT_EQUALS(property.GetValue(), -0.05);
    // GetName returns the leaf node name after binding
    TS_ASSERT_EQUALS(property.GetName(), "beta-rad");
    TS_ASSERT_EQUALS(property.GetNameWithSign(), "-beta-rad");
  }

  // Test 55: Multiple late bound properties to same path
  void testMultipleLateBoundSamePath() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue prop1("shared/value", pm, nullptr);
    FGPropertyValue prop2("shared/value", pm, nullptr);

    TS_ASSERT(prop1.IsLateBound());
    TS_ASSERT(prop2.IsLateBound());

    auto node = pm->GetNode("shared/value", true);
    node->setDoubleValue(42.0);

    TS_ASSERT_EQUALS(prop1.GetValue(), 42.0);
    TS_ASSERT_EQUALS(prop2.GetValue(), 42.0);
    TS_ASSERT(!prop1.IsLateBound());
    TS_ASSERT(!prop2.IsLateBound());
  }

  // Test 56: Late bound property resolution order
  void testLateBoundResolutionOrder() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("test/order", pm, nullptr);

    // Create the node
    auto node = pm->GetNode("test/order", true);
    node->setDoubleValue(100.0);

    // First access triggers binding
    TS_ASSERT_EQUALS(property.GetValue(), 100.0);

    // Subsequent modification via node
    node->setDoubleValue(200.0);
    TS_ASSERT_EQUALS(property.GetValue(), 200.0);

    // Modification via property
    property.SetValue(300.0);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 300.0);
  }

  /***************************************************************************
   * SetNode Advanced Tests
   ***************************************************************************/

  // Test 57: SetNode multiple times
  void testSetNodeMultipleTimes() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_a = root.getNode("a", true);
    SGPropertyNode_ptr node_b = root.getNode("b", true);
    SGPropertyNode_ptr node_c = root.getNode("c", true);
    FGPropertyValue property(node_a);

    node_a->setDoubleValue(1.0);
    node_b->setDoubleValue(2.0);
    node_c->setDoubleValue(3.0);

    TS_ASSERT_EQUALS(property.GetValue(), 1.0);

    property.SetNode(node_b);
    TS_ASSERT_EQUALS(property.GetValue(), 2.0);

    property.SetNode(node_c);
    TS_ASSERT_EQUALS(property.GetValue(), 3.0);

    property.SetNode(node_a);
    TS_ASSERT_EQUALS(property.GetValue(), 1.0);
  }

  // Test 58: SetNode with different paths
  void testSetNodeDifferentPaths() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_deep = root.getNode("deep/path/prop", true);
    SGPropertyNode_ptr node_shallow = root.getNode("shallow", true);
    FGPropertyValue property(node_deep);

    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/deep/path/prop");

    property.SetNode(node_shallow);
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/shallow");
  }

  /***************************************************************************
   * IsConstant Advanced Tests
   ***************************************************************************/

  // Test 59: Toggle constant status multiple times
  void testToggleConstantMultipleTimes() {
    auto pm = std::make_shared<FGPropertyManager>();
    SGPropertyNode_ptr node = pm->GetNode("toggle", true);
    FGPropertyValue property(node);

    for (int i = 0; i < 5; i++) {
      node->setAttribute(SGPropertyNode::WRITE, false);
      TS_ASSERT(property.IsConstant());

      node->setAttribute(SGPropertyNode::WRITE, true);
      TS_ASSERT(!property.IsConstant());
    }
  }

  // Test 60: Constant property value retrieval
  void testConstantPropertyValueRetrieval() {
    auto pm = std::make_shared<FGPropertyManager>();
    SGPropertyNode_ptr node = pm->GetNode("const-test", true);
    node->setDoubleValue(3.14159);
    node->setAttribute(SGPropertyNode::WRITE, false);
    FGPropertyValue property(node);

    TS_ASSERT(property.IsConstant());
    TS_ASSERT_EQUALS(property.GetValue(), 3.14159);
    // GetValue should work even when constant
    TS_ASSERT_EQUALS(property.GetValue(), 3.14159);
  }

  /***************************************************************************
   * Value Precision Tests
   ***************************************************************************/

  // Test 61: Very small difference detection
  void testVerySmallDifferenceDetection() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("precise", true);
    FGPropertyValue property(node);

    double val1 = 1.0;
    double val2 = 1.0 + 1e-15;

    property.SetValue(val1);
    double retrieved1 = property.GetValue();

    property.SetValue(val2);
    double retrieved2 = property.GetValue();

    // Should be able to detect small differences
    TS_ASSERT(retrieved1 != retrieved2 || val1 == val2);
  }

  // Test 62: Epsilon neighborhood values
  void testEpsilonNeighborhoodValues() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("epsilon", true);
    FGPropertyValue property(node);

    double epsilon = std::numeric_limits<double>::epsilon();
    property.SetValue(1.0 + epsilon);
    TS_ASSERT_DELTA(property.GetValue(), 1.0, 2.0 * epsilon);
  }

  // Test 63: Mantissa precision test
  void testMantissaPrecision() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("mantissa", true);
    FGPropertyValue property(node);

    // Value that exercises mantissa precision
    double precise = 1.234567890123456789;
    property.SetValue(precise);
    // IEEE 754 double has ~15-17 significant digits
    TS_ASSERT_DELTA(property.GetValue(), precise, 1e-15);
  }

  /***************************************************************************
   * Multiple Property Interaction Tests
   ***************************************************************************/

  // Test 64: Many properties sharing same node
  void testManyPropertiesSharingNode() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("shared", true);

    std::vector<FGPropertyValue> properties;
    for (int i = 0; i < 10; i++) {
      properties.emplace_back(node);
    }

    node->setDoubleValue(999.0);

    for (const auto& prop : properties) {
      TS_ASSERT_EQUALS(prop.GetValue(), 999.0);
    }
  }

  // Test 65: Properties in different managers with same name
  void testPropertiesDifferentManagersSameName() {
    std::vector<std::shared_ptr<FGPropertyManager>> managers;
    std::vector<FGPropertyValue*> properties;

    for (int i = 0; i < 5; i++) {
      managers.push_back(std::make_shared<FGPropertyManager>());
      properties.push_back(new FGPropertyValue("x", managers[i], nullptr));
      managers[i]->GetNode("x", true)->setDoubleValue(static_cast<double>(i * 10));
    }

    for (int i = 0; i < 5; i++) {
      TS_ASSERT_EQUALS(properties[i]->GetValue(), static_cast<double>(i * 10));
      delete properties[i];
    }
  }

  /***************************************************************************
   * Name Pattern Tests
   ***************************************************************************/

  // Test 66: Property name with many segments
  void testPropertyNameManySegments() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("seg1/seg2/seg3/seg4/seg5", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "seg5");
    TS_ASSERT(property.GetFullyQualifiedName().find("seg1") != std::string::npos);
    TS_ASSERT(property.GetFullyQualifiedName().find("seg5") != std::string::npos);
  }

  // Test 67: Property name with numbers
  void testPropertyNameWithNumbers() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("engine123/prop456", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "prop456");
  }

  // Test 68: Single character property name
  void testSingleCharacterPropertyName() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("x", true);
    FGPropertyValue property(node);

    TS_ASSERT_EQUALS(property.GetName(), "x");
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/x");
  }

  /***************************************************************************
   * Value Range Tests
   ***************************************************************************/

  // Test 69: Alternating positive and negative values
  void testAlternatingPositiveNegative() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("alt", true);
    FGPropertyValue property(node);

    for (int i = 0; i < 20; i++) {
      double val = (i % 2 == 0) ? 100.0 : -100.0;
      property.SetValue(val);
      TS_ASSERT_EQUALS(property.GetValue(), val);
    }
  }

  // Test 70: Exponential growth values
  void testExponentialGrowthValues() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("exp", true);
    FGPropertyValue property(node);

    double val = 1.0;
    for (int i = 0; i < 50; i++) {
      property.SetValue(val);
      TS_ASSERT_EQUALS(property.GetValue(), val);
      val *= 10.0;
    }
  }

  // Test 71: Exponential decay values
  void testExponentialDecayValues() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("decay", true);
    FGPropertyValue property(node);

    double val = 1.0;
    for (int i = 0; i < 50; i++) {
      property.SetValue(val);
      TS_ASSERT_DELTA(property.GetValue(), val, std::abs(val) * 1e-14 + 1e-300);
      val /= 10.0;
    }
  }

  /***************************************************************************
   * Signed Property Comprehensive Tests
   ***************************************************************************/

  // Test 72: Signed property with max double
  void testSignedPropertyMaxDouble() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    double max_val = std::numeric_limits<double>::max();
    node->setDoubleValue(max_val);
    TS_ASSERT_EQUALS(property.GetValue(), -max_val);
  }

  // Test 73: Signed property name variations
  void testSignedPropertyNameVariations() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue prop1("-simple", pm, nullptr);
    FGPropertyValue prop2("-path/to/prop", pm, nullptr);

    TS_ASSERT_EQUALS(prop1.GetName(), "simple");
    TS_ASSERT_EQUALS(prop1.GetNameWithSign(), "-simple");
    TS_ASSERT_EQUALS(prop2.GetName(), "path/to/prop");
    TS_ASSERT_EQUALS(prop2.GetNameWithSign(), "-path/to/prop");
  }

  // Test 74: Signed property value propagation
  void testSignedPropertyValuePropagation() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue signedProp("-x", pm, nullptr);
    FGPropertyValue unsignedProp("x", pm, nullptr);

    auto node = pm->GetNode("x", true);
    node->setDoubleValue(50.0);

    // Both properties read from same node but signed inverts
    TS_ASSERT_EQUALS(signedProp.GetValue(), -50.0);
    TS_ASSERT_EQUALS(unsignedProp.GetValue(), 50.0);
  }

  // Test 75: Property binding lifecycle
  void testPropertyBindingLifecycle() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Create late-bound property
    FGPropertyValue property("lifecycle/test", pm, nullptr);
    TS_ASSERT(property.IsLateBound());

    // Create node and verify binding
    auto node = pm->GetNode("lifecycle/test", true);
    node->setDoubleValue(1.0);
    TS_ASSERT_EQUALS(property.GetValue(), 1.0);
    TS_ASSERT(!property.IsLateBound());

    // Modify through various paths
    property.SetValue(2.0);
    TS_ASSERT_EQUALS(node->getDoubleValue(), 2.0);

    node->setDoubleValue(3.0);
    TS_ASSERT_EQUALS(property.GetValue(), 3.0);

    // Verify name accessors still work
    TS_ASSERT_EQUALS(property.GetName(), "test");
    TS_ASSERT_EQUALS(property.GetFullyQualifiedName(), "/lifecycle/test");
  }
};

/*******************************************************************************
 * Extended FGPropertyValue Tests (25 new tests: 76-100)
 ******************************************************************************/

class FGPropertyValueExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Rapid Value Change Tests
   ***************************************************************************/

  // Test 76: High frequency value updates
  void testHighFrequencyUpdates() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("rapid", true);
    FGPropertyValue property(node);

    for (int i = 0; i < 1000; i++) {
      double val = std::sin(static_cast<double>(i) * 0.01);
      property.SetValue(val);
      TS_ASSERT_DELTA(property.GetValue(), val, 1e-15);
    }
  }

  // Test 77: Oscillating value pattern
  void testOscillatingValuePattern() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("oscillate", true);
    FGPropertyValue property(node);

    for (int i = 0; i < 100; i++) {
      double val = std::sin(static_cast<double>(i) * 0.1) * 1000.0;
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(property.GetValue(), val, 1e-10);
    }
  }

  // Test 78: Sawtooth value pattern
  void testSawtoothValuePattern() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("sawtooth", true);
    FGPropertyValue property(node);

    for (int cycle = 0; cycle < 10; cycle++) {
      for (int i = 0; i < 100; i++) {
        double val = static_cast<double>(i) / 100.0;
        property.SetValue(val);
        TS_ASSERT_DELTA(property.GetValue(), val, 1e-15);
      }
    }
  }

  /***************************************************************************
   * Signed Property Advanced Tests
   ***************************************************************************/

  // Test 79: Signed property with oscillating source
  void testSignedPropertyOscillating() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue signedProp("-osc", pm, nullptr);

    auto node = pm->GetNode("osc", true);
    for (int i = 0; i < 50; i++) {
      double val = std::sin(static_cast<double>(i) * 0.2) * 100.0;
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(signedProp.GetValue(), -val, 1e-10);
    }
  }

  // Test 80: Multiple signed properties same source
  void testMultipleSignedPropertiesSameSource() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue signed1("-shared", pm, nullptr);
    FGPropertyValue signed2("-shared", pm, nullptr);
    FGPropertyValue unsigned1("shared", pm, nullptr);

    auto node = pm->GetNode("shared", true);
    node->setDoubleValue(42.0);

    TS_ASSERT_EQUALS(signed1.GetValue(), -42.0);
    TS_ASSERT_EQUALS(signed2.GetValue(), -42.0);
    TS_ASSERT_EQUALS(unsigned1.GetValue(), 42.0);
  }

  // Test 81: Signed property with epsilon value
  void testSignedPropertyEpsilon() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("-eps", pm, nullptr);

    auto node = pm->GetNode("eps", true);
    double eps = std::numeric_limits<double>::epsilon();
    node->setDoubleValue(eps);
    TS_ASSERT_DELTA(property.GetValue(), -eps, 1e-30);
  }

  /***************************************************************************
   * Property Node Hierarchy Tests
   ***************************************************************************/

  // Test 82: Sibling properties in same parent
  void testSiblingPropertiesSameParent() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_a = root.getNode("parent/child_a", true);
    SGPropertyNode_ptr node_b = root.getNode("parent/child_b", true);
    FGPropertyValue prop_a(node_a);
    FGPropertyValue prop_b(node_b);

    node_a->setDoubleValue(100.0);
    node_b->setDoubleValue(200.0);

    TS_ASSERT_EQUALS(prop_a.GetValue(), 100.0);
    TS_ASSERT_EQUALS(prop_b.GetValue(), 200.0);

    // Verify independence
    prop_a.SetValue(150.0);
    TS_ASSERT_EQUALS(prop_b.GetValue(), 200.0);
  }

  // Test 83: Properties at different tree levels
  void testPropertiesDifferentLevels() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_shallow = root.getNode("shallow", true);
    SGPropertyNode_ptr node_deep = root.getNode("a/b/c/d/deep", true);
    FGPropertyValue prop_shallow(node_shallow);
    FGPropertyValue prop_deep(node_deep);

    prop_shallow.SetValue(1.0);
    prop_deep.SetValue(2.0);

    TS_ASSERT_EQUALS(prop_shallow.GetName(), "shallow");
    TS_ASSERT_EQUALS(prop_deep.GetName(), "deep");
    TS_ASSERT_EQUALS(prop_shallow.GetValue(), 1.0);
    TS_ASSERT_EQUALS(prop_deep.GetValue(), 2.0);
  }

  // Test 84: Array indexed properties with values
  void testArrayIndexedPropertiesWithValues() {
    SGPropertyNode root;
    std::vector<FGPropertyValue*> properties;

    for (int i = 0; i < 5; i++) {
      SGPropertyNode_ptr node = root.getNode("array", i, true);
      node->setDoubleValue(static_cast<double>(i * 100));
      properties.push_back(new FGPropertyValue(node));
    }

    for (int i = 0; i < 5; i++) {
      TS_ASSERT_EQUALS(properties[i]->GetValue(), static_cast<double>(i * 100));
      delete properties[i];
    }
  }

  /***************************************************************************
   * SetNode Edge Cases
   ***************************************************************************/

  // Test 85: SetNode from late-bound to resolved
  void testSetNodeFromLateBoundToResolved() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("unresolved", pm, nullptr);

    TS_ASSERT(property.IsLateBound());

    SGPropertyNode_ptr resolved = pm->GetNode("resolved", true);
    resolved->setDoubleValue(99.0);

    property.SetNode(resolved);
    TS_ASSERT(!property.IsLateBound());
    TS_ASSERT_EQUALS(property.GetValue(), 99.0);
  }

  // Test 86: SetNode back and forth
  void testSetNodeBackAndForth() {
    SGPropertyNode root;
    SGPropertyNode_ptr node_a = root.getNode("a", true);
    SGPropertyNode_ptr node_b = root.getNode("b", true);
    FGPropertyValue property(node_a);

    node_a->setDoubleValue(1.0);
    node_b->setDoubleValue(2.0);

    for (int i = 0; i < 10; i++) {
      property.SetNode(node_b);
      TS_ASSERT_EQUALS(property.GetValue(), 2.0);

      property.SetNode(node_a);
      TS_ASSERT_EQUALS(property.GetValue(), 1.0);
    }
  }

  // Test 87: SetNode preserves property functionality
  void testSetNodePreservesPropertyFunctionality() {
    SGPropertyNode root;
    SGPropertyNode_ptr old_node = root.getNode("old", true);
    SGPropertyNode_ptr new_node = root.getNode("new", true);
    FGPropertyValue property(old_node);

    property.SetValue(100.0);
    TS_ASSERT_EQUALS(old_node->getDoubleValue(), 100.0);

    property.SetNode(new_node);
    property.SetValue(200.0);
    TS_ASSERT_EQUALS(new_node->getDoubleValue(), 200.0);
    TS_ASSERT_EQUALS(old_node->getDoubleValue(), 100.0);  // Old node unchanged
  }

  /***************************************************************************
   * Special Value Combinations
   ***************************************************************************/

  // Test 88: Subnormal values
  void testSubnormalValues() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("subnormal", true);
    FGPropertyValue property(node);

    double subnormal = std::numeric_limits<double>::denorm_min();
    property.SetValue(subnormal);
    TS_ASSERT_EQUALS(property.GetValue(), subnormal);

    property.SetValue(-subnormal);
    TS_ASSERT_EQUALS(property.GetValue(), -subnormal);
  }

  // Test 89: Quiet NaN vs signaling NaN (if distinguishable)
  void testQuietNaN() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("qnan", true);
    FGPropertyValue property(node);

    double qnan = std::numeric_limits<double>::quiet_NaN();
    property.SetValue(qnan);
    TS_ASSERT(std::isnan(property.GetValue()));
  }

  // Test 90: Lowest negative double
  void testLowestNegativeDouble() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("lowest", true);
    FGPropertyValue property(node);

    double lowest = std::numeric_limits<double>::lowest();
    property.SetValue(lowest);
    TS_ASSERT_EQUALS(property.GetValue(), lowest);
  }

  // Test 91: Near-zero values with different signs
  void testNearZeroSignedValues() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("nearzero", true);
    FGPropertyValue property(node);

    double tiny_pos = 1e-300;
    double tiny_neg = -1e-300;

    property.SetValue(tiny_pos);
    TS_ASSERT(property.GetValue() > 0.0);

    property.SetValue(tiny_neg);
    TS_ASSERT(property.GetValue() < 0.0);
  }

  /***************************************************************************
   * Late Binding Advanced Tests
   ***************************************************************************/

  // Test 92: Late binding with property created after multiple access attempts
  void testLateBoundMultipleAccessAttempts() {
    auto pm = std::make_shared<FGPropertyManager>();
    FGPropertyValue property("delayed", pm, nullptr);

    // Multiple access attempts should all throw
    for (int i = 0; i < 3; i++) {
      TS_ASSERT_THROWS(property.GetValue(), BaseException&);
    }

    // Create property and verify it binds
    auto node = pm->GetNode("delayed", true);
    node->setDoubleValue(42.0);
    TS_ASSERT_EQUALS(property.GetValue(), 42.0);
    TS_ASSERT(!property.IsLateBound());
  }

  // Test 93: Late bound property with very long path
  void testLateBoundVeryLongPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    std::string long_path = "level1/level2/level3/level4/level5/level6/level7/level8/prop";
    FGPropertyValue property(long_path, pm, nullptr);

    TS_ASSERT(property.IsLateBound());

    auto node = pm->GetNode(long_path, true);
    node->setDoubleValue(12345.0);

    TS_ASSERT_EQUALS(property.GetValue(), 12345.0);
    TS_ASSERT(!property.IsLateBound());
  }

  // Test 94: Late bound signed with very long path
  void testLateBoundSignedVeryLongPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    std::string long_path = "a/b/c/d/e/f/g/h/i/j";
    FGPropertyValue property("-" + long_path, pm, nullptr);

    auto node = pm->GetNode(long_path, true);
    node->setDoubleValue(500.0);

    TS_ASSERT_EQUALS(property.GetValue(), -500.0);
  }

  /***************************************************************************
   * Property Attribute Tests
   ***************************************************************************/

  // Test 95: Read-only property behavior
  void testReadOnlyPropertyBehavior() {
    auto pm = std::make_shared<FGPropertyManager>();
    SGPropertyNode_ptr node = pm->GetNode("readonly", true);
    node->setDoubleValue(100.0);
    node->setAttribute(SGPropertyNode::WRITE, false);

    FGPropertyValue property(node);
    TS_ASSERT(property.IsConstant());
    TS_ASSERT_EQUALS(property.GetValue(), 100.0);
  }

  // Test 96: Property with multiple attribute changes
  void testMultipleAttributeChanges() {
    auto pm = std::make_shared<FGPropertyManager>();
    SGPropertyNode_ptr node = pm->GetNode("attrs", true);
    FGPropertyValue property(node);

    double currentValue = 50.0;
    node->setDoubleValue(currentValue);

    // Toggle attributes
    for (int i = 0; i < 5; i++) {
      node->setAttribute(SGPropertyNode::WRITE, false);
      TS_ASSERT(property.IsConstant());
      TS_ASSERT_EQUALS(property.GetValue(), currentValue);

      node->setAttribute(SGPropertyNode::WRITE, true);
      TS_ASSERT(!property.IsConstant());
      currentValue = 50.0 + i + 1;
      property.SetValue(currentValue);
    }
  }

  /***************************************************************************
   * Value Consistency Tests
   ***************************************************************************/

  // Test 97: Consistent reads under no modification
  void testConsistentReadsNoModification() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("consistent", true);
    node->setDoubleValue(3.14159265358979);
    FGPropertyValue property(node);

    for (int i = 0; i < 1000; i++) {
      TS_ASSERT_EQUALS(property.GetValue(), 3.14159265358979);
    }
  }

  // Test 98: Value transition boundary
  void testValueTransitionBoundary() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("transition", true);
    FGPropertyValue property(node);

    // Test transition from negative to positive
    property.SetValue(-0.001);
    TS_ASSERT(property.GetValue() < 0.0);

    property.SetValue(0.0);
    TS_ASSERT_EQUALS(property.GetValue(), 0.0);

    property.SetValue(0.001);
    TS_ASSERT(property.GetValue() > 0.0);
  }

  // Test 99: Large magnitude transitions
  void testLargeMagnitudeTransitions() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("magnitude", true);
    FGPropertyValue property(node);

    double large = 1e100;
    double small = 1e-100;

    property.SetValue(large);
    TS_ASSERT_EQUALS(property.GetValue(), large);

    property.SetValue(small);
    TS_ASSERT_DELTA(property.GetValue(), small, 1e-115);

    property.SetValue(-large);
    TS_ASSERT_EQUALS(property.GetValue(), -large);

    property.SetValue(-small);
    TS_ASSERT_DELTA(property.GetValue(), -small, 1e-115);
  }

  // Test 100: Property value stress test
  void testPropertyValueStressTest() {
    SGPropertyNode root;
    SGPropertyNode_ptr node = root.getNode("stress", true);
    FGPropertyValue property(node);

    // Rapid alternation between special values
    double values[] = {
      0.0, 1.0, -1.0,
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::min(),
      std::numeric_limits<double>::epsilon(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()
    };

    for (int cycle = 0; cycle < 100; cycle++) {
      for (double val : values) {
        property.SetValue(val);
        if (std::isfinite(val)) {
          TS_ASSERT_EQUALS(property.GetValue(), val);
        } else if (std::isinf(val)) {
          TS_ASSERT(std::isinf(property.GetValue()));
          if (val > 0) {
            TS_ASSERT(property.GetValue() > 0);
          } else {
            TS_ASSERT(property.GetValue() < 0);
          }
        }
      }
    }
  }
};
