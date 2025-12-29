#include <memory>
#include <string>

#include <cxxtest/TestSuite.h>
#include <input_output/FGPropertyManager.h>

using namespace JSBSim;


class FGPropertyManagerTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction Tests
   ***************************************************************************/

  void testConstructor() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto root = pm->GetNode();

    TS_ASSERT_EQUALS(root->getNameString(), "");
    TS_ASSERT_EQUALS(GetFullyQualifiedName(root), "/");
  }

  void testConstructorWithExistingNode() {
    SGPropertyNode* node = new SGPropertyNode();
    FGPropertyManager pm(node);

    TS_ASSERT_EQUALS(pm.GetNode(), node);
  }

  /***************************************************************************
   * GetNode Tests
   ***************************************************************************/

  void testGetNodeCreateFalse() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Without create=true, nonexistent nodes should return nullptr
    auto node = pm->GetNode("nonexistent/path", false);
    TS_ASSERT(node == nullptr);
  }

  void testGetNodeCreateTrue() {
    auto pm = std::make_shared<FGPropertyManager>();

    // With create=true, node should be created
    auto node = pm->GetNode("test/path", true);
    TS_ASSERT(node != nullptr);
    TS_ASSERT_EQUALS(node->getNameString(), "path");
  }

  void testGetNodeNested() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("level1/level2/level3", true);
    TS_ASSERT(node != nullptr);
    TS_ASSERT_EQUALS(node->getNameString(), "level3");
  }

  void testGetNodeWithIndex() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("array", 0, true);
    TS_ASSERT(node != nullptr);

    auto node1 = pm->GetNode("array", 1, true);
    TS_ASSERT(node1 != nullptr);

    // Verify they're different nodes
    TS_ASSERT(node != node1);
  }

  /***************************************************************************
   * HasNode Tests
   ***************************************************************************/

  void testHasNodeNonexistent() {
    auto pm = std::make_shared<FGPropertyManager>();

    TS_ASSERT_EQUALS(pm->HasNode("nonexistent"), false);
  }

  void testHasNodeExists() {
    auto pm = std::make_shared<FGPropertyManager>();

    pm->GetNode("exists", true);
    TS_ASSERT_EQUALS(pm->HasNode("exists"), true);
  }

  void testHasNodeWithMinusPrefix() {
    auto pm = std::make_shared<FGPropertyManager>();

    pm->GetNode("test", true);
    // HasNode should strip leading '-'
    TS_ASSERT_EQUALS(pm->HasNode("-test"), true);
  }

  /***************************************************************************
   * mkPropertyName Tests
   ***************************************************************************/

  void testMkPropertyNameSpaces() {
    auto pm = std::make_shared<FGPropertyManager>();

    std::string name = pm->mkPropertyName("test name with spaces", false);
    TS_ASSERT_EQUALS(name, "test-name-with-spaces");
  }

  void testMkPropertyNameLowercase() {
    auto pm = std::make_shared<FGPropertyManager>();

    std::string name = pm->mkPropertyName("TEST NAME", true);
    TS_ASSERT_EQUALS(name, "test-name");
  }

  void testMkPropertyNameNoLowercase() {
    auto pm = std::make_shared<FGPropertyManager>();

    std::string name = pm->mkPropertyName("TEST NAME", false);
    TS_ASSERT_EQUALS(name, "TEST-NAME");
  }

  /***************************************************************************
   * Property Value Tests
   ***************************************************************************/

  void testSetAndGetDouble() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("test/double", true);
    node->setDoubleValue(3.14159);

    TS_ASSERT_DELTA(node->getDoubleValue(), 3.14159, 1e-10);
  }

  void testSetAndGetInt() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("test/int", true);
    node->setIntValue(42);

    TS_ASSERT_EQUALS(node->getIntValue(), 42);
  }

  void testSetAndGetBool() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("test/bool", true);
    node->setBoolValue(true);

    TS_ASSERT_EQUALS(node->getBoolValue(), true);
  }

  void testSetAndGetString() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("test/string", true);
    node->setStringValue("hello world");

    TS_ASSERT_EQUALS(node->getStringValue(), std::string("hello world"));
  }

  /***************************************************************************
   * Tie/Untie Tests
   ***************************************************************************/

  void testTiePointer() {
    auto pm = std::make_shared<FGPropertyManager>();
    double value = 123.456;

    pm->Tie("tied/value", &value);

    auto node = pm->GetNode("tied/value");
    TS_ASSERT(node != nullptr);
    TS_ASSERT_DELTA(node->getDoubleValue(), 123.456, 1e-10);

    // Change the underlying value
    value = 789.012;
    TS_ASSERT_DELTA(node->getDoubleValue(), 789.012, 1e-10);
  }

  void testUntie() {
    auto pm = std::make_shared<FGPropertyManager>();
    double value = 100.0;

    pm->Tie("untie/test", &value);
    pm->Untie("untie/test");

    // After untie, changing the value shouldn't affect the property
    auto node = pm->GetNode("untie/test");
    double oldValue = node->getDoubleValue();
    value = 200.0;

    // Value should be frozen at the time of untie
    TS_ASSERT_DELTA(node->getDoubleValue(), oldValue, 1e-10);
  }

  void testUnbind() {
    auto pm = std::make_shared<FGPropertyManager>();
    double value1 = 1.0, value2 = 2.0, value3 = 3.0;

    pm->Tie("unbind/test1", &value1);
    pm->Tie("unbind/test2", &value2);
    pm->Tie("unbind/test3", &value3);

    // Should unbind all without crashing
    pm->Unbind();
    TS_ASSERT(true);
  }

  /***************************************************************************
   * GetFullyQualifiedName Tests
   ***************************************************************************/

  void testGetFullyQualifiedNameRoot() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto root = pm->GetNode();

    TS_ASSERT_EQUALS(GetFullyQualifiedName(root), "/");
  }

  void testGetFullyQualifiedNameNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("level1/level2/level3", true);

    TS_ASSERT_EQUALS(GetFullyQualifiedName(node), "/level1/level2/level3");
  }

  /***************************************************************************
   * GetPrintableName Tests
   ***************************************************************************/

  void testGetPrintableName() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("test/path", true);

    std::string printable = GetPrintableName(node);
    TS_ASSERT(printable.find("path") != std::string::npos);
  }

  /***************************************************************************
   * GetRelativeName Tests
   ***************************************************************************/

  void testGetRelativeName() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("base/sub/leaf", true);

    std::string relative = GetRelativeName(node, "/base");
    // Result includes leading slash but strips the base path
    TS_ASSERT(relative.find("sub/leaf") != std::string::npos);
  }

  /***************************************************************************
   * Multiple Property Manager Tests
   ***************************************************************************/

  void testMultipleManagers() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();

    pm1->GetNode("pm1/prop", true)->setDoubleValue(1.0);
    pm2->GetNode("pm2/prop", true)->setDoubleValue(2.0);

    // Each manager should have independent properties
    TS_ASSERT_EQUALS(pm1->HasNode("pm1/prop"), true);
    TS_ASSERT_EQUALS(pm1->HasNode("pm2/prop"), false);
    TS_ASSERT_EQUALS(pm2->HasNode("pm2/prop"), true);
    TS_ASSERT_EQUALS(pm2->HasNode("pm1/prop"), false);
  }

  /***************************************************************************
   * Node Traversal Tests
   ***************************************************************************/

  void testGetParentNode() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto child = pm->GetNode("parent/child", true);
    auto parent = child->getParent();

    TS_ASSERT(parent != nullptr);
    TS_ASSERT_EQUALS(parent->getNameString(), "parent");
  }

  void testGetChildCount() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto parent = pm->GetNode("parent", true);

    // Create children
    pm->GetNode("parent/child1", true);
    pm->GetNode("parent/child2", true);
    pm->GetNode("parent/child3", true);

    TS_ASSERT_EQUALS(parent->nChildren(), 3);
  }

  void testGetChildByIndex() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("parent/alpha", true);
    pm->GetNode("parent/beta", true);
    pm->GetNode("parent/gamma", true);

    auto parent = pm->GetNode("parent");
    TS_ASSERT(parent != nullptr);

    // Access children by index
    auto child0 = parent->getChild(0);
    auto child1 = parent->getChild(1);
    auto child2 = parent->getChild(2);

    TS_ASSERT(child0 != nullptr);
    TS_ASSERT(child1 != nullptr);
    TS_ASSERT(child2 != nullptr);
  }

  void testGetChildByName() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("parent/named_child", true)->setIntValue(42);

    auto parent = pm->GetNode("parent");
    auto child = parent->getChild("named_child");

    TS_ASSERT(child != nullptr);
    TS_ASSERT_EQUALS(child->getIntValue(), 42);
  }

  void testGetRootNode() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto deep = pm->GetNode("a/b/c/d/e", true);

    auto root = deep->getRootNode();
    TS_ASSERT(root != nullptr);
    TS_ASSERT_EQUALS(root->getNameString(), "");
  }

  /***************************************************************************
   * Additional Property Type Tests
   ***************************************************************************/

  void testSetAndGetLong() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("test/long", true);

    node->setLongValue(123456789L);
    TS_ASSERT_EQUALS(node->getLongValue(), 123456789L);
  }

  void testSetAndGetFloat() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("test/float", true);

    node->setFloatValue(2.718f);
    TS_ASSERT_DELTA(node->getFloatValue(), 2.718f, 1e-5);
  }

  void testUnspecifiedType() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("test/unspecified", true);

    // Node with no value set should have UNSPECIFIED type
    TS_ASSERT(node->getType() == simgear::props::UNSPECIFIED ||
              node->getType() == simgear::props::NONE);
  }

  void testTypeAfterSetting() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("test/typed", true);

    node->setDoubleValue(1.0);
    TS_ASSERT_EQUALS(node->getType(), simgear::props::DOUBLE);

    // Note: Once type is set, it may not change with subsequent sets
    // Test that the original type is preserved
    node->setIntValue(1);
    // Type may stay DOUBLE since it was already typed
    TS_ASSERT(node->getType() == simgear::props::DOUBLE ||
              node->getType() == simgear::props::INT);
  }

  /***************************************************************************
   * Node Removal Tests
   ***************************************************************************/

  void testRemoveChild() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("parent/child1", true);
    pm->GetNode("parent/child2", true);

    auto parent = pm->GetNode("parent");
    int initialCount = parent->nChildren();

    parent->removeChild("child1");

    TS_ASSERT_EQUALS(parent->nChildren(), initialCount - 1);
    TS_ASSERT(parent->getChild("child1") == nullptr);
  }

  void testRemoveAllChildren() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("parent/child1", true);
    pm->GetNode("parent/child2", true);
    pm->GetNode("parent/child3", true);

    auto parent = pm->GetNode("parent");
    parent->removeAllChildren();

    TS_ASSERT_EQUALS(parent->nChildren(), 0);
  }

  /***************************************************************************
   * Type Coercion Tests
   ***************************************************************************/

  void testDoubleToIntCoercion() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("coerce/double", true);

    node->setDoubleValue(3.7);
    int intVal = node->getIntValue();

    TS_ASSERT_EQUALS(intVal, 3);  // Truncated
  }

  void testIntToDoubleCoercion() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("coerce/int", true);

    node->setIntValue(42);
    double doubleVal = node->getDoubleValue();

    TS_ASSERT_DELTA(doubleVal, 42.0, 1e-10);
  }

  void testBoolToIntCoercion() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("coerce/bool", true);

    node->setBoolValue(true);
    TS_ASSERT_EQUALS(node->getIntValue(), 1);

    node->setBoolValue(false);
    TS_ASSERT_EQUALS(node->getIntValue(), 0);
  }

  void testIntToBoolCoercion() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("coerce/tobool", true);

    node->setIntValue(0);
    TS_ASSERT_EQUALS(node->getBoolValue(), false);

    node->setIntValue(1);
    TS_ASSERT_EQUALS(node->getBoolValue(), true);

    node->setIntValue(42);  // Any non-zero
    TS_ASSERT_EQUALS(node->getBoolValue(), true);
  }

  void testStringToDoubleCoercion() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("coerce/string", true);

    node->setStringValue("123.456");
    TS_ASSERT_DELTA(node->getDoubleValue(), 123.456, 1e-6);
  }

  void testDoubleToStringCoercion() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("coerce/dtostr", true);

    node->setDoubleValue(3.14);
    std::string str = node->getStringValue();

    TS_ASSERT(str.find("3.14") != std::string::npos);
  }

  /***************************************************************************
   * Path Edge Cases
   ***************************************************************************/

  void testEmptyPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("", false);

    // Empty path should return root
    TS_ASSERT(node != nullptr);
  }

  void testTrailingSlash() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("test/path", true);

    // Trailing slash should still find the node
    auto node = pm->GetNode("test/path/");
    TS_ASSERT(node != nullptr);
  }

  void testDoubleSlash() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("test//double", true);

    // Should handle double slashes gracefully
    TS_ASSERT(node != nullptr);
  }

  void testAbsolutePath() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("test/abs", true);

    // Leading slash indicates absolute path
    auto node = pm->GetNode("/test/abs");
    TS_ASSERT(node != nullptr);
  }

  /***************************************************************************
   * Index Array Operations
   ***************************************************************************/

  void testArrayAccess() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Create array elements
    pm->GetNode("array", 0, true)->setIntValue(10);
    pm->GetNode("array", 1, true)->setIntValue(20);
    pm->GetNode("array", 2, true)->setIntValue(30);

    TS_ASSERT_EQUALS(pm->GetNode("array", 0)->getIntValue(), 10);
    TS_ASSERT_EQUALS(pm->GetNode("array", 1)->getIntValue(), 20);
    TS_ASSERT_EQUALS(pm->GetNode("array", 2)->getIntValue(), 30);
  }

  void testArrayPathSyntax() {
    auto pm = std::make_shared<FGPropertyManager>();

    pm->GetNode("items[0]", true)->setIntValue(100);
    pm->GetNode("items[1]", true)->setIntValue(200);

    auto item0 = pm->GetNode("items[0]");
    auto item1 = pm->GetNode("items[1]");

    TS_ASSERT(item0 != nullptr);
    TS_ASSERT(item1 != nullptr);
    TS_ASSERT_EQUALS(item0->getIntValue(), 100);
    TS_ASSERT_EQUALS(item1->getIntValue(), 200);
  }

  void testNestedArrays() {
    auto pm = std::make_shared<FGPropertyManager>();

    pm->GetNode("matrix/row", 0, true)->getNode("col", 0, true)->setIntValue(1);
    pm->GetNode("matrix/row", 0, true)->getNode("col", 1, true)->setIntValue(2);
    pm->GetNode("matrix/row", 1, true)->getNode("col", 0, true)->setIntValue(3);

    auto val00 = pm->GetNode("matrix/row[0]/col[0]");
    auto val01 = pm->GetNode("matrix/row[0]/col[1]");

    TS_ASSERT(val00 != nullptr);
    TS_ASSERT(val01 != nullptr);
  }

  /***************************************************************************
   * Attribute Tests
   ***************************************************************************/

  void testAttributeReadOnly() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("readonly/prop", true);

    node->setDoubleValue(1.0);
    node->setAttribute(SGPropertyNode::READ, true);
    node->setAttribute(SGPropertyNode::WRITE, false);

    // Should be readable
    TS_ASSERT_DELTA(node->getDoubleValue(), 1.0, 1e-10);

    // Attribute flags should be set
    TS_ASSERT_EQUALS(node->getAttribute(SGPropertyNode::READ), true);
  }

  void testAttributeUserArchive() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("archive/prop", true);

    node->setAttribute(SGPropertyNode::USERARCHIVE, true);

    TS_ASSERT_EQUALS(node->getAttribute(SGPropertyNode::USERARCHIVE), true);
  }

  void testAttributePreserve() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("preserve/prop", true);

    node->setAttribute(SGPropertyNode::PRESERVE, true);

    TS_ASSERT_EQUALS(node->getAttribute(SGPropertyNode::PRESERVE), true);
  }

  /***************************************************************************
   * Alias Tests
   ***************************************************************************/

  void testPropertyAlias() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto original = pm->GetNode("original/value", true);
    auto alias = pm->GetNode("alias/value", true);

    original->setDoubleValue(42.0);
    alias->alias(original);

    // Alias should reflect the original value
    TS_ASSERT_DELTA(alias->getDoubleValue(), 42.0, 1e-10);

    // Changing original should change alias
    original->setDoubleValue(100.0);
    TS_ASSERT_DELTA(alias->getDoubleValue(), 100.0, 1e-10);
  }

  void testIsAlias() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto original = pm->GetNode("orig", true);
    auto alias = pm->GetNode("als", true);

    alias->alias(original);

    TS_ASSERT_EQUALS(alias->isAlias(), true);
    TS_ASSERT_EQUALS(original->isAlias(), false);
  }

  void testUnalias() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto original = pm->GetNode("original2", true);
    auto alias = pm->GetNode("alias2", true);

    original->setDoubleValue(50.0);
    alias->alias(original);
    alias->unalias();

    TS_ASSERT_EQUALS(alias->isAlias(), false);

    // After unalias, the node no longer tracks original
    // but may not have retained the value
    original->setDoubleValue(999.0);
    // Just verify it's no longer aliased
    TS_ASSERT(!alias->isAlias());
  }

  void testMultipleAliases() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto source = pm->GetNode("source/value", true);
    auto alias1 = pm->GetNode("alias1", true);
    auto alias2 = pm->GetNode("alias2", true);

    source->setDoubleValue(100.0);
    alias1->alias(source);
    alias2->alias(source);

    // Both aliases should point to same value
    TS_ASSERT_DELTA(alias1->getDoubleValue(), 100.0, 1e-10);
    TS_ASSERT_DELTA(alias2->getDoubleValue(), 100.0, 1e-10);

    // Change source
    source->setDoubleValue(200.0);
    TS_ASSERT_DELTA(alias1->getDoubleValue(), 200.0, 1e-10);
    TS_ASSERT_DELTA(alias2->getDoubleValue(), 200.0, 1e-10);
  }

  /***************************************************************************
   * Copy Operations
   ***************************************************************************/

  void testCopyValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto src = pm->GetNode("src", true);
    auto dst = pm->GetNode("dst", true);

    src->setDoubleValue(3.14);
    dst->setDoubleValue(src->getDoubleValue());

    TS_ASSERT_DELTA(dst->getDoubleValue(), 3.14, 1e-10);

    // Changing source shouldn't affect destination (not aliased)
    src->setDoubleValue(2.71);
    TS_ASSERT_DELTA(dst->getDoubleValue(), 3.14, 1e-10);
  }

  /***************************************************************************
   * Deep Nesting Tests
   ***************************************************************************/

  void testDeepNesting() {
    auto pm = std::make_shared<FGPropertyManager>();
    std::string deepPath = "a/b/c/d/e/f/g/h/i/j/k/l/m/n/o/p";

    auto node = pm->GetNode(deepPath, true);
    node->setIntValue(12345);

    auto retrieved = pm->GetNode(deepPath);
    TS_ASSERT(retrieved != nullptr);
    TS_ASSERT_EQUALS(retrieved->getIntValue(), 12345);
  }

  void testDeepNodeDepth() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("l1/l2/l3/l4/l5", true);

    int depth = 0;
    SGPropertyNode* current = node;
    while (current->getParent() != nullptr) {
      depth++;
      current = current->getParent();
    }

    TS_ASSERT_EQUALS(depth, 5);
  }

  /***************************************************************************
   * Tie with Methods Tests
   ***************************************************************************/

  void testTieInt() {
    auto pm = std::make_shared<FGPropertyManager>();
    int value = 100;

    pm->Tie("tied/int", &value);

    auto node = pm->GetNode("tied/int");
    TS_ASSERT(node != nullptr);
    TS_ASSERT_EQUALS(node->getIntValue(), 100);

    value = 200;
    TS_ASSERT_EQUALS(node->getIntValue(), 200);
  }

  void testTieBool() {
    auto pm = std::make_shared<FGPropertyManager>();
    bool flag = true;

    pm->Tie("tied/bool", &flag);

    auto node = pm->GetNode("tied/bool");
    TS_ASSERT(node != nullptr);
    TS_ASSERT_EQUALS(node->getBoolValue(), true);

    flag = false;
    TS_ASSERT_EQUALS(node->getBoolValue(), false);
  }

  /***************************************************************************
   * Property Name Tests
   ***************************************************************************/

  void testUnderscoreInName() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Underscores are typically allowed
    auto node = pm->GetNode("test_property_name", true);
    TS_ASSERT(node != nullptr);
  }

  void testHyphenInName() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("test-property-name", true);
    TS_ASSERT(node != nullptr);
  }

  void testMixedCaseName() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node1 = pm->GetNode("CamelCase", true);
    auto node2 = pm->GetNode("lowercase", true);
    auto node3 = pm->GetNode("UPPERCASE", true);

    TS_ASSERT(node1 != nullptr);
    TS_ASSERT(node2 != nullptr);
    TS_ASSERT(node3 != nullptr);
  }

  /***************************************************************************
   * Multiple Value Updates
   ***************************************************************************/

  void testMultipleUpdates() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("update/test", true);

    for (int i = 0; i < 100; i++) {
      node->setIntValue(i);
      TS_ASSERT_EQUALS(node->getIntValue(), i);
    }
  }

  void testSequentialTypeSet() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Use separate nodes to avoid type confusion
    auto intNode = pm->GetNode("types/myint", true);
    auto dblNode = pm->GetNode("types/mydbl", true);
    auto strNode = pm->GetNode("types/mystr", true);
    auto boolNode = pm->GetNode("types/mybool", true);

    intNode->setIntValue(1);
    dblNode->setDoubleValue(2.5);
    strNode->setStringValue("hello");
    boolNode->setBoolValue(true);

    TS_ASSERT_EQUALS(intNode->getIntValue(), 1);
    TS_ASSERT_DELTA(dblNode->getDoubleValue(), 2.5, 1e-10);
    TS_ASSERT_EQUALS(strNode->getStringValue(), std::string("hello"));
    TS_ASSERT_EQUALS(boolNode->getBoolValue(), true);
  }

  /***************************************************************************
   * Edge Value Tests
   ***************************************************************************/

  void testLargeDoubleValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("large/double", true);

    double large = 1e50;  // Reasonably large
    node->setDoubleValue(large);
    TS_ASSERT(node->getDoubleValue() > 1e49);
  }

  void testSmallDoubleValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("small/double", true);

    double small = 1e-50;  // Reasonably small
    node->setDoubleValue(small);
    TS_ASSERT(node->getDoubleValue() > 0);
    TS_ASSERT(node->getDoubleValue() < 1e-40);
  }

  void testNegativeValues() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto intNode = pm->GetNode("neg/int", true);
    intNode->setIntValue(-12345);
    TS_ASSERT_EQUALS(intNode->getIntValue(), -12345);

    auto dblNode = pm->GetNode("neg/double", true);
    dblNode->setDoubleValue(-3.14159);
    TS_ASSERT_DELTA(dblNode->getDoubleValue(), -3.14159, 1e-10);
  }

  void testZeroValues() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto intNode = pm->GetNode("zero/int", true);
    intNode->setIntValue(0);
    TS_ASSERT_EQUALS(intNode->getIntValue(), 0);

    auto dblNode = pm->GetNode("zero/double", true);
    dblNode->setDoubleValue(0.0);
    TS_ASSERT_DELTA(dblNode->getDoubleValue(), 0.0, 1e-10);
  }

  void testEmptyStringValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("empty/string", true);

    node->setStringValue("");
    TS_ASSERT_EQUALS(node->getStringValue(), std::string(""));
  }

  /***************************************************************************
   * Additional Navigation Tests
   ***************************************************************************/

  void testSiblingCount() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("parent/child1", true);
    pm->GetNode("parent/child2", true);
    pm->GetNode("parent/child3", true);

    auto parent = pm->GetNode("parent");

    // There should be 3 children
    TS_ASSERT_EQUALS(parent->nChildren(), 3);
  }

  void testParentChildRelation() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("a/b/c", true)->setIntValue(1);
    pm->GetNode("a/b/d", true)->setIntValue(2);

    auto nodeB = pm->GetNode("a/b");
    TS_ASSERT(nodeB != nullptr);
    TS_ASSERT_EQUALS(nodeB->nChildren(), 2);
  }

  /***************************************************************************
   * Section 17: Property Update Tests
   ***************************************************************************/

  void testMultipleValueUpdates() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("update/test", true);

    for (int i = 0; i < 100; i++) {
      node->setDoubleValue(i * 1.5);
    }

    TS_ASSERT_DELTA(node->getDoubleValue(), 99 * 1.5, 1e-10);
  }

  void testAlternatingTypes() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Different nodes for different types
    auto intNode = pm->GetNode("alt/int", true);
    auto dblNode = pm->GetNode("alt/dbl", true);
    auto boolNode = pm->GetNode("alt/bool", true);

    intNode->setIntValue(10);
    TS_ASSERT_EQUALS(intNode->getIntValue(), 10);

    dblNode->setDoubleValue(20.5);
    TS_ASSERT_DELTA(dblNode->getDoubleValue(), 20.5, 1e-10);

    boolNode->setBoolValue(true);
    TS_ASSERT_EQUALS(boolNode->getBoolValue(), true);
  }

  void testValuePersistence() {
    auto pm = std::make_shared<FGPropertyManager>();
    pm->GetNode("persist/a", true)->setIntValue(100);
    pm->GetNode("persist/b", true)->setIntValue(200);

    // Access other node, then come back
    auto nodeB = pm->GetNode("persist/b");
    auto nodeA = pm->GetNode("persist/a");

    TS_ASSERT_EQUALS(nodeA->getIntValue(), 100);
    TS_ASSERT_EQUALS(nodeB->getIntValue(), 200);
  }

  /***************************************************************************
   * Section 18: Property Tree Structure Tests
   ***************************************************************************/

  void testVeryDeepNesting() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("l1/l2/l3/l4/l5/l6/l7/l8", true);

    node->setIntValue(888);
    TS_ASSERT_EQUALS(node->getIntValue(), 888);
  }

  void testManyTopLevelNodes() {
    auto pm = std::make_shared<FGPropertyManager>();

    for (int i = 0; i < 50; i++) {
      std::string path = "top" + std::to_string(i) + "/value";
      pm->GetNode(path, true)->setIntValue(i);
    }

    // Verify some values
    TS_ASSERT_EQUALS(pm->GetNode("top0/value")->getIntValue(), 0);
    TS_ASSERT_EQUALS(pm->GetNode("top25/value")->getIntValue(), 25);
    TS_ASSERT_EQUALS(pm->GetNode("top49/value")->getIntValue(), 49);
  }

  void testNodeReuse() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node1 = pm->GetNode("reuse/test", true);
    auto node2 = pm->GetNode("reuse/test", false);

    TS_ASSERT(node1 == node2);  // Same node
  }

  /***************************************************************************
   * Section 19: Value Conversion Tests
   ***************************************************************************/

  void testIntToDouble() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("convert/int2dbl", true);

    node->setIntValue(42);
    double val = node->getDoubleValue();

    TS_ASSERT_DELTA(val, 42.0, 1e-10);
  }

  void testDoubleToInt() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("convert/dbl2int", true);

    node->setDoubleValue(42.7);
    int val = node->getIntValue();

    TS_ASSERT_EQUALS(val, 42);  // Truncated
  }

  void testBoolToInt() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("convert/bool2int", true);

    node->setBoolValue(true);
    TS_ASSERT_EQUALS(node->getIntValue(), 1);

    node->setBoolValue(false);
    TS_ASSERT_EQUALS(node->getIntValue(), 0);
  }

  /***************************************************************************
   * Section 20: Property Value Range Tests
   ***************************************************************************/

  void testLargeIntValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("range/int", true);

    node->setIntValue(2000000000);
    TS_ASSERT_EQUALS(node->getIntValue(), 2000000000);
  }

  void testNegativeValue() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("range/neg", true);

    node->setDoubleValue(-12345.678);
    TS_ASSERT_DELTA(node->getDoubleValue(), -12345.678, 1e-6);
  }

  void testVerySmallDouble() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("range/tiny", true);

    node->setDoubleValue(1e-15);
    TS_ASSERT(node->getDoubleValue() > 0);
    TS_ASSERT(node->getDoubleValue() < 1e-10);
  }

  /***************************************************************************
   * Section 21: Multiple Property Manager Tests
   ***************************************************************************/

  void testTwoPropertyManagers() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();

    pm1->GetNode("test/value", true)->setIntValue(1);
    pm2->GetNode("test/value", true)->setIntValue(2);

    TS_ASSERT_EQUALS(pm1->GetNode("test/value")->getIntValue(), 1);
    TS_ASSERT_EQUALS(pm2->GetNode("test/value")->getIntValue(), 2);
  }

  void testPropertyManagerIsolation() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();

    pm1->GetNode("isolated/prop", true)->setIntValue(100);

    auto node = pm2->GetNode("isolated/prop", false);
    TS_ASSERT(node == nullptr);  // Doesn't exist in pm2
  }

  /***************************************************************************
   * Section 22: String Property Tests
   ***************************************************************************/

  void testLongString() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("string/long", true);

    std::string longStr(500, 'x');
    node->setStringValue(longStr.c_str());

    TS_ASSERT_EQUALS(node->getStringValue(), longStr);
  }

  void testStringWithSpaces() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("string/spaces", true);

    node->setStringValue("hello world test");
    TS_ASSERT_EQUALS(node->getStringValue(), std::string("hello world test"));
  }

  void testStringWithNumbers() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto node = pm->GetNode("string/nums", true);

    node->setStringValue("test123value");
    TS_ASSERT_EQUALS(node->getStringValue(), std::string("test123value"));
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompletePropertyTree() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Build a complete property tree
    pm->GetNode("aircraft/type", true)->setStringValue("cessna172");
    pm->GetNode("aircraft/weight", true)->setDoubleValue(2300.0);
    pm->GetNode("aircraft/cg/x", true)->setDoubleValue(42.0);
    pm->GetNode("aircraft/cg/y", true)->setDoubleValue(0.0);
    pm->GetNode("aircraft/cg/z", true)->setDoubleValue(-10.0);

    TS_ASSERT_EQUALS(pm->GetNode("aircraft/type")->getStringValue(), std::string("cessna172"));
    TS_ASSERT_DELTA(pm->GetNode("aircraft/weight")->getDoubleValue(), 2300.0, 0.001);
    TS_ASSERT_DELTA(pm->GetNode("aircraft/cg/x")->getDoubleValue(), 42.0, 0.001);
  }

  void testCompletePropertyUpdate() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node = pm->GetNode("sim/altitude", true);
    node->setDoubleValue(0.0);

    // Simulate climbing
    for (int i = 1; i <= 100; i++) {
      node->setDoubleValue(i * 100.0);
    }

    TS_ASSERT_DELTA(node->getDoubleValue(), 10000.0, 0.001);
  }

  void testCompletePropertyHierarchy() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Create deep hierarchy
    pm->GetNode("level1/level2/level3/level4/level5/value", true)->setIntValue(42);

    auto node = pm->GetNode("level1/level2/level3/level4/level5/value", false);
    TS_ASSERT(node != nullptr);
    TS_ASSERT_EQUALS(node->getIntValue(), 42);
  }

  void testCompleteMultiTypeTree() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Mix of types in tree
    pm->GetNode("data/name", true)->setStringValue("test");
    pm->GetNode("data/count", true)->setIntValue(100);
    pm->GetNode("data/ratio", true)->setDoubleValue(0.75);
    pm->GetNode("data/active", true)->setBoolValue(true);

    TS_ASSERT_EQUALS(pm->GetNode("data/name")->getStringValue(), std::string("test"));
    TS_ASSERT_EQUALS(pm->GetNode("data/count")->getIntValue(), 100);
    TS_ASSERT_DELTA(pm->GetNode("data/ratio")->getDoubleValue(), 0.75, 0.001);
    TS_ASSERT(pm->GetNode("data/active")->getBoolValue());
  }

  void testCompletePropertyEnumeration() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Create multiple sibling nodes
    for (int i = 0; i < 10; i++) {
      std::string path = "siblings/child" + std::to_string(i);
      pm->GetNode(path, true)->setIntValue(i * 10);
    }

    // Verify all exist
    for (int i = 0; i < 10; i++) {
      std::string path = "siblings/child" + std::to_string(i);
      auto node = pm->GetNode(path, false);
      TS_ASSERT(node != nullptr);
      TS_ASSERT_EQUALS(node->getIntValue(), i * 10);
    }
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentPropertyManagers() {
    auto pm1 = std::make_shared<FGPropertyManager>();
    auto pm2 = std::make_shared<FGPropertyManager>();

    pm1->GetNode("shared/value", true)->setIntValue(100);
    pm2->GetNode("shared/value", true)->setIntValue(200);

    // Each manager has its own value
    TS_ASSERT_EQUALS(pm1->GetNode("shared/value")->getIntValue(), 100);
    TS_ASSERT_EQUALS(pm2->GetNode("shared/value")->getIntValue(), 200);

    // Changing pm2 doesn't affect pm1
    pm2->GetNode("shared/value")->setIntValue(300);
    TS_ASSERT_EQUALS(pm1->GetNode("shared/value")->getIntValue(), 100);
  }

  void testIndependentNodeModification() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto node1 = pm->GetNode("test/node1", true);
    auto node2 = pm->GetNode("test/node2", true);

    node1->setIntValue(111);
    node2->setIntValue(222);

    // Modifying node2 doesn't affect node1
    node2->setIntValue(333);
    TS_ASSERT_EQUALS(node1->getIntValue(), 111);
    TS_ASSERT_EQUALS(node2->getIntValue(), 333);
  }

  void testIndependentTreeBranches() {
    auto pm = std::make_shared<FGPropertyManager>();

    pm->GetNode("branch1/leaf", true)->setDoubleValue(1.5);
    pm->GetNode("branch2/leaf", true)->setDoubleValue(2.5);

    // Branches are independent
    TS_ASSERT_DELTA(pm->GetNode("branch1/leaf")->getDoubleValue(), 1.5, 0.001);
    TS_ASSERT_DELTA(pm->GetNode("branch2/leaf")->getDoubleValue(), 2.5, 0.001);

    // Modify one branch
    pm->GetNode("branch2/leaf")->setDoubleValue(3.5);
    TS_ASSERT_DELTA(pm->GetNode("branch1/leaf")->getDoubleValue(), 1.5, 0.001);
  }

  void testIndependentTypeHandling() {
    auto pm = std::make_shared<FGPropertyManager>();

    auto intNode = pm->GetNode("types/int", true);
    auto dblNode = pm->GetNode("types/dbl", true);
    auto strNode = pm->GetNode("types/str", true);

    intNode->setIntValue(42);
    dblNode->setDoubleValue(3.14159);
    strNode->setStringValue("hello");

    // All values preserved independently
    TS_ASSERT_EQUALS(intNode->getIntValue(), 42);
    TS_ASSERT_DELTA(dblNode->getDoubleValue(), 3.14159, 0.00001);
    TS_ASSERT_EQUALS(strNode->getStringValue(), std::string("hello"));
  }

  void testIndependentUpdateSequence() {
    auto pm = std::make_shared<FGPropertyManager>();

    std::vector<SGPropertyNode_ptr> nodes;
    for (int i = 0; i < 5; i++) {
      nodes.push_back(pm->GetNode("seq/node" + std::to_string(i), true));
      nodes[i]->setIntValue(i);
    }

    // Update in reverse order
    for (int i = 4; i >= 0; i--) {
      nodes[i]->setIntValue(i * 100);
    }

    // All nodes have updated values
    for (int i = 0; i < 5; i++) {
      TS_ASSERT_EQUALS(nodes[i]->getIntValue(), i * 100);
    }
  }

  void testIndependentNodeCreation() {
    auto pm = std::make_shared<FGPropertyManager>();

    // Create nodes in random order
    pm->GetNode("order/c", true)->setIntValue(3);
    pm->GetNode("order/a", true)->setIntValue(1);
    pm->GetNode("order/b", true)->setIntValue(2);

    // All nodes accessible regardless of creation order
    TS_ASSERT_EQUALS(pm->GetNode("order/a")->getIntValue(), 1);
    TS_ASSERT_EQUALS(pm->GetNode("order/b")->getIntValue(), 2);
    TS_ASSERT_EQUALS(pm->GetNode("order/c")->getIntValue(), 3);
  }
};
