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
};
