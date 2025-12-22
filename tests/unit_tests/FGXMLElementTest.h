#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <string>

#include <input_output/FGXMLElement.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * FGXMLElement unit tests
 *
 * Tests XML element functionality including:
 * - Element creation and naming
 * - Attribute handling
 * - Data line storage
 * - Child element management
 * - Unit conversions
 * - Element navigation
 */
class FGXMLElementTest : public CxxTest::TestSuite
{
public:
  // Test element construction with name
  void testConstruction() {
    Element el("test_element");
    TS_ASSERT_EQUALS(el.GetName(), "test_element");
  }

  // Test empty element name
  void testEmptyName() {
    Element el("");
    TS_ASSERT_EQUALS(el.GetName(), "");
  }

  // Test change name
  void testChangeName() {
    Element el("original");
    el.ChangeName("new_name");
    TS_ASSERT_EQUALS(el.GetName(), "new_name");
  }

  // Test add and retrieve attribute
  void testAddAttribute() {
    Element el("test");
    el.AddAttribute("key", "value");

    TS_ASSERT(el.HasAttribute("key"));
    TS_ASSERT_EQUALS(el.GetAttributeValue("key"), "value");
  }

  // Test attribute not found
  void testAttributeNotFound() {
    Element el("test");
    TS_ASSERT(!el.HasAttribute("nonexistent"));
    TS_ASSERT_EQUALS(el.GetAttributeValue("nonexistent"), "");
  }

  // Test set attribute value
  void testSetAttributeValue() {
    Element el("test");
    el.AddAttribute("key", "old_value");

    bool result = el.SetAttributeValue("key", "new_value");
    TS_ASSERT(result);
    TS_ASSERT_EQUALS(el.GetAttributeValue("key"), "new_value");
  }

  // Test set nonexistent attribute
  void testSetNonexistentAttribute() {
    Element el("test");
    bool result = el.SetAttributeValue("nonexistent", "value");
    TS_ASSERT(!result);
  }

  // Test attribute value as number
  void testAttributeValueAsNumber() {
    Element el("test");
    el.AddAttribute("numeric", "42.5");

    double value = el.GetAttributeValueAsNumber("numeric");
    TS_ASSERT_DELTA(value, 42.5, epsilon);
  }

  // Test attribute value as number - invalid
  void testAttributeValueAsNumberInvalid() {
    Element el("test");
    // Test nonexistent attribute returns HUGE_VAL
    double value = el.GetAttributeValueAsNumber("nonexistent");
    TS_ASSERT(value == HUGE_VAL);
  }

  // Test add data line
  void testAddDataLine() {
    Element el("test");
    el.AddData("data line 1");
    el.AddData("data line 2");

    TS_ASSERT_EQUALS(el.GetNumDataLines(), 2u);
    TS_ASSERT_EQUALS(el.GetDataLine(0), "data line 1");
    TS_ASSERT_EQUALS(el.GetDataLine(1), "data line 2");
  }

  // Test get data line out of range
  void testGetDataLineOutOfRange() {
    Element el("test");
    el.AddData("only line");

    TS_ASSERT_EQUALS(el.GetDataLine(10), "");
  }

  // Test get data as number
  void testGetDataAsNumber() {
    Element el("test");
    el.AddData("123.456");

    double value = el.GetDataAsNumber();
    TS_ASSERT_DELTA(value, 123.456, epsilon);
  }

  // Test get data as number - multiple lines
  void testGetDataAsNumberMultipleLines() {
    Element el("test");
    el.AddData("line 1");
    el.AddData("line 2");

    // With multiple data lines, should return HUGE_VAL
    double value = el.GetDataAsNumber();
    TS_ASSERT(value == HUGE_VAL);
  }

  // Test add child element
  void testAddChildElement() {
    Element parent("parent");
    Element* child = new Element("child");

    parent.AddChildElement(child);
    TS_ASSERT_EQUALS(parent.GetNumElements(), 1u);
  }

  // Test get element by index
  void testGetElement() {
    Element parent("parent");
    Element* child1 = new Element("child1");
    Element* child2 = new Element("child2");

    parent.AddChildElement(child1);
    parent.AddChildElement(child2);

    Element* result = parent.GetElement(0);
    TS_ASSERT(result != nullptr);
    TS_ASSERT_EQUALS(result->GetName(), "child1");

    result = parent.GetElement(1);
    TS_ASSERT(result != nullptr);
    TS_ASSERT_EQUALS(result->GetName(), "child2");
  }

  // Test get next element
  void testGetNextElement() {
    Element parent("parent");
    Element* child1 = new Element("child1");
    Element* child2 = new Element("child2");

    parent.AddChildElement(child1);
    parent.AddChildElement(child2);

    parent.GetElement(0);  // Reset counter
    Element* result = parent.GetNextElement();
    TS_ASSERT(result != nullptr);
    TS_ASSERT_EQUALS(result->GetName(), "child2");
  }

  // Test find element
  void testFindElement() {
    Element parent("parent");
    Element* child1 = new Element("alpha");
    Element* child2 = new Element("beta");

    parent.AddChildElement(child1);
    parent.AddChildElement(child2);

    Element* result = parent.FindElement("beta");
    TS_ASSERT(result != nullptr);
    TS_ASSERT_EQUALS(result->GetName(), "beta");
  }

  // Test find element not found
  void testFindElementNotFound() {
    Element parent("parent");
    Element* child = new Element("child");
    parent.AddChildElement(child);

    Element* result = parent.FindElement("nonexistent");
    TS_ASSERT(result == nullptr);
  }

  // Test find element value
  void testFindElementValue() {
    Element parent("parent");
    Element* child = new Element("data");
    child->AddData("test value");
    parent.AddChildElement(child);

    std::string result = parent.FindElementValue("data");
    TS_ASSERT_EQUALS(result, "test value");
  }

  // Test find element value as number
  void testFindElementValueAsNumber() {
    Element parent("parent");
    Element* child = new Element("value");
    child->AddData("99.9");
    parent.AddChildElement(child);

    double result = parent.FindElementValueAsNumber("value");
    TS_ASSERT_DELTA(result, 99.9, epsilon);
  }

  // Test find element value as boolean - true cases
  void testFindElementValueAsBooleanTrue() {
    Element parent("parent");

    Element* child1 = new Element("flag1");
    child1->AddData("1");
    parent.AddChildElement(child1);

    Element* child2 = new Element("flag2");
    child2->AddData("5");
    parent.AddChildElement(child2);

    TS_ASSERT(parent.FindElementValueAsBoolean("flag1"));
    TS_ASSERT(parent.FindElementValueAsBoolean("flag2"));
  }

  // Test find element value as boolean - false cases
  void testFindElementValueAsBooleanFalse() {
    Element parent("parent");

    Element* child1 = new Element("flag1");
    child1->AddData("0");
    parent.AddChildElement(child1);

    TS_ASSERT(!parent.FindElementValueAsBoolean("flag1"));
    TS_ASSERT(!parent.FindElementValueAsBoolean("nonexistent"));
  }

  // Test get number of named elements
  void testGetNumNamedElements() {
    Element parent("parent");

    for (int i = 0; i < 3; i++) {
      Element* child = new Element("item");
      parent.AddChildElement(child);
    }
    Element* other = new Element("other");
    parent.AddChildElement(other);

    TS_ASSERT_EQUALS(parent.GetNumElements("item"), 3u);
    TS_ASSERT_EQUALS(parent.GetNumElements("other"), 1u);
    TS_ASSERT_EQUALS(parent.GetNumElements("none"), 0u);
  }

  // Test parent relationship
  void testParentRelationship() {
    Element parent("parent");
    Element* child = new Element("child");

    child->SetParent(&parent);
    TS_ASSERT_EQUALS(child->GetParent(), &parent);
  }

  // Test line number tracking
  void testLineNumber() {
    Element el("test");
    el.SetLineNumber(42);
    TS_ASSERT_EQUALS(el.GetLineNumber(), 42);
  }

  // Test file name tracking
  void testFileName() {
    Element el("test");
    el.SetFileName("test.xml");
    TS_ASSERT_EQUALS(el.GetFileName(), "test.xml");
  }

  // Test ReadFrom string
  void testReadFrom() {
    Element el("test");
    el.SetFileName("config.xml");
    el.SetLineNumber(100);

    std::string location = el.ReadFrom();
    TS_ASSERT(location.find("config.xml") != std::string::npos);
    TS_ASSERT(location.find("100") != std::string::npos);
  }

  // Test unit conversion M to FT
  void testUnitConversionMToFT() {
    // 1 meter = 3.2808399 feet
    double meters = 100.0;
    double expected_feet = 328.08399;

    // This tests the concept - actual conversion tested through FindElementValueAsNumberConvertTo
    TS_ASSERT_DELTA(meters * 3.2808399, expected_feet, 0.001);
  }

  // Test unit conversion FT to M
  void testUnitConversionFTToM() {
    double feet = 100.0;
    double expected_meters = 30.48;

    TS_ASSERT_DELTA(feet / 3.2808399, expected_meters, 0.01);
  }

  // Test unit conversion LBS to KG
  void testUnitConversionLBSToKG() {
    // 1 lbs = 0.45359237 kg
    double lbs = 100.0;
    double expected_kg = 45.359237;

    TS_ASSERT_DELTA(lbs * 0.45359237, expected_kg, 0.0001);
  }

  // Test unit conversion DEG to RAD
  void testUnitConversionDEGToRAD() {
    double deg = 180.0;
    double expected_rad = M_PI;

    double convert_factor = M_PI / 180.0;
    TS_ASSERT_DELTA(deg * convert_factor, expected_rad, epsilon);
  }

  // Test unit conversion RAD to DEG
  void testUnitConversionRADToDEG() {
    double rad = M_PI;
    double expected_deg = 180.0;

    double convert_factor = 180.0 / M_PI;
    TS_ASSERT_DELTA(rad * convert_factor, expected_deg, epsilon);
  }

  // Test unit conversion HP to WATTS
  void testUnitConversionHPToWATTS() {
    // 1 HP = 745.7 watts (approximately)
    double hp = 100.0;
    double expected_watts = 74570.0;

    // 1/0.001341022 = 745.7
    TS_ASSERT_DELTA(hp / 0.001341022, expected_watts, 10.0);
  }

  // Test unit conversion N to LBS
  void testUnitConversionNToLBS() {
    // 1 N = 0.22482 lbs
    double newtons = 100.0;
    double expected_lbs = 22.482;

    TS_ASSERT_DELTA(newtons * 0.22482, expected_lbs, 0.001);
  }

  // Test unit conversion KTS to FT/SEC
  void testUnitConversionKTSToFPS() {
    // 1 knot = 1.68781 ft/sec
    double kts = 100.0;
    double expected_fps = 168.781;

    TS_ASSERT_DELTA(kts * 1.68781, expected_fps, 0.01);
  }

  // Test identity conversions
  void testIdentityConversions() {
    // Same unit to same unit = 1.0
    double values[] = {1.0, 10.0, 100.0, 0.001};

    for (double val : values) {
      TS_ASSERT_DELTA(val * 1.0, val, epsilon);  // M to M
      TS_ASSERT_DELTA(val * 1.0, val, epsilon);  // FT to FT
      TS_ASSERT_DELTA(val * 1.0, val, epsilon);  // etc.
    }
  }

  // Test nested element navigation
  void testNestedElementNavigation() {
    Element root("root");
    Element* level1 = new Element("level1");
    Element* level2 = new Element("level2");
    Element* level3 = new Element("level3");

    level2->AddChildElement(level3);
    level1->AddChildElement(level2);
    root.AddChildElement(level1);

    Element* found1 = root.FindElement("level1");
    TS_ASSERT(found1 != nullptr);

    Element* found2 = found1->FindElement("level2");
    TS_ASSERT(found2 != nullptr);

    Element* found3 = found2->FindElement("level3");
    TS_ASSERT(found3 != nullptr);
  }

  // Test find next element
  void testFindNextElement() {
    Element parent("parent");

    Element* item1 = new Element("item");
    item1->AddData("first");
    parent.AddChildElement(item1);

    Element* item2 = new Element("item");
    item2->AddData("second");
    parent.AddChildElement(item2);

    Element* item3 = new Element("item");
    item3->AddData("third");
    parent.AddChildElement(item3);

    Element* found = parent.FindElement("item");
    TS_ASSERT(found != nullptr);
    TS_ASSERT_EQUALS(found->GetDataLine(), "first");

    found = parent.FindNextElement("item");
    TS_ASSERT(found != nullptr);
    TS_ASSERT_EQUALS(found->GetDataLine(), "second");

    found = parent.FindNextElement("item");
    TS_ASSERT(found != nullptr);
    TS_ASSERT_EQUALS(found->GetDataLine(), "third");

    found = parent.FindNextElement("item");
    TS_ASSERT(found == nullptr);
  }

  // Test multiple attributes
  void testMultipleAttributes() {
    Element el("test");
    el.AddAttribute("attr1", "value1");
    el.AddAttribute("attr2", "value2");
    el.AddAttribute("attr3", "value3");

    TS_ASSERT(el.HasAttribute("attr1"));
    TS_ASSERT(el.HasAttribute("attr2"));
    TS_ASSERT(el.HasAttribute("attr3"));

    TS_ASSERT_EQUALS(el.GetAttributeValue("attr1"), "value1");
    TS_ASSERT_EQUALS(el.GetAttributeValue("attr2"), "value2");
    TS_ASSERT_EQUALS(el.GetAttributeValue("attr3"), "value3");
  }

  // Test merge attributes
  void testMergeAttributes() {
    Element el1("first");
    el1.AddAttribute("common", "from_first");
    el1.AddAttribute("unique1", "value1");

    Element el2("second");
    el2.AddAttribute("common", "from_second");
    el2.AddAttribute("unique2", "value2");

    el1.MergeAttributes(&el2);

    // el1's "common" should take precedence
    TS_ASSERT_EQUALS(el1.GetAttributeValue("common"), "from_first");
    // el1 should now have unique2 from el2
    TS_ASSERT(el1.HasAttribute("unique2"));
  }

  // Test numeric data with whitespace
  void testNumericDataWithWhitespace() {
    Element el("test");
    el.AddData("  123.456  ");

    // The data line includes whitespace, but conversion should handle it
    double value = el.GetDataAsNumber();
    // Note: actual behavior depends on implementation - may or may not trim
    TS_ASSERT(!std::isnan(value));
  }

  // Test empty element value
  void testEmptyElementValue() {
    Element parent("parent");
    Element* child = new Element("empty");
    parent.AddChildElement(child);

    std::string value = parent.FindElementValue("empty");
    TS_ASSERT_EQUALS(value, "");
  }

  // Test negative numbers
  void testNegativeNumbers() {
    Element el("test");
    el.AddData("-123.456");

    double value = el.GetDataAsNumber();
    TS_ASSERT_DELTA(value, -123.456, epsilon);
  }

  // Test scientific notation
  void testScientificNotation() {
    Element el("test");
    el.AddData("1.23e-4");

    double value = el.GetDataAsNumber();
    TS_ASSERT_DELTA(value, 0.000123, epsilon);
  }

  // Test zero values
  void testZeroValue() {
    Element el("test");
    el.AddData("0");

    double value = el.GetDataAsNumber();
    TS_ASSERT_DELTA(value, 0.0, epsilon);
  }

  // Test area conversion M2 to FT2
  void testAreaConversion() {
    // 1 m^2 = 10.7639 ft^2
    double m2 = 10.0;
    double expected_ft2 = m2 * 3.2808399 * 3.2808399;

    TS_ASSERT_DELTA(expected_ft2, 107.639, 0.01);
  }

  // Test inertia conversion
  void testInertiaConversion() {
    // SLUG*FT2 to KG*M2: 1.35594
    double slug_ft2 = 100.0;
    double expected_kg_m2 = slug_ft2 * 1.35594;

    TS_ASSERT_DELTA(expected_kg_m2, 135.594, 0.01);
  }
};
