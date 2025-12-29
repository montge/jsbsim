#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <sstream>
#include <input_output/FGXMLParse.h>
#include <simgear/xml/easyxml.hxx>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

const double epsilon = 1e-8;

/**
 * FGXMLParse unit tests
 *
 * Tests XML parsing functionality including:
 * - Basic XML element parsing
 * - Attribute extraction
 * - Nested element handling
 * - XML special character escaping
 * - Comment handling
 * - CDATA section handling
 * - Whitespace handling
 * - Malformed XML detection
 * - Parser reset functionality
 */
class FGXMLParseTest : public CxxTest::TestSuite
{
public:
  // Helper function to parse XML from a string
  Element* parseXMLString(const std::string& xml, FGXMLParse& parser) {
    std::istringstream iss(xml);
    try {
      readXML(iss, parser, "test_string");
      return parser.GetDocument();
    } catch (...) {
      return nullptr;
    }
  }

  // Test 1: Basic element parsing
  void testBasicElementParsing() {
    FGXMLParse parser;
    std::string xml = "<root></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "root");
  }

  // Test 2: Element with simple text content
  void testElementWithTextContent() {
    FGXMLParse parser;
    std::string xml = "<value>42.5</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "value");
    TS_ASSERT_EQUALS(doc->GetDataLine(), "42.5");
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 42.5, epsilon);
  }

  // Test 3: Element with single attribute
  void testElementWithAttribute() {
    FGXMLParse parser;
    std::string xml = "<element name=\"test\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(doc->HasAttribute("name"));
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "test");
  }

  // Test 4: Element with multiple attributes
  void testElementWithMultipleAttributes() {
    FGXMLParse parser;
    std::string xml = "<element attr1=\"value1\" attr2=\"value2\" attr3=\"value3\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(doc->HasAttribute("attr1"));
    TS_ASSERT(doc->HasAttribute("attr2"));
    TS_ASSERT(doc->HasAttribute("attr3"));
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr1"), "value1");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr2"), "value2");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr3"), "value3");
  }

  // Test 5: Nested elements - simple hierarchy
  void testNestedElements() {
    FGXMLParse parser;
    std::string xml = "<root><child>data</child></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "root");
    TS_ASSERT_EQUALS(doc->GetNumElements(), 1u);

    Element* child = doc->FindElement("child");
    TS_ASSERT(child != nullptr);
    TS_ASSERT_EQUALS(child->GetName(), "child");
    TS_ASSERT_EQUALS(child->GetDataLine(), "data");
  }

  // Test 6: Multiple child elements
  void testMultipleChildElements() {
    FGXMLParse parser;
    std::string xml = "<root><child1>data1</child1><child2>data2</child2><child3>data3</child3></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements(), 3u);

    Element* child1 = doc->FindElement("child1");
    TS_ASSERT(child1 != nullptr);
    TS_ASSERT_EQUALS(child1->GetDataLine(), "data1");

    Element* child2 = doc->FindElement("child2");
    TS_ASSERT(child2 != nullptr);
    TS_ASSERT_EQUALS(child2->GetDataLine(), "data2");

    Element* child3 = doc->FindElement("child3");
    TS_ASSERT(child3 != nullptr);
    TS_ASSERT_EQUALS(child3->GetDataLine(), "data3");
  }

  // Test 7: Deeply nested elements
  void testDeeplyNestedElements() {
    FGXMLParse parser;
    std::string xml = "<level1><level2><level3><level4>deep</level4></level3></level2></level1>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "level1");

    Element* level2 = doc->FindElement("level2");
    TS_ASSERT(level2 != nullptr);

    Element* level3 = level2->FindElement("level3");
    TS_ASSERT(level3 != nullptr);

    Element* level4 = level3->FindElement("level4");
    TS_ASSERT(level4 != nullptr);
    TS_ASSERT_EQUALS(level4->GetDataLine(), "deep");
  }

  // Test 8: XML special character escaping in text content
  void testXMLSpecialCharactersInText() {
    FGXMLParse parser;
    std::string xml = "<text>&lt;tag&gt; &amp; &quot;quote&quot;</text>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // The parser should decode the entities
    std::string data = doc->GetDataLine();
    TS_ASSERT(data.find("<") != std::string::npos);
    TS_ASSERT(data.find(">") != std::string::npos);
    TS_ASSERT(data.find("&") != std::string::npos);
  }

  // Test 9: XML special characters in attributes
  void testXMLSpecialCharactersInAttributes() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"&lt;value&gt;\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string attrValue = doc->GetAttributeValue("attr");
    TS_ASSERT(attrValue.find("<") != std::string::npos);
    TS_ASSERT(attrValue.find(">") != std::string::npos);
  }

  // Test 10: Empty element with self-closing tag
  void testEmptyElementSelfClosing() {
    FGXMLParse parser;
    std::string xml = "<root><empty/></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements(), 1u);

    Element* empty = doc->FindElement("empty");
    TS_ASSERT(empty != nullptr);
    TS_ASSERT_EQUALS(empty->GetNumDataLines(), 0u);
  }

  // Test 11: Whitespace handling in text content
  void testWhitespaceHandling() {
    FGXMLParse parser;
    std::string xml = "<data>  \n  text with spaces  \n  </data>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // The parser may preserve whitespace - just check that we got data
    TS_ASSERT(doc->GetNumDataLines() > 0);
  }

  // Test 12: Multiple data lines within an element
  void testMultipleDataLines() {
    FGXMLParse parser;
    std::string xml = "<multi>line1\nline2\nline3</multi>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // Data should be split by newlines
    TS_ASSERT(doc->GetNumDataLines() >= 1);
  }

  // Test 13: CDATA section handling
  void testCDATASection() {
    FGXMLParse parser;
    std::string xml = "<code><![CDATA[<tag>not parsed</tag>]]></code>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // CDATA content should be treated as literal text
    std::string data = doc->GetDataLine();
    TS_ASSERT(data.find("<tag>") != std::string::npos);
  }

  // Test 14: XML comments are ignored
  void testXMLComments() {
    FGXMLParse parser;
    std::string xml = "<root><!-- This is a comment --><child>data</child></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "root");
    TS_ASSERT_EQUALS(doc->GetNumElements(), 1u);

    Element* child = doc->FindElement("child");
    TS_ASSERT(child != nullptr);
    TS_ASSERT_EQUALS(child->GetDataLine(), "data");
  }

  // Test 15: Mixed content with attributes and children
  void testMixedContentStructure() {
    FGXMLParse parser;
    std::string xml = "<aircraft name=\"F-16\">"
                      "<propulsion>"
                      "<engine type=\"turbine\">engine1</engine>"
                      "</propulsion>"
                      "<aerodynamics>"
                      "<coefficient name=\"CL\" value=\"0.5\"/>"
                      "</aerodynamics>"
                      "</aircraft>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "aircraft");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "F-16");
    TS_ASSERT_EQUALS(doc->GetNumElements(), 2u);

    Element* propulsion = doc->FindElement("propulsion");
    TS_ASSERT(propulsion != nullptr);

    Element* engine = propulsion->FindElement("engine");
    TS_ASSERT(engine != nullptr);
    TS_ASSERT_EQUALS(engine->GetAttributeValue("type"), "turbine");
    TS_ASSERT_EQUALS(engine->GetDataLine(), "engine1");

    Element* aero = doc->FindElement("aerodynamics");
    TS_ASSERT(aero != nullptr);

    Element* coeff = aero->FindElement("coefficient");
    TS_ASSERT(coeff != nullptr);
    TS_ASSERT_EQUALS(coeff->GetAttributeValue("name"), "CL");
    TS_ASSERT_EQUALS(coeff->GetAttributeValue("value"), "0.5");
  }

  // Test 16: Parser reset functionality
  void testParserReset() {
    FGXMLParse parser;
    std::string xml1 = "<first>data1</first>";

    Element* doc1 = parseXMLString(xml1, parser);
    TS_ASSERT(doc1 != nullptr);
    TS_ASSERT_EQUALS(doc1->GetName(), "first");

    // Reset the parser
    parser.reset();

    // Parse a different document
    std::string xml2 = "<second>data2</second>";
    Element* doc2 = parseXMLString(xml2, parser);
    TS_ASSERT(doc2 != nullptr);
    TS_ASSERT_EQUALS(doc2->GetName(), "second");
    TS_ASSERT_EQUALS(doc2->GetDataLine(), "data2");
  }

  // Test 17: Numeric data parsing
  void testNumericDataParsing() {
    FGXMLParse parser;
    std::string xml = "<values>"
                      "<integer>42</integer>"
                      "<negative>-123</negative>"
                      "<decimal>3.14159</decimal>"
                      "<scientific>1.5e-3</scientific>"
                      "</values>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);

    Element* integer = doc->FindElement("integer");
    TS_ASSERT_DELTA(integer->GetDataAsNumber(), 42.0, epsilon);

    Element* negative = doc->FindElement("negative");
    TS_ASSERT_DELTA(negative->GetDataAsNumber(), -123.0, epsilon);

    Element* decimal = doc->FindElement("decimal");
    TS_ASSERT_DELTA(decimal->GetDataAsNumber(), 3.14159, epsilon);

    Element* scientific = doc->FindElement("scientific");
    TS_ASSERT_DELTA(scientific->GetDataAsNumber(), 0.0015, epsilon);
  }

  // Test 18: Empty document
  void testEmptyDocument() {
    FGXMLParse parser;
    std::string xml = "";

    Element* doc = parseXMLString(xml, parser);

    // Empty XML should fail to parse
    TS_ASSERT(doc == nullptr);
  }

  // Test 19: Malformed XML - unclosed tag
  void testMalformedXMLUnclosedTag() {
    FGXMLParse parser;
    std::string xml = "<root><child>data</root>"; // child not closed

    Element* doc = parseXMLString(xml, parser);

    // Should return nullptr or throw exception (caught by helper)
    TS_ASSERT(doc == nullptr);
  }

  // Test 20: Malformed XML - mismatched tags
  void testMalformedXMLMismatchedTags() {
    FGXMLParse parser;
    std::string xml = "<root><child>data</wrong></root>";

    Element* doc = parseXMLString(xml, parser);

    // Should fail to parse
    TS_ASSERT(doc == nullptr);
  }

  // Test 21: Element with attribute containing quotes
  void testAttributeWithQuotes() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"value with &quot;quotes&quot;\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string attrValue = doc->GetAttributeValue("attr");
    TS_ASSERT(attrValue.find("\"") != std::string::npos);
  }

  // Test 22: Same-named sibling elements
  void testSameNamedSiblings() {
    FGXMLParse parser;
    std::string xml = "<root>"
                      "<item>first</item>"
                      "<item>second</item>"
                      "<item>third</item>"
                      "</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements("item"), 3u);

    Element* item = doc->FindElement("item");
    TS_ASSERT(item != nullptr);
    TS_ASSERT_EQUALS(item->GetDataLine(), "first");

    item = doc->FindNextElement("item");
    TS_ASSERT(item != nullptr);
    TS_ASSERT_EQUALS(item->GetDataLine(), "second");

    item = doc->FindNextElement("item");
    TS_ASSERT(item != nullptr);
    TS_ASSERT_EQUALS(item->GetDataLine(), "third");
  }

  // Test 23: Parent-child relationships
  void testParentChildRelationships() {
    FGXMLParse parser;
    std::string xml = "<root><parent><child>data</child></parent></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);

    Element* parent = doc->FindElement("parent");
    TS_ASSERT(parent != nullptr);
    TS_ASSERT_EQUALS(parent->GetParent(), doc);

    Element* child = parent->FindElement("child");
    TS_ASSERT(child != nullptr);
    TS_ASSERT_EQUALS(child->GetParent(), parent);
  }

  // Test 24: Attribute-only element
  void testAttributeOnlyElement() {
    FGXMLParse parser;
    std::string xml = "<config debug=\"true\" verbose=\"false\" level=\"3\"/>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "config");
    TS_ASSERT_EQUALS(doc->GetNumDataLines(), 0u);
    TS_ASSERT(doc->HasAttribute("debug"));
    TS_ASSERT(doc->HasAttribute("verbose"));
    TS_ASSERT(doc->HasAttribute("level"));
  }

  // Test 25: Line number and file name tracking
  void testLineNumberAndFileNameTracking() {
    FGXMLParse parser;
    std::string xml = "<root>\n<child>data</child>\n</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // The root element should have line number set
    TS_ASSERT(doc->GetLineNumber() > 0);
    // File name should be set to our test string identifier
    TS_ASSERT_EQUALS(doc->GetFileName(), "test_string");

    Element* child = doc->FindElement("child");
    TS_ASSERT(child != nullptr);
    // Child should have a different line number
    TS_ASSERT(child->GetLineNumber() > doc->GetLineNumber());
  }

  // Test 26: XML declaration handling
  void testXMLDeclaration() {
    FGXMLParse parser;
    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><root>data</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "root");
    TS_ASSERT_EQUALS(doc->GetDataLine(), "data");
  }

  // Test 27: Attribute with numeric value
  void testAttributeWithNumericValue() {
    FGXMLParse parser;
    std::string xml = "<value number=\"123.456\"></value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    double numValue = doc->GetAttributeValueAsNumber("number");
    TS_ASSERT_DELTA(numValue, 123.456, epsilon);
  }

  // Test 28: Complex JSBSim-style XML structure
  void testComplexJSBSimStyleXML() {
    FGXMLParse parser;
    std::string xml = "<?xml version=\"1.0\"?>"
                      "<fdm_config name=\"test_aircraft\">"
                      "<fileheader>"
                      "<author>Test Author</author>"
                      "<email>test@jsbsim.org</email>"
                      "</fileheader>"
                      "<metrics>"
                      "<wingarea unit=\"FT2\">174.0</wingarea>"
                      "<wingspan unit=\"FT\">35.8</wingspan>"
                      "<chord unit=\"FT\">4.86</chord>"
                      "</metrics>"
                      "<mass_balance>"
                      "<ixx unit=\"SLUG*FT2\">3530</ixx>"
                      "<iyy unit=\"SLUG*FT2\">3150</iyy>"
                      "<izz unit=\"SLUG*FT2\">6380</izz>"
                      "<emptywt unit=\"LBS\">7000</emptywt>"
                      "</mass_balance>"
                      "</fdm_config>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "fdm_config");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "test_aircraft");

    Element* fileheader = doc->FindElement("fileheader");
    TS_ASSERT(fileheader != nullptr);
    TS_ASSERT_EQUALS(fileheader->FindElementValue("author"), "Test Author");
    TS_ASSERT_EQUALS(fileheader->FindElementValue("email"), "test@jsbsim.org");

    Element* metrics = doc->FindElement("metrics");
    TS_ASSERT(metrics != nullptr);

    Element* wingarea = metrics->FindElement("wingarea");
    TS_ASSERT(wingarea != nullptr);
    TS_ASSERT_EQUALS(wingarea->GetAttributeValue("unit"), "FT2");
    TS_ASSERT_DELTA(wingarea->GetDataAsNumber(), 174.0, epsilon);

    Element* mass_balance = doc->FindElement("mass_balance");
    TS_ASSERT(mass_balance != nullptr);

    Element* emptywt = mass_balance->FindElement("emptywt");
    TS_ASSERT(emptywt != nullptr);
    TS_ASSERT_EQUALS(emptywt->GetAttributeValue("unit"), "LBS");
    TS_ASSERT_DELTA(emptywt->GetDataAsNumber(), 7000.0, epsilon);
  }

  // Test 29: Element name with hyphens
  void testElementNameWithHyphens() {
    FGXMLParse parser;
    std::string xml = "<root><mass-balance>100</mass-balance></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* massBalance = doc->FindElement("mass-balance");
    TS_ASSERT(massBalance != nullptr);
    TS_ASSERT_DELTA(massBalance->GetDataAsNumber(), 100.0, epsilon);
  }

  // Test 30: Element name with underscores
  void testElementNameWithUnderscores() {
    FGXMLParse parser;
    std::string xml = "<root><wing_area>150</wing_area></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* wingArea = doc->FindElement("wing_area");
    TS_ASSERT(wingArea != nullptr);
    TS_ASSERT_DELTA(wingArea->GetDataAsNumber(), 150.0, epsilon);
  }

  // Test 31: Empty attribute value
  void testEmptyAttributeValue() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(doc->HasAttribute("attr"));
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr"), "");
  }

  // Test 32: Attribute with single quotes
  void testAttributeWithSingleQuotes() {
    FGXMLParse parser;
    std::string xml = "<element attr='value'></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(doc->HasAttribute("attr"));
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr"), "value");
  }

  // Test 33: FindElementValue method
  void testFindElementValue() {
    FGXMLParse parser;
    std::string xml = "<root><name>TestName</name><value>42</value></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->FindElementValue("name"), "TestName");
    TS_ASSERT_EQUALS(doc->FindElementValue("value"), "42");
  }

  // Test 34: FindElementValue for non-existent element
  void testFindElementValueNonExistent() {
    FGXMLParse parser;
    std::string xml = "<root><name>Test</name></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string value = doc->FindElementValue("nonexistent");
    TS_ASSERT_EQUALS(value, "");
  }

  // Test 35: GetNumElements without name filter
  void testGetNumElementsTotal() {
    FGXMLParse parser;
    std::string xml = "<root><a>1</a><b>2</b><c>3</c><a>4</a></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements(), 4u);
    TS_ASSERT_EQUALS(doc->GetNumElements("a"), 2u);
    TS_ASSERT_EQUALS(doc->GetNumElements("b"), 1u);
    TS_ASSERT_EQUALS(doc->GetNumElements("c"), 1u);
    TS_ASSERT_EQUALS(doc->GetNumElements("d"), 0u);
  }

  // Test 36: GetElement by index
  void testGetElementByIndex() {
    FGXMLParse parser;
    std::string xml = "<root><first>1</first><second>2</second><third>3</third></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);

    Element* first = doc->GetElement(0);
    TS_ASSERT(first != nullptr);
    TS_ASSERT_EQUALS(first->GetName(), "first");

    Element* second = doc->GetElement(1);
    TS_ASSERT(second != nullptr);
    TS_ASSERT_EQUALS(second->GetName(), "second");

    Element* third = doc->GetElement(2);
    TS_ASSERT(third != nullptr);
    TS_ASSERT_EQUALS(third->GetName(), "third");
  }

  // Test 37: Zero numeric value
  void testZeroNumericValue() {
    FGXMLParse parser;
    std::string xml = "<value>0</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 0.0, epsilon);
  }

  // Test 38: Very small numeric value
  void testVerySmallNumericValue() {
    FGXMLParse parser;
    std::string xml = "<value>1e-15</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 1e-15, 1e-20);
  }

  // Test 39: Very large numeric value
  void testVeryLargeNumericValue() {
    FGXMLParse parser;
    std::string xml = "<value>1e15</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 1e15, 1e10);
  }

  // Test 40: Negative scientific notation
  void testNegativeScientificNotation() {
    FGXMLParse parser;
    std::string xml = "<value>-3.5e-2</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), -0.035, epsilon);
  }

  // Test 41: Multiple same-name elements iteration
  void testMultipleSameNameIteration() {
    FGXMLParse parser;
    std::string xml = "<root><point>1</point><point>2</point><point>3</point><point>4</point></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);

    std::vector<double> values;
    Element* point = doc->FindElement("point");
    while (point != nullptr) {
      values.push_back(point->GetDataAsNumber());
      point = doc->FindNextElement("point");
    }

    TS_ASSERT_EQUALS(values.size(), 4u);
    TS_ASSERT_DELTA(values[0], 1.0, epsilon);
    TS_ASSERT_DELTA(values[1], 2.0, epsilon);
    TS_ASSERT_DELTA(values[2], 3.0, epsilon);
    TS_ASSERT_DELTA(values[3], 4.0, epsilon);
  }

  // Test 42: Processing instruction (if supported)
  void testProcessingInstruction() {
    FGXMLParse parser;
    std::string xml = "<?xml version=\"1.0\"?><?custom data=\"value\"?><root>content</root>";

    Element* doc = parseXMLString(xml, parser);

    // Processing instructions should be ignored
    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "root");
  }

  // Test 43: Tabs and mixed whitespace
  void testTabsAndMixedWhitespace() {
    FGXMLParse parser;
    std::string xml = "<root>\t<child>\t\tdata\t</child>\t</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* child = doc->FindElement("child");
    TS_ASSERT(child != nullptr);
  }

  // Test 44: Attribute value with newlines
  void testAttributeValueWithWhitespace() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"value with spaces\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr"), "value with spaces");
  }

  // Test 45: Checking non-existent attribute
  void testNonExistentAttribute() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"value\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(!doc->HasAttribute("nonexistent"));
    TS_ASSERT_EQUALS(doc->GetAttributeValue("nonexistent"), "");
  }

  // Test 46: GetAttributeValueAsNumber with valid attribute
  void testAttributeAsNumberValid() {
    FGXMLParse parser;
    std::string xml = "<element count=\"42\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(doc->HasAttribute("count"));
    double value = doc->GetAttributeValueAsNumber("count");
    TS_ASSERT_DELTA(value, 42.0, epsilon);
  }

  // Test 47: Apostrophe in attribute (escaped)
  void testApostropheInAttribute() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"it&apos;s\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string value = doc->GetAttributeValue("attr");
    TS_ASSERT(value.find("'") != std::string::npos);
  }

  // Test 48: Deeply nested with mixed children
  void testDeeplyNestedMixedChildren() {
    FGXMLParse parser;
    std::string xml = "<l1><l2a>a</l2a><l2b><l3>deep</l3></l2b></l1>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements(), 2u);

    Element* l2b = doc->FindElement("l2b");
    TS_ASSERT(l2b != nullptr);

    Element* l3 = l2b->FindElement("l3");
    TS_ASSERT(l3 != nullptr);
    TS_ASSERT_EQUALS(l3->GetDataLine(), "deep");
  }

  // Test 49: JSBSim function-style XML
  void testJSBSimFunctionStyleXML() {
    FGXMLParse parser;
    std::string xml = "<function name=\"aero/coefficient/CLalpha\">"
                      "<description>Lift coefficient due to alpha</description>"
                      "<product>"
                      "<property>aero/qbar-psf</property>"
                      "<property>metrics/Sw-sqft</property>"
                      "<value>4.5</value>"
                      "</product>"
                      "</function>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "function");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "aero/coefficient/CLalpha");

    Element* desc = doc->FindElement("description");
    TS_ASSERT(desc != nullptr);

    Element* product = doc->FindElement("product");
    TS_ASSERT(product != nullptr);
    TS_ASSERT_EQUALS(product->GetNumElements(), 3u);

    Element* value = product->FindElement("value");
    TS_ASSERT(value != nullptr);
    TS_ASSERT_DELTA(value->GetDataAsNumber(), 4.5, epsilon);
  }

  // Test 50: JSBSim table-style XML
  void testJSBSimTableStyleXML() {
    FGXMLParse parser;
    std::string xml = "<table name=\"CLalpha\" type=\"internal\">"
                      "<independentVar lookup=\"row\">aero/alpha-rad</independentVar>"
                      "<tableData>"
                      "-0.20  -0.680\n"
                      " 0.00   0.200\n"
                      " 0.23   1.200\n"
                      " 0.60   0.600"
                      "</tableData>"
                      "</table>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "table");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "CLalpha");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("type"), "internal");

    Element* indVar = doc->FindElement("independentVar");
    TS_ASSERT(indVar != nullptr);
    TS_ASSERT_EQUALS(indVar->GetAttributeValue("lookup"), "row");

    Element* tableData = doc->FindElement("tableData");
    TS_ASSERT(tableData != nullptr);
    TS_ASSERT(tableData->GetNumDataLines() > 0);
  }

  // Test 51: Attribute with path-like value
  void testAttributeWithPathValue() {
    FGXMLParse parser;
    std::string xml = "<property>propulsion/engine[0]/thrust-lbs</property>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetDataLine(), "propulsion/engine[0]/thrust-lbs");
  }

  // Test 52: Boolean-like attribute values
  void testBooleanLikeAttributes() {
    FGXMLParse parser;
    std::string xml = "<element enabled=\"true\" disabled=\"false\" flag=\"1\" noflag=\"0\"/>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetAttributeValue("enabled"), "true");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("disabled"), "false");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("flag"), "1");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("noflag"), "0");
  }

  // Test 53: Data with plus sign
  void testDataWithPlusSign() {
    FGXMLParse parser;
    std::string xml = "<value>+42.5</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 42.5, epsilon);
  }

  // Test 54: Multiple CDATA sections
  void testMultipleCDATASections() {
    FGXMLParse parser;
    std::string xml = "<code><![CDATA[part1]]><![CDATA[part2]]></code>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string data = doc->GetDataLine();
    TS_ASSERT(data.find("part1") != std::string::npos);
  }

  // Test 55: Long element name
  void testLongElementName() {
    FGXMLParse parser;
    std::string xml = "<very_long_element_name_that_goes_on_and_on>data</very_long_element_name_that_goes_on_and_on>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "very_long_element_name_that_goes_on_and_on");
  }

  // Test 56: Long attribute value
  void testLongAttributeValue() {
    FGXMLParse parser;
    std::string longValue = std::string(200, 'a');
    std::string xml = "<element attr=\"" + longValue + "\"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetAttributeValue("attr").length(), 200u);
  }

  // Test 57: Numeric attribute leading/trailing spaces
  void testNumericAttributeSpaces() {
    FGXMLParse parser;
    std::string xml = "<element value=\" 42.5 \"></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // Parser may trim spaces
    double val = doc->GetAttributeValueAsNumber("value");
    TS_ASSERT_DELTA(val, 42.5, epsilon);
  }

  // Test 58: Element with only whitespace content
  void testElementWithOnlyWhitespace() {
    FGXMLParse parser;
    std::string xml = "<element>   \n\t   </element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // Should parse successfully even with whitespace-only content
  }

  // Test 59: Carriage return handling
  void testCarriageReturnHandling() {
    FGXMLParse parser;
    std::string xml = "<root>\r\n<child>data</child>\r\n</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* child = doc->FindElement("child");
    TS_ASSERT(child != nullptr);
    TS_ASSERT_EQUALS(child->GetDataLine(), "data");
  }

  // Test 60: Pure numeric data extraction
  void testPureNumericData() {
    FGXMLParse parser;
    std::string xml = "<value> 42.5 </value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // GetDataAsNumber extracts numeric value (whitespace trimmed)
    double val = doc->GetDataAsNumber();
    TS_ASSERT_DELTA(val, 42.5, epsilon);
  }

  // Test 61: Deeply nested same-named elements
  void testDeeplyNestedSameNamedElements() {
    FGXMLParse parser;
    std::string xml = "<group><group><group>deep</group></group></group>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "group");

    Element* level2 = doc->FindElement("group");
    TS_ASSERT(level2 != nullptr);

    Element* level3 = level2->FindElement("group");
    TS_ASSERT(level3 != nullptr);
    TS_ASSERT_EQUALS(level3->GetDataLine(), "deep");
  }

  // Test 62: Complex attribute names
  void testComplexAttributeNames() {
    FGXMLParse parser;
    std::string xml = "<element data-type=\"number\" xml-lang=\"en\" my_attr=\"val\"/>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT(doc->HasAttribute("data-type"));
    TS_ASSERT(doc->HasAttribute("xml-lang"));
    TS_ASSERT(doc->HasAttribute("my_attr"));
  }

  // Test 63: Sibling and first element access
  void testSiblingAndFirstElement() {
    FGXMLParse parser;
    std::string xml = "<root><a>1</a><b>2</b><c>3</c></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);

    // Get first element
    Element* first = doc->GetElement(0);
    TS_ASSERT(first != nullptr);
    TS_ASSERT_EQUALS(first->GetName(), "a");

    // Check we can iterate through all children
    Element* current = doc->GetElement(0);
    int count = 0;
    while (current != nullptr) {
      count++;
      current = doc->GetElement(count);
    }
    TS_ASSERT_EQUALS(count, 3);
  }

  /***************************************************************************
   * Additional Malformed XML Tests
   ***************************************************************************/

  // Test 64: Malformed XML - missing root element
  void testMalformedXMLMissingRoot() {
    FGXMLParse parser;
    std::string xml = "<child>data</child><sibling>more</sibling>";

    Element* doc = parseXMLString(xml, parser);

    // Multiple root elements should fail (well-formed XML has one root)
    // Note: Some parsers may accept the first element only
    // The behavior depends on the parser implementation
    if (doc != nullptr) {
      // If it parses, it should at least get the first element
      TS_ASSERT_EQUALS(doc->GetName(), "child");
    }
  }

  // Test 65: Malformed XML - illegal character in element name
  void testMalformedXMLIllegalCharInName() {
    FGXMLParse parser;
    std::string xml = "<123element>data</123element>";

    Element* doc = parseXMLString(xml, parser);

    // Element names cannot start with numbers
    TS_ASSERT(doc == nullptr);
  }

  // Test 66: Malformed XML - unclosed attribute
  void testMalformedXMLUnclosedAttribute() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"value></element>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc == nullptr);
  }

  // Test 67: Malformed XML - duplicate attributes
  void testMalformedXMLDuplicateAttributes() {
    FGXMLParse parser;
    std::string xml = "<element attr=\"value1\" attr=\"value2\"></element>";

    Element* doc = parseXMLString(xml, parser);

    // Duplicate attributes are not well-formed XML
    // Parser may either reject or use last value
    if (doc != nullptr) {
      TS_ASSERT(doc->HasAttribute("attr"));
    }
  }

  /***************************************************************************
   * JSBSim-Specific XML Pattern Tests
   ***************************************************************************/

  // Test 68: JSBSim channel definition
  void testJSBSimChannelDefinition() {
    FGXMLParse parser;
    std::string xml = "<channel name=\"pitch\">"
                      "<summer name=\"Pitch Trim Sum\">"
                      "<input>fcs/elevator-cmd-norm</input>"
                      "<input>fcs/pitch-trim-cmd-norm</input>"
                      "</summer>"
                      "<aerosurface_scale name=\"Elevator Control\">"
                      "<input>fcs/pitch-trim-sum</input>"
                      "<range>"
                      "<min>-0.35</min>"
                      "<max>0.3</max>"
                      "</range>"
                      "</aerosurface_scale>"
                      "</channel>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "channel");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "pitch");

    Element* summer = doc->FindElement("summer");
    TS_ASSERT(summer != nullptr);
    TS_ASSERT_EQUALS(summer->GetNumElements("input"), 2u);

    Element* scale = doc->FindElement("aerosurface_scale");
    TS_ASSERT(scale != nullptr);

    Element* range = scale->FindElement("range");
    TS_ASSERT(range != nullptr);

    Element* minElem = range->FindElement("min");
    TS_ASSERT(minElem != nullptr);
    TS_ASSERT_DELTA(minElem->GetDataAsNumber(), -0.35, epsilon);
  }

  // Test 69: JSBSim ground reactions
  void testJSBSimGroundReactions() {
    FGXMLParse parser;
    std::string xml = "<contact type=\"BOGEY\" name=\"NOSE\">"
                      "<location unit=\"IN\">"
                      "<x>33</x>"
                      "<y>0</y>"
                      "<z>-54</z>"
                      "</location>"
                      "<static_friction>0.8</static_friction>"
                      "<dynamic_friction>0.5</dynamic_friction>"
                      "<spring_coeff unit=\"LBS/FT\">1200</spring_coeff>"
                      "<damping_coeff unit=\"LBS/FT/SEC\">200</damping_coeff>"
                      "</contact>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "contact");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("type"), "BOGEY");

    Element* location = doc->FindElement("location");
    TS_ASSERT(location != nullptr);
    TS_ASSERT_EQUALS(location->GetAttributeValue("unit"), "IN");

    Element* staticFric = doc->FindElement("static_friction");
    TS_ASSERT(staticFric != nullptr);
    TS_ASSERT_DELTA(staticFric->GetDataAsNumber(), 0.8, epsilon);
  }

  // Test 70: JSBSim autopilot PID controller
  void testJSBSimAutopilotPID() {
    FGXMLParse parser;
    std::string xml = "<pid name=\"heading-hold\">"
                      "<input>ap/heading-error</input>"
                      "<kp>0.1</kp>"
                      "<ki>0.01</ki>"
                      "<kd>0.05</kd>"
                      "<clipto>"
                      "<min>-1.0</min>"
                      "<max>1.0</max>"
                      "</clipto>"
                      "<output>ap/heading-command</output>"
                      "</pid>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "pid");

    Element* kp = doc->FindElement("kp");
    TS_ASSERT(kp != nullptr);
    TS_ASSERT_DELTA(kp->GetDataAsNumber(), 0.1, epsilon);

    Element* clipto = doc->FindElement("clipto");
    TS_ASSERT(clipto != nullptr);
    TS_ASSERT_EQUALS(clipto->GetNumElements(), 2u);
  }

  /***************************************************************************
   * Edge Case Value Tests
   ***************************************************************************/

  // Test 71: Infinity representation
  void testInfinityValue() {
    FGXMLParse parser;
    std::string xml = "<value>inf</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // Parser behavior for "inf" varies - just check it parses
  }

  // Test 72: Negative zero
  void testNegativeZero() {
    FGXMLParse parser;
    std::string xml = "<value>-0.0</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 0.0, epsilon);
  }

  // Test 73: Extremely deep nesting
  void testExtremelyDeepNesting() {
    FGXMLParse parser;
    std::string xml = "<l1><l2><l3><l4><l5><l6><l7><l8><l9><l10>deep</l10></l9></l8></l7></l6></l5></l4></l3></l2></l1>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "l1");

    // Navigate to deepest level
    Element* current = doc;
    for (int i = 2; i <= 10; i++) {
      std::string name = "l" + std::to_string(i);
      current = current->FindElement(name);
      TS_ASSERT(current != nullptr);
    }
    TS_ASSERT_EQUALS(current->GetDataLine(), "deep");
  }

  // Test 74: Many attributes on single element
  void testManyAttributes() {
    FGXMLParse parser;
    std::string xml = "<element a1=\"v1\" a2=\"v2\" a3=\"v3\" a4=\"v4\" a5=\"v5\" "
                      "a6=\"v6\" a7=\"v7\" a8=\"v8\" a9=\"v9\" a10=\"v10\"/>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    for (int i = 1; i <= 10; i++) {
      std::string attrName = "a" + std::to_string(i);
      std::string attrValue = "v" + std::to_string(i);
      TS_ASSERT(doc->HasAttribute(attrName));
      TS_ASSERT_EQUALS(doc->GetAttributeValue(attrName), attrValue);
    }
  }

  // Test 75: Many children elements
  void testManyChildren() {
    FGXMLParse parser;
    std::string xml = "<root>";
    for (int i = 0; i < 50; i++) {
      xml += "<item>" + std::to_string(i) + "</item>";
    }
    xml += "</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements("item"), 50u);
  }

  /***************************************************************************
   * Special Character and Encoding Tests
   ***************************************************************************/

  // Test 76: Numeric character references
  void testNumericCharacterReferences() {
    FGXMLParse parser;
    std::string xml = "<text>&#60;tag&#62;</text>";  // < and > as numeric refs

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string data = doc->GetDataLine();
    TS_ASSERT(data.find("<") != std::string::npos);
    TS_ASSERT(data.find(">") != std::string::npos);
  }

  // Test 77: Mixed content and elements
  void testMixedContentElements() {
    FGXMLParse parser;
    std::string xml = "<root>text before <child>child content</child> text after</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* child = doc->FindElement("child");
    TS_ASSERT(child != nullptr);
  }

  // Test 78: Element with namespace-like prefix
  void testNamespaceLikePrefix() {
    FGXMLParse parser;
    std::string xml = "<root><jsbsim:config>data</jsbsim:config></root>";

    Element* doc = parseXMLString(xml, parser);

    // May or may not parse depending on namespace support
    if (doc != nullptr) {
      TS_ASSERT_EQUALS(doc->GetName(), "root");
    }
  }

  /***************************************************************************
   * Parser State and Reuse Tests
   ***************************************************************************/

  // Test 79: Multiple documents sequential parsing
  void testMultipleDocumentsSequential() {
    FGXMLParse parser;

    for (int i = 0; i < 5; i++) {
      parser.reset();
      std::string xml = "<doc" + std::to_string(i) + ">data" + std::to_string(i) + "</doc" + std::to_string(i) + ">";
      Element* doc = parseXMLString(xml, parser);

      TS_ASSERT(doc != nullptr);
      TS_ASSERT_EQUALS(doc->GetName(), "doc" + std::to_string(i));
    }
  }

  // Test 80: Parse after failure
  void testParseAfterFailure() {
    FGXMLParse parser;

    // First, try to parse malformed XML
    std::string badXml = "<broken>";
    Element* badDoc = parseXMLString(badXml, parser);
    TS_ASSERT(badDoc == nullptr);

    // Reset and parse valid XML
    parser.reset();
    std::string goodXml = "<valid>data</valid>";
    Element* goodDoc = parseXMLString(goodXml, parser);

    TS_ASSERT(goodDoc != nullptr);
    TS_ASSERT_EQUALS(goodDoc->GetName(), "valid");
  }

  /***************************************************************************
   * Additional Data Format Tests
   ***************************************************************************/

  // Test 81: Hexadecimal-like numeric value
  void testHexLikeNumericValue() {
    FGXMLParse parser;
    std::string xml = "<value>0x1A</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    // Most XML parsers treat this as a string, not a hex number
    TS_ASSERT_EQUALS(doc->GetDataLine(), "0x1A");
  }

  // Test 82: Data line with multiple values
  void testDataLineMultipleValues() {
    FGXMLParse parser;
    std::string xml = "<coords>1.0 2.0 3.0</coords>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    std::string data = doc->GetDataLine();
    TS_ASSERT(data.find("1.0") != std::string::npos);
    TS_ASSERT(data.find("2.0") != std::string::npos);
    TS_ASSERT(data.find("3.0") != std::string::npos);
  }

  // Test 83: Scientific notation with capital E
  void testScientificNotationCapitalE() {
    FGXMLParse parser;
    std::string xml = "<value>1.5E+10</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 1.5e10, 1e5);
  }

  // Test 84: Leading zeros in numeric value
  void testLeadingZerosNumeric() {
    FGXMLParse parser;
    std::string xml = "<value>007.5</value>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_DELTA(doc->GetDataAsNumber(), 7.5, epsilon);
  }

  /***************************************************************************
   * Stress and Boundary Tests
   ***************************************************************************/

  // Test 85: Very long text content
  void testVeryLongTextContent() {
    FGXMLParse parser;
    std::string longContent = std::string(5000, 'x');
    std::string xml = "<data>" + longContent + "</data>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetDataLine().length(), 5000u);
  }

  // Test 86: Many nested same-level elements
  void testManyNestedSameLevelElements() {
    FGXMLParse parser;
    std::string xml = "<root>";
    for (int i = 0; i < 100; i++) {
      xml += "<item" + std::to_string(i) + ">v" + std::to_string(i) + "</item" + std::to_string(i) + ">";
    }
    xml += "</root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetNumElements(), 100u);
  }

  // Test 87: Rapid parser reuse
  void testRapidParserReuse() {
    FGXMLParse parser;

    for (int i = 0; i < 100; i++) {
      parser.reset();
      std::string xml = "<test>" + std::to_string(i) + "</test>";
      Element* doc = parseXMLString(xml, parser);

      TS_ASSERT(doc != nullptr);
      TS_ASSERT_DELTA(doc->GetDataAsNumber(), static_cast<double>(i), epsilon);
    }
  }

  /***************************************************************************
   * JSBSim Real-World Pattern Tests
   ***************************************************************************/

  // Test 88: JSBSim engine definition
  void testJSBSimEngineDefinition() {
    FGXMLParse parser;
    std::string xml = "<turbine_engine name=\"F100-PW-200\">"
                      "<milthrust unit=\"LBS\">14590</milthrust>"
                      "<maxthrust unit=\"LBS\">23770</maxthrust>"
                      "<bypassratio>0.6</bypassratio>"
                      "<tsfc>0.97</tsfc>"
                      "<atsfc>1.9</atsfc>"
                      "<idlen1>65.0</idlen1>"
                      "<idlen2>60.0</idlen2>"
                      "</turbine_engine>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "turbine_engine");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "F100-PW-200");

    Element* milthrust = doc->FindElement("milthrust");
    TS_ASSERT(milthrust != nullptr);
    TS_ASSERT_EQUALS(milthrust->GetAttributeValue("unit"), "LBS");
    TS_ASSERT_DELTA(milthrust->GetDataAsNumber(), 14590.0, 1.0);

    Element* bypassratio = doc->FindElement("bypassratio");
    TS_ASSERT(bypassratio != nullptr);
    TS_ASSERT_DELTA(bypassratio->GetDataAsNumber(), 0.6, epsilon);
  }

  // Test 89: JSBSim script event
  void testJSBSimScriptEvent() {
    FGXMLParse parser;
    std::string xml = "<event name=\"Start Engine\">"
                      "<condition>"
                      "simulation/sim-time-sec ge 0.5"
                      "</condition>"
                      "<set name=\"propulsion/engine/set-running\" value=\"1\"/>"
                      "<notify>"
                      "<property>propulsion/engine/n1</property>"
                      "</notify>"
                      "</event>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "event");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "Start Engine");

    Element* condition = doc->FindElement("condition");
    TS_ASSERT(condition != nullptr);

    Element* set = doc->FindElement("set");
    TS_ASSERT(set != nullptr);
    TS_ASSERT_EQUALS(set->GetAttributeValue("name"), "propulsion/engine/set-running");
    TS_ASSERT_EQUALS(set->GetAttributeValue("value"), "1");
  }

  // Test 90: JSBSim 2D table
  void testJSBSim2DTable() {
    FGXMLParse parser;
    std::string xml = "<table>"
                      "<independentVar lookup=\"row\">velocities/mach</independentVar>"
                      "<independentVar lookup=\"column\">aero/alpha-rad</independentVar>"
                      "<tableData>\n"
                      "       -0.2   0.0   0.2\n"
                      " 0.0   -0.1   0.0   0.1\n"
                      " 0.5   -0.2   0.0   0.2\n"
                      " 1.0   -0.3   0.0   0.3\n"
                      "</tableData>"
                      "</table>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "table");
    TS_ASSERT_EQUALS(doc->GetNumElements("independentVar"), 2u);

    Element* tableData = doc->FindElement("tableData");
    TS_ASSERT(tableData != nullptr);
    TS_ASSERT(tableData->GetNumDataLines() > 0);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteAircraftConfigParsing() {
    FGXMLParse parser;
    std::string xml = "<fdm_config name=\"c172\" version=\"2.0\">"
                      "<fileheader>"
                      "<author>Test Author</author>"
                      "<filecreationdate>2024-01-01</filecreationdate>"
                      "</fileheader>"
                      "<metrics>"
                      "<wingspan unit=\"FT\">36.1</wingspan>"
                      "<chord unit=\"FT\">4.9</chord>"
                      "</metrics>"
                      "</fdm_config>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetName(), "fdm_config");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("name"), "c172");

    Element* metrics = doc->FindElement("metrics");
    TS_ASSERT(metrics != nullptr);

    Element* wingspan = metrics->FindElement("wingspan");
    TS_ASSERT(wingspan != nullptr);
    TS_ASSERT_DELTA(wingspan->GetDataAsNumber(), 36.1, 0.01);
  }

  void testCompleteSystemsParsing() {
    FGXMLParse parser;
    std::string xml = "<system name=\"fuel\">"
                      "<channel name=\"left\">"
                      "<summer name=\"total\">"
                      "<input>propulsion/tank[0]/contents-lbs</input>"
                      "<input>propulsion/tank[1]/contents-lbs</input>"
                      "</summer>"
                      "</channel>"
                      "</system>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* channel = doc->FindElement("channel");
    TS_ASSERT(channel != nullptr);
    Element* summer = channel->FindElement("summer");
    TS_ASSERT(summer != nullptr);
    TS_ASSERT_EQUALS(summer->GetNumElements("input"), 2u);
  }

  void testCompleteNestedElementAccess() {
    FGXMLParse parser;
    std::string xml = "<root><a><b><c><d>value</d></c></b></a></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    Element* a = doc->FindElement("a");
    TS_ASSERT(a != nullptr);
    Element* b = a->FindElement("b");
    TS_ASSERT(b != nullptr);
    Element* c = b->FindElement("c");
    TS_ASSERT(c != nullptr);
    Element* d = c->FindElement("d");
    TS_ASSERT(d != nullptr);
    TS_ASSERT_EQUALS(d->GetDataLine(), "value");
  }

  void testCompleteAttributeValueTypes() {
    FGXMLParse parser;
    std::string xml = "<elem int=\"42\" float=\"3.14\" bool=\"true\" str=\"hello\"/>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);
    TS_ASSERT_EQUALS(doc->GetAttributeValue("int"), "42");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("float"), "3.14");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("bool"), "true");
    TS_ASSERT_EQUALS(doc->GetAttributeValue("str"), "hello");
  }

  void testCompleteDocumentTraversal() {
    FGXMLParse parser;
    std::string xml = "<root><child1>a</child1><child2>b</child2><child3>c</child3></root>";

    Element* doc = parseXMLString(xml, parser);

    TS_ASSERT(doc != nullptr);

    int childCount = 0;
    Element* child = doc->GetElement();
    while (child) {
      childCount++;
      child = doc->GetNextElement();
    }

    TS_ASSERT_EQUALS(childCount, 3);
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentParserInstances() {
    FGXMLParse parser1;
    FGXMLParse parser2;

    std::string xml1 = "<doc1>data1</doc1>";
    std::string xml2 = "<doc2>data2</doc2>";

    Element* doc1 = parseXMLString(xml1, parser1);
    Element* doc2 = parseXMLString(xml2, parser2);

    TS_ASSERT(doc1 != nullptr);
    TS_ASSERT(doc2 != nullptr);
    TS_ASSERT_EQUALS(doc1->GetName(), "doc1");
    TS_ASSERT_EQUALS(doc2->GetName(), "doc2");
  }

  void testIndependentDocumentModification() {
    FGXMLParse parser;

    parser.reset();
    std::string xml1 = "<elem>value1</elem>";
    Element* doc1 = parseXMLString(xml1, parser);
    std::string val1 = doc1->GetDataLine();

    parser.reset();
    std::string xml2 = "<elem>value2</elem>";
    Element* doc2 = parseXMLString(xml2, parser);
    std::string val2 = doc2->GetDataLine();

    TS_ASSERT_EQUALS(val1, "value1");
    TS_ASSERT_EQUALS(val2, "value2");
  }

  void testIndependentElementQueries() {
    FGXMLParse parser;
    std::string xml = "<root><a>1</a><b>2</b></root>";

    Element* doc = parseXMLString(xml, parser);

    Element* a = doc->FindElement("a");
    Element* b = doc->FindElement("b");

    TS_ASSERT(a != nullptr);
    TS_ASSERT(b != nullptr);
    TS_ASSERT_EQUALS(a->GetDataLine(), "1");
    TS_ASSERT_EQUALS(b->GetDataLine(), "2");

    // Query a again
    Element* a2 = doc->FindElement("a");
    TS_ASSERT_EQUALS(a2->GetDataLine(), "1");
  }

  void testIndependentAttributeAccess() {
    FGXMLParse parser;
    std::string xml = "<elem attr1=\"val1\" attr2=\"val2\"/>";

    Element* doc = parseXMLString(xml, parser);

    std::string v1 = doc->GetAttributeValue("attr1");
    std::string v2 = doc->GetAttributeValue("attr2");

    TS_ASSERT_EQUALS(v1, "val1");
    TS_ASSERT_EQUALS(v2, "val2");

    // Access attr1 again
    std::string v1_again = doc->GetAttributeValue("attr1");
    TS_ASSERT_EQUALS(v1_again, "val1");
  }

  void testIndependentNumericParsing() {
    FGXMLParse parser;

    parser.reset();
    std::string xml1 = "<val>123.45</val>";
    Element* doc1 = parseXMLString(xml1, parser);
    double n1 = doc1->GetDataAsNumber();

    parser.reset();
    std::string xml2 = "<val>678.90</val>";
    Element* doc2 = parseXMLString(xml2, parser);
    double n2 = doc2->GetDataAsNumber();

    TS_ASSERT_DELTA(n1, 123.45, epsilon);
    TS_ASSERT_DELTA(n2, 678.90, epsilon);
  }
};
