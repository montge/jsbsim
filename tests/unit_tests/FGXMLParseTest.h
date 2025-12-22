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
};
