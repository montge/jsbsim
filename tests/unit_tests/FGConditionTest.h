#include <array>

#include <cxxtest/TestSuite.h>
#include <math/FGCondition.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;


class FGConditionTest : public CxxTest::TestSuite
{
public:
  void testXMLEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> XML{"<dummy> x == 1.0 </dummy>",
                                         "<dummy> x EQ 1.0 </dummy>",
                                         "<dummy> x eq 1.0 </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> conditions{"x == 1.0", "x EQ 1.0", "x eq 1.0"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testXMLNotEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> XML{"<dummy> x != 1.0 </dummy>",
                                         "<dummy> x NE 1.0 </dummy>",
                                         "<dummy> x ne 1.0 </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testNotEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> conditions{"x != 1.0", "x NE 1.0", "x ne 1.0"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testXMLGreaterThanConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> XML{"<dummy> x &gt; 1.0 </dummy>",
                                "<dummy> x GT 1.0 </dummy>",
                                "<dummy> x gt 1.0 </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testGreaterThanConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> conditions{"x > 1.0", "x GT 1.0", "x gt 1.0"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testXMLGreaterOrEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> XML{"<dummy> x &gt;= 1.0 </dummy>",
                                "<dummy> x GE 1.0 </dummy>",
                                "<dummy> x ge 1.0 </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testGreaterOrEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> conditions{"x >= 1.0", "x GE 1.0", "x ge 1.0"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testXMLLowerThanConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> XML{"<dummy> x &lt; 1.0 </dummy>",
                                "<dummy> x LT 1.0 </dummy>",
                                "<dummy> x lt 1.0 </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testLowerThanConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> conditions{"x < 1.0", "x LT 1.0", "x lt 1.0"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testXMLLowerOrEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> XML{"<dummy> x &lt;= 1.0 </dummy>",
                                         "<dummy> x LE 1.0 </dummy>",
                                         "<dummy> x le 1.0 </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testLowerOrEqualConstant() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    const std::array<std::string, 3> conditions{"x <= 1.0", "x LE 1.0", "x le 1.0"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testXMLEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> XML{"<dummy> x == y </dummy>",
                                         "<dummy> x EQ y </dummy>",
                                         "<dummy> x eq y </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> conditions{"x == y", "x EQ y", "x eq y"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testXMLNotEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> XML{"<dummy> x != y </dummy>",
                                         "<dummy> x NE y </dummy>",
                                         "<dummy> x ne y </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(0.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testNotEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> conditions{"x != y", "x NE y", "x ne y"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(0.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testXMLGreaterThanProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> XML{"<dummy> x &gt; y </dummy>",
                                         "<dummy> x GT y </dummy>",
                                         "<dummy> x gt y </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testGreaterThanProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> conditions{"x > y", "x GT y", "x gt y"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testXMLGreaterOrEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> XML{"<dummy> x &gt;= y </dummy>",
                                         "<dummy> x GE y </dummy>",
                                         "<dummy> x ge y </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testGreaterOrEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> conditions{"x >= y", "x GE y", "x ge y"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());
    }
  }

  void testXMLLowerThanProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> XML{"<dummy> x &lt; y </dummy>",
                                         "<dummy> x LT y </dummy>",
                                         "<dummy> x lt y </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testLowerThanProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> conditions{"x < y", "x LT y", "x lt y"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testXMLLowerOrEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> XML{"<dummy> x &lt;= y </dummy>",
                                         "<dummy> x LE y </dummy>",
                                         "<dummy> x le y </dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testLowerOrEqualProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 3> conditions{"x <= y", "x LE y", "x le y"};
    for(const std::string& line: conditions) {
      FGCondition cond(line, pm, nullptr);

      x->setDoubleValue(-1.0);
      y->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(0.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testAND() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto onoff = pm->GetNode("on-off", true);
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    const std::array<std::string, 2> XML{"<dummy> on-off == 1\nx GE y</dummy>",
                                         "<dummy logic=\"AND\"> on-off == 1\nx GE y</dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      onoff->setDoubleValue(0.0);
      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());

      onoff->setDoubleValue(1.0);
      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());

      y->setDoubleValue(3.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testANDLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    const std::array<std::string, 2> XML{"<dummy> on-off == 1\nx GE y</dummy>",
                                         "<dummy logic=\"AND\"> on-off == 1\nx GE y</dummy>"};
    for(const std::string& line: XML) {
      Element_ptr elm = readFromXML(line);
      FGCondition cond(elm, pm);

      auto onoff = pm->GetNode("on-off", true);
      auto x = pm->GetNode("x", true);
      auto y = pm->GetNode("y", true);

      onoff->setDoubleValue(0.0);
      x->setDoubleValue(0.0);
      y->setDoubleValue(1.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(2.0);
      TS_ASSERT(!cond.Evaluate());

      onoff->setDoubleValue(1.0);
      x->setDoubleValue(0.0);
      TS_ASSERT(!cond.Evaluate());

      x->setDoubleValue(2.0);
      TS_ASSERT(cond.Evaluate());

      y->setDoubleValue(3.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testOR() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto onoff = pm->GetNode("on-off", true);
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "on-off == 1\n"
                                  "x GE y"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    onoff->setDoubleValue(0.0);
    x->setDoubleValue(0.0);
    y->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(2.0);
    TS_ASSERT(cond.Evaluate());

    y->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());

    onoff->setDoubleValue(1.0);
    x->setDoubleValue(4.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(2.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto onoff = pm->GetNode("on-off", true);
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    Element_ptr elm = readFromXML("<dummy>"
                                  "  on-off == 1"
                                  "  <dummy logic=\"AND\">"
                                  "    x GE y\n"
                                  "    x LT 2.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    onoff->setDoubleValue(0.0);
    x->setDoubleValue(0.0);
    y->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(1.5);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());

    onoff->setDoubleValue(1.0);
    x->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    y->setDoubleValue(-1.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testIllegalLOGIC() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy logic=\"XOR\">"
                                  "  on-off == 1\n"
                                  "  x GE y"
                                  "</dummy>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);
  }

  void testWrongNumberOfElements() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy> on-off == </dummy>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);

    elm = readFromXML("<dummy> on-off </dummy>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);

    elm = readFromXML("<dummy/>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);

    elm = readFromXML("<dummy> 0.0 LE on-off GE 1.0 </dummy>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);
  }

  void testIllegalNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy>"
                                  "  on-off == 1"
                                  "  <crash logic=\"AND\">"
                                  "    x GE y\n"
                                  "    x LT 2.0"
                                  "  </crash>"
                                  "</dummy>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);
  }

  void testIllegalOperation() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy> on-off # 0.0 </dummy>");
    TS_ASSERT_THROWS(FGCondition cond(elm, pm), BaseException&);
  }

  void testORLateBound() {
    auto pm = std::make_shared<FGPropertyManager>();
    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "on-off == 1\n"
                                  "x GE y"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    auto onoff = pm->GetNode("on-off", true);
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    onoff->setDoubleValue(0.0);
    x->setDoubleValue(0.0);
    y->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(2.0);
    TS_ASSERT(cond.Evaluate());

    y->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());

    onoff->setDoubleValue(1.0);
    x->setDoubleValue(4.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(2.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testComplexNestedANDinOR() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  <dummy logic=\"AND\">"
                                  "    a GT 0.0\n"
                                  "    b LT 10.0"
                                  "  </dummy>"
                                  "  <dummy logic=\"AND\">"
                                  "    c EQ 5.0\n"
                                  "    d NE 0.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // Both AND conditions false
    a->setDoubleValue(-1.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    // First AND condition true
    a->setDoubleValue(5.0);
    b->setDoubleValue(5.0);
    TS_ASSERT(cond.Evaluate());

    // First AND false, second AND true
    a->setDoubleValue(-1.0);
    b->setDoubleValue(15.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    // Both AND conditions true
    a->setDoubleValue(5.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testComplexNestedORinAND() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  <dummy logic=\"OR\">"
                                  "    a GT 0.0\n"
                                  "    b LT 10.0"
                                  "  </dummy>"
                                  "  <dummy logic=\"OR\">"
                                  "    c EQ 5.0\n"
                                  "    d NE 0.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // Both OR conditions false
    a->setDoubleValue(-1.0);
    b->setDoubleValue(15.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    // First OR true, second OR false
    a->setDoubleValue(5.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    // First OR false, second OR true
    a->setDoubleValue(-1.0);
    b->setDoubleValue(15.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());

    // Both OR conditions true
    a->setDoubleValue(5.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    // First OR true (b<10), second OR true (d!=0)
    a->setDoubleValue(-1.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testThreeLevelNesting() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  a GT 0.0"
                                  "  <dummy logic=\"OR\">"
                                  "    b LT 10.0"
                                  "    <dummy logic=\"AND\">"
                                  "      c EQ 5.0\n"
                                  "      d NE 0.0"
                                  "    </dummy>"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // Top level AND: a must be > 0
    a->setDoubleValue(-1.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());

    // a > 0, but nested OR is false
    a->setDoubleValue(1.0);
    b->setDoubleValue(15.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    // a > 0, OR true via b < 10
    a->setDoubleValue(1.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(cond.Evaluate());

    // a > 0, OR true via nested AND
    a->setDoubleValue(1.0);
    b->setDoubleValue(15.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    // All conditions true
    a->setDoubleValue(1.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testNegativeValues() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    // Test negative constant
    FGCondition cond1("x GT -5.0", pm, nullptr);
    x->setDoubleValue(-10.0);
    TS_ASSERT(!cond1.Evaluate());
    x->setDoubleValue(-3.0);
    TS_ASSERT(cond1.Evaluate());
    x->setDoubleValue(0.0);
    TS_ASSERT(cond1.Evaluate());

    // Test negative property values
    FGCondition cond2("x LT y", pm, nullptr);
    x->setDoubleValue(-10.0);
    y->setDoubleValue(-5.0);
    TS_ASSERT(cond2.Evaluate());
    x->setDoubleValue(-5.0);
    y->setDoubleValue(-10.0);
    TS_ASSERT(!cond2.Evaluate());

    // Test equality with negative values
    FGCondition cond3("x EQ -3.5", pm, nullptr);
    x->setDoubleValue(-3.5);
    TS_ASSERT(cond3.Evaluate());
    x->setDoubleValue(3.5);
    TS_ASSERT(!cond3.Evaluate());
  }

  void testMultipleConditionsInAND() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  a GT 0.0\n"
                                  "  b LT 100.0\n"
                                  "  c GE 5.0\n"
                                  "  d LE 10.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // All conditions false
    a->setDoubleValue(-1.0);
    b->setDoubleValue(150.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(15.0);
    TS_ASSERT(!cond.Evaluate());

    // First condition false, others true
    a->setDoubleValue(-1.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(7.0);
    d->setDoubleValue(8.0);
    TS_ASSERT(!cond.Evaluate());

    // Last condition false, others true
    a->setDoubleValue(1.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(7.0);
    d->setDoubleValue(15.0);
    TS_ASSERT(!cond.Evaluate());

    // All conditions true
    a->setDoubleValue(1.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(7.0);
    d->setDoubleValue(8.0);
    TS_ASSERT(cond.Evaluate());

    // Test boundary values
    a->setDoubleValue(0.001);
    b->setDoubleValue(99.999);
    c->setDoubleValue(5.0);
    d->setDoubleValue(10.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testMultipleConditionsInOR() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  a GT 100.0\n"
                                  "  b LT 0.0\n"
                                  "  c EQ 5.0\n"
                                  "  d NE 0.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // All conditions false
    a->setDoubleValue(50.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    // Only first condition true
    a->setDoubleValue(150.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(cond.Evaluate());

    // Only second condition true
    a->setDoubleValue(50.0);
    b->setDoubleValue(-5.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(cond.Evaluate());

    // Only third condition true
    a->setDoubleValue(50.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(0.0);
    TS_ASSERT(cond.Evaluate());

    // Only last condition true
    a->setDoubleValue(50.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(3.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    // All conditions true
    a->setDoubleValue(150.0);
    b->setDoubleValue(-5.0);
    c->setDoubleValue(5.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testPrintConditionSimple() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Redirect cout to capture output
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

    FGCondition cond("x GT 5.0", pm, nullptr);
    cond.PrintCondition();

    std::string output = buffer.str();
    TS_ASSERT(output.find("x") != std::string::npos);
    TS_ASSERT(output.find("GT") != std::string::npos);

    // Restore cout
    std::cout.rdbuf(old);
  }

  void testPrintConditionAND() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    // Redirect cout to capture output
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  x GT 0.0\n"
                                  "  y LT 10.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);
    cond.PrintCondition();

    std::string output = buffer.str();
    TS_ASSERT(output.find("if all of the following are true") != std::string::npos);
    TS_ASSERT(output.find("x") != std::string::npos);
    TS_ASSERT(output.find("y") != std::string::npos);

    // Restore cout
    std::cout.rdbuf(old);
  }

  void testPrintConditionOR() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    // Redirect cout to capture output
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  x GT 0.0\n"
                                  "  y LT 10.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);
    cond.PrintCondition();

    std::string output = buffer.str();
    TS_ASSERT(output.find("if any of the following are true") != std::string::npos);
    TS_ASSERT(output.find("x") != std::string::npos);
    TS_ASSERT(output.find("y") != std::string::npos);

    // Restore cout
    std::cout.rdbuf(old);
  }

  void testPrintConditionNested() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);

    // Redirect cout to capture output
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  a GT 0.0"
                                  "  <dummy logic=\"OR\">"
                                  "    b LT 10.0\n"
                                  "    c EQ 5.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);
    cond.PrintCondition();

    std::string output = buffer.str();
    TS_ASSERT(output.find("if all of the following are true") != std::string::npos);
    TS_ASSERT(output.find("if any of the following are true") != std::string::npos);
    TS_ASSERT(output.find("a") != std::string::npos);
    TS_ASSERT(output.find("b") != std::string::npos);
    TS_ASSERT(output.find("c") != std::string::npos);

    // Restore cout
    std::cout.rdbuf(old);
  }

  void testMixedXMLAndStringConditions() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);
    auto z = pm->GetNode("z", true);

    // XML-based condition with multiple string conditions inside
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  x GT 0.0\n"
                                  "  y LT 100.0\n"
                                  "  z GE 5.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(5.0);
    y->setDoubleValue(50.0);
    z->setDoubleValue(10.0);
    TS_ASSERT(cond.Evaluate());

    z->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());

    z->setDoubleValue(5.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testPropertyComparison() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto altitude = pm->GetNode("position/altitude", true);
    auto target = pm->GetNode("target/altitude", true);

    // Test with hierarchical property names
    FGCondition cond("position/altitude GE target/altitude", pm, nullptr);

    altitude->setDoubleValue(1000.0);
    target->setDoubleValue(500.0);
    TS_ASSERT(cond.Evaluate());

    altitude->setDoubleValue(300.0);
    TS_ASSERT(!cond.Evaluate());

    altitude->setDoubleValue(500.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testFloatingPointComparison() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Test with floating point values
    FGCondition cond1("x EQ 1.23456789", pm, nullptr);
    x->setDoubleValue(1.23456789);
    TS_ASSERT(cond1.Evaluate());

    x->setDoubleValue(1.23456788);
    TS_ASSERT(!cond1.Evaluate());

    // Test with scientific notation-like values
    FGCondition cond2("x GT 0.0001", pm, nullptr);
    x->setDoubleValue(0.0002);
    TS_ASSERT(cond2.Evaluate());

    x->setDoubleValue(0.00001);
    TS_ASSERT(!cond2.Evaluate());
  }

  void testZeroComparison() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond1("x EQ 0.0", pm, nullptr);
    x->setDoubleValue(0.0);
    TS_ASSERT(cond1.Evaluate());

    FGCondition cond2("x NE 0.0", pm, nullptr);
    x->setDoubleValue(0.0);
    TS_ASSERT(!cond2.Evaluate());
    x->setDoubleValue(0.0001);
    TS_ASSERT(cond2.Evaluate());
    x->setDoubleValue(-0.0001);
    TS_ASSERT(cond2.Evaluate());
  }

  void testLargeValues() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x LT 1000000.0", pm, nullptr);
    x->setDoubleValue(999999.9);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(1000000.1);
    TS_ASSERT(!cond.Evaluate());
  }

  void testEqualityBoundary() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Test >= at boundary
    FGCondition cond1("x GE 10.0", pm, nullptr);
    x->setDoubleValue(9.999);
    TS_ASSERT(!cond1.Evaluate());
    x->setDoubleValue(10.0);
    TS_ASSERT(cond1.Evaluate());
    x->setDoubleValue(10.001);
    TS_ASSERT(cond1.Evaluate());

    // Test <= at boundary
    FGCondition cond2("x LE 10.0", pm, nullptr);
    x->setDoubleValue(9.999);
    TS_ASSERT(cond2.Evaluate());
    x->setDoubleValue(10.0);
    TS_ASSERT(cond2.Evaluate());
    x->setDoubleValue(10.001);
    TS_ASSERT(!cond2.Evaluate());
  }

  void testSingleConditionInAND() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Test AND with single condition (should work)
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  x GT 5.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());
    x->setDoubleValue(7.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testSingleConditionInOR() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Test OR with single condition (should work)
    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  x LT 5.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(3.0);
    TS_ASSERT(cond.Evaluate());
    x->setDoubleValue(7.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testConditionWithWhitespace() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Test parsing with extra whitespace
    Element_ptr elm = readFromXML("<dummy>   x   GT   5.0   </dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());
    x->setDoubleValue(7.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testComplexPropertyNames() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto prop1 = pm->GetNode("fcs/left-aileron-pos-rad", true);
    auto prop2 = pm->GetNode("fcs/right-aileron-pos-rad", true);

    FGCondition cond("fcs/left-aileron-pos-rad EQ fcs/right-aileron-pos-rad", pm, nullptr);

    prop1->setDoubleValue(0.5);
    prop2->setDoubleValue(0.5);
    TS_ASSERT(cond.Evaluate());

    prop2->setDoubleValue(0.3);
    TS_ASSERT(!cond.Evaluate());
  }

  void testANDShortCircuit() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  x GT 10.0\n"
                                  "  y LT 5.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // First condition false - should evaluate to false regardless
    x->setDoubleValue(5.0);
    y->setDoubleValue(3.0);
    TS_ASSERT(!cond.Evaluate());

    // Both conditions true
    x->setDoubleValue(15.0);
    y->setDoubleValue(3.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testORShortCircuit() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  x GT 10.0\n"
                                  "  y LT 5.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // First condition true - should evaluate to true regardless
    x->setDoubleValue(15.0);
    y->setDoubleValue(10.0);
    TS_ASSERT(cond.Evaluate());

    // Both conditions false
    x->setDoubleValue(5.0);
    y->setDoubleValue(10.0);
    TS_ASSERT(!cond.Evaluate());
  }
};
