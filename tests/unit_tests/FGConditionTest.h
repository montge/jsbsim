#include <array>

#include <cxxtest/TestSuite.h>
#include <math/FGCondition.h>
#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/FGAuxiliary.h>
#include <models/FGAircraft.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGPropertyManager.h>
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

  /***************************************************************************
   * Additional Condition Tests
   ***************************************************************************/

  void testDeepNesting4Levels() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);
    auto e = pm->GetNode("e", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  <dummy logic=\"AND\">"
                                  "    a GT 0.0"
                                  "    <dummy logic=\"OR\">"
                                  "      b LT 5.0"
                                  "      <dummy logic=\"AND\">"
                                  "        c EQ 1.0\n"
                                  "        d NE 0.0"
                                  "      </dummy>"
                                  "    </dummy>"
                                  "  </dummy>"
                                  "  e GT 100.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // All false
    a->setDoubleValue(-1.0);
    b->setDoubleValue(10.0);
    c->setDoubleValue(0.0);
    d->setDoubleValue(0.0);
    e->setDoubleValue(50.0);
    TS_ASSERT(!cond.Evaluate());

    // e > 100 makes outer OR true
    e->setDoubleValue(150.0);
    TS_ASSERT(cond.Evaluate());

    // Test deep path: a>0, b<5
    e->setDoubleValue(50.0);
    a->setDoubleValue(1.0);
    b->setDoubleValue(3.0);
    TS_ASSERT(cond.Evaluate());

    // Test deepest path: a>0, b>=5, c=1, d!=0
    b->setDoubleValue(10.0);
    c->setDoubleValue(1.0);
    d->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testManyConditionsPerformance() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);
    auto d = pm->GetNode("d", true);
    auto e = pm->GetNode("e", true);
    auto f = pm->GetNode("f", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  a GT 0.0\n"
                                  "  b GT 0.0\n"
                                  "  c GT 0.0\n"
                                  "  d GT 0.0\n"
                                  "  e GT 0.0\n"
                                  "  f GT 0.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    // All positive
    a->setDoubleValue(1.0);
    b->setDoubleValue(1.0);
    c->setDoubleValue(1.0);
    d->setDoubleValue(1.0);
    e->setDoubleValue(1.0);
    f->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    // One negative should fail
    c->setDoubleValue(-1.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testVerySmallValues() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x GT 0.000001", pm, nullptr);

    x->setDoubleValue(0.0000001);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(0.000002);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(0.000001);
    TS_ASSERT(!cond.Evaluate());
  }

  void testVeryLargeNegativeValues() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x LT -999999.0", pm, nullptr);

    x->setDoubleValue(-1000000.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(-999998.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testPropertyWithIndexedName() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto prop1 = pm->GetNode("gear/unit[0]/compression-ft", true);
    auto prop2 = pm->GetNode("gear/unit[1]/compression-ft", true);

    FGCondition cond("gear/unit[0]/compression-ft GT gear/unit[1]/compression-ft", pm, nullptr);

    prop1->setDoubleValue(1.5);
    prop2->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    prop1->setDoubleValue(0.5);
    TS_ASSERT(!cond.Evaluate());
  }

  void testMultipleLevelsOfPropertyPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto prop = pm->GetNode("systems/electrical/bus/voltage", true);

    FGCondition cond("systems/electrical/bus/voltage GE 24.0", pm, nullptr);

    prop->setDoubleValue(28.0);
    TS_ASSERT(cond.Evaluate());

    prop->setDoubleValue(12.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testAllOperatorsSameProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    x->setDoubleValue(5.0);

    FGCondition eq("x EQ 5.0", pm, nullptr);
    TS_ASSERT(eq.Evaluate());

    FGCondition ne("x NE 5.0", pm, nullptr);
    TS_ASSERT(!ne.Evaluate());

    FGCondition gt("x GT 4.0", pm, nullptr);
    TS_ASSERT(gt.Evaluate());

    FGCondition ge("x GE 5.0", pm, nullptr);
    TS_ASSERT(ge.Evaluate());

    FGCondition lt("x LT 6.0", pm, nullptr);
    TS_ASSERT(lt.Evaluate());

    FGCondition le("x LE 5.0", pm, nullptr);
    TS_ASSERT(le.Evaluate());
  }

  void testConditionWithSamePropertyBothSides() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x EQ x", pm, nullptr);

    x->setDoubleValue(100.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(-100.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(0.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testConditionNEWithSameProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x NE x", pm, nullptr);

    x->setDoubleValue(100.0);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testEmptyORCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);

    // OR with two always-false conditions
    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  a GT 1000000.0\n"
                                  "  b LT -1000000.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    a->setDoubleValue(0.0);
    b->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testAlwaysTrueANDCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);

    // AND with conditions that are true for most values
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  a LT 1000000.0\n"
                                  "  b GT -1000000.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    a->setDoubleValue(0.0);
    b->setDoubleValue(0.0);
    TS_ASSERT(cond.Evaluate());

    a->setDoubleValue(999999.0);
    b->setDoubleValue(-999999.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testRapidPropertyChanges() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x GT 0.0", pm, nullptr);

    // Rapid toggling
    for (int i = 0; i < 100; ++i) {
      x->setDoubleValue(1.0);
      TS_ASSERT(cond.Evaluate());
      x->setDoubleValue(-1.0);
      TS_ASSERT(!cond.Evaluate());
    }
  }

  void testMixedANDORwithNegation() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto active = pm->GetNode("active", true);
    auto value = pm->GetNode("value", true);
    auto limit = pm->GetNode("limit", true);

    // Complex condition: active AND (value > limit OR value < -limit)
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  active EQ 1.0"
                                  "  <dummy logic=\"OR\">"
                                  "    value GT limit\n"
                                  "    value LT -10.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    active->setDoubleValue(0.0);
    value->setDoubleValue(20.0);
    limit->setDoubleValue(10.0);
    TS_ASSERT(!cond.Evaluate());  // Not active

    active->setDoubleValue(1.0);
    value->setDoubleValue(5.0);
    TS_ASSERT(!cond.Evaluate());  // Active but value in range

    value->setDoubleValue(15.0);
    TS_ASSERT(cond.Evaluate());  // Active and value > limit

    value->setDoubleValue(-15.0);
    TS_ASSERT(cond.Evaluate());  // Active and value < -10
  }

  void testConditionWithIntegerValues() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x EQ 42", pm, nullptr);

    x->setDoubleValue(42.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(42.0001);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(41.9999);
    TS_ASSERT(!cond.Evaluate());
  }

  void testPropertyUpdateBetweenEvaluations() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    FGCondition cond("x EQ y", pm, nullptr);

    x->setDoubleValue(5.0);
    y->setDoubleValue(5.0);
    TS_ASSERT(cond.Evaluate());

    // Update y between evaluations
    y->setDoubleValue(6.0);
    TS_ASSERT(!cond.Evaluate());

    // Update x to match
    x->setDoubleValue(6.0);
    TS_ASSERT(cond.Evaluate());
  }

  void testNestedWithMixedOperators() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto alt = pm->GetNode("altitude", true);
    auto speed = pm->GetNode("speed", true);
    auto gear = pm->GetNode("gear-down", true);

    // Complex flight condition
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  altitude LT 1000.0\n"
                                  "  speed LT 150.0\n"
                                  "  gear-down EQ 1.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    alt->setDoubleValue(500.0);
    speed->setDoubleValue(120.0);
    gear->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Landing configuration

    gear->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());  // Gear up

    gear->setDoubleValue(1.0);
    speed->setDoubleValue(200.0);
    TS_ASSERT(!cond.Evaluate());  // Too fast

    speed->setDoubleValue(120.0);
    alt->setDoubleValue(2000.0);
    TS_ASSERT(!cond.Evaluate());  // Too high
  }

  void testConditionStatePreservation() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond1("x GT 5.0", pm, nullptr);
    FGCondition cond2("x LT 10.0", pm, nullptr);

    // Both conditions should work independently
    x->setDoubleValue(7.0);
    TS_ASSERT(cond1.Evaluate());
    TS_ASSERT(cond2.Evaluate());

    x->setDoubleValue(3.0);
    TS_ASSERT(!cond1.Evaluate());
    TS_ASSERT(cond2.Evaluate());

    x->setDoubleValue(12.0);
    TS_ASSERT(cond1.Evaluate());
    TS_ASSERT(!cond2.Evaluate());
  }

  void testExtremelyLongPropertyPath() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto prop = pm->GetNode("systems/hydraulics/main/left/pressure-psi", true);

    FGCondition cond("systems/hydraulics/main/left/pressure-psi GE 2500.0", pm, nullptr);

    prop->setDoubleValue(3000.0);
    TS_ASSERT(cond.Evaluate());

    prop->setDoubleValue(2000.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testConditionWithUnderscoreProperty() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto prop = pm->GetNode("engine_running", true);

    FGCondition cond("engine_running EQ 1.0", pm, nullptr);

    prop->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    prop->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());
  }

  void testThreePropertiesInChain() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);

    // Test a > b > c via AND of two conditions
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  a GT b\n"
                                  "  b GT c"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    a->setDoubleValue(10.0);
    b->setDoubleValue(5.0);
    c->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    // Break the chain: b < c
    c->setDoubleValue(7.0);
    TS_ASSERT(!cond.Evaluate());

    // Fix chain again
    c->setDoubleValue(3.0);
    TS_ASSERT(cond.Evaluate());
  }
};

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DOCUMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/** Extended test suite for additional FGCondition scenarios
 */

class FGConditionExtendedTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Flight Dynamics Condition Tests
   ***************************************************************************/

  // Test 76: Stall warning condition
  void testStallWarningCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto aoa = pm->GetNode("aero/alpha-deg", true);
    auto stall_aoa = pm->GetNode("aero/stall-alpha-deg", true);
    auto weight = pm->GetNode("inertia/weight-lbs", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  aero/alpha-deg GT aero/stall-alpha-deg\n"
                                  "  inertia/weight-lbs GT 0.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    aoa->setDoubleValue(18.0);
    stall_aoa->setDoubleValue(15.0);
    weight->setDoubleValue(5000.0);
    TS_ASSERT(cond.Evaluate());  // Stall warning active

    aoa->setDoubleValue(10.0);
    TS_ASSERT(!cond.Evaluate());  // Normal flight
  }

  // Test 77: Landing gear deployment condition
  void testGearDeploymentCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto alt = pm->GetNode("position/h-agl-ft", true);
    auto speed = pm->GetNode("velocities/vc-kts", true);
    auto gear_cmd = pm->GetNode("gear/gear-cmd-norm", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  position/h-agl-ft LT 3000.0\n"
                                  "  velocities/vc-kts LT 200.0\n"
                                  "  gear/gear-cmd-norm EQ 1.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    alt->setDoubleValue(1500.0);
    speed->setDoubleValue(150.0);
    gear_cmd->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    speed->setDoubleValue(250.0);
    TS_ASSERT(!cond.Evaluate());  // Too fast for gear
  }

  // Test 78: Engine start conditions
  void testEngineStartCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto n2 = pm->GetNode("propulsion/engine/n2", true);
    auto fuel = pm->GetNode("propulsion/tank/contents-lbs", true);
    auto starter = pm->GetNode("propulsion/engine/starter-cmd", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  propulsion/engine/n2 GE 20.0\n"
                                  "  propulsion/tank/contents-lbs GT 0.0\n"
                                  "  propulsion/engine/starter-cmd EQ 1.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    n2->setDoubleValue(25.0);
    fuel->setDoubleValue(1000.0);
    starter->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    fuel->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());  // No fuel
  }

  // Test 79: Overspeed warning condition
  void testOverspeedWarning() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto mach = pm->GetNode("velocities/mach", true);
    auto ias = pm->GetNode("velocities/vc-kts", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  velocities/mach GT 0.84\n"
                                  "  velocities/vc-kts GT 350.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    mach->setDoubleValue(0.80);
    ias->setDoubleValue(320.0);
    TS_ASSERT(!cond.Evaluate());

    mach->setDoubleValue(0.86);
    TS_ASSERT(cond.Evaluate());  // Mach overspeed

    mach->setDoubleValue(0.75);
    ias->setDoubleValue(360.0);
    TS_ASSERT(cond.Evaluate());  // IAS overspeed
  }

  // Test 80: Autopilot engage conditions
  void testAutopilotEngageCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto alt = pm->GetNode("position/h-sl-ft", true);
    auto bank = pm->GetNode("attitude/phi-deg", true);
    auto pitch = pm->GetNode("attitude/theta-deg", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  position/h-sl-ft GT 1000.0\n"
                                  "  attitude/phi-deg GT -30.0\n"
                                  "  attitude/phi-deg LT 30.0\n"
                                  "  attitude/theta-deg GT -15.0\n"
                                  "  attitude/theta-deg LT 15.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    alt->setDoubleValue(5000.0);
    bank->setDoubleValue(5.0);
    pitch->setDoubleValue(3.0);
    TS_ASSERT(cond.Evaluate());  // AP can engage

    bank->setDoubleValue(45.0);
    TS_ASSERT(!cond.Evaluate());  // Bank angle too high
  }

  /***************************************************************************
   * Range and Hysteresis Pattern Tests
   ***************************************************************************/

  // Test 81: In-range check
  void testInRangeCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  x GE 10.0\n"
                                  "  x LE 20.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(5.0);
    TS_ASSERT(!cond.Evaluate());  // Below range

    x->setDoubleValue(15.0);
    TS_ASSERT(cond.Evaluate());  // In range

    x->setDoubleValue(25.0);
    TS_ASSERT(!cond.Evaluate());  // Above range

    x->setDoubleValue(10.0);
    TS_ASSERT(cond.Evaluate());  // At lower boundary

    x->setDoubleValue(20.0);
    TS_ASSERT(cond.Evaluate());  // At upper boundary
  }

  // Test 82: Out-of-range check
  void testOutOfRangeCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  x LT 10.0\n"
                                  "  x GT 20.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(15.0);
    TS_ASSERT(!cond.Evaluate());  // In range

    x->setDoubleValue(5.0);
    TS_ASSERT(cond.Evaluate());  // Below range

    x->setDoubleValue(25.0);
    TS_ASSERT(cond.Evaluate());  // Above range
  }

  // Test 83: Deadband condition
  void testDeadbandCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto input = pm->GetNode("input", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  input LT -0.1\n"
                                  "  input GT 0.1"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    input->setDoubleValue(0.05);
    TS_ASSERT(!cond.Evaluate());  // In deadband

    input->setDoubleValue(-0.05);
    TS_ASSERT(!cond.Evaluate());  // In deadband

    input->setDoubleValue(0.2);
    TS_ASSERT(cond.Evaluate());  // Outside deadband

    input->setDoubleValue(-0.2);
    TS_ASSERT(cond.Evaluate());  // Outside deadband
  }

  /***************************************************************************
   * Edge Case Value Tests
   ***************************************************************************/

  // Test 84: Comparison near zero
  void testNearZeroComparison() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond("x GT 0.0", pm, nullptr);

    x->setDoubleValue(1e-10);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(-1e-10);
    TS_ASSERT(!cond.Evaluate());

    x->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());
  }

  // Test 85: Large magnitude difference
  void testLargeMagnitudeDifference() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    FGCondition cond("x GT y", pm, nullptr);

    x->setDoubleValue(1e10);
    y->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());

    x->setDoubleValue(1.0);
    y->setDoubleValue(1e10);
    TS_ASSERT(!cond.Evaluate());
  }

  // Test 86: Equality with very close values
  void testNearEquality() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition eq("x EQ 1.0", pm, nullptr);
    FGCondition ne("x NE 1.0", pm, nullptr);

    x->setDoubleValue(1.0 + 1e-15);
    // Very close to 1.0 but not exactly equal
    // Result depends on floating point representation

    x->setDoubleValue(1.0);
    TS_ASSERT(eq.Evaluate());
    TS_ASSERT(!ne.Evaluate());
  }

  /***************************************************************************
   * Complex Multi-Variable Tests
   ***************************************************************************/

  // Test 87: Quadrant detection
  void testQuadrantDetection() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    // First quadrant: x > 0 AND y > 0
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  x GT 0.0\n"
                                  "  y GT 0.0"
                                  "</dummy>");
    FGCondition q1(elm, pm);

    x->setDoubleValue(5.0);
    y->setDoubleValue(5.0);
    TS_ASSERT(q1.Evaluate());

    x->setDoubleValue(-5.0);
    y->setDoubleValue(5.0);
    TS_ASSERT(!q1.Evaluate());

    x->setDoubleValue(5.0);
    y->setDoubleValue(-5.0);
    TS_ASSERT(!q1.Evaluate());
  }

  // Test 88: Triple OR with different operators
  void testTripleORDifferentOperators() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto a = pm->GetNode("a", true);
    auto b = pm->GetNode("b", true);
    auto c = pm->GetNode("c", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  a EQ 1.0\n"
                                  "  b GT 100.0\n"
                                  "  c LT 0.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    a->setDoubleValue(0.0);
    b->setDoubleValue(50.0);
    c->setDoubleValue(10.0);
    TS_ASSERT(!cond.Evaluate());  // All false

    a->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // a == 1

    a->setDoubleValue(0.0);
    b->setDoubleValue(150.0);
    TS_ASSERT(cond.Evaluate());  // b > 100

    b->setDoubleValue(50.0);
    c->setDoubleValue(-5.0);
    TS_ASSERT(cond.Evaluate());  // c < 0
  }

  // Test 89: Multi-system condition
  void testMultiSystemCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto hyd1 = pm->GetNode("systems/hydraulic1/pressure", true);
    auto hyd2 = pm->GetNode("systems/hydraulic2/pressure", true);
    auto elec = pm->GetNode("systems/electrical/voltage", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  systems/hydraulic1/pressure GE 2500.0\n"
                                  "  systems/hydraulic2/pressure GE 2500.0\n"
                                  "  systems/electrical/voltage GE 24.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    hyd1->setDoubleValue(3000.0);
    hyd2->setDoubleValue(3000.0);
    elec->setDoubleValue(28.0);
    TS_ASSERT(cond.Evaluate());

    hyd1->setDoubleValue(1000.0);  // Low pressure
    TS_ASSERT(!cond.Evaluate());
  }

  /***************************************************************************
   * Flight Envelope Protection Tests
   ***************************************************************************/

  // Test 90: Alpha/G limit condition
  void testAlphaGLimitCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto alpha = pm->GetNode("aero/alpha-deg", true);
    auto g_load = pm->GetNode("accelerations/n-pilot-g", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  aero/alpha-deg GT 25.0\n"
                                  "  accelerations/n-pilot-g GT 4.0\n"
                                  "  accelerations/n-pilot-g LT -1.5"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    alpha->setDoubleValue(10.0);
    g_load->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());  // Normal flight

    alpha->setDoubleValue(30.0);
    TS_ASSERT(cond.Evaluate());  // Alpha limit

    alpha->setDoubleValue(10.0);
    g_load->setDoubleValue(5.0);
    TS_ASSERT(cond.Evaluate());  // G limit

    g_load->setDoubleValue(-2.0);
    TS_ASSERT(cond.Evaluate());  // Negative G limit
  }

  // Test 91: Speed envelope condition
  void testSpeedEnvelopeCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto ias = pm->GetNode("velocities/vc-kts", true);
    auto alt = pm->GetNode("position/h-sl-ft", true);

    // Stall speed increases with altitude (simplified)
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  velocities/vc-kts GE 80.0\n"
                                  "  velocities/vc-kts LE 250.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    alt->setDoubleValue(10000.0);
    ias->setDoubleValue(150.0);
    TS_ASSERT(cond.Evaluate());

    ias->setDoubleValue(60.0);
    TS_ASSERT(!cond.Evaluate());  // Too slow

    ias->setDoubleValue(300.0);
    TS_ASSERT(!cond.Evaluate());  // Too fast
  }

  /***************************************************************************
   * State Machine Pattern Tests
   ***************************************************************************/

  // Test 92: State transition condition
  void testStateTransitionCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto state = pm->GetNode("system/state", true);
    auto input = pm->GetNode("system/input", true);

    // Transition from state 0 to state 1 when input > 0
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  system/state EQ 0.0\n"
                                  "  system/input GT 0.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    state->setDoubleValue(0.0);
    input->setDoubleValue(-1.0);
    TS_ASSERT(!cond.Evaluate());

    input->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Transition allowed

    state->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());  // Already in state 1
  }

  // Test 93: Mode inhibit condition
  void testModeInhibitCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto mode_a = pm->GetNode("mode/a-active", true);
    auto mode_b = pm->GetNode("mode/b-active", true);
    auto request = pm->GetNode("mode/b-request", true);

    // Mode B can only activate if Mode A is not active
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  mode/b-request EQ 1.0\n"
                                  "  mode/a-active NE 1.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    mode_a->setDoubleValue(0.0);
    mode_b->setDoubleValue(0.0);
    request->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Can activate B

    mode_a->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());  // A blocks B
  }

  /***************************************************************************
   * XML Parsing Edge Cases
   ***************************************************************************/

  // Test 94: Condition with newlines in XML
  void testConditionWithNewlines() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);
    auto y = pm->GetNode("y", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">\n"
                                  "  x GT 0.0\n\n"
                                  "  y LT 10.0\n"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    x->setDoubleValue(5.0);
    y->setDoubleValue(5.0);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 95: Mixed case operators
  void testMixedCaseOperators() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    // Using different case combinations
    FGCondition gt1("x GT 5.0", pm, nullptr);
    FGCondition gt2("x gt 5.0", pm, nullptr);

    x->setDoubleValue(10.0);
    TS_ASSERT(gt1.Evaluate());
    TS_ASSERT(gt2.Evaluate());

    x->setDoubleValue(3.0);
    TS_ASSERT(!gt1.Evaluate());
    TS_ASSERT(!gt2.Evaluate());
  }

  /***************************************************************************
   * Property Dynamics Tests
   ***************************************************************************/

  // Test 96: Multiple conditions same property manager
  void testMultipleConditionsSamePM() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto x = pm->GetNode("x", true);

    FGCondition cond1("x GT 0.0", pm, nullptr);
    FGCondition cond2("x LT 100.0", pm, nullptr);
    FGCondition cond3("x EQ 50.0", pm, nullptr);

    x->setDoubleValue(50.0);
    TS_ASSERT(cond1.Evaluate());
    TS_ASSERT(cond2.Evaluate());
    TS_ASSERT(cond3.Evaluate());

    x->setDoubleValue(75.0);
    TS_ASSERT(cond1.Evaluate());
    TS_ASSERT(cond2.Evaluate());
    TS_ASSERT(!cond3.Evaluate());
  }

  // Test 97: Condition with derived property
  void testDerivedPropertyCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto speed = pm->GetNode("velocity/speed-fps", true);
    auto ref = pm->GetNode("reference/speed-fps", true);

    // Error = actual - reference
    FGCondition cond("velocity/speed-fps GT reference/speed-fps", pm, nullptr);

    speed->setDoubleValue(500.0);
    ref->setDoubleValue(450.0);
    TS_ASSERT(cond.Evaluate());  // Above reference

    speed->setDoubleValue(400.0);
    TS_ASSERT(!cond.Evaluate());  // Below reference
  }

  /***************************************************************************
   * Integration Pattern Tests
   ***************************************************************************/

  // Test 98: Condition for warning system
  void testWarningSystemCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto temp = pm->GetNode("engine/egt-degf", true);
    auto warning = pm->GetNode("engine/egt-warning", true);

    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  engine/egt-degf GT 850.0\n"
                                  "  engine/egt-warning EQ 1.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    temp->setDoubleValue(800.0);
    warning->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    temp->setDoubleValue(900.0);
    TS_ASSERT(cond.Evaluate());  // High temp

    temp->setDoubleValue(750.0);
    warning->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Warning latched
  }

  // Test 99: Mode logic with priority
  void testModePriorityCondition() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto emergency = pm->GetNode("mode/emergency", true);
    auto manual = pm->GetNode("mode/manual", true);
    auto auto_mode = pm->GetNode("mode/auto", true);

    // Emergency has priority over all
    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  mode/emergency EQ 1.0"
                                  "  <dummy logic=\"AND\">"
                                  "    mode/emergency NE 1.0\n"
                                  "    mode/manual EQ 1.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    emergency->setDoubleValue(0.0);
    manual->setDoubleValue(0.0);
    auto_mode->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());

    manual->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Manual active

    emergency->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Emergency overrides
  }

  // Test 100: Condition reset logic
  void testConditionResetLogic() {
    auto pm = std::make_shared<FGPropertyManager>();
    auto trigger = pm->GetNode("trigger", true);
    auto reset = pm->GetNode("reset", true);

    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  trigger EQ 1.0\n"
                                  "  reset NE 1.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);

    trigger->setDoubleValue(0.0);
    reset->setDoubleValue(0.0);
    TS_ASSERT(!cond.Evaluate());

    trigger->setDoubleValue(1.0);
    TS_ASSERT(cond.Evaluate());  // Triggered

    reset->setDoubleValue(1.0);
    TS_ASSERT(!cond.Evaluate());  // Reset inhibits
  }
};

/*******************************************************************************
 * C172x Integration Tests for FGCondition
 *
 * Tests condition evaluation in realistic flight simulation scenarios
 * using the C172x aircraft model and its property tree.
 ******************************************************************************/
class FGConditionC172xTest : public CxxTest::TestSuite
{
public:
  // Test 1: C172x model provides valid property manager
  void testC172xPropertyManagerValid() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    auto pm = fdmex.GetPropertyManager();
    TS_ASSERT(pm != nullptr);
  }

  // Test 2: Condition on gear/wow property
  void testC172xGearWOWCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("gear/wow GE 0.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());  // WOW should be >= 0
  }

  // Test 3: Condition on velocity properties
  void testC172xVelocityCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("velocities/vc-kts GT 50.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());  // Should be going faster than 50 kts
  }

  // Test 4: Condition on throttle command
  void testC172xThrottleCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    TS_ASSERT(fcs != nullptr);
    fcs->SetThrottleCmd(-1, 0.8);

    for (int i = 0; i < 10; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("fcs/throttle-cmd-norm GT 0.5", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 5: Condition on altitude properties
  void testC172xAltitudeCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetAltitudeASLFtIC(3000.0);
    ic->SetVcalibratedKtsIC(100.0);
    TS_ASSERT(fdmex.RunIC());

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("position/h-sl-ft GT 2000.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 6: Compound condition with AND logic
  void testC172xCompoundAndCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();

    // Check that altitude and velocity conditions both hold
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  position/h-sl-ft GT 1000.0\n"
                                  "  velocities/vc-kts GT 50.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 7: Condition on elevator position
  void testC172xElevatorCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    fcs->SetDeCmd(0.5);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("fcs/elevator-cmd-norm GT 0.2", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 8: Condition on propulsion rpm
  void testC172xRPMCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    auto fcs = fdmex.GetFCS();
    fcs->SetThrottleCmd(-1, 0.7);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("propulsion/engine[0]/engine-rpm GT 500.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 9: Condition on attitude angles
  void testC172xAttitudeCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto pm = fdmex.GetPropertyManager();
    // Check that roll is within reasonable range
    FGCondition cond("attitude/phi-rad GT -1.57", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 10: Compound OR condition
  void testC172xCompoundOrCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto pm = fdmex.GetPropertyManager();

    // Either on ground or flying - one must be true
    Element_ptr elm = readFromXML("<dummy logic=\"OR\">"
                                  "  gear/wow GE 1.0\n"
                                  "  position/h-agl-ft GT 100.0"
                                  "</dummy>");
    FGCondition cond(elm, pm);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 11: Condition with aileron input
  void testC172xAileronCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    fcs->SetDaCmd(0.3);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("fcs/aileron-cmd-norm GT 0.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 12: Condition with rudder input
  void testC172xRudderCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    fcs->SetDrCmd(-0.4);

    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("fcs/rudder-cmd-norm LT 0.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 13: Condition on atmosphere density
  void testC172xAtmosphereCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("atmosphere/rho-slugs_ft3 GT 0.0", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 14: Nested condition logic
  void testC172xNestedCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));

    auto ic = fdmex.GetIC();
    ic->SetVcalibratedKtsIC(100.0);
    ic->SetAltitudeASLFtIC(5000.0);
    TS_ASSERT(fdmex.RunIC());

    auto prop = fdmex.GetPropulsion();
    if (prop) prop->InitRunning(-1);

    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();

    // Nested condition: in flight with engine running
    Element_ptr elm = readFromXML("<dummy logic=\"AND\">"
                                  "  position/h-agl-ft GT 100.0"
                                  "  <dummy logic=\"OR\">"
                                  "    propulsion/engine[0]/engine-rpm GT 1000.0\n"
                                  "    propulsion/engine[0]/set-running EQ 1.0"
                                  "  </dummy>"
                                  "</dummy>");
    FGCondition cond(elm, pm);
    TS_ASSERT(cond.Evaluate());
  }

  // Test 15: Flaps position condition
  void testC172xFlapsCondition() {
    FGFDMExec fdmex;
    TS_ASSERT(fdmex.LoadModel("c172x"));
    TS_ASSERT(fdmex.RunIC());

    auto fcs = fdmex.GetFCS();
    fcs->SetDfCmd(0.5);

    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    auto pm = fdmex.GetPropertyManager();
    FGCondition cond("fcs/flap-cmd-norm GT 0.2", pm, nullptr);
    TS_ASSERT(cond.Evaluate());
  }
};
