/*******************************************************************************
 * FGTemplateTest.h - Unit tests for FGTemplate value handling
 *
 * Tests template-based value handling in JSBSim including:
 * - Value containers
 * - Type conversions
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <cmath>
#include <limits>

#include <FGFDMExec.h>
#include <math/FGRealValue.h>
#include <math/FGPropertyValue.h>

using namespace JSBSim;

const double epsilon = 1e-10;

class FGTemplateTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * FGRealValue Tests (additional coverage)
   ***************************************************************************/

  void testRealValueZero() {
    FGRealValue rv(0.0);
    TS_ASSERT_DELTA(rv.GetValue(), 0.0, epsilon);
  }

  void testRealValuePositive() {
    FGRealValue rv(123.456);
    TS_ASSERT_DELTA(rv.GetValue(), 123.456, epsilon);
  }

  void testRealValueNegative() {
    FGRealValue rv(-789.012);
    TS_ASSERT_DELTA(rv.GetValue(), -789.012, epsilon);
  }

  void testRealValueLarge() {
    FGRealValue rv(1e15);
    TS_ASSERT_DELTA(rv.GetValue(), 1e15, 1e5);
  }

  void testRealValueSmall() {
    FGRealValue rv(1e-15);
    TS_ASSERT_DELTA(rv.GetValue(), 1e-15, 1e-25);
  }

  void testRealValuePi() {
    FGRealValue rv(M_PI);
    TS_ASSERT_DELTA(rv.GetValue(), M_PI, epsilon);
  }

  void testRealValueE() {
    FGRealValue rv(M_E);
    TS_ASSERT_DELTA(rv.GetValue(), M_E, epsilon);
  }

  void testRealValueIsConstant() {
    FGRealValue rv(42.0);
    TS_ASSERT_EQUALS(rv.IsConstant(), true);
  }

  void testRealValueMultipleReads() {
    FGRealValue rv(3.14159);

    for (int i = 0; i < 100; i++) {
      TS_ASSERT_DELTA(rv.GetValue(), 3.14159, epsilon);
    }
  }

  /***************************************************************************
   * FGPropertyValue Tests (with property manager)
   ***************************************************************************/

  void testPropertyValueFromFDMExec() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    // Create a property value for simulation time
    FGPropertyValue pv(pm->GetNode("simulation/sim-time-sec", true));

    // Initially should be 0
    TS_ASSERT_DELTA(pv.GetValue(), 0.0, epsilon);
  }

  void testPropertyValueIsNotConstant() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    FGPropertyValue pv(pm->GetNode("simulation/sim-time-sec", true));

    TS_ASSERT_EQUALS(pv.IsConstant(), false);
  }

  void testPropertyValueName() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    FGPropertyValue pv(pm->GetNode("simulation/dt", true));

    std::string name = pv.GetName();
    TS_ASSERT(name.find("dt") != std::string::npos);
  }

  void testPropertyValueUpdatesWithProperty() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/value", true);
    node->setDoubleValue(100.0);

    FGPropertyValue pv(node);

    TS_ASSERT_DELTA(pv.GetValue(), 100.0, epsilon);

    // Update the property
    node->setDoubleValue(200.0);

    TS_ASSERT_DELTA(pv.GetValue(), 200.0, epsilon);
  }

  /***************************************************************************
   * Mixed Value Type Tests
   ***************************************************************************/

  void testRealAndPropertyValuesDifferent() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    FGRealValue rv(42.0);
    FGPropertyValue pv(pm->GetNode("simulation/dt", true));

    // Real value is constant, property value is not
    TS_ASSERT_EQUALS(rv.IsConstant(), true);
    TS_ASSERT_EQUALS(pv.IsConstant(), false);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testRealValueInfinity() {
    FGRealValue rv(std::numeric_limits<double>::infinity());
    TS_ASSERT(std::isinf(rv.GetValue()));
  }

  void testRealValueNegativeInfinity() {
    FGRealValue rv(-std::numeric_limits<double>::infinity());
    TS_ASSERT(std::isinf(rv.GetValue()));
    TS_ASSERT(rv.GetValue() < 0);
  }

  void testRealValueMaxDouble() {
    FGRealValue rv(std::numeric_limits<double>::max());
    TS_ASSERT_EQUALS(rv.GetValue(), std::numeric_limits<double>::max());
  }

  void testRealValueMinDouble() {
    FGRealValue rv(std::numeric_limits<double>::min());
    TS_ASSERT_EQUALS(rv.GetValue(), std::numeric_limits<double>::min());
  }

  void testRealValueLowestDouble() {
    FGRealValue rv(std::numeric_limits<double>::lowest());
    TS_ASSERT_EQUALS(rv.GetValue(), std::numeric_limits<double>::lowest());
  }
};
