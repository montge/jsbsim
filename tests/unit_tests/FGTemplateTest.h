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

  /***************************************************************************
   * Additional FGRealValue Tests
   ***************************************************************************/

  void testRealValueGetName() {
    FGRealValue rv(42.0);
    std::string name = rv.GetName();
    // Should return a string representation of the value
    TS_ASSERT(!name.empty());
  }

  void testRealValueDenormalized() {
    // Test denormalized (subnormal) numbers
    double denorm = std::numeric_limits<double>::denorm_min();
    FGRealValue rv(denorm);
    TS_ASSERT_EQUALS(rv.GetValue(), denorm);
  }

  void testRealValueEpsilon() {
    FGRealValue rv(std::numeric_limits<double>::epsilon());
    TS_ASSERT_EQUALS(rv.GetValue(), std::numeric_limits<double>::epsilon());
  }

  void testRealValueNaN() {
    FGRealValue rv(std::numeric_limits<double>::quiet_NaN());
    TS_ASSERT(std::isnan(rv.GetValue()));
  }

  void testRealValueSignalingNaN() {
    FGRealValue rv(std::numeric_limits<double>::signaling_NaN());
    TS_ASSERT(std::isnan(rv.GetValue()));
  }

  void testRealValueConsistency() {
    // Same value should give same result every time
    double val = 123.456789012345;
    FGRealValue rv(val);
    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(rv.GetValue(), val);
    }
  }

  void testRealValueMultipleInstances() {
    FGRealValue rv1(1.0);
    FGRealValue rv2(2.0);
    FGRealValue rv3(3.0);

    TS_ASSERT_DELTA(rv1.GetValue(), 1.0, epsilon);
    TS_ASSERT_DELTA(rv2.GetValue(), 2.0, epsilon);
    TS_ASSERT_DELTA(rv3.GetValue(), 3.0, epsilon);
  }

  void testRealValueNegativeZero() {
    FGRealValue rv(-0.0);
    TS_ASSERT_DELTA(rv.GetValue(), 0.0, epsilon);
  }

  void testRealValueScientificNotation() {
    FGRealValue rv1(1.23e10);
    FGRealValue rv2(4.56e-10);

    TS_ASSERT_DELTA(rv1.GetValue(), 1.23e10, 1e5);
    TS_ASSERT_DELTA(rv2.GetValue(), 4.56e-10, 1e-15);
  }

  /***************************************************************************
   * Additional FGPropertyValue Tests
   ***************************************************************************/

  void testPropertyValueSetValue() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/settable", true);
    FGPropertyValue pv(node);

    pv.SetValue(42.0);
    TS_ASSERT_DELTA(pv.GetValue(), 42.0, epsilon);

    pv.SetValue(-100.0);
    TS_ASSERT_DELTA(pv.GetValue(), -100.0, epsilon);
  }

  void testPropertyValueGetFullyQualifiedName() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("simulation/sim-time-sec", true);
    FGPropertyValue pv(node);

    std::string fqn = pv.GetFullyQualifiedName();
    TS_ASSERT(!fqn.empty());
    TS_ASSERT(fqn.find("sim-time-sec") != std::string::npos);
  }

  void testPropertyValueGetPrintableName() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("simulation/dt", true);
    FGPropertyValue pv(node);

    std::string printable = pv.GetPrintableName();
    TS_ASSERT(!printable.empty());
  }

  void testPropertyValueGetNameWithSign() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/value", true);
    FGPropertyValue pv(node);

    std::string nameWithSign = pv.GetNameWithSign();
    TS_ASSERT(!nameWithSign.empty());
  }

  void testPropertyValueIsLateBound() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/value", true);
    FGPropertyValue pv(node);

    // When constructed with a node, it's not late bound
    TS_ASSERT_EQUALS(pv.IsLateBound(), false);
  }

  void testPropertyValueMultipleProperties() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node1 = pm->GetNode("test/prop1", true);
    auto node2 = pm->GetNode("test/prop2", true);
    auto node3 = pm->GetNode("test/prop3", true);

    node1->setDoubleValue(10.0);
    node2->setDoubleValue(20.0);
    node3->setDoubleValue(30.0);

    FGPropertyValue pv1(node1);
    FGPropertyValue pv2(node2);
    FGPropertyValue pv3(node3);

    TS_ASSERT_DELTA(pv1.GetValue(), 10.0, epsilon);
    TS_ASSERT_DELTA(pv2.GetValue(), 20.0, epsilon);
    TS_ASSERT_DELTA(pv3.GetValue(), 30.0, epsilon);
  }

  void testPropertyValueDynamicUpdate() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/dynamic", true);
    FGPropertyValue pv(node);

    // Update multiple times
    for (int i = 0; i < 10; i++) {
      double val = i * 10.0;
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(pv.GetValue(), val, epsilon);
    }
  }

  void testPropertyValueNegativeValues() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/negative", true);
    node->setDoubleValue(-999.999);

    FGPropertyValue pv(node);
    TS_ASSERT_DELTA(pv.GetValue(), -999.999, epsilon);
  }

  void testPropertyValueZero() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/zero", true);
    node->setDoubleValue(0.0);

    FGPropertyValue pv(node);
    TS_ASSERT_DELTA(pv.GetValue(), 0.0, epsilon);
  }

  void testPropertyValueLargeValue() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/large", true);
    node->setDoubleValue(1e12);

    FGPropertyValue pv(node);
    TS_ASSERT_DELTA(pv.GetValue(), 1e12, 1e6);
  }

  void testPropertyValueSmallValue() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/small", true);
    node->setDoubleValue(1e-12);

    FGPropertyValue pv(node);
    TS_ASSERT_DELTA(pv.GetValue(), 1e-12, 1e-18);
  }

  /***************************************************************************
   * Multiple FDMExec Instance Tests
   ***************************************************************************/

  void testPropertyValueMultipleFDMExec() {
    FGFDMExec fdmex1;
    FGFDMExec fdmex2;

    auto pm1 = fdmex1.GetPropertyManager();
    auto pm2 = fdmex2.GetPropertyManager();

    auto node1 = pm1->GetNode("test/value", true);
    auto node2 = pm2->GetNode("test/value", true);

    node1->setDoubleValue(100.0);
    node2->setDoubleValue(200.0);

    FGPropertyValue pv1(node1);
    FGPropertyValue pv2(node2);

    // They should be independent
    TS_ASSERT_DELTA(pv1.GetValue(), 100.0, epsilon);
    TS_ASSERT_DELTA(pv2.GetValue(), 200.0, epsilon);
  }

  /***************************************************************************
   * SetNode Tests
   ***************************************************************************/

  void testPropertyValueSetNode() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node1 = pm->GetNode("test/node1", true);
    auto node2 = pm->GetNode("test/node2", true);

    node1->setDoubleValue(111.0);
    node2->setDoubleValue(222.0);

    FGPropertyValue pv(node1);
    TS_ASSERT_DELTA(pv.GetValue(), 111.0, epsilon);

    // Change to different node
    pv.SetNode(node2);
    TS_ASSERT_DELTA(pv.GetValue(), 222.0, epsilon);
  }

  /***************************************************************************
   * Stress Tests
   ***************************************************************************/

  void testManyRealValues() {
    std::vector<FGRealValue> values;
    for (int i = 0; i < 100; i++) {
      values.emplace_back(static_cast<double>(i));
    }

    for (int i = 0; i < 100; i++) {
      TS_ASSERT_DELTA(values[i].GetValue(), static_cast<double>(i), epsilon);
    }
  }

  void testManyPropertyValues() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    std::vector<SGPropertyNode*> nodes;
    for (int i = 0; i < 50; i++) {
      std::string name = "test/stress/prop" + std::to_string(i);
      auto node = pm->GetNode(name, true);
      node->setDoubleValue(static_cast<double>(i * 10));
      nodes.push_back(node);
    }

    for (int i = 0; i < 50; i++) {
      FGPropertyValue pv(nodes[i]);
      TS_ASSERT_DELTA(pv.GetValue(), static_cast<double>(i * 10), epsilon);
    }
  }

  void testRapidPropertyUpdates() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/rapid", true);
    FGPropertyValue pv(node);

    for (int i = 0; i < 100; i++) {
      double val = i * 0.1;
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(pv.GetValue(), val, epsilon);
    }
  }

  /***************************************************************************
   * Comparison Tests
   ***************************************************************************/

  void testRealValueVsPropertyValue() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    double testValue = 42.42;
    FGRealValue rv(testValue);

    auto node = pm->GetNode("test/compare", true);
    node->setDoubleValue(testValue);
    FGPropertyValue pv(node);

    // Both should return the same value
    TS_ASSERT_DELTA(rv.GetValue(), pv.GetValue(), epsilon);

    // But IsConstant differs
    TS_ASSERT_EQUALS(rv.IsConstant(), true);
    // PropertyValue's constness depends on node attributes
  }

  void testRealValueNameFormat() {
    FGRealValue rv1(0.0);
    FGRealValue rv2(1.0);
    FGRealValue rv3(-1.0);
    FGRealValue rv4(3.14159);

    // All should return non-empty names
    TS_ASSERT(!rv1.GetName().empty());
    TS_ASSERT(!rv2.GetName().empty());
    TS_ASSERT(!rv3.GetName().empty());
    TS_ASSERT(!rv4.GetName().empty());
  }

  void testPropertyValueNameFormat() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/name/format", true);
    FGPropertyValue pv(node);

    std::string name = pv.GetName();
    std::string fqn = pv.GetFullyQualifiedName();
    std::string printable = pv.GetPrintableName();
    std::string withSign = pv.GetNameWithSign();

    TS_ASSERT(!name.empty());
    TS_ASSERT(!fqn.empty());
    TS_ASSERT(!printable.empty());
    TS_ASSERT(!withSign.empty());
  }

  /***************************************************************************
   * Value Sign Tests
   ***************************************************************************/

  void testRealValueSignPositive() {
    FGRealValue rv(100.0);
    TS_ASSERT(rv.GetValue() > 0);
  }

  void testRealValueSignNegative() {
    FGRealValue rv(-100.0);
    TS_ASSERT(rv.GetValue() < 0);
  }

  void testRealValueSignZero() {
    FGRealValue rv(0.0);
    TS_ASSERT_EQUALS(rv.GetValue(), 0.0);
  }

  void testPropertyValueSignChange() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/sign/change", true);
    FGPropertyValue pv(node);

    node->setDoubleValue(50.0);
    TS_ASSERT(pv.GetValue() > 0);

    node->setDoubleValue(-50.0);
    TS_ASSERT(pv.GetValue() < 0);

    node->setDoubleValue(0.0);
    TS_ASSERT_DELTA(pv.GetValue(), 0.0, epsilon);
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  void testRealValueDoublePrecision() {
    // Test that double precision is maintained
    double precise = 1.23456789012345678;
    FGRealValue rv(precise);
    TS_ASSERT_DELTA(rv.GetValue(), precise, 1e-15);
  }

  void testPropertyValueDoublePrecision() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    double precise = 9.87654321098765432;
    auto node = pm->GetNode("test/precision", true);
    node->setDoubleValue(precise);

    FGPropertyValue pv(node);
    TS_ASSERT_DELTA(pv.GetValue(), precise, 1e-10);
  }

  void testRealValueVerySmallDifference() {
    FGRealValue rv1(1.0);
    FGRealValue rv2(1.0 + 1e-15);

    double diff = std::abs(rv1.GetValue() - rv2.GetValue());
    TS_ASSERT(diff < 1e-14);
    TS_ASSERT(diff > 0);
  }

  /***************************************************************************
   * Property Tree Hierarchy Tests
   ***************************************************************************/

  void testPropertyValueDeepHierarchy() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("a/b/c/d/e/f/deep", true);
    node->setDoubleValue(12345.0);

    FGPropertyValue pv(node);
    TS_ASSERT_DELTA(pv.GetValue(), 12345.0, epsilon);
  }

  void testPropertyValueSiblingNodes() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node1 = pm->GetNode("parent/child1", true);
    auto node2 = pm->GetNode("parent/child2", true);
    auto node3 = pm->GetNode("parent/child3", true);

    node1->setDoubleValue(1.0);
    node2->setDoubleValue(2.0);
    node3->setDoubleValue(3.0);

    FGPropertyValue pv1(node1), pv2(node2), pv3(node3);

    TS_ASSERT_DELTA(pv1.GetValue(), 1.0, epsilon);
    TS_ASSERT_DELTA(pv2.GetValue(), 2.0, epsilon);
    TS_ASSERT_DELTA(pv3.GetValue(), 3.0, epsilon);
  }

  void testPropertyValueIndexedNodes() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    // Create indexed nodes (array-like)
    std::vector<FGPropertyValue*> pvs;
    for (int i = 0; i < 5; i++) {
      std::string path = "array/element[" + std::to_string(i) + "]";
      auto node = pm->GetNode(path, true);
      node->setDoubleValue(i * 100.0);
      pvs.push_back(new FGPropertyValue(node));
    }

    for (int i = 0; i < 5; i++) {
      TS_ASSERT_DELTA(pvs[i]->GetValue(), i * 100.0, epsilon);
      delete pvs[i];
    }
  }

  /***************************************************************************
   * Boundary Value Tests
   ***************************************************************************/

  void testRealValueSlightlyPositive() {
    FGRealValue rv(1e-300);
    TS_ASSERT(rv.GetValue() > 0);
    TS_ASSERT_DELTA(rv.GetValue(), 1e-300, 1e-310);
  }

  void testRealValueSlightlyNegative() {
    FGRealValue rv(-1e-300);
    TS_ASSERT(rv.GetValue() < 0);
  }

  void testRealValueNearOverflow() {
    double nearMax = std::numeric_limits<double>::max() * 0.99;
    FGRealValue rv(nearMax);
    TS_ASSERT(std::isfinite(rv.GetValue()));
  }

  void testRealValueNearUnderflow() {
    double nearMin = std::numeric_limits<double>::min() * 1.01;
    FGRealValue rv(nearMin);
    TS_ASSERT(rv.GetValue() > 0);
    TS_ASSERT(std::isfinite(rv.GetValue()));
  }

  /***************************************************************************
   * Value Immutability Tests (for FGRealValue)
   ***************************************************************************/

  void testRealValueImmutability() {
    double original = 42.0;
    FGRealValue rv(original);

    // Read many times - should always be the same
    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(rv.GetValue(), original);
    }
  }

  void testRealValueIndependence() {
    FGRealValue rv1(10.0);
    FGRealValue rv2(20.0);

    // Verify they maintain their values independently
    TS_ASSERT_DELTA(rv1.GetValue(), 10.0, epsilon);
    TS_ASSERT_DELTA(rv2.GetValue(), 20.0, epsilon);

    // Read them interleaved
    for (int i = 0; i < 10; i++) {
      TS_ASSERT_DELTA(rv1.GetValue(), 10.0, epsilon);
      TS_ASSERT_DELTA(rv2.GetValue(), 20.0, epsilon);
    }
  }

  /***************************************************************************
   * Property Value Mutability Tests
   ***************************************************************************/

  void testPropertyValueMutabilityThroughNode() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/mutable", true);
    FGPropertyValue pv(node);

    double values[] = {0.0, 1.0, -1.0, 100.0, -100.0, 0.001, -0.001};

    for (double val : values) {
      node->setDoubleValue(val);
      TS_ASSERT_DELTA(pv.GetValue(), val, epsilon);
    }
  }

  void testPropertyValueMutabilityThroughSetValue() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/setvalue", true);
    FGPropertyValue pv(node);

    double values[] = {0.0, 1.0, -1.0, M_PI, M_E, 1e10, 1e-10};

    for (double val : values) {
      pv.SetValue(val);
      TS_ASSERT_DELTA(pv.GetValue(), val, std::abs(val) * 1e-10 + epsilon);
    }
  }

  /***************************************************************************
   * Special Numeric Values Tests
   ***************************************************************************/

  void testPropertyValueInfinity() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/infinity", true);
    node->setDoubleValue(std::numeric_limits<double>::infinity());

    FGPropertyValue pv(node);
    TS_ASSERT(std::isinf(pv.GetValue()));
  }

  void testPropertyValueNaN() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/nan", true);
    node->setDoubleValue(std::numeric_limits<double>::quiet_NaN());

    FGPropertyValue pv(node);
    TS_ASSERT(std::isnan(pv.GetValue()));
  }

  /***************************************************************************
   * String Representation Tests
   ***************************************************************************/

  void testRealValueGetNameForZero() {
    FGRealValue rv(0.0);
    std::string name = rv.GetName();
    TS_ASSERT(!name.empty());
  }

  void testRealValueGetNameForNegative() {
    FGRealValue rv(-42.5);
    std::string name = rv.GetName();
    TS_ASSERT(!name.empty());
    // Name should contain the minus or the value
  }

  void testRealValueGetNameForScientific() {
    FGRealValue rv(1.23e45);
    std::string name = rv.GetName();
    TS_ASSERT(!name.empty());
  }

  void testPropertyValuePathName() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("simulation/gravity/world-fps", true);
    FGPropertyValue pv(node);

    std::string fqn = pv.GetFullyQualifiedName();
    // Should contain path components
    TS_ASSERT(fqn.find("gravity") != std::string::npos ||
              fqn.find("world-fps") != std::string::npos);
  }

  /***************************************************************************
   * Copy and Assignment Tests
   ***************************************************************************/

  void testRealValueCopy() {
    FGRealValue rv1(42.0);
    FGRealValue rv2(rv1);

    TS_ASSERT_DELTA(rv1.GetValue(), rv2.GetValue(), epsilon);
    TS_ASSERT_EQUALS(rv1.IsConstant(), rv2.IsConstant());
  }

  void testPropertyValueCopy() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/copy", true);
    node->setDoubleValue(123.0);

    FGPropertyValue pv1(node);
    FGPropertyValue pv2(pv1);

    TS_ASSERT_DELTA(pv1.GetValue(), pv2.GetValue(), epsilon);
  }

  /***************************************************************************
   * Integration Tests
   ***************************************************************************/

  void testMixedValuesIntegration() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    // Create a mix of real and property values
    FGRealValue constant1(100.0);
    FGRealValue constant2(-50.0);

    auto node = pm->GetNode("test/variable", true);
    node->setDoubleValue(25.0);
    FGPropertyValue variable(node);

    // Use them in a calculation-like scenario
    double result = constant1.GetValue() + constant2.GetValue() + variable.GetValue();
    TS_ASSERT_DELTA(result, 75.0, epsilon);

    // Update variable
    node->setDoubleValue(50.0);
    result = constant1.GetValue() + constant2.GetValue() + variable.GetValue();
    TS_ASSERT_DELTA(result, 100.0, epsilon);

    // Constants unchanged
    TS_ASSERT_DELTA(constant1.GetValue(), 100.0, epsilon);
    TS_ASSERT_DELTA(constant2.GetValue(), -50.0, epsilon);
  }

  void testPropertyUpdatePropagation() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    auto node = pm->GetNode("test/propagate", true);
    node->setDoubleValue(0.0);

    // Create multiple property values pointing to same node
    FGPropertyValue pv1(node);
    FGPropertyValue pv2(node);
    FGPropertyValue pv3(node);

    // Update through node
    node->setDoubleValue(99.0);

    // All should see the update
    TS_ASSERT_DELTA(pv1.GetValue(), 99.0, epsilon);
    TS_ASSERT_DELTA(pv2.GetValue(), 99.0, epsilon);
    TS_ASSERT_DELTA(pv3.GetValue(), 99.0, epsilon);
  }

  /***************************************************************************
   * Performance/Stress Tests
   ***************************************************************************/

  void testRapidValueCreation() {
    for (int i = 0; i < 1000; i++) {
      FGRealValue rv(static_cast<double>(i));
      TS_ASSERT_DELTA(rv.GetValue(), static_cast<double>(i), epsilon);
    }
  }

  void testAlternatingReads() {
    FGFDMExec fdmex;
    auto pm = fdmex.GetPropertyManager();

    FGRealValue rv(42.0);
    auto node = pm->GetNode("test/alternate", true);
    node->setDoubleValue(84.0);
    FGPropertyValue pv(node);

    for (int i = 0; i < 100; i++) {
      TS_ASSERT_DELTA(rv.GetValue(), 42.0, epsilon);
      TS_ASSERT_DELTA(pv.GetValue(), 84.0, epsilon);
    }
  }
};
