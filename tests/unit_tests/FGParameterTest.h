#include <cxxtest/TestSuite.h>
#include <math/FGParameter.h>

using namespace JSBSim;


// Dummy class that inherits the abstract class `FGParameter`.
class FGDummy : public FGParameter
{
public:
  FGDummy(void) : count(0) {}
  double GetValue(void) const { return count++; }
  std::string GetName(void) const { return "Counting..."; }
private:
  mutable unsigned int count;
};

// Constant parameter class for testing IsConstant override
class FGConstantDummy : public FGParameter
{
public:
  FGConstantDummy(double val) : value(val) {}
  double GetValue(void) const { return value; }
  std::string GetName(void) const { return "Constant"; }
  bool IsConstant(void) const { return true; }
private:
  double value;
};

// Named parameter class for testing GetName variations
class FGNamedDummy : public FGParameter
{
public:
  FGNamedDummy(const std::string& name, double val)
    : paramName(name), value(val) {}
  double GetValue(void) const { return value; }
  std::string GetName(void) const { return paramName; }
private:
  std::string paramName;
  double value;
};

class FGParameterTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Basic Construction Tests
   ***************************************************************************/

  void testConstructor() {
    FGDummy x;

    TS_ASSERT(!x.IsConstant());
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT_EQUALS(x.getDoubleValue(), 1.0);
    TS_ASSERT_EQUALS(x.GetName(), "Counting...");
  }

  void testCopyConstructor() {
    FGDummy x;
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);

    FGDummy y(x);
    TS_ASSERT_EQUALS(x.GetValue(), 1.0);
    TS_ASSERT_EQUALS(x.getDoubleValue(), 2.0);
    TS_ASSERT(!x.IsConstant());

    TS_ASSERT(!y.IsConstant());
    TS_ASSERT_EQUALS(y.GetValue(), 1.0);
    TS_ASSERT_EQUALS(y.GetName(), "Counting...");
    TS_ASSERT_EQUALS(x.GetValue(), 3.0);
  }

  void testOperators() {
    SGSharedPtr<FGDummy> px(new FGDummy);

    TS_ASSERT_EQUALS(px*2.0, 0.0);
    TS_ASSERT_EQUALS(-3.0*px, -3.0);
    TS_ASSERT_EQUALS(px*2.0, 4.0);
  }

  /***************************************************************************
   * IsConstant Tests
   ***************************************************************************/

  void testIsConstantDefault() {
    // Default implementation returns false
    FGDummy x;
    TS_ASSERT_EQUALS(x.IsConstant(), false);
  }

  void testIsConstantOverride() {
    // Override returns true for constant parameters
    FGConstantDummy x(42.0);
    TS_ASSERT_EQUALS(x.IsConstant(), true);
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  void testConstantValueDoesNotChange() {
    FGConstantDummy x(100.0);
    double v1 = x.GetValue();
    double v2 = x.GetValue();
    double v3 = x.getDoubleValue();

    TS_ASSERT_EQUALS(v1, 100.0);
    TS_ASSERT_EQUALS(v2, 100.0);
    TS_ASSERT_EQUALS(v3, 100.0);
  }

  /***************************************************************************
   * GetValue Tests
   ***************************************************************************/

  void testGetValueIncrementing() {
    // The FGDummy class increments count on each call
    FGDummy x;
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 1.0);
    TS_ASSERT_EQUALS(x.GetValue(), 2.0);
    TS_ASSERT_EQUALS(x.GetValue(), 3.0);
  }

  void testGetDoubleValueAlias() {
    // getDoubleValue is an alias for GetValue
    FGConstantDummy x(3.14159);
    TS_ASSERT_EQUALS(x.GetValue(), x.getDoubleValue());
  }

  void testGetValueZero() {
    FGConstantDummy x(0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT_EQUALS(x.getDoubleValue(), 0.0);
  }

  void testGetValueNegative() {
    FGConstantDummy x(-50.5);
    TS_ASSERT_EQUALS(x.GetValue(), -50.5);
  }

  void testGetValueLargeNumber() {
    FGConstantDummy x(1e12);
    TS_ASSERT_EQUALS(x.GetValue(), 1e12);
  }

  void testGetValueSmallNumber() {
    FGConstantDummy x(1e-12);
    TS_ASSERT_EQUALS(x.GetValue(), 1e-12);
  }

  /***************************************************************************
   * GetName Tests
   ***************************************************************************/

  void testGetNameDefault() {
    FGDummy x;
    TS_ASSERT_EQUALS(x.GetName(), "Counting...");
  }

  void testGetNameConstant() {
    FGConstantDummy x(1.0);
    TS_ASSERT_EQUALS(x.GetName(), "Constant");
  }

  void testGetNameCustom() {
    FGNamedDummy x("my-parameter", 42.0);
    TS_ASSERT_EQUALS(x.GetName(), "my-parameter");
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  void testGetNameEmpty() {
    FGNamedDummy x("", 0.0);
    TS_ASSERT_EQUALS(x.GetName(), "");
  }

  void testGetNameLong() {
    std::string longName = "this-is-a-very-long-parameter-name-for-testing";
    FGNamedDummy x(longName, 1.0);
    TS_ASSERT_EQUALS(x.GetName(), longName);
  }

  void testGetNameSpecialChars() {
    FGNamedDummy x("param/sub/name", 1.0);
    TS_ASSERT_EQUALS(x.GetName(), "param/sub/name");
  }

  /***************************************************************************
   * Operator Multiplication Tests
   ***************************************************************************/

  void testOperatorMultiplyByPositive() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(5.0));
    TS_ASSERT_EQUALS(px * 3.0, 15.0);
    TS_ASSERT_EQUALS(3.0 * px, 15.0);
  }

  void testOperatorMultiplyByNegative() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(5.0));
    TS_ASSERT_EQUALS(px * (-2.0), -10.0);
    TS_ASSERT_EQUALS(-2.0 * px, -10.0);
  }

  void testOperatorMultiplyByZero() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(5.0));
    TS_ASSERT_EQUALS(px * 0.0, 0.0);
    TS_ASSERT_EQUALS(0.0 * px, 0.0);
  }

  void testOperatorMultiplyByOne() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(7.5));
    TS_ASSERT_EQUALS(px * 1.0, 7.5);
    TS_ASSERT_EQUALS(1.0 * px, 7.5);
  }

  void testOperatorMultiplyByFraction() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(10.0));
    TS_ASSERT_EQUALS(px * 0.5, 5.0);
    TS_ASSERT_EQUALS(0.5 * px, 5.0);
  }

  void testOperatorMultiplyCommutativity() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(4.0));
    double v = 3.5;
    TS_ASSERT_EQUALS(px * v, v * px);
  }

  /***************************************************************************
   * Shared Pointer Tests
   ***************************************************************************/

  void testSharedPointerConstruction() {
    SGSharedPtr<FGDummy> px(new FGDummy);
    TS_ASSERT(px.valid());
    TS_ASSERT_EQUALS(px->GetValue(), 0.0);
  }

  void testSharedPointerCopy() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(100.0));
    SGSharedPtr<FGConstantDummy> py = px;

    TS_ASSERT(px.valid());
    TS_ASSERT(py.valid());
    TS_ASSERT_EQUALS(px->GetValue(), 100.0);
    TS_ASSERT_EQUALS(py->GetValue(), 100.0);
  }

  void testSharedPointerNull() {
    SGSharedPtr<FGDummy> px;
    TS_ASSERT(!px.valid());
  }

  void testSharedPointerReset() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(50.0));
    TS_ASSERT(px.valid());

    px = nullptr;
    TS_ASSERT(!px.valid());
  }

  /***************************************************************************
   * Multiple Instance Tests
   ***************************************************************************/

  void testMultipleInstances() {
    FGDummy a;
    FGDummy b;

    // Each instance has its own count
    TS_ASSERT_EQUALS(a.GetValue(), 0.0);
    TS_ASSERT_EQUALS(b.GetValue(), 0.0);
    TS_ASSERT_EQUALS(a.GetValue(), 1.0);
    TS_ASSERT_EQUALS(b.GetValue(), 1.0);
  }

  void testMultipleNamedInstances() {
    FGNamedDummy a("param-a", 1.0);
    FGNamedDummy b("param-b", 2.0);
    FGNamedDummy c("param-c", 3.0);

    TS_ASSERT_EQUALS(a.GetName(), "param-a");
    TS_ASSERT_EQUALS(b.GetName(), "param-b");
    TS_ASSERT_EQUALS(c.GetName(), "param-c");

    TS_ASSERT_EQUALS(a.GetValue(), 1.0);
    TS_ASSERT_EQUALS(b.GetValue(), 2.0);
    TS_ASSERT_EQUALS(c.GetValue(), 3.0);
  }

  void testMixedParameterTypes() {
    FGDummy counting;
    FGConstantDummy constant(50.0);
    FGNamedDummy named("test", 25.0);

    TS_ASSERT_EQUALS(counting.IsConstant(), false);
    TS_ASSERT_EQUALS(constant.IsConstant(), true);
    TS_ASSERT_EQUALS(named.IsConstant(), false);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testVeryLargeValues() {
    FGConstantDummy x(1e308);
    TS_ASSERT_EQUALS(x.GetValue(), 1e308);

    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(1e100));
    TS_ASSERT_EQUALS(px * 1.0, 1e100);
  }

  void testVerySmallValues() {
    FGConstantDummy x(1e-308);
    TS_ASSERT_EQUALS(x.GetValue(), 1e-308);
  }

  void testInfinityValue() {
    double inf = std::numeric_limits<double>::infinity();
    FGConstantDummy x(inf);
    TS_ASSERT_EQUALS(x.GetValue(), inf);
  }

  void testNegativeInfinityValue() {
    double neg_inf = -std::numeric_limits<double>::infinity();
    FGConstantDummy x(neg_inf);
    TS_ASSERT_EQUALS(x.GetValue(), neg_inf);
  }

  /***************************************************************************
   * Parameter Pointer Typedef Tests
   ***************************************************************************/

  void testParameterPtrTypedef() {
    FGParameter_ptr ptr(new FGConstantDummy(42.0));
    TS_ASSERT(ptr.valid());
    TS_ASSERT_EQUALS(ptr->GetValue(), 42.0);
    TS_ASSERT_EQUALS(ptr->GetName(), "Constant");
  }

  void testParameterPtrOperators() {
    FGParameter_ptr ptr(new FGConstantDummy(10.0));
    TS_ASSERT_EQUALS(ptr * 5.0, 50.0);
    TS_ASSERT_EQUALS(5.0 * ptr, 50.0);
  }

  void testParameterPtrPolymorphism() {
    // Can hold different derived types
    FGParameter_ptr ptr1(new FGDummy);
    FGParameter_ptr ptr2(new FGConstantDummy(100.0));
    FGParameter_ptr ptr3(new FGNamedDummy("test", 50.0));

    TS_ASSERT_EQUALS(ptr1->IsConstant(), false);
    TS_ASSERT_EQUALS(ptr2->IsConstant(), true);
    TS_ASSERT_EQUALS(ptr3->IsConstant(), false);
  }
};
