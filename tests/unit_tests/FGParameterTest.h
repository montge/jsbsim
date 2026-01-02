#include <cxxtest/TestSuite.h>
#include <math/FGParameter.h>
#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;


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

  /***************************************************************************
   * NaN and Special Value Tests
   ***************************************************************************/

  // Test 39: NaN value handling
  void testNaNValue() {
    double nan_val = std::nan("");
    FGConstantDummy x(nan_val);
    TS_ASSERT(std::isnan(x.GetValue()));
  }

  // Test 40: Quiet NaN
  void testQuietNaN() {
    double qnan = std::numeric_limits<double>::quiet_NaN();
    FGConstantDummy x(qnan);
    TS_ASSERT(std::isnan(x.GetValue()));
  }

  // Test 41: Signaling NaN
  void testSignalingNaN() {
    double snan = std::numeric_limits<double>::signaling_NaN();
    FGConstantDummy x(snan);
    TS_ASSERT(std::isnan(x.GetValue()));
  }

  // Test 42: NaN multiplication
  void testNaNMultiplication() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(std::nan("")));
    double result = px * 5.0;
    TS_ASSERT(std::isnan(result));
  }

  // Test 43: Infinity multiplication
  void testInfinityMultiplication() {
    double inf = std::numeric_limits<double>::infinity();
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(inf));
    TS_ASSERT_EQUALS(px * 2.0, inf);
    TS_ASSERT_EQUALS(2.0 * px, inf);
  }

  // Test 44: Denormalized number
  void testDenormalizedNumber() {
    double denorm = std::numeric_limits<double>::denorm_min();
    FGConstantDummy x(denorm);
    TS_ASSERT_EQUALS(x.GetValue(), denorm);
    TS_ASSERT(x.GetValue() > 0.0);
  }

  // Test 45: Epsilon precision
  void testEpsilonPrecision() {
    double eps = std::numeric_limits<double>::epsilon();
    FGConstantDummy x(1.0 + eps);
    TS_ASSERT(x.GetValue() > 1.0);
  }

  /***************************************************************************
   * Double Precision Tests
   ***************************************************************************/

  // Test 46: Maximum double value
  void testMaxDoubleValue() {
    double max_val = std::numeric_limits<double>::max();
    FGConstantDummy x(max_val);
    TS_ASSERT_EQUALS(x.GetValue(), max_val);
  }

  // Test 47: Minimum positive double value
  void testMinPositiveDoubleValue() {
    double min_val = std::numeric_limits<double>::min();
    FGConstantDummy x(min_val);
    TS_ASSERT_EQUALS(x.GetValue(), min_val);
  }

  // Test 48: Lowest double value
  void testLowestDoubleValue() {
    double lowest = std::numeric_limits<double>::lowest();
    FGConstantDummy x(lowest);
    TS_ASSERT_EQUALS(x.GetValue(), lowest);
  }

  // Test 49: Near-zero positive
  void testNearZeroPositive() {
    FGConstantDummy x(1e-300);
    TS_ASSERT(x.GetValue() > 0.0);
    TS_ASSERT(x.GetValue() < 1e-100);
  }

  // Test 50: Near-zero negative
  void testNearZeroNegative() {
    FGConstantDummy x(-1e-300);
    TS_ASSERT(x.GetValue() < 0.0);
    TS_ASSERT(x.GetValue() > -1e-100);
  }

  /***************************************************************************
   * Mathematical Properties Tests
   ***************************************************************************/

  // Test 51: Pi value
  void testPiValue() {
    FGConstantDummy x(M_PI);
    TS_ASSERT_DELTA(x.GetValue(), 3.14159265358979, 1e-14);
  }

  // Test 52: E value
  void testEValue() {
    FGConstantDummy x(M_E);
    TS_ASSERT_DELTA(x.GetValue(), 2.71828182845904, 1e-14);
  }

  // Test 53: Negative zero
  void testNegativeZero() {
    FGConstantDummy x(-0.0);
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);  // -0.0 == 0.0 in IEEE 754
  }

  // Test 54: Associative multiplication
  void testAssociativeMultiplication() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(2.0));
    double a = 3.0, b = 4.0;
    // (px * a) * b should equal px * (a * b) for exact arithmetic
    double result1 = (px * a) * b;
    double result2 = px->GetValue() * (a * b);
    TS_ASSERT_EQUALS(result1, result2);
  }

  // Test 55: Distributive property
  void testDistributiveProperty() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(5.0));
    double a = 2.0, b = 3.0;
    // px * (a + b) should equal px * a + px * b
    double result1 = px->GetValue() * (a + b);
    double result2 = px * a + px * b;
    TS_ASSERT_EQUALS(result1, result2);
  }

  /***************************************************************************
   * Sign Handling Tests
   ***************************************************************************/

  // Test 56: Sign of positive
  void testSignPositive() {
    FGConstantDummy x(42.0);
    TS_ASSERT(x.GetValue() > 0);
  }

  // Test 57: Sign of negative
  void testSignNegative() {
    FGConstantDummy x(-42.0);
    TS_ASSERT(x.GetValue() < 0);
  }

  // Test 58: Negate via multiplication
  void testNegateViaMultiplication() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(100.0));
    TS_ASSERT_EQUALS(px * (-1.0), -100.0);
    TS_ASSERT_EQUALS((-1.0) * px, -100.0);
  }

  // Test 59: Double negate
  void testDoubleNegate() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(50.0));
    double result = px * (-1.0) * (-1.0);
    TS_ASSERT_EQUALS(result, 50.0);
  }

  /***************************************************************************
   * Name Pattern Tests
   ***************************************************************************/

  // Test 60: Name with dots
  void testNameWithDots() {
    FGNamedDummy x("fcs.aileron.position", 1.0);
    TS_ASSERT_EQUALS(x.GetName(), "fcs.aileron.position");
  }

  // Test 61: Name with brackets
  void testNameWithBrackets() {
    FGNamedDummy x("propulsion/engine[0]/thrust", 1000.0);
    TS_ASSERT_EQUALS(x.GetName(), "propulsion/engine[0]/thrust");
  }

  // Test 62: Name with numbers
  void testNameWithNumbers() {
    FGNamedDummy x("param123", 123.0);
    TS_ASSERT_EQUALS(x.GetName(), "param123");
  }

  // Test 63: Name with underscores
  void testNameWithUnderscores() {
    FGNamedDummy x("my_parameter_name", 42.0);
    TS_ASSERT_EQUALS(x.GetName(), "my_parameter_name");
  }

  // Test 64: Name with dashes
  void testNameWithDashes() {
    FGNamedDummy x("my-parameter-name", 42.0);
    TS_ASSERT_EQUALS(x.GetName(), "my-parameter-name");
  }

  // Test 65: Unicode in name (ASCII-safe representation)
  void testNameASCII() {
    FGNamedDummy x("alpha-deg", 10.0);
    TS_ASSERT_EQUALS(x.GetName(), "alpha-deg");
  }

  /***************************************************************************
   * Counting Parameter Tests
   ***************************************************************************/

  // Test 66: Counting after many calls
  void testCountingManyIterations() {
    FGDummy x;
    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(x.GetValue(), static_cast<double>(i));
    }
  }

  // Test 67: Counting with getDoubleValue
  void testCountingViaGetDoubleValue() {
    FGDummy x;
    // GetValue returns 0, then getDoubleValue returns 1
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT_EQUALS(x.getDoubleValue(), 1.0);
    TS_ASSERT_EQUALS(x.GetValue(), 2.0);
    TS_ASSERT_EQUALS(x.getDoubleValue(), 3.0);
  }

  // Test 68: Counting independence between instances
  void testCountingIndependence() {
    FGDummy a, b, c;
    TS_ASSERT_EQUALS(a.GetValue(), 0.0);
    TS_ASSERT_EQUALS(b.GetValue(), 0.0);
    TS_ASSERT_EQUALS(c.GetValue(), 0.0);

    TS_ASSERT_EQUALS(a.GetValue(), 1.0);
    TS_ASSERT_EQUALS(a.GetValue(), 2.0);

    TS_ASSERT_EQUALS(b.GetValue(), 1.0);
    TS_ASSERT_EQUALS(c.GetValue(), 1.0);
  }

  /***************************************************************************
   * Shared Pointer Advanced Tests
   ***************************************************************************/

  // Test 69: Shared pointer reassignment
  void testSharedPointerReassignment() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(10.0));
    TS_ASSERT_EQUALS(px->GetValue(), 10.0);

    px = SGSharedPtr<FGConstantDummy>(new FGConstantDummy(20.0));
    TS_ASSERT_EQUALS(px->GetValue(), 20.0);
  }

  // Test 70: Shared pointer comparison via operator->
  void testSharedPointerComparison() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(50.0));
    SGSharedPtr<FGConstantDummy> py = px;
    SGSharedPtr<FGConstantDummy> pz(new FGConstantDummy(50.0));

    // py points to same object as px, pz is different
    TS_ASSERT(px.valid() && py.valid() && pz.valid());
    TS_ASSERT_EQUALS(px->GetValue(), py->GetValue());
    TS_ASSERT_EQUALS(px->GetValue(), 50.0);
    TS_ASSERT_EQUALS(pz->GetValue(), 50.0);
  }

  // Test 71: Multiple shared pointer copies
  void testMultipleSharedPointerCopies() {
    SGSharedPtr<FGConstantDummy> p1(new FGConstantDummy(100.0));
    SGSharedPtr<FGConstantDummy> p2 = p1;
    SGSharedPtr<FGConstantDummy> p3 = p2;
    SGSharedPtr<FGConstantDummy> p4 = p1;

    TS_ASSERT(p1.valid());
    TS_ASSERT(p2.valid());
    TS_ASSERT(p3.valid());
    TS_ASSERT(p4.valid());

    TS_ASSERT_EQUALS(p1->GetValue(), 100.0);
    TS_ASSERT_EQUALS(p4->GetValue(), 100.0);
  }

  /***************************************************************************
   * Value Comparison Tests
   ***************************************************************************/

  // Test 72: Compare two parameter values
  void testCompareParameterValues() {
    FGConstantDummy a(10.0);
    FGConstantDummy b(20.0);

    TS_ASSERT(a.GetValue() < b.GetValue());
    TS_ASSERT(b.GetValue() > a.GetValue());
  }

  // Test 73: Equal parameter values
  void testEqualParameterValues() {
    FGConstantDummy a(42.5);
    FGConstantDummy b(42.5);

    TS_ASSERT_EQUALS(a.GetValue(), b.GetValue());
  }

  // Test 74: Near-equal values
  void testNearEqualValues() {
    FGConstantDummy a(1.0);
    FGConstantDummy b(1.0 + 1e-15);

    TS_ASSERT_DELTA(a.GetValue(), b.GetValue(), 1e-14);
  }

  // Test 75: Fractional values
  void testFractionalValues() {
    FGConstantDummy x(0.1 + 0.2);
    // Classic floating-point issue: 0.1 + 0.2 != 0.3 exactly
    TS_ASSERT_DELTA(x.GetValue(), 0.3, 1e-15);
  }
};

/***************************************************************************
 * Extended Parameter Tests (Tests 76-100)
 ***************************************************************************/

class FGParameterExtendedTest : public CxxTest::TestSuite
{
public:
  // Test 76: Parameter with zero value
  void testZeroValue() {
    FGConstantDummy x(0.0);

    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT(x.IsConstant());
  }

  // Test 77: Parameter with negative value
  void testNegativeValue() {
    FGConstantDummy x(-123.456);

    TS_ASSERT_EQUALS(x.GetValue(), -123.456);
    TS_ASSERT(x.GetValue() < 0);
  }

  // Test 78: Very large parameter value
  void testVeryLargeValue() {
    FGConstantDummy x(1.0e100);

    TS_ASSERT_EQUALS(x.GetValue(), 1.0e100);
    TS_ASSERT(x.GetValue() > 1.0e99);
  }

  // Test 79: Very small parameter value
  void testVerySmallValue() {
    FGConstantDummy x(1.0e-100);

    TS_ASSERT_EQUALS(x.GetValue(), 1.0e-100);
    TS_ASSERT(x.GetValue() < 1.0e-99);
    TS_ASSERT(x.GetValue() > 0);
  }

  // Test 80: Parameter name length
  void testParameterNameLength() {
    std::string longName(100, 'x');
    FGNamedDummy x(longName, 1.0);

    TS_ASSERT_EQUALS(x.GetName().length(), 100u);
    TS_ASSERT_EQUALS(x.GetName(), longName);
  }

  // Test 81: Empty parameter name
  void testEmptyParameterName() {
    FGNamedDummy x("", 42.0);

    TS_ASSERT(x.GetName().empty());
    TS_ASSERT_EQUALS(x.GetValue(), 42.0);
  }

  // Test 82: Parameter name with special characters
  void testParameterNameSpecialChars() {
    FGNamedDummy x("alpha/beta-gamma_delta", 1.0);

    TS_ASSERT_EQUALS(x.GetName(), "alpha/beta-gamma_delta");
  }

  // Test 83: Parameter name with spaces
  void testParameterNameWithSpaces() {
    FGNamedDummy x("parameter name with spaces", 2.0);

    TS_ASSERT_EQUALS(x.GetName(), "parameter name with spaces");
    TS_ASSERT_EQUALS(x.GetValue(), 2.0);
  }

  // Test 84: Multiply parameter by scalar
  void testMultiplyByScalar() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(5.0));

    double result = px * 3.0;

    TS_ASSERT_EQUALS(result, 15.0);
  }

  // Test 85: Multiply parameter by zero
  void testMultiplyByZero() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(100.0));

    double result = px * 0.0;

    TS_ASSERT_EQUALS(result, 0.0);
  }

  // Test 86: Multiply parameter by negative
  void testMultiplyByNegative() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(5.0));

    double result = px * (-2.0);

    TS_ASSERT_EQUALS(result, -10.0);
  }

  // Test 87: Pre-multiply parameter
  void testPreMultiply() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(4.0));

    double result = 2.5 * px;

    TS_ASSERT_EQUALS(result, 10.0);
  }

  // Test 88: Chained GetValue calls
  void testChainedGetValue() {
    FGDummy x;

    // Each call increments counter
    double v1 = x.GetValue();
    double v2 = x.GetValue();
    double v3 = x.GetValue();

    TS_ASSERT_EQUALS(v1, 0.0);
    TS_ASSERT_EQUALS(v2, 1.0);
    TS_ASSERT_EQUALS(v3, 2.0);
  }

  // Test 89: getDoubleValue alias
  void testGetDoubleValueAlias() {
    FGDummy x;

    // First call via GetValue
    x.GetValue();

    // getDoubleValue should also increment
    double v = x.getDoubleValue();

    TS_ASSERT_EQUALS(v, 1.0);
  }

  // Test 90: Shared pointer null check
  void testSharedPointerNullCheck() {
    SGSharedPtr<FGConstantDummy> px;

    TS_ASSERT(!px.valid());
  }

  // Test 91: Shared pointer valid check
  void testSharedPointerValidCheck() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(1.0));

    TS_ASSERT(px.valid());
  }

  // Test 92: Shared pointer reset
  void testSharedPointerReset() {
    SGSharedPtr<FGConstantDummy> px(new FGConstantDummy(10.0));
    TS_ASSERT(px.valid());

    px = SGSharedPtr<FGConstantDummy>();
    TS_ASSERT(!px.valid());
  }

  // Test 93: Parameter value sign check
  void testValueSignCheck() {
    FGConstantDummy positive(5.0);
    FGConstantDummy negative(-5.0);
    FGConstantDummy zero(0.0);

    TS_ASSERT(positive.GetValue() > 0);
    TS_ASSERT(negative.GetValue() < 0);
    TS_ASSERT(zero.GetValue() == 0);
  }

  // Test 94: Parameter infinity handling
  void testInfinityValue() {
    double inf = std::numeric_limits<double>::infinity();
    FGConstantDummy x(inf);

    TS_ASSERT(std::isinf(x.GetValue()));
    TS_ASSERT(x.GetValue() > 0);
  }

  // Test 95: Parameter negative infinity
  void testNegativeInfinityValue() {
    double negInf = -std::numeric_limits<double>::infinity();
    FGConstantDummy x(negInf);

    TS_ASSERT(std::isinf(x.GetValue()));
    TS_ASSERT(x.GetValue() < 0);
  }

  // Test 96: Parameter NaN handling
  void testNaNValue() {
    double nan = std::numeric_limits<double>::quiet_NaN();
    FGConstantDummy x(nan);

    TS_ASSERT(std::isnan(x.GetValue()));
  }

  // Test 97: Denormalized value
  void testDenormalizedValue() {
    double denorm = std::numeric_limits<double>::denorm_min();
    FGConstantDummy x(denorm);

    TS_ASSERT(x.GetValue() > 0);
    TS_ASSERT(x.GetValue() < std::numeric_limits<double>::min());
  }

  // Test 98: Max double value
  void testMaxDoubleValue() {
    double maxVal = std::numeric_limits<double>::max();
    FGConstantDummy x(maxVal);

    TS_ASSERT_EQUALS(x.GetValue(), maxVal);
  }

  // Test 99: Min double value
  void testMinDoubleValue() {
    double minVal = std::numeric_limits<double>::lowest();
    FGConstantDummy x(minVal);

    TS_ASSERT_EQUALS(x.GetValue(), minVal);
  }

  // Test 100: Epsilon precision
  void testEpsilonPrecision() {
    double epsilon = std::numeric_limits<double>::epsilon();
    FGConstantDummy x(1.0);
    FGConstantDummy y(1.0 + epsilon);

    // They should be different
    TS_ASSERT(x.GetValue() != y.GetValue());
    // But very close
    TS_ASSERT_DELTA(x.GetValue(), y.GetValue(), 2 * epsilon);
  }
};

// ============================================================================
// C172x Integration Tests for FGParameter
// ============================================================================

class FGParameterC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {
    std::string rootDir = JSBSIM_TEST_ROOT_DIR;
    fdm.SetRootDir(SGPath(rootDir));
    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: C172x model properties are accessible as parameters
  void testC172xPropertiesAsParameters() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double altitude = pm->GetDouble("position/h-sl-ft");

    TS_ASSERT(!std::isnan(altitude));
  }

  // Test 2: C172x parameter value retrieval consistency
  void testC172xParameterValueConsistency() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double v1 = pm->GetDouble("velocities/vc-kts");
    double v2 = pm->GetDouble("velocities/vc-kts");

    TS_ASSERT_EQUALS(v1, v2);
  }

  // Test 3: C172x parameter setting and getting
  void testC172xParameterSetGet() {
    fdm.RunIC();

    auto pm = fdm.GetPropertyManager();
    pm->SetDouble("fcs/throttle-cmd-norm", 0.75);

    double val = pm->GetDouble("fcs/throttle-cmd-norm");
    TS_ASSERT_DELTA(val, 0.75, 0.001);
  }

  // Test 4: C172x multiple parameter access
  void testC172xMultipleParameterAccess() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();

    double lat = pm->GetDouble("position/lat-gc-deg");
    double lon = pm->GetDouble("position/long-gc-deg");
    double alt = pm->GetDouble("position/h-sl-ft");
    double phi = pm->GetDouble("attitude/phi-rad");
    double theta = pm->GetDouble("attitude/theta-rad");
    double psi = pm->GetDouble("attitude/psi-rad");

    TS_ASSERT(!std::isnan(lat));
    TS_ASSERT(!std::isnan(lon));
    TS_ASSERT(!std::isnan(alt));
    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isnan(theta));
    TS_ASSERT(!std::isnan(psi));
  }

  // Test 5: C172x parameter zero value
  void testC172xParameterZeroValue() {
    fdm.RunIC();

    auto pm = fdm.GetPropertyManager();
    pm->SetDouble("fcs/throttle-cmd-norm", 0.0);

    double val = pm->GetDouble("fcs/throttle-cmd-norm");
    TS_ASSERT_DELTA(val, 0.0, 1e-10);
  }

  // Test 6: C172x parameter negative values
  void testC172xParameterNegativeValues() {
    fdm.RunIC();

    auto pm = fdm.GetPropertyManager();
    pm->SetDouble("fcs/elevator-cmd-norm", -0.5);

    double val = pm->GetDouble("fcs/elevator-cmd-norm");
    TS_ASSERT_DELTA(val, -0.5, 0.001);
  }

  // Test 7: C172x parameter value bounds
  void testC172xParameterValueBounds() {
    fdm.RunIC();

    auto pm = fdm.GetPropertyManager();

    // Set throttle to max and min
    pm->GetNode("fcs/throttle-cmd-norm")->setDoubleValue(1.0);
    TS_ASSERT_DELTA(pm->GetNode("fcs/throttle-cmd-norm")->getDoubleValue(), 1.0, 0.001);

    pm->GetNode("fcs/throttle-cmd-norm")->setDoubleValue(0.0);
    TS_ASSERT_DELTA(pm->GetNode("fcs/throttle-cmd-norm")->getDoubleValue(), 0.0, 0.001);
  }

  // Test 8: C172x parameter after simulation steps
  void testC172xParameterAfterSimSteps() {
    fdm.RunIC();
    fdm.GetFCS()->SetThrottleCmd(0, 0.8);

    for (int i = 0; i < 50; i++) {
      fdm.Run();
    }

    auto pm = fdm.GetPropertyManager();
    double simTime = pm->GetNode("simulation/sim-time-sec")->getDoubleValue();

    TS_ASSERT(simTime > 0.0);
  }

  // Test 9: C172x computed parameter values
  void testC172xComputedParameterValues() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();

    // These are computed parameters, not directly set
    double qbar = pm->GetNode("aero/qbar-psf")->getDoubleValue();
    double mach = pm->GetNode("velocities/mach")->getDoubleValue();

    TS_ASSERT(!std::isnan(qbar));
    TS_ASSERT(!std::isnan(mach));
    TS_ASSERT(qbar >= 0.0);
    TS_ASSERT(mach >= 0.0);
  }

  // Test 10: C172x parameter precision
  void testC172xParameterPrecision() {
    fdm.RunIC();

    auto pm = fdm.GetPropertyManager();

    // Set precise value
    double preciseValue = 0.123456789;
    pm->GetNode("fcs/throttle-cmd-norm")->setDoubleValue(preciseValue);

    double retrieved = pm->GetNode("fcs/throttle-cmd-norm")->getDoubleValue();
    TS_ASSERT_DELTA(retrieved, preciseValue, 1e-9);
  }

  // Test 11: C172x parameter rapid updates
  void testC172xParameterRapidUpdates() {
    fdm.RunIC();

    auto pm = fdm.GetPropertyManager();

    for (int i = 0; i < 100; i++) {
      double val = static_cast<double>(i) / 100.0;
      pm->GetNode("fcs/throttle-cmd-norm")->setDoubleValue(val);
      fdm.Run();

      double retrieved = pm->GetNode("fcs/throttle-cmd-norm")->getDoubleValue();
      TS_ASSERT_DELTA(retrieved, val, 0.001);
    }
  }

  // Test 12: C172x parameter stability
  void testC172xParameterStability() {
    fdm.RunIC();
    fdm.GetFCS()->SetThrottleCmd(0, 0.5);
    fdm.GetFCS()->SetMixtureCmd(0, 0.9);

    auto pm = fdm.GetPropertyManager();

    // Run and check parameter stability
    for (int i = 0; i < 100; i++) {
      fdm.Run();

      double altitude = pm->GetNode("position/h-sl-ft")->getDoubleValue();
      double velocity = pm->GetNode("velocities/vc-kts")->getDoubleValue();

      TS_ASSERT(!std::isnan(altitude));
      TS_ASSERT(!std::isnan(velocity));
      TS_ASSERT(!std::isinf(altitude));
      TS_ASSERT(!std::isinf(velocity));
    }
  }
};
