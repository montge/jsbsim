#include <cxxtest/TestSuite.h>
#include <math/FGRealValue.h>
#include <limits>
#include <cmath>

using namespace JSBSim;


class FGRealValueTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Constructor Tests - Basic Values
   ***************************************************************************/

  void testConstructorPositive() {
    // GIVEN: A positive real value
    FGRealValue x(1.0);

    // THEN: GetValue returns the value
    TS_ASSERT_EQUALS(x.GetValue(), 1.0);
    TS_ASSERT_EQUALS(x.GetName(), "constant value 1.000000");
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorNegative() {
    // GIVEN: A negative real value
    FGRealValue x(-5.5);

    // THEN: GetValue returns the negative value correctly
    TS_ASSERT_EQUALS(x.GetValue(), -5.5);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorZero() {
    // GIVEN: Zero value
    FGRealValue x(0.0);

    // THEN: GetValue returns zero
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT_EQUALS(x.GetName(), "constant value 0.000000");
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorVerySmall() {
    // GIVEN: A very small value
    FGRealValue x(1e-15);

    // THEN: GetValue returns the precise value
    TS_ASSERT_EQUALS(x.GetValue(), 1e-15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorVeryLarge() {
    // GIVEN: A very large value
    FGRealValue x(1e15);

    // THEN: GetValue returns the large value
    TS_ASSERT_EQUALS(x.GetValue(), 1e15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorPi() {
    // GIVEN: Pi value
    FGRealValue x(M_PI);

    // THEN: GetValue returns pi with precision
    TS_ASSERT_DELTA(x.GetValue(), M_PI, 1e-15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorE() {
    // GIVEN: Euler's number
    FGRealValue x(M_E);

    // THEN: GetValue returns e with precision
    TS_ASSERT_DELTA(x.GetValue(), M_E, 1e-15);
    TS_ASSERT(x.IsConstant());
  }

  void testConstructorNegativeZero() {
    // GIVEN: Negative zero
    FGRealValue x(-0.0);

    // THEN: GetValue returns zero (negative zero equals zero)
    TS_ASSERT_EQUALS(x.GetValue(), 0.0);
    TS_ASSERT(x.IsConstant());
  }

  /***************************************************************************
   * IsConstant Tests
   ***************************************************************************/

  void testIsConstantAlwaysTrue() {
    // GIVEN: Various FGRealValue instances
    FGRealValue x1(0.0);
    FGRealValue x2(100.0);
    FGRealValue x3(-999.9);

    // THEN: IsConstant is always true (FGRealValue is immutable)
    TS_ASSERT(x1.IsConstant());
    TS_ASSERT(x2.IsConstant());
    TS_ASSERT(x3.IsConstant());
  }

  void testIsConstantForSpecialValues() {
    FGRealValue inf_val(std::numeric_limits<double>::infinity());
    FGRealValue neg_inf(-std::numeric_limits<double>::infinity());
    FGRealValue max_val(std::numeric_limits<double>::max());

    TS_ASSERT(inf_val.IsConstant());
    TS_ASSERT(neg_inf.IsConstant());
    TS_ASSERT(max_val.IsConstant());
  }

  /***************************************************************************
   * GetValue Idempotency Tests
   ***************************************************************************/

  void testGetValueIsIdempotent() {
    // GIVEN: A FGRealValue
    FGRealValue x(42.0);

    // WHEN: Calling GetValue multiple times
    double v1 = x.GetValue();
    double v2 = x.GetValue();
    double v3 = x.GetValue();

    // THEN: All calls return the same value
    TS_ASSERT_EQUALS(v1, 42.0);
    TS_ASSERT_EQUALS(v2, 42.0);
    TS_ASSERT_EQUALS(v3, 42.0);
  }

  void testGetValueIdempotentManyTimes() {
    FGRealValue x(3.14159);

    for (int i = 0; i < 1000; i++) {
      TS_ASSERT_EQUALS(x.GetValue(), 3.14159);
    }
  }

  void testGetDoubleValueAlias() {
    // getDoubleValue should return the same as GetValue
    FGRealValue x(123.456);

    TS_ASSERT_EQUALS(x.GetValue(), x.getDoubleValue());
  }

  /***************************************************************************
   * GetName Format Tests
   ***************************************************************************/

  void testGetNameFormat() {
    // GIVEN: Values with different decimal representations
    FGRealValue x1(1.5);
    FGRealValue x2(100.0);
    FGRealValue x3(0.123456);

    // THEN: GetName should contain "constant value" and the formatted number
    TS_ASSERT(x1.GetName().find("constant value") != std::string::npos);
    TS_ASSERT(x2.GetName().find("constant value") != std::string::npos);
    TS_ASSERT(x3.GetName().find("constant value") != std::string::npos);
  }

  void testGetNameZero() {
    FGRealValue x(0.0);
    std::string name = x.GetName();

    TS_ASSERT(name.find("constant value") != std::string::npos);
    TS_ASSERT(name.find("0.000000") != std::string::npos);
  }

  void testGetNameOne() {
    FGRealValue x(1.0);
    std::string name = x.GetName();

    TS_ASSERT(name.find("constant value") != std::string::npos);
    TS_ASSERT(name.find("1.000000") != std::string::npos);
  }

  void testGetNameNegative() {
    FGRealValue x(-42.5);
    std::string name = x.GetName();

    TS_ASSERT(name.find("constant value") != std::string::npos);
    TS_ASSERT(name.find("-42.5") != std::string::npos);
  }

  void testGetNameNotEmpty() {
    FGRealValue x(999.999);
    TS_ASSERT(!x.GetName().empty());
  }

  /***************************************************************************
   * Special IEEE Values Tests
   ***************************************************************************/

  void testSpecialValues() {
    // GIVEN: Special IEEE floating point values
    FGRealValue maxVal(std::numeric_limits<double>::max());
    FGRealValue minVal(std::numeric_limits<double>::min());
    FGRealValue denorm(std::numeric_limits<double>::denorm_min());

    // THEN: Values are preserved
    TS_ASSERT_EQUALS(maxVal.GetValue(), std::numeric_limits<double>::max());
    TS_ASSERT_EQUALS(minVal.GetValue(), std::numeric_limits<double>::min());
    TS_ASSERT_EQUALS(denorm.GetValue(), std::numeric_limits<double>::denorm_min());
  }

  void testInfinityValue() {
    double inf = std::numeric_limits<double>::infinity();
    FGRealValue x(inf);

    TS_ASSERT_EQUALS(x.GetValue(), inf);
    TS_ASSERT(std::isinf(x.GetValue()));
    TS_ASSERT(x.IsConstant());
  }

  void testNegativeInfinityValue() {
    double neg_inf = -std::numeric_limits<double>::infinity();
    FGRealValue x(neg_inf);

    TS_ASSERT_EQUALS(x.GetValue(), neg_inf);
    TS_ASSERT(std::isinf(x.GetValue()));
    TS_ASSERT(x.GetValue() < 0);
    TS_ASSERT(x.IsConstant());
  }

  void testNaNValue() {
    double nan_val = std::nan("");
    FGRealValue x(nan_val);

    TS_ASSERT(std::isnan(x.GetValue()));
    TS_ASSERT(x.IsConstant());
  }

  void testQuietNaN() {
    double qnan = std::numeric_limits<double>::quiet_NaN();
    FGRealValue x(qnan);

    TS_ASSERT(std::isnan(x.GetValue()));
  }

  void testEpsilon() {
    double eps = std::numeric_limits<double>::epsilon();
    FGRealValue x(eps);

    TS_ASSERT_EQUALS(x.GetValue(), eps);
  }

  void testLowestValue() {
    double lowest = std::numeric_limits<double>::lowest();
    FGRealValue x(lowest);

    TS_ASSERT_EQUALS(x.GetValue(), lowest);
  }

  /***************************************************************************
   * Precision Tests
   ***************************************************************************/

  void testPrecisionSmallDifference() {
    double a = 1.0;
    double b = 1.0 + std::numeric_limits<double>::epsilon();

    FGRealValue xa(a);
    FGRealValue xb(b);

    TS_ASSERT_DIFFERS(xa.GetValue(), xb.GetValue());
  }

  void testPrecisionMantissa() {
    // Test precision of mantissa
    double val = 1.23456789012345678901234567890;
    FGRealValue x(val);

    // Should preserve at least 15 significant digits
    TS_ASSERT_DELTA(x.GetValue(), val, 1e-15);
  }

  void testPrecisionFractions() {
    FGRealValue half(0.5);
    FGRealValue third(1.0/3.0);
    FGRealValue seventh(1.0/7.0);

    TS_ASSERT_EQUALS(half.GetValue(), 0.5);
    TS_ASSERT_DELTA(third.GetValue(), 1.0/3.0, 1e-15);
    TS_ASSERT_DELTA(seventh.GetValue(), 1.0/7.0, 1e-15);
  }

  /***************************************************************************
   * Multiple Instances Tests
   ***************************************************************************/

  void testMultipleInstancesIndependent() {
    FGRealValue x1(10.0);
    FGRealValue x2(20.0);
    FGRealValue x3(30.0);

    TS_ASSERT_EQUALS(x1.GetValue(), 10.0);
    TS_ASSERT_EQUALS(x2.GetValue(), 20.0);
    TS_ASSERT_EQUALS(x3.GetValue(), 30.0);
  }

  void testMultipleInstancesSameValue() {
    FGRealValue x1(42.0);
    FGRealValue x2(42.0);

    TS_ASSERT_EQUALS(x1.GetValue(), x2.GetValue());
    TS_ASSERT_EQUALS(x1.GetName(), x2.GetName());
  }

  void testArrayOfRealValues() {
    std::vector<FGRealValue> values;
    for (int i = 0; i < 100; i++) {
      values.emplace_back(static_cast<double>(i));
    }

    for (int i = 0; i < 100; i++) {
      TS_ASSERT_EQUALS(values[i].GetValue(), static_cast<double>(i));
    }
  }

  /***************************************************************************
   * Mathematical Constants Tests
   ***************************************************************************/

  void testMathConstantPi() {
    FGRealValue pi(M_PI);
    TS_ASSERT_DELTA(pi.GetValue(), 3.14159265358979323846, 1e-15);
  }

  void testMathConstantE() {
    FGRealValue e(M_E);
    TS_ASSERT_DELTA(e.GetValue(), 2.71828182845904523536, 1e-15);
  }

  void testMathConstantSqrt2() {
    FGRealValue sqrt2(M_SQRT2);
    TS_ASSERT_DELTA(sqrt2.GetValue(), 1.41421356237309504880, 1e-15);
  }

  void testMathConstantLn2() {
    FGRealValue ln2(M_LN2);
    TS_ASSERT_DELTA(ln2.GetValue(), 0.69314718055994530942, 1e-15);
  }

  /***************************************************************************
   * Derived Operations Tests (through FGParameter interface)
   ***************************************************************************/

  void testMultiplicationOperator() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(5.0));

    TS_ASSERT_EQUALS(p * 2.0, 10.0);
    TS_ASSERT_EQUALS(2.0 * p, 10.0);
  }

  void testMultiplicationByZero() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(100.0));

    TS_ASSERT_EQUALS(p * 0.0, 0.0);
    TS_ASSERT_EQUALS(0.0 * p, 0.0);
  }

  void testMultiplicationByNegative() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(7.0));

    TS_ASSERT_EQUALS(p * (-3.0), -21.0);
    TS_ASSERT_EQUALS(-3.0 * p, -21.0);
  }

  void testMultiplicationByFraction() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(10.0));

    TS_ASSERT_EQUALS(p * 0.5, 5.0);
    TS_ASSERT_EQUALS(0.5 * p, 5.0);
  }

  void testMultiplicationCommutativity() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(4.5));
    double v = 3.0;

    TS_ASSERT_EQUALS(p * v, v * p);
  }

  /***************************************************************************
   * Pointer/Reference Tests
   ***************************************************************************/

  void testSharedPointerConstruction() {
    SGSharedPtr<FGRealValue> p(new FGRealValue(42.0));

    TS_ASSERT(p.valid());
    TS_ASSERT_EQUALS(p->GetValue(), 42.0);
    TS_ASSERT(p->IsConstant());
  }

  void testSharedPointerCopy() {
    SGSharedPtr<FGRealValue> p1(new FGRealValue(100.0));
    SGSharedPtr<FGRealValue> p2 = p1;

    TS_ASSERT(p1.valid());
    TS_ASSERT(p2.valid());
    TS_ASSERT_EQUALS(p1->GetValue(), p2->GetValue());
  }

  void testFGParameterPtrTypedef() {
    FGParameter_ptr p(new FGRealValue(55.5));

    TS_ASSERT(p.valid());
    TS_ASSERT_EQUALS(p->GetValue(), 55.5);
    TS_ASSERT(p->IsConstant());
  }

  /***************************************************************************
   * Immutability Tests
   ***************************************************************************/

  void testImmutabilityAfterCreation() {
    FGRealValue x(123.456);

    double v1 = x.GetValue();
    double v2 = x.GetValue();
    std::string n1 = x.GetName();
    std::string n2 = x.GetName();

    TS_ASSERT_EQUALS(v1, v2);
    TS_ASSERT_EQUALS(n1, n2);
    TS_ASSERT_EQUALS(v1, 123.456);
  }

  void testConstCorrectness() {
    const FGRealValue x(99.9);

    // These should compile - const methods
    double v = x.GetValue();
    std::string n = x.GetName();
    bool c = x.IsConstant();

    TS_ASSERT_EQUALS(v, 99.9);
    TS_ASSERT(!n.empty());
    TS_ASSERT(c);
  }

  /***************************************************************************
   * Scientific Notation Tests
   ***************************************************************************/

  void testScientificNotationPositive() {
    FGRealValue x1(1.5e10);
    FGRealValue x2(3.14159e-5);
    FGRealValue x3(6.022e23);

    TS_ASSERT_EQUALS(x1.GetValue(), 1.5e10);
    TS_ASSERT_EQUALS(x2.GetValue(), 3.14159e-5);
    TS_ASSERT_EQUALS(x3.GetValue(), 6.022e23);
  }

  void testScientificNotationNegative() {
    FGRealValue x1(-1.5e10);
    FGRealValue x2(-3.14159e-5);

    TS_ASSERT_EQUALS(x1.GetValue(), -1.5e10);
    TS_ASSERT_EQUALS(x2.GetValue(), -3.14159e-5);
  }

  void testScientificNotationExtremes() {
    FGRealValue x1(1e308);  // Near max
    FGRealValue x2(1e-308); // Near min positive

    TS_ASSERT_EQUALS(x1.GetValue(), 1e308);
    TS_ASSERT_EQUALS(x2.GetValue(), 1e-308);
  }
};
