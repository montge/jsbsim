#include <cxxtest/TestSuite.h>
#include <math/FGRealValue.h>
#include <limits>
#include <cmath>

using namespace JSBSim;


class FGRealValueTest : public CxxTest::TestSuite
{
public:
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
};
